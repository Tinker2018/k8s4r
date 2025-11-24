/*
Copyright 2025.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0
*/

package server

import (
	"context"
	"encoding/json"
	"fmt"
	"sync"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	"google.golang.org/grpc"
	"google.golang.org/grpc/credentials/insecure"

	pb "github.com/hxndghxndg/k8s4r/api/grpc"
	"sigs.k8s.io/controller-runtime/pkg/log"
)

// MQTT Topics (复用 server.go 中的定义)
// 这里重新定义是为了避免导入依赖，因为 server.go 有 K8s 依赖

// GRPCStreamServer 基于 gRPC Stream 的 Server
// 职责：
// 1. MQTT → gRPC：接收 Agent 的注册/心跳/任务状态，通过 gRPC 上报给 Manager
// 2. gRPC → MQTT：接收 Manager 推送的任务，转发到 MQTT
type GRPCStreamServer struct {
	mqttBroker string
	grpcAddr   string
	mqttClient mqtt.Client
	grpcConn   *grpc.ClientConn
	grpcClient pb.RobotManagerClient
	taskStream pb.RobotManager_StreamTasksClient

	// 任务分发跟踪
	dispatchedTasks map[string]bool // taskUID -> dispatched
	tasksMu         sync.RWMutex

	ctx    context.Context
	cancel context.CancelFunc
}

// NewGRPCStreamServer 创建新的 gRPC Stream Server
func NewGRPCStreamServer(mqttBroker, grpcAddr string) *GRPCStreamServer {
	ctx, cancel := context.WithCancel(context.Background())
	return &GRPCStreamServer{
		mqttBroker:      mqttBroker,
		grpcAddr:        grpcAddr,
		dispatchedTasks: make(map[string]bool),
		ctx:             ctx,
		cancel:          cancel,
	}
}

// Start 启动 Server
func (s *GRPCStreamServer) Start() error {
	logger := log.FromContext(s.ctx)
	logger.Info(" Starting GRPCStreamServer (gRPC + MQTT, NO K8s)")

	// 1. 连接 gRPC Manager
	if err := s.connectGRPC(); err != nil {
		return fmt.Errorf("failed to connect gRPC: %w", err)
	}

	// 2. 建立 StreamTasks 连接
	if err := s.initTaskStream(); err != nil {
		return fmt.Errorf("failed to init task stream: %w", err)
	}

	// 3. 连接 MQTT
	if err := s.connectMQTT(); err != nil {
		return fmt.Errorf("failed to connect MQTT: %w", err)
	}

	// 4. 启动 Stream 接收循环（接收 Manager 推送的任务）
	go s.receiveTasksFromStream()

	logger.Info(" GRPCStreamServer started successfully")
	return nil
}

// Stop 停止 Server
func (s *GRPCStreamServer) Stop() {
	logger := log.FromContext(s.ctx)
	logger.Info("Stopping GRPCStreamServer")

	s.cancel()

	if s.mqttClient != nil && s.mqttClient.IsConnected() {
		s.mqttClient.Disconnect(250)
	}

	if s.grpcConn != nil {
		s.grpcConn.Close()
	}
}

// connectGRPC 连接 Manager gRPC Server
func (s *GRPCStreamServer) connectGRPC() error {
	logger := log.FromContext(s.ctx)
	logger.Info("Connecting to Manager gRPC server", "address", s.grpcAddr)

	conn, err := grpc.NewClient(
		s.grpcAddr,
		grpc.WithTransportCredentials(insecure.NewCredentials()),
	)
	if err != nil {
		return fmt.Errorf("failed to create grpc client: %w", err)
	}

	s.grpcConn = conn
	s.grpcClient = pb.NewRobotManagerClient(conn)

	logger.Info(" Connected to Manager gRPC server")
	return nil
}

// initTaskStream 初始化 StreamTasks 双向流
func (s *GRPCStreamServer) initTaskStream() error {
	logger := log.FromContext(s.ctx)
	logger.Info("Initializing StreamTasks bidirectional stream")

	stream, err := s.grpcClient.StreamTasks(s.ctx)
	if err != nil {
		return fmt.Errorf("failed to create stream: %w", err)
	}

	s.taskStream = stream
	logger.Info(" StreamTasks initialized")
	return nil
}

// receiveTasksFromStream 持续接收 Manager 推送的任务
// ========== gRPC Stream 接收逻辑 ==========
// Manager 通过 stream.Send(TaskCommand) 推送任务
// Server 通过 stream.Recv() 接收任务
func (s *GRPCStreamServer) receiveTasksFromStream() {
	logger := log.FromContext(s.ctx)
	logger.Info(" Started receiving tasks from Manager stream")

	const (
		topicRegister                = "k8s4r/register"
		topicHeartbeat               = "k8s4r/heartbeat"
		topicRobotTaskDispatch       = "k8s4r/robots/%s/tasks/dispatch"
		topicRobotTaskStatusWildcard = "k8s4r/robots/+/tasks/+/status"
	)

	for {
		select {
		case <-s.ctx.Done():
			logger.Info("Stream receiver stopped")
			return
		default:
		}

		// 阻塞接收 Manager 发来的 TaskCommand
		taskCmd, err := s.taskStream.Recv()
		if err != nil {
			logger.Error(err, "Failed to receive task from stream, reconnecting...")
			time.Sleep(5 * time.Second)
			// TODO: 实现重连逻辑
			continue
		}

		// 处理不同类型的命令
		switch taskCmd.Type {
		case pb.TaskCommand_CREATE_TASK:
			if taskCmd.Task != nil {
				logger.Info(" [GRPC STREAM] Received CREATE_TASK from Manager",
					"taskUID", taskCmd.Task.Uid,
					"taskName", taskCmd.Task.Name)
				s.handleCreateTask(taskCmd.Task, topicRobotTaskDispatch)
			} else {
				logger.Error(nil, "Received CREATE_TASK but task is nil")
			}
		case pb.TaskCommand_DELETE_TASK:
			if taskCmd.Task != nil {
				logger.Info(" [GRPC STREAM] Received DELETE_TASK from Manager",
					"taskUID", taskCmd.Task.Uid)
				s.handleDeleteTask(taskCmd.Task)
			} else {
				logger.Error(nil, "Received DELETE_TASK but task is nil")
			}
		case pb.TaskCommand_KEEPALIVE:
			logger.V(1).Info("💓 Received KEEPALIVE from Manager")
			// 发送 KEEPALIVE 响应
			s.sendTaskEvent(&pb.TaskEvent{
				Type:    pb.TaskEvent_KEEPALIVE,
				TaskUid: "",
			})
		default:
			logger.Info("Unknown TaskCommand type", "type", taskCmd.Type)
		}
	}
}

// handleCreateTask 处理创建任务命令
// 流程：接收 gRPC TaskCommand → 转发到 MQTT → 发送 ACK → 等待发布成功 → 发送 PUBLISHED
func (s *GRPCStreamServer) handleCreateTask(task *pb.Task, topicTemplate string) {
	logger := log.FromContext(s.ctx)

	// 检查是否已分发
	s.tasksMu.Lock()
	if s.dispatchedTasks[task.Uid] {
		logger.Info("Task already dispatched, skipping", "taskUID", task.Uid)
		s.tasksMu.Unlock()
		return
	}
	s.dispatchedTasks[task.Uid] = true
	s.tasksMu.Unlock()

	// 发送 ACK
	s.sendTaskEvent(&pb.TaskEvent{
		Type:    pb.TaskEvent_ACK,
		TaskUid: task.Uid,
	})

	// 转发到 MQTT
	topic := fmt.Sprintf(topicTemplate, task.TargetRobot)

	// 构造任务消息（简化版，包含必要信息）
	taskMsg := map[string]interface{}{
		"taskUid":     task.Uid,
		"taskName":    task.Name,
		"driver":      task.Driver,
		"config":      task.Config,
		"timeout":     task.Timeout,
		"killTimeout": task.KillTimeout,
		"env":         task.Env,
	}

	payload, err := json.Marshal(taskMsg)
	if err != nil {
		logger.Error(err, "Failed to marshal task message")
		s.sendTaskEvent(&pb.TaskEvent{
			Type:    pb.TaskEvent_ERROR,
			TaskUid: task.Uid,
			Message: fmt.Sprintf("Failed to marshal task: %v", err),
		})
		return
	}

	// 发布到 MQTT
	token := s.mqttClient.Publish(topic, 1, false, payload)
	if token.Wait() && token.Error() != nil {
		logger.Error(token.Error(), "Failed to publish task to MQTT")
		s.sendTaskEvent(&pb.TaskEvent{
			Type:    pb.TaskEvent_ERROR,
			TaskUid: task.Uid,
			Message: fmt.Sprintf("MQTT publish failed: %v", token.Error()),
		})
		return
	}

	logger.Info(" [MQTT] Task dispatched successfully",
		"taskUID", task.Uid,
		"robot", task.TargetRobot,
		"topic", topic)

	// 发送 PUBLISHED 事件
	s.sendTaskEvent(&pb.TaskEvent{
		Type:    pb.TaskEvent_PUBLISHED,
		TaskUid: task.Uid,
	})
}

// handleDeleteTask 处理删除任务命令
func (s *GRPCStreamServer) handleDeleteTask(task *pb.Task) {
	logger := log.FromContext(s.ctx)
	logger.Info("Handling DELETE_TASK", "taskUID", task.Uid)

	// 清除已分发标记
	s.tasksMu.Lock()
	delete(s.dispatchedTasks, task.Uid)
	s.tasksMu.Unlock()

	// TODO: 通知 Agent 取消任务（如果支持）
	s.sendTaskEvent(&pb.TaskEvent{
		Type:    pb.TaskEvent_ACK,
		TaskUid: task.Uid,
	})
}

// sendTaskEvent 发送 TaskEvent 到 Manager
func (s *GRPCStreamServer) sendTaskEvent(event *pb.TaskEvent) {
	logger := log.FromContext(s.ctx)

	if err := s.taskStream.Send(event); err != nil {
		logger.Error(err, "Failed to send TaskEvent to Manager", "type", event.Type)
		return
	}

	logger.V(1).Info(" [GRPC STREAM] Sent TaskEvent to Manager",
		"type", event.Type,
		"taskUID", event.TaskUid)
}

// connectMQTT 连接 MQTT Broker
func (s *GRPCStreamServer) connectMQTT() error {
	logger := log.FromContext(s.ctx)
	logger.Info("Connecting to MQTT broker", "broker", s.mqttBroker)

	opts := mqtt.NewClientOptions()
	opts.AddBroker(s.mqttBroker)
	opts.SetClientID("k8s4r-server-grpc")
	opts.SetAutoReconnect(true)
	opts.SetKeepAlive(30 * time.Second)
	opts.SetPingTimeout(10 * time.Second)

	opts.OnConnect = func(c mqtt.Client) {
		logger.Info(" Connected to MQTT broker")
		s.subscribeTopics()
	}

	opts.OnConnectionLost = func(c mqtt.Client, err error) {
		logger.Error(err, " Lost connection to MQTT broker")
	}

	s.mqttClient = mqtt.NewClient(opts)
	token := s.mqttClient.Connect()
	if token.Wait() && token.Error() != nil {
		return fmt.Errorf("failed to connect to MQTT: %w", token.Error())
	}

	return nil
}

// subscribeTopics 订阅 MQTT Topics
func (s *GRPCStreamServer) subscribeTopics() {
	logger := log.FromContext(s.ctx)

	const (
		topicRegister                = "k8s4r/register"
		topicHeartbeat               = "k8s4r/heartbeat"
		topicRobotTaskStatusWildcard = "k8s4r/robots/+/tasks/+/status"
	)

	// 订阅注册消息
	if token := s.mqttClient.Subscribe(topicRegister, 1, s.handleRegister); token.Wait() && token.Error() != nil {
		logger.Error(token.Error(), "Failed to subscribe to register topic")
	}

	// 订阅心跳消息
	if token := s.mqttClient.Subscribe(topicHeartbeat, 1, s.handleHeartbeat); token.Wait() && token.Error() != nil {
		logger.Error(token.Error(), "Failed to subscribe to heartbeat topic")
	}

	// 订阅任务状态消息（通配符）
	if token := s.mqttClient.Subscribe(topicRobotTaskStatusWildcard, 1, s.handleTaskStatus); token.Wait() && token.Error() != nil {
		logger.Error(token.Error(), "Failed to subscribe to task status topic")
	}

	logger.Info(" Subscribed to MQTT topics")
}

// handleRegister 处理 Agent 注册消息
// MQTT → gRPC Unary RPC
func (s *GRPCStreamServer) handleRegister(client mqtt.Client, msg mqtt.Message) {
	logger := log.FromContext(s.ctx)

	// 打印完整的 MQTT 消息
	logger.Info(" [MQTT] Received registration message",
		"topic", msg.Topic(),
		"payload", string(msg.Payload()))

	var req struct {
		RobotID    string          `json:"robotId"`
		Token      string          `json:"token"`
		DeviceInfo json.RawMessage `json:"deviceInfo"`
	}

	if err := json.Unmarshal(msg.Payload(), &req); err != nil {
		logger.Error(err, "Failed to unmarshal register request")
		return
	}

	logger.Info(" [MQTT] Parsed registration", "robotId", req.RobotID)

	// 转换 DeviceInfo（简化版本）
	pbDeviceInfo, err := convertDeviceInfoFromJSON(req.DeviceInfo)
	if err != nil {
		logger.Error(err, "Failed to convert device info", "robotId", req.RobotID)
		// 即使 DeviceInfo 转换失败，也继续注册（使用 nil）
		pbDeviceInfo = nil
	} else if pbDeviceInfo != nil {
		logger.Info(" [DEVICE] Converted device info",
			"robotId", req.RobotID,
			"hostname", pbDeviceInfo.Hostname,
			"os", pbDeviceInfo.Os,
			"arch", pbDeviceInfo.Arch,
			"cpus", pbDeviceInfo.NumCpus,
			"memory", pbDeviceInfo.TotalMemory)
	}

	// 通过 gRPC Unary RPC 上报给 Manager
	ctx, cancel := context.WithTimeout(s.ctx, 5*time.Second)
	defer cancel()

	resp, err := s.grpcClient.ReportRobotRegistration(ctx, &pb.RegistrationRequest{
		RobotId:        req.RobotID,
		Token:          req.Token,
		DeviceInfo:     pbDeviceInfo,
		DeviceInfoJson: string(req.DeviceInfo), // 完整的JSON
	})

	if err != nil {
		logger.Error(err, "Failed to report registration to Manager")
		// 发送失败响应给 Agent
		s.publishResponse(req.RobotID, false, fmt.Sprintf("Registration failed: %v", err))
		return
	}

	logger.Info(" [GRPC] Registration reported to Manager", "success", resp.Success)

	// 发送成功响应给 Agent
	s.publishResponse(req.RobotID, resp.Success, resp.Message)
}

// handleHeartbeat 处理 Agent 心跳消息
// MQTT → gRPC Unary RPC
func (s *GRPCStreamServer) handleHeartbeat(client mqtt.Client, msg mqtt.Message) {
	logger := log.FromContext(s.ctx)

	// 打印完整的 MQTT 消息
	// logger.V(1).Info("💓 [MQTT] Received heartbeat message",
	// 	"topic", msg.Topic(),
	// 	"payload", string(msg.Payload()))

	var req struct {
		RobotID    string          `json:"robotId"`
		Token      string          `json:"token"`
		DeviceInfo json.RawMessage `json:"deviceInfo"`
	}

	if err := json.Unmarshal(msg.Payload(), &req); err != nil {
		logger.Error(err, "Failed to unmarshal heartbeat request")
		return
	}

	logger.V(1).Info("💓 [MQTT] Parsed heartbeat", "robotId", req.RobotID)

	// 转换 DeviceInfo（简化版本）
	pbDeviceInfo, err := convertDeviceInfoFromJSON(req.DeviceInfo)
	if err != nil {
		logger.Error(err, "Failed to convert device info", "robotId", req.RobotID)
		// 即使 DeviceInfo 转换失败，也继续心跳（使用 nil）
		pbDeviceInfo = nil
	}

	// 通过 gRPC Unary RPC 上报给 Manager
	ctx, cancel := context.WithTimeout(s.ctx, 5*time.Second)
	defer cancel()

	resp, err := s.grpcClient.ReportRobotHeartbeat(ctx, &pb.HeartbeatRequest{
		RobotId:        req.RobotID,
		Token:          req.Token,
		DeviceInfo:     pbDeviceInfo,
		DeviceInfoJson: string(req.DeviceInfo), // 完整的JSON
	})

	if err != nil {
		logger.Error(err, "Failed to report heartbeat to Manager")
		return
	}

	logger.V(1).Info(" [GRPC] Heartbeat reported to Manager", "success", resp.Success)
}

// handleTaskStatus 处理 Agent 任务状态上报
// MQTT → gRPC Unary RPC
func (s *GRPCStreamServer) handleTaskStatus(client mqtt.Client, msg mqtt.Message) {
	logger := log.FromContext(s.ctx)

	// 打印完整的 MQTT 消息
	logger.Info(" [MQTT] Received task status message",
		"topic", msg.Topic(),
		"payload", string(msg.Payload()))

	var status struct {
		TaskUID   string `json:"taskUid"`
		RobotName string `json:"robotName"`
		State     string `json:"state"`
		ExitCode  int32  `json:"exitCode"`
		Message   string `json:"message"`
		Event     string `json:"event"`
	}

	if err := json.Unmarshal(msg.Payload(), &status); err != nil {
		logger.Error(err, "Failed to unmarshal task status")
		return
	}

	logger.Info(" [MQTT] Parsed task status",
		"taskUID", status.TaskUID,
		"state", status.State,
		"event", status.Event)

	// 通过 gRPC Unary RPC 上报给 Manager
	ctx, cancel := context.WithTimeout(s.ctx, 5*time.Second)
	defer cancel()

	resp, err := s.grpcClient.ReportTaskStatus(ctx, &pb.TaskStatusRequest{
		TaskUid:   status.TaskUID,
		RobotName: status.RobotName,
		State:     status.State,
		ExitCode:  status.ExitCode,
		Message:   status.Message,
		Event:     status.Event,
	})

	if err != nil {
		logger.Error(err, "Failed to report task status to Manager")
		return
	}

	logger.Info(" [GRPC] Task status reported to Manager", "success", resp.Success)
}

// publishResponse 发送响应消息到 Agent
func (s *GRPCStreamServer) publishResponse(robotID string, success bool, message string) {
	logger := log.FromContext(s.ctx)

	topic := fmt.Sprintf("k8s4r/robots/%s/response", robotID)

	response := map[string]interface{}{
		"success": success,
		"message": message,
		"robotId": robotID,
	}

	payload, err := json.Marshal(response)
	if err != nil {
		logger.Error(err, "Failed to marshal response")
		return
	}

	token := s.mqttClient.Publish(topic, 1, false, payload)
	if token.Wait() && token.Error() != nil {
		logger.Error(token.Error(), "Failed to publish response", "topic", topic)
		return
	}

	logger.V(1).Info(" [MQTT] Published response to Agent",
		"robotId", robotID,
		"topic", topic,
		"success", success)
}

// convertDeviceInfoFromJSON 将 JSON DeviceInfo 转换为 protobuf DeviceInfo
func convertDeviceInfoFromJSON(deviceInfoJSON json.RawMessage) (*pb.DeviceInfo, error) {
	if len(deviceInfoJSON) == 0 {
		return nil, nil
	}

	// 解析为 K8s DeviceInfo 结构（包含完整信息）
	var k8sDeviceInfo struct {
		Hostname string `json:"hostname"`
		Platform struct {
			OS            string `json:"os"`
			Arch          string `json:"arch"`
			KernelVersion string `json:"kernelVersion"`
		} `json:"platform"`
		CPU struct {
			LogicalCores int `json:"logicalCores"`
		} `json:"cpu"`
		Memory struct {
			Total uint64 `json:"total"`
		} `json:"memory"`
	}

	if err := json.Unmarshal(deviceInfoJSON, &k8sDeviceInfo); err != nil {
		return nil, fmt.Errorf("failed to unmarshal device info: %w", err)
	}

	// 转换为 protobuf DeviceInfo
	pbDeviceInfo := &pb.DeviceInfo{
		Hostname:      k8sDeviceInfo.Hostname,
		Os:            k8sDeviceInfo.Platform.OS,
		Arch:          k8sDeviceInfo.Platform.Arch,
		NumCpus:       int32(k8sDeviceInfo.CPU.LogicalCores),
		TotalMemory:   int64(k8sDeviceInfo.Memory.Total),
		KernelVersion: k8sDeviceInfo.Platform.KernelVersion,
		Labels:        make(map[string]string),
	}

	return pbDeviceInfo, nil
}
