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
// 1. MQTT → gRPC：接收 Agent 的注册/心跳/TaskGroup状态，通过 gRPC 上报给 Manager
// 2. gRPC → MQTT：接收 Manager 推送的 TaskGroup，转发到 MQTT
type GRPCStreamServer struct {
	mqttBroker      string
	grpcAddr        string
	mqttClient      mqtt.Client
	grpcConn        *grpc.ClientConn
	grpcClient      pb.RobotManagerClient
	taskGroupStream pb.RobotManager_StreamTaskGroupsClient

	// TaskGroup 分发跟踪
	dispatchedTaskGroups map[string]bool // taskGroupUID-robotName -> dispatched
	taskGroupsMu         sync.RWMutex

	ctx    context.Context
	cancel context.CancelFunc
}

// NewGRPCStreamServer 创建新的 gRPC Stream Server
func NewGRPCStreamServer(mqttBroker, grpcAddr string) *GRPCStreamServer {
	ctx, cancel := context.WithCancel(context.Background())
	return &GRPCStreamServer{
		mqttBroker:           mqttBroker,
		grpcAddr:             grpcAddr,
		dispatchedTaskGroups: make(map[string]bool),
		ctx:                  ctx,
		cancel:               cancel,
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

	// 2. 建立 StreamTaskGroups 连接
	if err := s.initTaskGroupStream(); err != nil {
		return fmt.Errorf("failed to init taskgroup stream: %w", err)
	}

	// 3. 连接 MQTT
	if err := s.connectMQTT(); err != nil {
		return fmt.Errorf("failed to connect MQTT: %w", err)
	}

	// 4. 启动 Stream 接收循环（接收 Manager 推送的 TaskGroup）
	go s.receiveTaskGroupsFromStream()

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

// initTaskGroupStream 初始化 StreamTaskGroups 双向流
func (s *GRPCStreamServer) initTaskGroupStream() error {
	logger := log.FromContext(s.ctx)
	logger.Info("Initializing StreamTaskGroups bidirectional stream")

	stream, err := s.grpcClient.StreamTaskGroups(s.ctx)
	if err != nil {
		return fmt.Errorf("failed to create stream: %w", err)
	}

	s.taskGroupStream = stream
	logger.Info(" StreamTaskGroups initialized")
	return nil
}

// receiveTaskGroupsFromStream 持续接收 Manager 推送的 TaskGroup
// ========== gRPC Stream 接收逻辑 ==========
// Manager 通过 stream.Send(TaskGroupCommand) 推送 TaskGroup
// Server 通过 stream.Recv() 接收 TaskGroup
func (s *GRPCStreamServer) receiveTaskGroupsFromStream() {
	logger := log.FromContext(s.ctx)
	logger.Info(" Started receiving TaskGroups from Manager stream")

	const (
		topicRegister                     = "k8s4r/register"
		topicHeartbeat                    = "k8s4r/heartbeat"
		topicRobotTaskGroupDispatch       = "robot/%s/taskgroup"
		topicRobotTaskGroupStatusWildcard = "robot/+/taskgroup/status"
	)

	for {
		select {
		case <-s.ctx.Done():
			logger.Info("Stream receiver stopped")
			return
		default:
		}

		// 阻塞接收 Manager 发来的 TaskGroupCommand
		taskGroupCmd, err := s.taskGroupStream.Recv()
		if err != nil {
			logger.Error(err, "Failed to receive taskgroup from stream, reconnecting...")
			time.Sleep(5 * time.Second)
			// TODO: 实现重连逻辑
			continue
		}

		// 处理不同类型的命令
		switch taskGroupCmd.Type {
		case pb.TaskGroupCommand_CREATE_TASKGROUP:
			if taskGroupCmd.TaskgroupJson != "" {
				logger.Info(" [GRPC STREAM] Received CREATE_TASKGROUP from Manager",
					"taskGroupUID", taskGroupCmd.TaskgroupUid,
					"robotName", taskGroupCmd.RobotName)
				s.handleCreateTaskGroup(taskGroupCmd, topicRobotTaskGroupDispatch)
			} else {
				logger.Error(nil, "Received CREATE_TASKGROUP but taskgroup_json is empty")
			}
		case pb.TaskGroupCommand_DELETE_TASKGROUP:
			if taskGroupCmd.TaskgroupUid != "" {
				logger.Info(" [GRPC STREAM] Received DELETE_TASKGROUP from Manager",
					"taskGroupUID", taskGroupCmd.TaskgroupUid,
					"robotName", taskGroupCmd.RobotName)
				s.handleDeleteTaskGroup(taskGroupCmd)
			} else {
				logger.Error(nil, "Received DELETE_TASKGROUP but taskgroup_uid is empty")
			}
		case pb.TaskGroupCommand_KEEPALIVE:
			logger.V(1).Info("💓 Received KEEPALIVE from Manager")
			// 发送 KEEPALIVE 响应
			s.sendTaskGroupEvent(&pb.TaskGroupEvent{
				Type: pb.TaskGroupEvent_KEEPALIVE,
			})
		default:
			logger.Info("Unknown TaskGroupCommand type", "type", taskGroupCmd.Type)
		}
	}
}

// handleCreateTaskGroup 处理创建 TaskGroup 命令
// 流程：接收 gRPC TaskGroupCommand → 转发到 MQTT → 发送 ACK → 等待发布成功 → 发送 PUBLISHED
func (s *GRPCStreamServer) handleCreateTaskGroup(cmd *pb.TaskGroupCommand, topicTemplate string) {
	logger := log.FromContext(s.ctx)

	dispatchKey := cmd.TaskgroupUid + "-" + cmd.RobotName

	// 检查是否已分发
	s.taskGroupsMu.Lock()
	if s.dispatchedTaskGroups[dispatchKey] {
		logger.Info("TaskGroup already dispatched, skipping",
			"taskGroupUID", cmd.TaskgroupUid,
			"robot", cmd.RobotName)
		s.taskGroupsMu.Unlock()
		return
	}
	s.dispatchedTaskGroups[dispatchKey] = true
	s.taskGroupsMu.Unlock()

	// 发送 ACK
	s.sendTaskGroupEvent(&pb.TaskGroupEvent{
		Type:         pb.TaskGroupEvent_ACK,
		TaskgroupUid: cmd.TaskgroupUid,
		RobotName:    cmd.RobotName,
	})

	// 转发到 MQTT
	topic := fmt.Sprintf(topicTemplate, cmd.RobotName)

	// 构造 TaskGroup 消息（使用完整的 JSON）
	taskGroupMsg := map[string]interface{}{
		"action":    "create",
		"taskGroup": json.RawMessage(cmd.TaskgroupJson),
	}

	payload, err := json.Marshal(taskGroupMsg)
	if err != nil {
		logger.Error(err, "Failed to marshal taskgroup message")
		s.sendTaskGroupEvent(&pb.TaskGroupEvent{
			Type:         pb.TaskGroupEvent_ERROR,
			TaskgroupUid: cmd.TaskgroupUid,
			RobotName:    cmd.RobotName,
			Message:      fmt.Sprintf("Failed to marshal taskgroup: %v", err),
		})
		return
	}

	// 发布到 MQTT
	token := s.mqttClient.Publish(topic, 1, false, payload)
	if token.Wait() && token.Error() != nil {
		logger.Error(token.Error(), "Failed to publish taskgroup to MQTT")
		s.sendTaskGroupEvent(&pb.TaskGroupEvent{
			Type:         pb.TaskGroupEvent_ERROR,
			TaskgroupUid: cmd.TaskgroupUid,
			RobotName:    cmd.RobotName,
			Message:      fmt.Sprintf("MQTT publish failed: %v", token.Error()),
		})
		return
	}

	logger.Info(" [MQTT] TaskGroup dispatched successfully",
		"taskGroupUID", cmd.TaskgroupUid,
		"robot", cmd.RobotName,
		"topic", topic)

	// 发送 PUBLISHED 事件
	s.sendTaskGroupEvent(&pb.TaskGroupEvent{
		Type:         pb.TaskGroupEvent_PUBLISHED,
		TaskgroupUid: cmd.TaskgroupUid,
		RobotName:    cmd.RobotName,
	})
}

// handleDeleteTaskGroup 处理删除 TaskGroup 命令
func (s *GRPCStreamServer) handleDeleteTaskGroup(cmd *pb.TaskGroupCommand) {
	logger := log.FromContext(s.ctx)
	logger.Info("Handling DELETE_TASKGROUP",
		"taskGroupUID", cmd.TaskgroupUid,
		"robot", cmd.RobotName)

	// 清除已分发标记
	dispatchKey := cmd.TaskgroupUid + "-" + cmd.RobotName
	s.taskGroupsMu.Lock()
	delete(s.dispatchedTaskGroups, dispatchKey)
	s.taskGroupsMu.Unlock()

	// TODO: 通知 Agent 取消 TaskGroup（如果支持）
	s.sendTaskGroupEvent(&pb.TaskGroupEvent{
		Type:         pb.TaskGroupEvent_ACK,
		TaskgroupUid: cmd.TaskgroupUid,
		RobotName:    cmd.RobotName,
	})
}

// sendTaskGroupEvent 发送 TaskGroupEvent 到 Manager
func (s *GRPCStreamServer) sendTaskGroupEvent(event *pb.TaskGroupEvent) {
	logger := log.FromContext(s.ctx)

	if err := s.taskGroupStream.Send(event); err != nil {
		logger.Error(err, "Failed to send TaskGroupEvent to Manager", "type", event.Type)
		return
	}

	logger.V(1).Info(" [GRPC STREAM] Sent TaskGroupEvent to Manager",
		"type", event.Type,
		"taskGroupUID", event.TaskgroupUid,
		"robot", event.RobotName)
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
		topicRegister                     = "k8s4r/register"
		topicHeartbeat                    = "k8s4r/heartbeat"
		topicRobotTaskGroupStatusWildcard = "robot/+/taskgroup/status"
	)

	// 订阅注册消息
	if token := s.mqttClient.Subscribe(topicRegister, 1, s.handleRegister); token.Wait() && token.Error() != nil {
		logger.Error(token.Error(), "Failed to subscribe to register topic")
	}

	// 订阅心跳消息
	if token := s.mqttClient.Subscribe(topicHeartbeat, 1, s.handleHeartbeat); token.Wait() && token.Error() != nil {
		logger.Error(token.Error(), "Failed to subscribe to heartbeat topic")
	}

	// 订阅 TaskGroup 状态消息（通配符）
	if token := s.mqttClient.Subscribe(topicRobotTaskGroupStatusWildcard, 1, s.handleTaskGroupStatus); token.Wait() && token.Error() != nil {
		logger.Error(token.Error(), "Failed to subscribe to taskgroup status topic")
	}

	logger.Info(" Subscribed to MQTT topics",
		"register", topicRegister,
		"heartbeat", topicHeartbeat,
		"taskGroupStatus", topicRobotTaskGroupStatusWildcard)
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

	logger.Info(" [GRPC] Heartbeat reported to Manager", "success", resp.Success)
}

// handleTaskGroupStatus 处理 Agent TaskGroup 状态上报（新版）
// MQTT → 直接记录（不需要通过 gRPC，因为 Manager 会通过 K8s API 更新）
func (s *GRPCStreamServer) handleTaskGroupStatus(client mqtt.Client, msg mqtt.Message) {
	logger := log.FromContext(s.ctx)

	logger.Info(" [MQTT] Received TaskGroup status message",
		"topic", msg.Topic(),
		"payload", string(msg.Payload()))

	var status struct {
		TaskGroupUID string            `json:"taskGroupUid"`
		RobotName    string            `json:"robotName"`
		State        string            `json:"state"`
		Message      string            `json:"message"`
		TaskStates   map[string]string `json:"taskStates"`
		UpdatedAt    string            `json:"updatedAt"`
	}

	if err := json.Unmarshal(msg.Payload(), &status); err != nil {
		logger.Error(err, "Failed to unmarshal taskgroup status")
		return
	}

	logger.Info(" [MQTT] Parsed TaskGroup status",
		"taskGroupUID", status.TaskGroupUID,
		"robot", status.RobotName,
		"state", status.State,
		"taskStates", status.TaskStates)

	// TODO: 如果需要，可以通过 gRPC 上报给 Manager
	// 目前 Manager 通过 K8s API 监听状态变化，Server 只负责转发
}

// handleTaskStatus 处理 Agent 任务状态上报（旧版，已废弃）
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
