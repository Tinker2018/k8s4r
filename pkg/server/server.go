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
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	corev1 "k8s.io/api/core/v1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/log"

	robotv1alpha1 "github.com/hxndghxndg/k8s4r/api/v1alpha1"
)

const (
	// MQTT Topics 设计原则：
	// 1. 所有 topic 以 k8s4r/ 开头
	// 2. 每个机器人有独立的命名空间 k8s4r/robots/{robotId}/
	// 3. 使用点对点通信，避免全局广播

	// 全局 Topics（Agent → Server）
	TopicRegister  = "k8s4r/register"  // Agent 注册请求
	TopicHeartbeat = "k8s4r/heartbeat" // Agent 心跳上报

	// 机器人专属 Topics（双向通信）
	// Server → Agent（使用具体的 robotId）
	TopicRobotResponse     = "k8s4r/robots/%s/response"       // Server 发送：k8s4r/robots/{robotId}/response
	TopicRobotTaskDispatch = "k8s4r/robots/%s/tasks/dispatch" // Server 发送：k8s4r/robots/{robotId}/tasks/dispatch
	TopicRobotTaskState    = "k8s4r/robots/%s/tasks/state"    // Server 发送：k8s4r/robots/{robotId}/tasks/state（retained消息，用于Agent重启恢复）

	// Agent → Server（Server 使用通配符订阅，Agent 发布到具体 topic）
	TopicRobotTaskStatus = "k8s4r/robots/+/tasks/+/status" // Server 订阅通配符，Agent 发送：k8s4r/robots/{robotId}/tasks/{taskUID}/status
)

// Server MQTT 服务器，负责四个核心职责：
// 0. 处理机器人的注册事件（MQTT → Kubernetes）
// 1. 监听 Robot 的 heartbeat 事件，更新 Robot 心跳时间（MQTT → Kubernetes）
// 2. 发布 Task 到 Robot 监听的 MQTT topic（Kubernetes → MQTT）
// 3. 监听 Task 的处理状态，更新 Event 和 Task 状态到 Kubernetes（MQTT → Kubernetes）
type Server struct {
	Client     client.Client
	Namespace  string
	mqttClient mqtt.Client
	ctx        context.Context
	cancel     context.CancelFunc
}

// RegisterRequest Agent 注册请求
type RegisterRequest struct {
	RobotID    string                    `json:"robotId"`
	Token      string                    `json:"token"`
	DeviceInfo *robotv1alpha1.DeviceInfo `json:"deviceInfo,omitempty"`
}

// HeartbeatRequest Agent 心跳请求
type HeartbeatRequest struct {
	RobotID    string                    `json:"robotId"`
	Token      string                    `json:"token"`
	DeviceInfo *robotv1alpha1.DeviceInfo `json:"deviceInfo,omitempty"`
}

// Response 通用响应
type Response struct {
	Success bool   `json:"success"`
	Message string `json:"message"`
	RobotID string `json:"robotId,omitempty"`
}

// TaskStatusMessage Agent 上报的任务状态
type TaskStatusMessage struct {
	TaskUID   string    `json:"taskUid"`
	RobotName string    `json:"robotName"`
	State     string    `json:"state"` // pending, running, completed, failed, dead
	ExitCode  int       `json:"exitCode"`
	Message   string    `json:"message"`
	Event     string    `json:"event"` // 事件类型：Assigned, Downloading, DownloadFailed, Starting, Started, Completed, Failed
	UpdatedAt time.Time `json:"updatedAt"`
}

// NewServer 创建 Server 实例
func NewServer(client client.Client, namespace string) *Server {
	ctx, cancel := context.WithCancel(context.Background())
	return &Server{
		Client:    client,
		Namespace: namespace,
		ctx:       ctx,
		cancel:    cancel,
	}
}

// ===== MQTT → K8s 消息转发 =====

// RegisterHandler 处理 Agent 注册（MQTT → K8s）
// ========== 设计原则 ==========
// Server 只负责接收 MQTT 消息并通知 Manager（通过更新 annotation）
// RobotController 负责所有状态管理（创建 Robot、更新 Phase、更新心跳时间）
func (s *Server) RegisterHandler(client mqtt.Client, msg mqtt.Message) {
	logger := log.FromContext(s.ctx)

	var req RegisterRequest
	if err := json.Unmarshal(msg.Payload(), &req); err != nil {
		logger.Error(err, "Failed to decode register request")
		return
	}

	logger.Info(" [SERVER] Received register request from MQTT", "robotId", req.RobotID)

	// 查找或创建 Robot 资源
	robot := &robotv1alpha1.Robot{}
	err := s.Client.Get(s.ctx, types.NamespacedName{
		Name:      req.RobotID,
		Namespace: s.Namespace,
	}, robot)

	if err != nil {
		// Robot 不存在，创建新的（状态留空，由 RobotController 初始化）
		robot = &robotv1alpha1.Robot{
			ObjectMeta: metav1.ObjectMeta{
				Name:      req.RobotID,
				Namespace: s.Namespace,
				Annotations: map[string]string{
					"k8s4r.io/registration-time": time.Now().Format(time.RFC3339),
				},
			},
			Spec: robotv1alpha1.RobotSpec{
				RobotID:     req.RobotID,
				Description: "Registered by agent",
			},
		}

		if err := s.Client.Create(s.ctx, robot); err != nil {
			logger.Error(err, " [SERVER] Failed to create Robot resource", "robotId", req.RobotID)
			s.sendResponse(req.RobotID, false, "Failed to register robot")
			return
		}

		logger.Info(" [SERVER] Created Robot resource, waiting for RobotController to initialize",
			"robotId", req.RobotID)
	}

	// ==========  关键改动：Server 只更新 annotation，不修改 status ==========
	// 通过更新 annotation 通知 RobotController 处理注册事件
	// RobotController 会：1. 更新 LastHeartbeatTime  2. 更新 DeviceInfo  3. 设置 Phase = Online
	if robot.Annotations == nil {
		robot.Annotations = make(map[string]string)
	}
	robot.Annotations["k8s4r.io/last-register"] = time.Now().Format(time.RFC3339)

	// 将 DeviceInfo 序列化到 annotation（Controller 会读取并更新到 status）
	if req.DeviceInfo != nil {
		deviceInfoJSON, _ := json.Marshal(req.DeviceInfo)
		robot.Annotations["k8s4r.io/device-info"] = string(deviceInfoJSON)
	}

	if err := s.Client.Update(s.ctx, robot); err != nil {
		logger.Error(err, " [SERVER] Failed to update Robot annotations", "robotId", req.RobotID)
		s.sendResponse(req.RobotID, false, "Failed to update registration")
		return
	}

	logger.Info("� [SERVER] Notified RobotController about registration",
		"robotId", req.RobotID,
		"annotation", "k8s4r.io/last-register")

	s.sendResponse(req.RobotID, true, "Registration received, processing by controller")
}

// HeartbeatHandler 处理心跳（MQTT → K8s）
// ========== 设计原则 ==========
// Server 只负责接收 MQTT 消息并通知 Manager（通过更新 annotation）
// RobotController 负责所有状态管理（更新 LastHeartbeatTime、更新 Phase）
func (s *Server) HeartbeatHandler(client mqtt.Client, msg mqtt.Message) {
	logger := log.FromContext(s.ctx)

	var req HeartbeatRequest
	if err := json.Unmarshal(msg.Payload(), &req); err != nil {
		logger.Error(err, "Failed to decode heartbeat request")
		return
	}

	// 获取 Robot 资源
	robot := &robotv1alpha1.Robot{}
	err := s.Client.Get(s.ctx, types.NamespacedName{
		Name:      req.RobotID,
		Namespace: s.Namespace,
	}, robot)

	if err != nil {
		logger.Error(err, "Robot not found", "robotId", req.RobotID)
		return
	}

	// ==========  关键改动：Server 只更新 annotation，不修改 status ==========
	// 通过更新 annotation 通知 RobotController 处理心跳事件
	// RobotController 会：1. 更新 LastHeartbeatTime  2. 更新 DeviceInfo  3. 检查并设置 Phase
	if robot.Annotations == nil {
		robot.Annotations = make(map[string]string)
	}
	robot.Annotations["k8s4r.io/last-heartbeat"] = time.Now().Format(time.RFC3339)

	// 将 DeviceInfo 序列化到 annotation（如果有的话）
	if req.DeviceInfo != nil {
		deviceInfoJSON, _ := json.Marshal(req.DeviceInfo)
		robot.Annotations["k8s4r.io/device-info"] = string(deviceInfoJSON)
	}

	if err := s.Client.Update(s.ctx, robot); err != nil {
		logger.Error(err, " [SERVER] Failed to update robot annotations", "robotId", req.RobotID)
		return
	}

	logger.V(1).Info("� [SERVER] Notified RobotController about heartbeat",
		"robotId", req.RobotID,
		"annotation", "k8s4r.io/last-heartbeat")
}

// TaskStatusHandler 处理任务状态上报（MQTT → K8s）
func (s *Server) TaskStatusHandler(client mqtt.Client, msg mqtt.Message) {
	logger := log.FromContext(s.ctx)

	var statusMsg TaskStatusMessage
	if err := json.Unmarshal(msg.Payload(), &statusMsg); err != nil {
		logger.Error(err, "Failed to decode task status message")
		return
	}

	logger.Info("Received task status update",
		"taskUid", statusMsg.TaskUID,
		"robotName", statusMsg.RobotName,
		"state", statusMsg.State,
		"event", statusMsg.Event,
		"exitCode", statusMsg.ExitCode,
		"message", statusMsg.Message)

	// 查找对应的 Task
	taskList := &robotv1alpha1.TaskList{}
	if err := s.Client.List(s.ctx, taskList); err != nil {
		logger.Error(err, "Failed to list tasks")
		return
	}

	var task *robotv1alpha1.Task
	for i := range taskList.Items {
		t := &taskList.Items[i]
		if string(t.UID) == statusMsg.TaskUID {
			task = t
			break
		}
	}

	if task == nil {
		logger.Error(nil, "Task not found", "taskUid", statusMsg.TaskUID)
		return
	}

	// 记录状态变更
	oldState := task.Status.State

	// 更新 Task 状态
	// 如果任务是 dispatching，收到任何状态上报都说明 Agent 已接收，转为对应状态
	newState := robotv1alpha1.TaskState(statusMsg.State)

	// 特殊处理：如果是 pending 状态上报，且当前是 dispatching，转为 running
	if newState == "pending" && task.Status.State == robotv1alpha1.TaskStateScheduled {
		newState = robotv1alpha1.TaskStateRunning
		logger.Info("Task state transition: dispatching -> running (agent acknowledged)")
	}

	task.Status.State = newState
	task.Status.Message = statusMsg.Message
	exitCode := int32(statusMsg.ExitCode)
	task.Status.ExitCode = &exitCode

	if err := s.Client.Status().Update(s.ctx, task); err != nil {
		logger.Error(err, "Failed to update task status", "task", task.Name)
		return
	}

	// 创建 Event 记录任务执行过程
	if statusMsg.Event != "" {
		s.createTaskEvent(task, statusMsg.Event, statusMsg.Message)
	}

	// 如果任务完成或失败，更新 Robot 的任务状态列表
	if newState == robotv1alpha1.TaskStateCompleted ||
		newState == robotv1alpha1.TaskStateFailed ||
		(newState == robotv1alpha1.TaskStateExited && exitCode == 0) {
		logger.Info("Task finished, updating robot state", "task", task.Name, "state", newState)
		if err := s.removeTaskFromRobotState(s.ctx, task); err != nil {
			logger.Error(err, "Failed to remove task from robot state")
		}
	}

	logger.Info("Task status updated",
		"task", task.Name,
		"oldState", oldState,
		"newState", newState,
		"message", statusMsg.Message)
}

// sendResponse 发送响应到 Agent（发送到机器人专属 topic）
func (s *Server) sendResponse(robotID string, success bool, message string) {
	logger := log.FromContext(s.ctx)

	response := Response{
		Success: success,
		Message: message,
		RobotID: robotID,
	}

	payload, err := json.Marshal(response)
	if err != nil {
		logger.Error(err, "Failed to marshal response")
		return
	}

	// 发送到机器人专属的 response topic
	topic := fmt.Sprintf(TopicRobotResponse, robotID)
	token := s.mqttClient.Publish(topic, 1, false, payload)
	if token.Wait() && token.Error() != nil {
		logger.Error(token.Error(), "Failed to publish response", "topic", topic)
	}
}

// ===== K8s → MQTT 消息转发 =====

// StartTaskWatcher 监听 Task 状态变化，转发到 MQTT
// 当 Task.Status.State = "pending" 时触发转发，并更新为 "dispatching"
func (s *Server) StartTaskWatcher(ctx context.Context) error {
	logger := log.FromContext(ctx)
	logger.Info("Starting Task watcher")

	// 使用轮询方式监听（简化实现，生产环境应该使用 Informer）
	go func() {
		ticker := time.NewTicker(2 * time.Second)
		defer ticker.Stop()

		dispatchedTasks := make(map[string]bool) // 记录已分发的 Task

		for {
			select {
			case <-ctx.Done():
				return
			case <-ticker.C:
				taskList := &robotv1alpha1.TaskList{}
				if err := s.Client.List(ctx, taskList); err != nil {
					logger.Error(err, "Failed to list tasks")
					continue
				}

				for i := range taskList.Items {
					task := &taskList.Items[i]
					taskKey := string(task.UID)

					// 如果状态是 pending 且未分发过，则转发到 MQTT 并更新为 dispatching
					if task.Status.State == robotv1alpha1.TaskStatePending && !dispatchedTasks[taskKey] {
						if err := s.dispatchTaskToMQTT(ctx, task); err != nil {
							logger.Error(err, "Failed to dispatch task to MQTT", "task", task.Name)
						} else {
							dispatchedTasks[taskKey] = true
						}
					}

					// 如果 Task 被删除，发送删除消息
					if task.DeletionTimestamp != nil && !dispatchedTasks[taskKey+"-delete"] {
						if err := s.deleteTaskViaMQTT(ctx, task); err != nil {
							logger.Error(err, "Failed to send delete task message", "task", task.Name)
						} else {
							dispatchedTasks[taskKey+"-delete"] = true
						}
					}
				}
			}
		}
	}()

	return nil
}

// dispatchTaskToMQTT 将 Task 通过 MQTT 转发给 Agent
// 发送成功后，将 Task 状态从 pending 更新为 dispatching
func (s *Server) dispatchTaskToMQTT(ctx context.Context, task *robotv1alpha1.Task) error {
	logger := log.FromContext(ctx)

	if task.Spec.TargetRobot == "" {
		return fmt.Errorf("task has no target robot")
	}

	// 构造 MQTT 消息
	msg := map[string]interface{}{
		"action": "create",
		"task":   task,
	}

	payload, err := json.Marshal(msg)
	if err != nil {
		return fmt.Errorf("failed to marshal task message: %w", err)
	}

	// 发送到目标 Robot 的专属 task dispatch topic
	topic := fmt.Sprintf(TopicRobotTaskDispatch, task.Spec.TargetRobot)
	token := s.mqttClient.Publish(topic, 1, false, payload)
	token.Wait()
	if err := token.Error(); err != nil {
		return fmt.Errorf("failed to publish task: %w", err)
	}

	logger.Info("Task dispatched to MQTT",
		"task", task.Name,
		"robot", task.Spec.TargetRobot,
		"topic", topic)

	// 更新 Task 状态为 dispatching（表示已通过 MQTT 发送）
	task.Status.State = robotv1alpha1.TaskStateScheduled
	task.Status.Message = fmt.Sprintf("Dispatched to robot %s via MQTT", task.Spec.TargetRobot)
	if err := s.Client.Status().Update(ctx, task); err != nil {
		logger.Error(err, "Failed to update task status to dispatching", "task", task.Name)
		// 不返回错误，因为消息已经发送成功
	}

	// 更新 Robot 的任务状态列表（用于 Agent 重启恢复）
	if err := s.addTaskToRobotState(ctx, task); err != nil {
		logger.Error(err, "Failed to update robot task state", "robot", task.Spec.TargetRobot)
		// 不返回错误，因为任务已经分发成功
	}

	return nil
}

// deleteTaskViaMQTT 通过 MQTT 发送删除消息
func (s *Server) deleteTaskViaMQTT(ctx context.Context, task *robotv1alpha1.Task) error {
	logger := log.FromContext(ctx)

	if task.Spec.TargetRobot == "" {
		return nil // 未分配，无需删除
	}

	msg := map[string]interface{}{
		"action": "delete",
		"task":   task,
	}

	payload, err := json.Marshal(msg)
	if err != nil {
		return fmt.Errorf("failed to marshal delete message: %w", err)
	}

	// 发送删除消息到机器人专属的 task dispatch topic
	topic := fmt.Sprintf(TopicRobotTaskDispatch, task.Spec.TargetRobot)
	token := s.mqttClient.Publish(topic, 1, false, payload)
	token.Wait()
	if err := token.Error(); err != nil {
		return fmt.Errorf("failed to publish delete message: %w", err)
	}

	logger.Info("Delete task message sent", "task", task.Name, "robot", task.Spec.TargetRobot)
	return nil
}

// createTaskEvent 为 Task 创建 Kubernetes Event
func (s *Server) createTaskEvent(task *robotv1alpha1.Task, eventType, message string) {
	logger := log.FromContext(s.ctx)

	// 确定事件类型（Normal 或 Warning）
	eventKind := corev1.EventTypeNormal
	reason := eventType

	// 错误相关的事件使用 Warning 类型
	if eventType == "DownloadFailed" || eventType == "Failed" || eventType == "StartFailed" {
		eventKind = corev1.EventTypeWarning
	}

	// 创建 Event 对象
	event := &corev1.Event{
		ObjectMeta: metav1.ObjectMeta{
			Name:      fmt.Sprintf("%s.%d", task.Name, time.Now().Unix()),
			Namespace: task.Namespace,
		},
		InvolvedObject: corev1.ObjectReference{
			Kind:       "Task",
			Namespace:  task.Namespace,
			Name:       task.Name,
			UID:        task.UID,
			APIVersion: "robot.k8s4r.io/v1alpha1",
		},
		Reason:  reason,
		Message: message,
		Source: corev1.EventSource{
			Component: "k8s4r-agent",
			Host:      task.Spec.TargetRobot,
		},
		FirstTimestamp: metav1.Now(),
		LastTimestamp:  metav1.Now(),
		Count:          1,
		Type:           eventKind,
	}

	if err := s.Client.Create(s.ctx, event); err != nil {
		logger.Error(err, "Failed to create event",
			"task", task.Name,
			"eventType", eventType,
			"message", message)
	} else {
		logger.Info("Created task event",
			"task", task.Name,
			"eventType", eventType,
			"message", message)
	}
}

// Start 启动 MQTT 服务
func (s *Server) Start(ctx context.Context, brokerURL string) error {
	logger := log.FromContext(ctx)
	s.ctx = ctx

	opts := mqtt.NewClientOptions()
	opts.AddBroker(brokerURL)
	opts.SetClientID(fmt.Sprintf("k8s4r-server-%d", time.Now().Unix()))
	opts.SetKeepAlive(60 * time.Second)
	opts.SetPingTimeout(10 * time.Second)
	opts.SetCleanSession(true)
	opts.SetAutoReconnect(true)

	opts.SetOnConnectHandler(func(client mqtt.Client) {
		logger.Info("Connected to MQTT broker")

		// 订阅全局 Topics（Agent → Server）
		client.Subscribe(TopicRegister, 1, s.RegisterHandler)
		client.Subscribe(TopicHeartbeat, 1, s.HeartbeatHandler)

		// 订阅所有机器人的任务状态上报（使用通配符）
		client.Subscribe(TopicRobotTaskStatus, 1, s.TaskStatusHandler)

		logger.Info("Subscribed to MQTT topics",
			"register", TopicRegister,
			"heartbeat", TopicHeartbeat,
			"taskStatus", TopicRobotTaskStatus)
	})

	opts.SetConnectionLostHandler(func(client mqtt.Client, err error) {
		logger.Error(err, "MQTT connection lost")
	})

	s.mqttClient = mqtt.NewClient(opts)
	if token := s.mqttClient.Connect(); token.Wait() && token.Error() != nil {
		return fmt.Errorf("failed to connect to MQTT: %w", token.Error())
	}

	go func() {
		<-ctx.Done()
		s.mqttClient.Disconnect(250)
	}()

	return nil
}

// addTaskToRobotState 将任务添加到 Robot 的状态列表中
func (s *Server) addTaskToRobotState(ctx context.Context, task *robotv1alpha1.Task) error {
	logger := log.FromContext(ctx)

	logger.Info("Updating robot task state",
		"robot", task.Spec.TargetRobot,
		"task", task.Name,
		"taskUID", string(task.UID))

	// 查询所有任务
	taskList := &robotv1alpha1.TaskList{}
	if err := s.Client.List(ctx, taskList); err != nil {
		logger.Error(err, "Failed to list tasks")
		return fmt.Errorf("failed to list tasks: %w", err)
	}

	logger.Info("Total tasks in cluster", "count", len(taskList.Items))

	// 过滤出该 Robot 的未完成任务
	var pendingTasks []*robotv1alpha1.Task
	for i := range taskList.Items {
		t := &taskList.Items[i]

		// 只处理分配给该 Robot 的任务
		if t.Spec.TargetRobot != task.Spec.TargetRobot {
			continue
		}

		// 只保存 pending, dispatching, running 状态的任务
		if t.Status.State == robotv1alpha1.TaskStatePending ||
			t.Status.State == robotv1alpha1.TaskStateScheduled ||
			t.Status.State == robotv1alpha1.TaskStateRunning ||
			t.Status.State == "" {
			pendingTasks = append(pendingTasks, t)
			logger.Info("Found pending task for robot",
				"robot", t.Spec.TargetRobot,
				"task", t.Name,
				"state", t.Status.State,
				"taskUID", string(t.UID))
		}
	}

	logger.Info("Pending tasks for robot",
		"robot", task.Spec.TargetRobot,
		"count", len(pendingTasks))

	// 构造状态消息
	state := struct {
		Tasks []*robotv1alpha1.Task `json:"tasks"`
	}{
		Tasks: pendingTasks,
	}

	payload, err := json.Marshal(state)
	if err != nil {
		return fmt.Errorf("failed to marshal state: %w", err)
	}

	logger.Info("State payload size", "bytes", len(payload))

	// 发布到 state topic（使用 retained message）
	stateTopic := fmt.Sprintf(TopicRobotTaskState, task.Spec.TargetRobot)
	token := s.mqttClient.Publish(stateTopic, 1, true, payload) // retained=true
	token.Wait()
	if err := token.Error(); err != nil {
		logger.Error(err, "Failed to publish state to MQTT", "topic", stateTopic)
		return fmt.Errorf("failed to publish state: %w", err)
	}

	logger.Info("Successfully updated robot task state via MQTT",
		"robot", task.Spec.TargetRobot,
		"topic", stateTopic,
		"taskCount", len(pendingTasks),
		"retained", true)

	return nil
}

// removeTaskFromRobotState 从 Robot 的状态列表中移除任务
func (s *Server) removeTaskFromRobotState(ctx context.Context, task *robotv1alpha1.Task) error {
	// 重新计算并更新状态（和 addTaskToRobotState 逻辑相同）
	return s.addTaskToRobotState(ctx, task)
}
