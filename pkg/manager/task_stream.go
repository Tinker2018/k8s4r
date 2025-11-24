/*
Copyright 2025.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0
*/

package manager

import (
	"context"
	"encoding/json"
	"sync"
	"time"

	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/log"

	pb "github.com/hxndghxndg/k8s4r/api/grpc"
	robotv1alpha1 "github.com/hxndghxndg/k8s4r/api/v1alpha1"
)

// TaskStreamManager 管理与 Server 的 Task 流连接
type TaskStreamManager struct {
	client    client.Client
	namespace string
	streams   map[string]pb.RobotManager_StreamTasksServer // streamID -> stream
	streamsMu sync.RWMutex

	// 记录已分发的 Task（避免重复发送）
	dispatchedTasks map[string]bool // taskUID -> dispatched
	dispatchedMu    sync.RWMutex
}

// NewTaskStreamManager 创建 TaskStreamManager
func NewTaskStreamManager(client client.Client, namespace string) *TaskStreamManager {
	return &TaskStreamManager{
		client:          client,
		namespace:       namespace,
		streams:         make(map[string]pb.RobotManager_StreamTasksServer),
		dispatchedTasks: make(map[string]bool),
	}
}

// StreamTasks 处理 Server 的流连接
// ========== gRPC 双向流工作原理 ==========
// 1. Server 调用此方法，建立长连接
// 2. Manager 可以通过 stream.Send() 发送 TaskCommand 给 Server
// 3. Server 可以通过 stream (客户端) 的 Recv() 接收 TaskCommand
// 4. Server 可以通过 stream (客户端) 的 Send() 发送 TaskEvent 给 Manager
// 5. Manager 可以通过 stream.Recv() 接收 TaskEvent
func (m *TaskStreamManager) StreamTasks(stream pb.RobotManager_StreamTasksServer) error {
	logger := log.Log.WithName("task-stream")
	streamID := generateStreamID()

	logger.Info(" [STREAM] New stream connection established", "streamID", streamID)

	// 注册 stream
	m.streamsMu.Lock()
	m.streams[streamID] = stream
	m.streamsMu.Unlock()

	// 连接断开时清理
	defer func() {
		m.streamsMu.Lock()
		delete(m.streams, streamID)
		m.streamsMu.Unlock()
		logger.Info(" [STREAM] Stream connection closed", "streamID", streamID)
	}()

	// 发送 keepalive（验证连接）
	if err := stream.Send(&pb.TaskCommand{
		Type: pb.TaskCommand_KEEPALIVE,
	}); err != nil {
		logger.Error(err, "Failed to send keepalive")
		return err
	}

	// 持续接收 Server 发来的 TaskEvent
	for {
		event, err := stream.Recv()
		if err != nil {
			logger.Error(err, "Stream recv error")
			return err
		}

		logger.Info(" [STREAM] Received event from Server",
			"type", event.Type,
			"taskUid", event.TaskUid,
			"message", event.Message)

		// 处理 Server 的事件
		switch event.Type {
		case pb.TaskEvent_ACK:
			logger.V(1).Info("Server acknowledged command", "taskUid", event.TaskUid)

		case pb.TaskEvent_PUBLISHED:
			// Server 已将任务发布到 MQTT，更新 Task 状态为 dispatching
			if err := m.updateTaskStateToDispatching(stream.Context(), event.TaskUid); err != nil {
				logger.Error(err, "Failed to update task state", "taskUid", event.TaskUid)
			}

		case pb.TaskEvent_ERROR:
			logger.Error(nil, "Server reported error", "taskUid", event.TaskUid, "message", event.Message)
			// TODO: 可以重试或标记任务失败

		case pb.TaskEvent_KEEPALIVE:
			logger.V(1).Info("Received keepalive from Server")
		}
	}
}

// updateTaskStateToDispatching 更新任务状态为 dispatching
func (m *TaskStreamManager) updateTaskStateToDispatching(ctx context.Context, taskUID string) error {
	logger := log.FromContext(ctx)

	// 查找对应的 Task
	taskList := &robotv1alpha1.TaskList{}
	if err := m.client.List(ctx, taskList, client.InNamespace(m.namespace)); err != nil {
		return err
	}

	for i := range taskList.Items {
		task := &taskList.Items[i]
		if string(task.UID) == taskUID {
			// 更新状态
			task.Status.State = robotv1alpha1.TaskStateScheduled
			task.Status.Message = "Task published to MQTT"

			if err := m.client.Status().Update(ctx, task); err != nil {
				return err
			}

			logger.Info(" [STREAM] Task state updated to scheduled",
				"task", task.Name,
				"taskUid", taskUID)
			return nil
		}
	}

	return nil
}

// PushTaskToStream 推送任务到所有连接的 Server
// ========== 工作流程 ==========
// 1. Manager 的 TaskController 检测到 Task.Status.State = pending
// 2. 调用此方法推送任务
// 3. 遍历所有活跃的 stream
// 4. 通过 stream.Send() 发送 TaskCommand
// 5. Server 端会通过 stream.Recv() 接收到这个 TaskCommand
func (m *TaskStreamManager) PushTaskToStream(ctx context.Context, task *robotv1alpha1.Task) error {
	logger := log.FromContext(ctx)

	taskUID := string(task.UID)

	// 检查是否已经分发过
	m.dispatchedMu.RLock()
	if m.dispatchedTasks[taskUID] {
		m.dispatchedMu.RUnlock()
		logger.V(1).Info("Task already dispatched, skipping", "task", task.Name)
		return nil
	}
	m.dispatchedMu.RUnlock()

	// 构造 TaskCommand
	command := &pb.TaskCommand{
		Type: pb.TaskCommand_CREATE_TASK,
		Task: convertTaskToProto(task),
	}

	// 推送到所有活跃的 stream
	m.streamsMu.RLock()
	streamCount := len(m.streams)
	m.streamsMu.RUnlock()

	if streamCount == 0 {
		logger.Info("⚠️ [STREAM] No active streams, task will be queued", "task", task.Name)
		return nil
	}

	m.streamsMu.RLock()
	defer m.streamsMu.RUnlock()

	successCount := 0
	for streamID, stream := range m.streams {
		if err := stream.Send(command); err != nil {
			logger.Error(err, "Failed to send task to stream", "streamID", streamID, "task", task.Name)
		} else {
			successCount++
			logger.Info(" [STREAM] Task pushed to Server",
				"streamID", streamID,
				"task", task.Name,
				"targetRobot", task.Spec.TargetRobot)
		}
	}

	if successCount > 0 {
		// 标记为已分发
		m.dispatchedMu.Lock()
		m.dispatchedTasks[taskUID] = true
		m.dispatchedMu.Unlock()

		logger.Info(" [STREAM] Task dispatched successfully",
			"task", task.Name,
			"streamCount", successCount)
	}

	return nil
}

// PushDeleteTaskToStream 推送任务删除消息
func (m *TaskStreamManager) PushDeleteTaskToStream(ctx context.Context, taskUID string) error {
	logger := log.FromContext(ctx)

	command := &pb.TaskCommand{
		Type:    pb.TaskCommand_DELETE_TASK,
		TaskUid: taskUID,
	}

	m.streamsMu.RLock()
	defer m.streamsMu.RUnlock()

	for streamID, stream := range m.streams {
		if err := stream.Send(command); err != nil {
			logger.Error(err, "Failed to send delete command", "streamID", streamID, "taskUid", taskUID)
		} else {
			logger.Info(" [STREAM] Delete command sent", "streamID", streamID, "taskUid", taskUID)
		}
	}

	// 清理分发记录
	m.dispatchedMu.Lock()
	delete(m.dispatchedTasks, taskUID)
	m.dispatchedMu.Unlock()

	return nil
}

// convertTaskToProto 将 K8s Task 转换为 protobuf Task
func convertTaskToProto(task *robotv1alpha1.Task) *pb.Task {
	configJSON, _ := json.Marshal(task.Spec.Config)

	timeout := int64(0)
	if task.Spec.Timeout != nil {
		timeout = int64(task.Spec.Timeout.Seconds())
	}

	killTimeout := int64(0)
	if task.Spec.KillTimeout != nil {
		killTimeout = int64(task.Spec.KillTimeout.Seconds())
	}

	return &pb.Task{
		Uid:         string(task.UID),
		Name:        task.Name,
		Namespace:   task.Namespace,
		JobName:     task.Spec.JobName,
		TargetRobot: task.Spec.TargetRobot,
		Driver:      string(task.Spec.Driver),
		Config:      string(configJSON),
		Timeout:     timeout,
		KillTimeout: killTimeout,
		Env:         task.Spec.Env,
		State:       string(task.Status.State),
		Message:     task.Status.Message,
	}
}

// generateStreamID 生成唯一的 stream ID
func generateStreamID() string {
	return time.Now().Format("20060102-150405.000")
}
