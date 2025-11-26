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
	"fmt"
	"sync"
	"time"

	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/log"

	pb "github.com/hxndghxndg/k8s4r/api/grpc"
	robotv1alpha1 "github.com/hxndghxndg/k8s4r/api/v1alpha1"
)

// TaskGroupStreamManager 管理与 Server 的 TaskGroup 流连接
type TaskGroupStreamManager struct {
	client    client.Client
	namespace string
	streams   map[string]pb.RobotManager_StreamTaskGroupsServer // streamID -> stream
	streamsMu sync.RWMutex

	// 记录已分发的 TaskGroup（避免重复发送）
	dispatchedTaskGroups map[string]bool // taskGroupUID -> dispatched
	dispatchedMu         sync.RWMutex
}

// NewTaskGroupStreamManager 创建 TaskGroupStreamManager
func NewTaskGroupStreamManager(client client.Client, namespace string) *TaskGroupStreamManager {
	return &TaskGroupStreamManager{
		client:               client,
		namespace:            namespace,
		streams:              make(map[string]pb.RobotManager_StreamTaskGroupsServer),
		dispatchedTaskGroups: make(map[string]bool),
	}
}

// StreamTaskGroups 处理 Server 的流连接
// ========== gRPC 双向流工作原理 ==========
// 1. Server 调用此方法，建立长连接
// 2. Manager 可以通过 stream.Send() 发送 TaskGroupCommand 给 Server
// 3. Server 可以通过 stream (客户端) 的 Recv() 接收 TaskGroupCommand
// 4. Server 可以通过 stream (客户端) 的 Send() 发送 TaskGroupEvent 给 Manager
// 5. Manager 可以通过 stream.Recv() 接收 TaskGroupEvent
func (m *TaskGroupStreamManager) StreamTaskGroups(stream pb.RobotManager_StreamTaskGroupsServer) error {
	logger := log.Log.WithName("taskgroup-stream")
	streamID := generateStreamID()

	logger.Info(" [STREAM] New TaskGroup stream connection established", "streamID", streamID)

	// 注册 stream
	m.streamsMu.Lock()
	m.streams[streamID] = stream
	m.streamsMu.Unlock()

	// 连接断开时清理
	defer func() {
		m.streamsMu.Lock()
		delete(m.streams, streamID)
		m.streamsMu.Unlock()
		logger.Info(" [STREAM] TaskGroup stream connection closed", "streamID", streamID)
	}()

	// 发送 keepalive（验证连接）
	if err := stream.Send(&pb.TaskGroupCommand{
		Type: pb.TaskGroupCommand_KEEPALIVE,
	}); err != nil {
		logger.Error(err, "Failed to send keepalive")
		return err
	}

	// 持续接收 Server 发来的 TaskGroupEvent
	for {
		event, err := stream.Recv()
		if err != nil {
			logger.Error(err, "Stream recv error")
			return err
		}

		logger.Info(" [STREAM] Received event from Server",
			"type", event.Type,
			"taskGroupUID", event.TaskgroupUid,
			"robotName", event.RobotName,
			"message", event.Message)

		// 处理 Server 的事件
		switch event.Type {
		case pb.TaskGroupEvent_ACK:
			logger.V(1).Info("Server acknowledged command", "taskGroupUID", event.TaskgroupUid)

		case pb.TaskGroupEvent_PUBLISHED:
			// Server 已将 TaskGroup 发布到 MQTT
			logger.Info("TaskGroup published to MQTT",
				"taskGroupUID", event.TaskgroupUid,
				"robotName", event.RobotName)

		case pb.TaskGroupEvent_ERROR:
			logger.Error(nil, "Server reported error",
				"taskGroupUID", event.TaskgroupUid,
				"message", event.Message)

		case pb.TaskGroupEvent_KEEPALIVE:
			logger.V(1).Info("Received keepalive from Server")
		}
	}
}

// PushTaskGroupToStream 推送 TaskGroup 到所有连接的 Server
// ========== 工作流程 ==========
// 1. Manager 的 TaskGroupController 检测到 TaskGroup.Status.State = scheduled
// 2. 调用此方法推送 TaskGroup
// 3. 遍历所有活跃的 stream
// 4. 通过 stream.Send() 发送 TaskGroupCommand
// 5. Server 端会通过 stream.Recv() 接收到这个 TaskGroupCommand
func (m *TaskGroupStreamManager) PushTaskGroupToStream(ctx context.Context, taskGroup *robotv1alpha1.TaskGroup, robotName string) error {
	logger := log.FromContext(ctx)

	taskGroupUID := string(taskGroup.UID)
	dispatchKey := taskGroupUID + "-" + robotName

	// 检查是否已经分发过
	m.dispatchedMu.RLock()
	if m.dispatchedTaskGroups[dispatchKey] {
		m.dispatchedMu.RUnlock()
		logger.V(1).Info("TaskGroup already dispatched to robot, skipping",
			"taskGroup", taskGroup.Name,
			"robot", robotName)
		return nil
	}
	m.dispatchedMu.RUnlock()

	// 将 TaskGroup 序列化为 JSON
	taskGroupJSON, err := json.Marshal(taskGroup)
	if err != nil {
		return fmt.Errorf("failed to marshal taskgroup: %w", err)
	}

	// 构造 TaskGroupCommand
	command := &pb.TaskGroupCommand{
		Type:          pb.TaskGroupCommand_CREATE_TASKGROUP,
		TaskgroupJson: string(taskGroupJSON),
		TaskgroupUid:  taskGroupUID,
		RobotName:     robotName,
	}

	// 推送到所有活跃的 stream
	m.streamsMu.RLock()
	streamCount := len(m.streams)
	m.streamsMu.RUnlock()

	if streamCount == 0 {
		logger.Info("⚠️ [STREAM] No active streams, taskgroup will be queued",
			"taskGroup", taskGroup.Name,
			"robot", robotName)
		return nil
	}

	m.streamsMu.RLock()
	defer m.streamsMu.RUnlock()

	successCount := 0
	for streamID, stream := range m.streams {
		if err := stream.Send(command); err != nil {
			logger.Error(err, "Failed to send taskgroup to stream",
				"streamID", streamID,
				"taskGroup", taskGroup.Name,
				"robot", robotName)
		} else {
			successCount++
			logger.Info(" [STREAM] TaskGroup pushed to Server",
				"streamID", streamID,
				"taskGroup", taskGroup.Name,
				"robot", robotName)
		}
	}

	if successCount > 0 {
		// 标记为已分发
		m.dispatchedMu.Lock()
		m.dispatchedTaskGroups[dispatchKey] = true
		m.dispatchedMu.Unlock()

		logger.Info(" [STREAM] TaskGroup dispatched successfully",
			"taskGroup", taskGroup.Name,
			"robot", robotName,
			"streamCount", successCount)
	}

	return nil
}

// PushDeleteTaskGroupToStream 推送 TaskGroup 删除消息
func (m *TaskGroupStreamManager) PushDeleteTaskGroupToStream(ctx context.Context, taskGroupUID string, robotName string) error {
	logger := log.FromContext(ctx)

	command := &pb.TaskGroupCommand{
		Type:         pb.TaskGroupCommand_DELETE_TASKGROUP,
		TaskgroupUid: taskGroupUID,
		RobotName:    robotName,
	}

	m.streamsMu.RLock()
	defer m.streamsMu.RUnlock()

	for streamID, stream := range m.streams {
		if err := stream.Send(command); err != nil {
			logger.Error(err, "Failed to send delete command",
				"streamID", streamID,
				"taskGroupUID", taskGroupUID,
				"robot", robotName)
		} else {
			logger.Info(" [STREAM] Delete command sent",
				"streamID", streamID,
				"taskGroupUID", taskGroupUID,
				"robot", robotName)
		}
	}

	// 清理分发记录
	dispatchKey := taskGroupUID + "-" + robotName
	m.dispatchedMu.Lock()
	delete(m.dispatchedTaskGroups, dispatchKey)
	m.dispatchedMu.Unlock()

	return nil
}

// generateStreamID 生成唯一的 stream ID
func generateStreamID() string {
	return time.Now().Format("20060102-150405.000")
}
