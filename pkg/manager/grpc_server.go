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
	"time"

	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/log"

	pb "github.com/hxndghxndg/k8s4r/api/grpc"
	robotv1alpha1 "github.com/hxndghxndg/k8s4r/api/v1alpha1"
)

// GRPCServer 实现 RobotManager gRPC 服务
// 这是 Manager 暴露给 Server 的接口
// Server 通过 gRPC 通知 Manager 处理 MQTT 消息
type GRPCServer struct {
	pb.UnimplementedRobotManagerServer
	Client    client.Client
	Namespace string
}

// NewGRPCServer 创建 gRPC Server
func NewGRPCServer(client client.Client, namespace string) *GRPCServer {
	return &GRPCServer{
		Client:    client,
		Namespace: namespace,
	}
}

// CompositeGRPCService 组合 GRPCServer 和 TaskGroupStreamManager
// 实现完整的 RobotManagerServer 接口
type CompositeGRPCService struct {
	*GRPCServer
	*TaskGroupStreamManager
}

// ReportRobotRegistration 处理 Robot 注册
// ========== 设计原则 ==========
// Manager 是唯一的 K8s 状态管理者
// 负责创建 Robot、更新心跳时间、设置 Phase
func (s *GRPCServer) ReportRobotRegistration(ctx context.Context, req *pb.RegistrationRequest) (*pb.RegistrationResponse, error) {
	logger := log.FromContext(ctx)
	logger.Info(" [GRPC] Received registration request", "robotId", req.RobotId)

	// 查找或创建 Robot 资源
	robot := &robotv1alpha1.Robot{}
	err := s.Client.Get(ctx, types.NamespacedName{
		Name:      req.RobotId,
		Namespace: s.Namespace,
	}, robot)

	if err != nil {
		// Robot 不存在，创建新的
		robot = &robotv1alpha1.Robot{
			ObjectMeta: metav1.ObjectMeta{
				Name:      req.RobotId,
				Namespace: s.Namespace,
			},
			Spec: robotv1alpha1.RobotSpec{
				RobotID:     req.RobotId,
				Description: "Registered by agent",
			},
			Status: robotv1alpha1.RobotStatus{
				Phase:   robotv1alpha1.RobotPhasePending,
				Message: "Registration received",
			},
		}

		if err := s.Client.Create(ctx, robot); err != nil {
			logger.Error(err, " [GRPC] Failed to create Robot", "robotId", req.RobotId)
			return &pb.RegistrationResponse{
				Success: false,
				Message: "Failed to create robot: " + err.Error(),
			}, nil
		}
		logger.Info(" [GRPC] Created Robot resource", "robotId", req.RobotId)

		// 重新获取创建后的 Robot（获取 UID 等字段）
		if err := s.Client.Get(ctx, types.NamespacedName{
			Name:      req.RobotId,
			Namespace: s.Namespace,
		}, robot); err != nil {
			logger.Error(err, "Failed to get created robot")
			return &pb.RegistrationResponse{
				Success: false,
				Message: "Robot created but failed to retrieve",
			}, nil
		}
	}

	// 更新心跳时间和设备信息
	now := metav1.Now()
	robot.Status.LastHeartbeatTime = &now

	// 优先使用完整的JSON DeviceInfo
	if req.DeviceInfoJson != "" {
		var fullDeviceInfo robotv1alpha1.DeviceInfo
		if err := json.Unmarshal([]byte(req.DeviceInfoJson), &fullDeviceInfo); err != nil {
			logger.Error(err, "Failed to unmarshal device info JSON, falling back to protobuf")
			robot.Status.DeviceInfo = convertDeviceInfo(req.DeviceInfo)
		} else {
			robot.Status.DeviceInfo = &fullDeviceInfo
			logger.Info(" [DEVICE] Received full device info",
				"robotId", req.RobotId,
				"hostname", fullDeviceInfo.Hostname,
				"cpus", fullDeviceInfo.CPU.LogicalCores,
				"memory", fullDeviceInfo.Memory.Total,
				"disks", len(fullDeviceInfo.Disks))
		}
	} else {
		robot.Status.DeviceInfo = convertDeviceInfo(req.DeviceInfo)
	}

	robot.Status.Phase = robotv1alpha1.RobotPhaseOnline
	robot.Status.Message = "Robot registered and online"

	if err := s.Client.Status().Update(ctx, robot); err != nil {
		logger.Error(err, " [GRPC] Failed to update Robot status", "robotId", req.RobotId)
		return &pb.RegistrationResponse{
			Success: false,
			Message: "Failed to update robot status: " + err.Error(),
		}, nil
	}

	logger.Info(" [GRPC] Robot registration processed",
		"robotId", req.RobotId,
		"phase", robot.Status.Phase,
		"heartbeat", now.Format(time.RFC3339))

	return &pb.RegistrationResponse{
		Success: true,
		Message: "Registration successful",
	}, nil
}

// ReportRobotHeartbeat 处理 Robot 心跳
func (s *GRPCServer) ReportRobotHeartbeat(ctx context.Context, req *pb.HeartbeatRequest) (*pb.HeartbeatResponse, error) {
	logger := log.FromContext(ctx)
	logger.V(1).Info(" [GRPC] Received heartbeat", "robotId", req.RobotId)

	// 获取 Robot 资源
	robot := &robotv1alpha1.Robot{}
	err := s.Client.Get(ctx, types.NamespacedName{
		Name:      req.RobotId,
		Namespace: s.Namespace,
	}, robot)

	if err != nil {
		logger.Error(err, " [GRPC] Robot not found", "robotId", req.RobotId)
		return &pb.HeartbeatResponse{
			Success: false,
			Message: "Robot not found: " + err.Error(),
		}, nil
	}

	// 更新心跳时间和设备信息
	now := metav1.Now()
	robot.Status.LastHeartbeatTime = &now

	// 优先使用完整的JSON DeviceInfo
	if req.DeviceInfoJson != "" {
		var fullDeviceInfo robotv1alpha1.DeviceInfo
		if err := json.Unmarshal([]byte(req.DeviceInfoJson), &fullDeviceInfo); err != nil {
			logger.Error(err, "Failed to unmarshal device info JSON, falling back to protobuf")
			if req.DeviceInfo != nil {
				robot.Status.DeviceInfo = convertDeviceInfo(req.DeviceInfo)
			}
		} else {
			robot.Status.DeviceInfo = &fullDeviceInfo
		}
	} else if req.DeviceInfo != nil {
		robot.Status.DeviceInfo = convertDeviceInfo(req.DeviceInfo)
	}

	// 收到心跳，设置为 Online
	oldPhase := robot.Status.Phase
	robot.Status.Phase = robotv1alpha1.RobotPhaseOnline
	robot.Status.Message = "Robot is online"

	if err := s.Client.Status().Update(ctx, robot); err != nil {
		logger.Error(err, " [GRPC] Failed to update heartbeat", "robotId", req.RobotId)
		return &pb.HeartbeatResponse{
			Success: false,
			Message: "Failed to update heartbeat: " + err.Error(),
		}, nil
	}

	if oldPhase != robot.Status.Phase {
		logger.Info(" [GRPC] Robot phase changed",
			"robotId", req.RobotId,
			"oldPhase", oldPhase,
			"newPhase", robot.Status.Phase,
			"heartbeat", now.Format(time.RFC3339))
	}

	return &pb.HeartbeatResponse{
		Success: true,
		Message: "Heartbeat updated",
	}, nil
}

// ReportTaskStatus 处理任务状态上报
func (s *GRPCServer) ReportTaskStatus(ctx context.Context, req *pb.TaskStatusRequest) (*pb.TaskStatusResponse, error) {
	logger := log.FromContext(ctx)
	logger.Info(" [GRPC] Received task status",
		"taskUid", req.TaskUid,
		"state", req.State,
		"event", req.Event)

	// 查找对应的 Task
	taskList := &robotv1alpha1.TaskList{}
	if err := s.Client.List(ctx, taskList); err != nil {
		logger.Error(err, "Failed to list tasks")
		return &pb.TaskStatusResponse{
			Success: false,
			Message: "Failed to list tasks: " + err.Error(),
		}, nil
	}

	var task *robotv1alpha1.Task
	for i := range taskList.Items {
		t := &taskList.Items[i]
		if string(t.UID) == req.TaskUid {
			task = t
			break
		}
	}

	if task == nil {
		logger.Error(nil, " [GRPC] Task not found", "taskUid", req.TaskUid)
		return &pb.TaskStatusResponse{
			Success: false,
			Message: "Task not found",
		}, nil
	}

	// 更新 Task 状态
	oldState := task.Status.State
	newState := robotv1alpha1.TaskState(req.State)

	// 特殊处理：如果是 pending 状态上报，且当前是 scheduled，转为 running
	if newState == "pending" && task.Status.State == robotv1alpha1.TaskStateScheduled {
		newState = robotv1alpha1.TaskStateRunning
		logger.Info("Task state transition: scheduled -> running (agent acknowledged)")
	}

	task.Status.State = newState
	task.Status.Message = req.Message
	exitCode := int32(req.ExitCode)
	task.Status.ExitCode = &exitCode

	if err := s.Client.Status().Update(ctx, task); err != nil {
		logger.Error(err, " [GRPC] Failed to update task status", "task", task.Name)
		return &pb.TaskStatusResponse{
			Success: false,
			Message: "Failed to update task status: " + err.Error(),
		}, nil
	}

	logger.Info(" [GRPC] Task status updated",
		"task", task.Name,
		"oldState", oldState,
		"newState", newState,
		"event", req.Event)

	return &pb.TaskStatusResponse{
		Success: true,
		Message: "Task status updated",
	}, nil
}

// convertDeviceInfo 将 protobuf DeviceInfo 转换为 K8s DeviceInfo
func convertDeviceInfo(pbInfo *pb.DeviceInfo) *robotv1alpha1.DeviceInfo {
	if pbInfo == nil {
		return nil
	}

	now := metav1.Now()
	return &robotv1alpha1.DeviceInfo{
		Hostname: pbInfo.Hostname,
		Platform: robotv1alpha1.PlatformInfo{
			OS:   pbInfo.Os,
			Arch: pbInfo.Arch,
		},
		CPU: robotv1alpha1.CPUInfo{
			LogicalCores: int(pbInfo.NumCpus),
		},
		Memory: robotv1alpha1.MemoryInfo{
			Total: uint64(pbInfo.TotalMemory),
		},
		LastUpdated: &now,
	}
}

// convertDeviceInfoToProto 将 K8s DeviceInfo 转换为 protobuf DeviceInfo（如果需要）
func convertDeviceInfoToProto(info *robotv1alpha1.DeviceInfo) *pb.DeviceInfo {
	if info == nil {
		return nil
	}

	return &pb.DeviceInfo{
		Hostname:    info.Hostname,
		Os:          info.Platform.OS,
		Arch:        info.Platform.Arch,
		NumCpus:     int32(info.CPU.LogicalCores),
		TotalMemory: int64(info.Memory.Total),
	}
}

// DeviceInfoToJSON 将 DeviceInfo 转换为 JSON（工具函数）
func DeviceInfoToJSON(info *robotv1alpha1.DeviceInfo) string {
	if info == nil {
		return "{}"
	}
	data, _ := json.Marshal(info)
	return string(data)
}
