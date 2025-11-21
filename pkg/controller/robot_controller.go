/*
Copyright 2025.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0
*/

package controller

import (
	"context"
	"encoding/json"
	"time"

	"k8s.io/apimachinery/pkg/api/errors"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/runtime"
	ctrl "sigs.k8s.io/controller-runtime"
	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/log"

	robotv1alpha1 "github.com/hxndg/k8s4r/api/v1alpha1"
)

// RobotReconciler reconciles a Robot object
type RobotReconciler struct {
	client.Client
	Scheme *runtime.Scheme
}

const (
	// HeartbeatTimeout 定义心跳超时时间（90秒 = 3倍心跳间隔）
	// Agent 默认每30秒发送一次心跳，超时设为90秒留有容错空间
	HeartbeatTimeout = 90 * time.Second
)

// +kubebuilder:rbac:groups=robot.k8s4r.io,resources=robots,verbs=get;list;watch;create;update;patch;delete
// +kubebuilder:rbac:groups=robot.k8s4r.io,resources=robots/status,verbs=get;update;patch
// +kubebuilder:rbac:groups=robot.k8s4r.io,resources=robots/finalizers,verbs=update

// Reconcile is part of the main kubernetes reconciliation loop
// ========== 设计原则 ==========
// RobotController 是唯一的状态管理者，负责：
// 1. 初始化新创建的 Robot（Pending 状态）
// 2. 处理注册/心跳事件（读取 annotation，更新 status）
// 3. 检测心跳超时（设置 Offline 状态）
func (r *RobotReconciler) Reconcile(ctx context.Context, req ctrl.Request) (ctrl.Result, error) {
	logger := log.FromContext(ctx)

	// 获取 Robot 资源
	robot := &robotv1alpha1.Robot{}
	if err := r.Get(ctx, req.NamespacedName, robot); err != nil {
		if errors.IsNotFound(err) {
			// Robot 已被删除
			logger.Info("Robot resource not found. Ignoring since object must be deleted")
			return ctrl.Result{}, nil
		}
		logger.Error(err, "Failed to get Robot")
		return ctrl.Result{}, err
	}

	// ========== 🔥 状态更新点 #0: 初始化新创建的 Robot ==========
	if robot.Status.Phase == "" {
		robot.Status.Phase = robotv1alpha1.RobotPhasePending
		robot.Status.Message = "Waiting for agent to register"
		if err := r.Status().Update(ctx, robot); err != nil {
			logger.Error(err, "Failed to update Robot status")
			return ctrl.Result{}, err
		}
		logger.Info("🔥 [CONTROLLER] Initialize - Set Phase to Pending", "robotId", robot.Spec.RobotID)
		return ctrl.Result{RequeueAfter: 10 * time.Second}, nil
	}

	// ========== 🔥 状态更新点 #1: 处理注册/心跳事件（从 annotation 读取） ==========
	// Server 收到 MQTT 消息后会更新 annotation，这里检测并处理
	if lastHeartbeatAnnotation, exists := robot.Annotations["k8s4r.io/last-heartbeat"]; exists {
		// 解析 annotation 中的时间
		annotationTime, err := time.Parse(time.RFC3339, lastHeartbeatAnnotation)
		if err == nil {
			// 检查是否是新的心跳（比 status 中的时间更新）
			isNewHeartbeat := robot.Status.LastHeartbeatTime == nil ||
				annotationTime.After(robot.Status.LastHeartbeatTime.Time)

			if isNewHeartbeat {
				// 更新心跳时间
				now := metav1.NewTime(annotationTime)
				robot.Status.LastHeartbeatTime = &now

				// 更新 DeviceInfo（如果 annotation 中有）
				if deviceInfoJSON, exists := robot.Annotations["k8s4r.io/device-info"]; exists {
					var deviceInfo robotv1alpha1.DeviceInfo
					if err := json.Unmarshal([]byte(deviceInfoJSON), &deviceInfo); err == nil {
						robot.Status.DeviceInfo = &deviceInfo
					}
				}

				// 收到心跳，设置为 Online
				oldPhase := robot.Status.Phase
				robot.Status.Phase = robotv1alpha1.RobotPhaseOnline
				robot.Status.Message = "Robot is online"

				if err := r.Status().Update(ctx, robot); err != nil {
					logger.Error(err, "Failed to update Robot status")
					return ctrl.Result{}, err
				}

				logger.Info("🔥 [CONTROLLER] Heartbeat - Updated from annotation",
					"robotId", robot.Spec.RobotID,
					"oldPhase", oldPhase,
					"newPhase", robot.Status.Phase,
					"heartbeatTime", annotationTime.Format(time.RFC3339))
			}
		}
	}

	// ========== 🔥 状态更新点 #2: 检测心跳超时 ==========
	if robot.Status.LastHeartbeatTime != nil {
		timeSinceLastHeartbeat := time.Since(robot.Status.LastHeartbeatTime.Time)

		logger.V(1).Info("🔍 [CONTROLLER] Checking robot heartbeat",
			"robotId", robot.Spec.RobotID,
			"currentPhase", robot.Status.Phase,
			"lastHeartbeatTime", robot.Status.LastHeartbeatTime.Format(time.RFC3339),
			"timeSinceHeartbeat", timeSinceLastHeartbeat,
			"timeout", HeartbeatTimeout,
			"isOnline", timeSinceLastHeartbeat <= HeartbeatTimeout)

		// 心跳超时 → Offline
		if timeSinceLastHeartbeat > HeartbeatTimeout {
			if robot.Status.Phase != robotv1alpha1.RobotPhaseOffline {
				robot.Status.Phase = robotv1alpha1.RobotPhaseOffline
				robot.Status.Message = "Heartbeat timeout"

				if err := r.Status().Update(ctx, robot); err != nil {
					logger.Error(err, "Failed to update Robot status to Offline")
					return ctrl.Result{}, err
				}
				logger.Info("🔥 [CONTROLLER] Timeout - Set Phase to Offline",
					"robotId", robot.Spec.RobotID,
					"timeSinceHeartbeat", timeSinceLastHeartbeat,
					"timeout", HeartbeatTimeout)
			}
		}
	} else {
		// 没有心跳记录
		logger.V(1).Info("Robot has no heartbeat record", "robotId", robot.Spec.RobotID)
	}

	// 定期检查心跳（每30秒）
	return ctrl.Result{RequeueAfter: 30 * time.Second}, nil
}

// SetupWithManager sets up the controller with the Manager.
func (r *RobotReconciler) SetupWithManager(mgr ctrl.Manager) error {
	// ⚠️ 已禁用 periodicHealthCheck，避免缓存一致性问题
	// Reconcile 循环每 30 秒已经在检查心跳，且 Server 更新心跳后会通过 annotation 触发 Reconcile
	// go r.periodicHealthCheck(mgr.GetClient())

	return ctrl.NewControllerManagedBy(mgr).
		For(&robotv1alpha1.Robot{}).
		Complete(r)
}

// periodicHealthCheck 定期检查所有 Robot 的健康状态
func (r *RobotReconciler) periodicHealthCheck(client client.Client) {
	ticker := time.NewTicker(30 * time.Second)
	defer ticker.Stop()

	logger := ctrl.Log.WithName("robot-health-checker")

	for range ticker.C {
		ctx := context.Background()
		robotList := &robotv1alpha1.RobotList{}

		if err := client.List(ctx, robotList); err != nil {
			logger.Error(err, "Failed to list robots for health check")
			continue
		}

		for i := range robotList.Items {
			robot := &robotList.Items[i]

			// 检查心跳
			if robot.Status.LastHeartbeatTime != nil {
				timeSinceLastHeartbeat := time.Since(robot.Status.LastHeartbeatTime.Time)

				// ========== 🔥 ROBOT 状态更新点 #5: 定期健康检查 - 设置为 Offline ==========
				// 📌 心跳超时且状态不是 Offline
				if timeSinceLastHeartbeat > HeartbeatTimeout && robot.Status.Phase != robotv1alpha1.RobotPhaseOffline {
					logger.Info("🔥 [ROBOT UPDATE] Periodic Health Check - Detected offline robot",
						"robot", robot.Name,
						"timeSinceHeartbeat", timeSinceLastHeartbeat,
						"currentPhase", robot.Status.Phase)

					robot.Status.Phase = robotv1alpha1.RobotPhaseOffline
					robot.Status.Message = "Heartbeat timeout"

					if err := client.Status().Update(ctx, robot); err != nil {
						logger.Error(err, "Failed to mark robot as offline", "robot", robot.Name)
					} else {
						logger.Info("🔥 [ROBOT UPDATE] Periodic Health Check - Set Phase to Offline", "robot", robot.Name)
					}
				}

				// ========== 🔥 ROBOT 状态更新点 #6: 定期健康检查 - 设置为 Online ==========
				// 📌 心跳正常且状态不是 Online
				if timeSinceLastHeartbeat <= HeartbeatTimeout && robot.Status.Phase != robotv1alpha1.RobotPhaseOnline {
					logger.Info("🔥 [ROBOT UPDATE] Periodic Health Check - Detected online robot",
						"robot", robot.Name,
						"timeSinceHeartbeat", timeSinceLastHeartbeat,
						"currentPhase", robot.Status.Phase)

					robot.Status.Phase = robotv1alpha1.RobotPhaseOnline
					robot.Status.Message = "Robot is online"

					if err := client.Status().Update(ctx, robot); err != nil {
						logger.Error(err, "Failed to mark robot as online", "robot", robot.Name)
					} else {
						logger.Info("🔥 [ROBOT UPDATE] Periodic Health Check - Set Phase to Online", "robot", robot.Name)
					}
				}
			}
		}
	}
}
