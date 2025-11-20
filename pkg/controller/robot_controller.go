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
	"time"

	"k8s.io/apimachinery/pkg/api/errors"
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

	// 如果是新创建的 Robot，初始化状态
	if robot.Status.Phase == "" {
		robot.Status.Phase = robotv1alpha1.RobotPhasePending
		robot.Status.Message = "Waiting for agent to register"
		if err := r.Status().Update(ctx, robot); err != nil {
			logger.Error(err, "Failed to update Robot status")
			return ctrl.Result{}, err
		}
		logger.Info("Initialized Robot status", "robotId", robot.Spec.RobotID)
		return ctrl.Result{RequeueAfter: 10 * time.Second}, nil
	}

	// 检查心跳并更新状态
	if robot.Status.LastHeartbeatTime != nil {
		timeSinceLastHeartbeat := time.Since(robot.Status.LastHeartbeatTime.Time)

		logger.V(1).Info("Checking robot heartbeat",
			"robotId", robot.Spec.RobotID,
			"currentPhase", robot.Status.Phase,
			"timeSinceHeartbeat", timeSinceLastHeartbeat,
			"timeout", HeartbeatTimeout)

		// 如果心跳间隔小于超时时间，认为在线
		if timeSinceLastHeartbeat <= HeartbeatTimeout {
			// 任何非 Online 状态且心跳正常 → Online
			if robot.Status.Phase != robotv1alpha1.RobotPhaseOnline {
				robot.Status.Phase = robotv1alpha1.RobotPhaseOnline
				robot.Status.Message = "Robot is online"

				if err := r.Status().Update(ctx, robot); err != nil {
					logger.Error(err, "Failed to update Robot status to Online")
					return ctrl.Result{}, err
				}
				logger.Info("Robot status updated to online", "robotId", robot.Spec.RobotID, "timeSinceHeartbeat", timeSinceLastHeartbeat)
			}
		} else {
			// 心跳超时 → Offline
			if robot.Status.Phase != robotv1alpha1.RobotPhaseOffline {
				robot.Status.Phase = robotv1alpha1.RobotPhaseOffline
				robot.Status.Message = "Heartbeat timeout"

				if err := r.Status().Update(ctx, robot); err != nil {
					logger.Error(err, "Failed to update Robot status to Offline")
					return ctrl.Result{}, err
				}
				logger.Info("Robot marked as offline due to heartbeat timeout", "robotId", robot.Spec.RobotID, "timeSinceHeartbeat", timeSinceLastHeartbeat)
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
	// 启动定期检查所有 Robot 的 goroutine
	go r.periodicHealthCheck(mgr.GetClient())

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

				// 心跳超时且状态不是 Offline
				if timeSinceLastHeartbeat > HeartbeatTimeout && robot.Status.Phase != robotv1alpha1.RobotPhaseOffline {
					logger.Info("Detected offline robot",
						"robot", robot.Name,
						"timeSinceHeartbeat", timeSinceLastHeartbeat,
						"currentPhase", robot.Status.Phase)

					robot.Status.Phase = robotv1alpha1.RobotPhaseOffline
					robot.Status.Message = "Heartbeat timeout"

					if err := client.Status().Update(ctx, robot); err != nil {
						logger.Error(err, "Failed to mark robot as offline", "robot", robot.Name)
					} else {
						logger.Info("Marked robot as offline", "robot", robot.Name)
					}
				}

				// 心跳正常且状态不是 Online
				if timeSinceLastHeartbeat <= HeartbeatTimeout && robot.Status.Phase != robotv1alpha1.RobotPhaseOnline {
					logger.Info("Detected online robot",
						"robot", robot.Name,
						"timeSinceHeartbeat", timeSinceLastHeartbeat,
						"currentPhase", robot.Status.Phase)

					robot.Status.Phase = robotv1alpha1.RobotPhaseOnline
					robot.Status.Message = "Robot is online"

					if err := client.Status().Update(ctx, robot); err != nil {
						logger.Error(err, "Failed to mark robot as online", "robot", robot.Name)
					} else {
						logger.Info("Marked robot as online", "robot", robot.Name)
					}
				}
			}
		}
	}
}
