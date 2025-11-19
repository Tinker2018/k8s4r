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
	// HeartbeatTimeout 定义心跳超时时间（30秒）
	HeartbeatTimeout = 30 * time.Second
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

		// 如果心跳间隔小于 30 秒，认为在线
		if timeSinceLastHeartbeat <= HeartbeatTimeout {
			// Pending 状态且心跳正常 → Online
			if robot.Status.Phase == robotv1alpha1.RobotPhasePending {
				robot.Status.Phase = robotv1alpha1.RobotPhaseOnline
				robot.Status.Message = "Robot is online"

				if err := r.Status().Update(ctx, robot); err != nil {
					logger.Error(err, "Failed to update Robot status to Online")
					return ctrl.Result{}, err
				}
				logger.Info("Robot approved and online", "robotId", robot.Spec.RobotID)
			}
		} else {
			// 心跳间隔大于 30 秒 → Offline
			if robot.Status.Phase == robotv1alpha1.RobotPhaseOnline {
				robot.Status.Phase = robotv1alpha1.RobotPhaseOffline
				robot.Status.Message = "Heartbeat timeout"

				if err := r.Status().Update(ctx, robot); err != nil {
					logger.Error(err, "Failed to update Robot status to Offline")
					return ctrl.Result{}, err
				}
				logger.Info("Robot marked as offline due to heartbeat timeout", "robotId", robot.Spec.RobotID, "timeSinceHeartbeat", timeSinceLastHeartbeat)
			}
		}
	}

	// 定期检查心跳（每30秒）
	return ctrl.Result{RequeueAfter: 30 * time.Second}, nil
}

// SetupWithManager sets up the controller with the Manager.
func (r *RobotReconciler) SetupWithManager(mgr ctrl.Manager) error {
	return ctrl.NewControllerManagedBy(mgr).
		For(&robotv1alpha1.Robot{}).
		Complete(r)
}
