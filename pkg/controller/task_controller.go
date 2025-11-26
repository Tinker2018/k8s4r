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

	robotv1alpha1 "github.com/hxndghxndg/k8s4r/api/v1alpha1"
)

// TaskReconciler reconciles Task objects
// 职责：
// 1. 推送Task到gRPC Stream（Server → MQTT → Agent）
// 2. 监控Task状态变化
// 注意：Task由TaskGroup Controller创建，不由Task Controller创建
// ⚠️ 已废弃：现在直接使用 TaskGroup，不再创建单独的 Task CR
type TaskReconciler struct {
	client.Client
	Scheme            *runtime.Scheme
	TaskStreamManager interface{} // 已废弃，保留字段用于兼容性
}

//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=tasks,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=tasks/status,verbs=get;update;patch
//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=tasks/finalizers,verbs=update

// Reconcile 处理Task的生命周期
// Task状态流转：
// Pending → Scheduled(推送到MQTT) → Running(Agent执行) → Exited → Completed/Failed
func (r *TaskReconciler) Reconcile(ctx context.Context, req ctrl.Request) (ctrl.Result, error) {
	// 获取Task
	task := &robotv1alpha1.Task{}
	if err := r.Get(ctx, req.NamespacedName, task); err != nil {
		if errors.IsNotFound(err) {
			return ctrl.Result{}, nil
		}
		return ctrl.Result{}, err
	}

	// 处理Task生命周期
	return r.reconcileTask(ctx, task)
}

// reconcileTaskGroup 处理TaskGroup的Scheduled状态，创建Task实例
func (r *TaskReconciler) reconcileTask(ctx context.Context, task *robotv1alpha1.Task) (ctrl.Result, error) {
	logger := log.FromContext(ctx)

	logger.Info(" [TASK] Reconciling", "task", task.Name, "state", task.Status.State)

	// 处理删除
	if task.DeletionTimestamp != nil {
		return ctrl.Result{}, nil
	}

	// 状态机
	switch task.Status.State {
	case "", robotv1alpha1.TaskStatePending:
		// Task刚创建，推送到gRPC Stream
		return r.pushTaskToStream(ctx, task)

	case robotv1alpha1.TaskStateScheduled:
		// Server已发布到MQTT，等待Agent执行
		return r.monitorScheduled(ctx, task)

	case robotv1alpha1.TaskStateRunning:
		// Agent正在执行
		return r.monitorRunning(ctx, task)

	case robotv1alpha1.TaskStateExited:
		// 处理退出状态
		return r.handleExited(ctx, task)

	case robotv1alpha1.TaskStateCompleted, robotv1alpha1.TaskStateFailed:
		// 终止状态
		return ctrl.Result{}, nil

	default:
		logger.Info("Unknown state", "state", task.Status.State)
		return ctrl.Result{}, nil
	}
}

// pushTaskToStream 将Task推送到gRPC Stream
func (r *TaskReconciler) pushTaskToStream(ctx context.Context, task *robotv1alpha1.Task) (ctrl.Result, error) {
	logger := log.FromContext(ctx)

	if task.Spec.TargetRobot == "" {
		logger.Error(nil, " Task has no TargetRobot assigned", "task", task.Name)
		task.Status.State = robotv1alpha1.TaskStateFailed
		task.Status.Message = "No robot assigned"
		r.Status().Update(ctx, task)
		return ctrl.Result{}, nil
	}

	// ⚠️ TaskStreamManager 已废弃，现在使用 TaskGroup
	// Task CR 不再通过 gRPC Stream 推送
	logger.Info("⚠️ Task reconciliation is deprecated, use TaskGroup instead", "task", task.Name)

	return ctrl.Result{RequeueAfter: 10 * time.Second}, nil
}

// monitorScheduled 监控已调度的Task
func (r *TaskReconciler) monitorScheduled(ctx context.Context, task *robotv1alpha1.Task) (ctrl.Result, error) {
	logger := log.FromContext(ctx)
	logger.V(1).Info("Monitoring scheduled task", "task", task.Name)
	return ctrl.Result{RequeueAfter: 10 * time.Second}, nil
}

// monitorRunning 监控运行中的Task
func (r *TaskReconciler) monitorRunning(ctx context.Context, task *robotv1alpha1.Task) (ctrl.Result, error) {
	logger := log.FromContext(ctx)
	logger.V(1).Info("Monitoring running task", "task", task.Name)
	return ctrl.Result{RequeueAfter: 30 * time.Second}, nil
}

// handleExited 处理退出状态
func (r *TaskReconciler) handleExited(ctx context.Context, task *robotv1alpha1.Task) (ctrl.Result, error) {
	logger := log.FromContext(ctx)

	finalState := robotv1alpha1.TaskStateCompleted
	if task.Status.ExitCode != nil && *task.Status.ExitCode != 0 {
		finalState = robotv1alpha1.TaskStateFailed
		logger.Info("Task exited with error", "task", task.Name, "exitCode", *task.Status.ExitCode)
	}

	task.Status.State = finalState
	if err := r.Status().Update(ctx, task); err != nil {
		logger.Error(err, "Failed to update task state")
		return ctrl.Result{}, err
	}

	logger.Info(" Task completed", "task", task.Name, "state", finalState)
	return ctrl.Result{}, nil
}

// SetupWithManager sets up the controller with the Manager.
func (r *TaskReconciler) SetupWithManager(mgr ctrl.Manager) error {
	return ctrl.NewControllerManagedBy(mgr).
		For(&robotv1alpha1.Task{}).
		Complete(r)
}
