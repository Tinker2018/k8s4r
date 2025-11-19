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
	"fmt"
	"time"

	"k8s.io/apimachinery/pkg/api/errors"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/runtime"
	ctrl "sigs.k8s.io/controller-runtime"
	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/log"

	robotv1alpha1 "github.com/hxndg/k8s4r/api/v1alpha1"
)

// TaskReconciler reconciles a Task object
// 负责所有调度、分发、管理逻辑
type TaskReconciler struct {
	client.Client
	Scheme *runtime.Scheme
}

//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=tasks,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=tasks/status,verbs=get;update;patch
//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=tasks/finalizers,verbs=update
//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=robots,verbs=get;list;watch

// Reconcile 实现 Task 的业务逻辑
func (r *TaskReconciler) Reconcile(ctx context.Context, req ctrl.Request) (ctrl.Result, error) {
	logger := log.FromContext(ctx)

	// 获取 Task
	task := &robotv1alpha1.Task{}
	err := r.Get(ctx, req.NamespacedName, task)
	if err != nil {
		if errors.IsNotFound(err) {
			return ctrl.Result{}, nil
		}
		logger.Error(err, "Failed to get Task")
		return ctrl.Result{}, err
	}

	logger.Info("Reconciling Task", "task", task.Name, "state", task.Status.State)

	// 如果正在删除，不处理
	if task.DeletionTimestamp != nil {
		return ctrl.Result{}, nil
	}

	// 状态机处理
	switch task.Status.State {
	case "", robotv1alpha1.TaskStatePending:
		// 新创建的 Task，需要调度
		return r.scheduleTask(ctx, task)

	case robotv1alpha1.TaskStateDispatching:
		// 正在分发中，等待 Agent 响应
		return r.monitorDispatching(ctx, task)

	case robotv1alpha1.TaskStateRunning:
		// 运行中，监控超时
		return r.monitorRunning(ctx, task)

	case robotv1alpha1.TaskStateCompleted, robotv1alpha1.TaskStateFailed:
		// 已完成，无需处理
		return ctrl.Result{}, nil

	case robotv1alpha1.TaskStateExited:
		// Agent 返回的 exited 状态，根据 exitCode 决定最终状态
		logger.Info("Task exited, checking exit code", "task", task.Name, "exitCode", task.Status.ExitCode)

		finalState := robotv1alpha1.TaskStateCompleted
		if task.Status.ExitCode != nil && *task.Status.ExitCode != 0 {
			finalState = robotv1alpha1.TaskStateFailed
			logger.Info("Task exited with non-zero code, marking as failed",
				"task", task.Name,
				"exitCode", *task.Status.ExitCode)
		}

		task.Status.State = finalState
		if err := r.Status().Update(ctx, task); err != nil {
			logger.Error(err, "Failed to update task state")
			return ctrl.Result{}, err
		}
		return ctrl.Result{}, nil

	default:
		logger.Info("Unknown task state", "state", task.Status.State)
		return ctrl.Result{}, nil
	}
}

// scheduleTask 调度 Task 到合适的 Robot
// 这里包含所有调度逻辑：选择 Robot、检查资源、约束匹配等
func (r *TaskReconciler) scheduleTask(ctx context.Context, task *robotv1alpha1.Task) (ctrl.Result, error) {
	logger := log.FromContext(ctx)

	logger.Info("Scheduling task", "task", task.Name)

	// 如果已经指定了 TargetRobot，跳过调度
	if task.Spec.TargetRobot != "" {
		logger.Info("Task already has target robot", "robot", task.Spec.TargetRobot)
		return r.dispatchTask(ctx, task)
	}

	// 获取所有 Online 的 Robot
	robotList := &robotv1alpha1.RobotList{}
	if err := r.List(ctx, robotList); err != nil {
		logger.Error(err, "Failed to list robots")
		return ctrl.Result{}, err
	}

	var selectedRobot *robotv1alpha1.Robot
	for i := range robotList.Items {
		robot := &robotList.Items[i]

		// 只考虑 Online 的 Robot
		if robot.Status.Phase != robotv1alpha1.RobotPhaseOnline {
			continue
		}

		// TODO: 检查约束条件（Constraints）
		// TODO: 检查资源可用性
		// TODO: 负载均衡

		// 简单实现：选择第一个 Online 的 Robot
		selectedRobot = robot
		break
	}

	if selectedRobot == nil {
		logger.Info("No available robot found, requeueing")
		task.Status.State = robotv1alpha1.TaskStatePending
		task.Status.Message = "Waiting for available robot"
		if err := r.Status().Update(ctx, task); err != nil {
			logger.Error(err, "Failed to update task status")
		}
		return ctrl.Result{RequeueAfter: 10 * time.Second}, nil
	}

	// 设置目标 Robot
	task.Spec.TargetRobot = selectedRobot.Name
	if err := r.Update(ctx, task); err != nil {
		logger.Error(err, "Failed to update task spec")
		return ctrl.Result{}, err
	}

	logger.Info("Robot selected for task",
		"task", task.Name,
		"robot", selectedRobot.Name)

	// 分发任务
	return r.dispatchTask(ctx, task)
}

// dispatchTask 将任务状态设置为 dispatching，触发 Server 转发
func (r *TaskReconciler) dispatchTask(ctx context.Context, task *robotv1alpha1.Task) (ctrl.Result, error) {
	logger := log.FromContext(ctx)

	logger.Info("Dispatching task to robot",
		"task", task.Name,
		"robot", task.Spec.TargetRobot)

	// 更新状态为 dispatching，Server 会监听到这个变化并转发到 MQTT
	task.Status.State = robotv1alpha1.TaskStateDispatching
	task.Status.Message = fmt.Sprintf("Dispatching to robot %s", task.Spec.TargetRobot)

	if err := r.Status().Update(ctx, task); err != nil {
		logger.Error(err, "Failed to update task status to dispatching")
		return ctrl.Result{}, err
	}

	logger.Info("Task status updated to dispatching", "task", task.Name)

	// 10 秒后检查是否成功分发
	return ctrl.Result{RequeueAfter: 10 * time.Second}, nil
}

// monitorDispatching 监控分发中的任务
func (r *TaskReconciler) monitorDispatching(ctx context.Context, task *robotv1alpha1.Task) (ctrl.Result, error) {
	logger := log.FromContext(ctx)

	// 检查 StartedAt 是否为空
	if task.Status.StartedAt == nil {
		logger.Info("Task StartedAt is nil, initializing", "task", task.Name)
		now := metav1.Now()
		task.Status.StartedAt = &now
		if err := r.Status().Update(ctx, task); err != nil {
			logger.Error(err, "Failed to initialize StartedAt")
			return ctrl.Result{}, err
		}
		return ctrl.Result{RequeueAfter: 5 * time.Second}, nil
	}

	// 检查是否超时（60秒）
	dispatchTimeout := 60 * time.Second
	if time.Since(task.Status.StartedAt.Time) > dispatchTimeout {
		logger.Info("Task dispatch timeout", "task", task.Name)

		task.Status.State = robotv1alpha1.TaskStateFailed
		task.Status.Message = "Dispatch timeout"
		if err := r.Status().Update(ctx, task); err != nil {
			logger.Error(err, "Failed to update task status")
		}

		// TODO: 重新调度或标记失败
		return ctrl.Result{}, nil
	}

	// 继续等待 Agent 响应
	return ctrl.Result{RequeueAfter: 5 * time.Second}, nil
}

// monitorRunning 监控运行中的任务
func (r *TaskReconciler) monitorRunning(ctx context.Context, task *robotv1alpha1.Task) (ctrl.Result, error) {
	logger := log.FromContext(ctx)

	// TODO: 检查执行超时
	// TODO: 检查 Robot 是否离线
	// TODO: 资源使用监控

	logger.V(1).Info("Monitoring running task", "task", task.Name)

	// 每 30 秒检查一次
	return ctrl.Result{RequeueAfter: 30 * time.Second}, nil
}

// SetupWithManager sets up the controller with the Manager.
func (r *TaskReconciler) SetupWithManager(mgr ctrl.Manager) error {
	return ctrl.NewControllerManagedBy(mgr).
		For(&robotv1alpha1.Task{}).
		Complete(r)
}
