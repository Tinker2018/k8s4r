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
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/runtime"
	ctrl "sigs.k8s.io/controller-runtime"
	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/log"

	robotv1alpha1 "github.com/hxndg/k8s4r/api/v1alpha1"
)

// TaskReconciler reconciles a Task object
// 负责调度 Task 到 Robot 并监控状态
type TaskReconciler struct {
	client.Client
	Scheme *runtime.Scheme
}

//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=tasks,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=tasks/status,verbs=get;update;patch
//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=tasks/finalizers,verbs=update
//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=robots,verbs=get;list;watch

// Reconcile 实现 Task 的业务逻辑
// TaskController 负责：
// 1. 调度 Task（为 Task 选择并分配 Robot）
// 2. 监控 Task 状态（pending → dispatching → running → completed/failed）
// 注意：Task 由 TaskGroupController 创建，不由 TaskController 创建
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
		// Task 刚创建或等待分发
		// 检查是否已经分配了 TargetRobot
		if task.Spec.TargetRobot == "" {
			// 需要调度
			return r.scheduleTask(ctx, task)
		}
		// 已分配 Robot，等待 Server 分发到 MQTT
		logger.V(1).Info("Task waiting for dispatch", "task", task.Name, "robot", task.Spec.TargetRobot)
		return ctrl.Result{RequeueAfter: 5 * time.Second}, nil

	case robotv1alpha1.TaskStateDispatching:
		// Server 已通过 MQTT 发送，等待 Agent 响应
		return r.monitorDispatching(ctx, task)

	case robotv1alpha1.TaskStateRunning:
		// Agent 正在执行，监控超时
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
func (r *TaskReconciler) scheduleTask(ctx context.Context, task *robotv1alpha1.Task) (ctrl.Result, error) {
	logger := log.FromContext(ctx)

	logger.Info("Scheduling task", "task", task.Name)

	// 获取 Task 所属的 Job
	job := &robotv1alpha1.Job{}
	err := r.Get(ctx, client.ObjectKey{
		Name:      task.Spec.JobName,
		Namespace: task.Namespace,
	}, job)
	if err != nil {
		logger.Error(err, "Failed to get Job for task", "job", task.Spec.JobName)
		return ctrl.Result{}, err
	}

	// 根据 Job 的 RobotSelector 选择匹配的 Robot
	matchedRobots, err := r.selectRobotsByLabels(ctx, job.Spec.RobotSelector)
	if err != nil {
		logger.Error(err, "Failed to select robots")
		return ctrl.Result{}, err
	}

	if len(matchedRobots) == 0 {
		logger.Info("No available robot found, requeueing")
		task.Status.State = robotv1alpha1.TaskStatePending
		task.Status.Message = "Waiting for available robot"
		if err := r.Status().Update(ctx, task); err != nil {
			logger.Error(err, "Failed to update task status")
		}
		return ctrl.Result{RequeueAfter: 10 * time.Second}, nil
	}

	// 简单调度：选择第一个匹配的 Robot
	// TODO: 实现更复杂的调度策略（负载均衡、资源检查等）
	selectedRobot := matchedRobots[0]

	// 更新 Task 的 TargetRobot
	task.Spec.TargetRobot = selectedRobot.Name
	if err := r.Update(ctx, task); err != nil {
		logger.Error(err, "Failed to update task spec")
		return ctrl.Result{}, err
	}

	logger.Info("Robot selected for task",
		"task", task.Name,
		"robot", selectedRobot.Name)

	// 状态保持 pending，等待 Server 分发
	return ctrl.Result{}, nil
}

// selectRobotsByLabels 根据 label selector 选择 Robot
func (r *TaskReconciler) selectRobotsByLabels(ctx context.Context, selector map[string]string) ([]*robotv1alpha1.Robot, error) {
	logger := log.FromContext(ctx)

	// 获取所有 Robot
	robotList := &robotv1alpha1.RobotList{}
	if err := r.List(ctx, robotList); err != nil {
		return nil, err
	}

	var matchedRobots []*robotv1alpha1.Robot

	// 遍历所有 Robot，检查 label 是否匹配
	for i := range robotList.Items {
		robot := &robotList.Items[i]

		// 只考虑 Online 的 Robot
		if robot.Status.Phase != robotv1alpha1.RobotPhaseOnline {
			continue
		}

		// 检查所有 selector label 是否匹配
		if matchesLabels(robot.Spec.Labels, selector) {
			matchedRobots = append(matchedRobots, robot)
			logger.Info("Robot matched selector",
				"robot", robot.Name,
				"robotLabels", robot.Spec.Labels,
				"selector", selector)
		}
	}

	return matchedRobots, nil
}

// matchesLabels 检查 robot labels 是否匹配 selector
func matchesLabels(robotLabels, selector map[string]string) bool {
	if len(selector) == 0 {
		// 空 selector 匹配所有 robot
		return true
	}

	for key, value := range selector {
		robotValue, exists := robotLabels[key]
		if !exists || robotValue != value {
			return false
		}
	}

	return true
}

// monitorDispatching 监控分发中的任务
// Server 已通过 MQTT 发送，等待 Agent 响应并上报状态
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

	// 记录等待时间（用于调试）
	waitTime := time.Since(task.Status.StartedAt.Time)
	logger.V(1).Info("Waiting for agent response",
		"task", task.Name,
		"waitTime", waitTime.String())

	// 继续等待 Agent 通过 MQTT 上报状态
	// 状态上报后，Server 会将 Task.Status.State 更新为 running
	return ctrl.Result{RequeueAfter: 10 * time.Second}, nil
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
