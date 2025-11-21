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

// TaskGroupReconciler reconciles a TaskGroup object
type TaskGroupReconciler struct {
	client.Client
	Scheme *runtime.Scheme
}

//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=taskgroups,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=taskgroups/status,verbs=get;update;patch
//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=taskgroups/finalizers,verbs=update
//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=tasks,verbs=get;list;watch;create;update;patch;delete

// Reconcile 处理 TaskGroup 的生命周期
// TaskGroupController 负责：
// 1. 根据 TaskGroup 创建 Task
// 2. 协调和管理 TaskGroup 内的 Tasks
// 3. 更新 TaskGroup 状态
func (r *TaskGroupReconciler) Reconcile(ctx context.Context, req ctrl.Request) (ctrl.Result, error) {
	logger := log.FromContext(ctx)

	// 获取 TaskGroup
	taskGroup := &robotv1alpha1.TaskGroup{}
	err := r.Get(ctx, req.NamespacedName, taskGroup)
	if err != nil {
		if errors.IsNotFound(err) {
			return ctrl.Result{}, nil
		}
		logger.Error(err, "Failed to get TaskGroup")
		return ctrl.Result{}, err
	}

	logger.Info("Reconciling TaskGroup", "taskGroup", taskGroup.Name, "state", taskGroup.Status.State)

	// 如果正在删除，清理
	if taskGroup.DeletionTimestamp != nil {
		return r.handleTaskGroupDeletion(ctx, taskGroup)
	}

	// 初始化 TaskGroup 状态（如果是新创建的）
	if taskGroup.Status.State == "" {
		taskGroup.Status.State = robotv1alpha1.TaskGroupStatePending
		taskGroup.Status.StatusDescription = "TaskGroup created, waiting for tasks"
		if err := r.Status().Update(ctx, taskGroup); err != nil {
			logger.Error(err, "Failed to initialize taskGroup status")
			return ctrl.Result{}, err
		}
		logger.Info("TaskGroup initialized with pending status", "taskGroup", taskGroup.Name)

		// 创建 Tasks
		if err := r.ensureTasksForTaskGroup(ctx, taskGroup); err != nil {
			logger.Error(err, "Failed to create tasks for taskGroup")
			return ctrl.Result{}, err
		}

		return ctrl.Result{}, nil
	}

	// 如果 TaskGroup 已经是终止状态（completed/failed），只更新状态不做其他操作
	if taskGroup.Status.State == robotv1alpha1.TaskGroupStateCompleted ||
		taskGroup.Status.State == robotv1alpha1.TaskGroupStateFailed {
		logger.Info("TaskGroup already in terminal state", "taskGroup", taskGroup.Name, "state", taskGroup.Status.State)
		return ctrl.Result{}, r.updateTaskGroupStatus(ctx, taskGroup)
	}

	// 更新 TaskGroup 状态（基于 Tasks 的状态）
	if err := r.updateTaskGroupStatus(ctx, taskGroup); err != nil {
		logger.Error(err, "Failed to update taskGroup status")
		return ctrl.Result{}, err
	}

	// 如果 TaskGroup 还在运行中，定期检查
	if taskGroup.Status.State == robotv1alpha1.TaskGroupStateRunning ||
		taskGroup.Status.State == robotv1alpha1.TaskGroupStatePending {
		return ctrl.Result{RequeueAfter: 10 * time.Second}, nil
	}

	return ctrl.Result{}, nil
}

// ensureTasksForTaskGroup 确保 TaskGroup 的所有 Task 都已创建
func (r *TaskGroupReconciler) ensureTasksForTaskGroup(ctx context.Context, taskGroup *robotv1alpha1.TaskGroup) error {
	logger := log.FromContext(ctx)

	// 列出该 TaskGroup 下的所有 Task
	taskList := &robotv1alpha1.TaskList{}
	if err := r.List(ctx, taskList, client.InNamespace(taskGroup.Namespace), client.MatchingLabels{
		"taskgroup": taskGroup.Name,
	}); err != nil {
		return err
	}

	// 为 TaskGroup 中的每个 TaskDefinition 创建 Task
	for i, taskDef := range taskGroup.Spec.Tasks {
		taskName := taskGroup.Name + "-" + taskDef.Name

		// 检查是否已存在
		found := false
		for _, existingTask := range taskList.Items {
			if existingTask.Name == taskName {
				found = true
				break
			}
		}

		if found {
			continue
		}

		// 创建 Task CR
		task := &robotv1alpha1.Task{
			ObjectMeta: metav1.ObjectMeta{
				Name:      taskName,
				Namespace: taskGroup.Namespace,
				Labels: map[string]string{
					"job":       taskGroup.Spec.JobName,
					"taskgroup": taskGroup.Name,
					"taskindex": fmt.Sprintf("%d", i),
				},
				OwnerReferences: []metav1.OwnerReference{
					{
						APIVersion: taskGroup.APIVersion,
						Kind:       taskGroup.Kind,
						Name:       taskGroup.Name,
						UID:        taskGroup.UID,
						Controller: func() *bool { b := true; return &b }(),
					},
				},
			},
			Spec: robotv1alpha1.TaskSpec{
				Name:        taskDef.Name,
				JobName:     taskGroup.Spec.JobName, // 设置 JobName，TaskController 需要用它查找 Job
				Driver:      taskDef.Driver,
				Config:      taskDef.Config,
				Resources:   taskDef.Resources,
				Env:         taskDef.Env,
				User:        taskDef.User,
				KillTimeout: taskDef.KillTimeout,
				Artifacts:   taskDef.Artifacts,
				Templates:   taskDef.Templates,
			},
			Status: robotv1alpha1.TaskStatus{
				State: robotv1alpha1.TaskStatePending,
			},
		}

		if err := r.Create(ctx, task); err != nil {
			logger.Error(err, "Failed to create Task", "task", task.Name)
			return err
		}
		logger.Info("Created Task for TaskGroup", "taskGroup", taskGroup.Name, "task", task.Name, "taskDef", taskDef.Name)
	}

	return nil
}

// updateTaskGroupStatus 根据 Task 状态更新 TaskGroup 状态
func (r *TaskGroupReconciler) updateTaskGroupStatus(ctx context.Context, taskGroup *robotv1alpha1.TaskGroup) error {
	logger := log.FromContext(ctx)

	// 列出所有属于这个 TaskGroup 的 Task
	taskList := &robotv1alpha1.TaskList{}
	if err := r.List(ctx, taskList, client.InNamespace(taskGroup.Namespace), client.MatchingLabels{
		"taskgroup": taskGroup.Name,
	}); err != nil {
		logger.Error(err, "Failed to list tasks")
		return err
	}

	// 统计各状态的 Task 数量
	var totalTasks, pendingTasks, dispatchingTasks, runningTasks, succeededTasks, failedTasks int32
	totalTasks = int32(len(taskList.Items))

	for _, task := range taskList.Items {
		switch task.Status.State {
		case robotv1alpha1.TaskStatePending, "":
			pendingTasks++
		case robotv1alpha1.TaskStateDispatching:
			dispatchingTasks++
		case robotv1alpha1.TaskStateRunning:
			runningTasks++
		case robotv1alpha1.TaskStateCompleted:
			succeededTasks++
		case robotv1alpha1.TaskStateFailed:
			failedTasks++
		case robotv1alpha1.TaskStateExited:
			// exited 状态根据 exitCode 判断
			if task.Status.ExitCode != nil && *task.Status.ExitCode == 0 {
				succeededTasks++
			} else {
				failedTasks++
			}
		}
	}

	// 更新状态
	taskGroup.Status.TotalTasks = totalTasks
	taskGroup.Status.PendingTasks = pendingTasks
	taskGroup.Status.RunningTasks = runningTasks
	taskGroup.Status.SucceededTasks = succeededTasks
	taskGroup.Status.FailedTasks = failedTasks

	// 确定 TaskGroup 状态（级联状态）
	// 规则：只要有一个 Task 是 dispatching，TaskGroup 就是 running
	//      只要有一个 Task 是 running，TaskGroup 就是 running
	if totalTasks == 0 {
		taskGroup.Status.State = robotv1alpha1.TaskGroupStatePending
		taskGroup.Status.StatusDescription = "No tasks created yet"
	} else if succeededTasks == totalTasks {
		// 所有 Task 都成功
		taskGroup.Status.State = robotv1alpha1.TaskGroupStateCompleted
		taskGroup.Status.StatusDescription = fmt.Sprintf("All %d tasks completed successfully", totalTasks)
		if taskGroup.Status.CompletedAt == nil {
			now := metav1.Now()
			taskGroup.Status.CompletedAt = &now
		}
	} else if succeededTasks+failedTasks == totalTasks {
		// 所有 Task 都已结束（有失败）
		taskGroup.Status.State = robotv1alpha1.TaskGroupStateFailed
		taskGroup.Status.StatusDescription = fmt.Sprintf("%d succeeded, %d failed out of %d tasks",
			succeededTasks, failedTasks, totalTasks)
		if taskGroup.Status.CompletedAt == nil {
			now := metav1.Now()
			taskGroup.Status.CompletedAt = &now
		}
	} else if dispatchingTasks > 0 || runningTasks > 0 {
		// 只要有任何 Task 进入 dispatching 或 running，TaskGroup 就变为 running
		taskGroup.Status.State = robotv1alpha1.TaskGroupStateRunning
		taskGroup.Status.StatusDescription = fmt.Sprintf("Running: %d, Dispatching: %d, Succeeded: %d, Failed: %d, Pending: %d",
			runningTasks, dispatchingTasks, succeededTasks, failedTasks, pendingTasks)
		if taskGroup.Status.StartedAt == nil {
			now := metav1.Now()
			taskGroup.Status.StartedAt = &now
		}
	} else {
		// 所有 Task 都是 pending
		taskGroup.Status.State = robotv1alpha1.TaskGroupStatePending
		taskGroup.Status.StatusDescription = fmt.Sprintf("Waiting: %d pending tasks", pendingTasks)
	}

	logger.Info("Updated taskGroup status",
		"taskGroup", taskGroup.Name,
		"state", taskGroup.Status.State,
		"total", totalTasks,
		"succeeded", succeededTasks,
		"running", runningTasks,
		"dispatching", dispatchingTasks,
		"failed", failedTasks,
		"pending", pendingTasks)

	return r.Status().Update(ctx, taskGroup)
}

// handleTaskGroupDeletion 处理 TaskGroup 删除
func (r *TaskGroupReconciler) handleTaskGroupDeletion(ctx context.Context, taskGroup *robotv1alpha1.TaskGroup) (ctrl.Result, error) {
	logger := log.FromContext(ctx)
	logger.Info("TaskGroup is being deleted", "taskGroup", taskGroup.Name)

	// Task 会通过 OwnerReference 自动删除
	return ctrl.Result{}, nil
}

// SetupWithManager sets up the controller with the Manager.
func (r *TaskGroupReconciler) SetupWithManager(mgr ctrl.Manager) error {
	return ctrl.NewControllerManagedBy(mgr).
		For(&robotv1alpha1.TaskGroup{}).
		Owns(&robotv1alpha1.Task{}). // TaskGroup 拥有 Task
		Complete(r)
}
