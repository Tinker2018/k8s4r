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

// JobReconciler reconciles a Job object
type JobReconciler struct {
	client.Client
	Scheme *runtime.Scheme
}

//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=jobs,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=jobs/status,verbs=get;update;patch
//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=jobs/finalizers,verbs=update
//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=tasks,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=robots,verbs=get;list;watch

// Reconcile 处理 Job 的生命周期
func (r *JobReconciler) Reconcile(ctx context.Context, req ctrl.Request) (ctrl.Result, error) {
	logger := log.FromContext(ctx)

	// 获取 Job
	job := &robotv1alpha1.Job{}
	err := r.Get(ctx, req.NamespacedName, job)
	if err != nil {
		if errors.IsNotFound(err) {
			return ctrl.Result{}, nil
		}
		logger.Error(err, "Failed to get Job")
		return ctrl.Result{}, err
	}

	logger.Info("Reconciling Job", "job", job.Name)

	// 如果正在删除，清理 Task
	if job.DeletionTimestamp != nil {
		return r.handleJobDeletion(ctx, job)
	}

	// 如果 Job 已经是终止状态（complete/failed），只更新状态不重新创建 Task
	if job.Status.Status == "complete" || job.Status.Status == "failed" {
		logger.Info("Job already in terminal state", "job", job.Name, "status", job.Status.Status)
		// 仍然检查一次状态，防止状态不一致
		return ctrl.Result{}, r.updateJobStatus(ctx, job)
	}

	// 根据 RobotSelector 选择匹配的 Robot
	matchedRobots, err := r.selectRobotsByLabels(ctx, job.Spec.RobotSelector)
	if err != nil {
		logger.Error(err, "Failed to select robots")
		return ctrl.Result{}, err
	}

	if len(matchedRobots) == 0 {
		logger.Info("No robots match the selector", "selector", job.Spec.RobotSelector)
		job.Status.Status = "pending"
		job.Status.StatusDescription = "No matching robots found"
		r.Status().Update(ctx, job)
		return ctrl.Result{RequeueAfter: 30}, nil
	}

	logger.Info("Found matching robots",
		"count", len(matchedRobots),
		"selector", job.Spec.RobotSelector)

	// 为每个 TaskGroup 创建 Task
	for _, taskGroup := range job.Spec.TaskGroups {
		if err := r.createTasksForGroup(ctx, job, &taskGroup, matchedRobots); err != nil {
			logger.Error(err, "Failed to create tasks for group", "group", taskGroup.Name)
			return ctrl.Result{}, err
		}
	}

	// 查询所有属于这个 Job 的 Task，更新 Job 状态
	if err := r.updateJobStatus(ctx, job); err != nil {
		logger.Error(err, "Failed to update job status")
		return ctrl.Result{}, err
	}

	// 如果 Job 还在运行中，定期检查
	if job.Status.Status == "running" {
		return ctrl.Result{RequeueAfter: 10 * time.Second}, nil
	}

	return ctrl.Result{}, nil
}

// selectRobotsByLabels 根据 label selector 选择 Robot
// 类似 Kubernetes nodeSelector 的实现
func (r *JobReconciler) selectRobotsByLabels(ctx context.Context, selector map[string]string) ([]*robotv1alpha1.Robot, error) {
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
// selector 中的所有 label 都必须在 robot labels 中存在且值相同
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

// createTasksForGroup 为 TaskGroup 创建 Task 实例
func (r *JobReconciler) createTasksForGroup(
	ctx context.Context,
	job *robotv1alpha1.Job,
	taskGroup *robotv1alpha1.TaskGroup,
	matchedRobots []*robotv1alpha1.Robot,
) error {
	logger := log.FromContext(ctx)

	// 根据 count 创建多个 Task 实例
	for i := int32(0); i < taskGroup.Count; i++ {
		// 选择一个 Robot（简单轮询）
		robot := matchedRobots[int(i)%len(matchedRobots)]

		// 为 TaskGroup 中的每个 TaskDefinition 创建 Task
		for _, taskDef := range taskGroup.Tasks {
			taskName := fmt.Sprintf("%s-%s-%s-%d",
				job.Name,
				taskGroup.Name,
				taskDef.Name,
				i)

			// 检查 Task 是否已存在
			existingTask := &robotv1alpha1.Task{}
			err := r.Get(ctx, client.ObjectKey{
				Name:      taskName,
				Namespace: job.Namespace,
			}, existingTask)

			if err == nil {
				// Task 已存在，跳过
				logger.V(1).Info("Task already exists", "task", taskName)
				continue
			}

			// 创建新的 Task
			task := &robotv1alpha1.Task{
				ObjectMeta: metav1.ObjectMeta{
					Name:      taskName,
					Namespace: job.Namespace,
					Labels: map[string]string{
						"job":       job.Name,
						"taskGroup": taskGroup.Name,
					},
					OwnerReferences: []metav1.OwnerReference{
						{
							APIVersion: job.APIVersion,
							Kind:       job.Kind,
							Name:       job.Name,
							UID:        job.UID,
						},
					},
				},
				Spec: robotv1alpha1.TaskSpec{
					JobName:       job.Name,
					TaskGroupName: taskGroup.Name,
					Name:          taskDef.Name,
					Driver:        taskDef.Driver,
					Config:        taskDef.Config,
					TargetRobot:   robot.Name, // 分配给匹配的 Robot
					Resources:     taskDef.Resources,
					Constraints:   nil, // TODO: 从 taskGroup 继承
					Env:           taskDef.Env,
					User:          taskDef.User,
					KillTimeout:   taskDef.KillTimeout,
					Artifacts:     taskDef.Artifacts, // 包含下载地址
					Templates:     taskDef.Templates,
				},
			}

			if err := r.Create(ctx, task); err != nil {
				logger.Error(err, "Failed to create task", "task", taskName)
				return err
			}

			logger.Info("Task created",
				"task", taskName,
				"robot", robot.Name,
				"artifacts", len(taskDef.Artifacts))
		}
	}

	return nil
}

// updateJobStatus 根据 Task 状态更新 Job 状态
// 这个函数会在以下情况被调用：
// 1. JobReconciler 创建 Task 后
// 2. 任何 Task 状态变化时（通过 Owns() 机制自动触发 Job 的 Reconcile）
func (r *JobReconciler) updateJobStatus(ctx context.Context, job *robotv1alpha1.Job) error {
	logger := log.FromContext(ctx)

	// 查询所有属于这个 Job 的 Task
	taskList := &robotv1alpha1.TaskList{}
	if err := r.List(ctx, taskList, client.InNamespace(job.Namespace), client.MatchingLabels{
		"job": job.Name,
	}); err != nil {
		return fmt.Errorf("failed to list tasks: %w", err)
	}

	if len(taskList.Items) == 0 {
		// 还没有创建 Task
		job.Status.Status = "pending"
		job.Status.StatusDescription = "Creating tasks"
		return r.Status().Update(ctx, job)
	}

	// 统计各状态的 Task 数量
	var (
		totalTasks     = len(taskList.Items)
		pendingTasks   = 0
		runningTasks   = 0
		completedTasks = 0
		failedTasks    = 0
		exitedTasks    = 0
	)

	for _, task := range taskList.Items {
		switch task.Status.State {
		case robotv1alpha1.TaskStatePending, "":
			pendingTasks++
		case robotv1alpha1.TaskStateDispatching:
			pendingTasks++
		case robotv1alpha1.TaskStateRunning:
			runningTasks++
		case robotv1alpha1.TaskStateCompleted:
			completedTasks++
		case robotv1alpha1.TaskStateFailed:
			failedTasks++
		case robotv1alpha1.TaskStateExited:
			exitedTasks++
		}
	}

	// 决定 Job 的最终状态
	if completedTasks == totalTasks {
		// 所有 Task 都完成
		job.Status.Status = "complete"
		job.Status.StatusDescription = fmt.Sprintf("All %d tasks completed", totalTasks)
	} else if failedTasks > 0 {
		// 有失败的 Task
		job.Status.Status = "failed"
		job.Status.StatusDescription = fmt.Sprintf("%d/%d tasks failed", failedTasks, totalTasks)
	} else if runningTasks > 0 || exitedTasks > 0 {
		// 有正在运行的 Task
		job.Status.Status = "running"
		job.Status.StatusDescription = fmt.Sprintf("Running: %d, Completed: %d/%d",
			runningTasks, completedTasks, totalTasks)
	} else {
		// 所有 Task 都在等待
		job.Status.Status = "pending"
		job.Status.StatusDescription = fmt.Sprintf("Pending: %d/%d", pendingTasks, totalTasks)
	}

	logger.Info("Updated job status",
		"job", job.Name,
		"status", job.Status.Status,
		"total", totalTasks,
		"completed", completedTasks,
		"running", runningTasks,
		"failed", failedTasks,
		"pending", pendingTasks)

	return r.Status().Update(ctx, job)
}

// handleJobDeletion 处理 Job 删除
func (r *JobReconciler) handleJobDeletion(ctx context.Context, job *robotv1alpha1.Job) (ctrl.Result, error) {
	logger := log.FromContext(ctx)
	logger.Info("Job is being deleted", "job", job.Name)

	// Task 会通过 OwnerReference 自动删除
	return ctrl.Result{}, nil
}

// SetupWithManager sets up the controller with the Manager.
func (r *JobReconciler) SetupWithManager(mgr ctrl.Manager) error {
	return ctrl.NewControllerManagedBy(mgr).
		For(&robotv1alpha1.Job{}).
		Owns(&robotv1alpha1.Task{}). // 监听自己创建的 Task
		Complete(r)
}
