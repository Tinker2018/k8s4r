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

	robotv1alpha1 "github.com/hxndghxndg/k8s4r/api/v1alpha1"
)

// JobReconciler reconciles a Job object
type JobReconciler struct {
	client.Client
	Scheme *runtime.Scheme
}

//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=jobs,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=jobs/status,verbs=get;update;patch
//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=jobs/finalizers,verbs=update
//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=taskgroups,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=tasks,verbs=get;list;watch

// Reconcile 处理 Job 的生命周期
// JobController 负责创建 Job 并创建 TaskGroups
// JobController 只负责创建 Job 和更新 Job 状态，不创建 Task
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

	logger.Info("Reconciling Job", "job", job.Name, "status", job.Status.State)

	// 如果正在删除，清理
	if job.DeletionTimestamp != nil {
		return r.handleJobDeletion(ctx, job)
	}

	// 初始化 Job 状态（如果是新创建的）
	if job.Status.State == "" {
		job.Status.State = robotv1alpha1.JobStatePending
		job.Status.StatusDescription = "Job created, waiting for task groups"
		if err := r.Status().Update(ctx, job); err != nil {
			logger.Error(err, "Failed to initialize job status")
			return ctrl.Result{}, err
		}
		logger.Info("Job initialized with pending status", "job", job.Name)

		// 创建 TaskGroups
		if err := r.ensureTaskGroupsForJob(ctx, job); err != nil {
			logger.Error(err, "Failed to create taskGroups for job")
			return ctrl.Result{}, err
		}

		return ctrl.Result{}, nil
	}

	// 如果 Job 已经是终止状态（complete/failed），只更新状态不做其他操作
	if job.Status.State == robotv1alpha1.JobStateCompleted || job.Status.State == robotv1alpha1.JobStateFailed {
		logger.Info("Job already in terminal state", "job", job.Name, "status", job.Status.State)
		// 仍然检查一次状态，防止状态不一致
		return ctrl.Result{}, r.updateJobStatus(ctx, job)
	}

	// 查询所有属于这个 Job 的 Task，更新 Job 状态
	if err := r.updateJobStatus(ctx, job); err != nil {
		logger.Error(err, "Failed to update job status")
		return ctrl.Result{}, err
	}

	// 如果 Job 还在运行中，定期检查
	if job.Status.State == robotv1alpha1.JobStateRunning || job.Status.State == robotv1alpha1.JobStatePending {
		return ctrl.Result{RequeueAfter: 10 * time.Second}, nil
	}

	return ctrl.Result{}, nil
}

// boolPtr 返回 bool 指针
func boolPtr(b bool) *bool {
	return &b
}

// updateJobStatus 根据 TaskGroup 和 Task 状态更新 Job 状态
func (r *JobReconciler) updateJobStatus(ctx context.Context, job *robotv1alpha1.Job) error {
	logger := log.FromContext(ctx)

	// 查询所有属于这个 Job 的 TaskGroup
	taskGroupList := &robotv1alpha1.TaskGroupList{}
	if err := r.List(ctx, taskGroupList, client.InNamespace(job.Namespace), client.MatchingLabels{
		"job": job.Name,
	}); err != nil {
		return fmt.Errorf("failed to list taskGroups: %w", err)
	}

	// 查询所有属于这个 Job 的 Task
	taskList := &robotv1alpha1.TaskList{}
	if err := r.List(ctx, taskList, client.InNamespace(job.Namespace), client.MatchingLabels{
		"job": job.Name,
	}); err != nil {
		return fmt.Errorf("failed to list tasks: %w", err)
	}

	if len(taskGroupList.Items) == 0 {
		// 还没有创建 TaskGroup
		job.Status.State = robotv1alpha1.JobStatePending
		job.Status.StatusDescription = "Creating task groups"
		job.Status.TotalTasks = 0
		job.Status.SucceededTasks = 0
		job.Status.FailedTasks = 0
		job.Status.RunningTasks = 0
		job.Status.PendingTasks = 0
		return r.Status().Update(ctx, job)
	}

	// 统计各状态的 Task 数量
	var (
		totalTasks       = int32(len(taskList.Items))
		pendingTasks     = int32(0)
		dispatchingTasks = int32(0)
		runningTasks     = int32(0)
		succeededTasks   = int32(0)
		failedTasks      = int32(0)
	)

	for _, task := range taskList.Items {
		switch task.Status.State {
		case robotv1alpha1.TaskStatePending, "":
			pendingTasks++
		case robotv1alpha1.TaskStateScheduled:
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

	// 更新统计字段
	job.Status.TotalTasks = totalTasks
	job.Status.SucceededTasks = succeededTasks
	job.Status.FailedTasks = failedTasks
	job.Status.RunningTasks = runningTasks
	job.Status.PendingTasks = pendingTasks

	// 决定 Job 的最终状态（级联状态）
	// 规则：只要有一个 Task 是 dispatching/running，Job 就是 running
	//      只有所有 Task 都是 pending，Job 才是 pending
	if succeededTasks == totalTasks && totalTasks > 0 {
		// 所有 Task 都成功完成
		job.Status.State = robotv1alpha1.JobStateCompleted
		job.Status.StatusDescription = fmt.Sprintf("All %d tasks completed successfully", totalTasks)
	} else if succeededTasks+failedTasks == totalTasks && totalTasks > 0 {
		// 所有 Task 都已结束（有失败）
		job.Status.State = robotv1alpha1.JobStateFailed
		job.Status.StatusDescription = fmt.Sprintf("%d succeeded, %d failed out of %d tasks",
			succeededTasks, failedTasks, totalTasks)
	} else if dispatchingTasks > 0 || runningTasks > 0 {
		// 只要有任何 Task 进入 dispatching 或 running，Job 就变为 running
		job.Status.State = robotv1alpha1.JobStateRunning
		job.Status.StatusDescription = fmt.Sprintf("Running: %d, Dispatching: %d, Succeeded: %d, Failed: %d, Pending: %d",
			runningTasks, dispatchingTasks, succeededTasks, failedTasks, pendingTasks)
	} else {
		// 所有 Task 都是 pending
		job.Status.State = robotv1alpha1.JobStatePending
		job.Status.StatusDescription = fmt.Sprintf("Waiting: %d pending tasks", pendingTasks)
	}

	logger.Info("Updated job status",
		"job", job.Name,
		"status", job.Status.State,
		"total", totalTasks,
		"succeeded", succeededTasks,
		"running", runningTasks,
		"dispatching", dispatchingTasks,
		"failed", failedTasks,
		"pending", pendingTasks)

	return r.Status().Update(ctx, job)
}

// ensureTaskGroupsForJob 确保 Job 的所有 TaskGroup 都已创建
func (r *JobReconciler) ensureTaskGroupsForJob(ctx context.Context, job *robotv1alpha1.Job) error {
	logger := log.FromContext(ctx)

	// 列出该 Job 下的所有 TaskGroup
	taskGroupList := &robotv1alpha1.TaskGroupList{}
	if err := r.List(ctx, taskGroupList, client.InNamespace(job.Namespace), client.MatchingLabels{
		"job": job.Name,
	}); err != nil {
		return err
	}

	// 为每个 TaskGroupTemplate 创建 TaskGroup
	for _, groupTemplate := range job.Spec.TaskGroups {
		taskGroupNamePrefix := job.Name + "-" + groupTemplate.Name + "-"

		// 检查是否已存在（通过 JobName 和 GroupName 匹配）
		found := false
		for _, existingGroup := range taskGroupList.Items {
			if existingGroup.Spec.JobName == job.Name && existingGroup.Spec.GroupName == groupTemplate.Name {
				found = true
				break
			}
		}

		if found {
			continue
		}

		// 创建 TaskGroup CR（使用 GenerateName 自动生成唯一名称）
		taskGroup := &robotv1alpha1.TaskGroup{
			ObjectMeta: metav1.ObjectMeta{
				GenerateName: taskGroupNamePrefix, // Kubernetes 会自动添加随机后缀
				Namespace:    job.Namespace,
				Labels: map[string]string{
					"job":       job.Name,
					"groupName": groupTemplate.Name,
				},
				OwnerReferences: []metav1.OwnerReference{
					{
						APIVersion: job.APIVersion,
						Kind:       job.Kind,
						Name:       job.Name,
						UID:        job.UID,
						Controller: func() *bool { b := true; return &b }(),
					},
				},
			},
			Spec: robotv1alpha1.TaskGroupSpec{
				JobName:          job.Name,
				GroupName:        groupTemplate.Name,
				Count:            groupTemplate.Count,
				Tasks:            groupTemplate.Tasks,
				Constraints:      groupTemplate.Constraints,
				RestartPolicy:    groupTemplate.RestartPolicy,
				ReschedulePolicy: groupTemplate.ReschedulePolicy,
				EphemeralDisk:    groupTemplate.EphemeralDisk,
				Meta:             groupTemplate.Meta,
			},
			Status: robotv1alpha1.TaskGroupStatus{
				State: robotv1alpha1.TaskGroupStatePending,
			},
		}

		if err := r.Create(ctx, taskGroup); err != nil {
			logger.Error(err, "Failed to create TaskGroup", "taskGroup", taskGroup.Name)
			return err
		}
		logger.Info("Created TaskGroup for Job", "job", job.Name, "taskGroup", taskGroup.Name, "groupName", groupTemplate.Name)
	}

	return nil
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
		Owns(&robotv1alpha1.TaskGroup{}). // Job 拥有 TaskGroup
		Complete(r)
}
