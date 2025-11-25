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

// TaskGroupReconciler reconciles a TaskGroup object
type TaskGroupReconciler struct {
	client.Client
	Scheme *runtime.Scheme
}

//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=taskgroups,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=taskgroups/status,verbs=get;update;patch
//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=taskgroups/finalizers,verbs=update
//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=tasks,verbs=get;list;watch;create;update;patch;delete
//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=robots,verbs=get;list;watch
//+kubebuilder:rbac:groups=robot.k8s4r.io,resources=jobs,verbs=get;list;watch

// Reconcile 处理 TaskGroup 的调度和状态更新
// TaskGroupController 职责：
// 1. 根据 RobotSelector 和 Count 进行调度决策
// 2. 设置 AssignedRobots（副本到Robot的映射）
// 3. 更新状态为 Scheduled
// 4. 监控 Task 状态更新 TaskGroup 状态
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

	logger.Info("📋 [TASKGROUP] Reconciling", "name", taskGroup.Name, "state", taskGroup.Status.State)

	// 处理删除
	if taskGroup.DeletionTimestamp != nil {
		return ctrl.Result{}, nil
	}

	// 状态机
	switch taskGroup.Status.State {
	case "": // 新创建的TaskGroup
		return r.initializeTaskGroup(ctx, taskGroup)

	case robotv1alpha1.TaskGroupStatePending:
		return r.scheduleTaskGroup(ctx, taskGroup)

	case robotv1alpha1.TaskGroupStateScheduled, robotv1alpha1.TaskGroupStateRunning:
		return r.monitorTaskGroup(ctx, taskGroup)

	case robotv1alpha1.TaskGroupStateCompleted, robotv1alpha1.TaskGroupStateFailed:
		// 终止状态，不再处理
		return ctrl.Result{}, nil

	default:
		logger.Info("Unknown state", "state", taskGroup.Status.State)
		return ctrl.Result{}, nil
	}
}

// initializeTaskGroup 初始化TaskGroup状态
func (r *TaskGroupReconciler) initializeTaskGroup(ctx context.Context, taskGroup *robotv1alpha1.TaskGroup) (ctrl.Result, error) {
	logger := log.FromContext(ctx)

	taskGroup.Status.State = robotv1alpha1.TaskGroupStatePending
	taskGroup.Status.StatusDescription = "Waiting for scheduling"

	if err := r.Status().Update(ctx, taskGroup); err != nil {
		logger.Error(err, " Failed to initialize TaskGroup status")
		return ctrl.Result{}, err
	}

	logger.Info(" [TASKGROUP] Initialized", "name", taskGroup.Name)
	return ctrl.Result{Requeue: true}, nil
}

// scheduleTaskGroup 执行调度决策
// 选择匹配的Robots，设置AssignedRobots，更新状态为Scheduled
func (r *TaskGroupReconciler) scheduleTaskGroup(ctx context.Context, taskGroup *robotv1alpha1.TaskGroup) (ctrl.Result, error) {
	logger := log.FromContext(ctx)

	// 获取父Job
	job := &robotv1alpha1.Job{}
	if err := r.Get(ctx, client.ObjectKey{
		Name:      taskGroup.Spec.JobName,
		Namespace: taskGroup.Namespace,
	}, job); err != nil {
		logger.Error(err, " Failed to get parent Job")
		taskGroup.Status.State = robotv1alpha1.TaskGroupStateFailed
		taskGroup.Status.StatusDescription = fmt.Sprintf("Parent Job not found: %v", err)
		r.Status().Update(ctx, taskGroup)
		return ctrl.Result{}, err
	}

	// 查找匹配的Robots
	robotList := &robotv1alpha1.RobotList{}
	if err := r.List(ctx, robotList); err != nil {
		logger.Error(err, " Failed to list robots")
		return ctrl.Result{}, err
	}

	// 按在线状态分类robots
	var onlineRobots, pendingRobots, offlineRobots []*robotv1alpha1.Robot
	for i := range robotList.Items {
		robot := &robotList.Items[i]
		if matchesLabels(robot.Spec.Labels, job.Spec.RobotSelector) {
			switch robot.Status.Phase {
			case robotv1alpha1.RobotPhaseOnline:
				onlineRobots = append(onlineRobots, robot)
			case robotv1alpha1.RobotPhasePending:
				pendingRobots = append(pendingRobots, robot)
			default:
				offlineRobots = append(offlineRobots, robot)
			}
		}
	}

	// 合并robots（优先级：Online > Pending > Offline）
	allMatchedRobots := append(append(onlineRobots, pendingRobots...), offlineRobots...)

	if len(allMatchedRobots) == 0 {
		logger.Info("⚠️ No matched robots found, will retry")
		taskGroup.Status.StatusDescription = "No matched robots found"
		r.Status().Update(ctx, taskGroup)
		return ctrl.Result{RequeueAfter: 10 * time.Second}, nil
	}

	logger.Info(" [SCHEDULING] Robot classification",
		"online", len(onlineRobots),
		"pending", len(pendingRobots),
		"offline", len(offlineRobots),
		"total", len(allMatchedRobots))

	// 确定副本数：count字段或匹配的robot数量
	replicaCount := taskGroup.Spec.Count
	if replicaCount == 0 {
		replicaCount = int32(len(allMatchedRobots))
	}

	// 约束：每个robot最多运行一个副本
	if replicaCount > int32(len(allMatchedRobots)) {
		logger.Info("⚠️ Not enough robots for replicas",
			"requested", replicaCount,
			"available", len(allMatchedRobots))
		taskGroup.Status.StatusDescription = fmt.Sprintf("Insufficient robots: need %d, available %d", replicaCount, len(allMatchedRobots))
		r.Status().Update(ctx, taskGroup)
		return ctrl.Result{RequeueAfter: 10 * time.Second}, nil
	}

	// 创建调度分配
	assignments := make([]robotv1alpha1.RobotAssignment, replicaCount)
	for i := int32(0); i < replicaCount; i++ {
		assignments[i] = robotv1alpha1.RobotAssignment{
			ReplicaIndex: i,
			RobotName:    allMatchedRobots[i].Name,
		}
		logger.Info(" [SCHEDULING] Assigned replica to robot",
			"replica", i,
			"robot", allMatchedRobots[i].Name,
			"phase", allMatchedRobots[i].Status.Phase)
	}

	// 更新状态
	taskGroup.Status.AssignedRobots = assignments
	taskGroup.Status.State = robotv1alpha1.TaskGroupStateScheduled
	taskGroup.Status.StatusDescription = fmt.Sprintf("Scheduled %d replicas", replicaCount)
	taskGroup.Status.TotalTasks = replicaCount * int32(len(taskGroup.Spec.Tasks))

	if err := r.Status().Update(ctx, taskGroup); err != nil {
		logger.Error(err, " Failed to update TaskGroup status")
		return ctrl.Result{}, err
	}

	logger.Info(" [SCHEDULING] TaskGroup scheduled",
		"name", taskGroup.Name,
		"replicas", replicaCount,
		"totalTasks", taskGroup.Status.TotalTasks)

	// 立即创建Task实例
	return r.createTasks(ctx, taskGroup)
}

// createTasks 根据AssignedRobots创建Task实例
func (r *TaskGroupReconciler) createTasks(ctx context.Context, taskGroup *robotv1alpha1.TaskGroup) (ctrl.Result, error) {
	logger := log.FromContext(ctx)

	if len(taskGroup.Status.AssignedRobots) == 0 {
		logger.Error(nil, " No AssignedRobots found")
		return ctrl.Result{}, fmt.Errorf("no assigned robots")
	}

	logger.Info("📦 [TASK CREATION] Creating tasks for TaskGroup",
		"taskGroup", taskGroup.Name,
		"replicas", len(taskGroup.Status.AssignedRobots))

	createdCount := 0
	// 为每个副本创建Task
	for _, assignment := range taskGroup.Status.AssignedRobots {
		for _, taskDef := range taskGroup.Spec.Tasks {
			// 生成Task名称：{taskgroup}-r{replicaIndex}-{taskName}（固定名称，可重入）
			taskName := fmt.Sprintf("%s-r%d-%s",
				taskGroup.Name,
				assignment.ReplicaIndex,
				taskDef.Name)

			// 检查Task是否已存在
			existingTask := &robotv1alpha1.Task{}
			err := r.Get(ctx, client.ObjectKey{
				Name:      taskName,
				Namespace: taskGroup.Namespace,
			}, existingTask)

			if err == nil {
				// Task已存在，跳过
				logger.V(1).Info("Task already exists", "task", taskName)
				continue
			}

			if !errors.IsNotFound(err) {
				logger.Error(err, "Failed to check task existence", "task", taskName)
				continue
			}

			// 创建Task
			task := &robotv1alpha1.Task{
				ObjectMeta: metav1.ObjectMeta{
					Name:      taskName,
					Namespace: taskGroup.Namespace,
					Labels: map[string]string{
						"job":       taskGroup.Spec.JobName,
						"taskgroup": taskGroup.Name,
						"replica":   fmt.Sprintf("%d", assignment.ReplicaIndex),
						"task":      taskDef.Name,
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
					JobName:       taskGroup.Spec.JobName,
					TaskGroupName: taskGroup.Name,
					Name:          taskDef.Name,
					Driver:        taskDef.Driver,
					Config:        taskDef.Config,
					TargetRobot:   assignment.RobotName, // 预先分配Robot
					Resources:     taskDef.Resources,
					Env:           taskDef.Env,
					User:          taskDef.User,
					KillTimeout:   taskDef.KillTimeout,
					Artifacts:     taskDef.Artifacts,
					Templates:     taskDef.Templates,
					Network:       taskGroup.Spec.Network,   // 传递网络配置
					InitTasks:     taskGroup.Spec.InitTasks, // 传递初始化任务
				},
			}

			if err := r.Create(ctx, task); err != nil {
				logger.Error(err, " Failed to create task", "task", taskName)
				continue
			}

			createdCount++
			logger.Info(" [TASK CREATION] Created task",
				"task", taskName,
				"replica", assignment.ReplicaIndex,
				"robot", assignment.RobotName,
				"taskDef", taskDef.Name)
		}
	}

	logger.Info(" [TASK CREATION] All tasks processed for TaskGroup",
		"taskGroup", taskGroup.Name,
		"created", createdCount)
	return ctrl.Result{}, nil
}

// monitorTaskGroup 监控Task状态，更新TaskGroup状态
func (r *TaskGroupReconciler) monitorTaskGroup(ctx context.Context, taskGroup *robotv1alpha1.TaskGroup) (ctrl.Result, error) {
	logger := log.FromContext(ctx)

	// 列出所有属于这个TaskGroup的Tasks
	taskList := &robotv1alpha1.TaskList{}
	if err := r.List(ctx, taskList, client.InNamespace(taskGroup.Namespace)); err != nil {
		logger.Error(err, "Failed to list tasks")
		return ctrl.Result{}, err
	}

	// 统计各状态的Task数量
	var pending, scheduled, running, completed, failed int32
	for _, task := range taskList.Items {
		if task.Spec.TaskGroupName != taskGroup.Name {
			continue
		}

		switch task.Status.State {
		case "", robotv1alpha1.TaskStatePending:
			pending++
		case robotv1alpha1.TaskStateScheduled:
			scheduled++
		case robotv1alpha1.TaskStateRunning:
			running++
		case robotv1alpha1.TaskStateCompleted:
			completed++
		case robotv1alpha1.TaskStateFailed:
			failed++
		}
	}

	totalTasks := pending + scheduled + running + completed + failed

	// 更新统计
	taskGroup.Status.PendingTasks = pending
	taskGroup.Status.RunningTasks = running + scheduled // Scheduled也算运行中
	taskGroup.Status.SucceededTasks = completed
	taskGroup.Status.FailedTasks = failed

	// 更新状态
	oldState := taskGroup.Status.State

	if totalTasks > 0 {
		// 至少有一个Task被调度，进入Running
		if taskGroup.Status.State == robotv1alpha1.TaskGroupStateScheduled && (scheduled > 0 || running > 0) {
			taskGroup.Status.State = robotv1alpha1.TaskGroupStateRunning
			taskGroup.Status.StatusDescription = "Tasks are running"
		}

		// 所有Task都完成了
		if completed+failed == totalTasks {
			if failed > 0 {
				taskGroup.Status.State = robotv1alpha1.TaskGroupStateFailed
				taskGroup.Status.StatusDescription = fmt.Sprintf("Failed: %d/%d tasks failed", failed, totalTasks)
			} else {
				taskGroup.Status.State = robotv1alpha1.TaskGroupStateCompleted
				taskGroup.Status.StatusDescription = "All tasks completed successfully"
			}
		}
	}

	if err := r.Status().Update(ctx, taskGroup); err != nil {
		logger.Error(err, "Failed to update status")
		return ctrl.Result{}, err
	}

	if oldState != taskGroup.Status.State {
		logger.Info(" [TASKGROUP] State changed",
			"name", taskGroup.Name,
			"from", oldState,
			"to", taskGroup.Status.State,
			"pending", pending,
			"scheduled", scheduled,
			"running", running,
			"completed", completed,
			"failed", failed)
	}

	// 如果还在运行中，继续监控
	if taskGroup.Status.State == robotv1alpha1.TaskGroupStateScheduled ||
		taskGroup.Status.State == robotv1alpha1.TaskGroupStateRunning {
		return ctrl.Result{RequeueAfter: 10 * time.Second}, nil
	}

	return ctrl.Result{}, nil
}

// matchesLabels 检查robot的labels是否匹配selector
func matchesLabels(robotLabels, selector map[string]string) bool {
	// 空selector匹配所有
	if len(selector) == 0 {
		return true
	}

	for key, value := range selector {
		if robotLabels[key] != value {
			return false
		}
	}
	return true
}

// SetupWithManager sets up the controller with the Manager.
func (r *TaskGroupReconciler) SetupWithManager(mgr ctrl.Manager) error {
	return ctrl.NewControllerManagedBy(mgr).
		For(&robotv1alpha1.TaskGroup{}).
		Complete(r)
}
