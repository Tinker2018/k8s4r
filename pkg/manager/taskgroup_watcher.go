/*
Copyright 2025.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0
*/

package manager

import (
	"context"
	"time"

	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/log"

	robotv1alpha1 "github.com/hxndghxndg/k8s4r/api/v1alpha1"
)

// TaskGroupWatcher 监控 TaskGroup 状态变化，并推送到 gRPC Stream
type TaskGroupWatcher struct {
	client               client.Client
	namespace            string
	taskGroupStreamMgr   *TaskGroupStreamManager
	dispatchedTaskGroups map[string]map[string]bool // taskGroupUID -> robotName -> dispatched
	ctx                  context.Context
	cancel               context.CancelFunc
}

// NewTaskGroupWatcher 创建 TaskGroupWatcher
func NewTaskGroupWatcher(client client.Client, namespace string, streamMgr *TaskGroupStreamManager) *TaskGroupWatcher {
	ctx, cancel := context.WithCancel(context.Background())
	return &TaskGroupWatcher{
		client:               client,
		namespace:            namespace,
		taskGroupStreamMgr:   streamMgr,
		dispatchedTaskGroups: make(map[string]map[string]bool),
		ctx:                  ctx,
		cancel:               cancel,
	}
}

// Start 启动 TaskGroup Watcher
func (w *TaskGroupWatcher) Start() {
	logger := log.Log.WithName("taskgroup-watcher")
	logger.Info("🔍 Starting TaskGroup Watcher")

	go func() {
		ticker := time.NewTicker(2 * time.Second)
		defer ticker.Stop()

		for {
			select {
			case <-w.ctx.Done():
				logger.Info("TaskGroup Watcher stopped")
				return
			case <-ticker.C:
				w.watchTaskGroups()
			}
		}
	}()
}

// Stop 停止 Watcher
func (w *TaskGroupWatcher) Stop() {
	w.cancel()
}

// watchTaskGroups 监控所有 TaskGroup 并推送 scheduled 状态的
func (w *TaskGroupWatcher) watchTaskGroups() {
	logger := log.FromContext(w.ctx)

	taskGroupList := &robotv1alpha1.TaskGroupList{}
	if err := w.client.List(w.ctx, taskGroupList, client.InNamespace(w.namespace)); err != nil {
		logger.Error(err, "Failed to list taskgroups")
		return
	}

	for i := range taskGroupList.Items {
		taskGroup := &taskGroupList.Items[i]
		taskGroupUID := string(taskGroup.UID)

		// 只处理 scheduled 状态的 TaskGroup
		if taskGroup.Status.State != robotv1alpha1.TaskGroupStateScheduled {
			continue
		}

		// 为每个分配的机器人推送 TaskGroup
		for _, assignment := range taskGroup.Status.AssignedRobots {
			robotName := assignment.RobotName

			// 检查是否已经推送过
			if w.dispatchedTaskGroups[taskGroupUID] == nil {
				w.dispatchedTaskGroups[taskGroupUID] = make(map[string]bool)
			}

			if w.dispatchedTaskGroups[taskGroupUID][robotName] {
				continue // 已推送，跳过
			}

			// 推送到 gRPC Stream
			logger.Info("📤 Pushing TaskGroup to Stream",
				"taskGroup", taskGroup.Name,
				"robot", robotName,
				"replica", assignment.ReplicaIndex)

			if err := w.taskGroupStreamMgr.PushTaskGroupToStream(w.ctx, taskGroup, robotName); err != nil {
				logger.Error(err, "Failed to push taskgroup to stream",
					"taskGroup", taskGroup.Name,
					"robot", robotName)
			} else {
				// 标记为已推送
				w.dispatchedTaskGroups[taskGroupUID][robotName] = true
				logger.Info("✅ TaskGroup pushed successfully",
					"taskGroup", taskGroup.Name,
					"robot", robotName)
			}
		}

		// 如果 TaskGroup 被删除，推送删除消息
		if taskGroup.DeletionTimestamp != nil {
			for _, assignment := range taskGroup.Status.AssignedRobots {
				robotName := assignment.RobotName

				logger.Info("🗑️ Pushing DELETE TaskGroup to Stream",
					"taskGroup", taskGroup.Name,
					"robot", robotName)

				if err := w.taskGroupStreamMgr.PushDeleteTaskGroupToStream(w.ctx, taskGroupUID, robotName); err != nil {
					logger.Error(err, "Failed to push delete taskgroup",
						"taskGroup", taskGroup.Name,
						"robot", robotName)
				}
			}

			// 清理已分发记录
			delete(w.dispatchedTaskGroups, taskGroupUID)
		}
	}
}
