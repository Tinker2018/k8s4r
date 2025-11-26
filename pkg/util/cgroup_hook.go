/*
Copyright 2025.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0
*/

package util

import (
	"context"
	"fmt"
	"os"
	"path/filepath"
	"runtime"
	"sync"

	"github.com/hashicorp/go-hclog"
	"github.com/hashicorp/nomad/client/lib/cgroupslib"
	robotv1alpha1 "github.com/hxndghxndg/k8s4r/api/v1alpha1"
)

// CgroupHook 管理 cgroup 的创建、配置和清理
// 实现两层 cgroup 结构：
// - TaskGroup 层：限制整个 TaskGroup 的总资源
// - Task 层：限制单个 Task 的资源
type CgroupHook struct {
	logger hclog.Logger
	mu     sync.Mutex

	// cgroupRefCount 跟踪每个 TaskGroup 的活跃 Task 数量
	// key: taskGroupName, value: 活跃任务数
	cgroupRefCount map[string]int

	// taskGroupNames 记录 taskID -> taskGroupName 的映射
	// 用于在 PostStop 时查找对应的 TaskGroup
	taskGroupNames map[string]string

	// baseDir cgroup 的基础路径，这个目前是固定的/sys/fs/cgroup/nomad.slice/share.slice
	baseDir string
}

// NewCgroupHook 创建新的 CgroupHook
func NewCgroupHook(logger hclog.Logger, baseDir string) *CgroupHook {
	return &CgroupHook{
		logger:         logger.Named("cgroup_hook"),
		cgroupRefCount: make(map[string]int),
		taskGroupNames: make(map[string]string),
		baseDir:        baseDir,
	}
}

// Name 返回 Hook 名称
func (h *CgroupHook) Name() string {
	return "cgroup"
}

// PreStart 在启动之前调用
// 为 TaskGroup 或 Task 创建 cgroup
func (h *CgroupHook) PreStart(ctx context.Context, subject interface{}) error {
	switch v := subject.(type) {
	case *robotv1alpha1.TaskGroup:
		return h.createTaskGroupCgroup(v)
	case *robotv1alpha1.Task:
		return h.createTaskCgroup(v)
	default:
		// 不关心的类型，跳过
		return nil
	}
}

// PostStart 在启动之后调用
// 增加 Task 的引用计数
func (h *CgroupHook) PostStart(ctx context.Context, subject interface{}) error {
	// 检查是否是 Task 和 TaskHandle 的组合
	// 这里我们期望传入一个包含 Task 信息的结构
	switch v := subject.(type) {
	case *robotv1alpha1.Task:
		return h.incrementRefCount(v)
	case *robotv1alpha1.TaskGroup:
		return h.incrementTaskGroupRefCount(v)
	default:
		return nil
	}
}

// PreStop 在停止之前调用
func (h *CgroupHook) PreStop(ctx context.Context, subject interface{}) error {
	// 目前不需要在停止前做任何操作
	return nil
}

// PostStop 在停止之后调用
// 减少引用计数，可能清理 cgroup
func (h *CgroupHook) PostStop(ctx context.Context, subject interface{}) error {
	switch v := subject.(type) {
	case *robotv1alpha1.Task:
		return h.decrementAndCleanupTask(v)
	case *robotv1alpha1.TaskGroup:
		return h.decrementAndCleanupTaskGroup(v)
	default:
		return nil
	}
}

// createTaskGroupCgroup 创建 TaskGroup 级别的 cgroup
func (h *CgroupHook) createTaskGroupCgroup(taskGroup *robotv1alpha1.TaskGroup) error {
	h.mu.Lock()
	defer h.mu.Unlock()

	taskGroupName := taskGroup.Name

	// TaskGroup 级别的 cgroup 路径
	cgroupPath := filepath.Join(h.baseDir, taskGroupName)

	// 创建目录（如果不存在）
	if err := os.MkdirAll(cgroupPath, 0755); err != nil {
		return fmt.Errorf("failed to create TaskGroup cgroup %s: %w", cgroupPath, err)
	}

	h.logger.Info("[CGROUP] created TaskGroup-level cgroup",
		"taskGroup", taskGroupName,
		"path", cgroupPath)

	// TODO: 在这里配置 TaskGroup 级别的资源限制
	// 例如：从 taskGroup.Spec.Resources 读取配置并写入 cgroup 文件
	// - cpu.max
	// - memory.max
	// - pids.max
	cores := fmt.Sprintf("0-%d", runtime.NumCPU()-1)

	h.logger.Info("[CGROUP] initializing cgroups for k8s4r", "cores", cores, "mode", cgroupslib.GetMode())
	// 调用 Nomad 的 cgroup 初始化
	// 注意：这需要 root 权限！
	h.logger.Info("[CGROUP] calling cgroupslib.Init() - this requires root privileges")
	if err := cgroupslib.Init(h.logger, cores); err != nil {
		h.logger.Error("[CGROUP] failed to initialize cgroups", "error", err)
		return fmt.Errorf("[CGROUP] failed to initialize cgroups (requires root): %w", err)
	}

	return nil
}

// createTaskCgroup 创建 Task 级别的 cgroup
func (h *CgroupHook) createTaskCgroup(task *robotv1alpha1.Task) error {
	h.mu.Lock()
	defer h.mu.Unlock()

	taskID := string(task.UID)
	taskGroupName := task.Spec.TaskGroupName

	// Task 级别的 cgroup 路径（嵌套在 TaskGroup 下）
	cgroupPath := filepath.Join(h.baseDir, taskGroupName, taskID)

	// 创建目录
	if err := os.MkdirAll(cgroupPath, 0755); err != nil {
		return fmt.Errorf("failed to create Task cgroup %s: %w", cgroupPath, err)
	}

	h.logger.Info("[CGROUP] created Task-level cgroup",
		"taskID", taskID,
		"taskGroup", taskGroupName,
		"path", cgroupPath)

	// 记录 taskID -> taskGroupName 映射
	h.taskGroupNames[taskID] = taskGroupName

	// TODO: 在这里配置 Task 级别的资源限制
	// 从 task.Spec.Resources 读取配置

	cores := fmt.Sprintf("0-%d", runtime.NumCPU()-1)

	h.logger.Info("[CGROUP] initializing cgroups for k8s4r", "cores", cores, "mode", cgroupslib.GetMode())
	// 调用 Nomad 的 cgroup 初始化
	// 注意：这需要 root 权限！
	// 这里注意，在task里面也调用cgroup init，因为可能没有初始化taskgroup的cgroup（尤其是在test里面）
	h.logger.Info("[CGROUP] calling cgroupslib.Init() - this requires root privileges")
	if err := cgroupslib.Init(h.logger, cores); err != nil {
		h.logger.Error("[CGROUP] failed to initialize cgroups", "error", err)
		return fmt.Errorf("[CGROUP] failed to initialize cgroups (requires root): %w", err)
	}
	return nil
}

// incrementRefCount 增加 TaskGroup 的引用计数
func (h *CgroupHook) incrementRefCount(task *robotv1alpha1.Task) error {
	h.mu.Lock()
	defer h.mu.Unlock()

	taskGroupName := task.Spec.TaskGroupName
	h.cgroupRefCount[taskGroupName]++

	h.logger.Info("[CGROUP] incremented TaskGroup cgroup ref count",
		"taskGroup", taskGroupName,
		"count", h.cgroupRefCount[taskGroupName])

	return nil
}

// incrementRefCount 增加 TaskGroup 的引用计数
func (h *CgroupHook) incrementTaskGroupRefCount(taskGroup *robotv1alpha1.TaskGroup) error {
	h.mu.Lock()
	defer h.mu.Unlock()

	taskGroupName := taskGroup.Name
	h.cgroupRefCount[taskGroupName]++

	h.logger.Info("[CGROUP] incremented TaskGroup cgroup ref count",
		"taskGroup", taskGroupName,
		"count", h.cgroupRefCount[taskGroupName])

	return nil
}

// decrementAndCleanupTask 减少引用计数并清理 Task 级别的 cgroup
func (h *CgroupHook) decrementAndCleanupTask(task *robotv1alpha1.Task) error {
	h.mu.Lock()
	defer h.mu.Unlock()

	taskID := string(task.UID)
	// 查找对应的 TaskGroup
	taskGroupName, ok := h.taskGroupNames[taskID]
	if !ok {
		h.logger.Warn("[CGROUP] task not found in taskGroupNames map", "taskID", taskID)
		return nil
	}

	// 1. 删除 Task 级别的 cgroup
	taskCgroupPath := filepath.Join(h.baseDir, taskGroupName, taskID)
	if err := os.RemoveAll(taskCgroupPath); err != nil {
		h.logger.Warn("[CGROUP] failed to remove task-level cgroup",
			"taskID", taskID,
			"path", taskCgroupPath,
			"error", err)
	} else {
		h.logger.Info("[CGROUP] removed task-level cgroup",
			"taskID", taskID,
			"path", taskCgroupPath)
	}

	// 2. 减少 TaskGroup 的引用计数
	h.cgroupRefCount[taskGroupName]--
	refCount := h.cgroupRefCount[taskGroupName]

	h.logger.Info("[CGROUP] decremented TaskGroup cgroup ref count",
		"taskGroup", taskGroupName,
		"count", refCount)

	// 3. 如果引用计数为 0，清理 TaskGroup 级别的 cgroup
	if refCount <= 0 {
		delete(h.cgroupRefCount, taskGroupName)

		taskGroupCgroupPath := filepath.Join(h.baseDir, taskGroupName)
		if err := os.RemoveAll(taskGroupCgroupPath); err != nil {
			h.logger.Warn("[CGROUP] failed to remove TaskGroup-level cgroup",
				"taskGroup", taskGroupName,
				"path", taskGroupCgroupPath,
				"error", err)
			return err
		} else {
			h.logger.Info("[CGROUP] removed TaskGroup-level cgroup (all tasks completed)",
				"taskGroup", taskGroupName,
				"path", taskGroupCgroupPath)
		}
	}

	// 清理映射
	delete(h.taskGroupNames, taskID)

	return nil
}

// cleanupTaskGroup 清理整个 TaskGroup 的 cgroup（强制清理）
func (h *CgroupHook) decrementAndCleanupTaskGroup(taskGroup *robotv1alpha1.TaskGroup) error {
	return h.decrementAndCleanupTaskGroupByName(taskGroup.Name)
}

// cleanupTaskGroupByName 根据名称清理 TaskGroup 的 cgroup
func (h *CgroupHook) decrementAndCleanupTaskGroupByName(taskGroupName string) error {
	h.mu.Lock()
	defer h.mu.Unlock()

	// 强制清除引用计数
	delete(h.cgroupRefCount, taskGroupName)

	// 删除 TaskGroup 级别的 cgroup（会递归删除所有 Task 级别的 cgroup）
	cgroupPath := filepath.Join(h.baseDir, taskGroupName)
	if err := os.RemoveAll(cgroupPath); err != nil {
		h.logger.Warn("[CGROUP] failed to remove TaskGroup cgroup hierarchy",
			"taskGroup", taskGroupName,
			"path", cgroupPath,
			"error", err)
		return fmt.Errorf("[CGROUP] failed to remove cgroup hierarchy for TaskGroup %s: %w", taskGroupName, err)
	}

	h.logger.Info("[CGROUP] TaskGroup cgroup hierarchy cleaned up (forced)",
		"taskGroup", taskGroupName,
		"path", cgroupPath)

	// 清理所有相关的 taskID 映射
	for taskID, tgName := range h.taskGroupNames {
		if tgName == taskGroupName {
			delete(h.taskGroupNames, taskID)
		}
	}

	return nil
}
