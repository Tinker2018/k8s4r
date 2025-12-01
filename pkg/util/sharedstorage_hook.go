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
	"sync"

	getter "github.com/hashicorp/go-getter"
	"github.com/hashicorp/go-hclog"
	robotv1alpha1 "github.com/hxndghxndg/k8s4r/api/v1alpha1"
)

// SharedStorageHook 管理 TaskGroup 级别的共享存储
// 负责创建共享目录、下载 artifacts、创建 symlink 等
type SharedStorageHook struct {
	logger  hclog.Logger
	mu      sync.Mutex
	baseDir string

	// taskGroupDirs 记录已创建的 TaskGroup 共享目录
	// key: taskGroupName, value: shared directory path
	taskGroupDirs map[string]string

	// httpClient 用于下载 artifacts（带超时配置）
	httpClient *getter.HttpGetter
}

// NewSharedStorageHook 创建新的 SharedStorageHook
func NewSharedStorageHook(logger hclog.Logger, baseDir string) *SharedStorageHook {
	return &SharedStorageHook{
		logger:        logger.Named("sharedstorage_hook"),
		baseDir:       baseDir,
		taskGroupDirs: make(map[string]string),
		httpClient:    &getter.HttpGetter{},
	}
}

// Name 返回 Hook 名称
func (h *SharedStorageHook) Name() string {
	return "shared_storage"
}

// PreStart 在任务启动前创建共享目录和下载 artifacts
func (h *SharedStorageHook) PreStart(ctx context.Context, subject interface{}) error {
	switch v := subject.(type) {
	case *robotv1alpha1.Task:
		return h.prepareTaskStorage(ctx, v)
	default:
		return nil
	}
}

// PostStart 在任务启动后不需要额外操作
func (h *SharedStorageHook) PostStart(ctx context.Context, subject interface{}) error {
	return nil
}

// PreStop 在任务停止前不需要操作
func (h *SharedStorageHook) PreStop(ctx context.Context, subject interface{}) error {
	return nil
}

// PostStop 在任务停止后清理共享存储（如果是 TaskGroup 的最后一个任务）
func (h *SharedStorageHook) PostStop(ctx context.Context, subject interface{}) error {
	switch v := subject.(type) {
	case *robotv1alpha1.TaskGroup:
		return h.cleanupTaskGroupStorage(v.Name)
	default:
		return nil
	}
}

// prepareTaskStorage 为 Task 准备存储
func (h *SharedStorageHook) prepareTaskStorage(ctx context.Context, task *robotv1alpha1.Task) error {
	h.mu.Lock()
	defer h.mu.Unlock()

	taskGroupName := task.Spec.TaskGroupName
	taskID := string(task.UID)

	// 1. 确保 TaskGroup 共享目录存在，并且创建存储artifacts的目录
	taskGroupDir := filepath.Join(h.baseDir, taskGroupName)

	if err := os.MkdirAll(taskGroupDir, 0755); err != nil {
		return fmt.Errorf("failed to create taskGroupDir dir %s: %w", taskGroupDir, err)
	}

	h.logger.Debug("created TaskGroup shared directory")

	// 2. 为当前 Task 下载具体文件存储到 task 的工作目录
	// taskDir 由 Executor 创建，我们需要知道它的路径；如果不存在则创建
	taskDir := filepath.Join(taskGroupDir, taskID)
	if err := os.MkdirAll(taskDir, 0755); err != nil {
		return fmt.Errorf("failed to create task dir %s: %w", taskDir, err)
	}

	// 3. 下载 artifacts（如果有），直接写入 taskDir/<relativeDest>
	if len(task.Spec.Artifacts) > 0 {
		h.logger.Info("downloading artifacts for task",
			"taskID", taskID,
			"count", len(task.Spec.Artifacts),
			"destination", taskDir)

		if err := h.downloadArtifacts(ctx, task.Spec.Artifacts, taskDir); err != nil {
			return fmt.Errorf("failed to download artifacts: %w", err)
		}

		h.logger.Info("artifacts downloaded successfully", "taskID", taskID)
	}

	return nil
}

// downloadArtifacts 下载 artifacts 到指定目录
func (h *SharedStorageHook) downloadArtifacts(ctx context.Context, artifacts []robotv1alpha1.TaskArtifact, taskDir string) error {
	for _, artifact := range artifacts {
		source := artifact.GetterSource
		dest := filepath.Join(taskDir, artifact.RelativeDest)

		h.logger.Debug("downloading artifact",
			"source", source,
			"dest", dest)

		// ensure parent dir for destination exists
		if err := os.MkdirAll(filepath.Dir(dest), 0755); err != nil {
			return fmt.Errorf("failed to create parent dir for %s: %w", dest, err)
		}

		// 使用 go-getter 下载到 task 目录下
		client := &getter.Client{
			Ctx:  ctx,
			Src:  source,
			Dst:  dest,
			Mode: getter.ClientModeAny,
			Getters: map[string]getter.Getter{
				"http":  h.httpClient,
				"https": h.httpClient,
			},
		}

		if err := client.Get(); err != nil {
			return fmt.Errorf("failed to download artifact from %s: %w", source, err)
		}
	}

	return nil
}

// cleanupTaskGroupStorage 清理 TaskGroup 的共享存储, 这里我们先不删除，因为shared被挂载为根目录
func (h *SharedStorageHook) cleanupTaskGroupStorage(taskGroupName string) error {
	h.mu.Lock()
	defer h.mu.Unlock()

	// sharedDir, exists := h.taskGroupDirs[taskGroupName]
	// if !exists {
	// 	return nil
	// }

	// // 删除共享目录
	// if err := os.RemoveAll(sharedDir); err != nil {
	// 	h.logger.Warn("failed to remove TaskGroup shared directory",
	// 		"taskGroup", taskGroupName,
	// 		"sharedDir", sharedDir,
	// 		"error", err)
	// } else {
	// 	h.logger.Info("TaskGroup shared directory cleaned up",
	// 		"taskGroup", taskGroupName,
	// 		"sharedDir", sharedDir)
	// }

	delete(h.taskGroupDirs, taskGroupName)
	return nil
}
