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
)

// Hook 统一的生命周期钩子接口
// 适用于 Task、TaskGroup 等所有对象的生命周期管理
// Hook 通过类型断言判断是否需要处理特定类型的对象
type Hook interface {
	// Name 返回 Hook 的名称（用于日志和调试）
	Name() string

	// PreStart 在启动之前调用
	// subject 可以是 *robotv1alpha1.Task, *robotv1alpha1.TaskGroup, 或其他对象
	PreStart(ctx context.Context, subject interface{}) error

	// PostStart 在启动之后调用
	// subject 可以是 *robotv1alpha1.Task, *TaskHandle, *robotv1alpha1.TaskGroup 等
	PostStart(ctx context.Context, subject interface{}) error

	// PreStop 在停止之前调用
	PreStop(ctx context.Context, subject interface{}) error

	// PostStop 在停止之后调用
	PostStop(ctx context.Context, subject interface{}) error
}

// HookManager 管理和执行 Hook 的工具
type HookManager struct {
	hooks []Hook
}

// NewHookManager 创建新的 HookManager
func NewHookManager() *HookManager {
	return &HookManager{
		hooks: make([]Hook, 0),
	}
}

// AddHook 添加一个 Hook
func (hm *HookManager) AddHook(hook Hook) {
	hm.hooks = append(hm.hooks, hook)
}

// AddHooks 批量添加 Hooks
func (hm *HookManager) AddHooks(hooks ...Hook) {
	hm.hooks = append(hm.hooks, hooks...)
}

// RunPreStart 执行所有 PreStart hooks
func (hm *HookManager) RunPreStart(ctx context.Context, subject interface{}) error {
	for _, hook := range hm.hooks {
		if err := hook.PreStart(ctx, subject); err != nil {
			return err
		}
	}
	return nil
}

// RunPostStart 执行所有 PostStart hooks
func (hm *HookManager) RunPostStart(ctx context.Context, subject interface{}) error {
	for _, hook := range hm.hooks {
		if err := hook.PostStart(ctx, subject); err != nil {
			return err
		}
	}
	return nil
}

// RunPreStop 执行所有 PreStop hooks
func (hm *HookManager) RunPreStop(ctx context.Context, subject interface{}) error {
	for _, hook := range hm.hooks {
		if err := hook.PreStop(ctx, subject); err != nil {
			return err
		}
	}
	return nil
}

// RunPostStop 执行所有 PostStop hooks
// 注意：PostStop 会执行所有 hooks，即使某个 hook 返回错误
// 这样可以确保清理工作尽可能完成
func (hm *HookManager) RunPostStop(ctx context.Context, subject interface{}) error {
	var firstErr error
	for _, hook := range hm.hooks {
		if err := hook.PostStop(ctx, subject); err != nil && firstErr == nil {
			firstErr = err
		}
	}
	return firstErr
}
