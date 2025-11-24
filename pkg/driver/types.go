package driver

import (
	"context"
	"time"

	robotv1alpha1 "github.com/hxndghxndg/k8s4r/api/v1alpha1"
)

// EventCallback 事件回调函数，用于 Driver 向上层报告执行阶段
type EventCallback func(taskUID, event, message string)

// TaskDriver 定义任务驱动接口
type TaskDriver interface {
	// Name 返回驱动名称
	Name() string

	// Start 启动任务
	Start(ctx context.Context, task *robotv1alpha1.Task) (*TaskHandle, error)

	// Stop 停止任务
	Stop(ctx context.Context, handle *TaskHandle) error

	// Status 获取任务状态
	Status(ctx context.Context, handle *TaskHandle) (*TaskStatus, error)

	// Destroy 销毁任务（清理资源）
	Destroy(ctx context.Context, handle *TaskHandle) error

	// SetEventCallback 设置事件回调（可选）
	SetEventCallback(callback EventCallback)
} // TaskHandle 代表一个运行中的任务实例
type TaskHandle struct {
	TaskID     string
	DriverName string
	PID        int
	StartedAt  time.Time
	Config     map[string]interface{}
}

// TaskStatus 任务状态
type TaskStatus struct {
	State      TaskState
	ExitCode   int
	Signal     string
	Message    string
	StartedAt  time.Time
	FinishedAt time.Time
	Resources  *ResourceUsage
}

// TaskState 任务状态枚举
type TaskState string

const (
	TaskStateUnknown TaskState = "unknown"
	TaskStatePending TaskState = "pending"
	TaskStateRunning TaskState = "running"
	TaskStateExited  TaskState = "exited"
	TaskStateFailed  TaskState = "failed"
)

// ResourceUsage 资源使用情况
type ResourceUsage struct {
	CPUPercent float64
	MemoryMB   int64
	DiskMB     int64
}

// DriverConfig 驱动配置
type DriverConfig struct {
	Command string
	Args    []string
	Env     map[string]string
	WorkDir string
	User    string
}
