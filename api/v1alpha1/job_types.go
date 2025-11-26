/*
Copyright 2025.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0
*/

package v1alpha1

import (
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
)

// JobSpec defines the desired state of Job
// Job 是核心控制单元，定义了应用程序及其配置
type JobSpec struct {
	// Name 是 Job 的名称
	// +kubebuilder:validation:Required
	Name string `json:"name"`

	// RobotSelector 使用 label 选择目标 Robot（类似 Kubernetes nodeSelector）
	// 例如: {"env": "production", "region": "us-west"}
	// 只有匹配所有 label 的 Robot 才会被选中执行此 Job
	// +optional
	RobotSelector map[string]string `json:"robotSelector,omitempty"`

	// Type 指定 Job 类型
	// +kubebuilder:validation:Enum=service;batch;system
	// +optional
	Type JobType `json:"type,omitempty"`

	// Datacenters 数据中心列表（用于多数据中心部署）
	// +optional
	Datacenters []string `json:"datacenters,omitempty"`

	// Priority 优先级 (0-100)
	// +optional
	// +kubebuilder:validation:Minimum=0
	// +kubebuilder:validation:Maximum=100
	Priority *int32 `json:"priority,omitempty"`

	// TaskGroups 定义任务组模板列表
	// +kubebuilder:validation:Required
	// +kubebuilder:validation:MinItems=1
	TaskGroups []TaskGroupTemplate `json:"taskGroups"`

	// Meta 元数据
	// +optional
	Meta map[string]string `json:"meta,omitempty"`
}

// JobType 定义 Job 类型
type JobType string

const (
	// JobTypeService 长期运行的服务
	JobTypeService JobType = "service"

	// JobTypeBatch 批处理任务
	JobTypeBatch JobType = "batch"

	// JobTypeSystem 系统级任务
	JobTypeSystem JobType = "system"
)

// TaskGroupTemplate 定义任务组模板（在 Job Spec 中使用）
// JobController 会根据这个模板创建实际的 TaskGroup CRD
type TaskGroupTemplate struct {
	// Name 任务组名称
	// +kubebuilder:validation:Required
	Name string `json:"name"`

	// Count 需要运行的实例数量
	// +kubebuilder:validation:Required
	// +kubebuilder:validation:Minimum=1
	Count int32 `json:"count"`

	// InitTasks 初始化任务列表（类似 K8s initContainers）
	// 这些任务在主任务执行前按顺序执行
	// +optional
	InitTasks []TaskDefinition `json:"initTasks,omitempty"`

	// Tasks 任务列表
	// +kubebuilder:validation:Required
	// +kubebuilder:validation:MinItems=1
	Tasks []TaskDefinition `json:"tasks"`

	// Constraints 任务组级别的约束
	// +optional
	Constraints []Constraint `json:"constraints,omitempty"`

	// RestartPolicy 重启策略
	// +optional
	RestartPolicy *RestartPolicy `json:"restartPolicy,omitempty"`

	// ReschedulePolicy 重新调度策略
	// +optional
	ReschedulePolicy *ReschedulePolicy `json:"reschedulePolicy,omitempty"`

	// EphemeralDisk 临时磁盘配置
	// +optional
	EphemeralDisk *EphemeralDisk `json:"ephemeralDisk,omitempty"`

	// Meta 元数据
	// +optional
	Meta map[string]string `json:"meta,omitempty"`
}

// TaskDefinition 定义 Task 模板（用于 Job 中）
type TaskDefinition struct {
	// Name 任务名称
	// +kubebuilder:validation:Required
	Name string `json:"name"`

	// Driver 驱动程序类型
	// +kubebuilder:validation:Required
	// +kubebuilder:validation:Enum=exec;docker;java;raw_exec
	Driver TaskDriverType `json:"driver"`

	// Config 驱动程序配置
	// +kubebuilder:validation:Required
	Config TaskDriverConfig `json:"config"`

	// Resources 资源需求
	// +optional
	Resources *TaskResources `json:"resources,omitempty"`

	// Env 环境变量
	// +optional
	Env map[string]string `json:"env,omitempty"`

	// User 运行用户
	// +optional
	User string `json:"user,omitempty"`

	// KillTimeout 终止超时
	// +optional
	KillTimeout *metav1.Duration `json:"killTimeout,omitempty"`

	// Artifacts 需要下载的文件
	// +optional
	Artifacts []TaskArtifact `json:"artifacts,omitempty"`

	// Templates 配置模板
	// +optional
	Templates []TaskTemplate `json:"templates,omitempty"`

	// Daemon 是否为守护进程模式（用于 initTasks）
	// 当设置为 true 时，任务启动后不等待其完成即继续执行后续任务
	// 主要用于启动长期运行的后台服务（如 SPIRE Agent、Envoy 等）
	// +optional
	Daemon bool `json:"daemon,omitempty"`
}

// Constraint 定义约束条件
type Constraint struct {
	// LTarget 左目标（例如 ${attr.kernel.name}）
	// +optional
	LTarget string `json:"lTarget,omitempty"`

	// RTarget 右目标（比较值）
	// +optional
	RTarget string `json:"rTarget,omitempty"`

	// Operand 操作符
	// +kubebuilder:validation:Enum==;!=;>;>=;<;<=;regexp;set_contains;version
	// +optional
	Operand string `json:"operand,omitempty"`
}

// RestartPolicy 重启策略
type RestartPolicy struct {
	// Attempts 重启尝试次数
	// +optional
	Attempts *int32 `json:"attempts,omitempty"`

	// Interval 重启间隔时间窗口
	// +optional
	Interval *metav1.Duration `json:"interval,omitempty"`

	// Delay 重启延迟
	// +optional
	Delay *metav1.Duration `json:"delay,omitempty"`

	// Mode 重启模式
	// +kubebuilder:validation:Enum=fail;delay
	// +optional
	Mode string `json:"mode,omitempty"`
}

// ReschedulePolicy 重新调度策略
type ReschedulePolicy struct {
	// Attempts 重新调度尝试次数
	// +optional
	Attempts *int32 `json:"attempts,omitempty"`

	// Interval 时间窗口
	// +optional
	Interval *metav1.Duration `json:"interval,omitempty"`

	// Delay 延迟
	// +optional
	Delay *metav1.Duration `json:"delay,omitempty"`

	// DelayFunction 延迟函数
	// +kubebuilder:validation:Enum=constant;exponential;fibonacci
	// +optional
	DelayFunction string `json:"delayFunction,omitempty"`

	// MaxDelay 最大延迟
	// +optional
	MaxDelay *metav1.Duration `json:"maxDelay,omitempty"`

	// Unlimited 是否无限重试
	// +optional
	Unlimited *bool `json:"unlimited,omitempty"`
}

// EphemeralDisk 临时磁盘配置
type EphemeralDisk struct {
	// Migrate 是否在重新调度时迁移数据
	// +optional
	Migrate *bool `json:"migrate,omitempty"`

	// SizeMB 磁盘大小（MB）
	// +optional
	SizeMB *int32 `json:"sizeMB,omitempty"`

	// Sticky 是否粘性（保留数据）
	// +optional
	Sticky *bool `json:"sticky,omitempty"`
}

// JobStatus defines the observed state of Job
type JobStatus struct {
	// State Job 当前状态
	// +optional
	State JobState `json:"state,omitempty"`

	// StatusDescription 状态描述
	// +optional
	StatusDescription string `json:"statusDescription,omitempty"`

	// CreateIndex 创建索引
	// +optional
	CreateIndex int64 `json:"createIndex,omitempty"`

	// ModifyIndex 修改索引
	// +optional
	ModifyIndex int64 `json:"modifyIndex,omitempty"`

	// TotalTasks 总任务数
	// +optional
	TotalTasks int32 `json:"totalTasks,omitempty"`

	// SucceededTasks 成功完成的任务数
	// +optional
	SucceededTasks int32 `json:"succeededTasks,omitempty"`

	// FailedTasks 失败的任务数
	// +optional
	FailedTasks int32 `json:"failedTasks,omitempty"`

	// RunningTasks 运行中的任务数
	// +optional
	RunningTasks int32 `json:"runningTasks,omitempty"`

	// PendingTasks 等待中的任务数
	// +optional
	PendingTasks int32 `json:"pendingTasks,omitempty"`

	// TaskGroups 关联的TaskGroup资源列表
	// +optional
	TaskGroups []string `json:"taskGroups,omitempty"`

	// Tasks 关联的Task资源列表
	// +optional
	Tasks []string `json:"tasks,omitempty"`

	// TaskGroupSummary 任务组状态摘要
	// +optional
	TaskGroupSummary map[string]TaskGroupSummary `json:"taskGroupSummary,omitempty"`
}

// JobState 定义 Job 的状态
type JobState string

const (
	// JobStatePending Job 刚创建，等待TaskGroup调度
	JobStatePending JobState = "pending"

	// JobStateRunning 至少有一个TaskGroup已调度(Scheduled)
	JobStateRunning JobState = "running"

	// JobStateCompleted 所有TaskGroup都已完成
	JobStateCompleted JobState = "completed"

	// JobStateFailed 有TaskGroup失败
	JobStateFailed JobState = "failed"
)

// TaskGroupSummary 任务组状态摘要
type TaskGroupSummary struct {
	// Queued 排队中的数量
	Queued int32 `json:"queued"`

	// Complete 已完成的数量
	Complete int32 `json:"complete"`

	// Failed 失败的数量
	Failed int32 `json:"failed"`

	// Running 运行中的数量
	Running int32 `json:"running"`

	// Starting 启动中的数量
	Starting int32 `json:"starting"`

	// Lost 丢失的数量
	Lost int32 `json:"lost"`
}

//+kubebuilder:object:root=true
//+kubebuilder:subresource:status
//+kubebuilder:resource:scope=Namespaced,shortName=rjob
//+kubebuilder:printcolumn:name="Type",type=string,JSONPath=`.spec.type`
//+kubebuilder:printcolumn:name="State",type=string,JSONPath=`.status.state`
//+kubebuilder:printcolumn:name="Total",type=integer,JSONPath=`.status.totalTasks`,description="Total tasks"
//+kubebuilder:printcolumn:name="Succeeded",type=integer,JSONPath=`.status.succeededTasks`,description="Succeeded tasks"
//+kubebuilder:printcolumn:name="Failed",type=integer,JSONPath=`.status.failedTasks`,description="Failed tasks"
//+kubebuilder:printcolumn:name="Running",type=integer,JSONPath=`.status.runningTasks`,description="Running tasks"
//+kubebuilder:printcolumn:name="Age",type=date,JSONPath=`.metadata.creationTimestamp`

// Job is the Schema for the jobs API
type Job struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec   JobSpec   `json:"spec,omitempty"`
	Status JobStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// JobList contains a list of Job
type JobList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []Job `json:"items"`
}

func init() {
	SchemeBuilder.Register(&Job{}, &JobList{})
}
