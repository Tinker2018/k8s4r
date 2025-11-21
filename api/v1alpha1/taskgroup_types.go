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

// TaskGroupSpec defines the desired state of TaskGroup
type TaskGroupSpec struct {
	// JobName 所属的 Job 名称
	// +kubebuilder:validation:Required
	JobName string `json:"jobName"`

	// GroupName TaskGroup 在 Job 中的名称
	// +kubebuilder:validation:Required
	GroupName string `json:"groupName"`

	// Count 需要运行的实例数量
	// +kubebuilder:validation:Required
	// +kubebuilder:validation:Minimum=1
	Count int32 `json:"count"`

	// Tasks 任务定义列表（从 Job 的 TaskGroupTemplate 复制）
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

// TaskGroupStatus defines the observed state of TaskGroup
type TaskGroupStatus struct {
	// State TaskGroup 的当前状态
	// +optional
	State TaskGroupState `json:"state,omitempty"`

	// StatusDescription 状态描述
	// +optional
	StatusDescription string `json:"statusDescription,omitempty"`

	// TotalTasks 总任务数
	// +optional
	TotalTasks int32 `json:"totalTasks,omitempty"`

	// PendingTasks 等待中的任务数
	// +optional
	PendingTasks int32 `json:"pendingTasks,omitempty"`

	// RunningTasks 运行中的任务数
	// +optional
	RunningTasks int32 `json:"runningTasks,omitempty"`

	// SucceededTasks 成功的任务数
	// +optional
	SucceededTasks int32 `json:"succeededTasks,omitempty"`

	// FailedTasks 失败的任务数
	// +optional
	FailedTasks int32 `json:"failedTasks,omitempty"`

	// StartedAt 开始时间
	// +optional
	StartedAt *metav1.Time `json:"startedAt,omitempty"`

	// CompletedAt 完成时间
	// +optional
	CompletedAt *metav1.Time `json:"completedAt,omitempty"`
}

// TaskGroupState 定义 TaskGroup 的状态
type TaskGroupState string

const (
	// TaskGroupStatePending TaskGroup 等待中
	TaskGroupStatePending TaskGroupState = "pending"

	// TaskGroupStateRunning TaskGroup 运行中
	TaskGroupStateRunning TaskGroupState = "running"

	// TaskGroupStateCompleted TaskGroup 已完成
	TaskGroupStateCompleted TaskGroupState = "completed"

	// TaskGroupStateFailed TaskGroup 失败
	TaskGroupStateFailed TaskGroupState = "failed"
)

//+kubebuilder:object:root=true
//+kubebuilder:subresource:status
//+kubebuilder:resource:shortName=rtg
//+kubebuilder:printcolumn:name="Job",type=string,JSONPath=`.spec.jobName`
//+kubebuilder:printcolumn:name="State",type=string,JSONPath=`.status.state`
//+kubebuilder:printcolumn:name="Total",type=integer,JSONPath=`.status.totalTasks`
//+kubebuilder:printcolumn:name="Running",type=integer,JSONPath=`.status.runningTasks`
//+kubebuilder:printcolumn:name="Succeeded",type=integer,JSONPath=`.status.succeededTasks`
//+kubebuilder:printcolumn:name="Failed",type=integer,JSONPath=`.status.failedTasks`
//+kubebuilder:printcolumn:name="Age",type=date,JSONPath=`.metadata.creationTimestamp`

// TaskGroup is the Schema for the taskgroups API
type TaskGroup struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec   TaskGroupSpec   `json:"spec,omitempty"`
	Status TaskGroupStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// TaskGroupList contains a list of TaskGroup
type TaskGroupList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []TaskGroup `json:"items"`
}

func init() {
	SchemeBuilder.Register(&TaskGroup{}, &TaskGroupList{})
}
