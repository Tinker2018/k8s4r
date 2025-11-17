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

// RobotSpec defines the desired state of Robot
type RobotSpec struct {
	// RobotID 是机器人的唯一标识符
	// +kubebuilder:validation:Required
	RobotID string `json:"robotId"`

	// Description 是机器人的描述信息
	// +optional
	Description string `json:"description,omitempty"`

	// Labels 是机器人的标签
	// +optional
	Labels map[string]string `json:"labels,omitempty"`
}

// RobotStatus defines the observed state of Robot
type RobotStatus struct {
	// Phase 表示机器人的当前状态
	// +optional
	Phase RobotPhase `json:"phase,omitempty"`

	// LastHeartbeatTime 是最后一次心跳的时间
	// +optional
	LastHeartbeatTime *metav1.Time `json:"lastHeartbeatTime,omitempty"`

	// Message 是状态的详细信息
	// +optional
	Message string `json:"message,omitempty"`

	// DeviceInfo 是设备的硬件信息
	// +optional
	DeviceInfo *DeviceInfo `json:"deviceInfo,omitempty"`
}

// DeviceInfo 包含设备的硬件和系统信息（类似 psutil）
type DeviceInfo struct {
	// Hostname 主机名
	Hostname string `json:"hostname,omitempty"`

	// Platform 平台信息
	Platform PlatformInfo `json:"platform,omitempty"`

	// CPU CPU 信息
	CPU CPUInfo `json:"cpu,omitempty"`

	// Memory 内存信息
	Memory MemoryInfo `json:"memory,omitempty"`

	// Swap 交换分区信息
	Swap SwapInfo `json:"swap,omitempty"`

	// Disks 磁盘分区信息列表
	Disks []DiskInfo `json:"disks,omitempty"`

	// NetworkInterfaces 网卡信息列表
	NetworkInterfaces []NetworkInterface `json:"networkInterfaces,omitempty"`

	// BootTime 系统启动时间
	BootTime *metav1.Time `json:"bootTime,omitempty"`

	// Uptime 系统运行时间（秒）
	Uptime int64 `json:"uptime,omitempty"`

	// LastUpdated 信息最后更新时间
	LastUpdated *metav1.Time `json:"lastUpdated,omitempty"`
}

// PlatformInfo 平台和系统信息
type PlatformInfo struct {
	// OS 操作系统类型 (linux, darwin, windows)
	OS string `json:"os,omitempty"`

	// Arch 架构 (amd64, arm64, i386)
	Arch string `json:"arch,omitempty"`

	// Platform 平台名称 (ubuntu, centos, darwin)
	Platform string `json:"platform,omitempty"`

	// PlatformFamily 平台家族 (debian, rhel, darwin)
	PlatformFamily string `json:"platformFamily,omitempty"`

	// PlatformVersion 平台版本
	PlatformVersion string `json:"platformVersion,omitempty"`

	// KernelVersion 内核版本
	KernelVersion string `json:"kernelVersion,omitempty"`

	// KernelArch 内核架构
	KernelArch string `json:"kernelArch,omitempty"`
}

// CPUInfo CPU 信息
type CPUInfo struct {
	// PhysicalCores 物理核心数
	PhysicalCores int `json:"physicalCores,omitempty"`

	// LogicalCores 逻辑核心数（含超线程）
	LogicalCores int `json:"logicalCores,omitempty"`

	// ModelName CPU 型号
	ModelName string `json:"modelName,omitempty"`

	// Vendor CPU 厂商
	Vendor string `json:"vendor,omitempty"`

	// Mhz CPU 频率
	Mhz float64 `json:"mhz,omitempty"`

	// UsagePercent CPU 使用率（0-100）
	UsagePercent float64 `json:"usagePercent,omitempty"`

	// PerCPUUsage 每个 CPU 核心的使用率
	PerCPUUsage []float64 `json:"perCpuUsage,omitempty"`
}

// MemoryInfo 内存信息
type MemoryInfo struct {
	// Total 总内存（字节）
	Total uint64 `json:"total,omitempty"`

	// Available 可用内存（字节）
	Available uint64 `json:"available,omitempty"`

	// Used 已使用内存（字节）
	Used uint64 `json:"used,omitempty"`

	// Free 空闲内存（字节）
	Free uint64 `json:"free,omitempty"`

	// UsagePercent 内存使用率（0-100）
	UsagePercent float64 `json:"usagePercent,omitempty"`

	// Buffers 缓冲区内存（字节，Linux）
	Buffers uint64 `json:"buffers,omitempty"`

	// Cached 缓存内存（字节，Linux）
	Cached uint64 `json:"cached,omitempty"`
}

// SwapInfo 交换分区信息
type SwapInfo struct {
	// Total 总交换空间（字节）
	Total uint64 `json:"total,omitempty"`

	// Used 已使用交换空间（字节）
	Used uint64 `json:"used,omitempty"`

	// Free 空闲交换空间（字节）
	Free uint64 `json:"free,omitempty"`

	// UsagePercent 交换空间使用率（0-100）
	UsagePercent float64 `json:"usagePercent,omitempty"`
}

// DiskInfo 磁盘分区信息
type DiskInfo struct {
	// Device 设备名称 (/dev/sda1)
	Device string `json:"device,omitempty"`

	// MountPoint 挂载点 (/, /home)
	MountPoint string `json:"mountPoint,omitempty"`

	// FSType 文件系统类型 (ext4, xfs, apfs, ntfs)
	FSType string `json:"fsType,omitempty"`

	// Total 总容量（字节）
	Total uint64 `json:"total,omitempty"`

	// Used 已使用容量（字节）
	Used uint64 `json:"used,omitempty"`

	// Free 空闲容量（字节）
	Free uint64 `json:"free,omitempty"`

	// UsagePercent 使用率（0-100）
	UsagePercent float64 `json:"usagePercent,omitempty"`

	// InodesTotal 总 inode 数
	InodesTotal uint64 `json:"inodesTotal,omitempty"`

	// InodesUsed 已使用 inode 数
	InodesUsed uint64 `json:"inodesUsed,omitempty"`

	// InodesFree 空闲 inode 数
	InodesFree uint64 `json:"inodesFree,omitempty"`
}

// NetworkInterface 网络接口信息
type NetworkInterface struct {
	// Name 接口名称 (eth0, en0, wlan0)
	Name string `json:"name,omitempty"`

	// HardwareAddr MAC 地址
	HardwareAddr string `json:"hardwareAddr,omitempty"`

	// Addresses IP 地址列表（包含子网掩码，如 192.168.1.100/24）
	Addresses []string `json:"addresses,omitempty"`

	// MTU 最大传输单元
	MTU int `json:"mtu,omitempty"`

	// Flags 接口标志 (UP, BROADCAST, MULTICAST)
	Flags []string `json:"flags,omitempty"`

	// BytesSent 发送字节数
	BytesSent uint64 `json:"bytesSent,omitempty"`

	// BytesRecv 接收字节数
	BytesRecv uint64 `json:"bytesRecv,omitempty"`

	// PacketsSent 发送包数
	PacketsSent uint64 `json:"packetsSent,omitempty"`

	// PacketsRecv 接收包数
	PacketsRecv uint64 `json:"packetsRecv,omitempty"`

	// Errin 接收错误数
	Errin uint64 `json:"errin,omitempty"`

	// Errout 发送错误数
	Errout uint64 `json:"errout,omitempty"`

	// Dropin 接收丢包数
	Dropin uint64 `json:"dropin,omitempty"`

	// Dropout 发送丢包数
	Dropout uint64 `json:"dropout,omitempty"`
}

// RobotPhase 定义机器人的生命周期阶段
// +kubebuilder:validation:Enum=Pending;Online;Offline;Unknown
type RobotPhase string

const (
	// RobotPhasePending 表示机器人刚创建，等待 Agent 注册
	RobotPhasePending RobotPhase = "Pending"
	// RobotPhaseOnline 表示机器人在线
	RobotPhaseOnline RobotPhase = "Online"
	// RobotPhaseOffline 表示机器人离线（心跳超时）
	RobotPhaseOffline RobotPhase = "Offline"
	// RobotPhaseUnknown 表示机器人状态未知
	RobotPhaseUnknown RobotPhase = "Unknown"
)

// +kubebuilder:object:root=true
// +kubebuilder:subresource:status
// +kubebuilder:resource:scope=Namespaced,shortName=rb
// +kubebuilder:printcolumn:name="RobotID",type=string,JSONPath=`.spec.robotId`
// +kubebuilder:printcolumn:name="Phase",type=string,JSONPath=`.status.phase`
// +kubebuilder:printcolumn:name="Hostname",type=string,JSONPath=`.status.deviceInfo.hostname`
// +kubebuilder:printcolumn:name="OS",type=string,JSONPath=`.status.deviceInfo.platform.os`
// +kubebuilder:printcolumn:name="CPUs",type=integer,JSONPath=`.status.deviceInfo.cpu.logicalCores`
// +kubebuilder:printcolumn:name="LastHeartbeat",type=date,JSONPath=`.status.lastHeartbeatTime`
// +kubebuilder:printcolumn:name="Age",type=date,JSONPath=`.metadata.creationTimestamp`

// Robot is the Schema for the robots API
type Robot struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec   RobotSpec   `json:"spec,omitempty"`
	Status RobotStatus `json:"status,omitempty"`
}

// +kubebuilder:object:root=true

// RobotList contains a list of Robot
type RobotList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []Robot `json:"items"`
}

func init() {
	SchemeBuilder.Register(&Robot{}, &RobotList{})
}
