/*
Copyright 2025.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0
*/

package v1alpha1

import (
	"k8s.io/apimachinery/pkg/api/resource"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
)

// TaskSpec defines the desired state of Task
// Task 是在机器人上执行的具体任务（类比 Pod）
// 由 Job 的 Controller 创建
type TaskSpec struct {
	// JobName 所属 Job 名称
	// +optional
	JobName string `json:"jobName,omitempty"`

	// TaskGroupName 所属 TaskGroup 名称
	// +optional
	TaskGroupName string `json:"taskGroupName,omitempty"`

	// Name 是任务的名称
	// +kubebuilder:validation:Required
	Name string `json:"name"`

	// Driver 指定任务驱动程序类型
	// +kubebuilder:validation:Required
	// +kubebuilder:validation:Enum=exec;docker;java;raw_exec
	Driver TaskDriverType `json:"driver"`

	// Config 是驱动程序的特定配置
	// +kubebuilder:validation:Required
	Config TaskDriverConfig `json:"config"`

	// TargetRobot 目标执行的机器人（由 Controller 调度分配）
	// +optional
	TargetRobot string `json:"targetRobot,omitempty"`

	// Resources 定义任务所需的资源
	// +optional
	Resources *TaskResources `json:"resources,omitempty"`

	// Constraints 定义任务的约束条件
	// +optional
	Constraints []TaskConstraint `json:"constraints,omitempty"`

	// Env 环境变量
	// +optional
	Env map[string]string `json:"env,omitempty"`

	// User 指定运行任务的用户
	// +optional
	User string `json:"user,omitempty"`

	// Timeout 指定任务执行的总超时时间（从启动到完成）
	// 超过此时间任务将被强制终止
	// +optional
	Timeout *metav1.Duration `json:"timeout,omitempty"`

	// KillTimeout 指定强制终止任务的超时时间
	// +optional
	KillTimeout *metav1.Duration `json:"killTimeout,omitempty"`

	// RestartPolicy 重启策略
	// +optional
	RestartPolicy *TaskRestartPolicy `json:"restartPolicy,omitempty"`

	// Artifacts 需要下载的文件
	// +optional
	Artifacts []TaskArtifact `json:"artifacts,omitempty"`

	// Templates 配置模板
	// +optional
	Templates []TaskTemplate `json:"templates,omitempty"`

	// Services 服务定义
	// +optional
	Services []TaskService `json:"services,omitempty"`

	// Network 网络代理配置
	// +optional
	Network *NetworkProxy `json:"network,omitempty"`

	// InitTasks 初始化任务列表（从 TaskGroup 复制）
	// +optional
	InitTasks []TaskDefinition `json:"initTasks,omitempty"`
}

// TaskDriverType 定义支持的任务驱动程序类型
type TaskDriverType string

const (
	// TaskDriverExec 原生可执行文件驱动
	TaskDriverExec TaskDriverType = "exec"

	// TaskDriverDocker Docker 容器驱动
	TaskDriverDocker TaskDriverType = "docker"

	// TaskDriverJava Java 应用驱动
	TaskDriverJava TaskDriverType = "java"

	// TaskDriverRawExec 原始可执行文件驱动（无隔离）
	TaskDriverRawExec TaskDriverType = "raw_exec"
)

// TaskDriverConfig 驱动程序配置
type TaskDriverConfig struct {
	// ExecConfig 用于 exec 和 raw_exec 驱动
	// +optional
	ExecConfig *ExecDriverConfig `json:"execConfig,omitempty"`

	// DockerConfig 用于 docker 驱动
	// +optional
	DockerConfig *DockerDriverConfig `json:"dockerConfig,omitempty"`

	// JavaConfig 用于 java 驱动
	// +optional
	JavaConfig *JavaDriverConfig `json:"javaConfig,omitempty"`
}

// ExecDriverConfig exec 驱动配置
type ExecDriverConfig struct {
	// Command 要执行的命令
	// +kubebuilder:validation:Required
	Command string `json:"command"`

	// Args 命令参数
	// +optional
	Args []string `json:"args,omitempty"`
}

// DockerDriverConfig docker 驱动配置
type DockerDriverConfig struct {
	// Image Docker 镜像
	// +kubebuilder:validation:Required
	Image string `json:"image"`

	// Command 容器启动命令
	// +optional
	Command []string `json:"command,omitempty"`

	// Args 命令参数
	// +optional
	Args []string `json:"args,omitempty"`

	// WorkDir 工作目录
	// +optional
	WorkDir string `json:"workDir,omitempty"`

	// Ports 端口映射
	// +optional
	Ports []DockerPortMapping `json:"ports,omitempty"`

	// Volumes 卷挂载
	// +optional
	Volumes []DockerVolumeMount `json:"volumes,omitempty"`

	// NetworkMode 网络模式
	// +optional
	NetworkMode string `json:"networkMode,omitempty"`
}

// DockerPortMapping Docker 端口映射
type DockerPortMapping struct {
	// Label 端口标签
	Label string `json:"label"`

	// Value 端口号
	Value int32 `json:"value"`

	// To 映射到的主机端口（可选）
	// +optional
	To *int32 `json:"to,omitempty"`

	// HostIP 绑定的主机IP
	// +optional
	HostIP string `json:"hostIP,omitempty"`
}

// DockerVolumeMount Docker 卷挂载
type DockerVolumeMount struct {
	// Target 容器内目标路径
	Target string `json:"target"`

	// Source 主机源路径
	Source string `json:"source"`

	// ReadOnly 是否只读
	// +optional
	ReadOnly bool `json:"readOnly,omitempty"`
}

// JavaDriverConfig Java 驱动配置
type JavaDriverConfig struct {
	// JarPath JAR 文件路径
	// +kubebuilder:validation:Required
	JarPath string `json:"jarPath"`

	// JvmOptions JVM 选项
	// +optional
	JvmOptions []string `json:"jvmOptions,omitempty"`

	// Args 应用程序参数
	// +optional
	Args []string `json:"args,omitempty"`

	// Class 主类名（如果不是可执行 JAR）
	// +optional
	Class string `json:"class,omitempty"`

	// ClassPath 类路径
	// +optional
	ClassPath []string `json:"classpath,omitempty"`
}

// TaskResources 任务资源定义
type TaskResources struct {
	// CPU CPU 资源请求和限制
	// +optional
	CPU *resource.Quantity `json:"cpu,omitempty"`

	// Memory 内存资源请求和限制
	// +optional
	Memory *resource.Quantity `json:"memory,omitempty"`

	// Disk 磁盘空间需求
	// +optional
	Disk *resource.Quantity `json:"disk,omitempty"`

	// IOPS 磁盘 IOPS 需求
	// +optional
	IOPS *int64 `json:"iops,omitempty"`

	// Networks 网络资源需求
	// +optional
	Networks []TaskNetworkResource `json:"networks,omitempty"`
}

// TaskNetworkResource 网络资源定义
type TaskNetworkResource struct {
	// MBits 网络带宽需求 (Mbps)
	// +optional
	MBits *int32 `json:"mbits,omitempty"`

	// ReservedPorts 保留端口
	// +optional
	ReservedPorts []TaskPort `json:"reservedPorts,omitempty"`

	// DynamicPorts 动态端口
	// +optional
	DynamicPorts []TaskPort `json:"dynamicPorts,omitempty"`
}

// TaskPort 端口定义
type TaskPort struct {
	// Label 端口标签
	Label string `json:"label"`

	// Value 端口值（对于保留端口）
	// +optional
	Value *int32 `json:"value,omitempty"`

	// To 映射到的端口（对于动态端口）
	// +optional
	To *int32 `json:"to,omitempty"`
}

// TaskConstraint 任务约束
type TaskConstraint struct {
	// LTarget 约束目标（如 ${node.class}）
	LTarget string `json:"ltarget"`

	// RTarget 约束值
	RTarget string `json:"rtarget"`

	// Operand 操作符
	// +kubebuilder:validation:Enum=="=";"!=";">";">=";"<";"<=";"regexp";"set_contains";"version"
	Operand string `json:"operand"`
}

// TaskRestartPolicy 重启策略
type TaskRestartPolicy struct {
	// Attempts 最大重试次数
	// +optional
	Attempts *int32 `json:"attempts,omitempty"`

	// Interval 重试间隔时间窗口
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

// TaskArtifact 任务工件
type TaskArtifact struct {
	// GetterSource 下载源（支持 http, git, s3 等）
	// +kubebuilder:validation:Required
	GetterSource string `json:"getterSource"`

	// GetterOptions 下载选项
	// +optional
	GetterOptions map[string]string `json:"getterOptions,omitempty"`

	// RelativeDest 相对目标路径
	// +optional
	RelativeDest string `json:"relativeDest,omitempty"`

	// GetterMode 下载模式
	// +kubebuilder:validation:Enum=any;file;dir
	// +optional
	GetterMode string `json:"getterMode,omitempty"`
}

// TaskTemplate 配置模板
type TaskTemplate struct {
	// SourcePath 模板源路径
	// +optional
	SourcePath string `json:"sourcePath,omitempty"`

	// EmbeddedTmpl 内嵌模板内容
	// +optional
	EmbeddedTmpl string `json:"embeddedTmpl,omitempty"`

	// DestPath 目标文件路径
	// +kubebuilder:validation:Required
	DestPath string `json:"destPath"`

	// ChangeMode 变更模式
	// +kubebuilder:validation:Enum=restart;noop;signal
	// +optional
	ChangeMode string `json:"changeMode,omitempty"`

	// ChangeSignal 变更信号
	// +optional
	ChangeSignal string `json:"changeSignal,omitempty"`

	// Perms 文件权限
	// +optional
	Perms string `json:"perms,omitempty"`

	// LeftDelim 左分隔符
	// +optional
	LeftDelim string `json:"leftDelim,omitempty"`

	// RightDelim 右分隔符
	// +optional
	RightDelim string `json:"rightDelim,omitempty"`
}

// TaskService 服务定义
type TaskService struct {
	// Name 服务名称
	// +kubebuilder:validation:Required
	Name string `json:"name"`

	// PortLabel 端口标签
	// +kubebuilder:validation:Required
	PortLabel string `json:"portLabel"`

	// Tags 服务标签
	// +optional
	Tags []string `json:"tags,omitempty"`

	// Checks 健康检查
	// +optional
	Checks []TaskServiceCheck `json:"checks,omitempty"`
}

// TaskServiceCheck 服务健康检查
type TaskServiceCheck struct {
	// Name 检查名称
	Name string `json:"name"`

	// Type 检查类型
	// +kubebuilder:validation:Enum=http;tcp;script;grpc
	Type string `json:"type"`

	// Command 检查命令（script 类型）
	// +optional
	Command string `json:"command,omitempty"`

	// Args 命令参数
	// +optional
	Args []string `json:"args,omitempty"`

	// Path HTTP 路径（http 类型）
	// +optional
	Path string `json:"path,omitempty"`

	// Protocol 协议（http/https）
	// +optional
	Protocol string `json:"protocol,omitempty"`

	// PortLabel 端口标签
	// +optional
	PortLabel string `json:"portLabel,omitempty"`

	// Interval 检查间隔
	// +optional
	Interval *metav1.Duration `json:"interval,omitempty"`

	// Timeout 检查超时
	// +optional
	Timeout *metav1.Duration `json:"timeout,omitempty"`

	// InitialStatus 初始状态
	// +kubebuilder:validation:Enum=critical;warning;passing
	// +optional
	InitialStatus string `json:"initialStatus,omitempty"`
}

// TaskStatus defines the observed state of Task
type TaskStatus struct {
	// State 任务当前状态
	// +optional
	State TaskState `json:"state,omitempty"`

	// StartedAt 任务开始时间
	// +optional
	StartedAt *metav1.Time `json:"startedAt,omitempty"`

	// FinishedAt 任务结束时间
	// +optional
	FinishedAt *metav1.Time `json:"finishedAt,omitempty"`

	// RestartCount 重启次数
	// +optional
	RestartCount int32 `json:"restartCount,omitempty"`

	// LastRestartTime 最后重启时间
	// +optional
	LastRestartTime *metav1.Time `json:"lastRestartTime,omitempty"`

	// Message 状态描述信息
	// +optional
	Message string `json:"message,omitempty"`

	// Reason 状态原因
	// +optional
	Reason string `json:"reason,omitempty"`

	// ExitCode 退出码
	// +optional
	ExitCode *int32 `json:"exitCode,omitempty"`

	// Signal 接收到的信号
	// +optional
	Signal *int32 `json:"signal,omitempty"`

	// Logs 任务执行日志（仅保存最后 10KB）
	// Agent 在任务完成后会上报日志内容到这里
	// 用户可以通过 kubectl get task <name> -o jsonpath='{.status.logs}' 查看
	// +optional
	Logs string `json:"logs,omitempty"`

	// LogsStdout 标准输出日志（最后 5KB）
	// +optional
	LogsStdout string `json:"logsStdout,omitempty"`

	// LogsStderr 标准错误日志（最后 5KB）
	// +optional
	LogsStderr string `json:"logsStderr,omitempty"`

	// Events 任务事件列表
	// +optional
	Events []TaskEvent `json:"events,omitempty"`

	// AllocatedResources 分配的资源
	// +optional
	AllocatedResources *TaskAllocatedResources `json:"allocatedResources,omitempty"`
}

// TaskState 任务状态枚举
type TaskState string

const (
	// TaskStatePending 等待调度（Task刚创建）
	TaskStatePending TaskState = "pending"

	// TaskStateScheduled Server已发布到MQTT，等待Agent执行
	TaskStateScheduled TaskState = "scheduled"

	// TaskStateRunning Agent正在执行
	TaskStateRunning TaskState = "running"

	// TaskStateExited 已退出（无论 exitCode 是否为 0）
	// 由 Agent 上报，Controller 会根据 exitCode 转换为 completed 或 failed
	TaskStateExited TaskState = "exited"

	// TaskStateCompleted 成功完成（exitCode = 0）
	TaskStateCompleted TaskState = "completed"

	// TaskStateFailed 执行失败（exitCode != 0 或其他错误）
	TaskStateFailed TaskState = "failed"
)

// TaskEvent 任务事件
type TaskEvent struct {
	// Type 事件类型
	Type string `json:"type"`

	// Time 事件时间
	Time metav1.Time `json:"time"`

	// DisplayMessage 显示消息
	// +optional
	DisplayMessage string `json:"displayMessage,omitempty"`

	// Details 事件详情
	// +optional
	Details map[string]string `json:"details,omitempty"`
}

// TaskAllocatedResources 已分配的资源
type TaskAllocatedResources struct {
	// CPU 分配的 CPU
	// +optional
	CPU *resource.Quantity `json:"cpu,omitempty"`

	// Memory 分配的内存
	// +optional
	Memory *resource.Quantity `json:"memory,omitempty"`

	// Disk 分配的磁盘
	// +optional
	Disk *resource.Quantity `json:"disk,omitempty"`

	// Networks 分配的网络资源
	// +optional
	Networks []TaskAllocatedNetworkResource `json:"networks,omitempty"`
}

// TaskAllocatedNetworkResource 已分配的网络资源
type TaskAllocatedNetworkResource struct {
	// Interface 网络接口
	Interface string `json:"interface"`

	// IP 分配的IP地址
	IP string `json:"ip"`

	// ReservedPorts 保留端口
	// +optional
	ReservedPorts []TaskAllocatedPort `json:"reservedPorts,omitempty"`

	// DynamicPorts 动态端口
	// +optional
	DynamicPorts []TaskAllocatedPort `json:"dynamicPorts,omitempty"`
}

// TaskAllocatedPort 已分配的端口
type TaskAllocatedPort struct {
	// Label 端口标签
	Label string `json:"label"`

	// Value 端口值
	Value int32 `json:"value"`

	// To 映射端口
	// +optional
	To *int32 `json:"to,omitempty"`

	// HostIP 主机IP
	// +optional
	HostIP string `json:"hostIP,omitempty"`
}

// NetworkProxy 网络代理配置
type NetworkProxy struct {
	// Enabled 是否启用网络代理
	// +optional
	Enabled bool `json:"enabled,omitempty"`

	// Type 代理类型 (envoy/none)
	// +optional
	// +kubebuilder:validation:Enum=envoy;none
	// +kubebuilder:default=envoy
	Type string `json:"type,omitempty"`

	// SPIFFE SPIFFE/SPIRE 配置
	// +optional
	SPIFFE *SPIFFEConfig `json:"spiffe,omitempty"`

	// Upstreams 上游服务配置列表
	// +optional
	Upstreams []ProxyUpstream `json:"upstreams,omitempty"`
}

// SPIFFEConfig SPIFFE/SPIRE 配置
type SPIFFEConfig struct {
	// TrustDomain SPIFFE 信任域
	// +kubebuilder:validation:Required
	TrustDomain string `json:"trustDomain"`

	// AgentSocketPath SPIRE Agent Unix Socket 路径
	// +optional
	// +kubebuilder:default=/run/spire/sockets/agent.sock
	AgentSocketPath string `json:"agentSocketPath,omitempty"`
}

// ProxyUpstream 上游服务配置
type ProxyUpstream struct {
	// Name 服务名称（用于标识）
	// +kubebuilder:validation:Required
	Name string `json:"name"`

	// LocalPort 本地监听端口（业务进程连接此端口）
	// +kubebuilder:validation:Required
	// +kubebuilder:validation:Minimum=1
	// +kubebuilder:validation:Maximum=65535
	LocalPort int32 `json:"localPort"`

	// Upstream 上游服务地址 (host:port)
	// +kubebuilder:validation:Required
	Upstream string `json:"upstream"`

	// Protocol 协议类型
	// +optional
	// +kubebuilder:validation:Enum=tcp;http;grpc
	// +kubebuilder:default=tcp
	Protocol string `json:"protocol,omitempty"`

	// TLS 是否使用 TLS
	// +optional
	// +kubebuilder:default=true
	TLS bool `json:"tls,omitempty"`
}

//+kubebuilder:object:root=true
//+kubebuilder:subresource:status
//+kubebuilder:resource:scope=Namespaced,shortName=rtask
//+kubebuilder:printcolumn:name="Job",type=string,JSONPath=`.spec.jobName`,description="Job name"
//+kubebuilder:printcolumn:name="Robot",type=string,JSONPath=`.spec.targetRobot`,description="Target robot"
//+kubebuilder:printcolumn:name="Driver",type=string,JSONPath=`.spec.driver`
//+kubebuilder:printcolumn:name="State",type=string,JSONPath=`.status.state`
//+kubebuilder:printcolumn:name="ExitCode",type=integer,JSONPath=`.status.exitCode`,description="Exit code"
//+kubebuilder:printcolumn:name="Restarts",type=integer,JSONPath=`.status.restartCount`
//+kubebuilder:printcolumn:name="Age",type=date,JSONPath=`.metadata.creationTimestamp`

// Task is the Schema for the tasks API
// Task 是 Nomad 风格的任务资源，代表最小的工作单元
type Task struct {
	metav1.TypeMeta   `json:",inline"`
	metav1.ObjectMeta `json:"metadata,omitempty"`

	Spec   TaskSpec   `json:"spec,omitempty"`
	Status TaskStatus `json:"status,omitempty"`
}

//+kubebuilder:object:root=true

// TaskList contains a list of Task
type TaskList struct {
	metav1.TypeMeta `json:",inline"`
	metav1.ListMeta `json:"metadata,omitempty"`
	Items           []Task `json:"items"`
}

func init() {
	SchemeBuilder.Register(&Task{}, &TaskList{})
}
