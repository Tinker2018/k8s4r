package plugin

import (
	"context"
	"encoding/json"

	"github.com/hashicorp/go-hclog"
)

// Plugin 代表一个系统级插件（如 SPIRE、Envoy 等）
// 插件在 Agent 启动时初始化，Agent 停止时清理
type Plugin interface {
	// Name 返回插件名称
	Name() string

	// Version 返回插件版本
	Version() string

	// Initialize 初始化插件
	// 在 Agent 启动时调用，此时还没有 TaskGroup 运行
	Initialize(ctx context.Context, config Config) error

	// Start 启动插件
	// 在 Initialize 成功后调用
	Start(ctx context.Context) error

	// Stop 停止插件
	// 在 Agent 停止时调用，应该优雅关闭所有资源
	Stop(ctx context.Context) error

	// HealthCheck 健康检查
	// 定期调用以确保插件正常运行
	HealthCheck(ctx context.Context) error

	// GetRuntimeConfig 获取运行时配置
	// 返回可以传递给 Task 的配置信息（如 socket 路径、环境变量等）
	GetRuntimeConfig() (map[string]string, error)
}

// Config 插件配置
type Config struct {
	// Name 插件名称
	Name string `json:"name"`

	// Enabled 是否启用
	Enabled bool `json:"enabled"`

	// BaseDir 插件工作目录
	BaseDir string `json:"baseDir"`

	// Logger 日志记录器
	Logger hclog.Logger `json:"-"`

	// Metadata 元数据
	Metadata map[string]string `json:"metadata,omitempty"`

	// PluginConfig 插件特定配置（JSON 格式）
	PluginConfig json.RawMessage `json:"config,omitempty"`
}

// Status 插件状态
type Status struct {
	// Name 插件名称
	Name string

	// Running 是否正在运行
	Running bool

	// Healthy 是否健康
	Healthy bool

	// Message 状态消息
	Message string

	// RuntimeConfig 运行时配置
	RuntimeConfig map[string]string
}
