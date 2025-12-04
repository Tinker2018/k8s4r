package spire

import (
	"context"
	"encoding/json"
	"fmt"

	"github.com/hashicorp/go-hclog"
	"github.com/hxndghxndg/k8s4r/pkg/plugin"
)

const (
	PluginName    = "spire"
	PluginVersion = "v1.0.0"
)

// Plugin SPIRE 插件实现
type Plugin struct {
	logger hclog.Logger
	config *Config

	// 组件
	agentManager   *AgentManager
	keyringManager *KeyringManager

	// 状态
	initialized bool
	running     bool
}

// NewPlugin 创建 SPIRE 插件
func NewPlugin() *Plugin {
	return &Plugin{}
}

// Name 返回插件名称
func (p *Plugin) Name() string {
	return PluginName
}

// Version 返回插件版本
func (p *Plugin) Version() string {
	return PluginVersion
}

// Initialize 初始化插件
func (p *Plugin) Initialize(ctx context.Context, cfg plugin.Config) error {
	if p.initialized {
		return fmt.Errorf("plugin already initialized")
	}

	p.logger = cfg.Logger.Named(PluginName)
	p.logger.Info("initializing SPIRE plugin")

	// 解析 SPIRE 配置
	p.config = &Config{}
	if err := json.Unmarshal(cfg.PluginConfig, p.config); err != nil {
		return fmt.Errorf("failed to parse config: %w", err)
	}

	// 验证配置
	if err := p.config.Validate(); err != nil {
		return fmt.Errorf("invalid config: %w", err)
	}

	// 如果未指定 DataDir，使用 BaseDir
	if p.config.DataDir == "" {
		p.config.DataDir = cfg.BaseDir + "/spire"
	}

	p.logger.Info("SPIRE config loaded",
		"serverAddress", p.config.ServerAddress,
		"trustDomain", p.config.TrustDomain,
		"socketPath", p.config.SocketPath,
		"dataDir", p.config.DataDir)

	// 初始化 Agent Manager
	p.agentManager = NewAgentManager(p.logger, p.config)

	// 如果启用了 Keyring，初始化 Keyring Manager
	if p.config.Keyring.Enabled {
		p.logger.Info("initializing keyring",
			"type", p.config.Keyring.Type)

		keyringMgr, err := NewKeyringManager(p.logger, &p.config.Keyring)
		if err != nil {
			return fmt.Errorf("failed to initialize keyring: %w", err)
		}
		p.keyringManager = keyringMgr

		p.logger.Info("keyring initialized",
			"keyringID", keyringMgr.GetKeyringID())
	}

	p.initialized = true
	p.logger.Info("SPIRE plugin initialized successfully")

	return nil
}

// Start 启动插件
func (p *Plugin) Start(ctx context.Context) error {
	if !p.initialized {
		return fmt.Errorf("plugin not initialized")
	}

	if p.running {
		return fmt.Errorf("plugin already running")
	}

	p.logger.Info("starting SPIRE plugin")

	// 启动 SPIRE Agent
	if err := p.agentManager.Start(ctx); err != nil {
		return fmt.Errorf("failed to start agent: %w", err)
	}

	p.running = true
	p.logger.Info("SPIRE plugin started successfully")

	return nil
}

// Stop 停止插件
func (p *Plugin) Stop(ctx context.Context) error {
	if !p.running {
		return nil
	}

	p.logger.Info("stopping SPIRE plugin")

	// 停止 SPIRE Agent
	if err := p.agentManager.Stop(ctx); err != nil {
		p.logger.Error("failed to stop agent", "error", err)
		// 继续清理
	}

	// 清理 Keyring（可选）
	if p.keyringManager != nil {
		p.logger.Debug("keyring cleanup skipped (keys preserved for next start)")
		// 注意：我们不清空 keyring，因为密钥可能在下次启动时还需要使用
		// 如果需要清理，取消注释下面的代码：
		// if err := p.keyringManager.Clear(); err != nil {
		//     p.logger.Warn("failed to clear keyring", "error", err)
		// }
	}

	p.running = false
	p.logger.Info("SPIRE plugin stopped")

	return nil
}

// HealthCheck 健康检查
func (p *Plugin) HealthCheck(ctx context.Context) error {
	if !p.running {
		return fmt.Errorf("plugin not running")
	}

	// 检查 Agent 健康状态
	if err := p.agentManager.HealthCheck(ctx); err != nil {
		return fmt.Errorf("agent unhealthy: %w", err)
	}

	return nil
}

// GetRuntimeConfig 获取运行时配置
func (p *Plugin) GetRuntimeConfig() (map[string]string, error) {
	if !p.running {
		return nil, fmt.Errorf("plugin not running")
	}

	config := make(map[string]string)

	// Workload API socket 路径
	config["SPIRE_SOCKET_PATH"] = p.config.SocketPath
	config["SPIRE_AGENT_ADDRESS"] = "unix://" + p.config.SocketPath

	// Trust Domain
	config["SPIRE_TRUST_DOMAIN"] = p.config.TrustDomain

	// Agent PID（用于调试）
	config["SPIRE_AGENT_PID"] = fmt.Sprintf("%d", p.agentManager.GetPID())

	return config, nil
}

// GetAgentManager 获取 Agent Manager（用于高级操作）
func (p *Plugin) GetAgentManager() *AgentManager {
	return p.agentManager
}

// GetKeyringManager 获取 Keyring Manager（用于高级操作）
func (p *Plugin) GetKeyringManager() *KeyringManager {
	return p.keyringManager
}

// IsRunning 检查插件是否正在运行
func (p *Plugin) IsRunning() bool {
	return p.running
}
