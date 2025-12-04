package plugin

import (
	"context"
	"fmt"
	"sync"
	"time"

	"github.com/hashicorp/go-hclog"
)

// Manager 管理所有系统级插件
type Manager struct {
	logger  hclog.Logger
	plugins map[string]Plugin
	status  map[string]*Status
	mu      sync.RWMutex

	// 健康检查
	healthCheckInterval time.Duration
	healthCheckCancel   context.CancelFunc
}

// ManagerConfig PluginManager 配置
type ManagerConfig struct {
	Logger              hclog.Logger
	HealthCheckInterval time.Duration
}

// NewManager 创建 PluginManager
func NewManager(config ManagerConfig) *Manager {
	if config.HealthCheckInterval == 0 {
		config.HealthCheckInterval = 30 * time.Second
	}

	return &Manager{
		logger:              config.Logger,
		plugins:             make(map[string]Plugin),
		status:              make(map[string]*Status),
		healthCheckInterval: config.HealthCheckInterval,
	}
}

// Register 注册插件
func (m *Manager) Register(plugin Plugin) error {
	m.mu.Lock()
	defer m.mu.Unlock()

	name := plugin.Name()
	if _, exists := m.plugins[name]; exists {
		return fmt.Errorf("plugin %s already registered", name)
	}

	m.plugins[name] = plugin
	m.status[name] = &Status{
		Name:    name,
		Running: false,
		Healthy: false,
	}

	m.logger.Info("plugin registered", "name", name, "version", plugin.Version())
	return nil
}

// Initialize 初始化所有插件
func (m *Manager) Initialize(ctx context.Context, configs map[string]Config) error {
	m.logger.Info("initializing plugins", "count", len(configs))

	for name, config := range configs {
		if !config.Enabled {
			m.logger.Debug("plugin disabled, skipping", "name", name)
			continue
		}

		plugin, exists := m.plugins[name]
		if !exists {
			m.logger.Warn("plugin not registered, skipping", "name", name)
			continue
		}

		m.logger.Info("initializing plugin", "name", name)
		if err := plugin.Initialize(ctx, config); err != nil {
			return fmt.Errorf("failed to initialize plugin %s: %w", name, err)
		}

		m.updateStatus(name, func(s *Status) {
			s.Message = "initialized"
		})
	}

	return nil
}

// Start 启动所有已初始化的插件
func (m *Manager) Start(ctx context.Context) error {
	m.logger.Info("starting plugins")

	m.mu.RLock()
	plugins := make([]Plugin, 0, len(m.plugins))
	for _, plugin := range m.plugins {
		plugins = append(plugins, plugin)
	}
	m.mu.RUnlock()

	for _, plugin := range plugins {
		name := plugin.Name()
		m.logger.Info("starting plugin", "name", name)

		if err := plugin.Start(ctx); err != nil {
			return fmt.Errorf("failed to start plugin %s: %w", name, err)
		}

		m.updateStatus(name, func(s *Status) {
			s.Running = true
			s.Healthy = true
			s.Message = "running"
		})

		// 获取运行时配置
		if runtimeConfig, err := plugin.GetRuntimeConfig(); err == nil {
			m.updateStatus(name, func(s *Status) {
				s.RuntimeConfig = runtimeConfig
			})
		}
	}

	// 启动健康检查
	m.startHealthCheck(ctx)

	return nil
}

// Stop 停止所有插件
func (m *Manager) Stop(ctx context.Context) error {
	m.logger.Info("stopping plugins")

	// 停止健康检查
	if m.healthCheckCancel != nil {
		m.healthCheckCancel()
	}

	m.mu.RLock()
	plugins := make([]Plugin, 0, len(m.plugins))
	for _, plugin := range m.plugins {
		plugins = append(plugins, plugin)
	}
	m.mu.RUnlock()

	// 反向停止插件（LIFO）
	for i := len(plugins) - 1; i >= 0; i-- {
		plugin := plugins[i]
		name := plugin.Name()

		m.logger.Info("stopping plugin", "name", name)

		if err := plugin.Stop(ctx); err != nil {
			m.logger.Error("failed to stop plugin", "name", name, "error", err)
			// 继续停止其他插件
		}

		m.updateStatus(name, func(s *Status) {
			s.Running = false
			s.Healthy = false
			s.Message = "stopped"
		})
	}

	return nil
}

// GetPlugin 获取插件实例
func (m *Manager) GetPlugin(name string) (Plugin, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()

	plugin, exists := m.plugins[name]
	if !exists {
		return nil, fmt.Errorf("plugin %s not found", name)
	}

	return plugin, nil
}

// GetStatus 获取插件状态
func (m *Manager) GetStatus(name string) (*Status, error) {
	m.mu.RLock()
	defer m.mu.RUnlock()

	status, exists := m.status[name]
	if !exists {
		return nil, fmt.Errorf("plugin %s not found", name)
	}

	// 返回副本
	statusCopy := *status
	return &statusCopy, nil
}

// GetAllStatus 获取所有插件状态
func (m *Manager) GetAllStatus() map[string]*Status {
	m.mu.RLock()
	defer m.mu.RUnlock()

	result := make(map[string]*Status, len(m.status))
	for name, status := range m.status {
		statusCopy := *status
		result[name] = &statusCopy
	}

	return result
}

// GetRuntimeConfig 获取插件的运行时配置
func (m *Manager) GetRuntimeConfig(name string) (map[string]string, error) {
	m.mu.RLock()
	status, exists := m.status[name]
	m.mu.RUnlock()

	if !exists {
		return nil, fmt.Errorf("plugin %s not found", name)
	}

	if status.RuntimeConfig == nil {
		return make(map[string]string), nil
	}

	// 返回副本
	result := make(map[string]string, len(status.RuntimeConfig))
	for k, v := range status.RuntimeConfig {
		result[k] = v
	}

	return result, nil
}

// startHealthCheck 启动健康检查
func (m *Manager) startHealthCheck(ctx context.Context) {
	healthCtx, cancel := context.WithCancel(ctx)
	m.healthCheckCancel = cancel

	go func() {
		ticker := time.NewTicker(m.healthCheckInterval)
		defer ticker.Stop()

		for {
			select {
			case <-healthCtx.Done():
				return
			case <-ticker.C:
				m.performHealthCheck(healthCtx)
			}
		}
	}()
}

// performHealthCheck 执行健康检查
func (m *Manager) performHealthCheck(ctx context.Context) {
	m.mu.RLock()
	plugins := make([]Plugin, 0, len(m.plugins))
	for _, plugin := range m.plugins {
		plugins = append(plugins, plugin)
	}
	m.mu.RUnlock()

	for _, plugin := range plugins {
		name := plugin.Name()

		err := plugin.HealthCheck(ctx)
		healthy := err == nil

		m.updateStatus(name, func(s *Status) {
			s.Healthy = healthy
			if err != nil {
				s.Message = fmt.Sprintf("unhealthy: %v", err)
				m.logger.Warn("plugin health check failed", "name", name, "error", err)
			} else {
				s.Message = "healthy"
			}
		})
	}
}

// updateStatus 更新插件状态（线程安全）
func (m *Manager) updateStatus(name string, fn func(*Status)) {
	m.mu.Lock()
	defer m.mu.Unlock()

	if status, exists := m.status[name]; exists {
		fn(status)
	}
}
