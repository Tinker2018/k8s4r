package spire

import (
	"context"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"syscall"
	"time"

	"github.com/hashicorp/go-hclog"
	"github.com/hxndghxndg/k8s4r/pkg/plugin"
)

// AgentManager 管理 SPIRE Agent 进程
type AgentManager struct {
	logger hclog.Logger
	config *Config

	// 进程管理
	cmd     *exec.Cmd
	pid     int
	running bool

	// 配置文件路径
	configFile string
}

// NewAgentManager 创建 AgentManager
func NewAgentManager(logger hclog.Logger, config *Config) *AgentManager {
	return &AgentManager{
		logger: logger,
		config: config,
	}
}

// Start 启动 SPIRE Agent
func (m *AgentManager) Start(ctx context.Context) error {
	if m.running {
		return fmt.Errorf("agent already running")
	}

	m.logger.Info("========================================")
	m.logger.Info("🚀 Starting SPIRE Agent Process")
	m.logger.Info("========================================")

	// 1. 检查是否需要安装 SPIRE
	agentBinary, err := m.ensureBinary(ctx)
	if err != nil {
		return fmt.Errorf("failed to ensure spire-agent binary: %w", err)
	}
	m.logger.Info("✅ Found spire-agent binary", "path", agentBinary)

	// 2. 创建配置目录
	configDir := filepath.Join(m.config.DataDir, "config")
	if err := os.MkdirAll(configDir, 0755); err != nil {
		return fmt.Errorf("failed to create config dir: %w", err)
	}

	// 3. 检查是否是首次启动（DataDir 中没有 SVID）
	svidPath := filepath.Join(m.config.DataDir, "agent_svid.der")
	isFirstStart := true
	if _, err := os.Stat(svidPath); err == nil {
		isFirstStart = false
		m.logger.Info("📋 Found existing SVID, this is not first start")
		m.logger.Info("  - SVID will be automatically renewed from DataDir")
		m.logger.Info("  - Join token will be ignored")
	} else {
		m.logger.Info("📋 No existing SVID found, this is first start")
		m.logger.Info("  - Will use join token for initial registration")
	}

	// 4. 生成配置文件
	m.configFile = filepath.Join(configDir, "agent.conf")
	configContent := m.config.ToAgentConfig(isFirstStart)
	if err := os.WriteFile(m.configFile, []byte(configContent), 0644); err != nil {
		return fmt.Errorf("failed to write config file: %w", err)
	}
	m.logger.Info("✅ Generated agent config", "file", m.configFile)
	m.logger.Info("📋 Agent Configuration:")
	m.logger.Info("  - Server: %s", m.config.ServerAddr)
	m.logger.Info("  - Trust Domain: %s", m.config.TrustDomain)
	m.logger.Info("  - Socket: %s", m.config.SocketPath)
	if isFirstStart {
		m.logger.Info("  - Join Token: %s...", m.config.JoinToken[:16])
	} else {
		m.logger.Info("  - Authentication: Using existing SVID from DataDir")
	}

	// 5. 创建必要的目录
	if err := m.prepareDirs(); err != nil {
		return fmt.Errorf("failed to prepare directories: %w", err)
	}

	// 6. 启动 spire-agent 进程
	args := []string{"run", "-config", m.configFile}

	// 首次启动时，添加 -joinToken 参数
	if isFirstStart && m.config.JoinToken != "" {
		args = append(args, "-joinToken", m.config.JoinToken)
		m.logger.Info("  - Using join token from config", "tokenPrefix", m.config.JoinToken[:16]+"...")
	}

	m.cmd = exec.CommandContext(ctx, agentBinary, args...)

	// 设置环境变量
	m.cmd.Env = os.Environ()

	// 设置进程组（方便后续清理）
	m.cmd.SysProcAttr = &syscall.SysProcAttr{
		Setpgid: true,
	}

	// 配置日志输出
	logFile, err := os.OpenFile(
		filepath.Join(m.config.DataDir, "agent.log"),
		os.O_CREATE|os.O_WRONLY|os.O_APPEND,
		0644,
	)
	if err != nil {
		return fmt.Errorf("failed to open log file: %w", err)
	}
	m.cmd.Stdout = logFile
	m.cmd.Stderr = logFile

	m.logger.Info("🔧 Starting spire-agent process...")

	// 启动进程
	if err := m.cmd.Start(); err != nil {
		logFile.Close()
		return fmt.Errorf("failed to start agent: %w", err)
	}

	m.pid = m.cmd.Process.Pid
	m.running = true

	m.logger.Info("✅ SPIRE agent process started", "pid", m.pid)
	m.logger.Info("📝 Agent logs: %s", filepath.Join(m.config.DataDir, "agent.log"))

	// 等待 socket 文件创建
	m.logger.Info("⏳ Waiting for Workload API socket...")
	if err := m.waitForSocket(ctx); err != nil {
		m.Stop(ctx)
		return fmt.Errorf("agent started but socket not available: %w", err)
	}

	m.logger.Info("✅ Workload API socket ready", "socket", m.config.SocketPath)
	m.logger.Info("========================================")
	m.logger.Info("🎉 SPIRE Agent fully initialized")
	m.logger.Info("========================================")

	return nil
}

// Stop 停止 SPIRE Agent
func (m *AgentManager) Stop(ctx context.Context) error {
	if !m.running {
		return nil
	}

	m.logger.Info("stopping SPIRE agent", "pid", m.pid)

	// 1. 发送 SIGTERM 信号
	if err := m.cmd.Process.Signal(syscall.SIGTERM); err != nil {
		m.logger.Warn("failed to send SIGTERM", "error", err)
	}

	// 2. 等待进程退出（最多 10 秒）
	done := make(chan error, 1)
	go func() {
		done <- m.cmd.Wait()
	}()

	select {
	case err := <-done:
		if err != nil {
			m.logger.Warn("agent exited with error", "error", err)
		} else {
			m.logger.Info("agent stopped gracefully")
		}
	case <-time.After(10 * time.Second):
		// 3. 超时则强制杀死
		m.logger.Warn("agent did not stop gracefully, killing")
		if err := m.cmd.Process.Kill(); err != nil {
			m.logger.Error("failed to kill agent", "error", err)
		}
		<-done // 等待 Wait() 返回
	}

	m.running = false
	m.pid = 0

	// 4. 清理 socket 文件
	if err := os.Remove(m.config.SocketPath); err != nil && !os.IsNotExist(err) {
		m.logger.Warn("failed to remove socket file", "error", err)
	}

	return nil
}

// HealthCheck 健康检查
func (m *AgentManager) HealthCheck(ctx context.Context) error {
	if !m.running {
		return fmt.Errorf("agent not running")
	}

	// 检查进程是否存在
	if err := m.cmd.Process.Signal(syscall.Signal(0)); err != nil {
		return fmt.Errorf("agent process dead: %w", err)
	}

	// 检查 socket 文件是否存在
	if _, err := os.Stat(m.config.SocketPath); err != nil {
		return fmt.Errorf("socket file not available: %w", err)
	}

	return nil
}

// IsRunning 检查是否正在运行
func (m *AgentManager) IsRunning() bool {
	return m.running
}

// GetPID 获取进程 ID
func (m *AgentManager) GetPID() int {
	return m.pid
}

// ensureBinary 确保 SPIRE Agent 二进制文件存在（必要时安装）
func (m *AgentManager) ensureBinary(ctx context.Context) (string, error) {
	installer := plugin.NewBinaryInstaller(m.logger, nil)

	// 定义 fallback 路径
	fallbackPaths := []string{
		"/usr/local/bin/spire-agent",
		"/usr/bin/spire-agent",
		"/opt/spire/bin/spire-agent",
		"/opt/k8s4r/bin/spire-agent",
	}

	// 尝试从 PATH 查找
	if binary, err := exec.LookPath("spire-agent"); err == nil {
		fallbackPaths = append([]string{binary}, fallbackPaths...)
	}

	return installer.EnsureBinary(ctx, &m.config.Binary, fallbackPaths)
}

// findAgentBinary 查找 spire-agent 二进制文件（已废弃，使用 ensureBinary）
func (m *AgentManager) findAgentBinary() (string, error) {
	return m.ensureBinary(context.Background())
}

// prepareDirs 准备必要的目录
func (m *AgentManager) prepareDirs() error {
	dirs := []string{
		m.config.DataDir,
		filepath.Join(m.config.DataDir, "data"),
		filepath.Join(m.config.DataDir, "data/keys"),
		filepath.Dir(m.config.SocketPath),
	}

	for _, dir := range dirs {
		if err := os.MkdirAll(dir, 0755); err != nil {
			return fmt.Errorf("failed to create directory %s: %w", dir, err)
		}
	}

	return nil
}

// waitForSocket 等待 socket 文件创建
func (m *AgentManager) waitForSocket(ctx context.Context) error {
	timeout := time.After(30 * time.Second)
	ticker := time.NewTicker(500 * time.Millisecond)
	defer ticker.Stop()

	for {
		select {
		case <-timeout:
			return fmt.Errorf("timeout waiting for socket file")
		case <-ctx.Done():
			return ctx.Err()
		case <-ticker.C:
			if _, err := os.Stat(m.config.SocketPath); err == nil {
				return nil
			}
		}
	}
}

// WaitReady 等待 SPIRE Agent 就绪（可以响应 Workload API 请求）
func (m *AgentManager) WaitReady(ctx context.Context, timeout time.Duration) error {
	m.logger.Info("waiting for SPIRE agent to be ready", "timeout", timeout)

	ctx, cancel := context.WithTimeout(ctx, timeout)
	defer cancel()

	// 首先等待 socket 文件创建
	if err := m.waitForSocket(ctx); err != nil {
		return fmt.Errorf("socket not created: %w", err)
	}

	// 尝试连接并获取 SVID 来验证 Agent 是否就绪
	ticker := time.NewTicker(500 * time.Millisecond)
	defer ticker.Stop()

	for {
		select {
		case <-ctx.Done():
			return fmt.Errorf("timeout waiting for SPIRE agent to be ready: %w", ctx.Err())
		case <-ticker.C:
			// 尝试获取 SVID
			_, err := m.FetchSVID(ctx)
			if err == nil {
				m.logger.Info("SPIRE agent is ready")
				return nil
			}
			m.logger.Debug("SPIRE agent not ready yet", "error", err)
		}
	}
}

// FetchSVID 从 Workload API 获取 X.509 SVID
func (m *AgentManager) FetchSVID(ctx context.Context) (*WorkloadSVID, error) {
	return FetchX509SVID(ctx, m.config.SocketPath)
}
