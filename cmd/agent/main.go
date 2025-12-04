/*
Copyright 2025.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0
*/

package main

import (
	"context"
	"crypto/tls"
	"crypto/x509"
	"encoding/json"
	"flag"
	"fmt"
	"log"
	"os"
	"os/signal"
	"regexp"
	"syscall"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	"github.com/hashicorp/go-hclog"
	"gopkg.in/yaml.v2"

	robotv1alpha1 "github.com/hxndghxndg/k8s4r/api/v1alpha1"
	"github.com/hxndghxndg/k8s4r/pkg/agent"
	"github.com/hxndghxndg/k8s4r/pkg/collector"
	"github.com/hxndghxndg/k8s4r/pkg/plugin"
	spireplugin "github.com/hxndghxndg/k8s4r/pkg/plugin/spire"
)

// expandEnv 展开字符串中的环境变量（支持 ${VAR} 格式）
func expandEnv(s string) string {
	re := regexp.MustCompile(`\$\{([^}]+)\}`)
	return re.ReplaceAllStringFunc(s, func(match string) string {
		varName := match[2 : len(match)-1] // 去掉 ${ 和 }
		if val := os.Getenv(varName); val != "" {
			return val
		}
		return match // 如果环境变量不存在，保留原始字符串
	})
}

// RegisterRequest 是注册请求的结构
type RegisterRequest struct {
	RobotID    string                    `json:"robotId"`
	Token      string                    `json:"token"`
	DeviceInfo *robotv1alpha1.DeviceInfo `json:"deviceInfo,omitempty"`
}

// HeartbeatRequest 是心跳请求的结构
type HeartbeatRequest struct {
	RobotID    string                    `json:"robotId"`
	Token      string                    `json:"token"`
	DeviceInfo *robotv1alpha1.DeviceInfo `json:"deviceInfo,omitempty"`
}

// Response 是服务器响应的结构
type Response struct {
	Success bool   `json:"success"`
	Message string `json:"message"`
	RobotID string `json:"robotId,omitempty"`
}

// MQTT Topics (与server保持一致)
const (
	// 全局 Topics（Agent → Server）
	TopicRegister  = "k8s4r/register"
	TopicHeartbeat = "k8s4r/heartbeat"

	// 机器人专属 Topics（格式化字符串，需要填入robotId）
	TopicRobotResponse     = "k8s4r/robots/%s/response"       // 接收Server响应
	TopicRobotTaskDispatch = "k8s4r/robots/%s/tasks/dispatch" // 接收任务分发
	TopicRobotTaskState    = "k8s4r/robots/%s/tasks/state"    // 任务状态恢复（retained消息）
)

// AgentConfig Agent 配置文件结构
type AgentConfig struct {
	BrokerURL         string                 `yaml:"brokerUrl"`
	Token             string                 `yaml:"token"`
	RobotID           string                 `yaml:"robotId"`
	HeartbeatInterval int                    `yaml:"heartbeatInterval"` // 秒
	WorkDir           string                 `yaml:"workDir"`
	PluginsConfigPath string                 `yaml:"pluginsConfigPath"` // plugins.yaml 路径
	Plugins           map[string]interface{} `yaml:"plugins"`           // 内嵌的插件配置
}

// Agent 结构体
type Agent struct {
	BrokerURL         string
	Token             string
	RobotID           string
	HeartbeatInterval time.Duration
	WorkDir           string
	mqttClient        mqtt.Client
	responseChan      chan Response

	// Plugin 相关
	pluginManager *plugin.Manager
	spireAgent    *spireplugin.AgentManager
	logger        hclog.Logger
}

// NewAgent 创建一个新的 Agent 实例
func NewAgent(brokerURL, token, robotID string, heartbeatInterval time.Duration, workDir string, logger hclog.Logger) *Agent {
	return &Agent{
		BrokerURL:         brokerURL,
		Token:             token,
		RobotID:           robotID,
		HeartbeatInterval: heartbeatInterval,
		WorkDir:           workDir,
		responseChan:      make(chan Response, 10),
		logger:            logger,
	}
}

// InitPlugins 初始化所有插件
func (a *Agent) InitPlugins(ctx context.Context, pluginsConfigPath string) error {
	a.logger.Info("initializing plugins", "config", pluginsConfigPath)

	// 1. 创建 Plugin Manager
	a.pluginManager = plugin.NewManager(plugin.ManagerConfig{
		Logger:              a.logger.Named("plugin-manager"),
		HealthCheckInterval: 30 * time.Second,
	})

	// 2. 加载插件配置
	var pluginConfigs map[string]interface{}
	if pluginsConfigPath != "" {
		data, err := os.ReadFile(pluginsConfigPath)
		if err != nil {
			return fmt.Errorf("failed to read plugins config: %w", err)
		}

		// 展开环境变量
		configContent := expandEnv(string(data))

		a.logger.Debug("plugin config after env expansion", "content", configContent[:200])

		if err := yaml.Unmarshal([]byte(configContent), &pluginConfigs); err != nil {
			return fmt.Errorf("failed to parse plugins config: %w", err)
		}
	}

	// 3. 初始化 SPIRE Plugin（如果配置存在）
	if spireConfigData, ok := pluginConfigs["spire"]; ok {
		a.logger.Info("found SPIRE plugin configuration, initializing...")

		// 将 map 转换为 SPIRE Config
		spireConfigBytes, err := yaml.Marshal(spireConfigData)
		if err != nil {
			return fmt.Errorf("failed to marshal spire config: %w", err)
		}

		a.logger.Debug("spire config yaml", "yaml", string(spireConfigBytes))

		var spireConfig spireplugin.Config
		if err := yaml.Unmarshal(spireConfigBytes, &spireConfig); err != nil {
			return fmt.Errorf("failed to unmarshal spire config: %w", err)
		}

		a.logger.Debug("spire config struct",
			"socketPath", spireConfig.SocketPath,
			"trustDomain", spireConfig.TrustDomain,
			"serverAddr", spireConfig.ServerAddr,
			"joinToken", spireConfig.JoinToken,
		)

		// 验证配置
		if err := spireConfig.Validate(); err != nil {
			return fmt.Errorf("invalid spire config: %w", err)
		}

		a.logger.Info("SPIRE configuration loaded",
			"socketPath", spireConfig.SocketPath,
			"trustDomain", spireConfig.TrustDomain,
			"serverAddr", spireConfig.ServerAddr,
			"joinToken", spireConfig.JoinToken[:8]+"...",
		)

		// 创建 SPIRE Agent Manager（启动 spire-agent 进程）
		a.logger.Info("starting SPIRE agent process...")
		a.spireAgent = spireplugin.NewAgentManager(a.logger.Named("spire"), &spireConfig)

		if err := a.spireAgent.Start(ctx); err != nil {
			return fmt.Errorf("failed to start SPIRE agent: %w", err)
		}

		a.logger.Info("SPIRE agent started successfully")

		// 等待 SPIRE Agent 就绪（Workload API 可用）
		if err := a.spireAgent.WaitReady(ctx, 30*time.Second); err != nil {
			return fmt.Errorf("SPIRE agent not ready: %w", err)
		}

		a.logger.Info("SPIRE agent is ready, Workload API available")
	} else {
		a.logger.Warn("SPIRE plugin not configured, skipping")
	}

	return nil
}

// GetSPIRESVID 获取当前节点的 SPIRE SVID（供 MQTT 使用）
func (a *Agent) GetSPIRESVID(ctx context.Context) (*tls.Certificate, *x509.CertPool, error) {
	if a.spireAgent == nil {
		return nil, nil, fmt.Errorf("SPIRE agent not initialized")
	}

	a.logger.Info("========================================")
	a.logger.Info("🔐 Fetching Workload SVID from SPIRE...")
	a.logger.Info("========================================")

	// 从 Workload API 获取 SVID
	svid, err := a.spireAgent.FetchSVID(ctx)
	if err != nil {
		a.logger.Error("❌ Failed to fetch SVID from Workload API", "error", err)
		return nil, nil, fmt.Errorf("failed to fetch SVID: %w", err)
	}

	a.logger.Info("✅ Successfully fetched Workload SVID from SPIRE")
	a.logger.Info("📋 SVID Details:")
	a.logger.Info("  - SPIFFE ID: %s", svid.SpiffeID)

	// 解析证书以获取更多信息
	cert, err := tls.X509KeyPair(svid.X509Svid, svid.X509SvidKey)
	if err != nil {
		a.logger.Error("❌ Failed to create TLS certificate from SVID", "error", err)
		return nil, nil, fmt.Errorf("failed to create TLS certificate: %w", err)
	}

	// 显示证书详情
	if len(cert.Certificate) > 0 {
		x509Cert, err := x509.ParseCertificate(cert.Certificate[0])
		if err == nil {
			a.logger.Info("  - Subject: %s", x509Cert.Subject.String())
			a.logger.Info("  - Issuer: %s", x509Cert.Issuer.String())
			a.logger.Info("  - Valid From: %s", x509Cert.NotBefore.Format("2006-01-02 15:04:05"))
			a.logger.Info("  - Valid Until: %s", x509Cert.NotAfter.Format("2006-01-02 15:04:05"))
			a.logger.Info("  - Serial Number: %s", x509Cert.SerialNumber.String())
		}
	}

	// 创建 CA Pool
	caPool := x509.NewCertPool()
	if !caPool.AppendCertsFromPEM(svid.Bundle) {
		a.logger.Error("❌ Failed to append CA certificates to pool")
		return nil, nil, fmt.Errorf("failed to append CA certificates")
	}

	// 计算 CA Bundle 中的证书数量
	caCount := len(caPool.Subjects())

	a.logger.Info("  - CA Bundle: %d certificate(s) loaded", caCount)
	a.logger.Info("========================================")
	a.logger.Info("🎉 SVID ready for mTLS connection to MQTT")
	a.logger.Info("========================================")

	return &cert, caPool, nil
}

// Register 向服务器注册
func (a *Agent) Register() error {
	// 采集设备信息
	deviceInfo := collector.CollectDeviceInfo()

	req := RegisterRequest{
		RobotID:    a.RobotID,
		Token:      a.Token,
		DeviceInfo: deviceInfo,
	}

	payload, err := json.Marshal(req)
	if err != nil {
		return fmt.Errorf("failed to marshal register request: %w", err)
	}

	// 发布注册请求
	token := a.mqttClient.Publish(TopicRegister, 1, false, payload)
	if token.Wait() && token.Error() != nil {
		return fmt.Errorf("failed to publish register request: %w", token.Error())
	}

	// 等待响应（带超时）
	select {
	case response := <-a.responseChan:
		if !response.Success {
			return fmt.Errorf("registration failed: %s", response.Message)
		}
		log.Printf("Successfully registered robot: %s", a.RobotID)
		return nil
	case <-time.After(10 * time.Second):
		return fmt.Errorf("registration timeout")
	}
}

// SendHeartbeat 发送心跳
func (a *Agent) SendHeartbeat() error {
	// 采集设备信息
	deviceInfo := collector.CollectDeviceInfo()

	req := HeartbeatRequest{
		RobotID:    a.RobotID,
		Token:      a.Token,
		DeviceInfo: deviceInfo,
	}

	payload, err := json.Marshal(req)
	if err != nil {
		return fmt.Errorf("failed to marshal heartbeat request: %w", err)
	}

	// 发布心跳请求
	token := a.mqttClient.Publish(TopicHeartbeat, 1, false, payload)
	if token.Wait() && token.Error() != nil {
		return fmt.Errorf("failed to publish heartbeat: %w", token.Error())
	}

	log.Printf("Heartbeat sent successfully for robot: %s", a.RobotID)
	return nil
}

// setupMQTT 设置MQTT连接
func (a *Agent) setupMQTT(ctx context.Context) error {
	opts := mqtt.NewClientOptions()
	opts.AddBroker(a.BrokerURL)
	opts.SetClientID(fmt.Sprintf("k8s4r-agent-%s", a.RobotID))
	opts.SetKeepAlive(60 * time.Second)
	opts.SetPingTimeout(10 * time.Second)
	opts.SetCleanSession(true)
	opts.SetAutoReconnect(true)
	opts.SetMaxReconnectInterval(10 * time.Second)

	// 如果 SPIRE 已初始化，使用 mTLS
	if a.spireAgent != nil {
		a.logger.Info("configuring MQTT with SPIRE mTLS")

		cert, caPool, err := a.GetSPIRESVID(ctx)
		if err != nil {
			a.logger.Warn("failed to get SPIRE SVID, falling back to plain connection", "error", err)
		} else {
			tlsConfig := &tls.Config{
				Certificates: []tls.Certificate{*cert},
				RootCAs:      caPool,
				ClientCAs:    caPool,
				MinVersion:   tls.VersionTLS12,
			}
			opts.SetTLSConfig(tlsConfig)
			a.logger.Info("MQTT mTLS configured with SPIRE SVID")
		}
	}

	// 设置连接回调
	opts.SetOnConnectHandler(func(client mqtt.Client) {
		a.logger.Info("connected to MQTT broker", "broker", a.BrokerURL)

		// 订阅机器人专属的响应 topic
		responseTopic := fmt.Sprintf(TopicRobotResponse, a.RobotID)
		if token := client.Subscribe(responseTopic, 1, a.handleResponse); token.Wait() && token.Error() != nil {
			a.logger.Error("failed to subscribe to response topic", "error", token.Error())
		} else {
			a.logger.Info("subscribed to response topic", "topic", responseTopic)
		}

		// 注意：任务分发和状态恢复的订阅由 TaskExecutor 处理
	})

	opts.SetConnectionLostHandler(func(client mqtt.Client, err error) {
		a.logger.Warn("connection to MQTT broker lost", "error", err)
	})

	// 创建客户端并连接
	a.mqttClient = mqtt.NewClient(opts)
	if token := a.mqttClient.Connect(); token.Wait() && token.Error() != nil {
		return fmt.Errorf("failed to connect to MQTT broker: %w", token.Error())
	}

	return nil
}

// handleResponse 处理服务器响应
func (a *Agent) handleResponse(client mqtt.Client, msg mqtt.Message) {
	// 打印完整的 MQTT 消息
	log.Printf(" [MQTT] Received response message - topic: %s, payload: %s",
		msg.Topic(), string(msg.Payload()))

	var response Response
	if err := json.Unmarshal(msg.Payload(), &response); err != nil {
		log.Printf("Failed to unmarshal response: %v", err)
		return
	}

	log.Printf(" [MQTT] Parsed response: success=%v, message=%s", response.Success, response.Message)

	// 将响应发送到通道
	select {
	case a.responseChan <- response:
	default:
		log.Printf("Response channel is full, dropping response")
	}
}

// Run 运行 Agent
func (a *Agent) Run() {
	a.logger.Info("starting k8s4r agent", "robotId", a.RobotID, "broker", a.BrokerURL)

	ctx := context.Background()

	// 确保退出时清理 SPIRE Agent
	defer func() {
		if a.spireAgent != nil {
			a.logger.Info("stopping SPIRE agent...")
			if err := a.spireAgent.Stop(ctx); err != nil {
				a.logger.Error("failed to stop SPIRE agent", "error", err)
			} else {
				a.logger.Info("SPIRE agent stopped successfully")
			}
		}
	}()

	// 设置MQTT连接（可能使用 SPIRE mTLS）
	if err := a.setupMQTT(ctx); err != nil {
		a.logger.Error("failed to setup MQTT", "error", err)
		log.Fatalf("Failed to setup MQTT: %v", err)
	}
	defer a.mqttClient.Disconnect(250)

	// 首先注册
	for {
		if err := a.Register(); err != nil {
			a.logger.Warn("failed to register, retrying", "error", err)
			time.Sleep(5 * time.Second)
			continue
		}
		break
	}

	// 创建并启动 TaskGroupManager（管理所有 TaskGroup）
	taskGroupManager := agent.NewTaskGroupManager(a.RobotID, a.mqttClient, a.WorkDir, nil)
	if err := taskGroupManager.Start(ctx); err != nil {
		a.logger.Error("failed to start taskgroup manager", "error", err)
		log.Fatalf("Failed to start taskgroup manager: %v", err)
	}
	defer taskGroupManager.Stop(ctx)

	a.logger.Info("taskgroup manager started successfully")

	// 启动心跳循环
	ticker := time.NewTicker(a.HeartbeatInterval)
	defer ticker.Stop()

	// 设置信号处理
	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, syscall.SIGINT, syscall.SIGTERM)

	a.logger.Info("agent started", "heartbeatInterval", a.HeartbeatInterval)

	for {
		select {
		case <-ticker.C:
			if err := a.SendHeartbeat(); err != nil {
				log.Printf("Failed to send heartbeat: %v", err)
			}
		case sig := <-sigChan:
			log.Printf("Received signal %v, shutting down...", sig)
			return
		}
	}
}

func main() {
	var (
		brokerURL         string
		token             string
		robotID           string
		heartbeatInterval int
		workDir           string
		pluginsConfig     string
		logLevel          string
	)

	flag.StringVar(&brokerURL, "broker-url", "tcp://localhost:1883", "The MQTT broker URL")
	flag.StringVar(&token, "token", "fixed-token-123", "The authentication token")
	flag.StringVar(&robotID, "robot-id", "", "The unique ID of this robot (required)")
	flag.IntVar(&heartbeatInterval, "heartbeat-interval", 30, "Heartbeat interval in seconds")
	flag.StringVar(&workDir, "work-dir", "", "Working directory for tasks (default: $HOME/.k8s4r/tasks)")
	flag.StringVar(&pluginsConfig, "plugins-config", "", "Path to plugins configuration file (e.g., config/agent/plugins.yaml)")
	flag.StringVar(&logLevel, "log-level", "info", "Log level (trace, debug, info, warn, error)")
	flag.Parse()

	if robotID == "" {
		log.Fatal("robot-id is required")
	}

	// 如果未指定工作目录，使用用户主目录
	if workDir == "" {
		homeDir, err := os.UserHomeDir()
		if err != nil {
			log.Fatalf("Failed to get user home directory: %v", err)
		}
		workDir = fmt.Sprintf("%s/.k8s4r/tasks", homeDir)
	}

	// 创建 logger
	logger := hclog.New(&hclog.LoggerOptions{
		Name:   "k8s4r-agent",
		Level:  hclog.LevelFromString(logLevel),
		Output: os.Stdout,
		Color:  hclog.AutoColor,
	})

	agent := NewAgent(
		brokerURL,
		token,
		robotID,
		time.Duration(heartbeatInterval)*time.Second,
		workDir,
		logger,
	)

	// 如果提供了插件配置，初始化插件
	if pluginsConfig != "" {
		ctx := context.Background()
		if err := agent.InitPlugins(ctx, pluginsConfig); err != nil {
			logger.Error("failed to initialize plugins", "error", err)
			// 如果使用 ssl:// 协议，SPIRE 插件是必需的
			if brokerURL[:5] == "ssl://" || brokerURL[:6] == "tls://" {
				log.Fatalf("SPIRE plugin initialization failed but SSL/TLS broker URL is specified: %v", err)
			}
			logger.Warn("continuing without plugins")
		}
	}

	agent.Run()
}
