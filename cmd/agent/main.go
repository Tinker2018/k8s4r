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
	"encoding/json"
	"flag"
	"fmt"
	"log"
	"os"
	"os/signal"
	"syscall"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	robotv1alpha1 "github.com/hxndghxndg/k8s4r/api/v1alpha1"
	"github.com/hxndghxndg/k8s4r/pkg/agent"
	"github.com/hxndghxndg/k8s4r/pkg/collector"
)

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

// Agent 结构体
type Agent struct {
	BrokerURL         string
	Token             string
	RobotID           string
	HeartbeatInterval time.Duration
	WorkDir           string
	mqttClient        mqtt.Client
	responseChan      chan Response
}

// NewAgent 创建一个新的 Agent 实例
func NewAgent(brokerURL, token, robotID string, heartbeatInterval time.Duration, workDir string) *Agent {
	return &Agent{
		BrokerURL:         brokerURL,
		Token:             token,
		RobotID:           robotID,
		HeartbeatInterval: heartbeatInterval,
		WorkDir:           workDir,
		responseChan:      make(chan Response, 10),
	}
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
func (a *Agent) setupMQTT() error {
	opts := mqtt.NewClientOptions()
	opts.AddBroker(a.BrokerURL)
	opts.SetClientID(fmt.Sprintf("k8s4r-agent-%s", a.RobotID))
	opts.SetKeepAlive(60 * time.Second)
	opts.SetPingTimeout(10 * time.Second)
	opts.SetCleanSession(true)
	opts.SetAutoReconnect(true)
	opts.SetMaxReconnectInterval(10 * time.Second)

	// 设置连接回调
	opts.SetOnConnectHandler(func(client mqtt.Client) {
		log.Printf("Connected to MQTT broker: %s", a.BrokerURL)

		// 订阅机器人专属的响应 topic
		responseTopic := fmt.Sprintf(TopicRobotResponse, a.RobotID)
		if token := client.Subscribe(responseTopic, 1, a.handleResponse); token.Wait() && token.Error() != nil {
			log.Printf("Failed to subscribe to response topic: %v", token.Error())
		} else {
			log.Printf("Subscribed to response topic: %s", responseTopic)
		}

		// 注意：任务分发和状态恢复的订阅由 TaskExecutor 处理
	})

	opts.SetConnectionLostHandler(func(client mqtt.Client, err error) {
		log.Printf("Connection to MQTT broker lost: %v", err)
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
	log.Printf("Starting agent for robot: %s", a.RobotID)
	log.Printf("MQTT Broker: %s", a.BrokerURL)

	// 设置MQTT连接
	if err := a.setupMQTT(); err != nil {
		log.Fatalf("Failed to setup MQTT: %v", err)
	}
	defer a.mqttClient.Disconnect(250)

	// 首先注册
	for {
		if err := a.Register(); err != nil {
			log.Printf("Failed to register: %v, retrying in 5 seconds...", err)
			time.Sleep(5 * time.Second)
			continue
		}
		break
	}

	// 创建并启动 TaskGroupManager（管理所有 TaskGroup）
	ctx := context.Background()
	taskGroupManager := agent.NewTaskGroupManager(a.RobotID, a.mqttClient, a.WorkDir, nil)
	if err := taskGroupManager.Start(ctx); err != nil {
		log.Fatalf("Failed to start taskgroup manager: %v", err)
	}
	defer taskGroupManager.Stop(ctx)

	log.Printf("TaskGroup manager started successfully")

	// 启动心跳循环
	ticker := time.NewTicker(a.HeartbeatInterval)
	defer ticker.Stop()

	// 设置信号处理
	sigChan := make(chan os.Signal, 1)
	signal.Notify(sigChan, syscall.SIGINT, syscall.SIGTERM)

	log.Printf("Agent started, sending heartbeat every %v", a.HeartbeatInterval)

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
	)

	flag.StringVar(&brokerURL, "broker-url", "tcp://localhost:1883", "The MQTT broker URL")
	flag.StringVar(&token, "token", "fixed-token-123", "The authentication token")
	flag.StringVar(&robotID, "robot-id", "", "The unique ID of this robot (required)")
	flag.IntVar(&heartbeatInterval, "heartbeat-interval", 30, "Heartbeat interval in seconds")
	flag.StringVar(&workDir, "work-dir", "", "Working directory for tasks (default: $HOME/.k8s4r/tasks)")
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

	agent := NewAgent(
		brokerURL,
		token,
		robotID,
		time.Duration(heartbeatInterval)*time.Second,
		workDir,
	)

	agent.Run()
}
