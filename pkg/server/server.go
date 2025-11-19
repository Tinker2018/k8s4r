/*
Copyright 2025.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0
*/

package server

import (
	"context"
	"encoding/json"
	"fmt"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/log"

	robotv1alpha1 "github.com/hxndg/k8s4r/api/v1alpha1"
)

const (
	// FixedToken 是用于 Agent 认证的固定 Token
	FixedToken = "fixed-token-123"

	// MQTT Topics
	TopicRegister  = "k8s4r/register"
	TopicHeartbeat = "k8s4r/heartbeat"
	TopicResponse  = "k8s4r/response"
	TopicCommands  = "k8s4r/commands"
)

// Server 是 MQTT Server 的主结构
type Server struct {
	Client     client.Client
	Namespace  string
	mqttClient mqtt.Client
	ctx        context.Context
	cancel     context.CancelFunc
}

// RegisterRequest 是 Agent 注册请求的结构
type RegisterRequest struct {
	RobotID    string                    `json:"robotId"`
	Token      string                    `json:"token"`
	DeviceInfo *robotv1alpha1.DeviceInfo `json:"deviceInfo,omitempty"`
}

// HeartbeatRequest 是 Agent 心跳请求的结构
type HeartbeatRequest struct {
	RobotID    string                    `json:"robotId"`
	Token      string                    `json:"token"`
	DeviceInfo *robotv1alpha1.DeviceInfo `json:"deviceInfo,omitempty"`
}

// Response 是通用响应结构
type Response struct {
	Success bool   `json:"success"`
	Message string `json:"message"`
	RobotID string `json:"robotId,omitempty"`
}

// NewServer 创建一个新的 Server 实例
func NewServer(client client.Client, namespace string) *Server {
	ctx, cancel := context.WithCancel(context.Background())
	return &Server{
		Client:    client,
		Namespace: namespace,
		ctx:       ctx,
		cancel:    cancel,
	}
}

// RegisterHandler 处理 Agent 注册请求
func (s *Server) RegisterHandler(client mqtt.Client, msg mqtt.Message) {
	logger := log.FromContext(s.ctx)

	var req RegisterRequest
	if err := json.Unmarshal(msg.Payload(), &req); err != nil {
		logger.Error(err, "Failed to decode register request")
		s.sendMQTTResponse(req.RobotID, false, "Invalid request body")
		return
	}

	// 验证 Token
	if req.Token != FixedToken {
		logger.Info("Invalid token provided", "robotId", req.RobotID)
		s.sendMQTTResponse(req.RobotID, false, "Invalid token")
		return
	}

	// 检查 Robot 资源是否存在
	robot := &robotv1alpha1.Robot{}
	err := s.Client.Get(s.ctx, types.NamespacedName{
		Name:      req.RobotID,
		Namespace: s.Namespace,
	}, robot)

	if err != nil {
		// Robot 不存在，创建新的 Robot 资源
		robot = &robotv1alpha1.Robot{
			ObjectMeta: metav1.ObjectMeta{
				Name:      req.RobotID,
				Namespace: s.Namespace,
			},
			Spec: robotv1alpha1.RobotSpec{
				RobotID:     req.RobotID,
				Description: "Registered by agent",
			},
		}

		if err := s.Client.Create(s.ctx, robot); err != nil {
			logger.Error(err, "Failed to create Robot resource", "robotId", req.RobotID)
			s.sendMQTTResponse(req.RobotID, false, "Failed to register robot")
			return
		}

		logger.Info("Created new Robot resource", "robotId", req.RobotID)
	}

	// 更新状态
	now := metav1.Now()
	robot.Status.Phase = robotv1alpha1.RobotPhaseOnline
	robot.Status.LastHeartbeatTime = &now
	robot.Status.Message = "Agent registered successfully"
	robot.Status.DeviceInfo = req.DeviceInfo

	if err := s.Client.Status().Update(s.ctx, robot); err != nil {
		logger.Error(err, "Failed to update Robot status", "robotId", req.RobotID)
		s.sendMQTTResponse(req.RobotID, false, "Failed to update robot status")
		return
	}

	logger.Info("Robot registered successfully", "robotId", req.RobotID)
	s.sendMQTTResponse(req.RobotID, true, "Robot registered successfully")
}

// HeartbeatHandler 处理 Agent 心跳请求
func (s *Server) HeartbeatHandler(client mqtt.Client, msg mqtt.Message) {
	logger := log.FromContext(s.ctx)

	var req HeartbeatRequest
	if err := json.Unmarshal(msg.Payload(), &req); err != nil {
		logger.Error(err, "Failed to decode heartbeat request")
		s.sendMQTTResponse(req.RobotID, false, "Invalid request body")
		return
	}

	// 验证 Token
	if req.Token != FixedToken {
		logger.Info("Invalid token provided", "robotId", req.RobotID)
		s.sendMQTTResponse(req.RobotID, false, "Invalid token")
		return
	}

	// 获取 Robot 资源
	robot := &robotv1alpha1.Robot{}
	err := s.Client.Get(s.ctx, types.NamespacedName{
		Name:      req.RobotID,
		Namespace: s.Namespace,
	}, robot)

	if err != nil {
		logger.Error(err, "Robot not found", "robotId", req.RobotID)
		s.sendMQTTResponse(req.RobotID, false, "Robot not registered")
		return
	}

	// 更新心跳时间和设备信息
	now := metav1.Now()
	robot.Status.LastHeartbeatTime = &now
	robot.Status.Phase = robotv1alpha1.RobotPhaseOnline
	robot.Status.Message = "Heartbeat received"

	// 如果提供了设备信息，则更新
	if req.DeviceInfo != nil {
		robot.Status.DeviceInfo = req.DeviceInfo
	}

	if err := s.Client.Status().Update(s.ctx, robot); err != nil {
		logger.Error(err, "Failed to update heartbeat", "robotId", req.RobotID)
		s.sendMQTTResponse(req.RobotID, false, "Failed to update heartbeat")
		return
	}

	logger.V(1).Info("Heartbeat received", "robotId", req.RobotID)
	s.sendMQTTResponse(req.RobotID, true, "Heartbeat accepted")
}

// sendMQTTResponse 发送MQTT响应消息
func (s *Server) sendMQTTResponse(robotID string, success bool, message string) {
	logger := log.FromContext(s.ctx)

	response := Response{
		Success: success,
		Message: message,
		RobotID: robotID,
	}

	payload, err := json.Marshal(response)
	if err != nil {
		logger.Error(err, "Failed to marshal response", "robotId", robotID)
		return
	}

	// 发送到机器人专用的响应主题
	topic := fmt.Sprintf("%s/%s", TopicResponse, robotID)
	token := s.mqttClient.Publish(topic, 1, false, payload)
	if token.Wait() && token.Error() != nil {
		logger.Error(token.Error(), "Failed to publish response", "robotId", robotID, "topic", topic)
	}
}

// Start 启动 MQTT 服务器
func (s *Server) Start(ctx context.Context, brokerURL string) error {
	logger := log.FromContext(ctx)

	// 更新内部上下文
	s.ctx = ctx

	// 配置MQTT客户端选项
	opts := mqtt.NewClientOptions()
	opts.AddBroker(brokerURL)
	opts.SetClientID(fmt.Sprintf("k8s4r-server-%d", time.Now().Unix()))
	opts.SetKeepAlive(60 * time.Second)
	opts.SetPingTimeout(10 * time.Second)
	opts.SetCleanSession(true)
	opts.SetAutoReconnect(true)
	opts.SetMaxReconnectInterval(10 * time.Second)

	// 添加连接超时和写超时设置
	opts.SetConnectTimeout(10 * time.Second)
	opts.SetWriteTimeout(10 * time.Second)

	// 明确指定MQTT协议版本
	opts.SetProtocolVersion(4) // MQTT 3.1.1

	// 设置连接回调
	opts.SetOnConnectHandler(func(client mqtt.Client) {
		logger.Info("Connected to MQTT broker", "broker", brokerURL)

		// 订阅注册和心跳主题
		if token := client.Subscribe(TopicRegister, 1, s.RegisterHandler); token.Wait() && token.Error() != nil {
			logger.Error(token.Error(), "Failed to subscribe to register topic")
		}

		if token := client.Subscribe(TopicHeartbeat, 1, s.HeartbeatHandler); token.Wait() && token.Error() != nil {
			logger.Error(token.Error(), "Failed to subscribe to heartbeat topic")
		}

		logger.Info("Subscribed to MQTT topics successfully")
	})

	opts.SetConnectionLostHandler(func(client mqtt.Client, err error) {
		logger.Error(err, "Connection to MQTT broker lost")
	})

	// 创建MQTT客户端
	s.mqttClient = mqtt.NewClient(opts)

	// 连接到MQTT broker
	logger.Info("Starting MQTT server", "broker", brokerURL)
	if token := s.mqttClient.Connect(); token.Wait() && token.Error() != nil {
		return fmt.Errorf("failed to connect to MQTT broker: %w", token.Error())
	}

	// 等待上下文取消
	go func() {
		<-ctx.Done()
		logger.Info("Shutting down MQTT server")
		s.mqttClient.Disconnect(250)
		s.cancel()
	}()

	return nil
}
