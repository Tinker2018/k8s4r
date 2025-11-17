/*
Copyright 2025.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0
*/

package main

import (
	"bytes"
	"encoding/json"
	"flag"
	"fmt"
	"log"
	"net/http"
	"os"
	"os/signal"
	"syscall"
	"time"

	robotv1alpha1 "github.com/hxndg/k8s4r/api/v1alpha1"
	"github.com/hxndg/k8s4r/pkg/collector"
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
}

// Agent 结构体
type Agent struct {
	ServerURL         string
	Token             string
	RobotID           string
	HeartbeatInterval time.Duration
	httpClient        *http.Client
}

// NewAgent 创建一个新的 Agent 实例
func NewAgent(serverURL, token, robotID string, heartbeatInterval time.Duration) *Agent {
	return &Agent{
		ServerURL:         serverURL,
		Token:             token,
		RobotID:           robotID,
		HeartbeatInterval: heartbeatInterval,
		httpClient: &http.Client{
			Timeout: 10 * time.Second,
		},
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

	body, err := json.Marshal(req)
	if err != nil {
		return fmt.Errorf("failed to marshal register request: %w", err)
	}

	resp, err := a.httpClient.Post(
		fmt.Sprintf("%s/api/v1/register", a.ServerURL),
		"application/json",
		bytes.NewBuffer(body),
	)
	if err != nil {
		return fmt.Errorf("failed to send register request: %w", err)
	}
	defer resp.Body.Close()

	var result Response
	if err := json.NewDecoder(resp.Body).Decode(&result); err != nil {
		return fmt.Errorf("failed to decode response: %w", err)
	}

	if !result.Success {
		return fmt.Errorf("registration failed: %s", result.Message)
	}

	log.Printf("Successfully registered robot: %s", a.RobotID)
	return nil
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

	body, err := json.Marshal(req)
	if err != nil {
		return fmt.Errorf("failed to marshal heartbeat request: %w", err)
	}

	resp, err := a.httpClient.Post(
		fmt.Sprintf("%s/api/v1/heartbeat", a.ServerURL),
		"application/json",
		bytes.NewBuffer(body),
	)
	if err != nil {
		return fmt.Errorf("failed to send heartbeat: %w", err)
	}
	defer resp.Body.Close()

	var result Response
	if err := json.NewDecoder(resp.Body).Decode(&result); err != nil {
		return fmt.Errorf("failed to decode response: %w", err)
	}

	if !result.Success {
		return fmt.Errorf("heartbeat failed: %s", result.Message)
	}

	log.Printf("Heartbeat sent successfully for robot: %s", a.RobotID)
	return nil
}

// Run 运行 Agent
func (a *Agent) Run() {
	// 首先注册
	log.Printf("Starting agent for robot: %s", a.RobotID)
	log.Printf("Server URL: %s", a.ServerURL)

	for {
		if err := a.Register(); err != nil {
			log.Printf("Failed to register: %v, retrying in 5 seconds...", err)
			time.Sleep(5 * time.Second)
			continue
		}
		break
	}

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
		serverURL         string
		token             string
		robotID           string
		heartbeatInterval int
	)

	flag.StringVar(&serverURL, "server-url", "http://localhost:8080", "The URL of the API server")
	flag.StringVar(&token, "token", "fixed-token-123", "The authentication token")
	flag.StringVar(&robotID, "robot-id", "", "The unique ID of this robot (required)")
	flag.IntVar(&heartbeatInterval, "heartbeat-interval", 30, "Heartbeat interval in seconds")
	flag.Parse()

	if robotID == "" {
		log.Fatal("robot-id is required")
	}

	agent := NewAgent(
		serverURL,
		token,
		robotID,
		time.Duration(heartbeatInterval)*time.Second,
	)

	agent.Run()
}
