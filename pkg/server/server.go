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
	"net/http"
	"time"

	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
	"sigs.k8s.io/controller-runtime/pkg/client"
	"sigs.k8s.io/controller-runtime/pkg/log"

	robotv1alpha1 "github.com/hxndg/k8s4r/api/v1alpha1"
)

const (
	// FixedToken 是用于 Agent 认证的固定 Token
	FixedToken = "fixed-token-123"
)

// Server 是 API Server 的主结构
type Server struct {
	Client    client.Client
	Namespace string
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
}

// NewServer 创建一个新的 Server 实例
func NewServer(client client.Client, namespace string) *Server {
	return &Server{
		Client:    client,
		Namespace: namespace,
	}
}

// RegisterHandler 处理 Agent 注册请求
func (s *Server) RegisterHandler(w http.ResponseWriter, r *http.Request) {
	logger := log.FromContext(r.Context())

	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var req RegisterRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		logger.Error(err, "Failed to decode register request")
		s.sendErrorResponse(w, "Invalid request body", http.StatusBadRequest)
		return
	}

	// 验证 Token
	if req.Token != FixedToken {
		logger.Info("Invalid token provided", "robotId", req.RobotID)
		s.sendErrorResponse(w, "Invalid token", http.StatusUnauthorized)
		return
	}

	// 检查 Robot 资源是否存在
	robot := &robotv1alpha1.Robot{}
	err := s.Client.Get(r.Context(), types.NamespacedName{
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

		if err := s.Client.Create(r.Context(), robot); err != nil {
			logger.Error(err, "Failed to create Robot resource", "robotId", req.RobotID)
			s.sendErrorResponse(w, "Failed to register robot", http.StatusInternalServerError)
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

	if err := s.Client.Status().Update(r.Context(), robot); err != nil {
		logger.Error(err, "Failed to update Robot status", "robotId", req.RobotID)
		s.sendErrorResponse(w, "Failed to update robot status", http.StatusInternalServerError)
		return
	}

	logger.Info("Robot registered successfully", "robotId", req.RobotID)
	s.sendSuccessResponse(w, "Robot registered successfully")
}

// HeartbeatHandler 处理 Agent 心跳请求
func (s *Server) HeartbeatHandler(w http.ResponseWriter, r *http.Request) {
	logger := log.FromContext(r.Context())

	if r.Method != http.MethodPost {
		http.Error(w, "Method not allowed", http.StatusMethodNotAllowed)
		return
	}

	var req HeartbeatRequest
	if err := json.NewDecoder(r.Body).Decode(&req); err != nil {
		logger.Error(err, "Failed to decode heartbeat request")
		s.sendErrorResponse(w, "Invalid request body", http.StatusBadRequest)
		return
	}

	// 验证 Token
	if req.Token != FixedToken {
		logger.Info("Invalid token provided", "robotId", req.RobotID)
		s.sendErrorResponse(w, "Invalid token", http.StatusUnauthorized)
		return
	}

	// 获取 Robot 资源
	robot := &robotv1alpha1.Robot{}
	err := s.Client.Get(r.Context(), types.NamespacedName{
		Name:      req.RobotID,
		Namespace: s.Namespace,
	}, robot)

	if err != nil {
		logger.Error(err, "Robot not found", "robotId", req.RobotID)
		s.sendErrorResponse(w, "Robot not registered", http.StatusNotFound)
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

	if err := s.Client.Status().Update(r.Context(), robot); err != nil {
		logger.Error(err, "Failed to update heartbeat", "robotId", req.RobotID)
		s.sendErrorResponse(w, "Failed to update heartbeat", http.StatusInternalServerError)
		return
	}

	logger.V(1).Info("Heartbeat received", "robotId", req.RobotID)
	s.sendSuccessResponse(w, "Heartbeat accepted")
}

// sendSuccessResponse 发送成功响应
func (s *Server) sendSuccessResponse(w http.ResponseWriter, message string) {
	w.Header().Set("Content-Type", "application/json")
	json.NewEncoder(w).Encode(Response{
		Success: true,
		Message: message,
	})
}

// sendErrorResponse 发送错误响应
func (s *Server) sendErrorResponse(w http.ResponseWriter, message string, statusCode int) {
	w.Header().Set("Content-Type", "application/json")
	w.WriteHeader(statusCode)
	json.NewEncoder(w).Encode(Response{
		Success: false,
		Message: message,
	})
}

// Start 启动 HTTP 服务器
func (s *Server) Start(ctx context.Context, addr string) error {
	mux := http.NewServeMux()
	mux.HandleFunc("/api/v1/register", s.RegisterHandler)
	mux.HandleFunc("/api/v1/heartbeat", s.HeartbeatHandler)

	server := &http.Server{
		Addr:         addr,
		Handler:      mux,
		ReadTimeout:  10 * time.Second,
		WriteTimeout: 10 * time.Second,
	}

	logger := log.FromContext(ctx)
	logger.Info("Starting API server", "addr", addr)

	go func() {
		<-ctx.Done()
		shutdownCtx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
		defer cancel()
		if err := server.Shutdown(shutdownCtx); err != nil {
			logger.Error(err, "Failed to shutdown server gracefully")
		}
	}()

	if err := server.ListenAndServe(); err != nil && err != http.ErrServerClosed {
		return fmt.Errorf("failed to start server: %w", err)
	}

	return nil
}
