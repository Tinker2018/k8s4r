package spire

import (
	"context"

	"github.com/hashicorp/go-hclog"
)

// WorkloadAPIClient 直接连接 Workload API 的客户端
// 用于 workload-api 模式（不启动 spire-agent 进程）
type WorkloadAPIClient struct {
	SocketPath string
	Logger     hclog.Logger
}

// FetchSVID 从 Workload API 获取 SVID
func (c *WorkloadAPIClient) FetchSVID(ctx context.Context) (*WorkloadSVID, error) {
	return FetchX509SVID(ctx, c.SocketPath)
}

// WaitReady Workload API 模式不需要等待（socket 由外部管理）
func (c *WorkloadAPIClient) WaitReady(ctx context.Context, timeout interface{}) error {
	// 直接返回，不需要等待
	return nil
}

// Start Workload API 模式不需要启动进程
func (c *WorkloadAPIClient) Start(ctx context.Context) error {
	// 不需要启动，直接返回
	return nil
}

// Stop Workload API 模式不需要停止进程
func (c *WorkloadAPIClient) Stop(ctx context.Context) error {
	// 不需要停止，直接返回
	return nil
}
