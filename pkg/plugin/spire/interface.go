package spire

import (
	"context"
	"time"
)

// SPIREClient SPIRE 客户端接口
// 统一 AgentManager（进程模式）和 WorkloadAPIClient（直连模式）
type SPIREClient interface {
	// Start 启动（如果需要）
	Start(ctx context.Context) error

	// Stop 停止（如果需要）
	Stop(ctx context.Context) error

	// FetchSVID 获取 SVID
	FetchSVID(ctx context.Context) (*WorkloadSVID, error)

	// WaitReady 等待就绪
	WaitReady(ctx context.Context, timeout time.Duration) error
}
