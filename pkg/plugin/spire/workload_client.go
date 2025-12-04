package spire

import (
	"context"
	"fmt"

	"github.com/spiffe/go-spiffe/v2/workloadapi"
)

// WorkloadSVID 表示从 Workload API 获取的 SVID
type WorkloadSVID struct {
	SpiffeID    string // SPIFFE ID
	X509Svid    []byte // PEM 编码的证书
	X509SvidKey []byte // PEM 编码的私钥
	Bundle      []byte // PEM 编码的信任根证书
}

// FetchX509SVID 从 SPIRE Workload API 获取 X.509 SVID
// 使用官方 go-spiffe 库来处理所有安全细节（包括 security headers）
func FetchX509SVID(ctx context.Context, socketPath string) (*WorkloadSVID, error) {
	// 使用 go-spiffe 官方客户端（会自动处理 security headers）
	client, err := workloadapi.New(ctx, workloadapi.WithAddr("unix://"+socketPath))
	if err != nil {
		return nil, fmt.Errorf("failed to create workload API client: %w", err)
	}
	defer client.Close()

	// 获取 X.509 SVID
	x509Context, err := client.FetchX509Context(ctx)
	if err != nil {
		return nil, fmt.Errorf("failed to fetch X509 context: %w", err)
	}

	// 验证 SVIDs
	if len(x509Context.SVIDs) == 0 {
		return nil, fmt.Errorf("no SVIDs in response")
	}

	// 获取默认 SVID
	defaultSVID := x509Context.DefaultSVID()

	// 编码证书和私钥为 PEM 格式
	certPEM, keyPEM, err := defaultSVID.Marshal()
	if err != nil {
		return nil, fmt.Errorf("failed to marshal SVID: %w", err)
	}

	// 获取信任根证书
	bundleSet := x509Context.Bundles
	var bundlePEM []byte
	for _, bundle := range bundleSet.Bundles() {
		bundlePEM, err = bundle.Marshal()
		if err != nil {
			return nil, fmt.Errorf("failed to marshal bundle: %w", err)
		}
		break // 使用第一个 bundle
	}

	return &WorkloadSVID{
		SpiffeID:    defaultSVID.ID.String(),
		X509Svid:    certPEM,
		X509SvidKey: keyPEM,
		Bundle:      bundlePEM,
	}, nil
}
