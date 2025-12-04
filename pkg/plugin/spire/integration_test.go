//go:build integration
// +build integration

package spire

import (
	"context"
	"crypto/rand"
	"crypto/rsa"
	"crypto/tls"
	"crypto/x509"
	"crypto/x509/pkix"
	"encoding/pem"
	"io"
	"math/big"
	"net"
	"net/url"
	"os"
	"path/filepath"
	"testing"
	"time"

	"github.com/hashicorp/go-hclog"
	"github.com/spiffe/go-spiffe/v2/proto/spiffe/workload"
	"github.com/stretchr/testify/assert"
	"github.com/stretchr/testify/require"
	"google.golang.org/grpc"
)

// MockSPIREServer 模拟 SPIRE Workload API Server
type MockSPIREServer struct {
	workload.UnimplementedSpiffeWorkloadAPIServer

	listener   net.Listener
	grpcServer *grpc.Server
	socketPath string

	// 用于验证的数据
	issuedSVIDs []*workload.X509SVID
	trustBundle *x509.CertPool
	caCert      *x509.Certificate
	caKey       *rsa.PrivateKey
}

// NewMockSPIREServer 创建 mock server
func NewMockSPIREServer(t *testing.T) *MockSPIREServer {
	tmpDir := t.TempDir()
	socketPath := filepath.Join(tmpDir, "agent.sock")

	listener, err := net.Listen("unix", socketPath)
	require.NoError(t, err)

	// 生成自签名 CA 证书
	ca, caKey := generateCA(t)

	// 创建 gRPC server
	grpcServer := grpc.NewServer()

	mock := &MockSPIREServer{
		listener:    listener,
		grpcServer:  grpcServer,
		socketPath:  socketPath,
		issuedSVIDs: make([]*workload.X509SVID, 0),
		trustBundle: x509.NewCertPool(),
		caCert:      ca,
		caKey:       caKey,
	}
	mock.trustBundle.AddCert(ca)

	// 注册服务
	workload.RegisterSpiffeWorkloadAPIServer(grpcServer, mock)

	// 生成测试用的 SVID
	svid := mock.generateSVID(t, "spiffe://test.example.org/agent/test")
	mock.issuedSVIDs = append(mock.issuedSVIDs, svid)

	return mock
}

// Start 启动 mock server
func (m *MockSPIREServer) Start() {
	go m.grpcServer.Serve(m.listener)
	time.Sleep(100 * time.Millisecond) // 等待 server 启动
}

// Stop 停止 mock server
func (m *MockSPIREServer) Stop() {
	m.grpcServer.Stop()
	m.listener.Close()
}

// GetSocketPath 获取 socket 路径
func (m *MockSPIREServer) GetSocketPath() string {
	return m.socketPath
}

// GetIssuedSVIDs 获取已颁发的 SVID
func (m *MockSPIREServer) GetIssuedSVIDs() []*workload.X509SVID {
	return m.issuedSVIDs
}

// FetchX509SVID 实现 Workload API
func (m *MockSPIREServer) FetchX509SVID(req *workload.X509SVIDRequest, stream workload.SpiffeWorkloadAPI_FetchX509SVIDServer) error {
	bundleBytes := m.encodeTrustBundle()

	resp := &workload.X509SVIDResponse{
		Svids: m.issuedSVIDs,
	}

	// 添加 trust bundle
	for _, svid := range resp.Svids {
		svid.Bundle = bundleBytes
	}

	return stream.Send(resp)
}

// FetchX509Bundles 实现 Bundle API
func (m *MockSPIREServer) FetchX509Bundles(req *workload.X509BundlesRequest, stream workload.SpiffeWorkloadAPI_FetchX509BundlesServer) error {
	bundleBytes := m.encodeTrustBundle()

	resp := &workload.X509BundlesResponse{
		Bundles: map[string][]byte{
			"test.example.org": bundleBytes,
		},
	}

	return stream.Send(resp)
}

// generateSVID 生成 SVID
func (m *MockSPIREServer) generateSVID(t *testing.T, spiffeIDStr string) *workload.X509SVID {
	key, err := rsa.GenerateKey(rand.Reader, 2048)
	require.NoError(t, err)

	spiffeURL, err := url.Parse(spiffeIDStr)
	require.NoError(t, err)

	template := &x509.Certificate{
		SerialNumber: big.NewInt(time.Now().Unix()),
		Subject: pkix.Name{
			CommonName: spiffeIDStr,
		},
		URIs:                  []*url.URL{spiffeURL},
		NotBefore:             time.Now(),
		NotAfter:              time.Now().Add(1 * time.Hour),
		KeyUsage:              x509.KeyUsageDigitalSignature | x509.KeyUsageKeyEncipherment,
		ExtKeyUsage:           []x509.ExtKeyUsage{x509.ExtKeyUsageServerAuth, x509.ExtKeyUsageClientAuth},
		BasicConstraintsValid: true,
	}

	certDER, err := x509.CreateCertificate(rand.Reader, template, m.caCert, &key.PublicKey, m.caKey)
	require.NoError(t, err)

	certPEM := pem.EncodeToMemory(&pem.Block{Type: "CERTIFICATE", Bytes: certDER})
	keyDER := x509.MarshalPKCS1PrivateKey(key)
	keyPEM := pem.EncodeToMemory(&pem.Block{Type: "RSA PRIVATE KEY", Bytes: keyDER})

	return &workload.X509SVID{
		SpiffeId:    spiffeIDStr,
		X509Svid:    certPEM,
		X509SvidKey: keyPEM,
	}
}

// encodeTrustBundle 编码 trust bundle
func (m *MockSPIREServer) encodeTrustBundle() []byte {
	return pem.EncodeToMemory(&pem.Block{
		Type:  "CERTIFICATE",
		Bytes: m.caCert.Raw,
	})
}

// generateCA 生成 CA 证书
func generateCA(t *testing.T) (*x509.Certificate, *rsa.PrivateKey) {
	key, err := rsa.GenerateKey(rand.Reader, 2048)
	require.NoError(t, err)

	template := &x509.Certificate{
		SerialNumber: big.NewInt(1),
		Subject: pkix.Name{
			CommonName: "Test CA",
		},
		NotBefore:             time.Now(),
		NotAfter:              time.Now().Add(24 * time.Hour),
		KeyUsage:              x509.KeyUsageCertSign | x509.KeyUsageCRLSign,
		BasicConstraintsValid: true,
		IsCA:                  true,
	}

	certDER, err := x509.CreateCertificate(rand.Reader, template, template, &key.PublicKey, key)
	require.NoError(t, err)

	cert, err := x509.ParseCertificate(certDER)
	require.NoError(t, err)

	return cert, key
}

// === 集成测试 ===

// TestIntegration_FetchSVID 测试从 Workload API 获取 SVID
func TestIntegration_FetchSVID(t *testing.T) {
	mockServer := NewMockSPIREServer(t)
	mockServer.Start()
	defer mockServer.Stop()

	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	// 从 Workload API 获取 SVID
	svids, err := fetchSVIDFromWorkloadAPI(ctx, mockServer.GetSocketPath())
	require.NoError(t, err, "Failed to fetch SVID from Workload API")
	require.NotEmpty(t, svids, "No SVIDs returned")

	svid := svids[0]

	// 验证 SVID 内容
	assert.Equal(t, "spiffe://test.example.org/agent/test", svid.SpiffeId)
	assert.NotEmpty(t, svid.X509Svid, "SVID certificate is empty")
	assert.NotEmpty(t, svid.X509SvidKey, "SVID key is empty")
	assert.NotEmpty(t, svid.Bundle, "Trust bundle is empty")

	// 验证证书可以解析
	certBlock, _ := pem.Decode(svid.X509Svid)
	require.NotNil(t, certBlock, "Failed to decode certificate PEM")

	cert, err := x509.ParseCertificate(certBlock.Bytes)
	require.NoError(t, err, "Failed to parse certificate")

	// 验证 SPIFFE ID
	require.Equal(t, 1, len(cert.URIs), "Certificate should have exactly one URI")
	assert.Equal(t, "spiffe://test.example.org/agent/test", cert.URIs[0].String())

	t.Log("✓ SVID fetched and validated successfully")
}

// TestIntegration_VerifySVID 测试 SVID 证书链验证
func TestIntegration_VerifySVID(t *testing.T) {
	mockServer := NewMockSPIREServer(t)
	mockServer.Start()
	defer mockServer.Stop()

	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	svids, err := fetchSVIDFromWorkloadAPI(ctx, mockServer.GetSocketPath())
	require.NoError(t, err)
	require.NotEmpty(t, svids)

	svid := svids[0]

	// 解析证书
	certBlock, _ := pem.Decode(svid.X509Svid)
	require.NotNil(t, certBlock)
	cert, err := x509.ParseCertificate(certBlock.Bytes)
	require.NoError(t, err)

	// 解析 trust bundle
	bundleBlock, _ := pem.Decode(svid.Bundle)
	require.NotNil(t, bundleBlock, "Failed to decode trust bundle")
	caCert, err := x509.ParseCertificate(bundleBlock.Bytes)
	require.NoError(t, err, "Failed to parse CA certificate")

	// 验证证书链
	roots := x509.NewCertPool()
	roots.AddCert(caCert)

	opts := x509.VerifyOptions{
		Roots:     roots,
		KeyUsages: []x509.ExtKeyUsage{x509.ExtKeyUsageClientAuth},
	}

	chains, err := cert.Verify(opts)
	require.NoError(t, err, "Certificate chain verification failed")
	assert.NotEmpty(t, chains, "No valid certificate chains found")

	t.Logf("✓ Certificate chain verified, chain length: %d", len(chains[0]))
}

// TestIntegration_SVIDForMTLS 测试使用 SVID 建立 mTLS 连接
func TestIntegration_SVIDForMTLS(t *testing.T) {
	mockServer := NewMockSPIREServer(t)
	mockServer.Start()
	defer mockServer.Stop()

	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	// 获取 SVID
	svids, err := fetchSVIDFromWorkloadAPI(ctx, mockServer.GetSocketPath())
	require.NoError(t, err)
	require.NotEmpty(t, svids)

	svid := svids[0]

	// 创建 TLS 证书
	cert, err := tls.X509KeyPair(svid.X509Svid, svid.X509SvidKey)
	require.NoError(t, err, "Failed to create TLS certificate")

	// 创建 CA pool
	caCertPool := x509.NewCertPool()
	ok := caCertPool.AppendCertsFromPEM(svid.Bundle)
	require.True(t, ok, "Failed to append CA certificate")

	// 配置 TLS（服务端）
	serverTLSConfig := &tls.Config{
		Certificates:       []tls.Certificate{cert},
		ClientCAs:          caCertPool,
		ClientAuth:         tls.VerifyClientCertIfGiven, // 更宽松的验证
		MinVersion:         tls.VersionTLS12,
		InsecureSkipVerify: true, // 测试环境跳过验证
	}

	// 启动测试 server
	listener, err := tls.Listen("tcp", "127.0.0.1:0", serverTLSConfig)
	require.NoError(t, err)
	defer listener.Close()

	// 接受连接（在goroutine中）
	done := make(chan bool)
	go func() {
		conn, _ := listener.Accept()
		if conn != nil {
			// 简单读写以完成握手
			buf := make([]byte, 1)
			conn.Read(buf)
			conn.Write([]byte("OK"))
			conn.Close()
		}
		done <- true
	}()

	// 配置 TLS（客户端）
	clientTLSConfig := &tls.Config{
		Certificates:       []tls.Certificate{cert},
		RootCAs:            caCertPool,
		MinVersion:         tls.VersionTLS12,
		InsecureSkipVerify: true, // 测试环境跳过主机名验证
	}

	// 建立连接
	conn, err := tls.Dial("tcp", listener.Addr().String(), clientTLSConfig)
	require.NoError(t, err, "Failed to establish mTLS connection")
	defer conn.Close()

	// 等待握手完成
	time.Sleep(100 * time.Millisecond)

	// 验证连接状态
	state := conn.ConnectionState()
	assert.True(t, state.HandshakeComplete, "TLS handshake not complete")
	assert.NotEmpty(t, state.PeerCertificates, "No peer certificates")

	// 验证对端证书的 SPIFFE ID
	peerCert := state.PeerCertificates[0]
	assert.Equal(t, 1, len(peerCert.URIs))
	assert.Equal(t, "spiffe://test.example.org/agent/test", peerCert.URIs[0].String())

	t.Log("✓ mTLS connection established successfully")
}

// TestIntegration_KeyringStorage 测试 Keyring 存储（需要 root 权限）
func TestIntegration_KeyringStorage(t *testing.T) {
	if os.Getuid() != 0 {
		t.Skip("Keyring tests require root privileges, skipping")
	}

	mockServer := NewMockSPIREServer(t)
	mockServer.Start()
	defer mockServer.Stop()

	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	// 获取 SVID
	svids, err := fetchSVIDFromWorkloadAPI(ctx, mockServer.GetSocketPath())
	require.NoError(t, err)
	require.NotEmpty(t, svids)

	// 创建 Keyring Manager
	logger := hclog.New(&hclog.LoggerOptions{
		Name:   "keyring-test",
		Level:  hclog.Debug,
		Output: io.Discard,
	})

	config := &KeyringConfig{
		Enabled: true,
		Type:    "session",
	}

	keyringMgr, err := NewKeyringManager(logger, config)
	require.NoError(t, err, "Failed to create keyring manager")

	// 存储私钥到 keyring
	keyName := "test-svid-key"
	keyID, err := keyringMgr.StoreKey(keyName, svids[0].X509SvidKey)
	require.NoError(t, err, "Failed to store key in keyring")

	// 从 keyring 读取私钥
	retrievedKey, err := keyringMgr.ReadKey(keyID)
	require.NoError(t, err, "Failed to read key from keyring")

	// 验证密钥一致
	assert.Equal(t, svids[0].X509SvidKey, retrievedKey, "Retrieved key doesn't match stored key")

	// 清理（撤销单个密钥）
	err = keyringMgr.RevokeKey(keyID)
	require.NoError(t, err, "Failed to revoke key")

	t.Log("✓ SVID key stored and retrieved from keyring successfully")
}

// === 辅助函数 ===

// fetchSVIDFromWorkloadAPI 从 Workload API 获取 SVID
func fetchSVIDFromWorkloadAPI(ctx context.Context, socketPath string) ([]*workload.X509SVID, error) {
	conn, err := grpc.DialContext(ctx, "unix://"+socketPath,
		grpc.WithInsecure(),
		grpc.WithContextDialer(func(ctx context.Context, addr string) (net.Conn, error) {
			return net.Dial("unix", socketPath)
		}),
	)
	if err != nil {
		return nil, err
	}
	defer conn.Close()

	client := workload.NewSpiffeWorkloadAPIClient(conn)

	stream, err := client.FetchX509SVID(ctx, &workload.X509SVIDRequest{})
	if err != nil {
		return nil, err
	}

	resp, err := stream.Recv()
	if err != nil {
		return nil, err
	}

	return resp.Svids, nil
}
