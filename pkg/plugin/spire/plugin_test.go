package spire

import (
	"context"
	"encoding/json"
	"os"
	"path/filepath"
	"testing"

	"github.com/hashicorp/go-hclog"
	"github.com/hxndghxndg/k8s4r/pkg/plugin"
)

// TestSPIREPlugin_BasicLifecycle 测试 SPIRE 插件基本生命周期
func TestSPIREPlugin_BasicLifecycle(t *testing.T) {
	// 创建临时工作目录
	tmpDir := filepath.Join(os.TempDir(), "k8s4r-spire-test")
	defer os.RemoveAll(tmpDir)

	if err := os.MkdirAll(tmpDir, 0755); err != nil {
		t.Fatalf("Failed to create temp dir: %v", err)
	}

	logger := hclog.New(&hclog.LoggerOptions{
		Name:   "spire-test",
		Level:  hclog.Debug,
		Output: os.Stdout,
	})

	// 创建插件实例
	spirePlugin := NewPlugin()

	// 准备配置
	spireConfig := &Config{
		ServerAddress: "spire-server.example.com:8081",
		TrustDomain:   "test.example.org",
		SocketPath:    filepath.Join(tmpDir, "agent.sock"),
		DataDir:       filepath.Join(tmpDir, "data"),
		LogLevel:      "DEBUG",
		NodeAttestor: NodeAttestorConfig{
			Type: "join_token",
		},
		JoinToken: "test-join-token-12345",
		Keyring: KeyringConfig{
			Enabled: false, // 测试时禁用 keyring
		},
	}

	configJSON, err := json.Marshal(spireConfig)
	if err != nil {
		t.Fatalf("Failed to marshal config: %v", err)
	}

	pluginConfig := plugin.Config{
		Name:         "spire",
		Enabled:      true,
		BaseDir:      tmpDir,
		Logger:       logger,
		PluginConfig: configJSON,
	}

	// 测试初始化
	t.Log("Testing Initialize...")
	ctx := context.Background()
	if err := spirePlugin.Initialize(ctx, pluginConfig); err != nil {
		t.Fatalf("Initialize failed: %v", err)
	}

	// 验证配置
	if spirePlugin.config == nil {
		t.Fatal("Config not loaded")
	}
	if spirePlugin.config.TrustDomain != "test.example.org" {
		t.Errorf("Wrong trust domain: %s", spirePlugin.config.TrustDomain)
	}

	// 验证 Agent Manager 已创建
	if spirePlugin.agentManager == nil {
		t.Fatal("Agent manager not created")
	}

	t.Log("✓ Initialize succeeded")

	// 注意：Start 需要真实的 spire-agent 二进制和 SPIRE Server
	// 在 CI 环境中可能会跳过
	if os.Getenv("SKIP_SPIRE_INTEGRATION") != "" {
		t.Skip("Skipping SPIRE integration test (SKIP_SPIRE_INTEGRATION set)")
	}

	// 测试获取运行时配置（在未启动状态下应该失败）
	t.Log("Testing GetRuntimeConfig before start...")
	if _, err := spirePlugin.GetRuntimeConfig(); err == nil {
		t.Error("GetRuntimeConfig should fail before start")
	}

	t.Log("✓ All basic tests passed")
}

// TestConfig_Validate 测试配置验证
func TestConfig_Validate(t *testing.T) {
	tests := []struct {
		name    string
		config  *Config
		wantErr bool
	}{
		{
			name: "valid config",
			config: &Config{
				ServerAddress: "localhost:8081",
				TrustDomain:   "example.org",
				DataDir:       "/tmp/spire",
				NodeAttestor: NodeAttestorConfig{
					Type: "join_token",
				},
			},
			wantErr: false,
		},
		{
			name: "missing server address",
			config: &Config{
				TrustDomain: "example.org",
				DataDir:     "/tmp/spire",
			},
			wantErr: true,
		},
		{
			name: "missing trust domain",
			config: &Config{
				ServerAddress: "localhost:8081",
				DataDir:       "/tmp/spire",
			},
			wantErr: true,
		},
		{
			name: "missing data dir",
			config: &Config{
				ServerAddress: "localhost:8081",
				TrustDomain:   "example.org",
			},
			wantErr: true,
		},
		{
			name: "invalid keyring type",
			config: &Config{
				ServerAddress: "localhost:8081",
				TrustDomain:   "example.org",
				DataDir:       "/tmp/spire",
				NodeAttestor: NodeAttestorConfig{
					Type: "join_token",
				},
				Keyring: KeyringConfig{
					Enabled: true,
					Type:    "invalid",
				},
			},
			wantErr: true,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			err := tt.config.Validate()
			if (err != nil) != tt.wantErr {
				t.Errorf("Validate() error = %v, wantErr %v", err, tt.wantErr)
			}
		})
	}
}

// TestConfig_ToAgentConfig 测试配置文件生成
func TestConfig_ToAgentConfig(t *testing.T) {
	config := &Config{
		ServerAddress: "spire-server.example.com",
		TrustDomain:   "example.org",
		SocketPath:    "/run/spire/agent.sock",
		DataDir:       "/var/lib/spire",
		LogLevel:      "DEBUG",
		NodeAttestor: NodeAttestorConfig{
			Type: "join_token",
		},
		JoinToken: "test-token",
	}

	agentConfig := config.ToAgentConfig()

	// 验证关键配置是否存在
	expectedStrings := []string{
		"data_dir = \"/var/lib/spire\"",
		"log_level = \"DEBUG\"",
		"server_address = \"spire-server.example.com\"",
		"socket_path = \"/run/spire/agent.sock\"",
		"trust_domain = \"example.org\"",
		"NodeAttestor \"join_token\"",
		"join_token = \"test-token\"",
	}

	for _, expected := range expectedStrings {
		if !contains(agentConfig, expected) {
			t.Errorf("Agent config missing expected string: %s", expected)
		}
	}

	t.Logf("Generated agent config:\n%s", agentConfig)
}

// TestKeyringManager_Basic 测试 Keyring Manager 基本功能
func TestKeyringManager_Basic(t *testing.T) {
	// 这个测试需要 Linux 系统和适当的权限
	if os.Getenv("SKIP_KEYRING_TEST") != "" {
		t.Skip("Skipping keyring test (SKIP_KEYRING_TEST set)")
	}

	logger := hclog.New(&hclog.LoggerOptions{
		Name:   "keyring-test",
		Level:  hclog.Debug,
		Output: os.Stdout,
	})

	config := &KeyringConfig{
		Enabled: true,
		Type:    "session",
	}

	km, err := NewKeyringManager(logger, config)
	if err != nil {
		t.Skipf("Failed to create keyring manager (may not be supported): %v", err)
	}

	// 测试存储密钥
	testKey := []byte("test-secret-key-data")
	keyID, err := km.StoreKey("test-key", testKey)
	if err != nil {
		t.Fatalf("Failed to store key: %v", err)
	}
	t.Logf("Key stored with ID: %d", keyID)

	// 测试搜索密钥
	foundKeyID, err := km.SearchKey("test-key")
	if err != nil {
		t.Fatalf("Failed to search key: %v", err)
	}
	if foundKeyID != keyID {
		t.Errorf("Search returned wrong key ID: got %d, want %d", foundKeyID, keyID)
	}

	// 测试读取密钥
	readKey, err := km.ReadKey(keyID)
	if err != nil {
		t.Fatalf("Failed to read key: %v", err)
	}
	if string(readKey) != string(testKey) {
		t.Errorf("Read key mismatch: got %s, want %s", string(readKey), string(testKey))
	}

	// 测试撤销密钥
	if err := km.RevokeKey(keyID); err != nil {
		t.Fatalf("Failed to revoke key: %v", err)
	}

	// 验证密钥已撤销（读取应该失败）
	_, err = km.ReadKey(keyID)
	if err == nil {
		t.Error("Reading revoked key should fail")
	}

	t.Log("✓ Keyring manager basic tests passed")
}

// 辅助函数
func contains(s, substr string) bool {
	return len(s) >= len(substr) && (s == substr || len(s) > len(substr) && (s[:len(substr)] == substr || s[len(s)-len(substr):] == substr || containsMiddle(s, substr)))
}

func containsMiddle(s, substr string) bool {
	for i := 0; i <= len(s)-len(substr); i++ {
		if s[i:i+len(substr)] == substr {
			return true
		}
	}
	return false
}
