package agent

import (
	"testing"

	robotv1alpha1 "github.com/hxndghxndg/k8s4r/api/v1alpha1"
)

// TestGenerateEnvoyConfig 测试 Envoy 配置生成
func TestGenerateEnvoyConfig(t *testing.T) {
	tests := []struct {
		name      string
		taskName  string
		network   *robotv1alpha1.NetworkProxy
		expectErr bool
	}{
		{
			name:     "basic envoy config",
			taskName: "test-task",
			network: &robotv1alpha1.NetworkProxy{
				Enabled: true,
				Type:    "envoy",
				SPIFFE: &robotv1alpha1.SPIFFEConfig{
					TrustDomain:     "example.org",
					AgentSocketPath: "/run/spire/sockets/agent.sock",
				},
				Upstreams: []robotv1alpha1.ProxyUpstream{
					{
						Name:      "database",
						LocalPort: 5432,
						Upstream:  "postgres.default.svc.cluster.local:5432",
						Protocol:  "tcp",
						TLS:       true,
					},
				},
			},
			expectErr: false,
		},
		{
			name:     "multiple upstreams",
			taskName: "multi-upstream-task",
			network: &robotv1alpha1.NetworkProxy{
				Enabled: true,
				Type:    "envoy",
				SPIFFE: &robotv1alpha1.SPIFFEConfig{
					TrustDomain:     "example.org",
					AgentSocketPath: "/run/spire/sockets/agent.sock",
				},
				Upstreams: []robotv1alpha1.ProxyUpstream{
					{
						Name:      "database",
						LocalPort: 5432,
						Upstream:  "postgres.default.svc:5432",
						Protocol:  "tcp",
						TLS:       true,
					},
					{
						Name:      "api-service",
						LocalPort: 8081,
						Upstream:  "api.default.svc:443",
						Protocol:  "http",
						TLS:       true,
					},
					{
						Name:      "grpc-service",
						LocalPort: 9090,
						Upstream:  "grpc.default.svc:9090",
						Protocol:  "grpc",
						TLS:       true,
					},
				},
			},
			expectErr: false,
		},
		{
			name:     "http protocol upstream",
			taskName: "http-task",
			network: &robotv1alpha1.NetworkProxy{
				Enabled: true,
				Type:    "envoy",
				SPIFFE: &robotv1alpha1.SPIFFEConfig{
					TrustDomain:     "test.local",
					AgentSocketPath: "/var/run/spire/agent.sock",
				},
				Upstreams: []robotv1alpha1.ProxyUpstream{
					{
						Name:      "web-api",
						LocalPort: 8080,
						Upstream:  "web.backend.svc:80",
						Protocol:  "http",
						TLS:       false, // 测试非 TLS
					},
				},
			},
			expectErr: false,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			config, err := GenerateEnvoyConfig(tt.taskName, tt.network)

			if tt.expectErr && err == nil {
				t.Error("expected error but got none")
			}
			if !tt.expectErr && err != nil {
				t.Errorf("unexpected error: %v", err)
			}

			if err == nil {
				// 验证配置不为空
				if len(config) == 0 {
					t.Error("generated config is empty")
				}

				// 打印生成的配置用于调试
				t.Logf("Generated Envoy config:\n%s", config)

				// 验证包含关键字段
				configStr := string(config)
				if !contains(configStr, "admin") {
					t.Error("config missing admin section")
				}
				if !contains(configStr, "static_resources") {
					t.Error("config missing static_resources section")
				}

				// 验证 SPIFFE 配置
				if tt.network.SPIFFE != nil {
					if !contains(configStr, tt.network.SPIFFE.TrustDomain) {
						t.Errorf("config missing trust domain: %s", tt.network.SPIFFE.TrustDomain)
					}
					if !contains(configStr, tt.network.SPIFFE.AgentSocketPath) {
						t.Errorf("config missing agent socket path: %s", tt.network.SPIFFE.AgentSocketPath)
					}
				}

				// 验证每个 upstream 配置
				for _, upstream := range tt.network.Upstreams {
					if !contains(configStr, upstream.Name) {
						t.Errorf("config missing upstream: %s", upstream.Name)
					}
				}
			}
		})
	}
}

// TestEnvoyConfigSPIFFEIntegration 测试 SPIFFE 集成
func TestEnvoyConfigSPIFFEIntegration(t *testing.T) {
	network := &robotv1alpha1.NetworkProxy{
		Enabled: true,
		Type:    "envoy",
		SPIFFE: &robotv1alpha1.SPIFFEConfig{
			TrustDomain:     "production.example.org",
			AgentSocketPath: "/run/spire/sockets/agent.sock",
		},
		Upstreams: []robotv1alpha1.ProxyUpstream{
			{
				Name:      "secure-db",
				LocalPort: 5432,
				Upstream:  "postgres.prod.svc:5432",
				Protocol:  "tcp",
				TLS:       true,
			},
		},
	}

	config, err := GenerateEnvoyConfig("secure-task", network)
	if err != nil {
		t.Fatalf("failed to generate config: %v", err)
	}

	configStr := string(config)

	// 验证 SPIFFE 相关配置
	expectedFields := []string{
		"production.example.org",
		"/run/spire/sockets/agent.sock",
		"spiffe://", // SPIFFE ID
		"SdsSecretConfig",
	}

	for _, field := range expectedFields {
		if !contains(configStr, field) {
			t.Errorf("config missing expected field: %s", field)
		}
	}

	t.Log("SPIFFE integration config validated successfully")
}

// TestEnvoyConfigProtocols 测试不同协议的配置
func TestEnvoyConfigProtocols(t *testing.T) {
	protocols := []string{"tcp", "http", "grpc"}

	for _, proto := range protocols {
		t.Run("protocol_"+proto, func(t *testing.T) {
			network := &robotv1alpha1.NetworkProxy{
				Enabled: true,
				Type:    "envoy",
				SPIFFE: &robotv1alpha1.SPIFFEConfig{
					TrustDomain:     "example.org",
					AgentSocketPath: "/run/spire/sockets/agent.sock",
				},
				Upstreams: []robotv1alpha1.ProxyUpstream{
					{
						Name:      "test-service",
						LocalPort: 8080,
						Upstream:  "service.default.svc:8080",
						Protocol:  proto,
						TLS:       true,
					},
				},
			}

			config, err := GenerateEnvoyConfig("proto-test", network)
			if err != nil {
				t.Fatalf("failed to generate config for protocol %s: %v", proto, err)
			}

			if len(config) == 0 {
				t.Errorf("empty config for protocol %s", proto)
			}

			t.Logf("Protocol %s config generated successfully (%d bytes)", proto, len(config))
		})
	}
}

// contains 检查字符串是否包含子串
func contains(s, substr string) bool {
	return len(s) > 0 && len(substr) > 0 &&
		(s == substr || len(s) >= len(substr) && hasSubstring(s, substr))
}

func hasSubstring(s, substr string) bool {
	for i := 0; i <= len(s)-len(substr); i++ {
		if s[i:i+len(substr)] == substr {
			return true
		}
	}
	return false
}
