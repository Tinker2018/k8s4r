package agent

import (
	"fmt"
	"os"
	"path/filepath"
	"strings"

	robotv1alpha1 "github.com/hxndghxndg/k8s4r/api/v1alpha1"
)

// generateEnvoyConfig 根据 NetworkProxy 配置动态生成 Envoy 配置文件
func generateEnvoyConfig(taskGroupName string, network *robotv1alpha1.NetworkProxy, configDir string) (string, error) {
	if network == nil || !network.Enabled {
		return "", fmt.Errorf("network proxy not enabled")
	}

	// 确保配置目录存在
	if err := os.MkdirAll(configDir, 0755); err != nil {
		return "", fmt.Errorf("failed to create config dir: %w", err)
	}

	configPath := filepath.Join(configDir, fmt.Sprintf("envoy-%s.yaml", taskGroupName))

	// 获取 SPIFFE 配置
	spiffeConfig := network.SPIFFE
	if spiffeConfig == nil {
		spiffeConfig = &robotv1alpha1.SPIFFEConfig{
			TrustDomain:     "example.org",
			AgentSocketPath: "/run/spire/sockets/agent.sock",
		}
	}

	// 生成 listeners 配置
	var listeners []string
	for _, upstream := range network.Upstreams {
		listener := generateListener(upstream, spiffeConfig)
		listeners = append(listeners, listener)
	}

	// 生成 clusters 配置
	var clusters []string
	for _, upstream := range network.Upstreams {
		cluster := generateCluster(upstream, spiffeConfig)
		clusters = append(clusters, cluster)
	}

	// 添加 SPIRE Agent cluster
	spireCluster := fmt.Sprintf(`    - name: spire_agent
      connect_timeout: 1s
      type: STATIC
      load_assignment:
        cluster_name: spire_agent
        endpoints:
          - lb_endpoints:
              - endpoint:
                  address:
                    pipe:
                      path: %s`, spiffeConfig.AgentSocketPath)
	clusters = append(clusters, spireCluster)

	// 组装完整配置
	config := fmt.Sprintf(`admin:
  address:
    socket_address:
      address: 127.0.0.1
      port_value: 19000

static_resources:
  listeners:
%s

  clusters:
%s
`, strings.Join(listeners, "\n"), strings.Join(clusters, "\n"))

	// 写入文件
	if err := os.WriteFile(configPath, []byte(config), 0644); err != nil {
		return "", fmt.Errorf("failed to write config file: %w", err)
	}

	return configPath, nil
}

// generateListener 生成单个 listener 配置
func generateListener(upstream robotv1alpha1.ProxyUpstream, spiffe *robotv1alpha1.SPIFFEConfig) string {
	protocol := upstream.Protocol
	if protocol == "" {
		protocol = "tcp"
	}

	if protocol == "http" || protocol == "grpc" {
		return generateHTTPListener(upstream, spiffe)
	}
	return generateTCPListener(upstream, spiffe)
}

// generateTCPListener 生成 TCP listener
func generateTCPListener(upstream robotv1alpha1.ProxyUpstream, spiffe *robotv1alpha1.SPIFFEConfig) string {
	tlsConfig := ""
	if upstream.TLS {
		tlsConfig = fmt.Sprintf(`        transport_socket:
          name: envoy.transport_sockets.tls
          typed_config:
            "@type": type.googleapis.com/envoy.extensions.transport_sockets.tls.v3.DownstreamTlsContext
            common_tls_context:
              tls_certificate_sds_secret_configs:
                - name: spiffe://%s/workload
                  sds_config:
                    api_config_source:
                      api_type: GRPC
                      grpc_services:
                        - envoy_grpc:
                            cluster_name: spire_agent`, spiffe.TrustDomain)
	}

	return fmt.Sprintf(`    - name: %s_listener
      address:
        socket_address:
          address: 127.0.0.1
          port_value: %d
      filter_chains:
        - filters:
            - name: envoy.filters.network.tcp_proxy
              typed_config:
                "@type": type.googleapis.com/envoy.extensions.filters.network.tcp_proxy.v3.TcpProxy
                stat_prefix: %s
                cluster: %s_cluster
%s`, upstream.Name, upstream.LocalPort, upstream.Name, upstream.Name, tlsConfig)
}

// generateHTTPListener 生成 HTTP/gRPC listener
func generateHTTPListener(upstream robotv1alpha1.ProxyUpstream, spiffe *robotv1alpha1.SPIFFEConfig) string {
	return fmt.Sprintf(`    - name: %s_listener
      address:
        socket_address:
          address: 127.0.0.1
          port_value: %d
      filter_chains:
        - filters:
            - name: envoy.filters.network.http_connection_manager
              typed_config:
                "@type": type.googleapis.com/envoy.extensions.filters.network.http_connection_manager.v3.HttpConnectionManager
                stat_prefix: %s
                codec_type: AUTO
                route_config:
                  name: local_route
                  virtual_hosts:
                    - name: %s
                      domains: ["*"]
                      routes:
                        - match:
                            prefix: "/"
                          route:
                            cluster: %s_cluster
                http_filters:
                  - name: envoy.filters.http.router
                    typed_config:
                      "@type": type.googleapis.com/envoy.extensions.filters.http.router.v3.Router`,
		upstream.Name, upstream.LocalPort, upstream.Name, upstream.Name, upstream.Name)
}

// generateCluster 生成单个 cluster 配置
func generateCluster(upstream robotv1alpha1.ProxyUpstream, spiffe *robotv1alpha1.SPIFFEConfig) string {
	// 解析 upstream 地址 (host:port)
	parts := strings.Split(upstream.Upstream, ":")
	if len(parts) != 2 {
		// 默认使用原始地址
		parts = []string{upstream.Upstream, "80"}
	}
	host := parts[0]
	port := parts[1]

	tlsConfig := ""
	if upstream.TLS {
		tlsConfig = fmt.Sprintf(`      transport_socket:
        name: envoy.transport_sockets.tls
        typed_config:
          "@type": type.googleapis.com/envoy.extensions.transport_sockets.tls.v3.UpstreamTlsContext
          common_tls_context:
            tls_certificate_sds_secret_configs:
              - name: spiffe://%s/workload
                sds_config:
                  api_config_source:
                    api_type: GRPC
                    grpc_services:
                      - envoy_grpc:
                          cluster_name: spire_agent`, spiffe.TrustDomain)
	}

	return fmt.Sprintf(`    - name: %s_cluster
      connect_timeout: 1s
      type: STRICT_DNS
      load_assignment:
        cluster_name: %s_cluster
        endpoints:
          - lb_endpoints:
              - endpoint:
                  address:
                    socket_address:
                      address: %s
                      port_value: %s
%s`, upstream.Name, upstream.Name, host, port, tlsConfig)
}
