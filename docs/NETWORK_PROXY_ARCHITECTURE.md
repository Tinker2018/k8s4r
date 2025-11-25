# 网络代理架构说明

## 概述

k8s4r 的网络代理功能采用 **initTasks（通用初始化） + network（声明式配置）** 的设计模式。

## 架构设计

### 1. 职责分离

```
┌─────────────────────────────────────────────────────────────┐
│                        TaskGroup                             │
├─────────────────────────────────────────────────────────────┤
│  initTasks:          (通用初始化)                            │
│    - 创建目录结构                                             │
│    - 下载二进制文件（SPIRE, Envoy）                          │
│    - 生成配置文件                                             │
│    - 启动守护进程                                             │
│                                                              │
│  network:            (声明式配置)                            │
│    - 定义代理规则                                             │
│    - 定义上游服务                                             │
│    - 定义安全策略                                             │
│                                                              │
│  tasks:              (业务逻辑)                              │
│    - 应用程序代码                                             │
└─────────────────────────────────────────────────────────────┘
```

### 2. 执行流程

```
Controller 创建 TaskGroup
         ↓
创建多个 Task 实例（每个携带相同的 network 配置）
         ↓
Agent 收到第一个 Task
         ↓
┌────────────────────────────────────────────┐
│ 1. 执行 initTasks (per TaskGroup)         │
│    ✓ prepare-directories                  │
│    ✓ generate-spire-config                │
│    ✓ install-spire                        │
│    ✓ start-spire-agent (daemon)           │
│    ✓ install-envoy                        │
└────────────────────────────────────────────┘
         ↓
┌────────────────────────────────────────────┐
│ 2. 生成 Envoy 配置 (声明式 → 实际配置)    │
│    从 task.Spec.Network 读取:              │
│      - upstreams                           │
│      - spiffe 配置                         │
│    生成 Envoy YAML:                        │
│      - listeners (127.0.0.1:5432)         │
│      - clusters (db.example.com:5432)     │
│      - TLS/mTLS 配置                      │
└────────────────────────────────────────────┘
         ↓
┌────────────────────────────────────────────┐
│ 3. 启动 Envoy 代理                         │
│    /opt/k8s4r/bin/envoy -c config.yaml    │
│    (daemon process, per TaskGroup)         │
└────────────────────────────────────────────┘
         ↓
┌────────────────────────────────────────────┐
│ 4. 启动业务 Task                           │
│    业务进程连接 127.0.0.1:5432             │
│    Envoy 代理到 db.example.com:5432        │
└────────────────────────────────────────────┘
         ↓
Agent 收到第二个 Task (同一 TaskGroup)
         ↓
┌────────────────────────────────────────────┐
│ 1. initTasks 已完成 ✓ (跳过)              │
│ 2. Envoy 已启动 ✓ (跳过)                  │
│ 3. 直接启动业务 Task                       │
│    所有 Task 共享同一个 Envoy 实例         │
└────────────────────────────────────────────┘
```

## 网络可见性

### 问题：TaskGroup 内的 Task 如何共享网络代理？

**答案**：通过 localhost (127.0.0.1) + 共享的 Envoy 进程

```
┌─────────────────────────────────────────────────────────┐
│                Robot (同一台物理机)                      │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  ┌──────────────────────────────────────────────┐      │
│  │ SPIRE Agent (守护进程)                        │      │
│  │   Unix Socket: /run/spire/sockets/agent.sock │      │
│  └──────────────────────────────────────────────┘      │
│                        ↑                                │
│  ┌─────────────────────┴──────────────────────┐        │
│  │ Envoy Proxy (守护进程, per TaskGroup)       │        │
│  │   Listener: 127.0.0.1:5432                  │        │
│  │   Upstream: db.example.com:5432 (mTLS)      │        │
│  └──────────────────────────────────────────────┘      │
│           ↑                    ↑                        │
│           │                    │                        │
│  ┌────────┴────────┐  ┌────────┴────────┐              │
│  │ Task r0-app     │  │ Task r1-app     │              │
│  │ 连接 127.0.0.1: │  │ 连接 127.0.0.1: │              │
│  │      5432       │  │      5432       │              │
│  └─────────────────┘  └─────────────────┘              │
│                                                         │
└─────────────────────────────────────────────────────────┘
                         │
                         │ (mTLS over Internet)
                         ↓
              ┌──────────────────────┐
              │ db.example.com:5432  │
              └──────────────────────┘
```

### 关键点

1. **Envoy 监听 127.0.0.1**（而不是 0.0.0.0）
   - ✅ 只有同一台机器上的进程能访问
   - ✅ 避免端口冲突
   - ✅ 安全性更好

2. **Per TaskGroup 的 Envoy 实例**
   - 同一个 TaskGroup 的所有 Task 共享一个 Envoy
   - 不同 TaskGroup 有独立的 Envoy 实例
   - 通过 `daemonProcesses[taskGroupName-envoy]` 管理

3. **SPIRE Agent 全局共享**
   - 一个 Robot 上只有一个 SPIRE Agent
   - 所有 Envoy 实例都连接到同一个 SPIRE Agent
   - 通过 Unix Socket 通信

## 配置声明式转换

### 用户编写（声明式）

```yaml
network:
  enabled: true
  spiffe:
    trustDomain: example.org
    agentSocketPath: /run/spire/sockets/agent.sock
  upstreams:
    - name: database
      localPort: 5432
      upstream: db.example.com:5432
      protocol: tcp
      tls: true
```

### Agent 生成（实际配置）

```yaml
# /home/robot/.k8s4r/tasks/envoy/envoy-proxy-group.yaml
admin:
  address:
    socket_address:
      address: 127.0.0.1
      port_value: 19000

static_resources:
  listeners:
    - name: database_listener
      address:
        socket_address:
          address: 127.0.0.1    # 只监听 localhost
          port_value: 5432
      filter_chains:
        - filters:
            - name: envoy.filters.network.tcp_proxy
              typed_config:
                "@type": type.googleapis.com/envoy.extensions.filters.network.tcp_proxy.v3.TcpProxy
                stat_prefix: database
                cluster: database_cluster
          transport_socket:
            name: envoy.transport_sockets.tls
            typed_config:
              "@type": type.googleapis.com/envoy.extensions.transport_sockets.tls.v3.DownstreamTlsContext
              common_tls_context:
                tls_certificate_sds_secret_configs:
                  - name: spiffe://example.org/workload
                    sds_config:
                      api_config_source:
                        api_type: GRPC
                        grpc_services:
                          - envoy_grpc:
                              cluster_name: spire_agent

  clusters:
    - name: database_cluster
      connect_timeout: 1s
      type: STRICT_DNS
      load_assignment:
        cluster_name: database_cluster
        endpoints:
          - lb_endpoints:
              - endpoint:
                  address:
                    socket_address:
                      address: db.example.com
                      port_value: 5432
      transport_socket:
        name: envoy.transport_sockets.tls
        typed_config:
          "@type": type.googleapis.com/envoy.extensions.transport_sockets.tls.v3.UpstreamTlsContext
          common_tls_context:
            tls_certificate_sds_secret_configs:
              - name: spiffe://example.org/workload
                sds_config:
                  api_config_source:
                    api_type: GRPC
                    grpc_services:
                      - envoy_grpc:
                          cluster_name: spire_agent
    
    - name: spire_agent
      connect_timeout: 1s
      type: STATIC
      load_assignment:
        cluster_name: spire_agent
        endpoints:
          - lb_endpoints:
              - endpoint:
                  address:
                    pipe:
                      path: /run/spire/sockets/agent.sock
```

## 代码实现

### 配置生成（pkg/agent/envoy_config.go）

```go
func generateEnvoyConfig(taskGroupName string, network *NetworkProxy, configDir string) (string, error) {
    // 1. 读取声明式配置
    spiffeConfig := network.SPIFFE
    upstreams := network.Upstreams
    
    // 2. 生成 listeners (监听 127.0.0.1)
    for _, upstream := range upstreams {
        listener := generateListener(upstream, spiffeConfig)
        // listener.address = 127.0.0.1:{upstream.LocalPort}
    }
    
    // 3. 生成 clusters (连接上游服务)
    for _, upstream := range upstreams {
        cluster := generateCluster(upstream, spiffeConfig)
        // cluster.upstream = {upstream.Upstream}
        // cluster.tls = SPIFFE mTLS
    }
    
    // 4. 写入配置文件
    configPath := filepath.Join(configDir, fmt.Sprintf("envoy-%s.yaml", taskGroupName))
    os.WriteFile(configPath, config, 0644)
    
    return configPath, nil
}
```

### 代理启动（pkg/agent/task_executor.go）

```go
func (te *TaskExecutor) createTask(ctx context.Context, task *Task) {
    // 1. 执行 initTasks
    if err := te.executeInitTasks(ctx, taskGroupName, task); err != nil {
        return
    }
    
    // 2. 如果启用了网络代理
    if task.Spec.Network != nil && task.Spec.Network.Enabled {
        // 检查 Envoy 是否已启动
        envoyKey := fmt.Sprintf("%s-envoy", taskGroupName)
        if !te.daemonProcesses[envoyKey] {
            // 生成 Envoy 配置
            configPath, _ := generateEnvoyConfig(taskGroupName, task.Spec.Network, configDir)
            
            // 启动 Envoy
            handle, _ := te.driver.Start(ctx, envoyTask)
            te.daemonProcesses[envoyKey] = handle
        }
    }
    
    // 3. 启动业务 Task
    te.driver.Start(ctx, task)
}
```

## 使用示例

参见 `examples/job-with-envoy-proxy.yaml`

业务代码只需连接 localhost：

```go
// 业务代码
db, err := sql.Open("postgres", "postgresql://127.0.0.1:5432/mydb")
// Envoy 自动代理到 db.example.com:5432，并提供 mTLS
```

## 优势

1. ✅ **简单**：用户只需声明 upstreams，不需要懂 Envoy
2. ✅ **安全**：自动 mTLS，基于 SPIFFE 身份认证
3. ✅ **高效**：TaskGroup 级别复用，避免重复启动
4. ✅ **隔离**：只监听 localhost，避免端口冲突
5. ✅ **灵活**：支持 TCP/HTTP/gRPC 多种协议

## 后续改进

- [ ] 支持动态更新 Envoy 配置（无需重启）
- [ ] 支持健康检查和自动重启
- [ ] 支持多个 SPIRE Agent（不同 trust domain）
- [ ] 支持 Envoy 访问控制策略
