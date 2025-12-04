# SPIRE Plugin for K8s4R

K8s4R 的 SPIRE 集成插件，为边缘节点上的 Task 提供零信任身份认证和自动化证书管理。

## 特性

✅ **无需修改 SPIRE 代码**：通过配置和插件扩展实现集成  
✅ **Linux Keyring 保护**：在无硬件 TPM 环境中提供内核级私钥保护  
✅ **插件化架构**：方便未来扩展其他组件（Envoy、Vault 等）  
✅ **自动化管理**：SPIRE Agent 进程自动启动、监控、重启  
✅ **健康检查**：持续监控 Agent 状态，确保服务可用  
✅ **零配置使用**：Task 自动获取运行时配置  

## 快速开始

### 1. 安装 SPIRE Agent 二进制文件

```bash
# 下载 SPIRE release
wget https://github.com/spiffe/spire/releases/download/v1.8.0/spire-1.8.0-linux-amd64-musl.tar.gz

# 解压
tar -xzf spire-1.8.0-linux-amd64-musl.tar.gz

# 安装到系统路径
sudo cp spire-1.8.0/bin/spire-agent /usr/local/bin/
sudo chmod +x /usr/local/bin/spire-agent
```

### 2. 配置 SPIRE Plugin

编辑 `config/agent/plugins.yaml`：

```yaml
spire:
  enabled: true
  serverAddress: "spire-server.example.com:8081"
  trustDomain: "robot.example.org"
  socketPath: "/run/spire/agent.sock"
  dataDir: "/var/lib/k8s4r/plugins/spire"
  logLevel: "INFO"
  
  nodeAttestor:
    type: "join_token"
  
  keyring:
    enabled: true
    type: "session"
  
  joinToken: "YOUR-JOIN-TOKEN-HERE"
```

### 3. 在 Agent 中启用插件

```go
// cmd/agent/main.go

import (
    "github.com/hxndghxndg/k8s4r/pkg/plugin"
    "github.com/hxndghxndg/k8s4r/pkg/plugin/spire"
)

func main() {
    // ... Agent 初始化代码 ...
    
    // 创建 Plugin Manager
    pluginMgr := plugin.NewManager(plugin.ManagerConfig{
        Logger: logger,
    })
    
    // 注册 SPIRE Plugin
    spirePlugin := spire.NewPlugin()
    if err := pluginMgr.Register(spirePlugin); err != nil {
        log.Fatal(err)
    }
    
    // 加载并初始化插件
    pluginConfigs := loadPluginConfigs("config/agent/plugins.yaml")
    if err := pluginMgr.Initialize(ctx, pluginConfigs); err != nil {
        log.Fatal(err)
    }
    
    // 启动所有插件
    if err := pluginMgr.Start(ctx); err != nil {
        log.Fatal(err)
    }
    
    // ... Agent 正常运行 ...
    
    // 停止时清理
    defer pluginMgr.Stop(ctx)
}
```

### 4. 在 Task 中使用 SPIRE

Task 通过 Workload API 获取自己的身份证书：

```go
package main

import (
    "context"
    "log"
    
    "github.com/spiffe/go-spiffe/v2/workloadapi"
)

func main() {
    ctx := context.Background()
    
    // 连接到 Workload API
    source, err := workloadapi.NewX509Source(ctx)
    if err != nil {
        log.Fatalf("Unable to create X509Source: %v", err)
    }
    defer source.Close()
    
    // 获取 SVID（身份证书）
    svid, err := source.GetX509SVID()
    if err != nil {
        log.Fatalf("Unable to get SVID: %v", err)
    }
    
    log.Printf("My SPIFFE ID: %s", svid.ID)
    
    // 使用证书建立 mTLS 连接
    // ...
}
```

## 配置选项

### 节点认证方式

| 类型 | 描述 | 适用场景 |
|------|------|----------|
| `join_token` | 预共享 token | 开发/测试，简单场景 |
| `x509pop` | X.509 证书 | 已有 PKI 基础设施 |
| `aws_iid` | AWS Instance Identity | AWS EC2 |
| `gcp_iit` | GCP Instance Identity | Google Cloud |
| `azure_msi` | Azure Managed Identity | Azure VM |

### Keyring 类型

| 类型 | 描述 | 隔离级别 |
|------|------|----------|
| `session` | Session Keyring（推荐） | 登录会话级别 |
| `user` | User Keyring | 用户级别 |
| `process` | Process Keyring | 进程级别 |

## 架构

详细架构说明请参考：[SPIRE_ARCHITECTURE.md](../../../docs/SPIRE_ARCHITECTURE.md)

```
K8s4R Agent
  ↓
Plugin Manager
  ↓
SPIRE Plugin
  ├─ Agent Manager  → 管理 SPIRE Agent 进程
  └─ Keyring Manager → Linux Keyring 保护私钥
      ↓
SPIRE Agent (独立进程)
  ├─ Workload API (/run/spire/agent.sock)
  └─ Node Attestation
      ↓
SPIRE Server (中心化)
```

## 运维

### 查看 Agent 日志

```bash
tail -f /var/lib/k8s4r/plugins/spire/agent.log
```

### 查看 Agent 状态

```bash
# 检查进程
ps aux | grep spire-agent

# 检查 socket
ls -la /run/spire/agent.sock

# 检查 keyring (需要 root)
sudo keyctl show @s
```

### 健康检查

SPIRE Plugin 自动进行健康检查（默认每 30 秒）：

- ✅ Agent 进程存活
- ✅ Socket 文件可用
- ✅ Workload API 响应

### 故障排查

**问题：Agent 无法启动**

```bash
# 检查配置文件
cat /var/lib/k8s4r/plugins/spire/config/agent.conf

# 检查日志
tail -f /var/lib/k8s4r/plugins/spire/agent.log

# 手动启动测试
/usr/local/bin/spire-agent run -config /var/lib/k8s4r/plugins/spire/config/agent.conf
```

**问题：无法连接 Server**

```bash
# 测试网络连接
telnet spire-server.example.com 8081

# 检查 Server 日志
```

**问题：Keyring 权限**

```bash
# 查看当前 keyring
keyctl show

# 查看特定密钥
keyctl print <keyid>

# 如果权限不足，需要以正确的用户运行 Agent
```

## 开发

### 运行测试

```bash
# 运行所有测试
go test ./pkg/plugin/spire/... -v

# 只运行配置测试
go test ./pkg/plugin/spire/... -v -run TestConfig

# 跳过需要 SPIRE Server 的集成测试
SKIP_SPIRE_INTEGRATION=1 go test ./pkg/plugin/spire/... -v

# 跳过 Keyring 测试（在不支持的平台上）
SKIP_KEYRING_TEST=1 go test ./pkg/plugin/spire/... -v
```

### 构建

```bash
make build
```

## 未来计划

- [ ] SPIRE KeyManager Plugin（直接使用 Keyring，不经过文件系统）
- [ ] Workload API 代理（访问控制、审计、缓存）
- [ ] 与 Envoy 集成（SDS API）
- [ ] Federation 支持（跨信任域）
- [ ] 自动 Workload Registration
- [ ] Webhook 集成（自动注入配置）

## 参考资料

- [SPIRE 官方文档](https://spiffe.io/docs/latest/spire-about/)
- [Linux Keyring API](https://man7.org/linux/man-pages/man7/keyrings.7.html)
- [Go SPIFFE Library](https://github.com/spiffe/go-spiffe)

## License

Apache 2.0
