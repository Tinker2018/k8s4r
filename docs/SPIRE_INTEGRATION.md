# K8s4r SPIRE 集成快速指南

## 概述

K8s4r 已集成真实的 SPIRE (SPIFFE Runtime Environment) 用于：
- **Agent → MQTT**: mTLS 认证（端口 8883）
- **Server → MQTT**: 普通连接（端口 1883）
- **自动证书管理**: 通过 SPIRE Workload API

## 架构

```
┌─────────────────┐
│  SPIRE Server   │ 监听 :8081
│  (真实进程)      │
└────────┬────────┘
         │ Node Attestation (join_token)
         │
┌────────▼────────┐
│  SPIRE Agent    │ /tmp/spire-agent/agent.sock
│  (真实进程)      │
└────────┬────────┘
         │ Workload API (gRPC)
         │
┌────────▼────────┐              ┌─────────────┐
│  K8s4r Agent    │──mTLS:8883──>│  Mosquitto  │
│                 │              │             │
└─────────────────┘              │  Plain:1883 │
                                 └──────▲──────┘
                                        │
                                 ┌──────┴──────┐
                                 │ K8s4r Server│
                                 └─────────────┘
```

## 快速开始

### 前提条件

1. 安装 SPIRE Server 和 Agent 二进制文件：
   ```bash
   # 方法 1: 从官方下载
   wget https://github.com/spiffe/spire/releases/download/v1.9.0/spire-1.9.0-linux-amd64-musl.tar.gz
   tar xzf spire-1.9.0-linux-amd64-musl.tar.gz
   sudo cp spire-1.9.0/bin/spire-server /usr/local/bin/
   sudo cp spire-1.9.0/bin/spire-agent /usr/local/bin/
   
   # 方法 2: 使用系统包管理器（如果可用）
   # sudo apt install spire / brew install spire
   ```

2. 确保 Docker 运行（用于 Mosquitto）

### 一键启动

```bash
# 构建所有组件
make build-all

# 启动完整环境（SPIRE + Mosquitto + Server + Agent）
make dev-start

# 查看状态
make dev-status

# 查看日志
make dev-logs
```

### 手动步骤

如果你想逐步理解每个组件：

```bash
# 1. 构建
make build

# 2. 启动环境
./scripts/manage-dev-env.sh start

# 3. 监控日志
tail -f .dev-env/spire-server.log  # SPIRE Server
tail -f .dev-env/agent.log          # K8s4r Agent (包含 SPIRE Agent 启动信息)
tail -f .dev-env/server.log         # K8s4r Server

# 4. 验证 SPIRE
spire-server healthcheck            # 检查 SPIRE Server
ls -la /tmp/spire-agent/agent.sock  # 验证 Workload API Socket
cat config/mosquitto/certs/spire-ca.crt  # 查看导出的 CA
```

## 测试

### 端到端集成测试

```bash
make test-spire-e2e
```

此命令会：
1. 构建所有组件
2. 启动完整环境
3. 验证每个组件状态
4. 检查 SPIRE 认证流程
5. 验证 CA Bundle 导出
6. 检查 MQTT 连接

### 单元测试

```bash
# SPIRE 配置测试
make test-spire

# 集成测试（使用 Mock SPIRE）
make test-spire-integration
```

## 故障排查

### SPIRE Server 启动失败

```bash
# 查看日志
cat .dev-env/spire-server.log

# 检查端口占用
lsof -i :8081

# 手动启动测试
spire-server run -config config/spire/server.conf
```

### SPIRE Agent 未连接

```bash
# 查看 Agent 日志
cat .dev-env/agent.log | grep -i spire

# 检查 Socket
ls -la /tmp/spire-agent/agent.sock

# 验证 Join Token
echo $SPIRE_JOIN_TOKEN

# 手动测试 Workload API
spire-agent api fetch -socketPath /tmp/spire-agent/agent.sock
```

### MQTT mTLS 连接失败

```bash
# 检查 CA Bundle
ls -la config/mosquitto/certs/spire-ca.crt

# 查看 Mosquitto 日志
docker logs mosquitto

# 手动测试连接
openssl s_client -connect localhost:8883 \
  -CAfile config/mosquitto/certs/spire-ca.crt \
  -cert <(spire-agent api fetch -socketPath /tmp/spire-agent/agent.sock -write /dev/stdout) \
  -key <(spire-agent api fetch -socketPath /tmp/spire-agent/agent.sock -write /dev/stdout)
```

## 配置

### 插件配置 (`config/agent/plugins.yaml`)

```yaml
spire:
  socketPath: "/tmp/spire-agent/agent.sock"
  trustDomain: "k8s4r.example.org"
  serverAddr: "localhost:8081"
  joinToken: "${SPIRE_JOIN_TOKEN}"  # 从环境变量读取
  dataDir: "./.spire-data/agent"
  logLevel: "DEBUG"
```

### SPIRE Server 配置 (`config/spire/server.conf`)

- **数据目录**: `./.spire-data/server`
- **监听端口**: 8081
- **信任域**: `k8s4r.example.org`
- **节点认证**: `join_token`

### SPIRE Agent 配置

Agent 配置由 K8s4r Agent 自动生成，基于 `plugins.yaml` 配置。

## 环境变量

| 变量 | 说明 | 示例 |
|------|------|------|
| `SPIRE_JOIN_TOKEN` | SPIRE Agent 注册 Token | 自动生成 |
| `ROBOT_ID` | 机器人标识符 | robot-001 |
| `MQTT_BROKER_URL` | MQTT Broker 地址 | ssl://localhost:8883 |

## 清理

```bash
# 停止所有进程
make dev-stop

# 停止并清理数据
make dev-clean
```

清理操作会删除：
- `.dev-env/` - 日志和PID文件
- `.spire-data/` - SPIRE 数据目录
- `/tmp/spire-agent/` - Workload API Socket
- Docker Mosquitto 容器

## 生产部署注意事项

当前配置适用于**开发环境**，生产部署需要：

1. **禁用 insecure_bootstrap**
   ```hcl
   agent {
     insecure_bootstrap = false  # 生产环境必须关闭
   }
   ```

2. **使用更安全的节点认证**
   - AWS IID Attestor (AWS 环境)
   - GCP IIT Attestor (GCP 环境)
   - x509pop (基于证书)

3. **持久化存储**
   - 使用持久化卷存储 SPIRE 数据
   - 定期备份 CA 密钥

4. **高可用部署**
   - 使用 PostgreSQL/MySQL 作为 DataStore
   - 部署多个 SPIRE Server 实例

5. **监控和告警**
   - 监控 SPIRE Server 健康状态
   - 监控证书过期时间
   - 监控 Workload API 可用性

## 参考文档

- [SPIRE 官方文档](https://spiffe.io/docs/latest/spire/)
- [SPIFFE 规范](https://github.com/spiffe/spiffe)
- [K8s4r 架构文档](./ARCHITECTURE.md)
