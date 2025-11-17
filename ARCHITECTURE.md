# K8S4R 架构说明

## 系统架构

```
┌─────────────────────────────────────────────────────────────┐
│                    Kubernetes Cluster                        │
│                                                              │
│  ┌──────────────────────┐      ┌────────────────────────┐  │
│  │   Robot CRD          │      │   Manager (Controller) │  │
│  │                      │◄─────│                        │  │
│  │  apiVersion:         │      │  - 监控 Robot 资源      │  │
│  │  robot.k8s4r.io/v1alpha1   │  - 检查心跳超时        │  │
│  │  kind: Robot         │      │  - 更新状态           │  │
│  │                      │      └────────────────────────┘  │
│  │  spec:               │                                   │
│  │    robotId           │      ┌────────────────────────┐  │
│  │  status:             │      │   Server (HTTP API)    │  │
│  │    phase             │◄─────│                        │  │
│  │    lastHeartbeatTime │      │  - 接收注册请求        │  │
│  └──────────────────────┘      │  - 接收心跳请求        │  │
│                                 │  - 更新 Robot 资源     │  │
│                                 └────────┬───────────────┘  │
└──────────────────────────────────────────┼──────────────────┘
                                           │
                                           │ HTTP
                                           │
                        ┌──────────────────┼──────────────────┐
                        │                  │                  │
                ┌───────▼────────┐  ┌──────▼───────┐  ┌──────▼───────┐
                │  Agent         │  │  Agent       │  │  Agent       │
                │  (Robot 001)   │  │  (Robot 002) │  │  (Robot 003) │
                │                │  │              │  │              │
                │  - 注册        │  │  - 注册      │  │  - 注册      │
                │  - 心跳(30s)   │  │  - 心跳(30s) │  │  - 心跳(30s) │
                └────────────────┘  └──────────────┘  └──────────────┘
```

## 数据流

### 1. Agent 注册流程

```
Agent                Server               Kubernetes API
  │                     │                        │
  │──Register Request──►│                        │
  │  {robotId, token}   │                        │
  │                     │──Check Robot exists───►│
  │                     │                        │
  │                     │◄──Not Found────────────│
  │                     │                        │
  │                     │──Create Robot──────────►│
  │                     │  spec.robotId=xxx      │
  │                     │                        │
  │                     │──Update Status─────────►│
  │                     │  phase=Online          │
  │                     │  lastHeartbeatTime=now │
  │                     │                        │
  │◄──Success───────────│                        │
  │                     │                        │
```

### 2. Agent 心跳流程

```
Agent                Server               Kubernetes API
  │                     │                        │
  │──Heartbeat Request─►│                        │
  │  {robotId, token}   │                        │
  │                     │──Get Robot─────────────►│
  │                     │                        │
  │                     │◄──Robot object─────────│
  │                     │                        │
  │                     │──Update Status─────────►│
  │                     │  lastHeartbeatTime=now │
  │                     │                        │
  │◄──Success───────────│                        │
  │                     │                        │
  │                     │                        │
  │  (30秒后)            │                        │
  │──Heartbeat Request─►│                        │
  │                     │                        │
```

### 3. Controller 监控流程

```
Controller                    Kubernetes API
    │                              │
    │──Watch Robot resources──────►│
    │                              │
    │◄─Robot event (every 30s)─────│
    │                              │
    │──Check lastHeartbeatTime     │
    │                              │
    │  if (now - lastHeartbeat > 5min)
    │      ↓                        │
    │  Robot is offline            │
    │                              │
    │──Update Status───────────────►│
    │  phase=Offline               │
    │  message="Heartbeat timeout" │
    │                              │
```

## 关键常量

- **Token**: `fixed-token-123` (硬编码在 `pkg/server/server.go`)
- **心跳间隔**: 30秒 (Agent 发送心跳的频率)
- **心跳超时**: 5分钟 (Controller 判定离线的阈值)
- **Controller 检查周期**: 30秒 (Controller reconcile 频率)

## Robot 资源状态转换

```
    创建 Robot
        │
        ▼
   ┌─────────┐
   │ Pending │  ← 等待 Agent 注册
   └────┬────┘
        │ Agent 注册成功
        ▼
   ┌────────┐
   │ Online │  ← Agent 正常发送心跳
   └────┬───┘
        │ 5分钟无心跳
        ▼
   ┌─────────┐
   │ Offline │  ← Controller 检测到超时
   └────┬────┘
        │ Agent 重新发送心跳
        ▼
   ┌────────┐
   │ Online │
   └────────┘
```

## 组件职责

| 组件 | 职责 | 运行位置 |
|------|------|---------|
| **Robot CRD** | 定义 Robot 资源的数据结构 | Kubernetes API Server |
| **Manager** | 监控 Robot 资源，检测心跳超时 | Kubernetes 集群内（或本地） |
| **Server** | 提供 HTTP API，处理 Agent 请求 | Kubernetes 集群内（或本地） |
| **Agent** | 注册并定时上报心跳 | 机器人设备上 |

## 扩展点

后续可以扩展的功能：

1. **设备信息采集**: 在 RobotStatus 中添加 IP、Hostname、OS、内存、CPU 等字段
2. **Binary 分发**: 添加 Deployment 资源，支持向 Robot 分发二进制文件
3. **命令下发**: 添加 Command API，支持向 Robot 发送控制命令
4. **日志采集**: Agent 上报日志到中心化存储
5. **指标监控**: 集成 Prometheus，监控 Robot 状态
6. **多 Token 认证**: 支持为每个 Robot 生成独立 Token
7. **TLS/HTTPS**: Server 支持 HTTPS 加密通信
