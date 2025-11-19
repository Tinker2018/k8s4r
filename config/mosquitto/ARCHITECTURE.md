# K8S4R MQTT 架构说明

## 协议升级说明

K8S4R 已从 HTTP 协议升级到 MQTT 协议，以支持更好的弱网环境和实时通信。

## MQTT 协议优势

1. **弱网支持**：MQTT 协议专为低带宽、高延迟、不可靠网络设计
2. **实时性**：基于发布/订阅模式，支持实时消息推送
3. **低开销**：协议开销小，适合资源受限的设备
4. **自动重连**：内置断线重连机制
5. **QoS 保证**：支持不同级别的消息质量保证

## MQTT 主题设计

### 核心主题

- `k8s4r/register` - Agent 注册请求
- `k8s4r/heartbeat` - Agent 心跳请求
- `k8s4r/response/{robotId}` - 服务器对特定 Robot 的响应
- `k8s4r/commands/{robotId}` - 服务器向特定 Robot 发送的命令

### 消息流程

1. **Agent 启动**：
   - 连接到 MQTT Broker
   - 订阅响应主题：`k8s4r/response/{robotId}`
   - 订阅命令主题：`k8s4r/commands/{robotId}`

2. **Agent 注册**：
   - 发布消息到：`k8s4r/register`
   - 等待服务器响应

3. **心跳机制**：
   - 定期发布消息到：`k8s4r/heartbeat`
   - 无需等待响应（提高效率）

4. **服务器响应**：
   - 监听注册和心跳主题
   - 发布响应到：`k8s4r/response/{robotId}`

## 部署架构

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   K8s Manager   │    │   MQTT Broker   │    │    Robot Agent  │
│                 │    │                 │    │                 │
│  ┌─────────────┐│    │  ┌─────────────┐│    │  ┌─────────────┐│
│  │ Controller  ││    │  │ Mosquitto   ││    │  │ MQTT Client ││
│  │ Reconciler  ││    │  │ /Eclipse    ││    │  │             ││
│  └─────────────┘│    │  └─────────────┘│    │  └─────────────┘│
│         │        │    │         │       │    │         │       │
│  ┌─────────────┐│    │         │       │    │  ┌─────────────┐│
│  │K8s4R Server││◄───┼─────────┼───────┼────┤  │Device Info  ││
│  │MQTT Client  ││    │         │       │    │  │Collector    ││
│  └─────────────┘│    │         │       │    │  └─────────────┘│
│         │        │    │         │       │    │                 │
│  ┌─────────────┐│    │         │       │    │                 │
│  │  Robot CRD  ││    │         │       │    │                 │
│  │  Resources  ││    │         │       │    │                 │
│  └─────────────┘│    │         │       │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## 配置示例

### MQTT Broker 部署

使用 Docker 启动：
```bash
docker run -it -p 1883:1883 -p 9001:9001 eclipse-mosquitto:2.0
```

使用 Kubernetes 部署：
```bash
kubectl apply -f config/server/deployment.yaml
```

### Server 启动

```bash
./server --broker-url=tcp://mqtt-broker:1883 --namespace=default
```

### Agent 启动

```bash
./agent --broker-url=tcp://mqtt-broker:1883 --robot-id=robot-001 --token=fixed-token-123
```

## 消息格式

### 注册请求（Agent → Server）
主题：`k8s4r/register`
```json
{
  "robotId": "robot-001",
  "token": "fixed-token-123",
  "deviceInfo": {
    "hostname": "robot-001",
    "platform": "linux",
    "cpu": {...},
    "memory": {...},
    "disk": [...],
    "network": [...]
  }
}
```

### 心跳请求（Agent → Server）
主题：`k8s4r/heartbeat`
```json
{
  "robotId": "robot-001",
  "token": "fixed-token-123",
  "deviceInfo": {
    "hostname": "robot-001",
    "platform": "linux",
    "cpu": {...},
    "memory": {...},
    "disk": [...],
    "network": [...]
  }
}
```

### 响应消息（Server → Agent）
主题：`k8s4r/response/{robotId}`
```json
{
  "success": true,
  "message": "Robot registered successfully",
  "robotId": "robot-001"
}
```

## 故障恢复

1. **网络断开**：MQTT 客户端自动重连
2. **Broker 重启**：客户端检测连接丢失并重新连接
3. **消息丢失**：使用 QoS 1 保证消息至少传递一次
4. **Agent 重启**：重新注册并恢复心跳

## 监控和调试

### MQTT 客户端工具

使用 mosquitto 客户端监听消息：
```bash
# 监听所有主题
mosquitto_sub -h localhost -t "k8s4r/#" -v

# 监听特定机器人的响应
mosquitto_sub -h localhost -t "k8s4r/response/robot-001" -v

# 发送测试命令
mosquitto_pub -h localhost -t "k8s4r/commands/robot-001" -m "ping"
```

### 日志输出

Server 和 Agent 都会输出详细的 MQTT 连接和消息处理日志。