# K8S4R - Kubernetes for Robots

一个基于 Kubernetes 的机器人设备管理系统，采用声明式范式管理机器人资源。使用 **MQTT 协议** 进行通信，支持弱网环境和实时消息推送。

## ✨ 核心功能

- ✅ **Robot CRD 资源** - Kubernetes 原生资源管理
- ✅ **MQTT 通信协议** - 支持弱网环境和实时通信
- ✅ **Agent 注册机制** - 基于 Token 的认证
- ✅ **心跳监控** - 自动检测离线状态（5分钟超时）
- ✅ **设备信息采集** - CPU、内存、磁盘、网络等系统信息
- ✅ **自动恢复** - Agent 重连后状态自动恢复
- ✅ **实时命令下发** - 基于MQTT的双向通信

## 🎯 MQTT 协议优势

相比传统的HTTP协议，MQTT协议提供：

1. **弱网支持** - 专为低带宽、高延迟、不可靠网络设计
2. **实时性** - 发布/订阅模式，支持实时消息推送
3. **低开销** - 协议开销小，适合资源受限的设备
4. **自动重连** - 内置断线重连机制
5. **QoS 保证** - 支持不同级别的消息质量保证

## 🚀 快速开始

### 前置条件

- Kubernetes 集群 (v1.24+)
- kubectl 命令行工具
- Docker (用于运行 MQTT Broker)
- Go 1.21+ (仅开发环境需要)

### 1. 安装 CRD

```bash
kubectl apply -f config/crd/robot.k8s4r.io_robots.yaml
```

### 2. 启动 MQTT Broker（Terminal 1）

```bash
# 推荐：使用配置脚本启动
./config/mosquitto/start-mosquitto.sh simple

# 或者使用开发模式（包含 WebSocket 支持）
./config/mosquitto/start-mosquitto.sh dev

# 检查状态
./config/mosquitto/start-mosquitto.sh status
```

### 3. 运行 Manager（Terminal 2）

```bash
# 开发模式
make run-manager

# 或直接运行
go run cmd/manager/main.go
```

### 4. 运行 Server（Terminal 3）

```bash
# 开发模式
make run-server

# 或直接运行
go run cmd/server/main.go --broker-url=tcp://localhost:1883 --namespace=default
```

### 5. 运行 Agent（Terminal 4）

```bash
# 开发模式
make run-agent

# 或直接运行
go run cmd/agent/main.go \
  --broker-url=tcp://localhost:1883 \
  --token=fixed-token-123 \
  --robot-id=robot-001
```

### 6. 监控和验证

```bash
# 查看 Robot 资源状态
kubectl get robots

# 查看详细信息
kubectl describe robot robot-001

# 监控 MQTT 消息（需要安装 mosquitto-clients）
mosquitto_sub -h localhost -p 1883 -t "k8s4r/#" -v

# 或使用测试脚本
./config/mosquitto/test-mqtt.sh listen
```

## 📊 设备信息采集

Agent 自动采集并上报设备信息到 Robot 资源的 Status 中：

| 信息类型 | 说明 | 示例 |
|---------|------|------|
| 主机名 | 设备的 hostname | `robot-001` |
| 操作系统 | OS 类型和架构 | `linux/amd64`, `darwin/arm64` |
| CPU | 核心数和使用率 | `4 cores, 25.5% usage` |
| 内存 | 总容量和已使用量 | `8192MB total, 4096MB used` |
| 磁盘 | 各分区容量和使用情况 | `/dev/sda1: 100GB total, 50GB used` |
| 网络 | 所有网卡的 IPv4 地址 | `eth0: 192.168.1.100` |

## 📡 MQTT 主题设计

K8S4R 使用以下 MQTT 主题进行通信：

| 主题 | 方向 | 说明 |
|------|------|------|
| `k8s4r/register` | Agent → Server | Agent 注册请求 |
| `k8s4r/heartbeat` | Agent → Server | Agent 心跳上报 |
| `k8s4r/response/{robotId}` | Server → Agent | 服务器响应消息 |
| `k8s4r/commands/{robotId}` | Server → Agent | 命令下发 |

## 🛠️ 开发指南

### 构建项目

```bash
# 构建所有组件
make build

# 生成 CRD manifests
make manifests

# 运行测试
make test

# 构建 Docker 镜像
make docker-build
```

### 本地开发

```bash
# 启动完整开发环境
./config/mosquitto/start-mosquitto.sh dev
make run-manager    # Terminal 1
make run-server     # Terminal 2  
make run-agent      # Terminal 3
```

## 🏗️ 系统架构

K8S4R 采用云原生架构，包含以下核心组件：

### 组件说明

1. **CRD (Custom Resource Definition)** - 定义 Robot 资源类型
2. **Manager (Controller)** - 监控 Robot 资源，检查心跳超时  
3. **MQTT Broker** - 消息中间件，处理所有通信
4. **API Server** - 连接 MQTT Broker，处理业务逻辑
5. **Agent** - 运行在机器人设备上，负责注册和心跳

### 通信流程

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

## 🔧 API 参考

### Robot 资源定义

```yaml
apiVersion: robot.k8s4r.io/v1alpha1
kind: Robot
metadata:
  name: robot-001
  namespace: default
spec:
  robotId: "robot-001"        # 必需，机器人唯一标识
  description: "测试机器人"    # 可选，描述信息
status:
  phase: "Online"             # Pending | Online | Offline | Unknown
  lastHeartbeatTime: "2023-..." # 最后心跳时间
  message: "Robot is online"  # 状态消息
  deviceInfo:                 # 设备信息
    hostname: "robot-001"
    os: "linux"
    arch: "amd64"
    cpu: { cores: 4, usage: 25.5 }
    memory: { total: 8192, used: 4096 }
    # ... 更多设备信息
```

### MQTT 消息格式

#### 注册消息 (k8s4r/register)
```json
{
  "robotId": "robot-001",
  "token": "fixed-token-123",
  "deviceInfo": {
    "hostname": "robot-001",
    "os": "linux",
    "arch": "amd64"
  }
}
```

#### 心跳消息 (k8s4r/heartbeat)
```json
{
  "robotId": "robot-001", 
  "token": "fixed-token-123",
  "deviceInfo": { ... }
}
```

## 🐛 故障排除

### 常见问题

| 问题 | 原因 | 解决方案 |
|------|------|----------|
| Agent 无法连接 | MQTT Broker 未启动 | `./config/mosquitto/start-mosquitto.sh status` |
| Robot 状态为 Pending | Agent 未注册成功 | 检查 token 和网络连接 |
| Robot 变为 Offline | 心跳超时 | 重启 Agent 或检查网络 |
| 连接被重置 | Mosquitto 配置问题 | 使用 `simple` 或 `minimal` 模式 |

### 调试命令

```bash
# 查看组件状态
kubectl get robots -w
kubectl describe robot robot-001

# 查看日志
kubectl logs -n k8s4r-system -l app=k8s4r-manager -f
kubectl logs -n k8s4r-system -l app=k8s4r-server -f

# 监控 MQTT 消息
mosquitto_sub -h localhost -p 1883 -t "k8s4r/#" -v
./config/mosquitto/test-mqtt.sh listen

# 手动发送测试消息
./config/mosquitto/test-mqtt.sh register robot-test
./config/mosquitto/test-mqtt.sh heartbeat robot-test
```

## 📁 文档目录

- **[config/mosquitto/README.md](config/mosquitto/README.md)** - MQTT Broker 配置指南
- **[config/mosquitto/ARCHITECTURE.md](config/mosquitto/ARCHITECTURE.md)** - MQTT 架构详细说明
- **[config/mosquitto/MODE_COMPARISON.md](config/mosquitto/MODE_COMPARISON.md)** - MQTT 模式对比
- **[ARCHITECTURE.md](ARCHITECTURE.md)** - 整体系统架构
- **[DEBUG.md](DEBUG.md)** - 调试指南

## 📂 项目结构

```
k8s4r/
├── api/v1alpha1/                    # Robot CRD 类型定义
│   ├── robot_types.go               # Robot Spec/Status 定义
│   └── groupversion_info.go         # API Group 注册
├── cmd/                             # 主程序入口
│   ├── manager/main.go              # Controller Manager
│   ├── server/main.go               # MQTT API Server
│   └── agent/main.go                # Robot Agent
├── pkg/                             # 核心业务逻辑
│   ├── controller/robot_controller.go  # Robot Controller 实现
│   ├── server/server.go             # MQTT Server 实现
│   └── collector/device.go          # 设备信息采集
├── config/                          # 配置文件
│   ├── crd/                         # CRD YAML 文件
│   ├── manager/                     # Manager 部署配置
│   ├── server/                      # Server 部署配置
│   └── mosquitto/                   # MQTT Broker 配置
│       ├── README.md                # MQTT 配置说明
│       ├── ARCHITECTURE.md          # MQTT 架构文档
│       ├── MODE_COMPARISON.md       # 模式对比
│       ├── start-mosquitto.sh       # Broker 启动脚本
│       ├── test-mqtt.sh             # MQTT 测试工具
│       ├── mosquitto.conf           # 开发环境配置
│       ├── mosquitto-prod.conf      # 生产环境配置
│       └── mosquitto-simple.conf    # 简化配置
├── examples/                        # 示例文件
├── bin/                            # 构建产物
└── Dockerfile.*                    # Docker 构建文件
```

## 🔑 核心组件说明

| 组件 | 文件 | 功能 |
|------|------|------|
| **CRD 定义** | `api/v1alpha1/robot_types.go` | 定义 Robot 资源的数据结构 |
| **Controller** | `pkg/controller/robot_controller.go` | 监控 Robot 资源，检查心跳超时 |
| **MQTT Server** | `pkg/server/server.go` | 连接 MQTT Broker，处理消息 |
| **Agent 客户端** | `cmd/agent/main.go` | 机器人设备上的 MQTT 客户端 |
| **设备采集器** | `pkg/collector/device.go` | 采集系统信息（CPU、内存等）|
| **MQTT 工具** | `config/mosquitto/` | Broker 配置和管理脚本 |

## 🎯 技术栈

- **语言**: Go 1.21+
- **框架**: 
  - `controller-runtime` - Kubernetes Controller 框架
  - `client-go` - Kubernetes 客户端库
  - `paho.mqtt.golang` - MQTT 客户端库
- **协议**: MQTT 3.1.1 (Eclipse Mosquitto 2.0)
- **部署**: Docker + Kubernetes

## 📈 后续规划

### 已完成 ✅
- [x] Robot CRD 定义和 Controller
- [x] MQTT 协议通信
- [x] Agent 注册和心跳机制
- [x] 设备信息自动采集
- [x] 完整的开发工具链

### 开发中 🚧
- [ ] 任务下发和执行反馈
- [ ] 二进制文件分发部署
- [ ] Web 管理界面

### 规划中 📋
- [ ] 用户认证和权限控制
- [ ] 监控指标和告警
- [ ] 集群高可用部署
- [ ] SSL/TLS 加密通信
- [ ] 多租户支持

## 🤝 贡献指南

1. Fork 本项目
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 打开 Pull Request

## 📄 License

本项目采用 MIT 许可证 - 详见 [LICENSE](LICENSE) 文件
