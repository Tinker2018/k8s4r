# K8S4R - Kubernetes for Robots

一个基于 Kubernetes 的机器人设备管理系统，提供类似 K3S 的声明式管理范式。

## 核心功能

- ✅ Robot 资源的注册和心跳管理
- ✅ 基于 Token 的简单认证
- ✅ 自动检测离线状态（5分钟心跳超时）
- ✅ 设备信息自动采集和上报（CPU、内存、磁盘、网络、系统信息）

## 快速开始

### 1. 安装 CRD

```bash
kubectl apply -f config/crd/robot.k8s4r.io_robots.yaml
```

### 2. 运行 Manager（Terminal 1）

```bash
go run cmd/manager/main.go
```

### 3. 运行 Server（Terminal 2）

```bash
go run cmd/server/main.go --addr=:8080 --namespace=default
```

### 4. 运行 Agent（Terminal 3）

```bash
go run cmd/agent/main.go \
  --server-url=http://localhost:8080 \
  --token=fixed-token-123 \
  --robot-id=robot-001
```

### 5. 查看 Robot 资源

```bash
# 查看所有 Robot（包含设备信息摘要）
kubectl get robots

# 查看详细信息（包含完整设备信息）
kubectl get robot robot-001 -o yaml

# 使用 describe 查看格式化的设备信息
kubectl describe robot robot-001
```

## 设备信息查看

Agent 会自动采集并上报以下设备信息：

| 信息类型 | 说明 |
|---------|------|
| 主机名 | 设备的 hostname |
| 操作系统 | OS 类型和架构（linux/darwin/windows, amd64/arm64） |
| CPU | 核心数 |
| 内存 | 总容量和已使用量（MB） |
| 磁盘 | 总容量和已使用量（GB） |
| IP 地址 | 所有非回环网卡的 IPv4 地址 |

详细的设备信息采集和使用指南请参考 [DEVICE_INFO.md](DEVICE_INFO.md)

## 文档

- **[DEVICE_INFO.md](DEVICE_INFO.md)** - 设备信息采集功能详细说明
- **[ARCHITECTURE.md](ARCHITECTURE.md)** - 系统架构和设计说明

## 项目结构

```
k8s4r/
├── api/v1alpha1/              # Robot CRD 类型定义
│   ├── robot_types.go         # Robot Spec/Status 定义
│   └── groupversion_info.go   # API Group 注册
├── cmd/
│   ├── manager/main.go        # Controller Manager 入口
│   ├── server/main.go         # HTTP API Server 入口
│   └── agent/main.go          # Agent 入口
├── pkg/
│   ├── controller/            # Robot Controller 实现
│   └── server/                # HTTP Server 实现
├── config/crd/                # CRD YAML 文件
└── examples/                  # Robot 资源示例
```

## 核心代码说明

| 组件 | 文件 | 说明 |
|------|------|------|
| **CRD 定义** | `api/v1alpha1/robot_types.go` | Robot 资源的数据结构 |
| **Controller** | `pkg/controller/robot_controller.go` | 监控 Robot，检查心跳超时 |
| **API Server** | `pkg/server/server.go` | 处理 Agent 注册和心跳请求 |
| **Agent** | `cmd/agent/main.go` | 机器人设备上的客户端 |

## License

MIT
