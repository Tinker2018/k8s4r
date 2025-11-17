# K8S4R 使用指南

## 系统架构

K8S4R (Kubernetes for Robots) 是一个基于 Kubernetes 的机器人设备管理系统，包含以下组件：

1. **CRD (Custom Resource Definition)**: 定义 Robot 资源类型
2. **Manager (Controller)**: 监控 Robot 资源，检查心跳超时
3. **API Server**: 接收来自 Agent 的注册和心跳请求
4. **Agent**: 运行在机器人设备上，负责注册和上报心跳

## 快速开始

### 前置条件

- Kubernetes 集群 (v1.24+)
- kubectl 命令行工具
- Go 1.21+ (仅用于开发)

### 1. 安装 CRD

```bash
kubectl apply -f config/crd/robot_crd.yaml
```

验证 CRD 安装：

```bash
kubectl get crd robots.robot.k8s4r.io
```

### 2. 部署系统组件

#### 部署 Manager (Controller)

```bash
kubectl apply -f config/manager/deployment.yaml
```

检查 Manager 状态：

```bash
kubectl get pods -n k8s4r-system -l app=k8s4r-manager
kubectl logs -n k8s4r-system -l app=k8s4r-manager
```

#### 部署 API Server

```bash
kubectl apply -f config/server/deployment.yaml
```

检查 Server 状态：

```bash
kubectl get pods -n k8s4r-system -l app=k8s4r-server
kubectl get svc -n k8s4r-system k8s4r-server
```

### 3. 创建 Robot 资源

创建一个 Robot 资源：

```bash
kubectl apply -f examples/robot.yaml
```

查看 Robot 资源：

```bash
kubectl get robots
kubectl get rb  # 使用短名称
kubectl describe robot robot-001
```

### 4. 运行 Agent

#### 本地开发模式

首先构建 Agent：

```bash
make build
```

使用端口转发访问 API Server：

```bash
kubectl port-forward -n k8s4r-system svc/k8s4r-server 8080:8080
```

在另一个终端运行 Agent：

```bash
./bin/agent \
  --server-url=http://localhost:8080 \
  --token=fixed-token-123 \
  --robot-id=robot-001 \
  --heartbeat-interval=30
```

#### 在机器人设备上运行

1. 将编译好的 Agent 二进制文件复制到机器人设备
2. 确保机器人可以访问 Kubernetes 集群内的 API Server
3. 运行 Agent：

```bash
./agent \
  --server-url=http://k8s4r-server.k8s4r-system.svc.cluster.local:8080 \
  --token=fixed-token-123 \
  --robot-id=robot-001
```

## 开发指南

### 构建所有组件

```bash
make build
```

这会在 `bin/` 目录下生成三个可执行文件：
- `manager`: Controller Manager
- `server`: API Server
- `agent`: Agent 程序

### 本地运行 (开发模式)

#### 运行 Manager

需要配置好 kubeconfig：

```bash
make run-manager
```

或直接运行：

```bash
go run ./cmd/manager/main.go
```

#### 运行 Server

```bash
make run-server
```

或直接运行：

```bash
go run ./cmd/server/main.go --addr=:8080 --namespace=default
```

#### 运行 Agent

```bash
make run-agent
```

或直接运行：

```bash
go run ./cmd/agent/main.go \
  --server-url=http://localhost:8080 \
  --token=fixed-token-123 \
  --robot-id=robot-001
```

### 构建 Docker 镜像

```bash
make docker-build
```

或分别构建：

```bash
docker build -t k8s4r-manager:latest -f Dockerfile.manager .
docker build -t k8s4r-server:latest -f Dockerfile.server .
docker build -t k8s4r-agent:latest -f Dockerfile.agent .
```

## API 说明

### Agent API

#### 注册 API

**Endpoint**: `POST /api/v1/register`

**Request Body**:
```json
{
  "robotId": "robot-001",
  "token": "fixed-token-123"
}
```

**Response**:
```json
{
  "success": true,
  "message": "Robot registered successfully"
}
```

#### 心跳 API

**Endpoint**: `POST /api/v1/heartbeat`

**Request Body**:
```json
{
  "robotId": "robot-001",
  "token": "fixed-token-123"
}
```

**Response**:
```json
{
  "success": true,
  "message": "Heartbeat accepted"
}
```

## Robot 资源说明

### Robot Spec

```yaml
spec:
  robotId: string        # 必需，机器人唯一标识
  description: string    # 可选，机器人描述
  labels:                # 可选，标签
    key: value
```

### Robot Status

```yaml
status:
  phase: string               # Pending | Online | Offline | Unknown
  lastHeartbeatTime: time     # 最后心跳时间
  message: string             # 状态消息
```

### Robot Phase 说明

- **Pending**: Robot 资源刚创建，等待 Agent 注册
- **Online**: Agent 已注册且正常发送心跳
- **Offline**: 心跳超时（5分钟未收到心跳）
- **Unknown**: 未知状态

## 监控和故障排查

### 查看 Robot 列表

```bash
# 查看所有 Robot
kubectl get robots

# 查看特定 namespace 的 Robot
kubectl get robots -n default

# 查看详细信息
kubectl get robots -o wide
```

### 查看 Robot 详情

```bash
kubectl describe robot robot-001
```

### 查看日志

```bash
# Manager 日志
kubectl logs -n k8s4r-system -l app=k8s4r-manager -f

# Server 日志
kubectl logs -n k8s4r-system -l app=k8s4r-server -f
```

### 常见问题

#### Agent 无法连接到 Server

1. 检查 Server 是否运行：`kubectl get pods -n k8s4r-system`
2. 检查网络连接
3. 确认 URL 和端口正确

#### Robot 状态一直是 Pending

1. 检查 Agent 是否正常运行
2. 检查 Agent 日志是否有错误
3. 确认 Token 是否正确

#### Robot 变成 Offline

1. 检查 Agent 是否仍在运行
2. 查看最后心跳时间：`kubectl get robot <name> -o yaml`
3. 重启 Agent 恢复在线状态

## 配置说明

### Manager 配置

```bash
./manager \
  --metrics-bind-address=:8080 \
  --health-probe-bind-address=:8081 \
  --leader-elect
```

### Server 配置

```bash
./server \
  --addr=:8080 \
  --namespace=default
```

### Agent 配置

```bash
./agent \
  --server-url=http://localhost:8080 \
  --token=fixed-token-123 \
  --robot-id=robot-001 \
  --heartbeat-interval=30
```

## 卸载

```bash
# 删除 Robot 资源
kubectl delete robots --all

# 删除系统组件
kubectl delete -f config/server/deployment.yaml
kubectl delete -f config/manager/deployment.yaml

# 删除 CRD
kubectl delete -f config/crd/robot_crd.yaml

# 删除 namespace
kubectl delete namespace k8s4r-system
```

## 下一步计划

- [ ] 添加设备信息采集功能（IP、Hostname、硬件信息等）
- [ ] 支持二进制文件分发和部署
- [ ] 添加 WebSocket 支持实时通信
- [ ] 实现任务下发和执行
- [ ] 添加监控指标和告警
- [ ] 支持多租户和权限控制
