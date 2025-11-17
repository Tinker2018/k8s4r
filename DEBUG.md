# 调试指南

## 代码文件说明

### 核心文件（必读）

| 文件 | 说明 | 关键代码 |
|------|------|---------|
| `api/v1alpha1/robot_types.go` | Robot CRD 类型定义 | RobotSpec (robotId), RobotStatus (phase, lastHeartbeatTime) |
| `pkg/controller/robot_controller.go` | Controller 逻辑 | Reconcile() - 检查心跳超时，每30秒运行一次 |
| `pkg/server/server.go` | HTTP API Server | RegisterHandler() 和 HeartbeatHandler() |
| `cmd/manager/main.go` | Manager 入口 | 启动 Controller Manager |
| `cmd/server/main.go` | Server 入口 | 启动 HTTP Server (默认 :8080) |
| `cmd/agent/main.go` | Agent 入口 | 注册 + 每30秒发送心跳 |

### 配置文件

| 文件 | 说明 |
|------|------|
| `config/crd/robot_crd.yaml` | Robot CRD 定义，需要先安装到 K8s |
| `examples/robot.yaml` | Robot 资源示例 |

## 本地调试步骤

### 前提条件

确保有可用的 Kubernetes 集群（minikube/kind/远程集群）：

```bash
# 检查 K8s 连接
kubectl cluster-info

# 检查当前 namespace
kubectl config view --minify | grep namespace
```

### 第 1 步：安装 CRD

```bash
kubectl apply -f config/crd/robot_crd.yaml

# 验证 CRD 已安装
kubectl get crd robots.robot.k8s4r.io
```

### 第 2 步：运行 Manager（Terminal 1）

```bash
go run cmd/manager/main.go
```

**作用**：监控 K8s 中的 Robot 资源，每30秒检查心跳超时

**预期输出**：
```
INFO    setup   starting manager
INFO    controller-runtime.metrics      Metrics server is starting to listen
```

### 第 3 步：运行 Server（Terminal 2）

```bash
go run cmd/server/main.go --addr=:8080 --namespace=default
```

**作用**：提供 HTTP API，接收 Agent 的注册和心跳请求

**预期输出**：
```
INFO    setup   starting API server     {"address": ":8080", "namespace": "default"}
INFO    server  Server listening        {"addr": ":8080"}
```

### 第 4 步：运行 Agent（Terminal 3）

```bash
go run cmd/agent/main.go \
  --server-url=http://localhost:8080 \
  --token=fixed-token-123 \
  --robot-id=robot-001
```

**作用**：模拟机器人设备，向 Server 注册并发送心跳

**预期输出**：
```
Starting agent for robot: robot-001
Server URL: http://localhost:8080
Successfully registered robot: robot-001
Heartbeat sent successfully for robot: robot-001
Heartbeat sent successfully for robot: robot-001
...
```

### 第 5 步：查看 Robot 资源（Terminal 4）

```bash
# 查看所有 Robot
kubectl get robots

# 输出示例：
# NAME        ROBOTID      PHASE    LASTHEARTBEAT   AGE
# robot-001   robot-001    Online   5s              2m

# 查看详细信息
kubectl get robot robot-001 -o yaml

# 实时监控
kubectl get robots -w
```

## 验证测试场景

### 场景 1：正常注册和心跳

1. 按上述步骤启动 Manager、Server、Agent
2. Agent 注册后，Robot 状态应为 `Online`
3. 每30秒看到 Agent 发送心跳日志
4. `kubectl get robots` 看到 LASTHEARTBEAT 不断更新

### 场景 2：心跳超时检测

1. 停止 Agent (Ctrl+C)
2. 等待 5 分钟
3. `kubectl get robots` 看到 Robot 状态变为 `Offline`
4. Manager 日志输出：`Robot marked as offline due to heartbeat timeout`

### 场景 3：恢复在线

1. 重新启动 Agent
2. Robot 状态从 `Offline` 恢复为 `Online`
3. Manager 日志输出：`Robot recovered to online`

### 场景 4：Token 验证

1. 使用错误的 Token 启动 Agent：
   ```bash
   go run cmd/agent/main.go \
     --server-url=http://localhost:8080 \
     --token=wrong-token \
     --robot-id=robot-002
   ```
2. Agent 注册失败，输出：`registration failed: Invalid token`

## 常见问题

### 1. Manager 无法连接 K8s 集群

**错误**：`unable to get kubeconfig`

**解决**：
```bash
# 检查 kubeconfig
export KUBECONFIG=~/.kube/config
kubectl get nodes
```

### 2. CRD 未安装

**错误**：`no matches for kind "Robot" in version "robot.k8s4r.io/v1alpha1"`

**解决**：
```bash
kubectl apply -f config/crd/robot_crd.yaml
```

### 3. Server 端口被占用

**错误**：`bind: address already in use`

**解决**：更换端口
```bash
go run cmd/server/main.go --addr=:8081
# Agent 也要相应修改 URL
go run cmd/agent/main.go --server-url=http://localhost:8081 ...
```

### 4. Agent 无法连接 Server

**错误**：`failed to send register request: dial tcp`

**解决**：
- 确保 Server 已启动
- 检查 `--server-url` 是否正确
- 检查防火墙设置

## 关键常量

| 常量 | 值 | 位置 |
|------|-----|------|
| Fixed Token | `fixed-token-123` | `pkg/server/server.go:29` |
| 心跳间隔 | 30秒 | `cmd/agent/main.go:171` |
| 心跳超时 | 5分钟 | `pkg/controller/robot_controller.go:33` |
| Controller 检查周期 | 30秒 | `pkg/controller/robot_controller.go:94` |

## 下一步

系统运行正常后，可以尝试：

1. 同时运行多个 Agent（不同 robot-id）
2. 在生产 K8s 集群中部署（参考 `config/` 目录下的部署文件）
3. 添加设备信息采集功能（扩展 RobotStatus）
4. 实现 binary 分发功能
