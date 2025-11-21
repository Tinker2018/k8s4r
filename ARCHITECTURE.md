# K8S4R 架构说明

## 系统概览

K8S4R (Kubernetes for Robots) 是一个基于 Kubernetes 的机器人任务管理系统，采用声明式范式管理机器人资源和任务调度。

### 核心 CRD 资源

1. **Robot** - 机器人设备资源，包含设备信息和状态
2. **Job** - 任务定义，包含 TaskGroup 和 RobotSelector
3. **TaskGroup** - 任务组，包含 count 字段用于并发执行
4. **Task** - 任务执行实例（类比 Kubernetes Pod）

### 核心组件

1. **Manager** - Kubernetes Controller + gRPC Server
   - 包含 RobotController、JobController、TaskGroupController、TaskController
   - 暴露 gRPC 服务（端口 9090）接收 Server 上报的状态
   - 通过 gRPC 双向流推送任务到 Server

2. **Server** - gRPC Client + MQTT Bridge（**无 K8s 依赖**）
   - 连接 Manager 的 gRPC 服务
   - 连接 MQTT Broker
   - 双向转发：MQTT ↔ gRPC ↔ Manager ↔ Kubernetes

3. **Agent** - 运行在机器人设备上的 MQTT 客户端，执行任务

## 系统架构

### 整体架构图

```
┌──────────────────────────────────────────────────────────────────────────┐
│                          Kubernetes Cluster                              │
│                                                                          │
│  ┌───────────────────┐   ┌───────────────┐   ┌──────────────────────┐  │
│  │   Robot CRD       │   │   Job CRD     │   │   Task CRD           │  │
│  │                   │   │               │   │                      │  │
│  │  spec:            │   │  spec:        │   │  spec:               │  │
│  │    robotId        │   │   robotSelector   │    targetRobot       │  │
│  │    labels ───────────►│   taskGroups  │   │    driver            │  │
│  │                   │   │    - count    │   │    config            │  │
│  │  status:          │   │    - tasks    │   │    jobName           │  │
│  │    phase          │   │               │   │                      │  │
│  │    lastHeartbeat  │   │  status:      │   │  status:             │  │
│  └────────┬──────────┘   │   taskGroups  │   │    state (pending)   │  │
│           │              └────────┬──────┘   └───────┬──────────────┘  │
│           │                       │                   │                 │
│  ┌────────▼──────────────────────▼───────────────────▼───────────────┐ │
│  │              Manager (Controllers + gRPC Server)                  │ │
│  │                         Port: 9090                                │ │
│  │  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐   │ │
│  │  │Robot         │  │Job           │  │TaskGroup             │   │ │
│  │  │Controller    │  │Controller    │  │Controller            │   │ │
│  │  │              │  │              │  │                      │   │ │
│  │  │- 注册审批     │  │- 创建        │  │- 创建 count 个       │   │ │
│  │  │- 心跳检测     │  │  TaskGroup   │  │  Task 实例           │   │ │
│  │  │- 状态更新     │  │              │  │- 设置 JobName        │   │ │
│  │  └──────────────┘  └──────────────┘  └──────────────────────┘   │ │
│  │                                                                   │ │
│  │  ┌──────────────────────────────────────────────────────┐       │ │
│  │  │            Task Controller                            │       │ │
│  │  │                                                        │       │ │
│  │  │  - 调度 Task（选择 Robot）                             │       │ │
│  │  │  - 调用 TaskStreamManager.PushTaskToStream()          │       │ │
│  │  │  - 监控状态变化                                         │       │ │
│  │  └──────────────────────────────────────────────────────┘       │ │
│  │                                                                   │ │
│  │  ┌──────────────────────────────────────────────────────────┐   │ │
│  │  │             gRPC Service (RobotManager)                  │   │ │
│  │  │                                                           │   │ │
│  │  │  Unary RPCs (Server → Manager):                         │   │ │
│  │  │    - ReportRobotRegistration                            │   │ │
│  │  │    - ReportRobotHeartbeat                               │   │ │
│  │  │    - ReportTaskStatus                                   │   │ │
│  │  │                                                           │   │ │
│  │  │  Bidirectional Stream (Manager ↔ Server):               │   │ │
│  │  │    - StreamTasks                                         │   │ │
│  │  │      Manager → Server: TaskCommand (CREATE/DELETE)      │   │ │
│  │  │      Server → Manager: TaskEvent (ACK/PUBLISHED/ERROR)  │   │ │
│  │  └──────────────────────────────────────────────────────────┘   │ │
│  └────────────────────────────┬─────────────────────────────────────┘ │
└────────────────────────────────┼───────────────────────────────────────┘
                                 │
                                 │ gRPC Bidirectional Stream
                                 │ (localhost:9090)
                                 │
                  ┌──────────────▼──────────────┐
                  │    Server (gRPC + MQTT)     │
                  │    **无 Kubernetes 依赖**   │
                  │                             │
                  │  Components:                │
                  │    - gRPC Client            │
                  │    - MQTT Client/Publisher  │
                  │    - 纯转发层（无 K8s）     │
                  │                             │
                  │  Functions:                 │
                  │    - stream.Recv() 接收任务 │
                  │    - 转发到 MQTT            │
                  │    - stream.Send() 确认     │
                  └──────────────┬──────────────┘
                                 │
                                 │ MQTT Protocol
                                 │ (tcp://localhost:1883)
                                 │
                  ┌──────────────▼──────────────┐
                  │     MQTT Broker             │
                  │     (Mosquitto)             │
                  │                             │
                  │  Topics:                    │
                  │   - k8s4r/register          │
                  │   - k8s4r/heartbeat         │
                  │   - k8s4r/robots/+/tasks/*  │
                  └──────────────┬──────────────┘
                                 │
        ┌────────────────────────┼────────────────────┐
        │                        │                    │
   ┌────▼────┐             ┌─────▼────┐         ┌────▼────┐
   │ Agent   │             │ Agent    │         │ Agent   │
   │robot-001│             │robot-002 │         │robot-003│
   │         │             │          │         │         │
   │- 注册   │             │- 注册    │         │- 注册   │
   │- 心跳   │             │- 心跳    │         │- 心跳   │
   │- 执行   │             │- 执行    │         │- 执行   │
   └─────────┘             └──────────┘         └─────────┘
```

### gRPC 通信架构

#### 设计原则

**完全解耦 Server 与 Kubernetes**：
- ✅ Server 不直接访问 Kubernetes API
- ✅ Server 只依赖 gRPC Client 和 MQTT Client  
- ✅ 所有 K8s 操作由 Manager 独占处理
- ✅ Server 可以独立部署，不需要 kubeconfig

#### gRPC Service 定义

```protobuf
service RobotManager {
  // ========== Unary RPCs (Server → Manager 上报) ==========
  rpc ReportRobotRegistration(RegistrationRequest) returns (RegistrationResponse);
  rpc ReportRobotHeartbeat(HeartbeatRequest) returns (HeartbeatResponse);
  rpc ReportTaskStatus(TaskStatusRequest) returns (TaskStatusResponse);
  
  // ========== Bidirectional Stream (Manager ↔ Server 任务分发) ==========
  rpc StreamTasks(stream TaskEvent) returns (stream TaskCommand);
}
```

**双向流消息类型**：

```protobuf
// TaskCommand: Manager → Server 推送任务
message TaskCommand {
  enum CommandType {
    CREATE_TASK = 0;   // 创建任务
    DELETE_TASK = 1;   // 删除任务  
    KEEPALIVE = 2;     // 保持连接
  }
  CommandType type = 1;
  Task task = 2;
}

// TaskEvent: Server → Manager 确认
message TaskEvent {
  enum EventType {
    ACK = 0;           // 确认收到
    PUBLISHED = 1;     // 已发布到 MQTT
    ERROR = 2;         // 处理失败
    KEEPALIVE = 3;     // 保持连接
  }
  EventType type = 1;
  string task_uid = 2;
  string message = 3;
}
```

#### 双向流工作流程

**Manager 推送任务**：
```go
// pkg/manager/task_stream.go
stream.Send(&TaskCommand{
    Type: CREATE_TASK,
    Task: &Task{Uid: "task-123", TargetRobot: "robot-001", ...},
})
```

**Server 接收并转发**：
```go
// pkg/server/grpc_stream_server.go
taskCmd, _ := stream.Recv()  // 阻塞接收
if taskCmd.Type == CREATE_TASK {
    stream.Send(&TaskEvent{Type: ACK, TaskUid: taskCmd.Task.Uid})
    mqttClient.Publish("k8s4r/robots/robot-001/tasks/dispatch", taskPayload)
    stream.Send(&TaskEvent{Type: PUBLISHED, TaskUid: taskCmd.Task.Uid})
}
```

## 数据流

### 1. 任务创建与分发流程（gRPC Stream）

```
User              Manager            Server            Agent
 │                   │                   │                │
 │ 1. kubectl create │                   │                │
 │    Job.yaml       │                   │                │
 ├──────────────────>│                   │                │
 │                   │                   │                │
 │              2. JobController         │                │
 │                   │  创建 TaskGroup    │                │
 │                   │                   │                │
 │              3. TaskGroupController   │                │
 │                   │  创建 count 个 Task│                │
 │                   │  设置 JobName      │                │
 │                   │                   │                │
 │              4. TaskController        │                │
 │                   │  调度 Task         │                │
 │                   │  选择 Robot        │                │
 │                   │                   │                │
 │              5. PushTaskToStream()    │                │
 │                   ├─stream.Send()────>│                │
 │                   │  TaskCommand      │                │
 │                   │  {CREATE_TASK}    │                │
 │                   │                   │                │
 │                   │<─stream.Send()────┤                │
 │                   │  TaskEvent{ACK}   │                │
 │                   │                   │                │
 │                   │              6. MQTT Publish       │
 │                   │                   ├─dispatch──────>│
 │                   │                   │                │
 │                   │<─stream.Send()────┤                │
 │                   │ TaskEvent{PUBLISHED}              │
 │                   │                   │                │
 │                   │                   │<─status────────┤
 │                   │              7. ReportTaskStatus() │
 │                   │<───Unary RPC──────┤                │
 │                   │                   │                │
 │              8. Update Task.Status    │                │
 │                   │  state=running    │                │
```

### 2. Agent 注册流程（MQTT → gRPC）

```
Agent              Server             Manager           Kubernetes
  │                   │                   │                  │
  │─MQTT Publish─────>│                   │                  │
  │  topic:register   │                   │                  │
  │  {robotId,token}  │                   │                  │
  │                   │                   │                  │
  │              Unary RPC                │                  │
  │                   ├─ReportRegistration>│                  │
  │                   │                   │                  │
  │                   │                   │─Get Robot───────>│
  │                   │                   │                  │
  │                   │                   │<─Not Found───────│
  │                   │                   │                  │
  │                   │                   │─Create Robot────>│
  │                   │                   │  spec.robotId=xxx│
  │                   │                   │  phase=Online    │
  │                   │                   │                  │
  │                   │<──Success─────────│                  │
  │<─MQTT Response────│                   │                  │
```

### 3. Agent 心跳流程（MQTT → gRPC）

```
Agent              Server             Manager           Kubernetes
  │                   │                   │                  │
  │─MQTT Publish─────>│                   │                  │
  │  topic:heartbeat  │                   │                  │
  │  {robotId,token}  │                   │                  │
  │                   │                   │                  │
  │              Unary RPC                │                  │
  │                   ├─ReportHeartbeat──>│                  │
  │                   │                   │                  │
  │                   │                   │─Get Robot───────>│
  │                   │                   │                  │
  │                   │                   │<─Robot object────│
  │                   │                   │                  │
  │                   │                   │─Update Status───>│
  │                   │                   │  lastHeartbeat=now
  │                   │                   │  phase=Online    │
  │                   │                   │                  │
  │                   │<──Success─────────│                  │
  │                   │                   │                  │
  │  (30秒后重复)       │                   │                  │
```

### 4. 任务状态上报流程（MQTT → gRPC）

```
Agent              Server             Manager           Kubernetes
  │                   │                   │                  │
  │  执行任务          │                   │                  │
  │  state=running    │                   │                  │
  │                   │                   │                  │
  │─MQTT Publish─────>│                   │                  │
  │  task/status      │                   │                  │
  │  {taskUid,state}  │                   │                  │
  │                   │                   │                  │
  │              Unary RPC                │                  │
  │                   ├─ReportTaskStatus─>│                  │
  │                   │                   │                  │
  │                   │                   │─Get Task────────>│
  │                   │                   │                  │
  │                   │                   │<─Task object─────│
  │                   │                   │                  │
  │                   │                   │─Update Status───>│
  │                   │                   │  state=running   │
  │                   │                   │  message=xxx     │
  │                   │                   │                  │
  │                   │<──Success─────────│                  │
```

### 5. RobotController 监控离线流程

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

## 关键常量与配置

### Manager 配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--grpc-bind-address` | `:9090` | gRPC 服务监听地址 |
| `--namespace` | `default` | 监听的 K8s namespace |
| `--metrics-bind-address` | `:8082` | Metrics 端点地址 |

### Server 配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--broker-url` | `tcp://localhost:1883` | MQTT Broker 地址 |
| `--grpc-addr` | `localhost:9090` | Manager gRPC 地址 |

### Agent 配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--broker-url` | `tcp://localhost:1883` | MQTT Broker 地址 |
| `--robot-id` | 必填 | 机器人唯一标识 |
| `--token` | `fixed-token-123` | 认证 Token |

### 时间常量

- **心跳间隔**: 30秒（Agent 发送心跳的频率）
- **心跳超时**: 5分钟（RobotController 判定离线的阈值）
- **Controller 检查周期**: 30秒（Controller reconcile 频率）
- **gRPC Keepalive**: 60秒（双向流保活消息）

## CRD 资源状态机

### Robot 状态转换

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

### Task 状态转换

```
   ┌─────────┐
   │ Pending │  ← TaskGroup 创建 Task
   └────┬────┘
        │ TaskController 选择 Robot
        │ PushTaskToStream() 推送到 Server
        ▼
   ┌──────────────┐
   │ Dispatching  │  ← Server 已转发到 MQTT
   └──────┬───────┘
        │ Agent 接收任务并开始执行
        ▼
   ┌────────┐
   │ Running│  ← Agent 上报 state=running
   └────┬───┘
        │ Agent 执行完成
        ▼
   ┌────────┐
   │ Exited │  ← Agent 上报 exitCode
   └────┬───┘
        │ TaskController 检查 exitCode
        ├────────────┬────────────┐
        │            │            │
   exitCode=0   exitCode≠0    超时/错误
        │            │            │
        ▼            ▼            ▼
   ┌──────────┐ ┌─────────┐ ┌─────────┐
   │Completed │ │ Failed  │ │  Dead   │
   └──────────┘ └─────────┘ └─────────┘
```

### Job → TaskGroup → Task 层级

```yaml
apiVersion: robot.k8s4r.io/v1alpha1
kind: Job
metadata:
  name: example-job
spec:
  robotSelector:
    env: production
  taskGroups:
    - name: concurrent-tasks
      count: 2                    # 创建 2 个并发 Task
      template:
        driver: exec
        config:
          command: "sleep 5 && echo hello"
```

**创建流程**：
1. JobController 检测到 Job 创建
2. 为每个 taskGroups 创建 TaskGroup CRD
3. TaskGroupController 根据 `count: 2` 创建 2 个 Task
4. 每个 Task 设置 `spec.jobName` 指向原 Job
5. TaskController 分别调度这 2 个 Task

## 组件职责

| 组件 | 职责 | 依赖 | 运行位置 |
|------|------|------|---------|
| **Manager** | - 运行所有 Controller<br>- 提供 gRPC Server<br>- 管理 K8s 资源 | Kubernetes API<br>gRPC | K8s 集群内/外 |
| **Server** | - gRPC Client 连接 Manager<br>- MQTT Client 连接 Broker<br>- 消息双向转发 | gRPC<br>MQTT<br>**无 K8s** | 任意位置 |
| **Agent** | - MQTT Client<br>- 执行任务<br>- 上报状态 | MQTT | 机器人设备 |
| **MQTT Broker** | - 消息中转<br>- Topic 路由 | 无 | 任意位置 |

## MQTT Topics 设计

| Topic | 方向 | 用途 | QoS | Retained |
|-------|------|------|-----|----------|
| `k8s4r/register` | Agent → Server | Agent 注册请求 | 1 | No |
| `k8s4r/heartbeat` | Agent → Server | Agent 心跳上报 | 1 | No |
| `k8s4r/robots/{id}/tasks/dispatch` | Server → Agent | 任务分发 | 1 | No |
| `k8s4r/robots/{id}/tasks/{uid}/status` | Agent → Server | 任务状态上报 | 1 | No |
| `k8s4r/robots/{id}/response` | Server → Agent | 通用响应 | 1 | No |

## 部署架构

### 单机开发环境

```bash
# 1. 启动 MQTT Broker
docker run -d -p 1883:1883 eclipse-mosquitto:2.0

# 2. 启动 Manager（本地运行，连接 K8s）
bin/manager \
  --grpc-bind-address=:9090 \
  --namespace=default

# 3. 启动 Server（本地运行，无需 K8s）
bin/server \
  --broker-url=tcp://localhost:1883 \
  --grpc-addr=localhost:9090

# 4. 启动 Agent（模拟机器人）
bin/agent \
  --broker-url=tcp://localhost:1883 \
  --robot-id=robot-001
```

### 生产环境

```
┌─────────────────────────────────────┐
│         Kubernetes Cluster          │
│                                     │
│  ┌─────────────────────────────┐   │
│  │  Manager (Deployment)       │   │
│  │  - replicas: 1              │   │
│  │  - port: 9090 (gRPC)        │   │
│  │  - ServiceAccount + RBAC    │   │
│  └─────────────────────────────┘   │
│                                     │
│  ┌─────────────────────────────┐   │
│  │  Server (Deployment)        │   │
│  │  - replicas: 2+ (可扩展)    │   │
│  │  - 无 K8s 依赖              │   │
│  │  - 连接 Manager gRPC        │   │
│  └─────────────────────────────┘   │
└─────────────────────────────────────┘
           │                │
           │ gRPC           │ MQTT
           │                │
    ┌──────▼────────┐  ┌────▼────────┐
    │ Manager SVC   │  │ MQTT Broker │
    │ (ClusterIP)   │  │ (External)  │
    └───────────────┘  └─────────────┘
```

## 扩展点

### 已实现功能

- ✅ Robot 注册与心跳
- ✅ Task 调度与分发
- ✅ gRPC 双向流通信
- ✅ Server 与 K8s 解耦
- ✅ 并发任务支持（TaskGroup.count）
- ✅ 三层架构（Job → TaskGroup → Task）

### 后续可扩展

1. **多租户支持**: 添加 namespace 隔离
2. **认证增强**: 为每个 Robot 生成独立 Token/证书
3. **TLS 加密**: gRPC 和 MQTT 启用 TLS
4. **监控指标**: Prometheus metrics（任务成功率、延迟）
5. **日志采集**: Agent 日志上报到中心化存储
6. **任务重试**: Task 失败自动重试机制
7. **资源配额**: 限制每个 Robot 的并发任务数
8. **优先级调度**: Task 添加 priority 字段
9. **Artifact 管理**: 任务产物上传/下载
10. **Webhook 通知**: Task 状态变化通知外部系统

## 故障排查

### Manager 无法启动

```bash
# 检查 K8s 连接
kubectl cluster-info

# 检查 RBAC 权限
kubectl auth can-i create robots

# 查看日志
kubectl logs -f deployment/k8s4r-manager
```

### Server 无法连接 Manager

```bash
# 检查 gRPC 端口
telnet localhost 9090

# 查看 Server 日志
./bin/server | grep -i "grpc\|error"
```

### Agent 无法连接 MQTT

```bash
# 检查 Broker 状态
mosquitto_sub -h localhost -t '#' -v

# 测试发布消息
mosquitto_pub -h localhost -t test -m "hello"
```

### Task 未分发到 Agent

```bash
# 检查 Task 状态
kubectl get tasks -o wide

# 检查 Manager 日志（Stream）
kubectl logs -f deployment/k8s4r-manager | grep "GRPC STREAM"

# 检查 MQTT 消息
mosquitto_sub -h localhost -t 'k8s4r/robots/+/tasks/dispatch' -v
```

## 相关文档

- [api/v1alpha1/](api/v1alpha1/) - CRD 定义
- [api/grpc/robot_manager.proto](api/grpc/robot_manager.proto) - gRPC Proto 定义  
- [pkg/controller/](pkg/controller/) - Controller 实现
- [pkg/manager/](pkg/manager/) - gRPC Server 实现
- [pkg/server/](pkg/server/) - gRPC Client + MQTT Bridge
- [docs/DEBUG_GUIDE.md](docs/DEBUG_GUIDE.md) - 调试指南
- [docs/NOMAD_EXECUTOR_GUIDE.md](docs/NOMAD_EXECUTOR_GUIDE.md) - Nomad 执行器指南

