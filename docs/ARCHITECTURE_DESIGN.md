# K8s4r 架构设计

## 组件职责划分

### 1. Server（MQTT 桥接层）- 纯消息转发

**职责：MQTT ↔ Kubernetes 消息双向转发，不包含业务逻辑**

#### 监听 MQTT 消息 → 写入 K8s
- `k8s4r/register` → 创建 Robot 资源（初始状态：pending）
- `k8s4r/heartbeat` → 更新 Robot.Status.lastHeartbeatTime
- `robots/{robotName}/tasks/{taskUID}/status` → 更新 Task.Status

#### 监听 K8s 资源变化 → 发送 MQTT
- Task.Status.state = "dispatching" → 发送 MQTT 到 `robots/{targetRobot}/tasks/dispatch`
- Task.DeletionTimestamp != nil → 发送删除消息到 MQTT

**不负责：**
- ❌ 不决定 Task 分配给哪个 Robot
- ❌ 不处理 Robot 离线检测
- ❌ 不包含调度逻辑

---

### 2. Manager/Controller（业务逻辑层）

**职责：所有调度、管理、监控逻辑**

#### TaskController
- 监听 Task 创建事件
- **选择目标 Robot**（根据 constraints、resources 等）
- 设置 `Task.Spec.TargetRobot`
- 更新 `Task.Status.State = "dispatching"` 触发 Server 转发
- 监控 Task 执行超时
- 处理 Task 重启策略

#### RobotController
- 监听 Robot 注册（status.phase = pending）
- 验证、批准 Robot → 更新 status.phase = "Online"
- 监控心跳超时 → 标记 Robot 为 "Offline"
- 管理 Robot 生命周期

**不负责：**
- ❌ 不直接与 MQTT 通信
- ❌ 不处理 MQTT 消息格式转换

---

### 3. Agent（机器人端）

**职责：任务执行**

- MQTT 注册到 Server
- 定期发送心跳
- 接收任务并执行
- 上报任务状态

---

## 完整数据流

### Flow 1: Agent 注册

```
1. Agent → MQTT(k8s4r/register) → Server
2. Server → K8s: Create Robot (status.phase=pending)
3. Manager(RobotController) → 看到新 Robot
4. Manager → 验证 token、检查配额等
5. Manager → K8s: Update Robot.Status.phase=Online
6. Server → 监听到 Robot 变化 → MQTT(k8s4r/response/{robotId})
7. Agent ← MQTT ← 注册成功确认
```

### Flow 2: Task 分发和执行

```
1. User → kubectl create task my-task
2. K8s → 创建 Task（status.state=""）
3. Manager(TaskController) → 监听到新 Task
4. Manager → 选择目标 Robot（业务逻辑）
   - 检查 constraints
   - 检查 Robot 资源
   - 选择最佳 Robot
5. Manager → K8s: Update Task
   - spec.targetRobot = "robot-001"
   - status.state = "dispatching"
   - status.message = "Dispatching to robot-001"
6. Server → 监听到 Task.Status.state=dispatching
7. Server → MQTT(robots/robot-001/tasks/dispatch) ← 转发任务
8. Agent ← MQTT ← 收到任务
9. Agent → 执行任务
10. Agent → MQTT(robots/robot-001/tasks/{uid}/status) ← 上报 state=running
11. Server ← MQTT ← 收到状态
12. Server → K8s: Update Task.Status.state=running
13. Manager → 监控 Task 执行（超时检测等）
14. Agent → 任务完成 → MQTT 上报 state=completed
15. Server → K8s: Update Task.Status.state=completed
```

### Flow 3: Robot 离线检测

```
1. Manager(RobotController) → 定期检查所有 Robot
2. Manager → 发现 robot.status.lastHeartbeatTime 超过 60s
3. Manager → K8s: Update Robot.Status.phase=Offline
4. Manager(TaskController) → 监听到 Robot 离线
5. Manager → 重新调度该 Robot 上的运行中 Task
```

---

## 优势

### 1. 职责清晰
- Server：专注于协议转换（MQTT ↔ K8s）
- Manager：专注于业务逻辑
- 易于维护和测试

### 2. 扩展性强
- 添加新的调度策略：只修改 Manager
- 支持新的协议：只修改 Server
- 业务逻辑和通信解耦

### 3. 符合 Kubernetes 设计模式
- Controller 通过修改资源状态来触发操作
- Server 作为 Operator 响应资源变化
- 声明式 API

---

## 代码结构

```
pkg/
  server/
    server.go           # MQTT 连接管理
    mqtt_to_k8s.go      # MQTT → K8s 转发
    k8s_to_mqtt.go      # K8s → MQTT 转发
  
  controller/
    robot_controller.go # Robot 业务逻辑
    task_controller.go  # Task 调度逻辑

cmd/
  server/main.go        # Server 入口
  manager/main.go       # Manager 入口
  agent/main.go         # Agent 入口
```

---

## 关键设计点

### 1. Task 状态流转

```
"" (创建) 
  → Manager 分配 Robot 
  → "dispatching" (触发 Server 转发)
  → "running" (Agent 执行中)
  → "completed"/"failed" (执行结束)
```

### 2. Server 转发触发条件

Server 只在以下情况转发消息到 MQTT：
- Task.Status.State 变为 "dispatching"
- Task 被删除（DeletionTimestamp != nil）

### 3. Manager 调度策略

Manager 可以实现复杂的调度逻辑：
- Constraint 匹配
- 资源可用性检查
- 负载均衡
- 亲和性/反亲和性
- 优先级队列

这些逻辑都在 Manager 中，Server 无需关心。
