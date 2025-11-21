# K8s4r 架构设计

## 三层资源架构

K8s4r 采用三层资源模型，提供灵活的任务编排能力：

```
Job (最高层 - 作业)
 └── TaskGroup (中间层 - 任务组, 独立 CRD)
      └── Task (最底层 - 具体任务)
```

### 资源层次说明

#### 1. Job - 作业（最高层）
- **用途**: 定义完整的工作流程
- **特性**: 
  - 包含多个 TaskGroup
  - 定义 Robot 选择器（robotSelector）
  - 管理整体作业生命周期
  - 聚合所有 TaskGroup 的状态

#### 2. TaskGroup - 任务组（中间层，独立 CRD）
- **用途**: 组织相关的任务集合
- **特性**:
  - 属于某个 Job
  - 包含多个 Task 定义
  - 支持并发度控制（count 字段）
  - 独立的 CRD 资源，可单独查询和管理
  - 状态自动聚合子任务状态

#### 3. Task - 任务（最底层）
- **用途**: 实际执行的最小单位
- **特性**:
  - 由 TaskGroup 创建
  - 分配到具体 Robot 执行
  - 包含执行配置（driver、config、artifacts等）
  - **支持独立超时控制** (`timeout` 字段)
  - 上报详细的执行状态

### 状态级联机制

状态信息从底层向上层聚合：

```
Task.Status (running/completed/failed)
  ↓ 聚合
TaskGroup.Status (统计所有 Task 状态)
  ↓ 聚合  
Job.Status (统计所有 TaskGroup 状态)
```

---

## 组件职责划分

### 1. Server（MQTT 桥接层）- 纯消息转发

**职责：MQTT ↔ Kubernetes 消息双向转发，不包含业务逻辑**

#### 监听 MQTT 消息 → 写入 K8s
- `k8s4r/register` → 创建 Robot 资源（初始状态：pending）
- `k8s4r/heartbeat` → 更新 Robot.Status.lastHeartbeatTime
- `k8s4r/robots/{robotId}/tasks/{taskUID}/status` → 更新 Task.Status

#### 监听 K8s 资源变化 → 发送 MQTT
- Task.Status.state = "dispatching" → 发送 MQTT 到 `k8s4r/robots/{targetRobot}/tasks/dispatch`
- Task.DeletionTimestamp != nil → 发送删除消息到 MQTT

**不负责：**
- ❌ 不决定 Task 分配给哪个 Robot
- ❌ 不处理 Robot 离线检测
- ❌ 不包含调度逻辑

---

### 2. Manager/Controller（业务逻辑层）

**职责：所有调度、管理、监控逻辑**

#### JobController
- 监听 Job 创建事件
- 为每个 TaskGroup 定义创建 TaskGroup 资源
- 聚合 TaskGroup 状态到 Job.Status
- 管理 Job 生命周期

#### TaskGroupController  
- 监听 TaskGroup 创建事件
- 根据 count 字段创建多个 Task 实例
- 聚合 Task 状态到 TaskGroup.Status
- 管理 TaskGroup 生命周期

#### TaskController
- 监听 Task 创建事件
- **选择目标 Robot**（根据 Job.robotSelector、resources 等）
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

**职责：任务执行和状态上报**

#### 核心功能
- MQTT 注册到 Server（`k8s4r/register`）
- 定期发送心跳（`k8s4r/heartbeat`，每30秒）
- 接收任务分发（`k8s4r/robots/{robotId}/tasks/dispatch`）
- 执行任务（使用 Nomad Executor）
- 上报任务状态（`k8s4r/robots/{robotId}/tasks/{taskUID}/status`）

#### 性能优化
- **单协程监控**: 所有任务共享一个监控协程
  - 之前: N 个任务 = 2N 个协程（waitTask + monitorTask）
  - 现在: N 个任务 = N+1 个协程（waitTask + 1 个 monitorAllTasks）
  - **减少 45% 协程数量**，降低资源消耗
  
- **超时检测**: 
  - 监控协程每 5 秒检查所有运行中任务
  - 对比运行时间与 `task.Spec.Timeout`
  - 超时后调用 `terminateTask()` 终止进程
  - 上报 "Task timeout" 事件

#### 任务执行流程
```
1. 收到任务 → 创建 runningTask 记录（含 startTime）
2. 启动 waitTask 协程等待任务完成
3. 启动统一监控协程（仅第一次）
4. 监控协程每 5s 检查:
   - 任务是否仍在运行
   - 运行时间是否超时
   - 超时则调用 driver.Stop() + driver.Destroy()
5. 任务完成/失败/超时 → 清理资源 → 上报最终状态
```

---

## 完整数据流

### Flow 1: Agent 注册

```
1. Agent → MQTT(k8s4r/register) → Server
2. Server → K8s: Create Robot (status.phase=pending)
3. Manager(RobotController) → 看到新 Robot
4. Manager → 验证 token、检查配额等
5. Manager → K8s: Update Robot.Status.phase=Online
6. Server → 监听到 Robot 变化 → MQTT(k8s4r/robots/{robotId}/response)
7. Agent ← MQTT ← 注册成功确认
```

### Flow 2: Job/Task 分发和执行（三层架构）

```
1. User → kubectl create job my-job
2. K8s → 创建 Job 资源
3. Manager(JobController) → 监听到新 Job
4. Manager → 为每个 TaskGroup 定义创建 TaskGroup 资源
5. Manager(TaskGroupController) → 监听到新 TaskGroup
6. Manager → 根据 count 创建多个 Task 资源
7. Manager(TaskController) → 监听到新 Task
8. Manager → 选择目标 Robot（基于 Job.robotSelector）
   - 检查 constraints
   - 检查 Robot 资源
   - 选择最佳 Robot
9. Manager → K8s: Update Task
   - spec.targetRobot = "robot-001"
   - status.state = "dispatching"
   - status.message = "Dispatching to robot-001"
10. Server → 监听到 Task.Status.state=dispatching
11. Server → MQTT(k8s4r/robots/robot-001/tasks/dispatch) ← 转发任务
12. Agent ← MQTT ← 收到任务
13. Agent → 启动任务执行（Nomad Executor）
    - 记录 startTime（用于超时检测）
    - 启动 waitTask 协程
    - 启动统一 monitor 协程（如果尚未启动）
14. Agent → MQTT(k8s4r/robots/robot-001/tasks/{uid}/status) ← 上报 state=running
15. Server ← MQTT ← 收到状态
16. Server → K8s: Update Task.Status.state=running
17. Manager(TaskGroupController) → 聚合 Task 状态到 TaskGroup
18. Manager(JobController) → 聚合 TaskGroup 状态到 Job
19. Agent → 监控协程检查超时:
    - 每 5 秒检查: time.Since(startTime) vs task.Spec.Timeout
    - 如超时: terminateTask() → driver.Stop() → driver.Destroy()
    - 上报 state=failed, message="Task timeout"
20. Agent → 任务完成 → MQTT 上报 state=completed
21. Server → K8s: Update Task.Status.state=completed
22. Manager → 更新 TaskGroup 和 Job 状态
```

### Flow 3: Robot 离线检测

```
1. Manager(RobotController) → 定期检查所有 Robot
2. Manager → 发现 robot.status.lastHeartbeatTime 超过 5分钟
3. Manager → K8s: Update Robot.Status.phase=Offline
4. Manager(TaskController) → 监听到 Robot 离线
5. Manager → 重新调度该 Robot 上的运行中 Task
```

### Flow 4: 任务超时处理

```
1. Task 开始执行 → Agent 记录 startTime
2. Monitor 协程每 5s 检查:
   elapsed = time.Since(startTime)
   if elapsed > task.Spec.Timeout:
3. Agent → 调用 terminateTask():
   - logger.Info("task timeout detected")
   - driver.Stop(ctx) → 发送 SIGTERM
   - 等待 killTimeout
   - driver.Destroy(ctx) → 强制 SIGKILL
4. Agent → MQTT 上报:
   state=failed
   message="Task terminated: Task timeout"
   exitCode=143 (SIGTERM) 或 137 (SIGKILL)
5. Server → K8s: Update Task.Status
6. Manager → 根据重试策略决定是否重新调度
```

---

## 优势

### 1. 职责清晰
- Server：专注于协议转换（MQTT ↔ K8s）
- Manager：专注于业务逻辑（三层资源编排）
- Agent：专注于任务执行（高性能、低资源）
- 易于维护和测试

### 2. 扩展性强
- 添加新的调度策略：只修改 Manager
- 支持新的协议：只修改 Server
- 优化任务执行：只修改 Agent
- 业务逻辑和通信解耦

### 3. 符合 Kubernetes 设计模式
- Controller 通过修改资源状态来触发操作
- Server 作为 Operator 响应资源变化
- 声明式 API
- 三层资源提供灵活的编排能力

### 4. 性能优化
- **Agent 协程优化**: N 个任务仅需 N+1 个协程（减少 45%）
- **统一监控**: 单个协程轮询所有任务，避免资源浪费
- **超时控制**: 每个任务独立配置，精确控制执行时间
- **Nomad Executor**: 生产级进程管理，可靠性高

---

## 代码结构

```
api/v1alpha1/                   # CRD 定义
  job_types.go                  # Job CRD（最高层）
  taskgroup_types.go            # TaskGroup CRD（中间层，独立）
  task_types.go                 # Task CRD（最底层，含 Timeout 字段）
  robot_types.go                # Robot CRD

pkg/
  server/
    server.go                   # MQTT 连接管理
    mqtt_to_k8s.go             # MQTT → K8s 转发
    k8s_to_mqtt.go             # K8s → MQTT 转发
  
  controller/
    job_controller.go           # Job → TaskGroup 创建和状态聚合
    taskgroup_controller.go     # TaskGroup → Task 创建和状态聚合
    task_controller.go          # Task 调度和监控
    robot_controller.go         # Robot 业务逻辑
  
  agent/
    task_executor.go            # 任务执行器（优化版）
      - monitorAllTasks()       # 单协程监控所有任务
      - checkTask()             # 检查单个任务状态和超时
      - terminateTask()         # 终止超时任务
    task_executor_test.go       # 完整测试套件
      - TestTaskExecutor_ConcurrentTasks   # 并发测试
      - TestTaskExecutor_Timeout           # 超时测试
      - TestTaskExecutor_TaskLifecycle     # 生命周期测试
  
  driver/
    exec_driver_nomad.go        # Nomad Executor 驱动
    types.go                    # 驱动接口定义

cmd/
  server/main.go                # Server 入口
  manager/main.go               # Manager 入口
  agent/main.go                 # Agent 入口

config/
  crd/                          # CRD Manifests
    robot.k8s4r.io_jobs.yaml
    robot.k8s4r.io_taskgroups.yaml
    robot.k8s4r.io_tasks.yaml
    robot.k8s4r.io_robots.yaml
```

---

## 关键设计点

### 1. Task 状态流转

```
"" (创建) 
  → Manager(TaskController) 分配 Robot 
  → "dispatching" (触发 Server 转发)
  → "running" (Agent 执行中)
  → "completed"/"failed" (执行结束)
  ↑
  └─ "failed" (超时或错误)
```

### 2. Server 转发触发条件

Server 只在以下情况转发消息到 MQTT：
- Task.Status.State 变为 "dispatching"
- Task 被删除（DeletionTimestamp != nil）

### 3. Manager 调度策略

Manager 可以实现复杂的调度逻辑：
- Job.robotSelector 匹配
- 资源可用性检查
- 负载均衡
- 亲和性/反亲和性
- 优先级队列

这些逻辑都在 Manager 中，Server 无需关心。

### 4. Agent 监控机制

**单协程监控设计**:
- 第一个任务启动时创建监控协程
- 监控协程每 5 秒循环检查所有任务
- 对每个任务调用 `checkTask()`:
  - 检查任务是否仍在运行
  - 计算运行时间 = time.Since(startTime)
  - 如果超时，调用 `terminateTask()`
- 任务完成后从监控列表移除
- 所有任务完成后监控协程自动退出

**超时处理流程**:
```go
// 检测超时
if task.Spec.Timeout != nil && elapsed > task.Spec.Timeout.Duration {
    terminateTask(task, "Task timeout")
}

// 终止任务
func terminateTask(task, reason) {
    driver.Stop(ctx)      // 发送 SIGTERM
    time.Sleep(killTimeout)
    driver.Destroy(ctx)   // 强制 SIGKILL
    reportStatus(failed, reason)
}
```
