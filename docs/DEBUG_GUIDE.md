# K8s4R 调试指南 - 执行 ls 命令

本指南将帮助你从零开始启动系统，并让 Robot 执行一个简单的 `ls` 命令。

## ⚡ 重要更新

### 最新特性（2025-11-26）

**TaskGroup 架构升级**:
- ✅ TaskGroup 作为执行单元（类似 Kubernetes Pod）
- ✅ Agent 使用 TaskGroupManager 管理所有 TaskGroup
- ✅ 每个 TaskGroup 拥有独立的 TaskExecutor 实例
- ✅ 同一 TaskGroup 内的 Tasks 可以互相看见（通过 GetAllTasks）
- ✅ 不同 TaskGroup 之间的 Tasks 完全隔离
- ✅ Manager 通过 TaskGroupWatcher 自动推送 scheduled 状态的 TaskGroup
- ✅ gRPC Stream 使用 TaskGroupCommand/TaskGroupEvent 通信
- ✅ MQTT Topic 更新：
  - `robot/{robotName}/taskgroup` - Agent 接收 TaskGroup
  - `robot/{robotName}/taskgroup/status` - Agent 上报 TaskGroup 状态

**InitTask 守护进程支持**:
- ✅ daemon=true 的 InitTask 持续运行
- ✅ 2 秒健康检查确保守护进程启动成功
- ✅ TaskGroup 结束时自动清理所有守护进程

### 历史特性（2025-11-21）

**三层资源架构**:
- ✅ Job → TaskGroup → Task 层次化管理
- ✅ TaskGroup 作为独立 CRD，可单独查询
- ✅ 状态自动级联聚合

**性能优化**:
- ✅ Agent 协程优化：N 个任务仅需 N+1 个协程（减少 45%）
- ✅ 单协程统一监控所有任务
- ✅ 超时检测：每个任务可配置独立 timeout

**MQTT Topic 改进**:
- ✅ 统一使用 `k8s4r/` 前缀
- ✅ 机器人独立命名空间 `k8s4r/robots/{robotId}/`
- ✅ 任务级别的状态上报 topic

**任务执行引擎**:
- ✅ HashiCorp Nomad Executor - 生产级进程管理
- ✅ 自动日志轮转和资源监控
- ✅ 优雅停止和子进程清理
- ✅ 可选进程隔离（cgroups）

详见：[docs/NOMAD_EXECUTOR_GUIDE.md](./NOMAD_EXECUTOR_GUIDE.md)

## 前置准备

### 1. 环境检查

```bash
# 检查 Go 版本 (需要 1.21+)
go version

# 检查 kubectl
kubectl version --client

# 检查 Docker (用于运行 MQTT Broker)
docker --version

# 检查是否有可用的 Kubernetes 集群
kubectl cluster-info
```

### 2. 安装 MQTT 客户端工具（可选，用于监控消息）

```bash
# macOS
brew install mosquitto

# 验证安装
mosquitto_sub --help
```

## 快速启动（推荐流程）

推荐按照以下顺序启动各组件：

```bash
cd $PROJECT_ROOT

# 1. 安装所有 CRD (Robot, Job, TaskGroup, Task)
kubectl apply -f config/crd/

# 验证 CRD 安装
kubectl get crd | grep robot
# 应该看到:
# jobs.robot.k8s4r.io
# taskgroups.robot.k8s4r.io
# tasks.robot.k8s4r.io
# robots.robot.k8s4r.io
```
./config/mosquitto/start-mosquitto.sh simple

# 3. 启动 Manager (Terminal 2)
go run cmd/manager/main.go

# 4. 启动 Server (Terminal 3)
go run cmd/server/main.go --broker-url=tcp://localhost:1883 --namespace=default

# 5. 启动 Agent (Terminal 4)
go run cmd/agent/main.go \
  --broker-url=tcp://localhost:1883 \
  --token=fixed-token-123 \
  --robot-id=robot-debug-01

# 6. 验证 Robot 注册 (Terminal 5)
kubectl get robots

# 7. 创建测试 Job
kubectl apply -f examples/test-ls-job.yaml

# 8. 查看结果
kubectl get tasks
```

下面是详细的步骤说明。

## 手动启动步骤

### 步骤 1: 安装 CRD

```bash
cd $PROJECT_ROOT

# 安装所有 CRD (Robot, Job, Task)
kubectl apply -f config/crd/

# 验证 CRD 安装
kubectl get crd | grep robot
# 应该看到:
# jobs.robot.k8s4r.io
# robots.robot.k8s4r.io
# tasks.robot.k8s4r.io
```

### 步骤 2: 启动 MQTT Broker

使用项目提供的启动脚本（推荐）：

```bash
# Terminal 1
cd $PROJECT_ROOT

# 使用简化模式启动 MQTT Broker
./config/mosquitto/start-mosquitto.sh simple

# 验证 Broker 运行
docker ps | grep mosquitto
```

**其他启动选项：**
```bash
# 开发模式（包含 WebSocket 支持）
./config/mosquitto/start-mosquitto.sh dev

# 生产模式（需要认证）
./config/mosquitto/start-mosquitto.sh prod

# 检查 Broker 状态
./config/mosquitto/start-mosquitto.sh status

# 停止 Broker
./config/mosquitto/start-mosquitto.sh stop
```

**验证 MQTT Broker：**
```bash
# 使用项目提供的测试脚本
./config/mosquitto/test-mqtt.sh test

# 或手动测试
mosquitto_pub -h localhost -p 1883 -t "test" -m "hello"
```

### 步骤 3: 启动 Manager (Controller)

```bash
# Terminal 2
cd $PROJECT_ROOT

# 直接运行（无需编译）
go run cmd/manager/main.go
```

**预期输出:**
```
INFO    Starting manager
INFO    Controllers initialized
INFO    Starting RobotReconciler
INFO    Starting TaskReconciler
INFO    Starting JobReconciler
```

**保持这个终端运行**，Manager 会持续监控 Robot、Job、Task 资源。

### 步骤 4: 启动 Server (MQTT Bridge)

```bash
# Terminal 3
cd $PROJECT_ROOT

# 直接运行（无需编译）
go run cmd/server/main.go --broker-url=tcp://localhost:1883 --namespace=default
```

**预期输出:**
```
INFO    Starting K8s4R Server
INFO    Connected to MQTT broker    broker=tcp://localhost:1883
INFO    Subscribed to topic    topic=k8s4r/register
INFO    Subscribed to topic    topic=k8s4r/heartbeat
INFO    Subscribed to topic    topic=robots/+/tasks/+/status
INFO    Starting Task watcher
```

**保持这个终端运行**，Server 会转发 MQTT 和 K8s 之间的消息。

### 步骤 5: 启动 Agent (模拟 Robot)

```bash
# Terminal 4
cd $PROJECT_ROOT

# 直接运行（无需编译）
go run cmd/agent/main.go \
  --broker-url=tcp://localhost:1883 \
  --token=fixed-token-123 \
  --robot-id=robot-debug-01
```

**预期输出:**
```
INFO    Starting K8s4R Agent
INFO    Robot ID: robot-debug-01
INFO    Connecting to MQTT broker    broker=tcp://localhost:1883
INFO    Connected to MQTT broker
INFO    Registering with server...
INFO    Subscribed to topic    topic=k8s4r/response/robot-debug-01
INFO    Subscribed to topic    topic=robots/robot-debug-01/tasks/dispatch
INFO    Registration successful
INFO    Starting heartbeat (interval: 30s)
```

**保持这个终端运行**，Agent 会持续发送心跳。

## 步骤 6: 验证 Robot 注册成功

```bash
# Terminal 5 (新开一个终端)
cd $PROJECT_ROOT

# 查看 Robot 资源
kubectl get robots

# 预期输出:
# NAME              PHASE    LAST HEARTBEAT
# robot-debug-01    Online   2025-11-19T10:30:00Z

# 查看详细信息
kubectl describe robot robot-debug-01
```

**检查点:**
- `Phase` 应该是 `Online`（Manager 会自动批准有心跳的 Pending Robot）
- `Last Heartbeat Time` 应该是最近的时间
- `Device Info` 应该包含 CPU、内存等信息

**如果 Robot 一直是 Pending 状态：**
- 等待 10-30 秒，Manager 会自动将有心跳的 Pending Robot 批准为 Online
- 或者手动批准：`kubectl patch robot robot-debug-01 --type=merge -p '{"status":{"phase":"Online"}}'`

## 步骤 7: 为 Robot 设置 Labels（可选）

如果你想使用 label selector 来选择 Robot，可以添加 labels：

```bash
# 给 Robot 添加 labels
kubectl patch robot robot-debug-01 --type=merge -p '{"spec":{"labels":{"env":"debug","role":"test"}}}'

# 验证 labels
kubectl get robot robot-debug-01 -o jsonpath='{.spec.labels}'
```

## 步骤 8: 创建执行 ls 命令的 Job

### 方式 1: 使用示例 YAML（推荐）

```bash
kubectl apply -f examples/test-ls-job.yaml
```

### 方式 2: 手动创建（空 selector）

```bash
kubectl apply -f - <<EOF
apiVersion: robot.k8s4r.io/v1alpha1
kind: Job
metadata:
  name: test-ls-job
  namespace: default
spec:
  name: test-ls-job
  
  # 空 selector 匹配所有 Online 的 Robot
  robotSelector: {}
  
  type: batch
  priority: 100
  
  taskGroups:
    - name: ls-group
      count: 1  # 只创建 1 个 Task
      
      tasks:
        - name: ls-task
          driver: exec
          config:
            execConfig:
              command: /bin/ls
              args:
                - -lah
                - /tmp
          
          env:
            DEBUG: "true"
EOF
```

### 方式 3: 使用 label selector

如果之前设置了 Robot labels（步骤 7），可以使用 label selector：

```bash
kubectl apply -f - <<EOF
apiVersion: robot.k8s4r.io/v1alpha1
kind: Job
metadata:
  name: test-ls-job-with-selector
spec:
  name: test-ls-job-with-selector
  
  # 选择 env=debug 的 Robot
  robotSelector:
    env: debug
  
  type: batch
  taskGroups:
    - name: ls-group
      count: 1
      tasks:
        - name: ls-task
          driver: exec
          config:
            execConfig:
              command: /bin/ls
              args:
                - -lah
                - /tmp
EOF
```

## 步骤 9: 监控 Job 和 Task 执行

```bash
# 查看 Job 状态
kubectl get jobs.robot.k8s4r.io
# 预期: test-ls-job    batch    running

# 查看 Task 状态
kubectl get tasks
# 预期: test-ls-job-ls-group-ls-task-0    pending/dispatching/running/completed

# 查看 Task 详情
kubectl describe task test-ls-job-ls-group-ls-task-0

# 实时监控 Task 状态变化
kubectl get tasks -w
```

## 步骤 10: 查看各组件日志

## 步骤 10: 查看各组件日志

### Terminal 2 (Manager 日志)
查找类似输出:
```
INFO    Reconciling Job    job=test-ls-job
INFO    Found matching robots    count=1 selector={}
INFO    Robot matched selector    robot=robot-debug-01
INFO    Task created    task=test-ls-job-ls-group-ls-task-0 robot=robot-debug-01
```

### Terminal 3 (Server 日志)
查找类似输出:
```
INFO    Task state changed to dispatching    task=test-ls-job-ls-group-ls-task-0
INFO    Publishing task to MQTT    topic=robots/robot-debug-01/tasks/dispatch
```

### Terminal 4 (Agent 日志)
查找类似输出:
```
INFO    Received task    taskUID=xxx driver=exec
INFO    Executing command    command=/bin/ls args=[-lah /tmp]
INFO    Task completed    taskUID=xxx exitCode=0
INFO    Reported task status    state=completed
```

## 步骤 11: 验证任务执行结果

```bash
# 查看 Task 最终状态
kubectl get tasks

# 查看 Task 状态字段
kubectl get task test-ls-job-ls-group-ls-task-0 -o jsonpath='{.status.state}'
# 预期: completed

# 查看完整的 Task 信息
kubectl get task test-ls-job-ls-group-ls-task-0 -o yaml

# 查看 Job 状态
kubectl get job test-ls-job -o jsonpath='{.status}'
```

### 📄 查看任务日志（Nomad Executor 管理）

**Agent 使用 Nomad executor 执行任务，日志自动保存到本地文件：**

```bash
# 任务日志目录结构（在运行 Agent 的机器上）
/var/lib/k8s4r/tasks/
└── <task-uid>/
    └── logs/
        ├── stdout.log    # 标准输出（自动轮转）
        └── stderr.log    # 标准错误（自动轮转）

# 查看任务输出
# 替换 <task-uid> 为实际的 Task UID
sudo tail -f /var/lib/k8s4r/tasks/<task-uid>/logs/stdout.log

# 查看所有任务
ls -la /var/lib/k8s4r/tasks/

# 查看最新任务的日志
ls -lt /var/lib/k8s4r/tasks/ | head -2 | tail -1 | awk '{print $NF}' | xargs -I {} cat /var/lib/k8s4r/tasks/{}/logs/stdout.log
```

**日志特性（由 Nomad executor 提供）：**
- ✅ 自动创建日志目录
- ✅ stdout/stderr 分离
- ✅ 自动日志轮转（防止磁盘占满）
- ✅ 进程结束后日志保留
- ✅ 支持实时 tail -f

## 可选: 监控 MQTT 消息

在一个新的终端中运行:

```bash
# Terminal 6
cd $PROJECT_ROOT

# 监听所有 k8s4r 相关的 MQTT 消息
mosquitto_sub -h localhost -p 1883 -t "k8s4r/#" -v

# 或只监听特定 Robot 的消息
mosquitto_sub -h localhost -p 1883 -t "robots/robot-debug-01/#" -v

# 或只监听任务相关消息
mosquitto_sub -h localhost -p 1883 -t "robots/+/tasks/#" -v
```

**你应该看到:**
- 注册消息: `k8s4r/register`
- 心跳消息: `k8s4r/heartbeat`
- 任务下发: `robots/robot-debug-01/tasks/dispatch`
- 任务状态: `robots/robot-debug-01/tasks/{uid}/status`

## 常见问题排查

### 问题 1: Agent 无法连接 MQTT Broker

**症状:**
```
ERROR   Failed to connect to MQTT broker
```

**解决:**
```bash
# 1. 检查 Broker 是否运行
./config/mosquitto/start-mosquitto.sh status

# 或手动检查
docker ps | grep mosquitto

# 2. 检查端口是否被占用
lsof -i :1883

# 3. 重启 Broker
./config/mosquitto/start-mosquitto.sh stop
./config/mosquitto/start-mosquitto.sh simple

# 4. 测试连接
mosquitto_pub -h localhost -p 1883 -t "test" -m "hello"
```

### 问题 2: Robot 状态一直是 Pending

**症状:**
```bash
kubectl get robots
# robot-debug-01    Pending
```

**解决:**
```bash
# 1. 检查 Manager 日志 (Terminal 2)
# 应该有 "Approved robot" 的日志

# 2. 检查 Server 日志 (Terminal 3)
# 应该有 "Received register message" 的日志

# 3. 检查 Agent 日志 (Terminal 4)
# 应该有 "Registration successful" 的日志

# 4. 手动批准 Robot（如果必要）
kubectl patch robot robot-debug-01 --type=merge -p '{"status":{"phase":"Online"}}'
```

### 问题 3: Task 没有被创建

**症状:**
```bash
kubectl get jobs
# test-ls-job    pending

kubectl get tasks
# No resources found
```

**解决:**
```bash
# 1. 检查 Job 状态
kubectl describe job.robot.k8s4r.io test-ls-job

# 2. 如果显示 "No matching robots found"
# 检查 Robot 是否 Online
kubectl get robots

# 3. 检查 robotSelector 是否匹配
kubectl get job test-ls-job -o jsonpath='{.spec.robotSelector}'
kubectl get robot robot-debug-01 -o jsonpath='{.spec.labels}'

# 4. 如果使用了 selector，添加对应的 labels
kubectl patch robot robot-debug-01 --type=merge -p '{"spec":{"labels":{"env":"debug"}}}'

# 5. 或者使用空 selector
kubectl patch job test-ls-job --type=merge -p '{"spec":{"robotSelector":{}}}'
```

### 问题 4: Task 没有被下发到 Agent

**症状:**
- Task 状态一直是 `pending`
- Agent 没有收到任务

**解决:**
```bash
# 1. 检查 Task 的 targetRobot 是否设置
kubectl get task test-ls-job-ls-group-ls-task-0 -o jsonpath='{.spec.targetRobot}'

# 2. 检查 Task 状态
kubectl get task test-ls-job-ls-group-ls-task-0 -o jsonpath='{.status.state}'

# 3. 检查 TaskController 日志 (Terminal 2)
# 应该有 "Task scheduled" 或 "Dispatching task" 的日志

# 4. 检查 Server 日志 (Terminal 3)
# 应该有 "Task state changed to dispatching" 的日志

# 5. 手动触发 Task 调度（如果必要）
kubectl patch task test-ls-job-ls-group-ls-task-0 --type=merge -p '{"status":{"state":"dispatching"}}'
```

### 问题 5: Agent 执行命令失败

**症状:**
Agent 日志显示:
```
ERROR   Failed to execute command    error=...
```

**解决:**
```bash
# 1. 检查命令是否存在
which /bin/ls

# 2. 使用更简单的命令测试（echo）
kubectl apply -f examples/test-echo-job.yaml

# 3. 查看 Agent 日志中的详细错误信息
# 在 Terminal 4 查看

# 4. 检查 Agent 的工作目录权限
```

### 问题 6: MQTT Broker 无法启动

**症状:**
```
Error: Docker container already exists
```

**解决:**
```bash
# 停止并删除旧的容器
./config/mosquitto/start-mosquitto.sh stop

# 或手动清理
docker stop mosquitto 2>/dev/null || true
docker rm mosquitto 2>/dev/null || true

# 重新启动
./config/mosquitto/start-mosquitto.sh simple
```

## 清理环境

### 清理资源

```bash
# 删除 Job (会自动删除关联的 Task)
kubectl delete job.robot.k8s4r.io test-ls-job

# 删除所有 Job
kubectl delete jobs.robot.k8s4r.io --all

# 删除 Robot
kubectl delete robot robot-debug-01

# 删除所有 Robot
kubectl delete robots --all
```

### 停止所有组件

```bash
# 手动停止各个组件
# 在各个终端按 Ctrl+C 停止 Manager、Server、Agent

# 停止 MQTT Broker
./config/mosquitto/start-mosquitto.sh stop
```

## 完整的检查清单

启动系统前的检查:
- [ ] CRD 已安装 (`kubectl get crd | grep robot` 应该看到 jobs, robots, tasks)
- [ ] MQTT Broker 运行中 (`docker ps | grep mosquitto`)
- [ ] Manager 启动 (Terminal 2 有日志输出)
- [ ] Server 启动 (Terminal 3 已连接到 Broker)
- [ ] Agent 启动 (Terminal 4 注册成功)

验证 Robot:
- [ ] Robot 资源存在 (`kubectl get robots`)
- [ ] Robot 状态是 Online (`kubectl get robot robot-debug-01 -o jsonpath='{.status.phase}'`)
- [ ] Robot 有 labels (`kubectl get robot robot-debug-01 -o jsonpath='{.spec.labels}'`)

创建 Job:
- [ ] Job 创建成功 (`kubectl get jobs`)
- [ ] Task 被创建 (`kubectl get tasks`)
- [ ] Task 有 targetRobot (`kubectl get task <task-name> -o jsonpath='{.spec.targetRobot}'`)

执行验证:
- [ ] Task 状态变为 dispatching (`kubectl get task <task-name> -o jsonpath='{.status.state}'`)
- [ ] Agent 收到任务 (Terminal 4 有 "Received task" 日志)
- [ ] Agent 执行命令 (Terminal 4 有 "Executing command" 日志)
- [ ] Task 状态变为 completed

## 下一步

成功执行 `ls` 命令后，你可以尝试:

### 1. 执行更复杂的命令

```bash
kubectl apply -f - <<EOF
apiVersion: robot.k8s4r.io/v1alpha1
kind: Job
metadata:
  name: test-bash-job
spec:
  name: test-bash-job
  robotSelector: {}
  type: batch
  taskGroups:
    - name: bash-group
      count: 1
      tasks:
        - name: bash-task
          driver: exec
          config:
            execConfig:
              command: /bin/bash
              args:
                - -c
                - "ls -lah /tmp && echo 'Done!' && date"
EOF
```

### 2. 测试多个 Task 并发执行

```bash
kubectl apply -f - <<EOF
apiVersion: robot.k8s4r.io/v1alpha1
kind: Job
metadata:
  name: test-multi-job
spec:
  name: test-multi-job
  robotSelector: {}
  type: batch
  taskGroups:
    - name: multi-group
      count: 3  # 创建 3 个并发任务
      tasks:
        - name: multi-task
          driver: exec
          config:
            execConfig:
              command: /bin/echo
              args:
                - "Task completed"
EOF

# 查看所有创建的 Task
kubectl get tasks -l job=test-multi-job
```

### 3. 使用 label selector 选择特定 Robot

```bash
# 先给 Robot 添加更多 labels
kubectl patch robot robot-debug-01 --type=merge -p '{"spec":{"labels":{"env":"production","region":"us-west"}}}'

# 创建使用 selector 的 Job
kubectl apply -f - <<EOF
apiVersion: robot.k8s4r.io/v1alpha1
kind: Job
metadata:
  name: test-selector-job
spec:
  name: test-selector-job
  robotSelector:
    env: production
    region: us-west
  type: batch
  taskGroups:
    - name: selector-group
      count: 1
      tasks:
        - name: selector-task
          driver: exec
          config:
            execConfig:
              command: /bin/echo
              args:
                - "Matched by selector!"
EOF
```

### 4. 测试使用环境变量

```bash
kubectl apply -f - <<EOF
apiVersion: robot.k8s4r.io/v1alpha1
kind: Job
metadata:
  name: test-env-job
spec:
  name: test-env-job
  robotSelector: {}
  type: batch
  taskGroups:
    - name: env-group
      count: 1
      tasks:
        - name: env-task
          driver: exec
          config:
            execConfig:
              command: /bin/bash
              args:
                - -c
                - 'echo "Hello \$NAME from \$REGION"'
          env:
            NAME: "K8s4R"
            REGION: "us-west"
EOF
```

### 5. 测试 echo 命令（最简单）

```bash
kubectl apply -f examples/test-echo-job.yaml

# 查看结果
kubectl get tasks
kubectl describe task test-echo-job-echo-group-echo-task-0
```

### 6. 测试任务超时控制

超时功能允许为每个任务设置独立的执行时间限制。Agent 会每 5 秒检查一次所有运行中的任务，如果超时则自动终止。

```bash
# 创建一个会超时的任务（sleep 60s, timeout 10s）
kubectl apply -f - <<EOF
apiVersion: robot.k8s4r.io/v1alpha1
kind: Job
metadata:
  name: test-timeout-job
spec:
  name: test-timeout-job
  robotSelector: {}
  type: batch
  taskGroups:
    - name: timeout-group
      count: 1
      tasks:
        - name: timeout-task
          driver: exec
          
          # 超时配置
          timeout: 10s        # 任务总超时时间
          killTimeout: 5s     # 终止等待时间
          
          config:
            execConfig:
              command: /bin/sleep
              args: ["60"]    # 睡眠 60 秒（会在 10s 后被杀死）
EOF

# 观察任务状态变化
kubectl get tasks -w

# 任务应该在 10-15 秒内变为 failed 状态（10s timeout + 最多 5s 监控周期）
kubectl describe task test-timeout-job-timeout-group-timeout-task-0

# 查看失败原因（应该显示 "Task timeout"）
kubectl get task test-timeout-job-timeout-group-timeout-task-0 -o jsonpath='{.status.message}'
```

**预期行为**:
1. 任务开始执行（state: running）
2. 10 秒后超时被检测到
3. Agent 发送 SIGTERM 信号
4. 等待 killTimeout (5s)
5. 如果进程仍未退出，发送 SIGKILL
6. 任务状态更新为 failed，消息为 "Task timeout"

**Agent 日志应该显示**:
```
INFO  task timeout detected, terminating  taskUID=xxx timeout=10s elapsed=10.xxxs
INFO  terminating task  taskUID=xxx reason="Task timeout"
INFO  task stopped successfully  taskUID=xxx
```

---

## 单元测试

项目包含完整的单元测试套件，验证核心功能：

### 运行所有 Agent 测试

```bash
cd $PROJECT_ROOT

# 运行所有测试
go test -v ./pkg/agent -timeout 60s

# 或使用 make
make test
```

### 单独运行各项测试

```bash
# 1. 测试并发任务执行（2 个任务并行）
go test -v ./pkg/agent -run TestTaskExecutor_ConcurrentTasks -timeout 30s

# 预期: 同时启动 sleep 2s 和 sleep 5s 任务
# 应该在 5-6 秒内全部完成（而不是 7 秒）

# 2. 测试任务超时控制
go test -v ./pkg/agent -run TestTaskExecutor_Timeout -timeout 30s

# 预期: sleep 10s 任务设置 2s timeout
# 应该在 2-7 秒内被终止（2s timeout + 最多 5s 监控周期）

# 3. 测试任务生命周期
go test -v ./pkg/agent -run TestTaskExecutor_TaskLifecycle -timeout 30s

# 预期: 验证单个任务从创建到完成的完整流程
```

### 测试输出示例

```bash
=== RUN   TestTaskExecutor_ConcurrentTasks
    task_executor_test.go:203: Both tasks started successfully
    task_executor_test.go:345: Test completed successfully!
--- PASS: TestTaskExecutor_ConcurrentTasks (17.52s)

=== RUN   TestTaskExecutor_Timeout
    task_executor_test.go:433: ✓ Found timeout event at message 3
    task_executor_test.go:450: ✓ Test completed successfully! Task was terminated due to timeout.
--- PASS: TestTaskExecutor_Timeout (15.52s)

=== RUN   TestTaskExecutor_TaskLifecycle
--- PASS: TestTaskExecutor_TaskLifecycle (12.21s)

PASS
ok      github.com/hxndg/k8s4r/pkg/agent        45.250s
```

**性能验证**:
- ✅ 并发测试验证多任务同时执行
- ✅ 超时测试验证超时检测和终止机制
- ✅ 生命周期测试验证状态转换正确性
- ✅ 所有测试通过证明协程优化后功能正常

---

## MQTT Topic 验证

验证新的 topic 结构（使用 `k8s4r/` 前缀）：

```bash
# 监听所有 k8s4r 相关消息
mosquitto_sub -h localhost -p 1883 -t "k8s4r/#" -v
```

**应该看到的消息**:

```
# Agent 注册
k8s4r/register {"robotId":"robot-debug-01","token":"fixed-token-123",...}

# 心跳上报（每 30 秒）
k8s4r/heartbeat {"robotId":"robot-debug-01","timestamp":"2025-11-21T10:30:00Z"}

# 注册响应
k8s4r/robots/robot-debug-001/response {"status":"approved","message":"Registration successful"}

# 任务分发
k8s4r/robots/robot-debug-001/tasks/dispatch {"metadata":{"uid":"xxx"},"spec":{...}}

# 任务状态上报
k8s4r/robots/robot-debug-001/tasks/xxx-xxx-xxx/status {"state":"running","message":"Process started"}

# 任务状态同步（Agent 启动时恢复状态用）
k8s4r/robots/robot-debug-001/tasks/state {"tasks":[{"uid":"xxx","state":"running"}]}
```

---

## 🚀 gRPC 双向流架构测试

### 测试目标

验证 Server 与 Manager 之间的 gRPC 双向流通信，确保：
- ✅ Server 完全解耦 Kubernetes（无 K8s 依赖）
- ✅ Manager 通过 gRPC Stream 推送任务到 Server
- ✅ Server 通过 MQTT 转发任务到 Agent
- ✅ Agent 状态通过 MQTT → gRPC → Manager 上报

### 架构概览

```
┌─────────────┐  gRPC Stream   ┌─────────────┐    MQTT     ┌─────────────┐
│   Manager   │◄──────────────►│   Server    │◄───────────►│    Agent    │
│             │  TaskCommand   │             │  dispatch   │             │
│ (K8s+gRPC)  │  TaskEvent     │ (gRPC+MQTT) │  status     │   (MQTT)    │
│             │                │  无K8s依赖  │             │             │
└─────────────┘                └─────────────┘             └─────────────┘
```

### 步骤 1: 启动 MQTT Broker

**Terminal 1**:
```bash
cd $PROJECT_ROOT
./config/mosquitto/start-mosquitto.sh simple
```

**验证**:
```bash
./config/mosquitto/start-mosquitto.sh status
# 应该看到 mosquitto 进程在运行
```

---

### 步骤 2: 启动 Manager (gRPC Server)

**Terminal 2**:
```bash
cd $PROJECT_ROOT
go run cmd/manager/main.go \
  --grpc-bind-address=:9090 \
  --namespace=default
```

**期望输出**:
```
Controllers initialized
🚀 Starting gRPC server address=:9090
gRPC server listening address=:9090
starting manager
```

**验证 gRPC 端口**:
```bash
# 新开 terminal 验证
lsof -i :9090
# 应该看到 manager 进程监听 9090 端口
```

---

### 步骤 3: 启动 Server (gRPC Client + MQTT Bridge)

**Terminal 3**:
```bash
cd $PROJECT_ROOT
go run cmd/server/main.go \
  --broker-url=tcp://localhost:1883 \
  --grpc-addr=localhost:9090
```

**期望输出**:
```
🚀 Starting Server (gRPC + MQTT, NO Kubernetes dependency)
Connecting to Manager gRPC server address=localhost:9090
✅ Connected to Manager gRPC server
Initializing StreamTasks bidirectional stream
✅ StreamTasks initialized
Connecting to MQTT broker broker=tcp://localhost:1883
✅ Connected to MQTT broker
📡 Started receiving tasks from Manager stream
✅ Subscribed to MQTT topics
✅ GRPCStreamServer started successfully
```

**关键验证点**:
- ✅ gRPC 连接成功（localhost:9090）
- ✅ StreamTasks 双向流初始化成功
- ✅ MQTT 连接成功
- ✅ 订阅了 register, heartbeat, task status topics

**Manager 日志应该显示**:
```
📥 [GRPC STREAM] New stream connection registered
```

---

### 步骤 4: 启动 Agent

**Terminal 4**:
```bash
cd $PROJECT_ROOT
go run cmd/agent/main.go \
  --broker-url=tcp://localhost:1883 \
  --robot-id=robot-debug-001 \
  --token=fixed-token-123
```

**期望输出**:
```
Starting agent for robot: robot-debug-001
MQTT Broker: tcp://localhost:1883
Connected to MQTT broker: tcp://localhost:1883
Subscribed to response topic: k8s4r/robots/robot-debug-001/response
Attempting to register...
Published registration request
Received response: success=true, message=Robot registered successfully
✅ Registration successful
Starting heartbeat (interval: 30s)
Sent heartbeat
```

**Server 日志应该显示**:
```
📥 [MQTT] Received registration robotId=robot-debug-001
✅ [GRPC] Registration reported to Manager success=true
📤 [MQTT] Published response to Agent robotId=robot-debug-001 topic=k8s4r/robots/robot-debug-001/response success=true
```

**Manager 日志应该显示**:
```
📥 [GRPC] Received registration request robotId=robot-debug-001
Created/Updated Robot robotId=robot-debug-001 phase=Online
✅ Robot registered successfully
```

---

### 步骤 5: 验证 Robot 资源

**Terminal 5**:
```bash
# 查看 Robot 是否创建
kubectl get robots

# 期望输出：
# NAME               PHASE    AGE
# robot-debug-001    Online   30s

# 查看详细信息
kubectl describe robot robot-debug-001
```

**应该看到**:
```yaml
Status:
  Phase: Online
  Last Heartbeat Time: 2025-11-21T17:40:00Z
  Message: Robot is online
  Device Info:
    Hostname: xxx
    OS: darwin/arm64
    CPU: ...
```

---

### 步骤 6: 创建测试 Job (验证任务分发)

**Terminal 5**:
```bash
cat <<EOF | kubectl apply -f -
apiVersion: robot.k8s4r.io/v1alpha1
kind: Job
metadata:
  name: grpc-test-job
spec:
  robotSelector: {}  # 空 selector 匹配所有 robot
  taskGroups:
    - name: concurrent-tasks
      count: 2  # 创建 2 个并发任务
      template:
        driver: exec
        config:
          command: "sleep 3 && echo 'Task completed via gRPC Stream'"
EOF
```

---

### 步骤 7: 观察任务分发流程

**Manager 日志 (Terminal 2) 应该显示**:
```
📊 [JOB CONTROLLER] Creating TaskGroup job=grpc-test-job
✅ [JOB CONTROLLER] TaskGroup created name=grpc-test-job-concurrent-tasks

📊 [TASKGROUP CONTROLLER] Creating 2 Tasks taskGroup=grpc-test-job-concurrent-tasks count=2
✅ [TASKGROUP CONTROLLER] Task created name=grpc-test-job-concurrent-tasks-0
✅ [TASKGROUP CONTROLLER] Task created name=grpc-test-job-concurrent-tasks-1

🎯 [TASK CONTROLLER] Scheduling task task=grpc-test-job-concurrent-tasks-0
🎯 [TASK CONTROLLER] Robot selected task=grpc-test-job-concurrent-tasks-0 robot=robot-debug-001
📤 [GRPC STREAM] Pushing task to stream taskUID=xxx-xxx-xxx
```

**Server 日志 (Terminal 3) 应该显示**:
```
📥 [GRPC STREAM] Received CREATE_TASK from Manager taskUID=xxx-xxx-xxx taskName=grpc-test-job-concurrent-tasks-0
📤 [GRPC STREAM] Sent TaskEvent to Manager type=ACK taskUID=xxx-xxx-xxx
✅ [MQTT] Task dispatched successfully taskUID=xxx-xxx-xxx robot=robot-debug-001 topic=k8s4r/robots/robot-debug-001/tasks/dispatch
📤 [GRPC STREAM] Sent TaskEvent to Manager type=PUBLISHED taskUID=xxx-xxx-xxx
```

**Agent 日志 (Terminal 4) 应该显示**:
```
📥 [MQTT] Received task taskUID=xxx-xxx-xxx
▶️  Starting task taskUID=xxx-xxx-xxx command=sleep 3 && echo 'Task completed via gRPC Stream'
📤 [MQTT] Published task status state=running taskUID=xxx-xxx-xxx
[3秒后]
✅ Task completed taskUID=xxx-xxx-xxx exitCode=0
📤 [MQTT] Published task status state=exited exitCode=0
```

**Server 收到状态后上报 (Terminal 3)**:
```
📥 [MQTT] Received task status taskUID=xxx-xxx-xxx state=running
✅ [GRPC] Task status reported to Manager success=true

📥 [MQTT] Received task status taskUID=xxx-xxx-xxx state=exited
✅ [GRPC] Task status reported to Manager success=true
```

**Manager 更新状态 (Terminal 2)**:
```
📊 [TASK CONTROLLER] Task state updated task=grpc-test-job-concurrent-tasks-0 state=running
📊 [TASK CONTROLLER] Task exited exitCode=0
✅ [TASK CONTROLLER] Task completed successfully task=grpc-test-job-concurrent-tasks-0
```

---

### 步骤 8: 验证任务执行结果

```bash
# 查看 Job 状态
kubectl get jobs

# 查看 TaskGroup
kubectl get taskgroups

# 查看 Task（应该有 2 个）
kubectl get tasks

# 期望输出：
# NAME                                STATE       TARGET ROBOT       AGE
# grpc-test-job-concurrent-tasks-0    Completed   robot-debug-001    1m
# grpc-test-job-concurrent-tasks-1    Completed   robot-debug-001    1m

# 查看 Task 详情
kubectl describe task grpc-test-job-concurrent-tasks-0
```

**应该看到**:
```yaml
Spec:
  Driver: exec
  Job Name: grpc-test-job
  Target Robot: robot-debug-001
  Config:
    command: sleep 3 && echo 'Task completed via gRPC Stream'
Status:
  State: Completed
  Exit Code: 0
  Message: Task completed successfully
  Started At: 2025-11-21T17:45:00Z
  Finished At: 2025-11-21T17:45:03Z
```

---

### 🔍 关键验证点总结

| 组件 | 验证内容 | 期望结果 |
|------|---------|----------|
| **Manager** | gRPC Server 监听 9090 | ✅ 端口打开 |
| **Server** | 连接到 Manager gRPC | ✅ 连接成功 |
| **Server** | StreamTasks 初始化 | ✅ 双向流建立 |
| **Server** | 连接到 MQTT Broker | ✅ 连接成功 |
| **Server** | 订阅 MQTT topics | ✅ 订阅 3 个 topic |
| **Agent** | 注册消息发送 | ✅ MQTT publish |
| **Server** | 注册消息转发 | ✅ gRPC ReportRegistration |
| **Manager** | 创建 Robot 资源 | ✅ Robot phase=Online |
| **Server** | 响应消息回复 | ✅ MQTT publish response |
| **Agent** | 收到注册成功 | ✅ Registration successful |
| **Manager** | Task 推送到 Stream | ✅ stream.Send(TaskCommand) |
| **Server** | 接收 TaskCommand | ✅ stream.Recv() |
| **Server** | 转发到 MQTT | ✅ MQTT publish dispatch |
| **Agent** | 接收并执行任务 | ✅ 执行完成 |
| **Agent** | 状态上报 MQTT | ✅ MQTT publish status |
| **Server** | 状态转发 gRPC | ✅ ReportTaskStatus |
| **Manager** | 更新 Task 状态 | ✅ State=Completed |

---

### 📊 消息流验证

**完整的消息流应该是**:

```
1. Agent 注册:
   Agent --MQTT register--> Server --gRPC ReportRegistration--> Manager --K8s--> Create Robot
   Manager --gRPC Response--> Server --MQTT response--> Agent

2. Agent 心跳:
   Agent --MQTT heartbeat--> Server --gRPC ReportHeartbeat--> Manager --K8s--> Update Robot.LastHeartbeat

3. 任务分发:
   kubectl create Job --> Manager Controller --> Create Task
   Manager --gRPC stream.Send(TaskCommand)--> Server
   Server --gRPC stream.Send(TaskEvent.ACK)--> Manager
   Server --MQTT dispatch--> Agent
   Server --gRPC stream.Send(TaskEvent.PUBLISHED)--> Manager

4. 状态上报:
   Agent --MQTT status--> Server --gRPC ReportTaskStatus--> Manager --K8s--> Update Task.Status
```

---

### 🐛 常见问题排查

**问题 1: Server 无法连接 Manager**
```bash
# 检查 Manager 是否启动
lsof -i :9090

# 检查防火墙
telnet localhost 9090

# 查看 Manager 日志
# 应该看到 "gRPC server listening"
```

**问题 2: Agent 注册超时**
```bash
# 检查 Server 是否订阅了 register topic
# Server 日志应该显示 "Subscribed to MQTT topics"

# 监控 MQTT 消息
mosquitto_sub -h localhost -t 'k8s4r/#' -v

# 检查 response topic
mosquitto_sub -h localhost -t 'k8s4r/robots/robot-debug-001/response' -v
```

**问题 3: 任务未分发到 Agent**
```bash
# 检查 Manager 是否推送到 Stream
grep "GRPC STREAM.*Pushing" manager.log

# 检查 Server 是否接收
grep "Received CREATE_TASK" server.log

# 检查 MQTT 分发
mosquitto_sub -h localhost -t 'k8s4r/robots/+/tasks/dispatch' -v
```

**问题 4: 任务状态未更新**
```bash
# 检查 Agent 是否发送状态
# Agent 日志应该显示 "Published task status"

# 检查 Server 是否转发
grep "Task status reported to Manager" server.log

# 检查 K8s Task 资源
kubectl get tasks -w
```

---

### 🎯 测试成功标志

全部测试通过后，你应该看到：

1. **Manager**:
   - ✅ gRPC Server 运行在 9090 端口
   - ✅ 接收 Server 的 gRPC 连接
   - ✅ StreamTasks 双向流工作正常
   - ✅ 处理 Unary RPC (Registration, Heartbeat, TaskStatus)

2. **Server**:
   - ✅ 无任何 Kubernetes 依赖
   - ✅ 成功连接 Manager gRPC 和 MQTT Broker
   - ✅ 双向转发：MQTT ↔ gRPC
   - ✅ 任务分发和状态上报正常

3. **Agent**:
   - ✅ 通过 MQTT 成功注册
   - ✅ 收到注册响应
   - ✅ 定期发送心跳
   - ✅ 接收并执行任务
   - ✅ 上报任务状态

4. **Kubernetes**:
   - ✅ Robot 资源自动创建，phase=Online
   - ✅ Job 创建后自动生成 TaskGroup 和 Task
   - ✅ Task 状态正确更新（Pending → Dispatching → Running → Completed）

**整个流程验证了 gRPC 双向流架构的核心价值**:
- ✅ Server 完全解耦 Kubernetes，可独立部署
- ✅ Manager 通过 gRPC Stream 实时推送任务
- ✅ 双向流通信效率高，无需轮询
- ✅ MQTT + gRPC 混合架构工作正常

---

### 清理测试资源

测试完成后清理：

```bash
# 删除测试资源
kubectl delete job grpc-test-job
kubectl delete taskgroups --all
kubectl delete tasks --all
kubectl delete robot robot-debug-001

# 停止组件 (Ctrl+C)
# Terminal 2: Manager
# Terminal 3: Server
# Terminal 4: Agent

# 停止 MQTT Broker
./config/mosquitto/start-mosquitto.sh stop
```

---

## 🔒 测试 InitTask 功能

本节介绍如何测试 InitTask 功能，使用 `examples/test-inittask-simple.yaml` 进行验证。

### 什么是 InitTask

InitTask 用于在主任务执行前运行初始化任务，类似 Kubernetes 的 initContainers。典型应用场景：
- 启动 Envoy Proxy 作为 Sidecar（守护进程模式）
- 启动 SPIRE Agent 获取 mTLS 证书（守护进程模式）
- 创建目录、下载配置文件（一次性任务）
- 等待依赖服务就绪（一次性任务）

**InitTask 特性：**
- ✅ 按顺序执行多个 initTask
- ✅ 支持守护进程模式（daemon: true）和一次性任务（daemon: false）
- ✅ 同一 TaskGroup 的 initTask 只执行一次
- ✅ 所有 initTask 完成后才启动主任务
- ✅ **主任务完成后，daemon 进程自动清理**（不会成为孤儿进程）

**Daemon 进程生命周期：**
1. InitTask 以 `daemon: true` 启动时，进程在后台运行
2. 进程句柄保存在 `daemonProcesses` map 中，key 为 `{taskGroupName}-{initTaskName}`
3. 主任务完成时，自动调用 `cleanupDaemonProcesses()` 停止所有关联的 daemon 进程
4. Agent 停止时，也会清理所有未清理的 daemon 进程

### 快速测试步骤

#### 步骤 1: 启动所有组件

```bash
# Terminal 1: MQTT Broker
./config/mosquitto/start-mosquitto.sh simple

# Terminal 2: Manager
go run cmd/manager/main.go

# Terminal 3: Server
go run cmd/server/main.go --broker-url=tcp://localhost:1883

# Terminal 4: Agent
go run cmd/agent/main.go \
  --broker-url=tcp://localhost:1883 \
  --robot-id=robot-debug-001 \
  --token=fixed-token-123
```

#### 步骤 2: 创建 Robot 资源

```bash
# Terminal 5
kubectl apply -f - << 'EOF'
apiVersion: robot.k8s4r.io/v1alpha1
kind: Robot
metadata:
  name: robot-debug-001
  namespace: default
spec:
  robotId: robot-debug-001
status:
  phase: Online
EOF

# 验证 Robot
kubectl get robots
```

#### 步骤 3: 应用测试 Job

```bash
# 应用包含 InitTask 的测试 Job
kubectl apply -f examples/test-inittask-simple.yaml

# 观察资源创建
kubectl get jobs
kubectl get taskgroups
kubectl get tasks -w
```

#### 步骤 4: 观察 Agent 日志

在 Terminal 4（Agent）中，你应该看到：

```
2025/11/26 18:00:00 executing initTasks for taskgroup: test-group, count: 4

# 1. 第一个 initTask (非守护进程，创建目录)
2025/11/26 18:00:00 executing initTask: index=1, name=setup-dirs, daemon=false
2025/11/26 18:00:01 initTask completed successfully: setup-dirs

# 2. 第二个 initTask (守护进程，模拟 SPIRE Agent)
2025/11/26 18:00:01 executing initTask: index=2, name=mock-spire-agent, daemon=true
2025/11/26 18:00:01 initTask started as daemon: mock-spire-agent, pid: 12345

# 3. 第三个 initTask (非守护进程，等待 socket)
2025/11/26 18:00:01 executing initTask: index=3, name=wait-socket, daemon=false
2025/11/26 18:00:02 initTask completed successfully: wait-socket

# 4. 第四个 initTask (守护进程，模拟 Envoy)
2025/11/26 18:00:02 executing initTask: index=4, name=mock-envoy, daemon=true
2025/11/26 18:00:02 initTask started as daemon: mock-envoy, pid: 12346

# initTask 全部完成，启动主任务
2025/11/26 18:00:02 all initTasks completed successfully: test-group
2025/11/26 18:00:02 Starting main task: main-task
```

#### 步骤 5: 验证 InitTask 效果

**检查守护进程：**
```bash
# 查看 mock SPIRE Agent 和 Envoy 进程
ps aux | grep "Mock SPIRE Agent"
ps aux | grep "Mock Envoy"

# 应该看到两个正在运行的 bash 进程
```

**检查创建的文件：**
```bash
# 查看 initTask 创建的目录和文件
ls -la /tmp/k8s4r-test/run/spire/sockets/
ls -la /tmp/k8s4r-test/config/

# 应该看到 agent.sock 文件
```

**检查主任务状态：**
```bash
# 查看 Task 状态
kubectl get tasks

# 查看详细信息
kubectl describe task $(kubectl get tasks -o name | head -1)

# 主任务应该报告: "✓ Socket file exists - InitTasks executed successfully!"
```

#### 步骤 6: 监控 MQTT（可选）

```bash
# Terminal 6: 监控任务分发
mosquitto_sub -h localhost -t 'k8s4r/robots/robot-debug-001/tasks/dispatch' -v

# Terminal 7: 监控状态上报
mosquitto_sub -h localhost -t 'k8s4r/robots/+/tasks/+/status' -v
```

### 测试成功标志

✅ **TaskGroup 创建成功**
```bash
kubectl get taskgroups
# 应该看到: test-inittask-simple-test-group
```

✅ **4 个 InitTask 按顺序执行**
- setup-dirs (非守护) → 完成
- mock-spire-agent (守护) → 后台运行
- wait-socket (非守护) → 完成
- mock-envoy (守护) → 后台运行

✅ **守护进程持续运行**
```bash
ps aux | grep "Mock SPIRE\|Mock Envoy"
# 应该看到 2 个进程
```

✅ **主任务验证成功**
```bash
cat /tmp/k8s4r-test/run/spire/sockets/agent.sock
# 文件存在
```

✅ **Task 状态正确更新**
```bash
kubectl get tasks -o jsonpath='{.items[0].status.state}'
# 显示 "running" 或 "completed"
```

### 常见问题排查

**问题 1: InitTask 未执行**
```bash
# 检查 Job YAML 格式
kubectl get job test-inittask-simple -o yaml | grep -A 20 initTasks

# 查看 Agent 日志，确认收到任务
grep "executing initTasks" <agent-log>
```

**问题 2: 守护进程立即退出**
```bash
# 检查进程
ps aux | grep mock

# 手动测试守护进程命令
/bin/bash -c 'while true; do echo "test"; sleep 10; done' &
ps aux | grep bash
kill %1
```

**问题 3: Socket 文件未创建**
```bash
# 手动创建测试
mkdir -p /tmp/k8s4r-test/run/spire/sockets
touch /tmp/k8s4r-test/run/spire/sockets/agent.sock
ls -la /tmp/k8s4r-test/run/spire/sockets/
```

### 清理测试资源

```bash
# 删除 Job（级联删除 TaskGroup 和 Task）
kubectl delete job test-inittask-simple

# 删除 Robot
kubectl delete robot robot-debug-001

# 清理文件
rm -rf /tmp/k8s4r-test/*

# 停止守护进程
pkill -f "Mock SPIRE Agent"
pkill -f "Mock Envoy"

# 停止组件 (Ctrl+C 各终端)
# 或停止 MQTT
./config/mosquitto/start-mosquitto.sh stop
```

### 进阶测试

测试成功后，可以尝试：

**测试单元测试：**
```bash
# 测试 InitTask 执行逻辑
go test -v ./pkg/agent -run TestExecuteInitTasks

# 测试 InitTask 只执行一次
go test -v ./pkg/agent -run TestInitTasksOnlyRunOnce

# 测试网络代理配置
go test -v ./pkg/agent -run TestTaskExecutorWithNetworkProxy
```

**测试 Envoy 配置生成：**
```bash
# 测试配置生成
go test -v ./pkg/agent -run TestGenerateEnvoyConfig

# 测试 SPIFFE 集成
go test -v ./pkg/agent -run TestEnvoyConfigSPIFFEIntegration
```

**使用真实 Envoy 和 SPIRE：**
- 参考 `examples/job-with-spire-envoy.yaml`
- 需要先安装和配置 SPIRE Server/Agent
- 需要安装 Envoy 二进制文件

---
## 🚀 TaskGroup 集成测试指南（2025-11-26 最新架构）

本节介绍如何测试最新的 TaskGroup 架构，验证从 Manager → Server → Agent 的完整流程。

### 架构概览

```
Job CR (K8s)
    ↓ (创建)
TaskGroup CR (K8s)
    ↓ (状态: scheduled)
TaskGroupWatcher (Manager) ← 自动检测
    ↓ (gRPC Stream: TaskGroupCommand)
Server (gRPC → MQTT)
    ↓ (MQTT: robot/{robotName}/taskgroup)
Agent (TaskGroupManager)
    ↓ (创建)
TaskGroupExecutor (执行 InitTasks + Tasks)
    ↓ (拥有独立的)
TaskExecutor (实际执行器)
    ↓ (MQTT: robot/{robotName}/taskgroup/status)
Server → Manager
    ↓ (更新)
TaskGroup CR Status
```

### 测试步骤

#### 步骤 1: 编译所有组件

```bash
cd /home/eai/hexiaonan/k8s4r

# 编译 Manager
go build -o bin/manager ./cmd/manager

# 编译 Server
go build -o bin/server ./cmd/server

# 编译 Agent
go build -o bin/agent ./cmd/agent

# 验证编译结果
ls -lh bin/
```

#### 步骤 2: 启动 MQTT Broker

```bash
# Terminal 1
cd /home/eai/hexiaonan/k8s4r
./config/mosquitto/start-mosquitto.sh simple

# 验证
docker ps | grep mosquitto
```

#### 步骤 3: 启动 Server

```bash
# Terminal 2
cd /home/eai/hexiaonan/k8s4r
./bin/server

# 预期日志:
# INFO    gRPC server listening at :50051
# INFO    Subscribed to topic: robot/+/taskgroup/status
# INFO    Connected to Manager gRPC stream
```

#### 步骤 4: 启动 Manager

```bash
# Terminal 3
cd /home/eai/hexiaonan/k8s4r
./bin/manager

# 预期日志:
# INFO    Starting manager
# INFO    TaskGroup Watcher started
# INFO    gRPC server started on :50051
# INFO    Starting RobotReconciler
# INFO    Starting TaskGroupReconciler
```

#### 步骤 5: 启动 Agent

```bash
# Terminal 4
cd /home/eai/hexiaonan/k8s4r

# 设置 Robot 名称（需要与后续创建的 Robot CR 一致）
export ROBOT_NAME=robot-001

# 启动 Agent
./bin/agent

# 预期日志:
# INFO    Agent starting for robot: robot-001
# INFO    Connected to MQTT broker: localhost:1883
# INFO    Subscribed to topic: robot/robot-001/taskgroup
# INFO    TaskGroupManager started
```

#### 步骤 6: 创建 Robot CR

```bash
# Terminal 5
cd /home/eai/hexiaonan/k8s4r

# 创建 Robot
kubectl apply -f examples/robot.yaml

# 验证
kubectl get robot robot-001
# 应该显示 STATUS: Online (Agent 上报心跳后)
```

#### 步骤 7: 创建测试 Job

```bash
# 使用 InitTask 测试 Job
kubectl apply -f examples/test-inittask-simple.yaml

# 观察 Job 状态
kubectl get job test-inittask-simple -o yaml

# 观察 TaskGroup 状态
kubectl get taskgroup -l job=test-inittask-simple

# 查看详细信息
kubectl describe taskgroup <taskgroup-name>
```

#### 步骤 8: 观察日志流程

**Manager (Terminal 3) 预期日志:**
```
INFO    JobReconciler: Creating TaskGroup for Job test-inittask-simple
INFO    TaskGroupReconciler: Assigning robot robot-001 to TaskGroup
INFO    TaskGroupReconciler: TaskGroup state changed to: scheduled
INFO    TaskGroupWatcher: Pushing TaskGroup to robot-001
INFO    TaskGroupStreamManager: Sent TaskGroupCommand to Server
INFO    TaskGroupStreamManager: Received ACK from Server
```

**Server (Terminal 2) 预期日志:**
```
INFO    Received TaskGroupCommand: CREATE_TASKGROUP
INFO    TaskGroup UID: <taskgroup-uid>
INFO    Publishing to MQTT: robot/robot-001/taskgroup
INFO    MQTT publish successful
INFO    Sent PUBLISHED event to Manager
INFO    Received TaskGroup status from robot-001
INFO    Forwarding status to Manager (future: update Task CR)
```

**Agent (Terminal 4) 预期日志:**
```
INFO    Received TaskGroup from MQTT
INFO    TaskGroup UID: <taskgroup-uid>
INFO    Creating TaskGroupExecutor
INFO    Starting InitTask: setup-dirs
INFO    InitTask setup-dirs completed successfully
INFO    Starting daemon InitTask: mock-spire-agent
INFO    Daemon InitTask mock-spire-agent started, verifying health...
INFO    Daemon InitTask mock-spire-agent is running (health check passed)
INFO    Starting InitTask: wait-socket
INFO    InitTask wait-socket completed successfully
INFO    Starting daemon InitTask: mock-envoy
INFO    All InitTasks completed successfully
INFO    Starting main tasks...
INFO    Task main-task started
INFO    Task main-task output: Main task starting...
INFO    Task main-task output: ✓ Socket file exists - InitTasks executed successfully!
INFO    Task main-task completed successfully
INFO    Publishing TaskGroup status: completed
```

#### 步骤 9: 验证结果

```bash
# 查看 TaskGroup 状态
kubectl get taskgroup -l job=test-inittask-simple -o yaml

# 应该看到:
# status:
#   state: completed
#   assignedRobots:
#     - robot-001

# 查看 Job 状态
kubectl get job test-inittask-simple -o yaml

# 应该看到:
# status:
#   state: completed

# 验证 InitTask 创建的文件
ls -la /tmp/k8s4r-test/run/spire/sockets/agent.sock
# 应该存在

# 检查守护进程是否清理
ps aux | grep "Mock SPIRE"
ps aux | grep "Mock Envoy"
# TaskGroup 完成后应该被自动清理
```

### 监控 MQTT 消息（可选）

在单独的终端监控 MQTT 消息流：

```bash
# Terminal 6 - 监控 TaskGroup 下发
mosquitto_sub -h localhost -p 1883 -t "robot/+/taskgroup" -v

# Terminal 7 - 监控 TaskGroup 状态上报
mosquitto_sub -h localhost -p 1883 -t "robot/+/taskgroup/status" -v
```

### 测试要点验证

**✅ TaskGroup 隔离测试:**
```bash
# 创建两个 Job
kubectl apply -f examples/test-inittask-simple.yaml
# 修改 Job 名称后再次创建
# 观察 Agent 日志，确认两个 TaskGroupExecutor 互不干扰
```

**✅ 守护进程健康检查:**
```bash
# 修改 InitTask 的 daemon 为 false
# 观察是否跳过 2 秒健康检查
```

**✅ TaskGroup 自动推送:**
```bash
# 观察 Manager 的 TaskGroupWatcher 日志
# 确认在 TaskGroup 状态变为 scheduled 时自动推送
```

**✅ gRPC Stream 通信:**
```bash
# 观察 Manager 和 Server 的 gRPC 日志
# 确认 TaskGroupCommand 和 TaskGroupEvent 正常交互
```

### 常见问题排查

**问题 1: Agent 未收到 TaskGroup**
```bash
# 检查 MQTT 连接
mosquitto_sub -h localhost -p 1883 -t "robot/#" -v

# 检查 Robot 名称是否匹配
echo $ROBOT_NAME
kubectl get robot

# 检查 Server 日志是否显示 MQTT 发布成功
```

**问题 2: TaskGroup 状态未更新**
```bash
# 检查 Agent 是否上报状态
# 观察 Terminal 4 (Agent) 日志

# 检查 Server 是否订阅了状态 topic
# 观察 Terminal 2 (Server) 日志中的 "Subscribed to topic"

# 手动发布测试消息
mosquitto_pub -h localhost -p 1883 \
  -t "robot/robot-001/taskgroup/status" \
  -m '{"uid":"test","state":"running"}'
```

**问题 3: InitTask 守护进程未启动**
```bash
# 检查 Agent 日志中的健康检查
# 应该看到 "health check passed"

# 手动测试守护进程
ps aux | grep "Mock SPIRE"

# 检查进程是否在 2 秒内启动
```

**问题 4: TaskGroupWatcher 未推送**
```bash
# 检查 Manager 日志
# 应该看到 "TaskGroup Watcher started"

# 检查 TaskGroup 状态
kubectl get taskgroup -o yaml
# state 应该为 "scheduled"

# 手动触发（重启 Manager）
```

### 清理测试环境

```bash
# 删除 Job（会级联删除 TaskGroup）
kubectl delete job test-inittask-simple

# 删除 Robot
kubectl delete robot robot-001

# 清理测试文件
rm -rf /tmp/k8s4r-test/*

# 停止组件（各终端按 Ctrl+C）

# 停止 MQTT Broker
./config/mosquitto/start-mosquitto.sh stop

# 清理守护进程（如果有残留）
pkill -f "Mock SPIRE"
pkill -f "Mock Envoy"
```

### 性能测试

**并发 TaskGroup 测试:**
```bash
# 创建多个 Job
for i in {1..10}; do
  sed "s/test-inittask-simple/test-job-$i/g" examples/test-inittask-simple.yaml | kubectl apply -f -
done

# 观察 Agent 创建多个 TaskGroupExecutor
# 每个 TaskGroup 应该有独立的执行环境

# 查看所有 TaskGroup
kubectl get taskgroup

# 清理
for i in {1..10}; do
  kubectl delete job test-job-$i
done
```

### 下一步

TaskGroup 架构测试通过后，可以：
- 实现 Task 状态详细查看功能
- 添加 Task 日志查询 API
- 实现类似 `kubectl logs` 的日志查看
- 添加 TaskGroup 更新和回滚功能
