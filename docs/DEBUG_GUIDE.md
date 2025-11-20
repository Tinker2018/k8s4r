# K8s4R 调试指南 - 执行 ls 命令

本指南将帮助你从零开始启动系统，并让 Robot 执行一个简单的 `ls` 命令。

## ⚡ 重要更新

**当前版本使用 HashiCorp Nomad 的 executor 进行任务执行！**

优势：
- ✅ 生产级进程管理
- ✅ 自动日志轮转
- ✅ 完整资源监控（CPU、内存）
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
cd /Users/hxndg/code_test/k8s4r

# 1. 安装所有 CRD (Robot, Job, Task)
kubectl apply -f config/crd/

# 验证 CRD 安装
kubectl get crd | grep robot
# 应该看到:
# jobs.robot.k8s4r.io
# robots.robot.k8s4r.io
# tasks.robot.k8s4r.io
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
cd /Users/hxndg/code_test/k8s4r

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
cd /Users/hxndg/code_test/k8s4r

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
cd /Users/hxndg/code_test/k8s4r

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
cd /Users/hxndg/code_test/k8s4r

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
cd /Users/hxndg/code_test/k8s4r

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
cd /Users/hxndg/code_test/k8s4r

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
cd /Users/hxndg/code_test/k8s4r

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
