# 进程隔离和网络可见性详解

## 问题 1: 业务进程如何知道连接本地端口？

### 答案：自动环境变量注入

#### 工作流程

```
1. 用户在 YAML 中声明网络代理需求
   ↓
2. Agent 启动 Envoy，监听 127.0.0.1:5432
   ↓
3. Agent 启动业务 Task 前，自动注入环境变量
   ↓
4. 业务进程读取环境变量，连接到代理端口
```

### 示例

#### 用户编写的 YAML（声明式）

```yaml
taskGroups:
  - name: web-service
    network:
      enabled: true
      upstreams:
        - name: database
          localPort: 5432
          upstream: db.example.com:5432
          protocol: tcp
          tls: true
        - name: api
          localPort: 8080
          upstream: api.example.com:443
          protocol: http
          tls: true
    
    tasks:
      - name: app
        driver: exec
        config:
          execConfig:
            command: python3
            args: ["/app/main.py"]
        # 注意：用户不需要手动设置 env
```

#### Agent 自动注入的环境变量

```bash
# 通用代理信息
PROXY_DATABASE_PORT=5432
PROXY_DATABASE_ADDR=127.0.0.1:5432
PROXY_API_PORT=8080
PROXY_API_ADDR=127.0.0.1:8080

# 数据库专用（检测到 name 包含 'database' 或 'db'）
DATABASE_HOST=127.0.0.1
DATABASE_PORT=5432
DATABASE_URL=postgresql://127.0.0.1:5432/mydb

# HTTP/GRPC 服务专用
API_URL=https://127.0.0.1:8080
```

#### 业务代码使用（无需修改）

```python
# /app/main.py
import os
import psycopg2
import requests

# 方法 1: 直接使用环境变量
db_conn = psycopg2.connect(os.environ['DATABASE_URL'])

# 方法 2: 分别读取 host 和 port
db_host = os.environ['DATABASE_HOST']
db_port = os.environ['DATABASE_PORT']
db_conn = psycopg2.connect(host=db_host, port=db_port, database='mydb')

# 方法 3: HTTP 服务
api_url = os.environ['API_URL']
resp = requests.get(f"{api_url}/users")
```

### 代码实现

#### pkg/agent/task_executor.go

```go
// createTask 在启动业务 Task 前自动注入环境变量
func (te *TaskExecutor) createTask(ctx context.Context, task *Task) {
    // ... 执行 initTasks
    // ... 启动 Envoy
    
    // 自动注入环境变量
    if task.Spec.Network != nil && task.Spec.Network.Enabled {
        if task.Spec.Env == nil {
            task.Spec.Env = make(map[string]string)
        }
        
        // 为每个 upstream 注入环境变量
        for _, upstream := range task.Spec.Network.Upstreams {
            // 通用代理信息
            task.Spec.Env[fmt.Sprintf("PROXY_%s_PORT", strings.ToUpper(upstream.Name))] = 
                fmt.Sprintf("%d", upstream.LocalPort)
            task.Spec.Env[fmt.Sprintf("PROXY_%s_ADDR", strings.ToUpper(upstream.Name))] = 
                fmt.Sprintf("127.0.0.1:%d", upstream.LocalPort)
            
            // 智能检测协议类型
            switch upstream.Protocol {
            case "tcp":
                if strings.Contains(upstream.Name, "database") || strings.Contains(upstream.Name, "db") {
                    task.Spec.Env["DATABASE_HOST"] = "127.0.0.1"
                    task.Spec.Env["DATABASE_PORT"] = fmt.Sprintf("%d", upstream.LocalPort)
                    task.Spec.Env["DATABASE_URL"] = fmt.Sprintf("postgresql://127.0.0.1:%d/mydb", upstream.LocalPort)
                }
            case "http", "grpc":
                protocol := "http"
                if upstream.TLS {
                    protocol = "https"
                }
                task.Spec.Env[fmt.Sprintf("%s_URL", strings.ToUpper(upstream.Name))] = 
                    fmt.Sprintf("%s://127.0.0.1:%d", protocol, upstream.LocalPort)
            }
        }
    }
    
    // 启动业务进程（环境变量已注入）
    handle, err := te.driver.Start(taskCtx, task)
}
```

### 环境变量传递流程

```
┌──────────────────────────────────────────────────────────────┐
│ Agent (pkg/agent/task_executor.go)                          │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  1. 读取 task.Spec.Network.Upstreams                        │
│     └─> database: 5432 -> db.example.com:5432               │
│                                                              │
│  2. 注入环境变量到 task.Spec.Env                            │
│     ├─> DATABASE_HOST=127.0.0.1                             │
│     ├─> DATABASE_PORT=5432                                  │
│     └─> DATABASE_URL=postgresql://127.0.0.1:5432/mydb       │
│                                                              │
│  3. 调用 driver.Start(task)                                 │
│     └─> 传递给 Nomad Executor                               │
└──────────────────────────────────────────────────────────────┘
                        ↓
┌──────────────────────────────────────────────────────────────┐
│ Nomad Executor (pkg/driver/exec_driver_nomad.go)            │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  execCmd := &executor.ExecCommand{                          │
│      Cmd:  "python3",                                       │
│      Args: ["/app/main.py"],                                │
│      Env:  buildEnv(task.Spec.Env),  // 包含注入的环境变量  │
│  }                                                          │
│                                                              │
│  execImpl.Launch(execCmd)                                   │
└──────────────────────────────────────────────────────────────┘
                        ↓
┌──────────────────────────────────────────────────────────────┐
│ 业务进程 (python3 /app/main.py)                              │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  import os                                                  │
│  db_url = os.environ['DATABASE_URL']                        │
│  # postgresql://127.0.0.1:5432/mydb                         │
│                                                              │
│  db_conn = psycopg2.connect(db_url)                         │
│  # 连接到 127.0.0.1:5432                                     │
└──────────────────────────────────────────────────────────────┘
                        ↓
┌──────────────────────────────────────────────────────────────┐
│ Envoy Proxy (监听 127.0.0.1:5432)                           │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  收到业务进程的连接请求                                       │
│  └─> 代理到 db.example.com:5432 (with mTLS)                 │
└──────────────────────────────────────────────────────────────┘
```

---

## 问题 2: Nomad Executor 的进程隔离机制

### 关键误解澄清

**您的问题**：
> "nomad的executor是怎么做到一个taskgroup可以互相看到，但是具体的task看不到对面的"

**实际情况**：
Nomad Executor **并没有实现** TaskGroup 级别的网络隔离！

### Nomad Executor 的实际隔离能力

#### 在 Linux 上

```go
// hashicorp/nomad/drivers/shared/executor/executor_linux.go
func NewExecutorWithIsolation(logger hclog.Logger, compute cpustats.Compute) Executor {
    return &UniversalExecutor{
        // ... 
        // 使用 Linux Namespaces 实现隔离：
        // - PID Namespace: 进程 ID 隔离
        // - Mount Namespace: 文件系统隔离
        // - IPC Namespace: 进程间通信隔离
        // - UTS Namespace: 主机名隔离
        // 
        // 但是 Network Namespace 默认不启用！
        // 所以进程仍然共享宿主机的网络栈
    }
}
```

**隔离能力**：
- ✅ **进程隔离**：每个 Task 的进程看不到其他 Task 的进程
- ✅ **文件系统隔离**：每个 Task 有独立的工作目录
- ✅ **资源限制**：CPU、内存限制（通过 cgroups）
- ❌ **网络隔离**：**没有**网络隔离，所有进程共享网络栈

#### 在 macOS 上

```go
// hashicorp/nomad/drivers/shared/executor/executor_default.go
func NewExecutorWithIsolation(logger hclog.Logger, compute cpustats.Compute) Executor {
    return &UniversalExecutor{
        // macOS 没有 Linux Namespaces
        // 只有基本的进程管理
        // 完全没有隔离能力！
    }
}
```

**隔离能力**：
- ❌ **进程隔离**：无
- ❌ **文件系统隔离**：无
- ❌ **资源限制**：无
- ❌ **网络隔离**：无

### k8s4r 的实际网络共享机制

我们的设计中，**同一 TaskGroup 的所有 Task 共享网络代理** 是通过以下方式实现的：

#### 1. 共享 Envoy 进程（TaskGroup 级别）

```
Robot 机器（宿主机）
├─ SPIRE Agent (守护进程)
├─ Envoy-proxy-group-1 (监听 127.0.0.1:5432)  ← TaskGroup 1 共享
│   ├─ Task-r0-app (连接 127.0.0.1:5432)
│   ├─ Task-r1-app (连接 127.0.0.1:5432)
│   └─ Task-r2-app (连接 127.0.0.1:5432)
│
└─ Envoy-proxy-group-2 (监听 127.0.0.1:5433)  ← TaskGroup 2 共享
    ├─ Task-r0-worker (连接 127.0.0.1:5433)
    └─ Task-r1-worker (连接 127.0.0.1:5433)
```

**关键点**：
- 所有进程运行在**同一台机器**上
- 所有进程共享**同一个网络栈**（127.0.0.1）
- Envoy 监听 `127.0.0.1`，只有本机进程能访问
- 不同 TaskGroup 使用**不同端口**避免冲突

#### 2. 代码实现（pkg/agent/task_executor.go）

```go
func (te *TaskExecutor) createTask(ctx context.Context, task *Task) {
    taskGroupName := task.Spec.TaskGroupName
    
    // 检查这个 TaskGroup 的 Envoy 是否已启动
    envoyKey := fmt.Sprintf("%s-envoy", taskGroupName)
    
    te.daemonMu.RLock()
    _, envoyRunning := te.daemonProcesses[envoyKey]
    te.daemonMu.RUnlock()
    
    if !envoyRunning {
        // 第一个 Task 启动 Envoy
        configPath, _ := generateEnvoyConfig(taskGroupName, task.Spec.Network, configDir)
        handle, _ := te.driver.Start(ctx, envoyTask)
        
        // 保存到全局 map（所有 Task 共享）
        te.daemonProcesses[envoyKey] = handle
    } else {
        // 后续 Task 复用已启动的 Envoy
        te.logger.Info("envoy already running for taskgroup", "taskGroup", taskGroupName)
    }
    
    // 启动业务 Task（会连接到上面的 Envoy）
    te.driver.Start(ctx, task)
}
```

### 网络可见性总结

```
┌────────────────────────────────────────────────────────────┐
│                      Robot 宿主机                           │
│                   (共享网络栈 127.0.0.1)                    │
├────────────────────────────────────────────────────────────┤
│                                                            │
│  ┌──────────────────────────────────────────────────┐    │
│  │ TaskGroup: web-service                            │    │
│  │   Envoy: 127.0.0.1:5432                          │    │
│  │   ├─ Task r0-app  ──┐                            │    │
│  │   ├─ Task r1-app  ──┼──> 都连接 127.0.0.1:5432  │    │
│  │   └─ Task r2-app  ──┘                            │    │
│  └──────────────────────────────────────────────────┘    │
│                                                            │
│  ┌──────────────────────────────────────────────────┐    │
│  │ TaskGroup: worker-group                           │    │
│  │   Envoy: 127.0.0.1:5433  (不同端口)              │    │
│  │   ├─ Task r0-worker ──┐                          │    │
│  │   └─ Task r1-worker ──┼──> 都连接 127.0.0.1:5433 │    │
│  └──────────────────────────────────────────────────┘    │
│                                                            │
│  所有进程共享同一个 localhost (127.0.0.1)                  │
│  通过不同端口实现 TaskGroup 间的"逻辑隔离"                  │
└────────────────────────────────────────────────────────────┘
```

### 真正的网络隔离需要什么？

如果要实现真正的网络隔离（不同 TaskGroup 的 Task 互相看不到），需要：

#### 方案 1: Docker/Containerd 驱动

```yaml
tasks:
  - name: app
    driver: docker  # 使用 Docker 驱动
    config:
      dockerConfig:
        image: myapp:latest
        networkMode: bridge  # 独立网络命名空间
```

每个容器有独立的网络栈，完全隔离。

#### 方案 2: Linux Network Namespace（需要修改 Nomad Executor）

```go
// 需要在 Nomad Executor 中启用 Network Namespace
execCmd := &executor.ExecCommand{
    // ...
    NetworkIsolation: &executor.NetworkIsolation{
        Mode: "group",
        Path: fmt.Sprintf("/var/run/netns/%s", taskGroupName),
    },
}
```

但这需要 root 权限和额外的网络配置。

### 当前设计的优势

我们的设计（共享 localhost）虽然没有真正的网络隔离，但有以下优势：

1. ✅ **简单**：不需要复杂的网络配置
2. ✅ **高效**：零开销，直接使用 localhost
3. ✅ **跨平台**：macOS/Linux 都支持
4. ✅ **足够用**：大多数场景下不需要网络隔离

对于真正需要网络隔离的场景，应该使用 Docker 驱动。

## 总结

| 问题 | 答案 |
|------|------|
| **业务进程如何知道连接本地端口？** | Agent **自动注入环境变量**（DATABASE_URL, API_URL 等） |
| **TaskGroup 如何共享网络？** | 所有进程运行在**同一台机器**，共享 **127.0.0.1**，通过**不同端口**区分 TaskGroup |
| **Task 之间能互相看到吗？** | ✅ 可以！因为共享网络栈（但通常不需要互相通信） |
| **Nomad Executor 提供网络隔离吗？** | ❌ 不提供！只有进程/文件系统隔离（Linux）或无隔离（macOS） |
| **如何实现真正的网络隔离？** | 使用 Docker 驱动或手动配置 Network Namespace |
