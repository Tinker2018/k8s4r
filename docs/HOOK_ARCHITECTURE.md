# Hook 架构设计

## 概述

Hook 机制提供了一种**可扩展、解耦**的方式来管理任务和任务组的生命周期。通过 Hook，我们可以将不同的关注点（cgroup 管理、存储管理、网络配置等）分离到独立的模块中。

## 架构设计

### 核心接口

```go
type CommonHook interface {
    Name() string
    PreStart(ctx context.Context, subject interface{}) error
    PostStart(ctx context.Context, subject interface{}) error
    PreStop(ctx context.Context, subject interface{}) error
    PostStop(ctx context.Context, subject interface{}) error
}
```

### 职责分层

#### Executor 的职责（核心路径）
- 创建任务的基本工作目录（`taskDir`, `logDir`）
- 配置和启动 Nomad executor
- 管理任务进程的生命周期
- 处理日志流式输出
- 提供任务状态查询接口

#### Hook 的职责（可选功能）
- **CgroupHook**: 创建和管理两层 cgroup（TaskGroup 层 + Task 层）
- **SharedStorageHook**: 管理 TaskGroup 共享目录、下载 artifacts、创建 symlink
- **未来可扩展**: NetworkHook, MetricsHook, SecurityHook 等

## 已实现的 Hooks

### 1. CgroupHook

**功能**：
- 创建两层 cgroup 结构：
  - TaskGroup 层：`/sys/fs/cgroup/nomad.slice/share.slice/<taskGroupName>`
  - Task 层：`/sys/fs/cgroup/nomad.slice/share.slice/<taskGroupName>/<taskID>`
- 引用计数管理：跟踪每个 TaskGroup 的活跃任务数
- 自动清理：当最后一个任务完成时，自动删除 TaskGroup 级别的 cgroup
- 强制清理：当 TaskGroup 被删除时，强制清理所有 cgroup

**生命周期**：
```
PreStart(Task)  -> 创建 Task 级别的 cgroup
PostStart(Task) -> 增加 TaskGroup 的引用计数
PostStop(Task)  -> 减少引用计数，可能清理 cgroup
PostStop(TaskGroupCleanupEvent) -> 强制清理 TaskGroup cgroup
```

### 2. SharedStorageHook

**功能**：
- 创建 TaskGroup 共享目录结构：
  ```
  <baseDir>/
    <taskGroupName>/
      shared/
        local/          # artifacts 下载到这里
      <taskID-1>/
        local@ -> ../shared/local  # symlink
        logs/
      <taskID-2>/
        local@ -> ../shared/local
        logs/
  ```
- 下载 artifacts 到共享目录
- 为每个 Task 创建 `local/` symlink 指向共享的 artifacts
- 清理共享目录

**生命周期**：
```
PreStart(Task)  -> 创建共享目录、下载 artifacts、创建 symlink
PostStop(TaskGroupCleanupEvent) -> 清理共享目录
```

## 执行顺序

### 任务启动流程

```
1. Executor 创建核心工作目录（taskDir, logDir）
   └─ 如果失败，立即返回错误

2. 执行 PreStart Hooks
   ├─ CgroupHook: 创建 Task 级别的 cgroup
   ├─ SharedStorageHook: 创建共享目录、下载 artifacts、创建 symlink
   └─ 如果失败，回滚（删除 taskDir），返回错误

3. Executor 配置并启动进程
   └─ 如果失败，返回错误（Hooks 会在 Destroy 时清理）

4. 执行 PostStart Hooks
   ├─ CgroupHook: 增加引用计数
   └─ 如果失败，记录警告（不影响任务运行）
```

### 任务停止流程

```
1. Executor 删除 tasks map 中的任务

2. 执行 PostStop Hooks
   ├─ CgroupHook: 清理 Task 级别的 cgroup，减少引用计数
   ├─ 如果引用计数为 0，清理 TaskGroup 级别的 cgroup
   └─ 如果失败，记录警告（继续清理）

3. Executor 清理任务目录
```

### TaskGroup 强制清理流程

```
1. Executor 调用 CleanupTaskGroup(taskGroupName)

2. 执行 PostStop Hooks（传入 TaskGroupCleanupEvent）
   ├─ CgroupHook: 强制清理 TaskGroup 的 cgroup（递归删除所有 Task 级别的 cgroup）
   ├─ SharedStorageHook: 清理共享目录
   └─ 如果失败，记录警告（继续清理）

3. Executor 清理 TaskGroup 工作目录
```

## 错误处理策略

### PreStart Hook 失败
- **影响**: 任务无法启动
- **处理**: 立即回滚已创建的资源，返回错误
- **原因**: PreStart 是任务运行的前提条件

### PostStart Hook 失败
- **影响**: 不影响任务运行
- **处理**: 记录警告，继续执行
- **原因**: PostStart 通常是辅助性操作（如计数、日志）

### PostStop Hook 失败
- **影响**: 可能导致资源泄漏
- **处理**: 记录警告，继续执行其他 Hooks 和清理
- **原因**: 确保尽可能多的资源被清理

## 扩展性

### 添加新的 Hook

```go
// 1. 实现 CommonHook 接口
type MyCustomHook struct {
    logger hclog.Logger
}

func (h *MyCustomHook) Name() string { return "my_custom" }

func (h *MyCustomHook) PreStart(ctx context.Context, subject interface{}) error {
    switch v := subject.(type) {
    case *robotv1alpha1.Task:
        // 处理 Task 启动前的逻辑
        return nil
    case *robotv1alpha1.TaskGroup:
        // 处理 TaskGroup 启动前的逻辑
        return nil
    default:
        return nil
    }
}

// 实现其他方法...

// 2. 注册 Hook
driver := NewExecDriver(baseDir, logger)
driver.AddHook(NewMyCustomHook(logger))
```

### 未来可能的 Hooks

- **NetworkHook**: 配置 Envoy 代理、端口映射、网络隔离
- **MetricsHook**: 收集任务执行指标、资源使用统计
- **SecurityHook**: 设置 SELinux 上下文、AppArmor 配置
- **VolumeHook**: 挂载持久化卷、配置 NFS/CSI
- **LogForwardHook**: 转发日志到外部系统（Elasticsearch, Loki 等）

## 优势总结

1. **解耦**: Executor 专注于进程管理，Hook 处理特定功能
2. **可测试**: 每个 Hook 可以独立测试
3. **可组合**: 根据需求选择性启用不同的 Hooks
4. **易扩展**: 添加新功能无需修改 Executor 核心代码
5. **职责清晰**: 谁创建谁清理，错误边界明确
6. **灵活配置**: 不同环境可以使用不同的 Hook 组合
