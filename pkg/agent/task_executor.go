package agent

import (
	"context"
	"encoding/json"
	"fmt"
	"sync"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	"github.com/hashicorp/go-hclog"
	robotv1alpha1 "github.com/hxndghxndg/k8s4r/api/v1alpha1"
	"github.com/hxndghxndg/k8s4r/pkg/driver"
	"github.com/hxndghxndg/k8s4r/pkg/util"
)

type TaskExecutor struct {
	task       *robotv1alpha1.Task
	driver     driver.TaskDriver
	logger     hclog.Logger
	baseDir    string
	mqttClient mqtt.Client
	robotName  string

	// 运行时状态（多个 goroutine 并发访问，需要锁保护）
	mu        sync.RWMutex       // 保护下面的字段，防止数据竞争
	handle    *driver.TaskHandle // Driver 返回的进程句柄
	state     TaskState          // 当前状态（被 monitor goroutine 更新）
	exitCode  int                // 退出码（任务完成后设置）
	message   string             // 状态描述信息
	startTime time.Time          // 启动时间（用于超时检测）
	endTime   time.Time          // 结束时间

	// 事件通知
	eventListeners []TaskEventListener // 事件监听器列表

	// 控制（用于 goroutine 协调）
	ctx       context.Context    // 全局上下文（取消时通知 monitor 停止）
	cancel    context.CancelFunc // 取消函数
	monitorWg sync.WaitGroup     // 等待 monitor goroutine 结束

	hookManager *util.HookManager
}

// TaskState 任务状态
type TaskState string

const (
	TaskStateNew        TaskState = "new"
	TaskStateStarting   TaskState = "starting"
	TaskStateRunning    TaskState = "running"
	TaskStateCompleted  TaskState = "completed"
	TaskStateFailed     TaskState = "failed"
	TaskStateTerminated TaskState = "terminated"
)

// TaskEvent 任务事件
type TaskEvent struct {
	Type      TaskEventType
	TaskUID   string
	State     TaskState
	ExitCode  int
	Message   string
	Resources *driver.ResourceUsage
	Timestamp time.Time
}

// TaskEventType 事件类型
type TaskEventType string

const (
	EventTypeStateChanged  TaskEventType = "StateChanged"
	EventTypeStarted       TaskEventType = "Started"
	EventTypeCompleted     TaskEventType = "Completed"
	EventTypeFailed        TaskEventType = "Failed"
	EventTypeTerminated    TaskEventType = "Terminated"
	EventTypeResourceUsage TaskEventType = "ResourceUsage"
)

// TaskEventListener 任务事件监听器
type TaskEventListener interface {
	OnTaskEvent(event TaskEvent)
}

// TaskEventListenerFunc 函数式事件监听器
type TaskEventListenerFunc func(event TaskEvent)

func (f TaskEventListenerFunc) OnTaskEvent(event TaskEvent) {
	f(event)
}

// TaskExecutorConfig 任务执行器配置
type TaskExecutorConfig struct {
	Task       *robotv1alpha1.Task
	Driver     driver.TaskDriver
	Logger     hclog.Logger
	BaseDir    string
	MQTTClient mqtt.Client
	RobotName  string
}

// NewTaskExecutor 创建单个任务执行器
func NewTaskExecutor(config TaskExecutorConfig) (*TaskExecutor, error) {
	if config.Task == nil {
		return nil, fmt.Errorf("task is required")
	}
	if config.Driver == nil {
		return nil, fmt.Errorf("driver is required")
	}
	if config.Logger == nil {
		config.Logger = hclog.NewNullLogger()
	}
	if config.BaseDir == "" {
		config.BaseDir = "/tmp/k8s4r/tasks"
	}

	ctx, cancel := context.WithCancel(context.Background())

	hookManager := util.NewHookManager()

	new_te := &TaskExecutor{
		task:           config.Task,
		driver:         config.Driver,
		logger:         config.Logger.Named("task-executor"),
		baseDir:        config.BaseDir,
		mqttClient:     config.MQTTClient,
		robotName:      config.RobotName,
		state:          TaskStateNew,
		eventListeners: []TaskEventListener{},
		ctx:            ctx,
		cancel:         cancel,
		hookManager:    hookManager,
	}

	if err := new_te.setupHooks(); err != nil {
		new_te.mu.Lock()
		new_te.state = TaskStateFailed
		new_te.message = fmt.Sprintf("Failed to setup task hooks: %v", err)
		new_te.mu.Unlock()
		return nil, fmt.Errorf("failed to setup task hooks: %w", err)
	}

	if err := new_te.hookManager.RunPreStart(new_te.ctx, new_te.task); err != nil {
		new_te.mu.Lock()
		new_te.state = TaskStateFailed
		new_te.message = fmt.Sprintf("Failed to run Task hook PreStart: %v", err)
		new_te.mu.Unlock()
		return nil, fmt.Errorf("failed to run Task hook PreStart: %w", err)
	}

	return new_te, nil
}

// AddEventListener 添加事件监听器
func (te *TaskExecutor) AddEventListener(listener TaskEventListener) {
	te.mu.Lock()
	defer te.mu.Unlock()
	te.eventListeners = append(te.eventListeners, listener)
}

// Start 启动任务
func (te *TaskExecutor) Start(ctx context.Context) error {

	te.mu.Lock()
	if te.state != TaskStateNew {
		te.mu.Unlock()
		return fmt.Errorf("task already started")
	}
	te.state = TaskStateStarting
	te.mu.Unlock()

	te.logger.Info("starting task", "taskUID", te.task.UID, "name", te.task.Spec.Name)
	te.emitEvent(TaskEvent{
		Type:      EventTypeStateChanged,
		TaskUID:   string(te.task.UID),
		State:     TaskStateStarting,
		Message:   "Task is starting",
		Timestamp: time.Now(),
	})

	handle, err := te.driver.Start(ctx, te.task)
	if err != nil {
		te.mu.Lock()
		te.state = TaskStateFailed
		te.message = fmt.Sprintf("Failed to start: %v", err)
		te.mu.Unlock()

		te.logger.Error("failed to start task", "taskUID", te.task.UID, "error", err)
		te.emitEvent(TaskEvent{
			Type:      EventTypeFailed,
			TaskUID:   string(te.task.UID),
			State:     TaskStateFailed,
			Message:   te.message,
			Timestamp: time.Now(),
		})
		return err
	}

	te.mu.Lock()
	te.handle = handle
	te.state = TaskStateRunning
	te.startTime = time.Now()
	te.mu.Unlock()

	te.logger.Info("task started", "taskUID", te.task.UID, "pid", handle.PID)
	te.emitEvent(TaskEvent{
		Type:      EventTypeStarted,
		TaskUID:   string(te.task.UID),
		State:     TaskStateRunning,
		Message:   fmt.Sprintf("Task started with PID %d", handle.PID),
		Timestamp: time.Now(),
	})

	// 启动 monitor goroutine
	te.monitorWg.Add(1)
	go te.monitor()

	// 运行 Task executor hook 的 PostStart
	if err := te.hookManager.RunPostStart(te.ctx, te.task); err != nil {
		te.mu.Lock()
		te.state = TaskStateFailed
		te.message = fmt.Sprintf("Failed to run Task hook PostStart: %v", err)
		te.mu.Unlock()
		return fmt.Errorf("failed to run Task hook PostStart: %w", err)
	}
	return nil
}

// Stop 停止任务
func (te *TaskExecutor) Stop() error {
	te.mu.RLock()
	handle := te.handle
	state := te.state
	te.mu.RUnlock()

	if state != TaskStateRunning {
		return fmt.Errorf("task is not running")
	}
	if err := te.hookManager.RunPreStop(te.ctx, te.task); err != nil {
		te.logger.Error("failed to run Task hook PreStop", "error", err)
	}

	te.logger.Info("stopping task", "taskUID", te.task.UID)

	// 1. 取消 context，通知 monitor 停止（Context 的作用）
	// 这会让 monitor() 中的 select{case <-te.ctx.Done()} 触发
	te.cancel()

	// 2. 等待 monitor goroutine 结束（WaitGroup 的作用）
	// 这会阻塞直到 monitor() 执行完 defer te.monitorWg.Done()
	// 注意：这里只是等待 goroutine 退出，不是等待任务进程退出！
	te.monitorWg.Wait()

	// 3. 优雅停止进程（真正的杀进程逻辑）
	// Context 和 WaitGroup 不会杀进程，必须调用 driver.Stop()
	if err := te.driver.Stop(te.ctx, handle); err != nil {
		te.logger.Error("failed to stop task gracefully", "taskUID", te.task.UID, "error", err)
	}

	// 清理资源
	if err := te.driver.Destroy(te.ctx, handle); err != nil {
		te.logger.Error("failed to destroy task", "taskUID", te.task.UID, "error", err)
		return err
	}

	te.mu.Lock()
	te.state = TaskStateTerminated
	te.endTime = time.Now()
	te.message = "Task terminated by user"
	te.mu.Unlock()

	te.emitEvent(TaskEvent{
		Type:      EventTypeTerminated,
		TaskUID:   string(te.task.UID),
		State:     TaskStateTerminated,
		Message:   te.message,
		Timestamp: time.Now(),
	})

	te.logger.Info("task stopped", "taskUID", te.task.UID)
	if err := te.hookManager.RunPostStop(te.ctx, te.task); err != nil {
		te.logger.Error("failed to run Task hook PostStop", "error", err)
	}
	return nil
}

// Kill 强制杀死任务（上层调用接口）
//
// 与 Stop() 的区别：
// - Stop(): 等待任务处于 Running 状态才能停止，优雅关闭
// - Kill(): 任何状态都可以强制杀死，立即终止
//
// 工作流程：
// 1. 强制停止任务进程（driver.Stop）
// 2. 销毁任务资源（driver.Destroy）
// 3. 取消 Context（通知 monitor 停止）
// 4. 等待 monitor goroutine 退出
// 5. 更新状态为 Terminated
// 6. 发送 Terminated 事件
//
// 使用场景：
// - 用户主动取消任务
// - 上层调度器需要强制停止任务
// - 任务卡死需要强制终止
//
// Context 的作用：
// - cancel() 会通知 monitor goroutine 停止
// - 即使任务进程还在运行，monitor 也会停止监控
// - 配合 driver.Stop() 实现完整的任务终止
func (te *TaskExecutor) Kill() error {
	te.logger.Info("killing task", "taskUID", te.task.UID)

	te.mu.RLock()
	handle := te.handle
	currentState := te.state
	te.mu.RUnlock()

	// 如果已经是终止状态，直接返回
	if currentState == TaskStateCompleted || currentState == TaskStateFailed || currentState == TaskStateTerminated {
		te.logger.Info("task already terminated", "taskUID", te.task.UID, "state", currentState)
		return nil
	}

	// 1. 强制停止任务进程
	if handle != nil {
		te.logger.Info("stopping task process", "taskUID", te.task.UID)
		if err := te.driver.Stop(te.ctx, handle); err != nil {
			te.logger.Error("failed to stop task", "taskUID", te.task.UID, "error", err)
		}

		// 2. 销毁任务资源
		te.logger.Info("destroying task resources", "taskUID", te.task.UID)
		if err := te.driver.Destroy(te.ctx, handle); err != nil {
			te.logger.Error("failed to destroy task", "taskUID", te.task.UID, "error", err)
		}
	}

	// 3. 取消 Context（通知 monitor 停止）
	te.logger.Info("canceling monitor goroutine", "taskUID", te.task.UID)
	te.cancel()

	// 4. 等待 monitor goroutine 退出
	te.logger.Info("waiting for monitor goroutine to exit", "taskUID", te.task.UID)
	te.monitorWg.Wait()

	// 5. 更新状态
	te.mu.Lock()
	te.state = TaskStateTerminated
	te.endTime = time.Now()
	te.message = "Task killed by user"
	te.mu.Unlock()

	// 6. 发送事件
	te.emitEvent(TaskEvent{
		Type:      EventTypeTerminated,
		TaskUID:   string(te.task.UID),
		State:     TaskStateTerminated,
		Message:   "Task killed by user",
		Timestamp: time.Now(),
	})

	te.logger.Info("task killed successfully", "taskUID", te.task.UID)
	return nil
}

// Wait 等待任务完成（阻塞直到 monitor 结束）
func (te *TaskExecutor) Wait() error {
	te.monitorWg.Wait()
	if err := te.hookManager.RunPreStop(te.ctx, te.task); err != nil {
		te.logger.Error("failed to run Task hook PreStop", "error", err)
	}

	te.mu.RLock()
	defer te.mu.RUnlock()

	if te.state == TaskStateFailed {
		return fmt.Errorf("task failed: %s", te.message)
	}

	if err := te.hookManager.RunPostStop(te.ctx, te.task); err != nil {
		te.logger.Error("failed to run Task hook PostStop", "error", err)
	}

	return nil
}

// GetState 获取当前状态
func (te *TaskExecutor) GetState() TaskState {
	te.mu.RLock()
	defer te.mu.RUnlock()
	return te.state
}

// GetStatus 获取完整状态信息
func (te *TaskExecutor) GetStatus() TaskStatus {
	te.mu.RLock()
	defer te.mu.RUnlock()

	return TaskStatus{
		TaskUID:   string(te.task.UID),
		State:     te.state,
		ExitCode:  te.exitCode,
		Message:   te.message,
		StartTime: te.startTime,
		EndTime:   te.endTime,
	}
}

// TaskStatus 任务状态信息
type TaskStatus struct {
	TaskUID   string
	State     TaskState
	ExitCode  int
	Message   string
	StartTime time.Time
	EndTime   time.Time
}

// monitor 监控任务状态（独立 goroutine）
//
// 为什么需要 monitor goroutine？
// 1. Driver.Start() 只启动进程，不会主动通知任务完成
// 2. 需要定期轮询 Driver.Status() 检查进程是否退出
// 3. 需要检测超时并强制终止任务
// 4. 需要收集资源使用情况并发送事件
//
// 不能在主 goroutine 中阻塞等待，因为：
// - 用户可能需要同时管理多个任务
// - 需要支持查询状态、主动停止等操作
//
// Context 和 WaitGroup 的配合：
// - monitorWg.Add(1): Start() 中调用，表示有 1 个 goroutine 要运行
// - defer monitorWg.Done(): monitor() 开始时设置，确保退出时通知
// - cancel(): Stop() 中调用，向 ctx.Done() 发送信号
// - monitorWg.Wait(): Stop() 中调用，阻塞直到 Done() 被调用
//
// 为什么需要锁：
// - te.state, te.exitCode 等变量被多个 goroutine 访问
// - monitor() goroutine 读写这些变量
// - Stop(), GetState() 等方法也读写这些变量
// - 没有锁会导致数据竞争（go run -race 会报错）
func (te *TaskExecutor) monitor() {
	defer te.monitorWg.Done() // goroutine 结束时通知 WaitGroup

	ticker := time.NewTicker(5 * time.Second)
	defer ticker.Stop()

	for {
		select {
		case <-te.ctx.Done(): // Context 取消（调用 Stop() 或 cancel()）
			te.logger.Debug("monitor stopped", "taskUID", te.task.UID)
			return

		case <-ticker.C: // 定期检查（每 5 秒）
			te.checkStatus() // 真正的业务逻辑
		}
	}
}

// checkStatus 检查任务状态
//
// 这个函数负责：
// 1. 检测超时（比较 elapsed 和 timeout）
// 2. 获取 Driver 状态（调用 driver.Status()）
// 3. 更新内部状态（te.state, te.exitCode 等）
// 4. 发送状态变化事件
//
// 注意：Context 和 WaitGroup 不负责这些业务逻辑！
// - Context: 只负责通知 monitor goroutine 停止
// - WaitGroup: 只负责等待 monitor goroutine 结束
// - 锁: 必须保留，用于保护 te.state 等共享变量
func (te *TaskExecutor) checkStatus() {
	// 读取共享变量（需要锁保护，防止数据竞争）
	te.mu.RLock()
	handle := te.handle
	currentState := te.state
	te.mu.RUnlock()

	// 如果已经是终止状态，取消 context 让 monitor 退出
	if currentState == TaskStateCompleted || currentState == TaskStateFailed || currentState == TaskStateTerminated {
		te.cancel() // Context 的作用：通知 monitor 退出
		return
	}

	// 1. 检测超时（真正的超时检测逻辑）
	if te.task.Spec.Timeout != nil {
		te.mu.RLock()
		elapsed := time.Since(te.startTime) // 计算已运行时间
		te.mu.RUnlock()

		// 超时检测：比较运行时间和超时时间
		if elapsed > te.task.Spec.Timeout.Duration {
			te.logger.Info("task timeout detected", "taskUID", te.task.UID, "timeout", te.task.Spec.Timeout.Duration)
			te.handleTimeout() // 超时处理：杀死进程、更新状态、发送事件
			return
		}
	}

	// 2. 获取任务状态（调用 Driver）
	status, err := te.driver.Status(te.ctx, handle)
	if err != nil {
		te.logger.Error("failed to get task status", "taskUID", te.task.UID, "error", err)
		return
	}

	// 3. 更新内部状态（需要锁保护）
	te.updateStateFromDriverStatus(*status)

	// 4. 发送资源使用情况事件
	if status.Resources != nil {
		te.emitEvent(TaskEvent{
			Type:      EventTypeResourceUsage,
			TaskUID:   string(te.task.UID),
			State:     currentState,
			Resources: status.Resources,
			Timestamp: time.Now(),
		})
	}
}

// updateStateFromDriverStatus 根据 Driver 状态更新任务状态
//
// 这是真正的"状态上报"实现：
// 1. 将 Driver 的状态转换为 TaskState
// 2. 检测状态变化
// 3. 更新共享变量（需要锁保护）
// 4. 发送状态变化事件
//
// 注意：这里的锁是必须的！
// 如果没有锁，Stop() 和 monitor() 并发访问 te.state 会造成数据竞争。
// Go 的竞态检测器（-race）会报错。
func (te *TaskExecutor) updateStateFromDriverStatus(status driver.TaskStatus) {
	te.mu.Lock()
	oldState := te.state

	switch status.State {
	case driver.TaskStateRunning:
		te.state = TaskStateRunning
		te.message = "Task is running"

	case driver.TaskStateExited:
		te.endTime = time.Now()
		te.exitCode = status.ExitCode

		if status.ExitCode == 0 {
			te.state = TaskStateCompleted
			te.message = "Task completed successfully"
		} else {
			te.state = TaskStateFailed
			te.message = fmt.Sprintf("Task failed with exit code %d", status.ExitCode)
		}

	case driver.TaskStateFailed:
		te.endTime = time.Now()
		te.state = TaskStateFailed
		te.exitCode = status.ExitCode
		te.message = status.Message
	}

	if oldState != te.state {
		te.logger.Info("task state changed",
			"taskUID", te.task.UID,
			"oldState", oldState,
			"newState", te.state,
			"exitCode", te.exitCode)

		state := te.state
		exitCode := te.exitCode
		message := te.message
		te.mu.Unlock()

		event := TaskEvent{
			Type:      EventTypeStateChanged,
			TaskUID:   string(te.task.UID),
			State:     state,
			ExitCode:  exitCode,
			Message:   message,
			Timestamp: time.Now(),
		}

		if state == TaskStateCompleted {
			event.Type = EventTypeCompleted
		} else if state == TaskStateFailed {
			event.Type = EventTypeFailed
		}

		te.emitEvent(event)

		if state == TaskStateCompleted || state == TaskStateFailed {
			te.publishFinalStatus()
			te.cleanup()
		}
	} else {
		te.mu.Unlock()
	}
}

// handleTimeout 处理超时
//
// 这是真正的"超时杀死"实现：
// 1. 尝试优雅关闭（发送 SIGTERM）
// 2. 等待 5 秒
// 3. 强制杀死（发送 SIGKILL）
// 4. 更新状态为 Failed
// 5. 发送 Failed 事件
//
// 注意：Context.cancel() 不会杀死进程！
// cancel() 只会通知 monitor goroutine 停止，不会杀死任务进程。
// 必须调用 driver.Stop() 或 driver.Signal() 来杀死进程。
func (te *TaskExecutor) handleTimeout() {
	te.mu.Lock()
	handle := te.handle
	te.state = TaskStateFailed
	te.endTime = time.Now()
	te.message = "Task timeout"
	te.mu.Unlock()

	te.logger.Info("terminating task due to timeout", "taskUID", te.task.UID)

	if err := te.driver.Stop(te.ctx, handle); err != nil {
		te.logger.Error("failed to stop task", "taskUID", te.task.UID, "error", err)
	}

	if err := te.driver.Destroy(te.ctx, handle); err != nil {
		te.logger.Error("failed to destroy task", "taskUID", te.task.UID, "error", err)
	}

	te.emitEvent(TaskEvent{
		Type:      EventTypeFailed,
		TaskUID:   string(te.task.UID),
		State:     TaskStateFailed,
		Message:   "Task timeout",
		Timestamp: time.Now(),
	})

	te.publishFinalStatus()
}

// cleanup 清理资源
func (te *TaskExecutor) cleanup() {
	te.mu.RLock()
	handle := te.handle
	te.mu.RUnlock()

	if handle != nil {
		if err := te.driver.Destroy(te.ctx, handle); err != nil {
			te.logger.Error("failed to destroy task during cleanup", "taskUID", te.task.UID, "error", err)
		}
	}
}

// emitEvent 发送事件到所有监听器
func (te *TaskExecutor) emitEvent(event TaskEvent) {
	te.mu.RLock()
	listeners := make([]TaskEventListener, len(te.eventListeners))
	copy(listeners, te.eventListeners)
	te.mu.RUnlock()

	for _, listener := range listeners {
		go listener.OnTaskEvent(event)
	}
}

// publishFinalStatus 发布最终状态到 MQTT
func (te *TaskExecutor) publishFinalStatus() {
	if te.mqttClient == nil || te.robotName == "" {
		return
	}

	te.mu.RLock()
	status := TaskStatusMessage{
		TaskUID:   string(te.task.UID),
		RobotName: te.robotName,
		State:     string(te.state),
		ExitCode:  te.exitCode,
		Message:   te.message,
		UpdatedAt: time.Now(),
	}
	te.mu.RUnlock()

	payload, err := json.Marshal(status)
	if err != nil {
		te.logger.Error("failed to encode status", "error", err)
		return
	}

	topic := fmt.Sprintf("k8s4r/robots/%s/tasks/%s/status", te.robotName, te.task.UID)
	token := te.mqttClient.Publish(topic, 1, false, payload)
	token.Wait()

	if token.Error() != nil {
		te.logger.Error("failed to publish status", "error", token.Error())
	} else {
		te.logger.Debug("published final status to MQTT", "taskUID", te.task.UID, "state", te.state)
	}
}

// TaskStatusMessage MQTT 状态消息
type TaskStatusMessage struct {
	TaskUID   string    `json:"taskUid"`
	RobotName string    `json:"robotName"`
	State     string    `json:"state"`
	ExitCode  int       `json:"exitCode"`
	Message   string    `json:"message"`
	UpdatedAt time.Time `json:"updatedAt"`
}

// setupHooks 初始化 Task 级别的 hooks
func (te *TaskExecutor) setupHooks() error {
	te.logger.Info("setting up Task hooks")
	cgroupHook := util.NewCgroupHook(te.logger, "/sys/fs/cgroup/nomad.slice/share.slice")
	sharedStorageHook := util.NewSharedStorageHook(te.logger, te.baseDir)
	te.hookManager.AddHooks(cgroupHook, sharedStorageHook)
	return nil
}
