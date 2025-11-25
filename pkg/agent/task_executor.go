package agent

import (
	"context"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"sync"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	"github.com/hashicorp/go-hclog"
	robotv1alpha1 "github.com/hxndghxndg/k8s4r/api/v1alpha1"
	"github.com/hxndghxndg/k8s4r/pkg/driver"
	"k8s.io/apimachinery/pkg/types"
)

// TaskExecutor 负责在 Agent 端执行任务
type TaskExecutor struct {
	robotName         string
	mqttClient        mqtt.Client
	driver            driver.TaskDriver
	tasks             map[string]*runningTask
	tasksMu           sync.RWMutex
	logger            Logger
	workDir           string
	monitorStop       chan struct{}                 // 用于停止 monitor 协程
	monitorOnce       sync.Once                     // 确保 monitor 只启动一次
	initTasksComplete map[string]bool               // 记录哪些 TaskGroup 的 initTasks 已完成
	initTasksMu       sync.RWMutex                  // 保护 initTasksComplete
	daemonProcesses   map[string]*driver.TaskHandle // 记录后台守护进程
	daemonMu          sync.RWMutex                  // 保护 daemonProcesses
}

// runningTask 代表一个运行中的任务
type runningTask struct {
	task       *robotv1alpha1.Task
	handle     *driver.TaskHandle
	cancelFunc context.CancelFunc
	stopChan   chan struct{}
	startTime  time.Time // 任务启动时间，用于超时检测
}

// TaskMessage 从 Manager 接收的任务消息
type TaskMessage struct {
	Action string              `json:"action"` // create, delete
	Task   *robotv1alpha1.Task `json:"task"`
}

// TaskStatusMessage 上报给 Manager 的状态消息
type TaskStatusMessage struct {
	TaskUID   string                `json:"taskUid"`
	RobotName string                `json:"robotName"`
	State     string                `json:"state"`
	ExitCode  int                   `json:"exitCode"`
	Message   string                `json:"message"`
	Event     string                `json:"event"` // 事件类型：Assigned, Downloading, DownloadFailed, Starting, Started, Completed, Failed
	Resources *driver.ResourceUsage `json:"resources,omitempty"`
	UpdatedAt time.Time             `json:"updatedAt"`
}

// Logger 简单的日志接口
type Logger interface {
	Info(msg string, args ...interface{})
	Error(msg string, args ...interface{})
	Debug(msg string, args ...interface{})
}

// NewTaskExecutor 创建任务执行器
func NewTaskExecutor(robotName string, mqttClient mqtt.Client, workDir string, logger Logger) *TaskExecutor {
	if logger == nil {
		logger = &defaultLogger{}
	}

	// 如果未指定工作目录，使用用户主目录
	if workDir == "" {
		homeDir, err := os.UserHomeDir()
		if err != nil {
			logger.Error("failed to get user home directory", "error", err)
			workDir = "/tmp/k8s4r/tasks"
		} else {
			workDir = fmt.Sprintf("%s/.k8s4r/tasks", homeDir)
		}
	}

	// 创建 Nomad hclog 日志器（适配到 Nomad executor）
	hcLogger := hclog.New(&hclog.LoggerOptions{
		Name:   "task-executor",
		Level:  hclog.Debug, // 使用 Debug 级别查看详细日志
		Output: os.Stderr,   // 输出到标准错误流
		Color:  hclog.AutoColor,
	})

	// 使用 Nomad executor 驱动（生产级别的进程管理）
	taskDriver := driver.NewNomadExecDriver(workDir, hcLogger)

	logger.Info("task executor initialized", "workDir", workDir)

	te := &TaskExecutor{
		robotName:         robotName,
		mqttClient:        mqttClient,
		driver:            taskDriver,
		tasks:             make(map[string]*runningTask),
		logger:            logger,
		workDir:           workDir,
		monitorStop:       make(chan struct{}),
		initTasksComplete: make(map[string]bool),
		daemonProcesses:   make(map[string]*driver.TaskHandle),
	}

	// 设置驱动的事件回调，用于上报执行阶段
	taskDriver.SetEventCallback(func(taskUID, event, message string) {
		te.reportStatusWithEvent(taskUID, "running", 0, message, event, nil)
	})

	return te
}

// Start 启动任务执行器
func (te *TaskExecutor) Start(ctx context.Context) error {
	// 1. 首先订阅任务状态恢复 topic（retained message）
	// 这个 topic 记录了分配给这个 robot 的所有未完成任务
	stateTopic := fmt.Sprintf("k8s4r/robots/%s/tasks/state", te.robotName)
	te.logger.Info("subscribing to task state topic for recovery", "topic", stateTopic)

	// 使用 channel 等待状态恢复完成
	stateRecoveryDone := make(chan bool, 1)

	token := te.mqttClient.Subscribe(stateTopic, 1, func(client mqtt.Client, msg mqtt.Message) {
		te.logger.Info("received state recovery message",
			"topic", msg.Topic(),
			"payloadSize", len(msg.Payload()),
			"retained", msg.Retained())
		te.handleStateRecovery(ctx, msg, stateRecoveryDone)
	})
	token.Wait()
	if err := token.Error(); err != nil {
		return fmt.Errorf("failed to subscribe to state topic: %v", err)
	}

	te.logger.Info("waiting for state recovery...", "timeout", "5s")

	// 等待状态恢复完成（最多等待 5 秒）
	select {
	case <-stateRecoveryDone:
		te.logger.Info("task state recovery completed")
	case <-time.After(5 * time.Second):
		te.logger.Info("no state to recover or timeout")
	}

	// 2. 订阅任务分发 topic
	dispatchTopic := fmt.Sprintf("k8s4r/robots/%s/tasks/dispatch", te.robotName)
	te.logger.Info("subscribing to task dispatch topic", "topic", dispatchTopic)

	token = te.mqttClient.Subscribe(dispatchTopic, 1, func(client mqtt.Client, msg mqtt.Message) {
		te.handleTaskMessage(ctx, msg)
	})

	token.Wait()
	if err := token.Error(); err != nil {
		return fmt.Errorf("failed to subscribe to task topic: %v", err)
	}

	te.logger.Info("task executor started successfully")

	// 启动统一的任务监控协程（只启动一次）
	te.startMonitor(ctx)

	return nil
}

// handleStateRecovery 处理状态恢复消息
func (te *TaskExecutor) handleStateRecovery(ctx context.Context, msg mqtt.Message, done chan bool) {
	defer func() {
		select {
		case done <- true:
		default:
		}
	}()

	te.logger.Info("handling state recovery",
		"payloadSize", len(msg.Payload()),
		"retained", msg.Retained())

	// 如果消息为空，说明没有待恢复的任务
	if len(msg.Payload()) == 0 {
		te.logger.Info("no tasks to recover - empty payload")
		return
	}

	var state struct {
		Tasks []*robotv1alpha1.Task `json:"tasks"`
	}

	if err := json.Unmarshal(msg.Payload(), &state); err != nil {
		te.logger.Error("failed to unmarshal state message", "error", err, "payload", string(msg.Payload()))
		return
	}

	te.logger.Info("recovering tasks from state", "count", len(state.Tasks))

	// 恢复每个未完成的任务
	for _, task := range state.Tasks {
		taskUID := string(task.UID)

		te.logger.Info("found task in state",
			"taskUID", taskUID,
			"name", task.Name,
			"state", task.Status.State)

		// 检查任务是否已经在运行
		te.tasksMu.RLock()
		_, exists := te.tasks[taskUID]
		te.tasksMu.RUnlock()

		if exists {
			te.logger.Info("task already running, skipping recovery", "taskUID", taskUID)
			continue
		}

		te.logger.Info("recovering task", "taskUID", taskUID, "name", task.Name)
		te.createTask(ctx, task)
	}

	te.logger.Info("state recovery finished", "recoveredCount", len(state.Tasks))
}

// handleTaskMessage 处理接收到的任务消息
func (te *TaskExecutor) handleTaskMessage(ctx context.Context, msg mqtt.Message) {
	// 打印完整的 MQTT 消息
	te.logger.Info(" [MQTT] Received task dispatch message",
		"topic", msg.Topic(),
		"payload", string(msg.Payload()))

	// 解析简化的任务消息（从 Server 发送）
	var simplifiedMsg struct {
		TaskUID     string            `json:"taskUid"`
		TaskName    string            `json:"taskName"`
		Driver      string            `json:"driver"`
		Config      string            `json:"config"`
		Timeout     int32             `json:"timeout"`
		KillTimeout int32             `json:"killTimeout"`
		Env         map[string]string `json:"env"`
	}

	if err := json.Unmarshal(msg.Payload(), &simplifiedMsg); err != nil {
		te.logger.Error("failed to unmarshal task message", "error", err)
		return
	}

	te.logger.Info(" [MQTT] Parsed task message",
		"taskUID", simplifiedMsg.TaskUID,
		"taskName", simplifiedMsg.TaskName,
		"driver", simplifiedMsg.Driver)

	// 解析 Config JSON 字符串
	var config robotv1alpha1.TaskDriverConfig
	if simplifiedMsg.Config != "" {
		if err := json.Unmarshal([]byte(simplifiedMsg.Config), &config); err != nil {
			te.logger.Error("failed to unmarshal config", "error", err, "config", simplifiedMsg.Config)
			return
		}
	}

	// 构造 Task 对象
	task := &robotv1alpha1.Task{
		Spec: robotv1alpha1.TaskSpec{
			Name:   simplifiedMsg.TaskName,
			Driver: robotv1alpha1.TaskDriverType(simplifiedMsg.Driver),
			Config: config,
			Env:    simplifiedMsg.Env,
		},
	}
	task.UID = types.UID(simplifiedMsg.TaskUID)
	task.Name = simplifiedMsg.TaskName

	// 创建任务
	te.createTask(ctx, task)
}

// createTask 创建并启动任务
func (te *TaskExecutor) createTask(ctx context.Context, task *robotv1alpha1.Task) {
	taskUID := string(task.UID)

	te.logger.Info("creating task", "taskUID", taskUID, "name", task.Name, "driver", task.Spec.Driver)

	// 检查任务是否已存在
	te.tasksMu.Lock()
	if _, exists := te.tasks[taskUID]; exists {
		te.tasksMu.Unlock()
		te.logger.Error("task already exists", "taskUID", taskUID)
		return
	}
	te.tasksMu.Unlock()

	// 立即上报任务已分配到此 Robot（Event: Assigned）
	te.reportStatusWithEvent(taskUID, "pending", 0, fmt.Sprintf("Task assigned to robot %s", te.robotName), "Assigned", nil)
	te.logger.Info("task assigned and acknowledged", "taskUID", taskUID)

	// 1. 执行 initTasks（如果这个 TaskGroup 的 initTasks 还未执行）
	taskGroupName := task.Spec.TaskGroupName
	if taskGroupName != "" {
		if err := te.executeInitTasks(ctx, taskGroupName, task); err != nil {
			te.logger.Error("failed to execute initTasks", "error", err, "taskGroup", taskGroupName)
			te.reportStatusWithEvent(taskUID, "failed", -1, fmt.Sprintf("InitTasks failed: %v", err), "InitTasksFailed", nil)
			return
		}
	}

	// 2. 如果启用了网络代理，生成 Envoy 配置并启动
	if task.Spec.Network != nil && task.Spec.Network.Enabled && taskGroupName != "" {
		// 检查 Envoy 是否已经为这个 TaskGroup 启动
		te.daemonMu.RLock()
		envoyKey := fmt.Sprintf("%s-envoy", taskGroupName)
		_, envoyRunning := te.daemonProcesses[envoyKey]
		te.daemonMu.RUnlock()

		if !envoyRunning {
			te.logger.Info("generating envoy config and starting proxy", "taskGroup", taskGroupName)

			// 生成 Envoy 配置文件
			configDir := filepath.Join(te.workDir, "envoy")
			configPath, err := generateEnvoyConfig(taskGroupName, task.Spec.Network, configDir)
			if err != nil {
				te.logger.Error("failed to generate envoy config", "error", err)
				te.reportStatusWithEvent(taskUID, "failed", -1, fmt.Sprintf("Failed to generate envoy config: %v", err), "EnvoyConfigFailed", nil)
				return
			}
			te.logger.Info("envoy config generated", "path", configPath)

			// 构造启动 Envoy 的 Task
			envoyTask := &robotv1alpha1.Task{
				Spec: robotv1alpha1.TaskSpec{
					Name:   "envoy-proxy",
					Driver: robotv1alpha1.TaskDriverExec,
					Config: robotv1alpha1.TaskDriverConfig{
						ExecConfig: &robotv1alpha1.ExecDriverConfig{
							Command: "/opt/k8s4r/bin/envoy",
							Args:    []string{"-c", configPath},
						},
					},
				},
			}
			envoyTask.Name = fmt.Sprintf("%s-envoy", taskGroupName)
			envoyTask.UID = types.UID(fmt.Sprintf("%s-envoy", taskGroupName))

			// 启动 Envoy
			handle, err := te.driver.Start(ctx, envoyTask)
			if err != nil {
				te.logger.Error("failed to start envoy", "error", err)
				te.reportStatusWithEvent(taskUID, "failed", -1, fmt.Sprintf("Failed to start envoy: %v", err), "EnvoyStartFailed", nil)
				return
			}

			// 保存 Envoy 进程句柄
			te.daemonMu.Lock()
			te.daemonProcesses[envoyKey] = handle
			te.daemonMu.Unlock()

			te.logger.Info("envoy started successfully", "pid", handle.PID)

			// 等待 Envoy 就绪（简单等待）
			time.Sleep(2 * time.Second)
		} else {
			te.logger.Info("envoy already running for taskgroup", "taskGroup", taskGroupName)
		}
	}

	// 3. 创建任务上下文
	taskCtx, cancel := context.WithCancel(ctx)

	// 4. 如果启用了网络代理，自动注入环境变量
	if task.Spec.Network != nil && task.Spec.Network.Enabled {
		if task.Spec.Env == nil {
			task.Spec.Env = make(map[string]string)
		}

		// 为每个 upstream 自动注入环境变量
		for _, upstream := range task.Spec.Network.Upstreams {
			// 注入代理端口（业务进程连接这个端口）
			envKey := fmt.Sprintf("PROXY_%s_PORT", strings.ToUpper(upstream.Name))
			task.Spec.Env[envKey] = fmt.Sprintf("%d", upstream.LocalPort)

			// 注入代理地址（127.0.0.1:port）
			envKey = fmt.Sprintf("PROXY_%s_ADDR", strings.ToUpper(upstream.Name))
			task.Spec.Env[envKey] = fmt.Sprintf("127.0.0.1:%d", upstream.LocalPort)

			// 根据协议注入连接字符串
			switch upstream.Protocol {
			case "tcp":
				if upstream.Name == "database" || strings.Contains(upstream.Name, "db") {
					// 数据库连接字符串
					task.Spec.Env["DATABASE_HOST"] = "127.0.0.1"
					task.Spec.Env["DATABASE_PORT"] = fmt.Sprintf("%d", upstream.LocalPort)
					task.Spec.Env["DATABASE_URL"] = fmt.Sprintf("postgresql://127.0.0.1:%d/mydb", upstream.LocalPort)
				}
			case "http", "grpc":
				envKey = fmt.Sprintf("%s_URL", strings.ToUpper(upstream.Name))
				protocol := "http"
				if upstream.TLS {
					protocol = "https"
				}
				task.Spec.Env[envKey] = fmt.Sprintf("%s://127.0.0.1:%d", protocol, upstream.LocalPort)
			}
		}

		te.logger.Info("injected proxy environment variables", "taskUID", taskUID, "envCount", len(task.Spec.Env))
	}

	// 5. 启动任务（包含 artifact 下载）
	handle, err := te.driver.Start(taskCtx, task)
	if err != nil {
		cancel()
		te.logger.Error("failed to start task", "taskUID", taskUID, "error", err)
		te.reportStatusWithEvent(taskUID, "failed", -1, fmt.Sprintf("Failed to start: %v", err), "StartFailed", nil)
		return
	}

	// 保存运行中的任务
	rt := &runningTask{
		task:       task,
		handle:     handle,
		cancelFunc: cancel,
		stopChan:   make(chan struct{}),
		startTime:  time.Now(), // 记录启动时间
	}

	te.tasksMu.Lock()
	te.tasks[taskUID] = rt
	te.tasksMu.Unlock()

	// 更新 MQTT state（添加正在运行的任务）
	te.updateMQTTState()

	te.logger.Info("task started successfully", "taskUID", taskUID, "pid", handle.PID)

	// 注意：不再为每个任务启动单独的 monitor 协程
	// 统一的 monitor 协程会轮询所有任务
}

// executeInitTasks 执行 TaskGroup 的初始化任务
func (te *TaskExecutor) executeInitTasks(ctx context.Context, taskGroupName string, task *robotv1alpha1.Task) error {
	// 检查是否已经执行过这个 TaskGroup 的 initTasks
	te.initTasksMu.RLock()
	completed := te.initTasksComplete[taskGroupName]
	te.initTasksMu.RUnlock()

	if completed {
		te.logger.Info("initTasks already completed for taskgroup", "taskGroup", taskGroupName)
		return nil
	}

	// 获取 initTasks 列表
	initTasks := task.Spec.InitTasks
	if len(initTasks) == 0 {
		te.logger.Info("no initTasks defined for taskgroup", "taskGroup", taskGroupName)
		te.initTasksMu.Lock()
		te.initTasksComplete[taskGroupName] = true
		te.initTasksMu.Unlock()
		return nil
	}

	te.logger.Info("executing initTasks for taskgroup", "taskGroup", taskGroupName, "count", len(initTasks))

	// 按顺序执行每个 initTask
	for i, initTaskDef := range initTasks {
		te.logger.Info("executing initTask",
			"taskGroup", taskGroupName,
			"index", i+1,
			"name", initTaskDef.Name,
			"daemon", initTaskDef.Daemon)

		// 构造临时 Task 对象用于执行
		initTask := &robotv1alpha1.Task{
			Spec: robotv1alpha1.TaskSpec{
				Name:        initTaskDef.Name,
				Driver:      initTaskDef.Driver,
				Config:      initTaskDef.Config,
				Env:         initTaskDef.Env,
				User:        initTaskDef.User,
				KillTimeout: initTaskDef.KillTimeout,
				Artifacts:   initTaskDef.Artifacts,
				Templates:   initTaskDef.Templates,
			},
		}
		initTask.Name = fmt.Sprintf("%s-init-%s", taskGroupName, initTaskDef.Name)
		initTask.UID = types.UID(fmt.Sprintf("%s-init-%d", taskGroupName, i))

		// 启动 initTask
		handle, err := te.driver.Start(ctx, initTask)
		if err != nil {
			return fmt.Errorf("failed to start initTask %s: %w", initTaskDef.Name, err)
		}

		// 如果是 daemon 进程，启动后不等待，直接继续
		if initTaskDef.Daemon {
			te.logger.Info("initTask started as daemon", "name", initTaskDef.Name, "pid", handle.PID)

			// 保存 daemon 进程句柄
			te.daemonMu.Lock()
			daemonKey := fmt.Sprintf("%s-%s", taskGroupName, initTaskDef.Name)
			te.daemonProcesses[daemonKey] = handle
			te.daemonMu.Unlock()

			continue
		}

		// 非 daemon 进程，等待其完成
		te.logger.Info("waiting for initTask to complete", "name", initTaskDef.Name)

		// 轮询等待任务完成
		ticker := time.NewTicker(1 * time.Second)
		defer ticker.Stop()

		for {
			select {
			case <-ctx.Done():
				return fmt.Errorf("initTask %s cancelled: %w", initTaskDef.Name, ctx.Err())
			case <-ticker.C:
				status, err := te.driver.Status(ctx, handle)
				if err != nil {
					return fmt.Errorf("failed to get initTask status: %w", err)
				}

				if status.State == driver.TaskStateExited {
					if status.ExitCode != 0 {
						return fmt.Errorf("initTask %s failed with exit code %d", initTaskDef.Name, status.ExitCode)
					}
					te.logger.Info("initTask completed successfully", "name", initTaskDef.Name)
					break
				} else if status.State == driver.TaskStateFailed {
					return fmt.Errorf("initTask %s failed: %s", initTaskDef.Name, status.Message)
				}
			}

			// 如果已退出，跳出等待循环
			status, _ := te.driver.Status(ctx, handle)
			if status.State == driver.TaskStateExited {
				break
			}
		}
	}

	// 标记 initTasks 完成
	te.initTasksMu.Lock()
	te.initTasksComplete[taskGroupName] = true
	te.initTasksMu.Unlock()

	te.logger.Info("all initTasks completed successfully", "taskGroup", taskGroupName)
	return nil
}

// startMonitor 启动统一的任务监控协程（只启动一次）
func (te *TaskExecutor) startMonitor(ctx context.Context) {
	te.monitorOnce.Do(func() {
		go te.monitorAllTasks(ctx)
	})
}

// monitorAllTasks 监控所有任务的状态和超时（单个协程）
func (te *TaskExecutor) monitorAllTasks(ctx context.Context) {
	ticker := time.NewTicker(5 * time.Second)
	defer ticker.Stop()

	te.logger.Info("starting unified task monitor")

	for {
		select {
		case <-ctx.Done():
			te.logger.Info("task monitor stopped by context")
			return

		case <-te.monitorStop:
			te.logger.Info("task monitor stopped")
			return

		case <-ticker.C:
			// 获取所有任务的快照
			te.tasksMu.RLock()
			tasks := make([]*runningTask, 0, len(te.tasks))
			for _, rt := range te.tasks {
				tasks = append(tasks, rt)
			}
			te.tasksMu.RUnlock()

			// 逐个检查任务状态
			for _, rt := range tasks {
				te.checkTask(ctx, rt)
			}
		}
	}
}

// checkTask 检查单个任务的状态和超时
func (te *TaskExecutor) checkTask(ctx context.Context, rt *runningTask) {
	taskUID := string(rt.task.UID)

	// 1. 检查超时
	if rt.task.Spec.Timeout != nil {
		timeout := rt.task.Spec.Timeout.Duration
		if time.Since(rt.startTime) > timeout {
			te.logger.Info("task timeout detected, terminating",
				"taskUID", taskUID,
				"timeout", timeout,
				"elapsed", time.Since(rt.startTime))

			// 超时，强制终止任务
			te.terminateTask(ctx, rt, "Task timeout")
			return
		}
	}

	// 2. 获取任务状态
	status, err := te.driver.Status(ctx, rt.handle)
	if err != nil {
		te.logger.Error("failed to get task status", "taskUID", taskUID, "error", err)
		return
	}

	// 3. 映射驱动状态到 Task 状态
	taskState := "running"
	exitCode := 0
	message := "Task is running"

	switch status.State {
	case driver.TaskStateRunning:
		taskState = "running"
		message = "Task is running"
	case driver.TaskStateExited:
		if status.ExitCode == 0 {
			taskState = "completed"
			message = "Task completed successfully"
		} else {
			taskState = "failed"
			message = fmt.Sprintf("Task failed with exit code %d", status.ExitCode)
		}
		exitCode = status.ExitCode
	case driver.TaskStateFailed:
		taskState = "failed"
		message = status.Message
		exitCode = status.ExitCode
	}

	// 4. 周期性上报状态（让 Manager 知道任务还在运行）
	te.reportStatus(taskUID, taskState, exitCode, message, status.Resources)

	te.logger.Debug("reported task status",
		"taskUID", taskUID,
		"state", taskState,
		"exitCode", exitCode)

	// 5. 如果任务已结束，清理任务
	if status.State == driver.TaskStateExited || status.State == driver.TaskStateFailed {
		te.logger.Info("task finished", "taskUID", taskUID, "state", taskState, "exitCode", exitCode)

		// 清理任务
		te.tasksMu.Lock()
		delete(te.tasks, taskUID)
		te.tasksMu.Unlock()

		// 关闭 stopChan（通知其他可能在等待的协程）
		close(rt.stopChan)
		rt.cancelFunc()

		// 更新 MQTT state（移除已完成的任务）
		te.updateMQTTState()
	}
}

// terminateTask 终止超时或需要强制停止的任务
func (te *TaskExecutor) terminateTask(ctx context.Context, rt *runningTask, reason string) {
	taskUID := string(rt.task.UID)

	te.logger.Info("terminating task", "taskUID", taskUID, "reason", reason)

	// 上报超时状态
	te.reportStatusWithEvent(taskUID, "failed", -1,
		fmt.Sprintf("Task terminated: %s", reason), "Timeout", nil)

	// 停止任务
	if err := te.driver.Stop(ctx, rt.handle); err != nil {
		te.logger.Error("failed to stop task", "taskUID", taskUID, "error", err)
	}

	// 销毁任务
	if err := te.driver.Destroy(ctx, rt.handle); err != nil {
		te.logger.Error("failed to destroy task", "taskUID", taskUID, "error", err)
	}

	// 清理任务
	te.tasksMu.Lock()
	delete(te.tasks, taskUID)
	te.tasksMu.Unlock()

	// 关闭 stopChan
	close(rt.stopChan)
	rt.cancelFunc()

	// 更新 MQTT state
	te.updateMQTTState()

	te.logger.Info("task terminated", "taskUID", taskUID)
}

// deleteTask 删除任务
func (te *TaskExecutor) deleteTask(ctx context.Context, task *robotv1alpha1.Task) {
	taskUID := string(task.UID)

	te.logger.Info("deleting task", "taskUID", taskUID)

	te.tasksMu.Lock()
	rt, exists := te.tasks[taskUID]
	if !exists {
		te.tasksMu.Unlock()
		te.logger.Error("task not found", "taskUID", taskUID)
		return
	}
	delete(te.tasks, taskUID)
	te.tasksMu.Unlock()

	// 停止任务
	if err := te.driver.Stop(ctx, rt.handle); err != nil {
		te.logger.Error("failed to stop task", "taskUID", taskUID, "error", err)
	}

	// 销毁任务
	if err := te.driver.Destroy(ctx, rt.handle); err != nil {
		te.logger.Error("failed to destroy task", "taskUID", taskUID, "error", err)
	}

	// 停止监控
	close(rt.stopChan)
	rt.cancelFunc()

	te.logger.Info("task deleted successfully", "taskUID", taskUID)
}

// reportStatus 上报任务状态到 Manager
func (te *TaskExecutor) reportStatus(taskUID, state string, exitCode int, message string, resources *driver.ResourceUsage) {
	te.reportStatusWithEvent(taskUID, state, exitCode, message, "", resources)
}

// reportStatusWithEvent 上报任务状态和事件到 Manager
func (te *TaskExecutor) reportStatusWithEvent(taskUID, state string, exitCode int, message, event string, resources *driver.ResourceUsage) {
	statusMsg := TaskStatusMessage{
		TaskUID:   taskUID,
		RobotName: te.robotName,
		State:     state,
		ExitCode:  exitCode,
		Message:   message,
		Event:     event,
		Resources: resources,
		UpdatedAt: time.Now(),
	}

	payload, err := json.Marshal(statusMsg)
	if err != nil {
		te.logger.Error("failed to marshal status message", "error", err)
		return
	}

	// 发送到状态上报 topic
	topic := fmt.Sprintf("k8s4r/robots/%s/tasks/%s/status", te.robotName, taskUID)

	token := te.mqttClient.Publish(topic, 1, false, payload)
	token.Wait()
	if err := token.Error(); err != nil {
		te.logger.Error("failed to publish status", "error", err, "topic", topic)
	} else {
		te.logger.Debug("status reported", "taskUID", taskUID, "state", state)
	}
}

// updateMQTTState 更新 MQTT 中的任务状态列表
// 使用 retained message 保存当前正在运行的任务，供 Agent 重启时恢复
func (te *TaskExecutor) updateMQTTState() {
	te.tasksMu.RLock()
	tasks := make([]*robotv1alpha1.Task, 0, len(te.tasks))
	for _, rt := range te.tasks {
		tasks = append(tasks, rt.task)
	}
	te.tasksMu.RUnlock()

	state := struct {
		Tasks []*robotv1alpha1.Task `json:"tasks"`
	}{
		Tasks: tasks,
	}

	payload, err := json.Marshal(state)
	if err != nil {
		te.logger.Error("failed to marshal state", "error", err)
		return
	}

	// 发布到 state topic，使用 retained=true 保存状态
	stateTopic := fmt.Sprintf("k8s4r/robots/%s/tasks/state", te.robotName)

	// 如果没有任务，发送空消息清除 retained message
	if len(tasks) == 0 {
		payload = []byte{}
	}

	token := te.mqttClient.Publish(stateTopic, 1, true, payload) // retained=true
	token.Wait()
	if err := token.Error(); err != nil {
		te.logger.Error("failed to publish state", "error", err, "topic", stateTopic)
	} else {
		te.logger.Info("state updated", "topic", stateTopic, "taskCount", len(tasks))
	}
}

// Stop 停止任务执行器
func (te *TaskExecutor) Stop(ctx context.Context) error {
	te.logger.Info("stopping task executor")

	// 停止 monitor 协程
	close(te.monitorStop)

	// 停止所有运行中的任务
	te.tasksMu.Lock()
	tasks := make([]*runningTask, 0, len(te.tasks))
	for _, rt := range te.tasks {
		tasks = append(tasks, rt)
	}
	te.tasksMu.Unlock()

	for _, rt := range tasks {
		_ = te.driver.Stop(ctx, rt.handle)
		_ = te.driver.Destroy(ctx, rt.handle)
		close(rt.stopChan)
		rt.cancelFunc()
	}

	te.logger.Info("task executor stopped")
	return nil
}

// defaultLogger 默认日志实现
type defaultLogger struct{}

func (l *defaultLogger) Info(msg string, args ...interface{}) {
	fmt.Printf("[INFO] %s %v\n", msg, args)
}

func (l *defaultLogger) Error(msg string, args ...interface{}) {
	fmt.Printf("[ERROR] %s %v\n", msg, args)
}

func (l *defaultLogger) Debug(msg string, args ...interface{}) {
	fmt.Printf("[DEBUG] %s %v\n", msg, args)
}
