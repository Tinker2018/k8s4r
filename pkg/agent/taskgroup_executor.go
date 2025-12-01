package agent

import (
	"context"
	"fmt"
	"sync"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	"github.com/hashicorp/go-hclog"
	robotv1alpha1 "github.com/hxndghxndg/k8s4r/api/v1alpha1"
	"github.com/hxndghxndg/k8s4r/pkg/driver"
	"github.com/hxndghxndg/k8s4r/pkg/util"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
)

// TaskGroupExecutor 负责执行单个 TaskGroup
type TaskGroupExecutor struct {
	taskGroup     *robotv1alpha1.TaskGroup
	driverFactory DriverFactory // 工厂，根据类型创建 driver
	logger        hclog.Logger
	baseDir       string
	mqttClient    mqtt.Client
	robotName     string

	initTaskExecutors map[string]*TaskExecutor
	taskExecutors     map[string]*TaskExecutor
	mu                sync.RWMutex

	state         TaskGroupState
	startTime     time.Time
	endTime       time.Time
	message       string
	initTasksDone bool

	// 任务完成计数
	totalTasks      int
	completedTasks  int
	failedTasks     int
	terminatedTasks int

	ctx         context.Context
	cancel      context.CancelFunc
	hookManager *util.HookManager
}

// DriverFactory 驱动工厂接口
type DriverFactory interface {
	CreateDriver(driverType robotv1alpha1.TaskDriverType) (driver.TaskDriver, error)
}

// SimpleDriverFactory 简单的驱动工厂实现
// 注意：同一个 TaskGroup 的所有 Task 共享同一个 driver 实例，以支持 OverlayFS 等共享资源
type SimpleDriverFactory struct {
	logger    hclog.Logger
	baseDir   string
	mu        sync.Mutex
	drivers   map[robotv1alpha1.TaskDriverType]driver.TaskDriver // 缓存 driver 实例
	overlayMu sync.RWMutex                                       // 共享的 overlay 锁
}

// NewSimpleDriverFactory 创建简单驱动工厂
func NewSimpleDriverFactory(logger hclog.Logger) *SimpleDriverFactory {
	return &SimpleDriverFactory{
		logger:  logger,
		baseDir: "", // 使用默认路径
		drivers: make(map[robotv1alpha1.TaskDriverType]driver.TaskDriver),
	}
}

// NewSimpleDriverFactoryWithBaseDir 创建简单驱动工厂（指定baseDir）
func NewSimpleDriverFactoryWithBaseDir(baseDir string, logger hclog.Logger) *SimpleDriverFactory {
	return &SimpleDriverFactory{
		logger:  logger,
		baseDir: baseDir,
		drivers: make(map[robotv1alpha1.TaskDriverType]driver.TaskDriver),
	}
}

// CreateDriver 根据类型创建 driver（单例模式，同一 TaskGroup 共享）
func (f *SimpleDriverFactory) CreateDriver(driverType robotv1alpha1.TaskDriverType) (driver.TaskDriver, error) {
	f.mu.Lock()
	defer f.mu.Unlock()

	// 检查是否已创建（单例模式）
	if d, exists := f.drivers[driverType]; exists {
		return d, nil
	}

	// 首次创建，传递共享的 overlayMu
	var d driver.TaskDriver
	switch driverType {
	case robotv1alpha1.TaskDriverExec:
		d = driver.NewExecDriver(f.baseDir, f.logger, &f.overlayMu)
	case robotv1alpha1.TaskDriverRawExec:
		d = driver.NewRawExecDriver(f.baseDir, f.logger)
	default:
		return nil, fmt.Errorf("unsupported driver type: %s", driverType)
	}

	f.drivers[driverType] = d
	return d, nil
}

type TaskGroupState int

const (
	TaskGroupStateNew TaskGroupState = iota
	TaskGroupStateInitializing
	TaskGroupStateRunning
	TaskGroupStateCompleted
	TaskGroupStateFailed
	TaskGroupStateTerminated
)

type TaskGroupExecutorConfig struct {
	TaskGroup     *robotv1alpha1.TaskGroup
	DriverFactory DriverFactory // 改为工厂
	Logger        hclog.Logger
	BaseDir       string
	MQTTClient    mqtt.Client
	RobotName     string
}

func NewTaskGroupExecutor(config TaskGroupExecutorConfig) (*TaskGroupExecutor, error) {
	if config.TaskGroup == nil {
		return nil, fmt.Errorf("taskGroup is required")
	}
	if config.DriverFactory == nil {
		return nil, fmt.Errorf("driverFactory is required")
	}
	if config.Logger == nil {
		config.Logger = hclog.NewNullLogger()
	}
	if config.BaseDir == "" {
		config.BaseDir = "/tmp/k8s4r/tasks"
	}

	ctx, cancel := context.WithCancel(context.Background())

	// 创建 TaskGroup 级别的 cgroup hook
	hookManager := util.NewHookManager()

	// 不再需要单独存储 cgroupPath

	new_tge := &TaskGroupExecutor{
		taskGroup:         config.TaskGroup,
		driverFactory:     config.DriverFactory,
		logger:            config.Logger.Named("taskgroup-executor"),
		baseDir:           config.BaseDir,
		mqttClient:        config.MQTTClient,
		robotName:         config.RobotName,
		initTaskExecutors: make(map[string]*TaskExecutor),
		taskExecutors:     make(map[string]*TaskExecutor),
		state:             TaskGroupStateNew,
		ctx:               ctx,
		cancel:            cancel,
		hookManager:       hookManager,
	}

	if err := new_tge.setupHooks(); err != nil {
		new_tge.mu.Lock()
		new_tge.state = TaskGroupStateFailed
		new_tge.message = fmt.Sprintf("Failed to setup taskgroup hooks: %v", err)
		new_tge.mu.Unlock()
		return nil, fmt.Errorf("failed to setup taskgroup hooks: %w", err)
	}

	// 初始化并运行 TaskGroup 级别的 hook（可扩展，暂不实现初始化细节）
	if err := new_tge.hookManager.RunPreStart(new_tge.ctx, new_tge.taskGroup); err != nil {
		new_tge.mu.Lock()
		new_tge.state = TaskGroupStateFailed
		new_tge.message = fmt.Sprintf("Failed to run TaskGroup hook PreStart: %v", err)
		new_tge.mu.Unlock()
		return nil, fmt.Errorf("failed to run TaskGroup hook PreStart: %w", err)
	}

	return new_tge, nil
}

func (tge *TaskGroupExecutor) Start() error {

	tge.mu.Lock()
	if tge.state != TaskGroupStateNew {
		tge.mu.Unlock()
		return fmt.Errorf("taskGroup already started")
	}
	tge.state = TaskGroupStateInitializing
	tge.startTime = time.Now()
	tge.mu.Unlock()

	tge.logger.Info("starting TaskGroup", "name", tge.taskGroup.Name, "uid", tge.taskGroup.UID)

	if len(tge.taskGroup.Spec.InitTasks) > 0 {
		if err := tge.executeInitTasks(); err != nil {
			tge.mu.Lock()
			tge.state = TaskGroupStateFailed
			tge.message = fmt.Sprintf("InitTasks failed: %v", err)
			tge.mu.Unlock()
			return err
		}
		tge.initTasksDone = true
	}

	tge.mu.Lock()
	tge.state = TaskGroupStateRunning
	tge.totalTasks = len(tge.taskGroup.Spec.Tasks)
	tge.mu.Unlock()

	if err := tge.executeTasks(); err != nil {
		tge.mu.Lock()
		tge.state = TaskGroupStateFailed
		tge.message = fmt.Sprintf("Tasks failed: %v", err)
		tge.mu.Unlock()
		return err
	}

	tge.logger.Info("TaskGroup started successfully", "name", tge.taskGroup.Name)

	// 运行 TaskGroup hook 的 PostStart
	if err := tge.hookManager.RunPostStart(tge.ctx, tge.taskGroup); err != nil {
		tge.mu.Lock()
		tge.state = TaskGroupStateFailed
		tge.message = fmt.Sprintf("Failed to run TaskGroup hook PostStart: %v", err)
		tge.mu.Unlock()
		return fmt.Errorf("failed to run TaskGroup hook PostStart: %w", err)
	}
	return nil
}

func (tge *TaskGroupExecutor) executeInitTasks() error {
	tge.logger.Info("executing InitTasks", "count", len(tge.taskGroup.Spec.InitTasks))

	for i, initTaskDef := range tge.taskGroup.Spec.InitTasks {
		task := tge.buildTask(initTaskDef, true, i)

		// 根据 TaskDefinition.Driver 创建对应的 driver
		taskDriver, err := tge.driverFactory.CreateDriver(initTaskDef.Driver)
		if err != nil {
			return fmt.Errorf("failed to create driver for InitTask %s: %w", initTaskDef.Name, err)
		}

		te, err := NewTaskExecutor(TaskExecutorConfig{
			Task:       task,
			Driver:     taskDriver, // 使用专属 driver
			Logger:     tge.logger,
			BaseDir:    tge.baseDir,
			MQTTClient: tge.mqttClient,
			RobotName:  tge.robotName,
		})
		if err != nil {
			return fmt.Errorf("failed to create TaskExecutor for InitTask %s: %w", initTaskDef.Name, err)
		}

		// 添加事件监听器，监听 Task 状态变化
		te.AddEventListener(TaskEventListenerFunc(func(event TaskEvent) {
			tge.onTaskEvent(event)
		}))

		tge.mu.Lock()
		tge.initTaskExecutors[string(task.UID)] = te
		tge.mu.Unlock()

		tge.logger.Info("starting InitTask", "name", initTaskDef.Name, "daemon", initTaskDef.Daemon)
		if err := te.Start(tge.ctx); err != nil {
			return fmt.Errorf("failed to start InitTask %s: %w", initTaskDef.Name, err)
		}

		if initTaskDef.Daemon {
			tge.logger.Info("InitTask is daemon, continuing to next", "name", initTaskDef.Name)
			continue
		}

		tge.logger.Info("waiting for InitTask to complete", "name", initTaskDef.Name)
		if err := te.Wait(); err != nil {
			return fmt.Errorf("InitTask %s failed: %w", initTaskDef.Name, err)
		}

		tge.logger.Info("InitTask completed", "name", initTaskDef.Name)
	}

	tge.logger.Info("all InitTasks executed successfully")
	return nil
}

func (tge *TaskGroupExecutor) executeTasks() error {
	tge.logger.Info("executing Tasks", "count", len(tge.taskGroup.Spec.Tasks))

	var wg sync.WaitGroup
	errChan := make(chan error, len(tge.taskGroup.Spec.Tasks))

	for i, taskDef := range tge.taskGroup.Spec.Tasks {
		wg.Add(1)

		go func(td robotv1alpha1.TaskDefinition, idx int) {
			defer wg.Done()

			task := tge.buildTask(td, false, idx)

			// 根据 TaskDefinition.Driver 创建对应的 driver
			taskDriver, err := tge.driverFactory.CreateDriver(td.Driver)
			if err != nil {
				errChan <- fmt.Errorf("failed to create driver for Task %s: %w", td.Name, err)
				return
			}

			te, err := NewTaskExecutor(TaskExecutorConfig{
				Task:       task,
				Driver:     taskDriver, // 使用专属 driver
				Logger:     tge.logger,
				BaseDir:    tge.baseDir,
				MQTTClient: tge.mqttClient,
				RobotName:  tge.robotName,
			})
			if err != nil {
				errChan <- fmt.Errorf("failed to create TaskExecutor for Task %s: %w", td.Name, err)
				return
			}

			// 添加事件监听器
			te.AddEventListener(TaskEventListenerFunc(func(event TaskEvent) {
				tge.onTaskEvent(event)
			}))

			tge.mu.Lock()
			tge.taskExecutors[string(task.UID)] = te
			tge.mu.Unlock()

			tge.logger.Info("starting Task", "name", td.Name)
			if err := te.Start(tge.ctx); err != nil {
				errChan <- fmt.Errorf("failed to start Task %s: %w", td.Name, err)
				return
			}

			tge.logger.Info("Task started", "name", td.Name)
		}(taskDef, i)
	}

	wg.Wait()
	close(errChan)

	for err := range errChan {
		if err != nil {
			return err
		}
	}

	tge.logger.Info("all Tasks started successfully")
	return nil
}

func (tge *TaskGroupExecutor) Wait() error {
	tge.logger.Info("waiting for all Tasks to complete")

	tge.mu.RLock()
	executors := make([]*TaskExecutor, 0, len(tge.taskExecutors))
	for _, te := range tge.taskExecutors {
		executors = append(executors, te)
	}
	tge.mu.RUnlock()

	var wg sync.WaitGroup
	errChan := make(chan error, len(executors))

	for _, te := range executors {
		wg.Add(1)
		go func(executor *TaskExecutor) {
			defer wg.Done()
			if err := executor.Wait(); err != nil {
				errChan <- err
			}
		}(te)
	}

	wg.Wait()
	close(errChan)

	var lastErr error
	for err := range errChan {
		if err != nil {
			lastErr = err
		}
	}

	if lastErr != nil {
		tge.mu.Lock()
		tge.state = TaskGroupStateFailed
		tge.endTime = time.Now()
		tge.message = fmt.Sprintf("Some tasks failed: %v", lastErr)
		tge.mu.Unlock()
		return lastErr
	}

	tge.mu.Lock()
	tge.state = TaskGroupStateCompleted
	tge.endTime = time.Now()
	tge.mu.Unlock()

	tge.logger.Info("all Tasks completed successfully")
	return nil
}

func (tge *TaskGroupExecutor) Stop() error {
	tge.logger.Info("stopping TaskGroup", "name", tge.taskGroup.Name)

	// 运行 TaskGroup hook 的 PreStop
	if err := tge.hookManager.RunPreStop(tge.ctx, tge.taskGroup); err != nil {
		tge.logger.Error("failed to run TaskGroup hook PreStop", "error", err)
	}

	tge.cancel()

	tge.mu.RLock()
	executors := make([]*TaskExecutor, 0, len(tge.taskExecutors))
	for _, te := range tge.taskExecutors {
		executors = append(executors, te)
	}
	tge.mu.RUnlock()

	var wg sync.WaitGroup
	for _, te := range executors {
		wg.Add(1)
		go func(executor *TaskExecutor) {
			defer wg.Done()
			executor.Stop()
		}(te)
	}
	wg.Wait()

	tge.mu.RLock()
	initExecutors := make([]*TaskExecutor, 0, len(tge.initTaskExecutors))
	for _, te := range tge.initTaskExecutors {
		initExecutors = append(initExecutors, te)
	}
	tge.mu.RUnlock()

	for _, te := range initExecutors {
		wg.Add(1)
		go func(executor *TaskExecutor) {
			defer wg.Done()
			executor.Stop()
		}(te)
	}
	wg.Wait()

	// 运行 TaskGroup hook 的 PostStop
	if err := tge.hookManager.RunPostStop(tge.ctx, tge.taskGroup); err != nil {
		tge.logger.Error("failed to run TaskGroup hook PostStop", "error", err)
	}

	tge.mu.Lock()
	tge.state = TaskGroupStateTerminated
	tge.endTime = time.Now()
	tge.mu.Unlock()

	tge.logger.Info("TaskGroup stopped")
	return nil
}

func (tge *TaskGroupExecutor) Kill() error {
	tge.logger.Info("killing TaskGroup", "name", tge.taskGroup.Name)

	// 运行 TaskGroup hook 的 PreStop
	if err := tge.hookManager.RunPreStop(tge.ctx, tge.taskGroup); err != nil {
		tge.logger.Error("failed to run TaskGroup hook PreStop", "error", err)
	}

	tge.cancel()

	tge.mu.RLock()
	allExecutors := make([]*TaskExecutor, 0, len(tge.taskExecutors)+len(tge.initTaskExecutors))
	for _, te := range tge.taskExecutors {
		allExecutors = append(allExecutors, te)
	}
	for _, te := range tge.initTaskExecutors {
		allExecutors = append(allExecutors, te)
	}
	tge.mu.RUnlock()

	var wg sync.WaitGroup
	for _, te := range allExecutors {
		wg.Add(1)
		go func(executor *TaskExecutor) {
			defer wg.Done()
			executor.Kill()
		}(te)
	}
	wg.Wait()

	// 运行 TaskGroup hook 的 PostStop
	if err := tge.hookManager.RunPostStop(tge.ctx, tge.taskGroup); err != nil {
		tge.logger.Error("failed to run TaskGroup hook PostStop", "error", err)
	}

	tge.mu.Lock()
	tge.state = TaskGroupStateTerminated
	tge.endTime = time.Now()
	tge.message = "TaskGroup killed by user"
	tge.mu.Unlock()

	tge.logger.Info("TaskGroup killed")
	return nil
}

func (tge *TaskGroupExecutor) GetState() TaskGroupState {
	tge.mu.RLock()
	defer tge.mu.RUnlock()
	return tge.state
}

// GetProgress 获取任务组的执行进度
func (tge *TaskGroupExecutor) GetProgress() (completed, failed, terminated, total int, progress int) {
	tge.mu.RLock()
	defer tge.mu.RUnlock()

	completed = tge.completedTasks
	failed = tge.failedTasks
	terminated = tge.terminatedTasks
	total = tge.totalTasks

	if total > 0 {
		finishedTasks := completed + failed + terminated
		progress = (finishedTasks * 100) / total
	}

	return
}

// GetMessage 获取当前状态消息
func (tge *TaskGroupExecutor) GetMessage() string {
	tge.mu.RLock()
	defer tge.mu.RUnlock()
	return tge.message
}

// onTaskEvent 处理 Task 事件，聚合到 TaskGroup 状态
func (tge *TaskGroupExecutor) onTaskEvent(event TaskEvent) {
	tge.logger.Info("received task event",
		"type", event.Type,
		"taskUID", event.TaskUID,
		"state", event.State,
		"message", event.Message)

	// 只处理状态变化事件
	if event.Type != EventTypeStateChanged {
		return
	}

	tge.mu.Lock()
	defer tge.mu.Unlock()

	// 统计任务状态
	switch event.State {
	case TaskStateCompleted:
		tge.completedTasks++
		tge.logger.Info("task completed",
			"taskUID", event.TaskUID,
			"completed", tge.completedTasks,
			"total", tge.totalTasks)

	case TaskStateFailed:
		tge.failedTasks++
		tge.logger.Warn("task failed",
			"taskUID", event.TaskUID,
			"failed", tge.failedTasks,
			"message", event.Message)

	case TaskStateTerminated:
		tge.terminatedTasks++
		tge.logger.Info("task terminated",
			"taskUID", event.TaskUID,
			"terminated", tge.terminatedTasks)
	}

	// 检查是否所有任务都已完成
	finishedTasks := tge.completedTasks + tge.failedTasks + tge.terminatedTasks
	if finishedTasks >= tge.totalTasks {
		// 运行 TaskGroup hook 的 PreStop（任务完成时的清理）
		if err := tge.hookManager.RunPreStop(context.Background(), tge.taskGroup); err != nil {
			tge.logger.Error("failed to run TaskGroup hook PreStop on completion", "error", err)
		}

		if tge.failedTasks > 0 {
			tge.state = TaskGroupStateFailed
			tge.message = fmt.Sprintf("%d/%d tasks failed", tge.failedTasks, tge.totalTasks)
			tge.endTime = time.Now()
			tge.logger.Error("TaskGroup failed",
				"completed", tge.completedTasks,
				"failed", tge.failedTasks,
				"terminated", tge.terminatedTasks)
		} else if tge.terminatedTasks > 0 && tge.completedTasks < tge.totalTasks {
			tge.state = TaskGroupStateTerminated
			tge.message = fmt.Sprintf("%d/%d tasks terminated", tge.terminatedTasks, tge.totalTasks)
			tge.endTime = time.Now()
			tge.logger.Info("TaskGroup terminated",
				"completed", tge.completedTasks,
				"terminated", tge.terminatedTasks)
		} else {
			tge.state = TaskGroupStateCompleted
			tge.message = fmt.Sprintf("All %d tasks completed successfully", tge.totalTasks)
			tge.endTime = time.Now()
			tge.logger.Info("TaskGroup completed successfully",
				"completed", tge.completedTasks,
				"total", tge.totalTasks)
		}

		// 运行 TaskGroup hook 的 PostStop（任务完成后的最终清理）
		if err := tge.hookManager.RunPostStop(context.Background(), tge.taskGroup); err != nil {
			tge.logger.Error("failed to run TaskGroup hook PostStop on completion", "error", err)
		}

		// 发送 TaskGroup 状态更新 MQTT 消息
		tge.publishTaskGroupStatus()
	} else {
		// 发送进度更新
		tge.publishTaskGroupProgress()
	}
}

// publishTaskGroupStatus 发送 TaskGroup 状态到 MQTT
func (tge *TaskGroupExecutor) publishTaskGroupStatus() {
	if tge.mqttClient == nil || !tge.mqttClient.IsConnected() {
		return
	}

	topic := fmt.Sprintf("robot/%s/taskgroup/%s/status", tge.robotName, tge.taskGroup.Name)
	payload := fmt.Sprintf(`{"state":"%s","message":"%s","completed":%d,"failed":%d,"terminated":%d,"total":%d,"startTime":"%s","endTime":"%s"}`,
		tge.getStateName(),
		tge.message,
		tge.completedTasks,
		tge.failedTasks,
		tge.terminatedTasks,
		tge.totalTasks,
		tge.startTime.Format(time.RFC3339),
		tge.endTime.Format(time.RFC3339))

	token := tge.mqttClient.Publish(topic, 1, false, payload)
	token.Wait()

	tge.logger.Info("published TaskGroup status", "topic", topic)
}

// publishTaskGroupProgress 发送 TaskGroup 进度到 MQTT
func (tge *TaskGroupExecutor) publishTaskGroupProgress() {
	if tge.mqttClient == nil || !tge.mqttClient.IsConnected() {
		return
	}

	finishedTasks := tge.completedTasks + tge.failedTasks + tge.terminatedTasks
	progress := 0
	if tge.totalTasks > 0 {
		progress = (finishedTasks * 100) / tge.totalTasks
	}

	topic := fmt.Sprintf("robot/%s/taskgroup/%s/progress", tge.robotName, tge.taskGroup.Name)
	payload := fmt.Sprintf(`{"progress":%d,"completed":%d,"failed":%d,"terminated":%d,"total":%d}`,
		progress,
		tge.completedTasks,
		tge.failedTasks,
		tge.terminatedTasks,
		tge.totalTasks)

	token := tge.mqttClient.Publish(topic, 1, false, payload)
	token.Wait()

	tge.logger.Debug("published TaskGroup progress", "topic", topic, "progress", progress)
}

// getStateName 返回状态名称
func (tge *TaskGroupExecutor) getStateName() string {
	switch tge.state {
	case TaskGroupStateNew:
		return "new"
	case TaskGroupStateInitializing:
		return "initializing"
	case TaskGroupStateRunning:
		return "running"
	case TaskGroupStateCompleted:
		return "completed"
	case TaskGroupStateFailed:
		return "failed"
	case TaskGroupStateTerminated:
		return "terminated"
	default:
		return "unknown"
	}
}

func (tge *TaskGroupExecutor) buildTask(td robotv1alpha1.TaskDefinition, isInitTask bool, index int) *robotv1alpha1.Task {
	uidStr := fmt.Sprintf("%s-%s-%d", tge.taskGroup.UID, td.Name, time.Now().UnixNano())

	taskName := td.Name
	if isInitTask {
		taskName = fmt.Sprintf("init-%s", td.Name)
	}

	return &robotv1alpha1.Task{
		ObjectMeta: metav1.ObjectMeta{
			Name: taskName,
			UID:  types.UID(uidStr),
		},
		Spec: robotv1alpha1.TaskSpec{
			Name:          td.Name,
			TaskGroupName: tge.taskGroup.Name,
			Config:        td.Config,
		},
	}
}

// TaskGroupExecutor 负责执行单个 TaskGroup
type TaskGroupCgroupHook struct {
	taskGroup     *robotv1alpha1.TaskGroup
	driverFactory DriverFactory // 工厂，根据类型创建 driver
	logger        hclog.Logger
	baseDir       string
	mqttClient    mqtt.Client
	robotName     string

	initTaskExecutors map[string]*TaskExecutor
	taskExecutors     map[string]*TaskExecutor
	mu                sync.RWMutex

	state         TaskGroupState
	startTime     time.Time
	endTime       time.Time
	message       string
	initTasksDone bool

	// 任务完成计数
	totalTasks      int
	completedTasks  int
	failedTasks     int
	terminatedTasks int

	ctx         context.Context
	cancel      context.CancelFunc
	hookManager *util.HookManager
}

// setupHooks 初始化 TaskGroup 级别的 hooks
func (tge *TaskGroupExecutor) setupHooks() error {
	tge.logger.Info("setting up TaskGroup hooks",
		"taskgroup", tge.taskGroup.Name)
	cgroupHook := util.NewCgroupHook(tge.logger, "/sys/fs/cgroup/nomad.slice/share.slice")
	storageHook := util.NewSharedStorageHook(tge.logger, tge.baseDir)
	tge.hookManager.AddHooks(cgroupHook, storageHook)
	return nil
}
