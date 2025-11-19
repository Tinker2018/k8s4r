package agent

import (
	"context"
	"encoding/json"
	"fmt"
	"sync"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	robotv1alpha1 "github.com/hxndg/k8s4r/api/v1alpha1"
	"github.com/hxndg/k8s4r/pkg/driver"
)

// TaskExecutor 负责在 Agent 端执行任务
type TaskExecutor struct {
	robotName  string
	mqttClient mqtt.Client
	driver     driver.TaskDriver
	tasks      map[string]*runningTask
	tasksMu    sync.RWMutex
	logger     Logger
}

// runningTask 代表一个运行中的任务
type runningTask struct {
	task       *robotv1alpha1.Task
	handle     *driver.TaskHandle
	cancelFunc context.CancelFunc
	stopChan   chan struct{}
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
func NewTaskExecutor(robotName string, mqttClient mqtt.Client, logger Logger) *TaskExecutor {
	if logger == nil {
		logger = &defaultLogger{}
	}

	return &TaskExecutor{
		robotName:  robotName,
		mqttClient: mqttClient,
		driver:     driver.NewExecDriver(logger),
		tasks:      make(map[string]*runningTask),
		logger:     logger,
	}
}

// Start 启动任务执行器
func (te *TaskExecutor) Start(ctx context.Context) error {
	// 订阅任务分发 topic
	topic := fmt.Sprintf("robots/%s/tasks/dispatch", te.robotName)

	te.logger.Info("subscribing to task dispatch topic", "topic", topic)

	token := te.mqttClient.Subscribe(topic, 1, func(client mqtt.Client, msg mqtt.Message) {
		te.handleTaskMessage(ctx, msg)
	})

	token.Wait()
	if err := token.Error(); err != nil {
		return fmt.Errorf("failed to subscribe to task topic: %v", err)
	}

	te.logger.Info("task executor started successfully")
	return nil
}

// handleTaskMessage 处理接收到的任务消息
func (te *TaskExecutor) handleTaskMessage(ctx context.Context, msg mqtt.Message) {
	te.logger.Info("received task message", "topic", msg.Topic())

	var taskMsg TaskMessage
	if err := json.Unmarshal(msg.Payload(), &taskMsg); err != nil {
		te.logger.Error("failed to unmarshal task message", "error", err)
		return
	}

	switch taskMsg.Action {
	case "create":
		te.createTask(ctx, taskMsg.Task)
	case "delete":
		te.deleteTask(ctx, taskMsg.Task)
	default:
		te.logger.Error("unknown task action", "action", taskMsg.Action)
	}
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

	// 创建任务上下文
	taskCtx, cancel := context.WithCancel(ctx)

	// 启动任务
	handle, err := te.driver.Start(taskCtx, task)
	if err != nil {
		cancel()
		te.logger.Error("failed to start task", "taskUID", taskUID, "error", err)
		te.reportStatus(taskUID, "dead", -1, fmt.Sprintf("Failed to start: %v", err), nil)
		return
	}

	// 保存运行中的任务
	rt := &runningTask{
		task:       task,
		handle:     handle,
		cancelFunc: cancel,
		stopChan:   make(chan struct{}),
	}

	te.tasksMu.Lock()
	te.tasks[taskUID] = rt
	te.tasksMu.Unlock()

	te.logger.Info("task started successfully", "taskUID", taskUID, "pid", handle.PID)

	// 启动状态监控协程
	go te.monitorTask(taskCtx, rt)
}

// monitorTask 监控任务状态并周期性上报
func (te *TaskExecutor) monitorTask(ctx context.Context, rt *runningTask) {
	taskUID := string(rt.task.UID)
	ticker := time.NewTicker(5 * time.Second)
	defer ticker.Stop()

	te.logger.Info("starting task monitor", "taskUID", taskUID)

	for {
		select {
		case <-ctx.Done():
			te.logger.Info("task monitor stopped by context", "taskUID", taskUID)
			return

		case <-rt.stopChan:
			te.logger.Info("task monitor stopped", "taskUID", taskUID)
			return

		case <-ticker.C:
			// 获取任务状态
			status, err := te.driver.Status(ctx, rt.handle)
			if err != nil {
				te.logger.Error("failed to get task status", "taskUID", taskUID, "error", err)
				continue
			}

			// 上报状态
			exitCode := 0
			if status.ExitCode != 0 {
				exitCode = status.ExitCode
			}

			te.reportStatus(taskUID, string(status.State), exitCode, status.Message, status.Resources)

			// 如果任务已结束，停止监控
			if status.State == driver.TaskStateExited || status.State == driver.TaskStateFailed {
				te.logger.Info("task finished", "taskUID", taskUID, "state", status.State, "exitCode", exitCode)

				// 清理任务
				te.tasksMu.Lock()
				delete(te.tasks, taskUID)
				te.tasksMu.Unlock()

				return
			}
		}
	}
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
	statusMsg := TaskStatusMessage{
		TaskUID:   taskUID,
		RobotName: te.robotName,
		State:     state,
		ExitCode:  exitCode,
		Message:   message,
		Resources: resources,
		UpdatedAt: time.Now(),
	}

	payload, err := json.Marshal(statusMsg)
	if err != nil {
		te.logger.Error("failed to marshal status message", "error", err)
		return
	}

	// 发送到状态上报 topic
	topic := fmt.Sprintf("robots/%s/tasks/%s/status", te.robotName, taskUID)

	token := te.mqttClient.Publish(topic, 1, false, payload)
	token.Wait()
	if err := token.Error(); err != nil {
		te.logger.Error("failed to publish status", "error", err, "topic", topic)
	} else {
		te.logger.Debug("status reported", "taskUID", taskUID, "state", state)
	}
}

// Stop 停止任务执行器
func (te *TaskExecutor) Stop(ctx context.Context) error {
	te.logger.Info("stopping task executor")

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
