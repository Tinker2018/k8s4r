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
)

// TaskGroupManager 管理所有 TaskGroup 的执行
// 它负责创建和销毁 TaskGroupExecutor 实例
// 每个 TaskGroup 对应一个独立的 TaskGroupExecutor
type TaskGroupManager struct {
	robotName    string
	mqttClient   mqtt.Client
	taskGroups   map[string]*TaskGroupExecutor // key: TaskGroupUID
	taskGroupsMu sync.RWMutex
	logger       hclog.Logger
	workDir      string
	monitorStop  chan struct{}
	monitorOnce  sync.Once
}

// TaskGroupMessage 从 Manager 接收的 TaskGroup 消息
type TaskGroupMessage struct {
	Action    string                   `json:"action"` // create, delete
	TaskGroup *robotv1alpha1.TaskGroup `json:"taskGroup"`
}

// TaskGroupStatusMessage 上报 TaskGroup 整体状态给 Manager
type TaskGroupStatusMessage struct {
	TaskGroupUID string            `json:"taskGroupUid"`
	RobotName    string            `json:"robotName"`
	State        string            `json:"state"` // pending, running, completed, failed
	Message      string            `json:"message"`
	TaskStates   map[string]string `json:"taskStates"` // taskName -> state
	UpdatedAt    time.Time         `json:"updatedAt"`
}

// NewTaskGroupManager 创建 TaskGroup 管理器
func NewTaskGroupManager(robotName string, mqttClient mqtt.Client, workDir string, logger hclog.Logger) *TaskGroupManager {
	if logger == nil {
		logger = hclog.NewNullLogger()
	}

	return &TaskGroupManager{
		robotName:   robotName,
		mqttClient:  mqttClient,
		taskGroups:  make(map[string]*TaskGroupExecutor),
		logger:      logger.Named("taskgroup-manager"),
		workDir:     workDir,
		monitorStop: make(chan struct{}),
	}
}

// Start 启动 TaskGroup 管理器
func (tgm *TaskGroupManager) Start(ctx context.Context) error {
	tgm.logger.Info("starting taskgroup manager", "robot", tgm.robotName)

	// 1. 订阅 TaskGroup 消息 topic
	taskGroupTopic := fmt.Sprintf("robot/%s/taskgroup", tgm.robotName)
	token := tgm.mqttClient.Subscribe(taskGroupTopic, 1, func(client mqtt.Client, msg mqtt.Message) {
		tgm.handleTaskGroupMessage(ctx, msg)
	})

	if token.Wait() && token.Error() != nil {
		return fmt.Errorf("failed to subscribe to taskgroup topic: %w", token.Error())
	}

	tgm.logger.Info("subscribed to taskgroup topic", "topic", taskGroupTopic)

	// 2. 订阅状态恢复 topic（带 retained 标志）
	stateTopic := fmt.Sprintf("robot/%s/state", tgm.robotName)
	done := make(chan bool, 1)

	token = tgm.mqttClient.Subscribe(stateTopic, 1, func(client mqtt.Client, msg mqtt.Message) {
		tgm.handleStateRecovery(ctx, msg, done)
	})

	if token.Wait() && token.Error() != nil {
		return fmt.Errorf("failed to subscribe to state topic: %w", token.Error())
	}

	tgm.logger.Info("subscribed to state topic", "topic", stateTopic)

	// 等待状态恢复完成（超时 5 秒）
	select {
	case <-done:
		tgm.logger.Info("state recovery completed or empty")
	case <-time.After(5 * time.Second):
		tgm.logger.Info("state recovery timeout, continuing")
	}

	// 3. 启动监控协程
	tgm.startMonitor(ctx)

	tgm.logger.Info("taskgroup manager started successfully")
	return nil
}

// handleTaskGroupMessage 处理接收到的 TaskGroup 消息
func (tgm *TaskGroupManager) handleTaskGroupMessage(ctx context.Context, msg mqtt.Message) {
	tgm.logger.Info("received taskgroup message", "topic", msg.Topic(), "size", len(msg.Payload()))

	var taskGroupMsg TaskGroupMessage
	if err := json.Unmarshal(msg.Payload(), &taskGroupMsg); err != nil {
		tgm.logger.Error("failed to unmarshal taskgroup message", "error", err)
		return
	}

	switch taskGroupMsg.Action {
	case "create":
		tgm.createTaskGroup(ctx, taskGroupMsg.TaskGroup)
	case "delete":
		tgm.deleteTaskGroup(ctx, taskGroupMsg.TaskGroup)
	default:
		tgm.logger.Error("unknown action", "action", taskGroupMsg.Action)
	}
}

// handleStateRecovery 处理状态恢复消息
func (tgm *TaskGroupManager) handleStateRecovery(ctx context.Context, msg mqtt.Message, done chan bool) {
	defer func() {
		select {
		case done <- true:
		default:
		}
	}()

	tgm.logger.Info("handling state recovery", "payloadSize", len(msg.Payload()), "retained", msg.Retained())

	if len(msg.Payload()) == 0 {
		tgm.logger.Info("no taskgroups to recover - empty payload")
		return
	}

	var state struct {
		TaskGroups []*robotv1alpha1.TaskGroup `json:"taskGroups"`
	}

	if err := json.Unmarshal(msg.Payload(), &state); err != nil {
		tgm.logger.Error("failed to unmarshal state message", "error", err)
		return
	}

	tgm.logger.Info("recovering taskgroups from state", "count", len(state.TaskGroups))

	for _, taskGroup := range state.TaskGroups {
		tgm.logger.Info("recovering taskgroup", "uid", taskGroup.UID, "name", taskGroup.Name)
		tgm.createTaskGroup(ctx, taskGroup)
	}

	tgm.logger.Info("state recovery finished", "recoveredCount", len(state.TaskGroups))
}

// createTaskGroup 创建并启动 TaskGroup
func (tgm *TaskGroupManager) createTaskGroup(ctx context.Context, taskGroup *robotv1alpha1.TaskGroup) {
	taskGroupUID := string(taskGroup.UID)

	tgm.logger.Info("========================================")
	tgm.logger.Info("📥 Received TaskGroup",
		"uid", taskGroupUID,
		"name", taskGroup.Name,
		"initTasksCount", len(taskGroup.Spec.InitTasks),
		"tasksCount", len(taskGroup.Spec.Tasks))
	tgm.logger.Info("========================================")

	// 检查 TaskGroup 是否已存在
	tgm.taskGroupsMu.Lock()
	if _, exists := tgm.taskGroups[taskGroupUID]; exists {
		tgm.taskGroupsMu.Unlock()
		tgm.logger.Error("taskgroup already exists", "uid", taskGroupUID)
		return
	}
	tgm.taskGroupsMu.Unlock()

	// 上报 TaskGroup 已分配
	tgm.reportTaskGroupStatus(taskGroupUID, "pending", "TaskGroup assigned to robot", nil)

	taskGroupExecutorConfig := TaskGroupExecutorConfig{
		TaskGroup:     taskGroup,
		DriverFactory: NewSimpleDriverFactory(tgm.logger),
		Logger:        tgm.logger,
		BaseDir:       tgm.workDir,
		MQTTClient:    tgm.mqttClient,
		RobotName:     tgm.robotName,
	}

	// 创建该 TaskGroup 的专属执行器
	tge, err := NewTaskGroupExecutor(taskGroupExecutorConfig)
	if err != nil {
		tgm.logger.Error("failed to create taskgroup executor", "error", err)
	}

	// 保存到 map
	tgm.taskGroupsMu.Lock()
	tgm.taskGroups[taskGroupUID] = tge
	tgm.taskGroupsMu.Unlock()

	// 异步执行 TaskGroup（避免阻塞 MQTT 消息处理）
	go func() {
		defer func() {
			tgm.taskGroupsMu.Lock()
			delete(tgm.taskGroups, taskGroupUID)
			tgm.taskGroupsMu.Unlock()
		}()

		tgm.reportTaskGroupStatus(taskGroupUID, "running", "TaskGroup executing", nil)

		// 执行 TaskGroup
		err := tge.Start()
		tge.Wait()

		// 上报最终状态
		if err != nil {
			tgm.logger.Error("TaskGroup execution failed", "error", err)
			tgm.reportTaskGroupStatus(taskGroupUID, "failed", fmt.Sprintf("Execution failed: %v", err), nil)
		} else {
			tgm.reportTaskGroupStatus(taskGroupUID, "completed", "All tasks completed successfully", nil)
		}
	}()
}

// deleteTaskGroup 删除 TaskGroup
func (tgm *TaskGroupManager) deleteTaskGroup(ctx context.Context, taskGroup *robotv1alpha1.TaskGroup) {
	taskGroupUID := string(taskGroup.UID)

	tgm.logger.Info("deleting taskgroup", "uid", taskGroupUID, "name", taskGroup.Name)

	tgm.taskGroupsMu.RLock()
	_, exists := tgm.taskGroups[taskGroupUID]
	tgm.taskGroupsMu.RUnlock()

	if !exists {
		tgm.logger.Info("taskgroup not found, skipping", "uid", taskGroupUID)
		return
	}

	// 从 map 中移除
	tgm.taskGroupsMu.Lock()
	delete(tgm.taskGroups, taskGroupUID)
	tgm.taskGroupsMu.Unlock()
}

// reportTaskGroupStatus 上报 TaskGroup 状态
func (tgm *TaskGroupManager) reportTaskGroupStatus(taskGroupUID, state, message string, taskStates map[string]string) {
	statusMsg := TaskGroupStatusMessage{
		TaskGroupUID: taskGroupUID,
		RobotName:    tgm.robotName,
		State:        state,
		Message:      message,
		TaskStates:   taskStates,
		UpdatedAt:    time.Now(),
	}

	payload, err := json.Marshal(statusMsg)
	if err != nil {
		tgm.logger.Error("failed to marshal taskgroup status", "error", err)
		return
	}

	topic := fmt.Sprintf("robot/%s/taskgroup/status", tgm.robotName)
	token := tgm.mqttClient.Publish(topic, 1, false, payload)
	token.Wait()

	if token.Error() != nil {
		tgm.logger.Error("failed to publish taskgroup status", "error", token.Error())
	}
}

// startMonitor 启动监控协程
func (tgm *TaskGroupManager) startMonitor(ctx context.Context) {
	tgm.monitorOnce.Do(func() {
		tgm.logger.Info("starting taskgroup monitor")

		go func() {
			ticker := time.NewTicker(10 * time.Second)
			defer ticker.Stop()

			for {
				select {
				case <-ctx.Done():
					tgm.logger.Info("taskgroup monitor stopped")
					return
				case <-tgm.monitorStop:
					tgm.logger.Info("taskgroup monitor stopped")
					return
				case <-ticker.C:
					// 定期检查 TaskGroup 状态
					tgm.taskGroupsMu.RLock()
					count := len(tgm.taskGroups)
					tgm.taskGroupsMu.RUnlock()

					if count > 0 {
						tgm.logger.Debug("taskgroup monitor heartbeat", "runningTaskGroups", count)
					}
				}
			}
		}()
	})
}

// Stop 停止 TaskGroup 管理器
func (tgm *TaskGroupManager) Stop(ctx context.Context) error {
	tgm.logger.Info("stopping taskgroup manager")

	// 停止监控协程
	close(tgm.monitorStop)

	// 清理所有运行中的 TaskGroups
	tgm.taskGroupsMu.Lock()
	taskGroups := make(map[string]*TaskGroupExecutor)
	for uid, tge := range tgm.taskGroups {
		taskGroups[uid] = tge
	}
	tgm.taskGroupsMu.Unlock()

	for _, tge := range taskGroups {
		tge.Stop()
	}

	tgm.logger.Info("taskgroup manager stopped")
	return nil
}
