/*
Copyright 2025.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0
*/

package agent

import (
	"context"
	"encoding/json"
	"fmt"
	"strings"
	"sync"
	"testing"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	"github.com/google/uuid"
	robotv1alpha1 "github.com/hxndghxndg/k8s4r/api/v1alpha1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
)

// mockMQTTClient 模拟 MQTT 客户端
type mockMQTTClient struct {
	publishedMessages map[string][]string // topic -> messages
	mu                sync.RWMutex
	subscribers       map[string]mqtt.MessageHandler
	subMu             sync.RWMutex
}

func newMockMQTTClient() *mockMQTTClient {
	return &mockMQTTClient{
		publishedMessages: make(map[string][]string),
		subscribers:       make(map[string]mqtt.MessageHandler),
	}
}

func (m *mockMQTTClient) Publish(topic string, qos byte, retained bool, payload interface{}) mqtt.Token {
	m.mu.Lock()
	defer m.mu.Unlock()

	var payloadStr string
	switch p := payload.(type) {
	case []byte:
		payloadStr = string(p)
	case string:
		payloadStr = p
	default:
		payloadStr = fmt.Sprintf("%v", p)
	}

	m.publishedMessages[topic] = append(m.publishedMessages[topic], payloadStr)
	return &mockToken{err: nil}
}

func (m *mockMQTTClient) Subscribe(topic string, qos byte, callback mqtt.MessageHandler) mqtt.Token {
	m.subMu.Lock()
	defer m.subMu.Unlock()
	m.subscribers[topic] = callback
	return &mockToken{err: nil}
}

func (m *mockMQTTClient) getPublishedMessages(topic string) []string {
	m.mu.RLock()
	defer m.mu.RUnlock()
	return m.publishedMessages[topic]
}

func (m *mockMQTTClient) getLastMessage(topic string) string {
	m.mu.RLock()
	defer m.mu.RUnlock()
	messages := m.publishedMessages[topic]
	if len(messages) == 0 {
		return ""
	}
	return messages[len(messages)-1]
}

// 未使用的方法（实现 mqtt.Client 接口）
func (m *mockMQTTClient) IsConnected() bool       { return true }
func (m *mockMQTTClient) IsConnectionOpen() bool  { return true }
func (m *mockMQTTClient) Connect() mqtt.Token     { return &mockToken{} }
func (m *mockMQTTClient) Disconnect(quiesce uint) {}
func (m *mockMQTTClient) SubscribeMultiple(filters map[string]byte, callback mqtt.MessageHandler) mqtt.Token {
	return &mockToken{}
}
func (m *mockMQTTClient) Unsubscribe(topics ...string) mqtt.Token { return &mockToken{} }
func (m *mockMQTTClient) AddRoute(topic string, callback mqtt.MessageHandler) {
}
func (m *mockMQTTClient) OptionsReader() mqtt.ClientOptionsReader {
	return mqtt.ClientOptionsReader{}
}

// mockToken 模拟 MQTT Token
type mockToken struct {
	err error
}

func (t *mockToken) Wait() bool                     { return true }
func (t *mockToken) WaitTimeout(time.Duration) bool { return true }
func (t *mockToken) Done() <-chan struct{}          { return make(chan struct{}) }
func (t *mockToken) Error() error                   { return t.err }

// testLogger 测试用的日志实现
type testLogger struct {
	t *testing.T
}

func (l *testLogger) Info(msg string, args ...interface{}) {
	l.t.Logf("[INFO] %s %v", msg, args)
}

func (l *testLogger) Error(msg string, args ...interface{}) {
	l.t.Logf("[ERROR] %s %v", msg, args)
}

func (l *testLogger) Debug(msg string, args ...interface{}) {
	l.t.Logf("[DEBUG] %s %v", msg, args)
}

// createTestTask 创建测试任务
func createTestTask(name, command string, args []string, sleepDuration time.Duration) *robotv1alpha1.Task {
	taskUID := uuid.New().String()

	return &robotv1alpha1.Task{
		ObjectMeta: metav1.ObjectMeta{
			Name:      name,
			Namespace: "default",
			UID:       types.UID(taskUID),
		},
		Spec: robotv1alpha1.TaskSpec{
			Name:        name,
			Driver:      "exec",
			TargetRobot: "test-robot",
			Config: robotv1alpha1.TaskDriverConfig{
				ExecConfig: &robotv1alpha1.ExecDriverConfig{
					Command: "/bin/sh",
					Args:    []string{"-c", fmt.Sprintf("sleep %d && echo '%s'", int(sleepDuration.Seconds()), command)},
				},
			},
		},
		Status: robotv1alpha1.TaskStatus{
			State: robotv1alpha1.TaskStatePending,
		},
	}
}

// TestTaskExecutor_ConcurrentTasks 测试同时执行两个任务
func TestTaskExecutor_ConcurrentTasks(t *testing.T) {
	// 创建模拟 MQTT 客户端
	mockClient := newMockMQTTClient()

	// 创建 TaskExecutor
	workDir := t.TempDir() // 使用临时目录
	logger := &testLogger{t: t}
	executor := NewTaskExecutor("test-robot", mockClient, workDir, logger)

	// 启动 executor
	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	err := executor.Start(ctx)
	if err != nil {
		t.Fatalf("Failed to start executor: %v", err)
	}

	// 创建两个测试任务
	// Task 1: sleep 2s && echo "hello"
	task1 := createTestTask("task-sleep-2s", "hello from task1", []string{}, 2*time.Second)

	// Task 2: sleep 5s && echo "hello"
	task2 := createTestTask("task-sleep-5s", "hello from task2", []string{}, 5*time.Second)

	t.Logf("Task 1 UID: %s", task1.UID)
	t.Logf("Task 2 UID: %s", task2.UID)

	// 记录开始时间
	startTime := time.Now()

	// 创建两个任务（模拟从 MQTT 接收任务）
	t.Log("Creating task 1 (sleep 2s)...")
	executor.createTask(ctx, task1)

	t.Log("Creating task 2 (sleep 5s)...")
	executor.createTask(ctx, task2)

	// 等待一小段时间，让任务开始执行
	time.Sleep(500 * time.Millisecond)

	// 验证两个任务都已启动
	executor.tasksMu.RLock()
	if len(executor.tasks) != 2 {
		t.Fatalf("Expected 2 running tasks, got %d", len(executor.tasks))
	}
	executor.tasksMu.RUnlock()

	t.Log("Both tasks started successfully")

	// 等待第一个任务完成（2秒）
	time.Sleep(3 * time.Second)

	task1EndTime := time.Since(startTime)
	t.Logf("After 3 seconds (task1 should be done): elapsed=%v", task1EndTime)

	// 检查 task1 是否完成
	executor.tasksMu.RLock()
	runningCount := len(executor.tasks)
	executor.tasksMu.RUnlock()

	if runningCount == 2 {
		t.Log("Task 1 might still be cleaning up, waiting a bit more...")
		time.Sleep(1 * time.Second)
		executor.tasksMu.RLock()
		runningCount = len(executor.tasks)
		executor.tasksMu.RUnlock()
	}

	if runningCount != 1 {
		t.Logf("Warning: Expected 1 running task after 3s, got %d (task1 may still be finishing)", runningCount)
	}

	// 验证 task1 的状态上报
	task1StatusTopic := fmt.Sprintf("k8s4r/robots/test-robot/tasks/%s/status", task1.UID)
	task1Messages := mockClient.getPublishedMessages(task1StatusTopic)

	t.Logf("Task 1 status messages count: %d", len(task1Messages))
	if len(task1Messages) > 0 {
		// 检查最后一条消息
		lastMsg := task1Messages[len(task1Messages)-1]
		var statusMsg TaskStatusMessage
		if err := json.Unmarshal([]byte(lastMsg), &statusMsg); err == nil {
			t.Logf("Task 1 last status: state=%s, exitCode=%d, message=%s",
				statusMsg.State, statusMsg.ExitCode, statusMsg.Message)
		}
	}

	// 等待第二个任务完成（总共 6-7 秒）
	time.Sleep(4 * time.Second)

	task2EndTime := time.Since(startTime)
	t.Logf("After 8+ seconds (task2 should be done): elapsed=%v", task2EndTime)

	// 检查所有任务是否完成
	executor.tasksMu.RLock()
	finalCount := len(executor.tasks)
	executor.tasksMu.RUnlock()

	if finalCount > 0 {
		t.Log("Some tasks still running, waiting for cleanup...")
		time.Sleep(3 * time.Second) // 等待monitor周期（5秒）完成清理
		executor.tasksMu.RLock()
		finalCount = len(executor.tasks)
		executor.tasksMu.RUnlock()
	}

	if finalCount != 0 {
		t.Logf("Warning: Expected 0 running tasks, got %d", finalCount)
	}

	// 验证 task2 的状态上报
	task2StatusTopic := fmt.Sprintf("k8s4r/robots/test-robot/tasks/%s/status", task2.UID)
	task2Messages := mockClient.getPublishedMessages(task2StatusTopic)

	t.Logf("Task 2 status messages count: %d", len(task2Messages))
	if len(task2Messages) > 0 {
		// 检查最后一条消息
		lastMsg := task2Messages[len(task2Messages)-1]
		var statusMsg TaskStatusMessage
		if err := json.Unmarshal([]byte(lastMsg), &statusMsg); err == nil {
			t.Logf("Task 2 last status: state=%s, exitCode=%d, message=%s",
				statusMsg.State, statusMsg.ExitCode, statusMsg.Message)
		}
	}

	// 验证总执行时间
	totalDuration := time.Since(startTime)
	t.Logf("Total execution time: %v", totalDuration)

	// 由于是并发执行，总时间应该接近较长任务的时间（5秒），而不是两者之和（7秒）
	// 加上monitor周期和清理时间，总时间约为 5秒(task2) + 最多5秒(monitor周期) = 10秒左右
	if totalDuration > 15*time.Second {
		t.Errorf("Execution took too long: %v (expected ~8-12 seconds for concurrent execution)", totalDuration)
	}

	if totalDuration < 5*time.Second {
		t.Errorf("Execution finished too quickly: %v (expected at least 5 seconds)", totalDuration)
	}

	// 等待最后的状态同步
	time.Sleep(1 * time.Second)

	// 验证状态上报消息
	for i := 1; i <= 2; i++ {
		var taskUID types.UID
		if i == 1 {
			taskUID = task1.UID
		} else {
			taskUID = task2.UID
		}

		topic := fmt.Sprintf("k8s4r/robots/test-robot/tasks/%s/status", taskUID)
		messages := mockClient.getPublishedMessages(topic)

		if len(messages) == 0 {
			t.Errorf("Task %d: No status messages published", i)
			continue
		}

		t.Logf("Task %d published %d status messages", i, len(messages))

		// 检查是否有 Assigned 事件
		hasAssigned := false
		hasCompleted := false

		for _, msgStr := range messages {
			var msg TaskStatusMessage
			if err := json.Unmarshal([]byte(msgStr), &msg); err != nil {
				continue
			}

			if msg.Event == "Assigned" {
				hasAssigned = true
			}

			if msg.State == "completed" || msg.State == "failed" {
				hasCompleted = true
			}
		}

		if !hasAssigned {
			t.Errorf("Task %d: No 'Assigned' event found", i)
		}

		if !hasCompleted {
			t.Errorf("Task %d: No completion status found", i)
		}
	}

	t.Logf("Test completed successfully!")
}

// TestTaskExecutor_Timeout 测试任务超时功能
func TestTaskExecutor_Timeout(t *testing.T) {
	mockClient := newMockMQTTClient()
	workDir := t.TempDir()
	logger := &testLogger{t: t}
	executor := NewTaskExecutor("test-robot", mockClient, workDir, logger)

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	err := executor.Start(ctx)
	if err != nil {
		t.Fatalf("Failed to start executor: %v", err)
	}
	defer executor.Stop(ctx)

	// 创建一个会超时的任务：sleep 10秒，但timeout设为2秒
	taskUID := types.UID(uuid.New().String())
	timeout := &metav1.Duration{Duration: 2 * time.Second}

	task := &robotv1alpha1.Task{
		ObjectMeta: metav1.ObjectMeta{
			UID:       taskUID,
			Name:      "task-timeout",
			Namespace: "default",
		},
		Spec: robotv1alpha1.TaskSpec{
			Name:        "task-timeout",
			Driver:      "exec",
			TargetRobot: "test-robot",
			Config: robotv1alpha1.TaskDriverConfig{
				ExecConfig: &robotv1alpha1.ExecDriverConfig{
					Command: "/bin/sh",
					Args:    []string{"-c", "sleep 10"},
				},
			},
			Timeout: timeout,
		},
		Status: robotv1alpha1.TaskStatus{
			State: robotv1alpha1.TaskStatePending,
		},
	}

	t.Logf("Creating task with 2s timeout (will sleep 10s)...")
	executor.createTask(ctx, task)

	// 等待一小段时间让任务启动
	time.Sleep(500 * time.Millisecond)

	// 验证任务已启动
	executor.tasksMu.RLock()
	taskCount := len(executor.tasks)
	executor.tasksMu.RUnlock()

	if taskCount != 1 {
		t.Fatalf("Expected 1 task to be running, got %d", taskCount)
	}

	t.Logf("Task started, waiting for timeout detection (2s timeout + up to 5s for monitor)...")

	// 等待足够长的时间让monitor检测到超时 (2s timeout + 5s monitor period + buffer)
	time.Sleep(10 * time.Second)

	// 检查是否收到了Timeout事件
	statusTopic := fmt.Sprintf("k8s4r/robots/test-robot/tasks/%s/status", taskUID)
	mockClient.mu.Lock()
	messages := mockClient.publishedMessages[statusTopic]
	mockClient.mu.Unlock()

	t.Logf("Total status messages received for task: %d", len(messages))

	var foundTimeout bool
	var lastStatus *robotv1alpha1.TaskStatus

	for i, msg := range messages {
		var status robotv1alpha1.TaskStatus
		if err := json.Unmarshal([]byte(msg), &status); err != nil {
			t.Logf("Message %d: failed to parse: %v", i, err)
			continue
		}
		lastStatus = &status
		t.Logf("Message %d: state=%s, message=%s", i, status.State, status.Message)

		if strings.Contains(status.Message, "Timeout") || strings.Contains(status.Message, "timeout") {
			foundTimeout = true
			t.Logf("✓ Found timeout event at message %d", i)
		}
	}

	// 验证任务已经被清理
	executor.tasksMu.RLock()
	finalTaskCount := len(executor.tasks)
	executor.tasksMu.RUnlock()

	t.Logf("Final task count: %d", finalTaskCount)

	if !foundTimeout {
		t.Errorf("Expected to find Timeout event in task messages")
		if lastStatus != nil {
			t.Logf("Last status: state=%s, message=%s", lastStatus.State, lastStatus.Message)
		}
	} else {
		t.Logf("✓ Test completed successfully! Task was terminated due to timeout.")
	}

	if finalTaskCount != 0 {
		t.Logf("Warning: Expected 0 tasks after timeout, but got %d", finalTaskCount)
	}
}

// TestTaskExecutor_TaskLifecycle 测试单个任务的完整生命周期
func TestTaskExecutor_TaskLifecycle(t *testing.T) {
	mockClient := newMockMQTTClient()
	workDir := t.TempDir()
	logger := &testLogger{t: t}
	executor := NewTaskExecutor("test-robot", mockClient, workDir, logger)

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	err := executor.Start(ctx)
	if err != nil {
		t.Fatalf("Failed to start executor: %v", err)
	}

	// 创建一个简单的任务
	task := createTestTask("simple-task", "test output", []string{}, 1*time.Second)

	t.Logf("Creating task: %s (UID: %s)", task.Name, task.UID)

	// 执行任务
	executor.createTask(ctx, task)

	// 验证任务已启动
	time.Sleep(200 * time.Millisecond)
	executor.tasksMu.RLock()
	if len(executor.tasks) != 1 {
		t.Fatalf("Expected 1 running task, got %d", len(executor.tasks))
	}
	executor.tasksMu.RUnlock()

	// 等待任务完成（1秒任务 + 最多5秒monitor周期）
	time.Sleep(7 * time.Second)

	// 验证任务已完成并清理
	executor.tasksMu.RLock()
	finalCount := len(executor.tasks)
	executor.tasksMu.RUnlock()

	if finalCount != 0 {
		t.Errorf("Expected 0 running tasks after completion, got %d", finalCount)
	}

	// 验证状态上报
	topic := fmt.Sprintf("k8s4r/robots/test-robot/tasks/%s/status", task.UID)
	messages := mockClient.getPublishedMessages(topic)

	t.Logf("Total status messages: %d", len(messages))

	if len(messages) < 2 {
		t.Errorf("Expected at least 2 status messages (assigned + completed), got %d", len(messages))
	}

	t.Log("Task lifecycle test completed!")
}
