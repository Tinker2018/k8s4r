package agent

import (
	"context"
	"testing"
	"time"

	"github.com/hashicorp/go-hclog"
	robotv1alpha1 "github.com/hxndghxndg/k8s4r/api/v1alpha1"
	"github.com/hxndghxndg/k8s4r/pkg/driver"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
)

// TestTaskExecutor_NormalCompletion 测试正常执行并退出的任务
func TestTaskExecutor_NormalCompletion(t *testing.T) {
	logger := hclog.New(&hclog.LoggerOptions{
		Name:  "test",
		Level: hclog.Info,
	})

	baseDir := "/tmp/k8s4r-test-normal"
	nomadDriver := driver.NewNomadExecDriver(baseDir, logger)

	task := &robotv1alpha1.Task{
		ObjectMeta: metav1.ObjectMeta{
			UID:  types.UID("test-task-normal"),
			Name: "test-ls",
		},
		Spec: robotv1alpha1.TaskSpec{
			Name:          "test-ls",
			TaskGroupName: "test-group",
			Config: robotv1alpha1.TaskDriverConfig{
				ExecConfig: &robotv1alpha1.ExecDriverConfig{
					Command: "/bin/ls",
					Args:    []string{"-la", "/tmp/.X11-unix/"},
				},
			},
		},
	}

	te, err := NewTaskExecutor(TaskExecutorConfig{
		Task:    task,
		Driver:  nomadDriver,
		Logger:  logger,
		BaseDir: baseDir,
	})
	if err != nil {
		t.Fatalf("Failed to create TaskExecutor: %v", err)
	}

	events := []TaskEvent{}
	te.AddEventListener(TaskEventListenerFunc(func(event TaskEvent) {
		events = append(events, event)
		t.Logf("Event: %s - %s - %s", event.Type, event.State, event.Message)
	}))

	ctx := context.Background()
	if err := te.Start(ctx); err != nil {
		t.Fatalf("Failed to start task: %v", err)
	}

	t.Logf("Task started, waiting for completion...")

	if err := te.Wait(); err != nil {
		t.Fatalf("Task failed: %v", err)
	}

	status := te.GetStatus()
	t.Logf("Final status: State=%s, ExitCode=%d", status.State, status.ExitCode)

	if status.State != TaskStateCompleted {
		t.Errorf("Expected state Completed, got %s", status.State)
	}

	if status.ExitCode != 0 {
		t.Errorf("Expected exit code 0, got %d", status.ExitCode)
	}

	hasStarted := false
	hasCompleted := false
	for _, e := range events {
		if e.Type == EventTypeStarted {
			hasStarted = true
		}
		if e.Type == EventTypeCompleted {
			hasCompleted = true
		}
	}

	if !hasStarted {
		t.Errorf("Did not receive Started event")
	}
	if !hasCompleted {
		t.Errorf("Did not receive Completed event")
	}

	t.Logf("Test passed")
}

// TestTaskExecutor_Timeout 测试超时终止
func TestTaskExecutor_Timeout(t *testing.T) {
	logger := hclog.New(&hclog.LoggerOptions{
		Name:  "test",
		Level: hclog.Info,
	})

	baseDir := "/tmp/k8s4r-test-timeout"
	nomadDriver := driver.NewNomadExecDriver(baseDir, logger)

	task := &robotv1alpha1.Task{
		ObjectMeta: metav1.ObjectMeta{
			UID:  types.UID("test-task-timeout"),
			Name: "test-timeout",
		},
		Spec: robotv1alpha1.TaskSpec{
			Name:          "test-timeout",
			TaskGroupName: "test-group",
			Timeout: &metav1.Duration{
				Duration: 10 * time.Second,
			},
			Config: robotv1alpha1.TaskDriverConfig{
				ExecConfig: &robotv1alpha1.ExecDriverConfig{
					Command: "/bin/sleep",
					Args:    []string{"15"},
				},
			},
		},
	}

	te, err := NewTaskExecutor(TaskExecutorConfig{
		Task:    task,
		Driver:  nomadDriver,
		Logger:  logger,
		BaseDir: baseDir,
	})
	if err != nil {
		t.Fatalf("Failed to create TaskExecutor: %v", err)
	}

	events := []TaskEvent{}
	te.AddEventListener(TaskEventListenerFunc(func(event TaskEvent) {
		events = append(events, event)
		t.Logf("Event: %s - %s - %s", event.Type, event.State, event.Message)
	}))

	ctx := context.Background()
	if err := te.Start(ctx); err != nil {
		t.Fatalf("Failed to start task: %v", err)
	}

	t.Logf("Task started, waiting for timeout (10s)...")
	startTime := time.Now()

	te.Wait()

	elapsed := time.Since(startTime)
	t.Logf("Task terminated after %v", elapsed)

	status := te.GetStatus()
	t.Logf("Final status: State=%s, Message=%s", status.State, status.Message)

	if status.State != TaskStateFailed {
		t.Errorf("Expected state Failed (timeout), got %s", status.State)
	}

	if elapsed < 8*time.Second || elapsed > 13*time.Second {
		t.Errorf("Expected timeout around 10s, but took %v", elapsed)
	}

	hasFailed := false
	for _, e := range events {
		if e.Type == EventTypeFailed {
			hasFailed = true
		}
	}

	if !hasFailed {
		t.Errorf("Did not receive Failed event")
	}

	t.Logf("Test passed")
}

// TestTaskExecutor_Kill 测试主动杀死任务
func TestTaskExecutor_Kill(t *testing.T) {
	logger := hclog.New(&hclog.LoggerOptions{
		Name:  "test",
		Level: hclog.Info,
	})

	baseDir := "/tmp/k8s4r-test-kill"
	nomadDriver := driver.NewNomadExecDriver(baseDir, logger)

	task := &robotv1alpha1.Task{
		ObjectMeta: metav1.ObjectMeta{
			UID:  types.UID("test-task-kill"),
			Name: "test-kill",
		},
		Spec: robotv1alpha1.TaskSpec{
			Name:          "test-kill",
			TaskGroupName: "test-group",
			Config: robotv1alpha1.TaskDriverConfig{
				ExecConfig: &robotv1alpha1.ExecDriverConfig{
					Command: "/bin/sleep",
					Args:    []string{"15"},
				},
			},
		},
	}

	te, err := NewTaskExecutor(TaskExecutorConfig{
		Task:    task,
		Driver:  nomadDriver,
		Logger:  logger,
		BaseDir: baseDir,
	})
	if err != nil {
		t.Fatalf("Failed to create TaskExecutor: %v", err)
	}

	events := []TaskEvent{}
	te.AddEventListener(TaskEventListenerFunc(func(event TaskEvent) {
		events = append(events, event)
		t.Logf("Event: %s - %s - %s", event.Type, event.State, event.Message)
	}))

	ctx := context.Background()
	if err := te.Start(ctx); err != nil {
		t.Fatalf("Failed to start task: %v", err)
	}

	t.Logf("Task started, will kill after 3 seconds...")

	time.Sleep(3 * time.Second)

	t.Logf("Calling Kill()...")
	killStart := time.Now()
	if err := te.Kill(); err != nil {
		t.Fatalf("Failed to kill task: %v", err)
	}
	killDuration := time.Since(killStart)

	t.Logf("Kill() returned after %v", killDuration)

	status := te.GetStatus()
	t.Logf("Final status: State=%s, Message=%s", status.State, status.Message)

	if status.State != TaskStateTerminated {
		t.Errorf("Expected state Terminated, got %s", status.State)
	}

	if killDuration > 15*time.Second {
		t.Errorf("Kill took too long: %v", killDuration)
	}

	hasTerminated := false
	for _, e := range events {
		if e.Type == EventTypeTerminated {
			hasTerminated = true
		}
	}

	if !hasTerminated {
		t.Errorf("Did not receive Terminated event")
	}

	t.Logf("Test passed")
}
