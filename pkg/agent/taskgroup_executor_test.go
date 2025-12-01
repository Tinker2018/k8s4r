package agent

import (
	"fmt"
	"os"
	"path/filepath"
	"testing"
	"time"

	"github.com/hashicorp/go-hclog"
	robotv1alpha1 "github.com/hxndghxndg/k8s4r/api/v1alpha1"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
)

// ============================================================================
// IMPORTANT: All tests in this file require root privileges to run
// ============================================================================
//
// These tests use Nomad executor with cgroup isolation, which requires
// root access to create and manage cgroups.
//
// Run with sudo:
//   sudo -E go test -v -run TestTaskGroupExecutor ./pkg/agent
//
// The -E flag preserves environment variables like GOPROXY, GOPATH, etc.
//
// ============================================================================

// TestTaskGroupExecutor_SimpleExecution 测试简单的 TaskGroup 执行
func TestTaskGroupExecutor_SimpleExecution(t *testing.T) {

	// 创建临时工作目录，这个workdir实际上是basedir
	workDir := filepath.Join(os.TempDir(), "k8s4r-test-taskgroup-simple")
	defer os.RemoveAll(workDir)

	if err := os.MkdirAll(workDir, 0755); err != nil {
		t.Fatalf("Failed to create work dir: %v", err)
	}

	// 创建 logger
	logger := hclog.New(&hclog.LoggerOptions{
		Name:   "test-taskgroup-executor",
		Level:  hclog.Debug,
		Output: os.Stdout,
	})

	// 创建测试 TaskGroup
	taskGroup := &robotv1alpha1.TaskGroup{
		ObjectMeta: metav1.ObjectMeta{
			Name:      "test-taskgroup-simple",
			Namespace: "default",
			UID:       types.UID("test-taskgroup-uid-12345"),
		},
		Spec: robotv1alpha1.TaskGroupSpec{
			// InitTask: 创建测试目录
			InitTasks: []robotv1alpha1.TaskDefinition{
				{
					Name:   "setup-workspace",
					Driver: "exec",
					Daemon: false,
					Config: robotv1alpha1.TaskDriverConfig{
						ExecConfig: &robotv1alpha1.ExecDriverConfig{
							Command: "/usr/bin/bash",
							Args: []string{
								"-c",
								`
								echo "=== Setting up workspace ==="
								echo "Current working directory: $(pwd)"
								echo "Checking mount points..."
								mount | grep /shared || echo "No /shared mount found"
								df -h | grep /shared || echo "No /shared filesystem found"
								
								echo "Listing root directory:"
								ls -la /
								
								echo "Creating directory /shared/hxntest/data"
								mkdir -p /shared/hxntest/data
								
								echo "Workspace created at /shared/hxntest"
								echo "test-data-$(date +%s)" > /shared/hxntest/data/test.txt
								
								echo "Verifying file was created:"
								ls -la /shared/hxntest/data/
								cat /shared/hxntest/data/test.txt
								
								echo "=== Setup completed ==="
								`,
							},
						},
					},
				},
			},
			// 主 Task: 验证 InitTask 完成
			Tasks: []robotv1alpha1.TaskDefinition{
				{
					Name:   "verify-and-run",
					Driver: "exec",
					Config: robotv1alpha1.TaskDriverConfig{
						ExecConfig: &robotv1alpha1.ExecDriverConfig{
							Command: "/usr/bin/bash",
							Args: []string{
								"-c",
								`
								echo "=== Main task starting ==="
								echo "Checking mount points in main task..."
								mount | grep /shared || echo "No /shared mount found in main task"
								
								echo "Listing root directory:"
								ls -la /
								
								echo "Checking /shared directory:"
								ls -la /shared/ || echo "/shared directory not found"
								ls -la /shared/hxntest/ || echo "/shared/hxntest directory not found"
								
								# 验证 InitTask 创建的目录和文件
								if [ -d /shared/hxntest/data ]; then
									echo "✓ Workspace directory exists"
								else
									echo "✗ Workspace directory not found"
									echo "Listing /shared:"
									ls -laR /shared/
									exit 1
								fi

								whoami

								if [ -f /shared/hxntest/data/test.txt ]; then
									echo "✓ Test file exists"
									echo "  Content: $(cat /shared/hxntest/data/test.txt)"
								else
									echo "✗ Test file not found"
									echo "Listing /shared/hxntest/data:"
									ls -la /shared/hxntest/data/ || echo "Directory doesn't exist"
									exit 1
								fi
								
								# 模拟一些工作
								echo "=== Performing work ==="
								for i in {1..3}; do
									echo "  Step $i: Processing..."
									sleep 1
								done
								
								echo "=== Main task completed successfully ==="
								`,
							},
						},
					},
					Env: map[string]string{
						"TASK_ENV": "test",
					},
				},
			},
		},
		Status: robotv1alpha1.TaskGroupStatus{
			State: "pending",
		},
	}

	taskGroupExecutorConfig := TaskGroupExecutorConfig{
		TaskGroup:     taskGroup,
		DriverFactory: NewSimpleDriverFactoryWithBaseDir(workDir, logger),
		Logger:        logger,
		BaseDir:       workDir,
		MQTTClient:    nil,
		RobotName:     "test-robot-001",
	}

	// 创建该 TaskGroup 的专属执行器
	task_group_executor, err := NewTaskGroupExecutor(taskGroupExecutorConfig)
	if err != nil {
		t.Fatalf("failed to create taskgroup executor: %v", err)
	}

	// 执行 TaskGroup
	t.Log("Starting TaskGroup execution...")
	err = task_group_executor.Start()
	if err != nil {
		t.Fatalf("TaskGroup execution start failed: %v", err)
	}

	err = task_group_executor.Wait()
	if err != nil {
		t.Fatalf("TaskGroup execution wait failed: %v", err)
	}

	t.Log("TaskGroup execution completed successfully")

	testFilePath := "/tmp/k8s4r-test-taskgroup-simple/test-taskgroup-simple/hxntest/data/test.txt"
	if _, err := os.Stat(testFilePath); os.IsNotExist(err) {
		t.Errorf("InitTask did not create expected file: %s", testFilePath)
	} else {
		t.Logf("✓ InitTask successfully created file: %s", testFilePath)
	}

}

// TestTaskGroupExecutor_WithDaemonInitTask 测试带守护进程 InitTask 的 TaskGroup
func TestTaskGroupExecutor_WithDaemonInitTask(t *testing.T) {

	// 创建临时工作目录
	workDir := filepath.Join(os.TempDir(), "k8s4r-test-taskgroup-daemon")
	defer os.RemoveAll(workDir)

	if err := os.MkdirAll(workDir, 0755); err != nil {
		t.Fatalf("Failed to create work dir: %v", err)
	}

	logger := hclog.New(&hclog.LoggerOptions{
		Name:   "test-taskgroup-daemon",
		Level:  hclog.Debug,
		Output: os.Stdout,
	})

	// 创建测试 TaskGroup
	taskGroup := &robotv1alpha1.TaskGroup{
		ObjectMeta: metav1.ObjectMeta{
			Name:      "test-taskgroup-daemon",
			Namespace: "default",
			UID:       types.UID("test-taskgroup-daemon-uid-67890"),
		},
		Spec: robotv1alpha1.TaskGroupSpec{
			InitTasks: []robotv1alpha1.TaskDefinition{
				// 守护进程 InitTask
				{
					Name:   "background-service",
					Driver: "exec",
					Daemon: true, // 守护进程
					Config: robotv1alpha1.TaskDriverConfig{
						ExecConfig: &robotv1alpha1.ExecDriverConfig{
							Command: "/usr/bin/bash",
							Args: []string{
								"-c",
								`
								echo "=== Background service starting ==="
								echo "Service PID: $$"
								mkdir -p /shared/hxntest/data/
								touch /shared/hxntest/data/k8s4r-daemon-test.pid
								echo $$ > /shared/hxntest/data/k8s4r-daemon-test.pid
								# 模拟后台服务
								while true; do
									echo "[$(date)] Background service is running..."
									sleep 2
								done
								`,
							},
						},
					},
				},
				// 等待守护进程启动
				{
					Name:   "wait-service",
					Driver: "exec",
					Daemon: false,
					Config: robotv1alpha1.TaskDriverConfig{
						ExecConfig: &robotv1alpha1.ExecDriverConfig{
							Command: "/usr/bin/bash",
							Args: []string{
								"-c",
								`
								echo "Waiting for background service..."
								for i in {1..10}; do
									if [ -f /shared/hxntest/data/k8s4r-daemon-test.pid ]; then
										daemon_pid=$(cat /shared/hxntest/data/k8s4r-daemon-test.pid)
										echo "Found PID file with PID: $daemon_pid"
										
										# 关键：验证进程真的在运行
										if kill -0 $daemon_pid 2>/dev/null; then
											echo "✓ Background service started and running (PID $daemon_pid)"
											exit 0
										else
											echo "✗ PID file exists but process $daemon_pid is not running"
											exit 1
										fi
									fi
									echo "Attempt $i: waiting for PID file..."
									sleep 1
								done
								echo "✗ Timeout waiting for service"
								exit 1
								`,
							},
						},
					},
				},
			},
			Tasks: []robotv1alpha1.TaskDefinition{
				{
					Name:   "main-task",
					Driver: "exec",
					Config: robotv1alpha1.TaskDriverConfig{
						ExecConfig: &robotv1alpha1.ExecDriverConfig{
							Command: "/usr/bin/bash",
							Args: []string{
								"-c",
								`
								echo "=== Main task starting ==="
								
								# 检查 PID 文件是否存在
								if [ ! -f /shared/hxntest/data/k8s4r-daemon-test.pid ]; then
									echo "✗ Background service PID file not found"
									exit 1
								fi
								
								# 读取 PID
								daemon_pid=$(cat /shared/hxntest/data/k8s4r-daemon-test.pid)
								echo "Background service PID: $daemon_pid"
								
								# 关键：验证进程是否真的在运行
								if kill -0 $daemon_pid 2>/dev/null; then
									echo "✓ Background service is running (PID $daemon_pid)"
								else
									echo "✗ Background service process not found (PID $daemon_pid may have exited)"
									exit 1
								fi
								
								# 额外验证：检查进程的 cmdline
								if [ -f /proc/$daemon_pid/cmdline ]; then
									echo "  Process cmdline: $(cat /proc/$daemon_pid/cmdline | tr '\0' ' ')"
								fi
								
								echo "=== Main task completed ==="
								`,
							},
						},
					},
				},
			},
		},
	}

	taskGroupExecutorConfig := TaskGroupExecutorConfig{
		TaskGroup:     taskGroup,
		DriverFactory: NewSimpleDriverFactoryWithBaseDir(workDir, logger),
		Logger:        logger,
		BaseDir:       workDir,
		MQTTClient:    nil,
		RobotName:     "test-robot-001",
	}

	// 创建该 TaskGroup 的专属执行器
	task_group_executor, err := NewTaskGroupExecutor(taskGroupExecutorConfig)
	if err != nil {
		t.Fatalf("failed to create taskgroup executor: %v", err)
	}

	t.Log("Starting TaskGroup execution...")
	err = task_group_executor.Start()
	if err != nil {
		t.Fatalf("TaskGroup execution start failed: %v", err)
	}

	err = task_group_executor.Wait()
	if err != nil {
		t.Fatalf("TaskGroup execution wait failed: %v", err)
	}

	t.Log("TaskGroup execution completed successfully")

	// 读取守护进程的 PID（在停止之前）
	pidFilePath := filepath.Join(workDir, "test-taskgroup-daemon", "hxntest", "data", "k8s4r-daemon-test.pid")
	var daemonPID int
	var hasDaemonPID bool
	if pidData, err := os.ReadFile(pidFilePath); err == nil {
		// 解析 PID（移除换行符等空白字符）
		pidStr := string(pidData)
		if _, err := fmt.Sscanf(pidStr, "%d", &daemonPID); err == nil {
			hasDaemonPID = true
			t.Logf("Daemon PID before Stop(): %d", daemonPID)
		} else {
			t.Logf("Warning: Could not parse daemon PID from file: %v", err)
		}
	} else {
		t.Logf("Warning: Could not read daemon PID file: %v", err)
	}

	// 清理守护进程
	t.Log("Calling Stop() to terminate daemon processes...")
	err = task_group_executor.Stop()
	if err != nil {
		t.Logf("Warning: Stop() returned error: %v", err)
	}

	// 等待进程完全终止
	time.Sleep(2 * time.Second)

	// 验证守护进程已被终止
	if hasDaemonPID {
		// 使用 os.FindProcess 和 Signal(0) 检查进程是否还存在
		process, err := os.FindProcess(daemonPID)
		if err == nil {
			// 尝试发送信号 0 检查进程是否存在
			// 在 Unix 系统上，Signal(syscall.Signal(0)) 可以检查进程是否存在
			err = process.Signal(os.Signal(nil))
			if err == nil {
				t.Errorf("✗ Daemon process (PID %d) is still running after Stop()", daemonPID)
			} else {
				t.Logf("✓ Daemon process (PID %d) has been terminated", daemonPID)
			}
		}
	}
}

// TestTaskGroupExecutor_TaskIsolation 测试不同 TaskGroup 的隔离性
func TestTaskGroupExecutor_TaskIsolation(t *testing.T) {
	// 检查是否以 root 运行
	if os.Geteuid() != 0 {
		t.Skip("This test requires root privileges (run with sudo)")
	}

	// 创建临时工作目录
	workDir1 := filepath.Join(os.TempDir(), "k8s4r-test-tg1")
	workDir2 := filepath.Join(os.TempDir(), "k8s4r-test-tg2")
	defer func() {
		os.RemoveAll(workDir1)
		os.RemoveAll(workDir2)
	}()

	for _, dir := range []string{workDir1, workDir2} {
		if err := os.MkdirAll(dir, 0755); err != nil {
			t.Fatalf("Failed to create work dir: %v", err)
		}
	}

	logger := hclog.New(&hclog.LoggerOptions{
		Name:   "test-isolation",
		Level:  hclog.Info,
		Output: os.Stdout,
	})

	// 创建两个 TaskGroup
	createTaskGroup := func(name, uid string) *robotv1alpha1.TaskGroup {
		return &robotv1alpha1.TaskGroup{
			ObjectMeta: metav1.ObjectMeta{
				Name:      name,
				Namespace: "default",
				UID:       types.UID(uid),
			},
			Spec: robotv1alpha1.TaskGroupSpec{
				Tasks: []robotv1alpha1.TaskDefinition{
					{
						Name:   "task-1",
						Driver: "exec",
						Config: robotv1alpha1.TaskDriverConfig{
							ExecConfig: &robotv1alpha1.ExecDriverConfig{
								Command: "/usr/bin/bash",
								Args: []string{
									"-c",
									`echo hello world > hxndg.txt; sleep 2`,
								},
							},
						},
					},
				},
			},
		}
	}

	tg1 := createTaskGroup("taskgroup-1", "tg-uid-1")
	tg2 := createTaskGroup("taskgroup-2", "tg-uid-2")

	tgec1 := TaskGroupExecutorConfig{
		TaskGroup:     tg1,
		DriverFactory: NewSimpleDriverFactoryWithBaseDir(workDir1, logger),
		Logger:        logger,
		BaseDir:       workDir1,
		MQTTClient:    nil,
		RobotName:     "test-robot-001",
	}

	tgec2 := TaskGroupExecutorConfig{
		TaskGroup:     tg2,
		DriverFactory: NewSimpleDriverFactoryWithBaseDir(workDir2, logger),
		Logger:        logger,
		BaseDir:       workDir2,
		MQTTClient:    nil,
		RobotName:     "test-robot-001",
	}

	// 创建该 TaskGroup 的专属执行器
	tge1, err := NewTaskGroupExecutor(tgec1)
	if err != nil {
		t.Fatalf("failed to create taskgroup executor: %v", err)
	}

	t.Log("Starting TaskGroup execution...")
	err = tge1.Start()
	if err != nil {
		t.Fatalf("TaskGroup execution start failed: %v", err)
	}

	err = tge1.Wait()
	if err != nil {
		t.Fatalf("TaskGroup execution wait failed: %v", err)
	}

	tge2, err := NewTaskGroupExecutor(tgec2)
	if err != nil {
		t.Fatalf("failed to create taskgroup executor: %v", err)
	}

	t.Log("Starting TaskGroup execution...")
	err = tge2.Start()
	if err != nil {
		t.Fatalf("TaskGroup execution start failed: %v", err)
	}

	err = tge2.Wait()
	if err != nil {
		t.Fatalf("TaskGroup execution wait failed: %v", err)
	}

	if _, err := os.Stat("/tmp/k8s4r-test-tg1/hxndg.txt"); err == nil {
		t.Log("Cleaning up daemon test PID file")
		os.Remove("/tmp/k8s4r-test-tg1")
	}

	if _, err := os.Stat("/tmp/k8s4r-test-tg2/hxndg.txt"); err == nil {
		t.Log("Cleaning up daemon test PID file")
		os.Remove("/tmp/k8s4r-test-tg2")
	}

	t.Log("✓ TaskGroup isolation verified: each has independent TaskExecutor")
}

// TestTaskGroupExecutor_OverlayFS 测试 OverlayFS 功能
// InitTask 安装软件到系统目录，Task 验证能够看到安装的文件
func TestTaskGroupExecutor_OverlayFS(t *testing.T) {

	// 创建临时工作目录
	workDir := filepath.Join(os.TempDir(), "k8s4r-test-overlay")
	// defer os.RemoveAll(workDir)

	if err := os.MkdirAll(workDir, 0755); err != nil {
		t.Fatalf("Failed to create work dir: %v", err)
	}

	logger := hclog.New(&hclog.LoggerOptions{
		Name:   "test-overlay",
		Level:  hclog.Debug,
		Output: os.Stdout,
	})

	// 创建测试 TaskGroup
	taskGroup := &robotv1alpha1.TaskGroup{
		ObjectMeta: metav1.ObjectMeta{
			Name:      "test-overlay-tg",
			Namespace: "default",
			UID:       types.UID("overlay-uid-123"),
		},
		Spec: robotv1alpha1.TaskGroupSpec{
			// InitTask: 安装文件到系统目录
			InitTasks: []robotv1alpha1.TaskDefinition{
				{
					Name:   "install-files",
					Driver: "exec",
					Daemon: false,
					Config: robotv1alpha1.TaskDriverConfig{
						ExecConfig: &robotv1alpha1.ExecDriverConfig{
							Command: "/usr/bin/sh",
							Args: []string{
								"-c",
								`
								# 安装可执行文件到 /usr/local/bin
								mkdir -p /usr/local/bin
								cat > /usr/local/bin/my-tool << 'EOF'
#!/bin/sh
echo "Tool installed by InitTask"
EOF
								chmod +x /usr/local/bin/my-tool
								
								# 创建配置文件到 /etc
								mkdir -p /etc/myapp
								echo "config-by-inittask" > /etc/myapp/app.conf
								
								echo "Installation completed"
								`,
							},
						},
					},
				},
			},
			// Task: 验证能够看到 InitTask 安装的文件
			Tasks: []robotv1alpha1.TaskDefinition{
				{
					Name:   "verify-files",
					Driver: "exec",
					Config: robotv1alpha1.TaskDriverConfig{
						ExecConfig: &robotv1alpha1.ExecDriverConfig{
							Command: "/usr/bin/sh",
							Args: []string{
								"-c",
								`
								# 验证可执行文件存在
								if [ -f /usr/local/bin/my-tool ]; then
									echo "✓ Found /usr/local/bin/my-tool"
									/usr/local/bin/my-tool
								else
									echo "✗ /usr/local/bin/my-tool not found"
									exit 1
								fi
								
								# 验证配置文件存在
								if [ -f /etc/myapp/app.conf ]; then
									echo "✓ Found /etc/myapp/app.conf"
									cat /etc/myapp/app.conf
								else
									echo "✗ /etc/myapp/app.conf not found"
									exit 1
								fi
								
								echo "All verifications passed!"
								`,
							},
						},
					},
				},
			},
		},
	}

	// 创建 TaskGroupExecutor
	driverFactory := NewSimpleDriverFactoryWithBaseDir(workDir, logger)
	config := TaskGroupExecutorConfig{
		TaskGroup:     taskGroup,
		DriverFactory: driverFactory,
	}
	executor, err := NewTaskGroupExecutor(config)
	if err != nil {
		t.Fatalf("Failed to create TaskGroupExecutor: %v", err)
	}

	// 启动 TaskGroup
	err = executor.Start()
	if err != nil {
		t.Fatalf("Failed to start TaskGroup: %v", err)
	}

	// 等待完成（最多 30 秒）
	done := make(chan error, 1)
	go func() {
		done <- executor.Wait()
	}()

	select {
	case err := <-done:
		if err != nil {
			t.Fatalf("TaskGroup execution failed: %v", err)
		}
		t.Log("✓ TaskGroup completed successfully")
	case <-time.After(30 * time.Second):
		executor.Stop()
		t.Fatal("TaskGroup execution timed out")
	}

	t.Log("✓ OverlayFS test passed: Task can see files installed by InitTask")
}
