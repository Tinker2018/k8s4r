package driver

import (
	"context"
	"fmt"
	"os"
	"os/exec"
	"sync"
	"syscall"
	"time"

	robotv1alpha1 "github.com/hxndg/k8s4r/api/v1alpha1"
	"github.com/shirou/gopsutil/v3/process"
)

// ExecDriver 实现直接执行二进制文件的驱动
type ExecDriver struct {
	mu     sync.RWMutex
	tasks  map[string]*execTaskHandle
	logger Logger
}

// execTaskHandle exec 驱动的任务句柄
type execTaskHandle struct {
	TaskHandle
	cmd     *exec.Cmd
	process *process.Process
	doneCh  chan struct{}
	waitCh  chan *TaskStatus
}

// Logger 简单的日志接口
type Logger interface {
	Info(msg string, args ...interface{})
	Error(msg string, args ...interface{})
	Debug(msg string, args ...interface{})
}

// NewExecDriver 创建新的 Exec 驱动
func NewExecDriver(logger Logger) *ExecDriver {
	if logger == nil {
		logger = &defaultLogger{}
	}
	return &ExecDriver{
		tasks:  make(map[string]*execTaskHandle),
		logger: logger,
	}
}

// Name 返回驱动名称
func (d *ExecDriver) Name() string {
	return "exec"
}

// Start 启动任务
func (d *ExecDriver) Start(ctx context.Context, task *robotv1alpha1.Task) (*TaskHandle, error) {
	d.mu.Lock()
	defer d.mu.Unlock()

	taskID := string(task.UID)

	// 检查任务是否已存在
	if _, exists := d.tasks[taskID]; exists {
		return nil, fmt.Errorf("task %s already started", taskID)
	}

	// 解析配置
	cfg, err := d.parseConfig(task)
	if err != nil {
		return nil, fmt.Errorf("failed to parse config: %v", err)
	}

	// 创建命令
	cmd := exec.CommandContext(ctx, cfg.Command, cfg.Args...)

	// 设置工作目录
	if cfg.WorkDir != "" {
		cmd.Dir = cfg.WorkDir
	}

	// 设置环境变量
	if len(cfg.Env) > 0 {
		env := os.Environ()
		for k, v := range cfg.Env {
			env = append(env, fmt.Sprintf("%s=%s", k, v))
		}
		cmd.Env = env
	}

	// 设置进程组（用于后续清理子进程）
	cmd.SysProcAttr = &syscall.SysProcAttr{
		Setpgid: true,
	}

	// 启动进程
	if err := cmd.Start(); err != nil {
		return nil, fmt.Errorf("failed to start process: %v", err)
	}

	// 创建任务句柄
	handle := &execTaskHandle{
		TaskHandle: TaskHandle{
			TaskID:     taskID,
			DriverName: d.Name(),
			PID:        cmd.Process.Pid,
			StartedAt:  time.Now(),
			Config:     make(map[string]interface{}),
		},
		cmd:    cmd,
		doneCh: make(chan struct{}),
		waitCh: make(chan *TaskStatus, 1),
	}

	// 获取进程对象（用于资源监控）
	proc, err := process.NewProcess(int32(cmd.Process.Pid))
	if err != nil {
		d.logger.Error("failed to get process object", "pid", cmd.Process.Pid, "error", err)
	} else {
		handle.process = proc
	}

	// 保存句柄
	d.tasks[taskID] = handle

	// 异步等待进程结束
	go d.waitTask(handle)

	d.logger.Info("task started", "taskID", taskID, "pid", cmd.Process.Pid, "command", cfg.Command)

	return &handle.TaskHandle, nil
}

// Stop 停止任务
func (d *ExecDriver) Stop(ctx context.Context, handle *TaskHandle) error {
	d.mu.Lock()
	h, exists := d.tasks[handle.TaskID]
	d.mu.Unlock()

	if !exists {
		return fmt.Errorf("task %s not found", handle.TaskID)
	}

	d.logger.Info("stopping task", "taskID", handle.TaskID, "pid", handle.PID)

	// 发送 SIGTERM
	if err := h.cmd.Process.Signal(syscall.SIGTERM); err != nil {
		return fmt.Errorf("failed to send SIGTERM: %v", err)
	}

	// 等待一段时间后强制 SIGKILL
	select {
	case <-h.doneCh:
		// 进程已退出
		return nil
	case <-time.After(10 * time.Second):
		d.logger.Info("task did not exit gracefully, sending SIGKILL", "taskID", handle.TaskID)
		return h.cmd.Process.Kill()
	case <-ctx.Done():
		return ctx.Err()
	}
}

// Status 获取任务状态
func (d *ExecDriver) Status(ctx context.Context, handle *TaskHandle) (*TaskStatus, error) {
	d.mu.RLock()
	h, exists := d.tasks[handle.TaskID]
	d.mu.RUnlock()

	if !exists {
		return &TaskStatus{
			State:   TaskStateUnknown,
			Message: "task not found",
		}, nil
	}

	// 检查进程是否还在运行
	select {
	case status := <-h.waitCh:
		// 进程已结束
		return status, nil
	default:
		// 进程还在运行
		status := &TaskStatus{
			State:     TaskStateRunning,
			StartedAt: h.StartedAt,
		}

		// 获取资源使用情况
		if h.process != nil {
			if cpuPercent, err := h.process.CPUPercent(); err == nil {
				if memInfo, err := h.process.MemoryInfo(); err == nil {
					status.Resources = &ResourceUsage{
						CPUPercent: cpuPercent,
						MemoryMB:   int64(memInfo.RSS / 1024 / 1024),
					}
				}
			}
		}

		return status, nil
	}
}

// Destroy 销毁任务
func (d *ExecDriver) Destroy(ctx context.Context, handle *TaskHandle) error {
	d.mu.Lock()
	defer d.mu.Unlock()

	h, exists := d.tasks[handle.TaskID]
	if !exists {
		return nil // 已经清理了
	}

	// 确保进程已停止
	if h.cmd.Process != nil {
		_ = h.cmd.Process.Kill()
	}

	// 清理句柄
	delete(d.tasks, handle.TaskID)
	close(h.doneCh)

	d.logger.Info("task destroyed", "taskID", handle.TaskID)

	return nil
}

// waitTask 等待任务完成
func (d *ExecDriver) waitTask(h *execTaskHandle) {
	err := h.cmd.Wait()

	status := &TaskStatus{
		StartedAt:  h.StartedAt,
		FinishedAt: time.Now(),
	}

	if err != nil {
		if exitErr, ok := err.(*exec.ExitError); ok {
			status.State = TaskStateFailed
			status.ExitCode = exitErr.ExitCode()
			status.Message = fmt.Sprintf("process exited with code %d", exitErr.ExitCode())
		} else {
			status.State = TaskStateFailed
			status.Message = err.Error()
		}
	} else {
		status.State = TaskStateExited
		status.ExitCode = 0
		status.Message = "process completed successfully"
	}

	// 发送状态
	select {
	case h.waitCh <- status:
	default:
	}

	close(h.doneCh)

	d.logger.Info("task finished", "taskID", h.TaskID, "state", status.State, "exitCode", status.ExitCode)
}

// parseConfig 解析任务配置
func (d *ExecDriver) parseConfig(task *robotv1alpha1.Task) (*DriverConfig, error) {
	cfg := &DriverConfig{
		Env: make(map[string]string),
	}

	// 从 task.Spec.Config.ExecConfig 中解析
	if task.Spec.Config.ExecConfig == nil {
		return nil, fmt.Errorf("execConfig is required for exec driver")
	}

	execCfg := task.Spec.Config.ExecConfig

	// 获取 command
	if execCfg.Command == "" {
		return nil, fmt.Errorf("command is required")
	}
	cfg.Command = execCfg.Command

	// 获取 args
	cfg.Args = execCfg.Args

	// 获取 env（从 TaskSpec 中）
	if task.Spec.Env != nil {
		cfg.Env = task.Spec.Env
	}

	// 获取 user（从 TaskSpec 中）
	cfg.User = task.Spec.User

	// WorkDir 暂时不设置，可以后续扩展

	return cfg, nil
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
