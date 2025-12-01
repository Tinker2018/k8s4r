package driver

import (
	"bufio"
	"context"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"runtime"
	"strings"
	"sync"
	"syscall"
	"time"

	robotv1alpha1 "github.com/hxndghxndg/k8s4r/api/v1alpha1"

	// Nomad 的核心包
	getter "github.com/hashicorp/go-getter"
	"github.com/hashicorp/go-hclog"
	"github.com/hashicorp/nomad/client/lib/cpustats"
	cstructs "github.com/hashicorp/nomad/client/structs"
	"github.com/hashicorp/nomad/drivers/shared/executor"
	"github.com/hashicorp/nomad/nomad/structs"
	"github.com/hashicorp/nomad/plugins/drivers"
)

// overlayMount 记录一个 overlay 挂载信息
type overlayMount struct {
	sysDir    string // 系统目录名称（如 "usr", "etc"）
	lowerDir  string // 只读底层（宿主机目录）
	upperDir  string // 可写上层（TaskGroup 修改层）
	workDir   string // OverlayFS 工作目录
	mergedDir string // 合并视图（挂载点）
	mounted   bool   // 是否已成功挂载
}

// ExecDriver 使用 Nomad 的 executor 实现的驱动
// 这是真正使用 Nomad 代码的实现，提供完整的进程隔离、日志管理、资源监控
type ExecDriver struct {
	mu            sync.RWMutex
	tasks         map[string]*taskHandle
	logger        hclog.Logger
	baseDir       string
	compute       cpustats.Compute
	eventCallback EventCallback
	cgroupsInited bool

	// TaskGroup 级别的 overlay 挂载（所有 Task 共享）
	// 如果多个 ExecDriver 实例共享同一个 overlayMu，则它们共享 overlay 状态
	overlayMu        *sync.RWMutex              // overlay 操作的共享锁（可选）
	overlayMounts    map[string][]*overlayMount // key: taskGroupName
	overlaySupported bool                       // 系统是否支持 overlay
	overlayCheckOnce sync.Once                  // 只检查一次
}

// taskHandle Nomad executor 的任务句柄
type taskHandle struct {
	TaskHandle
	exec            executor.Executor
	taskDir         string
	logDir          string
	doneCh          chan struct{}
	exitState       *executor.ProcessState
	pluginCtx       context.Context
	pluginCtxCancel context.CancelFunc
	taskGroupName   string // 用于清理 overlay
}

// NewExecDriver 创建使用 Nomad executor 的驱动
// overlayMu: 可选的共享锁，如果提供，则同 TaskGroup 的多个 Driver 实例共享 overlay 状态
func NewExecDriver(baseDir string, logger hclog.Logger, overlayMu ...*sync.RWMutex) *ExecDriver {
	if logger == nil {
		logger = hclog.NewNullLogger()
	}
	if baseDir == "" {
		baseDir = "/tmp/k8s4r/tasks"
	}

	// 创建 CPU 计算信息（Nomad executor 需要）
	compute := cpustats.Compute{
		NumCores: runtime.NumCPU(),
	}

	d := &ExecDriver{
		tasks:         make(map[string]*taskHandle),
		logger:        logger.Named("nomad_exec_driver"),
		baseDir:       baseDir,
		compute:       compute,
		cgroupsInited: true,
		overlayMounts: make(map[string][]*overlayMount),
	}

	// 如果提供了共享锁，则使用它；否则创建独立的锁
	if len(overlayMu) > 0 && overlayMu[0] != nil {
		d.overlayMu = overlayMu[0]
		logger.Debug("using shared overlay mutex for TaskGroup-level overlay coordination")
	} else {
		d.overlayMu = &sync.RWMutex{}
		logger.Debug("using independent overlay mutex (no TaskGroup-level sharing)")
	}

	return d
}

// // initCgroups 初始化 Nomad cgroups 父目录（只需要初始化一次）
// // 这模拟了 Nomad client 的 cgroup 初始化过程
// // 注意：调用者必须已持有 d.mu 锁
// func (d *ExecDriver) initCgroups() error {
// 	d.logger.Info(">>> ENTERING initCgroups() <<<")

// 	if d.cgroupsInited {
// 		d.logger.Info("cgroups already initialized, skipping")
// 		return nil
// 	}

// 	// 检测可用的 CPU 核心
// 	cores := fmt.Sprintf("0-%d", runtime.NumCPU()-1)

// 	d.logger.Info("initializing cgroups for k8s4r", "cores", cores, "mode", cgroupslib.GetMode())

// 	// 调用 Nomad 的 cgroup 初始化
// 	// 注意：这需要 root 权限！
// 	d.logger.Info("calling cgroupslib.Init() - this requires root privileges")
// 	if err := cgroupslib.Init(d.logger, cores); err != nil {
// 		d.logger.Error("failed to initialize cgroups", "error", err)
// 		return fmt.Errorf("failed to initialize cgroups (requires root): %w", err)
// 	}

// 	d.cgroupsInited = true
// 	d.logger.Info("cgroups initialized successfully")
// 	return nil
// }

// Name 返回驱动名称
func (d *ExecDriver) Name() string {
	return "nomad-exec"
}

// SetEventCallback 设置事件回调
func (d *ExecDriver) SetEventCallback(callback EventCallback) {
	d.mu.Lock()
	defer d.mu.Unlock()
	d.eventCallback = callback
}

// emitEvent 触发事件回调（调用者必须已持有锁或在锁外调用）
func (d *ExecDriver) emitEvent(taskUID, event, message string) {
	// 注意：这里不获取锁，因为通常从已持有锁的方法中调用
	// 如果callback为nil，直接返回避免panic
	callback := d.eventCallback

	d.logger.Info("emitting event", "taskUID", taskUID, "event", event, "message", message)

	if callback != nil {
		// 异步调用callback，避免阻塞主流程和潜在的死锁
		go callback(taskUID, event, message)
	} else {
		d.logger.Warn("event callback is nil, event not sent", "event", event)
	}
}

// Start 启动任务（使用 Nomad executor）
func (d *ExecDriver) Start(ctx context.Context, task *robotv1alpha1.Task) (*TaskHandle, error) {
	d.mu.Lock()
	defer d.mu.Unlock()
	taskID := string(task.UID)
	taskGroupName := task.Spec.TaskGroupName

	d.logger.Info("starting task with Nomad executor",
		"taskID", taskID,
		"name", task.Name,
		"taskGroup", taskGroupName)
	// 检查任务是否已存在
	if _, exists := d.tasks[taskID]; exists {
		return nil, fmt.Errorf("task %s already started", taskID)
	}

	// 解析配置
	if task.Spec.Config.ExecConfig == nil {
		return nil, fmt.Errorf("execConfig is required")
	}

	cfg := task.Spec.Config.ExecConfig

	// 1. 创建 Task 级别的核心工作目录（必须先创建，Hook 可能需要用到）
	taskGroupDir := filepath.Join(d.baseDir, taskGroupName)
	taskDir := filepath.Join(taskGroupDir, taskID)
	logDir := filepath.Join(taskDir, "logs")

	// 确保核心目录存在
	if err := os.MkdirAll(logDir, 0755); err != nil {
		return nil, fmt.Errorf("failed to create log dir %s: %w", logDir, err)
	}

	d.logger.Info("=========== 2 overlay supported, setting up overlay started")

	// ============================================================================
	// 设置 OverlayFS（TaskGroup 级别，第一个 Task 启动时创建）
	// 使用双重检查锁定（double-checked locking）确保并发安全
	// 注意：使用 overlayMu 而不是 mu，因为可能多个 driver 实例共享 overlay 状态
	// ============================================================================
	var overlays []*overlayMount

	// 使用 overlay 专用锁进行同步
	d.overlayMu.Lock()
	overlays, overlayExists := d.overlayMounts[taskGroupName]
	if !overlayExists {
		// 第二次检查（有锁，慢速路径）
		// 确保只有一个 Task 执行 overlay 初始化
		d.logger.Info("first task in TaskGroup, setting up overlay filesystem",
			"taskGroup", taskGroupName)

		var err error
		overlays, err = d.setupOverlayFSLocked(taskGroupName, taskGroupDir)
		if err != nil {
			d.logger.Warn("failed to setup overlay, will use readonly system dirs", "error", err)
			overlays = nil
		}
		// setupOverlayFSLocked 已经将 overlays 保存到 d.overlayMounts[taskGroupName]
	} else {
		d.logger.Debug("overlay already setup for TaskGroup", "taskGroup", taskGroupName)
	}
	d.overlayMu.Unlock()
	d.logger.Info("=========== 23 overlay supported, setting up overlay started")

	// 预创建日志文件（Nomad executor 在某些平台上可能需要）
	stdoutPath := filepath.Join(logDir, "stdout.log")
	stderrPath := filepath.Join(logDir, "stderr.log")
	for _, logPath := range []string{stdoutPath, stderrPath} {
		if f, err := os.OpenFile(logPath, os.O_CREATE|os.O_WRONLY|os.O_TRUNC, 0644); err != nil {
			return nil, fmt.Errorf("failed to create log file %s: %w", logPath, err)
		} else {
			f.Close()
		}
	}

	// 创建 Nomad executor（使用完整隔离模式）
	// NewExecutorWithIsolation 提供：
	// 1. cgroup 资源隔离（CPU、内存限制）
	// 2. chroot 文件系统隔离（进程只能看到 TaskDir）
	// 3. 需要通过 Mounts 挂载系统二进制文件才能运行系统命令
	execImpl := executor.NewExecutorWithIsolation(d.logger, d.compute)

	// cgroup tree初始化操作（需要 root 权限）
	// if err := d.initCgroups(); err != nil {
	// 	return nil, fmt.Errorf("failed to initialize cgroups: %w", err)
	// }

	// 生成两层 cgroup 路径：
	// 1. TaskGroup 层：nomad.slice/share.slice/<taskGroupName>
	//    - 限制整个 TaskGroup 的总资源（如总共 4 CPU, 8GB 内存）
	// 2. Task 层：nomad.slice/share.slice/<taskGroupName>/<taskID>
	//    - 限制单个 Task 的资源（如单个 Task 最多 2 CPU, 4GB 内存）
	// 好处：
	//   - TaskGroup 层确保所有 Task 总和不超过上限
	//   - Task 层确保单个失控的 Task 不会占用所有资源
	cgroupScope := filepath.Join("/sys/fs/cgroup/nomad.slice/share.slice", taskGroupName, taskID)

	d.logger.Info("cgroup configuration (two-level hierarchy)",
		"cgroupScope", cgroupScope,
		"taskGroupName", taskGroupName,
		"taskID", taskID,
		"note", "TaskGroup-level and Task-level resource limits")

	d.logger.Info("准备挂载配置",
		"taskDir", taskDir,
		"taskGroupDir", taskGroupDir,
		"sharedMount", fmt.Sprintf("/shared -> %s", taskGroupDir),
		"overlayCount", len(overlays))

	d.logger.Info("=========== 23 overlay supported, setting up overlay started")

	// ============================================================================
	// 构建 Mounts 配置
	// ============================================================================
	mounts := make([]*drivers.MountConfig, 0)

	// 1. 添加 overlay 的 merged 目录（可写！）
	overlayMap := make(map[string]string) // sysDir -> mergedDir
	for _, overlay := range overlays {
		if overlay.mounted {
			overlayMap[overlay.sysDir] = overlay.mergedDir

			// 对于顶级目录，直接挂载
			if !strings.Contains(overlay.sysDir, "/") {
				mounts = append(mounts, &drivers.MountConfig{
					TaskPath: "/" + overlay.sysDir,
					HostPath: overlay.mergedDir,
					Readonly: false, // 关键：可写！
				})
				d.logger.Debug("添加 overlay 挂载",
					"taskPath", "/"+overlay.sysDir,
					"hostPath", overlay.mergedDir)
			}
		}
	}

	// 2. 动态链接器配置（必需）
	// 注意：如果 /etc 有 overlay，这些文件已经在 merged 中了
	if _, hasEtcOverlay := overlayMap["etc"]; !hasEtcOverlay {
		mounts = append(mounts,
			&drivers.MountConfig{TaskPath: "/etc/ld.so.cache", HostPath: "/etc/ld.so.cache", Readonly: true},
			&drivers.MountConfig{TaskPath: "/etc/ld.so.conf", HostPath: "/etc/ld.so.conf", Readonly: true},
			&drivers.MountConfig{TaskPath: "/etc/ld.so.conf.d", HostPath: "/etc/ld.so.conf.d", Readonly: true},
			&drivers.MountConfig{TaskPath: "/etc/passwd", HostPath: "/etc/passwd", Readonly: true},
			&drivers.MountConfig{TaskPath: "/etc/group", HostPath: "/etc/group", Readonly: true},
		)
	}

	if _, hasLibOverlay := overlayMap["lib"]; !hasLibOverlay {
		mounts = append(mounts,
			&drivers.MountConfig{TaskPath: "/lib", HostPath: "/lib", Readonly: true},
		)
	}
	if _, hasLib64Overlay := overlayMap["lib64"]; !hasLib64Overlay {
		mounts = append(mounts,
			&drivers.MountConfig{TaskPath: "/lib64", HostPath: "/lib64", Readonly: true},
		)
	}

	// 对于 /usr/lib，如果有 /usr overlay 就不需要单独挂载了
	if _, hasUsrOverlay := overlayMap["usr"]; !hasUsrOverlay {
		mounts = append(mounts,
			&drivers.MountConfig{TaskPath: "/usr/lib", HostPath: "/usr/lib", Readonly: true},
			&drivers.MountConfig{TaskPath: "/usr/bin", HostPath: "/usr/bin", Readonly: true},
			&drivers.MountConfig{TaskPath: "/usr/sbin", HostPath: "/usr/sbin", Readonly: true},
		)
	}

	if _, hasBinOverlay := overlayMap["bin"]; !hasBinOverlay {
		mounts = append(mounts,
			&drivers.MountConfig{TaskPath: "/bin", HostPath: "/bin", Readonly: true},
		)
	}

	if _, hasSBinOverlay := overlayMap["sbin"]; !hasSBinOverlay {
		mounts = append(mounts,
			&drivers.MountConfig{TaskPath: "/sbin", HostPath: "/sbin", Readonly: true},
		)
	}

	if _, hasTmpOverlay := overlayMap["tmp"]; !hasTmpOverlay {
		mounts = append(mounts,
			&drivers.MountConfig{TaskPath: "/tmp", HostPath: "/tmp", Readonly: true},
		)
	}

	// 6. /shared 目录（TaskGroup 共享）
	// mounts = append(mounts, &drivers.MountConfig{TaskPath: "/shared", HostPath: taskGroupDir, Readonly: false})

	// 构建执行命令（Nomad 的格式）
	execCmd := &executor.ExecCommand{
		Cmd:  cfg.Command,
		Args: cfg.Args,
		Env:  os.Environ(), // 继承环境变量

		// 日志文件路径（Nomad 自动处理日志轮转！）
		StdoutPath: stdoutPath,
		StderrPath: stderrPath,

		// 工作目录
		TaskDir: taskDir,

		// 启用资源限制（cgroup）
		ResourceLimits: true,

		// 配置 Mounts：使用我们构建的 mounts 列表
		Mounts: mounts,

		// 设置 cgroup 覆盖路径
		OverrideCgroupV2: cgroupScope, // cgroup v2 统一路径
		OverrideCgroupV1: map[string]string{ // cgroup v1 每个控制器的路径
			"cpu":     cgroupScope,
			"cpuset":  cgroupScope,
			"memory":  cgroupScope,
			"pids":    cgroupScope,
			"freezer": cgroupScope,
		},

		// 用户（可选，需要 root 权限）
		// User: "nobody",
	}

	// 设置任务的资源配置
	execCmd.Resources = &drivers.Resources{
		NomadResources: &structs.AllocatedTaskResources{
			Cpu: structs.AllocatedCpuResources{
				CpuShares: 100, // 最小 CPU shares
			},
			Memory: structs.AllocatedMemoryResources{
				MemoryMB: 256, // 最小内存限制 256MB
			},
		},
		LinuxResources: &drivers.LinuxResources{
			CPUShares:        100,
			MemoryLimitBytes: 256 * 1024 * 1024,
			CpusetCgroupPath: cgroupScope, // 关键：设置 cgroup 路径，用于 StatsCgroup()
		},
	}

	// 创建插件上下文（用于 Nomad executor 的生命周期管理）
	pluginCtx, pluginCtxCancel := context.WithCancel(context.Background())

	// 启动进程（调用 Nomad 的 Launch 方法）
	d.emitEvent(taskID, "Starting", fmt.Sprintf("Starting process: %s", cfg.Command))
	ps, err := execImpl.Launch(execCmd)
	if err != nil {
		pluginCtxCancel()
		d.emitEvent(taskID, "StartFailed", fmt.Sprintf("Failed to launch process: %v", err))
		return nil, fmt.Errorf("failed to launch process via Nomad executor: %w", err)
	}

	d.emitEvent(taskID, "Started", fmt.Sprintf("Process started with PID %d", ps.Pid))
	d.logger.Info("process launched via Nomad executor",
		"taskID", taskID,
		"pid", ps.Pid,
		"command", cfg.Command,
		"stdout", execCmd.StdoutPath,
		"stderr", execCmd.StderrPath)

	// 创建任务句柄
	handle := &taskHandle{
		TaskHandle: TaskHandle{
			TaskID:     taskID,
			DriverName: d.Name(),
			PID:        ps.Pid,
			StartedAt:  time.Now(),
			Config:     make(map[string]interface{}),
		},
		exec:            execImpl,
		taskDir:         taskDir,
		logDir:          logDir,
		doneCh:          make(chan struct{}),
		pluginCtx:       pluginCtx,
		pluginCtxCancel: pluginCtxCancel,
		taskGroupName:   taskGroupName, // 保存 taskGroupName 用于清理
	}

	// 保存句柄
	d.tasks[taskID] = handle

	// 启动日志流式输出（实时打印 stdout/stderr）
	go d.streamLogs(handle, stdoutPath, "stdout")
	go d.streamLogs(handle, stderrPath, "stderr")

	// 异步等待进程结束（使用 Nomad 的 Wait 机制）
	go d.waitTask(handle)

	return &handle.TaskHandle, nil
}

// Stop 停止任务（使用 Nomad 的优雅停止机制）
func (d *ExecDriver) Stop(ctx context.Context, handle *TaskHandle) error {
	d.mu.Lock()
	h, exists := d.tasks[handle.TaskID]
	d.mu.Unlock()

	if !exists {
		return fmt.Errorf("task %s not found", handle.TaskID)
	}

	d.logger.Info("stopping task via Nomad executor", "taskID", handle.TaskID, "pid", handle.PID)

	// 使用 Nomad 的 Shutdown 方法（支持优雅停止）
	// 这会：1. 发送 SIGTERM  2. 等待 5 秒  3. 发送 SIGKILL  4. 清理所有子进程
	if err := h.exec.Shutdown("SIGTERM", 5*time.Second); err != nil {
		d.logger.Error("failed to shutdown task", "taskID", handle.TaskID, "error", err)
		return err
	}

	// 取消插件上下文
	h.pluginCtxCancel()

	// 等待任务结束或超时
	select {
	case <-h.doneCh:
		d.logger.Info("task stopped successfully", "taskID", handle.TaskID)
		return nil
	case <-time.After(10 * time.Second):
		d.logger.Warn("task stop timeout", "taskID", handle.TaskID)
		return fmt.Errorf("task stop timeout")
	}
}

// Status 获取任务状态（使用 Nomad 的 Stats 获取资源信息）
func (d *ExecDriver) Status(ctx context.Context, handle *TaskHandle) (*TaskStatus, error) {
	d.mu.RLock()
	h, exists := d.tasks[handle.TaskID]
	d.mu.RUnlock()

	if !exists {
		return nil, fmt.Errorf("task %s not found", handle.TaskID)
	}

	status := &TaskStatus{
		State:     TaskStateRunning,
		StartedAt: handle.StartedAt,
	}

	// 检查是否已结束
	select {
	case <-h.doneCh:
		status.State = TaskStateExited
		if h.exitState != nil {
			status.ExitCode = h.exitState.ExitCode
			if h.exitState.Signal != 0 {
				status.Signal = fmt.Sprintf("signal %d", h.exitState.Signal)
			}
			status.FinishedAt = h.exitState.Time
		}
	default:
		// 使用 Nomad 的 Stats 方法获取资源使用情况
		statsCtx, cancel := context.WithTimeout(ctx, 2*time.Second)
		defer cancel()

		statsCh, err := h.exec.Stats(statsCtx, 1*time.Second)
		if err != nil {
			d.logger.Debug("failed to get stats", "taskID", handle.TaskID, "error", err)
		} else {
			// 从 channel 读取第一个统计数据
			select {
			case stats := <-statsCh:
				if stats != nil && stats.ResourceUsage != nil {
					status.Resources = &ResourceUsage{
						CPUPercent: stats.ResourceUsage.CpuStats.Percent,
						MemoryMB:   int64(stats.ResourceUsage.MemoryStats.RSS / 1024 / 1024),
					}
				}
			case <-time.After(500 * time.Millisecond):
				// 超时，不影响返回
			}
		}
	}

	return status, nil
}

// GetLogs 获取任务日志（从 Nomad 管理的日志文件读取）
func (d *ExecDriver) GetLogs(ctx context.Context, handle *TaskHandle, stdout bool, tail int) (string, error) {
	d.mu.RLock()
	h, exists := d.tasks[handle.TaskID]
	d.mu.RUnlock()

	if !exists {
		return "", fmt.Errorf("task %s not found", handle.TaskID)
	}

	// 日志文件路径
	logPath := filepath.Join(h.logDir, "stdout.log")
	if !stdout {
		logPath = filepath.Join(h.logDir, "stderr.log")
	}

	// 读取日志文件
	content, err := os.ReadFile(logPath)
	if err != nil {
		if os.IsNotExist(err) {
			return "", nil
		}
		return "", fmt.Errorf("failed to read log file: %w", err)
	}

	// 如果指定了 tail，只返回最后 N 行
	if tail > 0 {
		lines := splitLines(string(content))
		if len(lines) > tail {
			lines = lines[len(lines)-tail:]
		}
		return joinLines(lines), nil
	}

	return string(content), nil
}

// Destroy 清理任务资源
func (d *ExecDriver) Destroy(ctx context.Context, handle *TaskHandle) error {
	d.mu.Lock()

	h, exists := d.tasks[handle.TaskID]
	if !exists {
		d.mu.Unlock()
		return nil
	}

	// 从 tasks map 中删除
	delete(d.tasks, handle.TaskID)
	taskGroupName := h.taskGroupName

	// 检查这个 TaskGroup 是否还有其他 Task
	hasOtherTasks := false
	for _, task := range d.tasks {
		if task.taskGroupName == taskGroupName {
			hasOtherTasks = true
			break
		}
	}

	d.mu.Unlock()

	// 取消插件上下文
	h.pluginCtxCancel()

	// 清理任务目录
	if err := os.RemoveAll(h.taskDir); err != nil {
		d.logger.Warn("failed to remove task directory", "taskDir", h.taskDir, "error", err)
	}

	// 如果这是 TaskGroup 的最后一个 Task，清理 overlay
	if !hasOtherTasks && taskGroupName != "" {
		d.logger.Info("last task in TaskGroup, cleaning up overlay",
			"taskGroup", taskGroupName)

		if err := d.cleanupOverlayFS(taskGroupName); err != nil {
			d.logger.Error("failed to cleanup overlay", "taskGroup", taskGroupName, "error", err)
		}
	}

	d.logger.Info("task cleanup completed", "taskID", handle.TaskID)
	return nil
}

// --- 私有方法 ---

// waitTask 等待任务结束（使用 Nomad 的 Wait 方法）
func (d *ExecDriver) waitTask(h *taskHandle) {
	defer close(h.doneCh)

	// 使用 Nomad executor 的 Wait 方法等待进程结束
	exitState, err := h.exec.Wait(h.pluginCtx)

	if err != nil {
		d.logger.Error("error waiting for task", "taskID", h.TaskID, "error", err)
		d.emitEvent(h.TaskID, "Failed", fmt.Sprintf("Task wait error: %v", err))
		return
	}

	// 保存退出状态
	h.exitState = exitState

	// 根据退出码判断成功还是失败
	if exitState.ExitCode == 0 {
		d.emitEvent(h.TaskID, "Completed", "Task completed successfully (exit code 0)")
		d.logger.Info("task completed successfully",
			"taskID", h.TaskID,
			"pid", h.PID,
			"exitCode", exitState.ExitCode)
	} else {
		d.emitEvent(h.TaskID, "Failed", fmt.Sprintf("Task failed with exit code %d", exitState.ExitCode))
		d.logger.Info("task exited with error",
			"taskID", h.TaskID,
			"pid", h.PID,
			"exitCode", exitState.ExitCode,
			"signal", exitState.Signal)
	}
}

// streamLogs 实时流式输出任务日志（tail -f 方式）
func (d *ExecDriver) streamLogs(h *taskHandle, logPath string, streamName string) {
	// 等待日志文件创建
	for i := 0; i < 50; i++ {
		if _, err := os.Stat(logPath); err == nil {
			break
		}
		time.Sleep(100 * time.Millisecond)
	}

	file, err := os.Open(logPath)
	if err != nil {
		d.logger.Error("failed to open log file", "path", logPath, "error", err)
		return
	}
	defer file.Close()

	// 移到文件末尾开始读取（只读取新内容）
	file.Seek(0, io.SeekStart)

	reader := bufio.NewReader(file)
	ticker := time.NewTicker(100 * time.Millisecond)
	defer ticker.Stop()

	for {
		select {
		case <-h.doneCh:
			// 任务结束，读取剩余日志后退出
			for {
				line, err := reader.ReadString('\n')
				if err != nil {
					if err != io.EOF {
						d.logger.Error("error reading log", "stream", streamName, "error", err)
					}
					return
				}
				if line != "" {
					line = strings.TrimSuffix(line, "\n")
					if streamName == "stdout" {
						d.logger.Info(fmt.Sprintf("[%s] %s", h.TaskID[:8], line))
					} else {
						d.logger.Error(fmt.Sprintf("[%s] %s", h.TaskID[:8], line))
					}
				}
			}

		case <-ticker.C:
			// 定期检查新内容
			for {
				line, err := reader.ReadString('\n')
				if err != nil {
					if err == io.EOF {
						break // 没有更多数据，继续等待
					}
					d.logger.Error("error reading log", "stream", streamName, "error", err)
					return
				}

				if line != "" {
					line = strings.TrimSuffix(line, "\n")
					if streamName == "stdout" {
						d.logger.Info(fmt.Sprintf("[%s] %s", h.TaskID[:8], line))
					} else {
						d.logger.Error(fmt.Sprintf("[%s] %s", h.TaskID[:8], line))
					}
				}
			}
		}
	}
}

// 辅助函数
func splitLines(s string) []string {
	var lines []string
	start := 0
	for i := 0; i < len(s); i++ {
		if s[i] == '\n' {
			lines = append(lines, s[start:i+1])
			start = i + 1
		}
	}
	if start < len(s) {
		lines = append(lines, s[start:])
	}
	return lines
}

func joinLines(lines []string) string {
	result := ""
	for _, line := range lines {
		result += line
	}
	return result
}

// GetResourceStats 获取实时资源统计（演示 Nomad Stats 的完整用法）
func (d *ExecDriver) GetResourceStats(ctx context.Context, handle *TaskHandle, interval time.Duration) (<-chan *cstructs.TaskResourceUsage, error) {
	d.mu.RLock()
	h, exists := d.tasks[handle.TaskID]
	d.mu.RUnlock()

	if !exists {
		return nil, fmt.Errorf("task %s not found", handle.TaskID)
	}

	// 直接返回 Nomad 的 Stats channel
	return h.exec.Stats(ctx, interval)
}

// downloadArtifacts 使用 go-getter 下载 artifacts
func (d *ExecDriver) downloadArtifacts(ctx context.Context, artifacts []robotv1alpha1.TaskArtifact, destDir string) error {
	d.logger.Info("starting artifact downloads", "count", len(artifacts), "destDir", destDir)

	for i, artifact := range artifacts {
		// 检查 context 是否已取消
		select {
		case <-ctx.Done():
			d.logger.Error("download cancelled by context", "error", ctx.Err())
			return fmt.Errorf("download cancelled: %w", ctx.Err())
		default:
		}

		d.logger.Info("downloading artifact",
			"index", i+1,
			"total", len(artifacts),
			"source", artifact.GetterSource,
			"relativeDest", artifact.RelativeDest,
			"mode", artifact.GetterMode)

		// 确定目标路径
		dest := destDir
		if artifact.RelativeDest != "" {
			dest = filepath.Join(destDir, artifact.RelativeDest)
			// 确保父目录存在
			destParent := filepath.Dir(dest)
			if err := os.MkdirAll(destParent, 0755); err != nil {
				d.logger.Error("failed to create parent directory", "dir", destParent, "error", err)
				return fmt.Errorf("failed to create parent dir %s: %w", destParent, err)
			}
			d.logger.Debug("ensured parent directory exists", "dir", destParent)
		}

		// 确定下载模式
		mode := getter.ClientModeAny
		switch artifact.GetterMode {
		case "file":
			mode = getter.ClientModeFile
			d.logger.Debug("using file mode")
		case "dir":
			mode = getter.ClientModeDir
			d.logger.Debug("using directory mode")
		default:
			d.logger.Debug("using auto-detect mode")
		}

		// 创建 go-getter 客户端（使用默认配置）
		client := &getter.Client{
			Ctx:  ctx,
			Src:  artifact.GetterSource,
			Dst:  dest,
			Mode: mode,
			// 添加进度回调
			ProgressListener: &downloadProgress{
				logger: d.logger,
				source: artifact.GetterSource,
			},
		}

		// 应用 getter options
		if len(artifact.GetterOptions) > 0 {
			// go-getter 支持通过 URL query 参数传递选项
			// 例如: "git::https://github.com/user/repo.git?ref=main"
			d.logger.Info("artifact has custom options", "options", artifact.GetterOptions)
		}

		// 执行下载
		d.logger.Info("starting download", "from", artifact.GetterSource, "to", dest)

		downloadStart := time.Now()
		err := client.Get()
		downloadDuration := time.Since(downloadStart)

		if err != nil {
			// 检查是否是超时错误
			if ctx.Err() != nil {
				d.logger.Error("artifact download timeout",
					"index", i+1,
					"source", artifact.GetterSource,
					"dest", dest,
					"duration", downloadDuration,
					"contextError", ctx.Err(),
					"downloadError", err)
				return fmt.Errorf("download timeout after %v: %w", downloadDuration, err)
			}

			d.logger.Error("artifact download failed",
				"index", i+1,
				"source", artifact.GetterSource,
				"dest", dest,
				"duration", downloadDuration,
				"error", err)
			return fmt.Errorf("failed to download artifact %d from %s to %s: %w", i+1, artifact.GetterSource, dest, err)
		}

		d.logger.Info("artifact downloaded successfully",
			"index", i+1,
			"total", len(artifacts),
			"source", artifact.GetterSource,
			"dest", dest,
			"duration", downloadDuration)
	}

	d.logger.Info("all artifacts downloaded successfully", "count", len(artifacts))
	return nil
}

// downloadProgress 实现 go-getter 的进度监听器
type downloadProgress struct {
	logger      hclog.Logger
	source      string
	lastLog     time.Time
	logInterval time.Duration
}

func (p *downloadProgress) TrackProgress(src string, currentSize, totalSize int64, stream io.ReadCloser) io.ReadCloser {
	// 初始化 logInterval（默认10秒）
	if p.logInterval == 0 {
		p.logInterval = 10 * time.Second
	}

	// 检查是否应该输出日志
	now := time.Now()
	if now.Sub(p.lastLog) >= p.logInterval {
		p.lastLog = now

		if totalSize > 0 {
			percent := float64(currentSize) / float64(totalSize) * 100
			p.logger.Info("download progress",
				"source", p.source,
				"current", currentSize,
				"total", totalSize,
				"percent", fmt.Sprintf("%.1f%%", percent))
		} else {
			p.logger.Info("download progress",
				"source", p.source,
				"current", currentSize,
				"status", "streaming")
		}
	}

	return stream
}

// ============================================================================
// OverlayFS 辅助函数
// ============================================================================

// checkOverlaySupport 检查系统是否支持 OverlayFS
func (d *ExecDriver) checkOverlaySupport() bool {
	d.overlayCheckOnce.Do(func() {
		// 检查 /proc/filesystems 中是否有 overlay
		data, err := os.ReadFile("/proc/filesystems")
		if err != nil {
			d.logger.Debug("failed to read /proc/filesystems", "error", err)
			d.overlaySupported = false
			return
		}

		d.overlaySupported = strings.Contains(string(data), "overlay")
		if d.overlaySupported {
			d.logger.Info("OverlayFS is supported on this system")
		} else {
			d.logger.Warn("OverlayFS is not supported, will use readonly mounts")
		}
	})

	return d.overlaySupported
}

// setupOverlayFSLocked 为 TaskGroup 设置 overlay 文件系统
// 注意：调用此函数前必须已经持有 d.overlayMu 锁
// 这样 InitTask 可以安装软件到系统目录，所有 Task 都能看到，TaskGroup 结束后自动清理
func (d *ExecDriver) setupOverlayFSLocked(taskGroupName string, taskGroupDir string) ([]*overlayMount, error) {
	// 检查是否支持 overlay
	if !d.checkOverlaySupport() {
		d.logger.Info("overlay not supported, skipping overlay setup")
		return nil, nil
	}
	d.logger.Info("=========== 0 overlay supported, setting up overlay started")

	overlayBaseDir := filepath.Join(taskGroupDir, "overlay")

	// 定义需要 overlay 的系统目录
	// 这些目录将支持可写操作，类似 Docker 容器
	systemDirs := []string{
		"usr", // 本地安装（最常用）
		"etc", // 配置文件
		"opt", // 第三方软件
		"bin",
		"sbin",
		"lib",
		"lib64",
		"tmp",
	}

	overlays := make([]*overlayMount, 0, len(systemDirs))

	for _, sysDir := range systemDirs {
		overlay := &overlayMount{
			sysDir:    sysDir,
			lowerDir:  filepath.Join("/", sysDir),
			upperDir:  filepath.Join(overlayBaseDir, sysDir, "upper"),
			workDir:   filepath.Join(overlayBaseDir, sysDir, "work"),
			mergedDir: filepath.Join(overlayBaseDir, sysDir, "merged"),
		}

		// 创建必要的目录
		for _, dir := range []string{overlay.upperDir, overlay.workDir, overlay.mergedDir} {
			if err := os.MkdirAll(dir, 0755); err != nil {
				d.logger.Warn("failed to create overlay dir", "dir", dir, "error", err)
				continue
			}
		}

		// 检查宿主机的系统目录是否存在
		if _, err := os.Stat(overlay.lowerDir); os.IsNotExist(err) {
			d.logger.Debug("system dir not found, skipping overlay", "dir", overlay.lowerDir)
			continue
		}

		// 挂载 OverlayFS
		if err := d.mountOverlay(overlay); err != nil {
			d.logger.Warn("failed to mount overlay",
				"sysDir", sysDir,
				"error", err,
				"note", "falling back to readonly mount")
			overlay.mounted = false
		} else {
			overlay.mounted = true
			d.logger.Info("overlay mounted successfully",
				"sysDir", sysDir,
				"lowerDir", overlay.lowerDir,
				"upperDir", overlay.upperDir,
				"mergedDir", overlay.mergedDir)
		}

		overlays = append(overlays, overlay)
	}

	// 保存到 driver 的 map 中（注意：调用者已持有锁）
	if d.overlayMounts == nil {
		d.overlayMounts = make(map[string][]*overlayMount)
	}
	d.overlayMounts[taskGroupName] = overlays

	return overlays, nil
}

// mountOverlay 挂载单个 overlay 文件系统
func (d *ExecDriver) mountOverlay(overlay *overlayMount) error {
	// 构建 mount 选项
	// overlay 格式: lowerdir=lower,upperdir=upper,workdir=work
	mountOpts := fmt.Sprintf("lowerdir=%s,upperdir=%s,workdir=%s",
		overlay.lowerDir, overlay.upperDir, overlay.workDir)

	d.logger.Debug("mounting overlay",
		"target", overlay.mergedDir,
		"options", mountOpts)

	// 使用 syscall.Mount 进行挂载
	// syscall.Mount(source, target, fstype, flags, data)
	err := syscall.Mount("overlay", overlay.mergedDir, "overlay", 0, mountOpts)
	if err != nil {
		return fmt.Errorf("mount overlay failed: %w", err)
	}

	overlay.mounted = true
	return nil
}

// unmountOverlay 卸载单个 overlay 文件系统
func (d *ExecDriver) unmountOverlay(overlay *overlayMount) error {
	if !overlay.mounted {
		return nil
	}

	d.logger.Info("unmounting overlay", "sysDir", overlay.sysDir, "mergedDir", overlay.mergedDir)

	// 先尝试正常卸载
	err := syscall.Unmount(overlay.mergedDir, 0)
	if err != nil {
		// 如果失败，尝试强制卸载（lazy unmount）
		d.logger.Warn("normal unmount failed, trying lazy unmount", "error", err)
		err = syscall.Unmount(overlay.mergedDir, syscall.MNT_DETACH)
		if err != nil {
			return fmt.Errorf("failed to unmount overlay: %w", err)
		}
	}

	overlay.mounted = false
	return nil
}

// cleanupOverlayFS 清理 TaskGroup 的所有 overlay 挂载
func (d *ExecDriver) cleanupOverlayFS(taskGroupName string) error {
	d.overlayMu.Lock()
	overlays, exists := d.overlayMounts[taskGroupName]
	if !exists {
		d.overlayMu.Unlock()
		return nil
	}
	delete(d.overlayMounts, taskGroupName)
	d.overlayMu.Unlock()

	d.logger.Info("cleaning up overlay mounts", "taskGroup", taskGroupName, "count", len(overlays))

	var lastErr error
	for _, overlay := range overlays {
		if err := d.unmountOverlay(overlay); err != nil {
			d.logger.Error("failed to unmount overlay", "sysDir", overlay.sysDir, "error", err)
			lastErr = err
		}
	}

	return lastErr
}
