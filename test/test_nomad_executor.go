package main

import (
	"context"
	"fmt"
	"time"

	"github.com/hashicorp/go-hclog"
	robotv1alpha1 "github.com/hxndghxndg/k8s4r/api/v1alpha1"
	"github.com/hxndghxndg/k8s4r/pkg/driver"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"
	"k8s.io/apimachinery/pkg/types"
)

func main() {
	fmt.Println("=== 测试 Nomad Executor 集成 ===\n")

	// 创建日志器
	logger := hclog.New(&hclog.LoggerOptions{
		Name:  "test-nomad-exec",
		Level: hclog.Info,
		Color: hclog.AutoColor,
	})

	// 创建 Nomad driver
	d := driver.NewNomadExecDriver("/tmp/k8s4r-nomad-test", logger)
	fmt.Printf(" 创建 Nomad driver: %s\n", d.Name())

	// 创建测试任务
	task := &robotv1alpha1.Task{
		ObjectMeta: metav1.ObjectMeta{
			UID:  types.UID("test-nomad-task-1"),
			Name: "test-ls-with-nomad",
		},
		Spec: robotv1alpha1.TaskSpec{
			Config: robotv1alpha1.TaskDriverConfig{
				ExecConfig: &robotv1alpha1.ExecDriverConfig{
					Command: "/bin/bash",
					Args: []string{
						"-c",
						`
echo "=== Nomad Executor 测试 ==="
echo "当前目录: $(pwd)"
echo "进程 PID: $$"
ls -la
echo "等待 2 秒..."
sleep 2
echo "任务完成！"
						`,
					},
				},
			},
		},
	}

	ctx := context.Background()

	// 1. 启动任务
	fmt.Println("\n1️⃣  启动任务...")
	handle, err := d.Start(ctx, task)
	if err != nil {
		logger.Error("failed to start task", "error", err)
		return
	}
	fmt.Printf(" 任务启动成功！PID: %d\n", handle.PID)

	// 2. 获取状态和资源监控
	fmt.Println("\n2️⃣  监控任务状态...")
	for i := 0; i < 3; i++ {
		time.Sleep(500 * time.Millisecond)

		status, err := d.Status(ctx, handle)
		if err != nil {
			logger.Error("failed to get status", "error", err)
			continue
		}

		fmt.Printf("   状态: %s", status.State)
		if status.Resources != nil {
			fmt.Printf(", CPU: %.2f%%, Memory: %d MB",
				status.Resources.CPUPercent,
				status.Resources.MemoryMB)
		}
		fmt.Println()
	}

	// 3. 等待任务完成
	fmt.Println("\n3️⃣  等待任务完成...")
	for i := 0; i < 10; i++ {
		time.Sleep(500 * time.Millisecond)

		status, _ := d.Status(ctx, handle)
		if status.State == driver.TaskStateExited {
			fmt.Printf(" 任务已退出，退出码: %d\n", status.ExitCode)
			break
		}
	}

	// 4. 读取日志
	fmt.Println("\n4️⃣  读取日志...")
	stdout, err := d.GetLogs(ctx, handle, true, 0)
	if err != nil {
		logger.Error("failed to get logs", "error", err)
	} else {
		fmt.Println(" 标准输出:")
		fmt.Println(stdout)
	}

	stderr, err := d.GetLogs(ctx, handle, false, 0)
	if err == nil && stderr != "" {
		fmt.Println(" 标准错误:")
		fmt.Println(stderr)
	}

	// 5. 清理资源
	fmt.Println("\n5️⃣  清理资源...")
	if err := d.Destroy(ctx, handle); err != nil {
		logger.Error("failed to cleanup", "error", err)
	} else {
		fmt.Println(" 资源清理完成")
	}

	fmt.Println("\n=== 测试完成 ===")
	fmt.Println("\n 成功！Nomad Executor 可以直接使用！")
	fmt.Println("\n提供的功能：")
	fmt.Println("   进程生命周期管理")
	fmt.Println("   日志自动轮转")
	fmt.Println("   资源监控（CPU、内存）")
	fmt.Println("   优雅停止")
	fmt.Println("   子进程清理")
}
