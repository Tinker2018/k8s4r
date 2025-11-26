/*
Copyright 2025.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0
*/

package main

import (
	"flag"
	"net"
	"os"

	"google.golang.org/grpc"
	"k8s.io/apimachinery/pkg/runtime"
	utilruntime "k8s.io/apimachinery/pkg/util/runtime"
	clientgoscheme "k8s.io/client-go/kubernetes/scheme"
	ctrl "sigs.k8s.io/controller-runtime"
	"sigs.k8s.io/controller-runtime/pkg/healthz"
	"sigs.k8s.io/controller-runtime/pkg/log/zap"
	"sigs.k8s.io/controller-runtime/pkg/metrics/server"

	pb "github.com/hxndghxndg/k8s4r/api/grpc"
	robotv1alpha1 "github.com/hxndghxndg/k8s4r/api/v1alpha1"
	"github.com/hxndghxndg/k8s4r/pkg/controller"
	"github.com/hxndghxndg/k8s4r/pkg/manager"
)

var (
	scheme   = runtime.NewScheme()
	setupLog = ctrl.Log.WithName("setup")
)

func init() {
	utilruntime.Must(clientgoscheme.AddToScheme(scheme))
	utilruntime.Must(robotv1alpha1.AddToScheme(scheme))
}

func main() {
	var metricsAddr string
	var enableLeaderElection bool
	var probeAddr string
	var grpcAddr string
	var namespace string

	flag.StringVar(&metricsAddr, "metrics-bind-address", ":8082", "The address the metric endpoint binds to.")
	flag.StringVar(&probeAddr, "health-probe-bind-address", ":8081", "The address the probe endpoint binds to.")
	flag.StringVar(&grpcAddr, "grpc-bind-address", ":9090", "The address the gRPC server binds to.")
	flag.StringVar(&namespace, "namespace", "default", "The namespace to watch for resources.")
	flag.BoolVar(&enableLeaderElection, "leader-elect", false,
		"Enable leader election for controller manager. "+
			"Enabling this will ensure there is only one active controller manager.")

	opts := zap.Options{
		Development: true,
	}
	opts.BindFlags(flag.CommandLine)
	flag.Parse()

	ctrl.SetLogger(zap.New(zap.UseFlagOptions(&opts)))

	mgr, err := ctrl.NewManager(ctrl.GetConfigOrDie(), ctrl.Options{
		Scheme: scheme,
		Metrics: server.Options{
			BindAddress: metricsAddr,
		},
		HealthProbeBindAddress: probeAddr,
		LeaderElection:         enableLeaderElection,
		LeaderElectionID:       "robot.k8s4r.io",
	})
	if err != nil {
		setupLog.Error(err, "unable to start manager")
		os.Exit(1)
	}

	// 设置 RobotReconciler
	if err = (&controller.RobotReconciler{
		Client: mgr.GetClient(),
		Scheme: mgr.GetScheme(),
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "Robot")
		os.Exit(1)
	}

	// 设置 TaskGroupReconciler（负责调度和分发 TaskGroup）
	taskGroupStreamManager := manager.NewTaskGroupStreamManager(mgr.GetClient(), namespace)

	if err = (&controller.TaskGroupReconciler{
		Client: mgr.GetClient(),
		Scheme: mgr.GetScheme(),
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "TaskGroup")
		os.Exit(1)
	}

	// 设置 TaskReconciler（旧版，保留用于兼容性）
	if err = (&controller.TaskReconciler{
		Client:            mgr.GetClient(),
		Scheme:            mgr.GetScheme(),
		TaskStreamManager: nil, // 不再使用 Task Stream
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "Task")
		os.Exit(1)
	}

	// 设置 JobReconciler（负责根据 Job 创建 TaskGroup）
	if err = (&controller.JobReconciler{
		Client: mgr.GetClient(),
		Scheme: mgr.GetScheme(),
	}).SetupWithManager(mgr); err != nil {
		setupLog.Error(err, "unable to create controller", "controller", "Job")
		os.Exit(1)
	}

	setupLog.Info("Controllers initialized")

	// ========== 启动 TaskGroup Watcher ==========
	// 监控 TaskGroup 状态变化，自动推送到 gRPC Stream
	taskGroupWatcher := manager.NewTaskGroupWatcher(mgr.GetClient(), namespace, taskGroupStreamManager)
	taskGroupWatcher.Start()
	setupLog.Info("TaskGroup Watcher started")

	// ========== 启动 gRPC Server ==========
	// Manager 提供 gRPC 服务，Server 通过 gRPC 调用来通知状态变化
	setupLog.Info("Starting gRPC server", "address", grpcAddr)

	grpcServer := grpc.NewServer()

	// 创建复合服务，同时实现 unary RPC 和 StreamTaskGroups
	compositeService := &manager.CompositeGRPCService{
		GRPCServer:             manager.NewGRPCServer(mgr.GetClient(), namespace),
		TaskGroupStreamManager: taskGroupStreamManager,
	}
	pb.RegisterRobotManagerServer(grpcServer, compositeService)

	lis, err := net.Listen("tcp", grpcAddr)
	if err != nil {
		setupLog.Error(err, "failed to listen on gRPC address", "address", grpcAddr)
		os.Exit(1)
	}

	// 在单独的 goroutine 中启动 gRPC Server
	go func() {
		setupLog.Info("gRPC server listening", "address", grpcAddr)
		if err := grpcServer.Serve(lis); err != nil {
			setupLog.Error(err, "failed to serve gRPC")
			os.Exit(1)
		}
	}()

	if err := mgr.AddHealthzCheck("healthz", healthz.Ping); err != nil {
		setupLog.Error(err, "unable to set up health check")
		os.Exit(1)
	}
	if err := mgr.AddReadyzCheck("readyz", healthz.Ping); err != nil {
		setupLog.Error(err, "unable to set up ready check")
		os.Exit(1)
	}

	setupLog.Info("starting manager")
	if err := mgr.Start(ctrl.SetupSignalHandler()); err != nil {
		setupLog.Error(err, "problem running manager")
		os.Exit(1)
	}

	// 优雅停止 gRPC Server
	setupLog.Info("Shutting down gRPC server")
	grpcServer.GracefulStop()
}
