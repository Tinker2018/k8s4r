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
	"os"

	"k8s.io/apimachinery/pkg/runtime"
	utilruntime "k8s.io/apimachinery/pkg/util/runtime"
	clientgoscheme "k8s.io/client-go/kubernetes/scheme"
	ctrl "sigs.k8s.io/controller-runtime"
	"sigs.k8s.io/controller-runtime/pkg/log/zap"

	robotv1alpha1 "github.com/hxndg/k8s4r/api/v1alpha1"
	"github.com/hxndg/k8s4r/pkg/server"
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
	var brokerURL string
	var namespace string

	flag.StringVar(&brokerURL, "broker-url", "tcp://localhost:1883", "The MQTT broker URL to connect to.")
	flag.StringVar(&namespace, "namespace", "default", "The namespace to watch for Robot resources.")

	opts := zap.Options{
		Development: true,
	}
	opts.BindFlags(flag.CommandLine)
	flag.Parse()

	ctrl.SetLogger(zap.New(zap.UseFlagOptions(&opts)))

	// 创建 Kubernetes client
	config := ctrl.GetConfigOrDie()
	k8sClient, err := ctrl.NewManager(config, ctrl.Options{
		Scheme: scheme,
	})
	if err != nil {
		setupLog.Error(err, "unable to create manager")
		os.Exit(1)
	}

	// 创建并启动 server
	srv := server.NewServer(k8sClient.GetClient(), namespace)

	setupLog.Info("starting MQTT server", "broker", brokerURL, "namespace", namespace)

	ctx := ctrl.SetupSignalHandler()

	// 启动 manager 的 cache
	go func() {
		if err := k8sClient.Start(ctx); err != nil {
			setupLog.Error(err, "problem running manager")
			os.Exit(1)
		}
	}()

	// 等待 cache 同步
	setupLog.Info("waiting for cache to sync")
	if !k8sClient.GetCache().WaitForCacheSync(ctx) {
		setupLog.Error(nil, "failed to wait for cache sync")
		os.Exit(1)
	}

	// 启动 MQTT server
	if err := srv.Start(ctx, brokerURL); err != nil {
		setupLog.Error(err, "problem running server")
		os.Exit(1)
	}

	// 启动 Task 监听器
	if err := srv.StartTaskWatcher(ctx); err != nil {
		setupLog.Error(err, "problem starting task watcher")
		os.Exit(1)
	}

	setupLog.Info("server started successfully")

	// 保持程序运行直到收到信号
	<-ctx.Done()
	setupLog.Info("shutting down server")
}
