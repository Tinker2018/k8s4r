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

	ctrl "sigs.k8s.io/controller-runtime"
	"sigs.k8s.io/controller-runtime/pkg/log/zap"

	"github.com/hxndghxndg/k8s4r/pkg/server"
)

var (
	setupLog = ctrl.Log.WithName("setup")
)

func main() {
	var brokerURL string
	var grpcAddr string

	flag.StringVar(&brokerURL, "broker-url", "tcp://localhost:1883", "The MQTT broker URL to connect to.")
	flag.StringVar(&grpcAddr, "grpc-addr", "localhost:9090", "The gRPC Manager address to connect to.")

	opts := zap.Options{
		Development: true,
	}
	opts.BindFlags(flag.CommandLine)
	flag.Parse()

	ctrl.SetLogger(zap.New(zap.UseFlagOptions(&opts)))

	// 创建 gRPC Stream Server（不依赖 K8s）
	srv := server.NewGRPCStreamServer(brokerURL, grpcAddr)

	setupLog.Info(" Starting Server (gRPC + MQTT, NO Kubernetes dependency)",
		"mqttBroker", brokerURL,
		"grpcManager", grpcAddr)

	ctx := ctrl.SetupSignalHandler()

	// 启动 Server（连接 Manager gRPC + MQTT Broker）
	if err := srv.Start(); err != nil {
		setupLog.Error(err, "failed to start server")
		os.Exit(1)
	}

	setupLog.Info(" Server started successfully - Ready to relay MQTT ↔ gRPC")

	// 保持程序运行直到收到信号
	<-ctx.Done()
	setupLog.Info("Shutting down server")
	srv.Stop()
}
