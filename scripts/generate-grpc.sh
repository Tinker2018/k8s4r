#!/bin/bash

# 生成 gRPC 代码的脚本

set -e

# 检查 protoc 是否安装
if ! command -v protoc &> /dev/null; then
    echo "Error: protoc is not installed"
    echo "Install with: brew install protobuf"
    exit 1
fi

# 检查 protoc-gen-go 是否安装
if ! command -v protoc-gen-go &> /dev/null; then
    echo "Installing protoc-gen-go..."
    GOPROXY=https://goproxy.cn,direct go install google.golang.org/protobuf/cmd/protoc-gen-go@latest
fi

# 检查 protoc-gen-go-grpc 是否安装
if ! command -v protoc-gen-go-grpc &> /dev/null; then
    echo "Installing protoc-gen-go-grpc..."
    GOPROXY=https://goproxy.cn,direct go install google.golang.org/grpc/cmd/protoc-gen-go-grpc@latest
fi

echo "Generating gRPC code from proto files..."

# 生成 Go 代码
protoc \
    --go_out=. \
    --go_opt=paths=source_relative \
    --go-grpc_out=. \
    --go-grpc_opt=paths=source_relative \
    api/grpc/robot_manager.proto

echo "✅ gRPC code generated successfully!"
echo "Generated files:"
echo "  - api/grpc/robot_manager.pb.go"
echo "  - api/grpc/robot_manager_grpc.pb.go"
