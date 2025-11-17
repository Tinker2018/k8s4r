.PHONY: all build clean test docker-build install-crd deploy

# 变量定义
GOBIN := $(shell go env GOPATH)/bin
CONTROLLER_GEN := $(GOBIN)/controller-gen

all: build

# 安装 controller-gen
controller-gen:
	@which controller-gen > /dev/null || (echo "Installing controller-gen..." && go install sigs.k8s.io/controller-tools/cmd/controller-gen@v0.13.0)

# 生成 CRD manifests
manifests: controller-gen
	$(CONTROLLER_GEN) crd paths="./api/..." output:crd:artifacts:config=config/crd

# 生成代码 (DeepCopy 等)
generate: controller-gen
	$(CONTROLLER_GEN) object paths="./api/..."

# 构建所有组件
build: generate
	@echo "Building manager..."
	@mkdir -p bin
	go build -o bin/manager ./cmd/manager
	@echo "Building server..."
	go build -o bin/server ./cmd/server
	@echo "Building agent..."
	go build -o bin/agent ./cmd/agent

# 清理
clean:
	rm -rf bin/

# 运行测试
test:
	go test ./... -v

# 安装 CRD 到集群
install-crd: manifests
	kubectl apply -f config/crd/

# 卸载 CRD
uninstall-crd:
	kubectl delete -f config/crd/

# 运行 manager (需要 kubeconfig)
run-manager: generate
	go run ./cmd/manager/main.go

# 运行 server
run-server:
	go run ./cmd/server/main.go

# 运行 agent (示例)
run-agent:
	go run ./cmd/agent/main.go --server-url=http://localhost:8080 --token=fixed-token-123 --robot-id=robot-001

# Docker 构建
docker-build:
	docker build -t k8s4r-manager:latest -f Dockerfile.manager .
	docker build -t k8s4r-server:latest -f Dockerfile.server .
	docker build -t k8s4r-agent:latest -f Dockerfile.agent .

# 初始化依赖
deps:
	go mod tidy
	go mod download
