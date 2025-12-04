.PHONY: all build clean test docker-build install-crd deploy

# 变量定义
GOBIN := $(shell go env GOPATH)/bin
CONTROLLER_GEN := $(GOBIN)/controller-gen

all: build

# 安装 controller-gen (使用更新的版本以兼容 Go 1.24)
controller-gen:
	@which controller-gen > /dev/null || (echo "Installing controller-gen..." && GOPROXY=https://goproxy.cn,direct go install sigs.k8s.io/controller-tools/cmd/controller-gen@latest)

# 生成 CRD manifests
manifests: controller-gen
	$(CONTROLLER_GEN) crd:allowDangerousTypes=true paths="./api/..." output:crd:artifacts:config=config/crd

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
	go run ./cmd/server/main.go --broker-url=tcp://localhost:1883

# 运行 agent (示例)
run-agent:
	go run ./cmd/agent/main.go --broker-url=tcp://localhost:1883 --token=fixed-token-123 --robot-id=robot-001

# Docker 构建
docker-build:
	docker build -t k8s4r-manager:latest -f Dockerfile.manager .
	docker build -t k8s4r-server:latest -f Dockerfile.server .
	docker build -t k8s4r-agent:latest -f Dockerfile.agent .

# 初始化依赖
deps:
	go mod tidy
	go mod download

# ============================================
# 开发环境管理 (SPIRE + Mosquitto)
# ============================================

.PHONY: dev-start dev-stop dev-clean dev-logs dev-status

# 启动完整开发环境 (SPIRE + Mosquitto + Server + Agent)
dev-start:
	@./scripts/manage-dev-env.sh start

# 修复 AppArmor 权限问题（针对 snap 安装的 Go）
fix-apparmor:
	@echo "Fixing AppArmor for snap applications..."
	@sudo systemctl start apparmor || true
	@sudo apparmor_parser -r /var/lib/snapd/apparmor/profiles/* 2>/dev/null || true
	@echo "AppArmor profiles reloaded"

# 使用 go run 启动开发环境（无需预先编译，如遇 AppArmor 问题先运行 make fix-apparmor）
dev-start-gorun: fix-apparmor
	@USE_GO_RUN=1 ./scripts/manage-dev-env.sh start

# 停止所有开发环境组件
dev-stop:
	@./scripts/manage-dev-env.sh stop

# 重启开发环境
dev-restart:
	@./scripts/manage-dev-env.sh restart

# 清理开发环境 (停止 + 清理数据)
dev-clean:
	@./scripts/manage-dev-env.sh clean

# 查看开发环境日志
dev-logs:
	@./scripts/manage-dev-env.sh logs

# 查看开发环境状态
dev-status:
	@./scripts/manage-dev-env.sh status

# 仅启动 Mosquitto
dev-mqtt:
	@echo "Starting Mosquitto..."
	@docker run -d --name mosquitto \
		--network host \
		-v $$(pwd)/config/mosquitto/mosquitto-hybrid.conf:/mosquitto/config/mosquitto.conf \
		-v $$(pwd)/config/mosquitto/certs:/mosquitto/certs \
		eclipse-mosquitto:2.0
	@echo "✅ Mosquitto started"
	@echo "   Plain: tcp://localhost:1883"
	@echo "   mTLS: ssl://localhost:8883"

# 测试 SPIRE API
dev-test-spire:
	@echo "Testing Mock SPIRE Server..."
	@echo ""
	@echo "1. Health Check:"
	@curl -s http://localhost:8090/health && echo "" || echo "❌ Failed"
	@echo ""
	@echo "2. List Identities:"
	@curl -s http://localhost:8090/api/list-identities | jq '.' || echo "❌ Failed"
	@echo ""
	@echo "3. CA Bundle:"
	@curl -s http://localhost:8090/api/ca-bundle | head -5

# 注册新的机器人身份
dev-register-robot:
	@read -p "Enter robot ID (e.g., robot-002): " robot_id; \
	spiffe_id="spiffe://k8s4r.example.org/agent/$$robot_id"; \
	echo "Registering: $$spiffe_id"; \
	curl -X POST http://localhost:8090/api/register-workload \
		-H "Content-Type: application/json" \
		-d "{\"spiffe_id\": \"$$spiffe_id\"}" | jq '.'

# ============================================
# SPIRE 集成测试
# ============================================

.PHONY: test-spire verify-spire

# 验证 SPIRE 集成的三点要求
verify-spire:
	@./scripts/verify-requirements.sh

# 运行 SPIRE 单元测试
test-spire:
	@echo "Running SPIRE unit tests..."
	@go test -v ./pkg/plugin/spire

# ============================================
# 帮助信息
# ============================================

.PHONY: help

help:
	@echo "K8s4r Makefile Commands"
	@echo "======================="
	@echo ""
	@echo "Build:"
	@echo "  make build              - Build all binaries"
	@echo "  make clean              - Clean build artifacts"
	@echo ""
	@echo "Development Environment:"
	@echo "  make dev-start          - Start full dev environment (SPIRE + MQTT + Server + Agent)"
	@echo "  make dev-start-gorun    - Start with 'go run' (auto-fix AppArmor if needed)"
	@echo "  make fix-apparmor       - Fix AppArmor permissions for snap Go"
	@echo "  make dev-stop           - Stop all components"
	@echo "  make dev-restart        - Restart all components"
	@echo "  make dev-status         - Show component status"
	@echo "  make dev-logs           - Show recent logs"
	@echo "  make dev-clean          - Stop and clean all data"
	@echo ""
	@echo "Individual Components:"
	@echo "  make dev-mqtt           - Start only Mosquitto"
	@echo ""
	@echo "Testing:"
	@echo "  make test               - Run all tests"
	@echo "  make verify-spire       - Verify SPIRE integration requirements"
	@echo "  make test-spire         - Run SPIRE unit tests"
	@echo ""
	@echo "Direct Script Usage:"
	@echo "  ./scripts/manage-dev-env.sh start|stop|restart|status|logs|clean"
