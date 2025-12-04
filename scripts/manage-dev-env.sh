#!/bin/bash
# K8s4r 开发环境管理脚本
# 用法: ./scripts/dev.sh [start|stop|restart|status|logs|clean]

set -e

# 环境变量控制是否使用 go run（开发模式）
# USE_GO_RUN=1 ./scripts/manage-dev-env.sh start
USE_GO_RUN=${USE_GO_RUN:-0}

# 是否显示执行的命令（调试模式）
SHOW_COMMANDS=${SHOW_COMMANDS:-1}

# 颜色
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

# 项目根目录
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

# ============================================
# 辅助函数
# ============================================

# 打印并执行命令
run_cmd() {
    if [ "$SHOW_COMMANDS" = "1" ]; then
        echo -e "${CYAN}  $ $@${NC}"
    fi
    "$@"
}

# ============================================
# 函数定义
# ============================================

start_env() {
    echo -e "${BLUE}🚀 Starting K8s4r Development Environment${NC}"
    
    # 检查是否需要清理旧数据（如果 agent SVID 不存在但 server 数据存在，可能是 token 已被消耗）
    if [ -d ".spire-data/server" ] && [ ! -f ".spire-data/agent/agent_svid.der" ]; then
        echo -e "${YELLOW}⚠ Warning: Server data exists but agent SVID missing.${NC}"
        echo -e "${YELLOW}  This may indicate a failed previous start with consumed join token.${NC}"
        echo -e "${YELLOW}  Recommendation: Run 'make dev-clean' first, then 'make dev-start'${NC}"
        echo -e "${YELLOW}  Or press Ctrl+C to cancel, and we'll clean automatically...${NC}"
        sleep 2
        echo -e "${BLUE}  Cleaning old SPIRE data automatically...${NC}"
        rm -rf .spire-data/server .spire-data/agent
        echo -e "${GREEN}  ✓ Old data cleaned${NC}"
    fi
    
    # 清理旧进程
    stop_env_quiet
    sleep 1
    
    # 创建数据目录
    mkdir -p .spire-data/server .spire-data/agent /tmp/spire-agent config/mosquitto/certs
    
    # 1. SPIRE Server
    echo -e "${BLUE}[1/5] Starting SPIRE Server...${NC}"
    
    # 检查 spire-server，如果不存在则尝试下载
    SPIRE_SERVER_BIN="spire-server"
    SPIRE_AGENT_BIN="spire-agent"
    
    if ! command -v spire-server &> /dev/null; then
        echo -e "${YELLOW}  ⚠ spire-server not found, downloading...${NC}"
        
        SPIRE_VERSION="1.9.0"
        SPIRE_DIR="$PROJECT_ROOT/.spire-bin"
        mkdir -p "$SPIRE_DIR"
        
        if [ ! -f "$SPIRE_DIR/spire-server" ]; then
            echo -e "${BLUE}  Downloading SPIRE $SPIRE_VERSION...${NC}"
            cd "$SPIRE_DIR"
            wget "http://10.30.16.166/spire-${SPIRE_VERSION}-linux-amd64-musl.tar.gz"
            tar xzf "spire-${SPIRE_VERSION}-linux-amd64-musl.tar.gz"
            cp "spire-${SPIRE_VERSION}/bin/spire-server" .
            cp "spire-${SPIRE_VERSION}/bin/spire-agent" .
            chmod +x spire-server spire-agent
            rm -rf "spire-${SPIRE_VERSION}" "spire-${SPIRE_VERSION}-linux-amd64-musl.tar.gz"
            cd "$PROJECT_ROOT"
            echo -e "${GREEN}  ✓ SPIRE downloaded to .spire-bin/${NC}"
        fi
        SPIRE_SERVER_BIN="$SPIRE_DIR/spire-server"
        SPIRE_AGENT_BIN="$SPIRE_DIR/spire-agent"
    fi
    
    echo -e "${CYAN}  $ $SPIRE_SERVER_BIN run -config config/spire/server.conf > /tmp/spire-server.log 2>&1 &${NC}"
    "$SPIRE_SERVER_BIN" run -config config/spire/server.conf > /tmp/spire-server.log 2>&1 &
    echo -e "${GREEN}  ✓ Started (PID: $!)${NC}"
    sleep 3
    
    # 2. 生成 join token
    echo -e "${BLUE}[2/5] Generating join token...${NC}"
    echo -e "${CYAN}  $ $SPIRE_SERVER_BIN token generate -socketPath ./.spire-data/server/api.sock -spiffeID spiffe://k8s4r.example.org/agent/k8s4r-dev${NC}"
    JOIN_TOKEN=$("$SPIRE_SERVER_BIN" token generate -socketPath ./.spire-data/server/api.sock \
        -spiffeID spiffe://k8s4r.example.org/agent/k8s4r-dev 2>/dev/null | grep -oP 'Token: \K.*')
    
    if [ -z "$JOIN_TOKEN" ]; then
        echo -e "${RED}  ✗ Failed to generate join token${NC}"
        cat /tmp/spire-server.log
        exit 1
    fi
    echo -e "${GREEN}  ✓ Token: $JOIN_TOKEN${NC}"
    
    # 3. 注册 Workload (K8s4r Agent)
    echo -e "${BLUE}[3/5] Registering workload...${NC}"
    
    # 先删除已存在的 entry（如果有）
    EXISTING_ENTRY=$("$SPIRE_SERVER_BIN" entry show -socketPath ./.spire-data/server/api.sock \
        -spiffeID spiffe://k8s4r.example.org/agent/robot-001 2>/dev/null | grep -oP 'Entry ID\s+:\s+\K[a-f0-9-]+' || true)
    
    if [ -n "$EXISTING_ENTRY" ]; then
        echo -e "${YELLOW}  Found existing entry, deleting...${NC}"
        echo -e "${CYAN}  $ $SPIRE_SERVER_BIN entry delete -socketPath ./.spire-data/server/api.sock -entryID $EXISTING_ENTRY${NC}"
        "$SPIRE_SERVER_BIN" entry delete -socketPath ./.spire-data/server/api.sock \
            -entryID "$EXISTING_ENTRY" > /dev/null 2>&1
    fi
    
    # 创建新 entry
    echo -e "${CYAN}  $ $SPIRE_SERVER_BIN entry create -socketPath ./.spire-data/server/api.sock -spiffeID spiffe://k8s4r.example.org/agent/robot-001 -parentID spiffe://k8s4r.example.org/agent/k8s4r-dev -selector unix:uid:$(id -u)${NC}"
    "$SPIRE_SERVER_BIN" entry create -socketPath ./.spire-data/server/api.sock \
        -spiffeID spiffe://k8s4r.example.org/agent/robot-001 \
        -parentID spiffe://k8s4r.example.org/agent/k8s4r-dev \
        -selector unix:uid:$(id -u) > /dev/null 2>&1
    echo -e "${GREEN}  ✓ Registered: spiffe://k8s4r.example.org/agent/robot-001${NC}"
    
    # 4. 导出 CA Bundle 给 Mosquitto
    echo -e "${BLUE}[4/5] Exporting CA bundle...${NC}"
    
    # 确保 certs 目录有正确的权限（可能被 Docker 修改为 1883:1883）
    if [ -d config/mosquitto/certs ]; then
        sudo chown -R $(id -u):$(id -g) config/mosquitto/certs 2>/dev/null || true
    fi
    
    echo -e "${CYAN}  $ $SPIRE_SERVER_BIN bundle show -socketPath ./.spire-data/server/api.sock > config/mosquitto/certs/spire-ca.crt${NC}"
    "$SPIRE_SERVER_BIN" bundle show -socketPath ./.spire-data/server/api.sock \
        > config/mosquitto/certs/spire-ca.crt
    echo -e "${GREEN}  ✓ CA exported to config/mosquitto/certs/spire-ca.crt${NC}"
    
    # 5. Mosquitto
    echo -e "${BLUE}[5/7] Starting Mosquitto...${NC}"
    echo -e "${CYAN}  $ docker run -d --name mosquitto --network host -v $PROJECT_ROOT/config/mosquitto/mosquitto-hybrid.conf:/mosquitto/config/mosquitto.conf -v $PROJECT_ROOT/config/mosquitto/certs:/mosquitto/certs eclipse-mosquitto:2.0${NC}"
    docker run -d --name mosquitto --network host \
        -v "$PROJECT_ROOT/config/mosquitto/mosquitto-hybrid.conf:/mosquitto/config/mosquitto.conf" \
        -v "$PROJECT_ROOT/config/mosquitto/certs:/mosquitto/certs" \
        eclipse-mosquitto:2.0 > /dev/null
    echo -e "${GREEN}  ✓ Started (1883: plain, 8883: mTLS)${NC}"
    sleep 2
    
    # 6. K8s4r Agent (会自动启动 SPIRE Agent)
    echo -e "${BLUE}[6/7] Starting K8s4r Agent...${NC}"
    echo -e "${YELLOW}  K8s4r Agent will start SPIRE Agent automatically${NC}"
    echo -e "${YELLOW}  Join Token: $JOIN_TOKEN${NC}"
    
    # 确保环境变量可见
    echo -e "${BLUE}  Setting SPIRE_JOIN_TOKEN environment variable${NC}"
    export SPIRE_JOIN_TOKEN="$JOIN_TOKEN"
    echo -e "${GREEN}  SPIRE_JOIN_TOKEN=${SPIRE_JOIN_TOKEN:0:20}...${NC}"
    
    if [ "$USE_GO_RUN" = "1" ]; then
        echo -e "${YELLOW}  Using 'go run' mode${NC}"
        echo -e "${YELLOW}  Note: If AppArmor errors occur, use binary mode instead (without USE_GO_RUN=1)${NC}"
        echo -e "${CYAN}  $ SPIRE_JOIN_TOKEN=${SPIRE_JOIN_TOKEN} go run cmd/agent/main.go --robot-id robot-001 --broker-url ssl://localhost:8883 --plugins-config ./config/agent/plugins.yaml --log-level debug > /tmp/k8s4r-agent.log 2>&1 &${NC}"
        SPIRE_JOIN_TOKEN="$JOIN_TOKEN" go run cmd/agent/main.go \
            --robot-id robot-001 \
            --broker-url ssl://localhost:8883 \
            --plugins-config ./config/agent/plugins.yaml \
            --log-level debug > /tmp/k8s4r-agent.log 2>&1 &
    else
        if [ ! -f "bin/agent" ]; then
            echo -e "${YELLOW}  Binary not found, building...${NC}"
            echo -e "${CYAN}  $ go build -o bin/agent ./cmd/agent${NC}"
            go build -o bin/agent ./cmd/agent
        fi
        echo -e "${CYAN}  $ SPIRE_JOIN_TOKEN=${SPIRE_JOIN_TOKEN} ./bin/agent --robot-id robot-001 --broker-url ssl://localhost:8883 --plugins-config ./config/agent/plugins.yaml --log-level debug > /tmp/k8s4r-agent.log 2>&1 &${NC}"
        SPIRE_JOIN_TOKEN="$JOIN_TOKEN" ./bin/agent \
            --robot-id robot-001 \
            --broker-url ssl://localhost:8883 \
            --plugins-config ./config/agent/plugins.yaml \
            --log-level debug > /tmp/k8s4r-agent.log 2>&1 &
    fi
    AGENT_PID=$!
    echo -e "${GREEN}  ✓ Started (PID: $AGENT_PID)${NC}"
    sleep 3
    
    # 7. K8s4r Server
    echo -e "${BLUE}[7/7] Starting K8s4r Server...${NC}"
    if [ "$USE_GO_RUN" = "1" ]; then
        echo -e "${YELLOW}  Using 'go run' mode${NC}"
        echo -e "${CYAN}  $ go run cmd/server/main.go --mqtt-broker tcp://localhost:1883 --grpc-port 50051 > /tmp/k8s4r-server.log 2>&1 &${NC}"
        go run cmd/server/main.go --mqtt-broker tcp://localhost:1883 --grpc-port 50051 \
            > /tmp/k8s4r-server.log 2>&1 &
    else
        if [ ! -f "bin/server" ]; then
            echo -e "${YELLOW}  Binary not found, building...${NC}"
            echo -e "${CYAN}  $ make build${NC}"
            make build > /dev/null 2>&1
        fi
        echo -e "${CYAN}  $ ./bin/server --mqtt-broker tcp://localhost:1883 --grpc-port 50051 > /tmp/k8s4r-server.log 2>&1 &${NC}"
        ./bin/server --mqtt-broker tcp://localhost:1883 --grpc-port 50051 \
            > /tmp/k8s4r-server.log 2>&1 &
    fi
    echo -e "${GREEN}  ✓ Started (PID: $!)${NC}"
    
    echo -e "${YELLOW}  Press Ctrl+C to stop all components${NC}"
    echo -e "${GREEN}✅ All components started${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${CYAN}Tip: 查看日志可使用: make dev-logs${NC}"
    echo -e "${CYAN}Tip: 关闭命令显示: SHOW_COMMANDS=0 make dev-start${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    
    # 等待 Agent 进程（前台运行）
    wait $AGENT_PID
}

stop_env() {
    echo -e "${YELLOW}🛑 Stopping K8s4r Development Environment${NC}"
    pkill -f "spire-server" && echo -e "${GREEN}  ✓ SPIRE Server stopped${NC}" || true
    pkill -f "spire-agent" && echo -e "${GREEN}  ✓ SPIRE Agent stopped${NC}" || true
    pkill -f "cmd/server" && echo -e "${GREEN}  ✓ K8s4r Server stopped${NC}" || true
    pkill -f "cmd/agent" && echo -e "${GREEN}  ✓ K8s4r Agent stopped${NC}" || true
    docker rm -f mosquitto 2>/dev/null && echo -e "${GREEN}  ✓ Mosquitto stopped${NC}" || true
    echo -e "${GREEN}✅ All stopped${NC}"
}

stop_env_quiet() {
    pkill -f "spire-server" 2>/dev/null || true
    pkill -f "spire-agent" 2>/dev/null || true
    pkill -f "cmd/server" 2>/dev/null || true
    pkill -f "cmd/agent" 2>/dev/null || true
    # 强制删除 mosquitto 容器（-f 会停止并删除）
    docker rm -f mosquitto 2>/dev/null || true
}

clean_env() {
    stop_env
    echo -e "${YELLOW}🧹 Cleaning up...${NC}"
    rm -rf .spire-data
    rm -f /tmp/spire-server.log /tmp/k8s4r-server.log /tmp/k8s4r-agent.log
    rm -rf /tmp/spire-agent
    
    # 清理 Mosquitto certs（可能需要权限）
    if [ -f config/mosquitto/certs/spire-ca.crt ]; then
        if rm -f config/mosquitto/certs/spire-ca.crt 2>/dev/null; then
            echo -e "${GREEN}  ✓ Removed spire-ca.crt${NC}"
        else
            echo -e "${YELLOW}  ⚠ Need sudo to remove spire-ca.crt${NC}"
            sudo rm -f config/mosquitto/certs/spire-ca.crt && echo -e "${GREEN}  ✓ Removed spire-ca.crt (with sudo)${NC}"
        fi
    fi
    
    echo -e "${GREEN}✅ Cleaned${NC}"
}

show_status() {
    echo -e "${BLUE}📊 Status${NC}"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    
    # SPIRE Server
    if pgrep -f "spire-server" > /dev/null; then
        echo -e "SPIRE Server: ${GREEN}✅ Running${NC} (Port: 8081)"
    else
        echo -e "SPIRE Server: ${RED}❌ Stopped${NC}"
    fi
    
    # SPIRE Agent  
    if pgrep -f "spire-agent" > /dev/null; then
        echo -e "SPIRE Agent:  ${GREEN}✅ Running${NC} (Socket: /tmp/spire-agent/agent.sock)"
    else
        echo -e "SPIRE Agent:  ${RED}❌ Stopped${NC}"
    fi
    
    # Mosquitto
    if docker ps | grep -q mosquitto; then
        echo -e "Mosquitto:    ${GREEN}✅ Running${NC} (1883/8883)"
    else
        echo -e "Mosquitto:    ${RED}❌ Stopped${NC}"
    fi
    
    # Server
    if pgrep -f "cmd/server" > /dev/null; then
        echo -e "Server:       ${GREEN}✅ Running${NC}"
    else
        echo -e "Server:       ${RED}❌ Stopped${NC}"
    fi
    
    # Agent
    if pgrep -f "cmd/agent" > /dev/null; then
        echo -e "Agent:        ${GREEN}✅ Running${NC}"
    else
        echo -e "Agent:        ${RED}❌ Stopped${NC}"
    fi
}

show_logs() {
    echo -e "${BLUE}📋 Recent Logs${NC}"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo -e "${YELLOW}SPIRE Server:${NC}"
    tail -10 /tmp/spire-server.log 2>/dev/null || echo "  No logs"
    echo ""
    echo -e "${YELLOW}K8s4r Agent:${NC}"
    tail -10 /tmp/k8s4r-agent.log 2>/dev/null || echo "  No logs"
    echo ""
    echo -e "${YELLOW}K8s4r Server:${NC}"
    tail -10 /tmp/k8s4r-server.log 2>/dev/null || echo "  No logs"
    echo ""
    echo -e "${YELLOW}Mosquitto:${NC}"
    docker logs --tail 10 mosquitto 2>/dev/null || echo "  Not running"
}

show_help() {
    echo "K8s4r Development Environment Manager"
    echo ""
    echo "Usage: $0 <command>"
    echo ""
    echo "Commands:"
    echo "  start    - Start all components"
    echo "  stop     - Stop all components"
    echo "  restart  - Restart all components"
    echo "  status   - Show component status"
    echo "  logs     - Show recent logs"
    echo "  clean    - Stop and clean all data"
    echo ""
    echo "Examples:"
    echo "  $0 start"
    echo "  $0 status"
    echo "  make dev-start    # or use Makefile"
}

# ============================================
# 主逻辑
# ============================================

case "${1:-}" in
    start)
        start_env
        ;;
    stop)
        stop_env
        ;;
    restart)
        stop_env
        sleep 2
        start_env
        ;;
    status)
        show_status
        ;;
    logs)
        show_logs
        ;;
    clean)
        clean_env
        ;;
    *)
        show_help
        exit 1
        ;;
esac
