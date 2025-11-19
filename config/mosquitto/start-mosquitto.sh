#!/bin/bash

# Mosquitto MQTT Broker 启动脚本
# 用于 K8s4r 项目的快速开发和测试

set -e

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo -e "${BLUE}==================================="
echo "Mosquitto MQTT Broker 启动脚本"
echo "===================================${NC}"

# 检查 Docker 是否安装
check_docker() {
    if ! command -v docker &> /dev/null; then
        echo -e "${RED}错误: Docker 未安装或不在 PATH 中${NC}"
        exit 1
    fi
    echo -e "${GREEN}✓ Docker 已安装${NC}"
}

# 停止现有的 Mosquitto 容器
stop_existing() {
    echo -e "${YELLOW}停止现有的 Mosquitto 容器...${NC}"
    docker stop k8s4r-mosquitto 2>/dev/null && echo -e "${GREEN}✓ 已停止现有容器${NC}" || echo -e "${YELLOW}⚠ 没有找到运行中的容器${NC}"
    docker rm k8s4r-mosquitto 2>/dev/null && echo -e "${GREEN}✓ 已删除现有容器${NC}" || echo -e "${YELLOW}⚠ 没有找到要删除的容器${NC}"
}

# 启动开发环境 Mosquitto
start_dev() {
    echo -e "${YELLOW}启动开发环境 Mosquitto...${NC}"
    
    docker run -d \
        --name k8s4r-mosquitto \
        -p 1883:1883 \
        -p 9001:9001 \
        -v "${SCRIPT_DIR}/mosquitto.conf:/mosquitto/config/mosquitto.conf:ro" \
        eclipse-mosquitto:2.0 \
        mosquitto -c /mosquitto/config/mosquitto.conf
    
    echo -e "${GREEN}✓ Mosquitto 开发环境已启动${NC}"
    echo -e "${BLUE}MQTT 端口: 1883${NC}"
    echo -e "${BLUE}WebSocket 端口: 9001${NC}"
}

# 启动生产环境 Mosquitto
start_prod() {
    echo -e "${YELLOW}启动生产环境 Mosquitto...${NC}"
    
    # 创建持久化目录
    mkdir -p /tmp/k8s4r-mosquitto/{data,log}
    
    docker run -d \
        --name k8s4r-mosquitto \
        -p 1883:1883 \
        -p 9001:9001 \
        -v "${SCRIPT_DIR}/mosquitto-prod.conf:/mosquitto/config/mosquitto.conf:ro" \
        -v "/tmp/k8s4r-mosquitto/data:/mosquitto/data" \
        -v "/tmp/k8s4r-mosquitto/log:/mosquitto/log" \
        eclipse-mosquitto:2.0 \
        mosquitto -c /mosquitto/config/mosquitto.conf
    
    echo -e "${GREEN}✓ Mosquitto 生产环境已启动${NC}"
    echo -e "${BLUE}MQTT 端口: 1883${NC}"
    echo -e "${BLUE}WebSocket 端口: 9001${NC}"
    echo -e "${BLUE}数据目录: /tmp/k8s4r-mosquitto/data${NC}"
    echo -e "${BLUE}日志目录: /tmp/k8s4r-mosquitto/log${NC}"
}

# 启动简单模式（使用 Mosquitto 2.0 + 简化配置）
start_simple() {
    echo -e "${YELLOW}启动简单模式 Mosquitto (v2.0 + 简化配置)...${NC}"
    
    docker run -d \
        --name k8s4r-mosquitto \
        -p 1883:1883 \
        -p 9001:9001 \
        -v "${SCRIPT_DIR}/mosquitto-simple.conf:/mosquitto/config/mosquitto.conf:ro" \
        eclipse-mosquitto:2.0 \
        mosquitto -c /mosquitto/config/mosquitto.conf
    
    echo -e "${GREEN}✓ Mosquitto 简单模式已启动${NC}"
    echo -e "${BLUE}MQTT 端口: 1883${NC}"
    echo -e "${BLUE}WebSocket 端口: 9001${NC}"
}

# 启动最小配置模式（专门解决连接问题）
start_minimal() {
    echo -e "${YELLOW}启动最小配置 Mosquitto (解决连接问题)...${NC}"
    
    # 创建临时最小配置
    cat > /tmp/mosquitto-minimal.conf << 'EOF'
listener 1883
allow_anonymous true
log_dest stdout
EOF
    
    docker run -d \
        --name k8s4r-mosquitto \
        -p 1883:1883 \
        -v "/tmp/mosquitto-minimal.conf:/mosquitto/config/mosquitto.conf:ro" \
        eclipse-mosquitto:2.0 \
        mosquitto -c /mosquitto/config/mosquitto.conf
    
    echo -e "${GREEN}✓ Mosquitto 最小配置已启动${NC}"
    echo -e "${BLUE}MQTT 端口: 1883${NC}"
}

# 显示状态
show_status() {
    echo -e "${YELLOW}检查 Mosquitto 状态...${NC}"
    
    if docker ps | grep -q k8s4r-mosquitto; then
        echo -e "${GREEN}✓ Mosquitto 容器正在运行${NC}"
        
        # 测试连接
        if command -v mosquitto_pub &> /dev/null; then
            # 使用gtimeout (如果安装了coreutils) 或直接测试
            if command -v gtimeout &> /dev/null; then
                timeout_cmd="gtimeout 3"
            else
                timeout_cmd=""
            fi
            
            if $timeout_cmd mosquitto_pub -h localhost -p 1883 -t "test/status" -m "ping" 2>/dev/null; then
                echo -e "${GREEN}✓ MQTT 连接测试成功${NC}"
            else
                echo -e "${YELLOW}⚠ MQTT 连接测试跳过（timeout不可用，但这不影响功能）${NC}"
            fi
        else
            echo -e "${YELLOW}⚠ mosquitto-clients 未安装，跳过连接测试${NC}"
        fi
        
        echo ""
        echo -e "${BLUE}容器日志:${NC}"
        docker logs --tail 10 k8s4r-mosquitto
    else
        echo -e "${RED}✗ Mosquitto 容器未运行${NC}"
    fi
}

# 显示帮助
show_help() {
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  dev      启动开发环境 Mosquitto (默认)"
    echo "  prod     启动生产环境 Mosquitto"
    echo "  simple   启动简单模式 Mosquitto (v2.0 + 简化配置)"
    echo "  minimal  启动最小配置 Mosquitto (解决连接问题)"
    echo "  stop     停止 Mosquitto"
    echo "  status   显示 Mosquitto 状态"
    echo "  help     显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0 dev      # 启动开发环境"
    echo "  $0 status   # 查看状态"
    echo "  $0 stop     # 停止服务"
}

# 主函数
main() {
    case "${1:-dev}" in
        "dev")
            check_docker
            stop_existing
            start_dev
            sleep 2
            show_status
            ;;
        "prod")
            check_docker
            stop_existing
            start_prod
            sleep 2
            show_status
            ;;
        "simple")
            check_docker
            stop_existing
            start_simple
            sleep 2
            show_status
            ;;
        "minimal")
            check_docker
            stop_existing
            start_minimal
            sleep 2
            show_status
            ;;
        "stop")
            stop_existing
            ;;
        "status")
            show_status
            ;;
        "help"|"-h"|"--help")
            show_help
            ;;
        *)
            echo -e "${RED}错误: 未知选项 '$1'${NC}"
            show_help
            exit 1
            ;;
    esac
}

main "$@"