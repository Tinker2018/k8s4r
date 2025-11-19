#!/bin/bash

# K8s4r MQTT 测试脚本
# 用于测试 MQTT 消息的发布和订阅

set -e

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

BROKER_HOST="${1:-localhost}"
BROKER_PORT="${2:-1883}"

echo -e "${BLUE}==================================="
echo "K8s4r MQTT 消息测试"
echo "===================================${NC}"
echo -e "${BLUE}Broker: ${BROKER_HOST}:${BROKER_PORT}${NC}"

# 检查 mosquitto 客户端工具
check_mosquitto_clients() {
    if ! command -v mosquitto_pub &> /dev/null || ! command -v mosquitto_sub &> /dev/null; then
        echo -e "${RED}错误: mosquitto 客户端工具未安装${NC}"
        echo -e "${YELLOW}请安装 mosquitto 客户端:${NC}"
        echo "  macOS: brew install mosquitto"
        echo "  Ubuntu: sudo apt-get install mosquitto-clients"
        exit 1
    fi
    echo -e "${GREEN}✓ mosquitto 客户端工具已安装${NC}"
}

# 测试基本连接
test_connection() {
    echo -e "${YELLOW}测试 MQTT 连接...${NC}"
    
    if timeout 5 mosquitto_pub -h "$BROKER_HOST" -p "$BROKER_PORT" -t "test/connection" -m "ping" 2>/dev/null; then
        echo -e "${GREEN}✓ MQTT 连接成功${NC}"
    else
        echo -e "${RED}✗ MQTT 连接失败${NC}"
        echo -e "${YELLOW}请检查 Mosquitto broker 是否运行在 ${BROKER_HOST}:${BROKER_PORT}${NC}"
        exit 1
    fi
}

# 监听所有 k8s4r 消息
listen_all() {
    echo -e "${YELLOW}监听所有 k8s4r 消息 (按 Ctrl+C 停止)...${NC}"
    echo -e "${BLUE}主题模式: k8s4r/#${NC}"
    echo ""
    
    mosquitto_sub -h "$BROKER_HOST" -p "$BROKER_PORT" -t "k8s4r/#" -v
}

# 发送测试注册消息
send_register() {
    local robot_id="${1:-test-robot-$(date +%s)}"
    
    echo -e "${YELLOW}发送注册消息...${NC}"
    echo -e "${BLUE}Robot ID: $robot_id${NC}"
    
    local message=$(cat << EOF
{
  "robotId": "$robot_id",
  "token": "fixed-token-123",
  "deviceInfo": {
    "hostname": "test-host",
    "os": "linux",
    "arch": "amd64",
    "cpu": {
      "cores": 4,
      "usage": 25.5
    },
    "memory": {
      "total": 8192,
      "used": 4096
    },
    "disk": [
      {
        "device": "/dev/sda1",
        "total": 100,
        "used": 50
      }
    ],
    "network": [
      {
        "name": "eth0",
        "ip": "192.168.1.100"
      }
    ]
  }
}
EOF
    )
    
    mosquitto_pub -h "$BROKER_HOST" -p "$BROKER_PORT" -t "k8s4r/register" -m "$message"
    echo -e "${GREEN}✓ 注册消息已发送${NC}"
}

# 发送测试心跳消息
send_heartbeat() {
    local robot_id="${1:-test-robot-$(date +%s)}"
    
    echo -e "${YELLOW}发送心跳消息...${NC}"
    echo -e "${BLUE}Robot ID: $robot_id${NC}"
    
    local message=$(cat << EOF
{
  "robotId": "$robot_id",
  "token": "fixed-token-123",
  "deviceInfo": {
    "hostname": "test-host",
    "cpu": {
      "usage": $(( RANDOM % 100 ))
    },
    "memory": {
      "used": $(( 4000 + RANDOM % 1000 ))
    }
  }
}
EOF
    )
    
    mosquitto_pub -h "$BROKER_HOST" -p "$BROKER_PORT" -t "k8s4r/heartbeat" -m "$message"
    echo -e "${GREEN}✓ 心跳消息已发送${NC}"
}

# 监听特定机器人的响应
listen_response() {
    local robot_id="${1:-test-robot-001}"
    
    echo -e "${YELLOW}监听机器人响应 (按 Ctrl+C 停止)...${NC}"
    echo -e "${BLUE}Robot ID: $robot_id${NC}"
    echo -e "${BLUE}主题: k8s4r/response/$robot_id${NC}"
    echo ""
    
    mosquitto_sub -h "$BROKER_HOST" -p "$BROKER_PORT" -t "k8s4r/response/$robot_id" -v
}

# 发送测试命令
send_command() {
    local robot_id="${1:-test-robot-001}"
    local command="${2:-ping}"
    
    echo -e "${YELLOW}发送命令到机器人...${NC}"
    echo -e "${BLUE}Robot ID: $robot_id${NC}"
    echo -e "${BLUE}命令: $command${NC}"
    
    mosquitto_pub -h "$BROKER_HOST" -p "$BROKER_PORT" -t "k8s4r/commands/$robot_id" -m "$command"
    echo -e "${GREEN}✓ 命令已发送${NC}"
}

# 显示帮助
show_help() {
    echo "用法: $0 [broker_host] [broker_port] <命令> [参数]"
    echo ""
    echo "命令:"
    echo "  listen              监听所有 k8s4r 消息"
    echo "  register [robot_id] 发送注册消息"
    echo "  heartbeat [robot_id] 发送心跳消息"
    echo "  response [robot_id] 监听机器人响应"
    echo "  command [robot_id] [cmd] 发送命令到机器人"
    echo "  test                运行基本连接测试"
    echo "  help                显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  $0 test                          # 测试连接"
    echo "  $0 listen                        # 监听所有消息"
    echo "  $0 register robot-001            # 发送注册消息"
    echo "  $0 heartbeat robot-001           # 发送心跳"
    echo "  $0 response robot-001            # 监听响应"
    echo "  $0 command robot-001 restart     # 发送重启命令"
    echo ""
    echo "  # 使用不同的 broker"
    echo "  $0 test.mosquitto.org 1883 test"
}

# 主函数
main() {
    # 解析参数
    local cmd="test"
    local args=()
    
    # 如果第一个参数看起来像主机名，调整参数
    if [[ $# -gt 0 && ! "$1" =~ ^(listen|register|heartbeat|response|command|test|help)$ ]]; then
        BROKER_HOST="$1"
        shift
        if [[ $# -gt 0 && "$1" =~ ^[0-9]+$ ]]; then
            BROKER_PORT="$1"
            shift
        fi
    fi
    
    if [[ $# -gt 0 ]]; then
        cmd="$1"
        shift
        args=("$@")
    fi
    
    case "$cmd" in
        "test")
            check_mosquitto_clients
            test_connection
            ;;
        "listen")
            check_mosquitto_clients
            test_connection
            listen_all
            ;;
        "register")
            check_mosquitto_clients
            test_connection
            send_register "${args[0]}"
            ;;
        "heartbeat")
            check_mosquitto_clients
            test_connection
            send_heartbeat "${args[0]}"
            ;;
        "response")
            check_mosquitto_clients
            test_connection
            listen_response "${args[0]}"
            ;;
        "command")
            check_mosquitto_clients
            test_connection
            send_command "${args[0]}" "${args[1]}"
            ;;
        "help"|"-h"|"--help")
            show_help
            ;;
        *)
            echo -e "${RED}错误: 未知命令 '$cmd'${NC}"
            show_help
            exit 1
            ;;
    esac
}

main "$@"