#!/bin/bash

# K8S4R 快速启动脚本

set -e

echo "==================================="
echo "K8S4R 快速启动脚本"
echo "==================================="

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 检查依赖
check_dependencies() {
    echo -e "${YELLOW}检查依赖...${NC}"
    
    if ! command -v kubectl &> /dev/null; then
        echo -e "${RED}错误: kubectl 未安装${NC}"
        exit 1
    fi
    
    if ! command -v go &> /dev/null; then
        echo -e "${RED}错误: Go 未安装${NC}"
        exit 1
    fi
    
    echo -e "${GREEN}✓ 依赖检查通过${NC}"
}

# 安装 CRD
install_crd() {
    echo -e "${YELLOW}安装 CRD...${NC}"
    kubectl apply -f config/crd/robot_crd.yaml
    echo -e "${GREEN}✓ CRD 安装成功${NC}"
}

# 构建二进制文件
build_binaries() {
    echo -e "${YELLOW}构建二进制文件...${NC}"
    make build
    echo -e "${GREEN}✓ 构建完成${NC}"
}

# 显示使用说明
show_instructions() {
    echo ""
    echo -e "${GREEN}==================================="
    echo "安装完成！"
    echo "===================================${NC}"
    echo ""
    echo "接下来的步骤："
    echo ""
    echo "1. 在终端 1 中运行 Manager:"
    echo "   make run-manager"
    echo ""
    echo "2. 在终端 2 中启动 MQTT Broker:"
    echo "   docker run -it -p 1883:1883 -p 9001:9001 eclipse-mosquitto:2.0"
    echo ""
    echo "3. 在终端 3 中运行 Server:"
    echo "   make run-server"
    echo ""
    echo "4. 创建一个 Robot 资源 (在终端 4 中):"
    echo "   kubectl apply -f examples/robot.yaml"
    echo ""
    echo "5. 在终端 5 中运行 Agent:"
    echo "   make run-agent"
    echo ""
    echo "6. 查看 Robot 状态:"
    echo "   kubectl get robots"
    echo "   kubectl describe robot robot-001"
    echo ""
    echo "详细文档请查看 USAGE.md 和 MQTT_ARCHITECTURE.md"
    echo ""
}

# 主函数
main() {
    check_dependencies
    install_crd
    build_binaries
    show_instructions
}

main
