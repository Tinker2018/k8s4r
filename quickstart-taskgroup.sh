#!/bin/bash
# TaskGroup 集成测试快速启动脚本

set -e

PROJECT_ROOT="/home/eai/hexiaonan/k8s4r"
cd "$PROJECT_ROOT"

# 颜色输出
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}TaskGroup 集成测试 - 快速启动${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# 步骤 1: 检查并编译组件
echo -e "${GREEN}[1/6] 编译组件...${NC}"
if [ ! -f "bin/manager" ] || [ ! -f "bin/server" ] || [ ! -f "bin/agent" ]; then
    echo "  编译 Manager..."
    go build -o bin/manager ./cmd/manager
    echo "  编译 Server..."
    go build -o bin/server ./cmd/server
    echo "  编译 Agent..."
    go build -o bin/agent ./cmd/agent
    echo -e "  ${GREEN}✓ 编译完成${NC}"
else
    echo -e "  ${GREEN}✓ 二进制文件已存在${NC}"
fi
echo ""

# 步骤 2: 检查 MQTT Broker
echo -e "${GREEN}[2/6] 检查 MQTT Broker...${NC}"
if ! docker ps | grep -q mosquitto; then
    echo "  启动 MQTT Broker..."
    ./config/mosquitto/start-mosquitto.sh simple
    sleep 2
    echo -e "  ${GREEN}✓ MQTT Broker 已启动${NC}"
else
    echo -e "  ${GREEN}✓ MQTT Broker 已运行${NC}"
fi
echo ""

# 步骤 3: 检查 Kubernetes 集群
echo -e "${GREEN}[3/6] 检查 Kubernetes 集群...${NC}"
if ! kubectl cluster-info &>/dev/null; then
    echo -e "  ${RED}✗ Kubernetes 集群不可用${NC}"
    echo "  请确保 Kubernetes 集群正在运行"
    exit 1
fi
echo -e "  ${GREEN}✓ Kubernetes 集群可用${NC}"
echo ""

# 步骤 4: 安装 CRD
echo -e "${GREEN}[4/6] 安装 CRD...${NC}"
kubectl apply -f config/crd/robot.k8s4r.io_robots.yaml 2>/dev/null || true
kubectl apply -f config/crd/robot.k8s4r.io_jobs.yaml 2>/dev/null || true
kubectl apply -f config/crd/robot.k8s4r.io_taskgroups.yaml 2>/dev/null || true
kubectl apply -f config/crd/robot.k8s4r.io_tasks.yaml 2>/dev/null || true
echo -e "  ${GREEN}✓ CRD 已安装${NC}"
echo ""

# 步骤 5: 创建 Robot
echo -e "${GREEN}[5/6] 创建 Robot CR...${NC}"
if kubectl get robot robot-001 &>/dev/null; then
    echo -e "  ${GREEN}✓ Robot robot-001 已存在${NC}"
else
    kubectl apply -f examples/robot.yaml
    echo -e "  ${GREEN}✓ Robot robot-001 已创建${NC}"
fi
echo ""

# 步骤 6: 显示启动指令
echo -e "${GREEN}[6/6] 准备完成！${NC}"
echo ""
echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}请在不同终端执行以下命令:${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""
echo -e "${BLUE}Terminal 1 - Server:${NC}"
echo "  cd $PROJECT_ROOT && ./bin/server"
echo ""
echo -e "${BLUE}Terminal 2 - Manager:${NC}"
echo "  cd $PROJECT_ROOT && ./bin/manager"
echo ""
echo -e "${BLUE}Terminal 3 - Agent:${NC}"
echo "  cd $PROJECT_ROOT && export ROBOT_NAME=robot-001 && ./bin/agent"
echo ""
echo -e "${BLUE}Terminal 4 - 创建测试 Job:${NC}"
echo "  kubectl apply -f $PROJECT_ROOT/examples/test-taskgroup-simple.yaml"
echo ""
echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}监控命令（可选）:${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""
echo -e "${BLUE}查看 TaskGroup 状态:${NC}"
echo "  watch -n 1 'kubectl get taskgroup'"
echo ""
echo -e "${BLUE}监控 MQTT 消息:${NC}"
echo "  # TaskGroup 下发"
echo "  mosquitto_sub -h localhost -p 1883 -t 'robot/+/taskgroup' -v"
echo "  # TaskGroup 状态"
echo "  mosquitto_sub -h localhost -p 1883 -t 'robot/+/taskgroup/status' -v"
echo ""
echo -e "${BLUE}查看 Job 和 TaskGroup:${NC}"
echo "  kubectl get job,taskgroup"
echo ""
echo -e "${YELLOW}========================================${NC}"
echo -e "${YELLOW}清理命令:${NC}"
echo -e "${YELLOW}========================================${NC}"
echo ""
echo "  # 删除测试 Job"
echo "  kubectl delete job test-taskgroup-simple"
echo ""
echo "  # 清理测试文件"
echo "  rm -rf /tmp/k8s4r-taskgroup-test"
echo ""
echo "  # 停止守护进程"
echo "  pkill -f 'Background service'"
echo ""
echo -e "${GREEN}准备就绪！请按照上述说明启动各组件。${NC}"
echo ""
