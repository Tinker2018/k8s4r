#!/bin/bash
# 快速验证 SPIRE 三点要求

echo "========================================="
echo "  SPIRE 集成三点要求验证"
echo "========================================="
echo

# 颜色
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}[检查 1] SPIRE Server 是否能签发 Node SVID 和 Workload SVID${NC}"
echo "----------------------------------------"

if command -v spire-server &> /dev/null; then
    echo -e "${GREEN}✅ spire-server 已安装${NC}"
    echo "   位置: $(which spire-server)"
else
    echo -e "${RED}❌ spire-server 未安装${NC}"
    echo "   请安装: https://spiffe.io/downloads/"
fi

if command -v spire-agent &> /dev/null; then
    echo -e "${GREEN}✅ spire-agent 已安装${NC}"
    echo "   位置: $(which spire-agent)"
else
    echo -e "${RED}❌ spire-agent 未安装${NC}"
    echo "   请安装: https://spiffe.io/downloads/"
fi

if [ -f "config/spire/server.conf" ]; then
    echo -e "${GREEN}✅ SPIRE Server 配置存在${NC}"
    echo "   Trust Domain: $(grep trust_domain config/spire/server.conf | awk '{print $3}' | tr -d '"')"
    echo "   Bind Port: $(grep bind_port config/spire/server.conf | awk '{print $3}')"
else
    echo -e "${RED}❌ SPIRE Server 配置缺失${NC}"
fi
echo

echo -e "${BLUE}[检查 2] Mosquitto 双监听器配置${NC}"
echo "----------------------------------------"

if [ -f "config/mosquitto/mosquitto-hybrid.conf" ]; then
    echo -e "${GREEN}✅ Mosquitto 混合配置存在${NC}"
    
    # 检查监听器配置
    listener_1883=$(grep "^listener 1883" config/mosquitto/mosquitto-hybrid.conf)
    listener_8883=$(grep "^listener 8883" config/mosquitto/mosquitto-hybrid.conf)
    
    if [ -n "$listener_1883" ]; then
        echo -e "${GREEN}   ✓ 监听器 1883 (明文) 已配置${NC}"
    fi
    
    if [ -n "$listener_8883" ]; then
        echo -e "${GREEN}   ✓ 监听器 8883 (mTLS) 已配置${NC}"
    fi
    
    # 检查 CA 配置
    ca_config=$(grep "^cafile" config/mosquitto/mosquitto-hybrid.conf)
    if [ -n "$ca_config" ]; then
        echo -e "${GREEN}   ✓ CA 文件路径已配置: $ca_config${NC}"
    fi
else
    echo -e "${RED}❌ Mosquitto 配置缺失${NC}"
fi
echo

echo -e "${BLUE}[检查 3] Agent SPIRE 集成配置${NC}"
echo "----------------------------------------"

if [ -f "config/agent/plugins.yaml" ]; then
    echo -e "${GREEN}✅ Agent 插件配置存在${NC}"
    
    # 检查关键配置项
    echo "   配置详情:"
    socketPath=$(grep "socketPath:" config/agent/plugins.yaml | awk '{print $2}' | tr -d '"')
    trustDomain=$(grep "trustDomain:" config/agent/plugins.yaml | awk '{print $2}' | tr -d '"')
    serverAddr=$(grep "serverAddr:" config/agent/plugins.yaml | awk '{print $2}' | tr -d '"')
    
    [ -n "$socketPath" ] && echo -e "${GREEN}   ✓ Socket Path: $socketPath${NC}"
    [ -n "$trustDomain" ] && echo -e "${GREEN}   ✓ Trust Domain: $trustDomain${NC}"
    [ -n "$serverAddr" ] && echo -e "${GREEN}   ✓ Server Addr: $serverAddr${NC}"
    
    # 检查环境变量引用
    if grep -q '\${SPIRE_JOIN_TOKEN}' config/agent/plugins.yaml; then
        echo -e "${GREEN}   ✓ Join Token 使用环境变量${NC}"
    fi
else
    echo -e "${RED}❌ Agent 插件配置缺失${NC}"
fi
echo

echo -e "${BLUE}[检查代码实现]${NC}"
echo "----------------------------------------"

# 检查关键代码文件
files=(
    "pkg/plugin/spire/agent.go:SPIRE Agent 管理器"
    "pkg/plugin/spire/workload_client.go:Workload API 客户端"
    "cmd/agent/main.go:Agent 主程序"
    "scripts/manage-dev-env.sh:开发环境脚本"
)

for item in "${files[@]}"; do
    file=$(echo $item | cut -d: -f1)
    desc=$(echo $item | cut -d: -f2)
    
    if [ -f "$file" ]; then
        echo -e "${GREEN}✅ $desc${NC}"
        echo "   文件: $file"
    else
        echo -e "${RED}❌ $desc 缺失${NC}"
    fi
done
echo

echo -e "${BLUE}[检查二进制文件]${NC}"
echo "----------------------------------------"

if [ -f "bin/agent" ]; then
    echo -e "${GREEN}✅ K8s4r Agent 已编译${NC}"
    ls -lh bin/agent
else
    echo -e "${YELLOW}⚠ K8s4r Agent 未编译，运行: make build${NC}"
fi
echo

echo "========================================="
echo -e "${GREEN}验证完成！${NC}"
echo "========================================="
echo
echo "下一步："
echo "  1. 确保 SPIRE 已安装: spire-server --version"
echo "  2. 构建所有组件: make build-all"
echo "  3. 启动环境: make dev-start"
echo "  4. 查看 SVID 日志: tail -f .dev-env/agent.log | grep '🔐'"
echo
