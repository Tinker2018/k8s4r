# Mosquitto MQTT Broker 配置

这个目录包含了用于K8s4r项目的Mosquitto MQTT broker配置文件和辅助脚本。

## 文件说明

### 配置文件

- **`mosquitto.conf`** - 开发环境配置
  - 允许匿名连接
  - 启用详细日志
  - 适合开发和调试

- **`mosquitto-prod.conf`** - 生产环境配置
  - 基本的生产环境配置
  - 包含安全建议
  - 支持持久化

### 辅助脚本

- **`start-mosquitto.sh`** - Mosquitto 启动脚本
  - 支持多种启动模式（dev/prod/simple）
  - 自动停止现有容器
  - 包含状态检查功能

- **`test-mqtt.sh`** - MQTT 消息测试脚本
  - 测试MQTT连接
  - 发送/接收K8s4r消息
  - 监听和调试工具

## 快速使用

### 1. 启动 Mosquitto Broker

```bash
# 开发环境（推荐用于调试）
./config/mosquitto/start-mosquitto.sh dev

# 简单模式（使用 Mosquitto 1.6，无配置问题）
./config/mosquitto/start-mosquitto.sh simple

# 生产环境
./config/mosquitto/start-mosquitto.sh prod

# 查看状态
./config/mosquitto/start-mosquitto.sh status

# 停止服务
./config/mosquitto/start-mosquitto.sh stop
```

### 2. 测试 MQTT 连接

```bash
# 基本连接测试
./config/mosquitto/test-mqtt.sh test

# 监听所有K8s4r消息
./config/mosquitto/test-mqtt.sh listen

# 发送测试注册消息
./config/mosquitto/test-mqtt.sh register robot-001

# 发送测试心跳
./config/mosquitto/test-mqtt.sh heartbeat robot-001
```

### 3. 启动 K8s4r 组件

```bash
# 确保 Mosquitto 正在运行
./config/mosquitto/start-mosquitto.sh status

# 启动 Server
./bin/server --broker-url=tcp://localhost:1883 --namespace=default

# 启动 Agent  
./bin/agent --broker-url=tcp://localhost:1883 --robot-id=robot-001 --token=fixed-token-123
```

## 故障排除

### 问题1：连接被拒绝或立即断开

**解决方案：**
```bash
# 使用简单模式启动
./config/mosquitto/start-mosquitto.sh simple

# 或者重新启动开发环境
./config/mosquitto/start-mosquitto.sh dev
```

### 问题2：权限问题

**解决方案：**
```bash
# 确保脚本有执行权限
chmod +x ./config/mosquitto/*.sh

# 检查Docker权限
docker ps
```

### 问题3：端口占用

**解决方案：**
```bash
# 检查端口占用
lsof -i :1883

# 停止现有服务
./config/mosquitto/start-mosquitto.sh stop

# 重新启动
./config/mosquitto/start-mosquitto.sh dev
```

### 问题4：无法连接到Docker

**解决方案：**
```bash
# 检查Docker状态
docker version

# 启动Docker服务（如果需要）
# macOS: 启动 Docker Desktop
# Linux: sudo systemctl start docker
```

## MQTT主题结构

K8s4r使用以下MQTT主题：

- `k8s4r/register` - Agent注册请求
- `k8s4r/heartbeat` - Agent心跳消息  
- `k8s4r/response/{robotId}` - Server响应消息
- `k8s4r/commands/{robotId}` - Server命令消息

## 监控和调试

### 实时监控所有消息

```bash
# 监听所有K8s4r主题
./config/mosquitto/test-mqtt.sh listen

# 或者使用mosquitto客户端
mosquitto_sub -h localhost -p 1883 -t "k8s4r/#" -v
```

### 手动发送消息

```bash
# 发送注册消息
./config/mosquitto/test-mqtt.sh register my-robot

# 发送心跳消息
./config/mosquitto/test-mqtt.sh heartbeat my-robot

# 发送命令
./config/mosquitto/test-mqtt.sh command my-robot restart
```

### 查看容器日志

```bash
# 查看Mosquitto日志
docker logs k8s4r-mosquitto

# 实时查看日志
docker logs -f k8s4r-mosquitto
```

## 生产环境部署

在生产环境中，建议：

1. 启用用户认证
2. 配置SSL/TLS加密
3. 设置适当的ACL（访问控制列表）
4. 使用持久化存储
5. 配置日志轮转

详细的生产环境配置请参考 `mosquitto-prod.conf` 文件中的注释。