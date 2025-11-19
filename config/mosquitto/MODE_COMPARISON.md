# Mosquitto 配置模式对比

## 四种启动模式对比

| 特性 | minimal | simple | dev | prod |
|------|---------|--------|-----|------|
| **配置文件** | 动态生成 | mosquitto-simple.conf | mosquitto.conf | mosquitto-prod.conf |
| **MQTT端口** | ✅ 1883 | ✅ 1883 | ✅ 1883 | ✅ 1883 |
| **WebSocket端口** | ❌ | ❌ | ✅ 9001 | ✅ 9001 |
| **匿名连接** | ✅ | ✅ | ✅ | ✅ (建议生产环境禁用) |
| **日志输出** | stdout | stdout | stdout + 详细 | file + stdout |
| **持久化** | ❌ | ❌ | ❌ | ✅ |
| **连接限制** | 无限制 | 无限制 | 宽松限制 | 严格限制 |
| **消息大小限制** | 默认 | 默认 | 无限制 | 256MB |
| **数据目录** | ❌ | ❌ | ❌ | /tmp/k8s4r-mosquitto/data |
| **日志目录** | ❌ | ❌ | ❌ | /tmp/k8s4r-mosquitto/log |
| **用途** | 解决连接问题 | 基础开发 | 完整开发 | 生产部署 |

## 使用建议

### 🔧 **开发调试推荐顺序**

1. **minimal** - 如果遇到连接问题，先用这个模式排查
2. **simple** - 日常开发使用，功能够用且配置简单
3. **dev** - 需要WebSocket或详细日志时使用
4. **prod** - 生产环境或完整功能测试

### 📋 **具体场景**

| 场景 | 推荐模式 | 原因 |
|------|----------|------|
| 初次连接测试 | `minimal` | 最小配置，排除配置问题 |
| 日常开发 | `simple` | 功能够用，启动快速 |
| Web界面开发 | `dev` | 需要WebSocket支持 |
| 性能测试 | `dev` | 需要详细日志监控 |
| 生产部署 | `prod` | 完整安全配置 |
| CI/CD测试 | `simple` | 快速启动，功能完整 |

### 🚀 **快速启动命令**

```bash
# 解决连接问题
./config/mosquitto/start-mosquitto.sh minimal

# 日常开发（推荐）
./config/mosquitto/start-mosquitto.sh simple

# 完整开发环境
./config/mosquitto/start-mosquitto.sh dev

# 生产环境
./config/mosquitto/start-mosquitto.sh prod

# 查看状态
./config/mosquitto/start-mosquitto.sh status

# 停止服务
./config/mosquitto/start-mosquitto.sh stop
```

### 🔍 **配置文件详情**

#### minimal模式配置
```conf
listener 1883
allow_anonymous true
log_dest stdout
```

#### simple模式配置  
```conf
listener 1883
allow_anonymous true
log_dest stdout
log_type all
log_timestamp true
persistence false
```

#### dev模式配置
```conf
listener 1883 0.0.0.0
allow_anonymous true
protocol mqtt

listener 9001 0.0.0.0
allow_anonymous true
protocol websockets

log_type all
log_dest stdout
log_timestamp true
connection_messages true
# ... 更多开发友好配置
```

#### prod模式配置
```conf
listener 1883 0.0.0.0
protocol mqtt

listener 9001 0.0.0.0
protocol websockets

allow_anonymous true  # 生产环境建议禁用
log_type error
log_type warning  
log_type notice
log_type information
log_dest file /mosquitto/log/mosquitto.log
persistence true
persistence_location /mosquitto/data/
# ... 更多生产环境配置
```

### ⚠️ **注意事项**

1. **端口占用**：确保1883端口未被其他服务占用
2. **Docker权限**：确保有Docker运行权限
3. **配置文件**：所有配置文件都在 `config/mosquitto/` 目录下
4. **生产安全**：生产环境务必配置用户认证和SSL
5. **资源使用**：prod模式会使用更多磁盘空间用于持久化

### 🐛 **故障排除**

| 问题 | 解决方案 |
|------|----------|
| 连接被拒绝 | 使用 `minimal` 模式 |
| 端口占用 | 先执行 `stop` 再启动 |
| 配置错误 | 查看容器日志：`docker logs k8s4r-mosquitto` |
| 权限问题 | 检查Docker是否有权限运行 |