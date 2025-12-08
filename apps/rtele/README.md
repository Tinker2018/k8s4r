# RTele - Remote Teleoperation Application

## 设计
基于火山云veRTC设计，实现机器人远程遥控功能。

## 构建

本项目使用 Bazel 和 Bzlmod 进行构建。

### 前置要求

- Bazel 7.0.0+
- C++17 编译器
- VolcEngineRTC SDK (Linux x86_64)

### 构建步骤

```bash
# 在 x86_64 平台构建
bazel build --config=x86_64 //src:rtele

# 在 ARM64/aarch64 平台构建
bazel build --config=arm64 //src:rtele

# 或者让 Bazel 自动检测平台（使用默认配置）
bazel build //src:rtele

# 运行程序
bazel run //src:rtele -- \
  --room_id=room001 \
  --user_id=user001 \
  --app_id=your_app_id \
  --app_key=your_app_key \
  --enable_publish_video=true
```

### 平台支持

项目支持以下平台：
- **x86_64**: 使用 VolcEngineRTC_Linux_x86_64
- **aarch64 (ARM64)**: 使用 VolcEngineRTC_Linux_3.58.1.500_aarch64

Bazel 会根据 `--config` 参数自动选择对应的 SDK：
```bash
# 显式指定 x86_64 平台
bazel build --config=x86_64 //src:rtele

# 显式指定 ARM64 平台
bazel build --config=arm64 //src:rtele
```

### 开发构建

```bash
# Debug 模式
bazel build --config=dbg //src:rtele

# Optimized 模式
bazel build --config=opt //src:rtele

# 构建所有目标
bazel build //...

# 查看构建结构指南
cat BUILD_GUIDE.md
```

## 使用方法

### 临时测试
不同用户生成不同的临时token，在火山云veRTC控制台生成临时token，需要手动生成（在控制台指定UserID，RoomID，有效期等参数）

### 日产使用
不同用户通过调用相关服务API，申请Token使用，目前我们还不支持

## 基础概念
参考https://www.volcengine.com/docs/6348/70120?lang=zh
需要理解RoomID，UserID（不能重复），推流拉流，点对点，广播消息的概念

音视频流的限制参考https://www.volcengine.com/docs/6348/257549?lang=zh

## 运行前置条件
参考https://www.volcengine.com/docs/6348/141350?lang=zh，
源码里面的SDK版本，VolcEngineRTC_Linux_3.60.102.4700_x86_64_Release

## 命令行参数

### 必需参数

- `--room_id`: 房间 ID
- `--user_id`: 用户 ID
- `--app_id`: 应用 ID (从控制台获取)
- `--app_key`: 应用密钥 (从控制台获取)

### 可选参数

- `--enable_video`: 是否启用视频 (默认: true)

## 目录结构

- `deploy/` - K8s4r deployment configurations
- `third_party/` - Third-party libraries (VolcEngineRTC SDK)
- `main.cc` - Application entry point
- `BUILD.bazel` - Bazel build configuration
- `MODULE.bazel` - Bazel module configuration

## 部署

将应用部署到 K8s4r 集群：

```bash
kubectl apply -f deploy/
```
