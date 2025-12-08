#include <csignal>
#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <chrono>
#include <unistd.h>
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/strings/str_format.h"
#include "src/config/rtc_config.h"
#include "src/rtc/rtc_engine.h"
#include "src/util/video_file_reader.h"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "spdlog/spdlog.h"

ABSL_FLAG(std::string, room_id, "", "Room ID for RTC session (required)");
ABSL_FLAG(std::string, user_id, "", "User ID for authentication (required)");
ABSL_FLAG(std::string, app_id, "", "Application ID from VolcEngine RTC console (required)");
ABSL_FLAG(std::string, app_key, "", "Application Key from VolcEngine RTC console (required)");

ABSL_FLAG(std::string, token, "", "Pre-shared Key from VolcEngine RTC console, used to generate token (required)");

ABSL_FLAG(std::string, stream_config_path, "", "Path for stream config (optional)");

ABSL_FLAG(std::string, subscribe_room_id, "", "Room ID to subscribe to (optional, defaults to own room_id)");
ABSL_FLAG(std::string, subscribe_user_id, "", "User ID to subscribe to (optional, subscribe to specific user's stream)");


ABSL_FLAG(bool, enable_publish_video, true, "Enable publishing video stream (default: true)");
ABSL_FLAG(bool, enable_publish_audio, false, "Enable publishing audio stream (default: false)");

ABSL_FLAG(std::string, video_file_path, "", "Path for stream video file (optional)");
// 设备选择
ABSL_FLAG(int, video_device_index, -1, "Video device index (optional, -1 for auto-select first device)");
ABSL_FLAG(int, audio_device_index, -1, "Audio device index (optional, -1 for auto-select first device)");

// 视频配置
ABSL_FLAG(int, video_width, 1280, "Video width in pixels (default: 1280)");
ABSL_FLAG(int, video_height, 720, "Video height in pixels (default: 720)");
ABSL_FLAG(int, video_fps, 30, "Video frame rate (default: 30)");
ABSL_FLAG(int, video_bitrate, 3000, "Video bitrate in kbps (default: 3000)");
ABSL_FLAG(std::string, video_codec, "H264", "Video codec: H264, H265, VP8, VP9 (default: H264)");
ABSL_FLAG(int, video_max_bitrate, 5000, "Max video bitrate in kbps (default: 5000)");
ABSL_FLAG(int, video_min_bitrate, 1000, "Min video bitrate in kbps (default: 1000)");
ABSL_FLAG(std::string, video_pixel_format, "I420", "Video pixel format: I420, NV12, RGBA (default: I420)");

// 音频配置
ABSL_FLAG(int, audio_sample_rate, 48000, "Audio sample rate: 8000, 16000, 32000, 44100, 48000 (default: 48000)");
ABSL_FLAG(int, audio_channels, 2, "Audio channels: 1=mono, 2=stereo (default: 2)");
ABSL_FLAG(int, audio_bitrate, 128, "Audio bitrate in kbps (default: 128)");
ABSL_FLAG(std::string, audio_codec, "OPUS", "Audio codec: OPUS, AAC (default: OPUS)");

bool ValidateRequiredFlags() {
    bool valid = true;
    
    if (absl::GetFlag(FLAGS_room_id).empty()) {
        spdlog::error("Error: --room_id is required");
        valid = false;
    }
    if (absl::GetFlag(FLAGS_user_id).empty()) {
        spdlog::error("Error: --user_id is required");
        valid = false;
    }
    if (absl::GetFlag(FLAGS_app_id).empty()) {
        spdlog::error("Error: --app_id is required");
        valid = false;
    }
    if (absl::GetFlag(FLAGS_app_key).empty()) {
        spdlog::error("Error: --app_key is required");
        valid = false;
    }
    
    return valid;
}

// 从命令行参数构建配置
rtele::config::RTCConfig BuildConfigFromFlags() {
    rtele::config::RTCConfig config;
    
    // 基本配置
    config.app_id = absl::GetFlag(FLAGS_app_id);
    config.app_key = absl::GetFlag(FLAGS_app_key);
    config.room_id = absl::GetFlag(FLAGS_room_id);
    config.user_id = absl::GetFlag(FLAGS_user_id);
    config.token = absl::GetFlag(FLAGS_token);
    
    // 订阅配置
    config.subscribe_room_id = absl::GetFlag(FLAGS_subscribe_room_id);
    config.subscribe_user_id = absl::GetFlag(FLAGS_subscribe_user_id);
    
    // 设备配置
    config.video_file_path = absl::GetFlag(FLAGS_video_file_path);
    config.video_device_index = absl::GetFlag(FLAGS_video_device_index);
    config.audio_device_index = absl::GetFlag(FLAGS_audio_device_index);
    
    // 视频配置
    config.video_width = absl::GetFlag(FLAGS_video_width);
    config.video_height = absl::GetFlag(FLAGS_video_height);
    config.video_fps = absl::GetFlag(FLAGS_video_fps);
    config.video_bitrate = absl::GetFlag(FLAGS_video_bitrate);
    config.video_codec = absl::GetFlag(FLAGS_video_codec);
    config.video_max_bitrate = absl::GetFlag(FLAGS_video_max_bitrate);
    config.video_min_bitrate = absl::GetFlag(FLAGS_video_min_bitrate);
    config.video_pixel_format = absl::GetFlag(FLAGS_video_pixel_format);
    
    // 音频配置
    config.audio_sample_rate = absl::GetFlag(FLAGS_audio_sample_rate);
    config.audio_channels = absl::GetFlag(FLAGS_audio_channels);
    config.audio_bitrate = absl::GetFlag(FLAGS_audio_bitrate);
    config.audio_codec = absl::GetFlag(FLAGS_audio_codec);
    
    // 根据 codec 构造 RTC param JSON
    config.param = config.BuildParamJSON();
    
    return config;
}



// 全局变量用于信号处理
std::atomic<bool> g_shutdown_requested{false};

// 信号处理函数
void SignalHandler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        spdlog::info("Received signal {}, initiating graceful shutdown...", signal);
        g_shutdown_requested.store(true);
    }
}

// 将I420格式转换为BGR（用于保存JPG）
cv::Mat ConvertI420ToBGR(const std::vector<uint8_t>& i420_data, int width, int height) {
    int y_size = width * height;
    int uv_size = y_size / 4;
    
    cv::Mat yuv_mat(height + height / 2, width, CV_8UC1);
    memcpy(yuv_mat.data, i420_data.data(), y_size);
    memcpy(yuv_mat.data + y_size, i420_data.data() + y_size, uv_size);
    memcpy(yuv_mat.data + y_size + uv_size, i420_data.data() + y_size + uv_size, uv_size);
    
    cv::Mat bgr_mat;
    cv::cvtColor(yuv_mat, bgr_mat, cv::COLOR_YUV2BGR_I420);
    return bgr_mat;
}

// 将BGR转换为I420格式
std::vector<uint8_t> ConvertBGRToI420(const cv::Mat& bgr_mat) {
    int width = bgr_mat.cols;
    int height = bgr_mat.rows;
    int y_size = width * height;
    int uv_size = y_size / 4;
    
    cv::Mat yuv_mat;
    cv::cvtColor(bgr_mat, yuv_mat, cv::COLOR_BGR2YUV_I420);
    
    std::vector<uint8_t> i420_data(y_size + uv_size * 2);
    memcpy(i420_data.data(), yuv_mat.data, y_size + uv_size * 2);
    
    return i420_data;
}



// 视频发送线程函数 - 读取JPG并转换为I420发送
void VideoPublishThread(std::atomic<bool>* running,
                       rtele::rtc::RTCEngine* rtc_engine,
                       const rtele::config::RTCConfig& config) {
    spdlog::info("[Publish Thread] Starting...");
    
    std::string jpg_path = config.video_file_path;
    if (jpg_path.empty()) {
        jpg_path = "resources/20250812-131830.jpg";
        spdlog::info("No video file specified, using default: {}", jpg_path);
    }
    
    // 读取JPG图片
    cv::Mat bgr_mat = cv::imread(jpg_path, cv::IMREAD_COLOR);
    if (bgr_mat.empty()) {
        spdlog::error("Failed to load image: {}", jpg_path);
        return;
    }
    
    spdlog::info("✓ Image loaded: {} ({}x{})", jpg_path, bgr_mat.cols, bgr_mat.rows);
    
    // 调整图片尺寸到配置的分辨率
    if (bgr_mat.cols != config.video_width || bgr_mat.rows != config.video_height) {
        cv::Mat resized;
        cv::resize(bgr_mat, resized, cv::Size(config.video_width, config.video_height));
        bgr_mat = resized;
        spdlog::info("Image resized to {}x{}", config.video_width, config.video_height);
    }
    
    // 转换为I420格式
    std::vector<uint8_t> i420_data = ConvertBGRToI420(bgr_mat);
    spdlog::info("Image converted to I420 format, size: {} bytes", i420_data.size());
    
    int frame_count = 0;
    
    spdlog::info("[Publish Thread] Started, sending 1 frame per second");
    
    while (*running) {
        auto start_time = std::chrono::steady_clock::now();
        
        // 推送同一张图片作为视频帧
        if (!rtc_engine->PushVideoFrame(i420_data)) {
            // 错误已在 PushVideoFrame 中记录
        } else {
            frame_count++;
            
            // 获取当前时间戳（毫秒）
            auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            
            // 发送广播消息，包含帧号和时间戳
            std::string message = absl::StrFormat("Frame #%d sent at timestamp: %lld ms", 
                                                  frame_count, timestamp_ms);
            rtc_engine->SendRoomMessage(message);
            
            spdlog::info("[Publish-TX] Frame #{}: {}x{}, timestamp: {} ms, broadcast sent", 
                        frame_count, config.video_width, config.video_height, timestamp_ms);
        }
        
        // 休睠1秒
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    spdlog::info("[Publish Thread] Stopped (total frames: {})", frame_count);
}

int main(int argc, char* argv[]) {
    absl::ParseCommandLine(argc, argv);

    if (!ValidateRequiredFlags()) {
        spdlog::error("Error: room_id, user_id, app_id, app_key is required");
        return -1;
    }
    
    // 注册信号处理器
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);
    spdlog::info("Signal handlers registered (SIGINT, SIGTERM)");
    
    auto config = BuildConfigFromFlags();

    // 创建 RTC 引擎
    std::unique_ptr<rtele::rtc::RTCEngine> rtc_engine = 
        std::make_unique<rtele::rtc::RTCEngine>(config);
    
    // 创建 VideoFrameConsumer 用于接收和保存视频帧
    const std::string output_path = "/tmp/received_frame.jpg";
    spdlog::info("Will save received frames to: {}", output_path);
    
    auto video_consumer = std::make_shared<rtele::rtc::VideoFrameConsumer>(
        config.subscribe_user_id.empty() ? "remote_user" : config.subscribe_user_id,
        [output_path](std::shared_ptr<rtele::rtc::VideoFrameData> frame_data) {
            static std::atomic<int> processed_frames{0};
            static auto last_report_time = std::chrono::steady_clock::now();
            
            int count = processed_frames.fetch_add(1);
            
            // 将I420转换为BGR并保存为JPG
            try {
                cv::Mat bgr_mat = ConvertI420ToBGR(frame_data->data, frame_data->width, frame_data->height);
                
                if (cv::imwrite(output_path, bgr_mat)) {
                    if (count % 30 == 0) {
                        spdlog::info("[VideoConsumer-RX] Frame #{}: {}x{} from user {}, saved to {}", 
                                    count, frame_data->width, frame_data->height, 
                                    frame_data->user_id, output_path);
                    }
                } else {
                    spdlog::error("[VideoConsumer-RX] Failed to save frame #{}", count);
                }
            } catch (const cv::Exception& e) {
                spdlog::error("[VideoConsumer-RX] OpenCV error: {}", e.what());
            }
            
            // 每5秒报告一次统计
            auto now_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now_time - last_report_time).count();
            if (elapsed >= 5 && count > 0) {
                double fps = count / static_cast<double>(elapsed);
                spdlog::info("[VideoConsumer] Received FPS: {:.2f}, total frames: {}", fps, count);
                last_report_time = now_time;
                processed_frames = 0;
            }
        }
    );
    
    // 创建 MessageConsumer 用于接收房间消息
    auto message_consumer = std::make_shared<rtele::rtc::MessageConsumer>(
        // 二进制消息回调
        [](std::shared_ptr<rtele::rtc::MessageData> msg) {
            spdlog::info("[MessageConsumer-RX] Binary message from user '{}': {} bytes, timestamp: {} ms", 
                        msg->user_id, msg->GetSize(), msg->timestamp_ms);
        },
        // 文本消息回调
        [](std::shared_ptr<rtele::rtc::MessageData> msg) {
            spdlog::info("[MessageConsumer-RX] Text message from user '{}': '{}', timestamp: {} ms", 
                        msg->user_id, msg->AsString(), msg->timestamp_ms);
        }
    );
    
    // 注册 Consumer（如果指定了订阅用户）
    if (!config.subscribe_user_id.empty()) {
        rtc_engine->RegisterVideoConsumer(config.subscribe_user_id, video_consumer);
        spdlog::info("Registered VideoConsumer for user: {}", config.subscribe_user_id);
    } else {
        spdlog::warn("No subscribe_user_id specified, will not receive video frames");
    }
    rtc_engine->RegisterMessageConsumer(message_consumer);
    spdlog::info("Registered MessageConsumer");
    
    if (!rtc_engine->Initialize()) {
        spdlog::error("Failed to initialize RTC engine");
        return 1;
    }
    
    if (!rtc_engine->JoinRoom()) {
        spdlog::error("Failed to join room");
        return 1;
    }
    
    std::atomic<bool> publish_thread_running{false};
    std::unique_ptr<std::thread> publish_thread;
    
    if (absl::GetFlag(FLAGS_enable_publish_video)||absl::GetFlag(FLAGS_enable_publish_audio)) {
        spdlog::info("Publishing enabled, starting publish from video file");
        if (!rtc_engine->StartPublish()) {
            spdlog::error("Failed to start publishing");
        } else {
            publish_thread_running = true;
            publish_thread = std::make_unique<std::thread>(
                VideoPublishThread, &publish_thread_running, rtc_engine.get(), std::ref(config));
        }
    } else {
        spdlog::info("Publishing disabled, running in subscribe-only mode");
    }

    // 主循环
    while (!g_shutdown_requested) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    
    // 收到关闭信号或窗口关闭，开始清理
    if (g_shutdown_requested) {
        spdlog::info("Shutdown signal received, cleaning up...");
    } else {
        spdlog::info("Display window closed, cleaning up...");
    }
    
    // 停止发布线程
    if (publish_thread) {
        publish_thread_running = false;
    }

    // 先停止RTC推流和订阅
    if (rtc_engine) {
        spdlog::info("Stopping RTC publish and subscribe...");
        rtc_engine->StopSubscribe();
        rtc_engine->StopPublish();
    }
    
    // 等待发布线程结束
    if (publish_thread) {
        spdlog::info("Waiting for publish thread to stop...");
        if (publish_thread->joinable()) {
            publish_thread->join();
        }
        spdlog::info("Publish thread stopped");
    }
    
    // 销毁RTC引擎
    if (rtc_engine) {
        spdlog::info("Destroying RTC engine...");
        rtc_engine->Destroy();
        rtc_engine.reset();
        spdlog::info("RTC engine destroyed");
    }
    
    spdlog::info("RTele stopped gracefully");
    
    return 0;
}
