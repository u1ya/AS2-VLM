#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <poll.h>
#include <thread>
#include <unistd.h>
#include <unitree/common/json/json.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/go2/video/video_client.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/LidarState_.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>

using unitree::common::JsonMap;
using unitree::common::JsonArray;
double now_s() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
template <typename Values> JsonArray numbers(const Values& values) {
    JsonArray result;
    for (auto v : values) {
        if (!std::isfinite(v)) throw std::runtime_error("Nonfinite telemetry");
        result.emplace_back(double(v));
    }
    return result;
}
int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "Usage: as2_sensor_stream INTERFACE PRIVATE_FRAME_PATH.jpg\n"
                  << "Read-only JSON lines: odometry, lidar metadata and videohub JPEG.\n"
                  << "Close stdin or send any input to quit. No sport client or writers.\n";
        return 0;
    }
    if (argc != 3 || std::filesystem::exists(argv[2])) return 2;
    try {
        std::mutex mutex;
        JsonMap state, lidar;
        double state_time = 0, lidar_time = 0, first_cloud = 0;
        unsigned cloud_count = 0;
        unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
        unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_> odom("rt/lf/sportmodestate");
        unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_> cloud("rt/utlidar/cloud_livox_mid360");
        unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LidarState_> health("rt/utlidar/lidar_state");
        odom.InitChannel([&](const void* raw) {
            const auto& msg = *static_cast<const unitree_go::msg::dds_::SportModeState_*>(raw);
            try {
                JsonMap next{{"position", numbers(msg.position())}, {"velocity", numbers(msg.velocity())},
                             {"rpy", numbers(msg.imu_state().rpy())}, {"mode", int(msg.mode())},
                             {"error", std::to_string(msg.error_code())}};
                std::lock_guard<std::mutex> lock(mutex);
                state = std::move(next); state_time = now_s();
            } catch (const std::exception&) {}
        }, 1);
        cloud.InitChannel([&](const void* raw) {
            const auto& msg = *static_cast<const sensor_msgs::msg::dds_::PointCloud2_*>(raw);
            std::lock_guard<std::mutex> lock(mutex);
            lidar_time = now_s();
            if (cloud_count++ == 0) first_cloud = lidar_time;
            lidar["frame"] = msg.header().frame_id();
            lidar["points"] = double(std::uint64_t(msg.width()) * msg.height());
            lidar["bytes"] = double(msg.data().size());
            lidar["messages"] = int(cloud_count);
            lidar["hz"] = cloud_count > 1 ? (cloud_count - 1) / (lidar_time - first_cloud) : 0.0;
        }, 1);
        health.InitChannel([&](const void* raw) {
            const auto& msg = *static_cast<const unitree_go::msg::dds_::LidarState_*>(raw);
            std::lock_guard<std::mutex> lock(mutex);
            lidar["error"] = int(msg.error_state());
        }, 1);
        unitree::robot::go2::VideoClient video;
        video.SetTimeout(1.0F); video.Init();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        unsigned frame = 0;
        double frame_time = 0, next_camera = 0;
        int camera_code = -1;
        const auto deadline = now_s() + 8 * 3600;
        while (now_s() < deadline) {
            pollfd input{STDIN_FILENO, POLLIN | POLLHUP, 0};
            if (poll(&input, 1, 0) > 0) break;
            if (now_s() >= next_camera) {
                std::vector<uint8_t> bytes;
                camera_code = video.GetImageSample(bytes);
                if (camera_code == 0 && bytes.size() >= 4 && bytes.size() <= 10 * 1024 * 1024 &&
                    bytes[0] == 0xff && bytes[1] == 0xd8 && bytes[2] == 0xff) {
                    const auto temporary = std::string(argv[2]) + ".tmp";
                    std::ofstream file(temporary, std::ios::binary);
                    file.exceptions(std::ios::failbit | std::ios::badbit);
                    file.write(reinterpret_cast<const char*>(bytes.data()), bytes.size()); file.close();
                    std::filesystem::rename(temporary, argv[2]);
                    ++frame; frame_time = now_s();
                }
                next_camera = now_s() + 1.0;
            }
            JsonMap output;
            {
                std::lock_guard<std::mutex> lock(mutex);
                auto state_copy = state, lidar_copy = lidar;
                state_copy["age_s"] = state_time ? now_s() - state_time : -1.0;
                lidar_copy["age_s"] = lidar_time ? now_s() - lidar_time : -1.0;
                output = {{"state", state_copy}, {"lidar", lidar_copy},
                          {"camera", JsonMap{{"sequence", int(frame)}, {"code", camera_code},
                            {"age_s", frame_time ? now_s() - frame_time : -1.0}}}};
            }
            std::cout << unitree::common::ToJsonString(output) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        odom.CloseChannel(); cloud.CloseChannel(); health.CloseChannel();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Sensor stream failed: " << error.what() << '\n'; return 1;
    }
}
