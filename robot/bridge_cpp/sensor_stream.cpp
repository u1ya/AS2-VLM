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
#include <unitree/idl/ros2/Imu_.hpp>
#include "shared_recording.hpp"

using unitree::common::JsonMap;
using unitree::common::JsonArray;
double now_s() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
template <typename Header> JsonMap timing(const Header& header) {
    return {{"host_monotonic_ns", std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count())}, {"frame", header.frame_id()},
        {"stamp_sec", header.stamp().sec()}, {"stamp_nanosec", int(header.stamp().nanosec())}};
}
JsonMap cloud_metadata(const sensor_msgs::msg::dds_::PointCloud2_& cloud) {
    auto meta = timing(cloud.header());
    meta["width"] = double(cloud.width()); meta["height"] = double(cloud.height());
    meta["point_step"] = int(cloud.point_step()); meta["row_step"] = double(cloud.row_step());
    meta["bigendian"] = cloud.is_bigendian();
    JsonArray fields;
    for (const auto& f : cloud.fields()) fields.emplace_back(JsonMap{{"name", f.name()},
        {"offset", int(f.offset())}, {"datatype", int(f.datatype())}, {"count", int(f.count())}});
    meta["fields"] = fields;
    return meta;
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
                  << "Q or stdin EOF quits; RECORD<TAB>SECONDS<TAB>NEW_DIRECTORY records shared callbacks.\n"
                  << "No sport client or motion publishers.\n";
        return 0;
    }
    if (argc != 3 || std::filesystem::exists(argv[2])) return 2;
    try {
        SharedRecording recording;
        std::string recording_error;
        std::mutex mutex;
        JsonMap state, lidar;
        sensor_msgs::msg::dds_::PointCloud2_ latest_cloud;
        double state_time = 0, lidar_time = 0, first_cloud = 0;
        unsigned cloud_count = 0;
        unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
        unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_> odom("rt/lf/sportmodestate");
        unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_> cloud("rt/utlidar/cloud_livox_mid360");
        unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LidarState_> health("rt/utlidar/lidar_state");
        unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::Imu_> imu("rt/utlidar/imu_livox_mid360");
        odom.InitChannel([&](const void* raw) {
            const auto& msg = *static_cast<const unitree_go::msg::dds_::SportModeState_*>(raw);
            try {
                JsonMap next{{"position", numbers(msg.position())}, {"velocity", numbers(msg.velocity())},
                             {"rpy", numbers(msg.imu_state().rpy())}, {"mode", int(msg.mode())},
                             {"error", std::to_string(msg.error_code())}};
                if (recording.active()) {
                    auto row = next;
                    row["stamp_sec"] = msg.stamp().sec(); row["stamp_nanosec"] = int(msg.stamp().nanosec());
                    row["host_monotonic_ns"] = std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count());
                    recording.row(std::move(row), false);
                }
                std::lock_guard<std::mutex> lock(mutex);
                state = std::move(next); state_time = now_s();
            } catch (const std::exception&) {}
        }, 64);
        cloud.InitChannel([&](const void* raw) {
            const auto& msg = *static_cast<const sensor_msgs::msg::dds_::PointCloud2_*>(raw);
            if (msg.data().size() > 16 * 1024 * 1024) return;
            if (recording.active()) recording.cloud(msg, cloud_metadata(msg));
            std::lock_guard<std::mutex> lock(mutex);
            lidar_time = now_s();
            if (cloud_count++ == 0) first_cloud = lidar_time;
            lidar["frame"] = msg.header().frame_id();
            lidar["points"] = double(std::uint64_t(msg.width()) * msg.height());
            lidar["bytes"] = double(msg.data().size());
            lidar["messages"] = int(cloud_count);
            lidar["hz"] = cloud_count > 1 ? (cloud_count - 1) / (lidar_time - first_cloud) : 0.0;
            if (msg.data().size() <= 16 * 1024 * 1024) latest_cloud = msg;
        }, 16);
        imu.InitChannel([&](const void* raw) {
            if (!recording.active()) return;
            try {
                const auto& msg = *static_cast<const sensor_msgs::msg::dds_::Imu_*>(raw);
                auto row = timing(msg.header());
                const auto& a = msg.linear_acceleration(); const auto& g = msg.angular_velocity();
                const auto& q = msg.orientation();
                row["linear_acceleration"] = numbers(std::array<double, 3>{a.x(), a.y(), a.z()});
                row["angular_velocity"] = numbers(std::array<double, 3>{g.x(), g.y(), g.z()});
                row["orientation_xyzw"] = numbers(std::array<double, 4>{q.x(), q.y(), q.z(), q.w()});
                row["orientation_covariance"] = numbers(msg.orientation_covariance());
                recording.row(std::move(row), true);
            } catch (const std::exception&) {}
        }, 256);
        health.InitChannel([&](const void* raw) {
            const auto& msg = *static_cast<const unitree_go::msg::dds_::LidarState_*>(raw);
            std::lock_guard<std::mutex> lock(mutex);
            lidar["error"] = int(msg.error_state());
        }, 1);
        unitree::robot::go2::VideoClient video;
        video.SetTimeout(1.0F); video.Init();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        unsigned frame = 0;
        unsigned written_cloud = 0;
        double frame_time = 0, next_camera = 0;
        int camera_code = -1;
        const auto deadline = now_s() + 8 * 3600;
        while (now_s() < deadline) {
            pollfd input{STDIN_FILENO, POLLIN | POLLHUP, 0};
            if (poll(&input, 1, 0) > 0) {
                std::string command;
                if (!std::getline(std::cin, command) || command == "Q") break;
                try {
                    const auto tab = command.find('\t', 7);
                    if (command.rfind("RECORD\t", 0) != 0 || tab == std::string::npos || command.size() > 4096)
                        throw std::runtime_error("Invalid recording command");
                    const auto duration = command.substr(7, tab - 7);
                    if (duration.empty() || duration.size() > 2 || duration.find_first_not_of("0123456789") != std::string::npos)
                        throw std::runtime_error("Invalid duration");
                    recording.start(command.substr(tab+1), std::stoi(duration));
                    recording_error.clear();
                } catch (const std::exception& e) { recording_error = e.what(); }
            }
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
            sensor_msgs::msg::dds_::PointCloud2_ cloud_copy;
            unsigned cloud_sequence = 0;
            {
                std::lock_guard<std::mutex> lock(mutex);
                auto state_copy = state, lidar_copy = lidar;
                state_copy["age_s"] = state_time ? now_s() - state_time : -1.0;
                lidar_copy["age_s"] = lidar_time ? now_s() - lidar_time : -1.0;
                if (cloud_count != written_cloud) {
                    cloud_copy = latest_cloud;
                    cloud_sequence = cloud_count;
                    lidar_copy["cloud_sequence"] = int(cloud_sequence);
                    lidar_copy["width"] = double(cloud_copy.width());
                    lidar_copy["height"] = double(cloud_copy.height());
                    lidar_copy["point_step"] = int(cloud_copy.point_step());
                    lidar_copy["row_step"] = double(cloud_copy.row_step());
                    lidar_copy["bigendian"] = cloud_copy.is_bigendian();
                    JsonArray fields;
                    for (const auto& field : cloud_copy.fields()) {
                        fields.emplace_back(JsonMap{{"name", field.name()}, {"offset", int(field.offset())},
                            {"datatype", int(field.datatype())}, {"count", int(field.count())}});
                    }
                    lidar_copy["fields"] = fields;
                }
                output = {{"state", state_copy}, {"lidar", lidar_copy},
                          {"camera", JsonMap{{"sequence", int(frame)}, {"code", camera_code},
                            {"age_s", frame_time ? now_s() - frame_time : -1.0}}}};
            }
            if (cloud_sequence) {
                const std::string path = std::string(argv[2]) + ".cloud";
                std::ofstream file(path + ".tmp", std::ios::binary);
                file.exceptions(std::ios::failbit | std::ios::badbit);
                // Sequence header binds the atomically replaced binary to its JSON metadata.
                for (int i = 0; i < 8; ++i) file.put(char((std::uint64_t(cloud_sequence) >> (8 * i)) & 0xff));
                file.write(reinterpret_cast<const char*>(cloud_copy.data().data()), cloud_copy.data().size());
                file.close();
                std::filesystem::rename(path + ".tmp", path);
                written_cloud = cloud_sequence;
            }
            auto status = recording.status();
            status["command_error"] = recording_error;
            output["recording"] = status;
            std::cout << unitree::common::ToJsonString(output) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        odom.CloseChannel(); cloud.CloseChannel(); health.CloseChannel(); imu.CloseChannel();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Sensor stream failed: " << error.what() << '\n'; return 1;
    }
}
