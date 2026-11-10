#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <regex>
#include <thread>
#include <unitree/common/json/json.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/idl/ros2/Imu_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

using unitree::common::JsonMap;
using unitree::common::JsonArray;
using Cloud = sensor_msgs::msg::dds_::PointCloud2_;
volatile std::sig_atomic_t cancelled = 0;
void cancel(int) { cancelled = 1; }
std::string ns_now() {
    return std::to_string(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}
template <typename Header> JsonMap timing(const Header& header) {
    return {{"host_monotonic_ns", ns_now()}, {"frame", header.frame_id()},
            {"stamp_sec", header.stamp().sec()}, {"stamp_nanosec", int(header.stamp().nanosec())}};
}
template <typename Values> JsonArray array(const Values& values) {
    JsonArray result;
    for (auto value : values) {
        if (std::isfinite(value)) result.emplace_back(double(value));
        else result.emplace_back(std::string("nonfinite"));
    }
    return result;
}
template <typename Vec> JsonArray vector(const Vec& v) { return array(std::array<double, 3>{v.x(), v.y(), v.z()}); }
void write_json(std::ostream& output, const JsonMap& value) {
    output << unitree::common::ToJsonString(value) << '\n';
}
int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "Usage: as2_record_sensors INTERFACE NEW_DIRECTORY [SECONDS=8] [IMU_QUEUE=256]\n"
                  << "Read-only cloud/IMU/odometry recording; 3-30 seconds, raw cap 512 MiB.\n"
                  << "No camera capture, sensor configuration or robot commands.\n";
        return 0;
    }
    if (argc < 3 || argc > 5 || !std::regex_match(argv[1], std::regex("[A-Za-z0-9][A-Za-z0-9_.:-]{0,63}"))) return 2;
    try {
        const std::string duration_arg = argc >= 4 ? argv[3] : "8";
        const std::string queue_arg = argc == 5 ? argv[4] : "256";
        if (!std::regex_match(queue_arg, std::regex("[0-9]{1,4}"))) return 2;
        const int imu_queue = std::stoi(queue_arg);
        if (imu_queue < 1 || imu_queue > 4096) return 2;
        if (!std::regex_match(duration_arg, std::regex("[0-9]{1,2}"))) return 2;
        const int seconds = std::stoi(duration_arg);
        const std::filesystem::path root = argv[2];
        if (seconds < 3 || seconds > 30 || std::filesystem::exists(root)) return 2;
        std::filesystem::create_directories(root);
        if (std::filesystem::space(root).available < 600ULL * 1024 * 1024)
            throw std::runtime_error("At least 600 MiB of free storage required");
        std::ofstream clouds_file(root / "clouds.jsonl"), imu_file(root / "imu.jsonl"), odom_file(root / "odometry.jsonl");
        clouds_file.exceptions(std::ios::failbit | std::ios::badbit);
        imu_file.exceptions(std::ios::failbit | std::ios::badbit);
        odom_file.exceptions(std::ios::failbit | std::ios::badbit);
        std::mutex mutex, imu_mutex, odom_mutex;
        struct Pending { Cloud cloud; JsonMap metadata; unsigned sequence = 0; };
        std::deque<Pending> queue;
        std::atomic<bool> running{true}, io_error{false};
        unsigned cloud_seen = 0, dropped = 0, saved = 0, imus = 0, odoms = 0;
        std::uint64_t bytes_saved = 0;
        std::signal(SIGINT, cancel); std::signal(SIGTERM, cancel);
        unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
        unitree::robot::ChannelSubscriber<Cloud> cloud("rt/utlidar/cloud_livox_mid360");
        unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::Imu_> imu("rt/utlidar/imu_livox_mid360");
        unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_> odom("rt/lf/sportmodestate");
        cloud.InitChannel([&](const void* raw) {
            if (!running) return;
            const auto& msg = *static_cast<const Cloud*>(raw);
            std::lock_guard<std::mutex> lock(mutex);
            ++cloud_seen;
            if (queue.size() >= 8 || msg.data().empty() || msg.data().size() > 16 * 1024 * 1024) { ++dropped; return; }
            auto metadata = timing(msg.header());
            metadata["cloud_sequence"] = int(cloud_seen);
            metadata["width"] = double(msg.width()); metadata["height"] = double(msg.height());
            metadata["point_step"] = int(msg.point_step()); metadata["row_step"] = double(msg.row_step());
            metadata["bigendian"] = msg.is_bigendian();
            JsonArray fields;
            for (const auto& f : msg.fields()) fields.emplace_back(JsonMap{{"name", f.name()},
                {"offset", int(f.offset())}, {"datatype", int(f.datatype())}, {"count", int(f.count())}});
            metadata["fields"] = fields;
            queue.push_back({msg, std::move(metadata), cloud_seen});
        }, 16);
        imu.InitChannel([&](const void* raw) {
            if (!running || io_error) return;
            try {
                const auto& msg = *static_cast<const sensor_msgs::msg::dds_::Imu_*>(raw);
                auto row = timing(msg.header());
                row["angular_velocity"] = vector(msg.angular_velocity());
                row["linear_acceleration"] = vector(msg.linear_acceleration());
                const auto& q = msg.orientation();
                row["orientation_xyzw"] = array(std::array<double, 4>{q.x(), q.y(), q.z(), q.w()});
                row["orientation_covariance"] = array(msg.orientation_covariance());
                std::lock_guard<std::mutex> lock(imu_mutex);
                write_json(imu_file, row); ++imus;
            } catch (const std::exception&) { io_error = true; }
        }, imu_queue);
        odom.InitChannel([&](const void* raw) {
            if (!running || io_error) return;
            try {
                const auto& msg = *static_cast<const unitree_go::msg::dds_::SportModeState_*>(raw);
                JsonMap row{{"host_monotonic_ns", ns_now()}, {"stamp_sec", msg.stamp().sec()},
                    {"stamp_nanosec", int(msg.stamp().nanosec())}, {"position", array(msg.position())},
                    {"velocity", array(msg.velocity())}, {"rpy", array(msg.imu_state().rpy())},
                    {"mode", int(msg.mode())}, {"error", std::to_string(msg.error_code())}};
                std::lock_guard<std::mutex> lock(odom_mutex);
                write_json(odom_file, row); ++odoms;
            } catch (const std::exception&) { io_error = true; }
        }, 64);
        const auto start = std::chrono::steady_clock::now();
        bool byte_limit = false;
        while (!cancelled && !io_error && std::chrono::steady_clock::now() - start < std::chrono::seconds(seconds)) {
            Pending next;
            {
                std::lock_guard<std::mutex> lock(mutex);
                if (!queue.empty()) { next = std::move(queue.front()); queue.pop_front(); }
            }
            if (!next.sequence) { std::this_thread::sleep_for(std::chrono::milliseconds(2)); continue; }
            if (bytes_saved + next.cloud.data().size() + 8 > 512ULL * 1024 * 1024) { byte_limit = true; break; }
            const auto filename = "cloud-" + std::to_string(next.sequence) + ".bin";
            std::ofstream binary(root / filename, std::ios::binary);
            binary.exceptions(std::ios::failbit | std::ios::badbit);
            for (int i = 0; i < 8; ++i) binary.put(char((std::uint64_t(next.sequence) >> (8 * i)) & 0xff));
            binary.write(reinterpret_cast<const char*>(next.cloud.data().data()), next.cloud.data().size()); binary.close();
            bytes_saved += next.cloud.data().size() + 8;
            next.metadata["file"] = filename;
            write_json(clouds_file, next.metadata); ++saved;
        }
        running = false;
        cloud.CloseChannel(); imu.CloseChannel(); odom.CloseChannel();
        const unsigned queued_at_stop = queue.size();
        clouds_file.close(); imu_file.close(); odom_file.close();
        JsonMap manifest{{"schema", 1}, {"read_only", true}, {"interface", std::string(argv[1])},
            {"imu_callback_queue_depth", imu_queue},
            {"cloud_callback_queue_depth", 16}, {"odometry_callback_queue_depth", 64},
            {"host_timestamp_semantics", std::string("callback execution after SDK queue; not network arrival")},
            {"requested_seconds", seconds}, {"cloud_messages_seen", int(cloud_seen)}, {"cloud_files", int(saved)},
            {"callback_queue_drops", int(dropped)}, {"queued_at_stop", int(queued_at_stop)},
            {"imu_messages", int(imus)}, {"odometry_messages", int(odoms)}, {"raw_bytes", double(bytes_saved)},
            {"interrupted", bool(cancelled)}, {"io_error", bool(io_error)}, {"byte_limit", byte_limit},
            {"calibrated", false}, {"slam_ready", false}};
        std::ofstream manifest_file(root / "manifest.json");
        manifest_file.exceptions(std::ios::failbit | std::ios::badbit);
        write_json(manifest_file, manifest); manifest_file.close();
        std::cout << unitree::common::ToJsonString(manifest) << std::endl;
        return saved && imus && odoms && !cancelled && !io_error && !byte_limit ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "Recording failed; preserve partial directory: " << error.what() << '\n'; return 1;
    }
}
