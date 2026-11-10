#pragma once
#include <atomic>
#include <chrono>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>
#include <unitree/common/json/json.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>

// One bounded disk-writer consumer of the existing acquisition callbacks.
class SharedRecording {
    using Map = unitree::common::JsonMap;
    using Cloud = sensor_msgs::msg::dds_::PointCloud2_;
    struct Pending { Cloud cloud; Map metadata; unsigned sequence; };
    std::mutex mutex_;
    std::deque<Pending> clouds_;
    std::deque<Map> imu_, odom_;
    std::thread writer_;
    std::atomic<bool> active_{false}, quit_{false};
    std::chrono::steady_clock::time_point deadline_;
    unsigned seen_ = 0, dropped_ = 0, row_drops_ = 0;
    Map status_{{"active", false}};

    static void line(std::ostream& stream, const Map& row) {
        stream << unitree::common::ToJsonString(row) << '\n';
    }
    void run(std::filesystem::path root, int seconds) {
        unsigned saved = 0, imus = 0, odoms = 0;
        std::uint64_t bytes = 0;
        bool limited = false;
        std::string error;
        try {
            std::ofstream cf(root / "clouds.jsonl"), im(root / "imu.jsonl"), od(root / "odometry.jsonl");
            for (auto* stream : {&cf, &im, &od}) stream->exceptions(std::ios::failbit | std::ios::badbit);
            while (true) {
                std::deque<Pending> clouds;
                std::deque<Map> imu, odom;
                bool done;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    done = quit_ || std::chrono::steady_clock::now() >= deadline_;
                    if (done) active_ = false;
                    clouds.swap(clouds_); imu.swap(imu_); odom.swap(odom_);
                }
                for (const auto& row : imu) { line(im, row); ++imus; }
                for (const auto& row : odom) { line(od, row); ++odoms; }
                for (auto& pending : clouds) {
                    if (bytes + pending.cloud.data().size() + 8 > 512ULL * 1024 * 1024) {
                        limited = true; done = true; break;
                    }
                    const auto name = "cloud-" + std::to_string(pending.sequence) + ".bin";
                    std::ofstream binary(root / name, std::ios::binary);
                    binary.exceptions(std::ios::failbit | std::ios::badbit);
                    for (int i = 0; i < 8; ++i) binary.put(char((std::uint64_t(pending.sequence) >> (8*i)) & 255));
                    binary.write(reinterpret_cast<const char*>(pending.cloud.data().data()), pending.cloud.data().size());
                    binary.close();
                    pending.metadata["file"] = name;
                    line(cf, pending.metadata);
                    bytes += pending.cloud.data().size() + 8; ++saved;
                }
                if (done) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            cf.close(); im.close(); od.close();
        } catch (const std::exception& e) { error = e.what(); }
        active_ = false;
        std::lock_guard<std::mutex> lock(mutex_);
        Map manifest{{"schema", 1}, {"read_only", true}, {"shared_acquisition", true},
            {"requested_seconds", seconds}, {"cloud_files", int(saved)}, {"cloud_messages_seen", int(seen_)},
            {"imu_messages", int(imus)}, {"odometry_messages", int(odoms)}, {"raw_bytes", double(bytes)},
            {"callback_queue_drops", int(dropped_)}, {"row_queue_drops", int(row_drops_)},
            {"queued_at_stop", int(clouds_.size())}, {"interrupted", bool(quit_)},
            {"byte_limit", limited}, {"io_error", !error.empty()}, {"error", error},
            {"host_timestamp_semantics", std::string("callback execution after SDK queue; not network arrival")},
            {"imu_callback_queue_depth", 256}, {"cloud_callback_queue_depth", 16},
            {"odometry_callback_queue_depth", 64}, {"calibrated", false}, {"slam_ready", false}};
        try {
            std::ofstream file(root / "manifest.json");
            file.exceptions(std::ios::failbit | std::ios::badbit);
            line(file, manifest); file.close();
        } catch (const std::exception& e) { error = e.what(); }
        status_ = {{"active", false}, {"finished", true}, {"name", root.filename().string()},
                   {"cloud_files", int(saved)}, {"imu_messages", int(imus)}, {"error", error},
                   {"ok", error.empty() && !limited && !quit_ && saved > 0 && imus > 0 && odoms > 0}};
        clouds_.clear(); imu_.clear(); odom_.clear();
    }
public:
    ~SharedRecording() { quit_ = true; if (writer_.joinable()) writer_.join(); }
    bool active() const { return active_; }
    Map status() { std::lock_guard<std::mutex> lock(mutex_); return status_; }
    void start(const std::filesystem::path& root, int seconds) {
        if (active_) throw std::runtime_error("Recording already active");
        if (writer_.joinable()) writer_.join();
        if (seconds < 3 || seconds > 30 || std::filesystem::exists(root))
            throw std::runtime_error("New directory and duration 3-30 required");
        std::filesystem::create_directories(root);
        if (std::filesystem::space(root).available < 600ULL * 1024 * 1024)
            throw std::runtime_error("At least 600 MiB free storage required");
        std::lock_guard<std::mutex> lock(mutex_);
        seen_ = dropped_ = row_drops_ = 0; quit_ = false;
        deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
        status_ = {{"active", true}, {"finished", false}, {"name", root.filename().string()}, {"error", std::string("")}};
        active_ = true;
        writer_ = std::thread([this, root, seconds] { run(root, seconds); });
    }
    void cloud(const Cloud& cloud, Map metadata) {
        if (!active_) return;
        std::lock_guard<std::mutex> lock(mutex_);
        if (!active_ || std::chrono::steady_clock::now() >= deadline_) return;
        ++seen_;
        if (clouds_.size() >= 16 || cloud.data().empty() || cloud.data().size() > 16 * 1024 * 1024) { ++dropped_; return; }
        metadata["cloud_sequence"] = int(seen_);
        clouds_.push_back({cloud, std::move(metadata), seen_});
    }
    void row(Map row, bool imu) {
        if (!active_) return;
        std::lock_guard<std::mutex> lock(mutex_);
        if (!active_ || std::chrono::steady_clock::now() >= deadline_) return;
        auto& rows = imu ? imu_ : odom_;
        if (rows.size() >= 10000) { ++row_drops_; return; }
        rows.push_back(std::move(row));
    }
};
