#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <mutex>
#include <string>
#include <thread>

#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/idl/ros2/Imu_.hpp>
#include <unitree/idl/go2/LidarState_.hpp>

using Cloud = sensor_msgs::msg::dds_::PointCloud2_;
double clock_seconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

bool summarize(const Cloud& cloud) {
    std::cout << "frame=" << cloud.header().frame_id()
              << " stamp=" << cloud.header().stamp().sec() << ':' << cloud.header().stamp().nanosec()
              << " width=" << cloud.width() << " height=" << cloud.height()
              << " point_step=" << cloud.point_step() << " row_step=" << cloud.row_step()
              << " bytes=" << cloud.data().size() << " bigendian=" << cloud.is_bigendian() << '\n';
    std::size_t offsets[3] = {};
    bool found[3] = {};
    for (const auto& field : cloud.fields()) {
        std::cout << "field=" << field.name() << " offset=" << field.offset()
                  << " datatype=" << unsigned(field.datatype()) << " count=" << field.count() << '\n';
        for (int i = 0; i < 3; ++i) {
            if (field.name() == std::string(1, "xyz"[i]) && field.datatype() == 7 && field.count() == 1 &&
                field.offset() <= cloud.point_step() && cloud.point_step() - field.offset() >= 4) {
                offsets[i] = field.offset(); found[i] = true;
            }
        }
    }
    const std::uint64_t size = std::uint64_t(cloud.row_step()) * cloud.height();
    const std::uint64_t points = std::uint64_t(cloud.width()) * cloud.height();
    if (!found[0] || !found[1] || !found[2] || points == 0 || points > 1000000 ||
        cloud.data().size() > 16 * 1024 * 1024 || size > cloud.data().size() ||
        std::uint64_t(cloud.width()) * cloud.point_step() > cloud.row_step()) {
        std::cout << "XYZ layout unsupported/invalid; no range validation performed.\n";
        return false;
    }
    const std::uint16_t endian = 1;
    const bool host_big = *reinterpret_cast<const unsigned char*>(&endian) == 0;
    std::uint64_t finite = 0, nonzero = 0;
    double nearest = std::numeric_limits<double>::infinity(), farthest = 0;
    for (std::uint32_t row = 0; row < cloud.height(); ++row) {
        for (std::uint32_t col = 0; col < cloud.width(); ++col) {
            const auto base = std::size_t(row) * cloud.row_step() + std::size_t(col) * cloud.point_step();
            float xyz[3];
            for (int i = 0; i < 3; ++i) {
                unsigned char bytes[4];
                std::memcpy(bytes, cloud.data().data() + base + offsets[i], 4);
                if (host_big != cloud.is_bigendian()) std::reverse(bytes, bytes + 4);
                std::memcpy(&xyz[i], bytes, 4);
            }
            if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2])) continue;
            ++finite;
            const double distance = std::hypot(std::hypot(double(xyz[0]), double(xyz[1])), double(xyz[2]));
            if (distance <= 1e-6) continue;
            ++nonzero;
            nearest = std::min(nearest, distance); farthest = std::max(farthest, distance);
        }
    }
    std::cout << "finite_xyz=" << finite << " nonzero_xyz=" << nonzero;
    if (nonzero) std::cout << " range_raw_min=" << nearest << " range_raw_max=" << farthest;
    std::cout << " (coordinate units and calibration not independently verified)\n";
    return nonzero > 0;
}

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "Usage: as2_lidar_probe INTERFACE\n"
                  << "Read discovered AS2 cloud, lidar state and lidar IMU for 8 seconds.\n"
                  << "No sensor switch, configuration or motion commands.\n";
        return 0;
    }
    if (argc != 2) return 2;
    try {
        std::mutex mutex;
        Cloud last_cloud;
        unitree_go::msg::dds_::LidarState_ last_state;
        unsigned clouds = 0, states = 0, imus = 0;
        double first = 0, last = 0;
        unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
        unitree::robot::ChannelSubscriber<Cloud> cloud("rt/utlidar/cloud_livox_mid360");
        unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LidarState_> state("rt/utlidar/lidar_state");
        unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::Imu_> imu("rt/utlidar/imu_livox_mid360");
        cloud.InitChannel([&](const void* raw) {
            std::lock_guard<std::mutex> lock(mutex);
            last_cloud = *static_cast<const Cloud*>(raw);
            const double now = clock_seconds();
            if (clouds++ == 0) first = now;
            last = now;
        }, 1);
        state.InitChannel([&](const void* raw) {
            std::lock_guard<std::mutex> lock(mutex);
            last_state = *static_cast<const unitree_go::msg::dds_::LidarState_*>(raw); ++states;
        }, 1);
        imu.InitChannel([&](const void*) { std::lock_guard<std::mutex> lock(mutex); ++imus; }, 1);
        std::this_thread::sleep_for(std::chrono::seconds(8));
        cloud.CloseChannel(); state.CloseChannel(); imu.CloseChannel();
        std::lock_guard<std::mutex> lock(mutex);
        std::cout << "cloud_messages=" << clouds << " state_messages=" << states << " imu_messages=" << imus;
        if (clouds > 1 && last > first) std::cout << " observed_cloud_hz=" << (clouds - 1) / (last - first);
        if (clouds) std::cout << " last_cloud_age_s=" << clock_seconds() - last;
        std::cout << '\n';
        if (states) {
            std::cout << "firmware=" << last_state.firmware_version() << " software=" << last_state.software_version()
                      << " sdk=" << last_state.sdk_version() << " error_state=" << unsigned(last_state.error_state())
                      << " dirty_percentage=" << unsigned(last_state.dirty_percentage())
                      << " reported_cloud_hz=" << last_state.cloud_frequency()
                      << " reported_cloud_loss=" << last_state.cloud_packet_loss_rate()
                      << " reported_cloud_size=" << last_state.cloud_size() << '\n';
        }
        const bool valid = clouds && summarize(last_cloud);
        return valid && (!states || last_state.error_state() == 0) ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "Lidar read failed: " << error.what() << '\n'; return 1;
    }
}
