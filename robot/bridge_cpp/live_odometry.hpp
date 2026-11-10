#pragma once
#include <chrono>
#include <mutex>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include "meter_policy.hpp"

namespace as2_test {
inline double odom_now() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
class LiveOdometry {
    std::mutex mutex_;
    OdomSample latest_;
    unsigned count_ = 0;
    bool fault_ = false;
    unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_> subscriber_;
public:
    LiveOdometry() : subscriber_("rt/lf/sportmodestate") {
        subscriber_.InitChannel([this](const void* raw) {
            const auto& msg = *static_cast<const unitree_go::msg::dds_::SportModeState_*>(raw);
            if (msg.stamp().sec() < 0 || msg.stamp().nanosec() >= 1000000000U) return;
            OdomSample s;
            s.x = msg.position()[0]; s.y = msg.position()[1]; s.z = msg.position()[2];
            s.yaw = msg.imu_state().rpy()[2]; s.roll = msg.imu_state().rpy()[0]; s.pitch = msg.imu_state().rpy()[1];
            s.vx = msg.velocity()[0]; s.vy = msg.velocity()[1]; s.vz = msg.velocity()[2];
            s.mode = msg.mode(); s.error = msg.error_code(); s.received = odom_now();
            s.source_stamp = std::uint64_t(msg.stamp().sec()) * 1000000000ULL + msg.stamp().nanosec();
            std::lock_guard<std::mutex> lock(mutex_);
            if (fault_) return;
            if (latest_.source_stamp > 0 && s.source_stamp <= latest_.source_stamp) {
                if (s.source_stamp < latest_.source_stamp) { fault_ = true; latest_.error = 0xffffffffU; }
                return;
            }
            s.sequence = ++count_; latest_ = s;
        }, 64);
    }
    ~LiveOdometry() { subscriber_.CloseChannel(); }
    OdomSample snapshot() { std::lock_guard<std::mutex> lock(mutex_); return latest_; }
    unsigned count() { std::lock_guard<std::mutex> lock(mutex_); return count_; }
};
}
