#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace as2_test {

struct OdomSample {
    double x = 0, y = 0, z = 0, yaw = 0, roll = 0, pitch = 0;
    double vx = 0, vy = 0, vz = 0, received = 0;
    std::uint64_t sequence = 0, source_stamp = 0;
    unsigned mode = 0, error = 0;
};

inline bool valid_sample(const OdomSample& s) {
    return std::isfinite(s.x) && std::isfinite(s.y) && std::isfinite(s.z) &&
           std::isfinite(s.yaw) && std::isfinite(s.roll) && std::isfinite(s.pitch) &&
           std::isfinite(s.vx) && std::isfinite(s.vy) && std::isfinite(s.vz) &&
           std::isfinite(s.received) && s.sequence > 0 && s.error == 0 && s.mode == 3 &&
           std::abs(s.roll) < 0.3 && std::abs(s.pitch) < 0.3 && s.z > 0.2 && s.z < 0.6;
}

struct MeterDecision {
    bool stop;
    bool reached;
    float speed;
    double progress;
    const char* reason;
};

class MeterPolicy {
public:
    MeterPolicy(const OdomSample& origin, double now)
        : origin_(origin), last_(origin), started_(now), progress_time_(now) {
        if (!valid_sample(origin) || !std::isfinite(now) || now < origin.received || now - origin.received > 0.3 ||
            std::hypot(origin.vx, origin.vy) > 0.05 || std::abs(origin.vz) > 0.05) {
            throw std::runtime_error("Fresh, upright, stationary odometry is required");
        }
    }

    MeterDecision update(const OdomSample& s, double now) {
        if (stopped_) return {true, false, 0, progress_, "already_stopped"};
        auto stop = [&](const char* reason, bool reached = false) {
            stopped_ = true;
            return MeterDecision{true, reached, 0, progress_, reason};
        };
        if (!std::isfinite(now) || now < started_ || now < s.received || now - s.received > 0.3)
            return stop("stale_telemetry");
        if (!valid_sample(s)) return stop("invalid_state_or_posture");
        if (s.sequence < last_.sequence) return stop("delivery_sequence_reset");
        if (last_.source_stamp > 0 && s.source_stamp < last_.source_stamp)
            return stop("odometry_timestamp_reset");
        if (now - started_ >= 12.0) return stop("time_limit");
        const double dx = s.x - origin_.x, dy = s.y - origin_.y;
        progress_ = dx * std::cos(origin_.yaw) + dy * std::sin(origin_.yaw);
        const double lateral = -dx * std::sin(origin_.yaw) + dy * std::cos(origin_.yaw);
        const double yaw_delta = std::atan2(std::sin(s.yaw - origin_.yaw), std::cos(s.yaw - origin_.yaw));
        if (std::hypot(s.x - last_.x, s.y - last_.y) > 0.07)
            return stop("odometry_jump");
        if (std::abs(yaw_delta) > 0.25) return stop("yaw_deviation");
        if (std::abs(lateral) > 0.12) return stop("lateral_deviation");
        if (progress_ < -0.04) return stop("backward_motion");
        if (std::abs(s.z - origin_.z) > 0.06) return stop("height_deviation");
        if (std::hypot(s.vx, s.vy) > 0.35) return stop("horizontal_speed_limit");
        if (std::abs(s.vz) > 0.5) return stop("vertical_speed_limit");
        if (s.sequence == last_.sequence && (s.x != last_.x || s.y != last_.y || s.received != last_.received))
            return stop("non_advancing_sequence");
        last_ = s;
        if (progress_ >= 0.98) return stop("distance_reached", true);
        if (progress_ >= best_progress_ + 0.02) {
            best_progress_ = progress_;
            progress_time_ = now;
        }
        if (now - progress_time_ > 2.0) return stop("no_progress");
        const float speed = static_cast<float>(std::clamp(0.8 * (1.0 - progress_), 0.04, 0.15));
        return {false, false, speed, progress_, "walking"};
    }

private:
    OdomSample origin_, last_;
    double started_, progress_time_, best_progress_ = 0, progress_ = 0;
    bool stopped_ = false;
};

}  // namespace as2_test
