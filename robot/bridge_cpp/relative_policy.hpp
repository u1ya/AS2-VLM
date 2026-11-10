#pragma once
#include "meter_policy.hpp"

namespace as2_test {
struct RelativeDecision {
    bool stop = false, reached = false;
    float vx = 0, vyaw = 0;
    double progress = 0;
    const char* reason = "moving";
};
class RelativePolicy {
    OdomSample origin_, last_;
    bool rotate_, stopped_ = false;
    double target_, sign_, started_, progress_at_, best_ = 0, progress_ = 0, yaw_sum_ = 0;
public:
    RelativePolicy(const OdomSample& origin, double now, bool rotate, double value)
        : origin_(origin), last_(origin), rotate_(rotate), target_(std::abs(value)), sign_(value > 0 ? 1 : -1),
          started_(now), progress_at_(now) {
        MeterPolicy preflight(origin, now);
        if (!std::isfinite(value) || target_ < (rotate ? .05 : .05) || target_ > (rotate ? 1.570796326794897 : 1.0))
            throw std::runtime_error("Relative primitive exceeds bounded translation/rotation limits");
    }
    RelativeDecision update(const OdomSample& s, double now) {
        auto stop = [&](const char* reason, bool reached = false) {
            stopped_ = true; return RelativeDecision{true, reached, 0, 0, progress_, reason};
        };
        if (stopped_) return stop("already_stopped");
        if (!std::isfinite(now) || now < started_ || now < s.received || now - s.received > .3) return stop("stale_telemetry");
        if (!valid_sample(s)) return stop("invalid_state_or_posture");
        if (s.sequence < last_.sequence || (last_.source_stamp > 0 && s.source_stamp < last_.source_stamp)) return stop("odometry_reset");
        if (s.sequence == last_.sequence && (s.x != last_.x || s.y != last_.y || s.received != last_.received || s.yaw != last_.yaw))
            return stop("non_advancing_sequence");
        if (now - started_ >= 12) return stop("time_limit");
        const auto delta = std::atan2(std::sin(s.yaw-last_.yaw), std::cos(s.yaw-last_.yaw));
        if (std::hypot(s.x-last_.x, s.y-last_.y) > .07 || std::abs(delta) > .12) return stop("odometry_jump");
        const auto dx = s.x-origin_.x, dy = s.y-origin_.y;
        const auto heading = std::atan2(std::sin(s.yaw-origin_.yaw), std::cos(s.yaw-origin_.yaw));
        if (std::abs(s.z-origin_.z) > .06) return stop("height_deviation");
        if (std::hypot(s.vx, s.vy) > .35 || std::abs(s.vz) > .5) return stop("measured_speed_limit");
        if (rotate_) {
            if (std::hypot(dx, dy) > .12) return stop("translation_during_rotation");
            yaw_sum_ += delta; progress_ = sign_ * yaw_sum_;
        } else {
            if (std::abs(heading) > .25) return stop("yaw_deviation");
            if (std::abs(-dx*std::sin(origin_.yaw)+dy*std::cos(origin_.yaw)) > .12) return stop("lateral_deviation");
            progress_ = sign_ * (dx*std::cos(origin_.yaw)+dy*std::sin(origin_.yaw));
        }
        last_ = s;
        if (progress_ < -(rotate_ ? .08 : .04)) return stop("wrong_direction");
        if (progress_ >= target_ - (rotate_ ? .035 : .02)) return stop(rotate_ ? "angle_reached" : "distance_reached", true);
        if (progress_ > best_ + .01) {best_ = progress_; progress_at_ = now;}
        if (now - progress_at_ > 2) return stop("no_progress");
        const auto speed = sign_ * std::clamp(.8*(target_-progress_), rotate_ ? .10 : .08, rotate_ ? .25 : .15);
        return {false, false, rotate_ ? 0.0F : float(speed), rotate_ ? float(speed) : 0.0F, progress_, "moving"};
    }
};
}
