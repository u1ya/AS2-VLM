#include <iostream>
#include <limits>
#include <string>

#include "meter_policy.hpp"

void require(bool value) {
    if (!value) throw std::runtime_error("meter policy test failed");
}

as2_test::OdomSample origin() {
    as2_test::OdomSample s;
    s.z = 0.32; s.mode = 3; s.sequence = 100;
    return s;
}

int main() {
    auto base = origin();
    as2_test::MeterPolicy normal(base, 0);
    as2_test::MeterDecision result{};
    auto s = base;
    for (int i = 1; i <= 200; ++i) {
        s.x = i * 0.005; s.received = i * 0.05; s.sequence = 100 + i; s.vx = 0.1;
        result = normal.update(s, s.received);
        if (result.stop) break;
        require(result.speed <= 0.15F && result.speed >= 0.04F);
    }
    require(result.reached && result.progress >= 0.98 && result.progress <= 1.01 && result.speed == 0);
    require(normal.update(s, s.received).stop);

    auto reject = [&](as2_test::OdomSample value, double now, const std::string& reason) {
        as2_test::MeterPolicy policy(base, 0);
        auto decision = policy.update(value, now);
        require(decision.stop && !decision.reached && decision.speed == 0 && decision.reason == reason);
    };
    reject(base, 0.31, "stale_telemetry");
    s = base; s.received = 12; s.sequence = 101;
    reject(s, 12, "time_limit");
    s = base; s.received = 2.1; s.sequence = 101;
    reject(s, 2.1, "no_progress");
    s = base; s.sequence = 99;
    reject(s, 0, "delivery_sequence_reset");
    s = base; s.x = 1;
    reject(s, 0, "odometry_jump");
    s = base; s.yaw = 0.26;
    reject(s, 0, "yaw_deviation");
    s = base; s.x = -0.05;
    reject(s, 0, "backward_motion");
    s = base; s.z = 0.39;
    reject(s, 0, "height_deviation");
    s = base; s.vx = 0.5;
    reject(s, 0, "horizontal_speed_limit");
    s = base; s.vz = 0.5001;
    reject(s, 0, "vertical_speed_limit");
    s = base; s.vz = -0.5001;
    reject(s, 0, "vertical_speed_limit");
    for (double vz : {0.187889, -0.187889, 0.276612, -0.276612, 0.5, -0.5}) {
        as2_test::MeterPolicy policy(base, 0);
        s = base; s.vz = vz; ++s.sequence;
        require(!policy.update(s, 0).stop);
    }
    s = base; s.mode = 4;
    reject(s, 0, "invalid_state_or_posture");
    s = base; s.error = 1;
    reject(s, 0, "invalid_state_or_posture");
    s = base; s.x = std::numeric_limits<double>::quiet_NaN();
    reject(s, 0, "invalid_state_or_posture");
    s = base; s.roll = 0.4;
    reject(s, 0, "invalid_state_or_posture");
    s = base; s.received = 0.01;
    reject(s, 0.01, "non_advancing_sequence");

    // The target is relative to the original heading, not global x.
    base.yaw = std::acos(-1.0) / 2;
    as2_test::MeterPolicy rotated(base, 0);
    s = base; s.y = 0.01; s.sequence = 101; s.received = 0.1;
    result = rotated.update(s, 0.1);
    require(!result.stop && std::abs(result.progress - 0.01) < 1e-9);
    auto stamped = origin();
    stamped.source_stamp = 100000;
    as2_test::MeterPolicy timestamped(stamped, 0);
    s = stamped; s.source_stamp = 99999; ++s.sequence;
    require(timestamped.update(s, 0).stop);
    bool invalid_origin = false;
    base.vx = 0.1;
    try { as2_test::MeterPolicy moving(base, 0); }
    catch (const std::runtime_error&) { invalid_origin = true; }
    require(invalid_origin);
    std::cout << "Meter policy cases passed without hardware access.\n";
}
