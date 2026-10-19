#include <chrono>
#include <csignal>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>

#include <unitree/robot/as2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

#include "forward_pulse_policy.hpp"
#include "meter_policy.hpp"

namespace {
volatile std::sig_atomic_t cancelled = 0;
void cancel(int) { cancelled = 1; }
double now_seconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

class Odometry {
public:
    Odometry() : subscriber_("rt/lf/sportmodestate") {
        subscriber_.InitChannel([this](const void* raw) {
            const auto& msg = *static_cast<const unitree_go::msg::dds_::SportModeState_*>(raw);
            as2_test::OdomSample s;
            s.x = msg.position()[0]; s.y = msg.position()[1]; s.z = msg.position()[2];
            s.yaw = msg.imu_state().rpy()[2];
            s.roll = msg.imu_state().rpy()[0]; s.pitch = msg.imu_state().rpy()[1];
            s.vx = msg.velocity()[0]; s.vy = msg.velocity()[1]; s.vz = msg.velocity()[2];
            s.mode = msg.mode(); s.error = msg.error_code();
            if (msg.stamp().sec() < 0 || msg.stamp().nanosec() >= 1000000000U) return;
            s.source_stamp = static_cast<std::uint64_t>(msg.stamp().sec()) * 1000000000ULL + msg.stamp().nanosec();
            s.received = now_seconds();
            std::lock_guard<std::mutex> lock(mutex_);
            if (!raw_seen_) {
                std::cout << "First telemetry: source_stamp_ns=" << s.source_stamp
                          << " x=" << s.x << " y=" << s.y << " z=" << s.z
                          << " yaw=" << s.yaw << std::endl;
                raw_seen_ = true;
            }
            if (fault_) return;
            // DDS handles packet sequencing; this firmware leaves source stamp at zero.
            // Use arrival freshness, and additionally enforce source time when provided.
            if (latest_.source_stamp > 0 && s.source_stamp <= latest_.source_stamp) {
                if (s.source_stamp < latest_.source_stamp) {
                    fault_ = true; latest_.error = 0xffffffffU;
                }
                return;
            }
            s.sequence = ++count_;
            latest_ = s;
        }, 1);
    }
    ~Odometry() { subscriber_.CloseChannel(); }
    as2_test::OdomSample snapshot() {
        std::lock_guard<std::mutex> lock(mutex_);
        return latest_;
    }
    unsigned count() {
        std::lock_guard<std::mutex> lock(mutex_);
        return count_;
    }
private:
    std::mutex mutex_;
    as2_test::OdomSample latest_;
    unsigned count_ = 0;
    bool fault_ = false;
    bool raw_seen_ = false;
    unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_> subscriber_;
};
}

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "Usage: as2_walk_meter INTERFACE --observe|--execute\n"
                  << "--observe reads odometry only. --execute requires typing METER.\n"
                  << "Target: 1 m by odometry (2 cm tolerance); speed cap 0.15 m/s; 12 s limit.\n"
                  << "No obstacle detection or hardware watchdog. Independent operator stop required.\n";
        return 0;
    }
    if (argc != 3 || (std::string(argv[2]) != "--observe" && std::string(argv[2]) != "--execute")) return 2;
    const bool observe = std::string(argv[2]) == "--observe";
    if (!observe) {
        std::cout << "PHYSICAL 1 METER TEST. Clear level floor, cable slack, independent remote stop.\n"
                  << "Type METER only when ready; any other input cancels: " << std::flush;
        std::string confirmation;
        if (!std::getline(std::cin, confirmation) || confirmation != "METER") {
            std::cout << "Cancelled; no robot command sent.\n";
            return 2;
        }
    }
    std::signal(SIGINT, cancel); std::signal(SIGTERM, cancel);
    try {
        unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
        Odometry odometry;
        std::this_thread::sleep_for(std::chrono::seconds(3));
        const auto sample = odometry.snapshot();
        std::cout << "Samples=" << odometry.count() << " source_stamp_ns=" << sample.source_stamp
                  << " x=" << sample.x << " y=" << sample.y << " z=" << sample.z
                  << " yaw=" << sample.yaw << " mode=" << sample.mode << " error=" << sample.error << std::endl;
        as2_test::MeterPolicy preflight(sample, now_seconds());
        if (odometry.count() < 10 || cancelled) throw std::runtime_error("Insufficient fresh telemetry or cancelled");
        if (observe) {
            std::cout << "Read-only odometry check passed. No motion or stop requested.\n";
            return 0;
        }
        unitree::robot::as2::SportClient client;
        client.SetTimeout(0.25F); client.Init();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::map<std::string, std::string> state;
        if (client.GetState(state) != 0 || state["fsm_name"] != "NORMAL_WALK" || state["process_state"] != "0" || cancelled)
            throw std::runtime_error("Sport API state rejected; no forward command sent");
        if (client.StopMove() != 0) throw std::runtime_error("Initial StopMove not acknowledged; no forward command sent");
        as2_test::StopGuard<unitree::robot::as2::SportClient> guard(client);
        const auto origin = odometry.snapshot();
        as2_test::MeterPolicy policy(origin, now_seconds());
        bool reached = false;
        double last_log = 0;
        while (!cancelled) {
            const auto current = odometry.snapshot();
            const double now = now_seconds();
            const auto decision = policy.update(current, now);
            if (decision.stop) {
                std::cout << "Result=" << decision.reason << " odometry_progress_m=" << decision.progress << std::endl;
                std::cout << "Stop sample: x=" << current.x << " y=" << current.y << " z=" << current.z
                          << " yaw=" << current.yaw << " roll=" << current.roll << " pitch=" << current.pitch
                          << " vx=" << current.vx << " vy=" << current.vy << " vz=" << current.vz
                          << " mode=" << current.mode << " error=" << current.error
                          << " age_s=" << now - current.received
                          << " origin_x=" << origin.x << " origin_y=" << origin.y
                          << " origin_z=" << origin.z << " origin_yaw=" << origin.yaw << std::endl;
                reached = decision.reached;
                break;
            }
            if (client.Move(decision.speed, 0.0F, 0.0F) != 0) {
                std::cerr << "Move failed; stopping.\n"; break;
            }
            if (now - last_log >= 0.5) {
                std::cout << "progress_m=" << decision.progress << " speed_command=" << decision.speed << std::endl;
                last_log = now;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
        }
        const auto stopped = guard.finish();
        std::cout << "StopMove=" << stopped << " interrupted=" << cancelled << std::endl;
        if (stopped != 0) std::cerr << "USE THE PHYSICAL REMOTE STOP NOW.\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(600));
        const auto final = odometry.snapshot();
        const double progress = (final.x - origin.x) * std::cos(origin.yaw) + (final.y - origin.y) * std::sin(origin.yaw);
        const bool measured_still = as2_test::valid_sample(final) && now_seconds() - final.received <= 0.3 &&
                                    std::hypot(final.vx, final.vy) < 0.05 && std::abs(final.vz) < 0.05;
        std::cout << "Final odometry progress_m=" << progress << " telemetry_stationary=" << measured_still << std::endl;
        if (!measured_still) std::cerr << "Stationary state NOT verified. Check robot and use independent stop.\n";
        return reached && !cancelled && stopped == 0 && measured_still ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "Test rejected/failed: " << error.what() << ". Verify robot; use remote stop if needed.\n";
        return 1;
    }
}
