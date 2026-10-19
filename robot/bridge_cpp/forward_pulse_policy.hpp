#pragma once

#include <chrono>
#include <csignal>
#include <thread>

namespace as2_test {

constexpr float forward_speed = 0.1F;
constexpr auto pulse_duration = std::chrono::milliseconds(300);

template <typename Client>
class StopGuard {
public:
    explicit StopGuard(Client& client) : client_(client) {}
    StopGuard(const StopGuard&) = delete;
    StopGuard& operator=(const StopGuard&) = delete;

    int finish() noexcept {
        if (done_) return 0;
        int result = -1;
        for (int attempt = 0; attempt < 3; ++attempt) {
            try {
                result = client_.StopMove();
                if (result == 0) {
                    done_ = true;
                    return 0;
                }
            } catch (...) {
                result = -1;
            }
        }
        done_ = true;
        return result;
    }

    ~StopGuard() { finish(); }

private:
    Client& client_;
    bool done_ = false;
};

struct PulseResult {
    int move_code;
    int stop_code;
    bool interrupted;
};

// Best-effort host-side stopping, not a watchdog on the robot or a hardware E-stop.
template <typename Client>
PulseResult forward_pulse(Client& client, const volatile std::sig_atomic_t& cancelled) {
    if (cancelled) return {-1, -1, true};
    StopGuard<Client> guard(client);
    const auto deadline = std::chrono::steady_clock::now() + pulse_duration;
    const int moved = client.Move(forward_speed, 0.0F, 0.0F);
    while (moved == 0 && !cancelled && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return {moved, guard.finish(), cancelled != 0};
}

}  // namespace as2_test
