#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "forward_pulse_policy.hpp"

struct FakeClient {
    std::vector<char> calls;
    int move_code = 0;
    int stop_code = 0;
    bool throw_move = false;
    volatile std::sig_atomic_t* cancel_on_move = nullptr;

    int Move(float x, float y, float yaw) {
        if (std::abs(x - 0.1F) > 1e-6 || y != 0 || yaw != 0) throw std::runtime_error("wrong velocity");
        calls.push_back('M');
        if (cancel_on_move) *cancel_on_move = 1;
        if (throw_move) throw std::runtime_error("fake move exception");
        return move_code;
    }
    int StopMove() { calls.push_back('S'); return stop_code; }
};

void require(bool value) {
    if (!value) throw std::runtime_error("pulse policy test failed");
}

int main() {
    volatile std::sig_atomic_t cancelled = 0;
    FakeClient normal;
    auto result = as2_test::forward_pulse(normal, cancelled);
    require(normal.calls == std::vector<char>({'M', 'S'}) && result.stop_code == 0);
    FakeClient failed;
    failed.move_code = 3104;
    result = as2_test::forward_pulse(failed, cancelled);
    require(failed.calls == std::vector<char>({'M', 'S'}) && result.move_code == 3104);
    FakeClient throwing;
    throwing.throw_move = true;
    bool threw = false;
    try { as2_test::forward_pulse(throwing, cancelled); }
    catch (const std::runtime_error&) { threw = true; }
    require(threw && throwing.calls == std::vector<char>({'M', 'S'}));
    FakeClient stopping;
    stopping.move_code = 3104;
    stopping.stop_code = 3104;
    result = as2_test::forward_pulse(stopping, cancelled);
    require(stopping.calls == std::vector<char>({'M', 'S', 'S', 'S'}) && result.stop_code == 3104);
    FakeClient interrupting;
    interrupting.cancel_on_move = &cancelled;
    result = as2_test::forward_pulse(interrupting, cancelled);
    require(interrupting.calls == std::vector<char>({'M', 'S'}) && result.interrupted);
    FakeClient pre_cancelled;
    as2_test::forward_pulse(pre_cancelled, cancelled);
    require(pre_cancelled.calls.empty());
    std::cout << "6 pulse policy cases passed using fake client; no hardware access.\n";
}
