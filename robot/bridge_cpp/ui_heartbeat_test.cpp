#include "ui_heartbeat.hpp"
#include <cassert>

int main() {
    as2_test::UiHeartbeat heartbeat;
    assert(!heartbeat.ready());
    assert(heartbeat.feed("KEEP", 1));
    assert(!heartbeat.ready());
    assert(heartbeat.feed("ALIVE\n", 1.1));
    assert(heartbeat.alive(1.2));
    assert(!heartbeat.alive(2));
    assert(!heartbeat.feed("KEEPALIVE\n", 2));
    as2_test::UiHeartbeat stop;
    assert(!stop.feed("STOP\n", 1));
    assert(!stop.feed("KEEPALIVE\n", 1));
    as2_test::UiHeartbeat overflow;
    assert(!overflow.feed(std::string(4097, 'A'), 1));
    as2_test::UiHeartbeat clock;
    assert(clock.feed("KEEPALIVE\n", 2));
    assert(!clock.alive(1));
    as2_test::UiHeartbeat eof;
    assert(eof.feed("KEEPALIVE\n", 1));
    eof.stop();
    assert(!eof.alive(1.1));
    as2_test::UiHeartbeat late;
    assert(late.feed("KEEPALIVE\n", 1));
    assert(!late.feed("KEEPALIVE\n", 2));
}
