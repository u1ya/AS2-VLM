#pragma once
#include <cmath>
#include <limits>
#include <string>

namespace as2_test {
class UiHeartbeat {
    std::string pending_;
    double last_ = -std::numeric_limits<double>::infinity();
    bool stopped_ = false;
public:
    bool feed(const std::string& bytes, double now) {
        if (stopped_ || !std::isfinite(now) || now < last_ || (std::isfinite(last_) && now - last_ > .8)) return stop();
        pending_ += bytes;
        if (pending_.size() > 4096) return stop();
        std::size_t end;
        while ((end = pending_.find('\n')) != std::string::npos) {
            const auto command = pending_.substr(0, end);
            pending_.erase(0, end + 1);
            if (command != "KEEPALIVE") return stop();
            last_ = now;
        }
        return true;
    }
    bool stop() { stopped_ = true; return false; }
    bool ready() const { return std::isfinite(last_) && !stopped_; }
    bool alive(double now) {
        if (!ready() || !std::isfinite(now) || now < last_ || now - last_ > .8) return stop();
        return true;
    }
};
}
