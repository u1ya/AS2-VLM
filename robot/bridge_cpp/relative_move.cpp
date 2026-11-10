#include <csignal>
#include <iostream>
#include <map>
#include <poll.h>
#include <regex>
#include <thread>
#include <unistd.h>
#include <unitree/robot/as2/sport/sport_client.hpp>
#include "forward_pulse_policy.hpp"
#include "live_odometry.hpp"
#include "relative_policy.hpp"
#include "ui_heartbeat.hpp"

namespace {
volatile std::sig_atomic_t cancelled = 0;
void cancel(int) { cancelled = 1; }
bool input(as2_test::UiHeartbeat& lease) {
    pollfd fd{STDIN_FILENO, POLLIN | POLLHUP, 0};
    while (poll(&fd, 1, 0) > 0) {
        char bytes[512]; const auto count = read(STDIN_FILENO, bytes, sizeof(bytes));
        if (count <= 0 || !lease.feed(std::string(bytes, std::size_t(count)), as2_test::odom_now())) {cancelled=1; return false;}
    }
    return !cancelled;
}
}
int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "Usage: as2_relative_move INTERFACE walk|rotate SIGNED_METRES_OR_RADIANS --execute-ui\n"
                  << "Max step 1 m or pi/2 rad; max speeds .15 m/s and .25 rad/s; 12 s; .8 s UI heartbeat.\n"
                  << "RELATIVE confirmation required. Independent remote stop and clear space required. No obstacle detection.\n";
        return 0;
    }
    if (argc != 5 || std::string(argv[4]) != "--execute-ui" ||
        !std::regex_match(argv[1], std::regex("[A-Za-z0-9][A-Za-z0-9_.:-]{0,63}")) ||
        (std::string(argv[2]) != "walk" && std::string(argv[2]) != "rotate") ||
        !std::regex_match(argv[3], std::regex("[+-]?[0-9]+(\\.[0-9]+)?"))) return 2;
    try {
        const bool rotate = std::string(argv[2]) == "rotate";
        const double value = std::stod(argv[3]);
        if (!std::isfinite(value) || std::abs(value) < .05 || std::abs(value) > (rotate ? 1.570796326794897 : 1.0)) return 2;
        std::string confirmation; char ch;
        while (confirmation.size() < 16 && read(STDIN_FILENO, &ch, 1) == 1 && ch != '\n') confirmation += ch;
        if (confirmation != "RELATIVE") {std::cout << "Cancelled; no command sent.\n"; return 2;}
        std::signal(SIGINT, cancel); std::signal(SIGTERM, cancel);
        unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
        as2_test::LiveOdometry odometry;
        std::this_thread::sleep_for(std::chrono::seconds(3));
        as2_test::RelativePolicy preflight(odometry.snapshot(), as2_test::odom_now(), rotate, value);
        if (odometry.count() < 10 || cancelled) return 1;
        unitree::robot::as2::SportClient client; client.SetTimeout(.25F); client.Init();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::map<std::string, std::string> state;
        if (client.GetState(state) != 0 || state["fsm_name"] != "NORMAL_WALK" || state["process_state"] != "0" || cancelled) return 1;
        if (client.StopMove() != 0) return 1;
        as2_test::StopGuard<unitree::robot::as2::SportClient> guard(client);
        as2_test::UiHeartbeat lease;
        std::cout << "UI_READY" << std::endl;
        const double deadline = as2_test::odom_now() + 1;
        while (!cancelled && !lease.ready() && as2_test::odom_now() < deadline) {input(lease); std::this_thread::sleep_for(std::chrono::milliseconds(10));}
        bool reached = false;
        if (!lease.ready() || cancelled) {guard.finish(); return 1;}
        as2_test::RelativePolicy policy(odometry.snapshot(), as2_test::odom_now(), rotate, value);
        double last_log = 0;
        while (!cancelled) {
            if (!input(lease) || !lease.alive(as2_test::odom_now())) {std::cout << "Result=ui_heartbeat_lost\n"; break;}
            const auto decision = policy.update(odometry.snapshot(), as2_test::odom_now());
            if (decision.stop) {std::cout << "Result=" << decision.reason << " progress=" << decision.progress << std::endl; reached=decision.reached; break;}
            if (client.Move(decision.vx, 0.0F, decision.vyaw) != 0) {std::cout << "Result=move_failed\n"; break;}
            if (as2_test::odom_now() - last_log > .5) {std::cout << "progress=" << decision.progress << std::endl; last_log=as2_test::odom_now();}
            std::this_thread::sleep_for(std::chrono::milliseconds(40));
        }
        const auto stopped = guard.finish();
        std::this_thread::sleep_for(std::chrono::milliseconds(600));
        const auto a=odometry.snapshot();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        const auto b=odometry.snapshot();
        const bool still = as2_test::valid_sample(a) && as2_test::valid_sample(b) && as2_test::odom_now()-b.received<=.3 &&
            std::hypot(b.vx,b.vy)<.05 && std::abs(b.vz)<.05 && std::hypot(b.x-a.x,b.y-a.y)<.015 &&
            std::abs(std::atan2(std::sin(b.yaw-a.yaw),std::cos(b.yaw-a.yaw)))<.02;
        std::cout << "StopMove=" << stopped << " telemetry_stationary=" << still << std::endl;
        if (stopped != 0 || !still) std::cerr << "USE INDEPENDENT REMOTE STOP; stopping not verified.\n";
        return reached && stopped==0 && still && !cancelled ? 0 : 1;
    } catch (const std::exception& e) {std::cerr << "Rejected/failed: " << e.what() << "; verify robot with remote stop.\n"; return 1;}
}
