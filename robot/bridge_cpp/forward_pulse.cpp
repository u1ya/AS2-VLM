#include <csignal>
#include <iostream>
#include <map>
#include <string>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/as2/sport/sport_client.hpp>

#include "forward_pulse_policy.hpp"

namespace {
volatile std::sig_atomic_t cancelled = 0;
void cancel(int) { cancelled = 1; }
}

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "Usage: as2_forward_pulse NETWORK_INTERFACE --execute\n"
                  << "MANUAL physical test: +0.1 m/s forward for nominally 0.3 seconds, then StopMove.\n"
                  << "Requires a trained operator, clear level floor, and independent remote stop.\n"
                  << "Type FORWARD at the local prompt to authorize. EOF cancels.\n"
                  << "No hardware watchdog or guaranteed travel distance. Not a VLM action.\n";
        return 0;
    }
    if (argc != 3 || std::string(argv[2]) != "--execute") {
        std::cerr << "Use --help. No robot command sent.\n";
        return 2;
    }

    std::cout << "PHYSICAL TEST: 0.1 m/s forward, nominal duration 0.3 s.\n"
              << "Hold the independent remote stop; clear the floor and leave cable slack.\n"
              << "Type FORWARD only when ready, or anything else to cancel: " << std::flush;
    std::string confirmation;
    if (!std::getline(std::cin, confirmation) || confirmation != "FORWARD") {
        std::cout << "Cancelled; no robot command sent.\n";
        return 2;
    }
    std::signal(SIGINT, cancel);
    std::signal(SIGTERM, cancel);
    try {
        unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
        unitree::robot::as2::SportClient client;
        client.SetTimeout(0.5F);
        client.Init();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::map<std::string, std::string> state;
        const auto state_code = client.GetState(state);
        std::cout << "GetState=" << state_code << std::endl;
        if (cancelled || state_code != 0 || state["fsm_name"] != "NORMAL_WALK" || state["process_state"] != "0") {
            std::cerr << "Preflight rejected. No forward command sent; no mode changes attempted.\n";
            return 1;
        }
        const auto baseline_stop = client.StopMove();
        std::cout << "Initial StopMove=" << baseline_stop << std::endl;
        if (cancelled || baseline_stop != 0) {
            std::cerr << "Stop check failed or test cancelled. No forward command sent.\n";
            return 1;
        }
        const auto result = as2_test::forward_pulse(client, cancelled);
        std::cout << "Move=" << result.move_code << " StopMove=" << result.stop_code
                  << " interrupted=" << result.interrupted << std::endl;
        if (result.stop_code != 0) {
            std::cerr << "STOP NOT ACKNOWLEDGED: USE THE PHYSICAL REMOTE STOP NOW.\n";
        } else {
            std::cout << "Stop acknowledged. Verify physically; no measured distance is available.\n";
        }
        return result.move_code == 0 && result.stop_code == 0 && !result.interrupted ? 0 : 1;
    } catch (const std::exception& error) {
        std::cerr << "Test failed: " << error.what() << ". Use the physical remote stop.\n";
        return 1;
    }
}
