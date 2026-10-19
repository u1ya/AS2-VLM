#include <chrono>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/as2/sport/sport_client.hpp>

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "Usage: as2_stop_only NETWORK_INTERFACE --ready|--stop\n"
                  << "--ready: warm up DDS; Enter or STOP requests StopMove. Q exits.\n"
                  << "--stop: start DDS and request StopMove once.\n"
                  << "Network stop only; NOT an independent emergency stop.\n";
        return 0;
    }
    if (argc != 3 || (std::string(argv[2]) != "--ready" && std::string(argv[2]) != "--stop")) {
        std::cerr << "Use --help. No robot command sent.\n";
        return 2;
    }
    try {
        unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
        unitree::robot::as2::SportClient client;
        client.SetTimeout(1.0F);
        client.Init();
        std::this_thread::sleep_for(std::chrono::seconds(2));

        const bool ready = std::string(argv[2]) == "--ready";
        if (ready) {
            std::cout << "DDS initialized locally. Stop delivery has NOT been verified.\n"
                      << "Enter or STOP: request network StopMove. Q: quit.\n"
                      << "Keep the physical remote stop available.\n" << std::flush;
        }
        do {
            if (ready) {
                std::string input;
                if (!std::getline(std::cin, input) || input == "Q" || input == "q") {
                    return 0;
                }
                if (!input.empty() && input != "STOP") {
                    std::cerr << "Only Enter, STOP, or Q is accepted.\n";
                    continue;
                }
            }
            // This utility deliberately exposes no movement or mode-change API.
            const auto result = client.StopMove();
            std::cout << "StopMove sdk_return_code=" << result << std::endl;
            if (result != 0) {
                std::cerr << "STOP NOT ACKNOWLEDGED. Use the physical remote stop.\n";
                if (!ready) {
                    return 1;
                }
            } else {
                std::cout << "Stop request acknowledged; verify the robot physically.\n";
            }
        } while (ready);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Stop client failed: " << error.what()
                  << ". Use the physical remote stop.\n";
        return 1;
    }
}
