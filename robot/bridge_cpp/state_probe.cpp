#include <exception>
#include <chrono>
#include <iostream>
#include <map>
#include <string>
#include <thread>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/as2/sport/sport_client.hpp>

int main(int argc, char** argv) {
    if ((argc != 2 && argc != 3) || std::string(argv[1]) == "--help" ||
        (argc == 3 && std::string(argv[2]) != "--json")) {
        std::cerr << "Usage: as2_state_probe NETWORK_INTERFACE [--json]\n"
                  << "Read-only GetState probe. Does not issue motion commands.\n";
        return argc == 2 ? 0 : 2;
    }
    try {
        unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
        unitree::robot::as2::SportClient client;
        client.SetTimeout(5.0F);
        client.Init();
        // Allow DDS discovery before the single read-only API request.
        std::this_thread::sleep_for(std::chrono::seconds(2));
        std::cerr << "Requesting AS2 GetState on " << argv[1] << std::endl;
        std::map<std::string, std::string> state;
        const auto result = client.GetState(state);
        if (argc == 3) {
            std::cout << "{\"sdk_return_code\":" << result << ",\"state\":"
                      << unitree::common::ToJsonString(state) << "}\n";
            return result == 0 ? 0 : 1;
        }
        std::cout << "sdk_return_code=" << result << '\n';
        if (result != 0) {
            return 1;
        }
        for (const auto& entry : state) {
            std::cout << entry.first << '=' << entry.second << '\n';
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "GetState probe failed: " << error.what() << '\n';
        return 1;
    }
}
