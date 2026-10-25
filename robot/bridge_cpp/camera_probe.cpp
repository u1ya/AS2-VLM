#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/video/video_client.hpp>
#include <unitree/robot/b2/front_video/front_video_client.hpp>
#include <unitree/robot/b2/back_video/back_video_client.hpp>

template <typename Client>
int capture(const std::string& output) {
    Client client;
    client.SetTimeout(3.0F);
    client.Init();
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::vector<uint8_t> bytes;
    const auto start = std::chrono::steady_clock::now();
    const int code = client.GetImageSample(bytes);
    const double seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    std::cout << "sdk_return_code=" << code << " bytes=" << bytes.size()
              << " request_seconds=" << seconds << std::endl;
    if (code != 0) return 1;
    // Treat the response as untrusted; decode and verify it separately afterward.
    if (bytes.size() < 4 || bytes.size() > 10 * 1024 * 1024 ||
        bytes[0] != 0xff || bytes[1] != 0xd8 || bytes[2] != 0xff)
        throw std::runtime_error("Expected a JPEG response of at most 10 MiB");
    std::ofstream file(output, std::ios::binary);
    file.exceptions(std::ios::failbit | std::ios::badbit);
    file.write(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    file.close();
    std::cout << "Saved JPEG candidate: " << output
              << "; image decoding and freshness still require verification.\n";
    return 0;
}

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "Usage: as2_camera_probe INTERFACE videohub|front_videohub|back_videohub OUTPUT.jpg\n"
                  << "One read-only GetImageSample request. No motion or camera configuration.\n"
                  << "AS2 compatibility is experimental. Output must not already exist.\n";
        return 0;
    }
    if (argc != 4) return 2;
    try {
        const std::string service = argv[2];
        const std::filesystem::path output = argv[3];
        if ((service != "videohub" && service != "front_videohub" && service != "back_videohub") ||
            output.extension() != ".jpg" || std::filesystem::exists(output))
            throw std::runtime_error("Invalid service, extension, or existing output file");
        unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
        std::cout << "Read-only image request: service=" << service << std::endl;
        if (service == "videohub")
            return capture<unitree::robot::go2::VideoClient>(output.string());
        if (service == "back_videohub")
            return capture<unitree::robot::b2::BackVideoClient>(output.string());
        return capture<unitree::robot::b2::FrontVideoClient>(output.string());
    } catch (const std::exception& error) {
        std::cerr << "Camera probe failed: " << error.what() << '\n';
        return 1;
    }
}
