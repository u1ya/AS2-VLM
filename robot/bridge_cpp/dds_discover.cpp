#include <dds/dds.h>
#include <chrono>
#include <iostream>
#include <regex>
#include <set>
#include <string>
#include <thread>

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--help") {
        std::cout << "Usage: as2_dds_discover INTERFACE\n"
                  << "Read DDS domain 0 publisher announcements for 8 seconds. No control requests.\n";
        return 0;
    }
    if (argc != 2 || !std::regex_match(argv[1], std::regex("[a-zA-Z0-9][a-zA-Z0-9_.:-]{0,63}"))) return 2;
    const std::string config = "<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"" +
        std::string(argv[1]) + "\"/></Interfaces></General></Domain></CycloneDDS>";
    const auto domain = dds_create_domain(0, config.c_str());
    if (domain < 0) { std::cerr << "Domain error: " << domain << '\n'; return 1; }
    const auto participant = dds_create_participant(0, nullptr, nullptr);
    const auto reader = participant < 0 ? participant : dds_create_reader(
        participant, DDS_BUILTIN_TOPIC_DCPSPUBLICATION, nullptr, nullptr);
    if (reader < 0) { dds_delete(domain); return 1; }
    std::set<std::string> endpoints;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
    bool failed = false;
    while (std::chrono::steady_clock::now() < deadline) {
        void* samples[32] = {};
        dds_sample_info_t infos[32] = {};
        const auto count = dds_take(reader, samples, infos, 32, 32);
        if (count < 0) { failed = true; break; }
        for (int i = 0; i < count; ++i) {
            if (!infos[i].valid_data) continue;
            const auto* endpoint = static_cast<const dds_builtintopic_endpoint_t*>(samples[i]);
            const std::string value = std::string(endpoint->topic_name ? endpoint->topic_name : "") +
                " type=" + (endpoint->type_name ? endpoint->type_name : "");
            if (endpoints.insert(value).second) std::cout << value << std::endl;
        }
        if (count > 0) dds_return_loan(reader, samples, count);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::cout << "Distinct topic/type announcements=" << endpoints.size() << std::endl;
    dds_delete(participant);
    dds_delete(domain);
    return failed || endpoints.empty() ? 1 : 0;
}
