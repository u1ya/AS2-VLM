#include "shared_recording.hpp"
#include <cassert>
#include <iterator>
#include <string>

int main() {
    const auto root = std::filesystem::temp_directory_path() /
        ("as2-shared-record-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    {
        SharedRecording recorder;
        bool rejected = false;
        try { recorder.start(root, 2); } catch (const std::exception&) { rejected = true; }
        assert(rejected && !std::filesystem::exists(root));
        recorder.start(root, 3);
        rejected = false;
        try { recorder.start(root, 3); } catch (const std::exception&) { rejected = true; }
        assert(rejected);
        sensor_msgs::msg::dds_::PointCloud2_ cloud;
        cloud.data().resize(26);
        recorder.cloud(cloud, {{"width", 1}});
        recorder.row({{"sample", 1}}, true);
        recorder.row({{"sample", 1}}, false);
        std::this_thread::sleep_for(std::chrono::milliseconds(3150));
    }
    std::ifstream file(root / "manifest.json");
    const std::string manifest((std::istreambuf_iterator<char>(file)), {});
    assert(manifest.find("\"cloud_files\":1") != std::string::npos);
    assert(manifest.find("\"imu_messages\":1") != std::string::npos);
    assert(manifest.find("\"io_error\":false") != std::string::npos);
    assert(std::filesystem::file_size(root / "cloud-1.bin") == 34);
    std::filesystem::remove(root / "cloud-1.bin");
    for (const auto& name : {"manifest.json", "clouds.jsonl", "imu.jsonl", "odometry.jsonl"})
        std::filesystem::remove(root / name);
    std::filesystem::remove(root);
}
