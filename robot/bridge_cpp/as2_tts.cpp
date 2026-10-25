#include <cstdlib>
#include <exception>
#include <iostream>
#include <string>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/as2/audio/audio_client.hpp>

// Speaks one sentence through the AS2 onboard speaker (AudioClient::TtsMaker, as in
// unitree_sdk2 example/as2/audio). Used by RealUnitreeAs2Adapter via robot.tts_binary.
// Does not issue any motion command.
int main(int argc, char** argv) {
    if (argc < 3 || argc > 4 || std::string(argv[1]) == "--help") {
        std::cerr << "Usage: as2_tts NETWORK_INTERFACE TEXT [SPEAKER_ID]\n"
                  << "SPEAKER_ID: 0 = Chinese voice, 1 = English voice (per the SDK example; verify on firmware).\n";
        return argc == 2 ? 0 : 2;
    }
    try {
        unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);
        unitree::robot::as2::AudioClient client;
        client.SetTimeout(10.0F);
        client.Init();
        const int speaker = argc == 4 ? std::atoi(argv[3]) : 1;
        const auto result = client.TtsMaker(argv[2], speaker);
        if (result != 0) {
            std::cerr << "TtsMaker returned " << result << '\n';
            return 1;
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "TTS failed: " << error.what() << '\n';
        return 1;
    }
}
