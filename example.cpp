#include <iostream>
#include <vector>
#include <vad.h>
#include <wav_file.h>

int main(int argc, char **argv) {
    auto wad = webrtc::CreateVad(webrtc::Vad::kVadNormal);

    webrtc::WavReader reader("../leak-test.wav");

    std::vector<float> v(1000);
    reader.ReadSamples(v.size(), v.data());

    std::cout << "reader.sample_rate() " << reader.sample_rate() << "\n";

    webrtc::Vad::Activity activity = wad->VoiceActivity(reinterpret_cast<const int16_t*>(v.data()), v.size(), reader.sample_rate());

    std::cout << activity << "\n";

    return 0;
}
