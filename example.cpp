#include <iostream>
#include <vector>
#include <vad.h>
#include <wav_file.h>

struct Frame {
    int left;
    int right;
    float duration;
};

std::vector<Frame> generate_frames(int frame_duration_ms, const std::vector<int16_t>& audio, int sample_rate) {
    std::vector<Frame> frames;
    int n = int(sample_rate * (frame_duration_ms / 1000.0) * 2.0);
    int offset = 0;
    float timestamp = 0.0;
    float duration = frame_duration_ms * n;
    while (offset + n <= audio.size()) {
        timestamp += frame_duration_ms * 2;
        frames.emplace_back(Frame{offset, offset + n, timestamp});
        offset += n;
    }
    return frames;
}


int main(int argc, char **argv) {
    auto wad = webrtc::CreateVad(webrtc::Vad::kVadNormal);

    webrtc::WavReader reader("../leak-test.wav");
    std::cout << "reader.sample_rate() " << reader.sample_rate() << "\n";

    std::vector<int16_t> v(reader.num_samples());
    std::cout << "readed: " << reader.ReadSamples(reader.num_samples(), v.data()) << "\n";

    for (const Frame& frame: generate_frames(30, v, reader.sample_rate())) {
        webrtc::Vad::Activity activity = wad->VoiceActivity(v.data() + frame.left, (frame.right - frame.left) / 2, reader.sample_rate());
        std::cout << "timestamp: " << frame.duration / 1000. << "; activity: " << activity << "\n";
    }

    return 0;
}
