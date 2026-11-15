#include "audio.hpp"

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <fstream>
#include <numbers>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace {
static_assert(std::endian::native == std::endian::little, "PCM uses little-endian samples");
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;
constexpr std::size_t chunk_samples = input_rate / 50;
constexpr std::size_t max_wav_bytes = 60 * board_rate * 4;

std::uint32_t read_le(std::istream& file, unsigned bytes) {
    std::uint32_t value = 0;
    for (unsigned index = 0; index < bytes; ++index) {
        const auto byte = file.get();
        if (byte == std::char_traits<char>::eof()) {
            throw std::runtime_error("Truncated WAV header");
        }
        value |= static_cast<std::uint32_t>(byte) << (8 * index);
    }
    return value;
}

std::string read_tag(std::istream& file) {
    std::string tag(4, '\0');
    file.read(tag.data(), static_cast<std::streamsize>(tag.size()));
    return tag;
}

Samples read_wav(const std::filesystem::path& path) {
    std::ifstream file(path, std::ios::binary);
    if (read_tag(file) != "RIFF") throw std::runtime_error("Expected a RIFF WAV file");
    const auto riff_size = read_le(file, 4);
    if (read_tag(file) != "WAVE") throw std::runtime_error("Expected a WAVE header");
    unsigned channels = 0;
    unsigned rate = 0;
    std::uint64_t offset = 12;
    while (offset + 8 <= 8ull + riff_size) {
        const auto tag = read_tag(file);
        const auto size = read_le(file, 4);
        const auto next = offset + 8 + size + size % 2;
        if (next > 8ull + riff_size) throw std::runtime_error("Invalid WAV chunk size");
        if (tag == "fmt ") {
            if (size < 16 || read_le(file, 2) != 1) {
                throw std::runtime_error("WAV must contain uncompressed PCM");
            }
            channels = read_le(file, 2);
            rate = read_le(file, 4);
            read_le(file, 4);
            const auto alignment = read_le(file, 2);
            const auto bits = read_le(file, 2);
            if (bits != 16 || alignment != channels * 2 ||
                !((channels == 1 && rate == input_rate) ||
                  (channels == 2 && rate == board_rate))) {
                throw std::runtime_error("Expected 16-bit 16 kHz mono or 48 kHz stereo WAV");
            }
        } else if (tag == "data") {
            if (channels == 0 || size == 0 || size % (channels * 2) != 0 ||
                size > max_wav_bytes || size > 60 * rate * channels * 2) {
                throw std::runtime_error("Invalid WAV data or clip exceeds 60 seconds");
            }
            Samples samples(size / 2);
            if (!file.read(reinterpret_cast<char*>(samples.data()), size)) {
                throw std::runtime_error("Truncated WAV samples");
            }
            if (channels == 1) return samples;
            return RightChannelResampler{}.convert(samples);
        }
        file.seekg(static_cast<std::streamoff>(next));
        offset = next;
    }
    throw std::runtime_error("WAV is missing PCM data");
}

void write_le(std::ostream& file, std::uint32_t value, unsigned bytes) {
    for (unsigned index = 0; index < bytes; ++index) {
        file.put(static_cast<char>((value >> (8 * index)) & 0xff));
    }
}

void write_wav(const std::filesystem::path& path, const Samples& samples) {
    if (std::filesystem::exists(path)) throw std::runtime_error("Reply WAV already exists");
    std::ofstream file(path, std::ios::binary);
    const auto bytes = static_cast<std::uint32_t>(samples.size() * 2);
    file.write("RIFF", 4);
    write_le(file, 36 + bytes, 4);
    file.write("WAVEfmt ", 8);
    write_le(file, 16, 4);
    write_le(file, 1, 2);
    write_le(file, 1, 2);
    write_le(file, output_rate, 4);
    write_le(file, output_rate * 2, 4);
    write_le(file, 2, 2);
    write_le(file, 16, 2);
    file.write("data", 4);
    write_le(file, bytes, 4);
    file.write(reinterpret_cast<const char*>(samples.data()), bytes);
    file.close();
    if (!file) throw std::runtime_error("Could not write reply WAV");
}

class WavAudio final : public Audio {
public:
    WavAudio(std::vector<std::filesystem::path> inputs, std::filesystem::path output)
        : inputs_(std::move(inputs)), output_(std::move(output)) {
        std::filesystem::create_directories(output_);
    }

    bool start_turn() override {
        if (turn_ == inputs_.size()) return false;
        input_ = read_wav(inputs_[turn_++]);
        input_.resize(input_.size() + input_rate * (vad_silence_ms + 300) / 1000);
        offset_ = 0;
        next_capture_ = Clock::now();
        return true;
    }

    Samples capture() override {
        const auto now = Clock::now();
        if (input_ended() || now < next_capture_) return {};
        const auto length = std::min(chunk_samples, input_.size() - offset_);
        Samples chunk(input_.begin() + offset_, input_.begin() + offset_ + length);
        offset_ += length;
        next_capture_ = now + 20ms;
        return chunk;
    }

    bool input_ended() const override { return offset_ == input_.size(); }
    void pause_capture() override { input_.clear(); offset_ = 0; }
    void enqueue_playback(Samples samples) override {
        if (reply_.size() + samples.size() > max_reply_samples) {
            throw std::runtime_error("Reply exceeds the audio buffer limit");
        }
        reply_.insert(reply_.end(), samples.begin(), samples.end());
    }

    bool finish_playback() override {
        write_wav(output_ / ("reply-" + std::to_string(turn_) + ".wav"), reply_);
        reply_.clear();
        return true;
    }

private:
    std::vector<std::filesystem::path> inputs_;
    std::filesystem::path output_;
    std::size_t turn_ = 0;
    std::size_t offset_ = 0;
    Clock::time_point next_capture_{};
    Samples input_;
    Samples reply_;
};
}

Samples RightChannelResampler::convert(std::span<const std::int16_t> stereo) {
    // Low-pass the right channel before decimating by three to avoid aliasing.
    static const auto coefficients = [] {
        std::array<double, taps> values{};
        constexpr double cutoff = 7000.0 / board_rate;
        constexpr double pi = std::numbers::pi_v<double>;
        double sum = 0.0;
        for (std::size_t index = 0; index < taps; ++index) {
            const double x = static_cast<double>(index) - (taps - 1) / 2.0;
            const double sinc = x == 0.0 ? 2 * cutoff : std::sin(2 * pi * cutoff * x) / (pi * x);
            const double window = 0.54 - 0.46 * std::cos(2 * pi * index / (taps - 1));
            values[index] = sinc * window;
            sum += values[index];
        }
        for (auto& value : values) value /= sum;
        return values;
    }();

    Samples result;
    result.reserve(stereo.size() / 6 + 1);
    for (std::size_t index = 1; index < stereo.size(); index += 2) {
        history_[position_] = stereo[index];
        position_ = (position_ + 1) % taps;
        if (++phase_ == 3) {
            phase_ = 0;
            double sample = 0.0;
            for (std::size_t tap = 0; tap < taps; ++tap) {
                sample += history_[(position_ + tap) % taps] * coefficients[tap];
            }
            result.push_back(static_cast<std::int16_t>(std::clamp(std::lround(sample), -32768L, 32767L)));
        }
    }
    return result;
}

std::unique_ptr<Audio> make_wav_audio(std::vector<std::filesystem::path> inputs,
                                      std::filesystem::path output) {
    return std::make_unique<WavAudio>(std::move(inputs), std::move(output));
}
