#pragma once

#include <array>
#include <csignal>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <vector>

using Samples = std::vector<std::int16_t>;
inline constexpr unsigned input_rate = 16000;
inline constexpr unsigned output_rate = 24000;
inline constexpr unsigned board_rate = 48000;
inline constexpr unsigned vad_silence_ms = 800;
inline constexpr std::size_t max_reply_samples = 60 * output_rate;

class Audio {
public:
    virtual ~Audio() = default;
    virtual bool full_duplex() const { return false; }
    virtual bool start_turn() = 0;
    virtual Samples capture() = 0;
    virtual bool input_ended() const { return false; }
    virtual void pause_capture() = 0;
    virtual void enqueue_playback(Samples samples) = 0;
    virtual void pump_playback() {}
    virtual bool finish_playback() = 0;
    virtual void interrupt_playback() {}
};

class RightChannelResampler {
public:
    Samples convert(std::span<const std::int16_t> stereo);
private:
    static constexpr std::size_t taps = 63;
    std::array<double, taps> history_{};
    std::size_t position_ = 0;
    unsigned phase_ = 0;
};

std::unique_ptr<Audio> make_wav_audio(std::vector<std::filesystem::path> inputs,
                                      std::filesystem::path output);
#ifdef MICRODUCK_ALSA
std::unique_ptr<Audio> make_alsa_audio(std::string capture, std::string playback);
#ifdef MICRODUCK_SPEEXDSP
std::unique_ptr<Audio> make_duplex_audio(const std::string& capture, const std::string& playback);
void probe_duplex_audio(const std::string& capture, const std::string& playback,
                        const volatile std::sig_atomic_t& interrupted);
#endif
#endif
