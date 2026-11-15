#include "audio.hpp"

#include <alsa/asoundlib.h>

#include <algorithm>
#include <cerrno>
#include <deque>
#include <stdexcept>
#include <utility>
#ifdef MICRODUCK_SPEEXDSP
#include "speech_gate.hpp"
#include <speex/speex_echo.h>
#include <speex/speex_preprocess.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <optional>
#include <thread>
#endif

namespace {
using Pcm = std::unique_ptr<snd_pcm_t, decltype(&snd_pcm_close)>;

void check(int result, const char* operation) {
    if (result < 0) {
        throw std::runtime_error(std::string(operation) + ": " + snd_strerror(result));
    }
}

Pcm open_pcm(const std::string& device, snd_pcm_stream_t stream, unsigned channels,
             unsigned rate) {
    snd_pcm_t* handle = nullptr;
    check(snd_pcm_open(&handle, device.c_str(), stream, SND_PCM_NONBLOCK), "Open ALSA device");
    Pcm pcm(handle, snd_pcm_close);
    check(snd_pcm_set_params(handle, SND_PCM_FORMAT_S16_LE, SND_PCM_ACCESS_RW_INTERLEAVED,
                             channels, rate, 1, 80000), "Configure ALSA device");
    return pcm;
}

class AlsaAudio final : public Audio {
public:
    AlsaAudio(std::string capture, std::string playback)
        : capture_device_(std::move(capture)), playback_device_(std::move(playback)) {}

    bool start_turn() override {
        resampler_ = {};
        mic_ = open_pcm(capture_device_, SND_PCM_STREAM_CAPTURE, 2, board_rate);
        check(snd_pcm_start(mic_.get()), "Start ALSA capture");
        return true;
    }

    Samples capture() override {
        std::array<std::int16_t, 2 * (board_rate / 50)> buffer{};
        const auto frames = snd_pcm_readi(mic_.get(), buffer.data(), buffer.size() / 2);
        if (frames == -EAGAIN) return {};
        if (frames < 0) {
            check(snd_pcm_recover(mic_.get(), static_cast<int>(frames), 1), "Recover ALSA capture");
            if (snd_pcm_state(mic_.get()) == SND_PCM_STATE_PREPARED) {
                check(snd_pcm_start(mic_.get()), "Restart ALSA capture");
            }
            resampler_ = {};
            return {};
        }
        return resampler_.convert(std::span(buffer).first(static_cast<std::size_t>(frames) * 2));
    }

    void pause_capture() override {
        // Release the codec before playback so the two directions can negotiate independently.
        mic_.reset();
    }

    void enqueue_playback(Samples samples) override {
        if (samples.empty()) return;
        if (pending_ + samples.size() > max_reply_samples) {
            throw std::runtime_error("ALSA playback queue is full");
        }
        if (!speaker_) {
            speaker_ = open_pcm(playback_device_, SND_PCM_STREAM_PLAYBACK, 1, output_rate);
        }
        pending_ += samples.size();
        playback_.push_back(std::move(samples));
    }

    void pump_playback() override {
        if (playback_.empty()) return;
        const auto& samples = playback_.front();
        const auto frames = snd_pcm_writei(speaker_.get(), samples.data() + played_,
                                            samples.size() - played_);
        if (frames == -EAGAIN) return;
        if (frames < 0) {
            check(snd_pcm_recover(speaker_.get(), static_cast<int>(frames), 1),
                  "Recover ALSA playback");
            return;
        }
        if (frames > 0 && snd_pcm_state(speaker_.get()) == SND_PCM_STATE_PREPARED) {
            check(snd_pcm_start(speaker_.get()), "Start ALSA playback");
        }
        played_ += static_cast<std::size_t>(frames);
        pending_ -= static_cast<std::size_t>(frames);
        if (played_ == samples.size()) {
            playback_.pop_front();
            played_ = 0;
        }
    }

    bool finish_playback() override {
        if (pending_ != 0) return false;
        if (!speaker_) return true;
        snd_pcm_sframes_t delay = 0;
        const auto result = snd_pcm_delay(speaker_.get(), &delay);
        if (result != -EPIPE) {
            check(result, "Check ALSA playback delay");
            if (delay > 0) return false;
        }
        speaker_.reset();
        return true;
    }

private:
    std::string capture_device_;
    std::string playback_device_;
    Pcm mic_{nullptr, snd_pcm_close};
    Pcm speaker_{nullptr, snd_pcm_close};
    RightChannelResampler resampler_;
    std::deque<Samples> playback_;
    std::size_t played_ = 0;
    std::size_t pending_ = 0;
};

#ifdef MICRODUCK_SPEEXDSP
constexpr std::size_t block = board_rate / 50;
constexpr std::int64_t nanoseconds_per_second = 1000000000;
constexpr std::size_t playback_lead = 2 * block;
constexpr std::int64_t echo_tail_ns = 200000000;
constexpr std::int64_t reference_margin_ns = 2000000;

std::int64_t frame_time(std::int64_t frames) {
    return frames * nanoseconds_per_second / board_rate;
}

void align_clock(std::optional<std::int64_t>& origin, std::int64_t estimate) {
    // ALSA snapshots can move by several frames between reads. Keep a stable
    // sample clock for the adaptive filter, while tracking slow hardware drift.
    if (!origin || std::abs(estimate - *origin) > 10000000) origin = estimate;
    else *origin += (estimate - *origin) / 32;
}

struct PcmPosition {
    std::int64_t time_ns;
    snd_pcm_sframes_t delay;
    snd_pcm_uframes_t available;
};

void enable_timestamps(snd_pcm_t* pcm) {
    snd_pcm_sw_params_t* params;
    snd_pcm_sw_params_alloca(&params);
    check(snd_pcm_sw_params_current(pcm, params), "Read ALSA software parameters");
    check(snd_pcm_sw_params_set_tstamp_mode(pcm, params, SND_PCM_TSTAMP_ENABLE),
          "Enable ALSA timestamps");
    check(snd_pcm_sw_params_set_tstamp_type(pcm, params, SND_PCM_TSTAMP_TYPE_MONOTONIC),
          "Select monotonic ALSA timestamps");
    check(snd_pcm_sw_params(pcm, params), "Apply ALSA timestamps");
}

PcmPosition position(snd_pcm_t* pcm) {
    snd_pcm_status_t* status;
    snd_pcm_status_alloca(&status);
    check(snd_pcm_status(pcm, status), "Read ALSA position");
    snd_htimestamp_t stamp{};
    snd_pcm_status_get_htstamp(status, &stamp);
    return {stamp.tv_sec * nanoseconds_per_second + stamp.tv_nsec,
            snd_pcm_status_get_delay(status), snd_pcm_status_get_avail(status)};
}

// Indices follow samples accepted by ALSA. The timestamp and queued delay map
// those indices to the same monotonic clock as the microphone.
class RenderHistory {
public:
    void append(std::span<const std::int16_t> stereo) {
        for (std::size_t i = 0; i < stereo.size(); i += 2) {
            samples_[written_++ % samples_.size()] = stereo[i];
        }
    }

    void update(PcmPosition position) {
        played_ = written_ - std::clamp<std::int64_t>(position.delay, 0, written_);
        if (played_ > 0) align_clock(origin_, position.time_ns - frame_time(played_));
    }

    std::optional<std::int16_t> sample(std::int64_t time_ns, bool stopped = false) const {
        if (!origin_ || written_ == 0) return std::nullopt;
        const auto index = (time_ns - *origin_) * board_rate / nanoseconds_per_second;
        const auto end = stopped ? played_ : written_;
        if (index < std::max<std::int64_t>(0, written_ - static_cast<std::int64_t>(samples_.size())) ||
            index >= end) {
            return std::nullopt;
        }
        return samples_[index % samples_.size()];
    }

    void reset() { written_ = played_ = 0; origin_.reset(); }
    std::int64_t written() const { return written_; }
    std::int64_t played() const { return played_; }

private:
    std::array<std::int16_t, board_rate> samples_{};
    std::int64_t written_ = 0;
    std::int64_t played_ = 0;
    std::optional<std::int64_t> origin_;
};

class DuplexAudio final : public Audio {
public:
    DuplexAudio(const std::string& capture_device, const std::string& playback_device)
        : mic_(open_pcm(capture_device, SND_PCM_STREAM_CAPTURE, 2, board_rate)),
          speaker_(open_pcm(playback_device, SND_PCM_STREAM_PLAYBACK, 2, board_rate)) {
        if (!echo_) throw std::runtime_error("Could not initialize SpeexDSP echo cancellation");
        int rate = board_rate;
        if (speex_echo_ctl(echo_.get(), SPEEX_ECHO_SET_SAMPLING_RATE, &rate) != 0) {
            throw std::runtime_error("Could not configure SpeexDSP echo cancellation");
        }
        if (!preprocess_) throw std::runtime_error("Could not initialize SpeexDSP preprocessing");
        configure_preprocess(SPEEX_PREPROCESS_SET_DENOISE, 1);
        configure_preprocess(SPEEX_PREPROCESS_SET_NOISE_SUPPRESS, -20);
        configure_preprocess(SPEEX_PREPROCESS_SET_ECHO_SUPPRESS, -45);
        configure_preprocess(SPEEX_PREPROCESS_SET_ECHO_SUPPRESS_ACTIVE, -25);
        if (speex_preprocess_ctl(preprocess_.get(), SPEEX_PREPROCESS_SET_ECHO_STATE, echo_.get()) != 0) {
            throw std::runtime_error("Could not connect SpeexDSP residual echo suppression");
        }
        enable_timestamps(mic_.get());
        enable_timestamps(speaker_.get());
    }

    ~DuplexAudio() override { snd_pcm_drop(speaker_.get()); }
    bool full_duplex() const override { return true; }

    bool start_turn() override {
        pump_playback();
        check(snd_pcm_start(mic_.get()), "Start duplex ALSA capture");
        return true;
    }

    void pause_capture() override {}

    Samples capture() override {
        const auto capture_position = position(mic_.get());
        const auto frames = snd_pcm_readi(mic_.get(), microphone_.data() + captured_ * 2,
                                          block - captured_);
        if (frames == -EAGAIN || frames == 0) return {};
        if (frames < 0) {
            ++capture_recoveries_;
            check(snd_pcm_recover(mic_.get(), static_cast<int>(frames), 1), "Recover duplex capture");
            if (snd_pcm_state(mic_.get()) == SND_PCM_STATE_PREPARED) {
                check(snd_pcm_start(mic_.get()), "Restart duplex capture");
            }
            captured_ = 0;
            read_frames_ = 0;
            capture_origin_.reset();
            speex_echo_state_reset(echo_.get());
            resampler_ = {};
            speech_gate_.reset();
            return {};
        }
        if (captured_ == 0) {
            align_clock(capture_origin_, capture_position.time_ns -
                        frame_time(read_frames_ + static_cast<std::int64_t>(capture_position.available)));
            capture_start_ = *capture_origin_ + frame_time(read_frames_);
        }
        read_frames_ += frames;
        captured_ += static_cast<std::size_t>(frames);
        if (captured_ < block) return {};
        captured_ = 0;

        std::array<std::int16_t, block> near{}, reference{}, clean{};
        render_.update(position(speaker_.get()));
        double reference_power = 0;
        for (std::size_t i = 0; i < block; ++i) {
            near[i] = microphone_[i * 2 + 1];
            const auto stamp = capture_start_ + frame_time(i) - reference_margin_ns;
            const auto current = render_.sample(stamp);
            reference[i] = current ? *current : tail_.sample(stamp, true).value_or(0);
            reference_power += static_cast<double>(reference[i]) * reference[i];
        }
        speex_echo_cancellation(echo_.get(), near.data(), reference.data(), clean.data());
        speex_preprocess_run(preprocess_.get(), clean.data());
        for (std::size_t i = 0; i < block; ++i) microphone_[i * 2 + 1] = clean[i];
        auto samples = resampler_.convert(microphone_);
        return speech_gate_.filter(std::move(samples), capture_start_ < last_render_ns_ + echo_tail_ns,
                                   std::sqrt(reference_power / block));
    }

    void enqueue_playback(Samples samples) override {
        if (queued_.size() + samples.size() * 2 > max_reply_samples * 2) {
            throw std::runtime_error("Duplex ALSA playback queue is full");
        }
        for (auto sample : samples) {
            // Keep the codec gain unchanged and reduce the acoustic coupling in duplex mode.
            sample = static_cast<std::int16_t>(sample / 2);
            queued_.push_back(static_cast<std::int16_t>((static_cast<int>(previous_) + sample) / 2));
            queued_.push_back(sample);
            previous_ = sample;
        }
    }

    void pump_playback() override {
        const auto state = snd_pcm_state(speaker_.get());
        const auto cursor = position(speaker_.get());
        const auto desired = static_cast<snd_pcm_sframes_t>(playback_lead) -
            std::max<snd_pcm_sframes_t>(0, cursor.delay);
        if (desired <= 0 && state == SND_PCM_STATE_RUNNING) return;
        const auto frames = static_cast<std::size_t>(
            std::clamp<snd_pcm_sframes_t>(desired, 1, playback_lead));
        std::array<std::int16_t, 2 * playback_lead> stereo{};
        const auto speech_frames = std::min(frames, queued_.size());
        for (std::size_t i = 0; i < speech_frames; ++i) stereo[2*i] = stereo[2*i + 1] = queued_[i];
        const auto accepted = snd_pcm_writei(speaker_.get(), stereo.data(), frames);
        if (accepted == -EAGAIN || accepted == 0) return;
        if (accepted < 0) {
            ++playback_recoveries_;
            check(snd_pcm_recover(speaker_.get(), static_cast<int>(accepted), 1),
                  "Recover duplex playback");
            if (snd_pcm_state(speaker_.get()) == SND_PCM_STATE_PREPARED) {
                render_.update({cursor.time_ns, 0, 0});
                retire_render();
            }
            return;
        }
        const auto written = static_cast<std::size_t>(accepted);
        const auto used = std::min(written, speech_frames);
        if (used) {
            last_speech_end_ = render_.written() + used;
            const auto queued_frames = std::max<snd_pcm_sframes_t>(0, cursor.delay);
            last_render_ns_ = cursor.time_ns + frame_time(queued_frames + static_cast<std::int64_t>(used));
        }
        queued_.erase(queued_.begin(), queued_.begin() + used);
        render_.append(std::span(stereo).first(2 * written));
        if (snd_pcm_state(speaker_.get()) == SND_PCM_STATE_PREPARED) {
            check(snd_pcm_start(speaker_.get()), "Start duplex ALSA playback");
        }
    }

    bool finish_playback() override {
        if (!queued_.empty()) return false;
        render_.update(position(speaker_.get()));
        if (render_.played() < last_speech_end_) return false;
        previous_ = 0;
        return true;
    }

    void interrupt_playback() override {
        queued_.clear();
        previous_ = 0;
        render_.update(position(speaker_.get()));
        check(snd_pcm_drop(speaker_.get()), "Drop duplex ALSA playback");
        retire_render();
        check(snd_pcm_prepare(speaker_.get()), "Prepare duplex ALSA playback");
    }

    unsigned capture_recoveries() const { return capture_recoveries_; }
    unsigned playback_recoveries() const { return playback_recoveries_; }

private:
    void configure_preprocess(int request, int value) {
        if (speex_preprocess_ctl(preprocess_.get(), request, &value) != 0) {
            throw std::runtime_error("Could not configure SpeexDSP preprocessing");
        }
    }

    void retire_render() {
        std::swap(render_, tail_);
        render_.reset();
        last_speech_end_ = 0;
    }

    Pcm mic_;
    Pcm speaker_;
    std::unique_ptr<SpeexEchoState, decltype(&speex_echo_state_destroy)> echo_{
        speex_echo_state_init(block, board_rate / 5), speex_echo_state_destroy};
    std::unique_ptr<SpeexPreprocessState, decltype(&speex_preprocess_state_destroy)> preprocess_{
        speex_preprocess_state_init(block, board_rate), speex_preprocess_state_destroy};
    RenderHistory render_, tail_;
    RightChannelResampler resampler_;
    SpeechGate speech_gate_;
    std::array<std::int16_t, 2 * block> microphone_{};
    std::deque<std::int16_t> queued_;
    std::size_t captured_ = 0;
    std::int64_t capture_start_ = 0;
    std::int64_t read_frames_ = 0;
    std::optional<std::int64_t> capture_origin_;
    std::int64_t last_speech_end_ = 0;
    std::int64_t last_render_ns_ = -echo_tail_ns;
    std::int16_t previous_ = 0;
    unsigned capture_recoveries_ = 0;
    unsigned playback_recoveries_ = 0;
};
#endif
}

std::unique_ptr<Audio> make_alsa_audio(std::string capture, std::string playback) {
    return std::make_unique<AlsaAudio>(std::move(capture), std::move(playback));
}

#ifdef MICRODUCK_SPEEXDSP
std::unique_ptr<Audio> make_duplex_audio(const std::string& capture, const std::string& playback) {
    return std::make_unique<DuplexAudio>(capture, playback);
}

void probe_duplex_audio(const std::string& capture, const std::string& playback,
                        const volatile std::sig_atomic_t& interrupted) {
    using Clock = std::chrono::steady_clock;
    using namespace std::chrono_literals;
    auto audio = std::make_unique<DuplexAudio>(capture, playback);
    audio->start_turn();
    const auto end = Clock::now() + 60s;
    std::size_t captured = 0;
    while (!interrupted && Clock::now() < end) {
        audio->pump_playback();
        captured += audio->capture().size();
        std::this_thread::sleep_for(2ms);
    }
    std::cout << "Duplex probe: " << captured << " samples at 16 kHz, "
              << audio->capture_recoveries() << " capture recoveries, "
              << audio->playback_recoveries() << " playback recoveries\n";
    if (interrupted || captured < 59 * input_rate ||
        audio->capture_recoveries() || audio->playback_recoveries()) {
        throw std::runtime_error("60-second duplex audio probe did not pass");
    }
}
#endif
