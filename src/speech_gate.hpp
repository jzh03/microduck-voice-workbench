#pragma once

#include "audio.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <utility>

// Keep a fixed frame delay so confirming speech preserves its beginning
// without replaying audio in a burst or changing the input clock.
class SpeechGate {
public:
    Samples filter(Samples samples, bool speaker_active, double reference_rms) {
        if (samples.empty()) return samples;
        double power = 0;
        for (const auto sample : samples) power += static_cast<double>(sample) * sample;
        const auto rms = std::sqrt(power / samples.size());
        recent_reference_[reference_slot_] = reference_rms;
        reference_slot_ = (reference_slot_ + 1) % recent_reference_.size();
        const auto reference_peak = *std::max_element(recent_reference_.begin(), recent_reference_.end());
        const bool adapting = render_frames_ < 75;
        if (adapting && reference_rms >= 100) ++render_frames_;
        if (!speaker_active && hold_ == 0 && rms < std::max(85.0, noise_ * 2)) {
            noise_ = 0.98 * noise_ + 0.02 * rms;
        }
        // After echo cancellation has adapted, tolerate short dips between
        // syllables while still rejecting isolated transients.
        const auto threshold = speaker_active
            ? std::max({adapting ? 360.0 : 160.0, noise_ * 4,
                        reference_peak * (adapting ? 0.38 : 0.25)})
            : std::max(65.0, noise_ * 3);
        const bool voiced = rms >= threshold;
        consecutive_ = voiced ? std::min(consecutive_ + 1, 3u) : 0;
        recent_voice_[voice_slot_] = voiced;
        voice_slot_ = (voice_slot_ + 1) % recent_voice_.size();
        const bool confirmed = speaker_active && !adapting
            ? std::count(recent_voice_.begin(), recent_voice_.end(), true) >= 3
            : consecutive_ >= 3;
        if ((voiced && hold_ > 0) ||
            (voiced && confirmed)) {
            hold_ = 16;
        }
        if (speaker_active) {
            // Compensate for the residual suppressor on frames the gate passes.
            for (auto& sample : samples) {
                sample = static_cast<std::int16_t>(std::clamp<int>(sample * 2, -32768, 32767));
            }
        }
        delay_.push_back(std::move(samples));
        if (delay_.size() <= recent_voice_.size()) return Samples(delay_.back().size(), 0);
        auto output = std::move(delay_.front());
        delay_.pop_front();
        if (hold_ > 0) --hold_;
        else std::fill(output.begin(), output.end(), 0);
        return output;
    }

    void reset() { *this = {}; }

private:
    std::deque<Samples> delay_;
    std::array<double, 8> recent_reference_{};
    std::size_t reference_slot_ = 0;
    unsigned render_frames_ = 0;
    unsigned consecutive_ = 0;
    double noise_ = 20;
    std::array<bool, 5> recent_voice_{};
    std::size_t voice_slot_ = 0;
    unsigned hold_ = 0;
};
