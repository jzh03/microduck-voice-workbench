#include "audio.hpp"
#include "realtime.hpp"
#ifdef MICRODUCK_VOICE_CONTROL
#include "board_control.hpp"
#endif

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace {
volatile std::sig_atomic_t interrupted = 0;

void interrupt(int) { interrupted = 1; }

std::string environment(const char* name, const char* fallback = "") {
    const auto* value = std::getenv(name);
    return value && *value ? value : fallback;
}

void usage() {
    std::cout <<
        "microduck-voice --live [--full-duplex]\n"
        "microduck-voice --live --full-duplex [--control|--standup|--standup-walk]\n"
        "microduck-voice --probe-duplex\n"
        "microduck-voice --out DIRECTORY --wav TURN1.wav [--wav TURN2.wav ...]\n"
        "Set DASHSCOPE_REALTIME_URL and DASHSCOPE_API_KEY for qwen3.8-omni-flash-realtime.\n"
        "For local WAV tests, add --mock ws://127.0.0.1:PORT/.\n"
        "Linux live audio uses MICRODUCK_CAPTURE_PCM and MICRODUCK_PLAYBACK_PCM.\n";
}
}

int main(int argc, char** argv) {
    try {
        if (argc == 1 || (argc == 2 && std::string_view(argv[1]) == "--help")) {
            usage();
            return argc == 1 ? 2 : 0;
        }
        bool live = false;
        bool duplex = false;
        bool probe = false;
        bool control = false;
        bool standup = false;
        bool walk_after_standup = false;
        std::vector<std::filesystem::path> inputs;
        std::filesystem::path output;
        std::string mock;
        for (int index = 1; index < argc; ++index) {
            const std::string_view option(argv[index]);
            if (option == "--live") {
                live = true;
            } else if (option == "--full-duplex") {
                duplex = true;
            } else if (option == "--probe-duplex") {
                probe = true;
            } else if (option == "--control") {
                control = true;
            } else if (option == "--standup") {
                standup = true;
            } else if (option == "--standup-walk") {
                standup = true;
                walk_after_standup = true;
            } else if (option == "--wav" && index + 1 < argc) {
                inputs.emplace_back(argv[++index]);
            } else if (option == "--out" && index + 1 < argc) {
                output = argv[++index];
            } else if (option == "--mock" && index + 1 < argc) {
                mock = argv[++index];
            } else {
                throw std::runtime_error("Unknown option or missing argument. See --help.");
            }
        }
        std::signal(SIGINT, interrupt);
        std::signal(SIGTERM, interrupt);
        if (probe) {
            if (argc != 2) throw std::runtime_error("--probe-duplex takes no other options.");
#ifdef MICRODUCK_SPEEXDSP
            probe_duplex_audio(environment("MICRODUCK_CAPTURE_PCM", "plughw:aic3104,0"),
                               environment("MICRODUCK_PLAYBACK_PCM", "plughw:aic3104,0"),
                               interrupted);
            return 0;
#else
            throw std::runtime_error("Full duplex requires a Linux ALSA build with SpeexDSP.");
#endif
        }
        if (duplex && !live) throw std::runtime_error("--full-duplex requires --live.");
        if ((control && standup) || ((control || standup) && (!live || !duplex))) {
            throw std::runtime_error("Choose one control mode with full-duplex live audio.");
        }
#ifndef MICRODUCK_VOICE_CONTROL
        if (walk_after_standup || control || standup) {
            throw std::runtime_error("Voice control requires a Linux full-duplex build.");
        }
#endif
#ifndef MICRODUCK_SPEEXDSP
        if (duplex) throw std::runtime_error("Full duplex requires a Linux ALSA build with SpeexDSP.");
#endif
        if (live ? (!inputs.empty() || !output.empty() || !mock.empty())
                 : (inputs.empty() || output.empty())) {
            throw std::runtime_error("Choose --live or --wav with --out. See --help.");
        }

        const auto url = mock.empty() ? environment("DASHSCOPE_REALTIME_URL") : mock;
        const auto key = mock.empty() ? environment("DASHSCOPE_API_KEY") : "";
        if (mock.empty() && (!url.starts_with("wss://") || key.empty())) {
            throw std::runtime_error("Set a WSS DASHSCOPE_REALTIME_URL and DASHSCOPE_API_KEY.");
        }
        if (!mock.empty() && !mock.starts_with("ws://127.0.0.1:")) {
            throw std::runtime_error("Mock URL must use ws://127.0.0.1:PORT/.");
        }
        std::unique_ptr<Audio> audio;
        if (live) {
#ifdef MICRODUCK_ALSA
            const auto capture = environment("MICRODUCK_CAPTURE_PCM", "plughw:aic3104,0");
            const auto playback = environment("MICRODUCK_PLAYBACK_PCM", "plughw:aic3104,0");
#ifdef MICRODUCK_SPEEXDSP
            if (duplex) audio = make_duplex_audio(capture, playback);
            else
#endif
                audio = make_alsa_audio(capture, playback);
#else
            throw std::runtime_error("Live audio is available on Linux with ALSA.");
#endif
        } else {
            audio = make_wav_audio(std::move(inputs), std::move(output));
        }
#ifdef MICRODUCK_VOICE_CONTROL
        if (control || standup) {
            auto local = make_board_control(
                environment("MICRODUCK_CONTROL_SOCKET", "/run/microduck-voice/control.sock"), standup, walk_after_standup);
            run_realtime(*audio, url, key, interrupted, local.get());
            return 0;
        }
#endif
        run_realtime(*audio, url, key, interrupted);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "microduck-voice: " << error.what() << '\n';
        return 1;
    }
}
