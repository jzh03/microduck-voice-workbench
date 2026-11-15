#include "audio.hpp"
#include "motion_tools.hpp"
#include "realtime.hpp"

#include <iostream>
#include <stdexcept>
#include <utility>

#ifdef MICRODUCK_MOCK_BOARD_CONTROL
#include "board_control.hpp"
#endif

namespace {
class TestAudio final : public Audio {
public:
    bool full_duplex() const override { return true; }
    bool start_turn() override {
        if (started_) throw std::runtime_error("duplex capture reopened");
        started_ = true;
        return true;
    }
    Samples capture() override { return Samples(320, 17); }
    void pause_capture() override { throw std::runtime_error("duplex capture paused"); }
    void enqueue_playback(Samples samples) override {
        std::cout << "Queued " << samples.front() << std::endl;
    }
    // Hold the simulated hardware buffer after the cloud completes.
    bool finish_playback() override { return false; }
    void interrupt_playback() override { std::cout << "Dropped playback" << std::endl; }
private:
    bool started_ = false;
};

class TestMotion final : public MotionTools {
public:
    void update_cloud() override {}
    bool request(std::string_view direction) override {
        std::cout << "Requested " << direction << std::endl;
        result_ = "completed";
        return true;
    }
    std::optional<std::string> take_result() override { return std::exchange(result_, std::nullopt); }
    void cancel_pending() override { result_.reset(); }

private:
    std::optional<std::string> result_;
};
}

int main(int argc, char** argv) {
    if (argc < 2 || argc > 4) return 2;
    try {
        TestAudio audio;
        const std::sig_atomic_t interrupted = 0;
        TestMotion mock;
        MotionTools* motion = argc == 3 ? &mock : nullptr;
#ifdef MICRODUCK_MOCK_BOARD_CONTROL
        std::unique_ptr<MotionTools> local;
        if (argc >= 3 && std::string_view(argv[2]) != "tools") {
            const std::string_view mode = argc == 4 ? argv[3] : "";
            local = make_board_control(argv[2], mode == "standup" || mode == "standup-walk",
                                       mode == "standup-walk");
            motion = local.get();
        }
#endif
        run_realtime(audio, argv[1], "", interrupted, motion);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
