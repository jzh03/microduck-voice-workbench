#include "speech_gate.hpp"

#include <iostream>
#include <stdexcept>

int main() {
    SpeechGate gate;
    const auto expect = [&gate](int amplitude, bool playing, double reference, int expected) {
        const auto output = gate.filter(Samples(320, amplitude), playing, reference);
        if (output != Samples(320, expected)) throw std::runtime_error("Unexpected gated audio");
    };
    try {
        // The fixed threshold admitted this sustained startup echo.
        for (int i = 0; i < 6; ++i) expect(650, true, 2500, 0);
        // Reference history covers the short echo tail after a loud frame.
        for (int i = 0; i < 4; ++i) expect(650, true, 0, 0);
        expect(1800, true, 2500, 0);
        for (int i = 0; i < 5; ++i) expect(0, true, 2500, 0);

        // Clear speech still opens the gate during startup, with its onset.
        expect(1800, true, 2500, 0);
        expect(1600, true, 2500, 0);
        expect(1500, true, 2500, 0);
        expect(1400, true, 2500, 0);
        expect(1300, true, 2500, 0);
        expect(1200, true, 2500, 3600);
        expect(1100, true, 2500, 3200);

        gate.reset();
        for (int i = 0; i < 75; ++i) expect(0, true, 1000, 0);
        // Near speech can be quieter than the original gate and contain gaps.
        expect(265, true, 1000, 0);
        expect(35, true, 1000, 0);
        expect(262, true, 1000, 0);
        expect(45, true, 1000, 0);
        expect(260, true, 1000, 0);
        expect(250, true, 1000, 530);

        gate.reset();
        for (int i = 0; i < 5; ++i) expect(100, false, 0, 0);
        expect(100, false, 0, 100);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
