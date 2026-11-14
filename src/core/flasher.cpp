#include "flasher.h"

namespace dskymatic {

Flasher::Flasher()
    : m_t0(std::chrono::steady_clock::now())
{}

Flasher &Flasher::instance() {
    static Flasher f;
    return f;
}

long Flasher::elapsed_ms() const {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now() - m_t0).count();
}

bool Flasher::phase() const {
    // Lit on even half-cycles, dark on odd.
    return ((elapsed_ms() / kHalfPeriodMs) & 1) == 0;
}

} // namespace dskymatic
