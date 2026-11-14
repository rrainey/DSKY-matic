#pragma once

#include <chrono>

namespace dskymatic {

/*
 * Shared ~1.5 Hz square-wave clock for the Block II DSKY.
 *
 * The real DSKY has a single hardware flasher (a ~1.25-1.5 Hz square
 * wave in the Power Supply / Indicator Driver circuitry) that gates
 * the drive to the latching relays for the V/N FLASH, TEMP, KEY REL
 * and OPR ERR indicators.  Because the drive is gated on and off,
 * those relays physically cycle in lock-step at the flasher rate.
 *
 * To be faithful to that behavior, the visual blink in panel_render
 * and the relay-click dispatch in dsky_click both sample this one
 * instance, so the on-screen blink and the audible clicks stay
 * phase-aligned.
 *
 * Phase semantics:
 *   phase() == true   → lit half-cycle   (relays engaged)
 *   phase() == false  → dark half-cycle  (relays released)
 *
 * The clock is monotonic (steady_clock) and never resets after
 * construction, so phase alignment is stable across the run.
 */
class Flasher {
public:
    // 1.5 Hz full cycle → 333 ms per half (matches the "~1.5 cps"
    // commented in dsky_dispatch.cpp and the rate cited in the
    // Virtual AGC developer notes).
    static constexpr double kFlashHz      = 1.5;
    static constexpr int    kHalfPeriodMs =
        static_cast<int>(1000.0 / (kFlashHz * 2.0));

    bool phase() const;
    long elapsed_ms() const;

    static Flasher &instance();

private:
    Flasher();
    std::chrono::steady_clock::time_point m_t0;
};

inline Flasher &flasher() { return Flasher::instance(); }

} // namespace dskymatic
