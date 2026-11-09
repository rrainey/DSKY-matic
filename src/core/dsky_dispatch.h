#pragma once

#include "dsky_display_sink.h"

namespace dskymatic {

/*
 * AGC channel 010 / 011 decoders, factored out of studio_main.cpp so
 * all three modes (studio / panel / headless) share one decode path.
 *
 * Each call pushes zero or more field updates into the given sink.
 * The dispatch is stateful for sign tracking (ch010 writes R1S/R2S/R3S
 * via alternating "+" and "-" ops); call reset_sign_state() if you
 * reinitialise.
 */
class DskyDispatch {
public:
    DskyDispatch();

    void on_ch010(unsigned short value, IDskyDisplaySink &sink);
    void on_ch011(unsigned short value, IDskyDisplaySink &sink);

    void reset_sign_state();

private:
    bool m_r1p, m_r1m;
    bool m_r2p, m_r2m;
    bool m_r3p, m_r3m;
};

} // namespace dskymatic
