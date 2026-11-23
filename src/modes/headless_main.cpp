/*
 * dskymatic-headless — mode 1.
 *
 * Runs the Saturn-IB + CMC simulation at wall-clock pace, with no GL
 * window, driving the DSKY and lamp state through an IDskyHardware
 * driver.  Intended to run on a Raspberry Pi inside a physical
 * DSKY-matic; the default StderrDskyHardware prints updates so the
 * mode is also useful on any desktop for debugging the AGC output.
 *
 * Relay-click audio fires alongside the hardware pushes via the same
 * dsky_click dispatch used by dskymatic-studio (OpenAL+alut, when
 * DSKYMATIC_AUDIO is enabled at build time).
 */

#include <SaturnStartupHelper.h>
#include <SaturnIBAgcBridge.h>
#include <ChannelQueue.h>

#include "dsky_dispatch.h"
#include "dsky_click.h"
#include "StderrDskyHardware.h"
#include "tmx_as205_config.h"

#include <chrono>
#include <cstdio>
#include <memory>
#include <thread>

using namespace dskymatic;

int main(int /*argc*/, char ** /*argv*/) {
    std::fprintf(stderr, "dskymatic-headless: starting (CWD must contain Colossus249.bin,\n"
                         "                      TM-X-62831_PAD_LOAD.txt, ddd-*-Colossus249.tsv)\n");

    SaturnStartupHelper start_helper;
    start_helper.SetBatchConfig(dskymatic::tmx_as205_config());
    SaturnIBAgcBridge *lv = start_helper.GetLaunchVehicle();
    std::fprintf(stderr, "dskymatic-headless: launch vehicle = %p\n", (void *)lv);

    StderrDskyHardware hw;
    hw.init();

    DskyDispatch dispatch;
    dsky_click::init();

    // Match studio's ~60 Hz tick cadence so the AGC advances at wall-clock.
    constexpr auto k_tick = std::chrono::microseconds(16000);
    auto last_tick = std::chrono::steady_clock::now();

    sim::SimClock *clock = sim::GetSimClock();

    for (;;) {
        auto now = std::chrono::steady_clock::now();
        if (now - last_tick < k_tick) {
            std::this_thread::sleep_for(k_tick - (now - last_tick));
            continue;
        }
        last_tick = now;

        if (clock) clock->ClockTimeUpdate();

        // Drain the bridge's AGC output queue, dispatch each ch010/011.
        while (!g_AGCChannelOutputQueue.empty()) {
            ChannelIOOperation op = g_AGCChannelOutputQueue.front();
            g_AGCChannelOutputQueue.pop_front();

            if      (op.m_usChannel == 010) {
                dispatch.on_ch010(op.m_usValue, hw);
                dsky_click::on_ch010(op.m_usValue);
            }
            else if (op.m_usChannel == 011) {
                dispatch.on_ch011(op.m_usValue, hw);
                dsky_click::on_ch011(op.m_usValue);
            }
            // Other channels (013/034/035 downlink, etc.) are consumed by
            // the bridge itself; nothing to do here.
        }

        // Shared-flasher coalesced click (one per edge while any
        // flashed indicator is asserted).
        dsky_click::pump_flash();

        // Keyboard polling from hardware → bridge AGC input queue.
        int kc = hw.poll_keycode();
        if (kc >= 0) {
            g_AGCChannelInputQueue.push_back(
                ChannelIOOperation(015, (unsigned short)kc, 037));
        }
    }

    dsky_click::shutdown();
    hw.shutdown();
    return 0;
}
