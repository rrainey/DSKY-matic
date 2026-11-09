#pragma once

/*
 * DSKY relay-click dispatch.
 *
 * Ports the per-bank change-detection + playRelaySound() logic from
 * ~/src/dsky-hw/dmapp/DSKYmaticIF.cpp (processOutputChannelCommand case 010)
 * into a standalone module reusable by any DSKY-matic mode binary.
 *
 * Hook into the mode's AGC output channel drain:
 *
 *     dsky_click::init();
 *     // ... each time a ch010 output is observed:
 *     dsky_click::on_ch010(value);
 *     dsky_click::shutdown();
 *
 * on_ch010 decodes the DSPTAB op and compares each digit / sign position
 * against its cached value.  A click fires only when the specific
 * position actually changed — without this gate the AGC's periodic
 * rewrites would make the DSKY buzz.
 *
 * When built without DSKYMATIC_AUDIO (no OpenAL), all entry points are
 * no-ops; the mode binary sees the same API.
 */

namespace dsky_click {

/** Initialise OpenAL+alut, load one-relay-click.wav into one source per
 *  relay bank.  Idempotent; safe to call more than once.  Returns true
 *  on success (or when DSKYMATIC_AUDIO is off — no-op). */
bool init();

/** Shut down audio and free sources.  No-op when audio disabled. */
void shutdown();

/** Observe one AGC ch010 output word.  Fires per-bank clicks on change. */
void on_ch010(unsigned short value);

} // namespace dsky_click
