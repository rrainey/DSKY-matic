#pragma once

/*
 * DSKY relay-click dispatch.
 *
 * In the real Block II DSKY, every digit segment, sign, status lamp
 * and lamp-word indicator is switched through a latching relay in
 * the Indicator Driver Module (part 2003952-031, Relay Circuit
 * Assembly 2003910-021).  AGC output channels 010 and 011 drive
 * those relays; any state change produces an audible click.
 *
 *   on_ch010   — digit / sign relays (cmd 1..11) and the lamp word
 *                (cmd 12: NO ATT, GIMBAL LOCK, TRACKER, PROG, ...).
 *   on_ch011   — status-lamp latches (COMP ACTY, UPLINK ACTY,
 *                TEMP, KEY REL, OPR ERR).  Also latches the V/N
 *                FLASH + TEMP/KEY REL/OPR ERR flash state for the
 *                shared-flasher pump below.
 *   pump_flash — advance the shared ~1.5 Hz flasher; if any flashed
 *                indicator is currently asserted, fire one coalesced
 *                "flash edge" click (one click per edge total, not
 *                one per relay).
 *
 * Hook into the mode's AGC output drain:
 *
 *     dsky_click::init();
 *     // each tick:
 *     dsky_click::pump_flash();
 *     // for each AGC output word:
 *     dsky_click::on_ch010(value);   // channel 010
 *     dsky_click::on_ch011(value);   // channel 011
 *     dsky_click::shutdown();
 *
 * All handlers edge-detect against cached state: the AGC rewrites
 * the full DSKY register many times a second even when nothing has
 * changed, and we only click on an actual transition.  When built
 * without DSKYMATIC_AUDIO the entry points are no-ops.
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

/** Observe one AGC ch011 output word.  Fires status-lamp clicks on
 *  change; also latches V/N FLASH + TEMP/KEY REL/OPR ERR flash state
 *  for the flasher pump. */
void on_ch011(unsigned short value);

/** Advance the shared ~1.5 Hz DSKY flasher.  Call once per main-loop
 *  tick (not once per AGC output word).  Fires at most one "flash
 *  edge" click per half-period, and only when at least one flashed
 *  indicator is currently asserted. */
void pump_flash();

} // namespace dsky_click
