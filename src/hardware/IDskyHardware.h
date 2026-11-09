#pragma once

#include "dsky_display_sink.h"

#include <cstdint>

namespace dskymatic {

/*
 * Pluggable DSKY-matic hardware driver.
 *
 * Extends IDskyDisplaySink with lifecycle and input-polling methods.
 * Mode binaries that drive real hardware instantiate a concrete
 * subclass (today: StderrDskyHardware for diagnostics; later: a
 * real Pi GPIO/I2C/UART driver ported from ~/src/dsky-hw/).
 *
 * The real DSKY-matic hardware (per ASSEMBLY.md) consists of:
 *   - DSKY-digital-indicator-replica (EL display, USB serial from Pi)
 *   - DSKY-keyboard-replica          (keyboard + proceed, USB serial)
 *   - DSKY-alarm-panel-replica       (lamp cluster, I2C from Pi)
 *
 * Panel and headless modes own one IDskyHardware instance and feed it
 * via the DskyDispatch from each AGC channel-010/011 write.
 *
 * The keyboard-input side returns AGC keycodes (same encoding the
 * panel click handler uses) via poll_keycode(); -1 means "no new key".
 */
class IDskyHardware : public IDskyDisplaySink {
public:
    ~IDskyHardware() override = default;

    /** Open hardware interfaces (serial, I2C, GPIO).  Returns true on
     *  success.  A failed init should still leave the sink usable as
     *  a no-op (so partial-hardware setups don't block the sim). */
    virtual bool init() = 0;

    /** Close hardware interfaces.  Idempotent. */
    virtual void shutdown() = 0;

    /** Poll for a pending key press.
     *  @return AGC keycode (as per ch015 encoding), or -1 if none. */
    virtual int poll_keycode() = 0;

    /** @return true if PROCEED has transitioned to pressed since the
     *          last call.  (Separate from keycodes — ch032 BIT14.) */
    virtual bool poll_proceed_pressed() { return false; }
};

} // namespace dskymatic
