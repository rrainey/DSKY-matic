#pragma once

#include <string>

namespace dskymatic {

/*
 * Abstract DSKY display sink.
 *
 * process_010 / process_011 decode the AGC channel words into per-field
 * updates (digits, signs, lights, flash state).  Each mode binary
 * implements this interface to route those updates where it wants:
 *
 *   studio   → SceneSink (updates GL scene nodes)
 *   panel    → scene-subset SceneSink + hardware for the DSKY portion
 *   headless → hardware driver only (optionally a stderr sink alongside)
 *
 * Field names match the scene-node names historically used by apolloa:
 *
 *   Digits: PD1 PD2 VD1 VD2 ND1 ND2
 *           R1D1 R1D2 R1D3 R1D4 R1D5
 *           R2D1 R2D2 R2D3 R2D4 R2D5
 *           R3D1 R3D2 R3D3 R3D4 R3D5
 *   Signs : R1S R2S R3S                (value -1, 0, +1)
 *   Lights: NOATT_LIGHT GIMBAL_LOCK_LIGHT TRACKER_LIGHT PROG_LIGHT
 *           ELCOMPACTY UPLINK_LIGHT TEMP_LIGHT KEYREL_LIGHT ...
 *   Flash : VERB_FLASH NOUN_FLASH       (ch011 BIT5)
 *
 * Any field a sink doesn't recognise should be silently ignored.
 */
class IDskyDisplaySink {
public:
    virtual ~IDskyDisplaySink() = default;

    /** Set a 7-segment digit.  value: -1 = blank, 0..9 = digit. */
    virtual void set_digit(const std::string &field, int value) = 0;

    /** Set a sign indicator.  value: -1, 0, +1. */
    virtual void set_sign(const std::string &field, int value) = 0;

    /** Set a lamp / lamp cluster on or off. */
    virtual void set_light(const std::string &name, bool on) = 0;

    /** Toggle flashing on a field group (e.g. "VERB"/"NOUN"). */
    virtual void set_flashing(const std::string &group, bool flashing) = 0;
};

} // namespace dskymatic
