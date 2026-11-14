/*
 * DSKY relay-click dispatch — fires an OpenAL one-shot whenever an
 * AGC output word toggles a relay in the Block II DSKY's Indicator
 * Driver Modules.
 *
 * The DSKY has one latching relay per digit segment / sign / lamp
 * position.  The real hardware's audible "click" is that relay
 * engaging or disengaging.  We mirror that by:
 *
 *   • ch010 cmd 1..11  → per-digit / per-sign edge clicks (M, V, N,
 *                        R1..R3 banks).
 *   • ch010 cmd 12     → per-bit edge clicks for the lamp word
 *                        (NO ATT, GIMBAL LOCK, TRACKER, PROG).
 *   • ch011 bits 1..7  → per-bit edge clicks for the status-lamp
 *                        latches (COMP ACTY, UPLINK ACTY, TEMP,
 *                        KEY REL, OPR ERR).
 *   • pump_flash()     → one coalesced click per flasher edge while
 *                        any flashed indicator is asserted.
 *
 * Coalescing is deliberate (option "b"): on real hardware several
 * relays cycle simultaneously under the single flasher clock, which
 * would stack into a louder/sharper click rather than N independent
 * clicks.  A single representative click per edge keeps that
 * perceptual texture without flooding the output with overlapping
 * bank sources.
 *
 * All handlers edge-detect against cached state (BIT15 sentinel as
 * "no-value-yet"); otherwise the AGC's periodic rewrites would make
 * the DSKY buzz continuously.
 */

#include "dsky_click.h"

#include "flasher.h"

#include <cstdio>
#include <cstring>
#include <string>

#if DSKYMATIC_AUDIO
#include "Sound.h"
#include "SoundSource.h"
#include "SoundManager.h"
#endif

namespace {

#if DSKYMATIC_AUDIO
// One source per logical relay bank.  Digits / signs come first
// (matches the historical ordering in ~/src/dsky-hw/dmapp); the
// status-lamp and flash banks were added when ch011 and the
// cmd-12 lamp word were brought into the click path.
enum RelayBank {
    // Digit / sign banks (ch010 cmd 1..11).
    M1, M2,
    V1, V2,
    N1, N2,
    R11, R12, R13, R14, R15, R1S,
    R21, R22, R23, R24, R25, R2S,
    R31, R32, R33, R34, R35, R3S,

    // Lamp word (ch010 cmd 12).  In the real DSKY these are all
    // latching relays in the same Indicator Driver Modules as the
    // digit relays, just at different matrix positions.
    NO_ATT,
    GIMBAL_LOCK,
    TRACKER,
    PROG,

    // Status lamps (ch011 bits 2..7, minus V/N FLASH).
    COMP_ACTY,
    UPLINK,
    TEMP,
    KEYREL,
    OPR_ERR,

    // One coalesced source for the shared 1.5 Hz flasher edges.
    FLASH,

    BANK_COUNT
};

dso::SoundManager *g_pSoundManager = nullptr;
dso::SoundSource  *g_sources[BANK_COUNT] = { nullptr };
dso::Sound        *g_clicks [BANK_COUNT] = { nullptr };
#endif  // DSKYMATIC_AUDIO

// Cached digit / sign state for change detection.  BIT15 as "no-value-yet"
// so the first write after init always fires (matches dsky-hw startup).
constexpr unsigned short UNSET = 0xFFFFu;

unsigned short g_prog[2]  = {UNSET, UNSET};  // M1, M2
unsigned short g_verb[2]  = {UNSET, UNSET};  // V1, V2
unsigned short g_noun[2]  = {UNSET, UNSET};  // N1, N2
unsigned short g_r1[5]    = {UNSET, UNSET, UNSET, UNSET, UNSET};  // R11..R15
unsigned short g_r2[5]    = {UNSET, UNSET, UNSET, UNSET, UNSET};  // R21..R25
unsigned short g_r3[5]    = {UNSET, UNSET, UNSET, UNSET, UNSET};  // R31..R35
// Sign cache: lower bit = plus, next bit = minus.  dsky-hw tracked plus
// and minus as separate booleans; same information.
unsigned short g_sign[3]  = {UNSET, UNSET, UNSET};               // R1S, R2S, R3S

// Lamp-word (ch010 cmd 12) edge cache, one bit per lamp.
// UNSET = "no-value-yet"; otherwise 0/1.
unsigned char g_lw_no_att      = (unsigned char)0xFFu;
unsigned char g_lw_gimbal_lock = (unsigned char)0xFFu;
unsigned char g_lw_tracker     = (unsigned char)0xFFu;
unsigned char g_lw_prog        = (unsigned char)0xFFu;

// Ch011 status-lamp edge cache.
unsigned char g_sl_comp_acty = (unsigned char)0xFFu;
unsigned char g_sl_uplink    = (unsigned char)0xFFu;
unsigned char g_sl_temp      = (unsigned char)0xFFu;
unsigned char g_sl_keyrel    = (unsigned char)0xFFu;
unsigned char g_sl_oprerr    = (unsigned char)0xFFu;

// "Currently flashing" latches, updated on each ch011 write.  The
// flasher pump fires exactly one click per edge when any of these
// is set (option b: coalesced single click rather than per-bank).
bool g_flash_vn     = false;
bool g_flash_temp   = false;
bool g_flash_keyrel = false;
bool g_flash_oprerr = false;

// Flasher-pump prev-phase.  Seeded on first pump_flash() call.
bool g_flash_prev_phase    = false;
bool g_flash_prev_phase_set = false;

bool g_inited = false;
bool g_quiet  = false;

inline bool update_digit(unsigned short &cache, unsigned short next) {
    if (cache == next) return false;
    cache = next;
    return true;
}

// Plus bit encoded in BIT11 (0x400) on "2+" / "3+" ops; minus bit on
// "2-" / "3-" ops.  dsky-hw tracked both as separate booleans and only
// fired when either edge changed; here we pack both into one word.
inline bool update_sign(unsigned short &cache, unsigned short newbits) {
    if (cache == newbits) return false;
    cache = newbits;
    return true;
}

// Edge-detect for single-bit lamp caches (UNSET → 0xFF forces the
// first write to fire).  Returns true iff the stored value changed.
inline bool update_bit(unsigned char &cache, bool next) {
    const unsigned char n = next ? 1u : 0u;
    if (cache == n) return false;
    cache = n;
    return true;
}

void play(int bank) {
#if DSKYMATIC_AUDIO
    if (!g_inited) return;
    if (g_quiet)   return;
    if (bank < 0 || bank >= BANK_COUNT) return;
    if (g_clicks[bank]) g_clicks[bank]->Play(false);
#else
    (void)bank;
#endif
}

} // namespace

namespace dsky_click {

bool init() {
#if DSKYMATIC_AUDIO
    if (g_inited) return true;

    g_pSoundManager = new dso::SoundManager();

    const std::string audio_dir = DSKYMATIC_AUDIO_DIR;
    const std::string wav_path  = audio_dir + "/one-relay-click.wav";

    for (int i = 0; i < BANK_COUNT; ++i) {
        g_sources[i] = new dso::SoundSource(0, 0, 0);
        g_clicks[i]  = g_sources[i]->LoadSound(wav_path);
        if (!g_clicks[i]) {
            std::fprintf(stderr,
                "dsky_click: failed to load %s for bank %d — "
                "clicks disabled\n", wav_path.c_str(), i);
            // Partial-init cleanup; leave g_inited false so further calls
            // degrade to no-ops.
            return false;
        }
    }

    g_inited = true;
    std::fprintf(stderr, "dsky_click: %d banks loaded (%s)\n",
                 (int)BANK_COUNT, wav_path.c_str());
    return true;
#else
    return true;  // audio disabled at build — no-op API still works
#endif
}

void shutdown() {
#if DSKYMATIC_AUDIO
    if (!g_inited) return;
    for (int i = 0; i < BANK_COUNT; ++i) {
        delete g_clicks[i];  g_clicks[i]  = nullptr;
        delete g_sources[i]; g_sources[i] = nullptr;
    }
    delete g_pSoundManager; g_pSoundManager = nullptr;
    g_inited = false;
#endif
}

void on_ch010(unsigned short value) {
    // Decode matches apolloa's process_010 (which mirrors Virtual AGC's
    // channel 010 DSPTAB encoding).
    const int cmd    = (value >> 11) & 037;
    const unsigned short d1  = (value >>  5) & 037;
    const unsigned short d2  =  value        & 037;
    const unsigned short plus_minus = (value & (1 << 10)) ? 1 : 0;

    switch (cmd) {
    case  1:  // R3- side: R3D4, R3D5, and R3S minus-flag
        if (update_digit(g_r3[3], d1)) play(R34);
        if (update_digit(g_r3[4], d2)) play(R35);
        if (update_sign (g_sign[2], (unsigned short)(plus_minus ? 2 : 0) |
                                    (g_sign[2] & 1))) play(R3S);
        break;
    case  2:  // R3+ side: R3D2, R3D3, and R3S plus-flag
        if (update_digit(g_r3[1], d1)) play(R32);
        if (update_digit(g_r3[2], d2)) play(R33);
        if (update_sign (g_sign[2], (unsigned short)(plus_minus ? 1 : 0) |
                                    (g_sign[2] & 2))) play(R3S);
        break;
    case  3:  // R2D5, R3D1 (no sign)
        if (update_digit(g_r2[4], d1)) play(R25);
        if (update_digit(g_r3[0], d2)) play(R31);
        break;
    case  4:  // R2- side
        if (update_digit(g_r2[2], d1)) play(R23);
        if (update_digit(g_r2[3], d2)) play(R24);
        if (update_sign (g_sign[1], (unsigned short)(plus_minus ? 2 : 0) |
                                    (g_sign[1] & 1))) play(R2S);
        break;
    case  5:  // R2+ side
        if (update_digit(g_r2[0], d1)) play(R21);
        if (update_digit(g_r2[1], d2)) play(R22);
        if (update_sign (g_sign[1], (unsigned short)(plus_minus ? 1 : 0) |
                                    (g_sign[1] & 2))) play(R2S);
        break;
    case  6:  // R1- side
        if (update_digit(g_r1[3], d1)) play(R14);
        if (update_digit(g_r1[4], d2)) play(R15);
        if (update_sign (g_sign[0], (unsigned short)(plus_minus ? 2 : 0) |
                                    (g_sign[0] & 1))) play(R1S);
        break;
    case  7:  // R1+ side
        if (update_digit(g_r1[1], d1)) play(R12);
        if (update_digit(g_r1[2], d2)) play(R13);
        if (update_sign (g_sign[0], (unsigned short)(plus_minus ? 1 : 0) |
                                    (g_sign[0] & 2))) play(R1S);
        break;
    case  8:  // R1D1 only (upper-left, no second digit)
        if (update_digit(g_r1[0], d2)) play(R11);
        break;
    case  9:  // Noun
        if (update_digit(g_noun[0], d1)) play(N1);
        if (update_digit(g_noun[1], d2)) play(N2);
        break;
    case 10:  // Verb
        if (update_digit(g_verb[0], d1)) play(V1);
        if (update_digit(g_verb[1], d2)) play(V2);
        break;
    case 11:  // Prog (M1, M2)
        if (update_digit(g_prog[0], d1)) play(M1);
        if (update_digit(g_prog[1], d2)) play(M2);
        break;
    case 12:
        // Lamp word.  Bit numbering (1-indexed, per Virtual AGC docs):
        //   b4 NO ATT, b6 GIMBAL LOCK, b8 TRACKER, b9 PROG
        // (b1..b3,b5 are LM-only and never asserted by Colossus; left
        // out until we have a use for them.)
        if (update_bit(g_lw_no_att,      (value & (1 << 3)) != 0)) play(NO_ATT);
        if (update_bit(g_lw_gimbal_lock, (value & (1 << 5)) != 0)) play(GIMBAL_LOCK);
        if (update_bit(g_lw_tracker,     (value & (1 << 7)) != 0)) play(TRACKER);
        if (update_bit(g_lw_prog,        (value & (1 << 8)) != 0)) play(PROG);
        break;
    default:
        break;
    }
}

void on_ch011(unsigned short value) {
    // Bit numbering (1-indexed, per Virtual AGC docs):
    //   b2 COMP ACTY, b3 UPLINK ACTY, b4 TEMP, b5 KEY REL,
    //   b6 V/N FLASH, b7 OPR ERR
    const bool comp_acty = (value & (1 << 1)) != 0;
    const bool uplink    = (value & (1 << 2)) != 0;
    const bool temp      = (value & (1 << 3)) != 0;
    const bool keyrel    = (value & (1 << 4)) != 0;
    const bool vnflash   = (value & (1 << 5)) != 0;
    const bool oprerr    = (value & (1 << 6)) != 0;

    // Edge clicks for the solid-state latches.  Note: TEMP, KEY REL
    // and OPR ERR also click on each assertion edge even though they
    // subsequently flash — the initial relay engagement is itself a
    // click independent of the flasher-driven cycling.
    if (update_bit(g_sl_comp_acty, comp_acty)) play(COMP_ACTY);
    if (update_bit(g_sl_uplink,    uplink))    play(UPLINK);
    if (update_bit(g_sl_temp,      temp))      play(TEMP);
    if (update_bit(g_sl_keyrel,    keyrel))    play(KEYREL);
    if (update_bit(g_sl_oprerr,    oprerr))    play(OPR_ERR);

    // Latch current flash state for pump_flash() to consume.
    g_flash_vn     = vnflash;
    g_flash_temp   = temp;
    g_flash_keyrel = keyrel;
    g_flash_oprerr = oprerr;
}

void set_quiet(bool quiet) {
    g_quiet = quiet;
}

void pump_flash() {
#if DSKYMATIC_AUDIO
    if (!g_inited) return;

    const bool cur = dskymatic::flasher().phase();
    if (!g_flash_prev_phase_set) {
        g_flash_prev_phase     = cur;
        g_flash_prev_phase_set = true;
        return;
    }
    if (cur == g_flash_prev_phase) return;
    g_flash_prev_phase = cur;

    // Option (b): one coalesced click per flasher edge, regardless of
    // how many indicators are flashing simultaneously.  On the real
    // hardware the relays cycle in lock-step under a single flasher
    // clock, so N relays firing together sounds like one sharper
    // click rather than N independent events.
    if (g_flash_vn || g_flash_temp || g_flash_keyrel || g_flash_oprerr) {
        play(FLASH);
    }
#endif
}

} // namespace dsky_click
