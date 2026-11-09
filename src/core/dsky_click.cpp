/*
 * DSKY relay-click dispatch — ports the logic in
 * ~/src/dsky-hw/dmapp/DSKYmaticIF.cpp (processOutputChannelCommand case 010).
 *
 * One OpenAL SoundSource per relay bank so clicks in different banks
 * overlap cleanly.  All banks play the same one-relay-click.wav;
 * polyphony is per-source.
 */

#include "dsky_click.h"

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
// Ordered to match ~/src/dsky-hw/dmapp/DSKYmaticIF.h enum SoundSources,
// minus the unused COMP_ACTY slot.
enum RelayBank {
    M1, M2,
    V1, V2,
    N1, N2,
    R11, R12, R13, R14, R15, R1S,
    R21, R22, R23, R24, R25, R2S,
    R31, R32, R33, R34, R35, R3S,
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

bool g_inited = false;

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

void play(int bank) {
#if DSKYMATIC_AUDIO
    if (!g_inited) return;
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
        // Lamps (NOATT / GIMBAL_LOCK / TRACKER / PROG).  dsky-hw did not
        // fire a click for lamp changes — leave as-is.
        break;
    default:
        break;
    }
}

} // namespace dsky_click
