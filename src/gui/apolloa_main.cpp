// Interactive panel: loads the panel scene, drives SaturnStartupHelper +
// SaturnIBStack on the GLUT idle loop, feeds FDAI attitude into the
// FDAI sub-scene's rotations, dispatches DSKY button clicks into the
// AGC input channel queue.

#include "panel_pick.h"
#include "panel_render.h"
#include "panel_scene.h"
#include "panel_texture.h"

#include <SaturnStartupHelper.h>
#include <SaturnIBStack.h>
#include <SaturnIBAgcBridge.h>
#include <ChannelQueue.h>
#include <agc_engine.h>

#include <GL/glut.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

static std::shared_ptr<panel::Scene> g_scene;
static SaturnStartupHelper g_start_helper;
static SaturnIBAgcBridge *g_lv = nullptr;
static int g_win_w = 1200;
static int g_win_h = 800;
static std::chrono::steady_clock::time_point g_last_tick;

// Find a scene node by Name across the tree (including into components).
static panel::SceneNode *find_node(panel::SceneNode *n, const std::string &name) {
    if (!n) return nullptr;
    if (n->name == name) return n;
    for (auto &c : n->children) {
        if (auto r = find_node(c.get(), name)) return r;
    }
    if (n->component_scene && n->component_scene->root) {
        if (auto r = find_node(n->component_scene->root.get(), name)) return r;
    }
    return nullptr;
}

static panel::SceneNode *g_attitude_ball    = nullptr;
static panel::SceneNode *g_roll_indexer     = nullptr;
static panel::SceneNode *g_roll_rate_arrow  = nullptr;
static panel::SceneNode *g_pitch_rate_arrow = nullptr;
static panel::SceneNode *g_yaw_rate_arrow   = nullptr;
static panel::SceneNode *g_roll_err_wiper   = nullptr;
static panel::SceneNode *g_pitch_err_wiper  = nullptr;
static panel::SceneNode *g_yaw_err_wiper    = nullptr;
static panel::SceneNode *g_status_window    = nullptr;
static panel::SceneNode *g_debug_window     = nullptr;

static void update_text_from_sim() {
    if (!g_lv) return;
    if (!g_status_window && g_scene && g_scene->root)
        g_status_window = find_node(g_scene->root.get(), "StatusWindow");
    if (!g_debug_window && g_scene && g_scene->root)
        g_debug_window  = find_node(g_scene->root.get(), "DebugWindow");
    if (g_status_window) {
        const char *t = g_lv->GetDownlinkText();
        if (t) g_status_window->text = t;
    }
    if (g_debug_window) {
        const char *t = g_lv->GetDebugText();
        if (t) g_debug_window->text = t;
    }
}

static void update_fdai_from_sim() {
    if (!g_lv) return;
    sim::Vector3d angles_deg, rates_dps, err_deg;
    g_lv->GetFDAIState(&angles_deg, &rates_dps, &err_deg);

    // Cache node pointers on first pass.
    if (!g_attitude_ball && g_scene && g_scene->root) {
        auto *r = g_scene->root.get();
        g_attitude_ball    = find_node(r, "AttitudeBall");
        g_roll_indexer     = find_node(r, "RollIndexerArrow");
        g_roll_rate_arrow  = find_node(r, "RollRateArrow");
        g_pitch_rate_arrow = find_node(r, "PitchRateArrow");
        g_yaw_rate_arrow   = find_node(r, "YawRateArrow");
        g_roll_err_wiper   = find_node(r, "RollErrorWiper");
        g_pitch_err_wiper  = find_node(r, "PitchErrorWiper");
        g_yaw_err_wiper    = find_node(r, "YawErrorWiper");
        std::fprintf(stderr,
            "sim FDAI nodes: ball=%p indexer=%p rates=(%p,%p,%p) errs=(%p,%p,%p)\n",
            (void*)g_attitude_ball, (void*)g_roll_indexer,
            (void*)g_roll_rate_arrow, (void*)g_pitch_rate_arrow, (void*)g_yaw_rate_arrow,
            (void*)g_roll_err_wiper, (void*)g_pitch_err_wiper, (void*)g_yaw_err_wiper);
    }

    // AttitudeBall axis mapping.
    //
    // The historical FDAIClass::Roll/Pitch/Yaw setters in the original FDAI code
    // do DynamicRotate(-_yaw, Y_AXIS), DynamicRotate(_pitch, X_AXIS),
    // DynamicRotate(_roll, Z_AXIS) -- and the AttitudeBall's Radii matrix
    // (see fdai.json) reorients the sphere so its local +Y axis (the
    // sphere's pole) points to world -X, local +X to world +Y, and
    // local +Z stays at world +Z.
    //
    // The port's draw_sphere applies rotation.z around the original Y_AXIS,
    // rotation.y around X_AXIS, rotation.x around Z_AXIS -- IN
    // gluSphere-local coords, then Radii+Rx(-90) reorients.  Working
    // out where each axis ends up visually:
    //     rotation.x (around gluSphere +Z) -> world -X   -> visual PITCH (negated)
    //     rotation.y (around gluSphere +X) -> world +Y   -> visual YAW
    //     rotation.z (around gluSphere +Y) -> world +Z   -> visual ROLL
    // (the render's -rotation.z on Y_AXIS and Radii+Rx(-90) combine to
    // give positive roll for positive rotation.z).
    //
    // GetFDAIState returns angles as (Roll=OGA, Pitch=IGA, Yaw=MGA) in
    // .x/.y/.z.  To get each value on its correct visual axis:
    //     visual ROLL  <- rotation.z  <-  Roll value  (angles_deg.x)
    //     visual YAW   <- rotation.y  <-  Yaw value   (angles_deg.z)
    //     visual PITCH <- -rotation.x <-  Pitch value (angles_deg.y)   (negated to undo axis sign flip)
    if (g_attitude_ball) {
        g_attitude_ball->rotation.x = -static_cast<float>(angles_deg.y);  // Pitch
        g_attitude_ball->rotation.y =  static_cast<float>(angles_deg.z);  // Yaw
        g_attitude_ball->rotation.z =  static_cast<float>(angles_deg.x);  // Roll
    }

    // RollIndexerArrow rotates around Z with the roll angle (matches
    // FDAIClass::Roll in FDAI.cpp).
    if (g_roll_indexer) {
        g_roll_indexer->rotation.z = static_cast<float>(angles_deg.x);
    }

    // Rate arrows: linear translate proportional to rate value.
    // Constants pulled from FDAI.cpp: X-axis for roll/yaw,
    // Y-axis for pitch, full-scale span matches the arrow's Location
    // to the edge of its channel.
    constexpr float kRateXSpan = 336.65f - 250.0f;
    constexpr float kRateYSpan = 359.312f - 247.44f;
    if (g_roll_rate_arrow)
        g_roll_rate_arrow->dynamic_translate.x  = kRateXSpan * static_cast<float>(rates_dps.x);
    if (g_pitch_rate_arrow)
        g_pitch_rate_arrow->dynamic_translate.y = kRateYSpan * static_cast<float>(rates_dps.y);
    if (g_yaw_rate_arrow)
        g_yaw_rate_arrow->dynamic_translate.x   = kRateXSpan * static_cast<float>(rates_dps.z);

    // Error wipers: similar linear translate, different span (constants
    // from FDAI.cpp {Roll,Pitch,Yaw}Error).  Error values are in
    // normalized units (-1..+1 = full deflection).
    constexpr float kErrXSpan = 319.22f - 250.34f;
    constexpr float kErrYSpan = 317.41f - 247.44f;
    if (g_roll_err_wiper)
        g_roll_err_wiper->dynamic_translate.x  = kErrXSpan * static_cast<float>(err_deg.x);
    if (g_pitch_err_wiper)
        g_pitch_err_wiper->dynamic_translate.y = kErrYSpan * static_cast<float>(err_deg.y);
    if (g_yaw_err_wiper)
        g_yaw_err_wiper->dynamic_translate.x   = kErrXSpan * static_cast<float>(err_deg.z);

#if 0
    static int frame = 0;
    if ((frame++ % 60) == 0) {
        std::fprintf(stderr,
            "FDAI RPY=(%.2f,%.2f,%.2f) rates=(%.2f,%.2f,%.2f) err=(%.2f,%.2f,%.2f)\n",
            angles_deg.x, angles_deg.y, angles_deg.z,
            rates_dps.x,  rates_dps.y,  rates_dps.z,
            err_deg.x,    err_deg.y,    err_deg.z);
    }
#endif
}

// -----------------------------------------------------------------------
// DSKY helpers: set digit / sign / lamp state on scene nodes.  Ported
// from the panel scene Process010Command / Process011Command.  The
// segment layout is standard 7-segment, tabulated from the
// DSKYELSegmentClass::DisplayValue body in the generated source.
// -----------------------------------------------------------------------

// The AGC channel encodes each digit as a 5-bit pattern that isn't 0-9;
// this table matches DecodeDigit in ApolloA.cpp.
static int decode_digit(int enc) {
    switch (enc) {
        case 21: return 0;
        case  3: return 1;
        case 25: return 2;
        case 27: return 3;
        case 15: return 4;
        case 30: return 5;
        case 28: return 6;
        case 19: return 7;
        case 29: return 8;
        case 31: return 9;
        default: return -1;
    }
}

// 7-segment truth table for digits -1..9.  Segment order matches the
// dsky_segment.json polygon names: UC UR LR LC LL UL C.
static const bool k_seg_table[11][7] = {
    // UC   UR    LR    LC    LL    UL    C     value
    { false,false,false,false,false,false,false }, // -1 (blank)
    { true, true, true, true, true, true, false }, //  0
    { false,true, true, false,false,false,false }, //  1
    { true, true, false,true, true, false,true  }, //  2
    { true, true, true, true, false,false,true  }, //  3
    { false,true, true, false,false,true, true  }, //  4
    { true, false,true, true, false,true, true  }, //  5
    { true, false,true, true, true, true, true  }, //  6
    { true, true, true, false,false,false,false }, //  7
    { true, true, true, true, true, true, true  }, //  8
    { true, true, true, true, false,true, true  }, //  9
};

static const char *k_seg_names[7] = {"UC","UR","LR","LC","LL","UL","C"};

static void dsky_set_digit(const std::string &comp_name, int value) {
    panel::SceneNode *comp = find_node(g_scene->root.get(), comp_name);
    if (!comp || !comp->component_scene || !comp->component_scene->root) return;
    if (value < -1 || value > 9) value = -1;
    const bool *seg = k_seg_table[value + 1];
    for (int i = 0; i < 7; ++i) {
        panel::SceneNode *s = find_node(comp->component_scene->root.get(), k_seg_names[i]);
        if (s) s->visible = seg[i];
    }
}

static void dsky_set_sign(const std::string &comp_name, int value) {
    panel::SceneNode *comp = find_node(g_scene->root.get(), comp_name);
    if (!comp || !comp->component_scene || !comp->component_scene->root) return;
    panel::SceneNode *minus = find_node(comp->component_scene->root.get(), "MINUS");
    panel::SceneNode *tp    = find_node(comp->component_scene->root.get(), "TP");
    panel::SceneNode *bp    = find_node(comp->component_scene->root.get(), "BP");
    if (minus) minus->visible = (value != 0);
    if (tp)    tp->visible    = (value > 0);
    if (bp)    bp->visible    = (value > 0);
}

static void dsky_set_light(const std::string &name, bool visible) {
    panel::SceneNode *n = find_node(g_scene->root.get(), name);
    if (n) n->visible = visible;
}

// Sign-state tracking per Bank1/2/3 -- Process010Command sets the "+"
// or "-" bit alternately and we combine them into an int for
// dsky_set_sign.
static bool g_r1p = false, g_r1m = false;
static bool g_r2p = false, g_r2m = false;
static bool g_r3p = false, g_r3m = false;
static int sign_value(bool plus, bool minus) {
    if (plus && !minus) return +1;
    if (minus && !plus) return -1;
    return 0;
}

static void process_010(unsigned short w) {
    int cmd  = (w >> 11) & 037;
    int nw1  = (w >>  5) & 037;
    int nw2  =  w        & 037;
    bool flg = (w & (1 << 10)) != 0;

    switch (cmd) {
    case  1: g_r3m = flg; dsky_set_sign("R3S", sign_value(g_r3p, g_r3m));
             dsky_set_digit("R3D4", decode_digit(nw1));
             dsky_set_digit("R3D5", decode_digit(nw2)); break;
    case  2: g_r3p = flg; dsky_set_sign("R3S", sign_value(g_r3p, g_r3m));
             dsky_set_digit("R3D2", decode_digit(nw1));
             dsky_set_digit("R3D3", decode_digit(nw2)); break;
    case  3: dsky_set_digit("R2D5", decode_digit(nw1));
             dsky_set_digit("R3D1", decode_digit(nw2)); break;
    case  4: g_r2m = flg; dsky_set_sign("R2S", sign_value(g_r2p, g_r2m));
             dsky_set_digit("R2D3", decode_digit(nw1));
             dsky_set_digit("R2D4", decode_digit(nw2)); break;
    case  5: g_r2p = flg; dsky_set_sign("R2S", sign_value(g_r2p, g_r2m));
             dsky_set_digit("R2D1", decode_digit(nw1));
             dsky_set_digit("R2D2", decode_digit(nw2)); break;
    case  6: g_r1m = flg; dsky_set_sign("R1S", sign_value(g_r1p, g_r1m));
             dsky_set_digit("R1D4", decode_digit(nw1));
             dsky_set_digit("R1D5", decode_digit(nw2)); break;
    case  7: g_r1p = flg; dsky_set_sign("R1S", sign_value(g_r1p, g_r1m));
             dsky_set_digit("R1D2", decode_digit(nw1));
             dsky_set_digit("R1D3", decode_digit(nw2)); break;
    case  8: dsky_set_digit("R1D1", decode_digit(nw2)); break;
    case  9: dsky_set_digit("ND1", decode_digit(nw1));
             dsky_set_digit("ND2", decode_digit(nw2)); break;
    case 10: dsky_set_digit("VD1", decode_digit(nw1));
             dsky_set_digit("VD2", decode_digit(nw2)); break;
    case 11: dsky_set_digit("PD1", decode_digit(nw1));
             dsky_set_digit("PD2", decode_digit(nw2)); break;
    case 12:
        dsky_set_light("NOATT_LIGHT",       (w & (1 <<  3)) != 0);
        dsky_set_light("GIMBAL_LOCK_LIGHT", (w & (1 <<  5)) != 0);
        dsky_set_light("TRACKER_LIGHT",     (w & (1 <<  7)) != 0);
        dsky_set_light("PROG_LIGHT",        (w & (1 <<  8)) != 0);
        break;
    default: break;
    }
}

// Toggle "flashing" on every segment inside a DSKYELSegment component.
// Matches DSKYELSegmentClass::DisplayFlash which sets Segments->Blinking.
static void dsky_set_flash(const std::string &comp_name, bool on) {
    panel::SceneNode *comp = find_node(g_scene->root.get(), comp_name);
    if (!comp || !comp->component_scene || !comp->component_scene->root) return;
    for (const char *seg : k_seg_names) {
        if (panel::SceneNode *s = find_node(comp->component_scene->root.get(), seg))
            s->flashing = on;
    }
}

static void process_011(unsigned short w) {
    // Diagnostic: log ch011 writes when any interesting bit changes.
    static unsigned short prev = 0xFFFFu;
#if 0
    if (w != prev) {
        std::fprintf(stderr,
            "  agc ch011 val=%05o  ELCA=%d UPLK=%d TEMP=%d KREL=%d FLASH=%d OPRERR=%d\n",
            w,
            (w >> 1) & 1, (w >> 2) & 1, (w >> 3) & 1,
            (w >> 4) & 1, (w >> 5) & 1, (w >> 6) & 1);
        prev = w;
    }
#endif
    dsky_set_light("ELCOMPACTY",     (w & (1 << 1)) != 0);
    dsky_set_light("UPLINK_LIGHT",   (w & (1 << 2)) != 0);
    dsky_set_light("TEMP_LIGHT",     (w & (1 << 3)) != 0);
    dsky_set_light("KEYREL_LIGHT",   (w & (1 << 4)) != 0);
    // BIT6 = flash VERB/NOUN digits (matches Process011Command in scene).
    bool flash = (w & (1 << 5)) != 0;
    dsky_set_flash("VD1", flash);
    dsky_set_flash("VD2", flash);
    dsky_set_flash("ND1", flash);
    dsky_set_flash("ND2", flash);
    dsky_set_light("OPR_ERR_LIGHT",  (w & (1 << 6)) != 0);
}


static void drain_agc_channel_output() {
    static int total = 0, seen_010 = 0, seen_011 = 0, seen_other = 0;
    while (!g_AGCChannelOutputQueue.empty()) {
        ChannelIOOperation op = g_AGCChannelOutputQueue.front();
        g_AGCChannelOutputQueue.pop_front();
        ++total;
        if (op.m_usChannel == 010) {
            ++seen_010;
            int cmd = (op.m_usValue >> 11) & 037;
#if 0
            if (cmd != 0) {
                std::fprintf(stderr,
                    "  agc ch010 cmd=%02d val=%05o w1=%02o w2=%02o flag=%d\n",
                    cmd, op.m_usValue,
                    (op.m_usValue >> 5) & 037,
                     op.m_usValue       & 037,
                    (op.m_usValue >> 10) & 1);
            }
#endif
            process_010(op.m_usValue);
        }
        else if (op.m_usChannel == 011) { process_011(op.m_usValue); ++seen_011; }
        else {
            ++seen_other;
#if 0
            // Log ALL non-010/011 channel writes.  V37 alarms (FAILREG) and
            // various ROM diagnostics land here; without this we go blind
            // whenever the AGC signals through anything but the DSKY.
            std::fprintf(stderr, "  agc ch%03o val=%05o mask=%05o\n",
                         op.m_usChannel, op.m_usValue, op.m_usMask);
#endif

        }
    }
#ifdef debug
    if ((total / 500) > ((total - 1) / 500)) {
        std::fprintf(stderr, "agc_out totals: ch010=%d ch011=%d other=%d\n",
                     seen_010, seen_011, seen_other);
    }
#endif
}

static void display() {
    glClearColor(0.10f, 0.10f, 0.12f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    panel::begin_frame(*g_scene, g_win_w, g_win_h);
    panel::render_scene(*g_scene);
    glutSwapBuffers();
}

static void idle() {
    // Rate-limit to ~60 Hz to match the original the original panel Calculate()
    // cadence (FrameRate: 60 in the panel scene).  GLUT's idle func fires
    // as fast as possible otherwise, and while the sim residual chunker
    // handles small dt correctly, wasted CPU per tick can starve the
    // AGC of real-time throughput on marginal machines and calls
    // ClockTimeUpdate() may get skipped when two ticks land in the
    // same microsecond.
    using clock = std::chrono::steady_clock;
    static auto last_tick = clock::now();
    static int tick_count = 0;
    static auto stat_epoch = clock::now();
    auto now = clock::now();
    if (now - last_tick < std::chrono::microseconds(16000)) return;
    last_tick = now;

    if (sim::SimClock *app = sim::GetSimClock()) {
        app->ClockTimeUpdate();
        ++tick_count;
        // Every 5 seconds print the observed tick rate + sim elapsed
        // time so we can see if the sim is keeping up with real time.
#if 0
        if (now - stat_epoch >= std::chrono::seconds(5)) {
            double wall = std::chrono::duration<double>(now - stat_epoch).count();
            std::fprintf(stderr,
                "sim tick: %.1f Hz, elapsed sim time = %.3f s (wall %.3f s)\n",
                tick_count / wall, app->GetElapsedTime(), wall);
            tick_count = 0;
            stat_epoch = now;
        }
#endif
    }
    drain_agc_channel_output();
    update_fdai_from_sim();
    update_text_from_sim();

    // Poll AGC input channels 030-033 (sim -> AGC discretes).  Log
    // whenever any change with per-bit decode so we can see exactly
    // what discretes the AGC is being fed.  Most bits are active-low
    // ("0 = TRUE"); the decode below prints the *asserted* signal.
    if (g_lv) {
        agc_t *pAGC = g_lv->DebugGetAGC();
        if (pAGC) {
            static unsigned short prev[4] = {0xFFFFu, 0xFFFFu, 0xFFFFu, 0xFFFFu};
            auto B = [](unsigned short v, int bit) -> int {
                return (v >> (bit - 1)) & 1;
            };
            for (int i = 0; i < 4; ++i) {
                int ch = 030 + i;
                unsigned short v = static_cast<unsigned short>(pAGC->InputChannel[ch]);
                if (v == prev[i]) continue;
                prev[i] = v;
                std::fprintf(stderr, "  sim ch%03o = %05o", ch, v);
                switch (ch) {
                case 030:
                    // All bits active-low: 0 = asserted.
                    std::fprintf(stderr,
                        "  [TEMP_OK=%d TURN_ON_REQ=%d IMU_FAIL=%d ICDU_FAIL=%d"
                        " IMU_CAGE=%d SC_CTL=%d IMU_OPER=%d GRR=%d"
                        " LIFTOFF=%d SIVB_ABORT=%d]",
                        !B(v, 15), !B(v, 14), !B(v, 13), !B(v, 12),
                        !B(v, 11),  B(v, 10), !B(v,  9), !B(v,  6),
                        !B(v,  5), !B(v,  4));
                    break;
                case 031:
                    // BIT15 active-low: 0 = IMU power on.  BIT14/13 are
                    // FREE/HOLD DAP mode discretes.  BIT1..12 rot/trans
                    // (active-high jet commands from THC/RHC).
                    std::fprintf(stderr,
                        "  [IMU_PWR=%d FREE=%d HOLD=%d THC/RHC=%04o]",
                        !B(v, 15), B(v, 14), B(v, 13), v & 07777);
                    break;
                case 032:
                    // BIT14 active-low: 0 = PROCEED pushbutton pressed.
                    std::fprintf(stderr, "  [PROCEED=%d]", !B(v, 14));
                    break;
                case 033:
                    // BIT11 UPTMFAST, BIT12 DNTMFAST, BIT13 PIPA_FAIL
                    // are latched flip-flops; 0 = fault asserted.
                    // BIT10 AGC_WARNING (0 = warning).
                    std::fprintf(stderr,
                        "  [PIPA_FAIL=%d DNTM_FAST=%d UPTM_FAST=%d"
                        " AGC_WARN=%d OSCILLATOR_FAIL=%d]",
                        !B(v, 13), !B(v, 12), !B(v, 11),
                        !B(v, 10), !B(v,  9));
                    break;
                }
                std::fprintf(stderr, "\n");
            }
        }
    }

    glutPostRedisplay();
}

static void reshape(int w, int h) { g_win_w = w; g_win_h = h; }

// AGC keycodes lifted verbatim from the ObjectCallback bodies in the
// the original panel application.  The DSKY keyboard writes into channel
// 015 with a 5-bit code (mask 037); the STBY button is a channel-032
// bit-toggle.
struct KeyBinding {
    const char *name;
    unsigned short channel;
    unsigned short down_value;   // value pushed on mouse-down
    unsigned short up_value;     // value pushed on mouse-up (0xFFFF = no-op)
    unsigned short mask;
};
static constexpr unsigned short K_NO_UP = 0xFFFFu;
static const KeyBinding g_key_bindings[] = {
    { "Key0",      015, 020,     K_NO_UP, 037 },
    { "Key1",      015, 001,     K_NO_UP, 037 },
    { "Key2",      015, 002,     K_NO_UP, 037 },
    { "Key3",      015, 003,     K_NO_UP, 037 },
    { "Key4",      015, 004,     K_NO_UP, 037 },
    { "Key5",      015, 005,     K_NO_UP, 037 },
    { "Key6",      015, 006,     K_NO_UP, 037 },
    { "Key7",      015, 007,     K_NO_UP, 037 },
    { "Key8",      015, 010,     K_NO_UP, 037 },
    { "Key9",      015, 011,     K_NO_UP, 037 },
    { "KeyPlus",   015, 032,     K_NO_UP, 037 },
    { "KeyMinus",  015, 033,     K_NO_UP, 037 },
    { "KeyCLR",    015, 036,     K_NO_UP, 037 },
    { "KeyKEYREL", 015, 031,     K_NO_UP, 037 },
    { "KeyENTR",   015, 034,     K_NO_UP, 037 },
    { "KeyRSET",   015, 022,     K_NO_UP, 037 },
    { "KeyVerb",   015, 021,     K_NO_UP, 037 },
    { "KeyNoun",   015, 037,     K_NO_UP, 037 },
    // STBY is a momentary contact: press = clear bit 020000 on ch 032,
    // release = set it.  Both events are required or the AGC sees STBY
    // as latched.  Values match Key{STBY}_ObjectCallback in the scene.
    { "KeySTBY",   032, 000000,  020000,  020000 },
};

static const KeyBinding *find_binding(const std::string &name) {
    for (const auto &b : g_key_bindings) {
        if (name == b.name) return &b;
    }
    return nullptr;
}

// Track which node was hit on mouse-down so we can fire the matching
// mouse-up event even if the cursor moved off-object before release.
// Also lets us implement the SaveButton (which acts on click, not on
// AGC input queue).
static panel::SceneNode *g_hit_down = nullptr;

static void mouse(int button, int state, int mx, int my) {
    if (button != GLUT_LEFT_BUTTON) return;
    if (!g_scene) return;

    if (state == GLUT_DOWN) {
        float wx, wy;
        panel::screen_to_world(*g_scene, g_win_w, g_win_h, mx, my, &wx, &wy);
        g_hit_down = panel::pick_at(*g_scene, wx, wy);
        if (!g_hit_down) return;

        // SaveButton: special-case, calls SaturnIBStack::SaveState.
        if (g_hit_down->name == "SaveButton") {
            if (g_lv) {
                g_lv->SaveState("StateSave");
                std::fprintf(stderr, "pick: SaveButton -> SaveState(\"StateSave\")\n");
            }
            return;
        }

        const KeyBinding *b = find_binding(g_hit_down->name);
        if (!b) {
            std::fprintf(stderr, "pick: %s (no keycode binding)\n",
                         g_hit_down->name.c_str());
            return;
        }
        ChannelIOOperation op(b->channel, b->down_value, b->mask);
        g_AGCChannelInputQueue.push_back(op);
        //std::fprintf(stderr, "pick DOWN: %s -> ch=%o val=%o mask=%o\n",
        //             g_hit_down->name.c_str(),
        //             b->channel, b->down_value, b->mask);
    } else if (state == GLUT_UP) {
        if (!g_hit_down) return;
        const KeyBinding *b = find_binding(g_hit_down->name);
        if (b && b->up_value != K_NO_UP) {
            ChannelIOOperation op(b->channel, b->up_value, b->mask);
            g_AGCChannelInputQueue.push_back(op);
            //std::fprintf(stderr, "pick UP:   %s -> ch=%o val=%o mask=%o\n",
            //            g_hit_down->name.c_str(),
            //           b->channel, b->up_value, b->mask);
        }
        g_hit_down = nullptr;
    }
}

static void key(unsigned char c, int, int) {
    if (c == 27 || c == 'q') std::exit(0);
    if (c == 's') { panel::g_render_spheres_only = !panel::g_render_spheres_only; }
}

int main(int argc, char **argv) {
    const char *scene_path = (argc > 1) ? argv[1] : "panel/panel.json";

    g_scene = panel::Scene::load(scene_path, ".");
    std::printf("scene: %dx%d, %zu top-level children, %zu textures\n",
                g_scene->width, g_scene->height,
                g_scene->root->children.size(),
                g_scene->textures.entries.size());

    glutInit(&argc, argv);
    // GLUT_MULTISAMPLE requests an MSAA-capable framebuffer; the driver
    // typically gives 4x by default (8x on most modern GPUs).  Combined
    // with glEnable(GL_MULTISAMPLE) below, this smooths polygon edges
    // for DSKY digits, FDAI needles, and the panel outline.
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_MULTISAMPLE);
    glutInitWindowSize(g_scene->width, g_scene->height);
    glutCreateWindow(g_scene->title.c_str());
    glEnable(GL_MULTISAMPLE);

    panel::load_scene_textures(*g_scene);

    // Match DSKYELSegmentClass::Initialize (from the original DSKY segment code)
    // and DSKYELSignClass::Initialize (from the original DSKY sign code) which
    // in the original run once at startup:
    //     DSKYELSegment: DisplayValue(-1)  -> all segments off (blank)
    //     DSKYELSign:    SignValue(0)      -> minus/tp/bp off (blank)
    // Without these the scene's "Visible: 1" default leaves every
    // segment lit, so every register digit reads "8" until the AGC
    // sends a specific value on channel 010.
    const char *k_digit_names[] = {
        "PD1","PD2","VD1","VD2","ND1","ND2",
        "R1D1","R1D2","R1D3","R1D4","R1D5",
        "R2D1","R2D2","R2D3","R2D4","R2D5",
        "R3D1","R3D2","R3D3","R3D4","R3D5",
    };
    for (const char *n : k_digit_names) dsky_set_digit(n, -1);
    dsky_set_sign("R1S", 0);
    dsky_set_sign("R2S", 0);
    dsky_set_sign("R3S", 0);

    // Bring up the sim.  SaturnStartupHelper::Initialize is lazy on
    // first GetLaunchVehicle() call.
    g_lv = g_start_helper.GetLaunchVehicle();
    std::fprintf(stderr, "sim: launch vehicle = %p\n", (void *)g_lv);

    // Quick sanity check: make sure Colossus249.bin was findable.
    // agc_load_binfile is silent on failure; if the ROM isn't on the
    // CWD the AGC just runs zeros.  Verify by fstat.
    {
        std::FILE *rom = std::fopen("Colossus249.bin", "rb");
        if (!rom) {
            std::fprintf(stderr,
                "*** WARNING: Colossus249.bin not found in CWD -- AGC will "
                "have no code to execute.  Run from the data/ directory.\n");
        } else {
            std::fseek(rom, 0, SEEK_END);
            long sz = std::ftell(rom);
            std::fclose(rom);
            std::fprintf(stderr, "sim: Colossus249.bin OK (%ld bytes)\n", sz);
        }
    }

    g_last_tick = std::chrono::steady_clock::now();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(key);
    glutMouseFunc(mouse);
    glutIdleFunc(idle);
    glutMainLoop();
    return 0;
}
