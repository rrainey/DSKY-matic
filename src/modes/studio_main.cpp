// Interactive panel: loads the panel scene, drives SaturnStartupHelper +
// SaturnIBStack on the GLUT idle loop, feeds FDAI attitude into the
// FDAI sub-scene's rotations, dispatches DSKY button clicks into the
// AGC input channel queue.

#include "panel_pick.h"
#include "panel_render.h"
#include "panel_scene.h"
#include "panel_texture.h"
#include "dsky_click.h"
#include "dsky_dispatch.h"
#include "dsky_display_sink.h"

#include <SaturnStartupHelper.h>
#include <SaturnIBStack.h>
#include <SaturnIBAgcBridge.h>
#include <ChannelQueue.h>
#include <SimClock.h>
#include <save_state.h>
#include <agc_engine.h>

#include <GL/glut.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

static std::shared_ptr<panel::Scene> g_scene;
static SaturnStartupHelper g_start_helper;
static SaturnIBAgcBridge *g_lv = nullptr;
static int g_win_w = 1200;
static int g_win_h = 800;
static std::chrono::steady_clock::time_point g_last_tick;

// --save-state-time / --resume plumbing.  g_save_target_sec < 0 means
// no save is scheduled.  g_save_last_defer_log is rate-limited stderr
// output for the CDU-idle deferral.
static double g_save_target_sec = -1.0;
static bool   g_save_force      = false;      // ignore CDU-idle gate (testing)
static std::string g_save_resume_file;
static std::chrono::steady_clock::time_point g_save_last_defer_log;

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

// Toggle "flashing" on every segment inside a DSKYELSegment component.
// Matches DSKYELSegmentClass::DisplayFlash which sets Segments->Blinking.
static void dsky_set_flash_one(const std::string &comp_name, bool on) {
    panel::SceneNode *comp = find_node(g_scene->root.get(), comp_name);
    if (!comp || !comp->component_scene || !comp->component_scene->root) return;
    for (const char *seg : k_seg_names) {
        if (panel::SceneNode *s = find_node(comp->component_scene->root.get(), seg))
            s->flashing = on;
    }
}

// Scene-sink adapter: translates DskyDispatch field updates into
// updates on the GL panel scene (dsky_set_digit / _sign / _light /
// dsky_set_flash_one).
class SceneSink : public dskymatic::IDskyDisplaySink {
public:
    void set_digit(const std::string &field, int value) override {
        dsky_set_digit(field, value);
    }
    void set_sign(const std::string &field, int value) override {
        dsky_set_sign(field, value);
    }
    void set_light(const std::string &name, bool on) override {
        dsky_set_light(name, on);
    }
    void set_flashing(const std::string &group, bool flashing) override {
        // The dispatch emits one flashing call per group.  V/N digit
        // groups blink their underlying segments; status lamps blink
        // the lamp node directly (panel_render skips a frame when the
        // node's `flashing` flag is set and the shared flasher is in
        // its off-phase).
        if (group == "VERB") {
            dsky_set_flash_one("VD1", flashing);
            dsky_set_flash_one("VD2", flashing);
        } else if (group == "NOUN") {
            dsky_set_flash_one("ND1", flashing);
            dsky_set_flash_one("ND2", flashing);
        } else if (panel::SceneNode *n = find_node(g_scene->root.get(), group)) {
            n->flashing = flashing;
        }
    }
};

static SceneSink               g_scene_sink;
static dskymatic::DskyDispatch g_dispatch;


// --save-state-time: save the full sim state at the first tick on or
// after the requested time where the AGC is not driving the IMU CDUs.
// Writes saved-state-<ISO8601> and exits cleanly.
//
// Deferral is unbounded by design (per user: pick a valid save point);
// we log one line per second while we're waiting on CDU-idle so the
// user can see why.
static void maybe_save_state() {
    if (g_save_target_sec < 0.0 || !g_lv) return;

    sim::SimClock *clock = sim::GetSimClock();
    if (!clock) return;
    const double elapsed = clock->GetElapsedTime();
    if (elapsed < g_save_target_sec) return;

    // CDU-idle gate.  The AGC cycles the CDU error counter via ch012
    // BIT4 (CoarseAlignEnable) and drives fine-align pulses while the
    // IMU is in FINE_ALIGN mode.  Outside those two states the AGC is
    // not touching the stack-side CDU accumulators, which is what we
    // actually care about for a clean save point.  The IMU mode by
    // itself (INERTIAL_REFERENCE vs ATTITUDE_ERROR_DISPLAY etc.) says
    // nothing about pending CDU pulses — don't gate on it.
    const bool coarse_active = g_lv->GetCoarseAlignEnable();
    const ISS_IMU_Mode mode  = g_lv->GetIMUMode();
    const bool fine_active   = (mode == ISS_IMU_FINE_ALIGN);
    const bool cdu_idle      = !coarse_active && !fine_active;

    if (!cdu_idle && !g_save_force) {
        auto now = std::chrono::steady_clock::now();
        if (now - g_save_last_defer_log >= std::chrono::seconds(1)) {
            g_save_last_defer_log = now;
            std::fprintf(stderr,
                "save-state: deferred at t=%.3f (coarse=%d fine=%d imu_mode=%d)\n",
                elapsed, coarse_active ? 1 : 0, fine_active ? 1 : 0, (int)mode);
        }
        return;
    }

    dskymatic::SaveFile sf;
    sf.iso_timestamp   = dskymatic::make_iso_timestamp_local();
    sf.sim_elapsed_sec = elapsed;

    {
        dskymatic::BinaryWriter w;
        sim::GetSimClock()->SerializeState(w);
        sf.sections.push_back({ "sim_clock", std::move(w.bytes()) });
    }
    {
        dskymatic::BinaryWriter w;
        g_lv->SerializeState(w);
        sf.sections.push_back({ "sat_bridge", std::move(w.bytes()) });
    }

    std::string path = dskymatic::make_save_filename();
    if (!sf.write(path)) {
        std::fprintf(stderr, "save-state: write failed (%s)\n", path.c_str());
        std::exit(1);
    }
    std::fprintf(stderr,
        "save-state: wrote %s at sim-elapsed=%.3f s (%zu sections)\n",
        path.c_str(), elapsed, sf.sections.size());
    std::exit(0);
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
            g_dispatch.on_ch010(op.m_usValue, g_scene_sink);
            dsky_click::on_ch010(op.m_usValue);
        }
        else if (op.m_usChannel == 011) {
            g_dispatch.on_ch011(op.m_usValue, g_scene_sink);
            dsky_click::on_ch011(op.m_usValue);
            ++seen_011;
        }
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
    // One coalesced relay-click per shared-flasher edge (V/N FLASH +
    // TEMP/KEY REL/OPR ERR cycle together under one hardware clock).
    dsky_click::pump_flash();
    update_fdai_from_sim();
    update_text_from_sim();
    maybe_save_state();

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

static void print_usage(const char *argv0) {
    std::fprintf(stderr,
        "usage: %s [SCENE_PATH]\n"
        "           [--save-state-time SEC] [--resume FILE]\n"
        "\n"
        "  --save-state-time SEC   Save full sim state the first tick on or\n"
        "                          after sim-elapsed SEC seconds when the\n"
        "                          AGC is not driving the IMU CDUs (coarse\n"
        "                          align off, no fine-align axis, inertial\n"
        "                          reference).  Writes saved-state-<ISO8601>\n"
        "                          and exits cleanly.\n"
        "  --resume FILE           Load a saved state and continue from it.\n"
        "                          The ISS turn-on choreography is skipped.\n"
        "  --save-force            Bypass the CDU-idle save-point gate.  Only\n"
        "                          for testing the plumbing; may produce an\n"
        "                          inconsistent AGC/IMU state on resume.\n",
        argv0);
}

int main(int argc, char **argv) {
    const char *scene_path = "panel/panel.json";
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--save-state-time") == 0 && i + 1 < argc) {
            g_save_target_sec = std::atof(argv[++i]);
        } else if (std::strcmp(argv[i], "--resume") == 0 && i + 1 < argc) {
            g_save_resume_file = argv[++i];
        } else if (std::strcmp(argv[i], "--save-force") == 0) {
            g_save_force = true;
        } else if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        } else if (argv[i][0] != '-') {
            scene_path = argv[i];
        } else {
            std::fprintf(stderr, "unrecognized arg: %s\n", argv[i]);
            print_usage(argv[0]);
            return 2;
        }
    }

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

    // Relay-click audio (OpenAL+alut, per-bank).  Non-fatal if it fails —
    // the sim just runs silently.
    dsky_click::init();

    // Resume from a saved-state file, if requested.  The bridge has
    // already been constructed (ROM loaded, erasable patches applied);
    // we overwrite its state in-place and the sim picks up from there.
    if (!g_save_resume_file.empty()) {
        dskymatic::SaveFile sf;
        if (!dskymatic::SaveFile::read(g_save_resume_file, sf)) {
            std::fprintf(stderr, "resume: failed to read %s\n",
                         g_save_resume_file.c_str());
            return 1;
        }
        std::fprintf(stderr,
            "resume: %s (saved %s, sim-elapsed=%.3f s)\n",
            g_save_resume_file.c_str(),
            sf.iso_timestamp.c_str(), sf.sim_elapsed_sec);

        const dskymatic::Section *sec = nullptr;

        sec = sf.find("sim_clock");
        if (sec) {
            dskymatic::BinaryReader r(sec->data);
            if (!sim::GetSimClock()->DeserializeState(r))
                std::fprintf(stderr, "resume: sim_clock deserialize failed\n");
        } else {
            std::fprintf(stderr, "resume: missing sim_clock section\n");
        }

        sec = sf.find("sat_bridge");
        if (sec) {
            dskymatic::BinaryReader r(sec->data);
            if (!g_lv->DeserializeState(r))
                std::fprintf(stderr, "resume: sat_bridge deserialize failed\n");
        } else {
            std::fprintf(stderr, "resume: missing sat_bridge section\n");
        }


        // Repaint the DSKY from the restored AGC output caches.  The AGC
        // only emits a ch010/ch011 write on change, so without this the
        // scene sink starts blank and stays blank until the running
        // program next touches each field (e.g. P02's PROG digits
        // don't reappear until P11 overwrites them at liftoff).  yaAGC
        // keeps the last-written value per ch010 relay word in
        // OutputChannel10[AAAA] and the current ch011 bits in
        // InputChannel[011]; dispatch those through the normal path.
        // dsky_click is muted so priming its edge caches from UNSET→
        // saved-value doesn't fire a burst of spurious clicks.
        if (agc_t *agc = g_lv->DebugGetAGC()) {
            dsky_click::set_quiet(true);
            for (int cmd = 1; cmd <= 12; ++cmd) {
                unsigned short w = (unsigned short)agc->OutputChannel10[cmd];
                if (w == 0) continue;
                g_dispatch.on_ch010(w, g_scene_sink);
                dsky_click::on_ch010(w);
            }
            unsigned short ch11 = (unsigned short)agc->InputChannel[011];
            g_dispatch.on_ch011(ch11, g_scene_sink);
            dsky_click::on_ch011(ch11);
            dsky_click::set_quiet(false);
        }
    }

    g_last_tick = std::chrono::steady_clock::now();
    g_save_last_defer_log = g_last_tick;

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(key);
    glutMouseFunc(mouse);
    glutIdleFunc(idle);
    glutMainLoop();
    return 0;
}
