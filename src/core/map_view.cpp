#include "map_view.h"

#include "save_state.h"   // BinaryReader for portable LE reads

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

namespace dskymatic {

namespace {

constexpr double kDegToRad   = 3.14159265358979323846 / 180.0;
constexpr double kRadToDeg   = 180.0 / 3.14159265358979323846;
constexpr double kEarthR_m   = 6378136.3;         // WGS-84 equatorial radius
constexpr double kJD_J2000   = 2451545.0;
constexpr double kM_PER_NMI  = 1852.0;
constexpr double kM_PER_FT   = 0.3048;

// --- Visual tuning ---------------------------------------------------
constexpr float kBgR = 0.00f, kBgG = 0.00f, kBgB = 0.00f;
constexpr float kLandR = 0.28f, kLandG = 0.28f, kLandB = 0.28f;
constexpr float kCoastR = 0.55f, kCoastG = 0.55f, kCoastB = 0.55f;
constexpr float kDayFactor   = 1.00f;  // multiplier on land color, day side
constexpr float kNightFactor = 0.22f;  // multiplier on land color, night side
constexpr float kTerminatorSoftnessDeg = 8.0f;  // ±range of the soft blend

// Camera tunables.  The camera is placed on the vehicle's meridional
// plane, south of the vehicle, at a slant distance that scales with
// the vehicle's ECI speed (see slant_dist_for_velocity_mps), and at
// an altitude chosen (per-frame) so the Earth's north limb projects
// (FOV/2 − kCamLimbMarginDeg) above the vehicle on screen.  That
// keeps the vehicle framed with enough headroom to always show the
// curving horizon above it.  See solve_camera_altitude().
// Camera slant distance is a linear function of the vehicle's ECI
// speed, holding "close-in" on the pad and rising as the vehicle
// approaches orbital velocity.  Rationale: at low speed the story is
// the lofting of the vehicle off the pad / off the cape, which reads
// best with a close-in chase camera; at orbital speed the story is
// the ground track across the globe, which needs a pulled-back view.
constexpr double kCamFOVdeg        =   28.0;
constexpr double kCamLimbMarginDeg =    1.0;   // limb lands (FOV/2 − this) above V
constexpr double kCamSlantNearKm   =  332.0;   // at or below kCamVNearFPS
constexpr double kCamSlantFarKm    = 1108.0;   // at or above kCamVFarFPS
constexpr double kCamVNearFPS      =  4000.0;
constexpr double kCamVFarFPS       = 26500.0;

static double slant_dist_for_velocity_mps(double v_mps) {
    double v_fps = v_mps / kM_PER_FT;
    if (v_fps <= kCamVNearFPS) return kCamSlantNearKm;
    if (v_fps >= kCamVFarFPS)  return kCamSlantFarKm;
    double t = (v_fps - kCamVNearFPS) / (kCamVFarFPS - kCamVNearFPS);
    return kCamSlantNearKm + t * (kCamSlantFarKm - kCamSlantNearKm);
}

// Timeline (AS-205): approximate mission-event times post-liftoff (sec).
// label_side = +1 → label above the baseline, 0 → below, −1 → tick only
// (no label).  Alternating sides keeps the close-cluster events (MECO /
// STAGING / IGM START around T+150-180 s) readable.
struct TimelineEvent { double t_sec; const char *label; int label_side; };
constexpr TimelineEvent kAS205Events[] = {
    {   0.0, "LIFTOFF",    0 },
    { 144.0, nullptr,     -1 },  // IECO — tick only
    { 152.0, "MECO",       1 },  // above
    { 155.0, "STAGING",    0 },  // below  (S-IB sep + S-IVB ignition)
    { 180.0, "IGM\nSTART", 1 },  // above, two-line left-justified
    { 614.0, "SECO / INS", 0 },  // below
};

// --- Math helpers ----------------------------------------------------

struct V3 {
    double x, y, z;
    V3 operator+(const V3 &b) const { return {x+b.x, y+b.y, z+b.z}; }
    V3 operator-(const V3 &b) const { return {x-b.x, y-b.y, z-b.z}; }
    V3 operator*(double s) const    { return {x*s, y*s, z*s}; }
};

static double dot(const V3 &a, const V3 &b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static V3     cross(const V3 &a, const V3 &b) {
    return {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x};
}
static double length(const V3 &a) { return std::sqrt(dot(a, a)); }
static V3 normalize(const V3 &a) {
    double n = length(a);
    if (n < 1e-15) return {0, 0, 1};
    return {a.x / n, a.y / n, a.z / n};
}

// Given vehicle altitude h_v [m], slant distance D [m], and the
// desired framing angle beta_deg (positive = north limb this many
// degrees above vehicle on screen), solve for the camera altitude
// h_c [m] above Earth's surface and the angular separation eta_deg
// between camera and vehicle (both on the vehicle's meridian).
//
// Geometry (all in the meridional plane, Earth sphere radius R):
//   r_v = R + h_v, r_c = R + h_c
//   cos η = (r_v² + r_c² − D²) / (2 r_v r_c)                       (1)
//   sin δ = sin η · r_v / D       (angle at C between nadir & view) (2)
//   α     = asin(R / r_c)         (Earth's angular half-size)       (3)
//   β     = α − δ                 (north limb above V on screen)    (4)
//
// Solving (4) for r_c gives one equation in one unknown.  The
// function α − δ is non-monotonic over the whole r_c range (it
// dips near a minimum where the vehicle is right at the camera's
// horizon) but is monotonic from the minimum upward.  Bisecting
// in [r_v + 200 km, r_v + D − 10 km] stays on the upper branch,
// which is the "camera pulled high enough that the limb floats
// above the vehicle" regime we want.
static void solve_camera_altitude(double h_v_m, double D_m, double beta_deg,
                                  double &h_c_m, double &eta_deg)
{
    constexpr double kEarthR_m_ = 6378136.3;
    const double r_v   = kEarthR_m_ + h_v_m;
    const double beta  = beta_deg * kDegToRad;

    auto limb_above_V = [&](double r_c) -> double {
        double cos_eta = (r_v*r_v + r_c*r_c - D_m*D_m) / (2.0 * r_v * r_c);
        cos_eta = std::max(-1.0, std::min(1.0, cos_eta));
        double eta = std::acos(cos_eta);
        double sin_eta = std::sin(eta);
        double sin_del = sin_eta * r_v / D_m;
        sin_del = std::max(-1.0, std::min(1.0, sin_del));
        double delta = std::asin(sin_del);
        double alpha = std::asin(std::min(1.0, kEarthR_m_ / r_c));
        return alpha - delta;
    };

    // Floor scales with D so the search range contains the solution
    // even when the camera is pulled in close (small D).  15 % of D
    // puts the floor comfortably above the limb-at-horizon dip while
    // still allowing low-altitude solutions for small D.
    double lo = r_v + std::max(30000.0, 0.15 * D_m);
    double hi = r_v + D_m - 10000.0;         // just below η = 0 (pure overhead)
    if (hi < lo) hi = lo;                    // D_m smaller than margin — degenerate
    // Make sure the target lies in [f(lo), f(hi)]; clamp otherwise.
    double f_lo = limb_above_V(lo);
    double f_hi = limb_above_V(hi);
    double target = beta;                    // = β in radians
    if (target <= f_lo) { h_c_m = lo - kEarthR_m_; }
    else if (target >= f_hi) { h_c_m = hi - kEarthR_m_; }
    else {
        for (int i = 0; i < 40; ++i) {
            double mid = 0.5 * (lo + hi);
            if (limb_above_V(mid) < target) lo = mid;
            else                            hi = mid;
        }
        h_c_m = 0.5 * (lo + hi) - kEarthR_m_;
    }

    double r_c = kEarthR_m_ + h_c_m;
    double cos_eta = (r_v*r_v + r_c*r_c - D_m*D_m) / (2.0 * r_v * r_c);
    cos_eta = std::max(-1.0, std::min(1.0, cos_eta));
    eta_deg = std::acos(cos_eta) * kRadToDeg;
}

// Lat/Lon (deg) → unit vector in ECEF/ECI (Z = north pole).
static V3 latlon_to_unit(double lat_deg, double lon_deg) {
    const double la = lat_deg * kDegToRad;
    const double lo = lon_deg * kDegToRad;
    const double c  = std::cos(la);
    return { c * std::cos(lo), c * std::sin(lo), std::sin(la) };
}

static void unit_to_latlon(const V3 &u, double &lat_deg, double &lon_deg) {
    V3 n = normalize(u);
    lat_deg = std::asin(std::max(-1.0, std::min(1.0, n.z))) * kRadToDeg;
    lon_deg = std::atan2(n.y, n.x) * kRadToDeg;
}

// --- Sun ephemeris (low-precision; Meeus "Astronomical Algorithms") --

static double julian_date(int year, int month, int day,
                          int hour, int minute, double sec) {
    int y = year, m = month;
    if (m <= 2) { y -= 1; m += 12; }
    int A = y / 100;
    int B = 2 - A + A / 4;
    double jd0 = std::floor(365.25 * (y + 4716))
               + std::floor(30.6001 * (m + 1))
               + day + B - 1524.5;
    double frac = (hour + minute / 60.0 + sec / 3600.0) / 24.0;
    return jd0 + frac;
}

// Returns unit vector toward the Sun in ECI (equatorial, Z=north).
static V3 sun_eci_unit(int y, int mo, int d, int h, int mi, double s) {
    double jd = julian_date(y, mo, d, h, mi, s);
    double n  = jd - kJD_J2000;
    double L  = std::fmod(280.460 + 0.9856474 * n, 360.0);
    double g  = std::fmod(357.528 + 0.9856003 * n, 360.0);
    if (L < 0) L += 360; if (g < 0) g += 360;
    double lam = L + 1.915 * std::sin(g * kDegToRad)
                   + 0.020 * std::sin(2 * g * kDegToRad);
    double eps = 23.439 - 0.0000004 * n;
    double lam_r = lam * kDegToRad;
    double eps_r = eps * kDegToRad;
    double ra  = std::atan2(std::cos(eps_r) * std::sin(lam_r), std::cos(lam_r));
    double dec = std::asin(std::sin(eps_r) * std::sin(lam_r));
    // GMST in hours → convert to radians for Earth-rotation offset
    double gmst_h = std::fmod(18.697374558 + 24.06570982441908 * n, 24.0);
    if (gmst_h < 0) gmst_h += 24;
    double gmst_rad = gmst_h * 15.0 * kDegToRad;
    // RA is in inertial ECI frame; to get the sub-solar longitude we
    // would subtract gmst_rad, but the sun direction IN ECI is just
    // (RA, Dec), no Earth rotation applied.  Land/trajectory also live
    // in a frame rotating with Earth — for a diagrammatic stylized
    // view we draw everything in a snapshot of the ECEF frame at the
    // current GMST.  That means we need the sun direction in ECEF:
    //   ECEF azimuth = RA - GMST
    double az = ra - gmst_rad;
    return { std::cos(dec) * std::cos(az),
             std::cos(dec) * std::sin(az),
             std::sin(dec) };
}

// --- Binary loaders --------------------------------------------------

struct TriBuf {
    std::vector<float> xyz;   // 9 floats per triangle (3 verts × 3 coords, unit sphere)
    std::vector<float> ll;    // 6 floats per triangle (3 verts × 2 lat,lon) -- kept for terminator
    size_t count() const { return xyz.size() / 9; }
};

struct LineBuf {
    // Concatenated line strips, one entry per polyline.
    // xyz is the vertex stream (3 floats/vertex).
    // starts[i] / counts[i] slice into xyz for polyline i.
    std::vector<float>      xyz;
    std::vector<float>      ll;
    std::vector<GLint>      starts;
    std::vector<GLsizei>    counts;
};

static bool load_tri_file(const std::string &path, TriBuf &out, const char *magic) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f.is_open()) {
        std::fprintf(stderr, "map_view: cannot open %s\n", path.c_str());
        return false;
    }
    auto sz = f.tellg();
    f.seekg(0, std::ios::beg);
    std::vector<uint8_t> buf((size_t)sz);
    f.read((char *)buf.data(), sz);
    BinaryReader r(buf);
    char m[4]; r.read_bytes(m, 4);
    if (std::memcmp(m, magic, 4) != 0) {
        std::fprintf(stderr, "map_view: %s: bad magic\n", path.c_str());
        return false;
    }
    uint32_t v = r.read_u32();
    uint32_t n = r.read_u32();
    if (v != 1) {
        std::fprintf(stderr, "map_view: %s: unsupported version %u\n", path.c_str(), v);
        return false;
    }
    out.xyz.reserve(n * 9);
    out.ll.reserve(n * 6);
    for (uint32_t i = 0; i < n; ++i) {
        float la[3], lo[3];
        for (int k = 0; k < 3; ++k) {
            la[k] = r.read_f32();
            lo[k] = r.read_f32();
        }
        for (int k = 0; k < 3; ++k) {
            V3 u = latlon_to_unit(la[k], lo[k]);
            out.xyz.push_back((float)u.x);
            out.xyz.push_back((float)u.y);
            out.xyz.push_back((float)u.z);
            out.ll.push_back(la[k]);
            out.ll.push_back(lo[k]);
        }
    }
    return r.ok();
}

static bool load_line_file(const std::string &path, LineBuf &out, const char *magic) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f.is_open()) { std::fprintf(stderr, "map_view: cannot open %s\n", path.c_str()); return false; }
    auto sz = f.tellg(); f.seekg(0, std::ios::beg);
    std::vector<uint8_t> buf((size_t)sz);
    f.read((char *)buf.data(), sz);
    BinaryReader r(buf);
    char m[4]; r.read_bytes(m, 4);
    if (std::memcmp(m, magic, 4) != 0) { std::fprintf(stderr, "map_view: %s: bad magic\n", path.c_str()); return false; }
    uint32_t v = r.read_u32(); uint32_t n = r.read_u32();
    if (v != 1) { std::fprintf(stderr, "map_view: %s: unsupported version %u\n", path.c_str(), v); return false; }
    out.starts.reserve(n);
    out.counts.reserve(n);
    for (uint32_t i = 0; i < n; ++i) {
        uint32_t cnt = r.read_u32();
        (void)r.read_u32();  // flags
        GLint start = (GLint)(out.xyz.size() / 3);
        for (uint32_t k = 0; k < cnt; ++k) {
            float la = r.read_f32();
            float lo = r.read_f32();
            V3 u = latlon_to_unit(la, lo);
            out.xyz.push_back((float)u.x);
            out.xyz.push_back((float)u.y);
            out.xyz.push_back((float)u.z);
            out.ll.push_back(la);
            out.ll.push_back(lo);
        }
        out.starts.push_back(start);
        out.counts.push_back((GLsizei)cnt);
    }
    return r.ok();
}

// --- Trajectory samples ----------------------------------------------

struct Sample {
    double t_sec;
    double lat_deg, lon_deg, alt_m;  // ECEF geographic
    bool   is_sib_spent;
};

// Position on a sphere of radius (1 + alt_m/R_earth) in unit-sphere
// coords, using ECEF lat/lon so the result lives in the same frame as
// the GSHHG land triangles.
static V3 geo_to_unit_space(double lat_deg, double lon_deg, double alt_m) {
    V3 u = latlon_to_unit(lat_deg, lon_deg);
    double r = 1.0 + alt_m / kEarthR_m;
    return { u.x * r, u.y * r, u.z * r };
}

// --- GL state save/restore -------------------------------------------

struct SavedGLState {
    GLint   viewport[4];
    GLint   matrix_mode;
    GLint   blend_src, blend_dst;
    GLboolean depth_test, blend, lighting, texture_2d, cull_face;
};

static void save_gl(SavedGLState &s) {
    glGetIntegerv  (GL_VIEWPORT,    s.viewport);
    glGetIntegerv  (GL_MATRIX_MODE, &s.matrix_mode);
    glGetIntegerv  (GL_BLEND_SRC,   &s.blend_src);
    glGetIntegerv  (GL_BLEND_DST,   &s.blend_dst);
    glGetBooleanv  (GL_DEPTH_TEST,  &s.depth_test);
    glGetBooleanv  (GL_BLEND,       &s.blend);
    glGetBooleanv  (GL_LIGHTING,    &s.lighting);
    glGetBooleanv  (GL_TEXTURE_2D,  &s.texture_2d);
    glGetBooleanv  (GL_CULL_FACE,   &s.cull_face);
    glMatrixMode(GL_PROJECTION); glPushMatrix();
    glMatrixMode(GL_MODELVIEW);  glPushMatrix();
}
static void restore_gl(const SavedGLState &s) {
    glMatrixMode(GL_PROJECTION); glPopMatrix();
    glMatrixMode(GL_MODELVIEW);  glPopMatrix();
    glMatrixMode((GLenum)s.matrix_mode);
    glViewport(s.viewport[0], s.viewport[1], s.viewport[2], s.viewport[3]);
    glBlendFunc((GLenum)s.blend_src, (GLenum)s.blend_dst);
    if (s.depth_test) glEnable(GL_DEPTH_TEST); else glDisable(GL_DEPTH_TEST);
    if (s.blend)      glEnable(GL_BLEND);      else glDisable(GL_BLEND);
    if (s.lighting)   glEnable(GL_LIGHTING);   else glDisable(GL_LIGHTING);
    if (s.texture_2d) glEnable(GL_TEXTURE_2D); else glDisable(GL_TEXTURE_2D);
    if (s.cull_face)  glEnable(GL_CULL_FACE);  else glDisable(GL_CULL_FACE);
}

// --- Terminator shading ---------------------------------------------

// Returns the per-vertex "lit factor" in [0,1] for a surface point
// at unit `u`, given sun direction `sun`.  1.0 = fully lit, 0.0 =
// deep night, with a soft linear ramp spanning ±kTerminatorSoftnessDeg
// around the terminator great circle.
static float lit_factor(const V3 &u, const V3 &sun) {
    double c = dot(u, sun);                 // cos(angle from sub-solar point)
    double angle_deg = std::acos(std::max(-1.0, std::min(1.0, c))) * kRadToDeg;
    // 90° = terminator.  >90° is night side.
    double x = (90.0 - angle_deg) / kTerminatorSoftnessDeg;  // +ve = day, -ve = night
    double lit = 0.5 + 0.5 * std::tanh(x);      // smooth 0→1 ramp
    return (float)lit;
}

// --- Earth rendering ------------------------------------------------

static void draw_triangles_shaded(const TriBuf &tri, const V3 &sun,
                                  float base_r, float base_g, float base_b)
{
    if (tri.count() == 0) return;
    glBegin(GL_TRIANGLES);
    const float *xyz = tri.xyz.data();
    for (size_t t = 0; t < tri.count(); ++t) {
        for (int k = 0; k < 3; ++k) {
            const float *v = xyz + (t * 9) + k * 3;
            V3 u{v[0], v[1], v[2]};
            float lit = lit_factor(u, sun);
            float f = kNightFactor + (kDayFactor - kNightFactor) * lit;
            glColor3f(base_r * f, base_g * f, base_b * f);
            glVertex3fv(v);
        }
    }
    glEnd();
}

static void draw_coastlines_shaded(const LineBuf &lb, const V3 &sun) {
    if (lb.starts.empty()) return;
    glLineWidth(1.0f);
    for (size_t p = 0; p < lb.starts.size(); ++p) {
        GLint  s  = lb.starts[p];
        GLsizei c = lb.counts[p];
        glBegin(GL_LINE_LOOP);
        for (GLsizei k = 0; k < c; ++k) {
            const float *v = lb.xyz.data() + (s + k) * 3;
            V3 u{v[0], v[1], v[2]};
            float lit = lit_factor(u, sun);
            float f = kNightFactor + (kDayFactor - kNightFactor) * lit;
            glColor3f(kCoastR * f, kCoastG * f, kCoastB * f);
            glVertex3fv(v);
        }
        glEnd();
    }
}

// --- Atmospheric halo -----------------------------------------------
//
// A sphere slightly larger than the Earth (radius 1.025 in unit-sphere
// coords ≈ 160 km above the surface) tessellated and shaded with
// per-vertex alpha = pow(1 − |dot(normal, view)|, p).  At any given
// vertex, "grazing angle" is 0 when the surface faces the camera
// head-on and 1 at the silhouette — so the fragment alpha peaks on
// the halo's limb and fades to zero inside the disk.  Rendered with
// depth test ON (so the far side gets depth-occluded by the real
// Earth) and additive blend (so the near-side rim adds brightness
// against whatever is behind — space or terminator).  Draw after all
// Earth geometry.
static void draw_atmospheric_halo(const V3 &cam_pos, const V3 &sun)
{
    constexpr int    STACKS  = 48;
    constexpr int    SLICES  = 96;
    constexpr double HALO_R  = 1.010;     // 1 % larger than Earth (~65 km)
    constexpr double P       = 24.0;      // sharpness of the limb peak
    constexpr float  R_COL   = 0.55f;
    constexpr float  G_COL   = 0.62f;
    constexpr float  B_COL   = 0.78f;
    constexpr float  SCALE   = 3.0f;      // caps alpha at 1.0 near the limb
    // Terminator-centred modulation: alpha peaks where dot(n, sun) ≈ 0
    // (i.e. right along the day/night boundary of the atmosphere) and
    // fades as a Gaussian in dot(n, sun) toward both the sub-solar
    // and anti-solar directions.  Bigger TERM_SHARPNESS = tighter
    // peak around the terminator.
    constexpr double TERM_SHARPNESS = 6.0;

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE);    // additive
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);                // don't clobber Earth depth
    glDisable(GL_CULL_FACE);

    auto terminator_factor = [&](const V3 &n) -> double {
        double c = dot(n, sun);           // −1 (anti-solar) … +1 (sub-solar)
        return std::exp(-TERM_SHARPNESS * c * c);
    };

    for (int i = 0; i < STACKS; ++i) {
        double th1 = (double)i       / STACKS * M_PI - M_PI_2;
        double th2 = (double)(i + 1) / STACKS * M_PI - M_PI_2;
        double s1 = std::sin(th1), c1 = std::cos(th1);
        double s2 = std::sin(th2), c2 = std::cos(th2);
        glBegin(GL_QUAD_STRIP);
        for (int j = 0; j <= SLICES; ++j) {
            double ph = (double)j / SLICES * 2.0 * M_PI;
            double cp = std::cos(ph), sp = std::sin(ph);

            V3 n1 = {c1 * cp, c1 * sp, s1};
            V3 p1 = n1 * HALO_R;
            V3 vdir1 = normalize(cam_pos - p1);
            double g1 = 1.0 - std::fabs(dot(n1, vdir1));
            double a1 = std::pow(g1, P) * SCALE * terminator_factor(n1);
            if (a1 > 1.0) a1 = 1.0;
            glColor4f(R_COL, G_COL, B_COL, (float)a1);
            glVertex3f((float)p1.x, (float)p1.y, (float)p1.z);

            V3 n2 = {c2 * cp, c2 * sp, s2};
            V3 p2 = n2 * HALO_R;
            V3 vdir2 = normalize(cam_pos - p2);
            double g2 = 1.0 - std::fabs(dot(n2, vdir2));
            double a2 = std::pow(g2, P) * SCALE * terminator_factor(n2);
            if (a2 > 1.0) a2 = 1.0;
            glColor4f(R_COL, G_COL, B_COL, (float)a2);
            glVertex3f((float)p2.x, (float)p2.y, (float)p2.z);
        }
        glEnd();
    }

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
}

// --- 2D stroke text helper (uses GLUT_STROKE_MONO_ROMAN) ------------

static void draw_stroke_text(float x, float y, float cell_h, const char *s,
                             float r, float g, float b)
{
    constexpr float kStrokeAdvance = 104.76f;
    constexpr float kStrokeHeight  = 152.38f;
    const float sx = cell_h * 0.6f / kStrokeAdvance;  // monospace width ≈ 60% of cell_h
    const float sy = cell_h / kStrokeHeight;
    glLineWidth(std::max(1.0f, cell_h * 0.08f));
    glColor3f(r, g, b);
    float cursor = x;
    for (const char *p = s; *p; ++p) {
        glPushMatrix();
        glTranslatef(cursor, y, 0);
        glScalef(sx, sy, 1);
        glutStrokeCharacter(GLUT_STROKE_MONO_ROMAN, *p);
        glPopMatrix();
        cursor += cell_h * 0.6f;
    }
    glLineWidth(1.0f);
}

} // namespace


// =====================================================================
// MapView::Impl
// =====================================================================

struct MapView::Impl {
    bool loaded = false;

    TriBuf  land;
    TriBuf  lakes;
    LineBuf coast;

    std::vector<Sample> samples;
};

MapView::MapView() : m_impl(std::make_unique<Impl>()) {}
MapView::~MapView() = default;

bool MapView::init(const std::string &data_dir) {
    std::string base = data_dir + "/earth/";
    bool ok = true;
    ok &= load_tri_file (base + "gshhs_i_land.bin",      m_impl->land,  "ETRI");
    ok &= load_tri_file (base + "gshhs_i_lakes.bin",     m_impl->lakes, "ETRI");
    ok &= load_line_file(base + "gshhs_i_coastline.bin", m_impl->coast, "ELIN");
    m_impl->loaded = ok;
    if (ok) {
        std::fprintf(stderr,
            "map_view: loaded land=%zu tri, lakes=%zu tri, coast=%zu lines\n",
            m_impl->land.count(), m_impl->lakes.count(),
            m_impl->coast.starts.size());
    }
    return ok;
}

void MapView::push_sample(double t, double lat_deg, double lon_deg, double alt_m, bool spent) {
    m_impl->samples.push_back({t, lat_deg, lon_deg, alt_m, spent});
}

void MapView::render(float x_scene, float y_scene, float w_scene, float h_scene,
                     int scene_w, int scene_h,
                     double sim_elapsed_sec,
                     double vehicle_lat_deg,
                     double vehicle_lon_deg,
                     double vehicle_alt_m,
                     double display_altitude_m,
                     double display_velocity_mps,
                     int year, int month, int day,
                     int hour, int minute, double sec)
{
    if (!m_impl->loaded) return;

    SavedGLState saved; save_gl(saved);

    // Convert the scene-ortho rect to a pixel sub-viewport.
    const int px_x = (int)((x_scene / (float)scene_w) * saved.viewport[2]);
    const int px_y = (int)((y_scene / (float)scene_h) * saved.viewport[3]);
    const int px_w = (int)((w_scene / (float)scene_w) * saved.viewport[2]);
    const int px_h = (int)((h_scene / (float)scene_h) * saved.viewport[3]);
    const int vp_x = saved.viewport[0] + px_x;
    const int vp_y = saved.viewport[1] + px_y;
    glViewport(vp_x, vp_y, px_w, px_h);

    // Defensive depth clear restricted to the sub-viewport so we don't
    // depend on whatever depth state the enclosing scene left behind
    // (the panel renderer runs with depth test off, so values are
    // whatever glClear last set, typically 1.0 — but a future scene
    // that writes depth could otherwise wreck our depth test).
    glEnable(GL_SCISSOR_TEST);
    glScissor(vp_x, vp_y, px_w, px_h);
    glClear(GL_DEPTH_BUFFER_BIT);
    glDisable(GL_SCISSOR_TEST);

    // Clear the sub-rect to background (via a filled quad, since we
    // don't own the whole frame and glClear ignores viewport).
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glDisable(GL_TEXTURE_2D);
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); glOrtho(-1, 1, -1, 1, -1, 1);
    glMatrixMode(GL_MODELVIEW);  glLoadIdentity();
    glColor3f(kBgR, kBgG, kBgB);
    glBegin(GL_QUADS);
      glVertex2f(-1, -1); glVertex2f(1, -1); glVertex2f(1, 1); glVertex2f(-1, 1);
    glEnd();

    // Camera setup.  All positions in the ECEF/geographic frame — same
    // frame as the GSHHG land data — so land rendered at its catalog
    // lat/lon ends up where it should regardless of GMST.
    //
    // Camera altitude and south-offset are solved per-frame so the
    // Earth's north limb always projects β = (FOV/2 − margin) above
    // the vehicle on screen.  This keeps the framing consistent as
    // the vehicle climbs: the horizon stays visible at a fixed spot
    // in the upper portion of the view.
    V3 veh_u = geo_to_unit_space(vehicle_lat_deg, vehicle_lon_deg, vehicle_alt_m);
    double h_c_m = 0.0, eta_deg = 0.0;
    double beta_deg = 0.5 * kCamFOVdeg - kCamLimbMarginDeg;
    double slant_km = slant_dist_for_velocity_mps(display_velocity_mps);
    solve_camera_altitude(vehicle_alt_m,
                          slant_km * 1000.0,
                          beta_deg,
                          h_c_m, eta_deg);
    double cam_lat = vehicle_lat_deg - eta_deg;
    double cam_r   = 1.0 + h_c_m / kEarthR_m;
    V3 cam_u = latlon_to_unit(cam_lat, vehicle_lon_deg);
    V3 cam_pos = cam_u * cam_r;

    // Look at the vehicle.  Camera up = world +Z (rotation axis).
    V3 look_at = veh_u;
    V3 up      = {0, 0, 1};
    // If vehicle is nearly directly overhead of the camera (cam_lat → 90°),
    // use ECI +X as fallback up to avoid gimbal lock.
    V3 fwd = normalize(look_at - cam_pos);
    if (std::abs(dot(fwd, up)) > 0.99) up = {1, 0, 0};

    // Projection: perspective with narrow FOV.
    glMatrixMode(GL_PROJECTION); glLoadIdentity();
    double aspect = (double)px_w / (double)std::max(1, px_h);
    gluPerspective(kCamFOVdeg, aspect, 0.01, 100.0);
    glMatrixMode(GL_MODELVIEW);  glLoadIdentity();
    gluLookAt(cam_pos.x, cam_pos.y, cam_pos.z,
              look_at.x, look_at.y, look_at.z,
              up.x,      up.y,      up.z);

    // Sun direction in ECEF (so land shading tracks the real terminator).
    V3 sun = sun_eci_unit(year, month, day, hour, minute, sec);

    glEnable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);   // we don't have a reliable winding guarantee

    // 1. Dark sphere backdrop so the trajectory depth-clips behind Earth.
    glColor3f(kBgR, kBgG, kBgB);
    GLUquadric *q = gluNewQuadric();
    gluSphere(q, 0.999, 64, 48);
    gluDeleteQuadric(q);

    // 2. Land triangles
    draw_triangles_shaded(m_impl->land,  sun, kLandR, kLandG, kLandB);
    // 3. Lakes over land (space-colored holes)
    draw_triangles_shaded(m_impl->lakes, sun, kBgR,   kBgG,   kBgB);
    // 4. Coastline accent
    draw_coastlines_shaded(m_impl->coast, sun);

    // 5. Trajectory polylines (vehicle = bright thick; S-IB spent = dim thinner)
    if (!m_impl->samples.empty()) {
        glLineWidth(3.0f);
        glColor3f(0.90f, 0.90f, 0.90f);
        glBegin(GL_LINE_STRIP);
        for (const auto &s : m_impl->samples) {
            if (s.is_sib_spent) continue;
            V3 u = geo_to_unit_space(s.lat_deg, s.lon_deg, s.alt_m);
            glVertex3f((float)u.x, (float)u.y, (float)u.z);
        }
        glEnd();

        glLineWidth(1.5f);
        glColor3f(0.72f, 0.72f, 0.72f);
        glBegin(GL_LINE_STRIP);
        for (const auto &s : m_impl->samples) {
            if (!s.is_sib_spent) continue;
            V3 u = geo_to_unit_space(s.lat_deg, s.lon_deg, s.alt_m);
            glVertex3f((float)u.x, (float)u.y, (float)u.z);
        }
        glEnd();
    }

    // 6. Vehicle marker (double dot: outer ring + inner dot)
    V3 veh_render = veh_u;  // already in render space
    // Outer ring
    glPointSize(10.0f);
    glEnable(GL_POINT_SMOOTH);
    glColor3f(1.0f, 1.0f, 1.0f);
    glBegin(GL_POINTS);
    glVertex3f((float)veh_render.x, (float)veh_render.y, (float)veh_render.z);
    glEnd();
    glPointSize(4.0f);
    glColor3f(0.1f, 0.1f, 0.1f);
    glBegin(GL_POINTS);
    glVertex3f((float)veh_render.x, (float)veh_render.y, (float)veh_render.z);
    glEnd();
    glDisable(GL_POINT_SMOOTH);

    // 7. Atmospheric halo (3D sphere just outside Earth, per-vertex
    // alpha brightest at the silhouette, modulated by local day-
    // factor so the limb glow fades through the terminator and
    // disappears on the night side).  Drawn in world coords so it
    // automatically aligns with the Earth's actual on-screen
    // silhouette regardless of where the camera is looking.
    draw_atmospheric_halo(cam_pos, sun);

    // 8. 2D overlays (MET, altitude, velocity, timeline).  Use the
    //    same ortho that draw_rim_glow restored on exit of its push.
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_BLEND);
    glMatrixMode(GL_PROJECTION); glLoadIdentity();
    glOrtho(0, px_w, 0, px_h, -1, 1);
    glMatrixMode(GL_MODELVIEW);  glLoadIdentity();

    // Altitude / velocity readouts, upper-left.  Caller passes these
    // pre-computed from the right plant output channel for the current
    // flight phase; display "0" when below liftoff (altitude ≤ 0).
    // Text sizes are 2x the earlier draft; starting y is shifted down
    // to match (so top of text is at the same place), then dropped a
    // further 15% of frame height to clear the Earth's limb.
    double alt_nmi = display_altitude_m   / kM_PER_NMI;
    double vel_fps = display_velocity_mps / kM_PER_FT;
    bool   pre_liftoff = display_altitude_m <= 0.0;

    const float tx       = 10.0f;
    const float label_h  = 22.0f;  // was 11
    const float ch       = 32.0f;  // was 16  (value text height)
    float       ty       = (float)px_h - 72.0f - 0.08f * (float)px_h;
    char buf[64];
    draw_stroke_text(tx, ty, label_h, "ALTITUDE", 0.55f, 0.55f, 0.55f);
    ty -= ch * 1.2f;
    if (pre_liftoff) std::snprintf(buf, sizeof buf, "%5s NM", "0");
    else             std::snprintf(buf, sizeof buf, "%5.1f NM", alt_nmi);
    draw_stroke_text(tx, ty, ch, buf, 0.95f, 0.95f, 0.95f);
    ty -= ch * 1.6f;
    draw_stroke_text(tx, ty, label_h, "VELOCITY", 0.55f, 0.55f, 0.55f);
    ty -= ch * 1.2f;
    if (pre_liftoff) std::snprintf(buf, sizeof buf, "%5s FPS", "0");
    else             std::snprintf(buf, sizeof buf, "%5.0f FPS", vel_fps);
    draw_stroke_text(tx, ty, ch, buf, 0.95f, 0.95f, 0.95f);

    // MET / countdown at top-center.  Liftoff is at sim-elapsed 300 s
    // by current SaturnStartupHelper convention (T_L_prime_sec=300).
    constexpr double kLiftoffOffset_sec = 300.0;
    double met = sim_elapsed_sec - kLiftoffOffset_sec;
    const char *sign = (met < 0) ? "T-" : "T+";
    long tot = (long)std::floor(std::abs(met));
    std::snprintf(buf, sizeof buf, "%s %02ld:%02ld:%02ld",
                  sign, tot/3600, (tot/60)%60, tot%60);
    const float met_h  = 36.0f;                 // was 18
    const int   met_n  = (int)std::strlen(buf);
    const float met_w  = met_n * met_h * 0.6f;  // stroke mono advance = 0.6 * cell_h
    draw_stroke_text(px_w / 2.0f - met_w * 0.5f,
                     (float)px_h - 38.0f,       // 50% of met_h higher than before
                     met_h, buf,
                     0.95f, 0.95f, 0.95f);

    // Timeline at bottom.  Thin grey baseline; a thick yellow
    // progress segment overlays the baseline from the start to the
    // current MET.  Event ticks sit on the baseline; labels alternate
    // above/below (per kAS205Events[].label_side) to prevent collision
    // among the close-packed MECO / STAGING / IGM START events.
    const float tl_y    = 36.0f;
    const float tl_x0   = 10.0f;
    const float tl_x1   = (float)px_w - 10.0f;
    const float tl_tmin = -30.0f;
    const float tl_tmax = 700.0f;
    const float tl_label_h = 18.0f;

    // Baseline
    glColor3f(0.35f, 0.35f, 0.35f);
    glLineWidth(1.0f);
    glBegin(GL_LINES);
      glVertex2f(tl_x0, tl_y); glVertex2f(tl_x1, tl_y);
    glEnd();

    // Progress overlay: horizontal thick mid-gray line over the
    // baseline up to the current MET.  Nothing drawn before liftoff.
    {
        double u = (met - tl_tmin) / (tl_tmax - tl_tmin);
        if (u > 0.0) {
            if (u > 1.0) u = 1.0;
            float ex = tl_x0 + (float)u * (tl_x1 - tl_x0);
            glColor3f(0.78f, 0.78f, 0.78f);
            glLineWidth(6.0f);
            glBegin(GL_LINES);
              glVertex2f(tl_x0, tl_y); glVertex2f(ex, tl_y);
            glEnd();
            glLineWidth(1.0f);
        }
    }

    // Event ticks + optional labels.
    for (const auto &e : kAS205Events) {
        float u = (float)((e.t_sec - tl_tmin) / (tl_tmax - tl_tmin));
        if (u < 0 || u > 1) continue;
        float ex = tl_x0 + u * (tl_x1 - tl_x0);
        bool passed = (met >= e.t_sec);
        float g = passed ? 0.95f : 0.55f;
        glColor3f(g, g, g);
        glLineWidth(1.0f);
        glBegin(GL_LINES);
          glVertex2f(ex, tl_y - 8.0f); glVertex2f(ex, tl_y + 8.0f);
        glEnd();
        if (e.label_side >= 0 && e.label && e.label[0] != '\0') {
            // Multi-line (contains '\n'): left-justify each line at the
            // tick, stack upward from a baseline just above the tick.
            // Single-line: centre under/over the tick as before.
            const char *nl = std::strchr(e.label, '\n');
            if (nl) {
                char line1[32], line2[32];
                size_t n1 = (size_t)(nl - e.label);
                if (n1 >= sizeof line1) n1 = sizeof line1 - 1;
                std::memcpy(line1, e.label, n1); line1[n1] = '\0';
                std::snprintf(line2, sizeof line2, "%s", nl + 1);
                const float line_gap = 3.0f;
                const float bot_y    = tl_y + 10.0f;
                const float top_y    = bot_y + tl_label_h + line_gap;
                draw_stroke_text(ex, top_y, tl_label_h, line1, g, g, g);
                draw_stroke_text(ex, bot_y, tl_label_h, line2, g, g, g);
            } else {
                float lab_w = (float)std::strlen(e.label) * tl_label_h * 0.6f;
                float lab_y = (e.label_side == 1)
                              ? (tl_y + 14.0f)
                              : (tl_y - (tl_label_h + 6.0f));
                draw_stroke_text(ex - lab_w * 0.5f, lab_y,
                                 tl_label_h, e.label, g, g, g);
            }
        }
    }

    restore_gl(saved);
}

} // namespace dskymatic
