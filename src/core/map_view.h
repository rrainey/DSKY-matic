#pragma once

/*
 * Stylized launch-visualization renderer for dskymatic-studio.
 *
 * Draws a monochrome 3D Earth with coastline art (from GSHHG level-1/2
 * binaries preprocessed by Apollo-Saturn-IB/scripts/earth/gen_earth_data.py),
 * a day/night terminator computed from the sim date, an atmospheric
 * rim glow, the launch vehicle's trajectory history as a 3D polyline
 * (with a thinner trace for the spent S-IB stage), and 2D overlays
 * for MET, altitude, velocity and mission-event timeline.
 *
 * The renderer draws into a sub-viewport of the current window — the
 * caller provides a rect in scene-ortho coords and the pixel viewport
 * region is derived from it.  All GL state the renderer touches
 * (viewport, matrices, depth test, blend) is saved and restored
 * around the call so the surrounding 2D panel rendering is
 * unaffected.
 *
 * Camera: perspective, placed on the vehicle's meridional plane
 * (Earth-center-to-south-pole plane through the vehicle), offset
 * south by a tunable angular amount and slightly above the surface.
 * Camera-up is Earth's rotation axis (so "north is up" on screen).
 */

#include <cstdint>
#include <memory>
#include <string>

namespace dskymatic {

class MapView {
public:
    MapView();
    ~MapView();

    MapView(const MapView &) = delete;
    MapView &operator=(const MapView &) = delete;

    /** Load the three binary datasets.  data_dir is the parent of
     *  'earth/gshhs_i_land.bin' etc.  Returns true on success; prints
     *  per-file errors on failure. */
    bool init(const std::string &data_dir);

    /** Push one trajectory sample.  sim_elapsed_sec is monotonic.
     *  Position is in Earth-fixed (ECEF) geographic coordinates — same
     *  frame as the GSHHG land / coastline data, so samples project to
     *  the right spot on the rendered Earth regardless of how much
     *  the planet has rotated since launch epoch.  is_sib_spent marks
     *  samples belonging to the spent S-IB stage after separation
     *  (drawn as a thinner secondary trace). */
    void push_sample(double sim_elapsed_sec,
                     double lat_deg, double lon_deg, double alt_m,
                     bool is_sib_spent);

    /** Render the map view into a rectangle given in scene-ortho
     *  coords.  (x_scene, y_scene) is the lower-left corner,
     *  (w_scene, h_scene) is the extent.  scene_w_px / scene_h_px
     *  are the scene canvas dimensions (so the renderer can convert
     *  to pixels for glViewport).
     *
     *  vehicle ECI pose feeds the camera placement and the
     *  altitude/velocity readouts.  (year, month, day, hour, min, sec)
     *  is the current UTC date for sub-solar-point computation. */
    void render(float x_scene, float y_scene, float w_scene, float h_scene,
                int scene_w, int scene_h,
                double sim_elapsed_sec,
                double vehicle_lat_deg,
                double vehicle_lon_deg,
                double vehicle_alt_m,
                double display_altitude_m,
                double display_velocity_mps,
                int year, int month, int day,
                int hour, int minute, double second);

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace dskymatic
