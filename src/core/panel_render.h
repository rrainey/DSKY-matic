// Fixed-function GL renderer for the Scene tree.  Matches the original
// output: ortho projection sized to the DisplayFrame, textured quads
// with per-vertex color modulation, alpha-blended lamp overlays.

#pragma once

#include "panel_scene.h"

namespace panel {

// Set up ortho projection matching scene->width x scene->height.
// Call once per frame before render_scene.
void begin_frame(const Scene &scene, int viewport_w, int viewport_h);

// Recursively render the scene starting from the root.  Assumes
// begin_frame has been called.  Textures with gl_id == 0 fall back to
// vertex/flat color.
void render_scene(const Scene &scene);

// Debug: if set to true, only Sphere-kind nodes are drawn (all other
// leaves are skipped, but Components and Groups still descend).  Useful
// for isolating FDAI ball orientation without overlay clutter.
extern bool g_render_spheres_only;

// Alternative: render just one node subtree (useful for probes).
void render_node(const Scene &scene, const SceneNode &node);

// Multiplier applied to TextBox cell_w / cell_h at render time.  1.0
// leaves the native scene dimensions.  Values > 1.0 enlarge the text
// in world units (which the viewport then maps to pixels as usual).
void set_font_scale(float s);
float get_font_scale();

// Force the TextBox renderer onto GLUT stroke fonts regardless of
// font_scale.  Needed when the window is stretched above native size
// (--max-window): bitmap glyphs are a fixed pixel count and would
// look tiny in the enlarged window, while stroke glyphs render in
// world coords and scale with the viewport.  When this is false (the
// default), bitmap rendering is used at scale=1.0 and stroke at >1.0.
void set_use_stroke_fonts(bool on);

}  // namespace panel
