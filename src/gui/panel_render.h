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

}  // namespace panel
