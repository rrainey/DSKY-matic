// Very small pick-test helper: find the topmost pickable polygon at
// a world-space point.  Sufficient for the DSKY panel, whose buttons
// are all axis-aligned quads.

#pragma once

#include "panel_scene.h"
#include <string>

namespace panel {

// Convert GLUT mouse coords (mx from left, my from top) to world coords
// in the scene's ortho space (Y-up, origin bottom-left).
void screen_to_world(const Scene &scene, int win_w, int win_h,
                     int mx, int my, float *wx, float *wy);

// Walk the scene tree looking for the DEEPEST pickable polygon whose
// axis-aligned bounding box (in world coords) contains (wx, wy).
// Returns nullptr if nothing hits.
SceneNode *pick_at(Scene &scene, float wx, float wy);

}  // namespace panel
