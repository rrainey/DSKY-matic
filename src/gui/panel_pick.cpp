#include "panel_pick.h"

#include <algorithm>
#include <limits>

namespace panel {

void screen_to_world(const Scene &scene, int win_w, int win_h,
                     int mx, int my, float *wx, float *wy) {
    // Ortho maps (0..scene.width, 0..scene.height) uniformly across the
    // window.  Y is flipped because GLUT uses top-left, GL uses bottom-left.
    if (win_w <= 0) win_w = 1;
    if (win_h <= 0) win_h = 1;
    *wx = static_cast<float>(mx) * scene.width  / static_cast<float>(win_w);
    *wy = static_cast<float>(win_h - my) * scene.height / static_cast<float>(win_h);
}

namespace {

// Recursively find the deepest visible+pickable polygon whose local
// bounding box (translated by Location) contains (wx, wy).  We track
// a "best" hit by draw order (later-drawn wins, matches painter order).
struct HitState {
    SceneNode *best = nullptr;
    int order      = -1;    // higher = later drawn = on top
    int cur_order  = 0;
};

void visit(SceneNode *n, float wx, float wy, HitState &hit) {
    if (!n || !n->visible) return;

    if (n->kind == NodeKind::Polygon && n->pickable && !n->vertices.empty()) {
        // Axis-aligned bbox in local coords.
        float lx0 = std::numeric_limits<float>::infinity();
        float ly0 = lx0;
        float lx1 = -lx0, ly1 = -lx0;
        for (const auto &v : n->vertices) {
            lx0 = std::min(lx0, v.x); lx1 = std::max(lx1, v.x);
            ly0 = std::min(ly0, v.y); ly1 = std::max(ly1, v.y);
        }
        // Translate by Location.
        float x0 = n->location.x + lx0, x1 = n->location.x + lx1;
        float y0 = n->location.y + ly0, y1 = n->location.y + ly1;
        if (wx >= x0 && wx <= x1 && wy >= y0 && wy <= y1) {
            hit.best  = n;
            hit.order = hit.cur_order;
        }
    }

    ++hit.cur_order;

    for (auto &c : n->children) visit(c.get(), wx, wy, hit);
    if (n->component_scene && n->component_scene->root) {
        // Note: this ignores the Component's own Location + EditorMatrix
        // for pick purposes.  All DSKY buttons live at the top level of
        // ApolloA, not inside a Component, so it's fine for now.
        visit(n->component_scene->root.get(), wx, wy, hit);
    }
}

}  // namespace

SceneNode *pick_at(Scene &scene, float wx, float wy) {
    HitState hit;
    if (scene.root) visit(scene.root.get(), wx, wy, hit);
    return hit.best;
}

}  // namespace panel
