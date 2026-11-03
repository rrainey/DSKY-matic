#include "panel_render.h"

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <chrono>
#include <cmath>
#include <cstdio>

namespace panel {

bool g_render_spheres_only = false;

// Global blink phase, refreshed by begin_frame().
static bool g_blink_off = false;

static void update_blink_phase() {
    // 2 Hz square wave; half-period = 250 ms.
    using clock = std::chrono::steady_clock;
    static auto t0 = clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  clock::now() - t0).count();
    g_blink_off = ((ms / 250) & 1) != 0;
}

void begin_frame(const Scene &scene, int viewport_w, int viewport_h) {
    update_blink_phase();
    glViewport(0, 0, viewport_w, viewport_h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    // the original panel uses OpenGL's native Y-up convention -- e.g. ELPROG at
    // Y=370 draws above ELBar3 at Y=208.  Paired with V-flipped texture
    // load so image row 0 (top of file) still ends up at the top of
    // the geometry (small scene V value).
    //
    // Use glOrtho instead of gluOrtho2D so the Z range is wide enough
    // to keep 3D geometry visible: the FDAI attitude ball sits at
    // Z=-260 in the component's local frame, well beyond gluOrtho2D's
    // default Z=[-1,+1] clip planes.
    glOrtho(0, scene.width, 0, scene.height, -10000, 10000);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_DEPTH_TEST);
}

namespace {

void apply_transform(const SceneNode &n) {
    glPushMatrix();
    // Location + DynamicTranslate compose additively (matches original:
    // DynamicTranslate is an "offset from Location" applied every
    // frame, e.g. for FDAI rate arrows and error wipers).
    glTranslatef(n.location.x + n.dynamic_translate.x,
                 n.location.y + n.dynamic_translate.y,
                 n.location.z + n.dynamic_translate.z);
    // Rotation with RotationOrigin: rotation happens about
    // (Location + RotationOrigin) in world coords.  The polygon's
    // vertices are already relative to Location, so to pivot around
    // the offset origin we translate to it, rotate, translate back.
    // Used by e.g. FDAI's RollIndexerArrow which has
    // RotationOrigin = (-0.84, -108.30, 0) so it sweeps around the
    // ball's center rather than spinning on its own axis.
    if (n.kind != NodeKind::Sphere) {
        glTranslatef(n.rotation_origin.x, n.rotation_origin.y, n.rotation_origin.z);
        glRotatef(n.rotation.z, 0, 0, 1);
        glRotatef(n.rotation.y, 0, 1, 0);
        glRotatef(n.rotation.x, 1, 0, 0);
        glTranslatef(-n.rotation_origin.x, -n.rotation_origin.y, -n.rotation_origin.z);
    }
}

void bind_texture_or_disable(const Scene &scene, int idx) {
    if (idx >= 0 && idx < (int)scene.textures.entries.size()
        && scene.textures.entries[idx].gl_id != 0) {
        glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, scene.textures.entries[idx].gl_id);
        glColor4f(1, 1, 1, 1);  // texture modulated by white = untouched
    } else {
        glDisable(GL_TEXTURE_2D);
    }
}

void draw_textbox_content(const SceneNode &n) {
    if (n.text.empty()) return;
    // Draw the string as GLUT bitmap glyphs, positioned in world coords
    // using the scene CellWidth/CellHeight for advance and line spacing.
    // The polygon's vertex box gives the drawable region: origin at
    // (0,0), extent (max_x, max_y) after Location translate.
    float box_w = 0, box_h = 0;
    for (const auto &v : n.vertices) {
        if (v.x > box_w) box_w = v.x;
        if (v.y > box_h) box_h = v.y;
    }
    // Top-of-box is at world Y = box_h (since GL is Y-up and the box
    // is defined with (0,0)..(w,h)).  Text starts near the top.
    glDisable(GL_TEXTURE_2D);
    glColor4ub(n.text_color.r, n.text_color.g, n.text_color.b, n.text_color.a);
    // GLUT bitmap fonts are fixed-size; pick 8x13 when CellHeight is
    // below the historical 15 px default (e.g. StatusWindow at 80%).
    void *font = (n.cell_h < 15) ? GLUT_BITMAP_8_BY_13 : GLUT_BITMAP_9_BY_15;
    float x = 2.0f;
    float y = box_h - static_cast<float>(n.cell_h);
    glRasterPos2f(x, y);
    for (char c : n.text) {
        if (c == '\n') {
            y -= static_cast<float>(n.cell_h);
            x = 2.0f;
            glRasterPos2f(x, y);
        } else if (c == '\r') {
            // ignore
        } else if (c == '\t') {
            x += 4 * n.cell_w;
            glRasterPos2f(x, y);
        } else {
            glutBitmapCharacter(font, c);
            x += n.cell_w;
        }
    }
}

void draw_polygon(const Scene &scene, const SceneNode &n) {
    if (n.vertices.empty()) return;

    // POLY_MODE_POINTS in the scene marks an invisible pick-only
    // overlay (e.g. the DSKY button hitboxes).  Skip rendering; the
    // pick code still finds them via their vertex bounding box.
    if (n.polygon_mode == SceneNode::PolygonMode::Points) return;

    bind_texture_or_disable(scene, n.texture_index);
    bool textured = (n.texture_index >= 0
                     && n.texture_index < (int)scene.textures.entries.size()
                     && scene.textures.entries[n.texture_index].gl_id != 0);

    // the original panel fills textured polygons regardless of PolygonMode; the
    // POLY_MODE_OUTLINE setting on e.g. the DSKY panel seems to be an
    // editor-only pen mode.  Always fill.
    //
    // Per-vertex color is authoritative for both textured (modulate)
    // and untextured fills -- e.g. AttitudeBallDarkerFilter is an
    // untextured polygon whose Color attribute is opaque yellow but
    // whose per-vertex color is grey with alpha 26, making it a nearly
    // transparent tint over the ball behind it.  Using the base Color
    // for untextured polys occludes anything beneath.
    glBegin(GL_POLYGON);
    for (size_t i = 0; i < n.vertices.size(); ++i) {
        const Vertex &v = n.vertices[i];
        if (textured && i < n.texcoords.size()) {
            glTexCoord2f(n.texcoords[i].u, n.texcoords[i].v);
        }
        glColor4ub(v.color.r, v.color.g, v.color.b, v.color.a);
        glVertex3f(v.x, v.y, v.z);
    }
    glEnd();

    if (textured) glDisable(GL_TEXTURE_2D);
}

void draw_sphere(const Scene &scene, const SceneNode &n) {
    static bool once = false;
    if (!once) {
        once = true;
        std::fprintf(stderr, "draw_sphere name=%s tex=%d radii=(%f,%f,%f) loc=(%f,%f,%f)\n",
                     n.name.c_str(), n.texture_index,
                     n.radii.x, n.radii.y, n.radii.z,
                     n.location.x, n.location.y, n.location.z);
        int tex_ok = (n.texture_index >= 0
                     && n.texture_index < (int)scene.textures.entries.size()
                     && scene.textures.entries[n.texture_index].gl_id != 0);
        std::fprintf(stderr, "  scene.textures.size=%zu, tex_ok=%d\n",
                     scene.textures.entries.size(), tex_ok);
        if (n.texture_index >= 0 && n.texture_index < (int)scene.textures.entries.size())
            std::fprintf(stderr, "  tex[%d]: %s gl_id=%d\n",
                         n.texture_index,
                         scene.textures.entries[n.texture_index].filename.c_str(),
                         scene.textures.entries[n.texture_index].gl_id);
    }
    bind_texture_or_disable(scene, n.texture_index);
    // The scene Radii field stores three basis vectors that, in a full
    // interpretation, describe an ellipsoid and its orientation.  For
    // the FDAI ball (and every sphere in this project) all three are
    // approximately equal magnitude, so treating it as a plain sphere
    // with radius = length of the first non-zero component is exact.
    float r = std::fabs(n.radii.x);
    if (r == 0) r = std::fabs(n.radii.y);
    if (r == 0) r = std::fabs(n.radii.z);
    if (r == 0) r = 1.0f;

    glPushMatrix();

    // Match the original GlsSphere setup for AttitudeBall (from
    // the original FDAI code):
    //   GLS_SPHERE_RADII = 3 basis vectors, actual geometry-transforming
    //                      matrix.  the original sphere pole is local +Y,
    //                      and the Radii vectors reorient it in world
    //                      coords (for AttitudeBall: local +Y -> -X,
    //                      putting the pole on the LEFT rim, horizontal).
    //
    // GL matrix order note: code order runs LAST-to-FIRST when
    // applied to a point.  We want (applied to point in order):
    //     runtime rotations (original DynamicRotate, in sphere-local frame)
    //     -> Rx(-90) (aligns gluSphere's +Z pole with the original +Y pole)
    //     -> Radii   (the original semi-axis reorientation)
    // So the code order is Radii, Rx(-90), then the runtime rotations.
    const float R = r;
    const auto &rm = n.radii_matrix;
    const float radii4x4[16] = {
        rm[0]/R, rm[1]/R, rm[2]/R, 0,   // col 0 (X)
        rm[3]/R, rm[4]/R, rm[5]/R, 0,   // col 1 (Y = original pole)
        rm[6]/R, rm[7]/R, rm[8]/R, 0,   // col 2 (Z)
        0,       0,       0,       1,
    };
    glMultMatrixf(radii4x4);
    glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);   // gluSphere +Z pole -> +Y (the original pole axis)

    // Runtime rotations sequenced EXACTLY as FDAIClass::Roll/Pitch/Yaw
    // in the original ApolloA the original FDAI code:
    //   AttitudeBall->DynamicRotate( -_yaw,   Y_AXIS );
    //   AttitudeBall->DynamicRotate(  _pitch, X_AXIS );
    //   AttitudeBall->DynamicRotate(  _roll,  Z_AXIS );
    // These apply in the original sphere-local frame (before Radii reorients
    // the sphere in world), so they belong INSIDE the Rx(-90)+Radii
    // pair in code order.
    glRotatef(-n.rotation.z, 0.0f, 1.0f, 0.0f);  // -yaw around Y
    glRotatef( n.rotation.y, 1.0f, 0.0f, 0.0f);  //  pitch around X
    glRotatef( n.rotation.x, 0.0f, 0.0f, 1.0f);  //  roll around Z

    // Cull back faces so gluSphere's back hemisphere doesn't overdraw
    // the front (depth test is off globally for the panel work).
    GLboolean was_cull = glIsEnabled(GL_CULL_FACE);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    GLUquadric *q = gluNewQuadric();
    gluQuadricTexture(q, GL_TRUE);
    gluQuadricNormals(q, GLU_SMOOTH);
    // Supersample the sphere geometry: the original Stacks/Slices values
    // (36/18 for AttitudeBall) were sized for a much smaller on-screen
    // ball in the 2009 Windows app.  Bumping tessellation removes the
    // visible facet edges at the ball silhouette without measurable
    // cost (gluSphere is CPU-side triangulation of ~O(slices*stacks)
    // quads, still microseconds at 128*96).
    const int slices = n.slices > 0 ? std::max(n.slices * 4, 96) : 96;
    const int stacks = n.stacks > 0 ? std::max(n.stacks * 2, 72) : 72;
    gluSphere(q, r, slices, stacks);
    gluDeleteQuadric(q);

    if (!was_cull) glDisable(GL_CULL_FACE);
    glPopMatrix();

    if (n.texture_index >= 0) glDisable(GL_TEXTURE_2D);
}

}  // namespace

void render_node(const Scene &scene, const SceneNode &node) {
    if (!node.visible) return;
    // original Blinking(): flashing objects pulse at ~2 Hz.  Skip render
    // during the "off" half of the cycle.
    if (node.flashing && g_blink_off) return;

    // the original panel uses absolute placement -- Groups don't translate their
    // children.  Only apply Location for leaf drawable objects.
    bool xform = (node.kind == NodeKind::Polygon
                  || node.kind == NodeKind::Sphere
                  || node.kind == NodeKind::TextBox
                  || node.kind == NodeKind::Component);
    if (xform) apply_transform(node);
    switch (node.kind) {
        case NodeKind::Polygon:
            if (!g_render_spheres_only) draw_polygon(scene, node);
            break;
        case NodeKind::Sphere:   draw_sphere(scene, node);  break;
        case NodeKind::TextBox:
            // Background quad first, then the text on top.  Uses the
            // vertex data (populated from Vertices: like a GLPolygon)
            // for the box color and extent.
            draw_polygon(scene, node);
            draw_textbox_content(node);
            break;
        case NodeKind::Component:
            if (node.component_scene && node.component_scene->root) {
                // apply_transform already translated to Location; now
                // apply the per-embed scale/rotate.  GL matrix is
                // column-major, matching the scene storage order.
                glMultMatrixf(node.component_transform.data());
                // Render the component's own children directly.  Do NOT
                // re-enter render_node on the root, because that would
                // apply the parent Location a second time.
                for (const auto &c : node.component_scene->root->children)
                    render_node(*node.component_scene, *c);
            }
            break;
        default: break;
    }
    for (const auto &c : node.children) render_node(scene, *c);
    if (xform) glPopMatrix();
}

void render_scene(const Scene &scene) {
    if (scene.root) render_node(scene, *scene.root);
}

}  // namespace panel
