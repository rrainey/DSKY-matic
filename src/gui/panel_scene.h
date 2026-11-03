// Scene graph for the instrument panel, loaded from the JSON files in
// data/panel.  Just enough to render the DSKY and FDAI.

#pragma once

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace panel {

struct Vec3   { float x = 0, y = 0, z = 0; };
struct Color4 { unsigned char r = 255, g = 255, b = 255, a = 255; };

struct TextureEntry {
    std::string filename;      // e.g. "textures/DSKY-FDAI-01.png"
    int gl_id = 0;             // populated on first bind
    int width = 0, height = 0;
};

struct TexturePalette {
    std::vector<TextureEntry> entries;
};

// A vertex: position plus per-vertex color.
struct Vertex {
    float x = 0, y = 0, z = 0;
    Color4 color;
};

// A texture coordinate.  w is unused by ordinary 2D texturing.
struct TexCoord { float u = 0, v = 0, w = 0; };

enum class NodeKind {
    Root,      // implicit outer scope
    Group,
    Polygon,
    Sphere,
    TextBox,
    Component, // reference to another scene file
    Unknown,
};

// Callback signature: user code that fires on pick (mouse click on a
// pickable object).  Bound by name via SceneRuntime::bind_callback.
using PickCallback = std::function<void()>;

struct SceneNode {
    NodeKind kind = NodeKind::Unknown;
    std::string name;
    Vec3 location;
    Vec3 rotation;                          // degrees, XYZ order
    Vec3 rotation_origin;                   // pivot offset from Location for rotation
    Vec3 dynamic_translate;                 // runtime offset added on top of Location
    std::array<float, 16> dcs_matrix;       // column-major, identity by default
    bool flashing = false;                  // if true, render pulses on/off ~2Hz
    bool visible = true;
    bool pickable = true;

    // Polygon-specific
    std::vector<Vertex> vertices;
    std::vector<TexCoord> texcoords;
    int texture_index = -1;
    bool texture_repeat = true;
    Color4 flat_color;                      // used when texture_index < 0
    enum class PolygonMode { Filled, Outline, Points } polygon_mode = PolygonMode::Filled;
    bool polygon_filled = true;             // POLY_MODE_FILLED vs OUTLINE (kept for compat)

    // Sphere-specific: three basis vectors that span the ellipsoid.
    // For a plain sphere all three have the same length (= radius),
    // just permuted.  We keep the whole matrix so orientation is
    // recoverable if we ever need proper ellipsoid rendering.
    Vec3 radii;                             // convenience: length of row 0
    std::array<float, 9> radii_matrix{};    // row-major 3x3
    int stacks = 32, slices = 32;

    // TextBox-specific
    std::string text;
    Color4 text_color{0, 255, 0, 255};   // green default
    int cell_w = 10;                     // character advance, px
    int cell_h = 15;                     // line spacing, px

    // Component-specific: filename and the loaded sub-tree.
    std::string component_file;
    std::shared_ptr<class Scene> component_scene;
    // 4x4 transform embedding the component in the parent's coordinate space.
    std::array<float, 16> component_transform;

    // Children (Group / Root only).
    std::vector<std::shared_ptr<SceneNode>> children;

    // Pick callback -- name resolves via SceneRuntime::bind_callback.
    // Empty string means no callback.
    std::string callback_name;

    SceneNode();
};

class Scene {
public:
    // Ortho canvas size.
    int width = 1200;
    int height = 800;
    std::string title = "ApolloA";

    TexturePalette textures;
    std::shared_ptr<SceneNode> root;

    // Runtime-injected pick callbacks, keyed by object name.
    std::vector<std::pair<std::string, PickCallback>> callbacks;

    // Load a scene file.  Component and texture paths in the file are
    // relative to source_dir.
    static std::shared_ptr<Scene> load(const std::string &path,
                                       const std::string &source_dir);

    std::string source_dir;
};

}  // namespace panel
