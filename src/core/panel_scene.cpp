#include "panel_scene.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <unordered_map>

using nlohmann::json;

namespace panel {

SceneNode::SceneNode() {
    dcs_matrix          = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    component_transform = {1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
}

namespace {

// Each component file is read once, but every Component instance gets its
// own node tree so digits can carry independent visibility and rotation.
using FileCache = std::unordered_map<std::string, std::shared_ptr<json>>;

std::shared_ptr<Scene> build_scene(const json &doc, const std::string &source_dir, FileCache &cache);

Vec3 vec3(const json &n, const char *key) {
    if (!n.contains(key)) return {};
    const auto &a = n[key];
    return {a[0].get<float>(), a[1].get<float>(), a[2].get<float>()};
}

Color4 color4(const json &a) {
    return {a[0].get<unsigned char>(), a[1].get<unsigned char>(),
            a[2].get<unsigned char>(), a[3].get<unsigned char>()};
}

NodeKind kind_from_string(const std::string &k) {
    if (k == "group")     return NodeKind::Group;
    if (k == "polygon")   return NodeKind::Polygon;
    if (k == "sphere")    return NodeKind::Sphere;
    if (k == "text")      return NodeKind::TextBox;
    if (k == "component") return NodeKind::Component;
    return NodeKind::Unknown;
}

std::shared_ptr<json> read_json(const std::string &path, FileCache &cache) {
    auto it = cache.find(path);
    if (it != cache.end()) return it->second;
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open scene file " + path);
    auto doc = std::make_shared<json>(json::parse(in));
    cache.emplace(path, doc);
    return doc;
}

std::shared_ptr<SceneNode> build_node(const json &n, const std::string &source_dir, FileCache &cache) {
    auto sn = std::make_shared<SceneNode>();
    sn->kind = kind_from_string(n.at("kind").get<std::string>());
    sn->name = n.value("name", std::string());
    sn->location = vec3(n, "location");
    sn->rotation = vec3(n, "rotation");
    sn->rotation_origin = vec3(n, "rotation_origin");
    if (n.contains("dcs_matrix")) sn->dcs_matrix = n["dcs_matrix"].get<std::array<float, 16>>();
    sn->visible = n.value("visible", true);
    sn->pickable = n.value("pickable", true);
    sn->texture_index = n.value("texture_index", -1);
    sn->texture_repeat = n.value("texture_repeat", true);
    if (n.contains("color")) sn->flat_color = color4(n["color"]);

    const std::string mode = n.value("polygon_mode", std::string("filled"));
    if (mode == "points")       sn->polygon_mode = SceneNode::PolygonMode::Points;
    else if (mode == "outline") sn->polygon_mode = SceneNode::PolygonMode::Outline;
    sn->polygon_filled = (sn->polygon_mode == SceneNode::PolygonMode::Filled);

    if (n.contains("vertices")) {
        for (const auto &row : n["vertices"]) {
            Vertex v;
            v.x = row[0].get<float>(); v.y = row[1].get<float>(); v.z = row[2].get<float>();
            v.color = {row[3].get<unsigned char>(), row[4].get<unsigned char>(),
                       row[5].get<unsigned char>(), row[6].get<unsigned char>()};
            sn->vertices.push_back(v);
        }
    }
    if (n.contains("texcoords")) {
        for (const auto &row : n["texcoords"]) {
            sn->texcoords.push_back({row[0].get<float>(), row[1].get<float>(), row[2].get<float>()});
        }
    }

    switch (sn->kind) {
        case NodeKind::Sphere: {
            // Three basis vectors spanning the ellipsoid, row-major.
            sn->radii_matrix = n.at("radii_matrix").get<std::array<float, 9>>();
            const auto &v = sn->radii_matrix;
            auto len = [](float a, float b, float c) { return std::sqrt(a * a + b * b + c * c); };
            sn->radii = {len(v[0], v[1], v[2]), len(v[3], v[4], v[5]), len(v[6], v[7], v[8])};
            sn->stacks = n.value("stacks", 32);
            sn->slices = n.value("slices", 32);
            break;
        }
        case NodeKind::TextBox:
            sn->text = n.value("text", std::string());
            if (n.contains("text_color")) sn->text_color = color4(n["text_color"]);
            sn->cell_w = n.value("cell_w", 10);
            sn->cell_h = n.value("cell_h", 15);
            break;
        case NodeKind::Component: {
            sn->component_file = n.at("component").get<std::string>();
            if (n.contains("component_transform"))
                sn->component_transform = n["component_transform"].get<std::array<float, 16>>();
            std::filesystem::path p(sn->component_file);
            if (p.is_relative()) p = std::filesystem::path(source_dir) / p;
            sn->component_scene = build_scene(*read_json(p.lexically_normal().string(), cache),
                                              source_dir, cache);
            break;
        }
        case NodeKind::Group:
            if (n.contains("children"))
                for (const auto &c : n["children"])
                    sn->children.push_back(build_node(c, source_dir, cache));
            break;
        default:
            break;
    }
    return sn;
}

std::shared_ptr<Scene> build_scene(const json &doc, const std::string &source_dir, FileCache &cache) {
    auto scene = std::make_shared<Scene>();
    scene->source_dir = source_dir;
    scene->width  = doc.value("width", 1200);
    scene->height = doc.value("height", 800);
    scene->title  = doc.value("title", std::string("ApolloA"));

    for (const auto &f : doc.at("textures")) {
        TextureEntry e;
        e.filename = f.get<std::string>();
        scene->textures.entries.push_back(e);
    }

    auto root = std::make_shared<SceneNode>();
    root->kind = NodeKind::Root;
    root->name = "root";
    for (const auto &c : doc.at("nodes")) root->children.push_back(build_node(c, source_dir, cache));
    scene->root = root;
    return scene;
}

}  // namespace

std::shared_ptr<Scene> Scene::load(const std::string &path, const std::string &source_dir) {
    FileCache cache;
    return build_scene(*read_json(path, cache), source_dir, cache);
}

}  // namespace panel
