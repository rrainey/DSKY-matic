#include "panel_texture.h"
#include "stb_image.h"

#include <GL/gl.h>
#include <GL/glext.h>   // GL_TEXTURE_MAX_ANISOTROPY_EXT, glGenerateMipmap

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace panel {

unsigned int load_texture_file(const std::string &path, int *out_w, int *out_h) {
    // Flip vertically on load: scene TextureCoord V=0 refers to the top
    // of the image file, but with a standard Y-up ortho GL wants V=0
    // to appear at the bottom of the geometry.  Flipping in memory
    // reconciles the two conventions.
    int w, h, chan;
    stbi_set_flip_vertically_on_load(1);
    unsigned char *pixels = stbi_load(path.c_str(), &w, &h, &chan, 4);
    if (!pixels) {
        std::fprintf(stderr, "panel_texture: failed to load %s: %s\n",
                     path.c_str(), stbi_failure_reason());
        return 0;
    }
    GLuint tex = 0;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    // Trilinear minification (LINEAR_MIPMAP_LINEAR = filter between mip
    // levels).  Combined with anisotropic filtering below this gives the
    // FDAI ball sharp numbers at grazing angles without moire.
    // GL_GENERATE_MIPMAP=TRUE (GL 1.4 fixed-pipeline mechanism) tells
    // the driver to auto-build the mip chain from the level-0 upload
    // that immediately follows; used instead of glGenerateMipmap() so
    // we don't need an extension loader.  Must be set BEFORE glTexImage2D.
    glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP,   GL_TRUE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0,
                 GL_RGBA, GL_UNSIGNED_BYTE, pixels);
    // Anisotropic filtering.  Universally available on modern hardware
    // via the KHR/EXT extension.  Cap at 16x; the driver clamps to its
    // real max via GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT.
#ifdef GL_TEXTURE_MAX_ANISOTROPY_EXT
    GLfloat max_aniso = 1.0f;
    glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &max_aniso);
    if (max_aniso > 1.0f) {
        GLfloat use = max_aniso > 16.0f ? 16.0f : max_aniso;
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, use);
    }
#endif
    stbi_image_free(pixels);
    if (out_w) *out_w = w;
    if (out_h) *out_h = h;
    return tex;
}

static void visit_scenes(SceneNode *n, std::vector<Scene *> &out,
                         std::unordered_set<Scene *> &seen) {
    if (!n) return;
    if (n->component_scene) {
        Scene *s = n->component_scene.get();
        if (!seen.count(s)) {
            seen.insert(s);
            out.push_back(s);
            if (s->root) visit_scenes(s->root.get(), out, seen);
        }
    }
    for (auto &c : n->children) visit_scenes(c.get(), out, seen);
}

void load_scene_textures(Scene &scene) {
    load_palette_textures(scene.textures, scene.source_dir);
    std::vector<Scene *> subs;
    std::unordered_set<Scene *> seen{&scene};
    if (scene.root) visit_scenes(scene.root.get(), subs, seen);
    for (Scene *s : subs) load_palette_textures(s->textures, s->source_dir);
}

// Process-wide dedupe: absolute path -> (gl_id, w, h).  Multiple
// DSKYELSegment/DSKYELSign sub-scenes reference the same atlas file;
// we only want to bind it once.
static std::unordered_map<std::string, std::tuple<unsigned int, int, int>> g_tex_cache;

void load_palette_textures(TexturePalette &palette, const std::string &source_dir) {
    for (auto &e : palette.entries) {
        if (e.gl_id != 0 || e.filename.empty()) continue;
        std::filesystem::path p(e.filename);
        if (p.is_relative()) p = std::filesystem::path(source_dir) / p;
        std::string key = p.lexically_normal().string();
        auto it = g_tex_cache.find(key);
        if (it != g_tex_cache.end()) {
            e.gl_id  = std::get<0>(it->second);
            e.width  = std::get<1>(it->second);
            e.height = std::get<2>(it->second);
            continue;
        }
        e.gl_id = load_texture_file(key, &e.width, &e.height);
        if (e.gl_id != 0) {
            g_tex_cache.emplace(key, std::make_tuple(e.gl_id, e.width, e.height));
        }
    }
}

}  // namespace panel
