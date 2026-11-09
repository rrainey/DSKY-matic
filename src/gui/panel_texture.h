// Texture loading via stb_image with a simple cache keyed by path.

#pragma once

#include "panel_scene.h"

namespace panel {

// Load all textures in the palette that haven't been bound yet.
// Relative filenames are resolved against source_dir.  Silently
// tolerates unreadable/absent textures (leaves gl_id = 0).
void load_palette_textures(TexturePalette &palette, const std::string &source_dir);

// Load palettes for a scene AND every distinct component sub-scene
// reachable through it.
void load_scene_textures(Scene &scene);

// Load a single file into a new OpenGL texture.  Returns 0 on failure.
unsigned int load_texture_file(const std::string &path, int *out_w = nullptr, int *out_h = nullptr);

}  // namespace panel
