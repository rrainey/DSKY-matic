cmake_minimum_required(VERSION 3.16)
project(DSKYmatic C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# ---------------------------------------------------------------------------
# Apollo-Saturn-IB sim engine (plant + LVDC + IMU physics; submodule).
# Provides: saturnib (no yaAGC), saturnib_cmc (yaAGC bridge).
# Together these replace what used to live in Apollo-Saturn-IB/src/{sim,cmc}.
# ---------------------------------------------------------------------------
add_subdirectory(Apollo-Saturn-IB EXCLUDE_FROM_ALL)

# Runtime data (AGC ROM, PAD loads, downlink TSVs, panel scene, Earth
# map binaries) lives in this project's data/ subtree.  Run the mode
# binaries from there.  batch_sim's scenario files stay in the
# submodule under Apollo-Saturn-IB/data/examples/.
set(DSKYMATIC_AUDIO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/audio
    CACHE PATH "DSKY-matic audio assets (relay clicks, etc.)")

# ---------------------------------------------------------------------------
# dskymatic_audio: OpenAL+alut sound wrapper, vendored from ~/src/dsky-hw
# (dso_core — SoundManager / SoundSource / Sound).  Used by any mode that
# plays audio; currently relay clicks fired by the DSKY ch010 dispatch.
# ---------------------------------------------------------------------------
option(DSKYMATIC_AUDIO "Build audio library (needs libopenal-dev + libalut-dev)" ON)

if(DSKYMATIC_AUDIO)
    add_library(dskymatic_audio STATIC
        src/audio/Sound.cpp
        src/audio/SoundSource.cpp
        src/audio/SoundManager.cpp
    )
    target_include_directories(dskymatic_audio PUBLIC src/audio)
    target_link_libraries(dskymatic_audio PUBLIC openal alut)
endif()

# ---------------------------------------------------------------------------
# dskymatic_core: shared panel/scene/pick/render + texture loader.
# Independent of the AGC bridge; usable by any mode that opens a GL window.
# ---------------------------------------------------------------------------
add_library(dskymatic_core STATIC
    src/core/panel_pick.cpp
    src/core/panel_render.cpp
    src/core/panel_scene.cpp
    src/core/panel_texture.cpp
    src/core/stb_image_impl.cpp
    src/core/dsky_click.cpp
    src/core/dsky_dispatch.cpp
    src/core/flasher.cpp
    src/core/map_view.cpp
)
target_include_directories(dskymatic_core PUBLIC
    src/core
    Apollo-Saturn-IB/third_party
    Apollo-Saturn-IB/third_party/stb
)
target_link_libraries(dskymatic_core PUBLIC saturnib saturnib_cmc)
if(DSKYMATIC_AUDIO)
    target_link_libraries(dskymatic_core PUBLIC dskymatic_audio)
    target_compile_definitions(dskymatic_core PUBLIC DSKYMATIC_AUDIO=1)
    target_compile_definitions(dskymatic_core PRIVATE
        DSKYMATIC_AUDIO_DIR="${DSKYMATIC_AUDIO_DIR}")
endif()

find_package(OpenGL REQUIRED)
find_package(GLUT   REQUIRED)
target_link_libraries(dskymatic_core PUBLIC OpenGL::GL OpenGL::GLU GLUT::GLUT)

# ---------------------------------------------------------------------------
# Mode binaries.  Each mode is a distinct executable with its own entry
# point and its own link graph.  Only mode 3 (studio) is implemented at
# this checkpoint — modes 1 and 2 arrive in a later phase.
#
#   studio   — full on-screen DSKY + FDAI + panel.  No hardware.
#              (Reproduces current Apollo-Saturn-IB `apolloa`.)
#   panel    — GL window with FDAI + scenario only; DSKY on hardware.
#   headless — no GL; hardware DSKY only.
# ---------------------------------------------------------------------------
# ---------------------------------------------------------------------------
# dskymatic_hardware: DSKY-matic hardware driver abstraction.
# Interface in IDskyHardware.h; StderrDskyHardware prints updates to
# stderr (useful without a real DSKY-matic attached, and the default
# driver until a real Pi GPIO/serial driver is ported from ~/src/dsky-hw).
# No GL dependency.
# ---------------------------------------------------------------------------
add_library(dskymatic_hardware STATIC
    src/hardware/StderrDskyHardware.cpp
)
target_include_directories(dskymatic_hardware PUBLIC src/hardware)
target_link_libraries(dskymatic_hardware PUBLIC dskymatic_core)

add_executable(dskymatic-studio src/modes/studio_main.cpp)
target_link_libraries(dskymatic-studio PRIVATE dskymatic_core)

# dskymatic-headless: mode 1.  No GL/GLUT link; drives hardware via
# IDskyHardware (default StderrDskyHardware prints to stderr).
add_executable(dskymatic-headless src/modes/headless_main.cpp)
target_link_libraries(dskymatic-headless PRIVATE
    dskymatic_hardware
    saturnib
    saturnib_cmc
)
if(DSKYMATIC_AUDIO)
    target_link_libraries(dskymatic-headless PRIVATE dskymatic_audio)
endif()
