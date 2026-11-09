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

# Re-expose Apollo-Saturn-IB's data dir for the mode binaries.
set(SATURN_IB_DATA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Apollo-Saturn-IB/data
    CACHE PATH "Apollo-Saturn-IB runtime data (ROM, PAD loads, downlink TSVs)")

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
)
target_include_directories(dskymatic_core PUBLIC
    src/core
    Apollo-Saturn-IB/third_party
    Apollo-Saturn-IB/third_party/stb
)
target_link_libraries(dskymatic_core PUBLIC saturnib saturnib_cmc)

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
add_executable(dskymatic-studio src/modes/studio_main.cpp)
target_link_libraries(dskymatic-studio PRIVATE dskymatic_core)
