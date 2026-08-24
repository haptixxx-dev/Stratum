cmake_minimum_required(VERSION 3.24)

# Read version from VERSION file
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" STRATUM_VERSION)
string(STRIP "${STRATUM_VERSION}" STRATUM_VERSION)

project(Stratum VERSION ${STRATUM_VERSION} LANGUAGES C CXX)

message(STATUS "Stratum version: ${STRATUM_VERSION}")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ═══════════════════════════════════════════════════════════════════
# OPTIONS
# ═══════════════════════════════════════════════════════════════════
option(STRATUM_ENABLE_TRACY "Enable Tracy profiler" ON)
option(STRATUM_ENABLE_PYTHON "Enable Python scripting" ON)
option(STRATUM_BUILD_TESTS "Build tests" OFF)
option(STRATUM_BUILD_DOCS "Build documentation with Doxygen" OFF)
option(STRATUM_USE_LLD "Link with LLD instead of the default linker" OFF)

# ═══════════════════════════════════════════════════════════════════
# GLOBAL SETTINGS
# ═══════════════════════════════════════════════════════════════════
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# ═══════════════════════════════════════════════════════════════════
# LINKER
# ═══════════════════════════════════════════════════════════════════
# Opt-in, and deliberately not CMAKE_LINKER_TYPE: that is CMake 3.29+ and is
# silently ignored below it, while this project's floor is 3.24. LLD works with
# both GCC and Clang, so the linker choice stays independent of the compiler.
# The compiler itself cannot be set here at all -- it must be known before
# project(), so pass -DCMAKE_CXX_COMPILER=clang++ at configure time.
if(STRATUM_USE_LLD AND NOT MSVC)
    include(CheckLinkerFlag)
    check_linker_flag(CXX "-fuse-ld=lld" STRATUM_HAVE_LLD)
    if(STRATUM_HAVE_LLD)
        add_link_options(-fuse-ld=lld)
        message(STATUS "Linker: LLD")
    else()
        message(WARNING "STRATUM_USE_LLD is ON but -fuse-ld=lld does not work; using the default linker")
    endif()
endif()

# ═══════════════════════════════════════════════════════════════════
# EXTERNAL DIRECTORY
# ═══════════════════════════════════════════════════════════════════
set(EXTERNAL_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external)

# ═══════════════════════════════════════════════════════════════════
# EXTERNAL DEPENDENCIES
# ═══════════════════════════════════════════════════════════════════
add_subdirectory(external)

# ═══════════════════════════════════════════════════════════════════
# STRATUM CORE (The "Brain")
# ═══════════════════════════════════════════════════════════════════
# This library is engine-agnostic. It contains logic for:
# - OSM Parsing & Coordinates
# - Procedural Generation Algorithms
# - Data Structures (Scene, Components)
# - Math & Geometry processing
# It should NOT depend on SDL, OpenGL, ImGui.
# ═══════════════════════════════════════════════════════════════════
add_library(stratum_core STATIC
    # OSM
    src/osm/parser.cpp
    src/osm/coordinates.cpp
    src/osm/mesh_builder.cpp
    src/osm/quadtree.cpp

    # OSM road network
    src/osm/road/road_graph.cpp
    src/osm/road/road_style.cpp
    src/osm/road/road_profile.cpp
    src/osm/road/centerline.cpp
    src/osm/road/corridor.cpp
    src/osm/road/road_elevation.cpp
    src/osm/road/junction_trim.cpp
    src/osm/road/junction_polygon.cpp
    src/osm/road/junction_curb.cpp
    src/osm/road/junction_special.cpp
    src/osm/road/junction_builder.cpp

    # P5 detail features
    src/osm/road/sidewalk_dedup.cpp
    src/osm/road/marking_atlas.cpp
    src/osm/road/markings.cpp
    src/osm/road/crossings.cpp

    # P6 structures
    src/osm/road/bridge_builder.cpp
    src/osm/road/tunnel_builder.cpp

    # Geometry reduction. Ordered before the builder because the builder calls
    # both: select_stations() between the elevation solve and the extrusion, and
    # merge_coplanar_quads() inside the per-piece finish.
    src/osm/road/tessellation.cpp

    src/osm/road/road_network_builder.cpp

    # P7 game-ready output
    src/osm/road/mesh_optimize.cpp
    src/osm/road/collision_mesh.cpp
    src/osm/road/road_export.cpp

    # Chunk-level LOD. Built by the SPATIAL INDEX once it knows which pieces
    # landed in which leaf, not by RoadNetworkBuilder; see lod_chunk.hpp.
    src/osm/road/lod_chunk.cpp

    # Procedural Generation
    src/procgen/noise.cpp
    src/procgen/terrain_generator.cpp
    src/procgen/terrain_carve.cpp
    src/procgen/terrain_mesh_builder.cpp
    src/procgen/terrain_tile_manager.cpp
)

target_include_directories(stratum_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${EXTERNAL_DIR}
)

# Vulkan clips depth to [0,1]; GLM defaults to OpenGL's [-1,1]. Without this,
# glm::perspective wastes half the depth range and clips geometry nearer than
# ~2x the near plane. PUBLIC and set here rather than per-file because every
# translation unit that builds or consumes a projection matrix must agree --
# a mismatch is an ODR violation with no diagnostic.
target_compile_definitions(stratum_core PUBLIC GLM_FORCE_DEPTH_ZERO_TO_ONE)

target_link_libraries(stratum_core PUBLIC
    # Math
    glm::glm

    # OSM
    osmium

    # Geometry
    Clipper2
    meshoptimizer
    draco::draco
    earcut

    # Threading
    enkiTS

    # Data & Utils
    spdlog::spdlog
    nlohmann_json::nlohmann_json
    EnTT::EnTT

    # Spatial
    bvh
)

# ═══════════════════════════════════════════════════════════════════
# STRATUM EDITOR (The "Visualizer")
# ═══════════════════════════════════════════════════════════════════
# This library contains the editing tools, renderer, and UI.
# It is specific to this SDL3+ImGui application.
# ═══════════════════════════════════════════════════════════════════
add_library(stratum_editor_lib STATIC
    # Core
    src/core/application.cpp
    src/core/window.cpp

    # Renderer
    src/renderer/gpu_renderer.cpp
    src/renderer/gpu_buffer_pool.cpp
    src/renderer/texture.cpp
    src/renderer/material_library.cpp
    src/renderer/procedural_texture.cpp

    # Editor
    src/editor/editor.cpp
    src/editor/camera.cpp
    src/editor/im3d_impl.cpp
    src/editor/panels/procgen_panel.cpp
    src/editor/panels/material_panel.cpp
)

target_include_directories(stratum_editor_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${EXTERNAL_DIR}
)

target_link_libraries(stratum_editor_lib PUBLIC
    stratum_core

    # Core
    SDL3::SDL3
    imgui

    # Import/Export
    assimp

    # Materials
    MaterialXCore
    MaterialXFormat

    # Textures
    ktx

    # Compression
    lz4

    # Editor
    ImGuizmo
    im3d
    imnodes
)

# Tracy (optional)
if(STRATUM_ENABLE_TRACY)
    target_link_libraries(stratum_editor_lib PUBLIC TracyClient)
    target_compile_definitions(stratum_editor_lib PUBLIC STRATUM_ENABLE_TRACY)
endif()

# Python (optional)
if(STRATUM_ENABLE_PYTHON)
    target_link_libraries(stratum_editor_lib PUBLIC pybind11::embed)
    target_compile_definitions(stratum_editor_lib PUBLIC STRATUM_ENABLE_PYTHON)
endif()

# ═══════════════════════════════════════════════════════════════════
# STRATUM EXECUTABLE
# ═══════════════════════════════════════════════════════════════════
add_executable(stratum src/main.cpp)
target_link_libraries(stratum PRIVATE stratum_editor_lib)

# ═══════════════════════════════════════════════════════════════════
# SHADERS (GLSL -> SPIR-V)
# ═══════════════════════════════════════════════════════════════════
# assets/shaders holds GLSL sources AND the .spv the runtime actually loads, and
# until now nothing connected them: editing mesh_pbr.frag changed nothing until
# somebody remembered to run glslc by hand. A checked-in .spv that no longer
# matches its source is a trap that costs an afternoon, so the compile is a build
# step.
#
# Three things here look wrong and are deliberate:
#
#  1. The .spv is written back into the SOURCE tree, not the build tree. The
#     runtime path is SDL_GetBasePath() + "../../assets/shaders/...", and the
#     .spv are checked in, so a build-tree output would compile shaders nobody
#     loads.
#  2. glslc is OPTIONAL. A checkout without it must still build against the
#     committed .spv, hence the find_program guard and the message below rather
#     than a REQUIRED that fails configuration.
#  3. --target-env=vulkan1.0 matches what SDL's Vulkan backend consumes. A newer
#     target env emits SPIR-V that SDL_CreateGPUShader may reject.
#
# Incremental by construction: add_custom_command re-runs only when its DEPENDS
# source is newer than its OUTPUT. No depfile is generated -- glslc's -MD writes
# the .d next to the output, which would litter the source tree, and none of
# these four shaders uses #include. Add one here if that ever changes.
find_program(STRATUM_GLSLC NAMES glslc HINTS /usr/bin /usr/sbin /usr/local/bin)

set(STRATUM_SHADER_SOURCES
    mesh.vert
    mesh.frag
    mesh_pbr.vert
    mesh_pbr.frag
)

set(STRATUM_SHADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/assets/shaders)

if(STRATUM_GLSLC)
    message(STATUS "Shader compiler: ${STRATUM_GLSLC}")

    set(STRATUM_SPV_OUTPUTS "")
    foreach(shader IN LISTS STRATUM_SHADER_SOURCES)
        add_custom_command(
            OUTPUT  ${STRATUM_SHADER_DIR}/${shader}.spv
            COMMAND ${STRATUM_GLSLC}
                    --target-env=vulkan1.0
                    -O
                    ${STRATUM_SHADER_DIR}/${shader}
                    -o ${STRATUM_SHADER_DIR}/${shader}.spv
            DEPENDS ${STRATUM_SHADER_DIR}/${shader}
            COMMENT "glslc ${shader}"
            VERBATIM
        )
        list(APPEND STRATUM_SPV_OUTPUTS ${STRATUM_SHADER_DIR}/${shader}.spv)
    endforeach()

    add_custom_target(stratum_shaders ALL DEPENDS ${STRATUM_SPV_OUTPUTS})

    # The editor library loads the .spv at runtime, so the executable is what has
    # to wait for them. A link-time dependency is not needed and would serialise
    # the build for no reason.
    add_dependencies(stratum stratum_shaders)
else()
    # Say exactly what is missing, what the consequence is, and how to fix it.
    # A build that silently used stale SPIR-V is the failure mode this avoids.
    set(STRATUM_MISSING_SPV "")
    foreach(shader IN LISTS STRATUM_SHADER_SOURCES)
        if(NOT EXISTS ${STRATUM_SHADER_DIR}/${shader}.spv)
            list(APPEND STRATUM_MISSING_SPV ${shader}.spv)
        endif()
    endforeach()

    if(STRATUM_MISSING_SPV)
        list(JOIN STRATUM_MISSING_SPV ", " STRATUM_MISSING_SPV_TEXT)
        message(FATAL_ERROR
            "glslc was not found and these pre-compiled shaders are missing: "
            "${STRATUM_MISSING_SPV_TEXT}. Stratum cannot start without them. Install "
            "shaderc (Arch: pacman -S shaderc) and re-configure, or pass "
            "-DSTRATUM_GLSLC=/path/to/glslc.")
    else()
        message(WARNING
            "glslc was not found, so assets/shaders/*.glsl will NOT be compiled. "
            "The build will use the committed .spv, which may be older than the "
            "GLSL sources next to them. Install shaderc (Arch: pacman -S shaderc) "
            "and re-configure, or pass -DSTRATUM_GLSLC=/path/to/glslc.")
    endif()
endif()

# ═══════════════════════════════════════════════════════════════════
# TESTS
# ═══════════════════════════════════════════════════════════════════
# enable_testing() must live in the top-level file: ctest only discovers tests
# registered below a directory where it was enabled at or above the top level.
if(STRATUM_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# ═══════════════════════════════════════════════════════════════════
# INSTALL
# ═══════════════════════════════════════════════════════════════════
install(TARGETS stratum RUNTIME DESTINATION bin)
install(DIRECTORY assets/ DESTINATION share/stratum/assets)
# scripts/ does not exist yet (Python scripting is not wired up). Installing a
# missing directory is a hard error, so only install it if it is actually there.
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts)
    install(DIRECTORY scripts/ DESTINATION share/stratum/scripts)
endif()

# ═══════════════════════════════════════════════════════════════════
# DOCUMENTATION (Doxygen)
# ═══════════════════════════════════════════════════════════════════
if(STRATUM_BUILD_DOCS)
    find_package(Doxygen REQUIRED)

    if(DOXYGEN_FOUND)
        set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile)

        add_custom_target(docs
            COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_IN}
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            COMMENT "Generating API documentation with Doxygen"
            VERBATIM
        )

        message(STATUS "Doxygen found: Documentation target 'docs' available")
    else()
        message(WARNING "Doxygen not found: Documentation will not be built")
    endif()
endif()
