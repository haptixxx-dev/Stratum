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

# ═══════════════════════════════════════════════════════════════════
# GLOBAL SETTINGS
# ═══════════════════════════════════════════════════════════════════
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

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
    src/osm/tile_manager.cpp


)

target_include_directories(stratum_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${EXTERNAL_DIR}
)

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
    src/core/input.cpp
    
    # Renderer
    src/renderer/gpu_device.cpp
    src/renderer/pipeline.cpp
    src/renderer/mesh.cpp
    src/renderer/texture.cpp
    
    # Editor
    src/editor/editor.cpp
    src/editor/camera.cpp
    src/editor/im3d_impl.cpp
    src/editor/panels/viewport_panel.cpp
    src/editor/panels/scene_panel.cpp
    src/editor/panels/properties_panel.cpp
    src/editor/panels/osm_panel.cpp
    src/editor/gizmos.cpp
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
    ImGuiFileDialog
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
# INSTALL
# ═══════════════════════════════════════════════════════════════════
install(TARGETS stratum RUNTIME DESTINATION bin)
install(DIRECTORY assets/ DESTINATION share/stratum/assets)
install(DIRECTORY scripts/ DESTINATION share/stratum/scripts)

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
