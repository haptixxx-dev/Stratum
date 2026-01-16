# ═══════════════════════════════════════════════════════════════════
# STRATUM - EXTERNAL DEPENDENCIES
# ═══════════════════════════════════════════════════════════════════

set(EXTERNAL_DIR ${CMAKE_CURRENT_SOURCE_DIR})

# ───────────────────────────────────────────────────────────────────
# SDL3
# ───────────────────────────────────────────────────────────────────
set(SDL_SHARED OFF CACHE BOOL "" FORCE)
set(SDL_STATIC ON CACHE BOOL "" FORCE)
set(SDL_TEST OFF CACHE BOOL "" FORCE)
add_subdirectory(SDL)

# ───────────────────────────────────────────────────────────────────
# GLM
# ───────────────────────────────────────────────────────────────────
add_subdirectory(glm)

# ───────────────────────────────────────────────────────────────────
# IMGUI (manual setup - no CMakeLists)
# ───────────────────────────────────────────────────────────────────
add_library(imgui STATIC
    ${EXTERNAL_DIR}/imgui/imgui.cpp
    ${EXTERNAL_DIR}/imgui/imgui_demo.cpp
    ${EXTERNAL_DIR}/imgui/imgui_draw.cpp
    ${EXTERNAL_DIR}/imgui/imgui_tables.cpp
    ${EXTERNAL_DIR}/imgui/imgui_widgets.cpp
    # SDL3 + SDL_GPU backends
    ${EXTERNAL_DIR}/imgui/backends/imgui_impl_sdl3.cpp
    ${EXTERNAL_DIR}/imgui/backends/imgui_impl_sdlrenderer3.cpp
)
target_include_directories(imgui PUBLIC 
    ${EXTERNAL_DIR}/imgui
    ${EXTERNAL_DIR}/imgui/backends
)
target_link_libraries(imgui PUBLIC SDL3::SDL3)

# ───────────────────────────────────────────────────────────────────
# IMGUIZMO (manual setup)
# ───────────────────────────────────────────────────────────────────
add_library(ImGuizmo STATIC
    ${EXTERNAL_DIR}/ImGuizmo/ImGuizmo.cpp
    ${EXTERNAL_DIR}/ImGuizmo/GraphEditor.cpp
    ${EXTERNAL_DIR}/ImGuizmo/ImCurveEdit.cpp
    ${EXTERNAL_DIR}/ImGuizmo/ImGradient.cpp
    ${EXTERNAL_DIR}/ImGuizmo/ImSequencer.cpp
)
target_include_directories(ImGuizmo PUBLIC ${EXTERNAL_DIR}/ImGuizmo)
target_link_libraries(ImGuizmo PUBLIC imgui)

# ───────────────────────────────────────────────────────────────────
# IM3D (manual setup)
# ───────────────────────────────────────────────────────────────────
add_library(im3d STATIC ${EXTERNAL_DIR}/im3d/im3d.cpp)
target_include_directories(im3d PUBLIC ${EXTERNAL_DIR}/im3d)

# ───────────────────────────────────────────────────────────────────
# IMGUI FILE DIALOG
# ───────────────────────────────────────────────────────────────────
add_subdirectory(ImGuiFileDialog)
target_link_libraries(ImGuiFileDialog PUBLIC imgui)

# ───────────────────────────────────────────────────────────────────
# IMNODES (node editor)
# ───────────────────────────────────────────────────────────────────
set(IMNODES_IMGUI_TARGET_NAME imgui CACHE STRING "" FORCE)
set(IMNODES_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(imnodes)

# ───────────────────────────────────────────────────────────────────
# IMGUI COLOR TEXT EDIT (header-only)
# ───────────────────────────────────────────────────────────────────
add_library(ImGuiColorTextEdit INTERFACE)
target_include_directories(ImGuiColorTextEdit INTERFACE ${EXTERNAL_DIR}/ImGuiColorTextEdit)

# ───────────────────────────────────────────────────────────────────
# SPDLOG
# ───────────────────────────────────────────────────────────────────
set(SPDLOG_BUILD_EXAMPLE OFF CACHE BOOL "" FORCE)
add_subdirectory(spdlog)

# ───────────────────────────────────────────────────────────────────
# NLOHMANN JSON
# ───────────────────────────────────────────────────────────────────
set(JSON_BuildTests OFF CACHE BOOL "" FORCE)
add_subdirectory(json)

# ───────────────────────────────────────────────────────────────────
# ENTT
# ───────────────────────────────────────────────────────────────────
add_subdirectory(entt)

# ───────────────────────────────────────────────────────────────────
# LIBOSMIUM (header-only, needs protozero)
# ───────────────────────────────────────────────────────────────────
add_library(osmium INTERFACE)
target_include_directories(osmium INTERFACE 
    ${EXTERNAL_DIR}/libosmium/include
    ${EXTERNAL_DIR}/protozero/include
)

# ───────────────────────────────────────────────────────────────────
# EARCUT (header-only)
# ───────────────────────────────────────────────────────────────────
add_library(earcut INTERFACE)
target_include_directories(earcut INTERFACE ${EXTERNAL_DIR}/earcut/include)
# libosmium needs zlib, bzip2, expat for full functionality
find_package(ZLIB REQUIRED)
find_package(BZip2 REQUIRED)
find_package(EXPAT REQUIRED)
target_link_libraries(osmium INTERFACE ZLIB::ZLIB BZip2::BZip2 EXPAT::EXPAT)

# ───────────────────────────────────────────────────────────────────
# CLIPPER2
# ───────────────────────────────────────────────────────────────────
set(CLIPPER2_EXAMPLES OFF CACHE BOOL "" FORCE)
set(CLIPPER2_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(Clipper2/CPP)

# ───────────────────────────────────────────────────────────────────
# MESHOPTIMIZER
# ───────────────────────────────────────────────────────────────────
set(MESHOPT_BUILD_DEMO OFF CACHE BOOL "" FORCE)
set(MESHOPT_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
set(MESHOPT_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
add_subdirectory(meshoptimizer)

# ───────────────────────────────────────────────────────────────────
# DRACO
# ───────────────────────────────────────────────────────────────────
set(DRACO_TRANSCODER_SUPPORTED OFF CACHE BOOL "" FORCE)
set(DRACO_JS_GLUE OFF CACHE BOOL "" FORCE)
set(DRACO_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
add_subdirectory(draco)

# ───────────────────────────────────────────────────────────────────
# ASSIMP
# ───────────────────────────────────────────────────────────────────
set(ASSIMP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(ASSIMP_INSTALL OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_SAMPLES OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_ASSIMP_TOOLS OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
# Disable importers you don't need to speed up build
set(ASSIMP_BUILD_ALL_IMPORTERS_BY_DEFAULT OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_OBJ_IMPORTER ON CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_FBX_IMPORTER ON CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_GLTF_IMPORTER ON CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_BLEND_IMPORTER ON CACHE BOOL "" FORCE)
# Enable exporters
set(ASSIMP_BUILD_ALL_EXPORTERS_BY_DEFAULT OFF CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_OBJ_EXPORTER ON CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_GLTF_EXPORTER ON CACHE BOOL "" FORCE)
set(ASSIMP_BUILD_FBX_EXPORTER ON CACHE BOOL "" FORCE)
add_subdirectory(assimp)

# ───────────────────────────────────────────────────────────────────
# MATERIALX
# ───────────────────────────────────────────────────────────────────
set(MATERIALX_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(MATERIALX_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(MATERIALX_BUILD_RENDER OFF CACHE BOOL "" FORCE)
set(MATERIALX_BUILD_VIEWER OFF CACHE BOOL "" FORCE)
add_subdirectory(MaterialX)

# ───────────────────────────────────────────────────────────────────
# STB (header-only)
# ───────────────────────────────────────────────────────────────────
add_library(stb INTERFACE)
target_include_directories(stb INTERFACE ${EXTERNAL_DIR}/stb)

# ───────────────────────────────────────────────────────────────────
# KTX-SOFTWARE (includes basis_universal internally)
# ───────────────────────────────────────────────────────────────────
set(KTX_FEATURE_STATIC_LIBRARY ON CACHE BOOL "" FORCE)
set(KTX_FEATURE_TESTS OFF CACHE BOOL "" FORCE)
set(KTX_FEATURE_TOOLS OFF CACHE BOOL "" FORCE)
set(KTX_FEATURE_LOADTEST_APPS OFF CACHE BOOL "" FORCE)
set(KTX_FEATURE_DOC OFF CACHE BOOL "" FORCE)
add_subdirectory(KTX-Software)

# ───────────────────────────────────────────────────────────────────
# BVH (header-only)
# ───────────────────────────────────────────────────────────────────
add_library(bvh INTERFACE)
target_include_directories(bvh INTERFACE ${EXTERNAL_DIR}/bvh/src)

# ───────────────────────────────────────────────────────────────────
# ENKITS
# ───────────────────────────────────────────────────────────────────
set(ENKITS_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
add_subdirectory(enkiTS)

# ───────────────────────────────────────────────────────────────────
# PARALLEL-HASHMAP (header-only)
# ───────────────────────────────────────────────────────────────────
add_library(parallel-hashmap INTERFACE)
target_include_directories(parallel-hashmap INTERFACE ${EXTERNAL_DIR}/parallel-hashmap)

# ───────────────────────────────────────────────────────────────────
# MIO (header-only)
# ───────────────────────────────────────────────────────────────────
add_library(mio INTERFACE)
target_include_directories(mio INTERFACE ${EXTERNAL_DIR}/mio/include)

# ───────────────────────────────────────────────────────────────────
# LZ4
# ───────────────────────────────────────────────────────────────────
set(LZ4_BUILD_CLI OFF CACHE BOOL "" FORCE)
set(LZ4_BUILD_LEGACY_LZ4C OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
add_subdirectory(lz4/build/cmake)

# ───────────────────────────────────────────────────────────────────
# TRACY
# ───────────────────────────────────────────────────────────────────
if(STRATUM_ENABLE_TRACY)
    option(TRACY_ENABLE "" ON)
    option(TRACY_ON_DEMAND "" ON)
    add_subdirectory(tracy)
endif()

# ───────────────────────────────────────────────────────────────────
# PYBIND11
# ───────────────────────────────────────────────────────────────────
if(STRATUM_ENABLE_PYTHON)
    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
    add_subdirectory(pybind11)
endif()
