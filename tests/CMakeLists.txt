# ═══════════════════════════════════════════════════════════════════
# STRATUM TESTS
# ═══════════════════════════════════════════════════════════════════
# One executable, stratum_tests, linking stratum_core only. Nothing here may
# depend on SDL, ImGui, or a rendering API -- the point of the road pipeline
# living in stratum_core is that it is testable without a GPU or a window.
#
# INTEGRATOR: wire this in from the root CMakeLists.txt with
#
#     if(STRATUM_BUILD_TESTS)
#         enable_testing()
#         add_subdirectory(tests)
#     endif()
#
# enable_testing() must be called in the TOP-LEVEL CMakeLists.txt, not here:
# ctest only discovers tests registered below a directory where it was enabled
# at or above the top level. Calling it in this file produces a CTestTestfile
# under build/tests that a plain `ctest` from build/ never reads.
#
# Then:
#     cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DSTRATUM_BUILD_TESTS=ON
#     cmake --build build -j$(nproc)
#     ctest --test-dir build --output-on-failure
#
# Or run the binary directly, optionally filtered to one suite:
#     ./build/bin/stratum_tests
#     ./build/bin/stratum_tests RoadGraph
# ═══════════════════════════════════════════════════════════════════

if(NOT STRATUM_BUILD_TESTS)
    return()
endif()

# ───────────────────────────────────────────────────────────────────
# Suites
# ───────────────────────────────────────────────────────────────────
# Each entry is a suite name (the first TEST() argument, and the argv[1] filter)
# paired with the sources that define it. Adding a suite means adding one row
# here and one row to STRATUM_TEST_SUITES below.
set(STRATUM_TEST_SOURCES
    framework.cpp
    obj_dump.cpp
    road/test_road_graph.cpp
    road/test_centerline.cpp
    road/test_road_profile.cpp
    road/test_corridor.cpp
    road/test_road_network.cpp
    road/test_road_network_elevated.cpp
    road/test_road_elevation.cpp
    road/test_road_dump.cpp
    road/test_road_terrain_dump.cpp
    road/test_junction_trim.cpp
    road/test_junction_polygon.cpp
    road/test_junction_curb.cpp
    road/test_junction_special.cpp
    road/test_junction_integration.cpp
    road/test_junction_dump.cpp
    road/test_markings.cpp
    road/test_crossings.cpp
    road/test_sidewalk_dedup.cpp
    road/test_structures.cpp
    road/test_p5_p6_integration.cpp
    road/test_p5_p6_dump.cpp

    # P7 game-ready output
    road/test_mesh_optimize.cpp
    road/test_collision_mesh.cpp
    road/test_road_export.cpp
    road/test_p7_integration.cpp
    procgen/test_terrain_carve.cpp
    osm/test_quadtree_roads.cpp
)

# Suite names registered with ctest so each can be run and reported separately.
set(STRATUM_TEST_SUITES
    RoadGraph
    Centerline
    RoadProfile
    Corridor
    RoadNetwork
    RoadNetworkElevated
    RoadElevation
    JunctionTrim
    JunctionPolygon
    JunctionCurb
    JunctionSpecial
    JunctionIntegration
    Markings
    Crossings
    SidewalkDedup
    Structures
    P5P6Integration
    MeshOptimize
    CollisionMesh
    RoadExport
    P7Integration
    TerrainCarve
    QuadTreeRoads
    RoadDump
    RoadTerrainDump
    JunctionDump
    P5P6Dump
)

add_executable(stratum_tests ${STRATUM_TEST_SOURCES})

# The TerrainCarve suite runs carve_terrain() from several std::threads at once
# to prove the documented "safe for DIFFERENT heightmaps sharing one const
# CarveInput" guarantee, so the test binary needs the platform thread library in
# its own right rather than inheriting it from stratum_core.
find_package(Threads REQUIRED)

target_link_libraries(stratum_tests PRIVATE stratum_core Threads::Threads)

# framework.hpp is included as "framework.hpp" from every suite, wherever it sits
# in the tree, so tests/ itself is on the include path.
target_include_directories(stratum_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Fixtures are read from the source tree at run time rather than copied into the
# build tree. They are small, hand-authored, and version-controlled: a copy step
# would only add a way for a stale fixture to be tested.
# STRATUM_TEST_DUMP_DIR is under the build tree, not the source tree: the OBJ
# dumps written by the RoadDump suite are build output, regenerated on every run,
# and must never be mistaken for a committed fixture.
target_compile_definitions(stratum_tests PRIVATE
    STRATUM_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data"
    STRATUM_TEST_DUMP_DIR="${CMAKE_BINARY_DIR}/road_dump"
)

target_compile_features(stratum_tests PRIVATE cxx_std_20)

if(NOT MSVC)
    target_compile_options(stratum_tests PRIVATE -Wall -Wextra)
endif()

# ───────────────────────────────────────────────────────────────────
# GPU suites
# ───────────────────────────────────────────────────────────────────
# GPUBufferPool is the one piece of P7 that touches SDL_GPU, so it cannot be
# tested from stratum_tests, which links stratum_core and nothing else by
# design. It gets a second executable linking stratum_editor_lib.
#
# The suite runs against a REAL SDL_GPU device. SDL_CreateGPUDevice() succeeds
# with no window once the video subsystem is up, so no mock and no extracted
# free-list class is needed. On a machine with no usable backend the device
# cannot be created; the suite then prints a SKIPPED line to stderr and asserts
# nothing, because failing a build for the absence of a GPU is worse than not
# running.
#
# Guarded on the implementation file existing so that a tree without
# gpu_buffer_pool.cpp still configures and still builds stratum_tests.
if(TARGET stratum_editor_lib AND EXISTS ${CMAKE_SOURCE_DIR}/src/renderer/gpu_buffer_pool.cpp)
    set(STRATUM_GPU_TEST_SOURCES
        framework.cpp
        renderer/test_gpu_buffer_pool.cpp
        renderer/test_gpu_upload_batch.cpp
    )

    # Suites needing no SDL_GPUDevice. GPUBufferPool is the only one that does,
    # and it skips itself when no backend is available; everything listed here
    # runs unconditionally, so none of them is labelled "gpu".
    set(STRATUM_GPU_TEST_SUITES
        GPUUploadBatch
    )

    # ── Material system ────────────────────────────────────────────────────
    # MaterialUniforms is a header-only struct, so its layout suite compiles and
    # links with nothing more than material_library.hpp. It is the cheapest
    # possible guard against the one failure that is neither a compile error nor
    # a validation error: the C++ block and the GLSL block drifting apart.
    if(EXISTS ${CMAKE_SOURCE_DIR}/src/renderer/material_library.hpp)
        list(APPEND STRATUM_GPU_TEST_SOURCES renderer/test_material_uniforms.cpp)
        list(APPEND STRATUM_GPU_TEST_SUITES MaterialUniforms)
    endif()

    # The MaterialLibrary and ProceduralTexture suites need their implementation
    # translation units linked in. Guarded on the sources existing so a tree
    # part-way through this phase still configures and still builds everything
    # else, exactly as the gpu_buffer_pool.cpp guard above does.
    #
    # Both suites are DEVICE-INDEPENDENT despite living in this executable:
    # material_library.hpp and procedural_texture.hpp include SDL through
    # texture.hpp, which keeps them out of stratum_tests, but neither suite
    # creates a device, a window, or an SDL subsystem.
    if(EXISTS ${CMAKE_SOURCE_DIR}/src/renderer/material_library.cpp)
        list(APPEND STRATUM_GPU_TEST_SOURCES renderer/test_material_library.cpp)
        list(APPEND STRATUM_GPU_TEST_SUITES MaterialLibrary)
    endif()

    if(EXISTS ${CMAKE_SOURCE_DIR}/src/renderer/procedural_texture.cpp)
        list(APPEND STRATUM_GPU_TEST_SOURCES renderer/test_procedural_texture.cpp)
        list(APPEND STRATUM_GPU_TEST_SUITES ProceduralTexture)
    endif()

    # MaterialBinding is pure: three small functions lifted out of GPURenderer's
    # draw path -- which bind_material() mode a renderer state demands, the
    # reverse-Z decal depth-bias factors, and the decal pipeline cache key. Each
    # guards a defect that is a crash or an inverted sign rather than a wrong
    # pixel, and none of them needs a device.
    list(APPEND STRATUM_GPU_TEST_SOURCES renderer/test_material_binding.cpp)
    list(APPEND STRATUM_GPU_TEST_SUITES MaterialBinding)

    # TextureUpload and PbrShader DO need a real SDL_GPU device, and both skip
    # themselves with a line on stderr when there is no backend, exactly as
    # GPUBufferPool does. They are labelled "gpu" below for that reason.
    #
    # TextureUpload covers what only a real upload shows: readiness being
    # contingent on the submit, the built-in fallback texels, and what
    # install_procedural_textures() does to base_color.
    #
    # PbrShader renders one pixel through the checked-in mesh_pbr .spv with the
    # renderer's own vertex layout, uniform slots and sampler set. It is the only
    # place a NaN tangent frame or a metallic scalar multiplied by a zero ORM
    # channel is observable at all.
    if(EXISTS ${CMAKE_SOURCE_DIR}/src/renderer/texture.cpp)
        list(APPEND STRATUM_GPU_TEST_SOURCES renderer/test_texture_upload.cpp)
        list(APPEND STRATUM_GPU_DEVICE_SUITES TextureUpload)
    endif()

    list(APPEND STRATUM_GPU_TEST_SOURCES renderer/test_pbr_shader.cpp)
    list(APPEND STRATUM_GPU_DEVICE_SUITES PbrShader)

    add_executable(stratum_gpu_tests ${STRATUM_GPU_TEST_SOURCES})

    target_link_libraries(stratum_gpu_tests PRIVATE stratum_editor_lib Threads::Threads)

    target_include_directories(stratum_gpu_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

    # STRATUM_TEST_SHADER_DIR points at the SOURCE tree because that is where the
    # runtime loads shaders from: the .spv are checked in and resolved relative to
    # SDL_GetBasePath(). A copy under the build tree would let the suite pass
    # against a stale duplicate of the file the application never reads.
    #
    # STRATUM_TEST_TMP_DIR is under the BUILD tree, for the same reason
    # STRATUM_TEST_DUMP_DIR is: material set files written by the round-trip test
    # are throwaway output and must never be mistaken for a committed fixture.
    target_compile_definitions(stratum_gpu_tests PRIVATE
        STRATUM_TEST_SHADER_DIR="${CMAKE_SOURCE_DIR}/assets/shaders"
        STRATUM_TEST_TMP_DIR="${CMAKE_BINARY_DIR}/material_test"
    )

    target_compile_features(stratum_gpu_tests PRIVATE cxx_std_20)

    if(NOT MSVC)
        target_compile_options(stratum_gpu_tests PRIVATE -Wall -Wextra)
    endif()

    # Suites that need an SDL_GPUDevice and skip themselves without one.
    list(APPEND STRATUM_GPU_DEVICE_SUITES GPUBufferPool)

    foreach(suite IN LISTS STRATUM_GPU_DEVICE_SUITES)
        add_test(NAME ${suite} COMMAND stratum_gpu_tests ${suite})
        set_tests_properties(${suite} PROPERTIES
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            LABELS "unit;gpu"
        )
    endforeach()

    foreach(suite IN LISTS STRATUM_GPU_TEST_SUITES)
        add_test(NAME ${suite} COMMAND stratum_gpu_tests ${suite})
        set_tests_properties(${suite} PROPERTIES
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            LABELS "unit"
        )
    endforeach()
endif()

# ───────────────────────────────────────────────────────────────────
# ctest registration
# ───────────────────────────────────────────────────────────────────
# One ctest entry per suite, each invoking the binary with the suite filter, so a
# failure names the suite. The runner exits non-zero when a filter matches no
# test, which catches a suite renamed here but not in the source.
foreach(suite IN LISTS STRATUM_TEST_SUITES)
    add_test(NAME ${suite} COMMAND stratum_tests ${suite})
    set_tests_properties(${suite} PROPERTIES
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        LABELS "unit"
    )
endforeach()
