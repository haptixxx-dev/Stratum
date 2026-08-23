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
    add_executable(stratum_gpu_tests
        framework.cpp
        renderer/test_gpu_buffer_pool.cpp
        renderer/test_gpu_upload_batch.cpp
    )

    target_link_libraries(stratum_gpu_tests PRIVATE stratum_editor_lib Threads::Threads)

    target_include_directories(stratum_gpu_tests PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )

    target_compile_features(stratum_gpu_tests PRIVATE cxx_std_20)

    if(NOT MSVC)
        target_compile_options(stratum_gpu_tests PRIVATE -Wall -Wextra)
    endif()

    add_test(NAME GPUBufferPool COMMAND stratum_gpu_tests GPUBufferPool)
    set_tests_properties(GPUBufferPool PROPERTIES
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        LABELS "unit;gpu"
    )

    # The upload batch arithmetic is pure and needs no device, so it is not
    # labelled gpu and never skips.
    add_test(NAME GPUUploadBatch COMMAND stratum_gpu_tests GPUUploadBatch)
    set_tests_properties(GPUUploadBatch PROPERTIES
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        LABELS "unit"
    )
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
