# Restructure phase 0: baseline checks

**Date:** 2026-09-27
**Branch:** `restructure/phase-0-baseline`
**Machine:** local desktop, GCC (`release` and `tests` presets), Vulkan backend.

This records the state the phase 0 golden harness (`tools/golden/stratum_golden.cpp`,
`tests/golden/*.json`) was verified against, and three checks the later restructure
phases can diff their own runs against.

## Test suite

```
cmake --preset tests && cmake --build --preset tests && ctest --preset tests
```

Summary line:

```
100% tests passed out of 100
```

Label breakdown from the same run: `unit` 84 tests, `gpu` 4 tests (double-counted
into `unit`; see `tests/CMakeLists.txt`'s `STRATUM_GPU_DEVICE_SUITES` loop),
`golden` 16 tests — the fixtures added in this phase, one per `tests/data/*.osm`.

## Editor source size

```
wc -l src/editor/*.cpp src/editor/panels/*.cpp
```

```
   155 src/editor/camera.cpp
  4176 src/editor/editor.cpp
   487 src/editor/im3d_impl.cpp
   843 src/editor/panels/material_panel.cpp
   454 src/editor/panels/procgen_panel.cpp
   580 src/editor/panels/rule_panel.cpp
  6695 total
```

## Vulkan validation

```
cmake --preset release && cmake --build --preset release
timeout 8 ./build/release/bin/stratum 2>&1 | grep -c VUID
```

Result: `0`. The app initialised SDL, created the Vulkan GPU device, loaded the
Simple, PBR, sky and shadow pipelines, and ran for the full 8-second window with
no `VUID` line in its log. The only warnings on this run are pre-existing and
unrelated: `GPUTextureManager::shutdown` reports 27 live textures released at
shutdown, which the log itself already flags as a caller not returning a handle.

## Golden harness

`tools/golden/stratum_golden.cpp` runs the same core stages
`Editor::poll_osm_import()` runs before anything touches the GPU: `OSMParser`
with the editor's default `ParserConfig`, `road::RoadNetworkBuilder::build()`
with the editor's default `RoadNetworkConfig` (via `make_road_network_config()`
field-for-field, height sampler left null since this tool generates no terrain),
`QuadTree::init/assign_data/assign_road_pieces` with chunk LOD on and a default
`ChunkLodConfig`, and the per-leaf building/area mesh build. It prints one
deterministic JSON object — no timestamps, no pointers — to stdout.

`tests/golden/<fixture>.json` holds one snapshot per `tests/data/*.osm` fixture,
diffed by the `Golden_<fixture>` ctest (label `golden`) via
`tests/golden/compare_golden.cmake`. All 16 pass against this baseline.

`tests/golden/lucan.json` additionally records a run against
`~/Downloads/lucan.osm` (65 MB, not in the repository — the JSON records only
its sha256 and the pipeline's counts). It is not wired into ctest, since a CI
checkout has no copy of the file to run against. Selected numbers, quoted here
because they cross-check against an existing comment in
`src/osm/road/junction_builder.hpp` describing this same extract:

| Stat | Value |
|---|---|
| `quadtree.leaf_count` | 1516 |
| `quadtree.total_roads` | 10326 |
| `quadtree.total_buildings` | 24576 |
| `quadtree.total_areas` | 2122 |
| `quadtree.max_depth` | 8 |
| `junction_stats.junctions` | 7081 |
| `junction_stats.merged_into_neighbour` | 512 |
| `junction_stats.degenerate` | 0 |
| `junction_stats.self_intersecting` | 20 |
| `junction_stats.over_trimmed_edges` | 7069 |

`merged_into_neighbour` = 512 and `degenerate` = 0 against this same Lucan
extract is exactly the pair `junction_builder.hpp`'s `Stats::merged_into_neighbour`
doc comment cites for the width-scaled merge, which is independent evidence
the harness is exercising the real solve path rather than a stub.
