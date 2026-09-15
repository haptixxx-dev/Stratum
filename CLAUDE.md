# CLAUDE.md

Guidance for Claude Code (claude.ai/code) when working in this repository.

**Start here, then go deeper.** This file is the index. The detailed
subsystem guides live in `docs/agents/`:

| Guide | Covers | State |
|---|---|---|
| `docs/agents/build-test-run.md` | presets, build options, the test suite, submodules, Doxygen | complete |
| `docs/agents/core-and-osm.md` | `src/core`, `src/osm` and the coordinate chain | `src/osm/road`, `src/geometry`, `src/procgen` still to be written |
| `docs/agents/renderer-and-editor.md` | `src/renderer`, GPU resources, lighting v2 | shaders, `src/editor` and `src/editor/panels` still to be written |

Where a guide and this file disagree, the guide wins — it was written against
the code more recently.

## Project overview

Stratum is a C++20 desktop application that converts OpenStreetMap data into
optimized, textured 3D maps for video games. SDL3 + SDL_GPU (Vulkan backend)
for rendering, Dear ImGui for the editor UI, EnTT for scene management.
`VERSION` is `0.1.0`.

## Build

The project uses CMake presets (`CMakePresets.json`, schema 6, CMake >= 3.25).
Build trees go to `build/<presetName>`.

```bash
git submodule update --init --recursive   # all deps are vendored submodules

cmake --preset release
cmake --build --preset release
./build/release/bin/stratum
```

Nine configure presets exist: `release`, `debug`, `relwithdebinfo`,
`clang-release`, `clang-debug`, `tests`, `clang-tests`, `ci`, `docs`. Confirm
with `cmake --list-presets`. System prerequisites on Arch: `cmake ninja python
zlib bzip2 expat`.

See `docs/agents/build-test-run.md` for what each preset is for.

## Tests

**Tests exist.** `tests/` holds 48 `.cpp` files across `core`, `geometry`,
`osm`, `procgen`, `renderer` and `road`, producing two executables:
`stratum_tests` (core-only, `tests/CMakeLists.txt:160`) and `stratum_gpu_tests`
(`:299`, with self-skipping GPU-labelled suites).

```bash
cmake --preset tests && cmake --build --preset tests && ctest --preset tests
```

They are off by default because `STRATUM_BUILD_TESTS` defaults `OFF`, not
because they are absent.

## Architecture

### Two-library split

- **stratum_core** (static lib, `CMakeLists.txt:70`) — engine-agnostic. Contains
  `src/osm`, `src/osm/road`, `src/geometry`, `src/procgen` only. Links `glm`,
  `osmium`, `Clipper2`, `meshoptimizer`. Must NOT depend on SDL, ImGui or
  rendering code.
- **stratum_editor_lib** (static lib, `CMakeLists.txt:172`) — the SDL3 + ImGui
  editor: rendering, UI panels, camera, gizmos. Depends on stratum_core.
- **stratum** (executable) — just `main.cpp`, links stratum_editor_lib.

> **Trap:** `src/core` is NOT part of `stratum_core`. Its two classes
> (`Application`, `Window`) compile into `stratum_editor_lib` and include SDL
> and renderer headers. Engine-agnostic code belongs in `src/osm`,
> `src/geometry` or `src/procgen`.

### Source layout (`src/`)

- `core/` — `Application` (main loop) and `Window` (SDL3 wrapper). Neither
  destructor calls `shutdown()`; callers must.
- `renderer/` — SDL_GPU device, pipelines, mesh/texture GPU resources.
- `editor/`, `editor/panels/` — editor UI, camera, gizmos, im3d; ImGui panels
  for viewport, scene hierarchy, properties, OSM import, procgen.
- `osm/` — OSM parsing (libosmium), coordinate projection, mesh building,
  quadtree spatial index and streaming.
- `osm/road/` — road network topology and geometry, solved network-wide.
- `geometry/` — shared geometry utilities, including the ambient-occlusion baker.
- `procgen/` — noise, heightmap terrain, terrain mesh building, tile management.

### Key data flows

**OSM:** `.osm`/`.pbf` → `OSMParser` → `ParsedOSMData` → `MeshBuilder`
(buildings and areas) and `road::RoadNetworkBuilder` (roads, topology-driven)
→ `QuadTree` leaves → GPU upload via `GPURenderer`.

**Terrain:** `TerrainConfig` → `TerrainGenerator` (noise) → `Heightmap` →
`TerrainMeshBuilder` → `Mesh` → GPU.

### Coordinates

WGS84 (lat/lon) → Web Mercator → local metres → Y-up world space. See
`osm/coordinates.hpp` and the coordinate section of
`docs/agents/core-and-osm.md`, which documents the traps: `BoundingBox::center()`
returns `(lat, lon)` in a `dvec2`, and `wgs84_to_local()` silently returns raw
Mercator if `set_origin()` was never called.

### Rendering

Two shader modes switchable at runtime: **Simple** (`assets/shaders/mesh.vert`
/ `.frag`, Blinn-Phong) and **PBR** (`mesh_pbr.vert` / `.frag`, Cook-Torrance,
tone mapping, fog). Uniform sets: set 0 = scene (camera, lighting, fog), set 1 =
per-mesh (MVP, model, colour tint). Shaders are pre-compiled SPIR-V.

Lighting v2 (analytic sky, cascaded shadow maps, baked ambient occlusion) is
documented in `docs/agents/renderer-and-editor.md`. Read that section before
touching lighting, shadows or `mesh_pbr.frag` — it lists five regressions that
were fixed there and should not be reintroduced.

### Key patterns

- **EnTT ECS** for scene management.
- **Tile-based streaming** for OSM data and terrain, with frustum culling.
- **Handle-based GPU resources** (`uint32_t` IDs, not raw pointers).
- Junctions are found by shared OSM node **identity**, never endpoint proximity.

## Build options

Declared at `CMakeLists.txt:18-22`.

| Option | Default | Notes |
|--------|---------|-------|
| `STRATUM_ENABLE_TRACY` | ON | Tracy profiler zones |
| `STRATUM_ENABLE_PYTHON` | ON | pybind11 scripting (not yet functional) |
| `STRATUM_BUILD_TESTS` | OFF | Builds `stratum_tests` and `stratum_gpu_tests` |
| `STRATUM_BUILD_DOCS` | OFF | Doxygen generation; needs `doxygen` + `graphviz` |
| `STRATUM_USE_LLD` | OFF | Link with LLD instead of the default linker |

Set `STRATUM_LAUNCHER` in the environment (e.g. to `ccache`) to use a compiler
launcher.
