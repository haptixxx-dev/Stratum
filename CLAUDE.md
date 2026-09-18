# CLAUDE.md

Guidance for Claude Code (claude.ai/code) when working in this repository.

**Start here, then go deeper.** This file is the index. The detailed
subsystem guides live in `docs/agents/`:

| Guide | Covers | State |
|---|---|---|
| `docs/agents/build-test-run.md` | presets, build options, the test suite, submodules, Doxygen | complete |
| `docs/agents/core-and-osm.md` | `src/core`, `src/osm` and the coordinate chain | `src/osm/road` and `src/geometry` still to be written |
| `docs/agents/renderer-and-editor.md` | `src/renderer`, GPU resources, lighting v2 | shaders, `src/editor` and `src/editor/panels` still to be written |
| `docs/agents/rules-and-procgen.md` | `src/procgen`, the rule engine, the operation catalogue, determinism | complete |

Where a guide and this file disagree, the guide wins — it was written against
the code more recently.

## Project overview

Stratum is a C++20 desktop application that converts OpenStreetMap data into
optimized, textured 3D maps for video games. SDL3 + SDL_GPU (Vulkan backend)
for rendering, Dear ImGui for the editor UI, and a handle-based scene model in
`src/scene` (not an ECS — see Key patterns).
`VERSION` is `0.3.0`; it is read from the `VERSION` file at configure time, and
the release workflow refuses a tag that disagrees with it.

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

**Tests exist.** `tests/` holds 63 `.cpp` files across `core`, `geometry`,
`osm`, `procgen`, `renderer`, `road` and `scene`, producing two executables:
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
  `src/osm`, `src/osm/road`, `src/geometry`, `src/procgen`, `src/scene` only.
  Links `glm`, `osmium`, `Clipper2`, `meshoptimizer`, `draco`, `stb`. Must NOT
  depend on SDL, ImGui or rendering code. `stb` is there deliberately — decision
  Q2 (#13) allows core to read and write pixel data, so heightmap import and,
  later, facade texturing and atlas packing work with no window open.
- **stratum_editor_lib** (static lib, `CMakeLists.txt:172`) — the SDL3 + ImGui
  editor: rendering, UI panels, camera, gizmos. Depends on stratum_core.
- **stratum** (executable) — just `main.cpp`, links stratum_editor_lib.

> **Trap:** `src/core` is NOT part of `stratum_core`. Its two classes
> (`Application`, `Window`) compile into `stratum_editor_lib` and include SDL
> and renderer headers. Engine-agnostic code belongs in `src/osm`,
> `src/geometry`, `src/procgen` or `src/scene`. The one whose name says
> "core" is the one that is not.

### Source layout (`src/`)

- `core/` — `Application` (main loop) and `Window` (SDL3 wrapper). Neither
  destructor calls `shutdown()`; callers must.
- `renderer/` — SDL_GPU device, pipelines, mesh/texture GPU resources.
- `editor/`, `editor/panels/` — editor UI, camera, gizmos, im3d; ImGui panels
  for viewport, scene hierarchy, properties, OSM import, procgen.
- `osm/` — OSM parsing (libosmium), coordinate projection, mesh building,
  quadtree spatial index and streaming.
- `osm/road/` — road network topology and geometry, solved network-wide. Also
  `blocks.*` (the planar faces the streets enclose — the input to lot
  subdivision) and `graph_edit.*` (mutating the graph through the command stack).
- `geometry/` — shared geometry utilities, including the ambient-occlusion baker.
- `procgen/` — noise, heightmap terrain, terrain mesh building, tile management,
  and heightmap import from PNG and PGM. `procgen/rules/` is the shape-grammar
  engine — lexer, parser, interpreter, and the split/component/control
  operations. It is the largest thing in `stratum_core`; read
  `docs/agents/rules-and-procgen.md` before touching it.
- `scene/` — the scene model, complete as of M2: the undo/redo command stack,
  the layer tree, typed attributes with layer inheritance, selection,
  georeferencing, and save/load. **Every mutation of the scene goes through
  `CommandStack::execute()`.** A mutation that skips it is a hole in the history,
  and the symptom shows up as an undo several steps later restoring state that
  was never current.

  That rule is enforced structurally rather than by convention: `LayerTree`,
  `AttributeStore` and `MapLayer` all keep their mutating API private with their
  command classes as the only friends. Follow that pattern in anything new here.

  One subtlety worth knowing before touching `selection.cpp`: a const member
  function may FORGET a dead member where it lies, but may never move a survivor
  or change the length. Iterators index the storage directly, so moving one
  invalidates them. `rebuild()`, `trim_tail()` and `compact_if_sparse()` are
  non-const precisely so a const accessor cannot call them.

### Key data flows

**Export:** meshes → `osm/scene_export.*` → chunked OBJ or glTF. Generalised from
`osm/road/road_export.*`, and it keeps that file's invariant: every triangle
lands in exactly one chunk, chosen by centroid, and no triangle is split at a
boundary. Sum the chunk counts and you get the input count back.

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

- **Handle-based scene model, NOT an ECS.** EnTT is linked into `stratum_core`
  and used by nothing in `src/`. The scene uses plain owning structures with
  stable integer handles: `LayerId` is never reused, and `AttributeObject`
  carries a generation so a recycled slot cannot alias. Do not introduce EnTT
  into `src/scene` without a system that actually iterates components.
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
