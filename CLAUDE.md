# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Stratum is a C++20 desktop application that converts OpenStreetMap data into optimized, textured 3D maps for video games. It uses SDL3 + SDL_GPU (Vulkan backend) for rendering and Dear ImGui for the editor UI. Early development (v0.1.0).

## Build Commands

```bash
# Configure (first time or after CMake changes)
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build build -j$(nproc)

# Run
./build/bin/stratum

# Build + run shortcut
cmake --build build -j$(nproc) && ./build/bin/stratum
```

System prerequisites (Arch): `cmake ninja python zlib bzip2 expat`

All dependencies are vendored as git submodules in `external/`. After cloning, ensure submodules are initialized: `git submodule update --init --recursive`

## Architecture

### Two-library split

- **stratum_core** (static lib) — Engine-agnostic. OSM parsing, procedural generation, data structures, math/geometry. Must NOT depend on SDL, ImGui, or rendering code.
- **stratum_editor_lib** (static lib) — SDL3+ImGui editor. Rendering, UI panels, camera, gizmos. Depends on stratum_core.
- **stratum** (executable) — Just `main.cpp`, links stratum_editor_lib.

### Source layout (`src/`)

- `core/` — Application lifecycle, SDL3 window, input handling
- `renderer/` — SDL_GPU device, pipelines, mesh/texture GPU resources
- `editor/` — Editor UI, camera, gizmos, im3d integration
- `editor/panels/` — ImGui panels: viewport, scene hierarchy, properties, OSM import, procgen
- `osm/` — OSM file parsing (libosmium), coordinate projection (WGS84 → Mercator → local), mesh building from OSM features, spatial tile management
- `procgen/` — Noise generation (Perlin/Simplex FBM), heightmap terrain, terrain mesh building, terrain tile manager

### Key data flows

**OSM pipeline:** `.osm/.pbf` file → `OSMParser` → `ParsedOSMData` (buildings/roads/areas) → `MeshBuilder` (extrude/triangulate) → `Mesh` → GPU upload via `GPURenderer`

**Terrain pipeline:** `TerrainConfig` → `TerrainGenerator` (noise) → `Heightmap` → `TerrainMeshBuilder` → `Mesh` → GPU

### Rendering

Two shader modes switchable at runtime:
- **Simple** (`assets/shaders/mesh.vert/frag`) — Blinn-Phong
- **PBR** (`assets/shaders/mesh_pbr.vert/frag`) — Cook-Torrance BRDF, tone mapping, fog

Shader uniform sets: Set 0 = scene (camera, lighting, fog), Set 1 = per-mesh (MVP, model, color tint). Shaders are pre-compiled SPIR-V.

### Key patterns

- **EnTT ECS** for scene management
- **Tile-based streaming** for both OSM data and terrain (frustum culling per tile)
- **Handle-based GPU resources** (uint32_t IDs, not raw pointers)
- **Coordinate system chain:** WGS84 (lat/lon) → Web Mercator → local meters → Y-up world space (see `osm/coordinates.hpp`)

## Build Options

| Option | Default | Notes |
|--------|---------|-------|
| `STRATUM_ENABLE_TRACY` | ON | Tracy profiler zones |
| `STRATUM_ENABLE_PYTHON` | ON | pybind11 scripting (not yet functional) |
| `STRATUM_BUILD_TESTS` | OFF | No tests exist yet |
| `STRATUM_BUILD_DOCS` | OFF | Doxygen generation |
