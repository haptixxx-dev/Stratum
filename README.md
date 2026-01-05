# Stratum

**OSM to 3D, layer by layer.**

Stratum is a desktop application for converting OpenStreetMap data into optimized, textured 3D meshes. It supports kitbashing, LOD generation, MaterialX materials, and exports to industry-standard formats.

Built for city-scale scenes that run on mid-range hardware.

---

## Features

### Core Pipeline

- **OSM Import** — Parse `.osm` and `.pbf` files via libosmium
- **Mesh Generation** — Extrude buildings, generate roads, terrain, landuse
- **Triangulation** — Fast polygon triangulation with earcut
- **Boolean Operations** — Clipper2 for polygon unions, offsets, holes

### Materials & Textures

- **MaterialX** — Industry-standard, cross-application material definitions
- **GPU Compression** — Basis Universal / KTX2 for VRAM-efficient textures
- **Procedural Options** — Generate facades, roofs, roads programmatically

### Optimization

- **LOD Generation** — Automatic mesh simplification via meshoptimizer
- **Mesh Compression** — Draco for compact storage/streaming
- **Spatial Acceleration** — BVH for fast culling and queries
- **Texture Atlasing** — Reduce draw calls with packed textures

### Editor

- **ImGui Interface** — Fast, immediate-mode UI
- **Transform Gizmos** — Translate/rotate/scale with ImGuizmo
- **Debug Visualization** — BVH bounds, LOD rings, chunk boundaries
- **Node Editor** — Visual material graph editing
- **Python Scripting** — Automate workflows, batch processing

### Export

- **Formats** — glTF, FBX, OBJ via Assimp
- **Materials** — Embedded MaterialX or converted PBR

### Performance

- **SDL3 GPU API** — Modern Vulkan/Metal/D3D12 abstraction
- **Multi-threaded** — Parallel mesh generation with enkiTS
- **Streaming** — Chunked loading with memory-mapped I/O
- **Profiling** — Tracy integration for CPU/GPU profiling

---

## Building

### Prerequisites

**Arch Linux:**

```bash
sudo pacman -S cmake ninja python zlib bzip2 expat
```

**Ubuntu/Debian:**

```bash
sudo apt install cmake ninja-build python3-dev zlib1g-dev libbz2-dev libexpat1-dev
```

**macOS:**

```bash
brew install cmake ninja python zlib bzip2 expat
```

### Clone & Build

```bash
# Clone with submodules
git clone --recursive https://github.com/yourusername/stratum.git
cd stratum

# Or if already cloned
git submodule update --init --recursive

# Configure
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build build -j$(nproc)

# Run
./build/bin/stratum
```

### Build Options

| Option | Default | Description |
| ------ | ------- | ----------- |
| `STRATUM_ENABLE_TRACY` | `ON` | Enable Tracy profiler |
| `STRATUM_ENABLE_PYTHON` | `ON` | Enable Python scripting |
| `STRATUM_BUILD_TESTS` | `OFF` | Build test suite |

```bash
cmake -B build -DSTRATUM_ENABLE_TRACY=OFF -DSTRATUM_ENABLE_PYTHON=ON
```

---

## Project Structure

```text
stratum/
├── CMakeLists.txt
├── external/              # Git submodules (29 dependencies)
├── src/
│   ├── core/              # Application, windowing, input
│   ├── renderer/          # SDL_GPU abstraction, pipelines
│   ├── scene/             # ECS (entt), spatial structures
│   ├── osm/               # Parsing, mesh generation, tiles
│   ├── materials/         # MaterialX integration
│   ├── editor/            # UI panels, gizmos, tools
│   ├── scripting/         # Python bindings
│   └── main.cpp
├── scripts/               # Python automation scripts
├── assets/                # Kitbash models, textures
├── data/                  # OSM files, project files
└── docs/                  # Documentation
```

---

## Dependencies

| Category | Libraries |
| -------- | --------- |
| Core | SDL3, Dear ImGui |
| OSM | libosmium, protozero |
| Math | GLM |
| Geometry | Clipper2, earcut, meshoptimizer, Draco |
| Import/Export | Assimp |
| Materials | MaterialX |
| Textures | stb, Basis Universal, KTX-Software |
| Culling | bvh |
| Threading | enkiTS, parallel-hashmap |
| Streaming | mio, lz4 |
| Editor | ImGuizmo, im3d, ImGuiFileDialog, imgui-node-editor |
| Scene | EnTT |
| Scripting | pybind11 |
| Profiling | Tracy |
| Utilities | spdlog, nlohmann/json |

---

## Roadmap

### v0.1 — Foundation

- [ ] SDL3 + ImGui + SDL_GPU window
- [ ] Basic OSM parsing
- [ ] Building extrusion
- [ ] Simple viewport navigation

### v0.2 — Core Pipeline

- [ ] Road mesh generation
- [ ] Terrain/landuse areas
- [ ] Basic texturing
- [ ] glTF export

### v0.3 — Optimization

- [ ] LOD generation
- [ ] Frustum culling
- [ ] Chunk streaming
- [ ] Texture atlasing

### v0.4 — Editor

- [ ] Scene hierarchy panel
- [ ] Properties inspector
- [ ] Transform gizmos
- [ ] Undo/redo

### v0.5 — Materials & Kitbashing

- [ ] MaterialX integration
- [ ] Asset import (FBX, OBJ)
- [ ] Kitbash placement tools
- [ ] Procedural facade generation

### v0.6 — Scripting & Polish

- [ ] Python API
- [ ] Batch processing
- [ ] Custom export presets
- [ ] Documentation

---

## License

All Rights Reserved

---

## Acknowledgments

Built with open-source libraries from the community. See [Dependencies](#dependencies) for the full list.
