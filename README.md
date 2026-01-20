# Stratum

**Build 3D worlds layer by layer**
(yes its corny i know)

Stratum is a desktop application for converting OpenStreetMap data into optimized, textured 3D maps for use in video games.

It (will) support kitbashing, LOD generation, MaterialX materials, and exports to industry-standard formats.

NB: Docs and C4 Diagrams are AI-Generated / Made with LLM based tools. Trust but verify.

---

## Features

Completed ( or mostly completed ) features are checked. this is not in order.

- [x] OSM Import:  Parse `.osm` and `.pbf` files via libosmium
- [x] Mesh Generation:  Extrude buildings, generate roads, terrain, landuse
- [ ] Triangulation:  Fast polygon triangulation with earcut
- [ ] MaterialX:  Industry-standard, cross-application material definitions
- [ ] GPU Compression:  Basis Universal / KTX2 for VRAM-efficient textures
- [ ] LOD Generation:  Automatic mesh simplification via meshoptimizer
- [ ] Texture Atlasing:  Reduce draw calls with packed textures
- [ ] Debug Visualization:  BVH bounds, LOD rings, chunk boundaries
- [ ] Node Editor:  Visual material graph editing
- [ ] Python Scripting:  Automate workflows, batch processing
- [ ] Formats:  glTF, FBX, OBJ via Assimp
- [ ] Materials:  Embedded MaterialX or converted PBR
- [ ] Profiling:  Tracy integration for CPU/GPU profiling

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

# Configure
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release

# Build && run
cmake --build build -j$(nproc) && ./build/bin/stratum
```

### Build Options

Right now these dont do buch, but hopefully they will soon

NB: Python scripting will *eventually*(tm) be always on

| Option | Default | Description |
| ------ | ------- | ----------- |
| `STRATUM_ENABLE_TRACY` | `ON` | Enable Tracy profiler |
| `STRATUM_ENABLE_PYTHON` | `ON` | Enable Python scripting |
| `STRATUM_BUILD_TESTS` | `OFF` | Build test suite |
| `STRATUM_BUILD_DOCS` | `OFF` | Build Doxygen documentation |

```bash
cmake -B build -DSTRATUM_ENABLE_TRACY=OFF -DSTRATUM_ENABLE_PYTHON=ON
```

### Building Documentation

To generate the API documentation using Doxygen:

```bash
# Install Doxygen (if not already installed)
# Arch: sudo pacman -S doxygen graphviz
# Ubuntu: sudo apt install doxygen graphviz

# Configure with docs enabled
cmake -B build -DSTRATUM_BUILD_DOCS=ON

# Generate documentation
cmake --build build --target docs

# Open in browser
xdg-open docs/generated/html/index.html
```

The documentation includes:
- **API Reference**: Full class and function documentation
- **Architecture Guide**: UML diagrams in `docs/architecture.md`
- **Call Graphs**: Visual function call relationships
- **Class Diagrams**: UML class hierarchy

---

## Dependencies

There's a couple xd

| Category | Library | License |
| -------- | ------- | ------- |
| UI | [SDL3](https://github.com/libsdl-org/SDL) | zlib |
| | [Dear ImGui](https://github.com/ocornut/imgui) | MIT |
| OSM | [libosmium](https://github.com/osmcode/libosmium) | Boost |
| | [protozero](https://github.com/mapbox/protozero) | BSD-2-Clause |
| Math | [GLM](https://github.com/g-truc/glm) | MIT |
| Geometry | [Clipper2](https://github.com/AngusJohnson/Clipper2) | Boost |
| | [earcut](https://github.com/mapbox/earcut.hpp) | ISC |
| | [meshoptimizer](https://github.com/zeux/meshoptimizer) | MIT |
| | [Draco](https://github.com/google/draco) | Apache-2.0 |
| Import/Export | [Assimp](https://github.com/assimp/assimp) | BSD-3-Clause |
| Materials | [MaterialX](https://github.com/AcademySoftwareFoundation/MaterialX) | Apache-2.0 |
| Textures | [stb](https://github.com/nothings/stb) | MIT / Public Domain |
| | [KTX-Software](https://github.com/KhronosGroup/KTX-Software) | Apache-2.0 |
| Culling | [bvh](https://github.com/madmann91/bvh) | MIT |
| Threading | [enkiTS](https://github.com/dougbinks/enkiTS) | zlib |
| | [parallel-hashmap](https://github.com/greg7mdp/parallel-hashmap) | Apache-2.0 |
| Streaming | [mio](https://github.com/vimpunk/mio) | MIT |
| | [lz4](https://github.com/lz4/lz4) | BSD-2-Clause |
| Editor | [ImGuizmo](https://github.com/CedricGuillemet/ImGuizmo) | MIT |
| | [im3d](https://github.com/john-chapman/im3d) | MIT |
| | [ImGuiFileDialog](https://github.com/aiekick/ImGuiFileDialog) | MIT |
| | [imnodes](https://github.com/Nelarius/imnodes) | MIT |
| | [ImGuiColorTextEdit](https://github.com/BalazsJako/ImGuiColorTextEdit) | MIT |
| Scene | [EnTT](https://github.com/skypjack/entt) | MIT |
| Scripting | [pybind11](https://github.com/pybind/pybind11) | BSD-3-Clause |
| Profiling | [Tracy](https://github.com/wolfpld/tracy) | BSD-3-Clause |
| Utilities | [spdlog](https://github.com/gabime/spdlog) | MIT |
| | [nlohmann/json](https://github.com/nlohmann/json) | MIT |

---

## Roadmap

This is about as serious as The Onion is.

### v0.1 - Foundation

- [ ] SDL3 + ImGui + SDL_GPU window
- [ ] Basic OSM parsing
- [ ] Building extrusion
- [ ] Simple viewport navigation

### v0.2 - Core Pipeline

- [ ] Road mesh generation
- [ ] Terrain/landuse areas
- [ ] Basic texturing with albedo only
- [ ] glTF export

### v0.3 - Optimization

- [ ] LOD generation
- [ ] Frustum culling
- [ ] Chunk streaming
- [ ] Texture atlasing

### v0.4 - Editor

- [ ] Scene hierarchy panel
- [ ] Properties inspector
- [ ] Transform gizmos
- [ ] Undo/redo

### v0.5 - Materials & Kitbashing

- [ ] MaterialX integration
- [ ] Asset import (FBX, OBJ, etc.)
- [ ] Kitbash placement tools
- [ ] Procedural facade generation

### v0.6 - Scripting

- [ ] Python API
- [ ] Batch processing
- [ ] Custom export presets
- [ ] Some example scripts
- [ ] Documentation

---

## License

All Rights Reserved

---

## Acknowledgments

Built with open-source libraries from the community. See [Dependencies](#dependencies) for the full list.
