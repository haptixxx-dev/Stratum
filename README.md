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

# Configure, build, run
cmake --preset release
cmake --build --preset release
./build/release/bin/stratum
```

That is the whole thing. `CMakePresets.json` carries the generator, the build
type, the compiler and the output directory, so there are no flags to remember
and no way to end up with a build directory whose settings you have forgotten.

### CMake Presets

Presets need **CMake 3.25 or newer**. The project itself builds with 3.24 — if
you are on exactly that, configure by hand as described under
[Configuring without presets](#configuring-without-presets).

```bash
cmake --list-presets              # what is available
cmake --preset <name>             # configure
cmake --build --preset <name>     # build
ctest --preset <name>             # test, where the preset has tests
```

Each preset builds into its own directory, `build/<preset-name>`, so a GCC tree
and a Clang tree coexist without fighting over the same cache.

| Preset | Build type | Toolchain | What it is for |
| ------ | ---------- | --------- | -------------- |
| `release` | Release | GCC | The default. Use this unless you have a reason not to. |
| `debug` | Debug | GCC | Symbols, no optimisation. A city-sized extract is genuinely painful here. |
| `relwithdebinfo` | RelWithDebInfo | GCC | Profiling, or a bug that only reproduces at speed. |
| `clang-release` | Release | Clang + LLD | Same output, ~22% faster to build clean. |
| `clang-debug` | Debug | Clang + LLD | |
| `tests` | Release | GCC | Adds `stratum_tests` and `stratum_gpu_tests`. |
| `clang-tests` | Release | Clang + LLD | |
| `ci` | Release | GCC | What CI runs: tests, with Tracy and Python off. |
| `docs` | Release | GCC | Adds the `doxygen` target. |

#### Running the tests

```bash
cmake --workflow --preset tests
```

One command: configure, build, then run the suite. Or drive the three steps
yourself if you only want to re-run the last one:

```bash
cmake --preset tests
cmake --build --preset tests
ctest --preset tests                      # everything
ctest --preset tests -R RoadGraph         # one suite
./build/tests/bin/stratum_tests RoadGraph # or straight to the binary
```

`stratum_tests` links `stratum_core` only — no SDL, no window, no device — so it
runs anywhere. `stratum_gpu_tests` links the editor library; most of its suites
are pure logic and run headless, and the few that genuinely need an
`SDL_GPUDevice` skip themselves when there is not one.

The `ci` preset turns off Tracy and Python deliberately. Neither is exercised by
the suite, both are among the heaviest optional dependencies, and pybind11 wants
Python development headers that a runner may not have.

#### ccache

The single biggest build-time win, and bigger than the compiler choice: roughly
845 of 885 objects are vendored dependencies that never change but get rebuilt
from scratch every time a build directory is wiped.

It is opt-in rather than baked into the presets, because a preset that hardcoded
it would fail to configure on any machine without it installed:

```bash
sudo pacman -S ccache            # or apt install ccache

export STRATUM_LAUNCHER=ccache
cmake --preset release           # picks it up from the environment
```

Leave `STRATUM_LAUNCHER` unset and the presets simply use no launcher.

#### Local overrides

Anything machine-specific goes in `CMakeUserPresets.json` beside this file. Git
ignores it, so it will not follow you into a commit:

```json
{
  "version": 6,
  "configurePresets": [
    {
      "name": "my-release",
      "inherits": "release",
      "cacheVariables": { "STRATUM_ENABLE_TRACY": "OFF" }
    }
  ]
}
```

### Configuring without presets

Presets are a convenience, not a requirement. The equivalent of `--preset tests`
spelled out:

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DSTRATUM_BUILD_TESTS=ON
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
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
| `STRATUM_USE_LLD` | `OFF` | Link with LLD instead of the default linker |

Presets set these for you. To override one for a single configure, pass it
alongside the preset:

```bash
cmake --preset release -DSTRATUM_ENABLE_TRACY=OFF
```

If you find yourself passing the same override every time, put it in a
`CMakeUserPresets.json` that inherits the preset instead.

### Building with Clang

Clang is meaningfully faster than GCC here. Measured on a 16-thread machine,
clean builds from an empty build directory:

| Toolchain | Clean build | Binary |
| --------- | ----------- | ------ |
| GCC 16.1.1 + default ld | 144s | 8.5 MB |
| Clang 22.1.8 + LLD | 112s | 7.5 MB |

The compiler has to be chosen before `project()` runs, so it cannot be a
`STRATUM_*` option. The `clang-*` presets set it, along with LLD, and build into
their own directory so the two toolchains never share a cache:

```bash
cmake --preset clang-release
cmake --build --preset clang-release
```

Notes:

- The ~22% saving is almost entirely Clang's front end, not the linker. This
  build has few link edges and most are `ar` archive creation, so LLD is worth
  turning on only because it is free — expect milliseconds, not seconds.
  `STRATUM_USE_LLD` works with GCC too.
- **mold is not worth installing for this project.** Its advantage over LLD is
  in link time, which is not where the time goes here.
- The single biggest win is **ccache**, not the compiler. See
  [ccache](#ccache) above; it is a one-line environment variable with the presets.
- Keep a GCC build dir around. GCC 16 caught two real problems in vendored
  dependencies that Clang did not, and CI builds with GCC.

### Building Documentation

To generate the API documentation using Doxygen:

```bash
# Install Doxygen (if not already installed)
# Arch: sudo pacman -S doxygen graphviz
# Ubuntu: sudo apt install doxygen graphviz

# Configure and generate
cmake --preset docs
cmake --build --preset docs

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
