# Build, test and run Stratum

This is the build playbook for agents who have never built this repo. Everything below is traced to `CMakePresets.json`, `CMakeLists.txt`, `tests/CMakeLists.txt`, `.gitmodules`, or a named file — not to memory.

**First-time bootstrap** (all dependencies are vendored git submodules):

```bash
git submodule update --init --recursive
```

(`.gitmodules` lists each dependency twice, under `external/` and a stale `submodules/external/` prefix; the recursive init handles both. The build uses `external/`.)

System prerequisites on Arch: `cmake ninja python zlib bzip2 expat`. The presets file is schema version 6 and requires CMake >= 3.25 (the raw `CMakeLists.txt` floor is 3.24, but you should be using presets).

## Presets

Defined in `CMakePresets.json`. All presets use the Ninja generator, put the build tree at `build/<presetName>`, and export `compile_commands.json`. If the env var `STRATUM_LAUNCHER` is set (e.g. to `ccache`), it is used as the compiler launcher.

Confirmed with `cmake --list-presets`:

| Configure preset | What it is |
|---|---|
| `release` | Default build. GCC, Release. CI uses GCC. |
| `debug` | GCC, unoptimised with symbols. Large, slow binary; a city extract is painful here. |
| `relwithdebinfo` | GCC, optimised with symbols. For profiling, or bugs that only reproduce at speed. |
| `clang-release` | Clang + LLD, Release. ~22% faster clean build than GCC (the front end, not the linker). |
| `clang-debug` | Clang + LLD, Debug. |
| `tests` | GCC Release + `STRATUM_BUILD_TESTS=ON`. Builds `stratum_tests` and `stratum_gpu_tests`. |
| `clang-tests` | Same with Clang + LLD. |
| `ci` | `tests` plus `STRATUM_ENABLE_TRACY=OFF` and `STRATUM_ENABLE_PYTHON=OFF` (neither is exercised by the suite; pybind11 needs Python dev headers a runner may lack). |
| `docs` | GCC Release + `STRATUM_BUILD_DOCS=ON`. Adds the `docs` Doxygen target; needs `doxygen` and `graphviz` on PATH. |

Every configure preset has a matching build preset. There are test presets for `tests`, `clang-tests`, and `ci` (output on failure; zero discovered tests is an error), and two workflow presets, `tests` and `ci`, that chain configure + build + test.

```bash
# Standard developer build and run
cmake --preset release
cmake --build --preset release
./build/release/bin/stratum

# Full test cycle in one command (configure + build + ctest)
cmake --workflow --preset tests

# What CI runs
cmake --workflow --preset ci

# Build the Doxygen docs (the docs build preset targets `docs` directly)
cmake --preset docs && cmake --build --preset docs
```

## Build options

There are **five** `STRATUM_*` options, declared at `CMakeLists.txt:18-22`. (The root `CLAUDE.md` table lists only four — it omits `STRATUM_USE_LLD`.)

| Option | Default | Gates |
|---|---|---|
| `STRATUM_ENABLE_TRACY` | ON | Tracy profiler zones. |
| `STRATUM_ENABLE_PYTHON` | ON | pybind11 scripting (not yet functional). Needs Python dev headers. |
| `STRATUM_BUILD_TESTS` | OFF | `enable_testing()` + `add_subdirectory(tests)` at `CMakeLists.txt:346-349`. The `tests`, `clang-tests`, and `ci` presets turn it on. |
| `STRATUM_BUILD_DOCS` | OFF | The Doxygen `docs` custom target at `CMakeLists.txt:365-381`. The `docs` preset turns it on. |
| `STRATUM_USE_LLD` | OFF | Links with LLD via `check_linker_flag` + `add_link_options(-fuse-ld=lld)`. Deliberately not `CMAKE_LINKER_TYPE` (that is CMake 3.29+ and silently ignored below it). The `clang-*` presets turn it on. The compiler itself cannot be an option — it must be known before `project()`, so it is set per-preset via `CMAKE_CXX_COMPILER`. |

**Correction to the record:** the root `CLAUDE.md` claims "`STRATUM_BUILD_TESTS` OFF: No tests exist yet". That is false — see the next section. The option defaults OFF so plain builds stay fast, not because there is nothing to build.

`enable_testing()` must stay in the top-level `CMakeLists.txt`: ctest only discovers tests registered below a directory where testing was enabled at or above the top level. Calling it inside `tests/` produces a `CTestTestfile` a plain `ctest` never reads.

## Tests

Tests exist and are substantial: **48 `.cpp` files** under `tests/` (`road/`, `osm/`, `procgen/`, `geometry/`, `renderer/`, plus the framework at the root). Any document claiming "no tests exist yet" is stale.

**Two test executables**, both defined in `tests/CMakeLists.txt`:

- `stratum_tests` — links `stratum_core` **only**. Nothing in it may depend on SDL, ImGui, or a rendering API; the road pipeline lives in `stratum_core` precisely so it is testable without a GPU or a window. Covers road, osm, procgen, and geometry suites.
- `stratum_gpu_tests` — links `stratum_editor_lib`. Renderer suites (buffer pool, upload batch, materials, textures, PBR shader, sky). Suites that need a real `SDL_GPUDevice` carry the ctest label `gpu` and **skip themselves with a stderr line** on machines with no usable backend — absence of a GPU never fails the build.

**Framework:** hand-rolled, dependency-free, in `tests/framework.hpp` + `framework.cpp`. Self-registering `TEST(Suite, name)` macro, non-fatal `CHECK_EQ` / `CHECK_TRUE` / `CHECK_NEAR` checks, and a runner taking an optional suite filter as `argv[1]`.

**Registration:** each suite name in the `STRATUM_TEST_SUITES` list gets one `add_test(NAME <Suite> COMMAND stratum_tests <Suite>)` ctest entry (label `unit`), so a ctest failure names the suite, and a filter matching no test exits non-zero (catches suites renamed in CMake but not in source). Adding a suite = one row in `STRATUM_TEST_SOURCES` + one row in `STRATUM_TEST_SUITES`. Several suites are guarded on their implementation file existing (`tessellation.cpp`, `lod_chunk.cpp`, various `renderer/*.cpp`) so a partial tree still configures and links.

```bash
# Everything, one command
cmake --workflow --preset tests

# Stepwise
cmake --preset tests
cmake --build --preset tests
ctest --preset tests

# One suite, directly (fastest iteration; filter = suite name)
./build/tests/bin/stratum_tests JunctionTrim

# One suite via ctest
ctest --test-dir build/tests -R '^RoadGraph$' --output-on-failure

# Only the GPU-device suites / everything except them
ctest --test-dir build/tests -L gpu
ctest --test-dir build/tests -LE gpu
```

Fixtures are read from the **source** tree (`tests/data`, documented in `tests/data/README.md`; ctest working directory is `tests/`). OBJ dumps from the dump suites go to `build/<preset>/road_dump` — build output, never fixtures. GPU tests load the checked-in `.spv` shaders from `assets/shaders` in the source tree.

## Everything else

**`launch-features.sh`** — not part of the build. A tmux launcher that spins up parallel `claude -p` feature agents, one window each (entity selection, gizmos, save/load, shadows, pdf-docs), in git worktrees kept **outside** the repo (`../Stratum-Worktrees`, overridable via `STRATUM_WT_DIR`; in-repo worktrees once got committed as stray gitlinks). Treat it as a historical parallel-development driver, not something an agent should run.

**`.github/workflows/`** — two workflows. `build.yml` builds Release on a three-OS matrix (ubuntu/gcc, windows/cl, macos/clang++) with recursive submodule checkout, on push/PR to `master`/`main` and on `v*` tags; Linux deps are apt-installed in the job. `generate-docs.yml` produces a C4 architecture PDF (PlantUML jar + pandoc + xetex) and runs only when the commit message contains `[docs]` or on manual dispatch.

**Doxygen** — configure with the `docs` preset (or `-DSTRATUM_BUILD_DOCS=ON`); this adds a `docs` custom target that runs `Doxyfile` from the source dir (`cmake --build --preset docs` invokes it, since that build preset targets `docs`). Per `Doxyfile`: project "Stratum", input `src`, HTML output under `docs/generated`. Requires `doxygen` and `graphviz` on PATH; missing Doxygen is a hard configure error under the preset (`find_package(Doxygen REQUIRED)`).
