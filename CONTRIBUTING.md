# Contributing to Stratum

Stratum converts OpenStreetMap data into game-ready 3D cities. The long-term
goal is a game-development-focused alternative to Esri CityEngine. See
`docs/plans/milestones.md` for where the project is going and
`docs/plans/cityengine_parity.md` for the feature backlog.

## Licence

Stratum is Apache-2.0. By contributing you agree that your contribution is
licensed under the same terms. Every new source file needs the two-line header:

```cpp
// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors
```

## Build

All dependencies are vendored as git submodules. Nothing is fetched at
configure time.

```bash
git clone --recurse-submodules https://github.com/haptixxx-dev/Stratum.git
cd Stratum
cmake --preset release
cmake --build --preset release
./build/release/bin/stratum
```

System prerequisites on Arch: `cmake ninja python zlib bzip2 expat`. On Debian
and Ubuntu, see the package list in `.github/workflows/build.yml`.

Set `STRATUM_LAUNCHER=ccache` in the environment to use a compiler launcher.

## Tests

Tests are off by default (`STRATUM_BUILD_TESTS` defaults `OFF`). They exist and
they pass.

```bash
cmake --preset tests
cmake --build --preset tests
ctest --preset tests
```

Two executables are produced: `stratum_tests` (core only) and
`stratum_gpu_tests`, whose GPU-labelled suites skip themselves when no device
is present. CI runs the `ci` preset, which is the same thing with Tracy and
pybind11 off.

**Every behaviour change needs a test.** Geometry work in particular should use
golden tests — the road network suite under `tests/road/` is the pattern to
copy.

## Architecture rules

Two of these are load-bearing and easy to break:

1. **`stratum_core` must not depend on SDL, ImGui, or any rendering code.** It
   contains `src/osm`, `src/osm/road`, `src/geometry` and `src/procgen` only.
   Anything that needs a window or a GPU belongs in `stratum_editor_lib`.
2. **`src/core` is not part of `stratum_core`.** Its two classes
   (`Application`, `Window`) compile into `stratum_editor_lib` and include SDL
   headers. The naming is a trap; engine-agnostic code goes in `src/osm`,
   `src/geometry` or `src/procgen`.
3. **Junctions are found by shared OSM node identity, never by endpoint
   proximity.** A road with an empty `node_ids` cannot join the graph.

`CLAUDE.md` indexes the deeper subsystem guides in `docs/agents/`. Read the one
covering the code you are touching before changing it; where a guide and
`CLAUDE.md` disagree, the guide is newer and wins.

## Commits and pull requests

- Conventional Commits: `fix(osm): refuse a junction ring that excludes its own node`.
- Branch off `master`. One logical change per PR.
- The `Tests (Linux)` CI job gates merge. The three-platform matrix must also
  build.
- Explain *why* in the commit body when the *what* is not obvious from the
  diff. The road network history is a good model.

## Filing issues

Use the templates. For a geometry bug, the single most useful thing you can
attach is the `.osm` extract and the coordinates that reproduce it.
