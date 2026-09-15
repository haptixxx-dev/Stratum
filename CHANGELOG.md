# Changelog

All notable changes to Stratum are recorded here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and Stratum uses
[semantic versioning](https://semver.org/spec/v2.0.0.html).

Releases map to the milestones in `docs/plans/milestones.md`.

## [Unreleased]

## [0.3.0] — 2026-09-15

Milestones **M0 (open source ready)** and **M1 (visible wins)**, both complete.

No 0.2.0 was ever cut. M0 and M1 landed within hours of each other, and tagging
the same tree twice to honour a plan written before either shipped would have
been bookkeeping rather than a release. M0's content is in here.

### Added

- **Apache-2.0 licensing.** `LICENSE`, `NOTICE`, `THIRD_PARTY_LICENSES.md`
  covering all 28 vendored submodules, and SPDX headers across `src/` and
  `tests/`. Every dependency was checked against its own licence file; all are
  permissive. lz4 is dual-licensed by directory and only its BSD-2-Clause `lib/`
  is linked. `NOTICE` also records what the licence does not cover: OSM data is
  ODbL 1.0 and generated models are derivative works of it.
- **CI that builds on Linux, Windows and macOS and runs the test suite.**
- Skillion and dome roofs, so all six `roof:shape=*` values reach geometry.
- Building collision meshes (`build_building_collision_mesh()`), generated
  straight from the footprint rather than derived from the render mesh.
- Optional Draco compression on glTF export, off by default, with the
  uncompressed accessors still emitted so a loader without the extension falls
  back rather than failing.
- Colour-by-attribute viewport mode over `BuildingType`, `RoadType`, `AreaType`
  and quadtree leaf, with the colour table a pure, tested function in
  `stratum_core`.
- `CONTRIBUTING.md`, `CODE_OF_CONDUCT.md`, `SECURITY.md`, issue forms, a PR
  template and this changelog.
- The planning set: `docs/plans/cityengine_feature_inventory.md`,
  `cityengine_parity.md` and `milestones.md`.

### Fixed

- **Windows has never built this project.** Four stacked blockers, each hidden
  behind the last: `cl` was not on `PATH`; zlib, bzip2 and expat are absent from
  the runner image; `<windows.h>`'s `min`/`max` macros broke every
  `std::numeric_limits<T>::max()` in libosmium; and a global `/utf-8` collided
  with assimp's own `/source-charset:utf-8`. Only the first was visible from the
  original failure.
- **Gabled roofs emitted overlapping faces.** Every footprint edge was connected
  to the whole ridge segment, so the end edges of a rectangle each produced a
  quad spanning the full ridge — two large sheets lying across the roof.
- **Hipped roofs were built as pyramids**, with one apex and no ridge.
- **Building collision hulls could be open.** The guard tested the triangulation
  for emptiness, which is not the failure mode that occurs: earcut deletes points
  and returns a smaller triangulation whose boundary is no longer the input ring.
  A figure-eight footprint produced 8 unpaired edges; a hole straddling the outer
  ring produced 205 m² of collision roof against a 200 m² footprint. Now checked
  against `V + 2H - 2`.
- **Draco silently dropped degenerate triangles** and published counts that hid
  the loss, falsifying the export conservation invariant with respect to what is
  on disk.
- **One non-finite attribute deleted an entire chunk file** when compression was
  on. One bad vertex now costs one vertex.
- **The attribute overlay ignored the terrain drape**, drawing road centrelines
  at a constant world Y while the geometry they describe is lifted onto the
  surface. Im3d depth-tests, so the overlay was invisible rather than misplaced.
- **Building overlay rings were coplanar with their own walls** and were
  discarded by the depth test; they are now offset out of the surface.
- Shader loading resolved `SDL_GetBasePath() + "../../assets/shaders/"`, which
  from `build/<preset>/bin` lands in `build/<preset>/assets` and never finds the
  source tree.

### Removed

- 29 stale `.gitmodules` entries, none of which was registered in the git index.


### Added

- Apache-2.0 licensing: `LICENSE`, `NOTICE`, `THIRD_PARTY_LICENSES.md`, and
  SPDX headers across every file in `src/` and `tests/`.
- `CONTRIBUTING.md`, `CODE_OF_CONDUCT.md`, `SECURITY.md`, issue forms and a
  pull request template.
- CI now builds *and runs the test suite* on Linux via the `ci` preset. The
  test step had been commented out since the workflow was written.
- CI caches compiler output with ccache (sccache on Windows), cancels
  superseded runs, and refuses to publish a release whose tag disagrees with
  `VERSION`.
- `docs/plans/cityengine_feature_inventory.md` — full ArcGIS CityEngine 2026.0
  feature catalogue with Stratum's status against each item.
- `docs/plans/cityengine_parity.md` — the parity backlog, tracks A–K.
- `docs/plans/milestones.md` — ten milestones from here to v1.0.0.

### Fixed

- Windows CI has failed at the configure step since August 2026 with
  `CMAKE_CXX_COMPILER: cl is not a full path`. Ninja plus `cl` needs the MSVC
  environment on `PATH`; the workflow now sets it up before configuring.

### Removed

- 29 stale `.gitmodules` entries — 28 under the non-existent
  `submodules/external/` prefix plus `basis_universal`, none of which was
  registered in the git index.

## [0.1.0]

Initial development. OSM import, road network solve, terrain generation,
SDL_GPU renderer with PBR and cascaded shadows, ImGui editor.
