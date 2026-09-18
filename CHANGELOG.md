# Changelog

All notable changes to Stratum are recorded here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and Stratum uses
[semantic versioning](https://semver.org/spec/v2.0.0.html).

Releases map to the milestones in `docs/plans/milestones.md`.

## [Unreleased]

## [0.6.0] — 2026-09-18

Milestones **M3 (blocks and lots)** and **M4 (rule engine)** complete, plus
three of the seven M5 items. `0.5.0` was never tagged; this release carries M3
with M4 rather than back-dating one.

**Stratum stops being a viewer that edits and starts being a generator.** A
rule file now turns a lot into a building: floors, a facade split into bays,
windows with reveals, and a hipped roof.

```
rule Main    { floors(4, 3.0);
               select face { front : { Facade(); } top : { Roof(); } } }
rule Facade  { split(y) { repeat { 3.0 : { Floor(); } } } }
rule Floor   { split(x) { repeat { 2.0 : { Bay(); } } } }
rule Bay     { split(x) { 0.5 : { Pier(); } 1.0 : { Opening(); }
                          ~1.0 : { Pier(); } } }
rule Opening { window(); }
rule Roof    { align_scope("y_up"); roof("hip", 35.0); }
```

### Added

- **The rule language** (`src/procgen/rules/`) — lexer, AST, parser and a
  deterministic interpreter over a shape tree. Purpose-built rather than a
  CGA reader, which was decision Q3: conforming to CGA would make its limits
  our ceiling. Section 8 of the CityEngine inventory is a coverage checklist,
  not a conformance target.

  Names are **lexically** scoped: a rule sees its own parameters and lets, the
  file's attributes and constants, and nothing of its caller. A rule whose
  meaning depended on who called it could not be read on its own.

  Determinism is structural. A shape's random state comes from its **address
  in the tree**, never from a shared stream, so adding one `choose` arm at the
  top of a file does not move every shape generated after it.

- **`split` and `select`** — absolute, relative and floating (`~`) sizes,
  `repeat`, nested splits, and component selection by direction. The floating
  size is what makes a facade absorb its remainder into the last pier.
- **Roof operations** — one `roof(kind, pitch, overhang)` with the kind as an
  argument, so a rule can compute which roof to raise. Six kinds: gable, hip,
  pyramid, ridge, shed, dome. Four go through the shared straight skeleton, so
  a hip has a real ridge rather than an apex.
- **Mass modelling** — `floors`, `courtyard` and `podium`. A building as
  stacked floors rather than one extrusion, so the structure a later
  `split(y)` needs survives.
- **Facade operations** — `window`, `door` and `wall_panel`, each with a
  reveal. A window drawn flat on the wall plane is the clearest tell of
  generated architecture.
- **The rule editor panel** — write a rule, run it, see the building. Runtime
  errors render through the same caret a parse error gets, and a cap that was
  hit is called out in red rather than left to look like success.
- **Straight skeleton** (`src/geometry/straight_skeleton.*`) — shared by lot
  subdivision and the roof operations, which is why it sits under
  `src/geometry/` rather than under `road/`.
- **Lots, lot edges and zoning** (`osm/road/lots.*`) — blocks subdivided into
  lots with street-side classification.
- **`full_operations()` / `full_functions()`** (`procgen/rules/registry.*`) —
  the union of every operation family, in core rather than in the editor,
  because the Python bindings and any headless path need the same table.
- **Working Python bindings** — `stratum_python` builds and the bindings run.

### Fixed

- **`taper` on a footprint with a courtyard sloped the hole wall the wrong
  way.** The offset normal was flipped by each ring's signed area, so a face
  inset shrank its holes instead of growing them: over a 20×20 slab with a 4×4
  lightwell, `taper(1.0)` left a 2×2 hole where 6×6 is correct, and the volume
  came out 352 against the prismatoid integral's 336. The same bug then
  refused `taper(3.0)` — an ordinary frustum — as a collapse.
- **A mirrored scope emitted every triangle inside out.** The renderer culled
  the front faces and lit the back ones. Both operation orders were broken for
  opposite reasons, so both halves of the fix are needed.
- **`offset` and `setback` reported success having done nothing** for any
  distance between 2e-9 and about 5e-6 — a dead band between the point epsilon
  and the Clipper2 integer grid. `setback` also handed back an empty border
  while claiming to have removed one.
- **`scale` was a silent no-op on a shape with no geometry**, which is exactly
  the case reserved so a rule can build a scope and then insert an asset into
  it.
- **Every wall of a box got a different texture orientation.** The UV basis
  seeded itself from the face normal alone, so a wall facing x had its texture
  rotated a quarter turn against one facing z, and three of four ran negative.
- The GPU test suites crashed at exit, and a test that bailed early counted as
  a pass. The framework now has `skip_test()` and `register_teardown()`.
- CI ran the suite only on pull requests targeting `master`, so every stacked
  pull request merged on a green tick that had never run against its own code.

### Changed

- `src/procgen/rules/ast.hpp`'s `kBuiltinOperations` is the parse-time name
  catalogue: a name absent from it does not resolve, so registering a handler
  is only half of adding an operation. A test pins which rows still have no
  handler, so wiring a family in and forgetting `registry.cpp` cannot pass
  quietly.

### Known limitations

- The straight skeleton does not handle interior rings, so five of the six
  roof kinds refuse a courtyard footprint rather than emit geometry over land
  that does not exist (#127).
- `shapeL`, `shapeU` and `shapeO` were never implemented (#129).
- Texturing operations have nowhere to store a UV: `ShapeGeometry` carries no
  UV set, and `shape_to_mesh()` projects at triangulation time (#45).

## [0.4.0] — 2026-09-16

Milestone **M2 (scene foundations)** complete, plus the first of M3, M7 and M8.

Stratum stops being a viewer. It now has a scene it can edit, undo and save.

### Added

- **The scene model** (`src/scene/`), which everything after this stands on:
  an undo/redo command stack, a layer tree, typed attributes with layer
  inheritance and source tracking, selection, session georeferencing, and
  save/load to a `.stratum` document.

  Every mutation goes through the command stack, and that is enforced by the
  compiler rather than by convention: `LayerTree`, `AttributeStore` and
  `MapLayer` keep their mutating API private with their command classes as the
  only friends.

- **Block extraction** (`osm/road/blocks.*`) — the planar faces a street
  network encloses. The keystone of the roadmap: lots, rule targets and
  facades all sit on it.
- **Street graph editing** (`osm/road/graph_edit.*`) — add, move, delete and
  split, through the command stack, preserving the rule that junctions are
  found by shared OSM node identity rather than endpoint proximity.
- **Heightmap import** (`procgen/heightmap_io.*`) — PNG and PGM. Terrain is no
  longer noise-only.
- **Typed map layers** (`scene/map_layer.*`) — obstacle, water, texture, scalar
  and function layers, which street growth, zoning and scatter all consume.
- **Whole-scene export** (`osm/scene_export.*`) — buildings, areas and terrain
  leave through the same path as roads, keeping the invariant that every
  triangle lands in exactly one chunk and none is split at a boundary.
- `stratum_core` may now read and write images, a deliberate decision (#13) so
  that heightmap import and, later, facade texturing work with no window open.

### Fixed

Found by adversarial review with mutation testing, on code that compiled and
passed its own tests:

- **Block extraction produced faces with a slit in the ring.** Antennae were
  cancelled positionally rather than topologically, so a spur ending in a loop
  came back with a perimeter of 597.99 where the answer is 456.57. An exact
  duplicate way destroyed the whole surrounding block, returning zero faces
  where there should be one.
- **Saving a document could terminate the process.** Non-UTF-8 text in a layer
  name reached the JSON writer, which throws, with no handler — so a name
  pasted from a Latin-1 source killed the editor and took the unsaved document
  with it.
- A map layer would accept a short buffer and a channel/type mismatch on load,
  then be sampled out of bounds.
- Chunk indexing in the scene exporter was signed-overflow UB on a scene far
  from the origin, and object names were not sanitised against path escape.
- Selection could invalidate its own iterators: a const read swept dead members
  out of the storage those iterators indexed.

### Changed

- `CLAUDE.md` corrected. Among other drift it claimed an EnTT ECS for scene
  management; EnTT is linked and included by nothing. The scene model is plain
  owning structures with stable handles.
- `docs/roadmap.md` removed, superseded by `docs/plans/milestones.md`.


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
