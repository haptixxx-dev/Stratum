# Stratum milestones

Ten milestones from the current OSM viewer to CityEngine parity plus a runtime
story CityEngine cannot match. Each milestone is a GitHub Milestone, a semver
release, and a CI-verified tag.

**Inputs:** `docs/plans/cityengine_feature_inventory.md` (what CityEngine 2026.0
does) and `docs/plans/cityengine_parity.md` (the tracks and feature IDs, A1–K6).
Feature IDs below refer to that plan. This file supersedes `docs/roadmap.md`.

## Pace

This is roughly 60 features. With a funded budget and frontier coding models
driving the implementation, the realistic unit of work per milestone is weeks,
not quarters — the whole programme is a months-long effort, not a multi-year
one. The ordering constraints below are therefore the real schedule risk, not
raw engineering hours:

- **A4 (undo/redo) precedes every edit feature.** Retrofitting a command stack
  across Tracks B, C, E and K is the one genuinely unparallelisable cost.
- **C1 (block extraction) gates Tracks C, D, E and F.** Nothing downstream of
  lots can start until blocks come out of the road graph.
- **D1 (rule interpreter) gates D2–D11, E3 and F2.**
- Everything else fans out and can run concurrently.

## Release and CI contract

Every milestone ends the same way:

1. `VERSION` bumped, `CHANGELOG.md` updated.
2. Tag `vX.Y.Z` pushed to `master`.
3. `.github/workflows/build.yml` builds Linux, Windows and macOS, runs
   `ctest --preset tests`, and attaches per-platform archives to a GitHub
   Release.
4. The GitHub Milestone closes with every issue in it closed.

A milestone is not done while CI is red or its issues are open.

---

## M0 — v0.2.0 · Open source ready

Make the repository something a stranger can build, run and contribute to.

| Work | Detail |
|---|---|
| Apache-2.0 licensing | `LICENSE`, `NOTICE`, `THIRD_PARTY_LICENSES.md`, SPDX headers across `src/` and `tests/`, README licence section |
| CI runs the tests | `STRATUM_BUILD_TESTS=ON` in CI and `ctest --preset tests` wired in; the test step is currently commented out in `build.yml` |
| CI hygiene | Submodule caching, ccache, concurrency cancellation, a fast Linux-only PR job separate from the full three-platform matrix |
| Contributor docs | `CONTRIBUTING.md`, `CODE_OF_CONDUCT.md`, `SECURITY.md`, issue and PR templates |
| Repo hygiene | Drop the 29 stale `.gitmodules` entries (28 under `submodules/external/` plus `basis_universal`, none of which is in the git index) |
| Changelog | `CHANGELOG.md`, Keep a Changelog format |

**Exit criteria:** a clean clone builds on all three platforms in CI, tests pass,
`v0.2.0` is tagged and has release artefacts.

## M1 — v0.3.0 · Visible wins

Four features that need no new architecture and each ship independently.

| ID | Work |
|---|---|
| E1 | Finish `RoofType` support. Gabled, hipped and pyramidal already reach geometry; skillion and dome fall through to flat, hipped is built as a pyramid, gabled emits overlapping end faces, and courtyard footprints fall back to flat |
| I3 | Colour-by-attribute viewport mode — the debugging tool every later track needs |
| E7 | Building collision meshes, mirroring `road/collision_mesh.*` |
| H8 | Draco compression on glTF export; already linked into core and unused |

**Exit criteria:** golden tests for each roof shape; a colour-by-attribute mode
in the viewport; collision meshes in the export; Draco on/off as an export flag.

## M2 — v0.4.0 · Scene foundations

The milestone everything else assumes. Nothing user-visible ships here, and
skipping it makes every later milestone more expensive.

| ID | Work |
|---|---|
| A4 | Undo/redo command stack across every edit **(do this first)** |
| A1 | Scene document: named layers, visibility, lock, per-layer transform |
| A3 | Attribute system: typed key/values on any object, layer → object inheritance |
| A5 | Selection model: multi-select, filter by type and by attribute |
| A2 | Save/load scene (`.stratum`, JSON + binary blobs) |
| A6 | Georeferencing held for the whole session |

**Exit criteria:** create a scene, edit it, undo every edit, save it, reload it,
get the same scene back. Round-trip covered by tests.

## M3 — v0.5.0 · Blocks and lots

The keystone. Turns the road graph into something buildings can stand on.

| ID | Work |
|---|---|
| C1 | Block extraction: planar cycle enumeration over the street graph, minus carriageway and sidewalk footprint |
| C7 | Deterministic seeding **and stable lot indexing** — CityEngine uses barycentric mapping so lot identity survives a street edit |
| C2 | Recursive (OBB bisection) lot subdivision |
| C4 | Lot edge classification: front / side / back / interior |
| C6 | Zoning: type assignment per block or lot, from OSM `landuse` or painted |
| B1 | Mutable graph API: add/move/delete node and segment, split segment, invariants preserved |
| B3 | Per-segment attribute editing (the fields already exist on `Road`) |

**Exit criteria:** import Lucan, get blocks; subdivide into lots; move a street
and watch lot identity survive. Golden tests on block and lot counts.

## M4 — v0.6.0 · Rule engine

Answer **Q1** before starting. Recommendation on the evidence: a CGA-like text
DSL first, with a node graph layered over it later — which is exactly what Esri
converged on, since the Visual CGA Editor compiles to CGA rule files.

| ID | Work |
|---|---|
| D1 | Rule language: parser, AST, deterministic interpreter over a shape tree |
| D2 | Scope and geometry ops: `extrude`, `offset`, `inset`, `taper`, `setback`, `shapeL/U/O` |
| D3 | Split ops: absolute, relative and floating sizes; `repeat`; nested splits |
| D4 | Component split: faces, edges, by direction |
| D6 | Conditionals, stochastic rules, user attributes with defaults |
| D10 | Rule editor panel with live regeneration and error surfacing |

**Exit criteria:** a rule file turns a lot into a mass model with floors and a
roof; identical seeds give byte-identical geometry.

## M5 — v0.7.0 · Buildings for real

| ID | Work |
|---|---|
| D5 | Roof operations (`roofGable`, `roofHip`, `roofShed`, `roofPyramid`, `roofDome`) — shares the straight skeleton with C3 |
| D7 | Asset insertion with scope alignment |
| D8 | Texturing ops: `setupProjection`, `projectUV`, `texture`, `tileUV` |
| E2 | Mass modelling: floor stacking, setbacks, courtyards from `holes`, podium/tower |
| E3 | Facade generation: windows, doors, floors, ground-floor retail |
| E5 | UV unwrap plus a second lightmap UV channel |
| E6 | Per-building LOD chain matching the road LOD scheme |

**Exit criteria:** a generated block of buildings with real facades, LODs and
lightmap UVs, exported and opened in an engine.

## M6 — v0.8.0 · Assets, materials, content

ESRI.lib is a large shipped deliverable. A rule engine with no content is not
parity.

| ID | Work |
|---|---|
| F1 | Asset library: import and index `.glb`/`.fbx`/`.obj`, thumbnails, search paths |
| F2 | Instance placement as data (transform lists), not merged geometry |
| F3 | Texture atlas packer for facades and props |
| F4 | Material assignment rules by attribute and type |
| F5 | Vegetation and street furniture scatter |
| D11 | The content library: building rules, lane rules, named street configurations, window/balcony/massing component sets, plant loader |

**Exit criteria:** a stock content set ships with Stratum and generates a
plausible city with no hand-authoring.

## M7 — v0.9.0 · Street authoring

| ID | Work |
|---|---|
| B2 | Interactive street tool: draw, snap to node/segment/angle/grid, live preview |
| B4 | Incremental re-solve — edit one segment, rebuild only affected junctions and corridors |
| B5–B7 | Street growth: organic, raster, radial |
| B8 | Growth environment sensitivity: obstacles, water, slope |
| B9 | Per-node junction shape override |
| C3 | Offset and skeleton lot subdivision |
| C5 | Manual lot edit: split, merge, reshape, lock |
| K1–K5 | Inspector, layer panel, asset and rule browsers, background regeneration, viewport handles |

**Exit criteria:** author a neighbourhood from nothing — grow streets, get
blocks, get lots, get buildings — without importing OSM at all.

## M8 — v0.10.0 · Pipeline

| ID | Work |
|---|---|
| H1 | Whole-scene export: buildings, terrain, areas, props — generalise `road_export.*` while keeping the one-triangle-one-chunk invariant |
| H2 | FBX export (see **Q4**) |
| H3 | Export options matching CityEngine: mesh and feature granularity, memory budget, vertex precision, normals and UV controls, offsets, triangulation, atlas controls |
| H7 | LOD sets with engine-consumable naming |
| H9 | Batch and headless export from the CLI |
| J1–J3 | Make pybind11 functional; scene API; `stratum --script foo.py --out city/` |
| G1–G3 | Heightmap import, typed map layers, align shapes and graph to terrain |

**Exit criteria:** a scripted headless run imports data, generates a city and
exports engine-ready assets with no GUI.

## M9 — v1.0.0 · Parity and the runtime play

| ID | Work |
|---|---|
| I1, I2 | Rule-emitted reports and a dashboard over them |
| I4 | **Solar and shadow analysis — a differentiator, not parity.** CityEngine has none; users export to ArcGIS Pro. Stratum already owns the shadow pipeline |
| G4, G5 | Terrain paint/sculpt brush; ortho imagery drape |
| D9 | Occlusion and context queries |
| H4–H6 | USD export; Unity package export; Unreal via glTF plus a UE plugin |
| J4 | A runtime build of `stratum_core` for in-engine generation |

**Exit criteria:** feature parity on everything not marked ⛔ in the inventory,
plus a permissively licensed runtime that generates in-engine — the one thing
Esri's licensed PRT structurally cannot offer.

---

## Non-goals

Carried from the inventory, unchanged: ArcGIS and geodatabase integration, i3s
and Scene Layer Packages, web-scene publishing, CityEngine VR, 3VR export, and
Esri ecosystem lock-in generally. These are GIS deliverables, not game assets.

## Open questions

Tracked as GitHub issues, restated here because they gate milestones:

- **Q1 — rule language.** Gates M4. Recommendation: text DSL first, node graph
  over it later.
- **Q2 — does `stratum_core` stay SDL-free and GPU-free?** Facade texturing and
  atlasing want image work in core. Gates M5 and M6.
- **Q3 — is CityEngine `.cga` rule import a goal?** Large adoption lever, large
  constraint on Q1. Gates M4.
- **Q4 — FBX or glTF only?** FBX pulls assimp into an export path that is
  currently clean, testable and headless. Gates M8.
