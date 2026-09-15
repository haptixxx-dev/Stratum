# CityEngine parity plan

**Goal.** Stratum reaches feature parity with Esri CityEngine, then beats it on
game-engine ergonomics. This file enumerates the gaps. It does not restate what
already works.

**Premise.** CityEngine is four products welded together: a *street graph
editor*, a *lot subdivider*, a *shape grammar* (CGA), and an *export pipeline*.
Stratum currently has a very good road **mesher** and no authoring layer at all.
Parity is mostly about the three missing products, not about polish.

**Read with:** `docs/plans/cityengine_feature_inventory.md` (the full CityEngine
2026.0 feature catalogue this plan derives from), `docs/roadmap.md` (stale, Jan 2026, still names X3 as the target
engine — supersede it), `docs/plans/road_network_plan.md` (the P0–P7 road work
that is done).

---

## 0. Out of scope for this document

Already built, not repeated below:

| Have | Where |
|---|---|
| OSM parse (`.osm`/`.pbf`), roads/buildings/areas, tag semantics | `src/osm/parser.cpp`, `src/osm/types.hpp` |
| Network-wide road solve: topology, trim/fillet junctions, roundabouts, bridges, tunnels, crossings, sidewalks, markings | `src/osm/road/` |
| Road LOD chunking + collision meshes | `road/lod_chunk.*`, `road/collision_mesh.*` |
| Chunked OBJ + glTF export of the road network | `road/road_export.*` |
| Heightmap terrain: noise, carve, mesh, tile streaming | `src/procgen/` |
| PBR renderer, shadow cascades, baked AO, material library, procedural textures | `src/renderer/`, `src/geometry/ambient_occlusion.*` |
| Editor shell: docking, viewport, camera, im3d, gizmos, OSM/procgen/material panels | `src/editor/` |
| Quadtree spatial index, frustum culling | `src/osm/quadtree.*` |

---

## Track A — Scene model and persistence

CityEngine's scene is a typed layer stack (map layers, shape layers, graph
networks, static models) with per-object attributes and full undo. Stratum has
an EnTT registry populated by an importer and nothing that survives a restart.

| ID | Feature | Size | Notes |
|---|---|---|---|
| A1 | Scene document: named layers, visibility, lock, per-layer transform | M | Layer is the unit of export filtering later (H3) |
| A2 | Save/load scene to disk (`.stratum`, JSON + binary blobs) | L | `nlohmann_json` already links into core; geometry blobs beside it |
| A3 | Attribute system: arbitrary typed key/values on any object, inherited layer → object | M | Prerequisite for D (rules read attributes) and I (reports) |
| A4 | Undo/redo command stack across every edit | L | Retrofitting this later is far worse than doing it before B/C land |
| A5 | Selection model: multi-select, by-type filter, by-attribute filter | M | |
| A6 | Georeferencing kept through the whole session; re-project on demand | S | `set_origin()` trap in `osm/coordinates.hpp` becomes load-bearing |

**Do A4 first or accept never having it.** Every later track adds edit
operations; each one added without a command stack is a migration to pay for.

## Track B — Street graph authoring

Stratum consumes a graph. CityEngine *authors* one. `road::RoadGraph` is the
right substrate; it needs mutation, tooling, and generation.

| ID | Feature | Size | Notes |
|---|---|---|---|
| B1 | Mutable graph API: add/move/delete node and segment, split segment, with invariants preserved | M | Must keep node-identity junction rule intact |
| B2 | Interactive street tool: draw polyline, snap to node/segment/angle/grid, live preview | L | Needs A4 |
| B3 | Per-segment attribute editing: class, lanes, width, oneway, sidewalk/cycleway/parking, surface | S | Fields already exist on `Road` |
| B4 | Incremental re-solve: edit one segment, rebuild only the affected junctions and corridors | L | Full re-solve on every drag will not be usable at city scale |
| B5 | Street growth: organic pattern (angle deviation, segment length, seed) | L | CE's headline generator |
| B6 | Street growth: raster/grid pattern, with rotation and block-size targets | M | |
| B7 | Street growth: radial/ring pattern | S | After B5's framework exists |
| B8 | Growth environment sensitivity: obstacle layers, water, slope-aware routing | M | Feeds off Track G layers |
| B9 | Junction shape override per node (force roundabout, force signalled, manual polygon) | M | `junction_special.*` is the hook |

## Track C — Blocks and lots

Nothing exists here. This is the bridge from streets to buildings and it is
mandatory for parity.

| ID | Feature | Size | Notes |
|---|---|---|---|
| C1 | Block extraction: planar cycle enumeration over the street graph, minus carriageway/sidewalk footprint | L | The classic half-edge face-finding pass; must handle dangling ways and multi-component graphs |
| C2 | Lot subdivision — recursive bisection (CE "recursive") | M | Params: target area, width, irregularity, seed |
| C3 | Lot subdivision — offset/skeleton (CE "offset" and "skeleton") | L | Straight skeleton is the hard one; Clipper2 offsets get part of the way. **Plan it jointly with D5** — the roof operations need the same skeleton |
| C4 | Street-side classification per lot edge: front / side / back / interior | M | Rules depend on this for setbacks and facade choice |
| C5 | Manual lot edit: split, merge, reshape, lock against regeneration | M | |
| C6 | Zoning: assign a type/attribute set per block or lot, from OSM `landuse` or painted by hand | M | `Area` with `AreaType` is the seed data |
| C7 | Deterministic seeding **and stable lot indexing**: same seed and inputs give identical lots, and lot identity survives a street edit | M | CE maps lots through barycentric coordinates for exactly this. Without it, every rule assignment shuffles when a street moves. Do it at the start of C2 |

## Track D — Shape grammar

CityEngine *is* CGA. Without a rule engine there is no parity, only a nicer OSM
viewer. **This is the largest single decision in the plan — see Open questions.**

| ID | Feature | Size | Notes |
|---|---|---|---|
| D1 | Rule language: parser, AST, deterministic interpreter over a shape tree | XL | Or an embedded language — see Q1 |
| D2 | Scope/geometry ops: `extrude`, `offset`, `inset`, `taper`, `setback`, `shapeL/U/O` | L | |
| D3 | Split ops: `split` with absolute / relative / floating sizes, `repeat`, nested splits | L | The `~` floating-size semantics are what make CE facades work |
| D4 | Component split `comp`: faces, edges, by direction (front/side/top/bottom) | M | |
| D5 | Roof ops: `roofGable`, `roofHip`, `roofShed`, `roofPyramid`, `roofDome` | L | Straight skeleton again; shares work with C3 |
| D6 | Conditionals, stochastic rules (weighted alternatives), user attributes with defaults | M | |
| D7 | Asset insertion `i()` with scope alignment | M | Needs Track F |
| D8 | Texturing ops: `setupProjection`, `projectUV`, `texture`, `tileUV` | L | |
| D9 | Occlusion and context queries (`inside`, `overlaps`, `touches`) | L | CE uses these for shared walls and balcony trimming; defer past first parity pass |
| D10 | Rule editor panel: edit, live re-generate on selected shapes, error surfacing | M | |
| D11 | Content library shipped with Stratum: building rules (residential, apartment, office, retail, industrial, warehouse), lane rules, named street configurations, window/balcony/massing component sets, a plant loader | L | ESRI.lib is a large shipped deliverable, not an afterthought. Parity is worthless without stock content; building rules map onto existing `BuildingType` |

## Track E — Buildings beyond extrusion

`MeshBuilder::build_building_mesh()` is a flat extrude. `RoofType` is parsed and
then ignored.

| ID | Feature | Size | Notes |
|---|---|---|---|
| E1 | Finish `RoofType` support | M | **Partly done already.** Gabled, hipped and pyramidal reach geometry; skillion and dome fall through to flat, hipped is built as a pyramid (one apex, no ridge), gabled emits overlapping end faces, and any courtyard footprint falls back to flat. Ships before D5 and is reused by it |
| E2 | Mass modelling: floor stacking, setbacks, courtyards from `holes`, podium/tower | M | |
| E3 | Facade generation driven by D: windows, doors, floors, ground-floor retail | L | |
| E4 | Interior-block fill: yards, driveways, fences, pools on lots | M | |
| E5 | Building UV unwrap + lightmap UVs (second channel) | L | Game-engine requirement CE handles poorly |
| E6 | Per-building LOD chain, matching the road LOD scheme | M | Reuse `road/lod_chunk.*` ideas |
| E7 | Building collision meshes | S | Mirror `road/collision_mesh.*` |

## Track F — Assets, materials, instancing

| ID | Feature | Size | Notes |
|---|---|---|---|
| F1 | Asset library: import and index `.glb`/`.fbx`/`.obj` props, thumbnails, search paths | L | assimp already links into `stratum_editor_lib`; the importer must stay editor-side, the *placement data* must stay core |
| F2 | Instance placement as data, not merged geometry (transform lists per asset) | M | Engines want instances; CE merges and loses them |
| F3 | Texture atlas packer for facades and props | L | |
| F4 | Material assignment rules by attribute/type, beyond the road material set | M | `renderer/material_library.*` is the base |
| F5 | Vegetation and street furniture scatter (density maps, alignment to terrain/street) | M | CE ships plants; parity needs a stock set or a generator |

## Track G — Map layers and terrain

| ID | Feature | Size | Notes |
|---|---|---|---|
| G1 | Heightmap import (GeoTIFF, PNG16, ASC) with georeferencing | M | Currently terrain is noise-only |
| G2 | Typed map layers: obstacle, water, texture, and generic scalar | M | Consumed by B8, C6, F5 |
| G3 | Align shapes and graph to terrain; terrain-follows-street conform | M | `road_elevation.*` and `terrain_carve.*` do half of this for roads only |
| G4 | Terrain paint/sculpt brush in-editor | L | CE's is weak; cheap differentiator |
| G5 | Satellite/ortho imagery drape as a terrain texture layer | M | |

## Track H — Export and engine integration

Roads export well. Nothing else does.

| ID | Feature | Size | Notes |
|---|---|---|---|
| H1 | Whole-scene export, not just the road network: buildings, terrain, areas, props | L | Generalise `road_export.*` into a scene exporter; keep the "one triangle, one chunk" invariant |
| H2 | FBX export | M | Required for parity; assimp is editor-side, so either move it or hand-write |
| H3 | Export options matching CE: mesh granularity (no merge / merge by material / reuse asset instances), feature granularity (per shape / per leaf shape), memory budget, vertex precision and merge-within-precision, normals and UV controls, local and global offset, triangulation, texture atlas with max dimension and border, embed textures | L | A first-class subsystem in CE, not a checkbox row |
| H4 | USD export | M | Lower priority than FBX/glTF for games |
| H5 | Unity package export (prefabs, material mapping, instances preserved) | L | CE has this; it is a real adoption lever |
| H6 | Unreal export (Datasmith is proprietary — target glTF + a UE plugin instead) | L | |
| H7 | LOD sets emitted per object, engine-consumable naming (`_LOD0..n`) | M | |
| H8 | Draco compression on glTF output | S | Already linked into core, currently unused |
| H9 | Batch/headless export from the CLI | M | Needs A2 and J1 |

## Track I — Analysis and reporting

| ID | Feature | Size | Notes |
|---|---|---|---|
| I1 | Rule-emitted reports (CGA `report()` equivalent): floor area, unit counts, cost | M | Needs D and A3 |
| I2 | Dashboard panel: charts over the report data, live on selection | M | |
| I3 | Colour-by-attribute viewport mode | S | Cheap, very useful for debugging C and D |
| I4 | Solar/shadow analysis over a date range | M | **Not parity — a differentiator.** CityEngine has no solar or shadow analysis at all; users export to ArcGIS Pro for it. Stratum already owns the shadow pipeline |
| I5 | Viewshed and line-of-sight | M | Low priority for game dev |

## Track J — Scripting and automation

| ID | Feature | Size | Notes |
|---|---|---|---|
| J1 | Make the pybind11 bindings functional (they are stubbed, `STRATUM_ENABLE_PYTHON=ON` but non-working) | L | Exposes import → generate → export |
| J2 | Python API over the scene: query, mutate, run rules, export | L | CE's Python API is how studios actually use it |
| J3 | Headless CLI: `stratum --script foo.py --out city/` | M | |
| J4 | A runtime/SDK build of `stratum_core` for in-engine generation | XL | The real differentiator versus CE's PRT licensing |

## Track K — Editor UX

| ID | Feature | Size | Notes |
|---|---|---|---|
| K1 | Inspector that edits attributes on any selection, multi-edit aware | M | Needs A3, A5 |
| K2 | Layer/navigator panel | M | Needs A1 |
| K3 | Asset and rule browsers with thumbnails | M | |
| K4 | Generation progress, cancel, and background regeneration | M | City-scale rule runs cannot block the UI thread |
| K5 | Viewport handles for lots, blocks and graph nodes (beyond the current gizmos) | M | |
| K6 | Import wizard covering shapefile/GeoJSON alongside OSM | M | |

---

## Suggested order

Parity is a multi-year scope for one developer. Order matters more than the list.

1. **Foundations (A4, A1, A3, A2).** Command stack, layers, attributes, save/load.
   Everything after this assumes them; retrofitting is the single biggest
   avoidable cost in the plan.
2. **Visible wins that need no new architecture (E1, I3, H8, E7).** Real roofs,
   colour-by-attribute, compressed export, building collision.
3. **The streets→lots spine (B1, B3, C1, C7, C2, C4).** This is what turns the
   viewer into an authoring tool, and C1 unblocks every building feature.
4. **The grammar (D1–D6, D10, D11).** Answer Q1 before starting.
5. **Buildings for real (E2, E3, E5, E6, F1–F3).**
6. **Authoring polish (B2, B4, B5–B8, C3, C5, K1–K5).**
7. **Pipeline (H1–H3, H7, H9, J1–J3).**
8. **Long tail (I, G4, G5, D9, H4–H6, J4).**

## Decisions

Answered 2026-09-15; closed on the tracker as #12 to #15. Full reasoning is in
`docs/plans/milestones.md`.

- **Q1 — rule language: a purpose-built text language.** Node graph later, over
  the same language, as Esri did.
- **Q3 — no, do not read CityEngine `.cga`.** Conforming to CGA would make its
  limits our ceiling. Section 8 of the inventory is a coverage checklist —
  measure the language against those capabilities, not against that syntax. The
  cost is that existing CityEngine rule packs cannot be brought over.
- **Q2 — yes, `stratum_core` may read and write images.** A CPU-only image
  library in core, so D8 and F3 work headlessly. SDL, ImGui and renderer stay
  excluded.
- **Q4 — yes, FBX ships alongside glTF.** Behind an interface, so the glTF and
  OBJ paths stay independently testable and the suite runs without the FBX
  library present.

## Non-goals

Explicitly not chasing CityEngine here: ArcGIS/geodatabase integration, i3s and
Scene Layer Packages, web-scene publishing, CityEngine VR, and the Esri
ecosystem lock-in generally. These are GIS deliverables, not game assets.
