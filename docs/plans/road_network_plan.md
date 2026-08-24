# Road Network Generation Plan

## Overview

Replace the current flat-ribbon road extrusion with a topology-driven road network
generator that produces game-ready geometry: curbs, sidewalks, solved intersections,
crossings, lane markings, bridges, and terrain-integrated corridors.

Target coverage: modern urban, suburban, and rural road networks.

## Current State

| Component | File | Behaviour |
|---|---|---|
| Road extrusion | `src/osm/mesh_builder.cpp:401` | Flat ribbon at `y = 0.1`, single colour, per-quad UVs |
| Junctions | `src/osm/mesh_builder.cpp:609` | 12-segment disc at `y = 0.06` where way endpoints cluster within 2 m |
| Spatial index (live path) | `src/osm/quadtree.cpp:259` | `insert_road` routes each road to one leaf by centroid; meshes built per leaf |
| Spatial index (dead code) | `src/osm/tile_manager.cpp:82` | Full `Road` copy pushed into every tile the polyline touches. `TileManager` is not used by the editor. |
| Road data | `src/osm/types.hpp` | `polyline`, `type`, `width`, `lanes`, `is_bridge`, `is_tunnel` |

### Defects that make exported meshes unusable

1. **Geometry is built per spatial-index leaf, so topology stops at leaf boundaries.**
   The live path is `QuadTree` (`quadtree.cpp:486` `build_node_meshes_internal`), not
   `TileManager`. `insert_road` assigns each road to a single leaf by centroid, so there
   is no duplication there, but `build_junction_meshes` only ever sees one leaf's roads.
   Every junction whose arms fall in different leaves is missed or built wrong.
   Separately, `TileManager` **does** duplicate (`tile_manager.cpp:92` copies the whole
   `Road` into every tile it touches, then extrudes it in full). `TileManager` is dead
   code — the editor uses `QuadTree` — so it should be deleted rather than fixed.
2. **No topology.** `Road` discards OSM `NodeId`s at parse time. Junctions are inferred
   by clustering way *endpoints* within 2 m, which misses every T-junction where one
   way's shared node is interior to another way. Roundabouts, motorway links, and
   `layer` separation are invisible to the builder.
3. **No junction cut.** Ribbons pass straight through each other. The junction disc is
   emitted at `y = 0.06`, below the roads at `y = 0.1`, so it is never visible anyway.
4. **Incorrect miter.** `mesh_builder.cpp:456` averages normalised tangents but never
   divides the offset by `cos(theta/2)`. The ribbon pinches inward at every corner.
5. **Broken UVs.** Each quad is mapped `0..1`, so texture density varies with segment
   length. No tiling material can look correct.
6. **No cross-section.** One material, one vertex colour, no curb, gutter, shoulder,
   sidewalk, median, or verge.
7. **No terrain relationship.** Roads sit on a fixed plane and ignore the heightmap
   entirely.

## Design Decisions

| Decision | Choice |
|---|---|
| Junction geometry | Analytic trim + fillet per node. Clipper2 reserved for curb offsets and cleanup. |
| Cross-section granularity | Strip profile on a welded ribbon. Lane boundaries exist as vertex columns. |
| Terrain integration | Before the junction solver, using a provisional corridor, re-carved at P3. |
| Verification | Golden-file tests in `stratum_core` under `STRATUM_BUILD_TESTS`. |

### Note on terrain ordering

Terrain carving needs a corridor footprint, and the final footprint (with fillets)
only exists after the junction solver. The ordering is made to work by carving twice:

- **Pass 1 (P3, terrain):** carve against the P2 ribbon corridor (centerline offset by
  half the total profile width) plus a provisional junction footprint (convex hull of
  the arm end cross-sections).
- **Pass 2 (P4, junctions):** re-carve only the junction neighbourhoods once real
  fillet polygons exist.

The provisional footprint is a superset of the final one in the common case, so pass 2
is a refinement, not a correction of visible error.

## Module Layout

New directory `src/osm/road/`, all inside `stratum_core` (no SDL, no ImGui).

```
src/osm/road/
  road_material.hpp     MaterialId enum, SubMesh container
  road_graph.hpp/.cpp   Network topology from OSM node identity
  road_profile.hpp/.cpp Tag-driven cross-section strip model
  centerline.hpp/.cpp   Resample, smooth, miter, offset
  corridor.hpp/.cpp     Strip extrusion, submesh output, corridor polygon
  road_elevation.hpp/.cpp  Heightmap sampling, grade limiting, layer overrides
  junction_builder.hpp/.cpp  Trim, fillet, junction polygon, curb ring
  markings.hpp/.cpp     Lane markings, crossings, stop lines
```

Modified files:

- `src/osm/types.hpp` — `Road` gains topology and tag fields
- `src/osm/parser.cpp` — retain `NodeId`s and additional tags
- `src/osm/quadtree.cpp/.hpp` — consume global corridor output instead of per-leaf roads
- `src/osm/tile_manager.cpp/.hpp` — deleted (dead code, duplicates roads)
- `src/osm/mesh_builder.cpp` — `build_road_mesh` / `build_junction_meshes` removed
- `src/renderer/mesh.hpp` — submesh ranges
- `src/renderer/gpu_renderer.cpp` — draw per submesh range
- `src/procgen/terrain_generator.*`, `terrain_tile_manager.*` — carve hook
- `CMakeLists.txt` — test target

---

## P0 — Prerequisites

Blocking. Nothing downstream is correct without these.

### P0.1 Carry topology through the parser

```cpp
struct Road {
    WayId osm_id = 0;
    std::vector<glm::dvec2> polyline;
    std::vector<NodeId>     node_ids;      // NEW: parallel to polyline
    RoadType type = RoadType::Unknown;
    float width = 6.0f;
    int   lanes = 2;
    int   lanes_forward  = -1;             // NEW: -1 = unspecified
    int   lanes_backward = -1;             // NEW
    int   layer = 0;                       // NEW: layer=* for bridge/tunnel separation
    bool  is_roundabout = false;           // NEW: junction=roundabout|circular
    bool  is_link = false;                 // NEW: *_link
    SidewalkSide sidewalk = SidewalkSide::Unknown;  // NEW
    CyclewaySide cycleway = CyclewaySide::None;     // NEW
    std::string surface;                   // NEW
    // ... existing fields
};
```

`node_ids` is the load-bearing addition. Every junction decision derives from node
identity, not from proximity.

### P0.2 Move road geometry off the spatial index

Road geometry is built **once** against the global graph, then chunked. Leaves
reference ranges of the resulting mesh rather than each building their own.

Implementation:

- A new `RoadNetworkBuilder` runs after parse on the complete `ParsedOSMData`,
  produces global corridor submeshes, and chunks the output by triangle centroid into
  `QuadTreeNode::road_meshes`.
- `QuadTree::build_node_meshes_internal` (`quadtree.cpp:486`) stops calling
  `MeshBuilder::build_road_mesh` and `build_junction_meshes`. Buildings and areas keep
  their existing per-leaf path — they have no cross-leaf topology.
- **Delete `TileManager`** (`src/osm/tile_manager.hpp/.cpp`). It is unreferenced by the
  editor, carries the road-duplication bug, and would otherwise have to be ported
  alongside every change below.

### P0.3 Submesh support in `Mesh`

```cpp
enum class MaterialId : uint8_t {
    Asphalt, Concrete, Curb, Sidewalk, Markings, Gravel, Dirt, Grass, BridgeDeck, Parapet
};

struct SubMesh {
    uint32_t   index_offset = 0;
    uint32_t   index_count  = 0;
    MaterialId material = MaterialId::Asphalt;
};
```

`Mesh` gains `std::vector<SubMesh> submeshes`. Empty vector means one implicit
submesh over all indices, so existing callers keep working. `GPURenderer` draws per
range. This is what gives an exported mesh real material slots.

---

## P1 — RoadGraph

Build a proper network graph before any geometry exists.

```cpp
struct GraphNode {
    NodeId osm_id;
    glm::dvec2 position;
    int layer = 0;
    std::vector<HalfEdgeId> arms;   // sorted by outgoing bearing
    bool has_signals = false;       // highway=traffic_signals
    bool has_crossing = false;      // highway=crossing
    bool is_turning_circle = false;
};

struct GraphEdge {
    WayId source_way;
    NodeId from, to;
    std::vector<glm::dvec2> polyline;
    RoadProfile profile;
    int layer = 0;
    double trim_from = 0.0, trim_to = 0.0;   // filled by P4
};
```

Construction:

1. Count `NodeId` references across all roads. A node referenced by two or more ways,
   or appearing as a way endpoint, becomes a `GraphNode`.
2. Split every way at its graph nodes into `GraphEdge`s.
3. Sort each node's arms by outgoing bearing.
4. **Layer filter.** A bridge crossing a road shares a node in some OSM extracts but is
   not a junction. Arms with differing `layer` do not participate in the same junction.
   Split such nodes into per-layer nodes.
5. Tag roundabout loops (`junction=roundabout`) as closed edge cycles.

Degree classification drives the rest: 1 = dead end, 2 = continuation or profile
transition, 3+ = junction.

---

## P2 — Cross-section profiles and corridor extrusion

This is where the first large visual improvement lands. Curbs and sidewalks appear.

### Profile model

```cpp
enum class StripKind : uint8_t {
    Lane, Gutter, CurbTop, CurbFace, Sidewalk, Shoulder, Median, Verge, CycleLane, Parking
};

struct Strip {
    float      width;          // metres, lateral
    float      height_offset;  // metres, relative to carriageway surface
    MaterialId material;
    StripKind  kind;
};

struct RoadProfile {
    std::vector<Strip> strips;   // ordered left to right
    float total_width() const;
    float carriageway_width() const;
};
```

Examples:

- **Residential** — `sidewalk 2.0 @ +0.15`, `curb face 0.15`, `gutter 0.3`,
  `lane 3.5`, `lane 3.5`, `gutter 0.3`, `curb face 0.15`, `sidewalk 2.0 @ +0.15`
- **Motorway** — `shoulder 2.5`, `lane 3.75 x N`, `median 3.0`, `lane 3.75 x N`,
  `shoulder 2.5`. No sidewalk, no curb.
- **Rural track** — `verge 1.0`, `dirt 2.5`, `verge 1.0`. No curb.
- **Footway** — `sidewalk 2.0`. Nothing else.

Profiles are built from tags: `lanes`, `lanes:forward`, `lanes:backward`, `width`,
`sidewalk`, `cycleway:left/right/both`, `parking:lane:*`, `shoulder`, `surface`,
`median`, `oneway`.

**Adding a road feature is adding a strip rule, not writing new mesh code.** This is
the extensibility lever for the full urban/suburban/rural feature set.

### Centerline processing

1. **Resample.** OSM polylines are coarse and unevenly spaced. Fit a chordal
   arc-length Catmull-Rom (or biarc for tighter curvature control) and resample at
   curvature-adaptive spacing: dense in corners, sparse on straights.
2. **Correct miter.** Offset distance at a joint is `half_width / cos(theta/2)`, with a
   miter limit falling back to a bevel when the angle is too sharp. Fixes the pinch.
3. **Self-intersection guard.** When the inner offset folds back on a hairpin, clamp
   or collapse the fold.

### Extrusion and UVs

Walk the resampled centerline, emit one vertex column per strip boundary, weld shared
columns between adjacent strips, and triangulate along the ribbon.

UVs:

- **U** = lateral atlas coordinate for the strip, so each strip maps to an atlas row.
- **V** = accumulated arc length in metres divided by the material's tile length.

Vertical curb faces get their own UV treatment (`V` along the road, `U` up the face).

Output: one `Mesh` with `SubMesh` ranges per `MaterialId`, plus a **corridor polygon**
per edge (outer boundary of the full profile) for terrain carving.

---

## P3 — Terrain integration

Sequenced before junctions per the decision above.

### Pipeline change

The current order is `TerrainGenerator -> Heightmap -> TerrainMeshBuilder -> Mesh`.
It becomes:

```
TerrainGenerator -> Heightmap
                      |
                      v
              RoadElevation (sample + grade-limit)
                      |
                      v
              TerrainCarve (write back into Heightmap)
                      |
                      v
              TerrainMeshBuilder -> Mesh
```

Roads read the heightmap, then write back to it. `TerrainMeshBuilder` must run after
the carve, so terrain mesh construction moves downstream of road elevation solving.

### Road elevation solve

1. Sample the heightmap at each resampled centerline station.
2. **Grade limit.** Enforce a maximum longitudinal gradient per road class
   (motorway 4%, primary 6%, residential 8%, service 10%, path 15%). Solve as a
   monotone smoothing pass over the station heights, iterated to convergence. Without
   this, roads follow terrain noise and look like rollercoasters.
3. **Vertical curvature limit.** Clamp the second derivative so crests and sags are
   drivable.
4. **Junction height agreement.** All arms meeting at a graph node must terminate at a
   single node height. Solve node heights first, then interpolate arms between them.
   This is why elevation must be graph-aware and cannot be done per-way.
5. **Layer overrides.** `bridge` edges lift to a deck height above the terrain and
   ignore the underlying profile. `tunnel` edges drop below and suppress carving.

### Carve

For each heightmap sample inside a corridor polygon plus a falloff band:

- Inside the corridor: set terrain height to the road surface height.
- In the falloff band: blend from road height to natural terrain over a
  slope-limited distance, producing cut and fill embankments.

Pass 1 uses the P2 ribbon corridor plus the provisional junction hull. Pass 2 in P4
re-carves the junction neighbourhoods against the real fillet polygons.

---

## P4 — Junction solver

The hard part. Analytic trim and fillet, per the decision above.

For each `GraphNode` with degree >= 3:

1. **Sort arms by bearing.** Already done in P1.
2. **Compute the trim station per arm.** For each adjacent arm pair, intersect the
   near-side offset lines of the two arms. The arm's trim distance is the maximum over
   its two neighbours, plus a small clearance. Trim the ribbon back to that station.
   This is what stops ribbons overlapping inside the intersection.
3. **Build the junction polygon.** Chain the trimmed arm end cross-sections, joined by
   **corner fillet arcs** between consecutive arms. Fillet radius derives from the two
   arm widths and the turn angle. The fillet is what makes an intersection read as an
   intersection instead of a disc.
4. **Triangulate** with earcut, at the node height solved in P3.
5. **Curb ring.** Clipper2 `InflatePaths` inward with `JoinType::Round` produces the
   sidewalk corner ring and curb face around the junction, with gaps where a crossing
   or driveway requires a dropped kerb.
6. **Re-carve terrain** in the junction neighbourhood against the real polygon.

Special cases:

- **Roundabouts.** `junction=roundabout` closed cycles become an annulus: outer
  carriageway ring, inner island, splitter islands on approach arms, yield lines.
- **Degree-2 profile transitions.** Two edges meeting with different profiles get a
  taper over a class-dependent length rather than a hard discontinuity.
- **Motorway links.** `*_link` arms merge with a gore taper and a painted nose, not a
  square junction.
- **Dead ends.** Degree-1 nodes get a cap. `highway=turning_circle` gets a disc,
  otherwise a flat cap or a bulb for residential cul-de-sacs.

---

## P5 — Detail features

- **Pedestrian crossings.** Zebra stripe quads at `highway=crossing` nodes, aligned to
  the road cross-axis, inset inside the junction polygon, emitted to the `Markings`
  submesh. Dropped kerbs punched into the curb ring at the same locations.
- **Lane markings.** Derived from the P2 strip columns: dashed centre line, solid edge
  lines, double-solid on no-overtaking, stop lines at signalled arms, give-way
  triangles, turn arrows from `turn:lanes`, box junctions. All into `Markings`, so the
  consuming engine can put them in a decal atlas.
- **Sidewalk deduplication.** OSM represents sidewalks two ways: as `sidewalk=*` tags
  on the carriageway, and as separate `highway=footway` + `footway=sidewalk` ways.
  Both are frequently present in the same extract. Detect separately-mapped sidewalks
  running parallel and within range of a carriageway and suppress the synthesised one,
  otherwise emit from tags.
- **Service roads.** Driveways (`service=driveway`), parking aisles
  (`service=parking_aisle`), and alleys get narrow profiles with no curb, and drop the
  kerb where they meet the parent road.
- **Rural detail.** Verge strips, optional ditch profile, `surface=unpaved` and
  `surface=gravel` material assignment, cattle grids at `barrier=*` nodes.
- **Traffic islands and medians.** `Median` strips wider than a threshold become raised
  islands with their own curb faces rather than painted strips.

---

## P6 — Bridges and tunnels

- **Bridges.** `bridge=*` edges get a deck slab with thickness, parapets or railings
  along both edges from the profile outer boundary, and pier stubs dropped to terrain
  at an interval. Approach embankments come from the P3 carve.
- **Tunnels.** `tunnel=*` edges suppress surface geometry and carve a portal opening in
  the terrain at each end. Interior geometry is optional and out of scope for the first
  pass.
- **Layer correctness.** Already handled at P1, but this is where it becomes visible:
  a road under a bridge must not merge with it.

---

## P7 — Game-ready output

- **Material slots.** Already produced from P2 onward as `SubMesh` ranges.
- **Vertex welding.** Deduplicate across edge and junction boundaries so normals are
  continuous and the exported mesh is manifold where it should be.
- **`meshoptimizer` pass.** `meshopt_optimizeVertexCache`, `meshopt_optimizeOverdraw`,
  `meshopt_optimizeVertexFetch`, then `meshopt_simplify` for an LOD chain. The
  dependency is already vendored and linked.
- **Collision mesh.** A flat corridor variant with curb detail and markings stripped,
  emitted alongside the render mesh.
- **Chunked export.** Replace the per-tile duplication removed in P0 with a clip-once,
  assign-once chunking of the global mesh.

---

## Verification

Road geometry is pure math in `stratum_core` with no SDL dependency, so it is directly
testable. This is the only way to catch regressions without launching the GUI.

Enable `STRATUM_BUILD_TESTS` and add `tests/road/`:

| Test area | Checks |
|---|---|
| Miter | Offset distance equals `half_width / cos(theta/2)`; bevel fallback triggers past the miter limit; no pinch at corners |
| Profile | Total width matches tag-derived expectation across the profile table; strip ordering is symmetric where it should be |
| Graph | T-junction on an interior shared node produces degree 3; differing `layer` does not merge; roundabout cycles close |
| Trim | Trimmed arm ribbons do not overlap the junction polygon; trim distance is stable under arm reordering |
| Topology | Winding consistency, no degenerate triangles, no duplicate vertices after weld, manifold edges where expected |
| Elevation | Grade limit respected per class; all arms agree at node height; monotone smoothing converges |
| Golden files | Fixed small `.osm` extracts, hashed mesh output, diffed against committed goldens |

Golden extracts to commit: a four-way signalled urban junction, a suburban cul-de-sac,
a roundabout, a motorway link merge, an unpaved rural track, and a bridge over a road.

Complementary but not a substitute: an im3d debug overlay drawing graph nodes, arm
bearings, trim stations, corridor polygons, and junction polygons in the editor.

---

## Sequencing

| Phase | Depends on | Delivers |
|---|---|---|
| P0 | — | Topology in `Road`, no per-tile duplication, submesh support |
| P1 | P0 | `RoadGraph` with correct junction detection |
| P2 | P1 | Profiles, curbs, sidewalks, correct miters, arc-length UVs, material slots |
| P3 | P2 | Roads follow and carve terrain |
| P4 | P3 | Solved intersections, fillets, curb rings, roundabouts, links |
| P5 | P4 | Crossings, markings, service roads, rural and median detail |
| P6 | P4 | Bridges, tunnels, layer separation made visible |
| P7 | P5, P6 | Welding, LODs, collision, chunked export |

P0 through P2 are the ones that change the current output from unusable to usable.
P4 is the largest single body of work.

## UV Convention

Fixed before P2 so the extruder and any future material set agree.

**No atlas for surfaces.** P0.3 emits one `SubMesh` per `MaterialId`, so the consuming engine binds
a distinct material per range. Surface strips therefore use plain tiling UVs in metres, not atlas
sub-rects:

```
U = lateral_metres_from_strip_start / tile_u_metres(material)
V = arc_length_metres_along_road    / tile_v_metres(material)
```

`tile_*_metres` is a per-material constant, so texel density is uniform across the whole network
regardless of road width or segment length. This is what the current per-quad `0..1` mapping gets
wrong.

| Material | tile U (m) | tile V (m) | Note |
|---|---|---|---|
| Asphalt | 8.0 | 8.0 | Carriageway, junction fill |
| Concrete | 4.0 | 4.0 | Concrete carriageway, bridge deck |
| Sidewalk | 2.0 | 2.0 | Paving unit scale |
| Curb | 0.5 | 2.0 | See below |
| Gravel, Dirt, Grass | 4.0 | 4.0 | Unpaved surfaces and verges |

**Curb faces are the exception.** A curb face is vertical, so its `U` runs *up* the face rather than
laterally: `U = height_up_face / 0.5`, `V = arc_length / 2.0`. The curb top strip uses the normal
lateral convention. Both share the `Curb` material and therefore the same texture, which must be
authored with the face on one side and the top on the other.

**Markings are the only atlas.** `MaterialId::Markings` geometry is emitted as quads carrying
explicit atlas sub-rect UVs, because dashes, arrows, stop lines and zebra stripes are discrete
sprites rather than tiling patterns. P5 defines the sprite table. Marking quads are separate
geometry sitting just above the surface, so they never share vertices with the carriageway.

**Continuity across junctions.** `V` accumulates along an edge from its `from` node. At a junction
the arc-length restarts, which is correct — the junction polygon is planar-projected in its own
local frame rather than continuing the ribbon parameterisation, since it has no single direction of
travel.

## Open Items

- Dual carriageway merging (two OSM ways representing one road) is not addressed. It
  affects junction quality on primary roads and is deliberately deferred.
- Traffic signal and sign props are out of scope; the graph carries the tags so they
  can be placed later.
- Dual carriageway merging remains the largest known quality gap. See above.

---

## Implementation Status

Last updated 2026-08-24. Branch `feat/road-optimization`. Build green.

- `./build/bin/stratum_tests` — **423 passed / 0 failed**.
- `./build/bin/stratum_gpu_tests` — 16 passed / 0 failed (10 `GPUBufferPool`, 6 `GPUUploadBatch`).
- `ctest --test-dir build` — **36/36 suites passed**.

| Phase | State |
|---|---|
| P0 Prerequisites | **Done.** Reviewed, 3 confirmed defects fixed. |
| P1 RoadGraph | **Done.** Reviewed, defects fixed. |
| P2 Profiles + corridor | **Done.** Reviewed, 8 confirmed defects fixed. |
| P3 Terrain | **Done.** Reviewed; solver, carve and pipeline defects fixed. |
| P4 Junction solver | **Done.** Trim, fillet, junction polygon, curb ring, roundabouts, links, dead ends. |
| P5 Detail features | **Done.** Crossings, markings, sidewalk dedup, service roads, medians. |
| P6 Bridges / tunnels | **Done.** Decks, parapets, piers, portals, layer separation. |
| P7 Game-ready output | **Done.** Weld, meshoptimizer reorder + LOD chain, collision surface, chunked export. |
| GPU memory (not in the original plan) | **Done.** Pooled device buffers, batched uploads, resident budget with eviction. |
| P8 Geometry reduction (not in the original plan) | **Done.** Station decimation, coplanar quad merge, chunk LOD chains, tag-to-variant material table. See below. |

### What P7 delivers

- **Welding** (`mesh_optimize.*`) preserves creases, SubMesh ranges and material
  boundaries; simplification runs per material range so a kerb never collapses into
  the carriageway.
- **LOD chain** per piece from `meshopt_simplify`, borders locked.
- **Collision surface** (`collision_mesh.*`) derived from the finished render mesh by
  deletion: paint gone, kerb faces turned into bridged steps, real walls kept.
- **Chunked export** (`road_export.*`) to OBJ and glTF, clip-once/assign-once, with
  collision and LOD levels alongside.

### GPU memory work (outside the plan, required by the Lucan-scale load)

- `renderer/gpu_buffer_pool.*` — meshes no longer own device buffers. Both ranges are
  suballocated from one pool per usage, because thousands of meshes meant thousands of
  device allocations against `maxMemoryAllocationCount`.
- Batched uploads — `upload_mesh()` stages bytes CPU-side and
  `GPURenderer::flush_pending_uploads()` moves a budgeted prefix through ONE command
  buffer, ONE copy pass and ONE submit at the top of `begin_frame()`. This replaced a
  submit per mesh issued from inside an open render pass, which killed the driver on
  `vkCreateFence VK_ERROR_OUT_OF_HOST_MEMORY`.
- Resident budget with distance-ordered eviction (`MemoryBudget`, `evict_to_fit`,
  `on_mesh_evicted`), exposed in the editor.

### Defects fixed in the final review pass

Eight findings survived adversarial refutation and are fixed, each with a regression
test that was verified to fail with its fix reverted.

| Where | Defect |
|---|---|
| `collision_mesh.cpp` | Simplification deleted real walls. The hole guard measures PLAN area, and a tunnel headwall covers no plan, so `meshopt_simplify` removed all 64 wall triangles of a portal piece and the guard accepted it. Walls are now held out of the simplification and concatenated back. |
| `collision_mesh.cpp` | The step/wall test was per connected PATCH, so a bridge deck's end cap promoted every kerb on the deck to a wall. It is now per triangle, over the windowed extents at its own three vertices. |
| `road_export.cpp` | `snprintf`'s return value was used as a byte count against a 256-byte stack buffer, so a long file stem or `material_prefix` read off the end of the stack into the file. The unbounded lines are built as `std::string`; the rest are clamped. |
| `gpu_renderer.cpp` | `flush_pending_uploads()` assumed the staging arena was contiguous over the queue, but `release_mesh()` leaves a hole. Every entry after the hole was read past the end of the transfer buffer — an out-of-range copy, or silently stale geometry on a mesh marked ready. The transfer layout is now built by `plan_upload_batch()`. |
| `gpu_renderer.cpp` | The arena was erased and `shrink_to_fit()`-ed every flush: a full memcpy of the backlog per frame and a doubled host-memory peak. Compaction is now amortised (`staging_compaction_offset()`) and never reallocates. |
| `road_elevation.cpp` | A tunnel mapped as two ways charged its portal's unsatisfiable depth demand to the shared interior node on every override pass, burying it `kOverridePasses x tunnel_depth` down and dragging the surface approaches with it. A half-ramped edge is now measured at its pinned end's own station. |
| `road_network_builder.cpp` | The two junction plateaus were applied in sequence, so on a short edge the second overwrote the first and one arm mouth sat a full node-height difference off its junction plane. `apply_junction_plateaus()` now shares the disputed stations by arclength. |

### Open

Carried forward from the plan:

- **Dual carriageway merging** — two OSM ways representing one road are still two roads.
  The largest known quality gap, and it shows on primary-road junctions.
- **Traffic signal and sign props** — out of scope. The graph carries the tags.
- **The markings texture** — `MaterialId::Markings` geometry carries atlas sub-rect UVs
  and the sprite table is fixed, but no atlas image is authored. Nothing renders paint
  correctly until it is.

Known and deliberate, from this phase:

- **A collision surface's walls are never simplified.** Holding them out is what keeps
  a headwall in the mesh, but it also means a piece that is mostly parapet barely
  reduces. Simplifying walls with a wall-aware error metric is the better answer and is
  not written.
- **A very short edge cannot put both arm mouths on their own junction plane.** There is
  no station left between the two trims to hold a grade, so the plateau shares the
  error instead of favouring one end. Fixing it properly means resampling the edge
  against its trims rather than against `max_spacing`.
- **A half-ramped bore's mid-span depth comes from the ramp's own share weighting**, not
  from the override loop, so a ridge in the middle of one half of a split tunnel can end
  up shallower than `tunnel_depth`. Correct at the portals and at the interior node.
- **`upload_mesh()` has no per-frame staging cap.** Only the flush is budgeted, so one
  frame's traversal can stage far more than `kMaxUploadBytesPerFrame`; the ceiling is
  `MemoryBudget::max_resident_bytes`. The arena is now bounded at twice its live bytes,
  but the host-side peak during a large import is still set by the budget.
- **Building and area pads still flatten to `TerrainTileConfig::osm_base_height`** while
  roads sit at solved elevations. A pad at 0 beside a road at 40 m reads as a cliff. The
  crude pad path needs the treatment roads got in P3; no phase of this plan covers it.
- **Erosion is not seen by the grade solve.** `sample_surface()` has no closed form for
  thermal erosion. Mitigated by ordering — the carve runs after erosion — and currently
  vacuous, because `generate_chunk()` never applies erosion and only `generate()` does.
- **Bridges do not carve.** `CarveRibbon::suppress` is set for tunnels and bridge spans,
  so a deck floats rather than trenching the terrain beneath it. Approach embankments
  come from the adjacent non-bridge edges. Deliberate, matches the header contract,
  worth confirming against the intended look.

## P8 Geometry reduction and chunk LOD

Branch `feat/road-optimization`, on top of P7. Two new modules and one new
quadtree responsibility, plus the tag-to-appearance table the renderer binds
against.

| Module | What it does |
|---|---|
| `osm/road/tessellation.*` | Longitudinal station decimation (`select_stations` + `apply_station_selection`) and the lateral coplanar-quad merge (`merge_coplanar_quads`) |
| `osm/road/lod_chunk.*` | `build_chunk_lod()` — merges a leaf's road meshes, welds them, and simplifies with the chunk border locked so neighbouring chunks cannot crack |
| `osm/road/road_style.*` | The ONE table from OSM tags to `MaterialKey{slot, variant}`; `material_key_name()` is what an export writes |
| `osm/quadtree.cpp` | Triangle-centroid routing when chunk LOD is on, `measure_seam_bands()`, `build_chunk_lods()`, per-leaf level residency |

### Measured on the real extract

`/home/sarah/Downloads/lucan.osm` — 65 MB, 10,326 roads, 24,576 buildings,
2,122 areas; graph 18,443 nodes / 24,906 edges / 12,543 junctions. Default
`RoadNetworkConfig`, curb rings on, Release build, 16 threads.

| | `reduce_tessellation = false` | `reduce_tessellation = true` (default) |
|---|---|---|
| Pieces | 40,604 | 40,604 |
| Vertices | 4,013,865 | **2,766,999** (−31.1%) |
| Triangles | 3,417,509 | **2,190,467** (−35.9%) |
| Stations extruded | 316,518 | **184,108** (−41.8%) |
| Quads merged laterally | 0 | 204,683 |
| `build()` | 336 ms | 371 ms |

The two passes cost about 35 ms on this extract and remove a third of the
geometry. Splitting them: the longitudinal pass takes the corridor to 2,599,833
triangles, and the lateral merge takes that to 2,190,467.

Chunk LOD over the same network — 1,489 road-bearing leaves, 311 ms, widest
measured seam band 78.72 m:

| Level | Triangles | Vertices | Chunks |
|---|---|---|---|
| 0 | 2,190,467 | 2,817,466 | 1,489 |
| 1 | 1,155,433 | 1,621,876 | 1,127 |
| 2 | 271,973 | 399,813 | 390 |
| 3 | 13,528 | 20,383 | 40 |

Host memory high-water across `assign_road_pieces()`: **1,089 MB before the
per-chunk free, 1,078 MB after**. See the note under "Known and deliberate" for
why that is 11 MB and not the 194 MB the arithmetic suggests.

### Defects fixed in the P8 review pass

Seven findings survived adversarial refutation and are fixed, each with a
regression test verified to fail with its fix reverted.

| Where | Defect |
|---|---|
| `tessellation.cpp` | `apply_station_selection()` rebuilt the FIRST and LAST station's frame from the decimated chord and charged the absorbed runs' fold bounds to them. Those two stations are the ones the junction solver already consumed — `arm_end()` slices the same centerline at the same trim and offsets `stations.front()` to place the arm mouth and the curb ring's arm reach — so a decimated arm's ribbon stopped registering with its junction mouth. Because a trim almost never lands on an existing station, `slice()` synthesises that station with an INTERPOLATED miter frame while leaving the tangent equal to the band chord, so the rewrite rotated the normal (2.3 degrees on a 25 m kerb line, 0.25 m at a 7 m offset) with nothing else flagging it, and the fold tightening could clamp the same column metres further inboard. Both ends are now carried verbatim; interior stations still get the corrected bisector. |
| `quadtree.cpp` | The visible-leaf gate still asked `feature_count() > 0 \|\| !road_meshes.empty()`, and `build_chunk_lods()` CLEARS `road_meshes`. A leaf holding no feature centroid of its own — ordinary under triangle-centroid routing — was therefore never traversed, never uploaded and simply absent from the viewport with chunk LOD on. 12 leaves carrying 3,975 triangles on the Lucan extract. `has_road_lod()` is now the third arm of the gate. |
| `quadtree.cpp` | `build_chunk_lods()` held every chunk's input AND its whole chain until the parallel sweep finished. Each worker owns its leaf exclusively and `measure_seam_bands()` has already read every leaf, so the input is released inside the worker now. |
| `junction_polygon.cpp` | The adjacent-face clipping's inversion guard collapses an over-clipped arm onto its own midpoint but leaves its two NEIGHBOURS holding the crossing points, so a trident node comes back with a ring that is SIMPLE and CLOCKWISE. `ring_self_intersects()` correctly did not flag it, so the convex-hull fallback stopped firing and a backwards sliver became the asphalt fill and the terrain carve's winding test. `JunctionPolygon::inverted` reports it and every consumer asks `needs_hull_fallback()`. |
| `junction_trim.cpp` / `junction_builder.cpp` / `road_network_builder.cpp` / `crossings.cpp` | A near-coincident cluster's arms all go to the primary and every other member is solved away, and NOTHING recorded which junction absorbed which node. Every consumer keyed on a node id then dropped its feature silently: an approach to an absorbed member got no stop line and no give-way bar, and a junction crossing on it demanded a kerb drop from a ring that does not exist while the corridor still laid its own drop up to the trim station — the full-curb step `corridor_kerb_drops()` exists to remove. `collect_arms()` now fills an optional cluster out-parameter on every path, `JunctionBuilder::junction_owner()` publishes the node-to-primary map, and `dropped_kerb_spans()` takes the nodes a ring covers. Fires about 45 times per Dublin-sized build. |
| `road_profile.cpp` | The profile builder kept its OWN `surface=*` table and it disagreed with `road_style.cpp`'s on every modular running surface — cobblestone, sett, paving_stones, bricks went to `MaterialId::Sidewalk` here and `MaterialId::Asphalt` there. `strip_material()` may never change the slot it is handed, so a cobbled LANE resolved to the footway's brick variant, byte-identical to the pavement beside it; `sort_submeshes_by_material()` merged carriageway and footway into one draw range, `variants::kCobblestone`, `kSett` and `kPavingStones` were unreachable from any way in any extract, and the smoothness refinement was suppressed wherever the slots differed. There is one table now. |
| `road_export.cpp` | The exporter was variant-blind: triangles were bucketed by SLOT and `finish_chunk()` merged the ranges before any writer ran, so `{Asphalt, 0}` and `{Asphalt, kAsphaltWorn}` came out as one `usemtl`, one glTF material and one `newmtl`. `road_style.hpp` already documented `material_key_name()` as the name that travels in exported files and nothing in `src/` called it. Export is keyed on `MaterialKey::packed()` now. |

Also cleaned up: `MaterialId::Wall` and `MaterialId::Roof` left three switches
non-exhaustive. `tests/obj_dump.cpp`, `road_export.cpp` and `corridor.cpp` have
their arms back, so a new slot is a warning rather than a silent grey fallback.

### Known and deliberate, from P8

- **A bend absorbed into an edge's FIRST or LAST band is not represented in that
  band's end frame or fold bound.** Freezing the two end stations is what keeps
  the ribbon registered with its junction mouth, and the alternative is a visible
  wedge at every decimated arm. The band is inside the deviation budget by
  construction.
- **The eager per-chunk free is worth 11 MB, not 194 MB.** The chain is larger
  than the input it replaces, so glibc reuses the freed blocks for the next
  chunk's levels whether the free happens in the worker or in the serial pass;
  what the eager free removes is only the tail overlap. Kept because the running
  total is now monotonic and the window where both live is one chunk wide.
- **`build_junction_polygon()` still produces an inverted ring on a trident
  node.** The fix restores the hull fallback and the disc carve for it; it does
  not repair the ring. Propagating the collapse to an inverted arm's two
  neighbours is the real repair and is not written. Measured on the Lucan
  extract, no real junction reaches it — 0 negative-area rings across 12,498
  solved junctions.
- **An absorbed cluster member's approach is painted against the PRIMARY's
  plane.** That is right for the paint and still leaves the documented node-height
  step of up to `radius * max_grade` at its arm mouth, which is a separate,
  already-known consequence of the merge.
- **`surface=rock`, `stone`, `chippings`, `shells`, `clay`, `woodchips`, `metal`
  and `wood` now take the road class default slot** rather than the profile
  builder's old private mapping, because `road_style.cpp`'s table deliberately
  holds only running surfaces and `test_road_style.cpp` pins the
  unrecognised-falls-back-to-the-class contract. `cement`, `bitmac`, `tarmac` and
  `asphalt:lanes` were added to that table so nothing else was lost.
- **A surveyed footway keeps the `Sidewalk` slot whatever it is paved with.**
  `road_style.cpp`'s slot policy is a carriageway's — `paving_stones` is
  driveable, so Asphalt — and a footway is not a carriageway. Moving it would take
  its collision classification, its 2 m UV tiling and every `kSidewalk*` variant
  with it, and `strip_material()` cannot put it back.

### Working tree

Everything is uncommitted on `feat/road-optimization`: `src/osm/road/`,
`src/procgen/terrain_carve.*`, `src/renderer/gpu_buffer_pool.*` and `tests/` are new,
alongside modifications to the parser, quadtree, renderer, terrain and editor.
`src/osm/tile_manager.*` is staged for deletion.

`src/renderer/mesh.hpp` carries `SubMesh::variant` and `MaterialKey`, and this
branch appends `MaterialId::Wall` and `MaterialId::Roof` before the `Count`
sentinel. `feat/road-materials` owns the renderer side of the same contract and
will conflict on that file.
