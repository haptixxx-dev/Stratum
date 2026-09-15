# Core and OSM subsystems

## How the pieces fit

## src/core

**Misleading name.** `src/core` is NOT part of the `stratum_core` static library. It compiles into `stratum_editor_lib` (`CMakeLists.txt:172-175`). `stratum_core` (declared at `CMakeLists.txt:70`) contains `src/osm`, `src/osm/road`, `src/geometry` and `src/procgen` only. Nothing else in `src/core` exists — the directory is just two classes:

- `stratum::Application` (`src/core/application.hpp`) — owns the main loop. Members: `Window m_window`, `GPURenderer m_gpu_renderer`, `Editor m_editor`. `init()` does SDL init → window → ImGui; `run()` blocks on process_events/update/render; `shutdown()` tears down in reverse. Non-copyable. The destructor does NOT call `shutdown()` — callers must call it explicitly.
- `stratum::Window` (`src/core/window.hpp`) — SDL3 window wrapper with HiDPI scale (`get_scale()`), configured by `WindowConfig` (1280x800 resizable default). Holds the window handle only; frame/GPU device management lives in `GPURenderer`, not `Window`. Same explicit-`shutdown()` rule.

`application.hpp` includes `editor/editor.hpp` and `renderer/gpu_renderer.hpp`, so nothing under `src/core` is engine-agnostic. If you are adding engine-agnostic code, it belongs under `src/osm`, `src/geometry` or `src/procgen`, not here.

`stratum_core` links (PUBLIC): `glm`, `osmium`, `Clipper2`, `meshoptimizer` (`CMakeLists.txt:141-149`). It must never depend on SDL, ImGui or renderer code.

## src/osm

Purpose: parse OpenStreetMap files, project coordinates, hold the parsed feature model, build per-feature meshes, and spatially index everything for streaming. Namespace `stratum::osm`.

**Files (all verified):**
- `src/osm/types.hpp` — the whole data model, no .cpp. `OSMNode/OSMWay/OSMRelation` (raw), `Road/Building/Area` (processed), `ParsedOSMData` (container for both), `BoundingBox`, `CoordinateSystem`, enums `RoadType/BuildingType/AreaType/RoofType/SideFlags`.
- `src/osm/parser.hpp/.cpp` — `OSMParser`, libosmium-based. `parse(path)` accepts `.osm`, `.pbf`, `.osm.pbf`, `.osm.bz2`, `.osm.gz` (auto-detected by extension). Configured by `ParserConfig`; reports `ParseProgress` stages via callback. Get results with `get_data()` or `take_data()` (move).
- `src/osm/coordinates.hpp/.cpp` — `CoordinateConverter` plus a nested `stratum::osm::geometry` namespace of 2D polygon/polyline utilities (`polygon_area`, `ensure_ccw/ensure_cw`, `centroid`, Douglas-Peucker `simplify`, `point_to_line_distance`).
- `src/osm/mesh_builder.hpp/.cpp` — `MeshBuilder`: static `build_building_mesh(Building)`, `build_area_mesh(Area)`, `merge_meshes()`. **Roads are NOT built here** — road geometry is topology-driven, solved network-wide by `road::RoadNetworkBuilder` and handed to `QuadTree::assign_road_pieces()` as finished pieces. The old `build_road_mesh()`/`build_junction_meshes()` are gone (see `docs/plans/road_network_plan.md`, P0.2).
- `src/osm/quadtree.hpp/.cpp` — `QuadTree`/`QuadTreeNode` spatial index and streaming unit. Leaves hold features and merged meshes; `traverse_visible()` does frustum + distance + contribution culling; async per-leaf mesh builds via `queue_node_build_async()`/`poll_async_builds()`.

### The coordinate chain (verified in coordinates.cpp)

`WGS84 (lat, lon degrees) → Web Mercator EPSG:3857 (metres) → local metres (Mercator minus origin) → Y-up world space`

- `CoordinateConverter::wgs84_to_mercator(lat, lon)` — spherical Mercator, `EARTH_RADIUS_M = 6378137.0`; latitude clamped to ±85.051128°.
- `set_origin(lat, lon)` or `set_origin(BoundingBox)` stores the origin in Mercator; `wgs84_to_local()` returns `mercator - origin_mercator`. If `set_origin` was never called, `wgs84_to_local` silently falls back to raw Mercator (huge coordinates) — no assert.
- The 2D local plane is X-east / Y-north in `glm::dvec2` (doubles). The lift to 3D Y-up world space (local y → world −z or +z, height → world y) happens downstream in the mesh builders, not in `CoordinateConverter`.
- Convention trap: `BoundingBox::center()` and the WGS84-returning functions use `(lat, lon)` order in a `dvec2` — `.x` is latitude, `.y` is longitude.

### Invariants that break things if ignored

- **`Road::node_ids` is parallel to `Road::polyline`** — same size, `node_ids[i]` is the OSM node at `polyline[i]`. Junctions are found by shared node IDENTITY between ways, never by endpoint proximity. A road with empty `node_ids` is skipped by `RoadGraph::build()` entirely. `OSMParser::resolve_way_coords_with_ids()` guarantees the two vectors skip missing nodes together.
- **`Road::layer`** (OSM `layer=*`) splits graph nodes: two ways sharing a node but on different layers do NOT form a junction (bridge over road).
- **`SideFlags::Unknown` ≠ `SideFlags::None`** (sidewalk/cycleway/parking/shoulder). Unknown = tag absent, profile builder may infer a class-based default. None = tag explicitly negative (`sidewalk=no`), no default allowed. Left/Right are relative to way direction, not compass.
- **`Road` deliberately carries no `TagMap`.** Rare tags are looked up as `data.ways.at(road.osm_id).tags` (the way always exists for a processed road; use `find()` for the individual tag).
- **Winding:** building `footprint` outer ring is CCW, `holes` are CW (`ensure_ccw`/`ensure_cw`).
- **`OSMParser::recenter_on_features()`**: raw bounds are widened by every node Overpass drags in for ways crossing the query box — a Dublin export puts geometry ~480 km from the bounds centre, where float32 quantises to ~6 cm (z-fighting, camera jitter). The parser recentres geometry on the feature centre of mass after conversion.
- **Prefer `QuadTree::init(const ParsedOSMData&)` over `init(BoundingBox)`** for the same reason: bounds-based sizing pushes real geometry into a corner and exhausts `MAX_DEPTH` (8) before reaching useful leaf size. Likewise use `QuadTree::get_focus()` for camera framing, not the centre of `get_bounds()`.
- **`assign_road_pieces()` ordering:** call after `init()` and `assign_data()`, before any node mesh build is queued. It consumes (moves from) the pieces vector. `QuadTreeNode::road_meshes` is filled only by this hand-off; per-leaf builds never touch roads (`BuiltMeshes` has building/area meshes only).
- **Chunk LOD (`set_chunk_lod`) must be set BEFORE `assign_road_pieces()`** — it also switches routing: LOD off routes each piece whole by `RoadPiece::anchor`; LOD on routes per-TRIANGLE by centroid so the seam-band proof holds (see the long comment on `assign_road_pieces()` in `quadtree.hpp` — the crack-free argument via `measure_seam_bands()`). When a leaf's `ChunkLod` exists, `road_meshes` is cleared (`levels[0]` supersedes it) and only ONE LOD level is GPU-resident at a time (`road_lod_resident`, hysteresis `kRoadLodHysteresis = 0.15`).
- **Building ambient occlusion** is baked per leaf on the MERGED mesh (buildings occlude each other), via `stratum::geometry::AOSettings` (`QuadTree::set_building_ao`); `strength = 0` skips the bake.
- `stratum_core` uses the `Mesh`/`Vertex` types from `src/renderer/mesh.hpp` — that header is glm/STL-only (no SDL), so the include does not break engine-agnosticism, but do not add SDL-touching includes there.

## src/osm/road

## src/geometry

## src/procgen

## Gotchas
