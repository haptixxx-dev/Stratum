# ImportPipeline: design for Phase 3 of the UI restructure

**Status:** Proposed (design only; nothing in this note is implemented yet)
**Date:** 2026-09-27
**Decider:** Sarah
**Follows:** `docs/plans/ui-restructure-plan.md`, "Phase 3: pipeline extraction and tests"
**Line references:** `restructure/phase-2-model` at `3f311b0`. `ip.cpp:N` means
`src/editor/import_pipeline.cpp` line N. `editor.hpp:N` means `src/editor/editor.hpp` line N.

Phase 3 moves the import state machine out of `Editor` and into `stratum_core`, so the
same code runs in the editor, in `stratum_tests` and in the planned headless CLI (J3).
The rule for the move is the rule for the whole restructure: **zero behaviour change**.
The golden numbers (`ctest -L golden`, and `stratum_golden --osm ~/Downloads/lucan.osm
--json` compared with `tests/golden/lucan.json`) stay identical.

---

## 1. Public API: `stratum::app::ImportPipeline`

File: `src/app/import_pipeline.hpp` and `src/app/import_pipeline.cpp`, compiled into
`stratum_core`. The header includes only core headers: `osm/parser.hpp`,
`osm/quadtree.hpp`, `osm/road/road_network_builder.hpp`, `osm/road/road_export.hpp`,
`procgen/terrain_tile_manager.hpp`, `app/export_options.hpp`. No SDL, no ImGui, no
`renderer/gpu_*`, no `editor/*`.

### 1.1 Ownership

| Object | Owned or referenced | Why |
|---|---|---|
| `osm::OSMParser` (the committed parse) | **Owned** (`m_parser`) | Three workers read its `ParsedOSMData` by pointer. The rule "nothing may reassign or clear the parser while a future is valid" is only enforceable by the object that owns both. Callers get `const` access only. |
| `procgen::CarveConfig`, pending `CarveInput` | **Owned** | `Editor::m_carve_config` (`editor.hpp:931`) is never edited anywhere; it is pipeline data. |
| `osm::QuadTree` | **Referenced** | The viewport mutates leaves every frame (GPU flags, `queue_node_build_async`), and the procgen panel reads leaves. Owning it would route all of that through pipeline accessors for no gain. |
| `procgen::TerrainTileManager` | **Referenced** | The procgen panel drives its lifecycle (`init`, `generate_all_chunks`, `clear`). The pipeline only reads its config and installs or clears carve data. |

Consequence for `Editor`: `m_import_pipeline` is declared **after** `m_quadtree`
(`editor.hpp:432`) and after `m_terrain_tile_manager` (`editor.hpp:1084`), so the pipeline
is destroyed first. The listener adapter (section 2) is declared **before** the pipeline.

### 1.2 Types

```cpp
namespace stratum::app {

/// Identity of a terrain surface. Moved verbatim from Editor::terrain_surface_fingerprint.
/// Never returns 0; 0 is the "flat, no terrain" sentinel.
[[nodiscard]] uint64_t terrain_surface_fingerprint(const procgen::TerrainConfig& cfg);

enum class ImportStage {
    Idle,
    Parsing,         // OSMParser on worker 1
    BuildingRoads,   // RoadNetworkBuilder on worker 2, whole graph
    Indexing,        // QuadTree init/assign on the calling thread, inside one tick()
    BuildingMeshes,  // QuadTree's own async node builds, drained by tick()
    CarvingTerrain,  // CarveInput::build_index on worker 3, install one tick later
    Done,
    Failed,
    Cancelling,      // NEW: cancel() accepted, the discarded worker has not landed yet
    Cancelled,       // NEW: the discarded worker landed; previous data is intact
};

/// What Editor::m_model and Editor::m_use_chunked_terrain hold today, as one value.
/// Every field is read exactly where today's code reads the live model field.
struct RoadOptions {
    bool terrain_aware = true;        // EditorModel::m_terrain_aware_roads
    bool chunked_terrain = true;      // Editor::m_use_chunked_terrain
    bool solve_junctions = true;      // EditorModel::m_solve_junctions
    bool emit_markings = true;
    bool emit_crossings = true;
    bool emit_structures = true;
    bool reduce_tessellation = true;
    bool chunk_lod = true;            // read at Indexing, not at launch (as today)
};

struct ImportOptions {
    osm::ParserConfig parser;
    RoadOptions roads;
};

// ExportOptions moves from src/editor/export_options.hpp to src/app/export_options.hpp,
// unchanged (namespace stratum, plain data). EditorModel keeps including it.

enum class RebuildPolicy {
    Always,            // today's begin_road_network_rebuild()
    IfSurfaceChanged,  // today's maybe_rebuild_roads_for_terrain()
};

enum class RebuildOutcome { Started, Deferred, NotNeeded, NoRoads };

struct Launch {
    bool started = false;
    std::string reason;   // the exact refusal text logged today; empty when started
};

struct RoadBuildStats {          // was Editor::m_road_* (editor.hpp:944-964)
    bool have_stats = false;
    osm::road::RoadNetwork::Stats network{};
    osm::road::RoadElevationSolver::Stats elevation{};
    osm::road::JunctionBuilder::Stats junctions{};
    float max_grade = 0.0f;
    bool built_on_terrain = false;
    bool solved_junctions = false;
    bool emitted_markings = false;
    bool emitted_crossings = false;
    bool emitted_structures = false;
    size_t portal_mouths = 0;
    uint64_t terrain_fingerprint = 0; // surface the live network was solved against
};

struct QuadTreeSummary {         // what begin_mesh_rebuild logs at ip.cpp:776-781
    size_t leaf_count = 0, total_roads = 0, total_buildings = 0, total_areas = 0;
    uint8_t max_depth = 0;
};

struct PipelineState {
    ImportStage stage = ImportStage::Idle;
    float fraction = 0.0f;
    std::string message;
    bool rebuild_only = false;    // was m_road_rebuild_only
    bool rebuild_owed = false;    // was m_road_rebuild_owed
    size_t nodes_done = 0;        // BuildingMeshes progress
    size_t nodes_total = 0;       // was m_import_nodes_total
    bool export_running = false;  // was export_in_flight()
    RoadBuildStats roads;
    QuadTreeSummary quadtree;

    /// Parsing through CarvingTerrain, or Cancelling, or an export in flight.
    /// Replaces the five-way comparison at import_panel.cpp:162-167.
    [[nodiscard]] bool busy() const;
};

struct ExportResult {            // what poll_road_export builds at ip.cpp:91-131
    osm::road::ExportStats stats;
    std::string failure;          // exception text; empty on success
    std::string directory;
    bool wrote_collision = false;
    bool wrote_lods = false;
    std::string status;           // the exact line m_export_status gets today
};

/// Where the first node builds are queued from. Filled by the editor from its camera.
struct InitialView {
    std::array<glm::vec4, 6> frustum_planes;
    glm::vec3 camera_position;
    float view_radius;
    float fov_y;
    float contribution_threshold;
};

/// Test seam only. Each hook runs on the WORKER thread, first thing in the task.
/// Tests use it to hold a worker at a gate; production passes nothing.
struct WorkerHooks {
    std::function<void(ImportStage)> on_worker_start;   // Parsing, BuildingRoads, CarvingTerrain
    std::function<void()> on_export_worker_start;
};
```

### 1.3 The class

```cpp
class ImportPipeline {
public:
    struct Listener;   // section 2

    ImportPipeline(osm::QuadTree& quadtree,
                   procgen::TerrainTileManager& terrain,
                   WorkerHooks hooks = {});
    ~ImportPipeline();                 // joins every worker; see section 4
    ImportPipeline(const ImportPipeline&) = delete;
    ImportPipeline& operator=(const ImportPipeline&) = delete;

    void set_listener(Listener* listener);   // non-owning; may be null

    /// The latest road options. The editor calls this once per frame before tick(),
    /// and before any request or begin call, so every deferred read sees the live
    /// model exactly as today's code does.
    void set_road_options(const RoadOptions& options);

    Launch begin_import(const std::filesystem::path& path, const ImportOptions& options);
    RebuildOutcome request_road_rebuild(RebuildPolicy policy);
    Launch begin_export(const ExportOptions& options);

    /// Non-blocking. Advances every stage that can advance and returns. Never waits
    /// on a future (every wait is wait_for(0)). The only synchronous work it does is
    /// the work today's poll_osm_import() does on the main thread: the Indexing step
    /// and the carve install one tick after "Carving terrain...".
    void tick();

    /// True when accepted. See the table in 1.4.
    bool cancel();

    /// Drop all imported data. Refused (returns false) while state().busy().
    bool clear();

    [[nodiscard]] const PipelineState& state() const;
    [[nodiscard]] const osm::OSMParser& parser() const;   // rule, procgen, import panels
    [[nodiscard]] bool has_roads() const;                 // has_data() && !roads.empty()
    [[nodiscard]] uint64_t live_terrain_fingerprint() const;
};

} // namespace stratum::app
```

`begin_import` refusal reasons stay the three strings at `ip.cpp:343,349,354`, plus
"A cancelled import is still stopping" while `Cancelling`. `begin_export` refusal
reasons stay the four strings at `ip.cpp:27,31,35` and the in-flight early return at
`ip.cpp:23`; the editor copies `reason` into its own `m_export_status`.

`request_road_rebuild` returns `Deferred` (and sets `rebuild_owed`) whenever the old
guard at `ip.cpp:279-280` would have deferred, `NoRoads` for `ip.cpp:289-291` and
`ip.cpp:321-323`, `NotNeeded` for `ip.cpp:328-330`, and `Started` otherwise. It logs
the same lines as today.

### 1.4 `cancel()` semantics per stage

No core worker can be interrupted: `OSMParser::parse`, `RoadNetworkBuilder::build`,
`CarveInput::build_index` and `export_road_network` have no cancel hook. So a cancel is
a **discard**: the worker runs to its end, `tick()` collects the result with
`wait_for(0)` and throws it away. `cancel()` itself never blocks and never destroys a
future that is still running (that destructor would block).

| Stage when `cancel()` is called | Result | Why |
|---|---|---|
| `Parsing` | Accepted. Stage becomes `Cancelling`. When the parse lands, the job (and its own parser) is dropped. Stage becomes `Cancelled`. `m_parser`, the quadtree and the carve are untouched. | The parse job owns a separate parser (`editor.hpp:455-470`), so nothing committed has changed yet. |
| `BuildingRoads`, `rebuild_only == true` | Accepted. The result is dropped when it lands. `roads` stats, `terrain_fingerprint` and the quadtree are untouched. `rebuild_owed` is cleared. | The previous network is still in the quadtree and still matches `m_parser`. |
| `BuildingRoads`, fresh import | **Refused.** | `m_parser` already holds the NEW parse (`ip.cpp:413`) while the quadtree holds the OLD one. Discarding here would leave them inconsistent. A later phase can stage the new parser until Indexing; Phase 3 does not. |
| `Indexing`, `BuildingMeshes`, `CarvingTerrain` | **Refused.** | The quadtree is already replaced. Stopping leaves a half-built tree or a carve that does not match the roads. |
| Export in flight | **Refused.** | The worker writes files. A discarded result would hide a partial write. |
| `Idle`, `Done`, `Failed`, `Cancelled` | Refused (nothing to cancel). | |

Phase 3 adds no Cancel button. The import panel gets one in Phase 5, so the GUI
behaviour of Phase 3 is unchanged.

---

## 2. `Listener` and how the editor subscribes

```cpp
struct ImportPipeline::Listener {
    virtual ~Listener() = default;
    virtual void on_stage(ImportStage stage, float fraction, const std::string& message) {}
    virtual void on_log(const PipelineLogLine& line) {}   // {Info|Warn|Error, text}
    virtual void on_before_quadtree_reset() {}
    virtual void on_quadtree_ready(bool recenter_camera) {}
    virtual std::optional<InitialView> initial_view() { return std::nullopt; }
    virtual void on_carve_installed(bool carve_present) {}
    virtual void on_export_done(const ExportResult& result) {}
};
```

Rules:

- Every callback runs on the thread that called `tick()`, `begin_*`, `request_*`,
  `cancel()` or `clear()`. Never on a worker.
- A callback may read `state()`, `parser()` and the quadtree. It must not call
  `begin_*`, `request_*`, `cancel()`, `clear()` or `tick()`.
- `on_stage` fires once per stage change, including two changes inside one `tick()`
  (Indexing then BuildingMeshes; CarvingTerrain then Done when there is nothing to
  carve; Done then BuildingRoads when an owed rebuild starts).
- The pipeline keeps every `spdlog` call it makes today, with the same text, so the
  console (fed by `RingSinkMt`) and the log files do not change. `on_log` mirrors
  only those main-thread lines. The editor does **not** push them into `LogRing`
  (that would print each line twice); it only sets `m_console_scroll_to_bottom = true`,
  which is what every `ip.cpp` site that logs does today. The headless CLI prints them.
- `initial_view()` returning `nullopt` queues **every** leaf. That is the headless and
  test behaviour, and it matches Stage 4 of `tools/golden/stratum_golden.cpp:347-357`.

### Editor subscription

`Editor` gains a private nested `PipelineListener final : app::ImportPipeline::Listener`
holding `Editor&`, declared before `m_import_pipeline`, registered with
`set_listener()` in `Editor::init()`. Its handlers:

| Callback | Editor code that runs | Today at |
|---|---|---|
| `on_before_quadtree_reset` | SceneGpuSync: `if (m_gpu_renderer) for each leaf: release_node_from_gpu(*leaf, *m_gpu_renderer)` | `ip.cpp:756-760` and `ip.cpp:711-715` |
| `on_quadtree_ready(recenter)` | CameraFraming: `frame_camera_on_data(recenter)`, then set `m_model.m_use_tile_culling = true`, `m_use_distance_culling = true`, `m_use_contribution_culling = false` | `ip.cpp:783-788` |
| `initial_view()` | Fill `InitialView` from `m_camera.get_frustum().planes`, `m_camera.get_position()`, `m_model.m_view_radius`, `m_camera.m_fov`, `m_model.m_contribution_threshold` | `ip.cpp:795-800` |
| `on_log` | `m_console_scroll_to_bottom = true` | `ip.cpp:133,408,522,583,661,671` |
| `on_export_done(r)` | `m_export_status = r.status` | `ip.cpp:106-131` |
| `on_stage`, `on_carve_installed` | Nothing in Phase 3. Panels draw from `state()`. Terrain chunks re-upload through `TerrainChunk::gpu_uploaded`, which the regeneration already clears (`viewport_renderer.cpp:201-231`). | — |

`on_quadtree_ready` fires before `initial_view()` is asked, so the frustum is the
re-framed one, exactly as `frame_camera_on_data()` ends with `m_camera.update(1.0f)`
before the traversal today.

---

## 3. Mapping: existing members to pipeline members

### 3.1 Functions (`src/editor/import_pipeline.cpp`)

| Lines | Today | Becomes |
|---|---|---|
| 22-78 | `Editor::begin_road_export` | `ImportPipeline::begin_export(const ExportOptions&)`. 23-37 guards return `Launch{false, reason}`. 49-51 use the private `make_road_network_config()`. 68-73 is async site 4. 77 log kept. |
| 80-134 | `Editor::poll_road_export` | private `tick_export()`, called from `tick()`. 91-99 try/catch kept. 101-131 build `ExportResult` and fire `on_export_done`. 133 becomes `on_log`. |
| 140-155 | `hash_combine`, `hash_float` | anonymous namespace in `src/app/import_pipeline.cpp`, unchanged. |
| 157-180 | `Editor::terrain_surface_fingerprint` | free function `app::terrain_surface_fingerprint`. `procgen_panel.cpp:342` calls it. |
| 182-184 | `Editor::has_generated_terrain` | stays on `Editor` for `import_panel.cpp:73`; the pipeline has a private copy. |
| 186-190 | `Editor::live_road_terrain_fingerprint` | `ImportPipeline::live_terrain_fingerprint()`, from the stored `RoadOptions` and the referenced terrain manager. `import_panel.cpp:474` calls it. |
| 192-231 | `Editor::make_terrain_height_sampler` | private `make_height_sampler()`, unchanged body; `m_use_chunked_terrain` becomes `m_road_options.chunked_terrain`. |
| 233-242 | `Editor::make_road_network_config` | private `make_road_network_config()`, from `m_road_options`. |
| 244-276 | `Editor::run_road_network_build` | private static `run_road_build()`, unchanged. |
| 278-318 | `Editor::begin_road_network_rebuild` | `request_road_rebuild(RebuildPolicy::Always)` → private `launch_road_rebuild()`. 308-315 is async site 2 (rebuild launch). |
| 320-339 | `Editor::maybe_rebuild_roads_for_terrain` | `request_road_rebuild(RebuildPolicy::IfSurfaceChanged)`. |
| 341-384 | `Editor::begin_osm_import` | `begin_import(path, ImportOptions)`. Also stores `options.roads`. 365-368 progress callback and 372-374 async site 1 unchanged. |
| 386-444 | `poll_osm_import`, Stage 1 | `tick()` → private `tick_parse()`. 435-442 is async site 2 (import launch). |
| 446-545 | `poll_osm_import`, Stage 2 | private `tick_roads()`. 463-478 fill `state_.roads`. 480-521 logs kept. 527-531 carve handoff. 536-543 → `rebuild_index()` with `on_stage` for Indexing and BuildingMeshes. |
| 547-551 | `poll_osm_import`, carve branch | `tick()` dispatch to `tick_carve()`. |
| 553-589 | `poll_osm_import`, Stage 4 drain | private `tick_meshes()`. Also fills `nodes_done`. |
| 595-625 | `Editor::begin_road_carve` | private `begin_carve()`. 620-624 is async site 3. |
| 627-654 | `Editor::poll_road_carve` | private `tick_carve()`. The one-tick deferred apply (`m_carve_apply_pending`) is kept. |
| 656-679 | `Editor::install_road_carve_data` | private `install_carve()`; fires `on_carve_installed`. |
| 681-698 | `Editor::finish_osm_import` | private `finish()`; fires `on_stage(Done)`; the owed path calls `request_road_rebuild(IfSurfaceChanged)` as today. |
| 700-740 | `Editor::clear_imported_data` | `ImportPipeline::clear()`. 711-715 → `on_before_quadtree_reset`. 717-719 deleted (see below). |
| 742-813 | `Editor::begin_mesh_rebuild` | private `rebuild_index(pieces, recenter)`. 744-746 deleted. 756-760 → `on_before_quadtree_reset`. 761-781 kept, and fill `state_.quadtree`. 783-788 → `on_quadtree_ready`. 795-809 → `initial_view()` then `traverse_visible` with the same constants (600.0f, `true, true, false`), or every leaf when `nullopt`. 811-812 kept. |
| `camera_framing.cpp:10-75` | `Editor::frame_camera_on_data` | Stays in the editor. Called only from `on_quadtree_ready`. |

### 3.2 Members (`src/editor/editor.hpp`)

| Lines | Today | Becomes |
|---|---|---|
| 431 | `m_osm_parser` | `ImportPipeline::m_parser`; read through `parser()`. |
| 432 | `m_quadtree` | stays on `Editor`; passed by reference. |
| 435-437 | `m_building_meshes`, `m_road_meshes`, `m_area_meshes` | deleted. They are only ever cleared (`ip.cpp:717-719,744-746`); nothing reads them. |
| 444-453 | `ImportStage` | `app::ImportStage` (plus `Cancelling`, `Cancelled`). |
| 455-470, 472 | `OSMImportJob`, `m_import_job` | private in the pipeline, unchanged layout. |
| 489-533, 535 | `RoadBuildResult`, `m_road_build_future` | private in the pipeline. |
| 545, 557 | `m_road_rebuild_only`, `m_road_rebuild_owed` | pipeline; mirrored in `PipelineState`. |
| 559-563 | stage, message, fraction, pending nodes, node total | pipeline; `m_import_pending_nodes` stays private. |
| 572-573 | `m_import_status`, `m_import_error` | stay on `Editor` (panel-local). |
| 667-678 | `RoadExportJob`, `m_export_job` | private in the pipeline. |
| 681 | `m_export_status` | stays on `Editor`; written from `Launch::reason`, `on_export_done` and `file_dialogs.cpp:267`. |
| 684 | `export_in_flight()` | `state().export_running`. |
| 925, 928, 931, 941 | `m_pending_carve`, `m_carve_index_future`, `m_carve_config`, `m_carve_apply_pending` | pipeline. |
| 944-964 | `m_road_*` stats, `m_have_road_stats`, `m_road_terrain_fingerprint` | `PipelineState::roads`. |

### 3.3 Call sites in the editor

- `editor.cpp:254` `poll_osm_import()` → `m_import_pipeline.set_road_options(road_options()); m_import_pipeline.tick();`
- `editor.cpp:262` `poll_road_export()` → deleted; `tick()` drives the export. The export
  result now lands before `poll_file_dialog()` in the same frame instead of after it.
  Nothing reads both, so nothing observable changes.
- `import_panel.cpp:66`, `procgen_panel.cpp:22,166,383` → `Editor::request_road_rebuild(IfSurfaceChanged)`.
- `import_panel.cpp:85,98,107,115,129,141,489` → `Editor::request_road_rebuild(Always)`.
  `Editor::request_road_rebuild` is a two-line helper: `set_road_options(road_options())`,
  then forward. That keeps "the model changed this frame, the request sees it" true.
- `import_panel.cpp:177` → `begin_import(path, {config, road_options()})`.
- `import_panel.cpp:277` → `if (auto l = begin_export(m_model.m_export_options); !l.started) m_export_status = l.reason; else m_export_status = "Exporting...";`
- `import_panel.cpp:536` → `m_import_pipeline.clear()`.
- `rule_panel.cpp`, `procgen_panel.cpp:121,347`, `import_panel.cpp:236,300` → `parser()`.

---

## 4. Threading

Unchanged. The pipeline has the same four `std::async(std::launch::async, ...)` sites,
with the same captures:

| # | Site today | Captures | Reads while running |
|---|---|---|---|
| 1 | Parse, `ip.cpp:372` | `OSMImportJob*` | the job's own parser and path |
| 2 | Road build, `ip.cpp:435` (import) and `ip.cpp:308` (rebuild), one future member | `ParsedOSMData*` into `m_parser`, config by value (sampler owns its own `TerrainGenerator`), fingerprint by value | `m_parser` data |
| 3 | Carve index, `ip.cpp:620` | the `unique_ptr<CarveInput>`, moved | nothing shared |
| 4 | Export, `ip.cpp:68` | `ParsedOSMData*`, configs and directory by value | `m_parser` data |

The progress hand-off stays as it is: the parser's progress callback writes
`OSMImportJob::progress` under `OSMImportJob::mutex` on worker 1; `tick_parse()` reads it
under the same mutex. No other state crosses threads. QuadTree node builds keep their
own internal async path (`queue_node_build_async` / `poll_async_builds`), untouched.

**Futures are declared last.** Members destruct in reverse order, and `~future` of an
`std::async` future joins the worker. So:

- `OSMImportJob::future` stays the last member of `OSMImportJob`.
- `RoadExportJob::future` stays the last member of `RoadExportJob`.
- In `ImportPipeline`, the private section ends with, in this order:
  `m_import_job`, `m_export_job`, `m_road_build_future`, `m_carve_index_future`.
  Everything a worker can read (`m_parser` above all) is declared before them, so it
  is still alive while they join.
- In `Editor`, `m_import_pipeline` is declared after `m_quadtree` and
  `m_terrain_tile_manager`, and after the `PipelineListener` it points at.

Destroying a pipeline with a parse in flight still blocks until the parse ends. That
is today's behaviour, and the section 6 teardown test exercises it.

---

## 5. What stays in the editor

- **GPU residency (SceneGpuSync, `scene_gpu_sync.cpp`).** `upload_node_to_gpu`,
  `release_node_from_gpu`, `upload_tracked_mesh`, `release_tracked_mesh`,
  `m_mesh_owners`, eviction, `node_anchor`.
- **LOD residency.** `sync_node_road_lod`, `record_road_lod_residency`,
  `RoadLodFrameStats`, and terrain chunk upload and culling in `render_3d`.
- **Per-frame streaming.** The `traverse_visible` in `viewport_renderer.cpp:107` that
  queues node builds as the camera moves, and its `poll_async_builds`.
- **Camera (CameraFraming, `camera_framing.cpp`).** `frame_camera_on_data`, the
  culling flags it and `on_quadtree_ready` set, `m_view_radius`.
- **Options as data.** `EditorModel` keeps the road toggles; `Editor::road_options()`
  packs them with `m_use_chunked_terrain` into `app::RoadOptions`.
- **UI strings.** `m_import_status`, `m_import_error`, `m_export_status`, the file
  dialogs, and every ImGui call in the panels.
- **Terrain generation.** `generate_chunked_terrain`, `clear_chunked_terrain` and the
  legacy terrain path. They call `request_road_rebuild(IfSurfaceChanged)` where they
  call `maybe_rebuild_roads_for_terrain()` today.

---

## 6. Test plan: `tests/app/test_import_pipeline.cpp`

Suite `ImportPipeline` in `stratum_tests` (core only, no window, no GPU). Fixtures:
`tests/data/four_way.osm` and `tests/data/dual_carriageway.osm`. Golden values are read
from `tests/golden/<fixture>.json` with nlohmann (already a PUBLIC link of
`stratum_core`), through a new `STRATUM_TEST_GOLDEN_DIR` compile definition next to
`STRATUM_TEST_DATA_DIR` (`tests/CMakeLists.txt:291-292`). `spdlog` is set to `off` for
the suite, as `stratum_golden` does. A `RecordingListener` stores every `on_stage` call.

A helper `run_until(pipeline, predicate, timeout = 30 s)` calls `tick()` in a loop with a
1 ms `std::this_thread::sleep_for` between calls, and fails the test on timeout instead
of hanging.

| Test | Steps | Checks |
|---|---|---|
| `stage_order_<fixture>` (x2) | `begin_import`, run until `Done`. | Recorded stages, consecutive duplicates removed, equal `Parsing, BuildingRoads, Indexing, BuildingMeshes, CarvingTerrain, Done`. `fraction == 1`. Message "Import successful". |
| `counts_match_golden_<fixture>` (x2) | Same run, default `RoadOptions` (terrain-aware on, no terrain, so no sampler; chunk LOD on: the `stratum_golden` config). | `parser()` counts = `"parser"`. `state().roads.network` edges, pieces, vertices, triangles = `"road_network"`. `state().roads.junctions` = `"junction_stats"`. `state().quadtree` = `"quadtree"`. Leaf meshes summed as in `stratum_golden.cpp:361-368` = `"meshes"`. |
| `tick_never_blocks` | Gate each worker with `WorkerHooks` on a `std::promise`. For each of Parsing, BuildingRoads, CarvingTerrain (terrain present) and the export: while the gate is shut, call `tick()` 100 times. A watchdog thread opens the gate after 5 s and records a failure if it had to. | All 100 calls return while the gate is shut; stage unchanged; the watchdog did not fire. Then open the gate and run until `Done`. |
| `rebuild_request_during_import_is_honoured_after_done` | Import with no terrain; gate the road worker. While gated, `init` a one-chunk `TerrainTileManager` config and `generate_all_chunks()`; `request_road_rebuild(IfSurfaceChanged)`. Open the gate; run until `Done` with `rebuild_only == false`, then until the second `Done`. | Outcome `Deferred`, `rebuild_owed` true. After the first Done, a second `BuildingRoads` is recorded and `rebuild_only` true. Final message "Roads re-solved against the terrain". `roads.terrain_fingerprint == live_terrain_fingerprint() != 0`. `terrain.has_road_carve_data()` true. `on_carve_installed(true)` fired once. |
| `rebuild_when_idle_without_data` | Fresh pipeline, `request_road_rebuild(Always)`. | `NoRoads`; stage `Idle`. |
| `cancel_during_parse_keeps_previous_quadtree` | Import `four_way` to `Done`. Record leaf pointers from `get_all_leaves()`, the `QuadTreeSummary`, `parser().get_data().roads.size()`, and the reset count. Gate the parser; `begin_import(dual_carriageway)`; `cancel()`. Open the gate; run until `Cancelled`. | `cancel()` true; stage `Cancelling` before the gate opens; `begin_import` refused while `Cancelling`. After: same leaf pointers, same summary, same road count, `on_before_quadtree_reset` not called. A new `begin_import(four_way)` then reaches `Done`. |
| `cancel_refused_after_parse` | Gate the road worker of a fresh import; `cancel()`. Then run to `BuildingMeshes` and `cancel()`. | Both false; the import reaches `Done`. |
| `export_after_import` | Import to `Done`; `begin_export` into a directory under the test binary's temp directory. Run until `on_export_done`. | `started`; `result.failure` empty; `stats.files > 0`; `begin_import` refused while running. |
| `export_refused_while_importing` | `begin_import`, then `begin_export` at once. | `started == false`; reason "An import is still running". |
| `clear` | `clear()` while busy, then after Done. | First false. Second true: `leaf_count() == 0`, `parser().has_data() == false`, `roads.have_stats == false`, `on_before_quadtree_reset` fired once. |
| `teardown_mid_import` | `begin_import` with no gate; destroy the pipeline at once. Repeat after entering `BuildingRoads`. | No crash, no hang. The futures-last invariant. No preset enables a sanitizer today (`CMakePresets.json` has none); a local `-fsanitize=address,thread` build of `stratum_tests` is the stronger check. |
| `lucan_matches_golden` | Only when `STRATUM_LUCAN_OSM` is set in the environment; skipped otherwise. Same as `counts_match_golden` against `tests/golden/lucan.json`. | Replaces the throwaway `g++` harness from the verification memory. |

---

## 7. CMake and documentation

- `CMakeLists.txt`, `add_library(stratum_core ...)` (line 118): add a group
  `# Application pipeline: the import state machine, no SDL, no ImGui, no renderer`
  with `src/app/import_pipeline.cpp`, before `# Procedural Generation`.
- `stratum_editor_lib` (line 299): remove `src/editor/import_pipeline.cpp`.
- `git mv src/editor/export_options.hpp src/app/export_options.hpp`; update the include
  in `src/editor/editor_model.hpp`.
- `tests/CMakeLists.txt`: add `app/test_import_pipeline.cpp` to `STRATUM_TEST_SOURCES`,
  `ImportPipeline` to `STRATUM_TEST_SUITES`, and the `STRATUM_TEST_GOLDEN_DIR` definition.
- `CLAUDE.md`:
  - the **stratum_core** bullet: "Contains `src/app`, `src/osm`, `src/osm/road`,
    `src/geometry`, `src/procgen`, `src/scene` only."
  - the Trap paragraph: add `src/app` to "Engine-agnostic code belongs in ...".
  - Source layout: add `app/` — "the import pipeline (parse, roads, index, meshes,
    carve, export) as a tick-driven state machine with a Listener. Core; the editor
    subscribes to it."
  - Tests: 63 `.cpp` files becomes 64, and `app` joins the directory list.
- `docs/agents/core-and-osm.md`: a short `src/app` section pointing here.

---

## Open questions for Sarah

1. **Owed rebuild loses an `Always` request.** Today a toggle such as Solve Junctions
   flipped during `BuildingRoads` defers with `m_road_rebuild_owed`, but
   `finish_osm_import()` honours it through `maybe_rebuild_roads_for_terrain()`
   (`ip.cpp:694-697`), which compares only the terrain fingerprint. The toggle is then
   never applied. Phase 3 keeps this (zero behaviour change). The fix is to store the
   strongest owed policy and replay it. Fix in Phase 3, or in a follow-up?
2. **Worker exceptions.** An exception out of the road or carve future's `get()`
   (`ip.cpp:460,647`) today propagates out of `Editor::update()` and ends the process.
   Recommended: catch it and go to `Failed` with the text, as the export already does
   (`ip.cpp:93-99`). This changes only a crash path. Agree?
3. **Cancel during a fresh import's road stage** is refused (section 1.4). Staging the
   new parser until Indexing would allow it, at the cost of the panels showing the old
   parse counts for one extra stage. Wanted now or later?
