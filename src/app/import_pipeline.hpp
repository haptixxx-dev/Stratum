// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file import_pipeline.hpp
 * @brief The OSM import / road build / spatial index / terrain carve / export
 *        state machine, as a tick-driven class with no UI or GPU dependency.
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Design: `docs/plans/import-pipeline-design.md`, sections 1, 2 and 4. This is
 * the state machine that used to live entirely in `Editor` (see
 * `src/editor/import_pipeline.cpp`), moved here so the same code runs in the
 * editor, in `stratum_tests` and in a future headless CLI.
 *
 * Engine-agnostic, compiled into `stratum_core`. This header and
 * `import_pipeline.cpp` include only core headers -- no SDL, no ImGui, no
 * `renderer/gpu_*`, no `editor/*`. A caller that needs GPU residency, camera
 * framing or ImGui feedback gets it through `ImportPipeline::Listener`.
 */

#pragma once

#include <array>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "app/export_options.hpp"
#include "osm/parser.hpp"
#include "osm/quadtree.hpp"
#include "osm/road/road_export.hpp"
#include "osm/road/road_network_builder.hpp"
#include "procgen/terrain_carve.hpp"
#include "procgen/terrain_generator.hpp"
#include "procgen/terrain_tile_manager.hpp"

namespace stratum::app {

/**
 * @brief Identity of a terrain surface, for deciding whether roads need re-solving
 *
 * Depends only on the fields `TerrainGenerator::sample_surface()` reads, so a
 * change to resolution, water level, erosion or mesh settings does NOT
 * invalidate a solve. Never returns 0; 0 is the "flat, no terrain" sentinel.
 *
 * Moved verbatim from `Editor::terrain_surface_fingerprint`.
 */
[[nodiscard]] uint64_t terrain_surface_fingerprint(const procgen::TerrainConfig& cfg);

/// Stage of the state machine. See docs/plans/import-pipeline-design.md section 1.4
/// for what `cancel()` does at each of these.
enum class ImportStage {
    Idle,
    Parsing,         ///< OSMParser on worker 1
    BuildingRoads,   ///< RoadNetworkBuilder on worker 2, over the whole graph
    Indexing,        ///< QuadTree init/assign on the calling thread, inside one tick()
    BuildingMeshes,  ///< QuadTree's own async node builds, drained by tick()
    CarvingTerrain,  ///< CarveInput::build_index on worker 3, install one tick later
    Done,
    Failed,
    Cancelling,      ///< cancel() accepted; the discarded worker has not landed yet
    Cancelled,       ///< the discarded worker landed; previous data is intact
};

/// Every road-build toggle, read fresh at each begin/request call through
/// set_road_options(). One value standing in for EditorModel's toggles plus
/// Editor::m_use_chunked_terrain.
struct RoadOptions {
    bool terrain_aware = true;
    bool chunked_terrain = true;
    bool solve_junctions = true;
    bool emit_markings = true;
    bool emit_crossings = true;
    bool emit_structures = true;
    bool reduce_tessellation = true;
    bool chunk_lod = true;   ///< read at Indexing, not at launch
};

struct ImportOptions {
    osm::ParserConfig parser;
    RoadOptions roads;
};

enum class RebuildPolicy {
    Always,            ///< today's begin_road_network_rebuild()
    IfSurfaceChanged,  ///< today's maybe_rebuild_roads_for_terrain()
};

enum class RebuildOutcome { Started, Deferred, NotNeeded, NoRoads };

/// Result of a begin_import() or begin_export() call.
struct Launch {
    bool started = false;
    std::string reason;   ///< the exact refusal text logged; empty when started
};

/// Statistics of the last road build, for a panel or a test to read out.
struct RoadBuildStats {
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
    uint64_t terrain_fingerprint = 0; ///< surface the live network was solved against
};

/// What the spatial index looked like right after the last rebuild.
struct QuadTreeSummary {
    size_t leaf_count = 0;
    size_t total_roads = 0;
    size_t total_buildings = 0;
    size_t total_areas = 0;
    uint8_t max_depth = 0;
};

struct PipelineState {
    ImportStage stage = ImportStage::Idle;
    float fraction = 0.0f;
    std::string message;
    bool rebuild_only = false;    ///< was m_road_rebuild_only
    bool rebuild_owed = false;    ///< was m_road_rebuild_owed
    size_t nodes_done = 0;        ///< BuildingMeshes progress
    size_t nodes_total = 0;
    bool export_running = false;
    RoadBuildStats roads;
    QuadTreeSummary quadtree;

    /// Parsing through CarvingTerrain, or Cancelling, or an export in flight.
    [[nodiscard]] bool busy() const;
};

struct ExportResult {
    osm::road::ExportStats stats;
    std::string failure;   ///< exception text; empty on success
    std::string directory;
    bool wrote_collision = false;
    bool wrote_lods = false;
    std::string status;    ///< the exact line the export status gets
};

/// Where the first node builds are queued from. Filled by the caller from its
/// camera. `initial_view()` returning nullopt queues every leaf instead -- the
/// headless and test behaviour.
struct InitialView {
    std::array<glm::vec4, 6> frustum_planes{};
    glm::vec3 camera_position{0.0f};
    float view_radius = 0.0f;
    float fov_y = 45.0f;
    float contribution_threshold = 0.0f;
};

enum class LogLevel { Info, Warn, Error };

struct PipelineLogLine {
    LogLevel level = LogLevel::Info;
    std::string text;
};

/// Test seam only. Each hook runs on the WORKER thread, first thing in the
/// task. Tests use it to hold a worker at a gate; production passes nothing.
struct WorkerHooks {
    std::function<void(ImportStage)> on_worker_start;   ///< Parsing, BuildingRoads, CarvingTerrain
    std::function<void()> on_export_worker_start;
};

/**
 * @brief The import / road build / index / carve / export state machine
 *
 * Owns the committed OSM parse and the pending carve; references the
 * QuadTree and the TerrainTileManager it drives (see the ownership table in
 * docs/plans/import-pipeline-design.md section 1.1).
 *
 * `tick()` is non-blocking: it never waits on a future (every wait is
 * `wait_for(0)`). Every callback a Listener receives runs on the thread that
 * called `tick()`, `begin_*`, `request_*`, `cancel()` or `clear()` -- never on
 * a worker.
 */
class ImportPipeline {
public:
    /**
     * @brief Everything a host (the editor, a test, a headless CLI) hooks in
     *
     * A callback may read state(), parser() and the quadtree. It must not
     * call begin_*, request_*, cancel(), clear() or tick().
     */
    struct Listener {
        virtual ~Listener() = default;
        virtual void on_stage(ImportStage stage, float fraction, const std::string& message) {}
        virtual void on_log(const PipelineLogLine& line) {}
        virtual void on_before_quadtree_reset() {}
        virtual void on_quadtree_ready(bool recenter_camera) {}
        virtual std::optional<InitialView> initial_view() { return std::nullopt; }
        virtual void on_carve_installed(bool carve_present) {}
        virtual void on_export_done(const ExportResult& result) {}
    };

    ImportPipeline(osm::QuadTree& quadtree,
                   procgen::TerrainTileManager& terrain,
                   WorkerHooks hooks = {});

    /// Joins every worker. See docs/plans/import-pipeline-design.md section 4:
    /// destroying a pipeline with a parse or a road build in flight blocks
    /// until it ends.
    ~ImportPipeline();

    ImportPipeline(const ImportPipeline&) = delete;
    ImportPipeline& operator=(const ImportPipeline&) = delete;

    /// Non-owning; may be null.
    void set_listener(Listener* listener);

    /// The latest road options. Call once per frame before tick(), and before
    /// any request or begin call, so every deferred read sees the live model
    /// exactly as the caller currently holds it.
    void set_road_options(const RoadOptions& options);

    Launch begin_import(const std::filesystem::path& path, const ImportOptions& options);
    RebuildOutcome request_road_rebuild(RebuildPolicy policy);
    Launch begin_export(const ExportOptions& options);

    /// Advances every stage that can advance and returns. Never blocks.
    void tick();

    /// True when accepted. See docs/plans/import-pipeline-design.md section 1.4.
    bool cancel();

    /// Drop all imported data. Refused (returns false) while state().busy().
    bool clear();

    [[nodiscard]] const PipelineState& state() const { return state_; }
    [[nodiscard]] const osm::OSMParser& parser() const { return m_parser; }
    [[nodiscard]] bool has_roads() const;
    [[nodiscard]] uint64_t live_terrain_fingerprint() const;

private:
    // ------------------------------------------------------------------
    // Worker payloads
    // ------------------------------------------------------------------

    struct OSMImportJob {
        std::unique_ptr<osm::OSMParser> parser;
        std::string filepath;

        std::mutex mutex;             ///< Guards `progress` (written on the worker)
        osm::ParseProgress progress;

        /// Set by cancel(); the landed result is dropped instead of applied.
        bool discarded = false;

        /// MUST be declared last: ~future on an std::async future joins the
        /// worker, and `parser`/`filepath` must still be alive when it does.
        std::future<bool> future;
    };

    struct RoadBuildResult {
        osm::road::RoadNetwork network;
        osm::road::RoadElevationSolver::Stats elevation;
        float max_grade = 0.0f;
        bool elevated = false;
        bool solved_junctions = false;
        bool emitted_markings = false;
        bool emitted_crossings = false;
        bool emitted_structures = false;
        uint64_t terrain_fingerprint = 0;
    };

    struct RoadExportJob {
        std::string directory;
        osm::road::ExportConfig config;
        bool build_collision = false;
        bool build_lods = false;

        /// MUST be declared last; same rule as OSMImportJob::future.
        std::future<osm::road::ExportStats> future;
    };

    // ------------------------------------------------------------------
    // Notification helpers
    // ------------------------------------------------------------------

    void notify_stage();
    void notify_log(LogLevel level, const std::string& text);
    void log_info(const std::string& text);
    void log_warn(const std::string& text);
    void log_error(const std::string& text);

    // ------------------------------------------------------------------
    // Config and worker bodies
    // ------------------------------------------------------------------

    [[nodiscard]] bool has_generated_terrain() const;
    [[nodiscard]] osm::road::HeightSampler make_height_sampler() const;
    [[nodiscard]] osm::road::RoadNetworkConfig make_road_network_config() const;
    static RoadBuildResult run_road_build(const osm::ParsedOSMData& data,
                                          const osm::road::RoadNetworkConfig& cfg);

    // ------------------------------------------------------------------
    // State machine steps
    // ------------------------------------------------------------------

    void launch_road_build(bool rebuild_only);
    void tick_parse();
    void tick_roads();
    void tick_carve();
    void tick_meshes();
    void tick_export();
    void begin_carve();
    void install_carve();
    void finish();
    void rebuild_index(std::vector<osm::road::RoadPiece>&& road_pieces, bool recenter_camera);

    // ------------------------------------------------------------------
    // Data
    // ------------------------------------------------------------------

    osm::QuadTree& quadtree_;
    procgen::TerrainTileManager& terrain_;
    WorkerHooks hooks_;
    Listener* listener_ = nullptr;

    RoadOptions m_road_options;
    procgen::CarveConfig m_carve_config;

    /// The committed parse. Three workers read its ParsedOSMData by pointer,
    /// so nothing may reassign or clear it while a future below is valid.
    osm::OSMParser m_parser;

    std::unique_ptr<procgen::CarveInput> m_pending_carve;

    /// The indexed carve input is ready and is applied on the NEXT tick():
    /// installing it regenerates every terrain chunk in one blocking call,
    /// and doing that in the tick that sets the stage message would mean the
    /// message never reaches a listener before the hitch.
    bool m_carve_apply_pending = false;

    /// A road rebuild's result is discarded when its future lands. Only ever
    /// set by cancel(), and only while state_.rebuild_only is true.
    bool m_road_build_discarded = false;

    PipelineState state_;
    std::vector<osm::QuadTreeNode*> m_import_pending_nodes;

    // Futures last, in this order (docs/plans/import-pipeline-design.md
    // section 4). Members destruct in reverse declaration order, and ~future
    // of an std::async future joins the worker; everything a worker can
    // read -- m_parser above all -- is declared above this block, so it is
    // still alive while a worker joins.
    std::unique_ptr<OSMImportJob> m_import_job;
    std::unique_ptr<RoadExportJob> m_export_job;
    std::future<RoadBuildResult> m_road_build_future;
    std::future<std::unique_ptr<procgen::CarveInput>> m_carve_index_future;
};

} // namespace stratum::app
