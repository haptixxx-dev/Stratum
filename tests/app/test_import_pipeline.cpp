// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_import_pipeline.cpp
 * @brief Drives stratum::app::ImportPipeline end to end with no window and no GPU
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Written against `docs/plans/import-pipeline-design.md`, section 6. The pipeline
 * moved out of `Editor` in phase 3 of the UI restructure so it could be driven from
 * here: a plain `tick()` loop, no SDL, no ImGui, no GPU handle anywhere in the chain.
 *
 * Fixtures: `tests/data/four_way.osm` and `tests/data/dual_carriageway.osm`, both
 * tiny and both already golden (`tests/golden/<fixture>.json`, produced by
 * `stratum_golden`, the deterministic core-only snapshot tool). The golden numbers
 * are the contract: phase 3 moved this state machine into `stratum_core` on the
 * rule of "zero behaviour change", so a pipeline-driven import must produce exactly
 * the parser, road network, junction, quadtree and mesh counts stratum_golden
 * already recorded for these files. Rather than hard-code those numbers a second
 * time (and let the two copies drift), the golden JSON is read at test time with
 * nlohmann, already a PUBLIC link of stratum_core.
 *
 * `spdlog` is set to `off` for the whole suite, as `stratum_golden` does: nothing
 * here wants a log line, and nothing here should have one written under it either.
 *
 * Threading: `ImportPipeline::tick()` is documented as never blocking -- every wait
 * it does is `wait_for(0)`. `WorkerHooks::on_worker_start` runs on the WORKER thread,
 * first thing in the task, and is the seam this suite uses to hold a worker at a
 * gate while the test thread calls `tick()` and inspects state that would otherwise
 * race past in microseconds. `Gate` below is the gate; it is deliberately NOT
 * shared with framework.hpp, which is generic and takes no dependency on
 * <mutex>/<condition_variable>.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests ImportPipeline
 * @endcode
 */

#include "framework.hpp"

#include "app/export_options.hpp"
#include "app/import_pipeline.hpp"
#include "osm/quadtree.hpp"
#include "procgen/terrain_tile_manager.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifndef STRATUM_TEST_DATA_DIR
#error "STRATUM_TEST_DATA_DIR must be defined by the build; see tests/CMakeLists.txt"
#endif

#ifndef STRATUM_TEST_GOLDEN_DIR
#error "STRATUM_TEST_GOLDEN_DIR must be defined by the build; see tests/CMakeLists.txt"
#endif

namespace {

using stratum::Mesh;
using stratum::ExportOptions;
using stratum::app::ExportResult;
using stratum::app::ImportOptions;
using stratum::app::ImportPipeline;
using stratum::app::ImportStage;
using stratum::app::Launch;
using stratum::app::QuadTreeSummary;
using stratum::app::RebuildOutcome;
using stratum::app::RebuildPolicy;
using stratum::app::WorkerHooks;
using stratum::osm::QuadTree;
using stratum::osm::QuadTreeNode;
using stratum::procgen::TerrainTileConfig;
using stratum::procgen::TerrainTileManager;

// ============================================================================
// Quiet the suite, exactly as stratum_golden does
// ============================================================================

struct SilenceLogs {
    SilenceLogs() { spdlog::set_level(spdlog::level::off); }
} g_silence_logs;

// ============================================================================
// Gate: hold a worker thread until the test thread is ready to let it run
// ============================================================================

class Gate {
public:
    explicit Gate(bool initially_open = true) : open_(initially_open) {}

    /// Blocks the calling (worker) thread until open() is called.
    void wait() {
        std::unique_lock<std::mutex> lock(mutex_);
        cv_.wait(lock, [this] { return open_; });
    }

    void open() {
        std::lock_guard<std::mutex> lock(mutex_);
        open_ = true;
        cv_.notify_all();
    }

    void close() {
        std::lock_guard<std::mutex> lock(mutex_);
        open_ = false;
    }

private:
    std::mutex mutex_;
    std::condition_variable cv_;
    bool open_;
};

// ============================================================================
// run_until: tick() in a loop, fail the test on timeout instead of hanging it
// ============================================================================

/// Calls pipeline.tick() until predicate() is true or timeout elapses. Returns
/// false on timeout so the caller can CHECK_TRUE() it and bail out of the rest
/// of the test rather than reading state that never arrived.
bool run_until(ImportPipeline& pipeline, const std::function<bool()>& predicate,
               std::chrono::milliseconds timeout = std::chrono::seconds(30)) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (true) {
        pipeline.tick();
        if (predicate()) return true;
        if (std::chrono::steady_clock::now() >= deadline) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

// ============================================================================
// RecordingListener: every on_stage() call, in order, plus the counters the
// cancel/clear/carve tests need
// ============================================================================

struct StageEvent {
    ImportStage stage;
    float fraction;
    std::string message;
    /// PipelineState::rebuild_only / rebuild_owed AT THE MOMENT this callback
    /// fired -- read through `pipeline`, which is set once both objects exist.
    /// finish() can flip stage from Done to a fresh BuildingRoads (an owed
    /// rebuild) synchronously, inside the SAME tick(), before the test thread
    /// ever gets to poll state() again -- so a transient Done is only ever
    /// observable here, not by polling.
    bool rebuild_only = false;
    bool rebuild_owed = false;
};

class RecordingListener final : public ImportPipeline::Listener {
public:
    const ImportPipeline* pipeline = nullptr;

    std::vector<StageEvent> events;
    int before_reset_count = 0;
    int carve_installed_true_count = 0;
    int carve_installed_false_count = 0;
    std::vector<ExportResult> export_results;

    void on_stage(ImportStage stage, float fraction, const std::string& message) override {
        StageEvent e{stage, fraction, message, false, false};
        if (pipeline) {
            e.rebuild_only = pipeline->state().rebuild_only;
            e.rebuild_owed = pipeline->state().rebuild_owed;
        }
        events.push_back(std::move(e));
    }

    void on_before_quadtree_reset() override { ++before_reset_count; }

    void on_carve_installed(bool carve_present) override {
        if (carve_present) ++carve_installed_true_count;
        else ++carve_installed_false_count;
    }

    void on_export_done(const ExportResult& result) override {
        export_results.push_back(result);
    }

    [[nodiscard]] size_t count_stage(ImportStage stage) const {
        size_t n = 0;
        for (const auto& e : events) {
            if (e.stage == stage) ++n;
        }
        return n;
    }
};

/// The stage sequence a listener recorded, with consecutive duplicates removed.
/// Two DIFFERENT visits to the same stage (an owed rebuild's second
/// BuildingRoads) are kept -- only back-to-back repeats of one stage collapse.
std::vector<ImportStage> dedupe_consecutive(const std::vector<StageEvent>& events) {
    std::vector<ImportStage> out;
    for (const auto& e : events) {
        if (out.empty() || out.back() != e.stage) out.push_back(e.stage);
    }
    return out;
}

// ============================================================================
// Golden JSON
// ============================================================================

nlohmann::json load_golden_json(const std::string& fixture_stem) {
    const auto path = std::filesystem::path(STRATUM_TEST_GOLDEN_DIR) / (fixture_stem + ".json");
    std::ifstream in(path);
    CHECK_TRUE(static_cast<bool>(in));
    nlohmann::json j;
    if (in) {
        in >> j;
    }
    return j;
}

// ============================================================================
// Mesh totals -- summed from the quadtree's leaves exactly as
// tools/golden/stratum_golden.cpp does, so the two never quietly disagree
// ============================================================================

struct MeshTotals {
    uint64_t triangles = 0;
    uint64_t vertices = 0;
};

void accumulate(MeshTotals& totals, const Mesh& mesh) {
    totals.vertices += mesh.vertices.size();
    totals.triangles += mesh.indices.size() / 3;
}

void sum_leaf_meshes(QuadTree& quadtree, MeshTotals& roads, MeshTotals& buildings,
                     MeshTotals& areas) {
    for (const auto* leaf : quadtree.get_all_leaves()) {
        if (leaf->has_road_lod() && !leaf->road_lod.levels.empty()) {
            accumulate(roads, leaf->road_lod.levels[0]);
        } else {
            for (const auto& mesh : leaf->road_meshes) accumulate(roads, mesh);
        }
        for (const auto& mesh : leaf->building_meshes) accumulate(buildings, mesh);
        for (const auto& mesh : leaf->area_meshes) accumulate(areas, mesh);
    }
}

// ============================================================================
// Shared fixture path helper
// ============================================================================

std::filesystem::path fixture_path(const char* filename) {
    return std::filesystem::path(STRATUM_TEST_DATA_DIR) / filename;
}

// ============================================================================
// stage_order_<fixture>
// ============================================================================

void check_stage_order(const char* filename) {
    QuadTree quadtree;
    TerrainTileManager terrain;
    RecordingListener listener;
    ImportPipeline pipeline(quadtree, terrain);
    listener.pipeline = &pipeline;
    pipeline.set_listener(&listener);

    const Launch launch = pipeline.begin_import(fixture_path(filename), ImportOptions{});
    CHECK_TRUE(launch.started);
    if (!launch.started) return;

    const bool done = run_until(pipeline, [&] {
        return pipeline.state().stage == ImportStage::Done ||
               pipeline.state().stage == ImportStage::Failed;
    });
    CHECK_TRUE(done);
    if (!done) return;

    CHECK_EQ(static_cast<int>(pipeline.state().stage), static_cast<int>(ImportStage::Done));
    CHECK_NEAR(pipeline.state().fraction, 1.0f, 1e-6);
    CHECK_EQ(pipeline.state().message, std::string("Import successful"));

    const std::vector<ImportStage> stages = dedupe_consecutive(listener.events);
    const std::vector<ImportStage> expected = {
        ImportStage::Parsing,        ImportStage::BuildingRoads, ImportStage::Indexing,
        ImportStage::BuildingMeshes, ImportStage::CarvingTerrain, ImportStage::Done,
    };
    CHECK_TRUE(stages == expected);
}

} // namespace

TEST(ImportPipeline, stage_order_four_way) { check_stage_order("four_way.osm"); }
TEST(ImportPipeline, stage_order_dual_carriageway) { check_stage_order("dual_carriageway.osm"); }

namespace {

// ============================================================================
// counts_match_golden_<fixture>
// ============================================================================

void check_counts_match_golden(const char* filename) {
    const std::string stem = std::filesystem::path(filename).stem().string();
    const nlohmann::json golden = load_golden_json(stem);

    QuadTree quadtree;
    TerrainTileManager terrain;
    ImportPipeline pipeline(quadtree, terrain);

    const Launch launch = pipeline.begin_import(fixture_path(filename), ImportOptions{});
    CHECK_TRUE(launch.started);
    if (!launch.started) return;

    const bool done = run_until(pipeline, [&] { return pipeline.state().stage == ImportStage::Done; });
    CHECK_TRUE(done);
    if (!done) return;

    // ── Parser ──
    const auto& data = pipeline.parser().get_data();
    const auto& pgold = golden.at("parser");
    CHECK_EQ(data.stats.total_nodes, pgold.at("nodes").get<size_t>());
    CHECK_EQ(data.stats.total_ways, pgold.at("ways").get<size_t>());
    CHECK_EQ(data.stats.total_relations, pgold.at("relations").get<size_t>());
    CHECK_EQ(data.buildings.size(), pgold.at("buildings").get<size_t>());
    CHECK_EQ(data.roads.size(), pgold.at("roads").get<size_t>());
    CHECK_EQ(data.areas.size(), pgold.at("areas").get<size_t>());

    // ── Road network ──
    const auto& net = pipeline.state().roads.network;
    const auto& ngold = golden.at("road_network");
    CHECK_EQ(net.edges, ngold.at("edges").get<size_t>());
    CHECK_EQ(net.pieces, ngold.at("pieces").get<size_t>());
    CHECK_EQ(net.vertices, ngold.at("vertices").get<size_t>());
    CHECK_EQ(net.triangles, ngold.at("triangles").get<size_t>());

    // ── Junctions ──
    const auto& js = pipeline.state().roads.junctions;
    const auto& jgold = golden.at("junction_stats");
    CHECK_EQ(js.junctions, jgold.at("junctions").get<size_t>());
    CHECK_EQ(js.roundabouts, jgold.at("roundabouts").get<size_t>());
    CHECK_EQ(js.tapers, jgold.at("tapers").get<size_t>());
    CHECK_EQ(js.dead_ends, jgold.at("dead_ends").get<size_t>());
    CHECK_EQ(js.degenerate, jgold.at("degenerate").get<size_t>());
    CHECK_EQ(js.merged_into_neighbour, jgold.at("merged_into_neighbour").get<size_t>());
    CHECK_EQ(js.self_intersecting, jgold.at("self_intersecting").get<size_t>());
    CHECK_EQ(js.over_trimmed_edges, jgold.at("over_trimmed_edges").get<size_t>());

    // ── QuadTree: the live tree and PipelineState's own copy must agree ──
    const auto& qgold = golden.at("quadtree");
    CHECK_EQ(quadtree.leaf_count(), qgold.at("leaf_count").get<size_t>());
    CHECK_EQ(quadtree.total_roads(), qgold.at("total_roads").get<size_t>());
    CHECK_EQ(quadtree.total_buildings(), qgold.at("total_buildings").get<size_t>());
    CHECK_EQ(quadtree.total_areas(), qgold.at("total_areas").get<size_t>());
    CHECK_EQ(static_cast<int>(quadtree.max_depth()), qgold.at("max_depth").get<int>());

    const auto& summary = pipeline.state().quadtree;
    CHECK_EQ(summary.leaf_count, qgold.at("leaf_count").get<size_t>());
    CHECK_EQ(summary.total_roads, qgold.at("total_roads").get<size_t>());
    CHECK_EQ(summary.total_buildings, qgold.at("total_buildings").get<size_t>());
    CHECK_EQ(summary.total_areas, qgold.at("total_areas").get<size_t>());
    CHECK_EQ(static_cast<int>(summary.max_depth), qgold.at("max_depth").get<int>());

    // ── Meshes: leaf/road/building/area triangle and vertex counts ──
    MeshTotals road_totals, building_totals, area_totals;
    sum_leaf_meshes(quadtree, road_totals, building_totals, area_totals);

    const auto& mgold = golden.at("meshes");
    CHECK_EQ(road_totals.triangles, mgold.at("roads").at("triangles").get<uint64_t>());
    CHECK_EQ(road_totals.vertices, mgold.at("roads").at("vertices").get<uint64_t>());
    CHECK_EQ(building_totals.triangles, mgold.at("buildings").at("triangles").get<uint64_t>());
    CHECK_EQ(building_totals.vertices, mgold.at("buildings").at("vertices").get<uint64_t>());
    CHECK_EQ(area_totals.triangles, mgold.at("areas").at("triangles").get<uint64_t>());
    CHECK_EQ(area_totals.vertices, mgold.at("areas").at("vertices").get<uint64_t>());
}

} // namespace

TEST(ImportPipeline, counts_match_golden_four_way) { check_counts_match_golden("four_way.osm"); }
TEST(ImportPipeline, counts_match_golden_dual_carriageway) {
    check_counts_match_golden("dual_carriageway.osm");
}

namespace {

// ============================================================================
// tick_never_blocks
// ============================================================================

/// 200 tick() calls while a worker is gated shut must return well inside this,
/// or something on the hot path started waiting on a future.
constexpr auto kBoundedTickBudget = std::chrono::milliseconds(1000);
constexpr int kBoundedTickCount = 200;

} // namespace

TEST(ImportPipeline, tick_never_blocks) {
    QuadTree quadtree;
    TerrainTileManager terrain;

    // Terrain present BEFORE the import starts, so the road build's height
    // sampler is non-null from the first launch: only an elevated build
    // produces carve ribbons (RoadNetworkBuilder gates them on `on_terrain`),
    // and only then does CarvingTerrain actually spawn a worker to gate. With
    // no terrain, four_way.osm's build is flat, begin_carve() finds nothing to
    // index, and CarvingTerrain collapses into Done inside the same tick() --
    // there is no worker there to hold shut.
    TerrainTileConfig terrain_config;
    terrain_config.chunk_size = 100.0f;
    terrain_config.chunk_resolution = 4;
    terrain_config.world_min = glm::vec2(0.0f, 0.0f);
    terrain_config.world_max = glm::vec2(100.0f, 100.0f);
    terrain.init(terrain_config);
    terrain.generate_all_chunks();
    CHECK_TRUE(terrain.generated_count() > 0);

    Gate parse_gate(false);
    Gate roads_gate(false);
    Gate carve_gate(false);

    WorkerHooks hooks;
    hooks.on_worker_start = [&](ImportStage stage) {
        switch (stage) {
            case ImportStage::Parsing: parse_gate.wait(); break;
            case ImportStage::BuildingRoads: roads_gate.wait(); break;
            case ImportStage::CarvingTerrain: carve_gate.wait(); break;
            default: break;
        }
    };

    ImportPipeline pipeline(quadtree, terrain, hooks);

    const Launch launch = pipeline.begin_import(fixture_path("four_way.osm"), ImportOptions{});
    CHECK_TRUE(launch.started);
    if (!launch.started) return;

    auto check_bounded_while_gated = [&](ImportStage expected_stage) -> bool {
        const bool reached = run_until(pipeline, [&] { return pipeline.state().stage == expected_stage; });
        CHECK_TRUE(reached);
        if (!reached) return false;

        const auto start = std::chrono::steady_clock::now();
        for (int i = 0; i < kBoundedTickCount; ++i) {
            pipeline.tick();
        }
        const auto elapsed = std::chrono::steady_clock::now() - start;

        CHECK_TRUE(elapsed < kBoundedTickBudget);
        // Still gated: none of those 200 calls should have moved the stage on.
        CHECK_EQ(static_cast<int>(pipeline.state().stage), static_cast<int>(expected_stage));
        return true;
    };

    if (!check_bounded_while_gated(ImportStage::Parsing)) return;
    parse_gate.open();

    if (!check_bounded_while_gated(ImportStage::BuildingRoads)) return;
    roads_gate.open();

    if (!check_bounded_while_gated(ImportStage::CarvingTerrain)) return;
    carve_gate.open();

    CHECK_TRUE(run_until(pipeline, [&] { return pipeline.state().stage == ImportStage::Done; }));
}

// ============================================================================
// rebuild_when_idle_without_data
// ============================================================================

TEST(ImportPipeline, rebuild_when_idle_without_data) {
    QuadTree quadtree;
    TerrainTileManager terrain;
    ImportPipeline pipeline(quadtree, terrain);

    const RebuildOutcome outcome = pipeline.request_road_rebuild(RebuildPolicy::Always);
    CHECK_EQ(static_cast<int>(outcome), static_cast<int>(RebuildOutcome::NoRoads));
    CHECK_EQ(static_cast<int>(pipeline.state().stage), static_cast<int>(ImportStage::Idle));
}

// ============================================================================
// rebuild_request_during_import_is_honoured_after_done
// ============================================================================

TEST(ImportPipeline, rebuild_request_during_import_is_honoured_after_done) {
    QuadTree quadtree;
    TerrainTileManager terrain;  // no terrain generated yet

    Gate roads_gate(false);
    WorkerHooks hooks;
    hooks.on_worker_start = [&](ImportStage stage) {
        if (stage == ImportStage::BuildingRoads) roads_gate.wait();
    };

    RecordingListener listener;
    ImportPipeline pipeline(quadtree, terrain, hooks);
    listener.pipeline = &pipeline;
    pipeline.set_listener(&listener);

    // Import with no terrain: the first road build is solved flat.
    const Launch launch =
        pipeline.begin_import(fixture_path("four_way.osm"), ImportOptions{});
    CHECK_TRUE(launch.started);
    if (!launch.started) return;

    const bool reached_roads =
        run_until(pipeline, [&] { return pipeline.state().stage == ImportStage::BuildingRoads; });
    CHECK_TRUE(reached_roads);
    if (!reached_roads) return;

    // While the road worker is gated: bring terrain up so the surface the
    // roads would be solved against actually changes.
    TerrainTileConfig terrain_config;
    terrain_config.chunk_size = 100.0f;
    terrain_config.chunk_resolution = 4;
    terrain_config.world_min = glm::vec2(0.0f, 0.0f);
    terrain_config.world_max = glm::vec2(100.0f, 100.0f);
    terrain.init(terrain_config);
    terrain.generate_all_chunks();
    CHECK_TRUE(terrain.generated_count() > 0);

    const RebuildOutcome outcome =
        pipeline.request_road_rebuild(RebuildPolicy::IfSurfaceChanged);
    CHECK_EQ(static_cast<int>(outcome), static_cast<int>(RebuildOutcome::Deferred));
    CHECK_TRUE(pipeline.state().rebuild_owed);

    roads_gate.open();

    // The first Done is transient: finish() honours the owed rebuild
    // synchronously, inside the tick() that produced it, before this thread
    // gets to poll state() again. Only the listener sees it.
    const bool saw_first_done = run_until(pipeline, [&] { return listener.count_stage(ImportStage::Done) >= 1; });
    CHECK_TRUE(saw_first_done);
    if (!saw_first_done) return;

    const StageEvent* first_done = nullptr;
    for (const auto& e : listener.events) {
        if (e.stage == ImportStage::Done) { first_done = &e; break; }
    }
    CHECK_TRUE(first_done != nullptr);
    if (first_done) {
        CHECK_FALSE(first_done->rebuild_only);
        CHECK_EQ(first_done->message, std::string("Import successful"));
    }

    // A second BuildingRoads, this time a rebuild, was launched from inside
    // finish(). Run until the whole pipeline settles at Done again.
    const bool reached_final_done =
        run_until(pipeline, [&] { return pipeline.state().stage == ImportStage::Done; });
    CHECK_TRUE(reached_final_done);
    if (!reached_final_done) return;

    CHECK_EQ(listener.count_stage(ImportStage::BuildingRoads), size_t{2});
    bool saw_plain = false, saw_rebuild = false;
    for (const auto& e : listener.events) {
        if (e.stage != ImportStage::BuildingRoads) continue;
        if (e.rebuild_only) saw_rebuild = true; else saw_plain = true;
    }
    CHECK_TRUE(saw_plain);
    CHECK_TRUE(saw_rebuild);

    CHECK_EQ(pipeline.state().message, std::string("Roads re-solved against the terrain"));
    CHECK_FALSE(pipeline.state().rebuild_owed);

    const uint64_t live_fingerprint = pipeline.live_terrain_fingerprint();
    CHECK_TRUE(live_fingerprint != 0);
    CHECK_EQ(pipeline.state().roads.terrain_fingerprint, live_fingerprint);

    CHECK_TRUE(terrain.has_road_carve_data());
}

// ============================================================================
// cancel_during_parse_keeps_previous_quadtree
// ============================================================================

TEST(ImportPipeline, cancel_during_parse_keeps_previous_quadtree) {
    QuadTree quadtree;
    TerrainTileManager terrain;

    Gate parse_gate(true);  // open: the FIRST import must run to Done unhindered
    WorkerHooks hooks;
    hooks.on_worker_start = [&](ImportStage stage) {
        if (stage == ImportStage::Parsing) parse_gate.wait();
    };

    RecordingListener listener;
    ImportPipeline pipeline(quadtree, terrain, hooks);
    listener.pipeline = &pipeline;
    pipeline.set_listener(&listener);

    // First import: four_way, to Done, uncontested.
    CHECK_TRUE(pipeline.begin_import(fixture_path("four_way.osm"), ImportOptions{}).started);
    CHECK_TRUE(run_until(pipeline, [&] { return pipeline.state().stage == ImportStage::Done; }));

    const std::vector<QuadTreeNode*> leaves_before = quadtree.get_all_leaves();
    const QuadTreeSummary summary_before = pipeline.state().quadtree;
    const size_t roads_before = pipeline.parser().get_data().roads.size();
    const int reset_count_before = listener.before_reset_count;

    // Second import: dual_carriageway, gated at Parsing, then cancelled.
    parse_gate.close();
    const Launch second_launch =
        pipeline.begin_import(fixture_path("dual_carriageway.osm"), ImportOptions{});
    CHECK_TRUE(second_launch.started);

    CHECK_TRUE(pipeline.cancel());
    CHECK_EQ(static_cast<int>(pipeline.state().stage), static_cast<int>(ImportStage::Cancelling));

    // Refused while a cancel is still stopping.
    const Launch refused_while_cancelling =
        pipeline.begin_import(fixture_path("four_way.osm"), ImportOptions{});
    CHECK_FALSE(refused_while_cancelling.started);
    CHECK_EQ(refused_while_cancelling.reason, std::string("A cancelled import is still stopping"));

    parse_gate.open();
    CHECK_TRUE(run_until(pipeline, [&] { return pipeline.state().stage == ImportStage::Cancelled; }));
    CHECK_EQ(pipeline.state().message, std::string("Import cancelled"));

    // Nothing committed changed: same leaves, same summary, same parser data,
    // and the quadtree was never torn down.
    const std::vector<QuadTreeNode*> leaves_after = quadtree.get_all_leaves();
    CHECK_TRUE(leaves_before == leaves_after);

    const QuadTreeSummary& summary_after = pipeline.state().quadtree;
    CHECK_EQ(summary_after.leaf_count, summary_before.leaf_count);
    CHECK_EQ(summary_after.total_roads, summary_before.total_roads);
    CHECK_EQ(summary_after.total_buildings, summary_before.total_buildings);
    CHECK_EQ(summary_after.total_areas, summary_before.total_areas);
    CHECK_EQ(static_cast<int>(summary_after.max_depth), static_cast<int>(summary_before.max_depth));

    CHECK_EQ(pipeline.parser().get_data().roads.size(), roads_before);
    CHECK_EQ(listener.before_reset_count, reset_count_before);

    // A fresh import still works afterwards.
    CHECK_TRUE(pipeline.begin_import(fixture_path("four_way.osm"), ImportOptions{}).started);
    CHECK_TRUE(run_until(pipeline, [&] { return pipeline.state().stage == ImportStage::Done; }));
}

// ============================================================================
// cancel_refused_after_parse
// ============================================================================

TEST(ImportPipeline, cancel_refused_after_parse) {
    QuadTree quadtree;
    TerrainTileManager terrain;

    Gate roads_gate(false);
    WorkerHooks hooks;
    hooks.on_worker_start = [&](ImportStage stage) {
        if (stage == ImportStage::BuildingRoads) roads_gate.wait();
    };

    ImportPipeline pipeline(quadtree, terrain, hooks);

    CHECK_TRUE(pipeline.begin_import(fixture_path("four_way.osm"), ImportOptions{}).started);

    const bool reached_roads =
        run_until(pipeline, [&] { return pipeline.state().stage == ImportStage::BuildingRoads; });
    CHECK_TRUE(reached_roads);
    if (!reached_roads) return;

    // A fresh import's road stage: m_parser already holds the new parse while
    // the quadtree holds the old one, so cancel() must refuse.
    CHECK_FALSE(pipeline.cancel());

    roads_gate.open();

    const bool reached_meshes =
        run_until(pipeline, [&] { return pipeline.state().stage == ImportStage::BuildingMeshes; });
    CHECK_TRUE(reached_meshes);
    if (reached_meshes) {
        CHECK_FALSE(pipeline.cancel());
    }

    CHECK_TRUE(run_until(pipeline, [&] { return pipeline.state().stage == ImportStage::Done; }));
}

// ============================================================================
// export_after_import / export_refused_while_importing
// ============================================================================

TEST(ImportPipeline, export_after_import) {
    QuadTree quadtree;
    TerrainTileManager terrain;
    RecordingListener listener;
    ImportPipeline pipeline(quadtree, terrain);
    listener.pipeline = &pipeline;
    pipeline.set_listener(&listener);

    CHECK_TRUE(pipeline.begin_import(fixture_path("four_way.osm"), ImportOptions{}).started);
    CHECK_TRUE(run_until(pipeline, [&] { return pipeline.state().stage == ImportStage::Done; }));

    std::error_code ec;
    const auto export_dir = std::filesystem::temp_directory_path(ec) /
                            "stratum_import_pipeline_test_export_after_import";
    std::filesystem::remove_all(export_dir, ec);
    std::filesystem::create_directories(export_dir, ec);

    ExportOptions export_options;
    std::snprintf(export_options.dir, sizeof(export_options.dir), "%s", export_dir.string().c_str());

    const Launch launch = pipeline.begin_export(export_options);
    CHECK_TRUE(launch.started);
    if (!launch.started) return;

    // An import is refused while the export is running.
    const Launch refused =
        pipeline.begin_import(fixture_path("dual_carriageway.osm"), ImportOptions{});
    CHECK_FALSE(refused.started);

    const bool exported =
        run_until(pipeline, [&] { return !listener.export_results.empty(); });
    CHECK_TRUE(exported);
    if (exported) {
        const auto& result = listener.export_results.front();
        CHECK_TRUE(result.failure.empty());
        CHECK_TRUE(result.stats.files > 0);
    }
    CHECK_FALSE(pipeline.state().export_running);

    std::filesystem::remove_all(export_dir, ec);
}

TEST(ImportPipeline, export_refused_while_importing) {
    QuadTree quadtree;
    TerrainTileManager terrain;
    ImportPipeline pipeline(quadtree, terrain);

    CHECK_TRUE(pipeline.begin_import(fixture_path("four_way.osm"), ImportOptions{}).started);

    ExportOptions export_options;
    std::snprintf(export_options.dir, sizeof(export_options.dir), "%s", "/tmp/stratum-test-unused");
    const Launch launch = pipeline.begin_export(export_options);
    CHECK_FALSE(launch.started);
    CHECK_EQ(launch.reason, std::string("An import is still running"));

    CHECK_TRUE(run_until(pipeline, [&] { return pipeline.state().stage == ImportStage::Done; }));
}

// ============================================================================
// clear
// ============================================================================

TEST(ImportPipeline, clear) {
    QuadTree quadtree;
    TerrainTileManager terrain;

    Gate parse_gate(false);
    WorkerHooks hooks;
    hooks.on_worker_start = [&](ImportStage stage) {
        if (stage == ImportStage::Parsing) parse_gate.wait();
    };

    RecordingListener listener;
    ImportPipeline pipeline(quadtree, terrain, hooks);
    listener.pipeline = &pipeline;
    pipeline.set_listener(&listener);

    CHECK_TRUE(pipeline.begin_import(fixture_path("four_way.osm"), ImportOptions{}).started);

    // Busy (still Parsing, gated): refused.
    CHECK_FALSE(pipeline.clear());

    parse_gate.open();
    CHECK_TRUE(run_until(pipeline, [&] { return pipeline.state().stage == ImportStage::Done; }));

    const int reset_before = listener.before_reset_count;
    CHECK_TRUE(pipeline.clear());
    CHECK_EQ(listener.before_reset_count - reset_before, 1);

    CHECK_EQ(quadtree.leaf_count(), size_t{0});
    CHECK_FALSE(pipeline.parser().has_data());
    CHECK_FALSE(pipeline.state().roads.have_stats);
    CHECK_EQ(static_cast<int>(pipeline.state().stage), static_cast<int>(ImportStage::Idle));
}

// ============================================================================
// teardown_mid_import
// ============================================================================

TEST(ImportPipeline, teardown_mid_import) {
    const auto path = fixture_path("four_way.osm");

    {
        QuadTree quadtree;
        TerrainTileManager terrain;
        ImportPipeline pipeline(quadtree, terrain);
        CHECK_TRUE(pipeline.begin_import(path, ImportOptions{}).started);
        // ~ImportPipeline runs here, with the parse worker still in flight. It
        // must join it rather than crash or hang -- the futures-last invariant.
    }
    CHECK_TRUE(true);

    {
        QuadTree quadtree;
        TerrainTileManager terrain;
        ImportPipeline pipeline(quadtree, terrain);
        CHECK_TRUE(pipeline.begin_import(path, ImportOptions{}).started);
        const bool reached_roads =
            run_until(pipeline, [&] { return pipeline.state().stage == ImportStage::BuildingRoads; });
        CHECK_TRUE(reached_roads);
        // ~ImportPipeline runs here, with the road-build worker still in flight.
    }
    CHECK_TRUE(true);
}

// ============================================================================
// lucan_matches_golden -- only when STRATUM_LUCAN_OSM names a real file
// ============================================================================

TEST(ImportPipeline, lucan_matches_golden) {
    const char* lucan_env = std::getenv("STRATUM_LUCAN_OSM");
    if (!lucan_env || lucan_env[0] == '\0') {
        ::stratum::test::skip_test("STRATUM_LUCAN_OSM is not set; skipping the Lucan extract check");
        return;
    }

    const std::filesystem::path osm_path(lucan_env);
    if (!std::filesystem::exists(osm_path)) {
        ::stratum::test::skip_test("STRATUM_LUCAN_OSM does not point at an existing file");
        return;
    }

    const nlohmann::json golden = load_golden_json("lucan");

    QuadTree quadtree;
    TerrainTileManager terrain;
    ImportPipeline pipeline(quadtree, terrain);

    const Launch launch = pipeline.begin_import(osm_path, ImportOptions{});
    CHECK_TRUE(launch.started);
    if (!launch.started) return;

    // A real-world extract is orders of magnitude bigger than the hand-built
    // fixtures; give it a much longer budget than the default 30 s.
    const bool done = run_until(pipeline, [&] { return pipeline.state().stage == ImportStage::Done; },
                                std::chrono::minutes(10));
    CHECK_TRUE(done);
    if (!done) return;

    const auto& data = pipeline.parser().get_data();
    const auto& pgold = golden.at("parser");
    CHECK_EQ(data.stats.total_nodes, pgold.at("nodes").get<size_t>());
    CHECK_EQ(data.stats.total_ways, pgold.at("ways").get<size_t>());
    CHECK_EQ(data.stats.total_relations, pgold.at("relations").get<size_t>());
    CHECK_EQ(data.buildings.size(), pgold.at("buildings").get<size_t>());
    CHECK_EQ(data.roads.size(), pgold.at("roads").get<size_t>());
    CHECK_EQ(data.areas.size(), pgold.at("areas").get<size_t>());

    const auto& net = pipeline.state().roads.network;
    const auto& ngold = golden.at("road_network");
    CHECK_EQ(net.edges, ngold.at("edges").get<size_t>());
    CHECK_EQ(net.pieces, ngold.at("pieces").get<size_t>());
    CHECK_EQ(net.vertices, ngold.at("vertices").get<size_t>());
    CHECK_EQ(net.triangles, ngold.at("triangles").get<size_t>());

    const auto& qgold = golden.at("quadtree");
    CHECK_EQ(quadtree.leaf_count(), qgold.at("leaf_count").get<size_t>());
    CHECK_EQ(quadtree.total_roads(), qgold.at("total_roads").get<size_t>());
    CHECK_EQ(quadtree.total_buildings(), qgold.at("total_buildings").get<size_t>());
    CHECK_EQ(quadtree.total_areas(), qgold.at("total_areas").get<size_t>());

    MeshTotals road_totals, building_totals, area_totals;
    sum_leaf_meshes(quadtree, road_totals, building_totals, area_totals);
    const auto& mgold = golden.at("meshes");
    CHECK_EQ(road_totals.triangles, mgold.at("roads").at("triangles").get<uint64_t>());
    CHECK_EQ(road_totals.vertices, mgold.at("roads").at("vertices").get<uint64_t>());
    CHECK_EQ(building_totals.triangles, mgold.at("buildings").at("triangles").get<uint64_t>());
    CHECK_EQ(building_totals.vertices, mgold.at("buildings").at("vertices").get<uint64_t>());
    CHECK_EQ(area_totals.triangles, mgold.at("areas").at("triangles").get<uint64_t>());
    CHECK_EQ(area_totals.vertices, mgold.at("areas").at("vertices").get<uint64_t>());
}

