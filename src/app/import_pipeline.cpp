// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "app/import_pipeline.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <exception>
#include <utility>

namespace stratum::app {

// ============================================================================
// PipelineState
// ============================================================================

bool PipelineState::busy() const {
    if (export_running) return true;
    switch (stage) {
        case ImportStage::Parsing:
        case ImportStage::BuildingRoads:
        case ImportStage::Indexing:
        case ImportStage::BuildingMeshes:
        case ImportStage::CarvingTerrain:
        case ImportStage::Cancelling:
            return true;
        default:
            return false;
    }
}

// ============================================================================
// Terrain surface fingerprint
// ============================================================================

namespace {

/// Boost-style mixer, so the fingerprint depends on field ORDER as well as value
void hash_combine(uint64_t& seed, uint64_t value) {
    seed ^= value + 0x9e3779b97f4a7c15ull + (seed << 6) + (seed >> 2);
}

/// Bit pattern of a float, with the two zeroes folded together so they hash alike
uint64_t hash_float(float v) {
    if (v == 0.0f) v = 0.0f;
    uint32_t bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    return bits;
}

} // namespace

uint64_t terrain_surface_fingerprint(const procgen::TerrainConfig& cfg) {
    // Only the fields TerrainGenerator::sample_surface() reads: the raw height
    // field, plus the urban flattening modifier, which is positioned from
    // size_x/size_z. Resolution, water level, erosion and mesh settings do not
    // change the surface a road is solved against, so a change to any of them
    // must NOT invalidate the solve.
    uint64_t h = 0xcbf29ce484222325ull;
    hash_combine(h, static_cast<uint64_t>(cfg.seed));
    hash_combine(h, static_cast<uint64_t>(cfg.type));
    hash_combine(h, hash_float(cfg.base_height));
    hash_combine(h, hash_float(cfg.max_height));
    hash_combine(h, hash_float(cfg.noise_scale));
    hash_combine(h, static_cast<uint64_t>(cfg.octaves));
    hash_combine(h, hash_float(cfg.lacunarity));
    hash_combine(h, hash_float(cfg.persistence));
    hash_combine(h, static_cast<uint64_t>(cfg.flatten_center ? 1 : 0));
    hash_combine(h, hash_float(cfg.flatten_radius));
    hash_combine(h, hash_float(cfg.flatten_falloff));
    hash_combine(h, hash_float(cfg.size_x));
    hash_combine(h, hash_float(cfg.size_z));

    // Never collide with the "no terrain" sentinel.
    return h == 0 ? 1 : h;
}

// ============================================================================
// Construction
// ============================================================================

ImportPipeline::ImportPipeline(osm::QuadTree& quadtree,
                               procgen::TerrainTileManager& terrain,
                               WorkerHooks hooks)
    : quadtree_(quadtree), terrain_(terrain), hooks_(std::move(hooks)) {}

ImportPipeline::~ImportPipeline() = default;

void ImportPipeline::set_listener(Listener* listener) {
    listener_ = listener;
}

void ImportPipeline::set_road_options(const RoadOptions& options) {
    m_road_options = options;
}

// ============================================================================
// Notification helpers
// ============================================================================

void ImportPipeline::notify_stage() {
    if (listener_) listener_->on_stage(state_.stage, state_.fraction, state_.message);
}

void ImportPipeline::notify_log(LogLevel level, const std::string& text) {
    if (listener_) listener_->on_log(PipelineLogLine{level, text});
}

void ImportPipeline::log_info(const std::string& text) {
    spdlog::info("{}", text);
    notify_log(LogLevel::Info, text);
}

void ImportPipeline::log_warn(const std::string& text) {
    spdlog::warn("{}", text);
    notify_log(LogLevel::Warn, text);
}

void ImportPipeline::log_error(const std::string& text) {
    spdlog::error("{}", text);
    notify_log(LogLevel::Error, text);
}

// ============================================================================
// Terrain-aware roads: config and worker body
// ============================================================================

bool ImportPipeline::has_generated_terrain() const {
    return terrain_.generated_count() > 0;
}

uint64_t ImportPipeline::live_terrain_fingerprint() const {
    return (m_road_options.terrain_aware && m_road_options.chunked_terrain && has_generated_terrain())
               ? terrain_surface_fingerprint(terrain_.get_config().terrain)
               : 0;
}

bool ImportPipeline::has_roads() const {
    return m_parser.has_data() && !m_parser.get_data().roads.empty();
}

osm::road::HeightSampler ImportPipeline::make_height_sampler() const {
    if (!m_road_options.terrain_aware) return nullptr;

    // The legacy single-terrain path has no carve hook, so elevating roads
    // against it would leave them following a surface nothing ever cuts. Flat
    // roads on flat-looking terrain beat roads buried in an uncarved hillside.
    if (!m_road_options.chunked_terrain) return nullptr;
    if (!has_generated_terrain()) return nullptr;

    // The config the CHUNKS were generated from, not a live edit buffer: the
    // buffer may already hold settings nothing has generated yet, and solving
    // against a surface that does not exist would float every road.
    const procgen::TerrainConfig terrain = terrain_.get_config().terrain;

    // A generator of our own. TerrainTileManager's is private, and
    // TerrainGenerator::sample_surface() is only re-entrant as long as nothing
    // reseeds the same instance -- which generate_chunk() does, on the main
    // thread, while this sampler is being called from the solver's workers.
    // Seeded from the same config, a separate instance gives bit-identical
    // heights with none of that coupling. Held by shared_ptr so it outlives
    // the async build even if the terrain settings change meanwhile.
    auto generator = std::make_shared<procgen::TerrainGenerator>(terrain.seed);

    return [terrain, generator](double x, double y) -> float {
        // Roads carry LOCAL 2D metres, and so does the terrain height field:
        // its second argument is the same local y, NOT render-space Z. Both
        // sides negate independently on their way to render space, so the two
        // agree exactly when the second argument is passed through unchanged.
        return generator->sample_surface(terrain,
                                         static_cast<float>(x),
                                         static_cast<float>(y));
    };
}

osm::road::RoadNetworkConfig ImportPipeline::make_road_network_config() const {
    osm::road::RoadNetworkConfig cfg;
    cfg.height_sampler = make_height_sampler();
    cfg.solve_junctions = m_road_options.solve_junctions;
    cfg.emit_markings = m_road_options.emit_markings;
    cfg.emit_crossings = m_road_options.emit_crossings;
    cfg.emit_structures = m_road_options.emit_structures;
    cfg.reduce_tessellation = m_road_options.reduce_tessellation;
    return cfg;
}

ImportPipeline::RoadBuildResult ImportPipeline::run_road_build(const osm::ParsedOSMData& data,
                                                                const osm::road::RoadNetworkConfig& cfg) {
    RoadBuildResult result;
    result.elevated = static_cast<bool>(cfg.height_sampler);
    result.solved_junctions = cfg.solve_junctions;

    // Structures need a terrain height under the road, so the builder skips
    // them whatever the flag says when there is no sampler. Recording the
    // flag alone would leave the readout reporting a pass that never ran.
    result.emitted_markings = cfg.emit_markings;
    result.emitted_crossings = cfg.emit_crossings;
    result.emitted_structures = cfg.emit_structures && static_cast<bool>(cfg.height_sampler);

    osm::road::RoadNetworkBuilder builder;
    result.network = builder.build(data, cfg);

    // The builder owns the elevation solver and dies with this function, so
    // everything the caller reads out has to be copied here.
    const osm::road::RoadElevationSolver& solver = builder.elevation();
    if (solver.is_solved()) {
        result.elevation = solver.stats();
        for (const osm::road::EdgeElevation& edge : solver.edges()) {
            result.max_grade = std::max(result.max_grade, edge.max_grade_used);
        }
    } else {
        // A sampler was supplied but the solve did not produce a result (no
        // usable roads, or a centerline/graph size mismatch). The network is
        // flat, so say so rather than showing an empty elevation readout.
        result.elevated = false;
    }

    return result;
}

// ============================================================================
// Import
// ============================================================================

Launch ImportPipeline::begin_import(const std::filesystem::path& path, const ImportOptions& options) {
    if (state_.stage == ImportStage::Cancelling) {
        return Launch{false, "A cancelled import is still stopping"};
    }
    if (m_import_job) {
        log_warn("An OSM import is already running");
        return Launch{false, "An OSM import is already running"};
    }
    // The road worker reads m_parser's data by pointer, and a second import
    // would reassign that parser out from under it.
    if (m_road_build_future.valid() || m_carve_index_future.valid()) {
        log_warn("A road network build is still running; import refused");
        return Launch{false, "A road network build is still running; import refused"};
    }
    if (state_.export_running) {
        log_warn("A road export is still running; import refused");
        return Launch{false, "A road export is still running; import refused"};
    }

    m_road_options = options.roads;

    auto job = std::make_unique<OSMImportJob>();
    job->filepath = path.string();
    job->parser = std::make_unique<osm::OSMParser>();
    job->parser->set_config(options.parser);

    // The callback fires on the worker thread. Snapshot under the mutex; the
    // caller reads the same field on the main thread.
    OSMImportJob* j = job.get();
    job->parser->set_progress_callback([j](const osm::ParseProgress& p) {
        std::lock_guard<std::mutex> lock(j->mutex);
        j->progress = p;
    });

    // Safe to capture `j`: the job is only destroyed after its future is
    // ready, and ~OSMImportJob joins the worker before releasing anything it
    // touches.
    auto hook = hooks_.on_worker_start;
    job->future = std::async(std::launch::async, [j, hook]() {
        if (hook) hook(ImportStage::Parsing);
        return j->parser->parse(j->filepath);
    });

    m_import_job = std::move(job);
    state_.stage = ImportStage::Parsing;
    state_.message = "Parsing...";
    state_.fraction = 0.0f;
    state_.nodes_total = 0;
    state_.nodes_done = 0;
    m_import_pending_nodes.clear();

    log_info("Started async OSM import: " + path.string());
    notify_stage();

    return Launch{true, ""};
}

void ImportPipeline::tick_parse() {
    // Stage 1: parsing on the worker.
    {
        std::lock_guard<std::mutex> lock(m_import_job->mutex);
        const auto& p = m_import_job->progress;
        if (!p.message.empty()) state_.message = p.message;
        state_.fraction = p.percentage() / 100.0f;
    }

    if (m_import_job->future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return;  // still parsing
    }

    if (m_import_job->discarded) {
        // cancel() accepted a discard: the job owns its own parser, so
        // m_parser, the quadtree and the carve are untouched.
        m_import_job.reset();
        state_.stage = ImportStage::Cancelled;
        state_.message = "Import cancelled";
        notify_stage();
        return;
    }

    const bool ok = m_import_job->future.get();
    if (!ok) {
        const std::string err = m_import_job->parser->get_error();
        m_import_job.reset();
        state_.stage = ImportStage::Failed;
        state_.message = err;
        log_error("[OSM] Error: " + err);
        return;
    }

    // Hand the parsed data over on the main thread, then release the job.
    m_parser = std::move(*m_import_job->parser);
    m_import_job.reset();

    m_parser.log_statistics();
    m_parser.log_sample_data();

    // Stage 2: road network, solved once over the whole graph rather than per
    // quadtree leaf, so junctions, miters and profile transitions survive leaf
    // boundaries.
    launch_road_build(/*rebuild_only=*/false);
}

// ============================================================================
// Road rebuild request and launch
// ============================================================================

void ImportPipeline::launch_road_build(bool rebuild_only) {
    // Safe to hold a pointer into m_parser for the same reason import and the
    // Clear Data path do: begin_import() and clear() both refuse to run while
    // a road build is in flight.
    const osm::ParsedOSMData* parsed = &m_parser.get_data();

    state_.rebuild_only = rebuild_only;
    state_.stage = ImportStage::BuildingRoads;
    state_.message = rebuild_only ? "Re-solving the road network..." : "Building road network...";
    state_.fraction = 0.0f;
    state_.nodes_total = 0;
    state_.nodes_done = 0;
    m_import_pending_nodes.clear();
    m_road_build_discarded = false;

    // Both captures read the terrain config on THIS thread, in this
    // statement, so the sampler and the fingerprint describe one and the
    // same surface however the caller is driven while the build runs.
    auto hook = hooks_.on_worker_start;
    m_road_build_future = std::async(std::launch::async,
                                     [parsed,
                                      cfg = make_road_network_config(),
                                      surface = live_terrain_fingerprint(),
                                      hook]() {
        if (hook) hook(ImportStage::BuildingRoads);
        RoadBuildResult result = run_road_build(*parsed, cfg);
        result.terrain_fingerprint = surface;
        return result;
    });

    notify_stage();
    if (rebuild_only) {
        log_info("Rebuilding the road network against the current terrain");
    }
}

RebuildOutcome ImportPipeline::request_road_rebuild(RebuildPolicy policy) {
    if (m_import_job || m_road_build_future.valid() || m_carve_index_future.valid() ||
        state_.export_running) {
        // Deferred, not dropped: the refused request is the NEWER one, and
        // the build in flight is about to stamp a fingerprint that will then
        // look current. finish() picks this up.
        state_.rebuild_owed = true;
        log_warn("Road rebuild deferred: an import is already running");
        return RebuildOutcome::Deferred;
    }
    state_.rebuild_owed = false;

    if (!has_roads()) {
        return RebuildOutcome::NoRoads;
    }

    if (policy == RebuildPolicy::IfSurfaceChanged) {
        // The surface the roads WOULD be solved against now.
        const uint64_t surface = live_terrain_fingerprint();
        if (surface == state_.roads.terrain_fingerprint) {
            return RebuildOutcome::NotNeeded;  // already solved against exactly this surface
        }
    }

    launch_road_build(/*rebuild_only=*/true);
    return RebuildOutcome::Started;
}

void ImportPipeline::tick_roads() {
    if (!m_road_build_future.valid()) {
        // Cannot happen through the state machine above; recover rather than
        // wedge the pipeline in a stage that never advances.
        log_error("Road network stage entered with no job; aborting import");
        state_.stage = ImportStage::Failed;
        state_.message = "Road network build lost";
        notify_stage();
        return;
    }

    if (m_road_build_future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return;  // still solving
    }

    if (m_road_build_discarded) {
        // cancel() accepted a discard of a REBUILD: the previous network is
        // still in the quadtree and still matches m_parser.
        m_road_build_future.get();
        m_road_build_discarded = false;
        state_.rebuild_owed = false;
        state_.rebuild_only = false;
        state_.stage = ImportStage::Cancelled;
        notify_stage();
        return;
    }

    RoadBuildResult result = m_road_build_future.get();
    osm::road::RoadNetwork& network = result.network;

    state_.roads.have_stats = true;
    state_.roads.network = network.stats;
    state_.roads.elevation = result.elevation;
    state_.roads.junctions = network.junction_stats;
    state_.roads.max_grade = result.max_grade;
    state_.roads.built_on_terrain = result.elevated;
    state_.roads.solved_junctions = result.solved_junctions;
    state_.roads.emitted_markings = result.emitted_markings;
    state_.roads.emitted_crossings = result.emitted_crossings;
    state_.roads.emitted_structures = result.emitted_structures;
    state_.roads.portal_mouths = network.carve_portals.size();
    // The surface this build was SOLVED against, captured at launch. Reading
    // the terrain manager here instead would record whatever terrain is
    // loaded when the future happens to land, which can be a different
    // surface entirely if it was regenerated mid-build.
    state_.roads.terrain_fingerprint = result.elevated ? result.terrain_fingerprint : 0;

    char road_msg[320];
    snprintf(road_msg, sizeof(road_msg),
             "[OSM] Road network: %zu pieces, %zu triangles from %zu edges in %.0f ms",
             network.stats.pieces, network.stats.triangles, network.stats.edges,
             network.stats.build_ms);
    log_info(road_msg);

    if (result.elevated) {
        snprintf(road_msg, sizeof(road_msg),
                 "[OSM] Elevation: %zu edges solved in %.0f ms, %zu iterations, "
                 "max grade %.1f%%, %zu bridges, %zu tunnels",
                 network.stats.elevated_edges, network.stats.elevation_ms,
                 result.elevation.iterations, result.max_grade * 100.0f,
                 result.elevation.bridges, result.elevation.tunnels);
    } else {
        snprintf(road_msg, sizeof(road_msg),
                 "[OSM] Elevation: skipped, roads are flat (no terrain to follow)");
    }
    log_info(road_msg);

    if (result.solved_junctions) {
        snprintf(road_msg, sizeof(road_msg),
                 "[OSM] Junctions: %zu solved, %zu roundabouts, %zu tapers, "
                 "%zu dead ends, %zu degenerate, %zu over-trimmed edges in %.0f ms",
                 network.junction_stats.junctions, network.junction_stats.roundabouts,
                 network.junction_stats.tapers, network.junction_stats.dead_ends,
                 network.junction_stats.degenerate,
                 network.junction_stats.over_trimmed_edges,
                 network.stats.junction_ms);
    } else {
        snprintf(road_msg, sizeof(road_msg),
                 "[OSM] Junctions: solver off, ribbons run through every node");
    }
    log_info(road_msg);

    snprintf(road_msg, sizeof(road_msg),
             "[OSM] Detail: %zu marking pieces, %zu crossings, %zu bridges, "
             "%zu tunnels (%zu portal mouths), %zu sidewalk sides deduped",
             network.stats.markings_pieces, network.stats.crossings,
             network.stats.bridges, network.stats.tunnels,
             network.carve_portals.size(), network.stats.deduped_sidewalks);
    log_info(road_msg);

    // The corridors outlive the network: they are indexed and carved once the
    // meshes are done. Moved out before the pieces are handed to the
    // quadtree, because that call consumes the network.
    m_pending_carve = std::make_unique<procgen::CarveInput>();
    m_pending_carve->ribbons = std::move(network.carve_ribbons);
    m_pending_carve->discs   = std::move(network.carve_discs);
    m_pending_carve->portals = std::move(network.carve_portals);
    m_pending_carve->config  = m_carve_config;

    // Stage 3: spatial index. Blocking, but far cheaper than parsing, and it
    // mutates the quadtree, so it stays on the calling thread inside this
    // one tick().
    state_.stage = ImportStage::Indexing;
    state_.message = "Building spatial index...";
    state_.fraction = 0.0f;
    notify_stage();

    rebuild_index(std::move(network.pieces), !state_.rebuild_only);

    state_.stage = ImportStage::BuildingMeshes;
    state_.message = "Building meshes...";
    notify_stage();
}

// ============================================================================
// Spatial index rebuild
// ============================================================================

void ImportPipeline::rebuild_index(std::vector<osm::road::RoadPiece>&& road_pieces,
                                   bool recenter_camera) {
    const auto& osm_data = m_parser.get_data();

    log_info("Initializing quadtree...");

    // Every leaf about to be destroyed may still own GPU resources the host
    // tracks; the host releases them here, before the tree is torn down.
    if (listener_) listener_->on_before_quadtree_reset();

    quadtree_.clear();
    // Sized from the features, not osm_data.bounds -- see QuadTree::init(ParsedOSMData).
    quadtree_.init(osm_data);
    quadtree_.assign_data(osm_data);

    // Roads are not rebuilt per leaf. Hand the already-solved geometry over
    // now, while the leaves exist and before any node build is queued, so
    // the first upload of a leaf already carries its roads.
    //
    // Set BEFORE the hand-off: the flag decides how pieces are routed into
    // the leaves as well as whether a chain is built afterwards, and both
    // happen inside assign_road_pieces().
    quadtree_.set_chunk_lod(m_road_options.chunk_lod, osm::road::ChunkLodConfig{});
    quadtree_.assign_road_pieces(std::move(road_pieces));

    state_.quadtree.leaf_count = quadtree_.leaf_count();
    state_.quadtree.total_roads = quadtree_.total_roads();
    state_.quadtree.total_buildings = quadtree_.total_buildings();
    state_.quadtree.total_areas = quadtree_.total_areas();
    state_.quadtree.max_depth = quadtree_.max_depth();

    char msg[256];
    snprintf(msg, sizeof(msg), "QuadTree: %zu leaves, %zu roads, %zu buildings, %zu areas, max depth %u",
             state_.quadtree.leaf_count, state_.quadtree.total_roads,
             state_.quadtree.total_buildings, state_.quadtree.total_areas,
             static_cast<unsigned>(state_.quadtree.max_depth));
    log_info(msg);

    if (listener_) listener_->on_quadtree_ready(recenter_camera);

    // Queue the initially-visible leaves. These builds are already
    // asynchronous; tick() drains them across calls and reports progress.
    m_import_pending_nodes.clear();

    const std::optional<InitialView> view = listener_ ? listener_->initial_view() : std::nullopt;

    if (view) {
        quadtree_.traverse_visible(
            view->frustum_planes, view->camera_position, view->view_radius,
            600.0f, view->fov_y, view->contribution_threshold,
            true, true, false, // frustum + distance, no contribution cull
            [this](osm::QuadTreeNode* node, float /*dist_sq*/) {
                if (!node->meshes_built && !node->meshes_pending) {
                    if (quadtree_.queue_node_build_async(node)) {
                        m_import_pending_nodes.push_back(node);
                    }
                }
            });
    } else {
        // No view supplied (headless/test): queue every leaf.
        for (auto* node : quadtree_.get_all_leaves()) {
            if (node && !node->meshes_built && !node->meshes_pending) {
                if (quadtree_.queue_node_build_async(node)) {
                    m_import_pending_nodes.push_back(node);
                }
            }
        }
    }

    state_.nodes_total = m_import_pending_nodes.size();
    state_.nodes_done = 0;
    snprintf(msg, sizeof(msg), "Queued %zu initial node builds", state_.nodes_total);
    log_info(msg);
}

// ============================================================================
// Mesh build drain
// ============================================================================

void ImportPipeline::tick_meshes() {
    // Also polled by a host's own render loop, but tick() does it too so
    // progress advances even when nothing else is polling the quadtree.
    quadtree_.poll_async_builds();

    size_t done = 0;
    for (const auto* node : m_import_pending_nodes) {
        if (node->meshes_built) ++done;
    }

    state_.nodes_done = done;
    state_.fraction = state_.nodes_total > 0
        ? static_cast<float>(done) / static_cast<float>(state_.nodes_total)
        : 1.0f;

    if (done < state_.nodes_total) {
        return;
    }

    // Meshes done: hand the corridors to the terrain.
    m_import_pending_nodes.clear();

    const auto& data = m_parser.get_data();
    char msg[256];
    snprintf(msg, sizeof(msg), "[OSM] Loaded: %zu roads, %zu buildings, %zu areas",
             data.roads.size(), data.buildings.size(), data.areas.size());
    log_info(msg);

    state_.stage = ImportStage::CarvingTerrain;
    state_.fraction = 0.0f;
    state_.message = "Preparing terrain carve...";
    notify_stage();

    begin_carve();
}

// ============================================================================
// Terrain carve stage
// ============================================================================

void ImportPipeline::begin_carve() {
    m_carve_apply_pending = false;

    const bool have_corridors = m_pending_carve && m_pending_carve->item_count() > 0;

    if (!have_corridors) {
        // Roads were built flat, so there is nothing to carve. Any carve data
        // still installed belongs to a network that no longer exists, so
        // drop it. Dropping it regenerates the affected chunks, which is the
        // same blocking work an install is, so it goes through the same
        // deferred-by-one-tick path.
        m_pending_carve.reset();
        if (terrain_.has_road_carve_data()) {
            state_.message = "Clearing terrain carve...";
            m_carve_apply_pending = true;
        } else {
            finish();
        }
        return;
    }

    // Indexing is proportional to the number of corridors, which is
    // proportional to the size of the import, so it goes on a worker like
    // every other whole-network pass. It touches nothing the main thread reads.
    state_.message = "Indexing road corridors...";
    auto hook = hooks_.on_worker_start;
    m_carve_index_future = std::async(std::launch::async,
                                      [input = std::move(m_pending_carve), hook]() mutable {
        if (hook) hook(ImportStage::CarvingTerrain);
        input->build_index();
        return std::move(input);
    });
}

void ImportPipeline::tick_carve() {
    // Second half of the deferred apply: the stage message set last tick has
    // been observed, so the blocking regeneration may now run.
    if (m_carve_apply_pending) {
        m_carve_apply_pending = false;
        install_carve();
        finish();
        return;
    }

    if (!m_carve_index_future.valid()) {
        // No index job and no pending apply: nothing left to do in this stage.
        finish();
        return;
    }

    if (m_carve_index_future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return;  // still indexing
    }

    m_pending_carve = m_carve_index_future.get();

    // Apply on the NEXT tick(). set_road_carve_data() regenerates every
    // existing chunk in one call; doing that in the tick that names the
    // stage means the name never reaches a listener before the hitch.
    state_.message = "Carving terrain...";
    m_carve_apply_pending = true;
}

void ImportPipeline::install_carve() {
    if (!m_pending_carve) {
        if (terrain_.has_road_carve_data()) {
            terrain_.clear_road_carve_data();
            log_info("[Terrain] Road carve cleared; terrain returned to procedural");
            if (listener_) listener_->on_carve_installed(false);
        }
        return;
    }

    char msg[256];
    snprintf(msg, sizeof(msg), "[Terrain] Carving %zu corridors and %zu junctions into %zu chunks",
             m_pending_carve->ribbons.size(), m_pending_carve->discs.size(),
             terrain_.chunk_count());
    log_info(msg);

    // Regenerates every already-generated chunk, which is why this runs on
    // the calling thread: a host's render loop walks those chunks every
    // frame. Chunks generated LATER pick the carve up inside generate_chunk(),
    // so an install with no terrain yet is nearly free and still correct.
    terrain_.set_road_carve_data(std::move(*m_pending_carve));
    m_pending_carve.reset();
    if (listener_) listener_->on_carve_installed(true);
}

void ImportPipeline::finish() {
    m_pending_carve.reset();
    m_carve_apply_pending = false;
    state_.stage = ImportStage::Done;
    state_.fraction = 1.0f;
    state_.message = state_.rebuild_only ? "Roads re-solved against the terrain"
                                         : "Import successful";
    state_.rebuild_only = false;
    log_info("OSM import complete");
    notify_stage();

    // A rebuild asked for while this one was in flight. Honoured through
    // request_road_rebuild(IfSurfaceChanged) rather than launched directly,
    // so a request the just-landed build already satisfied costs nothing.
    if (state_.rebuild_owed) {
        state_.rebuild_owed = false;
        request_road_rebuild(RebuildPolicy::IfSurfaceChanged);
    }
}

// ============================================================================
// Export
// ============================================================================

Launch ImportPipeline::begin_export(const ExportOptions& options) {
    if (state_.export_running) {
        return Launch{false, ""};
    }
    if (m_import_job || m_road_build_future.valid() || m_carve_index_future.valid()) {
        return Launch{false, "An import is still running"};
    }
    if (!has_roads()) {
        return Launch{false, "No road data to export"};
    }
    if (options.dir[0] == '\0') {
        return Launch{false, "Choose an output directory first"};
    }

    // Safe to hold a pointer into m_parser for the same reason the import's
    // road stage does: begin_import() and clear() both refuse to run while
    // this future is valid.
    const osm::ParsedOSMData* parsed = &m_parser.get_data();

    // The network is re-solved rather than kept: an import MOVES its pieces
    // into the quadtree, which merges them per leaf and keeps only the
    // render mesh, so the collision variant and the LOD chain no longer
    // exist by the time anyone asks to export them.
    osm::road::RoadNetworkConfig cfg = make_road_network_config();
    cfg.build_collision = options.build_collision;
    cfg.build_lods = options.build_lods;

    auto job = std::make_unique<RoadExportJob>();
    job->directory = options.dir;
    job->config = options.config;
    // The exporter only writes what the build produced, so the two pairs of
    // flags are one decision and are stamped together.
    job->config.export_collision = options.build_collision;
    job->config.export_lods = options.build_lods;
    job->build_collision = options.build_collision;
    job->build_lods = options.build_lods;

    const std::string dir = job->directory;
    const osm::road::ExportConfig export_cfg = job->config;

    auto hook = hooks_.on_export_worker_start;
    job->future = std::async(std::launch::async, [parsed, cfg, export_cfg, dir, hook]() {
        if (hook) hook();
        osm::road::RoadNetworkBuilder builder;
        osm::road::RoadNetwork network = builder.build(*parsed, cfg);
        return osm::road::export_road_network(network.pieces, std::filesystem::path(dir),
                                              export_cfg);
    });

    m_export_job = std::move(job);
    state_.export_running = true;
    log_info("Exporting the road network to " + dir);

    return Launch{true, ""};
}

void ImportPipeline::tick_export() {
    if (!m_export_job) return;

    if (!m_export_job->future.valid()) {
        m_export_job.reset();
        state_.export_running = false;
        return;
    }
    if (m_export_job->future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return;  // still running
    }

    osm::road::ExportStats stats;
    std::string failure;
    try {
        stats = m_export_job->future.get();
    } catch (const std::exception& e) {
        // std::filesystem throws on an unwritable or unreachable destination,
        // and an uncaught exception out of a future's get() would take the
        // caller with it.
        failure = e.what();
    }

    const std::string directory = m_export_job->directory;
    const bool wrote_collision = m_export_job->build_collision;
    const bool wrote_lods = m_export_job->build_lods;
    m_export_job.reset();
    state_.export_running = false;

    ExportResult result;
    result.stats = stats;
    result.failure = failure;
    result.directory = directory;
    result.wrote_collision = wrote_collision;
    result.wrote_lods = wrote_lods;

    char msg[512];
    if (!failure.empty()) {
        result.status = "Export failed: " + failure;
        snprintf(msg, sizeof(msg), "[Export] Failed: %s", failure.c_str());
        log_error(msg);
    } else if (stats.files == 0) {
        // Not an exception: every file was refused, or the network held no
        // triangles. Either way nothing reached the disk, and saying "done"
        // would be a lie only discovered in the file manager.
        result.status = "Export wrote no files - check the destination is writable";
        snprintf(msg, sizeof(msg), "[Export] Wrote no files to %s", directory.c_str());
        log_warn(msg);
    } else {
        snprintf(msg, sizeof(msg),
                 "[Export] %zu chunk(s), %zu meshes, %zu triangles, %zu vertices, %zu file(s) "
                 "in %.0f ms -> %s%s%s",
                 stats.chunks, stats.meshes, stats.triangles, stats.vertices, stats.files,
                 stats.export_ms, directory.c_str(),
                 wrote_collision ? " (+collision)" : "",
                 wrote_lods ? " (+LODs)" : "");
        log_info(msg);
        char status[256];
        snprintf(status, sizeof(status), "Wrote %zu file(s), %zu triangles in %.0f ms",
                 stats.files, stats.triangles, stats.export_ms);
        result.status = status;
    }

    if (listener_) listener_->on_export_done(result);
}

// ============================================================================
// tick(), cancel(), clear()
// ============================================================================

void ImportPipeline::tick() {
    tick_export();

    if (m_import_job) {
        tick_parse();
        return;
    }
    if (m_road_build_future.valid()) {
        tick_roads();
        return;
    }
    if (state_.stage == ImportStage::CarvingTerrain) {
        tick_carve();
        return;
    }
    if (state_.stage == ImportStage::BuildingMeshes) {
        tick_meshes();
        return;
    }
}

bool ImportPipeline::cancel() {
    // No core worker can be interrupted, so a cancel is a discard: the
    // worker runs to its end, tick() collects the result with wait_for(0)
    // and throws it away. This function itself never blocks and never
    // destroys a future that is still running.
    if (state_.export_running) {
        return false;  // the worker writes files; a discarded result would hide a partial write
    }

    if (m_import_job) {
        if (state_.stage == ImportStage::Cancelling) return false;  // already cancelling
        m_import_job->discarded = true;
        state_.stage = ImportStage::Cancelling;
        notify_stage();
        return true;
    }

    if (m_road_build_future.valid()) {
        if (!state_.rebuild_only) {
            // A fresh import's road stage: m_parser already holds the NEW
            // parse while the quadtree holds the OLD one. Discarding here
            // would leave them inconsistent.
            return false;
        }
        if (state_.stage == ImportStage::Cancelling) return false;  // already cancelling
        m_road_build_discarded = true;
        state_.stage = ImportStage::Cancelling;
        notify_stage();
        return true;
    }

    // Indexing, BuildingMeshes, CarvingTerrain: the quadtree is already
    // replaced or a carve does not yet match the roads. Idle, Done, Failed,
    // Cancelled: nothing to cancel.
    return false;
}

bool ImportPipeline::clear() {
    if (state_.busy()) {
        return false;
    }

    // Every leaf about to be destroyed may still own resources a host
    // tracks; give it the chance to release them first.
    if (listener_) listener_->on_before_quadtree_reset();

    state_.stage = ImportStage::Idle;
    state_.fraction = 0.0f;
    state_.message.clear();
    state_.rebuild_only = false;
    state_.rebuild_owed = false;
    state_.nodes_done = 0;
    state_.nodes_total = 0;
    state_.roads = RoadBuildStats{};
    state_.quadtree = QuadTreeSummary{};

    m_import_pending_nodes.clear();
    m_parser.clear();
    quadtree_.clear();

    // The carve describes a road network that no longer exists, so leaving
    // it installed would keep cutting trenches for roads that were just
    // deleted. Dropping it regenerates the affected chunks.
    m_pending_carve.reset();
    m_carve_apply_pending = false;
    terrain_.clear_road_carve_data();

    notify_stage();
    return true;
}

} // namespace stratum::app
