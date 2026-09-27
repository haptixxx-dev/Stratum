// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "editor/editor.hpp"
#include "editor/im3d_impl.hpp"
#include "renderer/gpu_renderer.hpp"
#include "renderer/material_library.hpp"
#include "renderer/procedural_texture.hpp"
#include "renderer/texture.hpp"
#include <im3d.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <spdlog/spdlog.h>
#include <SDL3/SDL.h>
#include <map>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <utility>

namespace stratum {

// Defined here, not in the header: the unique_ptr members hold types that are
// only forward-declared there. See the note on ~Editor().
Editor::Editor() = default;
Editor::~Editor() = default;

void Editor::init() {
    spdlog::info("Editor initialized");
    Im3D_Init();
}

void Editor::set_renderer(GPURenderer* renderer) {
    m_gpu_renderer = renderer;
    if (renderer) {
        // Non-fatal: on failure Im3d simply renders nothing.
        Im3D_InitGPU(*renderer);

        // The renderer owns the budget and the eviction mechanism; the editor owns
        // the answers to "how far away is this?" and "your handle is gone". Without
        // both installed the renderer refuses to evict at all, because evicting
        // without a distance discards the road under the camera as readily as one
        // on the horizon.
        renderer->set_mesh_distance_fn([this](uint32_t id) { return mesh_distance_to_camera(id); });
        renderer->set_mesh_evicted_fn([this](uint32_t id) { on_mesh_evicted(id); });

        init_materials(*renderer);
    }
}

void Editor::init_materials(GPURenderer& renderer) {
    SDL_GPUDevice* device = renderer.get_device();
    if (!device) {
        spdlog::warn("No GPU device; roads will draw untextured");
        return;
    }

    // Every failure below is NON-FATAL and leaves the renderer with no material
    // library installed, which it treats as "draw exactly as before materials
    // existed". A broken material set must degrade to the old untextured look,
    // never to a black screen or a missing scene.
    auto textures = std::make_unique<GPUTextureManager>();
    if (!textures->init(device)) {
        spdlog::error("Texture manager init failed; roads will draw untextured");
        return;
    }

    auto materials = std::make_unique<MaterialLibrary>();
    if (!materials->init(textures.get())) {
        spdlog::error("Material library init failed; roads will draw untextured");
        textures->shutdown();
        return;
    }

    // The frozen slot table first, so every MaterialId is at least the right
    // colour and roughness even if texture generation below fails.
    materials->load_defaults();

    // Then the generated tiling detail. install_procedural_textures() also
    // installs the variant table, because a cobblestone variant without its stone
    // texture is only a slightly different shade of grey. Failure here is
    // survivable: the flat defaults above remain.
    if (!materials->install_procedural_textures()) {
        spdlog::warn("Procedural texture generation failed; materials stay flat");
    }

    m_texture_manager = std::move(textures);
    m_material_library = std::move(materials);

    // Order matters only in that both must be installed before the first draw.
    renderer.set_texture_manager(m_texture_manager.get());
    renderer.set_material_library(m_material_library.get());

    // Materials are a PBR-ONLY path by construction: the simple shader declares no
    // material uniform block and no samplers, so GPURenderer::bind_material()
    // returns immediately in ShaderMode::Simple. The renderer starts in Simple, so
    // without this the whole material system would be installed, populated, and
    // completely invisible until someone found the Render Settings combo -- which
    // is exactly the "threaded through and never read" failure this phase exists to
    // end. Switching here, and only once the library actually came up, is what
    // makes the world materially distinct at startup.
    //
    // set_shader_mode() refuses if the PBR pipelines failed to build and says so;
    // that leaves the editor in Simple mode drawing untextured, which is the
    // correct degraded state rather than a black screen.
    if (!renderer.set_shader_mode(ShaderMode::PBR)) {
        spdlog::warn("Materials installed but PBR is unavailable; roads draw untextured");
    }

    // The panel is worth opening by default the first time there is something in
    // it. It is a normal dockable panel afterwards and remembers nothing, so this
    // costs a keystroke to undo and saves a hunt through the View menu.
    m_show_material_panel = true;

    const auto stats = m_texture_manager->stats();
    spdlog::info("Materials ready: {} materials, {} textures, {} KB",
                 m_material_library->size(), stats.textures, stats.bytes / 1024);
}

void Editor::publish_camera(GPURenderer& renderer) {
    renderer.set_view_projection(m_camera.get_view(), m_camera.get_projection());
    renderer.set_camera_position(m_camera.get_position());
}

void Editor::im3d_end_frame_and_upload(GPURenderer& renderer) {
    Im3D_EndFrameAndUpload(renderer);
}

void Editor::shutdown() {
    Im3D_Shutdown();

    // Before GPURenderer::shutdown() destroys the device these textures and
    // samplers belong to. Application calls us first, which is what makes this
    // the right place; the renderer also tears them down defensively if some
    // other caller skips this path.
    if (m_gpu_renderer) {
        m_gpu_renderer->set_material_library(nullptr);
        m_gpu_renderer->set_texture_manager(nullptr);
    }
    if (m_material_library) {
        m_material_library->shutdown();
        m_material_library.reset();
    }
    if (m_texture_manager) {
        m_texture_manager->shutdown();
        m_texture_manager.reset();
    }

    spdlog::info("Editor shutdown");
}

void Editor::update() {
    // Update visible tile batches based on camera position
    // Note: Camera matrices are updated in draw_viewport, so we rebuild batches there
    // to ensure frustum is current
}

void Editor::render() {
    // Invalidate the viewport rect every frame. draw_viewport() republishes it
    // below, but only when the panel is actually drawn -- so if the Viewport is
    // closed, this leaves it zeroed and render_3d() bails out instead of
    // rendering the whole 3D scene into a stale rect.
    m_viewport_rect = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);

    // Global keyboard shortcuts
    if (ImGui::IsKeyPressed(ImGuiKey_F11)) {
        toggle_fullscreen();
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
        // Escape leaves fullscreen first; quitting outright is a nasty surprise
        // when the window is covering the whole screen.
        if (m_fullscreen) {
            toggle_fullscreen();
        } else if (m_quit_callback) {
            m_quit_callback();
        }
    }

    // Handle window resizing from edges
    handle_window_resize();

    // Advance any in-flight OSM import. Must run before the panels draw so the
    // progress bar reflects this frame's state.
    poll_osm_import();

    // Apply a native file-dialog result on the main thread; the SDL callback that
    // produced it may have run on another one.
    poll_file_dialog();
    poll_export_dir_dialog();

    // Advance an in-flight export. Same rule as the import: before the panels draw.
    poll_road_export();

    setup_dockspace();

    
    // Update Camera (moved to draw_viewport to sync with focus, but could be here)
    // We do it in draw_viewport to update aspects correctly

    if (m_show_demo_window) {
        ImGui::ShowDemoWindow(&m_show_demo_window);
    }

    if (m_show_style_editor) {
        ImGui::Begin("Style Editor", &m_show_style_editor);
        ImGui::ShowStyleEditor();
        ImGui::End();
    }

    if (m_show_viewport) draw_viewport();
    if (m_show_scene_hierarchy) draw_scene_hierarchy();
    if (m_show_properties) draw_properties();
    if (m_show_console) draw_console();
    if (m_show_osm_panel) draw_osm_panel();
    if (m_show_procgen_panel) draw_procgen_panel();
    if (m_show_render_settings) draw_render_settings();
    if (m_show_memory_panel) draw_memory_panel();
    if (m_show_material_panel) draw_material_panel();
    if (m_show_rule_panel) draw_rule_panel();
}

// ============================================================================
// Road network export
// ============================================================================

void Editor::begin_road_export() {
    if (export_in_flight()) {
        return;
    }
    if (m_import_job || m_road_build_future.valid() || m_carve_index_future.valid()) {
        m_export_status = "An import is still running";
        return;
    }
    if (!m_osm_parser.has_data() || m_osm_parser.get_data().roads.empty()) {
        m_export_status = "No road data to export";
        return;
    }
    if (m_export_dir[0] == '\0') {
        m_export_status = "Choose an output directory first";
        return;
    }

    // Safe to hold a pointer into m_osm_parser for the same reason the import's
    // road stage does: begin_osm_import(), begin_road_network_rebuild() and the
    // Clear Data button all refuse to run while this future is valid.
    const osm::ParsedOSMData* parsed = &m_osm_parser.get_data();

    // The network is re-solved rather than kept: an import MOVES its pieces into
    // the quadtree, which merges them per leaf and keeps only the render mesh, so
    // the collision variant and the LOD chain no longer exist by the time anyone
    // asks to export them. Solving again is a second of worker time; holding a
    // second copy of a city's geometry is permanent.
    osm::road::RoadNetworkConfig cfg = make_road_network_config();
    cfg.build_collision = m_export_build_collision;
    cfg.build_lods = m_export_build_lods;

    auto job = std::make_unique<RoadExportJob>();
    job->directory = m_export_dir;
    job->config = m_export_config;
    // The exporter only writes what the build produced, so the two pairs of flags
    // are one decision and are stamped together.
    job->config.export_collision = m_export_build_collision;
    job->config.export_lods = m_export_build_lods;
    job->build_collision = m_export_build_collision;
    job->build_lods = m_export_build_lods;

    const std::string dir = job->directory;
    const osm::road::ExportConfig export_cfg = job->config;

    // Off the UI thread, like the import. The solve alone is seconds on a city
    // extract and the write is hundreds of megabytes of file I/O.
    job->future = std::async(std::launch::async, [parsed, cfg, export_cfg, dir]() {
        osm::road::RoadNetworkBuilder builder;
        osm::road::RoadNetwork network = builder.build(*parsed, cfg);
        return osm::road::export_road_network(network.pieces, std::filesystem::path(dir),
                                              export_cfg);
    });

    m_export_job = std::move(job);
    m_export_status = "Exporting...";
    spdlog::info("Exporting the road network to {}", m_export_dir);
}

void Editor::poll_road_export() {
    if (!m_export_job) return;

    if (!m_export_job->future.valid()) {
        m_export_job.reset();
        return;
    }
    if (m_export_job->future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return;  // still running; the indeterminate bar keeps animating
    }

    osm::road::ExportStats stats;
    std::string failure;
    try {
        stats = m_export_job->future.get();
    } catch (const std::exception& e) {
        // std::filesystem throws on an unwritable or unreachable destination, and
        // an uncaught exception out of a future's get() takes the editor with it.
        failure = e.what();
    }

    const std::string directory = m_export_job->directory;
    const bool wrote_collision = m_export_job->build_collision;
    const bool wrote_lods = m_export_job->build_lods;
    m_export_job.reset();

    char msg[512];
    if (!failure.empty()) {
        m_export_status = "Export failed: " + failure;
        snprintf(msg, sizeof(msg), "[Export] Failed: %s\n", failure.c_str());
    } else if (stats.files == 0) {
        // Not an exception: every file was refused, or the network held no
        // triangles. Either way nothing reached the disk, and saying "done" would
        // be a lie the user only discovers in the file manager.
        m_export_status = "Export wrote no files - check the destination is writable";
        snprintf(msg, sizeof(msg), "[Export] Wrote no files to %s\n", directory.c_str());
    } else {
        snprintf(msg, sizeof(msg),
                 "[Export] %zu chunk(s), %zu meshes, %zu triangles, %zu vertices, %zu file(s) "
                 "in %.0f ms -> %s%s%s\n",
                 stats.chunks, stats.meshes, stats.triangles, stats.vertices, stats.files,
                 stats.export_ms, directory.c_str(),
                 wrote_collision ? " (+collision)" : "",
                 wrote_lods ? " (+LODs)" : "");
        char status[256];
        snprintf(status, sizeof(status), "Wrote %zu file(s), %zu triangles in %.0f ms",
                 stats.files, stats.triangles, stats.export_ms);
        m_export_status = status;
    }

    m_console_buffer.append(msg);
    m_console_scroll_to_bottom = true;
    spdlog::info("{}", msg);
}

// ============================================================================
// Terrain-aware roads
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

uint64_t Editor::terrain_surface_fingerprint(const procgen::TerrainConfig& cfg) {
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

bool Editor::has_generated_terrain() const {
    return m_terrain_tile_manager.generated_count() > 0;
}

uint64_t Editor::live_road_terrain_fingerprint() const {
    return (m_terrain_aware_roads && m_use_chunked_terrain && has_generated_terrain())
               ? terrain_surface_fingerprint(m_terrain_tile_manager.get_config().terrain)
               : 0;
}

osm::road::HeightSampler Editor::make_terrain_height_sampler() const {
    if (!m_terrain_aware_roads) return nullptr;

    // The legacy single-terrain path has no carve hook, so elevating roads
    // against it would leave them following a surface nothing ever cuts. Flat
    // roads on flat-looking terrain beat roads buried in an uncarved hillside.
    if (!m_use_chunked_terrain) return nullptr;
    if (!has_generated_terrain()) return nullptr;

    // The config the CHUNKS were generated from, not the panel's live edit
    // buffer: the buffer may already hold settings the user has not pressed
    // Generate on, and solving against a surface that does not exist yet would
    // float every road.
    const procgen::TerrainConfig terrain = m_terrain_tile_manager.get_config().terrain;

    // A generator of our own. TerrainTileManager's is private, and
    // TerrainGenerator::sample_surface() is only re-entrant as long as nothing
    // reseeds the same instance -- which generate_chunk() does, on the main
    // thread, while this sampler is being called from the solver's workers.
    // Seeded from the same config, a separate instance gives bit-identical
    // heights with none of that coupling. Held by shared_ptr so it outlives the
    // async build even if the editor's terrain settings change meanwhile.
    auto generator = std::make_shared<procgen::TerrainGenerator>(terrain.seed);

    return [terrain, generator](double x, double y) -> float {
        // Roads carry LOCAL 2D metres, and so does the terrain height field: its
        // second argument is the same local y, NOT render-space Z. Both sides
        // negate independently on their way to render space -- terrain through
        // TerrainMeshBuilder's vec3(world_x, h, -world_z), roads through
        // vec3(x, h, -y_2d) -- so the two agree exactly when the second argument
        // is passed through unchanged.
        //
        // Negate it and every road is elevated from the height at its own mirror
        // image across the equator of the import: still smooth, still plausible,
        // and completely wrong everywhere the terrain is not symmetric.
        return generator->sample_surface(terrain,
                                         static_cast<float>(x),
                                         static_cast<float>(y));
    };
}

osm::road::RoadNetworkConfig Editor::make_road_network_config() const {
    osm::road::RoadNetworkConfig cfg;
    cfg.height_sampler = make_terrain_height_sampler();
    cfg.solve_junctions = m_solve_junctions;
    cfg.emit_markings = m_emit_markings;
    cfg.emit_crossings = m_emit_crossings;
    cfg.emit_structures = m_emit_structures;
    cfg.reduce_tessellation = m_reduce_tessellation;
    return cfg;
}

Editor::RoadBuildResult Editor::run_road_network_build(const osm::ParsedOSMData& data,
                                                       const osm::road::RoadNetworkConfig& cfg) {
    RoadBuildResult result;
    result.elevated = static_cast<bool>(cfg.height_sampler);
    result.solved_junctions = cfg.solve_junctions;

    // Structures need a terrain height under the road, so the builder skips them
    // whatever the flag says when there is no sampler. Recording the flag alone
    // would leave the panel reporting a pass that never ran.
    result.emitted_markings = cfg.emit_markings;
    result.emitted_crossings = cfg.emit_crossings;
    result.emitted_structures = cfg.emit_structures && static_cast<bool>(cfg.height_sampler);

    osm::road::RoadNetworkBuilder builder;
    result.network = builder.build(data, cfg);

    // The builder owns the elevation solver and dies with this function, so
    // everything the panel reads out has to be copied here.
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

void Editor::begin_road_network_rebuild() {
    if (m_import_job || m_road_build_future.valid() || m_carve_index_future.valid() ||
        export_in_flight()) {
        // Deferred, not dropped: the refused request is the NEWER one, and the
        // build in flight is about to stamp a fingerprint that will then look
        // current. finish_osm_import() picks this up.
        m_road_rebuild_owed = true;
        spdlog::warn("Road rebuild deferred: an import is already running");
        return;
    }
    m_road_rebuild_owed = false;
    if (!m_osm_parser.has_data() || m_osm_parser.get_data().roads.empty()) {
        return;
    }

    // Safe to hold a pointer into m_osm_parser for the same reason the import
    // path does: begin_osm_import() and the Clear Data button both refuse to run
    // while a road build is in flight.
    const osm::ParsedOSMData* parsed = &m_osm_parser.get_data();

    m_road_rebuild_only = true;
    m_import_stage = ImportStage::BuildingRoads;
    m_import_message = "Re-solving the road network...";
    m_import_fraction = 0.0f;
    m_import_nodes_total = 0;
    m_import_pending_nodes.clear();

    // Both captures read the terrain config on THIS thread, in this statement, so
    // the sampler and the fingerprint describe one and the same surface however
    // the panel is driven while the build runs.
    m_road_build_future = std::async(std::launch::async,
                                     [parsed,
                                      cfg = make_road_network_config(),
                                      surface = live_road_terrain_fingerprint()]() {
        RoadBuildResult result = run_road_network_build(*parsed, cfg);
        result.terrain_fingerprint = surface;
        return result;
    });

    spdlog::info("Rebuilding the road network against the current terrain");
}

void Editor::maybe_rebuild_roads_for_terrain() {
    if (!m_osm_parser.has_data() || m_osm_parser.get_data().roads.empty()) {
        return;
    }

    // The surface the roads WOULD be solved against now.
    const uint64_t surface = live_road_terrain_fingerprint();

    if (surface == m_road_terrain_fingerprint) {
        return;  // already solved against exactly this surface
    }

    // Rebuilding beats telling the user to re-import. The parsed OSM data is
    // already in memory and only the vertical solve is stale, so a rebuild costs
    // the road pass alone; a re-import costs re-parsing a file that may be
    // hundreds of megabytes. It also cannot be skipped: a network built with no
    // sampler emits NO carve data at all, so without this the terrain would have
    // nothing to carve and every road would sit buried in the hillside.
    begin_road_network_rebuild();
}

void Editor::begin_osm_import(const std::string& filepath, const osm::ParserConfig& config) {
    if (m_import_job) {
        spdlog::warn("An OSM import is already running");
        return;
    }
    // The road worker reads m_osm_parser's data by pointer, and a second import
    // would reassign that parser out from under it.
    if (m_road_build_future.valid() || m_carve_index_future.valid()) {
        spdlog::warn("A road network build is still running; import refused");
        return;
    }
    if (export_in_flight()) {
        spdlog::warn("A road export is still running; import refused");
        return;
    }

    auto job = std::make_unique<OSMImportJob>();
    job->filepath = filepath;
    job->parser = std::make_unique<osm::OSMParser>();
    job->parser->set_config(config);

    // The callback fires on the worker thread. Snapshot under the mutex; the UI
    // reads the same field on the main thread.
    OSMImportJob* j = job.get();
    job->parser->set_progress_callback([j](const osm::ParseProgress& p) {
        std::lock_guard<std::mutex> lock(j->mutex);
        j->progress = p;
    });

    // Safe to capture `j`: the job is only destroyed after its future is ready,
    // and ~OSMImportJob joins the worker before releasing anything it touches.
    job->future = std::async(std::launch::async, [j]() {
        return j->parser->parse(j->filepath);
    });

    m_import_job = std::move(job);
    m_import_stage = ImportStage::Parsing;
    m_import_message = "Parsing...";
    m_import_fraction = 0.0f;
    m_import_nodes_total = 0;
    m_import_pending_nodes.clear();

    spdlog::info("Started async OSM import: {}", filepath);
}

void Editor::poll_osm_import() {
    if (m_import_job) {
        // ── Stage 1: parsing on the worker ──
        {
            std::lock_guard<std::mutex> lock(m_import_job->mutex);
            const auto& p = m_import_job->progress;
            if (!p.message.empty()) m_import_message = p.message;
            m_import_fraction = p.percentage() / 100.0f;
        }

        if (m_import_job->future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            return;  // still parsing; the UI keeps animating
        }

        const bool ok = m_import_job->future.get();
        if (!ok) {
            const std::string err = m_import_job->parser->get_error();
            m_import_job.reset();
            m_import_stage = ImportStage::Failed;
            m_import_message = err;

            char msg[512];
            snprintf(msg, sizeof(msg), "[OSM] Error: %s\n", err.c_str());
            m_console_buffer.append(msg);
            m_console_scroll_to_bottom = true;
            spdlog::error("OSM import failed: {}", err);
            return;
        }

        // Hand the parsed data over on the main thread, then release the job.
        m_osm_parser = std::move(*m_import_job->parser);
        m_import_job.reset();

        m_osm_parser.log_statistics();
        m_osm_parser.log_sample_data();

        // ── Stage 2: road network ──
        // Solved once over the whole graph rather than per quadtree leaf, so
        // junctions, miters and profile transitions survive leaf boundaries. It
        // is pure stratum_core CPU work, so it goes on a worker like parsing did;
        // the quadtree is not touched until it lands.
        //
        // The lambda holds a pointer into m_osm_parser. That is safe because
        // nothing may reassign or clear the parser while this future is valid:
        // begin_osm_import() and the Clear Data button are both gated on the
        // import being idle.
        m_import_stage = ImportStage::BuildingRoads;
        m_import_message = "Building road network...";
        m_import_fraction = 0.0f;

        const osm::ParsedOSMData* parsed = &m_osm_parser.get_data();
        m_road_rebuild_only = false;
        m_road_build_future = std::async(std::launch::async,
                                         [parsed,
                                          cfg = make_road_network_config(),
                                          surface = live_road_terrain_fingerprint()]() {
            RoadBuildResult result = run_road_network_build(*parsed, cfg);
            result.terrain_fingerprint = surface;
            return result;
        });
        return;
    }

    if (m_import_stage == ImportStage::BuildingRoads) {
        if (!m_road_build_future.valid()) {
            // Cannot happen through the state machine above; recover rather than
            // wedge the panel in a stage that never advances.
            spdlog::error("Road network stage entered with no job; aborting import");
            m_import_stage = ImportStage::Failed;
            m_import_message = "Road network build lost";
            return;
        }

        if (m_road_build_future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            return;  // still solving; the indeterminate bar keeps animating
        }

        RoadBuildResult result = m_road_build_future.get();
        osm::road::RoadNetwork& network = result.network;

        m_road_stats = network.stats;
        m_road_elevation_stats = result.elevation;
        m_road_junction_stats = network.junction_stats;
        m_road_max_grade = result.max_grade;
        m_road_built_on_terrain = result.elevated;
        m_road_solved_junctions = result.solved_junctions;
        m_road_emitted_markings = result.emitted_markings;
        m_road_emitted_crossings = result.emitted_crossings;
        m_road_emitted_structures = result.emitted_structures;
        m_road_portal_mouths = network.carve_portals.size();
        m_have_road_stats = true;
        // The surface this build was SOLVED against, captured at launch. Reading
        // the manager here instead would record whatever terrain is loaded when
        // the future happens to land, which is a different surface entirely
        // whenever the user regenerated terrain mid-build.
        m_road_terrain_fingerprint = result.elevated ? result.terrain_fingerprint : 0;

        char road_msg[320];
        snprintf(road_msg, sizeof(road_msg),
                 "[OSM] Road network: %zu pieces, %zu triangles from %zu edges in %.0f ms\n",
                 network.stats.pieces, network.stats.triangles, network.stats.edges,
                 network.stats.build_ms);
        m_console_buffer.append(road_msg);

        if (result.elevated) {
            snprintf(road_msg, sizeof(road_msg),
                     "[OSM] Elevation: %zu edges solved in %.0f ms, %zu iterations, "
                     "max grade %.1f%%, %zu bridges, %zu tunnels\n",
                     network.stats.elevated_edges, network.stats.elevation_ms,
                     result.elevation.iterations, result.max_grade * 100.0f,
                     result.elevation.bridges, result.elevation.tunnels);
        } else {
            snprintf(road_msg, sizeof(road_msg),
                     "[OSM] Elevation: skipped, roads are flat (no terrain to follow)\n");
        }
        m_console_buffer.append(road_msg);

        if (result.solved_junctions) {
            snprintf(road_msg, sizeof(road_msg),
                     "[OSM] Junctions: %zu solved, %zu roundabouts, %zu tapers, "
                     "%zu dead ends, %zu degenerate, %zu over-trimmed edges in %.0f ms\n",
                     network.junction_stats.junctions, network.junction_stats.roundabouts,
                     network.junction_stats.tapers, network.junction_stats.dead_ends,
                     network.junction_stats.degenerate,
                     network.junction_stats.over_trimmed_edges,
                     network.stats.junction_ms);
        } else {
            snprintf(road_msg, sizeof(road_msg),
                     "[OSM] Junctions: solver off, ribbons run through every node\n");
        }
        m_console_buffer.append(road_msg);

        snprintf(road_msg, sizeof(road_msg),
                 "[OSM] Detail: %zu marking pieces, %zu crossings, %zu bridges, "
                 "%zu tunnels (%zu portal mouths), %zu sidewalk sides deduped\n",
                 network.stats.markings_pieces, network.stats.crossings,
                 network.stats.bridges, network.stats.tunnels,
                 network.carve_portals.size(), network.stats.deduped_sidewalks);
        m_console_buffer.append(road_msg);
        m_console_scroll_to_bottom = true;

        // The corridors outlive the network: they are indexed and carved once the
        // meshes are done. Moved out before the pieces are handed to the quadtree,
        // because that call consumes the network.
        m_pending_carve = std::make_unique<procgen::CarveInput>();
        m_pending_carve->ribbons = std::move(network.carve_ribbons);
        m_pending_carve->discs   = std::move(network.carve_discs);
        m_pending_carve->portals = std::move(network.carve_portals);
        m_pending_carve->config  = m_carve_config;

        // ── Stage 3: spatial index ──
        // Blocking, but far cheaper than parsing, and it mutates the quadtree that
        // render_3d walks every frame, so it has to stay on the main thread.
        m_import_stage = ImportStage::Indexing;
        m_import_message = "Building spatial index...";
        m_import_fraction = 0.0f;

        begin_mesh_rebuild(std::move(network.pieces), !m_road_rebuild_only);

        m_import_stage = ImportStage::BuildingMeshes;
        m_import_message = "Building meshes...";
        return;
    }

    // ── Stage 5: carve the solved corridors into the terrain ──
    if (m_import_stage == ImportStage::CarvingTerrain) {
        poll_road_carve();
        return;
    }

    if (m_import_stage != ImportStage::BuildingMeshes) {
        return;
    }

    // ── Stage 3: drain the async node builds ──
    // Also polled by render_3d, but do it here too so progress advances even when
    // the Viewport panel is closed.
    m_quadtree.poll_async_builds();

    size_t done = 0;
    for (const auto* node : m_import_pending_nodes) {
        if (node->meshes_built) ++done;
    }

    m_import_fraction = m_import_nodes_total > 0
        ? static_cast<float>(done) / static_cast<float>(m_import_nodes_total)
        : 1.0f;

    if (done < m_import_nodes_total) {
        return;
    }

    // ── Meshes done: hand the corridors to the terrain ──
    m_import_pending_nodes.clear();

    const auto& data = m_osm_parser.get_data();
    char msg[256];
    snprintf(msg, sizeof(msg), "[OSM] Loaded: %zu roads, %zu buildings, %zu areas\n",
             data.roads.size(), data.buildings.size(), data.areas.size());
    m_console_buffer.append(msg);
    m_console_scroll_to_bottom = true;

    m_import_stage = ImportStage::CarvingTerrain;
    m_import_fraction = 0.0f;
    m_import_message = "Preparing terrain carve...";
    begin_road_carve();
}

// ============================================================================
// Terrain carve stage
// ============================================================================

void Editor::begin_road_carve() {
    m_carve_apply_pending = false;

    const bool have_corridors = m_pending_carve && m_pending_carve->item_count() > 0;

    if (!have_corridors) {
        // Roads were built flat, so there is nothing to carve. Any carve data
        // still installed belongs to a network that no longer exists and would
        // hold trenches under roads that have moved, so drop it. Dropping it
        // regenerates the affected chunks, which is the same blocking work an
        // install is, so it goes through the same deferred-by-one-frame path.
        m_pending_carve.reset();
        if (m_terrain_tile_manager.has_road_carve_data()) {
            m_import_message = "Clearing terrain carve...";
            m_carve_apply_pending = true;
        } else {
            finish_osm_import();
        }
        return;
    }

    // Indexing is proportional to the number of corridors, which is proportional
    // to the size of the import, so it goes on a worker like every other
    // whole-network pass. It touches nothing the main thread reads.
    m_import_message = "Indexing road corridors...";
    m_carve_index_future = std::async(std::launch::async,
                                      [input = std::move(m_pending_carve)]() mutable {
        input->build_index();
        return std::move(input);
    });
}

void Editor::poll_road_carve() {
    // Second half of the deferred apply: the stage message set last frame has
    // been drawn, so the blocking regeneration may now run.
    if (m_carve_apply_pending) {
        m_carve_apply_pending = false;
        install_road_carve_data();
        finish_osm_import();
        return;
    }

    if (!m_carve_index_future.valid()) {
        // No index job and no pending apply: nothing left to do in this stage.
        finish_osm_import();
        return;
    }

    if (m_carve_index_future.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return;  // still indexing; the indeterminate bar keeps animating
    }

    m_pending_carve = m_carve_index_future.get();

    // Apply on the NEXT frame. set_road_carve_data() regenerates every existing
    // chunk in one call; doing that in the frame that names the stage means the
    // name never reaches the screen.
    m_import_message = "Carving terrain...";
    m_carve_apply_pending = true;
}

void Editor::install_road_carve_data() {
    if (!m_pending_carve) {
        if (m_terrain_tile_manager.has_road_carve_data()) {
            m_terrain_tile_manager.clear_road_carve_data();
            m_console_buffer.append("[Terrain] Road carve cleared; terrain returned to procedural\n");
            m_console_scroll_to_bottom = true;
        }
        return;
    }

    char msg[256];
    snprintf(msg, sizeof(msg), "[Terrain] Carving %zu corridors and %zu junctions into %zu chunks\n",
             m_pending_carve->ribbons.size(), m_pending_carve->discs.size(),
             m_terrain_tile_manager.chunk_count());
    m_console_buffer.append(msg);
    m_console_scroll_to_bottom = true;

    // Regenerates every already-generated chunk, which is why this is on the main
    // thread: render_3d() walks those chunks every frame. Chunks generated LATER
    // pick the carve up inside generate_chunk(), so an install with no terrain yet
    // is nearly free and still correct.
    m_terrain_tile_manager.set_road_carve_data(std::move(*m_pending_carve));
    m_pending_carve.reset();
}

void Editor::finish_osm_import() {
    m_pending_carve.reset();
    m_carve_apply_pending = false;
    m_import_stage = ImportStage::Done;
    m_import_fraction = 1.0f;
    m_import_message = m_road_rebuild_only ? "Roads re-solved against the terrain"
                                           : "Import successful";
    m_road_rebuild_only = false;
    spdlog::info("OSM import complete");

    // A rebuild asked for while this one was in flight. Honoured through
    // maybe_rebuild_roads_for_terrain() rather than launched directly, so a
    // request that the just-landed build already satisfied costs nothing.
    if (m_road_rebuild_owed) {
        m_road_rebuild_owed = false;
        maybe_rebuild_roads_for_terrain();
    }
}

void Editor::begin_mesh_rebuild(std::vector<osm::road::RoadPiece>&& road_pieces,
                                bool recenter_camera) {
    m_building_meshes.clear();
    m_road_meshes.clear();
    m_area_meshes.clear();

    const auto& osm_data = m_osm_parser.get_data();

    // Initialize quadtree
    spdlog::info("Initializing quadtree...");
    // Every leaf about to be destroyed may still own GPU buffers. clear() only
    // drops the CPU-side tree, so without this the previous tree's buffers stay
    // alive with nothing referencing them for the rest of the session. A road
    // rebuild makes this path routine rather than once-per-import.
    if (m_gpu_renderer) {
        for (auto* leaf : m_quadtree.get_all_leaves()) {
            if (leaf) release_node_from_gpu(*leaf, *m_gpu_renderer);
        }
    }
    m_quadtree.clear();
    // Sized from the features, not osm_data.bounds -- see QuadTree::init(ParsedOSMData).
    m_quadtree.init(osm_data);
    m_quadtree.assign_data(osm_data);

    // Roads are not rebuilt per leaf any more. Hand the already-solved geometry
    // over now, while the leaves exist and before any node build is queued, so
    // the first upload of a leaf already carries its roads.
    //
    // Set BEFORE the hand-off: the flag decides how pieces are routed into the
    // leaves as well as whether a chain is built afterwards, and both happen
    // inside assign_road_pieces().
    m_quadtree.set_chunk_lod(m_chunk_lod, osm::road::ChunkLodConfig{});
    m_quadtree.assign_road_pieces(std::move(road_pieces));

    spdlog::info("QuadTree: {} leaves, {} roads, {} buildings, {} areas, max depth {}",
                 m_quadtree.leaf_count(),
                 m_quadtree.total_roads(),
                 m_quadtree.total_buildings(),
                 m_quadtree.total_areas(),
                 m_quadtree.max_depth());

    // Find geometry center for camera positioning
    glm::vec3 bounds_min, bounds_max;
    m_quadtree.get_bounds(bounds_min, bounds_max);
    bool found_geometry = (bounds_min.x < bounds_max.x || bounds_min.z < bounds_max.z);
    glm::vec3 data_center = (bounds_min + bounds_max) * 0.5f;

    if (!found_geometry) {
        spdlog::warn("No geometry found in quadtree!");
    }

    spdlog::info("Data center: ({}, {}, {})", data_center.x, data_center.y, data_center.z);

    // Center camera on data FIRST (before culling uses camera position)
    glm::vec3 focus_centre;
    float focus_radius = 0.0f;
    const bool have_focus = m_quadtree.get_focus(focus_centre, focus_radius);

    if (have_focus && recenter_camera) {
        // Frame where the features actually are, not the centre of their bounding
        // box. Those differ wildly for an Overpass export, whose box is stretched
        // by the nodes it pulls in for ways crossing the query area.
        data_center = focus_centre;

        // Scale to the data. The old fixed 300m/5000m numbers meant a large import
        // put the camera thousands of metres from anything, so distance culling
        // rejected every node, nothing was ever queued, and the viewport stayed
        // empty with no error shown.
        const float view_distance = std::clamp(focus_radius, 300.0f, 8000.0f);
        glm::vec3 cam_pos = data_center + glm::vec3(0.0f, view_distance * 0.8f, view_distance);

        m_camera.set_position(cam_pos);
        m_camera.set_target(data_center);
        // Depth precision is governed by far/near, so keep that ratio sane rather
        // than pairing a 0.1m near plane with a far plane tens of km out -- that
        // combination puts almost the whole depth buffer in the first few metres
        // and leaves coplanar roads, landuse and building footprints z-fighting.
        // Only draw as far as nodes are actually built, plus headroom.
        m_camera.m_far = std::clamp(view_distance * 6.0f, 20000.0f, 80000.0f);
        m_camera.m_near = std::clamp(m_camera.m_far / 20000.0f, 0.1f, 5.0f);
        m_camera.m_base_speed = std::clamp(focus_radius * 0.1f, 200.0f, 5000.0f);
        // Must reach past the camera's own distance from the data, or distance
        // culling rejects everything before it can be built. Bounded so a huge
        // import does not try to mesh the whole dataset at once -- the remainder
        // streams in as the camera moves.
        m_view_radius = std::clamp(view_distance * 3.0f, 5000.0f, 30000.0f);

        spdlog::info("Camera at ({:.0f}, {:.0f}, {:.0f}) looking at ({:.0f}, {:.0f}, {:.0f}), "
                     "focus radius {:.0f}m, view radius {:.0f}m",
                     cam_pos.x, cam_pos.y, cam_pos.z,
                     data_center.x, data_center.y, data_center.z,
                     focus_radius, m_view_radius);
    } else if (have_focus) {
        // A road rebuild after terrain generation. The geometry is the same data
        // in the same place, so re-framing it would only throw away wherever the
        // user was looking.
        data_center = focus_centre;
        spdlog::info("Road rebuild: leaving the camera where it is");
    } else if (found_geometry) {
        spdlog::warn("No populated quadtree leaves; leaving the camera where it is");
    }

    // Force camera matrix recalculation so frustum matches new position
    // (update() is normally called in draw_viewport, but we need it now for traversal)
    m_camera.update(1.0f); // aspect doesn't matter much, just need valid frustum

    // Enable culling for performance
    m_use_tile_culling = true;
    m_use_distance_culling = true;
    m_use_contribution_culling = false; // disable initially — camera is far, nodes appear small

    // Queue the initially-visible leaves. These builds are already asynchronous;
    // poll_osm_import() drains them across frames and reports progress. Blocking
    // here on a spin-wait used to freeze the UI for the whole build.
    m_import_pending_nodes.clear();

    Frustum frustum = m_camera.get_frustum();
    glm::vec3 cam_pos = m_camera.get_position();

    m_quadtree.traverse_visible(
        frustum.planes, cam_pos, m_view_radius,
        600.0f, m_camera.m_fov, m_contribution_threshold,
        true, true, false, // frustum + distance, no contribution cull
        [&](osm::QuadTreeNode* node, float /*dist_sq*/) {
            if (!node->meshes_built && !node->meshes_pending) {
                if (m_quadtree.queue_node_build_async(node)) {
                    m_import_pending_nodes.push_back(node);
                }
            }
        }
    );

    m_import_nodes_total = m_import_pending_nodes.size();
    spdlog::info("Queued {} initial node builds", m_import_nodes_total);
}



// ============================================================================
// Resident GPU geometry: ownership, distance, and eviction
// ============================================================================

glm::vec3 Editor::node_anchor(const osm::QuadTreeNode& node) {
    if (node.has_valid_bounds()) {
        return (node.bounds_min + node.bounds_max) * 0.5f;
    }
    // No geometry ever grew the AABB, but the cell still locates the leaf. Local
    // 2D (x, y) maps to world (x, height, -y); the height is unknown, so 0.
    return glm::vec3(static_cast<float>(node.center.x), 0.0f,
                     static_cast<float>(-node.center.y));
}

uint32_t Editor::upload_tracked_mesh(GPURenderer& renderer, const Mesh& mesh,
                                     const MeshOwner& owner) {
    const uint32_t id = renderer.upload_mesh(mesh);
    if (id == 0) {
        // Not a handle. Registering it would make the owner of mesh 0 whichever
        // upload failed most recently.
        return 0;
    }
    m_mesh_owners[id] = owner;
    return id;
}

void Editor::release_tracked_mesh(GPURenderer& renderer, uint32_t& mesh_id) {
    if (mesh_id == 0) return;
    m_mesh_owners.erase(mesh_id);
    renderer.release_mesh(mesh_id);
    mesh_id = 0;
}

float Editor::mesh_distance_to_camera(uint32_t mesh_id) const {
    const auto it = m_mesh_owners.find(mesh_id);
    if (it == m_mesh_owners.end()) {
        // Nothing is holding this handle, so nothing will miss it. Reporting it as
        // infinitely far away puts it at the front of the eviction order, which is
        // the right answer for geometry nobody is tracking any more.
        return std::numeric_limits<float>::max();
    }
    if (it->second.kind == MeshOwner::Kind::Pinned) {
        return -1.0f;
    }
    return glm::length(it->second.anchor - m_camera.get_position());
}

void Editor::on_mesh_evicted(uint32_t mesh_id) {
    const auto it = m_mesh_owners.find(mesh_id);
    if (it == m_mesh_owners.end()) return;

    const MeshOwner owner = it->second;
    m_mesh_owners.erase(it);

    // No call back into the renderer from here: it is mid-eviction and walking
    // its own mesh map. Clearing the handle is all this has to do.
    switch (owner.kind) {
        case MeshOwner::Kind::QuadTreeLeaf: {
            if (!owner.node) break;
            std::erase(owner.node->area_gpu_ids, mesh_id);
            if (std::erase(owner.node->road_gpu_ids, mesh_id) > 0) {
                // The resident LOD level went with it. Saying so is what makes
                // sync_node_road_lod() upload again instead of trusting a level
                // that is no longer on the device.
                owner.node->road_lod_resident = -1;
            }
            std::erase(owner.node->building_gpu_ids, mesh_id);
            // The leaf is no longer whole, so it is no longer uploaded. It streams
            // back in the next time it is visible, and upload_node_to_gpu()
            // releases whichever of its handles survived before re-uploading.
            owner.node->gpu_uploaded = false;
            break;
        }
        case MeshOwner::Kind::TerrainChunk: {
            auto* chunk = m_terrain_tile_manager.get_chunk(owner.coord);
            if (!chunk) break;
            if (chunk->terrain_gpu_id == mesh_id) chunk->terrain_gpu_id = 0;
            if (chunk->water_gpu_id == mesh_id) chunk->water_gpu_id = 0;
            // Same contract as a leaf: the re-upload path in render_3d() releases
            // the surviving handle before replacing both.
            chunk->gpu_uploaded = false;
            break;
        }
        case MeshOwner::Kind::Pinned:
            // Unreachable: a pinned mesh reports a negative distance and is never
            // a candidate. If it happens, the handle has already been forgotten
            // above, which is the most that can be done from here.
            spdlog::warn("A pinned mesh ({}) was evicted", mesh_id);
            break;
    }
}

void Editor::upload_node_to_gpu(osm::QuadTreeNode& node, GPURenderer& renderer) {
    if (node.gpu_uploaded) return;

    // A leaf can reach here still holding handles: eviction takes one of its
    // meshes and clears gpu_uploaded, leaving the others live. Clearing the id
    // vectors without releasing them -- which is what this used to do -- would
    // strand that geometry on the GPU for the rest of the session.
    release_node_from_gpu(node, renderer);

    MeshOwner owner;
    owner.kind = MeshOwner::Kind::QuadTreeLeaf;
    owner.node = &node;
    owner.anchor = node_anchor(node);

    size_t failed = 0;
    size_t uploaded = 0;
    const auto upload_all = [&](const std::vector<Mesh>& meshes, std::vector<uint32_t>& ids) {
        ids.reserve(meshes.size());
        for (const Mesh& mesh : meshes) {
            if (!mesh.is_valid()) {
                continue;  // nothing to draw, and not a failure either
            }
            const uint32_t id = upload_tracked_mesh(renderer, mesh, owner);
            if (id == 0) {
                ++failed;
                continue;
            }
            ids.push_back(id);
            ++uploaded;
        }
    };

    upload_all(node.area_meshes, node.area_gpu_ids);
    // Empty when the leaf carries a chunk LOD chain: the chain replaced this mesh
    // and sync_node_road_lod() uploads exactly one level of it, per frame, by
    // distance. Roads are therefore NOT part of the completeness test below --
    // they are not uploaded here and their absence is not a failure.
    upload_all(node.road_meshes, node.road_gpu_ids);
    upload_all(node.building_meshes, node.building_gpu_ids);

    // Only a leaf that uploaded IN FULL is uploaded.
    //
    // This used to push the 0 that upload_mesh() returns on FAILURE straight into
    // the id vector and set gpu_uploaded = true regardless, so a leaf that lost an
    // upload -- to a budget refusal, a pool refusal, or a device OOM -- was never
    // retried. draw_mesh() then discarded the 0 silently every frame and the leaf
    // rendered nothing for the rest of the session, with no error anywhere.
    //
    // The second half of the test covers eviction landing DURING this upload: an
    // upload under pressure evicts to make room, and the victim it picks can be a
    // mesh this very leaf uploaded a moment ago, which on_mesh_evicted() then
    // erases from the vectors below. Counting what is still held against what was
    // uploaded catches that without any extra state, and the leaf is retried
    // whole rather than latching as complete while missing a mesh.
    const size_t held = node.area_gpu_ids.size() + node.road_gpu_ids.size()
                      + node.building_gpu_ids.size();
    node.gpu_uploaded = (failed == 0) && (held == uploaded);

    if (failed > 0) {
        // Retried on every frame the leaf stays visible, so the warning is rate
        // limited rather than the retry: a failure that persists must not turn
        // into a 60 Hz log write.
        const uint64_t now = SDL_GetTicks();
        if (now >= m_next_upload_warn_ms) {
            m_next_upload_warn_ms = now + 2000;
            spdlog::warn("{} mesh(es) of quadtree leaf {} failed to upload; retrying while it "
                         "stays visible ({} renderer upload failures so far, {} MB resident)",
                         failed, node.node_id, renderer.upload_failures(),
                         renderer.resident_bytes() / (1024 * 1024));
        }
    }
}

void Editor::release_node_from_gpu(osm::QuadTreeNode& node, GPURenderer& renderer) {
    // Deliberately NOT guarded on gpu_uploaded. A leaf that lost one mesh to
    // eviction has the flag cleared while still holding the others, and a guard
    // here would leave exactly those behind -- which is the leak this function is
    // called to prevent, on every path that destroys the tree.
    const auto release_all = [&](std::vector<uint32_t>& ids) {
        for (uint32_t& id : ids) {
            release_tracked_mesh(renderer, id);
        }
        ids.clear();
    };

    release_all(node.area_gpu_ids);
    release_all(node.road_gpu_ids);
    release_all(node.building_gpu_ids);

    // The chain is still on the CPU, but nothing of it is on the device any
    // more. Leaving the level set would make sync_node_road_lod() believe the
    // right geometry was already resident and skip the re-upload.
    node.road_lod_resident = -1;
    node.gpu_uploaded = false;
}

void Editor::sync_node_road_lod(osm::QuadTreeNode& node, GPURenderer& renderer,
                                float distance) {
    if (!node.has_road_lod()) return;

    const int levels = static_cast<int>(node.road_lod.levels.size());

    // A forced level is clamped per chunk. Chains are not all the same length --
    // a chunk of seven pieces gives up after one level where a dense one gets
    // four -- so an override of 3 has to mean "the coarsest you have" rather than
    // "draw nothing".
    const int desired = (m_road_lod_override >= 0)
                      ? std::min(m_road_lod_override, levels - 1)
                      : osm::select_road_lod_level(node.road_lod, distance,
                                                   node.road_lod_resident,
                                                   m_road_lod_distance_scale);

    if (desired == node.road_lod_resident && !node.road_gpu_ids.empty()) {
        return;
    }

    ++m_road_lod_frame_build.swaps;

    // Release first, upload second. The other order would hold two levels of the
    // same chunk resident at once, and under a tight budget that is what makes an
    // upload evict some other leaf to make room for geometry about to be freed.
    for (uint32_t& id : node.road_gpu_ids) {
        release_tracked_mesh(renderer, id);
    }
    node.road_gpu_ids.clear();
    node.road_lod_resident = -1;

    const Mesh& mesh = node.road_lod.levels[static_cast<size_t>(desired)];
    if (!mesh.is_valid()) {
        // A level that simplified down to nothing is not a failure and must not
        // latch: leaving the level unset means the next frame tries again, which
        // is wrong. Record it as resident with no handle instead.
        node.road_lod_resident = desired;
        return;
    }

    MeshOwner owner;
    owner.kind = MeshOwner::Kind::QuadTreeLeaf;
    owner.node = &node;
    owner.anchor = node_anchor(node);

    const uint32_t id = upload_tracked_mesh(renderer, mesh, owner);
    if (id == 0) {
        return;  // retried on the next frame the leaf stays visible
    }

    // The upload may have evicted to make room, and the victim it picked can be
    // the mesh it just uploaded. m_mesh_owners is the record of what survived, so
    // pushing a handle that is no longer in it would leave the leaf drawing a
    // freed buffer.
    if (m_mesh_owners.find(id) == m_mesh_owners.end()) {
        return;
    }

    node.road_gpu_ids.push_back(id);
    node.road_lod_resident = desired;
}

void Editor::record_road_lod_residency(const osm::QuadTreeNode& node) {
    if (!node.has_road_lod()) {
        // A leaf with no chain but with road geometry is the chunk-LOD-off path,
        // not an empty leaf, and the panel has to be able to tell the two apart.
        if (!node.road_meshes.empty()) {
            ++m_road_lod_frame_build.leaves_no_chain;
        }
        return;
    }

    ++m_road_lod_frame_build.leaves_with_chain;

    const int level = node.road_lod_resident;
    if (level < 0 || level >= static_cast<int>(node.road_lod.levels.size())) {
        return;  // nothing resident: the upload was refused or the leaf was evicted
    }

    auto& per_level = m_road_lod_frame_build.leaves_per_level;
    const size_t idx = static_cast<size_t>(level);
    if (per_level.size() <= idx) per_level.resize(idx + 1, 0);
    ++per_level[idx];

    const Mesh& mesh = node.road_lod.levels[idx];
    m_road_lod_frame_build.resident_triangles += mesh.indices.size() / 3;
    m_road_lod_frame_build.resident_vertices += mesh.vertices.size();
}

void Editor::render_3d(GPURenderer& renderer) {
    // m_viewport_rect is only written while the Viewport panel is drawing. This
    // function is now called unconditionally from Application::render(), so bail
    // out if the panel is closed/collapsed - a zero-size viewport or scissor is a
    // Vulkan validation error.
    if (m_viewport_rect.z < 1.0f || m_viewport_rect.w < 1.0f) {
        return;
    }

    // m_viewport_rect comes from ImGui, which lays out in LOGICAL POINTS.
    // SDL_GPUViewport and SDL_SetGPUScissor address the swapchain, which is sized
    // in PIXELS (gpu_renderer.cpp, SDL_GetWindowSizeInPixels). The two coincide
    // only where the pixel density is 1: on a Retina Mac it is 2, and passing
    // points through unconverted scissors the 3D pass to a rect half as wide and
    // half as tall as the panel, anchored at half its offset. Most of that rect
    // then falls under the left dock panel and the tab bar, which the ImGui pass
    // paints over afterwards -- so the scene appears squeezed into the top-left
    // corner of the viewport. Linux never showed it because density is 1 there.
    //
    // DisplayFramebufferScale is the same factor the ImGui backend uses to map
    // its draw data onto the swapchain (imgui_impl_sdlgpu3.cpp: fb_width =
    // DisplaySize * FramebufferScale), so scaling by it keeps the 3D pass aligned
    // with the panel ImGui draws around it, whatever the backend reports.
    const ImGuiIO& io = ImGui::GetIO();
    const float scale_x = io.DisplayFramebufferScale.x > 0.0f ? io.DisplayFramebufferScale.x : 1.0f;
    const float scale_y = io.DisplayFramebufferScale.y > 0.0f ? io.DisplayFramebufferScale.y : 1.0f;

    const float px_x = m_viewport_rect.x * scale_x;
    const float px_y = m_viewport_rect.y * scale_y;
    const float px_w = m_viewport_rect.z * scale_x;
    const float px_h = m_viewport_rect.w * scale_y;

    // Set Viewport
    SDL_GPUViewport viewport;
    viewport.x = px_x;
    viewport.y = px_y;
    viewport.w = px_w;
    viewport.h = px_h;
    viewport.min_depth = 0.0f;
    viewport.max_depth = 1.0f;
    renderer.set_viewport(viewport);

    // The scissor must stay inside the render target -- a rect that pokes past
    // the edge is a Vulkan validation error, and the panel can hang off the
    // window edge mid-resize drag. The viewport above needs no such clamp: it
    // only defines the clip-space mapping, and the scissor is what clips.
    const int fb_w = static_cast<int>(renderer.get_swapchain_width());
    const int fb_h = static_cast<int>(renderer.get_swapchain_height());
    const int x0 = std::clamp(static_cast<int>(px_x), 0, fb_w);
    const int y0 = std::clamp(static_cast<int>(px_y), 0, fb_h);
    const int x1 = std::clamp(static_cast<int>(px_x + px_w), x0, fb_w);
    const int y1 = std::clamp(static_cast<int>(px_y + px_h), y0, fb_h);
    if (x1 <= x0 || y1 <= y0) {
        return;  // scrolled fully off-screen: nothing to draw, and a zero-area
                 // scissor is a validation error
    }

    SDL_Rect scissor;
    scissor.x = x0;
    scissor.y = y0;
    scissor.w = x1 - x0;
    scissor.h = y1 - y0;
    if (renderer.get_render_pass()) {
        SDL_SetGPUScissor(renderer.get_render_pass(), &scissor);
    }

    // Camera state FIRST. bind_mesh_pipeline() pushes SceneUniforms -- which
    // carries camera_position, and the PBR shader derives the view vector, the
    // double-sided normal flip and the fog distance from it -- so setting the
    // camera after the bind published last frame's position to this frame's
    // draws.
    //
    // Application::render() has already called this before the shadow cascades,
    // which need the same matrices earlier still. Repeating it is two assignments
    // and keeps this function correct on its own.
    publish_camera(renderer);
    glm::vec3 cam_pos = m_camera.get_position();

    // Sky BEFORE geometry and after the camera state above: it reconstructs a
    // world ray per pixel from the inverse view-projection, and it neither tests
    // nor writes depth, so everything drawn afterwards simply covers it.
    renderer.draw_sky();

    renderer.bind_mesh_pipeline();

    Frustum frustum = m_camera.get_frustum();
    glm::mat4 model(1.0f);

    // Gathered by the visitor below and published when the traversal returns, so
    // the panel never reads a partially counted frame.
    m_road_lod_frame_build.reset();

    // Use quadtree traversal for GPU rendering (front-to-back sorted)
    m_quadtree.traverse_visible(
        frustum.planes,
        cam_pos,
        m_view_radius,
        viewport.h,
        m_camera.m_fov,
        m_contribution_threshold,
        m_use_tile_culling,
        m_use_distance_culling,
        m_use_contribution_culling,
        [&](osm::QuadTreeNode* node, float dist_sq) {
            // Stream: a node that just became visible gets its mesh build queued
            // here. This traversal is the only one per frame, so it has to do the
            // queueing that rebuild_visible_batches() used to.
            if (!node->meshes_built) {
                if (!node->meshes_pending) {
                    m_quadtree.queue_node_build_async(node);
                }
                return;
            }

            // Upload to GPU if needed
            if (!node->gpu_uploaded) {
                upload_node_to_gpu(*node, renderer);
            }

            // Roads go through their own path when the leaf carries a chunk LOD
            // chain, because which level belongs on the device depends on where
            // the camera is and the rest of the leaf does not. dist_sq is the
            // squared XZ distance the traversal already computed for its
            // front-to-back sort, which is the same measure ChunkLod's switch
            // distances are expressed in.
            sync_node_road_lod(*node, renderer, std::sqrt(dist_sq));
            record_road_lod_residency(*node);

            // The trailing MaterialKey is the DEFAULT for geometry that carries no
            // material tag of its own -- these builders predate MaterialId and emit
            // no submeshes at all. Road meshes from the new road network DO carry
            // tagged ranges, and those always win over the default; passing Asphalt
            // here only affects the legacy flat road ribbons.
            //
            // The tint is white in every mode but Tile, so this is a no-op
            // multiply unless the user asked for it. Tile is the only attribute
            // whose value is constant across a whole leaf, which is the only shape
            // a per-draw tint can express -- the classification modes cannot use
            // this path at all, because every building in the leaf shares one
            // merged mesh and one draw call. They go through draw_attribute_overlay().
            const glm::vec4 tint = attribute_leaf_tint(*node);

            if (m_render_areas) {
                for (uint32_t id : node->area_gpu_ids)
                    renderer.draw_mesh(id, model, tint, MaterialKey{MaterialId::Grass, 0});
            }
            if (m_render_roads) {
                for (uint32_t id : node->road_gpu_ids)
                    renderer.draw_mesh(id, model, tint, MaterialKey{MaterialId::Asphalt, 0});
            }
            if (m_render_buildings) {
                // Buildings are not a road surface. Concrete is the closest of the
                // eleven slots and is at least not the untagged grey.
                for (uint32_t id : node->building_gpu_ids)
                    renderer.draw_mesh(id, model, tint, MaterialKey{MaterialId::Concrete, 0});
            }
        }
    );

    m_road_lod_frame = m_road_lod_frame_build;

    // Render procedural terrain
    float radius_sq = m_view_radius * m_view_radius;
    if (m_use_chunked_terrain) {
        // Render chunked terrain
        if (m_render_terrain) {
            for (const auto& coord : m_terrain_tile_manager.get_all_chunks()) {
                auto* chunk = const_cast<procgen::TerrainChunk*>(m_terrain_tile_manager.get_chunk(coord));
                if (!chunk || !chunk->mesh_built) continue;

                // Cull BEFORE uploading. The upload used to run for every chunk in
                // the manager whatever the camera could see, which was merely
                // wasteful when nothing was ever unloaded; with an eviction budget
                // it is a thrash loop, because a chunk evicted for being far away
                // would re-upload on the very next frame.
                if (m_use_tile_culling && !frustum.intersects_aabb(chunk->bounds_min, chunk->bounds_max)) {
                    continue;
                }

                // Distance culling
                if (m_use_distance_culling) {
                    glm::vec3 chunk_center = (chunk->bounds_min + chunk->bounds_max) * 0.5f;
                    float dist_sq = glm::dot(chunk_center - cam_pos, chunk_center - cam_pos);
                    if (dist_sq > radius_sq) continue;
                }

                // Upload to GPU if needed
                if (!chunk->gpu_uploaded && m_gpu_renderer) {
                    // A cleared gpu_uploaded on a chunk that still carries handles
                    // means either its mesh was REBUILT -- by a road carve install,
                    // or by a terrain settings change -- or one of its two meshes
                    // was evicted. Overwriting the handles without releasing them
                    // first strands the old ranges for the rest of the session, and
                    // a carve regenerates every chunk at once.
                    release_tracked_mesh(*m_gpu_renderer, chunk->terrain_gpu_id);
                    release_tracked_mesh(*m_gpu_renderer, chunk->water_gpu_id);

                    MeshOwner owner;
                    owner.kind = MeshOwner::Kind::TerrainChunk;
                    owner.coord = coord;
                    owner.anchor = (chunk->bounds_min + chunk->bounds_max) * 0.5f;

                    if (chunk->terrain_mesh.is_valid()) {
                        chunk->terrain_gpu_id =
                            upload_tracked_mesh(*m_gpu_renderer, chunk->terrain_mesh, owner);
                    }
                    if (chunk->water_mesh.is_valid()) {
                        chunk->water_gpu_id =
                            upload_tracked_mesh(*m_gpu_renderer, chunk->water_mesh, owner);
                    }

                    // Same rule as a quadtree leaf: a partial upload is not an
                    // upload and must be retried rather than latched. Read back
                    // from the handles AFTER both uploads rather than from each
                    // return value, because the second upload can evict the first
                    // one to make room -- on_mesh_evicted() then zeroes a handle
                    // that was good when it was returned.
                    chunk->gpu_uploaded =
                        (!chunk->terrain_mesh.is_valid() || chunk->terrain_gpu_id != 0) &&
                        (!chunk->water_mesh.is_valid() || chunk->water_gpu_id != 0);
                }
                
                // Terrain chunks are a second tile system, with their own grid and
                // their own streaming bugs, so the Tile mode covers them too. White
                // in every other mode.
                const glm::vec4 chunk_tint = attribute_chunk_tint(coord);

                // Draw terrain
                if (chunk->terrain_gpu_id != 0) {
                    renderer.draw_mesh(chunk->terrain_gpu_id, model, chunk_tint,
                                       MaterialKey{MaterialId::Grass, 0});
                }

                // Draw water. No water slot exists in MaterialId -- it is a road
                // material set -- so water keeps the untagged default rather than
                // borrowing a surface that would make it look like wet tarmac.
                if (m_render_water && chunk->water_gpu_id != 0) {
                    renderer.draw_mesh(chunk->water_gpu_id, model, chunk_tint);
                }
            }
        }
    } else {
        // Legacy single terrain rendering
        if (m_render_terrain && m_terrain_gpu_id != 0) {
            renderer.draw_mesh(m_terrain_gpu_id, model, glm::vec4(1.0f),
                               MaterialKey{MaterialId::Grass, 0});
        }
        if (m_render_water && m_water_gpu_id != 0) {
            renderer.draw_mesh(m_water_gpu_id, model, glm::vec4(1.0f));
        }
    }

    // The rule editor's preview, drawn with the rest of the opaque scene so it
    // lights and shadows like a building rather than floating as debug geometry.
    // Concrete because a rule that sets no material should still look like a
    // massing model, not like untextured default grey.
    if (m_render_rule_preview && m_rule_preview_gpu_id != 0) {
        renderer.draw_mesh(m_rule_preview_gpu_id, model, glm::vec4(1.0f),
                           MaterialKey{MaterialId::Concrete, 0});
    }

    // Im3d debug geometry last, so it depth-tests against the opaque scene above.
    // Inherits the viewport and scissor already set at the top of this function.
    Im3D_Render(renderer, m_camera.get_view_projection(), viewport.w, viewport.h);
}

} // namespace stratum
