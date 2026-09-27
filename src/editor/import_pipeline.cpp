// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file import_pipeline.cpp
 * @brief Wires Editor to app::ImportPipeline (stratum_core)
 *
 * The OSM import / road build / spatial index / terrain carve / export state
 * machine that used to live entirely in this file moved to
 * src/app/import_pipeline.cpp in phase 3 of the UI restructure (see
 * docs/plans/import-pipeline-design.md), so the same code runs in the editor,
 * in stratum_tests and in a future headless CLI. What is left here is the
 * editor-side glue the design's section 2 describes: the Listener adapter that
 * ties pipeline callbacks back to GPU residency (SceneGpuSync), camera framing
 * and the console, plus the thin per-frame wrappers (road_options(),
 * request_road_rebuild(), clear_imported_data()) that keep "the model changed
 * this frame, the request sees it" true.
 */

#include "editor/editor.hpp"
#include <spdlog/spdlog.h>

namespace stratum {

// ============================================================================
// PipelineListener: app::ImportPipeline::Listener -> Editor
// ============================================================================

void Editor::PipelineListener::on_before_quadtree_reset() {
    // Every leaf about to be destroyed may still own GPU meshes, and
    // m_mesh_owners holds a raw pointer to each of them. Releasing them here,
    // before the pipeline tears the tree down, is what used to happen inline
    // in Editor::clear_imported_data() (ip.cpp:711-715) and
    // Editor::begin_mesh_rebuild() (ip.cpp:756-760).
    if (editor_.m_gpu_renderer) {
        for (auto* leaf : editor_.m_quadtree.get_all_leaves()) {
            if (leaf) editor_.release_node_from_gpu(*leaf, *editor_.m_gpu_renderer);
        }
    }
}

void Editor::PipelineListener::on_quadtree_ready(bool recenter_camera) {
    // Was ip.cpp:783-788.
    editor_.frame_camera_on_data(recenter_camera);

    // Enable culling for performance.
    editor_.m_model.m_use_tile_culling = true;
    editor_.m_model.m_use_distance_culling = true;
    // Disabled initially: the camera is far, so nodes appear small.
    editor_.m_model.m_use_contribution_culling = false;
}

std::optional<app::InitialView> Editor::PipelineListener::initial_view() {
    // Was ip.cpp:795-800. Filled from the live camera so the pipeline queues
    // the same initially-visible leaves the old inline traversal did.
    app::InitialView view;
    const Frustum frustum = editor_.m_camera.get_frustum();
    view.frustum_planes = frustum.planes;
    view.camera_position = editor_.m_camera.get_position();
    view.view_radius = editor_.m_model.m_view_radius;
    view.fov_y = editor_.m_camera.m_fov;
    view.contribution_threshold = editor_.m_model.m_contribution_threshold;
    return view;
}

void Editor::PipelineListener::on_log(const app::PipelineLogLine& /*line*/) {
    // The pipeline already logged through spdlog itself; this only mirrors
    // the "scroll to bottom" side effect every ip.cpp logging site had.
    // Pushing the line into LogRing here as well would print it twice, since
    // m_log_sink already mirrors every spdlog record into the ring.
    editor_.m_console_scroll_to_bottom = true;
}

void Editor::PipelineListener::on_export_done(const app::ExportResult& result) {
    // Was poll_road_export(), ip.cpp:106-131.
    editor_.m_export_status = result.status;
}

// ============================================================================
// Editor-side wrappers
// ============================================================================

app::RoadOptions Editor::road_options() const {
    app::RoadOptions opts;
    opts.terrain_aware = m_model.m_terrain_aware_roads;
    opts.chunked_terrain = m_use_chunked_terrain;
    opts.solve_junctions = m_model.m_solve_junctions;
    opts.emit_markings = m_model.m_emit_markings;
    opts.emit_crossings = m_model.m_emit_crossings;
    opts.emit_structures = m_model.m_emit_structures;
    opts.reduce_tessellation = m_model.m_reduce_tessellation;
    opts.chunk_lod = m_model.m_chunk_lod;
    return opts;
}

app::RebuildOutcome Editor::request_road_rebuild(app::RebuildPolicy policy) {
    m_import_pipeline.set_road_options(road_options());
    return m_import_pipeline.request_road_rebuild(policy);
}

bool Editor::has_generated_terrain() const {
    return m_terrain_tile_manager.generated_count() > 0;
}

void Editor::clear_imported_data() {
    // cancel() is a no-op (returns false) in every state the "Clear Data"
    // button can be pressed from -- it disables itself while
    // m_import_pipeline.state().busy() -- but calling it first costs nothing
    // and means a future Cancel button (Phase 5) does not change this call.
    m_import_pipeline.cancel();
    m_import_pipeline.clear();
}

} // namespace stratum
