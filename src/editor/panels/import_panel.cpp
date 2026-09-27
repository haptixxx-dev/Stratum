// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "editor/editor.hpp"
#include <imgui.h>
#include <cmath>
#include <cstring>
#include <map>

namespace stratum {

void Editor::draw_osm_panel() {
    ImGui::Begin("OSM");

    // Render toggles at top
    ImGui::Text("Show:");
    ImGui::SameLine();
    ImGui::Checkbox("##show_areas", &m_render_areas);
    ImGui::SameLine(); ImGui::Text("Areas");
    ImGui::SameLine();
    ImGui::Checkbox("##show_roads", &m_render_roads);
    ImGui::SameLine(); ImGui::Text("Roads");
    ImGui::SameLine();
    ImGui::Checkbox("##show_buildings", &m_render_buildings);
    ImGui::SameLine(); ImGui::Text("Bldgs");

    draw_attribute_mode_selector();

    // Culling controls
    ImGui::Checkbox("Frustum Culling", &m_model.m_use_tile_culling);
    ImGui::SameLine();
    ImGui::Checkbox("Node Grid", &m_show_tile_grid);
    ImGui::Checkbox("Contribution Culling", &m_model.m_use_contribution_culling);
    if (m_model.m_use_contribution_culling) {
        ImGui::SetNextItemWidth(120);
        ImGui::SliderFloat("Threshold (px)", &m_model.m_contribution_threshold, 1.0f, 20.0f, "%.1f");
    }
    if (m_quadtree.leaf_count() > 0) {
        ImGui::Text("Leaves: %zu, Max Depth: %d", m_quadtree.leaf_count(), m_quadtree.max_depth());
    }

    ImGui::Separator();
    ImGui::Text("OpenStreetMap Import");
    ImGui::Separator();

    // Import options section
    ImGui::Text("Import Options");

    // Get mutable reference to config
    static osm::ParserConfig config;
    ImGui::Checkbox("Buildings", &config.import_buildings);
    ImGui::Checkbox("Roads", &config.import_roads);
    ImGui::Checkbox("Water", &config.import_water);
    ImGui::Checkbox("Landuse", &config.import_landuse);
    ImGui::Checkbox("Natural", &config.import_natural);

    ImGui::Spacing();
    ImGui::DragFloat("Default Height (m)", &config.default_building_height, 0.5f, 1.0f, 100.0f);
    ImGui::DragFloat("Meters/Level", &config.meters_per_level, 0.1f, 2.0f, 5.0f);

    ImGui::Spacing();
    if (ImGui::Checkbox("Terrain-Aware Roads", &m_model.m_terrain_aware_roads)) {
        // The toggle changes which surface the roads belong on, so the current
        // solve is stale either way round. Re-solve now rather than waiting for
        // the next terrain generate.
        request_road_rebuild(app::RebuildPolicy::IfSurfaceChanged);
    }
    ImGui::SetItemTooltip(
        "Solve road heights against the terrain and carve the terrain to match.\n"
        "Off: roads stay flat and the terrain keeps its procedural surface.\n"
        "Requires chunked terrain to have been generated.");

    if (m_model.m_terrain_aware_roads && !has_generated_terrain()) {
        ImGui::TextDisabled("No terrain generated: roads will be flat.");
    } else if (m_model.m_terrain_aware_roads && !m_use_chunked_terrain) {
        ImGui::TextDisabled("Legacy terrain mode has no carve: roads will be flat.");
    }

    if (ImGui::Checkbox("Solve Junctions", &m_model.m_solve_junctions)) {
        // The toggle changes the geometry of every edge that meets another, not
        // only the junction fills: the arms are extruded from a trimmed
        // centerline. Nothing in the current network survives it, so re-solve now
        // rather than leaving the panel describing a network the toggle no longer
        // matches.
        request_road_rebuild(app::RebuildPolicy::Always);
    }
    ImGui::SetItemTooltip(
        "Trim each arm back from its node, fill the intersection, and fillet the corners.\n"
        "Off: every road is extruded full length and ribbons overlap at every junction.\n"
        "Off is the P2 reference output, kept so a junction defect can be bisected.");

    // Detail passes. Each one reproduces the previous phase exactly on its own, so
    // a visual defect can be bisected to a pass by flipping one box and
    // re-solving, which is a second, rather than by rebuilding with it compiled
    // out. All three change the geometry of the pieces themselves, so each has to
    // re-solve the network rather than only redraw it.
    if (ImGui::Checkbox("Lane Markings", &m_model.m_emit_markings)) {
        request_road_rebuild(app::RebuildPolicy::Always);
    }
    ImGui::SetItemTooltip(
        "Centre lines, edge lines, stop lines, give-way triangles and turn arrows.\n"
        "Painted into the Markings material as separate quads above the surface.\n"
        "Off: the carriageway keeps its surfaces and carries no paint.");

    ImGui::SameLine();
    if (ImGui::Checkbox("Crossings", &m_model.m_emit_crossings)) {
        request_road_rebuild(app::RebuildPolicy::Always);
    }
    ImGui::SetItemTooltip(
        "Zebra stripes at highway=crossing nodes, and dropped kerbs in the curb ring.\n"
        "Independent of Lane Markings: a crossing is found from OSM topology, a lane\n"
        "line is derived from the profile, and the two fail in different ways.");

    if (ImGui::Checkbox("Bridges and Tunnels", &m_model.m_emit_structures)) {
        request_road_rebuild(app::RebuildPolicy::Always);
    }
    ImGui::SetItemTooltip(
        "Bridge deck slabs, parapets and piers; tunnel portal headwalls.\n"
        "Both are cut against the ground under the road, so both need terrain-aware\n"
        "roads. Off: a bridge is a bare ribbon and a tunnel has no mouth.");

    if (m_model.m_emit_structures && !m_model.m_terrain_aware_roads) {
        ImGui::TextDisabled("Structures need terrain-aware roads: none will be emitted.");
    }

    // Geometry reduction. Both change what is built rather than what is drawn, so
    // both re-solve, and both are bisectable the same way the detail passes are.
    if (ImGui::Checkbox("Reduce Tessellation", &m_model.m_reduce_tessellation)) {
        request_road_rebuild(app::RebuildPolicy::Always);
    }
    ImGui::SetItemTooltip(
        "Drop stations a straight road does not need, and merge coplanar strip quads.\n"
        "Bounded by a chord deviation and a span cap, so the centerline never moves far.\n"
        "Off: the pre-reduction geometry, which the golden tests diff against.");

    ImGui::SameLine();
    if (ImGui::Checkbox("Chunk LOD", &m_model.m_chunk_lod)) {
        // Not a draw-time switch. It decides how pieces are routed into the
        // leaves -- triangle by triangle when on -- so the tree has to be
        // rebuilt, not merely redrawn.
        request_road_rebuild(app::RebuildPolicy::Always);
    }
    ImGui::SetItemTooltip(
        "Merge each leaf's road pieces and simplify the merged mesh into a level chain.\n"
        "Only the level the camera distance selects is ever uploaded.\n"
        "Off: every leaf keeps one full-detail mesh, routed whole by piece anchor.");

    ImGui::Separator();

    // File path input. Kept alongside the picker so a path can still be pasted or
    // typed, which is also the fallback if the platform has no dialog available.
    ImGui::InputText("File Path", m_model.m_osm_filepath, sizeof(m_model.m_osm_filepath));
    ImGui::SameLine();
    ImGui::BeginDisabled(m_file_pick.pending);
    if (ImGui::Button(m_file_pick.pending ? "Browsing..." : "Browse...")) {
        open_osm_file_dialog();
    }
    ImGui::EndDisabled();

    // An export re-solves the network from the pipeline's parsed data on a
    // worker, so it locks the parser for exactly the same reason an import in
    // flight does. state().busy() covers both: parsing through carving, an
    // owed-rebuild's second BuildingRoads, and an export in flight.
    const app::PipelineState& pstate = m_import_pipeline.state();
    const bool importing = pstate.busy();

    ImGui::BeginDisabled(importing);
    if (ImGui::Button("Import OSM File", ImVec2(-1, 0))) {
        if (strlen(m_model.m_osm_filepath) == 0) {
            m_import_status = "Please enter a file path first";
            m_import_error = true;
        } else {
            m_import_status.clear();
            m_import_error = false;
            app::ImportOptions options;
            options.parser = config;
            options.roads = road_options();
            m_import_pipeline.begin_import(m_model.m_osm_filepath, options);
        }
    }
    ImGui::EndDisabled();

    // Progress, driven by m_import_pipeline.tick() (called from Editor::render()).
    if (importing) {
        const char* stage_label =
            pstate.stage == app::ImportStage::Parsing        ? "1/5 Parsing" :
            pstate.stage == app::ImportStage::BuildingRoads  ? "2/5 Road network" :
            pstate.stage == app::ImportStage::Indexing       ? "3/5 Spatial index" :
            pstate.stage == app::ImportStage::BuildingMeshes ? "4/5 Building meshes" :
                                                               "5/5 Terrain carve";

        // The parser reports item counts only for some stages, so a zero fraction
        // means "unknown", not "nothing done" -- show an indeterminate bar rather
        // than one frozen at 0%.
        if (pstate.fraction > 0.0f) {
            ImGui::ProgressBar(pstate.fraction, ImVec2(-1, 0));
        } else {
            const float t = fmodf((float)ImGui::GetTime() * 0.8f, 1.0f);
            ImGui::ProgressBar(-1.0f * t, ImVec2(-1, 0), "working...");
        }
        ImGui::Text("%s", stage_label);
        if (!pstate.message.empty()) {
            ImGui::SameLine();
            ImGui::TextDisabled("- %s", pstate.message.c_str());
        }
        if (pstate.stage == app::ImportStage::BuildingMeshes && pstate.nodes_total > 0) {
            ImGui::Text("Nodes: %zu / %zu", pstate.nodes_done, pstate.nodes_total);
        }
    } else if (pstate.stage == app::ImportStage::Failed) {
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s", pstate.message.c_str());
    } else if (pstate.stage == app::ImportStage::Done) {
        ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "%s", pstate.message.c_str());
    }

    // Show status message
    if (!m_import_status.empty()) {
        if (m_import_error) {
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%s", m_import_status.c_str());
        } else {
            ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "%s", m_import_status.c_str());
        }
    }

    // ── Export ──────────────────────────────────────────────────────────────
    // The only export section: it used to be duplicated in the GPU Memory panel,
    // which drew the same controls bound to the same Editor state.
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Text("Export road network");
    ImGui::Separator();

    {
        const bool busy = pstate.export_running;
        const bool export_blocked = importing; // an import/rebuild also locks the parser
        const bool have_roads = m_import_pipeline.has_roads();

        ImGui::BeginDisabled(busy);

        ImGui::SetNextItemWidth(-90.0f);
        ImGui::InputText("##ExportDir", m_model.m_export_options.dir, sizeof(m_model.m_export_options.dir));
        ImGui::SameLine();
        if (ImGui::Button("Browse##Export", ImVec2(-1, 0))) {
            open_export_dir_dialog();
        }

        int format = static_cast<int>(m_model.m_export_options.config.format);
        const char* formats[] = { "OBJ + MTL", "glTF 2.0 + .bin" };
        if (ImGui::Combo("Format", &format, formats, 2)) {
            m_model.m_export_options.config.format = static_cast<osm::road::ExportFormat>(format);
        }

        ImGui::SliderFloat("Chunk size", &m_model.m_export_options.config.chunk_size, 0.0f, 2000.0f,
                           "%.0f m");
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("0 writes the whole network as one file. The grid is anchored at "
                              "the\nworld origin, so two overlapping exports line up.");
        }

        ImGui::Checkbox("Collision mesh", &m_model.m_export_options.build_collision);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Derives a flat collision variant per piece during the re-solve.\n"
                              "Costs roughly a third of the render mesh again.");
        }

        ImGui::Checkbox("LOD chain", &m_model.m_export_options.build_lods);
        if (ImGui::IsItemHovered()) {
            ImGui::SetTooltip("Simplifies once per material range per level. The most expensive\n"
                              "option by a wide margin on a city extract.");
        }
        if (m_model.m_export_options.build_lods) {
            ImGui::SliderInt("LOD levels", &m_model.m_export_options.config.lod_levels, 2, 4);
        }

        ImGui::BeginDisabled(!have_roads || export_blocked);
        if (ImGui::Button("Export", ImVec2(-1, 0))) {
            if (const app::Launch launch = m_import_pipeline.begin_export(m_model.m_export_options);
                !launch.started) {
                m_export_status = launch.reason;
            } else {
                m_export_status = "Exporting...";
            }
        }
        ImGui::EndDisabled();

        ImGui::EndDisabled();

        if (busy) {
            // Indeterminate: the exporter reports nothing until it returns, and a bar
            // that sat at 0% would read as a hang.
            ImGui::ProgressBar(-1.0f * static_cast<float>(ImGui::GetTime()), ImVec2(-1, 0),
                               "Re-solving and writing...");
        } else if (!have_roads) {
            ImGui::TextDisabled("Import an OSM file with roads first");
        }

        if (!m_export_status.empty()) {
            ImGui::TextWrapped("%s", m_export_status.c_str());
        }
    }

    ImGui::Separator();

    // Display loaded data statistics
    if (m_import_pipeline.parser().has_data()) {
        const auto& data = m_import_pipeline.parser().get_data();

        ImGui::Text("Loaded Data:");
        ImGui::BulletText("Nodes: %zu", data.stats.total_nodes);
        ImGui::BulletText("Ways: %zu", data.stats.total_ways);
        ImGui::BulletText("Relations: %zu", data.stats.total_relations);

        ImGui::Spacing();
        ImGui::Text("Processed:");
        ImGui::BulletText("Roads: %zu", data.roads.size());
        ImGui::BulletText("Buildings: %zu", data.buildings.size());
        ImGui::BulletText("Areas: %zu", data.areas.size());

        if (data.bounds.is_valid()) {
            ImGui::Spacing();
            ImGui::Text("Bounds:");
            ImGui::BulletText("Lat: [%.4f, %.4f]", data.bounds.min_lat, data.bounds.max_lat);
            ImGui::BulletText("Lon: [%.4f, %.4f]", data.bounds.min_lon, data.bounds.max_lon);
            ImGui::BulletText("Size: ~%.0fm x %.0fm",
                            data.bounds.width_meters(), data.bounds.height_meters());
        }

        ImGui::Spacing();
        ImGui::Text("Timing:");
        ImGui::BulletText("Parse: %.1f ms", data.stats.parse_time_ms);
        ImGui::BulletText("Process: %.1f ms", data.stats.process_time_ms);

        if (pstate.roads.have_stats) {
            ImGui::Spacing();
            ImGui::Text("Road Network:");
            ImGui::BulletText("Pieces: %zu (%zu triangles)",
                              pstate.roads.network.pieces, pstate.roads.network.triangles);
            ImGui::BulletText("Build: %.1f ms", pstate.roads.network.build_ms);

            ImGui::Spacing();
            if (pstate.roads.built_on_terrain) {
                ImGui::Text("Elevation Solve:");
                ImGui::BulletText("Edges: %zu of %zu elevated in %.1f ms",
                                  pstate.roads.network.elevated_edges, pstate.roads.network.edges,
                                  pstate.roads.network.elevation_ms);
                ImGui::BulletText("Iterations: %zu (residual %.3f m)",
                                  pstate.roads.elevation.iterations,
                                  pstate.roads.elevation.max_residual);
                ImGui::BulletText("Max grade: %.1f%% (%zu edges grade-limited)",
                                  pstate.roads.max_grade * 100.0f,
                                  pstate.roads.elevation.grade_limited_edges);
                ImGui::BulletText("Bridges: %zu   Tunnels: %zu",
                                  pstate.roads.elevation.bridges,
                                  pstate.roads.elevation.tunnels);
                if (m_terrain_tile_manager.has_road_carve_data()) {
                    ImGui::BulletText("Terrain carved: yes");
                } else {
                    ImGui::BulletText("Terrain carved: no");
                }
            } else {
                ImGui::TextDisabled("Roads are flat: no terrain surface to follow.");
            }

            ImGui::Spacing();
            if (pstate.roads.solved_junctions) {
                ImGui::Text("Junctions:");
                ImGui::BulletText("Solved: %zu   Roundabouts: %zu",
                                  pstate.roads.junctions.junctions,
                                  pstate.roads.junctions.roundabouts);
                ImGui::BulletText("Tapers: %zu   Dead ends: %zu",
                                  pstate.roads.junctions.tapers,
                                  pstate.roads.junctions.dead_ends);
                ImGui::BulletText("Pieces: %zu   Trimmed edges: %zu",
                                  pstate.roads.network.junction_pieces,
                                  pstate.roads.network.trimmed_edges);
                ImGui::BulletText("Solve: %.1f ms", pstate.roads.network.junction_ms);

                // A degenerate node fell back to a provisional disc and emitted no
                // fill; an over-trimmed edge is one the junction polygon still
                // overlaps. Neither is visible in the geometry without looking for
                // it, so both are called out rather than buried in the list above.
                if (pstate.roads.junctions.degenerate > 0) {
                    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
                                       "  %zu degenerate: arms too wide for the node.",
                                       pstate.roads.junctions.degenerate);
                }
                if (pstate.roads.junctions.over_trimmed_edges > 0) {
                    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
                                       "  %zu over-trimmed: the trim clamp bound.",
                                       pstate.roads.junctions.over_trimmed_edges);
                }
                if (pstate.roads.network.trimmed_away_edges > 0) {
                    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
                                       "  %zu edges consumed entirely by their trims.",
                                       pstate.roads.network.trimmed_away_edges);
                }
            } else {
                ImGui::TextDisabled("Junction solver off: ribbons overlap at every node.");
            }

            ImGui::Spacing();
            ImGui::Text("Detail:");

            // Every count below is zero both when its pass was switched off and
            // when the pass ran and found nothing. The two mean opposite things to
            // anyone reading the panel, so the flag decides which is shown and the
            // count is never left to imply it.
            if (pstate.roads.emitted_markings) {
                ImGui::BulletText("Marking pieces: %zu", pstate.roads.network.markings_pieces);
            } else {
                ImGui::BulletText("Marking pieces: off");
            }

            if (pstate.roads.emitted_crossings) {
                ImGui::BulletText("Crossings: %zu", pstate.roads.network.crossings);
            } else {
                ImGui::BulletText("Crossings: off");
            }

            if (pstate.roads.emitted_structures) {
                ImGui::BulletText("Bridges: %zu   Tunnels: %zu (%zu portal mouths)",
                                  pstate.roads.network.bridges, pstate.roads.network.tunnels,
                                  pstate.roads.portal_mouths);
            } else if (!m_model.m_emit_structures) {
                ImGui::BulletText("Bridges and tunnels: off");
            } else {
                // The flag was on but the builder skipped the pass, which it does
                // whenever there is no terrain to cut a pier or a portal against.
                ImGui::BulletText("Bridges and tunnels: skipped, no terrain surface");
            }

            // Counted per SIDE, not per edge: an edge whose sidewalk is separately
            // mapped on both sides adds two.
            ImGui::BulletText("Sidewalk sides deduped: %zu", pstate.roads.network.deduped_sidewalks);

            ImGui::Spacing();
            ImGui::Text("Tessellation:");
            if (m_model.m_reduce_tessellation) {
                const size_t before = pstate.roads.network.stations_before;
                const size_t after = pstate.roads.network.stations_after;
                ImGui::BulletText("Stations: %zu -> %zu (%.1f%%)", before, after,
                                  before ? 100.0 * static_cast<double>(after)
                                                 / static_cast<double>(before)
                                         : 100.0);
                // A merge removes two triangles, so the pair is what the lateral
                // pass actually contributed and the count alone is half the story.
                ImGui::BulletText("Quads merged: %zu (%zu triangles)",
                                  pstate.roads.network.quads_merged, pstate.roads.network.quads_merged * 2);
                const size_t tri_before = pstate.roads.network.triangles_before_tess;
                ImGui::BulletText("Triangles: %zu -> %zu (%.1f%%)", tri_before,
                                  pstate.roads.network.triangles,
                                  tri_before ? 100.0 * static_cast<double>(pstate.roads.network.triangles)
                                                     / static_cast<double>(tri_before)
                                             : 100.0);
                ImGui::BulletText("Corridor kerbs dropped on %zu edges",
                                  pstate.roads.network.corridor_kerb_edges);
            } else {
                ImGui::BulletText("Reduction: off (%zu stations, %zu triangles)",
                                  pstate.roads.network.stations_before, pstate.roads.network.triangles);
            }

            draw_chunk_lod_stats();

            // A portal mouth is the only carve primitive the road geometry cannot
            // stand without: the headwall frames an opening the hillside would
            // otherwise close over. Say so when portals were built and the terrain
            // never received them.
            if (pstate.roads.portal_mouths > 0 && !m_terrain_tile_manager.has_road_carve_data()) {
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
                                   "  %zu portal mouths are not carved: the terrain has no "
                                   "carve data.",
                                   pstate.roads.portal_mouths);
            }

            // The surface the roads WOULD be solved against right now. It differs
            // from the one they WERE solved against whenever terrain was
            // generated or regenerated while an import was in flight, or the
            // terrain-aware toggle was flipped and the rebuild was refused.
            const uint64_t live_surface = m_import_pipeline.live_terrain_fingerprint();

            if (live_surface != pstate.roads.terrain_fingerprint) {
                // A live surface of zero is not "a different terrain", it is no
                // terrain at all: the chunks were cleared, chunked mode was
                // switched off, or terrain-aware roads were switched off. The
                // rebuild in that direction flattens the network rather than
                // elevating it, so say which one the button does.
                const bool to_flat = (live_surface == 0);
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
                                   to_flat ? "Roads are elevated but no terrain surface is active."
                                           : "Roads were solved against a different terrain.");
                ImGui::BeginDisabled(importing);
                if (ImGui::Button(to_flat ? "Re-solve Roads Flat" : "Re-solve Against Terrain",
                                  ImVec2(-1, 0))) {
                    request_road_rebuild(app::RebuildPolicy::Always);
                }
                ImGui::EndDisabled();
            }
        }

        // Road type breakdown
        if (!data.roads.empty() && ImGui::TreeNode("Road Types")) {
            std::map<osm::RoadType, int> road_counts;
            for (const auto& road : data.roads) {
                road_counts[road.type]++;
            }
            for (const auto& [type, count] : road_counts) {
                ImGui::BulletText("%s: %d", osm::road_type_name(type), count);
            }
            ImGui::TreePop();
        }

        // Building type breakdown
        if (!data.buildings.empty() && ImGui::TreeNode("Building Types")) {
            std::map<osm::BuildingType, int> building_counts;
            for (const auto& bldg : data.buildings) {
                building_counts[bldg.type]++;
            }
            for (const auto& [type, count] : building_counts) {
                ImGui::BulletText("%s: %d", osm::building_type_name(type), count);
            }
            ImGui::TreePop();
        }

        // Area type breakdown
        if (!data.areas.empty() && ImGui::TreeNode("Area Types")) {
            std::map<osm::AreaType, int> area_counts;
            for (const auto& area : data.areas) {
                area_counts[area.type]++;
            }
            for (const auto& [type, count] : area_counts) {
                ImGui::BulletText("%s: %d", osm::area_type_name(type), count);
            }
            ImGui::TreePop();
        }

        ImGui::Separator();
        // Clearing mid-import would drop quadtree nodes the pipeline's own
        // pending-node list still points at; m_import_pipeline.clear() refuses
        // outright while state().busy(), but disabling the button here too
        // keeps it from ever being pressed during an import in the first place.
        ImGui::BeginDisabled(importing);
        if (ImGui::Button("Clear Data", ImVec2(-1, 0))) {
            clear_imported_data();
        }
        ImGui::EndDisabled();
    }

    ImGui::End();
}

void Editor::draw_toolbar() {
    // Implemented as overlay in viewport
}

} // namespace stratum
