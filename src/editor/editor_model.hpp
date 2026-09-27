// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file editor_model.hpp
 * @brief The Editor's application/session state, split out from view state
 *
 * `EditorModel` groups the settings a person is authoring or has configured:
 * import options, the OSM source path, export and render settings, the rule
 * editor's source and seed, and which material is being picked. All of it
 * would still make sense if the editor had no viewport at all.
 *
 * What is deliberately NOT here: panel visibility (`Editor::m_show_*`), GPU
 * handles and futures, derived caches, and run statistics. Those stay on
 * `Editor` itself -- see CLAUDE.md and docs/agents/renderer-and-editor.md.
 */

#pragma once

#include <string>

#include "app/export_options.hpp"
#include "editor/render_settings.hpp"
#include "renderer/mesh.hpp"

namespace stratum {

/**
 * @brief Plain data: everything the user configured or is authoring
 *
 * No methods, no GPU handles. `Editor` holds one as `m_model` and every field
 * that used to be `Editor::m_foo` is now `Editor::m_model.m_foo` -- the names
 * are unchanged, only where they live.
 */
struct EditorModel {
    // ── OSM import options ──────────────────────────────────────────────────
    bool m_use_tile_culling = true;
    bool m_use_distance_culling = true;
    bool m_use_contribution_culling = true;
    float m_view_radius = 2000.0f;
    float m_contribution_threshold = 4.0f;

    /// The `.osm`/`.pbf` extract last imported, or being imported
    std::string m_osm_import_path;

    /// The "Import OSM File" text field's contents
    char m_osm_filepath[512] = "";

    // ── Road network export ─────────────────────────────────────────────────

    /// Destination, format/chunking and collision/LOD flags, edited by the OSM
    /// import panel's export section.
    ExportOptions m_export_options;

    // ── Render settings ─────────────────────────────────────────────────────

    /**
     * @brief Every lighting, sky, fog and shadow value the panel edits
     *
     * Loaded from Editor::m_render_settings_path in Editor::init() if that file
     * exists (kept at its compiled-in defaults otherwise), and pushed to the
     * renderer once in Editor::set_renderer() -- before the first frame ever
     * renders, so the persisted look is what is on screen from frame one
     * rather than a hardcoded default the panel would only correct once opened.
     */
    RenderSettings m_render_settings;

    // ── Terrain-aware roads (P3) ────────────────────────────────────────────
    // Road elevation is solved GLOBALLY, over the whole graph, BEFORE any terrain
    // chunk is carved. That is only possible because the procedural height field
    // is a pure function of (TerrainConfig, x, z) and needs no generated chunk to
    // be sampled, so the solver reaches the terrain through a callback and the
    // carve happens later, per chunk, at chunk generation time.

    /**
     * @brief Solve road heights against the terrain surface, and carve it to match
     *
     * When off, roads come out flat at the corridor base height -- the P2
     * behaviour -- and any installed carve data is dropped so the terrain returns
     * to its procedural surface.
     */
    bool m_terrain_aware_roads = true;

    /**
     * @brief Run the P4 junction solve
     *
     * Maps straight onto RoadNetworkConfig::solve_junctions. When off, every edge
     * is extruded over its full length and ribbons overlap at every junction --
     * the P2 output, and the reference the junction work is diffed against.
     *
     * Exposed because a junction defect that only shows on one extract is
     * bisectable by flipping this and re-solving, which takes a second, rather
     * than by rebuilding with the solver compiled out.
     */
    bool m_solve_junctions = true;

    /**
     * @brief Emit painted lane markings
     *
     * RoadNetworkConfig::emit_markings. Off restores the P4 surfaces exactly:
     * no centre lines, edge lines, stop lines or turn arrows.
     */
    bool m_emit_markings = true;

    /**
     * @brief Emit pedestrian crossings
     *
     * RoadNetworkConfig::emit_crossings. Independent of m_emit_markings even
     * though a zebra is Markings geometry too: a crossing is located from OSM
     * topology and a lane line is derived from the profile, so they fail in
     * different ways and are bisected separately.
     */
    bool m_emit_crossings = true;

    /**
     * @brief Emit bridge decks and tunnel portals
     *
     * RoadNetworkConfig::emit_structures. Both need a terrain height under the
     * road, so both are skipped whatever this says when terrain-aware roads are
     * off -- the panel says so rather than leaving the toggle looking broken.
     */
    bool m_emit_structures = true;

    /**
     * @brief Run the tessellation reduction passes
     *
     * RoadNetworkConfig::reduce_tessellation. Off restores the pre-reduction
     * geometry exactly, which is what the golden tests diff against, so a
     * geometry defect can be attributed to the decimator by flipping this and
     * re-solving.
     */
    bool m_reduce_tessellation = true;

    /**
     * @brief Build a chunk-level LOD chain per quadtree leaf
     *
     * QuadTree::set_chunk_lod(). Read at ASSIGNMENT, not at draw time, because it
     * also decides how road geometry is routed into the tree -- triangle by
     * triangle when on, whole pieces when off -- so flipping it re-solves the
     * network rather than only changing what is drawn.
     */
    bool m_chunk_lod = true;

    /**
     * @brief Multiplier on every ChunkLod::switch_distances entry
     *
     * 1 is the chain's own suggestion. Larger holds full detail further out and
     * costs memory; smaller drops to a coarse level sooner.
     */
    float m_road_lod_distance_scale = 1.0f;

    /**
     * @brief Force every chunk to one LOD level, or -1 to select by distance
     *
     * An inspection control. A level forced beyond a chunk's chain is clamped to
     * that chunk's coarsest, so a leaf with a short chain still draws.
     */
    int m_road_lod_override = -1;

    // ── Rule editor authoring state (D10) ───────────────────────────────────
    //
    // Deliberately PLAIN types only. A RuleFile or a GenerationResult member
    // would drag procgen/rules/ast.hpp and interpreter.hpp into every
    // translation unit that includes editor_model.hpp, for a panel that parses
    // on an edit and runs on a button. rule_panel.cpp holds both as locals and
    // keeps only what it draws, which is the same discipline material_panel.cpp
    // follows with MaterialLibrary.

    /// The rule text being edited. ImGui resizes it through a callback.
    std::string m_rule_source;

    /// Last path loaded or saved, shown in the header and used as the save default
    std::string m_rule_path;

    /**
     * @brief Re-run on every edit
     *
     * Off by default, and that is not timidity. A rule file is a program, and an
     * author halfway through typing a recursive rule has written a program that
     * does not terminate. The depth and shape caps bound it -- see
     * InterpreterLimits -- so the worst case is a stall rather than a hang, but a
     * stall on every keystroke makes the editor unusable. The PARSE runs on every
     * edit regardless: it is cheap, it cannot loop, and it is what puts a caret
     * under a typo while the author is still looking at it.
     */
    bool m_rule_auto_run = false;

    /// Source changed since the last run. Drives the "stale" marker on the output.
    bool m_rule_dirty = false;

    /**
     * @brief Where the rule gets its seed shapes from
     *
     * TestRectangle is a single rectangle on the ground plane, which is what
     * the panel opens with and what every example in examples/rules/ is
     * written against.
     *
     * ImportedBuildings runs the rule once per building in the OSM import,
     * seeded from the real footprint with the feature's tags as shape
     * attributes -- so a rule can read `attrs.get("osm.levels")` and build the
     * height the survey recorded. That is the workflow the whole project is
     * for, and until this existed the rule engine could only ever be pointed
     * at a test rectangle.
     */
    enum class RuleSeedSource { TestRectangle = 0, ImportedBuildings };
    RuleSeedSource m_rule_seed_source = RuleSeedSource::TestRectangle;

    /**
     * @brief Cap on how many imported buildings one run generates
     *
     * A city extract holds tens of thousands of footprints, and a rule that
     * makes a few hundred triangles each would produce a mesh no preview can
     * hold and take long enough that the editor looks hung. The cap is
     * REPORTED when it bites, the same way InterpreterLimits reports its own,
     * because a partial city that looks finished is the thing to avoid.
     */
    int m_rule_building_limit = 250;

    /// The seed shape: a rectangle of this size on the ground plane, in metres
    float m_rule_seed_width = 12.0f;
    float m_rule_seed_depth = 10.0f;

    /// GenerationOptions::seed. An int because ImGui has no uint64 scalar widget.
    int m_rule_seed = 0;

    // ── Material picking ────────────────────────────────────────────────────

    /**
     * @brief The material draw_material_panel() is editing
     *
     * A key, not an index or a pointer: MaterialLibrary::set() rehashes and
     * MaterialDef references are invalidated by it, so the panel re-resolves the
     * key every frame rather than holding anything across one.
     */
    MaterialKey m_selected_material{};

    /**
     * @brief The material a texture-load dialog was opened for
     *
     * Captured at open time rather than read at poll time. A native dialog is
     * modal to the window but the editor keeps running behind it on some
     * platforms, and applying a texture to whatever happens to be selected when
     * the user finally clicks Open is a way to quietly overwrite the wrong
     * material.
     */
    MaterialKey m_material_pick_key{};
};

} // namespace stratum
