// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file rule_panel.cpp
 * @brief The rule editor: write a rule file, run it, see the building
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * D1 to D4 built a lexer, a parser, an evaluator, `split`, `select` and the
 * geometry operations. Until this panel existed, every one of them could only
 * be exercised from a test executable. Track D's whole claim -- that Stratum
 * has a shape grammar -- was true and invisible at the same time.
 *
 * ### What the panel is for, in order
 *
 * 1. **Write.** A text box over the rule source, and the file it came from.
 * 2. **See the fault.** Every diagnostic is the full render_diagnostic() block,
 *    caret and all. interpreter.hpp promises a runtime error looks exactly like
 *    a parse error, and this panel is where that promise is either kept or
 *    quietly broken -- so both go through the same renderer and the same list.
 * 3. **See the building.** The terminals, merged into one mesh and drawn in the
 *    viewport with the rest of the opaque scene.
 *
 * ### What the rule is pointed AT
 *
 * A test rectangle, or every building in the OSM import. The second is the
 * workflow the project exists for -- import an extract and generate over what
 * came back -- and until it existed the rule engine could only ever be aimed
 * at a rectangle.
 *
 * An imported seed carries its feature's tags as shape attributes, so a rule
 * reads `attrs.get("osm.levels", 3)` and builds the height the survey
 * recorded rather than a number written into the rule file. osm/rule_seed.hpp
 * lists the names.
 *
 * ### Parse on every edit; RUN on a button
 *
 * The parse is cheap, cannot loop, and is what puts a caret under a typo while
 * the author is still looking at it. The run is a program, and an author
 * halfway through typing a recursive rule has written one that does not
 * terminate. InterpreterLimits bounds it -- the depth and shape caps mean the
 * worst case is a stall rather than a hang -- but a stall on every keystroke
 * makes the editor unusable. So auto-run is off by default and the output pane
 * says "stale" rather than pretending.
 *
 * ### The registries this panel passes
 *
 * `standard_operations()` is the D2 set only, and a panel that passed it alone
 * would report `set` and `tag` as "not implemented in this build" in an editor
 * where they demonstrably are. The union lives in procgen/rules/registry.hpp
 * rather than here, because the Python bindings and any headless generation
 * path need exactly the same table and must not each assemble their own.
 */

#include "editor/editor.hpp"

#include "procgen/rules/interpreter.hpp"
#include "procgen/rules/parser.hpp"
#include "osm/parser.hpp"
#include "osm/rule_seed.hpp"
#include "procgen/rules/registry.hpp"
#include "procgen/rules/shape.hpp"
#include "renderer/gpu_renderer.hpp"

#include <imgui.h>
#include <spdlog/spdlog.h>

#include <cstdint>
#include <string>

namespace stratum {

namespace {

using namespace stratum::procgen::rules;

/// Diagnostics kept from ONE run, across every seed it generated
constexpr size_t kMaxRunDiagnostics = 40;

/// `print` lines kept from one run. A city extract can emit one per building.
constexpr size_t kMaxRunLogLines = 200;

/**
 * @brief Append one generation's mesh onto @p combined
 *
 * Mesh::append() does the work -- it renumbers the indices, expands the
 * bounds, and materialises the implicit whole-mesh submesh range so the
 * geometry already there keeps its identity. Rebuilding that here would be a
 * second copy of logic that is already right.
 *
 * Every building is attributed to one material, so a preview of a thousand of
 * them is one draw call rather than a thousand tiny ones.
 */
void append_result_to_mesh(const GenerationResult& result, Mesh& combined) {
    const Mesh piece = result.build_mesh();
    if (piece.vertices.empty() || piece.indices.empty()) return;
    combined.append(piece, MaterialId::Concrete);
}

/// ImGui resize callback for an InputTextMultiline over a std::string
int rule_source_resize(ImGuiInputTextCallbackData* data) {
    if (data->EventFlag == ImGuiInputTextFlags_CallbackResize) {
        auto* text = static_cast<std::string*>(data->UserData);
        text->resize(static_cast<size_t>(data->BufTextLen));
        data->Buf = text->data();
    }
    return 0;
}

/**
 * @brief The rule file a fresh editor opens with
 *
 * Not a comment saying "write a rule here". It is a whole building -- floors,
 * a facade split into bays, windows with reveals and a hipped roof -- so the
 * first thing a person sees after pressing Generate is the thing the language
 * is for.
 *
 * It was the facade from tests/procgen/test_rule_statements.cpp, which checks
 * the split ARITHMETIC and nothing else: every one of its terminals is an
 * empty rule, so every terminal is a coplanar quad on the wall plane and the
 * whole thing renders as a flat wall. Correct as a test, useless as a demo.
 * `window()` is what gives an opening its depth, and this file now calls it.
 */
constexpr const char* kDefaultRuleSource =
    "// A shopfront with four floors of bays above it.\n"
    "// Press Generate, then change a number and press it again.\n"
    "\n"
    "@start\n"
    "rule Main {\n"
    "    floors(4, 3.0);\n"
    "    select face { front : { Facade(); } top : { Roof(); } }\n"
    "}\n"
    "\n"
    "// The wall is 12 m tall, so the shopfront takes 4 and the repeat tiles\n"
    "// the remaining 8. Give the shopfront all 12 and the repeat gets nothing.\n"
    "rule Facade {\n"
    "    split(y) {\n"
    "        4.0 : { Shopfront(); }\n"
    "        repeat { 2.0 : { UpperFloor(); } }\n"
    "    }\n"
    "}\n"
    "\n"
    "rule UpperFloor {\n"
    "    split(x) { repeat { 1.5 : { Bay(); } } }\n"
    "}\n"
    "\n"
    "// The ~ is a FLOATING size: it absorbs whatever the fixed parts leave.\n"
    "rule Bay {\n"
    "    split(x) {\n"
    "        0.4 : { Pier(); }\n"
    "        0.6 : { Window(); }\n"
    "        ~1.0 : { Pier(); }\n"
    "    }\n"
    "}\n"
    "\n"
    "// window() cuts a real opening with a reveal, a frame and a sill.\n"
    "// An empty `rule Window {}` only marks the region, and renders flat.\n"
    "rule Window { window(); }\n"
    "\n"
    "// A face from `select face` has its normal along local z, so the frame\n"
    "// has to be turned before a roof can rise on it.\n"
    "rule Roof { align_scope(\"y_up\"); roof(\"hip\", 35.0); }\n"
    "\n"
    "rule Shopfront {}\n"
    "rule Pier {}\n";

} // namespace

// ============================================================================
// Running
// ============================================================================

void Editor::clear_rule_preview() {
    if (m_gpu_renderer) {
        release_tracked_mesh(*m_gpu_renderer, m_rule_preview_gpu_id);
    } else {
        // No renderer means no handle was ever taken. Zero it anyway so the
        // draw guard cannot see a stale id if one is attached later.
        m_rule_preview_gpu_id = 0;
    }
    m_rule_preview_mesh = Mesh{};
}

void Editor::run_rule_source() {
    m_rule_diagnostics.clear();
    m_rule_log.clear();
    m_rule_has_error = false;

    const ParseResult parsed = parse(m_rule_source, m_rule_path.empty() ? "<editor>" : m_rule_path);
    for (const Diagnostic& diagnostic : parsed.diagnostics) {
        m_rule_diagnostics.push_back(render_diagnostic(diagnostic, m_rule_source, parsed.filename));
        if (diagnostic.severity == Severity::Error) m_rule_has_error = true;
    }

    // A file that did not parse has no program to run. Its partial AST is real
    // -- parser.hpp guarantees it is never garbage -- but running it would
    // produce geometry from half a rule and a second pile of diagnostics
    // describing the consequences of the first pile, which buries the fault
    // that actually matters.
    if (!parsed.ok()) {
        clear_rule_preview();
        m_rule_terminals = 0;
        m_rule_shapes = 0;
        m_rule_rules_invoked = 0;
        m_rule_operations = 0;
        m_rule_max_depth = 0;
        m_rule_depth_capped = false;
        m_rule_shape_capped = false;
        m_rule_has_run = true;
        m_rule_dirty = false;
        return;
    }

    GenerationOptions options;
    options.seed = static_cast<uint64_t>(static_cast<uint32_t>(m_rule_seed));

    // Every seed the run will use. One rectangle, or one per imported
    // building -- the loop below is the same either way, which is what keeps
    // the two paths from drifting apart.
    std::vector<Shape> seeds;
    m_rule_buildings_built = 0;
    m_rule_buildings_skipped = 0;

    if (m_rule_seed_source == RuleSeedSource::ImportedBuildings && m_osm_parser.has_data()) {
        const auto& buildings = m_osm_parser.get_data().buildings;
        const size_t limit = m_rule_building_limit > 0
                                 ? static_cast<size_t>(m_rule_building_limit)
                                 : buildings.size();
        for (const osm::Building& building : buildings) {
            if (seeds.size() >= limit) {
                ++m_rule_buildings_skipped;
                continue;
            }
            Shape seed = osm::seed_from_building(building);
            // A degenerate footprint seeds an empty shape. Running a rule on
            // it produces nothing and costs a diagnostic per building, which
            // on a city extract buries every message that matters.
            if (seed.geometry.faces.empty()) {
                ++m_rule_buildings_skipped;
                continue;
            }
            seeds.push_back(std::move(seed));
        }
    } else {
        seeds.push_back(shape_from_rect(static_cast<double>(m_rule_seed_width),
                                        static_cast<double>(m_rule_seed_depth)));
    }

    GenerationStats totals{};
    Mesh combined;
    size_t diagnostics_shown = 0;

    for (const Shape& seed : seeds) {
        const GenerationResult result =
            generate(parsed.file, seed, options, &full_operations(), &full_functions());

        // One building's diagnostics are worth reading; four hundred copies of
        // the same one are not. The budget is the same idea as
        // GenerationResult's own cap, applied across the run rather than
        // within it.
        for (const Diagnostic& diagnostic : result.diagnostics) {
            if (diagnostics_shown < kMaxRunDiagnostics) {
                m_rule_diagnostics.push_back(
                    render_diagnostic(diagnostic, m_rule_source, parsed.filename));
                ++diagnostics_shown;
            }
            if (diagnostic.severity == Severity::Error) m_rule_has_error = true;
        }
        for (const std::string& line : result.log) {
            if (m_rule_log.size() < kMaxRunLogLines) m_rule_log.push_back(line);
        }

        totals.terminals += result.stats.terminals;
        totals.shapes_created += result.stats.shapes_created;
        totals.rules_invoked += result.stats.rules_invoked;
        totals.operations_applied += result.stats.operations_applied;
        totals.max_depth_reached =
            std::max(totals.max_depth_reached, result.stats.max_depth_reached);
        totals.depth_limit_hit = totals.depth_limit_hit || result.stats.depth_limit_hit;
        totals.shape_limit_hit = totals.shape_limit_hit || result.stats.shape_limit_hit;

        append_result_to_mesh(result, combined);
        ++m_rule_buildings_built;
    }

    if (diagnostics_shown >= kMaxRunDiagnostics) {
        m_rule_diagnostics.push_back(
            "... more diagnostics were produced and are not shown. Fix these first,\n"
            "or switch the seed back to the test rectangle to read one building's.\n");
    }

    m_rule_terminals = totals.terminals;
    m_rule_shapes = totals.shapes_created;
    m_rule_rules_invoked = totals.rules_invoked;
    m_rule_operations = totals.operations_applied;
    m_rule_max_depth = totals.max_depth_reached;
    m_rule_depth_capped = totals.depth_limit_hit;
    m_rule_shape_capped = totals.shape_limit_hit;
    m_rule_has_run = true;
    m_rule_dirty = false;

    // The old preview goes whatever happens next. A run that produced nothing
    // must leave an empty viewport, not the previous building -- otherwise a
    // rule that stopped generating looks like a rule that still works.
    clear_rule_preview();

    m_rule_preview_mesh = std::move(combined);
    if (m_rule_preview_mesh.vertices.empty() || !m_gpu_renderer) return;

    MeshOwner owner;
    owner.kind = MeshOwner::Kind::Pinned;
    owner.anchor = m_rule_preview_mesh.bounds.center();
    m_rule_preview_gpu_id = upload_tracked_mesh(*m_gpu_renderer, m_rule_preview_mesh, owner);
    if (m_rule_preview_gpu_id == 0) {
        spdlog::warn("Rule preview upload failed: {} vertices", m_rule_preview_mesh.vertices.size());
    }
}

// ============================================================================
// Drawing
// ============================================================================

void Editor::draw_rule_source() {
    if (ImGui::Button("Load...")) open_file_dialog(FilePickTarget::RuleFileLoad);
    ImGui::SameLine();
    if (ImGui::Button("Save As...")) open_file_dialog(FilePickTarget::RuleFileSave);
    ImGui::SameLine();
    ImGui::BeginDisabled(m_rule_path.empty());
    if (ImGui::Button("Revert")) {
        // Re-open the dialog rather than re-reading the path directly: the file
        // may have moved, and the one place a path turns into bytes is
        // poll_file_dialog(), which already handles a read that fails.
        open_file_dialog(FilePickTarget::RuleFileLoad);
    }
    ImGui::EndDisabled();

    ImGui::TextUnformatted(m_rule_path.empty() ? "(unsaved)" : m_rule_path.c_str());
    if (!m_rule_status.empty()) {
        ImGui::TextDisabled("%s", m_rule_status.c_str());
    }

    ImGui::Separator();

    // Where the seeds come from. The imported option is disabled rather than
    // hidden when there is no import, so the feature is discoverable before
    // anyone has loaded an extract.
    const bool have_import = m_osm_parser.has_data() &&
                             !m_osm_parser.get_data().buildings.empty();

    int source = static_cast<int>(m_rule_seed_source);
    ImGui::SetNextItemWidth(200.0f);
    if (ImGui::Combo("Seed from", &source, "Test rectangle\0Imported buildings\0")) {
        if (source == 1 && !have_import) {
            source = 0;
        }
        m_rule_seed_source = static_cast<RuleSeedSource>(source);
        m_rule_dirty = true;
    }
    if (!have_import) {
        ImGui::SameLine();
        ImGui::TextDisabled("(no OSM import loaded)");
    } else if (m_rule_seed_source == RuleSeedSource::ImportedBuildings) {
        ImGui::SameLine();
        ImGui::Text("%zu available", m_osm_parser.get_data().buildings.size());
    }

    if (m_rule_seed_source == RuleSeedSource::ImportedBuildings) {
        ImGui::SetNextItemWidth(160.0f);
        ImGui::InputInt("Max buildings", &m_rule_building_limit);
        if (m_rule_building_limit < 1) m_rule_building_limit = 1;
        ImGui::SetItemTooltip(
            "A city extract holds tens of thousands of footprints. Generating "
            "all of them makes a mesh no preview can hold and an editor that "
            "looks hung. What was skipped is reported under Output.");
        // The rectangle's size is meaningless here, so it is not drawn. A
        // control that does nothing is worse than a missing one.
    } else {
        ImGui::SetNextItemWidth(120.0f);
        ImGui::DragFloat("Width (m)", &m_rule_seed_width, 0.1f, 0.5f, 500.0f, "%.2f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120.0f);
        ImGui::DragFloat("Depth (m)", &m_rule_seed_depth, 0.1f, 0.5f, 500.0f, "%.2f");
    }
    ImGui::SetNextItemWidth(160.0f);
    ImGui::InputInt("Seed", &m_rule_seed);
    ImGui::SetItemTooltip(
        "GenerationOptions::seed. Mixed with the shape's own key, so the same "
        "seed reproduces the same building exactly.");

    if (ImGui::Button("Generate")) run_rule_source();
    ImGui::SameLine();
    ImGui::Checkbox("Run on edit", &m_rule_auto_run);
    ImGui::SetItemTooltip(
        "Off by default. The parse runs on every edit regardless; only the "
        "generation waits, because a half-typed recursive rule is a program "
        "that does not terminate.");
    ImGui::SameLine();
    ImGui::Checkbox("Show preview", &m_render_rule_preview);

    if (m_rule_dirty && m_rule_has_run) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.2f, 1.0f), "stale");
        ImGui::SetItemTooltip("The source changed since this output was produced.");
    }

    // The box takes the space the two panes below do not.
    const float reserved = ImGui::GetTextLineHeightWithSpacing() * 12.0f;
    const float height = ImGui::GetContentRegionAvail().y - reserved;
    const ImGuiInputTextFlags flags =
        ImGuiInputTextFlags_AllowTabInput | ImGuiInputTextFlags_CallbackResize;

    // data() rather than &m_rule_source[0] so an empty string is still a valid
    // pointer; capacity+1 because ImGui writes its own terminator.
    if (ImGui::InputTextMultiline("##rule_source", m_rule_source.data(),
                                  m_rule_source.capacity() + 1,
                                  ImVec2(-FLT_MIN, height > 80.0f ? height : 80.0f),
                                  flags, rule_source_resize, &m_rule_source)) {
        m_rule_dirty = true;
        if (m_rule_auto_run) {
            run_rule_source();
        } else {
            // Parse only. Cheap, cannot loop, and it is what puts the caret
            // under a typo while the author is still looking at it. The stats
            // and the preview stay as they were and are marked stale above.
            m_rule_diagnostics.clear();
            m_rule_has_error = false;
            const ParseResult parsed =
                parse(m_rule_source, m_rule_path.empty() ? "<editor>" : m_rule_path);
            for (const Diagnostic& diagnostic : parsed.diagnostics) {
                m_rule_diagnostics.push_back(
                    render_diagnostic(diagnostic, m_rule_source, parsed.filename));
                if (diagnostic.severity == Severity::Error) m_rule_has_error = true;
            }
        }
    }
}

void Editor::draw_rule_diagnostics() {
    const size_t count = m_rule_diagnostics.size();
    if (count == 0) {
        ImGui::TextColored(ImVec4(0.4f, 0.85f, 0.4f, 1.0f), "No diagnostics");
        return;
    }

    if (m_rule_has_error) {
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%zu diagnostic%s",
                           count, count == 1 ? "" : "s");
    } else {
        ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "%zu warning%s",
                           count, count == 1 ? "" : "s");
    }

    // Monospace is not available, but the caret line render_diagnostic() emits
    // only lines up under a fixed-width font. TextUnformatted keeps the newlines
    // and the leading spaces; with a proportional font the caret sits near the
    // token rather than exactly under it, which is still enough to find it.
    for (const std::string& rendered : m_rule_diagnostics) {
        ImGui::TextUnformatted(rendered.c_str());
    }
}

void Editor::draw_rule_stats() {
    if (!m_rule_has_run) {
        ImGui::TextDisabled("Not run yet.");
        return;
    }

    ImGui::Text("Terminals: %u", m_rule_terminals);
    ImGui::SameLine();
    ImGui::Text("| Shapes: %u", m_rule_shapes);
    ImGui::SameLine();
    ImGui::Text("| Depth: %u", m_rule_max_depth);
    ImGui::Text("Rules invoked: %u", m_rule_rules_invoked);
    ImGui::SameLine();
    ImGui::Text("| Operations: %u", m_rule_operations);

    // A cap is an Error, not a warning, and the output is truncated rather than
    // wrong-looking. Say so where the numbers are, because "it generated 50000
    // shapes" reads like success until you know it was cut off.
    if (m_rule_depth_capped) {
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f),
                           "Depth cap hit - output is truncated.");
    }
    if (m_rule_shape_capped) {
        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f),
                           "Shape cap hit - output is truncated.");
    }

    if (m_rule_seed_source == RuleSeedSource::ImportedBuildings) {
        ImGui::Text("Buildings: %u generated", m_rule_buildings_built);
        if (m_rule_buildings_skipped > 0) {
            // Said, not swallowed. A city that is missing a third of itself
            // because of a cap looks exactly like a city that finished.
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.2f, 1.0f), "| %u skipped",
                               m_rule_buildings_skipped);
            ImGui::SetItemTooltip(
                "Past the Max buildings cap, or a footprint with fewer than "
                "three usable points.");
        }
    }

    ImGui::Text("Preview: %zu vertices, %zu triangles",
                m_rule_preview_mesh.vertices.size(),
                m_rule_preview_mesh.indices.size() / 3);

    if (!m_rule_log.empty()) {
        ImGui::SeparatorText("print");
        for (const std::string& line : m_rule_log) {
            ImGui::TextUnformatted(line.c_str());
        }
    }
}

void Editor::draw_rule_panel() {
    ImGui::SetNextWindowSize(ImVec2(560.0f, 640.0f), ImGuiCond_FirstUseEver);
    if (!ImGui::Begin("Rule Editor", &m_show_rule_panel)) {
        ImGui::End();
        return;
    }

    // Seeded on first draw rather than in the constructor so that a session
    // which never opens the panel never pays for the string, and RUN once so
    // the panel demonstrates itself. Running arbitrary source on open would be
    // the wrong default -- see m_rule_auto_run -- but this particular source is
    // known, small and bounded, and a panel that opens showing an empty
    // viewport next to a rule file teaches nothing.
    if (m_rule_source.empty() && !m_rule_has_run) {
        m_rule_source = kDefaultRuleSource;
        run_rule_source();
    }

    draw_rule_source();

    ImGui::Separator();

    if (ImGui::BeginTabBar("##rule_output")) {
        if (ImGui::BeginTabItem("Diagnostics")) {
            ImGui::BeginChild("##rule_diags", ImVec2(0, 0), ImGuiChildFlags_None,
                              ImGuiWindowFlags_HorizontalScrollbar);
            draw_rule_diagnostics();
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        if (ImGui::BeginTabItem("Output")) {
            ImGui::BeginChild("##rule_stats", ImVec2(0, 0));
            draw_rule_stats();
            ImGui::EndChild();
            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    ImGui::End();
}

} // namespace stratum
