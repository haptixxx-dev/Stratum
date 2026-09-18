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
 * Not a comment saying "write a rule here". This is the file from
 * tests/procgen/test_rule_statements.cpp that produces 37 terminals and 12
 * windows, so the first thing a person sees after opening the panel and
 * pressing Generate is a facade -- which teaches the language faster than any
 * placeholder, and fails loudly if a regression breaks it.
 */
constexpr const char* kDefaultRuleSource =
    "// A shopfront with four floors of bays above it.\n"
    "// 37 terminals, 12 windows. Edit and press Generate.\n"
    "\n"
    "@start\n"
    "rule Main {\n"
    "    extrude(10.0);\n"
    "    select face { front : { Facade(); } }\n"
    "}\n"
    "\n"
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
    "rule Shopfront {}\n"
    "rule Pier {}\n"
    "rule Window {}\n";

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

    const Shape seed = shape_from_rect(static_cast<double>(m_rule_seed_width),
                                       static_cast<double>(m_rule_seed_depth));

    const GenerationResult result =
        generate(parsed.file, seed, options, &full_operations(), &full_functions());

    for (const Diagnostic& diagnostic : result.diagnostics) {
        m_rule_diagnostics.push_back(render_diagnostic(diagnostic, m_rule_source, parsed.filename));
        if (diagnostic.severity == Severity::Error) m_rule_has_error = true;
    }
    m_rule_log = result.log;

    m_rule_terminals = result.stats.terminals;
    m_rule_shapes = result.stats.shapes_created;
    m_rule_rules_invoked = result.stats.rules_invoked;
    m_rule_operations = result.stats.operations_applied;
    m_rule_max_depth = result.stats.max_depth_reached;
    m_rule_depth_capped = result.stats.depth_limit_hit;
    m_rule_shape_capped = result.stats.shape_limit_hit;
    m_rule_has_run = true;
    m_rule_dirty = false;

    // The old preview goes whatever happens next. A run that produced nothing
    // must leave an empty viewport, not the previous building -- otherwise a
    // rule that stopped generating looks like a rule that still works.
    clear_rule_preview();

    m_rule_preview_mesh = result.build_mesh();
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

    ImGui::SetNextItemWidth(120.0f);
    ImGui::DragFloat("Width (m)", &m_rule_seed_width, 0.1f, 0.5f, 500.0f, "%.2f");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(120.0f);
    ImGui::DragFloat("Depth (m)", &m_rule_seed_depth, 0.1f, 0.5f, 500.0f, "%.2f");
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
