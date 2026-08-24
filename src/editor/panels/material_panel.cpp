/**
 * @file material_panel.cpp
 * @brief The Materials panel: browse, edit, diagnose and persist the material set
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The road system has emitted geometry tagged with material slots since P0.3 and
 * metre-based tiling UVs since P2. MaterialLibrary resolves those tags to
 * textures and PBR parameters, and GPURenderer binds the result per submesh
 * range. This panel is the only place a person can SEE that happening -- what the
 * library holds, what it is quietly falling back on, and what a change looks like
 * in the viewport while they drag the slider.
 *
 * ### The three things this panel is actually for
 *
 * 1. **Editing.** Every field of MaterialDef, applied live. The viewport updates
 *    on the next frame because GPURenderer's redundant-bind cache is cleared
 *    whenever a render pass opens, so an edited material is re-pushed at least
 *    once per frame without the panel having to invalidate anything.
 *
 * 2. **Diagnosis.** MaterialLibrary::resolve() cannot fail: an unknown variant
 *    falls back to its slot default and a road that should be cobbled draws as
 *    plain asphalt. That is the correct behaviour AND it is completely invisible,
 *    which is why the fallback count is given a headline position here rather
 *    than a log line. It is the number that tells someone the geometry side
 *    renumbered its variants.
 *
 * 3. **Persistence.** Save and load the whole set as JSON, through the same
 *    native file dialog the OSM import uses.
 *
 * ### On calling resolve() from UI code
 *
 * resolve() updates ResolveStats, so a panel that resolved arbitrary keys every
 * frame would corrupt the very counter it displays. The rule this file follows:
 * the panel only ever resolves keys that MaterialLibrary::has() reports as EXACT,
 * plus -- on an explicit button press, once -- a key that is already in the
 * fallback list and therefore already counted. Nothing here can create a new
 * distinct fallback key. The recovery in draw_material_editor() -- snapping the
 * selection to keys().front() when it is not exact -- is what enforces it.
 */

#include "editor/editor.hpp"
#include "renderer/gpu_renderer.hpp"
#include "renderer/material_library.hpp"
#include "renderer/texture.hpp"

#include <imgui.h>

#include <cstdio>
#include <filesystem>

namespace stratum {

namespace {

/**
 * @brief Human name for a variant number
 *
 * Mirrors the table in the material_variant namespace. Deliberately a LOCAL copy
 * of the naming rather than something exported from material_library.hpp: those
 * numbers are documented there as an assumption about what feat/road-optimization
 * emits, not as a contract, and a panel label is the least harmful place for an
 * assumption to be wrong. A number this does not know is shown as its digits,
 * which is exactly what someone reconciling two numberings needs to see.
 */
const char* variant_name(uint16_t variant) {
    switch (variant) {
        case material_variant::kDefault:       return "default";
        case material_variant::kSmooth:        return "smooth / granite";
        case material_variant::kWorn:          return "worn";
        case material_variant::kCobblestone:   return "cobblestone";
        case material_variant::kSett:          return "sett";
        case material_variant::kPavingStones:  return "paving stones";
        case material_variant::kTactilePaving: return "tactile paving";
        default:                               return nullptr;
    }
}

/// "Asphalt / cobblestone", or "Asphalt / variant 9" for a number we do not name.
void format_key(char* out, size_t n, MaterialKey key) {
    const char* slot = material_id_name(key.material);
    if (const char* v = variant_name(key.variant)) {
        std::snprintf(out, n, "%s / %s", slot, v);
    } else {
        std::snprintf(out, n, "%s / variant %u", slot, static_cast<unsigned>(key.variant));
    }
}

/// ImGui's texture identifier for an SDL_GPU texture. The SDL_GPU backend stores
/// SDL_GPUTexture* directly in ImTextureID as of ImGui 1.92.2; before that it was
/// an SDL_GPUTextureSamplerBinding*, which is why this is spelled out once here
/// rather than at each of the three call sites.
ImTextureID to_imgui_texture(SDL_GPUTexture* texture) {
    return static_cast<ImTextureID>(reinterpret_cast<intptr_t>(texture));
}

} // namespace

// ============================================================================
// Editor::draw_material_panel
// ============================================================================

void Editor::draw_material_panel() {
    if (!ImGui::Begin("Materials", &m_show_material_panel)) {
        ImGui::End();
        return;
    }

    // init_materials() treats every failure as survivable and leaves both pointers
    // null, which makes the renderer draw exactly as it did before materials
    // existed. Say so rather than showing an empty panel, because "no materials"
    // and "materials that are all grey" look identical in the viewport.
    if (!m_material_library || !m_texture_manager) {
        ImGui::TextColored(ImVec4(0.95f, 0.55f, 0.25f, 1.0f),
                           "The material system is not running.");
        ImGui::TextWrapped(
            "Texture manager or material library initialisation failed at startup; "
            "the console has the reason. Roads are drawing untextured, which is the "
            "intended degraded state rather than an error.");
        ImGui::End();
        return;
    }

    draw_material_panel_header();
    ImGui::Separator();

    // ── Two panes ───────────────────────────────────────────────────────────
    //
    // ResizeX rather than a fixed split: slot names are short but a variant list
    // under a slot with six entries is not, and the right pane's sliders have a
    // usable minimum width of their own.
    ImGui::BeginChild("##material_slots", ImVec2(240.0f, 0.0f),
                      ImGuiChildFlags_Borders | ImGuiChildFlags_ResizeX);
    draw_material_slot_tree();
    ImGui::EndChild();

    ImGui::SameLine();

    ImGui::BeginChild("##material_editor", ImVec2(0.0f, 0.0f), ImGuiChildFlags_Borders);
    draw_material_editor();
    ImGui::EndChild();

    ImGui::End();
}

// ============================================================================
// Header: state, diagnostics, persistence
// ============================================================================

void Editor::draw_material_panel_header() {
    MaterialLibrary& lib = *m_material_library;
    GPUTextureManager& textures = *m_texture_manager;

    // ── Is any of this reaching the screen? ─────────────────────────────────
    //
    // Two switches sit between an edited material and a pixel, and both of them
    // are off in states that look like the panel is broken. They are checked here,
    // above everything else, because every other readout on this panel is
    // meaningless while either is off.
    if (m_gpu_renderer) {
        bool enabled = m_gpu_renderer->materials_enabled();
        if (ImGui::Checkbox("Materials enabled", &enabled)) {
            m_gpu_renderer->set_materials_enabled(enabled);
        }
        ImGui::SetItemTooltip(
            "Off draws every range with a neutral material -- white albedo, flat "
            "normal, unit ORM, default scalars -- so the surface is its vertex "
            "colour alone. That is the A/B a bug report needs. It cannot mean "
            "'bind nothing': the PBR pipeline declares three samplers that every "
            "draw must fill whatever this switch says.");

        // Materials are a PBR-only path: the simple shader declares no material
        // uniform block and no samplers, so bind_material() returns immediately in
        // ShaderMode::Simple. init_materials() switches to PBR at startup, so
        // seeing this means someone switched back.
        if (m_gpu_renderer->get_shader_mode() != ShaderMode::PBR) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.95f, 0.55f, 0.25f, 1.0f), "Simple shader: no materials");
            ImGui::SetItemTooltip(
                "The simple shader has no material uniform block and no samplers. "
                "Switch to PBR in Render Settings.");
            ImGui::SameLine();
            if (ImGui::SmallButton("Use PBR")) {
                m_gpu_renderer->set_shader_mode(ShaderMode::PBR);
            }
        }
    }

    // ── Fallback accounting ─────────────────────────────────────────────────
    //
    // The headline diagnostic. resolve() cannot fail, so a whole variant set going
    // missing -- because the geometry side renumbered, or because a set file
    // predates a slot -- produces no error, no warning per draw and no visual
    // breakage: every affected surface quietly draws as its slot default. This
    // number is the only thing that says so.
    const MaterialLibrary::ResolveStats& stats = lib.resolve_stats();

    ImGui::Text("%zu materials", lib.size());
    ImGui::SameLine();
    ImGui::TextDisabled("|");
    ImGui::SameLine();

    if (stats.fallback_keys > 0) {
        const uint64_t fallback_draws =
            stats.variant_fallbacks + stats.slot_fallbacks + stats.hard_fallbacks;
        ImGui::TextColored(ImVec4(0.95f, 0.55f, 0.25f, 1.0f),
                           "%llu draws resolved by fallback (%zu distinct materials missing)",
                           static_cast<unsigned long long>(fallback_draws),
                           stats.fallback_keys);
    } else {
        ImGui::TextDisabled("0 draws resolved by fallback");
    }

    if (stats.fallback_keys > 0) {
        draw_material_fallback_list();
    }

    // ── Texture set ─────────────────────────────────────────────────────────
    const GPUTextureManager::Stats tex = textures.stats();
    ImGui::TextDisabled("%zu textures, %.1f MB", tex.textures,
                        static_cast<double>(tex.bytes) / (1024.0 * 1024.0));
    if (tex.load_failures > 0) {
        ImGui::SameLine();
        ImGui::TextColored(ImVec4(0.95f, 0.55f, 0.25f, 1.0f), "%zu load failures",
                           tex.load_failures);
        ImGui::SetItemTooltip(
            "Each failure left a material with an unbound map, drawing plain white, "
            "flat-normal or unit-ORM instead. The console names the files.");
    }

    // ── Persistence ─────────────────────────────────────────────────────────
    //
    // Both go through Editor::open_file_dialog(), the single native-dialog path
    // this editor has; the result lands in poll_file_dialog() and is routed by
    // FilePickTarget. Disabled while a dialog is already up, because SDL allows
    // only one and the second call would be silently dropped.
    ImGui::BeginDisabled(m_file_pick.pending);
    if (ImGui::Button("Save set...")) {
        open_file_dialog(FilePickTarget::MaterialSetSave);
    }
    ImGui::SetItemTooltip(
        "Writes every material as JSON. Texture paths are stored RELATIVE to the "
        "set file, so a set and its textures move together. Procedurally generated "
        "maps have no path and are omitted: they are regenerated on load, not stored.");

    ImGui::SameLine();
    if (ImGui::Button("Load set...")) {
        open_file_dialog(FilePickTarget::MaterialSetLoad);
    }
    ImGui::SetItemTooltip(
        "Replaces the whole set. An entry that fails to parse is skipped and "
        "logged; the rest still loads.");
    ImGui::EndDisabled();

    ImGui::SameLine();
    if (ImGui::Button("Restore defaults")) {
        // The frozen slot table, then the variant table, then the generated
        // textures -- the same order init_materials() uses, because
        // install_procedural_textures() edits the materials load_defaults()
        // installed and does nothing useful before them.
        lib.load_defaults();
        lib.install_procedural_textures();
        lib.reset_resolve_stats();
        m_material_set_status = "restored built-in defaults";
    }
    ImGui::SetItemTooltip(
        "Reinstalls the built-in slot table, the variant table and the generated "
        "tiling textures, discarding every edit.");

    if (!m_material_set_path.empty()) {
        ImGui::TextDisabled("Set: %s", m_material_set_path.c_str());
    }
    if (!m_material_set_status.empty()) {
        ImGui::TextDisabled("%s", m_material_set_status.c_str());
    }
}

// ============================================================================
// The list of keys resolve() could not answer exactly
// ============================================================================

void Editor::draw_material_fallback_list() {
    MaterialLibrary& lib = *m_material_library;

    if (!ImGui::TreeNode("##fallbacks", "Missing materials")) {
        return;
    }

    ImGui::TextWrapped(
        "These (slot, variant) pairs were asked for and do not exist, so they drew "
        "as their slot default. That is intended -- an unknown variant of asphalt "
        "is still asphalt -- but if a whole set of them appeared at once, the "
        "geometry side's variant numbering and this library's have diverged.");

    for (const MaterialKey key : lib.fallback_keys()) {
        char label[128];
        format_key(label, sizeof(label), key);
        ImGui::BulletText("%s", label);

        ImGui::SameLine();
        ImGui::PushID(static_cast<int>(key.packed()));
        if (ImGui::SmallButton("Create")) {
            // resolve() here is safe for the accounting: this key is ALREADY in
            // the fallback set -- that is how it got into this list -- so
            // resolving it cannot add a new distinct fallback key. Seeding from
            // the fallback rather than from a blank MaterialDef means the new
            // entry starts as what the surface was already drawing, so pressing
            // Create changes nothing on screen until something is edited.
            MaterialDef seeded = lib.resolve(key);
            char name[128];
            format_key(name, sizeof(name), key);
            seeded.name = name;
            lib.set(key, std::move(seeded));

            // The counts described a library this key was missing from; it is not
            // missing any more.
            lib.reset_resolve_stats();

            m_selected_material = key;
        }
        ImGui::SetItemTooltip("Add a real entry for this key, seeded from what it "
                              "currently falls back to.");
        ImGui::PopID();
    }

    if (ImGui::SmallButton("Reset counters")) {
        lib.reset_resolve_stats();
    }
    ImGui::SetItemTooltip("Forget the accounting, including the distinct-key set. "
                          "Does not touch the materials.");

    ImGui::TreePop();
}

// ============================================================================
// Left pane: slots, with their variants under them
// ============================================================================

void Editor::draw_material_slot_tree() {
    MaterialLibrary& lib = *m_material_library;
    const std::vector<MaterialKey> keys = lib.keys();

    // keys() is in ascending MaterialKey::packed() order and packed() is
    // (slot << 16) | variant, so the list is ALREADY grouped by slot and sorted by
    // variant within a slot. The walk below relies on that rather than building a
    // per-slot index every frame.
    size_t cursor = 0;

    for (uint8_t slot_index = 0; slot_index < static_cast<uint8_t>(MaterialId::Count);
         ++slot_index) {
        const auto slot = static_cast<MaterialId>(slot_index);

        // The half-open run of keys() belonging to this slot.
        const size_t first = cursor;
        while (cursor < keys.size() && keys[cursor].material == slot) {
            ++cursor;
        }
        const size_t count = cursor - first;

        if (count == 0) {
            // A slot with NO entry at all. After load_defaults() this cannot
            // happen; after loading a partial set file it can, and it means every
            // surface in that slot is falling all the way through to
            // MaterialId::Default grey. Shown rather than skipped: a gap in the
            // list is the only place that is visible.
            ImGui::TextDisabled("%s  (no entry)", material_id_name(slot));
            ImGui::SetItemTooltip("Nothing is installed for this slot, so its "
                                  "geometry falls back to Default.");
            continue;
        }

        ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_SpanAvailWidth;
        // Open the slot the selection is in, so a selection made from the
        // fallback list is visible without hunting for it.
        if (m_selected_material.material == slot) {
            ImGui::SetNextItemOpen(true, ImGuiCond_Once);
        }

        if (ImGui::TreeNodeEx(material_id_name(slot), flags, "%s (%zu)",
                              material_id_name(slot), count)) {
            for (size_t i = first; i < cursor; ++i) {
                const MaterialKey key = keys[i];

                const char* label = variant_name(key.variant);
                char fallback_label[32];
                if (!label) {
                    std::snprintf(fallback_label, sizeof(fallback_label), "variant %u",
                                  static_cast<unsigned>(key.variant));
                    label = fallback_label;
                }

                ImGui::PushID(static_cast<int>(key.packed()));
                if (ImGui::Selectable(label, m_selected_material == key)) {
                    m_selected_material = key;
                }
                ImGui::PopID();
            }
            ImGui::TreePop();
        }
    }
}

// ============================================================================
// Right pane: the selected material
// ============================================================================

void Editor::draw_material_editor() {
    MaterialLibrary& lib = *m_material_library;
    // The selection must name an EXACT entry. Anything else would mean resolving a
    // possibly-unknown key every frame, which would invent fallback counts out of
    // UI activity -- see the file note. Recover to the first real key instead.
    if (!lib.has(m_selected_material)) {
        const std::vector<MaterialKey> keys = lib.keys();
        if (keys.empty()) {
            ImGui::TextDisabled("The library is empty. Press Restore defaults.");
            return;
        }
        m_selected_material = keys.front();
    }

    const MaterialKey key = m_selected_material;

    // A COPY, not the reference resolve() returns: set() below rehashes the map and
    // invalidates it. Nothing in this function may hold a MaterialDef& across a
    // set(), and taking a copy up front is what makes that impossible to get wrong.
    MaterialDef def = lib.resolve(key);

    char title[128];
    format_key(title, sizeof(title), key);
    ImGui::SeparatorText(title);

    bool changed = false;

    // ── Name ────────────────────────────────────────────────────────────────
    {
        char name[128];
        std::snprintf(name, sizeof(name), "%s", def.name.c_str());
        if (ImGui::InputText("Name", name, sizeof(name))) {
            def.name = name;
            changed = true;
        }
        ImGui::SetItemTooltip("Shown here and written into an exported material slot. "
                              "Purely a label; nothing resolves by name.");
    }

    // ── Preview ─────────────────────────────────────────────────────────────
    draw_material_preview(def);

    // ── PBR parameters ──────────────────────────────────────────────────────
    ImGui::SeparatorText("Surface");

    // base_color is LINEAR and is multiplied into the albedo sample, not a
    // replacement for it. Float flag so the numbers shown are the numbers stored:
    // the 0-255 display would round 0.11 to 28 and back to 0.1098, which turns
    // every glance at the frozen default table into a mismatch.
    if (ImGui::ColorEdit4("Base colour", &def.base_color.x,
                          ImGuiColorEditFlags_Float | ImGuiColorEditFlags_AlphaBar)) {
        changed = true;
    }
    ImGui::SetItemTooltip("Linear, multiplied into the albedo texture sample and the "
                          "vertex colour. Alpha only reaches the screen on a material "
                          "with alpha blending on.");

    changed |= ImGui::SliderFloat("Metallic", &def.metallic, 0.0f, 1.0f);
    ImGui::SetItemTooltip("Multiplied by the ORM blue channel, which is 1 in the built-in "
                          "unit ORM. Nothing on a road surface is metal; this is here for "
                          "parapets and railings.");

    // Floored at 0.04 rather than 0: a Cook-Torrance specular lobe at roughness 0
    // is a delta function, and the highlight aliases into single blown-out pixels
    // that crawl as the camera moves.
    changed |= ImGui::SliderFloat("Roughness", &def.roughness, 0.04f, 1.0f);
    ImGui::SetItemTooltip("Multiplied by the ORM green channel.");

    changed |= ImGui::SliderFloat("Ambient occlusion", &def.ao, 0.0f, 1.0f);
    ImGui::SetItemTooltip("Multiplied by the ORM red channel. Ambient term only, so it "
                          "does not darken direct sunlight.");

    changed |= ImGui::SliderFloat("Emissive", &def.emissive, 0.0f, 4.0f);
    ImGui::SetItemTooltip("Added after direct lighting. Zero for every road surface; "
                          "useful for lit signage.");

    // ── UV scale ────────────────────────────────────────────────────────────
    ImGui::SeparatorText("Tiling");

    const bool atlased = (key.material == MaterialId::Markings);

    ImGui::BeginDisabled(atlased);
    if (ImGui::DragFloat2("UV scale", &def.uv_scale.x, 0.01f, 0.05f, 32.0f, "%.3f")) {
        changed = true;
    }
    ImGui::EndDisabled();

    if (atlased) {
        // Not merely inadvisable -- MaterialLibrary::set() logs it as a
        // configuration error. Marking geometry carries atlas sub-rect UVs, and an
        // atlas rect cannot wrap: scaling it samples the neighbouring sprite.
        ImGui::SetItemTooltip(
            "Markings is the one atlased material. Its geometry carries atlas "
            "sub-rect UVs, which cannot be scaled -- the sample would drift into "
            "the neighbouring sprite.");
    } else {
        ImGui::SetItemTooltip(
            "A multiplier ON TOP of the metre-based UVs the geometry already "
            "carries. Leave at 1 unless a texture's real-world size does not match "
            "the tile size below.");
    }

    // The plan's frozen UV Convention: the geometry divides lateral metres and
    // arc-length metres by these before writing UVs, so texel density is uniform
    // across the network regardless of road width or segment length. Showing the
    // EFFECTIVE size is what makes uv_scale meaningful -- 8 m at scale 2 is a 4 m
    // texture, and that is the number someone matching a scan needs.
    const glm::vec2 tile = material_tile_metres(key.material);
    if (tile.x > 0.0f && tile.y > 0.0f) {
        ImGui::TextDisabled("One tile covers %.2f x %.2f m", tile.x, tile.y);
        if (def.uv_scale.x > 0.0f && def.uv_scale.y > 0.0f &&
            def.uv_scale != glm::vec2(1.0f)) {
            ImGui::TextDisabled("  effective: %.2f x %.2f m", tile.x / def.uv_scale.x,
                                tile.y / def.uv_scale.y);
        }
        if (key.material == MaterialId::Curb) {
            ImGui::TextDisabled("  curb FACES run U up the face, not laterally");
        }
    } else {
        ImGui::TextDisabled("Atlased; no tile size");
    }

    // ── Sampling and pipeline ───────────────────────────────────────────────
    ImGui::SeparatorText("Sampling");

    {
        int sampler = static_cast<int>(def.sampler);
        const char* names[] = { "RepeatAniso", "ClampLinear", "RepeatPoint" };
        if (ImGui::Combo("Sampler", &sampler, names, IM_ARRAYSIZE(names))) {
            def.sampler = static_cast<SamplerKind>(sampler);
            changed = true;
        }
        ImGui::SetItemTooltip(
            "RepeatAniso for tiling surfaces. ClampLinear for the markings atlas -- "
            "a REPEAT sampler on a sub-rect UV that drifts one texel samples the "
            "next sprite. RepeatPoint is a debug view.");
    }

    if (ImGui::Checkbox("Alpha blend", &def.alpha_blend)) {
        changed = true;
    }
    ImGui::SetItemTooltip(
        "Draws through the decal pipeline instead of the opaque one. Blending is "
        "PIPELINE state in SDL_GPU, so this switches pipelines rather than setting "
        "a flag. Paint is a coverage mask over the carriageway, so markings need it.");

    // depth_bias comes from the same decal pipeline as alpha_blend, so a bias
    // without blending does nothing at all -- set() logs exactly that. Disabling
    // the control is better than logging after the fact.
    ImGui::BeginDisabled(!def.alpha_blend);
    if (ImGui::DragFloat("Depth bias", &def.depth_bias, 0.05f, -16.0f, 16.0f, "%.2f")) {
        changed = true;
    }
    ImGui::EndDisabled();
    if (!def.alpha_blend) {
        ImGui::SetItemTooltip(
            "Depth bias and alpha blending both come from the decal pipeline and "
            "travel together, so a bias without blending has no effect.");
    } else {
        ImGui::SetItemTooltip(
            "Negative pulls towards the camera, in reverse-Z. Both the constant and "
            "the slope-scaled term carry that sign. Depth bias is PIPELINE state, so "
            "the renderer keeps one decal pipeline per distinct value quantised to "
            "1/16; past 32 distinct values further ones draw at the markings bias.");
    }

    // ── Texture maps ────────────────────────────────────────────────────────
    ImGui::SeparatorText("Maps");

    changed |= draw_material_map_row("Albedo", def, MaterialMapSlot::Albedo, key);
    changed |= draw_material_map_row("Normal", def, MaterialMapSlot::Normal, key);
    changed |= draw_material_map_row("ORM",    def, MaterialMapSlot::Orm,    key);

    // ── Apply ───────────────────────────────────────────────────────────────
    //
    // One set() for the whole frame's worth of edits. The viewport picks it up on
    // the next frame with no invalidation call needed: GPURenderer's
    // redundant-bind cache is cleared whenever a render pass opens, so an edited
    // material is re-pushed at least once per frame regardless of whether its KEY
    // changed. That is what makes dragging a slider update the scene live.
    if (changed) {
        lib.set(key, std::move(def));
    }
}

// ============================================================================
// Preview swatches
// ============================================================================

void Editor::draw_material_preview(const MaterialDef& def) {
    GPUTextureManager& textures = *m_texture_manager;

    // bind_texture() substitutes the manager's fallback for a handle that is
    // unset, invalid, or staged-but-not-yet-copied. That last case matters here as
    // much as it does on the road: a texture whose copy has not run holds
    // uninitialised device memory, so a swatch drawn from it would be noise rather
    // than a loading state.
    const struct {
        const char* label;
        SDL_GPUTexture* texture;
        float size;
    } swatches[] = {
        { "albedo", textures.bind_texture(def.albedo, textures.white()),        96.0f },
        { "normal", textures.bind_texture(def.normal, textures.flat_normal()),  48.0f },
        { "ORM",    textures.bind_texture(def.orm,    textures.default_orm()),  48.0f },
    };

    // SameLine BETWEEN the swatches rather than after each: a trailing SameLine
    // would apply to whatever the caller draws next, which is the "Surface"
    // separator, and put it beside the swatches instead of under them.
    bool first = true;
    for (const auto& s : swatches) {
        if (!s.texture) continue;   // before init() there is nothing to draw

        if (!first) ImGui::SameLine();
        first = false;

        ImGui::BeginGroup();
        ImGui::Image(ImTextureRef(to_imgui_texture(s.texture)), ImVec2(s.size, s.size));
        ImGui::TextDisabled("%s", s.label);
        ImGui::EndGroup();
    }

    ImGui::TextDisabled(
        "Swatches sample the whole texture with ImGui's own clamped linear sampler, "
        "so a tiling surface is shown as one tile rather than as it appears in the "
        "viewport.");
}

// ============================================================================
// One map row
// ============================================================================

bool Editor::draw_material_map_row(const char* label, MaterialDef& def,
                                   MaterialMapSlot map, MaterialKey key) {
    MaterialLibrary& lib = *m_material_library;
    GPUTextureManager& textures = *m_texture_manager;

    // MaterialMapSlot exists only because editor.hpp cannot name
    // MaterialLibrary::TextureMap without pulling SDL into every editor
    // translation unit. This is the one place the two are reconciled, and the
    // handle each names is picked up in the same switch so they cannot drift apart.
    TextureHandle* handle_ptr = nullptr;
    FilePickTarget target = FilePickTarget::MaterialAlbedo;
    switch (map) {
        case MaterialMapSlot::Albedo:
            handle_ptr = &def.albedo;
            target = FilePickTarget::MaterialAlbedo;
            break;
        case MaterialMapSlot::Normal:
            handle_ptr = &def.normal;
            target = FilePickTarget::MaterialNormal;
            break;
        case MaterialMapSlot::Orm:
            handle_ptr = &def.orm;
            target = FilePickTarget::MaterialOrm;
            break;
    }
    TextureHandle& handle = *handle_ptr;

    bool changed = false;
    ImGui::PushID(label);

    ImGui::Text("%s", label);
    ImGui::SameLine(90.0f);

    if (handle == kInvalidTexture) {
        // Not an error. An unbound map draws as the manager's neutral fallback --
        // white albedo, flat normal, unit ORM -- which is the correct plain result
        // and is why load_defaults() alone produces a usable, differentiated world
        // with no assets present at all.
        ImGui::TextDisabled("unbound");
    } else {
        const std::filesystem::path source = lib.texture_source_path(handle);
        if (source.empty()) {
            // No recorded path means the texture was GENERATED, and save_to_file()
            // will omit its field: generated maps are regenerated on load, never
            // stored. Saying so here makes that visible BEFORE the save rather than
            // after the reload.
            ImGui::TextDisabled("procedural (not saved)");
        } else {
            // Filename only; the full path is long enough to push the buttons off
            // the pane, and it is one hover away.
            ImGui::TextDisabled("%s", source.filename().string().c_str());
            ImGui::SetItemTooltip("%s", source.string().c_str());
        }

        if (!textures.is_ready(handle)) {
            ImGui::SameLine();
            ImGui::TextDisabled("(uploading)");
            ImGui::SetItemTooltip(
                "Staged but not yet copied. It joins the renderer's existing "
                "per-frame copy pass and becomes samplable within a frame or two.");
        }
    }

    ImGui::SameLine();

    // One dialog at a time; SDL drops a second concurrent call.
    ImGui::BeginDisabled(m_file_pick.pending);
    if (ImGui::SmallButton("Load...")) {
        // Capture the key NOW. The native dialog is asynchronous and the selection
        // can move while it is open; applying the texture to whatever happens to be
        // selected when the user finally clicks Open is a quiet way to overwrite
        // the wrong material.
        m_material_pick_key = key;
        open_file_dialog(target);
    }
    ImGui::EndDisabled();

    if (handle != kInvalidTexture) {
        ImGui::SameLine();
        if (ImGui::SmallButton("Clear")) {
            // The handle is NOT released: a texture may be shared by many
            // materials -- eight surface materials commonly share one default ORM
            // -- and releasing one here would blank the others. GPUTextureManager
            // owns them and outlives every material set.
            handle = kInvalidTexture;
            changed = true;
        }
        ImGui::SetItemTooltip("Unbind this map. The texture itself is kept: other "
                              "materials may share it.");
    }

    ImGui::PopID();
    return changed;
}

} // namespace stratum
