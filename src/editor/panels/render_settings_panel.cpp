// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "editor/editor.hpp"
#include "editor/render_settings.hpp"
#include "renderer/gpu_renderer.hpp"
#include <imgui.h>

namespace stratum {

void Editor::draw_render_settings() {
    ImGui::Begin("Render Settings", &m_show_render_settings);

    if (m_gpu_renderer) {
        // Shader Mode selection
        ImGui::Text("Shader Mode");
        int shader_mode = static_cast<int>(m_gpu_renderer->get_shader_mode());
        const char* shader_options[] = { "Simple (Fast)", "PBR (Quality)" };
        if (ImGui::Combo("##ShaderMode", &shader_mode, shader_options, 2)) {
            m_gpu_renderer->set_shader_mode(static_cast<ShaderMode>(shader_mode));
        }

        // PBR settings (only visible in PBR mode)
        if (m_gpu_renderer->get_shader_mode() == ShaderMode::PBR) {
            ImGui::Separator();

            // The global Metallic / Roughness / Ambient Occlusion sliders that used
            // to sit here have been REMOVED rather than left as decoration. They
            // drove SceneUniforms::pbr_params, and mesh_pbr.frag stopped reading it
            // the moment materials owned those three values: the compiled module
            // contains no access to that member at all, so the sliders moved no
            // pixel at any value. They are per-material controls now.
            ImGui::TextDisabled("Metallic, roughness and AO are per-material.");
            if (ImGui::SmallButton("Open Materials panel")) {
                m_show_material_panel = true;
            }

            ImGui::Separator();
            ImGui::Text("Lighting");

            float exposure = m_gpu_renderer->get_exposure();
            if (ImGui::SliderFloat("Exposure", &exposure, 0.1f, 5.0f)) {
                m_gpu_renderer->set_exposure(exposure);
            }

            // Everything below edits m_render_settings directly. It is the single
            // source of truth for the sun, sky, fog and shadow values: it was
            // already loaded (or left at its defaults) in Editor::init() and
            // already pushed to the renderer in Editor::set_renderer(), before
            // this panel ever drew a frame. A control here only has to say
            // whether IT changed; every one of them ORs into `changed` below,
            // which is what actually pushes and persists -- one dirty flag
            // standing in for the three separate sun_pushed/sky_pushed/fog_pushed
            // first-frame statics this function used to carry.
            RenderSettings& rs = m_render_settings;
            bool changed = false;

            // Sun direction (simplified - azimuth angle)
            changed |= ImGui::SliderFloat("Sun Azimuth", &rs.sun_azimuth_deg, 0.0f, 360.0f, "%.0f°");
            changed |= ImGui::SliderFloat("Sun Height", &rs.sun_height_deg, 5.0f, 90.0f, "%.0f°");
            changed |= ImGui::SliderFloat("Sun Intensity", &rs.sun_intensity, 0.0f, 10.0f);
            changed |= ImGui::SliderFloat("Ambient", &rs.ambient_intensity, 0.0f, 1.0f);
            changed |= ImGui::ColorEdit3("Sun Color", &rs.sun_tint.x);

            // ----------------------------------------------------------------
            // Shadows
            // ----------------------------------------------------------------
            ImGui::Separator();
            ImGui::Text("Shadows");

            changed |= ImGui::Checkbox("Enable Shadows", &rs.shadows_enabled);

            if (rs.shadows_enabled) {
                // Cascade count and map size are the two dials that actually cost
                // something: each cascade replays the whole visible caster list.
                changed |= ImGui::SliderInt("Cascades", &rs.shadow_cascade_count, 1,
                                            kMaxShadowCascades);
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Each cascade is another depth pass over every\n"
                                      "visible mesh. This is the main cost dial.");
                }

                const char* size_labels[] = { "1024", "2048", "4096" };
                const uint32_t sizes[] = { 1024u, 2048u, 4096u };
                int size_index = 1;
                for (int i = 0; i < 3; ++i) {
                    if (sizes[i] == rs.shadow_map_size) size_index = i;
                }
                if (ImGui::Combo("Resolution", &size_index, size_labels, 3)) {
                    rs.shadow_map_size = sizes[size_index];
                    changed = true;
                }
                ImGui::TextDisabled("Atlas: %u x %u", rs.shadow_map_size *
                                    static_cast<uint32_t>(rs.shadow_cascade_count),
                                    rs.shadow_map_size);

                changed |= ImGui::SliderFloat("Shadow Distance", &rs.shadow_max_distance,
                                              100.0f, 4000.0f, "%.0f m");
                changed |= ImGui::SliderFloat("Split Blend", &rs.shadow_split_lambda,
                                              0.0f, 1.0f);
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("0 splits the range evenly, 1 logarithmically.\n"
                                      "Higher puts more resolution near the camera.");
                }
                changed |= ImGui::SliderFloat("Strength", &rs.shadow_strength, 0.0f, 1.0f);
                changed |= ImGui::SliderFloat("PCF Radius", &rs.shadow_pcf_radius,
                                              0.0f, 3.0f, "%.2f texels");
                changed |= ImGui::SliderFloat("Normal Offset", &rs.shadow_normal_offset,
                                              0.0f, 8.0f, "%.2f texels");
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Removes self-shadowing acne. Too high and contact\n"
                                      "shadows detach from what casts them.");
                }
                changed |= ImGui::SliderFloat("Depth Bias", &rs.shadow_depth_bias_metres,
                                              0.0f, 0.5f, "%.3f m");
            }

            // ----------------------------------------------------------------
            // Sky
            // ----------------------------------------------------------------
            // These are not decoration and they are not "the background colour":
            // assets/shaders/sky_common.glsl is included by BOTH sky.frag and
            // mesh_pbr.frag, so the dome drawn behind the scene and the ambient
            // light filling it are two evaluations of one function. Changing the
            // zenith here re-lights every upward-facing surface in the map.
            //
            // The values are scene-referred radiance, not display colours, so
            // they are allowed past 1.0 -- exposure and the ACES curve are what
            // bring them to the screen.
            ImGui::Separator();
            ImGui::Text("Sky and image-based lighting");

            changed |= ImGui::ColorEdit3("Zenith", &rs.sky_zenith.x,
                                         ImGuiColorEditFlags_Float | ImGuiColorEditFlags_HDR);
            changed |= ImGui::ColorEdit3("Horizon", &rs.sky_horizon.x,
                                         ImGuiColorEditFlags_Float | ImGuiColorEditFlags_HDR);
            changed |= ImGui::ColorEdit3("Ground Bounce", &rs.ground_bounce.x,
                                         ImGuiColorEditFlags_Float | ImGuiColorEditFlags_HDR);
            changed |= ImGui::SliderFloat("Sky Intensity", &rs.sky_intensity, 0.0f, 4.0f);
            changed |= ImGui::SliderFloat("Bounce Intensity", &rs.ground_intensity, 0.0f, 2.0f);
            changed |= ImGui::SliderFloat("Horizon Falloff", &rs.sky_falloff, 0.05f, 2.0f);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Smaller keeps the bright band tight to the horizon.");
            }
            changed |= ImGui::SliderFloat("Ambient Specular", &rs.ibl_specular, 0.0f, 2.0f);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Scale on the sky's specular reflection. 1.0 is physical.\n"
                                  "This is what stops asphalt and glass reading as matte paper.");
            }
            changed |= ImGui::SliderFloat("Aerial Perspective", &rs.aerial_perspective, 0.0f, 1.0f);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("0 fades distance into the authored fog colour.\n"
                                  "1 fades it into the sky along the view ray.");
            }
            changed |= ImGui::SliderFloat("Sun Size", &rs.sun_angular_deg, 0.1f, 8.0f, "%.2f deg");
            changed |= ImGui::SliderFloat("Sun Glow", &rs.sun_glow, 2.0f, 512.0f, "%.0f",
                                          ImGuiSliderFlags_Logarithmic);

            // ----------------------------------------------------------------
            // Fog
            // ----------------------------------------------------------------
            ImGui::Separator();
            ImGui::Text("Fog");

            // Defaults match GPURenderer::update_scene_uniforms()'s seeding (see
            // RenderSettings' own field defaults). Exponential and ON by default:
            // with no distance haze the far edge of a city extract keeps full
            // contrast right up to the horizon and then simply stops, which is
            // the strongest single cue that a scene has no atmosphere. The
            // extents are kilometres because that is the size of an OSM extract.
            const char* fog_modes[] = { "Off", "Linear", "Exponential", "Exponential Squared" };
            changed |= ImGui::Combo("Fog Mode", &rs.fog_mode, fog_modes, 4);

            if (rs.fog_mode > 0) {
                changed |= ImGui::ColorEdit3("Fog Color", &rs.fog_color.x);

                if (rs.fog_mode == 1) {
                    // Linear fog - use start/end distances
                    changed |= ImGui::SliderFloat("Fog Start", &rs.fog_start, 0.0f, 500.0f, "%.0f m");
                    changed |= ImGui::SliderFloat("Fog End", &rs.fog_end, 10.0f, 2000.0f, "%.0f m");
                    if (rs.fog_start >= rs.fog_end) rs.fog_end = rs.fog_start + 10.0f;
                } else {
                    // Exponential fog modes - use density. The lower bound has to
                    // reach 1e-5: at city scale a density of 1e-4 is already
                    // thick, and the useful range for aerial perspective sits
                    // below the old 1e-4 floor.
                    changed |= ImGui::SliderFloat("Fog Density", &rs.fog_density, 0.00001f, 0.05f,
                                                  "%.5f", ImGuiSliderFlags_Logarithmic);
                }
                if (rs.aerial_perspective > 0.0f) {
                    ImGui::TextDisabled("Fog Color is blended %.0f%% towards the sky.",
                                        rs.aerial_perspective * 100.0f);
                }
            }

            if (changed) {
                rs.dirty = true;
            }
            if (rs.dirty) {
                rs.push_to(*m_gpu_renderer);
                rs.save(m_render_settings_path);
                rs.dirty = false;
            }
        }

        ImGui::Separator();

        // Wireframe mode
        bool wireframe = (m_gpu_renderer->get_fill_mode() == FillMode::Wireframe);
        if (ImGui::Checkbox("Wireframe Mode", &wireframe)) {
            m_gpu_renderer->set_fill_mode(wireframe ? FillMode::Wireframe : FillMode::Solid);
        }

        // MSAA - disabled for now (requires app restart to change)
        // TODO: Implement offscreen MSAA rendering to allow runtime changes
        ImGui::Separator();
        ImGui::BeginDisabled();
        ImGui::Text("Anti-Aliasing");
        const char* msaa_options[] = { "Off", "2x MSAA", "4x MSAA", "8x MSAA" };
        int current_msaa = m_gpu_renderer->get_msaa_level();
        ImGui::Combo("MSAA", &current_msaa, msaa_options, 4);
        ImGui::EndDisabled();
        ImGui::TextDisabled("(Requires restart)");
    }

    // Culling settings
    ImGui::Separator();
    ImGui::Text("Culling");

    ImGui::Checkbox("Frustum Culling", &m_use_tile_culling);
    ImGui::Checkbox("Distance Culling", &m_use_distance_culling);

    if (m_use_distance_culling) {
        ImGui::SetNextItemWidth(150);
        ImGui::SliderFloat("View Radius", &m_view_radius, 500.0f, 20000.0f, "%.0f m");
    }

    // Contribution culling
    ImGui::Checkbox("Contribution Culling", &m_use_contribution_culling);
    if (m_use_contribution_culling) {
        ImGui::SetNextItemWidth(150);
        ImGui::SliderFloat("Threshold (px)", &m_contribution_threshold, 1.0f, 20.0f, "%.1f");
    }

    // Stats
    if (m_quadtree.leaf_count() > 0) {
        ImGui::Separator();
        ImGui::Text("QuadTree Statistics");


        ImGui::BulletText("Leaves: %zu", m_quadtree.leaf_count());
        ImGui::BulletText("Max Depth: %d", m_quadtree.max_depth());
        ImGui::BulletText("Roads: %zu", m_quadtree.total_roads());
        ImGui::BulletText("Buildings: %zu", m_quadtree.total_buildings());
        ImGui::BulletText("Areas: %zu", m_quadtree.total_areas());
        if (m_gpu_renderer) {
            const auto stats = m_gpu_renderer->get_frame_stats();
            ImGui::BulletText("Draw calls: %u", stats.draw_calls);
            ImGui::BulletText("Triangles drawn: %u", stats.triangles);
        }
    }

    ImGui::End();
}

} // namespace stratum
