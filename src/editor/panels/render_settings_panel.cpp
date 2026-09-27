// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "editor/editor.hpp"
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

            // Sun direction (simplified - azimuth angle)
            static float sun_angle = 45.0f;
            static float sun_height = 60.0f;
            static float sun_intensity = 3.14159265f;  // ~PI: cancels the shader's albedo / PI diffuse
            // A MASTER SCALE on the sky-derived ambient, not the ambient itself.
            // 1.0 means "the sky as authored"; the old 0.3 meant "30% grey".
            static float ambient_intensity = 1.0f;
            static glm::vec3 sun_tint = glm::vec3(1.0f, 0.98f, 0.95f);

            // These controls used to publish nothing until one of them moved, and
            // what they then published did not match the renderer's own startup
            // default -- a different sun direction and an ambient of 0.1 against
            // its 0.3. Touching any slider dropped the whole scene two thirds of
            // a stop for no reason the user asked for. Pushing once on the first
            // frame makes the widget positions the truth from the start.
            static bool sun_pushed = false;
            bool sun_changed = !sun_pushed;
            sun_pushed = true;

            sun_changed |= ImGui::SliderFloat("Sun Azimuth", &sun_angle, 0.0f, 360.0f, "%.0f°");
            sun_changed |= ImGui::SliderFloat("Sun Height", &sun_height, 5.0f, 90.0f, "%.0f°");
            sun_changed |= ImGui::SliderFloat("Sun Intensity", &sun_intensity, 0.0f, 10.0f);
            sun_changed |= ImGui::SliderFloat("Ambient", &ambient_intensity, 0.0f, 1.0f);
            sun_changed |= ImGui::ColorEdit3("Sun Color", &sun_tint.x);
            if (sun_changed) {
                float az_rad = glm::radians(sun_angle);
                float h_rad = glm::radians(sun_height);
                glm::vec3 sun_dir = glm::normalize(glm::vec3(
                    cos(h_rad) * sin(az_rad),
                    sin(h_rad),
                    cos(h_rad) * cos(az_rad)
                ));
                m_gpu_renderer->set_scene_lighting(sun_dir, sun_tint, sun_intensity,
                                                   ambient_intensity);
            }

            // ----------------------------------------------------------------
            // Shadows
            // ----------------------------------------------------------------
            ImGui::Separator();
            ImGui::Text("Shadows");

            ShadowConfig shadows = m_gpu_renderer->get_shadow_config();
            bool shadows_changed = false;

            shadows_changed |= ImGui::Checkbox("Enable Shadows", &shadows.enabled);

            if (shadows.enabled) {
                // Cascade count and map size are the two dials that actually cost
                // something: each cascade replays the whole visible caster list.
                shadows_changed |= ImGui::SliderInt("Cascades", &shadows.cascade_count, 1,
                                                    kMaxShadowCascades);
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Each cascade is another depth pass over every\n"
                                      "visible mesh. This is the main cost dial.");
                }

                const char* size_labels[] = { "1024", "2048", "4096" };
                const uint32_t sizes[] = { 1024u, 2048u, 4096u };
                int size_index = 1;
                for (int i = 0; i < 3; ++i) {
                    if (sizes[i] == shadows.map_size) size_index = i;
                }
                if (ImGui::Combo("Resolution", &size_index, size_labels, 3)) {
                    shadows.map_size = sizes[size_index];
                    shadows_changed = true;
                }
                ImGui::TextDisabled("Atlas: %u x %u", shadows.map_size *
                                    static_cast<uint32_t>(shadows.cascade_count),
                                    shadows.map_size);

                shadows_changed |= ImGui::SliderFloat("Shadow Distance", &shadows.max_distance,
                                                      100.0f, 4000.0f, "%.0f m");
                shadows_changed |= ImGui::SliderFloat("Split Blend", &shadows.split_lambda,
                                                      0.0f, 1.0f);
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("0 splits the range evenly, 1 logarithmically.\n"
                                      "Higher puts more resolution near the camera.");
                }
                shadows_changed |= ImGui::SliderFloat("Strength", &shadows.strength, 0.0f, 1.0f);
                shadows_changed |= ImGui::SliderFloat("PCF Radius", &shadows.pcf_radius,
                                                      0.0f, 3.0f, "%.2f texels");
                shadows_changed |= ImGui::SliderFloat("Normal Offset", &shadows.normal_offset,
                                                      0.0f, 8.0f, "%.2f texels");
                if (ImGui::IsItemHovered()) {
                    ImGui::SetTooltip("Removes self-shadowing acne. Too high and contact\n"
                                      "shadows detach from what casts them.");
                }
                shadows_changed |= ImGui::SliderFloat("Depth Bias", &shadows.depth_bias_metres,
                                                      0.0f, 0.5f, "%.3f m");
            }

            if (shadows_changed) {
                m_gpu_renderer->set_shadow_config(shadows);
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

            static glm::vec3 sky_zenith = glm::vec3(0.16f, 0.30f, 0.62f);
            static glm::vec3 sky_horizon = glm::vec3(0.56f, 0.68f, 0.86f);
            static glm::vec3 ground_bounce = glm::vec3(0.14f, 0.14f, 0.12f);
            static float sky_intensity = 1.0f;
            static float ground_intensity = 0.7f;
            static float sky_falloff = 0.45f;
            static float ibl_specular = 1.0f;
            static float sun_angular_deg = 0.53f;
            static float aerial_perspective = 1.0f;
            static float sun_glow = 64.0f;

            // Same first-frame push as the sun above, and for the same reason:
            // the widget positions have to be the truth from frame one, not from
            // whenever somebody first drags something.
            static bool sky_pushed = false;
            bool sky_changed = !sky_pushed;
            sky_pushed = true;

            sky_changed |= ImGui::ColorEdit3("Zenith", &sky_zenith.x,
                                             ImGuiColorEditFlags_Float | ImGuiColorEditFlags_HDR);
            sky_changed |= ImGui::ColorEdit3("Horizon", &sky_horizon.x,
                                             ImGuiColorEditFlags_Float | ImGuiColorEditFlags_HDR);
            sky_changed |= ImGui::ColorEdit3("Ground Bounce", &ground_bounce.x,
                                             ImGuiColorEditFlags_Float | ImGuiColorEditFlags_HDR);
            sky_changed |= ImGui::SliderFloat("Sky Intensity", &sky_intensity, 0.0f, 4.0f);
            sky_changed |= ImGui::SliderFloat("Bounce Intensity", &ground_intensity, 0.0f, 2.0f);
            sky_changed |= ImGui::SliderFloat("Horizon Falloff", &sky_falloff, 0.05f, 2.0f);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Smaller keeps the bright band tight to the horizon.");
            }
            sky_changed |= ImGui::SliderFloat("Ambient Specular", &ibl_specular, 0.0f, 2.0f);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("Scale on the sky's specular reflection. 1.0 is physical.\n"
                                  "This is what stops asphalt and glass reading as matte paper.");
            }
            sky_changed |= ImGui::SliderFloat("Aerial Perspective", &aerial_perspective, 0.0f, 1.0f);
            if (ImGui::IsItemHovered()) {
                ImGui::SetTooltip("0 fades distance into the authored fog colour.\n"
                                  "1 fades it into the sky along the view ray.");
            }
            sky_changed |= ImGui::SliderFloat("Sun Size", &sun_angular_deg, 0.1f, 8.0f, "%.2f deg");
            sky_changed |= ImGui::SliderFloat("Sun Glow", &sun_glow, 2.0f, 512.0f, "%.0f",
                                              ImGuiSliderFlags_Logarithmic);

            if (sky_changed) {
                m_gpu_renderer->set_sky(sky_zenith, sky_horizon, ground_bounce,
                                        sky_intensity, ground_intensity, sky_falloff);
                m_gpu_renderer->set_ibl_params(ibl_specular, sun_angular_deg,
                                               aerial_perspective, sun_glow);
            }

            // ----------------------------------------------------------------
            // Fog
            // ----------------------------------------------------------------
            ImGui::Separator();
            ImGui::Text("Fog");

            // Defaults match GPURenderer::update_scene_uniforms()'s seeding, and
            // are pushed on the first frame for the same reason the sun and sky
            // are. Exponential and ON by default: with no distance haze the far
            // edge of a city extract keeps full contrast right up to the horizon
            // and then simply stops, which is the strongest single cue that a
            // scene has no atmosphere. The extents are kilometres because that is
            // the size of an OSM extract.
            static int fog_mode = 2;  // 0 = off, 1 = linear, 2 = exp, 3 = exp squared
            static float fog_start = 50.0f;
            static float fog_end = 4000.0f;
            static float fog_density = 0.00035f;
            static glm::vec3 fog_color = glm::vec3(0.62f, 0.72f, 0.85f);
            static bool fog_pushed = false;
            bool fog_changed = !fog_pushed;
            fog_pushed = true;

            const char* fog_modes[] = { "Off", "Linear", "Exponential", "Exponential Squared" };
            fog_changed |= ImGui::Combo("Fog Mode", &fog_mode, fog_modes, 4);

            if (fog_mode > 0) {
                fog_changed |= ImGui::ColorEdit3("Fog Color", &fog_color.x);

                if (fog_mode == 1) {
                    // Linear fog - use start/end distances
                    fog_changed |= ImGui::SliderFloat("Fog Start", &fog_start, 0.0f, 500.0f, "%.0f m");
                    fog_changed |= ImGui::SliderFloat("Fog End", &fog_end, 10.0f, 2000.0f, "%.0f m");
                    if (fog_start >= fog_end) fog_end = fog_start + 10.0f;
                } else {
                    // Exponential fog modes - use density. The lower bound has to
                    // reach 1e-5: at city scale a density of 1e-4 is already
                    // thick, and the useful range for aerial perspective sits
                    // below the old 1e-4 floor.
                    fog_changed |= ImGui::SliderFloat("Fog Density", &fog_density, 0.00001f, 0.05f, "%.5f", ImGuiSliderFlags_Logarithmic);
                }
                if (aerial_perspective > 0.0f) {
                    ImGui::TextDisabled("Fog Color is blended %.0f%% towards the sky.",
                                        aerial_perspective * 100.0f);
                }
            }

            if (fog_changed) {
                m_gpu_renderer->set_fog(fog_mode, fog_color, fog_start, fog_end, fog_density);
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
