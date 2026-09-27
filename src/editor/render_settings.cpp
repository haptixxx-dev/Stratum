// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "editor/render_settings.hpp"

#include "renderer/gpu_renderer.hpp"

#include <nlohmann/json.hpp>
#include <spdlog/spdlog.h>

#include <cmath>
#include <fstream>
#include <system_error>

namespace stratum {

namespace {

// Non-throwing readers, matching the convention in renderer/material_library.cpp:
// a missing or wrong-typed field leaves the destination untouched instead of
// throwing, so one malformed field never fails the whole load.

void write_vec3(nlohmann::json& j, const char* key, const glm::vec3& v) {
    j[key] = { v.x, v.y, v.z };
}

void read_vec3(const nlohmann::json& j, const char* key, glm::vec3& out) {
    const auto it = j.find(key);
    if (it == j.end() || !it->is_array() || it->size() != 3) {
        return;
    }
    for (const auto& e : *it) {
        if (!e.is_number()) return;
    }
    out = glm::vec3((*it)[0].get<float>(), (*it)[1].get<float>(), (*it)[2].get<float>());
}

void read_float(const nlohmann::json& j, const char* key, float& out) {
    const auto it = j.find(key);
    if (it != j.end() && it->is_number()) {
        out = it->get<float>();
    }
}

void read_int(const nlohmann::json& j, const char* key, int& out) {
    const auto it = j.find(key);
    if (it != j.end() && it->is_number_integer()) {
        out = it->get<int>();
    }
}

void read_uint32(const nlohmann::json& j, const char* key, uint32_t& out) {
    const auto it = j.find(key);
    if (it != j.end() && it->is_number_unsigned()) {
        out = it->get<uint32_t>();
    }
}

void read_bool(const nlohmann::json& j, const char* key, bool& out) {
    const auto it = j.find(key);
    if (it != j.end() && it->is_boolean()) {
        out = it->get<bool>();
    }
}

} // namespace

void to_json(nlohmann::json& j, const RenderSettings& s) {
    j = nlohmann::json::object();

    j["sun_azimuth_deg"] = s.sun_azimuth_deg;
    j["sun_height_deg"] = s.sun_height_deg;
    j["sun_intensity"] = s.sun_intensity;
    j["ambient_intensity"] = s.ambient_intensity;
    write_vec3(j, "sun_tint", s.sun_tint);

    write_vec3(j, "sky_zenith", s.sky_zenith);
    write_vec3(j, "sky_horizon", s.sky_horizon);
    write_vec3(j, "ground_bounce", s.ground_bounce);
    j["sky_intensity"] = s.sky_intensity;
    j["ground_intensity"] = s.ground_intensity;
    j["sky_falloff"] = s.sky_falloff;
    j["ibl_specular"] = s.ibl_specular;
    j["sun_angular_deg"] = s.sun_angular_deg;
    j["aerial_perspective"] = s.aerial_perspective;
    j["sun_glow"] = s.sun_glow;

    j["fog_mode"] = s.fog_mode;
    j["fog_start"] = s.fog_start;
    j["fog_end"] = s.fog_end;
    j["fog_density"] = s.fog_density;
    write_vec3(j, "fog_color", s.fog_color);

    j["shadows_enabled"] = s.shadows_enabled;
    j["shadow_cascade_count"] = s.shadow_cascade_count;
    j["shadow_map_size"] = s.shadow_map_size;
    j["shadow_max_distance"] = s.shadow_max_distance;
    j["shadow_split_lambda"] = s.shadow_split_lambda;
    j["shadow_normal_offset"] = s.shadow_normal_offset;
    j["shadow_depth_bias_metres"] = s.shadow_depth_bias_metres;
    j["shadow_strength"] = s.shadow_strength;
    j["shadow_pcf_radius"] = s.shadow_pcf_radius;
}

void from_json(const nlohmann::json& j, RenderSettings& s) {
    if (!j.is_object()) {
        return;
    }

    read_float(j, "sun_azimuth_deg", s.sun_azimuth_deg);
    read_float(j, "sun_height_deg", s.sun_height_deg);
    read_float(j, "sun_intensity", s.sun_intensity);
    read_float(j, "ambient_intensity", s.ambient_intensity);
    read_vec3(j, "sun_tint", s.sun_tint);

    read_vec3(j, "sky_zenith", s.sky_zenith);
    read_vec3(j, "sky_horizon", s.sky_horizon);
    read_vec3(j, "ground_bounce", s.ground_bounce);
    read_float(j, "sky_intensity", s.sky_intensity);
    read_float(j, "ground_intensity", s.ground_intensity);
    read_float(j, "sky_falloff", s.sky_falloff);
    read_float(j, "ibl_specular", s.ibl_specular);
    read_float(j, "sun_angular_deg", s.sun_angular_deg);
    read_float(j, "aerial_perspective", s.aerial_perspective);
    read_float(j, "sun_glow", s.sun_glow);

    read_int(j, "fog_mode", s.fog_mode);
    read_float(j, "fog_start", s.fog_start);
    read_float(j, "fog_end", s.fog_end);
    read_float(j, "fog_density", s.fog_density);
    read_vec3(j, "fog_color", s.fog_color);

    read_bool(j, "shadows_enabled", s.shadows_enabled);
    read_int(j, "shadow_cascade_count", s.shadow_cascade_count);
    read_uint32(j, "shadow_map_size", s.shadow_map_size);
    read_float(j, "shadow_max_distance", s.shadow_max_distance);
    read_float(j, "shadow_split_lambda", s.shadow_split_lambda);
    read_float(j, "shadow_normal_offset", s.shadow_normal_offset);
    read_float(j, "shadow_depth_bias_metres", s.shadow_depth_bias_metres);
    read_float(j, "shadow_strength", s.shadow_strength);
    read_float(j, "shadow_pcf_radius", s.shadow_pcf_radius);
}

void RenderSettings::push_to(GPURenderer& renderer) const {
    const float az_rad = glm::radians(sun_azimuth_deg);
    const float h_rad = glm::radians(sun_height_deg);
    const glm::vec3 sun_dir = glm::normalize(glm::vec3(
        std::cos(h_rad) * std::sin(az_rad),
        std::sin(h_rad),
        std::cos(h_rad) * std::cos(az_rad)));
    renderer.set_scene_lighting(sun_dir, sun_tint, sun_intensity, ambient_intensity);

    renderer.set_sky(sky_zenith, sky_horizon, ground_bounce, sky_intensity, ground_intensity,
                     sky_falloff);
    renderer.set_ibl_params(ibl_specular, sun_angular_deg, aerial_perspective, sun_glow);

    renderer.set_fog(fog_mode, fog_color, fog_start, fog_end, fog_density);

    ShadowConfig shadows;
    shadows.enabled = shadows_enabled;
    shadows.cascade_count = shadow_cascade_count;
    shadows.map_size = shadow_map_size;
    shadows.max_distance = shadow_max_distance;
    shadows.split_lambda = shadow_split_lambda;
    shadows.normal_offset = shadow_normal_offset;
    shadows.depth_bias_metres = shadow_depth_bias_metres;
    shadows.strength = shadow_strength;
    shadows.pcf_radius = shadow_pcf_radius;
    renderer.set_shadow_config(shadows);

    // Without this, the very next render pass's update_scene_uniforms() would
    // find its own one-time seed still unset and silently overwrite everything
    // just pushed above with ITS hardcoded defaults -- see the doc comment on
    // mark_scene_lighting_initialized() in gpu_renderer.hpp.
    renderer.mark_scene_lighting_initialized();
}

bool RenderSettings::load(const std::filesystem::path& path) {
    std::error_code ec;
    if (path.empty() || !std::filesystem::exists(path, ec) || ec) {
        return false;
    }

    std::ifstream in(path);
    if (!in.is_open()) {
        spdlog::warn("RenderSettings: cannot open '{}'; keeping current settings",
                     path.string());
        return false;
    }

    nlohmann::json doc;
    try {
        in >> doc;
    } catch (const std::exception& e) {
        spdlog::warn("RenderSettings: '{}' is not valid JSON ({}); keeping current settings",
                     path.string(), e.what());
        return false;
    }

    if (!doc.is_object()) {
        spdlog::warn("RenderSettings: '{}' is not a JSON object; keeping current settings",
                     path.string());
        return false;
    }

    from_json(doc, *this);
    return true;
}

bool RenderSettings::save(const std::filesystem::path& path) const {
    if (path.empty()) {
        return false;
    }

    std::error_code ec;
    const std::filesystem::path base = path.parent_path();
    if (!base.empty()) {
        std::filesystem::create_directories(base, ec);
    }

    std::ofstream out(path);
    if (!out.is_open()) {
        spdlog::error("RenderSettings: cannot write '{}'", path.string());
        return false;
    }

    nlohmann::json doc;
    to_json(doc, *this);
    out << doc.dump(2) << '\n';
    if (!out.good()) {
        spdlog::error("RenderSettings: failed while writing '{}'", path.string());
        return false;
    }
    return true;
}

} // namespace stratum
