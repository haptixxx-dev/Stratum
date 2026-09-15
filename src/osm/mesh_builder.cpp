// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "osm/mesh_builder.hpp"
#include <glm/gtc/constants.hpp>
#include <mapbox/earcut.hpp>
#include <unordered_map>
#include <algorithm>
#include <cctype>
#include <limits>
#include <cmath>

// Earcut adapter for glm::dvec2
namespace mapbox {
namespace util {

template <>
struct nth<0, glm::dvec2> {
    inline static double get(const glm::dvec2& t) { return t.x; }
};

template <>
struct nth<1, glm::dvec2> {
    inline static double get(const glm::dvec2& t) { return t.y; }
};

} // namespace util
} // namespace mapbox

namespace stratum::osm {

// Helper: compute 2D centroid of polygon
static glm::dvec2 compute_centroid(const std::vector<glm::dvec2>& polygon) {
    glm::dvec2 centroid(0.0);
    for (const auto& pt : polygon) {
        centroid += pt;
    }
    return centroid / static_cast<double>(polygon.size());
}

// Helper: compute oriented bounding box and principal axis for gabled roofs
static void compute_principal_axis(const std::vector<glm::dvec2>& polygon,
                                   glm::dvec2& axis, glm::dvec2& center,
                                   double& length, double& width) {
    // Find the longest edge to determine ridge direction
    double max_edge_len = 0.0;
    glm::dvec2 longest_edge(1.0, 0.0);

    for (size_t i = 0; i < polygon.size(); ++i) {
        size_t next = (i + 1) % polygon.size();
        glm::dvec2 edge = polygon[next] - polygon[i];
        double len = glm::length(edge);
        if (len > max_edge_len) {
            max_edge_len = len;
            longest_edge = glm::normalize(edge);
        }
    }

    axis = longest_edge;
    center = compute_centroid(polygon);

    // Compute extent along axis and perpendicular
    glm::dvec2 perp(-axis.y, axis.x);
    double min_along = std::numeric_limits<double>::max();
    double max_along = std::numeric_limits<double>::lowest();
    double min_perp = std::numeric_limits<double>::max();
    double max_perp = std::numeric_limits<double>::lowest();

    for (const auto& pt : polygon) {
        glm::dvec2 rel = pt - center;
        double along = glm::dot(rel, axis);
        double across = glm::dot(rel, perp);
        min_along = std::min(min_along, along);
        max_along = std::max(max_along, along);
        min_perp = std::min(min_perp, across);
        max_perp = std::max(max_perp, across);
    }

    length = max_along - min_along;
    width = max_perp - min_perp;
}

// Parse color from OSM tag (hex "#RRGGBB" or named colors)
static glm::vec4 parse_color(const std::string& color_str, const glm::vec4& fallback) {
    if (color_str.empty()) return fallback;

    // Named colors commonly used in OSM
    static const std::unordered_map<std::string, glm::vec4> named_colors = {
        {"red",         {0.8f, 0.2f, 0.2f, 1.0f}},
        {"green",       {0.2f, 0.6f, 0.2f, 1.0f}},
        {"blue",        {0.2f, 0.4f, 0.8f, 1.0f}},
        {"yellow",      {0.9f, 0.85f, 0.2f, 1.0f}},
        {"orange",      {0.9f, 0.5f, 0.1f, 1.0f}},
        {"brown",       {0.55f, 0.35f, 0.2f, 1.0f}},
        {"white",       {0.95f, 0.95f, 0.95f, 1.0f}},
        {"black",       {0.1f, 0.1f, 0.1f, 1.0f}},
        {"grey",        {0.5f, 0.5f, 0.5f, 1.0f}},
        {"gray",        {0.5f, 0.5f, 0.5f, 1.0f}},
        {"beige",       {0.9f, 0.85f, 0.7f, 1.0f}},
        {"cream",       {1.0f, 0.95f, 0.8f, 1.0f}},
        {"tan",         {0.82f, 0.7f, 0.55f, 1.0f}},
        {"pink",        {1.0f, 0.7f, 0.75f, 1.0f}},
        {"maroon",      {0.5f, 0.15f, 0.15f, 1.0f}},
        {"terracotta",  {0.8f, 0.45f, 0.3f, 1.0f}},
        {"sandstone",   {0.85f, 0.75f, 0.6f, 1.0f}},
        {"brick",       {0.7f, 0.35f, 0.25f, 1.0f}},
        {"slate",       {0.4f, 0.45f, 0.5f, 1.0f}},
        {"copper",      {0.5f, 0.7f, 0.6f, 1.0f}},
        {"silver",      {0.75f, 0.75f, 0.8f, 1.0f}},
        {"gold",        {0.85f, 0.7f, 0.3f, 1.0f}},
    };

    // Convert to lowercase for matching
    std::string lower = color_str;
    std::transform(lower.begin(), lower.end(), lower.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    // Check named colors
    auto it = named_colors.find(lower);
    if (it != named_colors.end()) {
        return it->second;
    }

    // Try to parse hex color (#RGB, #RRGGBB, or without #)
    std::string hex = color_str;
    if (!hex.empty() && hex[0] == '#') {
        hex = hex.substr(1);
    }

    if (hex.length() == 3) {
        // #RGB -> #RRGGBB
        hex = std::string() + hex[0] + hex[0] + hex[1] + hex[1] + hex[2] + hex[2];
    }

    if (hex.length() == 6) {
        try {
            unsigned int r = std::stoul(hex.substr(0, 2), nullptr, 16);
            unsigned int g = std::stoul(hex.substr(2, 2), nullptr, 16);
            unsigned int b = std::stoul(hex.substr(4, 2), nullptr, 16);
            return glm::vec4(r / 255.0f, g / 255.0f, b / 255.0f, 1.0f);
        } catch (...) {
            // Parse failed, use fallback
        }
    }

    return fallback;
}

// ============================================================================
// Roof generation
// ============================================================================
//
// Five roof families, all driven by RoofType, which the parser fills from
// roof:shape=* (see OSMParser::classify_roof). Each emits only the roof; the
// walls are already extruded to `height` by the caller.
//
// Sloped roofs take their normals from the emitted triangle rather than from an
// analytic plane. A skillion's plane normal is easy; a dome ring's is not, and
// one rule for all of them is less to get wrong.

namespace {

/// Roof rise as a fraction of the span it covers.
constexpr float kRoofPitchRatio = 0.3f;

/// Latitude bands in a dome. Six is enough to read as curved at city scale.
constexpr int kDomeRings = 6;

/// 2D local metres to Y-up world space. The Z flip is the whole convention.
inline glm::vec3 to_world(const glm::dvec2& p, float y) {
    return {static_cast<float>(p.x), y, static_cast<float>(-p.y)};
}

/// Vertex count of a ring, ignoring an explicit closing duplicate.
///
/// OSM ways are closed by repeating the first node. Walking `size()` edges then
/// emits one degenerate edge, so every loop here walks this instead.
size_t ring_span(const std::vector<glm::dvec2>& ring) {
    if (ring.size() > 2 && ring.front() == ring.back()) {
        return ring.size() - 1;
    }
    return ring.size();
}

/// Emit one triangle with a geometric normal, forced to face upward.
void emit_tri(Mesh& mesh, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
              const glm::vec4& colour) {
    glm::vec3 n = glm::cross(b - a, c - a);
    const float len = glm::length(n);
    if (len < 1e-9f) {
        return;  // degenerate; contributes no surface
    }
    n /= len;
    if (n.y < 0.0f) {
        n = -n;
    }

    const auto base = static_cast<uint32_t>(mesh.vertices.size());
    mesh.vertices.push_back({a, n, glm::vec2(0.0f, 0.0f), colour});
    mesh.vertices.push_back({b, n, glm::vec2(1.0f, 0.0f), colour});
    mesh.vertices.push_back({c, n, glm::vec2(0.5f, 1.0f), colour});
    mesh.indices.push_back(base + 0);
    mesh.indices.push_back(base + 1);
    mesh.indices.push_back(base + 2);
}

/// Emit a quad as two triangles. Collapses to one when an edge is degenerate,
/// which is what a gable end or a hip corner reduces to.
void emit_quad(Mesh& mesh, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c,
               const glm::vec3& d, const glm::vec4& colour) {
    emit_tri(mesh, a, b, c, colour);
    emit_tri(mesh, a, c, d, colour);
}

/// Flat roof: the footprint triangulated in place, holes preserved.
void emit_flat_roof(Mesh& mesh, const std::vector<glm::dvec2>& footprint,
                    const std::vector<std::vector<glm::dvec2>>& holes, float height,
                    const glm::vec4& colour) {
    std::vector<std::vector<glm::dvec2>> polygon;
    polygon.push_back(footprint);
    for (const auto& hole : holes) {
        polygon.push_back(hole);
    }

    const std::vector<uint32_t> tri = mapbox::earcut<uint32_t>(polygon);
    const glm::vec3 up(0.0f, 1.0f, 0.0f);
    const auto base = static_cast<uint32_t>(mesh.vertices.size());

    for (const auto& ring : polygon) {
        for (const auto& pt : ring) {
            mesh.vertices.push_back({to_world(pt, height), up, glm::vec2(0.0f, 0.0f), colour});
        }
    }
    for (size_t i = 0; i + 2 < tri.size(); i += 3) {
        mesh.indices.push_back(base + tri[i]);
        mesh.indices.push_back(base + tri[i + 1]);
        mesh.indices.push_back(base + tri[i + 2]);
    }
}

/// Gabled and hipped roofs: every footprint edge rises to a ridge segment.
///
/// The two shapes differ only in ridge length. A gable's ridge spans the full
/// footprint, so the end edges project onto a single ridge endpoint and close
/// as a vertical triangle -- the gable end. A hip's ridge is inset by half the
/// building's width at each end, so those same triangles tilt and become hips.
///
/// Projecting each edge endpoint onto the ridge independently is what makes one
/// routine cover both. The previous implementation connected every edge to the
/// whole ridge, which gave the end edges of a gabled roof a quad spanning the
/// entire ridge line -- two large overlapping faces on every gabled building.
void emit_ridge_roof(Mesh& mesh, const std::vector<glm::dvec2>& footprint, float height,
                     const glm::vec4& colour, bool hipped) {
    glm::dvec2 axis, centre;
    double length = 0.0, width = 0.0;
    compute_principal_axis(footprint, axis, centre, length, width);

    if (length < 1e-6 || width < 1e-6) {
        emit_flat_roof(mesh, footprint, {}, height, colour);
        return;
    }

    // A hip cannot eat more ridge than there is; a square plan hips down to a
    // single point, which is a pyramid, and that is the correct answer.
    const double half = hipped ? std::max(0.0, (length - width) * 0.5) : length * 0.5;
    const glm::dvec2 ridge_a = centre - axis * half;
    const glm::dvec2 ridge_b = centre + axis * half;
    const auto ridge_y = static_cast<float>(height + width * 0.5 * kRoofPitchRatio);

    const double ridge_len = half * 2.0;
    auto project = [&](const glm::dvec2& pt) {
        if (ridge_len < 1e-9) {
            return ridge_a;
        }
        const double t = glm::clamp(glm::dot(pt - ridge_a, axis) / ridge_len, 0.0, 1.0);
        return ridge_a + axis * (ridge_len * t);
    };

    const size_t n = ring_span(footprint);
    for (size_t i = 0; i < n; ++i) {
        const glm::dvec2& a = footprint[i];
        const glm::dvec2& b = footprint[(i + 1) % n];
        emit_quad(mesh, to_world(a, height), to_world(b, height), to_world(project(b), ridge_y),
                  to_world(project(a), ridge_y), colour);
    }
}

/// Pyramidal roof: every edge rises to one apex over the centroid.
///
/// The apex height follows the inradius, not the bounding box, so a long thin
/// building gets a shallow pyramid instead of a spike.
void emit_apex_roof(Mesh& mesh, const std::vector<glm::dvec2>& footprint, float height,
                    const glm::vec4& colour) {
    const glm::dvec2 centre = compute_centroid(footprint);
    const size_t n = ring_span(footprint);

    double inradius = std::numeric_limits<double>::max();
    for (size_t i = 0; i < n; ++i) {
        const glm::dvec2 edge = footprint[(i + 1) % n] - footprint[i];
        const double edge_len2 = glm::dot(edge, edge);
        if (edge_len2 < 1e-9) {
            continue;
        }
        const double t = glm::clamp(glm::dot(centre - footprint[i], edge) / edge_len2, 0.0, 1.0);
        inradius = std::min(inradius, glm::length(centre - (footprint[i] + edge * t)));
    }
    if (inradius == std::numeric_limits<double>::max()) {
        emit_flat_roof(mesh, footprint, {}, height, colour);
        return;
    }

    const glm::vec3 apex = to_world(centre, static_cast<float>(height + inradius * kRoofPitchRatio));
    for (size_t i = 0; i < n; ++i) {
        emit_tri(mesh, to_world(footprint[i], height), to_world(footprint[(i + 1) % n], height),
                 apex, colour);
    }
}

/// Skillion roof: one flat plane tilted across the short axis.
///
/// The footprint is triangulated once and each vertex lifted by its position
/// along the slope, which keeps holes working. The wall tops stay level, so the
/// wedge between the level eave and the tilted plane is filled with a vertical
/// strip -- without it the building is open along three sides.
void emit_skillion_roof(Mesh& mesh, const std::vector<glm::dvec2>& footprint,
                        const std::vector<std::vector<glm::dvec2>>& holes, float height,
                        const glm::vec4& colour) {
    glm::dvec2 axis, centre;
    double length = 0.0, width = 0.0;
    compute_principal_axis(footprint, axis, centre, length, width);

    if (width < 1e-6) {
        emit_flat_roof(mesh, footprint, holes, height, colour);
        return;
    }

    const glm::dvec2 slope(-axis.y, axis.x);
    double low = std::numeric_limits<double>::max();
    for (const auto& pt : footprint) {
        low = std::min(low, glm::dot(pt - centre, slope));
    }
    const auto rise = static_cast<float>(width * kRoofPitchRatio);

    // Height of the tilted plane above the wall top at a given plan position.
    auto lift = [&](const glm::dvec2& pt) {
        const double t = glm::clamp((glm::dot(pt - centre, slope) - low) / width, 0.0, 1.0);
        return height + rise * static_cast<float>(t);
    };

    std::vector<std::vector<glm::dvec2>> polygon;
    polygon.push_back(footprint);
    for (const auto& hole : holes) {
        polygon.push_back(hole);
    }

    const std::vector<uint32_t> tri = mapbox::earcut<uint32_t>(polygon);
    std::vector<glm::dvec2> flat;
    for (const auto& ring : polygon) {
        flat.insert(flat.end(), ring.begin(), ring.end());
    }
    for (size_t i = 0; i + 2 < tri.size(); i += 3) {
        emit_tri(mesh, to_world(flat[tri[i]], lift(flat[tri[i]])),
                 to_world(flat[tri[i + 1]], lift(flat[tri[i + 1]])),
                 to_world(flat[tri[i + 2]], lift(flat[tri[i + 2]])), colour);
    }

    // Vertical infill between the level wall top and the tilted eave. Degenerate
    // along the low edge, where the plane already meets the wall.
    const size_t n = ring_span(footprint);
    for (size_t i = 0; i < n; ++i) {
        const glm::dvec2& a = footprint[i];
        const glm::dvec2& b = footprint[(i + 1) % n];
        emit_quad(mesh, to_world(a, height), to_world(b, height), to_world(b, lift(b)),
                  to_world(a, lift(a)), colour);
    }
}

/// Dome: the footprint shrunk toward its centroid over a quarter-sine profile.
///
/// A true dome over an arbitrary polygon is not well defined, so each latitude
/// band is the footprint itself scaled about the centroid. A circular footprint
/// gives a real hemisphere; anything else gives a plausible swept version of
/// its own outline, which is what a mapper tagging roof:shape=dome means.
void emit_dome_roof(Mesh& mesh, const std::vector<glm::dvec2>& footprint, float height,
                    const glm::vec4& colour) {
    const glm::dvec2 centre = compute_centroid(footprint);
    const size_t n = ring_span(footprint);
    if (n < 3) {
        emit_flat_roof(mesh, footprint, {}, height, colour);
        return;
    }

    double inradius = std::numeric_limits<double>::max();
    for (size_t i = 0; i < n; ++i) {
        const glm::dvec2 edge = footprint[(i + 1) % n] - footprint[i];
        const double edge_len2 = glm::dot(edge, edge);
        if (edge_len2 < 1e-9) {
            continue;
        }
        const double t = glm::clamp(glm::dot(centre - footprint[i], edge) / edge_len2, 0.0, 1.0);
        inradius = std::min(inradius, glm::length(centre - (footprint[i] + edge * t)));
    }
    if (inradius == std::numeric_limits<double>::max()) {
        emit_flat_roof(mesh, footprint, {}, height, colour);
        return;
    }

    const auto dome_height = static_cast<float>(inradius);
    const auto ring_at = [&](int k, size_t i) {
        const double s = static_cast<double>(k) / kDomeRings;
        const double scale = std::cos(s * glm::pi<double>() * 0.5);
        const auto y = height + dome_height * static_cast<float>(std::sin(s * glm::pi<double>() * 0.5));
        return to_world(centre + (footprint[i] - centre) * scale, y);
    };

    for (int k = 0; k < kDomeRings; ++k) {
        for (size_t i = 0; i < n; ++i) {
            const size_t j = (i + 1) % n;
            emit_quad(mesh, ring_at(k, i), ring_at(k, j), ring_at(k + 1, j), ring_at(k + 1, i),
                      colour);
        }
    }
}

}  // namespace

Mesh MeshBuilder::build_building_mesh(const Building& building) {
    Mesh mesh;

    if (building.footprint.size() < 3) {
        return mesh;
    }

    const float height = building.height;

    // Default colors based on building type
    glm::vec4 default_wall_color;
    glm::vec4 default_roof_color;

    switch (building.type) {
        case BuildingType::Commercial:
        case BuildingType::Office:
            default_wall_color = glm::vec4(0.6f, 0.7f, 0.8f, 1.0f); // Blue-gray
            default_roof_color = glm::vec4(0.3f, 0.35f, 0.4f, 1.0f);
            break;
        case BuildingType::Industrial:
        case BuildingType::Warehouse:
            default_wall_color = glm::vec4(0.5f, 0.5f, 0.5f, 1.0f); // Gray
            default_roof_color = glm::vec4(0.35f, 0.35f, 0.35f, 1.0f);
            break;
        case BuildingType::Residential:
        case BuildingType::House:
        case BuildingType::Detached:
            default_wall_color = glm::vec4(0.85f, 0.75f, 0.65f, 1.0f); // Tan/beige
            default_roof_color = glm::vec4(0.55f, 0.35f, 0.25f, 1.0f); // Brown roof
            break;
        case BuildingType::Apartments:
            default_wall_color = glm::vec4(0.8f, 0.75f, 0.7f, 1.0f); // Light tan
            default_roof_color = glm::vec4(0.4f, 0.4f, 0.45f, 1.0f);
            break;
        case BuildingType::Church:
            default_wall_color = glm::vec4(0.9f, 0.88f, 0.85f, 1.0f); // Off-white
            default_roof_color = glm::vec4(0.3f, 0.3f, 0.35f, 1.0f); // Dark slate
            break;
        case BuildingType::School:
        case BuildingType::Hospital:
            default_wall_color = glm::vec4(0.85f, 0.8f, 0.75f, 1.0f); // Cream
            default_roof_color = glm::vec4(0.5f, 0.3f, 0.25f, 1.0f);
            break;
        case BuildingType::Retail:
            default_wall_color = glm::vec4(0.75f, 0.7f, 0.65f, 1.0f);
            default_roof_color = glm::vec4(0.4f, 0.4f, 0.4f, 1.0f);
            break;
        case BuildingType::Garage:
        case BuildingType::Shed:
            default_wall_color = glm::vec4(0.6f, 0.55f, 0.5f, 1.0f);
            default_roof_color = glm::vec4(0.45f, 0.4f, 0.35f, 1.0f);
            break;
        default:
            default_wall_color = glm::vec4(0.7f, 0.7f, 0.7f, 1.0f); // Default gray
            default_roof_color = glm::vec4(0.4f, 0.4f, 0.45f, 1.0f);
            break;
    }

    // Use OSM tag colors if available, otherwise use defaults
    glm::vec4 wall_color = building.building_color.has_value()
        ? parse_color(building.building_color.value(), default_wall_color)
        : default_wall_color;

    glm::vec4 roof_color = building.roof_color.has_value()
        ? parse_color(building.roof_color.value(), default_roof_color)
        : default_roof_color;

    // === Generate walls ===
    // Each wall segment is a quad (2 triangles)
    size_t n = building.footprint.size();

    for (size_t i = 0; i < n; ++i) {
        size_t next = (i + 1) % n;

        // Skip if this would close the polygon with duplicate point
        if (i == n - 1 && building.footprint[0] == building.footprint[n-1]) {
            continue;
        }

        glm::vec3 p0(static_cast<float>(building.footprint[i].x), 0.0f, static_cast<float>(-building.footprint[i].y));
        glm::vec3 p1(static_cast<float>(building.footprint[next].x), 0.0f, static_cast<float>(-building.footprint[next].y));
        glm::vec3 p2 = p1 + glm::vec3(0.0f, height, 0.0f);
        glm::vec3 p3 = p0 + glm::vec3(0.0f, height, 0.0f);

        // Calculate wall normal (facing outward)
        glm::vec3 edge = p1 - p0;
        glm::vec3 up(0.0f, 1.0f, 0.0f);
        glm::vec3 normal = glm::normalize(glm::cross(up, edge));

        uint32_t base_idx = static_cast<uint32_t>(mesh.vertices.size());

        // Add 4 vertices for this wall quad
        mesh.vertices.push_back({p0, normal, glm::vec2(0.0f, 0.0f), wall_color});
        mesh.vertices.push_back({p1, normal, glm::vec2(1.0f, 0.0f), wall_color});
        mesh.vertices.push_back({p2, normal, glm::vec2(1.0f, 1.0f), wall_color});
        mesh.vertices.push_back({p3, normal, glm::vec2(0.0f, 1.0f), wall_color});

        // Two triangles for the quad (CCW winding)
        mesh.indices.push_back(base_idx + 0);
        mesh.indices.push_back(base_idx + 1);
        mesh.indices.push_back(base_idx + 2);

        mesh.indices.push_back(base_idx + 0);
        mesh.indices.push_back(base_idx + 2);
        mesh.indices.push_back(base_idx + 3);
    }

    // === Generate roof based on roof type ===
    //
    // Gabled, hipped, pyramidal and dome need a simple closed outline to sweep,
    // so a footprint with courtyards falls back to flat. Skillion does not: it
    // lifts a triangulation, and earcut already handles the holes.
    const bool simple_outline = building.holes.empty();

    switch (building.roof_type) {
        case RoofType::Gabled:
            if (simple_outline) {
                emit_ridge_roof(mesh, building.footprint, height, roof_color, /*hipped=*/false);
            } else {
                emit_flat_roof(mesh, building.footprint, building.holes, height, roof_color);
            }
            break;

        case RoofType::Hipped:
            if (simple_outline) {
                emit_ridge_roof(mesh, building.footprint, height, roof_color, /*hipped=*/true);
            } else {
                emit_flat_roof(mesh, building.footprint, building.holes, height, roof_color);
            }
            break;

        case RoofType::Pyramidal:
            if (simple_outline) {
                emit_apex_roof(mesh, building.footprint, height, roof_color);
            } else {
                emit_flat_roof(mesh, building.footprint, building.holes, height, roof_color);
            }
            break;

        case RoofType::Skillion:
            emit_skillion_roof(mesh, building.footprint, building.holes, height, roof_color);
            break;

        case RoofType::Dome:
            if (simple_outline) {
                emit_dome_roof(mesh, building.footprint, height, roof_color);
            } else {
                emit_flat_roof(mesh, building.footprint, building.holes, height, roof_color);
            }
            break;

        case RoofType::Flat:
        case RoofType::Unknown:
            emit_flat_roof(mesh, building.footprint, building.holes, height, roof_color);
            break;
    }

    // Compute bounding box for frustum culling
    mesh.compute_bounds();

    return mesh;
}

Mesh MeshBuilder::build_area_mesh(const Area& area) {
    Mesh mesh;

    if (area.polygon.size() < 3) {
        return mesh;
    }

    // Get color based on area type
    glm::vec4 area_color;
    float area_height = 0.02f; // Slightly above ground to avoid z-fighting

    switch (area.type) {
        case AreaType::Water:
            area_color = glm::vec4(0.25f, 0.45f, 0.65f, 1.0f); // Blue
            area_height = 0.01f;
            break;
        case AreaType::Park:
            area_color = glm::vec4(0.35f, 0.55f, 0.35f, 1.0f); // Medium green
            break;
        case AreaType::Forest:
            area_color = glm::vec4(0.25f, 0.4f, 0.25f, 1.0f); // Dark green
            break;
        case AreaType::Grass:
            area_color = glm::vec4(0.45f, 0.58f, 0.4f, 1.0f); // Light green
            break;
        case AreaType::Parking:
            area_color = glm::vec4(0.42f, 0.42f, 0.44f, 1.0f); // Gray asphalt
            area_height = 0.03f;
            break;
        case AreaType::Commercial:
            area_color = glm::vec4(0.55f, 0.5f, 0.6f, 1.0f); // Muted purple
            break;
        case AreaType::Residential:
            area_color = glm::vec4(0.52f, 0.52f, 0.48f, 1.0f); // Neutral gray-tan
            break;
        case AreaType::Industrial:
            area_color = glm::vec4(0.5f, 0.48f, 0.42f, 1.0f); // Brown-gray
            break;
        case AreaType::Farmland:
            area_color = glm::vec4(0.6f, 0.55f, 0.4f, 1.0f); // Wheat/tan
            break;
        case AreaType::Cemetery:
            area_color = glm::vec4(0.4f, 0.48f, 0.42f, 1.0f); // Muted sage
            break;
        default:
            area_color = glm::vec4(0.48f, 0.48f, 0.48f, 1.0f); // Neutral gray
            break;
    }

    // Prepare polygon for earcut (outer ring + holes)
    std::vector<std::vector<glm::dvec2>> polygon;
    polygon.push_back(area.polygon);

    // Add holes if any
    for (const auto& hole : area.holes) {
        polygon.push_back(hole);
    }

    // Run earcut triangulation
    std::vector<uint32_t> indices = mapbox::earcut<uint32_t>(polygon);

    if (indices.empty()) {
        return mesh; // Triangulation failed
    }

    // Flatten all polygon points for vertex lookup
    std::vector<glm::dvec2> all_points;
    for (const auto& ring : polygon) {
        for (const auto& pt : ring) {
            all_points.push_back(pt);
        }
    }

    // Add vertices
    glm::vec3 up_normal(0.0f, 1.0f, 0.0f);
    for (const auto& pt : all_points) {
        glm::vec3 pos(static_cast<float>(pt.x), area_height, static_cast<float>(-pt.y));
        mesh.vertices.push_back({pos, up_normal, glm::vec2(0.0f, 0.0f), area_color});
    }

    // Add indices
    for (uint32_t idx : indices) {
        mesh.indices.push_back(idx);
    }

    mesh.compute_bounds();
    return mesh;
}

Mesh MeshBuilder::merge_meshes(const std::vector<Mesh>& meshes) {
    if (meshes.empty()) return Mesh{};
    if (meshes.size() == 1) return meshes[0];

    // Pre-calculate total sizes for a single allocation
    size_t total_verts = 0;
    size_t total_indices = 0;
    for (const auto& m : meshes) {
        total_verts += m.vertices.size();
        total_indices += m.indices.size();
    }

    Mesh merged;
    merged.vertices.reserve(total_verts);
    merged.indices.reserve(total_indices);

    uint32_t base_vertex = 0;
    for (const auto& m : meshes) {
        // Append vertices directly
        merged.vertices.insert(merged.vertices.end(), m.vertices.begin(), m.vertices.end());

        // Append indices with offset
        for (uint32_t idx : m.indices) {
            merged.indices.push_back(idx + base_vertex);
        }

        base_vertex += static_cast<uint32_t>(m.vertices.size());
    }

    merged.compute_bounds();
    return merged;
}

} // namespace stratum::osm
