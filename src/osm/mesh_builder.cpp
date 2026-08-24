#include "osm/mesh_builder.hpp"
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
    const float roof_pitch_ratio = 0.3f; // Roof height = width * ratio

    if (building.roof_type == RoofType::Gabled && building.holes.empty()) {
        // Gabled roof: ridge along longest axis
        glm::dvec2 axis, center;
        double length, width;
        compute_principal_axis(building.footprint, axis, center, length, width);

        float ridge_height = static_cast<float>(width * 0.5 * roof_pitch_ratio);
        glm::dvec2 perp(-axis.y, axis.x);

        // Ridge endpoints (at center, along axis)
        glm::dvec2 ridge_start_2d = center - axis * (length * 0.5);
        glm::dvec2 ridge_end_2d = center + axis * (length * 0.5);

        glm::vec3 ridge_start(static_cast<float>(ridge_start_2d.x), height + ridge_height,
                              static_cast<float>(-ridge_start_2d.y));
        glm::vec3 ridge_end(static_cast<float>(ridge_end_2d.x), height + ridge_height,
                            static_cast<float>(-ridge_end_2d.y));

        // For each edge, create a sloped roof triangle fan to ridge
        for (size_t i = 0; i < n; ++i) {
            size_t next = (i + 1) % n;
            if (i == n - 1 && building.footprint[0] == building.footprint[n-1]) continue;

            glm::vec3 p0(static_cast<float>(building.footprint[i].x), height,
                         static_cast<float>(-building.footprint[i].y));
            glm::vec3 p1(static_cast<float>(building.footprint[next].x), height,
                         static_cast<float>(-building.footprint[next].y));

            // Determine which side of ridge this edge is on
            glm::dvec2 edge_mid = (building.footprint[i] + building.footprint[next]) * 0.5;
            double side = glm::dot(edge_mid - center, perp);

            // Create roof quad from edge to ridge
            uint32_t base_idx = static_cast<uint32_t>(mesh.vertices.size());

            // Calculate normal for this roof face
            glm::vec3 edge_vec = p1 - p0;
            glm::vec3 to_ridge = (side > 0) ? (ridge_start - p0) : (ridge_end - p0);
            glm::vec3 face_normal = glm::normalize(glm::cross(edge_vec, to_ridge));
            if (face_normal.y < 0) face_normal = -face_normal; // Ensure upward-facing

            mesh.vertices.push_back({p0, face_normal, glm::vec2(0.0f, 0.0f), roof_color});
            mesh.vertices.push_back({p1, face_normal, glm::vec2(1.0f, 0.0f), roof_color});
            mesh.vertices.push_back({ridge_end, face_normal, glm::vec2(1.0f, 1.0f), roof_color});
            mesh.vertices.push_back({ridge_start, face_normal, glm::vec2(0.0f, 1.0f), roof_color});

            // Two triangles for roof quad
            mesh.indices.push_back(base_idx + 0);
            mesh.indices.push_back(base_idx + 1);
            mesh.indices.push_back(base_idx + 2);

            mesh.indices.push_back(base_idx + 0);
            mesh.indices.push_back(base_idx + 2);
            mesh.indices.push_back(base_idx + 3);
        }
    } else if ((building.roof_type == RoofType::Hipped || building.roof_type == RoofType::Pyramidal)
               && building.holes.empty()) {
        // Hipped/Pyramidal roof: all edges slope to center apex
        glm::dvec2 center_2d = compute_centroid(building.footprint);

        // Find the minimum distance from center to any edge for apex height
        double min_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < n; ++i) {
            size_t next = (i + 1) % n;
            if (i == n - 1 && building.footprint[0] == building.footprint[n-1]) continue;

            glm::dvec2 edge = building.footprint[next] - building.footprint[i];
            glm::dvec2 to_center = center_2d - building.footprint[i];
            double edge_len = glm::length(edge);
            if (edge_len > 0.001) {
                double t = glm::clamp(glm::dot(to_center, edge) / (edge_len * edge_len), 0.0, 1.0);
                glm::dvec2 closest = building.footprint[i] + edge * t;
                double dist = glm::length(center_2d - closest);
                min_dist = std::min(min_dist, dist);
            }
        }

        float apex_height = static_cast<float>(min_dist * roof_pitch_ratio);
        glm::vec3 apex(static_cast<float>(center_2d.x), height + apex_height,
                       static_cast<float>(-center_2d.y));

        // Create triangular roof faces from each edge to apex
        for (size_t i = 0; i < n; ++i) {
            size_t next = (i + 1) % n;
            if (i == n - 1 && building.footprint[0] == building.footprint[n-1]) continue;

            glm::vec3 p0(static_cast<float>(building.footprint[i].x), height,
                         static_cast<float>(-building.footprint[i].y));
            glm::vec3 p1(static_cast<float>(building.footprint[next].x), height,
                         static_cast<float>(-building.footprint[next].y));

            // Calculate face normal
            glm::vec3 edge_vec = p1 - p0;
            glm::vec3 to_apex = apex - p0;
            glm::vec3 face_normal = glm::normalize(glm::cross(edge_vec, to_apex));
            if (face_normal.y < 0) face_normal = -face_normal;

            uint32_t base_idx = static_cast<uint32_t>(mesh.vertices.size());

            mesh.vertices.push_back({p0, face_normal, glm::vec2(0.0f, 0.0f), roof_color});
            mesh.vertices.push_back({p1, face_normal, glm::vec2(1.0f, 0.0f), roof_color});
            mesh.vertices.push_back({apex, face_normal, glm::vec2(0.5f, 1.0f), roof_color});

            mesh.indices.push_back(base_idx + 0);
            mesh.indices.push_back(base_idx + 1);
            mesh.indices.push_back(base_idx + 2);
        }
    } else {
        // Flat roof (default): use earcut triangulation
        std::vector<std::vector<glm::dvec2>> polygon;
        polygon.push_back(building.footprint);

        for (const auto& hole : building.holes) {
            polygon.push_back(hole);
        }

        std::vector<uint32_t> roof_indices = mapbox::earcut<uint32_t>(polygon);

        glm::vec3 roof_normal(0.0f, 1.0f, 0.0f);
        uint32_t roof_base_idx = static_cast<uint32_t>(mesh.vertices.size());

        std::vector<glm::dvec2> all_points;
        for (const auto& ring : polygon) {
            for (const auto& pt : ring) {
                all_points.push_back(pt);
            }
        }

        for (const auto& pt : all_points) {
            glm::vec3 pos(static_cast<float>(pt.x), height, static_cast<float>(-pt.y));
            mesh.vertices.push_back({pos, roof_normal, glm::vec2(0.0f, 0.0f), roof_color});
        }

        for (size_t i = 0; i < roof_indices.size(); i += 3) {
            mesh.indices.push_back(roof_base_idx + roof_indices[i]);
            mesh.indices.push_back(roof_base_idx + roof_indices[i + 1]);
            mesh.indices.push_back(roof_base_idx + roof_indices[i + 2]);
        }
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
