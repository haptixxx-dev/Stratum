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

Mesh MeshBuilder::build_road_mesh(const Road& road) {
    Mesh mesh;

    if (road.polyline.size() < 2) {
        return mesh;
    }

    // Get road color based on type
    glm::vec4 road_color;
    switch (road.type) {
        case RoadType::Motorway:
        case RoadType::Trunk:
            road_color = glm::vec4(0.2f, 0.2f, 0.25f, 1.0f); // Dark asphalt
            break;
        case RoadType::Primary:
            road_color = glm::vec4(0.25f, 0.25f, 0.28f, 1.0f);
            break;
        case RoadType::Secondary:
            road_color = glm::vec4(0.3f, 0.3f, 0.32f, 1.0f);
            break;
        case RoadType::Tertiary:
        case RoadType::Residential:
            road_color = glm::vec4(0.35f, 0.35f, 0.38f, 1.0f);
            break;
        case RoadType::Service:
            road_color = glm::vec4(0.4f, 0.4f, 0.42f, 1.0f);
            break;
        case RoadType::Footway:
        case RoadType::Path:
            road_color = glm::vec4(0.6f, 0.55f, 0.45f, 1.0f); // Tan/dirt
            break;
        case RoadType::Cycleway:
            road_color = glm::vec4(0.3f, 0.5f, 0.3f, 1.0f); // Greenish
            break;
        default:
            road_color = glm::vec4(0.4f, 0.4f, 0.4f, 1.0f);
            break;
    }

    const float half_width = road.width * 0.5f;
    const float road_height = 0.05f; // Slightly above ground to avoid z-fighting
    const glm::vec3 up_normal(0.0f, 1.0f, 0.0f);

    // Pre-compute perpendiculars for each point
    std::vector<glm::vec2> perpendiculars(road.polyline.size());

    for (size_t i = 0; i < road.polyline.size(); ++i) {
        glm::dvec2 dir;

        if (i == 0) {
            // First point: use direction to next point
            dir = glm::normalize(road.polyline[1] - road.polyline[0]);
        } else if (i == road.polyline.size() - 1) {
            // Last point: use direction from previous point
            dir = glm::normalize(road.polyline[i] - road.polyline[i - 1]);
        } else {
            // Middle points: average of incoming and outgoing directions (miter)
            glm::dvec2 dir_in = glm::normalize(road.polyline[i] - road.polyline[i - 1]);
            glm::dvec2 dir_out = glm::normalize(road.polyline[i + 1] - road.polyline[i]);
            dir = glm::normalize(dir_in + dir_out);

            // Handle case where directions are opposite (180 degree turn)
            if (glm::length(dir) < 0.001) {
                dir = dir_in;
            }
        }

        // Perpendicular is 90 degrees to direction (in 2D: rotate by 90)
        // Note: Y is flipped in our coordinate system
        perpendiculars[i] = glm::vec2(static_cast<float>(-dir.y), static_cast<float>(dir.x));
    }

    // Generate vertices and triangles for each segment
    for (size_t i = 0; i < road.polyline.size() - 1; ++i) {
        const glm::dvec2& p0 = road.polyline[i];
        const glm::dvec2& p1 = road.polyline[i + 1];
        const glm::vec2& perp0 = perpendiculars[i];
        const glm::vec2& perp1 = perpendiculars[i + 1];

        // Four corners of this road segment quad
        // Note: z is negated because we use -y for z coordinate
        glm::vec3 v0(static_cast<float>(p0.x) - perp0.x * half_width, road_height,
                     static_cast<float>(-p0.y) - perp0.y * half_width);
        glm::vec3 v1(static_cast<float>(p0.x) + perp0.x * half_width, road_height,
                     static_cast<float>(-p0.y) + perp0.y * half_width);
        glm::vec3 v2(static_cast<float>(p1.x) + perp1.x * half_width, road_height,
                     static_cast<float>(-p1.y) + perp1.y * half_width);
        glm::vec3 v3(static_cast<float>(p1.x) - perp1.x * half_width, road_height,
                     static_cast<float>(-p1.y) - perp1.y * half_width);

        uint32_t base_idx = static_cast<uint32_t>(mesh.vertices.size());

        // Add 4 vertices for this quad
        mesh.vertices.push_back({v0, up_normal, glm::vec2(0.0f, 0.0f), road_color});
        mesh.vertices.push_back({v1, up_normal, glm::vec2(1.0f, 0.0f), road_color});
        mesh.vertices.push_back({v2, up_normal, glm::vec2(1.0f, 1.0f), road_color});
        mesh.vertices.push_back({v3, up_normal, glm::vec2(0.0f, 1.0f), road_color});

        // Two triangles for the quad (CCW winding, facing up)
        mesh.indices.push_back(base_idx + 0);
        mesh.indices.push_back(base_idx + 1);
        mesh.indices.push_back(base_idx + 2);

        mesh.indices.push_back(base_idx + 0);
        mesh.indices.push_back(base_idx + 2);
        mesh.indices.push_back(base_idx + 3);
    }

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

std::vector<Mesh> MeshBuilder::build_junction_meshes(const std::vector<Road>& roads) {
    std::vector<Mesh> junctions;

    if (roads.empty()) return junctions;

    const double JUNCTION_THRESHOLD = 2.0; // meters - endpoints within this distance form a junction
    const float JUNCTION_HEIGHT = 0.06f;   // Slightly above roads
    const int CIRCLE_SEGMENTS = 12;
    const double CELL_SIZE = JUNCTION_THRESHOLD * 2.0;  // Spatial hash cell size

    // Collect all road endpoints with their width
    struct RoadEndpoint {
        glm::dvec2 position;
        float width;
        size_t road_idx;
    };
    std::vector<RoadEndpoint> endpoints;
    endpoints.reserve(roads.size() * 2);

    for (size_t ri = 0; ri < roads.size(); ++ri) {
        const auto& road = roads[ri];
        if (road.polyline.size() < 2) continue;

        // Start endpoint
        endpoints.push_back({road.polyline[0], road.width, ri});

        // End endpoint
        endpoints.push_back({road.polyline.back(), road.width, ri});
    }

    // Build spatial hash for O(1) neighbor lookup
    auto hash_cell = [CELL_SIZE](const glm::dvec2& pos) -> std::pair<int, int> {
        return {static_cast<int>(std::floor(pos.x / CELL_SIZE)),
                static_cast<int>(std::floor(pos.y / CELL_SIZE))};
    };

    std::unordered_map<int64_t, std::vector<size_t>> spatial_hash;
    auto cell_key = [](int cx, int cy) -> int64_t {
        return (static_cast<int64_t>(cx) << 32) | static_cast<uint32_t>(cy);
    };

    for (size_t i = 0; i < endpoints.size(); ++i) {
        auto [cx, cy] = hash_cell(endpoints[i].position);
        spatial_hash[cell_key(cx, cy)].push_back(i);
    }

    // Cluster endpoints into junctions using spatial hash
    std::vector<bool> processed(endpoints.size(), false);
    double thresh_sq = JUNCTION_THRESHOLD * JUNCTION_THRESHOLD;

    for (size_t i = 0; i < endpoints.size(); ++i) {
        if (processed[i]) continue;

        std::vector<size_t> cluster;
        cluster.push_back(i);
        processed[i] = true;

        auto [cx, cy] = hash_cell(endpoints[i].position);

        // Check neighboring cells (3x3 grid)
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                auto it = spatial_hash.find(cell_key(cx + dx, cy + dy));
                if (it == spatial_hash.end()) continue;

                for (size_t j : it->second) {
                    if (j <= i || processed[j]) continue;

                    glm::dvec2 diff = endpoints[j].position - endpoints[i].position;
                    if (diff.x * diff.x + diff.y * diff.y < thresh_sq) {
                        cluster.push_back(j);
                        processed[j] = true;
                    }
                }
            }
        }

        // Only create junction mesh if multiple roads meet
        if (cluster.size() < 2) continue;

        // Compute junction center and max radius
        glm::dvec2 center(0.0);
        float max_width = 0.0f;
        for (size_t idx : cluster) {
            center += endpoints[idx].position;
            max_width = std::max(max_width, endpoints[idx].width);
        }
        center /= static_cast<double>(cluster.size());

        float radius = max_width * 0.6f;

        // Create a filled circle at the junction
        Mesh mesh;
        glm::vec4 junction_color(0.3f, 0.3f, 0.32f, 1.0f);
        glm::vec3 up_normal(0.0f, 1.0f, 0.0f);

        // Center vertex
        uint32_t center_idx = 0;
        glm::vec3 center_pos(static_cast<float>(center.x), JUNCTION_HEIGHT,
                             static_cast<float>(-center.y));
        mesh.vertices.push_back({center_pos, up_normal, glm::vec2(0.5f, 0.5f), junction_color});

        // Circle vertices
        for (int s = 0; s < CIRCLE_SEGMENTS; ++s) {
            float angle = static_cast<float>(s) / CIRCLE_SEGMENTS * 2.0f * 3.14159265f;
            float x = static_cast<float>(center.x) + radius * std::cos(angle);
            float z = static_cast<float>(-center.y) + radius * std::sin(angle);

            mesh.vertices.push_back({glm::vec3(x, JUNCTION_HEIGHT, z), up_normal,
                                     glm::vec2(0.5f + 0.5f * std::cos(angle),
                                               0.5f + 0.5f * std::sin(angle)),
                                     junction_color});
        }

        // Triangle fan
        for (int s = 0; s < CIRCLE_SEGMENTS; ++s) {
            int next = (s + 1) % CIRCLE_SEGMENTS;
            mesh.indices.push_back(center_idx);
            mesh.indices.push_back(static_cast<uint32_t>(1 + s));
            mesh.indices.push_back(static_cast<uint32_t>(1 + next));
        }

        mesh.compute_bounds();
        junctions.push_back(std::move(mesh));
    }

    return junctions;
}

} // namespace stratum::osm
