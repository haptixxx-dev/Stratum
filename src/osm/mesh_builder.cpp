#include "osm/mesh_builder.hpp"
#include <mapbox/earcut.hpp>
#include <unordered_map>
#include <algorithm>
#include <cctype>

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

    // === Generate roof using earcut triangulation ===
    // Prepare polygon for earcut (outer ring + holes)
    std::vector<std::vector<glm::dvec2>> polygon;
    polygon.push_back(building.footprint);

    // Add holes if any
    for (const auto& hole : building.holes) {
        polygon.push_back(hole);
    }

    // Run earcut
    std::vector<uint32_t> roof_indices = mapbox::earcut<uint32_t>(polygon);

    // Add roof vertices and indices
    glm::vec3 roof_normal(0.0f, 1.0f, 0.0f);
    uint32_t roof_base_idx = static_cast<uint32_t>(mesh.vertices.size());

    // Flatten all polygon points for vertex lookup
    std::vector<glm::dvec2> all_points;
    for (const auto& ring : polygon) {
        for (const auto& pt : ring) {
            all_points.push_back(pt);
        }
    }

    // Add roof vertices
    for (const auto& pt : all_points) {
        glm::vec3 pos(static_cast<float>(pt.x), height, static_cast<float>(-pt.y));
        mesh.vertices.push_back({pos, roof_normal, glm::vec2(0.0f, 0.0f), roof_color});
    }

    // Add roof triangles
    for (size_t i = 0; i < roof_indices.size(); i += 3) {
        mesh.indices.push_back(roof_base_idx + roof_indices[i]);
        mesh.indices.push_back(roof_base_idx + roof_indices[i + 1]);
        mesh.indices.push_back(roof_base_idx + roof_indices[i + 2]);
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
    // TODO: Implement later
    return mesh;
}

} // namespace stratum::osm
