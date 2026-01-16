/**
 * @file coordinates.cpp
 * @brief Implementation of coordinate conversion utilities
 */

#include "osm/coordinates.hpp"
#include <cmath>
#include <algorithm>

namespace stratum::osm {

// ============================================================================
// BoundingBox Implementation
// ============================================================================

double BoundingBox::width_meters() const {
    if (!is_valid()) return 0.0;
    double center_lat = (min_lat + max_lat) / 2.0;
    return (max_lon - min_lon) * CoordinateConverter::meters_per_degree_lon(center_lat);
}

double BoundingBox::height_meters() const {
    if (!is_valid()) return 0.0;
    return (max_lat - min_lat) * CoordinateConverter::meters_per_degree_lat();
}

// ============================================================================
// CoordinateConverter Implementation
// ============================================================================

void CoordinateConverter::set_origin(double lat, double lon) {
    m_coord_system.origin_latlon = glm::dvec2(lat, lon);
    m_coord_system.origin_mercator = wgs84_to_mercator(lat, lon);
    m_initialized = true;
}

void CoordinateConverter::set_origin(const BoundingBox& bounds) {
    auto center = bounds.center();
    set_origin(center.x, center.y);
}

glm::dvec2 CoordinateConverter::wgs84_to_mercator(double lat, double lon) {
    // Clamp latitude to valid range for Web Mercator
    // (beyond ~85 degrees, projection becomes infinite)
    lat = std::clamp(lat, -85.051128, 85.051128);

    // Web Mercator (EPSG:3857) projection
    double x = EARTH_RADIUS_M * lon * DEG_TO_RAD;
    double y = EARTH_RADIUS_M * std::log(std::tan((M_PI / 4.0) + (lat * DEG_TO_RAD / 2.0)));

    return glm::dvec2(x, y);
}

glm::dvec2 CoordinateConverter::mercator_to_wgs84(double x, double y) {
    double lon = (x / EARTH_RADIUS_M) * RAD_TO_DEG;
    double lat = (2.0 * std::atan(std::exp(y / EARTH_RADIUS_M)) - M_PI / 2.0) * RAD_TO_DEG;
    return glm::dvec2(lat, lon);
}

glm::dvec2 CoordinateConverter::wgs84_to_local(double lat, double lon) const {
    if (!m_initialized) {
        // Fallback: just return mercator without offset
        return wgs84_to_mercator(lat, lon);
    }

    auto mercator = wgs84_to_mercator(lat, lon);
    // Subtract origin to get local coordinates centered at (0, 0)
    return mercator - m_coord_system.origin_mercator;
}

glm::dvec2 CoordinateConverter::local_to_wgs84(double x, double y) const {
    if (!m_initialized) {
        return mercator_to_wgs84(x, y);
    }

    auto mercator = glm::dvec2(x, y) + m_coord_system.origin_mercator;
    return mercator_to_wgs84(mercator.x, mercator.y);
}

double CoordinateConverter::meters_per_degree_lon(double lat) {
    // Meters per degree of longitude decreases toward poles
    return METERS_PER_DEG_LAT * std::cos(lat * DEG_TO_RAD);
}

double CoordinateConverter::meters_per_degree_lat() {
    return METERS_PER_DEG_LAT;
}

// ============================================================================
// Geometry Utilities Implementation
// ============================================================================

namespace geometry {

double polygon_area(const std::vector<glm::dvec2>& polygon) {
    if (polygon.size() < 3) return 0.0;

    // Shoelace formula
    double area = 0.0;
    size_t n = polygon.size();

    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        area += polygon[i].x * polygon[j].y;
        area -= polygon[j].x * polygon[i].y;
    }

    return area / 2.0;
}

bool is_clockwise(const std::vector<glm::dvec2>& polygon) {
    return polygon_area(polygon) < 0.0;
}

void ensure_ccw(std::vector<glm::dvec2>& polygon) {
    if (is_clockwise(polygon)) {
        std::reverse(polygon.begin(), polygon.end());
    }
}

void ensure_cw(std::vector<glm::dvec2>& polygon) {
    if (!is_clockwise(polygon)) {
        std::reverse(polygon.begin(), polygon.end());
    }
}

glm::dvec2 centroid(const std::vector<glm::dvec2>& polygon) {
    if (polygon.empty()) return glm::dvec2(0.0);
    if (polygon.size() == 1) return polygon[0];
    if (polygon.size() == 2) return (polygon[0] + polygon[1]) / 2.0;

    double cx = 0.0;
    double cy = 0.0;
    double signed_area = 0.0;
    size_t n = polygon.size();

    for (size_t i = 0; i < n; ++i) {
        size_t j = (i + 1) % n;
        double cross = polygon[i].x * polygon[j].y - polygon[j].x * polygon[i].y;
        signed_area += cross;
        cx += (polygon[i].x + polygon[j].x) * cross;
        cy += (polygon[i].y + polygon[j].y) * cross;
    }

    signed_area /= 2.0;
    if (std::abs(signed_area) < 1e-10) {
        // Degenerate polygon, return simple average
        glm::dvec2 sum(0.0);
        for (const auto& p : polygon) sum += p;
        return sum / static_cast<double>(n);
    }

    cx /= (6.0 * signed_area);
    cy /= (6.0 * signed_area);

    return glm::dvec2(cx, cy);
}

double point_to_line_distance(
    const glm::dvec2& point,
    const glm::dvec2& line_start,
    const glm::dvec2& line_end
) {
    glm::dvec2 line = line_end - line_start;
    double line_len_sq = glm::dot(line, line);

    if (line_len_sq < 1e-10) {
        // Degenerate line (start == end)
        return glm::length(point - line_start);
    }

    // Project point onto line, clamping to segment
    double t = std::clamp(glm::dot(point - line_start, line) / line_len_sq, 0.0, 1.0);
    glm::dvec2 projection = line_start + t * line;

    return glm::length(point - projection);
}

double polyline_length(const std::vector<glm::dvec2>& points) {
    if (points.size() < 2) return 0.0;

    double length = 0.0;
    for (size_t i = 1; i < points.size(); ++i) {
        length += glm::length(points[i] - points[i - 1]);
    }
    return length;
}

std::vector<glm::dvec2> simplify(
    const std::vector<glm::dvec2>& points,
    double epsilon
) {
    if (points.size() < 3) return points;

    // Douglas-Peucker algorithm
    // Find the point with maximum distance from line between first and last
    double max_dist = 0.0;
    size_t max_idx = 0;

    const auto& first = points.front();
    const auto& last = points.back();

    for (size_t i = 1; i < points.size() - 1; ++i) {
        double dist = point_to_line_distance(points[i], first, last);
        if (dist > max_dist) {
            max_dist = dist;
            max_idx = i;
        }
    }

    // If max distance is greater than epsilon, recursively simplify
    if (max_dist > epsilon) {
        // Recursively simplify the two segments
        std::vector<glm::dvec2> first_half(points.begin(), points.begin() + max_idx + 1);
        std::vector<glm::dvec2> second_half(points.begin() + max_idx, points.end());

        auto simplified_first = simplify(first_half, epsilon);
        auto simplified_second = simplify(second_half, epsilon);

        // Combine results (remove duplicate point at junction)
        std::vector<glm::dvec2> result;
        result.reserve(simplified_first.size() + simplified_second.size() - 1);
        result.insert(result.end(), simplified_first.begin(), simplified_first.end() - 1);
        result.insert(result.end(), simplified_second.begin(), simplified_second.end());

        return result;
    } else {
        // All intermediate points can be removed
        return {first, last};
    }
}

} // namespace geometry

} // namespace stratum::osm
