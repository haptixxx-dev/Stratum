/**
 * @file coordinates.hpp
 * @brief Coordinate conversion utilities for OSM data
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * This file provides coordinate conversion between WGS84 (lat/lon),
 * Web Mercator projection, and local meter-based coordinates.
 */

#pragma once

#include "osm/types.hpp"
#include <glm/glm.hpp>
#include <vector>

namespace stratum::osm {

// ============================================================================
// Constants
// ============================================================================

/// WGS84 Earth radius (semi-major axis) in meters
constexpr double EARTH_RADIUS_M = 6378137.0;

/// Degrees to radians conversion factor
constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;

/// Radians to degrees conversion factor
constexpr double RAD_TO_DEG = 180.0 / 3.14159265358979323846;

/// Approximate meters per degree of latitude (constant)
constexpr double METERS_PER_DEG_LAT = 111320.0;

// ============================================================================
// Coordinate Converter
// ============================================================================

/**
 * @brief Converts between WGS84, Web Mercator, and local coordinates
 *
 * The CoordinateConverter handles projection from geographic coordinates
 * (latitude/longitude in WGS84) to a local Cartesian coordinate system
 * centered at an origin point. This allows for high-precision local
 * coordinates without floating-point precision issues from large values.
 *
 * Coordinate flow:
 * @code
 * WGS84 (lat, lon) -> Web Mercator (x, y meters) -> Local (centered at origin)
 * @endcode
 *
 * Usage:
 * @code
 * CoordinateConverter converter;
 * converter.set_origin(bounds);  // Center at data bounds
 *
 * glm::dvec2 local = converter.wgs84_to_local(51.5074, -0.1278);
 * @endcode
 */
class CoordinateConverter {
public:
    CoordinateConverter() = default;

    /**
     * @brief Initialize with a specific origin point
     * @param lat Origin latitude in degrees
     * @param lon Origin longitude in degrees
     */
    void set_origin(double lat, double lon);

    /**
     * @brief Initialize with the center of a bounding box
     * @param bounds Bounding box to center on
     */
    void set_origin(const BoundingBox& bounds);

    /**
     * @brief Convert WGS84 coordinates to Web Mercator projection
     * @param lat Latitude in degrees (-85.051 to 85.051)
     * @param lon Longitude in degrees (-180 to 180)
     * @return Web Mercator coordinates in meters (x = easting, y = northing)
     *
     * Uses EPSG:3857 (Web Mercator / Pseudo-Mercator) projection.
     * Note: This projection is not suitable for areas near poles.
     */
    [[nodiscard]] static glm::dvec2 wgs84_to_mercator(double lat, double lon);

    /**
     * @brief Convert Web Mercator coordinates back to WGS84
     * @param x Easting in meters
     * @param y Northing in meters
     * @return WGS84 coordinates as (latitude, longitude) in degrees
     */
    [[nodiscard]] static glm::dvec2 mercator_to_wgs84(double x, double y);

    /**
     * @brief Convert WGS84 to local coordinates centered at origin
     * @param lat Latitude in degrees
     * @param lon Longitude in degrees
     * @return Local coordinates in meters, centered at (0, 0)
     *
     * @pre set_origin() must have been called
     */
    [[nodiscard]] glm::dvec2 wgs84_to_local(double lat, double lon) const;

    /**
     * @brief Convert local coordinates back to WGS84
     * @param x Local x coordinate in meters
     * @param y Local y coordinate in meters
     * @return WGS84 coordinates as (latitude, longitude) in degrees
     */
    [[nodiscard]] glm::dvec2 local_to_wgs84(double x, double y) const;

    /**
     * @brief Get the coordinate system information
     * @return Reference to internal CoordinateSystem
     */
    [[nodiscard]] const CoordinateSystem& get_coord_system() const {
        return m_coord_system;
    }

    /**
     * @brief Check if the converter has been initialized
     * @return true if set_origin() has been called
     */
    [[nodiscard]] bool is_initialized() const { return m_initialized; }

    /**
     * @brief Calculate meters per degree of longitude at a given latitude
     * @param lat Latitude in degrees
     * @return Meters per degree of longitude
     *
     * Longitude degrees get smaller toward the poles due to meridian convergence.
     */
    [[nodiscard]] static double meters_per_degree_lon(double lat);

    /**
     * @brief Calculate meters per degree of latitude
     * @return Approximately 111,320 meters (nearly constant)
     */
    [[nodiscard]] static double meters_per_degree_lat();

private:
    CoordinateSystem m_coord_system;
    bool m_initialized = false;
};

// ============================================================================
// Geometry Utilities
// ============================================================================

namespace geometry {

/**
 * @brief Calculate the signed area of a polygon
 * @param polygon Ordered list of vertices
 * @return Signed area (positive = CCW, negative = CW)
 *
 * Uses the shoelace formula.
 */
[[nodiscard]] double polygon_area(const std::vector<glm::dvec2>& polygon);

/**
 * @brief Check if a polygon has clockwise winding
 * @param polygon Ordered list of vertices
 * @return true if polygon is clockwise
 */
[[nodiscard]] bool is_clockwise(const std::vector<glm::dvec2>& polygon);

/**
 * @brief Ensure polygon has counter-clockwise winding (standard for outer rings)
 * @param polygon Polygon to potentially reverse
 */
void ensure_ccw(std::vector<glm::dvec2>& polygon);

/**
 * @brief Ensure polygon has clockwise winding (standard for inner rings/holes)
 * @param polygon Polygon to potentially reverse
 */
void ensure_cw(std::vector<glm::dvec2>& polygon);

/**
 * @brief Calculate the centroid of a polygon
 * @param polygon Ordered list of vertices
 * @return Centroid point
 */
[[nodiscard]] glm::dvec2 centroid(const std::vector<glm::dvec2>& polygon);

/**
 * @brief Simplify a polyline using Douglas-Peucker algorithm
 * @param points Input polyline
 * @param epsilon Maximum perpendicular distance for point removal
 * @return Simplified polyline
 */
[[nodiscard]] std::vector<glm::dvec2> simplify(
    const std::vector<glm::dvec2>& points,
    double epsilon
);

/**
 * @brief Calculate the length of a polyline
 * @param points Ordered list of points
 * @return Total length in same units as input
 */
[[nodiscard]] double polyline_length(const std::vector<glm::dvec2>& points);

/**
 * @brief Calculate perpendicular distance from point to line segment
 * @param point The point
 * @param line_start Start of line segment
 * @param line_end End of line segment
 * @return Perpendicular distance
 */
[[nodiscard]] double point_to_line_distance(
    const glm::dvec2& point,
    const glm::dvec2& line_start,
    const glm::dvec2& line_end
);

} // namespace geometry

} // namespace stratum::osm
