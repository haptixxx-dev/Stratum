/**
 * @file centerline.hpp
 * @brief Resampled, smoothed, mitred centerline for corridor extrusion
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * A Centerline turns a raw OSM polyline into an ordered list of Stations: the
 * sweep frames the corridor extruder rides along. It answers three questions the
 * raw polyline cannot:
 *
 * 1. Where should there be a vertex column? OSM polylines are coarse on straights
 *    and coarse in corners alike, so they are resampled at curvature-adaptive
 *    spacing.
 * 2. How far out does an offset point sit at a joint? The correct answer is
 *    half_width / cos(theta/2), not half_width. Station::miter_scale carries the
 *    1/cos(theta/2) factor; the old extruder omitted it and pinched at every corner.
 * 3. How long is the road so far? Station::arclength is metres from the start and
 *    is the only input to the V texture coordinate.
 *
 * Coordinates are the same 2D local metres as Road::polyline and
 * GraphEdge::polyline. Nothing here knows about world Y-up space; that mapping
 * belongs to the corridor extruder.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API.
 */

#pragma once

#include <glm/glm.hpp>

#include <algorithm>
#include <cstddef>
#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Stations
// ============================================================================

/**
 * @brief Lateral bound standing for "no fold limit on this side"
 *
 * A finite sentinel rather than an infinity, so a bound can be linearly
 * interpolated -- as slice() does -- without producing inf - inf. Roads are
 * measured in metres, so 1e30 is unreachable by any real profile.
 */
constexpr double kUnboundedLateral = 1e30;

/**
 * @brief One sweep frame along the centerline
 *
 * A station is a position, an orthonormal 2D frame, and the parameters needed to
 * place an offset vertex column correctly at that position.
 *
 * The frame convention, fixed here and relied on everywhere downstream:
 *
 * - `tangent` is the unit direction of travel, from the edge's `from` node
 *   towards its `to` node.
 * - `normal` is the unit LEFT normal. On a straight it is the tangent rotated a
 *   quarter turn counter-clockwise in the 2D plane: (-tangent.y, tangent.x). At a
 *   joint it is the MITER BISECTOR of the incoming and outgoing left normals,
 *   renormalised, which is why miter_scale exists as a separate factor.
 * - Positive lateral offsets are to the LEFT of travel. See offset_point().
 */
struct Station {
    glm::dvec2 position{0.0};
    glm::dvec2 tangent{1.0, 0.0};   ///< unit, direction of travel
    glm::dvec2 normal{0.0, 1.0};    ///< unit MITER BISECTOR pointing LEFT of travel
    double arclength = 0.0;         ///< metres from the start of the centerline
    double curvature = 0.0;         ///< signed, 1/metres

    /**
     * @brief Miter compensation factor, 1/cos(theta/2)
     *
     * theta is the exterior turn angle at the joint, so miter_scale is 1.0 on a
     * straight and grows without bound as the turn approaches a hairpin.
     * Multiply every lateral offset by it, otherwise the ribbon pinches inward at
     * corners. Always >= 1.0, and never above ResampleConfig::miter_limit: past
     * that limit the joint is bevelled instead and this returns to 1.0.
     */
    double miter_scale = 1.0;

    /**
     * @brief Miter limit exceeded; the joint is bevelled, not mitred
     *
     * A bevelled joint is represented as TWO stations sharing one position and
     * one arclength. The first carries the incoming segment's left normal, the
     * second the outgoing segment's left normal, and both have miter_scale 1.0.
     * The zero-length band between them is the bevel wedge, so a consumer that
     * walks stations pairwise gets the bevel for free without a special case.
     *
     * The two stations coincide exactly at lateral 0, so a vertex column emitted
     * on the centreline itself produces zero-area triangles across a bevel. The
     * corridor extruder drops degenerate triangles rather than special-casing
     * this.
     */
    bool is_bevel = false;

    /**
     * @brief Most-positive lateral offset that does not fold the ribbon
     *
     * kUnboundedLateral when nothing on the left of travel folds here, which is
     * every station of a straight and every joint turning to the right.
     *
     * A mitred joint pulls the INNER offset column backwards along both of its
     * legs by `lateral * tan(theta/2)`. Once that retreat exceeds the adjacent
     * band length the column overshoots its neighbour, the offset edge reverses,
     * and the ribbon folds through itself: the plan's "self-intersection guard"
     * at docs/plans/road_network_plan.md. The bound is the lateral distance at
     * which the retreat exactly consumes the shorter adjacent band, which on a
     * resampled curve is the local turn radius. A bevel pair bounds the inner
     * side at 0, because its wedge band has no length to retreat into.
     *
     * offset_point() clamps to it, so no caller can produce a folded column.
     * Clamping collapses the profile outboard of the bound onto the bound rather
     * than inverting it, which is the "clamp or collapse" the plan asks for.
     */
    double lateral_max = kUnboundedLateral;

    /// Most-negative lateral offset that does not fold the ribbon; see lateral_max
    double lateral_min = -kUnboundedLateral;
};

/**
 * @brief An ordered run of stations covering one edge
 *
 * Stations are strictly ordered by arclength, except across a bevel pair, whose
 * two stations share an arclength.
 */
struct Centerline {
    std::vector<Station> stations;

    /**
     * @brief Arclength of the last station, in metres
     *
     * For a centerline produced by build_centerline() this is the total length.
     * For one produced by slice() arclengths are NOT rebased, so this is the end
     * arclength in the source parameterisation, not the length of the slice. Take
     * `stations.back().arclength - stations.front().arclength` when the span is
     * what is wanted.
     */
    [[nodiscard]] double length() const {
        return stations.empty() ? 0.0 : stations.back().arclength;
    }

    /// At least 2 stations, so there is at least one band to extrude
    [[nodiscard]] bool is_valid() const { return stations.size() >= 2; }
};

// ============================================================================
// Construction
// ============================================================================

/**
 * @brief Resampling, smoothing, and miter tolerances
 *
 * All distances are metres.
 */
struct ResampleConfig {
    double max_spacing = 8.0;       ///< metres between stations on a straight
    double min_spacing = 0.5;
    double max_deviation = 0.05;    ///< chord-to-arc error tolerance driving density in curves
    double miter_limit = 4.0;       ///< beyond this, bevel instead of mitre
    bool   smooth = true;           ///< fit chordal Catmull-Rom before resampling
    double weld_epsilon = 1e-4;     ///< collapse consecutive points closer than this, metres

    /**
     * @brief How far the smoothed curve may depart from the surveyed line, metres
     *
     * max_deviation bounds the STATIONS against the fitted spline. It says
     * nothing about how far that spline sits from the polyline the surveyor
     * actually drew, and tightening it never moved that error: a Catmull-Rom
     * bows off each chord by roughly 0.15 * chord * sin(turn / 2), which is
     * linear in segment length, so two 200 m arms meeting at a 21 degree bend
     * put the road 5 m off its own survey however dense the stations are.
     *
     * This is the bound that does move it. Each segment's Bezier handles are
     * shortened until the segment cannot leave a band this wide either side of
     * its chord, so smoothing stays what it is meant to be -- rounding the
     * corners OSM left coarse -- instead of relocating the road. Ignored when
     * smooth is false, since nothing is then invented to bound.
     */
    double max_smoothing_offset = 0.5;
};

/**
 * @brief Build a centerline from a raw polyline
 *
 * Pipeline:
 *
 * 1. Weld consecutive points closer together than cfg.weld_epsilon. OSM extracts
 *    contain exact duplicates, and a zero-length segment has no tangent.
 * 2. When cfg.smooth, fit a chordal Catmull-Rom through the welded points. The
 *    curve interpolates the input points, so the surveyed geometry is preserved
 *    and only the space between vertices is invented.
 * 3. Resample at curvature-adaptive spacing: at most cfg.max_spacing on a
 *    straight, never below cfg.min_spacing, tightening in curves until the
 *    chord-to-arc deviation is within cfg.max_deviation.
 * 4. Compute the frame, arclength, signed curvature, and miter at each station,
 *    emitting a bevel pair wherever 1/cos(theta/2) would exceed cfg.miter_limit.
 *
 * The first and last input points are always stations, so an edge's ends still
 * meet its neighbours' ends exactly.
 *
 * @param polyline Centerline in 2D local metres, in the direction of travel
 * @param cfg      Resampling tolerances
 * @return The stations; invalid (fewer than 2 stations) when the input welds down
 *         to fewer than 2 distinct points
 */
[[nodiscard]] Centerline build_centerline(const std::vector<glm::dvec2>& polyline,
                                          const ResampleConfig& cfg);

/**
 * @brief Lateral offset of a station
 *
 * THE definition of the miter. Every offset column in the pipeline goes through
 * this function so that no caller can forget the miter_scale factor:
 *
 * @code
 *     position + normal * clamp(lateral, lateral_min, lateral_max) * miter_scale
 * @endcode
 *
 * Positive @p lateral is to the LEFT of travel.
 *
 * The clamp is the fold guard. On a straight and on any joint slack enough to
 * mitre without folding the bounds are kUnboundedLateral and this is exactly
 * `position + normal * lateral * miter_scale`; inside a fold it collapses the
 * offside profile onto the fold limit instead of pushing it back past the
 * neighbouring column. See Station::lateral_max.
 *
 * @param s       Station to offset from
 * @param lateral Signed lateral distance in metres, positive to the left
 * @return The offset point, in the same 2D local metres as the station
 */
[[nodiscard]] inline glm::dvec2 offset_point(const Station& s, double lateral) {
    // Written as min-of-max rather than std::clamp: an inverted bound pair is
    // undefined behaviour there, and this returns lateral_max instead.
    const double bounded = std::min(std::max(lateral, s.lateral_min), s.lateral_max);
    return s.position + s.normal * (bounded * s.miter_scale);
}

/**
 * @brief Arc-length sub-range of a centerline
 *
 * Used by the junction solver to trim ribbon ends back to the trim station, so
 * the corridor stops short of the junction polygon instead of overlapping it.
 *
 * Contract:
 *
 * - The range is clamped to [0, c.length()] and swapped if reversed. A range
 *   spanning less than one station gap yields an invalid Centerline.
 * - Endpoints falling between two stations are synthesised so that their offset
 *   column lands ON the untrimmed ribbon's edge, which is the straight chord the
 *   extruder draws between the two bracketing offset columns. That requires
 *   interpolating the miter VECTOR `normal * miter_scale`, not the normal and the
 *   scale separately: the two are only equivalent when the bracketing normals
 *   agree, and lerping them apart makes the trimmed arm WIDER than the corridor
 *   it was cut from. normal and miter_scale are recovered from the interpolated
 *   vector, position, curvature and the lateral bounds are interpolated linearly,
 *   and tangent is the band's own direction of travel. A synthesised station
 *   never has is_bevel set. Where a fold bound binds the offset is clamped, so
 *   the column lands on the clamped ribbon edge rather than the raw one.
 * - Arclengths are NOT rebased to zero. A slice keeps the source
 *   parameterisation, so the V texture coordinate of a trimmed ribbon is
 *   identical to that of the untrimmed one and re-trimming never shifts the
 *   texture.
 *
 * @param c              Source centerline
 * @param from_arclength Start of the range, metres along @p c
 * @param to_arclength   End of the range, metres along @p c
 * @return The sub-range, or an invalid Centerline when the range is empty
 */
[[nodiscard]] Centerline slice(const Centerline& c, double from_arclength, double to_arclength);

} // namespace stratum::osm::road
