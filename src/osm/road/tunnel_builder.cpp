/**
 * @file tunnel_builder.cpp
 * @brief Implementation of the tunnel portal builder
 *
 * Three problems, solved in this order.
 *
 * ### 1. Where the portal is
 *
 * The one thing that must not be assumed is that a tunnel starts where the
 * `tunnel=*` way starts. It usually does not: the way is tagged from the point
 * the surveyor called the tunnel, the elevation solver ramps the roadway down to
 * `terrain - ElevationConfig::tunnel_depth` over a grade-limited run, and the
 * road is still in the open for the first stretch of both. So the portal is
 * SEARCHED for, as the crossing of two curves already in hand:
 *
 *     cover(i) = terrain_at(station_i) - station_heights[i]
 *
 * cover is negative while the road is above ground and positive once it is
 * buried. Walking inward from each end, the first crossing of
 * TunnelConfig::min_portal_cover is the mouth. Station spacing on a straight is
 * ResampleConfig::max_spacing -- eight metres -- so the station pair only
 * BRACKETS the crossing; refine_crossing() then bisects inside the bracket
 * against the terrain sampler itself. Snapping to the nearer station, or even
 * lerping across the pair, puts the headwall metres clear of the slope it is
 * supposed to be set into.
 *
 * Walking inward from the ends, rather than scanning for the deepest point, is
 * what makes two of the three no-portal cases fall out for free: an edge already
 * buried at its first station gets no portal there (it is an interior slice of a
 * longer tunnel), and an edge with no crossing at all gets none anywhere. The
 * third, two mouths too close together to be two mouths, is an explicit test
 * applied to both hits before either produces geometry.
 *
 * ### 2. Making the opening a hole
 *
 * The headwall is a rectangle with an arch cut out of it, which is a polygon
 * with a hole, which usually means a triangulator. It does not here, because the
 * shape decomposes exactly: a left jamb panel, a right jamb panel, and one quad
 * strip between the arch ring and the top of the wall. That covers the arched
 * and the rectangular opening with the same loop, and it cannot produce the
 * failure a triangulator can -- a filled face across the opening, which is
 * indistinguishable from no portal at all because it also hides the road.
 *
 * ### 3. Winding
 *
 * `(x, y_2d) -> vec3(x, height, -y_2d)` flips handedness, so no fixed index
 * order is correct for both the outward headwall and the inward soffit. Every
 * quad here is therefore emitted through add_quad(), which is told the direction
 * the surface is meant to FACE and picks the index order from the sign of the
 * quad's own cross product against it. There is no per-surface winding rule to
 * get backwards, and the vertex normal is the resulting geometric normal rather
 * than the requested one, so a quad whose corners are not quite coplanar still
 * lights correctly.
 *
 * ### The bore
 *
 * TunnelConfig::emit_bore is ignored, per its own documentation. What building
 * it would need, beyond this file: the running surface is already swept by the
 * corridor for the whole edge, so the tube is the other three sides -- two walls
 * and a soffit swept along the same Centerline between the two portal
 * arclengths, on the same station frames, at the same heights plus the profile's
 * lateral bounds. That is a strip sweep with the profile replaced by a fixed
 * cross-section, so corridor.cpp's walk is the shape of it. The parts that are
 * genuinely absent are a decision about lighting a windowless interior and a
 * decision about whether the tube is worth its triangles when nothing outside
 * the mouth can see it.
 *
 * Everything here lives in stratum_core: no SDL, no ImGui, no rendering API.
 */

#include "osm/road/tunnel_builder.hpp"

#include "osm/road/corridor.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numbers>

namespace stratum::osm::road {

namespace {

// ============================================================================
// Tolerances
// ============================================================================

/// Squared length of the raw face cross product below which a quad is dropped
constexpr double kDegenerateCrossSq = 1e-16;

/// Lateral extents at or below this count as zero, metres
constexpr double kZeroWidth = 1e-6;

/**
 * @brief Shortest jamb worth springing an arch from, metres
 *
 * With a semicircular head the springing line sits at
 * `portal_height - half_width`. On a wide road and a low opening that is at or
 * below the carriageway, which is not an arch but a half-buried circle. Below
 * this the opening is rectangular instead, which is what a box portal on a dual
 * carriageway looks like anyway.
 */
constexpr double kMinSpringHeight = 0.25;

/// Fewest segments that make a semicircle read as a curve rather than a chamfer
constexpr int kMinArchSegments = 3;

/// Halvings used to narrow a bracketed cover crossing; see refine_crossing()
constexpr int kCrossingRefineIterations = 12;

/// Arclength span below which refine_crossing() stops halving, metres
constexpr double kCrossingRefineTolerance = 0.01;

// ============================================================================
// Helpers
// ============================================================================

/**
 * @brief The codebase-wide 2D-to-3D mapping, Y up
 *
 * (x, y_2d) -> (x, height, -y_2d). Matches corridor.cpp and mesh_builder.cpp.
 */
[[nodiscard]] inline glm::vec3 to_world(const glm::dvec2& p, double height) {
    return glm::vec3(static_cast<float>(p.x),
                     static_cast<float>(height),
                     static_cast<float>(-p.y));
}

/// True when every component is a real number
[[nodiscard]] inline bool is_finite2(const glm::dvec2& v) {
    return std::isfinite(v.x) && std::isfinite(v.y);
}

/**
 * @brief One point on the opening ring, in the portal face's own 2D frame
 *
 * `u` is lateral, positive to the LEFT of travel, matching Station::normal and
 * offset_point(). `v` is metres above the carriageway surface.
 */
struct FacePoint {
    double u = 0.0;
    double v = 0.0;
};

/**
 * @brief Where a portal sits along the edge
 *
 * Carries the interpolation the caller already did, so the station lookup and
 * the height lookup cannot disagree about which point on the edge is the mouth.
 */
struct PortalHit {
    bool found = false;
    double arclength = 0.0;     ///< in the centerline's own (un-rebased) parameterisation
    double surface = 0.0;       ///< world Y of the carriageway there
};

/**
 * @brief Narrow a bracketed cover crossing down against the real terrain
 *
 * The scan finds which pair of stations the mouth lies between, and lerping
 * across that pair is only as good as the assumption that the ground runs
 * straight between them. It does not: ResampleConfig::max_spacing is eight
 * metres on a straight, and a hillside that starts exactly halfway along such a
 * gap reads as starting at its beginning. On the test ramp that put the headwall
 * 3.4 m clear of the slope it is supposed to be set into.
 *
 * So the bracket is bisected against `terrain_at` itself. The station pair only
 * has to supply the invariant -- covered at one end, open at the other -- and a
 * dozen halvings take an eight metre gap under a centimetre, at a dozen terrain
 * samples per portal.
 *
 * Positions and heights are interpolated along the CHORD between the two
 * stations rather than along the fitted curve. Over one station gap the two
 * differ by less than ResampleConfig::max_deviation by construction, which is
 * five centimetres, well inside what this is correcting.
 *
 * @param a_pos,a_height,a_arc End of the bracket that is OPEN (cover below threshold)
 * @param b_pos,b_height,b_arc End of the bracket that is COVERED
 * @param threshold            Cover at which the mouth is declared
 * @param terrain_at           Terrain sampler
 * @param out_arclength        Refined arclength, only written on success
 * @param out_surface          Refined carriageway height, only written on success
 */
void refine_crossing(const glm::dvec2& a_pos, double a_height, double a_arc,
                     const glm::dvec2& b_pos, double b_height, double b_arc,
                     double threshold,
                     const std::function<float(double, double)>& terrain_at,
                     double& out_arclength,
                     double& out_surface) {
    double lo = 0.0;    // open
    double hi = 1.0;    // covered

    for (int iter = 0; iter < kCrossingRefineIterations; ++iter) {
        if (std::fabs(b_arc - a_arc) * (hi - lo) <= kCrossingRefineTolerance) {
            break;
        }
        const double mid = 0.5 * (lo + hi);
        const glm::dvec2 p = a_pos + (b_pos - a_pos) * mid;
        const double surface = a_height + (b_height - a_height) * mid;
        const double ground = static_cast<double>(terrain_at(p.x, p.y));
        if (!std::isfinite(ground)) {
            break;      // a hole in the sampler: keep the linear answer
        }
        if (ground - surface >= threshold) {
            hi = mid;
        } else {
            lo = mid;
        }
    }

    // hi is the shallowest point known to be covered, so the mouth is there: a
    // portal one bisection step INSIDE the hill is buried by a centimetre, one
    // step outside it stands in the open.
    out_arclength = a_arc + (b_arc - a_arc) * hi;
    out_surface = a_height + (b_height - a_height) * hi;
}

/**
 * @brief The station at an arbitrary arclength
 *
 * Delegates to slice() rather than interpolating a station here. A station is
 * not lerp-able field by field -- the miter VECTOR `normal * miter_scale` is what
 * interpolates, not the normal and the scale separately, per slice()'s contract
 * -- and duplicating that reasoning in a second file is how the two drift apart.
 *
 * @param cl         Source centerline; must be valid
 * @param arclength  Point of interest, in @p cl 's parameterisation
 * @param from_start True to take the last station of the leading slice, false to
 *                   take the first station of the trailing one. Only matters
 *                   across a bevel pair, where the two answers carry different
 *                   normals; taking the one on the side the portal faces keeps
 *                   the headwall square to the ribbon the driver sees.
 * @return The station at @p arclength, falling back to the nearer end station
 *         when the slice is too short to exist
 */
[[nodiscard]] Station station_at(const Centerline& cl, double arclength, bool from_start) {
    const double lo = cl.stations.front().arclength;
    const double hi = cl.stations.back().arclength;

    if (from_start) {
        const Centerline head = slice(cl, lo, arclength);
        if (head.is_valid()) {
            return head.stations.back();
        }
        const Centerline tail = slice(cl, arclength, hi);
        if (tail.is_valid()) {
            return tail.stations.front();
        }
        return cl.stations.front();
    }

    const Centerline tail = slice(cl, arclength, hi);
    if (tail.is_valid()) {
        return tail.stations.front();
    }
    const Centerline head = slice(cl, lo, arclength);
    if (head.is_valid()) {
        return head.stations.back();
    }
    return cl.stations.back();
}

/**
 * @brief First crossing into cover, walking forward from the start of the edge
 *
 * Returns nothing when the edge is ALREADY covered at its first station. That is
 * not a degenerate case to be tolerated, it is the common one: a tunnel long
 * enough to matter is split into several GraphEdges at its interior nodes, and
 * every edge but the first begins underground. A headwall there stands across
 * the middle of the bore.
 */
[[nodiscard]] PortalHit find_start_portal(const Centerline& cl,
                                          const std::vector<double>& cover,
                                          const std::vector<double>& arclength,
                                          const std::vector<float>& heights,
                                          double threshold,
                                          const std::function<float(double, double)>& terrain_at) {
    PortalHit hit;
    const size_t n = cover.size();
    if (n < 2 || cover.front() >= threshold) {
        return hit;
    }

    for (size_t i = 1; i < n; ++i) {
        if (cover[i] < threshold) {
            continue;
        }
        const size_t lo = i - 1;
        const double span = cover[i] - cover[lo];
        // cover[lo] < threshold <= cover[i], so span is strictly positive.
        const double t = (span > 0.0) ? std::clamp((threshold - cover[lo]) / span, 0.0, 1.0) : 1.0;

        hit.found = true;
        hit.arclength = arclength[lo] + (arclength[i] - arclength[lo]) * t;
        hit.surface = static_cast<double>(heights[lo])
                      + (static_cast<double>(heights[i]) - static_cast<double>(heights[lo])) * t;

        refine_crossing(cl.stations[lo].position, static_cast<double>(heights[lo]), arclength[lo],
                        cl.stations[i].position, static_cast<double>(heights[i]), arclength[i],
                        threshold, terrain_at, hit.arclength, hit.surface);
        return hit;
    }
    return hit;
}

/// Last crossing out of cover, walking backward from the end of the edge; see find_start_portal()
[[nodiscard]] PortalHit find_end_portal(const Centerline& cl,
                                        const std::vector<double>& cover,
                                        const std::vector<double>& arclength,
                                        const std::vector<float>& heights,
                                        double threshold,
                                        const std::function<float(double, double)>& terrain_at) {
    PortalHit hit;
    const size_t n = cover.size();
    if (n < 2 || cover.back() >= threshold) {
        return hit;
    }

    for (size_t i = n - 1; i-- > 0;) {
        if (cover[i] < threshold) {
            continue;
        }
        const size_t hi = i + 1;
        const double span = cover[i] - cover[hi];
        // cover[i] >= threshold > cover[hi], so span is strictly positive.
        const double t = (span > 0.0) ? std::clamp((cover[i] - threshold) / span, 0.0, 1.0) : 0.0;

        hit.found = true;
        hit.arclength = arclength[i] + (arclength[hi] - arclength[i]) * t;
        hit.surface = static_cast<double>(heights[i])
                      + (static_cast<double>(heights[hi]) - static_cast<double>(heights[i])) * t;

        // The OPEN end of the bracket goes first: refine_crossing() reads its
        // first triple as the side above ground.
        refine_crossing(cl.stations[hi].position, static_cast<double>(heights[hi]), arclength[hi],
                        cl.stations[i].position, static_cast<double>(heights[i]), arclength[i],
                        threshold, terrain_at, hit.arclength, hit.surface);
        return hit;
    }
    return hit;
}

/**
 * @brief The opening ring, from the left springing foot round to the right one
 *
 * The ring is OPEN: it deliberately omits the closing segment along the
 * carriageway, because that is where the road passes through and there is no
 * geometry there. Consumers walk consecutive pairs, so the omitted segment is
 * simply never visited.
 *
 * @param half_width    Half the clear opening, metres
 * @param height        Clear height above the carriageway, metres
 * @param arch_segments Segments across the semicircular head; below
 *                      kMinArchSegments the head is square
 * @return Ring points ordered from (-half_width, 0) to (+half_width, 0)
 */
[[nodiscard]] std::vector<FacePoint> build_opening_ring(double half_width,
                                                        double height,
                                                        int arch_segments) {
    std::vector<FacePoint> ring;

    const double springing = height - half_width;
    const bool arched = arch_segments >= kMinArchSegments && springing >= kMinSpringHeight;

    if (!arched) {
        ring.push_back(FacePoint{ -half_width, 0.0 });
        ring.push_back(FacePoint{ -half_width, height });
        ring.push_back(FacePoint{ half_width, height });
        ring.push_back(FacePoint{ half_width, 0.0 });
        return ring;
    }

    const int segments = arch_segments;
    ring.reserve(static_cast<size_t>(segments) + 3u);

    ring.push_back(FacePoint{ -half_width, 0.0 });
    for (int k = 0; k <= segments; ++k) {
        // pi at k = 0 (left springing) sweeping to 0 at k = segments (right one).
        const double theta =
            std::numbers::pi * (1.0 - static_cast<double>(k) / static_cast<double>(segments));
        ring.push_back(FacePoint{ half_width * std::cos(theta),
                                  springing + half_width * std::sin(theta) });
    }
    ring.push_back(FacePoint{ half_width, 0.0 });
    return ring;
}

// ============================================================================
// Mesh assembly
// ============================================================================

/**
 * @brief Accumulates portal quads into one Concrete mesh
 *
 * Exists to keep the orientation rule in exactly one place. Nothing else in this
 * file decides an index order.
 */
class PortalMesh {
public:
    /**
     * @brief Append one quad, oriented to face @p want
     *
     * The corners must be given in order around the quad; which way round does
     * not matter, since the order is chosen here. @p want need not be a unit
     * vector and need not be exactly perpendicular to the quad -- only its side
     * is read.
     *
     * @note Silently drops a quad whose area is below kDegenerateCrossSq, which
     *       is how the zero-width jamb panels of a wall with no surround
     *       disappear instead of producing null normals.
     */
    void add_quad(const glm::vec3& p0, const glm::vec2& uv0,
                  const glm::vec3& p1, const glm::vec2& uv1,
                  const glm::vec3& p2, const glm::vec2& uv2,
                  const glm::vec3& p3, const glm::vec2& uv3,
                  const glm::dvec3& want) {
        const glm::dvec3 a(p0);
        const glm::dvec3 b(p1);
        const glm::dvec3 c(p2);
        const glm::dvec3 d(p3);

        // Newell over both halves, so a slightly non-planar quad still yields the
        // normal of the surface rather than of one arbitrary half.
        const glm::dvec3 face = glm::cross(b - a, c - a) + glm::cross(c - a, d - a);
        const double area_sq = glm::dot(face, face);
        if (!(area_sq > kDegenerateCrossSq) || !std::isfinite(area_sq)) {
            return;
        }

        const bool flip = glm::dot(face, want) < 0.0;
        const glm::dvec3 oriented = flip ? -face : face;
        const glm::dvec3 unit = oriented / std::sqrt(area_sq);
        const glm::vec3 normal(static_cast<float>(unit.x),
                               static_cast<float>(unit.y),
                               static_cast<float>(unit.z));

        const uint32_t base = static_cast<uint32_t>(m_mesh.vertices.size());
        push_vertex(p0, uv0, normal);
        push_vertex(p1, uv1, normal);
        push_vertex(p2, uv2, normal);
        push_vertex(p3, uv3, normal);

        if (flip) {
            emit(base + 0u, base + 2u, base + 1u);
            emit(base + 0u, base + 3u, base + 2u);
        } else {
            emit(base + 0u, base + 1u, base + 2u);
            emit(base + 0u, base + 2u, base + 3u);
        }
    }

    /// Indices emitted so far; the caller reads it either side of a portal to tell whether one landed
    [[nodiscard]] size_t index_count() const { return m_mesh.indices.size(); }

    /// Tag every triangle Concrete, resolve bounds and tangents, and hand the mesh over
    [[nodiscard]] Mesh finish() {
        if (!m_mesh.indices.empty()) {
            m_mesh.submeshes.push_back(SubMesh{ 0u,
                                                static_cast<uint32_t>(m_mesh.indices.size()),
                                                MaterialId::Concrete });
            m_mesh.sort_submeshes_by_material();
            m_mesh.compute_bounds();
            m_mesh.compute_tangents();
        }
        return std::move(m_mesh);
    }

private:
    void push_vertex(const glm::vec3& p, const glm::vec2& uv, const glm::vec3& n) {
        Vertex v{};
        v.position = p;
        v.normal = n;
        v.uv = uv;
        v.color = glm::vec4(1.0f);
        m_mesh.vertices.push_back(v);
    }

    void emit(uint32_t i0, uint32_t i1, uint32_t i2) {
        m_mesh.indices.push_back(i0);
        m_mesh.indices.push_back(i1);
        m_mesh.indices.push_back(i2);
    }

    Mesh m_mesh;
};

/**
 * @brief Everything one portal needs, resolved from a PortalHit
 *
 * Separated from the emit step so the two-portal separation test can be applied
 * to both frames before either one produces geometry. Emitting the first and
 * then discovering the second overlaps it would leave a single headwall standing
 * in the middle of a short underpass.
 */
struct PortalFrame {
    glm::dvec2 center{0.0};     ///< opening centre in local 2D metres, on the ribbon
    glm::dvec2 lateral{0.0};    ///< unit, LEFT of travel, perpendicular to axis
    glm::dvec2 axis{0.0};       ///< unit, INTO the hillside
    double surface = 0.0;       ///< world Y of the carriageway at the mouth
    double arclength = 0.0;
    bool at_start = true;
};

/**
 * @brief Resolve the station at a hit into a portal frame
 *
 * The centre is taken through offset_point() so it lands on the mitred ribbon
 * the corridor actually emitted, while the face's lateral axis is the RAW
 * perpendicular of the tangent rather than the miter bisector. A miter bisector
 * is not perpendicular to the direction of travel at a bend, and a headwall
 * built on one is a parallelogram leaning along the road.
 */
[[nodiscard]] bool make_frame(const Centerline& cl,
                              const RoadProfile& profile,
                              const PortalHit& hit,
                              bool at_start,
                              PortalFrame& out) {
    const Station s = station_at(cl, hit.arclength, at_start);

    const double tangent_len_sq = glm::dot(s.tangent, s.tangent);
    if (!(tangent_len_sq > 0.0) || !std::isfinite(tangent_len_sq)) {
        return false;
    }
    const glm::dvec2 tangent = s.tangent / std::sqrt(tangent_len_sq);

    // The way is the centreline of the CARRIAGEWAY, not of the profile, so the
    // opening has to be centred on the profile's own middle or it sits off to
    // one side of a road with a footway down one edge only.
    const double centre_lateral = static_cast<double>(profile.left_edge_offset())
                                  - 0.5 * static_cast<double>(profile.total_width());

    out.center = offset_point(s, centre_lateral);
    out.lateral = glm::dvec2(-tangent.y, tangent.x);   // left of travel
    out.axis = at_start ? tangent : -tangent;          // into the hillside
    out.surface = hit.surface;
    out.arclength = hit.arclength;
    out.at_start = at_start;

    return is_finite2(out.center) && is_finite2(out.lateral) && is_finite2(out.axis)
           && std::isfinite(out.surface);
}

/**
 * @brief Emit one portal's headwall, side walls and soffit
 *
 * @param frame       Resolved placement
 * @param half_open   Half the CLEAR opening, metres
 * @param cfg         Portal dimensions
 * @param terrain_at  Terrain sampler, for the height the headwall must reach
 * @param mesh        Sink
 */
void emit_portal(const PortalFrame& frame,
                 double half_open,
                 const TunnelConfig& cfg,
                 const std::function<float(double, double)>& terrain_at,
                 PortalMesh& mesh) {
    const double thickness = std::max(0.0, static_cast<double>(cfg.wall_thickness));
    const double height = static_cast<double>(cfg.portal_height);
    const double half_outer = half_open + thickness;

    // Local frame -> world. `u` is lateral (positive left), `v` is metres above
    // the carriageway, `d` is metres into the hillside.
    const auto plan = [&](double u, double d) -> glm::dvec2 {
        return frame.center + frame.lateral * u + frame.axis * d;
    };
    const auto world = [&](double u, double v, double d) -> glm::vec3 {
        return to_world(plan(u, d), frame.surface + v);
    };
    // A direction in the face frame, mapped through the same (x, h, -y) flip.
    const auto world_dir = [&](double du, double dv, double dd) -> glm::dvec3 {
        const glm::dvec2 p = frame.lateral * du + frame.axis * dd;
        return glm::dvec3(p.x, dv, -p.y);
    };

    // ------------------------------------------------------------------------
    // How tall the wall has to be.
    //
    // The mouth sits where the road crosses the terrain, so the ground is at
    // road level on the centreline THERE and climbs going in and, in a cutting,
    // to either side. Sampling both planes and both edges finds the ground the
    // wall is actually set into, so the wall reaches it instead of leaving a
    // slot of daylight over the arch.
    // ------------------------------------------------------------------------
    const double min_top = height + thickness;
    double top = min_top;

    double ceiling = static_cast<double>(cfg.max_headwall_height);
    if (!std::isfinite(ceiling) || ceiling < min_top) {
        ceiling = min_top;
    }

    for (const double d : { 0.0, thickness }) {
        for (const double u : { -half_outer, 0.0, half_outer }) {
            const glm::dvec2 p = plan(u, d);
            const double ground = static_cast<double>(terrain_at(p.x, p.y));
            if (!std::isfinite(ground)) {
                continue;
            }
            top = std::max(top, ground - frame.surface);
        }
    }
    top = std::clamp(top, min_top, ceiling);

    const UVTiling tiling = uv_tiling(MaterialId::Concrete);
    const double u_scale = (tiling.u_metres > 0.0f) ? static_cast<double>(tiling.u_metres) : 1.0;
    const double v_scale = (tiling.v_metres > 0.0f) ? static_cast<double>(tiling.v_metres) : 1.0;

    // Texture coordinates are metres of real surface per repeat, per the frozen
    // UV Convention. U is measured from the wall's left edge so both jambs and
    // the arch strip share one continuous mapping across the face.
    const auto face_uv = [&](double u, double v) -> glm::vec2 {
        return glm::vec2(static_cast<float>((half_outer - u) / u_scale),
                         static_cast<float>(v / v_scale));
    };

    // The face is seen from outside the tunnel, which is the -axis side.
    const glm::dvec3 face_out = world_dir(0.0, 0.0, -1.0);

    const std::vector<FacePoint> ring =
        build_opening_ring(half_open, height, cfg.arch_segments);

    // Height at which each side of the OPENING ends: the top of the ring's own
    // vertical jamb segment, which is `portal_height` on a rectangular opening
    // and the springing line on an arched one. Every wall panel beside the
    // opening is split there, because that is where the geometry on its inner
    // side changes hands from the intrados to the strip over the arch. A panel
    // spanning the join in one quad meets two shorter edges and leaves a
    // T-junction, which welds and simplifies into a crack.
    const double v_left = ring.size() > 2u ? ring[1].v : height;
    const double v_right = ring.size() > 2u ? ring[ring.size() - 2u].v : height;

    /**
     * @brief One wall panel beside the opening, split at @p v_split
     *
     * @param u_inner Lateral edge against the opening
     * @param u_outer Lateral edge at the outside of the wall
     * @param v_split Height the panel is cut at
     * @param d       Depth of the face: 0 for the outward one, thickness for the rear
     * @param want    Outward direction the face must carry
     */
    const auto panel = [&](double u_inner, double u_outer, double v_split, double d,
                           const glm::dvec3& want) {
        const double bands[3] = { 0.0, std::clamp(v_split, 0.0, top), top };
        for (int b = 0; b < 2; ++b) {
            const double lo = bands[b];
            const double hi = bands[b + 1];
            if (hi - lo <= kZeroWidth) {
                continue;
            }
            mesh.add_quad(world(u_outer, lo, d), face_uv(u_outer, lo),
                          world(u_inner, lo, d), face_uv(u_inner, lo),
                          world(u_inner, hi, d), face_uv(u_inner, hi),
                          world(u_outer, hi, d), face_uv(u_outer, hi),
                          want);
        }
    };

    // ------------------------------------------------------------------------
    // Headwall, as two jamb panels plus a strip following the arch. Never a
    // filled quad: the opening has to be a hole, and a triangulator that closes
    // it hides the road as well as the tunnel.
    // ------------------------------------------------------------------------
    if (thickness > kZeroWidth) {
        panel(half_open, half_outer, v_right, 0.0, face_out);
        panel(-half_open, -half_outer, v_left, 0.0, face_out);
    }

    for (size_t k = 0; k + 1 < ring.size(); ++k) {
        const FacePoint& a = ring[k];
        const FacePoint& b = ring[k + 1];

        // The two jamb segments are vertical, so their span to the top of the
        // wall has no width. add_quad would drop them anyway; skipping is
        // cheaper and says why.
        if (std::fabs(b.u - a.u) <= kZeroWidth) {
            continue;
        }

        mesh.add_quad(world(a.u, a.v, 0.0), face_uv(a.u, a.v),
                      world(b.u, b.v, 0.0), face_uv(b.u, b.v),
                      world(b.u, top, 0.0), face_uv(b.u, top),
                      world(a.u, top, 0.0), face_uv(a.u, top),
                      face_out);
    }

    // ------------------------------------------------------------------------
    // Side walls and soffit: the ring swept `thickness` into the hillside, seen
    // from inside the opening. This is what gives the mouth a depth rather than
    // the look of a hole cut in a sheet of paper.
    // ------------------------------------------------------------------------
    if (thickness > kZeroWidth) {
        // Any interior point serves to orient the intrados; the centre of the
        // clear opening is the one that stays inside for both ring shapes.
        const FacePoint interior{ 0.0, height * 0.5 };

        double ring_arc = 0.0;
        for (size_t k = 0; k + 1 < ring.size(); ++k) {
            const FacePoint& a = ring[k];
            const FacePoint& b = ring[k + 1];

            const double du = b.u - a.u;
            const double dv = b.v - a.v;
            const double seg = std::sqrt(du * du + dv * dv);
            if (!(seg > kZeroWidth)) {
                continue;
            }

            const double mid_u = 0.5 * (a.u + b.u);
            const double mid_v = 0.5 * (a.v + b.v);
            const glm::dvec3 inward = world_dir(interior.u - mid_u, interior.v - mid_v, 0.0);

            const glm::vec2 uv_a0(static_cast<float>(ring_arc / u_scale), 0.0f);
            const glm::vec2 uv_b0(static_cast<float>((ring_arc + seg) / u_scale), 0.0f);
            const glm::vec2 uv_b1(static_cast<float>((ring_arc + seg) / u_scale),
                                  static_cast<float>(thickness / v_scale));
            const glm::vec2 uv_a1(static_cast<float>(ring_arc / u_scale),
                                  static_cast<float>(thickness / v_scale));

            mesh.add_quad(world(a.u, a.v, 0.0),       uv_a0,
                          world(b.u, b.v, 0.0),       uv_b0,
                          world(b.u, b.v, thickness), uv_b1,
                          world(a.u, a.v, thickness), uv_a1,
                          inward);

            ring_arc += seg;
        }
    }

    // ------------------------------------------------------------------------
    // Close the block.
    //
    // The mouth is placed where the ground is only TunnelConfig::min_portal_cover
    // above the carriageway, while the wall stands at least
    // portal_height + wall_thickness tall. So the headwall is always most of its
    // height PROUD of the hillside it is set into, and every face of it that is
    // not the outward one is a face a camera can reach. With the front and the
    // intrados alone the wall backface-culls to nothing from the hill side and
    // reads as a zero-thickness sheet edge-on.
    //
    // What follows makes the wall a closed solid -- the block
    // [-half_outer, half_outer] x [0, top] x [0, thickness] minus the opening
    // prism -- so its only boundary edges are the ones around the mouth, which is
    // the standard bridge_builder.cpp holds its own geometry to.
    // ------------------------------------------------------------------------
    if (thickness > kZeroWidth) {
        const glm::dvec3 back_out = world_dir(0.0, 0.0, 1.0);
        const glm::dvec3 up(0.0, 1.0, 0.0);
        const glm::dvec3 down(0.0, -1.0, 0.0);

        /// Metres across the wall and metres into the hillside, for a plan face
        const auto plan_uv = [&](double u, double d) -> glm::vec2 {
            return glm::vec2(static_cast<float>((half_outer - u) / u_scale),
                             static_cast<float>(d / v_scale));
        };
        /// Metres into the hillside and metres up, for a return face
        const auto side_uv = [&](double d, double v) -> glm::vec2 {
            return glm::vec2(static_cast<float>(d / u_scale),
                             static_cast<float>(v / v_scale));
        };

        // Rear face: the same two jambs and arch strip as the front, seen from
        // inside the hill.
        panel(half_open, half_outer, v_right, thickness, back_out);
        panel(-half_open, -half_outer, v_left, thickness, back_out);

        for (size_t k = 0; k + 1 < ring.size(); ++k) {
            const FacePoint& a = ring[k];
            const FacePoint& b = ring[k + 1];
            if (std::fabs(b.u - a.u) <= kZeroWidth) {
                continue;   // vertical jamb segment; no width to span
            }
            mesh.add_quad(world(a.u, a.v, thickness), face_uv(a.u, a.v),
                          world(b.u, b.v, thickness), face_uv(b.u, b.v),
                          world(b.u, top, thickness), face_uv(b.u, top),
                          world(a.u, top, thickness), face_uv(a.u, top),
                          back_out);
        }

        // Top cap, in the SAME columns the front and rear faces are cut into: one
        // per jamb and one per arch segment. A single full-width quad would meet
        // several shorter top edges and leave a row of T-junctions along the
        // crown, which is a crack under any vertex-welding or simplification
        // pass even though it renders closed today.
        const auto cap_column = [&](double u_lo, double u_hi) {
            mesh.add_quad(world(u_lo, top, 0.0),       plan_uv(u_lo, 0.0),
                          world(u_hi, top, 0.0),       plan_uv(u_hi, 0.0),
                          world(u_hi, top, thickness), plan_uv(u_hi, thickness),
                          world(u_lo, top, thickness), plan_uv(u_lo, thickness),
                          up);
        };

        cap_column(half_open, half_outer);
        cap_column(-half_outer, -half_open);
        for (size_t k = 0; k + 1 < ring.size(); ++k) {
            const double a_u = ring[k].u;
            const double b_u = ring[k + 1].u;
            if (std::fabs(b_u - a_u) <= kZeroWidth) {
                continue;
            }
            cap_column(std::min(a_u, b_u), std::max(a_u, b_u));
        }

        // The two returns, at the outer edges of the wall. Split at the same
        // heights as the panels they meet, for the same reason.
        const auto return_face = [&](double u, double v_split, const glm::dvec3& want) {
            const double bands[3] = { 0.0, std::clamp(v_split, 0.0, top), top };
            for (int b = 0; b < 2; ++b) {
                const double lo = bands[b];
                const double hi = bands[b + 1];
                if (hi - lo <= kZeroWidth) {
                    continue;
                }
                mesh.add_quad(world(u, lo, 0.0),       side_uv(0.0, lo),
                              world(u, lo, thickness), side_uv(thickness, lo),
                              world(u, hi, thickness), side_uv(thickness, hi),
                              world(u, hi, 0.0),       side_uv(0.0, hi),
                              want);
            }
        };

        return_face(half_outer, v_right, world_dir(1.0, 0.0, 0.0));
        return_face(-half_outer, v_left, world_dir(-1.0, 0.0, 0.0));

        // Undersides of the two jambs, at carriageway level. Buried wherever the
        // ground beside the mouth is at road level, and the only thing closing
        // the wall where it is not.
        mesh.add_quad(world(half_open, 0.0, 0.0),        plan_uv(half_open, 0.0),
                      world(half_outer, 0.0, 0.0),       plan_uv(half_outer, 0.0),
                      world(half_outer, 0.0, thickness), plan_uv(half_outer, thickness),
                      world(half_open, 0.0, thickness),  plan_uv(half_open, thickness),
                      down);

        mesh.add_quad(world(-half_outer, 0.0, 0.0),       plan_uv(-half_outer, 0.0),
                      world(-half_open, 0.0, 0.0),        plan_uv(-half_open, 0.0),
                      world(-half_open, 0.0, thickness),  plan_uv(-half_open, thickness),
                      world(-half_outer, 0.0, thickness), plan_uv(-half_outer, thickness),
                      down);
    }
}

} // namespace

// ============================================================================
// Construction
// ============================================================================

/**
 * @brief Frame the openings where a tunnel edge enters and leaves the ground
 *
 * Invariant: geometry and footprints agree exactly. A footprint is appended if
 * and only if the matching portal's geometry was emitted, in the same order, so
 * the terrain carve can never open a mouth in a hillside that has no headwall in
 * it.
 */
Mesh build_tunnel_portals(const GraphEdge& edge,
                          const Centerline& cl,
                          const RoadProfile& profile,
                          const std::vector<float>& station_heights,
                          const std::function<float(double, double)>& terrain_at,
                          const TunnelConfig& cfg,
                          std::vector<TunnelPortalFootprint>* footprints) {
    if (footprints != nullptr) {
        footprints->clear();
    }

    if (!edge.is_tunnel || !cfg.emit_portals) {
        return Mesh{};
    }
    if (!cl.is_valid() || !profile.is_valid()) {
        return Mesh{};
    }
    if (station_heights.size() != cl.stations.size()) {
        // Not an error worth a warning at every tunnel: the caller degrades to a
        // flat corridor on the same condition, and a portal at a guessed height
        // is a wall standing at random in the landscape.
        return Mesh{};
    }
    if (!terrain_at) {
        // Documented: without the terrain an opening in a hillside and an opening
        // in open air are indistinguishable.
        return Mesh{};
    }

    const size_t n = cl.stations.size();

    // ------------------------------------------------------------------------
    // Cover profile. One terrain sample per station, reused by both searches.
    // ------------------------------------------------------------------------
    std::vector<double> cover(n, 0.0);
    std::vector<double> arclength(n, 0.0);

    for (size_t i = 0; i < n; ++i) {
        const glm::dvec2& p = cl.stations[i].position;
        const double ground = static_cast<double>(terrain_at(p.x, p.y));
        const double surface = static_cast<double>(station_heights[i]);
        if (!std::isfinite(ground) || !std::isfinite(surface)) {
            // A hole in the sampler reads as "not buried", which costs a portal
            // rather than inventing one.
            cover[i] = -std::numeric_limits<double>::max();
        } else {
            cover[i] = ground - surface;
        }
        arclength[i] = cl.stations[i].arclength;
    }

    const double threshold = std::max(0.0, static_cast<double>(cfg.min_portal_cover));

    const PortalHit start =
        find_start_portal(cl, cover, arclength, station_heights, threshold, terrain_at);
    const PortalHit end =
        find_end_portal(cl, cover, arclength, station_heights, threshold, terrain_at);

    if (!start.found && !end.found) {
        return Mesh{};      // never goes under: a covered way, or bad data
    }

    // ------------------------------------------------------------------------
    // Opening size, and the short-tunnel rule.
    // ------------------------------------------------------------------------
    const double margin = std::max(0.0, static_cast<double>(cfg.portal_width_margin));
    const double half_open = 0.5 * static_cast<double>(profile.total_width()) + margin;
    if (!(half_open > kZeroWidth) || !std::isfinite(half_open)) {
        return Mesh{};
    }
    if (!(cfg.portal_height > 0.0f) || !std::isfinite(cfg.portal_height)) {
        return Mesh{};
    }

    const double thickness = std::max(0.0, static_cast<double>(cfg.wall_thickness));
    const double opening_span = 2.0 * (half_open + thickness);

    if (start.found && end.found) {
        const double covered_length = end.arclength - start.arclength;
        if (covered_length < 2.0 * opening_span) {
            // Two headwalls this close interpenetrate and read as a box, not as
            // two mouths. An unframed underpass is the better failure.
            spdlog::debug("build_tunnel_portals: way {} buried for only {:.1f} m against a "
                          "{:.1f} m opening; emitting no portals",
                          edge.source_way, std::max(0.0, covered_length), opening_span);
            return Mesh{};
        }
    }

    // ------------------------------------------------------------------------
    // Resolve both frames before emitting either, so the pair is accepted or
    // rejected as a pair.
    // ------------------------------------------------------------------------
    PortalFrame frames[2];
    bool frame_ok[2] = { false, false };

    if (start.found) {
        frame_ok[0] = make_frame(cl, profile, start, true, frames[0]);
    }
    if (end.found) {
        frame_ok[1] = make_frame(cl, profile, end, false, frames[1]);
    }

    PortalMesh mesh;

    for (int slot = 0; slot < 2; ++slot) {
        if (!frame_ok[slot]) {
            continue;
        }

        const size_t before = mesh.index_count();
        emit_portal(frames[slot], half_open, cfg, terrain_at, mesh);

        // A frame that resolved but produced nothing -- a profile that welded
        // down to no width, a wall of zero thickness with a zero-height opening
        // -- must not leave a footprint behind. The terrain would then cut a
        // notch for a headwall that is not there.
        if (mesh.index_count() == before) {
            continue;
        }

        if (footprints != nullptr) {
            TunnelPortalFootprint fp;
            fp.center = frames[slot].center;
            fp.axis = frames[slot].axis;
            fp.half_width = half_open + thickness;
            fp.depth = std::max(thickness, std::max(0.0, static_cast<double>(cfg.portal_cut_depth)));
            fp.surface_height = static_cast<float>(frames[slot].surface);
            fp.crown_height = static_cast<float>(frames[slot].surface
                                                 + static_cast<double>(cfg.portal_height));
            fp.at_start = frames[slot].at_start;
            footprints->push_back(fp);
        }
    }

    if (mesh.index_count() == 0) {
        if (footprints != nullptr) {
            footprints->clear();
        }
        return Mesh{};
    }

    return mesh.finish();
}

} // namespace stratum::osm::road
