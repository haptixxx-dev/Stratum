/**
 * @file junction_special.cpp
 * @brief Roundabout rings, degree-2 profile tapers, and dead-end caps
 *
 * The three cases the trim-and-fillet solver cannot express, implemented on top
 * of four shared primitives that live in the anonymous namespace below:
 *
 *   1. SectionModel  -- a RoadProfile flattened into boundary laterals plus
 *      boundary heights, optionally mirrored into the frame of whichever
 *      direction the caller is travelling in. Every lateral in this file goes
 *      through it, so no function has to remember which way an arm points.
 *   2. MeshAccum     -- vertex/index/material accumulation with area-weighted
 *      geometric normals and degenerate-triangle rejection, matching
 *      corridor.cpp exactly so the two producers cannot disagree.
 *   3. emit_sweep()  -- bands between consecutive cross-sections, with the ONE
 *      winding pattern corridor.cpp uses. Columns are ordered left-of-travel to
 *      right-of-travel and the pattern is never mirrored; getting that ordering
 *      right at the call site is what makes every normal come out correct.
 *   4. fan()         -- a star-shaped ring triangulated about an interior point,
 *      which is all the island top and the dead-end discs need. No earcut.
 *
 * ### Watertightness
 *
 * The failure mode this file exists to avoid is a crack where its geometry meets
 * a ribbon. Every cap therefore takes its weld edge from the ARM'S OWN geometry
 * -- offset_point() on the arm's end station, at the arm's own laterals -- and
 * never from an idealised circle or an axis-aligned box. Where a disc would
 * otherwise overlap the ribbon it feeds, the overlapping lobe is notched out
 * along the ribbon's real offset edge rather than left to z-fight.
 *
 * Everything here lives in stratum_core: no SDL, no ImGui, no rendering API.
 * Clipper2 is deliberately NOT used; junction_curb.cpp is its only consumer and
 * every ring here is analytic.
 */

#include "osm/road/junction_special.hpp"

#include "osm/road/corridor.hpp"

#include <mapbox/earcut.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

// Earcut reads a point through this trait rather than a fixed vector type, so
// glm::dvec2 rings can be triangulated with no intermediate copy.
namespace mapbox::util {
template <>
struct nth<0, glm::dvec2> {
    inline static double get(const glm::dvec2& p) { return p.x; }
};
template <>
struct nth<1, glm::dvec2> {
    inline static double get(const glm::dvec2& p) { return p.y; }
};
} // namespace mapbox::util

namespace stratum::osm::road {

namespace {

// ============================================================================
// Tolerances
// ============================================================================

/// Squared length of the raw face cross product below which a triangle is dropped
constexpr double kDegenerateCrossSq = 1e-16;

/// Consecutive ring points closer than this are collapsed, metres
constexpr double kWeldEpsilon = 1e-6;

/// Widths and height differences at or below this count as zero, metres
constexpr double kZeroExtent = 1e-6;

/// Two profiles agreeing to within this on every width and height need no taper, metres
constexpr double kProfileMatchEpsilon = 1e-3;

/// Vertices around a full turn for an analytic arc: matches RoundaboutConfig::segments
constexpr int kSegmentsPerTurn = 48;

/**
 * @brief Curb dimensions of a raised roundabout island, metres
 *
 * RoundaboutConfig carries no curb fields and is frozen, so these mirror the
 * CurbRingConfig defaults. An island curb that disagreed with the junction curb
 * ring would read as a different curb on the same intersection.
 */
constexpr double kIslandCurbHeight = 0.15;
constexpr double kIslandCurbBatter = 0.02;

constexpr double kPi = 3.14159265358979323846;

// ============================================================================
// Small geometry helpers
// ============================================================================

/**
 * @brief The codebase-wide 2D-to-3D mapping, Y up
 *
 * (x, y_2d) -> (x, height, -y_2d). Identical to corridor.cpp and
 * triangulate_junction(); changing it here would put caps on a different plane
 * from the ribbons they weld to.
 */
[[nodiscard]] inline glm::vec3 to_world(const glm::dvec2& p, double height) {
    return glm::vec3(static_cast<float>(p.x),
                     static_cast<float>(height),
                     static_cast<float>(-p.y));
}

/// 2D cross product, positive when b turns left of a
[[nodiscard]] inline double cross2(const glm::dvec2& a, const glm::dvec2& b) {
    return a.x * b.y - a.y * b.x;
}

/// Unit left normal of a direction: the 2D quarter turn counter-clockwise
[[nodiscard]] inline glm::dvec2 left_of(const glm::dvec2& d) {
    return glm::dvec2(-d.y, d.x);
}

/// Squared distance, so callers can compare against a squared epsilon
[[nodiscard]] inline double dist_sq(const glm::dvec2& a, const glm::dvec2& b) {
    const glm::dvec2 d = a - b;
    return glm::dot(d, d);
}

/// Twice the signed area of a closed ring; positive when counter-clockwise
[[nodiscard]] double ring_signed_area2(const std::vector<glm::dvec2>& ring) {
    double sum = 0.0;
    const size_t n = ring.size();
    for (size_t i = 0; i < n; ++i) {
        sum += cross2(ring[i], ring[(i + 1) % n]);
    }
    return sum;
}

/// Append unless coincident with the ring's current back
void push_unique(std::vector<glm::dvec2>& ring, const glm::dvec2& p) {
    if (!ring.empty() && dist_sq(ring.back(), p) <= kWeldEpsilon * kWeldEpsilon) {
        return;
    }
    ring.push_back(p);
}

/// Drop trailing points coincident with the front, so the ring closes implicitly
void close_ring(std::vector<glm::dvec2>& ring) {
    while (ring.size() >= 2 &&
           dist_sq(ring.back(), ring.front()) <= kWeldEpsilon * kWeldEpsilon) {
        ring.pop_back();
    }
}

/// Angle of @p v in the right-handed basis (@p ax, @p ay), radians in [-pi, pi]
[[nodiscard]] inline double angle_in_basis(const glm::dvec2& v,
                                           const glm::dvec2& ax,
                                           const glm::dvec2& ay) {
    return std::atan2(glm::dot(v, ay), glm::dot(v, ax));
}

// ============================================================================
// Mesh accumulation
// ============================================================================

/**
 * @brief Vertex, index, material and normal bookkeeping for one emitted cap
 *
 * Invariant: every triangle handed to triangle() that survives the degeneracy
 * test contributes its area-weighted face normal to all three of its vertices,
 * and the submesh ranges tile the whole index buffer. take() resolves the
 * normals, so no vertex keeps the placeholder +Y unless no surviving triangle
 * references it.
 *
 * Deliberately the same shape as the accumulation inside build_corridor(): a cap
 * and the ribbon it welds to must shade identically at their shared edge.
 */
class MeshAccum {
public:
    /// Add one vertex; returns its index
    uint32_t vertex(const glm::dvec2& p, double height, const glm::vec2& uv) {
        Vertex v{};
        v.position = to_world(p, height);
        v.normal = glm::vec3(0.0f, 1.0f, 0.0f);
        v.uv = uv;
        v.color = glm::vec4(1.0f);
        m_mesh.vertices.push_back(v);
        m_normals.emplace_back(0.0);
        return static_cast<uint32_t>(m_mesh.vertices.size() - 1u);
    }

    /// Emit a triangle under the current material, dropping zero-area ones
    void triangle(uint32_t i0, uint32_t i1, uint32_t i2) {
        if (i0 == i1 || i1 == i2 || i0 == i2) {
            return;
        }
        const glm::dvec3 p0(m_mesh.vertices[i0].position);
        const glm::dvec3 p1(m_mesh.vertices[i1].position);
        const glm::dvec3 p2(m_mesh.vertices[i2].position);

        // Unnormalised, so the per-vertex accumulation is area weighted.
        const glm::dvec3 face = glm::cross(p1 - p0, p2 - p0);
        if (glm::dot(face, face) <= kDegenerateCrossSq) {
            return;
        }

        m_mesh.indices.push_back(i0);
        m_mesh.indices.push_back(i1);
        m_mesh.indices.push_back(i2);

        m_normals[i0] += face;
        m_normals[i1] += face;
        m_normals[i2] += face;
    }

    /// Open a submesh range for @p m; a no-op when @p m is already current
    void material(MaterialId m) {
        if (m_open && m_current == m) {
            return;
        }
        close_range();
        m_current = m;
        m_range_start = static_cast<uint32_t>(m_mesh.indices.size());
        m_open = true;
    }

    [[nodiscard]] bool empty() const { return m_mesh.indices.empty(); }

    /// Resolve normals, close the last range, and hand over the finished mesh
    [[nodiscard]] Mesh take() {
        close_range();

        for (size_t i = 0; i < m_mesh.vertices.size(); ++i) {
            const glm::dvec3& n = m_normals[i];
            const double len_sq = glm::dot(n, n);
            if (len_sq > 0.0) {
                const glm::dvec3 unit = n / std::sqrt(len_sq);
                m_mesh.vertices[i].normal = glm::vec3(static_cast<float>(unit.x),
                                                      static_cast<float>(unit.y),
                                                      static_cast<float>(unit.z));
            }
            // else: referenced by no surviving triangle; the +Y placeholder is
            // never sampled.
        }

        if (m_mesh.indices.empty()) {
            return Mesh{};
        }

        m_mesh.sort_submeshes_by_material();
        m_mesh.compute_bounds();
        m_mesh.compute_tangents();
        return std::move(m_mesh);
    }

private:
    void close_range() {
        if (!m_open) {
            return;
        }
        const uint32_t added = static_cast<uint32_t>(m_mesh.indices.size()) - m_range_start;
        if (added > 0u) {
            m_mesh.submeshes.push_back(SubMesh{ m_range_start, added, m_current });
        }
        m_open = false;
    }

    Mesh m_mesh;
    std::vector<glm::dvec3> m_normals;
    MaterialId m_current = MaterialId::Default;
    uint32_t m_range_start = 0u;
    bool m_open = false;
};

// ============================================================================
// Sweeps
// ============================================================================

/**
 * @brief One cross-section of a sweep
 *
 * `pos` and `height` are parallel and hold n+1 strip BOUNDARIES, ordered
 * LEFT-OF-TRAVEL to RIGHT-OF-TRAVEL for the direction that runs from this
 * column to the next one. That ordering is the whole contract: emit_sweep()
 * applies one fixed winding pattern and never mirrors it, exactly as
 * build_corridor() does, so a column list in the wrong order comes out
 * inside-out rather than merely mis-textured.
 */
struct SweepColumn {
    std::vector<glm::dvec2> pos;     ///< n+1 boundary points, left to right
    std::vector<double>     height;  ///< n+1 heights above the sweep's base
    double                  v = 0.0; ///< metres travelled along the sweep, for V
};

/**
 * @brief Band a strip list between consecutive cross-sections
 *
 * The pattern, with L the strip's left column and R its right column, is the one
 * frozen in build_corridor():
 *
 * @code
 *     (L_i, R_i, R_{i+1})   and   (L_i, R_{i+1}, L_{i+1})
 * @endcode
 *
 * UVs follow the plan's UV Convention in metres. A strip flagged in
 * @p vertical_face has its U running UP the face from the lower edge; every
 * other strip has U running across it from its left boundary. V is the column's
 * own travelled distance.
 *
 * Strips are never welded to their neighbours: U restarts at each boundary and
 * adjacent strips usually differ in material, so a shared vertex could not carry
 * both. This matches build_corridor() and keeps a curb crisp.
 *
 * @param acc         Receives the geometry
 * @param materials   One per strip, size n
 * @param vertical_face One per strip, size n; non-zero means U runs up the face
 * @param cols        At least two columns, each with n+1 boundaries
 * @param base_height World Y the column heights are measured above
 */
void emit_sweep(MeshAccum& acc,
                const std::vector<MaterialId>& materials,
                const std::vector<uint8_t>& vertical_face,
                const std::vector<SweepColumn>& cols,
                double base_height) {
    if (cols.size() < 2 || materials.empty()) {
        return;
    }
    const size_t strip_count = materials.size();
    if (vertical_face.size() != strip_count) {
        return;
    }
    for (const SweepColumn& c : cols) {
        if (c.pos.size() != strip_count + 1u || c.height.size() != strip_count + 1u) {
            return;
        }
    }

    std::vector<uint32_t> left;
    std::vector<uint32_t> right;

    for (size_t k = 0; k < strip_count; ++k) {
        // A strip with no lateral extent and no height step anywhere along the
        // sweep would emit nothing but unreferenced vertices.
        bool has_extent = false;
        for (const SweepColumn& c : cols) {
            if (dist_sq(c.pos[k], c.pos[k + 1]) > kZeroExtent * kZeroExtent ||
                std::fabs(c.height[k] - c.height[k + 1]) > kZeroExtent) {
                has_extent = true;
                break;
            }
        }
        if (!has_extent) {
            continue;
        }

        const UVTiling tiling = uv_tiling(materials[k]);
        const double u_scale = (tiling.u_metres > 0.0f) ? static_cast<double>(tiling.u_metres) : 1.0;
        const double v_scale = (tiling.v_metres > 0.0f) ? static_cast<double>(tiling.v_metres) : 1.0;
        const bool face = vertical_face[k] != 0u;

        left.clear();
        right.clear();
        left.reserve(cols.size());
        right.reserve(cols.size());

        for (const SweepColumn& c : cols) {
            const double hl = c.height[k];
            const double hr = c.height[k + 1];

            double ul = 0.0;
            double ur = 0.0;
            if (face) {
                const double lower = std::min(hl, hr);
                ul = (hl - lower) / u_scale;
                ur = (hr - lower) / u_scale;
            } else {
                ur = std::sqrt(dist_sq(c.pos[k], c.pos[k + 1])) / u_scale;
            }
            const float v = static_cast<float>(c.v / v_scale);

            left.push_back(acc.vertex(c.pos[k], base_height + hl,
                                      glm::vec2(static_cast<float>(ul), v)));
            right.push_back(acc.vertex(c.pos[k + 1], base_height + hr,
                                       glm::vec2(static_cast<float>(ur), v)));
        }

        acc.material(materials[k]);
        for (size_t i = 0; i + 1 < cols.size(); ++i) {
            acc.triangle(left[i], right[i], right[i + 1]);
            acc.triangle(left[i], right[i + 1], left[i + 1]);
        }
    }
}

/**
 * @brief Triangulate a closed ring by ear clipping
 *
 * Earcut rather than a triangle fan, because two of the three shapes this file
 * fills are not star-shaped about any point. A turning circle is a disc with a
 * slot cut out of it along the ribbon it caps, which is a horseshoe: no interior
 * point sees both lobes, and a fan about the node emits a reversed triangle down
 * each leg of the slot.
 *
 * Winding is normalised per triangle rather than trusted from earcut, so a ring
 * handed in clockwise still comes out with +Y normals. A counter-clockwise 2D
 * triangle maps through (x, y_2d) -> (x, height, -y_2d) to a counter-clockwise
 * front face with a +Y normal, which is what the renderer culls against.
 *
 * UVs are planar-projected about @p uv_origin, per the plan's UV Convention for
 * a surface with no single direction of travel.
 *
 * @param acc       Receives the geometry
 * @param ring      Closed ring, first point NOT repeated
 * @param uv_origin Origin of the planar UV projection
 * @param height    World Y of the surface
 * @param material  Material slot for the fill
 */
void triangulate_ring(MeshAccum& acc,
                      const std::vector<glm::dvec2>& ring,
                      const glm::dvec2& uv_origin,
                      double height,
                      MaterialId material) {
    if (ring.size() < 3) {
        return;
    }
    const UVTiling tiling = uv_tiling(material);
    const double u_scale = (tiling.u_metres > 0.0f) ? static_cast<double>(tiling.u_metres) : 1.0;
    const double v_scale = (tiling.v_metres > 0.0f) ? static_cast<double>(tiling.v_metres) : 1.0;

    const std::vector<std::vector<glm::dvec2>> polygon{ ring };
    const std::vector<uint32_t> tris = mapbox::earcut<uint32_t>(polygon);
    if (tris.empty()) {
        return;
    }

    std::vector<uint32_t> rim;
    rim.reserve(ring.size());
    for (const glm::dvec2& p : ring) {
        rim.push_back(acc.vertex(p, height,
                                 glm::vec2(static_cast<float>((p.x - uv_origin.x) / u_scale),
                                           static_cast<float>((p.y - uv_origin.y) / v_scale))));
    }

    acc.material(material);
    for (size_t t = 0; t + 2 < tris.size(); t += 3) {
        const uint32_t a = tris[t];
        const uint32_t b = tris[t + 1];
        const uint32_t c = tris[t + 2];
        if (a >= ring.size() || b >= ring.size() || c >= ring.size()) {
            continue;
        }
        // Ear clipping preserves the input winding; normalise here so neither
        // caller has to know which way round its ring came out.
        const double area2 = cross2(ring[b] - ring[a], ring[c] - ring[a]);
        if (area2 >= 0.0) {
            acc.triangle(rim[a], rim[b], rim[c]);
        } else {
            acc.triangle(rim[a], rim[c], rim[b]);
        }
    }
}

// ============================================================================
// Cross-section model
// ============================================================================

/**
 * @brief A RoadProfile flattened into boundaries, in a caller-chosen frame
 *
 * `lateral` is DESCENDING -- positive is left of travel, so walking the profile
 * left to right walks the coordinate downwards -- and `height` is the profile
 * height at the same boundary. RoadProfile::is_valid() guarantees adjacent
 * strips agree at their shared boundary, so one height per boundary is well
 * defined and continuity holds by construction.
 *
 * `mirrored` records whether the model was flipped out of the edge's own
 * direction of travel. A lateral in this model maps back to an edge lateral, and
 * therefore to offset_point(), through edge_lateral().
 */
struct SectionModel {
    std::vector<double>     lateral;   ///< n+1 boundaries, descending
    std::vector<double>     height;    ///< n+1 heights above the carriageway surface
    std::vector<StripKind>  kind;      ///< n strips
    std::vector<MaterialId> material;  ///< n strips
    bool mirrored = false;

    [[nodiscard]] size_t strips() const { return kind.size(); }
    [[nodiscard]] bool empty() const { return kind.empty(); }

    /// Convert a lateral in this frame back into the owning edge's frame
    [[nodiscard]] double edge_lateral(double l) const { return mirrored ? -l : l; }
};

/**
 * @brief Flatten a profile, optionally mirroring it into the opposite frame
 *
 * @param p       Cross-section to flatten; an invalid one yields an empty model
 * @param mirror  True when the caller travels AGAINST the edge's own direction,
 *                so left and right swap. Negating the laterals reverses their
 *                ordering, and the arrays are reversed to keep them descending.
 */
[[nodiscard]] SectionModel model_from_profile(const RoadProfile& p, bool mirror) {
    SectionModel m;
    if (!p.is_valid()) {
        return m;
    }

    m.mirrored = mirror;
    m.lateral.reserve(p.strips.size() + 1u);
    m.height.reserve(p.strips.size() + 1u);
    m.kind.reserve(p.strips.size());
    m.material.reserve(p.strips.size());

    double lat = static_cast<double>(p.left_edge_offset());
    m.lateral.push_back(lat);
    m.height.push_back(static_cast<double>(p.strips.front().height_left));

    for (const Strip& s : p.strips) {
        lat -= static_cast<double>(s.width);
        m.lateral.push_back(lat);
        m.height.push_back(static_cast<double>(s.height_right));
        m.kind.push_back(s.kind);
        m.material.push_back(s.material);
    }

    if (mirror) {
        for (double& l : m.lateral) {
            l = -l;
        }
        std::reverse(m.lateral.begin(), m.lateral.end());
        std::reverse(m.height.begin(), m.height.end());
        std::reverse(m.kind.begin(), m.kind.end());
        std::reverse(m.material.begin(), m.material.end());
    }
    return m;
}

/**
 * @brief Index range of the carriageway envelope: first Lane-or-Median to last
 *
 * The same span RoadProfile::left_edge_offset() centres on zero, and therefore
 * the same definition as ArmRef::carriageway_half. Strips between the two ends
 * count towards the envelope even when they are neither kind, because a gutter
 * or a raised median inside a dual carriageway is part of it.
 *
 * @param m         Flattened cross-section
 * @param out_first First strip index of the envelope
 * @param out_last  Last strip index of the envelope, inclusive
 * @return false when the profile holds no Lane and no Median (informational; the
 *         outputs are always written), in which case
 *         left_edge_offset() centres the whole profile and the envelope is all
 *         of it
 */
bool carriageway_span(const SectionModel& m, size_t& out_first, size_t& out_last) {
    bool found = false;
    for (size_t i = 0; i < m.strips(); ++i) {
        if (m.kind[i] == StripKind::Lane || m.kind[i] == StripKind::Median) {
            if (!found) {
                out_first = i;
                found = true;
            }
            out_last = i;
        }
    }
    if (!found) {
        out_first = 0;
        out_last = m.strips() > 0 ? m.strips() - 1u : 0u;
    }
    return found;
}

/**
 * @brief Half the carriageway envelope, metres
 *
 * The carriageway occupies lateral [-carriageway_half, +carriageway_half]
 * exactly, because left_edge_offset() centres the envelope on zero. Symmetric by
 * construction, which is why every disc cap in this file needs only one
 * half-width rather than a left and a right one.
 */
[[nodiscard]] double carriageway_half(const SectionModel& m) {
    if (m.empty()) {
        return 0.0;
    }
    size_t first = 0;
    size_t last = 0;
    carriageway_span(m, first, last);
    return 0.5 * (m.lateral[first] - m.lateral[last + 1u]);
}

/**
 * @brief Half the run of strips lying ON the carriageway plane, metres
 *
 * The carriageway envelope, widened outward on each side for as long as the
 * strips stay coplanar with it, and then symmetrised by taking the larger of the
 * two sides. Every kerbed profile puts a Gutter at height 0 between its outer
 * lane and its curb face, and a shoulder, cycle lane or parking lane is coplanar
 * too, so the surface a flat disc cap would overlap reaches well past
 * carriageway_half().
 *
 * Symmetric because the disc caps notch themselves with one half-width, and
 * taking the LARGER side only ever cuts more disc away, which is the safe
 * direction: an overlap is a z-fight, a shortfall is only a slightly smaller
 * bulb.
 *
 * @param m Flattened cross-section
 * @return Half the coplanar span; never less than carriageway_half()
 */
[[nodiscard]] double coplanar_half(const SectionModel& m) {
    if (m.empty()) {
        return 0.0;
    }
    size_t first = 0;
    size_t last = 0;
    carriageway_span(m, first, last);

    double left = m.lateral[first];
    for (size_t i = first; i-- > 0;) {
        if (std::fabs(m.height[i]) > kZeroExtent) {
            break;
        }
        left = m.lateral[i];
    }

    double right = -m.lateral[last + 1u];
    for (size_t i = last + 1u; i < m.strips(); ++i) {
        if (std::fabs(m.height[i + 1u]) > kZeroExtent) {
            break;
        }
        right = -m.lateral[i + 1u];
    }

    return std::max(0.5 * (m.lateral[first] - m.lateral[last + 1u]), std::max(left, right));
}

/// Surface material of the carriageway: the first Lane strip's, else Asphalt
[[nodiscard]] MaterialId lane_material(const SectionModel& m) {
    for (size_t i = 0; i < m.strips(); ++i) {
        if (m.kind[i] == StripKind::Lane) {
            return m.material[i];
        }
    }
    return MaterialId::Asphalt;
}

// ============================================================================
// Least-squares circle fit
// ============================================================================

/**
 * @brief Weighted algebraic circle fit over a point set
 *
 * Minimises `sum w_i * (|p_i - c|^2 - r^2)^2` in the linearised (Kasa)
 * parameterisation, which reduces to a 3x3 normal system and has a closed form.
 * The points are shifted onto their weighted centroid first, so the system stays
 * well conditioned at city-scale local coordinates.
 *
 * The fit is used rather than an average of the loop's NODE positions because
 * nodes cluster wherever a mapper split the way -- typically all on one side of
 * a roundabout, at its approaches -- and their mean is pulled towards that side.
 * Weights are per-station arclength shares for the same reason at station level:
 * a curvature-adaptive resampling puts more stations in the tighter part of an
 * oval.
 *
 * @param pts        Sample positions, at least three
 * @param weights    Parallel to @p pts; non-positive entries are treated as zero
 * @param out_center Fitted centre, 2D local metres
 * @param out_radius Fitted radius, metres
 * @return false when the system is singular -- fewer than three points, zero
 *         total weight, or a nearly collinear sample -- in which case the
 *         outputs are untouched and the caller falls back to the centroid
 */
[[nodiscard]] bool fit_circle(const std::vector<glm::dvec2>& pts,
                              const std::vector<double>& weights,
                              glm::dvec2& out_center,
                              double& out_radius) {
    if (pts.size() < 3 || weights.size() != pts.size()) {
        return false;
    }

    double w_total = 0.0;
    glm::dvec2 mean{0.0};
    for (size_t i = 0; i < pts.size(); ++i) {
        const double w = std::max(weights[i], 0.0);
        w_total += w;
        mean += pts[i] * w;
    }
    if (w_total <= 0.0) {
        return false;
    }
    mean /= w_total;

    double sxx = 0.0, sxy = 0.0, syy = 0.0;
    double sxz = 0.0, syz = 0.0;
    double sx = 0.0, sy = 0.0, sz = 0.0;
    for (size_t i = 0; i < pts.size(); ++i) {
        const double w = std::max(weights[i], 0.0);
        const double x = pts[i].x - mean.x;
        const double y = pts[i].y - mean.y;
        const double z = x * x + y * y;
        sxx += w * x * x;
        sxy += w * x * y;
        syy += w * y * y;
        sxz += w * x * z;
        syz += w * y * z;
        sx  += w * x;
        sy  += w * y;
        sz  += w * z;
    }

    // Normal equations for z = A x + B y + C, in the shifted frame.
    const double m00 = sxx, m01 = sxy, m02 = sx;
    const double m10 = sxy, m11 = syy, m12 = sy;
    const double m20 = sx,  m21 = sy,  m22 = w_total;

    const double det = m00 * (m11 * m22 - m12 * m21)
                     - m01 * (m10 * m22 - m12 * m20)
                     + m02 * (m10 * m21 - m11 * m20);

    // Scale-relative singularity test: the determinant carries units of
    // metres^4 times weight^3, so an absolute threshold would mean nothing.
    const double scale = std::max(sxx + syy, 1e-12);
    if (std::fabs(det) <= 1e-12 * scale * scale * w_total) {
        return false;
    }

    const double a = (sxz * (m11 * m22 - m12 * m21)
                    - m01 * (syz * m22 - m12 * sz)
                    + m02 * (syz * m21 - m11 * sz)) / det;
    const double b = (m00 * (syz * m22 - m12 * sz)
                    - sxz * (m10 * m22 - m12 * m20)
                    + m02 * (m10 * sz - syz * m20)) / det;
    const double c = (m00 * (m11 * sz - syz * m21)
                    - m01 * (m10 * sz - syz * m20)
                    + sxz * (m10 * m21 - m11 * m20)) / det;

    const glm::dvec2 local_center(0.5 * a, 0.5 * b);
    const double r_sq = c + glm::dot(local_center, local_center);
    if (!(r_sq > 0.0) || !std::isfinite(r_sq)) {
        return false;
    }

    out_center = mean + local_center;
    out_radius = std::sqrt(r_sq);
    return std::isfinite(out_center.x) && std::isfinite(out_center.y);
}

// ============================================================================
// Graph walking helpers
// ============================================================================

/// Unit direction the edge leaves its `from` node in; (0,0) when degenerate
[[nodiscard]] glm::dvec2 leaving_dir(const GraphEdge& e) {
    for (size_t i = 1; i < e.polyline.size(); ++i) {
        const glm::dvec2 d = e.polyline[i] - e.polyline[0];
        if (glm::dot(d, d) > kWeldEpsilon * kWeldEpsilon) {
            return glm::normalize(d);
        }
    }
    return glm::dvec2(0.0);
}

/// Unit direction the edge arrives at its `to` node in; (0,0) when degenerate
[[nodiscard]] glm::dvec2 arrival_dir(const GraphEdge& e) {
    const size_t n = e.polyline.size();
    for (size_t i = n; i-- > 1;) {
        const glm::dvec2 d = e.polyline[n - 1] - e.polyline[i - 1];
        if (glm::dot(d, d) > kWeldEpsilon * kWeldEpsilon) {
            return glm::normalize(d);
        }
    }
    return glm::dvec2(0.0);
}

/**
 * @brief Stations of one edge end, ordered OUTWARD from the node
 *
 * Cut with slice() rather than by snapping to the nearest resampled station, so
 * the far end lands exactly @p distance along the edge and the offset column
 * there sits ON the untrimmed ribbon's edge. See the slice() contract.
 *
 * @param cl       Edge centerline
 * @param at_start True when the node is the edge's `from` end
 * @param distance Arclength to take, metres; clamped to the edge
 * @return Stations from the node outward; empty when nothing usable is left
 */
[[nodiscard]] std::vector<Station> end_run(const Centerline& cl, bool at_start, double distance) {
    std::vector<Station> out;
    if (!cl.is_valid() || distance <= kWeldEpsilon) {
        return out;
    }
    const double total = cl.length();
    const double d = std::min(distance, total);

    const Centerline sub = at_start ? slice(cl, 0.0, d) : slice(cl, total - d, total);
    if (!sub.is_valid()) {
        return out;
    }
    out = sub.stations;
    if (!at_start) {
        std::reverse(out.begin(), out.end());
    }
    return out;
}

/**
 * @brief Append a tessellated circular arc, snapping both ends to given points
 *
 * The endpoints are snapped rather than computed so the arc welds exactly to
 * whatever it is joining -- a ribbon's own offset edge, which is not on the
 * circle when the road curves. Intermediate vertices sit on the circle, and the
 * radius error at the joins is bounded by the ribbon's departure from straight
 * over a few metres.
 *
 * @param ring      Receives the arc, coincident points collapsed
 * @param center    Arc centre
 * @param radius    Arc radius, metres
 * @param ax, ay    Right-handed basis the angles are measured in
 * @param th0, th1  Start and end angle; the sweep runs counter-clockwise from
 *                  @p th0 up to @p th1, which must be the greater
 * @param snap0     Exact position of the first vertex
 * @param snap1     Exact position of the last vertex
 */
void append_arc(std::vector<glm::dvec2>& ring,
                const glm::dvec2& center,
                double radius,
                const glm::dvec2& ax,
                const glm::dvec2& ay,
                double th0,
                double th1,
                const glm::dvec2& snap0,
                const glm::dvec2& snap1) {
    const double sweep = th1 - th0;
    int segments = static_cast<int>(std::ceil(std::fabs(sweep) / (2.0 * kPi) * kSegmentsPerTurn));
    segments = std::max(segments, 2);

    for (int k = 0; k <= segments; ++k) {
        if (k == 0) {
            push_unique(ring, snap0);
            continue;
        }
        if (k == segments) {
            push_unique(ring, snap1);
            continue;
        }
        const double th = th0 + sweep * (static_cast<double>(k) / static_cast<double>(segments));
        push_unique(ring, center + ax * (radius * std::cos(th)) + ay * (radius * std::sin(th)));
    }
}

} // namespace

// ============================================================================
// Roundabout detection
// ============================================================================

/**
 * @brief Extract the closed cycles of the is_roundabout subgraph
 *
 * Invariant: every returned loop is a DIRECTED cycle -- `edges[i]` runs from
 * `nodes[i]` to `nodes[i + 1]`, and the last edge closes onto `nodes[0]` -- each
 * edge appears in at most one loop, and the output is ordered reproducibly
 * regardless of traversal order.
 *
 * The walk is directed because a roundabout is one-way and OSM draws it in the
 * direction of travel. Following bearing alone, or treating the subgraph as
 * undirected, admits a walk that reverses back down the edge it arrived on and
 * reports a two-edge "cycle" that is one way traversed twice.
 *
 * Three guards, in the order they matter:
 *
 * - **Termination.** Every edge the walk touches is consumed, whether or not the
 *   walk closed, and the step count is bounded by the edge count. An unclosed
 *   chain -- an extract cut through a roundabout, or a `junction=roundabout` tag
 *   on a way that is not a loop -- therefore ends, once, and yields no loop.
 * - **Re-entry, not just return.** A walk that runs down a tail chain INTO a
 *   cycle would never see its own start node again. The walk instead records the
 *   position of every node it enters and closes on the first REVISIT, taking the
 *   suffix from that first visit as the cycle and discarding the tail.
 * - **Figure of eight.** Two roundabouts sharing a node -- ordinary at a
 *   grade-separated interchange -- offer two unused outgoing edges at the shared
 *   node. The continuation is chosen as the one with the SMALLEST turn from the
 *   arrival direction, which stays on the circle being walked instead of cutting
 *   across into the other one, so the pair comes back as two loops. Ties break on
 *   the lower EdgeId, so the choice is deterministic.
 *
 * A single-edge cycle is a real roundabout whose ring way closes on itself with
 * no approach splitting it, and is accepted.
 */
std::vector<RoundaboutLoop> find_roundabouts(const RoadGraph& graph,
                                             const std::vector<Centerline>& centerlines) {
    std::vector<RoundaboutLoop> out;

    const std::vector<GraphEdge>& edges = graph.edges();
    const std::vector<GraphNode>& nodes = graph.nodes();
    if (edges.empty() || nodes.empty()) {
        return out;
    }

    std::vector<uint8_t> is_ring(edges.size(), 0u);
    std::vector<std::vector<EdgeId>> outgoing(nodes.size());

    for (size_t i = 0; i < edges.size(); ++i) {
        const GraphEdge& e = edges[i];
        if (!e.is_roundabout || e.polyline.size() < 2u) {
            continue;
        }
        if (e.from >= nodes.size() || e.to >= nodes.size()) {
            continue;
        }
        is_ring[i] = 1u;
        outgoing[e.from].push_back(static_cast<EdgeId>(i));
    }

    // Deterministic candidate order at every branch point.
    for (auto& list : outgoing) {
        std::sort(list.begin(), list.end());
    }

    std::vector<uint8_t> consumed(edges.size(), 0u);

    // Node -> index into the current walk. Cleared per walk; a full vector is
    // cheaper than a hash map and the graph is already indexed by node.
    std::vector<size_t> visit(nodes.size(), static_cast<size_t>(-1));
    std::vector<GraphNodeId> touched;

    for (size_t start = 0; start < edges.size(); ++start) {
        if (!is_ring[start] || consumed[start]) {
            continue;
        }

        std::vector<EdgeId> path;
        std::vector<GraphNodeId> path_nodes;
        size_t cycle_at = static_cast<size_t>(-1);

        EdgeId cur = static_cast<EdgeId>(start);
        for (size_t step = 0; step <= edges.size(); ++step) {
            const GraphEdge& e = edges[cur];

            visit[e.from] = path.size();
            touched.push_back(e.from);
            path.push_back(cur);
            path_nodes.push_back(e.from);
            consumed[cur] = 1u;

            if (visit[e.to] != static_cast<size_t>(-1)) {
                cycle_at = visit[e.to];   // closes here, tail before it discarded
                break;
            }

            const glm::dvec2 arrive = arrival_dir(e);
            EdgeId best = kInvalidId;
            double best_turn = 0.0;
            for (EdgeId cand : outgoing[e.to]) {
                if (consumed[cand]) {
                    continue;
                }
                const glm::dvec2 leave = leaving_dir(edges[cand]);
                const double dot = std::min(1.0, std::max(-1.0, glm::dot(arrive, leave)));
                const double turn = std::acos(dot);
                if (best == kInvalidId || turn < best_turn) {
                    best = cand;
                    best_turn = turn;
                }
            }
            if (best == kInvalidId) {
                break;      // unclosed chain: bad data, or an extract boundary
            }
            cur = best;
        }

        for (GraphNodeId n : touched) {
            visit[n] = static_cast<size_t>(-1);
        }
        touched.clear();

        if (cycle_at == static_cast<size_t>(-1)) {
            continue;
        }

        RoundaboutLoop loop;
        loop.edges.assign(path.begin() + static_cast<std::ptrdiff_t>(cycle_at), path.end());
        loop.nodes.assign(path_nodes.begin() + static_cast<std::ptrdiff_t>(cycle_at), path_nodes.end());

        // Rotate to the loop's own lowest EdgeId, so the same ring produces the
        // same traversal whichever edge the outer scan reached first.
        const size_t pivot = static_cast<size_t>(
            std::min_element(loop.edges.begin(), loop.edges.end()) - loop.edges.begin());
        if (pivot != 0u) {
            std::rotate(loop.edges.begin(),
                        loop.edges.begin() + static_cast<std::ptrdiff_t>(pivot),
                        loop.edges.end());
            std::rotate(loop.nodes.begin(),
                        loop.nodes.begin() + static_cast<std::ptrdiff_t>(pivot),
                        loop.nodes.end());
        }

        // ------------------------------------------------------------------
        // Centre and radius, from the centerline STATIONS of the whole cycle.
        // ------------------------------------------------------------------
        std::vector<glm::dvec2> samples;
        std::vector<double> weights;
        for (EdgeId e : loop.edges) {
            if (e >= centerlines.size()) {
                continue;
            }
            const Centerline& cl = centerlines[e];
            if (!cl.is_valid()) {
                continue;
            }
            for (size_t i = 0; i < cl.stations.size(); ++i) {
                const double prev = (i > 0) ? cl.stations[i - 1].arclength : cl.stations[i].arclength;
                const double next = (i + 1 < cl.stations.size()) ? cl.stations[i + 1].arclength
                                                                 : cl.stations[i].arclength;
                samples.push_back(cl.stations[i].position);
                weights.push_back(0.5 * (next - prev));
            }
        }

        if (samples.empty()) {
            for (GraphNodeId n : loop.nodes) {
                if (n < nodes.size()) {
                    samples.push_back(nodes[n].position);
                    weights.push_back(1.0);
                }
            }
        }

        if (!samples.empty()) {
            double w_total = 0.0;
            glm::dvec2 centroid{0.0};
            for (size_t i = 0; i < samples.size(); ++i) {
                const double w = std::max(weights[i], 0.0);
                w_total += w;
                centroid += samples[i] * w;
            }
            if (w_total > 0.0) {
                centroid /= w_total;
            } else {
                for (const glm::dvec2& p : samples) {
                    centroid += p;
                }
                centroid /= static_cast<double>(samples.size());
            }

            glm::dvec2 fitted{0.0};
            double fitted_r = 0.0;
            if (fit_circle(samples, weights, fitted, fitted_r)) {
                loop.center = fitted;
                loop.radius = fitted_r;
            } else {
                // Nearly collinear, or a degenerate weight set: fall back to the
                // centroid and the mean distance, which is what the fit reduces
                // to on a perfect circle anyway.
                loop.center = centroid;
                double sum = 0.0;
                for (const glm::dvec2& p : samples) {
                    sum += std::sqrt(dist_sq(p, centroid));
                }
                loop.radius = sum / static_cast<double>(samples.size());
            }
        }

        loop.valid = !loop.edges.empty() && loop.radius >= RoundaboutConfig{}.min_radius;
        out.push_back(std::move(loop));
    }

    std::sort(out.begin(), out.end(), [](const RoundaboutLoop& a, const RoundaboutLoop& b) {
        return a.edges.front() < b.edges.front();
    });
    return out;
}

// ============================================================================
// Roundabout geometry
// ============================================================================

/**
 * @brief Sweep the loop's carriageway annulus and its central island
 *
 * Invariant: the annulus is swept from the loop's OWN centerlines, so it follows
 * an oval ring as faithfully as a circular one, and the island is a single
 * closed surface however many ways the ring was split into.
 *
 * Two things are deliberately different between the two surfaces:
 *
 * - The annulus honours GraphEdge::trim_from and trim_to, so where the junction
 *   solve has already cut a ring edge back for an approach mouth, the annulus
 *   stops there too and the mouth's own fillet fills the gap. Nothing is built
 *   twice at an approach node. With trims still zero the ring closes all the way
 *   round, which is the correct answer before the trims are solved.
 * - The island ignores trims entirely and is built from the full untrimmed ring.
 *   An approach cuts the carriageway, never the island: the ground in the middle
 *   of a roundabout is continuous whatever joins it.
 */
Mesh build_roundabout(const RoundaboutLoop& loop,
                      const RoadGraph& graph,
                      const std::vector<Centerline>& centerlines,
                      const std::vector<RoadProfile>& profiles,
                      float height,
                      const RoundaboutConfig& cfg) {
    Mesh out;

    const std::vector<GraphEdge>& edges = graph.edges();
    if (!loop.valid || loop.edges.empty() || loop.edges.size() != loop.nodes.size()) {
        return out;
    }
    if (centerlines.size() != edges.size() || profiles.size() != edges.size()) {
        return out;
    }
    if (loop.radius < cfg.min_radius) {
        return out;    // a mini-roundabout: a painted dome, not an annulus
    }

    const double base = static_cast<double>(height);
    MeshAccum acc;

    // A roundabout is one surface, so it takes one material: the first usable
    // ring edge's carriageway material.
    MaterialId ring_material = MaterialId::Asphalt;
    bool have_material = false;

    // ------------------------------------------------------------------------
    // Carriageway annulus, in runs broken wherever a trim has opened a mouth.
    // ------------------------------------------------------------------------
    std::vector<SweepColumn> run;
    std::vector<glm::dvec2> run_centres;
    double v_accum = 0.0;
    bool run_open = false;
    bool first_edge_open_at_start = false;
    bool last_edge_open_at_end = false;
    size_t run_count = 0;

    const std::vector<uint8_t> annulus_faces{ 0u };

    const auto flush_run = [&](bool closed) {
        if (run.size() >= 2u) {
            std::vector<MaterialId> mats{ ring_material };
            emit_sweep(acc, mats, annulus_faces, run, base);
            ++run_count;
        }
        (void)closed;
        run.clear();
        run_centres.clear();
        run_open = false;
    };

    // ------------------------------------------------------------------------
    // Where the walk starts.
    //
    // The sweep breaks its run at every trimmed node and only rejoins the last
    // run to the first when the ring carries NO trim anywhere (`fully_closed`
    // below). Every real roundabout has at least one approach and therefore at
    // least one trim, so starting at loop.nodes[0] -- which find_roundabouts()
    // picks by lowest EdgeId and which is usually a plain degree-2 seam between
    // two ring ways -- leaves a run ENDING at that node and another STARTING
    // there, built from two independently framed end stations. Their columns are
    // centred on the same point but rotated against each other by one band's
    // turn, so the annulus opens a wedge of uncovered surface right there.
    //
    // Rotating the traversal to begin at a node that is already a break makes
    // the wrap unnecessary: every interior join is welded by the duplicate-column
    // test below, and the one place the surface is allowed to end is a mouth the
    // approach's own fillet fills. A ring with no break anywhere falls back to
    // index 0 and takes the fully_closed path, exactly as before.
    // ------------------------------------------------------------------------
    const size_t loop_size = loop.edges.size();
    size_t start_at = 0;
    for (size_t k = 0; k < loop_size; ++k) {
        const EdgeId e = loop.edges[k];
        const EdgeId prev = loop.edges[(k + loop_size - 1u) % loop_size];
        if (e >= edges.size() || prev >= edges.size()) {
            continue;
        }
        if (edges[e].trim_from > kWeldEpsilon || edges[prev].trim_to > kWeldEpsilon) {
            start_at = k;
            break;
        }
    }

    for (size_t step = 0; step < loop_size; ++step) {
        const size_t k = (start_at + step) % loop_size;
        const EdgeId e = loop.edges[k];
        if (e >= edges.size()) {
            flush_run(false);
            continue;
        }
        const GraphEdge& ge = edges[e];
        const Centerline& cl = centerlines[e];
        const SectionModel model = model_from_profile(profiles[e], false);
        if (!cl.is_valid() || model.empty()) {
            flush_run(false);
            continue;
        }

        if (!have_material) {
            ring_material = lane_material(model);
            have_material = true;
        }

        const double chalf = carriageway_half(model);
        if (chalf <= kZeroExtent) {
            flush_run(false);
            continue;
        }

        const double total = cl.length();
        const double t_from = std::max(ge.trim_from, 0.0);
        const double t_to = std::max(ge.trim_to, 0.0);
        const double a0 = std::min(t_from, total);
        const double a1 = std::max(total - t_to, a0);
        if (a1 - a0 <= kWeldEpsilon) {
            flush_run(false);
            continue;
        }

        const bool gap_before = t_from > kWeldEpsilon;
        const bool gap_after = t_to > kWeldEpsilon;
        if (step == 0) {
            first_edge_open_at_start = !gap_before;
        }
        if (step + 1 == loop_size) {
            last_edge_open_at_end = !gap_after;
        }

        const Centerline sub = slice(cl, a0, a1);
        if (!sub.is_valid()) {
            flush_run(false);
            continue;
        }

        if (gap_before) {
            flush_run(false);
        }
        run_open = true;

        for (const Station& s : sub.stations) {
            // A column whose centre coincides with the previous one is the
            // duplicate endpoint two consecutive ring edges share at a
            // degree-2 split. Keeping only the first welds the two runs into
            // one surface across the join.
            if (!run_centres.empty() &&
                dist_sq(run_centres.back(), s.position) <= kWeldEpsilon * kWeldEpsilon) {
                continue;
            }
            if (!run_centres.empty()) {
                v_accum += std::sqrt(dist_sq(run_centres.back(), s.position));
            }

            SweepColumn col;
            // Positive lateral is LEFT of travel by definition, so this ordering
            // is left-to-right whichever way the ring is drawn.
            col.pos = { offset_point(s, chalf), offset_point(s, -chalf) };
            col.height = { 0.0, 0.0 };
            col.v = v_accum;
            run.push_back(std::move(col));
            run_centres.push_back(s.position);
        }

        if (gap_after) {
            flush_run(false);
        }
    }

    // A ring with no trims anywhere is one closed run: repeat its first column
    // at the end, at the accumulated V, so the seam is watertight and the
    // texture keeps running.
    const bool fully_closed = run_open && run_count == 0u &&
                              first_edge_open_at_start && last_edge_open_at_end &&
                              run.size() >= 3u;
    if (fully_closed) {
        // The last edge ends on the same station the first edge starts from, and
        // the two carry different end normals, so keeping both would close the
        // ring with a zero-length band whose two columns are tilted against each
        // other -- a bowtie sliver with one face pointing down. Drop the
        // duplicate exactly as an internal join does, then wrap onto the first
        // column itself.
        if (run.size() >= 3u &&
            dist_sq(run_centres.back(), run_centres.front()) <= kWeldEpsilon * kWeldEpsilon) {
            run.pop_back();
            run_centres.pop_back();
        }
        SweepColumn wrap = run.front();
        v_accum += std::sqrt(dist_sq(run_centres.back(), run_centres.front()));
        wrap.v = v_accum;
        run.push_back(std::move(wrap));
    }
    flush_run(fully_closed);

    // ------------------------------------------------------------------------
    // Central island, from the full untrimmed ring.
    // ------------------------------------------------------------------------
    // Which lateral side the island is on. Positive lateral is left of travel,
    // so a counter-clockwise ring has its island to the left and a clockwise one
    // to the right. Decided by vote over every station rather than per edge, so
    // one badly digitised segment cannot flip a stretch of the island inside out.
    double inward_vote = 0.0;
    for (const EdgeId e : loop.edges) {
        if (e >= centerlines.size() || !centerlines[e].is_valid()) {
            continue;
        }
        for (const Station& s : centerlines[e].stations) {
            inward_vote += glm::dot(s.normal, loop.center - s.position);
        }
    }
    const double inward = (inward_vote >= 0.0) ? 1.0 : -1.0;

    std::vector<glm::dvec2> island;
    std::vector<glm::dvec2> island_centres;
    for (const EdgeId e : loop.edges) {
        if (e >= edges.size() || !centerlines[e].is_valid()) {
            continue;
        }
        const SectionModel model = model_from_profile(profiles[e], false);
        if (model.empty()) {
            continue;
        }
        const double chalf = carriageway_half(model);
        if (chalf <= kZeroExtent) {
            continue;
        }
        const double lateral = inward * (chalf + cfg.island_inset);
        for (const Station& s : centerlines[e].stations) {
            // Deduplicated on the STATION, not on the offset point. Two
            // consecutive ring edges share their join station exactly, but each
            // carries its own end normal, so their offset points differ by the
            // half turn angle and the ring backtracks on itself there. A
            // backtrack is a reversed ear, which is a hole in the island.
            if (!island_centres.empty() &&
                dist_sq(island_centres.back(), s.position) <= kWeldEpsilon * kWeldEpsilon) {
                continue;
            }
            island_centres.push_back(s.position);
            push_unique(island, offset_point(s, lateral));
        }
    }
    // The ring closes onto the loop's first station, which is the same station
    // the last edge ends on.
    if (island_centres.size() >= 2u &&
        dist_sq(island_centres.back(), island_centres.front()) <= kWeldEpsilon * kWeldEpsilon) {
        island.pop_back();
        island_centres.pop_back();
    }
    close_ring(island);

    if (island.size() >= 3u) {
        // The fan and the curb face both need a counter-clockwise ring; the loop
        // may be drawn either way round depending on which side of the road the
        // country drives on.
        if (ring_signed_area2(island) < 0.0) {
            std::reverse(island.begin(), island.end());
        }

        if (cfg.raised_island) {
            // Curb face: the island stands behind it, so the face leans away
            // from the carriageway -- inward -- over its height, and its normal
            // points outward, at the traffic. Columns run inner-then-outer,
            // which is left-then-right for a counter-clockwise traversal.
            std::vector<SweepColumn> face;
            face.reserve(island.size() + 1u);
            double face_v = 0.0;
            for (size_t i = 0; i <= island.size(); ++i) {
                const glm::dvec2& p = island[i % island.size()];
                if (i > 0) {
                    face_v += std::sqrt(dist_sq(island[(i - 1u) % island.size()], p));
                }
                const glm::dvec2 to_centre = loop.center - p;
                const double len = std::sqrt(glm::dot(to_centre, to_centre));
                const glm::dvec2 top = (len > kWeldEpsilon)
                                     ? p + to_centre * (kIslandCurbBatter / len)
                                     : p;

                SweepColumn col;
                col.pos = { top, p };
                col.height = { kIslandCurbHeight, 0.0 };
                col.v = face_v;
                face.push_back(std::move(col));
            }

            const std::vector<MaterialId> face_material{ MaterialId::Curb };
            const std::vector<uint8_t> face_vertical{ 1u };
            emit_sweep(acc, face_material, face_vertical, face, base);

            // Island top, inset by the batter so it meets the top of the face.
            std::vector<glm::dvec2> top_ring;
            top_ring.reserve(island.size());
            for (const glm::dvec2& p : island) {
                const glm::dvec2 to_centre = loop.center - p;
                const double len = std::sqrt(glm::dot(to_centre, to_centre));
                top_ring.push_back((len > kWeldEpsilon)
                                   ? p + to_centre * (kIslandCurbBatter / len)
                                   : p);
            }
            triangulate_ring(acc, top_ring, loop.center, base + kIslandCurbHeight,
                             MaterialId::Grass);
        } else {
            triangulate_ring(acc, island, loop.center, base, MaterialId::Grass);
        }
    } else if (!island.empty()) {
        spdlog::warn("build_roundabout: loop at ({:.1f}, {:.1f}) r={:.1f} m produced a "
                     "{}-point island ring; island suppressed",
                     loop.center.x, loop.center.y, loop.radius, island.size());
    }

    out = acc.take();
    return out;
}

namespace {

// ============================================================================
// Half-section matching, for the profile taper
// ============================================================================

/**
 * @brief One side of a cross-section, ordered OUTWARD from the carriageway centre
 *
 * `height` holds m+1 boundaries with `height[0]` on the centreline itself, and
 * the strip arrays hold m entries, so `width[j]` spans boundaries j and j+1.
 */
struct HalfSection {
    std::vector<double>     height;
    std::vector<double>     width;
    std::vector<StripKind>  kind;
    std::vector<MaterialId> material;

    [[nodiscard]] size_t strips() const { return width.size(); }
};

/**
 * @brief Insert a boundary at lateral 0 if none is there
 *
 * left_edge_offset() centres the carriageway envelope on zero, so zero is the
 * axis both profiles are aligned about. An odd lane count puts a lane ACROSS it;
 * splitting that lane into two half-lanes of the same kind and material is what
 * lets both sides be matched independently.
 *
 * @return Index of the boundary at lateral 0, or the boundary count when the
 *         profile does not span zero at all
 */
[[nodiscard]] size_t split_at_zero(SectionModel& m) {
    for (size_t i = 0; i < m.lateral.size(); ++i) {
        if (std::fabs(m.lateral[i]) <= kZeroExtent) {
            m.lateral[i] = 0.0;
            return i;
        }
    }
    for (size_t i = 0; i + 1 < m.lateral.size(); ++i) {
        if (m.lateral[i] > 0.0 && m.lateral[i + 1] < 0.0) {
            const double span = m.lateral[i] - m.lateral[i + 1];
            const double f = (span > 0.0) ? (m.lateral[i] / span) : 0.5;
            const double h = m.height[i] + (m.height[i + 1] - m.height[i]) * f;

            m.lateral.insert(m.lateral.begin() + static_cast<std::ptrdiff_t>(i) + 1, 0.0);
            m.height.insert(m.height.begin() + static_cast<std::ptrdiff_t>(i) + 1, h);
            m.kind.insert(m.kind.begin() + static_cast<std::ptrdiff_t>(i) + 1, m.kind[i]);
            m.material.insert(m.material.begin() + static_cast<std::ptrdiff_t>(i) + 1, m.material[i]);
            return i + 1u;
        }
    }
    // Wholly to one side of zero: an offset footway, or a profile with no
    // carriageway at all. Treat the nearer end as the centre.
    return m.lateral.front() < 0.0 ? 0u : m.lateral.size() - 1u;
}

/// Extract one side of a split model, ordered outward from @p zero
[[nodiscard]] HalfSection half_of(const SectionModel& m, size_t zero, bool left) {
    HalfSection h;
    h.height.push_back(m.height[zero]);
    if (left) {
        for (size_t i = zero; i-- > 0;) {
            h.width.push_back(m.lateral[i] - m.lateral[i + 1u]);
            h.kind.push_back(m.kind[i]);
            h.material.push_back(m.material[i]);
            h.height.push_back(m.height[i]);
        }
    } else {
        for (size_t i = zero; i < m.strips(); ++i) {
            h.width.push_back(m.lateral[i] - m.lateral[i + 1u]);
            h.kind.push_back(m.kind[i]);
            h.material.push_back(m.material[i]);
            h.height.push_back(m.height[i + 1u]);
        }
    }
    return h;
}

/// One strip of the blended cross-section: the two ends it interpolates between
struct MergedStrip {
    StripKind  kind = StripKind::Lane;
    MaterialId material_a = MaterialId::Asphalt;
    MaterialId material_b = MaterialId::Asphalt;
    double width_a = 0.0;
    double width_b = 0.0;
    bool vertical_face = false;
};

/// One side of the blended cross-section, outward from the centreline
struct MergedHalf {
    std::vector<double> height_a;   ///< m+1 boundaries
    std::vector<double> height_b;   ///< m+1 boundaries
    std::vector<MergedStrip> strips;
};

/**
 * @brief Align two half-sections by StripKind and blend them
 *
 * THE MATCHING RULE. Both halves are walked OUTWARD from the carriageway centre,
 * and their StripKind sequences are aligned on a longest common subsequence.
 * Every LCS member is a matched pair that interpolates width, height and
 * material; every strip outside it keeps its own end's width and takes ZERO
 * width at the other end, so it grows out of nothing or shrinks into nothing.
 *
 * The consequences that matter:
 *
 * - A strip is only ever paired with a strip of the SAME kind, so a cycle lane
 *   present on one edge and absent on the other opens from zero instead of a
 *   lane being dragged sideways into a sidewalk.
 * - Sides are matched independently, because the profile is split at lateral
 *   zero first. A sidewalk appearing on the left says nothing about the right.
 * - Matching outward from the centre rather than inward from the outer edge
 *   keeps the carriageway aligned with the carriageway. The outer edge is the
 *   end that MOVES in a taper; anchoring on it would slide every lane sideways.
 * - Ties in the LCS reconstruction advance side A first, so the alignment is a
 *   pure function of the two inputs.
 *
 * Boundary heights are carried through the same alignment, so an unmatched strip
 * inherits the height of the boundary it collapses onto and a curb rises out of
 * the surface instead of stepping up.
 */
[[nodiscard]] MergedHalf align_halves(const HalfSection& a, const HalfSection& b) {
    const size_t na = a.strips();
    const size_t nb = b.strips();

    // dp[i][j] = length of the LCS of a.kind[i..] and b.kind[j..]
    std::vector<std::vector<uint16_t>> dp(na + 1u, std::vector<uint16_t>(nb + 1u, 0u));
    for (size_t i = na; i-- > 0;) {
        for (size_t j = nb; j-- > 0;) {
            dp[i][j] = (a.kind[i] == b.kind[j])
                     ? static_cast<uint16_t>(dp[i + 1u][j + 1u] + 1u)
                     : std::max(dp[i + 1u][j], dp[i][j + 1u]);
        }
    }

    MergedHalf out;
    out.height_a.push_back(a.height.front());
    out.height_b.push_back(b.height.front());

    size_t i = 0;
    size_t j = 0;
    while (i < na || j < nb) {
        MergedStrip s;
        bool took_a = false;
        bool took_b = false;

        if (i < na && j < nb && a.kind[i] == b.kind[j] &&
            dp[i][j] == dp[i + 1u][j + 1u] + 1u) {
            s.kind = a.kind[i];
            s.width_a = a.width[i];
            s.width_b = b.width[j];
            s.material_a = a.material[i];
            s.material_b = b.material[j];
            took_a = took_b = true;
        } else if (j >= nb || (i < na && dp[i + 1u][j] >= dp[i][j + 1u])) {
            s.kind = a.kind[i];
            s.width_a = a.width[i];
            s.width_b = 0.0;
            s.material_a = a.material[i];
            s.material_b = a.material[i];
            took_a = true;
        } else {
            s.kind = b.kind[j];
            s.width_a = 0.0;
            s.width_b = b.width[j];
            s.material_a = b.material[j];
            s.material_b = b.material[j];
            took_b = true;
        }

        s.vertical_face = (s.kind == StripKind::CurbFace);

        if (took_a) {
            ++i;
        }
        if (took_b) {
            ++j;
        }
        out.height_a.push_back(a.height[i]);
        out.height_b.push_back(b.height[j]);
        out.strips.push_back(s);
    }
    return out;
}

/// True when two flattened cross-sections agree closely enough to need no taper
[[nodiscard]] bool sections_match(const SectionModel& a, const SectionModel& b) {
    if (a.strips() != b.strips() || a.lateral.size() != b.lateral.size()) {
        return false;
    }
    for (size_t i = 0; i < a.strips(); ++i) {
        if (a.kind[i] != b.kind[i]) {
            return false;
        }
    }
    for (size_t i = 0; i < a.lateral.size(); ++i) {
        if (std::fabs(a.lateral[i] - b.lateral[i]) > kProfileMatchEpsilon ||
            std::fabs(a.height[i] - b.height[i]) > kProfileMatchEpsilon) {
            return false;
        }
    }
    return true;
}

} // namespace

// ============================================================================
// Degree-2 profile transitions
// ============================================================================

/**
 * @brief Blend two cross-sections across a degree-2 node
 *
 * Invariant: when true is returned the emitted mesh spans exactly the two
 * arclengths reported in @p out_trim_a and @p out_trim_b, so cutting both edges
 * back by them leaves no gap and no overlap; when false is returned both trims
 * are zero and @p out_mesh is untouched.
 *
 * The taper is emitted as two sweeps meeting at the node column -- side A's
 * materials before it, side B's after. They share the node column's positions
 * exactly, so the seam is watertight, and splitting there is what lets a surface
 * change (asphalt to gravel, say) land at the node rather than being smeared
 * across the whole blend, which no material assignment can express.
 *
 * @note The taper is emitted FLAT at @p height. The contract supplies one height
 *       for the node and no per-station solve, so a taper on a steep grade
 *       departs from its arms by up to `grade * length / 2`. Keeping the taper
 *       short is the only mitigation available here; the ribbons themselves
 *       still follow the terrain.
 */
bool build_profile_taper(const RoadGraph& graph,
                         const std::vector<Centerline>& centerlines,
                         const std::vector<RoadProfile>& profiles,
                         GraphNodeId node,
                         float height,
                         const TaperConfig& cfg,
                         Mesh& out_mesh,
                         double& out_trim_a,
                         double& out_trim_b,
                         double limit_a,
                         double limit_b,
                         std::vector<glm::dvec2>* out_outline) {
    out_trim_a = 0.0;
    out_trim_b = 0.0;
    if (out_outline != nullptr) {
        out_outline->clear();
    }

    const std::vector<GraphEdge>& edges = graph.edges();
    if (node >= graph.nodes().size()) {
        return false;
    }
    if (centerlines.size() != edges.size() || profiles.size() != edges.size()) {
        return false;
    }

    const GraphNode& n = graph.node(node);
    if (n.degree() != 2u) {
        return false;
    }

    const Arm& arm_a = n.arms[0];
    const Arm& arm_b = n.arms[1];
    if (arm_a.edge >= edges.size() || arm_b.edge >= edges.size()) {
        return false;
    }

    const Centerline& cl_a = centerlines[arm_a.edge];
    const Centerline& cl_b = centerlines[arm_b.edge];
    if (!cl_a.is_valid() || !cl_b.is_valid()) {
        return false;
    }

    // The taper travels A -> node -> B. An edge whose node end is its `from`
    // travels the other way, so its cross-section is mirrored into that frame.
    const bool mirror_a = arm_a.at_start;
    const bool mirror_b = !arm_b.at_start;

    SectionModel model_a = model_from_profile(profiles[arm_a.edge], mirror_a);
    SectionModel model_b = model_from_profile(profiles[arm_b.edge], mirror_b);
    if (model_a.empty() || model_b.empty()) {
        return false;
    }

    if (sections_match(model_a, model_b)) {
        return false;   // a hard join here is imperceptible
    }

    // ------------------------------------------------------------------------
    // Length, from the LARGEST single-sided width change.
    // ------------------------------------------------------------------------
    const double dw_left = std::fabs(model_a.lateral.front() - model_b.lateral.front());
    const double dw_right = std::fabs(model_a.lateral.back() - model_b.lateral.back());
    const double max_side = std::max(dw_left, dw_right);

    double length = max_side * cfg.length_per_metre_width;
    length = std::min(std::max(length, cfg.min_length), cfg.max_length);
    // Never consume a whole short edge: half of the length is cut from each
    // side, so bounding the length by the shorter edge bounds each trim by half
    // of its own edge.
    length = std::min(length, std::min(cl_a.length(), cl_b.length()));
    if (length <= kWeldEpsilon) {
        return false;
    }

    // The caller's ceilings, not the solver's own. A taper's demand goes through
    // the same joint budget every junction trim does, and the budget can reduce
    // it; the wedge has to be built over the REDUCED value or the ribbon is
    // sliced short of it and the two surfaces overlap coplanarly.
    const double trim_a = std::min(0.5 * length, std::max(0.0, limit_a));
    const double trim_b = std::min(0.5 * length, std::max(0.0, limit_b));
    if (trim_a <= kWeldEpsilon || trim_b <= kWeldEpsilon) {
        return false;
    }

    const std::vector<Station> run_a = end_run(cl_a, arm_a.at_start, trim_a);
    const std::vector<Station> run_b = end_run(cl_b, arm_b.at_start, trim_b);
    if (run_a.size() < 2u || run_b.size() < 2u) {
        return false;
    }

    // ------------------------------------------------------------------------
    // Match the two cross-sections, side by side.
    // ------------------------------------------------------------------------
    const size_t zero_a = split_at_zero(model_a);
    const size_t zero_b = split_at_zero(model_b);

    const MergedHalf left = align_halves(half_of(model_a, zero_a, true),
                                         half_of(model_b, zero_b, true));
    const MergedHalf right = align_halves(half_of(model_a, zero_a, false),
                                          half_of(model_b, zero_b, false));

    const size_t lm = left.strips.size();
    const size_t rm = right.strips.size();
    const size_t strip_count = lm + rm;
    if (strip_count == 0u) {
        return false;
    }

    // Full cross-section, left to right: the left half reversed, then the right.
    std::vector<const MergedStrip*> strips(strip_count, nullptr);
    for (size_t s = 0; s < lm; ++s) {
        strips[s] = &left.strips[lm - 1u - s];
    }
    for (size_t s = 0; s < rm; ++s) {
        strips[lm + s] = &right.strips[s];
    }

    std::vector<MaterialId> materials_a(strip_count);
    std::vector<MaterialId> materials_b(strip_count);
    std::vector<uint8_t> vertical(strip_count, 0u);
    for (size_t s = 0; s < strip_count; ++s) {
        materials_a[s] = strips[s]->material_a;
        materials_b[s] = strips[s]->material_b;
        vertical[s] = strips[s]->vertical_face ? 1u : 0u;
    }

    // Boundary heights at either end of the taper, left to right.
    std::vector<double> h_a(strip_count + 1u, 0.0);
    std::vector<double> h_b(strip_count + 1u, 0.0);
    for (size_t k = 0; k <= strip_count; ++k) {
        if (k <= lm) {
            h_a[k] = left.height_a[lm - k];
            h_b[k] = left.height_b[lm - k];
        } else {
            h_a[k] = right.height_a[k - lm];
            h_b[k] = right.height_b[k - lm];
        }
    }

    // ------------------------------------------------------------------------
    // Stations, A's cut through the node to B's cut.
    // ------------------------------------------------------------------------
    struct PathStation {
        const Station* station;
        bool mirror;
    };
    std::vector<PathStation> path;
    path.reserve(run_a.size() + run_b.size());
    for (const Station& s : run_a) {
        path.push_back(PathStation{ &s, mirror_a });
    }
    // run_a runs from the node outward; the taper runs the other way.
    std::reverse(path.begin(), path.end());
    const size_t node_index = path.size() - 1u;
    for (size_t i = 1; i < run_b.size(); ++i) {
        path.push_back(PathStation{ &run_b[i], mirror_b });
    }
    if (path.size() < 3u) {
        return false;
    }

    std::vector<double> travelled(path.size(), 0.0);
    for (size_t i = 1; i < path.size(); ++i) {
        travelled[i] = travelled[i - 1u] +
                       std::sqrt(dist_sq(path[i - 1u].station->position, path[i].station->position));
    }
    const double total = travelled.back();
    if (total <= kWeldEpsilon) {
        return false;
    }

    // ------------------------------------------------------------------------
    // Columns.
    // ------------------------------------------------------------------------
    std::vector<SweepColumn> columns(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
        const double t = travelled[i] / total;
        const Station& s = *path[i].station;
        const bool mirror = path[i].mirror;

        std::vector<double> width(strip_count, 0.0);
        double left_extent = 0.0;
        for (size_t k = 0; k < strip_count; ++k) {
            width[k] = strips[k]->width_a + (strips[k]->width_b - strips[k]->width_a) * t;
        }
        for (size_t k = 0; k < lm; ++k) {
            left_extent += width[k];
        }

        SweepColumn& col = columns[i];
        col.pos.resize(strip_count + 1u);
        col.height.resize(strip_count + 1u);
        col.v = travelled[i];

        double lateral = left_extent;
        for (size_t k = 0; k <= strip_count; ++k) {
            // Positive lateral is left of TRAVEL through the taper; the station
            // belongs to an edge that may run the other way, so the lateral is
            // taken back into the edge's own frame before offsetting.
            col.pos[k] = offset_point(s, mirror ? -lateral : lateral);
            col.height[k] = h_a[k] + (h_b[k] - h_a[k]) * t;
            if (k < strip_count) {
                lateral -= width[k];
            }
        }
    }

    MeshAccum acc;
    const double base = static_cast<double>(height);

    const std::vector<SweepColumn> first_half(columns.begin(),
                                              columns.begin() + static_cast<std::ptrdiff_t>(node_index) + 1);
    const std::vector<SweepColumn> second_half(columns.begin() + static_cast<std::ptrdiff_t>(node_index),
                                               columns.end());
    emit_sweep(acc, materials_a, vertical, first_half, base);
    emit_sweep(acc, materials_b, vertical, second_half, base);

    if (acc.empty()) {
        return false;
    }

    // ------------------------------------------------------------------------
    // Carve footprint: the wedge's own outer boundary, up one side and back the
    // other. Both ribbons stop at their trims, so this strip of ground has no
    // other footprint and would otherwise keep its raw procedural surface while
    // the taper sits flat at the node height on top of it.
    // ------------------------------------------------------------------------
    if (out_outline != nullptr) {
        std::vector<glm::dvec2> outline;
        outline.reserve(columns.size() * 2u);
        for (const SweepColumn& col : columns) {
            push_unique(outline, col.pos.front());
        }
        for (size_t i = columns.size(); i-- > 0;) {
            push_unique(outline, columns[i].pos.back());
        }
        close_ring(outline);
        if (outline.size() >= 3u) {
            if (ring_signed_area2(outline) < 0.0) {
                std::reverse(outline.begin(), outline.end());
            }
            *out_outline = std::move(outline);
        }
    }

    out_mesh = acc.take();
    out_trim_a = trim_a;
    out_trim_b = trim_b;
    return true;
}

namespace {

// ============================================================================
// Dead-end helpers
// ============================================================================

/**
 * @brief The strips outboard of the carriageway envelope on one side
 *
 * Ordered outward, with `height[0]` on the carriageway edge itself, so the
 * result can be swept radially around a cul-de-sac bulb and still butt against
 * the arm's own curb and sidewalk at the mouth. Empty for a profile that is all
 * carriageway.
 */
[[nodiscard]] HalfSection outboard_half(const SectionModel& m, bool left) {
    HalfSection h;
    if (m.empty()) {
        return h;
    }
    size_t first = 0;
    size_t last = 0;
    carriageway_span(m, first, last);

    if (left) {
        h.height.push_back(m.height[first]);
        for (size_t i = first; i-- > 0;) {
            h.width.push_back(m.lateral[i] - m.lateral[i + 1u]);
            h.kind.push_back(m.kind[i]);
            h.material.push_back(m.material[i]);
            h.height.push_back(m.height[i]);
        }
    } else {
        h.height.push_back(m.height[last + 1u]);
        for (size_t i = last + 1u; i < m.strips(); ++i) {
            h.width.push_back(m.lateral[i] - m.lateral[i + 1u]);
            h.kind.push_back(m.kind[i]);
            h.material.push_back(m.material[i]);
            h.height.push_back(m.height[i + 1u]);
        }
    }
    return h;
}

/// True when two half-sections are the same cross-section to within tolerance
[[nodiscard]] bool halves_match(const HalfSection& a, const HalfSection& b) {
    if (a.strips() != b.strips() || a.strips() == 0u) {
        return false;
    }
    for (size_t i = 0; i < a.strips(); ++i) {
        if (a.kind[i] != b.kind[i] || a.material[i] != b.material[i] ||
            std::fabs(a.width[i] - b.width[i]) > kProfileMatchEpsilon) {
            return false;
        }
    }
    for (size_t i = 0; i < a.height.size(); ++i) {
        if (std::fabs(a.height[i] - b.height[i]) > kProfileMatchEpsilon) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Build a cross-section column from a half-section laid out along a ray
 *
 * @param origin  Innermost point, on the carriageway edge
 * @param outward Unit direction the strips run in
 * @param half    Strips ordered outward
 * @param v       Travelled distance for the column's V coordinate
 * @param flip    Emit the boundaries right-to-left instead of left-to-right
 */
[[nodiscard]] SweepColumn ray_column(const glm::dvec2& origin,
                                     const glm::dvec2& outward,
                                     const HalfSection& half,
                                     double v,
                                     bool flip) {
    SweepColumn col;
    col.v = v;
    double d = 0.0;
    col.pos.push_back(origin);
    col.height.push_back(half.height.front());
    for (size_t i = 0; i < half.strips(); ++i) {
        d += half.width[i];
        col.pos.push_back(origin + outward * d);
        col.height.push_back(half.height[i + 1u]);
    }
    if (flip) {
        std::reverse(col.pos.begin(), col.pos.end());
        std::reverse(col.height.begin(), col.height.end());
    }
    return col;
}

/// Reverse a column's boundary order, for a band swept the other way round
[[nodiscard]] SweepColumn flip_column(SweepColumn col) {
    std::reverse(col.pos.begin(), col.pos.end());
    std::reverse(col.height.begin(), col.height.end());
    return col;
}

} // namespace

// ============================================================================
// Dead ends
// ============================================================================

/**
 * @brief Cap the single arm of a degree-1 node
 *
 * Invariant: every shape emitted here shares its weld edge with the arm's own
 * end cross-section, taken through offset_point() on the arm's end station at
 * the arm's own laterals. Nothing is placed on an idealised circle where the
 * ribbon's real edge would disagree with it.
 *
 * The turning circle and the cul-de-sac bulb are ONE construction with two
 * settings. Both are a disc of radius R about a centre offset FORWARD from the
 * node by `o`, cut back wherever it would otherwise lie on top of the ribbon it
 * caps:
 *
 * - Turning circle: `o = 0`. The disc straddles the node, so its two backward
 *   lobes are notched out along the ribbon's real carriageway edges, out to the
 *   point `sqrt(R^2 - w^2)` back where those edges cross the circle. What is
 *   left is the pavement a turning circle actually adds, with no coplanar
 *   overlap to z-fight.
 * - Cul-de-sac bulb: `o = sqrt(R^2 - w^2)`, which is exactly the offset that
 *   puts the arm's two carriageway corners ON the circle. The notch then has
 *   zero depth, the disc closes on the arm's end cross-section itself, and the
 *   bulb sits wholly beyond the last surveyed point instead of swallowing the
 *   end of the street.
 *
 * The ring is star-shaped about the node in both settings -- the bulb's region
 * is a circle cut by a chord through the node, the turning circle's is a disc
 * with a slot removed along the node's own cross-section line -- so a fan about
 * the node triangulates either without an ear-clipping pass.
 */
Mesh build_dead_end(const RoadGraph& graph,
                    const std::vector<Centerline>& centerlines,
                    const std::vector<RoadProfile>& profiles,
                    GraphNodeId node,
                    float height,
                    const DeadEndConfig& cfg,
                    std::vector<glm::dvec2>* out_outline) {
    Mesh out;
    if (out_outline != nullptr) {
        out_outline->clear();
    }

    const std::vector<GraphEdge>& edges = graph.edges();
    if (node >= graph.nodes().size()) {
        return out;
    }
    if (centerlines.size() != edges.size() || profiles.size() != edges.size()) {
        return out;
    }

    const GraphNode& n = graph.node(node);
    if (n.degree() != 1u) {
        return out;
    }

    const Arm& arm = n.arms[0];
    if (arm.edge >= edges.size()) {
        return out;
    }
    const GraphEdge& edge = edges[arm.edge];
    const Centerline& cl = centerlines[arm.edge];
    if (!cl.is_valid()) {
        return out;
    }

    // The cap's own frame points AWAY from the road, so an arm whose node end is
    // its `from` runs against it and has its cross-section mirrored.
    const bool mirror = arm.at_start;
    const SectionModel model = model_from_profile(profiles[arm.edge], mirror);
    if (model.empty()) {
        return out;
    }

    const Station& sn = arm.at_start ? cl.stations.front() : cl.stations.back();
    const glm::dvec2 outward = arm.at_start ? -sn.tangent : sn.tangent;
    if (glm::dot(outward, outward) <= 0.0) {
        return out;
    }
    const glm::dvec2 side = left_of(outward);
    const glm::dvec2 origin = sn.position;
    const double base = static_cast<double>(height);

    MeshAccum acc;

    const double w = carriageway_half(model);

    const bool turning = n.is_turning_circle;
    const bool bulb = !turning && cfg.bulb_for_residential &&
                      (edge.type == RoadType::Residential || edge.type == RoadType::Service);

    // ------------------------------------------------------------------------
    // Which lateral the disc is cut back to.
    //
    // A TURNING CIRCLE straddles the node, so its two backward lobes lie on the
    // ribbon. The disc is FLAT, and so is every strip the profile emits at
    // height 0 -- the gutter beside each lane, a shoulder, a cycle or parking
    // lane -- so notching only to the lane edge leaves those lobes coplanar with
    // the gutter and z-fighting against it. The notch is taken at the outermost
    // coplanar boundary instead.
    //
    // A BULB is offset forward until its circle passes through the mouth corners
    // and has no backward lobe at all, so its cut stays at the CARRIAGEWAY edge:
    // that is the lateral its outboard ring starts its own gutter strip at, and
    // moving it out would double-cover the gutter instead of continuing it.
    // ------------------------------------------------------------------------
    const double notch_w = turning ? coplanar_half(model) : w;

    /**
     * Close every raised strip down to the carriageway at the arm's end station.
     *
     * A ribbon is a shell: a strip lying flat on the carriageway plane has no
     * open end, but one standing above it -- a curb face, a raised sidewalk --
     * terminates as a floating slab with an open boundary edge. This is the
     * vertical face that closes it, welded to offset_point() on the arm's own end
     * station at the arm's own laterals so it shares its weld edge with the
     * ribbon exactly.
     *
     * Every path through this function needs it except the matched bulb, whose
     * outboard ring and gussets carry the raised strips around the cap instead.
     */
    const auto emit_end_face = [&]() {
        double walked = 0.0;
        for (size_t k = 0; k < model.strips(); ++k) {
            const double h_left = model.height[k];
            const double h_right = model.height[k + 1u];
            const glm::dvec2 p_left = offset_point(sn, model.edge_lateral(model.lateral[k]));
            const glm::dvec2 p_right = offset_point(sn, model.edge_lateral(model.lateral[k + 1u]));
            const double width = std::sqrt(dist_sq(p_left, p_right));

            if (h_left <= kZeroExtent && h_right <= kZeroExtent) {
                walked += width;
                continue;
            }

            const UVTiling tiling = uv_tiling(model.material[k]);
            const double u_scale = (tiling.u_metres > 0.0f) ? static_cast<double>(tiling.u_metres) : 1.0;
            const double v_scale = (tiling.v_metres > 0.0f) ? static_cast<double>(tiling.v_metres) : 1.0;

            const float v0 = static_cast<float>(walked / v_scale);
            const float v1 = static_cast<float>((walked + width) / v_scale);

            const uint32_t t1 = acc.vertex(p_left, base + h_left,
                                           glm::vec2(static_cast<float>(h_left / u_scale), v0));
            const uint32_t t2 = acc.vertex(p_right, base + h_right,
                                           glm::vec2(static_cast<float>(h_right / u_scale), v1));
            const uint32_t b1 = acc.vertex(p_left, base, glm::vec2(0.0f, v0));
            const uint32_t b2 = acc.vertex(p_right, base, glm::vec2(0.0f, v1));

            // Faces along +outward: the direction the open end looks in.
            acc.material(model.material[k]);
            acc.triangle(t1, b2, b1);
            acc.triangle(t1, t2, b2);

            walked += width;
        }
    };

    if ((turning || bulb) && notch_w > kZeroExtent) {
        // Never smaller than the road it terminates, and always leaving an arc.
        double radius = std::max(cfg.turning_circle_radius,
                                 0.5 * static_cast<double>(profiles[arm.edge].total_width()));
        radius = std::max(radius, notch_w + 0.25);

        const double reach = std::sqrt(std::max(radius * radius - notch_w * notch_w, 0.0));
        const double offset = bulb ? reach : 0.0;
        const double notch = std::max(reach - offset, 0.0);
        const glm::dvec2 centre = origin + outward * offset;

        std::vector<Station> run = end_run(cl, arm.at_start, std::min(notch, 0.9 * cl.length()));
        if (run.empty()) {
            run.push_back(sn);
        }

        std::vector<glm::dvec2> left_leg;
        std::vector<glm::dvec2> right_leg;
        left_leg.reserve(run.size());
        right_leg.reserve(run.size());
        for (const Station& s : run) {
            left_leg.push_back(offset_point(s, model.edge_lateral(notch_w)));
            right_leg.push_back(offset_point(s, model.edge_lateral(-notch_w)));
        }

        const double th_r = angle_in_basis(right_leg.back() - centre, outward, side);
        double th_l = angle_in_basis(left_leg.back() - centre, outward, side);
        while (th_l <= th_r) {
            th_l += 2.0 * kPi;
        }

        std::vector<glm::dvec2> ring;
        append_arc(ring, centre, radius, outward, side, th_r, th_l,
                   right_leg.back(), left_leg.back());
        for (size_t i = left_leg.size(); i-- > 0;) {
            push_unique(ring, left_leg[i]);
        }
        for (size_t i = 0; i < right_leg.size(); ++i) {
            push_unique(ring, right_leg[i]);
        }
        close_ring(ring);

        if (ring.size() >= 3u) {
            if (ring_signed_area2(ring) < 0.0) {
                std::reverse(ring.begin(), ring.end());
            }
            triangulate_ring(acc, ring, origin, base, lane_material(model));
        }

        // --------------------------------------------------------------------
        // Curb and sidewalk carried around the bulb.
        // --------------------------------------------------------------------
        const HalfSection outboard_left = outboard_half(model, true);
        const HalfSection outboard_right = outboard_half(model, false);

        bool outboard_ring_drawn = false;
        if (bulb && halves_match(outboard_left, outboard_right)) {
            const HalfSection& section = outboard_left;

            std::vector<MaterialId> materials(section.strips());
            std::vector<uint8_t> vertical(section.strips(), 0u);
            for (size_t i = 0; i < section.strips(); ++i) {
                materials[i] = section.material[i];
                vertical[i] = (section.kind[i] == StripKind::CurbFace) ? 1u : 0u;
            }

            const double sweep = th_l - th_r;
            int segments = static_cast<int>(std::ceil(sweep / (2.0 * kPi) * kSegmentsPerTurn));
            segments = std::max(segments, 2);

            std::vector<SweepColumn> ring_cols;
            ring_cols.reserve(static_cast<size_t>(segments) + 1u);
            for (int k = 0; k <= segments; ++k) {
                const double th = th_r + sweep * (static_cast<double>(k) / static_cast<double>(segments));
                const glm::dvec2 radial = outward * std::cos(th) + side * std::sin(th);
                const glm::dvec2 p = centre + radial * radius;
                const double v = radius * (th - th_r);
                ring_cols.push_back(ray_column(p, radial, section, v, false));
            }

            // Gussets. The ring's cross-section runs radially out of the bulb
            // while the arm's runs laterally out of the road, so the two meet at
            // an angle even where they share their innermost point. A band
            // between them closes that wedge; without it every cul-de-sac shows
            // a triangular hole either side of its mouth.
            //
            // Both gussets sweep about the mouth corner they pivot on, and that
            // rotation runs the OPPOSITE way round from the ring's own sweep
            // about the bulb centre: the bulb is to the right of the travel, not
            // the left. The columns are therefore reversed, and the strip arrays
            // with them, or the wedge comes out inside-out and its curb faces
            // point into the pavement.
            std::vector<MaterialId> gusset_materials(materials.rbegin(), materials.rend());
            std::vector<uint8_t> gusset_vertical(vertical.rbegin(), vertical.rend());

            const glm::dvec2 mouth_right = offset_point(sn, model.edge_lateral(-w));
            const glm::dvec2 mouth_left = offset_point(sn, model.edge_lateral(w));

            const SweepColumn arm_right = ray_column(mouth_right, -side, outboard_right, 0.0, false);
            const SweepColumn arm_left = ray_column(mouth_left, side, outboard_left, 0.0, false);

            if (!ring_cols.empty()) {
                const double lead =
                    std::sqrt(dist_sq(arm_right.pos.back(), ring_cols.front().pos.back()));
                SweepColumn start_arm = flip_column(arm_right);
                start_arm.v = -lead;
                SweepColumn start_ring = flip_column(ring_cols.front());
                emit_sweep(acc, gusset_materials, gusset_vertical,
                           std::vector<SweepColumn>{ start_arm, start_ring }, base);

                const double tail =
                    std::sqrt(dist_sq(ring_cols.back().pos.back(), arm_left.pos.back()));
                SweepColumn end_ring = flip_column(ring_cols.back());
                SweepColumn end_arm = flip_column(arm_left);
                end_arm.v = ring_cols.back().v + tail;
                emit_sweep(acc, gusset_materials, gusset_vertical,
                           std::vector<SweepColumn>{ end_ring, end_arm }, base);
            }

            emit_sweep(acc, materials, vertical, ring_cols, base);
            outboard_ring_drawn = true;
        }

        // A turning circle never takes the branch above, and an asymmetric
        // profile fails its halves_match() test, so in both cases nothing has
        // carried the raised strips around the cap. The disc is flat, so the
        // arm's curb face and sidewalk would end in mid-air at the node station
        // with an open boundary edge -- the ribbon is a shell, and the viewer
        // sees its backfaces through the gap. Close them down to the carriageway
        // with the same end face the flat cap uses.
        if (!outboard_ring_drawn) {
            emit_end_face();
        }

        // The carve footprint. The disc reaches well past the ribbon's own end
        // cross-section, so without this the ground under a bulb keeps its raw
        // procedural surface and punches through the cap.
        if (out_outline != nullptr && ring.size() >= 3u) {
            std::vector<glm::dvec2> footprint = ring;
            // The disc is the CARRIAGEWAY; the outboard strips stand outside it.
            // Widening by the profile's own outboard reach keeps the footprint a
            // superset of everything the cap draws.
            const double outboard = std::max(0.0,
                0.5 * static_cast<double>(profiles[arm.edge].total_width()) - w);
            if (outboard > kZeroExtent) {
                for (glm::dvec2& q : footprint) {
                    const glm::dvec2 radial = q - centre;
                    const double len = std::sqrt(glm::dot(radial, radial));
                    if (len > kZeroExtent) {
                        q += radial * (outboard / len);
                    }
                }
            }
            if (ring_signed_area2(footprint) < 0.0) {
                std::reverse(footprint.begin(), footprint.end());
            }
            *out_outline = std::move(footprint);
        }
    } else {
        // --------------------------------------------------------------------
        // Flat cap: the vertical end face of the cross-section.
        //
        // A ribbon is a shell, so a profile that lies flat on the carriageway
        // plane has no open end to close and correctly emits nothing. What IS
        // open is every strip standing above that plane -- a raised sidewalk
        // ends as a floating slab -- and this closes each of those down to the
        // carriageway, inventing no road feature that the data does not carry.
        // --------------------------------------------------------------------
        emit_end_face();
    }

    out = acc.take();
    return out;
}

} // namespace stratum::osm::road
