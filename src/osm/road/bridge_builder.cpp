/**
 * @file bridge_builder.cpp
 * @brief Implementation of the deck slab, parapets and piers of a bridge edge
 *
 * The corridor extruder already swept the running surface along the lifted
 * centerline, so everything here hangs off that surface and never re-emits it.
 * Three closed shells are produced, in this order:
 *
 *   1. The deck slab: a flat underside `deck_thickness` below the carriageway
 *      surface, the two long fascia walls closing it to the profile's outer
 *      edges, and an end cap at each abutment that follows the profile's stepped
 *      cross-section down to the underside. Together with the corridor's own top
 *      surface this is a closed box.
 *   2. The parapets: an open-bottomed box along each outer edge, standing on the
 *      deck it is closed against.
 *   3. The piers: closed boxes dropped from the underside to the terrain.
 *
 * ### Winding is measured, not assumed
 *
 * The world mapping `(x, y_2d) -> (x, height, -y_2d)` flips handedness, so the
 * orientation of a ring in local 2D is NOT its orientation in world XZ. Rather
 * than reason about that per face and get one of them backwards, every face in
 * this file goes through Sink::quad(), which is handed the direction the face is
 * meant to point, computes the quad's own Newell normal, and reverses the winding
 * when the two disagree. Correct facing is therefore a property of the helper and
 * not of the twelve call sites. The renderer's front face is counter-clockwise,
 * so an outward-pointing normal and a counter-clockwise winding are the same
 * statement.
 *
 * A bridge is the one piece of road geometry routinely seen from below, which is
 * why the check is made mechanical here and nowhere else in the pipeline.
 *
 * ### Reference heights
 *
 * `station_heights[i]` is the CARRIAGEWAY surface, so the underside is
 * `station_heights[i] - deck_thickness`, flat across the section. The profile's
 * outer edges may sit above the carriageway -- a raised sidewalk does -- so the
 * fascia runs from the underside up to `station_heights[i] + outer strip height`,
 * which is exactly where the corridor's outermost strip edge is, and the two meet
 * with no crack.
 *
 * ### What is deliberately not watertight
 *
 * Two openings are by design. The deck shell has no top, because the corridor's
 * running surface IS its top and a second copy z-fights across the whole span;
 * every boundary edge of the shell therefore lies exactly on that surface, and a
 * boundary edge anywhere else would be a real hole. The parapets have no bottom,
 * for the same reason against the same surface.
 *
 * One more is inherited rather than chosen: on a hairpin tight enough to bind
 * Station::lateral_min / lateral_max, offset_point() collapses the profile onto
 * the bound, and a parapet whose two laterals both clamp becomes locally zero
 * thickness. Dropping that band would leave a gap in the wall, which is worse.
 *
 * Everything here lives in stratum_core: no SDL, no ImGui, no rendering API.
 */

#include "osm/road/bridge_builder.hpp"

#include "osm/road/corridor.hpp"   // uv_tiling(): the frozen UV Convention table

#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace stratum::osm::road {

namespace {

// ============================================================================
// Tolerances
// ============================================================================

/// Squared length of a raw face cross product below which a face is dropped
constexpr double kDegenerateCrossSq = 1e-16;

/// Widths and heights at or below this count as zero, metres
constexpr double kZeroExtent = 1e-6;

/// Lateral or vertical distance below which two section points are the same point
constexpr double kSectionWeldEpsilon = 1e-9;

/**
 * @brief How far a pier is pushed into the deck above and the ground below, metres
 *
 * The pier top is a flat quad while the deck underside is ruled along a grade, and
 * the ground under the four corners is only as flat as the height field. A
 * centimetre of interpenetration at both ends costs nothing -- both surfaces are
 * inside solid geometry -- and removes the hairline crack that an exactly flush
 * fit shows at grazing angles.
 */
constexpr double kPierEmbed = 0.01;

/**
 * @brief How far the deck must be under the terrain to count as buried, metres
 *
 * Only a deck buried at EVERY station is rejected. An abutment legitimately meets
 * the ground it springs from, so any looser test throws away ordinary bridges.
 */
constexpr double kBuriedEpsilon = 0.01;

/**
 * @brief Largest share of the deck width one parapet may occupy
 *
 * A 0.3 m parapet on a 2 m footbridge would leave a 1.4 m gap between the two
 * walls, which is defensible; the same parapet on a 0.5 m profile would not leave
 * one at all. Clamping keeps the two walls from meeting in the middle whatever
 * the profile turns out to be.
 */
constexpr double kMaxParapetWidthFraction = 0.4;

/**
 * @brief Most bays a span may be divided into, however small the spacing
 *
 * The bay count comes from a config value that nothing validates, and a spacing
 * of a millimetre on a kilometre of viaduct would otherwise ask for a million
 * piers. Far beyond any real structure, so it bites only on a bad config.
 */
constexpr long kMaxPierBays = 512;

// ============================================================================
// Coordinate mapping
// ============================================================================

/**
 * @brief The codebase-wide 2D-to-3D mapping for POSITIONS, Y up
 *
 * (x, y_2d) -> (x, height, -y_2d). Identical to corridor.cpp; the structure has
 * to land on the same plane as the surface it hangs from.
 */
[[nodiscard]] inline glm::dvec3 to_world(const glm::dvec2& p, double height) {
    return glm::dvec3(p.x, height, -p.y);
}

/**
 * @brief The same mapping for DIRECTIONS
 *
 * The point map is linear, so a 2D direction maps by dropping the translation:
 * (dx, dy) -> (dx, 0, -dy). Used for the outward direction of every vertical
 * face, which is where the handedness flip would otherwise bite.
 */
[[nodiscard]] inline glm::dvec3 dir_to_world(const glm::dvec2& d) {
    return glm::dvec3(d.x, 0.0, -d.y);
}

// ============================================================================
// Mesh sink
// ============================================================================

/**
 * @brief Accumulates oriented quads into a submeshed Mesh
 *
 * Vertices are not shared between quads. Structure faces meet at hard creases --
 * an underside against a fascia, a parapet top against its wall -- so a shared
 * vertex would need a split normal anyway, and the UV of a shared corner differs
 * between the two faces it belongs to. P7 welds what is safe to weld.
 */
class Sink {
public:
    /// Material attributed to every quad emitted from here on
    void set_material(MaterialId material) { m_material = material; }

    /**
     * @brief Emit one quad, wound so that it faces @p outward
     *
     * @param p       Four corners in world space, in ring order around the face
     * @param uv      Texture coordinate per corner, parallel to @p p
     * @param outward Direction the finished face must point. Need not be unit
     *                length or exactly perpendicular; only its sign against the
     *                quad's own normal is used.
     */
    void quad(const std::array<glm::dvec3, 4>& p,
              const std::array<glm::vec2, 4>& uv,
              const glm::dvec3& outward) {
        // Newell normal of the quad: the sum of its two triangle normals, so a
        // quad that has collapsed to a triangle still reports the survivor's
        // orientation instead of nothing.
        const glm::dvec3 face = glm::cross(p[1] - p[0], p[2] - p[0]) +
                                glm::cross(p[2] - p[0], p[3] - p[0]);
        const double len_sq = glm::dot(face, face);
        if (len_sq <= kDegenerateCrossSq) {
            return;     // no area at all: a strip of zero width and zero height
        }

        const bool flip = glm::dot(face, outward) < 0.0;
        const glm::dvec3 unit = (flip ? -face : face) / std::sqrt(len_sq);
        const glm::vec3 normal(static_cast<float>(unit.x),
                               static_cast<float>(unit.y),
                               static_cast<float>(unit.z));

        const std::array<size_t, 4> order = flip ? std::array<size_t, 4>{ 3, 2, 1, 0 }
                                                 : std::array<size_t, 4>{ 0, 1, 2, 3 };

        const uint32_t base = static_cast<uint32_t>(m_mesh.vertices.size());
        for (const size_t src : order) {
            Vertex v{};
            v.position = glm::vec3(static_cast<float>(p[src].x),
                                   static_cast<float>(p[src].y),
                                   static_cast<float>(p[src].z));
            v.normal = normal;
            v.uv = uv[src];
            v.color = glm::vec4(1.0f);
            m_mesh.vertices.push_back(v);
        }

        const uint32_t index_start = static_cast<uint32_t>(m_mesh.indices.size());
        emit_triangle(base + 0u, base + 1u, base + 2u);
        emit_triangle(base + 0u, base + 2u, base + 3u);

        const uint32_t added = static_cast<uint32_t>(m_mesh.indices.size()) - index_start;
        if (added == 0u) {
            return;
        }
        if (!m_mesh.submeshes.empty() && m_mesh.submeshes.back().material == m_material) {
            m_mesh.submeshes.back().index_count += added;
        } else {
            m_mesh.submeshes.push_back(SubMesh{ index_start, added, m_material });
        }
    }

    /// True when nothing survived the degeneracy checks
    [[nodiscard]] bool empty() const { return m_mesh.indices.empty(); }

    /// Finish the mesh: one contiguous range per material, bounds, tangents
    [[nodiscard]] Mesh take() {
        m_mesh.sort_submeshes_by_material();
        m_mesh.compute_bounds();
        m_mesh.compute_tangents();
        return std::move(m_mesh);
    }

private:
    void emit_triangle(uint32_t i0, uint32_t i1, uint32_t i2) {
        const glm::dvec3 p0(m_mesh.vertices[i0].position);
        const glm::dvec3 p1(m_mesh.vertices[i1].position);
        const glm::dvec3 p2(m_mesh.vertices[i2].position);
        const glm::dvec3 face = glm::cross(p1 - p0, p2 - p0);
        if (glm::dot(face, face) <= kDegenerateCrossSq) {
            return;     // half of a quad that collapsed to a triangle
        }
        m_mesh.indices.push_back(i0);
        m_mesh.indices.push_back(i1);
        m_mesh.indices.push_back(i2);
    }

    Mesh m_mesh;
    MaterialId m_material = MaterialId::Default;
};

// ============================================================================
// Profile queries
// ============================================================================

/**
 * @brief The profile's top boundary as a lateral/height polyline
 *
 * `x` is the signed lateral coordinate, positive to the LEFT, running from
 * RoadProfile::left_edge_offset() downwards. `y` is metres above the carriageway
 * surface. Consecutive duplicates are dropped, but a pair sharing a lateral and
 * differing in height is kept: that is a vertical curb face, and it is a real
 * edge of the end cap.
 *
 * @param profile Cross-section to walk
 * @return At least two points for any valid profile
 */
[[nodiscard]] std::vector<glm::dvec2> section_polyline(const RoadProfile& profile) {
    std::vector<glm::dvec2> section;
    section.reserve(profile.strips.size() * 2u);

    const auto push = [&section](double lateral, double height) {
        if (!section.empty()) {
            const glm::dvec2& back = section.back();
            if (std::abs(back.x - lateral) <= kSectionWeldEpsilon &&
                std::abs(back.y - height) <= kSectionWeldEpsilon) {
                return;
            }
        }
        section.emplace_back(lateral, height);
    };

    double lateral = static_cast<double>(profile.left_edge_offset());
    for (const Strip& strip : profile.strips) {
        const double lat_left = lateral;
        const double lat_right = lat_left - static_cast<double>(strip.width);
        push(lat_left, static_cast<double>(strip.height_left));
        push(lat_right, static_cast<double>(strip.height_right));
        lateral = lat_right;
    }
    return section;
}

/**
 * @brief Lowest surface height over a lateral range, metres above the carriageway
 *
 * The parapet stands on whatever the profile puts under it, which across
 * BridgeConfig::parapet_width may be a sidewalk, a curb face, and a gutter at
 * once. Seating it on the LOWEST of those buries the wall's foot in the deck
 * rather than leaving it hovering over the lower part of the range.
 *
 * @param section Profile boundary from section_polyline()
 * @param lat_a   One end of the lateral range
 * @param lat_b   The other end
 * @return Minimum height over the range; 0 when the section is empty
 */
[[nodiscard]] double section_min_height(const std::vector<glm::dvec2>& section,
                                        double lat_a, double lat_b) {
    if (section.empty()) {
        return 0.0;
    }
    const double lo = std::min(lat_a, lat_b);
    const double hi = std::max(lat_a, lat_b);

    double result = std::numeric_limits<double>::max();
    const auto consider = [&result](double h) { result = std::min(result, h); };

    for (size_t i = 0; i + 1 < section.size(); ++i) {
        const glm::dvec2& a = section[i];
        const glm::dvec2& b = section[i + 1];

        const double seg_lo = std::min(a.x, b.x);
        const double seg_hi = std::max(a.x, b.x);
        if (seg_hi < lo || seg_lo > hi) {
            continue;
        }

        // Height is linear within a segment, so the extremes over the clipped
        // range are at the clipped ends.
        const double span = b.x - a.x;
        if (std::abs(span) <= kSectionWeldEpsilon) {
            consider(a.y);
            consider(b.y);
            continue;
        }
        for (const double x : { std::max(seg_lo, lo), std::min(seg_hi, hi) }) {
            const double t = (x - a.x) / span;
            consider(a.y + (b.y - a.y) * t);
        }
    }

    if (result == std::numeric_limits<double>::max()) {
        // The range falls entirely outside the profile, which only happens for a
        // clamped parapet on a degenerate section. The outer edge is the honest
        // answer.
        return section.front().y;
    }
    return result;
}

// ============================================================================
// Longitudinal sampling
// ============================================================================

/**
 * @brief Mean of two station normals, safe against a 180 degree reversal
 *
 * The mean is only used as the OUTWARD hint handed to Sink::quad, but a hint of
 * NaN silently disables the orientation check -- every comparison against NaN is
 * false, so nothing is ever flipped -- and a hairpin whose two normals cancel
 * would produce exactly that. Falling back to the first normal keeps the hint
 * meaningful.
 *
 * @param a First station normal
 * @param b Second station normal
 * @return Unit mean, or @p a normalised when the two cancel
 */
[[nodiscard]] glm::dvec2 mean_normal(const glm::dvec2& a, const glm::dvec2& b) {
    const glm::dvec2 sum = a + b;
    const double len_sq = glm::dot(sum, sum);
    if (len_sq > 1e-12) {
        return sum / std::sqrt(len_sq);
    }
    const double a_len_sq = glm::dot(a, a);
    return (a_len_sq > 0.0) ? a / std::sqrt(a_len_sq) : glm::dvec2(0.0, 1.0);
}

/**
 * @brief Position, frame and surface height at an arbitrary arclength
 */
struct SpanSample {
    glm::dvec2 position{0.0};
    glm::dvec2 tangent{1.0, 0.0};
    glm::dvec2 normal{0.0, 1.0};    ///< unit LEFT normal of the band, not the miter bisector
    double surface = 0.0;           ///< world Y of the carriageway surface
};

/**
 * @brief Sample the centerline between stations
 *
 * Zero-length bands -- the two halves of a bevel pair share an arclength -- are
 * skipped, because they have no chord to take a direction from. The frame comes
 * from the band's own chord rather than from Station::normal: a pier is a
 * free-standing box in the middle of a band and wants the local direction of
 * travel, not the bisector that station happens to carry.
 *
 * @param cl              Centerline to sample
 * @param station_heights Carriageway surface per station; same size as cl.stations
 * @param arclength       Where to sample, clamped into the centerline's range
 * @return The interpolated sample
 */
[[nodiscard]] SpanSample sample_span(const Centerline& cl,
                                     const std::vector<float>& station_heights,
                                     double arclength) {
    SpanSample out;
    const std::vector<Station>& st = cl.stations;

    size_t band = 0;
    bool found = false;
    for (size_t i = 0; i + 1 < st.size(); ++i) {
        if (st[i + 1].arclength - st[i].arclength <= 0.0) {
            continue;
        }
        band = i;
        found = true;
        if (arclength <= st[i + 1].arclength) {
            break;
        }
    }
    if (!found) {
        out.position = st.front().position;
        out.tangent = st.front().tangent;
        out.normal = st.front().normal;
        out.surface = static_cast<double>(station_heights.front());
        return out;
    }

    const Station& a = st[band];
    const Station& b = st[band + 1];
    const double span = b.arclength - a.arclength;
    const double t = std::min(std::max((arclength - a.arclength) / span, 0.0), 1.0);

    out.position = a.position + (b.position - a.position) * t;
    out.surface = static_cast<double>(station_heights[band]) +
                  (static_cast<double>(station_heights[band + 1]) -
                   static_cast<double>(station_heights[band])) * t;

    const glm::dvec2 chord = b.position - a.position;
    const double chord_len = std::sqrt(glm::dot(chord, chord));
    if (chord_len > kZeroExtent) {
        out.tangent = chord / chord_len;
    } else {
        out.tangent = a.tangent;
    }
    // Left normal of the direction of travel: the tangent turned a quarter turn
    // counter-clockwise in the 2D plane, matching Station::normal on a straight.
    out.normal = glm::dvec2(-out.tangent.y, out.tangent.x);
    return out;
}

} // namespace

// ============================================================================
// Construction
// ============================================================================

/**
 * @brief Deck slab, parapets and piers for one bridge edge
 *
 * Invariant: every emitted face points out of the solid it bounds, the deck shell
 * is closed once the corridor's own running surface is counted as its top, and
 * nothing is emitted at all for an edge that is not a bridge or whose deck lies
 * under the terrain it is supposed to span.
 */
Mesh build_bridge(const GraphEdge& edge,
                  const Centerline& cl,
                  const RoadProfile& profile,
                  const std::vector<float>& station_heights,
                  const std::function<float(double, double)>& terrain_at,
                  const BridgeConfig& cfg) {
    // ------------------------------------------------------------------------
    // Preconditions. Every one of these is a normal occurrence on real data, so
    // each returns an empty mesh and lets the caller skip the edge.
    // ------------------------------------------------------------------------
    if (!edge.is_bridge) {
        spdlog::debug("build_bridge: way {} is not a bridge; nothing emitted", edge.source_way);
        return Mesh{};
    }
    if (!cl.is_valid() || !profile.is_valid()) {
        return Mesh{};
    }

    const size_t station_count = cl.stations.size();
    if (station_heights.size() != station_count) {
        spdlog::warn("build_bridge: way {} has {} station heights for {} stations; "
                     "a bridge is defined by its height and cannot be guessed",
                     edge.source_way, station_heights.size(), station_count);
        return Mesh{};
    }
    for (const float h : station_heights) {
        if (!std::isfinite(h)) {
            spdlog::warn("build_bridge: way {} has a non-finite station height", edge.source_way);
            return Mesh{};
        }
    }

    const double total_width = static_cast<double>(profile.total_width());
    if (total_width <= kZeroExtent) {
        spdlog::warn("build_bridge: way {} has a zero-width profile", edge.source_way);
        return Mesh{};
    }

    const std::vector<Station>& stations = cl.stations;
    const double span_start = stations.front().arclength;
    const double span_end = stations.back().arclength;
    const double span = span_end - span_start;

    const double left_edge = static_cast<double>(profile.left_edge_offset());
    const double right_edge = left_edge - total_width;
    const double outer_h_left = static_cast<double>(profile.strips.front().height_left);
    const double outer_h_right = static_cast<double>(profile.strips.back().height_right);

    const double deck_thickness = std::max(static_cast<double>(cfg.deck_thickness), 0.0);
    const bool has_slab = deck_thickness > kZeroExtent;
    if (!has_slab) {
        spdlog::warn("build_bridge: way {} has deck_thickness {:.3f}; emitting no slab and no "
                     "piers, since neither has a depth to occupy",
                     edge.source_way, cfg.deck_thickness);
    }

    const std::vector<glm::dvec2> section = section_polyline(profile);

    // ------------------------------------------------------------------------
    // Terrain, and the buried-deck rejection.
    //
    // A deck under the ground at EVERY station is bad data -- a bridge tag on a
    // way the elevation solver never lifted, or a layer the extract got wrong --
    // and the structure would be a box inside a hill. Reported and dropped. A
    // deck buried at SOME stations is an abutment meeting its embankment, which
    // is what a bridge end looks like.
    // ------------------------------------------------------------------------
    std::vector<double> terrain_at_station;
    const bool has_terrain = static_cast<bool>(terrain_at);
    if (has_terrain) {
        terrain_at_station.resize(station_count);
        size_t buried = 0;
        for (size_t i = 0; i < station_count; ++i) {
            const glm::dvec2& p = stations[i].position;
            const double ground = static_cast<double>(terrain_at(p.x, p.y));
            terrain_at_station[i] = std::isfinite(ground) ? ground
                                                          : -std::numeric_limits<double>::max();
            if (static_cast<double>(station_heights[i]) < terrain_at_station[i] - kBuriedEpsilon) {
                ++buried;
            }
        }
        if (buried == station_count) {
            spdlog::warn("build_bridge: way {} has its deck below the terrain at all {} stations; "
                         "emitting nothing rather than a structure buried in a hill",
                         edge.source_way, station_count);
            return Mesh{};
        }
    }

    Sink sink;

    const UVTiling deck_uv = uv_tiling(MaterialId::BridgeDeck);
    const double deck_u = (deck_uv.u_metres > 0.0f) ? static_cast<double>(deck_uv.u_metres) : 1.0;
    const double deck_v = (deck_uv.v_metres > 0.0f) ? static_cast<double>(deck_uv.v_metres) : 1.0;

    // ------------------------------------------------------------------------
    // 1. Deck slab.
    //
    // The top is the corridor's carriageway and is deliberately absent: a second
    // copy of it z-fights across the whole span. What is emitted is the
    // underside, the two fascia walls, and an end cap at each abutment.
    // ------------------------------------------------------------------------
    if (has_slab) {
        sink.set_material(MaterialId::BridgeDeck);

        // Plan position of every section boundary at every station. The underside
        // is subdivided at the SAME laterals as the end caps, so the caps' bottom
        // edges land on real underside edges instead of in the middle of one. A
        // T-junction there is not a hole, but it cracks the moment anything welds
        // or displaces the vertices, and the grid costs a few hundred triangles on
        // a structure that already carries thousands.
        const size_t lat_count = section.size();
        std::vector<glm::dvec2> plan(station_count * lat_count);
        std::vector<double> under(station_count);
        for (size_t i = 0; i < station_count; ++i) {
            const Station& s = stations[i];
            under[i] = static_cast<double>(station_heights[i]) - deck_thickness;
            for (size_t k = 0; k < lat_count; ++k) {
                plan[i * lat_count + k] = offset_point(s, section[k].x);
            }
        }
        const auto at = [&](size_t i, size_t k) -> const glm::dvec2& {
            return plan[i * lat_count + k];
        };

        // Underside. Faces straight down, which is the one direction a bridge is
        // actually inspected from.
        for (size_t i = 0; i + 1 < station_count; ++i) {
            const float v0 = static_cast<float>(stations[i].arclength / deck_v);
            const float v1 = static_cast<float>(stations[i + 1].arclength / deck_v);

            for (size_t k = 0; k + 1 < lat_count; ++k) {
                const float u_a = static_cast<float>((left_edge - section[k].x) / deck_u);
                const float u_b = static_cast<float>((left_edge - section[k + 1].x) / deck_u);
                sink.quad({ to_world(at(i, k), under[i]),
                            to_world(at(i, k + 1), under[i]),
                            to_world(at(i + 1, k + 1), under[i + 1]),
                            to_world(at(i + 1, k), under[i + 1]) },
                          { glm::vec2(u_a, v0), glm::vec2(u_b, v0),
                            glm::vec2(u_b, v1), glm::vec2(u_a, v1) },
                          glm::dvec3(0.0, -1.0, 0.0));
            }
        }

        // Fascia, closing the underside up to the profile's outer edges, which is
        // exactly where the corridor's outermost strip edge sits. U runs UP the
        // wall, as the frozen convention has it for every vertical face; V keeps
        // the road's own arclength so the fascia and the surface above it stay in
        // step.
        for (size_t i = 0; i + 1 < station_count; ++i) {
            const Station& sa = stations[i];
            const Station& sb = stations[i + 1];
            const float v0 = static_cast<float>(sa.arclength / deck_v);
            const float v1 = static_cast<float>(sb.arclength / deck_v);
            const glm::dvec2 band_normal = mean_normal(sa.normal, sb.normal);

            for (const bool is_left : { true, false }) {
                const size_t k = is_left ? 0u : lat_count - 1u;
                const double outer_h = is_left ? outer_h_left : outer_h_right;
                const double top_a = static_cast<double>(station_heights[i]) + outer_h;
                const double top_b = static_cast<double>(station_heights[i + 1]) + outer_h;

                const float u_a = static_cast<float>((top_a - under[i]) / deck_u);
                const float u_b = static_cast<float>((top_b - under[i + 1]) / deck_u);

                sink.quad({ to_world(at(i, k), under[i]), to_world(at(i, k), top_a),
                            to_world(at(i + 1, k), top_b), to_world(at(i + 1, k), under[i + 1]) },
                          { glm::vec2(0.0f, v0), glm::vec2(u_a, v0),
                            glm::vec2(u_b, v1), glm::vec2(0.0f, v1) },
                          dir_to_world(is_left ? band_normal : -band_normal));
            }
        }

        // End caps. The top edge follows the profile's stepped cross-section so
        // the cap meets the corridor's outermost strip edges exactly; the bottom
        // edge is the underside grid. A zero-width curb face makes one of these
        // quads collapse to a triangle, which Sink::quad handles.
        for (const bool at_start : { true, false }) {
            const size_t i = at_start ? 0u : station_count - 1u;
            const Station& s = stations[i];
            const double surface = static_cast<double>(station_heights[i]);
            const glm::dvec3 outward = dir_to_world(at_start ? -s.tangent : s.tangent);

            for (size_t k = 0; k + 1 < lat_count; ++k) {
                const float u_a = static_cast<float>((left_edge - section[k].x) / deck_u);
                const float u_b = static_cast<float>((left_edge - section[k + 1].x) / deck_u);
                sink.quad({ to_world(at(i, k), surface + section[k].y),
                            to_world(at(i, k + 1), surface + section[k + 1].y),
                            to_world(at(i, k + 1), under[i]),
                            to_world(at(i, k), under[i]) },
                          { glm::vec2(u_a, static_cast<float>((surface + section[k].y - under[i]) / deck_v)),
                            glm::vec2(u_b, static_cast<float>((surface + section[k + 1].y - under[i]) / deck_v)),
                            glm::vec2(u_b, 0.0f), glm::vec2(u_a, 0.0f) },
                          outward);
            }
        }
    }

    // ------------------------------------------------------------------------
    // 2. Parapets.
    //
    // One open-bottomed box per side, standing INBOARD of the deck edge so it
    // never overhangs the slab it stands on. No bottom face: it would be
    // coincident with the corridor's own surface and z-fight with it, and the
    // wall is closed against that surface anyway.
    // ------------------------------------------------------------------------
    const double parapet_height = std::max(static_cast<double>(cfg.parapet_height), 0.0);
    double parapet_width = std::max(static_cast<double>(cfg.parapet_width), 0.0);
    if (parapet_width > total_width * kMaxParapetWidthFraction) {
        spdlog::debug("build_bridge: way {} clamping parapet width {:.3f} to {:.3f} on a {:.3f} m "
                      "deck", edge.source_way, parapet_width,
                      total_width * kMaxParapetWidthFraction, total_width);
        parapet_width = total_width * kMaxParapetWidthFraction;
    }

    if (cfg.emit_parapets && parapet_height > kZeroExtent && parapet_width > kZeroExtent) {
        sink.set_material(MaterialId::Parapet);

        const UVTiling wall_uv = uv_tiling(MaterialId::Parapet);
        const double wall_u = (wall_uv.u_metres > 0.0f) ? static_cast<double>(wall_uv.u_metres) : 1.0;
        const double wall_v = (wall_uv.v_metres > 0.0f) ? static_cast<double>(wall_uv.v_metres) : 1.0;

        for (const bool is_left : { true, false }) {
            const double lat_outer = is_left ? left_edge : right_edge;
            const double lat_inner = is_left ? (left_edge - parapet_width)
                                             : (right_edge + parapet_width);
            const double outer_h = is_left ? outer_h_left : outer_h_right;

            // Seat the wall on the lowest thing under it and cap it at
            // parapet_height above the profile's OUTER edge, per BridgeConfig.
            const double base_h = std::min(section_min_height(section, lat_outer, lat_inner),
                                           outer_h);
            const double top_h = outer_h + parapet_height;
            if (top_h - base_h <= kZeroExtent) {
                continue;
            }

            for (size_t i = 0; i + 1 < station_count; ++i) {
                const Station& sa = stations[i];
                const Station& sb = stations[i + 1];
                const double ha = static_cast<double>(station_heights[i]);
                const double hb = static_cast<double>(station_heights[i + 1]);

                const glm::dvec2 oa = offset_point(sa, lat_outer);
                const glm::dvec2 ob = offset_point(sb, lat_outer);
                const glm::dvec2 ia = offset_point(sa, lat_inner);
                const glm::dvec2 ib = offset_point(sb, lat_inner);

                const float v0 = static_cast<float>(sa.arclength / wall_v);
                const float v1 = static_cast<float>(sb.arclength / wall_v);
                const float u_top = static_cast<float>((top_h - base_h) / wall_u);
                const float u_side = static_cast<float>(parapet_width / wall_u);

                const glm::dvec2 band_normal = mean_normal(sa.normal, sb.normal);
                const glm::dvec3 out_dir = dir_to_world(is_left ? band_normal : -band_normal);

                // Outer face: continues the fascia upward.
                sink.quad({ to_world(oa, ha + base_h), to_world(oa, ha + top_h),
                            to_world(ob, hb + top_h), to_world(ob, hb + base_h) },
                          { glm::vec2(0.0f, v0), glm::vec2(u_top, v0),
                            glm::vec2(u_top, v1), glm::vec2(0.0f, v1) },
                          out_dir);

                // Inner face: what a driver on the deck sees.
                sink.quad({ to_world(ia, ha + base_h), to_world(ia, ha + top_h),
                            to_world(ib, hb + top_h), to_world(ib, hb + base_h) },
                          { glm::vec2(0.0f, v0), glm::vec2(u_top, v0),
                            glm::vec2(u_top, v1), glm::vec2(0.0f, v1) },
                          -out_dir);

                // Coping.
                sink.quad({ to_world(oa, ha + top_h), to_world(ia, ha + top_h),
                            to_world(ib, hb + top_h), to_world(ob, hb + top_h) },
                          { glm::vec2(0.0f, v0), glm::vec2(u_side, v0),
                            glm::vec2(u_side, v1), glm::vec2(0.0f, v1) },
                          glm::dvec3(0.0, 1.0, 0.0));
            }

            // Cap both ends, where the parapet runs into the abutment.
            for (const bool at_start : { true, false }) {
                const size_t i = at_start ? 0u : station_count - 1u;
                const Station& s = stations[i];
                const double h = static_cast<double>(station_heights[i]);
                const glm::dvec2 o = offset_point(s, lat_outer);
                const glm::dvec2 in = offset_point(s, lat_inner);
                const float u_side = static_cast<float>(parapet_width / wall_u);
                const float v_top = static_cast<float>((top_h - base_h) / wall_v);

                sink.quad({ to_world(o, h + base_h), to_world(in, h + base_h),
                            to_world(in, h + top_h), to_world(o, h + top_h) },
                          { glm::vec2(0.0f, 0.0f), glm::vec2(u_side, 0.0f),
                            glm::vec2(u_side, v_top), glm::vec2(0.0f, v_top) },
                          dir_to_world(at_start ? -s.tangent : s.tangent));
            }
        }
    }

    // ------------------------------------------------------------------------
    // 3. Piers.
    //
    // Laid out by dividing the span into equal parts, so the pattern is symmetric
    // and no pier ever lands on an abutment. The number of parts is
    // ceil(span / pier_spacing), which keeps the achieved spacing at or below the
    // configured one and gives a span no longer than one spacing zero interior
    // piers -- a short bridge is a clear span and needs no columns.
    // ------------------------------------------------------------------------
    const double pier_spacing = std::max(static_cast<double>(cfg.pier_spacing), 0.0);
    const double pier_width = std::max(static_cast<double>(cfg.pier_width), 0.0);

    if (cfg.emit_piers && has_slab && has_terrain &&
        pier_spacing > kZeroExtent && pier_width > kZeroExtent && span > kZeroExtent) {
        sink.set_material(MaterialId::Concrete);

        const UVTiling pier_uv = uv_tiling(MaterialId::Concrete);
        const double p_u = (pier_uv.u_metres > 0.0f) ? static_cast<double>(pier_uv.u_metres) : 1.0;
        const double p_v = (pier_uv.v_metres > 0.0f) ? static_cast<double>(pier_uv.v_metres) : 1.0;

        const double parts_real = std::ceil(span / pier_spacing);
        long parts = std::max(1L, static_cast<long>(std::min(parts_real,
                                                             static_cast<double>(kMaxPierBays))));
        if (parts_real > static_cast<double>(kMaxPierBays)) {
            spdlog::warn("build_bridge: way {} would take {:.0f} pier bays at {:.2f} m spacing over "
                         "{:.1f} m; capping at {}", edge.source_way, parts_real, pier_spacing,
                         span, kMaxPierBays);
            parts = kMaxPierBays;
        }
        const long pier_count = parts - 1L;

        const double half = pier_width * 0.5;

        for (long k = 1; k <= pier_count; ++k) {
            const double s_centre = span_start + span * (static_cast<double>(k) /
                                                         static_cast<double>(parts));
            const SpanSample mid = sample_span(cl, station_heights, s_centre);

            // The underside is ruled along the grade, so each corner takes the
            // underside height at its OWN arclength. A flat top on a 6% grade
            // pokes through the slab by most of a decimetre.
            const double s_back = std::max(span_start, s_centre - half);
            const double s_fore = std::min(span_end, s_centre + half);
            const double under_back =
                sample_span(cl, station_heights, s_back).surface - deck_thickness;
            const double under_fore =
                sample_span(cl, station_heights, s_fore).surface - deck_thickness;
            const double under_mid = mid.surface - deck_thickness;

            // Plan corners, counter-clockwise in the 2D plane. Every face is
            // oriented by Sink::quad, so this order carries no facing meaning.
            const glm::dvec2 t = mid.tangent * half;
            const glm::dvec2 nrm = mid.normal * half;
            const std::array<glm::dvec2, 4> corner{
                mid.position - t - nrm,     // back right
                mid.position + t - nrm,     // fore right
                mid.position + t + nrm,     // fore left
                mid.position - t + nrm      // back left
            };
            const std::array<double, 4> top{
                under_back + kPierEmbed, under_fore + kPierEmbed,
                under_fore + kPierEmbed, under_back + kPierEmbed
            };

            // Ground under the whole footprint, not just under the centre: a pier
            // on a slope would otherwise stand on one corner.
            double ground = static_cast<double>(terrain_at(mid.position.x, mid.position.y));
            if (!std::isfinite(ground)) {
                continue;
            }
            for (const glm::dvec2& c : corner) {
                const double g = static_cast<double>(terrain_at(c.x, c.y));
                if (std::isfinite(g)) {
                    ground = std::min(ground, g);
                }
            }

            const double drop = under_mid - ground;
            if (drop < static_cast<double>(cfg.min_pier_height) || drop <= kZeroExtent) {
                continue;   // the deck is at grade here; a stub would be buried
            }

            // Down to the foundation, not to the sampled surface. terrain_at is
            // the NATURAL ground; the carve of the road this bridge spans lowers
            // the finished ground under mid-span, and only the bridge's own carve
            // ribbon is suppressed. See BridgeConfig::pier_foundation_depth.
            const double foundation = std::isfinite(cfg.pier_foundation_depth)
                                          ? std::max(0.0,
                                                     static_cast<double>(cfg.pier_foundation_depth))
                                          : 0.0;
            const double base = ground - std::max(kPierEmbed, foundation);

            // Four sides.
            for (size_t f = 0; f < 4; ++f) {
                const size_t g = (f + 1u) % 4u;
                const glm::dvec2 edge_dir = corner[g] - corner[f];
                const double edge_len = std::sqrt(glm::dot(edge_dir, edge_dir));
                if (edge_len <= kZeroExtent) {
                    continue;
                }
                // Outward is the face's own outward normal in 2D: the edge turned
                // a quarter turn CLOCKWISE, because the ring above runs
                // counter-clockwise in the 2D plane. Sink::quad still measures the
                // result, so this is a hint and not the last word.
                const glm::dvec2 outward2 = glm::dvec2(edge_dir.y, -edge_dir.x) / edge_len;

                const float u_far = static_cast<float>(edge_len / p_u);
                const float v_f = static_cast<float>((top[f] - base) / p_v);
                const float v_g = static_cast<float>((top[g] - base) / p_v);

                sink.quad({ to_world(corner[f], base), to_world(corner[g], base),
                            to_world(corner[g], top[g]), to_world(corner[f], top[f]) },
                          { glm::vec2(0.0f, 0.0f), glm::vec2(u_far, 0.0f),
                            glm::vec2(u_far, v_g), glm::vec2(0.0f, v_f) },
                          dir_to_world(outward2));
            }

            const float u_side = static_cast<float>(pier_width / p_u);
            const float v_side = static_cast<float>(pier_width / p_v);
            const std::array<glm::vec2, 4> box_uv{
                glm::vec2(0.0f, 0.0f), glm::vec2(u_side, 0.0f),
                glm::vec2(u_side, v_side), glm::vec2(0.0f, v_side)
            };

            // Top, buried kPierEmbed inside the slab, and bottom, buried in the
            // ground. Both are emitted so the pier is a closed solid for export;
            // neither is visible from anywhere a camera can reach.
            sink.quad({ to_world(corner[0], top[0]), to_world(corner[1], top[1]),
                        to_world(corner[2], top[2]), to_world(corner[3], top[3]) },
                      box_uv, glm::dvec3(0.0, 1.0, 0.0));
            sink.quad({ to_world(corner[0], base), to_world(corner[1], base),
                        to_world(corner[2], base), to_world(corner[3], base) },
                      box_uv, glm::dvec3(0.0, -1.0, 0.0));
        }
    }

    if (sink.empty()) {
        return Mesh{};
    }
    return sink.take();
}

} // namespace stratum::osm::road
