/**
 * @file markings.cpp
 * @brief Implementation of the painted lane-marking emitter
 *
 * Markings are derived, never re-derived. The P2 cross-section already fixed
 * where every lane boundary is, so this file NEVER computes a lane position from
 * width arithmetic: it walks RoadProfile::strips from
 * RoadProfile::left_edge_offset(), decreasing the lateral coordinate by each
 * strip's width exactly as corridor.cpp does, records the two lateral
 * coordinates of every Lane strip, and paints on those. A road whose profile is
 * asymmetric -- a sidewalk on one side only, a parking bay, a cycle lane --
 * therefore gets its paint in the right place for free, because
 * left_edge_offset() already put the carriageway off-centre inside the profile.
 *
 * ### Three conventions, all borrowed rather than invented
 *
 * 1. **Lateral.** Positive is LEFT of travel, so walking the profile left to
 *    right walks the lateral coordinate DOWNWARDS. Same as corridor.cpp.
 * 2. **Offsetting.** Every lateral goes through offset_point(), which applies
 *    Station::miter_scale and the fold clamp. Nothing here multiplies by a normal
 *    itself, so no marking can drift off its lane at a bend and none can be
 *    pushed through a fold.
 * 3. **Winding.** With L the travel-frame LEFT column, R the travel-frame RIGHT
 *    column, N the upstream (near) end and F the downstream (far) end, the quad
 *    is (L_N, R_N, R_F) and (L_N, R_F, L_F). Worked through the
 *    (x, y_2d) -> (x, height, -y_2d) mapping this yields +Y, which is the only
 *    direction paint is ever viewed from. The pattern is NOT mirrored for a
 *    reversed approach: swapping which lateral is "left of travel" and which end
 *    is "near" flips the winding twice, and two flips is none.
 *
 * ### Placement along the road
 *
 * A marking occupies an arclength span, and the geometry for that span comes
 * from slice(). That is deliberate: slice() already synthesises endpoint
 * stations whose offset column lands exactly on the untrimmed ribbon's edge, so
 * a dash that begins between two stations begins on the lane rather than near
 * it, and a bevelled joint inside a run is carried through as its own zero-length
 * band. Re-deriving those endpoints here would be a second, worse copy of that
 * code.
 *
 * ### Why there is one quad per repeat
 *
 * An atlas sub-rect cannot wrap, so a dash run is not one long quad with a
 * scaled V. It is one quad per dash, each mapping the whole sprite rect, on a
 * phase measured from the edge's own arclength origin so the pattern does not
 * restart at every station or shift when the junction solver re-trims the arm.
 *
 * Everything here lives in stratum_core: no SDL, no ImGui, no rendering API.
 */

#include "osm/road/markings.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace stratum::osm::road {

namespace {

// ============================================================================
// Tolerances and caps
// ============================================================================

/// Arclength differences at or below this count as zero, metres
constexpr double kArcEpsilon = 1e-9;

/// Widths at or below this count as zero, metres
constexpr double kZeroWidth = 1e-6;

/// Two strip boundaries closer than this are the same boundary, metres
constexpr double kBoundaryEpsilon = 1e-4;

/// Squared length of the raw face cross product below which a triangle is dropped
constexpr double kDegenerateCrossSq = 1e-16;

/// The single-line width the frozen sprite table is authored at, metres
constexpr double kNominalLineWidth = 0.15;

/**
 * @brief Hard cap on repeats emitted for one run
 *
 * A dash cycle or a symbol interval configured at or near zero would otherwise
 * spin on a long edge. The cap turns a bad config into missing paint at the far
 * end of one road rather than into an unbounded allocation.
 */
constexpr int kMaxRepeats = 20000;

/// Fraction of a lane's width a pictogram or arrow may occupy before it is shrunk
constexpr double kLaneFillFraction = 0.9;

// ============================================================================
// World mapping
// ============================================================================

/**
 * @brief The codebase-wide 2D-to-3D mapping, Y up
 *
 * (x, y_2d) -> (x, height, -y_2d). Identical to corridor.cpp; markings that used
 * a different mapping would sit beside the road instead of on it.
 */
[[nodiscard]] inline glm::vec3 to_world(const glm::dvec2& p, double height) {
    return glm::vec3(static_cast<float>(p.x),
                     static_cast<float>(height),
                     static_cast<float>(-p.y));
}

// ============================================================================
// Mesh accumulation
// ============================================================================

/**
 * @brief Collects marking quads and resolves them into the output contract
 *
 * Every quad owns its own four vertices. Markings are discrete sprites that must
 * not share a vertex with each other any more than with the carriageway: two
 * dashes that shared a corner would share a UV, and the atlas rect of one would
 * bleed into the other.
 */
class MarkingMesh {
public:
    /**
     * @brief Append one quad
     *
     * @param p  Corners in order: left-near, right-near, right-far, left-far,
     *           where near is UPSTREAM and far is DOWNSTREAM of the direction of
     *           travel, and left/right are relative to that same direction.
     * @param uv Atlas coordinates for the four corners, in the same order.
     */
    void quad(const glm::vec3 (&p)[4], const glm::vec2 (&uv)[4]) {
        const uint32_t base = static_cast<uint32_t>(m_mesh.vertices.size());
        for (int i = 0; i < 4; ++i) {
            Vertex v{};
            v.position = p[i];
            v.normal = glm::vec3(0.0f, 1.0f, 0.0f);
            v.uv = uv[i];
            v.color = glm::vec4(1.0f);
            m_mesh.vertices.push_back(v);
            m_accum.emplace_back(0.0);
        }
        triangle(base + 0u, base + 1u, base + 2u);
        triangle(base + 0u, base + 2u, base + 3u);
    }

    /**
     * @brief Resolve normals and produce the finished mesh
     *
     * Returns a completely empty Mesh when every triangle was degenerate, so a
     * caller can test Mesh::is_valid() rather than inspecting index counts.
     */
    [[nodiscard]] Mesh finish() {
        if (m_mesh.indices.empty()) {
            return Mesh{};
        }

        for (size_t i = 0; i < m_mesh.vertices.size(); ++i) {
            const glm::dvec3& n = m_accum[i];
            const double len_sq = glm::dot(n, n);
            if (len_sq > 0.0) {
                const glm::dvec3 unit = n / std::sqrt(len_sq);
                m_mesh.vertices[i].normal = glm::vec3(static_cast<float>(unit.x),
                                                     static_cast<float>(unit.y),
                                                     static_cast<float>(unit.z));
            }
            // else: no surviving triangle references this vertex, so its initial
            // +Y normal is never sampled.
        }

        m_mesh.submeshes.clear();
        m_mesh.submeshes.push_back(SubMesh{ 0u,
                                            static_cast<uint32_t>(m_mesh.indices.size()),
                                            MaterialId::Markings });
        m_mesh.sort_submeshes_by_material();
        m_mesh.compute_bounds();
        m_mesh.compute_tangents();
        return std::move(m_mesh);
    }

private:
    void triangle(uint32_t i0, uint32_t i1, uint32_t i2) {
        const glm::dvec3 p0(m_mesh.vertices[i0].position);
        const glm::dvec3 p1(m_mesh.vertices[i1].position);
        const glm::dvec3 p2(m_mesh.vertices[i2].position);

        // Unnormalised, so the accumulation is area weighted.
        const glm::dvec3 face = glm::cross(p1 - p0, p2 - p0);
        if (glm::dot(face, face) <= kDegenerateCrossSq) {
            return;     // a zero-area sliver, e.g. across a bevel pair
        }

        m_mesh.indices.push_back(i0);
        m_mesh.indices.push_back(i1);
        m_mesh.indices.push_back(i2);

        m_accum[i0] += face;
        m_accum[i1] += face;
        m_accum[i2] += face;
    }

    Mesh m_mesh;
    std::vector<glm::dvec3> m_accum;
};

// ============================================================================
// Height sampling
// ============================================================================

/**
 * @brief World Y of the carriageway surface at an arbitrary arclength
 *
 * Two modes, because the two entry points need different ones. An edge follows
 * the solved per-station heights and must interpolate between them, since a dash
 * boundary rarely lands on a station. A junction approach is FLAT: the solver
 * flattened the arm mouth onto the junction plane, so the approach is emitted on
 * that one plane and paint that followed the stations would sink into the road.
 */
class SurfaceHeight {
public:
    /// Flat mode: every arclength returns @p constant
    explicit SurfaceHeight(double constant) : m_constant(constant) {}

    /**
     * @brief Interpolating mode
     *
     * A mis-sized @p heights vector degrades to flat world Y 0 rather than to
     * paint threaded through the terrain, matching CorridorConfig::station_heights.
     */
    SurfaceHeight(const Centerline& cl, const std::vector<float>& heights)
        : m_cl(&cl) {
        if (!heights.empty() && heights.size() == cl.stations.size()) {
            m_heights = &heights;
        }
    }

    [[nodiscard]] double at(double arclength) const {
        if (m_heights == nullptr) {
            return m_constant;
        }
        const std::vector<Station>& st = m_cl->stations;
        const std::vector<float>& h = *m_heights;

        if (arclength <= st.front().arclength) {
            return static_cast<double>(h.front());
        }
        if (arclength >= st.back().arclength) {
            return static_cast<double>(h.back());
        }

        const auto it = std::lower_bound(st.begin(), st.end(), arclength,
                                         [](const Station& s, double value) {
                                             return s.arclength < value;
                                         });
        size_t hi = static_cast<size_t>(it - st.begin());
        if (hi == 0) {
            hi = 1;
        }
        const size_t lo = hi - 1;

        const double a0 = st[lo].arclength;
        const double a1 = st[hi].arclength;
        // A bevel pair shares an arclength; either half gives the same height.
        const double t = (a1 - a0 > kArcEpsilon) ? (arclength - a0) / (a1 - a0) : 0.0;
        return static_cast<double>(h[lo]) + (static_cast<double>(h[hi]) - static_cast<double>(h[lo])) * t;
    }

private:
    const Centerline* m_cl = nullptr;
    const std::vector<float>* m_heights = nullptr;
    double m_constant = 0.0;
};

// ============================================================================
// Run emission
// ============================================================================

/**
 * @brief Emit one continuous painted band over an arclength span
 *
 * The band is cut out of @p cl with slice(), so it picks up every station inside
 * the span plus synthesised ends, and a CONTINUOUS line therefore follows the
 * miter exactly as the corridor's own edge does. A DISCRETE sprite instead spans
 * the slice end to end as one quad; see @p stretch_per_band.
 *
 * @param out              Accumulator to append into
 * @param cl               UNTRIMMED centerline the span is measured against
 * @param hs               Surface height sampler
 * @param lat_center       Lateral coordinate of the band's centre, positive LEFT
 * @param half_width       Half the band's painted width, metres
 * @param s_lo             Lower arclength bound of the span
 * @param s_hi             Upper arclength bound of the span
 * @param rect             Atlas rect to map across the band
 * @param stretch_per_band True to map the WHOLE rect onto every station band and
 *                         emit ONE QUAD PER BAND, which is what a uniform
 *                         continuous line wants; false to map the rect once
 *                         across the whole span as ONE QUAD, which is what a
 *                         single discrete sprite -- a dash, an arrow, a stop
 *                         line, a pictogram -- requires, since an atlas rect
 *                         cannot be split between two quads
 * @param reversed         True when the direction of travel is DECREASING
 *                         arclength, which is the frame a `from`-end junction
 *                         approach is emitted in
 * @param lift             Metres added to every vertex above the surface
 */
void emit_run(MarkingMesh& out,
              const Centerline& cl,
              const SurfaceHeight& hs,
              double lat_center,
              double half_width,
              double s_lo,
              double s_hi,
              const SpriteRect& rect,
              bool stretch_per_band,
              bool reversed,
              double lift) {
    if (!(half_width > kZeroWidth) || !std::isfinite(lat_center)) {
        return;
    }
    if (!std::isfinite(s_lo) || !std::isfinite(s_hi) || !(s_hi - s_lo > kArcEpsilon)) {
        return;
    }

    const Centerline sub = slice(cl, s_lo, s_hi);
    if (!sub.is_valid()) {
        return;
    }

    // Profile frame: a LARGER lateral is further left. Travel frame: which of the
    // two is "left" depends on which way traffic is going along the edge.
    const double lat_more_left = lat_center + half_width;
    const double lat_more_right = lat_center - half_width;
    const double lat_travel_left = reversed ? lat_more_right : lat_more_left;
    const double lat_travel_right = reversed ? lat_more_left : lat_more_right;

    const double a0 = sub.stations.front().arclength;
    const double a1 = sub.stations.back().arclength;
    const double span = a1 - a0;

    // v0 is the DOWNSTREAM edge of the sprite and v1 the UPSTREAM one, so v runs
    // against travel. Downstream is the larger arclength going forward and the
    // smaller one going backward.
    const auto v_at = [&](double s) -> float {
        if (!(span > kArcEpsilon)) {
            return rect.v0;
        }
        const double raw = reversed ? (s - a0) / span : (a1 - s) / span;
        const double t = std::min(std::max(raw, 0.0), 1.0);
        return static_cast<float>(static_cast<double>(rect.v0)
                                  + (static_cast<double>(rect.v1) - static_cast<double>(rect.v0)) * t);
    };

    // One quad between two stations, wound from the near end (the downstream one
    // in the direction of travel) to the far end.
    const auto emit_band = [&](const Station& lower, const Station& upper) {
        const Station& near_st = reversed ? upper : lower;
        const Station& far_st = reversed ? lower : upper;

        const double h_near = hs.at(near_st.arclength) + lift;
        const double h_far = hs.at(far_st.arclength) + lift;

        const glm::vec3 corners[4] = {
            to_world(offset_point(near_st, lat_travel_left), h_near),
            to_world(offset_point(near_st, lat_travel_right), h_near),
            to_world(offset_point(far_st, lat_travel_right), h_far),
            to_world(offset_point(far_st, lat_travel_left), h_far),
        };

        float v_near = rect.v1;
        float v_far = rect.v0;
        if (!stretch_per_band) {
            v_near = v_at(near_st.arclength);
            v_far = v_at(far_st.arclength);
        }

        const glm::vec2 uvs[4] = {
            glm::vec2(rect.u0, v_near),
            glm::vec2(rect.u1, v_near),
            glm::vec2(rect.u1, v_far),
            glm::vec2(rect.u0, v_far),
        };

        out.quad(corners, uvs);
    };

    // A DISCRETE SPRITE IS ONE QUAD. An atlas rect cannot wrap, and a sprite cut
    // in two at a station boundary is two quads each carrying a partial rect: the
    // pair renders correctly only by accident, no consumer can recognise either
    // half as the sprite it came from, and a dash so cut measures dash_length
    // only when its two pieces are added back together. So a run mapping one rect
    // across its whole span -- a dash, an arrow, a stop line, a pictogram -- is
    // emitted as a single quad spanning the sliced centerline end to end, exactly
    // as markings.hpp and marking_atlas.hpp specify. The chord error over one
    // sprite is at most a few centimetres, because a sprite is metres long while
    // a station band is tens of metres.
    //
    // A CONTINUOUS LINE still goes one quad per station band: it has no rect to
    // preserve -- the same rect is stretched onto every band -- and it must
    // follow the centerline's curvature and its miter rather than cut the corner.
    if (!stretch_per_band) {
        emit_band(sub.stations.front(), sub.stations.back());
        return;
    }

    for (size_t i = 0; i + 1 < sub.stations.size(); ++i) {
        emit_band(sub.stations[i], sub.stations[i + 1]);
    }
}

/**
 * @brief Emit a continuous line over the whole span, one quad per station band
 */
void emit_solid_line(MarkingMesh& out,
                     const Centerline& cl,
                     const SurfaceHeight& hs,
                     double lat_center,
                     double width,
                     double s_begin,
                     double s_end,
                     MarkingSprite sprite,
                     double lift) {
    emit_run(out, cl, hs, lat_center, 0.5 * width, s_begin, s_end,
             sprite_rect(sprite), /*stretch_per_band=*/true, /*reversed=*/false, lift);
}

/**
 * @brief Emit a broken line over the span, one quad per dash
 *
 * The phase is measured from arclength 0 of @p cl -- the edge's `from` node --
 * plus @p phase, not from @p s_begin. Two consequences, both wanted: the pattern
 * does not restart when a run is interrupted, and re-trimming the arm at a
 * junction slides the paint's start without sliding the dashes themselves.
 *
 * @p phase is what carries the pattern across an edge boundary that is not a
 * junction. A street is split into a new GraphEdge wherever a way ends, which
 * happens at every `name`, `ref` or `maxspeed` change, and those splits get no
 * trim and no junction: the two ribbons meet flush. Without a shared datum the
 * second edge restarts at its own zero and the joint reads as a double-length
 * dash or a short gap, depending on where the first edge's length fell in the
 * cycle. See dash_phases() in road_network_builder.cpp.
 *
 * A dash that would be clipped by either end of the span is dropped whole rather
 * than shortened, because a shortened dash still maps the full sprite rect and
 * would render as a compressed dash rather than a partial one.
 */
void emit_dashed_line(MarkingMesh& out,
                      const Centerline& cl,
                      const SurfaceHeight& hs,
                      double lat_center,
                      double width,
                      double s_begin,
                      double s_end,
                      MarkingSprite sprite,
                      double dash_length,
                      double dash_gap,
                      double lift,
                      double phase) {
    const double period = dash_length + dash_gap;
    if (!(dash_length > kZeroWidth) || !(period > kZeroWidth)) {
        // A degenerate cycle is a config error, not a road with no centre line.
        emit_solid_line(out, cl, hs, lat_center, width, s_begin, s_end, sprite, lift);
        return;
    }

    const double datum = std::isfinite(phase) ? phase : 0.0;

    const SpriteRect rect = sprite_rect(sprite);
    const double half = 0.5 * width;

    // Indexed in the CHAIN's frame and mapped back into this edge's arclength, so
    // two edges of one street agree on where every dash starts.
    double index = std::floor((s_begin + datum) / period);
    for (int repeat = 0; repeat < kMaxRepeats; ++repeat, index += 1.0) {
        const double d0 = index * period - datum;
        if (d0 >= s_end) {
            break;
        }
        const double d1 = d0 + dash_length;
        if (d0 >= s_begin - kArcEpsilon && d1 <= s_end + kArcEpsilon) {
            emit_run(out, cl, hs, lat_center, half, d0, d1, rect,
                     /*stretch_per_band=*/false, /*reversed=*/false, lift);
        }
    }
}

/**
 * @brief Emit a pictogram repeated along a strip at a fixed interval
 *
 * The sprite keeps its aspect ratio and is shrunk, never squashed, when the strip
 * is narrower than the sprite's nominal width.
 */
void emit_symbol_run(MarkingMesh& out,
                     const Centerline& cl,
                     const SurfaceHeight& hs,
                     double lat_center,
                     double available_width,
                     double s_begin,
                     double s_end,
                     MarkingSprite sprite,
                     double spacing,
                     bool reversed,
                     double lift) {
    const SpriteSize size = sprite_size(sprite);
    if (!(size.width_m > 0.0f) || !(size.length_m > 0.0f) || !(spacing > kZeroWidth)) {
        return;
    }

    double width = static_cast<double>(size.width_m);
    double length = static_cast<double>(size.length_m);
    const double budget = available_width * kLaneFillFraction;
    if (budget > kZeroWidth && width > budget) {
        const double scale = budget / width;
        width *= scale;
        length *= scale;
    }

    const SpriteRect rect = sprite_rect(sprite);
    const double half_len = 0.5 * length;

    // The first symbol sits half an interval in, so a short run still gets one
    // and a long one is not front-loaded against the junction.
    for (int repeat = 0; repeat < kMaxRepeats; ++repeat) {
        const double centre = s_begin + 0.5 * spacing + static_cast<double>(repeat) * spacing;
        if (centre + half_len > s_end) {
            break;
        }
        emit_run(out, cl, hs, lat_center, 0.5 * width,
                 centre - half_len, centre + half_len, rect,
                 /*stretch_per_band=*/false, reversed, lift);
    }
}

// ============================================================================
// Profile walking
// ============================================================================

/// Lateral bounds of one strip, in the profile frame
struct StripLateral {
    double lat_left = 0.0;      ///< larger lateral coordinate
    double lat_right = 0.0;     ///< smaller lateral coordinate
};

/**
 * @brief Lateral bounds of every strip, walked exactly as corridor.cpp walks them
 *
 * This is the whole reason markings do not re-derive lane positions: the same
 * walk that placed the vertex columns places the paint.
 */
[[nodiscard]] std::vector<StripLateral> strip_laterals(const RoadProfile& profile) {
    std::vector<StripLateral> out;
    out.reserve(profile.strips.size());

    double lateral = static_cast<double>(profile.left_edge_offset());
    for (const Strip& s : profile.strips) {
        StripLateral bounds;
        bounds.lat_left = lateral;
        bounds.lat_right = lateral - static_cast<double>(s.width);
        lateral = bounds.lat_right;
        out.push_back(bounds);
    }
    return out;
}

/// One running lane of the carriageway, with its lateral bounds and strip index
struct LaneSpan {
    double lat_left = 0.0;
    double lat_right = 0.0;
    size_t strip_index = 0;

    [[nodiscard]] double width() const { return lat_left - lat_right; }
    [[nodiscard]] double centre() const { return 0.5 * (lat_left + lat_right); }
};

/// Every Lane strip of non-zero width, ordered left to right
[[nodiscard]] std::vector<LaneSpan> collect_lanes(const RoadProfile& profile,
                                                  const std::vector<StripLateral>& laterals) {
    std::vector<LaneSpan> lanes;
    for (size_t i = 0; i < profile.strips.size() && i < laterals.size(); ++i) {
        if (profile.strips[i].kind != StripKind::Lane) {
            continue;
        }
        if (!(profile.strips[i].width > static_cast<float>(kZeroWidth))) {
            continue;
        }
        LaneSpan lane;
        lane.lat_left = laterals[i].lat_left;
        lane.lat_right = laterals[i].lat_right;
        lane.strip_index = i;
        lanes.push_back(lane);
    }
    return lanes;
}

/**
 * @brief Whether two lanes share a boundary
 *
 * Compared on the boundary coordinate rather than on strip indices, so a
 * zero-width strip between two lanes -- a pure vertical riser, or a painted
 * median that resolved to nothing -- does not split a lane line in two, while a
 * real Median strip does.
 */
[[nodiscard]] bool lanes_adjacent(const LaneSpan& a, const LaneSpan& b) {
    return std::fabs(a.lat_right - b.lat_left) <= kBoundaryEpsilon;
}

// ============================================================================
// Lane groups
// ============================================================================

/**
 * @brief Which lanes travel which way, and where the centre line falls
 *
 * Index ranges are half-open over the lane list, which is ordered left to right
 * in the profile frame regardless of the traffic convention.
 */
struct LaneGroups {
    size_t forward_begin = 0;
    size_t forward_end = 0;
    size_t backward_begin = 0;
    size_t backward_end = 0;

    /// Lanes lying to the LEFT of the centre boundary; meaningless unless has_divide
    size_t divide = 0;
    bool has_divide = false;

    [[nodiscard]] size_t forward_count() const { return forward_end - forward_begin; }
    [[nodiscard]] size_t backward_count() const { return backward_end - backward_begin; }
};

/**
 * @brief Split the lane list into the two directions of travel
 *
 * Under left-hand traffic the FORWARD lanes -- those going from `from` towards
 * `to` -- occupy the LEFT of the profile; under right-hand traffic they occupy
 * the right. That single fact decides which boundary is the centre line and which
 * lanes a `turn:lanes` value applies to.
 *
 * The split is taken from lanes:forward and lanes:backward when either resolves,
 * and otherwise by halving the lane run. An odd lane count with no directional
 * tag is bad data with no right answer; the extra lane is given to the FORWARD
 * group, which at least makes the two directions of one way agree with each other
 * when both are imported.
 */
[[nodiscard]] LaneGroups resolve_groups(const GraphEdge& edge, size_t lane_count, bool lht) {
    LaneGroups g;
    if (lane_count == 0) {
        return g;
    }

    if (edge.is_oneway) {
        g.forward_end = lane_count;     // backward stays empty; no centre line
        return g;
    }

    size_t left_count = 0;
    const long long lf = static_cast<long long>(edge.lanes_forward);
    const long long lb = static_cast<long long>(edge.lanes_backward);
    const long long n = static_cast<long long>(lane_count);

    if (lf > 0 && lb > 0 && lf + lb == n) {
        left_count = static_cast<size_t>(lht ? lf : lb);
    } else if (lf > 0 && lf < n) {
        left_count = static_cast<size_t>(lht ? lf : n - lf);
    } else if (lb > 0 && lb < n) {
        left_count = static_cast<size_t>(lht ? n - lb : lb);
    } else if (lane_count >= 2) {
        left_count = (lane_count % 2 == 0)
                         ? lane_count / 2
                         : (lht ? (lane_count + 1) / 2 : lane_count / 2);
    }

    if (left_count >= 1 && left_count <= lane_count - 1) {
        g.has_divide = true;
        g.divide = left_count;
        if (lht) {
            g.forward_begin = 0;
            g.forward_end = left_count;
            g.backward_begin = left_count;
            g.backward_end = lane_count;
        } else {
            g.backward_begin = 0;
            g.backward_end = left_count;
            g.forward_begin = left_count;
            g.forward_end = lane_count;
        }
        return g;
    }

    // A two-way road with a single running lane: one shared surface serves both
    // directions, there is no boundary to paint, and either approach stops across
    // the same lane.
    g.forward_begin = 0;
    g.forward_end = lane_count;
    g.backward_begin = 0;
    g.backward_end = lane_count;
    return g;
}

// ============================================================================
// Road class rules
// ============================================================================

/// Whether an edge of this class carries painted carriageway markings at all
[[nodiscard]] bool class_is_painted(RoadType type) {
    switch (type) {
        case RoadType::Footway:
        case RoadType::Cycleway:
        case RoadType::Path:
            return false;
        default:
            return true;
    }
}

/**
 * @brief Whether a centre line divides opposing flows on this class
 *
 * Motorway and Trunk are excluded: their carriageways are separated by a Median
 * strip, so no Lane-to-Lane boundary divides opposing traffic in the first place,
 * and where such a road is mapped undivided the honest output is no line rather
 * than a line down the middle of an island.
 */
[[nodiscard]] bool class_has_centre_line(RoadType type, const MarkingConfig& cfg) {
    switch (type) {
        case RoadType::Motorway:
        case RoadType::Trunk:
            return false;
        case RoadType::Primary:
        case RoadType::Secondary:
        case RoadType::Tertiary:
            return true;
        case RoadType::Residential:
        case RoadType::Service:
        case RoadType::Unknown:
            return cfg.centre_line_on_minor_roads;
        default:
            return false;
    }
}

/// Whether the outer carriageway boundary is marked with an edge line on this class
[[nodiscard]] bool class_has_edge_lines(RoadType type, const MarkingConfig& cfg) {
    switch (type) {
        case RoadType::Motorway:
        case RoadType::Trunk:
        case RoadType::Primary:
        case RoadType::Secondary:
        case RoadType::Tertiary:
            return true;
        case RoadType::Residential:
        case RoadType::Service:
        case RoadType::Unknown:
            return cfg.edge_lines_on_minor_roads;
        default:
            return false;
    }
}

// ============================================================================
// Tag helpers
// ============================================================================

[[nodiscard]] const std::string* find_tag(const TagMap* tags, const char* key) {
    if (tags == nullptr) {
        return nullptr;
    }
    const auto it = tags->find(key);
    return (it == tags->end()) ? nullptr : &it->second;
}

/// Lowercase, with leading and trailing whitespace removed
[[nodiscard]] std::string lower_trim(std::string_view value) {
    size_t begin = 0;
    size_t end = value.size();
    const auto is_space = [](char c) {
        return std::isspace(static_cast<unsigned char>(c)) != 0;
    };
    while (begin < end && is_space(value[begin])) {
        ++begin;
    }
    while (end > begin && is_space(value[end - 1])) {
        --end;
    }

    std::string out;
    out.reserve(end - begin);
    for (size_t i = begin; i < end; ++i) {
        out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(value[i]))));
    }
    return out;
}

/**
 * @brief Split an OSM `*:lanes` value on its pipes
 *
 * One entry per lane, ordered left to right in the direction of travel. A
 * trailing pipe yields a trailing empty entry, which is a real lane with no
 * value rather than a parse error, so the count still matches the road.
 */
[[nodiscard]] std::vector<std::string> split_lane_values(const std::string& value) {
    std::vector<std::string> out;
    size_t start = 0;
    while (true) {
        const size_t pipe = value.find('|', start);
        if (pipe == std::string::npos) {
            out.push_back(lower_trim(std::string_view(value).substr(start)));
            break;
        }
        out.push_back(lower_trim(std::string_view(value).substr(start, pipe - start)));
        start = pipe + 1;
    }
    return out;
}

/// Split one lane's value on its semicolons
[[nodiscard]] std::vector<std::string> split_tokens(const std::string& value) {
    std::vector<std::string> out;
    size_t start = 0;
    while (true) {
        const size_t sep = value.find(';', start);
        if (sep == std::string::npos) {
            out.push_back(lower_trim(std::string_view(value).substr(start)));
            break;
        }
        out.push_back(lower_trim(std::string_view(value).substr(start, sep - start)));
        start = sep + 1;
    }
    return out;
}

/// Whether an OSM access-style value is affirmative
[[nodiscard]] bool tag_is_yes(const std::string& lowered) {
    return lowered == "yes" || lowered == "designated" || lowered == "official"
        || lowered == "true" || lowered == "1";
}

/**
 * @brief Whether overtaking is forbidden anywhere on this edge
 *
 * `overtaking=forward` and `overtaking=backward` restrict one direction only.
 * A one-sided solid-and-broken pair is not modelled in this pass, so both are
 * treated as `no` and the whole centre line goes solid: over-restricting the
 * paint is a legible road, under-restricting it is a wrong instruction.
 */
[[nodiscard]] bool overtaking_forbidden(const TagMap* tags) {
    static constexpr const char* kKeys[] = { "overtaking", "overtaking:both" };
    for (const char* key : kKeys) {
        if (const std::string* raw = find_tag(tags, key)) {
            const std::string value = lower_trim(*raw);
            if (value == "no" || value == "none" || value == "forward" || value == "backward") {
                return true;
            }
        }
    }
    return false;
}

// ============================================================================
// turn:lanes
// ============================================================================

/**
 * @brief Map one `turn:lanes` entry to an arrow sprite
 *
 * The OSM value for a lane is a semicolon-separated set of permitted movements.
 * Token handling:
 *
 * - `through` is a through movement.
 * - `left`, `slight_left` and `sharp_left` are all a left movement; likewise for
 *   right. The atlas holds one arrow per direction, not one per severity.
 * - `merge_to_left` and `merge_to_right` are a through movement that also shifts
 *   across, so they read as through PLUS that direction and get the combined
 *   arrow.
 * - `reverse` alone is a U-turn. Combined with a turn it is dropped, because a
 *   combined U-turn arrow does not exist in the atlas and the turn is the
 *   stronger instruction.
 * - `none`, an empty entry, and any unrecognised token contribute nothing.
 *
 * @param entry Lowercased, trimmed value for one lane
 * @param out   Receives the sprite when the function returns true
 * @return False when the entry states a movement set no single sprite can
 *         express -- `left;right` with no through movement -- in which case that
 *         lane gets no arrow. Painting one would instruct a driver to make a turn
 *         the tag did not permit.
 */
[[nodiscard]] bool turn_entry_to_sprite(const std::string& entry, MarkingSprite& out) {
    bool has_through = false;
    bool has_left = false;
    bool has_right = false;
    bool has_reverse = false;
    bool any_known = false;

    for (const std::string& token : split_tokens(entry)) {
        if (token.empty() || token == "none") {
            continue;
        }
        if (token == "through") {
            has_through = true;
        } else if (token == "left" || token == "slight_left" || token == "sharp_left") {
            has_left = true;
        } else if (token == "right" || token == "slight_right" || token == "sharp_right") {
            has_right = true;
        } else if (token == "merge_to_left") {
            has_through = true;
            has_left = true;
        } else if (token == "merge_to_right") {
            has_through = true;
            has_right = true;
        } else if (token == "reverse") {
            has_reverse = true;
        } else {
            continue;   // unrecognised; contributes nothing
        }
        any_known = true;
    }

    if (!any_known) {
        out = MarkingSprite::ArrowStraight;
        return true;
    }
    if (has_reverse && !has_through && !has_left && !has_right) {
        out = MarkingSprite::ArrowUTurn;
        return true;
    }
    if (has_through && has_left && !has_right) {
        out = MarkingSprite::ArrowStraightLeft;
        return true;
    }
    if (has_through && has_right && !has_left) {
        out = MarkingSprite::ArrowStraightRight;
        return true;
    }
    if (has_through) {
        // Through plus both turns: through is true and is the movement the lane
        // is named for, so it is stated and the turns are left unstated.
        out = MarkingSprite::ArrowStraight;
        return true;
    }
    if (has_left && !has_right) {
        out = MarkingSprite::ArrowLeft;
        return true;
    }
    if (has_right && !has_left) {
        out = MarkingSprite::ArrowRight;
        return true;
    }
    return false;   // left and right, no through: unrepresentable
}

/**
 * @brief Read the `turn:lanes` family for one approach direction
 *
 * The directional key wins over the plain one. The plain key is read ONLY on a
 * one-way edge, where it unambiguously describes the single direction of travel.
 *
 * On a two-way edge a plain `turn:lanes` describes the way, not an approach, and
 * there is no way to tell which of the two it was meant for. The count check at
 * the call site catches the well-formed case -- a value per lane of the whole
 * way will not match one direction's group -- but not the two that matter here:
 * a value per lane of ONE carriageway on a way tagged `lanes=4`, which matches a
 * half group exactly and would paint the forward instructions onto the opposing
 * lanes at the far junction; and a way with no forward/backward divide at all,
 * where both groups span every lane and any full-length value matches by
 * construction, giving one lane two contradictory arrows. Both are false
 * instructions to a driver, which is the one failure this whole family of tags
 * must not produce.
 *
 * @return Empty when nothing usable is present
 */
[[nodiscard]] std::vector<std::string> read_turn_lanes(const TagMap* tags, bool at_start,
                                                       bool oneway) {
    const char* directional = at_start ? "turn:lanes:backward" : "turn:lanes:forward";
    if (const std::string* raw = find_tag(tags, directional)) {
        return split_lane_values(*raw);
    }
    if (!oneway) {
        return {};
    }
    if (const std::string* raw = find_tag(tags, "turn:lanes")) {
        return split_lane_values(*raw);
    }
    return {};
}

// ============================================================================
// Bus lanes
// ============================================================================

/**
 * @brief Which lanes carry a bus pictogram
 *
 * Lane-positioned tags are preferred: `bus:lanes` and `psv:lanes` name a value
 * per lane, so a matching entry count places the symbol exactly. Failing that,
 * a way-level `psv`, `bus`, or `lanes:psv` says a bus lane exists but not where,
 * and the only defensible guess is the NEARSIDE lane of the forward group --
 * leftmost under left-hand traffic, rightmost under right-hand traffic -- which
 * is where a bus lane is in practice.
 */
[[nodiscard]] std::vector<bool> bus_lane_flags(const TagMap* tags, size_t lane_count, bool lht) {
    std::vector<bool> flags(lane_count, false);
    if (tags == nullptr || lane_count == 0) {
        return flags;
    }

    static constexpr const char* kLaneKeys[] = { "bus:lanes", "psv:lanes" };
    for (const char* key : kLaneKeys) {
        const std::string* raw = find_tag(tags, key);
        if (raw == nullptr) {
            continue;
        }
        const std::vector<std::string> entries = split_lane_values(*raw);
        if (entries.size() != lane_count) {
            continue;   // count mismatch: no safe way to place the symbols
        }
        bool any = false;
        for (size_t i = 0; i < lane_count; ++i) {
            for (const std::string& token : split_tokens(entries[i])) {
                if (tag_is_yes(token)) {
                    flags[i] = true;
                    any = true;
                    break;
                }
            }
        }
        if (any) {
            return flags;
        }
    }

    bool way_level = false;
    static constexpr const char* kWayKeys[] = { "psv", "bus" };
    for (const char* key : kWayKeys) {
        if (const std::string* raw = find_tag(tags, key)) {
            if (tag_is_yes(lower_trim(*raw))) {
                way_level = true;
            }
        }
    }
    if (!way_level) {
        static constexpr const char* kCountKeys[] = { "lanes:psv", "lanes:bus" };
        for (const char* key : kCountKeys) {
            if (const std::string* raw = find_tag(tags, key)) {
                const std::string value = lower_trim(*raw);
                if (!value.empty() && value != "0") {
                    way_level = true;
                }
            }
        }
    }

    if (way_level) {
        flags[lht ? 0 : lane_count - 1] = true;
    }
    return flags;
}

// ============================================================================
// Shared geometry helpers
// ============================================================================

/// Painted width of a line sprite, honouring the config override
[[nodiscard]] double line_width_for(MarkingSprite sprite, const MarkingConfig& cfg) {
    const double configured = static_cast<double>(cfg.line_width);
    if (!(configured > kZeroWidth)) {
        return static_cast<double>(sprite_size(sprite).width_m);
    }
    if (sprite == MarkingSprite::DoubleSolidYellow) {
        // The sprite holds two lines and the gap between them, so the override
        // scales it rather than replacing it: forcing 0.35 m down to 0.15 m would
        // paint a double line thinner than a single one.
        const double nominal = static_cast<double>(sprite_size(sprite).width_m);
        return nominal * (configured / kNominalLineWidth);
    }
    return configured;
}

/**
 * @brief Whether a strip sits on the side of the road that travels backward
 *
 * Under left-hand traffic forward traffic occupies the left of the profile, so a
 * strip at a negative lateral is on the backward side. A one-way edge has no
 * backward side.
 */
[[nodiscard]] bool strip_travels_backward(double lat_centre, const GraphEdge& edge, bool lht) {
    if (edge.is_oneway) {
        return false;
    }
    return lht ? (lat_centre < 0.0) : (lat_centre > 0.0);
}

} // namespace

// ============================================================================
// Edge markings
// ============================================================================

/**
 * @brief All longitudinal marking geometry for one edge
 *
 * Invariant: every emitted vertex lies on the ribbon the corridor extruder built
 * for this same edge, because every lateral goes through offset_point() against
 * the same stations, and every arclength span is cut with slice(). The paint can
 * therefore never drift off the lane at a bend, and it never extends into the
 * junction polygon, because the span is bounded by the solved trims.
 */
Mesh build_edge_markings(const GraphEdge& edge,
                         const Centerline& cl,
                         const RoadProfile& profile,
                         const std::vector<float>& station_heights,
                         const MarkingConfig& cfg,
                         const TagMap* tags,
                         double dash_phase) {
    if (!cl.is_valid() || !profile.is_valid() || !class_is_painted(edge.type)) {
        return Mesh{};
    }

    const std::vector<StripLateral> laterals = strip_laterals(profile);
    const std::vector<LaneSpan> lanes = collect_lanes(profile, laterals);
    if (lanes.empty()) {
        return Mesh{};      // no carriageway to paint
    }

    // The paint runs the full ribbon and stops where the junction solver cut it.
    const double s_front = cl.stations.front().arclength;
    const double s_back = cl.stations.back().arclength;
    const double s_begin = s_front + std::max(0.0, edge.trim_from);
    const double s_end = s_back - std::max(0.0, edge.trim_to);
    if (!(s_end - s_begin > kArcEpsilon)) {
        return Mesh{};      // entirely consumed by its own junctions
    }

    const SurfaceHeight heights(cl, station_heights);
    const double lift = static_cast<double>(cfg.height_above_surface);
    const bool lht = cfg.left_hand_traffic;
    const LaneGroups groups = resolve_groups(edge, lanes.size(), lht);

    MarkingMesh out;

    // ------------------------------------------------------------------------
    // Lane lines and the centre line
    // ------------------------------------------------------------------------
    if (cfg.emit_lane_lines) {
        const bool centre_allowed = class_has_centre_line(edge.type, cfg);
        const bool no_overtaking = overtaking_forbidden(tags);
        const double dash_length = static_cast<double>(cfg.dash_length);
        const double dash_gap = static_cast<double>(cfg.dash_gap);

        for (size_t i = 0; i + 1 < lanes.size(); ++i) {
            if (!lanes_adjacent(lanes[i], lanes[i + 1])) {
                continue;   // a Median or another strip separates them; not a lane line
            }

            const double lat = 0.5 * (lanes[i].lat_right + lanes[i + 1].lat_left);
            const bool is_centre = groups.has_divide && (i + 1 == groups.divide);

            if (!is_centre) {
                // Two lanes going the same way: a broken white line under both
                // traffic conventions.
                const MarkingSprite sprite = MarkingSprite::DashWhite;
                emit_dashed_line(out, cl, heights, lat, line_width_for(sprite, cfg),
                                 s_begin, s_end, sprite, dash_length, dash_gap, lift, dash_phase);
                continue;
            }

            if (!centre_allowed) {
                continue;
            }

            if (!no_overtaking) {
                const MarkingSprite sprite = lht ? MarkingSprite::DashWhite
                                                 : MarkingSprite::DashedYellow;
                emit_dashed_line(out, cl, heights, lat, line_width_for(sprite, cfg),
                                 s_begin, s_end, sprite, dash_length, dash_gap, lift, dash_phase);
            } else if (lht) {
                // Two separate white lines: yellow means a kerbside restriction
                // here, not a lane division.
                const MarkingSprite sprite = MarkingSprite::SolidWhite;
                const double width = line_width_for(sprite, cfg);
                const double offset = 0.5 * (width + static_cast<double>(cfg.double_line_gap));
                emit_solid_line(out, cl, heights, lat + offset, width, s_begin, s_end, sprite, lift);
                emit_solid_line(out, cl, heights, lat - offset, width, s_begin, s_end, sprite, lift);
            } else {
                const MarkingSprite sprite = MarkingSprite::DoubleSolidYellow;
                emit_solid_line(out, cl, heights, lat, line_width_for(sprite, cfg),
                                s_begin, s_end, sprite, lift);
            }
        }
    }

    // ------------------------------------------------------------------------
    // Edge lines, just inside the outermost lanes
    // ------------------------------------------------------------------------
    if (cfg.emit_edge_lines && class_has_edge_lines(edge.type, cfg)) {
        const MarkingSprite sprite = MarkingSprite::SolidWhite;
        const double width = line_width_for(sprite, cfg);
        const double inset = static_cast<double>(cfg.edge_line_inset) + 0.5 * width;

        const double lat_left = lanes.front().lat_left - inset;
        const double lat_right = lanes.back().lat_right + inset;

        // On a carriageway narrower than twice the inset the two lines would
        // cross. Paint neither rather than a saltire.
        if (lat_left - lat_right > width) {
            emit_solid_line(out, cl, heights, lat_left, width, s_begin, s_end, sprite, lift);
            emit_solid_line(out, cl, heights, lat_right, width, s_begin, s_end, sprite, lift);
        }
    }

    // ------------------------------------------------------------------------
    // Lane pictograms: cycle lanes from the profile, bus lanes from the tags
    // ------------------------------------------------------------------------
    const double spacing = static_cast<double>(cfg.symbol_spacing);

    for (size_t i = 0; i < profile.strips.size() && i < laterals.size(); ++i) {
        if (profile.strips[i].kind != StripKind::CycleLane) {
            continue;
        }
        const double lane_width = laterals[i].lat_left - laterals[i].lat_right;
        if (!(lane_width > kZeroWidth)) {
            continue;
        }
        const double centre = 0.5 * (laterals[i].lat_left + laterals[i].lat_right);
        emit_symbol_run(out, cl, heights, centre, lane_width, s_begin, s_end,
                        MarkingSprite::BikeSymbol, spacing,
                        strip_travels_backward(centre, edge, lht), lift);
    }

    const std::vector<bool> bus_lanes = bus_lane_flags(tags, lanes.size(), lht);
    for (size_t i = 0; i < lanes.size(); ++i) {
        if (!bus_lanes[i]) {
            continue;
        }
        // With a resolved divide the two groups are disjoint, so membership of the
        // backward range settles the direction. Without one -- a single shared
        // running lane -- fall back to which side of the road the lane sits on.
        const bool backward = groups.has_divide
                                  ? (i >= groups.backward_begin && i < groups.backward_end)
                                  : strip_travels_backward(lanes[i].centre(), edge, lht);
        emit_symbol_run(out, cl, heights, lanes[i].centre(), lanes[i].width(),
                        s_begin, s_end, MarkingSprite::BusSymbol, spacing, backward, lift);
    }

    return out.finish();
}

// ============================================================================
// Approach markings
// ============================================================================

/**
 * @brief Stop line and turn arrows on one arm approaching a junction
 *
 * Invariant: nothing is emitted outside the arm's own untrimmed-but-trimmed span
 * [trim_from, length - trim_to]. Each element is placed whole or not at all, so a
 * short arm loses its arrows before it loses its stop line, and neither ever
 * reaches into the junction polygon.
 */
Mesh build_approach_markings(const GraphEdge& edge,
                             const Centerline& cl,
                             const RoadProfile& profile,
                             bool at_start,
                             bool has_signals,
                             float height,
                             const MarkingConfig& cfg,
                             const TagMap* tags,
                             const std::vector<float>* station_heights) {
    if (!cl.is_valid() || !profile.is_valid() || !class_is_painted(edge.type)) {
        return Mesh{};
    }
    if (!cfg.emit_stop_lines && !cfg.emit_arrows) {
        return Mesh{};
    }

    const std::vector<StripLateral> laterals = strip_laterals(profile);
    const std::vector<LaneSpan> lanes = collect_lanes(profile, laterals);
    if (lanes.empty()) {
        return Mesh{};
    }

    const bool lht = cfg.left_hand_traffic;
    const LaneGroups groups = resolve_groups(edge, lanes.size(), lht);

    const size_t group_begin = at_start ? groups.backward_begin : groups.forward_begin;
    const size_t group_end = at_start ? groups.backward_end : groups.forward_end;
    if (group_begin >= group_end) {
        return Mesh{};      // e.g. a one-way edge approached from its exit end
    }
    const size_t group_size = group_end - group_begin;

    const double s_front = cl.stations.front().arclength;
    const double s_back = cl.stations.back().arclength;
    const double s_begin = s_front + std::max(0.0, edge.trim_from);
    const double s_end = s_back - std::max(0.0, edge.trim_to);
    if (!(s_end - s_begin > kArcEpsilon)) {
        return Mesh{};
    }

    // Travel runs towards the cut. Going forward that is increasing arclength and
    // the cut is at the `to` end; going backward it is decreasing arclength and
    // the cut is at the `from` end.
    const double direction = at_start ? -1.0 : 1.0;
    const double s_cut = at_start ? s_begin : s_end;
    const bool reversed = at_start;

    // The approach follows the corridor, it is not laid on one plane. The
    // plateau over the trim already IS the junction plane, so the stop line and
    // anything else near the cut land on it; the arrows sit up to
    // arrow_spacing + the arrow's own length back from the cut, which is well
    // outside the plateau and back on the solved grade. Emitting those on the
    // junction plane buries them under a graded carriageway.
    const bool follow_stations = station_heights != nullptr &&
                                 station_heights->size() == cl.stations.size() &&
                                 !station_heights->empty();
    const SurfaceHeight heights = follow_stations
                                      ? SurfaceHeight(cl, *station_heights)
                                      : SurfaceHeight(static_cast<double>(height));
    const double lift = static_cast<double>(cfg.height_above_surface);

    /// True when an arclength span lies wholly inside the paintable stretch
    const auto span_fits = [&](double a, double b) {
        const double lo = std::min(a, b);
        const double hi = std::max(a, b);
        return lo >= s_begin - kArcEpsilon && hi <= s_end + kArcEpsilon;
    };

    MarkingMesh out;

    // ------------------------------------------------------------------------
    // Stop line, or the give-way triangles that stand in for it at a priority
    // junction. Its geometry is computed even when suppressed, because the turn
    // arrows are positioned relative to it.
    // ------------------------------------------------------------------------
    const MarkingSprite stop_sprite = has_signals ? MarkingSprite::StopLine
                                                  : MarkingSprite::GiveWayTriangles;
    const double stop_length = has_signals
                                   ? static_cast<double>(cfg.stop_line_width)
                                   : static_cast<double>(sprite_size(stop_sprite).length_m);

    const double stop_downstream = s_cut - direction * static_cast<double>(cfg.stop_line_setback);
    const double stop_upstream = stop_downstream - direction * stop_length;

    if (cfg.emit_stop_lines && stop_length > kZeroWidth && span_fits(stop_downstream, stop_upstream)) {
        const double lo = std::min(stop_downstream, stop_upstream);
        const double hi = std::max(stop_downstream, stop_upstream);
        const SpriteRect rect = sprite_rect(stop_sprite);

        for (size_t i = group_begin; i < group_end; ++i) {
            const double lane_width = lanes[i].width();
            if (!(lane_width > kZeroWidth)) {
                continue;
            }

            if (has_signals) {
                // One continuous bar spanning the lane; StopLine's width is a unit.
                emit_run(out, cl, heights, lanes[i].centre(), 0.5 * lane_width,
                         lo, hi, rect, /*stretch_per_band=*/false, reversed, lift);
                continue;
            }

            // Give-way triangles: one sprite per repeat, spread across the lane
            // with a gap of half a triangle between them, centred on the lane.
            const double triangle_width = static_cast<double>(sprite_size(stop_sprite).width_m);
            if (!(triangle_width > kZeroWidth)) {
                continue;
            }
            const double gap = 0.5 * triangle_width;
            const double pitch = triangle_width + gap;
            size_t count = static_cast<size_t>(std::floor((lane_width + gap) / pitch));
            count = std::max<size_t>(count, 1u);
            const double row_span = static_cast<double>(count) * triangle_width
                                  + static_cast<double>(count - 1) * gap;
            const double first = lanes[i].centre() + 0.5 * row_span - 0.5 * triangle_width;

            for (size_t k = 0; k < count; ++k) {
                const double lat = first - static_cast<double>(k) * pitch;
                emit_run(out, cl, heights, lat, 0.5 * triangle_width,
                         lo, hi, rect, /*stretch_per_band=*/false, reversed, lift);
            }
        }
    }

    // ------------------------------------------------------------------------
    // Turn arrows
    // ------------------------------------------------------------------------
    if (cfg.emit_arrows) {
        std::vector<std::string> entries = read_turn_lanes(tags, at_start, edge.is_oneway);

        if (!entries.empty() && entries.size() != group_size) {
            spdlog::debug("build_approach_markings: way {} has {} turn:lanes entries for {} "
                          "approaching lanes; no arrows emitted",
                          edge.source_way, entries.size(), group_size);
            entries.clear();
        }

        if (!entries.empty()) {
            const SpriteSize arrow_size = sprite_size(MarkingSprite::ArrowStraight);
            const double arrow_length = static_cast<double>(arrow_size.length_m);
            const double arrow_width = static_cast<double>(arrow_size.width_m);

            const double tip = stop_upstream - direction * static_cast<double>(cfg.arrow_spacing);
            const double tail = tip - direction * arrow_length;

            if (arrow_length > kZeroWidth && span_fits(tip, tail)) {
                const double lo = std::min(tip, tail);
                const double hi = std::max(tip, tail);

                for (size_t j = 0; j < entries.size(); ++j) {
                    // Entries are ordered left to right in the DIRECTION OF
                    // TRAVEL. Going forward that matches the profile's own left to
                    // right; going backward it is the profile reversed.
                    const size_t lane_index = at_start ? (group_end - 1u - j) : (group_begin + j);

                    MarkingSprite sprite = MarkingSprite::ArrowStraight;
                    if (!turn_entry_to_sprite(entries[j], sprite)) {
                        continue;   // no single sprite states this lane's movements
                    }

                    const double lane_width = lanes[lane_index].width();
                    if (!(lane_width > kZeroWidth)) {
                        continue;
                    }

                    // Shrink, never squash: the arrow keeps its 1:3 aspect and the
                    // span it occupies shortens with it.
                    double width = arrow_width;
                    double length = arrow_length;
                    const double budget = lane_width * kLaneFillFraction;
                    double scaled_lo = lo;
                    double scaled_hi = hi;
                    if (budget > kZeroWidth && width > budget) {
                        const double scale = budget / width;
                        width *= scale;
                        length *= scale;
                        const double scaled_tail = tip - direction * length;
                        scaled_lo = std::min(tip, scaled_tail);
                        scaled_hi = std::max(tip, scaled_tail);
                    }

                    emit_run(out, cl, heights, lanes[lane_index].centre(), 0.5 * width,
                             scaled_lo, scaled_hi, sprite_rect(sprite),
                             /*stretch_per_band=*/false, reversed, lift);
                }
            }
        }
    }

    return out.finish();
}

} // namespace stratum::osm::road
