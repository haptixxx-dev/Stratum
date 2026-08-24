/**
 * @file tessellation.cpp
 * @brief Implementation of station decimation and coplanar quad merging
 *
 * Two passes that both remove geometry that carries no information, and share
 * nothing but a config struct.
 *
 * ### Longitudinal, select_stations()
 *
 * A Douglas-Peucker forward greedy walk. From the current kept station, extend
 * the span while every station it would absorb stays within
 * TessellationConfig::max_chord_deviation of the chord that replaces it, and
 * while the span stays under TessellationConfig::max_span_length. The deviation
 * is measured in THREE dimensions whenever the caller supplies solved station
 * heights, because a road can be dead straight in plan and still crest a hill:
 * measuring in plan view alone would merge the crest away and hand the flattened
 * result back to the extruder, undoing the elevation solve.
 *
 * The measure is a point-to-SEGMENT distance rather than a point-to-line one. On
 * a fold-back -- a hairpin resampled tightly enough that a station lies behind
 * its own chord -- the perpendicular distance to the infinite line is small while
 * the station is nowhere near the chord. The segment form reports the real error.
 *
 * ### Lateral, merge_coplanar_quads()
 *
 * Three stages, because the frozen UV convention makes the interesting merge
 * illegal until something is done about it:
 *
 * 1. **Pair triangles into quads.** Two triangles of one SubMesh range sharing a
 *    diagonal, whose union is a convex planar quadrilateral.
 * 2. **Merge what is already UV-continuous.** Adjacent quads whose UVs are
 *    affinely continuous across the shared edge merge with no rewrite at all.
 *    This is the along-the-road case: V is arclength in metres, so it is affine
 *    in position exactly where the stations are collinear, which is exactly where
 *    the merge is legal anyway.
 * 3. **Harmonise U, then merge again.** Across a STRIP boundary U restarts at 0,
 *    so two lanes of one carriageway are a U discontinuity. Removing it means
 *    shifting one strip's U by the other's U span -- and a vertex's UV is shared
 *    by every triangle that references it, so the shift is only legal if it is
 *    applied to every one of them at once.
 *
 *    That is what stage 3 solves. Quads are nodes of a constraint graph: sharing
 *    a vertex forces two quads to the SAME U offset, and a mergeable seam forces
 *    a DIFFERENCE of one U span. A connected component whose offsets are
 *    consistent, and none of whose vertices is touched by a triangle outside the
 *    component, can have the whole assignment applied in one write. On a corridor
 *    that component is precisely "every band of strip A at offset 0, every band of
 *    strip B at offset span(A)", which is why the lateral merge fires on a curved
 *    road and not only on a straight one.
 *
 * A merge is refused unless the merged quad's extent ALONG THE DIRECTION OF THE
 * MERGE stays within TessellationConfig::max_span_length. Without that, the
 * along-the-road merges of stage 2 would chain without limit and undo the span
 * cap that select_stations() just honoured. Measuring along the merge direction
 * rather than over the whole quad is what lets a 120 m band still widen
 * laterally.
 *
 * Everything here lives in stratum_core: no SDL, no ImGui, no rendering API.
 */

#include "osm/road/tessellation.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <array>
#include <cstring>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

namespace stratum::osm::road {

namespace {

// ============================================================================
// Shared constants
// ============================================================================

/// Below this a squared length is treated as zero rather than normalised
constexpr double kLenEpsilon = 1e-12;

/// Curvature magnitudes below this count as "straight" for the inflection test
constexpr double kCurvatureDeadband = 1e-7;

/// Two stations closer than this in metres are the two halves of a bevel pair
constexpr double kCoincidentEpsilon = 1e-9;

/**
 * @brief Hard cap on how many stations one span may absorb
 *
 * TessellationConfig::max_span_length normally closes a span long before this.
 * The cap exists for a degenerate centerline whose arclengths do not advance, on
 * which the span test never fires and the deviation scan would go quadratic in
 * the whole edge.
 */
constexpr size_t kMaxSpanStations = 256;

/**
 * @brief Slack on the curvature-derived early break
 *
 * The sagitta of a circular arc is kappa * L^2 / 8. A resampled polyline is not
 * an arc, so the estimate is only used to break out of a scan whose exact answer
 * is already far outside the budget. Breaking early can only KEEP a station that
 * might have been dropped, never drop one that should have been kept.
 */
constexpr double kCurvatureRejectFactor = 4.0;

/// Absolute floor of the geometric tolerances, metres
constexpr double kGeometryAbsTolerance = 1e-3;

/// Relative part of the geometric tolerances, against the coordinate magnitude
constexpr double kGeometryRelTolerance = 1e-6;

/// A seam vertex must sit strictly inside its new edge, by this parameter margin
constexpr double kParamMargin = 1e-3;

/// UV agreement tolerance, in tile units
constexpr float kUvEpsilon = 1e-3f;

/// Vertex colour agreement tolerance, per channel
constexpr float kColorEpsilon = 1e-3f;

/// Relative area agreement between a merged quad and the two it replaces
constexpr double kAreaRelTolerance = 1e-3;

/// Ceiling on merge/harmonise rounds, so a pathological mesh cannot spin
constexpr size_t kMaxMergeRounds = 8;

/// Ceiling on sweeps within one merge round
constexpr size_t kMaxSweepsPerRound = 64;

constexpr size_t kNoIndex = std::numeric_limits<size_t>::max();
constexpr uint32_t kNoTriangle = std::numeric_limits<uint32_t>::max();
constexpr uint32_t kNoQuadRef = std::numeric_limits<uint32_t>::max();

// ============================================================================
// Small 2D helpers, matching centerline.cpp exactly
// ============================================================================

/// Left normal of a unit tangent; the same quarter turn centerline.cpp uses
[[nodiscard]] inline glm::dvec2 left_normal_2d(const glm::dvec2& t) {
    return glm::dvec2(-t.y, t.x);
}

[[nodiscard]] inline double cross2(const glm::dvec2& a, const glm::dvec2& b) {
    return a.x * b.y - a.y * b.x;
}

[[nodiscard]] inline glm::dvec2 normalize_or(const glm::dvec2& v, const glm::dvec2& fallback) {
    const double len_sq = v.x * v.x + v.y * v.y;
    if (!(len_sq > kLenEpsilon) || !std::isfinite(len_sq)) return fallback;
    return v / std::sqrt(len_sq);
}

/// Two stations are the halves of one bevel pair when position and arclength coincide
[[nodiscard]] inline bool stations_coincide(const Station& a, const Station& b) {
    return glm::distance(a.position, b.position) <= kCoincidentEpsilon &&
           std::fabs(a.arclength - b.arclength) <= kCoincidentEpsilon;
}

/**
 * @brief Re-derive Station::lateral_min / lateral_max for a station list
 *
 * The same algebra as centerline.cpp's bound_lateral_folds(), reproduced rather
 * than exported because it is an internal of the centerline builder and this file
 * must not widen that header's surface. It only ever TIGHTENS, so running it over
 * bounds already tightened by the absorbed runs yields the intersection of the
 * two, which is what a decimated ribbon needs: the new bands must not fold, and
 * neither must the profile that the dropped bands used to bound.
 */
void tighten_lateral_folds(std::vector<Station>& st) {
    constexpr double kFoldSafety = 1.0 - 1e-12;

    for (size_t i = 0; i + 1 < st.size(); ++i) {
        Station& a = st[i];
        Station& b = st[i + 1];

        const glm::dvec2 ua = a.normal * a.miter_scale;
        const glm::dvec2 ub = b.normal * b.miter_scale;
        const glm::dvec2 d = b.position - a.position;

        const double c = cross2(ua, ub);
        if (!std::isfinite(c) || std::fabs(c) <= kLenEpsilon) continue;

        const double lim_a = -cross2(ua, d) / c;
        const double lim_b = -cross2(ub, d) / c;
        if (!std::isfinite(lim_a) || !std::isfinite(lim_b)) continue;

        if (c > 0.0) {
            const double lim = std::max(std::min(lim_a, lim_b), 0.0) * kFoldSafety;
            a.lateral_max = std::min(a.lateral_max, lim);
            b.lateral_max = std::min(b.lateral_max, lim);
        } else {
            const double lim = std::min(std::max(lim_a, lim_b), 0.0) * kFoldSafety;
            a.lateral_min = std::max(a.lateral_min, lim);
            b.lateral_min = std::max(b.lateral_min, lim);
        }
    }
}

} // namespace

// ============================================================================
// Longitudinal: station selection
// ============================================================================

std::vector<size_t> select_stations(const Centerline& cl,
                                    const TessellationConfig& cfg,
                                    const std::vector<bool>* feature_stations,
                                    const std::vector<float>* station_heights) {
    const size_t n = cl.stations.size();

    std::vector<size_t> keep;
    if (n == 0) return keep;
    if (n < 3) {
        keep.reserve(n);
        for (size_t i = 0; i < n; ++i) keep.push_back(i);
        return keep;
    }

    // A mis-sized height vector degrades to "no heights", exactly as
    // CorridorConfig::station_heights degrades to a flat road, rather than
    // indexing out of range or measuring against a shifted parallel run.
    const bool use_heights = (station_heights != nullptr) && (station_heights->size() == n);

    // The 2D-to-world mapping (x, y) -> (x, h, -y) is an isometry, so measuring
    // in (x, y, h) gives exactly the distance the player sees. No flip is needed.
    const auto point_at = [&](size_t i) -> glm::dvec3 {
        const double h = use_heights ? static_cast<double>((*station_heights)[i]) : 0.0;
        return glm::dvec3(cl.stations[i].position.x, cl.stations[i].position.y, h);
    };

    // ------------------------------------------------------------------------
    // What may never be dropped
    // ------------------------------------------------------------------------
    std::vector<char> forced(n, 0);
    forced[0] = 1;
    forced[n - 1] = 1;

    if (cfg.preserve_feature_stations && feature_stations != nullptr) {
        const size_t covered = std::min(n, feature_stations->size());
        for (size_t i = 0; i < covered; ++i) {
            if ((*feature_stations)[i]) forced[i] = 1;
        }
    }

    // A bevel is a real corner, not a smoothable one, and its two halves are a
    // unit: keeping one without the other turns the joint into a mitre carrying
    // the wrong normal. The partner is found by coincidence rather than trusted
    // to be flagged, so a single flagged half still protects both.
    for (size_t i = 0; i < n; ++i) {
        if (!cl.stations[i].is_bevel) continue;
        forced[i] = 1;
        if (i + 1 < n && stations_coincide(cl.stations[i], cl.stations[i + 1])) forced[i + 1] = 1;
        if (i > 0 && stations_coincide(cl.stations[i - 1], cl.stations[i])) forced[i - 1] = 1;
    }

    // An inflection is where the road stops turning one way and starts turning
    // the other. The chord test can pass straight through one: an S-bend whose
    // two halves cancel measures as almost straight while looking nothing like a
    // straight.
    {
        int last_sign = 0;
        for (size_t i = 0; i < n; ++i) {
            const double k = cl.stations[i].curvature;
            if (!std::isfinite(k) || !(std::fabs(k) > kCurvatureDeadband)) continue;
            const int sign = (k > 0.0) ? 1 : -1;
            if (last_sign != 0 && sign != last_sign) forced[i] = 1;
            last_sign = sign;
        }
    }

    // ------------------------------------------------------------------------
    // Deviation of a candidate span, in 3D, as a point-to-SEGMENT distance
    // ------------------------------------------------------------------------
    const auto span_deviation = [&](size_t i, size_t j) -> double {
        const glm::dvec3 a = point_at(i);
        const glm::dvec3 d = point_at(j) - a;
        const double len_sq = glm::dot(d, d);

        double worst = 0.0;
        for (size_t k = i + 1; k < j; ++k) {
            const glm::dvec3 w = point_at(k) - a;
            double t = (len_sq > kLenEpsilon) ? (glm::dot(w, d) / len_sq) : 0.0;
            t = std::min(std::max(t, 0.0), 1.0);
            const glm::dvec3 r = w - d * t;
            worst = std::max(worst, std::sqrt(glm::dot(r, r)));
        }
        return worst;
    };

    const double budget = (cfg.max_chord_deviation > 0.0 && std::isfinite(cfg.max_chord_deviation))
                              ? cfg.max_chord_deviation
                              : 0.0;
    const double span_cap = (cfg.max_span_length > 0.0 && std::isfinite(cfg.max_span_length))
                                ? cfg.max_span_length
                                : std::numeric_limits<double>::infinity();

    // ------------------------------------------------------------------------
    // Greedy forward walk
    // ------------------------------------------------------------------------
    keep.reserve(n);
    keep.push_back(0);

    size_t i = 0;
    while (i + 1 < n) {
        // A span may not skip a forced station, so it can reach no further than
        // the next one. n - 1 is forced, so this always terminates.
        size_t next_forced = i + 1;
        while (next_forced < n && !forced[next_forced]) ++next_forced;

        // The immediate neighbour is always reachable: there is nothing between
        // it and the current station to deviate, and refusing it would fail to
        // advance. A single band longer than the span cap is kept for the same
        // reason -- this pass removes stations, it never inserts them.
        size_t best = i + 1;

        double kappa_max = std::max(std::fabs(cl.stations[i].curvature),
                                    std::fabs(cl.stations[i + 1].curvature));
        if (!std::isfinite(kappa_max)) kappa_max = 0.0;

        const size_t reach = std::min(next_forced, i + kMaxSpanStations);
        for (size_t j = i + 2; j <= reach; ++j) {
            const double kj = std::fabs(cl.stations[j].curvature);
            if (std::isfinite(kj)) kappa_max = std::max(kappa_max, kj);

            const double span = cl.stations[j].arclength - cl.stations[i].arclength;
            if (!(span <= span_cap)) break;

            // Curvature is on the station already. It is used to abandon a scan
            // whose exact answer is certainly outside the budget, never as the
            // merge criterion itself -- the deviation test below is the
            // correctness condition.
            if (kappa_max > 0.0 && (kappa_max * span * span / 8.0) > kCurvatureRejectFactor * budget) {
                break;
            }

            if (span_deviation(i, j) > budget) break;

            best = j;
        }

        keep.push_back(best);
        i = best;
    }

    return keep;
}

// ============================================================================
// Longitudinal: applying a selection
// ============================================================================

Centerline apply_station_selection(const Centerline& cl,
                                   const std::vector<size_t>& keep,
                                   double miter_limit) {
    Centerline out;
    const size_t n = cl.stations.size();
    if (n == 0) return out;

    // Out-of-range indices are ignored and a non-ascending one is dropped rather
    // than trusted, so a caller error degrades to a coarser centerline instead of
    // an out-of-bounds read.
    std::vector<size_t> idx;
    idx.reserve(keep.size());
    for (const size_t k : keep) {
        if (k >= n) continue;
        if (!idx.empty() && k <= idx.back()) continue;
        idx.push_back(k);
    }
    if (idx.size() < 2) return out;

    out.stations.reserve(idx.size());
    for (const size_t k : idx) out.stations.push_back(cl.stations[k]);

    // ------------------------------------------------------------------------
    // 0. The two END stations are FROZEN
    //
    // select_stations() forces the first and last index, but forcing the index
    // only preserves the position -- everything below would still rewrite the
    // frame and the fold bounds there. It must not. An edge's end station is the
    // one the junction solver already consumed: arm_end() slices the SAME
    // centerline at the SAME cut and offsets `stations.front()` through
    // offset_point() to place the junction polygon's arm mouth and the curb
    // ring's arm reach. build_corridor() emits the ribbon's first column through
    // that identical call, so the two register only while the station is
    // bit-identical -- normal, miter_scale and lateral bounds included.
    //
    // A trim almost never lands on an existing station, so slice() synthesises
    // the end station by interpolating the miter frame across the band it cut.
    // Rebuilding that frame from the decimated chord rotates it (measured 2.3
    // degrees on a 25 m kerb line, moving a 7 m offset corner 0.25 m), and
    // charging an absorbed run's fold bounds to it can clamp the same column
    // metres inboard. Either one opens a wedge between the ribbon and the
    // junction mouth at every decimated arm.
    //
    // Interior stations are unaffected and still get the corrected bisector,
    // which is what the merge genuinely needs.
    // ------------------------------------------------------------------------
    const Station end_front = out.stations.front();
    const Station end_back = out.stations.back();

    // ------------------------------------------------------------------------
    // 1. The fold bounds are TIGHTENED over each absorbed run, not copied
    //
    // A kept station now stands for the whole span it absorbed, so a bend that no
    // longer has a station in it must still bound the profile that crosses it.
    // Each dropped run is charged to BOTH of the kept stations bracketing it,
    // which is the conservative reading.
    // ------------------------------------------------------------------------
    for (size_t a = 0; a + 1 < idx.size(); ++a) {
        const size_t lo = idx[a];
        const size_t hi = idx[a + 1];
        if (hi <= lo + 1) continue;

        double run_min = -kUnboundedLateral;
        double run_max = kUnboundedLateral;
        for (size_t k = lo + 1; k < hi; ++k) {
            run_min = std::max(run_min, cl.stations[k].lateral_min);
            run_max = std::min(run_max, cl.stations[k].lateral_max);
        }

        out.stations[a].lateral_min = std::max(out.stations[a].lateral_min, run_min);
        out.stations[a].lateral_max = std::min(out.stations[a].lateral_max, run_max);
        out.stations[a + 1].lateral_min = std::max(out.stations[a + 1].lateral_min, run_min);
        out.stations[a + 1].lateral_max = std::min(out.stations[a + 1].lateral_max, run_max);
    }

    // ------------------------------------------------------------------------
    // 2. The frame is REBUILT, because both of its angles changed
    //
    // Station::normal is the miter bisector of the two segments meeting at the
    // station and Station::miter_scale is 1/cos(theta/2) for that joint. Dropping
    // the stations between two kept ones changes both. Carrying the old frame
    // across pinches the ribbon at exactly the joints the merge created.
    //
    // The walk is over DISTINCT positions. A bevel pair is two stations at one
    // position, so the band between them has no direction; compressing the pair
    // to one entry gives the station after it the outgoing leg as its incoming
    // direction, which is what build_frames() does on the undecimated run.
    // ------------------------------------------------------------------------
    std::vector<glm::dvec2> dpos;
    std::vector<size_t> station_to_dpos(out.stations.size(), 0);
    dpos.reserve(out.stations.size());
    for (size_t s = 0; s < out.stations.size(); ++s) {
        const glm::dvec2& p = out.stations[s].position;
        if (dpos.empty() || glm::distance(dpos.back(), p) > kCoincidentEpsilon) {
            dpos.push_back(p);
        }
        station_to_dpos[s] = dpos.size() - 1;
    }

    if (dpos.size() >= 2) {
        std::vector<glm::dvec2> dir(dpos.size() - 1, glm::dvec2(1.0, 0.0));
        for (size_t e = 0; e + 1 < dpos.size(); ++e) {
            const glm::dvec2 fallback = (e > 0) ? dir[e - 1] : glm::dvec2(1.0, 0.0);
            dir[e] = normalize_or(dpos[e + 1] - dpos[e], fallback);
        }

        const double limit = (miter_limit >= 1.0 && std::isfinite(miter_limit)) ? miter_limit : 1.0;

        for (size_t s = 0; s < out.stations.size(); ++s) {
            Station& st = out.stations[s];

            // A bevel carries the incoming or outgoing leg's normal verbatim and
            // a miter_scale of exactly 1. Re-deriving a bisector for it would
            // destroy the wedge the pair exists to describe.
            if (st.is_bevel) continue;

            // The ends are frozen (step 0). Skipping them here rather than only
            // restoring them afterwards matters: step 3 measures the fold limit
            // of the first and last BAND from these frames, and it must measure
            // the frame the junction solver actually used.
            if (s == 0 || s + 1 == out.stations.size()) continue;

            const size_t e = station_to_dpos[s];
            if (e == 0 || e + 1 == dpos.size()) {
                const glm::dvec2 t = (e == 0) ? dir.front() : dir.back();
                st.tangent = t;
                st.normal = left_normal_2d(t);
                st.miter_scale = 1.0;
                continue;
            }

            const glm::dvec2 t_in = dir[e - 1];
            const glm::dvec2 t_out = dir[e];
            const glm::dvec2 n_in = left_normal_2d(t_in);
            const glm::dvec2 n_out = left_normal_2d(t_out);

            const glm::dvec2 sum = n_in + n_out;
            const double sum_len_sq = glm::dot(sum, sum);
            if (!(sum_len_sq > kLenEpsilon) || !std::isfinite(sum_len_sq)) continue;

            const glm::dvec2 bisector = sum / std::sqrt(sum_len_sq);
            const double cos_half = glm::dot(bisector, n_in);
            if (!(cos_half > kLenEpsilon) || !std::isfinite(cos_half)) continue;

            double scale = 1.0 / cos_half;
            if (!std::isfinite(scale)) continue;

            // A joint sharp enough to exceed the miter limit is a bevel, and a
            // bevel is two stations. This pass removes stations and never inserts
            // them, so an over-limit joint is clamped rather than bevelled. A
            // selection that respects the deviation budget cannot produce one:
            // the stations either side of a 150-degree turn are metres off any
            // chord through it, so no span absorbs them.
            st.tangent = normalize_or(t_in + t_out, t_in);
            st.normal = bisector;
            st.miter_scale = std::min(scale, limit);
        }
    }

    // ------------------------------------------------------------------------
    // 3. Fold bounds for the bands that now exist
    //
    // Step 1 carried forward what the dropped bands bounded; this bounds what the
    // merged bands bound. Both only tighten, so the result is the intersection.
    // ------------------------------------------------------------------------
    tighten_lateral_folds(out.stations);

    // ------------------------------------------------------------------------
    // 4. Restore the frozen ends
    //
    // Steps 1 to 3 only ever tighten, and step 2 only ever re-derives, so the two
    // end stations are put back exactly as they were read. Position, arclength,
    // curvature and is_bevel were already verbatim copies; the frame and the fold
    // bounds are what needed protecting. See the note at step 0.
    // ------------------------------------------------------------------------
    out.stations.front() = end_front;
    out.stations.back() = end_back;

    return out;
}

// ============================================================================
// Lateral: coplanar quad merging
// ============================================================================

namespace {

/**
 * @brief One quadrilateral, either a paired triangle couple or a merged result
 *
 * `v` is in counter-clockwise order. `source` names the two triangles the pair
 * was built from; while `merged` is false those exact triangles are re-emitted
 * verbatim, so a pass that merges nothing anywhere else does not perturb this
 * quad's winding or index order.
 */
struct QuadRec {
    uint32_t v[4] = {0, 0, 0, 0};
    uint32_t source[2] = {kNoTriangle, kNoTriangle};
    uint32_t range = 0;
    size_t   order = 0;         ///< lowest source triangle id, for output ordering
    bool     merged = false;
    bool     alive = true;

    /**
     * @brief Unit face normal and area, cached
     *
     * Recomputed on every merge, which is the only thing that moves a corner. The
     * coplanarity test is the first geometric filter a candidate pair meets and it
     * runs tens of millions of times over a full extract, so paying Newell's
     * method there rather than once per quad measured.
     */
    glm::dvec3 normal{0.0, 1.0, 0.0};
    double area = 0.0;
};

/// Directed-edge key over vertex indices
[[nodiscard]] inline uint64_t edge_key(uint32_t a, uint32_t b) {
    return (static_cast<uint64_t>(a) << 32) | static_cast<uint64_t>(b);
}

/// Undirected positional-edge key over position ids
[[nodiscard]] inline uint64_t pos_edge_key(uint32_t a, uint32_t b) {
    const uint32_t lo = std::min(a, b);
    const uint32_t hi = std::max(a, b);
    return (static_cast<uint64_t>(lo) << 32) | static_cast<uint64_t>(hi);
}

/**
 * @brief A vertex position compared by its exact float bit pattern
 *
 * Adjacent strips of a corridor share edge POSITIONS but never vertices, because
 * the frozen UV convention restarts U at each strip boundary. Adjacency between
 * two strips is therefore positional and cannot be found from vertex indices.
 *
 * The comparison is IDENTITY, not proximity. The extruder computes a shared
 * boundary from the same double on both sides and casts it to the same float, so
 * the two agree bit for bit, and every other producer in the pipeline coincides
 * the same way -- by computing the same number twice, not by landing near it. A
 * tolerance would buy nothing here and would start welding a thin strip shut; it
 * would also cost a neighbourhood probe per vertex, which measured as the single
 * most expensive thing this pass did.
 *
 * Negative zero is normalised so that 0.0f and -0.0f, which compare equal as
 * floats, also hash and compare equal here.
 */
struct PositionKey {
    uint32_t bits[3] = {0, 0, 0};

    explicit PositionKey(const glm::vec3& p) {
        const float c[3] = {p.x, p.y, p.z};
        for (int i = 0; i < 3; ++i) {
            uint32_t w = 0;
            std::memcpy(&w, &c[i], sizeof(w));
            if (w == 0x80000000u) w = 0u;
            bits[static_cast<size_t>(i)] = w;
        }
    }

    bool operator==(const PositionKey& o) const {
        return bits[0] == o.bits[0] && bits[1] == o.bits[1] && bits[2] == o.bits[2];
    }
};

struct PositionKeyHash {
    size_t operator()(const PositionKey& k) const noexcept {
        uint64_t h = 1469598103934665603ull;
        for (const uint32_t w : k.bits) {
            h ^= static_cast<uint64_t>(w);
            h *= 1099511628211ull;
        }
        return static_cast<size_t>(h);
    }
};

/// Collapse coincident vertex positions to shared position ids; see PositionKey
[[nodiscard]] std::vector<uint32_t> build_position_ids(const std::vector<Vertex>& verts) {
    std::vector<uint32_t> pid(verts.size(), 0);
    std::unordered_map<PositionKey, uint32_t, PositionKeyHash> seen;
    seen.reserve(verts.size() * 2);

    uint32_t next = 0;
    for (size_t i = 0; i < verts.size(); ++i) {
        const auto [it, inserted] = seen.emplace(PositionKey(verts[i].position), next);
        if (inserted) ++next;
        pid[i] = it->second;
    }
    return pid;
}

/// Scale-aware geometric tolerance: float positions lose absolute precision far from the origin
[[nodiscard]] inline double geometry_tolerance(const glm::dvec3& p) {
    const double magnitude = std::fabs(p.x) + std::fabs(p.y) + std::fabs(p.z);
    return std::max(kGeometryAbsTolerance, kGeometryRelTolerance * magnitude);
}

[[nodiscard]] inline bool colors_agree(const glm::vec4& a, const glm::vec4& b) {
    return std::fabs(a.x - b.x) <= kColorEpsilon && std::fabs(a.y - b.y) <= kColorEpsilon &&
           std::fabs(a.z - b.z) <= kColorEpsilon && std::fabs(a.w - b.w) <= kColorEpsilon;
}

/**
 * @brief dot(normalize(a), normalize(b)) >= eps, without normalising either
 *
 * Squared throughout: the comparison is called several times per candidate pair
 * and two square roots each measured.
 */
[[nodiscard]] inline bool unit_vectors_agree(const glm::vec3& a, const glm::vec3& b, double eps) {
    const glm::dvec3 da(a);
    const glm::dvec3 db(b);
    const double la = glm::dot(da, da);
    const double lb = glm::dot(db, db);
    if (!(la > kLenEpsilon) || !(lb > kLenEpsilon)) return false;
    const double d = glm::dot(da, db);
    if (eps >= 0.0) {
        return d >= 0.0 && (d * d) >= (eps * eps) * la * lb;
    }
    return d >= 0.0 || (d * d) <= (eps * eps) * la * lb;
}

/**
 * @brief Area-weighted normal of a planar polygon given as a vertex loop
 *
 * Newell's method, so a slightly warped quad still reports the normal of its best
 * fitting plane rather than that of whichever triangle happened to be measured.
 */
[[nodiscard]] glm::dvec3 loop_normal(const glm::dvec3* p, size_t count) {
    glm::dvec3 n(0.0);
    for (size_t i = 0; i < count; ++i) {
        const glm::dvec3& a = p[i];
        const glm::dvec3& b = p[(i + 1) % count];
        n.x += (a.y - b.y) * (a.z + b.z);
        n.y += (a.z - b.z) * (a.x + b.x);
        n.z += (a.x - b.x) * (a.y + b.y);
    }
    return n * 0.5;
}

/// Every corner turns the same way about @p normal, so the loop is convex
[[nodiscard]] bool loop_is_convex(const glm::dvec3* p, size_t count, const glm::dvec3& normal) {
    for (size_t i = 0; i < count; ++i) {
        const glm::dvec3& a = p[i];
        const glm::dvec3& b = p[(i + 1) % count];
        const glm::dvec3& c = p[(i + 2) % count];
        if (glm::dot(glm::cross(b - a, c - b), normal) <= 0.0) return false;
    }
    return true;
}

/// Convex, non-degenerate, and flat to within @p normal_eps
[[nodiscard]] bool loop_is_convex_planar(const glm::dvec3* p, size_t count, double normal_eps) {
    const glm::dvec3 n = loop_normal(p, count);
    const double area_sq = glm::dot(n, n);
    if (!(area_sq > kLenEpsilon) || !std::isfinite(area_sq)) return false;
    const glm::dvec3 unit = n / std::sqrt(area_sq);

    if (!loop_is_convex(p, count, unit)) return false;

    // Flatness: no corner may leave the plane through the loop centroid by more
    // than the sag that normal_eps allows over the loop's own extent.
    glm::dvec3 centroid(0.0);
    for (size_t i = 0; i < count; ++i) centroid += p[i];
    centroid /= static_cast<double>(count);

    double extent = 0.0;
    for (size_t i = 0; i < count; ++i) {
        extent = std::max(extent, glm::length(p[i] - centroid));
    }

    const double sin_eps = std::sqrt(std::max(0.0, 1.0 - normal_eps * normal_eps));
    const double allowed = std::max(geometry_tolerance(centroid), extent * sin_eps);
    for (size_t i = 0; i < count; ++i) {
        if (std::fabs(glm::dot(p[i] - centroid, unit)) > allowed) return false;
    }
    return true;
}

/// Fill QuadRec::normal and QuadRec::area from the quad's four current corners
inline void refresh_quad_plane(QuadRec& q, const std::vector<Vertex>& verts) {
    const glm::dvec3 loop[4] = {glm::dvec3(verts[q.v[0]].position), glm::dvec3(verts[q.v[1]].position),
                                glm::dvec3(verts[q.v[2]].position), glm::dvec3(verts[q.v[3]].position)};
    const glm::dvec3 n = loop_normal(loop, 4);
    const double len_sq = glm::dot(n, n);
    if (len_sq > kLenEpsilon && std::isfinite(len_sq)) {
        const double len = std::sqrt(len_sq);
        q.normal = n / len;
        q.area = len;
    } else {
        q.normal = glm::dvec3(0.0, 1.0, 0.0);
        q.area = 0.0;
    }
}

/**
 * @brief Where a point sits on a segment, and how far off it lies
 *
 * @return false when the segment is degenerate, when the point is off it by more
 *         than the scale-aware tolerance, or when the parameter is not strictly
 *         interior. The interior requirement is what makes the UV solve in
 *         evaluate_pair() well posed.
 */
[[nodiscard]] bool segment_parameter(const glm::dvec3& from, const glm::dvec3& to,
                                     const glm::dvec3& point, double& out_t) {
    const glm::dvec3 d = to - from;
    const double len_sq = glm::dot(d, d);
    if (!(len_sq > kLenEpsilon) || !std::isfinite(len_sq)) return false;

    const double t = glm::dot(point - from, d) / len_sq;
    if (!(t > kParamMargin) || !(t < 1.0 - kParamMargin)) return false;

    const glm::dvec3 offset = (point - from) - d * t;
    const double tol = geometry_tolerance(point);
    if (glm::dot(offset, offset) > tol * tol) return false;

    out_t = t;
    return true;
}

/**
 * @brief State the merge loop carries between its stages
 */
struct MergeState {
    Mesh* mesh = nullptr;
    const std::vector<uint32_t>* pid = nullptr;
    std::vector<QuadRec> quads;

    /**
     * @brief Positional side -> the quads presenting it, and which side it is
     *
     * Built once and MAINTAINED, never rebuilt. A merge changes a quad's corners,
     * which leaves entries behind pointing at sides it no longer has; those are
     * harmless, because evaluate_pair() re-derives both quads' current corners and
     * rejects a pair whose shared side is not actually shared. Rebuilding the map
     * on every sweep instead measured as the second most expensive thing this pass
     * did, after the position weld.
     *
     * The slots are inline rather than a vector per key. A side of a surface is
     * shared by two faces, so four slots absorb the two live entries plus the
     * stale ones a merge leaves behind; a key that fills anyway drops the new
     * entry, which costs a merge and never costs correctness. One heap allocation
     * per side was the third measurable cost of this pass.
     */
    struct SideRefs {
        uint32_t quad[4] = {kNoQuadRef, kNoQuadRef, kNoQuadRef, kNoQuadRef};
        int8_t   side[4] = {0, 0, 0, 0};
    };
    std::unordered_map<uint64_t, SideRefs> side_map;

    /**
     * @brief Candidate seams a sweep refused because they need a U rewrite
     *
     * Recorded by merge_sweep() and consumed by harmonise_u(), so the seam scan
     * happens once instead of twice. Only the list left by a sweep that merged
     * NOTHING is used: after such a sweep no quad has changed, so every pair in it
     * is still exactly as it was evaluated.
     */
    std::vector<std::array<uint32_t, 2>> pending_seams;
    std::vector<float> pending_deltas;

    /// pending_seams was left by a sweep that merged nothing, so it is usable
    bool pending_valid = false;

    /// Publish quad @p qi's four current sides into side_map
    void register_sides(uint32_t qi) {
        for (int s = 0; s < 4; ++s) {
            const uint32_t a = (*pid)[quads[qi].v[s]];
            const uint32_t b = (*pid)[quads[qi].v[(s + 1) % 4]];
            SideRefs& refs = side_map[pos_edge_key(a, b)];
            int free_slot = -1;
            for (int k = 0; k < 4; ++k) {
                if (refs.quad[k] == kNoQuadRef) { free_slot = k; break; }
                if (!quads[refs.quad[k]].alive) { free_slot = k; break; }
            }
            if (free_slot < 0) continue;
            refs.quad[free_slot] = qi;
            refs.side[free_slot] = static_cast<int8_t>(s);
        }
    }

    /**
     * @brief Vertex touched by a triangle that belongs to no quad
     *
     * The coverage rule harmonise_u() enforces. Every triangle is either part of
     * a quad or loose, and quads sharing a vertex are always in one component, so
     * "no loose triangle touches this vertex" is exactly "every triangle
     * referencing it is inside the component".
     */
    std::vector<char> vertex_has_loose;
    double normal_eps = 0.999;
    double span_cap = 120.0;
};

/**
 * @brief Everything one candidate merge needs to be judged and then applied
 */
struct PairEval {
    uint32_t merged[4] = {0, 0, 0, 0};   ///< A, B, C, D of the merged quad
    glm::vec2 delta{0.0f};               ///< U offset the far quad's UVs need
    bool needs_rewrite = false;          ///< delta is not zero
};

/**
 * @brief Judge quad @p qi against quad @p ri across sides @p sq / @p sr
 *
 * The merged quad is M = (A, B, C, D) where, writing the shared side of Q as
 * (v_k, v_k+1):
 *
 *     A = Q[sq+2]   B = Q[sq+3]   C = R[sr+2]   D = R[sr+3]
 *
 * and the two seam vertices are dropped. They can only be dropped because each
 * lies strictly inside one of M's new sides -- v_k inside B->C and v_k+1 inside
 * D->A -- which is checked, not assumed. That is what makes the merge a
 * topological rewrite rather than a simplification: M covers exactly the union of
 * the two quads it replaces.
 *
 * ### The UV argument
 *
 * The frozen convention is metres-based: U is lateral metres within the strip
 * over the material's tile size, V is arclength metres over the material's tile
 * size. Both are affine in position wherever the geometry the merge is about to
 * create is itself affine -- which the collinearity tests have just established.
 *
 * That makes the merged quad's corner UVs solvable rather than guessable. With
 * s_BC the parameter of v_k along B->C and s_DA that of v_k+1 along D->A, the
 * merged mapping reproduces the near quad exactly when
 *
 *     uv(v_k)   == lerp(uv(B), uv(C), s_BC)
 *     uv(v_k+1) == lerp(uv(D), uv(A), s_DA)
 *
 * and reproduces the far quad up to one constant offset `delta`, read off the
 * seam itself as uv(v_k) - uv(w_m+1). Both seam pairs must give the same delta,
 * and delta must be zero in V: a V shift would slide the far quad along the road
 * and break the arclength placement that markings, crossings and trims all hold.
 *
 * Two adjacent lanes are exactly the case where delta is non-zero, because U
 * restarts at 0 at every strip boundary. Applying it is legal only under the
 * coverage rule enforced by harmonise_u(); this function only measures it.
 */
[[nodiscard]] bool evaluate_pair(const MergeState& st, uint32_t qi, uint32_t ri,
                                 int sq, int sr, PairEval& out) {
    const QuadRec& Q = st.quads[qi];
    const QuadRec& R = st.quads[ri];
    if (!Q.alive || !R.alive || qi == ri) return false;

    // Never across a MaterialKey boundary. One SubMesh range is one (slot,
    // variant) pair, so range identity is the whole test.
    if (Q.range != R.range) return false;

    const std::vector<uint32_t>& pid = *st.pid;
    const std::vector<Vertex>& verts = st.mesh->vertices;

    const uint32_t vk  = Q.v[sq];
    const uint32_t vk1 = Q.v[(sq + 1) % 4];
    const uint32_t A   = Q.v[(sq + 2) % 4];
    const uint32_t B   = Q.v[(sq + 3) % 4];

    const uint32_t wm  = R.v[sr];
    const uint32_t wm1 = R.v[(sr + 1) % 4];
    const uint32_t C   = R.v[(sr + 2) % 4];
    const uint32_t D   = R.v[(sr + 3) % 4];

    // The shared side must be shared in OPPOSITE winding, or the two quads are
    // not two faces of one surface.
    if (pid[vk] != pid[wm1] || pid[vk1] != pid[wm]) return false;

    // Coplanarity, from the cached planes. First geometric filter because it is
    // three multiplies and it is what refuses a kerb face against its own kerb
    // top -- same material, same range, ninety degrees apart.
    if (!(Q.area > 0.0) || !(R.area > 0.0)) return false;
    if (glm::dot(Q.normal, R.normal) < st.normal_eps) return false;

    // The seam UVs must differ by ONE constant offset, read from both seam pairs
    // and identical in each. Cheap, and it refuses a mismatched parameterisation
    // before any of the geometry below is computed.
    const glm::vec2 delta_a = verts[vk].uv - verts[wm1].uv;
    const glm::vec2 delta_b = verts[vk1].uv - verts[wm].uv;
    if (std::fabs(delta_a.x - delta_b.x) > kUvEpsilon) return false;
    if (std::fabs(delta_a.y - delta_b.y) > kUvEpsilon) return false;

    // A constant U offset is a texture phase shift on a tiling material and is
    // sanctioned. A V offset is not: V is arclength, and every marking, crossing
    // and trim in the pipeline is expressed against it.
    if (std::fabs(delta_a.y) > kUvEpsilon) return false;

    const glm::dvec3 pA(verts[A].position);
    const glm::dvec3 pB(verts[B].position);
    const glm::dvec3 pC(verts[C].position);
    const glm::dvec3 pD(verts[D].position);

    // The two seam vertices must lie strictly inside the sides that replace them.
    // This is what lets them be dropped, and it is what makes the UV solve below
    // well posed.
    double s_bc = 0.0;
    double s_da = 0.0;
    if (!segment_parameter(pB, pC, glm::dvec3(verts[vk].position), s_bc)) return false;
    if (!segment_parameter(pD, pA, glm::dvec3(verts[vk1].position), s_da)) return false;

    // The merged quad must be a convex quadrilateral covering exactly the union:
    // no fold, no area gained or lost. Both sources were checked planar when they
    // were formed and have just been checked coplanar with each other, so the
    // union's flatness follows and only convexity is re-tested here.
    const glm::dvec3 loop_m[4] = {pA, pB, pC, pD};
    const glm::dvec3 nm = loop_normal(loop_m, 4);
    const double area_m_sq = glm::dot(nm, nm);
    if (!(area_m_sq > kLenEpsilon) || !std::isfinite(area_m_sq)) return false;
    const double area_m = std::sqrt(area_m_sq);
    if (std::fabs(area_m - (Q.area + R.area)) > kAreaRelTolerance * area_m) return false;
    if (!loop_is_convex(loop_m, 4, nm / area_m)) return false;

    // Growth cap, measured ALONG the direction of the merge. Without it the
    // along-the-road merges chain without limit and undo max_span_length; over
    // the whole quad instead of along the merge direction, a band already at the
    // cap could never widen laterally.
    if (std::isfinite(st.span_cap) && st.span_cap > 0.0) {
        const glm::dvec3 seam = glm::dvec3(verts[vk].position) - glm::dvec3(verts[vk1].position);
        const glm::dvec3 across = glm::cross(nm / area_m, seam);
        const double across_len_sq = glm::dot(across, across);
        if (across_len_sq > kLenEpsilon) {
            const glm::dvec3 axis = across / std::sqrt(across_len_sq);
            double lo = std::numeric_limits<double>::max();
            double hi = std::numeric_limits<double>::lowest();
            for (const glm::dvec3& p : loop_m) {
                const double t = glm::dot(p, axis);
                lo = std::min(lo, t);
                hi = std::max(hi, t);
            }
            if ((hi - lo) > st.span_cap) return false;
        }
    }

    // No merge may discard a vertex attribute it cannot reproduce. The seam
    // vertices are the ones being discarded, so each must agree with the two
    // corners of the side that absorbs it.
    const auto attributes_agree = [&](uint32_t seam, uint32_t c0, uint32_t c1) {
        const Vertex& s = verts[seam];
        for (const uint32_t c : {c0, c1}) {
            const Vertex& o = verts[c];
            if (!colors_agree(s.color, o.color)) return false;
            if (!unit_vectors_agree(s.normal, o.normal, st.normal_eps)) return false;
            const glm::vec3 ts(s.tangent);
            const glm::vec3 to(o.tangent);
            if (glm::dot(ts, ts) > 0.0f && glm::dot(to, to) > 0.0f) {
                if (!unit_vectors_agree(ts, to, st.normal_eps)) return false;
                if ((s.tangent.w < 0.0f) != (o.tangent.w < 0.0f)) return false;
            }
        }
        return true;
    };
    if (!attributes_agree(vk, B, C)) return false;
    if (!attributes_agree(vk1, D, A)) return false;
    if (!attributes_agree(wm1, B, C)) return false;
    if (!attributes_agree(wm, D, A)) return false;

    // ------------------------------------------------------------------------
    // UV affinity, with the offset applied to the far quad
    // ------------------------------------------------------------------------
    const glm::vec2 delta(0.5f * (delta_a.x + delta_b.x), 0.0f);

    const glm::vec2 uvA = verts[A].uv;
    const glm::vec2 uvB = verts[B].uv;
    const glm::vec2 uvC = verts[C].uv + delta;
    const glm::vec2 uvD = verts[D].uv + delta;

    const glm::vec2 predicted_vk = uvB + (uvC - uvB) * static_cast<float>(s_bc);
    const glm::vec2 predicted_vk1 = uvD + (uvA - uvD) * static_cast<float>(s_da);
    if (std::fabs(predicted_vk.x - verts[vk].uv.x) > kUvEpsilon) return false;
    if (std::fabs(predicted_vk.y - verts[vk].uv.y) > kUvEpsilon) return false;
    if (std::fabs(predicted_vk1.x - verts[vk1].uv.x) > kUvEpsilon) return false;
    if (std::fabs(predicted_vk1.y - verts[vk1].uv.y) > kUvEpsilon) return false;

    out.merged[0] = A;
    out.merged[1] = B;
    out.merged[2] = C;
    out.merged[3] = D;
    out.delta = delta;
    out.needs_rewrite = std::fabs(delta.x) > kUvEpsilon;
    return true;
}

/// Apply a judged merge: @p qi becomes the merged quad, @p ri dies
void apply_merge(MergeState& st, uint32_t qi, uint32_t ri, const PairEval& ev) {
    const size_t inherited_order = st.quads[ri].order;
    st.quads[ri].alive = false;

    QuadRec& q = st.quads[qi];
    q.v[0] = ev.merged[0];
    q.v[1] = ev.merged[1];
    q.v[2] = ev.merged[2];
    q.v[3] = ev.merged[3];
    q.order = std::min(q.order, inherited_order);
    q.merged = true;
    refresh_quad_plane(q, st.mesh->vertices);
}

/**
 * @brief One deterministic sweep of pairwise merges
 *
 * Only pairs that are ALREADY UV-continuous are merged. A pair that would need a
 * U rewrite is left for harmonise_u(), which is the only place a rewrite can be
 * shown to be legal; once it has run, that pair reads as continuous here.
 *
 * @return Merges performed
 */
[[nodiscard]] size_t merge_sweep(MergeState& st) {
    st.pending_seams.clear();
    st.pending_deltas.clear();

    size_t merges = 0;
    for (uint32_t qi = 0; qi < st.quads.size(); ++qi) {
        if (!st.quads[qi].alive) continue;

        bool merged_this_quad = false;
        for (int s = 0; s < 4 && !merged_this_quad; ++s) {
            const uint32_t a = (*st.pid)[st.quads[qi].v[s]];
            const uint32_t b = (*st.pid)[st.quads[qi].v[(s + 1) % 4]];
            const auto it = st.side_map.find(pos_edge_key(a, b));
            if (it == st.side_map.end()) continue;

            // Copied out: apply_merge() publishes the merged quad's new sides,
            // which can rehash the map and invalidate this iterator.
            const MergeState::SideRefs refs = it->second;

            for (int c = 0; c < 4; ++c) {
                const uint32_t ri = refs.quad[c];
                if (ri == kNoQuadRef) break;
                const int sr = refs.side[c];
                if (ri == qi || !st.quads[ri].alive) continue;

                PairEval ev;
                if (!evaluate_pair(st, qi, ri, s, sr, ev)) continue;
                if (ev.needs_rewrite) {
                    st.pending_seams.push_back({qi, ri});
                    st.pending_deltas.push_back(ev.delta.x);
                    continue;
                }

                apply_merge(st, qi, ri, ev);
                st.register_sides(qi);
                ++merges;
                merged_this_quad = true;
                break;
            }
        }
    }
    return merges;
}

/**
 * @brief Make strip-boundary seams UV-continuous, where doing so is legal
 *
 * The rewrite a lane-to-lane merge needs is not a property of one quad pair. A
 * vertex's UV is shared by every triangle that references it, so shifting U on
 * one strip's column is only correct if every triangle over that column is
 * shifted with it.
 *
 * The constraint graph makes that checkable:
 *
 * - Two quads sharing a VERTEX must end up at the same U offset, or that vertex
 *   would need two different UVs.
 * - Two quads joined by a mergeable seam must end up one seam `delta` apart.
 *
 * Breadth-first assignment over those two edge kinds gives every quad in a
 * connected component an offset, or finds a contradiction. The component may then
 * be written iff no vertex it touches is also touched by a triangle belonging to
 * no quad -- which is the rule that keeps a junction fill, whose triangles do
 * share vertices, from ever being harmonised into nonsense.
 *
 * On a corridor the component comes out as exactly "every band of strip A at
 * offset 0, every band of strip B at offset span(A)", because bands of one strip
 * share vertices while neighbouring strips do not. That is why the lateral merge
 * fires on a curve, where no along-the-road merge is available to make the
 * columns exclusive first.
 *
 * @return Vertices whose U was rewritten
 */
[[nodiscard]] size_t harmonise_u(MergeState& st) {
    const uint32_t quad_count = static_cast<uint32_t>(st.quads.size());
    if (quad_count == 0 || !st.pending_valid) return 0;

    // ------------------------------------------------------------------------
    // Constraint edges. Flat and sorted rather than a vector of vectors: a piece
    // holds a few dozen quads, so the allocations dominate the asymptotics.
    // ------------------------------------------------------------------------
    struct Constraint {
        uint32_t from = 0;
        uint32_t to = 0;
        float delta = 0.0f;
    };
    std::vector<Constraint> edges;

    // A mergeable seam forces a DIFFERENCE of one U span. The seams come from the
    // last merge sweep, which merged nothing and therefore left the quads exactly
    // as it evaluated them -- so they need no re-checking, and the scan that found
    // them is not repeated here.
    bool any_rewrite_wanted = false;
    for (size_t i = 0; i < st.pending_seams.size(); ++i) {
        const uint32_t qi = st.pending_seams[i][0];
        const uint32_t ri = st.pending_seams[i][1];
        if (!st.quads[qi].alive || !st.quads[ri].alive) continue;
        const float delta = st.pending_deltas[i];
        edges.push_back(Constraint{qi, ri, delta});
        edges.push_back(Constraint{ri, qi, -delta});
        if (std::fabs(delta) > kUvEpsilon) any_rewrite_wanted = true;
    }
    if (!any_rewrite_wanted) return 0;

    // Sharing a VERTEX forces two quads to the same offset: that vertex has one
    // UV and every triangle over it reads the same one. A star from the first
    // owner is enough to connect them.
    {
        std::unordered_map<uint32_t, uint32_t> first_owner;
        first_owner.reserve(static_cast<size_t>(quad_count) * 4);
        for (uint32_t qi = 0; qi < quad_count; ++qi) {
            if (!st.quads[qi].alive) continue;
            for (const uint32_t v : st.quads[qi].v) {
                const auto [it, inserted] = first_owner.emplace(v, qi);
                if (inserted || it->second == qi) continue;
                edges.push_back(Constraint{it->second, qi, 0.0f});
                edges.push_back(Constraint{qi, it->second, 0.0f});
            }
        }
    }

    std::sort(edges.begin(), edges.end(),
              [](const Constraint& a, const Constraint& b) { return a.from < b.from; });
    std::vector<uint32_t> edge_start(static_cast<size_t>(quad_count) + 1, 0);
    for (const Constraint& c : edges) ++edge_start[static_cast<size_t>(c.from) + 1];
    for (size_t i = 1; i < edge_start.size(); ++i) edge_start[i] += edge_start[i - 1];

    // ------------------------------------------------------------------------
    // Breadth-first offset assignment, one connected component at a time
    // ------------------------------------------------------------------------
    std::vector<int32_t> component(quad_count, -1);
    std::vector<float> offset(quad_count, 0.0f);
    std::vector<uint32_t> frontier;
    std::vector<uint32_t> members;
    std::vector<uint32_t> touched;

    // Sparse by construction: only vertices of a harmonised component are ever
    // read back, and each is written before it is read.
    std::vector<float> shift_for(st.mesh->vertices.size(), 0.0f);

    size_t rewritten = 0;
    int32_t next_component = 0;

    for (uint32_t seed = 0; seed < quad_count; ++seed) {
        if (!st.quads[seed].alive || component[seed] >= 0) continue;

        const int32_t cid = next_component++;
        bool conflicted = false;
        bool wants_rewrite = false;

        members.clear();
        frontier.clear();
        component[seed] = cid;
        offset[seed] = 0.0f;
        frontier.push_back(seed);

        while (!frontier.empty()) {
            const uint32_t cur = frontier.back();
            frontier.pop_back();
            members.push_back(cur);

            for (uint32_t e = edge_start[cur]; e < edge_start[cur + 1]; ++e) {
                const Constraint& c = edges[e];
                if (!st.quads[c.to].alive) continue;
                const float want = offset[cur] + c.delta;
                if (std::fabs(c.delta) > kUvEpsilon) wants_rewrite = true;
                if (component[c.to] < 0) {
                    component[c.to] = cid;
                    offset[c.to] = want;
                    frontier.push_back(c.to);
                } else if (std::fabs(offset[c.to] - want) > kUvEpsilon) {
                    conflicted = true;
                }
            }
        }

        if (conflicted || !wants_rewrite) continue;

        // Coverage. A vertex the component touches must not be referenced by a
        // triangle outside it. Quads sharing a vertex are joined by a zero-delta
        // constraint and are therefore in this component already, so the only way
        // out is a triangle belonging to no quad at all.
        bool covered = true;
        for (const uint32_t qi : members) {
            for (const uint32_t v : st.quads[qi].v) {
                if (st.vertex_has_loose[v]) { covered = false; break; }
            }
            if (!covered) break;
        }
        if (!covered) continue;

        // Apply. Every quad owning a vertex is in this component and they all
        // agree on its offset, so the writes are deduplicated by vertex and each
        // one lands exactly once.
        touched.clear();
        for (const uint32_t qi : members) {
            if (std::fabs(offset[qi]) <= kUvEpsilon) continue;
            for (const uint32_t v : st.quads[qi].v) {
                touched.push_back(v);
                shift_for[v] = offset[qi];
            }
        }
        std::sort(touched.begin(), touched.end());
        touched.erase(std::unique(touched.begin(), touched.end()), touched.end());

        for (const uint32_t v : touched) {
            st.mesh->vertices[v].uv.x += shift_for[v];
            ++rewritten;
        }
    }

    return rewritten;
}

} // namespace

size_t merge_coplanar_quads(Mesh& mesh, const TessellationConfig& cfg) {
    if (!cfg.merge_coplanar_strips) return 0;

    const size_t index_count = mesh.indices.size();
    if (index_count == 0 || (index_count % 3) != 0) return 0;

    const uint32_t vertex_count = static_cast<uint32_t>(mesh.vertices.size());
    for (const uint32_t i : mesh.indices) {
        if (i >= vertex_count) return 0;
    }

    // The ranges must tile the index buffer in order, which is what
    // sort_submeshes_by_material() and append() both guarantee. Anything else is
    // not a shape this pass knows how to preserve, so it declines rather than
    // guesses.
    const std::vector<SubMesh> ranges = mesh.effective_submeshes();
    if (ranges.empty()) return 0;
    {
        uint32_t cursor = 0;
        for (const SubMesh& sm : ranges) {
            if (sm.index_offset != cursor) return 0;
            if ((sm.index_count % 3) != 0) return 0;
            cursor += sm.index_count;
        }
        if (cursor != index_count) return 0;
    }

    const size_t tri_count = index_count / 3;
    std::vector<uint32_t> tri_range(tri_count, 0);
    for (size_t r = 0; r < ranges.size(); ++r) {
        const size_t first = ranges[r].index_offset / 3;
        const size_t last = first + ranges[r].index_count / 3;
        for (size_t t = first; t < last; ++t) tri_range[t] = static_cast<uint32_t>(r);
    }

    const std::vector<uint32_t> pid = build_position_ids(mesh.vertices);

    // ------------------------------------------------------------------------
    // Pair triangles into quads
    // ------------------------------------------------------------------------
    std::unordered_map<uint64_t, uint32_t> edge_tri;
    edge_tri.reserve(tri_count * 3);
    for (size_t t = 0; t < tri_count; ++t) {
        for (int e = 0; e < 3; ++e) {
            const uint32_t a = mesh.indices[t * 3 + static_cast<size_t>(e)];
            const uint32_t b = mesh.indices[t * 3 + static_cast<size_t>((e + 1) % 3)];
            const uint64_t key = edge_key(a, b);
            const auto [it, inserted] = edge_tri.emplace(key, static_cast<uint32_t>(t));
            // A directed edge used twice is a non-manifold fan. Poison it rather
            // than pair arbitrarily.
            if (!inserted) it->second = kNoTriangle;
        }
    }

    MergeState st;
    st.mesh = &mesh;
    st.pid = &pid;
    st.normal_eps = std::min(std::max(cfg.coplanar_normal_epsilon, -1.0), 1.0);
    st.span_cap = (cfg.max_span_length > 0.0 && std::isfinite(cfg.max_span_length))
                      ? cfg.max_span_length
                      : std::numeric_limits<double>::infinity();

    std::vector<size_t> tri_quad(tri_count, kNoIndex);
    for (size_t t = 0; t < tri_count; ++t) {
        if (tri_quad[t] != kNoIndex) continue;
        const uint32_t t0 = mesh.indices[t * 3 + 0];
        const uint32_t t1 = mesh.indices[t * 3 + 1];
        const uint32_t t2 = mesh.indices[t * 3 + 2];
        const uint32_t tv[3] = {t0, t1, t2};

        for (int e = 0; e < 3; ++e) {
            const uint32_t a = tv[e];
            const uint32_t b = tv[(e + 1) % 3];
            const uint32_t opp_t = tv[(e + 2) % 3];

            const auto it = edge_tri.find(edge_key(b, a));
            if (it == edge_tri.end() || it->second == kNoTriangle) continue;
            const uint32_t u = it->second;
            if (u == t || tri_quad[u] != kNoIndex) continue;
            if (tri_range[u] != tri_range[t]) continue;

            uint32_t opp_u = kNoTriangle;
            for (int k = 0; k < 3; ++k) {
                const uint32_t cand = mesh.indices[static_cast<size_t>(u) * 3 + static_cast<size_t>(k)];
                if (cand != a && cand != b) { opp_u = cand; break; }
            }
            if (opp_u == kNoTriangle) continue;

            // Quad in counter-clockwise order. Triangle u carries the shared edge
            // as (b, a) and triangle t as (a, b), so the loop closes as
            // (a, opp_u, b, opp_t).
            const uint32_t loop[4] = {a, opp_u, b, opp_t};
            const glm::dvec3 pts[4] = {glm::dvec3(mesh.vertices[loop[0]].position),
                                       glm::dvec3(mesh.vertices[loop[1]].position),
                                       glm::dvec3(mesh.vertices[loop[2]].position),
                                       glm::dvec3(mesh.vertices[loop[3]].position)};
            // Formed only when it really is a convex planar quadrilateral: a merge
            // re-triangulates across the other diagonal, and doing that to a
            // warped or reflex pair would move the surface.
            if (!loop_is_convex_planar(pts, 4, st.normal_eps)) continue;

            QuadRec q;
            q.v[0] = loop[0]; q.v[1] = loop[1]; q.v[2] = loop[2]; q.v[3] = loop[3];
            q.source[0] = static_cast<uint32_t>(std::min<size_t>(t, u));
            q.source[1] = static_cast<uint32_t>(std::max<size_t>(t, u));
            q.range = tri_range[t];
            q.order = std::min<size_t>(t, u);
            refresh_quad_plane(q, mesh.vertices);
            tri_quad[t] = st.quads.size();
            tri_quad[u] = st.quads.size();
            st.quads.push_back(q);
            break;
        }
    }

    if (st.quads.empty()) return 0;

    st.side_map.reserve(st.quads.size() * 4);
    for (uint32_t qi = 0; qi < st.quads.size(); ++qi) st.register_sides(qi);

    // ------------------------------------------------------------------------
    // Which vertices a triangle outside every quad touches
    // ------------------------------------------------------------------------
    st.vertex_has_loose.assign(mesh.vertices.size(), 0);
    for (size_t t = 0; t < tri_count; ++t) {
        if (tri_quad[t] != kNoIndex) continue;
        for (int k = 0; k < 3; ++k) {
            st.vertex_has_loose[mesh.indices[t * 3 + static_cast<size_t>(k)]] = 1;
        }
    }

    // ------------------------------------------------------------------------
    // Merge, harmonise, merge again
    // ------------------------------------------------------------------------
    size_t merges = 0;
    const auto merge_to_fixpoint = [&]() {
        st.pending_valid = false;
        for (size_t sweep = 0; sweep < kMaxSweepsPerRound; ++sweep) {
            const size_t done = merge_sweep(st);
            merges += done;
            if (done == 0) { st.pending_valid = true; break; }
        }
    };

    // Everything already UV-continuous first, so the strip columns a lateral
    // merge needs to own are owned by as few quads as possible when the
    // harmonisation coverage rule is applied.
    merge_to_fixpoint();

    for (size_t round = 0; round < kMaxMergeRounds; ++round) {
        if (harmonise_u(st) == 0) break;
        const size_t before = merges;
        merge_to_fixpoint();
        if (merges == before) break;
    }

    if (merges == 0) return 0;

    // ------------------------------------------------------------------------
    // Rebuild the index buffer, one range at a time, in the original order
    // ------------------------------------------------------------------------
    struct Primitive {
        size_t order = 0;
        uint32_t id = 0;
        bool is_quad = false;
    };
    std::vector<std::vector<Primitive>> per_range(ranges.size());
    for (size_t t = 0; t < tri_count; ++t) {
        if (tri_quad[t] != kNoIndex) continue;
        per_range[tri_range[t]].push_back(Primitive{t, static_cast<uint32_t>(t), false});
    }
    for (uint32_t qi = 0; qi < st.quads.size(); ++qi) {
        if (!st.quads[qi].alive) continue;
        per_range[st.quads[qi].range].push_back(Primitive{st.quads[qi].order, qi, true});
    }

    std::vector<uint32_t> out_indices;
    out_indices.reserve(index_count);
    std::vector<SubMesh> out_submeshes;
    out_submeshes.reserve(ranges.size());

    for (size_t r = 0; r < ranges.size(); ++r) {
        auto& prims = per_range[r];
        std::sort(prims.begin(), prims.end(),
                  [](const Primitive& a, const Primitive& b) { return a.order < b.order; });

        const uint32_t range_start = static_cast<uint32_t>(out_indices.size());
        for (const Primitive& p : prims) {
            if (!p.is_quad) {
                out_indices.push_back(mesh.indices[static_cast<size_t>(p.id) * 3 + 0]);
                out_indices.push_back(mesh.indices[static_cast<size_t>(p.id) * 3 + 1]);
                out_indices.push_back(mesh.indices[static_cast<size_t>(p.id) * 3 + 2]);
                continue;
            }

            const QuadRec& q = st.quads[p.id];
            if (!q.merged) {
                // Untouched: re-emit the exact triangles it was paired from, so a
                // quad that only ever served as a candidate keeps its winding and
                // its diagonal.
                for (const uint32_t src : q.source) {
                    out_indices.push_back(mesh.indices[static_cast<size_t>(src) * 3 + 0]);
                    out_indices.push_back(mesh.indices[static_cast<size_t>(src) * 3 + 1]);
                    out_indices.push_back(mesh.indices[static_cast<size_t>(src) * 3 + 2]);
                }
                continue;
            }

            out_indices.push_back(q.v[0]);
            out_indices.push_back(q.v[1]);
            out_indices.push_back(q.v[2]);
            out_indices.push_back(q.v[0]);
            out_indices.push_back(q.v[2]);
            out_indices.push_back(q.v[3]);
        }

        const uint32_t added = static_cast<uint32_t>(out_indices.size()) - range_start;
        if (added == 0) continue;   // a range that lost every triangle is removed

        SubMesh sm = ranges[r];
        sm.index_offset = range_start;
        sm.index_count = added;
        out_submeshes.push_back(sm);
    }

    mesh.indices.swap(out_indices);

    // An implicit whole-mesh range stays implicit: materialising it here would
    // hand the caller a submesh list it never had.
    if (!mesh.submeshes.empty()) {
        mesh.submeshes.swap(out_submeshes);
    }

    // No vertex moved, so Mesh::bounds is still correct. UVs did move, so a
    // caller that wants tangents matching the new parameterisation must call
    // Mesh::compute_tangents() after this.
    // MERGES, not triangles. Every merge replaces two quads with one and so
    // removes exactly two triangles, whether it fired on its own or as a link in
    // a chain of them, so the halving is exact and never rounds.
    const size_t new_tri_count = mesh.indices.size() / 3;
    return (tri_count > new_tri_count) ? ((tri_count - new_tri_count) / 2) : 0;
}

} // namespace stratum::osm::road
