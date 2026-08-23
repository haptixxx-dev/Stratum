/**
 * @file centerline.cpp
 * @brief Implementation of centerline cleanup, smoothing, resampling, and the miter
 *
 * The pipeline is four stages, each a free function in the anonymous namespace:
 *
 *   1. weld_polyline()   drop non-finite and coincident points.
 *   2. build_dense_curve()  fit a chordal Catmull-Rom through the welded points,
 *      split at corners, and flatten it to a dense polyline. Skipped when
 *      smoothing is off, in which case the welded polyline IS the dense
 *      polyline.
 *   3. choose_station_arclengths()  pick station positions along the dense
 *      polyline at curvature-adaptive spacing.
 *   4. build_frames() + compute_curvature()  turn positions into sweep frames,
 *      inserting a bevel pair wherever the miter would exceed the limit.
 *
 * ### Sign conventions, fixed here and relied on downstream
 *
 * The 2D-to-3D mapping used everywhere in this codebase is
 * `(x, y_2d) -> vec3(x, height, -y_2d)`, Y up. Take a unit tangent (1, 0), which
 * maps to world +X. With Y up, the left of travel is `cross(up, forward)` =
 * `cross((0,1,0), (1,0,0))` = `(0, 0, -1)`. The 2D vector that maps to world
 * `(0, 0, -1)` is `(0, 1)`. So `(-t.y, t.x)` is the LEFT normal in world space,
 * and that is what left_normal() returns. Note that the y flip in the mapping
 * means this is a counter-clockwise quarter turn in the 2D plane but reads as a
 * left turn in the world; both statements are true and neither can be dropped.
 *
 * Signed curvature follows the same handedness: `cross2(in, out) > 0` is a
 * counter-clockwise turn in 2D, which is a turn towards +normal, that is, a turn
 * to the LEFT of travel. Positive curvature therefore means the centre of the
 * osculating circle lies on the +normal side.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API.
 */

#include "osm/road/centerline.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace stratum::osm::road {

namespace {

// ============================================================================
// Numeric Guards
// ============================================================================

/// Below this a length is treated as zero and every normalize() falls back
constexpr double kLenEpsilon = 1e-12;

/// Arclengths closer than this are the same station
constexpr double kArcEpsilon = 1e-9;

/// Hard cap on flattened curve points, so a pathological input cannot run away
constexpr size_t kMaxDensePoints = 65536;

/// Hard cap on stations, so a degenerate spacing cannot run away
constexpr size_t kMaxStations = 262144;

/// Forced subdivisions per spline segment before the deviation test is trusted
constexpr int kMinFlattenDepth = 2;

/// Recursion cap for the flattener: at most 2^kMaxFlattenDepth pieces per segment
constexpr int kMaxFlattenDepth = 10;

/// True when both components of @p p are finite
[[nodiscard]] inline bool is_finite(const glm::dvec2& p) {
    return std::isfinite(p.x) && std::isfinite(p.y);
}

/// 2D cross product, positive when @p b is counter-clockwise from @p a
[[nodiscard]] inline double cross2(const glm::dvec2& a, const glm::dvec2& b) {
    return a.x * b.y - a.y * b.x;
}

/// Euclidean length, never NaN for finite input
[[nodiscard]] inline double length2d(const glm::dvec2& v) {
    return std::sqrt(v.x * v.x + v.y * v.y);
}

/**
 * @brief Normalize @p v, or return @p fallback when it has no usable direction
 *
 * Every normalize in this file goes through here. A zero-length tangent is the
 * single most common way a polyline pipeline produces NaN geometry.
 */
[[nodiscard]] inline glm::dvec2 normalize_or(const glm::dvec2& v, const glm::dvec2& fallback) {
    const double len = length2d(v);
    if (!(len > kLenEpsilon) || !std::isfinite(len)) return fallback;
    return v / len;
}

/**
 * @brief Left normal of a unit tangent
 *
 * See the file header for the derivation against the (x, y) -> (x, h, -y) world
 * mapping. This is the ONLY place the quarter turn is written down.
 */
[[nodiscard]] inline glm::dvec2 left_normal(const glm::dvec2& t) {
    return glm::dvec2(-t.y, t.x);
}

// ============================================================================
// Stage 1: Cleanup
// ============================================================================

/**
 * @brief Drop non-finite points and weld consecutive points closer than @p eps
 *
 * Exact duplicates are dropped whatever @p eps is, so a caller that passes a
 * zero or negative weld epsilon still cannot produce a zero-length segment.
 *
 * @param in  Raw polyline
 * @param eps Weld distance in metres
 * @return The surviving points, in order
 */
[[nodiscard]] std::vector<glm::dvec2> weld_polyline(const std::vector<glm::dvec2>& in, double eps) {
    std::vector<glm::dvec2> out;
    out.reserve(in.size());

    const double eps_sq = (eps > 0.0 && std::isfinite(eps)) ? (eps * eps) : 0.0;

    for (const glm::dvec2& p : in) {
        if (!is_finite(p)) continue;
        if (!out.empty()) {
            const glm::dvec2 d = p - out.back();
            if (d.x == 0.0 && d.y == 0.0) continue;
            if ((d.x * d.x + d.y * d.y) <= eps_sq) continue;
        }
        out.push_back(p);
    }
    return out;
}

// ============================================================================
// Stage 2: Chordal Catmull-Rom
// ============================================================================

/**
 * @brief Turn sharper than this is a real corner, not a bend to be smoothed
 *
 * cos(60 degrees). A Catmull-Rom fitted straight through a hard corner tilts the
 * tangent at the corner towards the bisector and then has to bow the neighbouring
 * spans the WRONG way to reach it, so a right-angle turn in a survey polyline
 * comes back as an S-bend metres away from the surveyed line. A corner is
 * survey data, so the fit is split there instead and the miter handles it,
 * which is exactly what the miter is for.
 */
constexpr double kCornerTurnCos = 0.5;

/**
 * @brief Perpendicular distance below which a survey vertex carries no geometry
 *
 * A vertex whose offset from the chord between its two neighbours is under this
 * is collinear with them: keeping it would place a station that reproduces a
 * point already on the line. Dropping it is what makes the unsmoothed path a
 * resampler rather than a passthrough, so a straight surveyed with fifty vertices
 * costs two stations and a curve surveyed with fifty keeps all fifty. The bound
 * is a hard geometric one, not a tolerance to be tuned: at 1e-6 m the dropped
 * vertex is below any scale the corridor can express.
 */
constexpr double kCollinearDeviation = 1e-6;

/**
 * @brief A chordal-parameterised Catmull-Rom curve, stored segment by segment in Bezier form
 *
 * Chordal, not uniform: knot spacing is the chord length between consecutive
 * control points. Uniform parameterisation overshoots badly on the uneven point
 * spacing OSM produces, because a long span and a short span get the same
 * parameter interval and the curve has to accelerate through the short one.
 *
 * The curve INTERPOLATES its control points by construction: the first and last
 * Bezier control point of every segment IS a survey point. OSM vertices are
 * never smoothed away, only joined.
 */
struct Spline {
    std::vector<glm::dvec2> p;    ///< control points, one per survey point
    std::vector<glm::dvec2> c0;   ///< per segment: first inner Bezier control point
    std::vector<glm::dvec2> c1;   ///< per segment: second inner Bezier control point
};

/**
 * @brief Fit a chordal Catmull-Rom through @p pts
 *
 * Endpoints get duplicated control points, so the phantom control point outside
 * each end sits on the terminal point. The chord to a duplicated point is zero,
 * which would make the chordal knot spacing degenerate, so the phantom knot
 * spacing is mirrored from the adjacent real span instead. That keeps the
 * tangent formula well conditioned while leaving the phantom POSITION duplicated
 * as specified. The resulting end tangent points exactly along the terminal
 * chord, which is what lets an edge's end frame match its neighbour's.
 *
 * Because the parameterisation is chordal, each one-sided difference quotient is
 * a UNIT vector, so every tangent is a convex combination of unit vectors and
 * has magnitude at most 1. The inner Bezier control points therefore never sit
 * further than chord/3 from their endpoint.
 *
 * That bound is still too loose where a long span meets a short one: the chordal
 * weights put almost all of the weight on the SHORT side, so the tangent at the
 * shared vertex is dominated by the short chord's direction and is then applied
 * across the whole long span. A 100 m straight running into a 1 m stub bows tens
 * of metres off the surveyed line. Each control point is therefore additionally
 * limited to a third of the SHORTER of its own segment and the neighbouring one.
 * On evenly spaced input the limit equals the natural bound and never binds.
 *
 * Neither bound says anything about how far the curve strays from the surveyed
 * chord, and on evenly spaced input neither one binds at all: a handle a full
 * chord/3 along the corner bisector bows the segment roughly
 * 0.15 * chord * sin(turn / 2) off the line the surveyor drew, which grows
 * without limit as the segments get longer. @p max_offset closes that. A cubic
 * Bezier never leaves 3/4 of the larger of its two handles' perpendicular
 * offsets from its own chord, so shrinking the PERPENDICULAR component of each
 * handle to 4/3 of @p max_offset bounds the whole segment inside a band that
 * wide. Only the perpendicular component is touched, so the handle directions --
 * and with them G1 continuity across the joint -- are untouched; the segment
 * just flattens towards its chord.
 *
 * @param pts        At least two welded, distinct points
 * @param max_offset Metres the curve may depart from the surveyed chords
 * @return The fitted curve
 */
[[nodiscard]] Spline build_spline(const std::vector<glm::dvec2>& pts, double max_offset) {
    Spline s;
    const size_t n = pts.size();
    s.p = pts;
    if (n < 2) return s;

    std::vector<double> chord(n - 1, 0.0);
    for (size_t i = 0; i + 1 < n; ++i) {
        chord[i] = std::max(length2d(pts[i + 1] - pts[i]), kLenEpsilon);
    }

    // Chordal knots are the running chord lengths, so dt over a span is that
    // span's chord length.
    std::vector<glm::dvec2> m(n, glm::dvec2(0.0));
    for (size_t i = 0; i < n; ++i) {
        const glm::dvec2 p_prev = (i == 0) ? pts[0] : pts[i - 1];
        const glm::dvec2 p_next = (i + 1 == n) ? pts[n - 1] : pts[i + 1];

        const double dt_prev = (i == 0) ? chord[0] : chord[i - 1];
        const double dt_next = (i + 1 == n) ? chord[n - 2] : chord[i];
        const double denom = dt_prev + dt_next;
        if (!(denom > 0.0)) continue;

        // Standard non-uniform Catmull-Rom tangent: a knot-weighted blend of the
        // two one-sided difference quotients. Reduces to (P[i+1]-P[i-1]) / 2h on
        // uniform knots.
        const glm::dvec2 back = (pts[i] - p_prev) / dt_prev;
        const glm::dvec2 fwd  = (p_next - pts[i]) / dt_next;
        m[i] = back * (dt_next / denom) + fwd * (dt_prev / denom);
    }

    // 3/4 is the cubic Bezier's convex-hull bound: the curve never departs its
    // chord by more than three quarters of the larger handle offset.
    const double perp_cap = (std::isfinite(max_offset) && max_offset > 0.0)
                                ? (max_offset * 4.0 / 3.0)
                                : 0.0;

    s.c0.resize(n - 1);
    s.c1.resize(n - 1);
    for (size_t i = 0; i + 1 < n; ++i) {
        const double c = chord[i];
        const double prev_c = (i > 0) ? chord[i - 1] : c;
        const double next_c = (i + 2 < n) ? chord[i + 1] : c;

        const double cap0 = std::min(c, prev_c) / 3.0;
        const double cap1 = std::min(c, next_c) / 3.0;

        glm::dvec2 d0 = m[i] * (c / 3.0);
        glm::dvec2 d1 = m[i + 1] * (c / 3.0);

        const double l0 = length2d(d0);
        const double l1 = length2d(d1);
        if (l0 > cap0 && l0 > kLenEpsilon) d0 *= (cap0 / l0);
        if (l1 > cap1 && l1 > kLenEpsilon) d1 *= (cap1 / l1);

        // Bound the departure from the surveyed chord. A handle is shrunk only
        // when its own perpendicular offset is what would carry the curve out of
        // the band, and shrinking it uniformly keeps its direction.
        const glm::dvec2 chord_dir = normalize_or(pts[i + 1] - pts[i], glm::dvec2(1.0, 0.0));
        const double perp0 = std::abs(cross2(chord_dir, d0));
        const double perp1 = std::abs(cross2(chord_dir, d1));
        if (perp0 > perp_cap) d0 *= (perp_cap / perp0);
        if (perp1 > perp_cap) d1 *= (perp_cap / perp1);

        s.c0[i] = pts[i] + d0;
        s.c1[i] = pts[i + 1] - d1;
    }

    return s;
}

/**
 * @brief Evaluate segment @p seg of @p s at local parameter @p u in [0, 1]
 *
 * @param s   Fitted curve
 * @param seg Segment index, the span from control point seg to seg + 1
 * @param u   Local parameter
 * @return The curve point
 */
[[nodiscard]] glm::dvec2 spline_eval(const Spline& s, size_t seg, double u) {
    const double v = 1.0 - u;
    const double b0 = v * v * v;
    const double b1 = 3.0 * v * v * u;
    const double b2 = 3.0 * v * u * u;
    const double b3 = u * u * u;
    return s.p[seg] * b0 + s.c0[seg] * b1 + s.c1[seg] * b2 + s.p[seg + 1] * b3;
}

/// State carried through the flattening recursion
struct FlattenContext {
    const Spline* spline = nullptr;
    std::vector<glm::dvec2>* out = nullptr;
    double tolerance = 1e-3;
};

/**
 * @brief Recursively subdivide one spline segment until it is flat within tolerance
 *
 * The midpoint deviation test alone can stop early on an S-shaped span whose
 * midpoint happens to sit on the chord, so kMinFlattenDepth subdivisions are
 * forced before the test is trusted.
 *
 * @param ctx   Curve, output buffer, and tolerance
 * @param seg   Segment index
 * @param u0    Start parameter
 * @param u1    End parameter
 * @param p0    Curve point at @p u0
 * @param p1    Curve point at @p u1
 * @param depth Current recursion depth
 */
void flatten_recurse(const FlattenContext& ctx, size_t seg, double u0, double u1,
                     const glm::dvec2& p0, const glm::dvec2& p1, int depth) {
    if (depth >= kMaxFlattenDepth || ctx.out->size() >= kMaxDensePoints) {
        ctx.out->push_back(p1);
        return;
    }

    const double um = 0.5 * (u0 + u1);
    const glm::dvec2 pm = spline_eval(*ctx.spline, seg, um);

    if (depth >= kMinFlattenDepth) {
        const glm::dvec2 chord = p1 - p0;
        const double chord_len = length2d(chord);
        const double deviation = (chord_len > kLenEpsilon)
                                     ? std::abs(cross2(chord, pm - p0)) / chord_len
                                     : length2d(pm - p0);
        if (deviation <= ctx.tolerance) {
            ctx.out->push_back(p1);
            return;
        }
    }

    flatten_recurse(ctx, seg, u0, um, p0, pm, depth + 1);
    flatten_recurse(ctx, seg, um, u1, pm, p1, depth + 1);
}

/**
 * @brief A polyline fine enough to resample from, plus the points that must be stations
 */
struct DenseCurve {
    std::vector<glm::dvec2> points;

    /// Indices into points that a station must land on exactly: the ends and every corner
    std::vector<size_t> breaks;
};

/**
 * @brief Flatten one run of survey points into @p out
 *
 * @param run        Survey points of the run, at least two
 * @param tolerance  Maximum chord-to-curve deviation in metres
 * @param max_offset Metres the fitted curve may depart from the surveyed chords
 * @param out        Destination, appended to; the run's first point is assumed present
 */
void flatten_run(const std::vector<glm::dvec2>& run, double tolerance, double max_offset,
                 std::vector<glm::dvec2>& out) {
    if (run.size() < 2) return;

    const Spline s = build_spline(run, max_offset);

    FlattenContext ctx;
    ctx.spline = &s;
    ctx.out = &out;
    ctx.tolerance = tolerance;

    for (size_t seg = 0; seg + 1 < run.size(); ++seg) {
        flatten_recurse(ctx, seg, 0.0, 1.0, s.p[seg], s.p[seg + 1], 0);
    }
}

/**
 * @brief Build the dense polyline the stations are resampled from
 *
 * With smoothing off the welded polyline IS the geometry, so it is returned
 * unchanged and every survey vertex that is not collinear with its neighbours is
 * a break.
 *
 * With smoothing on the polyline is split at every corner sharper than
 * kCornerTurnCos and each run between corners is fitted with its own chordal
 * Catmull-Rom, then flattened. Corners survive as breaks, so a station lands on
 * each one and the joint is mitred rather than rounded away.
 *
 * @param welded Cleaned survey points, at least two
 * @param cfg    Tolerances
 * @return The dense polyline and its mandatory station indices
 */
[[nodiscard]] DenseCurve build_dense_curve(const std::vector<glm::dvec2>& welded,
                                           const ResampleConfig& cfg) {
    DenseCurve dc;
    if (welded.size() < 2) {
        dc.points = welded;
        for (size_t i = 0; i < welded.size(); ++i) dc.breaks.push_back(i);
        return dc;
    }

    if (!cfg.smooth) {
        // The welded polyline IS the geometry, so every point of it is kept and a
        // station may land anywhere along it. Only the vertices that carry shape
        // are mandatory: the two ends, and every interior vertex that is not
        // collinear with its neighbours. A collinear vertex is dropped from the
        // stop list, so a long straight is spanned at max_spacing instead of
        // inheriting the survey's vertex density, while every bend still gets a
        // station on the surveyed point itself.
        dc.points = welded;
        dc.breaks.reserve(welded.size());
        dc.breaks.push_back(0);
        for (size_t i = 1; i + 1 < welded.size(); ++i) {
            const glm::dvec2 chord = welded[i + 1] - welded[i - 1];
            const double chord_len = length2d(chord);
            const double deviation = (chord_len > kLenEpsilon)
                                         ? std::abs(cross2(chord, welded[i] - welded[i - 1])) /
                                               chord_len
                                         : length2d(welded[i] - welded[i - 1]);
            if (deviation > kCollinearDeviation) dc.breaks.push_back(i);
        }
        dc.breaks.push_back(welded.size() - 1);
        return dc;
    }

    const double tolerance =
        std::max(1e-4, (std::isfinite(cfg.max_deviation) ? cfg.max_deviation : 0.05) * 0.05);

    // How far the invented curve may sit from the surveyed chords. A zero or
    // negative setting means "do not bow at all", which reduces the fit to the
    // welded polyline itself rather than being ignored.
    const double max_offset =
        (std::isfinite(cfg.max_smoothing_offset) && cfg.max_smoothing_offset > 0.0)
            ? cfg.max_smoothing_offset
            : 0.0;

    dc.points.reserve(welded.size() * 8);
    dc.points.push_back(welded.front());
    dc.breaks.push_back(0);

    std::vector<glm::dvec2> run;
    run.push_back(welded.front());

    for (size_t i = 1; i + 1 < welded.size(); ++i) {
        const glm::dvec2 t_in = normalize_or(welded[i] - welded[i - 1], glm::dvec2(1.0, 0.0));
        const glm::dvec2 t_out = normalize_or(welded[i + 1] - welded[i], t_in);
        run.push_back(welded[i]);

        if (glm::dot(t_in, t_out) < kCornerTurnCos) {
            flatten_run(run, tolerance, max_offset, dc.points);
            dc.breaks.push_back(dc.points.size() - 1);
            run.clear();
            run.push_back(welded[i]);
        }
    }

    run.push_back(welded.back());
    flatten_run(run, tolerance, max_offset, dc.points);
    dc.breaks.push_back(dc.points.size() - 1);

    return dc;
}

// ============================================================================
// Stage 3: Arc Length and Adaptive Spacing
// ============================================================================

/**
 * @brief Cumulative arc length along a polyline, one entry per point
 *
 * @param pts Polyline
 * @return Arclengths, starting at 0 and non-decreasing
 */
[[nodiscard]] std::vector<double> cumulative_arclength(const std::vector<glm::dvec2>& pts) {
    std::vector<double> s(pts.size(), 0.0);
    for (size_t i = 1; i < pts.size(); ++i) {
        s[i] = s[i - 1] + length2d(pts[i] - pts[i - 1]);
    }
    return s;
}

/**
 * @brief Signed Menger curvature at each point of a dense polyline, in 1/metres
 *
 * kappa = 2 * cross(a, b) / (|a| |b| |a + b|) for the circle through three
 * consecutive points. Positive is a turn to the LEFT of travel; see the file
 * header. End points copy their neighbour, which is a one-sided estimate rather
 * than a claim that the curve straightens at the ends.
 *
 * @param pts Dense polyline
 * @return Per-point signed curvature
 */
[[nodiscard]] std::vector<double> dense_curvature(const std::vector<glm::dvec2>& pts) {
    const size_t n = pts.size();
    std::vector<double> k(n, 0.0);
    if (n < 3) return k;

    for (size_t i = 1; i + 1 < n; ++i) {
        const glm::dvec2 a = pts[i] - pts[i - 1];
        const glm::dvec2 b = pts[i + 1] - pts[i];
        const glm::dvec2 c = pts[i + 1] - pts[i - 1];
        const double denom = length2d(a) * length2d(b) * length2d(c);
        if (!(denom > kLenEpsilon)) continue;
        const double value = 2.0 * cross2(a, b) / denom;
        if (std::isfinite(value)) k[i] = value;
    }
    k[0] = k[1];
    k[n - 1] = k[n - 2];
    return k;
}

/**
 * @brief Largest absolute curvature over an arclength window
 *
 * @param arc Cumulative arclengths of the dense polyline
 * @param k   Per-point curvature of the dense polyline
 * @param s0  Window start in metres
 * @param s1  Window end in metres
 * @return The maximum of |k| over the window, 0 when the window is empty
 */
[[nodiscard]] double max_abs_curvature(const std::vector<double>& arc,
                                       const std::vector<double>& k,
                                       double s0, double s1) {
    if (arc.empty()) return 0.0;

    // One point either side of the window, so a curvature peak between samples
    // is not missed.
    auto lo_it = std::lower_bound(arc.begin(), arc.end(), s0);
    size_t lo = static_cast<size_t>(lo_it - arc.begin());
    if (lo > 0) --lo;

    auto hi_it = std::upper_bound(arc.begin(), arc.end(), s1);
    size_t hi = static_cast<size_t>(hi_it - arc.begin());
    if (hi < arc.size()) ++hi;
    hi = std::min(hi, arc.size());

    double best = 0.0;
    for (size_t i = lo; i < hi; ++i) best = std::max(best, std::abs(k[i]));
    return best;
}

/**
 * @brief Station spacing admitted by a local curvature
 *
 * For a circle of radius r, a chord of length c has a mid-chord sagitta
 * d = r - sqrt(r^2 - c^2/4), so the chord admitted by a deviation budget d is
 * c = 2 * sqrt(2rd - d^2). A straight has no curvature and takes the whole
 * max_spacing; a tight corner gets dense stations.
 *
 * @param curvature Local |curvature| in 1/metres
 * @param cfg       Tolerances
 * @return Spacing in metres, clamped to [min_spacing, max_spacing]
 */
[[nodiscard]] double spacing_for_curvature(double curvature, const ResampleConfig& cfg) {
    double max_spacing = (cfg.max_spacing > 0.0 && std::isfinite(cfg.max_spacing)) ? cfg.max_spacing : 8.0;
    double min_spacing = (cfg.min_spacing > 0.0 && std::isfinite(cfg.min_spacing)) ? cfg.min_spacing : 0.5;
    min_spacing = std::max(min_spacing, 1e-3);
    min_spacing = std::min(min_spacing, max_spacing);

    if (!(curvature > kLenEpsilon) || !std::isfinite(curvature)) return max_spacing;

    const double radius = 1.0 / curvature;
    double deviation = cfg.max_deviation;
    if (!(deviation > 0.0) || !std::isfinite(deviation)) return min_spacing;

    // Keep 2rd - d^2 strictly positive: the formula is only meaningful while the
    // sagitta is below the radius.
    deviation = std::min(deviation, radius);

    const double chord = 2.0 * std::sqrt(std::max(0.0, 2.0 * radius * deviation - deviation * deviation));
    if (!std::isfinite(chord)) return min_spacing;
    return std::clamp(chord, min_spacing, max_spacing);
}

/**
 * @brief Spacing to use starting at arclength @p s
 *
 * The window matters as much as the formula. Reading the curvature only AT the
 * station lets a step start on the straight approach to a corner and jump the
 * corner whole, so the window starts a full max_spacing ahead and is then
 * tightened onto the step actually chosen.
 *
 * The tightening only ever SHRINKS the step. Letting it grow again would undo
 * the lookahead exactly where the lookahead was needed: the shrunk window no
 * longer covers the corner that caused the shrink, reports no curvature, and
 * hands back the full max_spacing.
 *
 * @param arc Cumulative arclengths of the dense polyline
 * @param k   Per-point curvature of the dense polyline
 * @param s   Start arclength in metres
 * @param cfg Tolerances
 * @return Spacing in metres
 */
[[nodiscard]] double adaptive_step(const std::vector<double>& arc,
                                   const std::vector<double>& k,
                                   double s, const ResampleConfig& cfg) {
    double step = (cfg.max_spacing > 0.0 && std::isfinite(cfg.max_spacing)) ? cfg.max_spacing : 8.0;

    for (int i = 0; i < 4; ++i) {
        const double tightened = spacing_for_curvature(max_abs_curvature(arc, k, s, s + step), cfg);
        if (tightened >= step) break;
        step = tightened;
    }
    return step;
}

/**
 * @brief Choose the arclength of every station
 *
 * Each span between consecutive mandatory stops is filled independently, so a
 * stop is always hit exactly. The tail of a span is split evenly rather than
 * leaving a runt band shorter than half a step, which would otherwise produce a
 * sliver of geometry at every stop.
 *
 * @param arc   Cumulative arclengths of the dense polyline
 * @param k     Per-point curvature of the dense polyline
 * @param stops Mandatory arclengths, sorted, starting at 0 and ending at the total length
 * @param cfg   Tolerances
 * @return Station arclengths, strictly increasing
 */
[[nodiscard]] std::vector<double> choose_station_arclengths(const std::vector<double>& arc,
                                                            const std::vector<double>& k,
                                                            const std::vector<double>& stops,
                                                            const ResampleConfig& cfg) {
    std::vector<double> out;
    if (stops.size() < 2) return out;

    out.push_back(stops.front());

    for (size_t si = 0; si + 1 < stops.size(); ++si) {
        const double span_end = stops[si + 1];
        double cur = out.back();

        while (out.size() < kMaxStations) {
            const double remaining = span_end - cur;
            if (remaining <= kArcEpsilon) break;

            const double step = adaptive_step(arc, k, cur, cfg);

            if (remaining <= step * 1.5) {
                // Finish the span here: either one band, or two even bands when a
                // single one would leave the next band shorter than half a step.
                if (remaining > step) out.push_back(cur + remaining * 0.5);
                out.push_back(span_end);
                break;
            }

            cur += step;
            out.push_back(cur);
        }

        if (out.back() < span_end - kArcEpsilon) out.push_back(span_end);
    }

    // Defensive: the walk above is monotone by construction, but a hostile config
    // must not be able to emit a non-increasing sequence.
    std::vector<double> cleaned;
    cleaned.reserve(out.size());
    for (const double s : out) {
        if (cleaned.empty() || s > cleaned.back() + kArcEpsilon) cleaned.push_back(s);
    }
    return cleaned;
}

/**
 * @brief Point at arclength @p s along a dense polyline
 *
 * @param pts Dense polyline
 * @param arc Cumulative arclengths, parallel to @p pts
 * @param s   Arclength in metres, clamped to the polyline
 * @return The interpolated point
 */
[[nodiscard]] glm::dvec2 sample_at_arclength(const std::vector<glm::dvec2>& pts,
                                             const std::vector<double>& arc,
                                             double s) {
    if (pts.empty()) return glm::dvec2(0.0);
    if (pts.size() == 1 || s <= arc.front()) return pts.front();
    if (s >= arc.back()) return pts.back();

    const auto it = std::upper_bound(arc.begin(), arc.end(), s);
    size_t hi = static_cast<size_t>(it - arc.begin());
    hi = std::clamp<size_t>(hi, 1, pts.size() - 1);
    const size_t lo = hi - 1;

    const double span = arc[hi] - arc[lo];
    const double u = (span > kLenEpsilon) ? (s - arc[lo]) / span : 0.0;
    return pts[lo] + (pts[hi] - pts[lo]) * u;
}

// ============================================================================
// Stage 4: Frames and the Miter
// ============================================================================

/**
 * @brief Turn station positions into sweep frames, mitring or bevelling each joint
 *
 * At an interior station the incoming and outgoing left normals n_in and n_out
 * define the joint:
 *
 * @code
 *     bisector    = normalize(n_in + n_out)
 *     cos_half    = dot(bisector, n_in)      // cos(theta/2)
 *     miter_scale = 1 / cos_half
 * @endcode
 *
 * offset_point() then traces a true parallel offset, so the OUTER edge of a
 * corner widens instead of pinching inward. Averaging the normals without the
 * 1/cos(theta/2) factor is exactly the defect this replaces.
 *
 * Past ResampleConfig::miter_limit the joint is bevelled: two stations at one
 * position and one arclength, the first carrying n_in and the second n_out, both
 * with miter_scale 1. A near-180-degree hairpin makes n_in + n_out vanish, which
 * is detected BEFORE the normalize and routed down the same bevel path, since a
 * hairpin has no finite miter.
 *
 * The frames alone do not stop the ribbon folding on the inside of a sharp turn;
 * bound_lateral_folds() runs over the result to do that.
 *
 * @param pos Station positions in 2D local metres
 * @param arc Station arclengths in metres, parallel to @p pos
 * @param cfg Tolerances, for miter_limit
 * @return The stations, with bevel pairs inserted
 */
[[nodiscard]] std::vector<Station> build_frames(const std::vector<glm::dvec2>& pos,
                                                const std::vector<double>& arc,
                                                const ResampleConfig& cfg) {
    std::vector<Station> out;
    const size_t n = pos.size();
    if (n < 2) return out;

    const double miter_limit = (cfg.miter_limit >= 1.0 && std::isfinite(cfg.miter_limit))
                                   ? cfg.miter_limit
                                   : 1.0;

    // Unit direction of every band. A band whose endpoints coincide inherits the
    // previous direction rather than producing a NaN frame.
    std::vector<glm::dvec2> dir(n - 1, glm::dvec2(1.0, 0.0));
    for (size_t i = 0; i + 1 < n; ++i) {
        const glm::dvec2 fallback = (i > 0) ? dir[i - 1] : glm::dvec2(1.0, 0.0);
        dir[i] = normalize_or(pos[i + 1] - pos[i], fallback);
    }

    out.reserve(n + 4);

    for (size_t i = 0; i < n; ++i) {
        if (i == 0 || i + 1 == n) {
            // One segment only, so there is no joint: bisector is that segment's
            // normal and the scale is exactly 1.
            const glm::dvec2 t = (i == 0) ? dir.front() : dir.back();
            Station s;
            s.position = pos[i];
            s.tangent = t;
            s.normal = left_normal(t);
            s.arclength = arc[i];
            s.miter_scale = 1.0;
            s.is_bevel = false;
            out.push_back(s);
            continue;
        }

        const glm::dvec2 t_in = dir[i - 1];
        const glm::dvec2 t_out = dir[i];
        const glm::dvec2 n_in = left_normal(t_in);
        const glm::dvec2 n_out = left_normal(t_out);

        const glm::dvec2 sum = n_in + n_out;
        const double sum_len = length2d(sum);

        bool bevel = false;
        glm::dvec2 bisector = n_in;
        double miter_scale = 1.0;

        if (!(sum_len > kLenEpsilon)) {
            // Hairpin: n_in and n_out are opposed, there is no bisector to
            // normalize and no finite miter. Bevel it.
            bevel = true;
        } else {
            bisector = sum / sum_len;
            const double cos_half = glm::dot(bisector, n_in);
            if (!(cos_half > kLenEpsilon) || !std::isfinite(cos_half)) {
                bevel = true;
            } else {
                miter_scale = 1.0 / cos_half;
                if (!std::isfinite(miter_scale) || miter_scale > miter_limit) bevel = true;
            }
        }

        if (bevel) {
            Station a;
            a.position = pos[i];
            a.tangent = t_in;
            a.normal = n_in;
            a.arclength = arc[i];
            a.miter_scale = 1.0;
            a.is_bevel = true;

            Station b = a;
            b.tangent = t_out;
            b.normal = n_out;

            out.push_back(a);
            out.push_back(b);
            continue;
        }

        Station s;
        s.position = pos[i];
        s.tangent = normalize_or(t_in + t_out, t_in);
        s.normal = bisector;
        s.arclength = arc[i];
        s.miter_scale = miter_scale;
        s.is_bevel = false;
        out.push_back(s);
    }

    return out;
}

/**
 * @brief Fill Station::lateral_min and Station::lateral_max, the fold bound
 *
 * A correct miter is not a sufficient one. The offset column at a joint retreats
 * along both of its legs on the inside of the turn, so past some lateral distance
 * it lands BEHIND its neighbour, the offset edge reverses, and the ribbon folds
 * through itself: inverted, backfacing triangles and a corridor outline that
 * overlaps itself. This is the plan's "self-intersection guard" for P2.
 *
 * The bound is derived from the geometry the extruder actually emits rather than
 * from the turn angle, so it is exact for every joint kind including a bevel.
 * For a band between two stations, with `ua = normal_a * miter_scale_a`,
 * `ub = normal_b * miter_scale_b` and `D` the chord between the two positions,
 * the two triangles of the band's quad at laterals A > B are positively oriented
 * exactly when
 *
 * @code
 *     cross(ua, D) + B * cross(ua, ub) < 0
 *     cross(ub, D) + A * cross(ua, ub) < 0
 * @endcode
 *
 * Both are linear in the lateral, so with `c = cross(ua, ub)` they are one bound:
 * an upper bound on the lateral when c is positive -- the turn is to the left and
 * the left of the profile is the inside -- and a lower bound when it is negative.
 * The tighter of the two is applied to BOTH ends of the band, because the first
 * condition constrains the far station's lateral against the near station's frame
 * and the second does the reverse; bounding only the joint leaves the straight
 * beside it free to overshoot and the sliver between them still inverts.
 *
 * A bevel pair falls out of the same algebra with no special case: its two
 * stations share a position, D is zero, and the bound collapses to 0 on the
 * inside of the turn. That is the fold that made a bevelled hairpin emit a
 * correct wedge on the outside of the turn and an inverted one on the inside.
 *
 * @param st Stations to bound, in order; unchanged where nothing folds
 */
void bound_lateral_folds(std::vector<Station>& st) {
    // The conditions are strict, so a lateral sitting exactly ON the limit gives a
    // band of zero extent, which the extruder drops as degenerate. Backing off by
    // a part in 1e12 keeps rounding on the safe side of that while leaving the
    // collapsed band far below the extruder's degeneracy threshold; a coarser
    // margin turns the collapse into a sliver with real area and arbitrary
    // winding instead of a triangle that is dropped.
    constexpr double kFoldSafety = 1.0 - 1e-12;

    for (size_t i = 0; i + 1 < st.size(); ++i) {
        Station& a = st[i];
        Station& b = st[i + 1];

        const glm::dvec2 ua = a.normal * a.miter_scale;
        const glm::dvec2 ub = b.normal * b.miter_scale;
        const glm::dvec2 d = b.position - a.position;

        const double c = cross2(ua, ub);
        if (!std::isfinite(c) || std::abs(c) <= kLenEpsilon) {
            continue;   // the two frames are parallel: no lateral can fold this band
        }

        const double lim_a = -cross2(ua, d) / c;
        const double lim_b = -cross2(ub, d) / c;
        if (!std::isfinite(lim_a) || !std::isfinite(lim_b)) continue;

        // A bound is clamped at the centreline first: collapsing the inside of the
        // turn onto the way is the worst a lateral clamp can usefully do, and a
        // negative upper bound would push the left of the profile out to the
        // RIGHT of the way instead of collapsing it. With the sign settled, the
        // safety factor always moves the bound towards zero, which is tighter.
        if (c > 0.0) {
            // Turning left, so the left of the profile is the inside.
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

/**
 * @brief Fill Station::curvature from the turn angle over the arc length at each station
 *
 * curvature = signed_turn_angle / mean_adjacent_band_length, positive for a turn
 * to the LEFT of travel, matching the left-normal convention.
 *
 * A bevel pair shares one position, so one of its two bands has zero length. The
 * station's own tangent stands in for the missing direction there, which makes
 * both halves of the pair report zero curvature. That is deliberate: a bevelled
 * corner is a curvature discontinuity, not a finite curvature, and reporting a
 * huge value would poison the vertical curvature limiter in P3.
 *
 * End stations copy their neighbour, a one-sided estimate.
 *
 * @param st Stations to annotate, in order
 */
void compute_curvature(std::vector<Station>& st) {
    const size_t n = st.size();
    if (n < 3) {
        for (Station& s : st) s.curvature = 0.0;
        return;
    }

    for (size_t i = 1; i + 1 < n; ++i) {
        const glm::dvec2 v_in = st[i].position - st[i - 1].position;
        const glm::dvec2 v_out = st[i + 1].position - st[i].position;
        const double len_in = length2d(v_in);
        const double len_out = length2d(v_out);

        const glm::dvec2 a = (len_in > kLenEpsilon) ? (v_in / len_in) : st[i].tangent;
        const glm::dvec2 b = (len_out > kLenEpsilon) ? (v_out / len_out) : st[i].tangent;

        const double turn = std::atan2(cross2(a, b), glm::dot(a, b));
        const double arc = 0.5 * (len_in + len_out);
        st[i].curvature = (arc > kLenEpsilon && std::isfinite(turn)) ? (turn / arc) : 0.0;
    }

    st[0].curvature = st[1].curvature;
    st[n - 1].curvature = st[n - 2].curvature;
}

// ============================================================================
// Slice Helpers
// ============================================================================

/**
 * @brief Synthesise a station at an arbitrary arclength inside a centerline
 *
 * Bands of zero length -- the gap inside a bevel pair -- are skipped, so the cut
 * always lands on a band with a real direction.
 *
 * The extruder draws a STRAIGHT chord between the two bracketing offset columns,
 * so a synthesised station's own offset column has to land on that chord for
 * every lateral. offset_point() is linear in the miter VECTOR `normal *
 * miter_scale`, and only in that product: interpolating the unit normal and the
 * scale separately gets both the direction and the length wrong -- at a
 * right-angle joint the trimmed arm comes out 0.7 m per 6 m of half width WIDER
 * than the corridor it was cut from, which is the opposite of what a trim is
 * for. The vector is therefore interpolated and the two fields recovered from it.
 *
 * The tangent is the band's own direction of travel rather than a lerp of the
 * two end tangents: inside a band the ribbon is straight, and a joint's tangent
 * is a bisector that does not describe the band it opens.
 *
 * @param st  Source stations, ordered by arclength
 * @param s   Arclength to cut at, assumed inside the station range
 * @return The interpolated station, never flagged as a bevel
 */
[[nodiscard]] Station interpolate_station(const std::vector<Station>& st, double s) {
    Station out = st.front();

    if (st.size() < 2) {
        out.arclength = s;
        return out;
    }

    // Last band whose start is at or before s, ignoring zero-length bands.
    size_t lo = 0;
    bool found = false;
    for (size_t i = 0; i + 1 < st.size(); ++i) {
        const double a0 = st[i].arclength;
        const double a1 = st[i + 1].arclength;
        if (!(a1 > a0 + kArcEpsilon)) continue;
        if (a0 <= s + kArcEpsilon) {
            lo = i;
            found = true;
        }
        if (a0 > s) break;
    }
    if (!found) {
        out.arclength = s;
        return out;
    }

    const Station& a = st[lo];
    const Station& b = st[lo + 1];
    const double span = b.arclength - a.arclength;
    const double u = std::clamp((span > kArcEpsilon) ? (s - a.arclength) / span : 0.0, 0.0, 1.0);

    out.position = a.position + (b.position - a.position) * u;
    out.tangent = normalize_or(b.position - a.position,
                               normalize_or(a.tangent + (b.tangent - a.tangent) * u, a.tangent));

    const glm::dvec2 mv_a = a.normal * a.miter_scale;
    const glm::dvec2 mv_b = b.normal * b.miter_scale;
    const glm::dvec2 mv = mv_a + (mv_b - mv_a) * u;
    const double mv_len = length2d(mv);
    if (mv_len > kLenEpsilon && std::isfinite(mv_len)) {
        out.normal = mv / mv_len;
        out.miter_scale = mv_len;
    } else {
        out.normal = a.normal;
        out.miter_scale = a.miter_scale;
    }

    out.curvature = a.curvature + (b.curvature - a.curvature) * u;
    out.lateral_max = a.lateral_max + (b.lateral_max - a.lateral_max) * u;
    out.lateral_min = a.lateral_min + (b.lateral_min - a.lateral_min) * u;
    out.arclength = s;
    out.is_bevel = false;
    return out;
}

} // namespace

// ============================================================================
// Construction
// ============================================================================

Centerline build_centerline(const std::vector<glm::dvec2>& polyline, const ResampleConfig& cfg) {
    Centerline result;

    const std::vector<glm::dvec2> welded = weld_polyline(polyline, cfg.weld_epsilon);
    if (welded.size() < 2) {
        spdlog::debug("build_centerline: polyline welded down to {} distinct point(s), skipping",
                      welded.size());
        return result;
    }

    // --- Dense geometry -----------------------------------------------------
    // Smoothing joins the surveyed points with a curve that passes through every
    // one of them, split at corners so a real corner stays a corner. With
    // smoothing off the welded polyline is the geometry unchanged.
    const DenseCurve dc = build_dense_curve(welded, cfg);
    const std::vector<glm::dvec2>& dense = dc.points;

    if (dense.size() < 2 || dc.breaks.size() < 2) {
        spdlog::debug("build_centerline: flattened curve collapsed, skipping");
        return result;
    }

    const std::vector<double> dense_arc = cumulative_arclength(dense);
    const double total_length = dense_arc.back();
    if (!(total_length > kLenEpsilon) || !std::isfinite(total_length)) {
        spdlog::debug("build_centerline: polyline has zero length, skipping");
        return result;
    }

    // --- Curvature driving the adaptive spacing -----------------------------
    // Curvature exists to bound the chord-to-arc deviation of a CURVE. A break --
    // an unsmoothed survey vertex, or a corner the fit was split at -- is a kink,
    // not curvature: it carries a station of its own and the miter reproduces it
    // exactly. Measuring the kink as curvature would instead carpet every corner
    // in min_spacing stations for no gain, so it is zeroed at the break and at
    // its two neighbours, which are the samples the three-point estimate reaches.
    std::vector<double> dense_k = dense_curvature(dense);
    for (const size_t b : dc.breaks) {
        const size_t lo = (b > 0) ? b - 1 : 0;
        const size_t hi = std::min(b + 1, dense.size() - 1);
        for (size_t i = lo; i <= hi; ++i) dense_k[i] = 0.0;
    }

    // --- Mandatory stops ----------------------------------------------------
    std::vector<double> stops;
    stops.reserve(dc.breaks.size());
    for (const size_t b : dc.breaks) {
        const double s = dense_arc[b];
        if (stops.empty() || s > stops.back() + kArcEpsilon) stops.push_back(s);
    }
    if (stops.size() < 2) {
        spdlog::debug("build_centerline: polyline has no usable span, skipping");
        return result;
    }

    const std::vector<double> station_arc =
        choose_station_arclengths(dense_arc, dense_k, stops, cfg);
    if (station_arc.size() < 2) {
        spdlog::debug("build_centerline: resampling produced {} station(s), skipping",
                      station_arc.size());
        return result;
    }

    std::vector<glm::dvec2> station_pos;
    station_pos.reserve(station_arc.size());
    for (const double s : station_arc) {
        station_pos.push_back(sample_at_arclength(dense, dense_arc, s));
    }

    // The ends must be the surveyed ends exactly, so neighbouring edges still
    // meet at a junction after any rounding in the arclength walk.
    station_pos.front() = dense.front();
    station_pos.back() = dense.back();

    result.stations = build_frames(station_pos, station_arc, cfg);
    if (!result.is_valid()) {
        result.stations.clear();
        return result;
    }

    bound_lateral_folds(result.stations);
    compute_curvature(result.stations);
    return result;
}

// ============================================================================
// Slicing
// ============================================================================

Centerline slice(const Centerline& c, double from_arclength, double to_arclength) {
    Centerline result;
    if (!c.is_valid()) return result;

    if (!std::isfinite(from_arclength) || !std::isfinite(to_arclength)) return result;
    if (from_arclength > to_arclength) std::swap(from_arclength, to_arclength);

    // Clamp to the source parameterisation. For a centerline from
    // build_centerline() the front arclength is 0 and this is the documented
    // [0, length()]; for a slice of a slice, whose arclengths are not rebased,
    // the front arclength is the correct lower bound.
    const double lo_bound = c.stations.front().arclength;
    const double hi_bound = c.stations.back().arclength;
    const double from = std::clamp(from_arclength, lo_bound, hi_bound);
    const double to = std::clamp(to_arclength, lo_bound, hi_bound);
    if (!(to - from > kArcEpsilon)) return result;

    const std::vector<Station>& src = c.stations;

    // Every source station inside the cut, plus synthesised ends where the cut
    // falls between stations. Exactness matters: the P4 junction solver trims
    // every arm to a computed station, and snapping to the nearest existing
    // station would leave a gap at the junction the width of one band.
    std::vector<Station> out;
    out.reserve(src.size() + 2);

    if (src.front().arclength < from - kArcEpsilon) {
        out.push_back(interpolate_station(src, from));
    }

    for (const Station& s : src) {
        if (s.arclength < from - kArcEpsilon) continue;
        if (s.arclength > to + kArcEpsilon) break;
        out.push_back(s);
    }

    if (src.back().arclength > to + kArcEpsilon) {
        out.push_back(interpolate_station(src, to));
    }

    // A cut landing exactly on a bevel pair would otherwise open or close the
    // slice with a zero-length band carrying the wrong side's normal. Keep the
    // outgoing half at the start and the incoming half at the end.
    while (out.size() >= 2 && !(out[1].arclength > out[0].arclength + kArcEpsilon)) {
        out.erase(out.begin());
    }
    while (out.size() >= 2
           && !(out[out.size() - 1].arclength > out[out.size() - 2].arclength + kArcEpsilon)) {
        out.pop_back();
    }

    if (out.size() < 2) return result;

    result.stations = std::move(out);
    return result;
}

} // namespace stratum::osm::road
