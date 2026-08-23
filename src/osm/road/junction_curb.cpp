/**
 * @file junction_curb.cpp
 * @brief Implementation of the junction sidewalk and curb ring
 *
 * The ring is built in four stages, and the interesting one is the third.
 *
 *   1. **Offset.** The junction carriageway ring is inflated outward by
 *      CurbRingConfig::ring_offset() with Clipper2. This is the only Clipper2
 *      call in the road pipeline. It is used rather than a hand-rolled miter
 *      offset because the junction ring already carries tessellated fillet arcs,
 *      and a naive offset of such a ring self-intersects at every tight corner.
 *
 *   2. **Correspondence.** The offset ring and the junction ring do not
 *      correspond by index -- the round join inserts vertices at every convex
 *      corner and the union pass deletes them wherever the offset folded through
 *      itself. They are zipped back together by walking the OUTER ring and
 *      projecting each of its vertices onto the nearest point of the INNER ring.
 *      That projection is exact for an outward offset: every point of the offset
 *      boundary is at distance `ring_offset()` from the polygon, so its nearest
 *      inner point is the point the offset was taken from. Several outer vertices
 *      collapsing onto one inner vertex is not a failure, it IS the round join at
 *      a convex corner.
 *
 *   3. **Gaps.** A ring closed all the way round walls the junction in with a
 *      curb across every approach. Each arm's mouth is skipped: the span of the
 *      ring lying between the two lines parallel to that arm through its outer
 *      profile corners ArmEnd::left and ArmEnd::right. What is left is N open
 *      SECTIONS, one per corner, each running from arm k's left mouth edge round
 *      the fillet to arm k+1's right mouth edge. The two boundary points are
 *      interpolated exactly rather than snapped to a vertex, so the gap is
 *      exactly as wide as the arm's own sidewalk-to-sidewalk extent and the two
 *      curbs meet end to end.
 *
 *   4. **Extrusion.** Each section is swept as four bands -- apron, curb face,
 *      curb top, sidewalk -- with the same winding pattern and the same UV convention
 *      build_corridor() uses, so the ring and the ribbons feeding it are the same
 *      surface.
 *
 * Everything here lives in stratum_core: no SDL, no ImGui, no rendering API.
 */

#include "osm/road/junction_curb.hpp"

#include "osm/road/corridor.hpp"    // uv_tiling()

#include <clipper2/clipper.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace stratum::osm::road {

namespace {

// ============================================================================
// Tolerances
// ============================================================================

/// Squared length of the raw face cross product below which a triangle is dropped
constexpr double kDegenerateCrossSq = 1e-16;

/// Lengths at or below this count as zero, metres
constexpr double kZeroLength = 1e-9;

/// A ring section shorter than this along its inner boundary emits nothing, metres
constexpr double kMinSectionLength = 1e-3;

/// Slack applied to the arclength window comparisons, metres
constexpr double kArclengthEpsilon = 1e-9;

constexpr double kTwoPi = 6.283185307179586476925286766559;
constexpr double kPi = 3.141592653589793238462643383279;

/**
 * @brief Spacing the ring is resampled at across a kerb drop, metres
 *
 * The offset ring's own vertices are metres apart on a straight run, so a 1 m
 * ramp laid out against them would fall between two columns and become a step.
 * 0.2 m puts at least five columns on the shortest ramp anyone would configure.
 */
constexpr double kDropSampleStep = 0.2;

/// Ceiling on the extra columns one band may be split into across a drop
constexpr size_t kMaxDropSplit = 64;

/// Shortest ramp a drop may use, metres; below this the ramp is a step and tears
constexpr double kMinDropRamp = 0.05;

/**
 * @brief Distance from an arm's cut line within which a ring point counts as ON it
 *
 * The ring vertex an arm's mouth ends at IS the arm's carriageway corner, so its
 * distance from the cut line is zero but for rounding. The next vertex round the
 * fillet is centimetres away at the very least. A micrometre separates the two
 * cases with room to spare.
 */
constexpr double kOnCutLine = 1e-6;

/**
 * @brief Largest local coordinate that survives the scale to int64, metres
 *
 * Clipper2 works in int64 and its own internal arithmetic squares coordinates,
 * so the usable range is nearer 2^31 than 2^63. At the default clipper_scale of
 * 1000 this cap is 1e6 m of local extent, roughly 20 times the widest OSM
 * extract anyone imports, and it exists only so a corrupt coordinate is refused
 * rather than silently wrapped.
 */
constexpr double kMaxScaledCoordinate = 1.0e9;

// ============================================================================
// Small geometry helpers
// ============================================================================

/**
 * @brief The codebase-wide 2D-to-3D mapping, Y up
 *
 * (x, y_2d) -> (x, height, -y_2d). The same mapping corridor.cpp and
 * junction_polygon.cpp use; changing it here would put the ring on a different
 * plane from the junction it wraps.
 */
[[nodiscard]] inline glm::vec3 to_world(const glm::dvec2& p, double height) {
    return glm::vec3(static_cast<float>(p.x),
                     static_cast<float>(height),
                     static_cast<float>(-p.y));
}

/// Wrap an angle into [0, 2pi)
[[nodiscard]] inline double wrap_positive(double a) {
    a = std::fmod(a, kTwoPi);
    if (a < 0.0) {
        a += kTwoPi;
    }
    return a;
}

/// 2D cross product, positive when b turns left of a
[[nodiscard]] inline double cross2(const glm::dvec2& a, const glm::dvec2& b) {
    return a.x * b.y - a.y * b.x;
}

/// Signed area of a closed ring, positive when counter-clockwise
[[nodiscard]] double signed_area(const std::vector<glm::dvec2>& ring) {
    if (ring.size() < 3) {
        return 0.0;
    }
    double twice = 0.0;
    for (size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
        twice += cross2(ring[j], ring[i]);
    }
    return twice * 0.5;
}

/// Crossing-number point-in-polygon test over a closed ring
[[nodiscard]] bool point_in_ring(const std::vector<glm::dvec2>& ring, const glm::dvec2& p) {
    if (ring.size() < 3) {
        return false;
    }
    bool inside = false;
    for (size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
        const glm::dvec2& a = ring[i];
        const glm::dvec2& b = ring[j];
        if ((a.y > p.y) != (b.y > p.y)) {
            const double x = (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x;
            if (p.x < x) {
                inside = !inside;
            }
        }
    }
    return inside;
}

[[nodiscard]] inline bool ring_is_finite(const std::vector<glm::dvec2>& ring) {
    for (const glm::dvec2& p : ring) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
            return false;
        }
    }
    return true;
}

// ============================================================================
// Clipper2 bridge
// ============================================================================

/**
 * @brief Inflate a closed ring outward, in metres, via Clipper2
 *
 * Clipper2 is integer-based, so metres are multiplied by @p scale, ROUNDED to
 * nearest rather than truncated -- truncation biases every coordinate towards
 * zero by up to one unit, which drifts the corners of the ring off the arm ends
 * they have to meet -- and divided back afterwards.
 *
 * `EndType::Polygon` offsets one side of a closed path; `JoinType::Round`
 * produces the rounded corner the plan asks for. Positive delta always inflates
 * regardless of the input's orientation, and Clipper2 returns the solution in
 * the orientation it was given, but the caller re-orients anyway rather than
 * trusting that.
 *
 * @param ring     Closed ring in local metres, first point not repeated
 * @param delta    Outward offset in metres; must be positive
 * @param scale    Metres-to-integer factor, CurbRingConfig::clipper_scale
 * @param arc_tol  Maximum chord deviation of a rounded corner, metres
 * @param out      Receives every solution path, scaled back to metres
 * @return False when the input could not be represented, or the offset returned
 *         nothing at all
 */
[[nodiscard]] bool inflate_ring(const std::vector<glm::dvec2>& ring,
                                double delta,
                                double scale,
                                double arc_tol,
                                std::vector<std::vector<glm::dvec2>>& out) {
    out.clear();

    if (ring.size() < 3 || !(delta > 0.0) || !(scale > 0.0)) {
        return false;
    }

    Clipper2Lib::Path64 subject;
    subject.reserve(ring.size());

    for (const glm::dvec2& p : ring) {
        const double sx = p.x * scale;
        const double sy = p.y * scale;
        if (!std::isfinite(sx) || !std::isfinite(sy) ||
            std::fabs(sx) > kMaxScaledCoordinate || std::fabs(sy) > kMaxScaledCoordinate) {
            spdlog::warn("build_curb_ring: junction ring coordinate {:.3f},{:.3f} is out of "
                         "range for clipper_scale {:.1f}; skipping the ring",
                         p.x, p.y, scale);
            return false;
        }
        const Clipper2Lib::Point64 q(static_cast<int64_t>(std::llround(sx)),
                                     static_cast<int64_t>(std::llround(sy)));
        // Millimetre rounding can collapse two ring vertices onto one point.
        if (!subject.empty() && subject.back().x == q.x && subject.back().y == q.y) {
            continue;
        }
        subject.push_back(q);
    }

    // The closing edge can be a duplicate too.
    while (subject.size() >= 2 &&
           subject.front().x == subject.back().x &&
           subject.front().y == subject.back().y) {
        subject.pop_back();
    }

    if (subject.size() < 3) {
        return false;
    }

    Clipper2Lib::Paths64 solution =
        Clipper2Lib::InflatePaths(Clipper2Lib::Paths64{ subject },
                                  delta * scale,
                                  Clipper2Lib::JoinType::Round,
                                  Clipper2Lib::EndType::Polygon,
                                  2.0,                    // miter limit, unused for Round
                                  arc_tol * scale);

    if (solution.empty()) {
        return false;
    }

    const double inv_scale = 1.0 / scale;
    out.reserve(solution.size());

    for (const Clipper2Lib::Path64& path : solution) {
        std::vector<glm::dvec2> ring_out;
        ring_out.reserve(path.size());
        for (const Clipper2Lib::Point64& q : path) {
            const glm::dvec2 p(static_cast<double>(q.x) * inv_scale,
                               static_cast<double>(q.y) * inv_scale);
            if (!ring_out.empty() &&
                glm::dot(p - ring_out.back(), p - ring_out.back()) <= kZeroLength * kZeroLength) {
                continue;
            }
            ring_out.push_back(p);
        }
        while (ring_out.size() >= 2 &&
               glm::dot(ring_out.front() - ring_out.back(),
                        ring_out.front() - ring_out.back()) <= kZeroLength * kZeroLength) {
            ring_out.pop_back();
        }
        if (ring_out.size() >= 3) {
            out.push_back(std::move(ring_out));
        }
    }

    return !out.empty();
}

// ============================================================================
// Inner-to-outer correspondence
// ============================================================================

/**
 * @brief One matched pair of ring points, plus where the inner one sits
 */
struct RingSample {
    /// Point on the junction carriageway ring
    glm::dvec2 inner{0.0};

    /// Matching point on the offset ring, outward of `inner`
    glm::dvec2 outer{0.0};

    /// Arclength of `inner` measured around the junction ring from ring[0], metres
    double u = 0.0;
};

/**
 * @brief Cumulative arclength of a closed ring's vertices, plus its perimeter
 *
 * `out_u[i]` is the distance from ring[0] to ring[i] walking forward. The
 * returned perimeter includes the closing edge.
 */
[[nodiscard]] double ring_arclengths(const std::vector<glm::dvec2>& ring,
                                     std::vector<double>& out_u) {
    const size_t n = ring.size();
    out_u.assign(n, 0.0);
    double running = 0.0;
    for (size_t i = 1; i < n; ++i) {
        running += glm::length(ring[i] - ring[i - 1]);
        out_u[i] = running;
    }
    return running + glm::length(ring[0] - ring[n - 1]);
}

/**
 * @brief Nearest point on a closed ring to @p q, as an arclength around the ring
 *
 * Exhaustive over the ring's edges. A junction ring is tens of vertices and an
 * offset of it is at most a couple of hundred, so the quadratic total is a few
 * thousand operations per junction and is not worth indexing.
 *
 * @param ring    Closed ring, first point not repeated
 * @param cum_u   Cumulative arclengths from ring_arclengths()
 * @param q       Query point
 * @param out_foot Receives the nearest point itself
 * @return Arclength of the nearest point around the ring, metres
 */
[[nodiscard]] double nearest_on_ring(const std::vector<glm::dvec2>& ring,
                                     const std::vector<double>& cum_u,
                                     double perimeter,
                                     const glm::dvec2& q,
                                     glm::dvec2& out_foot) {
    const size_t n = ring.size();
    double best_dist_sq = std::numeric_limits<double>::max();
    double best_u = 0.0;
    out_foot = ring[0];

    for (size_t i = 0; i < n; ++i) {
        const glm::dvec2& a = ring[i];
        const glm::dvec2& b = ring[(i + 1) % n];
        const glm::dvec2 d = b - a;
        const double len_sq = glm::dot(d, d);

        double t = 0.0;
        if (len_sq > kZeroLength * kZeroLength) {
            t = glm::dot(q - a, d) / len_sq;
            t = std::clamp(t, 0.0, 1.0);
        }

        const glm::dvec2 foot = a + d * t;
        const glm::dvec2 delta = q - foot;
        const double dist_sq = glm::dot(delta, delta);

        if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            out_foot = foot;
            const double edge_len = std::sqrt(len_sq);
            const double base = cum_u[i];
            best_u = base + t * edge_len;
            // The closing edge runs past the end of cum_u and wraps back onto
            // ring[0]. Wrapping AT the perimeter, not past it, matters: leaving a
            // sample at exactly `perimeter` puts a point that belongs at the start
            // of the ring at its end and breaks the rotation below.
            if (best_u >= perimeter) {
                best_u -= perimeter;
            }
        }
    }

    return best_u;
}

/**
 * @brief Zip the offset ring back onto the junction ring, in ring order
 *
 * Every outer vertex is projected onto the inner ring; the resulting arclengths
 * are monotonically non-decreasing around the ring, with exactly one wrap. The
 * samples are rotated so that the wrap sits at the end, which makes every later
 * arclength window a contiguous run.
 *
 * The wrap is found as the LARGEST backward step rather than the first one:
 * the true wrap drops by very nearly the whole perimeter, while floating-point
 * noise at a corner can produce a spurious backward step of a few nanometres.
 */
[[nodiscard]] std::vector<RingSample> zip_rings(const std::vector<glm::dvec2>& inner,
                                                const std::vector<glm::dvec2>& outer,
                                                const std::vector<double>& cum_u,
                                                double perimeter) {
    std::vector<RingSample> samples;
    samples.reserve(outer.size());

    for (const glm::dvec2& q : outer) {
        RingSample s;
        s.outer = q;
        s.u = nearest_on_ring(inner, cum_u, perimeter, q, s.inner);
        samples.push_back(s);
    }

    const size_t m = samples.size();
    if (m < 2) {
        return samples;
    }

    size_t wrap = 0;
    double biggest_drop = 0.0;
    for (size_t i = 0; i < m; ++i) {
        const double prev = samples[(i + m - 1) % m].u;
        const double drop = prev - samples[i].u;
        if (drop > biggest_drop) {
            biggest_drop = drop;
            wrap = i;
        }
    }

    if (wrap != 0) {
        std::rotate(samples.begin(), samples.begin() + static_cast<std::ptrdiff_t>(wrap),
                    samples.end());
    }

    return samples;
}

// ============================================================================
// Arm mouths
// ============================================================================

/**
 * @brief The cut line of one arm, as a signed distance
 *
 * `along_of(p)` is p's coordinate along the direction LEAVING the node, measured
 * from the trim station. It is zero exactly on the arm's cut line, positive out
 * along the approach, negative back inside the junction.
 */
struct ArmCut {
    glm::dvec2 center{0.0};
    glm::dvec2 dir{1.0, 0.0};       ///< unit, LEAVING the node
    glm::dvec2 perp{0.0, 1.0};      ///< unit, LEFT of `dir`; the cut line's own direction

    /// Half the CARRIAGEWAY width at the cut, metres; the lane region's reach
    double half = 0.0;

    [[nodiscard]] double along_of(const glm::dvec2& p) const {
        return glm::dot(p - center, dir);
    }

    /// Lateral coordinate of p on the cut line, positive towards `perp`
    [[nodiscard]] double across_of(const glm::dvec2& p) const {
        return glm::dot(p - center, perp);
    }

    /// True when p sits on the cut line to within kOnCutLine
    [[nodiscard]] bool on_line(const glm::dvec2& p) const {
        return std::fabs(along_of(p)) <= kOnCutLine;
    }

    /**
     * @brief Where the segment a -> b first enters this arm's running lanes
     *
     * The lane region is `along > 0 && |across| < half`: the drivable surface of
     * the approach, past the cut, between the carriageway edges. It is the
     * intersection of three half-planes and therefore convex, so a segment that
     * starts outside it crosses its boundary exactly once and the entry parameter
     * is the standard slab clip -- the largest of the three per-half-plane entry
     * parameters, valid only while it stays below the smallest of the three exit
     * parameters.
     *
     * @param a Segment start
     * @param b Segment end
     * @return Parameter in [0, 1] at which the segment enters the lanes, or 1.0
     *         when it never does
     */
    [[nodiscard]] double lane_entry(const glm::dvec2& a, const glm::dvec2& b) const {
        if (!(half > kZeroLength)) {
            return 1.0;
        }

        // Each row is one half-plane of the region, as f(a) and f(b); the region
        // is where every f is positive.
        const double f[3][2] = {
            {along_of(a), along_of(b)},
            {half - across_of(a), half - across_of(b)},
            {across_of(a) + half, across_of(b) + half},
        };

        double t_enter = 0.0;
        double t_exit = 1.0;

        for (const auto& row : f) {
            const double fa = row[0];
            const double fb = row[1];

            if (fa > 0.0 && fb > 0.0) {
                continue;                       // satisfied along the whole segment
            }
            if (fa <= 0.0 && fb <= 0.0) {
                return 1.0;                     // never satisfied: no entry at all
            }

            const double denom = fa - fb;
            if (!(std::fabs(denom) > kZeroLength)) {
                return 1.0;
            }
            const double crossing = fa / denom;

            if (fa <= 0.0) {
                t_enter = std::max(t_enter, crossing);
            } else {
                t_exit = std::min(t_exit, crossing);
            }
        }

        return (t_enter < t_exit) ? std::clamp(t_enter, 0.0, 1.0) : 1.0;
    }
};

/**
 * @brief Resolve an arm's cut cross-section into the cut line that bounds its mouth
 *
 * The mouth is bounded by the arm's CUT LINE and not, as an earlier draft of this
 * file's header had it, by the arm's outer profile corners ArmEnd::left and
 * ArmEnd::right. The profile corners cannot bound it. At a real junction the
 * carriageway is trimmed only as far back as the point where the two
 * carriageways stop overlapping -- 3.75 m from the node for two 7 m roads meeting
 * at a right angle -- while the arms' own sidewalks reach 5.95 m to either side
 * of their centrelines and run right up to that cut. The whole corner of the ring
 * therefore lies within both neighbouring arms' full-profile corridors, and a
 * mouth measured at the profile width swallows every corner of every junction and
 * emits nothing at all. That was measured, not guessed.
 *
 * What the ring must genuinely not do is run ACROSS an approach, and an approach
 * is crossed exactly where a band cross-section has its inner end ON the cut line
 * and its outer end beyond it. Bounding the mouth at the cut line expresses that
 * directly, and leaves the ring overlapping the ends of its arms' sidewalks by a
 * little at each corner, which is the lesser and less visible error.
 */
[[nodiscard]] ArmCut cut_of(const ArmEnd& end) {
    ArmCut c;
    c.center = end.center;

    glm::dvec2 dir = end.direction;
    const double dir_len = glm::length(dir);
    c.dir = (dir_len > kZeroLength) ? (dir / dir_len) : glm::dvec2(1.0, 0.0);
    c.perp = glm::dvec2(-c.dir.y, c.dir.x);

    // ------------------------------------------------------------------------
    // The axis comes from the CUT FACE, not from ArmEnd::direction.
    //
    // The face is laid out along the station's MITER BISECTOR, and on a curved
    // approach that bisector is NOT perpendicular to the direction the arm end
    // reports. arm_end() synthesises its station with slice(), and
    // interpolate_station() sets the tangent to the band CHORD while it sets the
    // normal to the interpolated miter VECTOR; the two are perpendicular only on
    // a dead-straight run. Taking `dir` at its word therefore leaves
    // along_of(carriage_left) at centimetres to decimetres rather than at zero --
    // measured at 0.22 m on a 20 m-radius bend -- so on_line()'s micron tolerance
    // never fires, clip_section() trims nothing, and the untrimmed round join is
    // emitted straight up the approach on top of the arm's own sidewalk.
    //
    // Deriving the axis from the two carriageway corners makes along_of() exactly
    // zero on both of them by construction, and across_of() exactly +/- half, so
    // the lane slab no longer swallows the arm's own corners either. On a
    // straight arm the two constructions agree bit for bit.
    // ------------------------------------------------------------------------
    glm::dvec2 face = end.carriage_left - end.carriage_right;
    const double face_len = glm::length(face);
    if (face_len > kZeroLength) {
        face /= face_len;
        glm::dvec2 axis(face.y, -face.x);
        if (glm::dot(axis, c.dir) < 0.0) {
            axis = -axis;       // keep `dir` pointing the way the arm leaves the node
        }
        c.dir = axis;
        c.perp = face;
    }

    // The carriageway corners lie on the cut line, so their separation is the
    // lane region's full width. Measured rather than taken from ArmRef so the
    // region agrees exactly with the ring vertices the mouth is opened at.
    c.half = 0.5 * face_len;
    return c;
}

// ============================================================================
// Section extraction
// ============================================================================

/// A ring section: matched inner and outer boundaries, open at both ends
using Section = std::vector<RingSample>;

[[nodiscard]] RingSample lerp_sample(const RingSample& a, const RingSample& b, double f) {
    RingSample s;
    s.inner = a.inner + (b.inner - a.inner) * f;
    s.outer = a.outer + (b.outer - a.outer) * f;
    s.u = a.u;      // not meaningful past the clip; nothing downstream reads it
    return s;
}

/**
 * @brief Collect the samples whose inner foot lies in one arclength window
 *
 * @p samples is rotated so its arclengths are non-decreasing, so the window is a
 * suffix, or a suffix followed by the wrapped prefix.
 *
 * @param samples Rotated ring samples
 * @param u_from  Window start, an arclength around the inner ring
 * @param span    Window length walked forward from @p u_from, metres
 * @param perimeter Inner ring perimeter, metres
 */
[[nodiscard]] Section window_samples(const std::vector<RingSample>& samples,
                                     double u_from,
                                     double span,
                                     double perimeter) {
    Section window;
    if (samples.empty() || perimeter <= kZeroLength) {
        return window;
    }

    // Normalise the window start into the same range the sample arclengths live
    // in, then measure every sample as a FORWARD distance from it. Working in
    // that relative coordinate is what makes the section that wraps past ring
    // vertex 0 behave like every other one -- comparing absolute arclengths
    // against a window ending exactly AT the perimeter silently drops the
    // terminal vertex of that one section, and its curb with it.
    double lo = std::fmod(u_from, perimeter);
    if (lo < 0.0) {
        lo += perimeter;
    }

    // The samples ascend in arclength from samples.front().u, so those at or past
    // the window start come first and the wrapped remainder follows.
    for (const RingSample& s : samples) {
        if (s.u >= lo - kArclengthEpsilon && (s.u - lo) <= span + kArclengthEpsilon) {
            window.push_back(s);
        }
    }
    for (const RingSample& s : samples) {
        if (s.u >= lo - kArclengthEpsilon) {
            continue;
        }
        if ((s.u - lo + perimeter) <= span + kArclengthEpsilon) {
            window.push_back(s);
        }
    }

    return window;
}

/**
 * @brief Clip one window down to the ring section that is actually drawn
 *
 * The window already excludes every arm's cut face -- it runs from arm k's
 * carriageway LEFT corner round to arm k+1's carriageway RIGHT corner -- so the
 * band never lies across a carriageway. Two smaller things are left to fix, both
 * at the two ends of the window.
 *
 * ### Trim the round join back to the cut line
 *
 * The window's first and last inner points are ring VERTICES and both sit on an
 * arm's cut line. The junction ring turns a convex corner at each, so the offset
 * sweeps a round join through the turn, and the join fans outward from a single
 * point. Everything in that fan lies OUTSIDE the carriageway -- it is the arm's
 * gutter, curb and sidewalk it reaches over, not the road -- but it reaches over
 * them at the ring's SIDEWALK height, so it hangs 15 cm above the arm's gutter
 * and z-fights the arm's own sidewalk. The part of the fan beyond the cut line is
 * therefore dropped, and where the corner turns far enough for the join to cross
 * the cut line the crossing is interpolated exactly, so the ring ends flush
 * against the arm's cross-section.
 *
 * The test stops as soon as the inner point leaves the cut line. Past that the
 * band is turning the corner rather than reaching up an approach, and its outer
 * boundary is expected to bulge past the cut line -- at a right-angle crossroads
 * the whole fillet does.
 *
 * ### Put the curb back on the corner
 *
 * Dropping the whole fan, which is what a 45 degree corner does, would start the
 * ring's curb at the first fillet vertex instead of at the arm's carriageway
 * corner, leaving a notch in the curb line as wide as one fillet segment. So when
 * the fan is gone the ring VERTEX is put back as the section's terminal
 * cross-section, carrying the outward direction of the sample next to it. The
 * curb then runs from the arm's carriageway corner without a break, and the
 * sliver of sidewalk this adds is one fillet segment wide.
 *
 * @param window   Samples in ring order, from arm k's carriageway left corner to
 *                 arm k+1's carriageway right corner
 * @param cut_from Arm k's cut line; the section starts on it
 * @param cut_to   Arm k+1's cut line; the section ends on it
 * @param width    CurbRingConfig::ring_offset(), the ring's reach
 */
[[nodiscard]] Section clip_section(const Section& window,
                                   const ArmCut& cut_from,
                                   const ArmCut& cut_to,
                                   double width) {
    Section section;
    if (window.size() < 2) {
        return section;
    }

    // Leading join: inner still on arm k's cut line, outer beyond it.
    size_t first = 0;
    while (first < window.size() &&
           cut_from.on_line(window[first].inner) &&
           cut_from.along_of(window[first].outer) > 0.0) {
        ++first;
    }
    if (first >= window.size()) {
        return section;
    }

    // Trailing join: the same at arm k+1.
    size_t last = window.size();
    while (last > first + 1 &&
           cut_to.on_line(window[last - 1].inner) &&
           cut_to.along_of(window[last - 1].outer) > 0.0) {
        --last;
    }
    if (last <= first + 1) {
        return section;
    }

    if (first > 0) {
        const double a = cut_from.along_of(window[first - 1].outer);    // dropped, > 0
        const double b = cut_from.along_of(window[first].outer);
        if (b <= 0.0 && (a - b) > kZeroLength) {
            section.push_back(lerp_sample(window[first - 1], window[first], a / (a - b)));
        }
    }

    for (size_t i = first; i < last; ++i) {
        section.push_back(window[i]);
    }

    if (last < window.size()) {
        const double b = cut_to.along_of(window[last - 1].outer);
        const double a = cut_to.along_of(window[last].outer);           // dropped, > 0
        if (b <= 0.0 && (a - b) > kZeroLength) {
            section.push_back(lerp_sample(window[last - 1], window[last], -b / (a - b)));
        }
    }

    if (section.size() < 2) {
        section.clear();
        return section;
    }

    // Put the ring vertex back on each end when its whole join was dropped, so
    // the curb starts and ends on the arm's carriageway corner.
    const auto restore_vertex = [&](const RingSample& vertex, const RingSample& neighbour,
                                    RingSample& out_sample) {
        const glm::dvec2 outward = neighbour.outer - neighbour.inner;
        const double len = glm::length(outward);
        if (len <= kZeroLength) {
            return false;
        }
        const glm::dvec2 delta = vertex.inner - neighbour.inner;
        if (glm::dot(delta, delta) <= kZeroLength * kZeroLength) {
            return false;       // already there
        }
        out_sample.inner = vertex.inner;
        out_sample.outer = vertex.inner + (outward / len) * width;
        out_sample.u = vertex.u;
        return true;
    };

    RingSample restored{};
    if (first > 0 && cut_from.on_line(window.front().inner) &&
        restore_vertex(window.front(), section.front(), restored)) {
        section.insert(section.begin(), restored);
    }
    if (last < window.size() && cut_to.on_line(window.back().inner) &&
        restore_vertex(window.back(), section.back(), restored)) {
        section.push_back(restored);
    }

    if (section.size() < 2) {
        section.clear();
    }
    return section;
}

/**
 * @brief Pull every band cross-section out of every arm's running lanes
 *
 * clip_section() trims the two ROUND JOINS at the ends of a window, which is the
 * only place the ring's INNER boundary reaches past a cut line. It cannot be the
 * whole story, because the band is 2 m wide and its OUTER boundary is free to go
 * somewhere the inner never does.
 *
 * At an ordinary crossroads that freedom is wanted: the fillet's outer boundary
 * bulges past both neighbouring cut lines, diagonally outward, well clear of
 * either carriageway, and clipping it would cut the corner off the sidewalk.
 *
 * At an ACUTE fork it is not. The gore edge of the junction polygon meets the
 * stem's cut line at a shallow angle, so that edge's outward normal points very
 * nearly straight UP the stem, and the plain parallel offset of the edge -- no
 * round join involved, so clip_section() never looks at it -- lands the full
 * sidewalk width up the middle of a running lane. Measured on a 15 degree fork:
 * 2.0 m past the cut and 0.25 m inside the carriageway edge.
 *
 * What separates the two is not whether the outer boundary passes a cut line but
 * whether it ends up in a LANE, so that is what is tested. Each sample's
 * cross-section is shortened to stop at the lane boundary, against EVERY arm and
 * not only the two bounding this section, since a long gore band can reach a
 * third arm. Shortening rather than dropping keeps the curb line continuous all
 * the way to the arm's carriageway corner -- build_columns() already caps the
 * curb face and top against a short reach -- so the sidewalk tapers into the
 * corner instead of leaving a notch in the curb.
 *
 * A sample whose INNER point is already inside a lane has no cross-section left
 * to keep and is dropped. That splits the section, so the longest surviving run
 * is kept and the rest discarded: a run of one sample cannot be swept anyway.
 *
 * @param section Samples to clamp, in ring order
 * @param cuts    Every arm's cut line and lane region
 * @return The clamped section, possibly shorter; empty when nothing survives
 */
[[nodiscard]] Section clamp_out_of_lanes(const Section& section,
                                         const std::vector<ArmCut>& cuts) {
    if (section.size() < 2 || cuts.empty()) {
        return section;
    }

    Section clamped;
    clamped.reserve(section.size());

    size_t best_begin = 0;
    size_t best_length = 0;
    size_t run_begin = 0;
    size_t run_length = 0;

    for (const RingSample& s : section) {
        double t = 1.0;
        for (const ArmCut& cut : cuts) {
            t = std::min(t, cut.lane_entry(s.inner, s.outer));
        }

        if (t <= kZeroLength) {
            // The inner foot itself is in a lane; there is no cross-section to
            // keep. End the current run.
            run_begin = clamped.size();
            run_length = 0;
            continue;
        }

        RingSample out = s;
        if (t < 1.0) {
            out.outer = s.inner + (s.outer - s.inner) * t;
        }

        if (run_length == 0) {
            run_begin = clamped.size();
        }
        clamped.push_back(out);
        ++run_length;

        if (run_length > best_length) {
            best_begin = run_begin;
            best_length = run_length;
        }
    }

    if (best_length < 2) {
        return Section{};
    }
    if (best_length == clamped.size()) {
        return clamped;
    }
    return Section(clamped.begin() + static_cast<std::ptrdiff_t>(best_begin),
                   clamped.begin() + static_cast<std::ptrdiff_t>(best_begin + best_length));
}

// ============================================================================
// Extrusion
// ============================================================================

/**
 * @brief The kerb-drop height of the ring's curb top, as a function of position
 *
 * A DroppedKerbSpan is a counter-clockwise pair of DIRECTIONS from the junction
 * centre, not a pair of points and not a pair of vertex indices. That is what
 * lets a span computed against the crossing geometry be applied to a ring whose
 * vertices Clipper2 chose: every point of either ring has a direction from the
 * centre, so the span can be evaluated at any point of either boundary.
 *
 * Evaluation is deliberately CONTINUOUS in that direction. Inside a span the
 * kerb is at the lip; outside it the kerb ramps back to full height over
 * KerbDrops::ramp_length of ARC, measured at the radius of the point being
 * evaluated so the ramp is the same number of metres of kerb wherever on the
 * ring it lands. Where two spans overlap the deepest drop wins, so the spans
 * from a crossing and from a driveway may simply be concatenated.
 */
class DropProfile {
public:
    DropProfile() = default;

    DropProfile(const KerbDrops* drops, double curb_height) {
        if (drops == nullptr || drops->spans.empty()) {
            return;
        }
        if (!std::isfinite(drops->center.x) || !std::isfinite(drops->center.y)) {
            return;
        }

        m_center = drops->center;
        // A ramp of zero is an instant step, which is the tear this whole
        // mechanism exists to avoid. Refuse to honour one.
        m_ramp = std::max(kMinDropRamp, drops->ramp_length);

        for (const DroppedKerbSpan& span : drops->spans) {
            if (!std::isfinite(span.from.x) || !std::isfinite(span.from.y) ||
                !std::isfinite(span.to.x) || !std::isfinite(span.to.y)) {
                continue;
            }
            if (glm::length(span.from) < 0.5 || glm::length(span.to) < 0.5) {
                continue;
            }

            const double start = std::atan2(span.from.y, span.from.x);
            const double end = std::atan2(span.to.y, span.to.x);
            const double sweep = wrap_positive(end - start);
            if (sweep <= 1e-9) {
                continue;   // from == to is an EMPTY span, never the whole ring
            }

            Arc arc;
            arc.start = wrap_positive(start);
            arc.sweep = sweep;
            arc.lip = std::clamp(static_cast<double>(span.height), 0.0, std::max(0.0, curb_height));
            m_arcs.push_back(arc);
        }

        m_active = !m_arcs.empty();
    }

    /// There is at least one usable span
    [[nodiscard]] bool active() const { return m_active; }

    /**
     * @brief Height of the curb top above the carriageway surface at @p p
     *
     * @param p    Point on the ring's inner boundary, in 2D local metres
     * @param full The undropped curb height, CurbRingConfig::curb_height
     * @return @p full away from every drop, the span's lip inside one, and a
     *         linear ramp between the two
     */
    [[nodiscard]] double top_height(const glm::dvec2& p, double full) const {
        if (!m_active) {
            return full;
        }
        const glm::dvec2 d = p - m_center;
        const double r = glm::length(d);
        if (r <= kZeroLength) {
            return full;
        }

        const double theta = std::atan2(d.y, d.x);
        double lowest = full;
        for (const Arc& arc : m_arcs) {
            const double f = factor(arc, theta, r);
            if (f <= 0.0) {
                continue;
            }
            lowest = std::min(lowest, full + (arc.lip - full) * f);
        }
        return lowest;
    }

    /**
     * @brief Could a drop or one of its ramps fall anywhere between @p a and @p b?
     *
     * Used to decide whether a band of the ring needs resampling. Both endpoints
     * reading full height is NOT evidence that the band is clear: a 2 m drop can
     * sit entirely between two ring vertices 4 m apart. So the test is an
     * interval overlap between the band's angular span and each drop's, the
     * latter padded by the ramp on both sides, rather than a test of the two
     * endpoints.
     */
    [[nodiscard]] bool touches(const glm::dvec2& a, const glm::dvec2& b) const {
        if (!m_active) {
            return false;
        }
        const glm::dvec2 da = a - m_center;
        const glm::dvec2 db = b - m_center;
        const double ra = glm::length(da);
        const double rb = glm::length(db);
        if (ra <= kZeroLength || rb <= kZeroLength) {
            return true;    // degenerate; resampling it is cheap and always safe
        }

        const double theta_a = std::atan2(da.y, da.x);
        const double theta_b = std::atan2(db.y, db.x);

        double band_start = theta_a;
        double band_sweep = wrap_positive(theta_b - theta_a);
        if (band_sweep > kPi) {
            band_start = theta_b;
            band_sweep = kTwoPi - band_sweep;
        }
        band_start = wrap_positive(band_start);

        const double radius = 0.5 * (ra + rb);
        const double pad = (radius > kZeroLength) ? (m_ramp / radius) : kPi;

        for (const Arc& arc : m_arcs) {
            const double padded_start = wrap_positive(arc.start - pad);
            const double padded_sweep = std::min(kTwoPi, arc.sweep + 2.0 * pad);
            if (wrap_positive(padded_start - band_start) <= band_sweep + 1e-12 ||
                wrap_positive(band_start - padded_start) <= padded_sweep + 1e-12) {
                return true;
            }
        }
        return false;
    }

private:
    struct Arc {
        double start = 0.0;     ///< counter-clockwise start, [0, 2pi)
        double sweep = 0.0;     ///< counter-clockwise sweep, (0, 2pi)
        double lip = 0.0;       ///< height the curb top drops to, metres above the surface
    };

    /// 1 inside the span, 0 beyond the ramp, linear between
    [[nodiscard]] double factor(const Arc& arc, double theta, double radius) const {
        const double delta = wrap_positive(theta - arc.start);
        if (delta <= arc.sweep) {
            return 1.0;
        }
        const double past_end = delta - arc.sweep;
        const double before_start = kTwoPi - delta;
        const double gap = std::min(past_end, before_start) * radius;
        return std::clamp(1.0 - gap / m_ramp, 0.0, 1.0);
    }

    glm::dvec2 m_center{0.0};
    std::vector<Arc> m_arcs;
    double m_ramp = 1.0;
    bool m_active = false;
};

/**
 * @brief Resample a ring section wherever a kerb drop or its ramps reach it
 *
 * This is what keeps the drop from tearing. The height modulation is continuous
 * in position, but the MESH only samples it at vertex columns, and the offset
 * ring's own columns are metres apart on a straight run. Without this pass a
 * 1 m ramp would land between two columns and be drawn as a single quad falling
 * 130 mm over its whole length -- or, where a whole span fell inside one band,
 * as nothing at all.
 *
 * Only bands a drop actually reaches are split, so a junction with no crossing
 * on it keeps exactly the vertex count it had before.
 */
[[nodiscard]] Section subdivide_for_drops(const Section& section, const DropProfile& drops) {
    if (!drops.active() || section.size() < 2) {
        return section;
    }

    Section out;
    out.reserve(section.size() * 2);

    for (size_t i = 0; i + 1 < section.size(); ++i) {
        out.push_back(section[i]);

        const double len = glm::length(section[i + 1].inner - section[i].inner);
        if (len <= kDropSampleStep || !drops.touches(section[i].inner, section[i + 1].inner)) {
            continue;
        }

        const auto splits = static_cast<size_t>(std::ceil(len / kDropSampleStep));
        const size_t n = std::min(kMaxDropSplit, std::max<size_t>(splits, 2));
        for (size_t k = 1; k < n; ++k) {
            out.push_back(lerp_sample(section[i], section[i + 1],
                                      static_cast<double>(k) / static_cast<double>(n)));
        }
    }

    out.push_back(section.back());
    return out;
}

/**
 * @brief One vertex column of the ring cross-section, inboard to outboard
 *
 * Five boundaries, four bands between them, laid out in the same order and at
 * the same widths build_profile() lays an arm's own kerb out in:
 *
 * @code
 *     p[0]  junction ring,    the carriageway edge,                  at the surface
 *     p[1]  curb face bottom, p[0] + outward * apron_width,          at the surface
 *     p[2]  curb face top,    p[1] + outward * curb_face_batter,     at +curb_height
 *     p[3]  curb top outer,   p[2] + outward * curb_top_width,       at +curb_height
 *     p[4]  sidewalk outer,   on the offset ring,                    at +curb_height
 * @endcode
 *
 * The apron is what puts p[1] on the same lateral as the arms' own curb faces:
 * the junction polygon's boundary is the LANE edge, and every kerbed profile
 * carries a gutter between that edge and its curb.
 */
struct RingColumn {
    glm::dvec2 p[5]{};
    double h[5]{};
    double along = 0.0;     ///< distance walked along the section's inner boundary
};

/**
 * @brief Resolve a section's samples into cross-section columns
 *
 * The outward direction at a column is the direction from the inner point to its
 * matched outer point, which for an outward offset is the outward normal of the
 * ring. Where that degenerates -- an offset that folded back onto the ring -- the
 * local ring tangent supplies it instead, rotated to point away from the
 * junction, since the ring is counter-clockwise and its interior is on the left.
 *
 * ### Where a kerb drop lands
 *
 * @p drops modulates p[2] and p[3] -- the top of the curb face and the outer
 * edge of the curb top -- and NOTHING ELSE. p[0] and p[1] are at the carriageway
 * surface however deep the drop is, and p[4], the outer edge of the sidewalk,
 * stays at full curb height so the ring still meets the arms' sidewalks and the
 * terrain at the level it always did. The sidewalk band therefore becomes the
 * ramp, which is what a real dropped kerb does to a footway.
 *
 * The whole column takes ONE height from ONE evaluation. That is what stops a
 * drop tearing the ring: p[2] is the outboard boundary of the curb-face band and
 * the inboard boundary of the curb-top band, and p[3] likewise joins the curb
 * top to the sidewalk, so the four bands can only stay welded if they read the
 * same number.
 */
[[nodiscard]] std::vector<RingColumn> build_columns(const Section& section,
                                                    double height,
                                                    const CurbRingConfig& cfg,
                                                    const DropProfile& drops) {
    std::vector<RingColumn> columns;
    columns.reserve(section.size());

    const double apron = std::max(0.0, cfg.apron_width);
    const double batter = std::max(0.0, cfg.curb_face_batter);
    const double top_width = std::max(0.0, cfg.curb_top_width);
    const double surface = height;
    const double top = height + cfg.curb_height;

    double along = 0.0;

    for (size_t i = 0; i < section.size(); ++i) {
        const RingSample& s = section[i];

        if (i > 0) {
            along += glm::length(s.inner - section[i - 1].inner);
        }

        glm::dvec2 outward = s.outer - s.inner;
        double reach = glm::length(outward);

        if (reach > kZeroLength) {
            outward /= reach;
        } else {
            // Fall back to the ring's own outward normal. Counter-clockwise ring,
            // interior on the left, so the outward normal is the tangent rotated
            // to the RIGHT.
            const size_t prev = (i > 0) ? (i - 1) : i;
            const size_t next = (i + 1 < section.size()) ? (i + 1) : i;
            glm::dvec2 tangent = section[next].inner - section[prev].inner;
            const double tangent_len = glm::length(tangent);
            if (tangent_len > kZeroLength) {
                tangent /= tangent_len;
                outward = glm::dvec2(tangent.y, -tangent.x);
            } else {
                outward = glm::dvec2(1.0, 0.0);
            }
            reach = 0.0;
        }

        // Never let the cross-section overshoot the offset ring on a corner the
        // offset pulled in tighter than the nominal one. The apron is consumed
        // first, because it is the band that keeps the ring's curb face on the
        // same lateral as the arms' curb faces; giving it up would put the hole
        // back in the kerb.
        const double gutter = (reach > 0.0) ? std::min(apron, reach) : apron;
        const double face =
            (reach > 0.0) ? std::min(batter, std::max(0.0, reach - gutter)) : batter;
        const double flat = (reach > 0.0)
                                ? std::min(top_width, std::max(0.0, reach - gutter - face))
                                : top_width;

        RingColumn c;
        c.p[0] = s.inner;
        c.p[1] = s.inner + outward * gutter;
        c.p[2] = s.inner + outward * (gutter + face);
        c.p[3] = s.inner + outward * (gutter + face + flat);
        c.p[4] = (reach > 0.0) ? s.outer : c.p[3];
        // One evaluation, shared by every band boundary that moves.
        const double curb_rise = drops.top_height(s.inner, cfg.curb_height);

        c.h[0] = surface;
        c.h[1] = surface;
        c.h[2] = surface + curb_rise;
        c.h[3] = surface + curb_rise;
        c.h[4] = top;
        c.along = along;
        columns.push_back(c);
    }

    return columns;
}

/**
 * @brief Accumulator that turns cross-section columns into banded triangles
 *
 * Deliberately the same shape as build_corridor()'s inner loop: one winding
 * pattern for every band, per-vertex geometric normals accumulated inside a band
 * and never across one, and U constant per column from the band's NOMINAL width
 * so the texture does not shear where the offset stretches around a corner.
 */
class RingMeshBuilder {
public:
    explicit RingMeshBuilder(Mesh& mesh) : m_mesh(mesh) {}

    /**
     * @brief Sweep one band of one section
     *
     * @param columns   Cross-section columns in ring order
     * @param boundary  Index of the band's INBOARD boundary; the band spans
     *                  [boundary, boundary + 1]
     * @param material  Material slot for the band
     * @param nominal_u Nominal lateral extent of the band in metres, used for U
     *                  alone. For the curb FACE this is the curb HEIGHT, because
     *                  the plan's UV convention runs a vertical face's U up it.
     */
    void emit_band(const std::vector<RingColumn>& columns,
                   size_t boundary,
                   MaterialId material,
                   double nominal_u) {
        if (columns.size() < 2 || boundary + 1 >= 5) {
            return;
        }
        if (nominal_u <= kZeroLength) {
            return;     // a band with no extent at all, e.g. a zero-height curb
        }

        const UVTiling tiling = uv_tiling(material);
        const float u_scale = (tiling.u_metres > 0.0f) ? tiling.u_metres : 1.0f;
        const float v_scale = (tiling.v_metres > 0.0f) ? tiling.v_metres : 1.0f;

        const float u_in = 0.0f;
        const float u_out = static_cast<float>(nominal_u) / u_scale;

        const uint32_t band_start = static_cast<uint32_t>(m_mesh.indices.size());

        m_column_in.clear();
        m_column_out.clear();

        for (const RingColumn& c : columns) {
            const float v = static_cast<float>(c.along / static_cast<double>(v_scale));

            Vertex vi{};
            vi.position = to_world(c.p[boundary], c.h[boundary]);
            vi.normal = glm::vec3(0.0f, 1.0f, 0.0f);
            vi.uv = glm::vec2(u_in, v);
            vi.color = glm::vec4(1.0f);

            Vertex vo{};
            vo.position = to_world(c.p[boundary + 1], c.h[boundary + 1]);
            vo.normal = glm::vec3(0.0f, 1.0f, 0.0f);
            vo.uv = glm::vec2(u_out, v);
            vo.color = glm::vec4(1.0f);

            m_column_in.push_back(static_cast<uint32_t>(m_mesh.vertices.size()));
            m_mesh.vertices.push_back(vi);
            m_normal_accum.emplace_back(0.0);

            m_column_out.push_back(static_cast<uint32_t>(m_mesh.vertices.size()));
            m_mesh.vertices.push_back(vo);
            m_normal_accum.emplace_back(0.0);
        }

        // The ring runs counter-clockwise with the junction on its LEFT, so the
        // inboard boundary is the band's LEFT column and build_corridor()'s
        // pattern applies unchanged: it yields +Y for the two horizontal bands and,
        // for the curb face, the normal pointing away from the raised side, that
        // is INWARD towards the carriageway.
        for (size_t i = 0; i + 1 < columns.size(); ++i) {
            const uint32_t l0 = m_column_in[i];
            const uint32_t r0 = m_column_out[i];
            const uint32_t l1 = m_column_in[i + 1];
            const uint32_t r1 = m_column_out[i + 1];

            emit_triangle(l0, r0, r1);
            emit_triangle(l0, r1, l1);
        }

        const uint32_t added = static_cast<uint32_t>(m_mesh.indices.size()) - band_start;
        if (added == 0u) {
            return;
        }

        if (!m_mesh.submeshes.empty() && m_mesh.submeshes.back().material == material) {
            m_mesh.submeshes.back().index_count += added;
        } else {
            m_mesh.submeshes.push_back(SubMesh{ band_start, added, material });
        }
    }

    /// Resolve the accumulated geometric normals into the mesh
    void finish() {
        for (size_t i = 0; i < m_mesh.vertices.size() && i < m_normal_accum.size(); ++i) {
            const glm::dvec3& n = m_normal_accum[i];
            const double len_sq = glm::dot(n, n);
            if (len_sq > 0.0) {
                const glm::dvec3 unit = n / std::sqrt(len_sq);
                m_mesh.vertices[i].normal = glm::vec3(static_cast<float>(unit.x),
                                                      static_cast<float>(unit.y),
                                                      static_cast<float>(unit.z));
            }
            // else: referenced by no surviving triangle, so its +Y is never sampled
        }
    }

private:
    void emit_triangle(uint32_t i0, uint32_t i1, uint32_t i2) {
        const glm::dvec3 p0(m_mesh.vertices[i0].position);
        const glm::dvec3 p1(m_mesh.vertices[i1].position);
        const glm::dvec3 p2(m_mesh.vertices[i2].position);

        // Unnormalised, so the accumulation is area weighted.
        const glm::dvec3 face = glm::cross(p1 - p0, p2 - p0);
        if (glm::dot(face, face) <= kDegenerateCrossSq) {
            return;
        }

        m_mesh.indices.push_back(i0);
        m_mesh.indices.push_back(i1);
        m_mesh.indices.push_back(i2);

        m_normal_accum[i0] += face;
        m_normal_accum[i1] += face;
        m_normal_accum[i2] += face;
    }

    Mesh& m_mesh;
    std::vector<glm::dvec3> m_normal_accum;
    std::vector<uint32_t> m_column_in;
    std::vector<uint32_t> m_column_out;
};

} // namespace

// ============================================================================
// build_curb_ring
// ============================================================================

CurbRing build_curb_ring(const JunctionPolygon& poly,
                         const std::vector<ArmRef>& arms,
                         const std::vector<ArmEnd>& ends,
                         float height,
                         const CurbRingConfig& cfg,
                         const KerbDrops* drops) {
    CurbRing out;

    // ------------------------------------------------------------------------
    // Refusals. Every one of these leaves the ring completely empty, per the
    // header contract, so a caller can treat `valid` as the only gate.
    // ------------------------------------------------------------------------
    if (!cfg.enabled) {
        return out;
    }
    if (!poly.valid || poly.self_intersecting || poly.ring.size() < 3) {
        return out;
    }
    if (arms.size() != ends.size() || poly.arm_ring_start.size() != arms.size()) {
        spdlog::warn("build_curb_ring: {} arms, {} ends, {} ring starts; refusing the ring",
                     arms.size(), ends.size(), poly.arm_ring_start.size());
        return out;
    }
    if (arms.size() < 3) {
        return out;
    }
    if (!(cfg.ring_offset() > kZeroLength)) {
        return out;
    }
    if (!std::isfinite(height) || !ring_is_finite(poly.ring)) {
        return out;
    }

    const size_t ring_size = poly.ring.size();
    for (size_t k = 0; k < arms.size(); ++k) {
        if (!ends[k].valid) {
            return out;
        }
        if (poly.arm_ring_start[k] >= ring_size) {
            spdlog::warn("build_curb_ring: arm {} ring start {} is past the {}-vertex ring",
                         k, poly.arm_ring_start[k], ring_size);
            return out;
        }
    }

    // The whole file walks the ring counter-clockwise with the junction interior
    // on the left, and the published arm_ring_start indices only mean anything in
    // that direction. A clockwise ring cannot be repaired by reversing it, because
    // that would reverse each arm's right-then-left pair as well.
    if (signed_area(poly.ring) <= 0.0) {
        spdlog::warn("build_curb_ring: junction ring is not counter-clockwise; refusing the ring");
        return out;
    }

    // ------------------------------------------------------------------------
    // 1. Offset outward with Clipper2.
    // ------------------------------------------------------------------------
    const double scale = (cfg.clipper_scale > 0.0) ? cfg.clipper_scale : 1000.0;
    const double arc_tol = (cfg.arc_tolerance > 0.0) ? cfg.arc_tolerance : 0.02;

    std::vector<std::vector<glm::dvec2>> offset_paths;
    if (!inflate_ring(poly.ring, cfg.ring_offset(), scale, arc_tol, offset_paths)) {
        spdlog::warn("build_curb_ring: offsetting a {}-vertex junction ring by {:.2f} m "
                     "produced no usable path",
                     ring_size, cfg.sidewalk_width);
        return out;
    }

    // An outward inflate of one simple polygon yields one outer boundary, plus a
    // hole for every gap the offset closed over. A hole is strictly inside the
    // boundary, so greatest absolute area always picks the boundary, and the
    // boundary always contains the polygon it came from. Holes are dropped: they
    // are places the ring has no sidewalk to draw, and Junction::footprint is a
    // single closed ring with no hole list to put them in.
    size_t best = 0;
    double best_area = std::fabs(signed_area(offset_paths[0]));
    for (size_t i = 1; i < offset_paths.size(); ++i) {
        const double area = std::fabs(signed_area(offset_paths[i]));
        if (area > best_area) {
            best_area = area;
            best = i;
        }
    }
    if (offset_paths.size() > 1) {
        spdlog::debug("build_curb_ring: offset returned {} paths; keeping the one of area "
                      "{:.2f} m^2 and dropping {} hole(s)",
                      offset_paths.size(), best_area, offset_paths.size() - 1);
    }

    std::vector<glm::dvec2> outer = std::move(offset_paths[best]);
    if (outer.size() < 3) {
        return out;
    }
    if (signed_area(outer) < 0.0) {
        std::reverse(outer.begin(), outer.end());
    }
    if (!point_in_ring(outer, poly.centroid)) {
        // Cannot happen for an outward offset of a simple ring, so it means the
        // ring was not as simple as JunctionPolygon claimed. The geometry is still
        // emitted; the log is there so the case is visible rather than silent.
        spdlog::warn("build_curb_ring: the offset ring does not contain the junction centroid "
                     "({:.2f}, {:.2f}); the junction polygon is probably not simple",
                     poly.centroid.x, poly.centroid.y);
    }

    out.inner = poly.ring;
    out.outer = outer;

    // ------------------------------------------------------------------------
    // 2. Zip the two rings together.
    // ------------------------------------------------------------------------
    std::vector<double> cum_u;
    const double perimeter = ring_arclengths(out.inner, cum_u);
    if (perimeter <= kZeroLength) {
        return out;
    }

    const std::vector<RingSample> samples = zip_rings(out.inner, out.outer, cum_u, perimeter);
    if (samples.size() < 2) {
        return out;
    }

    // ------------------------------------------------------------------------
    // 3. Open a mouth at every arm, leaving N ring sections.
    // ------------------------------------------------------------------------
    std::vector<ArmCut> cuts;
    cuts.reserve(arms.size());
    for (const ArmEnd& end : ends) {
        cuts.push_back(cut_of(end));
    }

    const double nominal_walk = std::max(0.0, cfg.sidewalk_width);
    const double nominal_apron = std::max(0.0, cfg.apron_width);

    const DropProfile drop_profile(drops, cfg.curb_height);

    RingMeshBuilder builder(out.mesh);
    size_t drawn_sections = 0;

    for (size_t k = 0; k < arms.size(); ++k) {
        const size_t next = (k + 1) % arms.size();

        // The corner between arm k and arm k+1 spans the ring from arm k's
        // carriageway LEFT corner to arm k+1's carriageway RIGHT corner. Every
        // arm's cut face is therefore already outside the window, which is what
        // opens the mouth; clipping below only trims the round join at each end.
        const size_t from_index = (poly.arm_ring_start[k] + 1) % ring_size;
        const size_t to_index = poly.arm_ring_start[next];

        double span = cum_u[to_index] - cum_u[from_index];
        if (span < 0.0) {
            span += perimeter;
        }
        if (span <= kZeroLength) {
            continue;
        }

        const Section window = window_samples(samples, cum_u[from_index], span, perimeter);
        const Section clipped =
            clip_section(window, cuts[k], cuts[next], cfg.ring_offset());
        if (clipped.size() < 2) {
            continue;   // the two mouths met; a sliver here is worse than nothing
        }

        // clip_section() trims the round joins, where the ring's INNER boundary
        // reaches past a cut line. This pulls the OUTER boundary out of the
        // approach lanes, which at an acute fork it enters without the inner ever
        // leaving the junction.
        const Section section = clamp_out_of_lanes(clipped, cuts);
        if (section.size() < 2) {
            continue;
        }

        double section_length = 0.0;
        for (size_t i = 1; i < section.size(); ++i) {
            section_length += glm::length(section[i].inner - section[i - 1].inner);
        }
        if (section_length < kMinSectionLength) {
            continue;
        }

        // --------------------------------------------------------------------
        // 4. Sweep the cross-section along the section.
        // --------------------------------------------------------------------
        // Resample across any kerb drop that reaches this corner, so the ramp is
        // carried by real vertex columns instead of falling between two of them.
        const Section sampled = subdivide_for_drops(section, drop_profile);

        const std::vector<RingColumn> columns =
            build_columns(sampled, static_cast<double>(height), cfg, drop_profile);
        if (columns.size() < 2) {
            continue;
        }

        // Apron: the gutter band that carries the ring's boundary out to the
        // lateral the arms' curb faces actually stand on.
        builder.emit_band(columns, 0, MaterialId::Concrete, nominal_apron);
        // Curb face: U runs UP the face, so its nominal extent is the curb height.
        builder.emit_band(columns, 1, MaterialId::Curb, cfg.curb_height);
        // Curb top and sidewalk: ordinary lateral U.
        builder.emit_band(columns, 2, MaterialId::Curb, cfg.curb_top_width);
        builder.emit_band(columns, 3, MaterialId::Sidewalk, nominal_walk);

        ++drawn_sections;
    }

    builder.finish();

    if (out.mesh.indices.empty()) {
        spdlog::debug("build_curb_ring: every one of {} corners collapsed; keeping the offset "
                      "ring as a footprint and emitting no geometry",
                      arms.size());
        out.mesh.clear();
        return out;     // inner and outer survive; valid stays false
    }

    if (drawn_sections < arms.size()) {
        spdlog::debug("build_curb_ring: {} of {} corners drawn; the rest were swallowed by "
                      "their arm mouths",
                      drawn_sections, arms.size());
    }

    out.mesh.sort_submeshes_by_material();
    out.mesh.compute_bounds();
    out.mesh.compute_tangents();

    out.valid = true;
    return out;
}

} // namespace stratum::osm::road
