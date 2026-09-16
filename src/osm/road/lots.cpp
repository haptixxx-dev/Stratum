// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file lots.cpp
 * @brief Implementation of recursive OBB lot subdivision and lot identity
 *
 * Three pieces, in the order the file is laid out:
 *
 *   1. A trig-free geometry kernel -- convex hull, minimum-area OBB, and a
 *      polygon/line split that returns EVERY piece on each side. The split is the
 *      part that earns its length: a block is routinely non-convex, one cut
 *      across the arms of a U produces two pieces above the line and one below,
 *      and a Sutherland-Hodgman clip would hand back a single ring with a
 *      zero-width bridge joining the two arms instead.
 *   2. The recursion, which is short, because every hard decision is a constraint
 *      check against LotParams.
 *   3. Identity: per-node key derivation, and the mean-value-coordinate transfer
 *      that repairs ids when the tree changes shape.
 *
 * See lots.hpp for the method, the CityEngine parameter mapping, and the reason
 * determinism is stated in terms of what is ABSENT from this file -- libm calls
 * on coordinates, hash containers, and a shared random stream.
 */

#include "osm/road/lots.hpp"

#include "osm/road/blocks.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace stratum::osm::road {

namespace {

// ============================================================================
// Tolerances
// ============================================================================

/**
 * @brief Points closer than this in local metres are the same point
 *
 * A micrometre, matching blocks.cpp and RoadGraph. It has to agree with
 * blocks.cpp in particular: the ring this file cuts up is the ring blocks.cpp
 * already de-duplicated at that tolerance, so a looser value here would start
 * merging vertices the block traversal considered distinct.
 */
constexpr double kPointEpsilon = 1e-6;
constexpr double kPointEpsilonSq = kPointEpsilon * kPointEpsilon;

/// Below this many square metres a ring has no interior at all
constexpr double kAreaEpsilon = 1e-9;

/**
 * @brief A vertex within this distance of the cut line is treated as ON it
 *
 * A nanometre, three orders tighter than kPointEpsilon, and deliberately so. This
 * is not a "same place" test, it is a "which side" test, and widening it snaps
 * genuinely-off-line vertices onto the line, which turns one clean crossing into
 * a pair of on-line vertices and confuses the chain pairing. Sub-nanometre
 * slivers are dropped afterwards by the area test instead.
 */
constexpr double kOnLineEpsilon = 1e-9;

/**
 * @brief Steepest lean LotParams::irregularity can give a cut, as a slope
 *
 * A slope, not an angle, because turning an angle into a direction needs sin and
 * cos and libm is not bit-reproducible across platforms. 0.3 is about seventeen
 * degrees, which is as far as a lot boundary can lean before it stops reading as
 * a property line and starts reading as a mistake.
 */
constexpr double kMaxCutSlope = 0.3;

/**
 * @brief How far the pivot may slide along the long axis, as a fraction of it
 *
 * A quarter each way at irregularity 1, so the cut stays within the middle half
 * of the piece. Letting it reach the ends would produce a sliver and a
 * near-full-size piece, and the sliver would then be vetoed by lot_width_min,
 * which makes high irregularity silently mean "subdivide less".
 */
constexpr double kMaxPivotFraction = 0.25;

/// No such chain
constexpr size_t kNoChain = std::numeric_limits<size_t>::max();

// ============================================================================
// Deterministic Mixing
//
// SplitMix64. Integer only, so it is bit-identical on every platform, and it is
// a FINALISER rather than a generator: there is no state to advance and
// therefore no stream whose position could leak between lots.
// ============================================================================

/// SplitMix64 finaliser
[[nodiscard]] uint64_t mix64(uint64_t x) noexcept {
    x += 0x9E3779B97F4A7C15ull;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
    return x ^ (x >> 31);
}

/// Combine two 64-bit values into one, order-sensitive
[[nodiscard]] uint64_t mix2(uint64_t a, uint64_t b) noexcept {
    return mix64(a ^ mix64(b + 0x165667B19E3779F9ull));
}

/**
 * @brief A value in [0, 1) from a hash
 *
 * 53 bits scaled by an exact power of two, so the conversion itself introduces no
 * rounding and the result is identical on every platform. The upper bound is
 * strict, which force_street_access relies on: at 1.0 the comparison
 * `draw < force` is always true and the constraint is absolute.
 */
[[nodiscard]] double unit_from(uint64_t hash) noexcept {
    return static_cast<double>(hash >> 11) * 0x1.0p-53;
}

/// A value in [-1, 1) from a hash
[[nodiscard]] double signed_unit_from(uint64_t hash) noexcept {
    return unit_from(hash) * 2.0 - 1.0;
}

/**
 * @brief Salts, so two draws at one node cannot collide
 *
 * Every draw in the recursion is `unit_from(mix2(node_key, salt))`. The node key
 * is the only state, which is the whole point: a draw depends on WHERE a node is
 * in the tree and on nothing that happened before it.
 */
enum : uint64_t {
    kSaltPivot = 0x01,
    kSaltSlope = 0x02,
    kSaltAccess = 0x03,
    kSaltCorner = 0x04,
    kSaltChild = 0x05,
    kSaltRoot = 0x06,
    kSaltId = 0x07,
};

// ============================================================================
// Geometry Helpers
// ============================================================================

[[nodiscard]] inline double cross2(const glm::dvec2& a, const glm::dvec2& b) noexcept {
    return a.x * b.y - a.y * b.x;
}

/// Rotate a quarter turn anticlockwise
[[nodiscard]] inline glm::dvec2 perp(const glm::dvec2& v) noexcept {
    return glm::dvec2{-v.y, v.x};
}

[[nodiscard]] inline double length2(const glm::dvec2& v) noexcept {
    return v.x * v.x + v.y * v.y;
}

[[nodiscard]] inline double length_of(const glm::dvec2& v) noexcept {
    return std::sqrt(length2(v));
}

/// True when two local-metre points coincide within kPointEpsilon
[[nodiscard]] inline bool same_point(const glm::dvec2& a, const glm::dvec2& b) noexcept {
    return length2(a - b) <= kPointEpsilonSq;
}

/**
 * @brief Unit vector, or (0,0) when the input is too short to have a direction
 *
 * glm::normalize divides by zero on a zero vector and hands back NaN, which then
 * propagates silently into every coordinate downstream. Returning zero lets the
 * caller test and skip.
 */
[[nodiscard]] glm::dvec2 safe_normalize(const glm::dvec2& v) noexcept {
    const double len = length_of(v);
    if (len <= kPointEpsilon) {
        return glm::dvec2{0.0};
    }
    return v / len;
}

/// Area centroid of a closed ring, first point not repeated
[[nodiscard]] glm::dvec2 ring_centroid(const std::vector<glm::dvec2>& ring) {
    if (ring.empty()) {
        return glm::dvec2{0.0};
    }

    double twice_area = 0.0;
    glm::dvec2 acc{0.0};
    for (size_t i = 0; i < ring.size(); ++i) {
        const glm::dvec2& a = ring[i];
        const glm::dvec2& b = ring[(i + 1) % ring.size()];
        const double c = a.x * b.y - b.x * a.y;
        twice_area += c;
        acc += (a + b) * c;
    }

    if (std::fabs(twice_area) <= kAreaEpsilon) {
        // A degenerate ring has no area centroid; fall back to the vertex mean so
        // callers still get a point inside the hull rather than a NaN.
        glm::dvec2 mean{0.0};
        for (const glm::dvec2& p : ring) {
            mean += p;
        }
        return mean / static_cast<double>(ring.size());
    }

    return acc / (3.0 * twice_area);
}

/// Crossing-number point-in-polygon. Boundary cases are not distinguished.
[[nodiscard]] bool point_in_ring(const std::vector<glm::dvec2>& ring, const glm::dvec2& p) {
    if (ring.size() < 3) {
        return false;
    }

    bool inside = false;
    for (size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
        const glm::dvec2& a = ring[i];
        const glm::dvec2& b = ring[j];
        if (((a.y > p.y) != (b.y > p.y)) &&
            (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)) {
            inside = !inside;
        }
    }
    return inside;
}

/**
 * @brief Convex hull by Andrew's monotone chain, anticlockwise
 *
 * Sorted lexicographically, so the hull starts at the lexicographically smallest
 * point whatever rotation the input ring happens to be in. That canonical start
 * is load-bearing: it is what makes the minimum-area OBB -- and therefore every
 * cut -- independent of where the block traversal began its walk.
 *
 * Collinear points are dropped, which halves the work the OBB does and removes a
 * class of tie between two hull edges of identical direction.
 */
[[nodiscard]] std::vector<glm::dvec2> convex_hull(std::vector<glm::dvec2> points) {
    std::sort(points.begin(), points.end(), [](const glm::dvec2& a, const glm::dvec2& b) {
        if (a.x != b.x) {
            return a.x < b.x;
        }
        return a.y < b.y;
    });
    points.erase(std::unique(points.begin(), points.end(),
                             [](const glm::dvec2& a, const glm::dvec2& b) {
                                 return same_point(a, b);
                             }),
                 points.end());

    if (points.size() < 3) {
        return points;
    }

    std::vector<glm::dvec2> hull(points.size() * 2);
    size_t k = 0;

    for (size_t i = 0; i < points.size(); ++i) {
        while (k >= 2 && cross2(hull[k - 1] - hull[k - 2], points[i] - hull[k - 2]) <= 0.0) {
            --k;
        }
        hull[k++] = points[i];
    }

    const size_t lower = k + 1;
    for (size_t i = points.size() - 1; i > 0; --i) {
        const glm::dvec2& p = points[i - 1];
        while (k >= lower && cross2(hull[k - 1] - hull[k - 2], p - hull[k - 2]) <= 0.0) {
            --k;
        }
        hull[k++] = p;
    }

    hull.resize(k > 0 ? k - 1 : 0);
    return hull;
}

/**
 * @brief The canonical representative of a direction, ignoring which way it points
 *
 * An OBB axis is a direction MOD PI: (1, 0) and (-1, 0) describe the same box. But
 * every side label and every ordering downstream is derived from the axis, so if
 * the sign is allowed to float, a tiny nudge to a block that made a different hull
 * edge win the minimum-area contest reverses the axis, swaps which child is
 * "first", and every lot id below moves to the other half of the block. That is
 * not hypothetical: a 140 x 100 block whose top-right corner moves four metres
 * does exactly this, and the test that perturbs a block and checks the ids landed
 * on the SAME LAND is what found it.
 *
 * Pinning the sign to the dominant component makes a near-horizontal axis always
 * point right and a near-vertical one always point up, which covers the
 * perturbations that actually happen.
 *
 * It cannot cover all of them, and that is a fact about the circle rather than
 * about this code: a direction mod pi lives on a circle whose double cover has no
 * continuous section, so SOME orientation has to flip. This rule puts the flip at
 * 45 degrees, as far from the common cases as it can be, and transfer_lot_ids()
 * is the backstop for a block that sits on it.
 */
[[nodiscard]] glm::dvec2 canonical_direction(const glm::dvec2& v) noexcept {
    const double ax = std::fabs(v.x);
    const double ay = std::fabs(v.y);

    bool flip = false;
    if (ax > ay) {
        flip = v.x < 0.0;
    } else if (ay > ax) {
        flip = v.y < 0.0;
    } else {
        flip = (v.x < 0.0) || (v.x == 0.0 && v.y < 0.0);
    }

    return flip ? -v : v;
}

/**
 * @brief A minimum-area oriented bounding box
 */
struct Obb {
    /// Unit vector along the LONGEST side of the box
    glm::dvec2 axis{1.0, 0.0};

    /// Centre of the box in local metres
    glm::dvec2 center{0.0};

    double long_extent = 0.0;   ///< Length of the longest side
    double short_extent = 0.0;  ///< Length of the shortest side
};

/**
 * @brief Minimum-area oriented bounding box of a ring
 *
 * Rotating calipers, in the naive O(h^2) form: the minimum-area box of a convex
 * polygon has a side flush with a hull edge, so trying every hull edge finds it.
 * Hull sizes here are single digits to low tens, and the alternative -- an
 * incremental caliper walk -- gains nothing and gets the wrap-around wrong in
 * exactly the degenerate cases a block ring produces.
 *
 * Ties are broken by keeping the FIRST box found, with the hull in its canonical
 * lexicographic order, so a square gets a repeatable axis instead of whichever
 * of its four equally good boxes floating-point noise happened to favour.
 *
 * @param ring Ring in local metres
 * @param out  Box, written only on success
 * @return false when the ring has no hull with area
 */
[[nodiscard]] bool min_area_obb(const std::vector<glm::dvec2>& ring, Obb& out) {
    const std::vector<glm::dvec2> hull = convex_hull(ring);
    if (hull.size() < 3) {
        return false;
    }

    bool found = false;
    double best_area = 0.0;

    for (size_t i = 0; i < hull.size(); ++i) {
        const glm::dvec2 edge = hull[(i + 1) % hull.size()] - hull[i];
        const glm::dvec2 u = safe_normalize(edge);
        if (u == glm::dvec2{0.0}) {
            continue;
        }
        const glm::dvec2 v = perp(u);

        double min_u = std::numeric_limits<double>::max();
        double max_u = -std::numeric_limits<double>::max();
        double min_v = std::numeric_limits<double>::max();
        double max_v = -std::numeric_limits<double>::max();

        for (const glm::dvec2& p : hull) {
            const double pu = glm::dot(p, u);
            const double pv = glm::dot(p, v);
            min_u = std::min(min_u, pu);
            max_u = std::max(max_u, pu);
            min_v = std::min(min_v, pv);
            max_v = std::max(max_v, pv);
        }

        const double width = max_u - min_u;
        const double height = max_v - min_v;
        const double area = width * height;

        if (found && !(area < best_area)) {
            continue;
        }

        found = true;
        best_area = area;
        out.center = u * (0.5 * (min_u + max_u)) + v * (0.5 * (min_v + max_v));
        // The tie at width == height goes to u, which is the hull edge direction.
        // Any consistent rule works; this one is stated here so the square case
        // has a documented answer rather than an emergent one.
        // Canonicalised, or the sign of the axis -- and with it the order the
        // children are emitted in -- depends on which hull edge happened to win.
        out.axis = canonical_direction((width >= height) ? u : v);
        out.long_extent = std::max(width, height);
        out.short_extent = std::min(width, height);
    }

    return found && out.long_extent > kPointEpsilon;
}

// ============================================================================
// Pieces
// ============================================================================

/**
 * @brief A candidate lot: a ring plus the block edge each segment came from
 *
 * `tags[i]` covers the segment from `points[i]` to `points[i + 1]`, wrapping.
 * Exactly the layout of Lot::ring / Lot::ring_edges, so emitting a piece is a
 * move rather than a conversion.
 */
struct Piece {
    std::vector<glm::dvec2> points;
    std::vector<uint32_t> tags;
};

/// Total length of the segments that came off the block boundary
[[nodiscard]] double frontage_of(const Piece& piece) {
    double total = 0.0;
    for (size_t i = 0; i < piece.points.size(); ++i) {
        if (piece.tags[i] == kNoBlockEdge) {
            continue;
        }
        total += length_of(piece.points[(i + 1) % piece.points.size()] - piece.points[i]);
    }
    return total;
}

/**
 * @brief Drop consecutive duplicate points, keeping the LATER segment tag
 *
 * A cut joins two boundary chains at a point they share, so the concatenation
 * names that point twice: once ending the first chain, once starting the second.
 * The segment that actually leaves the surviving vertex is the second one, so its
 * tag is the one to keep -- the same rule blocks.cpp uses when it prunes an
 * antenna, and for the same reason.
 */
void dedupe(Piece& piece) {
    Piece out;
    out.points.reserve(piece.points.size());
    out.tags.reserve(piece.tags.size());

    for (size_t i = 0; i < piece.points.size(); ++i) {
        if (!out.points.empty() && same_point(out.points.back(), piece.points[i])) {
            out.tags.back() = piece.tags[i];
            continue;
        }
        out.points.push_back(piece.points[i]);
        out.tags.push_back(piece.tags[i]);
    }

    // The ring is cyclic, so the seam needs the same treatment. Dropping the last
    // point keeps the FIRST point's tag, which is right: the segment leaving that
    // position is the one going to points[1], and the one being dropped had zero
    // length.
    while (out.points.size() >= 2 && same_point(out.points.front(), out.points.back())) {
        out.points.pop_back();
        out.tags.pop_back();
    }

    piece = std::move(out);
}

/**
 * @brief One boundary chain of a split: from the cut line, round, and back to it
 */
struct Chain {
    std::vector<glm::dvec2> points;
    std::vector<uint32_t> tags;
    double t_start = 0.0;   ///< Position of points.front() along the cut line
    double t_end = 0.0;     ///< Position of points.back() along the cut line
};

/// A ring vertex with its side of the cut line
struct AugVertex {
    glm::dvec2 point{0.0};
    uint32_t tag = kNoBlockEdge;
    double side = 0.0;  ///< Signed distance to the cut line, exactly 0 when on it
};

/**
 * @brief Close a side's chains into rings with segments along the cut line
 *
 * The one piece of real reasoning in the split. A chain leaves the cut line, runs
 * round the boundary, and comes back to the line somewhere else; the ring is
 * closed by running ALONG the line to the start of another chain.
 *
 * Which way along the line is fixed by orientation and is not a choice. An
 * anticlockwise ring has its interior on the left. On the positive side of the
 * line -- positive meaning the side the normal perp(u) points at -- the interior
 * is at perp(u) from the cut segment, and the direction whose left is perp(u) is
 * +u. So a positive-side ring runs along the line in +u and a negative-side ring
 * in -u. Reverse either and the pieces come out inside-out, which shows up as a
 * negative area and would then be silently discarded.
 *
 * @param chains Chains of one side, in ring order
 * @param side   +1 or -1, the side these chains bound
 * @param out    Rings, appended
 * @return false when the chains could not be paired into rings
 */
[[nodiscard]] bool close_chains(const std::vector<Chain>& chains, double side,
                                std::vector<Piece>& out) {
    const size_t count = chains.size();
    if (count == 0) {
        return false;
    }
    const size_t before = out.size();

    std::vector<size_t> next(count, kNoChain);
    std::vector<bool> targeted(count, false);

    for (size_t c = 0; c < count; ++c) {
        const double from_t = chains[c].t_end;

        size_t best = kNoChain;
        double best_gap = 0.0;

        for (size_t o = 0; o < count; ++o) {
            if (targeted[o]) {
                continue;
            }
            const double gap = (chains[o].t_start - from_t) * side;
            if (gap < 0.0) {
                continue;
            }
            if (best == kNoChain || gap < best_gap) {
                best = o;
                best_gap = gap;
            }
        }

        if (best == kNoChain) {
            // Nothing lies ahead along the line. Real geometry should not reach
            // here -- crossings alternate -- but a near-tangent cut can round a
            // gap to the wrong side of zero. Taking the nearest remaining start
            // keeps next[] a permutation, so the cycle walk below still
            // terminates; a piece that comes out malformed is caught by the area
            // and orientation tests and counted, not silently kept.
            for (size_t o = 0; o < count; ++o) {
                if (targeted[o]) {
                    continue;
                }
                const double gap = std::fabs(chains[o].t_start - from_t);
                if (best == kNoChain || gap < best_gap) {
                    best = o;
                    best_gap = gap;
                }
            }
        }

        if (best == kNoChain) {
            return false;
        }
        next[c] = best;
        targeted[best] = true;
    }

    std::vector<bool> visited(count, false);
    for (size_t c = 0; c < count; ++c) {
        if (visited[c]) {
            continue;
        }

        Piece piece;
        size_t cur = c;
        while (!visited[cur]) {
            visited[cur] = true;
            const Chain& chain = chains[cur];
            for (size_t i = 0; i < chain.points.size(); ++i) {
                piece.points.push_back(chain.points[i]);
                piece.tags.push_back(chain.tags[i]);
            }
            cur = next[cur];
        }

        dedupe(piece);
        if (piece.points.size() >= 3) {
            out.push_back(std::move(piece));
        }
    }

    return out.size() > before;
}

/**
 * @brief Cut a piece with an infinite line, returning EVERY resulting ring
 *
 * Not a half-plane clip. A non-convex piece cut by one line can produce several
 * rings on the same side -- a cut across the arms of a U-shaped block gives two
 * above and one below -- and each of those is a separate parcel of land that must
 * subdivide on its own. A Sutherland-Hodgman clip would return the two arms as
 * one ring joined by a zero-width bridge along the cut, and everything downstream
 * of that, from the area test to the OBB, would be wrong.
 *
 * Segment tags survive the cut: a boundary segment that the line crosses gives
 * both halves its own tag, and the segments along the cut are kNoBlockEdge, which
 * is what makes an interior lot recognisable as landlocked.
 *
 * @param in      Piece to cut
 * @param pivot   A point on the line
 * @param dir     Direction of the line, need not be unit
 * @param out_pos Rings on the perp(dir) side, appended
 * @param out_neg Rings on the other side, appended
 * @return false when the line does not actually divide the piece
 */
[[nodiscard]] bool split_piece(const Piece& in, const glm::dvec2& pivot, const glm::dvec2& dir,
                               std::vector<Piece>& out_pos, std::vector<Piece>& out_neg) {
    const size_t count = in.points.size();
    if (count < 3) {
        return false;
    }

    const glm::dvec2 u = safe_normalize(dir);
    if (u == glm::dvec2{0.0}) {
        return false;
    }
    const glm::dvec2 n = perp(u);

    std::vector<double> dist(count, 0.0);
    bool any_pos = false;
    bool any_neg = false;
    for (size_t i = 0; i < count; ++i) {
        const double d = glm::dot(in.points[i] - pivot, n);
        dist[i] = (std::fabs(d) <= kOnLineEpsilon) ? 0.0 : d;
        any_pos = any_pos || dist[i] > 0.0;
        any_neg = any_neg || dist[i] < 0.0;
    }

    if (!any_pos || !any_neg) {
        return false;   // everything on one side, or degenerate against the line
    }

    std::vector<AugVertex> aug;
    aug.reserve(count * 2);
    for (size_t i = 0; i < count; ++i) {
        aug.push_back(AugVertex{in.points[i], in.tags[i], dist[i]});

        const size_t j = (i + 1) % count;
        const bool crosses = (dist[i] > 0.0 && dist[j] < 0.0) || (dist[i] < 0.0 && dist[j] > 0.0);
        if (!crosses) {
            continue;
        }

        // Parameterised on the DISTANCES, not on a line/line intersection, so the
        // crossing point is exactly reproducible and lands on the segment even
        // when the segment is almost parallel to the cut.
        const double t = dist[i] / (dist[i] - dist[j]);
        const glm::dvec2 hit = in.points[i] + (in.points[j] - in.points[i]) * t;

        // The crossing inherits the tag of the segment it splits: both halves of a
        // street-facing segment still face that street.
        aug.push_back(AugVertex{hit, in.tags[i], 0.0});
    }

    const size_t m = aug.size();
    size_t first_zero = m;
    for (size_t i = 0; i < m; ++i) {
        if (aug[i].side == 0.0) {
            first_zero = i;
            break;
        }
    }
    if (first_zero == m) {
        return false;
    }

    std::vector<Chain> pos_chains;
    std::vector<Chain> neg_chains;

    size_t k = first_zero;
    do {
        std::vector<size_t> interior;
        size_t j = (k + 1) % m;
        while (aug[j].side != 0.0) {
            interior.push_back(j);
            j = (j + 1) % m;
        }

        // Two on-line vertices with nothing between them: a boundary edge lying
        // ALONG the cut. It bounds no interior, so it makes no ordinary chain --
        // but it is still a street, and close_chains() would otherwise lay a
        // synthetic kNoBlockEdge segment over its span and destroy the tag. A lot
        // that genuinely fronts a street there would then report
        // frontage_length 0 on that side, C4 would classify it as landlocked, and
        // nothing else would move: the area still balances and dropped_pieces
        // stays 0. The 100 x 200 block with a notch cut to the OBB's own cut line
        // loses exactly the 20 m notch floor this way.
        //
        // So record it as a two-point chain instead. It carries the tag and is
        // threaded into the cycle like any other chain, which puts the real street
        // segment into the ring in place of that stretch of synthetic cut.
        if (interior.empty()) {
            const glm::dvec2 span = aug[j].point - aug[k].point;
            const double along = glm::dot(span, u);

            // Which side gets it is not a choice. The input ring is anticlockwise,
            // so its interior is on the LEFT of the direction of travel: running
            // along +u the interior is at perp(u), which is the positive side, and
            // running along -u it is the negative side. Put it on the wrong side
            // and the piece comes back inside-out.
            if (aug[k].tag != kNoBlockEdge && std::fabs(along) > kPointEpsilon) {
                Chain edge_chain;
                edge_chain.points = {aug[k].point, aug[j].point};
                edge_chain.tags = {aug[k].tag, kNoBlockEdge};
                edge_chain.t_start = glm::dot(aug[k].point - pivot, u);
                edge_chain.t_end = glm::dot(aug[j].point - pivot, u);

                if (along > 0.0) {
                    pos_chains.push_back(std::move(edge_chain));
                } else {
                    neg_chains.push_back(std::move(edge_chain));
                }
            }
        } else {
            Chain chain;
            chain.points.reserve(interior.size() + 2);
            chain.tags.reserve(interior.size() + 2);

            chain.points.push_back(aug[k].point);
            chain.tags.push_back(aug[k].tag);
            for (const size_t idx : interior) {
                chain.points.push_back(aug[idx].point);
                chain.tags.push_back(aug[idx].tag);
            }
            chain.points.push_back(aug[j].point);
            // The segment leaving the chain's last vertex is the cut itself.
            chain.tags.push_back(kNoBlockEdge);

            chain.t_start = glm::dot(aug[k].point - pivot, u);
            chain.t_end = glm::dot(aug[j].point - pivot, u);

            // Every interior vertex of a chain is on one side: a sign change
            // between two on-line vertices would have inserted a crossing, and a
            // crossing is an on-line vertex.
            if (aug[interior.front()].side > 0.0) {
                pos_chains.push_back(std::move(chain));
            } else {
                neg_chains.push_back(std::move(chain));
            }
        }

        k = j;
    } while (k != first_zero);

    const bool pos_ok = close_chains(pos_chains, 1.0, out_pos);
    const bool neg_ok = close_chains(neg_chains, -1.0, out_neg);
    return pos_ok && neg_ok;
}

// ============================================================================
// Recursion
// ============================================================================

/// A node of the subdivision tree, with the key every draw at it comes from
struct Node {
    Piece piece;
    uint64_t key = 0;
    uint32_t depth = 0;
};

/// A candidate piece plus the ordering keys that make emission deterministic
struct OrderedPiece {
    Piece piece;
    double side = 0.0;   ///< +1 above the cut line, -1 below
    double along = 0.0;  ///< Centroid's position along the cut line
    double area = 0.0;
    double min_extent = 0.0;
    double frontage = 0.0;
};

/**
 * @brief Measure a cut's pieces and sort them into a reproducible order
 *
 * Ordered by side of the cut first, then by position ALONG the cut. Both are
 * distances from the cut line's own frame, and that is what makes them stable:
 * ordering by, say, centroid x would flip two stacked pieces the moment a nudge
 * to the block made the upper one's centroid a millimetre further left, and every
 * id beneath them would change with it. Side and along-position are separated by
 * metres, not millimetres.
 */
void order_pieces(std::vector<Piece>&& pieces, double side, const glm::dvec2& pivot,
                  const glm::dvec2& u, std::vector<OrderedPiece>& out, LotStats& stats) {
    for (Piece& piece : pieces) {
        const double area = signed_ring_area(piece.points);
        if (area <= kAreaEpsilon) {
            // Either a sliver with no interior, or -- if it is negative -- a ring
            // that came out clockwise, which means the chain pairing went wrong.
            // Counted rather than ignored: a caller that sees dropped_pieces rise
            // knows land went missing, and the area-conservation test fails.
            ++stats.dropped_pieces;
            continue;
        }

        OrderedPiece op;
        const glm::dvec2 centroid = ring_centroid(piece.points);
        op.side = side;
        op.along = glm::dot(centroid - pivot, u);
        op.area = area;
        op.frontage = frontage_of(piece);

        Obb box;
        op.min_extent = min_area_obb(piece.points, box) ? box.short_extent : 0.0;

        op.piece = std::move(piece);
        out.push_back(std::move(op));
    }
}

/// Emit one node as a lot
void emit_lot(const Node& node, bool is_corner, LotSubdivision& result) {
    Lot lot;
    lot.node_key = node.key;
    lot.id = lot_id_for_key(node.key);
    lot.area = signed_ring_area(node.piece.points);
    lot.frontage_length = frontage_of(node.piece);
    lot.depth = node.depth;
    lot.is_corner_lot = is_corner;
    lot.ring = node.piece.points;
    lot.ring_edges = node.piece.tags;

    result.stats.max_depth_reached = std::max(result.stats.max_depth_reached, node.depth);
    if (is_corner) {
        ++result.stats.corner_lots;
    }
    result.lots.push_back(std::move(lot));
}

/**
 * @brief Try one candidate cut and validate its pieces against the constraints
 *
 * @return true when the cut is acceptable; @p accepted then holds its pieces in
 *         emission order
 */
[[nodiscard]] bool try_cut(const Node& node, const glm::dvec2& pivot, const glm::dvec2& u,
                           const LotParams& params, std::vector<OrderedPiece>& accepted,
                           LotStats& stats) {
    std::vector<Piece> pos;
    std::vector<Piece> neg;
    if (!split_piece(node.piece, pivot, u, pos, neg)) {
        ++stats.rejected_degenerate;
        return false;
    }

    std::vector<OrderedPiece> pieces;
    order_pieces(std::move(pos), 1.0, pivot, u, pieces, stats);
    order_pieces(std::move(neg), -1.0, pivot, u, pieces, stats);

    if (pieces.size() < 2) {
        ++stats.rejected_degenerate;
        return false;
    }

    for (const OrderedPiece& piece : pieces) {
        if (piece.area < params.lot_area_min) {
            ++stats.rejected_area;
            return false;
        }
    }
    for (const OrderedPiece& piece : pieces) {
        if (piece.min_extent < params.lot_width_min) {
            ++stats.rejected_width;
            return false;
        }
    }

    if (params.force_street_access > 0.0) {
        bool landlocked = false;
        for (const OrderedPiece& piece : pieces) {
            landlocked = landlocked || piece.frontage <= 0.0;
        }
        if (landlocked) {
            // One draw per NODE, not per candidate cut, so the fallback cut is
            // judged by the same coin as the primary. Drawing again would let a
            // node that refused the primary accept the fallback purely because the
            // second draw came out differently, and the meaning of the parameter
            // would drift from "probability a lot keeps street access" to
            // something no one could state.
            const double draw = unit_from(mix2(node.key, kSaltAccess));
            if (draw < params.force_street_access) {
                ++stats.rejected_access;
                return false;
            }
        }
    }

    std::stable_sort(pieces.begin(), pieces.end(),
                     [](const OrderedPiece& a, const OrderedPiece& b) {
                         if (a.side != b.side) {
                             return a.side > b.side;
                         }
                         return a.along < b.along;
                     });

    accepted = std::move(pieces);
    return true;
}

/**
 * @brief Subdivide one node, appending its lots
 *
 * Iterative rather than recursive over an explicit stack: the depth ceiling makes
 * genuine recursion safe, but the stack also makes the emission order explicit
 * instead of implicit in the call order, and the order is part of the contract.
 */
void subdivide_node(Node root, const LotParams& params, LotSubdivision& result) {
    std::vector<Node> stack;
    stack.push_back(std::move(root));

    while (!stack.empty()) {
        Node node = std::move(stack.back());
        stack.pop_back();

        const double area = signed_ring_area(node.piece.points);
        if (node.piece.points.size() < 3 || area <= kAreaEpsilon) {
            ++result.stats.dropped_pieces;
            continue;
        }

        if (area <= params.lot_area_max) {
            emit_lot(node, false, result);
            continue;
        }

        if (node.depth >= params.max_depth) {
            ++result.stats.depth_limit_hits;
            emit_lot(node, false, result);
            continue;
        }

        Obb box;
        if (!min_area_obb(node.piece.points, box)) {
            emit_lot(node, false, result);
            continue;
        }

        ++result.stats.split_attempts;

        // The pivot is the midpoint of the largest OBB edge. Both of the two
        // largest edges have their midpoint on the same line through the box
        // centre, so the centre IS that pivot and there is no side to choose.
        glm::dvec2 pivot = box.center;
        glm::dvec2 cut_dir = perp(box.axis);   // the direction of the smallest edge

        if (params.irregularity > 0.0) {
            const double slide = signed_unit_from(mix2(node.key, kSaltPivot)) *
                                 kMaxPivotFraction * params.irregularity * box.long_extent;
            pivot = box.center + box.axis * slide;

            // Leaning by a SLOPE keeps the whole thing in +,-,*,/ and sqrt. A
            // rotation by an angle would need sin and cos, and two platforms'
            // libm would then disagree about the city.
            const double slope = signed_unit_from(mix2(node.key, kSaltSlope)) *
                                 kMaxCutSlope * params.irregularity;
            cut_dir = safe_normalize(cut_dir + box.axis * slope);
            if (cut_dir == glm::dvec2{0.0}) {
                cut_dir = perp(box.axis);
            }
        }

        std::vector<OrderedPiece> pieces;
        bool split = try_cut(node, pivot, cut_dir, params, pieces, result.stats);

        if (!split) {
            // The fallback: cut ALONG the long axis instead of across it. It is
            // the shallower cut, which is exactly what rescues a piece whose
            // across-cut would have put one half out of reach of the street, and
            // it turns a square into two street-facing strips rather than into one
            // front lot and one landlocked back lot.
            split = try_cut(node, box.center, box.axis, params, pieces, result.stats);
        }

        if (!split) {
            emit_lot(node, false, result);
            continue;
        }

        ++result.stats.splits;

        // Pushed in reverse so the stack pops them in the sorted order, which is
        // the order the lots come out in.
        for (size_t i = pieces.size(); i > 0; --i) {
            Node child;
            child.piece = std::move(pieces[i - 1].piece);
            child.depth = node.depth + 1;
            // The child's key comes from the PARENT's key and the child's index,
            // and from nothing else. No counter, no shared stream, no dependence
            // on how many lots have already been cut anywhere in the city.
            child.key = mix2(node.key, mix2(kSaltChild, static_cast<uint64_t>(i - 1)));
            stack.push_back(std::move(child));
        }
    }
}

// ============================================================================
// Corner Lots
// ============================================================================

/**
 * @brief Slice a wedge off every block corner sharper than the threshold
 *
 * Run once on the block, before the recursion, and never inside it: a corner is a
 * property of the block's street boundary, and re-testing it at every level would
 * shave the same corner repeatedly.
 *
 * Corners are ranked from the ring's lexicographically smallest vertex rather
 * than from vertex 0. Vertex 0 is wherever the face traversal in blocks.cpp began
 * its walk, which is an artefact of edge ordering; the lexicographic start is a
 * property of the shape. Keying corner lots off the shape is what lets their ids
 * survive an unrelated change upstream that rotated the ring.
 *
 * @param piece   Block ring, modified in place to the remainder
 * @param params  corner_angle_max_deg and corner_width
 * @param root_key Key of the block's root node
 * @param result  Corner lots are appended here
 */
void cut_corner_lots(Piece& piece, const LotParams& params, uint64_t root_key,
                     LotSubdivision& result) {
    if (params.corner_angle_max_deg <= 0.0 || params.corner_width <= 0.0) {
        return;
    }

    const size_t count = piece.points.size();
    if (count < 4) {
        // A triangle has nothing left once a corner comes off it.
        return;
    }

    // The ONLY libm call in this file, and it is applied to a configuration value
    // rather than to a coordinate. Comparing cosines instead of angles keeps acos
    // out of the per-corner loop, so a differing libm can at worst disagree about
    // a corner sitting exactly on the threshold -- it can never move a vertex.
    const double cos_threshold = std::cos(params.corner_angle_max_deg * 3.14159265358979323846 / 180.0);

    size_t start = 0;
    for (size_t i = 1; i < count; ++i) {
        const glm::dvec2& p = piece.points[i];
        const glm::dvec2& best = piece.points[start];
        if (p.x < best.x || (p.x == best.x && p.y < best.y)) {
            start = i;
        }
    }

    std::vector<uint64_t> corner_rank(count, 0);
    std::vector<bool> is_corner(count, false);
    uint64_t rank = 0;

    for (size_t o = 0; o < count; ++o) {
        const size_t i = (start + o) % count;
        const size_t prev = (i + count - 1) % count;
        const size_t next = (i + 1) % count;

        // Both sides must front a street. A wedge between two interior cuts is not
        // a block corner, and at the root there are no interior cuts yet, so this
        // is a guard for a caller that hands in a partly-cut ring.
        if (piece.tags[prev] == kNoBlockEdge || piece.tags[i] == kNoBlockEdge) {
            continue;
        }

        const glm::dvec2 a = piece.points[prev] - piece.points[i];
        const glm::dvec2 b = piece.points[next] - piece.points[i];
        const double la = length_of(a);
        const double lb = length_of(b);
        if (la <= kPointEpsilon || lb <= kPointEpsilon) {
            continue;
        }

        // Reflex corners have no wedge to take off: the "corner" points into the
        // block, and slicing it would cut a notch out of the middle of an edge.
        // The ring is anticlockwise, so convex is a positive turn.
        if (cross2(piece.points[i] - piece.points[prev], b) <= 0.0) {
            continue;
        }

        const double cos_interior = glm::dot(a, b) / (la * lb);
        if (cos_interior < cos_threshold) {
            continue;
        }

        is_corner[i] = true;
        corner_rank[i] = rank++;
    }

    if (rank == 0) {
        return;
    }

    Piece remainder;
    remainder.points.reserve(count + static_cast<size_t>(rank));
    remainder.tags.reserve(count + static_cast<size_t>(rank));

    for (size_t i = 0; i < count; ++i) {
        if (!is_corner[i]) {
            remainder.points.push_back(piece.points[i]);
            remainder.tags.push_back(piece.tags[i]);
            continue;
        }

        const size_t prev = (i + count - 1) % count;
        const size_t next = (i + 1) % count;
        const glm::dvec2 a = piece.points[prev] - piece.points[i];
        const glm::dvec2 b = piece.points[next] - piece.points[i];

        // Clamped to half of each side so two sharp corners at the ends of one
        // short edge cannot cut overlapping wedges out of it.
        const double reach_in = std::min(params.corner_width, 0.5 * length_of(a));
        const double reach_out = std::min(params.corner_width, 0.5 * length_of(b));

        const glm::dvec2 p_in = piece.points[i] + safe_normalize(a) * reach_in;
        const glm::dvec2 p_out = piece.points[i] + safe_normalize(b) * reach_out;

        Node corner;
        corner.key = mix2(root_key, mix2(kSaltCorner, corner_rank[i]));
        corner.depth = 0;
        corner.piece.points = {p_in, piece.points[i], p_out};
        // p_in -> corner lies on the incoming street, corner -> p_out on the
        // outgoing one, and p_out -> p_in is the chord this file cut.
        corner.piece.tags = {piece.tags[prev], piece.tags[i], kNoBlockEdge};

        if (signed_ring_area(corner.piece.points) <= kAreaEpsilon) {
            remainder.points.push_back(piece.points[i]);
            remainder.tags.push_back(piece.tags[i]);
            continue;
        }

        emit_lot(corner, true, result);

        // The remainder loses the corner vertex and gains the two ends of the
        // chord. The segment arriving at p_in still belongs to the incoming
        // street, which is already recorded on `prev`; p_in leaves along the
        // chord, and p_out leaves along the outgoing street.
        remainder.points.push_back(p_in);
        remainder.tags.push_back(kNoBlockEdge);
        remainder.points.push_back(p_out);
        remainder.tags.push_back(piece.tags[i]);
    }

    dedupe(remainder);
    piece = std::move(remainder);
}

} // namespace

// ============================================================================
// Keys and Ids
// ============================================================================

uint64_t lot_root_key(const LotParams& params) {
    return mix2(mix2(kSaltRoot, params.block_key), params.seed);
}

LotId lot_id_for_key(uint64_t node_key) {
    const LotId id = mix2(kSaltId, node_key);
    // kInvalidLotId has to stay unreachable, or a caller cannot test for "no lot"
    // without a one-in-2^64 false negative that nobody would ever reproduce.
    return id == kInvalidLotId ? 1u : id;
}

uint64_t block_key_from_ways(const Block& block, const RoadGraph& graph) {
    std::vector<WayId> ways;
    ways.reserve(block.edges.size());

    const std::vector<GraphEdge>& edges = graph.edges();
    for (const BlockEdge& edge : block.edges) {
        if (edge.edge >= edges.size()) {
            continue;
        }
        ways.push_back(edges[edge.edge].source_way);
    }

    // Sorted and de-duplicated so the key is a function of WHICH ways bound the
    // block. Hashing them in traversal order would make the key depend on where
    // the face walk started and on the order RoadGraph happened to build its
    // edges, neither of which is a property of the block.
    std::sort(ways.begin(), ways.end());
    ways.erase(std::unique(ways.begin(), ways.end()), ways.end());

    uint64_t key = 0x5EED10C5B10C0BADull;
    for (const WayId way : ways) {
        key = mix2(key, static_cast<uint64_t>(way));
    }
    return key;
}

// ============================================================================
// Subdivision
// ============================================================================

LotSubdivision subdivide_ring(const std::vector<glm::dvec2>& ring,
                              const std::vector<uint32_t>& ring_edges,
                              uint64_t node_key,
                              const LotParams& params,
                              uint32_t start_depth) {
    LotSubdivision result;
    if (ring.size() < 3) {
        return result;
    }

    Node root;
    root.key = node_key;
    // The re-entry point's depth in the tree it came from, NOT zero. Zero was
    // wrong twice over. Lot::depth came back short by the re-entry offset in every
    // lot of the subtree, and -- the part that made the header's reproduction
    // promise false -- a node that sat at depth 1 in the full run was handed the
    // WHOLE max_depth budget again instead of the remainder, so once the ceiling
    // actually bit, re-entry reproduced none of the full run's lots.
    root.depth = start_depth;
    root.piece.points = ring;
    root.piece.tags = ring_edges.size() == ring.size()
                          ? ring_edges
                          : std::vector<uint32_t>(ring.size(), kNoBlockEdge);

    subdivide_node(std::move(root), params, result);

    result.stats.lots = result.lots.size();
    return result;
}

/// Discard lots that landed inside a hole. See the note on subdivide_block().
void drop_lots_inside_holes(const Block& block, LotSubdivision& result) {
    if (block.holes.empty() || result.lots.empty()) return;

    const size_t before = result.lots.size();
    std::vector<Lot> kept;
    kept.reserve(before);

    for (Lot& lot : result.lots) {
        // WHOLLY inside, not centroid-inside.
        //
        // The centroid test looks equivalent and is not. A block that takes only
        // one cut -- or none, which is what force_street_access does to a ring
        // with no street frontage -- produces a lot that WRAPS the hole, and its
        // centroid sits in the middle of the hole. Discarding on the centroid
        // threw that lot away and with it every square metre of real land around
        // the hole: 10000 m2 of block became zero lots.
        //
        // Requiring every vertex to be inside can only ever discard a lot that
        // is genuinely all hole, which is the artefact this exists to remove. It
        // errs toward keeping land, and keeping too much is the failure mode the
        // header already documents for straddling lots.
        bool wholly_in_hole = false;
        for (const std::vector<glm::dvec2>& hole : block.holes) {
            bool all_in = !lot.ring.empty();
            for (const glm::dvec2& v : lot.ring) {
                if (!point_in_ring(hole, v)) {
                    all_in = false;
                    break;
                }
            }
            if (all_in && point_in_ring(hole, ring_centroid(lot.ring))) {
                wholly_in_hole = true;
                break;
            }
        }
        if (!wholly_in_hole) kept.push_back(std::move(lot));
    }

    result.stats.lots_in_holes = before - kept.size();
    result.lots = std::move(kept);
}

LotSubdivision subdivide_block(const Block& block, const LotParams& params) {
    LotSubdivision result;
    if (block.ring.size() < 3) {
        return result;
    }

    const uint64_t root_key = lot_root_key(params);

    Piece piece;
    piece.points = block.ring;
    piece.tags = block.ring_edges.size() == block.ring.size()
                     ? block.ring_edges
                     : std::vector<uint32_t>(block.ring.size(), kNoBlockEdge);

    cut_corner_lots(piece, params, root_key, result);

    if (piece.points.size() >= 3) {
        Node root;
        root.key = root_key;
        root.depth = 0;
        root.piece = std::move(piece);
        subdivide_node(std::move(root), params, result);
    }

    drop_lots_inside_holes(block, result);

    result.stats.lots = result.lots.size();
    return result;
}

// ============================================================================
// Barycentric Transfer
// ============================================================================

std::vector<double> mean_value_coordinates(const std::vector<glm::dvec2>& ring,
                                           const glm::dvec2& point) {
    const size_t count = ring.size();
    if (count < 3) {
        return {};
    }

    std::vector<double> weights(count, 0.0);
    std::vector<double> radius(count, 0.0);

    for (size_t i = 0; i < count; ++i) {
        radius[i] = length_of(ring[i] - point);
        if (radius[i] <= kPointEpsilon) {
            // On a vertex. The formula divides by this radius, so the limit is
            // taken here instead: the point IS that vertex.
            weights[i] = 1.0;
            return weights;
        }
    }

    std::vector<double> half_tan(count, 0.0);
    for (size_t i = 0; i < count; ++i) {
        const size_t j = (i + 1) % count;
        const glm::dvec2 a = ring[i] - point;
        const glm::dvec2 b = ring[j] - point;
        const double cr = cross2(a, b);
        const double dt = glm::dot(a, b);

        if (std::fabs(cr) <= kPointEpsilon * radius[i] * radius[j]) {
            // The cross product vanishes, so the point is COLLINEAR with the edge
            // and the half-angle formula below is 0/0. There are two such limits,
            // and the dot product is what tells them apart. Testing only one of
            // them is not a rounding error: the other falls through to 0/0 and
            // every weight in the vector comes back NaN.
            if (dt >= 0.0) {
                // Collinear with the edge but OUTSIDE its span: a and b point the
                // same way from the point, so the edge subtends a zero angle and
                // tan(0/2) is 0. A convex ring cannot produce this -- every edge
                // subtends a positive angle at an interior point -- which is why
                // it was missed. A ring with ONE reflex corner produces it for any
                // interior point that lines up with a far edge, and a block with a
                // reflex corner is ordinary.
                //
                // The NaN this used to leave behind was silent all the way down:
                // it defeated the drift gate in transfer_lot_ids(), because
                // `distance > max_centroid_drift` is FALSE for a NaN distance, so
                // a lot 50 m out of reach inherited an id it had no claim to with
                // no counter moving.
                half_tan[i] = 0.0;
                continue;
            }

            // On the edge between i and j: the two vectors are antiparallel, the
            // cross product vanishes, and the formula is 0/0. The limit is the
            // linear interpolation along that edge.
            const double total = radius[i] + radius[j];
            std::vector<double> edge(count, 0.0);
            if (total > kPointEpsilon) {
                edge[i] = radius[j] / total;
                edge[j] = radius[i] / total;
            } else {
                edge[i] = 1.0;
            }
            return edge;
        }

        // tan(alpha/2) = (|a||b| - dot(a,b)) / cross(a,b), the half-angle identity.
        // This is what keeps trigonometry out of the whole transfer: no atan2, so
        // no dependence on a platform's libm.
        half_tan[i] = (radius[i] * radius[j] - dt) / cr;
    }

    double sum = 0.0;
    for (size_t i = 0; i < count; ++i) {
        const size_t prev = (i + count - 1) % count;
        weights[i] = (half_tan[prev] + half_tan[i]) / radius[i];
        sum += weights[i];
    }

    if (std::fabs(sum) <= kAreaEpsilon) {
        // Degenerate ring, or a point so far outside that the weights cancel.
        // Returning a uniform set keeps the caller from dividing by zero; it maps
        // the point to the ring's vertex mean, which is at least inside.
        for (size_t i = 0; i < count; ++i) {
            weights[i] = 1.0 / static_cast<double>(count);
        }
        return weights;
    }

    for (size_t i = 0; i < count; ++i) {
        weights[i] /= sum;
    }
    return weights;
}

glm::dvec2 map_point_between_rings(const std::vector<glm::dvec2>& from_ring,
                                   const std::vector<glm::dvec2>& to_ring,
                                   const glm::dvec2& point) {
    if (from_ring.size() < 3 || from_ring.size() != to_ring.size()) {
        // No vertex correspondence, so no deformation to follow. Handing the point
        // back unchanged lets transfer_lot_ids() fall through to plain proximity,
        // which is a weaker answer but an honest one; interpolating between rings
        // of different lengths would invent a correspondence.
        return point;
    }

    const std::vector<double> weights = mean_value_coordinates(from_ring, point);
    if (weights.size() != to_ring.size()) {
        return point;
    }

    glm::dvec2 mapped{0.0};
    for (size_t i = 0; i < to_ring.size(); ++i) {
        mapped += to_ring[i] * weights[i];
    }
    return mapped;
}

size_t transfer_lot_ids(const std::vector<Lot>& previous,
                        const std::vector<glm::dvec2>& previous_ring,
                        const std::vector<glm::dvec2>& current_ring,
                        std::vector<Lot>& current,
                        const LotMatchConfig& config) {
    if (previous.empty() || current.empty()) {
        return 0;
    }

    std::vector<glm::dvec2> previous_centroids;
    previous_centroids.reserve(previous.size());
    for (const Lot& lot : previous) {
        previous_centroids.push_back(ring_centroid(lot.ring));
    }

    /// One possible pairing, with everything needed to order it totally
    struct Candidate {
        double cost = 0.0;
        size_t current_index = 0;
        size_t previous_index = 0;
    };

    // A containment match beats every distance match. The penalty is larger than
    // any drift a real block could produce, so adding it rather than sorting on a
    // separate flag keeps the ordering a single comparison.
    constexpr double kNotContainedPenalty = 1.0e9;

    std::vector<Candidate> candidates;
    for (size_t c = 0; c < current.size(); ++c) {
        const glm::dvec2 centroid = ring_centroid(current[c].ring);
        const glm::dvec2 mapped = map_point_between_rings(current_ring, previous_ring, centroid);

        if (!std::isfinite(mapped.x) || !std::isfinite(mapped.y)) {
            // Belt as well as braces. mean_value_coordinates() no longer returns
            // NaN, but a non-finite mapped point must never reach the loop below:
            // every gate there is a `>` against a tolerance, and a NaN is not
            // greater than anything, so the candidate would survive with a NaN
            // cost. That cost then makes the comparator further down violate
            // strict weak ordering -- NaN compares equivalent to every finite cost
            // while the finite costs are ordered among themselves -- which is
            // undefined behaviour in std::sort, not merely a wrong answer.
            //
            // Dropping the lot from the candidate list leaves it with its
            // provisional id, which is what an unmatchable lot is supposed to get.
            continue;
        }

        for (size_t p = 0; p < previous.size(); ++p) {
            const double distance = length_of(mapped - previous_centroids[p]);
            const bool contained = point_in_ring(previous[p].ring, mapped);
            if (!contained && distance > config.max_centroid_drift) {
                continue;
            }
            candidates.push_back(
                Candidate{distance + (contained ? 0.0 : kNotContainedPenalty), c, p});
        }
    }

    // Sorted on the full triple, so ties break on indices rather than on whatever
    // order the pairs were generated in. Two lots landing in one old lot at
    // exactly equal distance is not a hypothetical -- it is what a symmetric block
    // produces -- and without the index tie-break the winner would depend on the
    // sort implementation.
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        if (a.cost != b.cost) {
            return a.cost < b.cost;
        }
        if (a.current_index != b.current_index) {
            return a.current_index < b.current_index;
        }
        return a.previous_index < b.previous_index;
    });

    std::vector<bool> current_taken(current.size(), false);
    std::vector<bool> previous_taken(previous.size(), false);
    size_t matched = 0;

    for (const Candidate& candidate : candidates) {
        if (current_taken[candidate.current_index] || previous_taken[candidate.previous_index]) {
            continue;
        }
        current_taken[candidate.current_index] = true;
        previous_taken[candidate.previous_index] = true;
        current[candidate.current_index].id = previous[candidate.previous_index].id;
        ++matched;
    }

    return matched;
}

} // namespace stratum::osm::road
