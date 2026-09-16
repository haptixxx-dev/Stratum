// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file lots_offset.cpp
 * @brief The offset and skeleton lot subdivisions behind lots_offset.hpp
 *
 * The header says WHAT each mode produces and why both are built on the straight
 * skeleton. This file is about HOW, and about the four places the detail matters:
 *
 *   1. **Every boolean goes through Clipper2 on a fixed integer scale.** Not
 *      because the shapes are hard -- most of them are a convex region clipped by a
 *      half-plane -- but because a hand-rolled clip has to decide what to do when
 *      the cut passes exactly through a vertex, and there are hundreds of such cuts
 *      per block. Clipper2 decides once, in integer arithmetic, the same way on
 *      every platform.
 *   2. **A piece is cut from a SKELETON FACE, never from the block ring.** The face
 *      of edge e is the land whose nearest street is e, and the faces tile the
 *      block, so pieces cut from different faces cannot overlap however sharp the
 *      corner between the two streets is. Cutting the block ring instead and hoping
 *      the cuts do not cross is where the overlapping-lot bugs live.
 *   3. **Identity is derived from the contour edge, not from emission order.** A
 *      lot's key is `mix2(block root, contour edge)` then `mix2(that, index along
 *      the edge)`. Nudge a street and both are unchanged, so every id in the block
 *      survives with no matching step at all -- mechanism 2 of the three in the C7
 *      notes.
 *   4. **No transcendental function touches a coordinate.** Angles are compared as
 *      dot and cross products, directions are blended by lerp-and-normalise, and
 *      the only libm that reaches a coordinate is sqrt -- plus ceil, floor and
 *      llround, which are exact. libm is not bit-reproducible across platforms and
 *      a lot boundary has to be.
 */

#include "osm/road/lots_offset.hpp"

#include "geometry/straight_skeleton.hpp"

#include <clipper2/clipper.h>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <utility>

namespace stratum::osm::road {

namespace {

using Clipper2Lib::ClipType;
using Clipper2Lib::FillRule;
using Clipper2Lib::Path64;
using Clipper2Lib::Paths64;
using Clipper2Lib::Point64;

// ============================================================================
// Tolerances and Scale
// ============================================================================

/**
 * @brief Integer units per metre for every Clipper2 call in this file
 *
 * Clipper2 works in int64 and squares coordinates internally, so the usable range
 * is nearer 2^31 counts than 2^63. At 1e4 counts per metre that is a hundred
 * kilometres of local-metre coordinate, which is further from the projection
 * origin than any single extract, and the precision bought is a tenth of a
 * millimetre. A lot boundary a tenth of a millimetre out is a lot boundary.
 *
 * The same integer-scaled Path64 pattern as osm/road/zoning.cpp and
 * procgen/rules/shape.cpp, and for the same reason: integer arithmetic gives the
 * same answer on every platform, where Clipper2's double-coordinate API scales to
 * int64 internally with a precision it chooses.
 */
constexpr double kClipperScale = 1.0e4;

/// Largest scaled coordinate accepted, in Clipper2 units. See kClipperScale.
constexpr double kMaxScaledCoordinate = 1.0e9;

/// Below this many square metres a piece is not land
constexpr double kAreaEpsilon = 1e-6;

/// Below this, two lengths or two positions in metres are the same
constexpr double kPointEpsilon = 1e-9;

/**
 * @brief How close a lot segment must lie to a block segment to inherit its edge
 *
 * A millimetre: an order of magnitude above the tenth-of-a-millimetre Clipper2
 * quantisation, and three orders below the narrowest lot anyone would ask for. Too
 * tight and every frontage is lost to rounding; too loose and a lot one millimetre
 * from the kerb claims to front it.
 */
constexpr double kFrontageEpsilon = 1e-3;

/// Below this, a direction has no direction
constexpr double kDirectionEpsilon = 1e-12;

// ============================================================================
// Deterministic Mixing
//
// SplitMix64, byte for byte the same finaliser lots.cpp uses. It has to be the
// same one: lot_id_for_key() is called on keys built here, so a different mixer
// would put this file's ids in a different space from C2's and the two could
// collide. The SALTS are disjoint from lots.cpp's 0x01..0x07 so that a key built
// here can never equal a key built there.
// ============================================================================

[[nodiscard]] uint64_t mix64(uint64_t x) noexcept {
    x += 0x9E3779B97F4A7C15ull;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
    return x ^ (x >> 31);
}

[[nodiscard]] uint64_t mix2(uint64_t a, uint64_t b) noexcept {
    return mix64(a ^ mix64(b + 0x165667B19E3779F9ull));
}

/// A value in [0, 1) from a hash. 53 bits scaled by an exact power of two.
[[nodiscard]] double unit_from(uint64_t hash) noexcept {
    return static_cast<double>(hash >> 11) * 0x1.0p-53;
}

/// A value in [-1, 1) from a hash
[[nodiscard]] double signed_unit_from(uint64_t hash) noexcept {
    return unit_from(hash) * 2.0 - 1.0;
}

enum : uint64_t {
    kSaltFace = 0x11,      ///< One skeleton face of the block
    kSaltPiece = 0x12,     ///< One lot along a face's street
    kSaltCorner = 0x13,    ///< A wedge left by corner alignment
    kSaltBackland = 0x14,  ///< Land behind a shallow street lot
    kSaltInterior = 0x15,  ///< A yard region behind the offset band
    kSaltJitter = 0x16,    ///< The irregularity draw on a cut position

    /**
     * @brief Which MODE cut the block, mixed into its root key
     *
     * The two modes address their pieces the same way -- face, then index along the
     * face -- so without this they produce identical keys and therefore identical
     * ids for parcels that are not the same parcel. An attribute set on an offset
     * lot would then appear on a skeleton lot somewhere else on the block, which is
     * exactly the failure the whole C7 identity scheme exists to prevent.
     */
    kSaltOffsetMode = 0x21,
    kSaltSkeletonMode = 0x22,
};

/**
 * @brief The key of the @p index-th separate region a single cut produced
 *
 * A cut is one slab of one street, and it normally yields one region. It yields
 * SEVERAL when something splits it -- a hole in the block, or a block arm that
 * pinches -- and then every one of them needs its own key. Giving them all the cut's
 * key gave them all the same id, and a duplicate LotId is worse than a wrong one:
 * an attribute written to either lot lands on both.
 */
[[nodiscard]] uint64_t region_key(uint64_t cut_key, size_t index) noexcept {
    return mix2(cut_key, mix2(0x31ull, static_cast<uint64_t>(index)));
}

// ============================================================================
// Small Geometry
// ============================================================================

[[nodiscard]] double cross2(const glm::dvec2& a, const glm::dvec2& b) noexcept {
    return a.x * b.y - a.y * b.x;
}

/// Quarter turn anticlockwise; the inward normal of an anticlockwise ring's edge
[[nodiscard]] glm::dvec2 perp(const glm::dvec2& v) noexcept {
    return glm::dvec2{-v.y, v.x};
}

[[nodiscard]] double ring_area(const std::vector<glm::dvec2>& ring) noexcept {
    if (ring.size() < 3) return 0.0;
    double twice = 0.0;
    for (size_t i = 0; i < ring.size(); ++i) {
        const glm::dvec2& a = ring[i];
        const glm::dvec2& b = ring[(i + 1) % ring.size()];
        twice += a.x * b.y - b.x * a.y;
    }
    return 0.5 * twice;
}

/// Area centroid, falling back to the vertex mean for a ring with no area
[[nodiscard]] glm::dvec2 ring_centroid(const std::vector<glm::dvec2>& ring) {
    if (ring.empty()) return glm::dvec2{0.0};
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
        glm::dvec2 mean{0.0};
        for (const glm::dvec2& p : ring) mean += p;
        return mean / static_cast<double>(ring.size());
    }
    return acc / (3.0 * twice_area);
}

/// Crossing-number point-in-polygon, the same one lots.cpp uses
[[nodiscard]] bool point_in_ring(const std::vector<glm::dvec2>& ring, const glm::dvec2& p) {
    if (ring.size() < 3) return false;
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

/// Distance from @p p to the segment (a, b)
[[nodiscard]] double distance_to_segment(const glm::dvec2& a, const glm::dvec2& b,
                                         const glm::dvec2& p) {
    const glm::dvec2 ab = b - a;
    const double len2 = glm::dot(ab, ab);
    if (len2 <= kDirectionEpsilon) return glm::length(p - a);
    const double t = std::clamp(glm::dot(p - a, ab) / len2, 0.0, 1.0);
    return glm::length(p - (a + ab * t));
}

// ============================================================================
// Clipper2 Bridge
// ============================================================================

/**
 * @brief Quantise a ring of metres into a Clipper2 path
 *
 * Rounds to nearest rather than truncating, because truncation biases every
 * coordinate towards the origin by up to one unit and so shrinks every polygon
 * systematically -- and a systematic shrink of every lot is a systematic loss of
 * land that the area-conservation check would then have to tolerate.
 *
 * @return False when a coordinate is out of range or the ring collapses to under
 *         three distinct points once quantised
 */
[[nodiscard]] bool to_path(const std::vector<glm::dvec2>& ring, Path64& out) {
    out.clear();
    if (ring.size() < 3) return false;
    out.reserve(ring.size());
    for (const glm::dvec2& p : ring) {
        const double sx = p.x * kClipperScale;
        const double sy = p.y * kClipperScale;
        if (!std::isfinite(sx) || !std::isfinite(sy) ||
            std::fabs(sx) > kMaxScaledCoordinate || std::fabs(sy) > kMaxScaledCoordinate) {
            return false;
        }
        const Point64 q(static_cast<int64_t>(std::llround(sx)),
                        static_cast<int64_t>(std::llround(sy)));
        if (!out.empty() && out.back().x == q.x && out.back().y == q.y) continue;
        out.push_back(q);
    }
    while (out.size() >= 2 && out.front().x == out.back().x && out.front().y == out.back().y) {
        out.pop_back();
    }
    return out.size() >= 3;
}

[[nodiscard]] std::vector<glm::dvec2> from_path(const Path64& path) {
    std::vector<glm::dvec2> ring;
    ring.reserve(path.size());
    for (const Point64& p : path) {
        ring.push_back(glm::dvec2{static_cast<double>(p.x) / kClipperScale,
                                  static_cast<double>(p.y) / kClipperScale});
    }
    return ring;
}

/**
 * @brief Rotate a path to start at its lexicographically smallest point
 *
 * Clipper2 is deterministic for identical input, but WHERE in a ring it starts is
 * an implementation detail a version bump is free to change. Rotating to a
 * canonical start makes a lot's vertex order a function of the lot alone, which is
 * what the "two runs agree byte for byte" promise in the C7 notes actually needs.
 * The same treatment procgen/rules/shape.cpp gives its offset outlines.
 */
void canonicalise(Path64& path) {
    if (path.size() < 2) return;
    size_t best = 0;
    for (size_t i = 1; i < path.size(); ++i) {
        if (path[i].x < path[best].x || (path[i].x == path[best].x && path[i].y < path[best].y)) {
            best = i;
        }
    }
    std::rotate(path.begin(), path.begin() + static_cast<std::ptrdiff_t>(best), path.end());
}

/// The same rotation as canonicalise(), on a ring in metres
void rotate_to_canonical_start(std::vector<glm::dvec2>& ring) {
    if (ring.size() < 2) return;
    size_t best = 0;
    for (size_t i = 1; i < ring.size(); ++i) {
        if (ring[i].x < ring[best].x || (ring[i].x == ring[best].x && ring[i].y < ring[best].y)) {
            best = i;
        }
    }
    std::rotate(ring.begin(), ring.begin() + static_cast<std::ptrdiff_t>(best), ring.end());
}

/**
 * @brief Canonicalise every path, then put the paths themselves in a fixed order
 *
 * The per-path rotation is load-bearing and tested: see
 * LotsOffset/every_lot_ring_starts_at_its_smallest_vertex.
 *
 * THE SORT IS NOT, AND THE NEXT READER SHOULD KNOW THAT. It is called only on the
 * inward offset, and every boolean downstream re-canonicalises its own output, so
 * skipping it changes nothing: 120 random blocks in both modes, with two courtyards
 * listed both ways round, gave byte-identical lots with and without. It is kept as
 * insulation against a Clipper2 whose answer depends on the order of its subject
 * paths -- which is a property no version promises either way -- and there is no
 * test that fails when it goes, because there is no input that reaches it.
 */
void canonicalise(Paths64& paths) {
    for (Path64& path : paths) canonicalise(path);
    std::sort(paths.begin(), paths.end(), [](const Path64& a, const Path64& b) {
        if (a.empty() || b.empty()) return a.size() < b.size();
        if (a[0].x != b[0].x) return a[0].x < b[0].x;
        if (a[0].y != b[0].y) return a[0].y < b[0].y;
        return a.size() < b.size();
    });
}

/// A boolean result: one outer ring and the holes inside it, all in metres
struct Region {
    std::vector<glm::dvec2> outer;                   ///< Anticlockwise
    std::vector<std::vector<glm::dvec2>> holes;      ///< Clockwise, as Block::holes are
    double area = 0.0;                               ///< Outer area less the holes
};

/**
 * @brief Turn one node of a Clipper2 PolyTree into a region, and recurse
 *
 * A node at an odd level is an outer ring and its direct children are its holes;
 * the children of THOSE are outer rings again -- an island in a lake in an island.
 * Walking the tree is the only way to get that nesting right.
 *
 * ### Why the tree and not the ring orientations
 *
 * The first version of this function took the flat Paths64 solution, called the
 * positive-area rings outers and the negative ones holes, and attached each hole to
 * the smallest outer whose ring CONTAINED THE HOLE'S CENTROID. That is wrong twice
 * over, and it failed in the field rather than in theory: a non-convex hole's area
 * centroid need not be inside the hole, and on a star-shaped block it was not even
 * inside the outer. The hole was then attached to nothing and silently discarded,
 * the band came back as the whole block instead of the block less the yard, and the
 * lots covered 8822 square metres of a 5378 square metre block.
 *
 * Clipper2 already knows the nesting exactly. Asking it costs one extra call.
 */
void collect_regions(const Clipper2Lib::PolyPath64& node, std::vector<Region>& out) {
    for (size_t i = 0; i < node.Count(); ++i) {
        const Clipper2Lib::PolyPath64& child = *node.Child(i);

        Region region;
        region.outer = from_path(child.Polygon());
        region.area = ring_area(region.outer);
        // Clipper2 hands back outers and holes with opposite windings; Lot::ring and
        // Block::holes want anticlockwise and clockwise respectively, which is what
        // this fixes up so no caller has to care which way round Clipper2 was.
        if (region.area < 0.0) {
            std::reverse(region.outer.begin(), region.outer.end());
            region.area = -region.area;
        }

        for (size_t h = 0; h < child.Count(); ++h) {
            const Clipper2Lib::PolyPath64& grand = *child.Child(h);
            std::vector<glm::dvec2> hole = from_path(grand.Polygon());
            const double hole_area = ring_area(hole);
            if (hole.size() >= 3 && std::fabs(hole_area) > kAreaEpsilon) {
                if (hole_area > 0.0) std::reverse(hole.begin(), hole.end());
                region.area -= std::fabs(hole_area);
                region.holes.push_back(std::move(hole));
            }
            // An island inside this hole is a region in its own right.
            collect_regions(grand, out);
        }

        if (region.outer.size() >= 3 && region.area > kAreaEpsilon) {
            out.push_back(std::move(region));
        }
    }
}

/**
 * @brief Run one boolean and return its regions, holes nested in their outers
 *
 * The output order is canonicalised -- each ring rotated to its lexicographically
 * smallest point, the regions sorted by that point -- so the lots that come out of
 * it are a function of the input and not of the order Clipper2 happened to build
 * its solution in. Two runs have to agree byte for byte; see the C7 notes.
 */
[[nodiscard]] std::vector<Region> boolean_regions(ClipType type, const Paths64& subject,
                                                  const Paths64& clip) {
    if (subject.empty()) return {};
    Clipper2Lib::Clipper64 clipper;
    clipper.AddSubject(subject);
    if (!clip.empty()) clipper.AddClip(clip);
    Clipper2Lib::PolyTree64 tree;
    if (!clipper.Execute(type, FillRule::NonZero, tree)) return {};

    std::vector<Region> regions;
    collect_regions(tree, regions);
    for (Region& region : regions) {
        rotate_to_canonical_start(region.outer);
        for (std::vector<glm::dvec2>& hole : region.holes) rotate_to_canonical_start(hole);
        std::sort(region.holes.begin(), region.holes.end(),
                  [](const std::vector<glm::dvec2>& a, const std::vector<glm::dvec2>& b) {
                      if (a.empty() || b.empty()) return a.size() < b.size();
                      if (a[0].x != b[0].x) return a[0].x < b[0].x;
                      if (a[0].y != b[0].y) return a[0].y < b[0].y;
                      return a.size() < b.size();
                  });
    }
    // The region order decides which region of one cut gets index 0 in
    // region_key(), so it decides which of two lots gets which id. Removing this
    // sort does change the answer -- on a five-pointed block with two courtyards it
    // swaps the ids of two lots cut from one slab -- but only by swapping ids
    // between two lots that both exist and are both correct, which no assertion
    // this file's suite can make from the outside distinguishes from the right
    // answer. It is here for the C7 promise and it is NOT covered by a test.
    std::sort(regions.begin(), regions.end(), [](const Region& a, const Region& b) {
        if (a.outer.empty() || b.outer.empty()) return a.outer.size() < b.outer.size();
        if (a.outer[0].x != b.outer[0].x) return a.outer[0].x < b.outer[0].x;
        if (a.outer[0].y != b.outer[0].y) return a.outer[0].y < b.outer[0].y;
        return a.outer.size() < b.outer.size();
    });
    return regions;
}

/// The regions of a path set, as if it had been through a no-op boolean
[[nodiscard]] std::vector<Region> to_regions(const Paths64& paths) {
    return boolean_regions(ClipType::Union, paths, Paths64{});
}

/// Every region as a path set again, holes included, for feeding the next boolean
[[nodiscard]] Paths64 regions_to_paths(const std::vector<Region>& regions) {
    Paths64 paths;
    for (const Region& region : regions) {
        Path64 path;
        if (to_path(region.outer, path)) paths.push_back(std::move(path));
        for (const std::vector<glm::dvec2>& hole : region.holes) {
            Path64 hole_path;
            if (to_path(hole, hole_path)) paths.push_back(std::move(hole_path));
        }
    }
    return paths;
}

/**
 * @brief Cut every region of @p regions until none of them has a hole left
 *
 * A Lot::ring is ONE ring, so a piece with a hole in it cannot become a lot. The
 * first version of this file emitted such a piece as its outer ring and counted it,
 * which covered the hole: a 160 m block with a 40 m courtyard came out with 30400
 * square metres of lots over 24000 square metres of land, and a lot sitting on top
 * of the courtyard.
 *
 * Cutting is the answer rather than discarding, because the land around the hole is
 * real and has to end up in some lot. A straight cut through a hole puts that hole's
 * boundary into the OUTER boundary of both halves, so one cut removes one hole and
 * the loop terminates. The cut is vertical through the hole's centroid, and
 * horizontal if that fails to change anything -- which it can, for a hole whose
 * centroid lies outside it.
 *
 * The hole with the lexicographically smallest centroid is always cut first, so the
 * output is a function of the input and not of the order Clipper2 happened to
 * return the rings in.
 */
[[nodiscard]] std::vector<Region> split_until_simple(std::vector<Region> regions, double reach);

/**
 * @brief The half-plane `dot(x - origin, inward) >= 0`, as a big anticlockwise box
 *
 * Every cut this file makes is one of these: the depth limit of a shallow lot, the
 * two ends of a lot's slab along its street, and the squared-off end of a run.
 * Expressing all four as one primitive means there is one place where the
 * orientation convention can be wrong, rather than four.
 *
 * @param origin A point on the boundary line
 * @param inward Unit vector into the side that is KEPT
 * @param reach  Half-width of the box; must exceed the block's own diameter
 */
[[nodiscard]] Path64 half_plane(const glm::dvec2& origin, const glm::dvec2& inward, double reach) {
    const glm::dvec2 along = perp(inward);
    std::vector<glm::dvec2> ring = {
        origin - along * reach,
        origin + along * reach,
        origin + along * reach + inward * reach,
        origin - along * reach + inward * reach,
    };
    // Built from a frame whose handedness depends on `inward`, so the winding is
    // fixed here rather than reasoned about at each of the four call sites.
    if (ring_area(ring) < 0.0) std::reverse(ring.begin(), ring.end());
    Path64 path;
    // The box is four times the block's diagonal, so it cannot fail to_path()'s
    // range check for any block this file will accept, and a returned empty path
    // would clip everything away rather than nothing.
    if (!to_path(ring, path)) path.clear();
    return path;
}

std::vector<Region> split_until_simple(std::vector<Region> regions, double reach) {
    std::vector<Region> simple;
    // Bounded rather than `while (true)`: every pass removes at least one hole from
    // at least one region, so a real input finishes long before this, and an input
    // that does not is a bug that should show as missing land rather than as a hang.
    for (size_t guard = 0; guard < 64 && !regions.empty(); ++guard) {
        std::vector<Region> next;
        for (Region& region : regions) {
            if (region.holes.empty()) {
                simple.push_back(std::move(region));
                continue;
            }
            glm::dvec2 best = ring_centroid(region.holes.front());
            for (size_t h = 1; h < region.holes.size(); ++h) {
                const glm::dvec2 c = ring_centroid(region.holes[h]);
                if (c.x < best.x || (c.x == best.x && c.y < best.y)) best = c;
            }

            const Paths64 subject = regions_to_paths({region});
            bool cut = false;
            for (int axis = 0; axis < 2 && !cut; ++axis) {
                const glm::dvec2 inward =
                    (axis == 0) ? glm::dvec2{1.0, 0.0} : glm::dvec2{0.0, 1.0};
                const Paths64 keep{half_plane(best, inward, reach)};
                std::vector<Region> a = boolean_regions(ClipType::Intersection, subject, keep);
                std::vector<Region> b = boolean_regions(ClipType::Difference, subject, keep);
                if (a.empty() || b.empty()) continue;
                for (Region& r : a) next.push_back(std::move(r));
                for (Region& r : b) next.push_back(std::move(r));
                cut = true;
            }
            if (!cut) {
                // Neither cut divided it. Keeping the outer ring would cover the
                // hole, so the region is dropped instead: land that is missing is
                // visible in the area total, land that is covered twice is not.
                continue;
            }
        }
        regions = std::move(next);
    }
    return simple;
}

// ============================================================================
// Frontage Tagging
// ============================================================================

/// One segment of the block's own boundary, and the half-edge behind it
struct BoundarySegment {
    glm::dvec2 a{0.0};
    glm::dvec2 b{0.0};
    uint32_t edge = kNoBlockEdge;
};

/// Every segment of the block's outer ring and of its holes, with its half-edge
[[nodiscard]] std::vector<BoundarySegment> boundary_segments(const Block& block) {
    std::vector<BoundarySegment> segments;
    const auto add_ring = [&](const std::vector<glm::dvec2>& ring,
                              const std::vector<uint32_t>& tags) {
        for (size_t i = 0; i < ring.size(); ++i) {
            BoundarySegment segment;
            segment.a = ring[i];
            segment.b = ring[(i + 1) % ring.size()];
            segment.edge = (tags.size() == ring.size()) ? tags[i] : kNoBlockEdge;
            if (glm::length(segment.b - segment.a) > kPointEpsilon) {
                segments.push_back(segment);
            }
        }
    };
    add_ring(block.ring, block.ring_edges);
    for (size_t h = 0; h < block.holes.size(); ++h) {
        add_ring(block.holes[h], h < block.hole_edges.size() ? block.hole_edges[h]
                                                             : std::vector<uint32_t>{});
    }
    return segments;
}

/**
 * @brief Tag each segment of @p ring with the block half-edge it lies on
 *
 * This is what C4 reads, and it is the reason a lot never has to be matched back to
 * a street by proximity -- the same class of mistake as finding junctions by
 * endpoint proximity, failing in the same place, where two streets run a few metres
 * apart.
 *
 * A lot segment inherits a block segment's half-edge when BOTH its endpoints lie
 * within kFrontageEpsilon of that block segment AND the two are parallel. The
 * parallel test is what stops a lot edge that merely crosses the kerb at a shallow
 * angle from claiming to front it; both endpoints being near the segment is not
 * enough on its own for a short lot edge near a long street.
 */
void tag_frontage(const std::vector<BoundarySegment>& segments,
                  const std::vector<glm::dvec2>& ring, std::vector<uint32_t>& out_tags,
                  double& out_frontage_length) {
    out_tags.assign(ring.size(), kNoBlockEdge);
    out_frontage_length = 0.0;
    for (size_t i = 0; i < ring.size(); ++i) {
        const glm::dvec2& p = ring[i];
        const glm::dvec2& q = ring[(i + 1) % ring.size()];
        const double length = glm::length(q - p);
        if (length <= kPointEpsilon) continue;
        const glm::dvec2 dir = (q - p) / length;

        for (const BoundarySegment& segment : segments) {
            if (distance_to_segment(segment.a, segment.b, p) > kFrontageEpsilon) continue;
            if (distance_to_segment(segment.a, segment.b, q) > kFrontageEpsilon) continue;
            const glm::dvec2 seg = segment.b - segment.a;
            const double seg_length = glm::length(seg);
            if (seg_length <= kPointEpsilon) continue;
            // Parallel to within the frontage epsilon over the lot segment's own
            // length, so the test tightens for a long edge rather than staying a
            // fixed angle that a long edge could drift right across.
            //
            // Mostly redundant against the two endpoint tests above -- both ends
            // being within a millimetre of a straight segment already bounds the
            // angle -- and it earns its place only for a lot edge SHORTER than the
            // frontage epsilon, where an interior cut meeting the kerb at an acute
            // corner would otherwise be counted as frontage. No test fails when this
            // line goes, because the land involved is under a square millimetre.
            if (std::fabs(cross2(dir, seg / seg_length)) * length > kFrontageEpsilon) continue;
            out_tags[i] = segment.edge;
            out_frontage_length += length;
            break;
        }
    }
}

// ============================================================================
// Simplification
// ============================================================================

/**
 * @brief A lot vertex, keyed by its exact coordinates
 *
 * Exact, not rounded. Two lots that share a boundary got that boundary from the
 * same Clipper2 integer point through the same division, so their copies of it are
 * identical to the bit -- and a key that ROUNDED could put two genuinely distinct
 * points a tenth of a millimetre apart into one bucket, which would make the code
 * below merge two boundaries that are not the same boundary.
 *
 * The failure mode of exactness is the safe one. If two copies of what should be
 * one point ever differ in the last bit, each is seen once instead of twice, each
 * is then treated as unshared, and nothing is simplified there. Less
 * simplification, never a gap.
 *
 * std::pair's ordering is what puts the table in a canonical order, which is what
 * makes the whole pass deterministic; -0.0 and 0.0 compare equal under it, which is
 * what we want.
 */
using PointKey = std::pair<double, double>;

[[nodiscard]] PointKey key_of(const glm::dvec2& p) noexcept {
    return PointKey{p.x, p.y};
}

/// One lot vertex, and everything known about it from every ring it appears in
struct SharedVertex {
    PointKey lo{};                ///< The lexicographically smaller of its two neighbours
    PointKey hi{};                ///< The larger
    std::vector<size_t> lots;     ///< Indices of the lots whose ring contains it
    bool consistent = true;       ///< Every ring agreed on that pair of neighbours
};

/**
 * @brief Drop vertices from lot boundaries, deciding each ONCE for both sides
 *
 * ### The defect this replaces, because it is the whole reason for the machinery
 *
 * The first version of this ran a perpendicular-distance simplification on each lot
 * ring INDEPENDENTLY, after the lots were cut, and its comment claimed that was
 * safe because "it never moves a vertex". That is true of the vertex and false of
 * the BOUNDARY. Two lots share an edge. The test at a shared vertex is evaluated
 * against that vertex's neighbours IN THE RING BEING SIMPLIFIED, and the two rings
 * approach the shared vertex from different sides, so they have different
 * neighbours and can reach different answers. One lot drops the vertex, the other
 * keeps it, and the shared edge has moved by exactly as much as moving a point
 * would have moved it.
 *
 * It was invisible on a block with perfectly straight streets, because there the
 * dropped vertices are exactly collinear and the simplification is a no-op on the
 * geometry. Put a 1.5 metre sag in one kerb -- a curve, which every real street has
 * -- and at `simplify` 0.5, inside the range the header used to recommend, the lots
 * lost 0.38% of the block and three sample points of the test's own grid landed in
 * no lot at all. At 4 metres it was 4.6% and 186 points.
 *
 * ### What this does instead
 *
 * The decision is made on the SHARED GEOMETRY, once, and then applied to every ring
 * that contains it -- so both sides of every boundary see the same vertex set and
 * the boundary cannot move relative to itself.
 *
 * A vertex is a candidate only when it appears in at least two lot rings AND every
 * one of those rings gives it the same unordered pair of neighbours. That is the
 * precise condition under which "the line through its neighbours" is a property of
 * the vertex rather than of the ring, and it has three consequences worth stating:
 *
 *   - **The block's own outline never moves.** A kerb vertex is shared by the lot
 *     fronting the street before it and the lot fronting the street after it, and
 *     those two see different neighbours -- one looks back along the kerb, the
 *     other forward along the next kerb. So it is not a candidate, the outline is
 *     preserved exactly, and the lots still sum to the area of the block however
 *     large the tolerance is. Land moves BETWEEN two lots that share a straightened
 *     boundary; none of it leaves.
 *   - **A lot's own private vertex never moves either.** It appears in one ring, so
 *     it fails the two-ring test. That costs nothing: a vertex no other lot shares
 *     is on the block outline, which is the thing above.
 *   - **What is left is the interior boundaries**, which is what there were a lot
 *     of: the skeleton ridge down the middle of a block carries a node for every
 *     polyline point of both kerbs facing it.
 *
 * Candidates are then grouped into maximal CHAINS between two non-candidates and
 * each chain is simplified once, as a polyline with its two ends pinned. Running
 * the removal chain by chain rather than vertex by vertex is what lets a run of
 * points collapse to its two ends: each removal updates the neighbours the next
 * test sees, exactly as the per-ring version did, but now there is one such
 * sequence per boundary instead of one per side of it.
 *
 * Determinism: the vertex table and the removed set are ordered containers keyed on
 * coordinates, chains are walked from the smallest member and oriented smallest-end
 * first, so the same lots give the same removals in the same order on any platform.
 *
 * @param segments  Block boundary, for re-tagging the rings that changed
 * @param tolerance Perpendicular distance in metres; zero or less does nothing
 * @param out       Lots to simplify in place; areas and frontage tags are refreshed
 * @return How many DISTINCT boundary vertices were deleted
 */
[[nodiscard]] size_t simplify_shared_boundaries(const std::vector<BoundarySegment>& segments,
                                                double tolerance, LotSubdivision& out) {
    if (!(tolerance > 0.0) || out.lots.size() < 2) return 0;

    std::map<PointKey, SharedVertex> table;
    for (size_t li = 0; li < out.lots.size(); ++li) {
        const std::vector<glm::dvec2>& ring = out.lots[li].ring;
        const size_t n = ring.size();
        if (n < 3) continue;
        for (size_t i = 0; i < n; ++i) {
            PointKey a = key_of(ring[(i + n - 1) % n]);
            PointKey b = key_of(ring[(i + 1) % n]);
            if (b < a) std::swap(a, b);
            SharedVertex& entry = table[key_of(ring[i])];
            if (entry.lots.empty()) {
                entry.lo = a;
                entry.hi = b;
            } else if (entry.lo != a || entry.hi != b) {
                entry.consistent = false;
            }
            entry.lots.push_back(li);
        }
    }

    const auto candidate = [&table](const PointKey& k) {
        const auto it = table.find(k);
        return it != table.end() && it->second.consistent && it->second.lots.size() >= 2 &&
               it->second.lo != it->second.hi;
    };
    const auto step_from = [&table](const PointKey& v, const PointKey& came_from) {
        const SharedVertex& e = table.at(v);
        return (e.lo == came_from) ? e.hi : e.lo;
    };

    // Ring sizes are tracked as removals commit, because a lot may not be reduced
    // below a triangle -- and refusing the removal for ONE of its lots would be the
    // original defect all over again, so the refusal has to happen before the
    // vertex is removed from any of them.
    std::vector<size_t> ring_size(out.lots.size(), 0);
    for (size_t li = 0; li < out.lots.size(); ++li) ring_size[li] = out.lots[li].ring.size();

    std::set<PointKey> removed;
    std::set<PointKey> visited;

    for (const auto& seed : table) {
        if (!candidate(seed.first) || visited.count(seed.first) != 0) continue;

        // Walk out to a non-candidate in each direction. The two ends are ANCHORS:
        // they stay, and they are what the first and last interior vertex are
        // measured against.
        // Both walks are bounded by the table size, not by reaching the start. A
        // candidate has one pair of neighbours, so a walk either reaches a
        // non-candidate or comes back round -- unless the table is inconsistent,
        // and then an unbounded walk grows `before` until the process dies. The
        // bound turns that into a chain this pass declines to touch.
        bool cyclic = false;
        std::vector<PointKey> before;
        PointKey cur = seed.first;
        PointKey next = table.at(seed.first).lo;
        for (size_t guard = 0; guard <= table.size(); ++guard) {
            if (!candidate(next)) break;
            if (next == seed.first || guard == table.size()) {
                cyclic = true;
                break;
            }
            before.push_back(next);
            const PointKey beyond = step_from(next, cur);
            cur = next;
            next = beyond;
        }
        if (cyclic) continue;  // a closed boundary with no junction on it; leave it alone
        const PointKey anchor_lo = next;

        std::vector<PointKey> after;
        cur = seed.first;
        next = table.at(seed.first).hi;
        for (size_t guard = 0; guard <= table.size(); ++guard) {
            if (!candidate(next)) break;
            if (next == seed.first || guard == table.size()) {
                cyclic = true;
                break;
            }
            after.push_back(next);
            const PointKey beyond = step_from(next, cur);
            cur = next;
            next = beyond;
        }
        if (cyclic) continue;
        const PointKey anchor_hi = next;

        std::vector<PointKey> chain;
        chain.reserve(before.size() + after.size() + 3);
        chain.push_back(anchor_lo);
        for (size_t k = before.size(); k-- > 0;) chain.push_back(before[k]);
        chain.push_back(seed.first);
        for (const PointKey& k : after) chain.push_back(k);
        chain.push_back(anchor_hi);
        for (size_t k = 1; k + 1 < chain.size(); ++k) visited.insert(chain[k]);
        // Oriented smallest end first, so the removal sequence below does not depend
        // on which of the chain's vertices the table happened to reach first.
        if (chain.back() < chain.front()) std::reverse(chain.begin(), chain.end());

        bool changed = true;
        while (changed && chain.size() > 2) {
            changed = false;
            for (size_t i = 1; i + 1 < chain.size(); ++i) {
                const glm::dvec2 prev{chain[i - 1].first, chain[i - 1].second};
                const glm::dvec2 here{chain[i].first, chain[i].second};
                const glm::dvec2 beyond{chain[i + 1].first, chain[i + 1].second};
                if (distance_to_segment(prev, beyond, here) > tolerance) continue;

                const std::vector<size_t>& owners = table.at(chain[i]).lots;
                bool safe = true;
                for (const size_t li : owners) safe = safe && ring_size[li] > 3;
                if (!safe) continue;
                for (const size_t li : owners) --ring_size[li];

                removed.insert(chain[i]);
                chain.erase(chain.begin() + static_cast<std::ptrdiff_t>(i));
                changed = true;
                --i;
            }
        }
    }

    if (removed.empty()) return 0;

    // No lot is ADDED, DROPPED or REORDERED here, and callers rely on that: the
    // caller separates street lots from backland ones by their index in this
    // vector. The ring-size guard above is what makes it safe -- every ring keeps at
    // least three vertices -- and a lot that somehow came out with no area is still
    // kept, because its land is real and a lot that is a sliver is a smaller problem
    // than land in no lot at all.
    for (Lot& lot : out.lots) {
        std::vector<glm::dvec2> ring;
        ring.reserve(lot.ring.size());
        for (const glm::dvec2& p : lot.ring) {
            if (removed.count(key_of(p)) == 0) ring.push_back(p);
        }
        if (ring.size() == lot.ring.size()) continue;
        lot.ring = std::move(ring);
        lot.area = ring_area(lot.ring);
        // Re-tagged rather than trusted: ring_edges is parallel to the ring, and the
        // ring just lost vertices.
        tag_frontage(segments, lot.ring, lot.ring_edges, lot.frontage_length);
    }
    return removed.size();
}

// ============================================================================
// Emission
// ============================================================================

/// Everything a piece needs before it can become a Lot
struct Piece {
    std::vector<glm::dvec2> ring;
    uint64_t key = 0;
    uint32_t depth = 0;
    bool is_corner = false;
};

/**
 * @brief Turn a piece into a Lot and add it to @p out
 *
 * The one place a Lot is constructed, so `id`, `node_key`, `area` and
 * `frontage_length` cannot disagree with each other about which piece they
 * describe.
 *
 * Simplification does NOT happen here, and the reason is the whole point of
 * simplify_shared_boundaries(): a vertex on a boundary two lots share cannot be
 * decided by one of them. It runs once, over all the lots, after the last one is
 * emitted.
 *
 * @return True when a lot was added. False means the piece had no area, and the
 *         caller must not count it: StripStats::street_pieces is a count of lots
 *         that exist, not of pieces that were offered.
 */
bool emit_lot(const Piece& piece, const std::vector<BoundarySegment>& segments,
              LotSubdivision& out) {
    std::vector<glm::dvec2> ring = piece.ring;
    const double area = ring_area(ring);
    if (ring.size() < 3 || area <= kAreaEpsilon) {
        ++out.stats.dropped_pieces;
        return false;
    }

    Lot lot;
    lot.node_key = piece.key;
    lot.id = lot_id_for_key(piece.key);
    lot.ring = std::move(ring);
    lot.area = area;
    lot.depth = piece.depth;
    lot.is_corner_lot = piece.is_corner;
    tag_frontage(segments, lot.ring, lot.ring_edges, lot.frontage_length);

    if (lot.is_corner_lot) ++out.stats.corner_lots;
    out.stats.max_depth_reached = std::max(out.stats.max_depth_reached, lot.depth);
    out.lots.push_back(std::move(lot));
    return true;
}

/**
 * @brief Hand a piece to the C2 recursion, then adopt what it produced
 *
 * Used for land with no frontage -- the yard behind an offset band, the backland
 * behind a shallow street lot. The C2 recursion is the right tool there: it is
 * exactly the OBB halving, the piece has no street to align to, and re-deriving a
 * second halving here would be a second implementation of C2 that could disagree
 * with the first.
 *
 * The ring_edges the recursion is handed are this file's own tags, so a yard piece
 * that does happen to touch a street -- a block with no interior at all -- still
 * reports its frontage and `force_street_access` still sees it.
 */
void recurse_piece(const Piece& piece, const std::vector<BoundarySegment>& segments,
                   const LotParams& params, LotSubdivision& out) {
    std::vector<uint32_t> tags;
    double frontage = 0.0;
    tag_frontage(segments, piece.ring, tags, frontage);

    LotSubdivision sub = subdivide_ring(piece.ring, tags, piece.key, params, piece.depth);
    if (sub.lots.empty()) {
        (void)emit_lot(piece, segments, out);
        return;
    }

    for (Lot& lot : sub.lots) {
        lot.area = ring_area(lot.ring);
        if (lot.ring.size() < 3 || lot.area <= kAreaEpsilon) {
            ++out.stats.dropped_pieces;
            continue;
        }
        // Re-tagged rather than trusted: the tags the recursion returned are
        // parallel to the ring it built, and this file tags against the BLOCK's
        // segments rather than against the piece's.
        tag_frontage(segments, lot.ring, lot.ring_edges, lot.frontage_length);
        out.stats.max_depth_reached = std::max(out.stats.max_depth_reached, lot.depth);
        out.lots.push_back(std::move(lot));
    }

    out.stats.splits += sub.stats.splits;
    out.stats.split_attempts += sub.stats.split_attempts;
    out.stats.rejected_area += sub.stats.rejected_area;
    out.stats.rejected_width += sub.stats.rejected_width;
    out.stats.rejected_access += sub.stats.rejected_access;
    out.stats.rejected_degenerate += sub.stats.rejected_degenerate;
    out.stats.dropped_pieces += sub.stats.dropped_pieces;
    out.stats.depth_limit_hits += sub.stats.depth_limit_hits;
}

// ============================================================================
// The Shared Setup
// ============================================================================

/// The block reduced to what both modes need: a clipped polygon and its faces
struct Setup {
    bool ok = false;
    Paths64 block_paths;                       ///< Outer ring and holes, quantised
    geometry::StraightSkeleton skeleton;
    std::vector<BoundarySegment> segments;
    double reach = 0.0;                        ///< Half-plane box size; see half_plane()
    uint64_t root_key = 0;
};

/**
 * @brief Compute the skeleton and the clip paths, or report that it cannot be done
 *
 * Both modes start here. The skeleton is computed on `Block::ring` and NOT on the
 * block's holes, because compute_straight_skeleton() takes one ring -- see its
 * scope notes. The holes are dealt with by subtracting them from every piece
 * afterwards, which is why `block_paths` carries them.
 *
 * The face edge indices are only usable if the skeleton's own contour is the ring
 * that was passed in. It normally is: Block::ring is anticlockwise with no repeated
 * point, which is exactly what the skeleton would reduce it to. When it is not the
 * indices would silently name the wrong streets, so the setup refuses instead.
 *
 * ### The comparison is point by point, and a size check is not enough
 *
 * compute_straight_skeleton() REVERSES a clockwise ring and hands the reversed one
 * back in `contour` -- which has the same size. So a size check alone passes a
 * clockwise block through, face `e` is then matched against `block.ring[e]` of the
 * UN-reversed ring, `perp(u)` points out of the block instead of into it, and
 * corner_widths()'s convexity cross product comes out inverted. Nothing downstream
 * can tell: `skeleton_complete` stays true, the lots still tile, and
 * `tag_frontage()` still gets the frontage right because it matches by geometry
 * rather than by index. What changes is the answer. On a 200 x 60 block with
 * `shallow_lot_frac` 0.5 the anticlockwise ring gives 14 lots with 5100 square
 * metres of backland; the same ring reversed gives 18 lots, no backland at all and
 * `shallow_lot_frac` silently ignored.
 *
 * Refusing rather than reversing the block's own indices: an author who handed this
 * a clockwise ring has a block that came from somewhere other than
 * extract_blocks(), and quietly renumbering their streets would move every lot id
 * in the block without saying so.
 */
[[nodiscard]] Setup make_setup(const Block& block, const LotParams& params,
                               uint64_t mode_salt) {
    Setup setup;
    if (block.ring.size() < 3) return setup;

    setup.root_key = mix2(lot_root_key(params), mode_salt);
    setup.segments = boundary_segments(block);

    Path64 outer;
    if (!to_path(block.ring, outer)) return setup;
    setup.block_paths.push_back(outer);
    for (const std::vector<glm::dvec2>& hole : block.holes) {
        // Block::holes are CLOCKWISE by design, and Clipper2's NonZero fill is why
        // that matters here rather than being a style note. A hole wound the same
        // way as the outer ring has a winding number of two inside it, which is
        // non-zero, so the fill keeps it: the courtyard comes back as land, every
        // boolean downstream is computed over a block that is bigger than the block,
        // and a lot ends up sitting on the hole. On a 160 x 160 block with a 40 x 40
        // courtyard that is 25200 square metres of lots over 24000 square metres of
        // ground, with `skeleton_complete` true and nothing else out of place.
        //
        // Refused rather than reversed, for make_setup()'s reason above: a block
        // whose holes are wound the other way did not come from extract_blocks(),
        // and quietly reinterpreting it would hide that from the one caller that can
        // do something about it.
        if (hole.size() >= 3 && geometry::signed_ring_area(hole) > 0.0) return setup;
        Path64 path;
        if (to_path(hole, path)) setup.block_paths.push_back(std::move(path));
    }

    glm::dvec2 lo = block.ring.front();
    glm::dvec2 hi = lo;
    for (const glm::dvec2& p : block.ring) {
        lo = glm::min(lo, p);
        hi = glm::max(hi, p);
    }
    // Four times the diagonal, so a half-plane box always covers the whole block
    // however the block sits relative to the cut. A box that is merely "big" is a
    // silent truncation of a lot at the far corner.
    setup.reach = 4.0 * glm::length(hi - lo) + 1.0;

    setup.skeleton = geometry::compute_straight_skeleton(block.ring);
    if (!setup.skeleton.complete) return setup;
    if (setup.skeleton.contour.size() != block.ring.size()) return setup;
    for (size_t i = 0; i < block.ring.size(); ++i) {
        if (glm::length(setup.skeleton.contour[i] - block.ring[i]) > kPointEpsilon) return setup;
    }

    setup.ok = true;
    return setup;
}

/**
 * @brief The straight-skeleton face of contour edge @p edge, or nullptr
 *
 * Scanned rather than indexed, because straight_skeleton.hpp says a face can be
 * MISSING -- one that came out with no area is dropped and counted in
 * SkeletonStats::degenerate_faces -- so `faces[i].edge == i` is not a promise.
 *
 * It is a promise that has held every time it has been looked at: 200000 random
 * polygons produced no dropped face at all, and no test fails when this is replaced
 * by `faces[edge]`. The scan is kept anyway. It costs a linear pass over a handful
 * of faces, and the failure it guards against is a whole run of lots attributed to
 * the wrong street, which nothing downstream would notice.
 */
[[nodiscard]] const geometry::SkeletonFace* face_of(const geometry::StraightSkeleton& skeleton,
                                                    uint32_t edge) {
    for (const geometry::SkeletonFace& face : skeleton.faces) {
        if (face.edge == edge) return &face;
    }
    return nullptr;
}

/**
 * @brief Discard lots that landed wholly inside a hole
 *
 * lots.cpp's test, unchanged and for its reason: a lot that WRAPS a hole has its
 * centroid in the middle of the hole, so a centroid test throws away every square
 * metre of real land around it. Requiring every vertex to be inside can only
 * discard a lot that is genuinely all hole.
 *
 * It normally discards nothing here, because the holes were subtracted before the
 * lots were cut. It is kept because the SKELETON is computed on the outer ring
 * alone, so a face can span a hole, and a piece of one that the boolean missed --
 * a hole smaller than the Clipper2 quantisation, say -- would otherwise become a
 * lot over ground that is not there.
 *
 * NOTHING IS KNOWN TO REACH IT, and that is worth saying rather than leaving for
 * the next reader to find out. Every piece is clipped against `block_paths`, which
 * carries the holes, before it becomes a lot; a hole too small for `to_path()` to
 * keep is also too small to contain a whole lot; and a hole wound the wrong way --
 * which the boolean really would not subtract -- is refused by make_setup()
 * instead. Deleting the filter leaves the whole suite green and the output
 * byte-identical over 120 random blocks with courtyards. `LotStats::lots_in_holes`
 * is therefore documented, correct, and zero on every input anyone has produced.
 */
void drop_lots_inside_holes(const Block& block, LotSubdivision& result) {
    if (block.holes.empty() || result.lots.empty()) return;

    const size_t before = result.lots.size();
    std::vector<Lot> kept;
    kept.reserve(before);
    for (Lot& lot : result.lots) {
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

// ============================================================================
// Cutting a Face into a Run of Lots
// ============================================================================

/**
 * @brief How many lots to cut a run of @p length metres and @p area square metres into
 *
 * Area first, then width, then the floor -- in that order, because they pull
 * against each other and the order decides which one wins:
 *
 *   1. Enough lots that none exceeds `lot_area_max`. That is the constraint an
 *      author sets to get parcels of a size.
 *   2. Never so many that one is narrower than `lot_width_min`. A run of narrow
 *      unbuildable ribbons is worse than one lot that is too big, and this is the
 *      constraint that stops a long shallow strip being shaved into nothing.
 *   3. Never so many that one falls below `lot_area_min`, which is a floor on
 *      SPLITTING and not on lots: a run that is small to begin with comes out as
 *      one lot rather than as none.
 *
 * @param out_rejected_width Incremented when constraint 2 reduced the count
 * @param out_rejected_area  Incremented when constraint 3 reduced the count
 */
[[nodiscard]] size_t lots_along_run(double length, double area, const LotParams& params,
                                    size_t& out_rejected_width, size_t& out_rejected_area) {
    if (!(length > kPointEpsilon) || !(area > kAreaEpsilon)) return 1;

    size_t count = 1;
    if (params.lot_area_max > 0.0) {
        const double wanted = std::ceil(area / params.lot_area_max - 1e-9);
        count = static_cast<size_t>(std::max(1.0, std::min(wanted, 4096.0)));
    }

    if (params.lot_width_min > 0.0) {
        const auto by_width =
            static_cast<size_t>(std::max(1.0, std::floor(length / params.lot_width_min)));
        if (by_width < count) {
            ++out_rejected_width;
            count = by_width;
        }
    }

    if (params.lot_area_min > 0.0) {
        const auto by_area =
            static_cast<size_t>(std::max(1.0, std::floor(area / params.lot_area_min)));
        if (by_area < count) {
            ++out_rejected_area;
            count = by_area;
        }
    }
    return std::max<size_t>(count, 1);
}

/**
 * @brief Where along a run of @p length the @p count lots are divided
 *
 * `count - 1` interior positions, evenly spaced and then nudged by
 * `LotParams::irregularity`. The nudge is at most a quarter of a lot's width, so
 * two adjacent positions can never cross and the pieces can never come out in a
 * different order -- which is what keeps the index in the node key meaning the same
 * piece from one run to the next.
 *
 * The draw is `mix2(key, kSaltJitter ^ index)` and nothing else: no shared stream,
 * so inserting one more lot at the start of a run does not move the rest.
 */
[[nodiscard]] std::vector<double> run_cut_positions(double length, size_t count, uint64_t key,
                                                    double irregularity) {
    std::vector<double> positions;
    if (count < 2) return positions;
    const double step = length / static_cast<double>(count);
    const double swing = 0.25 * step * std::clamp(irregularity, 0.0, 1.0);
    positions.reserve(count - 1);
    for (size_t i = 1; i < count; ++i) {
        const double jitter =
            signed_unit_from(mix2(key, kSaltJitter ^ static_cast<uint64_t>(i))) * swing;
        positions.push_back(static_cast<double>(i) * step + jitter);
    }
    return positions;
}

/**
 * @brief Cut a face's core into a run of lots square to its street
 *
 * The core arrives already clipped: to the block, to the band in the offset mode,
 * to the shallow depth, and to the stretch of street left over by the corner lots
 * at each end. All that happens here is the run itself -- slabs perpendicular to
 * the street, cut with Clipper2 against half-planes so that a cut passing exactly
 * through a face vertex is decided once, in integer arithmetic, rather than by a
 * tolerance in this file.
 *
 * @param s0 Distance along the street where the run starts, from @p a
 * @param s1 Distance along the street where it ends
 */
void cut_face_run(const std::vector<Region>& core, const glm::dvec2& a, const glm::dvec2& b,
                  double s0, double s1, uint64_t face_key, const LotParams& params, double reach,
                  std::vector<Piece>& out_street, LotStats& stats) {
    const double edge_length = glm::length(b - a);
    const double run_length = s1 - s0;
    if (edge_length <= kPointEpsilon || run_length <= kPointEpsilon || core.empty()) return;
    const glm::dvec2 u = (b - a) / edge_length;

    const Paths64 subject = regions_to_paths(core);
    if (subject.empty()) return;

    double core_area = 0.0;
    for (const Region& region : core) core_area += region.area;
    if (core_area <= kAreaEpsilon) return;

    const size_t count = lots_along_run(run_length, core_area, params, stats.rejected_width,
                                        stats.rejected_area);
    const std::vector<double> cuts =
        run_cut_positions(run_length, count, face_key, params.irregularity);
    ++stats.split_attempts;
    if (count > 1) stats.splits += count - 1;

    for (size_t i = 0; i < count; ++i) {
        Paths64 slab = subject;
        if (i > 0) {
            const Paths64 keep{half_plane(a + u * (s0 + cuts[i - 1]), u, reach)};
            slab = regions_to_paths(boolean_regions(ClipType::Intersection, slab, keep));
        }
        if (i + 1 < count && !slab.empty()) {
            const Paths64 keep{half_plane(a + u * (s0 + cuts[i]), -u, reach)};
            slab = regions_to_paths(boolean_regions(ClipType::Intersection, slab, keep));
        }
        const std::vector<Region> regions = split_until_simple(to_regions(slab), reach);
        for (size_t r = 0; r < regions.size(); ++r) {
            Piece piece;
            piece.ring = regions[r].outer;
            piece.key = region_key(mix2(face_key, mix2(kSaltPiece, static_cast<uint64_t>(i))), r);
            piece.depth = 1;
            out_street.push_back(std::move(piece));
        }
    }
}

/**
 * @brief How far back from each block vertex a corner lot reaches, in metres
 *
 * ### Why a corner lot, rather than clipping the face square
 *
 * The obvious reading of "make the end of a run perpendicular to the street" is to
 * clip each skeleton face by a line square to its own street at each end. That
 * does nothing, and it took a probe to see why: at a convex corner the two faces
 * already meet along the bisector, and the bisector is entirely on the kept side
 * of both perpendicular lines. Squaring the bottom street's end lot into a
 * rectangle does not remove land from the bottom face, it takes land from the LEFT
 * face -- and doing that for both streets would have them claim the same corner.
 *
 * So the corner is given to neither. Each street gives up its last
 * `corner_alignment * LotParams::corner_width` metres, the two give-ups are merged
 * into ONE lot that fronts both streets, and the runs either side of it are then
 * genuinely rectangular. That is the parcel lots.cpp already calls a corner lot,
 * and the same reason is behind it: the wedge at a block corner is a shape no
 * building fits in.
 *
 * ### Why the width is per VERTEX and not per face
 *
 * Both faces at a corner read the same entry of this table, so the land one gives
 * up is exactly the land the other's corner lot takes. Computing it inside each
 * face instead would let two clamps disagree by a rounding error and leave a
 * sliver of unclaimed land between the two -- land that is in no lot at all, which
 * the area-conservation test is what catches.
 *
 * Only CONVEX corners are eligible, which is lots.cpp's rule too: a reflex corner
 * has no wedge to remove. The clamp to half of each incident edge is also
 * lots.cpp's, and it is what stops two corners at the ends of a short street from
 * cutting overlapping lots out of it.
 *
 * @return One width per block ring VERTEX, zero where no corner lot is cut
 */
[[nodiscard]] std::vector<double> corner_widths(const std::vector<glm::dvec2>& ring,
                                                double alignment, double corner_width) {
    const size_t n = ring.size();
    std::vector<double> widths(n, 0.0);
    const double wanted = std::clamp(alignment, 0.0, 1.0) * std::max(0.0, corner_width);
    if (!(wanted > 0.0) || n < 3) return widths;

    for (size_t v = 0; v < n; ++v) {
        const glm::dvec2 prev = ring[(v + n - 1) % n];
        const glm::dvec2 here = ring[v];
        const glm::dvec2 next = ring[(v + 1) % n];
        const double in_length = glm::length(here - prev);
        const double out_length = glm::length(next - here);
        if (in_length <= kPointEpsilon || out_length <= kPointEpsilon) continue;
        // Convex for an anticlockwise ring means a left turn, which is a positive
        // cross product. A cross product rather than an angle: acos is libm and
        // libm is not bit-reproducible, and this decides a coordinate.
        if (cross2((here - prev) / in_length, (next - here) / out_length) <= 0.0) continue;
        widths[v] = std::min(wanted, 0.5 * std::min(in_length, out_length));
    }
    return widths;
}

}  // namespace

// ============================================================================
// Offset Subdivision
// ============================================================================

std::vector<std::vector<std::vector<glm::dvec2>>> block_offset_band(
    const Block& block, double width, std::vector<std::vector<glm::dvec2>>* out_interior) {
    if (out_interior != nullptr) out_interior->clear();

    LotParams defaults;
    const Setup setup = make_setup(block, defaults, kSaltOffsetMode);
    if (!setup.ok) return {};

    std::vector<std::vector<std::vector<glm::dvec2>>> band(block.ring.size());

    // The yard is Clipper2's own inward offset of the block, holes and all. Taking
    // it from the offset rather than from "everything the band did not cover" means
    // the two are computed independently and a test can hold them against each
    // other.
    Paths64 interior;
    if (width > 0.0) {
        interior = Clipper2Lib::InflatePaths(setup.block_paths, -width * kClipperScale,
                                             Clipper2Lib::JoinType::Miter,
                                             Clipper2Lib::EndType::Polygon);
        canonicalise(interior);
    } else {
        interior = setup.block_paths;
    }

    const std::vector<Region> band_regions =
        boolean_regions(ClipType::Difference, setup.block_paths, interior);
    const Paths64 band_paths = regions_to_paths(band_regions);

    for (uint32_t e = 0; e < block.ring.size(); ++e) {
        const geometry::SkeletonFace* face = face_of(setup.skeleton, e);
        if (face == nullptr || band_paths.empty()) continue;
        Path64 face_path;
        if (!to_path(face->ring, face_path)) continue;
        for (const Region& region : split_until_simple(
                 boolean_regions(ClipType::Intersection, Paths64{face_path}, band_paths),
                 setup.reach)) {
            band[e].push_back(region.outer);
        }
    }

    if (out_interior != nullptr) {
        for (const Region& region : split_until_simple(
                 boolean_regions(ClipType::Intersection, interior, setup.block_paths),
                 setup.reach)) {
            out_interior->push_back(region.outer);
        }
    }
    return band;
}

StripSubdivision subdivide_block_offset(const Block& block, const OffsetLotParams& params) {
    StripSubdivision result;

    const Setup setup = make_setup(block, params.lot, kSaltOffsetMode);
    if (!setup.ok) return result;
    result.stats.skeleton_complete = true;
    result.stats.faces = setup.skeleton.faces.size();

    const double width = std::max(0.0, params.offset_width);

    // A band of zero depth is a legitimate way to ask for the C2 recursion through
    // this entry point, so the yard is the WHOLE block and the band is empty. Left
    // as an empty path set instead, the band would come out as `block minus
    // nothing`, which is the entire block cut into street lots -- the exact opposite
    // of what was asked for, and silent.
    Paths64 interior_paths = setup.block_paths;
    if (width > 0.0) {
        interior_paths = Clipper2Lib::InflatePaths(setup.block_paths, -width * kClipperScale,
                                                   Clipper2Lib::JoinType::Miter,
                                                   Clipper2Lib::EndType::Polygon);
        canonicalise(interior_paths);
        // Belt and braces, and labelled as such. The intent is that a mitred offset
        // cannot wander outside the block at a sharp concave corner, where the miter
        // limit squares a spike off; intersecting it back makes the yard a subset of
        // the block by construction, whatever Clipper2's offset did. No input has
        // been found where removing this changes the answer -- 4500 subdivisions of
        // spiky random blocks were identical with and without it -- so it is a
        // guard, not a fix, and there is no test that fails when it goes.
        interior_paths = regions_to_paths(
            boolean_regions(ClipType::Intersection, interior_paths, setup.block_paths));
    }

    const Paths64 band_paths = regions_to_paths(
        boolean_regions(ClipType::Difference, setup.block_paths, interior_paths));

    // --- The band, one street at a time -------------------------------------
    std::vector<Piece> street;
    for (uint32_t e = 0; e < block.ring.size(); ++e) {
        const geometry::SkeletonFace* face = face_of(setup.skeleton, e);
        if (face == nullptr || band_paths.empty()) continue;

        Path64 face_path;
        if (!to_path(face->ring, face_path)) continue;
        const std::vector<Region> piece_regions =
            boolean_regions(ClipType::Intersection, Paths64{face_path}, band_paths);
        if (piece_regions.empty()) {
            ++result.stats.empty_clips;
            continue;
        }

        const uint64_t face_key = mix2(setup.root_key, mix2(kSaltFace, e));
        const glm::dvec2 a = block.ring[e];
        const glm::dvec2 b = block.ring[(e + 1) % block.ring.size()];
        // No corner lots in this mode. A band already turns the corner, so the
        // parcel at a block corner is a band lot bent round it rather than a wedge
        // reaching back to the middle of the block -- the shape corner lots exist
        // to remove is not produced here. The run therefore spans the whole street.
        cut_face_run(piece_regions, a, b, 0.0, glm::length(b - a), face_key, params.lot,
                     setup.reach, street, result.subdivision.stats);
    }

    for (const Piece& piece : street) {
        if (emit_lot(piece, setup.segments, result.subdivision)) ++result.stats.street_pieces;
    }
    // Accumulated from the LOTS and not from the pieces they were cut from, so that
    // StripStats and Lot::area cannot describe different land. Taking it from the
    // pieces let a piece that emit_lot() dropped, or a ring that simplification
    // changed, count towards a total no lot backs.
    for (const Lot& lot : result.subdivision.lots) result.stats.street_area += lot.area;

    // --- The yard behind it --------------------------------------------------
    // Cut through any courtyard first. A yard that wraps a hole cannot become a lot
    // -- Lot::ring is one ring -- and emitting its outer ring instead covers the
    // hole, which is the defect that put 30400 square metres of lots over 24000
    // square metres of land.
    const std::vector<Region> yard =
        split_until_simple(to_regions(interior_paths), setup.reach);
    result.stats.interior_pieces = yard.size();
    const size_t before_yard = result.subdivision.lots.size();
    for (size_t i = 0; i < yard.size(); ++i) {
        Piece piece;
        piece.ring = yard[i].outer;
        piece.key = mix2(setup.root_key, mix2(kSaltInterior, static_cast<uint64_t>(i)));
        piece.depth = 1;
        recurse_piece(piece, setup.segments, params.lot, result.subdivision);
    }
    for (size_t i = before_yard; i < result.subdivision.lots.size(); ++i) {
        result.stats.interior_area += result.subdivision.lots[i].area;
    }

    drop_lots_inside_holes(block, result.subdivision);
    result.subdivision.stats.lots = result.subdivision.lots.size();
    return result;
}

// ============================================================================
// Skeleton Subdivision
// ============================================================================

StripSubdivision subdivide_block_skeleton(const Block& block, const SkeletonLotParams& params) {
    StripSubdivision result;

    const Setup setup = make_setup(block, params.lot, kSaltSkeletonMode);
    if (!setup.ok) return result;
    result.stats.skeleton_complete = true;
    result.stats.faces = setup.skeleton.faces.size();

    const double shallow = std::clamp(params.shallow_lot_frac, 0.0, 1.0);
    const double simplify = std::max(0.0, params.simplify);

    std::vector<Piece> street;
    std::vector<Piece> corner;
    std::vector<Piece> backland;

    const size_t n_ring = block.ring.size();
    const std::vector<double> corner_w =
        corner_widths(block.ring, params.corner_alignment, params.lot.corner_width);
    // One accumulator per block VERTEX, so the give-up from the street before it and
    // the give-up from the street after it end as ONE lot rather than two slivers
    // that meet at the corner and front one street each.
    std::vector<Paths64> corner_zone(n_ring);

    for (uint32_t e = 0; e < n_ring; ++e) {
        const geometry::SkeletonFace* face = face_of(setup.skeleton, e);
        if (face == nullptr) continue;

        const glm::dvec2 a = block.ring[e];
        const glm::dvec2 b = block.ring[(e + 1) % n_ring];
        const double length = glm::length(b - a);
        if (length <= kPointEpsilon) continue;
        const glm::dvec2 u = (b - a) / length;
        const glm::dvec2 n = perp(u);

        Path64 face_path;
        if (!to_path(face->ring, face_path)) continue;
        // Clipped to the block, which subtracts the holes. The skeleton was
        // computed on the outer ring alone and knows nothing about them, so a face
        // over a courtyard covers land that is not there until this runs.
        std::vector<Region> core =
            boolean_regions(ClipType::Intersection, Paths64{face_path}, setup.block_paths);
        if (core.empty()) {
            ++result.stats.empty_clips;
            continue;
        }

        const uint64_t face_key = mix2(setup.root_key, mix2(kSaltFace, e));

        // --- Shallow lots, and the backland behind them -----------------------
        //
        // The depth is a fraction of THIS face's depth, not of the block's. A
        // wedge-shaped block's two sides are not the same depth, and one block-wide
        // number would leave the shallow side with no backland and put most of the
        // deep side's land in it.
        if (shallow < 1.0) {
            double face_depth = 0.0;
            for (const double t : face->times) face_depth = std::max(face_depth, t);
            const double depth = face_depth * shallow;
            const Paths64 keep{half_plane(a + n * depth, -n, setup.reach)};
            const Paths64 subject = regions_to_paths(core);
            const std::vector<Region> behind = split_until_simple(
                boolean_regions(ClipType::Difference, subject, keep), setup.reach);
            for (size_t r = 0; r < behind.size(); ++r) {
                Piece piece;
                piece.ring = behind[r].outer;
                piece.key = region_key(mix2(face_key, kSaltBackland), r);
                piece.depth = 1;
                backland.push_back(std::move(piece));
            }
            core = boolean_regions(ClipType::Intersection, subject, keep);
            if (core.empty()) continue;
        }

        // --- Give the corners up, before the run is cut ----------------------
        //
        // Taken from the ends of the run rather than from the ends of the EDGE, so
        // that the same width is removed whether or not the shallow clip above ran.
        double s0 = std::min(corner_w[e], 0.5 * length);
        double s1 = length - std::min(corner_w[(e + 1) % n_ring], 0.5 * length);
        if (s1 < s0) {
            s0 = 0.0;
            s1 = length;
        }
        if (s0 > kPointEpsilon) {
            const Paths64 zone{half_plane(a + u * s0, -u, setup.reach)};
            const Paths64 subject = regions_to_paths(core);
            const Paths64 taken =
                regions_to_paths(boolean_regions(ClipType::Intersection, subject, zone));
            for (const Path64& path : taken) corner_zone[e].push_back(path);
            core = boolean_regions(ClipType::Difference, subject, zone);
        }
        if (s1 < length - kPointEpsilon && !core.empty()) {
            const Paths64 zone{half_plane(a + u * s1, u, setup.reach)};
            const Paths64 subject = regions_to_paths(core);
            const Paths64 taken =
                regions_to_paths(boolean_regions(ClipType::Intersection, subject, zone));
            for (const Path64& path : taken) corner_zone[(e + 1) % n_ring].push_back(path);
            core = boolean_regions(ClipType::Difference, subject, zone);
        }
        if (core.empty()) continue;

        cut_face_run(core, a, b, s0, s1, face_key, params.lot, setup.reach, street,
                     result.subdivision.stats);
    }

    // --- The corner lots -----------------------------------------------------
    for (size_t v = 0; v < n_ring; ++v) {
        if (corner_zone[v].empty()) continue;
        // A union, not two pieces: the land given up by the street before the corner
        // and the land given up by the street after it are one parcel, fronting both.
        const std::vector<Region> merged = split_until_simple(
            boolean_regions(ClipType::Union, corner_zone[v], Paths64{}), setup.reach);
        for (size_t r = 0; r < merged.size(); ++r) {
            Piece piece;
            piece.ring = merged[r].outer;
            piece.key = region_key(
                mix2(setup.root_key, mix2(kSaltCorner, static_cast<uint64_t>(v))), r);
            piece.depth = 1;
            piece.is_corner = true;
            corner.push_back(std::move(piece));
        }
    }

    for (const Piece& piece : street) {
        if (emit_lot(piece, setup.segments, result.subdivision)) ++result.stats.street_pieces;
    }
    for (const Piece& piece : corner) {
        if (emit_lot(piece, setup.segments, result.subdivision)) ++result.stats.corner_pieces;
    }
    const size_t street_and_corner = result.subdivision.lots.size();
    for (const Piece& piece : backland) {
        recurse_piece(piece, setup.segments, params.lot, result.subdivision);
    }
    result.stats.backland_pieces = backland.size();

    // ONE simplification pass over every lot in the block, after the last of them
    // exists and before anything reads an area or a tag. Per-lot simplification
    // cannot be correct here: a vertex on a boundary two lots share has to be kept
    // by both or dropped by both, and only a pass that can see both can decide
    // that. See simplify_shared_boundaries().
    result.stats.simplified_vertices =
        simplify_shared_boundaries(setup.segments, simplify, result.subdivision);

    // The areas come from the LOTS, so StripStats and Lot::area cannot disagree
    // about the same land -- which they did while the totals were accumulated from
    // the unsimplified pieces and the areas recomputed after simplification.
    for (size_t i = 0; i < result.subdivision.lots.size(); ++i) {
        if (i < street_and_corner) {
            result.stats.street_area += result.subdivision.lots[i].area;
        } else {
            result.stats.interior_area += result.subdivision.lots[i].area;
        }
    }

    drop_lots_inside_holes(block, result.subdivision);
    result.subdivision.stats.lots = result.subdivision.lots.size();
    return result;
}

}  // namespace stratum::osm::road
