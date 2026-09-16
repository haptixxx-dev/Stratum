// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_lots_offset.cpp
 * @brief The offset and skeleton lot subdivisions (C3)
 *
 * Every block here is written out as a ring rather than extracted from a fixture,
 * for the reason test_lots.cpp gives: the point of a test is an answer stated in
 * advance, and a fixture leaves the count assertable and nothing else.
 *
 * Three assertions run through the whole suite, and each is here because the
 * failure mode of a subdivision is silent -- a block that came out as one lot looks
 * exactly like a block that was correctly left alone.
 *
 *   - **The lots TILE the block.** Their areas sum to the block's area, and a grid
 *     of sample points over the block finds every interior point in exactly one
 *     lot. The area sum alone would pass two lots that overlap by as much as a
 *     third lot is missing; the sample alone would miss a lot outside the block.
 *     Together they say the lots partition the land.
 *   - **Every street lot fronts a street.** That is the entire promise of these two
 *     modes over the OBB recursion, and `Lot::frontage_length` is where a caller
 *     reads it. A landlocked lot in a skeleton subdivision is a defect, not a
 *     tuning problem.
 *   - **The negative case next to the positive one.** A determinism test passes
 *     trivially against an implementation that ignores its seed, so the test that
 *     asserts two runs agree sits beside one asserting that two IRREGULARITIES
 *     disagree. The same pairing guards corner alignment, shallow lots and
 *     simplification.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests LotsOffset
 * @endcode
 */

#include "framework.hpp"

#include "osm/road/blocks.hpp"
#include "osm/road/lots.hpp"
#include "osm/road/lots_offset.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <utility>
#include <vector>

using stratum::osm::road::Block;
using stratum::osm::road::BlockEdge;
using stratum::osm::road::kNoBlockEdge;
using stratum::osm::road::Lot;
using stratum::osm::road::LotId;
using stratum::osm::road::LotParams;
using stratum::osm::road::OffsetLotParams;
using stratum::osm::road::signed_ring_area;
using stratum::osm::road::SkeletonLotParams;
using stratum::osm::road::StripSubdivision;
using stratum::osm::road::block_offset_band;
using stratum::osm::road::subdivide_block;
using stratum::osm::road::subdivide_block_offset;
using stratum::osm::road::subdivide_block_skeleton;

namespace {

using Ring = std::vector<glm::dvec2>;

// ============================================================================
// Blocks
// ============================================================================

/**
 * @brief Wrap a ring as a Block, tagging segment i with block edge i
 *
 * One block edge per ring segment, the same tagging test_lots.cpp uses and for the
 * same reason: it is the simplest that is still distinguishable, so a lot that
 * comes back with the wrong tag names a different street and the tests below can
 * say which tag each lot must carry.
 */
Block make_block(Ring ring) {
    Block block;
    block.ring = std::move(ring);
    block.ring_edges.resize(block.ring.size());
    block.edges.resize(block.ring.size());
    for (size_t i = 0; i < block.ring.size(); ++i) {
        block.ring_edges[i] = static_cast<uint32_t>(i);
        block.edges[i] = BlockEdge{static_cast<uint32_t>(i), true};
    }
    block.area = signed_ring_area(block.ring);
    return block;
}

/// Anticlockwise axis-aligned rectangle with its lower-left corner at the origin
Ring rect(double width, double height) {
    return Ring{{0.0, 0.0}, {width, 0.0}, {width, height}, {0.0, height}};
}

/// 120 x 120 L: a 120 x 40 arm along the bottom and a 40 x 120 arm up the left
Ring ell() {
    return Ring{{0.0, 0.0},  {120.0, 0.0},  {120.0, 40.0},
                {40.0, 40.0}, {40.0, 120.0}, {0.0, 120.0}};
}

/**
 * @brief A 200 x 60 block whose bottom kerb sags 1.5 m in the middle
 *
 * The block the simplification defect needed and a straight-street block cannot
 * provide. The sag is a parabola rather than a sine so the coordinates come out of
 * +, - and * alone and the shape is the same on every platform. Every side carries
 * interior polyline points, which is what a block extracted from real streets has.
 */
Ring curved_kerb_block_ring() {
    Ring ring;
    for (int i = 0; i < 12; ++i) {
        const double u = static_cast<double>(i) / 12.0;
        ring.push_back({ 200.0 * u, -1.5 * 4.0 * u * (1.0 - u) });
    }
    for (int i = 0; i < 4; ++i) ring.push_back({ 200.0, 60.0 * i / 4.0 });
    for (int i = 0; i < 12; ++i) ring.push_back({ 200.0 - 200.0 * i / 12.0, 60.0 });
    for (int i = 0; i < 4; ++i) ring.push_back({ 0.0, 60.0 - 60.0 * i / 4.0 });
    return ring;
}

/**
 * @brief An irregular block on which SkeletonLotParams::simplify actually removes something
 *
 * Found by searching six hundred random star blocks, and kept verbatim for the
 * reason every other ring in this file is kept verbatim. What it has that a
 * rectangle does not is a bisector arc between two adjacent faces carrying more
 * than two nodes, with both lots that use it keeping the whole chain -- which is
 * the only thing the shared-boundary simplification can touch. On an axis-aligned
 * rectangle it removes nothing at any tolerance, correctly.
 */
Ring simplifiable_block_ring() {
    return Ring{ { 33.288599772271766, 0.0 },
                 { 57.453225467181383, 36.922946020211228 },
                 { 30.282511216744062, 66.309450164978500 },
                 { -3.9074455066578286, 27.176880446185027 },
                 { -35.426567362957790, 40.884438198160161 },
                 { -44.108647915714108, 12.951467595744331 },
                 { -30.290296226918606, -8.8940334511743409 },
                 { -24.912507988951408, -28.750566850019997 },
                 { -4.6703082267241403, -32.482707208136837 },
                 { 16.300553663634769, -35.693233730982620 },
                 { 25.907907482812561, -16.650001139291032 } };
}

/// Parameters with every source of variation switched off unless a test asks
LotParams base_lot_params() {
    LotParams params;
    params.corner_angle_max_deg = 0.0;
    params.corner_width = 12.0;
    params.irregularity = 0.0;
    params.force_street_access = 1.0;
    params.lot_area_min = 200.0;
    params.lot_area_max = 800.0;
    params.lot_width_min = 8.0;
    params.block_key = 0x0123456789ABCDEFull;
    params.seed = 7;
    return params;
}

OffsetLotParams offset_params(double width = 25.0) {
    OffsetLotParams params;
    params.lot = base_lot_params();
    params.offset_width = width;
    return params;
}

SkeletonLotParams skeleton_params() {
    SkeletonLotParams params;
    params.lot = base_lot_params();
    return params;
}

// ============================================================================
// Geometry Helpers
// ============================================================================

double cross2(const glm::dvec2& a, const glm::dvec2& b) {
    return a.x * b.y - a.y * b.x;
}

/// Crossing-number point-in-polygon; boundary cases are not distinguished
bool point_in_ring(const Ring& ring, const glm::dvec2& p) {
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

double distance_to_ring(const Ring& ring, const glm::dvec2& p) {
    double best = 1e300;
    for (size_t i = 0; i < ring.size(); ++i) {
        const glm::dvec2 a = ring[i];
        const glm::dvec2 b = ring[(i + 1) % ring.size()];
        const glm::dvec2 ab = b - a;
        const double len2 = glm::dot(ab, ab);
        double t = 0.0;
        if (len2 > 0.0) t = std::clamp(glm::dot(p - a, ab) / len2, 0.0, 1.0);
        best = std::min(best, glm::length(p - (a + ab * t)));
    }
    return best;
}

/// Sum of the lot areas
double total_area(const std::vector<Lot>& lots) {
    double total = 0.0;
    for (const Lot& lot : lots) total += lot.area;
    return total;
}

/**
 * @brief How a grid of sample points over @p block is covered by @p rings
 *
 * The check the area sum cannot make on its own. Two lots that overlap by as much
 * as a third is missing sum to exactly the right number; a sample point in the
 * overlap is in two rings and a point in the gap is in none.
 *
 * The grid is offset by an irrational-looking fraction of a cell so that samples
 * do not land on the axis-aligned lot boundaries these blocks are full of. A
 * sample exactly on a shared boundary is in both lots or in neither depending on
 * the last bit of a subtraction, and that is noise rather than a defect.
 */
struct Coverage {
    size_t inside = 0;    ///< Samples inside the block
    size_t covered = 0;   ///< Of those, in exactly one ring
    size_t gaps = 0;      ///< Of those, in no ring at all
    size_t overlaps = 0;  ///< Of those, in two or more rings
};

Coverage sample_coverage(const Block& block, const std::vector<Ring>& rings, size_t steps = 60) {
    glm::dvec2 lo = block.ring.front();
    glm::dvec2 hi = lo;
    for (const glm::dvec2& p : block.ring) {
        lo = glm::min(lo, p);
        hi = glm::max(hi, p);
    }

    Coverage coverage;
    for (size_t i = 0; i < steps; ++i) {
        for (size_t j = 0; j < steps; ++j) {
            const double fx = (static_cast<double>(i) + 0.3183098861837907) /
                              static_cast<double>(steps);
            const double fy = (static_cast<double>(j) + 0.2718281828459045) /
                              static_cast<double>(steps);
            const glm::dvec2 p{lo.x + (hi.x - lo.x) * fx, lo.y + (hi.y - lo.y) * fy};
            if (!point_in_ring(block.ring, p)) continue;
            bool in_hole = false;
            for (const Ring& hole : block.holes) in_hole = in_hole || point_in_ring(hole, p);
            if (in_hole) continue;
            // Only samples comfortably inside the block are judged. A sample a
            // micron from the kerb is on a lot boundary as often as not, and
            // counting it would make the test fail on rounding rather than on
            // coverage.
            if (distance_to_ring(block.ring, p) < 0.05) continue;

            ++coverage.inside;
            size_t hits = 0;
            for (const Ring& ring : rings) {
                if (point_in_ring(ring, p)) ++hits;
            }
            if (hits == 0) ++coverage.gaps;
            else if (hits == 1) ++coverage.covered;
            else ++coverage.overlaps;
        }
    }
    return coverage;
}

std::vector<Ring> lot_rings(const std::vector<Lot>& lots) {
    std::vector<Ring> rings;
    rings.reserve(lots.size());
    for (const Lot& lot : lots) rings.push_back(lot.ring);
    return rings;
}

/**
 * @brief The three things that must be true of any complete subdivision
 *
 * Called from most tests below rather than repeated in them, so a new block is one
 * line and cannot accidentally be checked more weakly than the others.
 */
void check_tiles_block(const Block& block, const StripSubdivision& result) {
    CHECK_TRUE(result.stats.skeleton_complete);
    if (!result.stats.skeleton_complete) return;
    CHECK((result.subdivision.lots.size()) > size_t{0});

    double expected = signed_ring_area(block.ring);
    for (const Ring& hole : block.holes) expected -= std::fabs(signed_ring_area(hole));
    CHECK_NEAR(total_area(result.subdivision.lots), expected, 1e-3 * expected);

    const Coverage coverage = sample_coverage(block, lot_rings(result.subdivision.lots));
    CHECK((coverage.inside) > size_t{100});
    CHECK_EQ(coverage.overlaps, size_t{0});
    CHECK_EQ(coverage.gaps, size_t{0});

    std::set<LotId> ids;
    for (const Lot& lot : result.subdivision.lots) {
        CHECK((lot.area) > 0.0);
        CHECK((lot.ring.size()) >= size_t{3});
        // Anticlockwise, the convention Block::ring and Corridor::outline share, so
        // all three go to the same triangulation code with no special case.
        CHECK((signed_ring_area(lot.ring)) > 0.0);
        CHECK_EQ(lot.ring_edges.size(), lot.ring.size());
        CHECK((lot.id) != stratum::osm::road::kInvalidLotId);
        CHECK_TRUE(ids.insert(lot.id).second);
    }
    CHECK_EQ(result.subdivision.stats.lots, result.subdivision.lots.size());
}

/**
 * @brief Group lot ids by the SET of block edges each lot fronts
 *
 * What "the lots fronting street 3" means when a corner lot fronts two streets. A
 * lot is filed under the exact set of tags on its ring, so `{3}` is the lots that
 * front only street 3 and `{2,3}` is the corner lot between them -- and comparing
 * the same key across two subdivisions compares like with like.
 */
std::map<std::set<uint32_t>, std::set<LotId>> ids_by_frontage(const std::vector<Lot>& lots) {
    std::map<std::set<uint32_t>, std::set<LotId>> grouped;
    for (const Lot& lot : lots) {
        std::set<uint32_t> edges;
        for (const uint32_t tag : lot.ring_edges) {
            if (tag != kNoBlockEdge) edges.insert(tag);
        }
        grouped[edges].insert(lot.id);
    }
    return grouped;
}

/// How many of @p wanted appear in @p have
size_t retained(const std::set<LotId>& wanted, const std::set<LotId>& have) {
    size_t count = 0;
    for (const LotId id : wanted) {
        if (have.count(id) != 0) ++count;
    }
    return count;
}

/// Total vertices over every lot ring
size_t total_vertices(const std::vector<Lot>& lots) {
    size_t total = 0;
    for (const Lot& lot : lots) total += lot.ring.size();
    return total;
}

/// The lots that came off a street, and the ones that did not
std::vector<Lot> with_frontage(const std::vector<Lot>& lots, bool wanted) {
    std::vector<Lot> out;
    for (const Lot& lot : lots) {
        if ((lot.frontage_length > 0.0) == wanted) out.push_back(lot);
    }
    return out;
}

} // namespace

// ============================================================================
// The Band Itself
// ============================================================================

/**
 * @brief The offset band tiles the block, one entry per street
 *
 * `block_offset_band()` is the decomposition every offset lot is cut out of, so it
 * is worth asserting on its own: the band pieces plus the yard are the whole block,
 * and no point of the block is in two of them.
 *
 * Asserted on a 200 x 60 block with a 25 m band, where the answer is known by hand:
 * the yard is the inward offset, 150 x 10, which is 1500 square metres, and the
 * band is the other 10500.
 */
TEST(LotsOffset, the_offset_band_and_the_yard_are_the_whole_block) {
    const Block block = make_block(rect(200.0, 60.0));

    std::vector<Ring> interior;
    const auto band = block_offset_band(block, 25.0, &interior);
    CHECK_EQ(band.size(), block.ring.size());

    double band_area = 0.0;
    std::vector<Ring> all;
    for (const auto& per_street : band) {
        for (const Ring& ring : per_street) {
            band_area += signed_ring_area(ring);
            all.push_back(ring);
        }
    }
    double interior_area = 0.0;
    for (const Ring& ring : interior) {
        interior_area += signed_ring_area(ring);
        all.push_back(ring);
    }

    CHECK_NEAR(interior_area, 1500.0, 1.0);
    CHECK_NEAR(band_area, 10500.0, 1.0);
    CHECK_NEAR(band_area + interior_area, 12000.0, 1.0);

    const Coverage coverage = sample_coverage(block, all);
    CHECK((coverage.inside) > size_t{1000});
    CHECK_EQ(coverage.overlaps, size_t{0});
    CHECK_EQ(coverage.gaps, size_t{0});
}

/**
 * @brief The band is a constant depth from the street, including at the corners
 *
 * The property that distinguishes a real inward offset from a mitred ring. Every
 * sample in a band piece is within the offset width of the block boundary and every
 * sample in the yard is beyond it -- which a mitred offset fails at a sharp corner,
 * where the mitre point shoots away from the outline and the band there is as deep
 * as the mitre is long.
 *
 * The test block is a triangle, because its 27-degree corner is where a mitre would
 * be worst.
 */
TEST(LotsOffset, the_band_is_a_constant_depth_from_the_street) {
    const Block block = make_block(Ring{{0.0, 0.0}, {120.0, 0.0}, {40.0, 80.0}});
    const double width = 12.0;

    std::vector<Ring> interior;
    const auto band = block_offset_band(block, width, &interior);

    size_t band_samples = 0;
    for (const auto& per_street : band) {
        for (const Ring& ring : per_street) {
            for (size_t i = 0; i < ring.size(); ++i) {
                const glm::dvec2 mid = 0.5 * (ring[i] + ring[(i + 1) % ring.size()]);
                if (!point_in_ring(block.ring, mid)) continue;
                CHECK((distance_to_ring(block.ring, mid)) <= width + 0.01);
                ++band_samples;
            }
        }
    }
    CHECK((band_samples) > size_t{4});

    // And the yard, from the other side: nothing in it is nearer the street than
    // the band is deep.
    size_t yard_samples = 0;
    for (const Ring& ring : interior) {
        const Block yard = make_block(ring);
        const Coverage unused = sample_coverage(yard, {ring}, 24);
        (void)unused;
        for (size_t i = 0; i < ring.size(); ++i) {
            CHECK((distance_to_ring(block.ring, ring[i])) >= width - 0.05);
            ++yard_samples;
        }
    }
    CHECK((yard_samples) > size_t{2});
}

// ============================================================================
// Offset Subdivision
// ============================================================================

/**
 * @brief The offset subdivision tiles the block and every band lot fronts a street
 *
 * The whole reason the mode exists: a 200 x 60 block halved by OBB puts parcels in
 * the middle with no street on any side, and `force_street_access` then refuses the
 * cut and leaves the middle as one enormous lot. Here the rim is parcels and only
 * the yard is landlocked.
 */
TEST(LotsOffset, offset_subdivision_lines_the_streets_and_leaves_a_yard) {
    const Block block = make_block(rect(200.0, 60.0));
    const StripSubdivision result = subdivide_block_offset(block, offset_params(25.0));

    check_tiles_block(block, result);
    CHECK((result.stats.street_pieces) >= size_t{4});
    CHECK_EQ(result.stats.interior_pieces, size_t{1});
    CHECK_NEAR(result.stats.street_area, 10500.0, 1.0);
    CHECK_NEAR(result.stats.interior_area, 1500.0, 1.0);

    // Every lot of the band fronts a street; the yard behind it does not.
    const std::vector<Lot> fronting = with_frontage(result.subdivision.lots, true);
    const std::vector<Lot> landlocked = with_frontage(result.subdivision.lots, false);
    CHECK_EQ(fronting.size(), result.stats.street_pieces);
    CHECK_EQ(landlocked.size(), size_t{1});
    CHECK_NEAR(landlocked.empty() ? 0.0 : landlocked.front().area, 1500.0, 1.0);
}

/**
 * @brief A band deeper than the block leaves no yard at all
 *
 * The degenerate end of the parameter, and it must not be a special case: every
 * point of the block is then within `offset_width` of a street, so every lot fronts
 * one and the yard is empty. An implementation that mitred the ring instead would
 * have the offset turn inside out here and produce a yard of NEGATIVE area.
 */
TEST(LotsOffset, a_band_deeper_than_the_block_leaves_no_yard) {
    const Block block = make_block(rect(100.0, 100.0));
    const StripSubdivision result = subdivide_block_offset(block, offset_params(1000.0));

    check_tiles_block(block, result);
    CHECK_EQ(result.stats.interior_pieces, size_t{0});
    CHECK_NEAR(result.stats.interior_area, 0.0, 1e-6);
    CHECK_NEAR(result.stats.street_area, 10000.0, 1.0);
    CHECK_EQ(with_frontage(result.subdivision.lots, false).size(), size_t{0});
}

/**
 * @brief A band of zero depth is the C2 recursion and nothing else
 *
 * `offset_width` of zero is a legitimate way to ask for OBB behaviour through this
 * entry point, so it must not be an error and must not produce an empty band lot
 * with no area. The whole block goes down the interior path.
 */
TEST(LotsOffset, a_band_of_zero_depth_is_all_yard) {
    const Block block = make_block(rect(100.0, 100.0));
    const StripSubdivision result = subdivide_block_offset(block, offset_params(0.0));

    check_tiles_block(block, result);
    CHECK_EQ(result.stats.street_pieces, size_t{0});
    CHECK_NEAR(result.stats.street_area, 0.0, 1e-9);
    CHECK_NEAR(result.stats.interior_area, 10000.0, 1.0);
}

/**
 * @brief The band follows a reflex corner round rather than crossing it
 *
 * An L-shaped block whose arms are 40 wide and a band 25 deep: there is no yard,
 * because nothing in a 40-wide arm is more than 20 from a street. The reflex corner
 * is where a mitred offset would fold, and the symptom would be band pieces
 * overlapping there -- which the coverage sample is what catches.
 */
TEST(LotsOffset, the_band_follows_a_reflex_corner) {
    const Block block = make_block(ell());
    const StripSubdivision result = subdivide_block_offset(block, offset_params(25.0));

    check_tiles_block(block, result);
    CHECK_NEAR(result.stats.interior_area, 0.0, 1.0);
    CHECK_EQ(with_frontage(result.subdivision.lots, false).size(), size_t{0});
}

/**
 * @brief A star-shaped block does not end up with its yard counted twice
 *
 * THE REGRESSION TEST FOR THE DEFECT THAT PUT 8822 SQUARE METRES OF LOTS OVER A
 * 5378 SQUARE METRE BLOCK.
 *
 * The band is the block less the yard, and working out which ring of a Clipper2
 * solution is the yard-shaped HOLE in that band used to be done by testing whether
 * the hole's area centroid fell inside the band's outer ring. On a star-shaped block
 * it does not: a deeply non-convex hole's centroid is not inside the hole, and here
 * it was not inside the block either. The hole was attached to nothing, silently
 * dropped, and the band came back as the entire block -- so every square metre of
 * the yard was in a band lot AND in the yard lot.
 *
 * These are the exact rings a search found, kept verbatim. A shape family that
 * usually reproduces a defect is not a regression test, and every one of these was
 * found by breaking the fix and searching for an input that noticed.
 *
 * The narrow offset is not incidental: at 5 metres the yard is most of the block, so
 * losing it doubles the total. A 25-metre band on the same shapes leaves almost no
 * yard and the defect would barely show.
 */
TEST(LotsOffset, a_star_shaped_block_does_not_double_cover_its_yard) {
    const std::vector<Ring> rings = {
        { { 81.937208073798658, 0.0 }, { 45.756183218422116, 45.756183218422109 }, { 1.0600368246163495e-15, 17.311715106010784 }, { -10.443864623782488, 10.44386462378249 }, { -24.835030563893973, 3.0414140686799441e-15 }, { -45.104768547413329, -45.104768547413315 }, { -1.5837025655488032e-14, -86.212752207490283 }, { 4.5637039155494108, -4.5637039155494135 } },
        { { 65.68672565222721, 0.0 }, { 45.351738040016251, 32.949966405030295 }, { 2.2610075057852121, 6.9586655779848279 }, { -21.74289453435636, 66.917748558926391 }, { -37.213854090384856, 27.037447627650863 }, { -42.634838367704482, 5.2212618339174056e-15 }, { -10.388951602655991, -7.5480151607190278 }, { -20.505098984902943, -63.108205573984762 }, { 3.8464848946830252, -11.838263236359243 }, { 7.0729683456467454, -5.1388123023480841 } },
        { { 66.116438930429865, 0.0 }, { 5.8918296918409201, 5.8918296918409192 }, { 5.1741690687097845e-15, 84.500593514999466 }, { -37.327008800092521, 37.327008800092528 }, { -15.382734220866542, 1.8838416225718664e-15 }, { -17.597855059266276, -17.597855059266273 }, { -1.290847985850001e-15, -7.0270491418355059 }, { 39.293288143131299, -39.293288143131321 } },
    };

    for (const Ring& ring : rings) {
        const Block block = make_block(ring);
        const StripSubdivision result = subdivide_block_offset(block, offset_params(5.0));

        CHECK_TRUE(result.stats.skeleton_complete);
        if (!result.stats.skeleton_complete) continue;

        const double expected = signed_ring_area(ring);
        CHECK_NEAR(total_area(result.subdivision.lots), expected, 1e-3 * expected);
        // Stated absolutely as well. A relative tolerance on a number that came back
        // at 164% of the block reads the same as one that was a little off, and the
        // whole point of these rings is which of the two it was.
        CHECK((total_area(result.subdivision.lots)) < 1.1 * expected);
        // The band and the yard are disjoint, so they add up rather than overlap.
        CHECK_NEAR(result.stats.street_area + result.stats.interior_area, expected,
                   1e-3 * expected);
        CHECK((result.stats.interior_area) > 0.0);

        const Coverage coverage = sample_coverage(block, lot_rings(result.subdivision.lots));
        CHECK((coverage.inside) > size_t{100});
        CHECK_EQ(coverage.overlaps, size_t{0});
        CHECK_EQ(coverage.gaps, size_t{0});
    }
}

// ============================================================================
// Skeleton Subdivision
// ============================================================================

/**
 * @brief Every skeleton lot reaches the street it fronts
 *
 * The promise of the mode. `force_street_access` can only achieve this by REFUSING
 * to cut, which leaves the middle of the block whole; here it is achieved by
 * construction, because every lot is cut out of the face of one street and a face
 * touches its own street by definition.
 */
TEST(LotsOffset, every_skeleton_lot_reaches_its_street) {
    const Block block = make_block(rect(200.0, 60.0));
    const StripSubdivision result = subdivide_block_skeleton(block, skeleton_params());

    check_tiles_block(block, result);
    CHECK_EQ(with_frontage(result.subdivision.lots, false).size(), size_t{0});
    CHECK_NEAR(result.stats.interior_area, 0.0, 1e-9);
    CHECK_EQ(result.stats.backland_pieces, size_t{0});
    for (const Lot& lot : result.subdivision.lots) {
        CHECK((lot.frontage_length) > 0.0);
    }
}

/**
 * @brief A skeleton lot's side boundaries are perpendicular to its street
 *
 * The other half of the promise, and the half that a lot which merely TOUCHES the
 * street would still satisfy. Checked on the middle of a run, where both sides are
 * cuts this file made: the cut runs along the street's normal, so its direction
 * dotted with the street direction is zero.
 *
 * The lots at the ends of a run are excluded, because there the side boundary is
 * the corner lot's edge, which is the next street's business.
 */
TEST(LotsOffset, skeleton_lot_sides_are_perpendicular_to_the_street) {
    const Block block = make_block(rect(240.0, 40.0));
    SkeletonLotParams params = skeleton_params();
    params.lot.lot_area_max = 600.0;
    const StripSubdivision result = subdivide_block_skeleton(block, params);

    check_tiles_block(block, result);

    // The bottom street runs along +x, so a cut across it runs along +/- y.
    size_t checked = 0;
    for (const Lot& lot : result.subdivision.lots) {
        if (lot.is_corner_lot) continue;
        // Only lots that front the bottom street, and only the ones away from its
        // two ends.
        bool on_bottom = false;
        for (size_t i = 0; i < lot.ring.size(); ++i) {
            if (lot.ring_edges[i] == 0) on_bottom = true;
        }
        if (!on_bottom) continue;
        const double min_x = std::min_element(lot.ring.begin(), lot.ring.end(),
                                              [](const glm::dvec2& a, const glm::dvec2& b) {
                                                  return a.x < b.x;
                                              })
                                 ->x;
        const double max_x = std::max_element(lot.ring.begin(), lot.ring.end(),
                                              [](const glm::dvec2& a, const glm::dvec2& b) {
                                                  return a.x < b.x;
                                              })
                                 ->x;
        if (min_x < 25.0 || max_x > 215.0) continue;

        for (size_t i = 0; i < lot.ring.size(); ++i) {
            if (lot.ring_edges[i] != kNoBlockEdge) continue;  // the kerb itself
            const glm::dvec2 seg = lot.ring[(i + 1) % lot.ring.size()] - lot.ring[i];
            if (glm::length(seg) < 0.5) continue;
            const glm::dvec2 dir = glm::normalize(seg);
            // Either along the street (the back of the lot, on the ridge) or
            // exactly across it. Nothing in between.
            const bool across = std::fabs(dir.x) < 1e-6;
            const bool along = std::fabs(dir.y) < 1e-6;
            CHECK_TRUE(across || along);
            ++checked;
        }
    }
    CHECK((checked) > size_t{4});
}

/**
 * @brief Corner alignment cuts a corner lot, and switching it off does not
 *
 * The positive and the negative in one test, because the positive alone passes
 * against an implementation that marks every lot a corner lot.
 *
 * A square has four convex corners, so four corner lots; at alignment 0 there are
 * none and the runs reach the corners along the skeleton's own bisectors. The block
 * is tiled either way -- the land does not go anywhere, it changes owner.
 */
TEST(LotsOffset, corner_alignment_cuts_corner_lots_and_zero_does_not) {
    const Block block = make_block(rect(100.0, 100.0));

    SkeletonLotParams squared = skeleton_params();
    squared.corner_alignment = 1.0;
    const StripSubdivision with = subdivide_block_skeleton(block, squared);
    check_tiles_block(block, with);
    CHECK((with.stats.corner_pieces) >= size_t{4});
    size_t marked = 0;
    for (const Lot& lot : with.subdivision.lots) {
        if (lot.is_corner_lot) ++marked;
    }
    CHECK_EQ(marked, with.subdivision.stats.corner_lots);
    CHECK((marked) >= size_t{4});

    SkeletonLotParams bisector = skeleton_params();
    bisector.corner_alignment = 0.0;
    const StripSubdivision without = subdivide_block_skeleton(block, bisector);
    check_tiles_block(block, without);
    CHECK_EQ(without.stats.corner_pieces, size_t{0});
    CHECK_EQ(without.subdivision.stats.corner_lots, size_t{0});
}

/**
 * @brief A corner lot takes the same depth from both of its streets
 *
 * `LotParams::corner_width` is clamped to half of EACH street at the corner, not
 * half of the one being cut. Clamp only the street in hand and a corner between a
 * short street and a long one takes the full width off the long one and half the
 * short one -- a lopsided parcel that fronts one street for five metres and the
 * other for twelve, and that eats most of the short street on its own.
 *
 * The block is a 200 x 100 rectangle with a ten-metre chamfer across its top-right
 * corner, which is what a real block has where three streets meet. `corner_width`
 * is 12, so every corner on the chamfer clamps to five: half the chamfer, not half
 * the long street beside it.
 */
TEST(LotsOffset, a_corner_lot_takes_the_same_depth_from_both_its_streets) {
    // The chamfer runs (200, 92) -> (194, 100): six across, eight up, ten long.
    const Block block = make_block(Ring{{0.0, 0.0},
                                        {200.0, 0.0},
                                        {200.0, 92.0},
                                        {194.0, 100.0},
                                        {0.0, 100.0}});
    SkeletonLotParams params = skeleton_params();
    params.corner_alignment = 1.0;
    params.lot.corner_width = 12.0;
    const StripSubdivision result = subdivide_block_skeleton(block, params);

    check_tiles_block(block, result);

    // The two corner lots on the chamfer are edge 2's neighbours: the one at
    // (200, 92) fronts edges 1 and 2, the one at (194, 100) fronts edges 2 and 3.
    size_t checked = 0;
    for (const Lot& lot : result.subdivision.lots) {
        if (!lot.is_corner_lot) continue;

        // Frontage per street this lot touches.
        std::vector<std::pair<uint32_t, double>> per_edge;
        for (size_t i = 0; i < lot.ring.size(); ++i) {
            const uint32_t tag = lot.ring_edges[i];
            if (tag == kNoBlockEdge) continue;
            const double length =
                glm::length(lot.ring[(i + 1) % lot.ring.size()] - lot.ring[i]);
            bool merged = false;
            for (auto& entry : per_edge) {
                if (entry.first == tag) {
                    entry.second += length;
                    merged = true;
                }
            }
            if (!merged) per_edge.emplace_back(tag, length);
        }
        bool on_chamfer = false;
        for (const auto& entry : per_edge) on_chamfer = on_chamfer || entry.first == 2;
        if (!on_chamfer || per_edge.size() != 2) continue;

        CHECK_NEAR(per_edge[0].second, per_edge[1].second, 0.2);
        CHECK_NEAR(per_edge[0].second, 5.0, 0.2);
        ++checked;
    }
    CHECK_EQ(checked, size_t{2});
}

/**
 * @brief A reflex corner gets no corner lot
 *
 * lots.cpp's rule, kept here: a reflex corner has no wedge to remove. An L has five
 * convex corners and one reflex one, so five corner lots and not six -- and an
 * implementation that took the cross product's sign the other way would produce
 * exactly one.
 */
TEST(LotsOffset, a_reflex_corner_gets_no_corner_lot) {
    const Block block = make_block(ell());
    SkeletonLotParams params = skeleton_params();
    params.corner_alignment = 1.0;
    const StripSubdivision result = subdivide_block_skeleton(block, params);

    check_tiles_block(block, result);

    // The reflex vertex of ell() is (40, 40). No corner lot may touch it.
    const glm::dvec2 reflex{40.0, 40.0};
    for (const Lot& lot : result.subdivision.lots) {
        if (!lot.is_corner_lot) continue;
        double nearest = 1e300;
        for (const glm::dvec2& p : lot.ring) nearest = std::min(nearest, glm::length(p - reflex));
        CHECK((nearest) > 1.0);
    }
    CHECK((result.subdivision.stats.corner_lots) >= size_t{4});
}

/**
 * @brief A shallow fraction puts land behind the lots, and 1 leaves none
 *
 * At 0.5 the street lots stop half way back and the rest is backland with no
 * frontage; at 1 they reach the ridge and there is no backland at all. The block is
 * tiled either way.
 *
 * A 100 x 100 square's faces are 50 deep, so half of each is 25 and the four street
 * runs hold 100*100 - 50*50 = 7500 square metres between them. That number is
 * asserted rather than just "some backland exists", because "some" is also what a
 * fraction applied to the wrong quantity produces.
 */
TEST(LotsOffset, a_shallow_fraction_leaves_backland_and_one_does_not) {
    const Block block = make_block(rect(100.0, 100.0));

    SkeletonLotParams shallow = skeleton_params();
    shallow.shallow_lot_frac = 0.5;
    shallow.corner_alignment = 0.0;
    const StripSubdivision cut = subdivide_block_skeleton(block, shallow);
    check_tiles_block(block, cut);
    CHECK((cut.stats.backland_pieces) >= size_t{1});
    CHECK_NEAR(cut.stats.street_area, 7500.0, 1.0);
    CHECK_NEAR(cut.stats.interior_area, 2500.0, 1.0);
    CHECK((with_frontage(cut.subdivision.lots, false).size()) > size_t{0});

    SkeletonLotParams full = skeleton_params();
    full.shallow_lot_frac = 1.0;
    full.corner_alignment = 0.0;
    const StripSubdivision deep = subdivide_block_skeleton(block, full);
    check_tiles_block(block, deep);
    CHECK_EQ(deep.stats.backland_pieces, size_t{0});
    CHECK_NEAR(deep.stats.street_area, 10000.0, 1.0);
    CHECK_EQ(with_frontage(deep.subdivision.lots, false).size(), size_t{0});
}

/**
 * @brief Simplify never moves a boundary two lots share, at any tolerance
 *
 * THE REGRESSION TEST FOR THE DEFECT THAT DELETED LAND INSIDE THE RANGE THE HEADER
 * RECOMMENDED.
 *
 * Simplification used to run on each lot ring INDEPENDENTLY, after the lots were
 * cut, and its comment claimed that was safe because the test never moves a vertex.
 * It does not move a vertex; it moves the BOUNDARY. Two lots share an edge and
 * approach a shared vertex from opposite sides, so the perpendicular test is
 * evaluated against different neighbours on the two sides and can answer
 * differently. One lot drops the vertex, the other keeps it, and the shared edge
 * has moved.
 *
 * The test that used to stand here could not see any of it, because its block had
 * perfectly straight streets: there the removed vertices are exactly collinear, the
 * simplification is a no-op on the geometry, and "every surviving vertex was there
 * before" stays true while the boundary walks away. Put a 1.5 metre sag in one
 * kerb -- a curve, which every real street has -- and at `simplify` 0.5 the lots
 * lost 0.38% of the block and three points of this suite's own sample grid landed
 * in no lot at all. At 4 metres it was 4.6% and 186 points. On the straight block
 * it was 136 points at 4 metres and nothing at all below that.
 *
 * So the assertion is now the thing the parameter must never do, swept over a range
 * that runs well past anything anyone would set: the lots tile the block, and their
 * total area is what it was with simplification switched off. Both blocks are
 * tested, because the straight one is the case the old test used.
 */
TEST(LotsOffset, simplify_never_moves_a_boundary_two_lots_share) {
    const Ring rings[] = { rect(200.0, 60.0), curved_kerb_block_ring() };
    const double tolerances[] = { 0.01, 0.1, 0.5, 1.0, 4.0 };

    for (const Ring& ring : rings) {
        const Block block = make_block(ring);

        SkeletonLotParams plain = skeleton_params();
        const StripSubdivision before = subdivide_block_skeleton(block, plain);
        check_tiles_block(block, before);
        const double baseline = total_area(before.subdivision.lots);
        CHECK_EQ(before.stats.simplified_vertices, size_t{0});

        for (const double tolerance : tolerances) {
            SkeletonLotParams simplified = skeleton_params();
            simplified.simplify = tolerance;
            const StripSubdivision after = subdivide_block_skeleton(block, simplified);

            // Tiles: no gap, no overlap, no lot outside the block. This is the
            // assertion the defect failed -- check_tiles_block's own 60 x 60 grid
            // found four uncovered points at a tolerance of half a metre.
            check_tiles_block(block, after);
            // And not one square metre of the block went anywhere. A millimetre of
            // slack on twelve thousand square metres, which is Clipper2's
            // quantisation and nothing else; the defect moved it by forty-five.
            CHECK_NEAR(total_area(after.subdivision.lots), baseline, 1e-3);
            // The statistics describe the lots that exist, so they cannot drift
            // away from them as the rings change.
            CHECK_NEAR(after.stats.street_area + after.stats.interior_area,
                       total_area(after.subdivision.lots), 1e-9);
        }
    }
}

/**
 * @brief Simplify does remove vertices, and both sides of the boundary lose the same ones
 *
 * The positive half, on a block that has something to remove. A rectangle has
 * nothing: a lot's kerb is a piece of ONE contour edge, so no lot ever inherits a
 * run of street polyline points, and the only thing the shared-boundary rule can
 * touch is a bisector arc between two adjacent faces that carries more than two
 * nodes. This block has one.
 *
 * `simplified_vertices` counts DISTINCT boundary vertices, and each of them is on
 * two lots, so the drop in the total vertex count is twice the count -- which is
 * the arithmetic that says both sides dropped the same vertices rather than one
 * side dropping some and the other dropping others.
 */
TEST(LotsOffset, simplify_removes_shared_vertices_from_both_lots_at_once) {
    const Block block = make_block(simplifiable_block_ring());

    SkeletonLotParams plain = skeleton_params();
    const StripSubdivision before = subdivide_block_skeleton(block, plain);
    check_tiles_block(block, before);

    SkeletonLotParams simplified = skeleton_params();
    simplified.simplify = 0.5;
    const StripSubdivision after = subdivide_block_skeleton(block, simplified);
    check_tiles_block(block, after);

    CHECK((after.stats.simplified_vertices) > size_t{0});
    CHECK_EQ(after.subdivision.lots.size(), before.subdivision.lots.size());
    CHECK((total_vertices(after.subdivision.lots)) <
          total_vertices(before.subdivision.lots));
    // Two lots per removed vertex, exactly. One fewer would mean a vertex went from
    // one side of a boundary and stayed on the other, which is the defect.
    CHECK_EQ(total_vertices(before.subdivision.lots) - total_vertices(after.subdivision.lots),
             2 * after.stats.simplified_vertices);
    // And the land did not move house. Exact, not near: the block outline is never
    // a candidate, so nothing leaves.
    CHECK_NEAR(total_area(after.subdivision.lots), total_area(before.subdivision.lots), 1e-9);

    // Nothing moved: every surviving vertex is one the unsimplified subdivision
    // produced. Kept from the test this replaces -- it is necessary and it was
    // never sufficient.
    for (const Lot& lot : after.subdivision.lots) {
        for (const glm::dvec2& p : lot.ring) {
            double nearest = 1e300;
            for (const Lot& original : before.subdivision.lots) {
                for (const glm::dvec2& q : original.ring) {
                    nearest = std::min(nearest, glm::length(p - q));
                }
            }
            CHECK((nearest) < 1e-9);
        }
    }
}

// ============================================================================
// Holes
// ============================================================================

/**
 * @brief A hole in the block is subtracted before the lots are cut
 *
 * `lots.cpp` subdivides the outer ring and then discards any lot that came out
 * wholly inside a hole, which leaves a lot that STRADDLES the hole boundary
 * covering ground that is not there. This file subtracts the hole first, so no lot
 * covers any of it: the areas sum to the block less the hole, and a sample point in
 * the hole is in no lot at all.
 */
TEST(LotsOffset, a_hole_is_subtracted_before_the_lots_are_cut) {
    Block block = make_block(rect(160.0, 160.0));
    // Clockwise, which is the orientation Block::holes are documented to carry.
    block.holes.push_back(Ring{{60.0, 60.0}, {60.0, 100.0}, {100.0, 100.0}, {100.0, 60.0}});
    block.hole_edges.emplace_back();
    CHECK((signed_ring_area(block.holes.front())) < 0.0);

    const StripSubdivision result = subdivide_block_offset(block, offset_params(20.0));
    check_tiles_block(block, result);
    CHECK_NEAR(total_area(result.subdivision.lots), 160.0 * 160.0 - 40.0 * 40.0, 30.0);

    // No lot covers the middle of the hole.
    const glm::dvec2 middle{80.0, 80.0};
    for (const Lot& lot : result.subdivision.lots) {
        CHECK_FALSE(point_in_ring(lot.ring, middle));
    }

    const StripSubdivision skeleton = subdivide_block_skeleton(block, skeleton_params());
    check_tiles_block(block, skeleton);
    for (const Lot& lot : skeleton.subdivision.lots) {
        CHECK_FALSE(point_in_ring(lot.ring, middle));
    }
}

/**
 * @brief A hole small enough to sit inside ONE lot's slab cuts that lot in two
 *
 * The path this reaches is `split_until_simple()` inside `cut_face_run()`, and
 * nothing else in this suite reaches it. A slab is one lot's share of one street's
 * run; when a courtyard falls entirely inside one, the Clipper2 intersection hands
 * back a single region WITH A HOLE IN IT, and a `Lot::ring` is one ring. Emitting
 * its outer ring instead would put a lot straight over the courtyard -- which is
 * the defect that once put 30400 square metres of lots over 24000 square metres of
 * land, seen here at the scale of one parcel rather than one block.
 *
 * So the bottom street's run is cut into slabs about twenty-seven metres wide and a
 * six-metre courtyard is placed in the middle of one. The lot that would have held
 * it comes out as TWO lots, one either side, and the count is asserted along with
 * the coverage: an implementation that covered the hole would have the right total
 * lot count and the wrong land under one of them.
 */
TEST(LotsOffset, a_hole_inside_one_slab_splits_that_lot_rather_than_covering_it) {
    Block block = make_block(rect(200.0, 60.0));
    // Clockwise, six metres square, at x = 100 -- inside one slab of the bottom run.
    block.holes.push_back(Ring{{97.0, 8.0}, {97.0, 14.0}, {103.0, 14.0}, {103.0, 8.0}});
    block.hole_edges.emplace_back();

    const Block plain = make_block(rect(200.0, 60.0));
    const StripSubdivision without = subdivide_block_skeleton(plain, skeleton_params());
    const StripSubdivision with = subdivide_block_skeleton(block, skeleton_params());

    check_tiles_block(block, with);
    CHECK_NEAR(total_area(with.subdivision.lots), 12000.0 - 36.0, 0.05);
    // Exactly one more lot than the same block without the courtyard: the slab that
    // held it was cut in two, and no other slab changed.
    CHECK_EQ(with.subdivision.lots.size(), without.subdivision.lots.size() + 1);

    // Nothing covers the courtyard, and nothing covers its corners either -- a lot
    // whose ring wrapped the hole would still miss a centre test if the hole were
    // non-convex, so the corners are checked too.
    const glm::dvec2 probes[] = {
        {100.0, 11.0}, {97.5, 8.5}, {102.5, 8.5}, {97.5, 13.5}, {102.5, 13.5},
    };
    for (const glm::dvec2& p : probes) {
        for (const Lot& lot : with.subdivision.lots) {
            CHECK_FALSE(point_in_ring(lot.ring, p));
        }
    }
}

/**
 * @brief A block wound the wrong way is refused, in either ring
 *
 * Two refusals, and neither used to happen.
 *
 * **The outer ring.** `compute_straight_skeleton()` REVERSES a clockwise ring and
 * returns the reversed one, which has the same size -- so the size check that was
 * the only guard here never fired. Face `e` was then matched against
 * `block.ring[e]` of the un-reversed ring, the inward normal pointed outward, and
 * `corner_widths()`'s convexity test inverted. Entirely silently:
 * `skeleton_complete` stayed true, the lots still tiled, and `tag_frontage()` still
 * got the frontage right because it matches by geometry. What changed was the
 * answer -- with `shallow_lot_frac` 0.5 the anticlockwise ring gives 5100 square
 * metres of backland and the reversed one gives none at all, because the shallow
 * clip was applied to the far side of the block.
 *
 * **A hole.** `Block::holes` are clockwise, and Clipper2's NonZero fill is why it
 * matters: a hole wound like the outer ring has a winding number of two inside it,
 * so the fill KEEPS it. The courtyard came back as land -- 25200 square metres of
 * lots over a 24000 square metre block, with a lot sitting on the courtyard.
 *
 * Asserted against the anticlockwise answer rather than as a bare refusal, so the
 * test also says what was being got wrong.
 */
TEST(LotsOffset, a_block_wound_the_wrong_way_is_refused) {
    SkeletonLotParams params = skeleton_params();
    params.shallow_lot_frac = 0.5;
    params.corner_alignment = 0.0;

    Ring ring = rect(200.0, 60.0);
    const Block anticlockwise = make_block(ring);
    const StripSubdivision good = subdivide_block_skeleton(anticlockwise, params);
    check_tiles_block(anticlockwise, good);
    CHECK((good.stats.backland_pieces) >= size_t{1});
    CHECK_NEAR(good.stats.interior_area, 5100.0, 1.0);

    std::reverse(ring.begin(), ring.end());
    const Block clockwise = make_block(ring);
    CHECK((signed_ring_area(clockwise.ring)) < 0.0);
    const StripSubdivision refused = subdivide_block_skeleton(clockwise, params);
    CHECK_FALSE(refused.stats.skeleton_complete);
    CHECK_TRUE(refused.subdivision.lots.empty());
    CHECK_NEAR(refused.stats.interior_area, 0.0, 1e-12);

    // The offset mode and the band helper go through the same setup and refuse too.
    CHECK_FALSE(subdivide_block_offset(clockwise, offset_params()).stats.skeleton_complete);
    CHECK_TRUE(block_offset_band(clockwise, 10.0, nullptr).empty());

    // And a hole with the outer ring's winding.
    Block wrong_hole = make_block(rect(160.0, 160.0));
    Ring hole{{60.0, 60.0}, {60.0, 100.0}, {100.0, 100.0}, {100.0, 60.0}};
    CHECK((signed_ring_area(hole)) < 0.0);
    std::reverse(hole.begin(), hole.end());
    wrong_hole.holes.push_back(hole);
    wrong_hole.hole_edges.emplace_back();
    CHECK_FALSE(subdivide_block_offset(wrong_hole, offset_params(20.0)).stats.skeleton_complete);
    CHECK_FALSE(subdivide_block_skeleton(wrong_hole, skeleton_params()).stats.skeleton_complete);
    CHECK_TRUE(block_offset_band(wrong_hole, 20.0, nullptr).empty());

    // The same block with the hole wound the documented way is subdivided, so the
    // refusal above is about the winding and not about the hole.
    Block right_hole = make_block(rect(160.0, 160.0));
    std::reverse(hole.begin(), hole.end());
    right_hole.holes.push_back(hole);
    right_hole.hole_edges.emplace_back();
    const StripSubdivision ok = subdivide_block_offset(right_hole, offset_params(20.0));
    CHECK_TRUE(ok.stats.skeleton_complete);
    check_tiles_block(right_hole, ok);
}

/**
 * @brief lot_area_min caps the run when it is close to lot_area_max
 *
 * The third constraint in `lots_along_run()`, and the only one with no test. It
 * reduces the count when a run would otherwise produce lots below the SPLITTING
 * floor, and with the suite's usual 200 and 800 it can never fire: the two are far
 * enough apart that the area target always leaves room. `count = by_area;` could be
 * deleted and every other test stayed green.
 *
 * It needs the two to be close. With a maximum of 100 and a minimum of 90, a run of
 * 150 square metres asks for two lots by area and can only have one by the floor --
 * `(count - 1) * lot_area_max < area < count * lot_area_min` is the band, and it is
 * empty unless the two are within a factor of `count / (count - 1)`.
 *
 * The block is a 15 x 10 rectangle, whose bottom face is a trapezium of about 75
 * square metres, and `lot_width_min` is dropped out of the way so that the area
 * floor is unambiguously the constraint that fired.
 */
TEST(LotsOffset, lot_area_min_caps_a_run_whose_lots_would_be_below_the_floor) {
    const Block block = make_block(rect(15.0, 10.0));

    SkeletonLotParams tight = skeleton_params();
    tight.corner_alignment = 0.0;
    tight.lot.lot_width_min = 0.5;
    tight.lot.lot_area_max = 30.0;
    tight.lot.lot_area_min = 28.0;
    const StripSubdivision capped = subdivide_block_skeleton(block, tight);
    check_tiles_block(block, capped);
    CHECK((capped.subdivision.stats.rejected_area) > size_t{0});

    // The same block with the floor out of the way cuts more lots, which is what
    // says the cap above changed the answer rather than merely being counted.
    SkeletonLotParams loose = tight;
    loose.lot.lot_area_min = 1.0;
    const StripSubdivision uncapped = subdivide_block_skeleton(block, loose);
    check_tiles_block(block, uncapped);
    CHECK_EQ(uncapped.subdivision.stats.rejected_area, size_t{0});
    CHECK((uncapped.subdivision.lots.size()) > capped.subdivision.lots.size());

    // And the floor is a floor on SPLITTING, not on lots: the capped run still
    // covers the whole block.
    CHECK_NEAR(total_area(capped.subdivision.lots), 150.0, 0.05);
}

// ============================================================================
// Identity and Determinism
// ============================================================================

/**
 * @brief The same block gives the same lots, to the bit
 *
 * The C7 determinism commitment. Compared exactly rather than with a tolerance,
 * because "the same to within an epsilon" is precisely what a container-order
 * dependence or a shared random stream would produce.
 */
TEST(LotsOffset, the_same_block_gives_the_same_lots) {
    const Block block = make_block(ell());

    for (int mode = 0; mode < 2; ++mode) {
        const StripSubdivision a = (mode == 0)
                                       ? subdivide_block_offset(block, offset_params())
                                       : subdivide_block_skeleton(block, skeleton_params());
        const StripSubdivision b = (mode == 0)
                                       ? subdivide_block_offset(block, offset_params())
                                       : subdivide_block_skeleton(block, skeleton_params());

        CHECK_EQ(a.subdivision.lots.size(), b.subdivision.lots.size());
        if (a.subdivision.lots.size() != b.subdivision.lots.size()) continue;
        for (size_t i = 0; i < a.subdivision.lots.size(); ++i) {
            CHECK_EQ(a.subdivision.lots[i].id, b.subdivision.lots[i].id);
            CHECK_EQ(a.subdivision.lots[i].node_key, b.subdivision.lots[i].node_key);
            CHECK_EQ(a.subdivision.lots[i].ring.size(), b.subdivision.lots[i].ring.size());
            if (a.subdivision.lots[i].ring.size() != b.subdivision.lots[i].ring.size()) continue;
            for (size_t j = 0; j < a.subdivision.lots[i].ring.size(); ++j) {
                CHECK_EQ(a.subdivision.lots[i].ring[j].x, b.subdivision.lots[i].ring[j].x);
                CHECK_EQ(a.subdivision.lots[i].ring[j].y, b.subdivision.lots[i].ring[j].y);
            }
        }
    }
}

/**
 * @brief Every lot ring starts at its own lexicographically smallest vertex
 *
 * The POST-CONDITION the canonicalisation exists to establish, asserted directly.
 *
 * `canonicalise()` and `rotate_to_canonical_start()` are there because Clipper2 is
 * deterministic for identical input but WHERE in a ring it starts is an
 * implementation detail a version bump is free to change. Calling the subdivision
 * twice in one process against one Clipper2 build -- which is what the test above
 * does -- cannot possibly see that: it is exactly the situation in which Clipper2
 * is already deterministic. Both rotations could be deleted and that test would
 * stay green.
 *
 * This one fails the moment either goes, because it does not compare two runs at
 * all. It states what a canonical ring looks like.
 *
 * `shallow_lot_frac` stays at 1 so that every lot comes off a face rather than out
 * of the C2 recursion: `subdivide_ring()` builds its rings itself and does not
 * promise this, which is lots.cpp's business rather than this file's.
 */
TEST(LotsOffset, every_lot_ring_starts_at_its_smallest_vertex) {
    const Ring rings[] = { rect(200.0, 60.0), ell(), simplifiable_block_ring() };

    size_t checked = 0;
    for (const Ring& ring : rings) {
        const Block block = make_block(ring);
        SkeletonLotParams params = skeleton_params();
        params.shallow_lot_frac = 1.0;
        const StripSubdivision result = subdivide_block_skeleton(block, params);
        check_tiles_block(block, result);
        CHECK_EQ(result.stats.backland_pieces, size_t{0});

        for (const Lot& lot : result.subdivision.lots) {
            if (lot.ring.size() < 2) continue;
            ++checked;
            const glm::dvec2& first = lot.ring.front();
            for (const glm::dvec2& p : lot.ring) {
                CHECK((first.x < p.x) || (first.x == p.x && first.y <= p.y));
            }
        }
    }
    CHECK((checked) > size_t{20});
}

/**
 * @brief Listing a block's holes in the other order gives the same lots, to the bit
 *
 * The other half of the canonicalisation, and the half the two-runs-agree test
 * cannot reach either. `boolean_regions()` sorts its regions, and
 * `canonicalise(Paths64&)` sorts the inward offset's paths, because the ORDER
 * Clipper2 returns a solution in is a function of the order it was fed -- and the
 * index of a region inside one cut goes straight into that lot's node key through
 * `region_key()`. Without the sorts, feeding the same two holes the other way round
 * renumbers lots that did not move.
 *
 * Permuting the holes is a real edit -- `extract_blocks()` has no reason to order a
 * block's courtyards one way rather than another -- and it is the smallest change
 * to the INPUT ORDER that must leave the OUTPUT untouched. Compared bit for bit,
 * ids and coordinates both.
 */
TEST(LotsOffset, reordering_a_blocks_holes_does_not_change_a_single_lot) {
    const Ring first_hole{ { 40.0, 40.0 }, { 40.0, 70.0 }, { 70.0, 70.0 }, { 70.0, 40.0 } };
    const Ring second_hole{ { 100.0, 95.0 }, { 100.0, 125.0 }, { 130.0, 125.0 }, { 130.0, 95.0 } };
    CHECK((signed_ring_area(first_hole)) < 0.0);
    CHECK((signed_ring_area(second_hole)) < 0.0);

    Block forward = make_block(rect(180.0, 180.0));
    forward.holes = { first_hole, second_hole };
    forward.hole_edges.resize(2);

    Block backward = make_block(rect(180.0, 180.0));
    backward.holes = { second_hole, first_hole };
    backward.hole_edges.resize(2);

    for (int mode = 0; mode < 2; ++mode) {
        const StripSubdivision a =
            (mode == 0) ? subdivide_block_offset(forward, offset_params(20.0))
                        : subdivide_block_skeleton(forward, skeleton_params());
        const StripSubdivision b =
            (mode == 0) ? subdivide_block_offset(backward, offset_params(20.0))
                        : subdivide_block_skeleton(backward, skeleton_params());

        check_tiles_block(forward, a);
        check_tiles_block(backward, b);

        CHECK_EQ(a.subdivision.lots.size(), b.subdivision.lots.size());
        if (a.subdivision.lots.size() != b.subdivision.lots.size()) continue;
        for (size_t i = 0; i < a.subdivision.lots.size(); ++i) {
            const Lot& left = a.subdivision.lots[i];
            const Lot& right = b.subdivision.lots[i];
            CHECK_EQ(left.id, right.id);
            CHECK_EQ(left.node_key, right.node_key);
            CHECK_EQ(left.ring.size(), right.ring.size());
            if (left.ring.size() != right.ring.size()) continue;
            for (size_t j = 0; j < left.ring.size(); ++j) {
                CHECK_EQ(left.ring[j].x, right.ring[j].x);
                CHECK_EQ(left.ring[j].y, right.ring[j].y);
            }
        }
    }
}

/**
 * @brief Irregularity moves the cuts, and zero irregularity ignores the seed
 *
 * The negative half of the determinism test. A subdivision that ignored its seed
 * entirely would pass "two runs agree" and fail here; one that consulted a shared
 * random stream would pass here and fail the other half of this test, because at
 * irregularity zero nothing may consult a draw at all.
 */
TEST(LotsOffset, irregularity_moves_the_cuts_and_zero_ignores_the_seed) {
    const Block block = make_block(rect(240.0, 40.0));

    SkeletonLotParams tidy = skeleton_params();
    tidy.lot.irregularity = 0.0;
    tidy.lot.seed = 1;
    SkeletonLotParams tidy_other_seed = tidy;
    tidy_other_seed.lot.seed = 999;

    const StripSubdivision a = subdivide_block_skeleton(block, tidy);
    const StripSubdivision b = subdivide_block_skeleton(block, tidy_other_seed);
    CHECK_EQ(a.subdivision.lots.size(), b.subdivision.lots.size());
    if (a.subdivision.lots.size() == b.subdivision.lots.size()) {
        for (size_t i = 0; i < a.subdivision.lots.size(); ++i) {
            // The GEOMETRY is seed-independent at irregularity zero. The ids are not
            // -- the seed is mixed into the block root key -- and that is the whole
            // point of separating Lot::id from Lot::ring.
            CHECK_EQ(a.subdivision.lots[i].ring.size(), b.subdivision.lots[i].ring.size());
            CHECK_NEAR(a.subdivision.lots[i].area, b.subdivision.lots[i].area, 1e-9);
        }
    }
    CHECK((a.subdivision.lots.empty() ? 0u : 1u) == 1u);
    if (!a.subdivision.lots.empty()) {
        CHECK((a.subdivision.lots.front().id) != b.subdivision.lots.front().id);
    }

    SkeletonLotParams wobbly = skeleton_params();
    wobbly.lot.irregularity = 1.0;
    const StripSubdivision c = subdivide_block_skeleton(block, wobbly);
    check_tiles_block(block, c);

    bool any_moved = false;
    const size_t common = std::min(a.subdivision.lots.size(), c.subdivision.lots.size());
    for (size_t i = 0; i < common; ++i) {
        if (std::fabs(a.subdivision.lots[i].area - c.subdivision.lots[i].area) > 1e-6) {
            any_moved = true;
        }
    }
    CHECK_TRUE(any_moved);
}

/**
 * @brief An id survives a street being nudged, AND a neighbouring street gaining a lot
 *
 * Mechanism 2 of the three in the C7 notes, and the one that matters most because
 * it is the one that happens on every drag of a node. A lot's key is its ADDRESS --
 * which street it fronts and its index along that street -- so an edit somewhere
 * else on the block leaves it alone with no matching step at all.
 *
 * ### Why the nudge alone is not the test
 *
 * Sliding the far kerb a metre changes no face's lot COUNT, and when no count
 * changes an emission-order key and an address key agree on everything: the lots
 * come out in the same order, so numbering them 0, 1, 2... by order gives the same
 * answer as numbering them by index along their own face. The test that used to
 * stand here asserted only that the whole block's id SET was unchanged, which is
 * true of both designs, so it could not tell them apart -- deriving the key from
 * `out_street.size()` instead of from the index along the face passed it untouched.
 *
 * The case that separates them is a change that makes ONE street gain a lot. With
 * address keys the other streets are untouched; with emission-order keys every lot
 * emitted after the new one shifts by one and its id changes. So the second half
 * grows the bottom street by 20 metres, which takes its run from eleven lots to
 * twelve, and asserts that the two SHORT streets -- which did not change length and
 * did not change count -- keep every id they had.
 *
 * Both modes are here. The offset mode had no identity test at all, and it mixes
 * the same face index into its keys for the same reason.
 */
TEST(LotsOffset, ids_survive_a_street_being_nudged_and_a_neighbour_gaining_a_lot) {
    // --- Half one: a nudge that changes no count leaves every id in the block ----
    const Block before = make_block(rect(200.0, 60.0));
    Ring moved = rect(200.0, 60.0);
    moved[2].y += 1.0;  // the far kerb slides a metre
    moved[3].y += 1.0;
    const Block after = make_block(moved);

    for (int mode = 0; mode < 2; ++mode) {
        const StripSubdivision a = (mode == 0)
                                       ? subdivide_block_skeleton(before, skeleton_params())
                                       : subdivide_block_offset(before, offset_params());
        const StripSubdivision b = (mode == 0)
                                       ? subdivide_block_skeleton(after, skeleton_params())
                                       : subdivide_block_offset(after, offset_params());
        check_tiles_block(before, a);
        check_tiles_block(after, b);

        std::set<LotId> ids_a;
        for (const Lot& lot : a.subdivision.lots) ids_a.insert(lot.id);
        std::set<LotId> ids_b;
        for (const Lot& lot : b.subdivision.lots) ids_b.insert(lot.id);
        CHECK_EQ(ids_a.size(), ids_b.size());
        CHECK_TRUE(ids_a == ids_b);
    }

    // --- Half two: one street gains a lot; the others keep theirs ----------------
    //
    // 200 x 120 rather than 200 x 60 so the two short streets carry five lots each
    // instead of one. "Five of five survived" is evidence; "one of one survived" is
    // a coin toss.
    const Block narrow = make_block(rect(200.0, 120.0));
    const Block wide = make_block(rect(220.0, 120.0));

    for (int mode = 0; mode < 2; ++mode) {
        const StripSubdivision a = (mode == 0)
                                       ? subdivide_block_skeleton(narrow, skeleton_params())
                                       : subdivide_block_offset(narrow, offset_params());
        const StripSubdivision b = (mode == 0)
                                       ? subdivide_block_skeleton(wide, skeleton_params())
                                       : subdivide_block_offset(wide, offset_params());
        check_tiles_block(narrow, a);
        check_tiles_block(wide, b);

        const auto grouped_a = ids_by_frontage(a.subdivision.lots);
        const auto grouped_b = ids_by_frontage(b.subdivision.lots);

        // The bottom street -- the one that grew -- must actually have gained a
        // lot, or the second half of this test is the first half again.
        const std::set<uint32_t> bottom{0};
        const size_t bottom_before = grouped_a.count(bottom) ? grouped_a.at(bottom).size() : 0;
        const size_t bottom_after = grouped_b.count(bottom) ? grouped_b.at(bottom).size() : 0;
        CHECK((bottom_after) > bottom_before);

        // And the two streets that did not change keep every id they had. THIS is
        // the assertion an emission-order key fails: growing street 0 renumbers
        // everything emitted after it.
        for (const uint32_t edge : { 1u, 3u }) {
            const std::set<uint32_t> key{edge};
            CHECK((grouped_a.count(key)) == size_t{1});
            CHECK((grouped_b.count(key)) == size_t{1});
            if (grouped_a.count(key) == 0 || grouped_b.count(key) == 0) continue;
            const std::set<LotId>& was = grouped_a.at(key);
            const std::set<LotId>& now = grouped_b.at(key);
            CHECK((was.size()) >= size_t{2});
            CHECK_EQ(was.size(), now.size());
            CHECK_EQ(retained(was, now), was.size());
        }
    }
}

/**
 * @brief Two blocks, and the two C2 modes, never share a lot id
 *
 * `LotParams::block_key` roots the identity of every lot in a block, so two blocks
 * subdivided the same way must still come out with different ids -- otherwise an
 * attribute set on one lot appears on another block's lot. And an id from this file
 * must not collide with one from `subdivide_block()`, which it cannot, because the
 * salts are disjoint; a test says so rather than the comment alone.
 */
TEST(LotsOffset, ids_do_not_collide_between_blocks_or_with_the_c2_recursion) {
    const Block block = make_block(rect(200.0, 60.0));

    SkeletonLotParams first = skeleton_params();
    SkeletonLotParams second = skeleton_params();
    second.lot.block_key = 0xFEDCBA9876543210ull;

    const StripSubdivision a = subdivide_block_skeleton(block, first);
    const StripSubdivision b = subdivide_block_skeleton(block, second);
    std::set<LotId> ids;
    for (const Lot& lot : a.subdivision.lots) ids.insert(lot.id);
    size_t shared = 0;
    for (const Lot& lot : b.subdivision.lots) {
        if (ids.count(lot.id) != 0) ++shared;
    }
    CHECK_EQ(shared, size_t{0});

    const auto c2 = subdivide_block(block, base_lot_params());
    CHECK((c2.lots.size()) > size_t{0});
    size_t crossed = 0;
    for (const Lot& lot : c2.lots) {
        if (ids.count(lot.id) != 0) ++crossed;
    }
    CHECK_EQ(crossed, size_t{0});

    // And the offset mode does not collide with the skeleton mode on the same block
    // either, because a lot of one is not a lot of the other.
    const StripSubdivision offset = subdivide_block_offset(block, offset_params());
    size_t across_modes = 0;
    for (const Lot& lot : offset.subdivision.lots) {
        if (ids.count(lot.id) != 0) ++across_modes;
    }
    CHECK_EQ(across_modes, size_t{0});
}

/**
 * @brief Lot::ring_edges names the street in front, not a street nearby
 *
 * What C4 reads. Each ring segment of `make_block()` carries its own index as a
 * block edge, so a lot cut out of the face of edge 2 must tag its kerb segment 2
 * and nothing else. Without this a frontage would have to be matched back to a
 * street by proximity, which is the same class of mistake as finding junctions by
 * endpoint proximity and fails in the same place.
 */
TEST(LotsOffset, ring_edges_name_the_street_the_lot_fronts) {
    const Block block = make_block(rect(200.0, 60.0));
    const StripSubdivision result = subdivide_block_skeleton(block, skeleton_params());
    check_tiles_block(block, result);

    size_t checked = 0;
    for (const Lot& lot : result.subdivision.lots) {
        for (size_t i = 0; i < lot.ring.size(); ++i) {
            if (lot.ring_edges[i] == kNoBlockEdge) continue;
            ++checked;
            const uint32_t edge = lot.ring_edges[i];
            CHECK((edge) < block.ring.size());
            if (edge >= block.ring.size()) continue;

            // The tagged segment really does lie on that block segment.
            const glm::dvec2 a = block.ring[edge];
            const glm::dvec2 b = block.ring[(edge + 1) % block.ring.size()];
            const glm::dvec2 p = lot.ring[i];
            const glm::dvec2 q = lot.ring[(i + 1) % lot.ring.size()];
            const glm::dvec2 dir = glm::normalize(b - a);
            CHECK((std::fabs(cross2(dir, p - a))) < 1e-3);
            CHECK((std::fabs(cross2(dir, q - a))) < 1e-3);
        }
    }
    CHECK((checked) >= size_t{4});
}

// ============================================================================
// Constraints and Refusals
// ============================================================================

/**
 * @brief lot_width_min stops a long run being shaved into ribbons
 *
 * The area constraint alone would cut a 240 x 12 block into 90 lots of 32 square
 * metres, each 2.6 metres wide. The width floor caps the count instead, which means
 * a lot LARGER than lot_area_max is a normal outcome and the stats say which
 * constraint fired -- exactly as lots.hpp documents for the OBB recursion.
 */
TEST(LotsOffset, lot_width_min_caps_the_run) {
    const Block block = make_block(rect(240.0, 12.0));
    SkeletonLotParams params = skeleton_params();
    params.corner_alignment = 0.0;
    params.lot.lot_area_max = 40.0;
    params.lot.lot_area_min = 1.0;
    params.lot.lot_width_min = 20.0;
    const StripSubdivision result = subdivide_block_skeleton(block, params);

    check_tiles_block(block, result);
    CHECK((result.subdivision.stats.rejected_width) > size_t{0});
    // 240 / 20 is twelve lots on the long sides; nothing like the seventy-two the
    // area constraint on its own would ask for.
    CHECK((result.subdivision.lots.size()) <= size_t{30});

    // Stated as a WIDTH, which is what the constraint is. Asserting a minimum AREA
    // instead would be wrong at the two ends of the block, where the whole 12-metre
    // kerb is one lot of 36 square metres -- correctly, because the floor is a floor
    // on splitting and that street cannot be split at all.
    size_t checked = 0;
    for (const Lot& lot : result.subdivision.lots) {
        uint32_t edge = kNoBlockEdge;
        for (const uint32_t tag : lot.ring_edges) {
            if (tag != kNoBlockEdge) edge = tag;
        }
        CHECK((edge) != kNoBlockEdge);
        if (edge == kNoBlockEdge) continue;
        const glm::dvec2 a = block.ring[edge];
        const glm::dvec2 b = block.ring[(edge + 1) % block.ring.size()];
        CHECK((lot.frontage_length) >= std::min(20.0, glm::length(b - a)) - 0.01);
        ++checked;
    }
    CHECK((checked) > size_t{4});
}

/**
 * @brief A block the skeleton refuses yields no lots, and says so
 *
 * The contract, and the reason there is no silent fallback: an author who chose
 * skeleton subdivision and got OBB parcels has no way to tell. A ring of two points
 * has no skeleton, so `skeleton_complete` is false and `lots` is empty.
 */
TEST(LotsOffset, a_block_with_no_skeleton_yields_no_lots) {
    const Block degenerate = make_block(Ring{{0.0, 0.0}, {10.0, 0.0}});
    const StripSubdivision offset = subdivide_block_offset(degenerate, offset_params());
    CHECK_FALSE(offset.stats.skeleton_complete);
    CHECK_TRUE(offset.subdivision.lots.empty());

    const StripSubdivision skeleton = subdivide_block_skeleton(degenerate, skeleton_params());
    CHECK_FALSE(skeleton.stats.skeleton_complete);
    CHECK_TRUE(skeleton.subdivision.lots.empty());

    // A ring with no area is refused the same way, and the band helper agrees.
    const Block flat = make_block(Ring{{0.0, 0.0}, {10.0, 0.0}, {20.0, 0.0}});
    CHECK_FALSE(subdivide_block_skeleton(flat, skeleton_params()).stats.skeleton_complete);
    CHECK_TRUE(block_offset_band(flat, 5.0, nullptr).empty());
}

/**
 * @brief A block smaller than one lot comes out as one lot, never as none
 *
 * lots.hpp's rule, kept here: `lot_area_min` is a floor on SPLITTING, not on lots.
 * The parcel exists whether or not it is big enough to build on, and a caller that
 * wants to skip it can read Lot::area itself. Discarding it would lose land.
 */
TEST(LotsOffset, a_block_smaller_than_one_lot_is_still_one_lot) {
    const Block tiny = make_block(rect(9.0, 9.0));
    SkeletonLotParams params = skeleton_params();
    params.corner_alignment = 0.0;
    const StripSubdivision result = subdivide_block_skeleton(tiny, params);

    check_tiles_block(tiny, result);
    CHECK((result.subdivision.lots.size()) >= size_t{1});
    CHECK_NEAR(total_area(result.subdivision.lots), 81.0, 0.2);
}

/**
 * @brief A triangular block is subdivided, which is where the OBB recursion is worst
 *
 * The case the mode exists for. A triangle's oriented bounding box shares almost
 * none of its boundary, so the OBB halving cuts across the streets rather than
 * along them. Here every lot fronts the street it was cut from, including at the
 * 27-degree corner.
 */
TEST(LotsOffset, a_triangular_block_is_subdivided_along_its_streets) {
    const Block block = make_block(Ring{{0.0, 0.0}, {120.0, 0.0}, {40.0, 80.0}});
    SkeletonLotParams params = skeleton_params();
    params.corner_alignment = 1.0;
    const StripSubdivision result = subdivide_block_skeleton(block, params);

    check_tiles_block(block, result);
    CHECK_EQ(with_frontage(result.subdivision.lots, false).size(), size_t{0});
    CHECK((result.stats.street_pieces) >= size_t{3});
    CHECK((result.subdivision.stats.corner_lots) >= size_t{1});
}
