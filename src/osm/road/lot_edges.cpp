// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file lot_edges.cpp
 * @brief Implementation of front / side / back / interior classification
 *
 * Four passes over one lot: resolve each edge's provenance, group the street
 * edges into frontages, pick the primary, then classify what is left by angle
 * against the primary's normal.
 *
 * There is no spatial index, no nearest-road query and no distance-to-centreline
 * anywhere in this file, and that absence is the design. The only geometry that
 * decides a FRONT is the provenance index the subdivider handed over; the angles
 * below only ever separate BACK from SIDE from INTERIOR, among edges already known
 * to touch no street at all. See the "What facing a street means" section of
 * lot_edges.hpp.
 */

#include "osm/road/lot_edges.hpp"

#include <spdlog/spdlog.h>

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
 * @brief Below this length in metres a ring segment has no direction
 *
 * A micrometre, the same value blocks.cpp treats as "the same point", so a segment
 * this file calls directionless is exactly one the block traversal would have
 * pruned. Disagreeing with it would mean deriving a normal from two points
 * RoadGraph considers identical, and the resulting direction is pure rounding.
 */
constexpr double kLengthEpsilon = 1e-6;

/// A ring enclosing less than this in square metres has no interior, and so no
/// inside to take normals away from. Matches blocks.cpp's own area floor.
constexpr double kAreaEpsilon = 1e-9;

/// No such index
constexpr size_t kNoIndex = std::numeric_limits<size_t>::max();

// ============================================================================
// Geometry Helpers
// ============================================================================

/// Squared length of a 2D vector
inline double length_sq(const glm::dvec2& v) noexcept {
    return v.x * v.x + v.y * v.y;
}

/// Length of a 2D vector
inline double length_of(const glm::dvec2& v) noexcept {
    return std::sqrt(length_sq(v));
}

/// Unit vector, or (0, 0) when @p v is shorter than kLengthEpsilon
inline glm::dvec2 safe_normalize(const glm::dvec2& v) noexcept {
    const double len = length_of(v);
    if (len < kLengthEpsilon) {
        return glm::dvec2{0.0};
    }
    return v / len;
}

/**
 * @brief Cosine of a cone half-angle given in degrees
 *
 * Clamped to [0, 180] first. A negative half-angle would make the cone reject
 * everything and a half-angle past 180 would make it accept everything, and both
 * are configuration mistakes that should behave predictably rather than produce a
 * cosine outside [-1, 1] that then compares in surprising ways.
 */
inline double cone_cosine(double degrees) noexcept {
    const double clamped = std::clamp(degrees, 0.0, 180.0);
    return std::cos(clamped * 3.14159265358979323846 / 180.0);
}

// ============================================================================
// Per-Edge Geometry
// ============================================================================

/**
 * @brief The direction facts about one ring segment, computed once
 *
 * The outward normal is (dy, -dx) for an ANTICLOCKWISE ring: walking
 * anticlockwise keeps the interior on the left, so the right-hand perpendicular
 * points out. The `orientation` argument of edge_geometry() below carries the sign
 * that makes that true for a ring which arrived the other way round.
 */
struct EdgeGeometry {
    glm::dvec2 normal{0.0};     ///< Outward unit normal, or (0, 0) for a null segment
    double length = 0.0;        ///< Segment length in metres
};

/// Outward normal and length of every segment of @p ring
std::vector<EdgeGeometry> edge_geometry(const std::vector<glm::dvec2>& ring, double orientation) {
    const size_t count = ring.size();
    std::vector<EdgeGeometry> geometry(count);

    for (size_t i = 0; i < count; ++i) {
        const glm::dvec2 delta = ring[(i + 1) % count] - ring[i];
        EdgeGeometry& g = geometry[i];
        g.length = length_of(delta);
        g.normal = safe_normalize(glm::dvec2{delta.y, -delta.x} * orientation);
    }

    return geometry;
}

// ============================================================================
// Source Inference
// ============================================================================

/**
 * @brief Find the block ring segment that @p a -> @p b lies along, or kNoIndex
 *
 * Containment, not proximity. The lot edge must be collinear with the block ring
 * segment to within @p tolerance, must lie inside that segment's span, and must
 * run the SAME way round the ring. Those three together mean the lot edge is
 * literally a piece of that boundary, which is the only thing that justifies
 * calling it frontage.
 *
 * The direction test is the one that is easy to leave out and expensive to omit. A
 * lot ring and a block ring are both anticlockwise, so a lot edge sitting on the
 * boundary runs with it. An edge running against it is the boundary of the lot on
 * the OTHER side, which means the caller has handed over a ring wound the wrong
 * way or a lot from a different block; adopting the source anyway would attribute
 * a frontage that belongs to somebody else.
 *
 * @param a         Start of the lot edge
 * @param b         End of the lot edge
 * @param block     Block supplying `ring` and `ring_edges`
 * @param tolerance Collinearity and containment tolerance in metres
 * @return Index into Block::edges, or kNoIndex
 */
size_t infer_source(const glm::dvec2& a, const glm::dvec2& b, const Block& block,
                    double tolerance) {
    const size_t ring_size = block.ring.size();
    if (ring_size < 3 || block.ring_edges.size() != ring_size) {
        return kNoIndex;
    }

    const glm::dvec2 lot_delta = b - a;
    if (length_of(lot_delta) < kLengthEpsilon) {
        return kNoIndex;
    }

    const double tol = std::max(tolerance, 0.0);

    for (size_t i = 0; i < ring_size; ++i) {
        const glm::dvec2& p = block.ring[i];
        const glm::dvec2& q = block.ring[(i + 1) % ring_size];
        const glm::dvec2 seg = q - p;
        const double seg_len = length_of(seg);
        if (seg_len < kLengthEpsilon) {
            continue;
        }

        // Perpendicular distance of both lot endpoints from the infinite line.
        const glm::dvec2 unit = seg / seg_len;
        const glm::dvec2 to_a = a - p;
        const glm::dvec2 to_b = b - p;
        const double perp_a = unit.x * to_a.y - unit.y * to_a.x;
        const double perp_b = unit.x * to_b.y - unit.y * to_b.x;
        if (std::fabs(perp_a) > tol || std::fabs(perp_b) > tol) {
            continue;
        }

        // Positions along the segment, in metres from p.
        const double at_a = unit.x * to_a.x + unit.y * to_a.y;
        const double at_b = unit.x * to_b.x + unit.y * to_b.y;
        if (at_a < -tol || at_b < -tol || at_a > seg_len + tol || at_b > seg_len + tol) {
            continue;
        }
        if (at_b - at_a <= kLengthEpsilon) {
            continue;   // zero-length overlap, or running against the boundary
        }

        return block.ring_edges[i];
    }

    return kNoIndex;
}

// ============================================================================
// Frontage Runs
// ============================================================================

/**
 * @brief One contiguous stretch of frontage: where it starts, how long it is
 *
 * The LENGTH is carried rather than recomputed by the caller because two separate
 * decisions read it -- the sliver floor and the midpoint -- and a caller that
 * re-derived it for one of them could disagree with the run the other used.
 */
struct FrontageRun {
    size_t start = kNoIndex;   ///< First ring segment of the run
    size_t count = 0;          ///< Segments in the run; 0 when there is no run
    double length = 0.0;       ///< Arc length of the run in metres
};

/**
 * @brief Longest contiguous cyclic run of set bits, and its arc length
 *
 * Contiguity is CYCLIC because a lot ring is. A frontage that straddles the ring's
 * seam -- the subdivider happened to start the outline halfway along the street
 * edge -- is one run, and a linear scan would report it as two halves and put the
 * midpoint at the join between them, which is a lot corner rather than the middle
 * of the frontage.
 *
 * Ties on length are broken by the lowest start index, so the answer does not
 * depend on scan order.
 *
 * @param mask     One entry per ring segment
 * @param geometry Segment lengths, parallel to @p mask
 * @return The longest run, or a default FrontageRun when nothing is set
 */
FrontageRun longest_run(const std::vector<uint8_t>& mask,
                        const std::vector<EdgeGeometry>& geometry) {
    const size_t count = mask.size();
    if (count == 0) {
        return {};
    }

    bool any_clear = false;
    bool any_set = false;
    for (const uint8_t bit : mask) {
        any_set = any_set || bit != 0;
        any_clear = any_clear || bit == 0;
    }
    if (!any_set) {
        return {};
    }
    if (!any_clear) {
        // Every segment belongs to the run, so there is no start to find and the
        // cycle never breaks -- the scan below looks for a segment whose PREDECESSOR
        // is clear, and there is none, so without this branch it would report no run
        // at all. Start anywhere; index 0 keeps it deterministic. The case is real:
        // a block ringed by one closed way (an estate loop road, a roundabout) hands
        // every ring segment to the same half-edge, so a lot filling that block has
        // its whole outline in one frontage.
        double total = 0.0;
        for (const EdgeGeometry& segment : geometry) {
            total += segment.length;
        }
        return {0, count, total};
    }

    size_t best_start = kNoIndex;
    size_t best_count = 0;
    double best_length = -1.0;

    for (size_t i = 0; i < count; ++i) {
        if (mask[i] == 0 || mask[(i + count - 1) % count] != 0) {
            continue;   // not the first segment of a run
        }

        size_t run = 0;
        double run_length = 0.0;
        while (run < count && mask[(i + run) % count] != 0) {
            run_length += geometry[(i + run) % count].length;
            ++run;
        }

        if (run_length > best_length) {
            best_length = run_length;
            best_start = i;
            best_count = run;
        }
    }

    // any_set with any_clear guarantees at least one set segment whose predecessor
    // is clear, so the scan above always fired and best_length is no longer -1.
    return {best_start, best_count, best_length};
}

/**
 * @brief Point half way along the run starting at @p start, by arc length
 *
 * Falls back to the run's first vertex when the run has no length at all, which is
 * the only honest answer for a frontage made entirely of null segments.
 */
glm::dvec2 run_midpoint(const std::vector<glm::dvec2>& ring,
                        const std::vector<EdgeGeometry>& geometry, size_t start,
                        size_t run) {
    const size_t count = ring.size();
    if (count == 0 || run == 0 || start >= count) {
        return glm::dvec2{0.0};
    }

    double total = 0.0;
    for (size_t k = 0; k < run; ++k) {
        total += geometry[(start + k) % count].length;
    }
    if (total < kLengthEpsilon) {
        return ring[start];
    }

    const double target = 0.5 * total;
    double walked = 0.0;
    for (size_t k = 0; k < run; ++k) {
        const size_t i = (start + k) % count;
        const double len = geometry[i].length;
        if (len >= kLengthEpsilon && walked + len >= target) {
            const double t = (target - walked) / len;
            return ring[i] + (ring[(i + 1) % count] - ring[i]) * t;
        }
        walked += len;
    }

    // Rounding only: the loop above consumed slightly less than `total`.
    return ring[(start + run - 1) % count];
}

// ============================================================================
// Primary Front
// ============================================================================

/**
 * @brief Index of the primary frontage in @p frontages, which must not be empty
 *
 * Two-stage rather than one comparator, and on purpose. A comparator holding a
 * tolerance -- "longer by more than 25 cm wins" -- is not transitive, so with three
 * frontages the winner of a pairwise scan depends on the order they are visited.
 * Selecting a candidate SET on the leading criterion and then settling it on the
 * next has no such hole: every stage is a plain minimum or maximum.
 *
 * @param frontages Surviving frontages, in ascending first-ring-edge order
 * @param ranks     Road class rank per frontage, parallel to @p frontages
 * @param config    Rule and tie band
 */
size_t pick_primary(const std::vector<LotFrontage>& frontages, const std::vector<int>& ranks,
                    const LotEdgeConfig& config) {
    const double tie = std::max(config.primary_tie_epsilon, 0.0);

    std::vector<size_t> candidates;
    candidates.reserve(frontages.size());

    if (config.primary_rule == PrimaryFrontRule::HighestRoadClass) {
        int best_rank = ranks[0];
        for (const int rank : ranks) {
            best_rank = std::min(best_rank, rank);
        }
        for (size_t i = 0; i < frontages.size(); ++i) {
            if (ranks[i] == best_rank) {
                candidates.push_back(i);
            }
        }

        double best_length = frontages[candidates[0]].length;
        for (const size_t i : candidates) {
            best_length = std::max(best_length, frontages[i].length);
        }
        std::vector<size_t> narrowed;
        for (const size_t i : candidates) {
            if (frontages[i].length >= best_length - tie) {
                narrowed.push_back(i);
            }
        }
        candidates.swap(narrowed);
    } else {
        double best_length = frontages[0].length;
        for (const LotFrontage& frontage : frontages) {
            best_length = std::max(best_length, frontage.length);
        }
        for (size_t i = 0; i < frontages.size(); ++i) {
            if (frontages[i].length >= best_length - tie) {
                candidates.push_back(i);
            }
        }

        int best_rank = ranks[candidates[0]];
        for (const size_t i : candidates) {
            best_rank = std::min(best_rank, ranks[i]);
        }
        std::vector<size_t> narrowed;
        for (const size_t i : candidates) {
            if (ranks[i] == best_rank) {
                narrowed.push_back(i);
            }
        }
        candidates.swap(narrowed);
    }

    // Still tied: the lexicographic minimum of (EdgeId, first ring-edge index).
    // Both keys are integers, so the minimum is unique and this terminates on a
    // single candidate every time. A corner lot at a crossroads of two identical
    // residential streets lands here, and it has to land on the same answer on
    // every run or every rule assignment downstream reshuffles when nothing
    // changed.
    //
    // Written as one comparison rather than a stage per key, and the ring index is
    // in the key even though pass 2 already emits frontages in ascending ring
    // order. With that order the FIRST candidate is always the one with the lowest
    // ring index, so the second key never overturns a winner and no test can catch
    // its removal -- but it is what makes the answer a function of the frontages
    // rather than of the order they were built in, and that order is established
    // eighty lines away in another pass. Reversing the second key DOES change the
    // answer, which is what pins it: see the cul-de-sac spur test, where one street
    // bounds the block twice (blocks.hpp walks up one side of the spur and back
    // down the other) and a lot at its head fronts that one street from both sides.
    // The front door goes on the side the lot outline reaches first.
    const auto key = [&frontages](size_t i) {
        const LotFrontage& frontage = frontages[i];
        // edges is never empty and is ascending: pass 2 appends the edge that
        // created the group in the same iteration, and appends in ring order after
        // that, so front() is the lowest ring index of the frontage.
        return std::pair<EdgeId, uint32_t>{frontage.street, frontage.edges.front()};
    };

    size_t best = candidates.front();
    for (const size_t i : candidates) {
        if (key(i) < key(best)) {
            best = i;
        }
    }
    return best;
}

} // namespace

// ============================================================================
// Shared Predicates
// ============================================================================

// street_class_rank() leans on RoadType being DECLARED in descending order of
// importance. That is true today and is not written down anywhere the compiler can
// see, so pin it here: reordering the enum for any reason -- alphabetising it,
// inserting Unclassified in the middle -- silently changes which street a corner
// lot addresses, and a silently different address is exactly the failure this file
// is supposed to make impossible.
static_assert(static_cast<int>(RoadType::Motorway) < static_cast<int>(RoadType::Primary),
              "RoadType must stay ordered by descending importance");
static_assert(static_cast<int>(RoadType::Primary) < static_cast<int>(RoadType::Residential),
              "RoadType must stay ordered by descending importance");
static_assert(static_cast<int>(RoadType::Residential) < static_cast<int>(RoadType::Service),
              "RoadType must stay ordered by descending importance");
static_assert(static_cast<int>(RoadType::Service) < static_cast<int>(RoadType::Unknown),
              "RoadType must stay ordered by descending importance");

const char* lot_edge_role_name(LotEdgeRole role) {
    switch (role) {
        case LotEdgeRole::Front:    return "Front";
        case LotEdgeRole::Side:     return "Side";
        case LotEdgeRole::Back:     return "Back";
        case LotEdgeRole::Interior: return "Interior";
    }
    return "Interior";
}

const char* street_side_name(StreetSide side) {
    return side == StreetSide::Left ? "Left" : "Right";
}

bool is_street_class(RoadType type, bool paths_are_streets) {
    switch (type) {
        case RoadType::Footway:
        case RoadType::Cycleway:
        case RoadType::Path:
            return paths_are_streets;
        default:
            return true;
    }
}

int street_class_rank(RoadType type) {
    return static_cast<int>(type);
}

// ============================================================================
// LotEdgeClassification
// ============================================================================

size_t LotEdgeClassification::count(LotEdgeRole role) const {
    size_t total = 0;
    for (const LotEdgeRole value : roles) {
        if (value == role) {
            ++total;
        }
    }
    return total;
}

// ============================================================================
// Classification
// ============================================================================

LotEdgeClassification classify_lot_edges(const LotOutline& lot, const Block& block,
                                         const RoadGraph& graph, const LotEdgeConfig& config) {
    LotEdgeClassification out;

    const size_t count = lot.ring.size();

    // A mismatched edge_sources is rejected rather than truncated to the shorter of
    // the two. Truncating classifies the edges that do line up and is therefore
    // almost right, which is worse: the lot comes back with a plausible front on
    // the wrong street and nothing anywhere says so.
    if (count < 3 || lot.edge_sources.size() != count) {
        out.malformed = true;
        return out;
    }

    const double area = signed_ring_area(lot.ring);
    if (std::fabs(area) < kAreaEpsilon) {
        out.malformed = true;   // no interior, so "outward" means nothing
        return out;
    }

    // A clockwise ring is corrected, not rejected and not trusted. Every normal
    // below is taken from the winding, so believing the documented anticlockwise
    // convention on a ring that arrived reversed swaps FRONT with BACK on every
    // edge of the lot -- and the result still looks like a valid classification.
    out.orientation_corrected = area < 0.0;
    const double orientation = out.orientation_corrected ? -1.0 : 1.0;

    const std::vector<EdgeGeometry> geometry = edge_geometry(lot.ring, orientation);

    // ------------------------------------------------------------------------
    // Pass 1: resolve provenance
    // ------------------------------------------------------------------------
    std::vector<uint32_t> source(count, kNoBoundaryEdge);
    for (size_t i = 0; i < count; ++i) {
        uint32_t src = lot.edge_sources[i];

        if (src != kNoBoundaryEdge && src >= block.edges.size()) {
            // The lot was cut from a different block, or the subdivider stored an
            // EdgeId here instead of a Block::edges index. Counted, not silently
            // dropped: both mistakes produce lots that classify "fine".
            ++out.invalid_sources;
            src = kNoBoundaryEdge;
        }

        if (src == kNoBoundaryEdge && config.infer_sources_from_block_ring) {
            const size_t inferred = infer_source(lot.ring[i], lot.ring[(i + 1) % count], block,
                                                 config.source_snap_tolerance);
            if (inferred != kNoIndex && inferred < block.edges.size()) {
                src = static_cast<uint32_t>(inferred);
                ++out.inferred_sources;
            }
        }

        source[i] = src;
    }

    // ------------------------------------------------------------------------
    // Pass 2: group the street edges into frontages
    //
    // Grouped by Block::edges index, which is one half-edge, so `side` is exact.
    // See the LotFrontage documentation for why not by EdgeId and not by name.
    // A linear scan rather than a hash map: a lot has a handful of edges, and the
    // scan keeps frontages in ascending first-ring-edge order for free, which the
    // primary tie-break relies on.
    // ------------------------------------------------------------------------
    out.roles.assign(count, LotEdgeRole::Interior);

    std::vector<LotFrontage> frontages;
    std::vector<glm::dvec2> weighted_normals;   // parallel to frontages
    std::vector<int> ranks;                     // parallel to frontages

    const std::vector<GraphEdge>& graph_edges = graph.edges();

    for (size_t i = 0; i < count; ++i) {
        if (source[i] == kNoBoundaryEdge) {
            continue;
        }

        const BlockEdge& block_edge = block.edges[source[i]];
        if (block_edge.edge >= graph_edges.size()) {
            ++out.invalid_sources;
            out.roles[i] = LotEdgeRole::Side;   // it bounds the block; it is not a street
            continue;
        }

        const GraphEdge& street = graph_edges[block_edge.edge];
        if (!is_street_class(street.type, config.paths_are_streets)) {
            out.roles[i] = LotEdgeRole::Side;
            continue;
        }

        size_t group = kNoIndex;
        for (size_t g = 0; g < frontages.size(); ++g) {
            if (frontages[g].block_edge == source[i]) {
                group = g;
                break;
            }
        }
        if (group == kNoIndex) {
            LotFrontage frontage;
            frontage.street = block_edge.edge;
            frontage.block_edge = source[i];
            // blocks.hpp: the block lies to the LEFT of each bounding half-edge. A
            // forward half-edge runs with the graph edge, so the lot is left of the
            // centreline; a reversed one runs against it, so the lot is right of it.
            frontage.side = block_edge.forward ? StreetSide::Left : StreetSide::Right;
            group = frontages.size();
            frontages.push_back(std::move(frontage));
            weighted_normals.emplace_back(0.0);
            ranks.push_back(street_class_rank(street.type));
        }

        frontages[group].edges.push_back(static_cast<uint32_t>(i));
        frontages[group].length += geometry[i].length;
        weighted_normals[group] += geometry[i].normal * geometry[i].length;
        out.roles[i] = LotEdgeRole::Front;
    }

    // ------------------------------------------------------------------------
    // Pass 3: demote slivers, then finish the survivors
    //
    // A four-centimetre clip of the boundary at a lot corner is a subdivision
    // artefact, not a frontage, and left alone it turns an ordinary mid-block lot
    // into a corner lot that a rule hangs a second shopfront on.
    //
    // The floor is measured on the LONGEST CONTIGUOUS RUN of the frontage, which
    // is what LotEdgeConfig::min_frontage_length says it is, and NOT on the
    // frontage total. One lot can clip one street in several places -- a notch
    // bitten out of its street edge, a subdivider rounding two corners against the
    // same boundary -- and two 60 cm slivers sum past a 1 m floor while no single
    // stretch of kerb is long enough to put a door on. Summing therefore lets
    // exactly the artefact this floor exists to suppress through, and gives it a
    // confident FRONT whose midpoint sits in the gap between the slivers.
    // ------------------------------------------------------------------------
    std::vector<LotFrontage> kept;
    std::vector<int> kept_ranks;
    kept.reserve(frontages.size());
    kept_ranks.reserve(ranks.size());

    for (size_t g = 0; g < frontages.size(); ++g) {
        LotFrontage frontage = std::move(frontages[g]);

        // frontage.edges is never empty -- pass 2 appends the edge that created the
        // group in the same iteration -- so the mask always has a bit set and
        // longest_run() always comes back with a real run.
        std::vector<uint8_t> mask(count, 0);
        for (const uint32_t edge : frontage.edges) {
            mask[edge] = 1;
        }
        const FrontageRun run = longest_run(mask, geometry);

        if (run.length < config.min_frontage_length) {
            for (const uint32_t edge : frontage.edges) {
                out.roles[edge] = LotEdgeRole::Side;
            }
            ++out.demoted_frontages;
            continue;
        }

        frontage.normal = safe_normalize(weighted_normals[g]);
        if (length_sq(frontage.normal) < 0.5) {
            // The weighted sum cancelled: the frontage wraps far enough round a
            // bend that its normals oppose each other. The extreme case is a lot
            // filling a block ringed by one closed way, where the frontage closes
            // on itself and the sum is exactly zero. Rare, but a zero normal would
            // aim the facade nowhere, so fall back to the longest single segment,
            // which is the best-supported direction available.
            size_t longest = frontage.edges.front();
            for (const uint32_t edge : frontage.edges) {
                if (geometry[edge].length > geometry[longest].length) {
                    longest = edge;
                }
            }
            frontage.normal = geometry[longest].normal;
        }

        frontage.midpoint = run_midpoint(lot.ring, geometry, run.start, run.count);

        kept.push_back(std::move(frontage));
        kept_ranks.push_back(ranks[g]);
    }

    out.frontages = std::move(kept);

    // ------------------------------------------------------------------------
    // Pass 4: the primary front, then everything that touches no boundary
    // ------------------------------------------------------------------------
    if (out.frontages.empty()) {
        // Landlocked. There is no reference direction, so BACK and the two flavours
        // of interior edge are all undefined for this lot -- and inventing one from
        // the longest edge, or from the lot's own axis, would hand a caller a front
        // normal with nothing behind it. Interior edges stay INTERIOR; boundary
        // edges already became SIDE above.
        out.landlocked = true;
        out.primary_frontage = kNoFrontage;
        return out;
    }

    const size_t primary = pick_primary(out.frontages, kept_ranks, config);
    out.frontages[primary].primary = true;
    out.primary_frontage = static_cast<uint32_t>(primary);

    const glm::dvec2 front_normal = out.frontages[primary].normal;
    const double back_cos = cone_cosine(config.back_cone_degrees);
    const double front_cos = cone_cosine(config.front_cone_degrees);

    for (size_t i = 0; i < count; ++i) {
        if (out.roles[i] != LotEdgeRole::Interior) {
            continue;   // already FRONT or SIDE from its provenance
        }
        if (geometry[i].length < kLengthEpsilon) {
            continue;   // no direction to compare; stays INTERIOR
        }

        const double facing = geometry[i].normal.x * front_normal.x +
                              geometry[i].normal.y * front_normal.y;

        // BACK first. With the default 60 degree cones the three bands partition
        // the circle exactly and the order does not matter; a caller that widens
        // both past 90 makes them overlap, and an edge pointing squarely away from
        // the street is a back before it is anything else.
        if (facing <= -back_cos) {
            out.roles[i] = LotEdgeRole::Back;
        } else if (facing >= front_cos) {
            out.roles[i] = LotEdgeRole::Interior;   // faces the street, touches nothing
        } else {
            out.roles[i] = LotEdgeRole::Side;       // the party line with the neighbour
        }
    }

    return out;
}

LotEdgeClassificationSet classify_block_lots(const std::vector<LotOutline>& lots,
                                             const Block& block, const RoadGraph& graph,
                                             const LotEdgeConfig& config) {
    LotEdgeClassificationSet result;
    result.lots.reserve(lots.size());
    result.stats.lots = lots.size();

    for (size_t i = 0; i < lots.size(); ++i) {
        // One malformed lot must not cost the other ninety their classification, so
        // the loop never breaks. It is recorded in malformed_lots instead.
        LotEdgeClassification classification = classify_lot_edges(lots[i], block, graph, config);

        result.stats.demoted_frontages += classification.demoted_frontages;
        result.stats.invalid_sources += classification.invalid_sources;
        result.stats.inferred_sources += classification.inferred_sources;

        if (classification.malformed) {
            ++result.stats.malformed;
            result.malformed_lots.push_back(static_cast<uint32_t>(i));
            result.lots.push_back(std::move(classification));
            continue;
        }

        ++result.stats.classified;
        if (classification.orientation_corrected) {
            ++result.stats.orientation_corrected;
        }
        if (classification.landlocked) {
            ++result.stats.landlocked;
            result.landlocked_lots.push_back(static_cast<uint32_t>(i));
        }
        if (classification.frontages.size() >= 2) {
            ++result.stats.corner_lots;
        }

        result.stats.front_edges += classification.count(LotEdgeRole::Front);
        result.stats.side_edges += classification.count(LotEdgeRole::Side);
        result.stats.back_edges += classification.count(LotEdgeRole::Back);
        result.stats.interior_edges += classification.count(LotEdgeRole::Interior);

        result.lots.push_back(std::move(classification));
    }

    spdlog::info("LotEdges: {} of {} lots classified — {} landlocked, {} corner, "
                 "{} malformed; edges {} front / {} side / {} back / {} interior",
                 result.stats.classified, result.stats.lots, result.stats.landlocked,
                 result.stats.corner_lots, result.stats.malformed, result.stats.front_edges,
                 result.stats.side_edges, result.stats.back_edges, result.stats.interior_edges);

    if (result.stats.invalid_sources > 0 || result.stats.demoted_frontages > 0) {
        spdlog::warn("LotEdges: {} lot edges carried a source outside Block::edges and "
                     "{} frontages were shorter than {:.2f} m",
                     result.stats.invalid_sources, result.stats.demoted_frontages,
                     config.min_frontage_length);
    }

    return result;
}

} // namespace stratum::osm::road
