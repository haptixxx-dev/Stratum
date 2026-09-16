// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file lot_edges.hpp
 * @brief Which side of a lot is the front, the side, the back, or nothing at all
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### Why this exists
 *
 * A rule that says "set the building back 5 m and put the shopfront on the
 * street" cannot run until something has decided which edge of the lot IS the
 * street. Every downstream stage asks the same question in a different voice:
 * E3 wants the setback distance per edge, the rule engine wants to know where the
 * glazed facade goes and where the blank party wall goes, and a driveway wants a
 * point on the kerb. All four are one classification, computed once here.
 *
 * Get it wrong and the failure is not a crash. It is a terrace of houses with
 * their front doors facing the back fence, which looks plausible in a thumbnail
 * and is obvious from street level.
 *
 * ### What "facing a street" means, precisely
 *
 * It means IDENTITY, not proximity.
 *
 * `extract_blocks()` builds a block ring out of road-graph half-edges and records,
 * in `Block::ring_edges`, exactly which half-edge carries each ring segment. A lot
 * subdivider cuts its lots out of that ring, so for every lot edge it can say
 * which block half-edge the edge was inherited from -- or that the edge is an
 * interior cut and was inherited from nothing. `LotOutline::edge_sources` is that
 * answer, and it is the whole input to the frontage test:
 *
 *     lot edge came from block half-edge h, and h's graph edge is a carriageway
 *       ==> that lot edge is frontage on that street
 *
 * Nothing here measures a distance to a road centreline, and the difference is
 * not academic. A service road running two metres behind a row of gardens is
 * nearer to the rear fences than the street out front is to the front gates,
 * because the front gates sit behind a pavement and a verge. Classifying by
 * proximity puts the shopfront on the back fence. It is the same mistake as
 * finding junctions by endpoint proximity, which the whole road pipeline is built
 * to avoid, and it fails in the same places: anywhere two streets run close
 * together, and anywhere a lot is deep.
 *
 * Identity also hands over two things proximity cannot. `BlockEdge::forward` says
 * the block lies to the LEFT of the half-edge, so the lot fronts a KNOWN side of
 * that centreline -- `LotFrontage::side` -- which is what E3 needs to pick the
 * correct kerb line and sidewalk width out of the road profile. And the frontage
 * carries the `EdgeId` itself, so a rule can read the street's name, class and
 * width without a search.
 *
 * ### When identity is missing
 *
 * An edge whose source is `kNoBoundaryEdge` is never frontage. That is a deliberate
 * refusal rather than an omission: the honest answer to "this lot lost its
 * provenance" is that the lot is landlocked and is REPORTED as such, not that the
 * classifier guessed from geometry. A landlocked lot is a real outcome of
 * subdivision anyway -- the interior of a deep block subdivides into parcels that
 * genuinely touch no street -- so the report has to exist regardless, and routing
 * lost provenance into it costs nothing.
 *
 * The one fallback offered is `LotEdgeConfig::infer_sources_from_block_ring`, and
 * it is OFF by default. It does not search for nearby roads. It asks whether the
 * lot edge lies ON a segment of the block ring it was cut from: collinear to
 * within a centimetre, contained within that segment's span, and running the same
 * way round. That can only ever recover a boundary the lot literally sits on, so
 * it cannot invent frontage for an interior parcel. It is off by default because a
 * subdivider that tracked its cuts correctly never needs it, and leaving it on
 * would hide the day one stops tracking them.
 *
 * ### The four roles
 *
 * Tested in this order, and the order is the specification:
 *
 *   1. **FRONT** -- inherited from a block half-edge whose graph edge is a street.
 *      A lot may have SEVERAL fronts. A corner lot has two and that is not an
 *      error; a through-lot spanning a block from street to street has two and no
 *      back at all.
 *   2. **SIDE** -- inherited from a block half-edge that is NOT a street (a
 *      footway bounding the block when `BlockConfig::include_paths` was on), or
 *      frontage too short to be frontage; see `min_frontage_length`.
 *   3. **BACK** -- an interior cut whose outward normal points away from the
 *      primary front, within `back_cone_degrees`.
 *   4. **INTERIOR** -- an interior cut whose outward normal points TOWARDS the
 *      primary front without touching it. That is the rear lot of a flag-shaped
 *      subdivision looking at the back of the lot in front of it, and calling it a
 *      back would aim its rear elevation at the street.
 *   5. Anything else interior, which is to say roughly perpendicular to the front,
 *      is **SIDE**: the party boundary with the neighbour beside you.
 *
 * A lot with no surviving front has no reference direction, so steps 3 to 5 have
 * no meaning for it. Every interior edge of a landlocked lot is INTERIOR, and the
 * lot is listed in `LotEdgeClassificationSet::landlocked_lots`.
 *
 * ### Which front is primary
 *
 * Rules want ONE front: one address, one shopfront, one driveway. By default the
 * primary front is the LONGEST frontage, because the long side of a lot is the one
 * a building is shaped along.
 *
 * Length alone is not always right, and the counter-example is the one that
 * matters commercially: a corner shop between a high street and a residential side
 * street puts its window on the high street even when the side frontage is longer.
 * `PrimaryFrontRule::HighestRoadClass` inverts the two criteria for callers that
 * want that, and either way the other criterion is the tie-break -- a square corner
 * lot at a crossroads of two identical residential streets has two frontages of
 * equal length and equal class, and the answer still has to be the same on every
 * run. See `LotEdgeConfig::primary_tie_epsilon`.
 *
 * ### Asking "which edge is the front" afterwards
 *
 * Never re-derive it. `LotEdgeClassification` carries the answer in three forms
 * and a caller picks whichever is convenient:
 *
 *   - `roles[i]` is the role of the lot ring segment from `ring[i]` to
 *     `ring[i + 1]`, for painting, for per-edge setbacks, for facade selection.
 *   - `frontages` groups the front edges by the street half-edge they front, with
 *     the street's `EdgeId`, which side of its centreline the lot is on, the total
 *     frontage length, the outward normal, and a midpoint on the kerb-facing edge.
 *   - `primary_front()` returns the one frontage a rule should use, or nullptr
 *     when the lot is landlocked. `front_normal()` and `front_midpoint()` are the
 *     two values a facade or a driveway actually wants, and both return zero
 *     vectors for a landlocked lot rather than a plausible-looking direction.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API. It is pure 2D geometry over a `Block` and its `RoadGraph`.
 */

#pragma once

#include "osm/road/blocks.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/types.hpp"

#include <glm/glm.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Sentinels
// ============================================================================

/**
 * @brief `LotOutline::edge_sources` value for an edge inherited from no boundary
 *
 * Numerically equal to `kInvalidId`, and deliberately spelled differently: this is
 * an index into `Block::edges`, not an `EdgeId`, and the two index spaces have
 * nothing to do with each other. A caller that stores an `EdgeId` here gets a
 * confident, wrong classification.
 *
 * It is also spelled differently from the lot subdivider's own `kNoBlockEdge`,
 * which holds the same value for the same meaning. Not a preference: an
 * `inline constexpr` variable may be defined once per translation unit, and this
 * file and the subdivider's header are meant to be included together, so two
 * definitions of one name in `stratum::osm::road` would not compile. The values
 * agree, so passing a subdivider's `ring_edges` straight into `edge_sources`
 * needs no translation.
 */
inline constexpr uint32_t kNoBoundaryEdge = 0xFFFFFFFFu;

/// `LotEdgeClassification::primary_frontage` value for a lot with no front
inline constexpr uint32_t kNoFrontage = 0xFFFFFFFFu;

// ============================================================================
// Roles
// ============================================================================

/**
 * @brief What one edge of a lot is for
 *
 * Deliberately four values and no "Unknown". Every edge of a well-formed lot gets
 * one of these, and the case that would want an Unknown -- a lot that touches no
 * street -- is reported as a landlocked LOT rather than smeared across its edges,
 * because a caller that has to notice one flag per lot will notice it, and a caller
 * scanning for an odd enum value in a role array will not.
 */
enum class LotEdgeRole : uint8_t {
    Front,      ///< On the block boundary, facing a street
    Side,       ///< Boundary that is not a street, or the party line with a neighbour
    Back,       ///< Interior cut facing away from the primary front
    Interior,   ///< Interior cut that faces no street; also every edge of a landlocked lot
};

/// Human-readable role name, for logs and for test failure messages
[[nodiscard]] const char* lot_edge_role_name(LotEdgeRole role);

/**
 * @brief Which side of a street's centreline the lot sits on
 *
 * Taken from `BlockEdge::forward` and the block-is-on-the-left contract of
 * blocks.hpp, so it is exact rather than inferred from a cross product against a
 * centreline that may curve away.
 *
 * E3 needs it: a road profile is asymmetric. `sidewalk`, `cycleway` and `parking`
 * are all per-side `SideFlags`, so the distance from the centreline to the kerb in
 * front of this lot depends on which side the lot is.
 */
enum class StreetSide : uint8_t {
    Left,   ///< Lot is left of the graph edge's own from -> to direction
    Right,  ///< Lot is right of it
};

/// Human-readable side name, for logs and for test failure messages
[[nodiscard]] const char* street_side_name(StreetSide side);

/**
 * @brief Which criterion decides the primary front of a corner lot
 */
enum class PrimaryFrontRule : uint8_t {
    /// Longest frontage wins; road class breaks a tie. The default.
    LongestFrontage,

    /// Most important road wins; frontage length breaks a tie. What a retail
    /// frontage wants: the shop window goes on the high street, not on the longer
    /// side street.
    HighestRoadClass,
};

// ============================================================================
// Input
// ============================================================================

/**
 * @brief A lot polygon and, per edge, the block boundary it was inherited from
 *
 * The minimal contract a subdivider has to meet, and it is deliberately two plain
 * vectors rather than a dependency on the subdivider's own lot type -- this file
 * must not have to change when that type grows an area, a seed or a zoning
 * attribute.
 *
 * Expectations, all of which are checked rather than assumed:
 *
 *   - `ring` is a closed ring in LOCAL METRES with the first point NOT repeated,
 *     the same convention as `Block::ring` and `Corridor::outline`. At least three
 *     points.
 *   - `ring` is ANTICLOCKWISE, matching `Block::ring`. A clockwise ring is not
 *     rejected: its orientation is detected from the signed area and every outward
 *     normal is flipped to suit, and `orientation_corrected` records that it
 *     happened. Trusting the documented winding instead would mirror front and
 *     back in silence, which is the exact failure this file exists to prevent.
 *   - `edge_sources` is PARALLEL TO `ring` and the same size. `edge_sources[i]`
 *     describes the segment from `ring[i]` to `ring[(i + 1) % ring.size()]`. It
 *     holds an index into `Block::edges` -- the same index space as
 *     `Block::ring_edges` -- or `kNoBoundaryEdge` for an edge the subdivider cut
 *     itself. A size mismatch sets `malformed` and classifies nothing, because a
 *     one-off misalignment would otherwise attribute every frontage to the
 *     neighbouring street.
 *
 * Joining this to a subdivider's own lot type is a copy of two vectors. A lot cut
 * by clipping the block ring already knows, per output vertex, which input segment
 * it came from; that index, passed through unchanged, IS `edge_sources`.
 */
struct LotOutline {
    /// Lot boundary in local metres, anticlockwise, first point not repeated
    std::vector<glm::dvec2> ring;

    /// Parallel to `ring`: index into `Block::edges`, or `kNoBoundaryEdge`
    std::vector<uint32_t> edge_sources;
};

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Tunables of the classification
 */
struct LotEdgeConfig {
    /**
     * @brief Half-angle of the cone around the reversed front normal that is BACK
     *
     * An interior edge whose outward normal sits within this many degrees of
     * directly away from the primary front is the back of the lot. 60 degrees is
     * wide enough for the rear boundary of a wedge-shaped lot on the outside of a
     * bend, which is never parallel to its frontage, and narrow enough that the
     * party boundary with the neighbour beside you stays a SIDE.
     */
    double back_cone_degrees = 60.0;

    /**
     * @brief Half-angle of the cone around the front normal that is INTERIOR
     *
     * An interior edge pointing the same way as the frontage is looking at
     * whatever stands between this lot and the street. It is not a front, because
     * it touches no street, and it is certainly not a back.
     *
     * With both cones at 60 the three bands partition the circle exactly: back
     * below -0.5, interior above +0.5, side between. Widening either past the gap
     * makes them overlap, and BACK is then tested first.
     */
    double front_cone_degrees = 60.0;

    /**
     * @brief Shortest run of frontage on one street that counts as a front, metres
     *
     * Subdivision leaves slivers. A cut that clips the block boundary for four
     * centimetres at a corner is a rounding artefact, and without this floor it
     * makes an ordinary mid-block lot into a corner lot with a second front, which
     * a rule then hangs a shopfront on.
     *
     * RUN, not total, and the distinction is the whole point. A lot can clip one
     * street in several places -- a notch bitten out of its street edge, a
     * subdivider rounding two corners against the same boundary -- and two 60 cm
     * slivers sum past a 1 m floor while no single stretch of kerb is long enough
     * to put a door on. Measuring the total would admit exactly the artefact this
     * floor exists to suppress, and would then place the front midpoint in the gap
     * between the slivers.
     *
     * A frontage whose longest run is below the floor is demoted to SIDE in its
     * entirety -- every sliver of it, not just the short ones -- and counted once
     * in `demoted_frontages`. A lot whose ONLY frontage is demoted becomes
     * landlocked and is reported, rather than quietly keeping a four-centimetre
     * front.
     */
    double min_frontage_length = 1.0;

    /**
     * @brief Treat footways, cycleways and paths as streets. Default OFF.
     *
     * The mirror of `BlockConfig::include_paths` and it should normally match it.
     * Off, a lot backing onto a mapped alley bounds the block there but does not
     * front it, so the alley gets a SIDE and the street keeps the shopfront. On is
     * right for a pedestrianised centre, where the footways ARE the streets and a
     * lot with no vehicle frontage would otherwise be reported landlocked.
     */
    bool paths_are_streets = false;

    /// Which criterion picks the primary front of a corner lot
    PrimaryFrontRule primary_rule = PrimaryFrontRule::LongestFrontage;

    /**
     * @brief Frontage lengths within this many metres of each other are a tie
     *
     * Not a tolerance on floating-point error -- it is a statement that a quarter
     * of a metre of extra frontage does not decide where the front door goes. A
     * square corner lot has two frontages that agree to the last bit, and without
     * an explicit tie band the winner would be decided by whichever accumulated
     * its length with less rounding. That is stable within a run and changes when
     * anything upstream moves a vertex, which is the worst kind of non-determinism
     * to debug.
     *
     * Ties are then settled by road class, then by the lower `EdgeId`, then by the
     * lower ring-edge index, so the answer never depends on iteration order.
     *
     * The last of those three stages decides only between two frontages on the
     * SAME graph edge. That is not a hypothetical: one street bounds a block twice
     * wherever a cul-de-sac spur hangs inside it, because the face traversal walks
     * up one side of the spur and back down the other, and a lot at the head of the
     * spur fronts that one street from both sides. The front door then goes on the
     * side the lot outline reaches first.
     */
    double primary_tie_epsilon = 0.25;

    /**
     * @brief Recover a missing source from the block ring geometry. Default OFF.
     *
     * See the file header. This is a containment test against the ring the lot was
     * cut from -- collinear within `source_snap_tolerance`, inside the segment's
     * span, same direction -- and NOT a search for a nearby road. It can only
     * adopt a boundary the lot already lies along.
     *
     * Costs O(lot edges * block ring size) per lot, which is why it is opt-in as
     * well as off by default: a block with a thousand ring vertices and a hundred
     * lots pays for it.
     */
    bool infer_sources_from_block_ring = false;

    /// Collinearity and containment tolerance for the inference above, in metres
    double source_snap_tolerance = 0.01;
};

// ============================================================================
// Output
// ============================================================================

/**
 * @brief Every FRONT edge that fronts one particular street half-edge
 *
 * Grouped by the block half-edge, not by the `EdgeId` and not by street name.
 *
 * Not by `EdgeId`, because blocks.hpp is explicit that the same edge can bound one
 * block TWICE, once each way, where a cul-de-sac spur hangs inside it. Merging
 * those two would give one frontage claiming to be on both sides of the same
 * centreline at once, and `side` would be a coin toss.
 *
 * Not by name, because two graph edges of one named street are two different
 * stretches with their own width, class and direction. A caller that wants "all
 * the frontage on Main Street" merges these on `GraphEdge::name` itself; a caller
 * that wants the profile of the road in front of this lot must not have them
 * merged for it.
 */
struct LotFrontage {
    /// The street. Index into `RoadGraph::edges()`.
    EdgeId street = kInvalidId;

    /// Index into `Block::edges` of the half-edge, for tracing back to the block
    uint32_t block_edge = kNoBoundaryEdge;

    /// Which side of `street`'s centreline this lot is on
    StreetSide side = StreetSide::Left;

    /**
     * @brief Ring-segment indices of this frontage, ascending
     *
     * More than one whenever the street bends: a curved graph edge carries several
     * polyline segments into the block ring, and a lot along it inherits all of
     * them as separate edges of one frontage.
     */
    std::vector<uint32_t> edges;

    /// Total length of those segments in metres
    double length = 0.0;

    /**
     * @brief Outward unit normal, length-weighted over `edges`
     *
     * Points FROM the lot TOWARDS the street: the direction a shopfront faces and
     * the direction a setback pushes a building away from.
     */
    glm::dvec2 normal{0.0};

    /**
     * @brief A point on the frontage, for an address marker or a driveway
     *
     * The arc midpoint of the LONGEST CONTIGUOUS run of this frontage, not of the
     * frontage as a whole. A lot with a notch bitten out of its street edge has two
     * separate runs on the same street, and the midpoint of their concatenated
     * length can land in the notch -- off the street entirely. The longest run is
     * always a real, connected stretch of kerb.
     */
    glm::dvec2 midpoint{0.0};

    /// True for exactly one frontage of a lot that has any
    bool primary = false;
};

/**
 * @brief The classification of one lot
 */
struct LotEdgeClassification {
    /**
     * @brief Role per ring segment, parallel to `LotOutline::ring`
     *
     * `roles[i]` covers the segment from `ring[i]` to `ring[(i + 1) % size]`.
     * Empty when `malformed`.
     */
    std::vector<LotEdgeRole> roles;

    /// One entry per street half-edge this lot fronts. Empty when `landlocked`.
    std::vector<LotFrontage> frontages;

    /// Index into `frontages` of the primary, or `kNoFrontage`
    uint32_t primary_frontage = kNoFrontage;

    /**
     * @brief The lot touches no street
     *
     * A real outcome of subdivision, not an error: the middle of a deep block
     * genuinely subdivides into parcels with no frontage. It is surfaced so a
     * caller can merge them, drop them, or drive an access easement through them --
     * anything except build a shopfront on a fence.
     */
    bool landlocked = false;

    /// The input ring was clockwise and the normals were flipped to suit
    bool orientation_corrected = false;

    /**
     * @brief The input could not be classified at all
     *
     * Fewer than three ring points, `edge_sources` a different size from `ring`,
     * or a ring enclosing no area. `roles` and `frontages` are both empty and every
     * other field is meaningless.
     */
    bool malformed = false;

    /// Frontages dropped for being shorter than `LotEdgeConfig::min_frontage_length`
    uint32_t demoted_frontages = 0;

    /// Sources that indexed past the end of `Block::edges`; treated as `kNoBoundaryEdge`
    uint32_t invalid_sources = 0;

    /// Sources recovered by `LotEdgeConfig::infer_sources_from_block_ring`
    uint32_t inferred_sources = 0;

    /// True when a rule may ask for a front
    [[nodiscard]] bool has_front() const { return primary_frontage < frontages.size(); }

    /// The frontage a rule should build against, or nullptr when landlocked
    [[nodiscard]] const LotFrontage* primary_front() const {
        return has_front() ? &frontages[primary_frontage] : nullptr;
    }

    /**
     * @brief Outward normal of the primary front, or (0, 0) when there is none
     *
     * Zero rather than a default direction on purpose. A caller that forgets to
     * check `has_front()` gets a vector that is obviously not a direction, instead
     * of every landlocked lot in the extract pointing its facade due east.
     */
    [[nodiscard]] glm::dvec2 front_normal() const {
        const LotFrontage* front = primary_front();
        return front != nullptr ? front->normal : glm::dvec2{0.0};
    }

    /// Midpoint of the primary front, or (0, 0) when there is none. See front_normal().
    [[nodiscard]] glm::dvec2 front_midpoint() const {
        const LotFrontage* front = primary_front();
        return front != nullptr ? front->midpoint : glm::dvec2{0.0};
    }

    /// How many ring segments carry @p role
    [[nodiscard]] size_t count(LotEdgeRole role) const;
};

/**
 * @brief What the batch classification saw, for logging and tests
 *
 * Carried for the same reason `BlockStats` is: the failure mode is silent. A run
 * that classified every lot's front as its back returns exactly as many lots, with
 * exactly as many edges, as a correct one.
 */
struct LotEdgeStats {
    size_t lots = 0;                    ///< Lots submitted
    size_t classified = 0;              ///< Lots that were not malformed
    size_t malformed = 0;               ///< Lots rejected; see LotEdgeClassification::malformed
    size_t landlocked = 0;              ///< Lots with no surviving frontage
    size_t corner_lots = 0;             ///< Lots with two or more frontages
    size_t orientation_corrected = 0;   ///< Lots whose ring arrived clockwise

    size_t front_edges = 0;             ///< Ring segments classified FRONT
    size_t side_edges = 0;              ///< Ring segments classified SIDE
    size_t back_edges = 0;              ///< Ring segments classified BACK
    size_t interior_edges = 0;          ///< Ring segments classified INTERIOR

    size_t demoted_frontages = 0;       ///< Slivers below min_frontage_length
    size_t invalid_sources = 0;         ///< Sources indexing past Block::edges
    size_t inferred_sources = 0;        ///< Sources recovered from the block ring
};

/**
 * @brief Classifications for a block's lots, plus the landlocked report
 */
struct LotEdgeClassificationSet {
    /// One per input lot, in input order
    std::vector<LotEdgeClassification> lots;

    /**
     * @brief Indices into `lots` of every landlocked lot
     *
     * The REPORT the brief for this stage demands. A flag buried one per lot is
     * easy to never read; a list a caller has to look at is not.
     */
    std::vector<uint32_t> landlocked_lots;

    /// Indices into `lots` of every malformed lot. Same reasoning as above.
    std::vector<uint32_t> malformed_lots;

    LotEdgeStats stats;
};

// ============================================================================
// Classification
// ============================================================================

/**
 * @brief Classify every edge of one lot
 *
 * Reads the lot, the block it was cut from, and the graph the block came from.
 * Writes nothing and touches no global state, so it is deterministic for a given
 * input and safe to run off the main thread or in parallel across lots.
 *
 * @param lot    Lot outline and its per-edge provenance. See LotOutline.
 * @param block  The block the lot was cut from; supplies `edges` and, for the
 *               optional source inference, `ring` and `ring_edges`.
 * @param graph  The graph `block` was extracted from; supplies the road classes
 * @param config Cone half-angles, the frontage floor, and the primary-front rule
 * @return Roles per ring segment, the frontages, and what went wrong
 */
[[nodiscard]] LotEdgeClassification classify_lot_edges(const LotOutline& lot,
                                                       const Block& block,
                                                       const RoadGraph& graph,
                                                       const LotEdgeConfig& config = {});

/**
 * @brief Classify every lot of one block and collect the reports
 *
 * Equivalent to calling classify_lot_edges() per lot, with the statistics and the
 * landlocked and malformed lists filled in. A malformed lot does not abort the
 * batch: one lot with a mismatched `edge_sources` must not cost the other ninety.
 *
 * @param lots   Lots cut from @p block, in any order
 * @param block  The block they were cut from
 * @param graph  The graph @p block was extracted from
 * @param config Shared configuration
 * @return One classification per lot, plus the reports and the counts
 */
[[nodiscard]] LotEdgeClassificationSet classify_block_lots(const std::vector<LotOutline>& lots,
                                                           const Block& block,
                                                           const RoadGraph& graph,
                                                           const LotEdgeConfig& config = {});

// ============================================================================
// Shared Predicates
// ============================================================================

/**
 * @brief Does a way of this class carry frontage?
 *
 * Exposed because the classifier and its callers must not disagree. E3 asking
 * "is this edge a street" with its own switch statement is how a lot ends up with
 * a FRONT that the setback stage then treats as a footway.
 *
 * `RoadType::Unknown` counts as a street. It means the tag mapper did not
 * recognise a `highway=*` value, not that the way is pedestrian, and refusing
 * frontage to an unrecognised road strands every lot along it.
 *
 * @param type              Class of the bounding way
 * @param paths_are_streets Value of LotEdgeConfig::paths_are_streets
 */
[[nodiscard]] bool is_street_class(RoadType type, bool paths_are_streets);

/**
 * @brief Importance of a road class; LOWER is more important
 *
 * Motorway is 0 and Unknown is last. Used only to break primary-front ties and to
 * implement PrimaryFrontRule::HighestRoadClass, so the absolute values mean
 * nothing and only the order matters.
 *
 * @param type Class of the street
 */
[[nodiscard]] int street_class_rank(RoadType type);

} // namespace stratum::osm::road
