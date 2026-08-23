/**
 * @file sidewalk_dedup.hpp
 * @brief Stop a doubly-mapped sidewalk from being built twice
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * OSM maps sidewalks two ways at once, and real extracts frequently carry BOTH
 * for the same stretch of street:
 *
 * 1. As a `sidewalk=left|right|both` tag on the carriageway way. build_profile()
 *    turns that into a Sidewalk strip on the corresponding side of the profile,
 *    which the corridor extruder sweeps along with everything else.
 * 2. As a separate `highway=footway` + `footway=sidewalk` way running parallel to
 *    the carriageway a few metres out. That is an ordinary road in the graph and
 *    gets its own edge, its own profile, and its own ribbon.
 *
 * With both present the same footway is built twice, a couple of metres apart,
 * at slightly different elevations, z-fighting where they overlap. This file
 * finds those pairs and reports which SIDE of which carriageway must stop
 * synthesising its own.
 *
 * ### Which one wins, and why
 *
 * The SEPARATELY MAPPED way wins and the tag-derived strip is suppressed. It is
 * the more specific of the two: it has surveyed geometry, so it follows the real
 * kerb line around a build-out or a bus stop, where the strip only ever parallels
 * the carriageway at a fixed offset. Suppressing the surveyed way instead would
 * throw away information that cannot be recovered from a tag.
 *
 * ### Ordering: this runs BETWEEN two profile passes
 *
 * Suppression changes the cross-section, so it cannot run after the profiles are
 * final, and it needs the profiles to measure against, so it cannot run before
 * them either. The sequence is:
 *
 * @code
 *     // 1. Provisional profiles, sidewalks synthesised as the tags ask.
 *     profiles[i] = build_profile(edge, profile_cfg, tags);
 *
 *     // 2. Find the doubly-mapped sides.
 *     DedupResult dd = dedup_sidewalks(graph, centerlines, profiles, dedup_cfg);
 *
 *     // 3. Rebuild only the affected profiles, with the tag masked off.
 *     GraphEdge masked = edge;
 *     masked.sidewalk = mask_side(edge.sidewalk, dd.suppress_side[i]);
 *     profiles[i] = build_profile(masked, profile_cfg, tags);
 * @endcode
 *
 * Step 3 is the caller's, not this file's: dedup_sidewalks() is a pure query and
 * writes to nothing. build_profile() also takes the suppression mask directly as
 * its fourth argument, which does the same thing without copying the edge and its
 * two vectors; mask_side() is the spelling for a caller that already holds a
 * GraphEdge it wants to alter.
 *
 * The provisional profile in step 1 is what step 2 measures, and its Sidewalk
 * strips are exactly what makes the offset test meaningful -- the question is
 * whether the surveyed footway lands where the synthesised one would be.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API.
 */

#pragma once

#include "osm/road/centerline.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/road/road_profile.hpp"
#include "osm/types.hpp"

#include <cstddef>
#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Thresholds of the parallel-and-nearby test
 */
struct DedupConfig {
    /**
     * @brief A footway further out than this is not this road's sidewalk, metres
     *
     * Measured laterally from the carriageway CENTRELINE, so it must cover half
     * the carriageway plus the verge plus the footway itself. Twelve metres
     * clears a dual-carriageway half-width and still excludes a path through the
     * park on the far side of the wall.
     */
    double max_offset = 12.0;

    /**
     * @brief Fraction of the footway's own length that must run parallel
     *
     * A footway is a match only when at least this share of its stations pass
     * both the offset and the bearing test against SOME carriageway edge. Below
     * it the footway merely touches the road -- a crossing, a driveway, a path
     * leaving a junction -- and suppressing a sidewalk for it would delete a real
     * footway from a street that has no other.
     *
     * The share is aggregated over the footway as a whole, not measured against
     * one carriageway edge at a time. Carriageways are split at their own
     * junctions, so a footway running the length of a block is routinely shared
     * between two or three edges of the same street; a per-edge threshold would
     * clear on none of them while the footway is plainly that street's sidewalk.
     * A crossing or a driveway fails the aggregate just as surely as it fails a
     * per-edge test, which is the case the threshold exists for.
     */
    double min_parallel_fraction = 0.6;

    /**
     * @brief Fraction of the CARRIAGEWAY edge the footway must run alongside
     *
     * The other half of the question, and the one min_parallel_fraction cannot
     * answer. That gate accepts the FOOTWAY; this one decides how much of a
     * carriageway edge the footway is entitled to replace.
     *
     * Suppression is per edge and per side -- RoadProfile is one cross-section
     * for the whole edge -- so a footway alongside part of an edge would
     * otherwise delete the synthesised sidewalk over ALL of it. A 120 m surveyed
     * path beside a 200 m edge leaves 80 m of street with paving and kerb from
     * neither source, and a 10 m path clears the aggregate gate trivially while
     * emptying an edge of any length.
     *
     * The measure is the arc length of the edge between the first and last
     * station the footway claimed, over the edge's own length. Below this share
     * the synthesised sidewalk stays and the surveyed footway is simply drawn
     * beside it, which is a visible duplicate over a short stretch rather than a
     * missing footway over a long one.
     */
    double min_edge_coverage = 0.5;

    /**
     * @brief Maximum absolute bearing difference to count as parallel, RADIANS
     *
     * Compared MODULO PI, not modulo 2*pi: a sidewalk digitised in the opposite
     * direction from its carriageway is still that carriageway's sidewalk, and
     * OSM way direction is arbitrary. 0.35 rad is about 20 degrees.
     */
    double max_bearing_delta = 0.35;

    /**
     * @brief Master switch
     *
     * When false, dedup_sidewalks() returns a result whose suppress_side is sized
     * to the graph and filled with SideFlags::None, and whose counts are zero. A
     * caller therefore needs no branch: applying an all-None result is a no-op
     * and reproduces the previous phase's output exactly.
     */
    bool enabled = true;
};

// ============================================================================
// Result
// ============================================================================

/**
 * @brief Which sides of which edges must not synthesise a sidewalk
 */
struct DedupResult {
    /**
     * @brief Per edge, the sides that already have a separately-mapped sidewalk
     *
     * Indexed by EdgeId and always the same size as `graph.edges()`, including
     * when DedupConfig::enabled is false, so a caller may index it unconditionally.
     *
     * Left and right are relative to the EDGE's direction of travel, matching
     * RoadProfile and GraphEdge::sidewalk. The value names the sides to SUPPRESS:
     *
     * - `SideFlags::None`  -- suppress nothing; build the profile as tagged
     * - `SideFlags::Left`  -- do not synthesise a left sidewalk
     * - `SideFlags::Right` -- do not synthesise a right sidewalk
     * - `SideFlags::Both`  -- synthesise neither
     *
     * `SideFlags::Unknown` is NEVER produced. It means "no tag, infer a default"
     * everywhere else in the codebase, which is the opposite of what a suppression
     * mask needs to mean, so a consumer that sees it should treat it as a bug
     * rather than as None.
     *
     * A matched FOOTWAY edge is not marked here and is not suppressed. It is real
     * surveyed geometry and is extruded normally; only the carriageway's
     * synthesised strip goes away.
     */
    std::vector<SideFlags> suppress_side;

    /// Edges with at least one suppressed side
    size_t suppressed_edges = 0;

    /**
     * @brief Distinct footway edges that matched a carriageway
     *
     * Not the same as suppressed_edges, in either direction. One long carriageway
     * edge may be matched by several footway edges split at their own junctions,
     * so this can be the larger; and a footway that matches a carriageway whose
     * profile has no Sidewalk strip on the matched side counts here while
     * suppressing nothing, so this can be the larger for that reason too. A
     * footway that matched several edges of one street suppresses each of them
     * and counts once.
     */
    size_t matched_footways = 0;
};

// ============================================================================
// Query
// ============================================================================

/**
 * @brief Find carriageway sides whose sidewalk is already mapped separately
 *
 * ### Candidates
 *
 * A FOOTWAY candidate is any edge whose RoadType is Footway, Cycleway, or Path.
 * GraphEdge carries no tag map, so `footway=sidewalk` cannot be read directly
 * here and the type is the whole filter. That is coarser than the tag and it is
 * deliberately compensated for downstream rather than upstream: a
 * `footway=crossing` runs perpendicular to its carriageway and fails the bearing
 * test; a park path is further out than DedupConfig::max_offset or fails the
 * parallel fraction. What survives both tests is a footway running alongside a
 * road for most of its length, which is a sidewalk whatever its tags say.
 *
 * A CARRIAGEWAY candidate is any edge that is not a footway candidate and whose
 * provisional profile holds at least one Sidewalk strip. An edge with no
 * synthesised sidewalk has nothing to suppress and is skipped, which is also why
 * `sidewalk=no` costs nothing here: build_profile() emitted no strip, so the
 * edge never enters the search.
 *
 * ### The test
 *
 * For each footway candidate, every one of its stations is tested against the
 * nearby carriageway candidates -- sampled along its length, never compared
 * endpoint to endpoint, because a footway CROSSING a road at its midpoint has
 * both endpoints far from that road and would pass any proximity test on them.
 *
 * A station matches a carriageway band when its lateral offset from that band is
 * within DedupConfig::max_offset AND the two tangents agree within
 * DedupConfig::max_bearing_delta modulo pi. Of the bands a station matches, the
 * NEAREST one claims it, so a footway between two parallel streets is the
 * sidewalk of the street it hugs rather than of both.
 *
 * The footway is a sidewalk when the fraction of its stations claimed by any
 * carriageway reaches DedupConfig::min_parallel_fraction. Each carriageway edge
 * the footway then covers for at least DedupConfig::min_edge_coverage of its
 * length is suppressed on ONE side: the sign of the median lateral offset over
 * the stations it claimed -- positive is the carriageway's left, matching the
 * lateral sign convention in road_profile.hpp.
 *
 * Two gates, because they answer two questions. The parallel fraction accepts or
 * rejects the FOOTWAY; the edge coverage decides how much of a carriageway that
 * accepted footway is entitled to replace. Suppression is per edge and per side,
 * so without the second gate a footway alongside a third of an edge deletes the
 * synthesised sidewalk along all of it.
 *
 * The median is used rather than the mean because a footway that swings around a
 * corner at one end, or steps out around a bus stop, would otherwise average its
 * way onto the wrong side.
 *
 * Two further filters, neither of which changes the shape of the test:
 *
 * - Arms of differing GraphEdge::layer never match. A footbridge over a road is
 *   metres from it in plan and would otherwise read as its sidewalk.
 * - A side is recorded only when the provisional profile actually carries a
 *   Sidewalk strip there. A street with a tag on the right and a surveyed footway
 *   on the left has nothing to suppress, and reporting a side anyway would make
 *   DedupResult::suppressed_edges overstate what changed.
 *
 * Only the carriageway is suppressed, and only on the matched side. A street
 * with a surveyed footway on one side and a tag on both keeps its synthesised
 * strip on the other side, which is the correct answer and the common one.
 *
 * ### Cost
 *
 * The carriageway bands -- consecutive station pairs -- are indexed in a uniform
 * grid, so the search is proportional to the footway stations rather than to the
 * product of the two edge counts. A city extract carries tens of thousands of
 * footways and the all-pairs form is not an option.
 *
 * @param graph       Built road graph
 * @param centerlines Parallel to graph.edges(); an invalid centerline's edge is skipped
 * @param profiles    Parallel to graph.edges(); the PROVISIONAL profiles, built
 *                    before any suppression. See the ordering note at the top.
 * @param cfg         Thresholds and the master switch
 * @return The suppression mask and its counts. suppress_side is always sized to
 *         graph.edges().
 */
[[nodiscard]] DedupResult dedup_sidewalks(const RoadGraph& graph,
                                          const std::vector<Centerline>& centerlines,
                                          const std::vector<RoadProfile>& profiles,
                                          const DedupConfig& cfg);

/**
 * @brief Remove the suppressed sides from a tag-derived SideFlags
 *
 * The step-3 helper from the ordering example at the top of this file. Pure set
 * subtraction on the two side bits, with the Unknown case handled explicitly
 * rather than by falling through:
 *
 * | @p tagged | @p suppress | result  |
 * |-----------|-------------|---------|
 * | Both      | Left        | Right   |
 * | Both      | Both        | None    |
 * | Left      | Left        | None    |
 * | Left      | Right       | Left    |
 * | Unknown   | None        | Unknown |
 * | Unknown   | Left        | Right   |
 * | Unknown   | Both        | None    |
 * | anything  | None        | tagged  |
 *
 * Unknown means "no tag, a class default may be inferred". Subtracting a side
 * from it must therefore RESOLVE it -- to the remaining side, or to None -- and
 * not leave it Unknown, or build_profile() would infer the default back and
 * re-synthesise exactly the sidewalk that was just suppressed.
 *
 * @param tagged   The edge's own GraphEdge::sidewalk
 * @param suppress The mask from DedupResult::suppress_side
 * @return The sides that may still be synthesised
 */
[[nodiscard]] SideFlags mask_side(SideFlags tagged, SideFlags suppress);

} // namespace stratum::osm::road
