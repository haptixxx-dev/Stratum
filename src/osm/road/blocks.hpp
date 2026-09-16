// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file blocks.hpp
 * @brief City blocks: the bounded faces of the street graph
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### Why this exists
 *
 * A block is the land a ring of streets encloses. Nothing downstream of the road
 * network can be placed without one: lots are cut out of blocks (C2), rules are
 * targeted at lots (C3, D), and facades are grown on what the rules produce.
 * Every one of those stages needs a closed polygon of land with a known street
 * boundary, and the street graph is the only thing in the pipeline that knows
 * where the land is.
 *
 * Blocks are NOT derived from `landuse=*` areas or from building footprints.
 * Those are mapped where a surveyor happened to draw them, they overlap, they
 * have holes, and most of a city has none of them at all. The street graph is
 * complete by construction, so the faces of the street graph are complete too.
 *
 * ### The method: half-edge face traversal
 *
 * Every RoadGraph edge is two directed half-edges, one per direction of travel.
 * The faces of the planar graph are the orbits of a single permutation over
 * those half-edges:
 *
 *     next(h) = the arm CLOCKWISE of h's own arm at the node h arrives at
 *
 * In full: arrive at node `v` along half-edge `h`; find `h`'s own arm at `v`
 * (the one pointing back the way you came); step to the arm before it in the
 * rotation at `v`, which runs anticlockwise, so the previous arm is the next one
 * clockwise; leave along that arm. Repeat until you arrive back at the half-edge
 * you started on.
 *
 * **The convention is clockwise-next, and it is what makes the orientation
 * predictable.** Bounded faces come out ANTICLOCKWISE (positive signed area);
 * the unbounded face of each connected component comes out CLOCKWISE (negative
 * signed area). Anticlockwise-next would give the mirror of both, and then the
 * outer-face test below would have the opposite sign. Pick the other convention
 * and every sign in this file and in C2 flips with it.
 *
 * The rotation at a node is `GraphNode::arms` sorted ascending by bearing, with
 * one correction this file makes for itself: RoadGraph sorts arms by bearing
 * alone, with a non-stable sort, so two arms that leave a node on the SAME
 * bearing come out in an arbitrary relative order. An arbitrary order is not an
 * embedding -- the orbit decomposition of a non-embedding loses whole faces
 * silently -- so ties are re-broken here by where the two arms first diverge.
 * See BlockStats::duplicate_edges for the case where they never diverge at all.
 *
 * Two properties follow, and both are load-bearing:
 *
 *   - **Every directed half-edge belongs to exactly one face.** `next` is a
 *     bijection -- reversing a half-edge is a bijection, and rotating within a
 *     node's arm list is a cyclic shift -- so the half-edges decompose into
 *     disjoint cycles. The traversal cannot loop forever and cannot visit a
 *     half-edge twice. The evidence that the decomposition is the RIGHT one is
 *     Euler's formula: see BlockStats::expected_faces, which a test asserts
 *     against BlockStats::faces. `half_edges_walked` cannot carry that evidence,
 *     because the seed loop visits every half-edge of every usable edge whatever
 *     `next` does.
 *   - **A block lies to the LEFT of each of its bounding half-edges.** That is
 *     what anticlockwise means, and it is the whole reason C4 can classify a lot
 *     frontage: given a bounding half-edge, the block is the left side of that
 *     street and the opposite side belongs to some other block.
 *
 * ### The traps, all of which are in real OSM data
 *
 *   - **The outer face.** One orbit per connected component walks the unbounded
 *     face. It is discarded by the SIGN of its signed area, never by its size:
 *     an extract with two disconnected street networks has two outer faces, and
 *     the small network's outer face is smaller than the large network's blocks,
 *     so "discard the biggest face" discards the wrong thing.
 *   - **Cut edges.** An edge whose two half-edges land in the SAME face is a cut
 *     edge: the land is continuous across it, so it bounds nothing. A cul-de-sac
 *     spur is the familiar one, walked out and back inside the face it hangs in.
 *     The general one is any street that joins a sub-network to the rest at a
 *     single node -- a turning bulb mapped as a closed way, a housing estate off
 *     one access road -- and there the two traversals are NOT adjacent in the
 *     walk, because the whole sub-network is walked between them. Cut edges are
 *     removed from the boundary TOPOLOGICALLY, by that same-face test, and never
 *     by looking for a point that repeats. Every half-edge of them stays in
 *     `Block::edges`, in both directions, because a cul-de-sac still fronts the
 *     block it dead-ends into.
 *   - **Cut vertices.** The same sub-network can hang off a single NODE with no
 *     street between -- a turning bulb or an estate loop whose closed way shares
 *     one node with the road it serves. There is no cut edge to find, and the face
 *     walks out round the loop and back through that one node, so its boundary
 *     pinches to a point there. Area and perimeter survive a pinch; the shape does
 *     not, so the curve is split at the repeated vertex into a boundary and a
 *     `Block::holes` entry.
 *   - **Multiple components.** Handled for free: the traversal enumerates orbits,
 *     not components, and each component contributes its own outer face.
 *   - **Bridges and tunnels.** NO intersection is ever invented from geometry.
 *     Two roads that cross on screen and share no OSM node are not a junction and
 *     do not divide a block -- which is also physically true, since the land under
 *     a flyover is continuous. The cost is that the planar-face assumption is only
 *     as planar as the data: a grade-separated edge that DOES bound a face can
 *     make that face's ring self-intersect, and can cost the traversal a face
 *     outright. Such blocks are flagged rather than silently trusted (see
 *     Block::has_grade_separated_edge), and the count that says a face went
 *     missing is BlockStats::expected_faces against BlockStats::faces.
 *   - **Duplicate ways.** Two ways mapped between the same pair of nodes with the
 *     same geometry are a mapping artefact, not topology. They give their arms
 *     identical bearings at both ends, which no tie-break can turn into an
 *     embedding, so the later one is dropped before the traversal and counted in
 *     BlockStats::duplicate_edges. Two ways that merely run CLOSE together still
 *     enclose a real sliver, and that is rejected by BlockConfig::min_area.
 *   - **Degenerate faces.** A component that is a tree walks one face that
 *     encloses nothing, and a cycle whose nodes are collinear walks a face of
 *     exactly zero area. They are counted apart, as BlockStats::tree_faces and
 *     BlockStats::zero_area_faces, and apart again from
 *     BlockStats::malformed_faces, which means the graph itself was inconsistent.
 *
 * ### What this does NOT do
 *
 *   - **No lot subdivision.** That is C2 and it reads this output.
 *   - **No island holes.** A disconnected street network sitting entirely inside
 *     another network's face is not subtracted from it, because a face traversal
 *     has no way to learn about a component it never touches. Islands are rare
 *     enough, and expensive enough to detect (a point-in-polygon test of every
 *     component against every face), that C2 gets to decide whether it cares.
 *     `Block::holes` is only ever the holes the traversal WALKED -- a loop the
 *     face reaches across a cut edge -- which it knows about exactly.
 *   - **No merging across a tunnel.** A tunnel whose two portals both attach to
 *     the surface network splits the land above it into two faces, which is wrong
 *     at ground level. `has_grade_separated_edge` marks both halves so a caller
 *     can merge or drop them.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API. It is pure topology and 2D geometry over a built RoadGraph.
 */

#pragma once

#include "osm/road/road_graph.hpp"

#include <glm/glm.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Type Aliases
// ============================================================================

/// Index into BlockExtraction::blocks
using BlockId = uint32_t;

// ============================================================================
// Block
// ============================================================================

/**
 * @brief One directed use of a graph edge along a block boundary
 *
 * Direction matters and is not decoration. The block is always on the LEFT of the
 * half-edge, so `forward` is what tells C4 whether the block fronts the left or
 * the right side of that street's centreline.
 *
 * The same EdgeId may appear TWICE in one block, once each way, when the edge has
 * the same face on both sides: a cul-de-sac spur hanging inside the block, the
 * access road of an estate inside it, or any other cut edge of the graph. That is
 * not a duplicate to be filtered out -- both sides of that street really do front
 * this block.
 */
struct BlockEdge {
    /// Edge in the source RoadGraph
    EdgeId edge = kInvalidId;

    /// True when the boundary runs along the edge from GraphEdge::from to ::to
    bool forward = true;
};

/**
 * @brief A parcel of land enclosed by streets
 *
 * Units are local metres throughout, the same frame as GraphEdge::polyline, so a
 * block can be compared against road geometry without a transform.
 */
struct Block {
    /// Index of this block in BlockExtraction::blocks. Stable within one extraction only.
    BlockId id = 0;

    /**
     * @brief Outer boundary as a closed anticlockwise ring, first point NOT repeated
     *
     * Same convention as Corridor::outline, so the two can be handed to the same
     * clipping and triangulation code without a special case.
     *
     * Carries every polyline vertex of every bounding edge, not just the graph
     * nodes, so a curved street stays curved. Cut edges never reach it and a curve
     * that pinches at a repeated vertex is split before it does, so the ring has at
     * least three points, repeats no point, and encloses real area.
     */
    std::vector<glm::dvec2> ring;

    /**
     * @brief Which half-edge carries each ring segment; parallel to `ring`
     *
     * `ring_edges[i]` indexes `edges`, and names the half-edge whose polyline
     * contains the segment from `ring[i]` to `ring[(i + 1) % ring.size()]`.
     *
     * This is the reason C4 does not have to do a nearest-segment search from a
     * lot boundary back to the street network. Without it, a lot frontage would be
     * attributed to a street by proximity, which is the same class of mistake as
     * finding junctions by endpoint proximity, and it fails in the same place --
     * where two streets run within a few metres of each other.
     */
    std::vector<uint32_t> ring_edges;

    /**
     * @brief Land the ring encloses but the block does not own, each ring CLOCKWISE
     *
     * A hole is a loop of street the face reaches and walks all the way round: the
     * turning bulb of a cul-de-sac, the loop road of an estate. It gets there
     * across a cut edge (the access road) or through a single shared node, and the
     * land inside it is its own block, so this block does not own it and `area`
     * already has it subtracted.
     *
     * Each hole is closed, its first point is not repeated, and it runs CLOCKWISE
     * -- the opposite of `ring` -- which is the orientation Clipper2 and earcut
     * both take to mean a hole, so C2 can hand ring and holes straight to either.
     *
     * Empty for the overwhelming majority of blocks. It is NOT the islands of
     * disconnected networks; see "What this does NOT do" in the file header.
     */
    std::vector<std::vector<glm::dvec2>> holes;

    /// Owning half-edge per hole vertex; `hole_edges[h]` is parallel to `holes[h]`,
    /// and indexes `edges` exactly as `ring_edges` does.
    std::vector<std::vector<uint32_t>> hole_edges;

    /**
     * @brief Every half-edge walked, in traversal order
     *
     * Includes the cut-edge half-edges that never reach `ring`, so this is a
     * complete list of the streets that front the block and `edges.size()` is
     * generally larger than the number of distinct ring segments.
     */
    std::vector<BlockEdge> edges;

    /**
     * @brief Enclosed area in square metres, holes already subtracted
     *
     * Always strictly positive; see BlockConfig::min_area.
     */
    double area = 0.0;

    /**
     * @brief Length of `ring` plus every hole, in metres
     *
     * The length of the land boundary, not the distance walked: a cut edge is
     * walked twice and contributes nothing here, because no land stops at it.
     * Carried because it is the cheap half of a compactness test
     * (4*pi*area / perimeter^2), which is the only way to tell a genuine long thin
     * block -- the median strip of a dual carriageway -- from a city block, and
     * BlockConfig::min_area cannot do it. Recomputing it downstream means walking
     * every ring again.
     */
    double perimeter = 0.0;

    /**
     * @brief True when any bounding edge is a bridge, a tunnel, or off layer 0
     *
     * A warning flag, not an error. The face is only planar where the data is:
     * such a block may have a self-intersecting ring, or may be one half of a
     * surface block that a tunnel beneath it wrongly divided. C2 can subdivide it
     * anyway, merge it, or skip it -- but it should not be trusted blindly.
     */
    bool has_grade_separated_edge = false;
};

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Tunables of the face traversal
 */
struct BlockConfig {
    /**
     * @brief Smallest face kept, in square metres. Default is a 10 m square.
     *
     * A degeneracy filter and nothing more. It exists for the sliver two ways
     * enclose when they run between the same pair of nodes a metre or two apart --
     * a divided way, a mapping duplicate, a service road drawn twice -- and for any
     * other face too small to be land.
     *
     * It is deliberately far below the size of a real block. A terrace block is
     * 800 m^2 and there is no floor that kills a dual carriageway's median strip
     * without also killing that. The median comes out as a genuine block, long and
     * thin, and Block::perimeter is what a caller uses to recognise it.
     */
    double min_area = 100.0;

    /**
     * @brief Let footways, cycleways and paths bound blocks. Default OFF.
     *
     * Off because a mapped pavement runs a few metres from the carriageway it
     * serves, so including it turns every street into a long sliver face between
     * kerb and pavement, and turns every marked crossing into a node that chops
     * that sliver up further. In a city extract pedestrian ways outnumber roads,
     * so the output would be mostly rubbish faces that clear min_area honestly.
     *
     * Turning it on is right for a pedestrianised centre where the footways ARE
     * the street network.
     */
    bool include_paths = false;

    /**
     * @brief Let bridges, tunnels and other off-layer edges bound blocks. Default ON.
     *
     * On because the topology is what it is: a bridge over a river is the boundary
     * of the land either side of it, and dropping it merges two blocks across
     * water. Turn it off when the extract is dominated by grade separation and a
     * clean surface-only partition matters more than the few blocks that dropping
     * these edges merges.
     */
    bool include_grade_separated = true;

    /**
     * @brief Drop an edge whose geometry duplicates another between the same nodes
     *
     * Default ON, and off is for diagnostics only. Two coincident ways have equal
     * arm bearings at both ends, and equal bearings are not an embedding: the orbit
     * decomposition collapses and the surrounding block is the thing that vanishes,
     * not the sliver. Turn it off to see how many edges that is; see
     * BlockStats::duplicate_edges.
     */
    bool drop_duplicate_edges = true;
};

// ============================================================================
// Output
// ============================================================================

/**
 * @brief What the traversal saw, for logging and tests
 *
 * Carried rather than logged and forgotten because the failure mode of a face
 * traversal is silent: an extraction that returns four blocks where there should
 * be five looks exactly like a correct one. These counts are how a test proves the
 * traversal covered the graph, and how a caller notices that half the extract was
 * filtered away.
 *
 * The three ways a face can come to nothing are counted APART. Merging them would
 * put the one count that means "the graph is inconsistent" in the same bucket as
 * the most common benign case -- a dead-end road that encloses nothing -- and then
 * nothing in the output says the difference.
 */
struct BlockStats {
    size_t edges_considered = 0;    ///< Edges in the source graph
    size_t edges_used = 0;          ///< Edges that passed the class, layer and duplicate filters
    size_t duplicate_edges = 0;     ///< Edges dropped as geometric duplicates; see BlockConfig
    size_t half_edges_walked = 0;   ///< Half-edges seeded or stepped on. Equals 2 * edges_used.
    size_t faces = 0;               ///< Orbits found, outer and degenerate faces included

    /**
     * @brief Faces a planar embedding of the usable subgraph must have
     *
     * Euler's formula: edges_used - nodes_used + 2 * components, over the usable
     * edges and the nodes that carry at least one usable arm.
     *
     * This is the only cheap signal that the embedding was NOT planar. Two ways
     * that cross where the data shares no node are not a junction, so the traversal
     * is right to walk straight past them -- but the orbits it then finds can be
     * fewer than the faces the drawing has, and the missing face is a block that
     * silently does not exist. `faces < expected_faces` says so; nothing else does.
     */
    size_t expected_faces = 0;

    size_t outer_faces = 0;         ///< Faces discarded for having negative signed area
    size_t tree_faces = 0;          ///< Faces with no boundary left once cut edges are removed
    size_t zero_area_faces = 0;     ///< Faces with a boundary but no interior, e.g. a collinear cycle
    size_t malformed_faces = 0;     ///< Faces abandoned: the graph or the embedding is inconsistent
    size_t rejected_small = 0;      ///< Bounded faces discarded by BlockConfig::min_area
    size_t blocks = 0;              ///< Faces kept. Equals BlockExtraction::blocks.size().
};

/**
 * @brief Blocks plus the evidence that they are all of them
 */
struct BlockExtraction {
    /// Blocks in traversal order. `blocks[i].id == i`.
    std::vector<Block> blocks;

    /// Counts covering every face, including the ones that became no block
    BlockStats stats;
};

// ============================================================================
// Extraction
// ============================================================================

/**
 * @brief Extract the bounded faces of @p graph as blocks
 *
 * Reads the graph and nothing else -- no tags, no coordinates, no reprojection --
 * so it is deterministic for a given graph and safe to run off the main thread.
 *
 * Complexity is O(E * d) where d is the mean node degree, because each of the 2E
 * half-edge steps scans the arm list of one node. Node degree is bounded by the
 * data (a node with more than a handful of arms is a mapping error), so this is
 * linear in practice.
 *
 * @param graph  A built RoadGraph. An empty graph yields an empty extraction.
 * @param config Class filters and the area floor
 * @return Blocks, anticlockwise and strictly positive in area, plus statistics
 */
[[nodiscard]] BlockExtraction extract_blocks(const RoadGraph& graph,
                                             const BlockConfig& config = {});

// ============================================================================
// Ring Helpers
// ============================================================================

/**
 * @brief Shoelace signed area of a closed ring whose first point is not repeated
 *
 * Positive anticlockwise, negative clockwise. Zero for fewer than three points.
 *
 * Exposed because the sign IS the outer-face test and the orientation contract of
 * Block::ring, and a caller that builds rings of its own (C2 cutting lots out of a
 * block) needs to agree with this file about which way round positive is.
 *
 * @param ring Ring in local metres, first point not repeated
 * @return Signed area in square metres
 */
[[nodiscard]] double signed_ring_area(const std::vector<glm::dvec2>& ring);

/**
 * @brief True when the edge is a bridge, a tunnel, or sits off layer 0
 *
 * The single definition of "grade separated" used by both BlockConfig::
 * include_grade_separated and Block::has_grade_separated_edge, so the flag can
 * never disagree with the filter.
 *
 * @param edge Edge to classify
 */
[[nodiscard]] bool is_grade_separated(const GraphEdge& edge);

} // namespace stratum::osm::road
