/**
 * @file terrain_carve.hpp
 * @brief Writes solved road corridors back into a terrain heightmap
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * This is the second half of P3. RoadElevationSolver decides where the road
 * surface is; this decides what the terrain does about it.
 *
 * The pipeline order the plan asks for:
 *
 * @code
 *     TerrainGenerator -> Heightmap
 *                           |
 *                           v
 *                   RoadElevation (sample + grade-limit)   [global, once]
 *                           |
 *                           v
 *                   TerrainCarve (write back into Heightmap)  [per chunk]
 *                           |
 *                           v
 *                   TerrainMeshBuilder -> Mesh
 * @endcode
 *
 * Elevation is solved GLOBALLY and BEFORE any chunk is carved, because a road
 * crossing a chunk boundary must not change height depending on which chunk was
 * generated first. Carving is then a purely local, per-chunk operation, which is
 * what lets terrain stay chunked and generated on demand.
 *
 * ### Layering
 *
 * CarveRibbon, CarveDisc and CarvePortal are NOT declared here. They are declared
 * by the module that produces them, `osm/road/carve_request.hpp`, and aliased
 * into this namespace below. Nothing under `src/osm/road/` may include anything under
 * `src/procgen/`, so the dependency runs procgen -> osm, the direction that
 * already exists everywhere else in this codebase. See carve_request.hpp for the
 * full justification.
 *
 * ### Coordinates
 *
 * The carve payload is in 2D LOCAL metres, and so is a Heightmap: its first axis
 * is local x and its SECOND AXIS IS LOCAL Y, not render-space Z. There is no
 * sign flip between the two, and carve_terrain() applies none.
 *
 * The negation everyone reaches for belongs one step later, in render space, and
 * both pipelines apply it independently:
 *
 * @code
 *     terrain: heightmap cell (ix, iz) -> vec3(world_x, h, -world_z)
 *     roads:   local (x, y_2d)         -> vec3(x, h, -y_2d)
 * @endcode
 *
 * so a heightmap row lands on a road exactly when `world_z == y_2d`. A heightmap
 * sample at (X, Z) is therefore tested against the 2D point (X, Z). Negating it
 * mirrors the whole network about y = 0 -- zero error at the origin, growing with
 * |y|, and invisible on symmetric test data.
 *
 * Heights need no conversion: the payload already carries world Y.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API.
 */

#pragma once

#include "osm/road/carve_request.hpp"
#include "procgen/terrain_generator.hpp"

#include <glm/glm.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace stratum::procgen {

// ============================================================================
// Payload
// ============================================================================

/**
 * @brief One road corridor to carve into the terrain
 *
 * Alias of osm::road::CarveRibbon. Declared in the producing module so no
 * conversion step exists between the two pipelines; see the file comment.
 */
using CarveRibbon = osm::road::CarveRibbon;

/**
 * @brief Junction footprint
 *
 * Alias of osm::road::CarveDisc. P3 carved these as flat discs. P4 fills
 * CarveDisc::outline with the real fillet-and-curb-ring boundary, and the carve
 * uses the polygon whenever one is present, falling back to the disc only for a
 * junction whose solve was degenerate. See the polygon-carve note on
 * carve_terrain().
 */
using CarveDisc = osm::road::CarveDisc;

/**
 * @brief One tunnel portal mouth to keep open
 *
 * Alias of osm::road::TunnelPortalFootprint. The third and last carve primitive,
 * and the only one that is a CLAMP rather than a target.
 *
 * A ribbon and a disc both say "the ground here is the road". A portal says the
 * opposite: the ground here is whatever it was, EXCEPT that it may not stand
 * above the crown of the arch within `depth` metres of the mouth. Setting the
 * terrain to `crown_height` instead of clamping it would dig a trench in front
 * of every portal built on level ground, where the hillside was never in the way.
 *
 * That difference is why a portal does not compete with the other two for a
 * cell. See the composition note on carve_terrain().
 */
using CarvePortal = osm::road::TunnelPortalFootprint;

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Tunables of the carve
 */
struct CarveConfig {
    /**
     * @brief Width of the cut/fill embankment band, metres
     *
     * Measured outward from the corridor footprint. Inside the footprint the
     * terrain is set to the road surface; across this band it blends back to the
     * natural surface, which is what produces embankments instead of a cliff at
     * the kerb.
     */
    float falloff_metres = 10.0f;

    /**
     * @brief Steepest embankment permitted, rise over run
     *
     * Where the height difference between road and natural terrain would need a
     * steeper slope than this to close within falloff_metres, the band is
     * widened locally instead of steepened, so a road cut into a hillside gets a
     * proportionally larger cutting rather than a vertical wall.
     *
     * The widening is capped at four times falloff_metres. The cap is what gives
     * the carve a bounded reach, which the spatial index relies on: every item is
     * binned by a box inflated by exactly that capped reach, so a single-cell
     * query is complete rather than merely likely. A height difference too large
     * to close even then produces a band at the cap and an embankment steeper
     * than this value, which is the correct failure -- visibly steep terrain
     * rather than terrain carved by whichever roads happened to be binned nearby.
     *
     * Zero or negative disables the widening; the band is then always
     * falloff_metres wide.
     */
    float max_embankment_slope = 0.6f;

    /// Master switch. When false carve_terrain() returns immediately with zeroed stats.
    bool enabled = true;
};

// ============================================================================
// Input
// ============================================================================

/**
 * @brief Every corridor, junction and portal mouth to carve, plus an index over them
 *
 * Built once after the elevation solve and then shared, by const reference, with
 * every chunk carve. A city extract carries tens of thousands of ribbons and a
 * chunk touches a handful, so the index is what keeps the per-chunk cost
 * proportional to the roads in the chunk rather than to the roads in the world.
 */
struct CarveInput {
    /// Corridor footprints, one per emitted RoadPiece
    std::vector<CarveRibbon> ribbons;

    /// Junction footprints, one per graph node of degree 3 or more, plus roundabouts
    std::vector<CarveDisc> discs;

    /**
     * @brief Tunnel portal mouths, one per portal the tunnel builder emitted
     *
     * Empty for a network built with no terrain sampler or with structures off,
     * which are the same conditions under which no portal geometry exists. The
     * carve never opens a mouth where no headwall stands.
     */
    std::vector<CarvePortal> portals;

    /// Tunables applied to every ribbon, disc and portal
    CarveConfig config;

    /**
     * @brief Axis-aligned bounds of one item, in 2D local metres
     *
     * Already inflated by the item's full carve REACH: its half width (times the
     * largest miter scale on its centerline) or its radius, plus the widest blend
     * band CarveConfig::max_embankment_slope may open up.
     * That is four times CarveConfig::falloff_metres, not one -- the slope limit
     * widens the band locally, and a box inflated by the nominal falloff alone
     * would let a cell be influenced by an item that was never binned into its
     * grid cell. An item whose bounds miss a point cannot change that point.
     *
     * Inverted (min > max) for an item the carve must never consider: a
     * suppressed ribbon or disc, or one whose centerline is empty, non-finite,
     * or not parallel to its heights.
     */
    struct ItemBounds {
        glm::dvec2 min{0.0};
        glm::dvec2 max{0.0};
    };

    /**
     * @brief Uniform-grid spatial index over ribbon and disc bounds
     *
     * Compressed-row layout: the items overlapping cell c are
     * `items[cell_starts[c] .. cell_starts[c + 1])`, ascending and without
     * duplicates. The id space runs ribbons, then discs, then portals; see
     * CarveInput::is_disc(), is_portal(), disc_index() and portal_index() rather
     * than comparing against `ribbons.size()` by hand.
     *
     * A ribbon is binned per SEGMENT rather than by its whole bounding box. A
     * long diagonal corridor has a box many times its own area, and binned whole
     * it becomes a candidate for every chunk near that box; binned per segment it
     * occupies only the cells it runs through. Duplicate ids are removed per cell
     * at build time, so the carve still visits each ribbon at most once per cell.
     *
     * Immutable once built, so concurrent readers need no synchronisation.
     */
    struct Index {
        glm::dvec2 min{0.0};                ///< Lower corner of the indexed region, local metres
        glm::dvec2 max{0.0};                ///< Upper corner of the indexed region, local metres
        double cell_size = 0.0;             ///< Grid cell edge in metres
        int width = 0;                      ///< Cells along x
        int height = 0;                     ///< Cells along y
        std::vector<uint32_t> cell_starts;  ///< width * height + 1 offsets into items
        std::vector<uint32_t> items;        ///< Item ids, grouped by cell
        std::vector<ItemBounds> bounds;     ///< Per item id, so the carve never recomputes them

        /**
         * @brief Per item id: the furthest distance at which it can move a cell
         *
         * Its own half extent -- for a ribbon, `half_width` times the LARGEST
         * miter scale on its centerline -- plus the widest blend band the slope
         * limit may open up. The same value the bounds were inflated by, kept so
         * the carve queries with exactly the reach the item was binned with
         * rather than recomputing a max over every station per cell. Zero for an
         * item the carve must never consider.
         */
        std::vector<double> reach;

        /**
         * @brief Per item id: carve this item as a polygon rather than as a disc
         *
         * Set only for a disc whose CarveDisc::outline is a usable ring; never for a
     * ribbon and never for a portal, whose mouth is always a rectangle --
         * `outline_is_simple`, at least three vertices, every vertex finite. Zero
         * for every ribbon and for every disc that falls back to `radius`.
         *
         * Decided ONCE, here, rather than re-derived per heightmap cell. The
         * decision and the reach the item was binned with have to agree: a disc
         * binned by its radius but carved against a ring reaching further would
         * influence cells it was never binned into, which is the exact trap the
         * miter widening already sprang once. Keeping the answer beside the reach
         * that was computed from it makes the two impossible to separate.
         */
        std::vector<uint8_t> polygon_footprint;

        bool built = false;                 ///< Set by build_index()
    };

    /// The spatial index. Do not fill this by hand; call build_index().
    Index index;

    /**
     * @brief Build the spatial index over the current ribbons and discs
     *
     * Call once after filling the vectors and setting config, and before carving
     * any chunk. Mutating ribbons, discs, or config afterwards invalidates the
     * index and requires another call.
     *
     * The cell size is chosen from the mean item extent so a typical ribbon
     * lands in a small constant number of cells. Degenerate inputs -- no items,
     * or all items at one point -- produce an index that is still marked built
     * and simply finds nothing.
     *
     * Every class of item is binned by a box inflated by exactly the reach stored
     * in Index::reach, and the carve then queries with that same value. Adding a
     * primitive means adding it to BOTH loops here, not only to the carve: an
     * item that influences a cell it was never binned into makes the result
     * depend on the grid rather than on the geometry. That has been the failure
     * mode twice; the portal pass below follows the disc pass line for line for
     * that reason.
     */
    void build_index();

    /// True once build_index() has run against the current contents
    [[nodiscard]] bool has_index() const { return index.built; }

    /**
     * @brief True when @p item is a disc rather than a ribbon or a portal
     *
     * The id space runs ribbons, then discs, then portals, so this is a RANGE
     * test and not merely a lower bound. It was a lower bound while discs were
     * the last class; leaving it that way while adding a third would have read
     * every portal as a disc and indexed past the end of `discs`.
     */
    [[nodiscard]] bool is_disc(uint32_t item) const {
        const size_t i = static_cast<size_t>(item);
        return i >= ribbons.size() && i < ribbons.size() + discs.size();
    }

    /// True when @p item is a tunnel portal mouth
    [[nodiscard]] bool is_portal(uint32_t item) const {
        return static_cast<size_t>(item) >= ribbons.size() + discs.size();
    }

    /// Disc index of a disc item id. Only meaningful when is_disc(@p item).
    [[nodiscard]] size_t disc_index(uint32_t item) const {
        return static_cast<size_t>(item) - ribbons.size();
    }

    /// Portal index of a portal item id. Only meaningful when is_portal(@p item).
    [[nodiscard]] size_t portal_index(uint32_t item) const {
        return static_cast<size_t>(item) - ribbons.size() - discs.size();
    }

    /// Total item count: ribbons, then discs, then portals
    [[nodiscard]] size_t item_count() const {
        return ribbons.size() + discs.size() + portals.size();
    }

    /// Drop every ribbon, disc, portal, and the index. Leaves config alone.
    void clear();
};

// ============================================================================
// Carve
// ============================================================================

/**
 * @brief Counts describing one chunk carve, for logging and tests
 */
struct CarveStats {
    size_t cells_modified = 0;  ///< Heightmap cells whose value changed
    float max_delta = 0.0f;     ///< Largest absolute height change, metres
    double carve_ms = 0.0;      ///< Wall-clock time of carve_terrain(), milliseconds
};

/**
 * @brief Carve the road corridors into one chunk's heightmap, in place
 *
 * For every heightmap cell inside a corridor footprint the terrain is set to the
 * road surface height at the nearest centerline station. Within
 * CarveConfig::falloff_metres of the footprint the height blends smoothly back
 * to the natural surface, producing cut and fill embankments. Cells outside both
 * are untouched, so a heightmap may be carved without having been regenerated.
 *
 * Where footprints overlap -- a junction disc and its arms, or two ribbons of a
 * grade-separated crossing at the same layer -- the carve takes the value of the
 * item with the greatest carve weight, so a junction does not step against the
 * roads feeding it. Weight is ranked by NORMALISED distance: distance past the
 * footprint edge divided by that item's own blend band width. That is the same
 * ordering as the weight itself, but it stays a strict order inside the
 * footprints too, where every weight is 1, so a cell deep inside a junction disc
 * beats a cell barely inside an arm. The result is never an average of two
 * items: averaging a junction against an arm at a different height gives a
 * surface matching neither, with a step at both kerbs. Nor is it first-found,
 * which would make the carve depend on index order.
 *
 * A ribbon or disc with `suppress` set is skipped entirely: a tunnel roadway is
 * below ground and a bridge deck floats above the terrain it spans, and in both
 * cases the natural surface is the correct one. A tunnel's two ends are the
 * exception, and they are handled by CarvePortal rather than by un-suppressing
 * the ribbon; see the portal note below.
 *
 * ### Junctions: polygon first, disc only as the fallback
 *
 * A junction whose P4 solve succeeded carries the real fillet-and-curb-ring
 * boundary in CarveDisc::outline, and that ring is what the carve tests: the
 * SIGNED distance to it, negative inside, drives the blend exactly as the
 * distance past a ribbon's kerb does. That is the plan's "pass 2" -- junction
 * neighbourhoods carved against real fillet polygons rather than a provisional
 * hull -- and it is a refinement rather than a correction, because the disc it
 * replaces bounds it.
 *
 * A polygon is safe here for the reason a ribbon's outline is not. A ribbon needs
 * a HEIGHT that varies along its length, and only a projection onto the
 * centerline gives one; a junction is PLANAR, at one solved node height, so a
 * point-in-polygon test plus a distance to the boundary answers everything. The
 * ring is used only when CarveDisc::outline_is_simple is set, for the same reason
 * that flag exists on a ribbon: a winding test against a ring that crosses itself
 * punches holes in the terrain. A degenerate junction, whose solve produced no
 * polygon at all, falls back to `radius` about `center`.
 *
 * Ribbon and polygon compose the same way two ribbons do -- nearest wins in
 * normalised distance, never an average -- so an arm meeting its junction hands
 * over at the point where the two footprint edges are equidistant, with no step
 * at either kerb. Both terminate at the same solved node height, so the hand-over
 * is continuous in value as well as in slope.
 *
 * Every ribbon is carved by DISTANCE to its centerline polyline, as a band of
 * `half_width` metres either side of it, and never by a point-in-polygon test
 * against `outline`. A footprint test answers "is this cell on the road", but
 * the carve needs a HEIGHT for every cell it touches, including every cell of the
 * embankment band outside the footprint, and a footprint gives none. The
 * projection parameter of the nearest point on the centerline interpolates
 * `centerline_heights` to give the road surface height there, and the distance
 * itself drives the blend, so one query answers both questions at once. It also
 * sidesteps the folded-ring problem entirely: a winding test against a
 * self-intersecting outline punches holes in the terrain, which is why
 * CarveRibbon carries `outline_is_simple` at all. A RIBBON's outline is therefore
 * used only to grow the spatial index bounds. A junction polygon is the other
 * case entirely; see the junction note above.
 *
 * The projection is a true point-to-segment projection, not a snap to the
 * nearest station. Nearest-station quantises the corridor into a chain of
 * circular scallops, one per station, most visible on straights, where
 * curvature-adaptive resampling puts stations furthest apart.
 *
 * The blend across the band is a Hermite smoothstep rather than a linear ramp. A
 * linear ramp has a discontinuous first derivative at both ends of the band,
 * which reads as a crease along the kerb and a second crease where the
 * embankment meets the natural surface.
 *
 * ### Tunnel portals: a ceiling applied after, never a competitor
 *
 * A CarvePortal does not enter the nearest-wins contest above, and it must not.
 * Ribbons and discs answer "what height is the road here", and the winner
 * replaces the terrain with that height. A portal answers a different question
 * -- "how high may the ground stand here" -- and the two cannot be ranked
 * against each other on one scale.
 *
 * So portals run as a SECOND pass over the same cell, against whatever the first
 * pass left there:
 *
 * @code
 *     ceiling = crown + (h - crown) * smoothstep(signed_distance_to_mouth / band)
 *     h       = min(h, ceiling)
 * @endcode
 *
 * where `h` is the height after the ribbon and disc pass and the mouth is the
 * plan rectangle `half_width` either side of CarvePortal::center running
 * `depth` metres along CarvePortal::axis, which points INTO the hillside. Inside
 * the rectangle the signed distance is negative, the smoothstep clamps to zero,
 * and the ceiling is exactly `crown_height`; at the outer edge of the band it is
 * exactly `h`, so the notch has sloped sides and no cliff.
 *
 * Three properties follow from that shape, and all three are the point of it:
 *
 * - **It never raises terrain.** `ceiling` is a blend between `crown` and `h`, so
 *   when the ground is already below the arch the ceiling is at or above `h` and
 *   the `min` changes nothing. A portal on level ground is inert, which is what
 *   TunnelPortalFootprint asks for -- a `set` would trench the approach.
 * - **It is order-independent.** Several portals compose as the minimum of their
 *   ceilings, and `min` does not care which was applied first.
 * - **It composes with the first pass rather than fighting it.** The ceiling
 *   blends back to the CARVED height, not to the natural one, so a mouth cut
 *   beside a carved approach embankment meets that embankment and not the terrain
 *   the embankment replaced.
 *
 * The band is widened by CarveConfig::max_embankment_slope exactly as a ribbon's
 * is, and bounded by the same cap, so the notch sides obey the same slope limit
 * as every other cut in the carve.
 *
 * @param heightmap Chunk heightmap, modified in place. Its origin, cell sizes,
 *                  and dimensions define the region carved.
 * @param input     Ribbons, discs, portals, config, and a built index
 * @return What changed. Zeroed when CarveConfig::enabled is false, when there
 *         are no items, or when @p input has no index.
 *
 * @pre input.has_index(). Without an index the function carves nothing and says
 *      so through zeroed stats rather than falling back to a linear scan, so a
 *      missing build_index() shows up as flat terrain instead of as a silent
 *      quadratic import.
 *
 * @note Safe to call concurrently for DIFFERENT heightmaps sharing one const
 *       CarveInput. It is not safe to call twice on the same heightmap
 *       concurrently.
 */
[[nodiscard]] CarveStats carve_terrain(Heightmap& heightmap, const CarveInput& input);

} // namespace stratum::procgen
