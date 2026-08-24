/**
 * @file corridor.hpp
 * @brief Sweep of a RoadProfile along a Centerline into a submeshed Mesh
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The corridor extruder is the only place in the road pipeline that turns 2D
 * local metres into world-space triangles. Everything upstream is topology and
 * plane geometry; everything downstream consumes a Mesh with MaterialId ranges.
 *
 * It walks the stations of a Centerline, emits one vertex column per strip edge
 * from a RoadProfile, and triangulates the band between consecutive stations.
 * Alongside the mesh it emits the corridor footprint polygon that terrain
 * carving needs in P3.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API. renderer/mesh.hpp is included for Mesh, Vertex, SubMesh, and MaterialId
 * only; that header is pure glm and is already compiled into stratum_core.
 */

#pragma once

#include "osm/road/centerline.hpp"
#include "osm/road/road_profile.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <cstddef>
#include <functional>
#include <vector>

namespace stratum::osm::road {

// ============================================================================
// Configuration
// ============================================================================

/**
 * @brief Vertical placement and optional outputs of one corridor sweep
 */
struct CorridorConfig {
    /// world Y of the carriageway surface when no elevation is supplied
    float base_height = 0.05f;

    /**
     * @brief Emit CurbFace strips
     *
     * When false, CurbFace strips contribute no geometry and the profile is left
     * visually stepped. Used by the collision-mesh variant in P7 and by any
     * caller that wants the cheapest possible ribbon.
     */
    bool emit_curb_faces = true;

    /// Fill Corridor::outline. When false the outline is left empty.
    bool emit_outline = true;

    /**
     * @brief Per-station world Y of the carriageway surface
     *
     * Empty means flat at base_height. P3 fills this from the terrain solve; size
     * must equal Centerline::stations.size(). A vector of any other non-zero size
     * is a caller error and is treated as empty, so a mis-sized elevation solve
     * degrades to a flat road rather than to mangled geometry.
     */
    std::vector<float> station_heights;

    /**
     * @brief Height the top of a kerb stands at, as a function of where it is
     *
     * The hook a dropped kerb is applied through. A drop is a MODULATION of the
     * profile the edge already has, never a second profile: the strip list, the
     * lateral layout and the outline are all unchanged, and only the heights of
     * the boundaries that a kerb owns move.
     *
     * Left null -- the default -- nothing is modulated and the sweep is
     * bit-identical to one with no drops anywhere, which is what makes the
     * feature bisectable.
     *
     * ### What is modulated
     *
     * Found structurally from the profile, in the order the kerb is built, and
     * mirrored on each side:
     *
     * - The RAISED edge of a CurbFace whose two heights differ. The lower edge
     *   sits at the carriageway surface and does not move.
     * - Both edges of every CurbTop outboard of that face.
     * - The INBOARD edge of the first strip beyond those -- the sidewalk. Its
     *   outboard edge keeps its full height, so the footway becomes the crossfall
     *   ramp and the ribbon can never tear away from the verge, the terrain or
     *   the junction ring beside it.
     * - Everything further outboard is untouched.
     *
     * A raised median has a kerb on each of its own sides and is modulated by the
     * same rule, which is what turns it into a pedestrian refuge at a crossing.
     *
     * Every boundary that moves takes exactly ONE call per station and side, so
     * two boundaries that must agree cannot be handed two different answers.
     *
     * @param arclength      Station::arclength, in the parameterisation the
     *                       centerline being swept carries. A trimmed slice does
     *                       not rebase arclength, so this is the same frame the
     *                       trims, the markings and the crossings are expressed
     *                       in.
     * @param left_of_travel Which of the edge's two kerb lines is being asked
     *                       about; true for the kerb at positive lateral.
     * @param full           The undropped height of that boundary, metres above
     *                       the carriageway surface.
     * @return The height that boundary stands at. Returning @p full is a no-op.
     *
     * @note The face's LATERAL width is not modulated with its height, so a
     *       dropped face keeps the full batter over a fraction of the rise and
     *       leans further than it should over the 20 mm it has left. Correcting
     *       it means recomputing the lateral layout per station, which moves
     *       every strip outboard of the kerb and the outline with them; at the
     *       shipping batter of 20 mm the error is smaller than the lip.
     * @note The centerline must carry stations across the ramp. A ramp laid out
     *       against columns metres apart is drawn as a step, and a drop that
     *       fits between two columns is not drawn at all. The caller is
     *       responsible for resampling first; see CorridorKerbProfile in
     *       crossings.hpp.
     */
    std::function<double(double arclength, bool left_of_travel, double full)> kerb_top_height;
};

// ============================================================================
// Output
// ============================================================================

/**
 * @brief One extruded road edge: geometry plus the footprint it occupies
 */
struct Corridor {
    /**
     * @brief The swept geometry, one SubMesh range per MaterialId
     *
     * build_corridor() calls Mesh::sort_submeshes_by_material() before returning,
     * so the ranges hold at most one entry per distinct material, are sorted
     * ascending by MaterialId, and tile the whole index buffer.
     */
    Mesh mesh;

    /**
     * @brief Closed CCW footprint of the full profile, for terrain carving
     *
     * In the same 2D local metres as the Centerline, not in world space. The ring
     * runs along the RIGHT edge of the profile from the first station to the
     * last, then back along the LEFT edge, which is counter-clockwise in the 2D
     * plane where positive lateral is to the left.
     *
     * The ring is closed implicitly: the first point is NOT repeated at the end.
     */
    std::vector<glm::dvec2> outline;

    /**
     * @brief The outline ring is not a simple polygon
     *
     * A hairpin tighter than the profile's half width folds the inner offset back
     * through itself, so the ring crosses itself and its winding number stops
     * meaning inside-or-outside. The extruder still emits the ring -- discarding
     * it would silently lose the road's footprint -- but flags it here and logs a
     * warning, so P3 can fall back to a per-band or union-of-quads carve instead
     * of a point-in-polygon test that would punch holes in the terrain.
     *
     * Always false when the outline is empty. Detection is a fold-back test on
     * both offset edges, which is O(n) and always runs, plus an exhaustive
     * pairwise crossing test that is skipped on very large rings, so a false
     * negative is possible on a long road with an exotic crossing that does not
     * reverse against the tangent.
     */
    bool outline_self_intersects = false;

    /// Arc length swept, in metres: the station arclength span of the centerline
    double length = 0.0;
};

// ============================================================================
// Extrusion
// ============================================================================

/**
 * @brief Sweep a cross-section along a centerline
 *
 * ### Lateral placement
 *
 * Strip edges are laid out from RoadProfile::left_edge_offset() walking right,
 * and every lateral coordinate is converted to a position through
 * offset_point(), which applies Station::miter_scale. No other conversion is
 * permitted; that function is the single definition of the miter.
 *
 * ### World mapping
 *
 * The 2D-to-3D mapping is the one used everywhere in this codebase, Y up:
 *
 * @code
 *     (x, y_2d) -> glm::vec3(x, height, -y_2d)
 * @endcode
 *
 * `height` is the carriageway surface height at that station -- base_height, or
 * CorridorConfig::station_heights[i] when supplied -- plus the strip's own
 * height above the carriageway surface, interpolated between Strip::height_left
 * and Strip::height_right across the strip.
 *
 * ### Vertex columns
 *
 * Each strip owns its own pair of vertex columns. Adjacent strips share edge
 * positions but not vertices, because U restarts at each strip boundary and
 * neighbouring strips usually differ in material. Normals are the strip's own
 * face normal, so a curb reads as a crisp edge rather than a smeared one. P7
 * welds what can safely be welded; the extruder does not.
 *
 * A strip of zero width emits no geometry UNLESS it is a CurbFace whose
 * height_left and height_right differ, which is the perfectly vertical curb and
 * must still produce its quad.
 *
 * ### Winding
 *
 * The renderer culls back faces with a counter-clockwise front face. For a band
 * between station i and station i+1, with the strip's left column vertices
 * L_i, L_{i+1} and its right column vertices R_i, R_{i+1}, emit:
 *
 * @code
 *     (L_i, R_i, R_{i+1})   and   (L_i, R_{i+1}, L_{i+1})
 * @endcode
 *
 * This single pattern gives the correct outward normal for every strip kind: +Y
 * for a horizontal surface, and for a CurbFace the normal that points away from
 * the raised side, that is, towards the carriageway for the curb of a raised
 * sidewalk. Do not flip it per side; the profile's left-to-right ordering
 * already encodes the mirroring.
 *
 * Triangles with zero area -- which occur on the centreline column across a
 * bevel pair -- are dropped rather than emitted.
 *
 * ### UVs
 *
 * The frozen UV convention, in metres, per uv_tiling():
 *
 * @code
 *     U = lateral_metres_within_strip / tile_u_metres(material)
 *     V = arclength_metres_along_road / tile_v_metres(material)
 * @endcode
 *
 * U restarts at 0 on each strip's LEFT edge and increases towards its right
 * edge. V is Station::arclength taken verbatim, so a trimmed ribbon keeps the
 * texture placement of the untrimmed one.
 *
 * CurbFace strips are the exception: their U runs UP the face, 0 at the lower
 * edge and height_difference / 0.5 at the upper edge. Their V is unchanged.
 * Curb top and curb face share MaterialId::Curb and therefore one texture, which
 * is authored with the face on one side and the top on the other.
 *
 * Vertex colour is opaque white for every corridor vertex. Appearance comes from
 * the material bound per SubMesh range, not from baked vertex colour.
 *
 * @param cl      Stations to sweep along; must be valid or the result is empty
 * @param profile Cross-section to sweep; must be valid or the result is empty
 * @param cfg     Vertical placement and optional outputs
 * @return The swept corridor. An empty Mesh and empty outline when either input
 *         is invalid.
 */
[[nodiscard]] Corridor build_corridor(const Centerline& cl,
                                      const RoadProfile& profile,
                                      const CorridorConfig& cfg);

/**
 * @brief Per-material UV tiling constants from the plan's UV Convention table
 *
 * Metres of surface covered by one full repeat of the texture, so texel density
 * is uniform across the whole network regardless of road width or segment
 * length.
 */
struct UVTiling {
    float u_metres;     ///< metres of lateral extent per U repeat
    float v_metres;     ///< metres of arc length per V repeat
};

/**
 * @brief Look up the tiling constants for a material
 *
 * The frozen table:
 *
 * | Material                     | tile U (m) | tile V (m) |
 * |------------------------------|-----------|-----------|
 * | Asphalt                      | 8.0       | 8.0       |
 * | Concrete, BridgeDeck         | 4.0       | 4.0       |
 * | Sidewalk                     | 2.0       | 2.0       |
 * | Curb                         | 0.5       | 2.0       |
 * | Gravel, Dirt, Grass          | 4.0       | 4.0       |
 * | Parapet                      | 2.0       | 2.0       |
 * | Default, Markings, Count     | 1.0       | 1.0       |
 *
 * Curb's 0.5 m U is the height of one curb texture repeat up the face, not a
 * lateral distance. See build_corridor().
 *
 * MaterialId::Markings is an atlas, not a tiling material: its geometry carries
 * explicit sub-rect UVs written by P5, so the 1.0 entry here is a neutral
 * placeholder and marking quads must not be scaled by it.
 *
 * @param material Material slot to look up
 * @return Tiling constants; {1.0f, 1.0f} for any material with no tiling entry
 */
[[nodiscard]] UVTiling uv_tiling(MaterialId material);

} // namespace stratum::osm::road
