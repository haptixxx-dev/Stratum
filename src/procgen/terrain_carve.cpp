/**
 * @file terrain_carve.cpp
 * @brief Implementation of the uniform-grid carve index and the per-chunk carve
 *
 * Two halves, in the order they run:
 *
 *   1. CarveInput::build_index()  bins every ribbon SEGMENT, every disc and
 *      every tunnel portal mouth into a uniform grid, keyed by the axis-aligned
 *      box each one can reach. Built once per import, then shared by const
 *      reference with every chunk carve.
 *   2. carve_terrain()  walks one chunk's heightmap cells, queries the single
 *      grid cell that cell falls in, blends the winning road surface into the
 *      natural surface, and then applies any portal mouth as a ceiling on the
 *      result.
 *
 * ### Why distance to the centerline and not the outline
 *
 * CarveRibbon carries a corridor outline, and a point-in-polygon test against it
 * would answer "is this cell on the road". That is the wrong question. The carve
 * has to produce a HEIGHT for every cell it touches, including every cell in the
 * embankment band outside the footprint, and a footprint test gives no height at
 * all -- neither inside, where the surface tilts along the road, nor outside,
 * where the blend needs to know what it is blending towards.
 *
 * Distance to the centerline polyline answers both at once: the projection
 * parameter of the nearest point interpolates centerline_heights to give the
 * road surface height there, and the distance itself drives the blend. The
 * outline is therefore left for P4, which replaces the provisional discs with
 * real junction polygons.
 *
 * The projection must be a true point-to-SEGMENT projection. Snapping to the
 * nearest station instead quantises the corridor into a chain of circular
 * scallops, one per station, which is plainly visible wherever stations are
 * sparse -- that is, on every straight, where curvature-adaptive resampling puts
 * them furthest apart.
 *
 * ### Why a junction IS carved against its outline
 *
 * The argument above is about ribbons, and it does not carry over. A junction is
 * PLANAR: every point of it sits at one solved node height, so there is no height
 * to interpolate and nothing a centerline projection would tell us that the
 * polygon does not. A point-in-polygon test plus the distance to the ring
 * boundary gives a signed distance -- negative inside, positive outside -- which
 * is the exact analogue of `distance_to_centerline - half_width` on a ribbon and
 * feeds the same blend.
 *
 * That is the plan's "pass 2": P3 carved a flat disc covering the arm mouths, P4
 * carves the real fillet-and-curb-ring footprint. The disc survives as the
 * fallback for a junction whose trim solve was degenerate and produced no
 * polygon, and because it bounds the polygon, the polygon is a refinement of it
 * rather than a contradiction.
 *
 * The self-intersection guard is the same one ribbons carry: a winding test
 * against a ring that crosses itself has no meaningful answer and punches holes
 * in the terrain, so `outline_is_simple` gates the polygon path and a ring that
 * fails it falls back to the disc.
 *
 * ### Why the index bins segments rather than whole ribbons
 *
 * A long diagonal ribbon has an axis-aligned box many times its own area. Binned
 * whole, it becomes a candidate for every chunk anywhere near that box. Binned
 * per segment the ribbon occupies only the cells it actually runs through, at
 * the cost of a duplicate-removal pass at build time. Item ids are deduplicated
 * per cell, so the carve still sees each ribbon at most once per cell.
 *
 * ### Reach, and why it is not just falloff_metres
 *
 * The embankment slope limit widens the blend band locally rather than letting
 * it steepen into a cliff, so a cell further than falloff_metres from a road can
 * still be influenced by it. The widening is capped at kMaxBandScale times the
 * configured falloff, and the index inflates every box by that same capped
 * amount. Reach at carve time can therefore never exceed reach at build time,
 * which is what makes a single-cell query complete rather than merely likely.
 *
 * The same rule binds the junction polygon and the portal mouth. A disc is binned by whichever is
 * larger, its stated radius or the furthest vertex of its ring, so a ring poking
 * outside the radius its producer claimed is still fully binned. The choice
 * between polygon and disc is made ONCE, at index build time, and cached in
 * Index::polygon_footprint -- deciding it again per cell would let the two
 * disagree, and a disc binned by its radius but carved against a wider ring is
 * precisely the "widened band outruns its bins" defect in a new costume.
 *
 * A portal is binned about CarvePortal::center -- the middle of the opening
 * itself -- inflated by the furthest corner of its mouth rectangle plus the same
 * capped band. The carve's radial reject then uses that stored reach rather than
 * recomputing one, which is the whole point of storing it: the two cannot drift.
 *
 * ### Why a portal mouth is a ceiling and not a target
 *
 * A ribbon and a disc both say what height the ground IS. A portal says only how
 * high the ground may STAND: the arch has to stay visible, and the hillside
 * either side of it must not. Setting the terrain to CarvePortal::crown_height
 * would trench the level approach in front of every portal, so the mouth is
 * applied as `min(height, ceiling)` after the ribbon and disc pass, where the
 * ceiling blends from the crown at the mouth back to the carved height at the
 * edge of the band. On ground already below the arch the ceiling sits at or above
 * that height and the min changes nothing, which is exactly the inertness
 * TunnelPortalFootprint asks for.
 *
 * Every mouth is measured against the height the ribbon and disc pass left, not
 * against a running value, so overlapping mouths compose as a plain minimum and
 * the answer does not depend on index order.
 *
 * Everything in this file lives in stratum_core: no SDL, no ImGui, no rendering
 * API.
 */

#include "procgen/terrain_carve.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

namespace stratum::procgen {

namespace {

// ============================================================================
// Tunables of the implementation
// ============================================================================

/**
 * @brief Hard cap on how far the slope limit may widen the falloff band
 *
 * Expressed as a multiple of CarveConfig::falloff_metres. The cap exists so the
 * carve has a bounded reach, which the spatial index depends on: a cell may only
 * be influenced by items binned into it, and items are binned by a box inflated
 * by exactly this reach.
 */
constexpr double kMaxBandScale = 4.0;

/// Floor on the blend band width, metres. Keeps the normalised distance finite.
constexpr double kMinBand = 1e-3;

/// Below this a squared length is treated as zero
constexpr double kLenEpsilon = 1e-12;

/// Smallest grid cell the index will use, metres
constexpr double kMinCellSize = 16.0;

/// Largest grid cell the index will use, metres
constexpr double kMaxCellSize = 256.0;

/// Cell budget. The cell size doubles until the grid fits inside this.
constexpr size_t kMaxIndexCells = size_t{1} << 22;

/// Height changes smaller than this are not worth a write, metres
constexpr float kChangeEpsilon = 1e-6f;

// ============================================================================
// Small geometry helpers
// ============================================================================

/// True when @p p lies inside the closed box @p b. False for an inverted box.
[[nodiscard]] bool bounds_contain(const CarveInput::ItemBounds& b, const glm::dvec2& p) {
    return p.x >= b.min.x && p.x <= b.max.x && p.y >= b.min.y && p.y <= b.max.y;
}

/// A box no point can be inside, used for items the carve must never consider
[[nodiscard]] CarveInput::ItemBounds empty_bounds() {
    CarveInput::ItemBounds b;
    b.min = glm::dvec2(std::numeric_limits<double>::max());
    b.max = glm::dvec2(std::numeric_limits<double>::lowest());
    return b;
}

/// True when the box was filled with at least one real point
[[nodiscard]] bool bounds_valid(const CarveInput::ItemBounds& b) {
    return b.min.x <= b.max.x && b.min.y <= b.max.y;
}

[[nodiscard]] bool is_finite(const glm::dvec2& p) {
    return std::isfinite(p.x) && std::isfinite(p.y);
}

/// Result of projecting a point onto one centerline segment
struct SegmentHit {
    double dist2 = 0.0;  ///< Squared distance from the point to the segment
    double t = 0.0;      ///< Projection parameter along the segment, clamped to [0, 1]
};

/**
 * @brief True point-to-segment projection
 *
 * Not nearest-vertex. See the scallop note in the file comment.
 */
[[nodiscard]] SegmentHit project_on_segment(const glm::dvec2& p,
                                            const glm::dvec2& a,
                                            const glm::dvec2& b) {
    const glm::dvec2 ab = b - a;
    const double len2 = glm::dot(ab, ab);

    double t = 0.0;
    if (len2 > kLenEpsilon) {
        t = std::clamp(glm::dot(p - a, ab) / len2, 0.0, 1.0);
    }

    const glm::dvec2 delta = p - (a + ab * t);
    return SegmentHit{glm::dot(delta, delta), t};
}

/// Nearest approach of a point to one ribbon, with the road surface height there
struct RibbonHit {
    bool valid = false;   ///< False when the ribbon is unusable or out of reach
    double dist2 = 0.0;   ///< Squared distance to the centerline polyline
    float height = 0.0f;  ///< Interpolated centerline_heights at the nearest point
    double miter = 1.0;   ///< Interpolated centerline_miter at the nearest point, >= 1
};

/**
 * @brief Largest miter scale carried by a ribbon, never below 1
 *
 * The corridor's outer edge reaches `half_width * miter_scale` from the
 * centerline, so this is what the band and the index reach must both be sized
 * from. Sizing only ONE of them by it is worse than sizing neither: a band wider
 * than the reach the item was binned with lets a cell be influenced by a ribbon
 * that was never binned into its grid cell, which makes the carve depend on the
 * grid rather than on the geometry.
 */
[[nodiscard]] double ribbon_max_miter(const CarveRibbon& ribbon) {
    if (ribbon.centerline_miter.size() != ribbon.centerline.size()) return 1.0;

    double worst = 1.0;
    for (float m : ribbon.centerline_miter) {
        if (std::isfinite(m) && static_cast<double>(m) > worst) worst = static_cast<double>(m);
    }
    return worst;
}

/**
 * @brief Distance from @p p to a ribbon's centerline, and the height there
 *
 * Segments whose box, inflated by @p reach, misses @p p are rejected before the
 * projection runs, so a cell near one end of a long ribbon does not pay for the
 * far end.
 *
 * @param ribbon Corridor to test against; its heights must be parallel to its centerline
 * @param p      Query point in 2D local metres
 * @param reach  Furthest distance at which this ribbon can influence a cell, metres
 */
[[nodiscard]] RibbonHit nearest_on_ribbon(const CarveRibbon& ribbon,
                                          const glm::dvec2& p,
                                          double reach) {
    const size_t count = ribbon.centerline.size();

    // No height means no surface to carve to. Silently skipping is correct here:
    // the alternative is carving to an invented height.
    if (count == 0 || ribbon.centerline_heights.size() != count) {
        return RibbonHit{};
    }

    // A single station is not a corridor. It has no direction, no segment to
    // project onto, and no length, so there is nothing to carve. Treating it as a
    // disc of half_width would invent a footprint the road pipeline never emitted
    // -- build_centerline() rejects anything under two stations -- and a stray
    // one-point ribbon would then flatten a circle of terrain out of nowhere.
    if (count < 2) {
        return RibbonHit{};
    }

    const bool have_miter = ribbon.centerline_miter.size() == count;

    RibbonHit best{};
    double best_dist2 = std::numeric_limits<double>::max();

    for (size_t i = 0; i + 1 < count; ++i) {
        const glm::dvec2& a = ribbon.centerline[i];
        const glm::dvec2& b = ribbon.centerline[i + 1];

        if (p.x < std::min(a.x, b.x) - reach || p.x > std::max(a.x, b.x) + reach) continue;
        if (p.y < std::min(a.y, b.y) - reach || p.y > std::max(a.y, b.y) + reach) continue;

        const SegmentHit hit = project_on_segment(p, a, b);
        if (hit.dist2 >= best_dist2) continue;

        const double h0 = static_cast<double>(ribbon.centerline_heights[i]);
        const double h1 = static_cast<double>(ribbon.centerline_heights[i + 1]);

        best_dist2 = hit.dist2;
        best.valid = true;
        best.dist2 = hit.dist2;
        best.height = static_cast<float>(h0 + (h1 - h0) * hit.t);

        // Interpolated on the SAME projection parameter as the height. The
        // corridor edge between two stations is the straight line joining their
        // mitred offset points, so its lateral distance varies with t the way
        // this does.
        if (have_miter) {
            const double m0 = static_cast<double>(ribbon.centerline_miter[i]);
            const double m1 = static_cast<double>(ribbon.centerline_miter[i + 1]);
            const double m = m0 + (m1 - m0) * hit.t;
            best.miter = std::isfinite(m) && m > 1.0 ? m : 1.0;
        } else {
            best.miter = 1.0;
        }
    }

    return best;
}

// ============================================================================
// Junction polygon primitive
// ============================================================================

/**
 * @brief Where a point sits relative to a closed ring
 */
struct RingHit {
    bool valid = false;   ///< The ring had at least three vertices
    bool inside = false;  ///< The point is inside the ring
    double dist = 0.0;    ///< Unsigned distance to the nearest ring edge, metres
};

/**
 * @brief Inside-test and boundary distance for a closed ring, in one pass
 *
 * Both answers are needed for every ring the carve considers, and both walk the
 * same edges, so they are computed together rather than in two loops.
 *
 * The inside test is a crossing count with a half-open y comparison, so a ray
 * passing exactly through a shared vertex crosses once rather than twice or zero
 * times. The result is only meaningful for a SIMPLE ring; callers gate on
 * CarveDisc::outline_is_simple.
 *
 * @param ring Closed ring in 2D local metres, first point NOT repeated
 * @param p    Query point in the same frame
 * @return Whether @p p is inside, and how far it is from the boundary
 */
[[nodiscard]] RingHit ring_hit(const std::vector<glm::dvec2>& ring, const glm::dvec2& p) {
    RingHit hit;

    const size_t n = ring.size();
    if (n < 3) return hit;

    double best2 = std::numeric_limits<double>::max();
    bool inside = false;

    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const glm::dvec2& a = ring[j];
        const glm::dvec2& b = ring[i];

        // (b.y > p.y) != (a.y > p.y) already implies a.y != b.y, so the divide
        // below cannot be by zero and a horizontal edge is never counted.
        if ((b.y > p.y) != (a.y > p.y)) {
            const double t = (p.y - b.y) / (a.y - b.y);
            if (p.x < b.x + t * (a.x - b.x)) inside = !inside;
        }

        const SegmentHit seg = project_on_segment(p, a, b);
        if (seg.dist2 < best2) best2 = seg.dist2;
    }

    hit.valid = true;
    hit.inside = inside;
    hit.dist = std::sqrt(best2);
    return hit;
}

/**
 * @brief How a junction disc is to be carved, and how far it reaches
 *
 * Decided once per disc at index build time and cached in
 * Index::polygon_footprint, never re-derived per heightmap cell. See the reach
 * note in the file comment for why the two must not be allowed to drift apart.
 */
struct DiscFootprint {
    /// Carve the ring rather than the circle
    bool use_polygon = false;

    /**
     * @brief Furthest the carved footprint reaches from CarveDisc::center, metres
     *
     * The stated radius, or the furthest ring vertex, whichever is larger.
     * CarveDisc documents the radius as bounding the outline, but the index must
     * not depend on the producer honouring that: a ring poking outside its own
     * radius has to be binned by the ring.
     */
    double half_extent = 0.0;
};

/**
 * @brief Validate a disc's polygon footprint and measure its extent
 *
 * A ring is usable only when it is marked simple, has at least three vertices,
 * and every vertex is finite. One non-finite vertex makes the whole ring
 * unusable rather than merely skipped: a NaN would poison both the inside test
 * and the bounds it is binned by, and the disc it falls back to is a valid
 * footprint in its own right.
 */
[[nodiscard]] DiscFootprint disc_footprint(const CarveDisc& disc) {
    DiscFootprint fp;
    fp.half_extent = std::isfinite(disc.radius)
                         ? std::max(0.0, static_cast<double>(disc.radius))
                         : 0.0;

    if (!disc.outline_is_simple || disc.outline.size() < 3) return fp;

    double worst = 0.0;
    for (const glm::dvec2& v : disc.outline) {
        if (!is_finite(v)) return fp;
        worst = std::max(worst, glm::length(v - disc.center));
    }

    fp.use_polygon = true;
    fp.half_extent = std::max(fp.half_extent, worst);
    return fp;
}

// ============================================================================
// Tunnel portal primitive
// ============================================================================

/**
 * @brief Plan frame of a portal mouth, and how far it reaches
 *
 * CarvePortal describes the mouth as a face centre, an axis pointing into the
 * hillside, a half width and a depth. Every query below wants the RECTANGLE
 * instead: its own centre, two unit axes and two half extents. Deriving that once
 * per portal at index build time, rather than per heightmap cell, is the same
 * trade DiscFootprint makes.
 */
struct PortalFrame {
    bool valid = false;         ///< The portal describes a rectangle with area

    glm::dvec2 face{0.0};       ///< CarvePortal::center: the middle of the opening itself
    glm::dvec2 center{0.0};     ///< Centre of the rectangle, half a depth in along the axis
    glm::dvec2 axis{1.0, 0.0};  ///< Unit, into the hillside
    glm::dvec2 side{0.0, 1.0};  ///< Unit, across the opening

    double half_depth = 0.0;    ///< Half CarvePortal::depth
    double half_width = 0.0;    ///< CarvePortal::half_width

    /**
     * @brief Furthest rectangle corner from `face`, metres
     *
     * Measured from the FACE and not from the rectangle centre, because the index
     * bins a portal about its face exactly as it bins a disc about its centre,
     * and the carve's radial reject then uses the same point. The far corners sit
     * at `hypot(depth, half_width)`; the near ones at `half_width`.
     */
    double half_extent = 0.0;

    float crown = 0.0f;         ///< World Y the ground is clamped down to inside the mouth
};

/**
 * @brief Derive the mouth rectangle of a portal, rejecting a degenerate one
 *
 * A portal with no depth, no width, a non-finite crown or an axis that cannot be
 * normalised describes no rectangle. It is rejected whole rather than repaired:
 * every one of those cases means the tunnel builder emitted a footprint it should
 * not have, and clamping the terrain against a guessed rectangle would cut a
 * notch somewhere no headwall stands.
 */
[[nodiscard]] PortalFrame portal_frame(const CarvePortal& portal) {
    PortalFrame frame;

    if (!is_finite(portal.center) || !is_finite(portal.axis)) return frame;
    if (!std::isfinite(portal.crown_height)) return frame;
    if (!std::isfinite(portal.half_width) || portal.half_width <= 0.0) return frame;
    if (!std::isfinite(portal.depth) || portal.depth <= 0.0) return frame;

    const double axis_len2 = glm::dot(portal.axis, portal.axis);
    if (axis_len2 <= kLenEpsilon) return frame;

    frame.axis = portal.axis / std::sqrt(axis_len2);

    // Left of the axis under the 2D convention the whole road module uses. Which
    // side it points at does not matter -- the mouth test takes |v| -- only that
    // it is unit and perpendicular.
    frame.side = glm::dvec2(-frame.axis.y, frame.axis.x);

    frame.face = portal.center;
    frame.half_depth = 0.5 * portal.depth;
    frame.half_width = portal.half_width;
    frame.center = frame.face + frame.axis * frame.half_depth;
    frame.half_extent = std::hypot(portal.depth, portal.half_width);
    frame.crown = portal.crown_height;
    frame.valid = true;
    return frame;
}

/**
 * @brief Signed distance from @p p to a portal mouth rectangle, negative inside
 *
 * The standard box distance in the rectangle's own frame. Zero exactly on the
 * boundary, so it drives the blend the way a ribbon's `distance - half_width`
 * and a junction ring's signed distance already do, and the three bands can be
 * described by one formula.
 */
[[nodiscard]] double mouth_signed_distance(const PortalFrame& frame, const glm::dvec2& p) {
    const glm::dvec2 d = p - frame.center;
    const double u = std::abs(glm::dot(d, frame.axis)) - frame.half_depth;
    const double v = std::abs(glm::dot(d, frame.side)) - frame.half_width;

    const double outside = glm::length(glm::dvec2(std::max(u, 0.0), std::max(v, 0.0)));
    const double inside = std::min(std::max(u, v), 0.0);
    return outside + inside;
}

/**
 * @brief Hermite smoothstep, 0 at t = 0 and 1 at t = 1
 *
 * Used instead of a linear ramp because a linear blend has a discontinuous first
 * derivative at both ends of the band, which reads as a crease along the kerb
 * and a second crease where the embankment meets the natural surface. Smoothstep
 * matches slope at both, so neither edge is visible.
 */
[[nodiscard]] double smoothstep01(double t) {
    const double c = std::clamp(t, 0.0, 1.0);
    return c * c * (3.0 - 2.0 * c);
}

// ============================================================================
// Index construction helpers
// ============================================================================

/// Grow @p b to contain @p p, ignoring non-finite input
void grow_bounds(CarveInput::ItemBounds& b, const glm::dvec2& p) {
    if (!is_finite(p)) return;
    b.min = glm::min(b.min, p);
    b.max = glm::max(b.max, p);
}

/// Expand @p b outward by @p amount on every side
void inflate_bounds(CarveInput::ItemBounds& b, double amount) {
    b.min -= glm::dvec2(amount);
    b.max += glm::dvec2(amount);
}

/**
 * @brief Furthest distance at which a footprint of @p half_extent can move a cell
 *
 * The footprint itself, plus the widest blend band the slope limit is allowed to
 * open up. Used identically at build time and at carve time.
 */
[[nodiscard]] double influence_reach(double half_extent, const CarveConfig& cfg) {
    const double falloff = std::max(0.0, static_cast<double>(cfg.falloff_metres));
    return std::max(0.0, half_extent) + falloff * kMaxBandScale;
}

} // namespace

// ============================================================================
// CarveInput
// ============================================================================

void CarveInput::clear() {
    ribbons.clear();
    discs.clear();
    portals.clear();
    index = Index{};
}

void CarveInput::build_index() {
    index = Index{};

    const size_t items_total = item_count();
    index.bounds.assign(items_total, empty_bounds());
    index.reach.assign(items_total, 0.0);
    index.polygon_footprint.assign(items_total, uint8_t{0});

    if (items_total == 0) {
        // Still "built": a carve against an empty network must be a no-op, not a
        // warning about a missing index.
        index.cell_starts.assign(1, 0);
        index.built = true;
        return;
    }

    if (items_total > static_cast<size_t>(std::numeric_limits<uint32_t>::max())) {
        spdlog::error("[carve] {} items exceeds the 32-bit item id space; index not built",
                      items_total);
        return;
    }

    // ------------------------------------------------------------------
    // Pass 1: per-item boxes, already inflated by the full carve reach
    // ------------------------------------------------------------------
    CarveInput::ItemBounds region = empty_bounds();
    double extent_sum = 0.0;
    size_t active_items = 0;

    for (size_t i = 0; i < ribbons.size(); ++i) {
        const CarveRibbon& ribbon = ribbons[i];

        // Tunnels and bridge spans are carried but never carved, so they get an
        // inverted box and are never binned.
        if (ribbon.suppress) continue;

        // Fewer than two stations is no corridor at all; see nearest_on_ribbon().
        // Such a ribbon keeps its inverted box and is never binned, so the carve
        // never even reaches it.
        if (ribbon.centerline.size() < 2) continue;
        if (ribbon.centerline_heights.size() != ribbon.centerline.size()) continue;

        CarveInput::ItemBounds b = empty_bounds();
        for (const glm::dvec2& p : ribbon.centerline) grow_bounds(b, p);

        // The outline is not what the carve tests against, but it is a footprint
        // the carve must cover, so it is folded into the box.
        for (const glm::dvec2& p : ribbon.outline) grow_bounds(b, p);

        if (!bounds_valid(b)) continue;

        // Sized from the ribbon's WIDEST miter, so the box covers the outer
        // corner of every mitred bend. The carve then queries with exactly this
        // value; see CarveInput::Index::reach.
        const double reach = influence_reach(
            static_cast<double>(ribbon.half_width) * ribbon_max_miter(ribbon), config);

        inflate_bounds(b, reach);
        index.bounds[i] = b;
        index.reach[i] = reach;

        grow_bounds(region, b.min);
        grow_bounds(region, b.max);
        extent_sum += 0.5 * ((b.max.x - b.min.x) + (b.max.y - b.min.y));
        ++active_items;
    }

    for (size_t i = 0; i < discs.size(); ++i) {
        const CarveDisc& disc = discs[i];
        if (disc.suppress) continue;
        if (!is_finite(disc.center)) continue;

        // Polygon or circle, and how far the chosen footprint reaches. Decided
        // here and cached, so the carve queries with exactly the reach the disc
        // was binned with; see the reach note in the file comment.
        const DiscFootprint fp = disc_footprint(disc);

        // A non-finite radius was already unusable before P4, and stays so, but
        // only when there is no ring to carve instead.
        if (!fp.use_polygon && !std::isfinite(disc.radius)) continue;

        CarveInput::ItemBounds b = empty_bounds();
        grow_bounds(b, disc.center);
        if (!bounds_valid(b)) continue;

        // Sized from the ring when there is one. Any point the polygon carve can
        // influence lies within `half_extent + widest band` of the centre: its
        // distance to the ring is at least its distance to the centre less
        // half_extent, and the blend stops once that exceeds the band.
        const double reach = influence_reach(fp.half_extent, config);
        inflate_bounds(b, reach);
        index.bounds[ribbons.size() + i] = b;
        index.reach[ribbons.size() + i] = reach;
        index.polygon_footprint[ribbons.size() + i] = fp.use_polygon ? uint8_t{1} : uint8_t{0};

        grow_bounds(region, b.min);
        grow_bounds(region, b.max);
        extent_sum += 0.5 * ((b.max.x - b.min.x) + (b.max.y - b.min.y));
        ++active_items;
    }

    // Portals are binned on exactly the disc pattern above: a box about one point
    // inflated by the reach, and that same reach stored for the carve to query
    // with. Deliberately line for line, because the one way this index goes wrong
    // is a primitive whose carve reaches further than its bins -- it has happened
    // twice -- and the way to keep it fixed is for a new primitive to be
    // indistinguishable from an old one here.
    const size_t portal_base = ribbons.size() + discs.size();
    for (size_t i = 0; i < portals.size(); ++i) {
        const PortalFrame frame = portal_frame(portals[i]);
        if (!frame.valid) continue;

        CarveInput::ItemBounds b = empty_bounds();
        grow_bounds(b, frame.face);
        if (!bounds_valid(b)) continue;

        // Every point the mouth can influence lies within `half_extent + band` of
        // the face: its distance to the rectangle is at least its distance to the
        // face less half_extent, and the ceiling stops biting once that exceeds
        // the band.
        const double reach = influence_reach(frame.half_extent, config);
        inflate_bounds(b, reach);
        index.bounds[portal_base + i] = b;
        index.reach[portal_base + i] = reach;

        grow_bounds(region, b.min);
        grow_bounds(region, b.max);
        extent_sum += 0.5 * ((b.max.x - b.min.x) + (b.max.y - b.min.y));
        ++active_items;
    }

    if (active_items == 0 || !bounds_valid(region)) {
        // Every item was suppressed or degenerate. The index is built and finds
        // nothing, which is exactly what the carve should then do.
        index.cell_starts.assign(1, 0);
        index.built = true;
        return;
    }

    // ------------------------------------------------------------------
    // Grid geometry
    // ------------------------------------------------------------------
    index.min = region.min;
    index.max = region.max;

    const double span_x = std::max(index.max.x - index.min.x, 1.0);
    const double span_y = std::max(index.max.y - index.min.y, 1.0);

    // Cell size follows the mean item extent, so a typical item lands in a small
    // constant number of cells regardless of the scale of the import. Clamped
    // because neither a 1 m grid over a city nor a 2 km grid over a village
    // discriminates usefully.
    double cell = extent_sum / static_cast<double>(active_items);
    if (!std::isfinite(cell) || cell <= 0.0) cell = kMinCellSize;
    cell = std::clamp(cell, kMinCellSize, kMaxCellSize);

    int grid_w = 1;
    int grid_h = 1;
    for (;;) {
        grid_w = std::max(1, static_cast<int>(std::ceil(span_x / cell)));
        grid_h = std::max(1, static_cast<int>(std::ceil(span_y / cell)));
        if (static_cast<size_t>(grid_w) * static_cast<size_t>(grid_h) <= kMaxIndexCells) break;
        cell *= 2.0;
    }

    index.cell_size = cell;
    index.width = grid_w;
    index.height = grid_h;

    const size_t cell_total = static_cast<size_t>(grid_w) * static_cast<size_t>(grid_h);

    // ------------------------------------------------------------------
    // Pass 2: bin every segment and every disc, then deduplicate per cell
    // ------------------------------------------------------------------
    // Key layout is (cell << 32) | item, so one sort orders by cell and then by
    // item, which is the compressed-row order the carve reads.
    std::vector<uint64_t> keys;
    keys.reserve(items_total * 4);

    const auto emit_box = [&](const CarveInput::ItemBounds& b, uint32_t item) {
        const int x0 = std::clamp(static_cast<int>(std::floor((b.min.x - index.min.x) / cell)),
                                  0, grid_w - 1);
        const int x1 = std::clamp(static_cast<int>(std::floor((b.max.x - index.min.x) / cell)),
                                  0, grid_w - 1);
        const int y0 = std::clamp(static_cast<int>(std::floor((b.min.y - index.min.y) / cell)),
                                  0, grid_h - 1);
        const int y1 = std::clamp(static_cast<int>(std::floor((b.max.y - index.min.y) / cell)),
                                  0, grid_h - 1);

        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                const uint64_t c = static_cast<uint64_t>(y) * static_cast<uint64_t>(grid_w)
                                 + static_cast<uint64_t>(x);
                keys.push_back((c << 32) | static_cast<uint64_t>(item));
            }
        }
    };

    for (size_t i = 0; i < ribbons.size(); ++i) {
        if (!bounds_valid(index.bounds[i])) continue;

        const CarveRibbon& ribbon = ribbons[i];
        const double reach = index.reach[i];
        const uint32_t item = static_cast<uint32_t>(i);

        // Per segment, not per ribbon: a long diagonal corridor otherwise floods
        // every cell of its bounding box.
        for (size_t s = 0; s + 1 < ribbon.centerline.size(); ++s) {
            CarveInput::ItemBounds b = empty_bounds();
            grow_bounds(b, ribbon.centerline[s]);
            grow_bounds(b, ribbon.centerline[s + 1]);
            if (!bounds_valid(b)) continue;
            inflate_bounds(b, reach);
            emit_box(b, item);
        }
    }

    for (size_t i = 0; i < discs.size(); ++i) {
        const size_t item = ribbons.size() + i;
        if (!bounds_valid(index.bounds[item])) continue;
        emit_box(index.bounds[item], static_cast<uint32_t>(item));
    }

    for (size_t i = 0; i < portals.size(); ++i) {
        const size_t item = portal_base + i;
        if (!bounds_valid(index.bounds[item])) continue;
        emit_box(index.bounds[item], static_cast<uint32_t>(item));
    }

    std::sort(keys.begin(), keys.end());
    keys.erase(std::unique(keys.begin(), keys.end()), keys.end());

    // ------------------------------------------------------------------
    // Compressed-row layout
    // ------------------------------------------------------------------
    index.cell_starts.assign(cell_total + 1, 0);
    index.items.resize(keys.size());

    for (size_t k = 0; k < keys.size(); ++k) {
        const size_t c = static_cast<size_t>(keys[k] >> 32);
        index.items[k] = static_cast<uint32_t>(keys[k] & 0xFFFFFFFFull);
        ++index.cell_starts[c + 1];
    }
    for (size_t c = 0; c < cell_total; ++c) {
        index.cell_starts[c + 1] += index.cell_starts[c];
    }

    index.built = true;

    spdlog::debug("[carve] index: {} items ({} ribbons, {} discs, {} portals; {} active), "
                  "{}x{} cells of {:.1f} m, {} entries",
                  items_total, ribbons.size(), discs.size(), portals.size(), active_items,
                  grid_w, grid_h, cell, index.items.size());
}

// ============================================================================
// Carve
// ============================================================================

CarveStats carve_terrain(Heightmap& heightmap, const CarveInput& input) {
    const auto started = std::chrono::steady_clock::now();

    CarveStats stats;

    const auto finish = [&]() {
        const auto finished = std::chrono::steady_clock::now();
        stats.carve_ms = std::chrono::duration<double, std::milli>(finished - started).count();
        return stats;
    };

    if (!input.config.enabled) return finish();
    if (input.item_count() == 0) return finish();
    if (heightmap.width <= 0 || heightmap.height <= 0) return finish();

    const size_t sample_total = static_cast<size_t>(heightmap.width)
                              * static_cast<size_t>(heightmap.height);
    if (heightmap.data.size() < sample_total) return finish();

    if (!input.has_index()) {
        // Deliberately not a fallback linear scan: a missing build_index() on a
        // city extract would turn one import into a quadratic one, and a slow
        // import is far harder to diagnose than flat terrain.
        spdlog::warn("[carve] CarveInput has no index; nothing carved. Call build_index() first.");
        return finish();
    }

    const CarveInput::Index& index = input.index;
    if (index.cell_size <= 0.0 || index.items.empty()) return finish();

    // Both are written together by build_index() and read per item below. A
    // mismatch means the index was filled by hand; refuse it rather than read
    // past the end.
    if (index.reach.size() != index.bounds.size()
        || index.polygon_footprint.size() != index.bounds.size()) {
        spdlog::error("[carve] index bounds, reach and footprint flags disagree "
                      "({} vs {} vs {}); nothing carved",
                      index.bounds.size(), index.reach.size(),
                      index.polygon_footprint.size());
        return finish();
    }

    const CarveConfig& cfg = input.config;
    const double falloff = std::max(0.0, static_cast<double>(cfg.falloff_metres));
    const double max_band = falloff * kMaxBandScale;
    const double slope = static_cast<double>(cfg.max_embankment_slope);

    // One frame per portal, derived once for the whole chunk rather than once per
    // cell. A degenerate portal comes back invalid and is skipped everywhere it
    // is met, so the mouth pass never has to re-validate.
    std::vector<PortalFrame> portal_frames;
    portal_frames.reserve(input.portals.size());
    for (const CarvePortal& portal : input.portals) {
        portal_frames.push_back(portal_frame(portal));
    }
    const bool have_portals = !portal_frames.empty();

    float max_delta = 0.0f;
    size_t modified = 0;

    for (int z = 0; z < heightmap.height; ++z) {
        // A Heightmap's second axis IS the 2D local y of the carve payload, with
        // no sign flip. Both sides of the world mapping negate on their way to
        // render space and nowhere else: TerrainMeshBuilder places cell (ix, iz)
        // at vec3(world_x, h, -world_z) (terrain_mesh_builder.cpp), and the road
        // pipeline places local (x, y) at vec3(x, h, -y) (corridor.cpp), so the
        // two coincide exactly when the heightmap row value equals the local y.
        // Negating here mirrors the whole network about y = 0; the error is zero
        // at the origin and grows with |y|, so it is invisible on symmetric test
        // data.
        const double local_y = static_cast<double>(heightmap.origin.y)
                             + static_cast<double>(z) * static_cast<double>(heightmap.cell_size_z);

        const int grid_y = static_cast<int>(std::floor((local_y - index.min.y) / index.cell_size));
        if (grid_y < 0 || grid_y >= index.height) continue;

        // Heightmap cells are metres and index cells are tens of metres, so a run
        // of samples shares one bin. Cache it rather than re-deriving per sample.
        int cached_grid_x = -1;
        uint32_t span_begin = 0;
        uint32_t span_end = 0;

        for (int x = 0; x < heightmap.width; ++x) {
            const double world_x = static_cast<double>(heightmap.origin.x)
                                 + static_cast<double>(x)
                                 * static_cast<double>(heightmap.cell_size_x);

            const int grid_x =
                static_cast<int>(std::floor((world_x - index.min.x) / index.cell_size));
            if (grid_x < 0 || grid_x >= index.width) continue;

            if (grid_x != cached_grid_x) {
                cached_grid_x = grid_x;
                const size_t c = static_cast<size_t>(grid_y) * static_cast<size_t>(index.width)
                               + static_cast<size_t>(grid_x);
                span_begin = index.cell_starts[c];
                span_end = index.cell_starts[c + 1];
            }
            if (span_begin == span_end) continue;

            const glm::dvec2 p{world_x, local_y};
            const float natural = heightmap.data[static_cast<size_t>(z)
                                                 * static_cast<size_t>(heightmap.width)
                                                 + static_cast<size_t>(x)];

            // Overlap resolution: the winner is the footprint this cell is
            // nearest to in NORMALISED distance -- distance past the footprint
            // edge measured in units of that item's own blend band. That is the
            // same ordering as "greatest carve weight", but it stays a strict
            // order inside the footprints too, where every weight is 1: a cell
            // deep inside a junction beats a cell barely inside an arm. Ribbons
            // and junction polygons rank against each other on exactly this scale,
            // so an arm hands over to its junction where the two footprint edges
            // are equidistant and neither kerb gets a step.
            //
            // Not an average. Averaging a junction against an arm at a different
            // height produces a surface that matches neither, with a step at both
            // kerbs. Not first-found either, which makes the result depend on
            // index order.
            double best_norm = std::numeric_limits<double>::max();
            float best_height = 0.0f;
            bool found = false;
            bool saw_portal = false;

            for (uint32_t k = span_begin; k < span_end; ++k) {
                const uint32_t item = index.items[k];
                if (!bounds_contain(index.bounds[item], p)) continue;

                // A portal is a CEILING on the ground, not a height for it, so it
                // cannot be ranked against a ribbon or a disc on the normalised
                // distance below. It is applied after this loop, against whatever
                // this loop decided. See the portal note on carve_terrain().
                if (input.is_portal(item)) {
                    saw_portal = true;
                    continue;
                }

                double dist = 0.0;
                double half_extent = 0.0;
                float surface = 0.0f;

                if (input.is_disc(item)) {
                    const CarveDisc& disc = input.discs[input.disc_index(item)];
                    if (disc.suppress) continue;

                    // Radial reject before the ring walk. Every point a junction
                    // can influence lies within its reach of the centre, whichever
                    // footprint it uses, so this trims the corners the box test
                    // leaves in for the cost of one length.
                    if (glm::length(p - disc.center) > index.reach[item]) continue;

                    // A junction is planar, so one flat height serves the whole
                    // footprint however that footprint is shaped.
                    surface = disc.height;

                    if (index.polygon_footprint[item] != 0) {
                        // Real fillet-and-curb-ring polygon. The signed distance
                        // to the ring -- negative inside -- is the exact analogue
                        // of `distance - half_width` on a ribbon, so the footprint
                        // edge sits at zero and half_extent is already folded in.
                        const RingHit ring = ring_hit(disc.outline, p);
                        if (!ring.valid) continue;

                        dist = ring.inside ? -ring.dist : ring.dist;
                        half_extent = 0.0;
                    } else {
                        // Degenerate junction: no polygon was solved, so the
                        // provisional disc is all there is.
                        dist = glm::length(p - disc.center);
                        half_extent = static_cast<double>(disc.radius);
                    }
                } else {
                    const CarveRibbon& ribbon = input.ribbons[item];
                    if (ribbon.suppress) continue;

                    const RibbonHit hit = nearest_on_ribbon(ribbon, p, index.reach[item]);
                    if (!hit.valid) continue;

                    dist = std::sqrt(hit.dist2);

                    // The corridor edge is half_width * miter_scale from the
                    // centerline, so the footprint the band starts at is too. The
                    // interpolated miter never exceeds the max the item was
                    // binned with, so the single-cell query stays complete.
                    half_extent = static_cast<double>(ribbon.half_width) * hit.miter;
                    surface = hit.height;
                }

                // Never for the polygon path, whose signed distance already
                // measures from the footprint edge and whose negative values are
                // exactly what ranks a cell deep inside a junction above a cell
                // barely inside an arm.
                half_extent = std::max(0.0, half_extent);

                // Embankment slope limit. Closing the height difference inside
                // falloff_metres would need a gradient of delta / falloff; where
                // that exceeds max_embankment_slope the band is WIDENED so the
                // gradient comes back to the limit, rather than left narrow and
                // allowed to stand up as a cliff.
                const double delta = std::abs(static_cast<double>(surface)
                                              - static_cast<double>(natural));
                double band = falloff;
                if (slope > 0.0) {
                    band = std::max(band, delta / slope);
                }
                band = std::clamp(band, kMinBand, std::max(max_band, kMinBand));

                const double norm = (dist - half_extent) / band;
                if (norm >= 1.0) continue;

                if (norm < best_norm) {
                    best_norm = norm;
                    best_height = surface;
                    found = true;
                }
            }

            if (!found && !saw_portal) continue;

            // Blend. Weight is 1 across the footprint, then smoothsteps to 0 at
            // the outer edge of the band.
            float carved = natural;
            if (found) {
                const double weight = 1.0 - smoothstep01(best_norm);
                carved = static_cast<float>(static_cast<double>(natural)
                         + (static_cast<double>(best_height)
                            - static_cast<double>(natural)) * weight);
            }

            // ── Tunnel portal mouths ──────────────────────────────────────
            // A ceiling, applied to what the pass above produced, never a
            // competitor with it. Each mouth is measured against `carved` -- the
            // height BEFORE any mouth touched this cell -- and the ceilings are
            // combined with min(), so two overlapping mouths give the same answer
            // whichever order the index happened to list them in.
            if (saw_portal && have_portals) {
                const double base = static_cast<double>(carved);
                double lowest = base;

                for (uint32_t k = span_begin; k < span_end; ++k) {
                    const uint32_t item = index.items[k];
                    if (!input.is_portal(item)) continue;
                    if (!bounds_contain(index.bounds[item], p)) continue;

                    const size_t pi = input.portal_index(item);
                    if (pi >= portal_frames.size()) continue;
                    const PortalFrame& frame = portal_frames[pi];
                    if (!frame.valid) continue;

                    // Radial reject before the rectangle test, on exactly the
                    // reach this portal was binned with, so the mouth can never
                    // reach a cell it was not indexed into.
                    if (glm::length(p - frame.face) > index.reach[item]) continue;

                    const double crown = static_cast<double>(frame.crown);

                    // Same slope-limited band every other cut in this file uses,
                    // so the sides of the notch are no steeper than an embankment.
                    double band = falloff;
                    if (slope > 0.0) {
                        band = std::max(band, std::abs(crown - base) / slope);
                    }
                    band = std::clamp(band, kMinBand, std::max(max_band, kMinBand));

                    const double norm = mouth_signed_distance(frame, p) / band;
                    if (norm >= 1.0) continue;

                    // 0 inside the mouth, 1 at the outer edge of the band, so the
                    // ceiling runs from the crown to the unchanged height with a
                    // matched slope at both ends.
                    const double ceiling = crown + (base - crown) * smoothstep01(norm);
                    lowest = std::min(lowest, ceiling);
                }

                // Never upward. A mouth on ground already below the arch has
                // nothing to cut, and a `set` here would trench the approach.
                if (lowest < base) {
                    carved = static_cast<float>(lowest);
                }
            }

            const float change = carved - natural;
            if (std::abs(change) <= kChangeEpsilon) continue;

            heightmap.data[static_cast<size_t>(z) * static_cast<size_t>(heightmap.width)
                           + static_cast<size_t>(x)] = carved;
            ++modified;
            max_delta = std::max(max_delta, std::abs(change));
        }
    }

    stats.cells_modified = modified;
    stats.max_delta = max_delta;
    return finish();
}

} // namespace stratum::procgen
