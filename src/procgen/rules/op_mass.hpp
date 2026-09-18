// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file op_mass.hpp
 * @brief Mass modelling (E2): a lot becomes a building VOLUME, before any facade
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ================================================================================
 * WHAT THIS IS FOR
 * ================================================================================
 *
 * D2 gave the language `extrude`, and `extrude` makes exactly one shape: a box.
 * A city of boxes is what every OSM viewer already produces, and the product
 * this tree is aiming at is a CityEngine for game developers, not a viewer. The
 * three operations here are what stands between the two:
 *
 *   - `floors(n, h)` -- a building of n storeys of h metres, rather than a block
 *     `extrude(n * h)` tall. The total height is the same number. What is
 *     different is that the shape afterwards KNOWS it has n floors, and a facade
 *     rule can cut exactly n of them without the storey height being written out
 *     twice in two files that will drift apart.
 *   - `podium(base, tower, inset, steps)` -- a base with an inset mass above it.
 *     This is the operation that stops every generated building being one
 *     extruded box, and it is the difference between a skyline and a warehouse
 *     estate.
 *   - `courtyard(depth)` -- a footprint with a hole carried through the massing,
 *     rather than filled in. `osm::road::Block::holes` and `osm::Building::holes`
 *     both already produce holes; without this they are lost the moment a rule
 *     touches the lot.
 *
 * All three run on a FOOTPRINT and produce a mass. Everything downstream --
 * `select face`, `split`, the facade -- runs on what they build.
 *
 * ================================================================================
 * THE ONE PRECONDITION: A FOOTPRINT, AND WHY IT IS CHECKED RATHER THAN COPED WITH
 * ================================================================================
 *
 * Every operation here requires a shape with geometry whose scope is FLAT in y:
 * `scope.size.y <= kMassFlatTolerance`. A shape that has already been extruded
 * is refused, by name, at the line that called it.
 *
 * The alternative -- silently doing something reasonable to a solid -- was
 * rejected because there is no one reasonable thing. `floors(6, 3)` on a solid
 * that is already 18 metres tall could mean "make it 18 tall" (a no-op that
 * hides the mistake), "add 18 more" (an operation nobody asked for), or "record
 * that the 18 you have is six floors" (which is `set`, and is spelled `set`).
 * Refusing costs the author one diagnostic at the right line. Guessing costs
 * them a building that is wrong in a way no single shape looks wrong.
 *
 * `fail_shape()` abandons the SUBTREE and not the run, so one lot that reached a
 * mass operation in the wrong state drops out and the other four thousand are
 * generated. That granularity is what makes refusal affordable here.
 *
 * ================================================================================
 * THE SCOPE: DERIVED, NEVER SET
 * ================================================================================
 *
 * shape.hpp's second invariant is that the scope box is TIGHT around the
 * geometry with its minimum at local zero, and it warns that an operation which
 * leaves a stale box does not fail where it is written -- it fails in a later
 * `split` that cuts empty air.
 *
 * Nothing in this file assigns `Scope::size` or `Scope::origin`. Each operation
 * builds geometry and calls `refit_scope()`, which derives both. The axes and
 * the pivot fraction are untouched, so `split(y)` still means up and a lot
 * placed at 40 degrees still has its own front wall. Three consequences a reader
 * should expect:
 *
 *   - After `floors`, `scope.size.y` is EXACTLY `FloorPlan::total_height()`.
 *     That equality is what makes the split in the next section come out at
 *     exactly n storeys, and it is asserted rather than assumed.
 *   - After `podium`, `scope.size.x` and `.z` are the PODIUM's extent, not the
 *     tower's, because the podium is the widest tier and the box is tight around
 *     the whole mass.
 *   - After `courtyard`, the scope does not change at all. A hole is interior;
 *     the outline it was cut from is still the bounds.
 *
 * ================================================================================
 * WHAT "KEEPING THE FLOOR STRUCTURE" ACTUALLY MEANS
 * ================================================================================
 *
 * The obvious reading of "stack N floors" is N solids, one per storey. That was
 * rejected, and the reason is what the whole feature turns on.
 *
 * A facade is applied to a WALL. `select face { front : ... }` on a stack of N
 * boxes finds N separate front walls with a seam and a buried cap at every
 * storey, so a shopfront cannot span two floors, a vertical pier cannot run past
 * one, and the terminals triple. Worse, the horizontal faces where box k meets
 * box k+1 are inside the solid and answer to `top` and `bottom` just as the roof
 * and the ground do.
 *
 * So `floors` produces ONE solid, exactly as `extrude` would, and the floor
 * structure survives as three inherited attributes:
 *
 *     floor_count     the number of storeys            (kFloorCountAttribute)
 *     floor_height    the height of an upper storey    (kFloorHeightAttribute)
 *     ground_height   the height of the ground storey  (kGroundHeightAttribute)
 *
 * Attributes inherit from a shape to its children, and both a `select`
 * component and a `split` slab are children, so the numbers reach the facade
 * rule unchanged. There they are read with `attrs.get`, and the split is exact:
 *
 * @code
 *     rule Main   { floors(6, 3.0, 4.0); select face { front : { Facade(); } } }
 *     rule Facade {
 *         split(y) {
 *             attrs.get("ground_height", 3.0) : { Shopfront(); }
 *             repeat { attrs.get("floor_height", 3.0) : { UpperFloor(); } }
 *         }
 *     }
 * @endcode
 *
 * `floors(6, 3.0, 4.0)` makes the wall `4 + 5 * 3 = 19` metres tall. The
 * shopfront takes 4 and leaves 15; op_split.hpp's repeat group has a period of 3
 * and `floor(15 / 3) = 5` copies of exactly 3. Six storeys in, six storeys out,
 * with no number written twice. THAT is what stacking buys over extruding, and
 * test_op_mass.cpp asserts it through a real rule file rather than by inspecting
 * the attribute map.
 *
 * The arithmetic is exact rather than nearly exact, which matters because
 * op_split.hpp's copy count is a `floor()` and a total that came out a hair
 * short would yield n-1 storeys and a fat one. `total_height()` is
 * `ground + (count - 1) * upper` and the split's remainder is
 * `total - ground`, computed from the same two doubles, so the division is by a
 * value that was built by multiplying by it.
 *
 * ================================================================================
 * `podium` IS NOT `setback`, AND THE DIFFERENCE IS THE AXIS
 * ================================================================================
 *
 * `setback(d)` already exists in shape.hpp and is the PLANAR CGA setback: it
 * insets an outline within its own plane and hands the removed border back as a
 * child shape. It is what a rule uses to hold a building off the street.
 *
 * `podium` is a VERTICAL step. Nothing is handed back, because nothing is
 * removed: the ring between the podium outline and the tower outline is not a
 * discarded border, it is the podium's ROOF TERRACE, and it is a face of the
 * solid that `select face { top : ... }` finds beside the tower roof. A rule
 * wanting to pave it selects it; a rule wanting a plaza at ground level uses
 * `setback`. Confusing the two produces a building with its terraces at street
 * level, which is why they are named differently and why this paragraph exists.
 *
 * `podium(base, tower, inset, steps)` with `steps > 1` is a wedding cake: each
 * tier is inset by `inset` from the one below and the tower height is divided
 * equally between them. `steps == 1` is the plain podium-and-tower that the
 * catalogue row names, and it is the default.
 *
 * ================================================================================
 * A COURTYARD IS A VERTICAL SHAFT
 * ================================================================================
 *
 * The hole does NOT move when a tier steps back. The outer outline insets; the
 * courtyard rings are the same rings at every level, from the ground to the
 * roof.
 *
 * That is a decision and not an oversight, and it buys the geometry its
 * simplicity. If the holes grew with the outline, the horizontal face at a tier
 * boundary would be two disjoint annuli -- one between the two outlines and one
 * between the two courtyards -- and the courtyard wall would be a stack of
 * stepped rings. With the holes fixed, the terrace is exactly one face: the
 * lower outline with the upper outline as its single hole. The derivation is
 * short enough to give in full. Writing `P_k` for tier k's cross-section,
 * `O_k` for its outline and `H` for the fixed holes,
 *
 *     P_k \ P_(k+1) = (O_k \ H) \ (O_(k+1) \ H) = O_k \ O_(k+1)
 *
 * because `H` is contained in `O_(k+1)`. That containment is the condition, and
 * it is CHECKED: a setback that would reach the courtyard fails and says which
 * tier did it. See the edge cases below.
 *
 * Architecturally it is also the right answer. A light well is a shaft; it does
 * not widen as the building steps back.
 *
 * ================================================================================
 * THE FOUR EDGE CASES, AND WHAT EACH ONE DOES
 * ================================================================================
 *
 * ### A setback larger than the footprint
 *
 * `podium(4, 16, 12)` on a twenty-metre-wide lot. Clipper2 returns nothing, and
 * the operation FAILS naming the tier: "the inset at tier 1 of 12 removes the
 * whole outline". It does not clamp the inset to whatever would just fit, because a
 * tower whose setback silently became 9.9 metres is a tower the author cannot
 * reproduce from the rule text.
 *
 * A near miss is caught too: an inset that splits the outline in two -- a
 * dumbbell whose neck the inset eats -- fails rather than building two towers on
 * one podium, because which of the two continues upward is a question the rule
 * did not answer.
 *
 * ### A floor count of zero
 *
 * `floors(0, 3)` FAILS. The alternative, leaving the footprint alone, produces a
 * flat terminal that looks exactly like a rule that forgot to extrude, and the
 * author has nothing to search for. It is the same argument shape.hpp makes for
 * refusing `extrude(0)`, and the blast radius is the same: one lot, not the run.
 *
 * A count is rounded to the nearest whole number, away from zero on a tie, since
 * the language has no integer type -- so `floors(2.5)` is three floors. Above
 * kMaxFloors it fails too: a count in the thousands is a unit error upstream,
 * and the height it computes makes a scope that no later split can use.
 *
 * ### A courtyard that a setback would close
 *
 * `courtyard` itself is PERMISSIVE and `floors` is STRICT, and the asymmetry is
 * deliberate. A lot too narrow to hold a courtyard of the asked depth is a
 * perfectly good solid building, and a rule that says `courtyard(6)` over a
 * whole city must not fail on every small lot. So the footprint is left solid,
 * the CourtyardReport says why, and the operation handler raises a WARNING --
 * visible in the diagnostics, but `GenerationResult::ok()` stays true. A floor
 * count of zero is not conditional in that way: it is the building.
 *
 * `podium` is strict again, for the reason the previous section gives: once a
 * courtyard exists, a tier outline that no longer contains it has no
 * single-face terrace, and the honest answer is a message naming the tier
 * rather than a roof laid over half a light well.
 *
 * ### What a later `split(y)` sees
 *
 * Covered above, and it is the point of the feature. One thing to add: a
 * `select face { front }` on a building WITH a courtyard finds two walls, the
 * street wall and the courtyard's own wall that faces the same way. Both are
 * front walls and both get the facade. That is correct and is asserted in the
 * suite, because the first instinct on seeing twelve terminals where six were
 * expected is to assume a bug.
 *
 * ================================================================================
 * WHY THIS IS NOT IN shape.cpp
 * ================================================================================
 *
 * The same reason op_split.cpp and op_comp.cpp are not: interpreter.hpp's
 * extension contract is "copy a table, register into the copy", and an operation
 * family must be addable without editing the files it plugs into.
 * register_mass_operations() adds these three rows to any OperationTable, and
 * nothing in interpreter.cpp, shape.cpp, op_split.* or op_comp.* changes.
 *
 * What IS reused rather than rewritten: `offset_shape()` for every inset, which
 * is the project's one Clipper2 call site and already handles the topology
 * changes an inset causes; `extrude_shape()` for the single-tier case, which is
 * what `floors` is; and `refit_scope()` for the box. The only new geometry here
 * is the stacked prism, which no existing function produces because no existing
 * function needs a cap with a hole in it.
 */

#pragma once

#include "procgen/rules/interpreter.hpp"
#include "procgen/rules/shape.hpp"

#include <glm/glm.hpp>

#include <cstdint>
#include <string_view>
#include <vector>

namespace stratum::procgen::rules {

// ============================================================================
// Tolerances and defaults
// ============================================================================

/**
 * @brief How much height a shape may have and still be called a footprint
 *
 * A micrometre, the same grid op_comp.hpp quantises its ordering key to and for
 * the same reason: far below any architectural distinction, far above the
 * double-precision noise left by a lot outline that has been offset, split and
 * re-offset on its way here. A footprint draped over terrain and then flattened
 * lands inside it; a shape somebody already extruded does not.
 */
inline constexpr double kMassFlatTolerance = 1e-6;

/// Length below which a distance is treated as zero, in scope units
inline constexpr double kMassLengthEpsilon = 1e-9;

/// Area below which a ring is treated as enclosing nothing, in squared scope units
inline constexpr double kMassAreaEpsilon = 1e-12;

/**
 * @brief Storey height `floors(n)` uses when the rule does not give one
 *
 * Three metres, which is an ordinary residential floor-to-floor. A default is
 * needed because the catalogue row admits one argument; it is a CONSTANT rather
 * than a read of the shape's own `floor_height` attribute, and that was the
 * harder call. An implicit attribute read would make `floors(6)` mean different
 * heights in different subtrees with nothing at the call site saying so. A rule
 * that wants the attribute asks for it: `floors(6, attrs.get("floor_height", 3.0))`.
 */
inline constexpr double kDefaultFloorHeight = 3.0;

/**
 * @brief Setback `podium` uses when the rule does not give one
 *
 * Two metres. Same argument as the floor height: the catalogue row admits two
 * arguments and a podium with no inset is not a podium, so there has to be a
 * number and it has to be written down where the reader can find it.
 */
inline constexpr double kDefaultPodiumInset = 2.0;

/**
 * @brief Most storeys `floors` will stack
 *
 * A thousand, about twice the tallest building ever built. Past it the count is
 * a unit error upstream -- a height in centimetres, a population figure, an OSM
 * tag that is not what it looked like -- and the resulting scope is one no
 * subsequent split can work in.
 */
inline constexpr uint32_t kMaxFloors = 1000;

/**
 * @brief Most tiers `podium` will step
 *
 * Each step is a Clipper2 inset of the previous outline, and each one is
 * narrower, so the useful count is bounded by the footprint long before this.
 * Thirty-two is generous for a wedding cake and small enough that a runaway
 * expression cannot spend a second inside one operation.
 */
inline constexpr uint32_t kMaxPodiumSteps = 32;

// ============================================================================
// The attributes the mass operations leave behind
// ============================================================================
//
// Plain names, not the '#'-prefixed reserved space op_control.hpp uses for tags,
// precisely because a rule is MEANT to read these back with attrs.get() and
// op_control.cpp's is_reserved_name() refuses a reserved one. They live in the
// same namespace a rule's own set() writes to, which means a rule can override
// them; that is the same openness `set` has everywhere else and not a hole.

/// Storeys stacked, as a number. Written by `floors`.
inline constexpr std::string_view kFloorCountAttribute = "floor_count";

/// Height of one upper storey. Written by `floors`.
inline constexpr std::string_view kFloorHeightAttribute = "floor_height";

/// Height of the ground storey, which equals the upper height unless one was given.
inline constexpr std::string_view kGroundHeightAttribute = "ground_height";

/// Height of the podium base. Written by `podium`.
inline constexpr std::string_view kPodiumHeightAttribute = "podium_height";

/// Height of everything above the podium, tiers included. Written by `podium`.
inline constexpr std::string_view kTowerHeightAttribute = "tower_height";

/// Inset applied at each step. Written by `podium`.
inline constexpr std::string_view kPodiumInsetAttribute = "podium_inset";

/// Number of tiers above the podium. Written by `podium`.
inline constexpr std::string_view kPodiumStepsAttribute = "podium_steps";

// ============================================================================
// Footprint
// ============================================================================

/**
 * @brief Is this a shape a mass operation can build on?
 *
 * Geometry present, at least one face, and a scope no taller than
 * kMassFlatTolerance. A shape with no geometry is NOT a footprint: shape.hpp
 * exempts it from the tight-box invariant, so its `size.y` is whatever was set
 * and reading it as flatness would admit an empty scope somebody built to insert
 * an asset into.
 */
[[nodiscard]] bool shape_is_footprint(const Shape& shape);

/**
 * @brief One footprint face as rings in the scope's xz plane
 *
 * The y coordinate is dropped. On a shape that passed shape_is_footprint() every
 * vertex is within kMassFlatTolerance of one plane, so any of them answers for
 * all of them, and @p plane_y records which one was taken -- the face's first
 * vertex, the same choice face_basis() makes.
 *
 * Winding is NORMALISED here, so a caller never has to ask which way round a
 * ring arrived: @p outer is wound so its face normal is +y, and every entry of
 * @p holes is wound the other way, which is Face's contract for a hole.
 */
struct FootprintFace {
    std::vector<glm::dvec2> outer;
    std::vector<std::vector<glm::dvec2>> holes;
    double plane_y = 0.0;
    MaterialKey material{};
};

/**
 * @brief Read a footprint's faces as rings
 *
 * @param shape Shape to read; not modified
 * @param out   Receives one entry per face with a usable outer ring. Cleared
 *              first. A face with fewer than three vertices or no area is
 *              dropped, exactly as extrude_shape() drops it.
 * @return failure when @p shape is not a footprint, or when no face survived
 */
[[nodiscard]] OpResult read_footprint(const Shape& shape, std::vector<FootprintFace>& out);

// ============================================================================
// Ring arithmetic
// ============================================================================

/**
 * @brief Twice the signed area of a ring in the xz plane, sign matching Newell's y
 *
 * POSITIVE for a ring whose face normal is +y, which is the OPPOSITE of the sign
 * the usual shoelace formula gives the same ring. That is not a mistake: as
 * shape.cpp's own winding note puts it, "(x, z) as plotted is a left-handed view
 * of the xz plane", so a ring that looks counter-clockwise on paper faces down.
 * This function returns the Newell term directly, so the sign here and the sign
 * face_normal() computes cannot drift apart.
 */
[[nodiscard]] double ring_newell_y(const std::vector<glm::dvec2>& ring);

/// Area of a ring in the xz plane, never negative
[[nodiscard]] double ring_area(const std::vector<glm::dvec2>& ring);

/**
 * @brief Does @p outer contain @p inner with nothing touching?
 *
 * Three tests:
 *
 *   1. every vertex of @p inner is strictly inside @p outer -- a vertex ON the
 *      boundary is not strictly inside, because a terrace face built against it
 *      has a zero-width neck that earcut triangulates into slivers;
 *   2. no edge of @p inner properly crosses an edge of @p outer, which catches a
 *      ring that leaves and re-enters between two of its own vertices;
 *   3. no vertex of @p outer is strictly inside @p inner.
 *
 * Test 1 alone passes a ring whose vertices are all inside but whose EDGES bulge
 * out between them, which is exactly the shape a miter-limited inset leaves at a
 * reflex corner. Test 2 is what refuses that, and it is the check that stands
 * between a podium and a roof laid over half a light well. test_op_mass.cpp
 * pins it with a U-shaped ring and a bar laid across the two prongs: every
 * vertex of the bar is inside one prong, and only its long edges leave.
 *
 * Test 3 is DEFENCE IN DEPTH and not a case the first two miss, which is worth
 * saying because the obvious reading -- that it catches the two rings handed
 * over the wrong way round -- is wrong. Swapped rings are refused by test 1: if
 * @p inner is the larger ring its vertices are outside @p outer. In fact no
 * input that passes tests 1 and 2 can fail test 3. If every vertex of @p inner
 * lies in @p outer's interior and no edges cross, then all of @p inner's
 * boundary lies in that interior; @p outer's boundary is one connected closed
 * curve disjoint from it, so it is wholly inside or wholly outside the region
 * @p inner encloses, and wholly inside is impossible when @p inner sits within
 * @p outer. A search of forty million random ring pairs on a coarse integer
 * grid -- where touching, collinear and repeated vertices are common -- found no
 * input that reaches it either.
 *
 * It is kept rather than deleted because test 1 answers with a TOLERANCE, not
 * exactly: a vertex that point_on_segment() misses by less than
 * kMassLengthEpsilon is the one input that could still get this far, and the
 * cost of the test is one more pass over two short rings. For the same reason
 * test_op_mass.cpp has no test that reaches it, and says so rather than
 * asserting a refusal that test 1 is really performing.
 *
 * Winding does not matter to any of the three.
 */
[[nodiscard]] bool ring_strictly_contains(const std::vector<glm::dvec2>& outer,
                                          const std::vector<glm::dvec2>& inner);

/**
 * @brief Inset a ring within the xz plane by @p distance
 *
 * A Clipper2 inset, reached through shape.hpp's offset_shape() rather than by
 * calling Clipper2 again here: that function is the project's one offsetting
 * call site, it already carries the integer scale, the miter limit and the
 * canonical ring rotation that make an offset reproducible, and a second call
 * site is a second set of those constants to keep in step.
 *
 * The ring makes a round trip through a scratch Shape, whose scope offset_shape()
 * refits. The result is read back through `Scope::to_world()`, which is the
 * invariant across that refit, so the coordinates returned are in the frame the
 * ring was given in.
 *
 * @param ring     Boundary, first point not repeated. Winding does not matter.
 * @param distance Inset distance; must be positive
 * @param out      Receives the inset ring or rings -- an inset can split a
 *                 dumbbell in two. Cleared first. Holes the inset may have
 *                 produced are ignored: a simple ring cannot grow one, and a
 *                 self-intersecting ring is not a footprint.
 * @return failure when @p distance is not positive or the inset removes
 *         everything
 */
[[nodiscard]] OpResult inset_ring(const std::vector<glm::dvec2>& ring,
                                  double distance,
                                  std::vector<std::vector<glm::dvec2>>& out);

// ============================================================================
// The stacked prism
// ============================================================================

/**
 * @brief One tier of a mass: an outline and the height it reaches
 *
 * @p top is an ABSOLUTE local y, not a thickness, because the builder needs the
 * boundaries and a list of thicknesses would have it accumulate them -- and an
 * accumulated boundary is a boundary whose value depends on every tier below it,
 * which is how a terrace ends up a rounding error away from the wall that meets
 * it.
 */
struct MassTier {
    std::vector<glm::dvec2> outline;
    double top = 0.0;
};

/**
 * @brief Build a closed solid from a stack of tiers with a shared set of holes
 *
 * The geometry a mass model is made of, and the one thing here that no existing
 * function produces: extrude_shape() makes a prism with two solid caps, and a
 * stack needs the cap between tier k and tier k+1 to be an ANNULUS -- the lower
 * outline with the upper one as a hole -- or the buried cap answers to
 * `select face { top }` beside the real roof.
 *
 * Winding follows build_prism() in shape.cpp exactly, because a mass built to a
 * different convention meets an extruded one in the same scene: outlines wound
 * for a +y normal, holes wound against them, side quads `(low i, low j, high j,
 * high i)`, the bottom cap reversed. geometry_volume() is positive for the
 * result and equals the sum of the tier volumes; the suite checks that number
 * against hand arithmetic, which is what catches a single flipped wall.
 *
 * The courtyard walls span the WHOLE stack as one quad per edge rather than one
 * per tier. A shaft that does not step is one wall, and cutting it at every
 * terrace would hand a facade rule seams that are not there in the building.
 *
 * @param tiers    Tiers in ascending order, tier 0 lowest. Each `top` must
 *                 exceed the previous one -- and the first must exceed @p base
 *                 -- by more than kMassLengthEpsilon.
 * @param holes    Rings shared by every tier, wound however; normalised here.
 *                 Each must be strictly inside every tier's outline.
 * @param base     Local y of the underside
 * @param material Material for every face produced. One material for the whole
 *                 mass, taken from the footprint face it grew out of: a stack is
 *                 one building, and a rule that wants the terrace paved
 *                 differently selects it and calls `material` on it.
 * @param out      Receives the faces and positions, APPENDED, so one call per
 *                 footprint face accumulates into one geometry
 * @return failure naming the tier at fault when an outline is degenerate, the
 *         heights do not ascend, or a hole is not inside a tier
 */
[[nodiscard]] OpResult build_mass_stack(const std::vector<MassTier>& tiers,
                                        const std::vector<std::vector<glm::dvec2>>& holes,
                                        double base,
                                        MaterialKey material,
                                        ShapeGeometry& out);

// ============================================================================
// floors
// ============================================================================

/**
 * @brief A resolved floor stack
 *
 * Separate from the operation so the arithmetic can be tested without geometry,
 * and because total_height() is the number the facade split depends on being
 * exact.
 */
struct FloorPlan {
    uint32_t count = 0;          ///< Storeys, at least 1
    double ground_height = 0.0;  ///< The lowest storey
    double upper_height = 0.0;   ///< Every storey above it

    /**
     * @brief The height the mass is extruded to
     *
     * `ground + (count - 1) * upper`, in that association. A facade rule cuts
     * the ground storey off and divides the rest by `upper`, so the remainder it
     * divides is `(count - 1) * upper` -- a product of the very number it
     * divides by, and therefore an exact multiple of it in IEEE-754 whenever the
     * product itself is exact.
     */
    [[nodiscard]] double total_height() const;
};

/**
 * @brief Check and resolve the arguments of `floors`
 *
 * @param count         Storeys, rounded to the nearest whole number
 * @param height        Upper-storey height
 * @param ground_height Ground-storey height; pass @p height when the rule gave none
 * @param out           Receives the plan when the call succeeds
 * @return failure when the count rounds below 1 or above kMaxFloors, or when
 *         either height is not positive
 */
[[nodiscard]] OpResult solve_floor_plan(double count,
                                        double height,
                                        double ground_height,
                                        FloorPlan& out);

/**
 * @brief Stack the floors: extrude the footprint and record the structure
 *
 * The geometry is exactly what `extrude(total)` produces -- one solid, holes
 * carried up as shafts -- and the difference is the three attributes. See the
 * header note on what keeping the floor structure means.
 *
 * @param shape Footprint, modified in place
 * @param plan  Already resolved by solve_floor_plan()
 * @return failure when @p shape is not a footprint or the extrude fails
 */
[[nodiscard]] OpResult floors_shape(Shape& shape, const FloorPlan& plan);

// ============================================================================
// courtyard
// ============================================================================

/// What `courtyard` was asked for
struct CourtyardOptions {
    /**
     * @brief Depth of the building ring left around the courtyard
     *
     * The courtyard is the footprint inset by this much, so it is the wall-to-
     * wall depth of the accommodation and not the size of the hole. A rule
     * author knows how deep a flat can be; nobody knows how big the courtyard
     * should be until the lot is measured.
     */
    double depth = 0.0;

    /**
     * @brief Smallest courtyard worth carving, in squared scope units
     *
     * Below it the footprint is left solid and the operation still succeeds. A
     * two-square-metre light well is a modelling artefact, not a courtyard, and
     * a rule run over a whole city meets hundreds of them.
     */
    double min_area = 0.0;

    /**
     * @brief Leave a footprint that already has holes alone
     *
     * The second half of the catalogue row: "or keep an existing hole open". An
     * OSM building with an inner ring already has its courtyard, and carving a
     * second one inside the ring left by the first is not what `courtyard(6)`
     * asked for. With this false the existing rings are discarded and a fresh
     * courtyard is cut at @p depth.
     */
    bool keep_existing = true;
};

/// What `courtyard` did, per face, for the caller to turn into diagnostics
struct CourtyardReport {
    uint32_t carved = 0;     ///< Faces given a new courtyard
    uint32_t kept = 0;       ///< Faces left alone because they already had holes
    uint32_t too_small = 0;  ///< Faces whose courtyard would be under CourtyardOptions::min_area
    uint32_t no_room = 0;    ///< Faces the inset emptied: the lot is narrower than 2 * depth
    double area = 0.0;       ///< Total courtyard area carved
};

/**
 * @brief Carve a courtyard into a footprint, or keep the one it has
 *
 * Permissive by design: a lot with no room for a courtyard is left solid and the
 * call SUCCEEDS, with the reason in @p report. See the edge-case section of the
 * header for why this one is permissive while `floors` is not.
 *
 * @param shape   Footprint, modified in place
 * @param options What to carve
 * @param report  Receives the per-face outcome; may be nullptr
 * @return failure only when @p shape is not a footprint or @p options.depth is
 *         not positive
 */
[[nodiscard]] OpResult courtyard_shape(Shape& shape,
                                       const CourtyardOptions& options,
                                       CourtyardReport* report);

// ============================================================================
// podium
// ============================================================================

/// A resolved podium-and-tower
struct PodiumPlan {
    double base_height = 0.0;                ///< The podium
    double tower_height = 0.0;               ///< Everything above it, all tiers together
    double inset = kDefaultPodiumInset;      ///< Step-back at each tier
    uint32_t steps = 1;                      ///< Tiers above the podium, at least 1

    /// Height of one tier: the tower divided equally between the steps
    [[nodiscard]] double step_height() const;
};

/**
 * @brief Check and resolve the arguments of `podium`
 *
 * @param base_height  Podium height; must be positive
 * @param tower_height Total height above the podium; must be positive
 * @param inset        Step-back per tier; must be positive, because a podium
 *                     with no inset is an extrude and the terrace face it would
 *                     build has a hole the same size as its outline
 * @param steps        Tiers, rounded to the nearest whole number
 * @param out          Receives the plan when the call succeeds
 */
[[nodiscard]] OpResult solve_podium_plan(double base_height,
                                         double tower_height,
                                         double inset,
                                         double steps,
                                         PodiumPlan& out);

/**
 * @brief Build a podium with an inset mass above it
 *
 * Each footprint face becomes its own stack, the way extrude_shape() gives each
 * face its own prism: it is the only reading that does not require choosing one
 * of them.
 *
 * @param shape Footprint, modified in place
 * @param plan  Already resolved by solve_podium_plan()
 * @return failure when @p shape is not a footprint, when a tier's inset removes
 *         or splits an outline, or when a tier's outline no longer contains the
 *         courtyard. The message names the tier, counting the podium as tier 0.
 */
[[nodiscard]] OpResult podium_shape(Shape& shape, const PodiumPlan& plan);

// ============================================================================
// Registration
// ============================================================================

/**
 * @brief Add E2's operations to @p table
 *
 * `courtyard`, `floors` and `podium`, three rows that already exist in ast.hpp's
 * kBuiltinOperations and that D2 deliberately left unimplemented.
 *
 * Additive, and separate from mass_operations(), because several operation
 * families land in this tree at once and each has to register into one table
 * without knowing what the others put there. A later registration of the same
 * name wins, which is OperationTable's documented behaviour.
 */
void register_mass_operations(OperationTable& table);

/// standard_operations() plus register_mass_operations(), built once
[[nodiscard]] const OperationTable& mass_operations();

} // namespace stratum::procgen::rules
