// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file op_roof.hpp
 * @brief D5: `roof` -- one operation that raises six roof shapes over a footprint
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ================================================================================
 * ONE OPERATION, NOT FIVE
 * ================================================================================
 *
 * CGA spells these `roofGable`, `roofHip`, `roofShed`, `roofPyramid` and
 * `roofDome`. This language spells all of them `roof(kind, ...)`, and ast.hpp's
 * catalogue has one row for them:
 *
 *     {"roof", 1, 4, "Raise a roof: gable, hip, pyramid, ridge, shed or dome"}
 *
 * The reason is decision Q3: conforming to CGA makes CGA's limits our ceiling.
 * Five fixed names can only ever raise one of five roofs, chosen when the rule
 * was TYPED. One operation that takes the shape as DATA can raise a roof the
 * rule COMPUTED -- which is the thing a game-focused city generator is for:
 *
 *     rule Roof {
 *         roof(random.pick(["gable", "hip", "shed"]), 25.0 + random.range(0, 15));
 *     }
 *
 * That rule is one line here and is not expressible at all with five names. The
 * second reason ast.hpp gives is maintenance: five names means five places to
 * add an overhang argument, and four of them will be forgotten.
 *
 * ================================================================================
 * THE SIGNATURE
 * ================================================================================
 *
 *     roof(kind)                            pitch 30 degrees, no overhang
 *     roof(kind, pitch)                     pitch in degrees, strictly 0 to 90
 *     roof(kind, pitch, overhang)           eave oversail in scope units
 *     roof(kind, pitch, overhang, extra)    one per-kind number; see below
 *
 * **Pitch, not rise.** Every height in this file is `run * tan(pitch)`, where
 * `run` is the horizontal distance from the eave to the point. One rule for six
 * shapes, so a hip and a gable at the same pitch have the same ridge height and
 * a dome at 45 degrees is a hemisphere. A rule that knows a RISE and not an
 * angle writes `roof("hip", atan(rise / run))` -- one conversion at the call
 * site is cheaper than two meanings for one argument, which is the mistake
 * `offset` avoided by having one sign convention instead of two names.
 *
 * **Overhang** offsets the outline the roof is raised on, and only that outline:
 * the walls stay where they are. It is therefore not `offset(d); roof(...)`,
 * which moves the walls too. A positive overhang oversails, and the horizontal
 * ring between the wall top and the eave is emitted as a SOFFIT facing down, so
 * the solid stays closed. A negative overhang insets the roof, and the same ring
 * is emitted facing up -- a parapet ledge. An overhang that folds the outline
 * through itself is refused by name rather than emitted as a knot.
 *
 * **The fourth argument** is the one number a kind needs beyond the pitch, and
 * what it means depends on the kind. That is unusual enough to be worth the
 * warning it earns: a kind that has no use for it REPORTS the argument rather
 * than ignoring it, because a silently ignored argument looks exactly like a
 * working one.
 *
 *     gable, ridge   ridge direction, degrees anticlockwise from the scope's +x
 *                    in its xz plane. Default: the footprint's principal axis.
 *     shed           downhill direction, same convention. Default: across the
 *                    principal axis.
 *     dome           latitude band count, rounded, clamped to 2..64. Default 6.
 *     hip, pyramid   nothing. Supplying one is reported.
 *
 * ================================================================================
 * THE SIX SHAPES, AND WHICH ONE IS EXACT
 * ================================================================================
 *
 *   - **hip** -- the straight skeleton of the footprint, lifted. Every contour
 *     edge gets one sloped plane, and the skeleton's inner arcs ARE the ridges
 *     and the hips. Exact over any simple footprint, convex or not.
 *   - **gable** -- the same, with two contour edges turned into vertical gable
 *     walls. See below: it is the straight skeleton of the footprint with those
 *     edges pushed out of reach, clipped back.
 *   - **pyramid** -- every contour edge rises to ONE apex, over the skeleton's
 *     deepest point at `max_time * tan(pitch)`. On a square this is the same
 *     solid as the hip, which is the correct answer and is asserted in the tests.
 *     It needs the footprint to be STAR-SHAPED from that apex, which is a
 *     stronger condition than the apex being inside the plan: see below.
 *   - **ridge** -- the cheap one, and the one whose limits have to be stated. Every
 *     contour edge rises to its projection on a ridge segment spanning the
 *     footprint along the ridge direction. It is a PROJECTION and not a lower
 *     envelope, and the difference is not a rounding: it is EXACT for a
 *     rectangular footprint, where it agrees with `gable` vertex for vertex and
 *     the tests assert that, and on any other plan it OVER-COVERS -- a slope
 *     reaches the ridge across ground that belongs to another slope. Over-covering
 *     is REPORTED, with the covered area and the footprint's own, so an author
 *     who wanted `gable` is told rather than shown. It is here for two reasons.
 *     It is the construction osm/mesh_builder.cpp uses for `roof:shape=gabled`,
 *     so a rule-generated terrace and an imported one have the same silhouette;
 *     and it calls no skeleton, so it still answers for a footprint the skeleton
 *     refuses. **For anything but a rectangle, `gable` is the one that is right.**
 *   - **shed** -- one tilted plane. The only kind that carries interior rings.
 *   - **dome** -- the footprint shrunk toward the skeleton's deepest point over a
 *     quarter-sine profile, `rings` bands high. A true dome over an arbitrary
 *     polygon is not defined; this is a swept version of the outline, which is
 *     what osm/mesh_builder.cpp settled on for the same question in E1. It needs
 *     the same star-shapedness `pyramid` does, and for the same reason: a band is
 *     the outline scaled about the centre.
 *
 * ### Star-shaped, not merely inside: the limit on `pyramid` and `dome`
 *
 * Both raise the whole footprint to one point, so both need that point to SEE
 * every contour edge -- to lie in the polygon's KERNEL, the intersection of its
 * edges' inner half-planes. The skeleton's deepest node is always inside the
 * plan, and inside is weaker: on a U, a Z or a comb the deepest node sits in one
 * arm and the far arm is round a corner from it. A triangle raised from the far
 * arm then crosses the notch -- ground that is not the building -- and overlaps
 * the triangles beside it.
 *
 * That failure is invisible to everything else this file guarantees. The solid
 * stays closed, every face stays flat, and the volume is a plausible number. So
 * the kernel test is made explicitly, before a face is emitted, and a footprint
 * that fails it is raised as a HIP and reported -- the same degradation
 * `gable` makes for an end it cannot push, and for the same reason: a visible,
 * named, differently-shaped roof beats a solid that passes every assertion and
 * is wrong.
 *
 * ### The ridge line is one polyline, shared edge for edge
 *
 * `ridge` projects each contour edge's two endpoints, and off a rectangle the
 * two sides of the ridge produce different sets of projections. Joining each
 * edge straight across to its own two points therefore left T-JUNCTIONS along
 * the ridge: one slope's top edge ran past the end of the slope facing it. A
 * T-junction is closed until something welds, displaces or decimates it, and
 * both the AO baker and the chunked export want the adjacency. So every
 * projection is a vertex of the whole ridge line, and each slope is emitted
 * through the ones inside its own span. This does not make `ridge` a lower
 * envelope, and it is not meant to: a plan whose slopes genuinely overlap still
 * puts more than two faces on a ridge segment, and that is what the
 * over-covering note is for.
 *
 * ### The two answers osm/mesh_builder.cpp got wrong the first time
 *
 * Both are recorded in its comments and both are honoured here. **A hip has a
 * RIDGE, not an apex** -- an apex is a pyramid, and a hipped roof over anything
 * longer than it is wide has a ridge segment. The skeleton produces that without
 * being asked. **A gable end is a VERTICAL triangle, not a sloped one** -- the
 * gable wall in this file is built in the vertical plane through its contour
 * edge, from the eave up to the roof profile above it, and it is the one face in
 * a gable whose normal is horizontal.
 *
 * ================================================================================
 * WHY THE STRAIGHT SKELETON, AND WHAT HAD TO BE ROUTED AROUND
 * ================================================================================
 *
 * `src/geometry/straight_skeleton.hpp` exists, is tested, and was put in
 * `src/geometry` rather than under `osm/road` for exactly this consumer: its own
 * header says "a gabled or hipped roof over an arbitrary footprint IS the
 * straight skeleton of that footprint, lifted". A second implementation here
 * would be free to disagree with the lot subdivision's copy by an epsilon, and
 * nobody would find out until a building overhung its own parcel. So hip, gable,
 * pyramid and dome all call it and none of them re-derives it.
 *
 * Three of its stated limits shape this file:
 *
 *   - **It does not do holes.** Its header is emphatic that handing it the outer
 *     ring of a polygon with a courtyard gives a skeleton of the SOLID polygon,
 *     and that the faces over the courtyard are "land that does not exist". So
 *     hip, gable, pyramid, ridge and dome REFUSE a footprint with interior rings
 *     and say which kind does carry them (`shed`) and which operation removes
 *     them (`delete_holes`). Roofing over a courtyard silently is the one answer
 *     that leaves no trace.
 *   - **It does not do weighted skeletons**, so every edge moves at one speed and
 *     a roof built from it has ONE pitch. A mansard, whose lower slope is steeper
 *     than its upper, is therefore not one of the six and is not faked.
 *   - **It refuses what it cannot finish** (`StraightSkeleton::complete`) rather
 *     than returning a self-intersecting answer. Every refusal here quotes its
 *     stats, so "the roof did not build" says whether the ring crossed itself,
 *     ran out of iterations, or had no area.
 *
 * ### The gable, which the skeleton does not do directly
 *
 * A gabled roof is the lower envelope of the edge planes with the gable edges'
 * planes REMOVED: the ridge then runs out to the end wall instead of hipping
 * down. There is no "drop this edge" input to a straight skeleton, and a
 * weighted skeleton -- edge moving at infinite speed -- is exactly what the
 * header says is not supported.
 *
 * What is done instead is equivalent and needs no new solver: **push the gable
 * edge outward by more than the footprint's diameter, skeletonise the enlarged
 * footprint, and clip the result back with the gable edge's own line.** A plane
 * that starts more than a diameter away can never be the lowest anywhere over
 * the original footprint, so it is removed in effect; the remaining planes are
 * untouched, because pushing one edge along its own normal moves its neighbours'
 * ENDPOINTS and not their supporting lines. The clip is a half-plane clip and so
 * is exact arithmetic, not a polygon boolean, which keeps a gable as reproducible
 * as a hip.
 *
 * That trick needs the whole footprint to lie on one side of the gable edge's
 * line, which every end wall of a rectangle, an L or a T satisfies and a notch
 * does not. An edge that fails the test is REPORTED and stays hipped, so a gable
 * over an awkward plan degrades to a visible, named, partly-hipped roof rather
 * than to a fold.
 *
 * ================================================================================
 * WHERE THE ROOF SITS
 * ================================================================================
 *
 * On the SHAPE, never on the world. shape.hpp's first invariant stores geometry
 * in scope-local coordinates, so this file works entirely in the shape's own
 * frame: the roof rises along local +y and the ridge follows the footprint's own
 * principal axis. A building placed on a lot at 40 degrees carries that rotation
 * in `Scope::axes`, its local coordinates are untouched, and its roof comes out
 * aligned to the building. A world-space construction would have pointed every
 * ridge at north.
 *
 * The footprint is every face whose local normal is within about a degree of
 * +y. A shape with no such face is refused with the fix in the message: a face
 * component picked out by `select face { top : ... }` has its own normal along
 * local +z, by op_comp.hpp's frame convention, and `align_scope("geometry")`
 * turns that frame so this operation can see it.
 *
 * The removed footprint face is NOT replaced with a floor when the shape was
 * closed there -- an extruded box's top cap is interior once a roof is on it, and
 * emitting it as well would leave a partition inside the building and make
 * geometry_volume() wrong by the cap's contribution. A FREE-STANDING footprint --
 * one whose boundary edges are shared with no other face, which is what
 * `shape_from_polygon()` produces -- does get a floor, or the roof would be an
 * open shell. The test is geometric (shared boundary edges), not a flag.
 *
 * ================================================================================
 * DETERMINISM
 * ================================================================================
 *
 * Same footprint, same arguments, byte-identical geometry on any machine, which
 * is shape.hpp's rule and straight_skeleton.hpp's rule and therefore has to be
 * this file's rule too. What that costs here:
 *
 *   - **libm touches a coordinate through the pitch, once per call.** `tan` comes
 *     from shape.hpp's deg_sin_cos(), which is exact on the quarter turns; every
 *     coordinate afterwards is that one number through +, -, *, / and sqrt. A
 *     pitch of 45 degrees is therefore bit-identical everywhere, and a pitch of
 *     31 degrees is identical to within one ulp of the platform's `tan`.
 *   - **`dome` is the exception and says so.** Its band profile needs a sine and
 *     a cosine per band, at angles that are not quarter turns, so a dome is the
 *     one kind whose last bits are a C library's business. It is called out here
 *     rather than discovered from a golden test that differs between CI and a
 *     developer's machine.
 *   - **No container is read in iteration order.** Edge selection scans vectors,
 *     ties are broken by the lowest index, and the shared-edge test that decides
 *     whether a footprint is free-standing keys a std::set on quantised integers
 *     rather than on doubles -- an epsilon compare is not transitive and a sort
 *     given a non-transitive comparator is undefined behaviour, not merely an
 *     odd order.
 */

#pragma once

#include "geometry/straight_skeleton.hpp"
#include "procgen/rules/interpreter.hpp"
#include "procgen/rules/shape.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace stratum::procgen::rules {

// ============================================================================
// Kinds
// ============================================================================

/**
 * @brief Which roof `roof(kind, ...)` raises
 *
 * Six, matching the catalogue row's summary word for word. The order is the
 * order the summary lists them in, so a reader comparing the two does not have
 * to hold a mapping in their head.
 */
enum class RoofKind : uint8_t {
    Gable,    ///< Two slopes to a ridge, vertical walls at the ends
    Hip,      ///< Every edge slopes; the straight skeleton, lifted
    Pyramid,  ///< Every edge rises to one apex
    Ridge,    ///< Every edge rises to its projection on a ridge segment
    Shed,     ///< One tilted plane. The only kind that carries interior rings.
    Dome      ///< The outline swept over a quarter-sine profile
};

/// Spell a kind the way the author writes it
[[nodiscard]] const char* roof_kind_name(RoofKind kind);

/**
 * @brief Parse a kind name written by the author
 * @param text One of gable, hip, pyramid, ridge, shed, dome; case-sensitive
 * @param out  Receives the kind when the name is known
 * @return false when @p text names no kind; @p out is untouched
 */
[[nodiscard]] bool parse_roof_kind(const std::string& text, RoofKind& out);

// ============================================================================
// Parameters
// ============================================================================

/// Default pitch when the author gives only a kind. A domestic roof.
inline constexpr double kDefaultRoofPitchDegrees = 30.0;

/// Default latitude band count for a dome. Enough to read as curved at city scale.
inline constexpr int kDefaultDomeBands = 6;

/**
 * @brief Everything `roof` was asked for
 *
 * Separated from the handler so that the geometry is testable without an
 * interpreter, a rule file or a parse -- which is how every other operation
 * family in this tree is arranged and is why op_split.cpp's solver has tests
 * that do not go through a rule at all.
 */
struct RoofParams {
    RoofKind kind = RoofKind::Hip;

    /// Slope in degrees. Strictly between 0 and 90; both ends are refused, since
    /// 0 is a flat roof (which is `roof` doing nothing) and 90 is a wall.
    double pitch_degrees = kDefaultRoofPitchDegrees;

    /// Eave oversail in scope units. Negative insets. Zero leaves the outline alone.
    double overhang = 0.0;

    /// True when the author supplied a fourth argument
    bool has_extra = false;

    /// The fourth argument. Its meaning depends on @p kind; see the file header.
    double extra = 0.0;
};

// ============================================================================
// Report
// ============================================================================

/**
 * @brief What the roof did, for a diagnostic and for a test
 *
 * Carried for the reason road::BlockStats and ComponentSplitReport are: a roof
 * that came out wrong still looks like a list of faces. These numbers are what
 * says which path the code took -- whether the skeleton ran, whether a gable end
 * was refused and hipped instead, whether a band count was clamped.
 */
struct RoofReport {
    /// Height of the highest roof point above the eave, in scope units
    double rise = 0.0;

    /// `tan(pitch)`. Every height in the roof is this number times a run.
    double pitch_tangent = 0.0;

    size_t footprints = 0;    ///< Upward faces that were roofed
    size_t roof_faces = 0;    ///< Faces the roof added, gable walls and soffits included
    size_t gable_walls = 0;   ///< Vertical gable walls built
    size_t holes_carried = 0; ///< Interior rings carried into the roof. `shed` only.

    /// True when compute_straight_skeleton() was called and finished
    bool used_skeleton = false;

    /// Stats of the LAST skeleton run, empty when @p used_skeleton is false
    stratum::geometry::SkeletonStats skeleton{};

    /**
     * @brief Things the author should know that did not stop the roof
     *
     * A gable end that could not be gabled and stayed hipped, a pyramid or a
     * dome over a plan its apex could not see and that was hipped instead, a
     * ridge that covers more ground than the footprint has, a band count that
     * was clamped, a fourth argument a kind has no use for. The handler turns
     * each into a Severity::Warning at the line that called `roof`, because the
     * alternative -- doing something other than what was asked and saying
     * nothing -- is how a rule file grows an argument that does nothing.
     *
     * Each is a sentence fragment with no trailing stop, in the same form the
     * lexer and parser use.
     */
    std::vector<std::string> notes;
};

// ============================================================================
// The operation
// ============================================================================

/**
 * @brief Replace each of the shape's upward faces with a roof mass
 *
 * Every face whose local normal is within about a degree of +y is a footprint:
 * it is removed and the roof raised in its place. Every other face is kept, so
 * `extrude(8.0); roof("hip", 30.0);` is a building with a roof on it and not a
 * roof floating over nothing.
 *
 * On failure the shape is left UNTOUCHED, unlike scale_shape(), which applies
 * what it can. A half-built roof is not a partial answer -- it is a hole in a
 * building -- so this function builds the whole new geometry beside the old one
 * and swaps it in only once every footprint has succeeded.
 *
 * @param shape  Shape to roof, modified in place on success
 * @param params Kind, pitch, overhang and the per-kind fourth argument
 * @param report Receives what happened. Cleared first.
 * @return failure when the pitch is outside (0, 90), when the shape has no
 *         upward face, when a footprint has interior rings a kind cannot carry,
 *         when the overhang folds the outline, when the footprint is too small
 *         to raise anything on, or when the straight skeleton refused the ring
 */
[[nodiscard]] OpResult roof_shape(Shape& shape, const RoofParams& params, RoofReport& report);

// ============================================================================
// Registration
// ============================================================================

/**
 * @brief Add `roof` to @p table
 *
 * Additive, and separate from roof_operations(), because several operation
 * families land in this tree at once and each has to register into one table
 * without knowing what the others put there. A later registration of the same
 * name wins, which is OperationTable's documented behaviour.
 *
 * `roof` is a row that already exists in ast.hpp's kBuiltinOperations, so
 * nothing in the parser changes: before this call the name parses and reports
 * "operation 'roof' is not implemented in this build" at the line that used it,
 * and after it the name runs.
 */
void register_roof_operations(OperationTable& table);

/// standard_operations() plus register_roof_operations(), built once
[[nodiscard]] const OperationTable& roof_operations();

} // namespace stratum::procgen::rules
