// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file op_facade.hpp
 * @brief E3: the three operations that turn a split-up wall into a facade
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ================================================================================
 * WHAT THIS IS
 * ================================================================================
 *
 * `split` divides a wall into tiles. This file fills the tiles in. Three
 * operations, all rows that already exist in ast.hpp's kBuiltinOperations and
 * that D2 deliberately left unimplemented:
 *
 *   - `window(inset, reveal, frame, sill, thickness)` -- an opening with a
 *     reveal, a frame, glazing and a sill
 *   - `door(inset, reveal, frame, threshold)` -- the same, standing on the floor,
 *     with a leaf instead of glazing and a threshold instead of a sill
 *   - `wall_panel(inset, thickness)` -- the wall surface between the openings
 *
 * Before this file, the end-to-end facade in
 * tests/procgen/test_rule_statements.cpp produced twelve terminals named
 * `Window` whose geometry was a flat rectangle of nothing. `rule Window {
 * window(); }` is what makes them windows.
 *
 * ================================================================================
 * THE ONE THING THIS FILE EXISTS FOR: THE REVEAL
 * ================================================================================
 *
 * A window drawn flat on the wall plane is the single clearest tell of generated
 * architecture. Real walls have thickness, the glazing sits back from the outer
 * face, and the returns between the two -- the jambs and the head, collectively
 * the REVEAL -- are what a facade reads as depth in raking light. Every
 * decision below is downstream of wanting that depth for free from a bare
 * `window()`.
 *
 * ### Where the wall thickness comes from
 *
 * The reveal is cut from the wall's thickness, and the shape usually has none:
 * after `select face { front : ... }` and two `split`s, the tile is a
 * ZERO-THICKNESS face. `scope.size.z` is exactly 0. So the thickness has to come
 * from somewhere else, and there are three places, tried in this order:
 *
 *   1. **The `thickness` argument**, when the rule passes one. `window(0, 0.1,
 *      0.05, 0.05, 0.45)` is a rule that knows it is dressing a masonry wall.
 *   2. **The `wall_thickness` shape attribute**, which the rule sets once at the
 *      top and every child inherits: `rule Main { set("wall_thickness", 0.45);
 *      ... }`. This is the form a facade rule actually wants, because the
 *      thickness is a property of the BUILDING and the window operation is
 *      written thirty lines further down in a rule that should not have to
 *      repeat it.
 *   3. **kDefaultWallThickness**, 0.3 m, the thickness of an ordinary cavity
 *      wall.
 *
 * The shape's own `scope.size.z` is deliberately NOT one of them. A tile that
 * does have depth got it from an `extrude`, and its depth is then the depth of
 * the whole wall slab -- reading it would make `window()` mean something
 * different depending on whether the rule extruded before or after splitting,
 * which is exactly the kind of order dependence that makes a rule file
 * unreadable. The thickness is named or it is defaulted; it is never inferred.
 *
 * A `wall_thickness` attribute that is not a positive number is a rule bug, so
 * it is REPORTED and the default is used, rather than silently producing a
 * flush window -- a flush window looks like a styling choice and points at
 * nothing.
 *
 * ### What the reveal is, given the thickness
 *
 * The reveal DEPTH defaults to half the wall thickness: the glazing line of a
 * real window sits roughly in the middle of the wall, and half is the only
 * fraction that needs no further justification. An explicit reveal is clamped to
 * the wall thickness, because a reveal deeper than the wall puts the glass
 * outside the building, and the clamp is reported.
 *
 * A reveal of exactly zero is legal and means a flush window. It emits no return
 * quads at all rather than four zero-area ones.
 *
 * ================================================================================
 * THE SCOPE AFTERWARDS
 * ================================================================================
 *
 * A later operation has to see a sensible box, so the scope is handled in two
 * steps and both are part of the contract.
 *
 * **First, the shape is REFRAMED into the panel's own frame**: x across the
 * panel, y up it, z out of the wall along the face normal. shape.hpp's
 * reframe() does it, so the world positions do not move. This is what makes
 * `window()` mean the same thing on a footprint, on a `select face` component
 * and on a wall that a `rotate_scope` has turned: the operation always works in
 * a frame where "up" is up and "out" is out. When the panel is already in that
 * frame -- which is every tile a `select face` / `split` / `split` chain
 * produces -- the frame is bit-identical and reframe() is skipped outright, so
 * the common path introduces no rounding whatsoever.
 *
 * **Then the box refits around everything the operation built.** Under
 * shape.hpp's invariant 2 the box is derived and tight, so after `window()`:
 *
 *   - `scope.size.x` is still the panel width.
 *   - `scope.size.y` is the panel height PLUS whatever the sill hangs below the
 *     opening, which for a bare `window()` on a tile is kSillThickness.
 *   - `scope.size.z` is the reveal depth plus the sill projection -- the total
 *     front-to-back depth of the assembly, which was zero before the call.
 *   - `scope.origin` moves to the new minimum corner: back into the wall by the
 *     reveal, and down by whatever the sill overhangs.
 *
 * So `window(); split(x) { ... }` still cuts across the window, and
 * `geometry.volume()` still answers for the assembly. The alternative -- keeping
 * the scope on the original wall plane -- would leave a box that does not
 * contain its own geometry, which shape.hpp rejects for the whole language.
 *
 * ### A following `split(y)` is measured against the GROWN box
 *
 * That is the one consequence of the refit worth stating outright, because it is
 * otherwise silent and it does not look like arithmetic anybody chose.
 *
 * `window()` on a tile 3.0 m high leaves a box 3.0 + kSillThickness high whose
 * origin is kSillThickness BELOW the panel's foot. So
 *
 *     window(); split(y) { 1.0 : { Low(); } ~1.0 : { High(); } }
 *
 * cuts 0.96 above the panel's foot, not 1.00, and hands `High` a tile 2.04 high
 * rather than 2.00. A door does the same with kThresholdThickness. The offset is
 * a FIXED length, not a fraction, so it does not shrink as the tile does: the
 * ledge hangs the same distance below the opening whatever the tile is.
 *
 * Building the ledge upward from the opening's foot instead, so that the
 * assembly never leaves the panel, would keep a later `split(y)` honest -- and
 * would lay the sill slab across the bottom of its own glazing, on every window
 * in the project, in exchange for tidier numbers in the one rule that splits
 * after glazing. The order a facade rule actually wants is the other one: split
 * the wall into tiles first, then glaze each tile. Every rule file in the tests
 * is written that way.
 *
 * ================================================================================
 * AN OPENING BIGGER THAN THE PANEL IT SITS IN
 * ================================================================================
 *
 * `window(-0.5)` asks for an opening half a metre wider than its tile on every
 * side. That is a rule bug and not an unusual one: `inset` is often an attribute
 * and an attribute is often wrong.
 *
 * The opening is CLIPPED to the panel and a warning names both sizes. Not
 * failed: a facade with one over-wide window still has to build, because the
 * author needs to see the other ninety-nine tiles to understand what the
 * hundredth did. Not silently accepted either: an unclipped opening puts the
 * jambs outside the wall, where they read as a floating frame nobody asked for
 * and nothing points at the line that caused it.
 *
 * The opposite case -- an inset so large that nothing is left -- cannot produce
 * a window at all. It produces a BLANK WALL PANEL and a warning, for the same
 * reason: a bay too narrow for a window is a wall, and dropping it would put a
 * hole in the facade where the mistake is least visible.
 *
 * `wall_panel(inset)` follows the SAME policy, and it has to: an inset wide
 * enough to empty a 0.4 m pier is the identical author mistake, arriving through
 * the identical attribute, and the identical too-narrow tile. It keeps the
 * UN-INSET panel and warns. The two operations disagreeing about this was a real
 * defect -- `window(0.5)` on a narrow tile left a blank panel while
 * `wall_panel(0.5)` on the tile beside it failed the shape, deleted the terminal
 * and failed the whole generation, so the facade the author was trying to read
 * was the one thing they could not get.
 *
 * Any other reason offset_shape() cannot inset the panel is treated the same
 * way, deliberately. The alternative is matching on offset_shape()'s message
 * text to tell "removed everything" from the rest, which is a test that passes
 * until somebody rewords a diagnostic. Every failure it can return here is a
 * panel that could not be inset, the warning carries offset_shape()'s own words,
 * and what survives is the panel the author would have got from `wall_panel()`.
 *
 * ================================================================================
 * WHAT A PANEL HAS TO BE
 * ================================================================================
 *
 * `window` and `door` require a shape with exactly one non-degenerate, planar,
 * RECTANGULAR face. They fail with a message that says so otherwise.
 *
 * That is a real restriction and it is deliberate. The surround -- the wall left
 * around the opening -- is emitted as up to four axis-aligned strips of the
 * panel's own box, which is exact for a rectangle and wrong for anything else.
 * Cutting a rectangle out of an arbitrary polygon needs a general polygon
 * difference, whose output vertex count nothing downstream can predict, for a
 * case that does not arise: every tile a facade rule produces comes from
 * `split`, and `split` cuts rectangles into rectangles. A gable end is split
 * into rectangles before it is glazed.
 *
 * `wall_panel` has no such restriction, because it never cuts anything: it
 * insets with shape.hpp's Clipper2-backed offset_shape() and thickens with
 * extrude_shape(). A triangular gable panel is an ordinary wall panel.
 *
 * ================================================================================
 * MATERIALS, AND WHAT IS NOT HERE
 * ================================================================================
 *
 * Every face is tagged with a MaterialKey so that D8's texturing, and
 * renderer/mesh.hpp's submesh ranges, can tell glass from stone without
 * re-deriving it from the geometry. renderer/mesh.hpp has no `Glass` slot and
 * this feature does not add one, so every part is MaterialId::Wall with a
 * distinct `variant`; see facade_material() and the note on FacadePart.
 *
 * UV coordinates are D8 and are not written here. What this file does for D8 is
 * leave the geometry in a frame a planar projection works in -- the panel frame
 * described above -- and record the panel's own extent in the
 * `wall_panel.width` and `wall_panel.height` attributes, which is the pair a
 * later `normalize_uv` needs and which cannot be recovered from the scope once
 * the sill has grown it.
 *
 * ================================================================================
 * DETERMINISM
 * ================================================================================
 *
 * Nothing here draws a random number, iterates a hash container or calls libm on
 * a coordinate. The construction is +, -, * and a comparison; the one square
 * root is inside face_component_axes(), which shape.hpp's determinism note
 * already covers. Two runs of the same rule file produce byte-identical
 * geometry, and dump_shape() -- which prints every vertex of every face -- is
 * what the determinism test asserts on, so a change of one millimetre in one
 * corner fails it.
 */

#pragma once

#include "procgen/rules/interpreter.hpp"
#include "procgen/rules/shape.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace stratum::procgen::rules {

// ============================================================================
// Constants
// ============================================================================

/**
 * @brief Wall thickness used when nothing names one, in metres
 *
 * An ordinary insulated cavity wall. Large enough that the default reveal of
 * half of it, 0.15 m, is plainly visible at street scale, and small enough that
 * it is not wrong for a partition.
 */
inline constexpr double kDefaultWallThickness = 0.3;

/// Default width of the window or door frame members, in metres
inline constexpr double kDefaultFrameWidth = 0.05;

/// Default distance the sill projects out past the wall face, in metres
inline constexpr double kDefaultSillProjection = 0.05;

/// Default distance the threshold projects out past the wall face, in metres
inline constexpr double kDefaultThresholdProjection = 0.03;

/**
 * @brief Vertical thickness of the sill slab, in metres
 *
 * Not an argument: `window` has five argument slots and the four that matter to
 * a rule author are the inset, the reveal, the frame and the projection. The
 * sill's own thickness is a detail of the part, like the number of quads it is
 * made of.
 */
inline constexpr double kSillThickness = 0.04;

/// Vertical thickness of the threshold slab, in metres. See kSillThickness.
inline constexpr double kThresholdThickness = 0.03;

/**
 * @brief The reveal depth as a fraction of the wall thickness, when none is given
 *
 * The glazing line of a real window sits about the middle of the wall. A half is
 * the only fraction that needs no further argument, and it makes the default
 * reveal 0.15 m against the default wall.
 */
inline constexpr double kDefaultRevealFraction = 0.5;

/**
 * @brief Shape attribute a rule sets to tell the facade operations the wall thickness
 *
 * `set("wall_thickness", 0.45)` at the top of the building rule, inherited all
 * the way down to the tile. See the header note on where the thickness comes
 * from.
 */
inline constexpr const char* kWallThicknessAttribute = "wall_thickness";

/// Attribute recording the panel width, written by wall_panel() for D8
inline constexpr const char* kPanelWidthAttribute = "wall_panel.width";

/// Attribute recording the panel height, written by wall_panel() for D8
inline constexpr const char* kPanelHeightAttribute = "wall_panel.height";

/**
 * @brief Lengths at or below this are treated as zero, in metres
 *
 * A micron. Everything this file builds is axis-aligned arithmetic on
 * author-supplied metres, so the only values near it are genuine zeros: a strip
 * of surround with no width, a reveal of zero, a sill that was switched off.
 */
inline constexpr double kFacadeEpsilon = 1.0e-9;

// ============================================================================
// Materials
// ============================================================================

/**
 * @brief The parts a facade operation builds, one material variant each
 *
 * renderer/mesh.hpp's MaterialId has no `Glass` and no `Frame`, and this feature
 * does not widen that enum: MaterialId is a slot index the renderer and the
 * exporters both key on, and adding to it is a change with consequences well
 * outside a rules file. So every part is MaterialId::Wall carrying a distinct
 * `variant`, which is the field renderer/mesh.hpp documents for exactly this and
 * which MaterialKey::packed() already folds into a shader-visible index.
 *
 * The numbers are stable and are part of the output: a change to one of them
 * changes which submesh a facade's glass lands in.
 */
enum class FacadePart : uint16_t {
    Panel = 0,     ///< The wall surface, including the surround around an opening
    Reveal = 1,    ///< The jambs, head and bottom return between the face and the glazing line
    Frame = 2,     ///< The window or door frame members
    Glass = 3,     ///< The glazing
    Sill = 4,      ///< The sill under a window
    Leaf = 5,      ///< The door leaf
    Threshold = 6  ///< The threshold under a door
};

/// The MaterialKey a part's faces carry
[[nodiscard]] MaterialKey facade_material(FacadePart part);

/// Spell a part for a diagnostic and for a test message
[[nodiscard]] const char* facade_part_name(FacadePart part);

// ============================================================================
// Parameters
// ============================================================================

/**
 * @brief What an author can vary about one opening
 *
 * A NEGATIVE value in any field but @p inset means "no value was given, use the
 * default". That convention is used rather than an optional per field because
 * every one of these is a length, every length is non-negative, and the
 * operation handlers reject a negative one from a rule file before it ever
 * reaches here -- so inside this file a negative can only mean "absent".
 */
struct OpeningParams {
    /**
     * @brief Margin of wall left around the opening, in metres
     *
     * Applied on all four sides for a window and on three -- left, right and
     * head -- for a door, because a door stands on the floor. Zero, the default,
     * makes the opening fill the whole tile, which is what a bare `window()`
     * asks for.
     *
     * May be negative, which asks for an opening bigger than its tile; see the
     * header note on what happens then.
     */
    double inset = 0.0;

    /// Depth the glazing sits back from the wall face. Clamped to @p thickness.
    double reveal = -1.0;

    /// Width of the frame members. Zero gives glazing with no frame.
    double frame = -1.0;

    /// How far the sill or threshold projects out past the wall face. Zero omits it.
    double ledge = -1.0;

    /// Wall thickness the reveal is cut from. See the header note on where it comes from.
    double thickness = -1.0;
};

// ============================================================================
// Report
// ============================================================================

/**
 * @brief What an operation did, and what it had to change to do it
 *
 * The numbers are here so that a test can assert the thickness and the reveal
 * that were actually used, rather than inferring them from a vertex and hoping.
 * The warnings are here because an operation in this file must never fail a
 * shape for something the shape can survive -- see the header note -- and a
 * change made silently is a change nobody finds.
 */
struct FacadeReport {
    /// Warnings, in the order they arose. The handler reports each at the call site.
    std::vector<std::string> warnings;

    /// The opening was bigger than the panel and was clipped to it
    bool opening_clipped = false;

    /// Nothing was left to make an opening from; the shape is a blank wall panel
    bool opening_empty = false;

    /**
     * @brief wall_panel()'s inset could not be applied, so the panel was kept whole
     *
     * Almost always an inset wide enough to consume the panel. Named for what the
     * operation DID rather than for the offset's reason, because the reason is in
     * the warning and the geometry is what a later operation has to reason about.
     */
    bool inset_dropped = false;

    /// An explicit reveal was deeper than the wall and was clamped to it
    bool reveal_clamped = false;

    /// The frame was wide enough to swallow the glazing, so there is none
    bool frame_filled_opening = false;

    /// Wall thickness actually used, in metres
    double wall_thickness = 0.0;

    /// Reveal depth actually used, in metres
    double reveal = 0.0;

    /// Panel width and height in the panel frame, before anything was cut
    double panel_width = 0.0;
    double panel_height = 0.0;

    /// The opening in the panel frame: (x0, y0, x1, y1), after any clipping
    glm::dvec4 opening{0.0};
};

// ============================================================================
// The operations
// ============================================================================

/**
 * @brief Reframe a shape into its panel frame: x across, y up, z out of the wall
 *
 * The first thing all three operations do, and exposed because it is the part a
 * test can assert exactly and because a later facade operation will want the
 * same frame. Uses op_comp.hpp's face_component_axes() for the frame and
 * shape.hpp's reframe() to move into it, so the world positions do not change
 * and no second copy of either exists.
 *
 * Skipped outright when the shape is already in that frame, which is every tile
 * a `select face` then `split` chain produces. That is not an optimisation: it
 * keeps the common path free of the rounding a transpose-and-multiply round trip
 * would otherwise add to every vertex.
 *
 * It also REDUCES the shape to that one face, dropping any degenerate faces
 * beside it. find_panel_face() has already established there is nothing else
 * with area, so what goes is zero-area geometry that every consumer downstream
 * would otherwise have to special-case.
 *
 * @param shape Shape to reframe, modified in place
 * @param why   Receives the reason on failure; untouched on success
 * @return false when the shape has no single non-degenerate planar face to take
 *         a frame from. The shape may already have been changed; see OpResult's
 *         note on partial application.
 */
[[nodiscard]] bool orient_panel(Shape& shape, std::string& why);

/**
 * @brief Cut a window into a rectangular wall panel
 *
 * Builds, in the panel frame: the surround left around the opening, the reveal
 * returns from the wall face back to the glazing line, the frame members, the
 * glazing, and the sill. Every part is one or more faces of the SAME shape,
 * each carrying its own material -- not a child shape, because a window is one
 * thing and a rule that wanted its parts separately would have written a
 * `split`.
 *
 * The bottom reveal return is omitted when there is a sill, because the sill's
 * top face is that return and two coincident surfaces is worse than none.
 *
 * @param shape  Shape to glaze, modified in place
 * @param params Opening parameters; see OpeningParams for the "absent" convention
 * @param report Receives what was used and what was changed; may be nullptr
 * @return failure only when the shape is not a usable rectangular panel. An
 *         opening that had to be clipped, emptied or un-glazed is a WARNING in
 *         @p report and a shape that still builds.
 */
[[nodiscard]] OpResult window_shape(Shape& shape,
                                    const OpeningParams& params,
                                    FacadeReport* report = nullptr);

/**
 * @brief Cut a door into a rectangular wall panel
 *
 * A window that stands on the floor. Four differences, all of them consequences
 * of that one:
 *
 *   - @p params.inset is applied to the left, the right and the head, never to
 *     the foot. The opening's bottom edge is the panel's bottom edge.
 *   - There is no bottom reveal return and no bottom frame member: the threshold
 *     is what the opening stands on.
 *   - The glazing is a door LEAF, which fills the opening inside the frame down
 *     to the floor.
 *   - @p params.ledge is the threshold's projection, not a sill's.
 *
 * WHICH panel gets a door is the rule's business, not this operation's: a rule
 * writes `split(y) { 4.0 : { Shopfront(); } ... }` and calls `door()` in the
 * shopfront. The operation has no way to know it is on the ground floor and does
 * not try to guess.
 *
 * @see window_shape() for the parameters, the report and the failure contract
 */
[[nodiscard]] OpResult door_shape(Shape& shape,
                                  const OpeningParams& params,
                                  FacadeReport* report = nullptr);

/**
 * @brief The wall surface between the openings, in its own frame
 *
 * Three things, any of which may be all the rule wanted:
 *
 *   - **A frame.** The shape is reframed into the panel frame, so a later
 *     operation, an exporter and D8's planar projection all see x across the
 *     wall, y up it and z out of it.
 *   - **An inset**, through shape.hpp's offset_shape(). A panel inset inside its
 *     tile is the recessed-panel and expressed-joint pattern, and offsetting
 *     rather than cutting strips is what lets a non-rectangular panel take one.
 *   - **A thickness**, through shape.hpp's extrude_shape() along -z, so the
 *     panel becomes a closed slab going INTO the wall rather than out of it.
 *     Out of it would put the wall in front of its own windows.
 *
 * It also records the panel's own width and height in the kPanelWidthAttribute
 * and kPanelHeightAttribute attributes. That is the one thing this operation
 * does for D8 that D8 cannot do for itself: once a sill or a thickness has grown
 * the scope, the panel's own extent is not recoverable from it.
 *
 * @param shape     Shape to make a panel of, modified in place
 * @param inset     Margin to inset the outline by; zero or negative for none
 * @param thickness Depth to give the panel; zero or negative to leave it flat
 * @param report    Receives the panel extent and any warnings; may be nullptr
 * @return failure only when the shape has no single non-degenerate planar face
 *         to be a panel of. An inset that cannot be applied -- an inset wide
 *         enough to consume the panel is the usual one -- is a WARNING in
 *         @p report, FacadeReport::inset_dropped, and the un-inset panel; see
 *         the header note, which is the same policy window() has for the same
 *         author mistake.
 */
[[nodiscard]] OpResult wall_panel_shape(Shape& shape,
                                        double inset,
                                        double thickness,
                                        FacadeReport* report = nullptr);

// ============================================================================
// Registration
// ============================================================================

/**
 * @brief Add E3's operations to @p table
 *
 * `window`, `door` and `wall_panel`, all three of them rows that already exist
 * in ast.hpp's kBuiltinOperations.
 *
 * Additive, and separate from facade_operations(), because several operation
 * families land in this tree at once and each has to register into one table
 * without knowing what the others put there. A later registration of the same
 * name wins, which is OperationTable's documented behaviour.
 */
void register_facade_operations(OperationTable& table);

/**
 * @brief standard_operations() plus register_facade_operations(), built once
 *
 * A caller that wants the facade operations AND another family copies that
 * family's table and calls register_facade_operations() on the copy; this
 * accessor is the convenience for a caller that wants D2 and E3 and nothing
 * else, which is what the tests want.
 */
[[nodiscard]] const OperationTable& facade_operations();

} // namespace stratum::procgen::rules
