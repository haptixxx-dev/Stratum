// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file op_split.hpp
 * @brief Splitting a shape along a scope axis: the sizing solver and the geometric cut
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ================================================================================
 * WHAT THIS IS
 * ================================================================================
 *
 * Everything architectural is a split. A facade becomes floors, a floor becomes
 * bays, a bay becomes window and wall. D2 gave the language an evaluator that can
 * extrude, offset and taper; without a split it can make boxes and nothing else.
 *
 * The feature is two separable halves, and they are separate here because they
 * fail in different ways and are tested against different things:
 *
 *   - **The solver.** Given a list of parts and an extent, where does each part
 *     start and how long is it? Pure arithmetic over doubles, no geometry, no
 *     AST. This is where `~` earns its keep and where every hand-computed number
 *     in the test suite points.
 *   - **The cut.** Given a shape and an interval along a scope axis, what is the
 *     geometry inside that slab? Polygon clipping plus capping, so that a slab of
 *     a closed solid is itself a closed solid rather than a box with two open
 *     ends.
 *
 * ================================================================================
 * THE THREE SIZE KINDS, AND WHY FLOATING IS THE ONE THAT MATTERS
 * ================================================================================
 *
 * ast.hpp declares four SizeKinds; they resolve here as follows, against an
 * extent `E`:
 *
 *   - `SizeKind::Absolute`, written `3.0` -- three scope units. Fixed.
 *   - `SizeKind::Relative`, written `25%` -- `value / 100 * E`. **The value is the
 *     PERCENTAGE as the author wrote it**, because that is what the parser stores:
 *     `25%` is a NumberExpr of 25 with the kind set to Relative, and dividing by a
 *     hundred anywhere but here would mean two places had to agree about it.
 *   - `SizeKind::Floating`, written `~3.0` -- a NOMINAL three units. Floating
 *     parts do not take their nominal size; they share out whatever the absolute
 *     and relative parts left over, in proportion to their nominals.
 *   - `SizeKind::FloatingRelative`, written `~25%` -- floating with a nominal of
 *     `value / 100 * E`.
 *
 * Inside a repeat group there are two extents in play, and each relative size
 * resolves against the one that makes it computable. A relative child always
 * contributes `value / 100 * E` to the group's PERIOD, because the period is what
 * decides how many copies there are and so cannot depend on the copy. What it
 * takes INSIDE a copy depends on what is beside it, and both answers fill the
 * copy:
 *
 *   - Beside something elastic, `value / 100 * copy`, which is what the author
 *     reads it as; the floating part beside it takes the rest.
 *   - Beside nothing elastic, its share of the period, scaled with the whole copy
 *     so that the copy fills its span. `repeat { 50% : Bay(); }` over an extent of
 *     4 is two copies of 2 and the bay is the whole of its copy -- NOT the one
 *     metre that "50% of a two-metre copy" reads as, which would leave half of
 *     every copy empty and half the facade missing. Once the copy count is
 *     settled a relative size is a RATIO to its siblings, and there is no reading
 *     of it that is both "half of the copy" and "no gaps".
 *
 * A facade is "a fixed-width pier, then as many window bays as fit, then a
 * fixed-width pier". A language without the floating size cannot say that: every
 * facade would be exactly as wide as the rule assumed, and a lot one metre wider
 * would either overflow or leave a gap. So the floating share is the operation
 * the whole family exists for, and solve_split() computes it in one pass with no
 * iteration, no relaxation and no tolerance:
 *
 *     fixed     = sum of the absolute and relative lengths
 *     remainder = max(0, E - fixed)
 *     share(p)  = remainder * nominal(p) / sum of all nominals
 *
 * ================================================================================
 * REPETITION, AND WHERE THE LEFTOVER GOES
 * ================================================================================
 *
 * `repeat { ~2.5 : Bay(); 0.4 : Pier(); }` is a GROUP that tiles. The group's
 * period `P` is the sum of its children's nominal sizes; given a span `S` it makes
 *
 *     n = floor(S / P)    copies, each of length S / n
 *
 * The brief for this feature asks the question directly: the leftover `S - n * P`
 * can be distributed into the copies, left as a gap, or given to a neighbour.
 * **It is distributed into the copies**, which is the only one of the three that
 * architecture accepts:
 *
 *   - Every copy is the same length, `S / n`. Bays that differ from each other by
 *     a centimetre are the single most visible defect a facade can have, and a
 *     gap parked at one end is that same defect concentrated in one place.
 *   - No gap opens anywhere, so the split still tiles the extent exactly and the
 *     next operation does not have to know about a hole.
 *   - It is predictable from the numbers the author wrote: `S / floor(S / P)`.
 *
 * Inside a copy the stretch lands wherever the children say it should. A copy
 * with a floating child lets the float absorb it and the fixed children keep
 * their exact metres -- so `repeat { 0.4 : Pier(); ~2.5 : Bay(); }` keeps every
 * pier at 0.4 and varies only the glass, which is what a real facade does. A copy
 * with NOTHING elastic in it -- every child absolute or relative -- has nowhere
 * to put the stretch, so the whole copy is laid out at its natural size and
 * scaled uniformly; the alternative is a gap at the end of every copy, which is
 * the answer this file has already rejected.
 *
 * `n` is a FLOOR, so a copy is never shorter than the period and may be up to
 * twice it. Rounding instead would keep copies closer to their nominal size, and
 * was rejected because "as many whole copies as fit" is what the author reads
 * `repeat` as meaning, and a rule that produces FEWER bays than fit is surprising
 * in a way that a rule producing slightly wide bays is not. A group that does not
 * fit even once still gets one copy, compressed to `S`, and says so in
 * SplitLayout::notes: a facade that silently loses its windows because the lot
 * came out narrow is a bug report with nothing in it.
 *
 * ================================================================================
 * THE FOUR DEGENERATE SPLITS, EACH OF WHICH A PERSON WRITES
 * ================================================================================
 *
 * None of these crashes, and none of them silently produces nothing:
 *
 *   - **No parts at all.** `solve_split({}, E)` returns no pieces and one note.
 *     The parser rejects `split(x) { }`, but a repeat group whose children were
 *     all dropped, and any programmatic caller, can still reach it.
 *   - **Absolutes that exceed the extent.** The parts are laid out in order and
 *     CLIPPED at the far end: the straddling part is truncated, the parts past it
 *     are zero-length, and SplitLayout::overflow says by how much. Scaling
 *     everything down to fit was rejected -- it turns the author's 3.0 into 2.4
 *     with nothing on screen to say so, and a storey height that quietly is not
 *     the storey height is worse than a visibly truncated top floor.
 *   - **A zero-width part.** Kept as a zero-length piece, so the part indices
 *     still line up with the parts the author wrote and the rule body still runs
 *     against an empty shape. Dropping it would renumber everything after it.
 *   - **A zero extent on the chosen axis.** `split(y)` of a footprint. Every
 *     piece is zero-length, the layout carries a note, and the pieces exist, so
 *     the caller reports one diagnostic instead of the shape vanishing.
 *
 * ================================================================================
 * THE CUT: WHY CLIPPING NEEDS CAPS, AND WHY THE CUT POINTS ARE CACHED
 * ================================================================================
 *
 * slice_shape() clips every face against the two slab planes. Clipping alone
 * leaves a slab of a solid open at both ends -- the volume is then meaningless,
 * the renderer shows the inside of the next floor through the cut, and
 * shape.hpp's own geometry_volume() reports nonsense. So the cut edges are
 * chained into loops and capped.
 *
 * Two details carry the determinism this project asks for everywhere:
 *
 *   - **Each cut point is computed once and shared.** Two faces meeting at an
 *     edge that crosses a plane both need the intersection. Computing it twice,
 *     once from each face and therefore once from each end of the edge, gives two
 *     points that differ in the last bit -- and then the cap loop has a crack in
 *     it that no tolerance in the chaining can close without also welding points
 *     that should stay apart. The intersection is keyed on the edge's ENDPOINT
 *     INDICES in ascending order, so both faces get the identical double.
 *   - **The plane coordinate is assigned, not interpolated.** `p[axis] = plane`
 *     after the interpolation, so a cut point is exactly on its plane and the
 *     cap-loop search is an equality test rather than a tolerance.
 *
 * Three things follow from the cut plane being allowed to coincide with geometry
 * that is already there, which is what a non-convex footprint does as a matter of
 * course, and each of them shipped as a silently wrong slab until it was named:
 *
 *   - **A face lying IN a plane the cut crossed is replaced by the cap**, not
 *     kept beside it. Splitting an L at the plane of its own reflex wall put that
 *     wall into BOTH slabs -- a point on the plane is inside both half-spaces --
 *     and laid the cap over the top of it in one and left it stranded in the
 *     other. An L of volume 12 came out as slabs of 9.33 and 4, neither closed.
 *   - **A hole bounds the cap exactly as an outline does.** Cutting a courtyard
 *     block across its light well otherwise roofs the well over: the cap ring
 *     closes around the whole cross-section, and the slab is sealed, heavier than
 *     the solid it came from, and open at the seam.
 *   - **A ring that doubles straight back on itself is trimmed.**
 *     Sutherland-Hodgman on a non-convex ring that touches the plane leaves a
 *     zero-area tail, which encloses nothing and so passes every area and volume
 *     check -- while putting a vertex the slab does not contain into the position
 *     list, which refit_scope() then reports as the slab's extent.
 *
 * What is still NOT done is a cut whose chain of cut edges does not close. That
 * needs an input that is already open or non-manifold -- half a box, a wall with
 * no thickness -- and the chain is dropped rather than guessed at, so the slab
 * stays as open as the shape it was cut from. slice_shape() has nowhere to report
 * it: it returns a Shape and not a diagnostic.
 *
 * ================================================================================
 * WHAT THIS FILE DOES NOT DO, AND SAYS SO
 * ================================================================================
 *
 *   - A hole that STRADDLES a cut plane comes back as a ring touching the
 *     outline rather than as a notch in the outline. The two describe the same
 *     set and face_area() reports the same number for both, but a consumer that
 *     assumes a hole is strictly interior has to know. Merging the two rings is a
 *     boolean operation, not a clip, and a wrong merge emits a self-intersecting
 *     face that looks right until it is exported. The CAP is not affected: it is
 *     chained from the boundary edges of both rings, so a slab cut across a light
 *     well closes correctly around the well even though the face that reaches it
 *     is in the non-canonical form.
 *   - The cut is AXIS-ALIGNED in the shape's own scope, which is the whole reason
 *     shape.hpp stores geometry scope-locally. A split along a direction that is
 *     not a scope axis is `rotate_scope` followed by a split.
 *
 * ================================================================================
 * HOW THIS IS REACHED FROM A RULE -- READ THIS BEFORE WIRING IT UP
 * ================================================================================
 *
 * It is not, yet, and that is a finding about the extension point rather than
 * about this file.
 *
 * interpreter.hpp offers two registries, OperationTable and FunctionTable, and
 * says that a later feature adds operations "without editing this file".
 * D3's operation is `split`, and `split` is not an operation: it is a STATEMENT,
 * a variant alternative in ast.hpp's StmtNode, dispatched by a `switch` inside
 * interpreter.cpp's State::exec(), which today reports "'split' is not
 * implemented in this build". There is no statement registry, `split` has no row
 * in kBuiltinOperations for the parser to resolve a call against, and `split` is
 * a keyword the lexer never produces an Ident for. Registering it is therefore
 * impossible from outside interpreter.cpp, and would remain impossible even with
 * a catalogue row: OperationArgs hands a handler a Shape, a list of Values and an
 * Interpreter whose public surface can report, log and emit a TERMINAL, but has
 * no way to invoke a rule. A split entry is `3.0 : Floor();` -- running `Floor`
 * on the slab is the point of it.
 *
 * So this file is written to make that wiring one short, obvious block rather
 * than a rewrite, and the pieces it needs are all public here:
 *
 * @code
 *     case StmtKind::Split: {
 *         const SplitStmt& node = std::get<SplitStmt>(stmt.node);
 *         const ScopeAxis axis = to_scope_axis(node.axis);
 *         const std::vector<SplitPart> parts = split_parts_from_statement(
 *             node, [&](ExprId id, const SourceLoc& at) {
 *                 return want_number(eval(id, shape), at, "a split size");
 *             });
 *         const SplitLayout layout = solve_split(parts, shape.scope.extent(axis));
 *         for (const std::string& note : layout.notes) {
 *             report_once(Severity::Warning, stmt.loc, note);
 *         }
 *         const std::vector<Shape> slabs = split_shape(shape, axis, layout);
 *         for (size_t i = 0; i < slabs.size(); ++i) {
 *             const StmtId body = split_body_for(node, layout.pieces[i]);
 *             // ... make a child shape from slabs[i] and run `body` against it,
 *             // exactly as exec_call() does for a rule call.
 *         }
 *         return Flow::Continue;
 *     }
 * @endcode
 *
 * Everything above that comment block is implemented and tested here.
 */

#pragma once

#include "procgen/rules/ast.hpp"
#include "procgen/rules/lexer.hpp"
#include "procgen/rules/shape.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace stratum::procgen::rules {

// ============================================================================
// Parts
// ============================================================================

/// "No index". Same choice of sentinel, for the same reason, as ast.hpp's kNoNode.
inline constexpr size_t kNoPart = static_cast<size_t>(-1);

/**
 * @brief One row of a split, as the solver sees it
 *
 * Deliberately NOT ast.hpp's SplitEntry. The solver takes numbers, not
 * expressions: a SplitEntry holds ExprIds that only an interpreter with a live
 * shape and a live frame can evaluate, and a solver that needed one could not be
 * tested without standing up a whole generation. split_parts_from_statement()
 * bridges the two, and is the only part of this file that knows the AST exists.
 *
 * @p value means different things per @p kind, and the difference is the feature:
 * metres for Absolute, PERCENT for Relative (`25%` is 25.0, not 0.25), nominal
 * metres for Floating, nominal PERCENT for FloatingRelative.
 */
struct SplitPart {
    SizeKind kind = SizeKind::Absolute;
    double value = 0.0;

    /// True for a `repeat { ... }` group. @p kind and @p value are then unread.
    bool is_repeat = false;

    /// The group's contents when @p is_repeat. Never itself contains a repeat;
    /// the parser rejects that, and solve_split() ignores one if it sees it.
    std::vector<SplitPart> children;
};

// ============================================================================
// Layout
// ============================================================================

/**
 * @brief One resolved interval along the split axis
 *
 * @p part, @p child and @p copy are what let a caller get back to the rule body
 * that belongs here: @p part indexes the list handed to solve_split(), @p child
 * indexes that part's children when the part is a repeat group, and @p copy
 * counts the tiling from zero. split_body_for() does that lookup against a
 * SplitStmt.
 */
struct SplitPiece {
    double begin = 0.0;   ///< Start along the axis, in scope units from the box minimum
    double length = 0.0;  ///< Extent along the axis. Never negative; may be zero.

    size_t part = 0;          ///< Index into the parts handed to solve_split()
    size_t child = kNoPart;   ///< Index into that part's children, kNoPart when it is not a repeat
    uint32_t copy = 0;        ///< Which copy of a repeat group, from 0
};

/**
 * @brief What the solver decided, and everything it had to say about it
 *
 * The notes are the "say what you do" half of the feature. They are complete
 * sentence fragments in the same style as shape.hpp's OpResult::message -- no
 * full stop, no operation name -- so a caller can hand one straight to
 * Interpreter::report() with the split statement's own SourceLoc.
 */
struct SplitLayout {
    /// In order along the axis, non-overlapping, and tiling [0, extent] to within
    /// rounding whenever nothing overflowed and something was elastic.
    std::vector<SplitPiece> pieces;

    double extent = 0.0;  ///< The extent that was split
    double used = 0.0;    ///< Far end of the last piece

    /// How much the fixed parts asked for beyond @p extent. Zero when they fit.
    double overflow = 0.0;

    /// Sum of the nominal sizes of everything elastic: the floating parts and the
    /// repeat groups. Zero means nothing can absorb a remainder, which is what
    /// tells a repeat copy to scale its contents instead.
    double elastic_weight = 0.0;

    uint32_t repeat_copies = 0;  ///< Total copies emitted by every repeat group

    /// A repeat group had to be squeezed below its period to appear at all
    bool compressed = false;

    std::vector<std::string> notes;

    /// Did the solve need to say anything? A clean solve is the common case.
    [[nodiscard]] bool clean() const { return notes.empty(); }
};

/**
 * @brief The nominal size of one part, in scope units
 *
 * The number the part would like to be: its metres for Absolute and Floating,
 * `value / 100 * extent` for the two relative kinds, and the sum over the
 * children for a repeat group -- which is that group's PERIOD.
 *
 * Negative values are clamped to zero here rather than at the call sites, so
 * every consumer of a nominal agrees about what a negative size means.
 */
[[nodiscard]] double nominal_size(const SplitPart& part, double extent);

/**
 * @brief Resolve a list of parts against an extent
 *
 * The whole of the sizing feature. See the file header for the arithmetic, for
 * where a repeat group's leftover goes, and for what each degenerate case does.
 *
 * @param parts  Rows of the split, in the order the author wrote them
 * @param extent Extent of the scope along the split axis. A negative extent is
 *               treated as zero.
 * @return Pieces in order, plus the numbers and notes the caller reports
 */
[[nodiscard]] SplitLayout solve_split(const std::vector<SplitPart>& parts, double extent);

// ============================================================================
// The cut
// ============================================================================

/// ast.hpp's split axis as shape.hpp's scope axis. The two enums agree by
/// construction -- shape.hpp says so -- and this is where that is checked.
[[nodiscard]] ScopeAxis to_scope_axis(SplitAxis axis);

/**
 * @brief The part of a shape between two planes perpendicular to a scope axis
 *
 * Every face is clipped to the slab and the cut is capped, so a slab of a closed
 * solid is a closed solid. See the file header for the clipping contract, the
 * shared cut points and the one documented limitation (a hole that straddles a
 * plane comes back as a ring touching the outline, not as a notch in it).
 *
 * The result is a COPY of @p shape with its geometry and scope replaced: the
 * attributes, the rule name and the tree address come along, which is the
 * pattern interpreter.hpp's emit_derived_terminal() documents for a handler that
 * wants the parent's attributes.
 *
 * @param shape Shape to cut. Not modified.
 * @param axis  Scope axis the planes are perpendicular to
 * @param begin Near plane, in scope-local units from the box minimum
 * @param end   Far plane. `end <= begin` gives an empty slice rather than an
 *              inside-out one.
 * @return The slab. When no geometry survives, the geometry is empty and the
 *         scope is the slab itself -- shape.hpp's documented exception to the
 *         tight-box invariant, and what lets D4 insert an asset into a gap.
 */
[[nodiscard]] Shape slice_shape(const Shape& shape, ScopeAxis axis, double begin, double end);

/**
 * @brief One slab per piece of a solved layout
 *
 * Each piece is cut from the ORIGINAL shape rather than from the remainder of the
 * previous cut, so a piece's geometry never depends on the pieces before it and a
 * rounding error in one cannot walk down the row. The cost is one clip of the
 * whole shape per piece, which is the right trade at the sizes architecture
 * produces -- a facade is tens of bays over tens of faces.
 *
 * @return Exactly `layout.pieces.size()` shapes, in piece order, so the caller
 *         can index the two together. A zero-length piece yields a shape with no
 *         geometry rather than being skipped.
 */
[[nodiscard]] std::vector<Shape> split_shape(const Shape& shape,
                                             ScopeAxis axis,
                                             const SplitLayout& layout);

// ============================================================================
// The AST bridge
// ============================================================================

/**
 * @brief How the bridge turns a size expression into a number
 *
 * The interpreter's own evaluator, wrapped. It is a callback rather than an
 * Interpreter& because this file must not depend on the evaluator: the solver is
 * tested against literal sizes with a four-line evaluator, and a dependency on
 * Interpreter would drag a whole generation into every one of those tests.
 *
 * A callback that cannot produce a number returns whatever it likes and reports
 * through its own interpreter; it may also abandon the shape by throwing, which
 * passes straight out of split_parts_from_statement().
 */
using SplitSizeEvaluator = std::function<double(ExprId, const SourceLoc&)>;

/**
 * @brief Build solver parts from a parsed split statement
 *
 * Preserves entry order and repeat nesting exactly, so SplitPiece::part and
 * SplitPiece::child index straight back into @p stmt.
 */
[[nodiscard]] std::vector<SplitPart> split_parts_from_statement(
    const SplitStmt& stmt, const SplitSizeEvaluator& evaluate);

/**
 * @brief The entry a piece came from
 * @return nullptr when @p piece does not address an entry of @p stmt, which
 *         means the layout and the statement do not belong together
 */
[[nodiscard]] const SplitEntry* split_entry_for(const SplitStmt& stmt, const SplitPiece& piece);

/**
 * @brief The BlockStmt a piece's slab should be evaluated against
 * @return kNoNode when the piece addresses no entry, or the entry has no body
 */
[[nodiscard]] StmtId split_body_for(const SplitStmt& stmt, const SplitPiece& piece);

} // namespace stratum::procgen::rules
