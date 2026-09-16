// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file op_comp.hpp
 * @brief Component split (D4): a solid into its faces, a face into its edges
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ================================================================================
 * WHAT THIS IS FOR
 * ================================================================================
 *
 * This is how a rule gets from a mass model to the surfaces it decorates. A
 * building is extruded, its front face is picked out, and a facade rule runs on
 * that face -- `select face { front: Facade(); top: Roof(); }`. Without a
 * component split the language can only ever transform one lump.
 *
 * Producing every face is easy and nearly useless. SELECTION is the feature: a
 * rule has to be able to say WHICH face, in words that keep meaning the same
 * thing when the building is rotated, when a storey is added, or when the OSM
 * footprint the mass came from has seven sides instead of four. Everything below
 * exists to make that possible and to make it reproducible.
 *
 * ================================================================================
 * THE FOUR THINGS A READER HAS TO KNOW
 * ================================================================================
 *
 * ### 1. Direction words mean the SHAPE'S SCOPE, never the world
 *
 *     top    = +y     bottom = -y
 *     right  = +x     left   = -x
 *     front  = +z     back   = -z
 *
 * of the shape's own scope. +z is the front because the project is y-up and
 * right-handed, so +z is the axis that comes out of the screen towards a viewer
 * looking down -z -- the face you are looking at is the front.
 *
 * This falls out of shape.hpp's first invariant rather than being bolted on:
 * geometry is stored in SCOPE-LOCAL coordinates, so `face_normal()` already
 * returns a normal in the scope's frame. A building placed on a lot at 40
 * degrees carries that rotation in `Scope::axes`, its local normals are
 * untouched, and its front is the same wall it was before it was placed. A
 * world-space test would have handed back whichever wall happened to point at
 * north.
 *
 * The flip side, and it is a feature: `rotate_scope(0, 90, 0)` turns the frame
 * and leaves the geometry, so afterwards `front` names what used to be the right
 * wall. That is the whole purpose of `rotate_scope` and `align_scope`, and it is
 * the supported way to say "treat THAT wall as the front".
 *
 * ### 2. A component's scope is oriented TO THE COMPONENT
 *
 * A face component's scope has
 *
 *     z = the face normal            (so the face is flat in z: `size.z == 0`)
 *     y = "up within the face"       (the parent's +y projected into the plane)
 *     x = cross(y, z)                (across the face, right-handed)
 *
 * so a facade rule can write `split(y) { 3.2 : Floor(); }` and get storeys, and
 * `split(x)` and get bays. Give the component the parent's axes instead and the
 * same rule cuts across the world, which for the front wall of an axis-aligned
 * box looks almost right and for the side wall of a rotated one is nonsense.
 * That is why the tests assert the AXES, not only the component count.
 *
 * For a horizontal face the parent's +y is not in the plane and cannot supply
 * the in-plane up, so the parent's +z is used instead. A roof's scope therefore
 * has z pointing up out of the roof and y running along the parent's z. The
 * switch happens within a whisker of the pole (see kComponentPoleCosine), so a
 * wall that is one degree off vertical still gets the frame a wall should get.
 *
 * An edge component's scope has x along the edge, z along the normal of the face
 * the edge came from, and y = cross(z, x), which points INTO that face because
 * Face::loop is wound counter-clockwise about its normal. So `translate(0, h, 0)`
 * on an edge moves into the face and `scale` along x runs the edge's length.
 *
 * ### 3. The component order is a function of the GEOMETRY, not of the buffer
 *
 * `select face { 3 : ... }` is worthless if face 3 is a different face on a
 * re-run or after an unrelated change to the extruder. So components are sorted
 * before they are numbered, on a key computed from the geometry alone:
 *
 *   - the centroid, quantised to kComponentOrderQuantum and compared as
 *     integers: ascending y first (lowest component first), then ascending x,
 *     then ascending z;
 *   - then the normal, the same way, which separates two coincident components;
 *   - then the position in the geometry buffer, which is only ever reached by
 *     two components that are geometrically identical and which makes the order
 *     a TOTAL one so that std::sort has a strict weak ordering to work with.
 *
 * Quantised integers rather than an epsilon compare, because an epsilon compare
 * is not transitive and a std::sort given a non-transitive comparator is
 * undefined behaviour, not merely an odd order.
 *
 * Two consequences, both deliberate:
 *
 *   - Permuting `ShapeGeometry::faces` does not change which component is number
 *     3. There is a test that does exactly that, because "the order is stable"
 *     asserted against an unpermuted buffer is a test that cannot fail.
 *   - Degenerate components are dropped BEFORE numbering, so index N always
 *     addresses a component that exists. Adding a degenerate face to a shape
 *     therefore does not renumber the real ones.
 *
 * ### 4. Nothing here is silent
 *
 * A degenerate face, a non-planar face, an index past the end, a domain this
 * build does not implement: each is counted and described in
 * ComponentSplitReport, and the caller turns that into a Diagnostic at the line
 * that asked. A component split that quietly returns fewer shapes than the
 * author expected is a facade with a missing wall and nothing to point at.
 *
 * ================================================================================
 * THE AWKWARD GEOMETRY, AND WHAT IS DONE ABOUT IT
 * ================================================================================
 *
 * ### A face with holes
 *
 * The holes travel with the component. They are part of the face -- a wall with
 * a courtyard opening is one face with one hole -- and dropping them would make
 * the component's area wrong and its triangulation solid. The rings keep their
 * winding (opposite to the outer loop, which is Face's contract), the scope box
 * is fitted to the OUTER loop because a hole is inside it, and
 * `Component::measure` is the outer area minus the holes'. `delete_holes` is a
 * separate catalogue row for the rule that wants them gone.
 *
 * ### A degenerate or zero-area face
 *
 * A face with fewer than three vertices, with collinear vertices, or whose holes
 * eat its whole area has no normal and therefore no direction and no frame. It
 * cannot be classified, cannot be oriented, and cannot be split further. It is
 * dropped and counted in ComponentSplitReport::degenerate, which
 * emit_components() turns into a Warning -- degenerate input geometry is
 * usually the footprint's doing, not the rule author's. It is NOT numbered: see
 * point 3.
 *
 * ### A non-planar quad
 *
 * OSM-derived geometry produces these constantly -- four corners of a building
 * footprint at four slightly different ground heights. Newell's method gives
 * such a face a perfectly good area-weighted normal, so it classifies and
 * orients like any other, and the plane is taken through the face's CENTROID
 * rather than through its first vertex, so the answer does not depend on where
 * the ring happens to start.
 *
 * The vertices are NOT flattened onto that plane. The component's `size.z` is
 * then the face's out-of-plane thickness rather than zero, which is honest and
 * measurable: `size.z == 0` is exactly the test for "this face was planar".
 * Flattening would move vertices that the neighbouring face still shares, and
 * the symptom is a crack in the mesh several operations downstream. The
 * deviation is reported through ComponentSplitReport::non_planar and
 * ::worst_planarity so a caller can warn about a facade that will not sit flat.
 *
 * ================================================================================
 * HOW THIS IS REACHED FROM A RULE FILE -- AND THE GAP
 * ================================================================================
 *
 * The expression half registers cleanly. `register_component_functions()` adds a
 * `comp` namespace -- `comp.count`, `comp.area`, `comp.size` -- to any
 * FunctionTable, exactly as interpreter.hpp's "copy a table, register into the
 * copy" contract describes, with no edit to interpreter.cpp.
 *
 * The geometry half CANNOT be registered, and this is a gap in the extension
 * point rather than a shortcoming of this file:
 *
 *   - `select face { front: ... }` is a STATEMENT. ast.hpp's StmtNode is a
 *     closed variant and interpreter.cpp's State::exec() switches over it, so
 *     SelectStmt can only ever be implemented by editing interpreter.cpp.
 *     interpreter.hpp says as much, and it is right.
 *   - An OPERATION cannot stand in for it either. exec_operation() resolves a
 *     statement callee through ast.hpp's kBuiltinOperations and looks the
 *     handler up by THAT row's name, so a handler registered under a name the
 *     catalogue does not carry is unreachable. The catalogue deliberately has no
 *     `comp` row, because `comp` was always going to be a statement.
 *
 * So emit_components() below is the whole body of the missing
 * `case StmtKind::Select:`, minus the two lines that run each arm's block. When
 * that case is written, it calls this; nothing else about this file changes.
 */

#pragma once

#include "procgen/rules/ast.hpp"
#include "procgen/rules/interpreter.hpp"
#include "procgen/rules/lexer.hpp"
#include "procgen/rules/shape.hpp"

#include <glm/glm.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace stratum::procgen::rules {

// ============================================================================
// Tolerances
// ============================================================================

/**
 * @brief How far off a quarter turn a face may be and still count as vertical or horizontal
 *
 * One degree. `vertical` and `horizontal` are orientation words, and a wall that
 * a footprint simplification left leaning by a thousandth of a degree is a wall.
 * The six DIRECTION words need no tolerance at all: they partition on which axis
 * dominates, so every face has one.
 */
inline constexpr double kComponentAxisToleranceDegrees = 1.0;

/**
 * @brief Out-of-plane deviation, in scope units, above which a face is called non-planar
 *
 * A nanometre. Below it the deviation is rounding in the vertex arithmetic that
 * built the face; above it the face really is bent and a caller may want to say
 * so. It is a REPORTING threshold only -- nothing is flattened or rejected.
 */
inline constexpr double kComponentPlanarityEpsilon = 1e-9;

/**
 * @brief Grid the ordering key is quantised to, in scope units
 *
 * A micrometre: far below any architectural distinction and far above the
 * double-precision noise of a shape whose coordinates are metres. Two components
 * whose centroids differ by less than this are ordered by the next key instead,
 * which keeps the order from flapping when an unrelated operation perturbs the
 * last bits of a coordinate.
 */
inline constexpr double kComponentOrderQuantum = 1e-6;

/**
 * @brief |normal . y| beyond which a face has no usable in-plane "up"
 *
 * The pole of the face frame. A face this close to horizontal takes its in-plane
 * y from the parent's +z instead of the parent's +y. Set within a whisker of 1
 * so that the frame of an almost-vertical wall is still a wall's frame: at this
 * value the fallback engages only inside about 0.0026 degrees of horizontal,
 * where the projected up vector still has a length of 4.5e-5 and normalises
 * without trouble.
 */
inline constexpr double kComponentPoleCosine = 1.0 - 1e-9;

/// Area below which a face is treated as having none at all, in squared scope units
inline constexpr double kComponentAreaEpsilon = 1e-12;

/// Length below which an edge is treated as having none at all, in scope units
inline constexpr double kComponentEdgeEpsilon = 1e-9;

// ============================================================================
// Selection
// ============================================================================

/**
 * @brief Which components a split claims
 *
 * Three independent narrowings, applied in this order and each optional:
 *
 *   1. @p selector, a word from ast.hpp's ComponentSelector. `All` admits every
 *      non-degenerate component.
 *   2. the angle filter, when @p use_angle is set.
 *   3. @p indices, when it is non-empty, addressing the SURVIVORS of 1 and 2 in
 *      the stable order.
 *
 * Applying the indices last is what makes `{front, indices {1}}` mean "the
 * second front-facing component" and `{All, indices {3}}` mean "component 3 of
 * the shape". Both readings are wanted, and one field gives both.
 */
struct ComponentSelection {
    /// Direction or orientation word. ComponentSelector::All admits everything.
    ComponentSelector selector = ComponentSelector::All;

    /// Narrow by the angle between the component normal and a scope axis
    bool use_angle = false;

    /// The axis the angle is measured to, as its POSITIVE direction
    ScopeAxis angle_axis = ScopeAxis::Y;

    /// Inclusive bounds in degrees, within [0, 180]. 0 is along +axis, 180 against it.
    double angle_min_degrees = 0.0;
    double angle_max_degrees = 180.0;

    /**
     * @brief Positions to keep, in the stable order of whatever survived above
     *
     * Empty keeps everything. An entry past the end is REPORTED, not dropped: a
     * rule asking for the fourth wall of a triangular tower has a mistake in it,
     * and silently producing three walls hides it.
     */
    std::vector<uint32_t> indices;

    /// Tolerance for the `vertical`, `horizontal`, `aslant` words
    double axis_tolerance_degrees = kComponentAxisToleranceDegrees;
};

/// A selection by angle to an axis, which is four fields written one way round
[[nodiscard]] ComponentSelection select_angle(ScopeAxis axis,
                                              double min_degrees,
                                              double max_degrees);

// ============================================================================
// Result
// ============================================================================

/**
 * @brief One produced component
 *
 * @p shape is the component as a shape in its own right: its own oriented scope,
 * its own geometry in that scope, and the parent's attributes. Its tree fields
 * -- id, parent, depth, index, seed_key -- are NOT set here, because only the
 * interpreter can hand out an address in the shape tree. emit_components() does
 * that; a caller using split_components() directly gets a shape with the
 * defaults and is expected to know it.
 */
struct Component {
    Shape shape;

    /// Position in the parent's stable component order, before any filtering
    uint32_t index = 0;

    /// Component normal in the PARENT's local frame, which is what the selectors read
    glm::dvec3 normal{0.0};

    /// Area for a face, length for an edge
    double measure = 0.0;

    /// Worst out-of-plane deviation of the face's vertices, in scope units. 0 for an edge.
    double planarity = 0.0;
};

/**
 * @brief What a split found, and what was wrong with it
 *
 * Separate from the components themselves so that a caller which only wants the
 * shapes can pass nullptr, and so that a caller which wants diagnostics gets
 * SENTENCES rather than having to compose them from counts. Each entry of
 * @p problems is a complete message fragment ready to hand to
 * Interpreter::report(), with no leading capital and no trailing full stop, the
 * same shape OpResult::message has.
 */
struct ComponentSplitReport {
    /// Components the domain found, degenerate ones excluded. The numbering space.
    uint32_t total = 0;

    /// Components that survived the selection
    uint32_t selected = 0;

    /// Dropped for having no normal, no area or no length
    uint32_t degenerate = 0;

    /// Faces whose vertices do not lie in one plane
    uint32_t non_planar = 0;

    /// Largest out-of-plane deviation seen, in scope units
    double worst_planarity = 0.0;

    /**
     * @brief One sentence fragment per fault, in the order the faults were found
     *
     * FAULTS, not observations: a domain this build does not implement, an empty
     * angle range, an index past the end, a shape with no geometry. Every one of
     * them is a mistake in the rule, so emit_components() reports them as
     * Errors. The degenerate and non-planar COUNTS above are observations about
     * the input geometry, are not entered here, and are reported as Warnings.
     */
    std::vector<std::string> problems;
};

// ============================================================================
// Classification
// ============================================================================

/**
 * @brief Which of the six direction words a normal answers to
 *
 * The axis with the largest magnitude wins, and its sign picks the word. That
 * makes the six words a PARTITION of every non-degenerate component: a rule
 * listing all six covers the shape with nothing left over and nothing counted
 * twice, which is what lets `select` do without an `else`.
 *
 * Exact ties are broken y, then x, then z -- a normal that is equally up and
 * sideways is called `top`. A face at exactly 45 degrees is a roof pitch far
 * more often than it is a leaning wall, and the tie-break has to be written down
 * somewhere or it is whatever the comparison order happened to be.
 *
 * @param normal_local Normal in the SHAPE'S scope, not in the world
 * @return One of Front, Back, Left, Right, Top, Bottom; or All when @p
 *         normal_local is not a direction at all, which is how a degenerate
 *         component answers
 */
[[nodiscard]] ComponentSelector component_direction(const glm::dvec3& normal_local);

/**
 * @brief Does a component with this normal answer to this word?
 *
 * The six direction words go through component_direction(). The orientation
 * words are measured against the scope's y:
 *
 *   - `vertical`   -- the face plane contains y, within @p axis_tolerance_degrees
 *   - `horizontal` -- the face plane is perpendicular to y, within the same
 *   - `aslant`     -- neither, so: a roof pitch
 *   - `side`       -- direction is one of front, back, left, right
 *
 * `side` and `vertical` agree on a box and disagree on a frustum, whose four
 * walls are sides but are not vertical. That difference is the reason ast.hpp
 * carries both words, and it is asserted in the tests.
 *
 * No inverse trigonometry is used anywhere in this comparison. The bounds are
 * turned into COSINES once, through shape.hpp's deg_sin_cos(), which is exact on
 * every quarter turn; comparing angles instead would put a libm call on the
 * wrong side of a boolean, and a classification that disagrees between two C
 * libraries is a building that differs between two machines.
 */
[[nodiscard]] bool selector_admits(ComponentSelector selector,
                                   const glm::dvec3& normal_local,
                                   double axis_tolerance_degrees =
                                       kComponentAxisToleranceDegrees);

/**
 * @brief The oriented frame for a face component, as columns in the PARENT's local frame
 *
 * Columns are x, y and z of the component's scope: z is the face normal, y is
 * the in-plane up, x is across. Right-handed by construction. See point 2 of the
 * file header for what each one is for and what happens at the pole.
 *
 * Exposed because it is the one piece of this file that a test can assert
 * exactly, and because D4's roof operations want the same frame.
 *
 * @param normal_local Face normal in the parent's local frame; need not be unit
 * @return Orthonormal columns, or the identity when @p normal_local is degenerate
 */
[[nodiscard]] glm::dmat3 face_component_axes(const glm::dvec3& normal_local);

/**
 * @brief The oriented frame for an edge component, as columns in the PARENT's local frame
 *
 * x along the edge, z along the face normal, y = cross(z, x) pointing into the
 * face. @p direction_local is orthogonalised against @p normal_local rather than
 * trusted, because an edge of a non-planar face is not exactly in the face's
 * plane and an un-orthogonalised frame makes to_local() -- a transpose, not an
 * inverse -- quietly wrong.
 *
 * @return Orthonormal columns, or the identity when either input is degenerate
 */
[[nodiscard]] glm::dmat3 edge_component_axes(const glm::dvec3& direction_local,
                                             const glm::dvec3& normal_local);

// ============================================================================
// Spellings
// ============================================================================

/**
 * @brief Parse a selector word as an author writes it
 *
 * Accepts exactly the spellings ast.hpp's component_selector_name() produces, so
 * the language has one vocabulary and not two.
 *
 * @return false when @p text names no selector; @p out is untouched
 */
[[nodiscard]] bool parse_component_selector(const std::string& text, ComponentSelector& out);

/// Parse a domain word: "face", "edge", "vertex", "object". False when unknown.
[[nodiscard]] bool parse_component_domain(const std::string& text, ComponentDomain& out);

// ============================================================================
// The split
// ============================================================================

/**
 * @brief Split a shape into the selected components of a domain
 *
 * `Face` decomposes a solid into its faces and leaves a lone face as itself.
 * `Edge` decomposes every face into its boundary segments, INCLUDING the
 * segments of its holes.
 *
 * An edge shared by two faces is produced twice, once per face, and that is
 * deliberate rather than an oversight: the two copies have different scopes,
 * because an edge's frame is taken from the face it belongs to, and there is no
 * answer to "which of the two faces orients the shared edge" that does not
 * depend on the order the faces were built in -- which is exactly the dependency
 * point 3 of the header removes everywhere else.
 *
 * `Vertex` and `Object` are reported as unimplemented rather than returning
 * nothing. A point has no frame to orient a scope by without inventing one from
 * its neighbours, and `Object` addresses inserted assets, which this build has
 * no notion of. Both are rows in the language that D4 does not fill, and
 * ast.hpp's rule for those is that they say so at the line that asked.
 *
 * @param shape     Shape to decompose. Not modified.
 * @param domain    What to decompose into
 * @param selection Which components to keep
 * @param report    Receives the counts and the faults; may be nullptr
 * @return The selected components, in the stable order, each with an oriented
 *         scope. Empty when nothing was selected, which is not by itself a
 *         fault -- a shape with no top face has no top face.
 */
[[nodiscard]] std::vector<Component> split_components(
    const Shape& shape,
    ComponentDomain domain,
    const ComponentSelection& selection,
    ComponentSplitReport* report = nullptr);

/**
 * @brief Split @p shape and emit every selected component as a terminal child
 *
 * This is the body of the `case StmtKind::Select:` that interpreter.cpp does not
 * yet have, minus running each arm's block on the component. Every fault the
 * split found is reported at @p loc through the interpreter, so a bad index or a
 * bent facade turns into the same caret a syntax error gets.
 *
 * Each child is given its address in the tree by
 * Interpreter::emit_derived_terminal(), which derives its seed_key from the
 * parent's key and the child's sibling index -- never from a shared stream, so
 * the components of one shape do not move when an unrelated `choose` is added
 * somewhere else in the file.
 *
 * The children are emitted in the CANONICAL ORDER of point 3, and that is a
 * contract rather than an accident of the loop below. The sibling index is half
 * of what makes each child's seed_key, so emitting the same components in a
 * different order gives every one of them a different random address -- and two
 * runs that both do it would still dump identically, which is why the emission
 * order is asserted on its own rather than left to a determinism test.
 *
 * @param shape       Shape to decompose. Not modified.
 * @param domain      What to decompose into
 * @param selection   Which components to keep
 * @param loc         Where the rule asked, for the diagnostics
 * @param role_prefix Name recorded on each child, suffixed with its index, e.g.
 *                    "comp.face[2]"
 * @return How many children were emitted, which is fewer than were selected when
 *         the shape cap refused one
 */
uint32_t emit_components(Interpreter& interpreter,
                         const Shape& shape,
                         ComponentDomain domain,
                         const ComponentSelection& selection,
                         const SourceLoc& loc,
                         const std::string& role_prefix = "comp");

// ============================================================================
// Registration
// ============================================================================

/**
 * @brief Add the `comp` expression namespace to a FunctionTable
 *
 * Three functions, each taking the selector word an author already knows from
 * `select`:
 *
 *   - `comp.count(selector)` / `comp.count(domain, selector)` -- how many there
 *     are. `if (comp.count("top") > 0)` is how a rule guards a roof.
 *   - `comp.area(selector)` / `comp.area(domain, selector)` -- their total area,
 *     or total length for edges. A facade rule sizes its glazing budget with it.
 *   - `comp.size(selector, axis)` -- the extent of the FIRST such component
 *     along one of ITS OWN axes. `comp.size("front", "x")` is the width of the
 *     front wall, measured across the wall rather than across the world, which
 *     is the number a facade rule actually needs.
 *
 * Every one of them fails the shape rather than returning zero when there is no
 * shape to read or when the selection is empty, for the reason interpreter.cpp's
 * fn_needs_shape() gives: zero is a plausible number, so it does not look like a
 * fault -- it looks like a blank wall.
 *
 * A later registration replaces an earlier one, so a caller may override any of
 * these.
 */
void register_component_functions(FunctionTable& table);

/// standard_functions() with the `comp` namespace added. Built once, shared.
[[nodiscard]] const FunctionTable& component_functions();

} // namespace stratum::procgen::rules
