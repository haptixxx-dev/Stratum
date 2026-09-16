// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file shape.hpp
 * @brief The unit a rule operates on: geometry, a scope, attributes, and the operations that move them
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ================================================================================
 * WHAT A SHAPE IS
 * ================================================================================
 *
 * D1 gave the language a parser and nothing that runs. This file and
 * `interpreter.hpp` are what make it run, and of the two this is the one that
 * decides whether every later operation family is easy or impossible: D3 (split
 * and repeat), D4 (component split, roofs, primitives), D5 (stochastics) and D6
 * (attributes) all consume the Shape and the Scope defined here and nothing else
 * of D2.
 *
 * A shape is four things:
 *
 *   - **Geometry.** Polygonal faces, not triangles. `extrude` extrudes a face,
 *     `select face` picks faces, `offset` moves an outline. A triangle soup has
 *     no outline and no faces to select, so the working representation is
 *     polygons and the triangulation happens once, at the very end, in
 *     shape_to_mesh().
 *   - **A scope.** A local coordinate frame: an origin, three orthonormal axes
 *     and a size. Every operation reads it and most write it.
 *   - **Attributes.** Named values that inherit from parent to child.
 *   - **The rule that produced it**, plus where it sits in the shape tree.
 *
 * ================================================================================
 * THE TWO INVARIANTS
 * ================================================================================
 *
 * Everything in this file exists to keep these true. They are stated here because
 * an operation that breaks one of them does not fail where it is written -- it
 * fails three operations later, in a split that cuts empty air.
 *
 * ### 1. Geometry is stored in SCOPE-LOCAL coordinates
 *
 * `ShapeGeometry::positions` are in the scope's frame, not in world space. World
 * space is `scope.to_world(local)`. The alternative -- world-space vertices with
 * the scope as a bounding frame beside them -- was rejected because every
 * operation then carries a rotation: a split along the scope's x axis becomes a
 * plane clip with an arbitrary normal instead of an interval cut, and the
 * coordinates that a rule author reasons about ("this wall is 3 m wide") are
 * never the numbers in the buffer. Local storage makes the common path pure
 * axis-aligned arithmetic, which is both simpler and more reproducible: see the
 * determinism note below.
 *
 * The cost is that any change to the frame has to re-express the vertices, and
 * that is exactly what reframe() is for. It is the only function permitted to
 * change `Scope::origin` or `Scope::axes` while geometry exists.
 *
 * ### 2. The scope box is TIGHT around the geometry, with its minimum at local zero
 *
 * After every operation in this file, a shape with geometry satisfies
 *
 *     min(positions) == (0, 0, 0)   and   max(positions) == scope.size
 *
 * to within rounding. So `scope.size` is always the real extent of the shape and
 * `scope.size.y` is always its height, with no operation able to leave a stale
 * value behind.
 *
 * The rejected alternative was CGA's: the scope is whatever the last scope
 * operation set it to, and the geometry may be smaller, larger or entirely
 * outside it. That is more expressive -- it is how CGA's `center()` and its
 * asset insertion work -- and it is also how a scope goes stale. A `split(x)`
 * against a box that no longer contains the geometry silently produces children
 * with no geometry in them, and the author sees a missing floor rather than an
 * error. A derived box cannot go stale.
 *
 * Two consequences the reader should expect, rather than discover:
 *
 *   - `center` is not implementable here and is deliberately not implemented.
 *     Geometry is always already centred in a tight box. It belongs with
 *     `insert` and `primitive` in D4, which are the operations that can put a
 *     small thing inside a large scope.
 *   - `scope { }` restores the AXES and the PIVOT, not the box. The box is a
 *     derived quantity, so it is recomputed from the geometry on the way out.
 *     What the block protects is the orientation, against a `rotate_scope` or
 *     an `align_scope` inside it, which is what push/pop was ever used for.
 *
 * The one exception is a shape with NO geometry: its scope is whatever was set,
 * untouched, so that D4 can build a scope and then insert an asset into it.
 *
 * ================================================================================
 * DETERMINISM
 * ================================================================================
 *
 * Same rule text, same seed, same input shape, byte-identical output on any
 * platform. Three rules, the same three that osm/road/lots.hpp documents, and for
 * the same reasons:
 *
 *   - **The seed is mixed PER SHAPE from the shape's address in the tree**, never
 *     drawn from one advancing stream. Shape::seed_key is that address.
 *     seed_mix2() is a finaliser, not a generator: there is no state to advance
 *     and therefore no stream whose position can leak from one shape to the next.
 *     With a shared stream, adding one `choose` at the top of a rule file changes
 *     every shape generated after it, and a bug report becomes unreproducible.
 *   - **No hash container is ever read in iteration order.** Attributes are a
 *     std::map, faces are a vector, and the offset code classifies rings by the
 *     sign of their area rather than by the order a library happened to return
 *     them in.
 *   - **libm touches a coordinate in exactly one place**, and it is named:
 *     rotation. `rotate` and `rotate_scope` need a sine and a cosine of an
 *     author-supplied angle, and there is no way around that. deg_sin_cos()
 *     therefore returns EXACT values for every multiple of 90 degrees -- which is
 *     what rule files overwhelmingly ask for, and what the tests ask for -- and
 *     falls through to std::sin and std::cos otherwise. Everything else in this
 *     file is +, -, *, / and sqrt, which IEEE-754 rounds identically everywhere.
 *
 * ================================================================================
 * WHY NOT osm/mesh_builder
 * ================================================================================
 *
 * The brief asks for reuse rather than a third extruder, and the honest answer is
 * that MeshBuilder does not fit: `build_building_mesh(const Building&)` takes an
 * OSM entity, extrudes upward in world metres from a tag-derived height, and
 * emits triangles. It has no notion of a scope, no face topology to hand back to
 * a later `select face`, and no way to extrude along anything but world up. What
 * IS reused is everything below the extruder: renderer/mesh.hpp for the output
 * type, mapbox::earcut for the triangulation (the same call MeshBuilder makes)
 * and Clipper2 for polygon offsetting (the same integer-scaled Path64 pattern
 * osm/road/zoning.cpp established).
 */

#pragma once

#include "procgen/rules/lexer.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <cstdint>
#include <map>
#include <string>
#include <variant>
#include <vector>

namespace stratum::procgen::rules {

// ============================================================================
// Deterministic mixing
//
// SplitMix64, integer only, so it is bit-identical on every platform. Taken from
// the same design as osm/road/lots.cpp and kept separate rather than shared
// because that copy is in an anonymous namespace in a file stratum_core's rule
// layer must not depend on.
// ============================================================================

/// SplitMix64 finaliser. No state, no stream, no position to leak.
[[nodiscard]] uint64_t seed_mix64(uint64_t x) noexcept;

/// Combine two 64-bit values into one, order-sensitive
[[nodiscard]] uint64_t seed_mix2(uint64_t a, uint64_t b) noexcept;

/**
 * @brief A value in [0, 1) from a hash
 *
 * 53 bits scaled by an exact power of two, so the conversion introduces no
 * rounding of its own and is identical everywhere. The upper bound is strict,
 * which the weighted pick in `choose` relies on: at exactly 1.0 the last arm
 * would be unreachable.
 */
[[nodiscard]] double seed_unit(uint64_t hash) noexcept;

/**
 * @brief Salts, so two draws made at one shape cannot collide
 *
 * Every draw is `seed_unit(seed_mix2(shape.seed_key, salt))`. The shape's key is
 * the only state.
 */
enum : uint64_t {
    kSaltRoot = 0x01,    ///< The root shape's key, mixed with the run seed
    kSaltChild = 0x02,   ///< A child's key, mixed with its sibling index
    kSaltChoose = 0x03,  ///< The weighted pick in a `choose` statement
    kSaltOp = 0x04       ///< Reserved for an operation that needs its own draw
};

// ============================================================================
// Scope
// ============================================================================

/// An axis of the scope's own frame. Same order as ast.hpp's SplitAxis.
enum class ScopeAxis : uint8_t { X, Y, Z };

/// Spell an axis for a message, as the author writes it
[[nodiscard]] const char* scope_axis_name(ScopeAxis axis);

/**
 * @brief Parse an axis name written by the author
 * @param text One of "x", "y", "z", case-sensitive
 * @param out  Receives the axis when the name is known
 * @return false when @p text names no axis; @p out is untouched
 */
[[nodiscard]] bool parse_scope_axis(const std::string& text, ScopeAxis& out);

/**
 * @brief The shape's local coordinate frame
 *
 * `world = origin + axes * local`, and because @p axes is orthonormal the
 * inverse is a transpose rather than an inversion -- which matters, because
 * every scope operation runs it and a general 3x3 inverse of a near-singular
 * matrix is where NaNs come from.
 */
struct Scope {
    /// World position of local (0,0,0), which is the minimum corner of the box
    glm::dvec3 origin{0.0};

    /**
     * @brief Local axes as COLUMNS, in world space
     *
     * `axes[0]`, `axes[1]`, `axes[2]` are the unit x, y and z axes. Orthonormal
     * and right-handed; orthonormalise() restores that after an operation that
     * could have drifted.
     */
    glm::dmat3 axes{1.0};

    /// Extent along each local axis. Never negative. May be zero on an axis: a
    /// footprint is a perfectly ordinary shape with size.y == 0.
    glm::dvec3 size{0.0};

    /**
     * @brief The point `rotate`, `scale` and `mirror` work about, as a fraction of the box
     *
     * (0,0,0) is the box minimum, (0.5,0.5,0.5) its centre, (1,1,1) its maximum.
     *
     * Stored as a FRACTION rather than as a local point on purpose. The box is
     * tight to the geometry (see the header note), so it is recomputed by almost
     * every operation; a pivot held as a local point would silently stop meaning
     * "the centre" the first time the geometry grew, and the symptom would be a
     * rotation about a drifting point several operations later.
     */
    glm::dvec3 pivot{0.0};

    [[nodiscard]] glm::dvec3 to_world(const glm::dvec3& local) const {
        return origin + axes * local;
    }

    [[nodiscard]] glm::dvec3 to_local(const glm::dvec3& world) const {
        return glm::transpose(axes) * (world - origin);
    }

    /// The pivot as a point in local coordinates
    [[nodiscard]] glm::dvec3 pivot_local() const { return pivot * size; }

    /// One local axis as a world-space unit vector
    [[nodiscard]] glm::dvec3 axis(ScopeAxis which) const {
        return axes[static_cast<int>(which)];
    }

    /// One component of @p size, by axis
    [[nodiscard]] double extent(ScopeAxis which) const {
        return size[static_cast<int>(which)];
    }

    /**
     * @brief Re-orthonormalise the axes by Gram-Schmidt
     *
     * Repeated rotations accumulate error, and an axis set that is no longer
     * orthonormal makes to_local() -- a transpose, not an inverse -- wrong by an
     * amount that grows silently. Called at the end of every operation that
     * touches the axes.
     */
    void orthonormalise();
};

// ============================================================================
// Geometry
// ============================================================================

/**
 * @brief One polygonal face
 *
 * ### Winding is the contract
 *
 * @p loop is counter-clockwise seen from OUTSIDE the solid, so the right-hand
 * rule gives the outward normal. @p holes are wound the opposite way. Every
 * function here maintains that, and face_normal() reads it.
 *
 * A convention that is merely documented gets broken, so the two functions that
 * accept author-supplied rings -- shape_from_polygon() and shape_from_rings() --
 * FIX the winding rather than trusting it. There is no way to construct a shape
 * with an inside-out base face by handing in a ring the wrong way round.
 */
struct Face {
    /// Outer ring, first vertex NOT repeated at the end
    std::vector<uint32_t> loop;

    /// Interior rings, each first vertex NOT repeated, wound opposite to @p loop
    std::vector<std::vector<uint32_t>> holes;

    /// Material slot and variant, in renderer/mesh.hpp's vocabulary
    MaterialKey material{};
};

/**
 * @brief Positions and faces, in scope-local coordinates
 *
 * Doubles, not floats. The whole operation chain -- offset, taper, split,
 * component split -- compounds, and a float lot boundary at city-scale metres
 * loses the millimetre the next offset needs. The conversion to float happens
 * once, in shape_to_mesh().
 */
struct ShapeGeometry {
    std::vector<glm::dvec3> positions;
    std::vector<Face> faces;

    [[nodiscard]] bool empty() const { return faces.empty() || positions.empty(); }

    void clear() {
        positions.clear();
        faces.clear();
    }
};

/// Outward unit normal of a face, by Newell's method. (0,0,0) for a degenerate face.
[[nodiscard]] glm::dvec3 face_normal(const ShapeGeometry& geometry, const Face& face);

/// Area of a face with its holes subtracted, never negative
[[nodiscard]] double face_area(const ShapeGeometry& geometry, const Face& face);

/// Sum of every face's area. For a closed solid this is its surface area.
[[nodiscard]] double geometry_area(const ShapeGeometry& geometry);

/**
 * @brief Enclosed volume, by the divergence theorem
 *
 * Meaningful only for a closed solid with outward-facing normals -- which is
 * what extrude() and taper() produce, and what the tests check. An open shape
 * such as a bare footprint returns a number with no meaning rather than an
 * error, because "is this mesh closed" is a more expensive question than the
 * answer is worth here.
 */
[[nodiscard]] double geometry_volume(const ShapeGeometry& geometry);

/**
 * @brief Axis-aligned bounds of the positions, in local coordinates
 * @return false when there are no positions; @p min and @p max are untouched
 */
[[nodiscard]] bool geometry_bounds(const ShapeGeometry& geometry,
                                   glm::dvec3& min,
                                   glm::dvec3& max);

// ============================================================================
// Values
// ============================================================================

struct Value;

/// An array value. std::vector of an incomplete type is well defined since C++17.
using ValueArray = std::vector<Value>;

/**
 * @brief A value in the rule language: a number, a boolean, a string or an array
 *
 * There is no integer type, by the same argument ast.hpp makes for TypeRef: every
 * quantity in a shape grammar is a length, an angle or a weight. Where an
 * operation needs a whole number -- an array index, a face index -- it rounds and
 * says so.
 */
struct Value {
    std::variant<double, bool, std::string, ValueArray> data{0.0};

    [[nodiscard]] static Value number(double v) { return Value{v}; }
    [[nodiscard]] static Value boolean(bool v) { return Value{v}; }
    [[nodiscard]] static Value text(std::string v) { return Value{std::move(v)}; }
    [[nodiscard]] static Value array(ValueArray v) { return Value{std::move(v)}; }

    [[nodiscard]] bool is_number() const { return data.index() == 0; }
    [[nodiscard]] bool is_bool() const { return data.index() == 1; }
    [[nodiscard]] bool is_text() const { return data.index() == 2; }
    [[nodiscard]] bool is_array() const { return data.index() == 3; }

    /// Valid only when is_number()
    [[nodiscard]] double as_number() const { return std::get<double>(data); }
    /// Valid only when is_bool()
    [[nodiscard]] bool as_bool() const { return std::get<bool>(data); }
    /// Valid only when is_text()
    [[nodiscard]] const std::string& as_text() const { return std::get<std::string>(data); }
    /// Valid only when is_array()
    [[nodiscard]] const ValueArray& as_array() const { return std::get<ValueArray>(data); }

    /// "float", "bool", "string" or "array", for a diagnostic
    [[nodiscard]] const char* type_name() const;

    /// Canonical text, for print() and for a diagnostic. Deterministic; see format_number().
    [[nodiscard]] std::string to_text() const;
};

/**
 * @brief Format a double the same way on every platform
 *
 * `%.10g` with negative zero normalised to zero. Ten significant digits is
 * enough to distinguish any two numbers a rule author typed and few enough that
 * 0.1 + 0.2 prints as 0.3, which is what print() is for. Negative zero is
 * normalised because -0.0 == 0.0 compares true but prints differently, and a
 * dump that differs for two runs that agree is a false failure.
 */
[[nodiscard]] std::string format_number(double value);

// ============================================================================
// Shape
// ============================================================================

/// "No shape". Same choice of sentinel, for the same reason, as ast.hpp's kNoNode.
inline constexpr uint32_t kNoShape = 0xFFFFFFFFu;

/**
 * @brief The unit a rule operates on
 *
 * Shapes form a tree: an operation consumes one and produces children. The tree
 * is not stored as a container of nodes -- the interpreter recurses and keeps
 * only the leaves -- so @p id and @p parent let a caller rebuild the parentage
 * from the terminals alone, which is what an editor needs to answer "which rule
 * made this wall".
 */
struct Shape {
    Scope scope{};
    ShapeGeometry geometry{};

    /**
     * @brief Named values, inherited by children
     *
     * std::map, not unordered_map: attributes are iterated when a shape is
     * dumped and when a child copies its parent, and a hash order is an
     * allocation accident that would make two runs differ.
     */
    std::map<std::string, Value> attributes;

    /// Name of the rule that produced this shape. Empty on the shape handed to generate().
    std::string rule;

    /// Where the call that produced this shape was written. Carried into runtime diagnostics.
    SourceLoc loc{};

    /**
     * @brief This shape's address in the tree, and the only random state it has
     *
     * Derived from the parent's key and this shape's sibling index, never from a
     * shared stream. See the determinism note at the top of the file.
     */
    uint64_t seed_key = 0;

    uint32_t depth = 0;   ///< Rule-call depth. The root shape is 0.
    uint32_t index = 0;   ///< Sibling index under the parent, from 0
    uint32_t id = kNoShape;      ///< Unique within one generate() call
    uint32_t parent = kNoShape;  ///< @p id of the parent, kNoShape at the root

    /// A draw in [0, 1) that depends only on where this shape is in the tree
    [[nodiscard]] double draw(uint64_t salt) const {
        return seed_unit(seed_mix2(seed_key, salt));
    }
};

// ============================================================================
// Frame maintenance
// ============================================================================

/**
 * @brief Turn the scope's axes while leaving the geometry where it is in the world
 *
 * The only function allowed to write Scope::axes on a shape that has geometry.
 * It re-expresses every position in the new frame, so the world positions are
 * unchanged to within rounding, and then refits the box.
 *
 * There is no origin parameter, and that is the point: under invariant 2 the
 * origin is a DERIVED quantity -- the world position of the box minimum -- so an
 * origin passed in here would be overwritten by the refit two lines later, and a
 * caller would have to know that to avoid being surprised by it.
 *
 * @param shape    Shape to reframe
 * @param new_axes New axes as columns; orthonormalised on the way in, keeping
 *                 whatever handedness they were given (see mirror_scope_shape())
 */
void reframe(Shape& shape, const glm::dmat3& new_axes);

/**
 * @brief Restore invariant 2: shift local coordinates so the tight box starts at zero
 *
 * Recomputes Scope::size from the geometry and moves Scope::origin so that the
 * minimum corner is local (0,0,0). Keeps the axes and the pivot fraction.
 *
 * Every operation in this file ends with this call. It is idempotent, and a
 * no-op on a shape with no geometry -- which is the documented exception to the
 * tight-box invariant and is what lets D4 build an empty scope to insert into.
 */
void refit_scope(Shape& shape);

// ============================================================================
// Construction
// ============================================================================

/**
 * @brief A shape from a ring in the local xz plane, lying at local y
 *
 * The workhorse: this is what a lot boundary, a block face and a test square all
 * become. The ring's winding does NOT matter -- the resulting face is oriented
 * so its normal is +y, which is what extrude() and taper() expect and what makes
 * "up" mean up.
 *
 * @param ring Boundary, first point not repeated. Fewer than three points gives
 *             an empty shape rather than a degenerate face.
 * @param y    Height of the plane in local coordinates
 * @return A shape with one face and a tight scope; size.y is 0
 */
[[nodiscard]] Shape shape_from_polygon(const std::vector<glm::dvec2>& ring, double y = 0.0);

/**
 * @brief As shape_from_polygon(), with interior rings
 *
 * Holes are re-wound opposite to the outer ring. A hole with fewer than three
 * points is dropped.
 */
[[nodiscard]] Shape shape_from_rings(const std::vector<glm::dvec2>& outer,
                                     const std::vector<std::vector<glm::dvec2>>& holes,
                                     double y = 0.0);

/// An axis-aligned rectangle in the xz plane with its minimum corner at the origin
[[nodiscard]] Shape shape_from_rect(double size_x, double size_z, double y = 0.0);

// ============================================================================
// Operation results
// ============================================================================

/**
 * @brief What an operation did, or why it could not do all of it
 *
 * An operation that cannot run is a USER error -- a taper that eats the whole
 * outline, an extrude of zero -- and the caller turns this into a diagnostic
 * carrying the AST location of the call. It is not an exception and not a
 * silent no-op: the first hides the line number, the second hides the mistake.
 *
 * A failure does NOT promise the shape is untouched. scale_shape() applies what
 * it can and reports the axis it had to skip, because refusing the whole call
 * over one flat axis would be a worse answer than a partial one with a message.
 *
 * @p message never ends with a full stop and never names the operation. The
 * interpreter prefixes the operation name, so the two do not have to agree.
 */
struct OpResult {
    bool ok = true;
    std::string message;

    [[nodiscard]] static OpResult success() { return OpResult{}; }
    [[nodiscard]] static OpResult failure(std::string why) {
        return OpResult{false, std::move(why)};
    }

    explicit operator bool() const { return ok; }
};

// ============================================================================
// Geometry operations (D2)
// ============================================================================

/**
 * @brief Extrude every face into a closed prism
 *
 * Each face becomes a solid: a cap at the far end, a reversed cap where the face
 * was, and a quad per boundary edge including the edges of every hole. A shape
 * with several faces becomes several prisms, which is the only reading that does
 * not require choosing one of them.
 *
 * Winding is handled rather than assumed: a face whose normal opposes @p axis is
 * flipped before the prism is built, and a negative @p distance reverses every
 * resulting face. So `extrude(3)` and `extrude(-3)` both produce a solid with
 * outward normals, and geometry_volume() is positive for both.
 *
 * @param shape    Shape to extrude, modified in place
 * @param axis     Scope axis to extrude along
 * @param distance Signed distance in scope units. Zero is an error: it would
 *                 produce two coincident caps and a ring of zero-area quads,
 *                 which is not a shape and is not what the author meant.
 * @return failure when @p distance is zero, or when a face lies parallel to
 *         @p axis so that the "prism" would be flat
 */
[[nodiscard]] OpResult extrude_shape(Shape& shape, ScopeAxis axis, double distance);

/**
 * @brief Extrude along the shape's own dominant face normal
 *
 * `extrude("normal", d)`. The dominant face is the one with the largest area,
 * ties broken by the lowest face index so the choice does not depend on anything
 * but the geometry.
 */
[[nodiscard]] OpResult extrude_shape_along_normal(Shape& shape, double distance);

/// What part of an offset the shape keeps
enum class OffsetSelector : uint8_t {
    Inside,  ///< The offset outline alone. The default, and what "inset" means.
    Border,  ///< The ring between the original outline and the offset one
    All      ///< Both, as separate faces
};

/**
 * @brief Parse an offset selector written by the author
 * @return false when @p text names no selector
 */
[[nodiscard]] bool parse_offset_selector(const std::string& text, OffsetSelector& out);

/**
 * @brief Move each face's outline in or out within its own plane
 *
 * A negative @p distance insets, which is what the language spells `inset`
 * everywhere else and what `offset(-d)` means in CGA. There is no separate inset
 * operation for that reason: one sign convention is easier to remember than two
 * names, and ast.hpp's catalogue has one row.
 *
 * Each face is offset in ITS OWN plane, so a flat footprint does the obvious
 * thing and a box offsets all six faces independently. That is the only
 * definition that needs no extra argument to say which face was meant.
 *
 * ### Why Clipper2 here and hand-rolled mitring in taper_shape()
 *
 * Offsetting can change the topology: inset a dumbbell far enough and it becomes
 * two polygons; inset a polygon with a hole and the hole can swallow it. Clipper2
 * is the project's answer to that and is already a dependency (see
 * osm/road/zoning.cpp for the same integer-scaled Path64 pattern). taper_shape()
 * cannot use it, because it needs a vertex-for-vertex correspondence between the
 * base ring and the tapered ring to build the side walls, and a robust offset is
 * exactly the thing that does not preserve vertex count.
 *
 * @param shape    Shape to offset, modified in place
 * @param distance Signed distance in scope units; negative insets
 * @param selector Which part to keep
 * @return failure when the offset removes everything the selector asked for
 */
[[nodiscard]] OpResult offset_shape(Shape& shape, double distance, OffsetSelector selector);

/**
 * @brief Inset the outline and hand the removed border back as its own geometry
 *
 * This is the whole difference between `setback` and `offset(-d)`: the border is
 * kept. The interpreter emits it as a terminal child shape, so a rule that sets
 * a building back from the street still has the pavement strip to work with.
 *
 * @param shape    Shape to set back, modified in place to the inner part
 * @param distance Setback distance; must be positive, since a setback that grows
 *                 the shape has no border to hand back
 * @param border   Receives the border ring geometry, in the SAME local frame the
 *                 shape had on entry. Cleared first. Empty when @p keep_border
 *                 is false or when the border is degenerate.
 * @param keep_border false to discard the border, making this offset(-d) with a
 *                 clearer name at the call site
 * @return failure when @p distance is not positive or the setback removes
 *         everything
 */
[[nodiscard]] OpResult setback_shape(Shape& shape,
                                     double distance,
                                     ShapeGeometry& border,
                                     bool keep_border = true);

/**
 * @brief Extrude with a narrowing top: a frustum whose sides slope at 45 degrees
 *
 * The top ring is the base ring inset by @p height, so the sides rise at 45
 * degrees and the operation needs one argument, which is what ast.hpp's
 * catalogue gives it.
 *
 * The inset is a per-edge miter -- each edge is moved inward along its in-plane
 * normal and consecutive edge lines are intersected -- rather than a Clipper2
 * offset, because the side walls need one quad per base edge and therefore need
 * the vertex count preserved. See the note on offset_shape().
 *
 * A miter offset is not robust, and that is handled rather than ignored: if any
 * edge reverses direction, the outline has folded through itself and the
 * operation FAILS with a message naming the height, instead of emitting a
 * self-intersecting solid that looks right until it is exported.
 *
 * @param shape  Shape to taper, modified in place
 * @param height Height and inset distance. Must be positive.
 * @return failure when @p height is not positive or the inset collapses a face
 */
[[nodiscard]] OpResult taper_shape(Shape& shape, double height);

// ============================================================================
// Scope operations (D2)
// ============================================================================

/**
 * @brief Move the geometry within the scope
 *
 * ast.hpp's catalogue says "Move the geometry within the scope", and that is
 * taken literally: the vertices move by @p delta in scope units and the box then
 * refits around them, so the net effect in world space is a translation. CGA's
 * `t()` instead moves the scope and drags the geometry along, which is the same
 * thing for a shape whose box is tight and a different thing for one whose box
 * is not. Since the box here is always tight, the catalogue's wording is the one
 * that survives.
 */
void translate_shape(Shape& shape, const glm::dvec3& delta);

/**
 * @brief Rotate the geometry about the scope pivot
 *
 * Euler angles in DEGREES, applied about the local x axis, then y, then z. The
 * order is fixed and stated because Euler angles do not commute and a rule file
 * that was authored against one order is wrong under another.
 *
 * The axes do not move -- this rotates the thing inside the frame, not the frame
 * -- but the box does, because it is tight. Use rotate_scope_shape() to turn the
 * frame and leave the geometry alone.
 *
 * Multiples of 90 degrees are exact; see deg_sin_cos().
 */
void rotate_shape(Shape& shape, const glm::dvec3& degrees);

/**
 * @brief Rotate the scope's axes about the pivot, leaving the geometry in place
 *
 * The world positions of every vertex are unchanged. What changes is the frame
 * they are described in, and therefore what a later split or extrude means by
 * "x". The box refits, because the tight box of a rotated frame is a different
 * box.
 */
void rotate_scope_shape(Shape& shape, const glm::dvec3& degrees);

/**
 * @brief Resize the scope, and the geometry with it
 *
 * @p new_size is an ABSOLUTE size in scope units, not a factor: `scale(10, 3, 8)`
 * makes the shape ten metres by three by eight. That is CGA's `s()`, and it is
 * the form a rule author wants, because a facade rule knows the storey height it
 * needs and not the factor that would reach it. A factor is `scale(shape.sx * 2,
 * ...)`.
 *
 * An axis whose current extent is zero cannot be scaled -- a flat footprint has
 * no thickness to multiply -- so that axis is left alone and the call reports it.
 * The alternative, a division by zero, puts an infinity in a coordinate and the
 * failure surfaces as a missing mesh several operations later.
 *
 * @return failure when any component of @p new_size is negative, or when an axis
 *         had to be skipped for having no extent
 */
[[nodiscard]] OpResult scale_shape(Shape& shape, const glm::dvec3& new_size);

/// Where to put the scope pivot
enum class PivotAnchor : uint8_t {
    Origin,        ///< The box minimum. The default.
    Center,        ///< The box centre
    Max,           ///< The box maximum
    CenterBottom,  ///< Centred in x and z, at the minimum in y
    CenterTop      ///< Centred in x and z, at the maximum in y
};

/// Parse a pivot anchor name; false when @p text names none
[[nodiscard]] bool parse_pivot_anchor(const std::string& text, PivotAnchor& out);

/// Move the pivot. Changes nothing about the geometry, only what rotate and scale turn about.
void set_pivot_shape(Shape& shape, PivotAnchor anchor);

/// How align_scope chooses the new axes
enum class AlignMode : uint8_t {
    World,     ///< The world axes. Undoes every rotation the scope has accumulated.
    YUp,       ///< Keep y as world up; keep the current x as far as it is horizontal
    Geometry   ///< Align to the geometry: y along the dominant face's normal
};

/// Parse an align mode name; false when @p text names none
[[nodiscard]] bool parse_align_mode(const std::string& text, AlignMode& out);

/**
 * @brief Point the scope's axes somewhere useful, leaving the geometry in place
 *
 * `Geometry` is "align to geometry" in the brief's sense: the dominant face --
 * largest area, ties by lowest index -- gives y, its longest edge gives x, and z
 * follows from the cross product. Both tie-breaks are stated because a tie
 * broken by whichever face happened to be built first is a run-to-run
 * difference, and the whole point of this header is that there are none.
 *
 * @return failure when the mode cannot be satisfied: `Geometry` on a shape with
 *         no usable face, or `YUp` on a scope whose x axis is exactly vertical
 */
[[nodiscard]] OpResult align_scope_shape(Shape& shape, AlignMode mode);

/// Mirror the geometry about the scope plane through the pivot, normal @p axis
void mirror_shape(Shape& shape, ScopeAxis axis);

/// Negate one scope axis, leaving the geometry in place. Turns the frame left-handed.
void mirror_scope_shape(Shape& shape, ScopeAxis axis);

/// Flip every face, so what was outside is inside
void reverse_normals_shape(Shape& shape);

// ============================================================================
// Trigonometry
// ============================================================================

/**
 * @brief Sine and cosine of an angle in degrees, exact on the quarter turns
 *
 * The one place in this file where libm can touch a coordinate. Every multiple
 * of 90 degrees returns exactly 0, 1 or -1, so the axis-aligned rotations that
 * rule files are almost entirely made of are bit-identical everywhere, and so
 * are the tests. Other angles go to std::sin and std::cos, whose last bit is not
 * guaranteed to agree across C libraries -- a residual that is named here rather
 * than discovered later.
 *
 * @param degrees Angle
 * @param sin_out Receives the sine
 * @param cos_out Receives the cosine
 */
void deg_sin_cos(double degrees, double& sin_out, double& cos_out);

// ============================================================================
// Output
// ============================================================================

/**
 * @brief Triangulate a shape into a world-space Mesh
 *
 * Faces are triangulated with mapbox::earcut in each face's own plane, so a
 * concave face and a face with holes both work. Vertices are duplicated per
 * face, which gives flat shading -- correct for architecture, where a wall meets
 * a roof at a crease and a shared normal would round it off.
 *
 * UVs are a planar projection in the face's own basis, in metres, so a texture
 * tiles at real-world scale without a `setup_projection` call. D4's UV
 * operations replace them.
 *
 * @param shape Shape to convert
 * @return A mesh with bounds computed and tangents computed. Its `submeshes` is
 *         left empty when every face uses MaterialId::Default, which
 *         renderer/mesh.hpp defines to mean one implicit whole-mesh range.
 */
[[nodiscard]] Mesh shape_to_mesh(const Shape& shape);

/// Append a shape's triangles to an existing mesh. Bounds and tangents are the caller's.
void append_shape_to_mesh(const Shape& shape, Mesh& mesh);

/**
 * @brief Triangulate one face into local-space triangles
 *
 * Exposed because D3 and D4 need it for their own output and because it is the
 * one piece of this file with a failure mode worth testing on its own: earcut
 * returns nothing for a degenerate ring, and a caller that assumes success
 * writes an empty draw call.
 *
 * @param geometry Geometry the face indexes into
 * @param face     Face to triangulate
 * @return Triples of indices into `geometry.positions`. Empty when the face is
 *         degenerate or has fewer than three vertices.
 */
[[nodiscard]] std::vector<uint32_t> triangulate_face(const ShapeGeometry& geometry,
                                                     const Face& face);

/**
 * @brief A canonical text rendering of a shape, for tests and for a dump
 *
 * Deterministic and platform-independent: every number goes through
 * format_number(), attributes are iterated in the map's key order, and no
 * pointer, address or iteration accident reaches the output. Two runs that
 * agree produce byte-identical text, which is how the determinism tests are
 * written.
 */
[[nodiscard]] std::string dump_shape(const Shape& shape);

} // namespace stratum::procgen::rules
