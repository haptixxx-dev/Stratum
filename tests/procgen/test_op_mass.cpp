// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_op_mass.cpp
 * @brief Mass modelling (E2): the stacked prism, the floor structure, and the two refusals
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ================================================================================
 * HOW THESE TESTS ARE WRITTEN SO THAT THEY CAN FAIL
 * ================================================================================
 *
 * This project's most-found defect across nine review rounds is the test that
 * cannot fail, including a determinism suite resting on a `dump()` that was
 * itself geometry-blind. Every assertion below is written against something an
 * implementation could get wrong, and the notes say which wrong thing.
 *
 *   - **Volume, not face count.** `geometry_volume()` is the divergence theorem
 *     over the triangulated faces, so it is wrong for a solid with one wall
 *     wound backwards, a missing cap, or a terrace laid over the tower instead
 *     of around it. Every mass built here is checked against a volume computed
 *     by hand from the rule's own numbers. A face count alone passes for a solid
 *     that is inside out.
 *
 *   - **Surface area as well as volume.** A buried cap -- the defect the annulus
 *     terrace exists to prevent -- is invisible to the volume, because two
 *     stacked closed solids enclose exactly the sum of their volumes. It shows
 *     up in the area and in the face normals, so both are asserted.
 *
 *   - **The inset is checked in a frame that is NOT the origin.** inset_ring()
 *     sends a ring through a scratch Shape whose scope offset_shape() refits, and
 *     reads it back through `Scope::to_world()`. A version that read `positions`
 *     directly is correct for every ring whose minimum corner is already at zero
 *     and wrong by the inset distance for every other one, so the test uses a
 *     ring at x in [100, 120].
 *
 *   - **The rule-level proof uses deliberately WRONG fallbacks.** The facade
 *     reads `attrs.get("floor_height", 7.0)`, and seven is not the storey height.
 *     A run in which `floors` wrote no attributes therefore produces two storeys
 *     rather than six and fails here, instead of quietly passing on the fallback.
 *
 *   - **Every refusal is matched against the sentence the code really writes**,
 *     and the count is exact. A test that only asserts `!result.ok` passes when
 *     the operation fails for a different reason than the one being tested --
 *     which is how a podium test that was really testing "not a footprint" gets
 *     written.
 *
 *   - **The seed rectangles are asymmetric** -- 20 by 12, not 20 by 20, wherever
 *     a transposed build would otherwise land on the same numbers.
 *
 * Nothing here needs a GPU, a window or a file on disk, so nothing here skips.
 */

#include "framework.hpp"

#include "procgen/rules/ast.hpp"
#include "procgen/rules/interpreter.hpp"
#include "procgen/rules/lexer.hpp"
#include "procgen/rules/op_control.hpp"
#include "procgen/rules/op_mass.hpp"
#include "procgen/rules/parser.hpp"
#include "procgen/rules/shape.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

using stratum::procgen::rules::build_mass_stack;
using stratum::procgen::rules::control_functions;
using stratum::procgen::rules::control_operations;
using stratum::procgen::rules::CourtyardOptions;
using stratum::procgen::rules::CourtyardReport;
using stratum::procgen::rules::courtyard_shape;
using stratum::procgen::rules::Diagnostic;
using stratum::procgen::rules::extrude_shape;
using stratum::procgen::rules::Face;
using stratum::procgen::rules::face_area;
using stratum::procgen::rules::face_normal;
using stratum::procgen::rules::FloorPlan;
using stratum::procgen::rules::floors_shape;
using stratum::procgen::rules::FootprintFace;
using stratum::procgen::rules::GenerationOptions;
using stratum::procgen::rules::GenerationResult;
using stratum::procgen::rules::generate;
using stratum::procgen::rules::geometry_area;
using stratum::procgen::rules::geometry_bounds;
using stratum::procgen::rules::geometry_volume;
using stratum::procgen::rules::inset_ring;
using stratum::procgen::rules::kDefaultFloorHeight;
using stratum::procgen::rules::kMaxFloors;
using stratum::procgen::rules::MassTier;
using stratum::procgen::rules::mass_operations;
using stratum::MaterialKey;
using stratum::procgen::rules::OperationTable;
using stratum::procgen::rules::OpResult;
using stratum::procgen::rules::parse;
using stratum::procgen::rules::ParseResult;
using stratum::procgen::rules::PodiumPlan;
using stratum::procgen::rules::podium_shape;
using stratum::procgen::rules::read_footprint;
using stratum::procgen::rules::refit_scope;
using stratum::procgen::rules::register_mass_operations;
using stratum::procgen::rules::ring_area;
using stratum::procgen::rules::ring_newell_y;
using stratum::procgen::rules::ring_strictly_contains;
using stratum::procgen::rules::ScopeAxis;
using stratum::procgen::rules::Severity;
using stratum::procgen::rules::Shape;
using stratum::procgen::rules::shape_from_polygon;
using stratum::procgen::rules::shape_from_rect;
using stratum::procgen::rules::shape_from_rings;
using stratum::procgen::rules::shape_is_footprint;
using stratum::procgen::rules::ShapeGeometry;
using stratum::procgen::rules::solve_floor_plan;
using stratum::procgen::rules::solve_podium_plan;
using stratum::procgen::rules::standard_operations;

namespace {

// ============================================================================
// Helpers
// ============================================================================

/// Extrude a footprint so it stops being one. Reports whether shape.cpp agreed.
[[nodiscard]] bool extrude_shape_ok(Shape& shape, double distance = 5.0) {
    return extrude_shape(shape, ScopeAxis::Y, distance).ok;
}

/**
 * @brief A numeric attribute, or a number that fails every comparison
 *
 * Not `attributes.at()`, which THROWS when the attribute is missing and takes
 * the whole run with it -- a mutation that stopped `floors` writing its
 * structure aborted this suite instead of failing three tests. Not
 * `attributes[name]` either: that inserts a default-constructed Value, whose
 * number is zero, so an expectation of zero would pass on an absent attribute.
 */
[[nodiscard]] double attr_number(const Shape& shape, const std::string& name) {
    const auto found = shape.attributes.find(name);
    if (found == shape.attributes.end() || !found->second.is_number()) {
        return -1.0e300;
    }
    return found->second.as_number();
}

/// A square ring in the xz plane, corners (x0, z0) to (x1, z1)
[[nodiscard]] std::vector<glm::dvec2> rect_ring(double x0, double z0, double x1, double z1) {
    return {{x0, z0}, {x1, z0}, {x1, z1}, {x0, z1}};
}

/// Axis-aligned bounds of a ring in the xz plane
void ring_bounds(const std::vector<glm::dvec2>& ring, glm::dvec2& min_out, glm::dvec2& max_out) {
    min_out = glm::dvec2{1.0e300};
    max_out = glm::dvec2{-1.0e300};
    for (const glm::dvec2& p : ring) {
        min_out = glm::min(min_out, p);
        max_out = glm::max(max_out, p);
    }
}

/**
 * @brief Faces whose normal points along @p direction, to within a degree
 *
 * Used to count the horizontal faces of a mass. A stack built as two prisms one
 * on top of the other has THREE up-facing faces -- the roof, the terrace and the
 * cap buried under the tower -- where a correct stack has two.
 */
[[nodiscard]] std::vector<size_t> faces_facing(const ShapeGeometry& geometry,
                                               const glm::dvec3& direction) {
    std::vector<size_t> out;
    for (size_t i = 0; i < geometry.faces.size(); ++i) {
        const glm::dvec3 n = face_normal(geometry, geometry.faces[i]);
        if (glm::dot(n, direction) > 0.9998) {
            out.push_back(i);
        }
    }
    return out;
}

/// The lowest y any vertex of a face has
[[nodiscard]] double face_min_y(const ShapeGeometry& geometry, const Face& face) {
    double y = 1.0e300;
    for (const uint32_t index : face.loop) {
        y = std::min(y, geometry.positions[index].y);
    }
    return y;
}

/**
 * @brief Restate shape.hpp's tight-box invariant as an assertion
 *
 * A mass operation changes the geometry AND the scope, and a stale box does not
 * fail where it is written -- it fails in a later `split` that cuts empty air.
 * Every operation here is checked against this rather than against a size it was
 * told to expect, because a size that agrees with the geometry is the only
 * property worth having.
 */
void check_tight_box(const Shape& shape) {
    glm::dvec3 min_corner{0.0};
    glm::dvec3 max_corner{0.0};
    if (!geometry_bounds(shape.geometry, min_corner, max_corner)) {
        CHECK_TRUE(false);
        return;
    }
    CHECK_NEAR(min_corner.x, 0.0, 1e-12);
    CHECK_NEAR(min_corner.y, 0.0, 1e-12);
    CHECK_NEAR(min_corner.z, 0.0, 1e-12);
    CHECK_NEAR(max_corner.x, shape.scope.size.x, 1e-12);
    CHECK_NEAR(max_corner.y, shape.scope.size.y, 1e-12);
    CHECK_NEAR(max_corner.z, shape.scope.size.z, 1e-12);
}

/// standard + D6 control + E2 mass, which is what a rule file here may call
[[nodiscard]] const OperationTable& test_operations() {
    static const OperationTable table = [] {
        OperationTable built = control_operations();
        register_mass_operations(built);
        return built;
    }();
    return table;
}

/// Parse @p source and run it, with the mass operations registered
[[nodiscard]] GenerationResult run_on(const std::string& source,
                                      const Shape& seed,
                                      const GenerationOptions& options = {}) {
    const ParseResult parsed = parse(source, "test.srl");
    CHECK_EQ(parsed.render_all(source), std::string{});
    return generate(parsed.file, seed, options, &test_operations(), &control_functions());
}

/// Every terminal the named rule produced, in evaluation order
[[nodiscard]] std::vector<const Shape*> terminals_named(const GenerationResult& result,
                                                        const std::string& rule) {
    std::vector<const Shape*> out;
    for (const Shape& shape : result.terminals) {
        if (shape.rule == rule) {
            out.push_back(&shape);
        }
    }
    return out;
}

/// Diagnostics of @p severity whose message contains @p needle
[[nodiscard]] size_t count_messages(const GenerationResult& result,
                                    Severity severity,
                                    const std::string& needle) {
    size_t found = 0;
    for (const Diagnostic& diagnostic : result.diagnostics) {
        if (diagnostic.severity == severity &&
            diagnostic.message.find(needle) != std::string::npos) {
            ++found;
        }
    }
    return found;
}

/// World-space bounds of a shape's vertices, through to_world() rather than the scope
void world_bounds(const Shape& shape, glm::dvec3& min_out, glm::dvec3& max_out) {
    min_out = glm::dvec3{1.0e300};
    max_out = glm::dvec3{-1.0e300};
    for (const glm::dvec3& local : shape.geometry.positions) {
        const glm::dvec3 world = shape.scope.to_world(local);
        min_out = glm::min(min_out, world);
        max_out = glm::max(max_out, world);
    }
}

} // namespace

// ============================================================================
// Ring arithmetic
//
// The sign convention is the one thing here that cannot be reasoned about from
// first principles without getting it backwards: "(x, z) as plotted is a
// left-handed view of the xz plane", as shape.cpp puts it. So it is tied to
// shape.cpp's own answer rather than to an expectation written out here.
// ============================================================================

TEST(OpMass, a_positive_newell_term_is_the_winding_that_faces_up) {
    const std::vector<glm::dvec2> ring = rect_ring(0.0, 0.0, 3.0, 2.0);

    // shape_from_polygon() FIXES the winding so the face normal is +y. Whatever
    // winding it chose, ring_newell_y() of the ring it produced must be positive
    // -- that is the claim, and it is read back from shape.cpp rather than
    // assumed here.
    const Shape shape = shape_from_polygon(ring);
    CHECK_EQ(shape.geometry.faces.size(), size_t{1});
    if (shape.geometry.faces.empty()) {
        return;
    }
    const glm::dvec3 normal = face_normal(shape.geometry, shape.geometry.faces[0]);
    CHECK_NEAR(normal.y, 1.0, 1e-12);

    std::vector<glm::dvec2> as_built;
    for (const uint32_t index : shape.geometry.faces[0].loop) {
        const glm::dvec3& p = shape.geometry.positions[index];
        as_built.push_back(glm::dvec2{p.x, p.z});
    }
    CHECK_TRUE(ring_newell_y(as_built) > 0.0);

    // And the magnitude is twice the area, which is what ring_area() divides.
    CHECK_NEAR(ring_area(as_built), 6.0, 1e-12);
    CHECK_NEAR(ring_area(ring), 6.0, 1e-12);

    // Reversing flips the sign and leaves the area alone.
    std::vector<glm::dvec2> reversed = as_built;
    std::reverse(reversed.begin(), reversed.end());
    CHECK_TRUE(ring_newell_y(reversed) < 0.0);
    CHECK_NEAR(ring_area(reversed), 6.0, 1e-12);

    // Fewer than three points encloses nothing rather than reading past the end.
    const std::vector<glm::dvec2> two = {{0.0, 0.0}, {1.0, 0.0}};
    CHECK_NEAR(ring_newell_y(two), 0.0, 1e-12);
    CHECK_NEAR(ring_area(two), 0.0, 1e-12);
}

TEST(OpMass, ring_containment_refuses_a_touching_swapped_or_escaping_ring) {
    const std::vector<glm::dvec2> outer = rect_ring(0.0, 0.0, 10.0, 10.0);
    const std::vector<glm::dvec2> inner = rect_ring(2.0, 3.0, 8.0, 7.0);

    CHECK_TRUE(ring_strictly_contains(outer, inner));

    // Swapped. Test 1 alone passes this whenever the two do not cross, which is
    // exactly the case of one ring nested in the other.
    CHECK_FALSE(ring_strictly_contains(inner, outer));

    // Identical rings: every vertex lies ON the boundary, and a terrace built
    // against that has a zero-width neck.
    CHECK_FALSE(ring_strictly_contains(outer, outer));

    // Touching along one edge, otherwise strictly inside.
    CHECK_FALSE(ring_strictly_contains(outer, rect_ring(0.0, 3.0, 8.0, 7.0)));

    // One corner outside.
    CHECK_FALSE(ring_strictly_contains(outer, rect_ring(2.0, 3.0, 12.0, 7.0)));

    // A ring lying in a notch: inside the bounding box, outside the polygon.
    // This is a TEST 1 case and is named as one. It used to stand in this suite
    // as the edge-crossing case, and it is not: all four of its vertices are in
    // the notch, so test 1 refuses it before test 2 is ever consulted. The real
    // crossing case is the test below, and deleting the crossing loop from
    // ring_strictly_contains() used to pass this whole suite.
    const std::vector<glm::dvec2> notched = {{0.0, 0.0},  {10.0, 0.0}, {10.0, 4.0},
                                             {4.0, 4.0},  {4.0, 6.0},  {10.0, 6.0},
                                             {10.0, 10.0}, {0.0, 10.0}};
    const std::vector<glm::dvec2> bar = rect_ring(5.0, 4.5, 9.0, 5.5);
    CHECK_FALSE(ring_strictly_contains(notched, bar));

    // A ring with fewer than three points contains nothing and is contained by
    // nothing, rather than indexing past the end.
    const std::vector<glm::dvec2> two = {{1.0, 1.0}, {2.0, 2.0}};
    CHECK_FALSE(ring_strictly_contains(outer, two));
    CHECK_FALSE(ring_strictly_contains(two, inner));
}

TEST(OpMass, ring_containment_refuses_a_ring_whose_edges_leave_and_re_enter) {
    // THE edge-crossing case, and the only one in this suite that reaches
    // op_mass.hpp's test 2. A U opening towards +z, and a bar laid across its
    // two prongs. Every vertex of the bar is strictly inside the U -- each sits
    // in one prong -- so test 1 accepts all four of them; the bar's two long
    // edges span the gap between the prongs, which is outside the U.
    //
    // This is the single-face terrace invariant's last line of defence for a
    // NON-CONVEX footprint: it is the shape a miter-limited inset leaves at a
    // reflex corner, and it is what stands between a podium and a roof laid
    // over half a light well.
    const std::vector<glm::dvec2> u = {{0.0, 0.0},  {10.0, 0.0}, {10.0, 10.0}, {7.0, 10.0},
                                       {7.0, 3.0},  {3.0, 3.0},  {3.0, 10.0},  {0.0, 10.0}};
    const std::vector<glm::dvec2> bar = rect_ring(1.0, 8.0, 9.0, 9.0);

    // Stated rather than assumed: if test 1 refused any of these, the assertion
    // below would pass for the wrong reason and the crossing loop could be
    // deleted without this test noticing -- which is exactly what went wrong
    // with the notched case above.
    for (const glm::dvec2& corner : bar) {
        const std::vector<glm::dvec2> speck = {corner,
                                               corner + glm::dvec2{0.01, 0.0},
                                               corner + glm::dvec2{0.0, 0.01}};
        CHECK_TRUE(ring_strictly_contains(u, speck));
    }

    CHECK_FALSE(ring_strictly_contains(u, bar));
}

TEST(OpMass, a_vertex_that_touches_an_edge_is_refused_at_every_edge_length) {
    // point_on_segment() divides the cross product by the segment length, so its
    // tolerance is a DISTANCE. An implementation that compares the raw cross
    // product against the same constant is comparing distance * length, which
    // makes a short edge far more forgiving than a long one. Both directions are
    // pinned here, because a footprint has edges of both sizes -- a road frontage
    // in the hundreds of metres and a chamfer in the millimetres.

    // Long edge, 200 000 units. The triangle's base sits a tenth of a nanometre
    // above it, which is inside the one-nanometre tolerance, so it TOUCHES and
    // is not strictly inside. Against the raw comparison the effective tolerance
    // here is 5e-15 units and the triangle would be accepted.
    const std::vector<glm::dvec2> long_ring = {
        {0.0, 0.0}, {200000.0, 0.0}, {200000.0, 100.0}, {0.0, 100.0}};
    const std::vector<glm::dvec2> touching = {
        {100.0, 1.0e-10}, {200.0, 1.0e-10}, {150.0, 50.0}};
    CHECK_FALSE(ring_strictly_contains(long_ring, touching));

    // Short edge, one millimetre. The base is a tenth of a micrometre above it,
    // a hundred times the tolerance, so it is clear of the boundary and IS
    // strictly inside. Against the raw comparison the effective tolerance here
    // is a micrometre and the triangle would be called touching.
    const std::vector<glm::dvec2> short_ring =
        rect_ring(0.0, 0.0, 0.001, 0.001);
    const std::vector<glm::dvec2> clear = {
        {0.0002, 1.0e-7}, {0.0008, 1.0e-7}, {0.0005, 0.0008}};
    CHECK_TRUE(ring_strictly_contains(short_ring, clear));
}

// ============================================================================
// inset_ring
// ============================================================================

TEST(OpMass, an_inset_ring_comes_back_in_the_frame_it_was_given_in) {
    // The ring is deliberately NOT at the origin. inset_ring() sends it through
    // a scratch Shape, and offset_shape() refits that scratch's scope so the box
    // minimum lands at local zero. An implementation that reads the positions
    // back directly rather than through Scope::to_world() is exactly right for a
    // ring at the origin and off by (100, 200) for this one.
    const std::vector<glm::dvec2> ring = rect_ring(100.0, 200.0, 120.0, 212.0);

    std::vector<std::vector<glm::dvec2>> out;
    const OpResult result = inset_ring(ring, 2.0, out);
    CHECK_TRUE(result.ok);
    CHECK_EQ(out.size(), size_t{1});
    if (out.size() != 1) {
        return;
    }

    glm::dvec2 min_corner{0.0};
    glm::dvec2 max_corner{0.0};
    ring_bounds(out[0], min_corner, max_corner);
    CHECK_NEAR(min_corner.x, 102.0, 1e-9);
    CHECK_NEAR(min_corner.y, 202.0, 1e-9);
    CHECK_NEAR(max_corner.x, 118.0, 1e-9);
    CHECK_NEAR(max_corner.y, 210.0, 1e-9);
    CHECK_NEAR(ring_area(out[0]), 16.0 * 8.0, 1e-9);
}

TEST(OpMass, an_inset_that_empties_or_splits_the_outline_says_which) {
    std::vector<std::vector<glm::dvec2>> out;

    // Wider than half the narrow dimension: nothing survives.
    const OpResult gone = inset_ring(rect_ring(0.0, 0.0, 20.0, 12.0), 7.0, out);
    CHECK_FALSE(gone.ok);
    CHECK_EQ(out.size(), size_t{0});

    // A dumbbell: two ten-metre squares joined by a two-metre neck. An inset of
    // 1.5 eats the neck and leaves two islands.
    const std::vector<glm::dvec2> dumbbell = {{0, 0},   {10, 0},  {10, 4},  {16, 4},
                                              {16, 0},  {26, 0},  {26, 10}, {16, 10},
                                              {16, 6},  {10, 6},  {10, 10}, {0, 10}};
    const OpResult split = inset_ring(dumbbell, 1.5, out);
    CHECK_TRUE(split.ok);
    CHECK_EQ(out.size(), size_t{2});

    // A distance of zero is refused rather than returning the ring unchanged: a
    // caller that meant "do not inset" did not need to call.
    const OpResult zero = inset_ring(rect_ring(0.0, 0.0, 20.0, 12.0), 0.0, out);
    CHECK_FALSE(zero.ok);
}

// ============================================================================
// The footprint precondition
// ============================================================================

TEST(OpMass, a_solid_is_not_a_footprint_and_neither_is_an_empty_scope) {
    Shape flat = shape_from_rect(20.0, 12.0);
    CHECK_TRUE(shape_is_footprint(flat));

    CHECK_TRUE(extrude_shape_ok(flat));
    CHECK_FALSE(shape_is_footprint(flat));

    // A shape with no geometry keeps whatever scope was set, so its size.y is
    // not a measurement and must not be read as flatness.
    Shape empty;
    empty.scope.size = glm::dvec3{5.0, 0.0, 5.0};
    CHECK_FALSE(shape_is_footprint(empty));

    std::vector<FootprintFace> faces;
    const OpResult read = read_footprint(empty, faces);
    CHECK_FALSE(read.ok);
    CHECK_EQ(read.message, std::string{"the shape has no geometry to build a mass on"});
}

TEST(OpMass, the_footprint_tolerance_is_a_micrometre_and_not_a_metre) {
    // Every other "this is a solid" case in this suite extrudes by five metres,
    // which a tolerance of a metre would still refuse. So none of them pins the
    // number, and widening kMassFlatTolerance from 1e-6 to 1.0 -- which would
    // accept a half-metre slab as a footprint and quietly build a second storey
    // inside the first -- used to pass the whole suite.
    //
    // The tolerance exists so a footprint draped over terrain and flattened is
    // still a footprint. That is a micrometre of noise, not a step.

    Shape slab = shape_from_rect(20.0, 12.0);
    CHECK_TRUE(extrude_shape(slab, ScopeAxis::Y, 0.5).ok);
    CHECK_NEAR(slab.scope.size.y, 0.5, 1e-12);
    CHECK_FALSE(shape_is_footprint(slab));

    // A hair under the tolerance is flat.
    Shape under = shape_from_rect(20.0, 12.0);
    under.geometry.positions[0].y += 9.0e-7;
    refit_scope(under);
    CHECK_NEAR(under.scope.size.y, 9.0e-7, 1e-15);
    CHECK_TRUE(shape_is_footprint(under));

    // A hair over it is not, and read_footprint() agrees with shape_is_footprint()
    // rather than carrying its own second opinion.
    Shape over = shape_from_rect(20.0, 12.0);
    over.geometry.positions[0].y += 2.0e-6;
    refit_scope(over);
    CHECK_FALSE(shape_is_footprint(over));
    std::vector<FootprintFace> faces;
    CHECK_FALSE(read_footprint(over, faces).ok);
}

TEST(OpMass, reading_a_footprint_normalises_the_winding_and_drops_dead_faces) {
    // The hole is handed in the SAME way round as the outer ring, which
    // shape_from_rings() fixes; read_footprint() must hand both back in the
    // convention build_mass_stack() expects whatever arrived.
    const std::vector<std::vector<glm::dvec2>> holes = {rect_ring(4.0, 4.0, 8.0, 8.0)};
    const Shape shape = shape_from_rings(rect_ring(0.0, 0.0, 20.0, 12.0), holes);

    std::vector<FootprintFace> faces;
    const OpResult read = read_footprint(shape, faces);
    CHECK_TRUE(read.ok);
    CHECK_EQ(faces.size(), size_t{1});
    if (faces.size() != 1) {
        return;
    }
    CHECK_TRUE(ring_newell_y(faces[0].outer) > 0.0);
    CHECK_EQ(faces[0].holes.size(), size_t{1});
    if (faces[0].holes.size() == 1) {
        CHECK_TRUE(ring_newell_y(faces[0].holes[0]) < 0.0);
        CHECK_NEAR(ring_area(faces[0].holes[0]), 16.0, 1e-9);
    }
    CHECK_NEAR(faces[0].plane_y, 0.0, 1e-12);
    CHECK_NEAR(ring_area(faces[0].outer), 240.0, 1e-9);
}

// ============================================================================
// build_mass_stack
// ============================================================================

TEST(OpMass, a_two_tier_stack_is_closed_and_its_terrace_is_an_annulus) {
    // Tier 0: 20 by 12 from y 0 to 4.   Volume 240 * 4  = 960.
    // Tier 1: 16 by 8  from y 4 to 20.  Volume 128 * 16 = 2048.
    std::vector<MassTier> tiers(2);
    tiers[0].outline = rect_ring(0.0, 0.0, 20.0, 12.0);
    tiers[0].top = 4.0;
    tiers[1].outline = rect_ring(2.0, 2.0, 18.0, 10.0);
    tiers[1].top = 20.0;

    ShapeGeometry geometry;
    const OpResult built = build_mass_stack(tiers, {}, 0.0, MaterialKey{}, geometry);
    CHECK_TRUE(built.ok);
    CHECK_EQ(built.message, std::string{});

    CHECK_NEAR(geometry_volume(geometry), 960.0 + 2048.0, 1e-9);

    // 4 walls + 4 walls + underside + roof + terrace.
    CHECK_EQ(geometry.faces.size(), size_t{11});

    // Underside 240, tier 0 walls 2*(20+12)*4 = 256, terrace 240 - 128 = 112,
    // tier 1 walls 2*(16+8)*16 = 768, roof 128.
    CHECK_NEAR(geometry_area(geometry), 240.0 + 256.0 + 112.0 + 768.0 + 128.0, 1e-9);

    // TWO up-facing faces, not three. A stack built as two prisms one on top of
    // the other encloses exactly the same volume and has a third one buried
    // under the tower, where `select face { top }` finds it beside the roof.
    const std::vector<size_t> up = faces_facing(geometry, glm::dvec3{0.0, 1.0, 0.0});
    const std::vector<size_t> down = faces_facing(geometry, glm::dvec3{0.0, -1.0, 0.0});
    CHECK_EQ(up.size(), size_t{2});
    CHECK_EQ(down.size(), size_t{1});

    for (const size_t index : up) {
        const Face& face = geometry.faces[index];
        const double y = face_min_y(geometry, face);
        if (std::fabs(y - 4.0) < 1e-9) {
            CHECK_EQ(face.holes.size(), size_t{1});  // the terrace is an annulus
            CHECK_NEAR(face_area(geometry, face), 112.0, 1e-9);
        } else {
            CHECK_NEAR(y, 20.0, 1e-9);
            CHECK_EQ(face.holes.size(), size_t{0});
            CHECK_NEAR(face_area(geometry, face), 128.0, 1e-9);
        }
    }
}

TEST(OpMass, a_courtyard_runs_the_whole_height_as_one_shaft) {
    // The same two tiers, with a 4 by 4 light well. The shaft is inside tier 1's
    // outline, so the terrace stays one face with one hole and the shaft wall is
    // one wall per edge for the whole height rather than one per tier.
    std::vector<MassTier> tiers(2);
    tiers[0].outline = rect_ring(0.0, 0.0, 20.0, 12.0);
    tiers[0].top = 4.0;
    tiers[1].outline = rect_ring(2.0, 2.0, 18.0, 10.0);
    tiers[1].top = 20.0;
    const std::vector<std::vector<glm::dvec2>> holes = {rect_ring(8.0, 4.0, 12.0, 8.0)};

    ShapeGeometry geometry;
    const OpResult built = build_mass_stack(tiers, holes, 0.0, MaterialKey{}, geometry);
    CHECK_TRUE(built.ok);

    // (240 - 16) * 4 + (128 - 16) * 16.
    CHECK_NEAR(geometry_volume(geometry), 896.0 + 1792.0, 1e-9);

    // 11 as before, plus four shaft walls -- not eight, which is what one set
    // per tier would give.
    CHECK_EQ(geometry.faces.size(), size_t{15});

    const std::vector<size_t> up = faces_facing(geometry, glm::dvec3{0.0, 1.0, 0.0});
    CHECK_EQ(up.size(), size_t{2});
    for (const size_t index : up) {
        const Face& face = geometry.faces[index];
        if (std::fabs(face_min_y(geometry, face) - 20.0) < 1e-9) {
            CHECK_EQ(face.holes.size(), size_t{1});           // the roof, with the shaft open
            CHECK_NEAR(face_area(geometry, face), 112.0, 1e-9);
        } else {
            // The terrace has ONE hole, the tier above it. Listing the shaft
            // again would subtract its area twice.
            CHECK_EQ(face.holes.size(), size_t{1});
            CHECK_NEAR(face_area(geometry, face), 112.0, 1e-9);
        }
    }
}

TEST(OpMass, a_stack_refuses_tiers_that_do_not_rise_and_a_courtyard_that_escapes) {
    std::vector<MassTier> tiers(2);
    tiers[0].outline = rect_ring(0.0, 0.0, 20.0, 12.0);
    tiers[0].top = 4.0;
    tiers[1].outline = rect_ring(2.0, 2.0, 18.0, 10.0);
    tiers[1].top = 4.0;  // no height

    ShapeGeometry geometry;
    const OpResult flat = build_mass_stack(tiers, {}, 0.0, MaterialKey{}, geometry);
    CHECK_FALSE(flat.ok);
    CHECK_TRUE(flat.message.find("tier 1") != std::string::npos);
    CHECK_TRUE(flat.message.find("is not above") != std::string::npos);
    CHECK_EQ(geometry.faces.size(), size_t{0});

    // A courtyard the upper tier no longer contains. The terrace between them is
    // not one face any more, so the stack refuses rather than roofing the well.
    tiers[1].top = 20.0;
    const std::vector<std::vector<glm::dvec2>> holes = {rect_ring(1.0, 1.0, 19.0, 11.0)};
    const OpResult escaped = build_mass_stack(tiers, holes, 0.0, MaterialKey{}, geometry);
    CHECK_FALSE(escaped.ok);
    CHECK_EQ(escaped.message,
             std::string{"the outline at tier 1 does not contain the courtyard"});

    // And an empty stack is refused rather than producing a shape with no faces.
    const OpResult nothing = build_mass_stack({}, {}, 0.0, MaterialKey{}, geometry);
    CHECK_FALSE(nothing.ok);
}

// ============================================================================
// floors
// ============================================================================

TEST(OpMass, a_floor_plan_rounds_the_count_and_refuses_zero) {
    FloorPlan plan;

    CHECK_TRUE(solve_floor_plan(6.0, 3.0, 4.0, plan).ok);
    CHECK_EQ(plan.count, uint32_t{6});
    CHECK_EQ(plan.upper_height, 3.0);
    CHECK_EQ(plan.ground_height, 4.0);

    // 4 + 5 * 3. Exact, and asserted as an equality rather than a tolerance,
    // because op_split.hpp's copy count is a floor() and a total a hair short
    // gives five storeys and a fat one.
    CHECK_EQ(plan.total_height(), 19.0);

    // One storey is the ground storey alone.
    CHECK_TRUE(solve_floor_plan(1.0, 3.0, 4.0, plan).ok);
    CHECK_EQ(plan.total_height(), 4.0);

    // Rounding is to nearest, away from zero on a tie.
    CHECK_TRUE(solve_floor_plan(2.4, 3.0, 3.0, plan).ok);
    CHECK_EQ(plan.count, uint32_t{2});
    CHECK_TRUE(solve_floor_plan(2.5, 3.0, 3.0, plan).ok);
    CHECK_EQ(plan.count, uint32_t{3});

    const OpResult zero = solve_floor_plan(0.0, 3.0, 3.0, plan);
    CHECK_FALSE(zero.ok);
    CHECK_EQ(zero.message, std::string{"the floor count must be at least 1, not 0"});

    const OpResult negative = solve_floor_plan(-2.0, 3.0, 3.0, plan);
    CHECK_FALSE(negative.ok);

    const OpResult absurd =
        solve_floor_plan(static_cast<double>(kMaxFloors) + 1.0, 3.0, 3.0, plan);
    CHECK_FALSE(absurd.ok);
    CHECK_TRUE(absurd.message.find("above the limit") != std::string::npos);

    const OpResult flat_storey = solve_floor_plan(6.0, 0.0, 3.0, plan);
    CHECK_FALSE(flat_storey.ok);
    CHECK_EQ(flat_storey.message, std::string{"the floor height must be positive, not 0"});

    const OpResult flat_ground = solve_floor_plan(6.0, 3.0, -1.0, plan);
    CHECK_FALSE(flat_ground.ok);
}

TEST(OpMass, floors_stacks_to_exactly_the_planned_height_and_records_the_structure) {
    Shape shape = shape_from_rect(20.0, 12.0);
    FloorPlan plan;
    CHECK_TRUE(solve_floor_plan(6.0, 3.0, 4.0, plan).ok);

    const OpResult stacked = floors_shape(shape, plan);
    CHECK_TRUE(stacked.ok);

    // Exactly, not nearly. The facade split in the rule-level test below divides
    // this number and a bit of slack turns six storeys into five.
    CHECK_EQ(shape.scope.size.y, 19.0);
    CHECK_NEAR(geometry_volume(shape.geometry), 240.0 * 19.0, 1e-9);
    check_tight_box(shape);

    // The structure, which is the whole difference from extrude(19).
    CHECK_EQ(shape.attributes.count("floor_count"), size_t{1});
    CHECK_EQ(shape.attributes.count("floor_height"), size_t{1});
    CHECK_EQ(shape.attributes.count("ground_height"), size_t{1});
    CHECK_EQ(attr_number(shape, "floor_count"), 6.0);
    CHECK_EQ(attr_number(shape, "floor_height"), 3.0);
    CHECK_EQ(attr_number(shape, "ground_height"), 4.0);

    // A second call is refused: the shape is no longer a footprint.
    const OpResult again = floors_shape(shape, plan);
    CHECK_FALSE(again.ok);
    CHECK_TRUE(again.message.find("already has a height of 19") != std::string::npos);
}

TEST(OpMass, floors_carries_a_courtyard_up_as_a_shaft) {
    Shape shape = shape_from_rect(20.0, 20.0);
    CourtyardOptions options;
    options.depth = 6.0;
    CHECK_TRUE(courtyard_shape(shape, options, nullptr).ok);

    FloorPlan plan;
    CHECK_TRUE(solve_floor_plan(6.0, 3.0, 3.0, plan).ok);
    CHECK_TRUE(floors_shape(shape, plan).ok);

    // (400 - 64) * 18. A courtyard that was filled in on the way up gives 7200.
    CHECK_NEAR(geometry_volume(shape.geometry), 336.0 * 18.0, 1e-9);
    CHECK_EQ(shape.scope.size.y, 18.0);
    check_tight_box(shape);
}

// ============================================================================
// courtyard
// ============================================================================

TEST(OpMass, courtyard_carves_a_ring_of_the_depth_it_was_given) {
    Shape shape = shape_from_rect(20.0, 20.0);
    CourtyardOptions options;
    options.depth = 6.0;

    CourtyardReport report;
    const OpResult carved = courtyard_shape(shape, options, &report);
    CHECK_TRUE(carved.ok);
    CHECK_EQ(report.carved, uint32_t{1});
    CHECK_EQ(report.kept, uint32_t{0});
    CHECK_EQ(report.no_room, uint32_t{0});
    CHECK_NEAR(report.area, 64.0, 1e-9);

    CHECK_EQ(shape.geometry.faces.size(), size_t{1});
    if (shape.geometry.faces.empty()) {
        return;
    }
    CHECK_EQ(shape.geometry.faces[0].holes.size(), size_t{1});
    CHECK_NEAR(geometry_area(shape.geometry), 400.0 - 64.0, 1e-9);

    // The outline did not move, so neither did the box. A courtyard operation
    // that rebuilt the face and forgot the refit shows up here.
    CHECK_NEAR(shape.scope.size.x, 20.0, 1e-12);
    CHECK_NEAR(shape.scope.size.y, 0.0, 1e-12);
    CHECK_NEAR(shape.scope.size.z, 20.0, 1e-12);
    check_tight_box(shape);

    // The hole is the outline inset by the depth, which is [6, 14] on both axes.
    const Face& face = shape.geometry.faces[0];
    if (face.holes.size() == 1) {
        glm::dvec2 min_corner{0.0};
        glm::dvec2 max_corner{0.0};
        std::vector<glm::dvec2> ring;
        for (const uint32_t index : face.holes[0]) {
            ring.push_back(glm::dvec2{shape.geometry.positions[index].x,
                                      shape.geometry.positions[index].z});
        }
        ring_bounds(ring, min_corner, max_corner);
        CHECK_NEAR(min_corner.x, 6.0, 1e-9);
        CHECK_NEAR(min_corner.y, 6.0, 1e-9);
        CHECK_NEAR(max_corner.x, 14.0, 1e-9);
        CHECK_NEAR(max_corner.y, 14.0, 1e-9);
        // Wound against the outer loop, which is Face's contract for a hole.
        CHECK_TRUE(ring_newell_y(ring) < 0.0);
    }
}

TEST(OpMass, a_footprint_with_no_room_for_a_courtyard_is_left_solid_and_succeeds) {
    Shape shape = shape_from_rect(8.0, 6.0);
    CourtyardOptions options;
    options.depth = 4.0;  // deeper than half of 6

    CourtyardReport report;
    const OpResult result = courtyard_shape(shape, options, &report);

    // Permissive on purpose: a rule that says courtyard(4) over a whole city
    // must not fail on every small lot. See op_mass.hpp.
    CHECK_TRUE(result.ok);
    CHECK_EQ(report.no_room, uint32_t{1});
    CHECK_EQ(report.carved, uint32_t{0});
    CHECK_EQ(shape.geometry.faces.size(), size_t{1});
    if (!shape.geometry.faces.empty()) {
        CHECK_EQ(shape.geometry.faces[0].holes.size(), size_t{0});
    }
    CHECK_NEAR(geometry_area(shape.geometry), 48.0, 1e-9);
}

TEST(OpMass, a_courtyard_under_the_minimum_area_is_not_carved) {
    Shape shape = shape_from_rect(20.0, 20.0);
    CourtyardOptions options;
    options.depth = 6.0;      // would give 64 square metres
    options.min_area = 100.0; // which is under the minimum asked for

    CourtyardReport report;
    CHECK_TRUE(courtyard_shape(shape, options, &report).ok);
    CHECK_EQ(report.too_small, uint32_t{1});
    CHECK_EQ(report.carved, uint32_t{0});
    CHECK_NEAR(geometry_area(shape.geometry), 400.0, 1e-9);

    // And just under the courtyard's own area it IS carved, so the comparison is
    // against the real number rather than against nothing.
    Shape other = shape_from_rect(20.0, 20.0);
    options.min_area = 63.0;
    CourtyardReport second;
    CHECK_TRUE(courtyard_shape(other, options, &second).ok);
    CHECK_EQ(second.carved, uint32_t{1});
    CHECK_NEAR(geometry_area(other.geometry), 336.0, 1e-9);

    // AT the minimum it is carved: the comparison is `area < min_area`, and a
    // courtyard of exactly the size asked for is a courtyard of the size asked
    // for. 100 and 63 are both clear of 64, so neither of them says which way
    // the boundary falls, and relaxing the test to `<=` used to pass the suite.
    //
    // This case is only worth anything while the carved area is EXACTLY 64, so
    // that is asserted first rather than assumed. Clipper2 works on a 1e-5
    // integer grid and a 20 by 20 lot inset by 6 lands on it exactly; if that
    // ever stops being true this fails loudly here, at the reason, instead of
    // turning into a boundary test of a number nobody knows.
    Shape edge = shape_from_rect(20.0, 20.0);
    options.min_area = 64.0;
    CourtyardReport third;
    CHECK_TRUE(courtyard_shape(edge, options, &third).ok);
    CHECK_EQ(third.area, 64.0);
    CHECK_EQ(third.carved, uint32_t{1});
    CHECK_EQ(third.too_small, uint32_t{0});
    CHECK_NEAR(geometry_area(edge.geometry), 336.0, 1e-9);
}

TEST(OpMass, an_existing_hole_is_kept_open_rather_than_carved_into) {
    const std::vector<std::vector<glm::dvec2>> holes = {rect_ring(8.0, 8.0, 12.0, 12.0)};

    Shape kept = shape_from_rings(rect_ring(0.0, 0.0, 20.0, 20.0), holes);
    CourtyardOptions options;
    options.depth = 6.0;
    options.keep_existing = true;

    CourtyardReport report;
    CHECK_TRUE(courtyard_shape(kept, options, &report).ok);
    CHECK_EQ(report.kept, uint32_t{1});
    CHECK_EQ(report.carved, uint32_t{0});
    // The 4 by 4 hole the footprint arrived with, not the 8 by 8 a fresh carve
    // at this depth would produce.
    CHECK_NEAR(geometry_area(kept.geometry), 400.0 - 16.0, 1e-9);

    // With keep_existing off, the old ring is discarded and a new one cut.
    Shape recut = shape_from_rings(rect_ring(0.0, 0.0, 20.0, 20.0), holes);
    options.keep_existing = false;
    CourtyardReport second;
    CHECK_TRUE(courtyard_shape(recut, options, &second).ok);
    CHECK_EQ(second.carved, uint32_t{1});
    CHECK_EQ(second.kept, uint32_t{0});
    CHECK_NEAR(geometry_area(recut.geometry), 400.0 - 64.0, 1e-9);
    if (!recut.geometry.faces.empty()) {
        CHECK_EQ(recut.geometry.faces[0].holes.size(), size_t{1});
    }
}

TEST(OpMass, courtyard_refuses_a_depth_that_is_not_positive_and_a_solid) {
    Shape shape = shape_from_rect(20.0, 20.0);
    CourtyardOptions options;
    options.depth = 0.0;

    const OpResult zero = courtyard_shape(shape, options, nullptr);
    CHECK_FALSE(zero.ok);
    CHECK_EQ(zero.message, std::string{"the courtyard depth must be positive, not 0"});

    options.depth = 6.0;
    CHECK_TRUE(extrude_shape_ok(shape));
    const OpResult solid = courtyard_shape(shape, options, nullptr);
    CHECK_FALSE(solid.ok);
    CHECK_TRUE(solid.message.find("already has a height") != std::string::npos);
}

// ============================================================================
// podium
// ============================================================================

TEST(OpMass, a_podium_plan_refuses_a_zero_inset_because_that_is_an_extrude) {
    PodiumPlan plan;

    CHECK_TRUE(solve_podium_plan(4.0, 16.0, 2.0, 1.0, plan).ok);
    CHECK_EQ(plan.steps, uint32_t{1});
    CHECK_EQ(plan.step_height(), 16.0);

    CHECK_TRUE(solve_podium_plan(4.0, 16.0, 2.0, 4.0, plan).ok);
    CHECK_EQ(plan.step_height(), 4.0);

    const OpResult no_inset = solve_podium_plan(4.0, 16.0, 0.0, 1.0, plan);
    CHECK_FALSE(no_inset.ok);
    CHECK_TRUE(no_inset.message.find("a podium with no inset is an extrude") !=
               std::string::npos);

    CHECK_FALSE(solve_podium_plan(0.0, 16.0, 2.0, 1.0, plan).ok);
    CHECK_FALSE(solve_podium_plan(4.0, 0.0, 2.0, 1.0, plan).ok);
    CHECK_FALSE(solve_podium_plan(4.0, 16.0, 2.0, 0.0, plan).ok);
    CHECK_FALSE(solve_podium_plan(4.0, 16.0, 2.0, 1000.0, plan).ok);
}

TEST(OpMass, the_podium_step_count_rounds_to_nearest_like_the_floor_count) {
    // The language has no integer type, so `podium(4, 12, 2, attrs.get("tiers"))`
    // arrives as a double and something has to round it. The floor count is
    // pinned at 2.4 and 2.5 a few tests up; the step count was only ever handed
    // whole numbers, so truncating instead of rounding used to pass the suite --
    // and truncation turns a 2.9 that came out of an expression into two tiers.
    PodiumPlan plan;

    CHECK_TRUE(solve_podium_plan(4.0, 12.0, 2.0, 2.4, plan).ok);
    CHECK_EQ(plan.steps, uint32_t{2});

    CHECK_TRUE(solve_podium_plan(4.0, 12.0, 2.0, 2.6, plan).ok);
    CHECK_EQ(plan.steps, uint32_t{3});

    // Away from zero on a tie, which is std::round() and is what an author
    // reading "two and a half tiers" expects of it.
    CHECK_TRUE(solve_podium_plan(4.0, 12.0, 2.0, 2.5, plan).ok);
    CHECK_EQ(plan.steps, uint32_t{3});
    CHECK_EQ(plan.step_height(), 4.0);

    // And it rounds before the limits are applied, so 0.4 is refused as zero
    // rather than accepted as "nearly one".
    CHECK_FALSE(solve_podium_plan(4.0, 12.0, 2.0, 0.4, plan).ok);
}

TEST(OpMass, a_podium_is_a_base_with_an_inset_mass_above_it) {
    Shape shape = shape_from_rect(20.0, 12.0);
    PodiumPlan plan;
    CHECK_TRUE(solve_podium_plan(4.0, 16.0, 2.0, 1.0, plan).ok);

    const OpResult built = podium_shape(shape, plan);
    CHECK_TRUE(built.ok);
    CHECK_EQ(built.message, std::string{});

    // Podium 20 by 12 for four metres, tower 16 by 8 for sixteen.
    CHECK_NEAR(geometry_volume(shape.geometry), 240.0 * 4.0 + 128.0 * 16.0, 1e-9);
    CHECK_EQ(shape.scope.size.y, 20.0);

    // The box is the PODIUM's footprint, because the podium is the widest tier.
    CHECK_NEAR(shape.scope.size.x, 20.0, 1e-12);
    CHECK_NEAR(shape.scope.size.z, 12.0, 1e-12);
    check_tight_box(shape);

    // Two front walls, at two heights: the podium's and the tower's. This is
    // what a facade rule sees, and it is the point of the operation.
    const std::vector<size_t> front = faces_facing(shape.geometry, glm::dvec3{0.0, 0.0, 1.0});
    CHECK_EQ(front.size(), size_t{2});
    double front_area = 0.0;
    for (const size_t index : front) {
        front_area += face_area(shape.geometry, shape.geometry.faces[index]);
    }
    CHECK_NEAR(front_area, 20.0 * 4.0 + 16.0 * 16.0, 1e-9);

    // The plan, for a rule that wants to split the mass back up.
    CHECK_EQ(attr_number(shape, "podium_height"), 4.0);
    CHECK_EQ(attr_number(shape, "tower_height"), 16.0);
    CHECK_EQ(attr_number(shape, "podium_inset"), 2.0);
    CHECK_EQ(attr_number(shape, "podium_steps"), 1.0);
}

TEST(OpMass, more_steps_make_a_wedding_cake) {
    Shape shape = shape_from_rect(20.0, 20.0);
    PodiumPlan plan;
    CHECK_TRUE(solve_podium_plan(4.0, 17.0, 2.0, 3.0, plan).ok);

    CHECK_TRUE(podium_shape(shape, plan).ok);
    CHECK_EQ(shape.scope.size.y, 21.0);
    check_tight_box(shape);

    // Tiers: 20x20 to y 4, 16x16 to 4 + 17/3, 12x12 to 4 + 34/3, 8x8 to 21.
    const double third = 17.0 / 3.0;
    const double expected = 400.0 * 4.0 + 256.0 * third + 144.0 * third +
                            64.0 * (21.0 - 4.0 - 2.0 * third);
    CHECK_NEAR(geometry_volume(shape.geometry), expected, 1e-9);

    // Four up-facing faces: three terraces and the roof. A fourth terrace or a
    // buried cap changes this number.
    const std::vector<size_t> up = faces_facing(shape.geometry, glm::dvec3{0.0, 1.0, 0.0});
    CHECK_EQ(up.size(), size_t{4});
    CHECK_EQ(faces_facing(shape.geometry, glm::dvec3{0.0, -1.0, 0.0}).size(), size_t{1});
}

/**
 * @brief The last tier's top is exactly the podium plus the whole tower
 *
 * 15.17 metres over five steps is chosen because it is the arithmetic that goes
 * wrong, not because it is a round number. Adding 4 to each of
 *
 *     15.17 * 5 / 5                          (the general formula, last step included)
 *     (15.17 / 5) * 5                        (a step height multiplied back up)
 *     15.17/5 + 15.17/5 + ... five times     (tier tops accumulated)
 *
 * gives a double 3.6e-15 away from `4.0 + 15.17` in IEEE-754, and a mass whose
 * scope is 19.169999999999998 rather than 19.17 hands a facade `split(y)` an
 * extent a hair short of a whole number of storeys -- which op_split.hpp's
 * floor() turns into one storey fewer and one fat one.
 *
 * So the last tier's top is the plan's own sum and not a computed approach to
 * it, and this asserts that with an equality rather than a tolerance. A
 * tolerance of 1e-9 here would pass every one of the three wrong forms above.
 */
TEST(OpMass, the_last_tier_stops_exactly_at_the_podium_plus_the_tower) {
    Shape shape = shape_from_rect(20.0, 12.0);
    PodiumPlan plan;
    CHECK_TRUE(solve_podium_plan(4.0, 15.17, 1.0, 5.0, plan).ok);

    CHECK_TRUE(podium_shape(shape, plan).ok);
    CHECK_EQ(shape.scope.size.y, 4.0 + 15.17);
    check_tight_box(shape);

    // Five step-backs of one metre each leave a 10 by 2 roof on a 20 by 12 base,
    // so the tiers really did narrow and the height above is not the height of a
    // single prism.
    CHECK_NEAR(shape.scope.size.x, 20.0, 1e-12);
    const std::vector<size_t> up = faces_facing(shape.geometry, glm::dvec3{0.0, 1.0, 0.0});
    CHECK_EQ(up.size(), size_t{6});
}

/**
 * @brief A face with no area is dropped, and the box refits to what is left
 *
 * The reason every operation here ends in refit_scope() rather than reasoning
 * that its own outline did not move. `courtyard` adds an interior ring and
 * changes no boundary, so it looks like an operation whose bounds cannot
 * change -- until the footprint it was handed carries a degenerate face that
 * read_footprint() drops, and the geometry left behind is smaller than the box
 * that was fitted around both.
 */
TEST(OpMass, a_degenerate_face_is_dropped_and_the_box_refits_to_what_is_left) {
    Shape shape = shape_from_rect(20.0, 12.0);

    // Three collinear points a hundred metres away: no area, no normal, nothing
    // a mass can be built on. extrude_shape() drops such a face too.
    const uint32_t base = static_cast<uint32_t>(shape.geometry.positions.size());
    shape.geometry.positions.push_back(glm::dvec3{60.0, 0.0, 0.0});
    shape.geometry.positions.push_back(glm::dvec3{80.0, 0.0, 0.0});
    shape.geometry.positions.push_back(glm::dvec3{100.0, 0.0, 0.0});
    Face sliver;
    sliver.loop = {base, base + 1, base + 2};
    shape.geometry.faces.push_back(sliver);
    refit_scope(shape);
    CHECK_EQ(shape.geometry.faces.size(), size_t{2});
    CHECK_NEAR(shape.scope.size.x, 100.0, 1e-12);

    std::vector<FootprintFace> faces;
    CHECK_TRUE(read_footprint(shape, faces).ok);
    CHECK_EQ(faces.size(), size_t{1});

    CourtyardOptions options;
    options.depth = 4.0;
    CHECK_TRUE(courtyard_shape(shape, options, nullptr).ok);
    CHECK_EQ(shape.geometry.faces.size(), size_t{1});
    CHECK_NEAR(shape.scope.size.x, 20.0, 1e-12);
    CHECK_NEAR(shape.scope.size.z, 12.0, 1e-12);
    check_tight_box(shape);
}

TEST(OpMass, a_setback_larger_than_the_footprint_is_refused_by_name) {
    Shape shape = shape_from_rect(20.0, 12.0);
    PodiumPlan plan;
    CHECK_TRUE(solve_podium_plan(4.0, 16.0, 8.0, 1.0, plan).ok);

    const OpResult result = podium_shape(shape, plan);
    CHECK_FALSE(result.ok);
    CHECK_EQ(result.message,
             std::string{"the inset at tier 1 of 8 removes the whole outline"});

    // The refusal leaves the shape alone rather than half-built, so the
    // interpreter's abandon is the only thing that discards it.
    CHECK_NEAR(shape.scope.size.y, 0.0, 1e-12);
    CHECK_NEAR(geometry_area(shape.geometry), 240.0, 1e-9);
}

TEST(OpMass, a_setback_that_splits_the_outline_is_refused) {
    const std::vector<glm::dvec2> dumbbell = {{0, 0},   {10, 0},  {10, 4},  {16, 4},
                                              {16, 0},  {26, 0},  {26, 10}, {16, 10},
                                              {16, 6},  {10, 6},  {10, 10}, {0, 10}};
    Shape shape = shape_from_polygon(dumbbell);
    PodiumPlan plan;
    CHECK_TRUE(solve_podium_plan(4.0, 16.0, 1.5, 1.0, plan).ok);

    const OpResult result = podium_shape(shape, plan);
    CHECK_FALSE(result.ok);
    CHECK_EQ(result.message,
             std::string{"the inset at tier 1 splits the outline into 2 parts"});
}

TEST(OpMass, a_setback_that_would_close_the_courtyard_is_refused_at_its_tier) {
    Shape shape = shape_from_rect(20.0, 20.0);
    CourtyardOptions options;
    options.depth = 4.0;  // a 12 by 12 light well spanning [4, 16]
    CHECK_TRUE(courtyard_shape(shape, options, nullptr).ok);

    // Tier 1 is 14 by 14 spanning [3, 17] and still contains the well.
    // Tier 2 is 8 by 8 spanning [6, 14] and does not.
    PodiumPlan plan;
    CHECK_TRUE(solve_podium_plan(4.0, 16.0, 3.0, 2.0, plan).ok);
    const OpResult result = podium_shape(shape, plan);
    CHECK_FALSE(result.ok);
    CHECK_EQ(result.message,
             std::string{"the outline at tier 2 does not contain the courtyard"});

    // One step back is fine on the same footprint, so the refusal above is about
    // the courtyard and not about the operation refusing courtyards at all.
    Shape single = shape_from_rect(20.0, 20.0);
    CHECK_TRUE(courtyard_shape(single, options, nullptr).ok);
    PodiumPlan gentle;
    CHECK_TRUE(solve_podium_plan(4.0, 16.0, 3.0, 1.0, gentle).ok);
    CHECK_TRUE(podium_shape(single, gentle).ok);
    // (400 - 144) * 4 + (196 - 144) * 16.
    CHECK_NEAR(geometry_volume(single.geometry), 256.0 * 4.0 + 52.0 * 16.0, 1e-9);
}

// ============================================================================
// Registration
// ============================================================================

TEST(OpMass, the_catalogue_rows_and_the_handlers_agree) {
    using stratum::procgen::rules::find_builtin_operation;
    using stratum::procgen::rules::kBuiltinOperations;
    using stratum::procgen::rules::kNoNode;

    struct Row {
        const char* name;
        uint8_t min_args;
        uint8_t max_args;
    };
    // The arities this file was written against. A catalogue edit that widened
    // or narrowed one of them without a matching change to the handler would
    // otherwise surface as a parse error in somebody else's rule file.
    const Row rows[] = {{"courtyard", 1, 3}, {"floors", 1, 3}, {"podium", 2, 4}};

    for (const Row& row : rows) {
        const uint32_t index = find_builtin_operation(row.name);
        CHECK_TRUE(index != kNoNode);
        if (index == kNoNode) {
            continue;
        }
        CHECK_EQ(kBuiltinOperations[index].min_args, row.min_args);
        CHECK_EQ(kBuiltinOperations[index].max_args, row.max_args);

        // Registered here and deliberately absent from D2's set, which is what
        // makes "copy a table, register into the copy" the whole wiring.
        CHECK_TRUE(standard_operations().find(row.name) == nullptr);
        CHECK_TRUE(mass_operations().find(row.name) != nullptr);
    }

    // Additive: registering into a table does not remove what was there.
    CHECK_TRUE(mass_operations().find("extrude") != nullptr);
    CHECK_EQ(mass_operations().size(), standard_operations().size() + 3);
}

// ============================================================================
// The rule-level proof
// ============================================================================

/**
 * @brief A rule file stacks six storeys and the facade split finds exactly six
 *
 * This is the test that says the feature is finished, and every number in it is
 * computed by hand from the rule text.
 *
 * The seed is 20 by 12 in the xz plane. `floors(6, 3.0, 4.0)` makes the mass
 * `4 + 5 * 3 = 19` metres tall, so the solid is x in [0, 20], y in [0, 19],
 * z in [0, 12].
 *
 * `front` is the +z face, the z = 12 wall, 20 wide and 19 tall. Its component
 * frame has x along world x and y along world y, so the split below is a split
 * in world y.
 *
 * The facade reads the storey heights back out of the attributes `floors` wrote.
 * The FALLBACKS ARE DELIBERATELY WRONG: with `attrs.get("ground_height", 7.0)`
 * and `attrs.get("floor_height", 7.0)`, a run in which `floors` wrote no
 * attributes splits 19 as 7 plus one copy of 12 -- two terminals -- instead of
 * the six asserted here. The test cannot pass on the fallback.
 *
 * With the attributes: the shopfront takes 4 and leaves 15; op_split.hpp's
 * repeat group has a period of 3 and `floor(15 / 3) = 5` copies of exactly 3.
 * So the floors sit at y in [0, 4], [4, 7], [7, 10], [10, 13], [13, 16] and
 * [16, 19], each of them the full 20 metres wide.
 */
TEST(OpMass, a_rule_file_stacks_six_storeys_and_the_facade_finds_exactly_six) {
    const std::string source =
        "@start\n"
        "rule Main {\n"
        "    floors(6, 3.0, 4.0);\n"
        "    select face {\n"
        "        front : { Facade(); }\n"
        "    }\n"
        "}\n"
        "\n"
        "rule Facade {\n"
        "    split(y) {\n"
        "        attrs.get(\"ground_height\", 7.0) : { Shopfront(); }\n"
        "        repeat { attrs.get(\"floor_height\", 7.0) : { UpperFloor(); } }\n"
        "    }\n"
        "}\n"
        "\n"
        "rule Shopfront { }\n"
        "rule UpperFloor { }\n";

    const GenerationResult result = run_on(source, shape_from_rect(20.0, 12.0));

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.diagnostics.size(), size_t{0});
    CHECK_EQ(result.terminals.size(), size_t{6});

    const std::vector<const Shape*> ground = terminals_named(result, "Shopfront");
    const std::vector<const Shape*> upper = terminals_named(result, "UpperFloor");
    CHECK_EQ(ground.size(), size_t{1});
    CHECK_EQ(upper.size(), size_t{5});

    if (ground.size() == 1) {
        glm::dvec3 min_corner{0.0};
        glm::dvec3 max_corner{0.0};
        world_bounds(*ground[0], min_corner, max_corner);
        CHECK_NEAR(min_corner.x, 0.0, 1e-9);
        CHECK_NEAR(min_corner.y, 0.0, 1e-9);
        CHECK_NEAR(max_corner.x, 20.0, 1e-9);
        CHECK_NEAR(max_corner.y, 4.0, 1e-9);
        CHECK_NEAR(min_corner.z, 12.0, 1e-9);
        CHECK_NEAR(max_corner.z, 12.0, 1e-9);
    }

    // Each upper storey sits on the one below it with no gap and no overlap, and
    // every one of them is a full three metres. A repeat that absorbed a
    // remainder into the copies would give five storeys of 3.0 only if the total
    // really was an exact multiple, which is the claim.
    std::vector<double> bottoms;
    for (const Shape* floor : upper) {
        glm::dvec3 min_corner{0.0};
        glm::dvec3 max_corner{0.0};
        world_bounds(*floor, min_corner, max_corner);
        CHECK_NEAR(max_corner.y - min_corner.y, 3.0, 1e-9);
        CHECK_NEAR(min_corner.x, 0.0, 1e-9);
        CHECK_NEAR(max_corner.x, 20.0, 1e-9);
        bottoms.push_back(min_corner.y);
    }
    std::sort(bottoms.begin(), bottoms.end());
    const std::vector<double> expected = {4.0, 7.0, 10.0, 13.0, 16.0};
    CHECK_EQ(bottoms.size(), expected.size());
    for (size_t i = 0; i < bottoms.size() && i < expected.size(); ++i) {
        CHECK_NEAR(bottoms[i], expected[i], 1e-9);
    }

    // And it reproduces, which is what a golden test of it would rest on.
    const GenerationResult again = run_on(source, shape_from_rect(20.0, 12.0));
    CHECK_TRUE(result.dump().size() > 500);
    CHECK_EQ(result.dump(), again.dump());
}

TEST(OpMass, a_courtyard_survives_the_massing_and_reaches_the_facade) {
    // 20 by 20 with a six-metre-deep ring leaves an 8 by 8 light well spanning
    // [6, 14]. Six storeys of three make the mass 18 tall.
    //
    // `select face { front }` then finds TWO walls: the street wall at z = 20,
    // 20 metres wide, and the courtyard's own +z-facing wall at z = 6, 8 metres
    // wide. Both are front walls and both get the facade, which is correct and
    // is asserted because the first instinct on seeing twelve terminals where
    // six were expected is to assume a bug.
    const std::string source =
        "@start\n"
        "rule Main {\n"
        "    courtyard(6.0);\n"
        "    floors(6, 3.0);\n"
        "    select face {\n"
        "        front : { Facade(); }\n"
        "    }\n"
        "}\n"
        "\n"
        "rule Facade {\n"
        "    split(y) { repeat { attrs.get(\"floor_height\", 7.0) : { Storey(); } } }\n"
        "}\n"
        "\n"
        "rule Storey { }\n";

    const GenerationResult result = run_on(source, shape_from_rect(20.0, 20.0));

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.diagnostics.size(), size_t{0});

    const std::vector<const Shape*> storeys = terminals_named(result, "Storey");
    CHECK_EQ(storeys.size(), size_t{12});

    size_t street = 0;
    size_t well = 0;
    for (const Shape* storey : storeys) {
        glm::dvec3 min_corner{0.0};
        glm::dvec3 max_corner{0.0};
        world_bounds(*storey, min_corner, max_corner);
        CHECK_NEAR(max_corner.y - min_corner.y, 3.0, 1e-9);
        if (std::fabs(min_corner.z - 20.0) < 1e-9) {
            ++street;
            CHECK_NEAR(max_corner.x - min_corner.x, 20.0, 1e-9);
        } else if (std::fabs(min_corner.z - 6.0) < 1e-9) {
            ++well;
            CHECK_NEAR(max_corner.x - min_corner.x, 8.0, 1e-9);
            CHECK_NEAR(min_corner.x, 6.0, 1e-9);
        }
    }
    CHECK_EQ(street, size_t{6});
    CHECK_EQ(well, size_t{6});
}

TEST(OpMass, a_podium_rule_leaves_a_terrace_and_a_roof_for_select_to_find) {
    const std::string source =
        "@start\n"
        "rule Main {\n"
        "    podium(4.0, 16.0, 2.0);\n"
        "    select face {\n"
        "        top : { Deck(); }\n"
        "    }\n"
        "}\n"
        "\n"
        "rule Deck { }\n";

    const GenerationResult result = run_on(source, shape_from_rect(20.0, 12.0));

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.diagnostics.size(), size_t{0});

    const std::vector<const Shape*> decks = terminals_named(result, "Deck");
    // Two, not three: the cap that would be buried under the tower if a podium
    // were two prisms stacked is not there to find.
    CHECK_EQ(decks.size(), size_t{2});

    double terrace = 0.0;
    double roof = 0.0;
    for (const Shape* deck : decks) {
        glm::dvec3 min_corner{0.0};
        glm::dvec3 max_corner{0.0};
        world_bounds(*deck, min_corner, max_corner);
        if (std::fabs(min_corner.y - 4.0) < 1e-9) {
            terrace = geometry_area(deck->geometry);
            CHECK_NEAR(min_corner.x, 0.0, 1e-9);
            CHECK_NEAR(max_corner.x, 20.0, 1e-9);
        } else if (std::fabs(min_corner.y - 20.0) < 1e-9) {
            roof = geometry_area(deck->geometry);
            CHECK_NEAR(min_corner.x, 2.0, 1e-9);
            CHECK_NEAR(max_corner.x, 18.0, 1e-9);
        }
    }
    CHECK_NEAR(terrace, 240.0 - 128.0, 1e-9);
    CHECK_NEAR(roof, 128.0, 1e-9);
}

TEST(OpMass, a_rule_that_stacks_zero_floors_fails_at_its_own_line) {
    const std::string source =
        "@start\n"
        "rule Main {\n"
        "    floors(0);\n"
        "}\n";

    const GenerationResult result = run_on(source, shape_from_rect(20.0, 12.0));

    CHECK_FALSE(result.ok());
    CHECK_EQ(count_messages(result, Severity::Error,
                            "'floors': the floor count must be at least 1, not 0"),
             size_t{1});
    CHECK_EQ(result.terminals.size(), size_t{0});
    // At the line the operation was written on, not at the start rule's.
    CHECK_EQ(result.diagnostics.size(), size_t{1});
    if (result.diagnostics.size() == 1) {
        CHECK_EQ(result.diagnostics[0].loc.line, uint32_t{3});
    }
}

TEST(OpMass, one_floors_argument_uses_the_default_storey_height) {
    const std::string source =
        "@start\n"
        "rule Main { floors(4); }\n";

    const GenerationResult result = run_on(source, shape_from_rect(20.0, 12.0));

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.terminals.size(), size_t{1});
    if (result.terminals.size() != 1) {
        return;
    }
    CHECK_EQ(result.terminals[0].scope.size.y, 4.0 * kDefaultFloorHeight);
    CHECK_EQ(attr_number(result.terminals[0], "floor_height"), kDefaultFloorHeight);
    // The ground storey follows the storey height when neither is given, so four
    // equal floors and not three plus an odd one.
    CHECK_EQ(attr_number(result.terminals[0], "ground_height"), kDefaultFloorHeight);
}

// ============================================================================
// The OPTIONAL arguments, exercised through a rule file
//
// Every one of these operations has arguments the catalogue row makes optional,
// and the handler is the only thing that maps argument index to meaning. That
// mapping is not tested by calling floors_shape() or courtyard_shape() with a
// hand-built plan, because a hand-built plan is the handler's output: it skips
// the very line under test.
//
// It is not tested by a rule file that passes the DEFAULT either, and that is
// the trap these tests exist to close. `floors(6, 3.0)`, `podium(4, 16, 2.0)`
// -- three is kDefaultFloorHeight and two is kDefaultPodiumInset, so a handler
// that never read the argument at all produced the identical building and the
// whole suite stayed green. Six such mutants survived thirty-three tests.
//
// So every value below is deliberately NOT the default, the expected volume is
// arithmetic from the rule's own numbers, and each test says which wrong
// handler it is written against.
// ============================================================================

TEST(OpMass, the_second_floors_argument_is_the_storey_height_and_the_ground_follows_it) {
    // 3.5, not kDefaultFloorHeight. A handler reading no second argument stacks
    // six three-metre storeys and comes out at 18 rather than 21.
    const std::string source =
        "@start\n"
        "rule Main { floors(6, 3.5); }\n";

    const GenerationResult result = run_on(source, shape_from_rect(20.0, 12.0));

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.terminals.size(), size_t{1});
    if (result.terminals.size() != 1) {
        return;
    }
    const Shape& built = result.terminals[0];

    // Six EQUAL storeys, which is the header's documented promise for two
    // arguments: the ground storey follows the storey height. A handler that
    // defaulted the ground to kDefaultFloorHeight builds 3 + 5 * 3.5 = 20.5, a
    // number no test in this suite used to be able to tell from 21.
    CHECK_EQ(built.scope.size.y, 21.0);
    CHECK_EQ(attr_number(built, "floor_height"), 3.5);
    CHECK_EQ(attr_number(built, "ground_height"), 3.5);
    CHECK_EQ(attr_number(built, "floor_count"), 6.0);
    CHECK_NEAR(geometry_volume(built.geometry), 20.0 * 12.0 * 21.0, 1e-9);
}

TEST(OpMass, the_second_courtyard_argument_is_the_minimum_area) {
    // A twenty-metre square inset by six leaves a courtyard of 64 square metres,
    // and the rule asks for a hundred. So the lot stays SOLID and warns. This
    // argument reached courtyard_shape() only through a hand-built
    // CourtyardOptions before, so a handler that never read argument 2 left the
    // minimum at zero, carved the courtyard anyway and passed the suite.
    const std::string source =
        "@start\n"
        "rule Main { courtyard(6.0, 100.0); floors(1, 3.0); }\n";

    const GenerationResult result = run_on(source, shape_from_rect(20.0, 20.0));

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.terminals.size(), size_t{1});
    if (result.terminals.size() != 1) {
        return;
    }
    // Solid: 400 square metres of floor, not 400 - 64.
    CHECK_NEAR(geometry_volume(result.terminals[0].geometry), 400.0 * 3.0, 1e-9);
    CHECK_EQ(count_messages(result, Severity::Warning,
                            "the courtyard would be smaller than the minimum area of 100"),
             size_t{1});
    CHECK_EQ(result.diagnostics.size(), size_t{1});
}

TEST(OpMass, the_third_courtyard_argument_discards_an_existing_hole) {
    // The seed arrives with a 4 by 4 hole of its own. `false` says do not keep
    // it: discard it and cut a fresh courtyard at the depth given, which on this
    // lot is 8 by 8. A handler that never read argument 3 keeps the default
    // true, keeps the 4 by 4 hole, and builds 384 square metres of floor instead
    // of 336 -- the difference between the two is the whole of this argument.
    const std::string source =
        "@start\n"
        "rule Main { courtyard(6.0, 0.0, false); floors(1, 3.0); }\n";

    const Shape seed = shape_from_rings(rect_ring(0.0, 0.0, 20.0, 20.0),
                                        {rect_ring(8.0, 8.0, 12.0, 12.0)});
    const GenerationResult result = run_on(source, seed);

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.diagnostics.size(), size_t{0});
    CHECK_EQ(result.terminals.size(), size_t{1});
    if (result.terminals.size() != 1) {
        return;
    }
    CHECK_NEAR(geometry_volume(result.terminals[0].geometry), (400.0 - 64.0) * 3.0, 1e-9);

    // The same seed and the same depth with the argument left off keeps the hole
    // it arrived with, so the two runs differ in nothing but that argument.
    const GenerationResult kept =
        run_on("@start\nrule Main { courtyard(6.0); floors(1, 3.0); }\n", seed);
    CHECK_TRUE(kept.ok());
    CHECK_EQ(kept.terminals.size(), size_t{1});
    if (kept.terminals.size() != 1) {
        return;
    }
    CHECK_NEAR(geometry_volume(kept.terminals[0].geometry), (400.0 - 16.0) * 3.0, 1e-9);
}

TEST(OpMass, the_third_podium_argument_is_the_inset) {
    // 3.0, not kDefaultPodiumInset. On a 20 by 12 lot the tower is 14 by 6 and
    // not the 16 by 8 that two metres would leave, so a handler that never read
    // argument 3 builds 3008 cubic metres where the rule asks for 2304.
    const std::string source =
        "@start\n"
        "rule Main { podium(4.0, 16.0, 3.0); }\n";

    const GenerationResult result = run_on(source, shape_from_rect(20.0, 12.0));

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.terminals.size(), size_t{1});
    if (result.terminals.size() != 1) {
        return;
    }
    const Shape& built = result.terminals[0];
    CHECK_NEAR(geometry_volume(built.geometry), 240.0 * 4.0 + 84.0 * 16.0, 1e-9);
    CHECK_EQ(attr_number(built, "podium_inset"), 3.0);
    CHECK_EQ(attr_number(built, "podium_steps"), 1.0);
    // The box is the PODIUM's extent, as the header promises, and the tower does
    // not shrink it.
    CHECK_NEAR(built.scope.size.x, 20.0, 1e-9);
    CHECK_NEAR(built.scope.size.y, 20.0, 1e-9);
    CHECK_NEAR(built.scope.size.z, 12.0, 1e-9);
}

TEST(OpMass, the_fourth_podium_argument_is_the_tier_count) {
    // A wedding cake of three tiers on a 30 by 30 lot: 24 by 24, 18 by 18 and
    // 12 by 12, each taking a third of the sixteen-metre tower. A handler that
    // never read argument 4 builds ONE tier of 24 by 24 over the whole sixteen
    // metres -- 12816 cubic metres against the 9168 the rule asks for.
    const std::string source =
        "@start\n"
        "rule Main { podium(4.0, 16.0, 3.0, 3.0); }\n";

    const GenerationResult result = run_on(source, shape_from_rect(30.0, 30.0));

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.terminals.size(), size_t{1});
    if (result.terminals.size() != 1) {
        return;
    }
    const Shape& built = result.terminals[0];
    const double tier = 16.0 / 3.0;
    CHECK_NEAR(geometry_volume(built.geometry),
               900.0 * 4.0 + (576.0 + 324.0 + 144.0) * tier, 1e-9);
    CHECK_EQ(attr_number(built, "podium_steps"), 3.0);
    CHECK_EQ(attr_number(built, "podium_inset"), 3.0);
    CHECK_NEAR(built.scope.size.y, 20.0, 1e-9);
}

TEST(OpMass, a_lot_with_no_room_for_a_courtyard_warns_once_and_still_generates) {
    // Three lots reach the same `courtyard` call and none of them has room. The
    // run must still succeed -- a warning, not an error -- and report_once()
    // must keep it to ONE line for the site rather than one per lot.
    const std::string source =
        "@start\n"
        "rule Main {\n"
        "    split(x) { repeat { 2.0 : { Lot(); } } }\n"
        "}\n"
        "\n"
        "rule Lot {\n"
        "    courtyard(4.0);\n"
        "    floors(2, 3.0);\n"
        "}\n";

    const GenerationResult result = run_on(source, shape_from_rect(6.0, 6.0));

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.terminals.size(), size_t{3});
    CHECK_EQ(count_messages(result, Severity::Warning,
                            "'courtyard': the footprint is narrower than twice the depth of 4"),
             size_t{1});
    CHECK_EQ(count_messages(result, Severity::Error, ""), size_t{0});

    // And the lots are solid two-storey blocks, so the permissive path really
    // did carry on rather than dropping the shape.
    for (const Shape& terminal : result.terminals) {
        CHECK_NEAR(geometry_volume(terminal.geometry), 2.0 * 6.0 * 6.0, 1e-9);
    }
}

TEST(OpMass, a_mass_operation_on_a_solid_costs_that_lot_and_not_the_run) {
    // Two lots. The first extrudes before calling `floors`, which is refused;
    // the second does not. The refusal must abandon one subtree and leave the
    // other generated, which is what makes the strict precondition affordable.
    const std::string source =
        "@start\n"
        "rule Main {\n"
        "    split(x) {\n"
        "        3.0 : { Bad(); }\n"
        "        3.0 : { Good(); }\n"
        "    }\n"
        "}\n"
        "\n"
        "rule Bad  { extrude(5.0); floors(2, 3.0); }\n"
        "rule Good { floors(2, 3.0); }\n";

    const GenerationResult result = run_on(source, shape_from_rect(6.0, 6.0));

    CHECK_FALSE(result.ok());
    CHECK_EQ(count_messages(result, Severity::Error,
                            "'floors': the shape already has a height of 5"),
             size_t{1});

    const std::vector<const Shape*> good = terminals_named(result, "Good");
    CHECK_EQ(good.size(), size_t{1});
    CHECK_EQ(terminals_named(result, "Bad").size(), size_t{0});
    if (good.size() == 1) {
        CHECK_NEAR(geometry_volume(good[0]->geometry), 6.0 * 3.0 * 6.0, 1e-9);
    }
}
