// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_interpreter.cpp
 * @brief The rule interpreter: the shape model, the D2 operations, the caps, and the diagnostics
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ================================================================================
 * WHAT THIS SUITE IS FOR
 * ================================================================================
 *
 * D1 gave the language a parser; D2 gives it an evaluator. This file is the first
 * time any of that has been RUN. D3 (split and repeat), D4 (component split, roofs,
 * primitives), D5 (stochastics) and D6 (attributes) all build on the Shape, the
 * Scope and the evaluation loop asserted here, so the assertions are written
 * against the interfaces those features will consume, not against how D2 happens
 * to implement them.
 *
 * ================================================================================
 * HOW THESE TESTS ARE WRITTEN SO THAT THEY CAN FAIL
 * ================================================================================
 *
 * A test that cannot fail is worse than no test, so each of the following is
 * deliberate:
 *
 *   - **Numbers are computed from the geometry, not read back from the scope.**
 *     `extrude(3)` on a unit square is checked by ENCLOSED VOLUME and by SURFACE
 *     AREA, both integrated over the faces. An extruder that built the walls but
 *     forgot a cap keeps `scope.size` at (1,3,1) and fails on area.
 *
 *   - **Winding is checked two ways.** Every face normal must point away from the
 *     solid's centroid, AND the vector areas of every face must sum to zero, which
 *     is only true for a closed surface with consistent winding. A single inverted
 *     face passes neither.
 *
 *   - **Every "restores it" test compares WORLD positions vertex by vertex.**
 *     `rotate_scope`, `mirror_scope` and `align_scope` all promise to move the
 *     frame and leave the geometry where it is. Checking the axes alone would pass
 *     for an implementation that turned the frame and dragged the geometry with it.
 *
 *   - **Every cap test asserts the cap's VALUE, not merely that a cap fired.**
 *     A depth cap of 4 and a depth cap of 8 are run against the same rule file and
 *     must stop at different depths, so a hard-coded constant fails.
 *
 *   - **The determinism tests assert that the compared thing is substantial and
 *     that a perturbation changes it.** Two empty strings compare equal, so the
 *     dump is checked for size and for sensitivity to the seed, to the input
 *     shape's key, and to a change in the rule text, before two runs of the same
 *     input are compared byte for byte.
 *
 *   - **Every error test asserts the MESSAGE and the LOCATION**, down to the line,
 *     column and byte offset. A diagnostic that carries no position is the failure
 *     mode the whole diagnostic design exists to prevent, and it is invisible to a
 *     test that only counts diagnostics.
 *
 * Nothing here needs a GPU, a window or a file on disk, so nothing here skips.
 */

#include "framework.hpp"

#include "procgen/rules/ast.hpp"
#include "procgen/rules/interpreter.hpp"
#include "procgen/rules/lexer.hpp"
#include "procgen/rules/parser.hpp"
#include "procgen/rules/shape.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

using stratum::Mesh;
using stratum::procgen::rules::AlignMode;
using stratum::procgen::rules::align_scope_shape;
using stratum::procgen::rules::Diagnostic;
using stratum::procgen::rules::extrude_shape;
using stratum::procgen::rules::extrude_shape_along_normal;
using stratum::procgen::rules::face_area;
using stratum::procgen::rules::face_normal;
using stratum::procgen::rules::GenerationOptions;
using stratum::procgen::rules::GenerationResult;
using stratum::procgen::rules::generate;
using stratum::procgen::rules::geometry_area;
using stratum::procgen::rules::geometry_bounds;
using stratum::procgen::rules::geometry_volume;
using stratum::procgen::rules::mirror_scope_shape;
using stratum::procgen::rules::mirror_shape;
using stratum::procgen::rules::offset_shape;
using stratum::procgen::rules::OffsetSelector;
using stratum::procgen::rules::OpResult;
using stratum::procgen::rules::parse;
using stratum::procgen::rules::ParseResult;
using stratum::procgen::rules::PivotAnchor;
using stratum::procgen::rules::render_diagnostic;
using stratum::procgen::rules::reverse_normals_shape;
using stratum::procgen::rules::rotate_scope_shape;
using stratum::procgen::rules::rotate_shape;
using stratum::procgen::rules::scale_shape;
using stratum::procgen::rules::ScopeAxis;
using stratum::procgen::rules::setback_shape;
using stratum::procgen::rules::set_pivot_shape;
using stratum::procgen::rules::Severity;
using stratum::procgen::rules::Shape;
using stratum::procgen::rules::ShapeGeometry;
using stratum::procgen::rules::shape_from_rect;
using stratum::procgen::rules::shape_from_polygon;
using stratum::procgen::rules::shape_from_rings;
using stratum::procgen::rules::taper_shape;
using stratum::procgen::rules::translate_shape;
using stratum::procgen::rules::Value;

namespace {

// ============================================================================
// Helpers
// ============================================================================

/// A 2 x 3 x 4 box: the rectangle 2 by 4 in the xz plane, extruded 3 up y.
[[nodiscard]] Shape make_box() {
    Shape shape = shape_from_rect(2.0, 4.0);
    const OpResult result = extrude_shape(shape, ScopeAxis::Y, 3.0);
    CHECK_TRUE(result.ok);
    return shape;
}

void check_vec3(const glm::dvec3& actual, const glm::dvec3& expected, double eps) {
    CHECK_NEAR(actual.x, expected.x, eps);
    CHECK_NEAR(actual.y, expected.y, eps);
    CHECK_NEAR(actual.z, expected.z, eps);
}

/**
 * @brief Does every face normal point away from the interior?
 *
 * The centroid of a convex solid's vertices is inside it, so a face whose normal
 * has a negative component along (face centre - centroid) is inside out. This is
 * the check that catches an extruder whose wall quads are wound the wrong way --
 * a fault that leaves the volume, the area and the scope all correct.
 */
[[nodiscard]] bool every_face_points_outward(const ShapeGeometry& geometry) {
    if (geometry.faces.empty() || geometry.positions.empty()) {
        return false;
    }
    glm::dvec3 centroid{0.0};
    for (const glm::dvec3& position : geometry.positions) {
        centroid += position;
    }
    centroid /= static_cast<double>(geometry.positions.size());

    for (const stratum::procgen::rules::Face& face : geometry.faces) {
        if (face.loop.empty()) {
            return false;
        }
        glm::dvec3 face_centre{0.0};
        for (const uint32_t index : face.loop) {
            face_centre += geometry.positions[index];
        }
        face_centre /= static_cast<double>(face.loop.size());
        if (glm::dot(face_normal(geometry, face), face_centre - centroid) <= 0.0) {
            return false;
        }
    }
    return true;
}

/**
 * @brief The sum of every face's vector area
 *
 * Zero for a closed surface with consistent winding, and for nothing else. It is
 * the one winding check that does not depend on the solid being convex, so it is
 * asserted alongside every_face_points_outward() rather than instead of it.
 */
[[nodiscard]] glm::dvec3 total_vector_area(const ShapeGeometry& geometry) {
    glm::dvec3 total{0.0};
    for (const stratum::procgen::rules::Face& face : geometry.faces) {
        total += face_normal(geometry, face) * face_area(geometry, face);
    }
    return total;
}

/// Index of the one face whose normal is @p normal, or -1 when there is not exactly one
[[nodiscard]] int only_face_facing(const ShapeGeometry& geometry,
                                   const glm::dvec3& normal,
                                   double eps) {
    int found = -1;
    for (size_t i = 0; i < geometry.faces.size(); ++i) {
        const glm::dvec3 n = face_normal(geometry, geometry.faces[i]);
        if (std::fabs(n.x - normal.x) <= eps && std::fabs(n.y - normal.y) <= eps &&
            std::fabs(n.z - normal.z) <= eps) {
            if (found != -1) {
                return -1;  // ambiguous, which is itself a failure at the call site
            }
            found = static_cast<int>(i);
        }
    }
    return found;
}

/// World positions of every vertex, in index order
[[nodiscard]] std::vector<glm::dvec3> world_positions(const Shape& shape) {
    std::vector<glm::dvec3> out;
    out.reserve(shape.geometry.positions.size());
    for (const glm::dvec3& local : shape.geometry.positions) {
        out.push_back(shape.scope.to_world(local));
    }
    return out;
}

/**
 * @brief The world-space bounding box of a shape's vertices
 *
 * Computed through `scope.to_world()` rather than from `scope.origin` and
 * `scope.size`, because a scope that has drifted away from its geometry is
 * exactly the fault this is used to catch: an operation that reads the scope one
 * step too late still reports a box of the right SIZE, in the wrong PLACE.
 */
void world_bounds(const Shape& shape, glm::dvec3& min_out, glm::dvec3& max_out) {
    min_out = glm::dvec3{0.0};
    max_out = glm::dvec3{0.0};
    if (shape.geometry.positions.empty()) {
        return;
    }
    min_out = glm::dvec3{1.0e300};
    max_out = glm::dvec3{-1.0e300};
    for (const glm::dvec3& local : shape.geometry.positions) {
        const glm::dvec3 world = shape.scope.to_world(local);
        min_out = glm::min(min_out, world);
        max_out = glm::max(max_out, world);
    }
}

/// The largest per-component difference between two world-position lists
[[nodiscard]] double worst_drift(const std::vector<glm::dvec3>& a,
                                 const std::vector<glm::dvec3>& b) {
    if (a.size() != b.size() || a.empty()) {
        return 1.0e30;  // a size mismatch must fail the tolerance, not pass it
    }
    double worst = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        const glm::dvec3 d = a[i] - b[i];
        worst = std::max(worst, std::fabs(d.x));
        worst = std::max(worst, std::fabs(d.y));
        worst = std::max(worst, std::fabs(d.z));
    }
    return worst;
}

/**
 * @brief Parse @p source and run it against @p seed
 *
 * The parse is asserted as a rendered string rather than as a boolean, so a test
 * whose rule text has a typo prints the parser's own caret diagnostic instead of
 * "false != true". Every test here supplies source it believes is valid; one that
 * does not is testing the parser by accident.
 */
[[nodiscard]] GenerationResult run_on(const std::string& source,
                                      const Shape& seed,
                                      const GenerationOptions& options = {}) {
    const ParseResult parsed = parse(source, "test.srl");
    CHECK_EQ(parsed.render_all(source), std::string{});
    return generate(parsed.file, seed, options);
}

/// run_on() against the unit square, which is what a lot boundary looks like
[[nodiscard]] GenerationResult run_square(const std::string& source,
                                          const GenerationOptions& options = {}) {
    return run_on(source, shape_from_rect(1.0, 1.0), options);
}

/// run_on() against a rectangle, for anything that needs the two ground axes
/// to differ -- a square cannot tell a u that runs along x from one along z.
[[nodiscard]] GenerationResult run_rect(double size_x, double size_z,
                                        const std::string& source,
                                        const GenerationOptions& options = {}) {
    return run_on(source, shape_from_rect(size_x, size_z), options);
}

/**
 * @brief Are two generations the same shape tree, bit for bit?
 *
 * Compared with `==` and not with a tolerance, because the promise is
 * byte-identical output and a tolerance hides exactly the drift it is there to
 * catch. Compared against the GEOMETRY rather than against dump(), because a
 * dump that quietly stopped printing vertex positions would make a
 * dump-to-dump determinism test pass for a non-deterministic interpreter --
 * which is a test that cannot fail, and this project's most-found defect.
 *
 * Returns false for an empty pair, so a run that produced nothing cannot pass
 * by agreeing with another run that produced nothing.
 */
[[nodiscard]] bool terminals_are_identical(const GenerationResult& a, const GenerationResult& b) {
    if (a.terminals.empty() || a.terminals.size() != b.terminals.size()) {
        return false;
    }
    for (size_t i = 0; i < a.terminals.size(); ++i) {
        const Shape& lhs = a.terminals[i];
        const Shape& rhs = b.terminals[i];
        if (lhs.rule != rhs.rule || lhs.depth != rhs.depth || lhs.index != rhs.index) {
            return false;
        }
        if (lhs.scope.origin != rhs.scope.origin || lhs.scope.size != rhs.scope.size ||
            lhs.scope.pivot != rhs.scope.pivot || lhs.scope.axes != rhs.scope.axes) {
            return false;
        }
        // Attributes are compared through dump(), not here: Value holds a variant
        // with no operator==, so a map comparison would not compile.
        if (lhs.geometry.positions != rhs.geometry.positions) {
            return false;
        }
        if (lhs.geometry.faces.size() != rhs.geometry.faces.size()) {
            return false;
        }
        for (size_t f = 0; f < lhs.geometry.faces.size(); ++f) {
            if (lhs.geometry.faces[f].loop != rhs.geometry.faces[f].loop ||
                lhs.geometry.faces[f].holes != rhs.geometry.faces[f].holes) {
                return false;
            }
        }
    }
    return true;
}

/// Is @p needle a substring of any Error diagnostic?
[[nodiscard]] bool has_error_saying(const GenerationResult& result, const std::string& needle) {
    for (const Diagnostic& diagnostic : result.diagnostics) {
        if (diagnostic.severity == Severity::Error &&
            diagnostic.message.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

/// Does @p message contain @p needle?
[[nodiscard]] bool says(const std::string& message, const std::string& needle) {
    return message.find(needle) != std::string::npos;
}

}  // namespace

// ============================================================================
// Extrude
// ============================================================================

TEST(Interpreter, extrude_a_unit_square_by_three_gives_a_closed_box) {
    Shape shape = shape_from_rect(1.0, 1.0);
    CHECK_EQ(shape.geometry.faces.size(), size_t{1});
    check_vec3(face_normal(shape.geometry, shape.geometry.faces[0]),
               glm::dvec3{0.0, 1.0, 0.0}, 1e-12);

    const OpResult result = extrude_shape(shape, ScopeAxis::Y, 3.0);
    CHECK_TRUE(result.ok);

    // Two caps and four walls. Not "at least six": a prism that emitted the same
    // wall twice would still enclose the right volume.
    CHECK_EQ(shape.geometry.faces.size(), size_t{6});

    // Integrated over the faces, so a missing cap or a wall wound inside out
    // moves these even though the scope box does not.
    CHECK_NEAR(geometry_volume(shape.geometry), 3.0, 1e-12);
    CHECK_NEAR(geometry_area(shape.geometry), 14.0, 1e-12);

    check_vec3(shape.scope.size, glm::dvec3{1.0, 3.0, 1.0}, 1e-12);
    check_vec3(shape.scope.origin, glm::dvec3{0.0, 0.0, 0.0}, 1e-12);
}

TEST(Interpreter, every_face_of_an_extruded_box_is_wound_outwards) {
    Shape shape = shape_from_rect(1.0, 1.0);
    CHECK_TRUE(extrude_shape(shape, ScopeAxis::Y, 3.0).ok);

    CHECK_TRUE(every_face_points_outward(shape.geometry));
    // Closed and consistently wound, which the centroid test alone does not prove.
    check_vec3(total_vector_area(shape.geometry), glm::dvec3{0.0}, 1e-12);

    // The cap where the face was points down, the far cap points up. Exactly one
    // face faces each way, so a prism that emitted two top caps fails here.
    CHECK((only_face_facing(shape.geometry, glm::dvec3{0.0, 1.0, 0.0}, 1e-12)) >= 0);
    CHECK((only_face_facing(shape.geometry, glm::dvec3{0.0, -1.0, 0.0}, 1e-12)) >= 0);
}

TEST(Interpreter, extruding_by_a_negative_distance_still_gives_a_positive_volume) {
    Shape shape = shape_from_rect(1.0, 1.0);
    CHECK_TRUE(extrude_shape(shape, ScopeAxis::Y, -3.0).ok);

    CHECK_EQ(shape.geometry.faces.size(), size_t{6});
    CHECK_NEAR(geometry_volume(shape.geometry), 3.0, 1e-12);
    CHECK_TRUE(every_face_points_outward(shape.geometry));

    // The box is below the original plane, and the scope followed it there.
    check_vec3(shape.scope.size, glm::dvec3{1.0, 3.0, 1.0}, 1e-12);
    check_vec3(shape.scope.origin, glm::dvec3{0.0, -3.0, 0.0}, 1e-12);
}

TEST(Interpreter, extruding_along_the_dominant_normal_uses_the_geometry_not_the_axis) {
    // A footprint stood on edge: its face normal is +z while the scope's y axis is
    // still world up. Extruding along y is then impossible and extruding along the
    // normal is the only reading that works, so the two cannot be confused.
    Shape standing = shape_from_rect(2.0, 4.0);
    rotate_shape(standing, glm::dvec3{90.0, 0.0, 0.0});
    check_vec3(face_normal(standing.geometry, standing.geometry.faces[0]),
               glm::dvec3{0.0, 0.0, 1.0}, 1e-12);

    Shape along_y = standing;
    const OpResult refused = extrude_shape(along_y, ScopeAxis::Y, 2.0);
    CHECK_FALSE(refused.ok);
    CHECK_TRUE(says(refused.message, "parallel"));

    Shape along_normal = standing;
    CHECK_TRUE(extrude_shape_along_normal(along_normal, 2.0).ok);
    CHECK_EQ(along_normal.geometry.faces.size(), size_t{6});
    CHECK_NEAR(geometry_volume(along_normal.geometry), 16.0, 1e-12);
    CHECK_TRUE(every_face_points_outward(along_normal.geometry));
}

TEST(Interpreter, extruding_by_zero_is_refused_rather_than_making_a_flat_solid) {
    Shape shape = shape_from_rect(1.0, 1.0);
    const OpResult result = extrude_shape(shape, ScopeAxis::Y, 0.0);
    CHECK_FALSE(result.ok);
    CHECK_TRUE(says(result.message, "zero"));
    // The refusal left the footprint alone.
    CHECK_EQ(shape.geometry.faces.size(), size_t{1});
}

// ============================================================================
// Taper
// ============================================================================

TEST(Interpreter, taper_leaves_a_top_inset_by_the_height) {
    Shape shape = shape_from_rect(1.0, 1.0);
    CHECK_TRUE(taper_shape(shape, 0.2).ok);

    CHECK_EQ(shape.geometry.faces.size(), size_t{6});
    check_vec3(shape.scope.size, glm::dvec3{1.0, 0.2, 1.0}, 1e-12);

    // The sides rise at 45 degrees, so the top ring is the base ring inset by the
    // height on every edge: 1 - 2 * 0.2 on a side, 0.36 of area. Asserting the
    // AREA rather than the size means a taper that inset by the height on only two
    // of the four edges still fails.
    const int top = only_face_facing(shape.geometry, glm::dvec3{0.0, 1.0, 0.0}, 1e-12);
    CHECK((top) >= 0);
    if (top >= 0) {
        CHECK_NEAR(face_area(shape.geometry, shape.geometry.faces[static_cast<size_t>(top)]),
                   0.36, 1e-12);
    }

    const int bottom = only_face_facing(shape.geometry, glm::dvec3{0.0, -1.0, 0.0}, 1e-12);
    CHECK((bottom) >= 0);
    if (bottom >= 0) {
        CHECK_NEAR(face_area(shape.geometry, shape.geometry.faces[static_cast<size_t>(bottom)]),
                   1.0, 1e-12);
    }

    // The frustum formula, which no amount of getting the top ring wrong satisfies.
    const double expected = 0.2 * (1.0 + 0.36 + std::sqrt(0.36)) / 3.0;
    CHECK_NEAR(geometry_volume(shape.geometry), expected, 1e-12);
    CHECK_TRUE(every_face_points_outward(shape.geometry));
    check_vec3(total_vector_area(shape.geometry), glm::dvec3{0.0}, 1e-12);
}

TEST(Interpreter, a_taper_that_eats_the_outline_is_refused_and_names_the_height) {
    Shape shape = shape_from_rect(1.0, 1.0);
    const OpResult result = taper_shape(shape, 0.6);
    CHECK_FALSE(result.ok);
    // Naming the number is the whole value of the message: 0.6 is fine on a wide
    // lot and impossible on this one, and the author has to know which.
    CHECK_TRUE(says(result.message, "0.6"));
    // Refused, not half-applied.
    CHECK_EQ(shape.geometry.faces.size(), size_t{1});
    CHECK_NEAR(geometry_area(shape.geometry), 1.0, 1e-12);
}

TEST(Interpreter, shape_from_rings_rewinds_a_hole_handed_in_the_wrong_way) {
    // shape.hpp calls the winding fix "the one place that has to be right",
    // and every test that builds a holed shape hands it a CORRECTLY wound hole
    // -- so the fix itself was never exercised. Deleting it passed the whole
    // suite. That matters more now than it used to: E2 carries courtyards
    // through the massing and D5 reads interior rings, so a hole wound the
    // wrong way is a wrong building rather than a wrong number.
    //
    // The face contract is outer counter-clockwise, holes clockwise, because
    // that is what puts the material on the LEFT of every directed edge and
    // lets one offset rule serve both rings.
    const std::vector<glm::dvec2> outer = {{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}};

    // Clockwise, which is what a hole should be.
    const std::vector<glm::dvec2> hole_cw = {{3.0, 3.0}, {3.0, 7.0}, {7.0, 7.0}, {7.0, 3.0}};
    // The same ring the other way round. An author, or an importer, can hand
    // either; the result must not depend on which.
    const std::vector<glm::dvec2> hole_ccw = {{3.0, 3.0}, {7.0, 3.0}, {7.0, 7.0}, {3.0, 7.0}};

    const Shape from_cw = shape_from_rings(outer, {hole_cw});
    const Shape from_ccw = shape_from_rings(outer, {hole_ccw});

    // 100 - 16 either way. Area alone is not enough -- it is a signed sum, and
    // a hole left wound the wrong way ADDS instead of subtracting, so this is
    // the assertion that bites.
    CHECK_NEAR(geometry_area(from_cw.geometry), 84.0, 1e-12);
    CHECK_NEAR(geometry_area(from_ccw.geometry), 84.0, 1e-12);

    CHECK_EQ(from_cw.geometry.faces.size(), size_t{1});
    CHECK_EQ(from_ccw.geometry.faces.size(), size_t{1});
    CHECK_EQ(from_cw.geometry.faces[0].holes.size(), size_t{1});
    CHECK_EQ(from_ccw.geometry.faces[0].holes.size(), size_t{1});

    // And it survives the operation that reads the winding. An inset of the
    // face draws the outer in by 1 on each side and pushes the hole out by 1 on
    // each side, so 10x10 with a 4x4 hole becomes 8x8 with a 6x6 hole:
    // 64 - 36 = 28.
    Shape a = from_cw;
    Shape b = from_ccw;
    CHECK_TRUE(taper_shape(a, 1.0).ok);
    CHECK_TRUE(taper_shape(b, 1.0).ok);
    const int top_a = only_face_facing(a.geometry, glm::dvec3{0.0, 1.0, 0.0}, 1e-9);
    const int top_b = only_face_facing(b.geometry, glm::dvec3{0.0, 1.0, 0.0}, 1e-9);
    CHECK((top_a) >= 0);
    CHECK((top_b) >= 0);
    if (top_a >= 0 && top_b >= 0) {
        CHECK_NEAR(face_area(a.geometry, a.geometry.faces[static_cast<size_t>(top_a)]), 28.0, 1e-9);
        CHECK_NEAR(face_area(b.geometry, b.geometry.faces[static_cast<size_t>(top_b)]), 28.0, 1e-9);
    }
}

TEST(Interpreter, taper_grows_a_hole_while_it_shrinks_the_outline) {
    // The case no taper test covered, and the one that was wrong. Every taper
    // test above uses shape_from_rect, which has no hole, so the per-ring
    // normal in miter_inset() could point the wrong way for holes and the whole
    // suite stayed green.
    //
    // A 20x20 slab with a 4x4 lightwell at (8,8)-(12,12), tapered by 1 m. The
    // sides rise at 45 degrees, so every edge moves 1 m into the MATERIAL: the
    // outer draws in to 18x18, and the hole boundary pushes out to 6x6. A hole
    // that shrank would mean its wall overhung the void.
    const std::vector<glm::dvec2> outer = {{0.0, 0.0}, {20.0, 0.0}, {20.0, 20.0}, {0.0, 20.0}};
    const std::vector<std::vector<glm::dvec2>> holes = {
        {{8.0, 8.0}, {12.0, 8.0}, {12.0, 12.0}, {8.0, 12.0}}};
    Shape shape = shape_from_rings(outer, holes);
    CHECK_NEAR(geometry_area(shape.geometry), 400.0 - 16.0, 1e-9);

    CHECK_TRUE(taper_shape(shape, 1.0).ok);

    const int top = only_face_facing(shape.geometry, glm::dvec3{0.0, 1.0, 0.0}, 1e-9);
    CHECK((top) >= 0);
    if (top >= 0) {
        // 18*18 - 6*6 = 288. The broken version shrank the hole to 2x2 and gave
        // 18*18 - 2*2 = 320, so this number alone separates the two.
        CHECK_NEAR(face_area(shape.geometry, shape.geometry.faces[static_cast<size_t>(top)]),
                   288.0, 1e-9);
        CHECK_EQ(shape.geometry.faces[static_cast<size_t>(top)].holes.size(), size_t{1});
    }

    // Derived independently of the implementation: for a prismatoid whose cross
    // section at height t is (20-2t)^2 - (4+2t)^2, the volume is
    // integral over [0,1] of (384 - 96t) dt = 384 - 48 = 336. A shrinking hole
    // integrates (384 - 64t) dt = 352 instead.
    CHECK_NEAR(geometry_volume(shape.geometry), 336.0, 1e-9);
}

TEST(Interpreter, a_taper_deeper_than_half_a_hole_is_still_allowed) {
    // The other half of the same bug: with the hole shrinking, the fold check
    // fired on legitimate input and refused any taper past half the smallest
    // hole width. On a 20x20 slab with a 4x4 hole, taper(3.0) is an ordinary
    // frustum -- outer 14x14, hole 10x10 -- and must be accepted.
    const std::vector<glm::dvec2> outer = {{0.0, 0.0}, {20.0, 0.0}, {20.0, 20.0}, {0.0, 20.0}};
    const std::vector<std::vector<glm::dvec2>> holes = {
        {{8.0, 8.0}, {12.0, 8.0}, {12.0, 12.0}, {8.0, 12.0}}};
    Shape shape = shape_from_rings(outer, holes);

    const OpResult result = taper_shape(shape, 3.0);
    CHECK_TRUE(result.ok);

    const int top = only_face_facing(shape.geometry, glm::dvec3{0.0, 1.0, 0.0}, 1e-9);
    CHECK((top) >= 0);
    if (top >= 0) {
        // 14*14 - 10*10 = 96.
        CHECK_NEAR(face_area(shape.geometry, shape.geometry.faces[static_cast<size_t>(top)]),
                   96.0, 1e-9);
    }
}

TEST(Interpreter, scale_sets_the_size_of_a_shape_that_has_no_geometry) {
    // shape.hpp reserves the geometry-less shape so that a rule can build a
    // scope and then insert an asset into it. refit_scope() leaves such a scope
    // alone by design, so `scale` is the only operation that can set its size --
    // and it used to derive the size from geometry that was not there, return
    // success, and change nothing.
    Shape shape;
    shape.scope.size = glm::dvec3{1.0, 1.0, 1.0};
    CHECK_TRUE(shape.geometry.positions.empty());

    const OpResult result = scale_shape(shape, glm::dvec3{10.0, 3.0, 8.0});
    CHECK_TRUE(result.ok);
    check_vec3(shape.scope.size, glm::dvec3{10.0, 3.0, 8.0}, 1e-12);

    // From a zero scope too -- that is what a freshly constructed Scope is, and
    // it is what a rule building a scope from nothing starts with.
    Shape fresh;
    CHECK_TRUE(scale_shape(fresh, glm::dvec3{2.0, 4.0, 6.0}).ok);
    check_vec3(fresh.scope.size, glm::dvec3{2.0, 4.0, 6.0}, 1e-12);

    // Still no geometry: scale sets the frame, it does not invent a box.
    CHECK_TRUE(fresh.geometry.positions.empty());
}

TEST(Interpreter, scale_names_the_axes_it_skipped_in_the_plural) {
    // A flat footprint has no thickness to multiply, and the message has to say
    // so. With geometry present the per-axis refusal still applies -- this is
    // only about it reading as English when more than one axis is skipped.
    Shape shape = shape_from_rect(2.0, 2.0);
    const OpResult result = scale_shape(shape, glm::dvec3{4.0, 1.0, 4.0});
    CHECK_FALSE(result.ok);
    CHECK_TRUE(says(result.message, "that axis"));
    CHECK_FALSE(says(result.message, "those axes"));
}

TEST(Interpreter, a_taper_of_a_non_positive_height_is_refused) {
    Shape shape = shape_from_rect(1.0, 1.0);
    CHECK_FALSE(taper_shape(shape, 0.0).ok);
    CHECK_FALSE(taper_shape(shape, -1.0).ok);
    CHECK_EQ(shape.geometry.faces.size(), size_t{1});
}

// ============================================================================
// Setback and offset
// ============================================================================

TEST(Interpreter, setback_keeps_the_inner_part_and_hands_the_border_back) {
    Shape shape = shape_from_rect(1.0, 1.0);
    ShapeGeometry border;
    const OpResult result = setback_shape(shape, 0.1, border, true);
    CHECK_TRUE(result.ok);

    // 0.8 by 0.8 remains.
    CHECK_NEAR(geometry_area(shape.geometry), 0.64, 1e-9);
    check_vec3(shape.scope.size, glm::dvec3{0.8, 0.0, 0.8}, 1e-9);
    check_vec3(shape.scope.origin, glm::dvec3{0.1, 0.0, 0.1}, 1e-9);

    // The border is the rest of the original, and nothing is counted twice: this
    // is the whole difference between setback and offset(-d).
    CHECK_NEAR(geometry_area(border), 0.36, 1e-9);
    CHECK_NEAR(geometry_area(shape.geometry) + geometry_area(border), 1.0, 1e-9);

    // One annulus, not two overlapping rectangles: an outer ring with a hole in it.
    CHECK_EQ(border.faces.size(), size_t{1});
    CHECK_EQ(border.faces[0].holes.size(), size_t{1});
}

TEST(Interpreter, setback_can_discard_the_border) {
    Shape shape = shape_from_rect(1.0, 1.0);
    ShapeGeometry border;
    CHECK_TRUE(setback_shape(shape, 0.1, border, false).ok);
    CHECK_NEAR(geometry_area(shape.geometry), 0.64, 1e-9);
    CHECK_TRUE(border.faces.empty());
}

TEST(Interpreter, a_setback_that_does_not_shrink_the_shape_is_refused) {
    Shape shape = shape_from_rect(1.0, 1.0);
    ShapeGeometry border;
    const OpResult zero = setback_shape(shape, 0.0, border, true);
    CHECK_FALSE(zero.ok);
    CHECK_TRUE(says(zero.message, "positive"));
    CHECK_FALSE(setback_shape(shape, -0.1, border, true).ok);
    CHECK_NEAR(geometry_area(shape.geometry), 1.0, 1e-12);

    // A setback that removes everything is a refusal, not an empty shape.
    Shape small = shape_from_rect(1.0, 1.0);
    CHECK_FALSE(setback_shape(small, 0.9, border, true).ok);
}

TEST(Interpreter, offset_inside_and_offset_border_partition_the_original_face) {
    Shape inside = shape_from_rect(1.0, 1.0);
    CHECK_TRUE(offset_shape(inside, -0.1, OffsetSelector::Inside).ok);
    CHECK_NEAR(geometry_area(inside.geometry), 0.64, 1e-9);

    Shape border = shape_from_rect(1.0, 1.0);
    CHECK_TRUE(offset_shape(border, -0.1, OffsetSelector::Border).ok);
    CHECK_NEAR(geometry_area(border.geometry), 0.36, 1e-9);

    // "all" is both, as separate faces, and together they are the original.
    Shape all = shape_from_rect(1.0, 1.0);
    CHECK_TRUE(offset_shape(all, -0.1, OffsetSelector::All).ok);
    CHECK_EQ(all.geometry.faces.size(), size_t{2});
    CHECK_NEAR(geometry_area(all.geometry), 1.0, 1e-9);
}

TEST(Interpreter, an_offset_too_small_to_move_anything_is_refused) {
    // Everything here goes through Clipper2 on a 1e-5 integer grid, so a
    // distance below one count rounds both outlines onto the same grid points
    // and changes nothing. That used to be ACCEPTED: the guard was
    // kPointEpsilon at 1e-9, leaving a dead band three and a half orders of
    // magnitude wide where the call reported success, the result was non-empty
    // so no "removed everything" guard fired, and the shape was untouched.
    for (const double tiny : {1e-8, 1e-7, 1e-6, 4e-6}) {
        Shape shape = shape_from_rect(10.0, 10.0);
        const OpResult result = offset_shape(shape, -tiny, OffsetSelector::Inside);
        CHECK_FALSE(result.ok);
        // The message names the resolution, because "too small" sends the
        // author looking for a bug in their own arithmetic.
        CHECK_TRUE(says(result.message, "1e-5"));
        CHECK_NEAR(geometry_area(shape.geometry), 100.0, 1e-12);
    }

    // Just above one count still works, and actually removes something.
    Shape shape = shape_from_rect(10.0, 10.0);
    CHECK_TRUE(offset_shape(shape, -2e-5, OffsetSelector::Inside).ok);
    CHECK((geometry_area(shape.geometry)) < 100.0);
}

TEST(Interpreter, a_setback_too_small_to_remove_anything_is_refused) {
    // Same dead band, and worse here: setback_shape's contract is that it hands
    // the removed border back. Succeeding with an empty border while
    // keep_border is true is a lie about what the operation did.
    ShapeGeometry border;
    for (const double tiny : {1e-8, 1e-6, 4e-6}) {
        Shape shape = shape_from_rect(10.0, 10.0);
        const OpResult result = setback_shape(shape, tiny, border, true);
        CHECK_FALSE(result.ok);
        CHECK_TRUE(says(result.message, "1e-5"));
        CHECK_TRUE(border.faces.empty());
        CHECK_NEAR(geometry_area(shape.geometry), 100.0, 1e-12);
    }
}

TEST(Interpreter, setback_partitions_a_footprint_that_is_not_on_the_grid) {
    // Every other partition test uses dimensions that are exact multiples of the
    // 1e-5 Clipper grid, so the quantisation is invisible to them and the 1e-9
    // tolerance they assert is free. A real footprint is not grid-aligned.
    //
    // This pins the actual precision rather than pretending it is exact: the
    // partition holds to a fraction of a square millimetre on a 234 m plot,
    // which is what a 10 micrometre coordinate grid buys. A test asserting
    // 1e-9 here would fail, and a test with no non-grid case at all -- which is
    // what existed -- hides the limit entirely.
    const std::vector<glm::dvec2> ring = {
        {0.0, 0.0}, {18.374829175, 0.0}, {19.203718264, 11.847261935},
        {7.918273645, 14.372615908}, {0.0, 9.283746152}};
    Shape shape = shape_from_polygon(ring);
    const double total = geometry_area(shape.geometry);
    CHECK((total) > 200.0);

    ShapeGeometry border;
    CHECK_TRUE(setback_shape(shape, 1.5, border, true).ok);

    const double inner = geometry_area(shape.geometry);
    const double removed = geometry_area(border);
    CHECK((inner) > 0.0);
    CHECK((removed) > 0.0);
    CHECK_NEAR(inner + removed, total, 1e-3);
}

TEST(Interpreter, a_positive_offset_through_each_selector) {
    // The existing positive-offset test only ever passes OffsetSelector::Inside,
    // so the Border and All paths had no coverage at all -- which is why
    // replacing the border's Xor with a Difference passed the whole suite.
    //
    // 10x10 grown by 1 on every side is 12x12 = 144, and the ring between the
    // two outlines is 144 - 100 = 44.
    {
        Shape shape = shape_from_rect(10.0, 10.0);
        CHECK_TRUE(offset_shape(shape, 1.0, OffsetSelector::Inside).ok);
        CHECK_NEAR(geometry_area(shape.geometry), 144.0, 1e-9);
        CHECK_EQ(shape.geometry.faces.size(), size_t{1});
        CHECK_EQ(shape.geometry.faces[0].holes.size(), size_t{0});
    }
    {
        Shape shape = shape_from_rect(10.0, 10.0);
        CHECK_TRUE(offset_shape(shape, 1.0, OffsetSelector::Border).ok);
        CHECK_NEAR(geometry_area(shape.geometry), 44.0, 1e-9);
        // The ring is ONE face with the original outline as a hole, not two
        // faces or four quads. That is what makes it usable as a shape.
        CHECK_EQ(shape.geometry.faces.size(), size_t{1});
        CHECK_EQ(shape.geometry.faces[0].holes.size(), size_t{1});
    }
    {
        // All hands back both pieces as separate faces: 144 + 44 = 188.
        //
        // Note that for a POSITIVE distance those two OVERLAP -- the grown
        // 12x12 outline contains the ring -- so the total is not the area of
        // anything. That is only a partition for a NEGATIVE distance, which is
        // the case setback() is built on and the case
        // offset_inside_and_offset_border_partition_the_original_face covers.
        // Pinned here so the difference is recorded rather than discovered.
        Shape shape = shape_from_rect(10.0, 10.0);
        CHECK_TRUE(offset_shape(shape, 1.0, OffsetSelector::All).ok);
        CHECK_EQ(shape.geometry.faces.size(), size_t{2});
        CHECK_NEAR(geometry_area(shape.geometry), 188.0, 1e-9);
    }
}

TEST(Interpreter, a_positive_offset_grows_the_outline) {
    Shape shape = shape_from_rect(1.0, 1.0);
    CHECK_TRUE(offset_shape(shape, 0.5, OffsetSelector::Inside).ok);
    // A square grown by 0.5 with a mitred corner is 2 by 2.
    CHECK_NEAR(geometry_area(shape.geometry), 4.0, 1e-6);
    check_vec3(shape.scope.size, glm::dvec3{2.0, 0.0, 2.0}, 1e-6);
}

// ============================================================================
// Scope operations
// ============================================================================

TEST(Interpreter, translate_moves_the_shape_in_the_world_and_keeps_the_box_tight) {
    Shape shape = shape_from_rect(2.0, 4.0);
    const std::vector<glm::dvec3> before = world_positions(shape);

    translate_shape(shape, glm::dvec3{5.0, 1.0, -2.0});

    // The size did not change, the world position did, and the box minimum is
    // still at local zero -- which is the invariant every later operation reads.
    check_vec3(shape.scope.size, glm::dvec3{2.0, 0.0, 4.0}, 1e-12);
    check_vec3(shape.scope.origin, glm::dvec3{5.0, 1.0, -2.0}, 1e-12);

    const std::vector<glm::dvec3> after = world_positions(shape);
    CHECK_EQ(before.size(), after.size());
    for (size_t i = 0; i < before.size() && i < after.size(); ++i) {
        check_vec3(after[i] - before[i], glm::dvec3{5.0, 1.0, -2.0}, 1e-12);
    }
}

TEST(Interpreter, rotate_turns_the_geometry_and_leaves_the_axes_alone) {
    Shape shape = shape_from_rect(2.0, 4.0);
    rotate_shape(shape, glm::dvec3{0.0, 90.0, 0.0});

    // The frame did not move. A quarter turn is exact, by deg_sin_cos().
    check_vec3(shape.scope.axes[0], glm::dvec3{1.0, 0.0, 0.0}, 0.0);
    check_vec3(shape.scope.axes[1], glm::dvec3{0.0, 1.0, 0.0}, 0.0);
    check_vec3(shape.scope.axes[2], glm::dvec3{0.0, 0.0, 1.0}, 0.0);

    // The geometry did: 2 by 4 became 4 by 2, and the tight box followed.
    check_vec3(shape.scope.size, glm::dvec3{4.0, 0.0, 2.0}, 1e-12);
    check_vec3(shape.scope.origin, glm::dvec3{0.0, 0.0, -2.0}, 1e-12);
    CHECK_NEAR(geometry_area(shape.geometry), 8.0, 1e-12);
}

TEST(Interpreter, a_compound_rotation_applies_the_axes_in_the_stated_order) {
    // Every other rotation test turns about ONE axis, where the order does not
    // matter and euler_matrix() could compose rz*ry*rx or rx*ry*rz and pass
    // either way. The header states an order; nothing held it to it.
    //
    // 90 degrees about x then 90 about y. The two orders send the SAME point to
    // DIFFERENT places, which is the whole reason a fixed order has to be
    // stated:
    //
    //   rz*ry*rx applied to (0,0,1):  rx sends it to (0,-1,0), then ry leaves
    //                                 y alone -> (0,-1,0)
    //   rx*ry*rz applied to (0,0,1):  ry sends it to (1,0,0), then rx sends
    //                                 x to x -> (1,0,0)
    //
    // So probing the image of the local +z corner separates them outright.
    Shape shape = shape_from_rect(2.0, 2.0);
    // A point at the far corner of the footprint, before any rotation:
    // shape_from_rect lays the rectangle on the ground plane, so take the
    // world-space bounds after the turn rather than guessing an index.
    rotate_shape(shape, glm::dvec3{90.0, 90.0, 0.0});

    glm::dvec3 low{0.0};
    glm::dvec3 high{0.0};
    CHECK_TRUE(geometry_bounds(shape.geometry, low, high));

    // A 2x2 ground rectangle has extent (2, 0, 2). Turning 90 about x makes it
    // (2, 2, 0); then 90 about y makes it (0, 2, 2). The other composition
    // order gives (2, 2, 0) instead, so the extent alone tells them apart.
    const glm::dvec3 extent = high - low;
    check_vec3(extent, glm::dvec3{0.0, 2.0, 2.0}, 1e-9);
}

TEST(Interpreter, rotate_scope_turns_the_frame_and_leaves_the_geometry_in_the_world) {
    Shape shape = make_box();
    const std::vector<glm::dvec3> before = world_positions(shape);

    rotate_scope_shape(shape, glm::dvec3{0.0, 90.0, 0.0});

    // Every vertex is where it was. This is the assertion that separates
    // rotate_scope from rotate; checking only the axes would pass for an
    // implementation that turned the frame and dragged the geometry along.
    CHECK_NEAR(worst_drift(before, world_positions(shape)), 0.0, 1e-12);

    // EXACTLY, with a tolerance of zero. A quarter turn goes through
    // deg_sin_cos()'s exact path, which is the single mechanism shape.hpp names
    // for cross-platform determinism; libm's cos(pi/2) is 6.1e-17 and would sit
    // comfortably inside a 1e-12 tolerance, so a tolerance here would not notice
    // that path being bypassed.
    check_vec3(shape.scope.axes[0], glm::dvec3{0.0, 0.0, -1.0}, 0.0);
    check_vec3(shape.scope.axes[1], glm::dvec3{0.0, 1.0, 0.0}, 0.0);
    check_vec3(shape.scope.axes[2], glm::dvec3{1.0, 0.0, 0.0}, 0.0);

    // The box of a turned frame is a different box, and it is recomputed.
    check_vec3(shape.scope.size, glm::dvec3{4.0, 3.0, 2.0}, 0.0);
}

TEST(Interpreter, a_quarter_turn_is_exact_and_not_merely_close) {
    // What shape.hpp sells as the reason a rule file is bit-identical on every
    // platform: multiples of 90 degrees never reach libm. std::cos(pi/2) is
    // 6.123e-17, so every tolerance this suite uses would accept the fallthrough
    // and the property would be untested. These are the only assertions in the
    // file that must hold to the last bit, so they are written with eps 0.0.
    const double angles[] = {0.0, 90.0, 180.0, 270.0, 360.0, -90.0, -180.0, 450.0, 1080.0};
    const double sines[] = {0.0, 1.0, 0.0, -1.0, 0.0, -1.0, 0.0, 1.0, 0.0};
    const double cosines[] = {1.0, 0.0, -1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 1.0};

    for (size_t i = 0; i < sizeof(angles) / sizeof(angles[0]); ++i) {
        double sine = 12.0;
        double cosine = 12.0;
        stratum::procgen::rules::deg_sin_cos(angles[i], sine, cosine);
        CHECK_NEAR(sine, sines[i], 0.0);
        CHECK_NEAR(cosine, cosines[i], 0.0);
    }

    // The control: an angle that is NOT a quarter turn does go to libm, so the
    // exact path is a special case and not an implementation that returns 0 and
    // 1 for everything.
    double sine = 0.0;
    double cosine = 0.0;
    stratum::procgen::rules::deg_sin_cos(45.0, sine, cosine);
    CHECK_NEAR(sine, 0.70710678118654752, 1e-15);
    CHECK_NEAR(cosine, 0.70710678118654752, 1e-15);
}

TEST(Interpreter, scale_sets_an_absolute_size_not_a_factor) {
    Shape shape = make_box();
    check_vec3(shape.scope.size, glm::dvec3{2.0, 3.0, 4.0}, 1e-12);
    CHECK_NEAR(geometry_volume(shape.geometry), 24.0, 1e-12);

    CHECK_TRUE(scale_shape(shape, glm::dvec3{4.0, 6.0, 8.0}).ok);

    check_vec3(shape.scope.size, glm::dvec3{4.0, 6.0, 8.0}, 1e-12);
    // Doubled on every axis, so eight times the volume. A scale that treated its
    // arguments as factors would give 2*3*4 * 4*6*8 here and fail.
    CHECK_NEAR(geometry_volume(shape.geometry), 192.0, 1e-12);
    CHECK_TRUE(every_face_points_outward(shape.geometry));
}

TEST(Interpreter, scale_applies_what_it_can_and_names_the_axis_it_skipped) {
    Shape flat = shape_from_rect(2.0, 4.0);
    const OpResult result = scale_shape(flat, glm::dvec3{4.0, 5.0, 2.0});

    CHECK_FALSE(result.ok);
    CHECK_TRUE(says(result.message, "y"));
    // x and z were applied anyway: a partial answer with a message beats losing a
    // building over one flat axis.
    check_vec3(flat.scope.size, glm::dvec3{4.0, 0.0, 2.0}, 1e-12);
    CHECK_NEAR(geometry_area(flat.geometry), 8.0, 1e-12);
}

TEST(Interpreter, scale_refuses_a_negative_size_and_changes_nothing) {
    Shape shape = make_box();
    const double before = geometry_volume(shape.geometry);
    const OpResult result = scale_shape(shape, glm::dvec3{-1.0, 3.0, 4.0});

    CHECK_FALSE(result.ok);
    CHECK_TRUE(says(result.message, "negative"));
    CHECK_NEAR(geometry_volume(shape.geometry), before, 0.0);
    check_vec3(shape.scope.size, glm::dvec3{2.0, 3.0, 4.0}, 0.0);
}

TEST(Interpreter, set_pivot_changes_what_rotate_turns_about) {
    // The same quarter turn about the same axis, differing only in the pivot. A
    // set_pivot that did nothing would make these two agree.
    Shape about_origin = shape_from_rect(1.0, 1.0);
    rotate_shape(about_origin, glm::dvec3{0.0, 90.0, 0.0});
    check_vec3(about_origin.scope.origin, glm::dvec3{0.0, 0.0, -1.0}, 1e-12);

    Shape about_centre = shape_from_rect(1.0, 1.0);
    set_pivot_shape(about_centre, PivotAnchor::Center);
    check_vec3(about_centre.scope.pivot, glm::dvec3{0.5, 0.5, 0.5}, 0.0);
    rotate_shape(about_centre, glm::dvec3{0.0, 90.0, 0.0});
    // A square turned about its own centre maps onto itself.
    check_vec3(about_centre.scope.origin, glm::dvec3{0.0, 0.0, 0.0}, 1e-12);
    check_vec3(about_centre.scope.size, glm::dvec3{1.0, 0.0, 1.0}, 1e-12);
}

TEST(Interpreter, align_scope_world_undoes_a_rotated_frame) {
    Shape shape = make_box();
    const std::vector<glm::dvec3> before = world_positions(shape);

    rotate_scope_shape(shape, glm::dvec3{0.0, 90.0, 0.0});
    CHECK_TRUE(align_scope_shape(shape, AlignMode::World).ok);

    // Exactly, for the reason given in rotate_scope_turns_the_frame...: a quarter
    // turn that went through libm would land within 1e-16 of these and a
    // tolerance would not see it.
    check_vec3(shape.scope.axes[0], glm::dvec3{1.0, 0.0, 0.0}, 0.0);
    check_vec3(shape.scope.axes[1], glm::dvec3{0.0, 1.0, 0.0}, 0.0);
    check_vec3(shape.scope.axes[2], glm::dvec3{0.0, 0.0, 1.0}, 0.0);
    check_vec3(shape.scope.size, glm::dvec3{2.0, 3.0, 4.0}, 0.0);
    CHECK_NEAR(worst_drift(before, world_positions(shape)), 0.0, 1e-12);
}

TEST(Interpreter, align_scope_y_up_keeps_world_up_and_refuses_a_vertical_x) {
    // The tilt is about y AND z, so the scope's x axis is neither the world x
    // axis nor horizontal. Both halves of shape.hpp's YUp contract therefore
    // have something to do: "y is world up" is not already true, and "keep the
    // current x as far as it is horizontal" has a real x to project. A tilt
    // about x alone leaves axes[0] at (1,0,0), for which YUp and World agree and
    // an implementation that simply reset to the world axes would pass.
    Shape tilted = make_box();
    rotate_scope_shape(tilted, glm::dvec3{0.0, 45.0, 30.0});
    const std::vector<glm::dvec3> before = world_positions(tilted);
    CHECK((std::fabs(tilted.scope.axes[1].y - 1.0)) > 1e-6);

    // What "as far as it is horizontal" means, computed from the frame as it
    // stands rather than pasted in: drop the vertical component of x and
    // renormalise. The premise of the test is that this is NOT the world x axis.
    const glm::dvec3 tilted_x = tilted.scope.axes[0];
    CHECK((std::fabs(tilted_x.y)) > 0.1);
    const glm::dvec3 expected_x = glm::normalize(glm::dvec3{tilted_x.x, 0.0, tilted_x.z});
    CHECK((std::fabs(expected_x.x - 1.0)) > 0.1);

    CHECK_TRUE(align_scope_shape(tilted, AlignMode::YUp).ok);
    check_vec3(tilted.scope.axes[1], glm::dvec3{0.0, 1.0, 0.0}, 0.0);
    check_vec3(tilted.scope.axes[0], expected_x, 1e-12);
    // Right-handed, and z derived rather than left wherever it was.
    check_vec3(tilted.scope.axes[2],
               glm::cross(tilted.scope.axes[0], tilted.scope.axes[1]), 1e-12);
    CHECK_NEAR(worst_drift(before, world_positions(tilted)), 0.0, 1e-12);

    // With x exactly vertical there is no horizontal direction to keep, and that
    // is a message rather than a normalise of a zero vector.
    Shape vertical_x = make_box();
    rotate_scope_shape(vertical_x, glm::dvec3{0.0, 0.0, 90.0});
    check_vec3(vertical_x.scope.axes[0], glm::dvec3{0.0, 1.0, 0.0}, 1e-12);
    const OpResult refused = align_scope_shape(vertical_x, AlignMode::YUp);
    CHECK_FALSE(refused.ok);
    CHECK_TRUE(says(refused.message, "vertical"));
}

TEST(Interpreter, align_scope_geometry_points_y_along_the_dominant_face_normal) {
    // The 2 x 3 x 4 box's largest faces are the two 4 x 3 walls, whose normal is
    // the world x axis. The tie between the two is broken by the lower index, so
    // the answer is +x and not -x, whichever order the faces were built in.
    Shape shape = make_box();
    const std::vector<glm::dvec3> before = world_positions(shape);

    CHECK_TRUE(align_scope_shape(shape, AlignMode::Geometry).ok);

    check_vec3(shape.scope.axes[1], glm::dvec3{1.0, 0.0, 0.0}, 1e-12);
    // x runs along that face's longest edge, which is 4 long, and z follows.
    check_vec3(shape.scope.size, glm::dvec3{4.0, 2.0, 3.0}, 1e-12);
    CHECK_NEAR(worst_drift(before, world_positions(shape)), 0.0, 1e-12);
}

TEST(Interpreter, align_scope_geometry_refuses_a_shape_with_no_face) {
    Shape empty;
    const OpResult result = align_scope_shape(empty, AlignMode::Geometry);
    CHECK_FALSE(result.ok);
    CHECK_TRUE(says(result.message, "no face"));
}

TEST(Interpreter, mirror_flips_the_geometry_and_keeps_the_normals_outward) {
    Shape shape = make_box();
    set_pivot_shape(shape, PivotAnchor::Center);
    mirror_shape(shape, ScopeAxis::X);

    // A reflection has determinant -1, so a mirror that did not reverse the faces
    // would leave the solid inside out and the volume negative.
    CHECK_NEAR(geometry_volume(shape.geometry), 24.0, 1e-12);
    CHECK_TRUE(every_face_points_outward(shape.geometry));
    check_vec3(total_vector_area(shape.geometry), glm::dvec3{0.0}, 1e-12);
    check_vec3(shape.scope.size, glm::dvec3{2.0, 3.0, 4.0}, 1e-12);
}

TEST(Interpreter, mirror_scope_leaves_the_geometry_in_the_world_and_flips_the_frame) {
    Shape shape = make_box();
    const std::vector<glm::dvec3> before = world_positions(shape);

    mirror_scope_shape(shape, ScopeAxis::X);

    CHECK_NEAR(worst_drift(before, world_positions(shape)), 0.0, 1e-12);
    check_vec3(shape.scope.axes[0], glm::dvec3{-1.0, 0.0, 0.0}, 1e-12);
    // Left-handed on purpose: orthonormalise() must not quietly put it back.
    CHECK_NEAR(glm::determinant(shape.scope.axes), -1.0, 1e-12);
    check_vec3(shape.scope.size, glm::dvec3{2.0, 3.0, 4.0}, 1e-12);
}

TEST(Interpreter, reverse_normals_turns_the_solid_inside_out) {
    Shape shape = make_box();
    CHECK_NEAR(geometry_volume(shape.geometry), 24.0, 1e-12);
    reverse_normals_shape(shape);
    CHECK_NEAR(geometry_volume(shape.geometry), -24.0, 1e-12);
    CHECK_FALSE(every_face_points_outward(shape.geometry));
}

// ============================================================================
// The evaluation loop
// ============================================================================

TEST(Interpreter, a_rule_with_no_children_is_the_terminal) {
    const GenerationResult result = run_square(
        "@start\n"
        "rule Main {\n"
        "    extrude(3.0);\n"
        "}\n");

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.terminals.size(), size_t{1});
    CHECK_EQ(result.stats.shapes_created, uint32_t{1});
    CHECK_EQ(result.stats.terminals, uint32_t{1});
    CHECK_EQ(result.stats.operations_applied, uint32_t{1});
    if (!result.terminals.empty()) {
        CHECK_EQ(result.terminals[0].rule, std::string{"Main"});
        CHECK_NEAR(geometry_volume(result.terminals[0].geometry), 3.0, 1e-12);
    }
}

TEST(Interpreter, each_rule_call_sees_the_operations_that_ran_before_it) {
    // Two children of one body. The second must see the second translate as well
    // as the first, which is what makes a body a sequence rather than a set.
    const GenerationResult result = run_square(
        "@start\n"
        "rule Main {\n"
        "    translate(1, 0, 0);\n"
        "    A();\n"
        "    translate(2, 0, 0);\n"
        "    B();\n"
        "}\n"
        "rule A { extrude(1.0); }\n"
        "rule B { extrude(2.0); }\n");

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.terminals.size(), size_t{2});
    if (result.terminals.size() == 2) {
        CHECK_EQ(result.terminals[0].rule, std::string{"A"});
        CHECK_EQ(result.terminals[0].index, uint32_t{0});
        CHECK_EQ(result.terminals[0].depth, uint32_t{1});
        CHECK_NEAR(result.terminals[0].scope.origin.x, 1.0, 1e-12);
        CHECK_NEAR(geometry_volume(result.terminals[0].geometry), 1.0, 1e-12);

        CHECK_EQ(result.terminals[1].rule, std::string{"B"});
        CHECK_EQ(result.terminals[1].index, uint32_t{1});
        // 1 + 2, not 2: the translates accumulate.
        CHECK_NEAR(result.terminals[1].scope.origin.x, 3.0, 1e-12);
        CHECK_NEAR(geometry_volume(result.terminals[1].geometry), 2.0, 1e-12);

        // The parentage a caller rebuilds the tree from.
        CHECK_EQ(result.terminals[0].parent, result.terminals[1].parent);
    }
}

TEST(Interpreter, a_discarded_shape_produces_nothing_and_its_siblings_still_do) {
    const GenerationResult result = run_square(
        "@start\n"
        "rule Main { A(); B(); }\n"
        "rule A { discard; }\n"
        "rule B { extrude(1.0); }\n");

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.terminals.size(), size_t{1});
    if (!result.terminals.empty()) {
        CHECK_EQ(result.terminals[0].rule, std::string{"B"});
    }
    // Both children were still created; only one produced output.
    CHECK_EQ(result.stats.shapes_created, uint32_t{3});
}

TEST(Interpreter, an_if_runs_its_then_arm_and_its_else_arm) {
    // Both arms, through one rule called twice, so a build in which `else` never
    // ran could not pass by taking `then` both times. Until this existed the only
    // `if` in the suite had no `else` at all, and deleting else-branch execution
    // from State::exec() left every test green -- a rule language whose `else`
    // silently does nothing.
    const GenerationResult result = run_square(
        "@start\n"
        "rule Main { A(1.0); A(9.0); }\n"
        "rule A(n : float) {\n"
        "    if (n < 5.0) { print(\"low\"); } else { print(\"high\"); }\n"
        "}\n");

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.log.size(), size_t{2});
    if (result.log.size() == 2) {
        CHECK_EQ(result.log[0], std::string{"low"});
        CHECK_EQ(result.log[1], std::string{"high"});
    }

    // An `if` with no else and a false condition falls through to the rest of the
    // body rather than discarding the shape.
    const GenerationResult bare = run_square(
        "@start\n"
        "rule Main { if (false) { print(\"taken\"); } extrude(2.0); }\n");
    CHECK_TRUE(bare.ok());
    CHECK_TRUE(bare.log.empty());
    CHECK_EQ(bare.terminals.size(), size_t{1});
    if (!bare.terminals.empty()) {
        CHECK_NEAR(geometry_volume(bare.terminals[0].geometry), 2.0, 1e-12);
    }

    // An `if` arm can make a child, and the child is the terminal.
    const GenerationResult branching = run_square(
        "@start\n"
        "rule Main { if (shape.sx > 0.5) { A(); } else { B(); } }\n"
        "rule A { extrude(1.0); }\n"
        "rule B { extrude(5.0); }\n");
    CHECK_TRUE(branching.ok());
    CHECK_EQ(branching.terminals.size(), size_t{1});
    if (!branching.terminals.empty()) {
        CHECK_EQ(branching.terminals[0].rule, std::string{"A"});
    }
}

TEST(Interpreter, every_operator_the_language_has_is_evaluated_here) {
    // A regression net rather than a bug hunt: each of these gives the right
    // answer today, and none of them was evaluated anywhere in this suite. An
    // operator that quietly became its opposite -- '<' for '>', '&&' for '||' --
    // is a wrong building with no diagnostic anywhere.
    const GenerationResult result = run_square(
        "@start\n"
        "rule Main {\n"
        "    print(2.0 + 3.0, 7.0 - 2.0, 3.0 * 4.0, 8.0 / 2.0, -3.0);\n"
        "    print(2.0 < 3.0, 3.0 < 2.0, 2.0 > 1.0, 1.0 > 2.0);\n"
        "    print(2.0 >= 2.0, 2.0 >= 3.0, 1.0 <= 0.0, 1.0 <= 1.0);\n"
        "    print(1.0 == 1.0, 1.0 == 2.0, 1.0 != 1.0, 1.0 != 2.0);\n"
        "    print(true && true, true && false, true || false, false || false, !true, !false);\n"
        "    print(true ? 1.0 : 2.0, false ? 1.0 : 2.0);\n"
        "    print(\"a\" + \"b\", \"a\" == \"a\", \"a\" == \"b\", \"a\" != \"b\");\n"
        "}\n");

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.log.size(), size_t{7});
    if (result.log.size() == 7) {
        CHECK_EQ(result.log[0], std::string{"5 5 12 4 -3"});
        CHECK_EQ(result.log[1], std::string{"true false true false"});
        CHECK_EQ(result.log[2], std::string{"true false false true"});
        CHECK_EQ(result.log[3], std::string{"true false false true"});
        CHECK_EQ(result.log[4], std::string{"true false true false false true"});
        CHECK_EQ(result.log[5], std::string{"1 2"});
        CHECK_EQ(result.log[6], std::string{"ab true false true"});
    }

    // && and || short-circuit, which is what makes a guard such as
    // `n != 0 && 10 / n < 2` safe to write. Evaluating both sides would divide.
    const GenerationResult guarded = run_square(
        "@start\n"
        "rule Main {\n"
        "    let n = 0.0;\n"
        "    print(n != 0.0 && 10.0 / n < 2.0);\n"
        "    print(n == 0.0 || 10.0 / n < 2.0);\n"
        "    print(n == 0.0 ? 0.0 : 10.0 / n);\n"
        "}\n");
    CHECK_TRUE(guarded.ok());
    CHECK_EQ(guarded.log.size(), size_t{3});
    if (guarded.log.size() == 3) {
        CHECK_EQ(guarded.log[0], std::string{"false"});
        CHECK_EQ(guarded.log[1], std::string{"true"});
        CHECK_EQ(guarded.log[2], std::string{"0"});
    }
}

TEST(Interpreter, a_scope_block_restores_the_axes_and_the_pivot) {
    const GenerationResult result = run_square(
        "@start\n"
        "rule Main {\n"
        "    scope {\n"
        "        rotate_scope(0, 90, 0);\n"
        "        set_pivot(\"center\");\n"
        "    }\n"
        "}\n");

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.terminals.size(), size_t{1});
    if (!result.terminals.empty()) {
        const Shape& shape = result.terminals[0];
        check_vec3(shape.scope.axes[0], glm::dvec3{1.0, 0.0, 0.0}, 1e-12);
        check_vec3(shape.scope.axes[2], glm::dvec3{0.0, 0.0, 1.0}, 1e-12);
        check_vec3(shape.scope.pivot, glm::dvec3{0.0, 0.0, 0.0}, 0.0);
    }

    // The control: without the block the same two operations stick, so the test
    // above is asserting the block and not the operations being no-ops.
    const GenerationResult leaked = run_square(
        "@start\n"
        "rule Main {\n"
        "    rotate_scope(0, 90, 0);\n"
        "    set_pivot(\"center\");\n"
        "}\n");
    CHECK_TRUE(leaked.ok());
    CHECK_EQ(leaked.terminals.size(), size_t{1});
    if (!leaked.terminals.empty()) {
        check_vec3(leaked.terminals[0].scope.axes[0], glm::dvec3{0.0, 0.0, -1.0}, 1e-12);
        check_vec3(leaked.terminals[0].scope.pivot, glm::dvec3{0.5, 0.5, 0.5}, 0.0);
    }
}

TEST(Interpreter, setback_emits_its_border_as_a_terminal_of_its_own) {
    const GenerationResult result = run_square(
        "@start\n"
        "rule Main {\n"
        "    setback(0.1);\n"
        "    extrude(2.0);\n"
        "}\n");

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.terminals.size(), size_t{2});

    double border_area = 0.0;
    double building_volume = 0.0;
    int borders = 0;
    int buildings = 0;
    glm::dvec3 border_min{0.0};
    glm::dvec3 border_max{0.0};
    glm::dvec3 building_min{0.0};
    glm::dvec3 building_max{0.0};
    for (const Shape& shape : result.terminals) {
        if (shape.rule == "setback.border") {
            ++borders;
            border_area = geometry_area(shape.geometry);
            world_bounds(shape, border_min, border_max);
        } else {
            ++buildings;
            building_volume = geometry_volume(shape.geometry);
            world_bounds(shape, building_min, building_max);
        }
    }
    CHECK_EQ(borders, 1);
    CHECK_EQ(buildings, 1);

    // The pavement strip the rule set the building back from.
    CHECK_NEAR(border_area, 0.36, 1e-9);
    // 0.8 by 0.8 by 2.
    CHECK_NEAR(building_volume, 1.28, 1e-9);

    // WHERE the border is, and not only how big it is. op_setback captures the
    // shape's frame BEFORE setback_shape() refits it, and the area alone cannot
    // tell the two apart: reading the frame afterwards gives a border of exactly
    // this area displaced by the whole setback distance, sitting on top of the
    // building instead of around it. The border still fills the original lot.
    check_vec3(border_min, glm::dvec3{0.0, 0.0, 0.0}, 1e-9);
    check_vec3(border_max, glm::dvec3{1.0, 0.0, 1.0}, 1e-9);
    // And the building is the inset square, raised by the extrude.
    check_vec3(building_min, glm::dvec3{0.1, 0.0, 0.1}, 1e-9);
    check_vec3(building_max, glm::dvec3{0.9, 2.0, 0.9}, 1e-9);
}

TEST(Interpreter, print_writes_every_argument_to_the_log_in_evaluation_order) {
    const GenerationResult result = run_square(
        "@start\n"
        "rule Main { A(); B(); }\n"
        "rule A { print(\"a\", 1.0, true); }\n"
        "rule B { print(\"b\", shape.sx); }\n");

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.log.size(), size_t{2});
    if (result.log.size() == 2) {
        CHECK_EQ(result.log[0], std::string{"a 1 true"});
        CHECK_EQ(result.log[1], std::string{"b 1"});
    }
}

TEST(Interpreter, build_mesh_triangulates_every_terminal) {
    const GenerationResult result = run_square(
        "@start\n"
        "rule Main { extrude(3.0); }\n");
    CHECK_TRUE(result.ok());

    const Mesh mesh = result.build_mesh();
    // Vertices are duplicated per face for flat shading: six quads, four vertices
    // and two triangles each.
    CHECK_EQ(mesh.vertices.size(), size_t{24});
    CHECK_EQ(mesh.indices.size(), size_t{36});
    // Every face uses the default material, which renderer/mesh.hpp spells as one
    // implicit whole-mesh range rather than an explicit submesh.
    CHECK_TRUE(mesh.submeshes.empty());
}

TEST(Interpreter, build_mesh_writes_world_space_normals_and_real_tangents) {
    // The scope is turned 45 degrees about y AFTER the extrude, so the scope's
    // axes and the world axes no longer agree and the two possible answers are
    // far apart. A builder that left each normal in scope-local space would emit
    // normals at 45 degrees to every world axis, which lights a wall as though
    // it faced a corner; the vertex and index counts would be identical.
    const GenerationResult result = run_square(
        "@start\n"
        "rule Main { extrude(3.0); rotate_scope(0, 45, 0); }\n");
    CHECK_TRUE(result.ok());

    const Mesh mesh = result.build_mesh();
    CHECK_EQ(mesh.vertices.size(), size_t{24});

    int axis_aligned = 0;
    int unit_tangents = 0;
    int perpendicular_tangents = 0;
    for (const stratum::Vertex& vertex : mesh.vertices) {
        const glm::dvec3 normal{vertex.normal.x, vertex.normal.y, vertex.normal.z};
        CHECK_NEAR(glm::length(normal), 1.0, 1e-6);
        // A box's faces face the world axes, whatever the scope was turned to.
        const double largest =
            std::max(std::fabs(normal.x), std::max(std::fabs(normal.y), std::fabs(normal.z)));
        if (largest > 1.0 - 1e-6) {
            ++axis_aligned;
        }
        // compute_tangents() left out entirely leaves the default (1,0,0,1),
        // which is not perpendicular to the two faces whose normal is world x.
        const glm::dvec3 tangent{vertex.tangent.x, vertex.tangent.y, vertex.tangent.z};
        if (std::fabs(glm::length(tangent) - 1.0) <= 1e-5) {
            ++unit_tangents;
        }
        if (std::fabs(glm::dot(tangent, normal)) <= 1e-5) {
            ++perpendicular_tangents;
        }
    }
    CHECK_EQ(axis_aligned, 24);
    CHECK_EQ(unit_tangents, 24);
    CHECK_EQ(perpendicular_tangents, 24);

    // Counting "axis aligned" is not enough, and this is where the test used to
    // stop. Forcing every one of the 24 normals to (0,1,0) scores 24/24 on all
    // three counters above -- it is unit, it is axis aligned, and (1,0,0) is
    // perpendicular to it. So pin WHICH axes, with their multiplicity: a box has
    // six faces, one per world axis direction, four vertices each.
    int per_direction[6] = {0, 0, 0, 0, 0, 0};
    for (const stratum::Vertex& vertex : mesh.vertices) {
        const glm::dvec3 n{vertex.normal.x, vertex.normal.y, vertex.normal.z};
        if (n.x > 0.5) ++per_direction[0];
        if (n.x < -0.5) ++per_direction[1];
        if (n.y > 0.5) ++per_direction[2];
        if (n.y < -0.5) ++per_direction[3];
        if (n.z > 0.5) ++per_direction[4];
        if (n.z < -0.5) ++per_direction[5];
    }
    for (const int count : per_direction) {
        CHECK_EQ(count, 4);
    }

    // And that each shaded normal agrees with the WINDING of a triangle that
    // uses it. A normal can name the right axis and point the wrong way along
    // it -- which is what happens to every triangle of a mirrored scope, where
    // the geometry is emitted inside out and the renderer culls the front faces.
    int inside_out = 0;
    for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
        const stratum::Vertex& a = mesh.vertices[mesh.indices[i]];
        const stratum::Vertex& b = mesh.vertices[mesh.indices[i + 1]];
        const stratum::Vertex& c = mesh.vertices[mesh.indices[i + 2]];
        const glm::dvec3 pa{a.position.x, a.position.y, a.position.z};
        const glm::dvec3 pb{b.position.x, b.position.y, b.position.z};
        const glm::dvec3 pc{c.position.x, c.position.y, c.position.z};
        const glm::dvec3 wound = glm::cross(pb - pa, pc - pa);
        const glm::dvec3 shaded{a.normal.x, a.normal.y, a.normal.z};
        if (glm::dot(glm::normalize(wound), shaded) < 0.0) {
            ++inside_out;
        }
    }
    CHECK_EQ(inside_out, 0);
}

TEST(Interpreter, a_mirrored_scope_still_emits_outward_facing_triangles) {
    // mirror_scope() makes the frame left-handed on purpose. A reflection
    // reverses orientation, so without a compensating flip every triangle of
    // the solid comes out wound against its own shaded normal: the renderer
    // culls the front faces, lights the back ones, and compute_tangents()
    // derives handedness from the wrong normal. A mirrored building renders as
    // a hole in the world.
    //
    // BOTH orders are checked, and they used to fail for opposite reasons --
    // after `extrude; mirror_scope` the winding was right and the normal
    // inverted; after `mirror_scope; extrude` the normal was right and the
    // winding inverted. A patch to only one of reframe() or
    // append_shape_to_mesh() fixes one order and cancels itself out on the
    // other, which is why this test runs both.
    const char* sources[] = {
        "@start\nrule Main { extrude(3.0); }\n",
        "@start\nrule Main { extrude(3.0); mirror_scope(\"x\"); }\n",
        "@start\nrule Main { mirror_scope(\"x\"); extrude(3.0); }\n",
    };

    for (const char* source : sources) {
        const GenerationResult result = run_rect(4.0, 2.0, source);
        CHECK_TRUE(result.ok());
        const Mesh mesh = result.build_mesh();
        CHECK_EQ(mesh.indices.size(), size_t{36});

        int inside_out = 0;
        for (size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
            const stratum::Vertex& a = mesh.vertices[mesh.indices[i]];
            const stratum::Vertex& b = mesh.vertices[mesh.indices[i + 1]];
            const stratum::Vertex& c = mesh.vertices[mesh.indices[i + 2]];
            const glm::dvec3 pa{a.position.x, a.position.y, a.position.z};
            const glm::dvec3 pb{b.position.x, b.position.y, b.position.z};
            const glm::dvec3 pc{c.position.x, c.position.y, c.position.z};
            const glm::dvec3 wound = glm::cross(pb - pa, pc - pa);
            if (glm::length(wound) < 1e-12) continue;
            const glm::dvec3 shaded{a.normal.x, a.normal.y, a.normal.z};
            if (glm::dot(glm::normalize(wound), shaded) < 0.0) {
                ++inside_out;
            }
        }
        CHECK_EQ(inside_out, 0);
    }
}

TEST(Interpreter, every_wall_of_a_box_gets_the_same_texture_orientation) {
    // UVs are a planar projection in metres, so a texture tiles at real-world
    // scale. That claim is only worth anything if the four walls agree on which
    // way is up: a facade texture has brick courses, and a wall whose texture is
    // rotated a quarter turn is the most visible defect a generated building
    // can have.
    //
    // The old basis seeded itself from the normal alone -- "the world axis the
    // normal is least aligned with" -- which is a DIFFERENT axis for a wall
    // facing x than for one facing z. Measured on this box it gave three
    // different orientations across four walls, and three of the four ran
    // negative.
    //
    // This also pins the UVs to the geometry at all. Replacing the projection
    // with any fixed function of the local position -- {x*7+3, z*7+3}, say --
    // used to pass the whole suite.
    const GenerationResult result = run_rect(4.0, 2.0, "@start\nrule Main { extrude(3.0); }\n");
    CHECK_TRUE(result.ok());

    const Mesh mesh = result.build_mesh();
    CHECK_EQ(mesh.vertices.size(), size_t{24});

    // For each of the four walls, v must run up the wall: the two vertices at
    // the top of the box must have a greater v than the two at the bottom, and
    // the difference must be the wall's height in metres.
    int walls_checked = 0;
    for (const glm::dvec3 facing : {glm::dvec3{1, 0, 0}, glm::dvec3{-1, 0, 0},
                                    glm::dvec3{0, 0, 1}, glm::dvec3{0, 0, -1}}) {
        double v_at_bottom = 0.0;
        double v_at_top = 0.0;
        double u_min = 1e9;
        double u_max = -1e9;
        int bottom = 0;
        int top = 0;
        for (const stratum::Vertex& vertex : mesh.vertices) {
            const glm::dvec3 n{vertex.normal.x, vertex.normal.y, vertex.normal.z};
            if (glm::dot(n, facing) < 0.9) continue;
            u_min = std::min(u_min, static_cast<double>(vertex.uv.x));
            u_max = std::max(u_max, static_cast<double>(vertex.uv.x));
            if (vertex.position.y < 0.001) { v_at_bottom += vertex.uv.y; ++bottom; }
            else { v_at_top += vertex.uv.y; ++top; }
        }
        CHECK_EQ(bottom, 2);
        CHECK_EQ(top, 2);
        if (bottom == 2 && top == 2) {
            // v increases upwards, by exactly the 3 m the box was extruded.
            CHECK_NEAR(v_at_top / 2.0 - v_at_bottom / 2.0, 3.0, 1e-5);
            // u spans the wall's width in metres: 4 m on the z-facing walls,
            // 2 m on the x-facing ones. Metres, not a normalised 0..1.
            const double expected_width = std::fabs(facing.x) > 0.5 ? 2.0 : 4.0;
            CHECK_NEAR(u_max - u_min, expected_width, 1e-5);
            ++walls_checked;
        }
    }
    CHECK_EQ(walls_checked, 4);
}

// ============================================================================
// The depth cap
// ============================================================================

TEST(Interpreter, the_depth_cap_stops_a_runaway_recursion_and_reports_it) {
    const std::string source =
        "@start\n"                     // line 1
        "rule Main { Tower(); }\n"     // line 2
        "rule Tower {\n"               // line 3
        "    translate(0, 1, 0);\n"    // line 4
        "    Tower();\n"               // line 5
        "}\n";                         // line 6

    GenerationOptions options;
    options.limits.max_depth = 4;
    const GenerationResult result = run_square(source, options);

    // An Error, not a warning: a generation that hit a cap produced the wrong
    // building and no caller may mistake it for a finished one.
    CHECK_FALSE(result.ok());
    CHECK_TRUE(result.stats.depth_limit_hit);
    CHECK_FALSE(result.stats.shape_limit_hit);
    CHECK_EQ(result.stats.max_depth_reached, uint32_t{4});
    CHECK_EQ(result.stats.shapes_created, uint32_t{6});

    // Reported once, at the recursive call, not once per level.
    CHECK_EQ(result.diagnostics.size(), size_t{1});
    if (!result.diagnostics.empty()) {
        const Diagnostic& diagnostic = result.diagnostics[0];
        CHECK_TRUE(diagnostic.severity == Severity::Error);
        CHECK_TRUE(says(diagnostic.message, "depth limit of 4"));
        CHECK_TRUE(says(diagnostic.message, "Tower"));
        CHECK_EQ(diagnostic.loc.line, uint32_t{5});
        CHECK_EQ(diagnostic.loc.column, uint32_t{5});
    }

    // Truncated, not silent: the shape at the cap is emitted as it stood, so the
    // author sees a short tower rather than an empty viewport.
    CHECK_EQ(result.terminals.size(), size_t{1});
    if (!result.terminals.empty()) {
        CHECK_EQ(result.terminals[0].depth, uint32_t{5});
        // Four translates ran on the way down, which is what "as it stood" means.
        CHECK_NEAR(result.terminals[0].scope.origin.y, 4.0, 1e-12);
    }
}

TEST(Interpreter, the_depth_cap_is_the_configured_depth_and_not_a_constant) {
    const std::string source =
        "@start\n"
        "rule Main { Tower(); }\n"
        "rule Tower {\n"
        "    translate(0, 1, 0);\n"
        "    Tower();\n"
        "}\n";

    GenerationOptions shallow;
    shallow.limits.max_depth = 2;
    const GenerationResult a = run_square(source, shallow);

    GenerationOptions deep;
    deep.limits.max_depth = 9;
    const GenerationResult b = run_square(source, deep);

    CHECK_TRUE(a.stats.depth_limit_hit);
    CHECK_TRUE(b.stats.depth_limit_hit);
    CHECK_EQ(a.stats.max_depth_reached, uint32_t{2});
    CHECK_EQ(b.stats.max_depth_reached, uint32_t{9});
    CHECK_EQ(a.terminals.size(), size_t{1});
    CHECK_EQ(b.terminals.size(), size_t{1});
    if (!a.terminals.empty() && !b.terminals.empty()) {
        CHECK_EQ(a.terminals[0].depth, uint32_t{3});
        CHECK_EQ(b.terminals[0].depth, uint32_t{10});
        CHECK_NEAR(a.terminals[0].scope.origin.y, 2.0, 1e-12);
        CHECK_NEAR(b.terminals[0].scope.origin.y, 9.0, 1e-12);
    }
}

TEST(Interpreter, a_recursion_that_ends_on_its_own_does_not_touch_the_cap) {
    // The control for the two tests above: the cap must not fire for a rule that
    // stops by itself, or "the cap was hit" would carry no information.
    const GenerationResult result = run_square(
        "@start\n"
        "rule Main { Tower(0.0); }\n"
        "rule Tower(level : float) {\n"
        "    translate(0, 1, 0);\n"
        "    if (level < 3.0) { Tower(level + 1.0); }\n"
        "}\n");

    CHECK_TRUE(result.ok());
    CHECK_FALSE(result.stats.depth_limit_hit);
    CHECK_EQ(result.stats.max_depth_reached, uint32_t{4});
    CHECK_EQ(result.terminals.size(), size_t{1});
    if (!result.terminals.empty()) {
        CHECK_NEAR(result.terminals[0].scope.origin.y, 4.0, 1e-12);
    }
}

// ============================================================================
// The shape cap
// ============================================================================

TEST(Interpreter, the_shape_cap_truncates_wide_branching_and_reports_it) {
    // Six calls, a cap of four. The depth cap cannot catch this: nothing here is
    // more than one level deep.
    GenerationOptions options;
    options.limits.max_shapes = 4;
    const GenerationResult result = run_square(
        "@start\n"
        "rule Main { A(); A(); A(); A(); A(); A(); }\n"
        "rule A { extrude(1.0); }\n",
        options);

    CHECK_FALSE(result.ok());
    CHECK_TRUE(result.stats.shape_limit_hit);
    CHECK_FALSE(result.stats.depth_limit_hit);
    CHECK_EQ(result.stats.shapes_created, uint32_t{4});

    // The root plus three children, and no more.
    CHECK_EQ(result.terminals.size(), size_t{3});

    CHECK_EQ(result.diagnostics.size(), size_t{1});
    CHECK_TRUE(has_error_saying(result, "shape limit of 4"));
}

TEST(Interpreter, the_shape_cap_is_the_configured_count_and_not_a_constant) {
    const std::string source =
        "@start\n"
        "rule Main { A(); A(); A(); A(); A(); A(); }\n"
        "rule A { extrude(1.0); }\n";

    GenerationOptions tight;
    tight.limits.max_shapes = 3;
    GenerationOptions loose;
    loose.limits.max_shapes = 5;

    const GenerationResult a = run_square(source, tight);
    const GenerationResult b = run_square(source, loose);

    CHECK_EQ(a.terminals.size(), size_t{2});
    CHECK_EQ(b.terminals.size(), size_t{4});
    CHECK_EQ(a.stats.shapes_created, uint32_t{3});
    CHECK_EQ(b.stats.shapes_created, uint32_t{5});
}

TEST(Interpreter, a_parent_whose_children_the_cap_refused_still_emits_its_own_geometry) {
    // With room for the root alone, the child is refused. The parent then made no
    // child that exists, so it is a terminal and the author sees the massing it
    // got as far as -- which is the argument the depth cap already makes.
    GenerationOptions options;
    options.limits.max_shapes = 1;
    const GenerationResult result = run_square(
        "@start\n"
        "rule Main { extrude(3.0); A(); }\n"
        "rule A { taper(0.1); }\n",
        options);

    CHECK_FALSE(result.ok());
    CHECK_TRUE(result.stats.shape_limit_hit);
    CHECK_EQ(result.stats.shapes_created, uint32_t{1});
    CHECK_EQ(result.terminals.size(), size_t{1});
    if (!result.terminals.empty()) {
        CHECK_EQ(result.terminals[0].rule, std::string{"Main"});
        CHECK_NEAR(geometry_volume(result.terminals[0].geometry), 3.0, 1e-12);
    }
    CHECK_TRUE(has_error_saying(result, "truncated"));
}

// ============================================================================
// Determinism
// ============================================================================

namespace {

/// Eight shapes, each drawing from a three-way choose: 3^8 outcomes, so two runs
/// that agree by accident happen once in 6561 rather than once in four.
const char* const kStochasticRules =
    "@start\n"
    "rule Main { A(); A(); A(); A(); A(); A(); A(); A(); }\n"
    "rule A {\n"
    "    choose {\n"
    "        1: { extrude(1.0); }\n"
    "        1: { extrude(2.0); }\n"
    "        1: { extrude(3.0); }\n"
    "    }\n"
    "}\n";

}  // namespace

TEST(Interpreter, the_same_text_seed_and_input_produce_a_byte_identical_dump) {
    GenerationOptions options;
    options.seed = 0x5EED;

    const GenerationResult first = run_square(kStochasticRules, options);
    const GenerationResult second = run_square(kStochasticRules, options);

    CHECK_TRUE(first.ok());
    CHECK_EQ(first.terminals.size(), size_t{8});

    const std::string a = first.dump();
    const std::string b = second.dump();

    // Two empty strings compare equal, so the thing being compared is checked for
    // substance before it is compared.
    CHECK((a.size()) > size_t{1000});
    CHECK_EQ(a, b);
    CHECK_EQ(first.log.size(), second.log.size());

    // And the geometry itself, vertex for vertex, so that determinism does not
    // rest on what dump() happens to print.
    CHECK_TRUE(terminals_are_identical(first, second));
}

TEST(Interpreter, the_dump_moves_when_the_seed_moves) {
    // The teeth of the test above. Without this, an implementation whose dump was
    // a constant string would pass it.
    GenerationOptions a;
    a.seed = 1;
    GenerationOptions b;
    b.seed = 2;

    const GenerationResult first = run_square(kStochasticRules, a);
    const GenerationResult second = run_square(kStochasticRules, b);

    CHECK_EQ(first.terminals.size(), size_t{8});
    CHECK_EQ(second.terminals.size(), size_t{8});
    CHECK((first.dump().size()) > size_t{1000});
    CHECK_FALSE(first.dump() == second.dump());
    CHECK_FALSE(terminals_are_identical(first, second));
}

TEST(Interpreter, two_input_shapes_under_one_run_seed_diverge) {
    // What a caller generating one building per lot depends on: one seed for the
    // city, the lot's own key on the shape, and different buildings on each lot.
    Shape lot_a = shape_from_rect(1.0, 1.0);
    lot_a.seed_key = 11;
    Shape lot_b = shape_from_rect(1.0, 1.0);
    lot_b.seed_key = 12;

    GenerationOptions options;
    options.seed = 7;

    const GenerationResult a = run_on(kStochasticRules, lot_a, options);
    const GenerationResult b = run_on(kStochasticRules, lot_b, options);
    const GenerationResult a_again = run_on(kStochasticRules, lot_a, options);

    CHECK_EQ(a.terminals.size(), size_t{8});
    CHECK((a.dump().size()) > size_t{1000});
    CHECK_FALSE(a.dump() == b.dump());
    CHECK_FALSE(terminals_are_identical(a, b));
    // And the same lot is the same building on a re-run.
    CHECK_EQ(a.dump(), a_again.dump());
    CHECK_TRUE(terminals_are_identical(a, a_again));
}

// ============================================================================
// Determinism: anchors to a literal
// ============================================================================

TEST(Interpreter, the_dump_of_a_known_shape_is_anchored_to_a_literal) {
    // The three tests above compare two runs made in ONE process, which cannot
    // see anything that is constant within a process and varies between builds,
    // platforms or C libraries. This one compares against text pasted into the
    // source, so the promise interpreter.hpp makes -- "byte-identical output on
    // any platform" -- is what is actually asserted.
    //
    // Every number here comes out of an exact path: the extrusion is +, - and *,
    // the axes never reach libm, and format_number() prints ten significant
    // digits. A platform on which this string differs has a real determinism
    // fault, not a rounding difference to be widened away.
    const GenerationResult result = run_square(
        "@start\n"
        "rule Main { extrude(2.0); }\n");
    CHECK_TRUE(result.ok());

    const std::string expected =
        "terminals 1\n"
        "shape Main depth=0 index=0\n"
        "  origin=(0 0 0) size=(1 2 1) pivot=(0 0 0)\n"
        "  axes x=(1 0 0) y=(0 1 0) z=(0 0 1)\n"
        "  face 0 normal=(0 -1 0) area=1\n"
        "    (0 0 0)\n"
        "    (1 0 0)\n"
        "    (1 0 1)\n"
        "    (0 0 1)\n"
        "  face 1 normal=(0 1 0) area=1\n"
        "    (0 2 1)\n"
        "    (1 2 1)\n"
        "    (1 2 0)\n"
        "    (0 2 0)\n"
        "  face 2 normal=(0 0 1) area=2\n"
        "    (0 0 1)\n"
        "    (1 0 1)\n"
        "    (1 2 1)\n"
        "    (0 2 1)\n"
        "  face 3 normal=(1 0 0) area=2\n"
        "    (1 0 1)\n"
        "    (1 0 0)\n"
        "    (1 2 0)\n"
        "    (1 2 1)\n"
        "  face 4 normal=(0 0 -1) area=2\n"
        "    (1 0 0)\n"
        "    (0 0 0)\n"
        "    (0 2 0)\n"
        "    (1 2 0)\n"
        "  face 5 normal=(-1 0 0) area=2\n"
        "    (0 0 0)\n"
        "    (0 0 1)\n"
        "    (0 2 1)\n"
        "    (0 2 0)\n";

    CHECK_EQ(result.dump(), expected);
}

TEST(Interpreter, the_choose_sequence_is_anchored_to_a_literal_and_not_only_to_another_run) {
    // The seeding chain, pinned end to end: the run seed, seed_mix2(), the
    // per-child address, the choose salt and seed_unit()'s 53-bit scaling all
    // decide these eight letters. A perturbation of any of them that happens to
    // be constant within one process -- a term mixed in from sizeof(void*), say
    // -- passes every run-against-run comparison in this file and fails here.
    //
    // Letters rather than geometry, so that the anchor is short enough to read
    // and a failure says which shape drew differently.
    GenerationOptions options;
    options.seed = 0x5EED;

    const GenerationResult result = run_square(
        "@start\n"
        "rule Main { A(); A(); A(); A(); A(); A(); A(); A(); }\n"
        "rule A {\n"
        "    choose {\n"
        "        1: { print(\"a\"); }\n"
        "        1: { print(\"b\"); }\n"
        "        1: { print(\"c\"); }\n"
        "    }\n"
        "}\n",
        options);

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.log.size(), size_t{8});

    std::string drawn;
    for (const std::string& line : result.log) {
        drawn += line;
    }
    // If the rule text below is reformatted this literal changes, and that is
    // correct: the draw is salted with the choose STATEMENT's byte offset so that
    // two choose statements in one rule cannot secretly agree. Re-derive it, do
    // not widen it.
    CHECK_EQ(drawn, std::string{"cbbaaabc"});

    // The control: a different seed draws a different sequence, so the anchor
    // above is not passing because every arm is the first one.
    GenerationOptions other = options;
    other.seed = 0x5EEE;
    const GenerationResult moved = run_square(
        "@start\n"
        "rule Main { A(); A(); A(); A(); A(); A(); A(); A(); }\n"
        "rule A {\n"
        "    choose {\n"
        "        1: { print(\"a\"); }\n"
        "        1: { print(\"b\"); }\n"
        "        1: { print(\"c\"); }\n"
        "    }\n"
        "}\n",
        other);
    std::string moved_draw;
    for (const std::string& line : moved.log) {
        moved_draw += line;
    }
    CHECK_EQ(moved.log.size(), size_t{8});
    CHECK_FALSE(moved_draw == drawn);
}

TEST(Interpreter, the_dump_records_the_vertices_and_not_only_the_scope_box) {
    // The other half of the teeth. A dump that printed the scope and the face
    // count but not the vertices would still differ for two shapes of different
    // SIZE, so the determinism comparison above would pass for an interpreter
    // whose vertex order wandered between runs.
    //
    // These two squares have the same bounding box, the same face count, the same
    // normal and the same area. The only thing that differs is one collinear
    // vertex in the middle of an edge.
    const std::vector<glm::dvec2> four = {{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};
    const std::vector<glm::dvec2> five = {
        {0.0, 0.0}, {0.5, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {0.0, 1.0}};

    const Shape square_four = stratum::procgen::rules::shape_from_polygon(four, 0.0);
    const Shape square_five = stratum::procgen::rules::shape_from_polygon(five, 0.0);

    // The premise: everything a scope-only dump would print is the same.
    check_vec3(square_four.scope.size, square_five.scope.size, 0.0);
    CHECK_EQ(square_four.geometry.faces.size(), square_five.geometry.faces.size());
    CHECK_NEAR(geometry_area(square_four.geometry), geometry_area(square_five.geometry), 1e-12);
    CHECK_EQ(square_four.geometry.positions.size(), size_t{4});
    CHECK_EQ(square_five.geometry.positions.size(), size_t{5});

    const std::string source = "@start\nrule Main { }\n";
    const std::string dump_four = run_on(source, square_four).dump();
    const std::string dump_five = run_on(source, square_five).dump();

    CHECK((dump_four.size()) > size_t{100});
    CHECK_FALSE(dump_four == dump_five);

    // And the size does reach the dump too, which is what the scope line is for.
    const std::string taller = run_square("@start\nrule Main { extrude(3.0); }\n").dump();
    const std::string shorter = run_square("@start\nrule Main { extrude(3.0001); }\n").dump();
    CHECK_FALSE(taller == shorter);
}

TEST(Interpreter, siblings_under_one_parent_draw_independently) {
    // A shape's random state comes from its ADDRESS in the tree -- the parent's
    // key mixed with the sibling index -- so eight siblings running the same rule
    // must not all draw the same arm. A child key that ignored the index would
    // make every sibling identical, which is a plausible-looking building and a
    // completely broken one.
    const GenerationResult result = run_square(kStochasticRules);
    CHECK_EQ(result.terminals.size(), size_t{8});

    bool any_two_differ = false;
    for (size_t i = 1; i < result.terminals.size(); ++i) {
        if (std::fabs(result.terminals[i].scope.size.y -
                      result.terminals[0].scope.size.y) > 1e-9) {
            any_two_differ = true;
        }
    }
    CHECK_TRUE(any_two_differ);
}

TEST(Interpreter, a_draw_elsewhere_in_the_file_does_not_move_this_shapes_draw) {
    // The property a shared random stream does not have, and the reason this
    // design has none. `Right` is declared ABOVE `Left`, so its choose statement
    // sits at the same source offset in both files; only `Left`, which runs
    // FIRST, differs. With one advancing stream, Left's extra draw would shift
    // Right's and a bug report would stop reproducing the moment anyone added an
    // alternative anywhere earlier in the evaluation.
    const std::string without =
        "@start\n"
        "rule Main { Left(); Right(); }\n"
        "rule Right { choose { 1: { print(\"r1\"); } 1: { print(\"r2\"); } 1: { print(\"r3\"); } } }\n"
        "rule Left { }\n";
    const std::string with =
        "@start\n"
        "rule Main { Left(); Right(); }\n"
        "rule Right { choose { 1: { print(\"r1\"); } 1: { print(\"r2\"); } 1: { print(\"r3\"); } } }\n"
        "rule Left { choose { 1: { print(\"l1\"); } 1: { print(\"l2\"); } 1: { print(\"l3\"); } } }\n";

    const GenerationResult a = run_square(without);
    const GenerationResult b = run_square(with);

    CHECK_TRUE(a.ok());
    CHECK_TRUE(b.ok());
    // The premise: Left really did draw in the second file and not in the first.
    CHECK_EQ(a.log.size(), size_t{1});
    CHECK_EQ(b.log.size(), size_t{2});

    if (!a.log.empty() && !b.log.empty()) {
        CHECK_EQ(a.log.back(), b.log.back());
    }
}

TEST(Interpreter, two_choose_statements_in_one_rule_are_independent) {
    // Salted by the statement's source offset as well as the shape's address. With
    // only the shape's address, both draws would agree for every shape and a window
    // style and a door style would be secretly the same coin.
    const GenerationResult result = run_square(
        "@start\n"
        "rule Main { A(); A(); A(); A(); A(); A(); A(); A(); }\n"
        "rule A {\n"
        "    choose { 1: { print(\"1a\"); } 1: { print(\"1b\"); } }\n"
        "    choose { 1: { print(\"2a\"); } 1: { print(\"2b\"); } }\n"
        "}\n");

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.log.size(), size_t{16});

    bool ever_disagreed = false;
    for (size_t i = 0; i + 1 < result.log.size(); i += 2) {
        // "1a" pairs with "2a" when the two draws agree.
        if (result.log[i].substr(1) != result.log[i + 1].substr(1)) {
            ever_disagreed = true;
        }
    }
    CHECK_TRUE(ever_disagreed);
}

// ============================================================================
// Runtime diagnostics
// ============================================================================

TEST(Interpreter, asking_for_a_face_that_does_not_exist_reports_the_source_location) {
    const std::string source =
        "@start\n"                                  // line 1, offsets 0..6
        "rule Main {\n"                             // line 2, offsets 7..18
        "    extrude(1.0);\n"                       // line 3, offsets 19..36
        "    print(geometry.face_area(7));\n"       // line 4, offsets 37..
        "}\n";                                      // line 5

    const GenerationResult result = run_square(source);

    // A user error, not a crash and not a silent zero.
    CHECK_FALSE(result.ok());
    CHECK_EQ(result.diagnostics.size(), size_t{1});
    if (result.diagnostics.empty()) {
        return;
    }
    const Diagnostic& diagnostic = result.diagnostics[0];

    CHECK_TRUE(diagnostic.severity == Severity::Error);
    // Both numbers, so the author does not have to go and count the faces.
    CHECK_TRUE(says(diagnostic.message, "face 7"));
    CHECK_TRUE(says(diagnostic.message, "6 faces"));

    // The AST location of the call that asked. This is the assertion that a
    // diagnostic with no position cannot pass.
    CHECK_EQ(diagnostic.loc.line, uint32_t{4});
    CHECK_EQ(diagnostic.loc.column, uint32_t{11});
    CHECK_EQ(diagnostic.loc.offset, uint32_t{47});
    CHECK_EQ(diagnostic.length, uint32_t{1});

    // And it renders through the same path a parse error does, caret and all.
    const std::string rendered = render_diagnostic(diagnostic, source, "test.srl");
    CHECK_TRUE(says(rendered, "test.srl:4:11: error:"));
    CHECK_TRUE(says(rendered, "geometry.face_area(7)"));
    CHECK_TRUE(says(rendered, "^"));

    // The shape that faulted produced nothing.
    CHECK_TRUE(result.terminals.empty());
}

TEST(Interpreter, a_failing_shape_does_not_cost_its_siblings) {
    const GenerationResult result = run_square(
        "@start\n"
        "rule Main { Bad(); Good(); }\n"
        "rule Bad { extrude(0.0); }\n"
        "rule Good { extrude(2.0); }\n");

    CHECK_FALSE(result.ok());
    CHECK_TRUE(has_error_saying(result, "extrusion distance is zero"));
    // One malformed lot must not cost the other four thousand.
    CHECK_EQ(result.terminals.size(), size_t{1});
    if (!result.terminals.empty()) {
        CHECK_EQ(result.terminals[0].rule, std::string{"Good"});
        CHECK_NEAR(geometry_volume(result.terminals[0].geometry), 2.0, 1e-12);
    }
}

TEST(Interpreter, an_operation_with_no_handler_reports_once_per_site) {
    // `center` is a catalogue row in ast.hpp that D2 does not implement. It must
    // say so at the line that called it, once, however many shapes reach it.
    const GenerationResult result = run_square(
        "@start\n"
        "rule Main { A(); A(); A(); }\n"
        "rule A { center(\"x\"); extrude(1.0); }\n");

    CHECK_FALSE(result.ok());
    CHECK_EQ(result.diagnostics.size(), size_t{1});
    CHECK_TRUE(has_error_saying(result, "'center' is not implemented in this build"));
    // Evaluation carried on, so the rest of the rule still produced its geometry.
    CHECK_EQ(result.terminals.size(), size_t{3});
}

TEST(Interpreter, split_and_select_run_rather_than_reporting_a_gap) {
    // This test used to pin the gap: `split` and `select` were variant
    // alternatives in ast.hpp with no case in State::exec(), and what was asserted
    // was that the interpreter SAID so rather than doing nothing quietly. Its own
    // comment said the expectation would flip when the cases were written. D2.5
    // wrote them, so it has flipped, and what is asserted now is that both
    // statements reach their operations and make children.
    //
    // The wiring itself -- frames, seeds, caps, the slab cut and the component
    // decomposition -- belongs to test_rule_statements.cpp. What is here is only
    // that this SWITCH dispatches them, which is this suite's subject.
    const GenerationResult split = run_square(
        "@start\n"
        "rule Main {\n"
        "    split(x) { 1.0 : { A(); } }\n"
        "    extrude(1.0);\n"
        "}\n"
        "rule A { extrude(1.0); }\n");
    CHECK_TRUE(split.ok());
    CHECK_FALSE(has_error_saying(split, "'split' is not implemented in this build"));
    // One slab, whose body called A, which extruded it. Main itself made a child
    // and is therefore not a terminal, so the one output is A's.
    CHECK_EQ(split.terminals.size(), size_t{1});
    if (!split.terminals.empty()) {
        CHECK_EQ(split.terminals[0].rule, std::string{"A"});
        CHECK_NEAR(geometry_volume(split.terminals[0].geometry), 1.0, 1e-12);
    }

    const GenerationResult select = run_square(
        "@start\n"
        "rule Main {\n"
        "    extrude(1.0);\n"
        "    select face { top : { A(); } }\n"
        "}\n"
        "rule A { taper(0.1); }\n");
    CHECK_TRUE(select.ok());
    CHECK_FALSE(has_error_saying(select, "'select' is not implemented in this build"));
    // The one top face of the cube became a shape, and A tapered it. The five
    // faces no arm claimed made no children at all.
    CHECK_EQ(select.terminals.size(), size_t{1});
    if (!select.terminals.empty()) {
        CHECK_EQ(select.terminals[0].rule, std::string{"A"});
    }
}

TEST(Interpreter, an_operation_argument_of_the_wrong_kind_is_reported_at_its_call) {
    const GenerationResult result = run_square(
        "@start\n"
        "rule Main { extrude(\"q\", 2.0); }\n");

    CHECK_FALSE(result.ok());
    CHECK_TRUE(has_error_saying(result, "does not know the axis 'q'"));
    // The message lists what it does know, rather than leaving the author guessing.
    CHECK_TRUE(has_error_saying(result, "x, y, z and normal"));
    CHECK_TRUE(result.terminals.empty());
    if (!result.diagnostics.empty()) {
        CHECK_EQ(result.diagnostics[0].loc.line, uint32_t{2});
    }
}

TEST(Interpreter, arithmetic_faults_are_reported_with_a_location_and_not_an_infinity) {
    const GenerationResult divide = run_square(
        "@start\n"
        "rule Main { let a = 1.0 / 0.0; print(a); }\n");
    CHECK_FALSE(divide.ok());
    CHECK_TRUE(has_error_saying(divide, "division by zero"));
    CHECK_TRUE(divide.log.empty());

    const GenerationResult root = run_square(
        "@start\n"
        "rule Main { print(sqrt(-1.0)); }\n");
    CHECK_FALSE(root.ok());
    CHECK_TRUE(has_error_saying(root, "sqrt"));
    CHECK_TRUE(root.log.empty());

    const GenerationResult index = run_square(
        "@start\n"
        "rule Main { let a = [1.0, 2.0]; print(a[5]); }\n");
    CHECK_FALSE(index.ok());
    CHECK_TRUE(has_error_saying(index, "outside the array of 2 elements"));
}

TEST(Interpreter, a_file_with_no_start_rule_reports_instead_of_picking_one) {
    const GenerationResult result = run_square("rule Main { extrude(1.0); }\n");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(has_error_saying(result, "no @start rule"));
    CHECK_TRUE(result.terminals.empty());
    CHECK_EQ(result.stats.shapes_created, uint32_t{0});
}

TEST(Interpreter, a_constant_defined_in_terms_of_itself_is_named_rather_than_recursed) {
    const GenerationResult result = run_square(
        "const a : float = a + 1.0\n"
        "@start\n"
        "rule Main { print(a); }\n");

    CHECK_FALSE(result.ok());
    CHECK_TRUE(has_error_saying(result, "defined in terms of itself"));
    CHECK_TRUE(result.terminals.empty());
}

TEST(Interpreter, a_bad_file_level_value_costs_only_the_shapes_that_read_it) {
    // One malformed declaration must not cost the generation, for the same reason
    // one malformed lot must not cost the other four thousand. `junk` divides by
    // zero and nothing reads it, so the start rule still produces its building --
    // with a diagnostic against the line that wrote the constant.
    const GenerationResult unread = run_square(
        "const junk : float = 1.0 / 0.0\n"
        "@start\n"
        "rule Main { extrude(3.0); }\n");

    CHECK_FALSE(unread.ok());
    CHECK_TRUE(has_error_saying(unread, "division by zero"));
    CHECK_EQ(unread.terminals.size(), size_t{1});
    if (!unread.terminals.empty()) {
        CHECK_NEAR(geometry_volume(unread.terminals[0].geometry), 3.0, 1e-12);
    }
    if (!unread.diagnostics.empty()) {
        CHECK_EQ(unread.diagnostics[0].loc.line, uint32_t{1});
    }

    // A rule that DOES read it loses its own shape and nothing else. The message
    // at the reading line names the constant, because repeating "division by
    // zero" there would point at the wrong line.
    const GenerationResult read = run_square(
        "const junk : float = 1.0 / 0.0\n"
        "@start\n"
        "rule Main { A(); B(); }\n"
        "rule A { print(junk); }\n"
        "rule B { extrude(2.0); }\n");

    CHECK_FALSE(read.ok());
    CHECK_TRUE(has_error_saying(read, "division by zero"));
    CHECK_TRUE(has_error_saying(read, "'junk' could not be worked out"));
    CHECK_TRUE(read.log.empty());
    CHECK_EQ(read.terminals.size(), size_t{1});
    if (!read.terminals.empty()) {
        CHECK_EQ(read.terminals[0].rule, std::string{"B"});
    }

    // And a good constant next to a bad one is still usable.
    const GenerationResult neighbour = run_square(
        "const junk : float = 1.0 / 0.0\n"
        "const fine : float = 4.0\n"
        "@start\n"
        "rule Main { extrude(fine); }\n");
    CHECK_EQ(neighbour.terminals.size(), size_t{1});
    if (!neighbour.terminals.empty()) {
        CHECK_NEAR(geometry_volume(neighbour.terminals[0].geometry), 4.0, 1e-12);
    }
}

TEST(Interpreter, several_bad_file_level_values_are_all_reported_in_source_order) {
    // Each declaration is resolved on its own, so the author sees every bad
    // default at once. With one try around the whole pre-pass the first failure
    // ended it, and the second and third were never evaluated -- which made the
    // "in declaration order" promise a promise about a list of length one.
    const GenerationResult result = run_square(
        "const a : float = 1.0 / 0.0\n"
        "const b : float = sqrt(-1.0)\n"
        "const c : float = 2.0 / 0.0\n"
        "@start\n"
        "rule Main { extrude(1.0); }\n");

    CHECK_FALSE(result.ok());
    CHECK_EQ(result.diagnostics.size(), size_t{3});
    if (result.diagnostics.size() == 3) {
        CHECK_EQ(result.diagnostics[0].loc.line, uint32_t{1});
        CHECK_EQ(result.diagnostics[1].loc.line, uint32_t{2});
        CHECK_EQ(result.diagnostics[2].loc.line, uint32_t{3});
        CHECK_TRUE(says(result.diagnostics[0].message, "division by zero"));
        CHECK_TRUE(says(result.diagnostics[1].message, "sqrt"));
        CHECK_TRUE(says(result.diagnostics[2].message, "division by zero"));
    }
    // None of them was wanted, so the building is still there.
    CHECK_EQ(result.terminals.size(), size_t{1});
}

TEST(Interpreter, a_bad_file_level_value_does_not_leave_the_interpreter_in_global_mode) {
    // The unwind out of a failed declaration used to leave `in_global` set,
    // because run() returned immediately afterwards and nobody noticed. Now that
    // the generation continues, a leak would refuse `shape.sx` for every shape in
    // the file and the symptom would be a rule that reads its own scope failing
    // for a reason stated one line at a time about a constant nothing read.
    const GenerationResult result = run_square(
        "const junk : float = 1.0 / 0.0\n"
        "@start\n"
        "rule Main { print(shape.sx, geometry.area()); }\n");

    CHECK_TRUE(has_error_saying(result, "division by zero"));
    CHECK_FALSE(has_error_saying(result, "does not have"));
    CHECK_EQ(result.log.size(), size_t{1});
    if (!result.log.empty()) {
        CHECK_EQ(result.log[0], std::string{"1 1"});
    }
}

// ============================================================================
// Names and file-level values
// ============================================================================

TEST(Interpreter, a_rule_cannot_read_a_binding_that_belongs_to_its_caller) {
    // Lexical scoping. A rule whose meaning depended on who called it could not be
    // read on its own, and the fault would surface in the callee with nothing
    // pointing at the caller that supplied the name.
    const GenerationResult parameter = run_square(
        "@start\n"
        "rule Main { A(7.0); }\n"
        "rule A(w : float) { B(); }\n"
        "rule B { print(w); }\n");
    CHECK_FALSE(parameter.ok());
    CHECK_TRUE(has_error_saying(parameter, "unknown name 'w'"));

    const GenerationResult local = run_square(
        "@start\n"
        "rule Main { let secret = 42.0; A(); }\n"
        "rule A { print(secret); }\n");
    CHECK_FALSE(local.ok());
    CHECK_TRUE(has_error_saying(local, "unknown name 'secret'"));
}

TEST(Interpreter, a_rule_can_read_its_own_bindings_and_its_callers_arguments) {
    // The control for the test above: the scoping rule must not have been
    // implemented by refusing every name.
    const GenerationResult result = run_square(
        "@start\n"
        "rule Main { let k = 5.0; A(k + 1.0); }\n"
        "rule A(w : float, h : float = w * 2.0) {\n"
        "    let total = w + h;\n"
        "    print(w, h, total);\n"
        "}\n");

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.log.size(), size_t{1});
    if (!result.log.empty()) {
        // 6 from the caller's k + 1, 12 from the default that reads the earlier
        // parameter, 18 from the local that reads both.
        CHECK_EQ(result.log[0], std::string{"6 12 18"});
    }
}

TEST(Interpreter, an_inner_let_shadows_an_outer_one_and_the_outer_one_comes_back) {
    const GenerationResult result = run_square(
        "@start\n"
        "rule Main {\n"
        "    let k = 1.0;\n"
        "    { let k = 2.0; print(k); }\n"
        "    print(k);\n"
        "}\n");

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.log.size(), size_t{2});
    if (result.log.size() == 2) {
        CHECK_EQ(result.log[0], std::string{"2"});
        CHECK_EQ(result.log[1], std::string{"1"});
    }
}

TEST(Interpreter, a_geometry_query_in_a_constant_is_refused_rather_than_answering_zero) {
    // Zero is a plausible area, so a silent zero here reads as an empty lot and the
    // building comes out wrong with nothing pointing at the declaration.
    const GenerationResult result = run_square(
        "const base : float = geometry.area()\n"
        "@start\n"
        "rule Main { print(base); }\n");

    CHECK_FALSE(result.ok());
    CHECK_TRUE(has_error_saying(result, "geometry.area"));
    CHECK_TRUE(has_error_saying(result, "does not have"));
    CHECK_TRUE(result.log.empty());

    // The same query inside a rule, where there IS a shape, answers.
    const GenerationResult inside = run_square(
        "@start\n"
        "rule Main { print(geometry.area(), geometry.face_count()); }\n");
    CHECK_TRUE(inside.ok());
    CHECK_EQ(inside.log.size(), size_t{1});
    if (!inside.log.empty()) {
        CHECK_EQ(inside.log[0], std::string{"1 1"});
    }
}

TEST(Interpreter, shape_members_are_refused_in_a_file_level_value) {
    const GenerationResult result = run_square(
        "const base : float = shape.sx\n"
        "@start\n"
        "rule Main { print(base); }\n");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(has_error_saying(result, "shape.sx"));
}

TEST(Interpreter, an_attribute_override_is_used_and_a_mistyped_one_is_reported) {
    GenerationOptions good;
    good.attributes["height"] = Value::number(9.0);
    const GenerationResult used = run_square(
        "attr height : float = 4.0\n"
        "@start\n"
        "rule Main { print(height); }\n",
        good);
    CHECK_TRUE(used.ok());
    CHECK_EQ(used.log.size(), size_t{1});
    if (!used.log.empty()) {
        CHECK_EQ(used.log[0], std::string{"9"});
    }

    GenerationOptions wrong_type;
    wrong_type.attributes["height"] = Value::text("tall");
    const GenerationResult typed = run_square(
        "attr height : float = 4.0\n"
        "@start\n"
        "rule Main { print(height); }\n",
        wrong_type);
    CHECK_FALSE(typed.ok());
    CHECK_TRUE(has_error_saying(typed, "declared float"));
    // The declared default was used, so the generation still produced something.
    CHECK_EQ(typed.log.size(), size_t{1});
    if (!typed.log.empty()) {
        CHECK_EQ(typed.log[0], std::string{"4"});
    }

    // A name no attribute declares is a typo, and a slider that does nothing is
    // the symptom of ignoring it.
    GenerationOptions unknown;
    unknown.attributes["heigth"] = Value::number(9.0);
    const GenerationResult typo = run_square(
        "attr height : float = 4.0\n"
        "@start\n"
        "rule Main { print(height); }\n",
        unknown);
    CHECK_FALSE(typo.ok());
    CHECK_TRUE(has_error_saying(typo, "no attribute called 'heigth'"));
}

TEST(Interpreter, a_rule_reads_its_own_scope_through_the_shape_namespace) {
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main {\n"
        "    extrude(3.0);\n"
        "    print(shape.sx, shape.sy, shape.sz, shape.depth, shape.index);\n"
        "}\n",
        shape_from_rect(2.0, 4.0));

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.log.size(), size_t{1});
    if (!result.log.empty()) {
        CHECK_EQ(result.log[0], std::string{"2 3 4 0 0"});
    }

    const GenerationResult unknown = run_square(
        "@start\n"
        "rule Main { print(shape.sw); }\n");
    CHECK_FALSE(unknown.ok());
    CHECK_TRUE(has_error_saying(unknown, "has no member 'sw'"));
}

// ============================================================================
// Registries
// ============================================================================

TEST(Interpreter, the_standard_tables_hold_the_operations_and_functions_d2_implements) {
    const stratum::procgen::rules::OperationTable& operations =
        stratum::procgen::rules::standard_operations();
    const stratum::procgen::rules::FunctionTable& functions =
        stratum::procgen::rules::standard_functions();

    // Present, so a rule that calls them runs.
    CHECK_TRUE(operations.find("extrude") != nullptr);
    CHECK_TRUE(operations.find("taper") != nullptr);
    CHECK_TRUE(operations.find("setback") != nullptr);
    CHECK_TRUE(operations.find("align_scope") != nullptr);
    CHECK_TRUE(functions.find("geometry.area") != nullptr);
    CHECK_TRUE(functions.find("clamp") != nullptr);

    // Absent, so a rule that calls them says so rather than doing nothing. These
    // are the rows D3 and D4 fill in; the table is what they register into.
    CHECK_TRUE(operations.find("roof") == nullptr);
    CHECK_TRUE(operations.find("primitive") == nullptr);
    CHECK_TRUE(operations.find("center") == nullptr);

    // The counts, so that a row added here without a test below is noticed. The
    // two tests that follow pin every one of them to an ANSWER, which presence
    // alone cannot: registering op_mirror_scope under "mirror" satisfies every
    // assertion above.
    CHECK_EQ(operations.size(), size_t{14});
    CHECK_EQ(functions.size(), size_t{18});
}

TEST(Interpreter, every_registered_operation_reaches_its_own_handler) {
    // Through a RULE, not by calling the shape.cpp function directly, so that the
    // op_* wrapper, its argument parsing and its row in the table are all on the
    // path. Each case is chosen so that the neighbouring handler gives a
    // different answer -- mirror against mirror_scope, rotate against
    // rotate_scope, scale against translate -- because a table whose rows are
    // shuffled is otherwise invisible.

    // scale sets an absolute size; translate in its place leaves the volume at 1.
    const GenerationResult scaled =
        run_square("@start\nrule Main { extrude(1.0); scale(4.0, 6.0, 8.0); }\n");
    CHECK_TRUE(scaled.ok());
    CHECK_EQ(scaled.terminals.size(), size_t{1});
    if (!scaled.terminals.empty()) {
        CHECK_NEAR(geometry_volume(scaled.terminals[0].geometry), 192.0, 1e-9);
    }

    // translate moves the shape in the world and leaves the size alone.
    const GenerationResult moved =
        run_square("@start\nrule Main { extrude(1.0); translate(4.0, 6.0, 8.0); }\n");
    CHECK_TRUE(moved.ok());
    if (!moved.terminals.empty()) {
        glm::dvec3 low{0.0};
        glm::dvec3 high{0.0};
        world_bounds(moved.terminals[0], low, high);
        check_vec3(low, glm::dvec3{4.0, 6.0, 8.0}, 1e-12);
        check_vec3(high, glm::dvec3{5.0, 7.0, 9.0}, 1e-12);
    }

    // rotate turns the GEOMETRY: the world box moves. rotate_scope in its place
    // would leave the world box at (0,0,0)..(2,1,4) and turn axes[0] instead.
    const GenerationResult turned =
        run_on("@start\nrule Main { extrude(1.0); rotate(0, 90, 0); }\n", shape_from_rect(2.0, 4.0));
    CHECK_TRUE(turned.ok());
    if (!turned.terminals.empty()) {
        glm::dvec3 low{0.0};
        glm::dvec3 high{0.0};
        world_bounds(turned.terminals[0], low, high);
        check_vec3(low, glm::dvec3{0.0, 0.0, -2.0}, 1e-12);
        check_vec3(high, glm::dvec3{4.0, 1.0, 0.0}, 1e-12);
        check_vec3(turned.terminals[0].scope.axes[0], glm::dvec3{1.0, 0.0, 0.0}, 0.0);
    }

    // rotate_scope turns the FRAME: the world box does not move and axes[0] does.
    const GenerationResult reframed = run_on(
        "@start\nrule Main { extrude(1.0); rotate_scope(0, 90, 0); }\n", shape_from_rect(2.0, 4.0));
    CHECK_TRUE(reframed.ok());
    if (!reframed.terminals.empty()) {
        glm::dvec3 low{0.0};
        glm::dvec3 high{0.0};
        world_bounds(reframed.terminals[0], low, high);
        check_vec3(low, glm::dvec3{0.0, 0.0, 0.0}, 1e-12);
        check_vec3(high, glm::dvec3{2.0, 1.0, 4.0}, 1e-12);
        check_vec3(reframed.terminals[0].scope.axes[0], glm::dvec3{0.0, 0.0, -1.0}, 0.0);
    }

    // mirror reflects the geometry about the pivot plane, which the default pivot
    // puts at the box minimum, so the whole solid lands on the far side of it.
    // The volume stays POSITIVE because the faces were reversed with it.
    const GenerationResult mirrored =
        run_square("@start\nrule Main { extrude(1.0); mirror(\"x\"); }\n");
    CHECK_TRUE(mirrored.ok());
    if (!mirrored.terminals.empty()) {
        glm::dvec3 low{0.0};
        glm::dvec3 high{0.0};
        world_bounds(mirrored.terminals[0], low, high);
        check_vec3(low, glm::dvec3{-1.0, 0.0, 0.0}, 1e-12);
        check_vec3(high, glm::dvec3{0.0, 1.0, 1.0}, 1e-12);
        CHECK_NEAR(geometry_volume(mirrored.terminals[0].geometry), 1.0, 1e-12);
        check_vec3(mirrored.terminals[0].scope.axes[0], glm::dvec3{1.0, 0.0, 0.0}, 0.0);
    }

    // mirror_scope negates a scope AXIS and leaves every vertex in the world, so
    // the frame turns left-handed. The HANDEDNESS lives in the frame and only in
    // the frame: the local loops keep the Face contract -- wound
    // counter-clockwise seen from outside -- so the local signed volume stays
    // POSITIVE, and reframe() re-winds them to make that true.
    //
    // This used to assert -1.0, pinning the opposite. A reflection reverses
    // orientation, so leaving the loops alone left every local loop violating
    // the invariant every consumer of ShapeGeometry reads, and
    // append_shape_to_mesh() then emitted all twelve triangles of the box
    // inside out -- culled front-first and lit from behind. The sign of a local
    // volume is not the thing to pin; whether the world triangles agree with
    // their own normals is, and that is asserted in
    // a_mirrored_scope_still_emits_outward_facing_triangles below.
    const GenerationResult flipped_frame =
        run_square("@start\nrule Main { extrude(1.0); mirror_scope(\"x\"); }\n");
    CHECK_TRUE(flipped_frame.ok());
    if (!flipped_frame.terminals.empty()) {
        glm::dvec3 low{0.0};
        glm::dvec3 high{0.0};
        world_bounds(flipped_frame.terminals[0], low, high);
        check_vec3(low, glm::dvec3{0.0, 0.0, 0.0}, 1e-12);
        check_vec3(high, glm::dvec3{1.0, 1.0, 1.0}, 1e-12);
        CHECK_NEAR(geometry_volume(flipped_frame.terminals[0].geometry), 1.0, 1e-12);
        check_vec3(flipped_frame.terminals[0].scope.axes[0], glm::dvec3{-1.0, 0.0, 0.0}, 0.0);
    }

    // reverse_normals turns the solid inside out and touches neither the frame
    // nor the vertices, which is what separates it from mirror_scope above.
    const GenerationResult inside_out =
        run_square("@start\nrule Main { extrude(1.0); reverse_normals(); }\n");
    CHECK_TRUE(inside_out.ok());
    if (!inside_out.terminals.empty()) {
        CHECK_NEAR(geometry_volume(inside_out.terminals[0].geometry), -1.0, 1e-12);
        check_vec3(inside_out.terminals[0].scope.axes[0], glm::dvec3{1.0, 0.0, 0.0}, 0.0);
    }

    // offset shrinks the outline by the distance on every side.
    const GenerationResult inset = run_square("@start\nrule Main { offset(-0.25); }\n");
    CHECK_TRUE(inset.ok());
    if (!inset.terminals.empty()) {
        CHECK_NEAR(geometry_area(inset.terminals[0].geometry), 0.25, 1e-9);
        glm::dvec3 low{0.0};
        glm::dvec3 high{0.0};
        world_bounds(inset.terminals[0], low, high);
        check_vec3(low, glm::dvec3{0.25, 0.0, 0.25}, 1e-9);
        check_vec3(high, glm::dvec3{0.75, 0.0, 0.75}, 1e-9);
    }

    // align_scope puts the frame back on the world axes and leaves the vertices.
    const GenerationResult aligned = run_on(
        "@start\nrule Main { rotate_scope(0, 90, 0); align_scope(\"world\"); }\n",
        shape_from_rect(1.0, 2.0));
    CHECK_TRUE(aligned.ok());
    if (!aligned.terminals.empty()) {
        check_vec3(aligned.terminals[0].scope.axes[0], glm::dvec3{1.0, 0.0, 0.0}, 0.0);
        check_vec3(aligned.terminals[0].scope.size, glm::dvec3{1.0, 0.0, 2.0}, 0.0);
    }

    // set_pivot changes what mirror reflects about: about the CENTRE the solid
    // maps onto itself, where the default pivot moved it a whole width.
    const GenerationResult about_centre =
        run_square("@start\nrule Main { extrude(1.0); set_pivot(\"center\"); mirror(\"x\"); }\n");
    CHECK_TRUE(about_centre.ok());
    if (!about_centre.terminals.empty()) {
        glm::dvec3 low{0.0};
        glm::dvec3 high{0.0};
        world_bounds(about_centre.terminals[0], low, high);
        check_vec3(low, glm::dvec3{0.0, 0.0, 0.0}, 1e-12);
        check_vec3(high, glm::dvec3{1.0, 1.0, 1.0}, 1e-12);
    }
}

TEST(Interpreter, every_registered_function_is_pinned_to_its_answer) {
    // Presence in the table is not the same as the name reaching the right
    // handler: `min` registered to fn_max, `floor` to std::ceil and `str`
    // returning the empty string are all invisible to a test that only looks the
    // names up. Every argument is chosen so that the neighbouring function gives
    // a DIFFERENT answer -- min against max, floor against ceil, abs against the
    // identity.
    const GenerationResult result = run_square(
        "@start\n"
        "rule Main {\n"
        "    print(min(3.0, -2.0, 7.0), max(3.0, -2.0, 7.0));\n"
        "    print(clamp(9.0, 0.0, 4.0), clamp(-9.0, 0.0, 4.0), clamp(2.0, 0.0, 4.0));\n"
        "    print(pow(2.0, 10.0), sqrt(9.0), abs(-2.5), abs(2.5));\n"
        "    print(floor(-2.5), ceil(-2.5), round(2.5), round(3.5));\n"
        "    print(sin(90.0), cos(180.0), tan(45.0), sin(0.0));\n"
        "    print(len([1.0, 2.0, 3.0]), len(\"abcd\"), str(2.5) + \"!\", str(true));\n"
        "}\n");

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.log.size(), size_t{6});
    if (result.log.size() == 6) {
        CHECK_EQ(result.log[0], std::string{"-2 7"});
        // The high bound, then the low bound, then neither: a clamp that ignored
        // its high bound would print 9 first.
        CHECK_EQ(result.log[1], std::string{"4 0 2"});
        CHECK_EQ(result.log[2], std::string{"1024 3 2.5 2.5"});
        // floor(-2.5) is -3 and ceil(-2.5) is -2, so swapping them is visible.
        // round() breaks ties to EVEN, which is why 2.5 goes to 2 and 3.5 to 4.
        CHECK_EQ(result.log[3], std::string{"-3 -2 2 4"});
        CHECK_EQ(result.log[4], std::string{"1 -1 1 0"});
        CHECK_EQ(result.log[5], std::string{"3 4 2.5! true"});
    }

    // The geometry namespace, against a shape whose numbers are all different
    // from each other so that face_area cannot be mistaken for area.
    const GenerationResult geometry = run_on(
        "@start\n"
        "rule Main {\n"
        "    extrude(3.0);\n"
        "    print(geometry.volume(), geometry.face_count());\n"
        "    print(geometry.area(), geometry.face_area(0.0));\n"
        "}\n",
        shape_from_rect(2.0, 4.0));
    CHECK_TRUE(geometry.ok());
    CHECK_EQ(geometry.log.size(), size_t{2});
    if (geometry.log.size() == 2) {
        // 2 x 3 x 4, six faces; the surface is 2*(8 + 6 + 12) = 52, and face 0 is
        // the 2 by 4 base.
        CHECK_EQ(geometry.log[0], std::string{"24 6"});
        CHECK_EQ(geometry.log[1], std::string{"52 8"});
    }

    // Each of these is a REFUSAL, not a plausible number: sqrt of a negative and
    // tan of a quarter turn both have no answer, and clamp with its bounds the
    // wrong way round is a mistake in the rule rather than a value to guess at.
    const GenerationResult bad_sqrt = run_square("@start\nrule Main { print(sqrt(-1.0)); }\n");
    CHECK_FALSE(bad_sqrt.ok());
    CHECK_TRUE(has_error_saying(bad_sqrt, "sqrt"));

    const GenerationResult bad_tan = run_square("@start\nrule Main { print(tan(90.0)); }\n");
    CHECK_FALSE(bad_tan.ok());
    CHECK_TRUE(has_error_saying(bad_tan, "'tan' is undefined at 90 degrees"));

    const GenerationResult bad_clamp =
        run_square("@start\nrule Main { print(clamp(1.0, 4.0, 0.0)); }\n");
    CHECK_FALSE(bad_clamp.ok());
    CHECK_TRUE(has_error_saying(bad_clamp, "above its high bound"));

    const GenerationResult bad_len = run_square("@start\nrule Main { print(len(3.0)); }\n");
    CHECK_FALSE(bad_len.ok());
    CHECK_TRUE(has_error_saying(bad_len, "'len' wants an array or a string"));
}

TEST(Interpreter, a_caller_can_register_an_operation_without_touching_the_interpreter) {
    // The extension point D3, D4, D5 and D6 use for their operation families. If
    // this stops working, adding an operation means editing interpreter.cpp.
    stratum::procgen::rules::OperationTable operations =
        stratum::procgen::rules::standard_operations();

    int calls = 0;
    operations.register_operation(
        "roof", [&calls](stratum::procgen::rules::OperationArgs& context) {
            ++calls;
            // A handler sees evaluated Values and the shape, and nothing of the AST.
            CHECK_EQ(context.args.size(), size_t{1});
            if (!context.args.empty()) {
                CHECK_TRUE(context.args[0].is_text());
            }
            context.interpreter.log("roof on a shape " +
                                    stratum::procgen::rules::format_number(context.shape.scope.size.y) +
                                    " tall");
        });

    const ParseResult parsed = parse(
        "@start\n"
        "rule Main { extrude(3.0); roof(\"gable\"); }\n",
        "test.srl");
    CHECK_EQ(parsed.render_all(""), std::string{});

    const GenerationResult result =
        generate(parsed.file, shape_from_rect(1.0, 1.0), {}, &operations, nullptr);

    CHECK_TRUE(result.ok());
    CHECK_EQ(calls, 1);
    CHECK_EQ(result.log.size(), size_t{1});
    if (!result.log.empty()) {
        CHECK_EQ(result.log[0], std::string{"roof on a shape 3 tall"});
    }
}

TEST(Interpreter, the_start_option_runs_the_rule_it_names_and_not_the_files_own) {
    // What an editor's "generate from this rule" button needs. Without a test an
    // interpreter that ignored GenerationOptions::start and always took @start
    // would pass, because every other test in this file leaves it unset.
    const ParseResult parsed = parse(
        "@start\n"
        "rule Main { extrude(1.0); }\n"
        "rule Other { extrude(7.0); }\n",
        "test.srl");
    CHECK_EQ(parsed.render_all(""), std::string{});
    CHECK_EQ(parsed.file.rules.size(), size_t{2});

    // The control: left unset, the file's own @start runs.
    const GenerationResult by_default = generate(parsed.file, shape_from_rect(1.0, 1.0), {});
    CHECK_TRUE(by_default.ok());
    CHECK_EQ(by_default.terminals.size(), size_t{1});
    if (!by_default.terminals.empty()) {
        CHECK_EQ(by_default.terminals[0].rule, std::string{"Main"});
        CHECK_NEAR(geometry_volume(by_default.terminals[0].geometry), 1.0, 1e-12);
    }

    GenerationOptions chosen;
    chosen.start = 1;
    const GenerationResult by_choice = generate(parsed.file, shape_from_rect(1.0, 1.0), chosen);
    CHECK_TRUE(by_choice.ok());
    CHECK_EQ(by_choice.terminals.size(), size_t{1});
    if (!by_choice.terminals.empty()) {
        CHECK_EQ(by_choice.terminals[0].rule, std::string{"Other"});
        CHECK_NEAR(geometry_volume(by_choice.terminals[0].geometry), 7.0, 1e-12);
    }

    // A start rule that is not in the file is reported rather than falling back
    // to @start, which would silently generate the wrong building.
    GenerationOptions missing;
    missing.start = 99;
    const GenerationResult refused = generate(parsed.file, shape_from_rect(1.0, 1.0), missing);
    CHECK_FALSE(refused.ok());
    CHECK_TRUE(refused.terminals.empty());
}

TEST(Interpreter, the_diagnostic_list_is_capped_and_says_that_it_was_cut) {
    // A generation that earns a hundred runtime errors has one mistake being
    // reported a hundred times. The list is cut, and the last entry says so --
    // otherwise a caller cannot tell a hundred faults from a hundred and one.
    std::string source = "@start\nrule Main {";
    for (int i = 0; i < 150; ++i) {
        source += " A();";
    }
    source += " }\nrule A { extrude(0.0); }\n";

    const GenerationResult result = run_square(source);

    CHECK_FALSE(result.ok());
    CHECK_EQ(result.diagnostics.size(),
             static_cast<size_t>(stratum::procgen::rules::kMaxDiagnostics));
    if (!result.diagnostics.empty()) {
        CHECK_EQ(result.diagnostics.back().message,
                 std::string{stratum::procgen::rules::kTooManyDiagnostics});
        CHECK_EQ(result.diagnostics.back().severity, Severity::Error);
        // And everything before the sentinel is a real fault, not filler.
        CHECK_TRUE(says(result.diagnostics.front().message, "extrusion distance is zero"));
    }

    // The control: below the cap nothing is cut and no sentinel appears.
    const GenerationResult small = run_square(
        "@start\n"
        "rule Main { A(); A(); }\n"
        "rule A { extrude(0.0); }\n");
    CHECK_EQ(small.diagnostics.size(), size_t{2});
    for (const Diagnostic& diagnostic : small.diagnostics) {
        CHECK_FALSE(diagnostic.message == std::string{stratum::procgen::rules::kTooManyDiagnostics});
    }
}

TEST(Interpreter, a_handler_that_calls_fail_shape_abandons_that_shape_and_not_its_siblings) {
    // interpreter.hpp devotes a paragraph to this path and nothing exercised it.
    // A handler in another translation unit cannot throw the interpreter's
    // private unwinding type, so it marks the shape and the interpreter throws on
    // its behalf as soon as the handler returns. If that throw stopped happening,
    // the handler's `return` would fall back into the rule and the operations
    // after the failure would run on a shape the handler has already refused.
    stratum::procgen::rules::OperationTable operations =
        stratum::procgen::rules::standard_operations();

    int calls = 0;
    int reached_after = 0;
    operations.register_operation(
        "primitive", [&calls, &reached_after](stratum::procgen::rules::OperationArgs& context) {
            ++calls;
            std::string name;
            if (!context.args.empty() && context.args[0].is_text()) {
                name = context.args[0].as_text();
            }
            if (name != "cube") {
                context.interpreter.fail_shape(context.loc,
                                               "there is no primitive called '" + name + "'");
                // Reached: fail_shape() records, it does not unwind. Anything
                // after it in a handler still runs, which is why the interpreter
                // throws at the call boundary rather than trusting the handler.
                ++reached_after;
                return;
            }
            context.interpreter.log("primitive " + name);
        });

    const ParseResult parsed = parse(
        "@start\n"
        "rule Main { Bad(); Good(); }\n"
        "rule Bad { primitive(\"tetrahedron\"); extrude(5.0); }\n"
        "rule Good { primitive(\"cube\"); extrude(2.0); }\n",
        "test.srl");
    CHECK_EQ(parsed.render_all(""), std::string{});

    const GenerationResult result =
        generate(parsed.file, shape_from_rect(1.0, 1.0), {}, &operations, nullptr);

    CHECK_EQ(calls, 2);
    CHECK_EQ(reached_after, 1);
    CHECK_FALSE(result.ok());
    CHECK_TRUE(has_error_saying(result, "there is no primitive called 'tetrahedron'"));

    // Bad produced nothing and its extrude(5) never ran; Good produced its own.
    CHECK_EQ(result.terminals.size(), size_t{1});
    if (!result.terminals.empty()) {
        CHECK_EQ(result.terminals[0].rule, std::string{"Good"});
        CHECK_NEAR(geometry_volume(result.terminals[0].geometry), 2.0, 1e-12);
    }
    CHECK_EQ(result.log.size(), size_t{1});
    if (!result.log.empty()) {
        CHECK_EQ(result.log[0], std::string{"primitive cube"});
    }

    // The diagnostic points at the line that called the handler, not at the
    // handler, because a rule author cannot see interpreter.cpp.
    if (!result.diagnostics.empty()) {
        CHECK_EQ(result.diagnostics[0].loc.line, uint32_t{3});
    }
}

TEST(Interpreter, a_derived_terminal_is_numbered_and_seeded_per_parent) {
    // interpreter.cpp argues at length for a PER-PARENT index rather than one
    // running counter, and nothing checked it. With a running counter every
    // assertion about the terminals themselves still passes: the shapes come out
    // in the same order and with the same geometry. What moves is every derived
    // shape's seed, so adding one setback to one rule would change the random
    // draws of every shape emitted after it anywhere in the file.
    stratum::procgen::rules::OperationTable operations =
        stratum::procgen::rules::standard_operations();

    operations.register_operation(
        "convexify", [](stratum::procgen::rules::OperationArgs& context) {
            Shape child = context.shape;
            (void)context.interpreter.emit_derived_terminal(context.shape, std::move(child),
                                                            "convexify.part");
        });

    const ParseResult parsed = parse(
        "@start\n"
        "rule Main { A(); A(); }\n"
        "rule A { convexify(); convexify(); }\n",
        "test.srl");
    CHECK_EQ(parsed.render_all(""), std::string{});

    const GenerationResult result =
        generate(parsed.file, shape_from_rect(1.0, 1.0), {}, &operations, nullptr);
    CHECK_TRUE(result.ok());

    // Two A shapes, two derived terminals each, and each A is still a terminal of
    // its own because a derived shape is not one of its children.
    std::vector<const Shape*> derived;
    for (const Shape& shape : result.terminals) {
        if (shape.rule == "convexify.part") {
            derived.push_back(&shape);
        }
    }
    CHECK_EQ(derived.size(), size_t{4});
    if (derived.size() != 4) {
        return;
    }

    // Numbered 0, 1 under the first parent and 0, 1 under the second -- not
    // 0, 1, 2, 3. The parents differ, so the pairs do too.
    CHECK_EQ(derived[0]->index, uint32_t{0});
    CHECK_EQ(derived[1]->index, uint32_t{1});
    CHECK_EQ(derived[2]->index, uint32_t{0});
    CHECK_EQ(derived[3]->index, uint32_t{1});
    CHECK_FALSE(derived[0]->parent == derived[2]->parent);
    CHECK_EQ(derived[0]->parent, derived[1]->parent);
    CHECK_EQ(derived[2]->parent, derived[3]->parent);

    // The key is the parent's address mixed with the per-parent index, exactly as
    // a rule call's child key is. Spelled out here rather than merely compared
    // between shapes, so that a change to the mixing has to be a deliberate one.
    for (const Shape& parent : result.terminals) {
        if (parent.rule != "A") {
            continue;
        }
        for (const Shape* child : derived) {
            if (child->parent != parent.id) {
                continue;
            }
            const uint64_t expected = stratum::procgen::rules::seed_mix2(
                parent.seed_key,
                stratum::procgen::rules::seed_mix2(stratum::procgen::rules::kSaltOp,
                                                   child->index));
            CHECK_EQ(child->seed_key, expected);
        }
    }

    // All four keys are different, which a running counter would also give. The
    // assertion that separates the two is the index pair above.
    CHECK_FALSE(derived[0]->seed_key == derived[2]->seed_key);
    CHECK_FALSE(derived[0]->seed_key == derived[1]->seed_key);
}

TEST(Interpreter, a_caller_can_register_an_expression_function_the_same_way) {
    stratum::procgen::rules::FunctionTable functions =
        stratum::procgen::rules::standard_functions();

    functions.register_function(
        "storeys", [](const stratum::procgen::rules::FunctionArgs& context) {
            return Value::number(std::floor(context.shape.scope.size.y / 3.0));
        });

    const ParseResult parsed = parse(
        "@start\n"
        "rule Main { extrude(10.0); print(storeys()); }\n",
        "test.srl");
    CHECK_EQ(parsed.render_all(""), std::string{});

    const GenerationResult result =
        generate(parsed.file, shape_from_rect(1.0, 1.0), {}, nullptr, &functions);

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.log.size(), size_t{1});
    if (!result.log.empty()) {
        CHECK_EQ(result.log[0], std::string{"3"});
    }
}
