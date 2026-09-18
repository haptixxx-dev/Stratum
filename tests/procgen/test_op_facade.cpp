// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_op_facade.cpp
 * @brief E3: window, door and wall_panel, against hand-computed geometry
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ================================================================================
 * HOW THESE TESTS ARE WRITTEN SO THAT THEY CAN FAIL
 * ================================================================================
 *
 * The defect this project has found nine times is a test that cannot fail.
 * Every assertion below is written against a number computed by hand from the
 * panel size and the four lengths, never read back from the output, and the
 * geometry is measured rather than described.
 *
 *   - **The reveal is asserted in WORLD space, as a distance from the wall
 *     plane.** `window()` moves the scope origin back into the wall by the
 *     reveal and then refits, so every LOCAL z in the output is non-negative and
 *     a test that asserted "the glass is at local z 0" would pass for a window
 *     with no reveal at all. The glass is asserted to be exactly 0.15 m behind
 *     the wall face in the world, which is the only statement of the thing this
 *     whole feature exists for.
 *
 *   - **Areas are summed per material and compared to hand arithmetic.** The
 *     twelve strips of a windowed panel have areas that are products of the
 *     inset, the frame width and the panel size; a face emitted at the wrong
 *     coordinate has the wrong area, and a face left out has none. Counting
 *     faces alone passes for an implementation that emits fourteen copies of the
 *     same quad.
 *
 *   - **Normals are asserted individually, per part.** A jamb wound backwards is
 *     invisible to an area assertion, to a face count and to the scope, and
 *     renders black from the street. Each reveal return is checked for the
 *     direction it must face INTO the opening.
 *
 *   - **The no-ledge case asserts a closed book.** With no sill, the surround
 *     area plus the opening area must equal the panel area exactly: no wall is
 *     lost and none is counted twice. That single equation catches a strip
 *     emitted at the wrong coordinate, a strip emitted twice and a strip left
 *     out, none of which a face count sees.
 *
 *   - **Warning paths assert the sentence the code really writes, and count it
 *     exactly.** `CHECK_EQ(count, 1)`, never `count <= 1`: an upper bound passes
 *     when the message disappears, which is half of what "reported once" means.
 *
 *   - **Registration is tested by its absence.** One test runs the same rule
 *     file through standard_operations() and asserts the "not implemented"
 *     diagnostic, so the suite cannot pass by the operations having been wired
 *     in somewhere else.
 *
 *   - **The determinism test asserts BOTH directions.** Two runs must agree byte
 *     for byte, and two runs whose reveal differs by one millimetre must not.
 *     Agreement alone passes for a dump that prints nothing about the geometry;
 *     dump_shape() prints every vertex of every face, and the second half of the
 *     test is what says so.
 *
 * Nothing here needs a GPU, a window or a file on disk, so nothing here skips.
 */

#include "framework.hpp"

#include "procgen/rules/ast.hpp"
#include "procgen/rules/interpreter.hpp"
#include "procgen/rules/lexer.hpp"
#include "osm/road/road_style.hpp"
#include "procgen/rules/op_facade.hpp"
#include "procgen/rules/parser.hpp"
#include "procgen/rules/registry.hpp"
#include "procgen/rules/shape.hpp"

#include <glm/glm.hpp>

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

using stratum::procgen::rules::Diagnostic;
using stratum::procgen::rules::Face;
using stratum::procgen::rules::face_area;
using stratum::procgen::rules::face_normal;
using stratum::procgen::rules::FacadePart;
using stratum::procgen::rules::facade_material;
using stratum::procgen::rules::facade_operations;
using stratum::procgen::rules::facade_part_name;
using stratum::procgen::rules::FacadeReport;
using stratum::procgen::rules::full_functions;
using stratum::procgen::rules::full_operations;
using stratum::procgen::rules::GenerationOptions;
using stratum::procgen::rules::GenerationResult;
using stratum::procgen::rules::generate;
using stratum::procgen::rules::geometry_area;
using stratum::procgen::rules::geometry_volume;
using stratum::procgen::rules::kDefaultFrameWidth;
using stratum::procgen::rules::kDefaultRevealFraction;
using stratum::procgen::rules::kDefaultSillProjection;
using stratum::procgen::rules::kDefaultWallThickness;
using stratum::procgen::rules::kPanelHeightAttribute;
using stratum::procgen::rules::kPanelWidthAttribute;
using stratum::procgen::rules::kSillThickness;
using stratum::procgen::rules::kThresholdThickness;
using stratum::procgen::rules::kWallThicknessAttribute;
using stratum::procgen::rules::OpeningParams;
using stratum::procgen::rules::OpResult;
using stratum::procgen::rules::orient_panel;
using stratum::procgen::rules::parse;
using stratum::procgen::rules::ParseResult;
using stratum::procgen::rules::refit_scope;
using stratum::procgen::rules::Severity;
using stratum::procgen::rules::Shape;
using stratum::procgen::rules::standard_functions;
using stratum::procgen::rules::standard_operations;
using stratum::procgen::rules::shape_from_rect;
using stratum::procgen::rules::Value;
using stratum::procgen::rules::wall_panel_shape;
using stratum::procgen::rules::window_shape;
using stratum::procgen::rules::door_shape;
using stratum::MaterialId;
using stratum::MaterialKey;

namespace {

// ============================================================================
// Helpers
// ============================================================================

/**
 * @brief A flat wall panel: @p width across, @p height up, facing +z, at the origin
 *
 * Built by hand rather than through `select face` so that the expected numbers
 * are the ones written in the test and not the ones a component split produced.
 * The genuine path -- extrude, `select face`, `split`, `split` -- is exercised
 * by the rule-language tests at the bottom of this file, against the same
 * arithmetic.
 */
[[nodiscard]] Shape make_wall(double width, double height) {
    Shape shape;
    shape.geometry.positions = {glm::dvec3{0.0, 0.0, 0.0},
                                glm::dvec3{width, 0.0, 0.0},
                                glm::dvec3{width, height, 0.0},
                                glm::dvec3{0.0, height, 0.0}};
    Face face;
    face.loop = {0, 1, 2, 3};  // counter-clockwise in xy, so the normal is +z
    shape.geometry.faces.push_back(face);
    refit_scope(shape);
    return shape;
}

/// A right triangle facing +z: a gable panel, which is not a rectangle
[[nodiscard]] Shape make_triangle(double width, double height) {
    Shape shape;
    shape.geometry.positions = {glm::dvec3{0.0, 0.0, 0.0},
                                glm::dvec3{width, 0.0, 0.0},
                                glm::dvec3{0.0, height, 0.0}};
    Face face;
    face.loop = {0, 1, 2};
    shape.geometry.faces.push_back(face);
    refit_scope(shape);
    return shape;
}

/**
 * @brief A four-cornered face from four points, in the order given
 *
 * Deliberately NOT a rectangle helper: the points are whatever the caller wrote,
 * so a trapezoid, a parallelogram and a quad bent out of plane are all one line.
 * Those three are the cases the rectangularity and flatness guards exist for, and
 * a helper that could only build rectangles is why they went untested.
 */
[[nodiscard]] Shape make_quad(const glm::dvec3& a,
                              const glm::dvec3& b,
                              const glm::dvec3& c,
                              const glm::dvec3& d) {
    Shape shape;
    shape.geometry.positions = {a, b, c, d};
    Face face;
    face.loop = {0, 1, 2, 3};
    shape.geometry.faces.push_back(face);
    refit_scope(shape);
    return shape;
}

/**
 * @brief A square panel with a square hole in it, facing +z
 *
 * @p width by @p width with a @p hole by @p hole opening at its centre, wound
 * against the outer ring so face_area() subtracts it. This is what proves
 * extract_face() copies the holes: a panel whose hole was dropped has the same
 * corner count, the same bounds and the same scope, and differs only in area.
 */
[[nodiscard]] Shape make_wall_with_hole(double width, double hole) {
    const double low = 0.5 * (width - hole);
    const double high = low + hole;
    Shape shape;
    shape.geometry.positions = {glm::dvec3{0.0, 0.0, 0.0},   glm::dvec3{width, 0.0, 0.0},
                                glm::dvec3{width, width, 0.0}, glm::dvec3{0.0, width, 0.0},
                                glm::dvec3{low, low, 0.0},   glm::dvec3{low, high, 0.0},
                                glm::dvec3{high, high, 0.0}, glm::dvec3{high, low, 0.0}};
    Face face;
    face.loop = {0, 1, 2, 3};
    face.holes.push_back({4, 5, 6, 7});  // clockwise in xy, against the outer ring
    shape.geometry.faces.push_back(face);
    refit_scope(shape);
    return shape;
}

[[nodiscard]] size_t part_faces(const Shape& shape, FacadePart part) {
    const MaterialKey key = facade_material(part);
    size_t count = 0;
    for (const Face& face : shape.geometry.faces) {
        if (face.material == key) {
            ++count;
        }
    }
    return count;
}

[[nodiscard]] double part_area(const Shape& shape, FacadePart part) {
    const MaterialKey key = facade_material(part);
    double total = 0.0;
    for (const Face& face : shape.geometry.faces) {
        if (face.material == key) {
            total += face_area(shape.geometry, face);
        }
    }
    return total;
}

/// Every normal carried by a part's faces, in the order the faces were built
[[nodiscard]] std::vector<glm::dvec3> part_normals(const Shape& shape, FacadePart part) {
    const MaterialKey key = facade_material(part);
    std::vector<glm::dvec3> out;
    for (const Face& face : shape.geometry.faces) {
        if (face.material == key) {
            out.push_back(face_normal(shape.geometry, face));
        }
    }
    return out;
}

/**
 * @brief World-space bounds of one part's vertices
 *
 * World, not local, because every facade operation moves the scope origin: a
 * local assertion is an assertion about the refit and not about where the glass
 * ended up.
 */
void part_world_bounds(const Shape& shape,
                       FacadePart part,
                       glm::dvec3& min_out,
                       glm::dvec3& max_out) {
    const MaterialKey key = facade_material(part);
    min_out = glm::dvec3{1.0e300};
    max_out = glm::dvec3{-1.0e300};
    for (const Face& face : shape.geometry.faces) {
        if (face.material != key) {
            continue;
        }
        for (const uint32_t index : face.loop) {
            const glm::dvec3 world = shape.scope.to_world(shape.geometry.positions[index]);
            min_out = glm::min(min_out, world);
            max_out = glm::max(max_out, world);
        }
    }
}

/**
 * @brief World-space bounds of a part, asserted to be one plane of constant z
 *
 * The narrow assertion the reveal needs, given a name because it is wanted in
 * four places: a part that lies flat on the wall and a part set back to the
 * glazing line have the SAME area, the SAME face count, the SAME normals and the
 * same everything else this file measures. Only the world z tells them apart, and
 * asserting the minimum alone passes for a part smeared across the reveal.
 */
void check_part_plane_z(const Shape& shape, FacadePart part, double z, double eps) {
    glm::dvec3 min_corner{0.0};
    glm::dvec3 max_corner{0.0};
    part_world_bounds(shape, part, min_corner, max_corner);
    CHECK_NEAR(min_corner.z, z, eps);
    CHECK_NEAR(max_corner.z, z, eps);
}

/**
 * @brief World-space bounds of a shape's own scope box, over its eight corners
 *
 * The scope and not the vertices, because what a later `split` cuts is the BOX.
 * A split after a window is measured against the box the window left, and that
 * box is the thing this measures.
 */
void scope_world_bounds(const Shape& shape, glm::dvec3& min_out, glm::dvec3& max_out) {
    min_out = glm::dvec3{1.0e300};
    max_out = glm::dvec3{-1.0e300};
    for (int corner = 0; corner < 8; ++corner) {
        const glm::dvec3 local{(corner & 1) != 0 ? shape.scope.size.x : 0.0,
                               (corner & 2) != 0 ? shape.scope.size.y : 0.0,
                               (corner & 4) != 0 ? shape.scope.size.z : 0.0};
        const glm::dvec3 world = shape.scope.to_world(local);
        min_out = glm::min(min_out, world);
        max_out = glm::max(max_out, world);
    }
}

[[nodiscard]] glm::dvec3 part_world_centroid(const Shape& shape, FacadePart part) {
    const MaterialKey key = facade_material(part);
    glm::dvec3 total{0.0};
    size_t count = 0;
    for (const Face& face : shape.geometry.faces) {
        if (face.material != key) {
            continue;
        }
        for (const uint32_t index : face.loop) {
            total += shape.scope.to_world(shape.geometry.positions[index]);
            ++count;
        }
    }
    if (count == 0) {
        return glm::dvec3{1.0e300};  // must fail any tolerance, never pass one
    }
    return total / static_cast<double>(count);
}

void check_vec3(const glm::dvec3& actual, const glm::dvec3& expected, double eps) {
    CHECK_NEAR(actual.x, expected.x, eps);
    CHECK_NEAR(actual.y, expected.y, eps);
    CHECK_NEAR(actual.z, expected.z, eps);
}

/// Parse @p source and run it with the facade operations registered
[[nodiscard]] GenerationResult run_on(const std::string& source,
                                      const Shape& seed,
                                      const GenerationOptions& options = {}) {
    const ParseResult parsed = parse(source, "test.srl");
    CHECK_EQ(parsed.render_all(source), std::string{});
    return generate(parsed.file, seed, options, &facade_operations(), nullptr);
}

/// How many diagnostics of @p severity contain @p needle
[[nodiscard]] size_t count_messages(const GenerationResult& result,
                                    Severity severity,
                                    const std::string& needle) {
    size_t count = 0;
    for (const Diagnostic& diagnostic : result.diagnostics) {
        if (diagnostic.severity == severity &&
            diagnostic.message.find(needle) != std::string::npos) {
            ++count;
        }
    }
    return count;
}

/// How many of @p report's warnings contain @p needle
[[nodiscard]] size_t count_warnings(const FacadeReport& report, const std::string& needle) {
    size_t count = 0;
    for (const std::string& warning : report.warnings) {
        if (warning.find(needle) != std::string::npos) {
            ++count;
        }
    }
    return count;
}

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

/**
 * @brief Every terminal's per-part face count and area, as one block of text
 *
 * dump_shape() prints every vertex of every face and prints NO MaterialKey, so a
 * comparison resting on it alone cannot see a facade whose glass and stone were
 * swapped -- the vertices are identical. This is the half it does not cover, and
 * the determinism test compares both.
 */
[[nodiscard]] std::string material_signature(const GenerationResult& result) {
    static const FacadePart kParts[] = {FacadePart::Panel, FacadePart::Reveal,
                                        FacadePart::Frame, FacadePart::Glass,
                                        FacadePart::Sill,  FacadePart::Leaf,
                                        FacadePart::Threshold};
    std::string out;
    for (const Shape& shape : result.terminals) {
        out += shape.rule;
        for (const FacadePart part : kParts) {
            char buffer[128];
            std::snprintf(buffer, sizeof(buffer), " %s=%zu/%.9f", facade_part_name(part),
                          part_faces(shape, part), part_area(shape, part));
            out += buffer;
        }
        out += '\n';
    }
    return out;
}

// The end-to-end facade from tests/procgen/test_rule_statements.cpp, with the
// three terminal rules filled in. Every number this file asserts about it is
// derived in that file's comment; what is added here is the depth.
const char* const kFacadeSource =
    "@start\n"
    "rule Main {\n"
    "    extrude(10.0);\n"
    "    select face {\n"
    "        front : { Facade(); }\n"
    "    }\n"
    "}\n"
    "\n"
    "rule Facade {\n"
    "    split(y) {\n"
    "        4.0 : { Shopfront(); }\n"
    "        repeat { 2.0 : { UpperFloor(); } }\n"
    "    }\n"
    "}\n"
    "\n"
    "rule UpperFloor {\n"
    "    split(x) { repeat { 1.5 : { Bay(); } } }\n"
    "}\n"
    "\n"
    "rule Bay {\n"
    "    split(x) {\n"
    "        0.4 : { Pier(); }\n"
    "        0.6 : { Window(); }\n"
    "        ~1.0 : { Pier(); }\n"
    "    }\n"
    "}\n"
    "\n"
    "rule Shopfront { door(0.3); }\n"
    "rule Pier       { wall_panel(); }\n"
    "rule Window     { window(); }\n";

// ============================================================================
// The reveal: the one thing this file exists for
// ============================================================================

/**
 * @brief The glass of a bare window sits back from the wall face
 *
 * The panel is 0.6 by 2.0 at the world origin facing +z, which is exactly what
 * the end-to-end facade's `Window` tile is. With no arguments: the wall is
 * kDefaultWallThickness thick, so the reveal is half of it, 0.15.
 *
 * Asserted in WORLD space. The operation moves the scope origin back by the
 * reveal and refits, so the glass ends up at LOCAL z 0 whatever the reveal was,
 * and a local assertion would pass for a window drawn flat on the wall.
 */
TEST(OpFacade, a_bare_window_sets_its_glass_back_from_the_wall_face) {
    Shape shape = make_wall(0.6, 2.0);
    FacadeReport report;
    const OpResult result = window_shape(shape, OpeningParams{}, &report);

    CHECK_TRUE(result.ok);
    CHECK_EQ(report.warnings.size(), size_t{0});
    CHECK_NEAR(report.wall_thickness, kDefaultWallThickness, 1e-12);
    CHECK_NEAR(report.reveal, kDefaultWallThickness * kDefaultRevealFraction, 1e-12);
    CHECK_NEAR(report.reveal, 0.15, 1e-12);

    // One pane, 0.6 - 2 * 0.05 wide by 2.0 - 2 * 0.05 high.
    CHECK_EQ(part_faces(shape, FacadePart::Glass), size_t{1});
    CHECK_NEAR(part_area(shape, FacadePart::Glass), 0.5 * 1.9, 1e-12);

    // The wall face is the plane z = 0; every glass vertex is 0.15 behind it.
    glm::dvec3 glass_min{0.0};
    glm::dvec3 glass_max{0.0};
    part_world_bounds(shape, FacadePart::Glass, glass_min, glass_max);
    check_vec3(glass_min, glm::dvec3{0.05, 0.05, -0.15}, 1e-12);
    check_vec3(glass_max, glm::dvec3{0.55, 1.95, -0.15}, 1e-12);
    check_vec3(part_world_centroid(shape, FacadePart::Glass),
               glm::dvec3{0.3, 1.0, -0.15}, 1e-12);

    // And the glass faces out of the wall, not into it.
    const std::vector<glm::dvec3> normals = part_normals(shape, FacadePart::Glass);
    CHECK_EQ(normals.size(), size_t{1});
    if (normals.size() == 1) {
        check_vec3(normals[0], glm::dvec3{0.0, 0.0, 1.0}, 1e-12);
    }
}

/// A reveal of zero is a flush window, and it emits no returns rather than four flat ones
TEST(OpFacade, a_window_with_no_reveal_leaves_the_glass_on_the_wall_plane) {
    Shape shape = make_wall(0.6, 2.0);
    OpeningParams params;
    params.reveal = 0.0;
    params.ledge = 0.0;
    FacadeReport report;
    const OpResult result = window_shape(shape, params, &report);

    CHECK_TRUE(result.ok);
    CHECK_NEAR(report.reveal, 0.0, 1e-12);
    CHECK_EQ(part_faces(shape, FacadePart::Reveal), size_t{0});
    check_vec3(part_world_centroid(shape, FacadePart::Glass),
               glm::dvec3{0.3, 1.0, 0.0}, 1e-12);
    // Flat: the whole assembly lies in one plane.
    CHECK_NEAR(shape.scope.size.z, 0.0, 1e-12);
}

/**
 * @brief Every reveal return faces into the opening
 *
 * A jamb wound the other way renders black from the street and is invisible to
 * an area assertion, to a face count and to the scope. With a sill there are
 * three returns -- the sill's own top face is the fourth -- and each one's
 * direction is fixed by which side of the opening it is on.
 */
TEST(OpFacade, every_reveal_return_faces_into_the_opening) {
    Shape shape = make_wall(0.6, 2.0);
    FacadeReport report;
    const OpResult result = window_shape(shape, OpeningParams{}, &report);
    CHECK_TRUE(result.ok);

    const std::vector<glm::dvec3> normals = part_normals(shape, FacadePart::Reveal);
    CHECK_EQ(normals.size(), size_t{3});
    if (normals.size() != 3) {
        return;
    }
    check_vec3(normals[0], glm::dvec3{1.0, 0.0, 0.0}, 1e-12);   // left jamb, looking right
    check_vec3(normals[1], glm::dvec3{-1.0, 0.0, 0.0}, 1e-12);  // right jamb, looking left
    check_vec3(normals[2], glm::dvec3{0.0, -1.0, 0.0}, 1e-12);  // head, looking down

    // 2.0 high by 0.15 deep for each jamb, 0.6 by 0.15 for the head.
    CHECK_NEAR(part_area(shape, FacadePart::Reveal), 2.0 * 0.15 * 2.0 + 0.6 * 0.15, 1e-12);
}

/// Switch the sill off and the bottom return comes back, because nothing else covers it
TEST(OpFacade, a_window_with_no_sill_gets_its_bottom_return_back) {
    Shape shape = make_wall(0.6, 2.0);
    OpeningParams params;
    params.ledge = 0.0;
    FacadeReport report;
    const OpResult result = window_shape(shape, params, &report);
    CHECK_TRUE(result.ok);

    CHECK_EQ(part_faces(shape, FacadePart::Sill), size_t{0});
    const std::vector<glm::dvec3> normals = part_normals(shape, FacadePart::Reveal);
    CHECK_EQ(normals.size(), size_t{4});
    if (normals.size() == 4) {
        check_vec3(normals[3], glm::dvec3{0.0, 1.0, 0.0}, 1e-12);  // bottom return, looking up
    }
    CHECK_NEAR(part_area(shape, FacadePart::Reveal),
               2.0 * 0.15 * 2.0 + 0.6 * 0.15 * 2.0, 1e-12);
}

// ============================================================================
// Where the wall thickness comes from
// ============================================================================

TEST(OpFacade, the_thickness_argument_wins_and_the_reveal_follows_it) {
    Shape shape = make_wall(1.0, 1.5);
    OpeningParams params;
    params.thickness = 0.5;
    FacadeReport report;
    const OpResult result = window_shape(shape, params, &report);

    CHECK_TRUE(result.ok);
    CHECK_EQ(report.warnings.size(), size_t{0});
    CHECK_NEAR(report.wall_thickness, 0.5, 1e-12);
    CHECK_NEAR(report.reveal, 0.25, 1e-12);
    check_vec3(part_world_centroid(shape, FacadePart::Glass),
               glm::dvec3{0.5, 0.75, -0.25}, 1e-12);
}

TEST(OpFacade, the_wall_thickness_attribute_is_used_when_no_argument_names_one) {
    Shape shape = make_wall(1.0, 1.5);
    shape.attributes[kWallThicknessAttribute] = Value::number(0.45);
    FacadeReport report;
    const OpResult result = window_shape(shape, OpeningParams{}, &report);

    CHECK_TRUE(result.ok);
    CHECK_EQ(report.warnings.size(), size_t{0});
    CHECK_NEAR(report.wall_thickness, 0.45, 1e-12);
    CHECK_NEAR(report.reveal, 0.225, 1e-12);
    check_vec3(part_world_centroid(shape, FacadePart::Glass),
               glm::dvec3{0.5, 0.75, -0.225}, 1e-12);
}

TEST(OpFacade, an_explicit_thickness_beats_the_attribute) {
    Shape shape = make_wall(1.0, 1.5);
    shape.attributes[kWallThicknessAttribute] = Value::number(0.45);
    OpeningParams params;
    params.thickness = 0.2;
    FacadeReport report;
    const OpResult result = window_shape(shape, params, &report);

    CHECK_TRUE(result.ok);
    CHECK_NEAR(report.wall_thickness, 0.2, 1e-12);
    CHECK_NEAR(report.reveal, 0.1, 1e-12);
}

/**
 * @brief A wall_thickness that is not a number is reported, not swallowed
 *
 * The default is used, because a facade still has to build. But a flush window
 * looks like a styling choice and points at nothing, so the fallback says so at
 * the line that asked.
 */
TEST(OpFacade, a_wall_thickness_attribute_that_is_not_a_number_warns_and_falls_back) {
    Shape shape = make_wall(1.0, 1.5);
    shape.attributes[kWallThicknessAttribute] = Value::text("thick");
    FacadeReport report;
    const OpResult result = window_shape(shape, OpeningParams{}, &report);

    CHECK_TRUE(result.ok);
    CHECK_EQ(report.warnings.size(), size_t{1});
    CHECK_EQ(count_warnings(report, "is a string and not a number"), size_t{1});
    CHECK_NEAR(report.wall_thickness, kDefaultWallThickness, 1e-12);
}

TEST(OpFacade, a_wall_thickness_attribute_that_is_not_positive_warns_and_falls_back) {
    Shape shape = make_wall(1.0, 1.5);
    shape.attributes[kWallThicknessAttribute] = Value::number(0.0);
    FacadeReport report;
    const OpResult result = window_shape(shape, OpeningParams{}, &report);

    CHECK_TRUE(result.ok);
    CHECK_EQ(report.warnings.size(), size_t{1});
    CHECK_EQ(count_warnings(report, "not a positive length"), size_t{1});
    CHECK_NEAR(report.wall_thickness, kDefaultWallThickness, 1e-12);
}

/// A reveal deeper than the wall would put the glass outside the building
TEST(OpFacade, a_reveal_deeper_than_the_wall_is_cut_back_to_it) {
    Shape shape = make_wall(1.0, 1.5);
    OpeningParams params;
    params.reveal = 1.0;
    FacadeReport report;
    const OpResult result = window_shape(shape, params, &report);

    CHECK_TRUE(result.ok);
    CHECK_TRUE(report.reveal_clamped);
    CHECK_EQ(report.warnings.size(), size_t{1});
    CHECK_EQ(count_warnings(report, "deeper than the wall is thick"), size_t{1});
    CHECK_NEAR(report.reveal, kDefaultWallThickness, 1e-12);
    check_vec3(part_world_centroid(shape, FacadePart::Glass),
               glm::dvec3{0.5, 0.75, -kDefaultWallThickness}, 1e-12);
}

/// A reveal exactly as deep as the wall is not clamped and not warned about
TEST(OpFacade, a_reveal_exactly_as_deep_as_the_wall_is_left_alone) {
    Shape shape = make_wall(1.0, 1.5);
    OpeningParams params;
    params.reveal = kDefaultWallThickness;
    FacadeReport report;
    const OpResult result = window_shape(shape, params, &report);

    CHECK_TRUE(result.ok);
    CHECK_FALSE(report.reveal_clamped);
    CHECK_EQ(report.warnings.size(), size_t{0});
}

// ============================================================================
// An opening that does not fit its panel
// ============================================================================

/**
 * @brief An opening wider than its panel is clipped to it, and says so
 *
 * A negative inset asks for that, and a negative inset is usually an attribute
 * that is wrong. The facade still builds -- the author needs to see the other
 * tiles -- but the message names both sizes.
 */
TEST(OpFacade, an_opening_wider_than_its_panel_is_clipped_and_reported) {
    Shape shape = make_wall(0.6, 2.0);
    OpeningParams params;
    params.inset = -0.5;  // asks for 1.6 by 3.0 in a panel 0.6 by 2.0
    FacadeReport report;
    const OpResult result = window_shape(shape, params, &report);

    CHECK_TRUE(result.ok);
    CHECK_TRUE(report.opening_clipped);
    CHECK_EQ(report.warnings.size(), size_t{1});
    CHECK_EQ(count_warnings(report, "does not fit a panel 0.6 by 2"), size_t{1});

    CHECK_NEAR(report.opening.x, 0.0, 1e-12);
    CHECK_NEAR(report.opening.y, 0.0, 1e-12);
    CHECK_NEAR(report.opening.z, 0.6, 1e-12);
    CHECK_NEAR(report.opening.w, 2.0, 1e-12);

    // Clipped to the panel, so the result is the bare window: no surround left.
    CHECK_EQ(part_faces(shape, FacadePart::Panel), size_t{0});
    CHECK_NEAR(part_area(shape, FacadePart::Glass), 0.5 * 1.9, 1e-12);
}

/**
 * @brief An inset that leaves nothing gives a blank wall, not a hole
 *
 * A bay too narrow for a window is a wall. Failing the shape would delete the
 * bay, and the missing tile is where the mistake is least visible.
 */
TEST(OpFacade, an_inset_that_leaves_no_opening_gives_a_blank_wall_panel) {
    Shape shape = make_wall(0.6, 2.0);
    OpeningParams params;
    params.inset = 0.4;  // 0.4 from each side of a 0.6 panel
    FacadeReport report;
    const OpResult result = window_shape(shape, params, &report);

    CHECK_TRUE(result.ok);
    CHECK_TRUE(report.opening_empty);
    CHECK_EQ(report.warnings.size(), size_t{1});
    CHECK_EQ(count_warnings(report, "leaves no opening in a panel 0.6 by 2"), size_t{1});

    CHECK_EQ(shape.geometry.faces.size(), size_t{1});
    CHECK_EQ(part_faces(shape, FacadePart::Panel), size_t{1});
    CHECK_EQ(part_faces(shape, FacadePart::Glass), size_t{0});
    CHECK_NEAR(geometry_area(shape.geometry), 1.2, 1e-12);
    check_vec3(shape.scope.size, glm::dvec3{0.6, 2.0, 0.0}, 1e-12);
}

/// A frame wide enough to swallow its opening fills it, and there is no glazing
TEST(OpFacade, a_frame_wider_than_its_opening_leaves_no_glazing) {
    Shape shape = make_wall(0.6, 2.0);
    OpeningParams params;
    params.frame = 0.4;  // 0.8 of frame across a 0.6 opening
    FacadeReport report;
    const OpResult result = window_shape(shape, params, &report);

    CHECK_TRUE(result.ok);
    CHECK_TRUE(report.frame_filled_opening);
    CHECK_EQ(report.warnings.size(), size_t{1});
    CHECK_EQ(count_warnings(report, "so there is no glazing in it"), size_t{1});
    CHECK_EQ(part_faces(shape, FacadePart::Glass), size_t{0});
    CHECK_EQ(part_faces(shape, FacadePart::Frame), size_t{1});
    CHECK_NEAR(part_area(shape, FacadePart::Frame), 1.2, 1e-12);
}

/// No frame at all is legal: the glazing then fills the opening edge to edge
TEST(OpFacade, a_window_with_no_frame_is_all_glass) {
    Shape shape = make_wall(0.6, 2.0);
    OpeningParams params;
    params.frame = 0.0;
    FacadeReport report;
    const OpResult result = window_shape(shape, params, &report);

    CHECK_TRUE(result.ok);
    CHECK_EQ(report.warnings.size(), size_t{0});
    CHECK_EQ(part_faces(shape, FacadePart::Frame), size_t{0});
    CHECK_EQ(part_faces(shape, FacadePart::Glass), size_t{1});
    CHECK_NEAR(part_area(shape, FacadePart::Glass), 1.2, 1e-12);
}

// ============================================================================
// The surround
// ============================================================================

/**
 * @brief The surround plus the opening is the whole panel and nothing more
 *
 * With no sill this is an equation, not an estimate: the four strips of wall
 * around a 0.1 inset in a 1.0 by 2.0 panel must come to exactly the panel area
 * less the opening. A strip at the wrong coordinate, a strip emitted twice and a
 * strip left out all break it, and a face count sees none of the three.
 */
TEST(OpFacade, the_surround_and_the_opening_account_for_the_whole_panel) {
    Shape shape = make_wall(1.0, 2.0);
    OpeningParams params;
    params.inset = 0.1;
    params.ledge = 0.0;
    FacadeReport report;
    const OpResult result = window_shape(shape, params, &report);
    CHECK_TRUE(result.ok);
    CHECK_EQ(report.warnings.size(), size_t{0});

    // left 0.1 x 2.0, right 0.1 x 2.0, bottom 0.8 x 0.1, top 0.8 x 0.1
    CHECK_EQ(part_faces(shape, FacadePart::Panel), size_t{4});
    const double surround = part_area(shape, FacadePart::Panel);
    CHECK_NEAR(surround, 0.2 + 0.2 + 0.08 + 0.08, 1e-12);

    const double opening = (1.0 - 0.2) * (2.0 - 0.2);
    CHECK_NEAR(surround + opening, 1.0 * 2.0, 1e-12);

    // And every strip faces out of the wall.
    for (const glm::dvec3& normal : part_normals(shape, FacadePart::Panel)) {
        check_vec3(normal, glm::dvec3{0.0, 0.0, 1.0}, 1e-12);
    }

    // The equation above is invariant under a shift in z: move one strip back to
    // the glazing line and the four areas still sum to the same number, still
    // face +z and still balance the opening. So the strips are pinned to the wall
    // plane here, which is the assertion that says the surround is WALL and not
    // part of the recess.
    check_part_plane_z(shape, FacadePart::Panel, 0.0, 1e-12);
    check_part_plane_z(shape, FacadePart::Frame, -0.15, 1e-12);
}

/**
 * @brief The surround stops short of the sill rather than running through it
 *
 * The sill is a solid that occupies the wall from the opening's foot down by
 * kSillThickness. The strip of wall below the opening therefore stops there, or
 * the two surfaces interpenetrate along the one edge a passer-by looks straight
 * at.
 */
TEST(OpFacade, the_surround_stops_short_of_the_sill) {
    Shape shape = make_wall(1.0, 2.0);
    OpeningParams params;
    params.inset = 0.1;
    FacadeReport report;
    const OpResult result = window_shape(shape, params, &report);
    CHECK_TRUE(result.ok);

    // Bottom strip is 0.8 wide by (0.1 - kSillThickness) high, not 0.8 by 0.1.
    CHECK_EQ(part_faces(shape, FacadePart::Panel), size_t{4});
    CHECK_NEAR(part_area(shape, FacadePart::Panel),
               0.2 + 0.2 + 0.8 * (0.1 - kSillThickness) + 0.08, 1e-12);

    // The sill's top is the opening's foot and its bottom is where the wall
    // strip stopped: the two meet exactly, with no gap and no overlap.
    glm::dvec3 sill_min{0.0};
    glm::dvec3 sill_max{0.0};
    part_world_bounds(shape, FacadePart::Sill, sill_min, sill_max);
    check_vec3(sill_min, glm::dvec3{0.1, 0.1 - kSillThickness, -0.15}, 1e-12);
    check_vec3(sill_max, glm::dvec3{0.9, 0.1, kDefaultSillProjection}, 1e-12);
}

/// The sill is a closed solid, because it is seen from below from the street
TEST(OpFacade, the_sill_is_a_closed_solid_and_projects_past_the_wall_face) {
    Shape shape = make_wall(0.6, 2.0);
    FacadeReport report;
    const OpResult result = window_shape(shape, OpeningParams{}, &report);
    CHECK_TRUE(result.ok);

    CHECK_EQ(part_faces(shape, FacadePart::Sill), size_t{6});
    // 0.6 across, kSillThickness thick, reveal + projection deep.
    const double depth = 0.15 + kDefaultSillProjection;
    CHECK_NEAR(part_area(shape, FacadePart::Sill),
               2.0 * (0.6 * kSillThickness + 0.6 * depth + kSillThickness * depth), 1e-12);

    glm::dvec3 sill_min{0.0};
    glm::dvec3 sill_max{0.0};
    part_world_bounds(shape, FacadePart::Sill, sill_min, sill_max);
    // In front of the wall plane, which is what makes a sill a sill.
    CHECK_NEAR(sill_max.z, kDefaultSillProjection, 1e-12);
    CHECK_NEAR(sill_min.z, -0.15, 1e-12);
}

// ============================================================================
// The door
// ============================================================================

/**
 * @brief A door's opening reaches the floor and a window's does not
 *
 * The one geometric difference the two operations have, and the reason a door is
 * not a window with a different material.
 */
TEST(OpFacade, a_door_opening_reaches_the_floor_and_a_window_opening_does_not) {
    OpeningParams params;
    params.inset = 0.1;

    Shape door = make_wall(1.0, 2.2);
    FacadeReport door_report;
    const OpResult door_result = door_shape(door, params, &door_report);
    CHECK_TRUE(door_result.ok);

    Shape window = make_wall(1.0, 2.2);
    FacadeReport window_report;
    const OpResult window_result = window_shape(window, params, &window_report);
    CHECK_TRUE(window_result.ok);

    // (x0, y0, x1, y1). Only y0 differs: the inset is applied to the head and
    // the two jambs, never to the foot.
    CHECK_NEAR(door_report.opening.x, 0.1, 1e-12);
    CHECK_NEAR(door_report.opening.y, 0.0, 1e-12);
    CHECK_NEAR(door_report.opening.z, 0.9, 1e-12);
    CHECK_NEAR(door_report.opening.w, 2.1, 1e-12);
    CHECK_NEAR(window_report.opening.y, 0.1, 1e-12);

    // Three strips of surround for a door, four for a window: a door has no wall
    // beneath it.
    CHECK_EQ(part_faces(door, FacadePart::Panel), size_t{3});
    CHECK_EQ(part_faces(window, FacadePart::Panel), size_t{4});
}

/// A door's frame has three members and its leaf reaches the floor between them
TEST(OpFacade, a_door_frame_has_three_members_and_the_leaf_reaches_the_floor) {
    Shape shape = make_wall(1.0, 2.2);
    OpeningParams params;
    params.inset = 0.1;
    FacadeReport report;
    const OpResult result = door_shape(shape, params, &report);
    CHECK_TRUE(result.ok);
    CHECK_EQ(report.warnings.size(), size_t{0});

    CHECK_EQ(part_faces(shape, FacadePart::Frame), size_t{3});
    // Two stiles 0.05 by 2.1 and a head 0.7 by 0.05.
    CHECK_NEAR(part_area(shape, FacadePart::Frame),
               2.0 * kDefaultFrameWidth * 2.1 + 0.7 * kDefaultFrameWidth, 1e-12);

    CHECK_EQ(part_faces(shape, FacadePart::Leaf), size_t{1});
    CHECK_EQ(part_faces(shape, FacadePart::Glass), size_t{0});
    CHECK_NEAR(part_area(shape, FacadePart::Leaf), 0.7 * 2.05, 1e-12);

    // The leaf stands on the panel's own foot, 0.15 behind the wall face.
    glm::dvec3 leaf_min{0.0};
    glm::dvec3 leaf_max{0.0};
    part_world_bounds(shape, FacadePart::Leaf, leaf_min, leaf_max);
    check_vec3(leaf_min, glm::dvec3{0.15, 0.0, -0.15}, 1e-12);
    check_vec3(leaf_max, glm::dvec3{0.85, 2.05, -0.15}, 1e-12);
}

/**
 * @brief A door's frame budget is ONE member deep, a window's is two
 *
 * The test above uses a 0.05 frame in a 2.1 m opening, where both budgets fit
 * comfortably and neither is the deciding comparison. So the door's
 * three-member rule is untested there: replace `door ? frame : 2.0 * frame` with
 * `2.0 * frame` and nothing moves.
 *
 * This is the case where the two budgets disagree. A 0.4 frame in an opening 1.0
 * wide by 0.6 high -- a heavy shopfront surround, or an author who set the frame
 * width in centimetres by mistake:
 *
 *   across:  2 x 0.4 = 0.8 fits inside 1.0, for both operations
 *   up:      a door needs 0.4 (the head alone) and 0.4 fits inside 0.6
 *            a window needs 2 x 0.4 = 0.8 and 0.8 does NOT fit inside 0.6
 *
 * So the door keeps its leaf and the window loses its glazing, on the same panel
 * with the same argument. Anything that gives the two the same budget breaks one
 * half of this test or the other.
 */
TEST(OpFacade, a_doors_frame_budget_fits_where_a_windows_does_not) {
    OpeningParams params;
    params.frame = 0.4;

    Shape door = make_wall(1.0, 0.6);
    FacadeReport door_report;
    CHECK_TRUE(door_shape(door, params, &door_report).ok);
    CHECK_FALSE(door_report.frame_filled_opening);
    CHECK_EQ(door_report.warnings.size(), size_t{0});

    // Two stiles 0.4 by 0.6 and a head 0.2 by 0.4, with a leaf in what is left.
    CHECK_EQ(part_faces(door, FacadePart::Frame), size_t{3});
    CHECK_NEAR(part_area(door, FacadePart::Frame), 2.0 * 0.4 * 0.6 + 0.2 * 0.4, 1e-12);
    CHECK_EQ(part_faces(door, FacadePart::Leaf), size_t{1});
    CHECK_NEAR(part_area(door, FacadePart::Leaf), 0.2 * 0.2, 1e-12);

    // The leaf still stands on the floor and still sits at the glazing line.
    glm::dvec3 min_corner{0.0};
    glm::dvec3 max_corner{0.0};
    part_world_bounds(door, FacadePart::Leaf, min_corner, max_corner);
    check_vec3(min_corner, glm::dvec3{0.4, 0.0, -0.15}, 1e-12);
    check_vec3(max_corner, glm::dvec3{0.6, 0.2, -0.15}, 1e-12);

    // The same panel and the same frame width, glazed instead: no glazing left.
    Shape window = make_wall(1.0, 0.6);
    FacadeReport window_report;
    CHECK_TRUE(window_shape(window, params, &window_report).ok);
    CHECK_TRUE(window_report.frame_filled_opening);
    CHECK_EQ(count_warnings(window_report, "so there is no glazing in it"), size_t{1});
    CHECK_EQ(part_faces(window, FacadePart::Glass), size_t{0});
    CHECK_EQ(part_faces(window, FacadePart::Frame), size_t{1});
    CHECK_NEAR(part_area(window, FacadePart::Frame), 1.0 * 0.6, 1e-12);
}

/**
 * @brief A door gets a threshold where a window gets a sill, and the two differ
 *
 * Counting six faces of threshold proves only that SOMETHING six-sided was built
 * under the door. A threshold built from the sill's own two constants is still
 * six faces, still under the door, still tagged Threshold -- and it is 4 cm thick
 * standing 5 cm proud, which is a sill somebody renamed.
 *
 * So both ledges are measured. The threshold is thinner than the sill
 * (kThresholdThickness 0.03 against kSillThickness 0.04) and projects less
 * (kDefaultThresholdProjection 0.03 against kDefaultSillProjection 0.05), because
 * a threshold is walked over and a sill throws water clear.
 *
 * The bounds are written as literal metres and the constants are pinned to the
 * same literals separately. Writing `-kThresholdThickness` would move the
 * expectation along with the constant, which is a test that cannot fail.
 */
TEST(OpFacade, a_door_gets_a_threshold_and_a_window_gets_a_sill) {
    CHECK_NEAR(kThresholdThickness, 0.03, 1e-12);
    CHECK_NEAR(kSillThickness, 0.04, 1e-12);
    CHECK_NEAR(kDefaultSillProjection, 0.05, 1e-12);

    Shape door = make_wall(1.0, 2.2);
    OpeningParams params;
    params.inset = 0.1;
    FacadeReport door_report;
    CHECK_TRUE(door_shape(door, params, &door_report).ok);

    CHECK_EQ(part_faces(door, FacadePart::Threshold), size_t{6});
    CHECK_EQ(part_faces(door, FacadePart::Sill), size_t{0});

    // The opening is x 0.1 .. 0.9 standing on y = 0, so the threshold hangs from
    // -0.03 to 0 and runs from the glazing line at -0.15 out to +0.03.
    glm::dvec3 min_corner{0.0};
    glm::dvec3 max_corner{0.0};
    part_world_bounds(door, FacadePart::Threshold, min_corner, max_corner);
    check_vec3(min_corner, glm::dvec3{0.1, -0.03, -0.15}, 1e-12);
    check_vec3(max_corner, glm::dvec3{0.9, 0.0, 0.03}, 1e-12);

    // 0.8 across, 0.03 thick, 0.18 deep: the closed box, both of each pair.
    CHECK_NEAR(part_area(door, FacadePart::Threshold),
               2.0 * (0.8 * 0.03 + 0.8 * 0.18 + 0.03 * 0.18), 1e-12);

    Shape window = make_wall(1.0, 2.2);
    FacadeReport window_report;
    CHECK_TRUE(window_shape(window, params, &window_report).ok);
    CHECK_EQ(part_faces(window, FacadePart::Sill), size_t{6});
    CHECK_EQ(part_faces(window, FacadePart::Threshold), size_t{0});
    CHECK_EQ(part_faces(window, FacadePart::Leaf), size_t{0});

    // The window's opening stands on y = 0.1, so its sill hangs from 0.06 to 0.1
    // and projects to +0.05: thicker and further out than the threshold, which is
    // the whole difference between the two parts.
    part_world_bounds(window, FacadePart::Sill, min_corner, max_corner);
    check_vec3(min_corner, glm::dvec3{0.1, 0.06, -0.15}, 1e-12);
    check_vec3(max_corner, glm::dvec3{0.9, 0.1, 0.05}, 1e-12);
    CHECK_NEAR(part_area(window, FacadePart::Sill),
               2.0 * (0.8 * 0.04 + 0.8 * 0.2 + 0.04 * 0.2), 1e-12);
}

/// A door has no bottom reveal return: the threshold is what the opening stands on
TEST(OpFacade, a_door_has_no_bottom_reveal_return) {
    Shape shape = make_wall(1.0, 2.2);
    OpeningParams params;
    params.inset = 0.1;
    FacadeReport report;
    CHECK_TRUE(door_shape(shape, params, &report).ok);

    const std::vector<glm::dvec3> normals = part_normals(shape, FacadePart::Reveal);
    CHECK_EQ(normals.size(), size_t{3});
    for (const glm::dvec3& normal : normals) {
        // Nothing looking straight up: that would be a return under the door.
        CHECK_TRUE(normal.y < 0.5);
    }
    CHECK_NEAR(part_area(shape, FacadePart::Reveal), 2.0 * 2.1 * 0.15 + 0.8 * 0.15, 1e-12);
}

// ============================================================================
// What a panel has to be
// ============================================================================

/// A solid has several faces and no obvious one to glaze; the message says so
TEST(OpFacade, window_refuses_a_solid_and_says_to_select_a_face_first) {
    Shape shape = shape_from_rect(2.0, 4.0);
    CHECK_TRUE(stratum::procgen::rules::extrude_shape(
                   shape, stratum::procgen::rules::ScopeAxis::Y, 3.0)
                   .ok);
    CHECK_EQ(shape.geometry.faces.size(), size_t{6});

    FacadeReport report;
    const OpResult result = window_shape(shape, OpeningParams{}, &report);
    CHECK_FALSE(result.ok);
    CHECK_TRUE(result.message.find("select face") != std::string::npos);
    CHECK_TRUE(result.message.find("has 6") != std::string::npos);
}

/**
 * @brief A gable is not a rectangle, and the surround strips would be wrong on it
 *
 * Three shapes, because the guard is three clauses and a triangle exercises only
 * one of them. A triangle has three corners, so `face.loop.size() != 4` refuses
 * it on its own and the area comparison beside it is never the deciding test.
 * The area comparison is the ONLY thing standing between window() and a
 * four-cornered non-rectangle, and a four-cornered non-rectangle is what a
 * skewed or tapered wall actually produces.
 *
 * Both of the extra cases are caught by area and by nothing else:
 *
 *   trapezoid      area 3.0 in a box 2.0 by 2.0 = 4.0
 *   parallelogram  area 4.0 in a box 2.5 by 2.0 = 5.0
 *
 * Accepting either glazes it against a bounding box it does not fill, which puts
 * the surround strips outside the panel and the glass over thin air.
 */
TEST(OpFacade, window_refuses_a_panel_that_is_not_a_rectangle) {
    Shape shape = make_triangle(1.0, 2.0);
    FacadeReport report;
    const OpResult result = window_shape(shape, OpeningParams{}, &report);
    CHECK_FALSE(result.ok);
    CHECK_TRUE(result.message.find("rectangular panel") != std::string::npos);

    Shape door = make_triangle(1.0, 2.0);
    const OpResult door_result = door_shape(door, OpeningParams{}, nullptr);
    CHECK_FALSE(door_result.ok);
    CHECK_TRUE(door_result.message.find("rectangular panel") != std::string::npos);

    // Four corners, no holes, perfectly planar: only the area clause refuses it.
    Shape trapezoid = make_quad({0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {1.5, 2.0, 0.0},
                                {0.5, 2.0, 0.0});
    const OpResult trapezoid_result = window_shape(trapezoid, OpeningParams{}, nullptr);
    CHECK_FALSE(trapezoid_result.ok);
    CHECK_TRUE(trapezoid_result.message.find("rectangular panel") != std::string::npos);
    CHECK_TRUE(trapezoid_result.message.find("4 corners") != std::string::npos);

    Shape parallelogram = make_quad({0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {2.5, 2.0, 0.0},
                                    {0.5, 2.0, 0.0});
    const OpResult parallelogram_result = window_shape(parallelogram, OpeningParams{}, nullptr);
    CHECK_FALSE(parallelogram_result.ok);
    CHECK_TRUE(parallelogram_result.message.find("rectangular panel") != std::string::npos);

    // A door refuses the same four-cornered panel for the same reason: the two
    // operations share the guard, and testing one of them is testing neither.
    Shape skewed_door = make_quad({0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {2.5, 2.0, 0.0},
                                  {0.5, 2.0, 0.0});
    const OpResult skewed_door_result = door_shape(skewed_door, OpeningParams{}, nullptr);
    CHECK_FALSE(skewed_door_result.ok);
    CHECK_TRUE(skewed_door_result.message.find("rectangular panel") != std::string::npos);

    // The control. Without it, every assertion above passes for a window() that
    // refuses every panel it is ever handed.
    Shape rectangle = make_quad({0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {2.0, 2.0, 0.0},
                                {0.0, 2.0, 0.0});
    CHECK_TRUE(window_shape(rectangle, OpeningParams{}, nullptr).ok);
}

/**
 * @brief A panel bent out of its own plane is refused, and says by how much
 *
 * op_comp.hpp's `select face` can hand over a quad whose four corners are not
 * coplanar -- an extruded non-planar footprint is the ordinary way to get one --
 * and a bent quad has no single plane to cut an opening in. Every part of the
 * assembly is placed at a constant local z, so glazing one would put the frame
 * through the wall on one side and short of it on the other.
 *
 * One millimetre of bend across a 2 m panel, which is far too little to see and
 * far more than the 1e-9 relative tolerance. The planar control on the same four
 * corners is what stops this passing for an operation that refuses everything.
 */
TEST(OpFacade, window_refuses_a_panel_that_is_bent_out_of_plane) {
    Shape bent = make_quad({0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {2.0, 2.0, 0.001},
                           {0.0, 2.0, 0.0});
    const OpResult result = window_shape(bent, OpeningParams{}, nullptr);
    CHECK_FALSE(result.ok);
    CHECK_TRUE(result.message.find("needs a flat panel") != std::string::npos);
    CHECK_TRUE(result.message.find("out of plane") != std::string::npos);

    // orient_panel() is where the guard lives, and it is public precisely so the
    // reason can be read without a window in the way.
    Shape again = make_quad({0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {2.0, 2.0, 0.001},
                            {0.0, 2.0, 0.0});
    std::string why;
    CHECK_FALSE(orient_panel(again, why));
    CHECK_TRUE(why.find("out of plane") != std::string::npos);

    // A door goes through the same orient_panel(), so it refuses the same quad.
    Shape bent_door = make_quad({0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {2.0, 2.0, 0.001},
                                {0.0, 2.0, 0.0});
    CHECK_FALSE(door_shape(bent_door, OpeningParams{}, nullptr).ok);

    // The control: the same four corners, flat. This one glazes.
    Shape flat = make_quad({0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {2.0, 2.0, 0.0},
                           {0.0, 2.0, 0.0});
    std::string flat_why;
    CHECK_TRUE(orient_panel(flat, flat_why));
    CHECK_EQ(flat_why, std::string{});

    Shape glazed = make_quad({0.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {2.0, 2.0, 0.0},
                             {0.0, 2.0, 0.0});
    CHECK_TRUE(window_shape(glazed, OpeningParams{}, nullptr).ok);
    CHECK_EQ(part_faces(glazed, FacadePart::Glass), size_t{1});
}

TEST(OpFacade, window_refuses_a_shape_with_nothing_to_work_on) {
    Shape shape;
    const OpResult result = window_shape(shape, OpeningParams{}, nullptr);
    CHECK_FALSE(result.ok);
    CHECK_TRUE(result.message.find("has none") != std::string::npos);
}

// ============================================================================
// wall_panel
// ============================================================================

/**
 * @brief A panel thickens INTO the wall, never out of it
 *
 * Out of it would put the wall in front of the windows in the tiles beside it,
 * which is a mistake that looks like a lighting bug.
 */
TEST(OpFacade, wall_panel_thickens_into_the_wall_and_not_out_of_it) {
    Shape shape = make_wall(1.0, 2.0);
    FacadeReport report;
    const OpResult result = wall_panel_shape(shape, 0.0, 0.2, &report);
    CHECK_TRUE(result.ok);

    CHECK_EQ(shape.geometry.faces.size(), size_t{6});
    CHECK_NEAR(geometry_volume(shape.geometry), 1.0 * 2.0 * 0.2, 1e-12);
    CHECK_NEAR(geometry_area(shape.geometry),
               2.0 * (1.0 * 2.0) + 2.0 * (1.0 * 0.2) + 2.0 * (2.0 * 0.2), 1e-12);

    // The original face stays where it was and the slab grows behind it.
    glm::dvec3 min_corner{0.0};
    glm::dvec3 max_corner{0.0};
    part_world_bounds(shape, FacadePart::Panel, min_corner, max_corner);
    check_vec3(min_corner, glm::dvec3{0.0, 0.0, -0.2}, 1e-12);
    check_vec3(max_corner, glm::dvec3{1.0, 2.0, 0.0}, 1e-12);
}

TEST(OpFacade, wall_panel_insets_its_outline_and_records_the_panel_extent) {
    Shape shape = make_wall(1.0, 2.0);
    FacadeReport report;
    const OpResult result = wall_panel_shape(shape, 0.1, 0.2, &report);
    CHECK_TRUE(result.ok);

    CHECK_NEAR(report.panel_width, 0.8, 1e-9);
    CHECK_NEAR(report.panel_height, 1.8, 1e-9);
    CHECK_NEAR(geometry_volume(shape.geometry), 0.8 * 1.8 * 0.2, 1e-9);

    // The pair D8 cannot recover once a sill or a thickness has grown the scope.
    const auto width = shape.attributes.find(kPanelWidthAttribute);
    const auto height = shape.attributes.find(kPanelHeightAttribute);
    CHECK_TRUE(width != shape.attributes.end());
    CHECK_TRUE(height != shape.attributes.end());
    if (width != shape.attributes.end() && height != shape.attributes.end()) {
        CHECK_TRUE(width->second.is_number());
        CHECK_TRUE(height->second.is_number());
        CHECK_NEAR(width->second.as_number(), 0.8, 1e-9);
        CHECK_NEAR(height->second.as_number(), 1.8, 1e-9);
    }
}

TEST(OpFacade, a_bare_wall_panel_keeps_its_face_and_tags_it) {
    Shape shape = make_wall(1.0, 2.0);
    FacadeReport report;
    const OpResult result = wall_panel_shape(shape, 0.0, 0.0, &report);
    CHECK_TRUE(result.ok);

    CHECK_EQ(shape.geometry.faces.size(), size_t{1});
    CHECK_EQ(part_faces(shape, FacadePart::Panel), size_t{1});
    CHECK_NEAR(geometry_area(shape.geometry), 2.0, 1e-12);
    check_vec3(shape.scope.size, glm::dvec3{1.0, 2.0, 0.0}, 1e-12);
    CHECK_TRUE(facade_material(FacadePart::Panel).material == MaterialId::Wall);
}

/// wall_panel never cuts anything, so it takes the gable that window refuses
TEST(OpFacade, wall_panel_takes_a_triangle_that_window_refuses) {
    Shape panel = make_triangle(1.0, 2.0);
    const OpResult result = wall_panel_shape(panel, 0.0, 0.0, nullptr);
    CHECK_TRUE(result.ok);
    CHECK_EQ(panel.geometry.faces.size(), size_t{1});
    CHECK_NEAR(geometry_area(panel.geometry), 1.0, 1e-12);

    Shape glazed = make_triangle(1.0, 2.0);
    CHECK_FALSE(window_shape(glazed, OpeningParams{}, nullptr).ok);
}

/**
 * @brief A panel with a hole in it keeps its hole
 *
 * orient_panel() rebuilds the shape from one face through extract_face(), which
 * copies the outer loop AND the hole rings. Dropping the holes there is invisible
 * to everything else this file measures: the corner count, the bounds, the scope,
 * the normal and the face count are all identical with the hole and without it.
 * Only the AREA differs, so only the area can catch it.
 *
 * The case is real rather than hypothetical. A hole in a wall panel is what a
 * lightwell, a vent or an arch cut by an earlier operation leaves behind, and
 * wall_panel() is the operation that has to survive one, because it is the one
 * that does not require a rectangle.
 */
TEST(OpFacade, wall_panel_keeps_the_hole_in_its_panel) {
    Shape shape = make_wall_with_hole(2.0, 1.0);
    CHECK_NEAR(geometry_area(shape.geometry), 4.0 - 1.0, 1e-12);

    FacadeReport report;
    const OpResult result = wall_panel_shape(shape, 0.0, 0.0, &report);
    CHECK_TRUE(result.ok);

    CHECK_EQ(shape.geometry.faces.size(), size_t{1});
    if (shape.geometry.faces.size() == 1) {
        CHECK_EQ(shape.geometry.faces[0].holes.size(), size_t{1});
        CHECK_EQ(shape.geometry.faces[0].loop.size(), size_t{4});
    }
    // 2 by 2 less a 1 by 1 hole. A dropped hole gives 4.0 here and nothing else.
    CHECK_NEAR(geometry_area(shape.geometry), 3.0, 1e-12);
    CHECK_EQ(part_faces(shape, FacadePart::Panel), size_t{1});

    // The outline is untouched by the hole: still the whole 2 by 2 tile.
    check_vec3(shape.scope.size, glm::dvec3{2.0, 2.0, 0.0}, 1e-12);
    CHECK_NEAR(report.panel_width, 2.0, 1e-12);
    CHECK_NEAR(report.panel_height, 2.0, 1e-12);
}

/**
 * @brief An inset that would consume the panel keeps the panel and warns
 *
 * The same author mistake window() answers with a blank panel and a warning: an
 * inset that is too big for the tile it landed on, because the inset came from an
 * attribute and the tile is the narrow one. The two operations used to disagree,
 * and the disagreement was expensive -- `wall_panel(0.5)` on a 0.4 m pier failed
 * the shape, so the terminal vanished, the generation reported ok() == false, and
 * the facade the author needed to look at to find the mistake was the one thing
 * they could not get.
 *
 * The un-inset panel is what survives, which is the panel a bare `wall_panel()`
 * would have made.
 */
TEST(OpFacade, a_wall_panel_inset_that_would_empty_the_panel_keeps_it_and_warns) {
    Shape shape = make_wall(1.0, 2.0);
    FacadeReport report;
    const OpResult result = wall_panel_shape(shape, 0.5, 0.0, &report);

    CHECK_TRUE(result.ok);
    CHECK_TRUE(report.inset_dropped);
    CHECK_EQ(report.warnings.size(), size_t{1});
    CHECK_EQ(count_warnings(report, "so the panel was left whole"), size_t{1});
    CHECK_EQ(count_warnings(report, "cannot inset this panel by 0.5"), size_t{1});

    // The whole tile is still there, tagged, and its extent was recorded from the
    // panel that survived rather than from the one that did not.
    CHECK_EQ(shape.geometry.faces.size(), size_t{1});
    CHECK_EQ(part_faces(shape, FacadePart::Panel), size_t{1});
    CHECK_NEAR(geometry_area(shape.geometry), 2.0, 1e-12);
    check_vec3(shape.scope.size, glm::dvec3{1.0, 2.0, 0.0}, 1e-12);
    CHECK_NEAR(report.panel_width, 1.0, 1e-12);
    CHECK_NEAR(report.panel_height, 2.0, 1e-12);

    // A thickness asked for alongside the dropped inset is still applied: one
    // argument failing does not cancel the other.
    Shape slab = make_wall(1.0, 2.0);
    FacadeReport slab_report;
    CHECK_TRUE(wall_panel_shape(slab, 0.5, 0.2, &slab_report).ok);
    CHECK_TRUE(slab_report.inset_dropped);
    CHECK_NEAR(geometry_volume(slab.geometry), 1.0 * 2.0 * 0.2, 1e-9);

    // The control: an inset the panel CAN take is still applied, so the warning
    // above is not simply wall_panel() refusing to inset anything at all.
    Shape inset = make_wall(1.0, 2.0);
    FacadeReport inset_report;
    CHECK_TRUE(wall_panel_shape(inset, 0.1, 0.0, &inset_report).ok);
    CHECK_FALSE(inset_report.inset_dropped);
    CHECK_EQ(inset_report.warnings.size(), size_t{0});
    CHECK_NEAR(geometry_area(inset.geometry), 0.8 * 1.8, 1e-9);
}

/**
 * @brief A rule file's over-wide wall_panel inset is a Warning, and the tile lives
 *
 * The end-to-end half of the test above, and the shape the defect really took: a
 * facade split into a narrow pier and a wide bay, with one inset written for the
 * bay. The pier must still be in the output and the generation must still be ok().
 */
TEST(OpFacade, an_over_wide_wall_panel_inset_does_not_delete_its_tile) {
    const std::string source =
        "@start\n"
        "rule Main {\n"
        "    extrude(3.0);\n"
        "    select face { front : { Wall(); } }\n"
        "}\n"
        "rule Wall { split(x) { 0.4 : { Pier(); } ~1.0 : { Bay(); } } }\n"
        "rule Pier { wall_panel(0.5); }\n"
        "rule Bay  { wall_panel(0.5); }\n";

    // A 2 by 4 rectangle extruded 3: the front face is 2 wide, split 0.4 and 1.6.
    const GenerationResult result = run_on(source, shape_from_rect(2.0, 4.0));

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.terminals.size(), size_t{2});
    CHECK_EQ(terminals_named(result, "Pier").size(), size_t{1});
    CHECK_EQ(terminals_named(result, "Bay").size(), size_t{1});

    // Exactly one Warning, on the pier alone: the bay is 1.6 wide and takes the
    // 0.5 inset, so this is not a rule that warns about everything.
    CHECK_EQ(result.diagnostics.size(), size_t{1});
    CHECK_EQ(count_messages(result, Severity::Warning, "so the panel was left whole"), size_t{1});

    const std::vector<const Shape*> pier = terminals_named(result, "Pier");
    if (pier.size() == 1) {
        CHECK_NEAR(geometry_area(pier[0]->geometry), 0.4 * 3.0, 1e-9);
    }
    const std::vector<const Shape*> bay = terminals_named(result, "Bay");
    if (bay.size() == 1) {
        CHECK_NEAR(geometry_area(bay[0]->geometry), (1.6 - 1.0) * (3.0 - 1.0), 1e-9);
    }
}

// ============================================================================
// The frame the operation works in
// ============================================================================

/**
 * @brief The panel frame is taken from the face normal, not from the world
 *
 * A footprint faces +y, so "out of the wall" is world up and "up the wall" is
 * world z. The reveal then cuts DOWNWARD, which is what makes a rooflight out of
 * the same operation and is the reason the frame is derived rather than assumed.
 */
TEST(OpFacade, the_panel_frame_is_taken_from_the_face_and_not_from_the_world) {
    Shape shape = shape_from_rect(1.0, 1.0);  // an xz footprint: normal +y
    FacadeReport report;
    const OpResult result = window_shape(shape, OpeningParams{}, &report);

    CHECK_TRUE(result.ok);
    CHECK_NEAR(report.panel_width, 1.0, 1e-12);
    CHECK_NEAR(report.panel_height, 1.0, 1e-12);

    // z of the panel frame is the face normal, which is world up here.
    check_vec3(shape.scope.axes[2], glm::dvec3{0.0, 1.0, 0.0}, 1e-12);

    // So the glass sits 0.15 BELOW the footprint plane, not behind it in z.
    check_vec3(part_world_centroid(shape, FacadePart::Glass),
               glm::dvec3{0.5, -0.15, 0.5}, 1e-12);
}

/// orient_panel is the shared first step, and it refuses what the operations refuse
TEST(OpFacade, orient_panel_refuses_a_shape_with_more_than_one_face) {
    Shape shape = shape_from_rect(2.0, 4.0);
    CHECK_TRUE(stratum::procgen::rules::extrude_shape(
                   shape, stratum::procgen::rules::ScopeAxis::Y, 3.0)
                   .ok);
    std::string why;
    CHECK_FALSE(orient_panel(shape, why));
    CHECK_TRUE(why.find("select face") != std::string::npos);

    Shape wall = make_wall(1.0, 2.0);
    std::string ok_why;
    CHECK_TRUE(orient_panel(wall, ok_why));
    CHECK_EQ(ok_why, std::string{});
    check_vec3(wall.scope.size, glm::dvec3{1.0, 2.0, 0.0}, 1e-12);
}

// ============================================================================
// The scope afterwards
// ============================================================================

/**
 * @brief The box after the call contains everything the call built
 *
 * shape.hpp's invariant 2, restated for this operation: a later `split` has to
 * cut real geometry, and it cannot if the box is still the flat tile the window
 * was cut into.
 */
TEST(OpFacade, the_scope_afterwards_contains_the_whole_window) {
    Shape shape = make_wall(0.6, 2.0);
    FacadeReport report;
    CHECK_TRUE(window_shape(shape, OpeningParams{}, &report).ok);

    // Width unchanged, height grown by the sill's overhang below the opening,
    // depth grown from nothing to the reveal plus the sill's projection.
    check_vec3(shape.scope.size,
               glm::dvec3{0.6, 2.0 + kSillThickness, 0.15 + kDefaultSillProjection}, 1e-12);
    check_vec3(shape.scope.origin,
               glm::dvec3{0.0, -kSillThickness, -0.15}, 1e-12);

    // And the box is tight: every vertex is inside it, and it touches every side.
    glm::dvec3 min_corner{1.0e300};
    glm::dvec3 max_corner{-1.0e300};
    for (const glm::dvec3& local : shape.geometry.positions) {
        min_corner = glm::min(min_corner, local);
        max_corner = glm::max(max_corner, local);
    }
    check_vec3(min_corner, glm::dvec3{0.0}, 1e-12);
    check_vec3(max_corner, shape.scope.size, 1e-12);
}

/**
 * @brief A `split(y)` after a window is measured against the GROWN box
 *
 * op_facade.hpp states this as part of the contract, and it is the one part that
 * is otherwise silent: the sill hangs kSillThickness below the opening, the refit
 * follows it down, and every later cut on the y axis moves with it. Nothing else
 * in this file measures a cut made AFTER an opening, so without this test the
 * offset could be any number at all and only a rule author would ever find out.
 *
 * Three runs on the identical 2 by 3 front face, with identical split ratios:
 *
 *   split alone     cuts at world y = 1.00, and High is 2.00 high
 *   window; split   cuts at world y = 0.96, and High is 2.04 high
 *   door;   split   cuts at world y = 0.97, and High is 2.03 high
 *
 * The control is what makes the other two mean something: it is the same rule
 * file with the opening taken out, so the 0.04 and the 0.03 are differences
 * attributable to the sill and the threshold and to nothing else. The literal
 * metres are written out rather than derived from kSillThickness, so changing the
 * constant fails this test instead of moving its expectation.
 */
TEST(OpFacade, a_split_after_an_opening_is_measured_against_the_grown_scope) {
    const char* const kShell =
        "@start\n"
        "rule Main { extrude(3.0); select face { front : { Tile(); } } }\n"
        "rule Low { }\n"
        "rule High { }\n";
    const char* const kSplit = "rule Tile { %ssplit(y) { 1.0 : { Low(); } ~1.0 : { High(); } } }\n";

    // (what the rule does first, where the cut lands, how high High ends up)
    struct Case {
        const char* opening;
        double cut;
        double high_height;
    };
    const Case cases[] = {
        {"", 1.0, 2.0},           // the control: no opening, no growth
        {"window(); ", 0.96, 2.04},  // the sill hangs 0.04 below the panel foot
        {"door(); ", 0.97, 2.03},    // the threshold hangs 0.03 below it
    };

    for (const Case& one : cases) {
        char body[256];
        std::snprintf(body, sizeof(body), kSplit, one.opening);
        const GenerationResult result =
            run_on(std::string{kShell} + body, shape_from_rect(2.0, 4.0));

        CHECK_TRUE(result.ok());
        CHECK_EQ(result.diagnostics.size(), size_t{0});
        CHECK_EQ(result.terminals.size(), size_t{2});

        const std::vector<const Shape*> low = terminals_named(result, "Low");
        const std::vector<const Shape*> high = terminals_named(result, "High");
        CHECK_EQ(low.size(), size_t{1});
        CHECK_EQ(high.size(), size_t{1});
        if (low.size() != 1 || high.size() != 1) {
            continue;
        }

        glm::dvec3 min_corner{0.0};
        glm::dvec3 max_corner{0.0};

        // The panel's own foot is world y = 0. Low starts BELOW it by exactly the
        // ledge, keeps the 1.0 it asked for, and therefore stops short of 1.0.
        scope_world_bounds(*low[0], min_corner, max_corner);
        CHECK_NEAR(min_corner.y, one.cut - 1.0, 1e-12);
        CHECK_NEAR(max_corner.y, one.cut, 1e-12);
        CHECK_NEAR(low[0]->scope.size.y, 1.0, 1e-12);

        // High takes the remainder, which is the panel plus the ledge less 1.0.
        scope_world_bounds(*high[0], min_corner, max_corner);
        CHECK_NEAR(min_corner.y, one.cut, 1e-12);
        CHECK_NEAR(max_corner.y, 3.0, 1e-12);
        CHECK_NEAR(high[0]->scope.size.y, one.high_height, 1e-12);
    }
}

/// Every part is a distinct material variant, and the variant MEANS what the part is
TEST(OpFacade, each_part_carries_the_material_variant_that_describes_it) {
    // This used to assert Glass == Wall/3, which was the bug rather than the
    // contract. The Wall slot's variants are a published vocabulary --
    // osm/road/road_style.hpp names them, and the OSM importer has written them
    // for every mapped building since P0.3 -- and variant 3 is kWallConcrete.
    //
    // Casting the part ordinal put the facade parts on top of that vocabulary:
    // glazing asked for concrete, a frame for stone, a sill for render. Nothing
    // is bound in ShaderMode::Simple, so it never showed.
    CHECK_TRUE(facade_material(FacadePart::Glass) ==
               (MaterialKey{MaterialId::Wall, stratum::osm::road::variants::kWallGlass}));
    CHECK_TRUE(facade_material(FacadePart::Panel) ==
               (MaterialKey{MaterialId::Wall, stratum::osm::road::variants::kWallDefault}));
    CHECK_TRUE(facade_material(FacadePart::Frame) ==
               (MaterialKey{MaterialId::Wall, stratum::osm::road::variants::kWallMetal}));
    CHECK_TRUE(facade_material(FacadePart::Sill) ==
               (MaterialKey{MaterialId::Wall, stratum::osm::road::variants::kWallStone}));

    // Still INJECTIVE, which is load-bearing beyond correctness: the helpers in
    // this file find a part's faces through facade_material(), so two parts
    // sharing a key would make them indistinguishable and quietly weaken every
    // part_faces() assertion below.
    static const FacadePart kAll[] = {FacadePart::Panel, FacadePart::Reveal,
                                      FacadePart::Frame, FacadePart::Glass,
                                      FacadePart::Sill,  FacadePart::Leaf,
                                      FacadePart::Threshold};
    for (const FacadePart a : kAll) {
        for (const FacadePart b : kAll) {
            if (a == b) continue;
            CHECK_TRUE(facade_material(a) != facade_material(b));
        }
    }

    // And kWallBrick stays free: brick is a decision about a whole building,
    // not about a window part, which is what lets a rule say
    // `material("wall", 1)` and mean it.
    for (const FacadePart part : kAll) {
        CHECK_TRUE(facade_material(part).variant != stratum::osm::road::variants::kWallBrick);
    }

    CHECK_EQ(std::string{facade_part_name(FacadePart::Glass)}, std::string{"glass"});

    Shape shape = make_wall(0.6, 2.0);
    CHECK_TRUE(window_shape(shape, OpeningParams{}, nullptr).ok);

    // 0 surround + 3 returns + 4 frame members + 1 pane + 6 sill faces.
    CHECK_EQ(shape.geometry.faces.size(), size_t{14});
    CHECK_EQ(part_faces(shape, FacadePart::Panel), size_t{0});
    CHECK_EQ(part_faces(shape, FacadePart::Reveal), size_t{3});
    CHECK_EQ(part_faces(shape, FacadePart::Frame), size_t{4});
    CHECK_EQ(part_faces(shape, FacadePart::Glass), size_t{1});
    CHECK_EQ(part_faces(shape, FacadePart::Sill), size_t{6});
    CHECK_NEAR(geometry_area(shape.geometry), 2.194, 1e-12);

    // ---- and every one of the fourteen faces is where the reveal puts it ----
    //
    // Counting the four frame members and totalling their area says nothing at
    // all about depth: the identical count and the identical total come out of a
    // frame drawn flat on the wall, 0.15 in front of the glass it is supposed to
    // hold. The wall face is the plane world z = 0, so:
    //
    //   frame and glass  sit ON the glazing line, at z = -0.15 exactly;
    //   the returns      BRIDGE the glazing line and the wall face, -0.15 to 0;
    //   the sill         runs from the glazing line out past the wall face.
    //
    // The numbers are the literal metres a reader can derive from the default
    // 0.3 wall, not kDefaultWallThickness * kDefaultRevealFraction, so that
    // moving a constant cannot move the expectation with it.
    check_part_plane_z(shape, FacadePart::Frame, -0.15, 1e-12);
    check_part_plane_z(shape, FacadePart::Glass, -0.15, 1e-12);

    glm::dvec3 min_corner{0.0};
    glm::dvec3 max_corner{0.0};
    part_world_bounds(shape, FacadePart::Reveal, min_corner, max_corner);
    CHECK_NEAR(min_corner.z, -0.15, 1e-12);
    CHECK_NEAR(max_corner.z, 0.0, 1e-12);

    part_world_bounds(shape, FacadePart::Sill, min_corner, max_corner);
    CHECK_NEAR(min_corner.z, -0.15, 1e-12);
    CHECK_NEAR(max_corner.z, kDefaultSillProjection, 1e-12);
}

// ============================================================================
// Through the rule language
// ============================================================================

/**
 * @brief Registering the handler is what makes the catalogue row run
 *
 * ast.hpp has carried a `window` row since D1, so the name parses either way.
 * Without register_facade_operations() it reports "not implemented" at the line
 * that called it, which is this suite's proof that it is testing wiring and not
 * a table something else already filled in.
 */
TEST(OpFacade, window_runs_only_because_the_handler_is_registered) {
    const std::string source = "@start\nrule Main { window(); }\n";
    const ParseResult parsed = parse(source, "test.srl");
    CHECK_EQ(parsed.render_all(source), std::string{});

    const GenerationResult without = generate(parsed.file, shape_from_rect(1.0, 1.0));
    CHECK_FALSE(without.ok());
    CHECK_EQ(count_messages(without, Severity::Error,
                            "operation 'window' is not implemented in this build"),
             size_t{1});

    const GenerationResult with = generate(parsed.file, shape_from_rect(1.0, 1.0), {},
                                           &facade_operations(), nullptr);
    CHECK_TRUE(with.ok());
    CHECK_EQ(with.diagnostics.size(), size_t{0});
    CHECK_EQ(with.terminals.size(), size_t{1});
    if (with.terminals.size() == 1) {
        CHECK_EQ(part_faces(with.terminals[0], FacadePart::Glass), size_t{1});
    }
}

/// The three operations reached from a rule file, on a genuine `select face` panel
TEST(OpFacade, a_rule_file_reaches_window_door_and_wall_panel) {
    const std::string source =
        "@start\n"
        "rule Main {\n"
        "    extrude(3.0);\n"
        "    select face {\n"
        "        front : { Glazed(); }\n"
        "        back  : { Doored(); }\n"
        "        left  : { Blank(); }\n"
        "    }\n"
        "}\n"
        "rule Glazed { window(); }\n"
        "rule Doored { door(); }\n"
        "rule Blank  { wall_panel(0.0, 0.25); }\n";

    // The rectangle 2 by 4 extruded 3 up: the front face is z = 4, 2 wide by 3 high.
    const GenerationResult result = run_on(source, shape_from_rect(2.0, 4.0));
    CHECK_TRUE(result.ok());
    CHECK_EQ(result.diagnostics.size(), size_t{0});
    CHECK_EQ(result.terminals.size(), size_t{3});

    const std::vector<const Shape*> glazed = terminals_named(result, "Glazed");
    CHECK_EQ(glazed.size(), size_t{1});
    if (glazed.size() == 1) {
        CHECK_EQ(part_faces(*glazed[0], FacadePart::Glass), size_t{1});
        // 2 - 2 * 0.05 wide by 3 - 2 * 0.05 high, 0.15 in front of z = 4.
        CHECK_NEAR(part_area(*glazed[0], FacadePart::Glass), 1.9 * 2.9, 1e-12);
        CHECK_NEAR(part_world_centroid(*glazed[0], FacadePart::Glass).z, 4.0 - 0.15, 1e-12);
    }

    const std::vector<const Shape*> doored = terminals_named(result, "Doored");
    CHECK_EQ(doored.size(), size_t{1});
    if (doored.size() == 1) {
        CHECK_EQ(part_faces(*doored[0], FacadePart::Leaf), size_t{1});
        CHECK_EQ(part_faces(*doored[0], FacadePart::Threshold), size_t{6});
        // The back face is z = 0, so its outward normal is -z and the reveal
        // cuts the other way: the leaf is 0.15 on the +z side of it.
        CHECK_NEAR(part_world_centroid(*doored[0], FacadePart::Leaf).z, 0.15, 1e-12);
    }

    const std::vector<const Shape*> blank = terminals_named(result, "Blank");
    CHECK_EQ(blank.size(), size_t{1});
    if (blank.size() == 1) {
        CHECK_NEAR(geometry_volume(blank[0]->geometry), 4.0 * 3.0 * 0.25, 1e-9);
    }
}

/**
 * @brief The end-to-end facade, now with windows that have depth
 *
 * The rule file and every number about its layout come from
 * tests/procgen/test_rule_statements.cpp, which derives them by hand from the
 * rule text. What is added here is the third coordinate: each of the twelve
 * window centres, which that file asserts at z = 4, now has its GLASS at
 * z = 3.85, one default reveal behind the wall.
 */
TEST(OpFacade, the_end_to_end_facade_now_has_windows_with_reveals_in_it) {
    const GenerationResult result = run_on(kFacadeSource, shape_from_rect(6.0, 4.0));

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.diagnostics.size(), size_t{0});

    // The layout is untouched: 1 shopfront + 3 floors x 4 bays x 3 parts.
    CHECK_EQ(result.terminals.size(), size_t{37});
    CHECK_EQ(result.stats.shapes_created, uint32_t{107});
    CHECK_EQ(terminals_named(result, "Pier").size(), size_t{24});

    const std::vector<const Shape*> windows = terminals_named(result, "Window");
    CHECK_EQ(windows.size(), size_t{12});

    // Every glass centre, computed by hand: the tile centre of the split, less
    // one reveal along the facade's outward normal.
    std::vector<bool> matched(windows.size(), false);
    size_t found = 0;
    for (int floor = 0; floor < 3; ++floor) {
        for (int bay = 0; bay < 4; ++bay) {
            const glm::dvec3 expected{1.5 * static_cast<double>(bay) + 0.7,
                                      4.0 + 2.0 * static_cast<double>(floor) + 1.0,
                                      4.0 - 0.15};
            for (size_t i = 0; i < windows.size(); ++i) {
                if (matched[i]) {
                    continue;
                }
                const glm::dvec3 centre = part_world_centroid(*windows[i], FacadePart::Glass);
                if (std::fabs(centre.x - expected.x) < 1e-9 &&
                    std::fabs(centre.y - expected.y) < 1e-9 &&
                    std::fabs(centre.z - expected.z) < 1e-9) {
                    matched[i] = true;
                    ++found;
                    break;
                }
            }
        }
    }
    CHECK_EQ(found, size_t{12});

    // Each pane is 0.5 by 1.9 of glass, and each window has the depth to prove
    // it is not drawn on the wall.
    //
    // The glass centres above pin ONE of the fourteen faces of each window. The
    // other thirteen may sit anywhere in z as far as that loop is concerned, and
    // a frame left on the wall plane is a frame floating 0.15 in front of its own
    // glazing -- which is a facade with no reveal at all, wearing a picture of
    // one. So every part of every window is pinned here, by the plane it belongs
    // to. The outer wall is world z = 4.0 and the glazing line is 3.85.
    double glass = 0.0;
    for (const Shape* window : windows) {
        CHECK_EQ(window->geometry.faces.size(), size_t{14});
        CHECK_NEAR(part_area(*window, FacadePart::Glass), 0.5 * 1.9, 1e-12);
        CHECK_NEAR(window->scope.size.z, 0.15 + kDefaultSillProjection, 1e-12);
        glass += part_area(*window, FacadePart::Glass);

        check_part_plane_z(*window, FacadePart::Frame, 3.85, 1e-9);
        check_part_plane_z(*window, FacadePart::Glass, 3.85, 1e-9);

        glm::dvec3 min_corner{0.0};
        glm::dvec3 max_corner{0.0};

        // The three returns bridge the glazing line and the wall face. A return
        // that did not would be a reveal with no sides to catch the light.
        part_world_bounds(*window, FacadePart::Reveal, min_corner, max_corner);
        CHECK_NEAR(min_corner.z, 3.85, 1e-9);
        CHECK_NEAR(max_corner.z, 4.0, 1e-9);

        // The sill starts at the glazing line and finishes PAST the wall face.
        part_world_bounds(*window, FacadePart::Sill, min_corner, max_corner);
        CHECK_NEAR(min_corner.z, 3.85, 1e-9);
        CHECK_NEAR(max_corner.z, 4.0 + kDefaultSillProjection, 1e-9);
    }
    CHECK_NEAR(glass, 12.0 * 0.95, 1e-12);

    // The shopfront took a door with a 0.3 inset: 6 wide by 4 high, so the
    // opening is 5.4 by 3.7 and it stands on the pavement.
    const std::vector<const Shape*> shopfront = terminals_named(result, "Shopfront");
    CHECK_EQ(shopfront.size(), size_t{1});
    if (shopfront.size() == 1) {
        CHECK_EQ(part_faces(*shopfront[0], FacadePart::Leaf), size_t{1});
        CHECK_NEAR(part_area(*shopfront[0], FacadePart::Leaf),
                   (5.4 - 2.0 * kDefaultFrameWidth) * (3.7 - kDefaultFrameWidth), 1e-12);
        glm::dvec3 leaf_min{0.0};
        glm::dvec3 leaf_max{0.0};
        part_world_bounds(*shopfront[0], FacadePart::Leaf, leaf_min, leaf_max);
        CHECK_NEAR(leaf_min.y, 0.0, 1e-12);  // on the ground, which is what a door does
        CHECK_NEAR(leaf_min.z, 4.0 - 0.15, 1e-12);

        // And the shopfront's own parts sit on the same two planes as the
        // windows': leaf and frame at the glazing line, surround on the wall.
        check_part_plane_z(*shopfront[0], FacadePart::Leaf, 3.85, 1e-9);
        check_part_plane_z(*shopfront[0], FacadePart::Frame, 3.85, 1e-9);
        check_part_plane_z(*shopfront[0], FacadePart::Panel, 4.0, 1e-9);
    }

    // The piers are the wall plane itself: an inset of zero and no thickness, so
    // nothing moved them off it.
    for (const Shape* pier : terminals_named(result, "Pier")) {
        check_part_plane_z(*pier, FacadePart::Panel, 4.0, 1e-9);
    }

    // The piers are wall, and nothing else got tagged as glazing.
    for (const Shape* pier : terminals_named(result, "Pier")) {
        CHECK_EQ(pier->geometry.faces.size(), size_t{1});
        CHECK_EQ(part_faces(*pier, FacadePart::Panel), size_t{1});
        CHECK_EQ(part_faces(*pier, FacadePart::Glass), size_t{0});
    }
}

/// A clipped opening is reported at the line that asked, once, and is not fatal
TEST(OpFacade, a_clipped_opening_is_reported_once_at_the_line_that_asked) {
    const std::string source =
        "@start\n"
        "rule Main {\n"
        "    extrude(3.0);\n"
        "    select face { front : { Glazed(); } }\n"
        "}\n"
        "rule Glazed { window(-0.5); }\n";

    const GenerationResult result = run_on(source, shape_from_rect(2.0, 4.0));

    // A Warning, so the facade still builds and ok() is still true.
    CHECK_TRUE(result.ok());
    CHECK_EQ(result.diagnostics.size(), size_t{1});
    CHECK_EQ(count_messages(result, Severity::Warning, "does not fit a panel 2 by 3"),
             size_t{1});
    CHECK_EQ(result.terminals.size(), size_t{1});
    if (result.terminals.size() == 1) {
        CHECK_EQ(part_faces(result.terminals[0], FacadePart::Glass), size_t{1});
    }
}

/// A length a rule file cannot mean is an Error at that line, not a clamp
TEST(OpFacade, a_negative_reveal_from_a_rule_file_fails_the_shape) {
    const std::string source =
        "@start\n"
        "rule Main {\n"
        "    extrude(3.0);\n"
        "    select face { front : { Glazed(); } }\n"
        "}\n"
        "rule Glazed { window(0.0, -1.0); }\n";

    const GenerationResult result = run_on(source, shape_from_rect(2.0, 4.0));
    CHECK_FALSE(result.ok());
    CHECK_EQ(count_messages(result, Severity::Error,
                            "'window' wants a reveal of zero or more, not -1"),
             size_t{1});
    // The subtree is abandoned, so the glazed face produced nothing.
    CHECK_EQ(result.terminals.size(), size_t{0});
}

TEST(OpFacade, a_non_positive_wall_thickness_from_a_rule_file_fails_the_shape) {
    const std::string source =
        "@start\n"
        "rule Main {\n"
        "    extrude(3.0);\n"
        "    select face { front : { Glazed(); } }\n"
        "}\n"
        "rule Glazed { window(0.0, 0.1, 0.05, 0.05, 0.0); }\n";

    const GenerationResult result = run_on(source, shape_from_rect(2.0, 4.0));
    CHECK_FALSE(result.ok());
    CHECK_EQ(count_messages(result, Severity::Error,
                            "'window' wants a positive wall thickness, not 0"),
             size_t{1});
}

/// A rule that hands a solid to window() gets the diagnostic at its own line
TEST(OpFacade, a_rule_that_glazes_a_solid_is_told_to_select_a_face) {
    const std::string source =
        "@start\n"
        "rule Main { extrude(3.0); window(); }\n";

    const GenerationResult result = run_on(source, shape_from_rect(2.0, 4.0));
    CHECK_FALSE(result.ok());
    CHECK_EQ(count_messages(result, Severity::Error, "pick the wall with `select face` first"),
             size_t{1});
}

/// A rule can set the wall thickness once and every tile beneath it inherits
TEST(OpFacade, a_rule_can_set_the_wall_thickness_once_for_the_whole_building) {
    const std::string source =
        "@start\n"
        "rule Main {\n"
        "    extrude(3.0);\n"
        "    select face { front : { Glazed(); } }\n"
        "}\n"
        "rule Glazed { window(); }\n";

    Shape seed = shape_from_rect(2.0, 4.0);
    seed.attributes[kWallThicknessAttribute] = Value::number(0.5);

    const GenerationResult result = run_on(source, seed);
    CHECK_TRUE(result.ok());
    CHECK_EQ(result.terminals.size(), size_t{1});
    if (result.terminals.size() == 1) {
        // Half of 0.5, not half of the 0.3 default.
        CHECK_NEAR(part_world_centroid(result.terminals[0], FacadePart::Glass).z,
                   4.0 - 0.25, 1e-12);
    }
}

// ============================================================================
// Determinism
// ============================================================================

/**
 * @brief The same facade twice is byte-identical, and one millimetre is not
 *
 * Both halves are needed. Agreement alone passes for a dump that says nothing
 * about the geometry, which is the defect this project has found before;
 * dump_shape() prints every vertex of every face, and the second half is what
 * proves this dump really is reading them.
 */
TEST(OpFacade, two_runs_agree_and_one_millimetre_of_reveal_does_not) {
    const GenerationResult first = run_on(kFacadeSource, shape_from_rect(6.0, 4.0));
    const GenerationResult again = run_on(kFacadeSource, shape_from_rect(6.0, 4.0));

    CHECK_TRUE(first.dump().size() > 20000);
    CHECK_EQ(first.dump(), again.dump());

    // dump_shape() prints every vertex and NO MaterialKey, so the comparison
    // above cannot see a facade whose glass and stone were swapped: the vertices
    // are identical either way. The signature is the half it does not cover, and
    // it has to agree across runs for the same reason.
    CHECK_EQ(material_signature(first), material_signature(again));
    CHECK_TRUE(material_signature(first).find("glass=1/") != std::string::npos);

    // One millimetre deeper, everywhere else identical.
    std::string moved{kFacadeSource};
    const size_t at = moved.find("rule Window     { window(); }");
    CHECK_TRUE(at != std::string::npos);
    if (at != std::string::npos) {
        moved.replace(at, std::string{"rule Window     { window(); }"}.size(),
                      "rule Window     { window(0.0, 0.151); }");
    }
    const GenerationResult shifted = run_on(moved, shape_from_rect(6.0, 4.0));
    CHECK_TRUE(shifted.ok());
    CHECK_EQ(shifted.terminals.size(), size_t{37});
    CHECK_TRUE(first.dump() != shifted.dump());

    // And a millimetre of inset moves different vertices again, so the dump is
    // not merely sensitive to one number.
    std::string inset{kFacadeSource};
    const size_t inset_at = inset.find("rule Window     { window(); }");
    if (inset_at != std::string::npos) {
        inset.replace(inset_at, std::string{"rule Window     { window(); }"}.size(),
                      "rule Window     { window(0.001); }");
    }
    const GenerationResult narrowed = run_on(inset, shape_from_rect(6.0, 4.0));
    CHECK_TRUE(narrowed.ok());
    CHECK_TRUE(first.dump() != narrowed.dump());
    CHECK_TRUE(shifted.dump() != narrowed.dump());

    // The signature is sensitive too: a millimetre of inset adds four surround
    // strips that were not there, which is a change of PART and not only of
    // vertex. Without this, the signature comparison above would pass for a
    // material_signature() that returned the empty string.
    CHECK_TRUE(material_signature(first) != material_signature(narrowed));
}

// ============================================================================
// The shipping table
// ============================================================================

/**
 * @brief The facade operations are reachable from the table the product runs
 *
 * EVERY other test in this file builds its own table through
 * facade_operations(), which is the accessor this file provides for itself. That
 * is the right thing for testing the operations and it is blind to the thing that
 * actually ships: registry.cpp's full_operations(), the single table the rule
 * editor panel, the Python bindings and every headless generation path are handed.
 *
 * A family whose handlers are registered but that registry.cpp never calls is
 * exactly as unusable as a family with no handlers at all, and it says so in the
 * most misleading way the language has -- `operation 'window' is not implemented
 * in this build`, at the right line, with the right caret, in a build where
 * window() demonstrably works. The author then goes looking in their rule file.
 * registry.hpp calls this out, and tests/procgen/test_registry.cpp exists for it.
 *
 * The paired half matters as much: standard_operations() must still REFUSE the
 * same rule. Without it this passes for a `window` that was quietly moved into
 * the D2 set, which would put facade geometry behind interpreter.cpp and undo the
 * reason the registry design exists.
 */
TEST(OpFacade, the_shipping_table_reaches_the_facade_operations) {
    if (full_operations().find("window") == nullptr) {
        // The three edits, spelled out, because "find(\"window\") != nullptr"
        // sends the reader looking at op_facade.cpp, where the handler is fine.
        std::printf(
            "  the facade family is registered in no shipping table. Three edits:\n"
            "    src/procgen/rules/registry.cpp  #include \"procgen/rules/op_facade.hpp\"\n"
            "                                    register_facade_operations(t); in full_operations()\n"
            "    CMakeLists.txt                  src/procgen/rules/op_facade.cpp in stratum_core\n"
            "    tests/procgen/test_registry.cpp drop door, wall_panel and window from\n"
            "                                    kUnimplemented, which that file asks for by name\n");
    }
    CHECK_TRUE(full_operations().find("window") != nullptr);
    CHECK_TRUE(full_operations().find("door") != nullptr);
    CHECK_TRUE(full_operations().find("wall_panel") != nullptr);

    // The names are catalogue rows in ast.hpp either way, so the D2 table parses
    // them and refuses to run them. That is what makes the pair a real test.
    CHECK_TRUE(standard_operations().find("window") == nullptr);
    CHECK_TRUE(standard_operations().find("door") == nullptr);
    CHECK_TRUE(standard_operations().find("wall_panel") == nullptr);

    // And the whole way through, on a genuine `select face` panel, because a
    // registered name that throws away its geometry passes the lookups above.
    const std::string source =
        "@start\n"
        "rule Main {\n"
        "    extrude(3.0);\n"
        "    select face { front : { Glazed(); } }\n"
        "}\n"
        "rule Glazed { window(); }\n";
    const ParseResult parsed = parse(source, "test.srl");
    CHECK_EQ(parsed.render_all(source), std::string{});

    const GenerationResult shipped = generate(parsed.file, shape_from_rect(2.0, 4.0), {},
                                              &full_operations(), &full_functions());
    CHECK_TRUE(shipped.ok());
    CHECK_EQ(count_messages(shipped, Severity::Error,
                            "operation 'window' is not implemented in this build"),
             size_t{0});
    CHECK_EQ(shipped.terminals.size(), size_t{1});
    if (shipped.terminals.size() == 1) {
        CHECK_EQ(part_faces(shipped.terminals[0], FacadePart::Glass), size_t{1});
        CHECK_NEAR(part_world_centroid(shipped.terminals[0], FacadePart::Glass).z,
                   4.0 - 0.15, 1e-12);
    }

    const GenerationResult d2 = generate(parsed.file, shape_from_rect(2.0, 4.0), {},
                                         &standard_operations(), &standard_functions());
    CHECK_FALSE(d2.ok());
    CHECK_EQ(count_messages(d2, Severity::Error,
                            "operation 'window' is not implemented in this build"),
             size_t{1});
}

} // namespace
