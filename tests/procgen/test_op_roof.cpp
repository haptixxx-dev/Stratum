// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_op_roof.cpp
 * @brief `roof`: the six shapes, the numbers they produce, and the things they refuse
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ================================================================================
 * HOW THESE TESTS ARE WRITTEN SO THAT THEY CAN FAIL
 * ================================================================================
 *
 * A roof that came out wrong still looks like a list of faces, and a suite that
 * counts faces would pass for almost any of the ways it can be wrong. So nothing
 * here asserts a face count on its own. Every assertion is one of five kinds,
 * and each of the five was chosen because a specific plausible bug survives the
 * other four:
 *
 *   - **Volume, against hand arithmetic.** A hipped roof over a `w` by `l`
 *     rectangle at rise `h` encloses `w*h/6 * (3l - w)` -- a prism of length
 *     `l - w` with the two hipped ends together making a `w` by `w` pyramid.
 *     That number is written out in each test that uses it, from the rectangle
 *     the test itself builds. A roof built to the wrong height, over the bounding
 *     box instead of the footprint, or with a ridge in the wrong place all miss
 *     it. A roof with an extra face in it does not: hence the next one.
 *
 *   - **Watertightness, by counting every boundary edge.** `is_watertight()`
 *     requires every undirected edge of every face to appear EXACTLY twice. A
 *     missing gable wall, a duplicated cap, a slope that meets the wall top half
 *     an ulp away, a hole whose ring was dropped: each leaves an edge with a
 *     count of one or three, and none of them changes the volume enough for a
 *     tolerance to notice. geometry_volume() is meaningless on an open shell, so
 *     this is what says the volume assertions mean anything at all.
 *
 *   - **Plan coverage, which is the straight skeleton's own invariant.** The
 *     skeleton's faces TILE the polygon. Summing the plan area of every
 *     upward-facing roof face and comparing it with the footprint's own area is
 *     the assertion a missing split event breaks, and it is the reason the
 *     non-convex cases are here at all.
 *
 *   - **Two constructions that must agree.** `gable` reaches a rectangle's roof
 *     through the straight skeleton, a push and a half-plane clip; `ridge`
 *     reaches the same roof by projecting each edge onto a ridge segment, with no
 *     skeleton anywhere. On a rectangle the two must produce the same vertices.
 *     Nothing in either implementation knows about the other, so the agreement is
 *     evidence and not a tautology. `hip` and `pyramid` over a square are the
 *     same pair of independent routes to one solid.
 *
 *   - **Two inputs that must disagree.** Every "the argument reached the
 *     geometry" test is written as a pair: a hip and a gable over the same box
 *     have DIFFERENT volumes, a dome with 2 bands and one with 32 have different
 *     volumes, two pitches produce different dumps. An assertion that a roof
 *     "exists" passes for an implementation that ignores its arguments.
 *
 * ### What was deliberately not asserted
 *
 * No test asserts `positions.size()`, and none asserts the ORDER of the faces.
 * Both are properties of how the builder happens to append, not of the roof, and
 * a suite that pins them turns every future refactor into a red run without ever
 * having caught a wrong roof.
 *
 * Nothing here needs a GPU, a window or a file on disk, so nothing here skips.
 */

#include "framework.hpp"

#include "geometry/straight_skeleton.hpp"
#include "procgen/rules/interpreter.hpp"
#include "procgen/rules/op_roof.hpp"
#include "procgen/rules/parser.hpp"
#include "procgen/rules/shape.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

using stratum::procgen::rules::Diagnostic;
using stratum::procgen::rules::extrude_shape;
using stratum::procgen::rules::Face;
using stratum::procgen::rules::face_normal;
using stratum::procgen::rules::GenerationOptions;
using stratum::procgen::rules::GenerationResult;
using stratum::procgen::rules::generate;
using stratum::procgen::rules::geometry_volume;
using stratum::procgen::rules::OpResult;
using stratum::procgen::rules::dump_shape;
using stratum::procgen::rules::parse;
using stratum::procgen::rules::parse_roof_kind;
using stratum::procgen::rules::ParseResult;
using stratum::procgen::rules::roof_kind_name;
using stratum::procgen::rules::roof_operations;
using stratum::procgen::rules::roof_shape;
using stratum::procgen::rules::RoofKind;
using stratum::procgen::rules::RoofParams;
using stratum::procgen::rules::RoofReport;
using stratum::procgen::rules::rotate_scope_shape;
using stratum::procgen::rules::rotate_shape;
using stratum::procgen::rules::ScopeAxis;
using stratum::procgen::rules::Severity;
using stratum::procgen::rules::Shape;
using stratum::procgen::rules::ShapeGeometry;
using stratum::procgen::rules::shape_from_polygon;
using stratum::procgen::rules::shape_from_rect;
using stratum::procgen::rules::shape_from_rings;
using stratum::procgen::rules::Value;

namespace {

// ============================================================================
// Shapes to roof
// ============================================================================

/// A closed box: the rectangle @p sx by @p sz in the xz plane, extruded @p height up
[[nodiscard]] Shape make_box(double sx, double sz, double height) {
    Shape shape = shape_from_rect(sx, sz);
    const OpResult result = extrude_shape(shape, ScopeAxis::Y, height);
    CHECK_TRUE(result.ok);
    return shape;
}

/// The L-shaped plan used by the non-convex cases: two arms, each 2 wide
///
/// Area 20, inradius 1. The inradius matters: it is the skeleton's max_time, so
/// a unit-pitch roof over this rises exactly 1, which is a number the tests can
/// state rather than read back.
[[nodiscard]] std::vector<glm::dvec2> ell_plan() {
    return {{0.0, 0.0}, {6.0, 0.0}, {6.0, 2.0}, {2.0, 2.0}, {2.0, 6.0}, {0.0, 6.0}};
}

/// A rectangle with a notch bitten out of one end. Area 22.
///
/// Built for one job: its leftmost end edge, the one `gable` would choose, has
/// part of the footprint on the far side of its own line. That is the
/// precondition select_gable_ends() checks, and this is the plan that fails it.
[[nodiscard]] std::vector<glm::dvec2> notched_plan() {
    return {{0.0, 0.0}, {6.0, 0.0}, {6.0, 4.0}, {0.0, 4.0}, {1.0, 2.0}};
}

/// A U: the 8 by 6 rectangle with a 4 by 4 notch cut down into it. Area 32.
///
/// The plan that separates "the apex is INSIDE the footprint" from "the apex can
/// SEE the footprint". Every arm is 2 wide, so the inradius is 1 and a
/// 45-degree roof rises exactly 1 -- and the deepest point is in one arm or the
/// base, from where the far arm is round a corner. A pyramid raised to it would
/// throw triangles across the notch, which is not the building.
[[nodiscard]] std::vector<glm::dvec2> u_plan() {
    return {{0.0, 0.0}, {8.0, 0.0}, {8.0, 6.0}, {6.0, 6.0},
            {6.0, 2.0}, {2.0, 2.0}, {2.0, 6.0}, {0.0, 6.0}};
}

/// A comb: the 6 by 6 square with a 4 by 2 slot cut in from one side. Area 28.
///
/// Built for one job too. Its edges (0,6)-(0,4) and (0,2)-(0,0) are two
/// DIFFERENT contour edges lying on one line, x = 0. `gable` picks the first as
/// an end and clips with that line, so the second edge's own slope has vertices
/// exactly on the cut -- which is the case clip_half_plane()'s on-the-line
/// branch exists for, and the case that decides whether the gable wall takes
/// only the profile points that are on its own edge.
[[nodiscard]] std::vector<glm::dvec2> comb_plan() {
    return {{0.0, 0.0}, {6.0, 0.0}, {6.0, 6.0}, {0.0, 6.0},
            {0.0, 4.0}, {4.0, 4.0}, {4.0, 2.0}, {0.0, 2.0}};
}

/// One long slanted side with a corner projecting into the middle of it. Area 32.
///
/// Built for one job: driven with a ridge along +x, the edge from (0,0) to (8,4)
/// spans the whole ridge, the corner (4,6) projects into the middle of that
/// span, and the edge is slanted -- so the slope over it is a FIVE-cornered ring
/// that is not on one plane. That is the one case a quad-only split cannot
/// handle, and the case that says the fan runs the whole way round.
[[nodiscard]] std::vector<glm::dvec2> long_slant_plan() {
    return {{0.0, 0.0}, {8.0, 4.0}, {8.0, 6.0}, {4.0, 6.0}, {0.0, 6.0}};
}

/// An 8 by 4 rectangle with two extra corners half a picometre out of line
///
/// Built for one job as well. Its corner at (4, 0) and its corner at
/// (4.0000000000005, 4) project onto the ridge 5e-13 apart -- near enough to be
/// one station, far enough to be two doubles. A ridge that does not weld them
/// leaves the two slopes meeting at two points that are one point, which no
/// quantised edge count can see and every mesh welder downstream can.
[[nodiscard]] std::vector<glm::dvec2> near_pair_plan() {
    return {{0.0, 0.0}, {4.0, 0.0}, {8.0, 0.0},
            {8.0, 4.0}, {4.0000000000005, 4.0}, {0.0, 4.0}};
}

/// A trapezoid with one slanted side. Area 24.
///
/// Its corners project onto a ridge along +x at three distinct places, 0, 4 and
/// 8, but only the slanted side produces the 4. That asymmetry is what left
/// `ridge` with a T-junction along its own ridge line.
[[nodiscard]] std::vector<glm::dvec2> trapezoid_plan() {
    return {{0.0, 0.0}, {8.0, 0.0}, {8.0, 4.0}, {4.0, 4.0}};
}

/// @p plan, extruded @p height up: a closed building to put a roof on
[[nodiscard]] Shape make_block(const std::vector<glm::dvec2>& plan, double height) {
    Shape shape = shape_from_polygon(plan);
    const OpResult lift = extrude_shape(shape, ScopeAxis::Y, height);
    CHECK_TRUE(lift.ok);
    return shape;
}

// ============================================================================
// Measuring a roof
// ============================================================================

/// Highest local y of any vertex
[[nodiscard]] double peak(const Shape& shape) {
    double top = -1.0e300;
    for (const glm::dvec3& p : shape.geometry.positions) {
        top = std::max(top, p.y);
    }
    return top;
}

/**
 * @brief How many DISTINCT positions sit at the shape's highest local y
 *
 * One for a pyramid's apex; several for a hip's or a gable's ridge. Counting raw
 * positions cannot tell them apart, because every face pushes its own copy of
 * each corner it uses -- so a pyramid over a six-sided plan has six positions at
 * the peak and they are all the same point.
 */
[[nodiscard]] size_t distinct_peak_points(const Shape& shape) {
    const double top = peak(shape);
    std::vector<glm::dvec3> found;
    for (const glm::dvec3& p : shape.geometry.positions) {
        if (std::fabs(p.y - top) > 1e-9) {
            continue;
        }
        bool seen = false;
        for (const glm::dvec3& q : found) {
            if (glm::all(glm::lessThan(glm::abs(p - q), glm::dvec3{1e-9}))) {
                seen = true;
                break;
            }
        }
        if (!seen) {
            found.push_back(p);
        }
    }
    return found.size();
}

/**
 * @brief Pairs of vertices at local y == @p height that are within @p eps but not equal
 *
 * Two vertices half a picometre apart are one corner to any renderer, mesh
 * welder or exporter, and the pair is exactly what a weld that did not happen
 * leaves behind. is_watertight() cannot see it: it keys on positions quantised
 * to 1e-7, so it reads the pair as one point and reports a closed solid. This is
 * the finer question, asked at the one height where the answer matters.
 */
[[nodiscard]] size_t nearly_coincident_at(const Shape& shape, double height, double eps) {
    const std::vector<glm::dvec3>& points = shape.geometry.positions;
    size_t found = 0;
    for (size_t i = 0; i < points.size(); ++i) {
        if (std::fabs(points[i].y - height) > 1e-6) {
            continue;
        }
        for (size_t j = i + 1; j < points.size(); ++j) {
            if (std::fabs(points[j].y - height) > 1e-6 || points[i] == points[j]) {
                continue;
            }
            if (glm::all(glm::lessThan(glm::abs(points[i] - points[j]), glm::dvec3{eps}))) {
                ++found;
            }
        }
    }
    return found;
}

/// Signed area of a ring of local points projected onto the xz plane
[[nodiscard]] double plan_area_of(const ShapeGeometry& geometry,
                                  const std::vector<uint32_t>& ring) {
    double twice = 0.0;
    for (size_t i = 0; i < ring.size(); ++i) {
        const glm::dvec3& a = geometry.positions[ring[i]];
        const glm::dvec3& b = geometry.positions[ring[(i + 1) % ring.size()]];
        twice += a.x * b.z - b.x * a.z;
    }
    return 0.5 * twice;
}

/**
 * @brief Ground covered by the faces that face upward
 *
 * The straight skeleton's faces tile the polygon, so this must come back as the
 * footprint's own area for every kind that is a lower envelope over it. A gable
 * wall is vertical and covers nothing, and a floor or a soffit faces down, so
 * both are excluded by the normal test rather than by a special case.
 */
[[nodiscard]] double upward_plan_area(const Shape& shape) {
    double total = 0.0;
    for (const Face& face : shape.geometry.faces) {
        if (face_normal(shape.geometry, face).y <= 1e-9) {
            continue;
        }
        total += std::fabs(plan_area_of(shape.geometry, face.loop));
        for (const std::vector<uint32_t>& hole : face.holes) {
            total -= std::fabs(plan_area_of(shape.geometry, hole));
        }
    }
    return total;
}

/// An undirected edge between two quantised local positions
using EdgeKey = std::array<int64_t, 6>;

[[nodiscard]] EdgeKey edge_key(const glm::dvec3& a, const glm::dvec3& b) {
    auto quantise = [](double v) { return static_cast<int64_t>(std::llround(v / 1e-7)); };
    const std::array<int64_t, 3> first = {quantise(a.x), quantise(a.y), quantise(a.z)};
    const std::array<int64_t, 3> second = {quantise(b.x), quantise(b.y), quantise(b.z)};
    const bool swap = second < first;
    const std::array<int64_t, 3>& low = swap ? second : first;
    const std::array<int64_t, 3>& high = swap ? first : second;
    return EdgeKey{low[0], low[1], low[2], high[0], high[1], high[2]};
}

/**
 * @brief Does every boundary edge belong to exactly two faces?
 *
 * The assertion that makes every volume assertion in this file mean something.
 * geometry_volume() is the divergence theorem and returns a number for an open
 * shell too -- a plausible number, which is worse than none. A solid whose edges
 * all pair up is closed, and a roof that left a gap, emitted a face twice, or
 * met the wall top a micron out has an edge with the wrong count.
 */
[[nodiscard]] bool is_watertight(const Shape& shape) {
    std::map<EdgeKey, int> counts;
    auto walk = [&](const std::vector<uint32_t>& ring) {
        for (size_t i = 0; i < ring.size(); ++i) {
            ++counts[edge_key(shape.geometry.positions[ring[i]],
                              shape.geometry.positions[ring[(i + 1) % ring.size()]])];
        }
    };
    for (const Face& face : shape.geometry.faces) {
        walk(face.loop);
        for (const std::vector<uint32_t>& hole : face.holes) {
            walk(hole);
        }
    }
    if (counts.empty()) {
        return false;
    }
    for (const auto& entry : counts) {
        if (entry.second != 2) {
            return false;
        }
    }
    return true;
}

/// Every vertex, in world space, sorted so that two shapes can be compared
/// without depending on the order their faces happened to be built in
[[nodiscard]] std::vector<glm::dvec3> sorted_world_points(const Shape& shape) {
    std::vector<glm::dvec3> points;
    points.reserve(shape.geometry.positions.size());
    for (const glm::dvec3& local : shape.geometry.positions) {
        points.push_back(shape.scope.to_world(local));
    }
    std::sort(points.begin(), points.end(), [](const glm::dvec3& a, const glm::dvec3& b) {
        if (a.x != b.x) {
            return a.x < b.x;
        }
        if (a.y != b.y) {
            return a.y < b.y;
        }
        return a.z < b.z;
    });
    return points;
}

/// Do two shapes have the same vertices, to within @p eps?
[[nodiscard]] bool same_points(const Shape& lhs, const Shape& rhs, double eps) {
    const std::vector<glm::dvec3> a = sorted_world_points(lhs);
    const std::vector<glm::dvec3> b = sorted_world_points(rhs);
    if (a.size() != b.size()) {
        return false;
    }
    for (size_t i = 0; i < a.size(); ++i) {
        if (glm::any(glm::greaterThan(glm::abs(a[i] - b[i]), glm::dvec3{eps}))) {
            return false;
        }
    }
    return true;
}

/// Faces every one of whose vertices sits at local y == @p height
[[nodiscard]] size_t faces_flat_at(const Shape& shape, double height) {
    size_t found = 0;
    for (const Face& face : shape.geometry.faces) {
        bool flat = true;
        for (const uint32_t index : face.loop) {
            if (std::fabs(shape.geometry.positions[index].y - height) > 1e-9) {
                flat = false;
                break;
            }
        }
        if (flat) {
            ++found;
        }
    }
    return found;
}

// ============================================================================
// Driving the operation
// ============================================================================

[[nodiscard]] RoofParams params_of(RoofKind kind, double pitch) {
    RoofParams params;
    params.kind = kind;
    params.pitch_degrees = pitch;
    return params;
}

/// roof_shape() with the result asserted, so a test that meant to succeed prints
/// the operation's own sentence rather than "false != true"
RoofReport roof_ok(Shape& shape, const RoofParams& params) {
    RoofReport report;
    const OpResult result = roof_shape(shape, params, report);
    CHECK_EQ(result.message, std::string{});
    CHECK_TRUE(result.ok);
    return report;
}

/// The message roof_shape() refused with, or empty when it did not refuse
[[nodiscard]] std::string roof_refusal(Shape shape, const RoofParams& params) {
    RoofReport report;
    const OpResult result = roof_shape(shape, params, report);
    if (result.ok) {
        return std::string{};
    }
    return result.message;
}

[[nodiscard]] bool has_note(const RoofReport& report, const std::string& needle) {
    for (const std::string& note : report.notes) {
        if (note.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

/// Parse and run @p source with `roof` registered
[[nodiscard]] GenerationResult run_on(const std::string& source,
                                      const Shape& seed,
                                      const GenerationOptions& options = {}) {
    const ParseResult parsed = parse(source, "test.srl");
    CHECK_EQ(parsed.render_all(source), std::string{});
    return generate(parsed.file, seed, options, &roof_operations(), nullptr);
}

[[nodiscard]] bool has_message(const GenerationResult& result,
                               Severity severity,
                               const std::string& needle) {
    for (const Diagnostic& diagnostic : result.diagnostics) {
        if (diagnostic.severity == severity &&
            diagnostic.message.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

/// Volume of a hipped roof over a @p length by @p width rectangle rising @p rise
///
/// A prism of length `length - width` with a triangular section, plus the two
/// hipped ends, which together are a `width` by `width` pyramid. Written out so
/// that the expectation in each test is arithmetic and not a recorded output.
[[nodiscard]] double hip_volume(double length, double width, double rise) {
    return width * rise / 6.0 * (3.0 * length - width);
}

} // namespace

// ============================================================================
// The numbers
// ============================================================================

/**
 * @brief A hip over a rectangle, checked against hand arithmetic
 *
 * The box is 6 by 4 in plan, extruded 10, so the wall top is a 6 by 4 rectangle
 * at y = 10 and the solid below it holds 240. At 45 degrees the rise is half the
 * SHORT side: 2. The ridge therefore sits at y = 12 and runs along the long axis
 * for `6 - 4 = 2` metres, and the roof holds `4*2/6 * (18 - 4) = 56/3`.
 *
 * Every one of those numbers fails for a roof built over the bounding box, for a
 * roof whose rise came from the long side, and for a roof with a single apex
 * instead of a ridge -- which is the answer osm/mesh_builder.cpp had to correct
 * in E1 and is the reason the ridge length is asserted separately below.
 */
TEST(OpRoof, a_hip_over_a_rectangle_has_the_volume_hand_arithmetic_gives) {
    Shape shape = make_box(6.0, 4.0, 10.0);
    const RoofReport report = roof_ok(shape, params_of(RoofKind::Hip, 45.0));

    CHECK_TRUE(report.used_skeleton);
    CHECK_NEAR(report.pitch_tangent, 1.0, 1e-12);
    CHECK_NEAR(report.rise, 2.0, 1e-9);
    CHECK_EQ(report.footprints, size_t{1});

    CHECK_NEAR(peak(shape), 12.0, 1e-9);
    CHECK_NEAR(shape.scope.size.x, 6.0, 1e-9);
    CHECK_NEAR(shape.scope.size.y, 12.0, 1e-9);
    CHECK_NEAR(shape.scope.size.z, 4.0, 1e-9);

    CHECK_TRUE(is_watertight(shape));
    CHECK_NEAR(geometry_volume(shape.geometry), 240.0 + hip_volume(6.0, 4.0, 2.0), 1e-9);
    CHECK_NEAR(geometry_volume(shape.geometry), 240.0 + 56.0 / 3.0, 1e-9);

    // The roof planes tile the footprint: 6 by 4 of ground, covered once.
    CHECK_NEAR(upward_plan_area(shape), 24.0, 1e-9);

    // A hip has a RIDGE and not an apex. The ridge is the set of vertices at the
    // peak, and over a 6 by 4 plan it is a segment two metres long -- the long
    // side less the short one. A pyramid would put one vertex there.
    double lowest = 1.0e300;
    double highest = -1.0e300;
    size_t on_ridge = 0;
    for (const glm::dvec3& p : shape.geometry.positions) {
        if (std::fabs(p.y - 12.0) < 1e-9) {
            ++on_ridge;
            lowest = std::min(lowest, p.x);
            highest = std::max(highest, p.x);
        }
    }
    CHECK_TRUE(on_ridge > 0);
    CHECK_NEAR(lowest, 2.0, 1e-9);
    CHECK_NEAR(highest, 4.0, 1e-9);
}

/**
 * @brief A gable over the same rectangle is a prism, and its ends are vertical
 *
 * The same box, the same pitch, a different silhouette: the ridge runs the whole
 * 6 metres instead of 2, so the roof is a plain triangular prism holding
 * `4*2/2 * 6 = 24` -- more than the hip's 56/3, which is the pair of numbers
 * that says the kind argument reached the geometry.
 *
 * The gable ends are asserted to be VERTICAL, because a sloped gable end is the
 * other answer osm/mesh_builder.cpp had to correct, and because a "gable" whose
 * ends slope is a hip with extra steps.
 */
TEST(OpRoof, a_gable_over_the_same_rectangle_is_a_prism_with_vertical_ends) {
    Shape shape = make_box(6.0, 4.0, 10.0);
    const RoofReport report = roof_ok(shape, params_of(RoofKind::Gable, 45.0));

    CHECK_EQ(report.gable_walls, size_t{2});
    CHECK_NEAR(report.rise, 2.0, 1e-9);
    CHECK_NEAR(peak(shape), 12.0, 1e-9);

    CHECK_TRUE(is_watertight(shape));
    CHECK_NEAR(geometry_volume(shape.geometry), 240.0 + 24.0, 1e-9);
    CHECK_NEAR(upward_plan_area(shape), 24.0, 1e-9);

    // The ridge spans the whole length, unlike the hip's two metres.
    double lowest = 1.0e300;
    double highest = -1.0e300;
    for (const glm::dvec3& p : shape.geometry.positions) {
        if (std::fabs(p.y - 12.0) < 1e-9) {
            lowest = std::min(lowest, p.x);
            highest = std::max(highest, p.x);
        }
    }
    CHECK_NEAR(lowest, 0.0, 1e-9);
    CHECK_NEAR(highest, 6.0, 1e-9);

    // Two vertical faces entirely above the wall top, four units of area each:
    // the triangles over the 4-metre ends, half of 4 by 2.
    size_t vertical = 0;
    double vertical_area = 0.0;
    for (const Face& face : shape.geometry.faces) {
        bool above = true;
        for (const uint32_t index : face.loop) {
            if (shape.geometry.positions[index].y < 10.0 - 1e-9) {
                above = false;
                break;
            }
        }
        if (above && std::fabs(face_normal(shape.geometry, face).y) < 1e-9) {
            ++vertical;
            vertical_area += stratum::procgen::rules::face_area(shape.geometry, face);
        }
    }
    CHECK_EQ(vertical, size_t{2});
    CHECK_NEAR(vertical_area, 8.0, 1e-9);
}

/**
 * @brief The gable and the ridge reach the same rectangle roof by different routes
 *
 * `gable` goes through the straight skeleton of a pushed-out footprint and a
 * half-plane clip. `ridge` projects each contour edge onto a ridge segment and
 * never calls the skeleton at all. Neither knows the other exists, so their
 * agreeing on a rectangle -- vertex for vertex -- is evidence that both are
 * right rather than a restatement of one of them.
 */
TEST(OpRoof, gable_and_ridge_agree_vertex_for_vertex_on_a_rectangle) {
    Shape gabled = make_box(6.0, 4.0, 10.0);
    Shape ridged = make_box(6.0, 4.0, 10.0);
    const RoofReport gable_report = roof_ok(gabled, params_of(RoofKind::Gable, 35.0));
    const RoofReport ridge_report = roof_ok(ridged, params_of(RoofKind::Ridge, 35.0));

    CHECK_TRUE(gable_report.used_skeleton);
    CHECK_FALSE(ridge_report.used_skeleton);
    CHECK_NEAR(gable_report.rise, ridge_report.rise, 1e-9);

    CHECK_TRUE(same_points(gabled, ridged, 1e-9));
    CHECK_NEAR(geometry_volume(gabled.geometry), geometry_volume(ridged.geometry), 1e-9);
    CHECK_TRUE(is_watertight(ridged));
}

/**
 * @brief A pyramid and a hip are the same solid over a square, by two routes
 *
 * On a square the straight skeleton's four faces meet at one point, so the hip
 * IS a pyramid. build_hip() gets there by lifting skeleton faces and
 * build_pyramid() by raising every edge to the deepest node; the two share only
 * the skeleton call, so the solids agreeing is a check on both lifts.
 *
 * A 4 by 4 plan at 45 degrees rises 2 and holds `16 * 2 / 3`.
 */
TEST(OpRoof, a_pyramid_and_a_hip_are_the_same_solid_over_a_square) {
    Shape hipped = make_box(4.0, 4.0, 3.0);
    Shape pyramidal = make_box(4.0, 4.0, 3.0);
    roof_ok(hipped, params_of(RoofKind::Hip, 45.0));
    roof_ok(pyramidal, params_of(RoofKind::Pyramid, 45.0));

    CHECK_NEAR(peak(hipped), 5.0, 1e-9);
    CHECK_NEAR(peak(pyramidal), 5.0, 1e-9);
    CHECK_TRUE(is_watertight(hipped));
    CHECK_TRUE(is_watertight(pyramidal));

    const double solid = 4.0 * 4.0 * 3.0;
    const double cap = 16.0 * 2.0 / 3.0;
    CHECK_NEAR(geometry_volume(hipped.geometry), solid + cap, 1e-9);
    CHECK_NEAR(geometry_volume(pyramidal.geometry), solid + cap, 1e-9);
    CHECK_NEAR(upward_plan_area(pyramidal), 16.0, 1e-9);
}

/**
 * @brief A shed is one tilted plane, and it falls across the short axis
 *
 * The 6 by 4 box again. The default downhill runs across the principal axis, so
 * the roof climbs the 4-metre span and at 45 degrees rises the whole 4 -- not 2,
 * which is what a shed built as half a gable would give. The wedge it adds is
 * `6 * (4 * 4 / 2) = 48`.
 */
TEST(OpRoof, a_shed_falls_across_the_short_axis_and_rises_the_whole_span) {
    Shape shape = make_box(6.0, 4.0, 10.0);
    const RoofReport report = roof_ok(shape, params_of(RoofKind::Shed, 45.0));

    CHECK_FALSE(report.used_skeleton);
    CHECK_NEAR(report.rise, 4.0, 1e-9);
    CHECK_NEAR(peak(shape), 14.0, 1e-9);
    CHECK_TRUE(is_watertight(shape));
    CHECK_NEAR(geometry_volume(shape.geometry), 240.0 + 48.0, 1e-9);

    // The high side is one edge of the plan and the low side another, so exactly
    // one horizontal line of vertices sits at the wall top on the roof.
    double high_z = -1.0e300;
    for (const glm::dvec3& p : shape.geometry.positions) {
        if (std::fabs(p.y - 14.0) < 1e-9) {
            high_z = std::max(high_z, p.z);
        }
    }
    CHECK_NEAR(high_z, 4.0, 1e-9);
}

/**
 * @brief The fourth argument turns a shed, and the turn is visible in the solid
 *
 * `roof("shed", 45, 0, 0)` points the fall along +x, which is the LONG axis, so
 * the run becomes 6 instead of 4 and the wedge becomes `4 * (6 * 6 / 2) = 72`.
 * Asserting the two wedges differ is the point: a fourth argument that were
 * quietly dropped would give 48 both times and every other assertion would still
 * pass.
 */
TEST(OpRoof, the_fourth_argument_turns_a_shed) {
    Shape along_x = make_box(6.0, 4.0, 10.0);
    RoofParams params = params_of(RoofKind::Shed, 45.0);
    params.has_extra = true;
    params.extra = 0.0;  // downhill towards +x
    const RoofReport report = roof_ok(along_x, params);

    CHECK_NEAR(report.rise, 6.0, 1e-9);
    CHECK_NEAR(peak(along_x), 16.0, 1e-9);
    CHECK_TRUE(is_watertight(along_x));
    CHECK_NEAR(geometry_volume(along_x.geometry), 240.0 + 72.0, 1e-9);

    // The high side is now the x = 0 edge, since the fall is towards +x.
    double high_x = 1.0e300;
    for (const glm::dvec3& p : along_x.geometry.positions) {
        if (std::fabs(p.y - 16.0) < 1e-9) {
            high_x = std::min(high_x, p.x);
        }
    }
    CHECK_NEAR(high_x, 0.0, 1e-9);
}

/**
 * @brief A dome rises to the inradius at 45 degrees, and its bands are its argument
 *
 * `rise = inradius * tan(pitch)` is the same rule every other kind follows, so a
 * dome at 45 degrees over a 4 by 4 plan is a hemisphere of radius 2. The band
 * count is checked by comparing two domes: more bands cut less of the sphere
 * away, so the 32-band dome must enclose more than the 2-band one and both must
 * sit between the pyramid they are refining and the prism that bounds them.
 */
TEST(OpRoof, a_dome_rises_to_the_inradius_and_its_band_count_is_its_argument) {
    Shape coarse = make_box(4.0, 4.0, 3.0);
    Shape fine = make_box(4.0, 4.0, 3.0);

    RoofParams params = params_of(RoofKind::Dome, 45.0);
    params.has_extra = true;
    params.extra = 2.0;
    const RoofReport coarse_report = roof_ok(coarse, params);
    params.extra = 32.0;
    roof_ok(fine, params);

    CHECK_NEAR(coarse_report.rise, 2.0, 1e-9);
    CHECK_NEAR(peak(coarse), 5.0, 1e-9);
    CHECK_NEAR(peak(fine), 5.0, 1e-9);
    CHECK_TRUE(is_watertight(coarse));
    CHECK_TRUE(is_watertight(fine));

    const double solid = 4.0 * 4.0 * 3.0;
    const double coarse_cap = geometry_volume(coarse.geometry) - solid;
    const double fine_cap = geometry_volume(fine.geometry) - solid;

    // A pyramid is the one-band limit from below; a 4 by 4 by 2 prism bounds it
    // from above. More bands must land strictly between the two and above the
    // coarser dome.
    CHECK_TRUE((16.0 * 2.0 / 3.0) < coarse_cap);
    CHECK_TRUE(coarse_cap < fine_cap);
    CHECK_TRUE(fine_cap < (16.0 * 2.0));

    // A band count outside the range it can build is clamped and SAID so.
    Shape silly = make_box(4.0, 4.0, 3.0);
    params.extra = 500.0;
    const RoofReport clamped = roof_ok(silly, params);
    CHECK_TRUE(has_note(clamped, "clamped to 64"));
}

// ============================================================================
// Non-convex footprints: where the skeleton earns its keep
// ============================================================================

/**
 * @brief A hip over an L tiles the L, which is the invariant a missing split event breaks
 *
 * The L has a reflex corner, so its skeleton needs a split event -- the part
 * straight_skeleton.hpp says naive implementations omit, and omitting which
 * gives faces that OVERLAP rather than faces that tile. Comparing the covered
 * ground with the plan's own 20 catches exactly that, and catches nothing else:
 * a roof at the wrong height covers the same ground.
 *
 * Both arms are 2 wide, so the inradius is 1 and a 45-degree roof rises 1.
 */
TEST(OpRoof, a_hip_over_an_l_tiles_the_l) {
    Shape shape = shape_from_polygon(ell_plan());
    const OpResult lift = extrude_shape(shape, ScopeAxis::Y, 5.0);
    CHECK_TRUE(lift.ok);

    const RoofReport report = roof_ok(shape, params_of(RoofKind::Hip, 45.0));
    CHECK_TRUE(report.used_skeleton);
    CHECK_TRUE(report.skeleton.reflex_vertices > 0);
    CHECK_NEAR(report.rise, 1.0, 1e-9);
    CHECK_NEAR(peak(shape), 6.0, 1e-9);

    CHECK_NEAR(upward_plan_area(shape), 20.0, 1e-9);
    CHECK_TRUE(is_watertight(shape));

    // The roof has real volume and is nowhere near the prism that bounds it.
    const double cap = geometry_volume(shape.geometry) - 20.0 * 5.0;
    CHECK_TRUE(cap > 0.0);
    CHECK_TRUE(cap < 20.0 * 1.0);

    // And no note was needed: an L is exactly what the skeleton is for.
    CHECK_EQ(report.notes.size(), size_t{0});
}

/**
 * @brief A gable over an L tiles it too, with the ends pushed out and clipped back
 *
 * The same coverage assertion over the gable path, which is the one that pushes
 * two edges out by four diameters, skeletonises a much larger polygon and clips
 * the answer back with two half-planes. Any of those three steps going wrong
 * shows up as ground covered twice or not at all.
 */
TEST(OpRoof, a_gable_over_an_l_tiles_the_l_after_the_clip) {
    Shape shape = shape_from_polygon(ell_plan());
    const OpResult lift = extrude_shape(shape, ScopeAxis::Y, 5.0);
    CHECK_TRUE(lift.ok);

    const RoofReport report = roof_ok(shape, params_of(RoofKind::Gable, 45.0));
    CHECK_TRUE(report.used_skeleton);
    CHECK_EQ(report.gable_walls, size_t{2});

    CHECK_NEAR(upward_plan_area(shape), 20.0, 1e-9);
    CHECK_TRUE(is_watertight(shape));

    // Pushing the ends out raises the ridge over the long arm: the gable must
    // enclose strictly more than the hip that would otherwise be built.
    Shape hipped = shape_from_polygon(ell_plan());
    const OpResult lift_again = extrude_shape(hipped, ScopeAxis::Y, 5.0);
    CHECK_TRUE(lift_again.ok);
    roof_ok(hipped, params_of(RoofKind::Hip, 45.0));
    CHECK_TRUE(geometry_volume(hipped.geometry) < geometry_volume(shape.geometry));
}

/**
 * @brief A pyramid's apex is inside the footprint, which a centroid does not promise
 *
 * The L's vertex average is (8/3, 8/3), which is in the notch: OUTSIDE the
 * footprint. An apex there gives a roof that leans off the building, and every
 * volume and coverage assertion in this file would still pass, because a cone to
 * a point outside still covers the plan once.
 *
 * So the apex is the straight skeleton's DEEPEST node, which is on the medial
 * axis and therefore always inside, and this test is what says so. It also
 * checks the apex height against the L's inradius of 1.
 */
TEST(OpRoof, a_pyramids_apex_stays_inside_a_non_convex_footprint) {
    Shape shape = shape_from_polygon(ell_plan());
    const OpResult lift = extrude_shape(shape, ScopeAxis::Y, 4.0);
    CHECK_TRUE(lift.ok);

    const RoofReport report = roof_ok(shape, params_of(RoofKind::Pyramid, 45.0));
    CHECK_NEAR(report.rise, 1.0, 1e-9);
    CHECK_NEAR(peak(shape), 5.0, 1e-9);
    CHECK_NEAR(upward_plan_area(shape), 20.0, 1e-9);

    // An L IS star-shaped from its deepest point, so this is a real pyramid and
    // not the hip that a plan failing that test falls back to. Two assertions
    // say so: no note was needed, and the top of the roof is ONE point. A hip
    // over an L has a ridge along both arms, which is ten.
    CHECK_FALSE(has_note(report, "star-shaped"));
    CHECK_EQ(report.notes.size(), size_t{0});
    CHECK_EQ(distinct_peak_points(shape), size_t{1});

    // The apex, and a point-in-polygon test of the L that the vertex average
    // (8/3, 8/3) fails.
    glm::dvec2 apex{0.0};
    for (const glm::dvec3& p : shape.geometry.positions) {
        if (std::fabs(p.y - 5.0) < 1e-9) {
            apex = glm::dvec2{p.x, p.z};
        }
    }
    const std::vector<glm::dvec2> plan = ell_plan();
    bool inside = false;
    for (size_t i = 0, j = plan.size() - 1; i < plan.size(); j = i++) {
        if ((plan[i].y > apex.y) != (plan[j].y > apex.y) &&
            apex.x < (plan[j].x - plan[i].x) * (apex.y - plan[i].y) /
                             (plan[j].y - plan[i].y) +
                         plan[i].x) {
            inside = !inside;
        }
    }
    CHECK_TRUE(inside);

    // The test only means something if the average really is outside.
    const glm::dvec2 average{8.0 / 3.0, 8.0 / 3.0};
    bool average_inside = false;
    for (size_t i = 0, j = plan.size() - 1; i < plan.size(); j = i++) {
        if ((plan[i].y > average.y) != (plan[j].y > average.y) &&
            average.x < (plan[j].x - plan[i].x) * (average.y - plan[i].y) /
                                (plan[j].y - plan[i].y) +
                            plan[i].x) {
            average_inside = !average_inside;
        }
    }
    CHECK_FALSE(average_inside);
}

/**
 * @brief A pyramid over a plan its apex cannot see is hipped, and says so
 *
 * The defect this test exists for: being on the skeleton's deepest node puts the
 * apex INSIDE the plan, and inside is not enough. On the U the apex sits in one
 * arm or in the base, and the far arm is round a corner from it -- so a triangle
 * raised from the far arm's eave to the apex crosses the notch, which is not
 * part of the building, and overlaps the triangles beside it.
 *
 * Nothing else in this file can see that. The solid stays closed, every face
 * stays flat, and geometry_volume() returns a number that looks like a roof.
 * What it fails is COVERAGE: the emitted faces shade 52 of ground over a
 * footprint that is 32. So that is the assertion, and it is checked against the
 * plan's own area, computed from the plan this file wrote.
 *
 * The answer is the hip -- the lower envelope, which is exact over any simple
 * footprint -- reported rather than substituted silently. Asserting the result
 * is vertex-for-vertex the hip is what says the fallback is the whole fallback
 * and not a pyramid with its worst triangle dropped.
 *
 * The U's hip volume is arithmetic, not a recording. Offsetting the U inward by
 * `s` leaves `A(s) = 32 - 36s + 4s²`, which reaches zero at `s = 1` -- the
 * inradius. A hip at 45 degrees encloses `∫A(s)ds` over [0, 1] = `32 - 18 + 4/3
 * = 46/3`.
 */
TEST(OpRoof, a_pyramid_over_a_plan_its_apex_cannot_see_is_hipped_and_said) {
    Shape shape = make_block(u_plan(), 5.0);
    const RoofReport report = roof_ok(shape, params_of(RoofKind::Pyramid, 45.0));

    CHECK_TRUE(has_note(report, "star-shaped"));
    CHECK_TRUE(has_note(report, "hip"));
    CHECK_NEAR(report.rise, 1.0, 1e-9);

    // The ground under the roof is the footprint's own, covered once.
    CHECK_NEAR(upward_plan_area(shape), 32.0, 1e-9);
    CHECK_TRUE(is_watertight(shape));
    CHECK_NEAR(geometry_volume(shape.geometry), 32.0 * 5.0 + 46.0 / 3.0, 1e-9);

    // And it IS the hip, vertex for vertex.
    Shape hipped = make_block(u_plan(), 5.0);
    const RoofReport hip_report = roof_ok(hipped, params_of(RoofKind::Hip, 45.0));
    CHECK_EQ(hip_report.notes.size(), size_t{0});
    CHECK_TRUE(same_points(shape, hipped, 1e-12));

    // A hip has a ridge over a U, so the fallback cannot have left one apex.
    CHECK_TRUE(distinct_peak_points(shape) > 1);
}

/**
 * @brief A dome over a plan its centre cannot see is hipped, and says so
 *
 * The same defect in the same place. A dome band is the outline scaled about the
 * deepest point, and a scaled outline only stays inside the plan when the plan
 * is star-shaped from that point -- so over the U the inner bands cross the
 * notch exactly as the pyramid's triangles do. The coverage number is the one
 * that moves: 51.33 of ground over a footprint of 32.
 */
TEST(OpRoof, a_dome_over_a_plan_its_centre_cannot_see_is_hipped_and_said) {
    Shape shape = make_block(u_plan(), 5.0);
    RoofParams params = params_of(RoofKind::Dome, 45.0);
    params.has_extra = true;
    params.extra = 8.0;
    const RoofReport report = roof_ok(shape, params);

    CHECK_TRUE(has_note(report, "star-shaped"));
    CHECK_NEAR(upward_plan_area(shape), 32.0, 1e-9);
    CHECK_TRUE(is_watertight(shape));
    CHECK_NEAR(geometry_volume(shape.geometry), 32.0 * 5.0 + 46.0 / 3.0, 1e-9);

    Shape hipped = make_block(u_plan(), 5.0);
    roof_ok(hipped, params_of(RoofKind::Hip, 45.0));
    CHECK_TRUE(same_points(shape, hipped, 1e-12));

    // A dome that DID build keeps its bands: over a square, which every point
    // of the plan can see, eight bands are eight bands and nothing is reported.
    Shape square = make_box(4.0, 4.0, 3.0);
    const RoofReport fine = roof_ok(square, params);
    CHECK_FALSE(has_note(fine, "star-shaped"));
    CHECK_EQ(fine.roof_faces, size_t{4 * 8});
    CHECK_TRUE(geometry_volume(square.geometry) > 4.0 * 4.0 * 3.0 + 16.0 * 2.0 / 3.0);
}

/**
 * @brief Every face a roof emits lies on one plane
 *
 * earcut triangulates a face in the face's own plane, so a face whose corners
 * are not on one comes out as warped triangles -- an artifact that no area,
 * volume or coverage assertion in this file can see, and which `ridge` produces
 * for every contour edge that is neither parallel nor perpendicular to its ridge
 * unless that quad is split. Checked over all six kinds and over a plan chosen
 * so that the slanted case actually occurs.
 */
TEST(OpRoof, every_face_a_roof_emits_is_flat) {
    const std::vector<glm::dvec2> slanted = {{0.0, 0.0}, {8.0, 0.0}, {8.0, 4.0}, {4.0, 4.0}};

    for (const RoofKind kind : {RoofKind::Gable, RoofKind::Hip, RoofKind::Pyramid,
                                RoofKind::Ridge, RoofKind::Shed, RoofKind::Dome}) {
        Shape shape = shape_from_polygon(slanted);
        const OpResult lift = extrude_shape(shape, ScopeAxis::Y, 5.0);
        CHECK_TRUE(lift.ok);
        roof_ok(shape, params_of(kind, 40.0));

        for (const Face& face : shape.geometry.faces) {
            const glm::dvec3 normal = face_normal(shape.geometry, face);
            if (glm::dot(normal, normal) < 0.25) {
                continue;
            }
            const double offset = glm::dot(normal, shape.geometry.positions[face.loop.front()]);
            for (const uint32_t index : face.loop) {
                CHECK_NEAR(glm::dot(normal, shape.geometry.positions[index]), offset, 1e-9);
            }
        }
    }
}

/**
 * @brief A ridge off a rectangle still closes, and says that it over-covers
 *
 * Two separate claims, and the first is the defect this test exists for.
 *
 * `ridge` projects each contour edge's two endpoints onto the ridge segment.
 * Off a rectangle the two sides of the ridge produce DIFFERENT sets of
 * projections -- the trapezoid's slanted side contributes a point at 4 that the
 * straight side opposite does not -- so one slope's top edge used to run the
 * whole way past the end of the slope facing it, and the ridge line acquired a
 * T-junction. is_watertight() counts undirected edges, so it sees it: the long
 * edge appears once and the two short ones appear once each.
 *
 * That is why this test asserts watertightness for `ridge` on plans that are not
 * rectangles. `every_face_a_roof_emits_is_flat` already drives `ridge` over the
 * trapezoid and looks only at flatness, which the broken version passed.
 *
 * The second claim is the documented limit, now said out loud. A ridge is a
 * projection and not a lower envelope, so off a rectangle its slopes reach the
 * ridge across ground that belongs to another slope. The note is what tells an
 * author who wanted `gable`, and the rectangle is here to show the note is not
 * unconditional.
 */
TEST(OpRoof, a_ridge_off_a_rectangle_closes_and_reports_that_it_over_covers) {
    // The trapezoid: projections at 0, 4 and 8, but only from three of its four
    // corners. This is the plan the old ridge left a T-junction on.
    Shape trapezoid = make_block(trapezoid_plan(), 5.0);
    const RoofReport trapezoid_report = roof_ok(trapezoid, params_of(RoofKind::Ridge, 45.0));
    CHECK_TRUE(is_watertight(trapezoid));
    CHECK_TRUE(has_note(trapezoid_report, "over this footprint covers"));
    CHECK_TRUE(has_note(trapezoid_report, "gable"));

    // The L: five distinct projections, and a ridge line crossed by both arms.
    Shape ell = make_block(ell_plan(), 5.0);
    const RoofReport ell_report = roof_ok(ell, params_of(RoofKind::Ridge, 45.0));
    CHECK_TRUE(is_watertight(ell));
    CHECK_TRUE(has_note(ell_report, "over this footprint covers"));

    // The U, which the L does not subsume: its ridge runs through the notch.
    Shape u = make_block(u_plan(), 5.0);
    const RoofReport u_report = roof_ok(u, params_of(RoofKind::Ridge, 45.0));
    CHECK_TRUE(is_watertight(u));
    CHECK_TRUE(has_note(u_report, "over this footprint covers"));

    // Over a rectangle a ridge IS the lower envelope, so it covers the plan
    // exactly once and nothing is reported. Without this the note above would
    // pass for an implementation that reported on every footprint.
    Shape rectangle = make_box(6.0, 4.0, 10.0);
    const RoofReport rectangle_report = roof_ok(rectangle, params_of(RoofKind::Ridge, 45.0));
    CHECK_EQ(rectangle_report.notes.size(), size_t{0});
    CHECK_TRUE(is_watertight(rectangle));
    CHECK_NEAR(upward_plan_area(rectangle), 24.0, 1e-9);
}

/**
 * @brief A slope that spans other corners' projections is split all the way round
 *
 * The long_slant plan's bottom edge runs the whole length of the ridge, is
 * slanted -- so its corners are not on one plane -- and has a third corner
 * projecting into the middle of its span. Its slope is therefore a FIVE-cornered
 * ring that has to become three triangles. Splitting it as if it were a quad
 * emits two, drops the third, and leaves edges belonging to one face.
 *
 * Driven with the ridge along +x, because the plan's own longest edge is the
 * slanted one and a ridge parallel to a slope makes it flat again.
 */
TEST(OpRoof, a_slope_that_spans_other_corners_is_split_all_the_way_round) {
    Shape shape = make_block(long_slant_plan(), 5.0);
    RoofParams params = params_of(RoofKind::Ridge, 45.0);
    params.has_extra = true;
    params.extra = 0.0;  // ridge along +x
    roof_ok(shape, params);

    CHECK_TRUE(is_watertight(shape));

    // And every triangle the fan emitted is flat, which is what the split is for.
    for (const Face& face : shape.geometry.faces) {
        const glm::dvec3 normal = face_normal(shape.geometry, face);
        if (glm::dot(normal, normal) < 0.25) {
            continue;
        }
        const double offset = glm::dot(normal, shape.geometry.positions[face.loop.front()]);
        for (const uint32_t index : face.loop) {
            CHECK_NEAR(glm::dot(normal, shape.geometry.positions[index]), offset, 1e-9);
        }
    }
}

/**
 * @brief Two corners that project onto nearly the same place become one ridge point
 *
 * The near_pair plan is an 8 by 4 rectangle with two extra corners whose
 * projections differ by 5e-13. Welded, the ridge has three points and the two
 * slopes meet at the same doubles. Unwelded, it has four, two of them a
 * half-picometre apart -- a crack that is closed only until something rounds.
 *
 * is_watertight() cannot fail for it: it quantises positions at 1e-7, a hundred
 * thousand times coarser than the gap. So the assertion is the finer one, and
 * the ordinary roof numbers are here too so that a weld that ate a real station
 * would be caught as well.
 */
TEST(OpRoof, two_corners_projecting_onto_one_place_make_one_ridge_point) {
    Shape shape = make_block(near_pair_plan(), 5.0);
    const RoofReport report = roof_ok(shape, params_of(RoofKind::Ridge, 45.0));

    // An 8 by 4 plan is a rectangle whatever its corner count, so the ridge is
    // exact: rise 2, a full-length ridge, and a prism of 8 * (4 * 2 / 2) = 32.
    CHECK_NEAR(report.rise, 2.0, 1e-9);
    CHECK_NEAR(peak(shape), 7.0, 1e-9);
    CHECK_EQ(report.notes.size(), size_t{0});
    CHECK_TRUE(is_watertight(shape));
    CHECK_NEAR(geometry_volume(shape.geometry), 8.0 * 4.0 * 5.0 + 32.0, 1e-9);

    CHECK_EQ(nearly_coincident_at(shape, 7.0, 1e-9), size_t{0});
}

/**
 * @brief The fourth argument turns a gable and a ridge, and the turn is in the solid
 *
 * `the_fourth_argument_turns_a_shed` makes this case for `shed` and for no other
 * kind, so resolve_extra() could ignore params.has_extra for Gable and Ridge and
 * the whole suite would still pass. It is the same argument as there: an
 * argument that is quietly dropped looks exactly like a working one.
 *
 * The 6 by 4 box again. The default ridge follows the principal axis, the long
 * one, so the span across is 4 and a 45-degree roof rises 2 over a prism of
 * `6 * (4 * 2 / 2) = 24`. Turning the ridge 90 degrees makes the span across 6,
 * the rise 3, and the prism `4 * (6 * 3 / 2) = 36`. Two different numbers, both
 * written out from the box this test builds.
 */
TEST(OpRoof, the_fourth_argument_turns_a_gable_and_a_ridge) {
    for (const RoofKind kind : {RoofKind::Gable, RoofKind::Ridge}) {
        Shape along_x = make_box(6.0, 4.0, 10.0);
        const RoofReport default_report = roof_ok(along_x, params_of(kind, 45.0));
        CHECK_NEAR(default_report.rise, 2.0, 1e-9);
        CHECK_NEAR(peak(along_x), 12.0, 1e-9);
        CHECK_TRUE(is_watertight(along_x));
        CHECK_NEAR(geometry_volume(along_x.geometry), 240.0 + 24.0, 1e-9);

        Shape along_z = make_box(6.0, 4.0, 10.0);
        RoofParams turned = params_of(kind, 45.0);
        turned.has_extra = true;
        turned.extra = 90.0;  // ridge along +z instead of +x
        const RoofReport turned_report = roof_ok(along_z, turned);
        CHECK_NEAR(turned_report.rise, 3.0, 1e-9);
        CHECK_NEAR(peak(along_z), 13.0, 1e-9);
        CHECK_TRUE(is_watertight(along_z));
        CHECK_NEAR(geometry_volume(along_z.geometry), 240.0 + 36.0, 1e-9);

        // The ridge now runs across the box, so the vertices at the peak span
        // the 4-metre side and not the 6-metre one.
        double lowest = 1.0e300;
        double highest = -1.0e300;
        for (const glm::dvec3& p : along_z.geometry.positions) {
            if (std::fabs(p.y - 13.0) < 1e-9) {
                lowest = std::min(lowest, p.z);
                highest = std::max(highest, p.z);
            }
        }
        CHECK_NEAR(lowest, 0.0, 1e-9);
        CHECK_NEAR(highest, 4.0, 1e-9);
    }
}

/**
 * @brief A gable wall takes only the profile points that are on its own edge
 *
 * The comb's two prongs put two DIFFERENT contour edges on one line, x = 0.
 * `gable` picks the upper one as an end and clips the enlarged skeleton with
 * that line, so the lower prong's own slope has vertices exactly on the cut.
 * Two things have to be right about those vertices and neither had a test:
 *
 *   - clip_half_plane() must KEEP a vertex that is already on the line. Dropping
 *     it takes the corner out of the lower prong's slope.
 *   - the gable wall must not take it as a profile point. It is on the line and
 *     not on the edge, so a wall built through it runs out past its own corners
 *     and down the far prong.
 *
 * Both show up here as ground covered wrongly and as edges that do not pair up,
 * which is what this asserts. The last assertion is the direct one: every vertex
 * of the wall on x = 0 lies between that end's own two corners.
 */
TEST(OpRoof, a_gable_wall_takes_only_the_profile_points_on_its_own_edge) {
    Shape shape = make_block(comb_plan(), 5.0);
    const RoofReport report = roof_ok(shape, params_of(RoofKind::Gable, 45.0));

    CHECK_EQ(report.gable_walls, size_t{2});
    CHECK_TRUE(is_watertight(shape));
    CHECK_NEAR(upward_plan_area(shape), 28.0, 1e-9);

    // The end on x = 0 is the edge from (0,6) to (0,4). Its wall is the only
    // thing above the eave on that plane, and it must stay between z = 4 and
    // z = 6 -- not reach down to the other prong at z = 2 and z = 0.
    size_t above_the_eave = 0;
    for (const glm::dvec3& p : shape.geometry.positions) {
        if (std::fabs(p.x) > 1e-9 || p.y <= 5.0 + 1e-9) {
            continue;
        }
        ++above_the_eave;
        CHECK_TRUE(p.z > 4.0 - 1e-9);
        CHECK_TRUE(p.z < 6.0 + 1e-9);
    }
    CHECK_TRUE(above_the_eave > 0);
}

/**
 * @brief A gable end whose line cuts the footprint is reported and left hipped
 *
 * The notched plan's left end edge has part of the footprint beyond its own
 * line, so the push-and-clip construction would clip away roof that belongs
 * there. That end is refused, SAID to be refused, and built as a hip; the other
 * end is gabled as usual. The coverage assertion is what says the partial answer
 * is still a whole roof.
 */
TEST(OpRoof, a_gable_end_that_cuts_the_footprint_is_reported_and_hipped) {
    Shape shape = shape_from_polygon(notched_plan());
    const OpResult lift = extrude_shape(shape, ScopeAxis::Y, 5.0);
    CHECK_TRUE(lift.ok);

    const RoofReport report = roof_ok(shape, params_of(RoofKind::Gable, 45.0));
    CHECK_TRUE(has_note(report, "lies on both sides"));
    CHECK_EQ(report.gable_walls, size_t{1});

    CHECK_NEAR(upward_plan_area(shape), 22.0, 1e-9);
    CHECK_TRUE(is_watertight(shape));
}

/**
 * @brief Two gable ends that share a corner cannot both be gabled, and it is said
 *
 * A triangle's two non-base edges meet, so pushing both would move each other's
 * corners. One is kept, the other is reported, and the roof still builds.
 */
TEST(OpRoof, two_gable_ends_that_share_a_corner_are_reported) {
    const std::vector<glm::dvec2> triangle = {{0.0, 0.0}, {6.0, 0.0}, {3.0, 5.0}};
    Shape shape = shape_from_polygon(triangle);
    const OpResult lift = extrude_shape(shape, ScopeAxis::Y, 3.0);
    CHECK_TRUE(lift.ok);

    const RoofReport report = roof_ok(shape, params_of(RoofKind::Gable, 30.0));
    CHECK_TRUE(has_note(report, "shares a corner"));
    CHECK_EQ(report.gable_walls, size_t{1});
    CHECK_NEAR(upward_plan_area(shape), 15.0, 1e-9);
    CHECK_TRUE(is_watertight(shape));
}

// ============================================================================
// Where the roof sits
// ============================================================================

/**
 * @brief Yawing the scope does not move the roof
 *
 * `rotate_scope` turns the frame and leaves the geometry in the world, so a
 * building whose scope has been yawed 90 degrees is the same building in the
 * same place described in different local coordinates. Its roof must therefore
 * come out in the same place, vertex for vertex, in WORLD space.
 *
 * This is the assertion behind "the roof sits on the shape rather than on the
 * world". An implementation that read the footprint in world coordinates would
 * pass every other test in this file and fail this one, because the footprint's
 * long axis is at 90 degrees to the frame's x after the yaw.
 */
TEST(OpRoof, yawing_the_scope_does_not_move_the_roof) {
    Shape plain = make_box(6.0, 4.0, 10.0);
    Shape yawed = make_box(6.0, 4.0, 10.0);
    rotate_scope_shape(yawed, glm::dvec3{0.0, 90.0, 0.0});

    roof_ok(plain, params_of(RoofKind::Gable, 40.0));
    roof_ok(yawed, params_of(RoofKind::Gable, 40.0));

    CHECK_TRUE(same_points(plain, yawed, 1e-9));
    CHECK_NEAR(geometry_volume(plain.geometry), geometry_volume(yawed.geometry), 1e-9);
    CHECK_TRUE(is_watertight(yawed));
}

/**
 * @brief A footprint turned within its own frame keeps its own roof
 *
 * `rotate_shape` moves the geometry and leaves the frame, which is the opposite
 * of the test above and catches the opposite mistake. A roof built over the
 * scope's bounding box rather than over the outline would gain volume as the
 * rectangle turns -- a 6 by 4 rectangle at 30 degrees has a bounding box of
 * about 7.2 by 6.5 -- so asserting the volume is UNCHANGED is the assertion that
 * the outline, and not the box, was roofed.
 */
TEST(OpRoof, a_footprint_turned_within_its_frame_keeps_its_own_roof) {
    Shape straight = make_box(6.0, 4.0, 10.0);
    Shape turned = make_box(6.0, 4.0, 10.0);
    rotate_shape(turned, glm::dvec3{0.0, 30.0, 0.0});

    roof_ok(straight, params_of(RoofKind::Hip, 45.0));
    const RoofReport report = roof_ok(turned, params_of(RoofKind::Hip, 45.0));

    CHECK_NEAR(report.rise, 2.0, 1e-9);
    CHECK_NEAR(geometry_volume(turned.geometry), geometry_volume(straight.geometry), 1e-9);
    CHECK_NEAR(upward_plan_area(turned), 24.0, 1e-9);
    CHECK_TRUE(is_watertight(turned));

    // The turn really happened: the scope of the turned box is its bounding box,
    // which is bigger than 6 by 4 in both directions.
    CHECK_TRUE(turned.scope.size.x > 6.5);
    CHECK_TRUE(turned.scope.size.z > 5.5);
}

/**
 * @brief A frame that is not level is refused, and the message names the fix
 *
 * Tilting the scope about z leaves the box where it is and makes its top cap
 * point somewhere other than local +y. There is then no level face to raise a
 * roof on, and guessing one would put the roof and the wall top in different
 * places. The message names align_scope, which is the operation that turns the
 * frame back.
 */
TEST(OpRoof, a_frame_that_is_not_level_is_refused_with_the_fix_in_the_message) {
    Shape shape = make_box(6.0, 4.0, 10.0);
    rotate_scope_shape(shape, glm::dvec3{0.0, 0.0, 30.0});

    const std::string why = roof_refusal(shape, params_of(RoofKind::Hip, 45.0));
    CHECK_TRUE(why.find("no upward face") != std::string::npos);
    CHECK_TRUE(why.find("align_scope") != std::string::npos);
}

// ============================================================================
// What closes the solid
// ============================================================================

/**
 * @brief A bare footprint gets a floor; a box does not get a partition
 *
 * The two halves of one rule, tested together because each is the other's
 * counter-example. A footprint out of shape_from_polygon() is closed by nothing,
 * so the roof needs a floor or it is an open shell. An extruded box's top cap is
 * closed by its walls, so a floor there would leave a partition inside the
 * building and make the volume wrong by the cap's own contribution -- and would
 * still be watertight, which is why the volume is asserted as well.
 */
TEST(OpRoof, a_bare_footprint_gets_a_floor_and_a_box_does_not_get_a_partition) {
    Shape sheet = shape_from_rect(4.0, 4.0);
    roof_ok(sheet, params_of(RoofKind::Hip, 45.0));
    CHECK_TRUE(is_watertight(sheet));
    CHECK_NEAR(geometry_volume(sheet.geometry), 16.0 * 2.0 / 3.0, 1e-9);
    // Four slopes and one floor. The floor is the only flat face at the eave.
    CHECK_EQ(faces_flat_at(sheet, 0.0), size_t{1});

    Shape box = make_box(4.0, 4.0, 3.0);
    roof_ok(box, params_of(RoofKind::Hip, 45.0));
    CHECK_TRUE(is_watertight(box));
    CHECK_NEAR(geometry_volume(box.geometry), 4.0 * 4.0 * 3.0 + 16.0 * 2.0 / 3.0, 1e-9);
    // Nothing flat left at the wall top: the cap went and no floor replaced it.
    CHECK_EQ(faces_flat_at(box, 3.0), size_t{0});
    // The base is still there, and is the only flat face at the ground.
    CHECK_EQ(faces_flat_at(box, 0.0), size_t{1});
}

/**
 * @brief An overhang oversails the walls, and the soffit keeps the solid closed
 *
 * A metre of overhang on the 6 by 4 box roofs an 8 by 6 outline instead, so the
 * rise becomes 3, the peak 13, and the roof holds `6*3/6 * (24 - 6) = 54`. The
 * ring between the wall top and the eave is emitted facing down, and the solid
 * is the box plus the whole roof mass -- which is only true if that ring is
 * there, so the volume and the watertightness assertions are testing the soffit
 * as much as the roof.
 */
TEST(OpRoof, an_overhang_oversails_the_walls_and_the_soffit_closes_the_solid) {
    Shape shape = make_box(6.0, 4.0, 10.0);
    RoofParams params = params_of(RoofKind::Hip, 45.0);
    params.overhang = 1.0;
    const RoofReport report = roof_ok(shape, params);

    CHECK_NEAR(report.rise, 3.0, 1e-9);
    CHECK_NEAR(peak(shape), 13.0, 1e-9);
    CHECK_NEAR(shape.scope.size.x, 8.0, 1e-9);
    CHECK_NEAR(shape.scope.size.y, 13.0, 1e-9);
    CHECK_NEAR(shape.scope.size.z, 6.0, 1e-9);

    CHECK_TRUE(is_watertight(shape));
    CHECK_NEAR(geometry_volume(shape.geometry), 240.0 + hip_volume(8.0, 6.0, 3.0), 1e-9);
    CHECK_NEAR(geometry_volume(shape.geometry), 240.0 + 54.0, 1e-9);

    // The roof covers the OFFSET outline, not the walls'.
    CHECK_NEAR(upward_plan_area(shape), 48.0, 1e-9);
    // One flat face at the eave: the soffit ring, 8 by 6 less 6 by 4.
    CHECK_EQ(faces_flat_at(shape, 10.0), size_t{1});
}

/**
 * @brief A negative overhang insets the roof and leaves a ledge
 *
 * The mirror of the test above, and it exercises the other sign of the same
 * mitre. A metre of inset roofs a 4 by 2 outline: the rise is 1, the peak 11,
 * and the roof holds `2*1/6 * (12 - 2) = 10/3`. The ring is now emitted facing
 * UP, because what is exposed is a ledge and not a soffit.
 */
TEST(OpRoof, a_negative_overhang_insets_the_roof_and_leaves_a_ledge) {
    Shape shape = make_box(6.0, 4.0, 10.0);
    RoofParams params = params_of(RoofKind::Hip, 45.0);
    params.overhang = -1.0;
    const RoofReport report = roof_ok(shape, params);

    CHECK_NEAR(report.rise, 1.0, 1e-9);
    CHECK_NEAR(peak(shape), 11.0, 1e-9);
    CHECK_NEAR(shape.scope.size.x, 6.0, 1e-9);
    CHECK_NEAR(shape.scope.size.y, 11.0, 1e-9);

    CHECK_TRUE(is_watertight(shape));
    CHECK_NEAR(geometry_volume(shape.geometry), 240.0 + hip_volume(4.0, 2.0, 1.0), 1e-9);
    CHECK_NEAR(geometry_volume(shape.geometry), 240.0 + 10.0 / 3.0, 1e-9);
    CHECK_NEAR(upward_plan_area(shape), 8.0 + 24.0 - 8.0, 1e-9);
}

/**
 * @brief An inset that eats the whole footprint is refused by name
 *
 * A 2 by 2 square insets to nothing at one metre, so an inset of 1.5 folds it
 * through itself. That is "a footprint too small for what was asked", and the
 * only answers are a named refusal or a knot.
 */
TEST(OpRoof, an_inset_that_eats_the_footprint_is_refused) {
    Shape shape = make_box(2.0, 2.0, 4.0);
    RoofParams params = params_of(RoofKind::Hip, 45.0);
    params.overhang = -1.5;

    const std::string why = roof_refusal(shape, params);
    CHECK_TRUE(why.find("overhang") != std::string::npos);
    CHECK_TRUE(why.find("-1.5") != std::string::npos);

    // And the shape was left exactly as it was: a roof that fails half way
    // through is a hole in a building, not a partial answer.
    RoofReport report;
    Shape untouched = make_box(2.0, 2.0, 4.0);
    const OpResult result = roof_shape(untouched, params, report);
    CHECK_FALSE(result.ok);
    CHECK_EQ(untouched.geometry.faces.size(), make_box(2.0, 2.0, 4.0).geometry.faces.size());
    CHECK_NEAR(geometry_volume(untouched.geometry), 16.0, 1e-9);
}

/**
 * @brief A refusal raised inside a kind builder leaves the shape untouched too
 *
 * The header promises the shape is UNTOUCHED on failure, and
 * `an_inset_that_eats_the_footprint_is_refused` checks it for one refusal only:
 * the one offset_ring() raises before any face is built. Every other refusal is
 * raised from inside a kind builder, AFTER that builder has appended its faces
 * to the geometry being assembled beside the old one -- so the assertion that
 * the assembled geometry is not swapped in on that path was the one missing.
 *
 * A pitch of a billionth of a degree over a 6 by 4 box gives a roof that rises
 * 3.5e-11, which every kind refuses as too small for the footprint, each from
 * inside its own builder. dump_shape() prints every vertex, so comparing it
 * before and after is the whole shape and not a face count.
 */
TEST(OpRoof, a_refusal_inside_a_kind_builder_leaves_the_shape_untouched) {
    for (const RoofKind kind : {RoofKind::Gable, RoofKind::Hip, RoofKind::Pyramid,
                                RoofKind::Ridge, RoofKind::Shed, RoofKind::Dome}) {
        Shape shape = make_box(6.0, 4.0, 10.0);
        const std::string before = dump_shape(shape);

        RoofReport report;
        const OpResult result = roof_shape(shape, params_of(kind, 1.0e-9), report);
        CHECK_FALSE(result.ok);
        CHECK_TRUE(result.message.find("too small") != std::string::npos);

        CHECK_EQ(dump_shape(shape), before);
        CHECK_EQ(shape.geometry.faces.size(), size_t{6});
        CHECK_NEAR(geometry_volume(shape.geometry), 240.0, 1e-9);
    }

    // And the same pitch over a footprint big enough for it does build, so the
    // refusal is about the footprint and not about every small pitch.
    Shape large = make_box(6.0e9, 4.0e9, 1.0);
    const RoofReport report = roof_ok(large, params_of(RoofKind::Hip, 1.0e-9));
    CHECK_TRUE(report.rise > 0.0);
}

/**
 * @brief A dome with no fourth argument gets kDefaultDomeBands, not some other count
 *
 * Every dome test until now passed the band count in, so the default could have
 * been any number at all. Two comparisons pin it: a dome with no argument is
 * vertex for vertex a dome asking for 6, and is NOT a dome asking for 3.
 */
TEST(OpRoof, a_dome_with_no_fourth_argument_uses_the_default_band_count) {
    Shape implied = make_box(4.0, 4.0, 3.0);
    const RoofReport implied_report = roof_ok(implied, params_of(RoofKind::Dome, 45.0));

    Shape stated = make_box(4.0, 4.0, 3.0);
    RoofParams params = params_of(RoofKind::Dome, 45.0);
    params.has_extra = true;
    params.extra = 6.0;
    const RoofReport stated_report = roof_ok(stated, params);

    CHECK_EQ(implied_report.roof_faces, stated_report.roof_faces);
    CHECK_EQ(implied_report.roof_faces, size_t{4 * 6});
    CHECK_TRUE(same_points(implied, stated, 1e-12));

    Shape coarser = make_box(4.0, 4.0, 3.0);
    params.extra = 3.0;
    roof_ok(coarser, params);
    CHECK_FALSE(same_points(implied, coarser, 1e-12));
    CHECK_TRUE(geometry_volume(coarser.geometry) < geometry_volume(implied.geometry));
}

/**
 * @brief Every face the roof added is counted, and the extras are counted too
 *
 * RoofReport::roof_faces was asserted nowhere, so it could have been left at
 * zero. The three numbers here are chosen so that each says something different:
 * a hip over a rectangle is its four skeleton faces; an overhang adds exactly
 * one more, the soffit ring; and a footprint with no walls under it adds exactly
 * one more, its floor. Each is a count this test can derive from the plan.
 */
TEST(OpRoof, the_report_counts_every_face_the_roof_added) {
    Shape closed = make_box(6.0, 4.0, 10.0);
    const RoofReport plain = roof_ok(closed, params_of(RoofKind::Hip, 45.0));
    CHECK_EQ(plain.roof_faces, size_t{4});
    CHECK_EQ(plain.footprints, size_t{1});

    Shape oversailing = make_box(6.0, 4.0, 10.0);
    RoofParams params = params_of(RoofKind::Hip, 45.0);
    params.overhang = 1.0;
    const RoofReport with_soffit = roof_ok(oversailing, params);
    CHECK_EQ(with_soffit.roof_faces, plain.roof_faces + 1);

    // A bare footprint out of shape_from_polygon() has no wall under it, so the
    // roof brings its own floor.
    Shape sheet = shape_from_polygon({{0.0, 0.0}, {6.0, 0.0}, {6.0, 4.0}, {0.0, 4.0}});
    const RoofReport with_floor = roof_ok(sheet, params_of(RoofKind::Hip, 45.0));
    CHECK_EQ(with_floor.roof_faces, plain.roof_faces + 1);
    CHECK_TRUE(is_watertight(sheet));

    // A gable's walls are faces the roof added as well, so they are in the count.
    Shape gabled = make_box(6.0, 4.0, 10.0);
    const RoofReport gable = roof_ok(gabled, params_of(RoofKind::Gable, 45.0));
    CHECK_EQ(gable.gable_walls, size_t{2});
    CHECK_EQ(gable.roof_faces, size_t{4});
}

// ============================================================================
// Holes
// ============================================================================

/**
 * @brief Only a shed carries a courtyard, and the others say so
 *
 * straight_skeleton.hpp refuses holes and is emphatic that its faces over one
 * are "land that does not exist", so roofing a courtyard over is not an
 * approximation. A shed is one tilted plane and carries the hole exactly, which
 * the volume assertion checks: an 8 by 8 plan with a 2 by 2 courtyard is 60 of
 * ground, and the wedge over it at 45 degrees across 8 metres is not the wedge
 * over a solid 8 by 8.
 */
TEST(OpRoof, only_a_shed_carries_a_courtyard) {
    const std::vector<glm::dvec2> outer = {{0.0, 0.0}, {8.0, 0.0}, {8.0, 8.0}, {0.0, 8.0}};
    const std::vector<std::vector<glm::dvec2>> holes = {
        {{3.0, 3.0}, {5.0, 3.0}, {5.0, 5.0}, {3.0, 5.0}}};

    for (const RoofKind kind : {RoofKind::Gable, RoofKind::Hip, RoofKind::Pyramid,
                                RoofKind::Ridge, RoofKind::Dome}) {
        Shape shape = shape_from_rings(outer, holes);
        const OpResult lift = extrude_shape(shape, ScopeAxis::Y, 5.0);
        CHECK_TRUE(lift.ok);
        const std::string why = roof_refusal(shape, params_of(kind, 45.0));
        CHECK_TRUE(why.find("interior ring") != std::string::npos);
        CHECK_TRUE(why.find(roof_kind_name(kind)) != std::string::npos);
        CHECK_TRUE(why.find("shed") != std::string::npos);
    }

    Shape shed = shape_from_rings(outer, holes);
    const OpResult lift = extrude_shape(shed, ScopeAxis::Y, 5.0);
    CHECK_TRUE(lift.ok);
    const RoofReport report = roof_ok(shed, params_of(RoofKind::Shed, 45.0));
    CHECK_EQ(report.holes_carried, size_t{1});
    CHECK_TRUE(is_watertight(shed));

    // 60 of ground under the roof, and the courtyard is still a hole in it.
    CHECK_NEAR(upward_plan_area(shed), 60.0, 1e-9);

    // The tilted plane over 60 of ground, from 0 to 8 of rise: the solid below
    // the plane over the outer square is 8 * 8 * 8 / 2 = 256, less the prism the
    // courtyard cuts out of it, which spans z in [3, 5] where the lift runs 3 to
    // 5, so 2 * 2 * 4 = 16.
    const double wedge = 256.0 - 16.0;
    CHECK_NEAR(geometry_volume(shed.geometry), 60.0 * 5.0 + wedge, 1e-9);
}

/**
 * @brief An overhang over a courtyard is refused, because nothing says which way it goes
 *
 * `shed` is the one kind that carries interior rings, so it is the only kind
 * that can reach this refusal at all -- every other kind has already refused the
 * courtyard. An oversail moves the outer outline out; the courtyard would have
 * to move in by the same amount for the roof to stay a constant width, and
 * nothing in the signature says so. Refused by name rather than guessed at.
 */
TEST(OpRoof, an_overhang_on_a_footprint_with_a_courtyard_is_refused) {
    const std::vector<glm::dvec2> outer = {{0.0, 0.0}, {8.0, 0.0}, {8.0, 8.0}, {0.0, 8.0}};
    const std::vector<std::vector<glm::dvec2>> holes = {
        {{3.0, 3.0}, {5.0, 3.0}, {5.0, 5.0}, {3.0, 5.0}}};

    for (const double overhang : {0.5, -0.5}) {
        Shape shape = shape_from_rings(outer, holes);
        const OpResult lift = extrude_shape(shape, ScopeAxis::Y, 5.0);
        CHECK_TRUE(lift.ok);

        RoofParams params = params_of(RoofKind::Shed, 45.0);
        params.overhang = overhang;
        const std::string why = roof_refusal(shape, params);
        CHECK_TRUE(why.find("overhang") != std::string::npos);
        CHECK_TRUE(why.find("interior rings") != std::string::npos);
    }

    // The same shed with no overhang builds, so the refusal is about the
    // overhang and not about the courtyard, which the kind does carry.
    Shape shape = shape_from_rings(outer, holes);
    const OpResult lift = extrude_shape(shape, ScopeAxis::Y, 5.0);
    CHECK_TRUE(lift.ok);
    const RoofReport report = roof_ok(shape, params_of(RoofKind::Shed, 45.0));
    CHECK_EQ(report.holes_carried, size_t{1});
}

// ============================================================================
// Refusals
// ============================================================================

TEST(OpRoof, a_pitch_outside_zero_to_ninety_is_refused) {
    for (const double pitch : {0.0, 90.0, -10.0, 120.0}) {
        Shape shape = make_box(6.0, 4.0, 10.0);
        const std::string why = roof_refusal(shape, params_of(RoofKind::Hip, pitch));
        CHECK_TRUE(why.find("pitch") != std::string::npos);
        CHECK_TRUE(why.find("90") != std::string::npos);
    }

    // And a pitch just inside the range still builds, so the bound is a bound
    // and not a whole band of refusals.
    Shape shape = make_box(6.0, 4.0, 10.0);
    const RoofReport report = roof_ok(shape, params_of(RoofKind::Hip, 1.0));
    CHECK_TRUE(report.rise > 0.0);
    CHECK_TRUE(report.rise < 0.1);
}

TEST(OpRoof, a_footprint_too_small_for_the_pitch_is_refused) {
    Shape shape = shape_from_rect(1.0e-9, 1.0e-9);
    const std::string why = roof_refusal(shape, params_of(RoofKind::Hip, 45.0));
    CHECK_TRUE(!why.empty());

    Shape nothing;
    const std::string bare = roof_refusal(nothing, params_of(RoofKind::Hip, 45.0));
    CHECK_TRUE(bare.find("no faces") != std::string::npos);
}

TEST(OpRoof, a_kind_that_cannot_use_a_fourth_argument_reports_it) {
    for (const RoofKind kind : {RoofKind::Hip, RoofKind::Pyramid}) {
        Shape shape = make_box(6.0, 4.0, 10.0);
        RoofParams params = params_of(kind, 45.0);
        params.has_extra = true;
        params.extra = 12.0;
        const RoofReport report = roof_ok(shape, params);
        CHECK_TRUE(has_note(report, "no use for a fourth argument"));
    }

    // A kind that DOES use it says nothing, so the note is about the argument
    // and not about every fourth argument.
    Shape shape = make_box(6.0, 4.0, 10.0);
    RoofParams params = params_of(RoofKind::Gable, 45.0);
    params.has_extra = true;
    params.extra = 90.0;
    const RoofReport report = roof_ok(shape, params);
    CHECK_FALSE(has_note(report, "no use for a fourth argument"));
}

TEST(OpRoof, every_kind_name_round_trips) {
    for (const RoofKind kind : {RoofKind::Gable, RoofKind::Hip, RoofKind::Pyramid,
                                RoofKind::Ridge, RoofKind::Shed, RoofKind::Dome}) {
        RoofKind parsed = RoofKind::Hip;
        CHECK_TRUE(parse_roof_kind(roof_kind_name(kind), parsed));
        CHECK_TRUE(parsed == kind);
    }
    RoofKind unused = RoofKind::Hip;
    CHECK_FALSE(parse_roof_kind("mansard", unused));
    CHECK_FALSE(parse_roof_kind("Hip", unused));
    CHECK_FALSE(parse_roof_kind("", unused));
}

// ============================================================================
// Through a rule file
// ============================================================================

/**
 * @brief A rule file raises a roof on an extruded footprint
 *
 * The end-to-end case: a rule text, a seed rectangle, a building with a roof on
 * it. Everything asserted is computed from the rule text and the seed and
 * nothing is read back from the output.
 *
 * The seed is 6 by 4. `extrude(10.0)` gives the 240 solid. `roof("gable", 45.0)`
 * puts a 2-metre ridge along the long axis over it, holding 24.
 */
TEST(OpRoof, a_rule_file_raises_a_roof_on_an_extruded_footprint) {
    const std::string source =
        "@start\n"
        "rule Main {\n"
        "    extrude(10.0);\n"
        "    roof(\"gable\", 45.0);\n"
        "}\n";

    const GenerationResult result = run_on(source, shape_from_rect(6.0, 4.0));
    CHECK_TRUE(result.ok());
    CHECK_EQ(result.diagnostics.size(), size_t{0});
    CHECK_EQ(result.terminals.size(), size_t{1});
    if (result.terminals.size() != 1) {
        return;
    }

    const Shape& building = result.terminals.front();
    CHECK_NEAR(peak(building), 12.0, 1e-9);
    CHECK_TRUE(is_watertight(building));
    CHECK_NEAR(geometry_volume(building.geometry), 264.0, 1e-9);
    CHECK_NEAR(upward_plan_area(building), 24.0, 1e-9);
}

/**
 * @brief The roof shape is DATA, which is the whole argument for one operation
 *
 * The same rule file, run twice, with only the value of an `attr` changed. Five
 * fixed operation names cannot express this at all, and it is the reason ast.hpp
 * has one catalogue row instead of five. The two runs must produce DIFFERENT
 * solids, or the argument reached nothing.
 */
TEST(OpRoof, the_roof_shape_is_data_and_a_rule_can_compute_it) {
    const std::string source =
        "attr kind : string = \"hip\"\n"
        "@start\n"
        "rule Main {\n"
        "    extrude(10.0);\n"
        "    roof(kind, 45.0);\n"
        "}\n";

    GenerationOptions hip_options;
    hip_options.attributes["kind"] = Value::text("hip");
    const GenerationResult hipped = run_on(source, shape_from_rect(6.0, 4.0), hip_options);

    GenerationOptions gable_options;
    gable_options.attributes["kind"] = Value::text("gable");
    const GenerationResult gabled = run_on(source, shape_from_rect(6.0, 4.0), gable_options);

    CHECK_TRUE(hipped.ok());
    CHECK_TRUE(gabled.ok());
    CHECK_EQ(hipped.terminals.size(), size_t{1});
    CHECK_EQ(gabled.terminals.size(), size_t{1});
    if (hipped.terminals.size() != 1 || gabled.terminals.size() != 1) {
        return;
    }

    CHECK_NEAR(geometry_volume(hipped.terminals.front().geometry), 240.0 + 56.0 / 3.0, 1e-9);
    CHECK_NEAR(geometry_volume(gabled.terminals.front().geometry), 264.0, 1e-9);
    CHECK_TRUE(hipped.dump() != gabled.dump());
}

TEST(OpRoof, an_unknown_roof_shape_is_refused_at_the_line_that_asked) {
    const std::string source =
        "@start\n"
        "rule Main { extrude(10.0); roof(\"mansard\", 45.0); }\n";

    const GenerationResult result = run_on(source, shape_from_rect(6.0, 4.0));
    CHECK_FALSE(result.ok());
    CHECK_TRUE(has_message(result, Severity::Error, "does not know the shape 'mansard'"));
    CHECK_TRUE(has_message(result, Severity::Error, "gable, hip, pyramid, ridge, shed and dome"));
    CHECK_EQ(result.terminals.size(), size_t{0});

    // A first argument of the wrong type is refused too, and by a different
    // sentence: "unknown name" and "wrong type" are different mistakes.
    const std::string numeric =
        "@start\n"
        "rule Main { extrude(10.0); roof(30.0); }\n";
    const GenerationResult typed = run_on(numeric, shape_from_rect(6.0, 4.0));
    CHECK_FALSE(typed.ok());
    CHECK_TRUE(has_message(typed, Severity::Error, "roof shape as its first argument"));
}

TEST(OpRoof, a_note_from_the_geometry_reaches_the_rule_as_a_warning) {
    const std::string source =
        "@start\n"
        "rule Main { extrude(10.0); roof(\"hip\", 45.0, 0.0, 12.0); }\n";

    const GenerationResult result = run_on(source, shape_from_rect(6.0, 4.0));
    // A warning, not an error: the roof was still built.
    CHECK_TRUE(result.ok());
    CHECK_EQ(result.terminals.size(), size_t{1});
    CHECK_TRUE(has_message(result, Severity::Warning, "no use for a fourth argument"));
}

TEST(OpRoof, a_refusal_abandons_the_shape_and_names_the_operation) {
    const std::string source =
        "@start\n"
        "rule Main { extrude(10.0); roof(\"hip\", 0.0); }\n";

    const GenerationResult result = run_on(source, shape_from_rect(6.0, 4.0));
    CHECK_FALSE(result.ok());
    CHECK_TRUE(has_message(result, Severity::Error, "'roof': a pitch of 0"));
    CHECK_EQ(result.terminals.size(), size_t{0});
}

// ============================================================================
// Determinism
// ============================================================================

/**
 * @brief Every kind runs byte for byte the same twice, and differently for a different pitch
 *
 * The first half is the determinism contract. The second half is what stops the
 * first from being vacuous: a dump that did not carry the geometry would be
 * equal across two runs AND across two pitches, and the suite would report a
 * determinism guarantee it had never tested. dump_shape() prints every vertex,
 * and these assertions are what depend on it.
 */
TEST(OpRoof, every_kind_is_reproducible_and_its_pitch_is_visible_in_the_dump) {
    for (const RoofKind kind : {RoofKind::Gable, RoofKind::Hip, RoofKind::Pyramid,
                                RoofKind::Ridge, RoofKind::Shed, RoofKind::Dome}) {
        Shape first = make_box(6.0, 4.0, 10.0);
        Shape second = make_box(6.0, 4.0, 10.0);
        Shape steeper = make_box(6.0, 4.0, 10.0);

        roof_ok(first, params_of(kind, 35.0));
        roof_ok(second, params_of(kind, 35.0));
        roof_ok(steeper, params_of(kind, 55.0));

        const std::string once = dump_shape(first);
        CHECK_EQ(once, dump_shape(second));
        CHECK_TRUE(once.size() > 500);
        CHECK_TRUE(once != dump_shape(steeper));
    }
}

/**
 * @brief The whole generation is reproducible through the rule file too
 */
TEST(OpRoof, a_rule_file_with_a_roof_in_it_dumps_identically_twice) {
    const std::string source =
        "@start\n"
        "rule Main { extrude(9.0); roof(\"hip\", 33.0, 0.4); }\n";

    const GenerationResult first = run_on(source, shape_from_rect(7.0, 5.0));
    const GenerationResult second = run_on(source, shape_from_rect(7.0, 5.0));
    CHECK_TRUE(first.ok());
    CHECK_TRUE(first.dump().size() > 1000);
    CHECK_EQ(first.dump(), second.dump());
}

// ============================================================================
// The remedy the refusal names has to be one that works
// ============================================================================

TEST(OpRoof, the_refusal_on_a_selected_face_names_a_remedy_that_works) {
    // A rule that selects the top face and roofs it is the natural way to write
    // a building, and `roof` cannot serve it: a face picked out by
    // `select face` is a shape whose LOCAL normal lies along z, while the roof
    // rises along local +y.
    //
    // Refusing is right. Naming the wrong cure is not, and this test exists
    // because the refusal used to say align_scope("geometry") -- which does
    // nothing here. Geometry mode points y at the dominant face's normal, and
    // on a shape that IS one face it already points there, so the author
    // follows the advice, gets the identical message, and has nowhere to go.
    const std::string select_then_roof =
        "@start\n"
        "rule Main { extrude(6.0); select face { top : { R(); } } }\n"
        "rule R { roof(\"hip\", 35.0); }\n";

    const GenerationResult refused = run_on(select_then_roof, shape_from_rect(12.0, 8.0));
    CHECK_FALSE(refused.ok());
    CHECK_TRUE(has_message(refused, Severity::Error, "align_scope"));
    CHECK_TRUE(has_message(refused, Severity::Error, "y_up"));
    // The mode that does not work must not be the one advertised.
    CHECK_FALSE(has_message(refused, Severity::Error, "\"geometry\""));

    // And the advertised remedy has to produce a roof.
    const std::string with_remedy =
        "@start\n"
        "rule Main { extrude(6.0); select face { top : { R(); } } }\n"
        "rule R { align_scope(\"y_up\"); roof(\"hip\", 35.0); }\n";

    const GenerationResult fixed = run_on(with_remedy, shape_from_rect(12.0, 8.0));
    CHECK_TRUE(fixed.ok());
    CHECK_EQ(fixed.terminals.size(), size_t{1});
    if (!fixed.terminals.empty()) {
        // Derived, not observed. A hip over 12 x 8 at 35 degrees has its ridge
        // along the long axis, so the span across is 8 and the rise is
        // 4 * tan(35) = 2.800832. The solid is a prismatoid: at half height the
        // cross section is (12-4) x (8-4) = 32, and the ridge itself has no
        // area, so V = h/6 * (96 + 4*32 + 0) = 104.5644.
        const double rise = 4.0 * std::tan(35.0 * 3.14159265358979323846 / 180.0);
        const double expected = rise / 6.0 * (96.0 + 4.0 * 32.0);
        CHECK_NEAR(geometry_volume(fixed.terminals[0].geometry), expected, 1e-6);
    }
}
