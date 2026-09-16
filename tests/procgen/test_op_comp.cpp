// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_op_comp.cpp
 * @brief The component split (D4): selection, the canonical order, and the oriented scopes
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ================================================================================
 * HOW THESE TESTS ARE WRITTEN SO THAT THEY CAN FAIL
 * ================================================================================
 *
 * The brief for this feature says the two subtle faults are a component scope
 * that is not oriented to its component, and an index order that is not stable.
 * Both produce output that LOOKS right: a facade whose storeys run across the
 * world instead of across the wall is still a facade until the building is
 * rotated, and an index order that follows the geometry buffer is stable right
 * up until someone changes the extruder. A test that counted components would
 * pass for both. So:
 *
 *   - **Every axis of every component scope is asserted, as a vector.** The
 *     front, right, top and bottom faces of one box are checked against
 *     hand-computed frames, because the four disagree with each other: an
 *     implementation that handed back the parent's axes passes on the front face
 *     and fails on the other three.
 *
 *   - **The order is asserted against a PERMUTED geometry buffer.** The same box
 *     is split twice, once with its faces reversed, and the two component lists
 *     must agree component for component. Asserting the order of an unpermuted
 *     buffer is a test that cannot fail, and this project's most-found defect.
 *
 *   - **The oriented scope is checked against WORLD positions.** A component's
 *     vertices must land in the world exactly where the parent's face was.
 *     Checking the local coordinates alone would pass for an implementation that
 *     turned the frame and dragged the geometry with it.
 *
 *   - **"Relative to the scope" is tested both ways.** One box is given a
 *     rotated frame with its local geometry untouched, and must keep the same
 *     front. A second has `rotate_scope` applied, and must CHANGE which face is
 *     the front. An implementation that read world normals fails the first; one
 *     that ignored the scope entirely fails the second.
 *
 *   - **Selection words are tested where they disagree.** `side` and `vertical`
 *     agree on a box, so the discriminating case is a frustum, whose four walls
 *     are sides and are not vertical.
 *
 *   - **The awkward geometry is built by hand, not hoped for.** A face with a
 *     hole, a zero-area face, and a quad with one corner lifted out of plane are
 *     each constructed and each asserted on.
 *
 *   - **The ordering fixture has REPEATED normals.** A box has six faces with
 *     six distinct normals, so its order is decided by the normal key alone and
 *     the centroid -- which op_comp.hpp calls the first key -- never has to do
 *     anything: every ordering assertion made on a box survives deleting the
 *     centroid from the comparison. make_u_block() is a U-shaped footprint
 *     extruded three metres, whose ten faces include five pairs that share a
 *     normal, and the order asserted on it is the whole key.
 *
 *   - **The emit path is asserted, severity by severity.** emit_components() is
 *     the one place where op_comp.hpp's "nothing here is silent" has to hold, so
 *     each fault that can reach a rule author -- an unimplemented domain, an
 *     index past the end, a shape with no geometry -- is run from a rule file
 *     and asserted to be an Error that fails the run, and a degenerate face is
 *     asserted to be a Warning that does not.
 *
 * Nothing here needs a GPU, a window or a file on disk, so nothing here skips.
 */

#include "framework.hpp"

#include "procgen/rules/ast.hpp"
#include "procgen/rules/interpreter.hpp"
#include "procgen/rules/lexer.hpp"
#include "procgen/rules/op_comp.hpp"
#include "procgen/rules/parser.hpp"
#include "procgen/rules/shape.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

using stratum::procgen::rules::Component;
using stratum::procgen::rules::component_direction;
using stratum::procgen::rules::ComponentDomain;
using stratum::procgen::rules::component_functions;
using stratum::procgen::rules::ComponentSelection;
using stratum::procgen::rules::ComponentSelector;
using stratum::procgen::rules::ComponentSplitReport;
using stratum::procgen::rules::Diagnostic;
using stratum::procgen::rules::dump_shape;
using stratum::procgen::rules::edge_component_axes;
using stratum::procgen::rules::emit_components;
using stratum::procgen::rules::extrude_shape;
using stratum::procgen::rules::Face;
using stratum::procgen::rules::face_area;
using stratum::procgen::rules::face_component_axes;
using stratum::procgen::rules::face_normal;
using stratum::procgen::rules::GenerationOptions;
using stratum::procgen::rules::GenerationResult;
using stratum::procgen::rules::generate;
using stratum::procgen::rules::OperationArgs;
using stratum::procgen::rules::OperationTable;
using stratum::procgen::rules::OpResult;
using stratum::procgen::rules::parse;
using stratum::procgen::rules::kComponentOrderQuantum;
using stratum::procgen::rules::parse_component_domain;
using stratum::procgen::rules::parse_component_selector;
using stratum::procgen::rules::ParseResult;
using stratum::procgen::rules::rotate_scope_shape;
using stratum::procgen::rules::scale_shape;
using stratum::procgen::rules::ScopeAxis;
using stratum::procgen::rules::select_angle;
using stratum::procgen::rules::selector_admits;
using stratum::procgen::rules::Severity;
using stratum::procgen::rules::Shape;
using stratum::procgen::rules::ShapeGeometry;
using stratum::procgen::rules::shape_from_polygon;
using stratum::procgen::rules::shape_from_rect;
using stratum::procgen::rules::shape_from_rings;
using stratum::procgen::rules::split_components;
using stratum::procgen::rules::standard_operations;
using stratum::procgen::rules::taper_shape;

namespace {

constexpr double kEps = 1e-9;

// ============================================================================
// Helpers
// ============================================================================

void check_vec3(const glm::dvec3& actual, const glm::dvec3& expected, double eps) {
    CHECK_NEAR(actual.x, expected.x, eps);
    CHECK_NEAR(actual.y, expected.y, eps);
    CHECK_NEAR(actual.z, expected.z, eps);
}

/// A 2 x 3 x 4 box: the rectangle 2 by 4 in the xz plane, extruded 3 up y
[[nodiscard]] Shape make_box() {
    Shape shape = shape_from_rect(2.0, 4.0);
    const OpResult result = extrude_shape(shape, ScopeAxis::Y, 3.0);
    CHECK_TRUE(result.ok);
    return shape;
}

/**
 * @brief A frustum whose walls are steep but NOT vertical
 *
 * taper_shape() slopes at exactly 45 degrees, which is the knife edge of the
 * dominant-axis tie-break and therefore the one slope that proves nothing about
 * it. Scaling the height to three makes the walls lean by about 18 degrees off
 * vertical: unambiguously `side`, unambiguously not `vertical`, and
 * unambiguously `aslant`.
 */
[[nodiscard]] Shape make_frustum() {
    Shape shape = shape_from_rect(6.0, 6.0);
    const OpResult tapered = taper_shape(shape, 1.0);
    CHECK_TRUE(tapered.ok);
    const OpResult scaled = scale_shape(shape, glm::dvec3{6.0, 3.0, 6.0});
    CHECK_TRUE(scaled.ok);
    return shape;
}

/// The one selection that admits everything
[[nodiscard]] ComponentSelection all_of_them() {
    return ComponentSelection{};
}

[[nodiscard]] ComponentSelection just(ComponentSelector selector) {
    ComponentSelection selection;
    selection.selector = selector;
    return selection;
}

/// How many components of @p domain answer to @p selector
[[nodiscard]] size_t count_of(const Shape& shape,
                              ComponentDomain domain,
                              ComponentSelector selector) {
    return split_components(shape, domain, just(selector)).size();
}

/// The single component answering to @p selector, or a default Component
[[nodiscard]] Component only_component(const Shape& shape, ComponentSelector selector) {
    const std::vector<Component> found =
        split_components(shape, ComponentDomain::Face, just(selector));
    CHECK_EQ(found.size(), size_t{1});
    if (found.size() != 1) {
        return Component{};
    }
    return found.front();
}

/// World positions of every vertex of a shape, in index order
[[nodiscard]] std::vector<glm::dvec3> world_positions(const Shape& shape) {
    std::vector<glm::dvec3> out;
    out.reserve(shape.geometry.positions.size());
    for (const glm::dvec3& local : shape.geometry.positions) {
        out.push_back(shape.scope.to_world(local));
    }
    return out;
}

/// Is every position in @p needle present in @p haystack, to within @p eps?
[[nodiscard]] bool positions_are_contained(const std::vector<glm::dvec3>& needle,
                                           const std::vector<glm::dvec3>& haystack,
                                           double eps) {
    if (needle.empty()) {
        return false;  // an empty needle must not pass by being empty
    }
    for (const glm::dvec3& point : needle) {
        bool found = false;
        for (const glm::dvec3& candidate : haystack) {
            if (std::fabs(point.x - candidate.x) <= eps && std::fabs(point.y - candidate.y) <= eps &&
                std::fabs(point.z - candidate.z) <= eps) {
                found = true;
                break;
            }
        }
        if (!found) {
            return false;
        }
    }
    return true;
}

/// A rotation of @p degrees about the world y axis, as columns
[[nodiscard]] glm::dmat3 rotation_about_y(double degrees) {
    const double radians = degrees * 3.14159265358979323846 / 180.0;
    const double c = std::cos(radians);
    const double s = std::sin(radians);
    return glm::dmat3{glm::dvec3{c, 0.0, -s}, glm::dvec3{0.0, 1.0, 0.0}, glm::dvec3{s, 0.0, c}};
}

/**
 * @brief Two roof pitches at 30 degrees, as a shape of their own
 *
 * Built by hand because no D2 operation makes a pitched roof, and because a
 * pitch is the case the angle filter exists for. Each quad is wound so that its
 * normal points up and outwards, which is Face's contract; the tests assert the
 * angle those normals make with +y, so a quad wound the wrong way fails rather
 * than passing with a sign flipped.
 */
[[nodiscard]] Shape make_gable_pitches(double& pitch_degrees) {
    pitch_degrees = 30.0;
    const double rise = 2.0 * std::tan(30.0 * 3.14159265358979323846 / 180.0);

    Shape shape;
    shape.geometry.positions = {
        glm::dvec3{0.0, 0.0, 0.0},   glm::dvec3{0.0, 0.0, 2.0},  glm::dvec3{2.0, rise, 2.0},
        glm::dvec3{2.0, rise, 0.0},  glm::dvec3{4.0, 0.0, 0.0},  glm::dvec3{4.0, 0.0, 2.0},
    };
    Face west;
    west.loop = {0, 1, 2, 3};
    Face east;
    east.loop = {4, 3, 2, 5};
    shape.geometry.faces = {west, east};
    stratum::procgen::rules::refit_scope(shape);
    return shape;
}

/**
 * @brief A U-shaped footprint extruded three metres: ten faces, five sharing pairs
 *
 * The fixture the ordering tests need and a box cannot be. A box has six faces
 * with six DISTINCT normals, so the normal half of the key orders it on its own
 * and every assertion made on it survives deleting the centroid half. This shape
 * has three faces looking along +z and one pair each along +x and -x, so within
 * those groups the centroid is the only thing that is not the buffer order.
 *
 * The notch runs in from +z, so the walls are at x = 0, 2, 4 and 6 and the
 * centroids of the three +z walls are two metres apart -- far enough to be an
 * architectural distinction, which is the distinction the order has to keep.
 */
[[nodiscard]] Shape make_u_block() {
    const std::vector<glm::dvec2> ring = {{0.0, 0.0}, {6.0, 0.0}, {6.0, 5.0}, {4.0, 5.0},
                                          {4.0, 2.0}, {2.0, 2.0}, {2.0, 5.0}, {0.0, 5.0}};
    Shape shape = shape_from_polygon(ring);
    const OpResult result = extrude_shape(shape, ScopeAxis::Y, 3.0);
    CHECK_TRUE(result.ok);
    return shape;
}

/// How many pairs of faces of @p shape have the same normal. The fixture's premise.
[[nodiscard]] size_t pairs_sharing_a_normal(const Shape& shape) {
    std::vector<glm::dvec3> normals;
    for (const Face& face : shape.geometry.faces) {
        normals.push_back(face_normal(shape.geometry, face));
    }
    size_t pairs = 0;
    for (size_t i = 0; i < normals.size(); ++i) {
        for (size_t j = i + 1; j < normals.size(); ++j) {
            if (glm::length(normals[i] - normals[j]) < 1e-9) {
                ++pairs;
            }
        }
    }
    return pairs;
}

/**
 * @brief The centroid of a component's own geometry, expressed in the PARENT's frame
 *
 * Taken from the component's OUTER loop and routed through world space, so it is
 * the component's real position and not a number copied out of the split. A
 * component that was built from a different face than the order claims lands in
 * a different place here.
 */
[[nodiscard]] glm::dvec3 component_centroid(const Shape& parent, const Shape& component) {
    if (component.geometry.faces.empty() || component.geometry.faces.front().loop.empty()) {
        return glm::dvec3{0.0};
    }
    const std::vector<uint32_t>& loop = component.geometry.faces.front().loop;
    glm::dvec3 total{0.0};
    for (const uint32_t index : loop) {
        const glm::dvec3 world = component.scope.to_world(component.geometry.positions[index]);
        total += parent.scope.to_local(world);
    }
    return total / static_cast<double>(loop.size());
}

/// One component as the canonical order describes it: where it is and where it looks
struct ExpectedComponent {
    glm::dvec3 centroid{0.0};
    glm::dvec3 normal{0.0};
};

/// One coordinate on the ordering grid, as op_comp.hpp point 3 defines it
[[nodiscard]] int64_t on_the_order_grid(double value) {
    return static_cast<int64_t>(std::llround(value / kComponentOrderQuantum));
}

/**
 * @brief The canonical order, re-derived from op_comp.hpp point 3 rather than from the split
 *
 * The spec written a second time, on purpose: ascending quantised centroid in
 * y, then x, then z, and then the normal the same way. Comparing the split
 * against a permuted buffer proves the order does not follow the buffer; it
 * cannot prove WHICH key produced it, and an implementation keyed on the first
 * vertex of the ring rather than on the centroid is buffer-independent too.
 * This is what tells those apart.
 */
[[nodiscard]] std::vector<ExpectedComponent> expected_order(const Shape& shape) {
    std::vector<ExpectedComponent> out;
    for (const Face& face : shape.geometry.faces) {
        if (face.loop.size() < 3) {
            continue;
        }
        const glm::dvec3 normal = face_normal(shape.geometry, face);
        if (!(glm::length(normal) > 0.5) || !(face_area(shape.geometry, face) > 1e-12)) {
            continue;  // degenerate: dropped before numbering, so never ordered
        }
        ExpectedComponent entry;
        entry.normal = normal;
        glm::dvec3 total{0.0};
        for (const uint32_t index : face.loop) {
            total += shape.geometry.positions[index];
        }
        entry.centroid = total / static_cast<double>(face.loop.size());
        out.push_back(entry);
    }

    std::sort(out.begin(), out.end(), [](const ExpectedComponent& a, const ExpectedComponent& b) {
        const int64_t left[6] = {on_the_order_grid(a.centroid.y), on_the_order_grid(a.centroid.x),
                                 on_the_order_grid(a.centroid.z), on_the_order_grid(a.normal.y),
                                 on_the_order_grid(a.normal.x),   on_the_order_grid(a.normal.z)};
        const int64_t right[6] = {on_the_order_grid(b.centroid.y), on_the_order_grid(b.centroid.x),
                                  on_the_order_grid(b.centroid.z), on_the_order_grid(b.normal.y),
                                  on_the_order_grid(b.normal.x),   on_the_order_grid(b.normal.z)};
        for (int i = 0; i < 6; ++i) {
            if (left[i] != right[i]) {
                return left[i] < right[i];
            }
        }
        return false;
    });
    return out;
}

/**
 * @brief The number inside `comp.face[N]`, so that the label is READ and not only matched
 *
 * A filter on the prefix alone passes an implementation that labels every child
 * `comp.face[0]`, and the label is what an author reads to find the wall the
 * rule meant.
 */
[[nodiscard]] bool role_index_of(const std::string& role,
                                 const std::string& prefix,
                                 uint32_t& out) {
    if (role.rfind(prefix, 0) != 0) {
        return false;
    }
    const size_t close = role.find(']', prefix.size());
    if (close == std::string::npos || close == prefix.size()) {
        return false;
    }
    const std::string digits = role.substr(prefix.size(), close - prefix.size());
    for (const char c : digits) {
        if (c < '0' || c > '9') {
            return false;
        }
    }
    out = static_cast<uint32_t>(std::stoul(digits));
    return true;
}

/**
 * @brief The dumps of every emitted component, in the order they were emitted
 *
 * dump_shape() prints the shape's rule and its sibling index, so this text
 * carries both the label a rule author reads and the address the component was
 * given in the tree -- which is what makes it sensitive to the emission ORDER
 * and not only to the geometry.
 */
[[nodiscard]] std::string emitted_components_dump(const GenerationResult& result) {
    std::string out;
    for (const Shape& terminal : result.terminals) {
        uint32_t label = 0;
        if (role_index_of(terminal.rule, "comp.face[", label)) {
            out += dump_shape(terminal);
        }
    }
    return out;
}

/// Parse @p source and run it, asserting the parse rather than assuming it
[[nodiscard]] GenerationResult run_on(const std::string& source,
                                      const Shape& seed,
                                      const OperationTable* operations = nullptr) {
    const ParseResult parsed = parse(source, "test.srl");
    CHECK_EQ(parsed.render_all(source), std::string{});
    return generate(parsed.file, seed, GenerationOptions{}, operations, &component_functions());
}

/// Is @p needle a substring of any diagnostic of @p severity?
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

/// Is @p needle a substring of any entry of @p problems?
[[nodiscard]] bool has_problem(const ComponentSplitReport& report, const std::string& needle) {
    for (const std::string& problem : report.problems) {
        if (problem.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

} // namespace

// ============================================================================
// Finding the components at all
// ============================================================================

TEST(OpComp, a_box_splits_into_its_six_faces) {
    const Shape box = make_box();
    ComponentSplitReport report;
    const std::vector<Component> faces =
        split_components(box, ComponentDomain::Face, all_of_them(), &report);

    CHECK_EQ(faces.size(), size_t{6});
    CHECK_EQ(report.total, uint32_t{6});
    CHECK_EQ(report.selected, uint32_t{6});
    CHECK_EQ(report.degenerate, uint32_t{0});
    CHECK_EQ(report.non_planar, uint32_t{0});
    CHECK_TRUE(report.problems.empty());

    // The surface area of a 2 x 3 x 4 box, summed from the components rather than
    // read off the scope: 2(2*3) + 2(2*4) + 2(3*4) = 52.
    double area = 0.0;
    for (const Component& component : faces) {
        area += component.measure;
    }
    CHECK_NEAR(area, 52.0, 1e-9);
}

TEST(OpComp, the_six_direction_words_partition_the_faces) {
    const Shape box = make_box();
    const ComponentSelector words[] = {
        ComponentSelector::Front, ComponentSelector::Back,  ComponentSelector::Left,
        ComponentSelector::Right, ComponentSelector::Top,   ComponentSelector::Bottom};

    size_t total = 0;
    for (const ComponentSelector word : words) {
        const size_t found = count_of(box, ComponentDomain::Face, word);
        CHECK_EQ(found, size_t{1});
        total += found;
    }
    // Every face claimed exactly once. A classifier that let a face answer to two
    // words, or to none, fails here rather than in a facade three features later.
    CHECK_EQ(total, size_t{6});
}

TEST(OpComp, a_lone_face_is_its_own_single_component) {
    const Shape square = shape_from_rect(2.0, 4.0);
    const std::vector<Component> faces =
        split_components(square, ComponentDomain::Face, all_of_them());

    CHECK_EQ(faces.size(), size_t{1});
    if (faces.size() == 1) {
        // shape_from_polygon orients the face normal to +y, so a footprint is a top.
        check_vec3(faces.front().normal, glm::dvec3{0.0, 1.0, 0.0}, kEps);
        CHECK_NEAR(faces.front().measure, 8.0, 1e-9);
    }
}

// ============================================================================
// The oriented scope -- the load-bearing tests
// ============================================================================

TEST(OpComp, the_front_face_scope_lies_in_the_face_with_its_normal_as_z) {
    const Shape box = make_box();
    const Component front = only_component(box, ComponentSelector::Front);

    // z is the face normal, y is up within the face, x is across it, and the
    // face is flat in z. For the front wall of an axis-aligned box that is the
    // identity -- which is exactly why the other three faces below are also
    // checked: an implementation that simply copied the parent's axes passes
    // this one test and fails every other.
    check_vec3(front.shape.scope.axes[0], glm::dvec3{1.0, 0.0, 0.0}, kEps);
    check_vec3(front.shape.scope.axes[1], glm::dvec3{0.0, 1.0, 0.0}, kEps);
    check_vec3(front.shape.scope.axes[2], glm::dvec3{0.0, 0.0, 1.0}, kEps);
    check_vec3(front.shape.scope.size, glm::dvec3{2.0, 3.0, 0.0}, kEps);
}

TEST(OpComp, the_right_face_scope_runs_across_the_wall_not_across_the_world) {
    const Shape box = make_box();
    const Component right = only_component(box, ComponentSelector::Right);

    // z = +x (the normal), y = +y (up is still up within a vertical wall), and
    // x = cross(y, z) = -z. So the wall's own x runs from its front edge to its
    // back edge, and its width is 4 -- the box's z extent, not its x extent.
    check_vec3(right.shape.scope.axes[0], glm::dvec3{0.0, 0.0, -1.0}, kEps);
    check_vec3(right.shape.scope.axes[1], glm::dvec3{0.0, 1.0, 0.0}, kEps);
    check_vec3(right.shape.scope.axes[2], glm::dvec3{1.0, 0.0, 0.0}, kEps);
    check_vec3(right.shape.scope.size, glm::dvec3{4.0, 3.0, 0.0}, kEps);

    // The load-bearing consequence: a split down this component's x cuts bays
    // across the wall. With the parent's axes the same split would cut along the
    // wall's thickness, which is zero, and produce nothing.
    CHECK((right.shape.scope.size.x) > 0.0);
}

TEST(OpComp, a_horizontal_face_takes_its_in_plane_up_from_the_parent_z) {
    const Shape box = make_box();

    const Component top = only_component(box, ComponentSelector::Top);
    // The parent's +y is the roof's normal and so cannot also be its in-plane up.
    // The fallback is the parent's +z: y = +z, z = +y, x = cross(y, z) = -x.
    check_vec3(top.shape.scope.axes[0], glm::dvec3{-1.0, 0.0, 0.0}, kEps);
    check_vec3(top.shape.scope.axes[1], glm::dvec3{0.0, 0.0, 1.0}, kEps);
    check_vec3(top.shape.scope.axes[2], glm::dvec3{0.0, 1.0, 0.0}, kEps);
    check_vec3(top.shape.scope.size, glm::dvec3{2.0, 4.0, 0.0}, kEps);

    const Component bottom = only_component(box, ComponentSelector::Bottom);
    check_vec3(bottom.shape.scope.axes[0], glm::dvec3{1.0, 0.0, 0.0}, kEps);
    check_vec3(bottom.shape.scope.axes[1], glm::dvec3{0.0, 0.0, 1.0}, kEps);
    check_vec3(bottom.shape.scope.axes[2], glm::dvec3{0.0, -1.0, 0.0}, kEps);
    check_vec3(bottom.shape.scope.size, glm::dvec3{2.0, 4.0, 0.0}, kEps);
}

TEST(OpComp, every_component_frame_is_orthonormal_and_right_handed) {
    const Shape box = make_box();
    const std::vector<Component> faces =
        split_components(box, ComponentDomain::Face, all_of_them());
    CHECK_EQ(faces.size(), size_t{6});

    for (const Component& component : faces) {
        const glm::dmat3& axes = component.shape.scope.axes;
        // Orthonormal, or Scope::to_local() -- a transpose, not an inverse -- is
        // wrong by an amount that grows silently with every later operation.
        CHECK_NEAR(glm::dot(axes[0], axes[0]), 1.0, 1e-12);
        CHECK_NEAR(glm::dot(axes[1], axes[1]), 1.0, 1e-12);
        CHECK_NEAR(glm::dot(axes[2], axes[2]), 1.0, 1e-12);
        CHECK_NEAR(glm::dot(axes[0], axes[1]), 0.0, 1e-12);
        CHECK_NEAR(glm::dot(axes[1], axes[2]), 0.0, 1e-12);
        CHECK_NEAR(glm::dot(axes[2], axes[0]), 0.0, 1e-12);
        // Right-handed: cross(x, y) == z, not -z. A mirrored frame flips the
        // winding of everything a later split builds on it.
        check_vec3(glm::cross(axes[0], axes[1]), axes[2], 1e-12);
    }
}

TEST(OpComp, a_component_lands_in_the_world_exactly_where_its_face_was) {
    const Shape box = make_box();
    const std::vector<glm::dvec3> parent_world = world_positions(box);

    const std::vector<Component> faces =
        split_components(box, ComponentDomain::Face, all_of_them());
    CHECK_EQ(faces.size(), size_t{6});

    for (const Component& component : faces) {
        // Compared in WORLD space. A component whose local coordinates are right
        // and whose frame is wrong sits somewhere else entirely, and a test
        // against local coordinates alone would never notice.
        CHECK_TRUE(positions_are_contained(world_positions(component.shape), parent_world, 1e-9));
        // The face is flat, so its component has no depth.
        CHECK_NEAR(component.shape.scope.size.z, 0.0, 1e-12);
    }
}

TEST(OpComp, a_component_inherits_the_parent_attributes) {
    Shape box = make_box();
    box.attributes["style"] = stratum::procgen::rules::Value::text("brick");

    const Component front = only_component(box, ComponentSelector::Front);
    const auto found = front.shape.attributes.find("style");
    CHECK_TRUE(found != front.shape.attributes.end());
    if (found != front.shape.attributes.end()) {
        CHECK_EQ(found->second.to_text(), std::string{"brick"});
    }
}

// ============================================================================
// "Relative to the scope", tested both ways
// ============================================================================

TEST(OpComp, a_rotated_building_keeps_the_same_front) {
    const Shape upright = make_box();

    // The same building, placed on its lot at 40 degrees: the LOCAL geometry is
    // untouched and the rotation lives in the scope's axes, which is what
    // shape.hpp's first invariant makes the normal case.
    Shape placed = upright;
    placed.scope.axes = rotation_about_y(40.0) * upright.scope.axes;
    placed.scope.origin = glm::dvec3{120.0, 0.0, -35.0};

    const Component before = only_component(upright, ComponentSelector::Front);
    const Component after = only_component(placed, ComponentSelector::Front);

    // The same face of the mass, by its index in the canonical order and by the
    // normal the selector reads.
    CHECK_EQ(after.index, before.index);
    check_vec3(after.normal, before.normal, kEps);
    CHECK_NEAR(after.measure, before.measure, 1e-12);

    // And it really is somewhere else in the world, so this is not passing by
    // the two shapes being identical.
    check_vec3(after.shape.scope.axes[2], rotation_about_y(40.0) * glm::dvec3{0.0, 0.0, 1.0}, kEps);
    CHECK((std::fabs(after.shape.scope.axes[2].z - 1.0)) > 0.1);
}

TEST(OpComp, rotate_scope_changes_which_face_is_the_front) {
    Shape box = make_box();
    const Component before = only_component(box, ComponentSelector::Front);
    check_vec3(before.shape.scope.axes[2], glm::dvec3{0.0, 0.0, 1.0}, kEps);

    // rotate_scope turns the frame and leaves the geometry in the world. That is
    // the supported way for a rule to say "treat THAT wall as the front", so the
    // answer has to change -- and an implementation reading world normals would
    // hand back the same wall.
    rotate_scope_shape(box, glm::dvec3{0.0, 90.0, 0.0});
    const Component after = only_component(box, ComponentSelector::Front);

    // Still +z of the SCOPE, which is now a world x direction.
    check_vec3(after.normal, glm::dvec3{0.0, 0.0, 1.0}, kEps);
    CHECK_NEAR(std::fabs(after.shape.scope.axes[2].x), 1.0, kEps);
    CHECK_NEAR(after.shape.scope.axes[2].z, 0.0, kEps);
    // The 2 x 3 front wall became the 4 x 3 side wall, so this is a different face.
    CHECK_NEAR(after.measure, 12.0, 1e-9);
    CHECK_NEAR(before.measure, 6.0, 1e-9);
}

// ============================================================================
// Classification
// ============================================================================

TEST(OpComp, the_dominant_axis_decides_a_direction_and_ties_go_to_up) {
    const double radians = 3.14159265358979323846 / 180.0;

    // A shallow pitch, 30 degrees off horizontal: mostly up, so `top`.
    const glm::dvec3 shallow{std::sin(30.0 * radians), std::cos(30.0 * radians), 0.0};
    CHECK_TRUE(component_direction(shallow) == ComponentSelector::Top);

    // A steep pitch, 60 degrees off horizontal: mostly sideways, so `right`.
    const glm::dvec3 steep{std::sin(60.0 * radians), std::cos(60.0 * radians), 0.0};
    CHECK_TRUE(component_direction(steep) == ComponentSelector::Right);
    CHECK_TRUE(selector_admits(ComponentSelector::Side, steep));

    // Exactly 45 degrees is the documented tie, and it goes to up.
    const glm::dvec3 knife{std::sqrt(0.5), std::sqrt(0.5), 0.0};
    CHECK_TRUE(component_direction(knife) == ComponentSelector::Top);

    // A normal that is not a direction answers to nothing.
    CHECK_TRUE(component_direction(glm::dvec3{0.0}) == ComponentSelector::All);
    CHECK_FALSE(selector_admits(ComponentSelector::All, glm::dvec3{0.0}));
}

TEST(OpComp, side_and_vertical_agree_on_a_box_and_disagree_on_a_frustum) {
    const Shape box = make_box();
    CHECK_EQ(count_of(box, ComponentDomain::Face, ComponentSelector::Side), size_t{4});
    CHECK_EQ(count_of(box, ComponentDomain::Face, ComponentSelector::Vertical), size_t{4});
    CHECK_EQ(count_of(box, ComponentDomain::Face, ComponentSelector::Horizontal), size_t{2});
    CHECK_EQ(count_of(box, ComponentDomain::Face, ComponentSelector::Aslant), size_t{0});

    // A frustum is the case that separates the two words, which is why ast.hpp
    // carries both. Its four walls lean, so they are sides and are not vertical.
    const Shape frustum = make_frustum();
    CHECK_EQ(count_of(frustum, ComponentDomain::Face, ComponentSelector::Side), size_t{4});
    CHECK_EQ(count_of(frustum, ComponentDomain::Face, ComponentSelector::Vertical), size_t{0});
    CHECK_EQ(count_of(frustum, ComponentDomain::Face, ComponentSelector::Aslant), size_t{4});
    CHECK_EQ(count_of(frustum, ComponentDomain::Face, ComponentSelector::Horizontal), size_t{2});
    CHECK_EQ(count_of(frustum, ComponentDomain::Face, ComponentSelector::Top), size_t{1});
    CHECK_EQ(count_of(frustum, ComponentDomain::Face, ComponentSelector::Bottom), size_t{1});
}

TEST(OpComp, an_angle_range_picks_the_roof_pitches_and_rejects_the_neighbours) {
    double pitch = 0.0;
    const Shape roof = make_gable_pitches(pitch);
    CHECK_NEAR(pitch, 30.0, 1e-12);

    // Both pitches are 30 degrees from +y, so a window around 30 takes both.
    CHECK_EQ(split_components(roof, ComponentDomain::Face, select_angle(ScopeAxis::Y, 20.0, 40.0))
                 .size(),
             size_t{2});
    // A window that excludes 30 takes neither, which is what makes the first
    // assertion mean something.
    CHECK_EQ(
        split_components(roof, ComponentDomain::Face, select_angle(ScopeAxis::Y, 0.0, 10.0)).size(),
        size_t{0});
    CHECK_EQ(split_components(roof, ComponentDomain::Face, select_angle(ScopeAxis::Y, 40.0, 90.0))
                 .size(),
             size_t{0});

    // The bounds are inclusive, and exact at the quarter turns that deg_sin_cos()
    // makes exact: a flat footprint is at 0 degrees from +y.
    const Shape flat = shape_from_rect(1.0, 1.0);
    CHECK_EQ(
        split_components(flat, ComponentDomain::Face, select_angle(ScopeAxis::Y, 0.0, 0.0)).size(),
        size_t{1});
    CHECK_EQ(split_components(flat, ComponentDomain::Face, select_angle(ScopeAxis::Y, 90.0, 180.0))
                 .size(),
             size_t{0});
}

TEST(OpComp, an_angle_filter_narrows_a_word_rather_than_replacing_it) {
    const Shape box = make_box();

    // `side` plus an angle band that only the vertical walls satisfy.
    ComponentSelection selection = select_angle(ScopeAxis::Y, 90.0, 90.0);
    selection.selector = ComponentSelector::Side;
    CHECK_EQ(split_components(box, ComponentDomain::Face, selection).size(), size_t{4});

    // The same band with `top` selects nothing: the two narrowings are combined,
    // not replaced by whichever was set last.
    selection.selector = ComponentSelector::Top;
    CHECK_EQ(split_components(box, ComponentDomain::Face, selection).size(), size_t{0});
}

TEST(OpComp, an_angle_is_measured_to_the_axis_it_names) {
    // Every other angle in this file is measured to +y, so an implementation
    // that ignored ComponentSelection::angle_axis and always used y would pass
    // all of them. Each axis of one box names a different face.
    const Shape box = make_box();

    const std::vector<Component> right =
        split_components(box, ComponentDomain::Face, select_angle(ScopeAxis::X, 0.0, 0.0));
    CHECK_EQ(right.size(), size_t{1});
    if (right.size() == 1) {
        check_vec3(right.front().normal, glm::dvec3{1.0, 0.0, 0.0}, kEps);
    }

    const std::vector<Component> left =
        split_components(box, ComponentDomain::Face, select_angle(ScopeAxis::X, 180.0, 180.0));
    CHECK_EQ(left.size(), size_t{1});
    if (left.size() == 1) {
        check_vec3(left.front().normal, glm::dvec3{-1.0, 0.0, 0.0}, kEps);
    }

    const std::vector<Component> front =
        split_components(box, ComponentDomain::Face, select_angle(ScopeAxis::Z, 0.0, 0.0));
    CHECK_EQ(front.size(), size_t{1});
    if (front.size() == 1) {
        check_vec3(front.front().normal, glm::dvec3{0.0, 0.0, 1.0}, kEps);
    }

    // The same band about y takes the roof and neither of those, which is what
    // makes the three above measure the axis rather than the band.
    const std::vector<Component> top =
        split_components(box, ComponentDomain::Face, select_angle(ScopeAxis::Y, 0.0, 0.0));
    CHECK_EQ(top.size(), size_t{1});
    if (top.size() == 1) {
        check_vec3(top.front().normal, glm::dvec3{0.0, 1.0, 0.0}, kEps);
    }

    // A right angle to +x is the four faces that are not the two perpendicular
    // to it -- a count that differs from the same band about y only because the
    // axis is read.
    CHECK_EQ(
        split_components(box, ComponentDomain::Face, select_angle(ScopeAxis::X, 90.0, 90.0)).size(),
        size_t{4});
}

TEST(OpComp, an_empty_angle_range_is_reported) {
    const Shape box = make_box();
    ComponentSplitReport report;
    const std::vector<Component> found = split_components(
        box, ComponentDomain::Face, select_angle(ScopeAxis::Y, 90.0, 10.0), &report);

    CHECK_EQ(found.size(), size_t{0});
    CHECK_TRUE(has_problem(report, "selects nothing"));
}

TEST(OpComp, selector_words_round_trip_through_the_language_spellings) {
    // The parse accepts exactly what ast.hpp prints, so `select` and `comp.count`
    // share one vocabulary instead of drifting into two.
    ComponentSelector selector = ComponentSelector::All;
    CHECK_TRUE(parse_component_selector("front", selector));
    CHECK_TRUE(selector == ComponentSelector::Front);
    CHECK_TRUE(parse_component_selector("aslant", selector));
    CHECK_TRUE(selector == ComponentSelector::Aslant);
    CHECK_FALSE(parse_component_selector("Front", selector));
    CHECK_FALSE(parse_component_selector("forward", selector));
    // Untouched on a failure.
    CHECK_TRUE(selector == ComponentSelector::Aslant);
}

// ============================================================================
// The canonical order
// ============================================================================

TEST(OpComp, the_component_order_does_not_follow_the_geometry_buffer) {
    // A U-block and not a box. Six faces with six distinct normals are ordered
    // by the normal key alone, so a box is buffer-independent for keys that have
    // no centroid in them at all; five of this shape's ten faces share a normal
    // with another, and within a sharing group the centroid is the only thing
    // left that is not the buffer order.
    const Shape block = make_u_block();
    CHECK_EQ(pairs_sharing_a_normal(block), size_t{5});

    // Two permutations, not one. Reversing a buffer maps some orders onto
    // themselves; rotating it by three does not.
    Shape reversed = block;
    std::reverse(reversed.geometry.faces.begin(), reversed.geometry.faces.end());
    Shape rotated = block;
    std::rotate(rotated.geometry.faces.begin(), rotated.geometry.faces.begin() + 3,
                rotated.geometry.faces.end());

    const std::vector<Component> original =
        split_components(block, ComponentDomain::Face, all_of_them());
    const std::vector<Component> from_reversed =
        split_components(reversed, ComponentDomain::Face, all_of_them());
    const std::vector<Component> from_rotated =
        split_components(rotated, ComponentDomain::Face, all_of_them());

    CHECK_EQ(original.size(), size_t{10});
    CHECK_EQ(from_reversed.size(), original.size());
    CHECK_EQ(from_rotated.size(), original.size());
    for (size_t i = 0; i < original.size() && i < from_reversed.size() && i < from_rotated.size();
         ++i) {
        CHECK_EQ(from_reversed[i].index, original[i].index);
        CHECK_EQ(from_rotated[i].index, original[i].index);
        check_vec3(from_reversed[i].normal, original[i].normal, 1e-12);
        check_vec3(from_rotated[i].normal, original[i].normal, 1e-12);
        CHECK_NEAR(from_reversed[i].measure, original[i].measure, 1e-12);
        CHECK_NEAR(from_rotated[i].measure, original[i].measure, 1e-12);
        // The whole component, not only its label: two shapes that dump
        // identically have the same geometry in the same frame.
        CHECK_EQ(dump_shape(from_reversed[i].shape), dump_shape(original[i].shape));
        CHECK_EQ(dump_shape(from_rotated[i].shape), dump_shape(original[i].shape));
    }
}

TEST(OpComp, the_order_follows_the_centroid_where_normals_repeat) {
    const Shape block = make_u_block();
    // The premise, asserted rather than assumed: if this shape ever stopped
    // having faces that share a normal, everything below would be satisfied by
    // the normal key and would measure nothing.
    CHECK_EQ(pairs_sharing_a_normal(block), size_t{5});

    const std::vector<Component> found =
        split_components(block, ComponentDomain::Face, all_of_them());
    const std::vector<ExpectedComponent> expected = expected_order(block);

    CHECK_EQ(found.size(), size_t{10});
    CHECK_EQ(expected.size(), found.size());
    for (size_t i = 0; i < found.size() && i < expected.size(); ++i) {
        CHECK_EQ(found[i].index, static_cast<uint32_t>(i));
        // Where the component IS, measured from the component's own geometry.
        check_vec3(component_centroid(block, found[i].shape), expected[i].centroid, 1e-9);
        check_vec3(found[i].normal, expected[i].normal, 1e-9);
    }

    // Spelled out for the group the centroid alone can order: three walls look
    // along +z, their centroids are at x = 1, 3 and 5, and they must come out
    // left to right. A key taken from the ring's first vertex instead of its
    // centroid gives 1, 5, 3 on this shape.
    std::vector<double> facing_front;
    for (const Component& component : found) {
        if (component.normal.z > 0.5) {
            facing_front.push_back(component_centroid(block, component.shape).x);
        }
    }
    CHECK_EQ(facing_front.size(), size_t{3});
    if (facing_front.size() == 3) {
        CHECK_NEAR(facing_front[0], 1.0, 1e-9);
        CHECK_NEAR(facing_front[1], 3.0, 1e-9);
        CHECK_NEAR(facing_front[2], 5.0, 1e-9);
    }
}

TEST(OpComp, two_faces_two_millimetres_apart_are_ordered_by_the_centroid) {
    // A pane and the panel two millimetres behind it: same size, same place,
    // same normal, two millimetres apart in y. Nothing but the centroid can
    // order them, and two millimetres is two thousand times
    // kComponentOrderQuantum -- the quantum is a noise floor, not an
    // architectural grid. An implementation that quantised to the metre would
    // tie these two and fall back to the buffer order, which is what the second
    // half of this test rules out.
    Shape pane;
    pane.geometry.positions = {glm::dvec3{0.0, 0.0, 0.0},   glm::dvec3{0.0, 0.0, 1.0},
                               glm::dvec3{1.0, 0.0, 1.0},   glm::dvec3{1.0, 0.0, 0.0},
                               glm::dvec3{0.0, 0.002, 0.0}, glm::dvec3{0.0, 0.002, 1.0},
                               glm::dvec3{1.0, 0.002, 1.0}, glm::dvec3{1.0, 0.002, 0.0}};
    Face lower;
    lower.loop = {0, 1, 2, 3};
    Face upper;
    upper.loop = {4, 5, 6, 7};
    pane.geometry.faces = {lower, upper};
    stratum::procgen::rules::refit_scope(pane);

    const std::vector<Component> found =
        split_components(pane, ComponentDomain::Face, all_of_them());
    CHECK_EQ(found.size(), size_t{2});
    if (found.size() == 2) {
        check_vec3(found[0].normal, glm::dvec3{0.0, 1.0, 0.0}, kEps);
        check_vec3(found[1].normal, glm::dvec3{0.0, 1.0, 0.0}, kEps);
        CHECK_NEAR(component_centroid(pane, found[0].shape).y, 0.0, 1e-12);
        CHECK_NEAR(component_centroid(pane, found[1].shape).y, 0.002, 1e-12);
    }

    Shape reversed = pane;
    std::reverse(reversed.geometry.faces.begin(), reversed.geometry.faces.end());
    const std::vector<Component> again =
        split_components(reversed, ComponentDomain::Face, all_of_them());
    CHECK_EQ(again.size(), size_t{2});
    if (again.size() == 2) {
        CHECK_NEAR(component_centroid(reversed, again[0].shape).y, 0.0, 1e-12);
        CHECK_NEAR(component_centroid(reversed, again[1].shape).y, 0.002, 1e-12);
    }
}

TEST(OpComp, two_coincident_faces_are_ordered_by_their_normals) {
    // A double-sided panel: one ring, two faces, opposite windings. Their
    // centroids are the SAME point, so the centroid key ties and the normal key
    // is the only thing left that is not the buffer order. Ascending y of the
    // normal puts the downward face first, which is the rule op_comp.hpp states
    // for "two coincident components".
    Shape panel;
    panel.geometry.positions = {glm::dvec3{0.0, 0.0, 0.0}, glm::dvec3{0.0, 0.0, 1.0},
                                glm::dvec3{1.0, 0.0, 1.0}, glm::dvec3{1.0, 0.0, 0.0}};
    Face up;
    up.loop = {0, 1, 2, 3};
    Face down;
    down.loop = {3, 2, 1, 0};
    panel.geometry.faces = {up, down};
    stratum::procgen::rules::refit_scope(panel);

    const std::vector<Component> found =
        split_components(panel, ComponentDomain::Face, all_of_them());
    CHECK_EQ(found.size(), size_t{2});
    if (found.size() == 2) {
        check_vec3(found[0].normal, glm::dvec3{0.0, -1.0, 0.0}, kEps);
        check_vec3(found[1].normal, glm::dvec3{0.0, 1.0, 0.0}, kEps);
    }

    // And that answer comes from the normals, not from the order the two faces
    // happened to be written in.
    Shape reversed = panel;
    std::reverse(reversed.geometry.faces.begin(), reversed.geometry.faces.end());
    const std::vector<Component> again =
        split_components(reversed, ComponentDomain::Face, all_of_them());
    CHECK_EQ(again.size(), size_t{2});
    if (again.size() == 2 && found.size() == 2) {
        check_vec3(again[0].normal, glm::dvec3{0.0, -1.0, 0.0}, kEps);
        CHECK_EQ(dump_shape(again[0].shape), dump_shape(found[0].shape));
        CHECK_EQ(dump_shape(again[1].shape), dump_shape(found[1].shape));
    }
}

TEST(OpComp, an_index_addresses_the_same_face_after_the_buffer_is_permuted) {
    // On the U-block, so that component 3 is one of a pair that share a normal:
    // on a box, index 3 is pinned by the normal key alone.
    const Shape block = make_u_block();
    Shape reversed = block;
    std::reverse(reversed.geometry.faces.begin(), reversed.geometry.faces.end());
    Shape rotated = block;
    std::rotate(rotated.geometry.faces.begin(), rotated.geometry.faces.begin() + 3,
                rotated.geometry.faces.end());

    ComponentSelection third;
    third.indices = {3};

    const std::vector<Component> a = split_components(block, ComponentDomain::Face, third);
    const std::vector<Component> b = split_components(reversed, ComponentDomain::Face, third);
    const std::vector<Component> c = split_components(rotated, ComponentDomain::Face, third);

    CHECK_EQ(a.size(), size_t{1});
    CHECK_EQ(b.size(), size_t{1});
    CHECK_EQ(c.size(), size_t{1});
    if (a.size() == 1 && b.size() == 1 && c.size() == 1) {
        CHECK_EQ(a.front().index, uint32_t{3});
        // Component 3 of this shape is a wall looking along +x, and one of two
        // that do, so naming it is a statement about the centroid.
        check_vec3(a.front().normal, glm::dvec3{1.0, 0.0, 0.0}, kEps);
        CHECK_EQ(dump_shape(b.front().shape), dump_shape(a.front().shape));
        CHECK_EQ(dump_shape(c.front().shape), dump_shape(a.front().shape));
    }
}

TEST(OpComp, the_order_runs_bottom_to_top) {
    const Shape box = make_box();
    const std::vector<Component> faces =
        split_components(box, ComponentDomain::Face, all_of_them());
    CHECK_EQ(faces.size(), size_t{6});
    if (faces.size() == 6) {
        // The first and last components of a box are its floor and its roof.
        // That is the sentence the header promises an author.
        check_vec3(faces.front().normal, glm::dvec3{0.0, -1.0, 0.0}, kEps);
        check_vec3(faces.back().normal, glm::dvec3{0.0, 1.0, 0.0}, kEps);
    }

    // A box cannot show that the CENTROID is what runs bottom to top: its floor
    // and its roof differ in their normals too, so quantise(normal.y) orders
    // them bottom-first on its own and the assertions above would hold for an
    // implementation with no centroid in its key at all. Two faces that look the
    // same way at two heights can only be told apart by where they are: a
    // ground slab and the deck three metres over it.
    Shape storeys;
    storeys.geometry.positions = {glm::dvec3{0.0, 0.0, 0.0}, glm::dvec3{0.0, 0.0, 4.0},
                                  glm::dvec3{2.0, 0.0, 4.0}, glm::dvec3{2.0, 0.0, 0.0},
                                  glm::dvec3{0.0, 3.0, 0.0}, glm::dvec3{0.0, 3.0, 4.0},
                                  glm::dvec3{2.0, 3.0, 4.0}, glm::dvec3{2.0, 3.0, 0.0}};
    Face slab;
    slab.loop = {0, 1, 2, 3};
    Face deck;
    deck.loop = {4, 5, 6, 7};
    storeys.geometry.faces = {slab, deck};
    stratum::procgen::rules::refit_scope(storeys);

    const std::vector<Component> levels =
        split_components(storeys, ComponentDomain::Face, all_of_them());
    CHECK_EQ(levels.size(), size_t{2});
    if (levels.size() == 2) {
        check_vec3(levels[0].normal, levels[1].normal, 1e-12);  // nothing to tell them apart but y
        CHECK_NEAR(component_centroid(storeys, levels[0].shape).y, 0.0, 1e-12);
        CHECK_NEAR(component_centroid(storeys, levels[1].shape).y, 3.0, 1e-12);
    }

    // ...and it is the geometry that says which is lower, not the buffer.
    Shape upside_down = storeys;
    std::reverse(upside_down.geometry.faces.begin(), upside_down.geometry.faces.end());
    const std::vector<Component> again =
        split_components(upside_down, ComponentDomain::Face, all_of_them());
    CHECK_EQ(again.size(), size_t{2});
    if (again.size() == 2) {
        CHECK_NEAR(component_centroid(upside_down, again[0].shape).y, 0.0, 1e-12);
        CHECK_NEAR(component_centroid(upside_down, again[1].shape).y, 3.0, 1e-12);
    }
}

TEST(OpComp, an_index_named_twice_produces_the_component_twice) {
    const Shape box = make_box();
    ComponentSelection selection;
    selection.indices = {0, 0, 5};

    const std::vector<Component> found = split_components(box, ComponentDomain::Face, selection);
    CHECK_EQ(found.size(), size_t{3});
    if (found.size() == 3) {
        CHECK_EQ(found[0].index, found[1].index);
        CHECK_EQ(found[2].index, uint32_t{5});
    }
}

TEST(OpComp, an_index_past_the_end_is_reported_rather_than_dropped) {
    const Shape box = make_box();
    ComponentSelection selection;
    selection.selector = ComponentSelector::Top;
    selection.indices = {0, 3};

    ComponentSplitReport report;
    const std::vector<Component> found =
        split_components(box, ComponentDomain::Face, selection, &report);

    // The valid index still produces its component; the invalid one produces a
    // sentence naming both numbers, because "index out of range" on its own
    // sends the author back to count the faces.
    CHECK_EQ(found.size(), size_t{1});
    CHECK_EQ(report.selected, uint32_t{1});
    CHECK_TRUE(has_problem(report, "component 3 was asked for"));
    CHECK_TRUE(has_problem(report, "has 1"));
}

// ============================================================================
// The awkward geometry
// ============================================================================

TEST(OpComp, a_face_with_a_hole_keeps_the_hole_and_its_area) {
    const std::vector<glm::dvec2> outer = {{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}};
    const std::vector<std::vector<glm::dvec2>> holes = {
        {{4.0, 4.0}, {6.0, 4.0}, {6.0, 6.0}, {4.0, 6.0}}};
    const Shape courtyard = shape_from_rings(outer, holes);

    const std::vector<Component> faces =
        split_components(courtyard, ComponentDomain::Face, all_of_them());
    CHECK_EQ(faces.size(), size_t{1});
    if (faces.size() != 1) {
        return;
    }

    // The hole travels with the component. Dropping it would make the area wrong
    // and the triangulation solid, and neither shows up in a component count.
    CHECK_EQ(faces.front().shape.geometry.faces.size(), size_t{1});
    CHECK_NEAR(faces.front().measure, 100.0 - 4.0, 1e-9);
    // The scope box is fitted to the outer loop, because a hole is inside it.
    CHECK_NEAR(faces.front().shape.scope.size.x, 10.0, 1e-9);
    CHECK_NEAR(faces.front().shape.scope.size.y, 10.0, 1e-9);

    const ShapeGeometry& geometry = faces.front().shape.geometry;
    if (geometry.faces.size() != 1) {
        return;
    }
    const Face& face = geometry.faces[0];
    CHECK_EQ(face.holes.size(), size_t{1});
    if (face.holes.size() != 1) {
        // Returning rather than carrying on, because the assertions below index
        // into the hole. A check in this framework is non-fatal, so an unguarded
        // index after a failed check turns one reported failure into a crash
        // that costs every test after it in the binary.
        return;
    }

    // The area is re-measured from the COMPONENT's own geometry and not taken
    // from Component::measure, which is computed on the parent. A component that
    // dropped the hole still reports the right measure and has the wrong hole.
    CHECK_NEAR(stratum::procgen::rules::face_area(geometry, face), 96.0, 1e-9);

    // And the hole is still wound opposite to the loop, which is Face's contract:
    // a re-wound hole reads as a second outer boundary and gets filled in.
    glm::dvec3 loop_area{0.0};
    glm::dvec3 hole_area{0.0};
    for (size_t i = 0; i < face.loop.size(); ++i) {
        const glm::dvec3& a = geometry.positions[face.loop[i]];
        const glm::dvec3& b = geometry.positions[face.loop[(i + 1) % face.loop.size()]];
        loop_area += glm::cross(a, b);
    }
    for (size_t i = 0; i < face.holes[0].size(); ++i) {
        const glm::dvec3& a = geometry.positions[face.holes[0][i]];
        const glm::dvec3& b = geometry.positions[face.holes[0][(i + 1) % face.holes[0].size()]];
        hole_area += glm::cross(a, b);
    }
    CHECK((glm::dot(loop_area, hole_area)) < 0.0);
}

TEST(OpComp, a_degenerate_face_is_dropped_and_counted) {
    Shape shape = shape_from_rect(2.0, 2.0);
    // Three collinear points: a face with no normal, no plane and no area. It
    // cannot be classified and it cannot orient a scope.
    const auto base = static_cast<uint32_t>(shape.geometry.positions.size());
    shape.geometry.positions.push_back(glm::dvec3{0.0, 0.0, 0.0});
    shape.geometry.positions.push_back(glm::dvec3{1.0, 0.0, 0.0});
    shape.geometry.positions.push_back(glm::dvec3{2.0, 0.0, 0.0});
    Face sliver;
    sliver.loop = {base, base + 1, base + 2};
    shape.geometry.faces.push_back(sliver);

    ComponentSplitReport report;
    const std::vector<Component> faces =
        split_components(shape, ComponentDomain::Face, all_of_them(), &report);

    CHECK_EQ(faces.size(), size_t{1});
    CHECK_EQ(report.total, uint32_t{1});
    CHECK_EQ(report.degenerate, uint32_t{1});
}

TEST(OpComp, a_degenerate_face_does_not_renumber_the_real_ones) {
    const Shape clean = make_box();

    Shape dirty = clean;
    const auto base = static_cast<uint32_t>(dirty.geometry.positions.size());
    dirty.geometry.positions.push_back(glm::dvec3{0.0, 0.0, 0.0});
    dirty.geometry.positions.push_back(glm::dvec3{0.0, 0.0, 0.0});
    dirty.geometry.positions.push_back(glm::dvec3{0.0, 0.0, 0.0});
    Face collapsed;
    collapsed.loop = {base, base + 1, base + 2};
    // Inserted at the FRONT of the buffer, which is where it would do the most
    // damage to an implementation that numbered components before dropping them.
    dirty.geometry.faces.insert(dirty.geometry.faces.begin(), collapsed);

    const std::vector<Component> a = split_components(clean, ComponentDomain::Face, all_of_them());
    const std::vector<Component> b = split_components(dirty, ComponentDomain::Face, all_of_them());

    CHECK_EQ(a.size(), size_t{6});
    CHECK_EQ(b.size(), size_t{6});
    for (size_t i = 0; i < a.size() && i < b.size(); ++i) {
        CHECK_EQ(b[i].index, a[i].index);
        CHECK_EQ(dump_shape(b[i].shape), dump_shape(a[i].shape));
    }
}

TEST(OpComp, a_non_planar_quad_reports_its_deviation_and_keeps_it_as_scope_depth) {
    Shape bent;
    // A quad with one corner lifted: what OSM-derived geometry produces whenever
    // four footprint corners sit at four ground heights.
    const double lift = 0.25;
    bent.geometry.positions = {glm::dvec3{0.0, 0.0, 0.0}, glm::dvec3{0.0, 0.0, 2.0},
                               glm::dvec3{2.0, 0.0, 2.0}, glm::dvec3{2.0, lift, 0.0}};
    Face quad;
    quad.loop = {0, 1, 2, 3};
    bent.geometry.faces = {quad};
    stratum::procgen::rules::refit_scope(bent);

    ComponentSplitReport report;
    const std::vector<Component> faces =
        split_components(bent, ComponentDomain::Face, all_of_them(), &report);

    CHECK_EQ(faces.size(), size_t{1});
    CHECK_EQ(report.non_planar, uint32_t{1});
    CHECK((report.worst_planarity) > 1e-6);
    if (faces.size() != 1) {
        return;
    }

    // Not flattened. The out-of-plane thickness shows up as the component's
    // depth, which is honest and measurable; moving the vertices onto the plane
    // would open a crack against whatever face shares them.
    const double depth = faces.front().shape.scope.size.z;
    CHECK((depth) > 1e-6);
    // The plane passes through the centroid, so the full spread is at least the
    // worst one-sided deviation and at most twice it.
    CHECK((depth) >= report.worst_planarity - 1e-12);
    CHECK((depth) <= 2.0 * report.worst_planarity + 1e-12);

    // Those bounds hold whether the plane is anchored at the centroid or at the
    // ring's first vertex, and op_comp.hpp argues specifically for the centroid,
    // so the number itself is asserted. Newell's normal of this quad is
    // (-0.5, 8, 0.5); the centroid is (1, 0.0625, 1); all four corners are half
    // a unit off that plane before the normal is scaled to length, so every
    // deviation is 0.5 / |n| and the spread is twice it. Anchored at vertex 0
    // instead, the worst deviation would be 1 / |n|.
    const double expected_deviation = 0.5 / std::sqrt(64.5);
    CHECK_NEAR(report.worst_planarity, expected_deviation, 1e-12);
    CHECK_NEAR(depth, 2.0 * expected_deviation, 1e-12);

    // And the same quad written from a different corner reports the same
    // deviation, which is the property the centroid anchor exists for: an
    // OSM-derived ring starts wherever the way started.
    Shape from_another_corner = bent;
    std::rotate(from_another_corner.geometry.faces[0].loop.begin(),
                from_another_corner.geometry.faces[0].loop.begin() + 1,
                from_another_corner.geometry.faces[0].loop.end());
    ComponentSplitReport rotated_report;
    const std::vector<Component> rotated_faces = split_components(
        from_another_corner, ComponentDomain::Face, all_of_them(), &rotated_report);
    CHECK_EQ(rotated_faces.size(), size_t{1});
    CHECK_NEAR(rotated_report.worst_planarity, report.worst_planarity, 1e-12);

    // A planar face has none of this, which is what makes the assertions above
    // measure bending rather than measure nothing.
    ComponentSplitReport flat_report;
    const std::vector<Component> flat = split_components(
        shape_from_rect(2.0, 2.0), ComponentDomain::Face, all_of_them(), &flat_report);
    CHECK_EQ(flat_report.non_planar, uint32_t{0});
    CHECK_EQ(flat.size(), size_t{1});
    if (flat.size() == 1) {
        CHECK_NEAR(flat.front().shape.scope.size.z, 0.0, 1e-12);
    }
}

TEST(OpComp, a_shape_with_no_geometry_is_reported) {
    const Shape empty;
    ComponentSplitReport report;
    const std::vector<Component> faces =
        split_components(empty, ComponentDomain::Face, all_of_them(), &report);

    CHECK_EQ(faces.size(), size_t{0});
    CHECK_TRUE(has_problem(report, "no geometry"));
}

TEST(OpComp, the_vertex_and_object_domains_say_they_are_not_implemented) {
    const Shape box = make_box();

    ComponentSplitReport vertices;
    CHECK_EQ(split_components(box, ComponentDomain::Vertex, all_of_them(), &vertices).size(),
             size_t{0});
    CHECK_TRUE(has_problem(vertices, "'vertex' component domain is not implemented"));

    ComponentSplitReport objects;
    CHECK_EQ(split_components(box, ComponentDomain::Object, all_of_them(), &objects).size(),
             size_t{0});
    CHECK_TRUE(has_problem(objects, "'object' component domain is not implemented"));
}

// ============================================================================
// Edges
// ============================================================================

TEST(OpComp, a_square_splits_into_four_edges_oriented_along_themselves) {
    const Shape square = shape_from_rect(2.0, 4.0);
    const std::vector<Component> edges =
        split_components(square, ComponentDomain::Edge, all_of_them());

    CHECK_EQ(edges.size(), size_t{4});

    double perimeter = 0.0;
    for (const Component& edge : edges) {
        perimeter += edge.measure;
        // x runs the edge, and the edge has no width and no depth.
        CHECK_NEAR(edge.shape.scope.size.x, edge.measure, 1e-9);
        CHECK_NEAR(edge.shape.scope.size.y, 0.0, 1e-12);
        CHECK_NEAR(edge.shape.scope.size.z, 0.0, 1e-12);
        // z is the normal of the face the edge came from.
        check_vec3(edge.shape.scope.axes[2], glm::dvec3{0.0, 1.0, 0.0}, kEps);

        // y points INTO that face. Checked against the face's centre, because an
        // outward y produces a cornice growing away from the building and the
        // component count would not notice.
        const glm::dvec3 midpoint =
            0.5 * (edge.shape.scope.to_world(glm::dvec3{0.0}) +
                   edge.shape.scope.to_world(glm::dvec3{edge.shape.scope.size.x, 0.0, 0.0}));
        const glm::dvec3 face_centre = square.scope.to_world(glm::dvec3{1.0, 0.0, 2.0});
        CHECK((glm::dot(edge.shape.scope.axes[1], face_centre - midpoint)) > 0.0);
    }
    CHECK_NEAR(perimeter, 12.0, 1e-9);
}

TEST(OpComp, an_edge_shared_by_two_faces_is_produced_once_per_face) {
    const Shape box = make_box();
    const std::vector<Component> edges =
        split_components(box, ComponentDomain::Edge, all_of_them());

    // Six faces of four edges each. A cube has twelve distinct edges; the split
    // gives twenty-four because each face orients its own copy, and there is no
    // way to pick one of the two that does not put the face buffer order back
    // into the answer.
    CHECK_EQ(edges.size(), size_t{24});
    // The four edges of the top face, addressed by the face's own normal.
    CHECK_EQ(count_of(box, ComponentDomain::Edge, ComponentSelector::Top), size_t{4});
    CHECK_EQ(count_of(box, ComponentDomain::Edge, ComponentSelector::Side), size_t{16});
}

TEST(OpComp, the_edges_of_a_hole_are_components_too) {
    // The Face domain keeps a hole, and a_face_with_a_hole_keeps_the_hole_and_its_area
    // asserts that. The Edge domain has to WALK it as well, or a rule that runs
    // a coping along every edge of a courtyard leaves the courtyard bare, and
    // gather_edges() could skip hole rings entirely with nothing to notice.
    const std::vector<glm::dvec2> outer = {{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}};
    const std::vector<std::vector<glm::dvec2>> holes = {
        {{4.0, 4.0}, {6.0, 4.0}, {6.0, 6.0}, {4.0, 6.0}}};
    const Shape courtyard = shape_from_rings(outer, holes);

    const std::vector<Component> edges =
        split_components(courtyard, ComponentDomain::Edge, all_of_them());

    // Four around the outside and four around the hole.
    CHECK_EQ(edges.size(), size_t{8});

    double total = 0.0;
    size_t around_the_hole = 0;
    for (const Component& edge : edges) {
        total += edge.measure;
        if (edge.measure < 5.0) {
            ++around_the_hole;  // the hole's sides are 2 m, the outer ring's are 10 m
        }
        // Every one of them is an edge of the same face, so they share its normal.
        check_vec3(edge.shape.scope.axes[2], glm::dvec3{0.0, 1.0, 0.0}, kEps);
    }
    CHECK_NEAR(total, 48.0, 1e-9);  // 40 outside, 8 around the hole
    CHECK_EQ(around_the_hole, size_t{4});
}

TEST(OpComp, an_edge_component_has_a_scope_and_no_surface) {
    const Shape square = shape_from_rect(2.0, 4.0);
    const std::vector<Component> edges =
        split_components(square, ComponentDomain::Edge, all_of_them());
    CHECK_EQ(edges.size(), size_t{4});
    if (edges.empty()) {
        return;
    }

    // Two positions and no face. A two-vertex Face would break Face's contract
    // that a loop is a ring, and every consumer would have to special-case it.
    CHECK_EQ(edges.front().shape.geometry.positions.size(), size_t{2});
    CHECK_EQ(edges.front().shape.geometry.faces.size(), size_t{0});
}

TEST(OpComp, an_edge_frame_is_orthonormal_even_on_a_bent_face) {
    // An edge of a non-planar face is not exactly in that face's plane, so the
    // normal has to be orthogonalised against the edge rather than trusted.
    const glm::dmat3 axes =
        edge_component_axes(glm::dvec3{1.0, 0.0, 0.0}, glm::dvec3{0.2, 1.0, 0.0});

    CHECK_NEAR(glm::dot(axes[0], axes[1]), 0.0, 1e-12);
    CHECK_NEAR(glm::dot(axes[1], axes[2]), 0.0, 1e-12);
    CHECK_NEAR(glm::dot(axes[2], axes[0]), 0.0, 1e-12);
    CHECK_NEAR(glm::dot(axes[0], axes[0]), 1.0, 1e-12);
    // The edge direction is kept exactly: the length a rule measures along the
    // edge has to be the edge's own length.
    check_vec3(axes[0], glm::dvec3{1.0, 0.0, 0.0}, 1e-12);
    check_vec3(glm::cross(axes[0], axes[1]), axes[2], 1e-12);
}

TEST(OpComp, a_repeated_vertex_makes_no_edge_and_is_counted) {
    Shape shape = shape_from_rect(2.0, 2.0);
    // Duplicate the first vertex of the loop, which is what a cleanup pass that
    // has not run yet leaves behind.
    Face& face = shape.geometry.faces[0];
    const uint32_t first = face.loop.front();
    face.loop.insert(face.loop.begin() + 1, first);

    ComponentSplitReport report;
    const std::vector<Component> edges =
        split_components(shape, ComponentDomain::Edge, all_of_them(), &report);

    CHECK_EQ(edges.size(), size_t{4});
    CHECK_EQ(report.degenerate, uint32_t{1});
}

// ============================================================================
// Determinism
// ============================================================================

TEST(OpComp, two_splits_of_one_shape_agree_byte_for_byte) {
    const Shape box = make_box();
    const std::vector<Component> a = split_components(box, ComponentDomain::Face, all_of_them());
    const std::vector<Component> b = split_components(box, ComponentDomain::Face, all_of_them());

    CHECK_EQ(a.size(), size_t{6});
    CHECK_EQ(b.size(), a.size());

    std::string first;
    std::string second;
    for (size_t i = 0; i < a.size() && i < b.size(); ++i) {
        first += dump_shape(a[i].shape);
        second += dump_shape(b[i].shape);
    }
    // Asserted to be substantial before it is asserted to be equal: two empty
    // strings compare equal, and a dump that stopped printing would make this
    // test pass for a split that produced nothing.
    CHECK((first.size()) > 500u);
    CHECK_EQ(first, second);

    // And the comparison is sensitive: a different shape must not dump the same.
    const std::vector<Component> other =
        split_components(make_frustum(), ComponentDomain::Face, all_of_them());
    std::string third;
    for (const Component& component : other) {
        third += dump_shape(component.shape);
    }
    CHECK_TRUE(first != third);
}

// ============================================================================
// Reaching it from a rule file
// ============================================================================

TEST(OpComp, the_comp_functions_register_without_touching_the_interpreter) {
    // interpreter.hpp's contract for a later feature: copy a table, register into
    // the copy, pass it to generate(). This is that, end to end.
    CHECK_TRUE(component_functions().find("comp.count") != nullptr);
    CHECK_TRUE(component_functions().find("comp.area") != nullptr);
    CHECK_TRUE(component_functions().find("comp.size") != nullptr);
    // And the D2 library is still there, so registering did not replace it.
    CHECK_TRUE(component_functions().find("geometry.area") != nullptr);
    CHECK_TRUE(component_functions().find("min") != nullptr);
    CHECK_TRUE(component_functions().find("comp.nonsense") == nullptr);
}

TEST(OpComp, a_rule_counts_and_measures_its_components) {
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main {\n"
        "    extrude(3);\n"
        "    print(comp.count(), comp.count(\"front\"), comp.area(\"top\"));\n"
        "    print(comp.count(\"edge\", \"all\"), comp.size(\"front\", \"x\"));\n"
        "}\n",
        shape_from_rect(2.0, 4.0));

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.log.size(), size_t{2});
    if (result.log.size() != 2) {
        return;
    }
    // Six faces, one front, a roof of 2 by 4.
    CHECK_EQ(result.log[0], std::string{"6 1 8"});
    // Twenty-four edges, and a front wall 2 wide measured ACROSS THE WALL.
    CHECK_EQ(result.log[1], std::string{"24 2"});
}

TEST(OpComp, comp_size_measures_the_component_axis_and_not_the_world) {
    // After rotate_scope the front is what was the right wall: 4 wide and 3 high.
    // `shape.sx` at that point is the mass's own extent, which is a different
    // number, so a comp.size() that quietly read the parent scope fails here.
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main {\n"
        "    extrude(3);\n"
        "    rotate_scope(0, 90, 0);\n"
        "    print(comp.size(\"front\", \"x\"), comp.size(\"front\", \"y\"),\n"
        "          comp.size(\"front\", \"z\"));\n"
        "}\n",
        shape_from_rect(2.0, 4.0));

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.log.size(), size_t{1});
    if (result.log.size() == 1) {
        CHECK_EQ(result.log[0], std::string{"4 3 0"});
    }
}

TEST(OpComp, a_comp_query_with_no_such_component_fails_the_shape) {
    // Not zero. A rule reading the width of a wall that is not there has a
    // mistake in it, and a zero width propagates into a split that produces
    // nothing, several operations from the line that asked.
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main { print(comp.size(\"aslant\", \"x\")); }\n",
        shape_from_rect(2.0, 4.0));

    CHECK_FALSE(result.ok());
    CHECK_TRUE(has_message(result, Severity::Error, "found no 'aslant' component"));
    CHECK_EQ(result.log.size(), size_t{0});
}

TEST(OpComp, a_comp_query_says_so_when_it_does_not_know_a_word) {
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main { print(comp.count(\"forward\")); }\n",
        shape_from_rect(2.0, 4.0));

    CHECK_FALSE(result.ok());
    CHECK_TRUE(has_message(result, Severity::Error, "does not know the selector 'forward'"));
}

TEST(OpComp, a_comp_query_refuses_a_constant_that_has_no_shape) {
    // The same argument interpreter.cpp makes for geometry.area(): zero out of a
    // constant is a plausible number that looks like a blank wall rather than
    // like a fault.
    const GenerationResult result = run_on(
        "const walls : float = comp.count(\"side\")\n"
        "@start\n"
        "rule Main { print(walls); }\n",
        shape_from_rect(2.0, 4.0));

    CHECK_FALSE(result.ok());
    CHECK_TRUE(has_message(result, Severity::Error, "comp.count"));
    CHECK_TRUE(has_message(result, Severity::Error, "does not have"));
}

// ============================================================================
// Emission into the shape tree
// ============================================================================

namespace {

/**
 * @brief The body of the `case StmtKind::Select:` that interpreter.cpp does not have
 *
 * Registered over an EXISTING catalogue row, and only in this test, because
 * ast.hpp's kBuiltinOperations has no `comp` row to hang it on and the language
 * spells a component split as a statement rather than as an operation. See the
 * closing section of op_comp.hpp: the gap is in the extension point, not here.
 *
 * What this proves is the half of the feature that a query function cannot:
 * components becoming real shapes in the output, with their own address in the
 * tree and their own oriented scope.
 */
void test_component_operation(OperationArgs& context) {
    // The optional argument stands in for what the missing statement's head
    // would carry, so that the emit path's faults can be reached the way a rule
    // author reaches them -- from a rule file, at a line with a caret -- and not
    // only through split_components(). A domain word picks the domain; a number
    // asks for that component by index.
    ComponentDomain domain = ComponentDomain::Face;
    ComponentSelection selection;
    if (!context.args.empty() && context.args.front().is_text()) {
        CHECK_TRUE(parse_component_domain(context.args.front().as_text(), domain));
    }
    if (!context.args.empty() && context.args.front().is_number()) {
        selection.indices = {static_cast<uint32_t>(context.args.front().as_number())};
    }
    (void)emit_components(context.interpreter, context.shape, domain, selection, context.loc);
}

[[nodiscard]] OperationTable table_with_component_split() {
    OperationTable table = standard_operations();
    table.register_operation("cleanup", test_component_operation);
    return table;
}

} // namespace

TEST(OpComp, emitted_components_are_real_shapes_in_the_tree) {
    const OperationTable table = table_with_component_split();
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main { extrude(3); cleanup(); }\n",
        shape_from_rect(2.0, 4.0), &table);

    CHECK_TRUE(result.ok());
    // Six components plus the shape they came from, which made no rule children
    // and is therefore a terminal in its own right.
    CHECK_EQ(result.terminals.size(), size_t{7});

    // (label, sibling index) for every emitted component. BOTH numbers are read.
    // The label is what an author sees, so it has to be the component's place in
    // the canonical order and not a constant. The sibling index is the order the
    // components were EMITTED in, and it is what
    // Interpreter::emit_derived_terminal() mixes with the parent's key to give
    // each child its seed_key -- so emitting the same six components in a
    // different order would hand every one of them a different random address
    // while two identical runs still dumped identically. Neither number is
    // visible to a test that only matches the "comp.face[" prefix.
    std::vector<std::pair<uint32_t, uint32_t>> labelled;
    for (const Shape& terminal : result.terminals) {
        uint32_t label = 0;
        if (!role_index_of(terminal.rule, "comp.face[", label)) {
            continue;
        }
        labelled.emplace_back(label, terminal.index);
        // Its own address in the tree, derived from the parent's key rather than
        // drawn from a shared stream.
        CHECK(terminal.id != stratum::procgen::rules::kNoShape);
        CHECK(terminal.parent != stratum::procgen::rules::kNoShape);
        CHECK(terminal.seed_key != uint64_t{0});
        // Flat in its own z, which is what says the scope was oriented to the
        // face and not copied from the mass.
        CHECK_NEAR(terminal.scope.size.z, 0.0, 1e-12);
    }

    CHECK_EQ(labelled.size(), size_t{6});
    std::sort(labelled.begin(), labelled.end());
    for (size_t i = 0; i < labelled.size(); ++i) {
        // comp.face[0] .. comp.face[5], each exactly once.
        CHECK_EQ(labelled[i].first, static_cast<uint32_t>(i));
        // And emitted in that order, so a component's address in the tree
        // follows its place in the shape rather than the other way round.
        CHECK_EQ(labelled[i].second, static_cast<uint32_t>(i));
    }
}

TEST(OpComp, two_runs_that_emit_components_agree_byte_for_byte) {
    const OperationTable table = table_with_component_split();
    const std::string source =
        "@start\n"
        "rule Main { extrude(3); cleanup(); }\n";

    const GenerationResult a = run_on(source, shape_from_rect(2.0, 4.0), &table);
    const GenerationResult b = run_on(source, shape_from_rect(2.0, 4.0), &table);

    CHECK_EQ(a.terminals.size(), size_t{7});
    CHECK((a.dump().size()) > 500u);
    CHECK_EQ(a.dump(), b.dump());

    // Sensitive to the geometry, so the equality above is not two identical
    // renderings of nothing.
    const GenerationResult other = run_on(source, shape_from_rect(2.0, 5.0), &table);
    CHECK_TRUE(a.dump() != other.dump());

    // Two runs that differ in NOTHING agree for any deterministic emission
    // order, so one more pair of runs is made to differ in the one thing that
    // must not reach the output: the order the faces sit in the buffer. Each
    // child's dump carries its label and its sibling index, so a split that let
    // the buffer through would print a different one of either.
    Shape box = shape_from_rect(2.0, 4.0);
    const OpResult extruded = extrude_shape(box, ScopeAxis::Y, 3.0);
    CHECK_TRUE(extruded.ok);
    Shape reversed = box;
    std::reverse(reversed.geometry.faces.begin(), reversed.geometry.faces.end());

    const std::string on_a_solid =
        "@start\n"
        "rule Main { cleanup(); }\n";
    const std::string straight = emitted_components_dump(run_on(on_a_solid, box, &table));
    const std::string shuffled = emitted_components_dump(run_on(on_a_solid, reversed, &table));
    CHECK((straight.size()) > 500u);
    CHECK_EQ(shuffled, straight);
}

TEST(OpComp, an_emitted_split_reports_an_unimplemented_domain_as_an_error) {
    // op_comp.hpp's fourth promise is that nothing here is silent, and
    // emit_components() is the only place that promise is kept: it is what turns
    // ComponentSplitReport::problems into diagnostics at the line that asked.
    // Deleting that loop leaves split_components() perfectly correct and the
    // rule author with a facade that has no walls and no message.
    const OperationTable table = table_with_component_split();
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main { extrude(3); cleanup(\"vertex\"); }\n",
        shape_from_rect(2.0, 4.0), &table);

    // An Error, and therefore a failed run: asking for a domain this build does
    // not implement is a mistake in the rule, not a fact about the geometry.
    CHECK_FALSE(result.ok());
    CHECK_TRUE(
        has_message(result, Severity::Error, "'vertex' component domain is not implemented"));
    CHECK_FALSE(has_message(result, Severity::Warning, "component domain is not implemented"));
    // Nothing was emitted; the shape it was asked of is still a terminal.
    CHECK_EQ(result.terminals.size(), size_t{1});
}

TEST(OpComp, an_emitted_split_reports_an_index_past_the_end_as_an_error) {
    const OperationTable table = table_with_component_split();
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main { extrude(3); cleanup(9); }\n",
        shape_from_rect(2.0, 4.0), &table);

    // The box has six faces, so component 9 is a mistake in the rule. The
    // message names both numbers, because "index out of range" on its own sends
    // the author back to count the faces.
    CHECK_FALSE(result.ok());
    CHECK_TRUE(has_message(result, Severity::Error, "component 9 was asked for"));
    CHECK_TRUE(has_message(result, Severity::Error, "has 6"));
    CHECK_EQ(result.terminals.size(), size_t{1});
}

TEST(OpComp, an_emitted_split_reports_a_shape_with_no_geometry_as_an_error) {
    const OperationTable table = table_with_component_split();
    const Shape nothing;
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main { cleanup(); }\n",
        nothing, &table);

    CHECK_FALSE(result.ok());
    CHECK_TRUE(has_message(result, Severity::Error, "no geometry to split into components"));
}

TEST(OpComp, an_emitted_split_reports_a_degenerate_face_as_a_warning_and_keeps_going) {
    // The distinction this test exists for: the two paragraphs above report
    // Errors and fail the run, and this one reports a WARNING and does not,
    // because a collapsed face is something the input footprint did and not
    // something the rule author did. Flipping it to an Error would fail every
    // run over OSM-derived geometry.
    Shape footprint = shape_from_rect(2.0, 4.0);
    const auto base = static_cast<uint32_t>(footprint.geometry.positions.size());
    footprint.geometry.positions.push_back(glm::dvec3{0.0, 0.0, 0.0});
    footprint.geometry.positions.push_back(glm::dvec3{0.0, 0.0, 0.0});
    footprint.geometry.positions.push_back(glm::dvec3{0.0, 0.0, 0.0});
    Face collapsed;
    collapsed.loop = {base, base + 1, base + 2};
    footprint.geometry.faces.push_back(collapsed);

    const OperationTable table = table_with_component_split();
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main { cleanup(); }\n",
        footprint, &table);

    CHECK_TRUE(result.ok());
    CHECK_TRUE(has_message(result, Severity::Warning, "were dropped for having no area"));
    CHECK_FALSE(has_message(result, Severity::Error, "were dropped for having no area"));
    // The real face was still emitted: the warning does not cost the author the
    // component that was fine.
    CHECK_EQ(result.terminals.size(), size_t{2});
}

TEST(OpComp, an_emitted_split_reports_a_bent_face_as_a_warning) {
    Shape bent;
    bent.geometry.positions = {glm::dvec3{0.0, 0.0, 0.0}, glm::dvec3{0.0, 0.0, 2.0},
                               glm::dvec3{2.0, 0.0, 2.0}, glm::dvec3{2.0, 0.25, 0.0}};
    Face quad;
    quad.loop = {0, 1, 2, 3};
    bent.geometry.faces = {quad};
    stratum::procgen::rules::refit_scope(bent);

    const OperationTable table = table_with_component_split();
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main { cleanup(); }\n",
        bent, &table);

    // A Warning, not an Error: the facade will not sit flat, and the author has
    // to know, but the input geometry is what bent it and the run is still usable.
    CHECK_TRUE(result.ok());
    CHECK_TRUE(has_message(result, Severity::Warning, "do not lie flat"));
}

// ============================================================================
// The gap in the extension point
// ============================================================================

TEST(OpComp, the_select_statement_still_has_no_seat_in_the_interpreter) {
    // This test pins the gap op_comp.hpp reports rather than describing it in a
    // comment nobody runs. `select` is a STATEMENT, ast.hpp's StmtNode is a
    // closed variant, and State::exec() switches over it -- so a component split
    // written the way the language spells it cannot be reached without editing
    // interpreter.cpp, which this feature was asked not to do.
    //
    // When that case is written, this expectation flips: the assertion becomes
    // that the components were produced. Until then, an interpreter that stayed
    // SILENT about an unimplemented statement would be the worse failure, and
    // that is what is asserted here.
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main { extrude(3); select face { front: Facade(); } }\n"
        "rule Facade() { }\n",
        shape_from_rect(2.0, 4.0));

    CHECK_FALSE(result.ok());
    CHECK_TRUE(has_message(result, Severity::Error, "'select' is not implemented in this build"));
}
