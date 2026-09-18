// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_instances.cpp
 * @brief F2 and D7: the placement data structure, and the `insert` that fills it
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ================================================================================
 * HOW THESE TESTS ARE WRITTEN SO THAT THEY CAN FAIL
 * ================================================================================
 *
 * The defect this project has found nine times is a test that cannot fail --
 * including a determinism suite resting on a dump() that was itself
 * geometry-blind. A placement is nothing but numbers, so the trap here is
 * especially easy to fall into: count the placements, find the right count, and
 * learn nothing about whether any of them is in the right place.
 *
 * So every assertion below is against a number computed by hand:
 *
 *   - **The end-to-end facade asserts twelve WORLD ORIGINS, each hit exactly
 *     once.** The expected values come from the arithmetic of the split
 *     statements -- `1.5 * bay + 0.4` across, `4 + 2 * floor` up, `4` out -- and
 *     they are asymmetric in both axes, so a facade built upside down, mirrored
 *     or one bay wide lands on none of them. A count of twelve passes for twelve
 *     benches stacked at the origin; this does not.
 *
 *   - **The AXES are asserted where they are not the identity.** A facade is
 *     axis-aligned, so on one the correct axes and a discarded copy of them are
 *     the same three vectors, and an assertion there says nothing. The frame is
 *     therefore checked on a forty-degree yaw at the struct level and through
 *     `rotate_scope` and `mirror_scope` end to end, where the identity, the
 *     transpose and the right answer are three different matrices. The pivot is
 *     asserted the same way: not carried, on a scope whose pivot is the centre
 *     of a box two metres across, so carrying it moves the origin a metre.
 *
 *   - **The product claim is asserted as an EQUATION.** `insert` must add
 *     nothing to the mesh, so the same rule file with and without the `insert`
 *     call must produce byte-identical vertex and index counts. That is the one
 *     statement this whole feature exists to make -- CityEngine merges, Stratum
 *     does not -- and it is the assertion an implementation that merged would
 *     fail first.
 *
 *   - **Inheritance is tested by a rule that inserts AND recurses.** Attributes
 *     inherit from a shape to its children. An implementation that marked the
 *     calling shape instead of emitting a separate leaf would pass every other
 *     test in this file and produce two placements here, one of them a phantom
 *     on the child. The test asserts exactly one.
 *
 *   - **The determinism test asserts BOTH directions.** Two runs must agree byte
 *     for byte, and two runs whose bay width differs by one millimetre must NOT.
 *     Agreement alone passes for a dump that prints only counts, which is
 *     precisely the defect named above. The second half is what says the dump
 *     carries the transform.
 *
 *   - **Bounds are asserted on a ROTATED placement.** An axis-aligned box hides
 *     the difference between expanding by two corners and expanding by eight;
 *     a frame turned 45 degrees does not, and the expected half-width is
 *     `sqrt(2)` by hand.
 *
 *   - **Every refusal asserts the sentence the code really writes, and counts it
 *     exactly.** `CHECK_EQ(count, 1)`, never `count <= 1`: an upper bound passes
 *     when the message disappears, which is half of what "reported once" means.
 *
 *   - **Registration is tested by its absence.** One test runs the same rule
 *     file through standard_operations() and asserts the "not implemented"
 *     diagnostic, so the suite cannot pass by the operation having been wired in
 *     somewhere else.
 *
 * Nothing here needs a GPU, a window or a file on disk, so nothing here skips.
 */

#include "framework.hpp"

#include "procgen/instances.hpp"
#include "procgen/rules/ast.hpp"
#include "procgen/rules/interpreter.hpp"
#include "procgen/rules/lexer.hpp"
#include "procgen/rules/op_insert.hpp"
#include "procgen/rules/parser.hpp"
#include "procgen/rules/shape.hpp"

#include <glm/glm.hpp>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

using stratum::Mesh;
using stratum::procgen::InstanceBounds;
using stratum::procgen::InstanceSet;
using stratum::procgen::InstanceTransform;
using stratum::procgen::rules::collect_instances;
using stratum::procgen::rules::Diagnostic;
using stratum::procgen::rules::GenerationOptions;
using stratum::procgen::rules::GenerationResult;
using stratum::procgen::rules::generate;
using stratum::procgen::rules::geometry_area;
using stratum::procgen::rules::InsertReport;
using stratum::procgen::rules::insert_operations;
using stratum::procgen::rules::insert_placement;
using stratum::procgen::rules::instance_transform;
using stratum::procgen::rules::is_placement;
using stratum::procgen::rules::kInsertAssetAttribute;
using stratum::procgen::rules::kInsertRole;
using stratum::procgen::rules::OpResult;
using stratum::procgen::rules::parse;
using stratum::procgen::rules::ParseResult;
using stratum::procgen::rules::scale_shape;
using stratum::procgen::rules::Scope;
using stratum::procgen::rules::Severity;
using stratum::procgen::rules::Shape;
using stratum::procgen::rules::shape_from_rect;
using stratum::procgen::rules::standard_functions;
using stratum::procgen::rules::standard_operations;
using stratum::procgen::rules::Value;

namespace {

// ============================================================================
// Helpers
// ============================================================================

/// A right-handed frame turned @p degrees about world y, exactly on the quarters
[[nodiscard]] glm::dmat3 yaw(double degrees) {
    double s = 0.0;
    double c = 0.0;
    stratum::procgen::rules::deg_sin_cos(degrees, s, c);
    // Columns: local x, y, z in world. Rotating about y sends +x to (c, 0, -s)
    // and +z to (s, 0, c), which is the right-handed sense.
    return glm::dmat3{glm::dvec3{c, 0.0, -s}, glm::dvec3{0.0, 1.0, 0.0}, glm::dvec3{s, 0.0, c}};
}

[[nodiscard]] InstanceTransform make_transform(const glm::dvec3& origin,
                                               const glm::dvec3& size,
                                               const glm::dmat3& axes = glm::dmat3{1.0}) {
    InstanceTransform transform;
    transform.origin = origin;
    transform.size = size;
    transform.axes = axes;
    return transform;
}

void check_vec3(const glm::dvec3& actual, const glm::dvec3& expected, double eps) {
    CHECK_NEAR(actual.x, expected.x, eps);
    CHECK_NEAR(actual.y, expected.y, eps);
    CHECK_NEAR(actual.z, expected.z, eps);
}

/// Parse @p source, failing the test with the parser's own message when it will not
[[nodiscard]] bool parse_ok(const std::string& source, ParseResult& out) {
    out = parse(source, "test.rule");
    const std::string rendered = out.render_all(source);
    if (!rendered.empty()) {
        std::printf("%s", rendered.c_str());
    }
    return rendered.empty();
}

/**
 * @brief Run @p source on @p seed
 *
 * @p standard_only picks D2's table, which has no `insert` in it. That is how
 * the registration-by-absence test below gets its "not implemented" diagnostic.
 */
[[nodiscard]] GenerationResult run_with(const std::string& source,
                                        const Shape& seed,
                                        bool standard_only) {
    ParseResult parsed;
    if (!parse_ok(source, parsed)) {
        return GenerationResult{};
    }
    const auto* operations = standard_only ? &standard_operations() : &insert_operations();
    return generate(parsed.file, seed, GenerationOptions{}, operations, &standard_functions());
}

[[nodiscard]] GenerationResult run(const std::string& source, const Shape& seed) {
    return run_with(source, seed, false);
}

/// Terminals whose Shape::rule is exactly @p name, in evaluation order
[[nodiscard]] std::vector<const Shape*> terminals_named(const GenerationResult& result,
                                                        const std::string& name) {
    std::vector<const Shape*> found;
    for (const Shape& shape : result.terminals) {
        if (shape.rule == name) {
            found.push_back(&shape);
        }
    }
    return found;
}

/// How many diagnostics contain @p needle
[[nodiscard]] size_t reported(const GenerationResult& result, const std::string& needle) {
    size_t count = 0;
    for (const Diagnostic& diagnostic : result.diagnostics) {
        if (diagnostic.message.find(needle) != std::string::npos) {
            ++count;
        }
    }
    return count;
}

/// A shape with no geometry at all, which is the case shape.hpp reserves for D7
[[nodiscard]] Shape empty_shape() {
    Shape shape;
    shape.scope.origin = glm::dvec3{3.0, 1.0, -2.0};
    return shape;
}

} // namespace

// ============================================================================
// InstanceTransform
// ============================================================================

TEST(Instances, a_transform_maps_the_unit_cube_onto_the_scope_box) {
    // A frame turned a quarter turn about y, so local x runs along world -z and
    // local z along world +x. An implementation that stored the axes as ROWS
    // instead of columns gets the opposite rotation and fails every corner but
    // the origin.
    const InstanceTransform transform =
        make_transform(glm::dvec3{10.0, 2.0, 5.0}, glm::dvec3{3.0, 4.0, 0.5}, yaw(90.0));

    check_vec3(transform.unit_to_world(glm::dvec3{0.0}), glm::dvec3{10.0, 2.0, 5.0}, 1e-12);

    // local x of 3 goes to world -z: (10, 2, 5) + 3 * (0, 0, -1)
    check_vec3(transform.unit_to_world(glm::dvec3{1.0, 0.0, 0.0}), glm::dvec3{10.0, 2.0, 2.0},
               1e-12);
    // local y of 4 is world up
    check_vec3(transform.unit_to_world(glm::dvec3{0.0, 1.0, 0.0}), glm::dvec3{10.0, 6.0, 5.0},
               1e-12);
    // local z of 0.5 goes to world +x
    check_vec3(transform.unit_to_world(glm::dvec3{0.0, 0.0, 1.0}), glm::dvec3{10.5, 2.0, 5.0},
               1e-12);

    check_vec3(transform.center(), glm::dvec3{10.25, 4.0, 3.5}, 1e-12);
}

TEST(Instances, the_matrix_and_unit_to_world_agree_on_every_corner) {
    // Two ways to say the same thing, and a consumer will use whichever suits
    // it. If they disagreed, a CPU-side bounds pass and a GPU-side instance
    // buffer would put the same bench in two different places.
    const InstanceTransform transform =
        make_transform(glm::dvec3{-4.0, 0.25, 7.5}, glm::dvec3{2.0, 6.0, 1.25}, yaw(270.0));
    const glm::dmat4 matrix = transform.to_matrix();

    for (int corner = 0; corner < 8; ++corner) {
        const glm::dvec3 unit{static_cast<double>((corner >> 0) & 1),
                              static_cast<double>((corner >> 1) & 1),
                              static_cast<double>((corner >> 2) & 1)};
        const glm::dvec4 through_matrix = matrix * glm::dvec4{unit, 1.0};
        check_vec3(glm::dvec3{through_matrix}, transform.unit_to_world(unit), 1e-12);
    }

    // And the matrix really carries the NON-UNIFORM scale, not one factor: the
    // three column lengths are the three extents. A uniform-scale implementation
    // passes a corner test on a cube and fails this.
    CHECK_NEAR(glm::length(glm::dvec3{matrix[0]}), 2.0, 1e-12);
    CHECK_NEAR(glm::length(glm::dvec3{matrix[1]}), 6.0, 1e-12);
    CHECK_NEAR(glm::length(glm::dvec3{matrix[2]}), 1.25, 1e-12);
}

TEST(Instances, a_mirrored_frame_is_left_handed_and_stays_so_through_the_matrix) {
    InstanceTransform transform = make_transform(glm::dvec3{0.0}, glm::dvec3{1.0, 2.0, 3.0});
    CHECK_TRUE(transform.right_handed());
    CHECK((glm::determinant(transform.to_matrix())) > 0.0);

    // `mirror_scope` negates one axis, which is the ordinary way to write a
    // facade whose right half mirrors its left. An exporter that missed this
    // draws the asset inside out.
    transform.axes[0] = -transform.axes[0];
    CHECK_FALSE(transform.right_handed());
    CHECK((glm::determinant(transform.to_matrix())) < 0.0);

    // A FLAT scope must not be mistaken for a mirrored one. to_matrix() is
    // singular here, so right_handed() has to read the axes and not the matrix.
    InstanceTransform flat = make_transform(glm::dvec3{0.0}, glm::dvec3{1.0, 2.0, 0.0});
    CHECK_TRUE(flat.right_handed());
    CHECK_NEAR(glm::determinant(flat.to_matrix()), 0.0, 1e-18);

    // The two determinants above are +1 and -1, so they cannot tell a strict
    // `> 0` from a `>= 0`. A DEGENERATE axis set can: the orthonormal invariant
    // excludes it, but a hand-built Shape carries one straight through
    // insert_placement(), which checks for a NaN and not for a rank. The
    // comparison is strict on purpose -- "right-handed" is never claimed for a
    // frame that has no handedness, because an exporter told that writes a
    // collapsed instance with no sign that anything was wrong.
    InstanceTransform degenerate = make_transform(glm::dvec3{0.0}, glm::dvec3{1.0, 2.0, 3.0});
    degenerate.axes = glm::dmat3{0.0};
    CHECK_NEAR(glm::determinant(degenerate.axes), 0.0, 1e-18);
    CHECK_FALSE(degenerate.right_handed());

    // The left-handed path is reached through `mirror_scope` and the operation
    // itself in a_mirrored_scope_reaches_the_placement_left_handed_through_insert;
    // everything above is a hand-built struct.
}

TEST(Instances, flat_axes_counts_the_zero_extents_and_a_point_is_three) {
    CHECK_EQ(make_transform(glm::dvec3{0.0}, glm::dvec3{1.0, 2.0, 3.0}).flat_axes(), 0);
    // The facade tile: a rectangle on a wall, flat in z. This is the normal case
    // and must not read as degenerate.
    CHECK_EQ(make_transform(glm::dvec3{0.0}, glm::dvec3{0.6, 2.0, 0.0}).flat_axes(), 1);
    CHECK_EQ(make_transform(glm::dvec3{0.0}, glm::dvec3{0.6, 0.0, 0.0}).flat_axes(), 2);
    CHECK_EQ(make_transform(glm::dvec3{0.0}, glm::dvec3{0.0}).flat_axes(), 3);

    CHECK_FALSE(make_transform(glm::dvec3{0.0}, glm::dvec3{0.6, 2.0, 0.0}).is_point());
    CHECK_TRUE(make_transform(glm::dvec3{0.0}, glm::dvec3{0.0}).is_point());

    // Exactly zero, and not a tolerance. Every extent above is either 0.0 or of
    // order one, so an implementation that called anything under 1e-6 flat --
    // a constant nobody chose -- satisfies all of it. A reveal a nanometre deep
    // is a real extent, and a scope a picometre across is not a point.
    CHECK_EQ(make_transform(glm::dvec3{0.0}, glm::dvec3{0.6, 2.0, 1e-9}).flat_axes(), 0);
    CHECK_EQ(make_transform(glm::dvec3{0.0}, glm::dvec3{1e-12, 1e-12, 1e-12}).flat_axes(), 0);
    CHECK_FALSE(make_transform(glm::dvec3{0.0}, glm::dvec3{1e-12}).is_point());
    // ...and one exact zero beside two tiny extents still counts as one.
    CHECK_EQ(make_transform(glm::dvec3{0.0}, glm::dvec3{1e-9, 0.0, 1e-9}).flat_axes(), 1);
}

TEST(Instances, transform_equality_is_exact_and_reads_every_field) {
    const InstanceTransform base =
        make_transform(glm::dvec3{1.0, 2.0, 3.0}, glm::dvec3{4.0, 5.0, 6.0}, yaw(90.0));
    CHECK_TRUE(base == base);

    // One field at a time, because an operator== that compared only the origin
    // would pass a test that changed everything at once.
    InstanceTransform moved = base;
    moved.origin.y += 1e-15;
    CHECK_TRUE(base != moved);

    InstanceTransform resized = base;
    resized.size.z += 1e-15;
    CHECK_TRUE(base != resized);

    InstanceTransform turned = base;
    turned.axes[2].x += 1e-15;
    CHECK_TRUE(base != turned);
}

// ============================================================================
// InstanceBounds
// ============================================================================

TEST(Instances, bounds_cover_all_eight_corners_of_a_rotated_placement) {
    // A 2x2 square in the xz plane turned 45 degrees about y, its minimum corner
    // at the world origin. Local x of 2 goes to (sqrt(2), 0, -sqrt(2)), local z
    // of 2 to (sqrt(2), 0, sqrt(2)), and the far corner to (2*sqrt(2), 0, 0) --
    // so the world box runs from z = -sqrt(2) to z = +sqrt(2) and x = 0 to
    // x = 2*sqrt(2).
    //
    // An implementation that expanded by the minimum corner and the far corner
    // alone gets z from 0 to 0 and misses half the shape. An axis-aligned test
    // cannot tell the two apart, which is why this one is turned.
    const double r = std::sqrt(2.0);
    const InstanceTransform transform =
        make_transform(glm::dvec3{0.0}, glm::dvec3{2.0, 0.0, 2.0}, yaw(45.0));

    InstanceBounds box;
    CHECK_FALSE(box.valid());
    box.expand(transform);
    CHECK_TRUE(box.valid());

    check_vec3(box.min, glm::dvec3{0.0, 0.0, -r}, 1e-12);
    check_vec3(box.max, glm::dvec3{2.0 * r, 0.0, r}, 1e-12);
    check_vec3(box.extent(), glm::dvec3{2.0 * r, 0.0, 2.0 * r}, 1e-12);
    check_vec3(box.center(), glm::dvec3{r, 0.0, 0.0}, 1e-12);
}

TEST(Instances, an_empty_set_has_no_bounds_rather_than_a_box_at_the_origin) {
    // A box that defaulted to (0,0,0)-(0,0,0) reports the world origin as being
    // inside the city, and a culling pass built on it keeps one tile alive
    // forever.
    const InstanceSet set;
    CHECK_TRUE(set.empty());
    CHECK_EQ(set.asset_count(), size_t{0});
    CHECK_EQ(set.placement_count(), size_t{0});
    CHECK_FALSE(set.bounds().valid());
    CHECK_FALSE(set.bounds_of("props/bench.glb").valid());
    CHECK_TRUE(set.find("props/bench.glb") == nullptr);
    CHECK_EQ(set.count_of("props/bench.glb"), size_t{0});
}

TEST(Instances, per_asset_bounds_cover_that_asset_alone) {
    InstanceSet set;
    CHECK_TRUE(set.add("a.glb", make_transform(glm::dvec3{0.0}, glm::dvec3{1.0})));
    CHECK_TRUE(set.add("b.glb", make_transform(glm::dvec3{100.0}, glm::dvec3{1.0})));

    const InstanceBounds a = set.bounds_of("a.glb");
    check_vec3(a.min, glm::dvec3{0.0}, 1e-12);
    check_vec3(a.max, glm::dvec3{1.0}, 1e-12);

    // The whole-set box spans both, which is the statement that the per-asset
    // box is not just the whole-set box under another name.
    const InstanceBounds all = set.bounds();
    check_vec3(all.min, glm::dvec3{0.0}, 1e-12);
    check_vec3(all.max, glm::dvec3{101.0}, 1e-12);
}

// ============================================================================
// InstanceSet
// ============================================================================

TEST(Instances, assets_iterate_in_path_order_whatever_the_insertion_order) {
    // A std::unordered_map passes a count test and fails this one on some other
    // standard library, which is exactly the failure the map exists to prevent.
    InstanceSet set;
    CHECK_TRUE(set.add("zebra.glb", make_transform(glm::dvec3{0.0}, glm::dvec3{1.0})));
    CHECK_TRUE(set.add("apple.glb", make_transform(glm::dvec3{0.0}, glm::dvec3{1.0})));
    CHECK_TRUE(set.add("mango.glb", make_transform(glm::dvec3{0.0}, glm::dvec3{1.0})));

    std::vector<std::string> order;
    for (const auto& entry : set.assets()) {
        order.push_back(entry.first);
    }
    CHECK_EQ(order.size(), size_t{3});
    if (order.size() == 3) {
        CHECK_EQ(order[0], std::string{"apple.glb"});
        CHECK_EQ(order[1], std::string{"mango.glb"});
        CHECK_EQ(order[2], std::string{"zebra.glb"});
    }
}

TEST(Instances, placements_of_one_asset_keep_the_order_they_were_added_in) {
    // Never sorted by position: a sort key built from a double re-orders two
    // placements that differ in the last bit, and an exporter writing in
    // iteration order then produces a different file for the same city.
    InstanceSet set;
    for (int i = 0; i < 5; ++i) {
        const double x = 4.0 - static_cast<double>(i);
        CHECK_TRUE(set.add("bench.glb", make_transform(glm::dvec3{x, 0.0, 0.0}, glm::dvec3{1.0})));
    }

    const std::vector<InstanceTransform>* list = set.find("bench.glb");
    CHECK_TRUE(list != nullptr);
    if (list != nullptr) {
        CHECK_EQ(list->size(), size_t{5});
        for (size_t i = 0; i < list->size(); ++i) {
            CHECK_NEAR((*list)[i].origin.x, 4.0 - static_cast<double>(i), 1e-12);
        }
    }
    CHECK_EQ(set.count_of("bench.glb"), size_t{5});
    CHECK_EQ(set.asset_count(), size_t{1});
    CHECK_EQ(set.placement_count(), size_t{5});
}

TEST(Instances, an_empty_asset_path_is_refused_rather_than_given_a_bucket) {
    InstanceSet set;
    CHECK_FALSE(set.add("", make_transform(glm::dvec3{0.0}, glm::dvec3{1.0})));
    CHECK_TRUE(set.empty());
    CHECK_EQ(set.placement_count(), size_t{0});
}

TEST(Instances, merge_appends_and_keeps_each_assets_order) {
    InstanceSet first;
    CHECK_TRUE(first.add("a.glb", make_transform(glm::dvec3{1.0, 0.0, 0.0}, glm::dvec3{1.0})));
    CHECK_TRUE(first.add("b.glb", make_transform(glm::dvec3{2.0, 0.0, 0.0}, glm::dvec3{1.0})));

    InstanceSet second;
    CHECK_TRUE(second.add("a.glb", make_transform(glm::dvec3{3.0, 0.0, 0.0}, glm::dvec3{1.0})));
    CHECK_TRUE(second.add("c.glb", make_transform(glm::dvec3{4.0, 0.0, 0.0}, glm::dvec3{1.0})));

    first.merge(second);
    CHECK_EQ(first.asset_count(), size_t{3});
    CHECK_EQ(first.placement_count(), size_t{4});

    const std::vector<InstanceTransform>* a = first.find("a.glb");
    CHECK_TRUE(a != nullptr);
    if (a != nullptr) {
        CHECK_EQ(a->size(), size_t{2});
        // The merged one goes after, not before: merge appends.
        CHECK_NEAR((*a)[0].origin.x, 1.0, 1e-12);
        CHECK_NEAR((*a)[1].origin.x, 3.0, 1e-12);
    }
}

TEST(Instances, the_dump_carries_every_coordinate_of_every_placement) {
    // This is the test that makes the determinism tests below able to fail. A
    // dump that printed counts alone would satisfy every "two runs agree"
    // assertion in this file while the placements were all at the origin.
    InstanceSet a;
    CHECK_TRUE(a.add("prop.glb", make_transform(glm::dvec3{1.0, 2.0, 3.0}, glm::dvec3{4.0, 5.0, 6.0})));

    InstanceSet same;
    CHECK_TRUE(same.add("prop.glb",
                        make_transform(glm::dvec3{1.0, 2.0, 3.0}, glm::dvec3{4.0, 5.0, 6.0})));
    CHECK_EQ(a.dump(), same.dump());

    // Every one of the fifteen numbers, moved one at a time. Three hand-picked
    // perturbations -- an origin, a size and one whole rotation -- leave holes:
    // yaw(90) differs from the identity in the x column AND the z column, so a
    // dump that printed two of the three columns still tells them apart, and a
    // single turned case cannot see a column go missing. "Every coordinate" is
    // only proved by moving every coordinate.
    const glm::dvec3 base_origin{1.0, 2.0, 3.0};
    const glm::dvec3 base_size{4.0, 5.0, 6.0};
    for (int k = 0; k < 3; ++k) {
        InstanceTransform moved = make_transform(base_origin, base_size);
        moved.origin[k] += 0.000001;
        InstanceSet moved_set;
        CHECK_TRUE(moved_set.add("prop.glb", moved));
        CHECK_TRUE(a.dump() != moved_set.dump());

        InstanceTransform resized = make_transform(base_origin, base_size);
        resized.size[k] += 0.5;
        InstanceSet resized_set;
        CHECK_TRUE(resized_set.add("prop.glb", resized));
        CHECK_TRUE(a.dump() != resized_set.dump());

        for (int c = 0; c < 3; ++c) {
            InstanceTransform bent = make_transform(base_origin, base_size);
            bent.axes[k][c] += 0.25;
            InstanceSet bent_set;
            CHECK_TRUE(bent_set.add("prop.glb", bent));
            CHECK_TRUE(a.dump() != bent_set.dump());
        }
    }

    // And a whole rotation, which is the difference between a bench facing the
    // street and one facing the wall.
    InstanceSet turned;
    CHECK_TRUE(turned.add("prop.glb", make_transform(base_origin, base_size, yaw(90.0))));
    CHECK_TRUE(a.dump() != turned.dump());
}

// ============================================================================
// insert_placement(): the core, with no interpreter
// ============================================================================

TEST(Instances, a_placement_copies_the_scope_and_keeps_no_geometry) {
    Shape wall = shape_from_rect(6.0, 4.0);
    wall.scope.origin = glm::dvec3{10.0, 3.0, -1.0};
    wall.scope.axes = yaw(90.0);

    Shape placement;
    InsertReport report;
    const OpResult result = insert_placement(wall, "props/bench.glb", placement, &report);
    CHECK_TRUE(result.ok);
    CHECK_EQ(report.warnings.size(), size_t{0});
    CHECK_FALSE(report.point_scope);

    // The scope is carried verbatim -- that IS the transform.
    check_vec3(placement.scope.origin, wall.scope.origin, 1e-15);
    check_vec3(placement.scope.size, wall.scope.size, 1e-15);
    for (int k = 0; k < 3; ++k) {
        check_vec3(placement.scope.axes[k], wall.scope.axes[k], 1e-15);
    }

    // And no geometry, which is what keeps it out of build_mesh(). The wall it
    // came from still has its quad: `insert` does not consume the shape.
    CHECK_TRUE(placement.geometry.empty());
    CHECK_EQ(placement.geometry.positions.size(), size_t{0});
    CHECK_EQ(wall.geometry.faces.size(), size_t{1});
    CHECK_NEAR(geometry_area(wall.geometry), 24.0, 1e-12);

    // The transform read back out of it matches the scope it was taken from --
    // the origin, the size AND the axes. The axes are the half a copy can get
    // wrong in a way no origin or size assertion notices, so they are written
    // out as literals rather than compared against wall.scope.axes: a
    // TRANSPOSED copy satisfies a comparison against the source on the diagonal
    // and fails these. A quarter turn about y sends local x to world -z and
    // local z to world +x.
    const InstanceTransform transform = instance_transform(placement.scope);
    check_vec3(transform.origin, glm::dvec3{10.0, 3.0, -1.0}, 1e-15);
    check_vec3(transform.size, glm::dvec3{6.0, 0.0, 4.0}, 1e-15);
    check_vec3(transform.axes[0], glm::dvec3{0.0, 0.0, -1.0}, 1e-15);
    check_vec3(transform.axes[1], glm::dvec3{0.0, 1.0, 0.0}, 1e-15);
    check_vec3(transform.axes[2], glm::dvec3{1.0, 0.0, 0.0}, 1e-15);
}

TEST(Instances, instance_transform_carries_the_axes_as_columns_and_never_the_pivot) {
    // instance_transform() is the one function this whole feature is about, and
    // every END-TO-END test in this file runs on an axis-aligned facade where
    // the correct axes ARE the identity. That makes them blind to an
    // implementation that discarded the scope axes, so the discriminating case
    // is here: forty degrees is not a quarter turn, so the identity, the
    // transpose and the right answer are three different matrices.
    double sine = 0.0;
    double cosine = 0.0;
    stratum::procgen::rules::deg_sin_cos(40.0, sine, cosine);

    Shape shape = empty_shape();
    shape.scope.axes = yaw(40.0);
    shape.scope.size = glm::dvec3{2.0, 3.0, 4.0};
    // The pivot at the centre of the box. instances.hpp spends a section arguing
    // that it must NOT be carried, and carrying it would move the origin by
    // axes * (pivot * size) -- roughly (2.05, 1.5, 0.89) here, which is far
    // outside every tolerance below.
    shape.scope.pivot = glm::dvec3{0.5};

    Shape placement;
    CHECK_TRUE(insert_placement(shape, "props/bench.glb", placement, nullptr).ok);
    const InstanceTransform transform = instance_transform(placement.scope);

    // Column by column. axes[0] is local x IN WORLD; the transposed matrix keeps
    // its first component and flips its third, which is the single most likely
    // real fault in a function whose body copies a matrix.
    check_vec3(transform.axes[0], glm::dvec3{cosine, 0.0, -sine}, 1e-15);
    check_vec3(transform.axes[1], glm::dvec3{0.0, 1.0, 0.0}, 1e-15);
    check_vec3(transform.axes[2], glm::dvec3{sine, 0.0, cosine}, 1e-15);

    // The origin is the box MINIMUM, untouched by the pivot.
    check_vec3(transform.origin, glm::dvec3{3.0, 1.0, -2.0}, 1e-15);

    // And the frame really turns a direction rather than merely being stored:
    // local x of 2 lands two metres along (cos 40, 0, -sin 40).
    check_vec3(transform.unit_to_world(glm::dvec3{1.0, 0.0, 0.0}),
               glm::dvec3{3.0 + 2.0 * cosine, 1.0, -2.0 - 2.0 * sine}, 1e-15);
    // The far corner needs all three columns and both other extents, so a frame
    // right in x alone does not reach it.
    check_vec3(transform.unit_to_world(glm::dvec3{1.0}),
               glm::dvec3{3.0 + 2.0 * cosine + 4.0 * sine, 4.0, -2.0 - 2.0 * sine + 4.0 * cosine},
               1e-15);
}

TEST(Instances, a_placement_inherits_the_attributes_of_the_shape_it_was_placed_on) {
    // A consumer asking "which floor is this bench on" reads it off the
    // placement, without having to find the parent shape it came from.
    Shape wall = shape_from_rect(2.0, 2.0);
    wall.attributes["floor_count"] = Value::number(6.0);

    Shape placement;
    CHECK_TRUE(insert_placement(wall, "props/bench.glb", placement, nullptr).ok);
    const auto it = placement.attributes.find("floor_count");
    CHECK_TRUE(it != placement.attributes.end());
    if (it != placement.attributes.end()) {
        CHECK_TRUE(it->second.is_number());
        CHECK_NEAR(it->second.as_number(), 6.0, 1e-15);
    }

    const auto asset = placement.attributes.find(std::string{kInsertAssetAttribute});
    CHECK_TRUE(asset != placement.attributes.end());
    if (asset != placement.attributes.end()) {
        CHECK_TRUE(asset->second.is_text());
        CHECK_EQ(asset->second.as_text(), std::string{"props/bench.glb"});
    }
}

TEST(Instances, a_blank_asset_path_is_refused_with_the_sentence_the_code_writes) {
    const Shape wall = shape_from_rect(2.0, 2.0);
    Shape placement;

    const OpResult empty = insert_placement(wall, "", placement, nullptr);
    CHECK_FALSE(empty.ok);
    CHECK_EQ(empty.message, std::string{"wants an asset path, not an empty string"});

    // Whitespace is nothing at all, and it is what a rule file written with a
    // trailing tab in a string literal produces.
    const OpResult blank = insert_placement(wall, "  \t ", placement, nullptr);
    CHECK_FALSE(blank.ok);
    CHECK_EQ(blank.message, std::string{"wants an asset path, not an empty string"});
}

TEST(Instances, a_negative_or_non_finite_scope_is_refused_and_names_the_axis) {
    // Neither is reachable from a rule file today -- `scale` refuses a negative
    // size and refit_scope() derives the rest -- but a placement is DATA that
    // outlives the generation, and a NaN in an instance buffer takes out a draw
    // call a long way from here.
    Shape shape = empty_shape();
    Shape placement;

    shape.scope.size = glm::dvec3{1.0, -2.0, 3.0};
    const OpResult negative = insert_placement(shape, "a.glb", placement, nullptr);
    CHECK_FALSE(negative.ok);
    CHECK_EQ(negative.message,
             std::string{"cannot place an asset in a scope of negative size on y"});

    shape.scope.size = glm::dvec3{1.0, 2.0, 3.0};
    shape.scope.origin.x = std::nan("");
    const OpResult not_a_number = insert_placement(shape, "a.glb", placement, nullptr);
    CHECK_FALSE(not_a_number.ok);
    CHECK_EQ(not_a_number.message,
             std::string{"cannot place an asset in a scope holding a value that is not a number"});

    // An infinite AXIS too, not only the origin: the axes are nine of the
    // fifteen numbers and a check that read only the origin and the size would
    // pass here.
    shape.scope.origin.x = 0.0;
    shape.scope.axes[1][2] = std::numeric_limits<double>::infinity();
    CHECK_FALSE(insert_placement(shape, "a.glb", placement, nullptr).ok);
}

TEST(Instances, a_scope_with_no_size_warns_and_is_still_recorded) {
    // The author almost certainly forgot `scale`. Warning, not failure: "place
    // it at this point at its native size" is a legitimate thing to want, and
    // dropping the placement would lose data silently.
    const Shape shape = empty_shape();

    Shape placement;
    InsertReport report;
    const OpResult result = insert_placement(shape, "props/bench.glb", placement, &report);
    CHECK_TRUE(result.ok);
    CHECK_TRUE(report.point_scope);
    CHECK_EQ(report.warnings.size(), size_t{1});
    if (report.warnings.size() == 1) {
        CHECK_TRUE(report.warnings[0].find("props/bench.glb") != std::string::npos);
        CHECK_TRUE(report.warnings[0].find("use 'scale'") != std::string::npos);
    }

    // Recorded anyway, with the orientation intact. The role is supplied by
    // emit_derived_terminal() one layer up, so it is set by hand here -- that is
    // the whole of what the interpreter adds to what insert_placement() built.
    Shape terminal = placement;
    terminal.rule = std::string{kInsertRole};
    std::string asset;
    CHECK_TRUE(is_placement(terminal, &asset));
    CHECK_EQ(asset, std::string{"props/bench.glb"});
    CHECK_TRUE(instance_transform(terminal.scope).is_point());
    check_vec3(terminal.scope.origin, glm::dvec3{3.0, 1.0, -2.0}, 1e-15);
}

TEST(Instances, a_flat_facade_tile_is_not_a_point_and_says_nothing) {
    // The case the whole feature was built for: a window frame into a bay. A
    // warning here would fire on every window of every building.
    Shape tile;
    tile.scope.size = glm::dvec3{0.6, 2.0, 0.0};

    Shape placement;
    InsertReport report;
    CHECK_TRUE(insert_placement(tile, "props/window.glb", placement, &report).ok);
    CHECK_FALSE(report.point_scope);
    CHECK_EQ(report.warnings.size(), size_t{0});
    CHECK_EQ(instance_transform(placement.scope).flat_axes(), 1);
}

TEST(Instances, a_geometry_less_shape_takes_its_size_from_scale_and_carries_it_into_the_placement) {
    // shape.hpp reserves the geometry-less shape precisely so a rule can build a
    // scope and then insert into it, and `scale` is the only operation that can
    // set that scope's size. Before that was fixed it was a silent no-op, and
    // the symptom would have been exactly this: every asset at a point.
    Shape shape = empty_shape();
    CHECK_TRUE(scale_shape(shape, glm::dvec3{1.5, 0.8, 0.5}).ok);
    check_vec3(shape.scope.size, glm::dvec3{1.5, 0.8, 0.5}, 1e-15);

    Shape placement;
    InsertReport report;
    CHECK_TRUE(insert_placement(shape, "props/bench.glb", placement, &report).ok);
    CHECK_FALSE(report.point_scope);
    CHECK_EQ(report.warnings.size(), size_t{0});

    const InstanceTransform transform = instance_transform(placement.scope);
    check_vec3(transform.size, glm::dvec3{1.5, 0.8, 0.5}, 1e-15);
    check_vec3(transform.origin, glm::dvec3{3.0, 1.0, -2.0}, 1e-15);
    check_vec3(transform.center(), glm::dvec3{3.75, 1.4, -1.75}, 1e-15);
}

// ============================================================================
// Recognising a placement
// ============================================================================

TEST(Instances, a_placement_needs_both_marks_so_neither_alone_is_a_false_positive) {
    Shape shape;
    shape.rule = std::string{kInsertRole};
    shape.attributes[std::string{kInsertAssetAttribute}] = Value::text("props/bench.glb");

    std::string asset;
    CHECK_TRUE(is_placement(shape, &asset));
    CHECK_EQ(asset, std::string{"props/bench.glb"});

    // A rule the author named `insert` is not a placement.
    Shape role_only;
    role_only.rule = std::string{kInsertRole};
    CHECK_FALSE(is_placement(role_only, nullptr));

    // Nor is a shape that merely carries the attribute -- which is what a future
    // `set("insert.asset", ...)` would produce, and which INHERITS.
    Shape attribute_only;
    attribute_only.rule = "Wall";
    attribute_only.attributes[std::string{kInsertAssetAttribute}] = Value::text("props/bench.glb");
    CHECK_FALSE(is_placement(attribute_only, nullptr));

    // Nor is one whose attribute is the wrong type or empty.
    Shape wrong_type = shape;
    wrong_type.attributes[std::string{kInsertAssetAttribute}] = Value::number(3.0);
    CHECK_FALSE(is_placement(wrong_type, nullptr));

    Shape empty_path = shape;
    empty_path.attributes[std::string{kInsertAssetAttribute}] = Value::text("");
    CHECK_FALSE(is_placement(empty_path, nullptr));
}

// ============================================================================
// The rule language, end to end
// ============================================================================

namespace {

/**
 * @brief The facade of tests/procgen/test_rule_statements.cpp, with an asset per bay
 *
 * @p pier_width is the FIXED left pier of each bay, so it moves every window
 * across without changing how many there are. The bay period is held at 1.5 on
 * purpose: `repeat` makes `floor(S / P)` copies of length `S / n`, so perturbing
 * the period changes the copy count and then divides back to the same length,
 * and a determinism test written against it would compare two identical dumps.
 */
[[nodiscard]] std::string facade_source(const std::string& window_body, double pier_width = 0.4) {
    char pier[32];
    std::snprintf(pier, sizeof(pier), "%.4f", pier_width);
    return std::string{
               "@start\n"
               "rule Main {\n"
               "    extrude(10.0);\n"
               "    select face {\n"
               "        front : { Facade(); }\n"
               "    }\n"
               "}\n"
               "rule Facade {\n"
               "    split(y) {\n"
               "        4.0 : { Shopfront(); }\n"
               "        repeat { 2.0 : { UpperFloor(); } }\n"
               "    }\n"
               "}\n"
               "rule UpperFloor {\n"
               "    split(x) { repeat { 1.5 : { Bay(); } } }\n"
               "}\n"
               "rule Bay {\n"
               "    split(x) {\n"
               "        "} +
           pier +
           std::string{" : { Pier(); }\n"
                       "        0.6 : { Window(); }\n"
                       "        ~1.0 : { Pier(); }\n"
                       "    }\n"
                       "}\n"
                       "rule Shopfront { }\n"
                       "rule Pier { }\n"
                       "rule Window { "} +
           window_body + std::string{" }\n"};
}

} // namespace

TEST(Instances, a_facade_rule_places_one_asset_per_bay_where_hand_arithmetic_puts_it) {
    // The arithmetic, from the split statements alone:
    //   extrude(10) on a 6 x 4 footprint, front face is the 6 x 10 wall at z = 4
    //   split(y): 4 for the shopfront, then floor(6 / 2) = 3 upper floors of 2
    //   split(x): floor(6 / 1.5) = 4 bays of 1.5
    //   split(x) in a bay: 0.4 pier, 0.6 window, 0.5 pier
    // so window bay j on floor k has its scope MINIMUM at
    //   (1.5 j + 0.4, 4 + 2 k, 4) and its size is (0.6, 2, 0).
    // Asymmetric in both axes on purpose: a facade built upside down, mirrored
    // or one bay wide hits none of these.
    const GenerationResult result =
        run(facade_source("insert(\"props/window.glb\");"), shape_from_rect(6.0, 4.0));

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.diagnostics.size(), size_t{0});

    // 37 shapes of the facade, plus one placement per window.
    CHECK_EQ(result.terminals.size(), size_t{49});
    CHECK_EQ(terminals_named(result, "Window").size(), size_t{12});
    CHECK_EQ(terminals_named(result, std::string{kInsertRole}).size(), size_t{12});

    const InstanceSet set = collect_instances(result);
    CHECK_EQ(set.asset_count(), size_t{1});
    CHECK_EQ(set.placement_count(), size_t{12});

    const std::vector<InstanceTransform>* placements = set.find("props/window.glb");
    CHECK_TRUE(placements != nullptr);
    if (placements == nullptr) {
        return;
    }
    CHECK_EQ(placements->size(), size_t{12});

    std::vector<bool> matched(placements->size(), false);
    size_t found = 0;
    for (int floor = 0; floor < 3; ++floor) {
        for (int bay = 0; bay < 4; ++bay) {
            const glm::dvec3 expected{1.5 * static_cast<double>(bay) + 0.4,
                                      4.0 + 2.0 * static_cast<double>(floor), 4.0};
            for (size_t i = 0; i < placements->size(); ++i) {
                if (matched[i]) {
                    continue;
                }
                const glm::dvec3 origin = (*placements)[i].origin;
                if (std::fabs(origin.x - expected.x) < 1e-9 &&
                    std::fabs(origin.y - expected.y) < 1e-9 &&
                    std::fabs(origin.z - expected.z) < 1e-9) {
                    matched[i] = true;
                    ++found;
                    break;
                }
            }
        }
    }
    CHECK_EQ(found, size_t{12});

    // Every one is the size of its bay slot and oriented to the wall: x across
    // it, y up it, z out of it. A placement that took the parent's scope instead
    // of the tile's would be 6 by 10 and would pass a count test.
    //
    // The three axis assertions are the WALL's own frame, which is the identity,
    // so they are also what an implementation that dropped `scope.axes`
    // altogether would produce -- they pin the facade's orientation and say
    // nothing about the copy. The statement that the axes are carried at all is
    // made where the frame is not the identity:
    // instance_transform_carries_the_axes_as_columns_and_never_the_pivot at the
    // struct level, and the `rotate_scope` and `mirror_scope` arms below through
    // the operation.
    for (const InstanceTransform& transform : *placements) {
        check_vec3(transform.size, glm::dvec3{0.6, 2.0, 0.0}, 1e-9);
        check_vec3(transform.axes[0], glm::dvec3{1.0, 0.0, 0.0}, 1e-9);
        check_vec3(transform.axes[1], glm::dvec3{0.0, 1.0, 0.0}, 1e-9);
        check_vec3(transform.axes[2], glm::dvec3{0.0, 0.0, 1.0}, 1e-9);
        CHECK_TRUE(transform.right_handed());
        CHECK_EQ(transform.flat_axes(), 1);
    }

    // And the whole set sits on the front wall, six metres wide and from the top
    // of the shopfront to the top of the building.
    const InstanceBounds box = set.bounds();
    CHECK_TRUE(box.valid());
    check_vec3(box.min, glm::dvec3{0.4, 4.0, 4.0}, 1e-9);
    check_vec3(box.max, glm::dvec3{5.5, 10.0, 4.0}, 1e-9);
}

TEST(Instances, a_rotated_scope_reaches_the_placement_as_the_frame_insert_ran_in) {
    // The same facade, with one `rotate_scope` in the window rule, so that the
    // expected frame is one no default and no dropped copy can reach. This is
    // the end-to-end half of the axes contract; the facade test above runs
    // entirely on the identity.
    //
    // The arithmetic. `rotate_scope` turns the frame forty degrees about y and
    // leaves the geometry where it is, then the box refits around it. For a
    // 0.6 x 2 tile lying in the plane z = 4:
    //   local x spans 0.6 cos40, local z spans 0.6 sin40, local y is unchanged;
    //   both minima sit on the SAME world corner -- the one at minimum x, since
    //   cos40 and sin40 are both positive -- so the origin is that corner and is
    //   exactly where it was before the rotation.
    double sine = 0.0;
    double cosine = 0.0;
    stratum::procgen::rules::deg_sin_cos(40.0, sine, cosine);

    const GenerationResult result =
        run(facade_source("rotate_scope(0.0, 40.0, 0.0); insert(\"props/window.glb\");"),
            shape_from_rect(6.0, 4.0));
    CHECK_TRUE(result.ok());
    CHECK_EQ(result.diagnostics.size(), size_t{0});

    const InstanceSet set = collect_instances(result);
    CHECK_EQ(set.placement_count(), size_t{12});
    const std::vector<InstanceTransform>* placements = set.find("props/window.glb");
    CHECK_TRUE(placements != nullptr);
    if (placements == nullptr || placements->size() != 12) {
        return;
    }

    for (const InstanceTransform& transform : *placements) {
        check_vec3(transform.axes[0], glm::dvec3{cosine, 0.0, -sine}, 1e-12);
        check_vec3(transform.axes[1], glm::dvec3{0.0, 1.0, 0.0}, 1e-12);
        check_vec3(transform.axes[2], glm::dvec3{sine, 0.0, cosine}, 1e-12);
        check_vec3(transform.size, glm::dvec3{0.6 * cosine, 2.0, 0.6 * sine}, 1e-12);
        CHECK_TRUE(transform.right_handed());
        // No longer flat on any axis: the tile is a rectangle held at an angle
        // inside its own frame, so the tight box has depth in local z.
        CHECK_EQ(transform.flat_axes(), 0);
    }

    // The origins are exactly the twelve of the unrotated facade -- see the
    // arithmetic above -- so this test differs from that one in the axes, the
    // sizes and the bounds and in nothing else.
    for (int floor = 0; floor < 3; ++floor) {
        for (int bay = 0; bay < 4; ++bay) {
            const InstanceTransform& transform =
                (*placements)[static_cast<size_t>(floor * 4 + bay)];
            CHECK_NEAR(transform.origin.x, 1.5 * static_cast<double>(bay) + 0.4, 1e-9);
            CHECK_NEAR(transform.origin.y, 4.0 + 2.0 * static_cast<double>(floor), 1e-9);
            CHECK_NEAR(transform.origin.z, 4.0, 1e-9);
        }
    }

    // The world bounds are the second, independent way this test sees the axes:
    // unit_to_world() runs every corner through them, so the boxes now lean out
    // of the wall by 0.6 sin40 cos40 either side of z = 4. An implementation
    // that dropped the axes leaves the bounds flat at z = 4 exactly.
    const InstanceBounds box = set.bounds();
    CHECK_TRUE(box.valid());
    check_vec3(box.min, glm::dvec3{0.4, 4.0, 4.0 - 0.6 * sine * cosine}, 1e-9);
    check_vec3(box.max, glm::dvec3{5.5, 10.0, 4.0 + 0.6 * sine * cosine}, 1e-9);
    // In x it is the same wall either way: the projection of the turned box back
    // onto world x is 0.6 (cos^2 + sin^2), which is 0.6.
    CHECK_NEAR(box.extent().x, 5.1, 1e-9);
}

TEST(Instances, a_mirrored_scope_reaches_the_placement_left_handed_through_insert) {
    // `mirror_scope` negates one axis, which is how a facade mirrors its left
    // half onto its right, and the handedness has to survive to the placement or
    // an exporter draws the window inside out -- back faces to the street. Every
    // other handedness assertion in this file is made against a hand-built
    // struct; this one goes through the operation and the interpreter.
    //
    // The frame flips and the geometry does not, so the box refits and the
    // origin moves to the other end of the tile: local x now runs from the
    // tile's right edge back towards its left, and the right edge is
    // 1.5 bay + 0.4 + 0.6.
    const GenerationResult result =
        run(facade_source("mirror_scope(\"x\"); insert(\"props/window.glb\");"),
            shape_from_rect(6.0, 4.0));
    CHECK_TRUE(result.ok());
    CHECK_EQ(result.diagnostics.size(), size_t{0});

    const InstanceSet set = collect_instances(result);
    CHECK_EQ(set.placement_count(), size_t{12});
    const std::vector<InstanceTransform>* placements = set.find("props/window.glb");
    CHECK_TRUE(placements != nullptr);
    if (placements == nullptr || placements->size() != 12) {
        return;
    }

    for (const InstanceTransform& transform : *placements) {
        check_vec3(transform.axes[0], glm::dvec3{-1.0, 0.0, 0.0}, 1e-12);
        check_vec3(transform.axes[1], glm::dvec3{0.0, 1.0, 0.0}, 1e-12);
        check_vec3(transform.axes[2], glm::dvec3{0.0, 0.0, 1.0}, 1e-12);
        CHECK_FALSE(transform.right_handed());
        CHECK_NEAR(glm::determinant(transform.axes), -1.0, 1e-12);
        // The size is still the bay slot: mirroring turns the frame round, it
        // does not resize anything.
        check_vec3(transform.size, glm::dvec3{0.6, 2.0, 0.0}, 1e-12);
        CHECK_EQ(transform.flat_axes(), 1);
    }

    for (int floor = 0; floor < 3; ++floor) {
        for (int bay = 0; bay < 4; ++bay) {
            const InstanceTransform& transform =
                (*placements)[static_cast<size_t>(floor * 4 + bay)];
            CHECK_NEAR(transform.origin.x, 1.5 * static_cast<double>(bay) + 1.0, 1e-9);
            CHECK_NEAR(transform.origin.y, 4.0 + 2.0 * static_cast<double>(floor), 1e-9);
            CHECK_NEAR(transform.origin.z, 4.0, 1e-9);
        }
    }

    // The statement in one line: the unit cube's far corner in x is the tile's
    // LEFT edge, because the frame runs backwards along world x.
    check_vec3((*placements)[0].unit_to_world(glm::dvec3{1.0, 0.0, 0.0}),
               glm::dvec3{0.4, 4.0, 4.0}, 1e-9);

    // And the wall the set covers is the same wall as the unmirrored facade's.
    // The frames differ; the geometry they sit on does not.
    const InstanceBounds box = set.bounds();
    CHECK_TRUE(box.valid());
    check_vec3(box.min, glm::dvec3{0.4, 4.0, 4.0}, 1e-9);
    check_vec3(box.max, glm::dvec3{5.5, 10.0, 4.0}, 1e-9);
}

TEST(Instances, moving_the_scope_pivot_does_not_move_the_placement) {
    // `set_pivot` changes what a later `rotate` or `scale` turns about and
    // nothing else. instances.hpp is explicit that the pivot must not reach the
    // transform: a consumer handed both a box minimum and a pivot has two
    // contradictory answers to "where does the asset go", and they are wrong in
    // different places.
    //
    // An implementation that carried it would put every window half a bay across
    // and one metre up -- (0.7, 5, 4) instead of (0.4, 4, 4) for the first --
    // which is what these origins assert against.
    const GenerationResult result =
        run(facade_source("set_pivot(\"center\"); insert(\"props/window.glb\");"),
            shape_from_rect(6.0, 4.0));
    CHECK_TRUE(result.ok());
    CHECK_EQ(result.diagnostics.size(), size_t{0});

    const InstanceSet set = collect_instances(result);
    CHECK_EQ(set.placement_count(), size_t{12});
    const std::vector<InstanceTransform>* placements = set.find("props/window.glb");
    CHECK_TRUE(placements != nullptr);
    if (placements == nullptr || placements->size() != 12) {
        return;
    }

    for (int floor = 0; floor < 3; ++floor) {
        for (int bay = 0; bay < 4; ++bay) {
            const InstanceTransform& transform =
                (*placements)[static_cast<size_t>(floor * 4 + bay)];
            CHECK_NEAR(transform.origin.x, 1.5 * static_cast<double>(bay) + 0.4, 1e-9);
            CHECK_NEAR(transform.origin.y, 4.0 + 2.0 * static_cast<double>(floor), 1e-9);
            CHECK_NEAR(transform.origin.z, 4.0, 1e-9);
        }
    }

    // The centre of the box is where the pivot now sits, and it is the
    // coordinate the two answers would have collided on: the placement reports
    // it as a centre, never as an origin.
    check_vec3((*placements)[0].center(), glm::dvec3{0.7, 5.0, 4.0}, 1e-9);

    // Byte for byte the same placements as the facade with no `set_pivot` at
    // all, which is the whole claim stated once more as an equation.
    const InstanceSet plain =
        collect_instances(run(facade_source("insert(\"props/window.glb\");"),
                              shape_from_rect(6.0, 4.0)));
    CHECK_EQ(set.dump(), plain.dump());
}

TEST(Instances, an_inserted_asset_adds_not_one_triangle_to_the_mesh) {
    // THE product claim. CityEngine merges the asset into the building and the
    // instances are gone; Stratum keeps the transform list and the mesh is
    // untouched. Byte-identical counts, not "roughly the same".
    const Shape seed = shape_from_rect(6.0, 4.0);
    const GenerationResult without = run(facade_source(""), seed);
    const GenerationResult with =
        run(facade_source("insert(\"props/window.glb\");"), seed);

    CHECK_TRUE(without.ok());
    CHECK_TRUE(with.ok());

    const Mesh plain = without.build_mesh();
    const Mesh inserted = with.build_mesh();
    CHECK_EQ(inserted.vertices.size(), plain.vertices.size());
    CHECK_EQ(inserted.indices.size(), plain.indices.size());
    CHECK_TRUE(plain.indices.size() > 0);

    // The paired half, without which this would pass for an `insert` that did
    // nothing whatsoever: the placements are there, twelve of them.
    CHECK_EQ(collect_instances(without).placement_count(), size_t{0});
    CHECK_EQ(collect_instances(with).placement_count(), size_t{12});
}

TEST(Instances, the_shape_that_inserted_is_still_a_terminal_and_the_placement_is_a_second_one) {
    const GenerationResult result =
        run(facade_source("insert(\"props/window.glb\");"), shape_from_rect(6.0, 4.0));

    const std::vector<const Shape*> windows = terminals_named(result, "Window");
    const std::vector<const Shape*> placements = terminals_named(result, std::string{kInsertRole});
    CHECK_EQ(windows.size(), size_t{12});
    CHECK_EQ(placements.size(), size_t{12});

    // The Window shape keeps its wall quad and still meshes.
    for (const Shape* window : windows) {
        CHECK_NEAR(geometry_area(window->geometry), 1.2, 1e-9);
        CHECK_FALSE(is_placement(*window, nullptr));
    }

    // The placement produced no children AND no geometry of its own, which is
    // what makes it an instance rather than a mesh.
    for (const Shape* placement : placements) {
        CHECK_TRUE(placement->geometry.empty());
        CHECK_TRUE(is_placement(*placement, nullptr));
        // A derived child sits at its parent's depth; see emit_derived_terminal().
        CHECK_TRUE(placement->parent != stratum::procgen::rules::kNoShape);
    }
}

TEST(Instances, a_rule_that_inserts_and_then_recurses_still_places_exactly_one) {
    // The inheritance trap. Attributes inherit from a shape to its children, so
    // an implementation that marked the CALLING shape instead of emitting a
    // separate leaf would mark Window, Sub would inherit the mark, and both
    // would be counted -- two placements, one of them a phantom in the wrong
    // scope. Everything else in this file passes for that implementation.
    const std::string source =
        "@start\n"
        "rule Main { split(x) { 1.0 : { Window(); } ~1.0 : { } } }\n"
        "rule Window { insert(\"props/window.glb\"); Sub(); }\n"
        "rule Sub { }\n";

    const GenerationResult result = run(source, shape_from_rect(4.0, 2.0));
    CHECK_TRUE(result.ok());

    CHECK_EQ(terminals_named(result, "Window").size(), size_t{0});
    CHECK_EQ(terminals_named(result, "Sub").size(), size_t{1});

    const InstanceSet set = collect_instances(result);
    CHECK_EQ(set.placement_count(), size_t{1});
    CHECK_EQ(set.count_of("props/window.glb"), size_t{1});

    // And the one placement carries the scope as it stood WHEN insert ran, which
    // is the 1.0-wide slab.
    const std::vector<InstanceTransform>* placements = set.find("props/window.glb");
    if (placements != nullptr && placements->size() == 1) {
        check_vec3((*placements)[0].size, glm::dvec3{1.0, 0.0, 2.0}, 1e-9);
    }
}

TEST(Instances, two_rules_inserting_the_same_asset_share_one_transform_list) {
    // The second case the brief names. Neither rule knows about the other; the
    // grouping is by path, on the way in.
    const std::string source =
        "@start\n"
        "rule Main { split(x) { 1.0 : { Left(); } 1.0 : { Right(); } ~1.0 : { } } }\n"
        "rule Left  { insert(\"props/bench.glb\"); }\n"
        "rule Right { insert(\"props/bench.glb\"); insert(\"props/bin.glb\"); }\n";

    const GenerationResult result = run(source, shape_from_rect(6.0, 2.0));
    CHECK_TRUE(result.ok());

    const InstanceSet set = collect_instances(result);
    CHECK_EQ(set.asset_count(), size_t{2});
    CHECK_EQ(set.placement_count(), size_t{3});
    CHECK_EQ(set.count_of("props/bench.glb"), size_t{2});
    CHECK_EQ(set.count_of("props/bin.glb"), size_t{1});

    // In evaluation order: the left slab first, then the right one. Positions,
    // not just a count -- a grouping that reversed the list would pass the count.
    const std::vector<InstanceTransform>* bench = set.find("props/bench.glb");
    CHECK_TRUE(bench != nullptr);
    if (bench != nullptr && bench->size() == 2) {
        CHECK_NEAR((*bench)[0].origin.x, 0.0, 1e-9);
        CHECK_NEAR((*bench)[1].origin.x, 1.0, 1e-9);
    }
}

TEST(Instances, two_inserts_on_one_shape_are_two_placements_with_two_addresses) {
    const std::string source =
        "@start\n"
        "rule Main { insert(\"a.glb\"); insert(\"b.glb\"); }\n";

    const GenerationResult result = run(source, shape_from_rect(2.0, 2.0));
    CHECK_TRUE(result.ok());

    const std::vector<const Shape*> placements = terminals_named(result, std::string{kInsertRole});
    CHECK_EQ(placements.size(), size_t{2});
    if (placements.size() == 2) {
        // Separate derived children, so separate sibling indices and separate
        // seeds. A shared address would make a later stochastic operation on one
        // placement repeat on the other.
        CHECK_EQ(placements[0]->index, uint32_t{0});
        CHECK_EQ(placements[1]->index, uint32_t{1});
        CHECK_TRUE(placements[0]->seed_key != placements[1]->seed_key);
        CHECK_TRUE(placements[0]->id != placements[1]->id);
    }

    const InstanceSet set = collect_instances(result);
    CHECK_EQ(set.asset_count(), size_t{2});
    CHECK_EQ(set.count_of("a.glb"), size_t{1});
    CHECK_EQ(set.count_of("b.glb"), size_t{1});

    // Main made two derived children but no ordinary ones, so it is still a
    // terminal in its own right and still meshes.
    CHECK_EQ(terminals_named(result, "Main").size(), size_t{1});
    CHECK_EQ(result.terminals.size(), size_t{3});
}

// ============================================================================
// Diagnostics through the interpreter
// ============================================================================

TEST(Instances, insert_is_absent_from_the_standard_table_and_says_so) {
    // Registration tested by its absence: this file cannot pass by `insert`
    // having been wired in somewhere else.
    const std::string source = "@start\nrule Main { insert(\"a.glb\"); }\n";

    const GenerationResult standard = run_with(source, shape_from_rect(1.0, 1.0), true);
    CHECK_EQ(reported(standard, "not implemented in this build"), size_t{1});
    CHECK_EQ(collect_instances(standard).placement_count(), size_t{0});

    const GenerationResult full = run_with(source, shape_from_rect(1.0, 1.0), false);
    CHECK_EQ(reported(full, "not implemented in this build"), size_t{0});
    CHECK_EQ(collect_instances(full).placement_count(), size_t{1});
}

TEST(Instances, insert_refuses_a_non_string_argument_at_the_line_that_wrote_it) {
    const std::string source = "@start\nrule Main { insert(3.0); }\n";
    const GenerationResult result = run(source, shape_from_rect(1.0, 1.0));

    CHECK_FALSE(result.ok());
    CHECK_EQ(reported(result, "'insert' wants an asset path as a string, not float"), size_t{1});
    CHECK_EQ(collect_instances(result).placement_count(), size_t{0});
}

TEST(Instances, an_empty_path_abandons_only_the_subtree_that_wrote_it) {
    // One malformed lot must not cost the other four thousand. `Good` still
    // generates and its placement still lands.
    const std::string source =
        "@start\n"
        "rule Main { split(x) { 1.0 : { Bad(); } ~1.0 : { Good(); } } }\n"
        "rule Bad  { insert(\"\"); }\n"
        "rule Good { insert(\"props/bench.glb\"); }\n";

    const GenerationResult result = run(source, shape_from_rect(4.0, 2.0));
    CHECK_FALSE(result.ok());
    CHECK_EQ(reported(result, "'insert' wants an asset path, not an empty string"), size_t{1});

    CHECK_EQ(terminals_named(result, "Bad").size(), size_t{0});
    CHECK_EQ(terminals_named(result, "Good").size(), size_t{1});

    const InstanceSet set = collect_instances(result);
    CHECK_EQ(set.asset_count(), size_t{1});
    CHECK_EQ(set.count_of("props/bench.glb"), size_t{1});
}

TEST(Instances, a_point_scope_warns_once_however_many_shapes_reach_the_line) {
    // A rule run over a city must not spend the whole diagnostic budget saying
    // the same sentence about the same line. Exactly one, not at most one: an
    // upper bound passes when the message disappears altogether.
    //
    // A `split` slab always has geometry, so the only way to reach a point scope
    // from a rule file today is a shape whose scope was never given a size --
    // which is what the seed here is.
    const std::string source =
        "@start\n"
        "rule Main { Prop(); Prop(); Prop(); }\n"
        "rule Prop { insert(\"props/bench.glb\"); }\n";

    Shape seed;
    seed.scope.origin = glm::dvec3{5.0, 0.0, 5.0};

    ParseResult parsed;
    CHECK_TRUE(parse_ok(source, parsed));
    const GenerationResult result = generate(parsed.file, seed, GenerationOptions{},
                                             &insert_operations(), &standard_functions());

    // A warning, not an error: the run still succeeds and the placements land.
    CHECK_TRUE(result.ok());
    CHECK_EQ(reported(result, "the scope has no size"), size_t{1});

    const InstanceSet set = collect_instances(result);
    CHECK_EQ(set.placement_count(), size_t{3});
    const std::vector<InstanceTransform>* placements = set.find("props/bench.glb");
    if (placements != nullptr) {
        for (const InstanceTransform& transform : *placements) {
            CHECK_TRUE(transform.is_point());
            check_vec3(transform.origin, glm::dvec3{5.0, 0.0, 5.0}, 1e-12);
        }
    }
}

// ============================================================================
// Determinism
// ============================================================================

TEST(Instances, the_same_rule_file_gives_the_same_placements_byte_for_byte) {
    const std::string source = facade_source("insert(\"props/window.glb\");");
    const Shape seed = shape_from_rect(6.0, 4.0);

    const InstanceSet first = collect_instances(run(source, seed));
    const InstanceSet second = collect_instances(run(source, seed));

    const std::string a = first.dump();
    const std::string b = second.dump();
    // Big enough that it is demonstrably the twelve placements and not a header
    // line: two lines of coordinates each, at about seventy bytes a placement.
    CHECK_TRUE(a.size() > 800);
    CHECK_EQ(a, b);

    // The paired half, and the one that makes the equality above mean something.
    // A centimetre on the pier moves every window on every floor across, and
    // changes nothing else: the same twelve placements, the same twelve sizes,
    // twelve different origins. A dump that could not tell these apart could not
    // tell a working run from a broken one either.
    const InstanceSet moved =
        collect_instances(run(facade_source("insert(\"props/window.glb\");", 0.41), seed));
    CHECK_TRUE(a != moved.dump());
    // ...and it is the positions that differ, not the count.
    CHECK_EQ(moved.placement_count(), first.placement_count());
}

TEST(Instances, the_placement_order_within_an_asset_is_the_evaluation_order) {
    // Not merely "the same on both runs", which a sort would also satisfy. The
    // bays are cut left to right, so the placements come out left to right, and
    // the x coordinates are strictly increasing.
    const InstanceSet set = collect_instances(
        run(facade_source("insert(\"props/window.glb\");"), shape_from_rect(6.0, 4.0)));

    const std::vector<InstanceTransform>* placements = set.find("props/window.glb");
    CHECK_TRUE(placements != nullptr);
    if (placements == nullptr || placements->size() != 12) {
        return;
    }

    // Floor by floor, four bays each, ascending x within a floor and ascending y
    // between them.
    for (size_t floor = 0; floor < 3; ++floor) {
        for (size_t bay = 0; bay < 4; ++bay) {
            const InstanceTransform& transform = (*placements)[floor * 4 + bay];
            CHECK_NEAR(transform.origin.x, 1.5 * static_cast<double>(bay) + 0.4, 1e-9);
            CHECK_NEAR(transform.origin.y, 4.0 + 2.0 * static_cast<double>(floor), 1e-9);
        }
    }
}
