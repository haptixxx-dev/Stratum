// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_op_cleanup.cpp
 * @brief `cleanup()` repairs a surveyed footprint without changing its shape
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The weak version of every test here is "it removed some vertices". That
 * passes for an operation that removes the WRONG ones, which would be a
 * simplifier rather than a repair and would quietly change the building.
 *
 * So the assertion that matters, and that appears in almost every test below,
 * is that the AREA IS UNCHANGED. A spur encloses nothing; a duplicate point
 * encloses nothing; a collinear vertex carries no shape. Removing any of them
 * is provably free, and a tolerance that moved a real corner would show up
 * here immediately.
 */

#include "framework.hpp"

#include "procgen/rules/op_cleanup.hpp"
#include "procgen/rules/parser.hpp"
#include "procgen/rules/registry.hpp"
#include "procgen/rules/shape.hpp"

#include <string>
#include <vector>

using namespace stratum::procgen::rules;

namespace {

/// A 10 x 10 square carrying a zero-width spike up the middle of its top edge
[[nodiscard]] std::vector<glm::dvec2> square_with_spur() {
    return {{0.0, 0.0},  {10.0, 0.0}, {10.0, 10.0}, {5.0, 10.0},
            {5.0, 15.0}, {5.0, 10.0}, {0.0, 10.0}};
}

[[nodiscard]] size_t ring_size(const Shape& shape) {
    return shape.geometry.faces.empty() ? 0 : shape.geometry.faces[0].loop.size();
}

[[nodiscard]] GenerationResult run(const std::string& source,
                                   const std::vector<glm::dvec2>& ring) {
    const ParseResult parsed = parse(source, "cleanup.srl");
    CHECK_EQ(parsed.render_all(source), std::string{});
    return generate(parsed.file, shape_from_polygon(ring), GenerationOptions{},
                    &full_operations(), &full_functions());
}

[[nodiscard]] bool has_error(const GenerationResult& result, const std::string& needle) {
    for (const Diagnostic& d : result.diagnostics) {
        if (d.severity == Severity::Error && d.message.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

} // namespace

// ============================================================================
// It repairs without reshaping
// ============================================================================

TEST(OpCleanup, a_spur_goes_and_the_area_does_not_move) {
    Shape shape = shape_from_polygon(square_with_spur());
    const double before = geometry_area(shape.geometry);
    CHECK_NEAR(before, 100.0, 1e-9);
    CHECK_EQ(ring_size(shape), size_t{7});

    CleanupReport report;
    CHECK_TRUE(cleanup_shape(shape, 1e-3, report).ok);

    // The spike tip AND the duplicate it leaves behind.
    CHECK((report.removed_spurs) >= 1);
    CHECK_EQ(ring_size(shape), size_t{4});

    // The whole claim: a repair, not a simplification. The spur enclosed no
    // area, so removing it changed the footprint by nothing at all.
    CHECK_NEAR(geometry_area(shape.geometry), 100.0, 1e-9);
}

TEST(OpCleanup, duplicate_points_merge_and_the_area_does_not_move) {
    std::vector<glm::dvec2> ring = {{0.0, 0.0},  {5.0, 0.0},  {5.0, 0.0},
                                    {10.0, 0.0}, {10.0, 8.0}, {0.0, 8.0}};
    Shape shape = shape_from_polygon(ring);
    CHECK_NEAR(geometry_area(shape.geometry), 80.0, 1e-9);

    CleanupReport report;
    CHECK_TRUE(cleanup_shape(shape, 1e-3, report).ok);
    CHECK((report.total_removed()) >= 2);  // the duplicate, and the collinear point
    CHECK_NEAR(geometry_area(shape.geometry), 80.0, 1e-9);
}

TEST(OpCleanup, a_collinear_vertex_goes_and_the_corners_stay) {
    std::vector<glm::dvec2> ring = {{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0},
                                    {10.0, 6.0}, {0.0, 6.0}};
    Shape shape = shape_from_polygon(ring);

    CleanupReport report;
    CHECK_TRUE(cleanup_shape(shape, 1e-3, report).ok);
    CHECK_EQ(report.removed_collinear, uint32_t{1});
    CHECK_EQ(ring_size(shape), size_t{4});
    CHECK_NEAR(geometry_area(shape.geometry), 60.0, 1e-9);
}

TEST(OpCleanup, a_clean_footprint_is_left_exactly_alone) {
    // The case that says the operation is not a simplifier wearing a repair's
    // name: nothing to fix means nothing removed.
    Shape shape = shape_from_rect(12.0, 8.0);
    CleanupReport report;
    CHECK_TRUE(cleanup_shape(shape, 1e-3, report).ok);
    CHECK_FALSE(report.changed_anything());
    CHECK_EQ(ring_size(shape), size_t{4});
    CHECK_NEAR(geometry_area(shape.geometry), 96.0, 1e-9);
}

TEST(OpCleanup, nested_spurs_are_removed_because_the_pass_repeats) {
    // Removing a spur tip makes its neighbours adjacent, which can expose
    // another spur behind it. A single pass leaves the second one, and the
    // roof still refuses. This is why cleanup loops until the ring settles.
    std::vector<glm::dvec2> ring = {{0.0, 0.0},  {10.0, 0.0}, {10.0, 10.0},
                                    {5.0, 10.0}, {5.0, 13.0}, {5.0, 16.0},
                                    {5.0, 13.0}, {5.0, 10.0}, {0.0, 10.0}};
    Shape shape = shape_from_polygon(ring);
    CHECK_NEAR(geometry_area(shape.geometry), 100.0, 1e-9);

    CleanupReport report;
    CHECK_TRUE(cleanup_shape(shape, 1e-3, report).ok);
    CHECK_EQ(ring_size(shape), size_t{4});
    CHECK_NEAR(geometry_area(shape.geometry), 100.0, 1e-9);
}

TEST(OpCleanup, a_hole_is_cleaned_too_and_a_collapsed_one_is_dropped) {
    const std::vector<glm::dvec2> outer = {{0.0, 0.0}, {20.0, 0.0}, {20.0, 20.0}, {0.0, 20.0}};
    const std::vector<std::vector<glm::dvec2>> holes = {
        // A real courtyard with a duplicate point in it.
        {{6.0, 6.0}, {6.0, 14.0}, {6.0, 14.0}, {14.0, 14.0}, {14.0, 6.0}},
        // A survey artefact: three points on one line, no area at all.
        {{2.0, 2.0}, {3.0, 2.0}, {4.0, 2.0}},
    };
    Shape shape = shape_from_rings(outer, holes);

    CleanupReport report;
    CHECK_TRUE(cleanup_shape(shape, 1e-3, report).ok);
    CHECK_EQ(shape.geometry.faces.size(), size_t{1});
    // The real courtyard survives; the artefact is gone and was counted.
    CHECK_EQ(shape.geometry.faces[0].holes.size(), size_t{1});
    CHECK((report.dropped_rings) >= 1);
    CHECK_NEAR(geometry_area(shape.geometry), 400.0 - 64.0, 1e-9);
}

// ============================================================================
// The reason it exists
// ============================================================================

TEST(OpCleanup, a_roof_that_a_spur_refused_is_raised_after_cleanup) {
    // THE POINT OF THE WHOLE FILE. Measured over the Lucan extract, 53 of 400
    // real footprints refused a pitched roof with "corner 0 of the outline is
    // a spike with no mitre" -- issue #133. This is that, in one test.
    const std::string without =
        "@start\n"
        "rule M { extrude(6.0); select face { top : { R(); } } }\n"
        "rule R { align_scope(\"y_up\"); roof(\"gable\", 35.0, 0.3); }\n";

    // A ring whose first point is repeated at the end -- the shape every OSM
    // way arrives in. shape_from_polygon does not strip it (osm/rule_seed.cpp
    // does that at the import seam), so a rule building this ring by hand
    // still meets the zero-length edge.
    const std::vector<glm::dvec2> closed = {{0.0, 0.0},  {10.0, 0.0}, {10.0, 8.0},
                                            {0.0, 8.0},  {0.0, 0.0}};
    const GenerationResult refused = run(without, closed);
    CHECK_FALSE(refused.ok());
    CHECK_TRUE(has_error(refused, "spike with no mitre"));

    const std::string with =
        "@start\n"
        "rule M { extrude(6.0); select face { top : { R(); } } }\n"
        "rule R { cleanup(); align_scope(\"y_up\"); roof(\"gable\", 35.0, 0.3); }\n";

    const GenerationResult fixed = run(with, closed);
    CHECK_TRUE(fixed.ok());
    CHECK_FALSE(has_error(fixed, "spike with no mitre"));

    // And it is a real roof over the real footprint, not an empty success.
    bool roofed = false;
    for (const Shape& terminal : fixed.terminals) {
        if (terminal.rule == "R" && terminal.geometry.faces.size() >= 4) roofed = true;
    }
    CHECK_TRUE(roofed);
}

// ============================================================================
// Refusals
// ============================================================================

TEST(OpCleanup, a_tolerance_that_is_not_positive_is_refused) {
    Shape shape = shape_from_rect(10.0, 10.0);
    CleanupReport report;
    CHECK_FALSE(cleanup_shape(shape, 0.0, report).ok);
    CHECK_FALSE(cleanup_shape(shape, -1.0, report).ok);
    // Refused, not half-applied.
    CHECK_EQ(ring_size(shape), size_t{4});
}

TEST(OpCleanup, cleaning_everything_away_is_a_failure_rather_than_an_empty_shape) {
    // A rule that cleans a footprint and silently gets nothing has lost a
    // building with no way to know it.
    Shape shape = shape_from_polygon({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}});
    CleanupReport report;
    const OpResult result = cleanup_shape(shape, 1e-3, report);
    CHECK_FALSE(result.ok);
    CHECK_TRUE(result.message.find("cleaned away") != std::string::npos);
}

TEST(OpCleanup, the_tolerance_argument_reaches_the_operation) {
    // A ring whose points are 5 cm apart: untouched at the default millimetre,
    // merged at ten centimetres. If the argument were ignored both would give
    // the same answer.
    const std::vector<glm::dvec2> ring = {{0.0, 0.0},  {5.0, 0.0},  {5.0, 0.05},
                                          {10.0, 0.0}, {10.0, 8.0}, {0.0, 8.0}};

    Shape tight = shape_from_polygon(ring);
    CleanupReport tight_report;
    CHECK_TRUE(cleanup_shape(tight, 1e-3, tight_report).ok);
    CHECK_EQ(tight_report.merged_points, uint32_t{0});

    Shape loose = shape_from_polygon(ring);
    CleanupReport loose_report;
    CHECK_TRUE(cleanup_shape(loose, 0.1, loose_report).ok);
    CHECK((loose_report.merged_points) >= 1);
}
