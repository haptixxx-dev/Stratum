// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_rule_statements.cpp
 * @brief `split` and `select` as the interpreter runs them: children, frames, seeds and caps
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ================================================================================
 * WHAT THIS SUITE IS FOR
 * ================================================================================
 *
 * The sizing solver, the slab cut and the component decomposition are tested
 * against hand-computed numbers in test_op_split.cpp and test_op_comp.cpp, and
 * nothing here repeats that. What is asserted here is the WIRING: that a rule
 * file can reach them, that each piece becomes a child shape with the address,
 * the seed and the caps every other child gets, and that the body attached to a
 * piece runs in the frame that wrote it.
 *
 * Those are exactly the four things the brief for this feature said would be
 * wrong if nobody watched them, so each has its own section below:
 * determinism, the caps, lexical scope, and abandon-and-discard. A fifth
 * section was added after the first review round: the diagnostics `select`
 * makes ITSELF. exec_select() does not call op_comp.cpp's emit_components() --
 * it cannot, because a `select` arm has a body to run and emit_components()
 * emits terminals -- so every test of that function in test_op_comp.cpp is a
 * test of code the interpreter never reaches, and the reporting `select`
 * actually does had no test at all.
 *
 * ================================================================================
 * HOW THESE TESTS ARE WRITTEN SO THAT THEY CAN FAIL
 * ================================================================================
 *
 *   - **Seeds are recomputed from the published mix, not read back.** A child's
 *     key must be `seed_mix2(parent, seed_mix2(kSaltChild, index))`, and the
 *     tests build that expression from shape.hpp's own helpers and compare. An
 *     implementation that drew from a shared stream would still be internally
 *     consistent and would still dump identically twice in a row; it fails here
 *     on the first assertion.
 *
 *   - **"Independent of what came before" is tested with two different files.**
 *     One generates thousands of unrelated shapes before the split, the other
 *     generates none, and the split's children must carry the same keys in both.
 *     Running the same file twice cannot catch a shared stream. The test also
 *     asserts that the two runs really did differ in size, so it cannot pass by
 *     the "before" part having quietly done nothing.
 *
 *   - **Every cap test asserts truncated OUTPUT, not merely that a cap fired.**
 *     D2's argument is that a cap which produces nothing is worse than one that
 *     truncates, so each cap test asserts a non-zero terminal count as well as
 *     the flag, and asserts a different cap VALUE gives a different count.
 *
 *   - **The lexical-scope tests assert both directions.** A body must SEE the
 *     enclosing rule's bindings, and a `let` inside a body must NOT be visible
 *     to the next part or after the statement. One without the other passes for
 *     an implementation that runs the body in a fresh empty frame, and for one
 *     that runs it in the caller's and never pops.
 *
 *   - **Geometry is checked by world position and by area**, never by the scope
 *     alone. A slab whose scope says 0.6 by 2.0 and whose vertices are still the
 *     parent's passes every scope assertion and no area assertion.
 *
 *   - **One end-to-end case generates a facade** and asserts twelve window
 *     centres computed by hand from the rule text. It is the only assertion in
 *     this file that exercises every piece at once, and it is the one that says
 *     the feature is finished.
 *
 *   - **A message is matched against the sentence the code really writes, and
 *     counted exactly.** Two tests here failed review for breaking that rule, and
 *     both failed the same way: the first counted diagnostics containing
 *     "overflow", a word solve_split() has never emitted, so the count was
 *     always zero and `count <= 1` held whatever the interpreter did. Quote the
 *     sentence from the source that produces it, and assert `CHECK_EQ(n, 1)`
 *     rather than `CHECK_TRUE(n <= 1)` -- an upper bound passes when the message
 *     disappears altogether, which is half of what "reported once" means.
 *
 *   - **A `select` test states which arm runs FIRST, and does not assume it is
 *     the one written first.** Components are visited in op_comp.hpp's canonical
 *     order, which on a box puts `bottom` before `top` however the arms are
 *     written. The other test that failed review read a name in the `bottom` arm
 *     that the `top` arm bound, and got "unknown name" because the name had not
 *     been bound yet rather than because the binding had been popped. Where the
 *     order decides what is being tested, both assignments are run, so one of
 *     the two is always the interesting one whatever the emission order is.
 *
 * Nothing here needs a GPU, a window or a file on disk, so nothing here skips.
 */

#include "framework.hpp"

#include "procgen/rules/ast.hpp"
#include "procgen/rules/interpreter.hpp"
#include "procgen/rules/lexer.hpp"
#include "procgen/rules/parser.hpp"
#include "procgen/rules/shape.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

using stratum::procgen::rules::Diagnostic;
using stratum::procgen::rules::extrude_shape;
using stratum::procgen::rules::face_area;
using stratum::procgen::rules::GenerationOptions;
using stratum::procgen::rules::GenerationResult;
using stratum::procgen::rules::generate;
using stratum::procgen::rules::geometry_area;
using stratum::procgen::rules::geometry_volume;
using stratum::procgen::rules::kSaltChild;
using stratum::procgen::rules::kSaltComponent;
using stratum::procgen::rules::kSaltRoot;
using stratum::procgen::rules::OpResult;
using stratum::procgen::rules::parse;
using stratum::procgen::rules::ParseResult;
using stratum::procgen::rules::ScopeAxis;
using stratum::procgen::rules::seed_mix2;
using stratum::procgen::rules::Severity;
using stratum::procgen::rules::Shape;
using stratum::procgen::rules::shape_from_rect;
using stratum::procgen::rules::Value;

namespace {

// ============================================================================
// Helpers
// ============================================================================

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

/// Is @p needle a substring of a diagnostic of @p severity?
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

/**
 * @brief The average of a shape's vertices, in WORLD space
 *
 * Through scope.to_world() rather than from scope.origin and scope.size, because
 * a slab whose scope was refitted but whose vertices were never cut has a scope
 * of exactly the right size in exactly the right place and geometry that is
 * still the whole parent. That fault is invisible to a scope assertion.
 */
[[nodiscard]] glm::dvec3 world_centroid(const Shape& shape) {
    if (shape.geometry.positions.empty()) {
        return glm::dvec3{1.0e300};  // must fail any tolerance, never pass one
    }
    glm::dvec3 total{0.0};
    for (const glm::dvec3& local : shape.geometry.positions) {
        total += shape.scope.to_world(local);
    }
    return total / static_cast<double>(shape.geometry.positions.size());
}

/// World-space bounds of a shape's vertices. See world_centroid() for why world.
void world_bounds(const Shape& shape, glm::dvec3& min_out, glm::dvec3& max_out) {
    min_out = glm::dvec3{1.0e300};
    max_out = glm::dvec3{-1.0e300};
    for (const glm::dvec3& local : shape.geometry.positions) {
        const glm::dvec3 world = shape.scope.to_world(local);
        min_out = glm::min(min_out, world);
        max_out = glm::max(max_out, world);
    }
}

void check_vec3(const glm::dvec3& actual, const glm::dvec3& expected, double eps) {
    CHECK_NEAR(actual.x, expected.x, eps);
    CHECK_NEAR(actual.y, expected.y, eps);
    CHECK_NEAR(actual.z, expected.z, eps);
}

/// The key the root shape gets for a run seed of @p seed over an input key of 0
[[nodiscard]] uint64_t root_key(uint64_t seed) {
    return seed_mix2(seed_mix2(static_cast<uint64_t>(kSaltRoot), seed), uint64_t{0});
}

/// The key a child at @p index under @p parent_key must have. This is the whole
/// determinism contract, written out rather than read back from the output.
[[nodiscard]] uint64_t child_key(uint64_t parent_key, uint32_t index) {
    return seed_mix2(parent_key, seed_mix2(static_cast<uint64_t>(kSaltChild),
                                           static_cast<uint64_t>(index)));
}

/// The key a `select` child gets: addressed by WHICH COMPONENT it is, under its
/// own salt, so it cannot collide with a sibling-addressed child of the same
/// parent at the same index.
[[nodiscard]] uint64_t component_key(uint64_t parent_key, uint32_t component_index) {
    return seed_mix2(parent_key, seed_mix2(static_cast<uint64_t>(kSaltComponent),
                                           static_cast<uint64_t>(component_index)));
}

/// A 2 x 3 x 4 box: the rectangle 2 by 4 in the xz plane, extruded 3 up y
[[nodiscard]] Shape make_box() {
    Shape shape = shape_from_rect(2.0, 4.0);
    const OpResult result = extrude_shape(shape, ScopeAxis::Y, 3.0);
    CHECK_TRUE(result.ok);
    return shape;
}

// ============================================================================
// split: the pieces become shapes
// ============================================================================

TEST(RuleStatements, split_cuts_the_shape_into_one_child_per_solved_piece) {
    // A 2 x 3 x 4 box cut across y into 1, 1 and 1. The solver's arithmetic is
    // tested elsewhere; what is asserted here is that three CHILDREN came out,
    // each carrying the geometry of its own slab.
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main {\n"
        "    split(y) {\n"
        "        1.0 : { Slab(); }\n"
        "        1.0 : { Slab(); }\n"
        "        1.0 : { Slab(); }\n"
        "    }\n"
        "}\n"
        "rule Slab { }\n",
        make_box());

    CHECK_TRUE(result.ok());
    const std::vector<const Shape*> slabs = terminals_named(result, "Slab");
    CHECK_EQ(slabs.size(), size_t{3});
    if (slabs.size() != 3) {
        return;
    }

    // Each slab is a closed solid of 2 x 1 x 4 = 8, stacked up y. Volume, not
    // scope: a cut that clipped the walls and forgot the caps leaves the scope
    // right and the volume meaningless.
    double total = 0.0;
    for (size_t i = 0; i < slabs.size(); ++i) {
        CHECK_NEAR(geometry_volume(slabs[i]->geometry), 8.0, 1e-9);
        glm::dvec3 min_corner{0.0};
        glm::dvec3 max_corner{0.0};
        world_bounds(*slabs[i], min_corner, max_corner);
        check_vec3(min_corner, glm::dvec3{0.0, static_cast<double>(i), 0.0}, 1e-9);
        check_vec3(max_corner, glm::dvec3{2.0, static_cast<double>(i) + 1.0, 4.0}, 1e-9);
        total += geometry_volume(slabs[i]->geometry);
    }
    // The slabs tile the box: nothing was lost at a seam and nothing counted twice.
    CHECK_NEAR(total, 24.0, 1e-9);
}

TEST(RuleStatements, a_split_makes_children_so_the_shape_it_split_is_not_a_terminal) {
    // The point of a child is that the parent stops being the output. A split
    // that produced the slabs but left the parent a terminal would draw the
    // whole box on top of its own floors, and every volume assertion above would
    // still pass.
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main { split(y) { 1.0 : { } ~1.0 : { } } }\n",
        make_box());

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.terminals.size(), size_t{2});
    CHECK_EQ(result.stats.shapes_created, uint32_t{3});
    for (const Shape& terminal : result.terminals) {
        CHECK_EQ(terminal.depth, uint32_t{1});
        CHECK_TRUE(geometry_volume(terminal.geometry) < 24.0);
    }
}

TEST(RuleStatements, a_split_size_is_measured_against_the_shape_being_split) {
    // `shape.sy` inside a size must mean the height being divided, not the height
    // of a piece of it. An implementation that evaluated the sizes against a slab
    // would loop or would produce halves of halves.
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main {\n"
        "    split(y) {\n"
        "        shape.sy / 4 : { Low(); }\n"
        "        ~1.0 : { High(); }\n"
        "    }\n"
        "}\n"
        "rule Low { }\n"
        "rule High { }\n",
        make_box());

    CHECK_TRUE(result.ok());
    const std::vector<const Shape*> low = terminals_named(result, "Low");
    const std::vector<const Shape*> high = terminals_named(result, "High");
    CHECK_EQ(low.size(), size_t{1});
    CHECK_EQ(high.size(), size_t{1});
    if (low.size() != 1 || high.size() != 1) {
        return;
    }
    // 3 / 4 = 0.75, and the floating part takes the remaining 2.25.
    CHECK_NEAR(low[0]->scope.size.y, 0.75, 1e-9);
    CHECK_NEAR(high[0]->scope.size.y, 2.25, 1e-9);
    CHECK_NEAR(geometry_volume(low[0]->geometry), 6.0, 1e-9);
    CHECK_NEAR(geometry_volume(high[0]->geometry), 18.0, 1e-9);
}

TEST(RuleStatements, a_relative_size_and_a_repeat_group_both_reach_the_interpreter) {
    // `25%` and `repeat` are solver features, and this asserts only that the
    // statement carries them through: a wiring that dropped the repeat's children
    // or read `25` as 25 metres would still produce SOME slabs.
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main {\n"
        "    split(x) {\n"
        "        25% : { Pier(); }\n"
        "        repeat { 0.25 : { Bay(); } }\n"
        "    }\n"
        "}\n"
        "rule Pier { }\n"
        "rule Bay { }\n",
        make_box());

    CHECK_TRUE(result.ok());
    // Extent 2. The pier takes 25% of 2 = 0.5. The group is elastic and takes the
    // remaining 1.5; its period is 0.25, so floor(1.5 / 0.25) = 6 copies of 0.25.
    const std::vector<const Shape*> piers = terminals_named(result, "Pier");
    const std::vector<const Shape*> bays = terminals_named(result, "Bay");
    CHECK_EQ(piers.size(), size_t{1});
    CHECK_EQ(bays.size(), size_t{6});
    if (piers.size() != 1 || bays.size() != 6) {
        return;
    }
    CHECK_NEAR(piers[0]->scope.size.x, 0.5, 1e-9);
    for (size_t i = 0; i < bays.size(); ++i) {
        CHECK_NEAR(bays[i]->scope.size.x, 0.25, 1e-9);
        CHECK_NEAR(bays[i]->scope.origin.x, 0.5 + 0.25 * static_cast<double>(i), 1e-9);
    }
}

// ============================================================================
// split: determinism
// ============================================================================

TEST(RuleStatements, each_split_child_takes_its_key_from_its_parent_and_its_own_index) {
    // The published mix, recomputed here rather than read back. A child seeded
    // from a shared stream would still be internally consistent, would still
    // dump identically on a re-run, and fails on the first line below.
    GenerationOptions options;
    options.seed = 12345;
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main { split(y) { 1.0 : { } 1.0 : { } 1.0 : { } } }\n",
        make_box(), options);

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.terminals.size(), size_t{3});
    if (result.terminals.size() != 3) {
        return;
    }
    const uint64_t root = root_key(options.seed);
    for (uint32_t i = 0; i < 3; ++i) {
        CHECK_EQ(result.terminals[i].index, i);
        CHECK_EQ(result.terminals[i].seed_key, child_key(root, i));
    }
    // And the three differ from each other, so an implementation that handed
    // every child the parent's key cannot pass the line above by accident.
    CHECK_TRUE(result.terminals[0].seed_key != result.terminals[1].seed_key);
    CHECK_TRUE(result.terminals[1].seed_key != result.terminals[2].seed_key);
}

TEST(RuleStatements, a_split_child_key_does_not_depend_on_what_was_generated_before_it) {
    // Two DIFFERENT files, not one file run twice. Running one file twice cannot
    // catch a shared stream; generating thousands of unrelated shapes before the
    // split and demanding the same keys can.
    const std::string busy =
        "@start\n"
        "rule Main { Noise(); Facade(); }\n"
        "rule Noise { split(x) { repeat { 0.01 : { } } } }\n"
        "rule Facade {\n"
        "    split(x) { 0.25 : { Panel(); } 0.25 : { Panel(); } ~1.0 : { Panel(); } }\n"
        "}\n"
        "rule Panel { }\n";
    const std::string quiet =
        "@start\n"
        "rule Main { Noise(); Facade(); }\n"
        "rule Noise { }\n"
        "rule Facade {\n"
        "    split(x) { 0.25 : { Panel(); } 0.25 : { Panel(); } ~1.0 : { Panel(); } }\n"
        "}\n"
        "rule Panel { }\n";

    const GenerationResult a = run_square(busy);
    const GenerationResult b = run_square(quiet);
    CHECK_TRUE(a.ok());
    CHECK_TRUE(b.ok());

    // The "before" really did differ, so this cannot pass by both runs being the
    // same size.
    CHECK_TRUE(a.terminals.size() > b.terminals.size() + 50);

    const std::vector<const Shape*> a_panels = terminals_named(a, "Panel");
    const std::vector<const Shape*> b_panels = terminals_named(b, "Panel");
    CHECK_EQ(a_panels.size(), size_t{3});
    CHECK_EQ(b_panels.size(), size_t{3});
    if (a_panels.size() != 3 || b_panels.size() != 3) {
        return;
    }
    // Facade is Main's child index 1 in both files, so its slabs and the panels
    // beneath them must carry identical keys.
    const uint64_t facade = child_key(root_key(0), 1);
    for (uint32_t i = 0; i < 3; ++i) {
        const uint64_t expected = child_key(child_key(facade, i), 0);
        CHECK_EQ(a_panels[i]->seed_key, expected);
        CHECK_EQ(b_panels[i]->seed_key, expected);
    }
}

TEST(RuleStatements, the_same_file_and_seed_generate_the_same_split_geometry_twice) {
    const std::string source =
        "@start\n"
        "rule Main {\n"
        "    split(x) { repeat { 0.3 : { Bay(); } } }\n"
        "}\n"
        "rule Bay { split(y) { 0.5 : { } ~1.0 : { } } }\n";

    GenerationOptions options;
    options.seed = 99;
    const GenerationResult a = run_on(source, make_box(), options);
    const GenerationResult b = run_on(source, make_box(), options);

    CHECK_TRUE(a.ok());
    // Substantial, so two empty dumps cannot pass by agreeing, and sensitive to
    // the seed, so a dump that stopped printing anything cannot pass either.
    CHECK_TRUE(a.dump().size() > 1000);
    CHECK_EQ(a.dump(), b.dump());

    GenerationOptions other;
    other.seed = 100;
    const GenerationResult c = run_on(source, make_box(), other);
    // The geometry is seed-independent here, but the KEYS are not, and the dump
    // has to stay equal while the keys move -- so the keys are compared directly.
    CHECK_TRUE(a.terminals.size() == c.terminals.size());
    if (!a.terminals.empty()) {
        CHECK_TRUE(a.terminals[0].seed_key != c.terminals[0].seed_key);
    }
}

// ============================================================================
// split: lexical scope
// ============================================================================

TEST(RuleStatements, a_split_part_body_reads_the_enclosing_rule_bindings) {
    // A body runs in the frame that wrote it. Without this,
    // `split(y) { storey : { Floor(storey); } }` -- the ordinary way to write a
    // facade -- cannot be written at all.
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main { Facade(0.5); }\n"
        "rule Facade(pier: float) {\n"
        "    let rest = 2.0 - pier;\n"
        "    split(x) {\n"
        "        pier : { Pier(pier); }\n"
        "        rest : { Bay(rest); }\n"
        "    }\n"
        "}\n"
        "rule Pier(w: float) { }\n"
        "rule Bay(w: float) { }\n",
        make_box());

    CHECK_TRUE(result.ok());
    const std::vector<const Shape*> piers = terminals_named(result, "Pier");
    const std::vector<const Shape*> bays = terminals_named(result, "Bay");
    CHECK_EQ(piers.size(), size_t{1});
    CHECK_EQ(bays.size(), size_t{1});
    if (piers.size() != 1 || bays.size() != 1) {
        return;
    }
    // Both the SIZE expression and the CALL inside the body saw the parameter.
    CHECK_NEAR(piers[0]->scope.size.x, 0.5, 1e-9);
    CHECK_NEAR(bays[0]->scope.size.x, 1.5, 1e-9);
}

TEST(RuleStatements, a_let_inside_a_split_part_is_not_visible_after_that_part) {
    // The other direction, and the one that puts dynamic scoping back if it is
    // missed: a body that never popped its bindings would leak `here` into the
    // next part and into the rest of the rule.
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main {\n"
        "    split(x) {\n"
        "        1.0 : { let here = 5.0; Kept(here); }\n"
        "        1.0 : { Leaked(here); }\n"
        "    }\n"
        "}\n"
        "rule Kept(v: float) { }\n"
        "rule Leaked(v: float) { }\n",
        make_box());

    CHECK_FALSE(result.ok());
    CHECK_TRUE(has_message(result, Severity::Error, "unknown name 'here'"));
    // The FIRST part still produced its geometry: only the part that read the
    // leaked name was abandoned.
    CHECK_EQ(terminals_named(result, "Kept").size(), size_t{1});
    CHECK_EQ(terminals_named(result, "Leaked").size(), size_t{0});
}

TEST(RuleStatements, an_abandoned_split_part_takes_its_bindings_with_it) {
    // The leak that survives the ordinary scope test. A body's own BlockStmt pops
    // its `let`s when it ENDS, so a body that runs to completion cannot leak one.
    // A body that is abandoned part way never reaches the end of its block, and
    // the bindings it had pushed would still be on the stack for the next part to
    // find -- lexical scope quietly turning dynamic on the error path only.
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main {\n"
        "    split(y) {\n"
        "        1.0 : { let stray = 5.0; extrude(0.0); Never(); }\n"
        "        1.0 : { Leaked(stray); }\n"
        "    }\n"
        "}\n"
        "rule Never { }\n"
        "rule Leaked(v: float) { }\n",
        make_box());

    CHECK_FALSE(result.ok());
    // The first part died of the extrude, and the second of not knowing `stray`.
    CHECK_TRUE(has_message(result, Severity::Error, "unknown name 'stray'"));
    CHECK_EQ(terminals_named(result, "Never").size(), size_t{0});
    CHECK_EQ(terminals_named(result, "Leaked").size(), size_t{0});
}

TEST(RuleStatements, a_split_part_body_cannot_read_the_bindings_of_the_calling_rule) {
    // Lexical scope survives a statement body. `Facade` is called from `Main`,
    // and a body inside `Facade` must not see `Main`'s `w` any more than
    // `Facade`'s own statements may.
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main { let w = 1.0; Facade(); }\n"
        "rule Facade { split(x) { 1.0 : { Panel(w); } ~1.0 : { } } }\n"
        "rule Panel(v: float) { }\n",
        make_box());

    CHECK_FALSE(result.ok());
    CHECK_TRUE(has_message(result, Severity::Error, "unknown name 'w'"));
    CHECK_EQ(terminals_named(result, "Panel").size(), size_t{0});
}

// ============================================================================
// split: abandon and discard
// ============================================================================

TEST(RuleStatements, a_discard_in_one_split_part_costs_only_that_part) {
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main {\n"
        "    split(y) {\n"
        "        1.0 : { Keep(); }\n"
        "        1.0 : { discard; }\n"
        "        1.0 : { Keep(); }\n"
        "    }\n"
        "}\n"
        "rule Keep { }\n",
        make_box());

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.terminals.size(), size_t{2});
    CHECK_EQ(terminals_named(result, "Keep").size(), size_t{2});
    // The discarded slab was still CREATED -- it took its address and its index --
    // so the third part is child 2 and not child 1.
    const std::vector<const Shape*> kept = terminals_named(result, "Keep");
    if (kept.size() == 2) {
        CHECK_NEAR(kept[0]->scope.origin.y, 0.0, 1e-9);
        CHECK_NEAR(kept[1]->scope.origin.y, 2.0, 1e-9);
    }
}

TEST(RuleStatements, a_runtime_fault_in_one_split_part_costs_only_that_part) {
    // `extrude` of zero is a fault the shape cannot survive, so the handler
    // abandons it. The siblings must still run: one malformed slab must not cost
    // the other floors of the facade.
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main {\n"
        "    split(y) {\n"
        "        1.0 : { Good(); }\n"
        "        1.0 : { extrude(0.0); Bad(); }\n"
        "        1.0 : { Good(); }\n"
        "    }\n"
        "}\n"
        "rule Good { }\n"
        "rule Bad { }\n",
        make_box());

    CHECK_FALSE(result.ok());
    CHECK_EQ(terminals_named(result, "Good").size(), size_t{2});
    CHECK_EQ(terminals_named(result, "Bad").size(), size_t{0});
}

TEST(RuleStatements, a_split_size_that_is_not_a_number_abandons_the_shape_not_one_slab) {
    // The layout cannot be solved at all when one row of it is unreadable, so
    // there is no truncated facade to fall back to and the shape goes. The rule
    // that CALLED it keeps its other children, which is what the second
    // assertion pins.
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main { Bad(); Other(); }\n"
        "rule Bad { split(y) { \"tall\" : { Never(); } ~1.0 : { Never(); } } }\n"
        "rule Other { }\n"
        "rule Never { }\n",
        make_box());

    CHECK_FALSE(result.ok());
    CHECK_TRUE(has_message(result, Severity::Error, "a split size must be a number"));
    CHECK_EQ(terminals_named(result, "Never").size(), size_t{0});
    CHECK_EQ(terminals_named(result, "Other").size(), size_t{1});
}

TEST(RuleStatements, an_overflowing_split_reports_once_per_site_and_still_produces_slabs) {
    // A split inside a recursive rule would otherwise report the same overflow at
    // every storey and fill the diagnostic cap with one mistake.
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main { split(x) { repeat { 0.5 : { Bay(); } } } }\n"
        "rule Bay { split(y) { 5.0 : { Tall(); } 5.0 : { Tall(); } } }\n"
        "rule Tall { }\n",
        make_box());

    // Four bays, each splitting a 3-high slab into two 5s: the same overflow four
    // times over, and exactly one diagnostic.
    CHECK_EQ(terminals_named(result, "Bay").size(), size_t{0});
    CHECK_EQ(terminals_named(result, "Tall").size(), size_t{8});

    // Counted against the sentence solve_split() actually writes, quoted from
    // op_split.cpp. An earlier version of this test counted "overflow" and
    // "beyond the extent", neither of which the solver has ever emitted, so the
    // count was always zero and the assertion held whatever the interpreter did.
    // The exact count, not an upper bound: `<= 1` passes when the report is
    // missing altogether, which is the other half of what once-per-site means.
    size_t overflow_messages = 0;
    for (const Diagnostic& diagnostic : result.diagnostics) {
        if (diagnostic.message.find("the fixed parts of this split ask for") !=
            std::string::npos) {
            ++overflow_messages;
        }
    }
    CHECK_EQ(overflow_messages, size_t{1});
    // The one message is a Warning: truncated slabs are still slabs, and the run
    // is usable. Only the caps make a truncation an Error.
    CHECK_TRUE(has_message(result, Severity::Warning, "so the surplus is cut off at the far end"));
}

// ============================================================================
// split: the caps
// ============================================================================

TEST(RuleStatements, the_depth_cap_applies_to_split_children_and_truncates_rather_than_empties) {
    // A rule that splits and recurses runs away exactly as readily as one that
    // only recurses. The cap has to see the slabs, and -- D2's argument -- has to
    // leave output behind when it fires.
    const std::string source =
        "@start\n"
        "rule Main { Tower(); }\n"
        "rule Tower { split(y) { ~1.0 : { Tower(); } } }\n";

    GenerationOptions shallow;
    shallow.limits.max_depth = 4;
    const GenerationResult a = run_on(source, make_box(), shallow);

    GenerationOptions deeper;
    deeper.limits.max_depth = 8;
    const GenerationResult b = run_on(source, make_box(), deeper);

    CHECK_TRUE(a.stats.depth_limit_hit);
    CHECK_TRUE(b.stats.depth_limit_hit);
    CHECK_FALSE(a.ok());
    // Truncated, not empty: the shape at the cap is emitted as it stood.
    CHECK_EQ(a.terminals.size(), size_t{1});
    CHECK_EQ(b.terminals.size(), size_t{1});
    // The cap's VALUE, not merely that a cap fired: a hard-coded constant fails.
    CHECK_EQ(a.stats.max_depth_reached, uint32_t{4});
    CHECK_EQ(b.stats.max_depth_reached, uint32_t{8});
    CHECK_TRUE(a.stats.shapes_created < b.stats.shapes_created);
    if (!a.terminals.empty() && !b.terminals.empty()) {
        CHECK_EQ(a.terminals[0].depth, uint32_t{5});
        CHECK_EQ(b.terminals[0].depth, uint32_t{9});
    }
}

TEST(RuleStatements, the_shape_cap_applies_to_split_children_and_truncates_rather_than_empties) {
    const std::string source =
        "@start\n"
        "rule Main { split(x) { repeat { 0.05 : { Bay(); } } } }\n"
        "rule Bay { }\n";

    GenerationOptions tight;
    tight.limits.max_shapes = 6;
    const GenerationResult a = run_on(source, make_box(), tight);

    GenerationOptions looser;
    looser.limits.max_shapes = 12;
    const GenerationResult b = run_on(source, make_box(), looser);

    CHECK_TRUE(a.stats.shape_limit_hit);
    CHECK_FALSE(a.ok());
    CHECK_EQ(a.stats.shapes_created, uint32_t{6});
    CHECK_EQ(b.stats.shapes_created, uint32_t{12});
    // The cap's value again, and output on both sides of it.
    CHECK_TRUE(a.terminals.size() < b.terminals.size());
    CHECK_TRUE(!a.terminals.empty());
    CHECK_TRUE(has_message(a, Severity::Error, "shape limit of 6"));
}

// ============================================================================
// select
// ============================================================================

TEST(RuleStatements, select_runs_each_arm_against_the_components_that_answer_to_it) {
    // A 2 x 3 x 4 box has one top, one bottom and four sides.
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main {\n"
        "    select face {\n"
        "        top : { Roof(); }\n"
        "        side : { Wall(); }\n"
        "    }\n"
        "}\n"
        "rule Roof { }\n"
        "rule Wall { }\n",
        make_box());

    CHECK_TRUE(result.ok());
    const std::vector<const Shape*> roofs = terminals_named(result, "Roof");
    const std::vector<const Shape*> walls = terminals_named(result, "Wall");
    CHECK_EQ(roofs.size(), size_t{1});
    CHECK_EQ(walls.size(), size_t{4});
    // The bottom answers to neither word, so it makes no child and no geometry.
    CHECK_EQ(result.terminals.size(), size_t{5});

    if (roofs.size() == 1) {
        // The roof is the 2 by 4 face at y = 3, and it is a FACE: area 8, no volume.
        CHECK_NEAR(geometry_area(roofs[0]->geometry), 8.0, 1e-9);
        check_vec3(world_centroid(*roofs[0]), glm::dvec3{1.0, 3.0, 2.0}, 1e-9);
    }
    // The four walls are 2 x 3 twice and 4 x 3 twice: 6 + 6 + 12 + 12 = 36.
    double wall_area = 0.0;
    for (const Shape* wall : walls) {
        wall_area += geometry_area(wall->geometry);
    }
    CHECK_NEAR(wall_area, 36.0, 1e-9);
}

TEST(RuleStatements, a_component_no_arm_claims_makes_no_child_at_all) {
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main { select face { top : { Roof(); } } }\n"
        "rule Roof { }\n",
        make_box());

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.terminals.size(), size_t{1});
    // Six faces were found, and exactly one became a shape. A wiring that made a
    // child per face and ran nothing against it would create seven shapes and
    // emit six of them.
    CHECK_EQ(result.stats.shapes_created, uint32_t{3});
    CHECK_EQ(terminals_named(result, "Roof").size(), size_t{1});
}

TEST(RuleStatements, select_numbers_its_children_by_component_and_not_by_arm) {
    // The same two arms, written in the two orders. The components are decomposed
    // once, in op_comp.hpp's canonical order, so the children get the same
    // indices and the same keys either way. An implementation that looped over
    // the ARMS and emitted each arm's components together would renumber every
    // child when the arms were swapped, and every seed with it.
    const std::string top_first =
        "@start\n"
        "rule Main { select face { top : { Roof(); } side : { Wall(); } } }\n"
        "rule Roof { }\n"
        "rule Wall { }\n";
    const std::string side_first =
        "@start\n"
        "rule Main { select face { side : { Wall(); } top : { Roof(); } } }\n"
        "rule Roof { }\n"
        "rule Wall { }\n";

    const GenerationResult a = run_on(top_first, make_box());
    const GenerationResult b = run_on(side_first, make_box());
    CHECK_TRUE(a.ok());
    CHECK_TRUE(b.ok());
    CHECK_EQ(a.terminals.size(), size_t{5});
    CHECK_EQ(a.terminals.size(), b.terminals.size());
    CHECK_TRUE(a.dump().size() > 500);
    CHECK_EQ(a.dump(), b.dump());

    for (size_t i = 0; i < a.terminals.size() && i < b.terminals.size(); ++i) {
        CHECK_EQ(a.terminals[i].seed_key, b.terminals[i].seed_key);
        CHECK_EQ(a.terminals[i].index, b.terminals[i].index);
    }
}

TEST(RuleStatements, a_select_component_takes_its_key_from_which_component_it_is) {
    GenerationOptions options;
    options.seed = 7;
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main { select face { all : { Panel(); } } }\n"
        "rule Panel { }\n",
        make_box(), options);

    CHECK_TRUE(result.ok());
    const std::vector<const Shape*> panels = terminals_named(result, "Panel");
    CHECK_EQ(panels.size(), size_t{6});
    const uint64_t root = root_key(options.seed);
    for (uint32_t i = 0; i < panels.size(); ++i) {
        // Panel is child 0 of the component shape, and the component shape is
        // addressed by BEING component i of the root -- not by being the i-th
        // child the root happened to make. Under the component salt, so it
        // cannot collide with a rule-call child at the same index.
        CHECK_EQ(panels[i]->seed_key, child_key(component_key(root, i), 0));
    }
}

TEST(RuleStatements, a_rule_call_and_a_component_at_the_same_index_get_different_keys) {
    // The collision the separate salt exists to prevent. `A()` is sibling 0 and
    // the first component is component 0; sharing one salt would give them the
    // same address and therefore the same random draw.
    GenerationOptions options;
    options.seed = 11;
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main { A(); select face { all : { P(); } } }\n"
        "rule A { }\n"
        "rule P { }\n",
        make_box(), options);

    CHECK_TRUE(result.ok());
    const std::vector<const Shape*> a = terminals_named(result, "A");
    const std::vector<const Shape*> p = terminals_named(result, "P");
    CHECK_EQ(a.size(), size_t{1});
    CHECK_EQ(p.size(), size_t{6});
    if (a.size() != 1 || p.empty()) return;

    const uint64_t root = root_key(options.seed);
    CHECK_EQ(a[0]->seed_key, child_key(root, 0));
    CHECK_EQ(p[0]->seed_key, child_key(component_key(root, 0), 0));
    CHECK_TRUE(a[0]->seed_key != p[0]->seed_key);
}

TEST(RuleStatements, a_select_arm_body_reads_the_enclosing_rule_bindings_and_leaks_none_back) {
    // Written as two runs, and the reason is the whole point of the test. The
    // arms are visited in op_comp.hpp's canonical COMPONENT order and not in
    // source order, and on a box that order puts `bottom` before `top`. So a
    // single file with the binding arm written first proves nothing: whichever
    // arm the file lists first, one of the two orders reports "unknown name"
    // because the name was never bound at all rather than because a binding was
    // popped, and that reading passes for an implementation that never pops.
    //
    // Running BOTH assignments means one of the two runs is always the one where
    // the binding arm ran first, whatever the emission order turns out to be. An
    // implementation that left a body's `let` on the stack passes one run and
    // fails the other; there is no emission order in which both can pass.
    const std::string bottom_binds =
        "@start\n"
        "rule Main { Shell(0.25); }\n"
        "rule Shell(t: float) {\n"
        "    select face {\n"
        "        bottom : { let local = 9.0; Floor(t); }\n"
        "        top : { Roof(local); }\n"
        "    }\n"
        "}\n"
        "rule Roof(v: float) { }\n"
        "rule Floor(v: float) { }\n";
    const std::string top_binds =
        "@start\n"
        "rule Main { Shell(0.25); }\n"
        "rule Shell(t: float) {\n"
        "    select face {\n"
        "        top : { let local = 9.0; Roof(t); }\n"
        "        bottom : { Floor(local); }\n"
        "    }\n"
        "}\n"
        "rule Roof(v: float) { }\n"
        "rule Floor(v: float) { }\n";

    const GenerationResult a = run_on(bottom_binds, make_box());
    const GenerationResult b = run_on(top_binds, make_box());

    // The arm that binds ran and read `t`, which is the enclosing rule's
    // parameter: the binding arm's rule call succeeded in both runs.
    CHECK_EQ(terminals_named(a, "Floor").size(), size_t{1});
    CHECK_EQ(terminals_named(b, "Roof").size(), size_t{1});

    // The other arm could not see `local`, whichever of the two ran second.
    CHECK_FALSE(a.ok());
    CHECK_FALSE(b.ok());
    CHECK_TRUE(has_message(a, Severity::Error, "unknown name 'local'"));
    CHECK_TRUE(has_message(b, Severity::Error, "unknown name 'local'"));
    CHECK_EQ(terminals_named(a, "Roof").size(), size_t{0});
    CHECK_EQ(terminals_named(b, "Floor").size(), size_t{0});

    // Neither run lost the arm that worked: one malformed arm costs its own
    // component and nothing else.
    CHECK_EQ(a.terminals.size(), size_t{1});
    CHECK_EQ(b.terminals.size(), size_t{1});
}

TEST(RuleStatements, a_let_inside_a_select_arm_is_not_visible_after_the_select) {
    // The other direction a body could leak: forward out of the whole statement
    // rather than sideways into the next arm. `run_statement_body()` resizes the
    // locals stack on both the normal and the abandoned path, and this reaches
    // the normal one.
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main {\n"
        "    select face { all : { let local = 9.0; } }\n"
        "    After(local);\n"
        "}\n"
        "rule After(v: float) { }\n",
        make_box());

    CHECK_FALSE(result.ok());
    CHECK_TRUE(has_message(result, Severity::Error, "unknown name 'local'"));
    CHECK_EQ(terminals_named(result, "After").size(), size_t{0});
    // The six components still became shapes and still emitted: the fault after
    // the select costs the shape that hit it, not the children already made.
    CHECK_EQ(result.terminals.size(), size_t{6});
}

TEST(RuleStatements, a_component_goes_to_the_first_arm_that_admits_it_and_not_the_last) {
    // The six direction words partition the components, so every `select` test
    // written with `top`/`side` or `top`/`bottom` passes under first-match and
    // under last-match alike. Only arms that OVERLAP tell the two apart, and
    // `all` overlaps everything. The same two arms in the two orders, and the
    // counts have to differ: that is what "resolved by source order" means.
    const std::string all_first =
        "@start\n"
        "rule Main { select face { all : { A(); } top : { T(); } } }\n"
        "rule A { }\n"
        "rule T { }\n";
    const std::string top_first =
        "@start\n"
        "rule Main { select face { top : { T(); } all : { A(); } } }\n"
        "rule A { }\n"
        "rule T { }\n";

    const GenerationResult a = run_on(all_first, make_box());
    const GenerationResult b = run_on(top_first, make_box());
    CHECK_TRUE(a.ok());
    CHECK_TRUE(b.ok());

    // `all` is written first, so it claims all six and `top` never sees one.
    CHECK_EQ(terminals_named(a, "A").size(), size_t{6});
    CHECK_EQ(terminals_named(a, "T").size(), size_t{0});
    // `top` is written first, so it takes the one top face and `all` takes the
    // other five. A last-match rule would give exactly the numbers above.
    CHECK_EQ(terminals_named(b, "A").size(), size_t{5});
    CHECK_EQ(terminals_named(b, "T").size(), size_t{1});

    // The face `top` took really is the top face, so the counts above are the
    // right five and one rather than any five and one.
    const std::vector<const Shape*> tops = terminals_named(b, "T");
    if (tops.size() == 1) {
        check_vec3(world_centroid(*tops[0]), glm::dvec3{1.0, 3.0, 2.0}, 1e-9);
    }
}

TEST(RuleStatements, adding_a_select_arm_does_not_re_key_the_children_of_the_other_arms) {
    // What exec_select()'s comment claims, pinned, including the half that is a
    // limitation rather than a promise.
    //
    // A child's sibling index is the number of components CLAIMED before it, not
    // its place in the canonical component order. So writing the same two arms in
    // the other order moves nothing -- the components are decomposed once and
    // visited in their own order -- but ADDING an arm that claims a component
    // which sorts earlier pushes every later child along by one and re-keys it.
    // The `front` face is the one being watched, and `bottom` sorts before it.
    const std::string front_only =
        "@start\n"
        "rule Main { select face { front : { P(); } } }\n"
        "rule P { }\n";
    const std::string front_then_bottom =
        "@start\n"
        "rule Main { select face { front : { P(); } bottom : { Q(); } } }\n"
        "rule P { }\n"
        "rule Q { }\n";
    const std::string bottom_then_front =
        "@start\n"
        "rule Main { select face { bottom : { Q(); } front : { P(); } } }\n"
        "rule P { }\n"
        "rule Q { }\n";

    const GenerationResult one = run_on(front_only, make_box());
    const GenerationResult two = run_on(front_then_bottom, make_box());
    const GenerationResult three = run_on(bottom_then_front, make_box());
    CHECK_TRUE(one.ok());
    CHECK_TRUE(two.ok());
    CHECK_TRUE(three.ok());

    const std::vector<const Shape*> p_one = terminals_named(one, "P");
    const std::vector<const Shape*> p_two = terminals_named(two, "P");
    const std::vector<const Shape*> p_three = terminals_named(three, "P");
    CHECK_EQ(p_one.size(), size_t{1});
    CHECK_EQ(p_two.size(), size_t{1});
    CHECK_EQ(p_three.size(), size_t{1});
    if (p_one.size() != 1 || p_two.size() != 1 || p_three.size() != 1) {
        return;
    }

    // Arm ORDER moves nothing. This is the property decomposing once buys.
    CHECK_EQ(p_two[0]->seed_key, p_three[0]->seed_key);
    CHECK_EQ(two.dump(), three.dump());

    // Adding the arm does NOT move it, and that is the whole point of
    // addressing a component child by which component it is.
    //
    // Under claim-count addressing this assertion was inverted: `bottom` claims
    // first, took index 0, and pushed `front` to 1, so adding an arm reshuffled
    // the random variation of every child of every OTHER arm. An author edits
    // one line and the rest of the building changes -- the same stable-identity
    // problem lots.cpp solves for lot ids, with the same answer.
    //
    // `front` is component 3 of make_box() in op_comp.hpp's canonical order, so
    // the key is written out rather than compared only for equality: a test
    // that just said "the two match" would also pass if both were wrong.
    const uint64_t root = root_key(0);
    const uint32_t front_component = 3;
    CHECK_EQ(p_one[0]->seed_key, child_key(component_key(root, front_component), 0));
    CHECK_EQ(p_two[0]->seed_key, child_key(component_key(root, front_component), 0));
    CHECK_EQ(p_one[0]->seed_key, p_two[0]->seed_key);

    // The geometry did not move with the key: it is the same face either way, so
    // what changed is the random address and nothing else.
    check_vec3(world_centroid(*p_one[0]), world_centroid(*p_two[0]), 1e-12);
}

// ============================================================================
// select: the diagnostics it makes itself
// ============================================================================

TEST(RuleStatements, a_bent_face_reaching_select_is_one_warning_however_many_shapes_reach_it) {
    // exec_select() does not call emit_components(); it reports the
    // ComponentSplitReport itself, at severities of its own. That reporting has
    // to be tested through `select`, because a test of emit_components() is a
    // test of a function the interpreter never calls.
    Shape bent;
    bent.geometry.positions = {glm::dvec3{0.0, 0.0, 0.0}, glm::dvec3{0.0, 0.0, 2.0},
                               glm::dvec3{2.0, 0.0, 2.0}, glm::dvec3{2.0, 0.25, 0.0}};
    stratum::procgen::rules::Face quad;
    quad.loop = {0, 1, 2, 3};
    bent.geometry.faces = {quad};
    stratum::procgen::rules::refit_scope(bent);

    // Two shapes reach the one `select` site, both carrying the bent face.
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main { A(); A(); }\n"
        "rule A { select face { all : { P(); } } }\n"
        "rule P { }\n",
        bent);

    // A Warning, so the run is still usable: the input geometry bent the face,
    // not the rule.
    CHECK_TRUE(result.ok());
    size_t bent_messages = 0;
    for (const Diagnostic& diagnostic : result.diagnostics) {
        if (diagnostic.message.find("do not lie flat") != std::string::npos) {
            CHECK_TRUE(diagnostic.severity == Severity::Warning);
            ++bent_messages;
        }
    }
    // Exactly one: twice would mean once per shape rather than once per site, and
    // none would mean the branch is not reached at all.
    CHECK_EQ(bent_messages, size_t{1});
    // The component was still produced. A diagnostic must not cost the output.
    CHECK_EQ(terminals_named(result, "P").size(), size_t{2});
}

TEST(RuleStatements, a_degenerate_face_reaching_select_is_a_warning_and_the_rest_still_run) {
    Shape footprint = shape_from_rect(2.0, 4.0);
    const auto base = static_cast<uint32_t>(footprint.geometry.positions.size());
    footprint.geometry.positions.push_back(glm::dvec3{0.0, 0.0, 0.0});
    footprint.geometry.positions.push_back(glm::dvec3{0.0, 0.0, 0.0});
    footprint.geometry.positions.push_back(glm::dvec3{0.0, 0.0, 0.0});
    stratum::procgen::rules::Face collapsed;
    collapsed.loop = {base, base + 1, base + 2};
    footprint.geometry.faces.push_back(collapsed);

    const GenerationResult result = run_on(
        "@start\n"
        "rule Main { select face { all : { P(); } } }\n"
        "rule P { }\n",
        footprint);

    // A Warning and not an Error, for the reason op_comp.hpp gives: a collapsed
    // face is something the input footprint did. An Error here would fail every
    // run over OSM-derived geometry.
    CHECK_TRUE(result.ok());
    CHECK_TRUE(has_message(result, Severity::Warning, "were dropped for having no area"));
    CHECK_FALSE(has_message(result, Severity::Error, "were dropped for having no area"));
    // The face that was fine still became a component.
    CHECK_EQ(terminals_named(result, "P").size(), size_t{1});
}

TEST(RuleStatements, an_unimplemented_domain_is_one_error_however_many_shapes_reach_it) {
    // The same once-per-site rule as the bent face, on the branch that IS an
    // Error. Four bays hit one `select vertex` site.
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main { split(x) { repeat { 0.5 : { Bay(); } } } }\n"
        "rule Bay { select vertex { all : { Dot(); } } }\n"
        "rule Dot { }\n",
        make_box());

    CHECK_FALSE(result.ok());
    size_t domain_messages = 0;
    for (const Diagnostic& diagnostic : result.diagnostics) {
        if (diagnostic.message.find("component domain is not implemented") != std::string::npos) {
            CHECK_TRUE(diagnostic.severity == Severity::Error);
            ++domain_messages;
        }
    }
    CHECK_EQ(domain_messages, size_t{1});
    // Four bays, each its own terminal because its select made no child.
    CHECK_EQ(result.terminals.size(), size_t{4});
    CHECK_EQ(terminals_named(result, "Dot").size(), size_t{0});
}

TEST(RuleStatements, a_select_on_an_empty_slab_is_a_warning_rather_than_a_failed_generation) {
    // The case that decides the severity, and it is not exotic: a lot narrower
    // than the fixed parts of its own facade. op_split.cpp cuts the surplus at the
    // far end and produces zero-length slabs on purpose, and runs their bodies on
    // purpose -- `1.0 : { }` is a piece of wall. A `select` inside such a body
    // meets a shape with no geometry through no fault of the rule text.
    //
    // ok() is how a caller decides the building came out wrong. A narrow lot must
    // not make it say so, or every facade on a narrow lot reads as a failure.
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main { split(x) { repeat { 1.2 : { Bay(); } } } }\n"
        "rule Bay { split(x) { 1.5 : { Pier(); } ~1.0 : { Window(); } 0.6 : { Pier(); } } }\n"
        "rule Pier { }\n"
        "rule Window { select face { all : { Glass(); } } }\n"
        "rule Glass { }\n",
        make_box());

    CHECK_TRUE(result.ok());
    CHECK_TRUE(has_message(result, Severity::Warning, "this shape has no geometry"));
    CHECK_FALSE(has_message(result, Severity::Error, "this shape has no geometry"));
    // The split still said the lot was too narrow, which is the diagnostic that
    // names the real cause; the downgrade above must not silence it.
    CHECK_TRUE(has_message(result, Severity::Warning, "so the surplus is cut off at the far end"));
    // The piers are real geometry and still came out, and the empty window slab
    // is still a terminal, so the author sees a truncated bay rather than nothing.
    CHECK_EQ(terminals_named(result, "Pier").size(), size_t{2});
    CHECK_EQ(terminals_named(result, "Window").size(), size_t{1});
    CHECK_EQ(terminals_named(result, "Glass").size(), size_t{0});
}

TEST(RuleStatements, a_select_on_a_shape_the_rule_emptied_is_still_only_a_warning) {
    // The same sentence from the other direction: the shape reaching `select` has
    // no geometry because nothing gave it any, rather than because a split cut it
    // to zero. The interpreter cannot tell the two apart and does not try; what it
    // must not do is fail the run. An unimplemented DOMAIN in the same position
    // stays an Error, because that one is in the rule text.
    Shape empty;
    empty.scope.size = glm::dvec3{1.0, 1.0, 1.0};

    const GenerationResult faces = run_on(
        "@start\n"
        "rule Main { select face { all : { P(); } } }\n"
        "rule P { }\n",
        empty);
    CHECK_TRUE(faces.ok());
    CHECK_TRUE(has_message(faces, Severity::Warning, "this shape has no geometry"));

    const GenerationResult domain = run_on(
        "@start\n"
        "rule Main { select vertex { all : { P(); } } }\n"
        "rule P { }\n",
        empty);
    CHECK_FALSE(domain.ok());
    CHECK_TRUE(has_message(domain, Severity::Error,
                           "'vertex' component domain is not implemented in this build"));
    CHECK_FALSE(has_message(domain, Severity::Warning, "this shape has no geometry"));
}

TEST(RuleStatements, a_select_domain_this_build_does_not_implement_is_reported_at_its_line) {
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main {\n"
        "    select vertex { all : { Dot(); } }\n"
        "}\n"
        "rule Dot { }\n",
        make_box());

    CHECK_FALSE(result.ok());
    CHECK_TRUE(has_message(result, Severity::Error,
                           "'vertex' component domain is not implemented in this build"));
    bool located = false;
    for (const Diagnostic& diagnostic : result.diagnostics) {
        if (diagnostic.message.find("'vertex'") != std::string::npos) {
            CHECK_EQ(diagnostic.loc.line, uint32_t{3});
            CHECK_EQ(diagnostic.loc.column, uint32_t{5});
            located = true;
        }
    }
    CHECK_TRUE(located);
    // No components, so no children, so the shape is its own output rather than
    // vanishing.
    CHECK_EQ(result.terminals.size(), size_t{1});
    CHECK_EQ(terminals_named(result, "Dot").size(), size_t{0});
}

TEST(RuleStatements, a_runtime_fault_in_one_select_arm_costs_only_that_component) {
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main {\n"
        "    select face {\n"
        "        top : { extrude(0.0); Bad(); }\n"
        "        side : { Wall(); }\n"
        "    }\n"
        "}\n"
        "rule Bad { }\n"
        "rule Wall { }\n",
        make_box());

    CHECK_FALSE(result.ok());
    CHECK_EQ(terminals_named(result, "Bad").size(), size_t{0});
    CHECK_EQ(terminals_named(result, "Wall").size(), size_t{4});
}

TEST(RuleStatements, select_and_split_compose_so_a_component_can_be_split_in_turn) {
    // The join between the two statements, which is what the end-to-end case
    // below is made of: a face becomes a shape, and that shape divides.
    const GenerationResult result = run_on(
        "@start\n"
        "rule Main { select face { top : { Roof(); } } }\n"
        "rule Roof { split(x) { 1.0 : { Half(); } ~1.0 : { Half(); } } }\n"
        "rule Half { }\n",
        make_box());

    CHECK_TRUE(result.ok());
    const std::vector<const Shape*> halves = terminals_named(result, "Half");
    CHECK_EQ(halves.size(), size_t{2});
    if (halves.size() != 2) {
        return;
    }
    // The 2 by 4 top face split across its own x: 1 by 4 and 1 by 4.
    CHECK_NEAR(geometry_area(halves[0]->geometry), 4.0, 1e-9);
    CHECK_NEAR(geometry_area(halves[1]->geometry), 4.0, 1e-9);

    // The top face's frame is NOT the identity, and the expectation is written
    // against the frame op_comp.hpp defines rather than against the world. Its
    // normal is +y, which is the pole case: there is no in-plane "up" along +y,
    // so face_component_axes() falls back to the parent's +z for the face's y and
    // takes x = cross(y, z) = cross((0,0,1), (0,1,0)) = (-1, 0, 0). The face's
    // own x therefore runs along world MINUS x, so the first slab of
    // `split(x)` is the half at world x in [1, 2].
    check_vec3(world_centroid(*halves[0]), glm::dvec3{1.5, 3.0, 2.0}, 1e-9);
    check_vec3(world_centroid(*halves[1]), glm::dvec3{0.5, 3.0, 2.0}, 1e-9);
    // Not a symmetry: the two halves are on opposite sides of the face, so an
    // implementation that split in the world frame instead of the component's
    // fails both lines above.
    CHECK_TRUE(world_centroid(*halves[0]).x > world_centroid(*halves[1]).x);
}

// ============================================================================
// The end-to-end case
// ============================================================================

/**
 * @brief A box becomes a facade with windows in it
 *
 * The whole point of the exercise, and the one test in this file that says the
 * feature is finished. Everything asserted below is computed by hand from the
 * rule text and from the seed box; nothing is read back from the output.
 *
 * The seed is the rectangle 6 by 4 in the xz plane. The rule extrudes it 10 up,
 * so the solid is x in [0, 6], y in [0, 10], z in [0, 4].
 *
 * `front` is the face whose normal is dominated by +z, which is the z = 4 face.
 * Its component frame is z = the normal (0, 0, 1), y = the in-plane up
 * (0, 1, 0), and x = cross(y, z) = (1, 0, 0) -- the identity -- so the face's
 * own local coordinates are world coordinates less its minimum corner
 * (0, 0, 4). A split along the face's x is therefore a split along world x, and
 * a split along its y is a split along world y.
 *
 *   - `split(y) { 4.0 : Shopfront(); repeat { 2.0 : UpperFloor(); } }` over the
 *     10-metre height: the shopfront takes 4, leaving 6 for the group. The
 *     group's period is 2, floor(6 / 2) = 3 copies, each 6 / 3 = 2. So the upper
 *     floors sit at y in [4, 6], [6, 8] and [8, 10].
 *   - `split(x) { repeat { 1.5 : Bay(); } }` over the 6-metre width: period 1.5,
 *     floor(6 / 1.5) = 4 copies of 1.5, at x in [0, 1.5], [1.5, 3], [3, 4.5] and
 *     [4.5, 6].
 *   - `split(x) { 0.4 : Pier(); 0.6 : Window(); ~1.0 : Pier(); }` over a
 *     1.5-metre bay: the two fixed parts take 1.0, and the one floating part
 *     takes the remaining 0.5. So within a bay the window is x in [0.4, 1.0].
 *
 * That gives 3 floors x 4 bays x 3 parts = 36 terminals, plus the shopfront: 37.
 * Twelve of them are windows, each 0.6 wide and 2.0 tall, centred at
 * (1.5 * bay + 0.7, 4 + 2 * floor + 1, 4). The sizes are deliberately
 * asymmetric in both axes, so a facade built upside down or back to front does
 * not land on the same numbers.
 */
TEST(RuleStatements, a_rule_file_turns_a_box_into_a_facade_with_windows_in_it) {
    const std::string source =
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
        "rule Shopfront { }\n"
        "rule Pier { }\n"
        "rule Window { }\n";

    const GenerationResult result = run_on(source, shape_from_rect(6.0, 4.0));

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.diagnostics.size(), size_t{0});

    // 1 shopfront + 3 floors x 4 bays x 3 parts.
    CHECK_EQ(result.terminals.size(), size_t{37});
    CHECK_EQ(terminals_named(result, "Shopfront").size(), size_t{1});
    CHECK_EQ(terminals_named(result, "Pier").size(), size_t{24});

    const std::vector<const Shape*> windows = terminals_named(result, "Window");
    CHECK_EQ(windows.size(), size_t{12});

    // The shopfront is the full width of the facade for its first four metres.
    const std::vector<const Shape*> shopfront = terminals_named(result, "Shopfront");
    if (shopfront.size() == 1) {
        glm::dvec3 min_corner{0.0};
        glm::dvec3 max_corner{0.0};
        world_bounds(*shopfront[0], min_corner, max_corner);
        check_vec3(min_corner, glm::dvec3{0.0, 0.0, 4.0}, 1e-9);
        check_vec3(max_corner, glm::dvec3{6.0, 4.0, 4.0}, 1e-9);
    }

    // Every window centre, computed above, must be hit exactly once. A grid built
    // upside down, mirrored, or one bay wide fails: the expected values are not
    // symmetric in either axis.
    std::vector<bool> matched(windows.size(), false);
    size_t found = 0;
    for (int floor = 0; floor < 3; ++floor) {
        for (int bay = 0; bay < 4; ++bay) {
            const glm::dvec3 expected{1.5 * static_cast<double>(bay) + 0.7,
                                      4.0 + 2.0 * static_cast<double>(floor) + 1.0,
                                      4.0};
            int hits = 0;
            for (size_t i = 0; i < windows.size(); ++i) {
                if (matched[i]) {
                    continue;
                }
                const glm::dvec3 centre = world_centroid(*windows[i]);
                if (std::fabs(centre.x - expected.x) < 1e-9 &&
                    std::fabs(centre.y - expected.y) < 1e-9 &&
                    std::fabs(centre.z - expected.z) < 1e-9) {
                    matched[i] = true;
                    ++hits;
                    break;
                }
            }
            found += static_cast<size_t>(hits);
        }
    }
    CHECK_EQ(found, size_t{12});

    // Each window is 0.6 by 2.0 of glass, and there are twelve of them. Area, not
    // scope: a window whose scope was refitted but whose vertices are still the
    // whole bay has the right scope and the wrong area.
    double glass = 0.0;
    for (const Shape* window : windows) {
        CHECK_NEAR(geometry_area(window->geometry), 1.2, 1e-9);
        CHECK_NEAR(window->scope.size.x, 0.6, 1e-9);
        CHECK_NEAR(window->scope.size.y, 2.0, 1e-9);
        glass += geometry_area(window->geometry);
    }
    CHECK_NEAR(glass, 14.4, 1e-9);

    // The solid itself is gone: every terminal is a piece of the front face, so
    // none of them still encloses the ten-metre block.
    for (const Shape& terminal : result.terminals) {
        CHECK_NEAR(geometry_volume(terminal.geometry), 0.0, 1e-9);
    }

    // The tree the editor rebuilds from the terminals: root, face, Facade, four
    // floor slabs, their four rule shapes, twelve bay slabs, twelve Bay shapes,
    // thirty-six part slabs and thirty-six leaf shapes.
    CHECK_EQ(result.stats.shapes_created, uint32_t{107});
    CHECK_EQ(result.stats.terminals, uint32_t{37});
    CHECK_FALSE(result.stats.depth_limit_hit);
    CHECK_FALSE(result.stats.shape_limit_hit);

    // And it is reproducible, which is what a golden test of it would rest on.
    const GenerationResult again = run_on(source, shape_from_rect(6.0, 4.0));
    CHECK_TRUE(result.dump().size() > 5000);
    CHECK_EQ(result.dump(), again.dump());
}

} // namespace
