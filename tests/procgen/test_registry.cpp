// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_registry.cpp
 * @brief full_operations() and full_functions(): the union every non-test caller runs
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ================================================================================
 * WHY THIS FILE EXISTS
 * ================================================================================
 *
 * `standard_operations()` is the D2 set and nothing else, and that is correct:
 * a feature family registers its own operations so that nothing has to edit
 * interpreter.cpp. The cost is that the whole language exists in no single
 * table until registry.cpp assembles one.
 *
 * The failure mode is quiet and specific. A caller handed only the standard
 * table does not crash and does not produce a link error. It reports
 * `operation 'set' is not implemented in this build` -- at the right line, with
 * the right caret, in a build where `set` is demonstrably implemented. The
 * author then goes looking for a bug in their rule file.
 *
 * ================================================================================
 * HOW THESE TESTS ARE WRITTEN SO THAT THEY CAN FAIL
 * ================================================================================
 *
 * Asserting `full_operations().size() > 0` passes for a table holding only the
 * D2 set, which is the exact bug. So every test here runs a rule that uses an
 * operation or a function from a NON-standard family and asserts on the
 * result -- and the paired test asserts the standard table REFUSES the same
 * rule. If registry.cpp stopped calling register_control_operations(), the
 * pair stops disagreeing and the test goes red.
 */

#include "framework.hpp"

#include "procgen/rules/parser.hpp"
#include "procgen/rules/registry.hpp"
#include "procgen/rules/shape.hpp"

#include <string>
#include <string_view>

using namespace stratum::procgen::rules;

namespace {

/// Parse @p source and run it against a 10x8 rectangle with the given tables.
GenerationResult run_with(std::string_view source,
                          const OperationTable* operations,
                          const FunctionTable* functions) {
    const ParseResult parsed = parse(source, "test");
    CHECK_TRUE(parsed.ok());
    const Shape seed = shape_from_rect(10.0, 8.0);
    return generate(parsed.file, seed, GenerationOptions{}, operations, functions);
}

/// True when any diagnostic mentions @p needle
bool reported(const GenerationResult& result, std::string_view needle) {
    for (const auto& diagnostic : result.diagnostics) {
        if (diagnostic.message.find(needle) != std::string::npos) return true;
    }
    return false;
}

} // namespace

// ============================================================================
// Operations
// ============================================================================

TEST(Registry, the_full_table_runs_an_operation_the_standard_table_refuses) {
    // `tag` belongs to op_control, not to the D2 set. It is the cheapest
    // operation in a non-standard family, so this is the narrowest possible
    // probe of "did registry.cpp call register_control_operations()".
    const std::string source = "@start\nrule Main { tag(\"probe\"); }\n";

    const GenerationResult full = run_with(source, &full_operations(), &full_functions());
    CHECK_TRUE(full.ok());
    CHECK_FALSE(reported(full, "not implemented in this build"));

    // The paired half. Without it, this file would pass for a full_operations()
    // that simply returned standard_operations() unchanged -- as long as `tag`
    // had been moved into the D2 set at some point.
    const GenerationResult standard =
        run_with(source, &standard_operations(), &standard_functions());
    CHECK_TRUE(reported(standard, "not implemented in this build"));
}

TEST(Registry, the_full_table_still_has_every_standard_operation) {
    // A registry.cpp that built its table from scratch instead of copying
    // standard_operations() would pass the test above and fail here.
    const std::string source = "@start\nrule Main { extrude(3.0); }\n";
    const GenerationResult full = run_with(source, &full_operations(), &full_functions());
    CHECK_TRUE(full.ok());
    CHECK_FALSE(reported(full, "not implemented in this build"));
    CHECK((full.stats.operations_applied) >= 1u);
}

// ============================================================================
// Functions
// ============================================================================

TEST(Registry, the_full_table_resolves_a_component_function) {
    // comp.count belongs to op_comp. An unregistered function is an unknown
    // name, which is a PARSE-time failure for a qualified call, so this probes
    // the function table specifically rather than the operation table.
    const std::string source =
        "@start\nrule Main { extrude(3.0); print(comp.count(\"top\")); }\n";

    const GenerationResult full = run_with(source, &full_operations(), &full_functions());
    CHECK_TRUE(full.ok());
    CHECK_EQ(full.log.size(), size_t{1});

    const GenerationResult standard =
        run_with(source, &standard_operations(), &standard_functions());
    CHECK_FALSE(standard.ok());
}

TEST(Registry, the_full_table_resolves_a_control_function) {
    // random.* belongs to op_control's function half, which is registered by a
    // different call than its operation half. A registry.cpp that called
    // register_control_operations() but forgot register_control_functions()
    // passes every test above and fails this one.
    const std::string source =
        "@start\nrule Main { print(random.unit()); }\n";

    const GenerationResult full = run_with(source, &full_operations(), &full_functions());
    CHECK_TRUE(full.ok());
    CHECK_EQ(full.log.size(), size_t{1});

    const GenerationResult standard =
        run_with(source, &standard_operations(), &standard_functions());
    CHECK_FALSE(standard.ok());
}

TEST(Registry, the_full_table_still_has_every_standard_function) {
    const std::string source = "@start\nrule Main { print(sqrt(16.0)); }\n";
    const GenerationResult full = run_with(source, &full_operations(), &full_functions());
    CHECK_TRUE(full.ok());
    CHECK_EQ(full.log.size(), size_t{1});
    CHECK_TRUE(full.log[0].find('4') != std::string::npos);
}

// ============================================================================
// The tables themselves
// ============================================================================

TEST(Registry, the_tables_are_the_same_objects_every_call) {
    // They are returned by reference and built once. A copy per call would work
    // and would cost a map rebuild on every generate(), which is the sort of
    // thing that is only ever noticed in a profile.
    CHECK_TRUE(&full_operations() == &full_operations());
    CHECK_TRUE(&full_functions() == &full_functions());
}

TEST(Registry, the_full_tables_are_not_the_standard_tables) {
    // Guards the whole file: if these were the same object, every "full" case
    // above would be testing standard_operations() under another name.
    CHECK_TRUE(&full_operations() != &standard_operations());
    CHECK_TRUE(&full_functions() != &standard_functions());
}
