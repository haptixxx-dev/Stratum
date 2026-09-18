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

#include <cstdio>
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

/**
 * @brief Which catalogue rows still have no handler
 *
 * Sorted, as kBuiltinOperations is. Updating this list is the LAST step of
 * landing an operation family, and the test below is what makes forgetting it
 * impossible: register a handler without wiring the family into registry.cpp
 * and the name stays here, so the list still matches and nothing tells you.
 * Wire it in and the list stops matching, which is the reminder.
 */
constexpr std::string_view kUnimplemented[] = {
    "center", "color", "convexify", "delete_holes",
    "delete_uv", "footprint", "inner_rect",
    "normalize_uv", "primitive", "project_uv", "reduce",
    "report", "scale_uv", "scatter", "setup_projection",
    "soften_normals", "texture", "tile_uv", "translate_uv",
    "trim",
};

TEST(Registry, the_unimplemented_catalogue_rows_are_exactly_the_expected_ones) {
    // Two directions, because one alone is half a test.
    //
    // A name in kBuiltinOperations with no handler is not a bug -- ast.hpp's
    // catalogue is the PUBLISHED surface of the language and a row is a promise
    // with a date on it, reported at the call site as "not implemented in this
    // build". What IS a bug is a family that registered its handlers and was
    // never added to registry.cpp: every one of its operations then reports
    // exactly that message, in a build where they demonstrably work, and
    // nothing else goes wrong to point at the cause.
    for (size_t i = 0; i < kBuiltinOperationCount; ++i) {
        const std::string_view name = kBuiltinOperations[i].name;
        bool expected_missing = false;
        for (const std::string_view listed : kUnimplemented) {
            if (listed == name) { expected_missing = true; break; }
        }
        const bool actually_missing = full_operations().find(name) == nullptr;
        if (expected_missing != actually_missing) {
            // The name is the whole point of the message: "17 != 18" sends the
            // reader to count rows, and the answer is one word.
            std::printf("  operation '%.*s': expected %s, found %s\n",
                        static_cast<int>(name.size()), name.data(),
                        expected_missing ? "no handler" : "a handler",
                        actually_missing ? "none" : "one");
        }
        CHECK_EQ(expected_missing, actually_missing);
    }
}

TEST(Registry, every_unimplemented_name_is_a_real_catalogue_row) {
    // Guards the list above against a typo. Without this, misspelling a name in
    // kUnimplemented makes the test above demand a handler for a row that does
    // not exist, and the failure names a row nobody can find.
    for (const std::string_view listed : kUnimplemented) {
        CHECK_TRUE(find_builtin_operation(listed) != kNoNode);
    }
}

TEST(Registry, the_full_tables_are_not_the_standard_tables) {
    // Guards the whole file: if these were the same object, every "full" case
    // above would be testing standard_operations() under another name.
    CHECK_TRUE(&full_operations() != &standard_operations());
    CHECK_TRUE(&full_functions() != &standard_functions());
}
