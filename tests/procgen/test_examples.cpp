// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_examples.cpp
 * @brief Every rule file in examples/rules/ parses, runs and builds geometry
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ================================================================================
 * WHY THIS EXISTS
 * ================================================================================
 *
 * A shipped example that does not run is worse than no example. It is the
 * first thing a new reader opens, and when it reports an error they conclude
 * the language is broken rather than that the file rotted. Examples also rot
 * silently: they are not compiled, so nothing notices when an operation's
 * arity changes or a catalogue row is renamed.
 *
 * So the examples are TESTS. Every `.rule` file in examples/rules/ is parsed
 * and generated on each build, through `full_operations()` -- the same table
 * the rule editor panel runs -- and has to come back with no error diagnostic
 * and with geometry.
 *
 * ================================================================================
 * HOW THIS TEST CAN FAIL
 * ================================================================================
 *
 * The obvious way to write it is "for each file, check it runs", and that
 * passes for an empty directory, a mistyped path, a build that never defined
 * STRATUM_EXAMPLES_DIR, and a CI checkout that did not fetch the folder. Every
 * one of those is a silent pass, which is this project's most-found defect.
 *
 * So the count is pinned. `kExampleCount` is asserted against what the
 * directory actually holds, and adding or deleting an example is expected to
 * break this file -- that is the reminder to look at it.
 *
 * Each example is also asserted to produce a MINIMUM amount of geometry, taken
 * from what it produced when it was written. A rule file that still parses but
 * now yields four vertices because a split stopped repeating would otherwise
 * pass, and a flat box is exactly the failure these examples exist to prevent.
 */

#include "framework.hpp"

#include "procgen/rules/parser.hpp"
#include "procgen/rules/registry.hpp"
#include "procgen/rules/shape.hpp"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace stratum::procgen::rules;

namespace {

/// How many .rule files examples/rules/ holds. See the header: this is pinned
/// on purpose, so adding one is a deliberate act rather than an accident.
constexpr size_t kExampleCount = 12;

struct Example {
    const char* file;
    double width;
    double depth;
    size_t min_terminals;
    size_t min_triangles;
};

/// The seed each example is written for, and the floor its output must clear.
/// The numbers are what each produced when it was written, rounded well down --
/// they are a rot detector, not a golden value, so an improvement does not
/// fail them.
constexpr Example kExamples[] = {
    {"01_office_tower.rule",      30.0, 20.0, 400, 5000},
    {"02_terrace.rule",           24.0, 12.0,  30,  300},
    {"03_courtyard_block.rule",   60.0, 40.0, 600, 6000},
    {"04_townhouse.rule",          7.0, 10.0,  20,  200},
    {"05_tower_and_spire.rule",   30.0, 14.0,  50,  600},
    {"06_warehouse.rule",         48.0, 24.0,  40,  500},
    {"07_by_attribute.rule",      16.0, 12.0, 100, 2000},
    {"08_stochastic_street.rule", 60.0, 14.0, 150, 1500},
    // The recursive four. Their floors are set well under what they produce,
    // because a recursion's output moves with any change to its termination
    // test and a tight bound here would be a brittle golden value.
    {"09_twisting_tower.rule",    10.0, 10.0, 200,  4000},
    {"10_recursive_district.rule", 120.0, 80.0, 1500, 15000},
    {"11_fractal_tower.rule",     32.0, 32.0, 1200, 25000},
    {"12_stacked_modules.rule",   24.0, 14.0, 100,  1200},
};

[[nodiscard]] std::filesystem::path examples_dir() {
    return std::filesystem::path{STRATUM_EXAMPLES_DIR};
}

[[nodiscard]] bool read_file(const std::filesystem::path& path, std::string& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    std::ostringstream buffer;
    buffer << in.rdbuf();
    out = buffer.str();
    return true;
}

[[nodiscard]] std::vector<std::string> rule_files() {
    std::vector<std::string> names;
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator(examples_dir(), ec)) {
        if (entry.is_regular_file() && entry.path().extension() == ".rule") {
            names.push_back(entry.path().filename().string());
        }
    }
    // Sorted, so the failure message names files in a stable order whatever the
    // filesystem hands back.
    std::sort(names.begin(), names.end());
    return names;
}

[[nodiscard]] std::string first_error(const GenerationResult& result) {
    for (const Diagnostic& d : result.diagnostics) {
        if (d.severity == Severity::Error) return d.message;
    }
    return {};
}

} // namespace

// ============================================================================
// The directory itself
// ============================================================================

TEST(Examples, the_examples_directory_holds_what_the_table_below_expects) {
    // The guard that stops every other test in this file passing vacuously.
    // An empty directory, a mistyped STRATUM_EXAMPLES_DIR, or a checkout that
    // did not fetch the folder all land here rather than sliding through as a
    // zero-iteration loop.
    const std::vector<std::string> found = rule_files();
    CHECK_EQ(found.size(), kExampleCount);

    for (const Example& example : kExamples) {
        bool present = false;
        for (const std::string& name : found) {
            if (name == example.file) { present = true; break; }
        }
        if (!present) {
            std::printf("  missing example: %s\n", example.file);
        }
        CHECK_TRUE(present);
    }

    // And the other direction: a file on disk that the table does not know
    // about is not being exercised, which is how an example rots unnoticed.
    for (const std::string& name : found) {
        bool listed = false;
        for (const Example& example : kExamples) {
            if (name == example.file) { listed = true; break; }
        }
        if (!listed) {
            std::printf("  example not in the table: %s\n", name.c_str());
        }
        CHECK_TRUE(listed);
    }
}

TEST(Examples, the_readme_lists_every_example) {
    // The README is the first thing a reader opens. An example missing from its
    // table is an example nobody finds.
    std::string readme;
    CHECK_TRUE(read_file(examples_dir() / "README.md", readme));
    for (const Example& example : kExamples) {
        if (readme.find(example.file) == std::string::npos) {
            std::printf("  README does not mention: %s\n", example.file);
        }
        CHECK_TRUE(readme.find(example.file) != std::string::npos);
    }
}

// ============================================================================
// Every example runs
// ============================================================================

TEST(Examples, every_example_parses_with_no_error) {
    for (const Example& example : kExamples) {
        std::string source;
        CHECK_TRUE(read_file(examples_dir() / example.file, source));
        const ParseResult parsed = parse(source, example.file);
        if (!parsed.ok()) {
            std::printf("  %s:\n%s", example.file, parsed.render_all(source).c_str());
        }
        CHECK_TRUE(parsed.ok());
    }
}

TEST(Examples, every_example_generates_geometry_without_an_error) {
    for (const Example& example : kExamples) {
        std::string source;
        CHECK_TRUE(read_file(examples_dir() / example.file, source));
        const ParseResult parsed = parse(source, example.file);
        CHECK_TRUE(parsed.ok());
        if (!parsed.ok()) continue;

        const Shape seed = shape_from_rect(example.width, example.depth);
        const GenerationResult result =
            generate(parsed.file, seed, GenerationOptions{}, &full_operations(), &full_functions());

        const std::string error = first_error(result);
        if (!error.empty()) {
            std::printf("  %s: %s\n", example.file, error.c_str());
        }
        CHECK_TRUE(error.empty());

        // Not a flat box. The whole point of an example is that it builds
        // something, and "it parsed" is the weaker claim these files exist to
        // go beyond.
        const stratum::Mesh mesh = result.build_mesh();
        const size_t triangles = mesh.indices.size() / 3;
        if (result.terminals.size() < example.min_terminals || triangles < example.min_triangles) {
            std::printf("  %s: %zu terminals (want >= %zu), %zu triangles (want >= %zu)\n",
                        example.file, result.terminals.size(), example.min_terminals,
                        triangles, example.min_triangles);
        }
        CHECK((result.terminals.size()) >= example.min_terminals);
        CHECK((triangles) >= example.min_triangles);
    }
}

TEST(Examples, no_example_hits_a_generation_cap) {
    // A cap means the output was truncated, so the example is showing a
    // fragment of the building it describes. An example is exactly the wrong
    // place for that.
    for (const Example& example : kExamples) {
        std::string source;
        CHECK_TRUE(read_file(examples_dir() / example.file, source));
        const ParseResult parsed = parse(source, example.file);
        if (!parsed.ok()) continue;
        const GenerationResult result =
            generate(parsed.file, shape_from_rect(example.width, example.depth),
                     GenerationOptions{}, &full_operations(), &full_functions());
        if (result.stats.depth_limit_hit || result.stats.shape_limit_hit) {
            std::printf("  %s hit a cap: depth=%d shapes=%d\n", example.file,
                        result.stats.depth_limit_hit ? 1 : 0,
                        result.stats.shape_limit_hit ? 1 : 0);
        }
        CHECK_FALSE(result.stats.depth_limit_hit);
        CHECK_FALSE(result.stats.shape_limit_hit);
    }
}

TEST(Examples, every_select_face_in_an_example_accounts_for_the_top) {
    // A face that `select face` does not name is DROPPED, not left alone. So a
    // block that lists front/back/left/right and omits `top` produces a
    // building with walls and no roof -- and reports nothing, because nothing
    // went wrong. It is simply a building with a hole in it.
    //
    // That is exactly what 01_office_tower and two rules in 07_by_attribute
    // did until it was spotted in a screenshot. Nothing in the geometry checks
    // caught it: every terminal had geometry, every normal pointed the right
    // way, no triangle was inside out, and the roof that was never asked for
    // was never missed.
    //
    // This is a source lint rather than a geometry check because that is what
    // the defect is. The rule file is asking for the wrong thing, and the
    // generator is right to give it exactly what it asked for.
    for (const Example& example : kExamples) {
        std::string source;
        CHECK_TRUE(read_file(examples_dir() / example.file, source));

        size_t at = 0;
        while ((at = source.find("select face", at)) != std::string::npos) {
            const size_t open = source.find('{', at);
            if (open == std::string::npos) break;
            // Walk to the matching brace so a nested arm body does not end the
            // block early.
            size_t depth = 0;
            size_t close = open;
            for (size_t i = open; i < source.size(); ++i) {
                if (source[i] == '{') ++depth;
                else if (source[i] == '}') {
                    --depth;
                    if (depth == 0) { close = i; break; }
                }
            }
            const std::string block = source.substr(open, close - open + 1);
            const bool covered = block.find("top") != std::string::npos ||
                                 block.find("all") != std::string::npos;
            if (!covered) {
                const size_t line = 1 + static_cast<size_t>(
                    std::count(source.begin(), source.begin() + static_cast<long>(at), '\n'));
                std::printf("  %s:%zu: a select face block names no top or all arm, "
                            "so the top face is dropped and the building has no roof\n",
                            example.file, line);
            }
            CHECK_TRUE(covered);
            at = close + 1;
        }
    }
}

TEST(Examples, the_stochastic_street_reproduces_exactly_and_varies_with_the_seed) {
    // 08 makes a claim in its own header comment, and this is the test that
    // holds it to it: the same seed gives a byte-identical street, a different
    // seed gives a different one. A test asserting only the first half passes
    // for a file that ignores the seed entirely.
    std::string source;
    CHECK_TRUE(read_file(examples_dir() / "08_stochastic_street.rule", source));
    const ParseResult parsed = parse(source, "08_stochastic_street.rule");
    CHECK_TRUE(parsed.ok());
    if (!parsed.ok()) return;

    const Shape seed = shape_from_rect(60.0, 14.0);
    auto run = [&](uint64_t s) {
        GenerationOptions options;
        options.seed = s;
        return generate(parsed.file, seed, options, &full_operations(), &full_functions()).dump();
    };

    const std::string a = run(0);
    const std::string b = run(0);
    const std::string c = run(7);

    CHECK_TRUE(a == b);
    CHECK_FALSE(a == c);
    CHECK_TRUE(a.size() > 1000);
}
