// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_op_control.cpp
 * @brief D6: user attributes with provenance, shape tags, stochastic draws, and the annotation schema
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ================================================================================
 * HOW THESE TESTS ARE WRITTEN SO THAT THEY CAN FAIL
 * ================================================================================
 *
 * A test that cannot fail is worse than no test. Every group below has a
 * deliberate way of failing, and each was checked by breaking the implementation
 * on purpose and watching the named test go red:
 *
 *   - **The determinism tests perturb something and assert the perturbation was
 *     real.** "Two runs agree" passes for an interpreter that produces nothing.
 *     So a_draw_does_not_depend_on_how_many_shapes_preceded_it asserts that the
 *     shape COUNT changed between the two runs before it asserts that the drawn
 *     number did not. Swap the draw for a counter drawn from one shared stream
 *     and this is the test that goes red.
 *
 *   - **Both halves of the seed mix are pinned separately.**
 *     two_call_sites_in_one_rule_draw_independently fails if the call site is
 *     dropped from the mix; siblings_of_one_rule_draw_independently fails if the
 *     shape's address is. A test of only one would pass for an implementation
 *     that used only the other.
 *
 *   - **Every "never chosen" test has a control that shows the value CAN be
 *     chosen.** a_zero_weight_is_never_chosen would pass for an implementation
 *     that always returned the last element, so the same rule with equal weights
 *     must produce both values in the same test.
 *
 *   - **Every refusal test asserts the MESSAGE, not the count.** A generation
 *     that failed for an unrelated reason satisfies `!ok()`, so each of these
 *     names a phrase from the diagnostic it expects.
 *
 *   - **A guard every function shares is pinned for every function.**
 *     every_shape_reading_function_is_refused_while_a_file_level_value_resolves
 *     is a table with one row per function and a count check tying it to the
 *     registration list, because an earlier version named two of the twelve and
 *     a mutation that dropped fn_needs_shape() from a third left the suite
 *     green -- turning `attr h : float = random.range(3.0, 9.0)` into a fixed
 *     number that looks random.
 *
 *   - **A containment claim is asserted on a shape that was never given the
 *     key.** a_shape_that_was_never_given_an_attribute_does_not_see_it exists
 *     because the sibling test above it re-sets the same key before each call,
 *     so a `set` that leaked through one shared map was overwritten before
 *     anybody read it and 40 tests stayed green.
 *
 *   - **The known hole is a test, not a comment.** a_bare_name_does_not_yet_see
 *     _a_set records what the language does TODAY: `set("h", 20)` is invisible
 *     to a bare `h`, because interpreter.cpp's lookup_name() never consults
 *     Shape::attributes and nothing a registered handler can do changes that.
 *     That test is expected to fail the day those six lines land, and its
 *     comment says what to change it to. A hole recorded in prose is a hole
 *     nobody notices being filled.
 *
 * Nothing here needs a GPU, a window or a file on disk, so nothing here skips.
 */

#include "framework.hpp"

#include "procgen/rules/ast.hpp"
#include "procgen/rules/interpreter.hpp"
#include "procgen/rules/lexer.hpp"
#include "procgen/rules/op_control.hpp"
#include "procgen/rules/parser.hpp"
#include "procgen/rules/shape.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

using stratum::procgen::rules::attr_source_name;
using stratum::procgen::rules::AttrSource;
using stratum::procgen::rules::attribute_schema;
using stratum::procgen::rules::attribute_source;
using stratum::procgen::rules::AttributeSchema;
using stratum::procgen::rules::AttrInfo;
using stratum::procgen::rules::control_functions;
using stratum::procgen::rules::control_operations;
using stratum::procgen::rules::Diagnostic;
using stratum::procgen::rules::FunctionTable;
using stratum::procgen::rules::generate;
using stratum::procgen::rules::GenerationOptions;
using stratum::procgen::rules::GenerationResult;
using stratum::procgen::rules::kTagsAttribute;
using stratum::procgen::rules::literal_value;
using stratum::procgen::rules::OperationArgs;
using stratum::procgen::rules::OperationTable;
using stratum::procgen::rules::parse;
using stratum::procgen::rules::ParseResult;
using stratum::procgen::rules::PrimitiveType;
using stratum::procgen::rules::register_control_functions;
using stratum::procgen::rules::register_control_operations;
using stratum::procgen::rules::Severity;
using stratum::procgen::rules::Shape;
using stratum::procgen::rules::shape_from_rect;
using stratum::procgen::rules::shape_has_tag;
using stratum::procgen::rules::shape_tags;
using stratum::procgen::rules::standard_functions;
using stratum::procgen::rules::standard_operations;
using stratum::procgen::rules::Value;

namespace {

// ============================================================================
// Helpers
// ============================================================================

/**
 * @brief Parse @p source and assert it parsed cleanly
 *
 * The parse is asserted as a rendered string and not as a boolean, so a test
 * whose rule text has a typo prints the parser's own caret diagnostic instead of
 * "false != true". render_all() is given the SOURCE for that reason: it returns
 * a non-empty string for a real diagnostic either way, so an empty argument
 * still fails, but it fails with the message and no line under it. Every parse
 * in this file goes through here so that none of them can drift back.
 */
[[nodiscard]] ParseResult parse_clean(const std::string& source) {
    const ParseResult parsed = parse(source, "test.srl");
    CHECK_EQ(parsed.render_all(source), std::string{});
    return parsed;
}

/**
 * @brief Parse @p source and run it against the unit square with D6 registered
 */
[[nodiscard]] GenerationResult run(const std::string& source,
                                   const GenerationOptions& options = GenerationOptions{}) {
    const ParseResult parsed = parse_clean(source);
    return generate(parsed.file, shape_from_rect(1.0, 1.0), options, &control_operations(),
                    &control_functions());
}

/// Does any diagnostic contain @p fragment?
[[nodiscard]] bool has_error_saying(const GenerationResult& result, const std::string& fragment) {
    for (const Diagnostic& diagnostic : result.diagnostics) {
        if (diagnostic.severity == Severity::Error &&
            diagnostic.message.find(fragment) != std::string::npos) {
            return true;
        }
    }
    return false;
}

/**
 * @brief What @p result refused with, as a string a table-driven check compares
 *
 * A loop of CHECK_TRUE(has_error_saying(...)) reports "false != true" for one of
 * twelve rows and does not say which one, nor what happened instead. This
 * returns @p wanted when the refusal is the expected one and otherwise returns
 * what the generation actually did, so the failure names the function that was
 * let through and the message it gave.
 */
[[nodiscard]] std::string refusal(const GenerationResult& result, const std::string& wanted) {
    if (result.ok()) {
        return "accepted with no error at all";
    }
    if (has_error_saying(result, wanted)) {
        return wanted;
    }
    for (const Diagnostic& diagnostic : result.diagnostics) {
        if (diagnostic.severity == Severity::Error) {
            return diagnostic.message;
        }
    }
    return "refused without an error diagnostic";
}

[[nodiscard]] bool has_diagnostic_saying(const std::vector<Diagnostic>& diagnostics,
                                         const std::string& fragment) {
    for (const Diagnostic& diagnostic : diagnostics) {
        if (diagnostic.message.find(fragment) != std::string::npos) {
            return true;
        }
    }
    return false;
}

/**
 * @brief A rule file that runs @p body once per item, @p count items deep
 *
 * The workhorse for the stochastic tests, which need many shapes that differ
 * only in their address in the tree. Each Item is a child of a different Row, so
 * every one of them has a distinct Shape::seed_key and a draw that has to differ
 * from its neighbours' for the right reason.
 */
[[nodiscard]] std::string rows_of(int count, const std::string& body) {
    return "@start\n"
           "rule Main { Row(" +
           std::to_string(count) +
           ".0); }\n"
           "rule Row(k : float) { if (k > 0.0) { Item(); Row(k - 1.0); } }\n"
           "rule Item { " +
           body + " extrude(1.0); }\n";
}

/// How many log lines equal @p wanted
[[nodiscard]] size_t count_lines(const GenerationResult& result, const std::string& wanted) {
    size_t total = 0;
    for (const std::string& line : result.log) {
        if (line == wanted) {
            ++total;
        }
    }
    return total;
}

/// Every log line read as a number. Lines that are not numbers become 1e30, which
/// fails every bound a caller checks rather than quietly reading as zero.
[[nodiscard]] std::vector<double> logged_numbers(const GenerationResult& result) {
    std::vector<double> values;
    values.reserve(result.log.size());
    for (const std::string& line : result.log) {
        try {
            size_t used = 0;
            const double value = std::stod(line, &used);
            values.push_back(used == line.size() ? value : 1.0e30);
        } catch (...) {
            values.push_back(1.0e30);
        }
    }
    return values;
}

// ============================================================================
// Provenance
// ============================================================================

TEST(OpControl, the_precedence_of_the_chain_is_pinned) {
    // The order is the whole content of attribute_source(), and every other
    // attribute test in this file depends on it. It is asserted directly rather
    // than through a generation so that a change to the order fails HERE, at the
    // one function that decides it, and not in five tests that each look like
    // something else broke.
    const ParseResult parsed = parse_clean("attr height : float = 12.0\n"
                                           "@start\n"
                                           "rule Main { extrude(1.0); }\n");

    Shape shape = shape_from_rect(1.0, 1.0);
    GenerationOptions options;

    // Nothing declares 'width' and nothing set it.
    CHECK_EQ(std::string{attr_source_name(attribute_source(parsed.file, options, shape, "width"))},
             std::string{"none"});

    // Declared, nothing else: the declaration's own default answers.
    CHECK_EQ(std::string{attr_source_name(attribute_source(parsed.file, options, shape, "height"))},
             std::string{"default"});

    // An Inspector or a layer supplied one: it beats the default.
    options.attributes["height"] = Value::number(30.0);
    CHECK_EQ(std::string{attr_source_name(attribute_source(parsed.file, options, shape, "height"))},
             std::string{"supplied"});

    // A rule set one on this shape: it beats both. See op_control.hpp for why
    // the most specific write wins rather than the most global.
    shape.attributes["height"] = Value::number(9.0);
    CHECK_EQ(std::string{attr_source_name(attribute_source(parsed.file, options, shape, "height"))},
             std::string{"shape"});

    // And a shape value for a name nothing declares is still a shape value: a
    // rule may write an attribute the file never declared.
    shape.attributes["width"] = Value::number(4.0);
    CHECK_EQ(std::string{attr_source_name(attribute_source(parsed.file, options, shape, "width"))},
             std::string{"shape"});

    // The four words are distinct, so a test comparing them is comparing
    // something.
    CHECK_FALSE(std::string{attr_source_name(AttrSource::None)} ==
                std::string{attr_source_name(AttrSource::Default)});
    CHECK_FALSE(std::string{attr_source_name(AttrSource::Default)} ==
                std::string{attr_source_name(AttrSource::Supplied)});
    CHECK_FALSE(std::string{attr_source_name(AttrSource::Supplied)} ==
                std::string{attr_source_name(AttrSource::Shape)});
}

TEST(OpControl, a_mistyped_override_is_not_reported_as_supplied) {
    // attribute_source() has its own copy of the interpreter's type check, and a
    // copy that drifted would tell an author their override is live while the
    // interpreter quietly used the default. Both halves are asserted in one
    // test: the interpreter's complaint AND the provenance that must agree with
    // it.
    GenerationOptions options;
    options.attributes["height"] = Value::text("tall");

    const GenerationResult result = run("attr height : float = 12.0\n"
                                        "@start\n"
                                        "rule Main {\n"
                                        "  print(attrs.source(\"height\"));\n"
                                        "  print(attrs.get(\"height\"));\n"
                                        "  extrude(1.0);\n"
                                        "}\n",
                                        options);

    CHECK_TRUE(has_error_saying(result, "the supplied value for attribute 'height' is a string"));
    CHECK_EQ(result.log.size(), size_t{2});
    if (result.log.size() == 2) {
        CHECK_EQ(result.log[0], std::string{"default"});
        CHECK_EQ(result.log[1], std::string{"12"});
    }
}

TEST(OpControl, an_override_of_a_name_nothing_declares_is_not_supplied) {
    // The interpreter reports an undeclared override and uses nothing. A
    // provenance of "supplied" here would confirm a slider that does nothing,
    // which is the exact failure scene/attributes.hpp says the source exists to
    // prevent.
    GenerationOptions options;
    options.attributes["heigth"] = Value::number(30.0);

    const GenerationResult result = run("attr height : float = 12.0\n"
                                        "@start\n"
                                        "rule Main { print(attrs.source(\"heigth\")); extrude(1.0); }\n",
                                        options);

    CHECK_TRUE(has_error_saying(result, "there is no attribute called 'heigth' to set"));
    CHECK_EQ(result.log.size(), size_t{1});
    if (!result.log.empty()) {
        CHECK_EQ(result.log[0], std::string{"none"});
    }
}

TEST(OpControl, a_supplied_value_beats_the_default_and_a_set_beats_both) {
    GenerationOptions options;
    options.attributes["height"] = Value::number(30.0);

    const GenerationResult result = run("attr height : float = 12.0\n"
                                        "@start\n"
                                        "rule Main {\n"
                                        "  print(attrs.get(\"height\"));\n"
                                        "  print(attrs.source(\"height\"));\n"
                                        "  set(\"height\", 9.0);\n"
                                        "  print(attrs.get(\"height\"));\n"
                                        "  print(attrs.source(\"height\"));\n"
                                        "  extrude(1.0);\n"
                                        "}\n",
                                        options);

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.log.size(), size_t{4});
    if (result.log.size() == 4) {
        CHECK_EQ(result.log[0], std::string{"30"});
        CHECK_EQ(result.log[1], std::string{"supplied"});
        CHECK_EQ(result.log[2], std::string{"9"});
        CHECK_EQ(result.log[3], std::string{"shape"});
    }
}

TEST(OpControl, a_set_is_inherited_by_the_subtree_in_the_order_it_was_written) {
    // The ORDER half of what `set` promises: a child sees what its parent set
    // before the call and not what the parent set after it, because the child
    // was snapshotted at the call.
    //
    // This test cannot make the containment claim, and does not try to: Main
    // sets "style" before BOTH calls, so a leak of that key from one child into
    // the other is overwritten before anybody reads it. The test below is the
    // one that catches a leak.
    const GenerationResult result = run("@start\n"
                                        "rule Main {\n"
                                        "  set(\"style\", \"brick\");\n"
                                        "  Left();\n"
                                        "  set(\"style\", \"glass\");\n"
                                        "  Right();\n"
                                        "}\n"
                                        "rule Left { print(attrs.get(\"style\")); extrude(1.0); }\n"
                                        "rule Right { print(attrs.get(\"style\")); extrude(1.0); }\n");

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.log.size(), size_t{2});
    if (result.log.size() == 2) {
        CHECK_EQ(result.log[0], std::string{"brick"});
        CHECK_EQ(result.log[1], std::string{"glass"});
    }

    // And it survives onto the terminal, where an exporter can read it.
    CHECK_EQ(result.terminals.size(), size_t{2});
    if (result.terminals.size() == 2) {
        const auto found = result.terminals[0].attributes.find("style");
        CHECK_TRUE(found != result.terminals[0].attributes.end());
        if (found != result.terminals[0].attributes.end()) {
            CHECK_TRUE(found->second.is_text());
            CHECK_EQ(found->second.as_text(), std::string{"brick"});
        }
    }
}

TEST(OpControl, a_shape_that_was_never_given_an_attribute_does_not_see_it) {
    // ---------------------------------------------------------------------
    // THE CONTAINMENT CLAIM, AND THE ONE MUTATION IT EXISTS TO CATCH.
    //
    // Replace `context.shape.attributes[name] = context.args[1]` with a write
    // into a process-wide map whose accumulated keys are copied onto every
    // shape, and the test above still passes: every read it makes is of a key
    // the parent set on that very path. This one fails, because nothing above
    // B ever sets "style", so B may not have it -- not at the read, and not on
    // the terminal an exporter walks. Both are asserted; a leak that reached
    // only the terminal would be invisible in the log.
    //
    // Inner is in the same test so that the two halves cannot drift apart: an
    // implementation that contained `set` by never inheriting it at all would
    // pass the containment half and fail the inheritance half.
    // ---------------------------------------------------------------------
    const GenerationResult result =
        run("@start\n"
            "rule Main { A(); B(); }\n"
            "rule A { set(\"style\", \"brick\"); Inner(); }\n"
            "rule Inner { print(attrs.get(\"style\", \"none\")); extrude(1.0); }\n"
            "rule B { set(\"floors\", 2.0); print(attrs.get(\"style\", \"none\")); extrude(1.0); }\n");

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.log.size(), size_t{2});
    if (result.log.size() == 2) {
        CHECK_EQ(result.log[0], std::string{"brick"});  // down into A's subtree
        CHECK_EQ(result.log[1], std::string{"none"});   // and not across to B
    }

    CHECK_EQ(result.terminals.size(), size_t{2});
    if (result.terminals.size() != 2) {
        return;
    }

    const auto& inner = result.terminals[0].attributes;
    const auto inherited = inner.find("style");
    CHECK_TRUE(inherited != inner.end());
    if (inherited != inner.end()) {
        CHECK_TRUE(inherited->second.is_text());
        CHECK_EQ(inherited->second.as_text(), std::string{"brick"});
    }

    // B carries what B set and NOTHING of A's. The size check is what catches a
    // leak of a key this test did not think to name.
    const auto& sibling = result.terminals[1].attributes;
    CHECK_TRUE(sibling.find("style") == sibling.end());
    CHECK_TRUE(sibling.find("floors") != sibling.end());
    CHECK_EQ(sibling.size(), size_t{1});
}

TEST(OpControl, a_set_needs_a_name) {
    // The tag("") twin of this is in the tag section, and the reason is the
    // same: an empty name is a key no read can name, so a rule that computed one
    // by mistake would write onto the shape and every later read would miss it
    // with nothing saying why. Delete the check in op_set and the generation
    // below succeeds.
    const GenerationResult result = run("@start\n"
                                        "rule Main { set(\"\", 1.0); extrude(1.0); }\n");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(has_error_saying(result, "'set' needs a name to set"));
    CHECK_TRUE(result.terminals.empty());
}

TEST(OpControl, a_set_that_breaks_the_declared_type_is_refused) {
    const GenerationResult result = run("attr height : float = 12.0\n"
                                        "@start\n"
                                        "rule Main { set(\"height\", \"tall\"); extrude(1.0); }\n");

    CHECK_FALSE(result.ok());
    CHECK_TRUE(has_error_saying(result, "'height' is declared float but was set to a string"));
    // The shape is abandoned, so the wrong building is not produced at all.
    CHECK_TRUE(result.terminals.empty());
}

TEST(OpControl, a_set_of_an_undeclared_name_is_allowed) {
    // A rule writing its own bookkeeping onto a shape is ordinary, and refusing
    // it would force a declaration for every intermediate value.
    const GenerationResult result = run("@start\n"
                                        "rule Main { set(\"storeys\", 4.0); "
                                        "print(attrs.source(\"storeys\")); extrude(1.0); }\n");

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.log.size(), size_t{1});
    if (!result.log.empty()) {
        CHECK_EQ(result.log[0], std::string{"shape"});
    }
}

TEST(OpControl, the_reserved_tag_name_cannot_be_written_or_read_as_an_attribute) {
    const GenerationResult written = run("@start\n"
                                         "rule Main { set(\"#tags\", 1.0); extrude(1.0); }\n");
    CHECK_FALSE(written.ok());
    CHECK_TRUE(has_error_saying(written, "reserved for the tags this shape carries"));

    const GenerationResult read = run("@start\n"
                                      "rule Main { print(attrs.get(\"#tags\")); extrude(1.0); }\n");
    CHECK_FALSE(read.ok());
    CHECK_TRUE(has_error_saying(read, "is reserved for the shape's tags"));
}

TEST(OpControl, attrs_get_answers_a_written_out_default_and_refuses_a_computed_one) {
    const GenerationResult plain = run("attr height : float = 12.0\n"
                                       "@start\n"
                                       "rule Main { print(attrs.get(\"height\")); extrude(1.0); }\n");
    CHECK_TRUE(plain.ok());
    CHECK_EQ(plain.log.size(), size_t{1});
    if (!plain.log.empty()) {
        CHECK_EQ(plain.log[0], std::string{"12"});
    }

    // A computed default is refused rather than evaluated by a second evaluator,
    // and the message says both ways to read it. Reading it by NAME still works,
    // which is what the first log line proves.
    const GenerationResult computed =
        run("attr storeys : float = 4.0\n"
            "attr height : float = storeys * 3.0\n"
            "@start\n"
            "rule Main { print(height); print(attrs.get(\"height\")); extrude(1.0); }\n");
    CHECK_FALSE(computed.ok());
    CHECK_TRUE(has_error_saying(computed, "is computed rather than written out"));
    CHECK_EQ(computed.log.size(), size_t{1});
    if (!computed.log.empty()) {
        CHECK_EQ(computed.log[0], std::string{"12"});
    }
    CHECK_TRUE(computed.terminals.empty());

    // The fallback form is the escape the message names, and it answers.
    const GenerationResult fallback =
        run("attr storeys : float = 4.0\n"
            "attr height : float = storeys * 3.0\n"
            "@start\n"
            "rule Main { print(attrs.get(\"height\", height)); extrude(1.0); }\n");
    CHECK_TRUE(fallback.ok());
    CHECK_EQ(fallback.log.size(), size_t{1});
    if (!fallback.log.empty()) {
        CHECK_EQ(fallback.log[0], std::string{"12"});
    }
}

TEST(OpControl, attrs_get_with_a_fallback_answers_for_a_name_nothing_declares) {
    const GenerationResult result =
        run("@start\n"
            "rule Main {\n"
            "  print(attrs.get(\"depth\", 7.0));\n"
            "  set(\"depth\", 3.0);\n"
            "  print(attrs.get(\"depth\", 7.0));\n"
            "  extrude(1.0);\n"
            "}\n");

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.log.size(), size_t{2});
    if (result.log.size() == 2) {
        CHECK_EQ(result.log[0], std::string{"7"});
        CHECK_EQ(result.log[1], std::string{"3"});
    }

    // Without a fallback the same read is refused rather than answered with a
    // zero nobody asked for.
    const GenerationResult bare = run("@start\n"
                                      "rule Main { print(attrs.get(\"depth\")); extrude(1.0); }\n");
    CHECK_FALSE(bare.ok());
    CHECK_TRUE(has_error_saying(bare, "nothing declares or sets 'depth'"));
}

TEST(OpControl, a_bare_name_does_not_yet_see_a_set) {
    // ---------------------------------------------------------------------
    // THIS TEST RECORDS A HOLE, NOT A DESIGN.
    //
    // interpreter.cpp's lookup_name() resolves a bare `height` through the
    // locals and then the file's attributes and constants. It never consults
    // Shape::attributes, and no registered operation or function can make it.
    // So today a `set` is read back with attrs.get() and NOT by name.
    //
    // When interpreter.cpp gains the six lines named in op_control.hpp, this
    // test goes red and the fix is to change the first expected line from "12"
    // to "20" -- NOT to delete the test. A hole recorded in prose is a hole
    // nobody notices being filled.
    // ---------------------------------------------------------------------
    const GenerationResult result =
        run("attr height : float = 12.0\n"
            "@start\n"
            "rule Main {\n"
            "  set(\"height\", 20.0);\n"
            "  print(height);\n"
            "  print(attrs.get(\"height\"));\n"
            "  extrude(1.0);\n"
            "}\n");

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.log.size(), size_t{2});
    if (result.log.size() == 2) {
        CHECK_EQ(result.log[0], std::string{"12"});  // the declaration, not the set
        CHECK_EQ(result.log[1], std::string{"20"});  // the set
    }
}

TEST(OpControl, attrs_has_and_attrs_declared_answer_different_questions) {
    const GenerationResult result =
        run("attr height : float = 12.0\n"
            "@start\n"
            "rule Main {\n"
            "  print(attrs.has(\"height\"));\n"
            "  print(attrs.declared(\"height\"));\n"
            "  print(attrs.has(\"storeys\"));\n"
            "  print(attrs.declared(\"storeys\"));\n"
            "  set(\"storeys\", 4.0);\n"
            "  print(attrs.has(\"storeys\"));\n"
            "  print(attrs.declared(\"storeys\"));\n"
            "  extrude(1.0);\n"
            "}\n");

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.log.size(), size_t{6});
    if (result.log.size() == 6) {
        CHECK_EQ(result.log[0], std::string{"true"});
        CHECK_EQ(result.log[1], std::string{"true"});
        CHECK_EQ(result.log[2], std::string{"false"});
        CHECK_EQ(result.log[3], std::string{"false"});
        // Set on the shape: something holds it, but the FILE still declares
        // nothing. An Inspector needs the second answer, a condition the first.
        CHECK_EQ(result.log[4], std::string{"true"});
        CHECK_EQ(result.log[5], std::string{"false"});
    }
}

TEST(OpControl, every_shape_reading_function_is_refused_while_a_file_level_value_resolves) {
    // ---------------------------------------------------------------------
    // ONE ROW PER FUNCTION, BECAUSE A SAMPLE OF TWO WAS NOT ENOUGH.
    //
    // The worst possible failure for this feature is a draw made against a
    // default-constructed shape: every declaration would draw against seed_key
    // 0, so `attr h : float = random.range(3.0, 9.0)` would be a city-wide
    // constant that LOOKS random, and nothing in the output would say so.
    //
    // An earlier version of this test named random.unit and attrs.get only.
    // Removing `|| !fn_needs_shape(context)` from fn_random_range left the whole
    // suite green, and a driver then printed the same height for every building
    // in the file. So the guard is now pinned for each of the twelve functions
    // that carries it, in BOTH positions it exists for -- an attribute default
    // and a constant -- and each row asserts the message NAMES ITS OWN
    // FUNCTION, so no row can be satisfied by a different function failing.
    //
    // The count check below ties the table to the registration list: a D6
    // function added without a row here fails it rather than going unguarded.
    // ---------------------------------------------------------------------
    struct Probe {
        const char* call;  ///< The call as a rule file writes it
        const char* type;  ///< A declared type its result fits, so nothing else complains
    };
    const Probe probes[] = {
        {"random.unit()", "float"},
        {"random.range(2.0, 4.0)", "float"},
        {"random.integer(1.0, 4.0)", "float"},
        {"random.chance(0.5)", "bool"},
        {"random.pick([\"brick\", \"stone\"])", "string"},
        {"random.weighted([\"brick\", \"stone\"], [1.0, 1.0])", "string"},
        {"tags.has(\"corner\")", "bool"},
        {"tags.count()", "float"},
        {"tags.list()", "string[]"},
        {"attrs.get(\"style\", \"brick\")", "string"},
        {"attrs.source(\"style\")", "string"},
        {"attrs.has(\"style\")", "bool"},
    };
    const size_t rows = sizeof(probes) / sizeof(probes[0]);

    // Every function this family registers except attrs.declared(), which reads
    // the file rather than the shape and is the control at the end of the test.
    CHECK_EQ(rows, control_functions().size() - standard_functions().size() - size_t{1});

    for (const Probe& probe : probes) {
        const std::string call{probe.call};
        const std::string name = call.substr(0, call.find('('));
        const std::string says = "'" + name + "' reads the current shape";

        // An attribute default. This is the one an Inspector shows, and the one
        // the fixed-number failure hides in.
        const GenerationResult defaulted =
            run("attr probe : " + std::string{probe.type} + " = " + call +
                "\n@start\nrule Main { extrude(1.0); }\n");
        CHECK_EQ(refusal(defaulted, says), says);

        // A constant. Resolved once for the file, so a draw here would be one
        // number every shape in the city shared.
        const GenerationResult constant =
            run("const probe : " + std::string{probe.type} + " = " + call +
                "\n@start\nrule Main { print(probe); extrude(1.0); }\n");
        CHECK_EQ(refusal(constant, says), says);
        // The shape that read the unresolvable constant is abandoned rather
        // than built from a value the interpreter never worked out.
        CHECK_TRUE(constant.terminals.empty());
    }

    // THE CONTROL. attrs.declared() reads the FILE, not the shape, so it is
    // legal exactly where the twelve above are not. Without it, a guard bolted
    // onto every function in the family would satisfy every row above and this
    // test would be pinning "nothing works in a declaration".
    const GenerationResult declared = run("attr known : bool = attrs.declared(\"known\")\n"
                                          "@start\n"
                                          "rule Main { print(known); extrude(1.0); }\n");
    CHECK_TRUE(declared.ok());
    CHECK_EQ(declared.log.size(), size_t{1});
    if (!declared.log.empty()) {
        CHECK_EQ(declared.log[0], std::string{"true"});
    }

    // And the same functions ARE legal on a shape, so the refusal is about the
    // missing shape and not about the function being broken everywhere.
    const GenerationResult on_a_shape =
        run("@start\n"
            "rule Main { tag(\"corner\"); print(tags.count()); print(random.unit()); extrude(1.0); }\n");
    CHECK_TRUE(on_a_shape.ok());
    CHECK_EQ(on_a_shape.log.size(), size_t{2});
    if (!on_a_shape.log.empty()) {
        CHECK_EQ(on_a_shape.log[0], std::string{"1"});
    }
}

// ============================================================================
// Tags
// ============================================================================

TEST(OpControl, tags_are_a_set_and_their_order_does_not_depend_on_the_rule) {
    // Two rules say the same true things about their shapes in different orders.
    // If the stored order were the call order, the two shapes would differ in the
    // canonical dump and a determinism test over tagged output would report a
    // difference that is not one.
    const GenerationResult result =
        run("@start\n"
            "rule Main { First(); Second(); }\n"
            "rule First { tag(\"corner\"); tag(\"retail\"); tag(\"corner\"); extrude(1.0); }\n"
            "rule Second { tag(\"retail\"); tag(\"corner\"); extrude(1.0); }\n");

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.terminals.size(), size_t{2});
    if (result.terminals.size() != 2) {
        return;
    }

    const std::vector<std::string> first = shape_tags(result.terminals[0]);
    const std::vector<std::string> second = shape_tags(result.terminals[1]);
    CHECK_EQ(first.size(), size_t{2});  // the repeat did not make a third
    CHECK_TRUE(first == second);
    if (first.size() == 2) {
        CHECK_EQ(first[0], std::string{"corner"});
        CHECK_EQ(first[1], std::string{"retail"});
    }
    CHECK_TRUE(shape_has_tag(result.terminals[0], "retail"));
    CHECK_FALSE(shape_has_tag(result.terminals[0], "residential"));
}

TEST(OpControl, tags_are_inherited_and_readable_in_a_condition) {
    const GenerationResult result =
        run("@start\n"
            "rule Main { tag(\"corner\"); Facade(); }\n"
            "rule Facade {\n"
            "  print(tags.count());\n"
            "  if (tags.has(\"corner\")) { print(\"chamfer\"); } else { print(\"flat\"); }\n"
            "  print(tags.list()[0]);\n"
            "  extrude(1.0);\n"
            "}\n");

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.log.size(), size_t{3});
    if (result.log.size() == 3) {
        CHECK_EQ(result.log[0], std::string{"1"});
        CHECK_EQ(result.log[1], std::string{"chamfer"});
        CHECK_EQ(result.log[2], std::string{"corner"});
    }
}

TEST(OpControl, delete_tags_leaves_a_shape_indistinguishable_from_one_never_tagged) {
    // Stored as an empty array instead of erased, a cleared shape would carry an
    // attribute entry a never-tagged shape does not, and the two would dump
    // differently while being the same shape.
    const GenerationResult result =
        run("@start\n"
            "rule Main { Cleared(); Never(); }\n"
            "rule Cleared { tag(\"corner\"); delete_tags(); extrude(1.0); }\n"
            "rule Never { extrude(1.0); }\n");

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.terminals.size(), size_t{2});
    if (result.terminals.size() != 2) {
        return;
    }
    CHECK_TRUE(result.terminals[0].attributes.empty());
    CHECK_TRUE(result.terminals[1].attributes.empty());
    CHECK_TRUE(result.terminals[0].attributes.find(std::string{kTagsAttribute}) ==
               result.terminals[0].attributes.end());
}

TEST(OpControl, a_tag_needs_a_name) {
    const GenerationResult result = run("@start\n"
                                        "rule Main { tag(\"\"); extrude(1.0); }\n");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(has_error_saying(result, "'tag' needs a tag to add"));
}

// ============================================================================
// Determinism
// ============================================================================

TEST(OpControl, a_draw_does_not_depend_on_how_many_shapes_preceded_it) {
    // ---------------------------------------------------------------------
    // THE TEST THIS WHOLE DESIGN EXISTS FOR.
    //
    // `Watched` is the second child of Main either way. What changes between the
    // two runs is how many shapes the FIRST child made before it. A draw taken
    // from one shared stream moves with that count, and regenerating one block
    // would then require regenerating the city.
    //
    // The shape count is asserted to have changed FIRST, so that a perturbation
    // which silently did nothing cannot let the second assertion pass for the
    // wrong reason.
    // ---------------------------------------------------------------------
    const std::string source = "attr crowd : float = 1.0\n"
                               "@start\n"
                               "rule Main { Noise(crowd); Watched(); }\n"
                               "rule Noise(k : float) { if (k > 0.0) { Leaf(); Noise(k - 1.0); } }\n"
                               "rule Leaf { extrude(1.0); }\n"
                               "rule Watched { print(random.unit()); extrude(2.0); }\n";

    GenerationOptions few;
    few.seed = 1234;
    few.attributes["crowd"] = Value::number(1.0);

    GenerationOptions many;
    many.seed = 1234;
    many.attributes["crowd"] = Value::number(9.0);

    const GenerationResult thin = run(source, few);
    const GenerationResult fat = run(source, many);

    CHECK_TRUE(thin.ok());
    CHECK_TRUE(fat.ok());
    CHECK_TRUE(thin.stats.shapes_created < fat.stats.shapes_created);

    CHECK_EQ(thin.log.size(), size_t{1});
    CHECK_EQ(fat.log.size(), size_t{1});
    if (thin.log.empty() || fat.log.empty()) {
        return;
    }
    CHECK_EQ(thin.log[0], fat.log[0]);

    // And the drawn value is a real number in range, not an empty string two
    // runs agree on.
    const std::vector<double> value = logged_numbers(thin);
    CHECK_EQ(value.size(), size_t{1});
    if (!value.empty()) {
        CHECK(value[0] >= 0.0);
        CHECK(value[0] < 1.0);
    }
}

TEST(OpControl, the_same_seed_repeats_and_a_different_seed_does_not) {
    // The draw moves GEOMETRY, so it reaches the dump. A draw that only landed
    // in an attribute would make this test pass for an interpreter whose
    // geometry was non-deterministic.
    const std::string source = rows_of(12, "translate(random.range(0.0, 5.0), 0.0, 0.0);");

    GenerationOptions first;
    first.seed = 77;
    GenerationOptions second;
    second.seed = 78;

    const GenerationResult a = run(source, first);
    const GenerationResult b = run(source, first);
    const GenerationResult c = run(source, second);

    CHECK_TRUE(a.ok());
    CHECK_EQ(a.terminals.size(), size_t{13});  // 12 items and the tail of Row

    // Substantial before identical: two empty dumps also compare equal.
    CHECK(a.dump().size() > size_t{1000});
    CHECK_EQ(a.dump(), b.dump());
    CHECK_FALSE(a.dump() == c.dump());
}

TEST(OpControl, two_call_sites_in_one_rule_draw_independently) {
    // Pins the CALL SITE half of the mix. Drop loc.offset from the salt and both
    // lines print the same number, which is a window style and a door style that
    // are secretly the same coin.
    const GenerationResult result = run("@start\n"
                                        "rule Main {\n"
                                        "  print(random.unit());\n"
                                        "  print(random.unit());\n"
                                        "  extrude(1.0);\n"
                                        "}\n");

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.log.size(), size_t{2});
    if (result.log.size() == 2) {
        CHECK_FALSE(result.log[0] == result.log[1]);
    }
}

TEST(OpControl, siblings_of_one_rule_draw_independently) {
    // Pins the SHAPE ADDRESS half of the mix. Drop shape.seed_key from the salt
    // and every shape in the city draws the same number at this one line.
    const GenerationResult result = run("@start\n"
                                        "rule Main { Item(); Item(); Item(); }\n"
                                        "rule Item { print(random.unit()); extrude(1.0); }\n");

    CHECK_TRUE(result.ok());
    CHECK_EQ(result.log.size(), size_t{3});
    if (result.log.size() == 3) {
        CHECK_FALSE(result.log[0] == result.log[1]);
        CHECK_FALSE(result.log[1] == result.log[2]);
        CHECK_FALSE(result.log[0] == result.log[2]);
    }
}

TEST(OpControl, a_draw_is_spread_across_the_unit_interval) {
    // A draw that always returned 0.5 satisfies every bound check elsewhere in
    // this file and would make a street of houses a row of clones. Forty shapes
    // must reach both ends of the interval.
    const GenerationResult result = run(rows_of(40, "print(random.unit());"));
    CHECK_TRUE(result.ok());

    const std::vector<double> values = logged_numbers(result);
    CHECK_EQ(values.size(), size_t{40});
    if (values.size() != 40) {
        return;
    }
    double low = 2.0;
    double high = -1.0;
    for (const double value : values) {
        CHECK(value >= 0.0);
        CHECK(value < 1.0);
        low = std::min(low, value);
        high = std::max(high, value);
    }
    CHECK(low < 0.25);
    CHECK(high > 0.75);
}

// ============================================================================
// The stochastic library
// ============================================================================

TEST(OpControl, random_range_stays_inside_its_bounds_and_refuses_a_transposed_pair) {
    const GenerationResult result = run(rows_of(24, "print(random.range(2.5, 3.5));"));
    CHECK_TRUE(result.ok());

    const std::vector<double> values = logged_numbers(result);
    CHECK_EQ(values.size(), size_t{24});
    bool below_middle = false;
    bool above_middle = false;
    for (const double value : values) {
        CHECK(value >= 2.5);
        CHECK(value < 3.5);
        below_middle = below_middle || value < 3.0;
        above_middle = above_middle || value >= 3.0;
    }
    // Both halves are reached, so a bound check alone cannot pass for a constant.
    CHECK_TRUE(below_middle);
    CHECK_TRUE(above_middle);

    // A collapsed range is an ordinary computed result, not an error.
    const GenerationResult flat = run("@start\n"
                                      "rule Main { print(random.range(3.0, 3.0)); extrude(1.0); }\n");
    CHECK_TRUE(flat.ok());
    if (!flat.log.empty()) {
        CHECK_EQ(flat.log[0], std::string{"3"});
    }

    // A transposed pair is a transposition, not a request to swap.
    const GenerationResult backwards =
        run("@start\n"
            "rule Main { print(random.range(9.0, 1.0)); extrude(1.0); }\n");
    CHECK_FALSE(backwards.ok());
    CHECK_TRUE(has_error_saying(backwards, "random.range wants its low bound first"));
}

TEST(OpControl, random_integer_reaches_both_ends_and_leaves_neither) {
    const GenerationResult result = run(rows_of(40, "print(random.integer(1.0, 3.0));"));
    CHECK_TRUE(result.ok());

    const std::vector<double> values = logged_numbers(result);
    CHECK_EQ(values.size(), size_t{40});
    for (const double value : values) {
        CHECK(value >= 1.0);
        CHECK(value <= 3.0);
        CHECK_NEAR(value, std::nearbyint(value), 0.0);
    }
    // Both ends are inclusive, which is the whole difference from random.range.
    CHECK(count_lines(result, "1") > size_t{0});
    CHECK(count_lines(result, "3") > size_t{0});
    CHECK(count_lines(result, "2") > size_t{0});

    const GenerationResult backwards =
        run("@start\n"
            "rule Main { print(random.integer(4.0, 2.0)); extrude(1.0); }\n");
    CHECK_FALSE(backwards.ok());
    CHECK_TRUE(has_error_saying(backwards, "random.integer wants its low bound first"));
}

TEST(OpControl, random_chance_refuses_a_percentage_and_is_total_at_both_ends) {
    const GenerationResult percent =
        run("@start\n"
            "rule Main { if (random.chance(50.0)) { extrude(1.0); } }\n");
    CHECK_FALSE(percent.ok());
    CHECK_TRUE(has_error_saying(percent, "write 0.5 rather than 50 for half the time"));

    const GenerationResult never = run(rows_of(24, "print(random.chance(0.0));"));
    CHECK_TRUE(never.ok());
    CHECK_EQ(count_lines(never, "true"), size_t{0});
    CHECK_EQ(count_lines(never, "false"), size_t{24});

    const GenerationResult always = run(rows_of(24, "print(random.chance(1.0));"));
    CHECK_TRUE(always.ok());
    CHECK_EQ(count_lines(always, "true"), size_t{24});

    // And the control: in between, both answers occur. Without this, an
    // implementation that always returned false would pass the first two.
    const GenerationResult sometimes = run(rows_of(24, "print(random.chance(0.5));"));
    CHECK_TRUE(sometimes.ok());
    CHECK(count_lines(sometimes, "true") > size_t{0});
    CHECK(count_lines(sometimes, "false") > size_t{0});
}

TEST(OpControl, random_pick_returns_only_listed_values_and_refuses_an_empty_array) {
    const GenerationResult result =
        run(rows_of(32, "print(random.pick([\"brick\", \"render\", \"stone\"]));"));
    CHECK_TRUE(result.ok());
    CHECK_EQ(result.log.size(), size_t{32});

    const size_t brick = count_lines(result, "brick");
    const size_t render = count_lines(result, "render");
    const size_t stone = count_lines(result, "stone");
    CHECK_EQ(brick + render + stone, size_t{32});
    // Every option is reachable: a pick that always returned the first element
    // would satisfy the sum above.
    CHECK(brick > size_t{0});
    CHECK(render > size_t{0});
    CHECK(stone > size_t{0});

    const GenerationResult empty = run("@start\n"
                                       "rule Main { print(random.pick([])); extrude(1.0); }\n");
    CHECK_FALSE(empty.ok());
    CHECK_TRUE(has_error_saying(empty, "random.pick was given an empty array"));
}

TEST(OpControl, the_weighted_pick_holds_at_both_ends_of_the_draw) {
    // ---------------------------------------------------------------------
    // The rules that matter here hold at the BOUNDARIES of the draw, and a
    // draw derived from a shape's address cannot be steered to a boundary from
    // a rule file. This test was added because a mutation -- turning the
    // interval test from `<` into `<=` -- survived the rule-driven zero-weight
    // test below: a zero-width interval is only landed on by a draw of exactly
    // zero, which 32 sampled shapes will never produce. Logic a test cannot
    // reach is logic that is not tested.
    // ---------------------------------------------------------------------
    using stratum::procgen::rules::weighted_index;

    const double almost_one = std::nextafter(1.0, 0.0);

    // A zero weight is not chosen even by the draw that lands exactly on its
    // zero-width interval.
    CHECK_EQ(weighted_index({0.0, 1.0}, 0.0), size_t{1});
    CHECK_EQ(weighted_index({0.0, 1.0}, almost_one), size_t{1});

    // Nor is a TRAILING zero weight chosen by the rounding path, which is the
    // one an implementation reaches by falling out of the scan.
    CHECK_EQ(weighted_index({1.0, 0.0}, 0.0), size_t{0});
    CHECK_EQ(weighted_index({1.0, 0.0}, almost_one), size_t{0});
    CHECK_EQ(weighted_index({1.0, 0.0, 0.0}, almost_one), size_t{0});

    // Equal weights split the interval in half, at the half.
    CHECK_EQ(weighted_index({1.0, 1.0}, 0.0), size_t{0});
    CHECK_EQ(weighted_index({1.0, 1.0}, 0.499999), size_t{0});
    CHECK_EQ(weighted_index({1.0, 1.0}, 0.5), size_t{1});
    CHECK_EQ(weighted_index({1.0, 1.0}, almost_one), size_t{1});

    // Weights are relative: the same proportions written larger choose the same.
    CHECK_EQ(weighted_index({3.0, 1.0}, 0.7), weighted_index({75.0, 25.0}, 0.7));
    CHECK_EQ(weighted_index({3.0, 1.0}, 0.8), size_t{1});
    CHECK_EQ(weighted_index({3.0, 1.0}, 0.74), size_t{0});

    // A draw at or above 1 falls off the end of the scan. seed_unit() never
    // produces one, so this is the caller's promise rather than this function's
    // -- but the line that handles it is only worth having if something checks
    // it, and the wrong answer here is a zero-weight entry.
    CHECK_EQ(weighted_index({1.0, 0.0}, 1.0), size_t{0});
    CHECK_EQ(weighted_index({0.0, 2.0, 0.0, 3.0}, 1.0), size_t{3});
    CHECK_EQ(weighted_index({0.0, 2.0, 3.0, 0.0}, 4.0), size_t{2});

    // A NEGATIVE or NaN weight, which the caller refuses before it gets here
    // and which this function skips rather than trusting. The skip is in two
    // places -- the total and the scan -- and each can be deleted on its own:
    //
    //   - out of the TOTAL, a negative weight makes the sum smaller than the
    //     positive weights it competes with, or NaN, and the function then
    //     answers "nothing positive to choose" for a list that plainly has
    //     some;
    //   - out of the SCAN, its interval shifts every later boundary and the
    //     answer falls through to the last positive entry.
    //
    // Both need the bad weight FIRST, where a low draw would land in its
    // interval, and a list of three, so that falling through to the last
    // positive entry is a DIFFERENT answer from the right one.
    const double nan_weight = std::numeric_limits<double>::quiet_NaN();
    CHECK_EQ(weighted_index({-5.0, 1.0, 1.0}, 0.4), size_t{1});
    CHECK_EQ(weighted_index({nan_weight, 1.0, 1.0}, 0.4), size_t{1});
    CHECK_EQ(weighted_index({-5.0, 1.0, 1.0}, 0.9), size_t{2});
    CHECK_EQ(weighted_index({nan_weight, 1.0}, 0.5), size_t{1});
    // A trailing negative is not the answer on the rounding path either.
    CHECK_EQ(weighted_index({1.0, -5.0}, 1.0), size_t{0});
    // And a list with nothing positive in it is refused whatever the sign.
    CHECK_EQ(weighted_index({-1.0, -2.0}, 0.5), size_t{2});
    CHECK_EQ(weighted_index({nan_weight}, 0.5), size_t{1});

    // Nothing positive to choose: the caller turns this into its own message
    // rather than dividing by zero.
    CHECK_EQ(weighted_index({}, 0.5), size_t{0});
    CHECK_EQ(weighted_index({0.0, 0.0}, 0.5), size_t{2});

    // A whole sweep of the interval never reaches a zero-weight entry, and
    // reaches both of the entries that have one.
    bool saw_one = false;
    bool saw_three = false;
    for (int step = 0; step < 1000; ++step) {
        const double unit = static_cast<double>(step) / 1000.0;
        const size_t index = weighted_index({0.0, 2.0, 0.0, 3.0}, unit);
        CHECK(index == size_t{1} || index == size_t{3});
        saw_one = saw_one || index == size_t{1};
        saw_three = saw_three || index == size_t{3};
    }
    CHECK_TRUE(saw_one);
    CHECK_TRUE(saw_three);
}

TEST(OpControl, a_zero_weight_is_never_chosen_and_equal_weights_reach_both) {
    // The zero-weight rule, with the control that makes it able to fail: an
    // implementation that always returned the last value would pass the first
    // half of this test and fail the second.
    const GenerationResult zeroed =
        run(rows_of(32, "print(random.weighted([\"never\", \"always\"], [0.0, 1.0]));"));
    CHECK_TRUE(zeroed.ok());
    CHECK_EQ(count_lines(zeroed, "never"), size_t{0});
    CHECK_EQ(count_lines(zeroed, "always"), size_t{32});

    // The other way round, so the rule is not "the second one always wins".
    const GenerationResult flipped =
        run(rows_of(32, "print(random.weighted([\"always\", \"never\"], [1.0, 0.0]));"));
    CHECK_TRUE(flipped.ok());
    CHECK_EQ(count_lines(flipped, "never"), size_t{0});
    CHECK_EQ(count_lines(flipped, "always"), size_t{32});

    const GenerationResult even =
        run(rows_of(32, "print(random.weighted([\"left\", \"right\"], [1.0, 1.0]));"));
    CHECK_TRUE(even.ok());
    CHECK(count_lines(even, "left") > size_t{0});
    CHECK(count_lines(even, "right") > size_t{0});
}

TEST(OpControl, weights_are_relative_and_are_honoured_in_proportion) {
    // Three to one, over sixty shapes. The bounds are wide because the point is
    // that the weights are USED, not that the sample is exact -- but they
    // exclude both degenerate answers, which an implementation that ignored the
    // weights could not do.
    const GenerationResult result =
        run(rows_of(60, "print(random.weighted([\"common\", \"rare\"], [3.0, 1.0]));"));
    CHECK_TRUE(result.ok());

    const size_t common = count_lines(result, "common");
    const size_t rare = count_lines(result, "rare");
    CHECK_EQ(common + rare, size_t{60});
    CHECK(common > size_t{33});  // well above the half an ignored weight would give
    CHECK(common < size_t{57});  // and well below all of them
    CHECK(rare > size_t{0});

    // The same proportions written larger mean the same thing: weights are
    // relative, so there is no error class for "the percentages are wrong".
    const GenerationResult scaled =
        run(rows_of(60, "print(random.weighted([\"common\", \"rare\"], [75.0, 25.0]));"));
    CHECK_TRUE(scaled.ok());
    CHECK_EQ(count_lines(scaled, "common"), common);
}

TEST(OpControl, random_weighted_refuses_every_way_a_weight_list_can_be_wrong) {
    const GenerationResult negative =
        run("@start\n"
            "rule Main { print(random.weighted([\"a\", \"b\"], [1.0, 0.0 - 1.0])); extrude(1.0); }\n");
    CHECK_FALSE(negative.ok());
    CHECK_TRUE(has_error_saying(negative, "a weight cannot be negative, and weight 2 is -1"));

    const GenerationResult all_zero =
        run("@start\n"
            "rule Main { print(random.weighted([\"a\", \"b\"], [0.0, 0.0])); extrude(1.0); }\n");
    CHECK_FALSE(all_zero.ok());
    CHECK_TRUE(has_error_saying(all_zero, "every weight given to random.weighted is zero"));

    const GenerationResult mismatched =
        run("@start\n"
            "rule Main { print(random.weighted([\"a\", \"b\"], [1.0])); extrude(1.0); }\n");
    CHECK_FALSE(mismatched.ok());
    CHECK_TRUE(has_error_saying(mismatched, "2 values and 1 weights"));

    // The mismatch the OTHER way round, which the one above cannot catch:
    // weaken the length test from `!=` to `>` and it still refuses two values
    // with one weight, while a list with more weights than values reaches
    // weighted_index(), which can choose an index past the end of the values.
    // The caller then reports that as an all-zero weight list -- the one thing
    // [1.0, 1.0] is not -- so the message is asserted to be the length one AND
    // asserted not to be the all-zero one.
    const GenerationResult surplus =
        run("@start\n"
            "rule Main { print(random.weighted([\"a\"], [1.0, 1.0])); extrude(1.0); }\n");
    CHECK_FALSE(surplus.ok());
    CHECK_TRUE(has_error_saying(surplus, "1 values and 2 weights"));
    CHECK_FALSE(has_error_saying(surplus, "every weight given to random.weighted is zero"));
    CHECK_TRUE(surplus.terminals.empty());

    const GenerationResult not_a_number =
        run("@start\n"
            "rule Main { print(random.weighted([\"a\"], [\"heavy\"])); extrude(1.0); }\n");
    CHECK_FALSE(not_a_number.ok());
    CHECK_TRUE(has_error_saying(not_a_number, "weight 1 must be a number, not a string"));
}

TEST(OpControl, an_expression_function_checks_the_arity_the_parser_does_not) {
    // ast.hpp resolves a STATEMENT callee against its catalogue and checks the
    // count at parse time. An expression callee is resolved here and nowhere
    // else, so a wrong count has to be caught at run time or not at all.
    const GenerationResult result = run("@start\n"
                                        "rule Main { print(random.unit(3.0)); extrude(1.0); }\n");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(has_error_saying(result, "'random.unit' takes 0 arguments but was given 1"));
}

// ============================================================================
// Conditionals over the new vocabulary
// ============================================================================

TEST(OpControl, a_condition_branches_on_a_scope_query_and_an_attribute_together) {
    // What the feature is for: "if the lot is deeper than 20 metres, add a rear
    // wing". The two lots differ only in the seed shape, so a condition that
    // ignored the geometry would give both the same answer.
    const std::string source = "attr min_depth : float = 20.0\n"
                               "@start\n"
                               "rule Lot {\n"
                               "  if (shape.sz > min_depth) {\n"
                               "    set(\"plan\", \"wing\");\n"
                               "  } else {\n"
                               "    set(\"plan\", \"block\");\n"
                               "  }\n"
                               "  print(attrs.get(\"plan\"));\n"
                               "  extrude(3.0);\n"
                               "}\n";

    const ParseResult parsed = parse(source, "test.srl");
    CHECK_EQ(parsed.render_all(source), std::string{});

    const GenerationResult shallow =
        generate(parsed.file, shape_from_rect(10.0, 12.0), {}, &control_operations(),
                 &control_functions());
    const GenerationResult deep =
        generate(parsed.file, shape_from_rect(10.0, 30.0), {}, &control_operations(),
                 &control_functions());

    CHECK_TRUE(shallow.ok());
    CHECK_TRUE(deep.ok());
    CHECK_EQ(shallow.log.size(), size_t{1});
    CHECK_EQ(deep.log.size(), size_t{1});
    if (!shallow.log.empty()) {
        CHECK_EQ(shallow.log[0], std::string{"block"});
    }
    if (!deep.log.empty()) {
        CHECK_EQ(deep.log[0], std::string{"wing"});
    }

    // And the threshold is the attribute, not a constant baked into the rule: a
    // supplied value moves the branch.
    GenerationOptions lowered;
    lowered.attributes["min_depth"] = Value::number(5.0);
    const GenerationResult moved =
        generate(parsed.file, shape_from_rect(10.0, 12.0), lowered, &control_operations(),
                 &control_functions());
    CHECK_TRUE(moved.ok());
    if (!moved.log.empty()) {
        CHECK_EQ(moved.log[0], std::string{"wing"});
    }
}

TEST(OpControl, a_choose_arm_and_a_draw_agree_with_themselves_across_runs) {
    // The statement form and the expression form of the same idea, in one rule,
    // both driven from the shape's address. Two runs of the same seed must give
    // the same street.
    const std::string source =
        rows_of(20,
                "choose { 2.0: { tag(\"tall\"); } 1.0: { tag(\"short\"); } }"
                " set(\"height\", random.range(3.0, 9.0));"
                " print(attrs.get(\"height\"));");

    GenerationOptions options;
    options.seed = 4242;

    const GenerationResult first = run(source, options);
    const GenerationResult second = run(source, options);

    CHECK_TRUE(first.ok());
    CHECK_EQ(first.log.size(), size_t{20});
    CHECK_TRUE(first.log == second.log);

    // Both arms were taken by somebody, so the choose is choosing.
    size_t tall = 0;
    size_t shortish = 0;
    for (const Shape& terminal : first.terminals) {
        tall += shape_has_tag(terminal, "tall") ? 1u : 0u;
        shortish += shape_has_tag(terminal, "short") ? 1u : 0u;
    }
    CHECK(tall > size_t{0});
    CHECK(shortish > size_t{0});

    // And the drawn heights are not all the same number.
    const std::vector<double> heights = logged_numbers(first);
    CHECK_EQ(heights.size(), size_t{20});
    if (heights.size() == 20) {
        bool varied = false;
        for (const double height : heights) {
            CHECK(height >= 3.0);
            CHECK(height < 9.0);
            varied = varied || height != heights[0];
        }
        CHECK_TRUE(varied);
    }
}

// ============================================================================
// Literals and the declared schema
// ============================================================================

TEST(OpControl, literal_value_reads_what_is_written_out_and_refuses_what_is_computed) {
    const ParseResult parsed = parse_clean("attr plain : float = 12.5\n"
                                           "attr negative : float = -3.0\n"
                                           "attr word : string = \"brick\"\n"
                                           "attr flag : bool = false\n"
                                           "attr list : float[] = [1.0, 2.0, -3.0]\n"
                                           "attr computed : float = 2.0 * 3.0\n"
                                           "attr borrowed : float = plain\n"
                                           "@start\n"
                                           "rule Main { extrude(1.0); }\n");
    CHECK_EQ(parsed.file.attributes.size(), size_t{7});
    if (parsed.file.attributes.size() != 7) {
        return;
    }

    Value value;
    CHECK_TRUE(literal_value(parsed.file, parsed.file.attributes[0].default_value, value));
    CHECK_TRUE(value.is_number());
    CHECK_NEAR(value.as_number(), 12.5, 0.0);

    // `-3.0` is a unary minus over a number, never a signed literal. Without
    // that case every negative default would look computed.
    CHECK_TRUE(literal_value(parsed.file, parsed.file.attributes[1].default_value, value));
    CHECK_TRUE(value.is_number());
    CHECK_NEAR(value.as_number(), -3.0, 0.0);

    CHECK_TRUE(literal_value(parsed.file, parsed.file.attributes[2].default_value, value));
    CHECK_TRUE(value.is_text());
    CHECK_EQ(value.as_text(), std::string{"brick"});

    CHECK_TRUE(literal_value(parsed.file, parsed.file.attributes[3].default_value, value));
    CHECK_TRUE(value.is_bool());
    CHECK_FALSE(value.as_bool());

    CHECK_TRUE(literal_value(parsed.file, parsed.file.attributes[4].default_value, value));
    CHECK_TRUE(value.is_array());
    if (value.is_array()) {
        CHECK_EQ(value.as_array().size(), size_t{3});
        if (value.as_array().size() == 3) {
            CHECK_NEAR(value.as_array()[2].as_number(), -3.0, 0.0);
        }
    }

    CHECK_FALSE(literal_value(parsed.file, parsed.file.attributes[5].default_value, value));
    CHECK_FALSE(literal_value(parsed.file, parsed.file.attributes[6].default_value, value));

    // A missing node is not a literal, and does not read one node past the end.
    CHECK_FALSE(literal_value(parsed.file, stratum::procgen::rules::kNoNode, value));
}

TEST(OpControl, the_schema_decodes_every_annotation_an_inspector_needs) {
    const ParseResult parsed =
        parse_clean("@description(\"Height above the pavement\")\n"
                    "@unit(\"m\")\n"
                    "@range(3.0, 60.0)\n"
                    "@step(0.5)\n"
                    "@order(2.0)\n"
                    "@group(\"Massing\", \"Height\")\n"
                    "attr height : float = 12.0\n"
                    "@enum(\"brick\", \"render\", \"stone\")\n"
                    "attr wall : string = \"brick\"\n"
                    "@hidden\n"
                    "attr derived : float = 1.0\n"
                    "@color\n"
                    "attr tint : string = \"#808080\"\n"
                    "@asset(\"*.obj\")\n"
                    "attr model : string = \"tree.obj\"\n"
                    "@start\n"
                    "rule Main { extrude(1.0); }\n");

    const AttributeSchema schema = attribute_schema(parsed.file);
    CHECK_TRUE(schema.ok());
    CHECK_EQ(schema.diagnostics.size(), size_t{0});
    CHECK_EQ(schema.attributes.size(), size_t{5});

    const AttrInfo* height = schema.find("height");
    CHECK_TRUE(height != nullptr);
    if (height != nullptr) {
        CHECK_EQ(height->description, std::string{"Height above the pavement"});
        CHECK_EQ(height->unit, std::string{"m"});
        CHECK_TRUE(height->has_range);
        CHECK_NEAR(height->range_min, 3.0, 0.0);
        CHECK_NEAR(height->range_max, 60.0, 0.0);
        CHECK_TRUE(height->has_step);
        CHECK_NEAR(height->step, 0.5, 0.0);
        CHECK_TRUE(height->has_order);
        CHECK_NEAR(height->order, 2.0, 0.0);
        CHECK_EQ(height->group.size(), size_t{2});
        if (height->group.size() == 2) {
            CHECK_EQ(height->group[0], std::string{"Massing"});
            CHECK_EQ(height->group[1], std::string{"Height"});
        }
        CHECK_TRUE(height->default_is_literal);
        CHECK_NEAR(height->default_value.as_number(), 12.0, 0.0);
        CHECK_FALSE(height->hidden);
        CHECK_TRUE(height->type.type == PrimitiveType::Float);
    }

    const AttrInfo* wall = schema.find("wall");
    CHECK_TRUE(wall != nullptr);
    if (wall != nullptr) {
        CHECK_EQ(wall->enum_values.size(), size_t{3});
        if (wall->enum_values.size() == 3) {
            CHECK_EQ(wall->enum_values[1].as_text(), std::string{"render"});
        }
    }

    const AttrInfo* derived = schema.find("derived");
    CHECK_TRUE(derived != nullptr);
    if (derived != nullptr) {
        CHECK_TRUE(derived->hidden);
        CHECK_FALSE(derived->has_range);
    }

    const AttrInfo* tint = schema.find("tint");
    CHECK_TRUE(tint != nullptr);
    if (tint != nullptr) {
        CHECK_TRUE(tint->is_color);
    }

    const AttrInfo* model = schema.find("model");
    CHECK_TRUE(model != nullptr);
    if (model != nullptr) {
        CHECK_TRUE(model->is_asset);
        CHECK_EQ(model->asset_filter, std::string{"*.obj"});
    }

    CHECK_TRUE(schema.find("nothing") == nullptr);
}

TEST(OpControl, an_annotation_the_parser_counted_but_cannot_be_used_is_reported) {
    // ast.hpp: "a silently ignored @rnage(0, 10) looks exactly like a working
    // one until someone opens the panel and finds a free-text box". The parser
    // checks the COUNT of the arguments and nothing else, so this is where the
    // rest is caught. Each complaint names the attribute or the annotation, and
    // the attribute list is produced anyway -- one unusable slider must not cost
    // the Inspector the other four.
    const ParseResult parsed = parse_clean("@range(60.0, 3.0)\n"
                                           "attr crossed : float = 12.0\n"
                                           "@range(\"low\", \"high\")\n"
                                           "attr wordy : float = 12.0\n"
                                           "@step(0.0)\n"
                                           "attr flat : float = 12.0\n"
                                           "@range(0.0, 10.0)\n"
                                           "attr outside : float = 40.0\n"
                                           "@enum(\"brick\", \"stone\")\n"
                                           "attr wall : string = \"glass\"\n"
                                           "@range(0.0, 10.0)\n"
                                           "attr worded : string = \"x\"\n"
                                           "@order(1.0)\n"
                                           "@order(2.0)\n"
                                           "attr twice : float = 1.0\n"
                                           "attr mistyped : float = \"tall\"\n"
                                           "@enum(1.0, 2.0)\n"
                                           "attr kinds : string = \"brick\"\n"
                                           "@enum(\"brick\", 1.0 + 1.0)\n"
                                           "attr computed : string = \"brick\"\n"
                                           "@asset(\"*.obj\")\n"
                                           "attr count : float = 1.0\n"
                                           "@group(1.0)\n"
                                           "attr grouped : float = 1.0\n"
                                           "@start\n"
                                           "rule Main { extrude(1.0); }\n");

    const AttributeSchema schema = attribute_schema(parsed.file);
    CHECK_FALSE(schema.ok());
    CHECK_EQ(schema.attributes.size(), size_t{12});

    CHECK_TRUE(has_diagnostic_saying(schema.diagnostics, "wants a minimum below its maximum"));
    CHECK_TRUE(has_diagnostic_saying(schema.diagnostics, "wants a plain number for argument 1"));
    CHECK_TRUE(has_diagnostic_saying(schema.diagnostics, "would let the slider take no positions"));
    CHECK_TRUE(has_diagnostic_saying(schema.diagnostics, "which is outside its own @range"));
    CHECK_TRUE(has_diagnostic_saying(schema.diagnostics, "which its own @enum does not list"));
    CHECK_TRUE(has_diagnostic_saying(schema.diagnostics, "only meaningful on a float attribute"));
    CHECK_TRUE(has_diagnostic_saying(schema.diagnostics, "is given twice on this attribute"));
    CHECK_TRUE(has_diagnostic_saying(schema.diagnostics,
                                     "'mistyped' is declared float but its default is a string"));

    // The three checks nothing used to assert. Each is one `break` away from
    // being deleted, and the first is the one op_control.hpp names as a
    // diagnostic this decoder promises: "an @enum value of the wrong type".
    // Each fragment names its own annotation, so one of them going missing
    // cannot be covered by another's message.
    CHECK_TRUE(has_diagnostic_saying(schema.diagnostics,
                                     "'@enum' on 'kinds' wants string values"));
    CHECK_TRUE(has_diagnostic_saying(schema.diagnostics,
                                     "'@enum' wants plain values it can list"));
    CHECK_TRUE(has_diagnostic_saying(
        schema.diagnostics, "'@asset' names a file, so it is only meaningful on a string"));
    CHECK_TRUE(has_diagnostic_saying(schema.diagnostics,
                                     "'@group' wants a quoted string for argument 1"));

    // Every diagnostic carries a position, which is the whole point of a
    // Diagnostic rather than a string.
    for (const Diagnostic& diagnostic : schema.diagnostics) {
        CHECK(diagnostic.loc.line >= 1u);
        CHECK_FALSE(diagnostic.message.empty());
    }

    // The first of a repeated annotation is the one kept, so the Inspector shows
    // something rather than nothing.
    const AttrInfo* twice = schema.find("twice");
    CHECK_TRUE(twice != nullptr);
    if (twice != nullptr) {
        CHECK_TRUE(twice->has_order);
        CHECK_NEAR(twice->order, 1.0, 0.0);
    }

    // A crossed range leaves no range at all rather than a reversed one that an
    // Inspector would draw backwards.
    const AttrInfo* crossed = schema.find("crossed");
    CHECK_TRUE(crossed != nullptr);
    if (crossed != nullptr) {
        CHECK_FALSE(crossed->has_range);
    }

    // And a refused annotation leaves NOTHING decoded, which is the second half
    // of each of the three checks above: a complaint that still kept the value
    // would give the Inspector a dropdown of numbers on a string attribute, a
    // file picker on a float, and a group path with a number in it. Asserting
    // the message alone would pass for all three.
    const AttrInfo* kinds = schema.find("kinds");
    CHECK_TRUE(kinds != nullptr);
    if (kinds != nullptr) {
        CHECK_TRUE(kinds->enum_values.empty());
    }
    const AttrInfo* computed = schema.find("computed");
    CHECK_TRUE(computed != nullptr);
    if (computed != nullptr) {
        CHECK_TRUE(computed->enum_values.empty());
    }
    const AttrInfo* count = schema.find("count");
    CHECK_TRUE(count != nullptr);
    if (count != nullptr) {
        CHECK_FALSE(count->is_asset);
    }
    const AttrInfo* grouped = schema.find("grouped");
    CHECK_TRUE(grouped != nullptr);
    if (grouped != nullptr) {
        CHECK_TRUE(grouped->group.empty());
    }
}

TEST(OpControl, a_file_with_no_attributes_has_an_empty_schema_and_no_complaints) {
    const ParseResult parsed = parse_clean("@start\nrule Main { extrude(1.0); }\n");

    const AttributeSchema schema = attribute_schema(parsed.file);
    CHECK_TRUE(schema.ok());
    CHECK_TRUE(schema.attributes.empty());
    CHECK_TRUE(schema.diagnostics.empty());
}

// ============================================================================
// The extension point
// ============================================================================

TEST(OpControl, the_family_registers_without_the_interpreter_knowing_about_it) {
    // If this stops holding, adding an operation family means editing
    // interpreter.cpp, and three families cannot land in one tree at once.
    CHECK_EQ(control_operations().size(), standard_operations().size() + size_t{3});
    CHECK_EQ(control_functions().size(), standard_functions().size() + size_t{13});

    // D2's operations still work beside D6's in one rule.
    const GenerationResult result =
        run("@start\n"
            "rule Main { tag(\"lot\"); extrude(3.0); print(geometry.volume()); }\n");
    CHECK_TRUE(result.ok());
    CHECK_EQ(result.log.size(), size_t{1});
    if (!result.log.empty()) {
        CHECK_EQ(result.log[0], std::string{"3"});
    }
}

TEST(OpControl, another_family_can_register_into_the_same_two_tables) {
    // The composition three concurrent features need: each adds to a table it
    // did not build, in either order, without knowing what the others put there.
    OperationTable operations = standard_operations();
    int roofs = 0;
    operations.register_operation("roof", [&roofs](OperationArgs& context) {
        ++roofs;
        context.interpreter.log("roof");
    });
    register_control_operations(operations);

    FunctionTable functions = standard_functions();
    functions.register_function("storeys", [](const stratum::procgen::rules::FunctionArgs& args) {
        return Value::number(std::floor(args.shape.scope.size.y / 3.0));
    });
    register_control_functions(functions);

    const ParseResult parsed = parse_clean("@start\n"
                                           "rule Main {\n"
                                           "  extrude(9.0);\n"
                                           "  set(\"storeys\", storeys());\n"
                                           "  roof(\"gable\");\n"
                                           "  print(attrs.get(\"storeys\"));\n"
                                           "}\n");

    const GenerationResult result =
        generate(parsed.file, shape_from_rect(1.0, 1.0), {}, &operations, &functions);

    CHECK_TRUE(result.ok());
    CHECK_EQ(roofs, 1);
    CHECK_EQ(result.log.size(), size_t{2});
    if (result.log.size() == 2) {
        CHECK_EQ(result.log[0], std::string{"roof"});
        CHECK_EQ(result.log[1], std::string{"3"});
    }
}

TEST(OpControl, the_operations_are_absent_from_the_standard_tables) {
    // The other side of the same claim: D6's operations are not in D2's table,
    // so a caller who did not ask for them gets the interpreter's "not
    // implemented in this build" message rather than silence.
    CHECK_TRUE(standard_operations().find("set") == nullptr);
    CHECK_TRUE(standard_operations().find("tag") == nullptr);
    CHECK_TRUE(standard_functions().find("random.unit") == nullptr);

    const ParseResult parsed = parse_clean("@start\n"
                                           "rule Main { set(\"a\", 1.0); extrude(1.0); }\n");
    const GenerationResult result = generate(parsed.file, shape_from_rect(1.0, 1.0), {});
    CHECK_FALSE(result.ok());
    CHECK_TRUE(has_error_saying(result, "operation 'set' is not implemented in this build"));
}

} // namespace
