// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_rule_parser.cpp
 * @brief The rule language: tokens, the grammar, the checked AST, and the error messages
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Three kinds of test live here, and the middle kind is the one that matters.
 *
 * **Shape.** Did the parser build the tree the source describes? These are written
 * so that a plausible bug CHANGES the assertion. The split tests use the SAME
 * numeric literal for all four size kinds, so a parser that dropped `~` or `%`
 * cannot pass by producing the right number. The associativity tests use `-` and
 * `/`, never `+` or `*`, because a right-associative bug in a commuting operator
 * is invisible. The order tests use three DIFFERENT operations, because a
 * one-element container proves nothing about order.
 *
 * **Messages.** An error path with no test says "parse error" on line 1 forever.
 * So the exact text, line, column and caret of a diagnostic are asserted here, and
 * so are the spelling suggestions. They are a feature; they are tested like one.
 *
 * **Determinism.** Two parses of the same text, and two parses of the same text
 * with different whitespace and comments, must dump identically. dump() carries no
 * locations for exactly that reason.
 *
 * Written against the headers' guarantees. Nothing here reaches into how the
 * parser recovers or how the arenas are laid out, only into what ast.hpp promises.
 */

#include "framework.hpp"

#include "procgen/rules/ast.hpp"
#include "procgen/rules/lexer.hpp"
#include "procgen/rules/parser.hpp"

#include <cstddef>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

using stratum::procgen::rules::Annotation;
using stratum::procgen::rules::AnnotationKind;
using stratum::procgen::rules::CallStmt;
using stratum::procgen::rules::CallTarget;
using stratum::procgen::rules::call_target_name;
using stratum::procgen::rules::ChooseStmt;
using stratum::procgen::rules::component_domain_name;
using stratum::procgen::rules::component_selector_name;
using stratum::procgen::rules::Diagnostic;
using stratum::procgen::rules::dump;
using stratum::procgen::rules::IfStmt;
using stratum::procgen::rules::kMaxDiagnostics;
using stratum::procgen::rules::kNoNode;
using stratum::procgen::rules::LetStmt;
using stratum::procgen::rules::LexResult;
using stratum::procgen::rules::parse;
using stratum::procgen::rules::ParseResult;
using stratum::procgen::rules::primitive_type_name;
using stratum::procgen::rules::render_diagnostic;
using stratum::procgen::rules::RuleFile;
using stratum::procgen::rules::SelectStmt;
using stratum::procgen::rules::Severity;
using stratum::procgen::rules::size_kind_name;
using stratum::procgen::rules::SizeKind;
using stratum::procgen::rules::split_axis_name;
using stratum::procgen::rules::SplitStmt;
using stratum::procgen::rules::Stmt;
using stratum::procgen::rules::Token;
using stratum::procgen::rules::TokenKind;
using stratum::procgen::rules::tokenize;

namespace {

// ============================================================================
// Helpers
// ============================================================================

/// Parse, and turn a failure into a readable one by printing the rendered errors
ParseResult must_parse(const std::string& source) {
    ParseResult result = parse(source, "test.srl");
    if (!result.ok()) {
        // Comparing against the empty string is how the rendered diagnostics get
        // into the failure output; the framework has no message argument.
        CHECK_EQ(result.render_all(source), std::string{});
    }
    return result;
}

size_t error_count(const ParseResult& result) {
    size_t count = 0;
    for (const Diagnostic& diagnostic : result.diagnostics) {
        if (diagnostic.severity == Severity::Error) {
            ++count;
        }
    }
    return count;
}

/// Message of the n-th error, or a marker that reads clearly in a failure
std::string error_message(const ParseResult& result, size_t index) {
    size_t seen = 0;
    for (const Diagnostic& diagnostic : result.diagnostics) {
        if (diagnostic.severity != Severity::Error) {
            continue;
        }
        if (seen == index) {
            return diagnostic.message;
        }
        ++seen;
    }
    return "<no error at index " + std::to_string(index) + ">";
}

const Diagnostic* first_error(const ParseResult& result) {
    for (const Diagnostic& diagnostic : result.diagnostics) {
        if (diagnostic.severity == Severity::Error) {
            return &diagnostic;
        }
    }
    return nullptr;
}

bool contains(std::string_view haystack, std::string_view needle) {
    return haystack.find(needle) != std::string_view::npos;
}

/// How many diagnostics carry the cap sentence. The cap must appear exactly
/// once however the flood was produced -- two producers each capping their own
/// list would say it twice, which is how a merged-list bug would look.
size_t cap_message_count(const std::vector<Diagnostic>& diagnostics) {
    size_t count = 0;
    for (const Diagnostic& diagnostic : diagnostics) {
        if (diagnostic.message.find("too many errors") != std::string::npos) {
            ++count;
        }
    }
    return count;
}

/// True when SOME error message contains @p needle. Used where the position of
/// the message among several is not the property under test.
bool any_error_contains(const ParseResult& result, std::string_view needle) {
    for (const Diagnostic& diagnostic : result.diagnostics) {
        if (diagnostic.severity == Severity::Error && contains(diagnostic.message, needle)) {
            return true;
        }
    }
    return false;
}

template <typename T>
const T* find_first(const RuleFile& file) {
    for (const Stmt& statement : file.stmts) {
        if (const T* found = std::get_if<T>(&statement.node)) {
            return found;
        }
    }
    return nullptr;
}

const CallStmt* find_call(const RuleFile& file, std::string_view name) {
    for (const Stmt& statement : file.stmts) {
        const CallStmt* call = std::get_if<CallStmt>(&statement.node);
        if (call != nullptr && call->callee.name == name) {
            return call;
        }
    }
    return nullptr;
}

std::string kind_name(SizeKind kind) { return std::string{size_kind_name(kind)}; }
std::string target_name(CallTarget target) { return std::string{call_target_name(target)}; }

/// The expression sub-tree of the split entry at @p index, as canonical text.
/// Reaching it through dump() keeps the tests off the arena layout.
std::string dump_of(const std::string& source) {
    return dump(must_parse(source).file);
}

} // namespace

// ============================================================================
// Lexer
// ============================================================================

TEST(RuleParser, lexer_produces_the_expected_token_kinds_in_order) {
    const LexResult lexed = tokenize("rule R { trim(); }");
    CHECK_TRUE(lexed.ok());
    CHECK_EQ(lexed.tokens.size(), size_t{9});
    CHECK_TRUE(lexed.tokens[0].kind == TokenKind::KwRule);
    CHECK_TRUE(lexed.tokens[1].kind == TokenKind::Identifier);
    CHECK_TRUE(lexed.tokens[2].kind == TokenKind::LBrace);
    CHECK_TRUE(lexed.tokens[3].kind == TokenKind::Identifier);
    CHECK_TRUE(lexed.tokens[4].kind == TokenKind::LParen);
    CHECK_TRUE(lexed.tokens[5].kind == TokenKind::RParen);
    CHECK_TRUE(lexed.tokens[6].kind == TokenKind::Semicolon);
    CHECK_TRUE(lexed.tokens[7].kind == TokenKind::RBrace);
    CHECK_TRUE(lexed.tokens[8].kind == TokenKind::End);
}

TEST(RuleParser, lexer_records_the_line_and_column_of_every_token) {
    // Positions are asserted one by one rather than "the file has 9 tokens":
    // an off-by-one in the column only shows up as a caret under the wrong
    // character, which no count can detect.
    const LexResult lexed = tokenize("rule R {\n  trim();\n}\n");
    CHECK_EQ(lexed.tokens.size(), size_t{9});

    CHECK_EQ(lexed.tokens[0].loc.line, 1u);
    CHECK_EQ(lexed.tokens[0].loc.column, 1u);
    CHECK_EQ(lexed.tokens[1].loc.column, 6u);  // R
    CHECK_EQ(lexed.tokens[2].loc.column, 8u);  // {

    CHECK_EQ(lexed.tokens[3].loc.line, 2u);    // trim
    CHECK_EQ(lexed.tokens[3].loc.column, 3u);
    CHECK_EQ(lexed.tokens[3].loc.offset, 11u);
    CHECK_EQ(lexed.tokens[4].loc.column, 7u);  // (
    CHECK_EQ(lexed.tokens[5].loc.column, 8u);  // )
    CHECK_EQ(lexed.tokens[6].loc.column, 9u);  // ;

    CHECK_EQ(lexed.tokens[7].loc.line, 3u);    // }
    CHECK_EQ(lexed.tokens[7].loc.column, 1u);

    CHECK_EQ(lexed.tokens[8].loc.line, 4u);    // end of file
    CHECK_EQ(lexed.tokens[8].loc.column, 1u);
}

TEST(RuleParser, lexer_records_the_length_of_a_token) {
    const LexResult lexed = tokenize("extrude <= 12.5");
    CHECK_EQ(lexed.tokens[0].length, 7u);
    CHECK_EQ(lexed.tokens[1].length, 2u);
    CHECK_EQ(lexed.tokens[2].length, 4u);
}

TEST(RuleParser, lexer_reads_numbers_including_exponents) {
    const LexResult lexed = tokenize("12 3.25 1e3 2.5e-2");
    CHECK_TRUE(lexed.ok());
    CHECK_EQ(lexed.tokens.size(), size_t{5});
    CHECK_NEAR(lexed.tokens[0].number, 12.0, 1e-12);
    CHECK_NEAR(lexed.tokens[1].number, 3.25, 1e-12);
    CHECK_NEAR(lexed.tokens[2].number, 1000.0, 1e-12);
    CHECK_NEAR(lexed.tokens[3].number, 0.025, 1e-12);
}

TEST(RuleParser, lexer_rolls_back_an_exponent_with_no_digits) {
    // `2e` is the number 2 followed by the name `e`, not a malformed number. The
    // rollback is what keeps the failure inside the grammar, where the message
    // can say what was expected.
    const LexResult lexed = tokenize("2e");
    CHECK_TRUE(lexed.ok());
    CHECK_EQ(lexed.tokens.size(), size_t{3});
    CHECK_TRUE(lexed.tokens[0].kind == TokenKind::Number);
    CHECK_NEAR(lexed.tokens[0].number, 2.0, 1e-12);
    CHECK_TRUE(lexed.tokens[1].kind == TokenKind::Identifier);
    CHECK_EQ(std::string{lexed.tokens[1].text}, std::string{"e"});
}

TEST(RuleParser, lexer_does_not_absorb_a_dot_into_a_number) {
    // If it did, `1.foo` would lex as a number and the namespace separator would
    // depend on whether the qualifier happened to be numeric.
    const LexResult lexed = tokenize("1.foo");
    CHECK_EQ(lexed.tokens.size(), size_t{4});
    CHECK_TRUE(lexed.tokens[0].kind == TokenKind::Number);
    CHECK_NEAR(lexed.tokens[0].number, 1.0, 1e-12);
    CHECK_TRUE(lexed.tokens[1].kind == TokenKind::Dot);
    CHECK_TRUE(lexed.tokens[2].kind == TokenKind::Identifier);
}

TEST(RuleParser, lexer_decodes_string_escapes) {
    const LexResult lexed = tokenize("\"a\\nb\\t\\\"c\\\\\"");
    CHECK_TRUE(lexed.ok());
    CHECK_TRUE(lexed.tokens[0].kind == TokenKind::String);
    CHECK_EQ(lexed.tokens[0].value, std::string{"a\nb\t\"c\\"});
}

TEST(RuleParser, lexer_reports_an_unknown_escape_and_keeps_the_character) {
    const LexResult lexed = tokenize("\"a\\qb\"");
    CHECK_FALSE(lexed.ok());
    CHECK_EQ(lexed.diagnostics.size(), size_t{1});
    CHECK_TRUE(contains(lexed.diagnostics[0].message, "unknown escape sequence '\\q'"));
    CHECK_EQ(lexed.tokens[0].value, std::string{"aqb"});
}

TEST(RuleParser, lexer_reports_an_unterminated_string_at_its_opening_quote) {
    const LexResult lexed = tokenize("let s = \"abc\nlet t = 1;");
    CHECK_FALSE(lexed.ok());
    CHECK_EQ(lexed.diagnostics.size(), size_t{1});
    CHECK_TRUE(contains(lexed.diagnostics[0].message, "unterminated string literal"));
    CHECK_EQ(lexed.diagnostics[0].loc.line, 1u);
    CHECK_EQ(lexed.diagnostics[0].loc.column, 9u);
    // Closing at end of LINE, not at the next quote anywhere in the file, is what
    // stops one missing quote swallowing everything below it. The second line is
    // still tokenised.
    bool saw_second_let = false;
    for (const Token& token : lexed.tokens) {
        if (token.kind == TokenKind::KwLet && token.loc.line == 2) {
            saw_second_let = true;
        }
    }
    CHECK_TRUE(saw_second_let);
}

TEST(RuleParser, lexer_reports_an_unterminated_block_comment) {
    const LexResult lexed = tokenize("rule R {\n/* never closed\n");
    CHECK_FALSE(lexed.ok());
    CHECK_TRUE(contains(lexed.diagnostics[0].message, "unterminated block comment"));
    CHECK_EQ(lexed.diagnostics[0].loc.line, 2u);
    CHECK_EQ(lexed.diagnostics[0].loc.column, 1u);
}

TEST(RuleParser, lexer_skips_an_unexpected_character_and_carries_on) {
    const LexResult lexed = tokenize("a $ b");
    CHECK_FALSE(lexed.ok());
    CHECK_EQ(lexed.diagnostics.size(), size_t{1});
    CHECK_TRUE(contains(lexed.diagnostics[0].message, "unexpected character '$'"));
    // No placeholder token: the parser must not produce a second, derived error
    // for the same character.
    CHECK_EQ(lexed.tokens.size(), size_t{3});
    CHECK_TRUE(lexed.tokens[0].kind == TokenKind::Identifier);
    CHECK_TRUE(lexed.tokens[1].kind == TokenKind::Identifier);
    CHECK_TRUE(lexed.tokens[2].kind == TokenKind::End);
}

TEST(RuleParser, lexer_reports_a_single_ampersand_and_a_single_pipe) {
    const LexResult amp = tokenize("a & b");
    CHECK_TRUE(contains(amp.diagnostics[0].message, "write '&&' for logical and"));
    const LexResult pipe = tokenize("a | b");
    CHECK_TRUE(contains(pipe.diagnostics[0].message, "write '||' for logical or"));
}

TEST(RuleParser, lexer_distinguishes_one_and_two_character_operators) {
    const LexResult lexed = tokenize("< <= > >= = == ! != && ||");
    CHECK_TRUE(lexed.ok());
    CHECK_EQ(lexed.tokens.size(), size_t{11});
    CHECK_TRUE(lexed.tokens[0].kind == TokenKind::Less);
    CHECK_TRUE(lexed.tokens[1].kind == TokenKind::LessEqual);
    CHECK_TRUE(lexed.tokens[2].kind == TokenKind::Greater);
    CHECK_TRUE(lexed.tokens[3].kind == TokenKind::GreaterEqual);
    CHECK_TRUE(lexed.tokens[4].kind == TokenKind::Assign);
    CHECK_TRUE(lexed.tokens[5].kind == TokenKind::EqualEqual);
    CHECK_TRUE(lexed.tokens[6].kind == TokenKind::Bang);
    CHECK_TRUE(lexed.tokens[7].kind == TokenKind::BangEqual);
    CHECK_TRUE(lexed.tokens[8].kind == TokenKind::AmpAmp);
    CHECK_TRUE(lexed.tokens[9].kind == TokenKind::PipePipe);
}

TEST(RuleParser, lexer_drops_comments_entirely) {
    const LexResult lexed = tokenize("a // trailing\n  /* block */ b");
    CHECK_TRUE(lexed.ok());
    CHECK_EQ(lexed.tokens.size(), size_t{3});
    CHECK_EQ(std::string{lexed.tokens[0].text}, std::string{"a"});
    CHECK_EQ(std::string{lexed.tokens[1].text}, std::string{"b"});
}

TEST(RuleParser, lexer_reserves_keywords_but_not_contextual_words) {
    const LexResult keywords = tokenize("rule attr const let import from if else");
    CHECK_TRUE(keywords.tokens[0].kind == TokenKind::KwRule);
    CHECK_TRUE(keywords.tokens[1].kind == TokenKind::KwAttr);
    CHECK_TRUE(keywords.tokens[2].kind == TokenKind::KwConst);
    CHECK_TRUE(keywords.tokens[3].kind == TokenKind::KwLet);

    // These are recognised by the PARSER from a table, in the one position each
    // can appear, so they stay usable as ordinary names.
    const LexResult words = tokenize("float face front x top");
    for (size_t i = 0; i + 1 < words.tokens.size(); ++i) {
        CHECK_TRUE(words.tokens[i].kind == TokenKind::Identifier);
    }
}

// ============================================================================
// Diagnostic rendering
// ============================================================================

TEST(RuleParser, render_diagnostic_puts_the_caret_under_the_offending_token) {
    const std::string source = "rule R {\n    extrude(1 2);\n}\n";
    const ParseResult result = parse(source, "test.srl");
    CHECK_FALSE(result.ok());
    const Diagnostic* diagnostic = first_error(result);
    CHECK_TRUE(diagnostic != nullptr);
    if (diagnostic == nullptr) {
        return;
    }
    CHECK_EQ(diagnostic->loc.line, 2u);
    CHECK_EQ(diagnostic->loc.column, 15u);

    const std::string expected =
        "test.srl:2:15: error: expected ')' to close the argument list, found '2'\n"
        " 2 |     extrude(1 2);\n"
        "   | " + std::string(14, ' ') + "^\n";
    CHECK_EQ(render_diagnostic(*diagnostic, source, "test.srl"), expected);
}

TEST(RuleParser, render_diagnostic_underlines_the_whole_token) {
    // A caret alone under a six-character name says which character, not which
    // token. The tilde run is what makes the span readable.
    const std::string source = "rule R {\n  extrud(1);\n}\n";
    const ParseResult result = parse(source, "test.srl");
    const Diagnostic* diagnostic = first_error(result);
    CHECK_TRUE(diagnostic != nullptr);
    if (diagnostic == nullptr) {
        return;
    }
    CHECK_EQ(diagnostic->length, 6u);
    CHECK_TRUE(contains(render_diagnostic(*diagnostic, source, "test.srl"), "^~~~~~"));
}

TEST(RuleParser, render_diagnostic_echoes_a_tab_in_the_caret_line) {
    const std::string source = "rule R {\n\textrude(1 2);\n}\n";
    const ParseResult result = parse(source, "test.srl");
    const Diagnostic* diagnostic = first_error(result);
    CHECK_TRUE(diagnostic != nullptr);
    if (diagnostic == nullptr) {
        return;
    }
    const std::string rendered = render_diagnostic(*diagnostic, source, "test.srl");
    // The caret line must reproduce the tab, or a tab-indented file lines up in
    // the source line and not in the caret line underneath it.
    CHECK_TRUE(contains(rendered, "\n   | \t"));
}

TEST(RuleParser, render_diagnostic_omits_an_empty_filename) {
    const std::string source = "rule R {\n  extrud(1);\n}\n";
    const ParseResult result = parse(source, "");
    const Diagnostic* diagnostic = first_error(result);
    CHECK_TRUE(diagnostic != nullptr);
    if (diagnostic == nullptr) {
        return;
    }
    const std::string rendered = render_diagnostic(*diagnostic, source, "");
    CHECK_TRUE(rendered.rfind("2:3: error:", 0) == size_t{0});
}

// ============================================================================
// Declarations
// ============================================================================

TEST(RuleParser, parses_a_whole_small_file) {
    const ParseResult result = must_parse(
        "version \"0.1\"\n"
        "import facades from \"lib/facades.srl\"\n"
        "@range(2.0, 40.0)\n"
        "@group(\"Massing\")\n"
        "attr height : float = 12.0\n"
        "const floor_height : float = 3.2\n"
        "@start\n"
        "rule Lot(setback_m : float = 2.0) {\n"
        "  setback(setback_m);\n"
        "  extrude(height);\n"
        "}\n");
    CHECK_TRUE(result.ok());
    CHECK_EQ(result.file.version, std::string{"0.1"});
    CHECK_EQ(result.file.imports.size(), size_t{1});
    CHECK_EQ(result.file.imports[0].alias, std::string{"facades"});
    CHECK_EQ(result.file.imports[0].path, std::string{"lib/facades.srl"});
    CHECK_EQ(result.file.attributes.size(), size_t{1});
    CHECK_EQ(result.file.attributes[0].name, std::string{"height"});
    CHECK_EQ(result.file.constants.size(), size_t{1});
    CHECK_EQ(result.file.constants[0].name, std::string{"floor_height"});
    CHECK_EQ(result.file.rules.size(), size_t{1});
    CHECK_EQ(result.file.rules[0].params.size(), size_t{1});
    CHECK_EQ(result.file.start_rule, 0u);
}

TEST(RuleParser, a_file_with_no_version_still_parses) {
    const ParseResult result = must_parse("rule R { trim(); }");
    CHECK_TRUE(result.ok());
    CHECK_EQ(result.file.version, std::string{});
}

TEST(RuleParser, annotations_keep_their_source_order) {
    // Two annotations that a UI treats differently. If the parser reversed them,
    // @order(2) would be read as the group and nothing would say so.
    const ParseResult result = must_parse(
        "@group(\"Massing\")\n@order(2)\n@hidden\nattr h : float = 1.0\n");
    CHECK_TRUE(result.ok());
    const std::vector<Annotation>& annotations = result.file.attributes[0].annotations;
    CHECK_EQ(annotations.size(), size_t{3});
    CHECK_EQ(annotations[0].name, std::string{"group"});
    CHECK_EQ(annotations[1].name, std::string{"order"});
    CHECK_EQ(annotations[2].name, std::string{"hidden"});
    CHECK_TRUE(annotations[0].kind == AnnotationKind::Group);
    CHECK_TRUE(annotations[2].kind == AnnotationKind::Hidden);
}

TEST(RuleParser, an_unknown_annotation_is_rejected_with_a_suggestion) {
    // Silently ignoring it would look exactly like a working one until someone
    // opened the Inspector and found a free-text box.
    const ParseResult result = parse("@rnage(0, 10)\nattr h : float = 1.0\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_EQ(error_message(result, 0),
             std::string{"unknown annotation '@rnage' -- did you mean 'range'?"});
}

TEST(RuleParser, an_annotation_with_the_wrong_argument_count_is_rejected) {
    const ParseResult result = parse("@range(0)\nattr h : float = 1.0\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_EQ(error_message(result, 0),
             std::string{"'@range' expects 2 arguments, but 1 was given"});
}

TEST(RuleParser, a_variadic_annotation_accepts_many_arguments) {
    const ParseResult result =
        must_parse("@enum(\"brick\", \"glass\", \"stone\")\nattr s : string = \"brick\"\n");
    CHECK_TRUE(result.ok());
    CHECK_EQ(result.file.attributes[0].annotations[0].args.size(), size_t{3});
}

TEST(RuleParser, the_start_annotation_marks_exactly_one_rule) {
    const ParseResult result = must_parse("rule A { trim(); }\n@start\nrule B { trim(); }\n");
    CHECK_TRUE(result.ok());
    CHECK_EQ(result.file.start_rule, 1u);
    CHECK_FALSE(result.file.rules[0].is_start);
    CHECK_TRUE(result.file.rules[1].is_start);
}

TEST(RuleParser, two_start_annotations_are_an_error_naming_the_first_rule) {
    const ParseResult result =
        parse("@start\nrule A { trim(); }\n@start\nrule B { trim(); }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(contains(error_message(result, 0), "only one rule in a file can be marked"));
    CHECK_TRUE(contains(error_message(result, 0), "'A' already is"));
    CHECK_EQ(result.file.start_rule, 0u);
}

TEST(RuleParser, two_start_annotations_on_one_rule_are_an_error_and_not_a_crash) {
    // The same mistake as above, but both '@start' on ONE declaration -- and that
    // is the case that used to SEGFAULT. '@start' is checked before the rule is
    // pushed, so the first annotation set file_.start_rule to an index one past
    // the end of file_.rules and the second read it back. The test above, with
    // the two annotations on two SEPARATE rules, cannot reach that.
    const ParseResult result = parse("@start @start\nrule A { trim(); }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_EQ(error_count(result), size_t{1});
    CHECK_EQ(error_message(result, 0),
             std::string{"only one rule in a file can be marked '@start'; 'A' already is"});
    // The rule itself still lands, and still as the start rule: one bad
    // annotation does not cost the declaration it was written on.
    CHECK_EQ(result.file.rules.size(), size_t{1});
    CHECK_EQ(result.file.start_rule, 0u);
    CHECK_TRUE(result.file.rules[0].is_start);
}

TEST(RuleParser, a_second_start_on_a_later_rule_names_the_rule_that_claimed_it) {
    // The pending index is 1 here and the winner is 0, so this pins that the
    // message names the EARLIER rule rather than the one being declared -- the
    // difference the fix for the crash above could have flattened.
    const ParseResult result =
        parse("@start\nrule A { trim(); }\n@start @start\nrule B { trim(); }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_EQ(error_count(result), size_t{2});
    CHECK_EQ(error_message(result, 0),
             std::string{"only one rule in a file can be marked '@start'; 'A' already is"});
    CHECK_EQ(error_message(result, 1),
             std::string{"only one rule in a file can be marked '@start'; 'A' already is"});
    CHECK_EQ(result.file.start_rule, 0u);
    CHECK_FALSE(result.file.rules[1].is_start);
}

TEST(RuleParser, start_on_an_attribute_is_an_error) {
    const ParseResult result = parse("@start\nattr h : float = 1.0\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(contains(error_message(result, 0), "cannot go on an attribute"));
}

TEST(RuleParser, a_duplicate_rule_names_the_earlier_declaration) {
    const ParseResult result =
        parse("rule Facade { trim(); }\n\nrule Facade { trim(); }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_EQ(error_message(result, 0),
             std::string{"duplicate rule 'Facade'; previously declared at line 1, column 1"});
}

TEST(RuleParser, a_rule_may_not_take_a_builtin_operation_name) {
    // Allowing the shadow would make every call site's meaning depend on lookup
    // order, and that order would change the day a library added an operation.
    const ParseResult result = parse("rule extrude { trim(); }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(contains(error_message(result, 0), "a rule cannot be called 'extrude'"));
}

TEST(RuleParser, attributes_and_constants_share_one_namespace) {
    const ParseResult result =
        parse("attr h : float = 1.0\nconst h : float = 2.0\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(contains(error_message(result, 0), "duplicate declaration of 'h'"));
    CHECK_TRUE(contains(error_message(result, 0), "line 1"));
}

TEST(RuleParser, a_duplicate_import_alias_is_rejected) {
    const ParseResult result = parse(
        "import a from \"one.srl\"\nimport a from \"two.srl\"\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(contains(error_message(result, 0), "duplicate import alias 'a'"));
}

TEST(RuleParser, parameters_with_defaults_must_come_last) {
    const ParseResult result =
        parse("rule R(a : float = 1.0, b : float) { trim(); }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(contains(error_message(result, 0), "parameter 'b' has no default but follows one"));
}

TEST(RuleParser, a_duplicate_parameter_is_rejected) {
    const ParseResult result = parse("rule R(a : float, a : float) { trim(); }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(contains(error_message(result, 0), "duplicate parameter 'a'"));
}

TEST(RuleParser, an_unknown_type_is_rejected_with_a_suggestion) {
    const ParseResult result = parse("attr h : flaot = 1.0\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(contains(error_message(result, 0), "unknown type 'flaot'"));
    CHECK_TRUE(contains(error_message(result, 0), "did you mean 'float'?"));
}

TEST(RuleParser, array_types_parse) {
    const ParseResult result = must_parse("attr widths : float[] = [1.0, 2.0, 3.0]\n");
    CHECK_TRUE(result.ok());
    CHECK_TRUE(result.file.attributes[0].type.is_array);
    CHECK_EQ(std::string{primitive_type_name(result.file.attributes[0].type.type)},
             std::string{"float"});
    CHECK_TRUE(contains(dump(result.file), "(array (num 1) (num 2) (num 3))"));
}

TEST(RuleParser, an_attribute_without_a_default_is_rejected) {
    const ParseResult result = parse("attr h : float\nrule R { trim(); }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(contains(error_message(result, 0), "every attribute needs one"));
}

TEST(RuleParser, an_import_cannot_carry_annotations) {
    const ParseResult result = parse("@hidden\nimport a from \"x.srl\"\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(contains(error_message(result, 0), "an import cannot carry annotations"));
}

TEST(RuleParser, version_after_a_declaration_is_rejected) {
    // The COUNT is the point. Reporting the misplaced 'version' without consuming
    // it left recovery standing on the same token, and the version STRING was
    // then read as a declaration and reported a second time -- one mistake, two
    // messages. Only an assertion on the count can see that.
    const ParseResult result = parse("attr h : float = 1.0\nversion \"0.1\"\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_EQ(error_count(result), size_t{1});
    CHECK_EQ(error_message(result, 0),
             std::string{"'version' must come before every other declaration in the file"});
}

TEST(RuleParser, a_misplaced_version_does_not_hide_the_rule_after_it) {
    // The other half of the same recovery: one message, and the rest of the file
    // still parses.
    const ParseResult result =
        parse("attr h : float = 1.0\nversion \"0.1\"\nrule Good { trim(); }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_EQ(error_count(result), size_t{1});
    CHECK_EQ(result.file.rules.size(), size_t{1});
    CHECK_EQ(result.file.rules[0].name, std::string{"Good"});
}

// ============================================================================
// Statements
// ============================================================================

TEST(RuleParser, statements_keep_their_source_order) {
    // Three DIFFERENT operations. With one, or with three of the same, a parser
    // that reversed or dropped a statement would still pass.
    const std::string text = dump_of("rule R {\n  extrude(1);\n  taper(2);\n  trim();\n}\n");
    const size_t first = text.find("(call operation extrude");
    const size_t second = text.find("(call operation taper");
    const size_t third = text.find("(call operation trim");
    CHECK_TRUE(first != std::string::npos);
    CHECK_TRUE(second != std::string::npos);
    CHECK_TRUE(third != std::string::npos);
    CHECK((first) < (second));
    CHECK((second) < (third));
}

TEST(RuleParser, a_call_resolves_to_an_operation_or_to_a_rule) {
    const ParseResult result = must_parse("rule Facade { trim(); }\nrule R { Facade(); trim(); }\n");
    CHECK_TRUE(result.ok());
    const CallStmt* facade = find_call(result.file, "Facade");
    const CallStmt* trim = find_call(result.file, "trim");
    CHECK_TRUE(facade != nullptr);
    CHECK_TRUE(trim != nullptr);
    if (facade == nullptr || trim == nullptr) {
        return;
    }
    CHECK_EQ(target_name(facade->target), std::string{"rule"});
    CHECK_EQ(facade->rule, 0u);
    CHECK_EQ(target_name(trim->target), std::string{"operation"});
    CHECK_TRUE(trim->operation != kNoNode);
}

TEST(RuleParser, a_rule_may_be_called_above_its_declaration) {
    // Resolution runs after the whole file is read. Without that, a facade rule
    // would have to be written below the massing rule that calls it.
    const ParseResult result = must_parse("rule R { Facade(); }\nrule Facade { trim(); }\n");
    CHECK_TRUE(result.ok());
    const CallStmt* call = find_call(result.file, "Facade");
    CHECK_TRUE(call != nullptr);
    if (call == nullptr) {
        return;
    }
    CHECK_EQ(target_name(call->target), std::string{"rule"});
    CHECK_EQ(call->rule, 1u);
}

TEST(RuleParser, an_unknown_callee_is_rejected_with_a_suggestion) {
    const ParseResult result = parse("rule R { extrud(1); }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_EQ(error_message(result, 0),
             std::string{"unknown rule or operation 'extrud' -- did you mean 'extrude'?"});
    const CallStmt* call = find_call(result.file, "extrud");
    CHECK_TRUE(call != nullptr);
    if (call != nullptr) {
        CHECK_EQ(target_name(call->target), std::string{"unresolved"});
    }
}

TEST(RuleParser, an_operations_argument_count_is_checked) {
    const ParseResult too_many = parse("rule R { extrude(1, 2, 3); }\n", "test.srl");
    CHECK_FALSE(too_many.ok());
    CHECK_EQ(error_message(too_many, 0),
             std::string{"'extrude' expects 1 to 2 arguments, but 3 were given"});

    const ParseResult too_few = parse("rule R { extrude(); }\n", "test.srl");
    CHECK_FALSE(too_few.ok());
    CHECK_EQ(error_message(too_few, 0),
             std::string{"'extrude' expects 1 to 2 arguments, but 0 were given"});

    const ParseResult exact = parse("rule R { rotate(1, 2); }\n", "test.srl");
    CHECK_FALSE(exact.ok());
    CHECK_EQ(error_message(exact, 0),
             std::string{"'rotate' expects 3 arguments, but 2 were given"});
}

TEST(RuleParser, a_variadic_operation_has_only_a_lower_bound) {
    const ParseResult many = must_parse("rule R { print(1, 2, 3, 4); }\n");
    CHECK_TRUE(many.ok());
    const ParseResult none = parse("rule R { print(); }\n", "test.srl");
    CHECK_FALSE(none.ok());
    CHECK_EQ(error_message(none, 0),
             std::string{"'print' expects at least 1 argument, but 0 were given"});
}

TEST(RuleParser, a_rules_argument_count_accounts_for_defaults) {
    const std::string prelude = "rule Facade(a : float, b : float = 1.0) { trim(); }\n";
    CHECK_TRUE(must_parse(prelude + "rule R { Facade(1); }\n").ok());
    CHECK_TRUE(must_parse(prelude + "rule R { Facade(1, 2); }\n").ok());

    const ParseResult too_few = parse(prelude + "rule R { Facade(); }\n", "test.srl");
    CHECK_FALSE(too_few.ok());
    CHECK_EQ(error_message(too_few, 0),
             std::string{"'Facade' expects 1 to 2 arguments, but 0 were given"});

    const ParseResult too_many = parse(prelude + "rule R { Facade(1, 2, 3); }\n", "test.srl");
    CHECK_FALSE(too_many.ok());
    CHECK_TRUE(contains(error_message(too_many, 0), "but 3 were given"));
}

TEST(RuleParser, a_qualified_call_resolves_to_an_import_and_is_not_arity_checked) {
    // The other file is not loaded here, so guessing at its parameters would mean
    // an error about a rule this parser has never seen.
    const ParseResult result = must_parse(
        "import facades from \"lib/facades.srl\"\n"
        "rule R { facades.Window(1, 2, 3, 4, 5); }\n");
    CHECK_TRUE(result.ok());
    const CallStmt* call = find_call(result.file, "Window");
    CHECK_TRUE(call != nullptr);
    if (call == nullptr) {
        return;
    }
    CHECK_EQ(target_name(call->target), std::string{"imported"});
    CHECK_EQ(call->callee.qualifier, std::string{"facades"});
    CHECK_EQ(call->callee.text(), std::string{"facades.Window"});
}

TEST(RuleParser, an_unknown_import_alias_is_rejected_with_a_suggestion) {
    const ParseResult result = parse(
        "import facades from \"lib/facades.srl\"\nrule R { facads.Window(); }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(contains(error_message(result, 0), "unknown import alias 'facads'"));
    CHECK_TRUE(contains(error_message(result, 0), "did you mean 'facades'?"));
}

TEST(RuleParser, let_records_an_explicit_type_and_the_absence_of_one) {
    const ParseResult result = must_parse("rule R {\n  let a = 1.0;\n  let b : float = 2.0;\n}\n");
    CHECK_TRUE(result.ok());
    size_t seen = 0;
    bool untyped_first = false;
    for (const Stmt& statement : result.file.stmts) {
        const LetStmt* let = std::get_if<LetStmt>(&statement.node);
        if (let == nullptr) {
            continue;
        }
        if (seen == 0) {
            untyped_first = !let->has_type && let->name == "a";
        } else {
            CHECK_TRUE(let->has_type);
            CHECK_EQ(let->name, std::string{"b"});
        }
        ++seen;
    }
    CHECK_EQ(seen, size_t{2});
    CHECK_TRUE(untyped_first);
}

TEST(RuleParser, scope_and_discard_parse) {
    const std::string text = dump_of("rule R {\n  scope {\n    extrude(1);\n  }\n  discard;\n}\n");
    CHECK_TRUE(contains(text, "(scope"));
    CHECK_TRUE(contains(text, "(discard)"));
    // The scope's body must be INSIDE it, not a sibling.
    CHECK((text.find("(scope")) < (text.find("(call operation extrude")));
    CHECK((text.find("(call operation extrude")) < (text.find("(discard)")));
}

TEST(RuleParser, an_else_if_chain_nests_rather_than_flattening) {
    const std::string text = dump_of(
        "rule A { trim(); }\nrule B { trim(); }\nrule C { trim(); }\n"
        "rule R {\n"
        "  if (1 < 2) { A(); } else if (2 < 3) { B(); } else { C(); }\n"
        "}\n");
    // Two `if` nodes, and the second one must be deeper than the first. Counting
    // them alone would pass for a flattened chain.
    const size_t outer = text.find("(if ");
    CHECK_TRUE(outer != std::string::npos);
    const size_t inner = text.find("(if ", outer + 1);
    CHECK_TRUE(inner != std::string::npos);
    const size_t outer_indent = outer - text.rfind('\n', outer);
    const size_t inner_indent = inner - text.rfind('\n', inner);
    CHECK((outer_indent) < (inner_indent));
    CHECK_TRUE(contains(text, "(binary < (num 2) (num 3))"));
}

TEST(RuleParser, an_if_without_an_else_has_no_else_branch) {
    const ParseResult result = must_parse("rule A { trim(); }\nrule R { if (true) { A(); } }\n");
    CHECK_TRUE(result.ok());
    const IfStmt* branch = find_first<IfStmt>(result.file);
    CHECK_TRUE(branch != nullptr);
    if (branch == nullptr) {
        return;
    }
    CHECK_TRUE(branch->then_branch != kNoNode);
    CHECK_EQ(branch->else_branch, kNoNode);
}

TEST(RuleParser, a_statement_in_an_unexpected_place_names_what_was_expected) {
    const ParseResult result = parse("rule R { 3.0; }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(contains(error_message(result, 0), "expected a statement"));
    CHECK_TRUE(contains(error_message(result, 0), "found '3.0'"));
}

TEST(RuleParser, a_call_without_parentheses_says_so) {
    const ParseResult result = parse("rule A { trim(); }\nrule R { A; }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(contains(error_message(result, 0), "always called with parentheses"));
}

// ============================================================================
// Split -- the construct the language exists for
// ============================================================================

TEST(RuleParser, a_split_distinguishes_all_four_size_kinds) {
    // Every entry uses the SAME literal, 2. A parser that dropped '~' or '%'
    // cannot pass this by producing the right number.
    const ParseResult result = must_parse(
        "rule A { trim(); }\n"
        "rule R {\n"
        "  split(x) {\n"
        "    2 : A();\n"
        "    2% : A();\n"
        "    ~2 : A();\n"
        "    ~2% : A();\n"
        "  }\n"
        "}\n");
    CHECK_TRUE(result.ok());
    const SplitStmt* split = find_first<SplitStmt>(result.file);
    CHECK_TRUE(split != nullptr);
    if (split == nullptr) {
        return;
    }
    CHECK_EQ(split->entries.size(), size_t{4});
    CHECK_EQ(kind_name(split->entries[0].size.kind), std::string{"absolute"});
    CHECK_EQ(kind_name(split->entries[1].size.kind), std::string{"relative"});
    CHECK_EQ(kind_name(split->entries[2].size.kind), std::string{"floating"});
    CHECK_EQ(kind_name(split->entries[3].size.kind), std::string{"floating-relative"});
}

TEST(RuleParser, split_entries_keep_their_order) {
    // Distinct sizes, so a reordering shows up as a wrong number rather than as
    // a right one in the wrong place.
    const ParseResult result = must_parse(
        "rule A { trim(); }\n"
        "rule R { split(y) { 1 : A(); 2 : A(); 3 : A(); } }\n");
    CHECK_TRUE(result.ok());
    const SplitStmt* split = find_first<SplitStmt>(result.file);
    CHECK_TRUE(split != nullptr);
    if (split == nullptr) {
        return;
    }
    CHECK_EQ(split->entries.size(), size_t{3});
    const std::string text = dump(result.file);
    const size_t one = text.find("(entry absolute (num 1)");
    const size_t two = text.find("(entry absolute (num 2)");
    const size_t three = text.find("(entry absolute (num 3)");
    CHECK_TRUE(three != std::string::npos);
    CHECK((one) < (two));
    CHECK((two) < (three));
    CHECK_EQ(std::string{split_axis_name(split->axis)}, std::string{"y"});
}

TEST(RuleParser, a_floating_repeat_between_two_fixed_ends_is_the_facade_case) {
    // The shape every facade in the product will be: fixed corner, a repeating
    // band that absorbs the remainder, fixed corner.
    const ParseResult result = must_parse(
        "rule Corner { trim(); }\nrule Bay { trim(); }\nrule Pier { trim(); }\n"
        "rule Facade {\n"
        "  split(x) {\n"
        "    1.0 : Corner();\n"
        "    repeat {\n"
        "      ~2.5 : Bay();\n"
        "      0.4  : Pier();\n"
        "    }\n"
        "    1.0 : Corner();\n"
        "  }\n"
        "}\n");
    CHECK_TRUE(result.ok());
    const SplitStmt* split = find_first<SplitStmt>(result.file);
    CHECK_TRUE(split != nullptr);
    if (split == nullptr) {
        return;
    }
    CHECK_EQ(split->entries.size(), size_t{3});
    CHECK_FALSE(split->entries[0].is_repeat);
    CHECK_TRUE(split->entries[1].is_repeat);
    CHECK_FALSE(split->entries[2].is_repeat);
    CHECK_EQ(split->entries[1].children.size(), size_t{2});
    CHECK_EQ(kind_name(split->entries[1].children[0].size.kind), std::string{"floating"});
    CHECK_EQ(kind_name(split->entries[1].children[1].size.kind), std::string{"absolute"});
    // The group's own entries must keep their order too: bay then pier, not pier
    // then bay, or every window sits against the wrong side of its column.
    const std::string text = dump(result.file);
    CHECK((text.find("(entry floating (num 2.5)")) < (text.find("(entry absolute (num 0.4)")));
}

TEST(RuleParser, repeat_without_braces_is_sugar_for_a_group_of_one) {
    const std::string sugar =
        dump_of("rule A { trim(); }\nrule R { split(x) { repeat ~1 : A(); } }\n");
    const std::string braced =
        dump_of("rule A { trim(); }\nrule R { split(x) { repeat { ~1 : A(); } } }\n");
    CHECK_EQ(sugar, braced);
    CHECK_TRUE(contains(sugar, "(repeat"));
}

TEST(RuleParser, a_repeat_group_may_not_contain_a_repeat_group) {
    const ParseResult result = parse(
        "rule A { trim(); }\nrule R { split(x) { repeat { repeat { 1 : A(); } } } }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(contains(error_message(result, 0), "a repeat group cannot contain another repeat"));
}

TEST(RuleParser, an_empty_split_is_rejected) {
    const ParseResult result = parse("rule R { split(x) { } }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(any_error_contains(result, "a 'split' needs at least one entry"));
}

TEST(RuleParser, an_empty_repeat_group_is_rejected) {
    const ParseResult result =
        parse("rule A { trim(); }\nrule R { split(x) { repeat { } } }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(any_error_contains(result, "a repeat group needs at least one entry"));
}

TEST(RuleParser, an_unknown_split_axis_is_rejected_with_the_list) {
    // Asserted as ONE exact string, tail included. 'w' is one edit from all three
    // axes, so the suggestion is decided entirely by suggest()'s tie-break; two
    // contains() calls that stop before "-- did you mean" cannot see it change.
    const ParseResult result = parse("rule R { split(w) { 1 : trim(); } }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_EQ(error_message(result, 0),
             std::string{"unknown split axis 'w'; the axes are 'x', 'y' or 'z' "
                         "-- did you mean 'x'?"});
}

TEST(RuleParser, a_tied_spelling_suggestion_names_the_first_candidate) {
    // 'Ac' is exactly one edit from BOTH 'Aa' and 'Ab'. suggest() keeps the
    // FIRST, and that strict-less-than is the whole of the determinism its own
    // comment claims. Turning it into `<=` names 'Ab' here and changes nothing
    // any other suggestion test in this file can see, because none of the others
    // uses a token that two candidates are equally close to.
    const ParseResult result =
        parse("rule Aa { trim(); }\nrule Ab { trim(); }\nrule R { Ac(); }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_EQ(error_count(result), size_t{1});
    CHECK_EQ(error_message(result, 0),
             std::string{"unknown rule or operation 'Ac' -- did you mean 'Aa'?"});
}

TEST(RuleParser, a_conditional_size_does_not_swallow_the_entry_colon) {
    // The one real ambiguity in the grammar. The else-branch of a ternary is
    // parsed as a ternary, not as an expression, so it cannot take the split's
    // colon. Three colons on one line, and the entry still has a body.
    const ParseResult result = must_parse(
        "rule Wide { trim(); }\nrule Narrow { trim(); }\n"
        "attr h : float = 2.0\n"
        "rule R {\n"
        "  split(x) {\n"
        "    h > 1 ? 2 : 3 : Wide();\n"
        "    1 : Narrow();\n"
        "  }\n"
        "}\n");
    CHECK_TRUE(result.ok());
    const SplitStmt* split = find_first<SplitStmt>(result.file);
    CHECK_TRUE(split != nullptr);
    if (split == nullptr) {
        return;
    }
    CHECK_EQ(split->entries.size(), size_t{2});
    const std::string text = dump(result.file);
    CHECK_TRUE(contains(
        text, "(entry absolute (ternary (binary > (name h) (num 1)) (num 2) (num 3))"));
    CHECK_TRUE(contains(text, "(call rule Wide)"));
    CHECK_TRUE(contains(text, "(call rule Narrow)"));
}

TEST(RuleParser, a_chained_conditional_size_still_stops_at_its_own_colon) {
    // The harder half of the same ambiguity: FOUR colons on the line, two opened
    // by '?' and two belonging to the split. A grammar that parsed the else
    // branch at a tighter level than `ternary` would stop at the first '?' it
    // could not chain into, and the split would lose its entry.
    const ParseResult result = must_parse(
        "rule Wide { trim(); }\nrule Mid { trim(); }\nrule Narrow { trim(); }\n"
        "attr h : float = 2.0\n"
        "rule R {\n"
        "  split(x) {\n"
        "    h > 2 ? 4 : h > 1 ? 3 : 2 : Wide();\n"
        "    1 : Narrow();\n"
        "  }\n"
        "}\n");
    CHECK_TRUE(result.ok());
    const SplitStmt* split = find_first<SplitStmt>(result.file);
    CHECK_TRUE(split != nullptr);
    if (split == nullptr) {
        return;
    }
    CHECK_EQ(split->entries.size(), size_t{2});
    CHECK_TRUE(contains(dump(result.file),
                        "(entry absolute (ternary (binary > (name h) (num 2)) (num 4) "
                        "(ternary (binary > (name h) (num 1)) (num 3) (num 2)))"));
    CHECK_TRUE(contains(dump(result.file), "(call rule Wide)"));
}

TEST(RuleParser, a_bare_case_body_and_a_braced_one_produce_the_same_tree) {
    const std::string bare = dump_of("rule A { trim(); }\nrule R { split(x) { 1 : A(); } }\n");
    const std::string braced =
        dump_of("rule A { trim(); }\nrule R { split(x) { 1 : { A(); } } }\n");
    CHECK_EQ(bare, braced);
}

TEST(RuleParser, a_split_entry_body_may_hold_several_statements) {
    const ParseResult result = must_parse(
        "rule R { split(x) { 1 : { extrude(2); taper(3); } } }\n");
    CHECK_TRUE(result.ok());
    const std::string text = dump(result.file);
    CHECK((text.find("(call operation extrude")) < (text.find("(call operation taper")));
}

// ============================================================================
// Select
// ============================================================================

TEST(RuleParser, select_keeps_its_domain_and_its_selectors_in_order) {
    const ParseResult result = must_parse(
        "rule Facade { trim(); }\nrule Roof { trim(); }\nrule Base { trim(); }\n"
        "rule R {\n"
        "  select face {\n"
        "    side   : Facade();\n"
        "    top    : Roof();\n"
        "    bottom : Base();\n"
        "  }\n"
        "}\n");
    CHECK_TRUE(result.ok());
    const SelectStmt* select = find_first<SelectStmt>(result.file);
    CHECK_TRUE(select != nullptr);
    if (select == nullptr) {
        return;
    }
    CHECK_EQ(std::string{component_domain_name(select->domain)}, std::string{"face"});
    CHECK_EQ(select->cases.size(), size_t{3});
    CHECK_EQ(std::string{component_selector_name(select->cases[0].selector)}, std::string{"side"});
    CHECK_EQ(std::string{component_selector_name(select->cases[1].selector)}, std::string{"top"});
    CHECK_EQ(std::string{component_selector_name(select->cases[2].selector)},
             std::string{"bottom"});
}

TEST(RuleParser, an_unknown_selector_is_rejected_with_a_suggestion) {
    const ParseResult result =
        parse("rule A { trim(); }\nrule R { select face { fornt : A(); } }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(contains(error_message(result, 0), "unknown face selector 'fornt'"));
    CHECK_TRUE(contains(error_message(result, 0), "did you mean 'front'?"));
}

TEST(RuleParser, an_unknown_domain_suggests_the_right_one) {
    // `faces` is the name a CityEngine user will reach for first.
    const ParseResult result =
        parse("rule A { trim(); }\nrule R { select faces { front : A(); } }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(contains(error_message(result, 0), "unknown component domain 'faces'"));
    CHECK_TRUE(contains(error_message(result, 0), "did you mean 'face'?"));
}

TEST(RuleParser, a_duplicate_selector_is_rejected) {
    const ParseResult result = parse(
        "rule A { trim(); }\nrule R { select face { top : A(); top : A(); } }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(contains(error_message(result, 0), "duplicate selector 'top'"));
}

TEST(RuleParser, an_empty_select_is_rejected) {
    const ParseResult result = parse("rule R { select face { } }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(any_error_contains(result, "a 'select' needs at least one selector"));
}

TEST(RuleParser, every_component_domain_parses) {
    for (const char* domain : {"face", "edge", "vertex", "object"}) {
        const std::string source =
            "rule A { trim(); }\nrule R { select " + std::string{domain} + " { all : A(); } }\n";
        const ParseResult result = must_parse(source);
        CHECK_TRUE(result.ok());
        const SelectStmt* select = find_first<SelectStmt>(result.file);
        CHECK_TRUE(select != nullptr);
        if (select != nullptr) {
            CHECK_EQ(std::string{component_domain_name(select->domain)}, std::string{domain});
        }
    }
}

// ============================================================================
// Choose
// ============================================================================

TEST(RuleParser, choose_keeps_its_weights_and_its_order) {
    const ParseResult result = must_parse(
        "rule Brick { trim(); }\nrule Glass { trim(); }\nrule Stone { trim(); }\n"
        "rule R {\n"
        "  choose {\n"
        "    30 : Brick();\n"
        "    50 : Glass();\n"
        "    20 : Stone();\n"
        "  }\n"
        "}\n");
    CHECK_TRUE(result.ok());
    const ChooseStmt* choose = find_first<ChooseStmt>(result.file);
    CHECK_TRUE(choose != nullptr);
    if (choose == nullptr) {
        return;
    }
    CHECK_EQ(choose->cases.size(), size_t{3});
    const std::string text = dump(result.file);
    const size_t brick = text.find("(case (num 30)");
    const size_t glass = text.find("(case (num 50)");
    const size_t stone = text.find("(case (num 20)");
    CHECK_TRUE(stone != std::string::npos);
    CHECK((brick) < (glass));
    CHECK((glass) < (stone));
}

TEST(RuleParser, a_negative_literal_weight_is_rejected) {
    const ParseResult result = parse(
        "rule A { trim(); }\nrule B { trim(); }\n"
        "rule R { choose { -1 : A(); 2 : B(); } }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(contains(error_message(result, 0), "weight cannot be negative"));
}

TEST(RuleParser, a_computed_weight_is_left_to_the_interpreter) {
    // Checking it here would need an evaluator, which is not this feature.
    const ParseResult result = must_parse(
        "attr bias : float = 1.0\nrule A { trim(); }\n"
        "rule R { choose { bias : A(); 1.0 - bias : A(); } }\n");
    CHECK_TRUE(result.ok());
}

TEST(RuleParser, an_empty_choose_is_rejected) {
    const ParseResult result = parse("rule R { choose { } }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(any_error_contains(result, "a 'choose' needs at least one alternative"));
}

// ============================================================================
// Expressions
// ============================================================================

TEST(RuleParser, multiplication_binds_tighter_than_addition) {
    CHECK_TRUE(contains(dump_of("attr a : float = 1 + 2 * 3\n"),
                        "(binary + (num 1) (binary * (num 2) (num 3)))"));
    CHECK_TRUE(contains(dump_of("attr a : float = 1 * 2 + 3\n"),
                        "(binary + (binary * (num 1) (num 2)) (num 3))"));
}

TEST(RuleParser, subtraction_is_left_associative) {
    // Written with '-' rather than '+' on purpose: a right-associative bug in a
    // commuting operator produces the same answer and no test can see it.
    CHECK_TRUE(contains(dump_of("attr a : float = 1 - 2 - 3\n"),
                        "(binary - (binary - (num 1) (num 2)) (num 3))"));
}

TEST(RuleParser, division_is_left_associative) {
    CHECK_TRUE(contains(dump_of("attr a : float = 8 / 4 / 2\n"),
                        "(binary / (binary / (num 8) (num 4)) (num 2))"));
}

TEST(RuleParser, comparison_binds_looser_than_arithmetic) {
    CHECK_TRUE(contains(dump_of("attr a : bool = 1 + 2 < 4\n"),
                        "(binary < (binary + (num 1) (num 2)) (num 4))"));
}

TEST(RuleParser, equality_binds_looser_than_comparison) {
    CHECK_TRUE(contains(dump_of("attr a : bool = 1 < 2 == true\n"),
                        "(binary == (binary < (num 1) (num 2)) (bool true))"));
}

TEST(RuleParser, and_binds_tighter_than_or) {
    CHECK_TRUE(contains(dump_of("attr a : bool = true || false && true\n"),
                        "(binary || (bool true) (binary && (bool false) (bool true)))"));
    CHECK_TRUE(contains(dump_of("attr a : bool = true && false || true\n"),
                        "(binary || (binary && (bool true) (bool false)) (bool true))"));
    // An '&&' on BOTH sides of the '||'. With '&&' on one side only, giving the
    // two operators the same precedence produces the same tree by accident;
    // this arrangement is the one that tells them apart.
    CHECK_TRUE(contains(
        dump_of("attr a : bool = true && false || false && true\n"),
        "(binary || (binary && (bool true) (bool false)) (binary && (bool false) (bool true)))"));
}

TEST(RuleParser, unary_minus_binds_tighter_than_multiplication) {
    CHECK_TRUE(contains(dump_of("attr a : float = -2 * 3\n"),
                        "(binary * (unary - (num 2)) (num 3))"));
}

TEST(RuleParser, unary_operators_stack) {
    CHECK_TRUE(contains(dump_of("attr a : bool = !!true\n"),
                        "(unary ! (unary ! (bool true)))"));
}

TEST(RuleParser, the_ternary_is_right_associative) {
    CHECK_TRUE(contains(
        dump_of("attr a : float = true ? 1 : false ? 2 : 3\n"),
        "(ternary (bool true) (num 1) (ternary (bool false) (num 2) (num 3)))"));
}

TEST(RuleParser, parentheses_override_precedence) {
    CHECK_TRUE(contains(dump_of("attr a : float = (1 + 2) * 3\n"),
                        "(binary * (binary + (num 1) (num 2)) (num 3))"));
}

TEST(RuleParser, arrays_and_indexing_parse) {
    CHECK_TRUE(contains(dump_of("attr a : float = [1, 2, 3][1]\n"),
                        "(index (array (num 1) (num 2) (num 3)) (num 1))"));
}

TEST(RuleParser, a_function_call_in_expression_position_is_recorded_unresolved) {
    // The parser owns the statement-position catalogue and nothing else. Owning
    // the built-in FUNCTION library too would make every later feature a change
    // to the parser.
    const ParseResult result = must_parse("attr a : float = max(geometry.area(), 3)\n");
    CHECK_TRUE(result.ok());
    const std::string text = dump(result.file);
    CHECK_TRUE(contains(text, "(fcall max (fcall geometry.area) (num 3))"));
}

TEST(RuleParser, a_trailing_comma_is_allowed_in_a_list) {
    CHECK_TRUE(must_parse("attr a : float[] = [1, 2, 3,]\n").ok());
    CHECK_TRUE(must_parse("rule R(a : float = 1, b : float = 2,) { trim(); }\n").ok());
}

TEST(RuleParser, percent_outside_a_split_gets_its_own_message) {
    const ParseResult result = parse("attr a : float = 5 % 2\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(contains(error_message(result, 0), "there is no modulo operator"));
}

TEST(RuleParser, tilde_outside_a_split_gets_its_own_message) {
    const ParseResult result = parse("attr a : float = ~5\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(contains(error_message(result, 0), "'~' marks a floating size"));
}

TEST(RuleParser, string_values_survive_the_round_trip) {
    const ParseResult result = must_parse("attr s : string = \"walls/brick_01.png\"\n");
    CHECK_TRUE(result.ok());
    CHECK_TRUE(contains(dump(result.file), "(str \"walls/brick_01.png\")"));
}

// ============================================================================
// Recovery, limits and determinism
// ============================================================================

TEST(RuleParser, several_broken_statements_all_get_reported) {
    // One mistake per save is what an unrecovering parser gives you.
    const ParseResult result = parse(
        "rule R {\n"
        "  extrude(1;\n"
        "  taper 2);\n"
        "  trim(;\n"
        "  cleanup();\n"
        "}\n",
        "test.srl");
    CHECK_FALSE(result.ok());
    CHECK((size_t{3}) <= (error_count(result)));
    // An upper bound as well: without one, a cascade of derived errors also
    // passes a "reports several" test.
    CHECK((error_count(result)) <= (size_t{5}));
    // Recovery must reach the last statement, or the errors are just noise
    // before an abandoned parse.
    CHECK_TRUE(contains(dump(result.file), "(call operation cleanup)"));
}

TEST(RuleParser, a_call_statement_needs_its_semicolon) {
    // parser.hpp's grammar writes the ';' of a callStmt as a literal token and
    // nothing required it: turning that expect() into a match() made
    // `trim() cleanup()` legal and every test still passed.
    const ParseResult result = parse("rule R { trim() cleanup() }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_EQ(error_count(result), size_t{1});
    CHECK_EQ(error_message(result, 0),
             std::string{"expected ';' after the call, found 'cleanup'"});
}

TEST(RuleParser, a_broken_declaration_does_not_hide_the_next_rule) {
    const ParseResult result =
        parse("attr h : flaot = 1.0\nrule Good { trim(); }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_EQ(error_count(result), size_t{1});
    CHECK_EQ(result.file.rules.size(), size_t{1});
    CHECK_EQ(result.file.rules[0].name, std::string{"Good"});
}

TEST(RuleParser, diagnostics_come_out_in_source_order) {
    // The lexer produces its diagnostics before the parser produces any, so
    // without a sort the '$' at column 20 would be reported before the bad call
    // at column 10.
    const ParseResult result = parse("rule R { extrud(); $ }\n", "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_EQ(error_count(result), size_t{2});
    CHECK_TRUE(contains(error_message(result, 0), "unknown rule or operation 'extrud'"));
    CHECK_TRUE(contains(error_message(result, 1), "unexpected character '$'"));
    CHECK((result.diagnostics[0].loc.offset) < (result.diagnostics[1].loc.offset));
}

TEST(RuleParser, a_deeply_nested_expression_is_refused_rather_than_overflowing_the_stack) {
    std::string source = "rule R { let x = ";
    source.append(400, '(');
    source += "1";
    source.append(400, ')');
    source += "; }\n";
    const ParseResult result = parse(source, "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(any_error_contains(result, "nested too deeply"));
}

TEST(RuleParser, a_chained_conditional_is_refused_rather_than_overflowing_the_stack) {
    // `1 ? 1 : 1 ? 1 : ...` recurses through parse_ternary's ELSE branch, which
    // is a tail call back into parse_ternary and never passes through
    // parse_expression's guard again. The nesting cap therefore did nothing for
    // this shape: 20 000 links parsed clean and 50 000 dumped core. 400 is well
    // past the 128-level limit and costs a few microseconds.
    std::string source = "rule R { let x = ";
    for (int i = 0; i < 400; ++i) {
        source += "1 ? 1 : ";
    }
    source += "1; }\n";
    const ParseResult result = parse(source, "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(any_error_contains(result, "nested too deeply"));
}

TEST(RuleParser, a_chained_else_if_is_refused_rather_than_overflowing_the_stack) {
    // The second unguarded tail recursion: `else if` goes straight back into
    // parse_if and skips parse_statement's guard. 200 000 links dumped core.
    std::string source = "rule R { if (true) { } ";
    for (int i = 0; i < 400; ++i) {
        source += "else if (true) { } ";
    }
    source += "}\n";
    const ParseResult result = parse(source, "test.srl");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(any_error_contains(result, "nested too deeply"));
}

TEST(RuleParser, a_flood_of_errors_stops_at_the_cap) {
    std::string source = "rule R {\n";
    for (int i = 0; i < 400; ++i) {
        source += "  nosuchop();\n";
    }
    source += "}\n";
    const ParseResult result = parse(source, "test.srl");
    CHECK_FALSE(result.ok());
    CHECK((result.diagnostics.size()) <= (kMaxDiagnostics));
    CHECK_EQ(cap_message_count(result.diagnostics), size_t{1});
}

TEST(RuleParser, a_flood_of_lexical_errors_stops_at_the_cap) {
    // The case the cap was written for -- someone points the parser at a .png --
    // is entirely LEXICAL, and lexical diagnostics never went through the
    // parser's counter. A 130 KB image returned 42 907 diagnostics that rendered
    // to 64 MB of text, with the cap sentence sitting uselessly in the middle of
    // them. A rule file is all-ASCII, so a run of '$' is the same flood.
    const std::string source(5000, '$');
    const ParseResult result = parse(source, "test.srl");
    CHECK_FALSE(result.ok());
    CHECK((result.diagnostics.size()) <= (kMaxDiagnostics));
    CHECK_EQ(cap_message_count(result.diagnostics), size_t{1});
}

TEST(RuleParser, tokenize_caps_its_own_diagnostics) {
    // tokenize() is a public entry point, so the cap has to hold for a caller
    // that never reaches the parser at all.
    const LexResult lexed = tokenize(std::string(5000, '$'));
    CHECK_FALSE(lexed.ok());
    CHECK((lexed.diagnostics.size()) <= (kMaxDiagnostics));
    CHECK_EQ(cap_message_count(lexed.diagnostics), size_t{1});
    CHECK_TRUE(contains(lexed.diagnostics.back().message, "too many errors"));
}

TEST(RuleParser, a_flood_from_both_the_lexer_and_the_parser_stops_at_one_cap) {
    // Both producers count, and the cap is a promise about the merged list. If
    // each capped its own list independently the sentence would appear twice and
    // the total would be twice the cap.
    std::string source;
    for (int i = 0; i < 500; ++i) {
        source += "$ nosuchop();\n";
    }
    const ParseResult result = parse(source, "test.srl");
    CHECK_FALSE(result.ok());
    CHECK((result.diagnostics.size()) <= (kMaxDiagnostics));
    CHECK_EQ(cap_message_count(result.diagnostics), size_t{1});
}

TEST(RuleParser, the_same_text_produces_the_same_tree_twice) {
    const std::string source =
        "version \"0.1\"\n"
        "attr h : float = 12.0\n"
        "rule A { trim(); }\n"
        "rule R { split(y) { ~3.2 : A(); 20% : A(); } }\n";
    CHECK_EQ(dump(parse(source, "a.srl").file), dump(parse(source, "b.srl").file));
}

TEST(RuleParser, whitespace_and_comments_do_not_change_the_tree) {
    // dump() carries no locations, so this comparison is possible at all -- and
    // it is what "same text in, identical AST out" actually buys.
    const std::string tidy =
        "attr h : float = 12.0\n"
        "rule A { trim(); }\n"
        "rule R {\n"
        "  split(y) {\n"
        "    ~3.2 : A();\n"
        "    20%  : A();\n"
        "  }\n"
        "}\n";
    const std::string scruffy =
        "// the same file, written badly\n"
        "attr h:float=12.0\n\n\n"
        "rule A{/* nothing */trim();}\n"
        "rule R{split(y){~3.2:A();20%:A();}}\n";
    CHECK_EQ(dump_of(tidy), dump_of(scruffy));
}

TEST(RuleParser, the_dump_of_a_minimal_rule_is_exactly_this) {
    // A format lock. dump() is what the determinism tests compare and what a
    // failure elsewhere gets printed as, so a silent change to it is a change to
    // every one of those.
    const std::string expected =
        "(rule-file\n"
        "  (rule R -\n"
        "    (block\n"
        "      (call operation extrude (num 1)))))\n";
    CHECK_EQ(dump_of("rule R { extrude(1); }\n"), expected);
}

TEST(RuleParser, the_dump_records_an_attributes_annotations_and_default) {
    const std::string expected =
        "(rule-file\n"
        "  (attr h float (annotation range (num 0) (num 10)) (default (num 2))))\n";
    CHECK_EQ(dump_of("@range(0, 10)\nattr h : float = 2.0\n"), expected);
}

TEST(RuleParser, the_dump_marks_required_and_defaulted_parameters_differently) {
    const ParseResult result = must_parse("rule R(a : float, b : string = \"x\") { trim(); }\n");
    const std::string text = dump(result.file);
    CHECK_TRUE(contains(text, "(param a float required)"));
    CHECK_TRUE(contains(text, "(param b string (default (str \"x\")))"));
}

TEST(RuleParser, parameters_keep_their_source_order) {
    // The parameter list is the last ORDERED list in the language that nothing
    // asserted the order of, and it is the one D2 binds positional arguments by:
    // a reversed list makes R(3, 10) mean R(b: 3, a: 10) in silence. Two
    // contains() calls cannot see a reversal, so both the tree and the dump are
    // checked positionally. The two parameters differ in type as well as in name
    // so that a swap cannot be masked by a coincidence.
    const ParseResult result =
        must_parse("rule R(a : float, b : string = \"x\") { trim(); }\n");
    CHECK_TRUE(result.ok());
    CHECK_EQ(result.file.rules.size(), size_t{1});
    CHECK_EQ(result.file.rules[0].params.size(), size_t{2});
    CHECK_EQ(result.file.rules[0].params[0].name, std::string{"a"});
    CHECK_EQ(result.file.rules[0].params[1].name, std::string{"b"});
    const std::string text = dump(result.file);
    CHECK((text.find("(param a ")) < (text.find("(param b ")));
}

TEST(RuleParser, a_parameter_list_may_end_with_a_comma) {
    // The grammar in parser.hpp spells paramList with an optional trailing comma
    // because the code accepts one; this is the test that keeps the two honest.
    const ParseResult result = must_parse("rule R(a : float, b : float,) { trim(); }\n");
    CHECK_TRUE(result.ok());
    CHECK_EQ(result.file.rules[0].params.size(), size_t{2});
    CHECK_EQ(result.file.rules[0].params[1].name, std::string{"b"});
}

TEST(RuleParser, an_empty_source_parses_to_an_empty_file) {
    const ParseResult result = must_parse("");
    CHECK_TRUE(result.ok());
    CHECK_EQ(result.file.rules.size(), size_t{0});
    CHECK_EQ(dump(result.file), std::string{"(rule-file)\n"});
}

TEST(RuleParser, a_rule_with_an_empty_body_is_legal) {
    // A terminal rule that produces nothing is a normal thing to write while a
    // grammar is being built up.
    const ParseResult result = must_parse("rule Leaf { }\n");
    CHECK_TRUE(result.ok());
    CHECK_EQ(result.file.rules.size(), size_t{1});
}
