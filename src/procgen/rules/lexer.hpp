// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file lexer.hpp
 * @brief Source text to tokens for the Stratum rule language, and the diagnostic it reports with
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### What this file is the bottom of
 *
 * The rule language (issue #38, feature D1) is three layers: this lexer, the
 * AST in `ast.hpp`, and the recursive-descent parser in `parser.hpp`. Nothing
 * here evaluates anything. Nothing here knows what `extrude` means. It turns
 * bytes into tokens with exact positions, and it reports what it could not turn
 * into a token.
 *
 * ### Why SourceLoc and Diagnostic live HERE and not in ast.hpp
 *
 * Both layers need them, and one of the two headers has to own them. The lexer
 * wins on two counts: a source location is a purely lexical fact, and the FIRST
 * diagnostic a rule file can produce is a lexical one -- an unterminated string
 * is found before any grammar rule runs. So `ast.hpp` includes this header, not
 * the other way round. The alternative, a fourth `diagnostic.hpp`, was rejected
 * only because D1 ships five source files and a header holding two structs is
 * not worth one of them; when this directory grows, that is the first thing to
 * split out.
 *
 * ### Error messages are a feature, not an afterthought
 *
 * A rule author is an artist with a text editor, not a compiler engineer. Every
 * Diagnostic carries a line, a column, the length of the offending run and the
 * spelling of the offending token, so render_diagnostic() can draw a caret under
 * it. "parse error on line 1" is the failure mode this design exists to prevent,
 * and the tests assert on the rendered text for exactly that reason.
 *
 * ### Columns are BYTES
 *
 * `SourceLoc::column` counts bytes from the start of the line, one-based. For
 * ASCII, which every keyword and operator in the language is, that is the column
 * a text editor shows. For a UTF-8 string literal with multi-byte characters the
 * caret can land mid-glyph. Counting code points instead would need the lexer to
 * decode UTF-8, and a decoder that must not reject anything -- comments and
 * string literals hold arbitrary bytes -- is a larger liability than a caret that
 * is occasionally a byte or two off inside a non-ASCII literal. A tab counts as
 * one column and is echoed as a tab in the caret line, so a tab-indented file
 * still lines up in the terminal that rendered it.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace stratum::procgen::rules {

// ============================================================================
// Locations and diagnostics
// ============================================================================

/**
 * @brief A point in a source file
 *
 * @p offset is kept alongside @p line and @p column because render_diagnostic()
 * needs to find the enclosing line in O(line length) rather than by rescanning
 * the file, and because two locations can be ordered by a single integer compare.
 */
struct SourceLoc {
    uint32_t line = 1;    ///< One-based line number
    uint32_t column = 1;  ///< One-based byte column within the line
    uint32_t offset = 0;  ///< Zero-based byte offset from the start of the source
};

/// How much a diagnostic matters. A file with no Error parses; a Warning is advice.
enum class Severity : uint8_t {
    Error,   ///< The AST is incomplete or wrong here. ParseResult::ok() is false.
    Warning  ///< The AST is usable. Something is suspicious.
};

/**
 * @brief One thing wrong with a rule file
 *
 * The message is a complete sentence fragment in the form
 * `expected <what>, found <spelling>`, or a statement of the fault. It never
 * ends with a full stop: render_diagnostic() puts it after a colon on the first
 * line, and a trailing stop there reads badly.
 *
 * @p length spans the offending token so the caret line can underline all of it.
 * It is at least 1 even at end of file, where there is no token to underline.
 */
struct Diagnostic {
    Severity severity = Severity::Error;
    SourceLoc loc{};
    uint32_t length = 1;      ///< Bytes to underline, starting at @p loc
    std::string message;      ///< What is wrong and what was expected
    std::string token_text;   ///< The offending token as it was spelled, or empty at end of file
};

/**
 * @brief Most diagnostics one parse will produce, lexical and grammatical together
 *
 * Past this many, the file is almost certainly not a rule file at all -- someone
 * pointed the parser at a .png -- and the remaining messages describe the
 * confusion rather than the cause.
 *
 * It lives HERE rather than beside parse(), for the same reason SourceLoc and
 * Diagnostic do: the lexer produces the first diagnostic a file can have, and it
 * has to be able to count against the same budget the parser counts against.
 * When it was declared in parser.hpp alone, the lexer had no cap and a 130 KB
 * image returned 42 907 diagnostics -- 64 MB of rendered text -- from a parse
 * whose header promised at most a hundred.
 */
inline constexpr size_t kMaxDiagnostics = 100;

/**
 * @brief The last diagnostic of a capped list, and the only one either layer adds
 *
 * Shared so that the sentence a caller greps for is one string and not two.
 */
inline constexpr const char* kTooManyDiagnostics =
    "too many errors; this file is not being read as a rule file";

/**
 * @brief Render a diagnostic the way a compiler does, with a caret
 *
 * @code
 * facade.srl:3:20: error: expected ')' to close the argument list, found '{'
 *  3 |     extrude(height {
 *    |                    ^
 * @endcode
 *
 * The source must be the same text the diagnostic was produced from; passing a
 * different string produces a caret under the wrong thing rather than an error,
 * which is why the parser hands its own source back out in ParseResult.
 *
 * @param diagnostic Diagnostic to render
 * @param source     The exact source text the diagnostic came from
 * @param filename   Name to print before the line and column. May be empty.
 * @return A two or three line block, newline-terminated
 */
[[nodiscard]] std::string render_diagnostic(const Diagnostic& diagnostic,
                                            std::string_view source,
                                            std::string_view filename);

// ============================================================================
// Tokens
// ============================================================================

/**
 * @brief The lexical categories of the rule language
 *
 * ### Why there is no Modulo
 *
 * `%` is the relative-size suffix in a split -- `50%` means half the parent
 * extent -- and it is nothing else. CGA spells the same idea `'0.5`, which is
 * unreadable and collides with nothing only because CGA has no character
 * literals. Spelling it `%` is worth giving up a modulo operator that a shape
 * grammar has no use for: with one numeric type and no integers, `a % b` would
 * have been a rounding trap anyway. The parser accepts Percent in exactly one
 * position and says so when it appears anywhere else.
 *
 * ### Why type names and selectors are not keywords
 *
 * `float`, `face`, `front`, `x` and the rest are lexed as Identifier. They are
 * recognised by the parser from a table, in the one grammatical position where
 * each can appear. That keeps them usable as ordinary names elsewhere, and it
 * lets the parser say "unknown type 'flaot' -- did you mean 'float'?" instead of
 * the shrug a keyword mismatch produces.
 */
enum class TokenKind : uint8_t {
    End,         ///< End of input. Always the last token, always exactly one.
    Number,      ///< A numeric literal. Token::number holds the value.
    String,      ///< A string literal. Token::value holds the DECODED text.
    Identifier,  ///< A name, or a contextual word such as `float` or `front`.

    // Keywords, in the order they are declared in the grammar
    KwVersion,
    KwImport,
    KwFrom,
    KwAttr,
    KwConst,
    KwRule,
    KwLet,
    KwIf,
    KwElse,
    KwChoose,
    KwSplit,
    KwSelect,
    KwRepeat,
    KwScope,
    KwDiscard,
    KwTrue,
    KwFalse,

    // Grouping
    LParen,
    RParen,
    LBrace,
    RBrace,
    LBracket,
    RBracket,

    // Punctuation
    Comma,
    Semicolon,
    Colon,
    Dot,
    At,        ///< Introduces an annotation
    Tilde,     ///< Floating-size marker in a split
    Percent,   ///< Relative-size marker in a split. Never modulo; see the enum note.
    Question,  ///< Ternary conditional

    // Operators
    Plus,
    Minus,
    Star,
    Slash,
    Assign,
    EqualEqual,
    BangEqual,
    Bang,
    Less,
    LessEqual,
    Greater,
    GreaterEqual,
    AmpAmp,
    PipePipe
};

/**
 * @brief Spell a token kind for an error message
 *
 * Punctuation and keywords come back quoted, e.g. `"')'"` and `"'rule'"`, so a
 * message reads `expected ')'`. The open categories come back as phrases:
 * `"a number"`, `"a string"`, `"a name"`, `"end of file"`.
 */
[[nodiscard]] const char* token_kind_name(TokenKind kind);

/**
 * @brief One token
 *
 * @warning @p text is a view into the source string passed to tokenize(). The
 *          source must outlive the token vector. The parser copies whatever it
 *          keeps into the AST, so the AST has no such requirement.
 */
struct Token {
    TokenKind kind = TokenKind::End;
    SourceLoc loc{};
    uint32_t length = 0;       ///< Bytes of source this token spans
    std::string_view text;     ///< Raw spelling, quotes and escapes included for a String
    double number = 0.0;       ///< Value of a Number token, otherwise 0
    std::string value;         ///< Decoded value of a String token, otherwise empty
};

/// Tokens plus whatever could not be tokenised
struct LexResult {
    std::vector<Token> tokens;            ///< Always ends with exactly one End token
    std::vector<Diagnostic> diagnostics;  ///< In source order

    /// True when no diagnostic has Severity::Error
    [[nodiscard]] bool ok() const;
};

/**
 * @brief Turn source text into tokens
 *
 * Never throws for malformed input and never stops lexing: an unlexable byte is
 * reported and skipped, an unterminated string is reported and closed at end of
 * line, an unterminated block comment is reported and closed at end of file. The
 * parser therefore always gets a well-formed token stream to work on and can
 * report grammar errors from the rest of the file, which is what makes "fix five
 * mistakes per save" possible instead of "fix one mistake per save".
 *
 * It does stop REPORTING at kMaxDiagnostics, whose last slot then holds
 * kTooManyDiagnostics. Scanning continues, so the token stream is unaffected and
 * so is everything the parser can still say about the file; only the list of
 * lexical complaints is bounded. A file that earns a hundred lexical errors is
 * not a rule file, and the hundredth message is worth less than knowing why the
 * rest are missing.
 *
 * @param source Text to lex. Must outlive the returned tokens; see Token::text.
 * @return Tokens and diagnostics, both in source order
 */
[[nodiscard]] LexResult tokenize(std::string_view source);

} // namespace stratum::procgen::rules
