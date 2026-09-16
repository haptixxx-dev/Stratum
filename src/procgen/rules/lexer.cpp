// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "procgen/rules/lexer.hpp"

#include <algorithm>
#include <charconv>
#include <cstddef>

namespace stratum::procgen::rules {

namespace {

// ============================================================================
// Keyword table
// ============================================================================
// Sorted by spelling, checked by the static_assert below, so a reviewer can see
// at a glance that a word is or is not reserved. The lookup is a linear scan:
// seventeen string_view compares on a word that is already in cache is cheaper
// than a hash, and it keeps the table the single source of truth with no second
// structure to fall out of step with it.

struct KeywordRow {
    std::string_view text;
    TokenKind kind;
};

constexpr KeywordRow kKeywords[] = {
    {"attr", TokenKind::KwAttr},       {"choose", TokenKind::KwChoose},
    {"const", TokenKind::KwConst},     {"discard", TokenKind::KwDiscard},
    {"else", TokenKind::KwElse},       {"false", TokenKind::KwFalse},
    {"from", TokenKind::KwFrom},       {"if", TokenKind::KwIf},
    {"import", TokenKind::KwImport},   {"let", TokenKind::KwLet},
    {"repeat", TokenKind::KwRepeat},   {"rule", TokenKind::KwRule},
    {"scope", TokenKind::KwScope},     {"select", TokenKind::KwSelect},
    {"split", TokenKind::KwSplit},     {"true", TokenKind::KwTrue},
    {"version", TokenKind::KwVersion},
};

constexpr size_t kKeywordCount = sizeof(kKeywords) / sizeof(kKeywords[0]);

constexpr bool keywords_are_sorted() {
    for (size_t i = 1; i < kKeywordCount; ++i) {
        if (!(kKeywords[i - 1].text < kKeywords[i].text)) {
            return false;
        }
    }
    return true;
}

static_assert(keywords_are_sorted(),
              "kKeywords must stay sorted: the table is the language's reserved-word list and "
              "a reviewer reads it as one");

[[nodiscard]] TokenKind keyword_or_identifier(std::string_view word) {
    for (size_t i = 0; i < kKeywordCount; ++i) {
        if (kKeywords[i].text == word) {
            return kKeywords[i].kind;
        }
    }
    return TokenKind::Identifier;
}

[[nodiscard]] constexpr bool is_digit(char c) { return c >= '0' && c <= '9'; }

[[nodiscard]] constexpr bool is_name_start(char c) {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_';
}

[[nodiscard]] constexpr bool is_name_part(char c) { return is_name_start(c) || is_digit(c); }

// ============================================================================
// The scanner
// ============================================================================

class Lexer {
public:
    explicit Lexer(std::string_view source) : source_(source) {
        // One token per three bytes is a rough fit for rule files, which are
        // dense in punctuation. Getting this wrong costs a reallocation, not
        // correctness.
        result_.tokens.reserve(source.size() / 3 + 8);
    }

    LexResult run() {
        for (;;) {
            skip_trivia();
            if (at_end()) {
                break;
            }
            scan_token();
        }
        push_simple(TokenKind::End, here(), 0);
        return std::move(result_);
    }

private:
    // --- position -----------------------------------------------------------

    [[nodiscard]] bool at_end() const { return pos_ >= source_.size(); }
    [[nodiscard]] char peek() const { return at_end() ? '\0' : source_[pos_]; }
    [[nodiscard]] char peek_next() const {
        return pos_ + 1 >= source_.size() ? '\0' : source_[pos_ + 1];
    }
    [[nodiscard]] SourceLoc here() const {
        return SourceLoc{line_, static_cast<uint32_t>(pos_ - line_start_ + 1),
                         static_cast<uint32_t>(pos_)};
    }

    char advance() {
        const char c = source_[pos_++];
        if (c == '\n') {
            ++line_;
            line_start_ = pos_;
        }
        return c;
    }

    bool match(char expected) {
        if (at_end() || source_[pos_] != expected) {
            return false;
        }
        advance();
        return true;
    }

    // --- output -------------------------------------------------------------

    void push_simple(TokenKind kind, SourceLoc loc, size_t length) {
        Token token;
        token.kind = kind;
        token.loc = loc;
        token.length = static_cast<uint32_t>(length);
        token.text = source_.substr(loc.offset, length);
        result_.tokens.push_back(std::move(token));
    }

    /**
     * Scanning never stops -- the parser needs the whole token stream whatever
     * the file turns out to be -- but REPORTING does, at kMaxDiagnostics. The
     * budget is shared with the parser, which counts on top of this list, so a
     * cap enforced only on the parser's own share was no cap at all: a 130 KB
     * .png produced 42 907 lexical diagnostics that rendered to 64 MB. The last
     * slot goes to the cap sentence rather than to a hundredth complaint about
     * the same nonsense, and once it is spent nothing more is added.
     */
    void error(SourceLoc loc, size_t length, std::string message) {
        if (capped_) {
            return;
        }
        if (result_.diagnostics.size() + 1 >= kMaxDiagnostics) {
            capped_ = true;
            Diagnostic cap;
            cap.severity = Severity::Error;
            cap.loc = loc;
            cap.length = 1;
            cap.message = kTooManyDiagnostics;
            result_.diagnostics.push_back(std::move(cap));
            return;
        }
        Diagnostic diagnostic;
        diagnostic.severity = Severity::Error;
        diagnostic.loc = loc;
        diagnostic.length = static_cast<uint32_t>(std::max<size_t>(length, 1));
        diagnostic.message = std::move(message);
        diagnostic.token_text = std::string{source_.substr(loc.offset, std::max<size_t>(length, 1))};
        result_.diagnostics.push_back(std::move(diagnostic));
    }

    // --- trivia -------------------------------------------------------------

    void skip_trivia() {
        for (;;) {
            if (at_end()) {
                return;
            }
            const char c = peek();
            if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
                advance();
                continue;
            }
            if (c == '/' && peek_next() == '/') {
                while (!at_end() && peek() != '\n') {
                    advance();
                }
                continue;
            }
            if (c == '/' && peek_next() == '*') {
                skip_block_comment();
                continue;
            }
            return;
        }
    }

    /**
     * Block comments do NOT nest. A nesting comment is the more forgiving rule,
     * but it makes commenting out a block that contains the characters `/` and
     * `*` inside a STRING change meaning, and a rule file is full of asset paths
     * like "walls/star.png" with a wildcard in it. Non-nesting is the rule every editor
     * highlighter already implements, so the colours in the editor agree with
     * what the lexer did.
     */
    void skip_block_comment() {
        const SourceLoc start = here();
        advance();  // '/'
        advance();  // '*'
        while (!at_end()) {
            if (peek() == '*' && peek_next() == '/') {
                advance();
                advance();
                return;
            }
            advance();
        }
        error(start, 2, "unterminated block comment, opened here and never closed with '*/'");
    }

    // --- tokens -------------------------------------------------------------

    void scan_token() {
        const SourceLoc loc = here();
        const char c = peek();

        if (is_name_start(c)) {
            scan_name(loc);
            return;
        }
        if (is_digit(c)) {
            scan_number(loc);
            return;
        }
        if (c == '"') {
            scan_string(loc);
            return;
        }

        advance();
        switch (c) {
            case '(': push_simple(TokenKind::LParen, loc, 1); return;
            case ')': push_simple(TokenKind::RParen, loc, 1); return;
            case '{': push_simple(TokenKind::LBrace, loc, 1); return;
            case '}': push_simple(TokenKind::RBrace, loc, 1); return;
            case '[': push_simple(TokenKind::LBracket, loc, 1); return;
            case ']': push_simple(TokenKind::RBracket, loc, 1); return;
            case ',': push_simple(TokenKind::Comma, loc, 1); return;
            case ';': push_simple(TokenKind::Semicolon, loc, 1); return;
            case ':': push_simple(TokenKind::Colon, loc, 1); return;
            case '.': push_simple(TokenKind::Dot, loc, 1); return;
            case '@': push_simple(TokenKind::At, loc, 1); return;
            case '~': push_simple(TokenKind::Tilde, loc, 1); return;
            case '%': push_simple(TokenKind::Percent, loc, 1); return;
            case '?': push_simple(TokenKind::Question, loc, 1); return;
            case '+': push_simple(TokenKind::Plus, loc, 1); return;
            case '-': push_simple(TokenKind::Minus, loc, 1); return;
            case '*': push_simple(TokenKind::Star, loc, 1); return;
            case '/': push_simple(TokenKind::Slash, loc, 1); return;
            case '=':
                if (match('=')) {
                    push_simple(TokenKind::EqualEqual, loc, 2);
                } else {
                    push_simple(TokenKind::Assign, loc, 1);
                }
                return;
            case '!':
                if (match('=')) {
                    push_simple(TokenKind::BangEqual, loc, 2);
                } else {
                    push_simple(TokenKind::Bang, loc, 1);
                }
                return;
            case '<':
                if (match('=')) {
                    push_simple(TokenKind::LessEqual, loc, 2);
                } else {
                    push_simple(TokenKind::Less, loc, 1);
                }
                return;
            case '>':
                if (match('=')) {
                    push_simple(TokenKind::GreaterEqual, loc, 2);
                } else {
                    push_simple(TokenKind::Greater, loc, 1);
                }
                return;
            case '&':
                if (match('&')) {
                    push_simple(TokenKind::AmpAmp, loc, 2);
                    return;
                }
                error(loc, 1, "'&' is not an operator; write '&&' for logical and");
                return;
            case '|':
                if (match('|')) {
                    push_simple(TokenKind::PipePipe, loc, 2);
                    return;
                }
                error(loc, 1, "'|' is not an operator; write '||' for logical or");
                return;
            default: break;
        }

        // An unlexable byte produces NO token. Emitting a placeholder token
        // instead would make the parser report a second, derived error for the
        // same character, and two messages for one typo is how a diagnostic list
        // becomes noise a user learns to ignore.
        error(loc, 1, std::string{"unexpected character '"} + c + "' in a rule file");
    }

    void scan_name(SourceLoc loc) {
        while (!at_end() && is_name_part(peek())) {
            advance();
        }
        const size_t length = pos_ - loc.offset;
        push_simple(keyword_or_identifier(source_.substr(loc.offset, length)), loc, length);
    }

    /**
     * Grammar: DIGIT+ [ '.' DIGIT+ ] [ ('e'|'E') ['+'|'-'] DIGIT+ ]
     *
     * A fraction needs a digit on BOTH sides. `1.` and `.5` are not numbers, so
     * `1.` lexes as the number 1 followed by a Dot, and the parser then complains
     * about the dot. That is deliberate: `.` is also the namespace separator in a
     * qualified name, and a lexer that guessed which one was meant would make
     * `facades.Window()` depend on whether the qualifier happened to be numeric.
     */
    void scan_number(SourceLoc loc) {
        while (!at_end() && is_digit(peek())) {
            advance();
        }
        if (peek() == '.' && is_digit(peek_next())) {
            advance();
            while (!at_end() && is_digit(peek())) {
                advance();
            }
        }
        if (peek() == 'e' || peek() == 'E') {
            const size_t save_pos = pos_;
            const uint32_t save_line = line_;
            const size_t save_line_start = line_start_;
            advance();
            if (peek() == '+' || peek() == '-') {
                advance();
            }
            if (is_digit(peek())) {
                while (!at_end() && is_digit(peek())) {
                    advance();
                }
            } else {
                // `2e` with no exponent digits is the number 2 followed by the
                // name `e`, not an error: rolling back keeps the failure inside
                // the grammar, where the message can be about what was expected.
                pos_ = save_pos;
                line_ = save_line;
                line_start_ = save_line_start;
            }
        }

        const size_t length = pos_ - loc.offset;
        const std::string_view text = source_.substr(loc.offset, length);

        double value = 0.0;
        const char* first = text.data();
        const char* last = text.data() + text.size();
        const std::from_chars_result parsed = std::from_chars(first, last, value);
        if (parsed.ec != std::errc{} || parsed.ptr != last) {
            // Reachable for a literal too large for a double, e.g. 1e400.
            error(loc, length, std::string{"'"} + std::string{text} +
                                   "' is not a number a rule file can hold");
            return;
        }

        Token token;
        token.kind = TokenKind::Number;
        token.loc = loc;
        token.length = static_cast<uint32_t>(length);
        token.text = text;
        token.number = value;
        result_.tokens.push_back(std::move(token));
    }

    /**
     * A string is closed by `"`, by end of line, or by end of file. Closing at
     * end of LINE rather than scanning on to the next quote matters: a missing
     * close quote otherwise swallows the rest of the file and reports one error
     * about something a hundred lines below the mistake.
     */
    void scan_string(SourceLoc loc) {
        advance();  // opening quote
        std::string value;
        bool closed = false;

        while (!at_end()) {
            const char c = peek();
            if (c == '\n') {
                break;
            }
            if (c == '"') {
                advance();
                closed = true;
                break;
            }
            if (c == '\\') {
                const SourceLoc escape_loc = here();
                advance();
                if (at_end() || peek() == '\n') {
                    break;
                }
                const char escaped = advance();
                switch (escaped) {
                    case 'n': value.push_back('\n'); break;
                    case 't': value.push_back('\t'); break;
                    case 'r': value.push_back('\r'); break;
                    case '0': value.push_back('\0'); break;
                    case '\\': value.push_back('\\'); break;
                    case '"': value.push_back('"'); break;
                    case '\'': value.push_back('\''); break;
                    default:
                        error(escape_loc, 2,
                              std::string{"unknown escape sequence '\\"} + escaped +
                                  "'; the known ones are \\n \\t \\r \\0 \\\\ \\\" \\'");
                        value.push_back(escaped);
                        break;
                }
                continue;
            }
            value.push_back(advance());
        }

        const size_t length = pos_ - loc.offset;
        if (!closed) {
            error(loc, length, "unterminated string literal; add a closing '\"'");
        }

        Token token;
        token.kind = TokenKind::String;
        token.loc = loc;
        token.length = static_cast<uint32_t>(length);
        token.text = source_.substr(loc.offset, length);
        token.value = std::move(value);
        result_.tokens.push_back(std::move(token));
    }

    std::string_view source_;
    size_t pos_ = 0;
    size_t line_start_ = 0;
    uint32_t line_ = 1;
    /// Set once the diagnostic budget is spent; see error()
    bool capped_ = false;
    LexResult result_;
};

} // namespace

// ============================================================================
// Public entry points
// ============================================================================

bool LexResult::ok() const {
    for (const Diagnostic& diagnostic : diagnostics) {
        if (diagnostic.severity == Severity::Error) {
            return false;
        }
    }
    return true;
}

LexResult tokenize(std::string_view source) {
    return Lexer{source}.run();
}

const char* token_kind_name(TokenKind kind) {
    switch (kind) {
        case TokenKind::End: return "end of file";
        case TokenKind::Number: return "a number";
        case TokenKind::String: return "a string";
        case TokenKind::Identifier: return "a name";
        case TokenKind::KwVersion: return "'version'";
        case TokenKind::KwImport: return "'import'";
        case TokenKind::KwFrom: return "'from'";
        case TokenKind::KwAttr: return "'attr'";
        case TokenKind::KwConst: return "'const'";
        case TokenKind::KwRule: return "'rule'";
        case TokenKind::KwLet: return "'let'";
        case TokenKind::KwIf: return "'if'";
        case TokenKind::KwElse: return "'else'";
        case TokenKind::KwChoose: return "'choose'";
        case TokenKind::KwSplit: return "'split'";
        case TokenKind::KwSelect: return "'select'";
        case TokenKind::KwRepeat: return "'repeat'";
        case TokenKind::KwScope: return "'scope'";
        case TokenKind::KwDiscard: return "'discard'";
        case TokenKind::KwTrue: return "'true'";
        case TokenKind::KwFalse: return "'false'";
        case TokenKind::LParen: return "'('";
        case TokenKind::RParen: return "')'";
        case TokenKind::LBrace: return "'{'";
        case TokenKind::RBrace: return "'}'";
        case TokenKind::LBracket: return "'['";
        case TokenKind::RBracket: return "']'";
        case TokenKind::Comma: return "','";
        case TokenKind::Semicolon: return "';'";
        case TokenKind::Colon: return "':'";
        case TokenKind::Dot: return "'.'";
        case TokenKind::At: return "'@'";
        case TokenKind::Tilde: return "'~'";
        case TokenKind::Percent: return "'%'";
        case TokenKind::Question: return "'?'";
        case TokenKind::Plus: return "'+'";
        case TokenKind::Minus: return "'-'";
        case TokenKind::Star: return "'*'";
        case TokenKind::Slash: return "'/'";
        case TokenKind::Assign: return "'='";
        case TokenKind::EqualEqual: return "'=='";
        case TokenKind::BangEqual: return "'!='";
        case TokenKind::Bang: return "'!'";
        case TokenKind::Less: return "'<'";
        case TokenKind::LessEqual: return "'<='";
        case TokenKind::Greater: return "'>'";
        case TokenKind::GreaterEqual: return "'>='";
        case TokenKind::AmpAmp: return "'&&'";
        case TokenKind::PipePipe: return "'||'";
    }
    return "an unknown token";
}

std::string render_diagnostic(const Diagnostic& diagnostic,
                              std::string_view source,
                              std::string_view filename) {
    // Find the line containing the diagnostic. Clamping rather than trusting the
    // offset keeps a stale diagnostic from indexing out of a shorter source; it
    // then points at the end of the file, which is wrong but not undefined.
    const size_t offset = std::min<size_t>(diagnostic.loc.offset, source.size());
    size_t line_start = offset;
    while (line_start > 0 && source[line_start - 1] != '\n') {
        --line_start;
    }
    size_t line_end = offset;
    while (line_end < source.size() && source[line_end] != '\n') {
        ++line_end;
    }
    // A trailing '\r' belongs to the line terminator, not to the text.
    if (line_end > line_start && source[line_end - 1] == '\r') {
        --line_end;
    }
    const std::string_view line_text = source.substr(line_start, line_end - line_start);

    const std::string line_number = std::to_string(diagnostic.loc.line);
    const std::string gutter(line_number.size(), ' ');

    std::string out;
    if (!filename.empty()) {
        out.append(filename);
        out.push_back(':');
    }
    out.append(line_number);
    out.push_back(':');
    out.append(std::to_string(diagnostic.loc.column));
    out.append(": ");
    out.append(diagnostic.severity == Severity::Error ? "error: " : "warning: ");
    out.append(diagnostic.message);
    out.push_back('\n');

    out.push_back(' ');
    out.append(line_number);
    out.append(" | ");
    out.append(line_text);
    out.push_back('\n');

    out.push_back(' ');
    out.append(gutter);
    out.append(" | ");
    // Echo tabs as tabs so the caret lands under the token in a terminal that
    // rendered the source line above with the same tab stops.
    const size_t caret_column = diagnostic.loc.column == 0 ? 0 : diagnostic.loc.column - 1;
    for (size_t i = 0; i < caret_column; ++i) {
        out.push_back(i < line_text.size() && line_text[i] == '\t' ? '\t' : ' ');
    }
    out.push_back('^');
    const size_t remaining = line_text.size() > caret_column ? line_text.size() - caret_column : 1;
    const size_t underline = std::min<size_t>(std::max<uint32_t>(diagnostic.length, 1), remaining);
    for (size_t i = 1; i < underline; ++i) {
        out.push_back('~');
    }
    out.push_back('\n');

    return out;
}

} // namespace stratum::procgen::rules
