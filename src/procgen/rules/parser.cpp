// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "procgen/rules/parser.hpp"

#include <algorithm>
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace stratum::procgen::rules {

// ============================================================================
// Spellings
// ============================================================================

const char* primitive_type_name(PrimitiveType type) {
    switch (type) {
        case PrimitiveType::Float: return "float";
        case PrimitiveType::Bool: return "bool";
        case PrimitiveType::String: return "string";
    }
    return "?";
}

const char* split_axis_name(SplitAxis axis) {
    switch (axis) {
        case SplitAxis::X: return "x";
        case SplitAxis::Y: return "y";
        case SplitAxis::Z: return "z";
    }
    return "?";
}

const char* size_kind_name(SizeKind kind) {
    switch (kind) {
        case SizeKind::Absolute: return "absolute";
        case SizeKind::Relative: return "relative";
        case SizeKind::Floating: return "floating";
        case SizeKind::FloatingRelative: return "floating-relative";
    }
    return "?";
}

const char* component_domain_name(ComponentDomain domain) {
    switch (domain) {
        case ComponentDomain::Face: return "face";
        case ComponentDomain::Edge: return "edge";
        case ComponentDomain::Vertex: return "vertex";
        case ComponentDomain::Object: return "object";
    }
    return "?";
}

const char* component_selector_name(ComponentSelector selector) {
    switch (selector) {
        case ComponentSelector::All: return "all";
        case ComponentSelector::Front: return "front";
        case ComponentSelector::Back: return "back";
        case ComponentSelector::Left: return "left";
        case ComponentSelector::Right: return "right";
        case ComponentSelector::Top: return "top";
        case ComponentSelector::Bottom: return "bottom";
        case ComponentSelector::Side: return "side";
        case ComponentSelector::Vertical: return "vertical";
        case ComponentSelector::Horizontal: return "horizontal";
        case ComponentSelector::Aslant: return "aslant";
    }
    return "?";
}

const char* unary_op_name(UnaryOp op) {
    switch (op) {
        case UnaryOp::Negate: return "-";
        case UnaryOp::Not: return "!";
    }
    return "?";
}

const char* binary_op_name(BinaryOp op) {
    switch (op) {
        case BinaryOp::Add: return "+";
        case BinaryOp::Subtract: return "-";
        case BinaryOp::Multiply: return "*";
        case BinaryOp::Divide: return "/";
        case BinaryOp::Less: return "<";
        case BinaryOp::LessEqual: return "<=";
        case BinaryOp::Greater: return ">";
        case BinaryOp::GreaterEqual: return ">=";
        case BinaryOp::Equal: return "==";
        case BinaryOp::NotEqual: return "!=";
        case BinaryOp::And: return "&&";
        case BinaryOp::Or: return "||";
    }
    return "?";
}

const char* call_target_name(CallTarget target) {
    switch (target) {
        case CallTarget::Rule: return "rule";
        case CallTarget::Operation: return "operation";
        case CallTarget::Imported: return "imported";
        case CallTarget::Unresolved: return "unresolved";
    }
    return "?";
}

const char* annotation_kind_name(AnnotationKind kind) {
    switch (kind) {
        case AnnotationKind::Asset: return "asset";
        case AnnotationKind::Color: return "color";
        case AnnotationKind::Description: return "description";
        case AnnotationKind::Enum: return "enum";
        case AnnotationKind::Group: return "group";
        case AnnotationKind::Hidden: return "hidden";
        case AnnotationKind::Order: return "order";
        case AnnotationKind::Range: return "range";
        case AnnotationKind::Start: return "start";
        case AnnotationKind::Step: return "step";
        case AnnotationKind::Unit: return "unit";
    }
    return "?";
}

namespace {

// ============================================================================
// Contextual word tables
// ============================================================================
// Type names, split axes, component domains and component selectors are lexed as
// plain identifiers and recognised here, each in the single grammatical position
// where it can appear. Reserving them would stop an author calling a parameter
// `top` or a rule `Object`, for no gain -- and it would cost the good message,
// because a keyword that turns up in the wrong place produces "unexpected token"
// while a table lookup produces "unknown face selector 'fornt' -- did you mean
// 'front'?".

template <typename T>
struct WordRow {
    std::string_view text;
    T value;
};

constexpr WordRow<PrimitiveType> kTypeWords[] = {
    {"bool", PrimitiveType::Bool},
    {"float", PrimitiveType::Float},
    {"string", PrimitiveType::String},
};

constexpr WordRow<SplitAxis> kAxisWords[] = {
    {"x", SplitAxis::X},
    {"y", SplitAxis::Y},
    {"z", SplitAxis::Z},
};

constexpr WordRow<ComponentDomain> kDomainWords[] = {
    {"edge", ComponentDomain::Edge},
    {"face", ComponentDomain::Face},
    {"object", ComponentDomain::Object},
    {"vertex", ComponentDomain::Vertex},
};

constexpr WordRow<ComponentSelector> kSelectorWords[] = {
    {"all", ComponentSelector::All},         {"aslant", ComponentSelector::Aslant},
    {"back", ComponentSelector::Back},       {"bottom", ComponentSelector::Bottom},
    {"front", ComponentSelector::Front},     {"horizontal", ComponentSelector::Horizontal},
    {"left", ComponentSelector::Left},       {"right", ComponentSelector::Right},
    {"side", ComponentSelector::Side},       {"top", ComponentSelector::Top},
    {"vertical", ComponentSelector::Vertical},
};

struct AnnotationRow {
    std::string_view text;
    AnnotationKind kind;
    uint8_t min_args;
    uint8_t max_args;
};

constexpr AnnotationRow kAnnotationWords[] = {
    {"asset", AnnotationKind::Asset, 0, 1},
    {"color", AnnotationKind::Color, 0, 0},
    {"description", AnnotationKind::Description, 1, 1},
    {"enum", AnnotationKind::Enum, 1, kVariadic},
    {"group", AnnotationKind::Group, 1, 4},
    {"hidden", AnnotationKind::Hidden, 0, 0},
    {"order", AnnotationKind::Order, 1, 1},
    {"range", AnnotationKind::Range, 2, 2},
    {"start", AnnotationKind::Start, 0, 0},
    {"step", AnnotationKind::Step, 1, 1},
    {"unit", AnnotationKind::Unit, 1, 1},
};

template <typename Row, size_t N>
constexpr bool rows_are_sorted(const Row (&rows)[N]) {
    for (size_t i = 1; i < N; ++i) {
        if (!(rows[i - 1].text < rows[i].text)) {
            return false;
        }
    }
    return true;
}

static_assert(rows_are_sorted(kTypeWords));
static_assert(rows_are_sorted(kAxisWords));
static_assert(rows_are_sorted(kDomainWords));
static_assert(rows_are_sorted(kSelectorWords));
static_assert(rows_are_sorted(kAnnotationWords));

template <typename Row, size_t N>
std::vector<std::string_view> row_names(const Row (&rows)[N]) {
    std::vector<std::string_view> names;
    names.reserve(N);
    for (size_t i = 0; i < N; ++i) {
        names.push_back(rows[i].text);
    }
    return names;
}

/// Comma-separated, quoted, in table order, for "expected one of ..." messages
template <typename Row, size_t N>
std::string row_list(const Row (&rows)[N]) {
    std::string out;
    for (size_t i = 0; i < N; ++i) {
        if (i != 0) {
            out += i + 1 == N ? " or " : ", ";
        }
        out += '\'';
        out.append(rows[i].text);
        out += '\'';
    }
    return out;
}

// ============================================================================
// Spelling suggestions
// ============================================================================
// An unknown name is nearly always a typo or a half-remembered name from CGA.
// Levenshtein over a table of at most forty short words costs nothing at the
// point where the parse has already failed, and it turns "unknown rule or
// operation 'extrud'" into a message that fixes itself.

size_t edit_distance(std::string_view a, std::string_view b) {
    // Two rows rather than a full matrix: the names involved are short, but this
    // also makes the worst case (a garbage token hundreds of bytes long against
    // forty candidates) bounded in memory.
    std::vector<size_t> previous(b.size() + 1);
    std::vector<size_t> current(b.size() + 1);
    for (size_t j = 0; j <= b.size(); ++j) {
        previous[j] = j;
    }
    for (size_t i = 1; i <= a.size(); ++i) {
        current[0] = i;
        for (size_t j = 1; j <= b.size(); ++j) {
            const size_t cost = a[i - 1] == b[j - 1] ? 0u : 1u;
            current[j] = std::min({previous[j] + 1, current[j - 1] + 1, previous[j - 1] + cost});
        }
        previous.swap(current);
    }
    return previous[b.size()];
}

/**
 * @return `" -- did you mean 'x'?"`, or an empty string when nothing is close
 *
 * Ties are broken by taking the FIRST candidate at the best distance -- note the
 * strict `<` below -- and the caller always passes a sequence whose order is
 * fixed: table order for the contextual words (sorted, and static_asserted so),
 * declaration order for rule names. That pair of facts is what keeps the message
 * identical between runs and between platforms; picking "the first match found"
 * out of an unordered container would not.
 *
 * The tie-break is only a real guarantee if something checks it, so two tests do:
 * `an_unknown_split_axis_is_rejected_with_the_list` ('w' is one edit from each of
 * 'x', 'y' and 'z') and `a_tied_spelling_suggestion_names_the_first_candidate`
 * ('Ac' against rules 'Aa' and 'Ab'). Relaxing `<` to `<=` flips both.
 */
std::string suggest(std::string_view unknown, const std::vector<std::string_view>& candidates) {
    constexpr size_t kMaxDistance = 2;
    size_t best = kMaxDistance + 1;
    std::string_view best_name;
    for (std::string_view candidate : candidates) {
        const size_t distance = edit_distance(unknown, candidate);
        if (distance < best) {
            best = distance;
            best_name = candidate;
        }
    }
    if (best > kMaxDistance || best_name.empty()) {
        return {};
    }
    return " -- did you mean '" + std::string{best_name} + "'?";
}

// ============================================================================
// Message helpers
// ============================================================================

std::string count_arguments(size_t n) {
    return std::to_string(n) + (n == 1 ? " argument" : " arguments");
}

std::string expected_arity(uint8_t min_args, uint8_t max_args) {
    if (max_args == kVariadic) {
        return "at least " + count_arguments(min_args);
    }
    if (min_args == max_args) {
        return count_arguments(min_args);
    }
    return std::to_string(min_args) + " to " + count_arguments(max_args);
}

std::string arity_message(std::string_view name, uint8_t min_args, uint8_t max_args, size_t given) {
    return "'" + std::string{name} + "' expects " + expected_arity(min_args, max_args) + ", but " +
           std::to_string(given) + (given == 1 ? " was" : " were") + " given";
}

std::string describe(const Token& token) {
    if (token.kind == TokenKind::End) {
        return "end of file";
    }
    return "'" + std::string{token.text} + "'";
}

// ============================================================================
// The parser
// ============================================================================

/// Thrown by a failed production, caught at the nearest recovery point. Never
/// escapes parse(); see the note in parser.hpp.
struct ParseAbort {};

/// Deepest nesting of expressions or statements the parser will follow
constexpr int kMaxNesting = 128;

class Parser {
public:
    Parser(std::string_view filename, LexResult lexed)
        : filename_(filename), tokens_(std::move(lexed.tokens)),
          diagnostics_(std::move(lexed.diagnostics)) {}

    ParseResult run() {
        parse_file();
        resolve_calls();

        // Lexical diagnostics were produced before any grammatical one, so the
        // two lists are each in source order but the concatenation is not. A
        // stable sort by position interleaves them the way the file reads, which
        // is the order a user fixes them in. Stable, so two diagnostics at the
        // same position keep lexer-then-parser order rather than swapping between
        // runs.
        std::stable_sort(diagnostics_.begin(), diagnostics_.end(),
                         [](const Diagnostic& a, const Diagnostic& b) {
                             return a.loc.offset < b.loc.offset;
                         });

        ParseResult result;
        result.file = std::move(file_);
        result.diagnostics = std::move(diagnostics_);
        result.filename = std::string{filename_};
        return result;
    }

private:
    // --- token access -------------------------------------------------------

    [[nodiscard]] const Token& peek() const { return tokens_[pos_]; }
    [[nodiscard]] bool check(TokenKind kind) const { return tokens_[pos_].kind == kind; }
    [[nodiscard]] bool at_end() const { return tokens_[pos_].kind == TokenKind::End; }

    const Token& advance() {
        const Token& token = tokens_[pos_];
        if (!at_end()) {
            ++pos_;
        }
        return token;
    }

    bool match(TokenKind kind) {
        if (!check(kind)) {
            return false;
        }
        advance();
        return true;
    }

    const Token& expect(TokenKind kind, const char* what) {
        if (check(kind)) {
            return advance();
        }
        error_at(peek(), std::string{"expected "} + what + ", found " + describe(peek()));
    }

    // --- diagnostics --------------------------------------------------------

    /**
     * The cap is a promise about the MERGED list, not about the parser's own
     * share of it. diagnostics_ starts out holding the lexer's, so both of the
     * checks below are needed and both are made BEFORE the push rather than
     * after it: the old code pushed first and then asked whether it had gone too
     * far, which is correct only when the list started empty. Pointed at a .png,
     * where every diagnostic is lexical, it returned 42 907 of them -- 64 MB of
     * rendered text -- with the cap sentence sitting uselessly among them.
     *
     * Exactly one cap sentence ends up in the list: if the lexer already spent
     * the budget it wrote the sentence itself and the first branch stays quiet.
     */
    void report(Severity severity, SourceLoc loc, uint32_t length, std::string token_text,
                std::string message) {
        if (bailed_) {
            return;
        }
        if (diagnostics_.size() >= kMaxDiagnostics) {
            bailed_ = true;  // the lexer filled the budget and said so; nothing to add
            return;
        }
        if (diagnostics_.size() + 1 >= kMaxDiagnostics) {
            // The last slot says why the rest are missing. That is worth more
            // than the hundredth complaint about the same nonsense.
            bailed_ = true;
            Diagnostic cap;
            cap.severity = Severity::Error;
            cap.loc = loc;
            cap.length = 1;
            cap.message = kTooManyDiagnostics;
            diagnostics_.push_back(std::move(cap));
            return;
        }
        Diagnostic diagnostic;
        diagnostic.severity = severity;
        diagnostic.loc = loc;
        diagnostic.length = std::max<uint32_t>(length, 1);
        diagnostic.token_text = std::move(token_text);
        diagnostic.message = std::move(message);
        diagnostics_.push_back(std::move(diagnostic));
    }

    void report_at(const Token& token, std::string message) {
        report(Severity::Error, token.loc, token.length, std::string{token.text},
               std::move(message));
    }

    void report_at(SourceLoc loc, std::string_view text, std::string message) {
        report(Severity::Error, loc, static_cast<uint32_t>(text.size()), std::string{text},
               std::move(message));
    }

    [[noreturn]] void error_at(const Token& token, std::string message) {
        report_at(token, std::move(message));
        throw ParseAbort{};
    }

    /// `; previously declared at line N` -- the second half of every duplicate message
    static std::string previously(SourceLoc loc) {
        return "; previously declared at line " + std::to_string(loc.line) + ", column " +
               std::to_string(loc.column);
    }

    // --- arenas -------------------------------------------------------------

    ExprId add_expr(SourceLoc loc, ExprNode node) {
        file_.exprs.push_back(Expr{loc, std::move(node)});
        return static_cast<ExprId>(file_.exprs.size() - 1);
    }

    StmtId add_stmt(SourceLoc loc, StmtNode node) {
        file_.stmts.push_back(Stmt{loc, std::move(node)});
        return static_cast<StmtId>(file_.stmts.size() - 1);
    }

    /// RAII guard against a file engineered to recurse the parser off the stack
    class Nesting {
    public:
        explicit Nesting(Parser& parser) : parser_(parser) {
            if (++parser_.depth_ > kMaxNesting) {
                parser_.report_at(parser_.peek(),
                                  "nested too deeply here; the limit is " +
                                      std::to_string(kMaxNesting) + " levels");
                --parser_.depth_;
                throw ParseAbort{};
            }
        }
        ~Nesting() { --parser_.depth_; }
        Nesting(const Nesting&) = delete;
        Nesting& operator=(const Nesting&) = delete;

    private:
        Parser& parser_;
    };

    /// Permits a trailing '%' for the lifetime of the guard; see parse_size_spec()
    class PercentGuard {
    public:
        explicit PercentGuard(Parser& parser) : parser_(parser), saved_(parser.percent_allowed_) {
            parser_.percent_allowed_ = true;
        }
        ~PercentGuard() { parser_.percent_allowed_ = saved_; }
        PercentGuard(const PercentGuard&) = delete;
        PercentGuard& operator=(const PercentGuard&) = delete;

    private:
        Parser& parser_;
        bool saved_;
    };

    // --- recovery -----------------------------------------------------------

    void synchronize_statement() {
        int depth = 0;
        while (!at_end()) {
            const TokenKind kind = peek().kind;
            if (kind == TokenKind::LBrace) {
                ++depth;
                advance();
                continue;
            }
            if (kind == TokenKind::RBrace) {
                if (depth == 0) {
                    return;  // the enclosing block's loop wants this one
                }
                --depth;
                advance();
                continue;
            }
            if (kind == TokenKind::Semicolon) {
                advance();
                if (depth == 0) {
                    return;
                }
                continue;
            }
            advance();
        }
    }

    void synchronize_declaration() {
        while (!at_end()) {
            switch (peek().kind) {
                case TokenKind::KwRule:
                case TokenKind::KwAttr:
                case TokenKind::KwConst:
                case TokenKind::KwImport:
                case TokenKind::KwVersion:
                    return;
                default: advance();
            }
        }
    }

    // --- file ---------------------------------------------------------------

    void parse_file() {
        if (check(TokenKind::KwVersion)) {
            try {
                const Token& keyword = advance();
                const Token& text = expect(TokenKind::String, "a version string, as in version \"0.1\"");
                file_.version = text.value;
                file_.version_loc = keyword.loc;
            } catch (const ParseAbort&) {
                synchronize_declaration();
            }
        }

        while (!bailed_ && !at_end()) {
            const size_t before = pos_;
            try {
                parse_declaration();
            } catch (const ParseAbort&) {
                synchronize_declaration();
            }
            // The same backstop as in parse_block(), at file level. The one
            // input that used to reach it was a misplaced `version`, reported on
            // a token it did not consume and then found again by
            // synchronize_declaration(), which stops at KwVersion; the second,
            // derived "expected a declaration" that produced is why the keyword
            // is now consumed before it is reported. Kept for the same reason
            // the other four are.
            if (pos_ == before && !at_end()) {
                advance();
            }
        }
    }

    void parse_declaration() {
        std::vector<Annotation> annotations;
        while (check(TokenKind::At)) {
            annotations.push_back(parse_annotation());
        }

        switch (peek().kind) {
            case TokenKind::KwAttr: parse_attr(std::move(annotations)); return;
            case TokenKind::KwConst: parse_const(std::move(annotations)); return;
            case TokenKind::KwRule: parse_rule(std::move(annotations)); return;
            case TokenKind::KwImport:
                if (!annotations.empty()) {
                    error_at(peek(),
                             "an import cannot carry annotations; move them to the declaration "
                             "they describe");
                }
                parse_import();
                return;
            case TokenKind::KwVersion: {
                // Consumed BEFORE it is reported. Recovery skips to the next
                // top-level keyword and 'version' is one of them, so leaving the
                // cursor on it made synchronize_declaration() stop exactly where
                // it started; parse_file()'s no-progress guard then stepped onto
                // the version STRING and reported it a second time as "expected a
                // declaration". One mistake, two messages. The string goes too,
                // so recovery restarts after the whole misplaced declaration.
                const Token& keyword = advance();
                match(TokenKind::String);
                error_at(keyword,
                         "'version' must come before every other declaration in the file");
            }
            default:
                error_at(peek(), "expected a declaration -- 'import', 'attr', 'const' or 'rule' "
                                 "-- found " +
                                     describe(peek()));
        }
    }

    Annotation parse_annotation() {
        const Token& at = expect(TokenKind::At, "'@' to start an annotation");
        const Token& name = expect(TokenKind::Identifier, "an annotation name after '@'");

        const AnnotationRow* row = nullptr;
        for (const AnnotationRow& candidate : kAnnotationWords) {
            if (candidate.text == name.text) {
                row = &candidate;
                break;
            }
        }
        if (row == nullptr) {
            error_at(name, "unknown annotation '@" + std::string{name.text} + "'" +
                               suggest(name.text, row_names(kAnnotationWords)));
        }

        Annotation annotation;
        annotation.kind = row->kind;
        annotation.name = std::string{name.text};
        annotation.loc = at.loc;

        if (match(TokenKind::LParen)) {
            if (!check(TokenKind::RParen)) {
                annotation.args = parse_expression_list();
            }
            expect(TokenKind::RParen, "')' to close the annotation arguments");
        }

        if (annotation.args.size() < static_cast<size_t>(row->min_args) ||
            (row->max_args != kVariadic &&
             annotation.args.size() > static_cast<size_t>(row->max_args))) {
            report_at(name, arity_message("@" + std::string{name.text}, row->min_args,
                                          row->max_args, annotation.args.size()));
        }
        return annotation;
    }

    void parse_import() {
        const Token& keyword = expect(TokenKind::KwImport, "'import'");
        const Token& alias = expect(TokenKind::Identifier,
                                    "an alias after 'import', as in import facades from \"...\"");
        expect(TokenKind::KwFrom, "'from' after the import alias");
        const Token& path = expect(TokenKind::String, "a quoted path after 'from'");

        for (const ImportDecl& existing : file_.imports) {
            if (existing.alias == alias.text) {
                report_at(alias, "duplicate import alias '" + std::string{alias.text} + "'" +
                                     previously(existing.loc));
                break;
            }
        }

        ImportDecl decl;
        decl.alias = std::string{alias.text};
        decl.path = path.value;
        decl.loc = keyword.loc;
        decl.path_loc = path.loc;
        file_.imports.push_back(std::move(decl));
    }

    /// Attributes and constants share one value namespace, so one lookup serves both
    void check_value_name_unique(const Token& name) {
        for (const AttrDecl& existing : file_.attributes) {
            if (existing.name == name.text) {
                report_at(name, "duplicate declaration of '" + std::string{name.text} + "'" +
                                    previously(existing.loc));
                return;
            }
        }
        for (const ConstDecl& existing : file_.constants) {
            if (existing.name == name.text) {
                report_at(name, "duplicate declaration of '" + std::string{name.text} + "'" +
                                    previously(existing.loc));
                return;
            }
        }
    }

    void reject_start_annotation(const std::vector<Annotation>& annotations, const char* what) {
        for (const Annotation& annotation : annotations) {
            if (annotation.kind == AnnotationKind::Start) {
                report(Severity::Error, annotation.loc, 6, "@start",
                       std::string{"'@start' marks the rule a file begins at; it cannot go on "} +
                           what);
            }
        }
    }

    void parse_attr(std::vector<Annotation> annotations) {
        const Token& keyword = expect(TokenKind::KwAttr, "'attr'");
        const Token& name = expect(TokenKind::Identifier, "a name after 'attr'");
        check_value_name_unique(name);
        reject_start_annotation(annotations, "an attribute");

        expect(TokenKind::Colon, "':' and a type after the attribute name");
        const TypeRef type = parse_type();
        expect(TokenKind::Assign,
               "'=' and a default value; every attribute needs one so the Inspector always has "
               "something to show");
        const ExprId value = parse_expression();

        AttrDecl decl;
        decl.name = std::string{name.text};
        decl.type = type;
        decl.default_value = value;
        decl.annotations = std::move(annotations);
        decl.loc = keyword.loc;
        file_.attributes.push_back(std::move(decl));
    }

    void parse_const(std::vector<Annotation> annotations) {
        const Token& keyword = expect(TokenKind::KwConst, "'const'");
        const Token& name = expect(TokenKind::Identifier, "a name after 'const'");
        check_value_name_unique(name);
        reject_start_annotation(annotations, "a constant");

        expect(TokenKind::Colon, "':' and a type after the constant name");
        const TypeRef type = parse_type();
        expect(TokenKind::Assign, "'=' and a value");
        const ExprId value = parse_expression();

        ConstDecl decl;
        decl.name = std::string{name.text};
        decl.type = type;
        decl.value = value;
        decl.annotations = std::move(annotations);
        decl.loc = keyword.loc;
        file_.constants.push_back(std::move(decl));
    }

    void parse_rule(std::vector<Annotation> annotations) {
        const Token& keyword = expect(TokenKind::KwRule, "'rule'");
        const Token& name = expect(TokenKind::Identifier, "a name after 'rule'");

        for (const RuleDecl& existing : file_.rules) {
            if (existing.name == name.text) {
                report_at(name, "duplicate rule '" + std::string{name.text} + "'" +
                                    previously(existing.loc));
                break;
            }
        }
        if (find_builtin_operation(name.text) != kNoNode) {
            // Allowing the shadow would make every call site ambiguous, and the
            // winner would depend on lookup order -- exactly the kind of rule that
            // is fine until a library adds an operation with your rule's name.
            report_at(name, "a rule cannot be called '" + std::string{name.text} +
                                "': that is the name of a built-in operation");
        }

        RuleDecl decl;
        decl.name = std::string{name.text};
        decl.loc = keyword.loc;

        if (match(TokenKind::LParen)) {
            if (!check(TokenKind::RParen)) {
                parse_parameters(decl);
            }
            expect(TokenKind::RParen, "')' to close the parameter list");
        }

        decl.body = parse_block();

        // The index THIS declaration will have once it is pushed, which is after
        // the loop below. That matters: with two '@start' on the SAME rule, the
        // second one finds file_.start_rule already set to this pending index,
        // and file_.rules does not contain it yet. Indexing file_.rules with it
        // was an out-of-bounds read that took the process down -- an editor that
        // re-parses on every keystroke died the moment someone typed '@start'
        // twice, which parser.hpp's "parse() throws nothing for bad input"
        // promises cannot happen. The name is therefore resolved against decl
        // whenever the winning index is the pending one.
        const RuleId pending_id = static_cast<RuleId>(file_.rules.size());
        for (const Annotation& annotation : annotations) {
            if (annotation.kind != AnnotationKind::Start) {
                continue;
            }
            if (file_.start_rule != kNoNode) {
                const std::string& owner = file_.start_rule < file_.rules.size()
                                               ? file_.rules[file_.start_rule].name
                                               : decl.name;
                report(Severity::Error, annotation.loc, 6, "@start",
                       "only one rule in a file can be marked '@start'; '" + owner +
                           "' already is");
            } else {
                decl.is_start = true;
                file_.start_rule = pending_id;
            }
        }
        decl.annotations = std::move(annotations);
        file_.rules.push_back(std::move(decl));
    }

    void parse_parameters(RuleDecl& decl) {
        bool seen_default = false;
        for (;;) {
            const Token& name = expect(TokenKind::Identifier, "a parameter name");
            for (const Param& existing : decl.params) {
                if (existing.name == name.text) {
                    report_at(name, "duplicate parameter '" + std::string{name.text} + "'" +
                                        previously(existing.loc));
                    break;
                }
            }
            expect(TokenKind::Colon,
                   "':' and a type; every parameter is typed, so that a caller gets a message "
                   "rather than a surprise");
            Param param;
            param.name = std::string{name.text};
            param.type = parse_type();
            param.loc = name.loc;

            if (match(TokenKind::Assign)) {
                param.default_value = parse_expression();
                seen_default = true;
            } else if (seen_default) {
                // Without this, `Rule(1)` would have to guess which parameter the
                // argument belongs to.
                report_at(name, "parameter '" + std::string{name.text} +
                                    "' has no default but follows one that does; move the "
                                    "parameters with defaults to the end");
            }

            decl.params.push_back(std::move(param));
            if (!match(TokenKind::Comma)) {
                return;
            }
            if (check(TokenKind::RParen)) {
                return;  // trailing comma
            }
        }
    }

    TypeRef parse_type() {
        const Token& name = expect(TokenKind::Identifier, "a type name");
        TypeRef type;
        type.loc = name.loc;
        bool found = false;
        for (const WordRow<PrimitiveType>& row : kTypeWords) {
            if (row.text == name.text) {
                type.type = row.value;
                found = true;
                break;
            }
        }
        if (!found) {
            error_at(name, "unknown type '" + std::string{name.text} + "'; the types are " +
                               row_list(kTypeWords) + suggest(name.text, row_names(kTypeWords)));
        }
        if (match(TokenKind::LBracket)) {
            expect(TokenKind::RBracket, "']' -- an array type is written 'float[]'");
            type.is_array = true;
        }
        return type;
    }

    // --- statements ---------------------------------------------------------

    StmtId parse_block() {
        const Token& open = expect(TokenKind::LBrace, "'{' to open a block");
        const SourceLoc loc = open.loc;
        BlockStmt block;
        while (!bailed_ && !check(TokenKind::RBrace) && !at_end()) {
            const size_t before = pos_;
            try {
                block.statements.push_back(parse_statement());
            } catch (const ParseAbort&) {
                synchronize_statement();
            }
            // BACKSTOP, and as of today an unreachable one. This loop terminates
            // because of an invariant spread over a dozen functions: every
            // production consumes at least one token before it can succeed, and
            // synchronize_statement() either advances or stops on a '}' that the
            // loop condition above already tests for. No input reaches the
            // advance() below -- a reviewer confirmed it with 40 000 fuzz inputs
            // and by deleting all four copies of it -- and it is kept anyway,
            // because the invariant is a property of every production added from
            // here on, not a property of this function. The day one of them can
            // succeed on zero tokens, the cost of being wrong is an editor that
            // hangs on a keystroke, against one integer compare per statement.
            // The RBrace test is load-bearing: without it this would eat the
            // brace the enclosing production is waiting for.
            if (pos_ == before && !check(TokenKind::RBrace) && !at_end()) {
                advance();
            }
        }
        expect(TokenKind::RBrace, "'}' to close the block");
        return add_stmt(loc, BlockStmt{std::move(block)});
    }

    StmtId parse_statement() {
        const Nesting guard{*this};
        switch (peek().kind) {
            case TokenKind::LBrace: return parse_block();
            case TokenKind::KwLet: return parse_let();
            case TokenKind::KwIf: return parse_if();
            case TokenKind::KwChoose: return parse_choose();
            case TokenKind::KwSplit: return parse_split();
            case TokenKind::KwSelect: return parse_select();
            case TokenKind::KwScope: return parse_scope();
            case TokenKind::KwDiscard: return parse_discard();
            case TokenKind::Identifier: return parse_call_statement();
            default:
                error_at(peek(),
                         "expected a statement -- a rule or operation call, 'let', 'if', "
                         "'choose', 'split', 'select', 'scope' or 'discard' -- found " +
                             describe(peek()));
        }
    }

    /// A case body is always stored as a block, so `4: Floor();` and
    /// `4: { Floor(); }` produce the same tree and D3 has one shape to walk.
    StmtId parse_case_body() {
        const SourceLoc loc = peek().loc;
        const StmtId body = parse_statement();
        if (std::holds_alternative<BlockStmt>(file_.stmts[body].node)) {
            return body;
        }
        BlockStmt block;
        block.statements.push_back(body);
        return add_stmt(loc, BlockStmt{std::move(block)});
    }

    QualifiedName parse_qualified_name() {
        const Token& first = expect(TokenKind::Identifier, "a name");
        QualifiedName qualified;
        qualified.name = std::string{first.text};
        qualified.loc = first.loc;
        if (match(TokenKind::Dot)) {
            const Token& second = expect(TokenKind::Identifier,
                                         "a name after '.'; a dot means \"from that import\"");
            qualified.qualifier = std::move(qualified.name);
            qualified.name = std::string{second.text};
        }
        return qualified;
    }

    StmtId parse_call_statement() {
        const SourceLoc loc = peek().loc;
        CallStmt call;
        call.callee = parse_qualified_name();
        expect(TokenKind::LParen,
               "'(' after the name; a rule or operation is always called with parentheses, even "
               "with no arguments");
        if (!check(TokenKind::RParen)) {
            call.args = parse_expression_list();
        }
        expect(TokenKind::RParen, "')' to close the argument list");
        expect(TokenKind::Semicolon, "';' after the call");
        return add_stmt(loc, CallStmt{std::move(call)});
    }

    StmtId parse_let() {
        const Token& keyword = expect(TokenKind::KwLet, "'let'");
        const Token& name = expect(TokenKind::Identifier, "a name after 'let'");
        LetStmt let;
        let.name = std::string{name.text};
        if (match(TokenKind::Colon)) {
            let.has_type = true;
            let.type = parse_type();
        }
        expect(TokenKind::Assign, "'=' after the name; a local always has a value");
        let.value = parse_expression();
        expect(TokenKind::Semicolon, "';' after the local");
        return add_stmt(keyword.loc, LetStmt{std::move(let)});
    }

    StmtId parse_if() {
        const Token& keyword = expect(TokenKind::KwIf, "'if'");
        expect(TokenKind::LParen, "'(' after 'if'");
        IfStmt statement;
        statement.condition = parse_expression();
        expect(TokenKind::RParen, "')' to close the condition");
        statement.then_branch = parse_block();
        if (match(TokenKind::KwElse)) {
            // `else if` chains as a nested IfStmt rather than as a case list: it
            // keeps the tree the same shape as the source and costs nothing.
            //
            // The chain needs its own guard for the same reason parse_ternary's
            // else branch does: this recursion goes straight back into parse_if
            // and never passes through parse_statement's guard again, so a chain
            // of 200 000 `else if` ran the stack out instead of reporting. The
            // `else { ... }` arm needs none -- parse_block reaches
            // parse_statement, which is guarded.
            if (check(TokenKind::KwIf)) {
                const Nesting guard{*this};
                statement.else_branch = parse_if();
            } else {
                statement.else_branch = parse_block();
            }
        }
        return add_stmt(keyword.loc, IfStmt{statement});
    }

    StmtId parse_scope() {
        const Token& keyword = expect(TokenKind::KwScope, "'scope'");
        ScopeStmt statement;
        statement.body = parse_block();
        return add_stmt(keyword.loc, ScopeStmt{statement});
    }

    StmtId parse_discard() {
        const Token& keyword = expect(TokenKind::KwDiscard, "'discard'");
        expect(TokenKind::Semicolon, "';' after 'discard'");
        return add_stmt(keyword.loc, DiscardStmt{});
    }

    StmtId parse_choose() {
        const Token& keyword = expect(TokenKind::KwChoose, "'choose'");
        expect(TokenKind::LBrace, "'{' to open the alternatives");
        ChooseStmt statement;
        while (!bailed_ && !check(TokenKind::RBrace) && !at_end()) {
            const size_t before = pos_;
            try {
                ChooseCase entry;
                entry.loc = peek().loc;
                entry.weight = parse_expression();
                reject_negative_literal_weight(entry.weight);
                expect(TokenKind::Colon, "':' after the weight");
                entry.body = parse_case_body();
                statement.cases.push_back(entry);
            } catch (const ParseAbort&) {
                synchronize_statement();
            }
            if (pos_ == before && !check(TokenKind::RBrace) && !at_end()) {
                advance();  // backstop; see the note in parse_block()
            }
        }
        const Token& close = expect(TokenKind::RBrace, "'}' to close the alternatives");
        if (statement.cases.empty()) {
            report_at(close, "a 'choose' needs at least one alternative");
        }
        return add_stmt(keyword.loc, ChooseStmt{std::move(statement)});
    }

    /**
     * Only a literal is checked. The weights are normalised by their sum at
     * generation time, so a negative one poisons every other arm in the block;
     * catching the written-down case here is cheap, and catching the computed
     * case needs an evaluator, which is D5's.
     */
    void reject_negative_literal_weight(ExprId weight) {
        const Expr& expr = file_.exprs[weight];
        if (!std::holds_alternative<UnaryExpr>(expr.node)) {
            return;
        }
        const UnaryExpr& unary = std::get<UnaryExpr>(expr.node);
        if (unary.op != UnaryOp::Negate || unary.operand == kNoNode) {
            return;
        }
        if (!std::holds_alternative<NumberExpr>(file_.exprs[unary.operand].node)) {
            return;
        }
        report(Severity::Error, expr.loc, 1, "-",
               "a 'choose' weight cannot be negative; weights are relative shares and are "
               "normalised by their sum");
    }

    StmtId parse_split() {
        const Token& keyword = expect(TokenKind::KwSplit, "'split'");
        expect(TokenKind::LParen, "'(' and an axis after 'split'");
        const Token& axis_token = expect(TokenKind::Identifier, "a split axis");
        SplitStmt statement;
        bool found = false;
        for (const WordRow<SplitAxis>& row : kAxisWords) {
            if (row.text == axis_token.text) {
                statement.axis = row.value;
                found = true;
                break;
            }
        }
        if (!found) {
            error_at(axis_token, "unknown split axis '" + std::string{axis_token.text} +
                                     "'; the axes are " + row_list(kAxisWords) +
                                     suggest(axis_token.text, row_names(kAxisWords)));
        }
        expect(TokenKind::RParen, "')' after the split axis");
        expect(TokenKind::LBrace, "'{' to open the split");

        while (!bailed_ && !check(TokenKind::RBrace) && !at_end()) {
            const size_t before = pos_;
            try {
                statement.entries.push_back(parse_split_entry(/*inside_repeat=*/false));
            } catch (const ParseAbort&) {
                synchronize_statement();
            }
            if (pos_ == before && !check(TokenKind::RBrace) && !at_end()) {
                advance();  // backstop; see the note in parse_block()
            }
        }
        const Token& close = expect(TokenKind::RBrace, "'}' to close the split");
        if (statement.entries.empty()) {
            report_at(close, "a 'split' needs at least one entry");
        }
        return add_stmt(keyword.loc, SplitStmt{std::move(statement)});
    }

    SplitEntry parse_split_entry(bool inside_repeat) {
        if (check(TokenKind::KwRepeat)) {
            if (inside_repeat) {
                error_at(peek(),
                         "a repeat group cannot contain another repeat group; the outer "
                         "repetition count would depend on the inner one");
            }
            const Token& keyword = advance();
            SplitEntry entry;
            entry.is_repeat = true;
            entry.loc = keyword.loc;

            if (match(TokenKind::LBrace)) {
                while (!bailed_ && !check(TokenKind::RBrace) && !at_end()) {
                    const size_t before = pos_;
                    try {
                        entry.children.push_back(parse_split_entry(/*inside_repeat=*/true));
                    } catch (const ParseAbort&) {
                        synchronize_statement();
                    }
                    if (pos_ == before && !check(TokenKind::RBrace) && !at_end()) {
                        advance();  // backstop; see the note in parse_block()
                    }
                }
                const Token& close = expect(TokenKind::RBrace, "'}' to close the repeat group");
                if (entry.children.empty()) {
                    report_at(close, "a repeat group needs at least one entry");
                }
            } else {
                // `repeat ~3.0 : Floor();` is sugar for a group of one. The tree
                // is the same either way, so D3 never sees the difference.
                entry.children.push_back(parse_split_entry(/*inside_repeat=*/true));
            }
            return entry;
        }

        SplitEntry entry;
        entry.loc = peek().loc;
        entry.size = parse_size_spec();
        expect(TokenKind::Colon, "':' after the split size");
        entry.body = parse_case_body();
        return entry;
    }

    SizeSpec parse_size_spec() {
        SizeSpec size;
        size.loc = peek().loc;
        const bool floating = match(TokenKind::Tilde);
        // The one position where a trailing '%' is legal. Saved and restored
        // through a guard rather than by two assignments: the expression can
        // throw, and a permission left switched on by a failed parse would
        // silently disable the modulo message for the rest of the file.
        {
            const PercentGuard allow{*this};
            size.value = parse_expression();
        }
        const bool relative = match(TokenKind::Percent);
        if (floating) {
            size.kind = relative ? SizeKind::FloatingRelative : SizeKind::Floating;
        } else {
            size.kind = relative ? SizeKind::Relative : SizeKind::Absolute;
        }
        return size;
    }

    StmtId parse_select() {
        const Token& keyword = expect(TokenKind::KwSelect, "'select'");
        const Token& domain_token = expect(TokenKind::Identifier,
                                           "a component domain after 'select'");
        SelectStmt statement;
        bool found = false;
        for (const WordRow<ComponentDomain>& row : kDomainWords) {
            if (row.text == domain_token.text) {
                statement.domain = row.value;
                found = true;
                break;
            }
        }
        if (!found) {
            error_at(domain_token, "unknown component domain '" + std::string{domain_token.text} +
                                       "'; the domains are " + row_list(kDomainWords) +
                                       suggest(domain_token.text, row_names(kDomainWords)));
        }
        expect(TokenKind::LBrace, "'{' to open the selectors");

        while (!bailed_ && !check(TokenKind::RBrace) && !at_end()) {
            const size_t before = pos_;
            try {
                const Token& selector_token = expect(TokenKind::Identifier, "a component selector");
                SelectCase entry;
                entry.loc = selector_token.loc;
                bool known = false;
                for (const WordRow<ComponentSelector>& row : kSelectorWords) {
                    if (row.text == selector_token.text) {
                        entry.selector = row.value;
                        known = true;
                        break;
                    }
                }
                if (!known) {
                    error_at(selector_token,
                             "unknown " + std::string{component_domain_name(statement.domain)} +
                                 " selector '" + std::string{selector_token.text} +
                                 "'; the selectors are " + row_list(kSelectorWords) +
                                 suggest(selector_token.text, row_names(kSelectorWords)));
                }
                for (const SelectCase& existing : statement.cases) {
                    if (existing.selector == entry.selector) {
                        report_at(selector_token,
                                  "duplicate selector '" + std::string{selector_token.text} + "'" +
                                      previously(existing.loc));
                        break;
                    }
                }
                expect(TokenKind::Colon, "':' after the selector");
                entry.body = parse_case_body();
                statement.cases.push_back(entry);
            } catch (const ParseAbort&) {
                synchronize_statement();
            }
            if (pos_ == before && !check(TokenKind::RBrace) && !at_end()) {
                advance();  // backstop; see the note in parse_block()
            }
        }
        const Token& close = expect(TokenKind::RBrace, "'}' to close the selectors");
        if (statement.cases.empty()) {
            report_at(close, "a 'select' needs at least one selector");
        }
        return add_stmt(keyword.loc, SelectStmt{std::move(statement)});
    }

    // --- expressions --------------------------------------------------------

    std::vector<ExprId> parse_expression_list() {
        std::vector<ExprId> list;
        for (;;) {
            list.push_back(parse_expression());
            if (!match(TokenKind::Comma)) {
                return list;
            }
            if (check(TokenKind::RParen) || check(TokenKind::RBracket)) {
                return list;  // trailing comma
            }
        }
    }

    ExprId parse_expression() {
        const Nesting guard{*this};
        return parse_ternary();
    }

    ExprId parse_ternary() {
        const ExprId condition = parse_logical_or();
        if (!check(TokenKind::Question)) {
            return condition;
        }
        const Token& question = advance();
        TernaryExpr ternary;
        ternary.condition = condition;
        ternary.then_value = parse_expression();
        expect(TokenKind::Colon, "':' in the conditional");
        // The else branch is a ternary so that conditionals CHAIN to the right:
        // `a ? 1 : b ? 2 : 3` nests rather than failing at the second '?'. What
        // keeps a conditional usable as a split size -- `a ? 2 : 3 : Wide();` --
        // is the line above: one '?' consumes exactly one ':', so a colon this
        // call did not open is left for the enclosing production. Parsing the
        // else branch at logicalOr level would break the chaining; parsing it as
        // `expression` would be the same function and the same behaviour, since
        // `expression` IS `ternary`.
        //
        // Guarded HERE and not only in parse_expression, because this is a tail
        // call into the same function: the enclosing parse_expression has already
        // returned its guard by the time a chain gets going, so depth_ never
        // accumulated and kMaxNesting was enforced on every expression shape
        // except this one. `1 ? 1 : 1 ? 1 : ...` parsed clean at 20 000 links and
        // dumped core at 50 000. One guard per link is exactly the chain length.
        {
            const Nesting guard{*this};
            ternary.else_value = parse_ternary();
        }
        return add_expr(question.loc, ternary);
    }

    ExprId parse_logical_or() {
        ExprId lhs = parse_logical_and();
        while (check(TokenKind::PipePipe)) {
            const Token& op = advance();
            const ExprId rhs = parse_logical_and();
            lhs = add_expr(op.loc, BinaryExpr{BinaryOp::Or, lhs, rhs});
        }
        return lhs;
    }

    ExprId parse_logical_and() {
        ExprId lhs = parse_equality();
        while (check(TokenKind::AmpAmp)) {
            const Token& op = advance();
            const ExprId rhs = parse_equality();
            lhs = add_expr(op.loc, BinaryExpr{BinaryOp::And, lhs, rhs});
        }
        return lhs;
    }

    ExprId parse_equality() {
        ExprId lhs = parse_comparison();
        for (;;) {
            BinaryOp op{};
            if (check(TokenKind::EqualEqual)) {
                op = BinaryOp::Equal;
            } else if (check(TokenKind::BangEqual)) {
                op = BinaryOp::NotEqual;
            } else {
                return lhs;
            }
            const Token& token = advance();
            const ExprId rhs = parse_comparison();
            lhs = add_expr(token.loc, BinaryExpr{op, lhs, rhs});
        }
    }

    ExprId parse_comparison() {
        ExprId lhs = parse_additive();
        for (;;) {
            BinaryOp op{};
            if (check(TokenKind::Less)) {
                op = BinaryOp::Less;
            } else if (check(TokenKind::LessEqual)) {
                op = BinaryOp::LessEqual;
            } else if (check(TokenKind::Greater)) {
                op = BinaryOp::Greater;
            } else if (check(TokenKind::GreaterEqual)) {
                op = BinaryOp::GreaterEqual;
            } else {
                return lhs;
            }
            const Token& token = advance();
            const ExprId rhs = parse_additive();
            lhs = add_expr(token.loc, BinaryExpr{op, lhs, rhs});
        }
    }

    ExprId parse_additive() {
        ExprId lhs = parse_multiplicative();
        for (;;) {
            BinaryOp op{};
            if (check(TokenKind::Plus)) {
                op = BinaryOp::Add;
            } else if (check(TokenKind::Minus)) {
                op = BinaryOp::Subtract;
            } else {
                return lhs;
            }
            const Token& token = advance();
            const ExprId rhs = parse_multiplicative();
            lhs = add_expr(token.loc, BinaryExpr{op, lhs, rhs});
        }
    }

    ExprId parse_multiplicative() {
        ExprId lhs = parse_unary();
        for (;;) {
            // '%' is the relative-size suffix and nothing else, so anywhere an
            // infix operator could go it is a mistake -- almost always someone
            // reaching for modulo. Caught HERE rather than left to the enclosing
            // production, which would otherwise report it as "expected a
            // declaration, found '%'" three lines further on.
            if (check(TokenKind::Percent) && !percent_allowed_) {
                error_at(peek(),
                         "'%' is only a size suffix inside a split, as in '50%'; there is no "
                         "modulo operator");
            }
            BinaryOp op{};
            if (check(TokenKind::Star)) {
                op = BinaryOp::Multiply;
            } else if (check(TokenKind::Slash)) {
                op = BinaryOp::Divide;
            } else {
                return lhs;
            }
            const Token& token = advance();
            const ExprId rhs = parse_unary();
            lhs = add_expr(token.loc, BinaryExpr{op, lhs, rhs});
        }
    }

    ExprId parse_unary() {
        if (check(TokenKind::Minus) || check(TokenKind::Bang)) {
            const Nesting guard{*this};
            const Token& token = advance();
            const UnaryOp op = token.kind == TokenKind::Minus ? UnaryOp::Negate : UnaryOp::Not;
            const ExprId operand = parse_unary();
            return add_expr(token.loc, UnaryExpr{op, operand});
        }
        return parse_postfix();
    }

    ExprId parse_postfix() {
        ExprId base = parse_primary();
        while (check(TokenKind::LBracket)) {
            const Token& open = advance();
            const ExprId index = parse_expression();
            expect(TokenKind::RBracket, "']' to close the index");
            base = add_expr(open.loc, IndexExpr{base, index});
        }
        return base;
    }

    ExprId parse_primary() {
        const Nesting guard{*this};
        const Token& token = peek();
        switch (token.kind) {
            case TokenKind::Number: {
                const Token& number = advance();
                return add_expr(number.loc, NumberExpr{number.number});
            }
            case TokenKind::String: {
                const Token& text = advance();
                return add_expr(text.loc, StringExpr{text.value});
            }
            case TokenKind::KwTrue: {
                const Token& keyword = advance();
                return add_expr(keyword.loc, BoolExpr{true});
            }
            case TokenKind::KwFalse: {
                const Token& keyword = advance();
                return add_expr(keyword.loc, BoolExpr{false});
            }
            case TokenKind::LParen: {
                advance();
                const ExprId inner = parse_expression();
                expect(TokenKind::RParen, "')' to close the group");
                return inner;
            }
            case TokenKind::LBracket: {
                const Token& open = advance();
                ArrayExpr array;
                if (!check(TokenKind::RBracket)) {
                    array.elements = parse_expression_list();
                }
                expect(TokenKind::RBracket, "']' to close the array");
                return add_expr(open.loc, ArrayExpr{std::move(array)});
            }
            case TokenKind::Identifier: {
                const SourceLoc loc = token.loc;
                QualifiedName name = parse_qualified_name();
                if (!match(TokenKind::LParen)) {
                    return add_expr(loc, NameExpr{std::move(name)});
                }
                CallExpr call;
                call.callee = std::move(name);
                if (!check(TokenKind::RParen)) {
                    call.args = parse_expression_list();
                }
                expect(TokenKind::RParen, "')' to close the function arguments");
                return add_expr(loc, CallExpr{std::move(call)});
            }
            case TokenKind::Percent:
                error_at(token,
                         "'%' is only a size suffix inside a split, as in '50%'; there is no "
                         "modulo operator");
            case TokenKind::Tilde:
                error_at(token,
                         "'~' marks a floating size and is only valid at the start of a split "
                         "entry");
            default:
                error_at(token, "expected an expression, found " + describe(token));
        }
    }

    // --- resolution ---------------------------------------------------------

    /**
     * Resolve every call statement, after the whole file is parsed so that a rule
     * may be called above its declaration.
     *
     * The walk is a linear pass over the statement arena rather than a recursive
     * descent of the tree: every statement in the file is in that vector exactly
     * once, whatever its depth, so the pass cannot miss a branch and cannot visit
     * one twice. It also visits them in creation order, which is source order,
     * so the diagnostics come out in a stable sequence.
     */
    void resolve_calls() {
        std::vector<std::string_view> rule_names;
        rule_names.reserve(file_.rules.size());
        for (const RuleDecl& rule : file_.rules) {
            rule_names.push_back(rule.name);
        }
        std::vector<std::string_view> operation_names;
        operation_names.reserve(kBuiltinOperationCount + file_.rules.size());
        for (size_t i = 0; i < kBuiltinOperationCount; ++i) {
            operation_names.push_back(kBuiltinOperations[i].name);
        }
        // Rules after operations: when a name is equally close to both, the
        // built-in wins the tie, because a misspelled built-in is the commoner
        // mistake and the ordering has to be decided by something fixed.
        std::vector<std::string_view> callable_names = operation_names;
        callable_names.insert(callable_names.end(), rule_names.begin(), rule_names.end());

        for (Stmt& statement : file_.stmts) {
            if (!std::holds_alternative<CallStmt>(statement.node)) {
                continue;
            }
            CallStmt& call = std::get<CallStmt>(statement.node);

            if (call.callee.qualified()) {
                bool known = false;
                for (const ImportDecl& import : file_.imports) {
                    if (import.alias == call.callee.qualifier) {
                        known = true;
                        break;
                    }
                }
                if (!known) {
                    std::vector<std::string_view> aliases;
                    aliases.reserve(file_.imports.size());
                    for (const ImportDecl& import : file_.imports) {
                        aliases.push_back(import.alias);
                    }
                    report_at(call.callee.loc, call.callee.qualifier,
                              "unknown import alias '" + call.callee.qualifier +
                                  "'; declare it with 'import " + call.callee.qualifier +
                                  " from \"...\"'" + suggest(call.callee.qualifier, aliases));
                    call.target = CallTarget::Unresolved;
                    continue;
                }
                // The other file is not loaded here, so the argument count cannot
                // be checked. Deferring is correct: guessing would produce an
                // error about a rule this parser has never seen.
                call.target = CallTarget::Imported;
                continue;
            }

            bool matched = false;
            for (size_t i = 0; i < file_.rules.size(); ++i) {
                if (file_.rules[i].name == call.callee.name) {
                    call.target = CallTarget::Rule;
                    call.rule = static_cast<RuleId>(i);
                    check_rule_arity(call, file_.rules[i]);
                    matched = true;
                    break;
                }
            }
            if (matched) {
                continue;
            }

            const uint32_t operation = find_builtin_operation(call.callee.name);
            if (operation != kNoNode) {
                call.target = CallTarget::Operation;
                call.operation = operation;
                const BuiltinOperation& builtin = kBuiltinOperations[operation];
                if (call.args.size() < static_cast<size_t>(builtin.min_args) ||
                    (builtin.max_args != kVariadic &&
                     call.args.size() > static_cast<size_t>(builtin.max_args))) {
                    report_at(call.callee.loc, call.callee.name,
                              arity_message(builtin.name, builtin.min_args, builtin.max_args,
                                            call.args.size()));
                }
                continue;
            }

            call.target = CallTarget::Unresolved;
            report_at(call.callee.loc, call.callee.name,
                      "unknown rule or operation '" + call.callee.name + "'" +
                          suggest(call.callee.name, callable_names));
        }
    }

    void check_rule_arity(const CallStmt& call, const RuleDecl& rule) {
        size_t required = 0;
        for (const Param& param : rule.params) {
            if (param.default_value == kNoNode) {
                ++required;
            }
        }
        if (call.args.size() >= required && call.args.size() <= rule.params.size()) {
            return;
        }
        report_at(call.callee.loc, call.callee.name,
                  arity_message(rule.name, static_cast<uint8_t>(required),
                                static_cast<uint8_t>(rule.params.size()), call.args.size()));
    }

    std::string_view filename_;
    std::vector<Token> tokens_;
    std::vector<Diagnostic> diagnostics_;
    RuleFile file_;
    size_t pos_ = 0;
    int depth_ = 0;
    bool bailed_ = false;
    /// True only while the size expression of a split entry is being parsed
    bool percent_allowed_ = false;
};

// ============================================================================
// Canonical dump
// ============================================================================

std::string format_number(double value) {
    // std::to_chars is the only formatter the standard requires to produce the
    // shortest representation that reads back as the same double. printf("%.17g")
    // is correctly rounded on glibc and not guaranteed elsewhere, and the dump is
    // the thing the determinism tests compare, so "on any platform" has to be a
    // guarantee and not an observation.
    char buffer[64];
    const std::to_chars_result result =
        std::to_chars(buffer, buffer + sizeof(buffer), value);
    if (result.ec != std::errc{}) {
        return "?";
    }
    return std::string(buffer, result.ptr);
}

std::string quote(std::string_view text) {
    std::string out = "\"";
    for (const char c : text) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\t': out += "\\t"; break;
            case '\r': out += "\\r"; break;
            default: out.push_back(c); break;
        }
    }
    out.push_back('"');
    return out;
}

std::string type_text(const TypeRef& type) {
    return std::string{primitive_type_name(type.type)} + (type.is_array ? "[]" : "");
}

std::string expr_text(const RuleFile& file, ExprId id) {
    if (id == kNoNode || id >= file.exprs.size()) {
        return "(none)";
    }
    const Expr& expr = file.exprs[id];
    switch (expr_kind(expr)) {
        case ExprKind::Number:
            return "(num " + format_number(std::get<NumberExpr>(expr.node).value) + ")";
        case ExprKind::String:
            return "(str " + quote(std::get<StringExpr>(expr.node).value) + ")";
        case ExprKind::Bool:
            return std::string{"(bool "} + (std::get<BoolExpr>(expr.node).value ? "true" : "false") +
                   ")";
        case ExprKind::Name: return "(name " + std::get<NameExpr>(expr.node).name.text() + ")";
        case ExprKind::Array: {
            std::string out = "(array";
            for (const ExprId element : std::get<ArrayExpr>(expr.node).elements) {
                out += " " + expr_text(file, element);
            }
            return out + ")";
        }
        case ExprKind::Index: {
            const IndexExpr& index = std::get<IndexExpr>(expr.node);
            return "(index " + expr_text(file, index.base) + " " + expr_text(file, index.index) +
                   ")";
        }
        case ExprKind::Unary: {
            const UnaryExpr& unary = std::get<UnaryExpr>(expr.node);
            return std::string{"(unary "} + unary_op_name(unary.op) + " " +
                   expr_text(file, unary.operand) + ")";
        }
        case ExprKind::Binary: {
            const BinaryExpr& binary = std::get<BinaryExpr>(expr.node);
            return std::string{"(binary "} + binary_op_name(binary.op) + " " +
                   expr_text(file, binary.lhs) + " " + expr_text(file, binary.rhs) + ")";
        }
        case ExprKind::Ternary: {
            const TernaryExpr& ternary = std::get<TernaryExpr>(expr.node);
            return "(ternary " + expr_text(file, ternary.condition) + " " +
                   expr_text(file, ternary.then_value) + " " + expr_text(file, ternary.else_value) +
                   ")";
        }
        case ExprKind::Call: {
            const CallExpr& call = std::get<CallExpr>(expr.node);
            std::string out = "(fcall " + call.callee.text();
            for (const ExprId arg : call.args) {
                out += " " + expr_text(file, arg);
            }
            return out + ")";
        }
    }
    return "(?)";
}

std::string indent_of(int columns) { return std::string(static_cast<size_t>(columns), ' '); }

std::string stmt_text(const RuleFile& file, StmtId id, int indent);

std::string entry_text(const RuleFile& file, const SplitEntry& entry, int indent) {
    if (entry.is_repeat) {
        std::string out = "(repeat";
        for (const SplitEntry& child : entry.children) {
            out += "\n" + indent_of(indent + 2) + entry_text(file, child, indent + 2);
        }
        return out + ")";
    }
    std::string out = std::string{"(entry "} + size_kind_name(entry.size.kind) + " " +
                      expr_text(file, entry.size.value);
    out += "\n" + indent_of(indent + 2) + stmt_text(file, entry.body, indent + 2);
    return out + ")";
}

std::string stmt_text(const RuleFile& file, StmtId id, int indent) {
    if (id == kNoNode || id >= file.stmts.size()) {
        return "(none)";
    }
    const Stmt& statement = file.stmts[id];
    const std::string child_indent = indent_of(indent + 2);

    switch (stmt_kind(statement)) {
        case StmtKind::Block: {
            std::string out = "(block";
            for (const StmtId child : std::get<BlockStmt>(statement.node).statements) {
                out += "\n" + child_indent + stmt_text(file, child, indent + 2);
            }
            return out + ")";
        }
        case StmtKind::Call: {
            const CallStmt& call = std::get<CallStmt>(statement.node);
            std::string out = std::string{"(call "} + call_target_name(call.target) + " " +
                              call.callee.text();
            for (const ExprId arg : call.args) {
                out += " " + expr_text(file, arg);
            }
            return out + ")";
        }
        case StmtKind::Let: {
            const LetStmt& let = std::get<LetStmt>(statement.node);
            return "(let " + let.name + " " + (let.has_type ? type_text(let.type) : "-") + " " +
                   expr_text(file, let.value) + ")";
        }
        case StmtKind::If: {
            const IfStmt& branch = std::get<IfStmt>(statement.node);
            std::string out = "(if " + expr_text(file, branch.condition);
            out += "\n" + child_indent + stmt_text(file, branch.then_branch, indent + 2);
            out += "\n" + child_indent + stmt_text(file, branch.else_branch, indent + 2);
            return out + ")";
        }
        case StmtKind::Choose: {
            std::string out = "(choose";
            for (const ChooseCase& entry : std::get<ChooseStmt>(statement.node).cases) {
                out += "\n" + child_indent + "(case " + expr_text(file, entry.weight);
                out += "\n" + indent_of(indent + 4) + stmt_text(file, entry.body, indent + 4) + ")";
            }
            return out + ")";
        }
        case StmtKind::Split: {
            const SplitStmt& split = std::get<SplitStmt>(statement.node);
            std::string out = std::string{"(split "} + split_axis_name(split.axis);
            for (const SplitEntry& entry : split.entries) {
                out += "\n" + child_indent + entry_text(file, entry, indent + 2);
            }
            return out + ")";
        }
        case StmtKind::Select: {
            const SelectStmt& select = std::get<SelectStmt>(statement.node);
            std::string out = std::string{"(select "} + component_domain_name(select.domain);
            for (const SelectCase& entry : select.cases) {
                out += "\n" + child_indent + "(case " + component_selector_name(entry.selector);
                out += "\n" + indent_of(indent + 4) + stmt_text(file, entry.body, indent + 4) + ")";
            }
            return out + ")";
        }
        case StmtKind::Scope: {
            const ScopeStmt& scope = std::get<ScopeStmt>(statement.node);
            return "(scope\n" + child_indent + stmt_text(file, scope.body, indent + 2) + ")";
        }
        case StmtKind::Discard: return "(discard)";
    }
    return "(?)";
}

std::string annotations_text(const RuleFile& file, const std::vector<Annotation>& annotations) {
    std::string out;
    for (const Annotation& annotation : annotations) {
        out += " (annotation " + std::string{annotation_kind_name(annotation.kind)};
        for (const ExprId arg : annotation.args) {
            out += " " + expr_text(file, arg);
        }
        out += ")";
    }
    return out;
}

} // namespace

std::string dump(const RuleFile& file) {
    std::string out = "(rule-file";
    if (!file.version.empty()) {
        out += "\n  (version " + quote(file.version) + ")";
    }
    for (const ImportDecl& import : file.imports) {
        out += "\n  (import " + import.alias + " " + quote(import.path) + ")";
    }
    for (const AttrDecl& attr : file.attributes) {
        out += "\n  (attr " + attr.name + " " + type_text(attr.type) +
               annotations_text(file, attr.annotations) + " (default " +
               expr_text(file, attr.default_value) + "))";
    }
    for (const ConstDecl& constant : file.constants) {
        out += "\n  (const " + constant.name + " " + type_text(constant.type) +
               annotations_text(file, constant.annotations) + " (value " +
               expr_text(file, constant.value) + "))";
    }
    for (const RuleDecl& rule : file.rules) {
        out += "\n  (rule " + rule.name + " " + (rule.is_start ? "start" : "-") +
               annotations_text(file, rule.annotations);
        for (const Param& param : rule.params) {
            out += "\n    (param " + param.name + " " + type_text(param.type) + " ";
            out += param.default_value == kNoNode
                       ? std::string{"required"}
                       : "(default " + expr_text(file, param.default_value) + ")";
            out += ")";
        }
        out += "\n    " + stmt_text(file, rule.body, 4) + ")";
    }
    out += ")\n";
    return out;
}

// ============================================================================
// Public entry points
// ============================================================================

bool ParseResult::ok() const {
    for (const Diagnostic& diagnostic : diagnostics) {
        if (diagnostic.severity == Severity::Error) {
            return false;
        }
    }
    return true;
}

std::string ParseResult::render_all(std::string_view source) const {
    std::string out;
    for (const Diagnostic& diagnostic : diagnostics) {
        out += render_diagnostic(diagnostic, source, filename);
    }
    return out;
}

ParseResult parse(std::string_view source, std::string_view filename) {
    Parser parser{filename, tokenize(source)};
    return parser.run();
}

} // namespace stratum::procgen::rules
