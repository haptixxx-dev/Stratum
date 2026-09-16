// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file parser.hpp
 * @brief The grammar of the Stratum rule language, and the function that applies it
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ================================================================================
 * THE GRAMMAR
 * ================================================================================
 *
 * Written here, above the code, because a grammar that exists only as a set of
 * recursive-descent functions cannot be reviewed and cannot be argued with. The
 * functions in parser.cpp are named after the productions below; if one drifts
 * from the other, this block is the one that is right and the code is the bug.
 *
 * Notation: `{ x }` is zero or more, `[ x ]` is optional, `|` is alternation,
 * UPPERCASE is a token from the lexer, quoted text is a literal token.
 *
 * @code{.ebnf}
 *   ruleFile      = [ versionDecl ] { declaration } EOF ;
 *   versionDecl   = "version" STRING ;
 *
 *   declaration   = { annotation } ( attrDecl | constDecl | ruleDecl )
 *                 | importDecl ;
 *
 *   importDecl    = "import" IDENT "from" STRING ;
 *   attrDecl      = "attr"  IDENT ":" type "=" expression ;
 *   constDecl     = "const" IDENT ":" type "=" expression ;
 *   ruleDecl      = "rule"  IDENT [ "(" [ paramList ] ")" ] block ;
 *
 *   annotation    = "@" IDENT [ "(" [ exprList ] ")" ] ;
 *   paramList     = param { "," param } [ "," ] ;
 *   param         = IDENT ":" type [ "=" expression ] ;
 *   type          = IDENT [ "[" "]" ] ;            (* float | bool | string *)
 *
 *   block         = "{" { statement } "}" ;
 *
 *   statement     = block
 *                 | callStmt
 *                 | letStmt
 *                 | ifStmt
 *                 | chooseStmt
 *                 | splitStmt
 *                 | selectStmt
 *                 | scopeStmt
 *                 | discardStmt ;
 *
 *   callStmt      = qualifiedName "(" [ exprList ] ")" ";" ;
 *   letStmt       = "let" IDENT [ ":" type ] "=" expression ";" ;
 *   ifStmt        = "if" "(" expression ")" block [ "else" ( ifStmt | block ) ] ;
 *   scopeStmt     = "scope" block ;
 *   discardStmt   = "discard" ";" ;
 *
 *   chooseStmt    = "choose" "{" chooseCase { chooseCase } "}" ;
 *   chooseCase    = expression ":" statement ;
 *
 *   splitStmt     = "split" "(" IDENT ")" "{" splitEntry { splitEntry } "}" ;
 *   splitEntry    = sizedEntry
 *                 | "repeat" ( "{" sizedEntry { sizedEntry } "}" | sizedEntry ) ;
 *   sizedEntry    = sizeSpec ":" statement ;
 *   sizeSpec      = [ "~" ] expression [ "%" ] ;
 *
 *   selectStmt    = "select" IDENT "{" selectCase { selectCase } "}" ;
 *   selectCase    = IDENT ":" statement ;
 *
 *   expression    = ternary ;
 *   ternary       = logicalOr [ "?" expression ":" ternary ] ;
 *   logicalOr     = logicalAnd { "||" logicalAnd } ;
 *   logicalAnd    = equality { "&&" equality } ;
 *   equality      = comparison { ( "==" | "!=" ) comparison } ;
 *   comparison    = additive { ( "<" | "<=" | ">" | ">=" ) additive } ;
 *   additive      = multiplicative { ( "+" | "-" ) multiplicative } ;
 *   multiplicative= unary { ( "*" | "/" ) unary } ;
 *   unary         = ( "-" | "!" ) unary | postfix ;
 *   postfix       = primary { "[" expression "]" } ;
 *   primary       = NUMBER | STRING | "true" | "false"
 *                 | "[" [ exprList ] "]"
 *                 | "(" expression ")"
 *                 | qualifiedName [ "(" [ exprList ] ")" ] ;
 *
 *   qualifiedName = IDENT [ "." IDENT ] ;
 *   exprList      = expression { "," expression } [ "," ] ;
 * @endcode
 *
 * ### Two ambiguities that are not ambiguities
 *
 * **A ternary inside a split size.** `split(x) { a > 1 ? 2 : 3 : Wide(); }` has
 * three colons' worth of trouble on the face of it. It parses without a lookahead
 * hack because one `?` consumes exactly one `:` and nothing in the expression
 * grammar consumes a bare colon, so a colon no `?` opened is left for the split.
 * Chaining -- `a ? 1 : b ? 2 : 3 : Wide();` -- works for the same reason: the
 * else branch is itself a `ternary`, so it may open its own `?` and claim its own
 * `:`, and still cannot reach the split's.
 *
 * **`%` and modulo.** There is no modulo operator, so `%` in expression position
 * is always a mistake and always gets the message it deserves. See the note on
 * TokenKind::Percent.
 *
 * ### What the parser checks, and where it stops
 *
 * It checks STRUCTURE, and it resolves names in STATEMENT position:
 *
 *   - every call statement names a declared rule, a built-in operation, or an
 *     import alias -- otherwise it is an error with a spelling suggestion;
 *   - a call to a rule or an operation passes an acceptable number of arguments;
 *   - rules, attributes, constants, import aliases and parameters are unique, and
 *     a rule may not take a built-in operation's name;
 *   - parameters with defaults come last;
 *   - annotations are known, take the right number of arguments, and `@start` is
 *     on a rule and on at most one;
 *   - split axes, component domains, component selectors and type names are known;
 *   - a repeat group does not contain a repeat group;
 *   - a split, a select and a choose each have at least one entry.
 *
 * It does NOT type-check, does not resolve names in EXPRESSION position, does not
 * load imported files, and does not evaluate anything -- not even a constant. All
 * of that is the semantic pass, and it belongs with the interpreter that needs it
 * (D2), not with the parser, because every one of those checks needs a symbol
 * table that spans files and D1 reads one file.
 *
 * ### Recovery
 *
 * One mistake must not hide the next four. After an error the parser skips to the
 * next `;` or `}` at the current nesting depth inside a rule body, or to the next
 * top-level keyword outside one, and carries on. Diagnostics are capped at
 * kMaxDiagnostics -- LEXICAL AND GRAMMATICAL TOGETHER, since the two lists are
 * merged before the caller sees either, and a cap on one of them is not a cap;
 * past that the file is being read as something it is not and more messages are
 * noise. The last diagnostic of a capped list is kTooManyDiagnostics, once.
 * Both constants are declared in lexer.hpp, which this header includes, because
 * the lexer has to count against the same budget.
 *
 * ### Exceptions
 *
 * parse() throws nothing for bad input. Internally a private exception type
 * unwinds from a failed production to the nearest recovery point, which is
 * enormously simpler than threading a failure flag through thirty functions, and
 * it never crosses the public boundary. std::bad_alloc can still escape, as
 * everywhere.
 */

#pragma once

#include "procgen/rules/ast.hpp"
#include "procgen/rules/lexer.hpp"

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

namespace stratum::procgen::rules {

/// A parsed file and everything wrong with it
struct ParseResult {
    RuleFile file;                        ///< Partial when ok() is false, never garbage
    /// In source order, lexical and grammatical interleaved. Never more than
    /// kMaxDiagnostics of them; see the note on recovery above.
    std::vector<Diagnostic> diagnostics;
    std::string filename;                 ///< As passed to parse(), for render_diagnostic()

    /// True when no diagnostic has Severity::Error
    [[nodiscard]] bool ok() const;

    /// Render every diagnostic against @p source. Empty when there are none.
    [[nodiscard]] std::string render_all(std::string_view source) const;
};

/**
 * @brief Parse a rule file
 *
 * @param source   The whole file. Need not outlive the result: everything the AST
 *                 keeps is copied out of it.
 * @param filename Name used in diagnostics. May be empty.
 * @return The AST, plus diagnostics. A result with errors still carries every
 *         declaration the parser got through, so an editor can offer completion
 *         in a file that does not yet parse.
 */
[[nodiscard]] ParseResult parse(std::string_view source, std::string_view filename);

} // namespace stratum::procgen::rules
