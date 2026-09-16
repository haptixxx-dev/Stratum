// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file op_control.hpp
 * @brief D6: user attributes with provenance, shape tags, and per-shape stochastic draws
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ================================================================================
 * WHAT THIS IS
 * ================================================================================
 *
 * D2 gave the language an evaluator that already BRANCHES: `if` and `choose` are
 * statements in ast.hpp and interpreter.cpp executes both. What D2 did not give
 * it is anything worth branching ON, and that is what this file adds:
 *
 *   - **User attributes.** `attr height : float = 12.0` is declared, defaulted
 *     and overridable per generation in D2 -- but a rule could only read the one
 *     value the run settled on, could not write one, and could not ask where it
 *     came from. `set(name, value)` writes an attribute onto the shape, children
 *     inherit it, and `attrs.source(name)` reports which rung of the chain
 *     answered.
 *   - **Tags.** A set of words on a shape, which is what a condition over
 *     "is this a corner lot" is actually written against.
 *   - **Stochastic draws in EXPRESSION position.** `choose` picks a statement
 *     branch; `random.range(2.8, 3.4)` picks a NUMBER, which is what makes a
 *     street of houses differ in storey height rather than only in shape.
 *
 * Everything here registers through interpreter.hpp's two tables.
 * interpreter.cpp is untouched. See "THE ONE THING THE EXTENSION POINT CANNOT
 * DO" below, which is a real hole and is named rather than hidden.
 *
 * ================================================================================
 * PROVENANCE: A VALUE PLUS WHERE IT CAME FROM
 * ================================================================================
 *
 * scene/attributes.hpp makes the argument at length and this file follows it:
 * asking "what is this building's height" has several answers and the NUMBER is
 * only half of each. A store that returns only the value makes *did my edit
 * stick, or is something overriding it?* unanswerable, and every "why is this
 * building still 9 m" report unfixable.
 *
 * The rule language's chain, in ascending precedence, is AttrSource:
 *
 *   - **None** -- no `attr` declares the name and no shape holds it.
 *   - **Default** -- the `attr` declaration's own default expression.
 *   - **Supplied** -- GenerationOptions::attributes, which is what an Inspector
 *     slider or a layer's value arrives as.
 *   - **Shape** -- `set()` wrote it on this shape, or on an ancestor this shape
 *     inherited from.
 *
 * Mapped onto scene/attributes.hpp's AttributeSource: Default is Default,
 * Supplied is the Layer/User rung reaching the generation from outside the rule
 * file, and Shape is Object -- "the importer or a rule wrote it onto this
 * object", which is exactly what `set()` does. The names differ because the
 * chains differ: a generation has no undo stack and no layer handle, and reusing
 * a five-rung enum for a four-rung chain would leave two enumerators that can
 * never be returned.
 *
 * **Shape outranks Supplied on purpose**, which is the opposite way round from
 * a first guess. The supplied value is the whole building's; a `set()` is one
 * rule, at one line, deciding something about one shape and its subtree. Letting
 * the global value win would make `set("material", "glass")` on the ground floor
 * silently do nothing, and the author would have no way to say what they meant.
 * This is also how scene/attributes.hpp orders Object above Layer.
 *
 * ================================================================================
 * THE ONE THING THE EXTENSION POINT CANNOT DO
 * ================================================================================
 *
 * A bare NAME in a rule -- `height` -- is resolved by the interpreter's own
 * lookup_name(): locals, then the file's attributes and constants. It never
 * consults Shape::attributes, and nothing a registered operation or function can
 * do changes that. So a value written by `set("height", 20)` is read back with
 * `attrs.get("height")` and NOT with `height`.
 *
 * That is a hole in the language, not a design. Closing it is six lines in
 * interpreter.cpp's lookup_name(), between the locals search and
 * resolve_global():
 *
 *     if (!in_global) {
 *         const auto found = shape.attributes.find(name.name);
 *         if (found != shape.attributes.end()) { return found->second; }
 *     }
 *
 * The precedence that produces is exactly the one attribute_source() already
 * reports, and OpControl.the_precedence_of_the_chain_is_pinned pins it, so the
 * day those lines land the two ways of reading agree by construction rather than
 * by luck. Until they do, `attrs.get()` is the only read that sees a `set()`,
 * and it says so at the line that asked when it cannot answer.
 *
 * ================================================================================
 * DETERMINISM
 * ================================================================================
 *
 * The hard part, and not optional. Same rule text, same seed, byte-identical
 * output, *regardless of how many shapes were generated before this one*. A
 * draw from one shared stream makes every building depend on the count of the
 * buildings before it, and regenerating one block then requires regenerating the
 * city.
 *
 * Every draw here is
 *
 *     seed_unit(seed_mix2(shape.seed_key, seed_mix2(kSaltControlDraw, loc.offset)))
 *
 * which is the form interpreter.cpp's `choose` already uses, for the same
 * reasons. There is no state: seed_mix2() is a finaliser, not a generator, so
 * there is no stream position to leak from one shape to the next.
 * Shape::seed_key is the shape's ADDRESS in the tree, mixed from its parent's
 * key and its sibling index.
 *
 * Two consequences the reader should expect rather than discover:
 *
 *   - **Two draws at one call site on one shape are the same number.**
 *     `random.unit() - random.unit()` written twice at one site is zero. Call
 *     sites differ by byte offset, so `translate(random.unit(), 0, random.unit())`
 *     does get two numbers, and so does any pair of separate lines. This is
 *     referential transparency, and it is what the address-derived design buys:
 *     the alternative is a per-shape counter, which reintroduces "this value
 *     depends on how many draws happened before it" one scope down.
 *   - **Editing text ABOVE a draw changes it.** Byte offsets move. A generation
 *     is reproducible from the same file, not across edits to it. `choose`
 *     already behaves this way; making the draw depend on anything more stable
 *     would mean an identity the parser does not record.
 */

#pragma once

#include "procgen/rules/ast.hpp"
#include "procgen/rules/interpreter.hpp"
#include "procgen/rules/lexer.hpp"
#include "procgen/rules/shape.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace stratum::procgen::rules {

// ============================================================================
// Salt
// ============================================================================

/**
 * @brief The salt every draw in this file mixes with its call site
 *
 * Distinct from shape.hpp's kSaltRoot, kSaltChild, kSaltChoose and kSaltOp, and
 * far away from them numerically, so that a draw made here can never coincide
 * with the interpreter's own however the source offsets fall. It is declared
 * here rather than added to shape.hpp's enum because an operation family must be
 * addable without editing the files it plugs into, and a salt is just a number
 * nobody else uses.
 */
enum : uint64_t { kSaltControlDraw = 0xD6000001ull };

// ============================================================================
// Attribute provenance
// ============================================================================

/**
 * @brief Which rung of the resolution chain answered a read
 *
 * Ascending precedence: a later enumerator beats an earlier one. See the
 * provenance section at the top of the file for the mapping onto
 * scene/attributes.hpp's AttributeSource.
 */
enum class AttrSource : uint8_t {
    None = 0,  ///< No `attr` declares it and no shape holds it
    Default,   ///< The `attr` declaration's default expression
    Supplied,  ///< GenerationOptions::attributes -- an Inspector or a layer
    Shape      ///< `set()` on this shape, or inherited from an ancestor
};

/// "none", "default", "supplied" or "shape" -- the words `attrs.source()` returns
[[nodiscard]] const char* attr_source_name(AttrSource source);

/**
 * @brief Which rung would answer a read of @p name, without evaluating anything
 *
 * Pure, and deliberately independent of whether the value can be COMPUTED: a
 * default that divides by zero is still the rung that would answer, and an
 * author debugging it needs to be told that before they are told it failed.
 *
 * A supplied value counts only when an `attr` declares the name AND the declared
 * type matches, because that is exactly when the interpreter uses it -- run()
 * reports an undeclared override and resolve_global() reports a mistyped one and
 * falls back to the default. A provenance that disagreed with that would be
 * worse than none: it would confirm an override that is not in effect.
 */
[[nodiscard]] AttrSource attribute_source(const RuleFile& file,
                                          const GenerationOptions& options,
                                          const Shape& shape,
                                          std::string_view name);

// ============================================================================
// Tags
// ============================================================================

/**
 * @brief Where a shape's tags live inside Shape::attributes
 *
 * A reserved key rather than a new field on Shape, because Shape is D2's and a
 * feature that needed a member on it could not be added without editing the file
 * it belongs to. The leading '#' cannot be spelled by the lexer as an
 * identifier, and `set()` refuses a name that starts with it, so the reserved
 * space stays reserved rather than merely being unlikely.
 *
 * Stored as an array of strings, so the tags survive into dump_shape() -- which
 * is what makes a determinism test over tagged output able to fail.
 */
inline constexpr std::string_view kTagsAttribute = "#tags";

/**
 * @brief The shape's tags, sorted and unique
 *
 * Sorted rather than in the order `tag()` was called: a tag set is a SET, two
 * rules that add the same tags in different orders describe the same shape, and
 * a canonical dump that differed between them would be a false failure in the
 * determinism suite. Returns empty for a shape with no tags or with a `#tags`
 * entry of some other type, which a caller can only produce by writing the
 * reserved key itself.
 */
[[nodiscard]] std::vector<std::string> shape_tags(const Shape& shape);

/// Does the shape carry @p tag?
[[nodiscard]] bool shape_has_tag(const Shape& shape, std::string_view tag);

/// Add @p tag. @return false when the shape already had it, which is not an error.
bool add_shape_tag(Shape& shape, std::string_view tag);

/// Remove every tag, and the reserved entry with them
void clear_shape_tags(Shape& shape);

// ============================================================================
// Weighted choice
// ============================================================================

/**
 * @brief Choose an index from relative weights, given a draw in [0, 1)
 *
 * Split out of the `random.weighted` handler and exposed because the rules that
 * matter here hold at the BOUNDARIES of the draw -- at exactly 0, and at the
 * largest double below 1 -- and a draw derived from a shape's address cannot be
 * steered to either from a rule file. Logic a test cannot reach is logic that is
 * not tested, and "a zero weight is never chosen" is the whole point of the
 * function.
 *
 * Two promises, and both are structural rather than a matter of which way an
 * inequality faces:
 *
 *   - **A weight that is not positive is never chosen**, for any @p unit. Such
 *     entries are skipped rather than contributing a zero-width interval that
 *     an exact-zero draw could still land on.
 *   - **The answer is always an entry with a positive weight**, including for a
 *     @p unit at or above 1. A trailing zero-weight entry is never the answer.
 *
 * seed_unit() never returns 1, so that second case cannot arise from a draw made
 * here -- but the bound is the caller's promise rather than this function's, and
 * an out-of-range @p unit that fell off the end of the scan would otherwise
 * return whatever the last entry happened to be. Handling it makes the line
 * reachable, which is what makes it testable; a defensive return no input can
 * reach is a defensive return nothing checks.
 *
 * Weights are relative and normalised by their sum, so `{1, 1}` and `{50, 50}`
 * mean the same thing. A negative or NaN weight is REFUSED by the caller before
 * it gets here; this function skips it rather than trusting it.
 *
 * @param weights Relative weights, any length
 * @param unit    A draw, normally in [0, 1)
 * @return The chosen index, or weights.size() when no weight is positive
 */
[[nodiscard]] size_t weighted_index(const std::vector<double>& weights, double unit);

// ============================================================================
// Literal defaults and the declared schema
// ============================================================================

/**
 * @brief Evaluate @p id if and only if it is a literal
 *
 * Numbers, strings, booleans, arrays of those, and a unary `-` or `!` on one.
 * Anything else -- a name, a call, an arithmetic expression -- returns false
 * rather than a guess, because evaluating it properly is the interpreter's job
 * and a second evaluator would be a second set of semantics to keep in step.
 *
 * This is what lets an Inspector show the declared default of `attr height :
 * float = 12.0` without running a generation, and what lets `attrs.get()`
 * answer for the overwhelmingly common declaration.
 *
 * @return false when @p id is kNoNode or is not a literal; @p out is untouched
 */
[[nodiscard]] bool literal_value(const RuleFile& file, ExprId id, Value& out);

/**
 * @brief One declared attribute, with its annotations decoded
 *
 * ast.hpp records annotations as name plus argument expressions and checks only
 * their COUNT. Nothing decoded them, and an annotation nothing reacts to is
 * exactly what ast.hpp says an annotation must not be: "a silently ignored
 * `@rnage(0, 10)` looks exactly like a working one until someone opens the panel
 * and finds a free-text box". This is the decode, and it reports the arguments
 * it cannot use.
 */
struct AttrInfo {
    std::string name;
    TypeRef type{};
    SourceLoc loc{};

    /// True when the default is a literal and @p default_value holds it
    bool default_is_literal = false;
    Value default_value{};

    std::string description;  ///< @description
    std::string unit;         ///< @unit
    std::vector<std::string> group;  ///< @group("Massing", "Height"), outermost first
    std::string asset_filter;        ///< @asset("*.obj"), empty when @asset had no filter

    /// @enum values, each already checked against @p type
    std::vector<Value> enum_values;

    bool is_asset = false;  ///< Carries @asset
    bool is_color = false;  ///< Carries @color
    bool hidden = false;    ///< Carries @hidden

    bool has_range = false;
    double range_min = 0.0;
    double range_max = 0.0;

    bool has_step = false;
    double step = 0.0;

    bool has_order = false;
    double order = 0.0;
};

/**
 * @brief Every `attr` declaration in the file, in source order, with diagnostics
 *
 * The diagnostics are the annotation arguments the parser accepted by count but
 * that cannot be used: `@range("a", "b")`, `@range(10, 0)`, `@step(0)`, an
 * `@enum` value of the wrong type, a repeated `@range`. Each carries the
 * annotation's own SourceLoc, so render_diagnostic() draws the same caret it
 * draws for a syntax error.
 *
 * A file whose annotations are all fine produces an empty diagnostic list, and
 * the attribute list is produced either way: one unusable slider must not cost
 * the Inspector the other nine.
 */
struct AttributeSchema {
    std::vector<AttrInfo> attributes;
    std::vector<Diagnostic> diagnostics;

    /// True when no diagnostic has Severity::Error
    [[nodiscard]] bool ok() const;

    /// @return The entry for @p name, or nullptr
    [[nodiscard]] const AttrInfo* find(std::string_view name) const;
};

[[nodiscard]] AttributeSchema attribute_schema(const RuleFile& file);

// ============================================================================
// Registration
// ============================================================================

/**
 * @brief Add D6's statement-position operations to @p table
 *
 * `set`, `tag` and `delete_tags`, all three of them rows that already exist in
 * ast.hpp's kBuiltinOperations and that D2 deliberately left unimplemented.
 *
 * Additive, and separate from control_operations(), because three operation
 * families land in this tree at once and each has to be able to register into
 * one table without knowing what the other two put there. A later registration
 * of the same name wins, which is OperationTable's documented behaviour.
 */
void register_control_operations(OperationTable& table);

/**
 * @brief Add D6's expression-position functions to @p table
 *
 * `attrs.get`, `attrs.source`, `attrs.has`, `attrs.declared`, `tags.has`,
 * `tags.count`, `tags.list`, and `random.unit`, `random.range`, `random.integer`,
 * `random.chance`, `random.pick`, `random.weighted`.
 *
 * The namespaces are `attrs` and not `attr`, and `random` and not `rand` only
 * because `attr` is a KEYWORD: the lexer turns it into TokenKind::KwAttr and
 * `attr.get(...)` would not parse. `scope` is a keyword too, which is why there
 * is no `scope.` namespace here.
 */
void register_control_functions(FunctionTable& table);

/// standard_operations() plus register_control_operations(), built once
[[nodiscard]] const OperationTable& control_operations();

/// standard_functions() plus register_control_functions(), built once
[[nodiscard]] const FunctionTable& control_functions();

} // namespace stratum::procgen::rules
