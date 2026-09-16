// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file op_control.cpp
 * @brief D6: the attribute, tag and stochastic handlers, and the annotation decoder
 *
 * The reasoning is in op_control.hpp. What is here is the machinery, plus local
 * notes on the four places it is not obvious: the argument helpers every handler
 * shares, the one line every draw goes through, the rung order in
 * attribute_source(), and why the annotation decoder refuses rather than guesses.
 */

#include "procgen/rules/op_control.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace stratum::procgen::rules {

namespace {

// ============================================================================
// Shared checks
// ============================================================================

/**
 * @brief Does a value fit a declared type
 *
 * The same test interpreter.cpp makes before accepting a supplied attribute
 * override. It is duplicated here rather than shared because the interpreter's
 * copy is in an anonymous namespace, and the copy is load-bearing:
 * attribute_source() must agree with the interpreter about when an override is
 * IN EFFECT, or it would confirm a slider that does nothing. That agreement is
 * pinned by OpControl.a_mistyped_override_is_not_reported_as_supplied.
 */
[[nodiscard]] bool type_matches(const TypeRef& type, const Value& value) {
    if (type.is_array) {
        return value.is_array();
    }
    switch (type.type) {
        case PrimitiveType::Float: return value.is_number();
        case PrimitiveType::Bool: return value.is_bool();
        case PrimitiveType::String: return value.is_text();
    }
    return false;
}

/// "float", "bool[]" and so on, for a message
[[nodiscard]] std::string type_text(const TypeRef& type) {
    std::string out = primitive_type_name(type.type);
    if (type.is_array) {
        out += "[]";
    }
    return out;
}

/**
 * @brief Are two scalar values the same value?
 *
 * Value has no operator== and cannot easily get one: it holds a
 * std::vector<Value>, so a defaulted comparison would need Value to be
 * comparable before it is complete. Arrays compare false rather than element by
 * element, because the one caller -- checking a default against its own @enum --
 * has already excluded them.
 */
[[nodiscard]] bool values_equal(const Value& a, const Value& b) {
    if (a.data.index() != b.data.index()) {
        return false;
    }
    if (a.is_number()) {
        return a.as_number() == b.as_number();
    }
    if (a.is_bool()) {
        return a.as_bool() == b.as_bool();
    }
    if (a.is_text()) {
        return a.as_text() == b.as_text();
    }
    return false;
}

[[nodiscard]] const AttrDecl* find_attr_decl(const RuleFile& file, std::string_view name) {
    for (const AttrDecl& decl : file.attributes) {
        if (decl.name == name) {
            return &decl;
        }
    }
    return nullptr;
}

/**
 * @brief Is this one of the reserved names a rule may not write or read?
 *
 * '#' cannot begin an identifier, so nothing the lexer produces can collide with
 * kTagsAttribute -- but `set()` takes its name as a STRING, and a string can
 * spell anything. Refusing the prefix is what keeps the reserved space reserved
 * rather than merely unlikely, and it costs an author nothing: no attribute they
 * would want to name starts with '#'.
 */
[[nodiscard]] bool is_reserved_name(std::string_view name) {
    return !name.empty() && name.front() == '#';
}

// ============================================================================
// Argument helpers
// ============================================================================
//
// Every one reports through fail_shape() and returns false, and every caller
// returns immediately on false. fail_shape() does not throw -- the unwinding
// type is private to interpreter.cpp -- so a handler that carried on after one
// of these would run with the argument it did not get.

[[nodiscard]] bool op_text(OperationArgs& context, size_t index, std::string& out) {
    if (index >= context.args.size() || !context.args[index].is_text()) {
        context.interpreter.fail_shape(
            context.loc, "'" + std::string{context.name} + "' wants a string for argument " +
                             std::to_string(index + 1));
        return false;
    }
    out = context.args[index].as_text();
    return true;
}

[[nodiscard]] bool fn_text(const FunctionArgs& context, size_t index, std::string& out) {
    if (index >= context.args.size() || !context.args[index].is_text()) {
        context.interpreter.fail_shape(
            context.loc, "'" + std::string{context.name} + "' wants a string for argument " +
                             std::to_string(index + 1));
        return false;
    }
    out = context.args[index].as_text();
    return true;
}

[[nodiscard]] bool fn_number(const FunctionArgs& context, size_t index, double& out) {
    if (index >= context.args.size() || !context.args[index].is_number()) {
        context.interpreter.fail_shape(
            context.loc, "'" + std::string{context.name} + "' wants a number for argument " +
                             std::to_string(index + 1));
        return false;
    }
    out = context.args[index].as_number();
    return true;
}

[[nodiscard]] bool fn_array(const FunctionArgs& context, size_t index, ValueArray& out) {
    if (index >= context.args.size() || !context.args[index].is_array()) {
        context.interpreter.fail_shape(
            context.loc, "'" + std::string{context.name} + "' wants an array for argument " +
                             std::to_string(index + 1));
        return false;
    }
    out = context.args[index].as_array();
    return true;
}

/// The arity the FunctionTable cannot check: ast.hpp resolves operation arity at
/// parse time, but an expression callee is looked up here and nowhere else.
[[nodiscard]] bool fn_arity(const FunctionArgs& context, size_t min_args, size_t max_args) {
    if (context.args.size() >= min_args && context.args.size() <= max_args) {
        return true;
    }
    std::string wanted = min_args == max_args
                             ? std::to_string(min_args)
                             : std::to_string(min_args) + " to " + std::to_string(max_args);
    context.interpreter.fail_shape(context.loc, "'" + std::string{context.name} + "' takes " +
                                                    wanted + " arguments but was given " +
                                                    std::to_string(context.args.size()));
    return false;
}

/**
 * @brief Refuse a shape-reading function while a file-level value is resolving
 *
 * interpreter.hpp: while an attribute default or a constant is being evaluated
 * the FunctionArgs::shape is a default-constructed Shape and not the shape of
 * anything. For a draw that matters twice over -- every declaration would draw
 * against seed_key 0 and `attr h : float = random.range(3, 6)` would be a
 * constant that merely looks random, which is the worst possible failure for
 * this feature because it is invisible in the output.
 */
[[nodiscard]] bool fn_needs_shape(const FunctionArgs& context) {
    if (context.interpreter.has_current_shape()) {
        return true;
    }
    context.interpreter.fail_shape(
        context.loc, "'" + std::string{context.name} +
                         "' reads the current shape, which an attribute default or a constant "
                         "does not have");
    return false;
}

/**
 * @brief The one line every draw in this file goes through
 *
 * shape.seed_key is the shape's ADDRESS in the tree; loc.offset is the call
 * site. No state, no stream, nothing that depends on how many shapes came
 * before. See the determinism section of op_control.hpp for what follows from
 * that and what does not.
 */
[[nodiscard]] double draw_unit(const FunctionArgs& context) {
    return seed_unit(
        seed_mix2(context.shape.seed_key, seed_mix2(kSaltControlDraw, context.loc.offset)));
}

// ============================================================================
// Attribute operations
// ============================================================================

/**
 * @brief `set("height", 20)` -- write an attribute onto this shape
 *
 * The value lands in Shape::attributes, which interpreter.cpp copies into every
 * child, so a `set` applies to this shape and its whole subtree and to nothing
 * else. That is the containment a rule author expects from a statement written
 * inside one rule, and it is what makes `set` usable for "the ground floor is
 * glass" without a way to un-set it further down.
 *
 * A `scope { }` block does NOT restore it. The block saves the axes and the
 * pivot because those are frame state; an attribute is a decision, and undoing
 * decisions on a brace would make `scope { set("style", "brick"); Wall(); }`
 * mean something no reader would predict.
 */
void op_set(OperationArgs& context) {
    std::string name;
    if (!op_text(context, 0, name)) {
        return;
    }
    if (name.empty()) {
        context.interpreter.fail_shape(context.loc, "'set' needs a name to set");
        return;
    }
    if (is_reserved_name(name)) {
        context.interpreter.fail_shape(
            context.loc, "'" + name + "' starts with '#', which is reserved for the tags this "
                                      "shape carries; use tag() instead");
        return;
    }
    if (context.args.size() < 2) {
        context.interpreter.fail_shape(context.loc, "'set' needs a value to set");
        return;
    }

    // A declared attribute keeps its declared type. Without this, `attr height :
    // float` followed by `set("height", "tall")` reads back as a string three
    // rules later, and the message points at the read rather than at the write.
    const AttrDecl* decl = find_attr_decl(context.interpreter.file(), name);
    if (decl != nullptr && !type_matches(decl->type, context.args[1])) {
        context.interpreter.fail_shape(context.loc,
                                       "'" + name + "' is declared " + type_text(decl->type) +
                                           " but was set to a " + context.args[1].type_name());
        return;
    }

    context.shape.attributes[name] = context.args[1];
}

void op_tag(OperationArgs& context) {
    std::string name;
    if (!op_text(context, 0, name)) {
        return;
    }
    if (name.empty()) {
        context.interpreter.fail_shape(context.loc, "'tag' needs a tag to add");
        return;
    }
    // A repeat is not an error: a rule that tags "corner" on two paths has said
    // the same true thing twice, and add_shape_tag() keeps the set a set.
    (void)add_shape_tag(context.shape, name);
}

void op_delete_tags(OperationArgs& context) {
    clear_shape_tags(context.shape);
}

// ============================================================================
// Attribute functions
// ============================================================================

/// The shape rung, then the supplied rung. Shared by both forms of attrs.get().
[[nodiscard]] bool read_above_default(const FunctionArgs& context,
                                      const std::string& name,
                                      Value& out) {
    const auto on_shape = context.shape.attributes.find(name);
    if (on_shape != context.shape.attributes.end()) {
        out = on_shape->second;
        return true;
    }
    const RuleFile& file = context.interpreter.file();
    const AttrDecl* decl = find_attr_decl(file, name);
    if (decl == nullptr) {
        return false;
    }
    const auto& supplied = context.interpreter.options().attributes;
    const auto found = supplied.find(name);
    if (found != supplied.end() && type_matches(decl->type, found->second)) {
        out = found->second;
        return true;
    }
    return false;
}

/**
 * @brief `attrs.get("height")` and `attrs.get("height", height)`
 *
 * The one-argument form answers from the shape, from the supplied overrides, or
 * from a LITERAL default. It refuses a computed default rather than evaluating
 * the expression itself: a second evaluator is a second set of semantics, and
 * the day the two disagree the building is wrong with nothing pointing at the
 * disagreement. The message names the two ways to get the value instead.
 *
 * The two-argument form takes the fallback as an already-evaluated Value, which
 * is how an author hands the interpreter's own resolution back:
 * `attrs.get("height", height)` evaluates `height` through lookup_name() in the
 * ordinary way and this function only decides whether the shape overrides it.
 */
Value fn_attrs_get(const FunctionArgs& context) {
    if (!fn_arity(context, 1, 2) || !fn_needs_shape(context)) {
        return Value{};
    }
    std::string name;
    if (!fn_text(context, 0, name)) {
        return Value{};
    }
    if (is_reserved_name(name)) {
        context.interpreter.fail_shape(context.loc,
                                       "'" + name + "' is reserved for the shape's tags; ask "
                                                    "tags.has() instead");
        return Value{};
    }

    Value value;
    if (read_above_default(context, name, value)) {
        return value;
    }
    if (context.args.size() == 2) {
        return context.args[1];
    }

    const RuleFile& file = context.interpreter.file();
    const AttrDecl* decl = find_attr_decl(file, name);
    if (decl == nullptr) {
        context.interpreter.fail_shape(
            context.loc, "nothing declares or sets '" + name +
                             "'; declare it with 'attr " + name +
                             " : float = ...' or pass a fallback to attrs.get()");
        return Value{};
    }
    if (literal_value(file, decl->default_value, value)) {
        return value;
    }
    context.interpreter.fail_shape(
        context.loc, "the default of '" + name +
                         "' is computed rather than written out, so attrs.get() cannot read it; "
                         "read it by its name as '" +
                         name + "', or pass it as a fallback: attrs.get(\"" + name + "\", " +
                         name + ")");
    return Value{};
}

/// `attrs.source("height")` -- "shape", "supplied", "default" or "none"
Value fn_attrs_source(const FunctionArgs& context) {
    if (!fn_arity(context, 1, 1) || !fn_needs_shape(context)) {
        return Value{};
    }
    std::string name;
    if (!fn_text(context, 0, name)) {
        return Value{};
    }
    const AttrSource source = attribute_source(context.interpreter.file(),
                                               context.interpreter.options(), context.shape, name);
    return Value::text(attr_source_name(source));
}

/// `attrs.has("height")` -- would a read find anything at all?
Value fn_attrs_has(const FunctionArgs& context) {
    if (!fn_arity(context, 1, 1) || !fn_needs_shape(context)) {
        return Value{};
    }
    std::string name;
    if (!fn_text(context, 0, name)) {
        return Value{};
    }
    const AttrSource source = attribute_source(context.interpreter.file(),
                                               context.interpreter.options(), context.shape, name);
    return Value::boolean(source != AttrSource::None);
}

/**
 * @brief `attrs.declared("height")` -- is there an `attr` declaration for it?
 *
 * Reads the FILE and not the shape, so unlike everything else here it is legal
 * while an attribute default is resolving: which names the file declares does
 * not depend on which shape is asking.
 */
Value fn_attrs_declared(const FunctionArgs& context) {
    if (!fn_arity(context, 1, 1)) {
        return Value{};
    }
    std::string name;
    if (!fn_text(context, 0, name)) {
        return Value{};
    }
    return Value::boolean(find_attr_decl(context.interpreter.file(), name) != nullptr);
}

// ============================================================================
// Tag functions
// ============================================================================

Value fn_tags_has(const FunctionArgs& context) {
    if (!fn_arity(context, 1, 1) || !fn_needs_shape(context)) {
        return Value{};
    }
    std::string name;
    if (!fn_text(context, 0, name)) {
        return Value{};
    }
    return Value::boolean(shape_has_tag(context.shape, name));
}

Value fn_tags_count(const FunctionArgs& context) {
    if (!fn_arity(context, 0, 0) || !fn_needs_shape(context)) {
        return Value{};
    }
    return Value::number(static_cast<double>(shape_tags(context.shape).size()));
}

Value fn_tags_list(const FunctionArgs& context) {
    if (!fn_arity(context, 0, 0) || !fn_needs_shape(context)) {
        return Value{};
    }
    ValueArray items;
    for (const std::string& tag : shape_tags(context.shape)) {
        items.push_back(Value::text(tag));
    }
    return Value::array(std::move(items));
}

// ============================================================================
// Stochastic functions
// ============================================================================

Value fn_random_unit(const FunctionArgs& context) {
    if (!fn_arity(context, 0, 0) || !fn_needs_shape(context)) {
        return Value{};
    }
    return Value::number(draw_unit(context));
}

/// `random.range(2.8, 3.4)` -- a number in [low, high)
Value fn_random_range(const FunctionArgs& context) {
    if (!fn_arity(context, 2, 2) || !fn_needs_shape(context)) {
        return Value{};
    }
    double low = 0.0;
    double high = 0.0;
    if (!fn_number(context, 0, low) || !fn_number(context, 1, high)) {
        return Value{};
    }
    if (high < low) {
        // Not silently swapped. `random.range(high, low)` in a rule file is a
        // transposition, and swapping it hides the mistake behind numbers that
        // look plausible.
        context.interpreter.fail_shape(context.loc,
                                       "random.range wants its low bound first, and " +
                                           format_number(low) + " is above " +
                                           format_number(high));
        return Value{};
    }
    // low == high is not an error: a range collapsed by an attribute set to its
    // own bound is an ordinary thing for a rule to compute, and it returns low.
    return Value::number(low + draw_unit(context) * (high - low));
}

/// `random.integer(1, 4)` -- a whole number in [low, high], both ends included
Value fn_random_integer(const FunctionArgs& context) {
    if (!fn_arity(context, 2, 2) || !fn_needs_shape(context)) {
        return Value{};
    }
    double low = 0.0;
    double high = 0.0;
    if (!fn_number(context, 0, low) || !fn_number(context, 1, high)) {
        return Value{};
    }
    // nearbyint, the same tie-break the `round` function uses, so a bound
    // computed as 2.5 lands on the same integer on every platform.
    low = std::nearbyint(low);
    high = std::nearbyint(high);
    if (high < low) {
        context.interpreter.fail_shape(context.loc,
                                       "random.integer wants its low bound first, and " +
                                           format_number(low) + " is above " +
                                           format_number(high));
        return Value{};
    }
    const double span = high - low + 1.0;
    double value = low + std::floor(draw_unit(context) * span);
    if (value > high) {
        // Only reachable when span is enormous and the draw is the largest
        // double below 1: the product can round up to span. Clamping is the same
        // guard exec_choose() puts on its last arm, and for the same reason --
        // a value outside the range once in a few million is a bug that cannot
        // be reproduced.
        value = high;
    }
    return Value::number(value);
}

/// `random.chance(0.25)` -- true a quarter of the time
Value fn_random_chance(const FunctionArgs& context) {
    if (!fn_arity(context, 1, 1) || !fn_needs_shape(context)) {
        return Value{};
    }
    double probability = 0.0;
    if (!fn_number(context, 0, probability)) {
        return Value{};
    }
    if (!(probability >= 0.0) || !(probability <= 1.0)) {
        // Not clamped. `random.chance(50)` is someone writing a percentage, and
        // clamping turns it into an always-true that looks like a working rule.
        context.interpreter.fail_shape(
            context.loc, "a chance is a probability between 0 and 1, and " +
                             format_number(probability) +
                             " is not one; write 0.5 rather than 50 for half the time");
        return Value{};
    }
    // draw_unit() is in [0, 1): at probability 0 nothing passes, at 1 everything
    // does, and neither end is a special case.
    return Value::boolean(draw_unit(context) < probability);
}

/// `random.pick(["brick", "render", "stone"])` -- one element, uniformly
Value fn_random_pick(const FunctionArgs& context) {
    if (!fn_arity(context, 1, 1) || !fn_needs_shape(context)) {
        return Value{};
    }
    ValueArray items;
    if (!fn_array(context, 0, items)) {
        return Value{};
    }
    if (items.empty()) {
        context.interpreter.fail_shape(context.loc, "random.pick was given an empty array");
        return Value{};
    }
    const double count = static_cast<double>(items.size());
    double index = std::floor(draw_unit(context) * count);
    if (index > count - 1.0) {
        index = count - 1.0;
    }
    return items[static_cast<size_t>(index)];
}

/**
 * @brief `random.weighted(["brick", "glass"], [3, 1])` -- the expression `choose`
 *
 * Weights are relative and normalised by their sum, which is the argument
 * ast.hpp makes for ChooseStmt: any non-negative numbers work, adding an option
 * does not require editing the others, and there is no error class for "the
 * percentages are wrong". The three corners the brief names are each their own
 * answer: a zero weight is never picked, a negative weight is refused, and an
 * all-zero list is refused rather than divided by.
 */
Value fn_random_weighted(const FunctionArgs& context) {
    if (!fn_arity(context, 2, 2) || !fn_needs_shape(context)) {
        return Value{};
    }
    ValueArray items;
    ValueArray weights;
    if (!fn_array(context, 0, items) || !fn_array(context, 1, weights)) {
        return Value{};
    }
    if (items.empty()) {
        context.interpreter.fail_shape(context.loc, "random.weighted was given an empty array");
        return Value{};
    }
    if (items.size() != weights.size()) {
        context.interpreter.fail_shape(
            context.loc, "random.weighted was given " + std::to_string(items.size()) +
                             " values and " + std::to_string(weights.size()) +
                             " weights; there must be one weight for each value");
        return Value{};
    }

    std::vector<double> numbers;
    numbers.reserve(weights.size());
    for (size_t i = 0; i < weights.size(); ++i) {
        if (!weights[i].is_number()) {
            context.interpreter.fail_shape(context.loc,
                                           "weight " + std::to_string(i + 1) +
                                               " must be a number, not a " +
                                               weights[i].type_name());
            return Value{};
        }
        const double weight = weights[i].as_number();
        // !(w >= 0) and not (w < 0), so a NaN weight is refused too rather than
        // slipping through every comparison and leaving the total NaN.
        if (!(weight >= 0.0)) {
            context.interpreter.fail_shape(context.loc,
                                           "a weight cannot be negative, and weight " +
                                               std::to_string(i + 1) + " is " +
                                               format_number(weight));
            return Value{};
        }
        numbers.push_back(weight);
    }

    const size_t chosen = weighted_index(numbers, draw_unit(context));
    if (chosen >= items.size()) {
        context.interpreter.fail_shape(
            context.loc, "every weight given to random.weighted is zero, so there is nothing "
                         "to pick");
        return Value{};
    }
    return items[chosen];
}

// ============================================================================
// Table construction
// ============================================================================

[[nodiscard]] OperationTable build_control_operations() {
    OperationTable table = standard_operations();
    register_control_operations(table);
    return table;
}

[[nodiscard]] FunctionTable build_control_functions() {
    FunctionTable table = standard_functions();
    register_control_functions(table);
    return table;
}

// ============================================================================
// Annotation decoding
// ============================================================================

/// One decoded annotation's complaint, at the annotation's own location
void complain(std::vector<Diagnostic>& out, const SourceLoc& loc, std::string message) {
    Diagnostic diagnostic;
    diagnostic.severity = Severity::Error;
    diagnostic.loc = loc;
    // As interpreter.cpp does for a runtime fault: the AST records a location
    // but not a token length, so the caret underlines one byte.
    diagnostic.length = 1;
    diagnostic.message = std::move(message);
    out.push_back(std::move(diagnostic));
}

/// @return false and a complaint when the annotation appears twice
[[nodiscard]] bool claim_once(bool& already,
                              const Annotation& annotation,
                              std::vector<Diagnostic>& out) {
    if (already) {
        // The first wins. Merging two @range annotations has no defined answer,
        // and taking the last would make the Inspector depend on an order the
        // author cannot see in a diff.
        complain(out, annotation.loc,
                 "'@" + annotation.name + "' is given twice on this attribute; the first was kept");
        return false;
    }
    already = true;
    return true;
}

/// A literal number argument of an annotation
[[nodiscard]] bool annotation_number(const RuleFile& file,
                                     const Annotation& annotation,
                                     size_t index,
                                     std::vector<Diagnostic>& out,
                                     double& value) {
    Value literal;
    if (index >= annotation.args.size() || !literal_value(file, annotation.args[index], literal) ||
        !literal.is_number()) {
        complain(out, annotation.loc, "'@" + annotation.name + "' wants a plain number for "
                                                               "argument " +
                                          std::to_string(index + 1));
        return false;
    }
    value = literal.as_number();
    return true;
}

/// A literal string argument of an annotation
[[nodiscard]] bool annotation_text(const RuleFile& file,
                                   const Annotation& annotation,
                                   size_t index,
                                   std::vector<Diagnostic>& out,
                                   std::string& value) {
    Value literal;
    if (index >= annotation.args.size() || !literal_value(file, annotation.args[index], literal) ||
        !literal.is_text()) {
        complain(out, annotation.loc, "'@" + annotation.name + "' wants a quoted string for "
                                                               "argument " +
                                          std::to_string(index + 1));
        return false;
    }
    value = literal.as_text();
    return true;
}

void decode_annotations(const RuleFile& file,
                        const AttrDecl& decl,
                        AttrInfo& info,
                        std::vector<Diagnostic>& out) {
    bool seen_range = false;
    bool seen_step = false;
    bool seen_order = false;
    bool seen_description = false;
    bool seen_unit = false;
    bool seen_group = false;
    bool seen_enum = false;
    bool seen_asset = false;

    for (const Annotation& annotation : decl.annotations) {
        switch (annotation.kind) {
            case AnnotationKind::Range: {
                if (!claim_once(seen_range, annotation, out)) {
                    break;
                }
                if (decl.type.type != PrimitiveType::Float) {
                    complain(out, annotation.loc,
                             "'@range' draws a slider, so it is only meaningful on a float "
                             "attribute, and '" +
                                 decl.name + "' is " + type_text(decl.type));
                    break;
                }
                double low = 0.0;
                double high = 0.0;
                if (!annotation_number(file, annotation, 0, out, low) ||
                    !annotation_number(file, annotation, 1, out, high)) {
                    break;
                }
                if (!(low < high)) {
                    // A slider whose ends are equal or crossed has no positions
                    // to take. Accepting it would produce a control the author
                    // cannot move and no message saying why.
                    complain(out, annotation.loc,
                             "'@range' wants a minimum below its maximum, and " +
                                 format_number(low) + " is not below " + format_number(high));
                    break;
                }
                info.has_range = true;
                info.range_min = low;
                info.range_max = high;
                break;
            }
            case AnnotationKind::Step: {
                if (!claim_once(seen_step, annotation, out)) {
                    break;
                }
                double step = 0.0;
                if (!annotation_number(file, annotation, 0, out, step)) {
                    break;
                }
                if (!(step > 0.0)) {
                    complain(out, annotation.loc,
                             "'@step' wants a size above zero, and " + format_number(step) +
                                 " would let the slider take no positions");
                    break;
                }
                info.has_step = true;
                info.step = step;
                break;
            }
            case AnnotationKind::Order: {
                if (!claim_once(seen_order, annotation, out)) {
                    break;
                }
                double order = 0.0;
                if (!annotation_number(file, annotation, 0, out, order)) {
                    break;
                }
                info.has_order = true;
                info.order = order;
                break;
            }
            case AnnotationKind::Description: {
                if (!claim_once(seen_description, annotation, out)) {
                    break;
                }
                (void)annotation_text(file, annotation, 0, out, info.description);
                break;
            }
            case AnnotationKind::Unit: {
                if (!claim_once(seen_unit, annotation, out)) {
                    break;
                }
                (void)annotation_text(file, annotation, 0, out, info.unit);
                break;
            }
            case AnnotationKind::Group: {
                if (!claim_once(seen_group, annotation, out)) {
                    break;
                }
                std::vector<std::string> path;
                bool good = true;
                for (size_t i = 0; i < annotation.args.size(); ++i) {
                    std::string level;
                    if (!annotation_text(file, annotation, i, out, level)) {
                        good = false;
                        break;
                    }
                    path.push_back(std::move(level));
                }
                if (good && !path.empty()) {
                    info.group = std::move(path);
                }
                break;
            }
            case AnnotationKind::Enum: {
                if (!claim_once(seen_enum, annotation, out)) {
                    break;
                }
                std::vector<Value> values;
                bool good = true;
                for (const ExprId arg : annotation.args) {
                    Value literal;
                    if (!literal_value(file, arg, literal)) {
                        complain(out, annotation.loc,
                                 "'@enum' wants plain values it can list, and one of these is "
                                 "computed");
                        good = false;
                        break;
                    }
                    // Checked against the ELEMENT type: an @enum on a string[]
                    // restricts what each element may be, not what the array is.
                    TypeRef element = decl.type;
                    element.is_array = false;
                    if (!type_matches(element, literal)) {
                        complain(out, annotation.loc,
                                 "'@enum' on '" + decl.name + "' wants " +
                                     primitive_type_name(decl.type.type) + " values, and " +
                                     literal.to_text() + " is a " + literal.type_name());
                        good = false;
                        break;
                    }
                    values.push_back(std::move(literal));
                }
                if (good) {
                    info.enum_values = std::move(values);
                }
                break;
            }
            case AnnotationKind::Asset: {
                if (!claim_once(seen_asset, annotation, out)) {
                    break;
                }
                if (decl.type.type != PrimitiveType::String) {
                    complain(out, annotation.loc,
                             "'@asset' names a file, so it is only meaningful on a string "
                             "attribute, and '" +
                                 decl.name + "' is " + type_text(decl.type));
                    break;
                }
                info.is_asset = true;
                if (!annotation.args.empty()) {
                    (void)annotation_text(file, annotation, 0, out, info.asset_filter);
                }
                break;
            }
            case AnnotationKind::Color:
                info.is_color = true;
                break;
            case AnnotationKind::Hidden:
                info.hidden = true;
                break;
            case AnnotationKind::Start:
                // The parser already refuses @start on an attribute and reported
                // it at its own line. Saying so again here would be a second
                // diagnostic for one mistake.
                break;
        }
    }
}

} // namespace

// ============================================================================
// Provenance
// ============================================================================

const char* attr_source_name(AttrSource source) {
    switch (source) {
        case AttrSource::None: return "none";
        case AttrSource::Default: return "default";
        case AttrSource::Supplied: return "supplied";
        case AttrSource::Shape: return "shape";
    }
    return "none";
}

AttrSource attribute_source(const RuleFile& file,
                            const GenerationOptions& options,
                            const Shape& shape,
                            std::string_view name) {
    // Highest rung first, so the answer is the one a read would actually get.
    // The order is the whole content of this function; see op_control.hpp for
    // why Shape outranks Supplied.
    const std::string key{name};
    if (shape.attributes.find(key) != shape.attributes.end()) {
        return AttrSource::Shape;
    }
    const AttrDecl* decl = find_attr_decl(file, name);
    if (decl == nullptr) {
        // An undeclared name with a supplied value is NOT Supplied: the
        // interpreter reports "there is no attribute called '...' to set" and
        // uses nothing. Reporting Supplied here would tell an author their
        // override is live when it is not.
        return AttrSource::None;
    }
    const auto supplied = options.attributes.find(key);
    if (supplied != options.attributes.end() && type_matches(decl->type, supplied->second)) {
        return AttrSource::Supplied;
    }
    return AttrSource::Default;
}

// ============================================================================
// Weighted choice
// ============================================================================

size_t weighted_index(const std::vector<double>& weights, double unit) {
    // A weight that is not positive is left out of the total AND out of the
    // scan. Adding it and relying on `target < accumulated` to reject it works
    // for every draw except exactly zero, where a zero-width interval is still
    // landed on -- once in 2^53 draws, in a system whose whole promise is that a
    // result reproduces. Skipping is the same rule with no boundary to get wrong.
    double total = 0.0;
    size_t last_positive = weights.size();
    for (size_t i = 0; i < weights.size(); ++i) {
        if (weights[i] > 0.0) {
            total += weights[i];
            last_positive = i;
        }
    }
    if (!(total > 0.0)) {
        return weights.size();
    }

    const double target = unit * total;
    double accumulated = 0.0;
    for (size_t i = 0; i < weights.size(); ++i) {
        if (!(weights[i] > 0.0)) {
            continue;
        }
        accumulated += weights[i];
        if (target < accumulated) {
            return i;
        }
    }
    // Reached only for a `unit` at or above 1, which seed_unit() never produces
    // and which a caller could still pass. The answer is the last entry that
    // could have been chosen at ALL -- not the last entry, which may be a
    // zero-weight one that this function has just promised never to return.
    return last_positive;
}

// ============================================================================
// Tags
// ============================================================================

std::vector<std::string> shape_tags(const Shape& shape) {
    std::vector<std::string> tags;
    const auto found = shape.attributes.find(std::string{kTagsAttribute});
    if (found == shape.attributes.end() || !found->second.is_array()) {
        return tags;
    }
    for (const Value& entry : found->second.as_array()) {
        if (entry.is_text()) {
            tags.push_back(entry.as_text());
        }
    }
    return tags;
}

bool shape_has_tag(const Shape& shape, std::string_view tag) {
    for (const std::string& existing : shape_tags(shape)) {
        if (existing == tag) {
            return true;
        }
    }
    return false;
}

bool add_shape_tag(Shape& shape, std::string_view tag) {
    std::vector<std::string> tags = shape_tags(shape);
    const std::string wanted{tag};
    const auto at = std::lower_bound(tags.begin(), tags.end(), wanted);
    if (at != tags.end() && *at == wanted) {
        return false;
    }
    tags.insert(at, wanted);

    ValueArray stored;
    stored.reserve(tags.size());
    for (const std::string& entry : tags) {
        stored.push_back(Value::text(entry));
    }
    shape.attributes[std::string{kTagsAttribute}] = Value::array(std::move(stored));
    return true;
}

void clear_shape_tags(Shape& shape) {
    // Erased rather than set to an empty array, so a shape that was never tagged
    // and a shape whose tags were removed dump identically. Two shapes that are
    // the same shape must not compare different.
    shape.attributes.erase(std::string{kTagsAttribute});
}

// ============================================================================
// Literals and the schema
// ============================================================================

bool literal_value(const RuleFile& file, ExprId id, Value& out) {
    if (id == kNoNode || id >= file.exprs.size()) {
        return false;
    }
    const Expr& expr = file.expr(id);
    switch (expr_kind(expr)) {
        case ExprKind::Number:
            out = Value::number(std::get<NumberExpr>(expr.node).value);
            return true;
        case ExprKind::String:
            out = Value::text(std::get<StringExpr>(expr.node).value);
            return true;
        case ExprKind::Bool:
            out = Value::boolean(std::get<BoolExpr>(expr.node).value);
            return true;
        case ExprKind::Array: {
            ValueArray items;
            for (const ExprId element : std::get<ArrayExpr>(expr.node).elements) {
                Value value;
                if (!literal_value(file, element, value)) {
                    return false;
                }
                items.push_back(std::move(value));
            }
            out = Value::array(std::move(items));
            return true;
        }
        case ExprKind::Unary: {
            // `-1.0` is a Negate over a Number, not a negative literal: the lexer
            // never produces a signed number. Without this case every attribute
            // with a negative default would look computed.
            const UnaryExpr& node = std::get<UnaryExpr>(expr.node);
            Value inner;
            if (!literal_value(file, node.operand, inner)) {
                return false;
            }
            if (node.op == UnaryOp::Negate) {
                if (!inner.is_number()) {
                    return false;
                }
                out = Value::number(-inner.as_number());
                return true;
            }
            if (!inner.is_bool()) {
                return false;
            }
            out = Value::boolean(!inner.as_bool());
            return true;
        }
        case ExprKind::Name:
        case ExprKind::Index:
        case ExprKind::Binary:
        case ExprKind::Ternary:
        case ExprKind::Call:
            // Computed. Answering these would mean a second evaluator beside the
            // interpreter's, and two evaluators drift.
            return false;
    }
    return false;
}

bool AttributeSchema::ok() const {
    for (const Diagnostic& diagnostic : diagnostics) {
        if (diagnostic.severity == Severity::Error) {
            return false;
        }
    }
    return true;
}

const AttrInfo* AttributeSchema::find(std::string_view name) const {
    for (const AttrInfo& info : attributes) {
        if (info.name == name) {
            return &info;
        }
    }
    return nullptr;
}

AttributeSchema attribute_schema(const RuleFile& file) {
    AttributeSchema schema;
    schema.attributes.reserve(file.attributes.size());

    for (const AttrDecl& decl : file.attributes) {
        AttrInfo info;
        info.name = decl.name;
        info.type = decl.type;
        info.loc = decl.loc;
        info.default_is_literal = literal_value(file, decl.default_value, info.default_value);

        // A literal default of the wrong type is caught here and nowhere else:
        // the parser checks the declaration's shape, and the interpreter stores
        // whatever the default evaluates to without comparing it against the
        // declaration. `attr height : float = "tall"` otherwise surfaces as a
        // type fault at the first rule that does arithmetic on it, pointing at
        // that rule instead of at the declaration.
        if (info.default_is_literal && !type_matches(decl.type, info.default_value)) {
            complain(schema.diagnostics, decl.loc,
                     "'" + decl.name + "' is declared " + type_text(decl.type) +
                         " but its default is a " + info.default_value.type_name());
        }

        decode_annotations(file, decl, info, schema.diagnostics);

        // A default outside the slider it declares is a control that cannot show
        // its own starting value.
        if (info.has_range && info.default_is_literal && info.default_value.is_number()) {
            const double value = info.default_value.as_number();
            if (value < info.range_min || value > info.range_max) {
                complain(schema.diagnostics, decl.loc,
                         "the default of '" + decl.name + "' is " + format_number(value) +
                             ", which is outside its own @range of " +
                             format_number(info.range_min) + " to " +
                             format_number(info.range_max));
            }
        }

        // A default that is not one of the values the author said were allowed.
        if (!info.enum_values.empty() && info.default_is_literal &&
            !info.default_value.is_array()) {
            bool listed = false;
            for (const Value& allowed : info.enum_values) {
                if (values_equal(allowed, info.default_value)) {
                    listed = true;
                    break;
                }
            }
            if (!listed) {
                complain(schema.diagnostics, decl.loc,
                         "the default of '" + decl.name + "' is " + info.default_value.to_text() +
                             ", which its own @enum does not list");
            }
        }

        schema.attributes.push_back(std::move(info));
    }
    return schema;
}

// ============================================================================
// Registration
// ============================================================================

void register_control_operations(OperationTable& table) {
    table.register_operation("delete_tags", op_delete_tags);
    table.register_operation("set", op_set);
    table.register_operation("tag", op_tag);
}

void register_control_functions(FunctionTable& table) {
    table.register_function("attrs.declared", fn_attrs_declared);
    table.register_function("attrs.get", fn_attrs_get);
    table.register_function("attrs.has", fn_attrs_has);
    table.register_function("attrs.source", fn_attrs_source);
    table.register_function("random.chance", fn_random_chance);
    table.register_function("random.integer", fn_random_integer);
    table.register_function("random.pick", fn_random_pick);
    table.register_function("random.range", fn_random_range);
    table.register_function("random.unit", fn_random_unit);
    table.register_function("random.weighted", fn_random_weighted);
    table.register_function("tags.count", fn_tags_count);
    table.register_function("tags.has", fn_tags_has);
    table.register_function("tags.list", fn_tags_list);
}

const OperationTable& control_operations() {
    static const OperationTable table = build_control_operations();
    return table;
}

const FunctionTable& control_functions() {
    static const FunctionTable table = build_control_functions();
    return table;
}

} // namespace stratum::procgen::rules
