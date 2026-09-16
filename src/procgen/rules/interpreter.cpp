// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file interpreter.cpp
 * @brief The evaluator, the D2 operation handlers and the D2 function library
 *
 * The reasoning is in interpreter.hpp. What is here is the machinery, plus the
 * local notes on the four places it is not obvious: the unwinding type, the
 * lazy symbol resolution that lets a constant and an attribute refer to each
 * other in either order, the per-site salt that keeps two `choose` statements in
 * one rule independent, and the argument coercion every handler shares.
 */

#include "procgen/rules/interpreter.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <utility>

namespace stratum::procgen::rules {

namespace {

/**
 * @brief The private unwinding type
 *
 * Thrown when a shape cannot continue, caught at the rule-invocation boundary so
 * that the shape's SIBLINGS still evaluate. It is declared in an anonymous
 * namespace, so nothing outside this file can catch it, name it or throw it --
 * which is why Interpreter::fail_shape() exists for handlers in other
 * translation units.
 *
 * parser.hpp makes the same argument for the same technique: threading a failure
 * flag back through thirty recursive functions is enormously more code than one
 * unwind, and the flag gets forgotten in exactly one of the thirty.
 */
struct AbandonShape {};

/// Whether the rest of a statement list still runs
enum class Flow : uint8_t { Continue, Discard };

/// Does a value fit a declared type
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

} // namespace

// ============================================================================
// Registries
// ============================================================================

void OperationTable::register_operation(std::string_view name, OperationHandler handler) {
    handlers_[std::string{name}] = std::move(handler);
}

const OperationHandler* OperationTable::find(std::string_view name) const {
    const auto found = handlers_.find(name);
    return found == handlers_.end() ? nullptr : &found->second;
}

void FunctionTable::register_function(std::string_view name, FunctionHandler handler) {
    handlers_[std::string{name}] = std::move(handler);
}

const FunctionHandler* FunctionTable::find(std::string_view name) const {
    const auto found = handlers_.find(name);
    return found == handlers_.end() ? nullptr : &found->second;
}

// ============================================================================
// Interpreter state
// ============================================================================

/**
 * @brief Everything one generation mutates
 *
 * Split out of Interpreter so that the header carries the contract and not the
 * machinery: adding a field here does not recompile every consumer of the rule
 * language.
 */
struct Interpreter::State {
    Interpreter& owner;
    const RuleFile& file;
    const GenerationOptions& options;
    const OperationTable& operations;
    const FunctionTable& functions;

    GenerationResult result;

    /// Lexical bindings: rule parameters at a frame's base, then `let`s.
    /// A vector searched backwards, so an inner `let` shadows an outer one and
    /// leaving a block is a resize rather than a map erase.
    std::vector<std::pair<std::string, Value>> locals;

    /// Attribute and constant values, resolved once each. See resolve_global().
    std::map<std::string, Value> globals;
    std::map<std::string, const AttrDecl*> attr_decls;
    std::map<std::string, const ConstDecl*> const_decls;
    std::set<std::string> resolving;

    /// True while an attribute default or a constant is being evaluated, which is
    /// when `shape.*` has no shape to mean.
    bool in_global = false;

    /// Set by fail_shape(), cleared when the throw happens
    bool abandon_requested = false;

    std::set<std::string> reported_once;
    bool diagnostics_capped = false;
    bool depth_reported = false;
    bool shape_reported = false;

    uint32_t next_id = 0;

    /**
     * @brief How many shapes each parent has had emitted by an operation
     *
     * PER PARENT, keyed by Shape::id, and not one running counter. A running
     * counter would make a setback border's seed depend on how many borders were
     * emitted anywhere before it, which is the shared-stream mistake this whole
     * design exists to avoid: adding a setback to one rule would change the
     * random draws of every shape produced after it.
     */
    std::map<uint32_t, uint32_t> derived_counts;

    State(Interpreter& owner_in,
          const RuleFile& file_in,
          const GenerationOptions& options_in,
          const OperationTable& operations_in,
          const FunctionTable& functions_in)
        : owner(owner_in),
          file(file_in),
          options(options_in),
          operations(operations_in),
          functions(functions_in) {}

    // ---- diagnostics ----

    void report(Severity severity, const SourceLoc& loc, std::string message) {
        if (diagnostics_capped) {
            return;
        }
        if (result.diagnostics.size() + 1 >= kMaxDiagnostics) {
            diagnostics_capped = true;
            Diagnostic diagnostic;
            diagnostic.severity = Severity::Error;
            diagnostic.loc = loc;
            diagnostic.message = kTooManyDiagnostics;
            result.diagnostics.push_back(std::move(diagnostic));
            return;
        }
        Diagnostic diagnostic;
        diagnostic.severity = severity;
        diagnostic.loc = loc;
        // The AST records a location but no token length, so a runtime caret
        // underlines one byte. Reconstructing the length would mean re-lexing the
        // source, which the interpreter does not have.
        diagnostic.length = 1;
        diagnostic.message = std::move(message);
        result.diagnostics.push_back(std::move(diagnostic));
    }

    void report_once(Severity severity, const SourceLoc& loc, std::string message) {
        std::string key = std::to_string(loc.offset) + "|" + message;
        if (!reported_once.insert(key).second) {
            return;
        }
        report(severity, loc, std::move(message));
    }

    [[noreturn]] void fail(const SourceLoc& loc, std::string message) {
        report(Severity::Error, loc, std::move(message));
        throw AbandonShape{};
    }

    /// Throw on behalf of a handler that asked to abandon the shape
    void check_abandon() {
        if (abandon_requested) {
            abandon_requested = false;
            throw AbandonShape{};
        }
    }

    // ---- symbols ----

    /**
     * @brief Resolve an attribute or a constant, once, with cycle detection
     *
     * Lazy rather than eager because a constant may be written in terms of an
     * attribute and an attribute's default in terms of a constant, in either
     * order, and the language says nothing about declaration order ACROSS kinds
     * (ast.hpp says so explicitly). Evaluating one group before the other would
     * make one of the two orders an error for no reason a rule author could see.
     *
     * A cycle IS an error, and a named one, because the alternative is a stack
     * overflow with no line number on it.
     */
    bool resolve_global(const std::string& name, const SourceLoc& use_loc, Value& out) {
        const auto cached = globals.find(name);
        if (cached != globals.end()) {
            out = cached->second;
            return true;
        }

        const auto attr = attr_decls.find(name);
        const auto konst = const_decls.find(name);
        if (attr == attr_decls.end() && konst == const_decls.end()) {
            return false;
        }

        if (!resolving.insert(name).second) {
            fail(use_loc, "'" + name + "' is defined in terms of itself");
        }

        const bool outer_in_global = in_global;
        in_global = true;

        Value value;
        if (attr != attr_decls.end()) {
            const AttrDecl& decl = *attr->second;
            const auto override_it = options.attributes.find(name);
            bool used_override = false;
            if (override_it != options.attributes.end()) {
                if (type_matches(decl.type, override_it->second)) {
                    value = override_it->second;
                    used_override = true;
                } else {
                    report(Severity::Error, decl.loc,
                           "the supplied value for attribute '" + name + "' is a " +
                               override_it->second.type_name() + ", but it is declared " +
                               type_text(decl.type) + "; the default was used instead");
                }
            }
            if (!used_override) {
                // Shape{} rather than the shape being evaluated: a file-level
                // value must not depend on which shape happened to read it first,
                // and eval() refuses `shape.*` while in_global is set anyway.
                const Shape empty;
                value = eval(decl.default_value, empty);
            }
        } else {
            const Shape empty;
            value = eval(konst->second->value, empty);
        }

        in_global = outer_in_global;
        resolving.erase(name);
        globals.emplace(name, value);
        out = std::move(value);
        return true;
    }

    [[nodiscard]] Value lookup_name(const QualifiedName& name,
                                    const SourceLoc& loc,
                                    const Shape& shape) {
        if (name.qualified()) {
            if (name.qualifier != "shape") {
                fail(loc, "unknown namespace '" + name.qualifier + "' in '" + name.text() +
                              "'; the interpreter knows 'shape'");
            }
            if (in_global) {
                fail(loc, "'" + name.text() +
                              "' reads the current shape, which an attribute default or a "
                              "constant does not have");
            }
            if (name.name == "sx") return Value::number(shape.scope.size.x);
            if (name.name == "sy") return Value::number(shape.scope.size.y);
            if (name.name == "sz") return Value::number(shape.scope.size.z);
            if (name.name == "depth") return Value::number(static_cast<double>(shape.depth));
            if (name.name == "index") return Value::number(static_cast<double>(shape.index));
            fail(loc, "'shape' has no member '" + name.name +
                          "'; it has sx, sy, sz, depth and index");
        }

        for (size_t i = locals.size(); i-- > 0;) {
            if (locals[i].first == name.name) {
                return locals[i].second;
            }
        }

        Value global;
        if (resolve_global(name.name, loc, global)) {
            return global;
        }

        if (name.name == "pi") {
            return Value::number(3.14159265358979323846);
        }

        fail(loc, "unknown name '" + name.name + "'");
    }

    // ---- expressions ----

    [[nodiscard]] double want_number(const Value& value, const SourceLoc& loc, const char* what) {
        if (!value.is_number()) {
            fail(loc, std::string{what} + " must be a number, not a " + value.type_name());
        }
        return value.as_number();
    }

    [[nodiscard]] bool want_bool(const Value& value, const SourceLoc& loc, const char* what) {
        if (!value.is_bool()) {
            fail(loc, std::string{what} + " must be a boolean, not a " + value.type_name());
        }
        return value.as_bool();
    }

    [[nodiscard]] Value eval(ExprId id, const Shape& shape) {
        if (id == kNoNode) {
            // Only reachable from an AST that did not come from parse(): the
            // parser never leaves a required expression unset in a file it
            // reports as ok.
            fail(SourceLoc{}, "an expression is missing from the rule file");
        }
        const Expr& expr = file.expr(id);
        switch (expr_kind(expr)) {
            case ExprKind::Number:
                return Value::number(std::get<NumberExpr>(expr.node).value);
            case ExprKind::String:
                return Value::text(std::get<StringExpr>(expr.node).value);
            case ExprKind::Bool:
                return Value::boolean(std::get<BoolExpr>(expr.node).value);
            case ExprKind::Name:
                return lookup_name(std::get<NameExpr>(expr.node).name, expr.loc, shape);
            case ExprKind::Array: {
                ValueArray items;
                for (const ExprId element : std::get<ArrayExpr>(expr.node).elements) {
                    items.push_back(eval(element, shape));
                }
                return Value::array(std::move(items));
            }
            case ExprKind::Index: {
                const IndexExpr& node = std::get<IndexExpr>(expr.node);
                const Value base = eval(node.base, shape);
                const Value index = eval(node.index, shape);
                if (!base.is_array()) {
                    fail(expr.loc, "only an array can be indexed, and this is a " +
                                       std::string{base.type_name()});
                }
                const double raw = want_number(index, expr.loc, "an array index");
                const double rounded = std::nearbyint(raw);
                const ValueArray& items = base.as_array();
                if (!(rounded >= 0.0) || rounded >= static_cast<double>(items.size())) {
                    fail(expr.loc, "index " + format_number(raw) +
                                       " is outside the array of " +
                                       std::to_string(items.size()) + " elements");
                }
                return items[static_cast<size_t>(rounded)];
            }
            case ExprKind::Unary: {
                const UnaryExpr& node = std::get<UnaryExpr>(expr.node);
                const Value operand = eval(node.operand, shape);
                if (node.op == UnaryOp::Negate) {
                    return Value::number(-want_number(operand, expr.loc, "the operand of '-'"));
                }
                return Value::boolean(!want_bool(operand, expr.loc, "the operand of '!'"));
            }
            case ExprKind::Binary:
                return eval_binary(std::get<BinaryExpr>(expr.node), expr.loc, shape);
            case ExprKind::Ternary: {
                const TernaryExpr& node = std::get<TernaryExpr>(expr.node);
                const Value condition = eval(node.condition, shape);
                // Only the branch that is taken is evaluated, so `n > 0 ? 1/n : 0`
                // does not divide by zero on the way to not using the result.
                return want_bool(condition, expr.loc, "the condition of '?'")
                           ? eval(node.then_value, shape)
                           : eval(node.else_value, shape);
            }
            case ExprKind::Call:
                return eval_call(std::get<CallExpr>(expr.node), expr.loc, shape);
        }
        fail(expr.loc, "unsupported expression");
    }

    [[nodiscard]] Value eval_binary(const BinaryExpr& node,
                                    const SourceLoc& loc,
                                    const Shape& shape) {
        // && and || short-circuit, so a guard such as `n > 0 && 10 / n < 2` is
        // safe to write. Evaluating both sides first would make the guard useless.
        if (node.op == BinaryOp::And || node.op == BinaryOp::Or) {
            const bool lhs = want_bool(eval(node.lhs, shape), loc, "the left side of a boolean operator");
            if (node.op == BinaryOp::And && !lhs) {
                return Value::boolean(false);
            }
            if (node.op == BinaryOp::Or && lhs) {
                return Value::boolean(true);
            }
            return Value::boolean(
                want_bool(eval(node.rhs, shape), loc, "the right side of a boolean operator"));
        }

        const Value lhs = eval(node.lhs, shape);
        const Value rhs = eval(node.rhs, shape);

        if (node.op == BinaryOp::Equal || node.op == BinaryOp::NotEqual) {
            bool equal = false;
            if (lhs.is_number() && rhs.is_number()) {
                equal = lhs.as_number() == rhs.as_number();
            } else if (lhs.is_bool() && rhs.is_bool()) {
                equal = lhs.as_bool() == rhs.as_bool();
            } else if (lhs.is_text() && rhs.is_text()) {
                equal = lhs.as_text() == rhs.as_text();
            } else {
                fail(loc, std::string{"a "} + lhs.type_name() + " and a " + rhs.type_name() +
                              " cannot be compared");
            }
            return Value::boolean(node.op == BinaryOp::Equal ? equal : !equal);
        }

        if (node.op == BinaryOp::Add && lhs.is_text() && rhs.is_text()) {
            return Value::text(lhs.as_text() + rhs.as_text());
        }

        const double a = want_number(lhs, loc, "the left side of an arithmetic operator");
        const double b = want_number(rhs, loc, "the right side of an arithmetic operator");
        switch (node.op) {
            case BinaryOp::Add: return Value::number(a + b);
            case BinaryOp::Subtract: return Value::number(a - b);
            case BinaryOp::Multiply: return Value::number(a * b);
            case BinaryOp::Divide:
                if (b == 0.0) {
                    // An infinity here would travel into a coordinate and surface
                    // as a missing mesh with nothing pointing back at the line
                    // that divided.
                    fail(loc, "division by zero");
                }
                return Value::number(a / b);
            case BinaryOp::Less: return Value::boolean(a < b);
            case BinaryOp::LessEqual: return Value::boolean(a <= b);
            case BinaryOp::Greater: return Value::boolean(a > b);
            case BinaryOp::GreaterEqual: return Value::boolean(a >= b);
            default: break;
        }
        fail(loc, "unsupported operator");
    }

    [[nodiscard]] Value eval_call(const CallExpr& node,
                                  const SourceLoc& loc,
                                  const Shape& shape) {
        std::vector<Value> args;
        args.reserve(node.args.size());
        for (const ExprId arg : node.args) {
            args.push_back(eval(arg, shape));
        }

        const std::string name = node.callee.text();
        const FunctionHandler* handler = functions.find(name);
        if (handler == nullptr) {
            fail(loc, "unknown function '" + name + "'");
        }
        const FunctionArgs call{args, loc, name, shape, owner};
        Value value = (*handler)(call);
        check_abandon();
        return value;
    }

    // ---- statements ----

    struct Frame {
        RuleId rule = kNoNode;
        size_t locals_base = 0;
        uint32_t children = 0;
    };

    Flow exec(StmtId id, Shape& shape, Frame& frame) {
        const Stmt& stmt = file.stmt(id);
        switch (stmt_kind(stmt)) {
            case StmtKind::Block: {
                const size_t mark = locals.size();
                Flow flow = Flow::Continue;
                for (const StmtId child : std::get<BlockStmt>(stmt.node).statements) {
                    flow = exec(child, shape, frame);
                    if (flow == Flow::Discard) {
                        break;
                    }
                }
                locals.resize(mark);
                return flow;
            }
            case StmtKind::Call:
                exec_call(std::get<CallStmt>(stmt.node), stmt, shape, frame);
                return Flow::Continue;
            case StmtKind::Let: {
                const LetStmt& node = std::get<LetStmt>(stmt.node);
                Value value = eval(node.value, shape);
                if (node.has_type && !type_matches(node.type, value)) {
                    fail(stmt.loc, "'" + node.name + "' is declared " + type_text(node.type) +
                                       " but was given a " + value.type_name());
                }
                locals.emplace_back(node.name, std::move(value));
                return Flow::Continue;
            }
            case StmtKind::If: {
                const IfStmt& node = std::get<IfStmt>(stmt.node);
                const bool taken =
                    want_bool(eval(node.condition, shape), stmt.loc, "an 'if' condition");
                if (taken) {
                    return exec(node.then_branch, shape, frame);
                }
                if (node.else_branch != kNoNode) {
                    return exec(node.else_branch, shape, frame);
                }
                return Flow::Continue;
            }
            case StmtKind::Choose:
                return exec_choose(std::get<ChooseStmt>(stmt.node), stmt, shape, frame);
            case StmtKind::Split:
                // D3. Reported once per site rather than per shape, and evaluation
                // CONTINUES: the rest of the rule still produces its geometry, so
                // the author sees a building missing its floors rather than
                // nothing at all.
                report_once(Severity::Error, stmt.loc,
                            "'split' is not implemented in this build");
                return Flow::Continue;
            case StmtKind::Select:
                // D4, same treatment as split.
                report_once(Severity::Error, stmt.loc,
                            "'select' is not implemented in this build");
                return Flow::Continue;
            case StmtKind::Scope: {
                const ScopeStmt& node = std::get<ScopeStmt>(stmt.node);
                // The axes and the pivot are saved; the BOX is not, because under
                // shape.hpp's tight-box invariant the box is derived from the
                // geometry and cannot be restored independently of it. What a
                // scope block protects is the orientation, which is what push/pop
                // was ever used for.
                const glm::dmat3 saved_axes = shape.scope.axes;
                const glm::dvec3 saved_pivot = shape.scope.pivot;
                Flow flow = Flow::Continue;
                try {
                    flow = exec(node.body, shape, frame);
                } catch (const AbandonShape&) {
                    shape.scope.pivot = saved_pivot;
                    reframe(shape, saved_axes);
                    throw;
                }
                shape.scope.pivot = saved_pivot;
                reframe(shape, saved_axes);
                return flow;
            }
            case StmtKind::Discard:
                return Flow::Discard;
        }
        return Flow::Continue;
    }

    Flow exec_choose(const ChooseStmt& node,
                     const Stmt& stmt,
                     Shape& shape,
                     Frame& frame) {
        if (node.cases.empty()) {
            return Flow::Continue;
        }

        std::vector<double> weights;
        weights.reserve(node.cases.size());
        double total = 0.0;
        for (const ChooseCase& arm : node.cases) {
            const double weight = want_number(eval(arm.weight, shape), arm.loc, "a choose weight");
            if (!(weight >= 0.0)) {
                fail(arm.loc, "a choose weight cannot be negative, and this one is " +
                                  format_number(weight));
            }
            weights.push_back(weight);
            total += weight;
        }
        if (!(total > 0.0)) {
            fail(stmt.loc, "every arm of this choose has weight zero, so there is nothing to pick");
        }

        // The draw is salted with the STATEMENT's source offset as well as the
        // shape's address. Without that, two `choose` statements in one rule body
        // would both draw shape.draw(kSaltChoose) and always agree -- a window
        // style and a door style that are secretly the same coin.
        const double unit = seed_unit(
            seed_mix2(shape.seed_key, seed_mix2(kSaltChoose, stmt.loc.offset)));
        const double target = unit * total;

        double accumulated = 0.0;
        for (size_t i = 0; i < node.cases.size(); ++i) {
            accumulated += weights[i];
            if (target < accumulated) {
                return exec(node.cases[i].body, shape, frame);
            }
        }
        // Rounding in the accumulation can leave `target` just past the last
        // boundary. The last arm is the answer; falling out with no arm chosen
        // would make a shape silently vanish once in every few million.
        return exec(node.cases.back().body, shape, frame);
    }

    void exec_call(const CallStmt& call, const Stmt& stmt, Shape& shape, Frame& frame) {
        switch (call.target) {
            case CallTarget::Rule:
                exec_rule_call(call, stmt, shape, frame);
                return;
            case CallTarget::Operation:
                exec_operation(call, stmt, shape);
                return;
            case CallTarget::Imported:
                report_once(Severity::Error, stmt.loc,
                            "'" + call.callee.text() +
                                "' comes from an import, and this build does not load imports");
                return;
            case CallTarget::Unresolved:
                report_once(Severity::Error, stmt.loc,
                            "'" + call.callee.text() + "' names no rule and no operation");
                return;
        }
    }

    void exec_operation(const CallStmt& call, const Stmt& stmt, Shape& shape) {
        if (call.operation >= kBuiltinOperationCount) {
            fail(stmt.loc, "'" + call.callee.text() + "' is not a known operation");
        }
        const BuiltinOperation& builtin = kBuiltinOperations[call.operation];

        std::vector<Value> args;
        args.reserve(call.args.size());
        for (const ExprId arg : call.args) {
            args.push_back(eval(arg, shape));
        }

        // The parser already checked the arity, so this only fires for an AST
        // that did not come from parse(). Checking anyway costs two comparisons
        // and turns a handler reading args[1] of a one-argument call from
        // undefined behaviour into a message.
        if (args.size() < builtin.min_args ||
            (builtin.max_args != kVariadic && args.size() > builtin.max_args)) {
            fail(stmt.loc, "'" + std::string{builtin.name} + "' was given " +
                               std::to_string(args.size()) + " arguments");
        }

        const OperationHandler* handler = operations.find(builtin.name);
        if (handler == nullptr) {
            // ast.hpp's catalogue is declared once and filled in over several
            // features. A row with no handler says so at the line that called it,
            // which is the behaviour that header promises.
            report_once(Severity::Error, stmt.loc,
                        "operation '" + std::string{builtin.name} +
                            "' is not implemented in this build");
            return;
        }

        OperationArgs context{shape, args, stmt.loc, builtin.name, owner};
        (*handler)(context);
        check_abandon();
        ++result.stats.operations_applied;
    }

    void exec_rule_call(const CallStmt& call, const Stmt& stmt, Shape& shape, Frame& frame) {
        if (call.rule >= file.rules.size()) {
            fail(stmt.loc, "'" + call.callee.text() + "' does not name a rule in this file");
        }

        // Arguments are evaluated in the CALLER's environment, before the child
        // exists, so `Floor(height - 1)` means the caller's `height`.
        std::vector<Value> args;
        args.reserve(call.args.size());
        for (const ExprId arg : call.args) {
            args.push_back(eval(arg, shape));
        }

        Shape child = shape;
        child.rule = call.callee.name;
        child.loc = stmt.loc;
        child.depth = shape.depth + 1;
        child.index = frame.children;
        child.parent = shape.id;
        // The child's address in the tree: its parent's address mixed with its
        // own sibling index. Never a draw from a shared stream -- see the
        // determinism note in interpreter.hpp.
        child.seed_key =
            seed_mix2(shape.seed_key, seed_mix2(kSaltChild, child.index));
        ++frame.children;

        if (!allocate_shape(child, stmt.loc)) {
            return;
        }

        if (child.depth > options.limits.max_depth) {
            if (!depth_reported) {
                depth_reported = true;
                result.stats.depth_limit_hit = true;
                report(Severity::Error, stmt.loc,
                       "the rule-call depth limit of " +
                           std::to_string(options.limits.max_depth) + " was reached at '" +
                           call.callee.name +
                           "'; the recursion was cut and the shape emitted as it stood");
            }
            // Emitted rather than dropped. Dropping it would leave a runaway
            // tower with no output at all, because every shape above it is
            // non-terminal by virtue of having made a child.
            emit_terminal(std::move(child));
            return;
        }

        invoke_rule(call.rule, std::move(child), args, stmt.loc);
    }

    // ---- rules ----

    bool allocate_shape(Shape& shape, const SourceLoc& loc) {
        if (result.stats.shapes_created >= options.limits.max_shapes) {
            if (!shape_reported) {
                shape_reported = true;
                result.stats.shape_limit_hit = true;
                report(Severity::Error, loc,
                       "the shape limit of " + std::to_string(options.limits.max_shapes) +
                           " was reached; the output is truncated");
            }
            return false;
        }
        shape.id = next_id++;
        ++result.stats.shapes_created;
        return true;
    }

    void emit_terminal(Shape shape) {
        ++result.stats.terminals;
        result.terminals.push_back(std::move(shape));
    }

    void invoke_rule(RuleId rule_id,
                     Shape shape,
                     const std::vector<Value>& args,
                     const SourceLoc& call_loc) {
        const RuleDecl& rule = file.rules[rule_id];
        ++result.stats.rules_invoked;
        result.stats.max_depth_reached = std::max(result.stats.max_depth_reached, shape.depth);

        Frame frame;
        frame.rule = rule_id;
        frame.locals_base = locals.size();

        try {
            if (args.size() > rule.params.size()) {
                fail(call_loc, "'" + rule.name + "' takes " +
                                   std::to_string(rule.params.size()) + " parameters but was given " +
                                   std::to_string(args.size()));
            }

            for (size_t i = 0; i < rule.params.size(); ++i) {
                const Param& param = rule.params[i];
                Value value;
                if (i < args.size()) {
                    value = args[i];
                } else if (param.default_value != kNoNode) {
                    // Defaults are evaluated in the frame being built, so a later
                    // default may refer to an earlier parameter.
                    value = eval(param.default_value, shape);
                } else {
                    fail(call_loc, "'" + rule.name + "' needs an argument for '" + param.name + "'");
                }
                if (!type_matches(param.type, value)) {
                    fail(call_loc, "'" + rule.name + "' wants a " + type_text(param.type) +
                                       " for '" + param.name + "' but was given a " +
                                       value.type_name());
                }
                locals.emplace_back(param.name, std::move(value));
            }

            const Flow flow = rule.body == kNoNode ? Flow::Continue : exec(rule.body, shape, frame);
            locals.resize(frame.locals_base);

            if (flow != Flow::Discard && frame.children == 0) {
                emit_terminal(std::move(shape));
            }
        } catch (const AbandonShape&) {
            // The shape and its subtree produce nothing. Its SIBLINGS still run:
            // one malformed lot must not cost the other four thousand.
            locals.resize(frame.locals_base);
            abandon_requested = false;
        }
    }

    // ---- entry ----

    void run(const Shape& seed) {
        for (const AttrDecl& decl : file.attributes) {
            attr_decls.emplace(decl.name, &decl);
        }
        for (const ConstDecl& decl : file.constants) {
            const_decls.emplace(decl.name, &decl);
        }

        // A name in the overrides that no attribute declares is a typo, and the
        // symptom of ignoring it is a slider in the Inspector that does nothing.
        for (const auto& entry : options.attributes) {
            if (attr_decls.find(entry.first) == attr_decls.end()) {
                report(Severity::Error, SourceLoc{},
                       "there is no attribute called '" + entry.first + "' to set");
            }
        }

        RuleId start = options.start;
        if (start == kNoNode) {
            start = file.start_rule;
        }
        if (start == kNoNode || start >= file.rules.size()) {
            report(Severity::Error, SourceLoc{},
                   "the rule file has no @start rule, and no start rule was chosen");
            return;
        }

        try {
            // Forced in declaration order so the diagnostics of a file with
            // several bad defaults come out in source order rather than in
            // whatever order a rule happened to read them.
            Value ignored;
            for (const AttrDecl& decl : file.attributes) {
                (void)resolve_global(decl.name, decl.loc, ignored);
            }
            for (const ConstDecl& decl : file.constants) {
                (void)resolve_global(decl.name, decl.loc, ignored);
            }
        } catch (const AbandonShape&) {
            // A cycle or a bad default in a file-level value. Nothing can run.
            abandon_requested = false;
            return;
        }

        Shape root = seed;
        root.rule = file.rules[start].name;
        root.loc = file.rules[start].loc;
        root.depth = 0;
        root.index = 0;
        root.parent = kNoShape;
        // The run seed and the input shape's own key together, so a caller that
        // generates one building per lot passes one seed for the city and the
        // lot's key on the shape, and gets lots that differ from each other and
        // repeat exactly on a re-run.
        root.seed_key = seed_mix2(seed_mix2(kSaltRoot, options.seed), seed.seed_key);

        if (!allocate_shape(root, root.loc)) {
            return;
        }
        invoke_rule(start, std::move(root), {}, file.rules[start].loc);
    }
};

// ============================================================================
// Interpreter
// ============================================================================

Interpreter::Interpreter(const RuleFile& file,
                         const GenerationOptions& options,
                         const OperationTable& operations,
                         const FunctionTable& functions)
    : file_(file),
      options_(options),
      operations_(operations),
      functions_(functions),
      state_(new State(*this, file_, options_, operations_, functions_)) {}

Interpreter::~Interpreter() {
    delete state_;
}

GenerationResult Interpreter::run(const Shape& seed) {
    state_->run(seed);
    return std::move(state_->result);
}

void Interpreter::report(Severity severity, const SourceLoc& loc, std::string message) {
    state_->report(severity, loc, std::move(message));
}

void Interpreter::report_once(Severity severity, const SourceLoc& loc, std::string message) {
    state_->report_once(severity, loc, std::move(message));
}

void Interpreter::fail_shape(const SourceLoc& loc, std::string message) {
    state_->report(Severity::Error, loc, std::move(message));
    state_->abandon_requested = true;
}

void Interpreter::log(std::string line) {
    state_->result.log.push_back(std::move(line));
}

bool Interpreter::emit_derived_terminal(const Shape& parent, Shape child, std::string role) {
    child.rule = std::move(role);
    child.loc = parent.loc;
    child.depth = parent.depth;
    child.parent = parent.id;
    child.index = state_->derived_counts[parent.id]++;
    child.seed_key = seed_mix2(parent.seed_key, seed_mix2(kSaltOp, child.index));
    if (!state_->allocate_shape(child, parent.loc)) {
        return false;
    }
    state_->emit_terminal(std::move(child));
    return true;
}

// ============================================================================
// Result
// ============================================================================

bool GenerationResult::ok() const {
    for (const Diagnostic& diagnostic : diagnostics) {
        if (diagnostic.severity == Severity::Error) {
            return false;
        }
    }
    return true;
}

Mesh GenerationResult::build_mesh() const {
    Mesh mesh;
    for (const Shape& shape : terminals) {
        append_shape_to_mesh(shape, mesh);
    }
    mesh.compute_bounds();
    mesh.compute_tangents();
    return mesh;
}

std::string GenerationResult::dump() const {
    std::string out = "terminals " + std::to_string(terminals.size()) + "\n";
    for (const Shape& shape : terminals) {
        out += dump_shape(shape);
    }
    return out;
}

// ============================================================================
// Argument helpers for the handlers
// ============================================================================

namespace {

/// A number argument, or a reported fault and a request to abandon the shape
[[nodiscard]] bool arg_number(OperationArgs& context, size_t index, double& out) {
    if (index >= context.args.size() || !context.args[index].is_number()) {
        context.interpreter.fail_shape(
            context.loc, "'" + std::string{context.name} + "' wants a number for argument " +
                             std::to_string(index + 1));
        return false;
    }
    out = context.args[index].as_number();
    return true;
}

/// A string argument, same contract as arg_number()
[[nodiscard]] bool arg_text(OperationArgs& context, size_t index, std::string& out) {
    if (index >= context.args.size() || !context.args[index].is_text()) {
        context.interpreter.fail_shape(
            context.loc, "'" + std::string{context.name} + "' wants a string for argument " +
                             std::to_string(index + 1));
        return false;
    }
    out = context.args[index].as_text();
    return true;
}

[[nodiscard]] bool arg_bool(OperationArgs& context, size_t index, bool& out) {
    if (index >= context.args.size() || !context.args[index].is_bool()) {
        context.interpreter.fail_shape(
            context.loc, "'" + std::string{context.name} + "' wants a boolean for argument " +
                             std::to_string(index + 1));
        return false;
    }
    out = context.args[index].as_bool();
    return true;
}

/// Turn an OpResult failure into a reported fault that abandons the shape
void check(OperationArgs& context, const OpResult& result) {
    if (!result.ok) {
        context.interpreter.fail_shape(
            context.loc, "'" + std::string{context.name} + "': " + result.message);
    }
}

/// The three-number argument list that translate, rotate and scale share
[[nodiscard]] bool arg_vec3(OperationArgs& context, glm::dvec3& out) {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (!arg_number(context, 0, x) || !arg_number(context, 1, y) || !arg_number(context, 2, z)) {
        return false;
    }
    out = glm::dvec3{x, y, z};
    return true;
}

/// An axis named by a string argument
[[nodiscard]] bool arg_axis(OperationArgs& context, size_t index, ScopeAxis& out) {
    std::string text;
    if (!arg_text(context, index, text)) {
        return false;
    }
    if (!parse_scope_axis(text, out)) {
        context.interpreter.fail_shape(context.loc, "'" + std::string{context.name} +
                                                        "' does not know the axis '" + text +
                                                        "'; it knows x, y and z");
        return false;
    }
    return true;
}

// ============================================================================
// D2 operation handlers
// ============================================================================

void op_extrude(OperationArgs& context) {
    // extrude(d) goes up the scope's y axis, which is what a footprint means by
    // "up". extrude(axis, d) names the axis, and "normal" uses the dominant
    // face's own normal -- which is the reading ast.hpp's summary gives it.
    if (context.args.size() == 1) {
        double distance = 0.0;
        if (!arg_number(context, 0, distance)) {
            return;
        }
        check(context, extrude_shape(context.shape, ScopeAxis::Y, distance));
        return;
    }

    std::string axis_text;
    double distance = 0.0;
    if (!arg_text(context, 0, axis_text) || !arg_number(context, 1, distance)) {
        return;
    }
    if (axis_text == "normal") {
        check(context, extrude_shape_along_normal(context.shape, distance));
        return;
    }
    ScopeAxis axis = ScopeAxis::Y;
    if (!parse_scope_axis(axis_text, axis)) {
        context.interpreter.fail_shape(
            context.loc, "'extrude' does not know the axis '" + axis_text +
                             "'; it knows x, y, z and normal");
        return;
    }
    check(context, extrude_shape(context.shape, axis, distance));
}

void op_offset(OperationArgs& context) {
    double distance = 0.0;
    if (!arg_number(context, 0, distance)) {
        return;
    }
    OffsetSelector selector = OffsetSelector::Inside;
    if (context.args.size() >= 2) {
        std::string text;
        if (!arg_text(context, 1, text)) {
            return;
        }
        if (!parse_offset_selector(text, selector)) {
            context.interpreter.fail_shape(context.loc,
                                           "'offset' does not know the selector '" + text +
                                               "'; it knows inside, border and all");
            return;
        }
    }
    check(context, offset_shape(context.shape, distance, selector));
}

void op_setback(OperationArgs& context) {
    double distance = 0.0;
    if (!arg_number(context, 0, distance)) {
        return;
    }
    bool keep = true;
    if (context.args.size() >= 2 && !arg_bool(context, 1, keep)) {
        return;
    }

    // The border comes back in the frame the shape had BEFORE the setback, and
    // setback_shape() refits the shape's own scope. So the frame is captured
    // first; reading it afterwards would put the border half a setback away.
    const Scope before = context.shape.scope;

    ShapeGeometry border;
    const OpResult result = setback_shape(context.shape, distance, border, keep);
    if (!result.ok) {
        check(context, result);
        return;
    }
    if (!keep || border.faces.empty()) {
        return;
    }

    Shape border_shape = context.shape;  // for the attributes and the material
    border_shape.scope = before;
    border_shape.geometry = std::move(border);
    refit_scope(border_shape);
    (void)context.interpreter.emit_derived_terminal(context.shape, std::move(border_shape),
                                                    "setback.border");
}

void op_taper(OperationArgs& context) {
    double height = 0.0;
    if (!arg_number(context, 0, height)) {
        return;
    }
    check(context, taper_shape(context.shape, height));
}

void op_translate(OperationArgs& context) {
    glm::dvec3 delta{0.0};
    if (!arg_vec3(context, delta)) {
        return;
    }
    translate_shape(context.shape, delta);
}

void op_rotate(OperationArgs& context) {
    glm::dvec3 degrees{0.0};
    if (!arg_vec3(context, degrees)) {
        return;
    }
    rotate_shape(context.shape, degrees);
}

void op_rotate_scope(OperationArgs& context) {
    glm::dvec3 degrees{0.0};
    if (!arg_vec3(context, degrees)) {
        return;
    }
    rotate_scope_shape(context.shape, degrees);
}

void op_scale(OperationArgs& context) {
    glm::dvec3 size{0.0};
    if (!arg_vec3(context, size)) {
        return;
    }
    const OpResult result = scale_shape(context.shape, size);
    if (!result.ok) {
        // Reported but NOT abandoned: scale applies what it can, and losing a
        // whole building because one axis was flat is a worse answer than a
        // building with a message against it.
        context.interpreter.report(Severity::Error, context.loc,
                                   "'scale': " + result.message);
    }
}

void op_align_scope(OperationArgs& context) {
    std::string text;
    if (!arg_text(context, 0, text)) {
        return;
    }
    AlignMode mode = AlignMode::World;
    if (!parse_align_mode(text, mode)) {
        context.interpreter.fail_shape(context.loc,
                                       "'align_scope' does not know the mode '" + text +
                                           "'; it knows world, y_up and geometry");
        return;
    }
    check(context, align_scope_shape(context.shape, mode));
}

void op_set_pivot(OperationArgs& context) {
    std::string text;
    if (!arg_text(context, 0, text)) {
        return;
    }
    PivotAnchor anchor = PivotAnchor::Origin;
    if (!parse_pivot_anchor(text, anchor)) {
        context.interpreter.fail_shape(
            context.loc, "'set_pivot' does not know the anchor '" + text +
                             "'; it knows origin, center, max, center_bottom and center_top");
        return;
    }
    set_pivot_shape(context.shape, anchor);
}

void op_mirror(OperationArgs& context) {
    ScopeAxis axis = ScopeAxis::X;
    if (!arg_axis(context, 0, axis)) {
        return;
    }
    mirror_shape(context.shape, axis);
}

void op_mirror_scope(OperationArgs& context) {
    ScopeAxis axis = ScopeAxis::X;
    if (!arg_axis(context, 0, axis)) {
        return;
    }
    mirror_scope_shape(context.shape, axis);
}

void op_reverse_normals(OperationArgs& context) {
    reverse_normals_shape(context.shape);
}

void op_print(OperationArgs& context) {
    std::string line;
    for (size_t i = 0; i < context.args.size(); ++i) {
        if (i != 0) {
            line += " ";
        }
        line += context.args[i].to_text();
    }
    context.interpreter.log(std::move(line));
}

// ============================================================================
// D2 function library
// ============================================================================

/// A number argument to a function, or a reported fault
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

/// One-argument numeric function, e.g. sqrt
[[nodiscard]] FunctionHandler unary_fn(double (*fn)(double)) {
    return [fn](const FunctionArgs& context) {
        double x = 0.0;
        if (!fn_number(context, 0, x)) {
            return Value::number(0.0);
        }
        return Value::number(fn(x));
    };
}

Value fn_min(const FunctionArgs& context) {
    if (context.args.empty()) {
        context.interpreter.fail_shape(context.loc, "'min' needs at least one number");
        return Value::number(0.0);
    }
    double best = 0.0;
    for (size_t i = 0; i < context.args.size(); ++i) {
        double x = 0.0;
        if (!fn_number(context, i, x)) {
            return Value::number(0.0);
        }
        best = i == 0 ? x : std::min(best, x);
    }
    return Value::number(best);
}

Value fn_max(const FunctionArgs& context) {
    if (context.args.empty()) {
        context.interpreter.fail_shape(context.loc, "'max' needs at least one number");
        return Value::number(0.0);
    }
    double best = 0.0;
    for (size_t i = 0; i < context.args.size(); ++i) {
        double x = 0.0;
        if (!fn_number(context, i, x)) {
            return Value::number(0.0);
        }
        best = i == 0 ? x : std::max(best, x);
    }
    return Value::number(best);
}

Value fn_clamp(const FunctionArgs& context) {
    double x = 0.0;
    double lo = 0.0;
    double hi = 0.0;
    if (!fn_number(context, 0, x) || !fn_number(context, 1, lo) || !fn_number(context, 2, hi)) {
        return Value::number(0.0);
    }
    if (lo > hi) {
        context.interpreter.fail_shape(context.loc, "'clamp' was given a low bound of " +
                                                        format_number(lo) +
                                                        " above its high bound of " +
                                                        format_number(hi));
        return Value::number(0.0);
    }
    return Value::number(std::min(std::max(x, lo), hi));
}

Value fn_pow(const FunctionArgs& context) {
    double base = 0.0;
    double exponent = 0.0;
    if (!fn_number(context, 0, base) || !fn_number(context, 1, exponent)) {
        return Value::number(0.0);
    }
    const double value = std::pow(base, exponent);
    if (!std::isfinite(value)) {
        context.interpreter.fail_shape(context.loc, "'pow' of " + format_number(base) + " and " +
                                                        format_number(exponent) +
                                                        " is not a finite number");
        return Value::number(0.0);
    }
    return Value::number(value);
}

Value fn_sqrt(const FunctionArgs& context) {
    double x = 0.0;
    if (!fn_number(context, 0, x)) {
        return Value::number(0.0);
    }
    if (x < 0.0) {
        // std::sqrt of a negative returns NaN, and a NaN in a coordinate makes
        // two identical runs compare different, because NaN is not equal to
        // itself. Saying so at the line that asked is the only useful answer.
        context.interpreter.fail_shape(context.loc,
                                       "'sqrt' of the negative number " + format_number(x));
        return Value::number(0.0);
    }
    return Value::number(std::sqrt(x));
}

Value fn_sin(const FunctionArgs& context) {
    double degrees = 0.0;
    if (!fn_number(context, 0, degrees)) {
        return Value::number(0.0);
    }
    double s = 0.0;
    double c = 0.0;
    deg_sin_cos(degrees, s, c);
    return Value::number(s);
}

Value fn_cos(const FunctionArgs& context) {
    double degrees = 0.0;
    if (!fn_number(context, 0, degrees)) {
        return Value::number(0.0);
    }
    double s = 0.0;
    double c = 0.0;
    deg_sin_cos(degrees, s, c);
    return Value::number(c);
}

Value fn_tan(const FunctionArgs& context) {
    double degrees = 0.0;
    if (!fn_number(context, 0, degrees)) {
        return Value::number(0.0);
    }
    double s = 0.0;
    double c = 0.0;
    deg_sin_cos(degrees, s, c);
    if (c == 0.0) {
        context.interpreter.fail_shape(context.loc,
                                       "'tan' is undefined at " + format_number(degrees) +
                                           " degrees");
        return Value::number(0.0);
    }
    return Value::number(s / c);
}

Value fn_len(const FunctionArgs& context) {
    if (context.args.empty()) {
        context.interpreter.fail_shape(context.loc, "'len' needs an argument");
        return Value::number(0.0);
    }
    const Value& value = context.args[0];
    if (value.is_array()) {
        return Value::number(static_cast<double>(value.as_array().size()));
    }
    if (value.is_text()) {
        return Value::number(static_cast<double>(value.as_text().size()));
    }
    context.interpreter.fail_shape(context.loc, "'len' wants an array or a string, not a " +
                                                    std::string{value.type_name()});
    return Value::number(0.0);
}

Value fn_str(const FunctionArgs& context) {
    if (context.args.empty()) {
        context.interpreter.fail_shape(context.loc, "'str' needs an argument");
        return Value::text("");
    }
    return Value::text(context.args[0].to_text());
}

Value fn_geometry_area(const FunctionArgs& context) {
    return Value::number(geometry_area(context.shape.geometry));
}

Value fn_geometry_volume(const FunctionArgs& context) {
    return Value::number(geometry_volume(context.shape.geometry));
}

Value fn_geometry_face_count(const FunctionArgs& context) {
    return Value::number(static_cast<double>(context.shape.geometry.faces.size()));
}

Value fn_geometry_face_area(const FunctionArgs& context) {
    double raw = 0.0;
    if (!fn_number(context, 0, raw)) {
        return Value::number(0.0);
    }
    const double rounded = std::nearbyint(raw);
    const size_t count = context.shape.geometry.faces.size();
    if (!(rounded >= 0.0) || rounded >= static_cast<double>(count)) {
        // A rule that asks for face 7 of a cube is a user error, not a crash and
        // not a silent zero. The message names both numbers, because "index out
        // of range" without them sends the author back to count the faces.
        context.interpreter.fail_shape(context.loc, "face " + format_number(raw) +
                                                        " is outside the shape's " +
                                                        std::to_string(count) + " faces");
        return Value::number(0.0);
    }
    const Face& face = context.shape.geometry.faces[static_cast<size_t>(rounded)];
    return Value::number(face_area(context.shape.geometry, face));
}

[[nodiscard]] OperationTable build_standard_operations() {
    OperationTable table;
    table.register_operation("align_scope", op_align_scope);
    table.register_operation("extrude", op_extrude);
    table.register_operation("mirror", op_mirror);
    table.register_operation("mirror_scope", op_mirror_scope);
    table.register_operation("offset", op_offset);
    table.register_operation("print", op_print);
    table.register_operation("reverse_normals", op_reverse_normals);
    table.register_operation("rotate", op_rotate);
    table.register_operation("rotate_scope", op_rotate_scope);
    table.register_operation("scale", op_scale);
    table.register_operation("set_pivot", op_set_pivot);
    table.register_operation("setback", op_setback);
    table.register_operation("taper", op_taper);
    table.register_operation("translate", op_translate);
    return table;
}

[[nodiscard]] FunctionTable build_standard_functions() {
    FunctionTable table;
    table.register_function("abs", unary_fn(+[](double x) { return std::fabs(x); }));
    table.register_function("ceil", unary_fn(+[](double x) { return std::ceil(x); }));
    table.register_function("clamp", fn_clamp);
    table.register_function("cos", fn_cos);
    table.register_function("floor", unary_fn(+[](double x) { return std::floor(x); }));
    table.register_function("len", fn_len);
    table.register_function("max", fn_max);
    table.register_function("min", fn_min);
    table.register_function("pow", fn_pow);
    // std::nearbyint, not std::round: nearbyint honours the rounding mode and
    // breaks ties to even, so 0.5 and 1.5 both go to even numbers rather than
    // both going away from zero. Consistent tie-breaking is what keeps a split
    // count from differing by one between two platforms.
    table.register_function("round", unary_fn(+[](double x) { return std::nearbyint(x); }));
    table.register_function("sin", fn_sin);
    table.register_function("sqrt", fn_sqrt);
    table.register_function("str", fn_str);
    table.register_function("tan", fn_tan);
    table.register_function("geometry.area", fn_geometry_area);
    table.register_function("geometry.face_area", fn_geometry_face_area);
    table.register_function("geometry.face_count", fn_geometry_face_count);
    table.register_function("geometry.volume", fn_geometry_volume);
    return table;
}

} // namespace

const OperationTable& standard_operations() {
    static const OperationTable table = build_standard_operations();
    return table;
}

const FunctionTable& standard_functions() {
    static const FunctionTable table = build_standard_functions();
    return table;
}

// ============================================================================
// Entry point
// ============================================================================

GenerationResult generate(const RuleFile& file,
                          const Shape& seed,
                          const GenerationOptions& options,
                          const OperationTable* operations,
                          const FunctionTable* functions) {
    Interpreter interpreter(file, options,
                            operations != nullptr ? *operations : standard_operations(),
                            functions != nullptr ? *functions : standard_functions());
    return interpreter.run(seed);
}

} // namespace stratum::procgen::rules
