// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file interpreter.cpp
 * @brief The evaluator, the D2 operation handlers and the D2 function library
 *
 * The reasoning is in interpreter.hpp. What is here is the machinery, plus the
 * local notes on the five places it is not obvious: the unwinding type, the
 * lazy symbol resolution that lets a constant and an attribute refer to each
 * other in either order, the per-site salt that keeps two `choose` statements in
 * one rule independent, the argument coercion every handler shares, and
 * begin_child(), through which every child shape in the language is born.
 *
 * The geometry of `split` and `select` is NOT here. It is in op_split.cpp and
 * op_comp.cpp, and this file is the seat those two are wired into: the sizing
 * solver, the slab cut and the component decomposition know nothing about
 * frames, seeds or caps, and everything about frames, seeds and caps is here.
 */

#include "procgen/rules/interpreter.hpp"

#include "procgen/rules/op_comp.hpp"
#include "procgen/rules/op_split.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <utility>
#include <variant>

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

    /**
     * @brief Where the CURRENT rule's bindings start in @p locals
     *
     * The search floor, and the whole of what makes this language lexically
     * scoped rather than dynamically scoped. One vector holds every frame's
     * bindings, so a search that ran to index 0 would walk out of the rule being
     * evaluated and into the rule that CALLED it: `rule A(w: float) { B(); }`
     * would let `B` read `w`, and `B` would then mean something different
     * depending on who called it. That is a bug that cannot be found by reading
     * `B`, which is the worst kind this file can have.
     *
     * Saved and restored around every rule invocation, so it is the callee's
     * base while the callee runs and the caller's again afterwards. Arguments
     * are evaluated BEFORE it moves, which is what keeps `Floor(height - 1)`
     * meaning the caller's `height`.
     */
    size_t frame_base = 0;

    /// Attribute and constant values, resolved once each. See resolve_global().
    std::map<std::string, Value> globals;
    std::map<std::string, const AttrDecl*> attr_decls;
    std::map<std::string, const ConstDecl*> const_decls;
    std::set<std::string> resolving;

    /**
     * @brief File-level values whose expression failed, reported once each
     *
     * A constant that divides by zero is a fault in that constant, not in the
     * file. Recording it here rather than abandoning the run is what lets a good
     * `@start` rule still produce its building, and it is the same argument the
     * shape caps and the AbandonShape unwind make: one malformed thing must not
     * cost the other four thousand. A name in here has already been reported at
     * its declaration, so a rule that READS it gets the short message from
     * resolve_global() and abandons only its own shape.
     */
    std::set<std::string> unresolvable;

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

    /// fail(), but at most one diagnostic per site and message
    ///
    /// For a fault that one bad declaration causes at every shape that reads it:
    /// a city of four thousand lots reading one broken constant should say so
    /// once per reading SITE, not four thousand times into a hundred-entry cap
    /// that then hides everything else.
    [[noreturn]] void fail_once(const SourceLoc& loc, std::string message) {
        report_once(Severity::Error, loc, std::move(message));
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

        // Already tried and already reported, at its own declaration. Saying
        // "division by zero" again here would point at the wrong line; saying
        // which name the rule wanted points at both.
        if (unresolvable.find(name) != unresolvable.end()) {
            fail_once(use_loc, "'" + name +
                                   "' could not be worked out; see the error on its declaration");
        }

        if (!resolving.insert(name).second) {
            fail(use_loc, "'" + name + "' is defined in terms of itself");
        }

        const bool outer_in_global = in_global;
        in_global = true;

        Value value;
        try {
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
                    // value must not depend on which shape happened to read it
                    // first, and eval() refuses `shape.*` while in_global is set.
                    const Shape empty;
                    value = eval(decl.default_value, empty);
                }
            } else {
                const Shape empty;
                value = eval(konst->second->value, empty);
            }
        } catch (...) {
            // The unwind used to leave `in_global` true and `name` in
            // `resolving` for the rest of the run, because run() returned
            // straight afterwards and nobody noticed. Now that one bad
            // declaration no longer ends the generation, both would poison every
            // later shape: `shape.sx` would be refused everywhere, and a second
            // read of this name would be reported as a cycle it is not.
            in_global = outer_in_global;
            resolving.erase(name);
            unresolvable.insert(name);
            throw;
        }

        in_global = outer_in_global;
        resolving.erase(name);
        globals.emplace(name, value);
        out = std::move(value);
        return true;
    }

    /**
     * @brief resolve_global() for the file-level pre-pass, which cannot abandon
     *
     * The pre-pass has no shape to abandon, so an AbandonShape out of one
     * declaration is caught here and the name is left in `unresolvable`. The
     * next declaration is still resolved, which is what makes the source-order
     * promise above true for a file with more than one bad default.
     */
    void resolve_global_or_record(const std::string& name, const SourceLoc& loc, Value& out) {
        // Already marked while an earlier declaration was being resolved -- it
        // was reported there, and resolve_global() would only add a second
        // diagnostic saying the same thing at the same line.
        if (unresolvable.find(name) != unresolvable.end()) {
            return;
        }
        try {
            (void)resolve_global(name, loc, out);
        } catch (const AbandonShape&) {
            abandon_requested = false;
            unresolvable.insert(name);
        }
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

        // Down to frame_base, never to 0: see the note on State::frame_base. And
        // not at all while a file-level value is being resolved, because an
        // attribute default that could see whichever rule happened to read it
        // first would not be a file-level value at all.
        if (!in_global) {
            for (size_t i = locals.size(); i-- > frame_base;) {
                if (locals[i].first == name.name) {
                    return locals[i].second;
                }
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

    /**
     * @brief Run one statement
     *
     * The switch below has no `default:` so that -Wswitch names a StmtKind that
     * was added without a case here. A WARNING is not a guarantee: nothing in
     * this tree sets -Werror, so a missing case builds, and the fallthrough past
     * the switch would then be a silent no-op -- a rule statement that does
     * nothing at all, which is the failure this whole language most has to
     * avoid. Two things close that:
     *
     *   - the static_assert below, which is a hard compile error the moment
     *     ast.hpp's StmtNode grows an alternative, and which says what to do;
     *   - the report past the switch, so that even a build which somehow got
     *     past both says so at the line that asked, once per site, instead of
     *     quietly producing a building with no floors in it.
     *
     * The arms are wired DIRECTLY, not through a registry. interpreter.hpp says
     * why: a statement is a variant alternative, the population is `split`,
     * `select` and a possible `scatter`, and a table of function pointers would
     * buy indirection for three cases at the cost of the one file where control
     * flow has to read top to bottom. What makes a fourth statement cheap is not
     * a registry but begin_child() and run_statement_body(), which already own
     * everything a statement child needs.
     */
    Flow exec(StmtId id, Shape& shape, Frame& frame) {
        // A later feature adds a StmtKind. When one is added this assert fires,
        // and the fix is to give the switch below a case for it -- not to widen
        // the number. A statement with no case is a rule that silently does
        // nothing.
        static_assert(std::variant_size_v<StmtNode> == 9,
                      "ast.hpp gained a statement kind. Add its case to State::exec() "
                      "and then update this count.");

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
                return exec_split(std::get<SplitStmt>(stmt.node), stmt, shape, frame);
            case StmtKind::Select:
                return exec_select(std::get<SelectStmt>(stmt.node), stmt, shape, frame);
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
        // Unreachable while every StmtKind above has a case, and reported rather
        // than ignored for the build in which one does not. `split` and `select`
        // already refuse to be quiet about being unimplemented; a statement kind
        // nobody wired up at all has no better claim to silence. Evaluation
        // continues so the rest of the rule still produces its geometry.
        report_once(Severity::Error, stmt.loc,
                    "this statement is not implemented in this build");
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

        // A rule call's child starts as a COPY of the caller's shape: the callee
        // transforms what it was handed. A statement child starts as a slab or a
        // component instead, which is the only difference between the three.
        Shape child;
        if (begin_child(shape, frame, shape, call.callee.name, stmt.loc, child) !=
            ChildOutcome::Ready) {
            return;
        }

        invoke_rule(call.rule, std::move(child), args, stmt.loc);
    }

    // ---- statement children ----

    /**
     * @brief What begin_child() did
     *
     * Only Ready leaves the caller anything to do. Both other outcomes have
     * already reported and already done whatever the cap asks for, so a caller
     * that treats them alike -- go on to the next sibling -- is correct.
     */
    enum class ChildOutcome : uint8_t {
        Ready,     ///< @p out is a live child shape
        Refused,   ///< The shape cap said no. Nothing was created, nothing emitted.
        Truncated  ///< The depth cap cut it. The child was emitted as a terminal.
    };

    /**
     * @brief Give a shape a child, or say why there is no body to run
     *
     * EVERY child in this language is born here. A rule call, a split slab and a
     * `select` component are the same act -- one shape handing out an address in
     * the tree -- and differ only in what content goes in and what runs against
     * it afterwards. The four lines that carry the language's promises live here
     * exactly once: the sibling index, the seed mix, the shape cap and the depth
     * cap. A statement family that copied them instead would be a working split
     * and a subtly different seed, which no test reads as a failure until a
     * reproduction case stops reproducing.
     *
     * @param parent  Shape the child hangs off. Not modified.
     * @param frame   The enclosing rule's frame. Its child counter IS the child's
     *                sibling index, and is advanced here.
     * @param content Geometry, scope and attributes for the child. Its tree
     *                fields -- rule, loc, depth, index, parent, seed_key, id --
     *                are all overwritten, so a caller passes the parent, a slab
     *                or a component and does not have to know which fields
     *                travel.
     * @param role    Rule name to record: "Floor", "split[2]", "select.front[0]"
     * @param loc     Where the rule asked, for the cap diagnostics
     * @param out     Receives the child, and only when the result is Ready
     */
    /// How a child is addressed within its parent, which is what its seed is
    /// mixed from.
    ///
    /// Sibling order is right for a rule call and for a split part: the third
    /// slab is the third slab whatever else the rule does. It is WRONG for a
    /// `select` arm. A component child is "face 3 of this solid", and keying it
    /// by claim order means adding a `bottom:` arm renumbers every `front:`
    /// child and reshuffles its random variation -- the author edits one arm and
    /// the rest of the building changes. That is the same stable-identity
    /// problem lots.cpp solves for lot ids, and it has the same answer: address
    /// the thing by what it IS.
    enum class ChildAddress { Sibling, Component };

    ChildOutcome begin_child(const Shape& parent,
                             Frame& frame,
                             Shape content,
                             std::string role,
                             const SourceLoc& loc,
                             Shape& out,
                             ChildAddress addressing = ChildAddress::Sibling,
                             uint32_t component_index = 0) {
        content.rule = std::move(role);
        content.loc = loc;
        content.depth = parent.depth + 1;
        content.index = frame.children;
        content.parent = parent.id;
        // The child's address in the tree: its parent's address mixed with its
        // own address within the parent. Never a draw from a shared stream --
        // see the determinism note in interpreter.hpp. Nothing outside this
        // parent feeds the mix, so a split of four parts gives the same four
        // keys on every run and whatever was generated before it.
        //
        // The salts differ so the two numbering schemes cannot collide:
        // `rule Main { A(); select face { all : { P(); } } }` gives A index 0
        // as a sibling and component 0 index 0 as a component, and they must
        // not land on the same key.
        content.seed_key =
            addressing == ChildAddress::Component
                ? seed_mix2(parent.seed_key, seed_mix2(kSaltComponent, component_index))
                : seed_mix2(parent.seed_key, seed_mix2(kSaltChild, content.index));

        if (!allocate_shape(content, loc)) {
            // The child does NOT count against the parent, so a parent whose
            // every child the cap refused is still a terminal and still emits
            // its own geometry. Counting it would turn "the output is
            // truncated" into "there is no output", which is the empty viewport
            // the depth cap goes out of its way to avoid.
            //
            // No surviving shape's seed moves because of this: shapes_created
            // never decreases, so once one child is refused every later one is
            // too, and no shape that exists ever gets a different index.
            return ChildOutcome::Refused;
        }
        ++frame.children;

        // GenerationStats::max_depth_reached is NOT updated here. It counts the
        // depth actually EVALUATED, and the child below has not been: a cap of 4
        // that truncates at depth 5 reached 4. invoke_rule() and
        // run_statement_body() record it, on the one path where the child is run.
        if (content.depth > options.limits.max_depth) {
            if (!depth_reported) {
                depth_reported = true;
                result.stats.depth_limit_hit = true;
                report(Severity::Error, loc,
                       "the depth limit of " + std::to_string(options.limits.max_depth) +
                           " was reached at '" + content.rule +
                           "'; the recursion was cut and the shape emitted as it stood");
            }
            // Emitted rather than dropped. Dropping it would leave a runaway
            // tower with no output at all, because every shape above it is
            // non-terminal by virtue of having made a child. Statement children
            // come through here for the same reason: a rule that splits and then
            // calls itself runs away exactly as readily as one that only calls
            // itself, and a cap that produced NO output would be the worse of
            // the two failures in both cases.
            emit_terminal(std::move(content));
            return ChildOutcome::Truncated;
        }

        out = std::move(content);
        return ChildOutcome::Ready;
    }

    /**
     * @brief Run a split part's or a `select` arm's body against its child shape
     *
     * This is invoke_rule() minus the two things that make a CALL a call, and
     * the subtraction is the point.
     *
     * No parameters, because a body has none. And no move of State::frame_base,
     * because a body is written inside the rule it belongs to and must read that
     * rule's bindings: `split(y) { storey : { Floor(storey); } }` is the ordinary
     * way to write a facade, and it only works if `storey` is still in view. Its
     * OWN lets do not leak back out -- the language was dynamically scoped until
     * two days ago, and a body that left its bindings behind would put half of
     * that back on the error path without touching lookup_name().
     *
     * Two resizes do that, and only one of them can fire. A body is a BlockStmt,
     * and exec()'s Block case pops its own mark when it ENDS, so the resize on
     * the normal path below is a backstop for a body that is not a block -- there
     * is none today, and no test can reach it. The resize in the CATCH is the one
     * that matters: an abandoned body never reaches the end of its block, so
     * without it the bindings it had pushed would still be on the stack for the
     * next part to find.
     *
     * Everything else matches invoke_rule(). The body gets a Frame of its own, so
     * rule calls inside it count as @p child's children rather than as the
     * PARENT's -- without that, a slab whose body calls a rule would make its
     * grandparent non-terminal and would hand the next slab the wrong sibling
     * index. A body that makes no child and does not discard emits @p child as a
     * terminal, which is what makes `1.0 : { }` a piece of wall. A body that
     * discards, or that abandons, costs its own subtree and nothing else: one
     * malformed slab must not cost the other floors of the facade.
     *
     * @param body      BlockStmt to run, or kNoNode for a part that has none
     * @param child     The slab or component. Consumed.
     * @param enclosing The frame the statement was written in, for its rule id
     */
    void run_statement_body(StmtId body, Shape child, const Frame& enclosing) {
        result.stats.max_depth_reached = std::max(result.stats.max_depth_reached, child.depth);

        Frame inner;
        inner.rule = enclosing.rule;
        inner.locals_base = locals.size();
        try {
            const Flow flow = body == kNoNode ? Flow::Continue : exec(body, child, inner);
            locals.resize(inner.locals_base);
            if (flow != Flow::Discard && inner.children == 0) {
                emit_terminal(std::move(child));
            }
        } catch (const AbandonShape&) {
            locals.resize(inner.locals_base);
            abandon_requested = false;
        }
    }

    // ---- split ----

    /**
     * @brief `split(y) { 4.0 : { Shopfront(); } repeat { 2.0 : { Floor(); } } }`
     *
     * The solver and the cut are op_split.cpp's, and this is the wiring block
     * its file header asks for. What is decided HERE, and nowhere else:
     *
     *   - The sizes are evaluated against the shape being split, in the frame
     *     that wrote the statement, BEFORE any slab exists. So a size reads the
     *     enclosing rule's bindings and never a slab's, and `shape.sy` in a size
     *     is the height being divided rather than the height of a piece of it.
     *   - A size that is not a number abandons the whole shape rather than one
     *     slab. The layout cannot be solved at all when one row of it is
     *     unreadable, so there is no truncated facade to fall back to.
     *   - Each slab is a child, with the caps and the seed mix every other child
     *     gets, and its body runs in the current frame.
     */
    Flow exec_split(const SplitStmt& node, const Stmt& stmt, Shape& shape, Frame& frame) {
        const ScopeAxis axis = to_scope_axis(node.axis);

        const std::vector<SplitPart> parts =
            split_parts_from_statement(node, [&](ExprId id, const SourceLoc& at) {
                return want_number(eval(id, shape), at, "a split size");
            });

        const SplitLayout layout = solve_split(parts, shape.scope.extent(axis));
        for (const std::string& note : layout.notes) {
            // Once per site, not once per shape. A split inside a recursive rule
            // reports the same overflow at every storey otherwise, and one
            // mistake fills a diagnostic cap that then hides everything else.
            report_once(Severity::Warning, stmt.loc, note);
        }

        std::vector<Shape> slabs = split_shape(shape, axis, layout);
        // split_shape() promises one slab per piece. Indexing the shorter of the
        // two anyway costs one comparison and turns a broken promise into a short
        // facade rather than into a read past the end.
        const size_t count = std::min(slabs.size(), layout.pieces.size());
        for (size_t i = 0; i < count; ++i) {
            Shape child;
            if (begin_child(shape, frame, std::move(slabs[i]),
                            "split[" + std::to_string(i) + "]", stmt.loc,
                            child) != ChildOutcome::Ready) {
                continue;
            }
            // The body is a StmtId and not a rule call, so it runs in the frame
            // that wrote it rather than through invoke_rule().
            run_statement_body(split_body_for(node, layout.pieces[i]), std::move(child), frame);
        }
        return Flow::Continue;
    }

    // ---- select ----

    /**
     * @brief Turn what the decomposition found into diagnostics
     *
     * A function of its own, and not a block inside exec_select(), because the
     * SEVERITIES are a policy decision rather than an implementation detail. The
     * sentences are op_comp.cpp's -- ComponentSplitReport hands out finished
     * message fragments precisely so two callers cannot word them differently --
     * but the policy here is NOT emit_components()'s, in two deliberate ways:
     *
     *   - **Once per site, not once per shape.** A `select` inside a recursive
     *     rule meets the same bent facade at every storey, and four thousand lots
     *     would fill GenerationResult's diagnostic cap with one mistake and hide
     *     everything else behind it. emit_components() reports every time because
     *     its caller is an operation the rule invoked on purpose, once per call.
     *
     *   - **"this shape has no geometry" is a Warning here, not an Error.**
     *     op_comp.hpp argues that everything in ComponentSplitReport::problems is
     *     a mistake in the RULE, and for a caller that chose the shape it is. A
     *     `select` inside a split part did not choose its shape: op_split.cpp
     *     deliberately produces zero-length slabs when the fixed parts overrun the
     *     extent, and deliberately runs their bodies against the empty shape. A
     *     lot narrower than its own piers is a Warning-grade event -- the split
     *     solver already calls it one -- so an Error here would turn an ordinary
     *     facade on a narrow lot into a generation that ok() reports as failed.
     *     Every OTHER problem stays an Error, because an unimplemented domain, an
     *     index past the end and an empty angle range are all things the rule text
     *     asked for and got wrong.
     *
     * The empty-shape test mirrors split_components()' own condition rather than
     * matching the sentence it produces. Matching the text would break silently
     * the day someone rewords it. Mirroring the condition can drift instead, and
     * that drift is safe in the one direction that matters: the worst it does is
     * report an Error where a Warning was meant, which is the old behaviour and is
     * loud. Downgrading every problem is downgrading exactly one, because
     * split_components() pushes the no-geometry sentence and returns immediately,
     * so an empty shape on an implemented domain cannot carry a second problem.
     *
     * @param split_report What split_components() found. Not named `report`,
     *                     which would shadow State::report() inside this body.
     * @param shape        The shape that was decomposed, for the empty-shape test
     * @param domain       What it was decomposed into
     * @param loc          The `select` statement, so every message lands on it
     */
    void report_component_split(const ComponentSplitReport& split_report,
                                const Shape& shape,
                                ComponentDomain domain,
                                const SourceLoc& loc) {
        const bool domain_implemented =
            domain == ComponentDomain::Face || domain == ComponentDomain::Edge;
        const bool shape_is_empty =
            shape.geometry.faces.empty() || shape.geometry.positions.empty();
        const Severity problem_severity =
            domain_implemented && shape_is_empty ? Severity::Warning : Severity::Error;

        for (const std::string& problem : split_report.problems) {
            report_once(problem_severity, loc, problem);
        }
        if (split_report.degenerate > 0) {
            // A Warning: a degenerate face is something the INPUT geometry did,
            // not something the rule author wrote. The same argument
            // emit_components() makes, and the same sentence.
            report_once(Severity::Warning, loc,
                        std::to_string(split_report.degenerate) +
                            " components were dropped for having no area, no length or no "
                            "direction");
        }
        if (split_report.non_planar > 0) {
            report_once(Severity::Warning, loc,
                        std::to_string(split_report.non_planar) +
                            " of these faces do not lie flat; the worst is out of plane by " +
                            format_number(split_report.worst_planarity) +
                            ", which is the depth of its component scope");
        }
    }

    /**
     * @brief `select face { front : { Facade(); } top : { Roof(); } }`
     *
     * The decomposition is op_comp.cpp's. What is decided here is the mapping
     * from components to arms, and it is the half that is easy to get wrong.
     *
     * The shape is decomposed ONCE, with no selector, and the arms are applied to
     * the result. Calling split_components() once per arm instead would decompose
     * the shape once per arm, and would visit the children in ARM order: two files
     * that say the same thing with `side:` above `top:` and below it would then
     * hand every child a different sibling index and a different seed_key.
     * Decomposing once buys exactly that -- the children are visited in
     * op_comp.hpp's canonical component order, so the ORDER the arms are written
     * in moves no key at all.
     *
     * What it does NOT buy is worth stating, because the obvious reading of the
     * paragraph above claims more than the code does. A child's sibling index is
     * the number of components CLAIMED before it, not Component::index. So adding
     * an arm that claims a component earlier in the canonical order -- a `bottom:`
     * arm to a file that had only `front:` -- shifts every later child by one and
     * re-keys it. That is an edit to the rule TEXT, which is allowed to change the
     * output; it is recorded here because it changes more of the output than it
     * looks like it should, and because the alternative is not free either:
     * keying a child by Component::index would collide with the rule calls made
     * before the `select`, which are numbered from the same counter.
     * `select_children_are_addressed_by_claim_count_so_adding_an_arm_re_keys_them`
     * in test_rule_statements.cpp pins it either way. Nothing OUTSIDE this
     * `select` moves: a key is mixed from the parent and the index and is never
     * drawn from a shared stream.
     *
     * Each component goes to the FIRST arm that admits it. The six direction
     * words partition the components, so a file listing all six covers the shape
     * with nothing counted twice; the words that overlap -- `side`, `vertical`,
     * `all` -- are resolved by source order, which is the only reading an author
     * can predict from the text. A component no arm claims makes no child, which
     * is how `select face { top : Roof(); }` says it wants the roof and nothing
     * else.
     */
    Flow exec_select(const SelectStmt& node, const Stmt& stmt, Shape& shape, Frame& frame) {
        ComponentSplitReport split_report;
        std::vector<Component> components =
            split_components(shape, node.domain, ComponentSelection{}, &split_report);

        report_component_split(split_report, shape, node.domain, stmt.loc);

        for (Component& component : components) {
            const SelectCase* arm = nullptr;
            for (const SelectCase& candidate : node.cases) {
                if (selector_admits(candidate.selector, component.normal)) {
                    arm = &candidate;
                    break;
                }
            }
            if (arm == nullptr) {
                continue;
            }

            std::string role = "select." +
                               std::string{component_selector_name(arm->selector)} + "[" +
                               std::to_string(component.index) + "]";
            Shape child;
            // Addressed by WHICH COMPONENT it is, not by claim order. Adding a
            // `bottom:` arm must not renumber the `front:` children and
            // reshuffle their variation; see ChildAddress.
            const uint32_t component_index = component.index;
            if (begin_child(shape, frame, std::move(component.shape), std::move(role), stmt.loc,
                            child, ChildAddress::Component, component_index)
                != ChildOutcome::Ready) {
                continue;
            }
            run_statement_body(arm->body, std::move(child), frame);
        }
        return Flow::Continue;
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

        // The callee's bindings start here, and nothing it evaluates may look
        // below this line. Restored on both paths out of the try below.
        const size_t outer_frame_base = frame_base;
        frame_base = frame.locals_base;

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
        frame_base = outer_frame_base;
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

        // Forced in declaration order so the diagnostics of a file with several
        // bad defaults come out in source order rather than in whatever order a
        // rule happened to read them.
        //
        // The catch is INSIDE the loop, per declaration. With one try around the
        // whole loop the first bad default ended the pre-pass, so the second one
        // was never reached and the promise of source order above was a promise
        // about a list of length one. Worse, it returned before the start rule
        // ran at all: a `const junk : float = 1.0 / 0.0` that no rule reads cost
        // the entire generation and the author got zero terminals for a value
        // nothing wanted. A declaration that fails is recorded as unresolvable
        // and only the shapes that READ it are abandoned.
        Value ignored;
        for (const AttrDecl& decl : file.attributes) {
            resolve_global_or_record(decl.name, decl.loc, ignored);
        }
        for (const ConstDecl& decl : file.constants) {
            resolve_global_or_record(decl.name, decl.loc, ignored);
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

bool Interpreter::has_current_shape() const {
    return !state_->in_global;
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

/**
 * @brief Refuse a geometry query that has no shape to query
 *
 * `FunctionArgs::shape` is a default-constructed Shape while an attribute
 * default or a constant is being resolved, and every geometry query over it
 * answers zero. Zero is a plausible number, so it does not look like a fault --
 * it looks like an empty lot, and the building comes out wrong with nothing
 * pointing at the declaration that asked. The interpreter already refuses
 * `shape.sx` there for the same reason; a registered function has to ask.
 */
[[nodiscard]] bool fn_needs_shape(const FunctionArgs& context) {
    if (context.interpreter.has_current_shape()) {
        return true;
    }
    context.interpreter.fail_shape(
        context.loc, "'" + std::string{context.name} +
                         "' reads the current shape, which an attribute default or a "
                         "constant does not have");
    return false;
}

Value fn_geometry_area(const FunctionArgs& context) {
    if (!fn_needs_shape(context)) {
        return Value::number(0.0);
    }
    return Value::number(geometry_area(context.shape.geometry));
}

Value fn_geometry_volume(const FunctionArgs& context) {
    if (!fn_needs_shape(context)) {
        return Value::number(0.0);
    }
    return Value::number(geometry_volume(context.shape.geometry));
}

Value fn_geometry_face_count(const FunctionArgs& context) {
    if (!fn_needs_shape(context)) {
        return Value::number(0.0);
    }
    return Value::number(static_cast<double>(context.shape.geometry.faces.size()));
}

Value fn_geometry_face_area(const FunctionArgs& context) {
    if (!fn_needs_shape(context)) {
        return Value::number(0.0);
    }
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
