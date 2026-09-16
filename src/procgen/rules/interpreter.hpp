// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file interpreter.hpp
 * @brief The evaluator: what turns a parsed rule file into shapes
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ================================================================================
 * WHAT THIS IS
 * ================================================================================
 *
 * D1 produced a lexer, an AST and a parser, and no evaluator. This is the piece
 * that makes the language real. D3 (split and repeat), D4 (component split, roofs,
 * primitives), D5 (stochastics) and D6 (attributes) plug into what is defined
 * here, so the interfaces matter more than the operation count does.
 *
 * ### The evaluation loop, in one paragraph
 *
 * A rule invocation takes a shape and a RuleDecl. It runs the rule's statements
 * in order against that shape, mutating it: an operation transforms the shape, a
 * `let` binds a name, an `if` chooses a branch. A statement that calls another
 * RULE snapshots the shape as it stands at that point, makes it a child, and
 * recurses. So two rule calls in one body produce two children, and the second
 * sees the operations that ran between them -- which is what makes
 * `translate(1,0,0); A(); translate(2,0,0); B();` put A and B in different
 * places. When the body ends, a shape that produced no children and was not
 * discarded is a TERMINAL: it is the output.
 *
 * ================================================================================
 * THE THREE THINGS THAT ARE EASY TO GET WRONG
 * ================================================================================
 *
 * ### Recursion bounds
 *
 * A rule that calls itself is the normal way to write a tower. It is also the
 * normal way to hang the editor, because nothing in the language forces the
 * recursion to end and an author who forgets the `if` gets an infinite tree. Two
 * caps, both in InterpreterLimits, and both REPORTED rather than silently hit:
 *
 *   - **A depth cap.** When a call would exceed it, the child is not recursed
 *     into -- it is emitted as a TERMINAL instead. That matters: refusing to
 *     create it at all would make a runaway tower produce no output whatsoever,
 *     since every shape above it is non-terminal by virtue of having made a
 *     child. Emitting it means the author sees a truncated tower, which points
 *     at the mistake, rather than an empty viewport, which does not.
 *   - **A shape cap.** A total, so that a rule which recurses shallowly but
 *     branches widely is caught too.
 *
 * Both are Severity::Error, not warnings, so GenerationResult::ok() is false and
 * no caller mistakes truncated output for finished output. A generation that hit
 * a cap produced the wrong building; the author has to know.
 *
 * ### Determinism
 *
 * Same rule text, same seed, same input shape, byte-identical output on any
 * platform. The seeding rule is the one osm/road/lots.cpp argues for and this
 * file follows: a shape's random state is derived from its ADDRESS in the tree,
 * never drawn from one shared stream. See Shape::seed_key and shape.hpp's
 * determinism section. With a shared stream, adding one `choose` arm at the top
 * of a file changes every shape generated after it, and a reproduction case
 * stops reproducing.
 *
 * Everything else that could leak an ordering is closed the same way: attributes
 * are a std::map, the operation and function tables are std::map, constants are
 * evaluated in declaration order, and nothing is read out of a hash container.
 *
 * ### Runtime error reporting
 *
 * A rule that asks for face 7 of a cube is a USER error, not a crash and not a
 * silent zero. Every diagnostic this file produces is a `Diagnostic` -- the same
 * type the lexer and parser produce -- carrying the SourceLoc of the AST node
 * that asked, so render_diagnostic() draws the same caret for a runtime fault as
 * for a syntax error and the author never has to learn two formats.
 *
 * The AST stores a location but not a token length, so a runtime diagnostic
 * underlines one byte. That is the one visible difference from a parse-time
 * message, and it is a consequence of ast.hpp's shape rather than an oversight.
 *
 * A runtime error abandons the SUBTREE it happened in, not the whole run: one
 * malformed lot in a city should not cost the other four thousand. The shape
 * being evaluated produces no output and evaluation carries on with its
 * siblings. Internally this unwinds through a private exception type, exactly as
 * parser.hpp describes for its own recovery; it never crosses this header.
 *
 * ================================================================================
 * EXTENDING IT: D3, D4, D5, D6
 * ================================================================================
 *
 * Two registries, so that a later feature adds operations and functions without
 * editing this file or shape.cpp:
 *
 *   - **OperationTable** -- statement-position built-ins, the rows of
 *     ast.hpp's kBuiltinOperations. `standard_operations()` is the D2 set.
 *   - **FunctionTable** -- expression-position built-ins: `min`, `sqrt`,
 *     `geometry.area()` and the rest. ast.hpp deliberately does NOT resolve
 *     these at parse time, precisely so that the library can grow here.
 *
 * Copy a table, register into the copy, pass it to generate(). An operation named
 * in kBuiltinOperations with no handler registered is not a silent no-op: it
 * reports "operation 'roof' is not implemented in this build" at the line that
 * called it, which is what ast.hpp says should happen to a catalogue row whose
 * implementation has not landed yet.
 *
 * STATEMENTS are a closed set -- they are variant alternatives in ast.hpp -- so
 * `split`, `repeat` and `select` cannot be registered and are implemented in
 * interpreter.cpp when D3 and D4 land. Until then they report themselves the same
 * way, once per source location, and evaluation continues so that the rest of the
 * rule still produces its geometry.
 */

#pragma once

#include "procgen/rules/ast.hpp"
#include "procgen/rules/lexer.hpp"
#include "procgen/rules/shape.hpp"

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <string_view>
#include <vector>

namespace stratum::procgen::rules {

class Interpreter;

// ============================================================================
// Limits
// ============================================================================

/**
 * @brief The bounds that stop a runaway rule
 *
 * Both are generous for anything an author writes on purpose and small enough
 * that a mistake is caught in milliseconds rather than in swap.
 */
struct InterpreterLimits {
    /**
     * @brief Deepest rule-call nesting
     *
     * A 60-storey tower written as a recursive rule is 60 deep. 64 leaves room
     * for the facade rules beneath it without letting an unbounded recursion run
     * long enough to matter.
     */
    uint32_t max_depth = 64;

    /**
     * @brief Most shapes one generate() call may create, terminals included
     *
     * The guard against wide branching, which a depth cap does not catch: a rule
     * that makes eight children per level reaches a million shapes at depth 7.
     */
    uint32_t max_shapes = 200000;
};

// ============================================================================
// Options
// ============================================================================

/**
 * @brief Everything a caller can vary about one generation
 */
struct GenerationOptions {
    /**
     * @brief The run seed
     *
     * Mixed with the input shape's own Shape::seed_key to make the root key, so
     * a caller generating one building per lot passes the lot's key on the shape
     * and one seed for the whole city, and gets shapes that differ between lots
     * and repeat exactly on a re-run.
     */
    uint64_t seed = 0;

    InterpreterLimits limits{};

    /**
     * @brief Values for `attr` declarations, as an Inspector would supply them
     *
     * A name that is not a declared attribute is reported rather than ignored:
     * a typo in an override is invisible otherwise, and the symptom is a slider
     * that does nothing. A value whose type does not match the declaration is
     * reported too, and the declared default is used.
     */
    std::map<std::string, Value> attributes;

    /**
     * @brief Rule to start from
     *
     * kNoNode uses the file's `@start` rule. A file with neither is an error:
     * there is no sensible default, and picking the first rule in the file would
     * make adding a rule at the top change what the file does.
     */
    RuleId start = kNoNode;
};

// ============================================================================
// Result
// ============================================================================

/// What one generation did, for the status bar and for a test
struct GenerationStats {
    uint32_t shapes_created = 0;     ///< Every shape, including the root and the terminals
    uint32_t terminals = 0;          ///< Shapes that produced no children
    uint32_t rules_invoked = 0;      ///< Rule bodies run
    uint32_t operations_applied = 0; ///< Built-in operations that ran
    uint32_t max_depth_reached = 0;  ///< Deepest rule-call nesting actually used
    bool depth_limit_hit = false;    ///< InterpreterLimits::max_depth stopped a recursion
    bool shape_limit_hit = false;    ///< InterpreterLimits::max_shapes stopped a recursion
};

/**
 * @brief The terminals, the diagnostics and the numbers
 */
struct GenerationResult {
    /// Output shapes, in the order the evaluation reached them, which is source order
    std::vector<Shape> terminals;

    /**
     * @brief Everything that went wrong, in the order it went wrong
     *
     * Capped at kMaxDiagnostics -- the same budget the lexer and parser count
     * against -- with kTooManyDiagnostics as the last entry. A generation that
     * earns a hundred runtime errors has one mistake being reported a hundred
     * times, and the hundred-and-first message is worth less than knowing the
     * list was cut.
     */
    std::vector<Diagnostic> diagnostics;

    /// Lines written by the `print` operation, in evaluation order
    std::vector<std::string> log;

    GenerationStats stats{};

    /// True when no diagnostic has Severity::Error
    [[nodiscard]] bool ok() const;

    /// Every terminal triangulated into one world-space mesh
    [[nodiscard]] Mesh build_mesh() const;

    /**
     * @brief A canonical text rendering of every terminal
     *
     * Deterministic and location-free, so two runs of the same rule file can be
     * compared byte for byte. This is what the determinism tests assert on, for
     * the same reason ast.hpp's dump() exists.
     */
    [[nodiscard]] std::string dump() const;
};

// ============================================================================
// Operation registry
// ============================================================================

/**
 * @brief What a built-in operation is handed
 *
 * @p shape is the shape being transformed, in place. @p args are already
 * evaluated: an operation never sees an Expr, which is what keeps D3's and D4's
 * handlers free of the AST entirely.
 */
struct OperationArgs {
    Shape& shape;
    const std::vector<Value>& args;
    SourceLoc loc;
    std::string_view name;
    Interpreter& interpreter;
};

/// One operation's implementation
using OperationHandler = std::function<void(OperationArgs&)>;

/**
 * @brief Name to handler, for statement-position built-ins
 *
 * std::map with a transparent comparator: lookups take a string_view without
 * allocating, and iteration order is the name order rather than a hash order, so
 * nothing about a generation depends on which operations happen to be registered.
 */
class OperationTable {
public:
    /// Register or replace a handler. A later registration wins, so a caller can override one.
    void register_operation(std::string_view name, OperationHandler handler);

    /// @return The handler, or nullptr when the name has none
    [[nodiscard]] const OperationHandler* find(std::string_view name) const;

    [[nodiscard]] size_t size() const { return handlers_.size(); }

private:
    std::map<std::string, OperationHandler, std::less<>> handlers_;
};

// ============================================================================
// Function registry
// ============================================================================

/// What a built-in expression function is handed
struct FunctionArgs {
    const std::vector<Value>& args;
    SourceLoc loc;
    std::string_view name;
    const Shape& shape;
    Interpreter& interpreter;
};

/// One function's implementation. Report through @p interpreter and return any
/// value on a bad call; the interpreter abandons the subtree either way.
using FunctionHandler = std::function<Value(const FunctionArgs&)>;

/// Name to handler, for expression-position built-ins. See OperationTable.
class FunctionTable {
public:
    void register_function(std::string_view name, FunctionHandler handler);

    [[nodiscard]] const FunctionHandler* find(std::string_view name) const;

    [[nodiscard]] size_t size() const { return handlers_.size(); }

private:
    std::map<std::string, FunctionHandler, std::less<>> handlers_;
};

/**
 * @brief The operations D2 implements
 *
 * extrude, offset, setback, taper, translate, rotate, rotate_scope, scale,
 * align_scope, set_pivot, mirror, mirror_scope, reverse_normals and print.
 *
 * Everything else in ast.hpp's kBuiltinOperations belongs to a later feature and
 * is deliberately absent, so that calling it says so at the line that called it
 * instead of doing nothing.
 */
[[nodiscard]] const OperationTable& standard_operations();

/**
 * @brief The expression functions D2 implements
 *
 * Arithmetic (min, max, abs, floor, ceil, round, sqrt, pow, clamp), trigonometry
 * in degrees (sin, cos, tan), len and str, and the `geometry` namespace: area,
 * volume, face_count and face_area.
 */
[[nodiscard]] const FunctionTable& standard_functions();

// ============================================================================
// Interpreter
// ============================================================================

/**
 * @brief The evaluator
 *
 * Constructed per generation. The public surface beyond run() exists for
 * operation and function handlers, which need to report, to log and to emit a
 * shape of their own -- `setback` hands back the border it removed.
 *
 * Not copyable, not reusable: run() consumes the state it builds. Call
 * generate() unless you need to hold the interpreter for a handler.
 */
class Interpreter {
public:
    Interpreter(const RuleFile& file,
                const GenerationOptions& options,
                const OperationTable& operations,
                const FunctionTable& functions);
    ~Interpreter();

    Interpreter(const Interpreter&) = delete;
    Interpreter& operator=(const Interpreter&) = delete;

    /**
     * @brief Run the start rule against @p seed
     *
     * @param seed Input shape. Its geometry, scope and attributes are the root's;
     *             its Shape::seed_key is mixed into the root key so that two
     *             different inputs under one run seed diverge.
     * @return Terminals, diagnostics and stats. Never throws for a bad rule file.
     */
    [[nodiscard]] GenerationResult run(const Shape& seed);

    // ---- handler-facing ----

    /// Report a runtime fault at an AST location. Capped; see GenerationResult.
    /// Evaluation CONTINUES: use it for a fault the shape can survive, such as
    /// `scale` skipping an axis the shape has no extent on.
    void report(Severity severity, const SourceLoc& loc, std::string message);

    /**
     * @brief Report a fault and abandon the shape being evaluated
     *
     * For a fault the shape cannot survive -- an extrude of zero, a taper that
     * eats the outline, a face index past the end. The shape produces no
     * terminal and its subtree is not walked; its SIBLINGS still are, because one
     * malformed lot must not cost the other four thousand.
     *
     * A handler cannot simply throw: the unwinding type is private to
     * interpreter.cpp so that it cannot escape generate(), and a handler
     * registered from another translation unit has no way to name it. So a
     * handler marks the shape here and the interpreter throws on its behalf as
     * soon as the handler returns.
     */
    void fail_shape(const SourceLoc& loc, std::string message);

    /// Report at @p loc, but only the first time for that location and message
    void report_once(Severity severity, const SourceLoc& loc, std::string message);

    /// Append a line to GenerationResult::log
    void log(std::string line);

    /**
     * @brief Emit a shape produced by an operation as a terminal of its own
     *
     * `setback` uses it for the border it removed. The child sits at @p parent's
     * depth, gets its own address in the tree, and counts against
     * InterpreterLimits::max_shapes like any other shape.
     *
     * @param parent Shape the operation was running on
     * @param child  Shape to emit. Its rule, id, parent, depth, index and
     *               seed_key are overwritten here; its geometry, scope and
     *               ATTRIBUTES are kept, so a handler that wants the parent's
     *               attributes copies the parent shape and replaces its geometry.
     * @param role   Rule name to record on the child, e.g. "setback.border"
     * @return false when the shape cap refused it
     */
    bool emit_derived_terminal(const Shape& parent, Shape child, std::string role);

    /// The file being run
    [[nodiscard]] const RuleFile& file() const { return file_; }

    /// The options this run was given
    [[nodiscard]] const GenerationOptions& options() const { return options_; }

private:
    struct State;

    const RuleFile& file_;
    GenerationOptions options_;
    const OperationTable& operations_;
    const FunctionTable& functions_;
    State* state_;
};

// ============================================================================
// Entry point
// ============================================================================

/**
 * @brief Run a rule file against one input shape
 *
 * @param file       Parsed file. A file that parsed with errors may be passed;
 *                   the faults it contains are reported again at run time, at
 *                   the line that hits them, and the rest still runs.
 * @param seed       Input shape: a lot boundary, a block face, a test square
 * @param options    Seed, limits and attribute overrides
 * @param operations Operation registry, or nullptr for standard_operations()
 * @param functions  Function registry, or nullptr for standard_functions()
 * @return Terminals, diagnostics and stats. Throws nothing for bad input;
 *         std::bad_alloc can still escape, as everywhere.
 */
[[nodiscard]] GenerationResult generate(const RuleFile& file,
                                        const Shape& seed,
                                        const GenerationOptions& options = {},
                                        const OperationTable* operations = nullptr,
                                        const FunctionTable* functions = nullptr);

} // namespace stratum::procgen::rules
