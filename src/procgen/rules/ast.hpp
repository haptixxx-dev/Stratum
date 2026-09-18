// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file ast.hpp
 * @brief The parsed form of a rule file: the contract between the parser and everything downstream
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### This header is a contract, not an implementation detail
 *
 * D1 parses. D2 walks a shape tree, D3 splits, D4 does components and roofs, D5
 * does stochastics, D6 does attributes -- and every one of them reads this file
 * and nothing else of the parser. So the shapes here are chosen for the reader,
 * not for the producer.
 *
 * Three rules follow, and the tests hold them:
 *
 *   - **Every node is a tagged union.** Expr and Stmt each hold a std::variant.
 *     `std::visit` gives a visitor; `expr_kind()` and `stmt_kind()` give a tag to
 *     switch on, and static_asserts below pin the enum to the variant order so
 *     the two cannot drift. A `switch` with no default over ExprKind fails to
 *     compile when a node type is added, which is the point.
 *   - **Every node carries a SourceLoc.** A diagnostic from D4 about a roof angle
 *     has to point at the line that asked for it. Locations that are not attached
 *     at parse time cannot be reconstructed afterwards.
 *   - **No evaluation concerns.** No values, no caches, no resolved symbol
 *     pointers into other files, no seeds. The one resolution the parser DOES do
 *     -- whether a call names a rule or a built-in operation -- is recorded as an
 *     index, because it decides the SHAPE of the tree and a later pass would have
 *     to redo the whole name lookup to learn it.
 *
 * ### Why arenas and indices instead of unique_ptr
 *
 * Expressions and statements live in flat vectors on RuleFile and refer to each
 * other by `uint32_t` index. Pointers would order nodes by whatever the allocator
 * did that day; indices order them by the order the parser created them, which is
 * source order. That is what makes "same text in, identical AST out" a property
 * this file can actually promise, and what makes dump() a canonical form two runs
 * can be compared by. It also makes a RuleFile trivially copyable-by-value with
 * no deep-copy code.
 *
 * ### What the AST deliberately does NOT record
 *
 * Declaration order ACROSS kinds. Imports, attributes, constants and rules are in
 * four vectors, each in source order within itself. Nothing in the language makes
 * a rule mean something different for being written above an attribute, and a
 * single interleaved declaration list would have made every consumer switch on a
 * kind to find the rules.
 */

#pragma once

#include "procgen/rules/lexer.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <type_traits>
#include <variant>
#include <vector>

namespace stratum::procgen::rules {

// ============================================================================
// Node handles
// ============================================================================

/// Index into RuleFile::exprs
using ExprId = uint32_t;

/// Index into RuleFile::stmts
using StmtId = uint32_t;

/// Index into RuleFile::rules
using RuleId = uint32_t;

/**
 * @brief "No node here"
 *
 * Used for an absent `else`, an absent parameter default, and an unresolved
 * call target. Deliberately the maximum value rather than 0: index 0 is a
 * perfectly ordinary node, and a sentinel of 0 turns "the first expression in
 * the file" into "nothing" the first time someone forgets to check.
 */
inline constexpr uint32_t kNoNode = 0xFFFFFFFFu;

// ============================================================================
// Small vocabulary
// ============================================================================

/// The one numeric type, the one boolean type, the one text type
enum class PrimitiveType : uint8_t { Float, Bool, String };

/**
 * @brief A declared type, possibly an array of one
 *
 * There is no integer type. CGA has none either, and for a good reason a shape
 * grammar makes obvious: every quantity in the language is a length, an angle or
 * a weight, and a language with both 3 and 3.0 invites `split(y) { 7/2: Floor(); }`
 * to mean 3 on one implementation and 3.5 on another.
 */
struct TypeRef {
    PrimitiveType type = PrimitiveType::Float;
    bool is_array = false;
    SourceLoc loc{};
};

/// Split axis, in the shape's own scope, not world space
enum class SplitAxis : uint8_t { X, Y, Z };

/**
 * @brief How one split entry's size is measured
 *
 * The distinction is the whole of facade authoring, so it is a first-class enum
 * rather than a flag or a sign convention:
 *
 *   - **Absolute** `3.0` -- three metres, whatever the parent is.
 *   - **Relative** `25%` -- a quarter of the parent extent along the split axis.
 *   - **Floating** `~3.0` -- three metres NOMINAL. Every floating entry in the
 *     split shares out whatever the absolute and relative entries left over, in
 *     proportion to its nominal size. This is what makes a window band fill a
 *     wall of any width without the author knowing the width.
 *   - **FloatingRelative** `~25%` -- as Floating, but the share is expressed as a
 *     fraction rather than a length. Useful when the remainder is what matters
 *     and the absolute size is meaningless.
 *
 * A parser that dropped `~` or `%` would still produce a tree that runs and
 * silently builds the wrong building, which is why the four are separate values
 * and why the test suite constructs all four from the SAME numeric literal.
 */
enum class SizeKind : uint8_t { Absolute, Relative, Floating, FloatingRelative };

/// What `select` decomposes the current shape into
enum class ComponentDomain : uint8_t { Face, Edge, Vertex, Object };

/**
 * @brief Which components a `select` case claims
 *
 * Directions are in the shape's scope. `Side` is the shorthand for the four
 * roughly-vertical faces of a box, `Aslant` for faces that are neither vertical
 * nor horizontal -- a roof pitch. `All` is the catch-all and is the reason there
 * is no `else` keyword in a select.
 */
enum class ComponentSelector : uint8_t {
    All,
    Front,
    Back,
    Left,
    Right,
    Top,
    Bottom,
    Side,
    Vertical,
    Horizontal,
    Aslant
};

/// Unary operators. There are two, and neither is a surprise.
enum class UnaryOp : uint8_t { Negate, Not };

/// Binary operators. No modulo; see the note on TokenKind::Percent.
enum class BinaryOp : uint8_t {
    Add,
    Subtract,
    Multiply,
    Divide,
    Less,
    LessEqual,
    Greater,
    GreaterEqual,
    Equal,
    NotEqual,
    And,
    Or
};

/**
 * @brief A name, optionally qualified by an import alias
 *
 * `Window` has an empty qualifier. `facades.Window` has qualifier `facades`.
 * There is exactly one level: a dot in this language means "from that imported
 * file", never "a field of that value". Member access on a value does not exist,
 * so `geometry.area()` is a qualified call into the built-in `geometry`
 * namespace and reads the same way an import does.
 */
struct QualifiedName {
    std::string qualifier;  ///< Import alias, or empty when unqualified
    std::string name;       ///< The name itself
    SourceLoc loc{};        ///< Location of the first component

    [[nodiscard]] bool qualified() const { return !qualifier.empty(); }

    [[nodiscard]] std::string text() const {
        return qualifier.empty() ? name : qualifier + "." + name;
    }
};

// ============================================================================
// Expressions
// ============================================================================

struct NumberExpr {
    double value = 0.0;
};

struct StringExpr {
    std::string value;
};

struct BoolExpr {
    bool value = false;
};

/// A bare name: an attribute, a constant, a parameter, a local, or a symbol
/// such as `gable` in `roof(gable, 30)`. Which one it is, is D2's question.
struct NameExpr {
    QualifiedName name;
};

struct ArrayExpr {
    std::vector<ExprId> elements;
};

struct IndexExpr {
    ExprId base = kNoNode;
    ExprId index = kNoNode;
};

struct UnaryExpr {
    UnaryOp op = UnaryOp::Negate;
    ExprId operand = kNoNode;
};

struct BinaryExpr {
    BinaryOp op = BinaryOp::Add;
    ExprId lhs = kNoNode;
    ExprId rhs = kNoNode;
};

struct TernaryExpr {
    ExprId condition = kNoNode;
    ExprId then_value = kNoNode;
    ExprId else_value = kNoNode;
};

/**
 * @brief A function call in expression position, e.g. `min(a, b)` or `geometry.area()`
 *
 * Unlike a CallStmt, this is NOT resolved by the parser. A statement callee
 * decides whether a child shape is created, so its kind is part of the tree's
 * shape and has to be settled here. An expression callee only produces a value,
 * and resolving it means knowing the built-in function library, which is D2's
 * and grows with every later feature. Making the parser own that list would make
 * every one of D2 through D11 a change to the parser.
 */
struct CallExpr {
    QualifiedName callee;
    std::vector<ExprId> args;
};

/// The tag for Expr::node. Values match the variant alternative order; see the
/// static_asserts below, which are what keeps that true.
enum class ExprKind : uint8_t {
    Number,
    String,
    Bool,
    Name,
    Array,
    Index,
    Unary,
    Binary,
    Ternary,
    Call
};

using ExprNode = std::variant<NumberExpr,
                              StringExpr,
                              BoolExpr,
                              NameExpr,
                              ArrayExpr,
                              IndexExpr,
                              UnaryExpr,
                              BinaryExpr,
                              TernaryExpr,
                              CallExpr>;

/// One expression node. Children are ExprIds into RuleFile::exprs.
struct Expr {
    SourceLoc loc{};
    ExprNode node;
};

/// The tag of an expression, for a switch. `std::visit(v, e.node)` for a visitor.
[[nodiscard]] inline ExprKind expr_kind(const Expr& expr) {
    return static_cast<ExprKind>(expr.node.index());
}

// These are the whole safety net under expr_kind(). Reorder the variant without
// reordering the enum and the build stops here instead of silently reading a
// BinaryExpr as an ArrayExpr somewhere in D3.
static_assert(std::is_same_v<std::variant_alternative_t<0, ExprNode>, NumberExpr>);
static_assert(std::is_same_v<std::variant_alternative_t<1, ExprNode>, StringExpr>);
static_assert(std::is_same_v<std::variant_alternative_t<2, ExprNode>, BoolExpr>);
static_assert(std::is_same_v<std::variant_alternative_t<3, ExprNode>, NameExpr>);
static_assert(std::is_same_v<std::variant_alternative_t<4, ExprNode>, ArrayExpr>);
static_assert(std::is_same_v<std::variant_alternative_t<5, ExprNode>, IndexExpr>);
static_assert(std::is_same_v<std::variant_alternative_t<6, ExprNode>, UnaryExpr>);
static_assert(std::is_same_v<std::variant_alternative_t<7, ExprNode>, BinaryExpr>);
static_assert(std::is_same_v<std::variant_alternative_t<8, ExprNode>, TernaryExpr>);
static_assert(std::is_same_v<std::variant_alternative_t<9, ExprNode>, CallExpr>);
static_assert(std::variant_size_v<ExprNode> == 10,
              "ExprKind and ExprNode must have the same number of cases");

// ============================================================================
// Statements
// ============================================================================

/// What a CallStmt's callee turned out to name
enum class CallTarget : uint8_t {
    Rule,       ///< A rule declared in this file. CallStmt::rule is its index.
    Operation,  ///< A built-in operation. CallStmt::operation indexes kBuiltinOperations.
    Imported,   ///< A qualified name from an import. Neither index is set; D2 resolves it.
    Unresolved  ///< The name matched nothing. An Error diagnostic was reported.
};

struct BlockStmt {
    std::vector<StmtId> statements;
};

/**
 * @brief `extrude(3);` or `Facade(2.0);` or `facades.Window();`
 *
 * One node covers both because the author writes one thing. The difference --
 * an operation transforms the current shape, a rule creates a child shape and
 * recurses -- is recorded in @p target, resolved by the parser after the whole
 * file is read so that a rule may be called above its declaration.
 */
struct CallStmt {
    QualifiedName callee;
    std::vector<ExprId> args;
    CallTarget target = CallTarget::Unresolved;
    RuleId rule = kNoNode;      ///< Valid when target == Rule
    uint32_t operation = kNoNode;  ///< Valid when target == Operation
};

/// `let width = scope.sx / 4;` -- a name for a value, scoped to the enclosing block
struct LetStmt {
    std::string name;
    bool has_type = false;
    TypeRef type{};
    ExprId value = kNoNode;
};

struct IfStmt {
    ExprId condition = kNoNode;
    StmtId then_branch = kNoNode;  ///< Always a BlockStmt
    StmtId else_branch = kNoNode;  ///< A BlockStmt, an IfStmt for `else if`, or kNoNode
};

/// One arm of a `choose`. @p weight is relative; the interpreter normalises by
/// the sum, so `{1: A; 1: B;}` and `{50: A; 50: B;}` mean the same thing.
struct ChooseCase {
    ExprId weight = kNoNode;
    StmtId body = kNoNode;  ///< Always a BlockStmt
    SourceLoc loc{};
};

/**
 * @brief Weighted stochastic alternatives
 *
 * CGA writes these as percentages that must sum to 100 and then has to define
 * what happens when they do not. Relative weights normalised by their sum have
 * no such corner: any non-negative numbers work, adding an arm does not require
 * editing the others, and there is no error class for "the percentages are wrong".
 */
struct ChooseStmt {
    std::vector<ChooseCase> cases;
};

/// A size as written: `3.0`, `25%`, `~3.0` or `~25%`
struct SizeSpec {
    SizeKind kind = SizeKind::Absolute;
    ExprId value = kNoNode;
    SourceLoc loc{};
};

/**
 * @brief One row of a split
 *
 * Either a sized successor (`3.0 : Floor();`) or a repeat group
 * (`repeat { ~2.5 : Bay(); }`), never both. @p is_repeat says which.
 *
 * A repeat group's entries are consumed as a unit, over and over, until the
 * parent extent runs out. Nesting a repeat inside a repeat is rejected by the
 * parser: the outer repetition count would depend on the inner one, which has no
 * defined answer, and "it silently did something" is worse than a message.
 */
struct SplitEntry {
    bool is_repeat = false;
    SizeSpec size{};                     ///< Valid when !is_repeat
    StmtId body = kNoNode;               ///< BlockStmt, valid when !is_repeat
    std::vector<SplitEntry> children;    ///< Valid when is_repeat; never contains a repeat
    SourceLoc loc{};
};

struct SplitStmt {
    SplitAxis axis = SplitAxis::X;
    std::vector<SplitEntry> entries;
};

struct SelectCase {
    ComponentSelector selector = ComponentSelector::All;
    StmtId body = kNoNode;  ///< Always a BlockStmt
    SourceLoc loc{};
};

/// `select face { front: Facade(); top: Roof(); }`
struct SelectStmt {
    ComponentDomain domain = ComponentDomain::Face;
    std::vector<SelectCase> cases;
};

/**
 * @brief `scope { ... }` -- save the shape's scope, run the body, restore it
 *
 * This replaces CGA's `push()` and `pop()` operations. Paired operations in a
 * flat statement list can be left unbalanced, and an unbalanced pop corrupts
 * every shape generated after it, far from the rule that did it. A block cannot
 * be unbalanced, and the nesting is visible in the indentation.
 */
struct ScopeStmt {
    StmtId body = kNoNode;  ///< Always a BlockStmt
};

/// `discard;` -- this shape produces no geometry and no children. CGA spells it NIL.
struct DiscardStmt {};

/// The tag for Stmt::node. Values match the variant alternative order.
enum class StmtKind : uint8_t {
    Block,
    Call,
    Let,
    If,
    Choose,
    Split,
    Select,
    Scope,
    Discard
};

using StmtNode = std::variant<BlockStmt,
                              CallStmt,
                              LetStmt,
                              IfStmt,
                              ChooseStmt,
                              SplitStmt,
                              SelectStmt,
                              ScopeStmt,
                              DiscardStmt>;

struct Stmt {
    SourceLoc loc{};
    StmtNode node;
};

[[nodiscard]] inline StmtKind stmt_kind(const Stmt& stmt) {
    return static_cast<StmtKind>(stmt.node.index());
}

static_assert(std::is_same_v<std::variant_alternative_t<0, StmtNode>, BlockStmt>);
static_assert(std::is_same_v<std::variant_alternative_t<1, StmtNode>, CallStmt>);
static_assert(std::is_same_v<std::variant_alternative_t<2, StmtNode>, LetStmt>);
static_assert(std::is_same_v<std::variant_alternative_t<3, StmtNode>, IfStmt>);
static_assert(std::is_same_v<std::variant_alternative_t<4, StmtNode>, ChooseStmt>);
static_assert(std::is_same_v<std::variant_alternative_t<5, StmtNode>, SplitStmt>);
static_assert(std::is_same_v<std::variant_alternative_t<6, StmtNode>, SelectStmt>);
static_assert(std::is_same_v<std::variant_alternative_t<7, StmtNode>, ScopeStmt>);
static_assert(std::is_same_v<std::variant_alternative_t<8, StmtNode>, DiscardStmt>);
static_assert(std::variant_size_v<StmtNode> == 9,
              "StmtKind and StmtNode must have the same number of cases");

// ============================================================================
// Declarations
// ============================================================================

/**
 * @brief The annotations a declaration may carry
 *
 * These drive the Inspector, not the geometry. An annotation the parser does not
 * know is an ERROR rather than a pass-through: the whole value of an annotation
 * is that a UI reacts to it, and a silently ignored `@rnage(0, 10)` looks exactly
 * like a working one until someone opens the panel and finds a free-text box.
 */
enum class AnnotationKind : uint8_t {
    Asset,        ///< `@asset("*.obj")` -- the value is a file, with an optional filter
    Color,        ///< `@color` -- the value is a colour, show a swatch
    Description,  ///< `@description("Height in metres")`
    Enum,         ///< `@enum("brick", "glass")` -- restrict to a set
    Group,        ///< `@group("Massing", "Height")` -- nest in the Inspector
    Hidden,       ///< `@hidden` -- computed, do not offer it
    Order,        ///< `@order(2)` -- position within the group
    Range,        ///< `@range(0, 100)` -- a slider
    Start,        ///< `@start` -- on a rule: the entry point. At most one per file.
    Step,         ///< `@step(0.5)` -- slider granularity
    Unit          ///< `@unit("m")`
};

struct Annotation {
    AnnotationKind kind = AnnotationKind::Hidden;
    std::string name;             ///< Spelling as written, without the '@'
    std::vector<ExprId> args;
    SourceLoc loc{};
};

/// `attr height : float = 12.0` -- a knob the Inspector shows and a rule reads
struct AttrDecl {
    std::string name;
    TypeRef type{};
    ExprId default_value = kNoNode;  ///< Never kNoNode in a file that parsed
    std::vector<Annotation> annotations;
    SourceLoc loc{};
};

/// `const floor_height : float = 3.2` -- not shown, not overridable
struct ConstDecl {
    std::string name;
    TypeRef type{};
    ExprId value = kNoNode;
    std::vector<Annotation> annotations;
    SourceLoc loc{};
};

struct Param {
    std::string name;
    TypeRef type{};
    ExprId default_value = kNoNode;  ///< kNoNode when the parameter is required
    SourceLoc loc{};
};

struct RuleDecl {
    std::string name;
    std::vector<Param> params;
    StmtId body = kNoNode;  ///< Always a BlockStmt, possibly empty
    std::vector<Annotation> annotations;
    bool is_start = false;  ///< Carries an @start annotation
    SourceLoc loc{};
};

/**
 * @brief `import facades from "lib/facades.srl"`
 *
 * The alias is mandatory. An anonymous import would merge another file's rule
 * names into this one, and then whether `Window()` means yours or theirs would
 * depend on which file was read first -- a resolution order that changes when a
 * library adds a rule. D1 records the import and resolves nothing across it;
 * loading the file is the job of whatever drives the parser.
 */
struct ImportDecl {
    std::string alias;
    std::string path;
    SourceLoc loc{};
    SourceLoc path_loc{};
};

/**
 * @brief One parsed rule file
 *
 * The four declaration vectors are in source order within themselves. The two
 * arenas are in creation order, which is also source order, and that is what
 * makes dump() canonical.
 */
struct RuleFile {
    std::string version;  ///< From `version "0.1"`, empty when the file omits it
    SourceLoc version_loc{};

    std::vector<ImportDecl> imports;
    std::vector<AttrDecl> attributes;
    std::vector<ConstDecl> constants;
    std::vector<RuleDecl> rules;

    /// Index into @p rules of the `@start` rule, or kNoNode. A library file has none.
    RuleId start_rule = kNoNode;

    std::vector<Expr> exprs;
    std::vector<Stmt> stmts;

    [[nodiscard]] const Expr& expr(ExprId id) const { return exprs[id]; }
    [[nodiscard]] const Stmt& stmt(StmtId id) const { return stmts[id]; }
};

// ============================================================================
// Built-in operations
// ============================================================================

/**
 * @brief One operation the language knows how to name
 *
 * The parser resolves a statement callee against this table, so an unknown
 * operation is caught with a spelling suggestion at parse time rather than
 * turning into a mystery no-op at generation time.
 *
 * Adding an operation is one row here, and nothing else in D1. Implementing it is
 * D2 through D6. A row with no implementation parses and then fails at run time
 * with the operation's own message, which is the right order: the language is
 * declared once and filled in over four features.
 *
 * @p max_args is kVariadic for the handful that take a list.
 */
struct BuiltinOperation {
    std::string_view name;
    uint8_t min_args = 0;
    uint8_t max_args = 0;
    std::string_view summary;
};

/// max_args value meaning "no upper bound"
inline constexpr uint8_t kVariadic = 0xFFu;

/**
 * @brief The operation catalogue, sorted by name
 *
 * Measured against section 8.1 of docs/plans/cityengine_feature_inventory.md,
 * which is a coverage checklist and not a specification. Three places where this
 * list deliberately departs from it:
 *
 *   - CGA's five roof operations (roofGable, roofHip, roofPyramid, roofRidge,
 *     roofShed) are one `roof(kind, ...)`. They share every parameter but the
 *     silhouette, and five names means five places to add an overhang argument.
 *   - CGA's six primitive operations are one `primitive(kind, ...)`, for the
 *     same reason.
 *   - CGA's one-letter `t`, `r` and `s` are `translate`, `rotate` and `scale`.
 *     A rule file is read far more often than it is typed.
 *
 * `push` and `pop` are absent on purpose: `scope { }` replaces them. `split`,
 * `comp`, `p` and `NIL` are absent because they are statements in this language,
 * not operations.
 */
inline constexpr BuiltinOperation kBuiltinOperations[] = {
    {"align_scope", 1, 1, "Align the scope to an axis system"},
    {"center", 1, 1, "Centre the shape on the named axes within its scope"},
    {"cleanup", 0, 1, "Merge duplicate vertices and drop degenerate faces"},
    {"color", 1, 1, "Set the shape colour"},
    {"convexify", 0, 0, "Split concave faces into convex ones"},
    {"courtyard", 1, 3, "Carve a courtyard into the footprint, or keep an existing hole open"},
    {"delete_holes", 0, 0, "Remove interior rings from the faces"},
    {"delete_tags", 0, 0, "Remove every tag from the shape"},
    {"delete_uv", 0, 1, "Discard a texture coordinate set"},
    {"door", 0, 4, "Cut a door into the shape: opening, reveal, frame and threshold"},
    {"extrude", 1, 2, "Extrude the face along its normal by a distance"},
    {"floors", 1, 3, "Stack N floors of a given height, keeping the floor structure"},
    {"footprint", 0, 0, "Replace the shape with its ground-plane outline"},
    {"inner_rect", 0, 0, "Replace the shape with its largest inscribed rectangle"},
    {"insert", 1, 1, "Insert an asset into the scope"},
    {"material", 1, 1, "Assign a named material"},
    {"mirror", 1, 1, "Mirror the geometry about a scope plane"},
    {"mirror_scope", 1, 1, "Mirror the scope without moving the geometry"},
    {"normalize_uv", 0, 1, "Rescale a texture coordinate set into the unit square"},
    {"offset", 1, 2, "Offset the face outline inwards or outwards"},
    {"podium", 2, 4, "Podium and tower: a base of a height, then an inset mass above it"},
    {"primitive", 1, 4, "Replace the shape with a primitive: cube, quad, disk, sphere, cylinder, cone"},
    {"print", 1, kVariadic, "Write values to the generation log"},
    {"project_uv", 1, 2, "Project texture coordinates from the current projection"},
    {"reduce", 0, 1, "Simplify the geometry within a tolerance"},
    {"report", 2, 2, "Accumulate a named number up the shape tree"},
    {"reverse_normals", 0, 0, "Flip every face"},
    {"roof", 1, 4, "Raise a roof: gable, hip, pyramid, ridge, shed or dome"},
    {"rotate", 3, 3, "Rotate the geometry about the scope origin"},
    {"rotate_scope", 3, 3, "Rotate the scope without moving the geometry"},
    {"scale", 3, 3, "Resize the scope, and the geometry with it"},
    {"scale_uv", 2, 2, "Scale texture coordinates"},
    {"scatter", 2, 3, "Place N points over the shape and emit a child at each"},
    {"set", 2, 2, "Set a named shape attribute"},
    {"set_pivot", 1, 1, "Move the scope pivot"},
    {"setback", 1, 2, "Inset the outline, keeping the removed border as a child"},
    {"setup_projection", 2, 4, "Define the texture projection for later project_uv calls"},
    {"soften_normals", 0, 1, "Average normals across edges below an angle"},
    {"tag", 1, 1, "Add a tag to the shape"},
    {"taper", 1, 1, "Extrude with a narrowing top"},
    {"texture", 1, 1, "Assign a texture by path"},
    {"tile_uv", 2, 2, "Tile texture coordinates at a real-world size"},
    {"translate", 3, 3, "Move the geometry within the scope"},
    {"translate_uv", 2, 2, "Offset texture coordinates"},
    {"trim", 0, 1, "Trim the geometry against the scope planes"},
    {"wall_panel", 0, 2, "The wall surface between openings, with its own scope and UVs"},
    {"window", 0, 5, "Cut a window into the shape: opening, reveal, frame and sill"},
};

inline constexpr size_t kBuiltinOperationCount =
    sizeof(kBuiltinOperations) / sizeof(kBuiltinOperations[0]);

constexpr bool builtin_operations_are_sorted() {
    for (size_t i = 1; i < kBuiltinOperationCount; ++i) {
        if (!(kBuiltinOperations[i - 1].name < kBuiltinOperations[i].name)) {
            return false;
        }
    }
    return true;
}

// Sortedness is not needed for the lookup, which is a linear scan. It is needed
// so that the "unknown operation" message can list the catalogue in an order a
// human can search, and so that a reviewer adding a row puts it where the next
// reader will look for it.
static_assert(builtin_operations_are_sorted(),
              "kBuiltinOperations must stay sorted by name");

/**
 * @brief Find a built-in operation by name
 * @return Its index into kBuiltinOperations, or kNoNode
 */
[[nodiscard]] inline uint32_t find_builtin_operation(std::string_view name) {
    for (size_t i = 0; i < kBuiltinOperationCount; ++i) {
        if (kBuiltinOperations[i].name == name) {
            return static_cast<uint32_t>(i);
        }
    }
    return kNoNode;
}

// ============================================================================
// Spellings
// ============================================================================
// Every enum above is written by the author as a word, so every enum needs the
// word back: for error messages, for dump(), and eventually for a formatter.
// These are the single source of truth in both directions.

[[nodiscard]] const char* primitive_type_name(PrimitiveType type);
[[nodiscard]] const char* split_axis_name(SplitAxis axis);
[[nodiscard]] const char* size_kind_name(SizeKind kind);
[[nodiscard]] const char* component_domain_name(ComponentDomain domain);
[[nodiscard]] const char* component_selector_name(ComponentSelector selector);
[[nodiscard]] const char* unary_op_name(UnaryOp op);
[[nodiscard]] const char* binary_op_name(BinaryOp op);
[[nodiscard]] const char* call_target_name(CallTarget target);
[[nodiscard]] const char* annotation_kind_name(AnnotationKind kind);

/**
 * @brief A canonical text form of a parsed file
 *
 * Deterministic: the same source text produces byte-identical output on any
 * platform. Locations are deliberately NOT included, so two files that differ
 * only in whitespace, comments or line breaks dump identically -- which is the
 * property the determinism tests assert, and which would be untestable if the
 * dump carried line numbers.
 *
 * Expressions print on one line, statements over several, so a test can assert
 * on an exact expression tree -- `(binary + (num 1) (binary * (num 2) (num 3)))`
 * -- while a human reading a failure still gets an indented statement structure.
 *
 * Defined in parser.cpp: the AST has no translation unit of its own while D1 is
 * five files. It moves to ast.cpp the moment there is one.
 *
 * @param file Parsed file, including one that parsed with errors
 * @return An S-expression rendering, newline-terminated
 */
[[nodiscard]] std::string dump(const RuleFile& file);

} // namespace stratum::procgen::rules
