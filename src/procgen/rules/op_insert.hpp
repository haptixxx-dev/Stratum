// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file op_insert.hpp
 * @brief D7: `insert`, the operation that puts an asset in the scope
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ================================================================================
 * WHAT THIS IS
 * ================================================================================
 *
 * One catalogue row, `{"insert", 1, 1, "Insert an asset into the scope"}`, so
 * the call is `insert("props/bench.glb")` and there is no second argument. That
 * is CityEngine's `i()`, and the missing second argument is the whole trick: the
 * transform is not written at the call site, it is the SHAPE'S SCOPE. A rule
 * splits a facade into bays, each bay is a shape with its own scope, and
 * `insert` drops an asset aligned to whichever bay it is running on.
 *
 * @code
 *     rule Bay    { split(x) { 0.4 : { Pier(); } 0.6 : { Window(); } ~1.0 : { Pier(); } } }
 *     rule Window { insert("props/window.glb"); }
 * @endcode
 *
 * Twelve bays, twelve placements, no coordinate written anywhere.
 *
 * What a placement IS -- the pivot, the handedness, the unit-cube contract with
 * the asset -- is instances.hpp's subject and is not repeated here. This file is
 * about the three questions the OPERATION has to answer: where the data goes,
 * what happens to the shape, and what counts as an error.
 *
 * ================================================================================
 * 1. HOW THE PLACEMENTS REACH THE CALLER
 * ================================================================================
 *
 * `GenerationResult` carries `terminals`, and a consumer that can get the mesh
 * but not the instances has half a building. There is no `instances` field on
 * `GenerationResult` and this feature does not add one, because a new field is
 * an edit to interpreter.hpp that every operation family would then feel
 * entitled to make -- which is the coupling interpreter.hpp's registry design
 * exists to prevent.
 *
 * So a placement is a TERMINAL. `insert` builds a shape that carries the scope
 * and the asset path, no geometry at all, and hands it to
 * `Interpreter::emit_derived_terminal()` -- the same facility `setback` uses for
 * the border it removes, which exists precisely so an operation can emit output
 * of its own. `collect_instances()` then walks the terminals and groups them.
 *
 * That buys four things, each of which would have to be built by hand otherwise:
 *
 *   - **It is not merged, and cannot become merged.** The placement shape has no
 *     geometry, so `GenerationResult::build_mesh()` appends nothing for it. The
 *     product claim in instances.hpp -- one mesh, ten thousand transforms -- is
 *     not a promise this file makes, it is a consequence of there being no
 *     triangles to merge. The test asserts the triangle count is unchanged by
 *     adding `insert` to a rule file.
 *   - **Determinism for free.** Terminals are in evaluation order, so placements
 *     are too, and `emit_derived_terminal()` gives each one an address in the
 *     shape tree and therefore its own seed.
 *   - **It counts.** A placement is a shape, so it counts against
 *     `InterpreterLimits::max_shapes` like any other. A rule that inserts a
 *     million benches is stopped by the same cap that stops one that splits a
 *     million slabs, and it is reported the same way.
 *   - **Every existing path already carries it.** The editor's shape tree, the
 *     dump, anything that iterates terminals: a placement is visible in all of
 *     them without a line of new plumbing, and `dump_shape()` already prints the
 *     scope and the attributes, so a determinism test over `GenerationResult`
 *     sees the transform.
 *
 * ### How a placement is recognised
 *
 * Two marks, both required, and the pair matters:
 *
 *   - `Shape::rule` is kInsertRole, which `emit_derived_terminal()` sets;
 *   - the attribute kInsertAssetAttribute holds the path as a string.
 *
 * Either alone would be a false positive waiting to happen. An author may write
 * a rule named `insert`; an author may one day `set("insert.asset", ...)`, and
 * because attributes INHERIT that single mistake would turn every descendant
 * into a phantom placement. Requiring both means a collision needs a rule named
 * `insert` that also carries that attribute, which is not an accident.
 *
 * The `insert.` attribute prefix is reserved for this file.
 *
 * ================================================================================
 * 2. WHAT HAPPENS TO THE SHAPE THAT CALLED `insert`
 * ================================================================================
 *
 * Nothing. Its geometry, its scope and its attributes are exactly as they were,
 * and it goes on to be a terminal or a parent on its own merits.
 *
 * This is the one place where the operation deliberately departs from CGA, and
 * the reason is that CGA's departure is a consequence of merging. `i()` REPLACES
 * the shape's geometry with the asset's, because after the merge the asset IS
 * the shape. Here there is nothing to replace it with -- core never loads the
 * asset -- so replacing would mean deleting the shape's geometry, and a rule
 * that inserts a window frame into a wall tile would lose the wall.
 *
 * So the answer to "is an inserted shape still a terminal" has two halves, and
 * the test asserts both:
 *
 *   - The shape that called `insert` is a terminal if and only if it made no
 *     children, exactly as it would be without the call. `rule Window {
 *     insert("w.glb"); }` yields the Window shape, with its wall quad, AND the
 *     placement beside it: two terminals, one of which meshes.
 *   - The PLACEMENT is always a terminal, and always one with no geometry. It
 *     produced no children, and it produced no geometry of its own either, which
 *     is what makes it an instance rather than a mesh.
 *
 * A rule wanting the wall gone deletes it the ordinary way, and a rule calling
 * `insert` twice gets two placements from the one shape -- they are separate
 * derived children with separate addresses, so their seeds differ.
 *
 * ================================================================================
 * 3. WHAT IS AN ERROR, AND WHAT IS MERELY WORTH SAYING
 * ================================================================================
 *
 * ### An empty path FAILS
 *
 * `insert("")`, or a path of nothing but spaces. There is no asset there to
 * place, and an empty bucket in the InstanceSet is a placement a consumer cannot
 * resolve, cannot report and cannot draw. `fail_shape()` abandons the subtree
 * and names the line, and the other four thousand lots still generate.
 *
 * Whether the path names a file that EXISTS is not checked, here or anywhere in
 * `stratum_core`. That is F1's, at import time, against the whole set; see
 * instances.hpp on the layer boundary.
 *
 * ### A flat scope is SILENT
 *
 * A facade tile has `size.z == 0`. Inserting a window frame into it is the case
 * this feature was built for, so it says nothing. The consumer sees
 * `flat_axes() == 1` and fits the asset flat, or uses the origin and the axes
 * and the asset's own depth.
 *
 * ### A scope with NO size at all is a WARNING, reported once
 *
 * `size == (0,0,0)`: the scope is a point, so the placement carries an
 * orientation and nothing else, and a consumer fitting the asset to the box gets
 * a bench of zero metres. Almost always this is a rule that built a geometry-less
 * shape and forgot the `scale` that gives it a size -- shape.hpp reserves the
 * geometry-less shape for exactly this, and `scale` is the only operation that
 * can set its scope size.
 *
 * It is a warning and not a failure, because "place it at this point at its
 * native size" is a legitimate thing to want from a prop scatter, and refusing
 * would make that impossible to express. It is reported ONCE per call site, so a
 * rule run over four thousand lots produces one message rather than four
 * thousand. The placement is recorded either way: dropping it would lose data
 * silently, which is the worse of the two failures.
 *
 * ### A negative or non-finite scope FAILS
 *
 * `Scope::size` is documented as never negative and `scale` refuses to make it
 * so, so neither should be reachable from a rule file. They are checked anyway
 * because a placement is DATA that outlives the generation: a NaN origin written
 * into an instance buffer takes out a whole draw call, and the crash is a long
 * way from the rule that caused it. Cheap to check, and the check has a test
 * against a hand-built shape.
 */

#pragma once

#include "procgen/instances.hpp"
#include "procgen/rules/interpreter.hpp"
#include "procgen/rules/shape.hpp"

#include <string>
#include <string_view>
#include <vector>

namespace stratum::procgen::rules {

// ============================================================================
// The marks that make a terminal a placement
// ============================================================================

/**
 * @brief `Shape::rule` on a placement terminal
 *
 * Lowercase, matching "setback.border", so that it does not read as a rule the
 * author wrote. See the header on why both marks are required.
 */
inline constexpr std::string_view kInsertRole = "insert";

/// The attribute holding the asset path, as a string value
inline constexpr std::string_view kInsertAssetAttribute = "insert.asset";

// ============================================================================
// Scope to transform
// ============================================================================

/**
 * @brief Copy a scope into a placement transform
 *
 * The whole of D7's geometry, and it is a copy: `Scope` already holds an origin,
 * three orthonormal axes and a size, and instances.hpp defines a placement to be
 * those three. `Scope::pivot` is deliberately dropped -- see instances.hpp.
 */
[[nodiscard]] InstanceTransform instance_transform(const Scope& scope);

// ============================================================================
// Building a placement
// ============================================================================

/// What insert_placement() wants to say beyond success or failure
struct InsertReport {
    /// Messages for the caller to raise as warnings at the call site
    std::vector<std::string> warnings;

    /// True when the scope had no size on any axis; see the header
    bool point_scope = false;
};

/**
 * @brief Build the placement shape for `insert(asset)` on @p shape
 *
 * Validates the path and the scope, then produces the shape that
 * `Interpreter::emit_derived_terminal()` turns into a terminal. It does not
 * touch @p shape, and it does not emit anything: it is separated from the
 * handler so that a test can assert the transform and every refusal without
 * running an interpreter.
 *
 * The placement inherits @p shape's attributes, so a consumer can read the
 * `floor_count` or the material of the wall a prop was placed on. Its geometry
 * is empty, which is what keeps it out of build_mesh().
 *
 * @param shape     Shape the operation is running on; read only
 * @param asset     Asset path as the author wrote it, not normalised
 * @param placement Receives the placement shape on success; untouched otherwise
 * @param report    Optional; receives warnings and the point-scope flag
 * @return failure when @p asset is blank, or when the scope is negative or
 *         non-finite. A flat or point-sized scope is not a failure.
 */
[[nodiscard]] OpResult insert_placement(const Shape& shape,
                                        std::string_view asset,
                                        Shape& placement,
                                        InsertReport* report = nullptr);

// ============================================================================
// Reading placements back
// ============================================================================

/**
 * @brief Is @p shape a placement terminal, and of what asset?
 *
 * Both marks are required; see the header on why. @p asset_out receives the
 * path when the answer is true and is untouched otherwise.
 */
[[nodiscard]] bool is_placement(const Shape& shape, std::string* asset_out = nullptr);

/**
 * @brief Gather every placement in @p terminals into a set, grouped by asset
 *
 * The bridge between D7 and F2, and the function a consumer calls. Terminals
 * that are not placements are skipped, so it is safe to hand it the whole of
 * `GenerationResult::terminals`.
 *
 * Order is the order of @p terminals, which for a generation is evaluation
 * order, which is source order. Two different rules that insert the same path
 * land in one list in the order the evaluator reached them -- the grouping is by
 * path and neither rule knows about the other.
 */
[[nodiscard]] InstanceSet collect_instances(const std::vector<Shape>& terminals);

/// collect_instances() over a whole result. See the vector overload.
[[nodiscard]] InstanceSet collect_instances(const GenerationResult& result);

// ============================================================================
// Registration
// ============================================================================

/**
 * @brief Add `insert` to @p table
 *
 * The one line registry.cpp needs. This file does not edit registry.cpp and
 * does not include it: the dependency runs one way, from the assembly to the
 * family.
 *
 * ### Wiring this family in takes FOUR edits, not one
 *
 * Registering the handler is the visible half and the easy half to report. The
 * other three are build files, and leaving them out gives an integrator a link
 * error rather than a working build -- the family compiles into nothing, and the
 * suite that proves it never runs. All four, in the order they are needed:
 *
 *   1. `CMakeLists.txt`, the stratum_core source list beside
 *      `src/procgen/rules/op_split.cpp` ... `op_roof.cpp`: add
 *      `src/procgen/instances.cpp` AND `src/procgen/rules/op_insert.cpp`. The
 *      list is explicit, not a glob, so a new .cpp that nobody adds is a file
 *      that nobody compiles.
 *   2. `tests/CMakeLists.txt`, the test source list beside
 *      `procgen/test_op_roof.cpp`: add `procgen/test_instances.cpp`.
 *   3. `src/procgen/rules/registry.cpp`: include this header and call
 *      register_insert_operations() from full_operations(), which is what the
 *      rule editor panel and the Python bindings run.
 *   4. `tests/procgen/test_registry.cpp`: remove `"insert"` from
 *      kUnimplemented. That list is the catalogue rows that are DELIBERATELY
 *      without a handler, and it is checked in both directions -- so leaving
 *      "insert" in it turns step 3 into a test failure.
 */
void register_insert_operations(OperationTable& table);

/**
 * @brief standard_operations() plus register_insert_operations(), built once
 *
 * For a caller that wants D2 and this family and nothing else -- which is what
 * the tests want, so that they pass before registry.cpp has been touched and so
 * that they cannot pass by some other family having registered the name.
 */
[[nodiscard]] const OperationTable& insert_operations();

} // namespace stratum::procgen::rules
