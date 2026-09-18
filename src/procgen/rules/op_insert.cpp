// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file op_insert.cpp
 * @brief D7: `insert`
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The argument for every choice made here is in op_insert.hpp. What is in this
 * file is short enough to summarise in a sentence: a placement is the shape's
 * scope copied into an InstanceTransform, wrapped in a geometry-less shape, and
 * emitted as a terminal.
 *
 * There is no geometry code in this file, and that is the point of the feature.
 * The moment `insert` built a triangle it would be a merge, and the transform
 * list -- the thing a game engine wants and the thing CityEngine throws away --
 * would be gone.
 */

#include "procgen/rules/op_insert.hpp"

#include <cmath>
#include <string>
#include <utility>

namespace stratum::procgen::rules {

namespace {

// ============================================================================
// Validation
// ============================================================================

/**
 * @brief Nothing but whitespace, which for a path is the same as nothing at all
 *
 * The empty string is blank too -- the loop runs zero times and returns true --
 * so the caller needs this test and not a separate empty() beside it.
 */
[[nodiscard]] bool is_blank(std::string_view text) {
    for (const char c : text) {
        if (c != ' ' && c != '\t' && c != '\n' && c != '\r' && c != '\f' && c != '\v') {
            return false;
        }
    }
    return true;
}

/**
 * @brief Is every number in the scope a real number?
 *
 * All fifteen: the origin, the nine axis components and the size. A placement
 * outlives the generation -- it is written to a file and uploaded to an instance
 * buffer -- so a NaN that reached one would surface as a missing draw call a very
 * long way from the rule that produced it. See op_insert.hpp.
 */
[[nodiscard]] bool scope_is_finite(const Scope& scope) {
    for (int k = 0; k < 3; ++k) {
        if (!std::isfinite(scope.origin[k]) || !std::isfinite(scope.size[k])) {
            return false;
        }
        for (int c = 0; c < 3; ++c) {
            if (!std::isfinite(scope.axes[k][c])) {
                return false;
            }
        }
    }
    return true;
}

// ============================================================================
// Handler
// ============================================================================

void op_insert(OperationArgs& context) {
    // The catalogue row is {1, 1}, so the parser has already refused a call with
    // no argument or with two. What it cannot check is the TYPE, because an
    // argument is an arbitrary expression: `insert(width)` parses.
    if (context.args.empty() || !context.args[0].is_text()) {
        const char* got = context.args.empty() ? "nothing" : context.args[0].type_name();
        context.interpreter.fail_shape(
            context.loc, "'insert' wants an asset path as a string, not " + std::string{got});
        return;
    }

    Shape placement;
    InsertReport report;
    const OpResult result =
        insert_placement(context.shape, context.args[0].as_text(), placement, &report);
    if (!result.ok) {
        context.interpreter.fail_shape(context.loc, "'insert' " + result.message);
        return;
    }

    for (const std::string& warning : report.warnings) {
        // report_once, not report: a rule run over four thousand lots would
        // otherwise spend the whole diagnostic budget saying the same sentence
        // about the same line, and the budget is what stops one mistake from
        // hiding every other one.
        context.interpreter.report_once(Severity::Warning, context.loc, "'insert': " + warning);
    }

    // The return value is deliberately dropped. False means the shape cap
    // refused the placement, and that cap has ALREADY reported itself as an
    // error -- interpreter.hpp's contract, and it is why a truncated generation
    // has ok() == false. Failing the shape here as well would abandon a subtree
    // over a condition the author has been told about, and would turn a
    // truncated building into a missing one.
    (void)context.interpreter.emit_derived_terminal(context.shape, std::move(placement),
                                                    std::string{kInsertRole});
}

[[nodiscard]] OperationTable build_insert_operations() {
    OperationTable table = standard_operations();
    register_insert_operations(table);
    return table;
}

} // namespace

// ============================================================================
// Scope to transform
// ============================================================================

InstanceTransform instance_transform(const Scope& scope) {
    InstanceTransform transform;
    transform.origin = scope.origin;
    transform.axes = scope.axes;
    transform.size = scope.size;
    return transform;
}

// ============================================================================
// Building a placement
// ============================================================================

OpResult insert_placement(const Shape& shape,
                          std::string_view asset,
                          Shape& placement,
                          InsertReport* report) {
    if (report != nullptr) {
        report->warnings.clear();
        report->point_scope = false;
    }

    if (is_blank(asset)) {
        return OpResult::failure("wants an asset path, not an empty string");
    }

    const Scope& scope = shape.scope;
    if (!scope_is_finite(scope)) {
        return OpResult::failure("cannot place an asset in a scope holding a value that is "
                                 "not a number");
    }
    for (int k = 0; k < 3; ++k) {
        if (scope.size[k] < 0.0) {
            return OpResult::failure("cannot place an asset in a scope of negative size on " +
                                     std::string{scope_axis_name(static_cast<ScopeAxis>(k))});
        }
    }

    const InstanceTransform transform = instance_transform(scope);
    if (transform.is_point() && report != nullptr) {
        report->point_scope = true;
        // Named rather than described, because the fix is one call and the
        // author has to know which one. shape.hpp's exception to the tight-box
        // invariant is exactly this case, and `scale` is the only operation that
        // can set the size of a scope with no geometry in it.
        report->warnings.push_back("the scope has no size, so '" + std::string{asset} +
                                   "' is placed at a point; use 'scale' to give the scope a "
                                   "size before inserting");
    }

    // The parent is copied WHOLE and then emptied, rather than field by field.
    // The copy of the geometry is thrown away one line later, which is a real
    // cost on a large shape, and it is paid on purpose: a field-by-field copy
    // silently drops whatever Shape gains next -- tags, a material, a second UV
    // set are all on the roadmap -- and a placement that quietly lost the
    // material of the wall it sits on is the kind of fault nothing points at.
    //
    // What the copy is FOR is the attributes, so a consumer can read the
    // floor_count of the wall a prop was placed on. emit_derived_terminal()
    // overwrites the rule, id, parent, depth, index and seed_key afterwards,
    // and keeps everything else.
    placement = shape;
    placement.geometry.clear();
    placement.attributes[std::string{kInsertAssetAttribute}] = Value::text(std::string{asset});
    return OpResult::success();
}

// ============================================================================
// Reading placements back
// ============================================================================

bool is_placement(const Shape& shape, std::string* asset_out) {
    if (shape.rule != kInsertRole) {
        return false;
    }

    // A function-local static rather than a std::string built per call:
    // Shape::attributes is a std::map<std::string, Value> without a transparent
    // comparator, so find() needs a std::string, and collect_instances() runs
    // this over every terminal of a city.
    static const std::string key{kInsertAssetAttribute};
    const auto it = shape.attributes.find(key);
    if (it == shape.attributes.end() || !it->second.is_text() || it->second.as_text().empty()) {
        return false;
    }

    if (asset_out != nullptr) {
        *asset_out = it->second.as_text();
    }
    return true;
}

InstanceSet collect_instances(const std::vector<Shape>& terminals) {
    InstanceSet set;
    std::string asset;
    for (const Shape& shape : terminals) {
        if (!is_placement(shape, &asset)) {
            continue;
        }
        // In terminal order, which is evaluation order. Never sorted: see the
        // determinism note in instances.hpp.
        (void)set.add(asset, instance_transform(shape.scope));
    }
    return set;
}

InstanceSet collect_instances(const GenerationResult& result) {
    return collect_instances(result.terminals);
}

// ============================================================================
// Registration
// ============================================================================

void register_insert_operations(OperationTable& table) {
    table.register_operation("insert", op_insert);
}

const OperationTable& insert_operations() {
    static const OperationTable table = build_insert_operations();
    return table;
}

} // namespace stratum::procgen::rules
