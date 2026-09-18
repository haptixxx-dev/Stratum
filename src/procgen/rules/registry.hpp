// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file registry.hpp
 * @brief Every operation and function the build has, in one table
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * `standard_operations()` is the D2 set and nothing else, deliberately: a
 * feature that added itself there would mean every feature editing
 * interpreter.cpp, which is the thing interpreter.hpp's registry design exists
 * to prevent. Each feature file registers its own instead --
 * `register_control_operations()`, `register_component_functions()`, and so on.
 *
 * The consequence is that no single caller has the whole language unless
 * somebody assembles it. Three callers need exactly that:
 *
 *   - the rule editor panel, where reporting `set` as "not implemented in this
 *     build" in an editor where it demonstrably is would be a lie;
 *   - the Python bindings, which expose one `generate`;
 *   - any headless generation path, which has no panel to inherit a table from.
 *
 * This file is that assembly, and it is the ONE file a new operation family
 * edits besides its own. One line per feature. Keeping it separate from
 * interpreter.cpp is what stops the dependency running the wrong way: the
 * evaluator still knows nothing about roofs or facades.
 *
 * Both tables are built once, on first use, and returned by reference.
 */

#pragma once

#include "procgen/rules/interpreter.hpp"

namespace stratum::procgen::rules {

/**
 * @brief Every registered operation in this build
 *
 * `standard_operations()` plus every feature family's own registrations.
 * Suitable to pass straight to generate().
 */
[[nodiscard]] const OperationTable& full_operations();

/**
 * @brief Every registered expression function in this build
 */
[[nodiscard]] const FunctionTable& full_functions();

} // namespace stratum::procgen::rules
