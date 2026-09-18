// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "procgen/rules/registry.hpp"

#include "procgen/rules/op_comp.hpp"
#include "procgen/rules/op_control.hpp"
#include "procgen/rules/op_facade.hpp"
#include "procgen/rules/op_insert.hpp"
#include "procgen/rules/op_mass.hpp"
#include "procgen/rules/op_material.hpp"
#include "procgen/rules/op_roof.hpp"

namespace stratum::procgen::rules {

const OperationTable& full_operations() {
    // Function-local, not namespace-scope. It is built by COPYING
    // standard_operations(), which is itself a function-local static in
    // interpreter.cpp; a namespace-scope object here would depend on the
    // initialisation order of another translation unit, and the failure mode is
    // an empty registry in a release build only.
    static const OperationTable table = [] {
        OperationTable t = standard_operations();
        register_control_operations(t);
        register_roof_operations(t);
        register_mass_operations(t);
        register_facade_operations(t);
        register_insert_operations(t);
        register_material_operations(t);
        return t;
    }();
    return table;
}

const FunctionTable& full_functions() {
    static const FunctionTable table = [] {
        FunctionTable t = standard_functions();
        register_control_functions(t);
        register_component_functions(t);
        return t;
    }();
    return table;
}

} // namespace stratum::procgen::rules
