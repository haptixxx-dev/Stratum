// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "procgen/rules/op_material.hpp"

#include <limits>
#include <string>

namespace stratum::procgen::rules {
namespace {

/// Every slot, in MaterialId order, for the refusal message
constexpr const char* kSlotNames[] = {
    "default", "asphalt", "concrete", "curb",  "sidewalk", "markings", "gravel",
    "dirt",    "grass",   "bridgedeck", "parapet", "wall",  "roof",
};

/**
 * @brief One string argument, or a reported failure
 *
 * The interpreter's own arg_text() is private to its translation unit, so each
 * operation family carries its own pair. op_control.cpp's op_text() is the
 * model; the wording matches so two families never phrase the same refusal
 * differently.
 */
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

[[nodiscard]] bool op_number(OperationArgs& context, size_t index, double& out) {
    if (index >= context.args.size() || !context.args[index].is_number()) {
        context.interpreter.fail_shape(
            context.loc, "'" + std::string{context.name} + "' wants a number for argument " +
                             std::to_string(index + 1));
        return false;
    }
    out = context.args[index].as_number();
    return true;
}

[[nodiscard]] std::string slot_list() {
    std::string out;
    for (size_t i = 0; i < sizeof(kSlotNames) / sizeof(kSlotNames[0]); ++i) {
        if (i != 0) out += ", ";
        out += kSlotNames[i];
    }
    return out;
}

void op_material(OperationArgs& context) {
    std::string name;
    if (!op_text(context, 0, name)) {
        return;
    }

    MaterialId slot = MaterialId::Default;
    if (!parse_material_slot(name, slot)) {
        // Named, not shrugged at. A slot that quietly fell back to Default
        // draws in grey, and the author goes looking for a missing texture
        // rather than a misspelt word.
        context.interpreter.fail_shape(
            context.loc, "'material' does not know the slot '" + name +
                             "'. The slots are: " + slot_list());
        return;
    }

    uint16_t variant = 0;
    if (context.args.size() >= 2) {
        double raw = 0.0;
        if (!op_number(context, 1, raw)) {
            return;
        }
        // MaterialLibrary resolves an unknown variant to its slot default
        // rather than failing, which is right for drawing and means a bad
        // number here would never surface. So it is checked at the call site
        // instead, where the author can still be told.
        if (raw < 0.0 || raw > static_cast<double>(std::numeric_limits<uint16_t>::max())) {
            context.interpreter.fail_shape(
                context.loc, "'material' wants a variant between 0 and 65535, not " +
                                 format_number(raw));
            return;
        }
        variant = static_cast<uint16_t>(raw);
    }

    const MaterialKey key{slot, variant};
    for (Face& face : context.shape.geometry.faces) {
        face.material = key;
    }
}

} // namespace

bool parse_material_slot(const std::string& text, MaterialId& out) {
    for (size_t i = 0; i < sizeof(kSlotNames) / sizeof(kSlotNames[0]); ++i) {
        if (text == kSlotNames[i]) {
            out = static_cast<MaterialId>(i);
            return true;
        }
    }
    return false;
}

void register_material_operations(OperationTable& table) {
    table.register_operation("material", op_material);
}

const OperationTable& material_operations() {
    static const OperationTable table = [] {
        OperationTable t = standard_operations();
        register_material_operations(t);
        return t;
    }();
    return table;
}

} // namespace stratum::procgen::rules
