// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file test_op_material.cpp
 * @brief `material(slot, variant)` reaches the mesh as a submesh the renderer can bind
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The weak version of every test here is "the face's MaterialKey was set".
 * That passes for an operation whose result never leaves ShapeGeometry, and
 * the whole point is that it reaches a SubMesh -- because a submesh is what
 * the renderer binds a texture for. So the assertions are on the built Mesh
 * wherever they can be.
 */

#include "framework.hpp"

#include "procgen/rules/op_material.hpp"
#include "procgen/rules/parser.hpp"
#include "procgen/rules/registry.hpp"
#include "procgen/rules/shape.hpp"

#include <string>

using namespace stratum::procgen::rules;
using stratum::MaterialId;

namespace {

[[nodiscard]] GenerationResult run(const std::string& source, double w = 10.0, double d = 8.0) {
    const ParseResult parsed = parse(source, "material.srl");
    CHECK_EQ(parsed.render_all(source), std::string{});
    return generate(parsed.file, shape_from_rect(w, d), GenerationOptions{},
                    &full_operations(), &full_functions());
}

[[nodiscard]] bool has_error(const GenerationResult& result, const std::string& needle) {
    for (const Diagnostic& d : result.diagnostics) {
        if (d.severity == Severity::Error && d.message.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

/// Triangles in @p mesh attributed to @p slot
[[nodiscard]] size_t triangles_of(const stratum::Mesh& mesh, MaterialId slot) {
    size_t count = 0;
    for (const stratum::SubMesh& sub : mesh.effective_submeshes()) {
        if (sub.material == slot) count += sub.index_count / 3;
    }
    return count;
}

} // namespace

// ============================================================================
// It reaches the mesh
// ============================================================================

TEST(OpMaterial, a_material_becomes_a_submesh_the_renderer_can_bind) {
    const GenerationResult result = run(
        "@start\n"
        "rule M { extrude(4.0); select face { vertical : { W(); } top : { R(); } } }\n"
        // material AFTER the facade operation. wall_panel() assigns its own
        // key -- facade_material() gives every part a Wall variant -- so a
        // material set before it is overwritten. See the header.
        "rule W { wall_panel(); material(\"wall\", 2); }\n"
        "rule R { wall_panel(0.0, 0.2); material(\"roof\", 1); }\n");
    CHECK_TRUE(result.ok());

    const stratum::Mesh mesh = result.build_mesh();
    // Four walls of two triangles each; the deck is a closed slab of twelve.
    CHECK((triangles_of(mesh, MaterialId::Wall)) >= 8);
    CHECK((triangles_of(mesh, MaterialId::Roof)) >= 8);
    // And nothing was left in the default slot, which is what a building that
    // still draws grey would look like.
    CHECK_EQ(triangles_of(mesh, MaterialId::Default), size_t{0});
}

TEST(OpMaterial, the_variant_survives_onto_the_submesh) {
    // The slot picks the texture set and the variant picks which of them. A
    // variant dropped on the way to the mesh gives every wall the same brick,
    // which is exactly the defect this asserts against.
    const GenerationResult result = run(
        "@start\nrule M { extrude(3.0); material(\"wall\", 5); }\n");
    CHECK_TRUE(result.ok());

    const stratum::Mesh mesh = result.build_mesh();
    bool found = false;
    for (const stratum::SubMesh& sub : mesh.effective_submeshes()) {
        if (sub.material == MaterialId::Wall && sub.variant == 5) found = true;
    }
    CHECK_TRUE(found);
}

TEST(OpMaterial, a_facade_operation_assigns_its_own_material_and_the_later_call_wins) {
    // The trap this test exists for. wall_panel(), window() and door() each
    // tag their output through facade_material(), which gives every part its
    // own Wall variant -- panel 0, reveal 1, frame 2, glass 3, sill 4. That is
    // useful by default and it means a `material()` call BEFORE one of them is
    // silently thrown away.
    const GenerationResult before = run(
        "@start\n"
        "rule M { extrude(3.0); select face { top : { R(); } } }\n"
        "rule R { material(\"roof\", 1); wall_panel(0.0, 0.2); }\n");
    CHECK_TRUE(before.ok());
    const stratum::Mesh lost = before.build_mesh();
    // Wall, from facade_material -- not the Roof that was asked for.
    CHECK_EQ(triangles_of(lost, MaterialId::Roof), size_t{0});
    CHECK((triangles_of(lost, MaterialId::Wall)) > 0);

    const GenerationResult after = run(
        "@start\n"
        "rule M { extrude(3.0); select face { top : { R(); } } }\n"
        "rule R { wall_panel(0.0, 0.2); material(\"roof\", 1); }\n");
    CHECK_TRUE(after.ok());
    const stratum::Mesh kept = after.build_mesh();
    CHECK((triangles_of(kept, MaterialId::Roof)) > 0);
    CHECK_EQ(triangles_of(kept, MaterialId::Wall), size_t{0});
}

TEST(OpMaterial, a_material_is_inherited_by_the_children_of_a_split) {
    // Set once on the wall, and every bay keeps it. Without inheritance a
    // facade rule would have to repeat the material in every leaf, and one
    // forgotten leaf is a grey patch in a brick wall.
    const GenerationResult result = run(
        "@start\n"
        "rule M { extrude(6.0); select face { front : { F(); } } }\n"
        "rule F { split(x) { repeat { 2.0 : { B(); } } } }\n"
        "rule B { wall_panel(); material(\"wall\", 3); }\n");
    CHECK_TRUE(result.ok());
    CHECK((result.terminals.size()) >= 4);

    const stratum::Mesh mesh = result.build_mesh();
    CHECK((triangles_of(mesh, MaterialId::Wall)) >= 8);
    CHECK_EQ(triangles_of(mesh, MaterialId::Default), size_t{0});
}

TEST(OpMaterial, a_child_can_override_what_it_inherited) {
    const GenerationResult result = run(
        "@start\n"
        "rule M { extrude(6.0); select face { front : { F(); } } }\n"
        "rule F { material(\"wall\", 1); split(x) { 3.0 : { B(); } ~1.0 : { G(); } } }\n"
        "rule B { wall_panel(); }\n"
        "rule G { wall_panel(); material(\"concrete\", 0); }\n");
    CHECK_TRUE(result.ok());

    const stratum::Mesh mesh = result.build_mesh();
    CHECK((triangles_of(mesh, MaterialId::Wall)) >= 2);
    CHECK((triangles_of(mesh, MaterialId::Concrete)) >= 2);
}

// ============================================================================
// Refusals
// ============================================================================

TEST(OpMaterial, an_unknown_slot_is_refused_and_the_message_lists_the_slots) {
    // A slot that quietly fell back to Default draws in grey, and the author
    // goes looking for a missing texture rather than a misspelt word.
    const GenerationResult result = run(
        "@start\nrule M { extrude(3.0); material(\"brick\"); }\n");
    CHECK_FALSE(result.ok());
    CHECK_TRUE(has_error(result, "does not know the slot 'brick'"));
    // The remedy has to be in the message, not in a header somewhere.
    CHECK_TRUE(has_error(result, "wall"));
    CHECK_TRUE(has_error(result, "roof"));
}

TEST(OpMaterial, the_slot_names_are_lower_case) {
    // material_id_name() returns "BridgeDeck" for a panel to display. A rule
    // file's vocabulary is lower case throughout, and accepting both would
    // make the language ambiguous about its own convention.
    const GenerationResult upper = run(
        "@start\nrule M { extrude(3.0); material(\"Wall\"); }\n");
    CHECK_FALSE(upper.ok());
    CHECK_TRUE(has_error(upper, "does not know the slot 'Wall'"));

    const GenerationResult lower = run(
        "@start\nrule M { extrude(3.0); material(\"wall\"); }\n");
    CHECK_TRUE(lower.ok());
}

TEST(OpMaterial, a_variant_out_of_range_is_refused) {
    // MaterialLibrary resolves an unknown variant to its slot default rather
    // than failing, so a bad number would never surface at draw time. It is
    // checked here, where the author can still be told.
    const GenerationResult negative = run(
        "@start\nrule M { extrude(3.0); material(\"wall\", -1); }\n");
    CHECK_FALSE(negative.ok());
    CHECK_TRUE(has_error(negative, "between 0 and 65535"));

    const GenerationResult huge = run(
        "@start\nrule M { extrude(3.0); material(\"wall\", 70000); }\n");
    CHECK_FALSE(huge.ok());
}

TEST(OpMaterial, the_variant_defaults_to_zero_when_it_is_not_given) {
    const GenerationResult result = run(
        "@start\nrule M { extrude(3.0); material(\"roof\"); }\n");
    CHECK_TRUE(result.ok());
    const stratum::Mesh mesh = result.build_mesh();
    for (const stratum::SubMesh& sub : mesh.effective_submeshes()) {
        if (sub.material == MaterialId::Roof) {
            CHECK_EQ(sub.variant, uint16_t{0});
        }
    }
}

TEST(OpMaterial, every_slot_name_parses_and_round_trips) {
    // The parser and the refusal message read from one table; this is what
    // says the table covers MaterialId rather than most of it.
    const char* names[] = {"default", "asphalt", "concrete", "curb", "sidewalk",
                           "markings", "gravel", "dirt", "grass", "bridgedeck",
                           "parapet", "wall", "roof"};
    for (size_t i = 0; i < sizeof(names) / sizeof(names[0]); ++i) {
        MaterialId slot = MaterialId::Count;
        CHECK_TRUE(parse_material_slot(names[i], slot));
        CHECK_EQ(static_cast<int>(slot), static_cast<int>(i));
    }
    // And that it is exactly MaterialId's population, so a slot added to the
    // enum without a word here fails rather than silently being unreachable
    // from the rule language.
    CHECK_EQ(sizeof(names) / sizeof(names[0]), static_cast<size_t>(MaterialId::Count));
}
