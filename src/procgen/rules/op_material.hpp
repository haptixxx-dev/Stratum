// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file op_material.hpp
 * @brief `material(slot, variant)` -- give a shape's faces a material to draw with
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ================================================================================
 * WHY THIS WORKS BEFORE D8 DOES
 * ================================================================================
 *
 * D8 (#45) is the texturing feature: `setup_projection`, `project_uv`,
 * `tile_uv` and the rest. It is blocked on something structural -- ShapeGeometry
 * carries no UV set at all, and `shape_to_mesh()` projects coordinates at
 * triangulation time -- so none of those operations has anywhere to store its
 * result.
 *
 * `material` is not blocked on any of that, and it is most of what an author
 * actually wants. Three things already exist and only needed connecting:
 *
 *   1. `Face` already carries a `MaterialKey`. It has since the road system
 *      needed one, and nothing in the rule language ever set it.
 *   2. `shape_to_mesh()` already opens a SubMesh per material key, and merges
 *      runs that share one, so a shape whose faces carry materials comes out as
 *      a mesh the renderer can bind per range.
 *   3. UVs are already a planar projection IN METRES in each face's own basis,
 *      with v up the wall (see shape.cpp's texture_basis). A brick texture
 *      tiling at real-world scale therefore needs no projection call at all --
 *      which is exactly the case `setup_projection` exists to handle when it is
 *      NOT true.
 *
 * So a rule can say `material("wall", 3)` and get a textured building today,
 * using the material library the editor already ships with. What D8 adds on top
 * is control over the projection, not the ability to have one.
 *
 * ================================================================================
 * THE SLOT NAMES
 * ================================================================================
 *
 * The words are `renderer/mesh.hpp`'s MaterialId, lower-cased: default,
 * asphalt, concrete, curb, sidewalk, markings, gravel, dirt, grass, bridgedeck,
 * parapet, wall, roof.
 *
 * Lower case for the same reason osm.type is -- a rule file's vocabulary is
 * lower case throughout, and `material_id_name()` returns "BridgeDeck" for a
 * human to read in a panel. An unknown name is REFUSED and the message lists
 * what was expected, because the alternative is a building that silently draws
 * in the default grey and an author hunting for a texture path that was never
 * wrong.
 *
 * The VARIANT is the second argument and defaults to 0. MaterialLibrary resolves
 * an unknown variant to its slot default rather than failing, which is correct
 * for rendering and invisible to the author -- so the variant is range-checked
 * here instead, against uint16_t, and a negative one is refused.
 *
 * ================================================================================
 * WHAT IT APPLIES TO
 * ================================================================================
 *
 * EVERY face of the current shape, not a selection. A rule that wants one face
 * treated differently selects it first -- `select face { top : { Roof(); } }` --
 * which is the same composition every other operation here uses, and keeps this
 * one from growing a selector argument that duplicates `select`.
 *
 * The material is part of the shape, so it is INHERITED by children the way the
 * geometry is: `material("wall"); split(x) { ... }` gives every slab the wall
 * material without repeating it. A child that wants its own says so.
 *
 * ================================================================================
 * THE ORDER MATTERS AGAINST THE FACADE OPERATIONS
 * ================================================================================
 *
 * `wall_panel()`, `window()` and `door()` tag their own output through
 * op_facade.cpp's facade_material(), which gives every part its own variant of
 * the Wall slot -- panel 0, reveal 1, frame 2, glass 3, sill 4, leaf 5,
 * threshold 6. That is a good default: a facade is textured without the author
 * saying anything, and the glass is already separable from the brick.
 *
 * It also means a `material()` call BEFORE one of them is thrown away:
 *
 *     rule R { material("roof", 1); wall_panel(0.0, 0.2); }   // Wall/0
 *     rule R { wall_panel(0.0, 0.2); material("roof", 1); }   // Roof/1
 *
 * Put it after. `extrude`, `split`, `roof` and the rest leave the material
 * alone, so only the facade family has this ordering.
 *
 * ================================================================================
 * NOTHING IS TEXTURED IN ShaderMode::Simple
 * ================================================================================
 *
 * Worth knowing before hunting for a missing texture. GPURenderer binds
 * materials only in ShaderMode::PBR -- the simple shader has no samplers and no
 * material uniform block, so every range draws with the bound pipeline and the
 * whole city comes out flat grey however carefully its materials were assigned.
 * Render Settings, Shader Mode, PBR.
 */

#pragma once

#include "procgen/rules/interpreter.hpp"

namespace stratum::procgen::rules {

/**
 * @brief Register `material` into @p table
 */
void register_material_operations(OperationTable& table);

/**
 * @brief standard_operations() plus register_material_operations()
 *
 * For a caller that wants this family alone. The union of every family is
 * procgen/rules/registry.hpp's full_operations().
 */
[[nodiscard]] const OperationTable& material_operations();

/**
 * @brief Parse a material slot name; false when @p text names none
 *
 * Exposed for the tests and for any caller that needs the same vocabulary.
 */
[[nodiscard]] bool parse_material_slot(const std::string& text, MaterialId& out);

} // namespace stratum::procgen::rules
