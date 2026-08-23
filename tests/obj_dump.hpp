/**
 * @file obj_dump.hpp
 * @brief Wavefront OBJ writer for test meshes, one usemtl group per MaterialId
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The road pipeline has no other way to be looked at. OSM import is GUI-only,
 * there is no CLI, and a numeric assertion cannot tell a correct ribbon from one
 * that is inside out, folded at a corner, or missing its curb. This writes what
 * the corridor extruder produced to a file any 3D viewer can open, so a failure
 * that reads as "the triangle count is plausible" can still be caught by eye.
 *
 * The output is deliberately plain: positions, normals, UVs, and one `usemtl`
 * group per material slot, in ascending MaterialId order. Groups are the point of
 * the file. A viewer shows Asphalt, Curb, and Sidewalk as separate objects, so a
 * sidewalk that came out at road level or a curb face that faces inward is
 * visible immediately rather than buried in a single grey shell.
 *
 * A companion .mtl is written beside the .obj with one flat colour per slot,
 * because a viewer that honours `usemtl` shows nothing useful without it.
 *
 * Test-only. This lives in tests/ and is not part of stratum_core.
 */

#pragma once

#include "renderer/mesh.hpp"

#include <array>
#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace stratum::test {

/**
 * @brief Geometry counts of what was written
 *
 * Reported per fixture so a change in the profile builder shows up as a change in
 * the material mix, not only as a total that happens to still be plausible.
 */
struct ObjDumpStats {
    size_t vertices = 0;    ///< Vertices written, summed over every input mesh
    size_t triangles = 0;   ///< Triangles written, summed over every input mesh

    /// Triangles per material slot, indexed by the numeric value of MaterialId
    std::array<size_t, static_cast<size_t>(MaterialId::Count)> triangles_by_material{};

    /**
     * @brief Surface area per material slot, in square metres
     *
     * Triangle counts alone do not say what a fixture looks like: the extruder
     * tessellates every strip the same way, so a 0.15 m curb top costs as many
     * triangles as a 3.5 m lane. Area is the number that says whether the mix is
     * plausible.
     */
    std::array<double, static_cast<size_t>(MaterialId::Count)> area_by_material{};

    /// Triangles attributed to @p material
    [[nodiscard]] size_t count(MaterialId material) const {
        const auto i = static_cast<size_t>(material);
        return (i < triangles_by_material.size()) ? triangles_by_material[i] : 0;
    }

    /// Square metres attributed to @p material
    [[nodiscard]] double area(MaterialId material) const {
        const auto i = static_cast<size_t>(material);
        return (i < area_by_material.size()) ? area_by_material[i] : 0.0;
    }
};

/**
 * @brief Write several meshes to one OBJ file, grouped by material
 *
 * Every mesh's vertices are written in order, so vertex indices are offset per
 * mesh, then faces are emitted material by material across ALL the meshes. One
 * road network therefore produces one group per slot rather than one group per
 * piece, which is what makes the file navigable when a fixture has forty edges.
 *
 * Each mesh's Mesh::effective_submeshes() is used, so a mesh with no explicit
 * submeshes contributes to MaterialId::Default rather than being skipped.
 * Degenerate and out-of-range faces are dropped and not counted.
 *
 * Positions are written unchanged: OBJ and this codebase are both Y-up
 * right-handed, so no axis conversion is applied and the file measures in metres.
 *
 * @param meshes Meshes to write, in order. Null entries are skipped.
 * @param path   Destination .obj path. Parent directories are created.
 * @param stats  Optional; receives the counts of what was written.
 * @param error  Optional; receives a message when the write fails.
 * @return True when the file was written.
 */
bool write_obj(const std::vector<const Mesh*>& meshes, const std::filesystem::path& path,
               ObjDumpStats* stats = nullptr, std::string* error = nullptr);

/**
 * @brief Write one mesh to an OBJ file, grouped by material
 *
 * @param mesh  Mesh to write
 * @param path  Destination .obj path
 * @param stats Optional; receives the counts of what was written
 * @param error Optional; receives a message when the write fails
 * @return True when the file was written
 */
bool write_obj(const Mesh& mesh, const std::filesystem::path& path,
               ObjDumpStats* stats = nullptr, std::string* error = nullptr);

} // namespace stratum::test
