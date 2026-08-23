/**
 * @file obj_dump.cpp
 * @brief Wavefront OBJ writer for test meshes
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 */

#include "obj_dump.hpp"

#include <glm/glm.hpp>

#include <cstdint>
#include <fstream>
#include <ios>
#include <sstream>
#include <system_error>

namespace stratum::test {

namespace {

/**
 * @brief A flat diffuse colour per material slot, for the companion .mtl
 *
 * Chosen so the slots are told apart at a glance rather than to look like the
 * real surface: curb is red and sidewalk is blue because those two are what a
 * reader is checking for on a residential fixture.
 *
 * @param material Slot to colour
 * @return Linear RGB in 0..1
 */
[[nodiscard]] glm::vec3 material_colour(MaterialId material) {
    switch (material) {
        case MaterialId::Default:    return {1.00f, 0.00f, 1.00f};   // magenta: untagged
        case MaterialId::Asphalt:    return {0.25f, 0.25f, 0.27f};
        case MaterialId::Concrete:   return {0.70f, 0.70f, 0.68f};
        case MaterialId::Curb:       return {0.85f, 0.20f, 0.15f};
        case MaterialId::Sidewalk:   return {0.20f, 0.45f, 0.85f};
        case MaterialId::Markings:   return {0.95f, 0.95f, 0.90f};
        case MaterialId::Gravel:     return {0.55f, 0.50f, 0.42f};
        case MaterialId::Dirt:       return {0.45f, 0.35f, 0.25f};
        case MaterialId::Grass:      return {0.30f, 0.55f, 0.25f};
        case MaterialId::BridgeDeck: return {0.60f, 0.60f, 0.65f};
        case MaterialId::Parapet:    return {0.80f, 0.75f, 0.55f};
        case MaterialId::Count:      break;
    }
    return {1.0f, 1.0f, 1.0f};
}

/**
 * @brief Write the companion material library
 *
 * @param path Destination .mtl path
 * @return True when the file was written
 */
bool write_mtl(const std::filesystem::path& path) {
    std::ofstream out(path, std::ios::trunc);
    if (!out) return false;

    out << "# Stratum road material slots, one per MaterialId\n";
    for (size_t i = 0; i < static_cast<size_t>(MaterialId::Count); ++i) {
        const auto material = static_cast<MaterialId>(i);
        const glm::vec3 c = material_colour(material);
        out << "\nnewmtl " << material_id_name(material) << '\n'
            << "Kd " << c.r << ' ' << c.g << ' ' << c.b << '\n'
            << "Ka 0 0 0\n"
            << "Ks 0 0 0\n"
            << "d 1\n"
            << "illum 1\n";
    }
    return static_cast<bool>(out);
}

} // namespace

bool write_obj(const std::vector<const Mesh*>& meshes, const std::filesystem::path& path,
               ObjDumpStats* stats, std::string* error) {
    ObjDumpStats counts;

    std::error_code ec;
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path(), ec);
        if (ec) {
            if (error) *error = "create_directories: " + ec.message();
            return false;
        }
    }

    std::filesystem::path mtl_path = path;
    mtl_path.replace_extension(".mtl");
    if (!write_mtl(mtl_path)) {
        if (error) *error = "could not write " + mtl_path.string();
        return false;
    }

    std::ofstream out(path, std::ios::trunc);
    if (!out) {
        if (error) *error = "could not open " + path.string();
        return false;
    }
    out.precision(6);
    out << std::fixed;

    out << "# Stratum road network dump\n"
        << "# Y up, right-handed, metres. One group per MaterialId.\n"
        << "mtllib " << mtl_path.filename().string() << "\n\n";

    // --- Vertices -----------------------------------------------------------
    // All three attribute arrays are written mesh by mesh in lockstep, so vertex
    // n of mesh m is at the same 1-based index in v, vt, and vn. The per-mesh
    // base offset below is what lets the faces of every mesh be emitted later,
    // out of mesh order, grouped by material instead.
    std::vector<uint32_t> base(meshes.size(), 0);
    uint32_t running = 0;
    for (size_t m = 0; m < meshes.size(); ++m) {
        base[m] = running;
        if (meshes[m] == nullptr) continue;
        running += static_cast<uint32_t>(meshes[m]->vertices.size());
    }

    for (const Mesh* mesh : meshes) {
        if (mesh == nullptr) continue;
        for (const Vertex& v : mesh->vertices) {
            out << "v " << v.position.x << ' ' << v.position.y << ' ' << v.position.z << '\n';
        }
    }
    for (const Mesh* mesh : meshes) {
        if (mesh == nullptr) continue;
        for (const Vertex& v : mesh->vertices) {
            out << "vt " << v.uv.x << ' ' << v.uv.y << '\n';
        }
    }
    for (const Mesh* mesh : meshes) {
        if (mesh == nullptr) continue;
        for (const Vertex& v : mesh->vertices) {
            out << "vn " << v.normal.x << ' ' << v.normal.y << ' ' << v.normal.z << '\n';
        }
    }
    counts.vertices = running;

    // --- Faces, grouped by material ----------------------------------------
    for (size_t slot = 0; slot < static_cast<size_t>(MaterialId::Count); ++slot) {
        const auto material = static_cast<MaterialId>(slot);

        std::ostringstream faces;
        size_t emitted = 0;
        double area = 0.0;

        for (size_t m = 0; m < meshes.size(); ++m) {
            const Mesh* mesh = meshes[m];
            if (mesh == nullptr) continue;

            const uint32_t vertex_count = static_cast<uint32_t>(mesh->vertices.size());
            for (const SubMesh& sub : mesh->effective_submeshes()) {
                if (sub.material != material) continue;

                const size_t end = static_cast<size_t>(sub.index_offset) + sub.index_count;
                if (end > mesh->indices.size()) continue;

                for (size_t i = sub.index_offset; i + 2 < end; i += 3) {
                    const uint32_t a = mesh->indices[i];
                    const uint32_t b = mesh->indices[i + 1];
                    const uint32_t c = mesh->indices[i + 2];
                    if (a >= vertex_count || b >= vertex_count || c >= vertex_count) continue;
                    if (a == b || b == c || a == c) continue;

                    // OBJ indices are 1-based, and v/vt/vn share one numbering
                    // here because the three arrays were written in lockstep.
                    const uint32_t ia = base[m] + a + 1;
                    const uint32_t ib = base[m] + b + 1;
                    const uint32_t ic = base[m] + c + 1;
                    faces << "f " << ia << '/' << ia << '/' << ia << ' '
                          << ib << '/' << ib << '/' << ib << ' '
                          << ic << '/' << ic << '/' << ic << '\n';
                    ++emitted;

                    const glm::vec3& pa = mesh->vertices[a].position;
                    const glm::vec3& pb = mesh->vertices[b].position;
                    const glm::vec3& pc = mesh->vertices[c].position;
                    area += 0.5 * static_cast<double>(glm::length(glm::cross(pb - pa, pc - pa)));
                }
            }
        }

        if (emitted == 0) continue;

        out << "\ng " << material_id_name(material) << '\n'
            << "usemtl " << material_id_name(material) << '\n'
            << faces.str();

        counts.triangles_by_material[slot] = emitted;
        counts.area_by_material[slot] = area;
        counts.triangles += emitted;
    }

    out.flush();
    if (!out) {
        if (error) *error = "write failed for " + path.string();
        return false;
    }

    if (stats) *stats = counts;
    return true;
}

bool write_obj(const Mesh& mesh, const std::filesystem::path& path, ObjDumpStats* stats,
               std::string* error) {
    return write_obj(std::vector<const Mesh*>{&mesh}, path, stats, error);
}

} // namespace stratum::test
