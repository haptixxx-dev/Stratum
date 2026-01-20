#pragma once

#include <vector>
#include <limits>
#include <glm/glm.hpp>

namespace stratum {

/**
 * @brief Vertex structure matching the optimized shader layout
 * 
 * Layout matches mesh.vert:
 * - location 0: position (vec3)
 * - location 1: normal (vec3)
 * - location 2: uv (vec2)
 * - location 3: color (vec4)
 * - location 4: tangent (vec4) - xyz = tangent, w = bitangent sign
 */
struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec2 uv;
    glm::vec4 color{1.0f};
    glm::vec4 tangent{1.0f, 0.0f, 0.0f, 1.0f};  // xyz = tangent direction, w = bitangent sign

    bool operator==(const Vertex& other) const {
        return position == other.position && 
               normal == other.normal && 
               uv == other.uv && 
               color == other.color &&
               tangent == other.tangent;
    }
};

struct BoundingBox3D {
    glm::vec3 min{std::numeric_limits<float>::max()};
    glm::vec3 max{std::numeric_limits<float>::lowest()};

    void expand(const glm::vec3& point) {
        min = glm::min(min, point);
        max = glm::max(max, point);
    }

    glm::vec3 center() const { return (min + max) * 0.5f; }
    glm::vec3 extents() const { return (max - min) * 0.5f; }
    float radius() const { return glm::length(extents()); }

    bool is_valid() const { return min.x <= max.x; }
};

class Mesh {
public:
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    BoundingBox3D bounds;

    void clear() {
        vertices.clear();
        indices.clear();
        bounds = BoundingBox3D{};
    }

    bool is_valid() const {
        return !vertices.empty();
    }

    void compute_bounds() {
        bounds = BoundingBox3D{};
        for (const auto& v : vertices) {
            bounds.expand(v.position);
        }
    }

    /**
     * @brief Compute tangents for normal mapping using MikkTSpace algorithm (simplified)
     * 
     * For each triangle, computes tangent vectors based on UV gradients.
     * Assumes the mesh has valid UVs.
     */
    void compute_tangents() {
        if (indices.empty() || vertices.empty()) return;

        // Reset tangents
        for (auto& v : vertices) {
            v.tangent = glm::vec4(0.0f);
        }

        // Accumulate tangents per-triangle
        for (size_t i = 0; i < indices.size(); i += 3) {
            uint32_t i0 = indices[i];
            uint32_t i1 = indices[i + 1];
            uint32_t i2 = indices[i + 2];

            const glm::vec3& p0 = vertices[i0].position;
            const glm::vec3& p1 = vertices[i1].position;
            const glm::vec3& p2 = vertices[i2].position;

            const glm::vec2& uv0 = vertices[i0].uv;
            const glm::vec2& uv1 = vertices[i1].uv;
            const glm::vec2& uv2 = vertices[i2].uv;

            glm::vec3 edge1 = p1 - p0;
            glm::vec3 edge2 = p2 - p0;

            glm::vec2 duv1 = uv1 - uv0;
            glm::vec2 duv2 = uv2 - uv0;

            float f = 1.0f / (duv1.x * duv2.y - duv2.x * duv1.y + 1e-8f);

            glm::vec3 tangent;
            tangent.x = f * (duv2.y * edge1.x - duv1.y * edge2.x);
            tangent.y = f * (duv2.y * edge1.y - duv1.y * edge2.y);
            tangent.z = f * (duv2.y * edge1.z - duv1.y * edge2.z);

            glm::vec3 bitangent;
            bitangent.x = f * (-duv2.x * edge1.x + duv1.x * edge2.x);
            bitangent.y = f * (-duv2.x * edge1.y + duv1.x * edge2.y);
            bitangent.z = f * (-duv2.x * edge1.z + duv1.x * edge2.z);

            // Accumulate
            vertices[i0].tangent += glm::vec4(tangent, 0.0f);
            vertices[i1].tangent += glm::vec4(tangent, 0.0f);
            vertices[i2].tangent += glm::vec4(tangent, 0.0f);

            // Store bitangent sign (handedness)
            float sign = (glm::dot(glm::cross(vertices[i0].normal, tangent), bitangent) < 0.0f) ? -1.0f : 1.0f;
            vertices[i0].tangent.w = sign;
            vertices[i1].tangent.w = sign;
            vertices[i2].tangent.w = sign;
        }

        // Normalize tangents (Gram-Schmidt orthogonalize)
        for (auto& v : vertices) {
            glm::vec3 t = glm::vec3(v.tangent);
            glm::vec3 n = v.normal;
            
            // Orthogonalize
            t = glm::normalize(t - n * glm::dot(n, t));
            v.tangent = glm::vec4(t, v.tangent.w);
        }
    }

    // Future: GPU buffer handles
};

} // namespace stratum
