#pragma once

#include <vector>
#include <limits>
#include <glm/glm.hpp>

namespace stratum {

struct Vertex {
    glm::vec3 position;
    glm::vec3 normal;
    glm::vec2 uv;
    glm::vec4 color{1.0f};

    bool operator==(const Vertex& other) const {
        return position == other.position && 
               normal == other.normal && 
               uv == other.uv && 
               color == other.color;
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

    // Future: GPU buffer handles
};

} // namespace stratum
