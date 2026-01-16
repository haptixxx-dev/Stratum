#pragma once

#include <vector>
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

class Mesh {
public:
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;

    void clear() {
        vertices.clear();
        indices.clear();
    }

    bool is_valid() const {
        return !vertices.empty();
    }
    
    // Future: GPU buffer handles
};

} // namespace stratum
