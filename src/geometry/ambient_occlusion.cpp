#include "geometry/ambient_occlusion.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <thread>
#include <vector>

namespace stratum::geometry {
namespace {

constexpr float kEpsilon = 1e-6f;

// ---------------------------------------------------------------------------
// Bounding volume hierarchy
// ---------------------------------------------------------------------------
//
// A median-split BVH over triangle centroids, flattened into one array with a
// skip index. Deliberately the simplest structure that turns the bake from
// quadratic into something usable: a city tile is tens of thousands of triangles
// and a vertex casts a dozen rays, so a linear scan is minutes and this is
// milliseconds. A surface-area-heuristic build would traverse maybe 20% fewer
// nodes and is not worth the code here -- the bake is not on any frame's path.

struct Aabb {
    glm::vec3 min{ std::numeric_limits<float>::max() };
    glm::vec3 max{ std::numeric_limits<float>::lowest() };

    void expand(const glm::vec3& p) {
        min = glm::min(min, p);
        max = glm::max(max, p);
    }
    void expand(const Aabb& other) {
        min = glm::min(min, other.min);
        max = glm::max(max, other.max);
    }

    /**
     * @brief Slab test against a ray with precomputed reciprocal direction
     *
     * The reciprocal is passed in rather than computed here because it is
     * constant for a ray and this is called once per node visited.
     *
     * A zero component in the direction gives an infinite reciprocal, and the
     * min/max pair below then resolves to +/-inf rather than NaN for every plane
     * the ray is parallel to -- which is the correct answer, and the reason this
     * uses min/max instead of branching on the sign.
     */
    [[nodiscard]] bool intersects(const glm::vec3& origin, const glm::vec3& inv_dir,
                                  float max_t) const {
        const glm::vec3 t0 = (min - origin) * inv_dir;
        const glm::vec3 t1 = (max - origin) * inv_dir;
        const glm::vec3 near = glm::min(t0, t1);
        const glm::vec3 far = glm::max(t0, t1);
        const float enter = std::max(std::max(near.x, near.y), near.z);
        const float exit = std::min(std::min(far.x, far.y), far.z);
        return exit >= std::max(enter, 0.0f) && enter <= max_t;
    }
};

struct Triangle {
    glm::vec3 a{ 0.0f };
    glm::vec3 edge1{ 0.0f };   // b - a
    glm::vec3 edge2{ 0.0f };   // c - a
};

struct Node {
    Aabb bounds;
    /// Index of the first triangle for a leaf; unused for an interior node.
    uint32_t first = 0;
    /// Triangle count for a leaf, 0 for an interior node.
    uint32_t count = 0;
    /// Interior nodes only: index of the right child. The left child is this
    /// node's index + 1, which is what lets the whole tree live in one array.
    uint32_t right = 0;
};

class Bvh {
public:
    void build(const Mesh& mesh) {
        const size_t triangle_count = mesh.indices.size() / 3;
        if (triangle_count == 0) return;

        m_triangles.reserve(triangle_count);
        m_bounds.reserve(triangle_count);
        std::vector<glm::vec3> centroids;
        centroids.reserve(triangle_count);

        for (size_t t = 0; t < triangle_count; ++t) {
            const uint32_t i0 = mesh.indices[t * 3 + 0];
            const uint32_t i1 = mesh.indices[t * 3 + 1];
            const uint32_t i2 = mesh.indices[t * 3 + 2];
            if (i0 >= mesh.vertices.size() || i1 >= mesh.vertices.size() ||
                i2 >= mesh.vertices.size()) {
                continue;   // a malformed index buffer must not take the bake down
            }

            const glm::vec3 a = mesh.vertices[i0].position;
            const glm::vec3 b = mesh.vertices[i1].position;
            const glm::vec3 c = mesh.vertices[i2].position;

            m_triangles.push_back(Triangle{ a, b - a, c - a });

            Aabb box;
            box.expand(a);
            box.expand(b);
            box.expand(c);
            m_bounds.push_back(box);
            centroids.push_back((a + b + c) / 3.0f);
        }

        m_order.resize(m_triangles.size());
        std::iota(m_order.begin(), m_order.end(), 0u);
        m_nodes.reserve(m_triangles.size() * 2);
        subdivide(centroids, 0, static_cast<uint32_t>(m_order.size()));
    }

    [[nodiscard]] bool empty() const { return m_nodes.empty(); }

    /**
     * @brief Is there any hit in (0, @p max_t] along the ray?
     *
     * ANY hit, not the nearest: occlusion is a yes/no question, so traversal stops
     * at the first triangle found rather than sorting children front to back.
     *
     * @param[out] out_t Distance to the hit that stopped the search. Only written
     *                   when the function returns true, and only used for the
     *                   distance falloff -- it is not necessarily the nearest hit.
     */
    [[nodiscard]] bool occluded(const glm::vec3& origin, const glm::vec3& dir, float max_t,
                                float& out_t) const {
        if (m_nodes.empty()) return false;

        const glm::vec3 inv_dir = 1.0f / dir;

        // An explicit stack, not recursion: the bake calls this millions of times
        // and the tree is deep enough for the call overhead to show.
        uint32_t stack[64];
        int depth = 0;
        stack[depth++] = 0;

        while (depth > 0) {
            const Node& node = m_nodes[stack[--depth]];
            if (!node.bounds.intersects(origin, inv_dir, max_t)) continue;

            if (node.count > 0) {
                for (uint32_t i = 0; i < node.count; ++i) {
                    const Triangle& tri = m_triangles[m_order[node.first + i]];
                    float t = 0.0f;
                    if (intersect(origin, dir, tri, max_t, t)) {
                        out_t = t;
                        return true;
                    }
                }
                continue;
            }

            // Depth is bounded by the median split, which halves the triangle
            // count at every level, so 64 is far more than a real mesh reaches.
            // Dropping a subtree at the limit loses occlusion; it cannot corrupt
            // anything, which is the right failure for a shading term.
            if (depth + 2 <= 64) {
                stack[depth++] = node.right;
                stack[depth++] = left_child_of(node);
            }
        }
        return false;
    }

private:
    static constexpr uint32_t kLeafSize = 8;

    [[nodiscard]] uint32_t left_child_of(const Node& node) const {
        // The left child always sits immediately after its parent, so its index is
        // the parent's index plus one. Recovering it from the address keeps Node at
        // one child index instead of two.
        return static_cast<uint32_t>(&node - m_nodes.data()) + 1u;
    }

    /**
     * @brief Moller-Trumbore, WITHOUT backface culling
     *
     * Culling would be wrong here: a wall occludes whether the ray meets its front
     * or its back, and much of this geometry is single-sided anyway.
     */
    static bool intersect(const glm::vec3& origin, const glm::vec3& dir, const Triangle& tri,
                          float max_t, float& out_t) {
        const glm::vec3 pvec = glm::cross(dir, tri.edge2);
        const float det = glm::dot(tri.edge1, pvec);
        if (std::abs(det) < kEpsilon) return false;   // ray parallel to the triangle

        const float inv_det = 1.0f / det;
        const glm::vec3 tvec = origin - tri.a;
        const float u = glm::dot(tvec, pvec) * inv_det;
        if (u < 0.0f || u > 1.0f) return false;

        const glm::vec3 qvec = glm::cross(tvec, tri.edge1);
        const float v = glm::dot(dir, qvec) * inv_det;
        if (v < 0.0f || u + v > 1.0f) return false;

        const float t = glm::dot(tri.edge2, qvec) * inv_det;
        if (t <= kEpsilon || t > max_t) return false;

        out_t = t;
        return true;
    }

    uint32_t subdivide(std::vector<glm::vec3>& centroids, uint32_t begin, uint32_t end) {
        const uint32_t index = static_cast<uint32_t>(m_nodes.size());
        m_nodes.emplace_back();

        Aabb bounds;
        for (uint32_t i = begin; i < end; ++i) bounds.expand(m_bounds[m_order[i]]);

        const uint32_t count = end - begin;
        if (count <= kLeafSize) {
            Node& leaf = m_nodes[index];
            leaf.bounds = bounds;
            leaf.first = begin;
            leaf.count = count;
            return index;
        }

        // Split on the longest axis of the centroid spread, at the median. The
        // median rather than the midpoint so the tree stays balanced when the
        // geometry is not -- a row of buildings along a street is exactly that.
        Aabb centroid_bounds;
        for (uint32_t i = begin; i < end; ++i) centroid_bounds.expand(centroids[m_order[i]]);
        const glm::vec3 extent = centroid_bounds.max - centroid_bounds.min;
        int axis = 0;
        if (extent.y > extent.x) axis = 1;
        if (extent.z > extent[axis]) axis = 2;

        const uint32_t mid = begin + count / 2;
        std::nth_element(m_order.begin() + begin, m_order.begin() + mid, m_order.begin() + end,
                         [&](uint32_t lhs, uint32_t rhs) {
                             return centroids[lhs][axis] < centroids[rhs][axis];
                         });

        subdivide(centroids, begin, mid);            // the left child, at index + 1
        const uint32_t right = subdivide(centroids, mid, end);

        Node& node = m_nodes[index];
        node.bounds = bounds;
        node.count = 0;
        node.right = right;
        return index;
    }

    std::vector<Triangle> m_triangles;
    std::vector<Aabb> m_bounds;
    std::vector<uint32_t> m_order;
    std::vector<Node> m_nodes;
};

// ---------------------------------------------------------------------------
// Sampling
// ---------------------------------------------------------------------------

/// Radical inverse base 2, the second half of a Hammersley pair.
float radical_inverse(uint32_t bits) {
    bits = (bits << 16u) | (bits >> 16u);
    bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
    bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
    bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
    bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
    return static_cast<float>(bits) * 2.3283064365386963e-10f;
}

/// A cheap integer hash, used to rotate each vertex's sample set independently.
uint32_t hash(uint32_t x) {
    x ^= x >> 16;
    x *= 0x7feb352du;
    x ^= x >> 15;
    x *= 0x846ca68bu;
    x ^= x >> 16;
    return x;
}

/**
 * @brief An orthonormal basis around @p n, without a branch on its dominant axis
 *
 * Duff et al.'s construction. The obvious alternative -- cross with (0,1,0) unless
 * the normal is nearly vertical -- has a discontinuity exactly where this mesh has
 * most of its area, because roads and terrain are horizontal, and a discontinuous
 * basis makes the sample pattern jump between neighbouring vertices.
 */
void build_basis(const glm::vec3& n, glm::vec3& t, glm::vec3& b) {
    const float sign = std::copysign(1.0f, n.z);
    const float a = -1.0f / (sign + n.z);
    const float d = n.x * n.y * a;
    t = glm::vec3(1.0f + sign * n.x * n.x * a, sign * d, -sign * n.x);
    b = glm::vec3(d, sign + n.y * n.y * a, -n.y);
}

} // namespace

void bake_ambient_occlusion(Mesh& mesh, const AOSettings& settings) {
    for (Vertex& vertex : mesh.vertices) {
        vertex.ao = 1.0f;
    }

    const int ray_count = std::max(settings.ray_count, 1);
    const float strength = std::clamp(settings.strength, 0.0f, 1.0f);
    const float max_distance = std::max(settings.max_distance, 1e-3f);
    if (strength <= 0.0f || mesh.vertices.empty()) return;

    Bvh bvh;
    bvh.build(mesh);
    // With no triangles AND no ground plane there is nothing to occlude against,
    // and every vertex keeps the 1 it was just given.
    if (bvh.empty() && !settings.use_ground_plane) return;

    const float min_ao = std::clamp(settings.min_ao, 0.0f, 1.0f);

    const auto bake_range = [&](size_t begin, size_t end) {
        for (size_t v = begin; v < end; ++v) {
            Vertex& vertex = mesh.vertices[v];
            const float normal_length_sq = glm::dot(vertex.normal, vertex.normal);
            if (normal_length_sq < kEpsilon) continue;   // no hemisphere to sample

            const glm::vec3 n = vertex.normal * glm::inversesqrt(normal_length_sq);
            glm::vec3 tangent;
            glm::vec3 bitangent;
            build_basis(n, tangent, bitangent);

            const glm::vec3 origin = vertex.position + n * settings.bias;

            // Rotating each vertex's sample set by its own hash decorrelates
            // neighbouring vertices. Without it, a flat wall bakes with a visible
            // banding pattern, because every vertex on it sampled the same set of
            // directions and hit the same distant occluder or missed it together.
            const float rotation =
                static_cast<float>(hash(static_cast<uint32_t>(v) ^ 0x9e3779b9u)) *
                2.3283064365386963e-10f;

            float occlusion = 0.0f;
            for (int i = 0; i < ray_count; ++i) {
                // COSINE-WEIGHTED hemisphere sampling, via Malley's method: a
                // uniform disc lifted onto the hemisphere. The cosine term of the
                // occlusion integral is then carried by the sample density, so the
                // estimate is a plain average of the samples rather than a
                // weighted one.
                const float u1 = std::fmod(
                    static_cast<float>(i) / static_cast<float>(ray_count) + rotation, 1.0f);
                const float u2 = radical_inverse(static_cast<uint32_t>(i) + 1u);

                const float r = std::sqrt(u1);
                const float phi = 6.2831853071795864f * u2;
                const float x = r * std::cos(phi);
                const float y = r * std::sin(phi);
                const float z = std::sqrt(std::max(0.0f, 1.0f - u1));

                const glm::vec3 dir = tangent * x + bitangent * y + n * z;

                float hit_t = max_distance;
                bool hit = bvh.occluded(origin, dir, max_distance, hit_t);

                // The analytic ground. An OSM building is walls and a roof with no
                // floor, so without this the base of every wall is as open as its
                // top -- and the base is the one place the eye insists on seeing
                // contact.
                if (settings.use_ground_plane && dir.y < -kEpsilon) {
                    const float ground_t = (settings.ground_height - origin.y) / dir.y;
                    // `>= 0`, NOT `> epsilon`. The plane is a solid HALF-SPACE, not
                    // a sheet, and the vertex this matters most for -- the foot of a
                    // wall -- sits exactly on it, giving t = 0 for every downward
                    // ray. Rejecting those left the base of every wall as open as
                    // its top, which is the whole artefact this setting exists to
                    // remove.
                    if (ground_t >= 0.0f && ground_t < max_distance && ground_t < hit_t) {
                        hit_t = ground_t;
                        hit = true;
                    }
                }

                if (hit) {
                    // Distance falloff. A binary count makes a distant wall across
                    // a street occlude exactly as hard as one touching this vertex,
                    // which is what turns a whole street into an interior.
                    const float falloff = 1.0f - std::clamp(hit_t / max_distance, 0.0f, 1.0f);
                    occlusion += falloff * falloff;
                }
            }

            occlusion /= static_cast<float>(ray_count);
            vertex.ao = std::clamp(1.0f - strength * occlusion, min_ao, 1.0f);
        }
    };

    // Vertices are independent and each is written exactly once, so the split is
    // invisible in the result: the bake is deterministic at any thread count.
    unsigned threads = settings.thread_count > 0
                           ? static_cast<unsigned>(settings.thread_count)
                           : std::thread::hardware_concurrency();
    if (threads == 0) threads = 1;
    threads = std::min<unsigned>(threads, static_cast<unsigned>(mesh.vertices.size()));

    if (threads <= 1) {
        bake_range(0, mesh.vertices.size());
        return;
    }

    std::vector<std::thread> workers;
    workers.reserve(threads - 1);
    const size_t per_thread = (mesh.vertices.size() + threads - 1) / threads;
    for (unsigned t = 1; t < threads; ++t) {
        const size_t begin = std::min(mesh.vertices.size(), per_thread * t);
        const size_t end = std::min(mesh.vertices.size(), begin + per_thread);
        if (begin >= end) break;
        workers.emplace_back(bake_range, begin, end);
    }
    bake_range(0, std::min(mesh.vertices.size(), per_thread));
    for (std::thread& worker : workers) worker.join();
}

} // namespace stratum::geometry
