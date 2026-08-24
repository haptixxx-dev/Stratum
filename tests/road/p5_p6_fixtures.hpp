/**
 * @file p5_p6_fixtures.hpp
 * @brief Shared helpers for the P5 detail and P6 structure suites
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The four P5/P6 suites -- markings, crossings, sidewalk dedup, structures --
 * plus their integration suite all need the same handful of things that
 * tests/road/junction_fixtures.hpp does not carry, because the questions this
 * phase asks are different in kind from the ones P4 asked.
 *
 * P4 asked "where is this polygon". P5 and P6 ask "which discrete quads came
 * out, what sprite is on each of them, and is each one sitting on the surface it
 * claims to be sitting on". So the helpers here are:
 *
 * 1. **Connected components.** Marking geometry is a scatter of independent
 *    quads: one per dash, one per arrow, one per zebra stripe. Nothing in a Mesh
 *    says where one quad ends and the next begins except that they share no
 *    vertices, which is exactly what a union-find over the index buffer
 *    recovers. Every "how many dashes" and "which sprite is on this arrow"
 *    assertion is built on components_of().
 * 2. **Sprite recognition.** A quad's UV rect is its identity, because
 *    MaterialId::Markings is the one atlased material in the pipeline. See
 *    marking_atlas.hpp.
 * 3. **Road-frame projection.** "Inside the carriageway" and "3 m back from the
 *    trim station" are statements in the road's own frame, not in world space.
 * 4. **Terrain samplers.** A .osm file holds no terrain, and P6 is entirely
 *    about the relationship between a road and the ground it crosses, so the
 *    hill has to come from the test. It also has to be positioned against the
 *    PARSED geometry, since OSMParser::recenter_on_features() moves every
 *    coordinate.
 * 5. **A segment-versus-mesh ray test.** The one question a plan-view predicate
 *    cannot answer is whether a hole is really a hole. A tunnel portal is a
 *    vertical wall, so in plan it is a line; the only way to ask whether the
 *    road can get through it is to fire a segment along the carriageway and
 *    check that it hits nothing.
 *
 * Everything that IS in junction_fixtures.hpp -- parse_fixture(), triangles_of(),
 * world_to_local(), the 2D predicates, profiles_from_tags() -- is used from
 * there rather than duplicated. This header includes it.
 *
 * ### Coordinates
 *
 * 2D local metres throughout, matching GraphEdge::polyline and
 * Centerline::stations. Mesh vertices are world space, Y up, and are brought back
 * with junction::world_to_local(): `(x, y, z) -> (x, -z)`.
 */

#pragma once

#include "framework.hpp"
#include "road/junction_fixtures.hpp"

#include "osm/road/centerline.hpp"
#include "osm/road/marking_atlas.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/road/road_profile.hpp"
#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace stratum::test::p5 {

namespace junction = ::stratum::test::junction;

// ============================================================================
// Constants
// ============================================================================

/// Tolerance for a normalised UV compared against the atlas table
inline constexpr double kUvEps = 1e-6;

/// The fixtures this phase added, in tests/data/README.md table order
inline constexpr const char* kNewFixtures[] = {
    "crossing.osm", "sidewalk_dup.osm", "turn_lanes.osm",
    "dual_carriageway.osm", "service_roads.osm", "tunnel.osm",
};

/**
 * @brief Every fixture in tests/data, old and new
 *
 * The integration suite sweeps this rather than the P4 list, so a fixture added
 * for one narrow rule is still checked for NaN, for material tagging, and for
 * the switch-off reproduction by every later phase.
 */
inline std::vector<std::string> all_fixtures() {
    std::vector<std::string> out;
    for (const char* name : junction::kAllFixtures) out.emplace_back(name);
    for (const char* name : kNewFixtures) out.emplace_back(name);
    return out;
}

// ============================================================================
// Pipeline setup
// ============================================================================

/**
 * @brief A parsed fixture taken as far as profiles, without extruding anything
 *
 * The state every P5 entry point takes: a graph, one centerline per EdgeId, one
 * profile per EdgeId. Built exactly the way RoadNetworkBuilder builds it, raw way
 * tags included, so a test measures the same cross-section the pipeline would.
 */
struct Network {
    osm::ParsedOSMData data;
    osm::road::RoadGraph graph;
    std::vector<osm::road::Centerline> centerlines;
    std::vector<osm::road::RoadProfile> profiles;
    bool ok = false;
};

/**
 * @brief Parse a fixture and build its graph, centerlines and profiles
 *
 * Smoothing is off. These tests reason about arclength along straight fixture
 * roads and compare it against chord lengths they computed themselves; a fitted
 * Catmull-Rom would move every station a little and turn an exact expectation
 * into an approximate one. The curved cases build their centerlines by hand.
 *
 * @param filename Fixture file name in tests/data
 * @param cfg      Profile widths and heights
 * @return The network; `ok` false when the fixture did not parse
 */
inline Network make_network(const char* filename,
                            const osm::road::ProfileConfig& cfg = {}) {
    Network net;
    auto parsed = junction::parse_fixture(filename);
    if (!parsed) return net;

    net.data = std::move(*parsed);
    net.graph.build(net.data);
    net.centerlines = junction::make_centerlines(net.graph);
    net.profiles = junction::profiles_from_tags(net.graph, net.data, cfg);
    net.ok = true;
    return net;
}

/**
 * @brief Every EdgeId built from one OSM way, in EdgeId order
 *
 * A way splits at its graph nodes, so "the road tagged turn:lanes" is a set of
 * edges rather than one, and a test that assumed one would break the first time
 * a fixture gained a junction in the middle of it.
 *
 * @param graph  Built road graph
 * @param way_id OSM way ID
 * @return Matching EdgeIds
 */
inline std::vector<osm::road::EdgeId> edges_of_way(const osm::road::RoadGraph& graph,
                                                   osm::WayId way_id) {
    std::vector<osm::road::EdgeId> out;
    for (size_t i = 0; i < graph.edges().size(); ++i) {
        if (graph.edges()[i].source_way == way_id) {
            out.push_back(static_cast<osm::road::EdgeId>(i));
        }
    }
    return out;
}

/**
 * @brief The single EdgeId built from one OSM way, when the way did not split
 *
 * @param graph  Built road graph
 * @param way_id OSM way ID
 * @return Its EdgeId, or kInvalidId when the way produced zero or several edges
 */
inline osm::road::EdgeId sole_edge_of_way(const osm::road::RoadGraph& graph,
                                          osm::WayId way_id) {
    const auto ids = edges_of_way(graph, way_id);
    return ids.size() == 1 ? ids[0] : osm::road::kInvalidId;
}

/// Raw way tags for an edge's parent way, or nullptr when the way is absent
inline const osm::TagMap* tags_of(const osm::ParsedOSMData& data, const osm::road::GraphEdge& edge) {
    const auto it = data.ways.find(edge.source_way);
    return (it != data.ways.end()) ? &it->second.tags : nullptr;
}

/// A flat vector of per-station heights, the shape every P5/P6 entry point wants
inline std::vector<float> flat_heights(const osm::road::Centerline& cl, float height) {
    return std::vector<float>(cl.stations.size(), height);
}

// ============================================================================
// Road-frame projection
// ============================================================================

/**
 * @brief Index of the station nearest a point, by plan distance
 *
 * @param cl Centerline to search
 * @param p  Point in 2D local metres
 * @return Station index, or 0 for an empty centerline
 */
inline size_t nearest_station(const osm::road::Centerline& cl, const glm::dvec2& p) {
    size_t best = 0;
    double best_d2 = 1e300;
    for (size_t i = 0; i < cl.stations.size(); ++i) {
        const glm::dvec2 d = cl.stations[i].position - p;
        const double d2 = glm::dot(d, d);
        if (d2 < best_d2) {
            best_d2 = d2;
            best = i;
        }
    }
    return best;
}

/**
 * @brief Signed lateral offset of a point from the centerline, positive LEFT
 *
 * Projected onto the nearest station's frame. Exact on a straight road, which is
 * what every fixture that asserts a lateral is; on a curve, use the plan-view
 * coverage predicates instead, which do not depend on picking a station.
 *
 * @param cl Centerline to measure against
 * @param p  Point in 2D local metres
 * @return Metres left of the centerline, negative to the right
 */
inline double lateral_of(const osm::road::Centerline& cl, const glm::dvec2& p) {
    if (cl.stations.empty()) return 0.0;
    const auto& s = cl.stations[nearest_station(cl, p)];
    return glm::dot(p - s.position, s.normal);
}

/**
 * @brief Arclength of the point on the centerline nearest @p p
 *
 * Interpolated within the band the projection falls in, so the result is not
 * quantised to the station spacing.
 *
 * @param cl Centerline to measure along
 * @param p  Point in 2D local metres
 * @return Metres from the start of @p cl, in its own parameterisation
 */
inline double arclength_of(const osm::road::Centerline& cl, const glm::dvec2& p) {
    if (cl.stations.size() < 2) return 0.0;
    double best_d2 = 1e300;
    double best_arc = cl.stations.front().arclength;
    for (size_t i = 0; i + 1 < cl.stations.size(); ++i) {
        const glm::dvec2 a = cl.stations[i].position;
        const glm::dvec2 b = cl.stations[i + 1].position;
        const glm::dvec2 ab = b - a;
        const double len2 = glm::dot(ab, ab);
        if (len2 <= 1e-18) continue;
        const double t = std::clamp(glm::dot(p - a, ab) / len2, 0.0, 1.0);
        const glm::dvec2 q = a + ab * t;
        const glm::dvec2 d = q - p;
        const double d2 = glm::dot(d, d);
        if (d2 < best_d2) {
            best_d2 = d2;
            best_arc = cl.stations[i].arclength +
                       t * (cl.stations[i + 1].arclength - cl.stations[i].arclength);
        }
    }
    return best_arc;
}

/**
 * @brief Lateral coordinate of every strip boundary, left to right
 *
 * The same walk RoadProfile documents: start at left_edge_offset() and subtract
 * each width. The returned vector has `strips.size() + 1` entries, so entry i is
 * strip i's LEFT edge and entry i+1 its right.
 *
 * @param profile Cross-section to walk
 * @return Boundary laterals, descending
 */
inline std::vector<double> strip_boundaries(const osm::road::RoadProfile& profile) {
    std::vector<double> out;
    out.reserve(profile.strips.size() + 1);
    double lateral = static_cast<double>(profile.left_edge_offset());
    out.push_back(lateral);
    for (const auto& strip : profile.strips) {
        lateral -= static_cast<double>(strip.width);
        out.push_back(lateral);
    }
    return out;
}

/**
 * @brief Lateral span of the outermost Lane strips, left edge first
 *
 * The carriageway the paint has to stay inside. Not
 * RoadProfile::carriageway_width(), which sums Lane widths and so collapses a
 * dual carriageway's two groups against each other as if the median were not
 * there.
 *
 * @param profile Cross-section to measure
 * @param out_left  Receives the leftmost Lane strip's left edge
 * @param out_right Receives the rightmost Lane strip's right edge
 * @return True when the profile holds at least one Lane strip
 */
inline bool lane_span(const osm::road::RoadProfile& profile, double& out_left, double& out_right) {
    const std::vector<double> bounds = strip_boundaries(profile);
    bool found = false;
    for (size_t i = 0; i < profile.strips.size(); ++i) {
        if (profile.strips[i].kind != osm::road::StripKind::Lane) continue;
        if (!found) {
            out_left = bounds[i];
            found = true;
        }
        out_right = bounds[i + 1];
    }
    return found;
}

// ============================================================================
// Mesh components
// ============================================================================

/**
 * @brief One connected run of triangles inside a Mesh
 *
 * Connectivity is by shared VERTEX INDEX, not by shared position. That is the
 * right relation for marking geometry: two dashes on one line sit on the same
 * plane a few metres apart and are separate quads, while the two triangles of
 * one dash share an index pair by construction. It is also why this must not be
 * used on corridor geometry to count strips: the extruder gives each strip its
 * own columns, so a corridor decomposes into one component per strip whether or
 * not the strips touch.
 */
struct Component {
    std::vector<uint32_t> vertices;   ///< Vertex indices, ascending
    std::vector<size_t> triangles;    ///< Triangle indices into the index buffer / 3
    MaterialId material = MaterialId::Default;
};

/**
 * @brief Split a mesh into vertex-connected components
 *
 * Vertices no triangle references are not reported at all: they carry no
 * geometry and a producer that emitted one has a leak, not a component.
 *
 * @param mesh Mesh to split
 * @return Components, ordered by their lowest triangle index so the result is
 *         reproducible run to run
 */
inline std::vector<Component> components_of(const Mesh& mesh) {
    std::vector<Component> out;
    const size_t tri_count = mesh.indices.size() / 3;
    if (tri_count == 0) return out;

    std::vector<uint32_t> parent(mesh.vertices.size());
    for (uint32_t i = 0; i < parent.size(); ++i) parent[i] = i;

    const std::function<uint32_t(uint32_t)> find = [&](uint32_t v) {
        while (parent[v] != v) {
            parent[v] = parent[parent[v]];
            v = parent[v];
        }
        return v;
    };
    const auto unite = [&](uint32_t a, uint32_t b) {
        const uint32_t ra = find(a);
        const uint32_t rb = find(b);
        if (ra != rb) parent[ra] = rb;
    };

    std::vector<MaterialId> per_triangle(tri_count, MaterialId::Default);
    for (const SubMesh& sub : mesh.effective_submeshes()) {
        const size_t first = sub.index_offset / 3u;
        const size_t last = (static_cast<size_t>(sub.index_offset) + sub.index_count) / 3u;
        for (size_t t = first; t < last && t < tri_count; ++t) per_triangle[t] = sub.material;
    }

    std::vector<bool> usable(tri_count, false);
    for (size_t t = 0; t < tri_count; ++t) {
        const uint32_t i0 = mesh.indices[t * 3 + 0];
        const uint32_t i1 = mesh.indices[t * 3 + 1];
        const uint32_t i2 = mesh.indices[t * 3 + 2];
        if (i0 >= mesh.vertices.size() || i1 >= mesh.vertices.size() ||
            i2 >= mesh.vertices.size()) {
            continue;
        }
        usable[t] = true;
        unite(i0, i1);
        unite(i1, i2);
    }

    // Roots in first-triangle order, so the component list does not depend on
    // the vertex numbering the union-find happened to settle on.
    std::vector<uint32_t> order;
    std::vector<uint32_t> slot(mesh.vertices.size(), 0xFFFFFFFFu);
    for (size_t t = 0; t < tri_count; ++t) {
        if (!usable[t]) continue;
        const uint32_t root = find(mesh.indices[t * 3]);
        if (slot[root] == 0xFFFFFFFFu) {
            slot[root] = static_cast<uint32_t>(order.size());
            order.push_back(root);
            out.push_back(Component{});
            out.back().material = per_triangle[t];
        }
        out[slot[root]].triangles.push_back(t);
    }

    for (size_t t = 0; t < tri_count; ++t) {
        if (!usable[t]) continue;
        const uint32_t root = find(mesh.indices[t * 3]);
        Component& comp = out[slot[root]];
        for (int k = 0; k < 3; ++k) comp.vertices.push_back(mesh.indices[t * 3 + k]);
    }
    for (Component& comp : out) {
        std::sort(comp.vertices.begin(), comp.vertices.end());
        comp.vertices.erase(std::unique(comp.vertices.begin(), comp.vertices.end()),
                            comp.vertices.end());
    }
    return out;
}

/// Every world-space position of a component's vertices
inline std::vector<glm::vec3> positions_of(const Mesh& mesh, const Component& comp) {
    std::vector<glm::vec3> out;
    out.reserve(comp.vertices.size());
    for (uint32_t v : comp.vertices) out.push_back(mesh.vertices[v].position);
    return out;
}

/// Every 2D local position of a component's vertices
inline std::vector<glm::dvec2> locals_of(const Mesh& mesh, const Component& comp) {
    std::vector<glm::dvec2> out;
    out.reserve(comp.vertices.size());
    for (uint32_t v : comp.vertices) {
        out.push_back(junction::world_to_local(mesh.vertices[v].position));
    }
    return out;
}

/// Mean 2D local position of a component's vertices
inline glm::dvec2 centroid_of(const Mesh& mesh, const Component& comp) {
    glm::dvec2 sum{0.0};
    const std::vector<glm::dvec2> pts = locals_of(mesh, comp);
    for (const glm::dvec2& p : pts) sum += p;
    return pts.empty() ? sum : sum / static_cast<double>(pts.size());
}

/// Min and max of a component's vertices projected onto @p axis, in 2D local metres
inline void extent_along(const Mesh& mesh, const Component& comp, const glm::dvec2& origin,
                         const glm::dvec2& axis, double& out_min, double& out_max) {
    out_min = 1e300;
    out_max = -1e300;
    for (const glm::dvec2& p : locals_of(mesh, comp)) {
        const double s = glm::dot(p - origin, axis);
        out_min = std::min(out_min, s);
        out_max = std::max(out_max, s);
    }
}

// ============================================================================
// Sprite recognition
// ============================================================================

/// UV bounding rect of a component's vertices
inline osm::road::SpriteRect uv_rect_of(const Mesh& mesh, const Component& comp) {
    osm::road::SpriteRect r{1e30f, 1e30f, -1e30f, -1e30f};
    for (uint32_t v : comp.vertices) {
        const glm::vec2 uv = mesh.vertices[v].uv;
        r.u0 = std::min(r.u0, uv.x);
        r.v0 = std::min(r.v0, uv.y);
        r.u1 = std::max(r.u1, uv.x);
        r.v1 = std::max(r.v1, uv.y);
    }
    return r;
}

/**
 * @brief Which sprite a component's UV rect maps
 *
 * A marking quad's identity IS its atlas rect; MaterialId::Markings is the only
 * atlased material and the rects do not overlap, so the match is unambiguous.
 * The comparison is on the bounding rect, which means it is insensitive to the
 * emitter flipping the quad -- flipping is the emitter's decision and is checked
 * separately where it matters.
 *
 * @param mesh Mesh the component belongs to
 * @param comp Component to identify
 * @return The sprite, or MarkingSprite::Count when no rect matches
 */
inline osm::road::MarkingSprite sprite_of(const Mesh& mesh, const Component& comp) {
    const osm::road::SpriteRect got = uv_rect_of(mesh, comp);
    const auto count = static_cast<uint8_t>(osm::road::MarkingSprite::Count);
    for (uint8_t i = 0; i < count; ++i) {
        const auto s = static_cast<osm::road::MarkingSprite>(i);
        const osm::road::SpriteRect want = osm::road::sprite_rect(s);
        if (std::fabs(got.u0 - want.u0) <= kUvEps && std::fabs(got.v0 - want.v0) <= kUvEps &&
            std::fabs(got.u1 - want.u1) <= kUvEps && std::fabs(got.v1 - want.v1) <= kUvEps) {
            return s;
        }
    }
    return osm::road::MarkingSprite::Count;
}

/// Human-readable sprite name, "Count" included, for a failure message
inline const char* sprite_name(osm::road::MarkingSprite s) {
    return s == osm::road::MarkingSprite::Count ? "<no matching rect>"
                                                : osm::road::marking_sprite_name(s);
}

// ============================================================================
// Mesh predicates
// ============================================================================

/// Triangles attributed to one material, from Mesh::effective_submeshes()
inline size_t triangles_with_material(const Mesh& mesh, MaterialId material) {
    size_t n = 0;
    for (const junction::Tri2D& tri : junction::triangles_of(mesh)) {
        if (tri.material == material) ++n;
    }
    return n;
}

/// Plan-view area of the triangles attributed to one material, square metres
inline double plan_area_of_material(const Mesh& mesh, MaterialId material) {
    double sum = 0.0;
    for (const junction::Tri2D& tri : junction::triangles_of(mesh)) {
        if (tri.material != material) continue;
        sum += 0.5 * std::fabs(junction::cross2(tri.a, tri.b, tri.c));
    }
    return sum;
}

/// Triangles attributed to one material, projected into 2D
inline std::vector<junction::Tri2D> triangles_with(const Mesh& mesh, MaterialId material) {
    std::vector<junction::Tri2D> out;
    for (const junction::Tri2D& tri : junction::triangles_of(mesh)) {
        if (tri.material == material) out.push_back(tri);
    }
    return out;
}

/// Every vertex position is finite in all three components
inline bool mesh_is_finite(const Mesh& mesh) {
    for (const Vertex& v : mesh.vertices) {
        if (!junction::is_finite(v.position)) return false;
        if (!std::isfinite(v.uv.x) || !std::isfinite(v.uv.y)) return false;
        if (!std::isfinite(v.normal.x) || !std::isfinite(v.normal.y) ||
            !std::isfinite(v.normal.z)) {
            return false;
        }
    }
    return true;
}

/// Every index is inside the vertex array and the index count is a whole number of triangles
inline bool mesh_indices_are_sane(const Mesh& mesh) {
    if ((mesh.indices.size() % 3u) != 0u) return false;
    for (uint32_t i : mesh.indices) {
        if (i >= mesh.vertices.size()) return false;
    }
    return true;
}

/**
 * @brief No triangle of the mesh has near-zero area in 3D
 *
 * A zero-area triangle carries no surface, has no usable normal, and is the
 * shape a fold or a collapsed column comes out as. The threshold is in square
 * metres of true 3D area, so a vertical wall is measured as honestly as a
 * horizontal slab.
 *
 * @param mesh    Mesh to check
 * @param min_area Smallest area still counted as a real triangle
 * @return True when every triangle is above the threshold
 */
inline bool no_degenerate_triangles(const Mesh& mesh, double min_area = 1e-9) {
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const glm::dvec3 a(mesh.vertices[mesh.indices[t + 0]].position);
        const glm::dvec3 b(mesh.vertices[mesh.indices[t + 1]].position);
        const glm::dvec3 c(mesh.vertices[mesh.indices[t + 2]].position);
        if (0.5 * glm::length(glm::cross(b - a, c - a)) < min_area) return false;
    }
    return true;
}

/**
 * @brief Two meshes share no vertex POSITION
 *
 * Positions, not indices: the two meshes have separate index spaces, and the
 * property markings.hpp states -- "they never share vertices with the corridor"
 * -- is a statement about geometry that survives the two being appended into one
 * buffer.
 *
 * @param a       First mesh
 * @param b       Second mesh
 * @param epsilon Distance below which two positions count as the same, metres
 * @return True when no vertex of @p a is within @p epsilon of any vertex of @p b
 */
inline bool share_no_vertex(const Mesh& a, const Mesh& b, double epsilon = 1e-6) {
    for (const Vertex& va : a.vertices) {
        for (const Vertex& vb : b.vertices) {
            if (glm::length(glm::dvec3(va.position) - glm::dvec3(vb.position)) <= epsilon) {
                return false;
            }
        }
    }
    return true;
}

/**
 * @brief A segment crosses a triangle, Moller-Trumbore with the ray clipped to [0, 1]
 *
 * @param p0 Segment start, world space
 * @param p1 Segment end, world space
 * @param a  Triangle corner
 * @param b  Triangle corner
 * @param c  Triangle corner
 * @return True when the segment passes through the triangle's interior
 */
inline bool segment_hits_triangle(const glm::dvec3& p0, const glm::dvec3& p1,
                                  const glm::dvec3& a, const glm::dvec3& b,
                                  const glm::dvec3& c) {
    const glm::dvec3 dir = p1 - p0;
    const glm::dvec3 e1 = b - a;
    const glm::dvec3 e2 = c - a;
    const glm::dvec3 h = glm::cross(dir, e2);
    const double det = glm::dot(e1, h);
    if (std::fabs(det) < 1e-12) return false;   // parallel to the plane
    const double inv = 1.0 / det;
    const glm::dvec3 s = p0 - a;
    const double u = inv * glm::dot(s, h);
    if (u < 0.0 || u > 1.0) return false;
    const glm::dvec3 q = glm::cross(s, e1);
    const double v = inv * glm::dot(dir, q);
    if (v < 0.0 || u + v > 1.0) return false;
    const double t = inv * glm::dot(e2, q);
    return t > 0.0 && t < 1.0;
}

/**
 * @brief A segment crosses any triangle of a mesh
 *
 * The "is this hole really a hole" predicate. A plan-view coverage test cannot
 * answer it, because a headwall projects to a line in plan.
 *
 * @param mesh Mesh to test against
 * @param p0   Segment start, world space
 * @param p1   Segment end, world space
 * @return True when the segment is blocked
 */
inline bool segment_hits_mesh(const Mesh& mesh, const glm::dvec3& p0, const glm::dvec3& p1) {
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const glm::dvec3 a(mesh.vertices[mesh.indices[t + 0]].position);
        const glm::dvec3 b(mesh.vertices[mesh.indices[t + 1]].position);
        const glm::dvec3 c(mesh.vertices[mesh.indices[t + 2]].position);
        if (segment_hits_triangle(p0, p1, a, b, c)) return true;
    }
    return false;
}

/// Lowest world Y over the vertices of one material's triangles; +inf when absent
inline double min_height_of_material(const Mesh& mesh, MaterialId material) {
    double lowest = 1e300;
    const size_t tri_count = mesh.indices.size() / 3;
    std::vector<MaterialId> per_triangle(tri_count, MaterialId::Default);
    for (const SubMesh& sub : mesh.effective_submeshes()) {
        const size_t first = sub.index_offset / 3u;
        const size_t last = (static_cast<size_t>(sub.index_offset) + sub.index_count) / 3u;
        for (size_t t = first; t < last && t < tri_count; ++t) per_triangle[t] = sub.material;
    }
    for (size_t t = 0; t < tri_count; ++t) {
        if (per_triangle[t] != material) continue;
        for (int k = 0; k < 3; ++k) {
            lowest = std::min(lowest,
                              static_cast<double>(mesh.vertices[mesh.indices[t * 3 + k]].position.y));
        }
    }
    return lowest;
}

/// Highest world Y over the vertices of one material's triangles; -inf when absent
inline double max_height_of_material(const Mesh& mesh, MaterialId material) {
    double highest = -1e300;
    const size_t tri_count = mesh.indices.size() / 3;
    std::vector<MaterialId> per_triangle(tri_count, MaterialId::Default);
    for (const SubMesh& sub : mesh.effective_submeshes()) {
        const size_t first = sub.index_offset / 3u;
        const size_t last = (static_cast<size_t>(sub.index_offset) + sub.index_count) / 3u;
        for (size_t t = first; t < last && t < tri_count; ++t) per_triangle[t] = sub.material;
    }
    for (size_t t = 0; t < tri_count; ++t) {
        if (per_triangle[t] != material) continue;
        for (int k = 0; k < 3; ++k) {
            highest = std::max(highest,
                               static_cast<double>(mesh.vertices[mesh.indices[t * 3 + k]].position.y));
        }
    }
    return highest;
}

// ============================================================================
// Terrain
// ============================================================================

/// A perfectly flat terrain at one height
inline std::function<float(double, double)> flat_sampler(float height) {
    return [height](double, double) { return height; };
}

/**
 * @brief A raised-cosine ridge running ACROSS a road
 *
 * The height depends only on the distance measured ALONG @p axis from
 * @p center, so the hill is an infinite ridge perpendicular to the road rather
 * than a dome. That matters: a dome would leave the road climbing out sideways
 * as well as longitudinally, and every assertion about where the road passes
 * under the terrain would then depend on the profile width too.
 *
 * @code
 *     h(s) = base + peak * 0.5 * (1 + cos(pi * s / half_length))   for |s| < half_length
 *     h(s) = base                                                  otherwise
 * @endcode
 *
 * Continuous in value and in first derivative at the foot, so the grade solve
 * has nothing artificial to catch on.
 *
 * @param center      Crest position in 2D local metres
 * @param axis        Unit direction the ridge is measured along, normally the road tangent
 * @param half_length Distance from the crest to the foot, metres
 * @param base        Ground height away from the ridge, metres
 * @param peak        Height of the crest above @p base, metres
 * @return The sampler, safe to call from several threads
 */
inline std::function<float(double, double)> ridge_sampler(glm::dvec2 center, glm::dvec2 axis,
                                                          double half_length, float base,
                                                          float peak) {
    const double len = glm::length(axis);
    const glm::dvec2 unit = len > 1e-12 ? axis / len : glm::dvec2{1.0, 0.0};
    const double span = std::max(half_length, 1e-6);
    return [center, unit, span, base, peak](double x, double y) {
        const double s = glm::dot(glm::dvec2{x, y} - center, unit);
        if (std::fabs(s) >= span) return base;
        const double pi = 3.14159265358979323846;
        return base + peak * static_cast<float>(0.5 * (1.0 + std::cos(pi * s / span)));
    };
}

/// A constant slope, for a bridge that has real ground falling away beneath it
inline std::function<float(double, double)> slope_sampler(glm::dvec2 origin, glm::dvec2 axis,
                                                          float base, float grade) {
    const double len = glm::length(axis);
    const glm::dvec2 unit = len > 1e-12 ? axis / len : glm::dvec2{1.0, 0.0};
    return [origin, unit, base, grade](double x, double y) {
        return base + grade * static_cast<float>(glm::dot(glm::dvec2{x, y} - origin, unit));
    };
}

} // namespace stratum::test::p5
