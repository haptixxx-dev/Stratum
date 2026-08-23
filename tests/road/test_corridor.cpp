/**
 * @file test_corridor.cpp
 * @brief Corridor extrusion tests: UVs, winding, submeshes, weld, and outline
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * These tests are written against the contract in src/osm/road/corridor.hpp and
 * the frozen "UV Convention" section of docs/plans/road_network_plan.md.
 *
 * The convention is a contract, not a preference:
 *
 * @code
 *     U = lateral_metres_within_strip / tile_u_metres(material)
 *     V = arclength_metres_along_road / tile_v_metres(material)
 * @endcode
 *
 * Asphalt tiles every 8 m in both axes, so on a straight road sampled every 8 m
 * the V of consecutive vertex rows must differ by exactly 1.0, and a 3.5 m lane
 * must span exactly 0.4375 in U. The old extruder mapped each quad 0..1, so texel
 * density varied with segment length and no tiling material could look right.
 * That is what the texel-density test pins down.
 *
 * Winding is checked here rather than in the GUI: an inverted road surface is
 * invisible from above with back-face culling on and reads as missing geometry,
 * which is an expensive thing to diagnose by eye.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests Corridor
 * @endcode
 */

#include "framework.hpp"

#include "osm/road/centerline.hpp"
#include "osm/road/corridor.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/road/road_profile.hpp"
#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace {

using stratum::MaterialId;
using stratum::Mesh;
using stratum::SubMesh;
using stratum::Vertex;
using stratum::material_id_name;
using stratum::osm::RoadType;
using stratum::osm::SideFlags;
using stratum::osm::road::Centerline;
using stratum::osm::road::Corridor;
using stratum::osm::road::CorridorConfig;
using stratum::osm::road::GraphEdge;
using stratum::osm::road::ProfileConfig;
using stratum::osm::road::ResampleConfig;
using stratum::osm::road::RoadProfile;
using stratum::osm::road::Station;
using stratum::osm::road::Strip;
using stratum::osm::road::StripKind;
using stratum::osm::road::build_centerline;
using stratum::osm::road::build_corridor;
using stratum::osm::road::build_profile;
using stratum::osm::road::uv_tiling;

/// Asphalt tiles every 8 m, per the frozen UV Convention table
constexpr double kAsphaltTile = 8.0;

/// Float UV comparisons: the mesh stores UVs as float, so exactness stops here
constexpr double kUvEps = 1e-4;

/// Sentinel for a vertex no submesh range references
constexpr MaterialId kNoMaterial = MaterialId::Count;

/**
 * @brief A straight centerline along +X with a station every 8 m
 *
 * 80 m at the default 8 m max spacing gives exactly 11 stations, so the expected
 * V step is exactly 8 / 8 = 1.0 and every assertion below can be stated exactly.
 *
 * @param length Total length in metres
 * @return The centerline, smoothing disabled so the geometry is exactly the input
 */
Centerline straight_centerline(double length) {
    ResampleConfig cfg;
    cfg.smooth = false;
    return build_centerline({{0.0, 0.0}, {length, 0.0}}, cfg);
}

/**
 * @brief The canonical two-lane residential cross-section
 *
 * @param cfg Profile tunables
 * @return The profile: sidewalk, curb, gutter, lane, lane, gutter, curb, sidewalk
 */
RoadProfile residential_profile(const ProfileConfig& cfg = ProfileConfig{}) {
    GraphEdge edge;
    edge.source_way = 1;
    edge.polyline = {{0.0, 0.0}, {80.0, 0.0}};
    edge.node_ids = {1, 2};
    edge.type = RoadType::Residential;
    edge.lanes = 2;
    edge.width = 0.0f;                  // no width tag: fall back to lane_width_default
    edge.sidewalk = SideFlags::Both;
    return build_profile(edge, cfg);
}

/**
 * @brief Strips that must produce geometry
 *
 * A zero-width strip emits nothing unless it is a CurbFace with a height change,
 * which is the perfectly vertical curb and still needs its quad. CurbFace strips
 * emit nothing at all when the caller turned them off.
 *
 * @param p   Profile being swept
 * @param cfg Corridor configuration, for emit_curb_faces
 * @return Number of strips that contribute a band
 */
size_t emitted_strip_count(const RoadProfile& p, const CorridorConfig& cfg) {
    size_t n = 0;
    for (const Strip& s : p.strips) {
        if (s.kind == StripKind::CurbFace) {
            if (!cfg.emit_curb_faces) continue;
            if (s.width > 0.0f || s.height_left != s.height_right) ++n;
            continue;
        }
        if (s.width > 0.0f) ++n;
    }
    return n;
}

/// Widest Lane strip in the profile, in metres
double widest_lane(const RoadProfile& p) {
    double w = 0.0;
    for (const Strip& s : p.strips) {
        if (s.kind == StripKind::Lane) w = std::max(w, static_cast<double>(s.width));
    }
    return w;
}

/**
 * @brief Material of every vertex, resolved through the submesh ranges
 *
 * Each strip owns its own vertex columns, so a vertex belongs to exactly one
 * material. A vertex no range references comes back as kNoMaterial.
 *
 * @param mesh Mesh to resolve
 * @return One MaterialId per vertex, parallel to mesh.vertices
 */
std::vector<MaterialId> vertex_materials(const Mesh& mesh) {
    std::vector<MaterialId> out(mesh.vertices.size(), kNoMaterial);
    for (const SubMesh& sub : mesh.effective_submeshes()) {
        const size_t end = static_cast<size_t>(sub.index_offset) + sub.index_count;
        for (size_t i = sub.index_offset; i < end && i < mesh.indices.size(); ++i) {
            const uint32_t v = mesh.indices[i];
            if (v < out.size()) out[v] = sub.material;
        }
    }
    return out;
}

/// Indices of every vertex carrying a given material
std::vector<size_t> vertices_of(const Mesh& mesh, MaterialId material) {
    const std::vector<MaterialId> owner = vertex_materials(mesh);
    std::vector<size_t> out;
    for (size_t i = 0; i < owner.size(); ++i) {
        if (owner[i] == material) out.push_back(i);
    }
    return out;
}

/// Sorted, deduplicated values, with a tolerance
std::vector<double> unique_sorted(std::vector<double> values, double eps) {
    std::sort(values.begin(), values.end());
    std::vector<double> out;
    for (double v : values) {
        if (out.empty() || std::fabs(v - out.back()) > eps) out.push_back(v);
    }
    return out;
}

/// Shoelace signed area of a ring; positive is counter-clockwise
double signed_area(const std::vector<glm::dvec2>& ring) {
    double a = 0.0;
    for (size_t i = 0; i < ring.size(); ++i) {
        const glm::dvec2& p = ring[i];
        const glm::dvec2& q = ring[(i + 1) % ring.size()];
        a += p.x * q.y - q.x * p.y;
    }
    return a * 0.5;
}

/// True when every component of a world position is finite
bool is_finite(const glm::vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

/**
 * @brief Assert that the submesh ranges tile the index buffer exactly once
 *
 * The contract is that build_corridor() calls sort_submeshes_by_material(), so
 * the ranges are ascending by MaterialId, hold at most one entry per material,
 * and leave no gap and no overlap.
 *
 * @param mesh  Mesh to check
 * @param label Test context, for the failure message
 */
void check_submeshes_tile(const Mesh& mesh, const std::string& label) {
    if (mesh.indices.empty()) {
        stratum::test::report_failure(__FILE__, __LINE__, "mesh has indices",
                                      label + ": no indices to tile");
        return;
    }
    if (mesh.submeshes.empty()) {
        stratum::test::report_failure(__FILE__, __LINE__, "mesh carries explicit submeshes",
                                      label + ": submeshes is empty");
        return;
    }

    uint32_t expected_offset = 0;
    int previous = -1;
    for (size_t i = 0; i < mesh.submeshes.size(); ++i) {
        const SubMesh& sub = mesh.submeshes[i];
        const std::string where = label + " submesh " + std::to_string(i) + " (" +
                                  material_id_name(sub.material) + ")";

        if (sub.index_offset != expected_offset) {
            stratum::test::report_failure(__FILE__, __LINE__, "submesh ranges are contiguous",
                                          where + ": offset " + std::to_string(sub.index_offset) +
                                              " expected " + std::to_string(expected_offset));
        }
        if (sub.index_count == 0 || (sub.index_count % 3u) != 0u) {
            stratum::test::report_failure(__FILE__, __LINE__, "submesh covers whole triangles",
                                          where + ": count " + std::to_string(sub.index_count));
        }
        const int slot = static_cast<int>(sub.material);
        if (slot <= previous) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "materials are ascending and appear exactly once",
                where + ": slot " + std::to_string(slot) + " after " + std::to_string(previous));
        }
        previous = slot;
        expected_offset += sub.index_count;
    }

    if (expected_offset != mesh.indices.size()) {
        stratum::test::report_failure(__FILE__, __LINE__, "submeshes cover the whole index buffer",
                                      label + ": covered " + std::to_string(expected_offset) +
                                          " of " + std::to_string(mesh.indices.size()));
    }
}

} // namespace

// ============================================================================
// The frozen UV tiling table
// ============================================================================

TEST(Corridor, uv_tiling_matches_the_frozen_table) {
    CHECK_NEAR(uv_tiling(MaterialId::Asphalt).u_metres, 8.0, 1e-9);
    CHECK_NEAR(uv_tiling(MaterialId::Asphalt).v_metres, 8.0, 1e-9);
    CHECK_NEAR(uv_tiling(MaterialId::Concrete).u_metres, 4.0, 1e-9);
    CHECK_NEAR(uv_tiling(MaterialId::Concrete).v_metres, 4.0, 1e-9);
    CHECK_NEAR(uv_tiling(MaterialId::BridgeDeck).u_metres, 4.0, 1e-9);
    CHECK_NEAR(uv_tiling(MaterialId::BridgeDeck).v_metres, 4.0, 1e-9);
    CHECK_NEAR(uv_tiling(MaterialId::Sidewalk).u_metres, 2.0, 1e-9);
    CHECK_NEAR(uv_tiling(MaterialId::Sidewalk).v_metres, 2.0, 1e-9);

    // Curb's U is the height of one repeat UP the face, not a lateral distance.
    CHECK_NEAR(uv_tiling(MaterialId::Curb).u_metres, 0.5, 1e-9);
    CHECK_NEAR(uv_tiling(MaterialId::Curb).v_metres, 2.0, 1e-9);

    for (MaterialId m : {MaterialId::Gravel, MaterialId::Dirt, MaterialId::Grass}) {
        CHECK_NEAR(uv_tiling(m).u_metres, 4.0, 1e-9);
        CHECK_NEAR(uv_tiling(m).v_metres, 4.0, 1e-9);
    }

    CHECK_NEAR(uv_tiling(MaterialId::Parapet).u_metres, 2.0, 1e-9);
    CHECK_NEAR(uv_tiling(MaterialId::Parapet).v_metres, 2.0, 1e-9);

    // Markings is an atlas, so its entry is a neutral placeholder.
    CHECK_NEAR(uv_tiling(MaterialId::Markings).u_metres, 1.0, 1e-9);
    CHECK_NEAR(uv_tiling(MaterialId::Markings).v_metres, 1.0, 1e-9);
    CHECK_NEAR(uv_tiling(MaterialId::Default).u_metres, 1.0, 1e-9);
    CHECK_NEAR(uv_tiling(MaterialId::Default).v_metres, 1.0, 1e-9);
    CHECK_NEAR(uv_tiling(MaterialId::Count).u_metres, 1.0, 1e-9);
    CHECK_NEAR(uv_tiling(MaterialId::Count).v_metres, 1.0, 1e-9);
}

// ============================================================================
// Texel density
// ============================================================================

TEST(Corridor, asphalt_v_advances_by_station_spacing_over_eight_metres) {
    const Centerline cl = straight_centerline(80.0);
    const RoadProfile profile = residential_profile();
    if (!cl.is_valid() || !profile.is_valid()) {
        stratum::test::report_failure(__FILE__, __LINE__, "fixture inputs are valid",
                                      "straight residential corridor could not be set up");
        return;
    }

    const Corridor corridor = build_corridor(cl, profile, CorridorConfig{});
    const Mesh& mesh = corridor.mesh;
    CHECK_TRUE(mesh.is_valid());
    if (!mesh.is_valid()) return;

    const std::vector<size_t> asphalt = vertices_of(mesh, MaterialId::Asphalt);
    CHECK_TRUE(!asphalt.empty());
    if (asphalt.empty()) return;

    std::vector<double> vs;
    vs.reserve(asphalt.size());
    for (size_t i : asphalt) vs.push_back(static_cast<double>(mesh.vertices[i].uv.y));
    const std::vector<double> rows = unique_sorted(vs, kUvEps);

    // One V row per station, and nothing in between.
    CHECK_EQ(rows.size(), cl.stations.size());
    if (rows.size() != cl.stations.size()) return;

    for (size_t i = 0; i < rows.size(); ++i) {
        // V = arclength / tile_v_metres(Asphalt)
        CHECK_NEAR(rows[i], cl.stations[i].arclength / kAsphaltTile, kUvEps);
    }
    for (size_t i = 1; i < rows.size(); ++i) {
        const double spacing = cl.stations[i].arclength - cl.stations[i - 1].arclength;
        CHECK_NEAR(rows[i] - rows[i - 1], spacing / kAsphaltTile, kUvEps);
    }

    // Stated once more without reference to the station list: an 80 m road at 8 m
    // spacing runs V from 0 to 10 in steps of exactly 1.
    CHECK_NEAR(rows.front(), 0.0, kUvEps);
    CHECK_NEAR(rows.back(), 80.0 / kAsphaltTile, kUvEps);
}

TEST(Corridor, a_lane_spans_its_width_over_eight_metres_in_u) {
    const Centerline cl = straight_centerline(80.0);
    const RoadProfile profile = residential_profile();
    const Corridor corridor = build_corridor(cl, profile, CorridorConfig{});
    const Mesh& mesh = corridor.mesh;
    if (!mesh.is_valid()) return;

    const std::vector<size_t> asphalt = vertices_of(mesh, MaterialId::Asphalt);
    if (asphalt.empty()) {
        stratum::test::report_failure(__FILE__, __LINE__, "corridor has asphalt vertices",
                                      "no vertex resolved to MaterialId::Asphalt");
        return;
    }

    double min_u = 1e300;
    double max_u = -1e300;
    for (size_t i : asphalt) {
        const double u = static_cast<double>(mesh.vertices[i].uv.x);
        min_u = std::min(min_u, u);
        max_u = std::max(max_u, u);
    }

    // U restarts at 0 on each strip's left edge, so the smallest U in the whole
    // material is exactly 0 and the largest is the widest strip over 8 m.
    CHECK_NEAR(min_u, 0.0, kUvEps);
    CHECK_NEAR(max_u, widest_lane(profile) / kAsphaltTile, kUvEps);
    CHECK_NEAR(widest_lane(profile), 3.5, 1e-5);
    CHECK_NEAR(max_u, 3.5 / 8.0, kUvEps);
}

TEST(Corridor, sidewalk_and_curb_carry_their_own_tiling) {
    const Centerline cl = straight_centerline(80.0);
    const ProfileConfig pcfg;
    const RoadProfile profile = residential_profile(pcfg);
    const Corridor corridor = build_corridor(cl, profile, CorridorConfig{});
    const Mesh& mesh = corridor.mesh;
    if (!mesh.is_valid()) return;

    // Sidewalk tiles every 2 m, so V is arclength / 2.
    for (size_t i : vertices_of(mesh, MaterialId::Sidewalk)) {
        const double v = static_cast<double>(mesh.vertices[i].uv.y);
        const double arclength = v * static_cast<double>(uv_tiling(MaterialId::Sidewalk).v_metres);
        bool matched = false;
        for (const Station& s : cl.stations) {
            if (std::fabs(s.arclength - arclength) <= 1e-3) { matched = true; break; }
        }
        if (!matched) {
            stratum::test::report_failure(__FILE__, __LINE__, "sidewalk V is arclength / 2",
                                          "V " + std::to_string(v) + " maps to arclength " +
                                              std::to_string(arclength) + ", which is no station");
            break;
        }
    }

    // Curb tiles every 2 m along the road and every 0.5 m up the face. A face
    // spanning one curb height therefore reaches U = curb_height / 0.5 at its top.
    const std::vector<size_t> curb = vertices_of(mesh, MaterialId::Curb);
    CHECK_TRUE(!curb.empty());
    bool face_top_found = false;
    for (size_t i : curb) {
        const double u = static_cast<double>(mesh.vertices[i].uv.x);
        const double v = static_cast<double>(mesh.vertices[i].uv.y);
        const double arclength = v * static_cast<double>(uv_tiling(MaterialId::Curb).v_metres);
        CHECK_TRUE(std::isfinite(u) && std::isfinite(v));
        CHECK_TRUE(arclength >= -1e-3 && arclength <= cl.length() + 1e-3);
        if (std::fabs(u - static_cast<double>(pcfg.curb_height) / 0.5) <= kUvEps) {
            face_top_found = true;
        }
    }
    CHECK_TRUE(face_top_found);
}

// ============================================================================
// Winding
// ============================================================================

TEST(Corridor, upward_facing_triangles_wind_counter_clockwise_seen_from_above) {
    const Centerline cl = straight_centerline(80.0);
    const RoadProfile profile = residential_profile();
    const Corridor corridor = build_corridor(cl, profile, CorridorConfig{});
    const Mesh& mesh = corridor.mesh;
    if (!mesh.is_valid() || mesh.indices.empty()) {
        stratum::test::report_failure(__FILE__, __LINE__, "corridor produced triangles",
                                      "straight residential corridor is empty");
        return;
    }

    CHECK_EQ(mesh.indices.size() % 3u, size_t{0});

    size_t upward = 0;
    size_t inverted = 0;
    size_t degenerate = 0;
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const uint32_t i0 = mesh.indices[t];
        const uint32_t i1 = mesh.indices[t + 1];
        const uint32_t i2 = mesh.indices[t + 2];
        if (i0 >= mesh.vertices.size() || i1 >= mesh.vertices.size() ||
            i2 >= mesh.vertices.size()) {
            stratum::test::report_failure(__FILE__, __LINE__, "index is in range",
                                          "triangle " + std::to_string(t / 3));
            return;
        }

        const Vertex& a = mesh.vertices[i0];
        const Vertex& b = mesh.vertices[i1];
        const Vertex& c = mesh.vertices[i2];
        const glm::vec3 geometric = glm::cross(b.position - a.position, c.position - a.position);

        // Zero-area triangles are dropped, not emitted.
        if (glm::length(geometric) <= 1e-12f) ++degenerate;

        const bool all_up = a.normal.y > 0.5f && b.normal.y > 0.5f && c.normal.y > 0.5f;
        if (!all_up) continue;
        ++upward;
        if (geometric.y <= 0.0f) ++inverted;
    }

    CHECK_TRUE(upward > 0);
    CHECK_EQ(inverted, size_t{0});
    CHECK_EQ(degenerate, size_t{0});
}

TEST(Corridor, every_vertex_normal_is_unit_length_and_colour_is_opaque_white) {
    const Centerline cl = straight_centerline(80.0);
    const RoadProfile profile = residential_profile();
    const Corridor corridor = build_corridor(cl, profile, CorridorConfig{});
    const Mesh& mesh = corridor.mesh;
    if (!mesh.is_valid()) return;

    for (const Vertex& v : mesh.vertices) {
        CHECK_TRUE(is_finite(v.position));
        CHECK_NEAR(glm::length(v.normal), 1.0, 1e-4);
        // Appearance comes from the bound material, never from baked vertex colour.
        CHECK_NEAR(v.color.r, 1.0, 1e-6);
        CHECK_NEAR(v.color.g, 1.0, 1e-6);
        CHECK_NEAR(v.color.b, 1.0, 1e-6);
        CHECK_NEAR(v.color.a, 1.0, 1e-6);
    }
}

// ============================================================================
// Submeshes and the weld rule
// ============================================================================

TEST(Corridor, submeshes_tile_the_index_buffer_with_each_material_once) {
    const Centerline cl = straight_centerline(80.0);
    const RoadProfile profile = residential_profile();
    const Corridor corridor = build_corridor(cl, profile, CorridorConfig{});

    check_submeshes_tile(corridor.mesh, "straight residential");

    // A residential road carries at least asphalt, curb and sidewalk.
    CHECK_TRUE(corridor.mesh.submeshes.size() >= size_t{3});
    CHECK_TRUE(!vertices_of(corridor.mesh, MaterialId::Asphalt).empty());
    CHECK_TRUE(!vertices_of(corridor.mesh, MaterialId::Curb).empty());
    CHECK_TRUE(!vertices_of(corridor.mesh, MaterialId::Sidewalk).empty());

    // No vertex is orphaned by the ranges.
    for (MaterialId m : vertex_materials(corridor.mesh)) {
        CHECK_TRUE(m != kNoMaterial);
    }
}

TEST(Corridor, vertex_count_is_two_columns_per_strip_per_station) {
    const Centerline cl = straight_centerline(80.0);
    const RoadProfile profile = residential_profile();
    if (!cl.is_valid() || !profile.is_valid()) return;

    CorridorConfig cfg;
    const Corridor corridor = build_corridor(cl, profile, cfg);

    // Each strip owns one left and one right column, and a column carries one
    // vertex per station. Anything larger means the extruder emitted per-quad
    // vertices instead of welding along the ribbon.
    const size_t expected = 2 * emitted_strip_count(profile, cfg) * cl.stations.size();
    CHECK_EQ(corridor.mesh.vertices.size(), expected);

    // Two triangles per strip per band, and there is one band fewer than stations.
    const size_t bands = cl.stations.size() - 1;
    const size_t expected_triangles = 2 * emitted_strip_count(profile, cfg) * bands;
    CHECK_EQ(corridor.mesh.indices.size(), expected_triangles * 3);

    CHECK_NEAR(corridor.length, cl.length(), 1e-9);
    CHECK_TRUE(corridor.mesh.bounds.is_valid());
}

TEST(Corridor, disabling_curb_faces_drops_exactly_those_strips) {
    const Centerline cl = straight_centerline(80.0);
    const RoadProfile profile = residential_profile();
    if (!cl.is_valid() || !profile.is_valid()) return;

    CorridorConfig with;
    CorridorConfig without;
    without.emit_curb_faces = false;

    const Corridor a = build_corridor(cl, profile, with);
    const Corridor b = build_corridor(cl, profile, without);

    CHECK_TRUE(b.mesh.vertices.size() < a.mesh.vertices.size());
    CHECK_EQ(b.mesh.vertices.size(), 2 * emitted_strip_count(profile, without) * cl.stations.size());
    check_submeshes_tile(b.mesh, "residential without curb faces");
}

// ============================================================================
// Outline
// ============================================================================

TEST(Corridor, outline_is_a_closed_ccw_ring_the_width_of_the_profile) {
    const Centerline cl = straight_centerline(80.0);
    const RoadProfile profile = residential_profile();
    if (!cl.is_valid() || !profile.is_valid()) return;

    const Corridor corridor = build_corridor(cl, profile, CorridorConfig{});
    const std::vector<glm::dvec2>& ring = corridor.outline;

    CHECK_TRUE(ring.size() >= size_t{4});
    if (ring.size() < 4) return;

    // Right edge from first station to last, then back along the left edge.
    CHECK_EQ(ring.size(), 2 * cl.stations.size());

    // The first point is not repeated at the end: the ring is closed implicitly.
    CHECK_TRUE(glm::length(ring.front() - ring.back()) > 1e-6);

    // Non-zero area, wound counter-clockwise in the 2D plane.
    const double area = signed_area(ring);
    CHECK_TRUE(std::fabs(area) > 1e-6);
    CHECK_TRUE(area > 0.0);

    // A straight road's footprint is a rectangle as long as the road and as wide
    // as the whole profile.
    // Tolerance is float-storage limited: the widths are floats, so a 12 m wide
    // profile carries about 1e-5 m of rounding over an 80 m road.
    CHECK_NEAR(std::fabs(area), 80.0 * static_cast<double>(profile.total_width()), 1e-2);

    const double left = static_cast<double>(profile.left_edge_offset());
    const double right = left - static_cast<double>(profile.total_width());
    double min_y = 1e300;
    double max_y = -1e300;
    for (const glm::dvec2& p : ring) {
        CHECK_TRUE(std::isfinite(p.x) && std::isfinite(p.y));
        min_y = std::min(min_y, p.y);
        max_y = std::max(max_y, p.y);
    }
    CHECK_NEAR(max_y, left, 1e-5);
    CHECK_NEAR(min_y, right, 1e-5);

    // A straight ribbon cannot fold back on itself.
    CHECK_FALSE(corridor.outline_self_intersects);
}

TEST(Corridor, outline_is_suppressed_when_the_caller_turns_it_off) {
    const Centerline cl = straight_centerline(40.0);
    const RoadProfile profile = residential_profile();

    CorridorConfig cfg;
    cfg.emit_outline = false;
    const Corridor corridor = build_corridor(cl, profile, cfg);

    CHECK_TRUE(corridor.outline.empty());
    CHECK_FALSE(corridor.outline_self_intersects);
    // Turning the outline off must not change the geometry.
    CHECK_TRUE(corridor.mesh.is_valid());
}

// ============================================================================
// Elevation
// ============================================================================

TEST(Corridor, station_heights_drive_the_carriageway_y) {
    const Centerline cl = straight_centerline(80.0);
    const RoadProfile profile = residential_profile();
    if (!cl.is_valid() || !profile.is_valid()) return;

    // A 0.5 m rise per station: the road climbs a constant ramp.
    CorridorConfig cfg;
    cfg.station_heights.resize(cl.stations.size());
    for (size_t i = 0; i < cl.stations.size(); ++i) {
        cfg.station_heights[i] = static_cast<float>(1.0 + 0.5 * static_cast<double>(i));
    }

    const Corridor corridor = build_corridor(cl, profile, cfg);
    const Mesh& mesh = corridor.mesh;
    if (!mesh.is_valid()) return;

    // Asphalt sits on the carriageway surface itself, height offset 0, so its Y is
    // the station height exactly.
    for (size_t i : vertices_of(mesh, MaterialId::Asphalt)) {
        const Vertex& v = mesh.vertices[i];
        const double arclength = static_cast<double>(v.uv.y) * kAsphaltTile;

        size_t nearest = 0;
        double best = 1e300;
        for (size_t s = 0; s < cl.stations.size(); ++s) {
            const double d = std::fabs(cl.stations[s].arclength - arclength);
            if (d < best) { best = d; nearest = s; }
        }
        if (best > 1e-2) {
            stratum::test::report_failure(__FILE__, __LINE__, "vertex V maps to a station",
                                          "arclength " + std::to_string(arclength));
            break;
        }
        if (std::fabs(static_cast<double>(v.position.y) -
                      static_cast<double>(cfg.station_heights[nearest])) > 1e-4) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "asphalt Y follows the station height ramp",
                "station " + std::to_string(nearest) + ": Y " + std::to_string(v.position.y) +
                    " expected " + std::to_string(cfg.station_heights[nearest]));
            break;
        }
    }

    // The ramp spans 1.0 m to 6.0 m over eleven stations, and the sidewalks add a
    // curb height on top.
    CHECK_NEAR(mesh.bounds.min.y, 1.0, 1e-4);
    CHECK_TRUE(mesh.bounds.max.y >= 6.0f - 1e-4f);
}

TEST(Corridor, an_empty_or_mis_sized_height_vector_leaves_the_road_flat) {
    const Centerline cl = straight_centerline(80.0);
    const RoadProfile profile = residential_profile();
    if (!cl.is_valid() || !profile.is_valid()) return;

    CorridorConfig flat;
    const Corridor a = build_corridor(cl, profile, flat);
    if (!a.mesh.is_valid()) return;
    for (size_t i : vertices_of(a.mesh, MaterialId::Asphalt)) {
        CHECK_NEAR(a.mesh.vertices[i].position.y, flat.base_height, 1e-6);
    }

    // A mis-sized vector is a caller error and degrades to flat rather than to
    // mangled geometry.
    CorridorConfig wrong;
    wrong.station_heights.assign(cl.stations.size() + 3, 42.0f);
    const Corridor b = build_corridor(cl, profile, wrong);
    CHECK_EQ(b.mesh.vertices.size(), a.mesh.vertices.size());
    for (size_t i : vertices_of(b.mesh, MaterialId::Asphalt)) {
        CHECK_NEAR(b.mesh.vertices[i].position.y, wrong.base_height, 1e-6);
    }
}

// ============================================================================
// World mapping
// ============================================================================

TEST(Corridor, the_two_d_to_three_d_mapping_is_x_height_minus_y) {
    // A +X road with the profile centred on the way: the left edge of the profile
    // is at lateral +left_edge_offset in 2D, which is -left_edge_offset in world Z.
    const Centerline cl = straight_centerline(80.0);
    const RoadProfile profile = residential_profile();
    if (!cl.is_valid() || !profile.is_valid()) return;

    const Corridor corridor = build_corridor(cl, profile, CorridorConfig{});
    const Mesh& mesh = corridor.mesh;
    if (!mesh.is_valid()) return;

    const double left = static_cast<double>(profile.left_edge_offset());
    const double right = left - static_cast<double>(profile.total_width());

    CHECK_NEAR(mesh.bounds.min.x, 0.0, 1e-4);
    CHECK_NEAR(mesh.bounds.max.x, 80.0, 1e-4);
    CHECK_NEAR(mesh.bounds.min.z, -left, 1e-4);
    CHECK_NEAR(mesh.bounds.max.z, -right, 1e-4);
}

// ============================================================================
// Degenerate input
// ============================================================================

TEST(Corridor, invalid_inputs_produce_an_empty_corridor) {
    const RoadProfile profile = residential_profile();
    const Centerline good = straight_centerline(40.0);

    // Fewer than two stations: no band to extrude.
    Centerline one_station;
    one_station.stations.push_back(Station{});
    const Corridor a = build_corridor(one_station, profile, CorridorConfig{});
    CHECK_TRUE(a.mesh.vertices.empty());
    CHECK_TRUE(a.mesh.indices.empty());
    CHECK_TRUE(a.outline.empty());
    CHECK_NEAR(a.length, 0.0, 1e-9);

    const Corridor b = build_corridor(Centerline{}, profile, CorridorConfig{});
    CHECK_TRUE(b.mesh.vertices.empty());
    CHECK_TRUE(b.outline.empty());

    // Empty profile: nothing to sweep.
    const Corridor c = build_corridor(good, RoadProfile{}, CorridorConfig{});
    CHECK_TRUE(c.mesh.vertices.empty());
    CHECK_TRUE(c.mesh.indices.empty());
    CHECK_TRUE(c.outline.empty());

    // Invalid profile: a step with no riser must be skipped, not folded.
    RoadProfile stepped;
    stepped.strips.push_back(Strip{3.5f, 0.0f, 0.0f, MaterialId::Asphalt, StripKind::Lane});
    stepped.strips.push_back(Strip{2.0f, 0.4f, 0.4f, MaterialId::Sidewalk, StripKind::Sidewalk});
    CHECK_FALSE(stepped.is_valid());
    const Corridor d = build_corridor(good, stepped, CorridorConfig{});
    CHECK_TRUE(d.mesh.vertices.empty());
    CHECK_TRUE(d.outline.empty());

    // Both invalid at once.
    const Corridor e = build_corridor(Centerline{}, RoadProfile{}, CorridorConfig{});
    CHECK_TRUE(e.mesh.vertices.empty());
    CHECK_TRUE(e.outline.empty());
    CHECK_FALSE(e.outline_self_intersects);
}

TEST(Corridor, a_mitred_corner_keeps_the_profile_width_through_the_joint) {
    // The corridor equivalent of the centerline miter test: a right-angle corner
    // must stay full width at the joint instead of pinching to width * cos(45).
    ResampleConfig rcfg;
    rcfg.smooth = false;
    const Centerline cl =
        build_centerline({{-40.0, 0.0}, {0.0, 0.0}, {0.0, 40.0}}, rcfg);
    const RoadProfile profile = residential_profile();
    if (!cl.is_valid() || !profile.is_valid()) return;

    const Corridor corridor = build_corridor(cl, profile, CorridorConfig{});
    const Mesh& mesh = corridor.mesh;
    CHECK_TRUE(mesh.is_valid());
    if (!mesh.is_valid()) return;

    for (const Vertex& v : mesh.vertices) {
        CHECK_TRUE(is_finite(v.position));
    }
    check_submeshes_tile(mesh, "right-angle residential");

    // The corner station's outer offset must reach the full half width away from
    // both legs. In world space the outer corner of a left turn is at
    // +total_width_beyond_the_way in +X and -Z.
    const double right_edge = static_cast<double>(profile.left_edge_offset()) -
                              static_cast<double>(profile.total_width());
    // right_edge is negative, so the outer corner sits at x = -right_edge.
    CHECK_NEAR(mesh.bounds.max.x, -right_edge, 1e-3);
    CHECK_NEAR(mesh.bounds.max.z, -right_edge, 1e-3);
}

// ============================================================================
// The fold guard - no inverted triangles at a sharp joint
// ============================================================================

namespace {

/**
 * @brief A profile built for one road class, with sidewalks on both sides
 *
 * @param type  Road class the profile rules are taken from
 * @param lanes Running lane count
 * @param width Surveyed carriageway width in metres, 0 for none
 * @return The profile
 */
RoadProfile profile_for(RoadType type, int lanes, float width) {
    GraphEdge edge;
    edge.source_way = 1;
    edge.node_ids = {1, 2};
    edge.type = type;
    edge.lanes = lanes;
    edge.width = width;
    edge.sidewalk = SideFlags::Both;
    return build_profile(edge, ProfileConfig{});
}

/**
 * @brief Triangles of a horizontal strip that wind clockwise seen from above
 *
 * The renderer culls back faces with a counter-clockwise front face, so a
 * road-surface triangle wound the other way is not a shading artefact: it is a
 * hole, with the surrounding surface overlapping itself where the hole is.
 *
 * Only triangles whose three vertices share one world Y are measured, which is
 * exactly the horizontal strips -- carriageway, gutter, sidewalk, verge, curb
 * top. Their outward normal is +Y by construction, so the test is the sign of
 * the face cross product's Y and needs nothing from the stored vertex normals,
 * which are themselves averaged over neighbouring faces and would make the check
 * circular. A vertical curb face never has three vertices at one height and is
 * skipped.
 *
 * @param mesh Mesh to walk
 * @return Number of horizontal triangles wound clockwise from above
 */
size_t inverted_triangle_count(const Mesh& mesh) {
    size_t inverted = 0;
    for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
        const uint32_t i0 = mesh.indices[t];
        const uint32_t i1 = mesh.indices[t + 1];
        const uint32_t i2 = mesh.indices[t + 2];
        if (i0 >= mesh.vertices.size() || i1 >= mesh.vertices.size() ||
            i2 >= mesh.vertices.size()) {
            return inverted + 1;
        }

        const glm::vec3& a = mesh.vertices[i0].position;
        const glm::vec3& b = mesh.vertices[i1].position;
        const glm::vec3& c = mesh.vertices[i2].position;
        if (std::fabs(b.y - a.y) > 1e-6f || std::fabs(c.y - a.y) > 1e-6f) continue;

        // The cross product is twice the signed area, so this is one square
        // millimetre of slack. Where the fold guard collapses a band the triangles
        // either vanish or survive as slivers this far below any renderable size;
        // a real inversion is square metres, four orders of magnitude past it.
        if (glm::cross(b - a, c - a).y < -2e-6f) ++inverted;
    }
    return inverted;
}

/**
 * @brief Build one corridor from a raw polyline and the default resampling
 *
 * @param poly    Centerline in 2D local metres
 * @param profile Cross-section to sweep
 * @return The corridor
 */
Corridor corridor_from(const std::vector<glm::dvec2>& poly, const RoadProfile& profile) {
    return build_corridor(build_centerline(poly, ResampleConfig{}), profile, CorridorConfig{});
}

/// A polyline turning by @p degrees at the origin, with legs of @p leg metres
std::vector<glm::dvec2> bend(double degrees, double leg) {
    const double turn = degrees * 3.14159265358979323846 / 180.0;
    return {{-leg, 0.0}, {0.0, 0.0},
            {-leg * std::cos(turn), leg * std::sin(turn)}};
}

} // namespace

TEST(Corridor, a_sharp_corner_emits_no_inverted_triangles) {
    // The extruder applied miter_scale to every column with no bound, so at a
    // sharp joint the INNER columns overshot the neighbouring station and the
    // band inverted: triangles wound clockwise from above, culled as backfaces,
    // leaving holes while the surrounding surface overlapped itself. Detection
    // existed for the outline only, and nothing acted on it.
    const RoadProfile residential = profile_for(RoadType::Residential, 2, 0.0f);
    const RoadProfile motorway = profile_for(RoadType::Motorway, 2, 0.0f);
    CHECK_TRUE(residential.is_valid());
    CHECK_TRUE(motorway.is_valid());

    for (double degrees : {60.0, 80.0, 95.0, 100.0, 120.0, 150.0, 170.0}) {
        for (double leg : {20.0, 30.0, 40.0}) {
            const Corridor r = corridor_from(bend(degrees, leg), residential);
            CHECK_EQ(inverted_triangle_count(r.mesh), size_t{0});

            const Corridor m = corridor_from(bend(degrees, leg), motorway);
            CHECK_EQ(inverted_triangle_count(m.mesh), size_t{0});
        }
    }
}

TEST(Corridor, a_bevelled_hairpin_emits_no_inverted_triangles) {
    // Past the miter limit the joint bevels into two coincident stations, and the
    // zero-length wedge band between them inverted on the INSIDE of the turn: the
    // outer half was a correct wedge, the inner half was a bowtie. Dropping
    // zero-area triangles never covered it, because only the centreline column is
    // zero-area across a bevel; every off-centre strip had real inverted area.
    const RoadProfile residential = profile_for(RoadType::Residential, 2, 0.0f);
    if (!residential.is_valid()) return;

    const std::vector<std::vector<glm::dvec2>> hairpins = {
        {{0.0, 0.0}, {100.0, 0.0}, {0.0, 1.0}},                 // 179 degrees
        {{0.0, 0.0}, {100.0, 0.0}, {0.0, -1.0}},                // 179 degrees, other way
        {{0.0, 0.0}, {60.0, 0.0}, {4.5, 20.5}},                 // about 160 degrees
        {{0.0, 0.0}, {60.0, 0.0}, {60.0, 10.0}, {0.0, 10.0}},   // switchback
    };

    for (const std::vector<glm::dvec2>& poly : hairpins) {
        const Corridor c = corridor_from(poly, residential);
        CHECK_TRUE(c.mesh.is_valid());
        CHECK_EQ(inverted_triangle_count(c.mesh), size_t{0});
        for (const Vertex& v : c.mesh.vertices) CHECK_TRUE(is_finite(v.position));
    }
}

TEST(Corridor, a_sharp_corner_no_longer_reports_a_self_intersecting_outline) {
    // The footprint the P3 terrain carve runs a point-in-polygon test against is
    // the same offset the mesh is built from, so bounding one bounds the other. A
    // bend a 12 m profile has room to mitre through now produces a simple ring and
    // no warning. A hairpin whose two arms genuinely overlap in space still
    // reports, correctly: no local bound can make that footprint simple.
    const RoadProfile residential = profile_for(RoadType::Residential, 2, 0.0f);
    if (!residential.is_valid()) return;

    for (double degrees : {60.0, 95.0, 100.0, 120.0}) {
        const Corridor c = corridor_from(bend(degrees, 30.0), residential);
        CHECK_FALSE(c.outline_self_intersects);
        CHECK_TRUE(c.outline.size() >= 3);
    }
}

TEST(Corridor, the_fold_guard_leaves_a_slack_corner_at_full_width) {
    // The guard must cost nothing where the corner had room to mitre. A right
    // angle on 40 m legs resampled at 8 m allows 8 m of inner lateral, so a 5.97 m
    // half width still reaches its full mitred offset on both sides and the
    // corridor keeps the width the un-guarded extruder gave it.
    const RoadProfile profile = residential_profile();
    const Centerline cl = build_centerline({{-40.0, 0.0}, {0.0, 0.0}, {0.0, 40.0}},
                                           ResampleConfig{});
    if (!cl.is_valid() || !profile.is_valid()) return;

    const Corridor corridor = build_corridor(cl, profile, CorridorConfig{});
    const double half = 0.5 * static_cast<double>(profile.total_width());

    // The outer corner of the left turn is the extreme of the whole corridor, and
    // a full mitred offset puts it exactly half a profile width beyond the way in
    // both axes. A guard that bit here would pull it in.
    const double right_edge = static_cast<double>(profile.left_edge_offset()) -
                              static_cast<double>(profile.total_width());
    CHECK_NEAR(corridor.mesh.bounds.max.x, -right_edge, 1e-3);
    CHECK_NEAR(corridor.mesh.bounds.max.z, -right_edge, 1e-3);
    CHECK_NEAR(half, -right_edge, 1e-3);
    CHECK_EQ(inverted_triangle_count(corridor.mesh), size_t{0});
}
