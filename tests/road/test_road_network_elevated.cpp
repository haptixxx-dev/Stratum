/**
 * @file test_road_network_elevated.cpp
 * @brief End-to-end RoadNetworkBuilder tests with a terrain sampler attached
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * P3 adds one field to RoadNetworkConfig -- height_sampler -- and everything
 * else about the road pipeline is meant to be unchanged. These tests assert both
 * halves of that claim over the .osm fixtures, using a SYNTHETIC surface so no
 * procgen header is involved.
 *
 * The second test is the one worth keeping. `height_sampler` defaults to null,
 * and a null sampler must leave the network exactly as P2 built it: flat at
 * CorridorConfig::base_height, no elevation solve, no carve payload. It is very
 * easy for a vertical solve to leak into the flat path -- an unconditional
 * resample, a station_heights vector that is sized rather than left empty, a
 * base height read from the wrong config -- and the symptom is that every
 * existing test still passes while the shipped flat-terrain output quietly moves.
 * Comparing a null-sampler build against a constant-sampler build catches that:
 * a constant surface may translate the road vertically and may do nothing else,
 * so X and Z must come back bit-identical and Y must differ by exactly one
 * constant everywhere.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests RoadNetworkElevated
 * @endcode
 */

#include "framework.hpp"
#include "road/junction_fixtures.hpp"

#include "osm/parser.hpp"
#include "osm/road/road_elevation.hpp"
#include "osm/road/road_graph.hpp"
#include "osm/road/junction_special.hpp"
#include "osm/road/road_network_builder.hpp"
#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#ifndef STRATUM_TEST_DATA_DIR
#error "STRATUM_TEST_DATA_DIR must be defined by the build; see tests/CMakeLists.txt"
#endif

namespace {

using stratum::Mesh;
using stratum::Vertex;
using stratum::osm::ParsedOSMData;
using stratum::osm::road::EdgeElevation;
using stratum::osm::road::ElevationConfig;
using stratum::osm::road::HeightSampler;
using stratum::osm::road::RoadGraph;
using stratum::osm::road::RoadNetwork;
using stratum::osm::road::RoadNetworkBuilder;
using stratum::osm::road::RoadNetworkConfig;
using stratum::osm::road::RoadPiece;
using stratum::osm::road::kInvalidId;
using stratum::osm::road::Centerline;
using stratum::osm::road::ResampleConfig;
using stratum::osm::road::apply_junction_plateaus;
using stratum::osm::road::build_centerline;
using stratum::osm::Road;
using stratum::osm::RoadType;

/// One road with explicit node identity, as the graph builder wants it
Road make_road(stratum::osm::WayId way_id,
               const std::vector<stratum::osm::NodeId>& node_ids,
               const std::vector<glm::dvec2>& points,
               RoadType type) {
    Road road;
    road.osm_id = way_id;
    road.polyline = points;
    road.node_ids = node_ids;
    road.type = type;
    road.lanes = 2;
    road.width = 7.0f;
    return road;
}

/// Wrap roads into a ParsedOSMData the builder will accept
ParsedOSMData make_data(std::vector<Road> roads) {
    ParsedOSMData data;
    data.roads = std::move(roads);
    data.stats.processed_roads = data.roads.size();
    return data;
}

/// A flat surface at 10 m, so every solved height is easy to read by eye
float flat_ten(double, double) {
    return 10.0f;
}

/**
 * @brief Height at an arclength, interpolated exactly as the reslice does it
 *
 * The corridor is built on the TRIMMED centerline, whose first station sits at
 * arclength `trim` on the untrimmed one, and its height comes from interpolating
 * the untrimmed profile there. So this is the number that decides whether the
 * ribbon mouth meets the junction plane.
 */
double height_at(const Centerline& cl, const std::vector<float>& heights, double arc) {
    if (heights.size() != cl.stations.size() || heights.empty()) return 0.0;
    for (size_t j = 1; j < cl.stations.size(); ++j) {
        const double a = cl.stations[j - 1].arclength;
        const double b = cl.stations[j].arclength;
        if (arc <= b || j + 1 == cl.stations.size()) {
            const double span = b - a;
            const double t = span > 1e-12 ? (arc - a) / span : 0.0;
            return static_cast<double>(heights[j - 1])
                 + (static_cast<double>(heights[j]) - static_cast<double>(heights[j - 1])) * t;
        }
    }
    return static_cast<double>(heights.back());
}

/// Fixtures with enough length in both axes for a tilted surface to show
constexpr const char* kFixtures[] = {"four_way.osm", "t_junction.osm"};

/// Absolute path of a fixture in tests/data
std::filesystem::path fixture_path(const char* filename) {
    return std::filesystem::path(STRATUM_TEST_DATA_DIR) / filename;
}

/**
 * @brief Parse one fixture with roads only
 *
 * @param filename Fixture file name in tests/data
 * @return Parsed data, or std::nullopt when the parse failed
 */
std::optional<ParsedOSMData> parse_fixture(const char* filename) {
    const auto path = fixture_path(filename);
    if (!std::filesystem::exists(path)) {
        stratum::test::report_failure(__FILE__, __LINE__, "fixture exists",
                                      "missing: " + path.string());
        return std::nullopt;
    }

    stratum::osm::OSMParser parser;
    stratum::osm::ParserConfig config;
    config.import_buildings = false;
    config.import_water = false;
    config.import_landuse = false;
    config.import_natural = false;
    config.simplify_geometry = false;
    parser.set_config(config);

    if (!parser.parse(path)) {
        stratum::test::report_failure(__FILE__, __LINE__, "parser.parse(fixture)",
                                      path.string() + ": " + parser.get_error());
        return std::nullopt;
    }
    return parser.take_data();
}

/**
 * @brief A tilted plane with a slow undulation, in 2D local metres
 *
 * The gradients here -- 4% along x, 3% along y -- are inside every class limit
 * in the table, so the solved road follows this surface rather than departing
 * from it, and the piece geometry has to move with it.
 */
float tilted_surface(double x, double y) {
    return static_cast<float>(40.0 + 0.04 * x - 0.03 * y
                              + 1.5 * std::sin(0.02 * x) * std::cos(0.02 * y));
}

/// A perfectly flat surface at 250 m, which may translate the network and nothing else
float constant_surface(double, double) {
    return 250.0f;
}

/// True when every component of a world position is a finite number
bool is_finite(const glm::vec3& v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

/// Number of graph nodes of degree 3 or more
size_t junction_count(const RoadGraph& graph) {
    size_t n = 0;
    for (const auto& node : graph.nodes()) {
        if (node.is_junction()) ++n;
    }
    return n;
}

/// Pieces built from one graph edge, as opposed to from a solved junction
size_t edge_piece_count(const RoadNetwork& network) {
    size_t n = 0;
    for (const RoadPiece& piece : network.pieces) {
        if (piece.edge != stratum::osm::road::kInvalidId) ++n;
    }
    return n;
}

/**
 * @brief Junctions that contributed their own CarveDisc on top of the per-node ones
 *
 * The per-node pass emits one disc for every node of degree 3 or more. Three
 * other kinds carve ground no CarveRibbon covers and are emitted from the
 * junction list instead: a roundabout, whose ring edges emit no ribbon at all; a
 * degree-2 profile taper, whose two ribbons are trimmed back off the wedge; and a
 * degree-1 bulb or turning circle, which reaches past the ribbon's last station.
 * A flat cap adds no footprint and therefore no disc.
 */
size_t extra_disc_count(const RoadNetwork& network) {
    size_t n = 0;
    for (const auto& junction : network.junctions) {
        using stratum::osm::road::JunctionKind;
        if (junction.footprint.empty()) continue;
        if (junction.kind == JunctionKind::Roundabout) {
            ++n;
        } else if (junction.valid &&
                   (junction.kind == JunctionKind::Taper ||
                    junction.kind == JunctionKind::DeadEnd)) {
            ++n;
        }
    }
    return n;
}

} // namespace

// ============================================================================
// Built on terrain
// ============================================================================

TEST(RoadNetworkElevated, piece_geometry_follows_the_solved_profile) {
    for (const char* fixture : kFixtures) {
        auto parsed = parse_fixture(fixture);
        if (!parsed) continue;

        RoadNetworkConfig cfg;
        cfg.height_sampler = HeightSampler{tilted_surface};

        RoadNetworkBuilder builder;
        const RoadNetwork network = builder.build(*parsed, cfg);

        const std::string label{fixture};

        CHECK_TRUE(!network.pieces.empty());
        if (network.pieces.empty()) continue;

        CHECK_TRUE(builder.elevation().is_solved());
        CHECK_TRUE(network.stats.elevated_edges > size_t{0});
        CHECK_TRUE(network.stats.elevation_ms >= 0.0);

        // The carve payload comes back filled and parallel to the EDGE pieces,
        // which is what lets a chunk carve find the corridor that crosses it. P4
        // appends junction pieces after them; a junction carries a disc, not a
        // ribbon.
        CHECK_EQ(network.carve_ribbons.size(), edge_piece_count(network));
        CHECK_EQ(network.carve_discs.size(),
                 junction_count(builder.graph()) + extra_disc_count(network));

        size_t non_finite_vertices = 0;
        size_t off_profile_vertices = 0;
        size_t flat_pieces = 0;
        size_t checked_pieces = 0;
        float network_min_y = 0.0f;
        float network_max_y = 0.0f;
        bool have_y = false;

        for (const RoadPiece& piece : network.pieces) {
            CHECK_TRUE(!piece.mesh.vertices.empty());
            if (piece.mesh.vertices.empty()) continue;

            float piece_min_y = piece.mesh.vertices.front().position.y;
            float piece_max_y = piece_min_y;

            for (const Vertex& v : piece.mesh.vertices) {
                if (!is_finite(v.position) || !is_finite(v.normal)) ++non_finite_vertices;
                piece_min_y = std::min(piece_min_y, v.position.y);
                piece_max_y = std::max(piece_max_y, v.position.y);
            }

            if (!have_y) {
                network_min_y = piece_min_y;
                network_max_y = piece_max_y;
                have_y = true;
            } else {
                network_min_y = std::min(network_min_y, piece_min_y);
                network_max_y = std::max(network_max_y, piece_max_y);
            }

            if (piece.edge == kInvalidId) continue;

            const EdgeElevation& elev = builder.elevation().edge(piece.edge);
            if (elev.station_heights.empty()) continue;
            ++checked_pieces;

            float solved_min = elev.station_heights.front();
            float solved_max = solved_min;
            for (float h : elev.station_heights) {
                solved_min = std::min(solved_min, h);
                solved_max = std::max(solved_max, h);
            }

            // Every vertex sits inside the solved band, allowing for the strips
            // that stand above the carriageway surface -- a curb top and a
            // sidewalk at +0.15 m on this profile.
            if (piece_min_y < solved_min - 0.1f || piece_max_y > solved_max + 0.5f) {
                ++off_profile_vertices;
                stratum::test::report_failure(
                    __FILE__, __LINE__, "piece Y range inside solved profile band",
                    label + " edge " + std::to_string(piece.edge)
                        + " mesh Y [" + stratum::test::stringify(piece_min_y) + ", "
                        + stratum::test::stringify(piece_max_y) + "] solved ["
                        + stratum::test::stringify(solved_min) + ", "
                        + stratum::test::stringify(solved_max) + "]");
            }

            // A piece whose Y never moves is a flat P2 ribbon that the solve
            // failed to reach. The arms of these fixtures are tens of metres
            // long on a 4% slope, so each must climb by well over a decimetre.
            if (piece_max_y - piece_min_y < 0.3f) ++flat_pieces;
        }

        CHECK_EQ(non_finite_vertices, size_t{0});
        CHECK_EQ(off_profile_vertices, size_t{0});
        CHECK_TRUE(checked_pieces > size_t{0});
        CHECK_EQ(flat_pieces, size_t{0});

        // And the network as a whole sits on the surface rather than at the flat
        // base height, which is 0.05 m.
        CHECK_TRUE(have_y);
        CHECK_TRUE(network_min_y > 20.0f);
        CHECK_TRUE(network_max_y - network_min_y > 1.0f);
    }
}

TEST(RoadNetworkElevated, carve_ribbons_describe_the_pieces_they_came_from) {
    auto parsed = parse_fixture("four_way.osm");
    if (!parsed) return;

    RoadNetworkConfig cfg;
    cfg.height_sampler = HeightSampler{tilted_surface};

    RoadNetworkBuilder builder;
    const RoadNetwork network = builder.build(*parsed, cfg);

    CHECK_EQ(network.carve_ribbons.size(), edge_piece_count(network));
    if (network.carve_ribbons.size() != edge_piece_count(network)) return;

    size_t bad_ribbons = 0;
    for (size_t i = 0; i < network.carve_ribbons.size(); ++i) {
        const auto& ribbon = network.carve_ribbons[i];

        // Heights are parallel to the centerline or the carve cannot read them.
        if (ribbon.centerline.size() != ribbon.centerline_heights.size()) ++bad_ribbons;
        if (ribbon.centerline.size() < 2) ++bad_ribbons;
        if (!(ribbon.half_width > 0.0f)) ++bad_ribbons;

        for (const glm::dvec2& p : ribbon.centerline) {
            if (!std::isfinite(p.x) || !std::isfinite(p.y)) ++bad_ribbons;
        }
        for (float h : ribbon.centerline_heights) {
            if (!std::isfinite(h)) ++bad_ribbons;
        }
        if (ribbon.outline.empty() && ribbon.outline_is_simple) ++bad_ribbons;
    }
    CHECK_EQ(bad_ribbons, size_t{0});

    size_t bad_discs = 0;
    for (const auto& disc : network.carve_discs) {
        if (!(disc.radius > 0.0f)) ++bad_discs;
        if (!std::isfinite(disc.height)) ++bad_discs;
        if (!std::isfinite(disc.center.x) || !std::isfinite(disc.center.y)) ++bad_discs;
    }
    CHECK_EQ(bad_discs, size_t{0});

    // One per node of degree 3 or more -- four_way.osm has exactly one -- plus
    // whatever the taper, dead-end and roundabout paths added.
    CHECK_EQ(network.carve_discs.size(), size_t{1} + extra_disc_count(network));
    CHECK_EQ(junction_count(builder.graph()), size_t{1});
}

// ============================================================================
// Flat-mode regression guard
// ============================================================================

TEST(RoadNetworkElevated, null_sampler_reproduces_the_flat_p2_network) {
    auto parsed = parse_fixture("four_way.osm");
    if (!parsed) return;

    const RoadNetworkConfig flat_cfg;   // height_sampler defaults to null
    CHECK_TRUE(flat_cfg.height_sampler == nullptr);

    RoadNetworkBuilder builder_a;
    const RoadNetwork a = builder_a.build(*parsed, flat_cfg);

    CHECK_TRUE(!a.pieces.empty());
    if (a.pieces.empty()) return;

    // Nothing about the terrain path may run.
    CHECK_FALSE(builder_a.elevation().is_solved());
    CHECK_EQ(a.stats.elevated_edges, size_t{0});
    CHECK_TRUE(a.carve_ribbons.empty());
    CHECK_TRUE(a.carve_discs.empty());

    // Every vertex sits on the flat base plane, plus whatever its strip stands
    // above the carriageway.
    const double base = static_cast<double>(flat_cfg.corridor.base_height);
    size_t off_base = 0;
    for (const RoadPiece& piece : a.pieces) {
        for (const Vertex& v : piece.mesh.vertices) {
            const double y = static_cast<double>(v.position.y);
            if (y < base - 1e-4 || y > base + 0.5) ++off_base;
        }
    }
    CHECK_EQ(off_base, size_t{0});

    // A second build of the same data is bit-identical, so the parallel extrude
    // has not become order-dependent.
    RoadNetworkBuilder builder_b;
    const RoadNetwork b = builder_b.build(*parsed, flat_cfg);

    CHECK_EQ(a.pieces.size(), b.pieces.size());
    if (a.pieces.size() != b.pieces.size()) return;

    size_t rebuild_diffs = 0;
    for (size_t i = 0; i < a.pieces.size(); ++i) {
        if (a.pieces[i].edge != b.pieces[i].edge) ++rebuild_diffs;
        if (a.pieces[i].mesh.vertices.size() != b.pieces[i].mesh.vertices.size()) {
            ++rebuild_diffs;
            continue;
        }
        if (a.pieces[i].mesh.indices != b.pieces[i].mesh.indices) ++rebuild_diffs;
        for (size_t v = 0; v < a.pieces[i].mesh.vertices.size(); ++v) {
            if (!(a.pieces[i].mesh.vertices[v] == b.pieces[i].mesh.vertices[v])) {
                ++rebuild_diffs;
            }
        }
    }
    CHECK_EQ(rebuild_diffs, size_t{0});
}

TEST(RoadNetworkElevated, constant_surface_only_translates_the_flat_network) {
    auto parsed = parse_fixture("four_way.osm");
    if (!parsed) return;

    const RoadNetworkConfig flat_cfg;

    RoadNetworkConfig level_cfg;
    level_cfg.height_sampler = HeightSampler{constant_surface};

    RoadNetworkBuilder flat_builder;
    RoadNetworkBuilder level_builder;
    const RoadNetwork flat = flat_builder.build(*parsed, flat_cfg);
    const RoadNetwork level = level_builder.build(*parsed, level_cfg);

    CHECK_EQ(flat.pieces.size(), level.pieces.size());
    CHECK_EQ(flat.stats.triangles, level.stats.triangles);
    CHECK_EQ(flat.stats.vertices, level.stats.vertices);
    CHECK_EQ(flat.stats.skipped_edges, level.stats.skipped_edges);
    if (flat.pieces.size() != level.pieces.size()) return;

    // A flat surface at 250 m puts the carriageway at 250 m plus the surface
    // offset, where the flat build puts it at base_height. Nothing else changes.
    const double expected_shift = 250.0
                                  + static_cast<double>(level_cfg.elevation.surface_offset)
                                  - static_cast<double>(flat_cfg.corridor.base_height);

    // The comparison is by CONTENT, not by index. optimize_mesh() runs on the
    // shipping default and its overdraw pass sorts clusters by a heuristic read
    // off ABSOLUTE vertex positions, so the same network built 250 m higher comes
    // back with the same triangles in a different order and a different vertex
    // numbering. Comparing index for index would be asserting that the reorder
    // does not exist. Every other claim survives intact, and one is added: the
    // triangles themselves are compared, material included, which the per-index
    // form never did.
    //
    // X, Z and UV compare BIT for bit, so the assertion that P3 left the flat
    // extruder alone is as exact as it was. Y is compared on a 1 mm grid after
    // the shift is removed, which is the same 2 mm slack the per-index form used.
    size_t shape_diffs = 0;
    size_t shift_diffs = 0;
    size_t compared = 0;

    /// One vertex reduced to its untouched channels, exactly
    struct PlanKey {
        float x = 0.0f;
        float z = 0.0f;
        float u = 0.0f;
        float v = 0.0f;
        bool operator<(const PlanKey& o) const {
            if (x != o.x) return x < o.x;
            if (z != o.z) return z < o.z;
            if (u != o.u) return u < o.u;
            return v < o.v;
        }
        bool operator==(const PlanKey& o) const { return !(*this < o) && !(o < *this); }
        bool operator!=(const PlanKey& o) const { return !(*this == o); }
    };

    /// The same vertex with its de-shifted height, on a 1 mm grid
    struct FullKey {
        PlanKey plan;
        int64_t y = 0;
        bool operator<(const FullKey& o) const {
            if (plan != o.plan) return plan < o.plan;
            return y < o.y;
        }
        bool operator==(const FullKey& o) const { return !(*this < o) && !(o < *this); }
        bool operator!=(const FullKey& o) const { return !(*this == o); }
    };

    /// A triangle as its material and its three de-shifted corners, canonically rotated
    struct TriKey {
        stratum::MaterialId material = stratum::MaterialId::Default;
        FullKey c[3];
        bool operator<(const TriKey& o) const {
            if (material != o.material) return material < o.material;
            for (size_t k = 0; k < 3; ++k) {
                if (c[k] != o.c[k]) return c[k] < o.c[k];
            }
            return false;
        }
        bool operator==(const TriKey& o) const { return !(*this < o) && !(o < *this); }
        bool operator!=(const TriKey& o) const { return !(*this == o); }
    };

    const auto full_key = [](const Vertex& vert, double shift) {
        FullKey key;
        key.plan.x = vert.position.x;
        key.plan.z = vert.position.z;
        key.plan.u = vert.uv.x;
        key.plan.v = vert.uv.y;
        key.y = std::llround((static_cast<double>(vert.position.y) - shift) / 1e-3);
        return key;
    };

    const auto vertex_keys = [&](const Mesh& mesh, double shift) {
        std::vector<FullKey> keys;
        keys.reserve(mesh.vertices.size());
        for (const Vertex& vert : mesh.vertices) keys.push_back(full_key(vert, shift));
        std::sort(keys.begin(), keys.end());
        return keys;
    };

    const auto triangle_keys = [&](const Mesh& mesh, double shift) {
        std::vector<TriKey> keys;
        for (const stratum::SubMesh& sub : mesh.effective_submeshes()) {
            const size_t end = std::min<size_t>(
                static_cast<size_t>(sub.index_offset) + sub.index_count, mesh.indices.size());
            for (size_t i = sub.index_offset; i + 2 < end + 1 && i + 2 < end; i += 3) {
                TriKey tri;
                tri.material = sub.material;
                for (size_t k = 0; k < 3; ++k) {
                    tri.c[k] = full_key(mesh.vertices[mesh.indices[i + k]], shift);
                }
                // Rotate so the smallest corner leads. Winding is preserved, so a
                // flipped triangle still compares as different.
                size_t lead = 0;
                for (size_t k = 1; k < 3; ++k) {
                    if (tri.c[k] < tri.c[lead]) lead = k;
                }
                TriKey rotated;
                rotated.material = tri.material;
                for (size_t k = 0; k < 3; ++k) rotated.c[k] = tri.c[(lead + k) % 3];
                keys.push_back(rotated);
            }
        }
        std::sort(keys.begin(), keys.end());
        return keys;
    };

    for (size_t i = 0; i < flat.pieces.size(); ++i) {
        const Mesh& fm = flat.pieces[i].mesh;
        const Mesh& lm = level.pieces[i].mesh;

        if (flat.pieces[i].edge != level.pieces[i].edge) ++shape_diffs;
        if (fm.vertices.size() != lm.vertices.size()) {
            ++shape_diffs;
            continue;
        }
        if (fm.indices.size() != lm.indices.size()) ++shape_diffs;
        if (fm.submeshes.size() != lm.submeshes.size()) ++shape_diffs;
        if (flat.pieces[i].outline.size() != level.pieces[i].outline.size()) ++shape_diffs;

        // The 2D geometry is untouched, so X, Z and UV come back bit for bit.
        const std::vector<FullKey> flat_v = vertex_keys(fm, 0.0);
        const std::vector<FullKey> level_v = vertex_keys(lm, expected_shift);
        compared += flat_v.size();

        std::vector<PlanKey> flat_plan;
        std::vector<PlanKey> level_plan;
        flat_plan.reserve(flat_v.size());
        level_plan.reserve(level_v.size());
        for (const FullKey& key : flat_v) flat_plan.push_back(key.plan);
        for (const FullKey& key : level_v) level_plan.push_back(key.plan);
        std::sort(flat_plan.begin(), flat_plan.end());
        std::sort(level_plan.begin(), level_plan.end());
        if (flat_plan != level_plan) {
            ++shape_diffs;
            continue;
        }

        // Same plan geometry, so anything left is a height that did not move by
        // exactly one constant.
        if (flat_v != level_v) ++shift_diffs;

        if (triangle_keys(fm, 0.0) != triangle_keys(lm, expected_shift)) ++shape_diffs;
    }

    CHECK_TRUE(compared > size_t{0});
    CHECK_EQ(shape_diffs, size_t{0});
    CHECK_EQ(shift_diffs, size_t{0});
}

// ============================================================================
// Carve target versus road surface
// ============================================================================
//
// ElevationConfig::surface_offset exists to stop the carriageway and the carved
// terrain being coplanar. That only works if exactly ONE of the pair carries it.
// EdgeElevation::station_heights is the road SURFACE and goes to the extruder
// unchanged, so the carve payload -- which is a TARGET -- must be the same
// heights less the offset. Adding it to both lifts them together and the
// clearance is zero, which is the failure the field is named after.

TEST(RoadNetworkElevated, carve_target_sits_one_surface_offset_below_the_road) {
    auto parsed = parse_fixture("four_way.osm");
    if (!parsed) return;

    RoadNetworkConfig cfg;
    cfg.height_sampler = HeightSampler{tilted_surface};
    CHECK_TRUE(cfg.elevation.surface_offset > 0.0f);

    // The offset contract is stated between EdgeElevation::station_heights and
    // CarveRibbon::centerline_heights, which are only INDEX-PARALLEL on the
    // untrimmed path: P4 cuts the ribbon back and reslices its heights onto the
    // trimmed stations, so the two vectors stop having the same length. The
    // arithmetic being asserted is unchanged either way, so it is asserted on the
    // reference path that can express it. See RoadNetworkConfig::solve_junctions.
    cfg.solve_junctions = false;

    // The longitudinal decimation breaks the same parallelism for the same
    // reason: it drops stations, so the ribbon carries fewer heights than the
    // solve produced. The arithmetic asserted below is per station and is
    // unchanged by which stations survive, so it is asserted on the path that can
    // express it. See RoadNetworkConfig::reduce_tessellation.
    cfg.reduce_tessellation = false;

    RoadNetworkBuilder builder;
    const RoadNetwork network = builder.build(*parsed, cfg);

    CHECK_EQ(network.carve_ribbons.size(), edge_piece_count(network));
    if (network.carve_ribbons.size() != edge_piece_count(network)) return;

    const double offset = static_cast<double>(cfg.elevation.surface_offset);

    size_t compared = 0;
    size_t wrong_clearance = 0;
    for (size_t i = 0; i < network.pieces.size(); ++i) {
        const RoadPiece& piece = network.pieces[i];
        const auto& ribbon = network.carve_ribbons[i];
        if (piece.edge == kInvalidId) continue;

        const EdgeElevation& elev = builder.elevation().edge(piece.edge);
        if (elev.station_heights.size() != ribbon.centerline_heights.size()) continue;

        for (size_t j = 0; j < elev.station_heights.size(); ++j) {
            ++compared;
            const double gap = static_cast<double>(elev.station_heights[j])
                             - static_cast<double>(ribbon.centerline_heights[j]);
            if (std::fabs(gap - offset) > 1e-4) ++wrong_clearance;
        }
    }
    CHECK_TRUE(compared > size_t{0});
    CHECK_EQ(wrong_clearance, size_t{0});

    // The junction disc is on the same contract: node_height() already excludes
    // the offset, so the target is that height as it stands.
    size_t wrong_discs = 0;
    for (size_t n = 0, d = 0; n < builder.graph().nodes().size(); ++n) {
        if (!builder.graph().nodes()[n].is_junction()) continue;
        if (d >= network.carve_discs.size()) break;

        const float node_height =
            builder.elevation().node_height(static_cast<stratum::osm::road::GraphNodeId>(n));
        if (std::fabs(network.carve_discs[d].height - node_height) > 1e-4f) ++wrong_discs;
        ++d;
    }
    CHECK_EQ(wrong_discs, size_t{0});
}

// ============================================================================
// Junction disc suppression
// ============================================================================
//
// A CarveRibbon is suppressed for tunnels AND for bridge spans. The disc that
// joins those ribbons has to agree, and it has to agree by reading the SOLVED
// classification rather than the raw OSM tag: testing GraphEdge::is_tunnel alone
// leaves an all-bridge fork emitting a carving disc, which raises a mesa of
// terrain to deck height under the flyover.

TEST(RoadNetworkElevated, all_bridge_junction_suppresses_its_disc) {
    // Three bridge ways meeting at one shared node: a viaduct fork.
    ParsedOSMData data = make_data({
        make_road(10, {1, 2}, {{-200.0, 0.0}, {0.0, 0.0}}, RoadType::Primary),
        make_road(11, {2, 3}, {{0.0, 0.0}, {200.0, 0.0}}, RoadType::Primary),
        make_road(12, {2, 4}, {{0.0, 0.0}, {0.0, 200.0}}, RoadType::Primary),
    });
    for (Road& road : data.roads) {
        road.is_bridge = true;
        road.layer = 1;
    }

    RoadNetworkConfig cfg;
    cfg.height_sampler = HeightSampler{flat_ten};

    RoadNetworkBuilder builder;
    const RoadNetwork network = builder.build(data, cfg);

    CHECK_TRUE(builder.elevation().is_solved());
    CHECK_EQ(network.carve_discs.size(), size_t{1});
    if (network.carve_discs.empty()) return;

    // Every ribbon is a suppressed bridge span...
    size_t carveable = 0;
    for (const auto& ribbon : network.carve_ribbons) {
        if (!ribbon.suppress) ++carveable;
    }
    CHECK_EQ(carveable, size_t{0});

    // ...so the disc that joins them must be suppressed too.
    CHECK_TRUE(network.carve_discs[0].suppress);
}

TEST(RoadNetworkElevated, ordinary_junction_still_carves_its_disc) {
    // The same fork with no bridge and no tunnel tag. This is the control: it is
    // what stops the fix above from suppressing every disc in the network.
    ParsedOSMData data = make_data({
        make_road(10, {1, 2}, {{-200.0, 0.0}, {0.0, 0.0}}, RoadType::Primary),
        make_road(11, {2, 3}, {{0.0, 0.0}, {200.0, 0.0}}, RoadType::Primary),
        make_road(12, {2, 4}, {{0.0, 0.0}, {0.0, 200.0}}, RoadType::Primary),
    });

    RoadNetworkConfig cfg;
    cfg.height_sampler = HeightSampler{flat_ten};

    RoadNetworkBuilder builder;
    const RoadNetwork network = builder.build(data, cfg);

    CHECK_EQ(network.carve_discs.size(), size_t{1});
    if (network.carve_discs.empty()) return;
    CHECK_FALSE(network.carve_discs[0].suppress);

    // Flat ground at 10 m, so the carve target is the ground itself: the road
    // surface then clears it by exactly surface_offset.
    CHECK_NEAR(static_cast<double>(network.carve_discs[0].height), 10.0, 1e-3);
}

// ============================================================================
// Junction plateaus
// ============================================================================

/**
 * A trimmed arm must still arrive at its junction's own plane.
 *
 * The vertical solve pins each edge's first station to its node height, because
 * before P4 that station sat ON the node. The trims then move it `trim` metres
 * away, where the solved profile is `grade * trim` higher or lower, while the
 * junction fill and its curb ring are one flat plane at the node height. With the
 * shipping trims on a tilted surface that is a decimetre-scale open step at every
 * arm mouth, with nothing bridging it.
 *
 * Measured where it shows: the ribbon vertex sitting on each arm's carriageway
 * corner must be at the junction's own Y.
 */
TEST(RoadNetworkElevated, trimmed_arms_land_on_their_junctions_plane) {
    for (const char* fixture : kFixtures) {
        auto parsed = parse_fixture(fixture);
        if (!parsed) continue;

        RoadNetworkConfig cfg;
        cfg.height_sampler = HeightSampler{tilted_surface};

        RoadNetworkBuilder builder;
        const RoadNetwork network = builder.build(*parsed, cfg);
        const std::string label{fixture};

        size_t mouths_checked = 0;

        for (const auto& junction : network.junctions) {
            if (junction.kind != stratum::osm::road::JunctionKind::Intersection) continue;
            if (!junction.valid) continue;

            for (size_t k = 0; k < junction.ends.size(); ++k) {
                const auto& end = junction.ends[k];
                if (!end.valid) continue;
                if (k >= junction.arms.size()) continue;

                // A trim really was applied, or there is no step to look for.
                if (!(junction.arms[k].trim > 0.1)) continue;

                // The ribbon vertex closest in PLAN to the arm's carriageway
                // corner. The junction ring has a vertex at exactly that point,
                // so the two are the surfaces that have to meet.
                double best_plan = 1e300;
                double best_y = 0.0;
                for (const RoadPiece& piece : network.pieces) {
                    if (piece.edge != junction.arms[k].edge) continue;
                    for (const auto& v : piece.mesh.vertices) {
                        const glm::dvec2 local{static_cast<double>(v.position.x),
                                               -static_cast<double>(v.position.z)};
                        const double d = glm::length(local - end.carriage_left);
                        if (d < best_plan) {
                            best_plan = d;
                            best_y = static_cast<double>(v.position.y);
                        }
                    }
                }

                if (best_plan > 1e-3) continue;   // the arm emitted no piece here
                ++mouths_checked;

                if (std::fabs(best_y - static_cast<double>(junction.height)) > 1e-3) {
                    stratum::test::report_failure(
                        __FILE__, __LINE__, "the arm mouth lands on the junction plane",
                        label + ": arm " + std::to_string(k) + " is " +
                            stratum::test::stringify(best_y -
                                                     static_cast<double>(junction.height)) +
                            " m off it");
                }
            }
        }

        CHECK_TRUE(mouths_checked >= size_t{3});
    }
}

/**
 * Two plateaus that meet share the stations, they do not overwrite each other.
 *
 * A graph edge shorter than about ResampleConfig::max_spacing / max_trim_fraction
 * has both trims landing on the same stations -- a staggered T, a
 * dual-carriageway crossover, a service-road stub, all of which a real extract
 * has in quantity. Applied one after the other, the second plateau wins outright:
 * the whole edge flattens to the TO node's height, and the FROM mouth ends up a
 * full node-height difference below its junction plane. That is a bigger step
 * than the plateau pass exists to remove, and it appears at one end only.
 *
 * Nothing can put both mouths exactly on their own plane on an edge that short --
 * there is no station left in between to hold a grade. What is required is that
 * neither mouth is WORSE than it would be with no plateau at all, and that the
 * two ends are treated alike.
 */
TEST(RoadNetworkElevated, meeting_plateaus_share_the_stations_they_both_claim) {
    constexpr double kGrade = 0.08;      // an 8% hillside
    constexpr double kFraction = 0.4;    // TrimConfig::max_trim_fraction

    for (double length : { 8.0, 12.0, 16.0, 50.0, 400.0 }) {
        const Centerline cl = build_centerline({ glm::dvec2{ 0.0, 0.0 },
                                                 glm::dvec2{ length, 0.0 } },
                                               ResampleConfig{});
        if (!cl.is_valid()) continue;

        const double base = cl.stations.front().arclength;
        const double span = cl.stations.back().arclength - base;
        const double trim = std::min(kFraction * span, 3.5);

        // The solved profile: a constant grade from one node height to the other.
        std::vector<float> heights;
        heights.reserve(cl.stations.size());
        for (const auto& station : cl.stations) {
            heights.push_back(static_cast<float>(-kGrade * (station.arclength - base)));
        }
        const std::vector<float> solved = heights;

        const float level_from = solved.front();
        const float level_to = solved.back();
        const double delta = std::fabs(static_cast<double>(level_to)
                                       - static_cast<double>(level_from));

        // What the un-plateaued solve leaves at each mouth: the baseline this
        // pass has to improve on, or at least not make worse.
        const double baseline_from =
            std::fabs(height_at(cl, solved, base + trim) - static_cast<double>(level_from));
        const double baseline_to =
            std::fabs(height_at(cl, solved, base + span - trim) - static_cast<double>(level_to));

        apply_junction_plateaus(cl, trim, trim, level_from, level_to, heights);

        const double err_from =
            std::fabs(height_at(cl, heights, base + trim) - static_cast<double>(level_from));
        const double err_to =
            std::fabs(height_at(cl, heights, base + span - trim) - static_cast<double>(level_to));

        const std::string where =
            "L = " + stratum::test::stringify(length) + " m, trim " +
            stratum::test::stringify(trim) + " m, node delta " + stratum::test::stringify(delta);

        if (err_from > baseline_from + 1e-6 || err_to > baseline_to + 1e-6) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "a plateau never leaves a mouth worse than no plateau",
                where + ": from " + stratum::test::stringify(err_from) + " m against a baseline of "
                    + stratum::test::stringify(baseline_from) + " m, to "
                    + stratum::test::stringify(err_to) + " m against "
                    + stratum::test::stringify(baseline_to) + " m");
        }

        // Neither end is favoured. The sequential application put the whole
        // error on the from end and none on the to end.
        if (std::fabs(err_from - err_to) > 1e-4) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "both junctions are treated alike",
                where + ": from is " + stratum::test::stringify(err_from) + " m off, to is " +
                    stratum::test::stringify(err_to) + " m off");
        }
    }
}

/**
 * On an edge long enough for both, each plateau still lands its own mouth exactly
 * on its own junction plane. The overlap rule must not cost anything on the edges
 * that were already right.
 */
TEST(RoadNetworkElevated, separated_plateaus_still_land_exactly) {
    constexpr double kGrade = 0.08;
    const Centerline cl = build_centerline({ glm::dvec2{ 0.0, 0.0 }, glm::dvec2{ 400.0, 0.0 } },
                                           ResampleConfig{});
    CHECK_TRUE(cl.is_valid());
    if (!cl.is_valid()) return;

    const double base = cl.stations.front().arclength;
    const double span = cl.stations.back().arclength - base;

    std::vector<float> heights;
    for (const auto& station : cl.stations) {
        heights.push_back(static_cast<float>(-kGrade * (station.arclength - base)));
    }
    const float level_from = heights.front();
    const float level_to = heights.back();

    apply_junction_plateaus(cl, 6.0, 6.0, level_from, level_to, heights);

    CHECK_NEAR(height_at(cl, heights, base + 6.0), static_cast<double>(level_from), 1e-4);
    CHECK_NEAR(height_at(cl, heights, base + span - 6.0), static_cast<double>(level_to), 1e-4);
}

// ============================================================================
// Roundabouts
// ============================================================================

/**
 * A roundabout ring is ONE surface at ONE height, so its nodes have to be solved
 * that way.
 *
 * The annulus is swept flat at the height of the loop's first node and its carve
 * is a single flat disc, but the grade limit alone leaves the ring free to follow
 * the terrain around its own circumference: on a tilted surface the approach nodes
 * legitimately end up decimetres apart. Every one of those differences reappears
 * as a vertical crack between the annulus and an approach mouth.
 */
TEST(RoadNetworkElevated, roundabout_ring_nodes_are_solved_to_one_height) {
    auto parsed = parse_fixture("roundabout.osm");
    if (!parsed) return;

    RoadNetworkConfig cfg;
    cfg.height_sampler = HeightSampler{tilted_surface};

    RoadNetworkBuilder builder;
    const RoadNetwork network = builder.build(*parsed, cfg);
    CHECK_TRUE(builder.elevation().is_solved());
    if (!builder.elevation().is_solved()) return;

    const std::vector<stratum::osm::road::RoundaboutLoop> loops =
        stratum::osm::road::find_roundabouts(builder.graph(), builder.centerlines());
    CHECK_EQ(loops.size(), size_t{1});
    if (loops.empty() || loops[0].nodes.size() < 3) return;

    // The terrain under the ring is NOT flat, or this would pass for the wrong
    // reason.
    double terrain_spread = 0.0;
    for (const auto& node : loops[0].nodes) {
        for (const auto& other : loops[0].nodes) {
            terrain_spread = std::max(
                terrain_spread,
                std::fabs(static_cast<double>(tilted_surface(builder.graph().node(node).position.x,
                                                             builder.graph().node(node).position.y)) -
                          static_cast<double>(tilted_surface(builder.graph().node(other).position.x,
                                                             builder.graph().node(other).position.y))));
        }
    }
    CHECK_TRUE(terrain_spread > 0.5);

    const float level = builder.elevation().node_height(loops[0].nodes.front());
    for (const auto& node : loops[0].nodes) {
        CHECK_NEAR(static_cast<double>(builder.elevation().node_height(node)),
                   static_cast<double>(level), 1e-4);
    }

    // And the annulus, which is drawn at that one height, agrees with the carve
    // disc that flattens the ground under it.
    for (const auto& junction : network.junctions) {
        if (junction.kind != stratum::osm::road::JunctionKind::Roundabout) continue;
        CHECK_NEAR(static_cast<double>(junction.height),
                   static_cast<double>(level) + static_cast<double>(cfg.elevation.surface_offset),
                   1e-4);
    }
}

// ============================================================================
// Tapers
// ============================================================================

/**
 * @brief Two ways meeting at one degree-2 node, with different lane counts
 *
 * A lane change is a profile change, so the node gets a taper: a flat wedge
 * blending one cross-section into the other, with BOTH ribbons trimmed back off
 * it. Nothing else in the network covers that stretch of ground.
 */
ParsedOSMData lane_change_at_origin() {
    Road narrow = make_road(10, {1, 2}, {{-200.0, 0.0}, {0.0, 0.0}}, RoadType::Secondary);
    Road wide = make_road(11, {2, 3}, {{0.0, 0.0}, {200.0, 0.0}}, RoadType::Secondary);
    narrow.lanes = 2;
    wide.lanes = 4;
    wide.width = 14.0f;
    return make_data({narrow, wide});
}

/**
 * A taper's ground has to be carved.
 *
 * The carve footprint pass walks nodes of degree 3 or more, and a taper node has
 * degree 2, so before P4 there was nothing to emit there and nothing to miss:
 * both ribbons ran to the node. P4 trims them back by half the taper length each
 * -- up to 30 m a side -- and the wedge in between is emitted FLAT at the node
 * height, so without a footprint of its own the raw procedural terrain runs
 * straight through the road surface.
 */
TEST(RoadNetworkElevated, a_profile_taper_carves_the_ground_it_covers) {
    ParsedOSMData data = lane_change_at_origin();

    RoadNetworkConfig cfg;
    cfg.height_sampler = HeightSampler{tilted_surface};

    RoadNetworkBuilder builder;
    const RoadNetwork network = builder.build(data, cfg);

    // The premise: one taper, and both of its edges really were trimmed.
    const stratum::osm::road::Junction* taper = nullptr;
    for (const auto& junction : network.junctions) {
        if (junction.kind == stratum::osm::road::JunctionKind::Taper) taper = &junction;
    }
    CHECK(taper != nullptr);
    if (taper == nullptr) return;
    CHECK_TRUE(taper->valid);

    double trimmed = 0.0;
    for (const auto& edge : builder.graph().edges()) {
        trimmed = std::max(trimmed, std::max(edge.trim_from, edge.trim_to));
    }
    CHECK_TRUE(trimmed > 1.0);

    // The footprint exists, is a real polygon, and covers the node.
    CHECK_TRUE(taper->footprint.size() >= size_t{3});
    if (taper->footprint.size() < 3) return;

    // And a carve disc carries it, reaching at least as far as the trims do.
    const stratum::osm::road::CarveDisc* disc = nullptr;
    for (const auto& candidate : network.carve_discs) {
        if (glm::length(candidate.center - taper->center) < 1e-6) disc = &candidate;
    }
    CHECK(disc != nullptr);
    if (disc == nullptr) return;

    CHECK_TRUE(disc->outline.size() >= size_t{3});
    CHECK_TRUE(static_cast<double>(disc->radius) >= trimmed);
    CHECK_FALSE(disc->suppress);

    // The stretch of ground the trims opened is inside it, all the way to each
    // ribbon's new end.
    for (const double x : {-0.9 * trimmed, 0.0, 0.9 * trimmed}) {
        if (!stratum::test::junction::point_in_ring(disc->outline, glm::dvec2{x, 0.0})) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "the taper's carve footprint covers the trimmed stretch",
                "uncovered at x = " + stratum::test::stringify(x));
        }
    }
}

/**
 * The wedge has to be built over the trims the extruder actually cuts at.
 *
 * A taper's demand goes through the same joint budget every junction trim does,
 * and on a short edge between two taper nodes the budget reduces it. The wedge
 * was built in pass B from the demand; if it is not rebuilt, the surviving ribbon
 * lies coplanar on top of it.
 *
 * The fixture is the smallest one that clamps: a 3 m edge between two lane
 * changes, whose two demands of 1.5 m each exceed the 2.75 m the budget allows.
 */
TEST(RoadNetworkElevated, a_clamped_taper_is_rebuilt_against_its_final_trim) {
    Road west = make_road(10, {1, 2}, {{-200.0, 0.0}, {0.0, 0.0}}, RoadType::Secondary);
    Road middle = make_road(11, {2, 3}, {{0.0, 0.0}, {3.0, 0.0}}, RoadType::Secondary);
    Road east = make_road(12, {3, 4}, {{3.0, 0.0}, {203.0, 0.0}}, RoadType::Secondary);
    west.lanes = 2;
    middle.lanes = 4;
    middle.width = 14.0f;
    east.lanes = 2;

    ParsedOSMData data = make_data({west, middle, east});

    RoadNetworkConfig cfg;
    RoadNetworkBuilder builder;
    const RoadNetwork network = builder.build(data, cfg);

    // The premise: the short edge was clamped, so its two trims no longer add up
    // to the 1.5 m each taper asked for.
    stratum::osm::road::EdgeId short_edge = kInvalidId;
    for (size_t i = 0; i < builder.graph().edges().size(); ++i) {
        if (builder.graph().edge(static_cast<stratum::osm::road::EdgeId>(i)).polyline.size() == 2 &&
            builder.centerlines()[i].length() < 3.5) {
            short_edge = static_cast<stratum::osm::road::EdgeId>(i);
        }
    }
    CHECK(short_edge != kInvalidId);
    if (short_edge == kInvalidId) return;

    const auto& edge = builder.graph().edge(short_edge);
    CHECK_TRUE(edge.trim_from > 0.0);
    CHECK_TRUE(edge.trim_to > 0.0);
    CHECK_TRUE(edge.trim_from < 1.5 - 1e-6);   // the budget really did bind

    // Every taper wedge stops at the trim its own edge was cut at. The wedge runs
    // along +x from the node at x = 0 and along -x from the node at x = 3.
    size_t checked = 0;
    for (const auto& junction : network.junctions) {
        if (junction.kind != stratum::osm::road::JunctionKind::Taper || !junction.valid) continue;

        const double node_x = junction.center.x;
        const bool at_origin = std::fabs(node_x) < 1e-6;
        const double limit = at_origin ? edge.trim_from : edge.trim_to;

        double reach = 0.0;
        for (const auto& v : junction.mesh.vertices) {
            const double dx = static_cast<double>(v.position.x) - node_x;
            reach = std::max(reach, at_origin ? dx : -dx);
        }
        ++checked;

        if (reach > limit + 1e-3) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "the taper wedge stops where the ribbon is cut",
                "wedge reaches " + stratum::test::stringify(reach) + " m but the trim is " +
                    stratum::test::stringify(limit) + " m");
        }
    }
    CHECK_EQ(checked, size_t{2});
}

/**
 * A turning circle reaches past the end of the ribbon it caps -- 6 m of disc
 * against a 3.5 m half width -- so the ground under it belongs to no CarveRibbon.
 * Its node has degree 1 and is skipped by the per-node footprint pass, so without
 * a footprint of its own the bulb sits on raw procedural terrain.
 */
TEST(RoadNetworkElevated, a_turning_circle_carves_the_ground_under_its_disc) {
    auto parsed = parse_fixture("cul_de_sac.osm");
    if (!parsed) return;

    RoadNetworkConfig cfg;
    cfg.height_sampler = HeightSampler{tilted_surface};

    RoadNetworkBuilder builder;
    const RoadNetwork network = builder.build(*parsed, cfg);

    // The fixture's turning circle, found by the tag rather than by position.
    stratum::osm::road::GraphNodeId circle = kInvalidId;
    for (size_t i = 0; i < builder.graph().nodes().size(); ++i) {
        const auto& node = builder.graph().nodes()[i];
        if (node.is_turning_circle && node.is_dead_end()) {
            circle = static_cast<stratum::osm::road::GraphNodeId>(i);
        }
    }
    CHECK(circle != kInvalidId);
    if (circle == kInvalidId) return;

    const stratum::osm::road::Junction* cap = nullptr;
    for (const auto& junction : network.junctions) {
        if (junction.node == circle &&
            junction.kind == stratum::osm::road::JunctionKind::DeadEnd) {
            cap = &junction;
        }
    }
    CHECK(cap != nullptr);
    if (cap == nullptr) return;
    CHECK_TRUE(cap->footprint.size() >= size_t{3});

    const glm::dvec2 centre = builder.graph().node(circle).position;
    const stratum::osm::road::CarveDisc* disc = nullptr;
    for (const auto& candidate : network.carve_discs) {
        if (glm::length(candidate.center - centre) < 1e-6) disc = &candidate;
    }
    CHECK(disc != nullptr);
    if (disc == nullptr) return;

    // The whole disc is inside the footprint, not merely the node.
    CHECK_TRUE(disc->outline.size() >= size_t{3});
    CHECK_TRUE(static_cast<double>(disc->radius) >= 6.0);
    CHECK_FALSE(disc->suppress);

    // Sampled across the half of the disc that lies BEYOND the node, because that
    // is the half no ribbon covers. The other half is notched out of the cap on
    // purpose -- it is the ribbon's own surface -- and carries the ribbon's carve.
    const auto& arm = builder.graph().node(circle).arms.front();
    const auto& centerline = builder.centerlines()[arm.edge];
    CHECK_TRUE(centerline.is_valid());
    if (!centerline.is_valid()) return;
    const glm::dvec2 into_road = arm.at_start ? centerline.stations.front().tangent
                                              : -centerline.stations.back().tangent;
    const glm::dvec2 outward = -glm::normalize(into_road);
    const glm::dvec2 side{-outward.y, outward.x};

    for (int i = -2; i <= 2; ++i) {
        const double angle = static_cast<double>(i) * (3.14159265358979323846 / 8.0);
        const glm::dvec2 probe =
            centre + (outward * std::cos(angle) + side * std::sin(angle)) * 4.0;
        if (!stratum::test::junction::point_in_ring(disc->outline, probe)) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "the turning circle's footprint covers its own disc",
                "uncovered 4 m beyond the node at " + stratum::test::stringify(angle) + " rad");
        }
    }
}
