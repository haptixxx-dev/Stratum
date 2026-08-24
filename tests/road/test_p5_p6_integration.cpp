/**
 * @file test_p5_p6_integration.cpp
 * @brief The whole pipeline with P5 and P6 on, over every fixture, and off again
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The other four suites in this phase each take one entry point and pin its
 * behaviour exactly. This one takes RoadNetworkBuilder::build() and asks the two
 * questions that only fall out when everything runs together.
 *
 * ### Does anything come out broken
 *
 * Swept over EVERY fixture in tests/data, old and new, with the terrain on and
 * with it off. A NaN vertex position reaching the GPU neither crashes nor
 * renders: it silently corrupts the bounding box, breaks frustum culling for the
 * whole chunk, and is the hardest failure in this pipeline to trace back to its
 * cause. The same sweep checks that marking geometry is tagged
 * MaterialId::Markings and shares no vertex with anything else, which is the
 * property that lets P7 weld the corridor without dragging the paint into it.
 *
 * ### Do the switches actually switch off
 *
 * RoadNetworkConfig carries four flags that each reproduce an earlier phase:
 * emit_markings, emit_crossings, emit_structures, and DedupConfig::enabled. The
 * plan's claim is that turning them off gives back exactly the previous phase's
 * output, and that claim is only worth anything if it is tested. A pass that
 * "switches off" but leaves the profiles a few centimetres different has quietly
 * made every earlier golden file wrong.
 *
 * The check is deliberately made on the SURFACE materials rather than on the
 * whole mesh. Markings, BridgeDeck and Parapet are additions and are expected to
 * appear and disappear; Asphalt, Curb, Sidewalk, Grass, Gravel and Dirt are the
 * P4 output and must be identical triangle for triangle and square metre for
 * square metre.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests P5P6Integration
 * @endcode
 */

#include "framework.hpp"
#include "road/p5_p6_fixtures.hpp"

#include "osm/road/road_graph.hpp"
#include "osm/road/road_network_builder.hpp"
#include "osm/types.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace {

using stratum::MaterialId;
using stratum::Mesh;
using stratum::SubMesh;
using stratum::material_id_name;
using stratum::osm::ParsedOSMData;
using stratum::osm::road::HeightSampler;
using stratum::osm::road::RoadNetwork;
using stratum::osm::road::RoadNetworkBuilder;
using stratum::osm::road::RoadNetworkConfig;
using stratum::osm::road::RoadPiece;
using stratum::osm::road::kInvalidId;

namespace p5 = stratum::test::p5;
namespace jt = stratum::test::junction;

// ============================================================================
// Terrain
// ============================================================================

/**
 * @brief Smooth rolling ground, for the sweep that runs on terrain
 *
 * Deliberately not flat and deliberately not noise. Flat terrain never exercises
 * the grade solve, the carve, or the pier drop, and every structure then sits at
 * one height where a sign error is invisible. Noise would make the elevation
 * solve's own smoothing the dominant effect and the test unreproducible in the
 * head. Two long sine waves give real slope, real crests and real sags, with an
 * exact closed form at every point and the same value on every thread.
 */
HeightSampler rolling() {
    return [](double x, double y) {
        return static_cast<float>(9.0 * std::sin(x / 130.0) + 6.0 * std::cos(y / 95.0) + 20.0);
    };
}

/// The three P5/P6 passes, all on
RoadNetworkConfig everything_on(const HeightSampler& sampler) {
    RoadNetworkConfig cfg;
    cfg.solve_junctions = true;
    cfg.emit_markings = true;
    cfg.emit_crossings = true;
    cfg.emit_structures = true;
    cfg.dedup.enabled = true;
    cfg.height_sampler = sampler;
    return cfg;
}

/// The three P5/P6 passes and the dedup query, all off: the P4 configuration
RoadNetworkConfig everything_off(const HeightSampler& sampler) {
    RoadNetworkConfig cfg;
    cfg.solve_junctions = true;
    cfg.emit_markings = false;
    cfg.emit_crossings = false;
    cfg.emit_structures = false;
    cfg.dedup.enabled = false;
    cfg.height_sampler = sampler;
    return cfg;
}

/// True when a material is one P5 or P6 adds and P4 never emitted
bool is_added_by_this_phase(MaterialId material) {
    return material == MaterialId::Markings || material == MaterialId::BridgeDeck ||
           material == MaterialId::Parapet;
}

/**
 * @brief The surface materials P4 owned, which must not move when P5 and P6 run
 *
 * MaterialId::Concrete is deliberately absent. P4 emits it for the junction
 * apron and P6 emits it for piers and portals, so it is the one slot that
 * legitimately grows, and including it would make this comparison fail for the
 * right reason at the wrong time.
 */
constexpr MaterialId kSurfaceMaterials[] = {
    MaterialId::Default, MaterialId::Asphalt, MaterialId::Curb,   MaterialId::Sidewalk,
    MaterialId::Gravel,  MaterialId::Dirt,    MaterialId::Grass,
};

/// Triangle count and plan area per surface material, summed over a whole network
struct SurfaceTally {
    size_t triangles[sizeof(kSurfaceMaterials) / sizeof(kSurfaceMaterials[0])] = {};
    double area[sizeof(kSurfaceMaterials) / sizeof(kSurfaceMaterials[0])] = {};
};

SurfaceTally tally_surfaces(const RoadNetwork& network) {
    SurfaceTally out;
    for (const RoadPiece& piece : network.pieces) {
        for (const jt::Tri2D& tri : jt::triangles_of(piece.mesh)) {
            for (size_t m = 0; m < sizeof(kSurfaceMaterials) / sizeof(kSurfaceMaterials[0]); ++m) {
                if (tri.material != kSurfaceMaterials[m]) continue;
                ++out.triangles[m];
                out.area[m] += 0.5 * std::fabs(jt::cross2(tri.a, tri.b, tri.c));
            }
        }
    }
    return out;
}

/// Per-triangle material of a mesh, resolving the implicit whole-mesh range
std::vector<MaterialId> triangle_materials(const Mesh& mesh) {
    std::vector<MaterialId> out(mesh.indices.size() / 3, MaterialId::Default);
    for (const SubMesh& sub : mesh.effective_submeshes()) {
        const size_t first = sub.index_offset / 3u;
        const size_t last = (static_cast<size_t>(sub.index_offset) + sub.index_count) / 3u;
        for (size_t t = first; t < last && t < out.size(); ++t) out[t] = sub.material;
    }
    return out;
}

/// A vertex position as an exact, orderable key
struct PositionKey {
    uint32_t x = 0;
    uint32_t y = 0;
    uint32_t z = 0;
    bool operator<(const PositionKey& o) const {
        if (x != o.x) return x < o.x;
        if (y != o.y) return y < o.y;
        return z < o.z;
    }
    bool operator==(const PositionKey& o) const { return x == o.x && y == o.y && z == o.z; }
};

PositionKey key_of(const glm::vec3& p) {
    PositionKey k;
    std::memcpy(&k.x, &p.x, sizeof(float));
    std::memcpy(&k.y, &p.y, sizeof(float));
    std::memcpy(&k.z, &p.z, sizeof(float));
    return k;
}

/**
 * @brief Marking geometry shares no vertex with any other material in one mesh
 *
 * Checked two ways, because they catch different mistakes. Sharing an INDEX
 * means one vertex is referenced by a marking triangle and by a corridor
 * triangle, which is what a weld produces and what would let P7 average the two
 * surfaces' normals together. Sharing a POSITION means two distinct vertices sit
 * at exactly the same point, which is what a marking emitted ON the carriageway
 * rather than above it produces, and which z-fights.
 *
 * The position test compares the float bit patterns rather than a tolerance.
 * Marking and corridor positions are computed by different code from different
 * laterals, so an accidental exact match is not a thing that happens; an exact
 * match means one was copied from the other.
 *
 * @param mesh   Mesh to check
 * @param reason Receives a description of the first violation found
 * @return True when no marking vertex is shared
 */
bool markings_are_separate(const Mesh& mesh, std::string& reason) {
    const std::vector<MaterialId> per_triangle = triangle_materials(mesh);

    std::vector<bool> marking_vertex(mesh.vertices.size(), false);
    std::vector<bool> other_vertex(mesh.vertices.size(), false);
    for (size_t t = 0; t < per_triangle.size(); ++t) {
        const bool marking = per_triangle[t] == MaterialId::Markings;
        for (int k = 0; k < 3; ++k) {
            const uint32_t index = mesh.indices[t * 3 + k];
            if (index >= mesh.vertices.size()) continue;
            (marking ? marking_vertex : other_vertex)[index] = true;
        }
    }
    for (size_t v = 0; v < mesh.vertices.size(); ++v) {
        if (marking_vertex[v] && other_vertex[v]) {
            reason = "vertex " + std::to_string(v) + " is used by a marking and by another material";
            return false;
        }
    }

    std::vector<PositionKey> others;
    others.reserve(mesh.vertices.size());
    for (size_t v = 0; v < mesh.vertices.size(); ++v) {
        if (other_vertex[v]) others.push_back(key_of(mesh.vertices[v].position));
    }
    std::sort(others.begin(), others.end());

    for (size_t v = 0; v < mesh.vertices.size(); ++v) {
        if (!marking_vertex[v]) continue;
        const PositionKey k = key_of(mesh.vertices[v].position);
        if (std::binary_search(others.begin(), others.end(), k)) {
            reason = "a marking vertex sits at exactly the position of a non-marking vertex";
            return false;
        }
    }
    return true;
}

/// Submesh ranges tile the index buffer exactly once, with no gap and no overlap
bool submeshes_tile_exactly(const Mesh& mesh, std::string& reason) {
    if (mesh.submeshes.empty()) return true;
    uint32_t expected = 0;
    for (const SubMesh& sub : mesh.submeshes) {
        if (sub.index_offset != expected) {
            reason = "submesh starts at " + std::to_string(sub.index_offset) + ", expected " +
                     std::to_string(expected);
            return false;
        }
        if ((sub.index_count % 3u) != 0u) {
            reason = "submesh index count is not a whole number of triangles";
            return false;
        }
        expected += sub.index_count;
    }
    if (expected != mesh.indices.size()) {
        reason = "submeshes cover " + std::to_string(expected) + " of " +
                 std::to_string(mesh.indices.size()) + " indices";
        return false;
    }
    return true;
}

/// Build one fixture, reporting a parse failure rather than returning silently
bool build_fixture(const std::string& filename, const RoadNetworkConfig& cfg,
                   RoadNetworkBuilder& builder, RoadNetwork& out) {
    const auto parsed = jt::parse_fixture(filename.c_str());
    if (!parsed) return false;
    out = builder.build(*parsed, cfg);
    return true;
}

} // namespace

// ============================================================================
// The sweep
// ============================================================================

/**
 * Every fixture, everything on, on terrain and off it: nothing infinite, nothing
 * mistagged, no marking welded to anything.
 *
 * The sweep runs over the P4 fixtures as well as the six this phase added, so a
 * defect that only shows on a roundabout or on a motorway link is caught by the
 * pass that introduced it rather than by the one that happens to have a fixture
 * for it.
 *
 * Both terrain modes matter. With a sampler the elevation solve, the carve
 * requests and the whole of P6 run; without one the roads stay flat at
 * CorridorConfig::base_height and every structure is skipped, which is the P2
 * path and the one an editor uses before any terrain exists.
 */
TEST(P5P6Integration, every_fixture_builds_clean_with_every_pass_on) {
    const std::vector<std::string> fixtures = p5::all_fixtures();
    CHECK_TRUE(fixtures.size() >= 15);

    for (const std::string& filename : fixtures) {
        for (int mode = 0; mode < 2; ++mode) {
            const HeightSampler sampler = mode == 0 ? rolling() : HeightSampler{};
            RoadNetworkBuilder builder;
            RoadNetwork network;
            if (!build_fixture(filename, everything_on(sampler), builder, network)) continue;

            const std::string label =
                filename + (mode == 0 ? " [on terrain]" : " [flat]");

            CHECK_TRUE(!network.pieces.empty());

            for (size_t i = 0; i < network.pieces.size(); ++i) {
                const RoadPiece& piece = network.pieces[i];
                const std::string where = label + " piece " + std::to_string(i);

                if (!p5::mesh_is_finite(piece.mesh)) {
                    stratum::test::report_failure(__FILE__, __LINE__,
                                                  "every vertex is finite", where);
                }
                if (!p5::mesh_indices_are_sane(piece.mesh)) {
                    stratum::test::report_failure(__FILE__, __LINE__,
                                                  "every index is in range", where);
                }
                if (!std::isfinite(piece.anchor.x) || !std::isfinite(piece.anchor.y)) {
                    stratum::test::report_failure(__FILE__, __LINE__, "the anchor is finite",
                                                  where);
                }
                for (const glm::dvec2& p : piece.outline) {
                    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
                        stratum::test::report_failure(__FILE__, __LINE__,
                                                      "every outline point is finite", where);
                        break;
                    }
                }

                std::string reason;
                if (!submeshes_tile_exactly(piece.mesh, reason)) {
                    stratum::test::report_failure(__FILE__, __LINE__,
                                                  "submeshes tile the index buffer",
                                                  where + ": " + reason);
                }
                if (!markings_are_separate(piece.mesh, reason)) {
                    stratum::test::report_failure(__FILE__, __LINE__,
                                                  "markings share no vertex with the surface",
                                                  where + ": " + reason);
                }

                // Every material a piece reports must be a real slot.
                for (const SubMesh& sub : piece.mesh.effective_submeshes()) {
                    if (static_cast<uint8_t>(sub.material) >=
                        static_cast<uint8_t>(MaterialId::Count)) {
                        stratum::test::report_failure(__FILE__, __LINE__,
                                                      "the submesh material is a real slot", where);
                    }
                }
            }

            // Carve requests are only produced on terrain, and there is one per
            // edge piece when they are.
            if (mode == 0) {
                CHECK_EQ(network.carve_ribbons.size(),
                         network.stats.pieces - network.stats.junction_pieces);
            } else {
                CHECK_EQ(network.carve_ribbons.size(), size_t{0});
                CHECK_EQ(network.carve_discs.size(), size_t{0});
            }
        }
    }
}

/**
 * Marking geometry only ever appears in the Markings slot, and only on edges
 * that may carry paint.
 *
 * A footway, a cycleway and a path are never painted, so a piece built from one
 * must hold no Markings triangle at all. That rule is stated in markings.hpp and
 * it is the one that stops a zebra being drawn along a park path, which is what
 * happens if the type filter is applied to the crossing search but not to the
 * lane-line search.
 */
TEST(P5P6Integration, footway_pieces_never_carry_paint) {
    for (const std::string& filename : p5::all_fixtures()) {
        RoadNetworkBuilder builder;
        RoadNetwork network;
        if (!build_fixture(filename, everything_on(rolling()), builder, network)) continue;

        for (const RoadPiece& piece : network.pieces) {
            if (piece.edge == kInvalidId) continue;   // a junction piece, not an edge
            const auto& edge = builder.graph().edge(piece.edge);
            const bool paintable = edge.type != stratum::osm::RoadType::Footway &&
                                   edge.type != stratum::osm::RoadType::Cycleway &&
                                   edge.type != stratum::osm::RoadType::Path;
            if (paintable) continue;

            const size_t painted = p5::triangles_with(piece.mesh, MaterialId::Markings).size();
            if (painted != 0) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "a footway piece carries no marking geometry",
                    filename + ": " + std::to_string(painted) + " marking triangles on a " +
                        stratum::osm::road_type_name(edge.type) + " edge");
            }
        }
    }
}

// ============================================================================
// Switching the phase off
// ============================================================================

/**
 * With every P5 and P6 switch off, the surface materials are exactly what P4
 * produced and none of the new ones appear.
 *
 * Two halves. The first is the easy one: no Markings, no BridgeDeck and no
 * Parapet triangle exists anywhere. The second is the one that catches a
 * regression: the triangle count and the plan area of every surface material are
 * identical, fixture by fixture, between the all-off build and the all-on build.
 *
 * Two passes are held off in BOTH builds for that comparison, because they are
 * the two that change P4 surfaces rather than adding to them, and leaving either
 * on would make the areas differ for a correct reason and hide an incorrect one
 * behind it:
 *
 * - **DedupConfig::enabled.** Dedup legitimately REMOVES a Sidewalk strip from a
 *   carriageway whose sidewalk is separately mapped.
 * - **CrossingConfig::emit_dropped_kerbs.** A dropped kerb is defined as a change
 *   to the junction's curb ring -- it lowers the curb face and top across a span
 *   and resamples the ring so the ramp has columns to sit on -- so it moves Curb
 *   and Sidewalk triangles by design. See crossings.hpp. The zebra PAINT stays on
 *   in the additive build; it is Markings, so it is outside this tally anyway,
 *   and the crossing pass is still proving it adds no surface geometry of its
 *   own.
 *
 * Both have their own switch-off asserted separately below.
 */
TEST(P5P6Integration, switching_the_phase_off_reproduces_the_p4_surfaces) {
    for (const std::string& filename : p5::all_fixtures()) {
        RoadNetworkBuilder off_builder;
        RoadNetwork off;
        if (!build_fixture(filename, everything_off(rolling()), off_builder, off)) continue;

        // Nothing this phase adds may appear.
        for (const RoadPiece& piece : off.pieces) {
            for (const jt::Tri2D& tri : jt::triangles_of(piece.mesh)) {
                if (is_added_by_this_phase(tri.material)) {
                    stratum::test::report_failure(
                        __FILE__, __LINE__, "the switched-off build emits no P5 or P6 material",
                        filename + ": " + material_id_name(tri.material));
                }
            }
        }
        CHECK_EQ(off.stats.markings_pieces, size_t{0});
        CHECK_EQ(off.stats.crossings, size_t{0});
        CHECK_EQ(off.stats.bridges, size_t{0});
        CHECK_EQ(off.stats.tunnels, size_t{0});
        CHECK_EQ(off.stats.deduped_sidewalks, size_t{0});

        // The surfaces are untouched by turning the additions back on.
        RoadNetworkConfig additive = everything_on(rolling());
        additive.dedup.enabled = false;
        additive.crossings.emit_dropped_kerbs = false;

        RoadNetworkBuilder on_builder;
        RoadNetwork on;
        if (!build_fixture(filename, additive, on_builder, on)) continue;

        CHECK_EQ(on.stats.pieces, off.stats.pieces);
        CHECK_EQ(on.junctions.size(), off.junctions.size());
        CHECK_EQ(on.carve_ribbons.size(), off.carve_ribbons.size());
        CHECK_EQ(on.carve_discs.size(), off.carve_discs.size());

        const SurfaceTally a = tally_surfaces(off);
        const SurfaceTally b = tally_surfaces(on);
        for (size_t m = 0; m < sizeof(kSurfaceMaterials) / sizeof(kSurfaceMaterials[0]); ++m) {
            if (a.triangles[m] != b.triangles[m]) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "the surface triangle count is unchanged",
                    filename + ": " + material_id_name(kSurfaceMaterials[m]) + " off " +
                        std::to_string(a.triangles[m]) + ", on " + std::to_string(b.triangles[m]));
            }
            CHECK_NEAR(b.area[m], a.area[m], 1e-6);
        }
    }
}

/**
 * Each of the three switches reproduces the earlier phase on its own.
 *
 * The plan asks for exactly this, so that a visual regression can be bisected to
 * a pass without a rebuild. Turning one pass off must remove only that pass's
 * materials and must leave every other count where it was.
 */
TEST(P5P6Integration, each_switch_removes_only_its_own_geometry) {
    for (const std::string& filename : p5::all_fixtures()) {
        RoadNetworkConfig base = everything_on(rolling());
        base.dedup.enabled = false;

        RoadNetworkBuilder all_builder;
        RoadNetwork all;
        if (!build_fixture(filename, base, all_builder, all)) continue;

        {
            RoadNetworkConfig cfg = base;
            cfg.emit_markings = false;
            RoadNetworkBuilder builder;
            RoadNetwork net;
            if (build_fixture(filename, cfg, builder, net)) {
                CHECK_EQ(net.stats.markings_pieces, size_t{0});
                CHECK_EQ(net.stats.bridges, all.stats.bridges);
                CHECK_EQ(net.stats.tunnels, all.stats.tunnels);
                CHECK_EQ(net.stats.crossings, all.stats.crossings);
                CHECK_EQ(net.stats.pieces, all.stats.pieces);
            }
        }
        {
            RoadNetworkConfig cfg = base;
            cfg.emit_crossings = false;
            RoadNetworkBuilder builder;
            RoadNetwork net;
            if (build_fixture(filename, cfg, builder, net)) {
                CHECK_EQ(net.stats.crossings, size_t{0});
                CHECK_EQ(net.stats.bridges, all.stats.bridges);
                CHECK_EQ(net.stats.tunnels, all.stats.tunnels);
                CHECK_EQ(net.stats.pieces, all.stats.pieces);
            }
        }
        {
            RoadNetworkConfig cfg = base;
            cfg.emit_structures = false;
            RoadNetworkBuilder builder;
            RoadNetwork net;
            if (build_fixture(filename, cfg, builder, net)) {
                CHECK_EQ(net.stats.bridges, size_t{0});
                CHECK_EQ(net.stats.tunnels, size_t{0});
                CHECK_EQ(net.stats.crossings, all.stats.crossings);
                CHECK_EQ(net.stats.pieces, all.stats.pieces);

                size_t structural = 0;
                for (const RoadPiece& piece : net.pieces) {
                    structural += p5::triangles_with(piece.mesh, MaterialId::BridgeDeck).size();
                    structural += p5::triangles_with(piece.mesh, MaterialId::Parapet).size();
                }
                CHECK_EQ(structural, size_t{0});
            }
        }
    }
}

/**
 * Structures need the terrain, whatever the switch says.
 *
 * A pier with nothing to stand on and a portal with no hillside to cut are not
 * worth emitting, so a null height_sampler skips both even with emit_structures
 * on. Asserted on the two fixtures that would otherwise produce them.
 */
/**
 * A zebra is on the same paint plane as the stop line beside it.
 *
 * find_crossings() records Crossing::height as the raw carriageway SURFACE from
 * the vertical solve, and build_crossing() emits at exactly that height with no
 * offset of its own -- both by contract. The caller therefore has to put the
 * zebra on the paint plane, and nothing did.
 *
 * Two failures, one cause. With no height sampler the solve never runs, so every
 * Crossing::height is zero while the corridor is extruded at
 * CorridorConfig::base_height: the whole zebra is 50 mm UNDER the asphalt and is
 * never visible, though the crossing count still reports it. With a sampler the
 * height is the corridor's own surface, so the zebra is exactly COPLANAR with
 * the lane it crosses and z-fights at distance, which crossings.hpp calls the
 * worst kind of this bug to find.
 *
 * Both are checked on one fixture and one property: every marking vertex in the
 * network, zebra and line alike, is on one plane, and that plane is the
 * carriageway plus MarkingConfig::height_above_surface.
 */
TEST(P5P6Integration, a_zebra_sits_on_the_same_paint_plane_as_the_lines_beside_it) {
    const float ground = 20.0f;

    struct Case {
        const char* label;
        bool on_terrain;
    };
    const Case cases[] = { { "flat, no sampler", false }, { "solved on flat terrain", true } };

    for (const Case& c : cases) {
        RoadNetworkConfig cfg;
        cfg.solve_junctions = true;
        cfg.emit_markings = true;
        cfg.emit_crossings = true;
        if (c.on_terrain) cfg.height_sampler = p5::flat_sampler(ground);

        RoadNetworkBuilder builder;
        RoadNetwork network;
        if (!build_fixture("crossing.osm", cfg, builder, network)) continue;

        // The premise: a zebra really was emitted, or the plane check is vacuous.
        CHECK_TRUE(network.stats.crossings > 0);

        const double surface =
            (c.on_terrain ? static_cast<double>(ground) : 0.0) +
            static_cast<double>(cfg.corridor.base_height);
        const double plane = surface + static_cast<double>(cfg.markings.height_above_surface);

        size_t marking_vertices = 0;
        for (const RoadPiece& piece : network.pieces) {
            const std::vector<MaterialId> per_triangle = triangle_materials(piece.mesh);
            for (size_t t = 0; t < per_triangle.size(); ++t) {
                if (per_triangle[t] != MaterialId::Markings) continue;
                for (int k = 0; k < 3; ++k) {
                    const uint32_t index = piece.mesh.indices[t * 3 + k];
                    if (index >= piece.mesh.vertices.size()) continue;
                    ++marking_vertices;
                    const double y = static_cast<double>(piece.mesh.vertices[index].position.y);
                    if (std::fabs(y - plane) > 1e-3) {
                        stratum::test::report_failure(
                            __FILE__, __LINE__, "every marking is on the one paint plane",
                            std::string(c.label) + ": " + std::to_string(y) + " against " +
                                std::to_string(plane));
                        t = per_triangle.size();
                        break;
                    }
                }
            }
        }
        CHECK_TRUE(marking_vertices > 0);
    }
}

/**
 * The centre line's dashes march at one period along a street split into two ways.
 *
 * An OSM way ends at every `name`, `ref` or `maxspeed` change, and the graph
 * splits the street there. The node is plain degree 2: the profiles agree, so no
 * taper is built, no trim is written and the two ribbons meet flush. The paint
 * has to meet flush as well, and it only does if the builder carries a
 * longitudinal datum from one edge to the next. Without one the second edge
 * restarts its pattern at its own zero, and this fixture -- 93 m, ten whole
 * periods plus one dash -- butts a second dash straight onto the first, making a
 * single 6 m dash at the join.
 *
 * Asserted as an alternation of dash and gap over the whole street, which fails
 * on a merged dash and on a short gap alike.
 */
TEST(P5P6Integration, dashes_keep_one_period_across_a_plain_way_split) {
    const double first_length = 93.0;
    const double second_length = 60.0;

    stratum::osm::Road first = jt::make_road(1, {1, 2}, {{0.0, 0.0}, {first_length, 0.0}},
                                             stratum::osm::RoadType::Primary);
    first.lanes_forward = 1;
    first.lanes_backward = 1;

    stratum::osm::Road second =
        jt::make_road(2, {2, 3}, {{first_length, 0.0}, {first_length + second_length, 0.0}},
                      stratum::osm::RoadType::Primary);
    second.lanes_forward = 1;
    second.lanes_backward = 1;

    const ParsedOSMData data = jt::make_data({first, second});

    RoadNetworkConfig cfg;
    cfg.solve_junctions = true;
    cfg.emit_markings = true;
    cfg.emit_crossings = false;
    cfg.emit_structures = false;

    RoadNetworkBuilder builder;
    const RoadNetwork network = builder.build(data, cfg);
    CHECK_TRUE(network.pieces.size() >= 2);

    // Every distinct x a CENTRE LINE vertex sits at, in millimetres, over the
    // whole street. The road runs due east from the origin, so x is the arc
    // length along it and lateral is the local y.
    std::vector<long long> edges_mm;
    for (const RoadPiece& piece : network.pieces) {
        const std::vector<MaterialId> per_triangle = triangle_materials(piece.mesh);
        for (size_t t = 0; t < per_triangle.size(); ++t) {
            if (per_triangle[t] != MaterialId::Markings) continue;
            for (int k = 0; k < 3; ++k) {
                const uint32_t index = piece.mesh.indices[t * 3 + k];
                if (index >= piece.mesh.vertices.size()) continue;
                const glm::dvec2 local = jt::world_to_local(piece.mesh.vertices[index].position);
                if (std::fabs(local.y) > 0.2) continue;   // not the centre line
                edges_mm.push_back(static_cast<long long>(std::llround(local.x * 1000.0)));
            }
        }
    }
    std::sort(edges_mm.begin(), edges_mm.end());
    edges_mm.erase(std::unique(edges_mm.begin(), edges_mm.end()), edges_mm.end());

    CHECK_TRUE(edges_mm.size() >= 20);
    if (edges_mm.size() < 4) return;

    // The premise: the paint really does run on both sides of the join, so the
    // alternation below is a statement about the join and not about one edge.
    const long long join_mm = static_cast<long long>(std::llround(first_length * 1000.0));
    CHECK_TRUE(edges_mm.front() < join_mm - 1000);
    CHECK_TRUE(edges_mm.back() > join_mm + 1000);

    // Dash, gap, dash, gap, all the way down the street and through the join.
    const long long dash = static_cast<long long>(std::llround(cfg.markings.dash_length * 1000.0));
    const long long gap = static_cast<long long>(std::llround(cfg.markings.dash_gap * 1000.0));
    for (size_t i = 0; i + 1 < edges_mm.size(); ++i) {
        const long long step = edges_mm[i + 1] - edges_mm[i];
        const long long want = (i % 2u == 0u) ? dash : gap;
        if (std::llabs(step - want) > 2) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "the dash pattern keeps one period across the way split",
                "step " + std::to_string(step) + " mm at x " + std::to_string(edges_mm[i]) +
                    " mm, expected " + std::to_string(want));
            break;
        }
    }
}

TEST(P5P6Integration, structures_are_skipped_without_a_height_sampler) {
    for (const char* filename : {"bridge_abutment.osm", "bridge_over.osm", "tunnel.osm"}) {
        RoadNetworkBuilder builder;
        RoadNetwork network;
        if (!build_fixture(filename, everything_on(HeightSampler{}), builder, network)) continue;

        CHECK_EQ(network.stats.bridges, size_t{0});
        CHECK_EQ(network.stats.tunnels, size_t{0});
        for (const RoadPiece& piece : network.pieces) {
            CHECK_EQ(p5::triangles_with(piece.mesh, MaterialId::BridgeDeck).size(), size_t{0});
            CHECK_EQ(p5::triangles_with(piece.mesh, MaterialId::Parapet).size(), size_t{0});
        }
    }
}

/**
 * The dedup switch changes the sidewalk on the fixture that has a doubly-mapped
 * one, and nothing on the fixtures that do not.
 *
 * Its own switch-off, kept apart from the other three because it is the one that
 * subtracts from the cross-section rather than adding to it. tests/data has
 * exactly one fixture with a surveyed footway alongside a tagged carriageway, so
 * exactly one fixture may report a suppression.
 */
TEST(P5P6Integration, dedup_changes_only_the_fixture_that_has_a_duplicate) {
    size_t fixtures_with_a_duplicate = 0;

    for (const std::string& filename : p5::all_fixtures()) {
        RoadNetworkConfig without = everything_on(rolling());
        without.dedup.enabled = false;
        RoadNetworkBuilder off_builder;
        RoadNetwork off;
        if (!build_fixture(filename, without, off_builder, off)) continue;
        CHECK_EQ(off.stats.deduped_sidewalks, size_t{0});

        RoadNetworkBuilder on_builder;
        RoadNetwork on;
        if (!build_fixture(filename, everything_on(rolling()), on_builder, on)) continue;

        const SurfaceTally a = tally_surfaces(off);
        const SurfaceTally b = tally_surfaces(on);
        double walk_off = 0.0;
        double walk_on = 0.0;
        for (size_t m = 0; m < sizeof(kSurfaceMaterials) / sizeof(kSurfaceMaterials[0]); ++m) {
            if (kSurfaceMaterials[m] != MaterialId::Sidewalk) continue;
            walk_off = a.area[m];
            walk_on = b.area[m];
        }

        if (on.stats.deduped_sidewalks > 0) {
            ++fixtures_with_a_duplicate;
            CHECK_EQ(filename, std::string("sidewalk_dup.osm"));
            CHECK_TRUE(walk_on < walk_off);   // a synthesised strip stopped being built
        } else {
            CHECK_NEAR(walk_on, walk_off, 1e-6);
        }
    }

    CHECK_EQ(fixtures_with_a_duplicate, size_t{1});
}

// ============================================================================
// Reproducibility
// ============================================================================

/**
 * Two builds of one fixture with one config give bit-identical geometry.
 *
 * build() extrudes edges in parallel across the enkiTS scheduler, and the whole
 * design of that parallelism is that workers fill pre-sized slots indexed by
 * EdgeId and never push. P5 and P6 append into those same slots, and a pass that
 * appended from a shared work queue instead would produce a piece list that
 * differed run to run without ever being wrong in any single run. That defect is
 * invisible to every other test in this tree and fatal to the golden files P7
 * needs.
 */
TEST(P5P6Integration, a_build_is_reproducible_run_to_run) {
    for (const std::string& filename : p5::all_fixtures()) {
        RoadNetworkBuilder first_builder;
        RoadNetwork first;
        if (!build_fixture(filename, everything_on(rolling()), first_builder, first)) continue;

        RoadNetworkBuilder second_builder;
        RoadNetwork second;
        if (!build_fixture(filename, everything_on(rolling()), second_builder, second)) continue;

        CHECK_EQ(second.pieces.size(), first.pieces.size());
        CHECK_EQ(second.stats.vertices, first.stats.vertices);
        CHECK_EQ(second.stats.triangles, first.stats.triangles);
        CHECK_EQ(second.stats.markings_pieces, first.stats.markings_pieces);
        CHECK_EQ(second.stats.crossings, first.stats.crossings);
        CHECK_EQ(second.stats.bridges, first.stats.bridges);
        CHECK_EQ(second.stats.tunnels, first.stats.tunnels);
        CHECK_EQ(second.stats.deduped_sidewalks, first.stats.deduped_sidewalks);

        const size_t pieces = std::min(first.pieces.size(), second.pieces.size());
        for (size_t i = 0; i < pieces; ++i) {
            const Mesh& a = first.pieces[i].mesh;
            const Mesh& b = second.pieces[i].mesh;
            CHECK_EQ(second.pieces[i].edge, first.pieces[i].edge);
            CHECK_EQ(b.vertices.size(), a.vertices.size());
            CHECK_EQ(b.indices.size(), a.indices.size());
            CHECK_EQ(b.submeshes.size(), a.submeshes.size());

            bool identical = b.vertices.size() == a.vertices.size();
            for (size_t v = 0; identical && v < a.vertices.size(); ++v) {
                identical = key_of(a.vertices[v].position) == key_of(b.vertices[v].position);
            }
            if (!identical) {
                stratum::test::report_failure(__FILE__, __LINE__,
                                              "two builds give identical vertices",
                                              filename + " piece " + std::to_string(i));
                break;
            }
        }
    }
}

// ============================================================================
// Stats coherence
// ============================================================================

/**
 * The counts describe the geometry that came out.
 *
 * The stats are what an operator reads when an import looks wrong, so a count
 * that disagrees with the mesh is worse than no count: it sends the reader
 * looking in the wrong place. Each of these ties a number to something
 * observable in the pieces.
 */
TEST(P5P6Integration, the_p5_and_p6_counts_agree_with_the_geometry) {
    for (const std::string& filename : p5::all_fixtures()) {
        RoadNetworkBuilder builder;
        RoadNetwork network;
        if (!build_fixture(filename, everything_on(rolling()), builder, network)) continue;

        size_t painted_pieces = 0;
        size_t structural_pieces = 0;
        size_t vertices = 0;
        size_t triangles = 0;
        for (const RoadPiece& piece : network.pieces) {
            vertices += piece.mesh.vertices.size();
            triangles += piece.mesh.indices.size() / 3;
            if (!p5::triangles_with(piece.mesh, MaterialId::Markings).empty()) ++painted_pieces;
            if (!p5::triangles_with(piece.mesh, MaterialId::BridgeDeck).empty() ||
                !p5::triangles_with(piece.mesh, MaterialId::Parapet).empty()) {
                ++structural_pieces;
            }
        }

        CHECK_EQ(network.stats.pieces, network.pieces.size());
        CHECK_EQ(network.stats.vertices, vertices);
        CHECK_EQ(network.stats.triangles, triangles);
        CHECK_EQ(network.stats.markings_pieces, painted_pieces);
        CHECK_TRUE(network.stats.markings_pieces <= network.stats.pieces);
        CHECK_TRUE(network.stats.bridges <= structural_pieces + network.stats.tunnels);
        CHECK_TRUE(network.stats.junction_pieces <= network.stats.pieces);
        CHECK_TRUE(network.stats.build_ms >= 0.0);
    }
}
