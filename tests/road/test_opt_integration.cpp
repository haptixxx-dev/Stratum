/**
 * @file test_opt_integration.cpp
 * @brief The tessellation reduction inside the pipeline, and switched off again
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * tests/road/test_tessellation.cpp pins the two passes as entry points. This one
 * runs them where they actually live -- between the elevation solve and the
 * extrusion for the longitudinal pass, and inside the per-piece finish for the
 * lateral one -- and asks the questions that only fall out in place.
 *
 * ### Does it reduce
 *
 * Fewer triangles than with the switch off, over every fixture in tests/data,
 * with terrain and without. Not "no more than": strictly fewer in aggregate, or
 * the passes are running and achieving nothing and the Lucan numbers do not move.
 *
 * ### Does it break anything
 *
 * The reduction runs after the trims and after the elevation solve, and it moves
 * the stations that markings, crossings, bridge spans and carve requests are all
 * anchored to. So: finite positions, in-range indices, SubMesh ranges that still
 * tile the index buffer, and every MaterialKey that existed with the passes off
 * still present with them on. A crossing whose station vanished shows up here as
 * a missing Markings range rather than as a visibly detached zebra.
 *
 * ### Does the switch switch off
 *
 * RoadNetworkConfig::reduce_tessellation false must reproduce the previous
 * phase's output exactly -- the same bisectability contract solve_junctions,
 * emit_markings and emit_structures already carry. That is asserted three ways:
 * the triangle multiset is identical across two builds, every tessellation count
 * in RoadNetwork::Stats reports the documented no-op values, and changing the
 * TessellationConfig budgets with the flag off changes nothing at all.
 *
 * Run just this suite with:
 * @code
 *     ./stratum_tests OptIntegration
 * @endcode
 */

#include "framework.hpp"
#include "road/p5_p6_fixtures.hpp"
#include "road/p7_fixtures.hpp"

#include "osm/road/road_network_builder.hpp"
#include "renderer/mesh.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <set>
#include <string>
#include <vector>

namespace {

using stratum::MaterialKey;
using stratum::Mesh;
using stratum::SubMesh;
using stratum::osm::road::HeightSampler;
using stratum::osm::road::RoadNetwork;
using stratum::osm::road::RoadNetworkBuilder;
using stratum::osm::road::RoadNetworkConfig;
using stratum::osm::road::RoadPiece;

namespace p5 = stratum::test::p5;
namespace p7 = stratum::test::p7;
namespace jt = stratum::test::junction;

/// Rolling ground, the same sampler shape the P5/P6 and P7 sweeps use
HeightSampler rolling() {
    return [](double x, double y) {
        return static_cast<float>(9.0 * std::sin(x / 130.0) + 6.0 * std::cos(y / 95.0) + 20.0);
    };
}

/// Everything the previous phases deliver, with the new passes OFF
RoadNetworkConfig without_reduction(const HeightSampler& sampler) {
    RoadNetworkConfig cfg;
    cfg.height_sampler = sampler;
    cfg.reduce_tessellation = false;
    return cfg;
}

/// The same, with both passes ON
RoadNetworkConfig with_reduction(const HeightSampler& sampler) {
    RoadNetworkConfig cfg;
    cfg.height_sampler = sampler;
    cfg.reduce_tessellation = true;
    return cfg;
}

/// Build one fixture, returning false when it did not parse
bool build_fixture(const std::string& fixture, const RoadNetworkConfig& cfg, RoadNetwork& out) {
    const auto parsed = jt::parse_fixture(fixture.c_str());
    if (!parsed) return false;
    RoadNetworkBuilder builder;
    out = builder.build(*parsed, cfg);
    return true;
}

/// Triangles summed over every piece of a network
size_t total_triangles(const RoadNetwork& network) {
    size_t total = 0;
    for (const RoadPiece& piece : network.pieces) total += p7::triangle_count(piece.mesh);
    return total;
}

/// Vertices summed over every piece of a network
size_t total_vertices(const RoadNetwork& network) {
    size_t total = 0;
    for (const RoadPiece& piece : network.pieces) total += piece.mesh.vertices.size();
    return total;
}

/**
 * @brief Every distinct MaterialKey the network emits
 *
 * The PAIR, not the slot. Two Asphalt ranges with different variants are two
 * materials, and a pass that merged them would look like a triangle reduction
 * while actually putting cobblestone under an asphalt texture.
 */
std::set<uint32_t> material_keys_of(const RoadNetwork& network) {
    std::set<uint32_t> keys;
    for (const RoadPiece& piece : network.pieces) {
        for (const SubMesh& range : piece.mesh.effective_submeshes()) {
            if (range.index_count == 0) continue;
            keys.insert(MaterialKey{range.material, range.variant}.packed());
        }
    }
    return keys;
}

/// Every piece's mesh reduced to one sorted multiset of content keys
std::vector<p7::TriKey> network_content(const RoadNetwork& network) {
    std::vector<p7::TriKey> out;
    for (const RoadPiece& piece : network.pieces) {
        const std::vector<p7::TriKey> keys = p7::triangle_multiset(piece.mesh);
        out.insert(out.end(), keys.begin(), keys.end());
    }
    std::sort(out.begin(), out.end());
    return out;
}

/// Finite, in range, and still tiling: applied to every piece of a network
void check_network_is_sane(const RoadNetwork& network, const std::string& label,
                           const char* file, int line) {
    for (size_t i = 0; i < network.pieces.size(); ++i) {
        const Mesh& mesh = network.pieces[i].mesh;
        if (!p7::mesh_is_finite(mesh)) {
            stratum::test::report_failure(file, line, "no NaN in the reduced network",
                                          label + ": piece " + std::to_string(i));
            return;
        }
        if (!p7::indices_are_sane(mesh)) {
            stratum::test::report_failure(file, line, "indices stay in range",
                                          label + ": piece " + std::to_string(i));
            return;
        }
        std::string reason;
        if (!p7::submeshes_tile_exactly(mesh, reason)) {
            stratum::test::report_failure(file, line, "submeshes still tile the index buffer",
                                          label + ": piece " + std::to_string(i) + ": " + reason);
            return;
        }
    }
}

#define CHECK_NETWORK_SANE(network, label) \
    check_network_is_sane((network), (label), __FILE__, __LINE__)

/// Every fixture in tests/data
std::vector<std::string> fixtures() { return p5::all_fixtures(); }

} // namespace

// ============================================================================
// Does it reduce
// ============================================================================

/**
 * With the passes on, the whole network has fewer triangles than with them off,
 * and nothing about it is broken.
 *
 * Swept over every fixture on rolling terrain, because the reduction runs AFTER
 * the elevation solve and the junction trims and therefore has to survive both.
 * The aggregate is asserted strictly: a per-fixture bound would let a build where
 * every pass silently no-opped still pass on "no more than before".
 */
TEST(OptIntegration, reduction_removes_triangles_and_breaks_nothing) {
    const HeightSampler sampler = rolling();

    size_t triangles_off = 0;
    size_t triangles_on = 0;
    size_t fixtures_built = 0;

    for (const std::string& fixture : fixtures()) {
        RoadNetwork off;
        RoadNetwork on;
        if (!build_fixture(fixture, without_reduction(sampler), off)) continue;
        if (!build_fixture(fixture, with_reduction(sampler), on)) continue;
        ++fixtures_built;

        CHECK_NETWORK_SANE(on, fixture);

        const size_t a = total_triangles(off);
        const size_t b = total_triangles(on);
        triangles_off += a;
        triangles_on += b;

        if (b > a) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "reduction never adds triangles",
                fixture + ": " + std::to_string(a) + " off, " + std::to_string(b) + " on");
        }

        // The passes may not delete a road. Every fixture that produced pieces
        // still produces the same pieces; only their contents get cheaper.
        CHECK_EQ(on.pieces.size(), off.pieces.size());
        CHECK_TRUE(b > 0);
    }

    if (fixtures_built == 0) {
        stratum::test::report_failure(__FILE__, __LINE__, "at least one fixture parsed",
                                      "no fixture in tests/data built");
        return;
    }

    if (!(triangles_on < triangles_off)) {
        stratum::test::report_failure(
            __FILE__, __LINE__, "the reduction actually reduces",
            std::to_string(triangles_on) + " triangles with the passes on versus " +
                std::to_string(triangles_off) + " with them off, over " +
                std::to_string(fixtures_built) + " fixtures");
    }
}

/// Fewer stations means fewer vertex columns, so the vertex count falls too
TEST(OptIntegration, reduction_removes_vertices) {
    const HeightSampler sampler = rolling();

    size_t vertices_off = 0;
    size_t vertices_on = 0;
    for (const std::string& fixture : fixtures()) {
        RoadNetwork off;
        RoadNetwork on;
        if (!build_fixture(fixture, without_reduction(sampler), off)) continue;
        if (!build_fixture(fixture, with_reduction(sampler), on)) continue;
        vertices_off += total_vertices(off);
        vertices_on += total_vertices(on);
    }

    CHECK_TRUE(vertices_off > 0);
    CHECK_TRUE(vertices_on <= vertices_off);
    if (!(vertices_on < vertices_off)) {
        stratum::test::report_failure(
            __FILE__, __LINE__, "the reduction removes vertex columns",
            std::to_string(vertices_on) + " vertices on versus " +
                std::to_string(vertices_off) + " off");
    }
}

/**
 * Every MaterialKey present with the passes off is present with them on.
 *
 * A material that vanished is a surface the renderer stops drawing, and it is the
 * quietest possible failure: the road still looks like a road, with no kerb, or
 * with no paint. Compared as the (slot, variant) PAIR, because merging two
 * variants of one slot loses a material without losing a slot.
 */
TEST(OptIntegration, every_material_key_survives_the_reduction) {
    const HeightSampler sampler = rolling();

    for (const std::string& fixture : fixtures()) {
        RoadNetwork off;
        RoadNetwork on;
        if (!build_fixture(fixture, without_reduction(sampler), off)) continue;
        if (!build_fixture(fixture, with_reduction(sampler), on)) continue;

        const std::set<uint32_t> before = material_keys_of(off);
        const std::set<uint32_t> after = material_keys_of(on);

        for (const uint32_t key : before) {
            if (after.count(key) == 0) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "the material key survived the reduction",
                    fixture + ": packed key " + std::to_string(key) +
                        " is gone with reduce_tessellation on");
            }
        }

        // Nor may it invent one: a variant nothing tagged has no business appearing.
        for (const uint32_t key : after) {
            if (before.count(key) == 0) {
                stratum::test::report_failure(
                    __FILE__, __LINE__, "the reduction invents no material",
                    fixture + ": packed key " + std::to_string(key) +
                        " appeared with reduce_tessellation on");
            }
        }
    }
}

/// The reduction must work with no terrain under it as well as with terrain
TEST(OptIntegration, reduction_works_on_flat_ground_too) {
    size_t triangles_off = 0;
    size_t triangles_on = 0;

    for (const std::string& fixture : fixtures()) {
        RoadNetwork off;
        RoadNetwork on;
        if (!build_fixture(fixture, without_reduction(nullptr), off)) continue;
        if (!build_fixture(fixture, with_reduction(nullptr), on)) continue;

        CHECK_NETWORK_SANE(on, fixture + " (flat)");
        triangles_off += total_triangles(off);
        triangles_on += total_triangles(on);
    }

    CHECK_TRUE(triangles_off > 0);
    CHECK_TRUE(triangles_on < triangles_off);
}

// ============================================================================
// The counts report what happened
// ============================================================================

/**
 * RoadNetwork::Stats is the only window a caller has onto how much the passes
 * removed, and the editor prints it. A count that does not move is a pass that
 * did not run.
 */
TEST(OptIntegration, the_stats_describe_the_reduction) {
    const HeightSampler sampler = rolling();

    size_t stations_before = 0;
    size_t stations_after = 0;
    size_t quads_merged = 0;

    for (const std::string& fixture : fixtures()) {
        RoadNetwork on;
        if (!build_fixture(fixture, with_reduction(sampler), on)) continue;

        CHECK_TRUE(on.stats.stations_after <= on.stats.stations_before);
        CHECK_TRUE(on.stats.triangles <= on.stats.triangles_before_tess);
        CHECK_TRUE(on.stats.triangles_before_tess > 0);

        stations_before += on.stats.stations_before;
        stations_after += on.stats.stations_after;
        quads_merged += on.stats.quads_merged;
    }

    CHECK_TRUE(stations_before > 0);
    if (!(stations_after < stations_before)) {
        stratum::test::report_failure(
            __FILE__, __LINE__, "the longitudinal pass dropped stations",
            std::to_string(stations_after) + " stations after versus " +
                std::to_string(stations_before) + " before");
    }
    if (quads_merged == 0) {
        stratum::test::report_failure(__FILE__, __LINE__, "the lateral pass merged something",
                                      "quads_merged is zero over every fixture");
    }
}

// ============================================================================
// The switch switches off
// ============================================================================

/**
 * All new flags off reproduces the previous output exactly.
 *
 * The documented no-op state, asserted field by field: stations_after equals
 * stations_before rather than zero, because the stations exist either way, while
 * quads_merged is zero and triangles equals triangles_before_tess because neither
 * pass ran.
 */
TEST(OptIntegration, the_switch_off_state_is_the_documented_no_op) {
    const HeightSampler sampler = rolling();

    for (const std::string& fixture : fixtures()) {
        RoadNetwork off;
        if (!build_fixture(fixture, without_reduction(sampler), off)) continue;

        CHECK_EQ(off.stats.quads_merged, size_t{0});
        CHECK_EQ(off.stats.stations_after, off.stats.stations_before);
        CHECK_EQ(off.stats.triangles, off.stats.triangles_before_tess);
        CHECK_NETWORK_SANE(off, fixture + " (reduction off)");
    }
}

/**
 * Two builds with the passes off are byte-for-byte the same network.
 *
 * The build is parallel across edges and across pieces, and a parallel stage that
 * writes anything shared shows up as output that varies run to run. This is the
 * same reproducibility contract every earlier phase carries, re-checked with the
 * new switch in the config because a new stage is a new place to push to a shared
 * vector.
 */
TEST(OptIntegration, the_switch_off_state_is_reproducible) {
    const HeightSampler sampler = rolling();

    for (const std::string& fixture : fixtures()) {
        RoadNetwork first;
        RoadNetwork second;
        if (!build_fixture(fixture, without_reduction(sampler), first)) continue;
        if (!build_fixture(fixture, without_reduction(sampler), second)) continue;

        CHECK_EQ(first.pieces.size(), second.pieces.size());
        if (network_content(first) != network_content(second)) {
            stratum::test::report_failure(__FILE__, __LINE__,
                                          "two builds with the passes off are identical",
                                          fixture + " varies run to run");
        }
    }
}

/**
 * With the flag off, the TessellationConfig budgets are inert.
 *
 * Turning a budget to a value that would strip a road to two stations must change
 * NOTHING while reduce_tessellation is false. Without this the switch is only
 * half a switch: a caller who bisects a defect by clearing the flag would still
 * be running whatever the config said.
 */
TEST(OptIntegration, the_budgets_are_inert_while_the_flag_is_off) {
    const HeightSampler sampler = rolling();

    for (const std::string& fixture : fixtures()) {
        RoadNetworkConfig defaults = without_reduction(sampler);

        RoadNetworkConfig extreme = without_reduction(sampler);
        extreme.tessellation.max_chord_deviation = 100.0;
        extreme.tessellation.max_span_length = 1e9;
        extreme.tessellation.preserve_feature_stations = false;
        extreme.tessellation.merge_coplanar_strips = true;
        extreme.tessellation.coplanar_normal_epsilon = 0.0;

        RoadNetwork a;
        RoadNetwork b;
        if (!build_fixture(fixture, defaults, a)) continue;
        if (!build_fixture(fixture, extreme, b)) continue;

        CHECK_EQ(a.pieces.size(), b.pieces.size());
        if (network_content(a) != network_content(b)) {
            stratum::test::report_failure(
                __FILE__, __LINE__, "the budgets do nothing while the flag is off",
                fixture + ": changing TessellationConfig changed the output with "
                          "reduce_tessellation false");
        }
    }
}

/**
 * With the flag on, the budgets are NOT inert.
 *
 * The mirror of the test above, and the one that stops it passing vacuously: if
 * the passes were never wired into the builder at all, both networks would be
 * identical in both directions and only this assertion would notice.
 */
TEST(OptIntegration, the_budgets_bite_while_the_flag_is_on) {
    const HeightSampler sampler = rolling();

    size_t tight_total = 0;
    size_t loose_total = 0;

    for (const std::string& fixture : fixtures()) {
        RoadNetworkConfig tight = with_reduction(sampler);
        tight.tessellation.max_chord_deviation = 0.001;
        tight.tessellation.max_span_length = 8.0;
        tight.tessellation.merge_coplanar_strips = false;

        RoadNetworkConfig loose = with_reduction(sampler);
        loose.tessellation.max_chord_deviation = 0.5;
        loose.tessellation.max_span_length = 1000.0;

        RoadNetwork a;
        RoadNetwork b;
        if (!build_fixture(fixture, tight, a)) continue;
        if (!build_fixture(fixture, loose, b)) continue;

        CHECK_NETWORK_SANE(a, fixture + " (tight)");
        CHECK_NETWORK_SANE(b, fixture + " (loose)");

        tight_total += total_triangles(a);
        loose_total += total_triangles(b);
    }

    CHECK_TRUE(tight_total > 0);
    if (!(loose_total < tight_total)) {
        stratum::test::report_failure(
            __FILE__, __LINE__, "a looser budget produces less geometry",
            std::to_string(loose_total) + " triangles loose versus " +
                std::to_string(tight_total) + " tight");
    }
}

/**
 * The reduction is reproducible too.
 *
 * The longitudinal pass runs per edge and the lateral pass per piece, both inside
 * the parallel stages, so both are new opportunities for a shared write. The
 * build order guarantee is what makes the golden dumps meaningful.
 */
TEST(OptIntegration, the_reduction_is_reproducible) {
    const HeightSampler sampler = rolling();

    for (const std::string& fixture : fixtures()) {
        RoadNetwork first;
        RoadNetwork second;
        if (!build_fixture(fixture, with_reduction(sampler), first)) continue;
        if (!build_fixture(fixture, with_reduction(sampler), second)) continue;

        CHECK_EQ(first.pieces.size(), second.pieces.size());
        CHECK_EQ(first.stats.stations_after, second.stats.stations_after);
        CHECK_EQ(first.stats.quads_merged, second.stats.quads_merged);
        if (network_content(first) != network_content(second)) {
            stratum::test::report_failure(__FILE__, __LINE__,
                                          "two reduced builds are identical",
                                          fixture + " varies run to run");
        }
    }
}
