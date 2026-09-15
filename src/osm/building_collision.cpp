// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file building_collision.cpp
 * @brief Implementation of the footprint-to-collision-prism build
 *
 * The header argues why the hull is built from the Building rather than derived
 * from the render mesh. This file is about the three decisions inside the build
 * that are not obvious from the outside, because each is a place where the
 * straightforward version produces a hull that looks right and is not closed.
 *
 * ### 1. Winding is forced, never trusted
 *
 * Building documents its outer ring as counter-clockwise and its holes as
 * clockwise. An extract does not honour that. The parser builds rings from OSM
 * ways, a multipolygon relation names its inner and outer members without
 * promising either a direction, and a way reversed by a mapper years ago is
 * still a valid way.
 *
 * Every wall here takes its outward direction from its ring's winding, so a
 * hole that arrives counter-clockwise gets a facade pointing into the masonry
 * and a footprint that arrives clockwise turns the whole building inside out.
 * Neither is visible in a triangle count or a bounding box; both are visible as
 * a collider a player walks straight through. So both rings are re-wound from
 * their own signed area before anything reads them.
 *
 * ### 2. One vertex set, shared by the walls and both caps
 *
 * Two vertices per ring point -- one at the cap, one at the floor -- and the
 * walls, the roof cap and the floor all index off that same set. Nothing is
 * duplicated at a seam.
 *
 * That is what makes the output closed by INDEX rather than merely closed in
 * space. A consumer gets a manifold without running a weld pass first, and the
 * edge-parity invariant that proves the hull has no hole is checkable directly
 * on the index buffer. The cost is that a vertex has one normal: the vertices
 * are authored with the facade's outward horizontal normal, which is right for
 * the walls and wrong for the caps. Nothing reads them -- a physics engine takes
 * the geometric normal from the positions -- and giving the caps their own would
 * double the vertex count and break the property the sharing exists to provide.
 *
 * ### 3. Cap triangles are wound from their own area, not from the triangulator
 *
 * earcut normalises its input rings internally and emits triangles in the
 * orientation that normalisation left, which is a detail of the vendored library
 * rather than a documented contract. A hull whose roof faces down is invisible
 * to a physics engine that culls backfaces and fully solid to one that does not,
 * which is the worst kind of bug to carry: it reproduces on one engine and not
 * the next.
 *
 * So each cap triangle is re-wound from its own plan-view signed area. In local
 * metres a counter-clockwise triangle faces UP once to_world() has flipped Z,
 * because that flip negates the plan-view signed area; the floor is the same
 * triangle with two corners swapped.
 *
 * ### Nothing, rather than half a hull
 *
 * A triangulation that fails -- a self-intersecting footprint is the usual cause
 * -- leaves the prism with walls and no cap. That is a bucket: anything that
 * falls in stands on the floor inside a wall it cannot pass, and nothing above
 * ever lands on the building. A missing collider is noticed the first time
 * somebody walks at the building; a bucket is noticed in a playtest, weeks
 * later, with nothing in the geometry to say why. So a failed cap returns an
 * empty mesh and says so in the log.
 */

#include "osm/building_collision.hpp"

#include "osm/coordinates.hpp"

#include <mapbox/earcut.hpp>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

// Earcut adapter for glm::dvec2. Duplicated verbatim from mesh_builder.cpp and
// junction_polygon.cpp rather than shared: it is an explicit specialisation, so
// every translation unit that triangulates a dvec2 ring needs its own, and the
// definitions must stay token-identical.
namespace mapbox {
namespace util {

template <>
struct nth<0, glm::dvec2> {
    inline static double get(const glm::dvec2& t) { return t.x; }
};

template <>
struct nth<1, glm::dvec2> {
    inline static double get(const glm::dvec2& t) { return t.y; }
};

} // namespace util
} // namespace mapbox

namespace stratum::osm {
namespace {

// ============================================================================
// Constants
// ============================================================================

/// Two ring points closer than this in plan are the same point, metres
constexpr double kPointMergeEpsilon = 1e-6;

/// Shortest prism worth building, metres. Below it the caps are the same surface.
constexpr float kMinPrismHeight = 1e-4f;

/// Vector length below which a direction is treated as having cancelled out
constexpr float kDegenerateDirection = 1e-6f;

// ============================================================================
// Small helpers
// ============================================================================

/// 2D local metres to Y-up world space. The Z flip is the whole convention.
[[nodiscard]] glm::vec3 to_world(const glm::dvec2& p, float y) {
    return { static_cast<float>(p.x), y, static_cast<float>(-p.y) };
}

/**
 * @brief Drop a ring's non-finite points, its coincident neighbours and its closing duplicate
 *
 * An OSM way closes by repeating its first node, and a repeated point makes a
 * zero-width wall quad whose two triangles have no orientation. Dropping it here
 * rather than skipping it at emission time keeps the ring indices contiguous,
 * which is what lets an earcut index name a wall vertex without translation.
 */
[[nodiscard]] std::vector<glm::dvec2> clean_ring(const std::vector<glm::dvec2>& ring) {
    std::vector<glm::dvec2> out;
    out.reserve(ring.size());

    for (const glm::dvec2& p : ring) {
        // A non-finite coordinate is not a point. Letting one through would put a
        // NaN in the vertex buffer, and a NaN in a collision mesh takes the whole
        // broadphase with it rather than failing locally.
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
            continue;
        }
        if (!out.empty() && glm::distance(p, out.back()) <= kPointMergeEpsilon) {
            continue;
        }
        out.push_back(p);
    }

    while (out.size() > 1 && glm::distance(out.front(), out.back()) <= kPointMergeEpsilon) {
        out.pop_back();
    }
    return out;
}

/**
 * @brief Re-wind a ring from its own signed area
 *
 * @param ring              Ring to re-wind in place
 * @param counter_clockwise true for the outer ring, false for a hole
 */
void force_winding(std::vector<glm::dvec2>& ring, bool counter_clockwise) {
    const bool is_ccw = geometry::polygon_area(ring) >= 0.0;
    if (is_ccw != counter_clockwise) {
        std::reverse(ring.begin(), ring.end());
    }
}

/**
 * @brief Outward horizontal facade normal at each point of an already-wound ring
 *
 * A ring wound counter-clockwise in local metres has `(dy, -dx)` pointing out of
 * it along the edge `(dx, dy)`, and to_world()'s Z flip turns that into
 * `(dy, 0, dx)`. A hole is wound the other way, so the same expression points
 * INTO the courtyard -- which is the direction that facade faces, and the reason
 * one routine covers both rings.
 */
[[nodiscard]] std::vector<glm::vec3> ring_normals(const std::vector<glm::dvec2>& ring) {
    const size_t n = ring.size();
    std::vector<glm::vec3> edge(n, glm::vec3(0.0f));

    for (size_t i = 0; i < n; ++i) {
        const glm::dvec2 d = ring[(i + 1u) % n] - ring[i];
        const glm::vec3 candidate(static_cast<float>(d.y), 0.0f, static_cast<float>(d.x));
        const float len = glm::length(candidate);
        edge[i] = len > kDegenerateDirection ? candidate / len : glm::vec3(0.0f);
    }

    std::vector<glm::vec3> out(n, glm::vec3(1.0f, 0.0f, 0.0f));
    for (size_t i = 0; i < n; ++i) {
        const glm::vec3 sum = edge[i] + edge[(i + n - 1u) % n];
        const float len = glm::length(sum);

        // A spike where the ring doubles back on itself cancels its two edge
        // normals exactly, and there is then no outward direction at that point.
        // The outgoing edge's normal stands in, because a zero normal exports as
        // an unlit facade and reads as a broken mesh rather than a sharp one.
        glm::vec3 chosen = len > kDegenerateDirection ? sum / len : edge[i];
        if (glm::length(chosen) <= kDegenerateDirection) {
            chosen = glm::vec3(1.0f, 0.0f, 0.0f);
        }
        out[i] = chosen;
    }
    return out;
}

} // namespace

// ============================================================================
// Entry point
// ============================================================================

Mesh build_building_collision_mesh(const Building& building, const BuildingCollisionConfig& cfg) {
    Mesh out;

    std::vector<glm::dvec2> outer = clean_ring(building.footprint);
    if (outer.size() < 3u || std::fabs(geometry::polygon_area(outer)) < kMinRingArea) {
        return out;   // no interior to extrude
    }
    force_winding(outer, /*counter_clockwise=*/true);

    const float floor_y = -cfg.foundation_depth;
    const float cap_y = cfg.cap_height.value_or(building.height);
    if (!std::isfinite(floor_y) || !std::isfinite(cap_y)
        || !(cap_y - floor_y > kMinPrismHeight)) {
        return out;   // the two caps are the same surface, or the prism is inside out
    }

    // ------------------------------------------------------------------------
    // 1. Rings, in the order the triangulator wants them: outer first, then the
    //    holes. That same order indexes the vertex arrays below, so an earcut
    //    index names a ring point directly and needs no translation table.
    // ------------------------------------------------------------------------
    std::vector<std::vector<glm::dvec2>> rings;
    rings.reserve(building.holes.size() + 1u);
    rings.push_back(std::move(outer));

    for (const std::vector<glm::dvec2>& hole : building.holes) {
        std::vector<glm::dvec2> ring = clean_ring(hole);
        if (ring.size() < 3u || std::fabs(geometry::polygon_area(ring)) < kMinRingArea) {
            // A degenerate hole handed to the triangulator produces a
            // triangulation whose boundary is no longer the rings, and the hull
            // stops being closed. Dropping it costs a courtyard nobody can fit
            // in; keeping it costs the invariant.
            continue;
        }
        force_winding(ring, /*counter_clockwise=*/false);
        rings.push_back(std::move(ring));
    }

    // ------------------------------------------------------------------------
    // 2. Triangulate the cap once. Both caps are this triangulation; the floor
    //    is the same triangles with two corners swapped.
    // ------------------------------------------------------------------------
    const std::vector<uint32_t> cap = mapbox::earcut<uint32_t>(rings);

    // Emptiness is NOT the failure mode to guard against here, and testing for it
    // alone let the file's own guarantee go unmet.
    //
    // earcut almost never gives up outright on a bad ring. cureLocalIntersections()
    // DELETES points and hands back a smaller triangulation whose boundary is no
    // longer the ring it was given -- so the walls get built around a cap that does
    // not span them, and the result is exactly the open-topped shell this function
    // promises never to emit. OSM supplies such rings routinely: a closed way that
    // revisits a node passes Way::is_closed(), and OSMParser::process_buildings()
    // applies no simplicity test beyond a point count. A figure-eight footprint
    // came out of here as 24 triangles with 8 unpaired edges.
    //
    // Count instead. A triangulation of a simple polygon with V total ring points
    // and H holes has exactly V + 2H - 2 triangles, always. Anything less means
    // earcut discarded input, and a hull built on discarded input is worse than no
    // hull at all: a physics body a character falls through is harder to notice
    // than one that was never there.
    size_t total_points = 0u;
    for (const std::vector<glm::dvec2>& ring : rings) {
        total_points += ring.size();
    }
    const size_t expected = 3u * (total_points + 2u * (rings.size() - 1u) - 2u);

    if (cap.size() != expected) {
        spdlog::warn("build_building_collision_mesh: footprint of way {} did not triangulate "
                     "cleanly ({} rings, {} points, {} triangles where {} were required). The "
                     "ring is most likely self-intersecting or a hole is not strictly inside "
                     "the outer ring. Returning nothing rather than a walled shell with no cap.",
                     building.osm_id, rings.size(), total_points, cap.size() / 3u,
                     expected / 3u);
        return out;
    }

    // ------------------------------------------------------------------------
    // 3. Two vertices per ring point, cap ring first and floor ring second, so
    //    the floor vertex of ring point i is at i + floor_base for every i.
    // ------------------------------------------------------------------------
    std::vector<glm::dvec2> points;
    std::vector<glm::vec3> normals;
    for (const std::vector<glm::dvec2>& ring : rings) {
        const std::vector<glm::vec3> ring_normal = ring_normals(ring);
        points.insert(points.end(), ring.begin(), ring.end());
        normals.insert(normals.end(), ring_normal.begin(), ring_normal.end());
    }

    const uint32_t floor_base = static_cast<uint32_t>(points.size());

    out.vertices.reserve(points.size() * 2u);
    const float ring_y[2] = { cap_y, floor_y };
    for (const float y : ring_y) {
        for (size_t i = 0; i < points.size(); ++i) {
            Vertex v{};
            v.position = to_world(points[i], y);
            v.normal = normals[i];
            // A planar projection in metres. A physics mesh has no use for UVs;
            // this is here so that they are deterministic and non-degenerate
            // rather than because anything reads them.
            v.uv = glm::vec2(static_cast<float>(points[i].x), static_cast<float>(points[i].y));
            out.vertices.push_back(v);
        }
    }

    // ------------------------------------------------------------------------
    // 4. Walls, wound outward. cross(bot_j - bot_i, top_j - bot_i) is the ring
    //    edge crossed with the rise, which is the outward normal for a ring
    //    wound as force_winding() left it -- outward from the building for the
    //    footprint, into the courtyard for a hole.
    //
    //    The diagonal bot_i-top_j is shared by the quad's two triangles and
    //    cancels, so each quad contributes its four ring edges exactly once and
    //    the prism stays manifold.
    // ------------------------------------------------------------------------
    out.indices.reserve(points.size() * 6u + cap.size() * (cfg.close_bottom ? 2u : 1u));

    uint32_t ring_base = 0u;
    for (const std::vector<glm::dvec2>& ring : rings) {
        const uint32_t n = static_cast<uint32_t>(ring.size());
        for (uint32_t i = 0u; i < n; ++i) {
            const uint32_t top_i = ring_base + i;
            const uint32_t top_j = ring_base + ((i + 1u) % n);
            const uint32_t bot_i = floor_base + top_i;
            const uint32_t bot_j = floor_base + top_j;

            out.indices.push_back(bot_i);
            out.indices.push_back(bot_j);
            out.indices.push_back(top_j);

            out.indices.push_back(bot_i);
            out.indices.push_back(top_j);
            out.indices.push_back(top_i);
        }
        ring_base += n;
    }

    // ------------------------------------------------------------------------
    // 5. The two caps. See the file comment for why the winding comes from each
    //    triangle's own signed area rather than from the triangulator.
    // ------------------------------------------------------------------------
    for (size_t t = 0; t + 2u < cap.size(); t += 3u) {
        const uint32_t a = cap[t];
        uint32_t b = cap[t + 1u];
        uint32_t c = cap[t + 2u];
        if (a >= floor_base || b >= floor_base || c >= floor_base) {
            // Unreachable for a well-formed triangulation. Skipping just this
            // triangle would leave a hole in the cap, which is the one output
            // this function refuses to produce, so the whole hull goes instead.
            spdlog::warn("build_building_collision_mesh: way {} triangulated to an index past "
                         "its own ring set. Returning nothing.", building.osm_id);
            out.clear();
            return out;
        }

        const glm::dvec2& pa = points[a];
        const glm::dvec2& pb = points[b];
        const glm::dvec2& pc = points[c];
        const double area2 = (pb.x - pa.x) * (pc.y - pa.y) - (pc.x - pa.x) * (pb.y - pa.y);
        if (area2 < 0.0) {
            std::swap(b, c);
        }

        out.indices.push_back(a);
        out.indices.push_back(b);
        out.indices.push_back(c);

        if (cfg.close_bottom) {
            out.indices.push_back(floor_base + a);
            out.indices.push_back(floor_base + c);
            out.indices.push_back(floor_base + b);
        }
    }

    // ------------------------------------------------------------------------
    // 6. No material ranges. An empty submeshes vector is the implicit single
    //    MaterialId::Default range, which is what osm::road::build_collision_mesh()
    //    leaves behind too: a physics mesh has one surface type here, or it gets
    //    its surface types from a mechanism that does not live in the geometry.
    // ------------------------------------------------------------------------
    out.submeshes.clear();
    out.compute_bounds();

    assert(out.indices.size() % 3u == 0u && "building collision hull is not a triangle list");
    for ([[maybe_unused]] uint32_t index : out.indices) {
        assert(index < out.vertices.size() && "building collision hull index is out of range");
    }

    spdlog::debug("build_building_collision_mesh: way {} -> {} triangles, {} vertices "
                  "({} rings, cap at {:.2f} m, floor at {:.2f} m)",
                  building.osm_id, out.indices.size() / 3u, out.vertices.size(), rings.size(),
                  cap_y, floor_y);

    return out;
}

} // namespace stratum::osm
