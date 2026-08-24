/**
 * @file corridor.cpp
 * @brief Implementation of the RoadProfile-along-Centerline strip extruder
 *
 * The sweep is three nested walks and nothing else:
 *
 *   1. Lateral: walk RoadProfile::strips left to right from
 *      RoadProfile::left_edge_offset(), DECREASING the lateral coordinate by each
 *      strip's width, because positive lateral is to the LEFT of travel.
 *   2. Longitudinal: for each strip, emit its own pair of vertex columns, one
 *      vertex per station per column.
 *   3. Banding: triangulate the quad between consecutive stations with one fixed
 *      winding pattern that is correct for every strip kind.
 *
 * Two conventions are load-bearing and are stated here because getting either
 * one backwards produces geometry that is invisible from the only angle anyone
 * ever looks at a road from.
 *
 * ### Lateral direction
 *
 * Station::normal points LEFT of travel and offset_point() adds `normal *
 * lateral`, so a LARGER lateral coordinate is FURTHER LEFT. Walking the profile
 * left to right therefore walks the lateral coordinate DOWNWARDS:
 *
 *     lat_left  = running lateral coordinate
 *     lat_right = lat_left - strip.width
 *
 * ### Winding
 *
 * The renderer's front face is counter-clockwise. With L the strip's left column
 * and R its right column, the band between stations i and i+1 is
 *
 *     (L_i, R_i, R_{i+1})  and  (L_i, R_{i+1}, L_{i+1})
 *
 * Worked through the (x, y_2d) -> (x, height, -y_2d) mapping this yields +Y for
 * a horizontal strip, and for a CurbFace the normal pointing away from the
 * RAISED side -- towards the carriageway for the curb of a raised sidewalk,
 * which is the face a driver and a camera actually see. The pattern is never
 * flipped per side: the profile's left-to-right strip ordering already mirrors
 * the two curbs, so the left curb's height falls left-to-right while the right
 * curb's rises, and the one pattern covers both.
 *
 * ### Welding
 *
 * Adjacent strips share edge POSITIONS but never vertices. This is forced by the
 * frozen UV convention: U restarts at 0 on each strip's own left edge, so the
 * shared boundary carries two different U values and cannot be one vertex. It is
 * also what keeps a curb crisp, since each strip then owns its own face normal.
 * P7 does the welding that is actually safe.
 *
 * Everything here lives in stratum_core: no SDL, no ImGui, no rendering API.
 */

#include "osm/road/corridor.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace stratum::osm::road {

namespace {

// ============================================================================
// Tolerances
// ============================================================================

/// Squared length of the raw face cross product below which a triangle is dropped
constexpr double kDegenerateCrossSq = 1e-16;

/// Strip widths at or below this count as zero, metres
constexpr float kZeroWidth = 1e-6f;

/// Strip height differences at or below this count as zero, metres
constexpr float kZeroHeight = 1e-6f;

/// Consecutive outline points closer than this are collapsed, metres
constexpr double kOutlineWeldEpsilon = 1e-6;

/**
 * @brief Ring size past which the exact O(n^2) self-intersection test is skipped
 *
 * The cheap fold-back test still runs at every size, so a hairpin is still
 * reported on a long road; only the exhaustive pairwise check is bounded.
 */
constexpr size_t kMaxExactOutlinePoints = 2048;

// ============================================================================
// Helpers
// ============================================================================

/**
 * @brief The codebase-wide 2D-to-3D mapping, Y up
 *
 * (x, y_2d) -> (x, height, -y_2d). Matches mesh_builder.cpp; changing it here
 * would put roads on a different plane from every other OSM feature.
 */
[[nodiscard]] inline glm::vec3 to_world(const glm::dvec2& p, double height) {
    return glm::vec3(static_cast<float>(p.x),
                     static_cast<float>(height),
                     static_cast<float>(-p.y));
}

/// 2D cross product, positive when b turns left of a
[[nodiscard]] inline double cross2(const glm::dvec2& a, const glm::dvec2& b) {
    return a.x * b.y - a.y * b.x;
}

/**
 * @brief Relative tolerance below which a cross product counts as exactly zero
 *
 * The determinants in segments_cross() are areas, so the tolerance has to scale
 * with the two lengths that produced them or it means nothing at one road size
 * and everything at another. 1e-12 of that product puts the tolerated offset far
 * below any distance a road can express: on a 5 m edge measured against a 30 m
 * one it is 3e-11 m.
 */
constexpr double kCollinearRelEpsilon = 1e-12;

/**
 * @brief Proper crossing test for two 2D segments
 *
 * Proper only: shared endpoints and collinear overlap do not count. A ring edge
 * pair that merely touches is not a bowtie, and a dead-straight road would
 * otherwise report collinear neighbours as intersections.
 *
 * "Collinear" has to be decided with a tolerance, not against exact zero. A miter
 * point sits precisely ON the offset line of both of its legs -- that is what a
 * miter IS -- so every corner puts a run of exactly collinear points in the ring,
 * and their determinants come out as rounding noise of either sign. Testing those
 * against exact zero reports a crossing on a perfectly ordinary bend.
 */
[[nodiscard]] bool segments_cross(const glm::dvec2& a, const glm::dvec2& b,
                                  const glm::dvec2& c, const glm::dvec2& d) {
    const glm::dvec2 ab = b - a;
    const glm::dvec2 cd = d - c;

    const double len_ab = std::sqrt(glm::dot(ab, ab));
    const double len_cd = std::sqrt(glm::dot(cd, cd));

    const auto span = [](const glm::dvec2& p, const glm::dvec2& q) {
        return std::sqrt(std::max(glm::dot(p, p), glm::dot(q, q)));
    };
    const double eps_ab = kCollinearRelEpsilon * len_ab * span(c - a, d - a);
    const double eps_cd = kCollinearRelEpsilon * len_cd * span(a - c, b - c);

    const auto snap = [](double value, double eps) {
        return (std::abs(value) <= eps) ? 0.0 : value;
    };

    const double d1 = snap(cross2(ab, c - a), eps_ab);
    const double d2 = snap(cross2(ab, d - a), eps_ab);
    const double d3 = snap(cross2(cd, a - c), eps_cd);
    const double d4 = snap(cross2(cd, b - c), eps_cd);

    const bool straddles_ab = ((d1 > 0.0) && (d2 < 0.0)) || ((d1 < 0.0) && (d2 > 0.0));
    const bool straddles_cd = ((d3 > 0.0) && (d4 < 0.0)) || ((d3 < 0.0) && (d4 > 0.0));
    return straddles_ab && straddles_cd;
}

/**
 * @brief Whether an offset edge run folds back against the direction of travel
 *
 * On a hairpin the inner offset reverses: the offset point at station i+1 sits
 * BEHIND the one at station i measured along the band. That reversal is the
 * signature of a fold, and it is O(n) to find, so it runs whatever the ring size.
 *
 * The band's own chord is the direction the step is measured against, not the
 * mean of the two station tangents. A mitred joint's tangent is the BISECTOR of
 * its two legs, so it points half the turn away from the band it opens, and
 * measuring against it reports an ordinary sharp corner -- one whose offset
 * still advances along the band -- as folded.
 *
 * @param cl      Stations the offset was taken from
 * @param lateral Signed lateral coordinate of the offset edge, positive to the left
 * @return True when some consecutive pair of offset points runs backwards
 */
[[nodiscard]] bool offset_folds_back(const Centerline& cl, double lateral) {
    for (size_t i = 0; i + 1 < cl.stations.size(); ++i) {
        const Station& a = cl.stations[i];
        const Station& b = cl.stations[i + 1];

        // A bevel pair shares a position; its zero-length band has no chord to
        // measure against and its wedge is bounded at the station instead.
        if (a.arclength >= b.arclength) {
            continue;
        }

        const glm::dvec2 step = offset_point(b, lateral) - offset_point(a, lateral);
        const glm::dvec2 chord = b.position - a.position;
        if (glm::dot(step, chord) < 0.0) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Exhaustive proper-crossing test over a closed ring
 *
 * Adjacent edges are skipped because they legitimately share an endpoint. Bounded
 * by kMaxExactOutlinePoints so a very long road does not pay O(n^2).
 *
 * @param ring Closed ring, first point NOT repeated at the end
 * @return True when two non-adjacent ring edges cross properly
 */
[[nodiscard]] bool ring_self_intersects(const std::vector<glm::dvec2>& ring) {
    const size_t n = ring.size();
    if (n < 4 || n > kMaxExactOutlinePoints) {
        return false;
    }

    for (size_t i = 0; i < n; ++i) {
        const glm::dvec2& a = ring[i];
        const glm::dvec2& b = ring[(i + 1) % n];

        for (size_t j = i + 2; j < n; ++j) {
            // Edge n-1 wraps to edge 0, so those two are adjacent as well.
            if (i == 0 && j == n - 1) {
                continue;
            }
            const glm::dvec2& c = ring[j];
            const glm::dvec2& d = ring[(j + 1) % n];
            if (segments_cross(a, b, c, d)) {
                return true;
            }
        }
    }
    return false;
}

} // namespace

// ============================================================================
// UV tiling
// ============================================================================

/**
 * @brief The frozen UV Convention table
 *
 * Invariant: these are metres of real surface per texture repeat, never atlas
 * sub-rects, so texel density is identical on a 3 m alley and a 30 m motorway.
 */
UVTiling uv_tiling(MaterialId material) {
    switch (material) {
        case MaterialId::Asphalt:    return UVTiling{ 8.0f, 8.0f };
        case MaterialId::Concrete:   return UVTiling{ 4.0f, 4.0f };
        case MaterialId::BridgeDeck: return UVTiling{ 4.0f, 4.0f };
        case MaterialId::Sidewalk:   return UVTiling{ 2.0f, 2.0f };
        case MaterialId::Curb:       return UVTiling{ 0.5f, 2.0f };
        case MaterialId::Gravel:     return UVTiling{ 4.0f, 4.0f };
        case MaterialId::Dirt:       return UVTiling{ 4.0f, 4.0f };
        case MaterialId::Grass:      return UVTiling{ 4.0f, 4.0f };
        case MaterialId::Parapet:    return UVTiling{ 2.0f, 2.0f };

        // Building slots. A road strip never carries one -- surface_material() and
        // strip_material() cannot return them -- so they take the neutral 1x1 m
        // entry alongside Default, and the switch stays exhaustive.
        case MaterialId::Wall:
        case MaterialId::Roof:

        // Markings is an atlas; P5 writes explicit sub-rects and must not scale
        // by this neutral entry.
        case MaterialId::Default:
        case MaterialId::Markings:
        case MaterialId::Count:      break;
    }
    return UVTiling{ 1.0f, 1.0f };
}

// ============================================================================
// Extrusion
// ============================================================================

/**
 * @brief Sweep a cross-section along a centerline
 *
 * Invariant: every emitted triangle has non-zero area and counter-clockwise
 * front-face winding seen from outside the surface; the submesh ranges tile the
 * whole index buffer with one contiguous range per material; and the outline, when
 * emitted, is a closed CCW ring in the centerline's own 2D metres.
 */
Corridor build_corridor(const Centerline& cl,
                        const RoadProfile& profile,
                        const CorridorConfig& cfg) {
    Corridor out;

    // Degenerate input is a normal occurrence in an OSM extract, not an error:
    // a way that welds down to one point, or a profile a tag rule could not
    // resolve. Return nothing and let the caller skip the edge.
    if (!cl.is_valid() || !profile.is_valid()) {
        return out;
    }

    const size_t station_count = cl.stations.size();

    // A mis-sized elevation solve degrades to a flat road rather than to mangled
    // geometry, per CorridorConfig::station_heights.
    bool use_station_heights = cfg.station_heights.size() == station_count;
    if (!cfg.station_heights.empty() && !use_station_heights) {
        spdlog::warn("build_corridor: station_heights has {} entries for {} stations; "
                     "ignoring and using base_height {:.3f}",
                     cfg.station_heights.size(), station_count, cfg.base_height);
        use_station_heights = false;
    }

    const auto surface_height = [&](size_t station) -> double {
        return use_station_heights ? static_cast<double>(cfg.station_heights[station])
                                   : static_cast<double>(cfg.base_height);
    };

    out.length = cl.stations.back().arclength - cl.stations.front().arclength;

    Mesh& mesh = out.mesh;

    // Geometric normals, accumulated per vertex from the faces of that vertex's
    // OWN strip only. Strips do not share vertices, so a curb face can never
    // smear its normal into the lane beside it.
    std::vector<glm::dvec3> normal_accum;

    // Reused per strip so the columns are not reallocated for every strip.
    std::vector<uint32_t> column_left;
    std::vector<uint32_t> column_right;
    column_left.reserve(station_count);
    column_right.reserve(station_count);

    const auto emit_triangle = [&](uint32_t i0, uint32_t i1, uint32_t i2) {
        const glm::dvec3 p0(mesh.vertices[i0].position);
        const glm::dvec3 p1(mesh.vertices[i1].position);
        const glm::dvec3 p2(mesh.vertices[i2].position);

        // Unnormalised, so the accumulation is area weighted.
        const glm::dvec3 face = glm::cross(p1 - p0, p2 - p0);
        if (glm::dot(face, face) <= kDegenerateCrossSq) {
            return;     // zero-area, e.g. the centreline column across a bevel pair
        }

        mesh.indices.push_back(i0);
        mesh.indices.push_back(i1);
        mesh.indices.push_back(i2);

        normal_accum[i0] += face;
        normal_accum[i1] += face;
        normal_accum[i2] += face;
    };

    // ------------------------------------------------------------------------
    // Kerb drop roles.
    //
    // Which strip boundaries a dropped kerb owns, found once from the profile's
    // own structure rather than from a strip index the caller had to supply. See
    // CorridorConfig::kerb_top_height for the rule and for why the sidewalk's
    // inboard edge is included and its outboard edge is not.
    // ------------------------------------------------------------------------
    const size_t strip_count = profile.strips.size();
    const bool modulating = static_cast<bool>(cfg.kerb_top_height);

    std::vector<uint8_t> drop_left(modulating ? strip_count : 0u, 0u);
    std::vector<uint8_t> drop_right(modulating ? strip_count : 0u, 0u);
    std::vector<uint8_t> drop_side_left(modulating ? strip_count : 0u, 1u);

    if (modulating) {
        for (size_t i = 0; i < strip_count; ++i) {
            const Strip& face = profile.strips[i];
            if (face.kind != StripKind::CurbFace) {
                continue;
            }
            if (std::fabs(face.height_left - face.height_right) <= kZeroHeight) {
                continue;   // a face with no rise is not a kerb
            }

            // The raised edge names the side: a kerb whose top is on its LEFT is
            // the kerb of the left-of-travel footway, and outboard from it runs
            // towards the front of the strip list.
            const bool side_left = face.height_left > face.height_right;
            const int step = side_left ? -1 : 1;

            (side_left ? drop_left : drop_right)[i] = 1u;
            drop_side_left[i] = side_left ? 1u : 0u;

            // Every CurbTop outboard of the face moves as a whole: it is the top
            // of the same kerb.
            auto next = [&](size_t from) -> size_t {
                return static_cast<size_t>(static_cast<long long>(from) + step);
            };

            size_t j = next(i);
            while (j < strip_count && profile.strips[j].kind == StripKind::CurbTop) {
                drop_left[j] = 1u;
                drop_right[j] = 1u;
                drop_side_left[j] = side_left ? 1u : 0u;
                j = next(j);
            }

            // The first strip beyond the kerb takes the drop on its INBOARD edge
            // only, and becomes the crossfall ramp. For a left kerb the inboard
            // edge is the strip's RIGHT edge, because the lateral walk runs from
            // the outboard left towards the carriageway.
            if (j < strip_count) {
                (side_left ? drop_right : drop_left)[j] = 1u;
                drop_side_left[j] = side_left ? 1u : 0u;
            }
        }
    }

    // ------------------------------------------------------------------------
    // Lateral walk: left to right is DECREASING lateral, because positive
    // lateral is to the LEFT of travel.
    // ------------------------------------------------------------------------
    double lateral = static_cast<double>(profile.left_edge_offset());

    for (size_t strip_index = 0; strip_index < strip_count; ++strip_index) {
        const Strip& strip = profile.strips[strip_index];
        const double lat_left = lateral;
        const double lat_right = lat_left - static_cast<double>(strip.width);
        lateral = lat_right;

        // Advance the lateral coordinate first, above, then decide whether to
        // emit. A suppressed strip must still consume its width or everything
        // outboard of it shifts inwards.
        const bool is_curb_face = strip.kind == StripKind::CurbFace;
        if (is_curb_face && !cfg.emit_curb_faces) {
            continue;
        }

        const float height_delta = std::fabs(strip.height_right - strip.height_left);
        const bool has_width = strip.width > kZeroWidth;
        const bool is_vertical_riser = is_curb_face && height_delta > kZeroHeight;
        if (!has_width && !is_vertical_riser) {
            continue;   // a zero-width strip that is not a pure vertical riser
        }

        const UVTiling tiling = uv_tiling(strip.material);
        const float u_scale = (tiling.u_metres > 0.0f) ? tiling.u_metres : 1.0f;
        const float v_scale = (tiling.v_metres > 0.0f) ? tiling.v_metres : 1.0f;

        // U per column. Constant along the road on purpose: the NOMINAL strip
        // width is used, not the mitre-stretched one, so the texture does not
        // shear at every corner. V alone carries the along-road parameterisation.
        float u_left = 0.0f;
        float u_right = 0.0f;
        if (is_vertical_riser) {
            // Vertical face: U runs UP the face from the lower edge. Keyed on
            // is_vertical_riser rather than the kind, so a CurbFace authored with
            // equal heights falls back to the lateral convention instead of
            // collapsing both columns onto U = 0 and producing a null tangent.
            const float lower = std::min(strip.height_left, strip.height_right);
            u_left = (strip.height_left - lower) / u_scale;
            u_right = (strip.height_right - lower) / u_scale;
        } else {
            u_right = strip.width / u_scale;
        }

        column_left.clear();
        column_right.clear();

        // Nothing on this strip moves with a drop, so the whole modulation is one
        // branch outside the station loop for the overwhelming majority of strips.
        const bool modulate_left = modulating && drop_left[strip_index] != 0u;
        const bool modulate_right = modulating && drop_right[strip_index] != 0u;
        const bool side_left_of_travel = !modulating || drop_side_left[strip_index] != 0u;

        for (size_t i = 0; i < station_count; ++i) {
            const Station& s = cl.stations[i];
            const double base = surface_height(i);
            const float v = static_cast<float>(s.arclength / static_cast<double>(v_scale));

            // ONE evaluation per station and side, per the contract on
            // CorridorConfig::kerb_top_height: two boundaries that have to agree
            // must not be handed two answers.
            double height_left = static_cast<double>(strip.height_left);
            double height_right = static_cast<double>(strip.height_right);
            if (modulate_left) {
                height_left = cfg.kerb_top_height(s.arclength, side_left_of_travel, height_left);
            }
            if (modulate_right) {
                height_right = cfg.kerb_top_height(s.arclength, side_left_of_travel, height_right);
            }

            // A dropped face is shorter, so its texture must cover less of the
            // face rather than being stretched over what is left.
            float u_l = u_left;
            float u_r = u_right;
            if (is_vertical_riser && (modulate_left || modulate_right)) {
                const double lower = std::min(height_left, height_right);
                u_l = static_cast<float>((height_left - lower) / static_cast<double>(u_scale));
                u_r = static_cast<float>((height_right - lower) / static_cast<double>(u_scale));
            }

            Vertex vl{};
            vl.position = to_world(offset_point(s, lat_left), base + height_left);
            vl.normal = glm::vec3(0.0f, 1.0f, 0.0f);
            vl.uv = glm::vec2(u_l, v);
            vl.color = glm::vec4(1.0f);

            Vertex vr{};
            vr.position = to_world(offset_point(s, lat_right), base + height_right);
            vr.normal = glm::vec3(0.0f, 1.0f, 0.0f);
            vr.uv = glm::vec2(u_r, v);
            vr.color = glm::vec4(1.0f);

            column_left.push_back(static_cast<uint32_t>(mesh.vertices.size()));
            mesh.vertices.push_back(vl);
            normal_accum.emplace_back(0.0);

            column_right.push_back(static_cast<uint32_t>(mesh.vertices.size()));
            mesh.vertices.push_back(vr);
            normal_accum.emplace_back(0.0);
        }

        // --------------------------------------------------------------------
        // Band triangulation. One winding pattern for every strip kind; see the
        // file header for why it is never mirrored per side.
        // --------------------------------------------------------------------
        const uint32_t strip_index_start = static_cast<uint32_t>(mesh.indices.size());

        for (size_t i = 0; i + 1 < station_count; ++i) {
            const uint32_t l0 = column_left[i];
            const uint32_t r0 = column_right[i];
            const uint32_t l1 = column_left[i + 1];
            const uint32_t r1 = column_right[i + 1];

            emit_triangle(l0, r0, r1);
            emit_triangle(l0, r1, l1);
        }

        const uint32_t added = static_cast<uint32_t>(mesh.indices.size()) - strip_index_start;
        if (added == 0u) {
            continue;
        }

        // Keyed on the PAIR, not the slot. Two Asphalt strips with different
        // variants are two materials, and folding them into one range would put
        // one texture under both.
        const MaterialKey key = strip.key();
        if (!mesh.submeshes.empty() && mesh.submeshes.back().material == key.material &&
            mesh.submeshes.back().variant == key.variant) {
            mesh.submeshes.back().index_count += added;
        } else {
            mesh.submeshes.push_back(SubMesh{ strip_index_start, added, key.material,
                                              key.variant });
        }
    }

    // ------------------------------------------------------------------------
    // Resolve the accumulated geometric normals.
    // ------------------------------------------------------------------------
    for (size_t i = 0; i < mesh.vertices.size(); ++i) {
        const glm::dvec3& n = normal_accum[i];
        const double len_sq = glm::dot(n, n);
        if (len_sq > 0.0) {
            const glm::dvec3 unit = n / std::sqrt(len_sq);
            mesh.vertices[i].normal = glm::vec3(static_cast<float>(unit.x),
                                                static_cast<float>(unit.y),
                                                static_cast<float>(unit.z));
        }
        // else: the vertex is referenced by no surviving triangle, so its
        // initial +Y normal is never sampled.
    }

    // One contiguous range per material, ascending by MaterialId.
    mesh.sort_submeshes_by_material();
    mesh.compute_bounds();
    mesh.compute_tangents();

    // ------------------------------------------------------------------------
    // Corridor footprint for the P3 terrain carve.
    //
    // RIGHT edge forward from the first station to the last, then LEFT edge
    // backward. With positive lateral to the left that traversal is CCW in the
    // 2D plane. The first point is not repeated at the end.
    // ------------------------------------------------------------------------
    if (cfg.emit_outline) {
        const double left_edge = static_cast<double>(profile.left_edge_offset());
        const double right_edge = left_edge - static_cast<double>(profile.total_width());

        if (profile.total_width() > kZeroWidth) {
            std::vector<glm::dvec2>& ring = out.outline;
            ring.reserve(station_count * 2u);

            const auto push_unique = [&ring](const glm::dvec2& p) {
                if (!ring.empty()) {
                    const glm::dvec2 d = p - ring.back();
                    if (glm::dot(d, d) <= kOutlineWeldEpsilon * kOutlineWeldEpsilon) {
                        return;
                    }
                }
                ring.push_back(p);
            };

            for (size_t i = 0; i < station_count; ++i) {
                push_unique(offset_point(cl.stations[i], right_edge));
            }
            for (size_t i = station_count; i-- > 0;) {
                push_unique(offset_point(cl.stations[i], left_edge));
            }

            // The ring closes implicitly, so a final point coincident with the
            // first would be a zero-length closing edge.
            while (ring.size() >= 2) {
                const glm::dvec2 d = ring.back() - ring.front();
                if (glm::dot(d, d) > kOutlineWeldEpsilon * kOutlineWeldEpsilon) {
                    break;
                }
                ring.pop_back();
            }

            if (ring.size() < 3) {
                ring.clear();
            } else {
                // A hairpin tighter than the profile half-width folds the inner
                // offset back through itself. Report it rather than handing P3 a
                // bowtie whose winding number is meaningless.
                out.outline_self_intersects =
                    offset_folds_back(cl, left_edge) ||
                    offset_folds_back(cl, right_edge) ||
                    ring_self_intersects(ring);

                if (out.outline_self_intersects) {
                    spdlog::warn("build_corridor: corridor outline self-intersects over {:.1f} m "
                                 "with a {:.2f} m profile; terrain carving must treat it as "
                                 "non-simple",
                                 out.length, profile.total_width());
                }
            }
        }
    }

    return out;
}

} // namespace stratum::osm::road
