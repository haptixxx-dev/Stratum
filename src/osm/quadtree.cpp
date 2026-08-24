#include "osm/quadtree.hpp"
#include "osm/mesh_builder.hpp"
#include <TaskScheduler.h>
#include <spdlog/spdlog.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <utility>

namespace stratum::osm {

namespace {

/**
 * @brief Append @p src into @p dst, keeping @p src's material ranges intact
 *
 * Mesh::append() attributes everything it copies to a single MaterialId, which
 * would flatten a road piece's asphalt, curb and sidewalk ranges into one slot
 * and undo the whole point of P0.3. This walks the source's effective ranges
 * instead and extends or opens a matching range in the destination, so the
 * material slots survive the merge of many pieces into one leaf mesh.
 *
 * Submesh bookkeeping otherwise follows Mesh::append() exactly: a destination
 * that has indices but no submeshes first materialises its implicit whole-mesh
 * range, so pre-existing geometry keeps its identity.
 */
void append_preserving_materials(Mesh& dst, const Mesh& src) {
    if (src.vertices.empty() || src.indices.empty()) return;

    if (!dst.indices.empty() && dst.submeshes.empty()) {
        dst.submeshes.push_back({0u, static_cast<uint32_t>(dst.indices.size()),
                                 MaterialId::Default, uint16_t{0}});
    }

    const uint32_t base_vertex = static_cast<uint32_t>(dst.vertices.size());
    dst.vertices.insert(dst.vertices.end(), src.vertices.begin(), src.vertices.end());
    for (const auto& v : src.vertices) {
        dst.bounds.expand(v.position);
    }

    for (const SubMesh& range : src.effective_submeshes()) {
        if (range.index_count == 0) continue;

        const size_t end = static_cast<size_t>(range.index_offset) + range.index_count;
        if (end > src.indices.size()) continue;  // malformed range; drop rather than over-read

        const uint32_t start = static_cast<uint32_t>(dst.indices.size());

        // Grow geometrically, never to the exact size needed. reserve() allocates
        // EXACTLY what is asked for, and the loop below then fills the buffer to
        // capacity precisely, so an exact reserve per range reallocates and copies
        // the whole leaf index buffer on every single append -- quadratic in the
        // ranges routed to one leaf, and seconds of frozen UI on a city extract.
        const size_t needed = dst.indices.size() + range.index_count;
        if (needed > dst.indices.capacity()) {
            dst.indices.reserve(std::max(needed, dst.indices.capacity() * 2));
        }
        for (size_t i = range.index_offset; i < end; ++i) {
            dst.indices.push_back(src.indices[i] + base_vertex);
        }

        // Coalesced on the FULL key. Two ranges that share a slot but differ in
        // variant are different materials -- see MaterialKey -- so merging them
        // here would silently repaint one of them with the other's textures.
        SubMesh* back = dst.submeshes.empty() ? nullptr : &dst.submeshes.back();
        if (back && back->material == range.material && back->variant == range.variant &&
            back->index_offset + back->index_count == start) {
            back->index_count += range.index_count;
        } else {
            dst.submeshes.push_back({start, range.index_count, range.material, range.variant});
        }
    }
}


/**
 * @brief Distance from a local-space point to the PERIMETER of a rectangle
 *
 * Identical in meaning to the private one in lod_chunk.cpp, and deliberately so:
 * measure_seam_bands() has to measure with the SAME ruler build_lock_set() locks
 * with, or the band it computes does not mean what the crack argument says it
 * means. Zero on the rectangle, growing in both directions.
 */
[[nodiscard]] double rect_perimeter_distance(const glm::dvec2& point,
                                             const glm::dvec2& rect_min,
                                             const glm::dvec2& rect_max) {
    const glm::dvec2 centre = (rect_min + rect_max) * 0.5;
    const glm::dvec2 half = (rect_max - rect_min) * 0.5;
    const glm::dvec2 q = glm::abs(point - centre) - half;

    const double outside_x = std::max(q.x, 0.0);
    const double outside_y = std::max(q.y, 0.0);
    const double outside = std::sqrt(outside_x * outside_x + outside_y * outside_y);
    const double inside = std::min(std::max(q.x, q.y), 0.0);

    return outside > 0.0 ? outside : -inside;
}

/// How far @p point lies OUTSIDE the rectangle; zero when it is inside
[[nodiscard]] double rect_outside_distance(const glm::dvec2& point,
                                           const glm::dvec2& rect_min,
                                           const glm::dvec2& rect_max) {
    const glm::dvec2 centre = (rect_min + rect_max) * 0.5;
    const glm::dvec2 half = (rect_max - rect_min) * 0.5;
    const glm::dvec2 q = glm::max(glm::abs(point - centre) - half, glm::dvec2(0.0));
    return std::sqrt(q.x * q.x + q.y * q.y);
}

/// The leaf's own 2D rectangle, in the local metres road geometry is solved in
void leaf_rect(const QuadTreeNode& node, glm::dvec2& out_min, glm::dvec2& out_max) {
    out_min = glm::dvec2(node.center.x - node.half_size, node.center.y - node.half_size);
    out_max = glm::dvec2(node.center.x + node.half_size, node.center.y + node.half_size);
}

/// World (x, height, -y) back to the local (x, y) the quadtree indexes in
[[nodiscard]] glm::dvec2 world_to_local(const glm::vec3& p) {
    return glm::dvec2(static_cast<double>(p.x), -static_cast<double>(p.z));
}

/**
 * @brief Slack added to a measured seam band before it is locked, metres
 *
 * The band is compared against a distance recomputed from a float position in a
 * different translation unit, so the two can disagree in the last bit. A
 * centimetre of slack costs nothing -- the band is metres wide -- and removes
 * the only way a vertex measured as exactly on the limit could come out free.
 */
constexpr double kSeamBandSlack = 0.01;

/// Chunks below which the LOD build stays on the calling thread
constexpr size_t kParallelMinChunks = 8;

/// Sentinel for "this source vertex has not been copied into this leaf yet"
constexpr uint32_t kNoVertex = std::numeric_limits<uint32_t>::max();

/**
 * @brief Append one triangle of @p src into @p dst, copying vertices on demand
 *
 * @param dst    Destination leaf mesh
 * @param src    Source piece mesh
 * @param tri    Triangle index into @p src
 * @param remap  Source vertex -> destination vertex, kNoVertex for uncopied
 */
bool append_triangle(Mesh& dst, const Mesh& src, size_t tri, std::vector<uint32_t>& remap) {
    const uint32_t s0 = src.indices[tri * 3];
    const uint32_t s1 = src.indices[tri * 3 + 1];
    const uint32_t s2 = src.indices[tri * 3 + 2];

    // Validated before anything is written. Bailing out halfway would leave one
    // or two orphan indices in dst and shear every triangle after them.
    const size_t n = src.vertices.size();
    if (s0 >= n || s1 >= n || s2 >= n) return false;

    for (const uint32_t si : {s0, s1, s2}) {
        uint32_t& di = remap[si];
        if (di == kNoVertex) {
            di = static_cast<uint32_t>(dst.vertices.size());
            dst.vertices.push_back(src.vertices[si]);
            dst.bounds.expand(src.vertices[si].position);
        }
        dst.indices.push_back(di);
    }
    return true;
}

} // namespace

// ============================================================================
// Initialization
// ============================================================================

void QuadTree::init(const BoundingBox& bounds) {
    clear();

    if (!bounds.is_valid()) {
        spdlog::warn("QuadTree: Invalid bounds, cannot initialize");
        return;
    }

    double width = bounds.width_meters();
    double height = bounds.height_meters();

    // Use half the larger dimension to make a square root node
    double half = std::max(width, height) / 2.0;

    m_root = std::make_unique<QuadTreeNode>();
    m_root->center = glm::dvec2(0.0, 0.0); // local coords are centered
    m_root->half_size = half;
    m_root->node_id = m_next_id++;
    m_root->depth = 0;
    compute_3d_bounds(m_root.get());

    spdlog::info("QuadTree: Initialized root node, half_size={:.0f}m", half);
}

void QuadTree::init(const ParsedOSMData& data) {
    clear();

    glm::dvec2 mn(std::numeric_limits<double>::max());
    glm::dvec2 mx(std::numeric_limits<double>::lowest());
    bool any = false;

    const auto accumulate = [&](const std::vector<glm::dvec2>& pts) {
        for (const auto& p : pts) {
            mn = glm::min(mn, p);
            mx = glm::max(mx, p);
            any = true;
        }
    };

    for (const auto& r : data.roads) accumulate(r.polyline);
    for (const auto& b : data.buildings) {
        accumulate(b.footprint);
        for (const auto& h : b.holes) accumulate(h);
    }
    for (const auto& a : data.areas) {
        accumulate(a.polygon);
        for (const auto& h : a.holes) accumulate(h);
    }

    if (!any) {
        spdlog::warn("QuadTree: no feature geometry, cannot initialize");
        return;
    }

    const glm::dvec2 centre = (mn + mx) * 0.5;
    // Square root node, with a small margin so features exactly on the boundary
    // still land inside it.
    double half = std::max(mx.x - mn.x, mx.y - mn.y) * 0.5 * 1.01;
    half = std::max(half, QuadTreeConfig::MIN_NODE_SIZE);

    m_root = std::make_unique<QuadTreeNode>();
    m_root->center = centre;
    m_root->half_size = half;
    m_root->node_id = m_next_id++;
    m_root->depth = 0;
    compute_3d_bounds(m_root.get());

    spdlog::info("QuadTree: root centred on ({:.0f}, {:.0f}), half_size={:.0f}m "
                 "(feature extent {:.0f} x {:.0f}m)",
                 centre.x, centre.y, half, mx.x - mn.x, mx.y - mn.y);
}

bool QuadTree::get_focus(glm::vec3& out_centre, float& out_radius) {
    std::vector<QuadTreeNode*> leaves;
    collect_leaves(m_root.get(), leaves);

    // Feature-count-weighted mean of leaf centres.
    glm::dvec3 accum(0.0);
    double total = 0.0;
    for (const auto* leaf : leaves) {
        const double w = static_cast<double>(leaf->feature_count());
        if (w <= 0.0) continue;
        const glm::vec3 c = (leaf->bounds_min + leaf->bounds_max) * 0.5f;
        accum += glm::dvec3(c) * w;
        total += w;
    }
    if (total <= 0.0) return false;

    const glm::dvec3 centre = accum / total;
    out_centre = glm::vec3(centre);

    // Radius covering the bulk of the mass: grow outward until most features are
    // enclosed, so a few distant strays cannot drag the framing out to the horizon.
    constexpr double MASS_FRACTION = 0.9;

    std::vector<std::pair<double, double>> by_distance;  // (distance, weight)
    by_distance.reserve(leaves.size());
    for (const auto* leaf : leaves) {
        const double w = static_cast<double>(leaf->feature_count());
        if (w <= 0.0) continue;
        const glm::vec3 c = (leaf->bounds_min + leaf->bounds_max) * 0.5f;
        const glm::dvec3 d = glm::dvec3(c) - centre;
        by_distance.emplace_back(std::sqrt(d.x * d.x + d.z * d.z), w);
    }
    std::sort(by_distance.begin(), by_distance.end());

    double running = 0.0;
    double radius = 0.0;
    for (const auto& [dist, w] : by_distance) {
        running += w;
        radius = dist;
        if (running >= total * MASS_FRACTION) break;
    }

    out_radius = static_cast<float>(std::max(radius, QuadTreeConfig::MIN_NODE_SIZE));
    return true;
}

void QuadTree::clear() {
    {
        std::lock_guard<std::mutex> lock(m_pending_mutex);
        m_pending_builds.clear();
    }
    m_root.reset();
    m_next_id = 0;
    m_road_lod_stats = RoadLodStats{};
}

// ============================================================================
// Data Assignment
// ============================================================================

void QuadTree::assign_data(const ParsedOSMData& data) {
    if (!m_root) return;

    for (const auto& road : data.roads) {
        if (!road.polyline.empty()) {
            insert_road(m_root.get(), road);
        }
    }

    for (const auto& building : data.buildings) {
        if (!building.footprint.empty()) {
            insert_building(m_root.get(), building);
        }
    }

    for (const auto& area : data.areas) {
        if (!area.polygon.empty()) {
            insert_area(m_root.get(), area);
        }
    }

    // Recompute 3D bounds bottom-up after all insertions
    // (bounds need to encompass actual building heights)
    recompute_bounds(m_root.get());

    spdlog::info("QuadTree: Assigned data — {} leaves, {} roads, {} buildings, {} areas",
                 leaf_count(), total_roads(), total_buildings(), total_areas());
}

// ============================================================================
// Road Geometry Assignment
// ============================================================================

void QuadTree::set_chunk_lod(bool enabled, const road::ChunkLodConfig& cfg) {
    m_chunk_lod_enabled = enabled;
    m_chunk_lod_config = cfg;
}

void QuadTree::assign_pieces_by_anchor(std::vector<road::RoadPiece>& pieces,
                                       std::vector<QuadTreeNode*>& touched) {
    size_t assigned = 0;
    size_t skipped = 0;
    size_t triangles = 0;

    for (auto& piece : pieces) {
        if (piece.mesh.vertices.empty() || piece.mesh.indices.empty()) {
            ++skipped;
            continue;
        }

        // Route by the anchor only. The piece is never split, so part of the
        // road may lie outside this leaf; recompute_bounds() grows the leaf AABB
        // to cover it.
        //
        // RoadPiece::edge is deliberately not consulted. A junction piece carries
        // kInvalidId and is anchored at its graph node, so branching on
        // provenance here would be a way to get junctions wrong and no way to get
        // anything right.
        QuadTreeNode* leaf = find_leaf(piece.anchor);
        if (!leaf) {
            ++skipped;
            continue;
        }

        if (leaf->road_meshes.empty()) {
            leaf->road_meshes.emplace_back();
            touched.push_back(leaf);
        }

        append_preserving_materials(leaf->road_meshes.front(), piece.mesh);
        triangles += piece.mesh.indices.size() / 3;
        ++assigned;

        // Release the source as it is consumed, so peak memory holds one copy of
        // the road network rather than two. Mesh::clear() empties the vectors but
        // keeps their capacity, which releases nothing: the whole source network
        // would stay allocated beside the copy being built until pieces.clear()
        // below. Move-assigning a fresh Mesh frees the storage instead.
        piece.mesh = Mesh{};
    }

    m_road_lod_stats.triangles_in = triangles;
    spdlog::info("QuadTree: assigned {} road pieces ({} triangles) to {} leaves, {} skipped",
                 assigned, triangles, touched.size(), skipped);
}

void QuadTree::assign_triangles_by_centroid(std::vector<road::RoadPiece>& pieces,
                                            std::vector<QuadTreeNode*>& touched) {
    size_t assigned = 0;
    size_t skipped = 0;
    size_t triangles = 0;
    size_t vertices = 0;
    size_t split_pieces = 0;

    // Scratch, reused across pieces so the whole pass allocates a handful of
    // times rather than once per piece.
    std::vector<uint32_t> tri_bucket;          // triangle -> index into buckets
    std::vector<MaterialKey> tri_key;          // triangle -> material it came in under
    std::unordered_map<QuadTreeNode*, uint32_t> bucket_of;
    std::vector<QuadTreeNode*> bucket_leaf;
    std::vector<std::vector<uint32_t>> buckets;  // triangles per leaf, ascending
    std::vector<uint32_t> remap;

    for (auto& piece : pieces) {
        const Mesh& src = piece.mesh;
        if (src.vertices.empty() || src.indices.size() < 3) {
            ++skipped;
            piece.mesh = Mesh{};
            continue;
        }

        const size_t tri_count = src.indices.size() / 3;
        tri_key.assign(tri_count, MaterialKey{});
        tri_bucket.assign(tri_count, kNoVertex);
        bucket_of.clear();
        bucket_leaf.clear();
        for (auto& b : buckets) b.clear();

        // Material per triangle. effective_submeshes() tiles the whole index
        // buffer, so every triangle gets a key even on a piece that carries no
        // explicit ranges.
        for (const SubMesh& range : src.effective_submeshes()) {
            const size_t end = std::min(static_cast<size_t>(range.index_offset) + range.index_count,
                                        src.indices.size());
            for (size_t i = range.index_offset; i < end; i += 3) {
                tri_key[i / 3] = MaterialKey{range.material, range.variant};
            }
        }

        // Bucket by the leaf holding each triangle's own centroid. This is the
        // split that keeps a piece inside the chunk rectangles it is simplified
        // against; see the crack argument on assign_road_pieces().
        for (size_t t = 0; t < tri_count; ++t) {
            const uint32_t i0 = src.indices[t * 3];
            const uint32_t i1 = src.indices[t * 3 + 1];
            const uint32_t i2 = src.indices[t * 3 + 2];
            if (i0 >= src.vertices.size() || i1 >= src.vertices.size() ||
                i2 >= src.vertices.size()) {
                continue;
            }

            const glm::dvec2 centroid =
                (world_to_local(src.vertices[i0].position) +
                 world_to_local(src.vertices[i1].position) +
                 world_to_local(src.vertices[i2].position)) / 3.0;

            QuadTreeNode* leaf = find_leaf(centroid);
            if (!leaf) continue;

            auto [it, inserted] = bucket_of.emplace(leaf, static_cast<uint32_t>(bucket_leaf.size()));
            if (inserted) {
                bucket_leaf.push_back(leaf);
                if (buckets.size() < bucket_leaf.size()) buckets.emplace_back();
            }
            tri_bucket[t] = it->second;
            buckets[it->second].push_back(static_cast<uint32_t>(t));
        }

        if (bucket_leaf.empty()) {
            ++skipped;
            piece.mesh = Mesh{};
            continue;
        }
        if (bucket_leaf.size() > 1) ++split_pieces;

        remap.assign(src.vertices.size(), kNoVertex);

        for (size_t b = 0; b < bucket_leaf.size(); ++b) {
            QuadTreeNode* leaf = bucket_leaf[b];
            if (leaf->road_meshes.empty()) {
                leaf->road_meshes.emplace_back();
                touched.push_back(leaf);
            }
            Mesh& dst = leaf->road_meshes.front();

            // A destination that already has indices but no ranges is carrying an
            // implicit whole-mesh range. Materialise it before opening a new one,
            // exactly as Mesh::append() does, so pre-existing geometry keeps its
            // identity.
            if (!dst.indices.empty() && dst.submeshes.empty()) {
                dst.submeshes.push_back({0u, static_cast<uint32_t>(dst.indices.size()),
                                         MaterialId::Default, uint16_t{0}});
            }

            // One remap generation per (piece, leaf) pair: a source vertex used by
            // triangles that went to two different leaves is copied into both, and
            // that duplication is the only cost of splitting at triangle
            // granularity.
            for (const uint32_t t : buckets[b]) {
                for (size_t k = 0; k < 3; ++k) remap[src.indices[t * 3 + k]] = kNoVertex;
            }

            for (const uint32_t t : buckets[b]) {
                const MaterialKey key = tri_key[t];
                const uint32_t start = static_cast<uint32_t>(dst.indices.size());
                if (!append_triangle(dst, src, t, remap)) continue;

                SubMesh* back = dst.submeshes.empty() ? nullptr : &dst.submeshes.back();
                if (back && back->material == key.material && back->variant == key.variant &&
                    back->index_offset + back->index_count == start) {
                    back->index_count += 3u;
                } else {
                    dst.submeshes.push_back({start, 3u, key.material, key.variant});
                }
                ++triangles;
            }
        }

        vertices += src.vertices.size();
        ++assigned;
        piece.mesh = Mesh{};
    }

    m_road_lod_stats.triangles_in = triangles;
    m_road_lod_stats.vertices_in = vertices;

    spdlog::info("QuadTree: assigned {} road pieces ({} triangles) to {} leaves by triangle "
                 "centroid, {} pieces spanning more than one leaf, {} skipped",
                 assigned, triangles, touched.size(), split_pieces, skipped);
}

double QuadTree::measure_seam_bands(const std::vector<QuadTreeNode*>& touched,
                                    std::vector<double>& out_bands) {
    out_bands.assign(touched.size(), 0.0);

    std::unordered_map<const QuadTreeNode*, size_t> index_of;
    index_of.reserve(touched.size() * 2);
    for (size_t i = 0; i < touched.size(); ++i) index_of[touched[i]] = i;

    size_t straddling = 0;

    for (size_t i = 0; i < touched.size(); ++i) {
        QuadTreeNode* leaf = touched[i];
        glm::dvec2 mn, mx;
        leaf_rect(*leaf, mn, mx);

        for (const Mesh& mesh : leaf->road_meshes) {
            for (const Vertex& v : mesh.vertices) {
                const glm::dvec2 local = world_to_local(v.position);
                const double out = rect_outside_distance(local, mn, mx);
                if (out <= 0.0) continue;  // interior: shares nothing with a neighbour

                // R_out for this chunk: how far its OWN geometry reaches out.
                out_bands[i] = std::max(out_bands[i], out);
                ++straddling;

                // R_in for whichever chunk it reached into: how far a NEIGHBOUR's
                // geometry reaches in. A vertex inside that chunk's rectangle may
                // be shared with this one, so that chunk has to pin it too.
                const QuadTreeNode* other = find_leaf(local);
                if (!other || other == leaf) continue;
                const auto it = index_of.find(other);
                if (it == index_of.end()) continue;  // no road geometry, so no chunk

                glm::dvec2 omn, omx;
                leaf_rect(*other, omn, omx);
                out_bands[it->second] =
                    std::max(out_bands[it->second], rect_perimeter_distance(local, omn, omx));
            }
        }
    }

    m_road_lod_stats.straddling_triangles = straddling / 3;

    double widest = 0.0;
    for (const double b : out_bands) widest = std::max(widest, b);
    return widest;
}

void QuadTree::build_chunk_lods(const std::vector<QuadTreeNode*>& touched) {
    if (touched.empty()) return;

    const auto t0 = std::chrono::steady_clock::now();

    std::vector<double> bands;
    m_road_lod_stats.max_seam_band = measure_seam_bands(touched, bands);

    // Written into pre-sized slots and never pushed, so the result does not
    // depend on which worker got which chunk. Chunks share nothing: each reads
    // one leaf's merged mesh and its own rectangle.
    std::vector<road::ChunkLod> built(touched.size());

    const auto run = [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
            QuadTreeNode* leaf = touched[i];
            if (leaf->road_meshes.empty() || !leaf->road_meshes.front().is_valid()) continue;

            road::ChunkLodConfig cfg = m_chunk_lod_config;
            // The configured band is a FLOOR. The measured one is what the
            // crack-free argument needs; anything narrower is a seam.
            cfg.border_band = std::max(cfg.border_band,
                                       static_cast<float>(bands[i] + kSeamBandSlack));

            glm::dvec2 mn, mx;
            leaf_rect(*leaf, mn, mx);

            const std::vector<const Mesh*> input{&leaf->road_meshes.front()};
            built[i] = road::build_chunk_lod(input, mn, mx, cfg);

            // Release the input HERE, not in the serial pass below. levels[0] is
            // this mesh welded and reordered, so holding both to the end of the
            // sweep keeps the whole road network on the host twice at peak
            // (measured 571 MB against 377 MB on the Lucan extract). Each worker
            // owns touched[i] exclusively and measure_seam_bands() finished
            // reading every leaf before the sweep started, so freeing it now is
            // safe. The levels.empty() guard mirrors the serial loop: with no
            // chain, road_meshes IS still the geometry and must survive.
            if (!built[i].levels.empty()) {
                leaf->road_meshes.clear();
                leaf->road_meshes.shrink_to_fit();
            }
        }
    };

    if (touched.size() < kParallelMinChunks) {
        run(0, touched.size());
    } else {
        enki::TaskScheduler scheduler;
        scheduler.Initialize();
        enki::TaskSet sweep(static_cast<uint32_t>(touched.size()),
                            [&](enki::TaskSetPartition range, uint32_t /*threadnum*/) {
                                run(static_cast<size_t>(range.start),
                                    static_cast<size_t>(range.end));
                            });
        sweep.m_MinRange = 1;
        scheduler.AddTaskSetToPipe(&sweep);
        scheduler.WaitforTask(&sweep);
    }

    for (size_t i = 0; i < touched.size(); ++i) {
        QuadTreeNode* leaf = touched[i];
        if (built[i].levels.empty()) continue;  // keep road_meshes; it is still the geometry

        leaf->road_lod = std::move(built[i]);
        leaf->road_lod_resident = -1;

        // road_meshes was already released by the worker that built this chain;
        // see the note there. Nothing left to free.

        ++m_road_lod_stats.chunks_with_lod;
        const size_t depth = leaf->road_lod.levels.size();
        if (m_road_lod_stats.triangles_per_level.size() < depth) {
            m_road_lod_stats.triangles_per_level.resize(depth, 0);
            m_road_lod_stats.vertices_per_level.resize(depth, 0);
            m_road_lod_stats.chunks_per_level.resize(depth, 0);
        }
        for (size_t l = 0; l < depth; ++l) {
            m_road_lod_stats.triangles_per_level[l] += leaf->road_lod.levels[l].indices.size() / 3;
            m_road_lod_stats.vertices_per_level[l] += leaf->road_lod.levels[l].vertices.size();
            ++m_road_lod_stats.chunks_per_level[l];
        }
    }

    m_road_lod_stats.build_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

    spdlog::info("QuadTree: chunk LOD over {} chunks in {:.1f} ms, widest seam band {:.2f} m",
                 m_road_lod_stats.chunks_with_lod, m_road_lod_stats.build_ms,
                 m_road_lod_stats.max_seam_band);
    for (size_t l = 0; l < m_road_lod_stats.triangles_per_level.size(); ++l) {
        spdlog::info("  level {}: {} triangles across {} chunks", l,
                     m_road_lod_stats.triangles_per_level[l],
                     m_road_lod_stats.chunks_per_level[l]);
    }
}

void QuadTree::assign_road_pieces(std::vector<road::RoadPiece>&& pieces) {
    if (!m_root) {
        spdlog::warn("QuadTree: assign_road_pieces() before init(); {} road pieces dropped",
                     pieces.size());
        pieces.clear();
        return;
    }

    m_road_lod_stats = RoadLodStats{};

    // This is the only writer of QuadTreeNode::road_meshes, so an empty
    // road_meshes means the leaf has not been touched yet on this pass. That is
    // what makes the first-touch test below O(1).
    std::vector<QuadTreeNode*> touched;

    if (m_chunk_lod_enabled) {
        assign_triangles_by_centroid(pieces, touched);
    } else {
        assign_pieces_by_anchor(pieces, touched);
    }

    // Pieces arrive interleaved by material, so each leaf mesh now holds one
    // range per material per piece. Collapse them to one range per material so
    // the leaf costs a handful of draw calls rather than thousands.
    for (QuadTreeNode* leaf : touched) {
        leaf->road_meshes.front().sort_submeshes_by_material();
    }

    m_road_lod_stats.chunks = touched.size();

    // A piece may overhang the leaf that owns it, and a leaf that received one
    // may sit outside the AABB assign_data() computed from features alone. Run
    // this BEFORE the LOD build, which consumes road_meshes.
    recompute_bounds(m_root.get());

    pieces.clear();

    if (m_chunk_lod_enabled) {
        build_chunk_lods(touched);
    }
}

int select_road_lod_level(const road::ChunkLod& lod, float distance, int current, float scale) {
    const size_t depth = lod.levels.size();
    if (depth <= 1) return 0;

    const size_t stops = std::min(depth, lod.switch_distances.size());
    const float bias = (scale > 0.0f) ? scale : 1.0f;

    int level = 0;
    for (size_t i = 1; i < stops; ++i) {
        const float stop = lod.switch_distances[i] * bias;
        // Coarsening costs an upload, so it waits until the distance is clear of
        // the stop; refining is what the viewer notices, so it happens as soon as
        // the distance is clear of it the other way. Between the two the resident
        // level stays put.
        const float threshold = (static_cast<int>(i) > current)
                              ? stop * (1.0f + kRoadLodHysteresis)
                              : stop * (1.0f - kRoadLodHysteresis);
        if (distance >= threshold) {
            level = static_cast<int>(i);
        } else {
            break;  // switch_distances is ascending: nothing further can pass
        }
    }
    return level;
}

// ============================================================================
// Feature Insertion
// ============================================================================

int QuadTree::child_index(const QuadTreeNode* node, const glm::dvec2& point) const {
    // NW=0, NE=1, SW=2, SE=3
    int idx = 0;
    if (point.x >= node->center.x) idx |= 1; // East
    if (point.y < node->center.y) idx |= 2;  // South
    return idx;
}

QuadTreeNode* QuadTree::find_leaf(const glm::dvec2& point) {
    QuadTreeNode* node = m_root.get();
    if (!node) return nullptr;

    // Same descent insert_road() uses, minus the insertion: walk children by
    // quadrant until a leaf. child_index() clamps nothing, so a point outside
    // the root still resolves to the nearest boundary leaf rather than falling
    // out of the tree.
    while (!node->is_leaf()) {
        QuadTreeNode* child = node->children[child_index(node, point)].get();
        if (!child) break;
        node = child;
    }
    return node;
}

void QuadTree::subdivide(QuadTreeNode* node) {
    double quarter = node->half_size / 2.0;

    // NW=0, NE=1, SW=2, SE=3
    glm::dvec2 offsets[4] = {
        {-quarter,  quarter}, // NW
        { quarter,  quarter}, // NE
        {-quarter, -quarter}, // SW
        { quarter, -quarter}, // SE
    };

    for (int i = 0; i < 4; i++) {
        node->children[i] = std::make_unique<QuadTreeNode>();
        node->children[i]->center = node->center + offsets[i];
        node->children[i]->half_size = quarter;
        node->children[i]->node_id = m_next_id++;
        node->children[i]->depth = node->depth + 1;
        compute_3d_bounds(node->children[i].get());
    }

    // Redistribute features from parent to children by centroid
    for (auto& road : node->roads) {
        glm::dvec2 centroid(0.0);
        for (const auto& pt : road.polyline) centroid += pt;
        centroid /= static_cast<double>(road.polyline.size());
        int idx = child_index(node, centroid);
        node->children[idx]->roads.push_back(std::move(road));
    }
    node->roads.clear();
    node->roads.shrink_to_fit();

    for (auto& building : node->buildings) {
        glm::dvec2 centroid(0.0);
        for (const auto& pt : building.footprint) centroid += pt;
        centroid /= static_cast<double>(building.footprint.size());
        int idx = child_index(node, centroid);
        node->children[idx]->buildings.push_back(std::move(building));
    }
    node->buildings.clear();
    node->buildings.shrink_to_fit();

    for (auto& area : node->areas) {
        glm::dvec2 centroid(0.0);
        for (const auto& pt : area.polygon) centroid += pt;
        centroid /= static_cast<double>(area.polygon.size());
        int idx = child_index(node, centroid);
        node->children[idx]->areas.push_back(std::move(area));
    }
    node->areas.clear();
    node->areas.shrink_to_fit();
}

void QuadTree::insert_road(QuadTreeNode* node, const Road& road) {
    if (!node) return;

    // Compute centroid
    glm::dvec2 centroid(0.0);
    for (const auto& pt : road.polyline) centroid += pt;
    centroid /= static_cast<double>(road.polyline.size());

    // If internal node, route to correct child
    if (!node->is_leaf()) {
        int idx = child_index(node, centroid);
        insert_road(node->children[idx].get(), road);
        return;
    }

    // Leaf node — add feature
    node->roads.push_back(road);

    // Check if we need to subdivide
    if (node->feature_count() > QuadTreeConfig::MAX_FEATURES_PER_LEAF &&
        node->half_size > QuadTreeConfig::MIN_NODE_SIZE &&
        node->depth < QuadTreeConfig::MAX_DEPTH) {
        subdivide(node);
    }
}

void QuadTree::insert_building(QuadTreeNode* node, const Building& building) {
    if (!node) return;

    glm::dvec2 centroid(0.0);
    for (const auto& pt : building.footprint) centroid += pt;
    centroid /= static_cast<double>(building.footprint.size());

    if (!node->is_leaf()) {
        int idx = child_index(node, centroid);
        insert_building(node->children[idx].get(), building);
        return;
    }

    node->buildings.push_back(building);

    if (node->feature_count() > QuadTreeConfig::MAX_FEATURES_PER_LEAF &&
        node->half_size > QuadTreeConfig::MIN_NODE_SIZE &&
        node->depth < QuadTreeConfig::MAX_DEPTH) {
        subdivide(node);
    }
}

void QuadTree::insert_area(QuadTreeNode* node, const Area& area) {
    if (!node) return;

    glm::dvec2 centroid(0.0);
    for (const auto& pt : area.polygon) centroid += pt;
    centroid /= static_cast<double>(area.polygon.size());

    if (!node->is_leaf()) {
        int idx = child_index(node, centroid);
        insert_area(node->children[idx].get(), area);
        return;
    }

    node->areas.push_back(area);

    if (node->feature_count() > QuadTreeConfig::MAX_FEATURES_PER_LEAF &&
        node->half_size > QuadTreeConfig::MIN_NODE_SIZE &&
        node->depth < QuadTreeConfig::MAX_DEPTH) {
        subdivide(node);
    }
}

void QuadTree::compute_3d_bounds(QuadTreeNode* node) {
    double hs = node->half_size;
    double min_x = node->center.x - hs;
    double min_y = node->center.y - hs;
    double max_x = node->center.x + hs;
    double max_y = node->center.y + hs;

    // Find max building height in this node for Y extent
    float max_height = 50.0f; // default reasonable height
    if (node->is_leaf()) {
        for (const auto& b : node->buildings) {
            max_height = std::max(max_height, b.height);
        }
    }

    // Convert from local 2D (x = east, y = north) to 3D rendering coords
    // In rendering: X = east, Y = up, Z = -north
    node->bounds_min = glm::vec3(
        static_cast<float>(min_x),
        0.0f,
        static_cast<float>(-max_y)
    );
    node->bounds_max = glm::vec3(
        static_cast<float>(max_x),
        max_height,
        static_cast<float>(-min_y)
    );
}

void QuadTree::recompute_bounds(QuadTreeNode* node) {
    if (!node) return;

    if (node->is_leaf()) {
        compute_3d_bounds(node);

        // Road pieces are anchored inside this leaf but are never split, so a
        // road may hang over the boundary. Grow the AABB to cover it, otherwise
        // frustum culling drops the leaf while part of its geometry is still on
        // screen.
        //
        // This is what covers a junction straddling a leaf boundary as well. Its
        // fill and curb ring reach out to the arm mouths in every direction from
        // the node it is anchored at, so a node near a boundary always overhangs;
        // the bounds are grown from the accumulated vertex bounds of the leaf's
        // road mesh, which append_preserving_materials() maintains, so the reach
        // is measured rather than assumed.
        for (const auto& mesh : node->road_meshes) {
            if (!mesh.bounds.is_valid()) continue;
            node->bounds_min = glm::min(node->bounds_min, mesh.bounds.min);
            node->bounds_max = glm::max(node->bounds_max, mesh.bounds.max);
        }

        // Once the chunk LOD is built, road_meshes is empty and level 0 holds the
        // same triangles. A later recompute -- a road rebuild that reuses the tree
        // -- has to read the bounds from there or the leaf loses its overhang.
        if (!node->road_lod.levels.empty() && node->road_lod.levels.front().bounds.is_valid()) {
            node->bounds_min = glm::min(node->bounds_min, node->road_lod.levels.front().bounds.min);
            node->bounds_max = glm::max(node->bounds_max, node->road_lod.levels.front().bounds.max);
        }
        return;
    }

    node->bounds_min = glm::vec3(std::numeric_limits<float>::max());
    node->bounds_max = glm::vec3(std::numeric_limits<float>::lowest());
    for (auto& child : node->children) {
        if (!child) continue;
        recompute_bounds(child.get());
        node->bounds_min = glm::min(node->bounds_min, child->bounds_min);
        node->bounds_max = glm::max(node->bounds_max, child->bounds_max);
    }
}

// ============================================================================
// Traversal
// ============================================================================

bool QuadTree::frustum_intersects_aabb(const std::array<glm::vec4, 6>& planes,
                                        const glm::vec3& mn, const glm::vec3& mx) const {
    for (const auto& plane : planes) {
        glm::vec3 p = mn;
        if (plane.x >= 0) p.x = mx.x;
        if (plane.y >= 0) p.y = mx.y;
        if (plane.z >= 0) p.z = mx.z;

        if (glm::dot(glm::vec3(plane), p) + plane.w < 0) {
            return false;
        }
    }
    return true;
}

void QuadTree::traverse_visible(
    const std::array<glm::vec4, 6>& frustum_planes,
    const glm::vec3& cam_pos,
    float view_radius,
    float screen_height,
    float fov_y,
    float contribution_threshold,
    bool use_frustum_culling,
    bool use_distance_culling,
    bool use_contribution_culling,
    const QuadTreeVisitor& visitor)
{
    if (!m_root) return;

    float radius_sq = view_radius * view_radius;

    std::vector<std::pair<QuadTreeNode*, float>> visible;
    visible.reserve(128);

    traverse_recursive(m_root.get(), frustum_planes, cam_pos, radius_sq,
                       screen_height, fov_y, contribution_threshold,
                       use_frustum_culling, use_distance_culling, use_contribution_culling,
                       visible);

    // Sort front-to-back by distance for early-Z benefit
    std::sort(visible.begin(), visible.end(),
              [](const auto& a, const auto& b) { return a.second < b.second; });

    for (auto& [node, dist_sq] : visible) {
        visitor(node, dist_sq);
    }
}

void QuadTree::traverse_recursive(
    QuadTreeNode* node,
    const std::array<glm::vec4, 6>& frustum_planes,
    const glm::vec3& cam_pos,
    float radius_sq,
    float screen_height,
    float fov_y,
    float contribution_threshold,
    bool use_frustum,
    bool use_distance,
    bool use_contribution,
    std::vector<std::pair<QuadTreeNode*, float>>& visible)
{
    if (!node) return;

    // 1. Frustum cull
    if (use_frustum && !frustum_intersects_aabb(frustum_planes, node->bounds_min, node->bounds_max)) {
        return;
    }

    // Centre distance, used for front-to-back sorting and for the contribution
    // estimate below.
    glm::vec3 node_center_3d = (node->bounds_min + node->bounds_max) * 0.5f;
    glm::vec3 diff = node_center_3d - cam_pos;
    float dist_sq = diff.x * diff.x + diff.z * diff.z; // XZ plane

    // 2. Distance cull, measured to the NEAREST POINT of the node's XZ footprint.
    //
    // Testing the centre instead prunes the entire subtree of any internal node
    // whose midpoint sits beyond the radius, even when the node's extent reaches
    // the camera -- and internal nodes are large. On a 123km import that rejected
    // every node at depth 1, so nothing was ever built and the viewport stayed
    // empty even though thousands of leaves were within the view radius.
    const float near_dx = cam_pos.x - std::clamp(cam_pos.x, node->bounds_min.x, node->bounds_max.x);
    const float near_dz = cam_pos.z - std::clamp(cam_pos.z, node->bounds_min.z, node->bounds_max.z);
    const float nearest_sq = near_dx * near_dx + near_dz * near_dz;

    if (use_distance && nearest_sq > radius_sq) {
        return;
    }

    // 3. Contribution cull — skip if projected size on screen is too small
    if (use_contribution && contribution_threshold > 0.0f) {
        // Approximate: project the node's world-space size to screen pixels
        float node_world_size = static_cast<float>(node->half_size * 2.0);
        float dist = std::sqrt(dist_sq);
        if (dist > 1.0f) {
            float projected_pixels = (node_world_size / dist) * (screen_height / (2.0f * std::tan(glm::radians(fov_y) * 0.5f)));
            if (projected_pixels < contribution_threshold) {
                return;
            }
        }
    }

    // 4. If leaf: collect
    if (node->is_leaf()) {
        // road_meshes is checked separately: a road piece is routed by its
        // anchor, so a leaf can own road geometry without owning any feature.
        // has_road_lod() is the third case: build_chunk_lods() moves a leaf's
        // roads into road_lod and CLEARS road_meshes, so a leaf whose only
        // content is road geometry would otherwise stop being traversed the
        // moment chunk LOD is on, and its roads would never be uploaded.
        if (node->feature_count() > 0 || !node->road_meshes.empty() || node->has_road_lod()) {
            visible.emplace_back(node, dist_sq);
        }
        return;
    }

    // 5. If internal: recurse children
    for (auto& child : node->children) {
        if (child) {
            traverse_recursive(child.get(), frustum_planes, cam_pos, radius_sq,
                               screen_height, fov_y, contribution_threshold,
                               use_frustum, use_distance, use_contribution, visible);
        }
    }
}

// ============================================================================
// Mesh Building
// ============================================================================

QuadTree::BuiltMeshes QuadTree::build_node_meshes_internal(const QuadTreeNode& node) {
    BuiltMeshes result;

    // No roads here. Road geometry is solved once against the whole road graph
    // and delivered by assign_road_pieces(); building it per leaf is what made
    // junctions stop at leaf boundaries. Buildings and areas keep the per-leaf
    // path because they carry no cross-leaf topology.

    // Buildings
    {
        std::vector<Mesh> individual;
        individual.reserve(node.buildings.size());
        for (const auto& building : node.buildings) {
            Mesh mesh = MeshBuilder::build_building_mesh(building);
            if (mesh.is_valid()) individual.push_back(std::move(mesh));
        }
        Mesh merged = MeshBuilder::merge_meshes(individual);
        if (merged.is_valid()) {
            result.building_meshes.push_back(std::move(merged));
        }
    }

    // Areas
    {
        std::vector<Mesh> individual;
        individual.reserve(node.areas.size());
        for (const auto& area : node.areas) {
            Mesh mesh = MeshBuilder::build_area_mesh(area);
            if (mesh.is_valid()) individual.push_back(std::move(mesh));
        }
        Mesh merged = MeshBuilder::merge_meshes(individual);
        if (merged.is_valid()) {
            result.area_meshes.push_back(std::move(merged));
        }
    }

    return result;
}

bool QuadTree::queue_node_build_async(QuadTreeNode* node) {
    if (!node || node->meshes_built || node->meshes_pending) return false;

    node->meshes_pending = true;

    // Copy node data for thread safety. Roads are deliberately not copied: the
    // worker no longer builds road geometry, so copying every Road per node was
    // pure cost.
    QuadTreeNode node_copy;
    node_copy.buildings = node->buildings;
    node_copy.areas = node->areas;

    std::lock_guard<std::mutex> lock(m_pending_mutex);
    m_pending_builds.push_back({
        node,
        std::async(std::launch::async, [this, nc = std::move(node_copy)]() {
            return build_node_meshes_internal(nc);
        })
    });

    return true;
}

size_t QuadTree::poll_async_builds() {
    std::lock_guard<std::mutex> lock(m_pending_mutex);
    size_t completed = 0;

    for (auto it = m_pending_builds.begin(); it != m_pending_builds.end(); ) {
        if (it->future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            BuiltMeshes meshes = it->future.get();
            QuadTreeNode* node = it->node;
            if (node) {
                // road_meshes is untouched on purpose: it was filled by
                // assign_road_pieces() before any build was queued, and this
                // result carries no road geometry to replace it with.
                node->building_meshes = std::move(meshes.building_meshes);
                node->area_meshes = std::move(meshes.area_meshes);
                node->meshes_built = true;
                node->meshes_pending = false;
            }
            it = m_pending_builds.erase(it);
            completed++;
        } else {
            ++it;
        }
    }

    return completed;
}

// ============================================================================
// Accessors
// ============================================================================

void QuadTree::collect_leaves(QuadTreeNode* node, std::vector<QuadTreeNode*>& out) {
    if (!node) return;
    if (node->is_leaf()) {
        out.push_back(node);
        return;
    }
    for (auto& child : node->children) {
        collect_leaves(child.get(), out);
    }
}

std::vector<QuadTreeNode*> QuadTree::get_all_leaves() {
    std::vector<QuadTreeNode*> leaves;
    collect_leaves(m_root.get(), leaves);
    return leaves;
}

void QuadTree::get_bounds(glm::vec3& out_min, glm::vec3& out_max) const {
    if (m_root) {
        out_min = m_root->bounds_min;
        out_max = m_root->bounds_max;
    } else {
        out_min = glm::vec3(0.0f);
        out_max = glm::vec3(0.0f);
    }
}

void QuadTree::count_leaves(const QuadTreeNode* node, size_t& count) const {
    if (!node) return;
    if (node->is_leaf()) { count++; return; }
    for (const auto& child : node->children) {
        count_leaves(child.get(), count);
    }
}

size_t QuadTree::leaf_count() const {
    size_t count = 0;
    count_leaves(m_root.get(), count);
    return count;
}

void QuadTree::count_features(const QuadTreeNode* node, size_t& roads, size_t& buildings, size_t& areas) const {
    if (!node) return;
    if (node->is_leaf()) {
        roads += node->roads.size();
        buildings += node->buildings.size();
        areas += node->areas.size();
        return;
    }
    for (const auto& child : node->children) {
        count_features(child.get(), roads, buildings, areas);
    }
}

size_t QuadTree::total_roads() const {
    size_t r = 0, b = 0, a = 0;
    count_features(m_root.get(), r, b, a);
    return r;
}

size_t QuadTree::total_buildings() const {
    size_t r = 0, b = 0, a = 0;
    count_features(m_root.get(), r, b, a);
    return b;
}

size_t QuadTree::total_areas() const {
    size_t r = 0, b = 0, a = 0;
    count_features(m_root.get(), r, b, a);
    return a;
}

void QuadTree::find_max_depth(const QuadTreeNode* node, uint8_t& depth) const {
    if (!node) return;
    depth = std::max(depth, node->depth);
    for (const auto& child : node->children) {
        find_max_depth(child.get(), depth);
    }
}

uint8_t QuadTree::max_depth() const {
    uint8_t d = 0;
    find_max_depth(m_root.get(), d);
    return d;
}

} // namespace stratum::osm
