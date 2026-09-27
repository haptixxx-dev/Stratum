// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

#include "editor/editor.hpp"
#include "renderer/gpu_renderer.hpp"
#include <spdlog/spdlog.h>
#include <SDL3/SDL.h>
#include <algorithm>
#include <limits>

namespace stratum {

// ============================================================================
// Resident GPU geometry: ownership, distance, and eviction
// ============================================================================

glm::vec3 Editor::node_anchor(const osm::QuadTreeNode& node) {
    if (node.has_valid_bounds()) {
        return (node.bounds_min + node.bounds_max) * 0.5f;
    }
    // No geometry ever grew the AABB, but the cell still locates the leaf. Local
    // 2D (x, y) maps to world (x, height, -y); the height is unknown, so 0.
    return glm::vec3(static_cast<float>(node.center.x), 0.0f,
                     static_cast<float>(-node.center.y));
}

uint32_t Editor::upload_tracked_mesh(GPURenderer& renderer, const Mesh& mesh,
                                     const MeshOwner& owner) {
    const uint32_t id = renderer.upload_mesh(mesh);
    if (id == 0) {
        // Not a handle. Registering it would make the owner of mesh 0 whichever
        // upload failed most recently.
        return 0;
    }
    m_mesh_owners[id] = owner;
    return id;
}

void Editor::release_tracked_mesh(GPURenderer& renderer, uint32_t& mesh_id) {
    if (mesh_id == 0) return;
    m_mesh_owners.erase(mesh_id);
    renderer.release_mesh(mesh_id);
    mesh_id = 0;
}

float Editor::mesh_distance_to_camera(uint32_t mesh_id) const {
    const auto it = m_mesh_owners.find(mesh_id);
    if (it == m_mesh_owners.end()) {
        // Nothing is holding this handle, so nothing will miss it. Reporting it as
        // infinitely far away puts it at the front of the eviction order, which is
        // the right answer for geometry nobody is tracking any more.
        return std::numeric_limits<float>::max();
    }
    if (it->second.kind == MeshOwner::Kind::Pinned) {
        return -1.0f;
    }
    return glm::length(it->second.anchor - m_camera.get_position());
}

void Editor::on_mesh_evicted(uint32_t mesh_id) {
    const auto it = m_mesh_owners.find(mesh_id);
    if (it == m_mesh_owners.end()) return;

    const MeshOwner owner = it->second;
    m_mesh_owners.erase(it);

    // No call back into the renderer from here: it is mid-eviction and walking
    // its own mesh map. Clearing the handle is all this has to do.
    switch (owner.kind) {
        case MeshOwner::Kind::QuadTreeLeaf: {
            if (!owner.node) break;
            std::erase(owner.node->area_gpu_ids, mesh_id);
            if (std::erase(owner.node->road_gpu_ids, mesh_id) > 0) {
                // The resident LOD level went with it. Saying so is what makes
                // sync_node_road_lod() upload again instead of trusting a level
                // that is no longer on the device.
                owner.node->road_lod_resident = -1;
            }
            std::erase(owner.node->building_gpu_ids, mesh_id);
            // The leaf is no longer whole, so it is no longer uploaded. It streams
            // back in the next time it is visible, and upload_node_to_gpu()
            // releases whichever of its handles survived before re-uploading.
            owner.node->gpu_uploaded = false;
            break;
        }
        case MeshOwner::Kind::TerrainChunk: {
            auto* chunk = m_terrain_tile_manager.get_chunk(owner.coord);
            if (!chunk) break;
            if (chunk->terrain_gpu_id == mesh_id) chunk->terrain_gpu_id = 0;
            if (chunk->water_gpu_id == mesh_id) chunk->water_gpu_id = 0;
            // Same contract as a leaf: the re-upload path in render_3d() releases
            // the surviving handle before replacing both.
            chunk->gpu_uploaded = false;
            break;
        }
        case MeshOwner::Kind::Pinned:
            // Unreachable: a pinned mesh reports a negative distance and is never
            // a candidate. If it happens, the handle has already been forgotten
            // above, which is the most that can be done from here.
            spdlog::warn("A pinned mesh ({}) was evicted", mesh_id);
            break;
    }
}

void Editor::upload_node_to_gpu(osm::QuadTreeNode& node, GPURenderer& renderer) {
    if (node.gpu_uploaded) return;

    // A leaf can reach here still holding handles: eviction takes one of its
    // meshes and clears gpu_uploaded, leaving the others live. Clearing the id
    // vectors without releasing them -- which is what this used to do -- would
    // strand that geometry on the GPU for the rest of the session.
    release_node_from_gpu(node, renderer);

    MeshOwner owner;
    owner.kind = MeshOwner::Kind::QuadTreeLeaf;
    owner.node = &node;
    owner.anchor = node_anchor(node);

    size_t failed = 0;
    size_t uploaded = 0;
    const auto upload_all = [&](const std::vector<Mesh>& meshes, std::vector<uint32_t>& ids) {
        ids.reserve(meshes.size());
        for (const Mesh& mesh : meshes) {
            if (!mesh.is_valid()) {
                continue;  // nothing to draw, and not a failure either
            }
            const uint32_t id = upload_tracked_mesh(renderer, mesh, owner);
            if (id == 0) {
                ++failed;
                continue;
            }
            ids.push_back(id);
            ++uploaded;
        }
    };

    upload_all(node.area_meshes, node.area_gpu_ids);
    // Empty when the leaf carries a chunk LOD chain: the chain replaced this mesh
    // and sync_node_road_lod() uploads exactly one level of it, per frame, by
    // distance. Roads are therefore NOT part of the completeness test below --
    // they are not uploaded here and their absence is not a failure.
    upload_all(node.road_meshes, node.road_gpu_ids);
    upload_all(node.building_meshes, node.building_gpu_ids);

    // Only a leaf that uploaded IN FULL is uploaded.
    //
    // This used to push the 0 that upload_mesh() returns on FAILURE straight into
    // the id vector and set gpu_uploaded = true regardless, so a leaf that lost an
    // upload -- to a budget refusal, a pool refusal, or a device OOM -- was never
    // retried. draw_mesh() then discarded the 0 silently every frame and the leaf
    // rendered nothing for the rest of the session, with no error anywhere.
    //
    // The second half of the test covers eviction landing DURING this upload: an
    // upload under pressure evicts to make room, and the victim it picks can be a
    // mesh this very leaf uploaded a moment ago, which on_mesh_evicted() then
    // erases from the vectors below. Counting what is still held against what was
    // uploaded catches that without any extra state, and the leaf is retried
    // whole rather than latching as complete while missing a mesh.
    const size_t held = node.area_gpu_ids.size() + node.road_gpu_ids.size()
                      + node.building_gpu_ids.size();
    node.gpu_uploaded = (failed == 0) && (held == uploaded);

    if (failed > 0) {
        // Retried on every frame the leaf stays visible, so the warning is rate
        // limited rather than the retry: a failure that persists must not turn
        // into a 60 Hz log write.
        const uint64_t now = SDL_GetTicks();
        if (now >= m_next_upload_warn_ms) {
            m_next_upload_warn_ms = now + 2000;
            spdlog::warn("{} mesh(es) of quadtree leaf {} failed to upload; retrying while it "
                         "stays visible ({} renderer upload failures so far, {} MB resident)",
                         failed, node.node_id, renderer.upload_failures(),
                         renderer.resident_bytes() / (1024 * 1024));
        }
    }
}

void Editor::release_node_from_gpu(osm::QuadTreeNode& node, GPURenderer& renderer) {
    // Deliberately NOT guarded on gpu_uploaded. A leaf that lost one mesh to
    // eviction has the flag cleared while still holding the others, and a guard
    // here would leave exactly those behind -- which is the leak this function is
    // called to prevent, on every path that destroys the tree.
    const auto release_all = [&](std::vector<uint32_t>& ids) {
        for (uint32_t& id : ids) {
            release_tracked_mesh(renderer, id);
        }
        ids.clear();
    };

    release_all(node.area_gpu_ids);
    release_all(node.road_gpu_ids);
    release_all(node.building_gpu_ids);

    // The chain is still on the CPU, but nothing of it is on the device any
    // more. Leaving the level set would make sync_node_road_lod() believe the
    // right geometry was already resident and skip the re-upload.
    node.road_lod_resident = -1;
    node.gpu_uploaded = false;
}

void Editor::sync_node_road_lod(osm::QuadTreeNode& node, GPURenderer& renderer,
                                float distance) {
    if (!node.has_road_lod()) return;

    const int levels = static_cast<int>(node.road_lod.levels.size());

    // A forced level is clamped per chunk. Chains are not all the same length --
    // a chunk of seven pieces gives up after one level where a dense one gets
    // four -- so an override of 3 has to mean "the coarsest you have" rather than
    // "draw nothing".
    const int desired = (m_model.m_road_lod_override >= 0)
                      ? std::min(m_model.m_road_lod_override, levels - 1)
                      : osm::select_road_lod_level(node.road_lod, distance,
                                                   node.road_lod_resident,
                                                   m_model.m_road_lod_distance_scale);

    if (desired == node.road_lod_resident && !node.road_gpu_ids.empty()) {
        return;
    }

    ++m_road_lod_frame_build.swaps;

    // Release first, upload second. The other order would hold two levels of the
    // same chunk resident at once, and under a tight budget that is what makes an
    // upload evict some other leaf to make room for geometry about to be freed.
    for (uint32_t& id : node.road_gpu_ids) {
        release_tracked_mesh(renderer, id);
    }
    node.road_gpu_ids.clear();
    node.road_lod_resident = -1;

    const Mesh& mesh = node.road_lod.levels[static_cast<size_t>(desired)];
    if (!mesh.is_valid()) {
        // A level that simplified down to nothing is not a failure and must not
        // latch: leaving the level unset means the next frame tries again, which
        // is wrong. Record it as resident with no handle instead.
        node.road_lod_resident = desired;
        return;
    }

    MeshOwner owner;
    owner.kind = MeshOwner::Kind::QuadTreeLeaf;
    owner.node = &node;
    owner.anchor = node_anchor(node);

    const uint32_t id = upload_tracked_mesh(renderer, mesh, owner);
    if (id == 0) {
        return;  // retried on the next frame the leaf stays visible
    }

    // The upload may have evicted to make room, and the victim it picked can be
    // the mesh it just uploaded. m_mesh_owners is the record of what survived, so
    // pushing a handle that is no longer in it would leave the leaf drawing a
    // freed buffer.
    if (m_mesh_owners.find(id) == m_mesh_owners.end()) {
        return;
    }

    node.road_gpu_ids.push_back(id);
    node.road_lod_resident = desired;
}

void Editor::record_road_lod_residency(const osm::QuadTreeNode& node) {
    if (!node.has_road_lod()) {
        // A leaf with no chain but with road geometry is the chunk-LOD-off path,
        // not an empty leaf, and the panel has to be able to tell the two apart.
        if (!node.road_meshes.empty()) {
            ++m_road_lod_frame_build.leaves_no_chain;
        }
        return;
    }

    ++m_road_lod_frame_build.leaves_with_chain;

    const int level = node.road_lod_resident;
    if (level < 0 || level >= static_cast<int>(node.road_lod.levels.size())) {
        return;  // nothing resident: the upload was refused or the leaf was evicted
    }

    auto& per_level = m_road_lod_frame_build.leaves_per_level;
    const size_t idx = static_cast<size_t>(level);
    if (per_level.size() <= idx) per_level.resize(idx + 1, 0);
    ++per_level[idx];

    const Mesh& mesh = node.road_lod.levels[idx];
    m_road_lod_frame_build.resident_triangles += mesh.indices.size() / 3;
    m_road_lod_frame_build.resident_vertices += mesh.vertices.size();
}

} // namespace stratum
