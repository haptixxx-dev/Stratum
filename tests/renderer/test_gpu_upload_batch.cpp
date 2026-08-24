/**
 * @file test_gpu_upload_batch.cpp
 * @brief The staging arena: what one frame uploads, from where, and what it reclaims
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * upload_mesh() no longer submits a command buffer of its own. It reserves the
 * pooled ranges, copies the bytes into a CPU-side arena, and queues a
 * PendingUpload; flush_pending_uploads() then moves a budgeted prefix of that
 * queue through ONE transfer buffer, in ONE copy pass, at the top of the frame.
 * That change fixed a driver-level crash and introduced two pieces of arithmetic
 * that have to be right, both of which this suite covers directly.
 *
 * ### Where each entry is READ from
 *
 * The transfer buffer is packed by the plan, not inherited from the arena. It has
 * to be: release_mesh() drops the queue entry of a mesh released before its copy
 * ever ran, and leaves its bytes behind in the arena as a hole. Sourcing an entry
 * at `staging_offset - front().staging_offset` then reads past the end of the
 * transfer buffer by exactly the size of that hole -- an out-of-range
 * vkCmdCopyBuffer against a freshly sized buffer, and silently stale geometry on
 * a mesh already marked ready when a larger buffer was carried over from an
 * earlier frame. The hole is not exotic: the editor releases still-staged meshes
 * on a failed partial upload, on a terrain chunk rebuilt by a road carve, and on
 * a sibling's eviction, all while later leaves are queued behind them.
 *
 * ### What the arena RECLAIMS
 *
 * Reclaiming the drained prefix every frame is a memmove of the whole remaining
 * backlog, once per frame, for as long as the backlog lasts. During an import that
 * backlog is hundreds of megabytes and the queue drains 24 MB a frame. So
 * compaction is amortised: it happens only when the dead prefix is at least half
 * the arena, which bounds the arena at twice the live bytes.
 *
 * Both functions are pure and need no GPU, so this suite always runs.
 *
 * Run this suite with:
 * @code
 *     ./stratum_gpu_tests GPUUploadBatch
 * @endcode
 */

#include "framework.hpp"

#include "renderer/gpu_renderer.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace {

using stratum::PendingUpload;
using stratum::UploadBatch;
using stratum::plan_upload_batch;
using stratum::staging_compaction_offset;

/// 100 KB of vertices and nothing else, the size the reported case used
constexpr uint32_t kMeshBytes = 100u * 1024u;

/**
 * @brief A queue staged end to end, as upload_mesh() leaves it
 *
 * @param count Number of entries
 * @param size  Bytes per entry
 */
std::vector<PendingUpload> packed_queue(size_t count, uint32_t size = kMeshBytes) {
    std::vector<PendingUpload> queue;
    size_t offset = 0;
    for (size_t i = 0; i < count; ++i) {
        queue.push_back(PendingUpload{ static_cast<uint32_t>(i + 1), offset, size, 0u });
        offset += size;
    }
    return queue;
}

/// Every admitted entry's read range lies inside the transfer buffer the plan sized
bool batch_fits(const std::vector<PendingUpload>& queue, const UploadBatch& batch,
                std::string& reason) {
    for (size_t i = 0; i < batch.offsets.size(); ++i) {
        const size_t end = batch.offsets[i] + queue[i].total_bytes();
        if (end > batch.bytes) {
            reason = "entry " + std::to_string(i) + " reads [" +
                     std::to_string(batch.offsets[i]) + ", " + std::to_string(end) +
                     ") from a " + std::to_string(batch.bytes) + " byte transfer buffer";
            return false;
        }
    }
    return true;
}

} // namespace

// ============================================================================
// The batch
// ============================================================================

/**
 * A queue with no holes packs exactly as it is staged, which is the case the
 * original contiguous-window copy got right and the case everything else is
 * measured against.
 */
TEST(GPUUploadBatch, a_packed_queue_keeps_its_layout) {
    const std::vector<PendingUpload> queue = packed_queue(3);
    const UploadBatch batch = plan_upload_batch(queue, 24u * 1024u * 1024u);

    CHECK_EQ(batch.offsets.size(), size_t{3});
    CHECK_EQ(batch.bytes, size_t{3} * kMeshBytes);
    CHECK_EQ(batch.offsets[0], size_t{0});
    CHECK_EQ(batch.offsets[1], size_t{kMeshBytes});
    CHECK_EQ(batch.offsets[2], size_t{2} * kMeshBytes);
}

/**
 * THE HOLE TEST.
 *
 * Three meshes are staged; the middle one is released before it ever flushed, so
 * release_mesh() drops its queue entry and leaves its 100 KB in the arena. The
 * surviving entries still carry their original arena offsets, which no longer run
 * end to end.
 *
 * The transfer buffer is sized at the SUM of the admitted entries. Sourcing the
 * last one at `staging_offset - front().staging_offset` puts its read at 204800
 * in a 204800-byte buffer: entirely out of range. Every admitted entry has to be
 * read from inside the buffer the plan sized, whatever the arena looks like.
 */
TEST(GPUUploadBatch, a_hole_in_the_arena_stays_inside_the_transfer_buffer) {
    std::vector<PendingUpload> queue = packed_queue(3);
    queue.erase(queue.begin() + 1);   // exactly what release_mesh() does

    const UploadBatch batch = plan_upload_batch(queue, 24u * 1024u * 1024u);
    CHECK_EQ(batch.offsets.size(), size_t{2});
    CHECK_EQ(batch.bytes, size_t{2} * kMeshBytes);

    std::string reason;
    if (!batch_fits(queue, batch, reason)) {
        stratum::test::report_failure(__FILE__, __LINE__,
                                      "every admitted entry reads from inside the transfer buffer",
                                      reason);
    }

    // And it is read from the packed position, not the arena position: the arena
    // offset of the survivor is 204800 and its transfer offset must not be.
    CHECK_EQ(queue[1].staging_offset, size_t{2} * kMeshBytes);
    CHECK_EQ(batch.offsets[1], size_t{kMeshBytes});
}

/**
 * Several holes, in the shape a frame of the import actually produces: leaves and
 * terrain chunks interleaved, with one leaf's entries released on a retry. The
 * last admitted entry is the one that always overruns, by exactly the total hole
 * size, so a queue with holes in several places is the strongest form of the case.
 */
TEST(GPUUploadBatch, several_holes_still_pack) {
    std::vector<PendingUpload> queue;
    size_t arena = 0;
    const uint32_t sizes[6] = { 850u * 1024u, 512u * 1024u, 300u * 1024u,
                                850u * 1024u, 128u * 1024u, 900u * 1024u };
    for (size_t i = 0; i < 6; ++i) {
        queue.push_back(PendingUpload{ static_cast<uint32_t>(i + 1), arena, sizes[i], 0u });
        arena += sizes[i];
    }
    queue.erase(queue.begin() + 4);
    queue.erase(queue.begin() + 1);

    const UploadBatch batch = plan_upload_batch(queue, 24u * 1024u * 1024u);
    CHECK_EQ(batch.offsets.size(), queue.size());

    size_t expected = 0;
    for (size_t i = 0; i < queue.size(); ++i) {
        CHECK_EQ(batch.offsets[i], expected);
        expected += queue[i].total_bytes();
    }
    CHECK_EQ(batch.bytes, expected);

    std::string reason;
    if (!batch_fits(queue, batch, reason)) {
        stratum::test::report_failure(__FILE__, __LINE__,
                                      "every admitted entry reads from inside the transfer buffer",
                                      reason);
    }
}

/**
 * The budget stops the batch, and never at zero entries: a mesh bigger than one
 * frame's budget has to go through anyway or it never uploads at all.
 */
TEST(GPUUploadBatch, the_budget_admits_a_prefix_and_never_nothing) {
    const std::vector<PendingUpload> queue = packed_queue(10);

    const UploadBatch three = plan_upload_batch(queue, 3u * kMeshBytes);
    CHECK_EQ(three.offsets.size(), size_t{3});
    CHECK_EQ(three.bytes, size_t{3} * kMeshBytes);

    // Smaller than a single entry: one is still admitted.
    const UploadBatch one = plan_upload_batch(queue, 1u);
    CHECK_EQ(one.offsets.size(), size_t{1});
    CHECK_EQ(one.bytes, size_t{kMeshBytes});

    CHECK_EQ(plan_upload_batch({}, 24u * 1024u * 1024u).offsets.size(), size_t{0});
}

// ============================================================================
// The arena
// ============================================================================

/**
 * A small dead prefix is left alone.
 *
 * Compacting it would memmove the whole remaining backlog for the sake of a few
 * per cent, once per frame, for the tens of frames a large import takes to drain.
 */
TEST(GPUUploadBatch, a_small_dead_prefix_is_not_compacted) {
    std::vector<PendingUpload> queue = packed_queue(100);
    queue.erase(queue.begin(), queue.begin() + 10);   // 10 of 100 drained

    CHECK_EQ(staging_compaction_offset(queue, size_t{100} * kMeshBytes), size_t{0});
}

/**
 * Once the dead prefix is half the arena it is reclaimed, which bounds the arena
 * at twice the live bytes however long the backlog lasts.
 */
TEST(GPUUploadBatch, a_half_dead_arena_is_compacted) {
    std::vector<PendingUpload> queue = packed_queue(100);
    queue.erase(queue.begin(), queue.begin() + 50);

    const size_t arena = size_t{100} * kMeshBytes;
    CHECK_EQ(staging_compaction_offset(queue, arena), size_t{50} * kMeshBytes);
}
