/**
 * @file gpu_buffer_pool.hpp
 * @brief Suballocating device buffer pool: many meshes, few device allocations
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### The problem this exists to solve
 *
 * Today every uploaded mesh calls SDL_CreateGPUBuffer twice, once for vertices and
 * once for indices. That is fine for a demo scene and fatal for a city. A 63 MB
 * Lucan extract is thousands of road pieces plus buildings and terrain tiles, so
 * it is tens of thousands of device allocations, and a device allocation is not a
 * malloc:
 *
 * - Vulkan caps them. `VkPhysicalDeviceLimits::maxMemoryAllocationCount` is
 *   commonly 4096. Past it, allocation fails with VK_ERROR_OUT_OF_DEVICE_MEMORY
 *   and "Failed to bind memory for buffer" while the GPU still has gigabytes free
 *   -- the failure has nothing to do with how much memory is left.
 * - Each one carries driver-side bookkeeping, and every bind is a distinct object
 *   the driver must track and validate.
 * - Each one is padded up to the allocator's granularity, so a 900-byte index
 *   buffer can cost several kilobytes of real memory.
 *
 * The fix is the one every streaming renderer arrives at: allocate a few large
 * blocks and hand out RANGES inside them. A mesh stops being a buffer and becomes
 * an offset and a size.
 *
 * ### Why this is expressible without touching the shaders
 *
 * SDL_GPUBufferBinding carries a byte `offset` alongside its buffer, and
 * SDL_DrawGPUIndexedPrimitives takes a `first_index`. So a mesh living at byte
 * offset N of a shared block binds `{block, N}` and draws from index 0 of its own
 * range. Vertex indices stay mesh-local and zero-based, the vertex layout is
 * unchanged, and no shader knows anything happened.
 *
 * ### Alignment is the caller's problem, and it is a real one
 *
 * A binding offset must satisfy the API's alignment rules for what is bound
 * there. An index buffer of 32-bit indices needs a 4-byte-aligned offset. A vertex
 * buffer binding offset must be a multiple of the vertex stride if the draw also
 * uses a non-zero vertex offset, and is safest at the stride regardless --
 * `sizeof(Vertex)` is 64 bytes, which satisfies everything. allocate() takes the
 * alignment rather than guessing, because the pool does not know what will be bound
 * to the range it is handing out.
 *
 * This file is RENDERER, not core: it is the one piece of P7 that touches SDL_GPU.
 * The mesh optimisation, collision and export work stays in stratum_core.
 */

#pragma once

#include <SDL3/SDL.h>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace stratum {

/**
 * @brief A range inside a shared device buffer
 *
 * Returned by GPUBufferPool::allocate() and handed back to
 * GPUBufferPool::free(). Copyable and trivially small; it is a handle, not an
 * owner, and destroying one leaks the range.
 *
 * To bind it:
 * @code
 *     SDL_GPUBufferBinding binding{ alloc.buffer, alloc.offset };
 * @endcode
 *
 * To copy into it:
 * @code
 *     SDL_GPUBufferRegion region{ alloc.buffer, alloc.offset, alloc.size };
 * @endcode
 */
struct BufferAlloc {
    /// The shared device block this range lives in. Owned by the pool, never released by the holder.
    SDL_GPUBuffer* buffer = nullptr;

    /// Byte offset of the range within @ref buffer
    uint32_t offset = 0;

    /**
     * @brief Byte size of the range
     *
     * The size that was REQUESTED, not the padded size the pool reserved. A copy
     * of exactly this many bytes to exactly this offset is always in bounds.
     */
    uint32_t size = 0;

    /**
     * @brief Index of the owning block, so free() does not have to search
     *
     * Meaningless on its own and only valid against the pool that issued it.
     * Passing an alloc to a different pool's free() is undefined.
     */
    uint32_t block = 0;

    /// True when this handle refers to a real range
    bool valid() const { return buffer != nullptr && size > 0; }
};

/**
 * @brief Suballocates fixed-size device blocks into per-mesh ranges
 *
 * One pool per SDL_GPUBufferUsageFlags: a block is created with a single usage
 * and a range inherits it, so vertex and index data cannot share a pool.
 *
 * @warning NOT thread safe. Every entry point mutates the free lists without
 *          locking. Call from the render thread only -- which is where
 *          GPURenderer::upload_mesh, flush_pending_uploads and release_mesh
 *          already are.
 *
 * @warning free() makes a range immediately available for reuse, with no regard
 *          for whether the GPU has finished reading it. A range freed while a
 *          previously submitted command buffer still draws from it, then
 *          reallocated and overwritten by a copy pass, corrupts that draw. The
 *          POOL cannot know when that is safe. The CALLER must defer free() by at
 *          least the renderer's frame latency; GPURenderer does this with a
 *          retirement queue rather than freeing at release_mesh() time.
 */
class GPUBufferPool {
public:
    /**
     * @brief Construct an empty, uninitialised pool
     *
     * Defined out of line, like the destructor and for the same reason: @ref Block
     * is incomplete in this header, and a defaulted constructor here would be
     * instantiated in every translation unit that merely declares a
     * GPUBufferPool member -- GPURenderer does -- forcing std::vector<Block> to
     * be instantiated against an incomplete type. libstdc++ rejects that outright.
     * Both special members are declared here and defaulted in gpu_buffer_pool.cpp,
     * where Block is complete.
     */
    GPUBufferPool();
    ~GPUBufferPool();

    // Non-copyable: it owns device allocations.
    GPUBufferPool(const GPUBufferPool&) = delete;
    GPUBufferPool& operator=(const GPUBufferPool&) = delete;

    /**
     * @brief Prepare the pool. No device memory is allocated until the first allocate().
     *
     * @param device     Live SDL_GPU device. Must outlive the pool.
     * @param usage      SDL_GPU_BUFFERUSAGE_VERTEX or SDL_GPU_BUFFERUSAGE_INDEX.
     *                   One pool per usage; a pool never mixes them.
     * @param block_size Bytes per device block. Should comfortably exceed the
     *                   largest single allocation -- an allocation larger than this
     *                   still succeeds, via a dedicated block, but a pool that hits
     *                   that path routinely has bought nothing. 32 MB for vertices
     *                   and 8 MB for indices are the shipping values.
     * @return true on success. false when @p device is null, @p block_size is 0, or
     *         the pool is already initialised.
     */
    bool init(SDL_GPUDevice* device, SDL_GPUBufferUsageFlags usage, uint32_t block_size);

    /**
     * @brief Release every device block and reset to the uninitialised state
     *
     * @warning Every outstanding BufferAlloc dangles afterwards. Release the meshes
     *          that hold them first. Outstanding allocations are logged as a leak
     *          rather than silently dropped, because a non-zero count here means a
     *          mesh was destroyed without its ranges being freed.
     *
     * Safe to call on an uninitialised pool, and safe to call twice.
     */
    void shutdown();

    /// True once init() has succeeded and shutdown() has not been called
    [[nodiscard]] bool is_initialized() const { return m_device != nullptr; }

    /**
     * @brief Reserve a range
     *
     * First fit over the existing blocks' free lists, in block order, then the
     * first block with room at its high-water mark, then a new block. First fit
     * rather than best fit: the allocation sizes here cluster hard -- most road
     * pieces are within an order of magnitude of each other -- so best fit costs a
     * scan and buys almost nothing.
     *
     * An allocation LARGER than the block size gets a dedicated block sized to fit
     * it exactly. That block is destroyed the moment its single allocation is
     * freed, instead of being retained like a normal block, because a 90 MB block
     * held for reuse is worse than a re-allocation.
     *
     * @param size      Bytes required. 0 returns an invalid BufferAlloc.
     * @param alignment Required alignment of the returned offset, in bytes. Must be
     *                  a power of two; anything else is rounded up to one. Pass
     *                  `sizeof(Vertex)` for a vertex pool and 4 for a 32-bit index
     *                  pool. See the alignment note in this file's overview.
     * @return The range, or a default-constructed BufferAlloc when the pool is not
     *         initialised or the device refused a new block. valid() is the only
     *         check a caller needs.
     */
    [[nodiscard]] BufferAlloc allocate(uint32_t size, uint32_t alignment = 16);

    /**
     * @brief Return a range to its block
     *
     * The freed range is merged with any free range immediately before or after it.
     * That coalescing is not an optimisation, it is the difference between a pool
     * that works for an hour and one that does not: a streaming session frees and
     * reallocates constantly, and without coalescing the free list degenerates into
     * thousands of unusable slivers. The pool then reports megabytes free while
     * every allocation fails, which is the worst possible failure mode because the
     * stats say nothing is wrong. Stats::fragmentation exists to make that
     * visible if it ever happens anyway.
     *
     * A dedicated oversized block is destroyed here rather than retained.
     *
     * @param alloc Range from this pool's allocate(). An invalid alloc is a no-op.
     *              Freeing the same range twice, or a range from another pool, is
     *              undefined.
     */
    void free(const BufferAlloc& alloc);

    /**
     * @brief Occupancy and fragmentation, for the stats panel and for tests
     */
    struct Stats {
        /// Device allocations held. This is the number that hits maxMemoryAllocationCount.
        size_t blocks = 0;

        /// Bytes across every block, whether used or not
        size_t bytes_reserved = 0;

        /// Bytes covered by live allocations, padding for alignment included
        size_t bytes_used = 0;

        /// Ranges handed out and not yet freed
        size_t live_allocations = 0;

        /**
         * @brief How broken up the free space is, in [0, 1]
         *
         * `1 - largest_free_range / total_free_bytes`, over every block. 0 means
         * all free space is one contiguous run, 1 means it is dust. 0 when there
         * is no free space at all, since undefined fragmentation should not read
         * as bad fragmentation.
         *
         * A value climbing past roughly 0.8 while allocations start failing is the
         * signature of coalescing not doing its job.
         */
        float fragmentation = 0.0f;
    };

    /// Current occupancy. Cheap: computed from the free lists, which are short.
    [[nodiscard]] Stats stats() const;

private:
    /**
     * @brief One device buffer and the free ranges inside it
     *
     * Implementation notes for whoever writes gpu_buffer_pool.cpp:
     *
     * - Keep the free list SORTED BY OFFSET. Coalescing on free() is then a look at
     *   the neighbours either side of the insertion point, rather than a scan.
     * - `used` must count the PADDED size so it agrees with what the block can still
     *   hand out; BufferAlloc::size stays the requested size so copies are exact.
     * - Emptied normal blocks are released back to the device, but with hysteresis:
     *   ONE completely empty block is always retained. Releasing on sight would
     *   make a streaming session alternate between creating and destroying the same
     *   32 MB allocation, which is the churn this class exists to remove; retaining
     *   one means the cycle needs the working set to swing by more than a whole
     *   block. It is also what guarantees the pool never drops back to zero blocks
     *   once it has grown one. Dedicated oversized blocks are the exception and are
     *   destroyed on free, because holding 90 MB against the chance of an equally
     *   large mesh is worse than paying for the re-allocation.
     * - A released block leaves a RETIRED SLOT: the device buffer is gone, the
     *   vector element stays so BufferAlloc::block indices keep meaning what they
     *   meant. A retired slot is reused by the next block that needs creating, so
     *   m_blocks does not grow without bound over a long session. This is safe
     *   because a slot is only ever retired while it holds no live allocation.
     */
    struct Block;

    /**
     * @brief Blocks in creation order; BufferAlloc::block indexes this vector
     *
     * Never reordered and never erased while the pool lives, so a stored index
     * stays valid across any number of allocations. A destroyed block -- a
     * dedicated one on free, or an empty one beyond the retained spare -- leaves a
     * retired slot in place rather than shifting its successors.
     *
     * Block is incomplete here on purpose: the free-list representation is an
     * implementation detail of gpu_buffer_pool.cpp, and no consumer of this header
     * should be able to reach into it. That is why GPUBufferPool() and
     * ~GPUBufferPool() are declared here and defined there.
     */
    std::vector<Block> m_blocks;

    SDL_GPUDevice* m_device = nullptr;
    SDL_GPUBufferUsageFlags m_usage = 0;
    uint32_t m_block_size = 0;
};

} // namespace stratum
