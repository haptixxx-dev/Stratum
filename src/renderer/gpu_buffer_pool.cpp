/**
 * @file gpu_buffer_pool.cpp
 * @brief Suballocating device buffer pool implementation
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * ### Free-list design
 *
 * Every block owns a free list that covers ALL of its unallocated bytes, kept
 * sorted by offset, disjoint, and maximal -- no two entries ever touch. A fresh
 * block therefore starts as a single free range `[0, capacity)`, which means the
 * "high-water mark" of a mostly-fresh block is just the last free range and needs
 * no separate bump allocator: first fit over the free list already finds it, and
 * on a monotonically growing import it finds it on the first probe.
 *
 * Two rules make the whole thing small:
 *
 * 1. **An allocation consumes a PREFIX of exactly one free range.** The alignment
 *    padding in front of the aligned offset is charged to the allocation instead
 *    of being left behind as a free sliver, so a split produces at most one
 *    remainder rather than two. That is why the padding is <= alignment-1 bytes
 *    of waste but zero free-list entries: a 63-byte hole nothing can ever use
 *    still costs a list entry, still shows up in `total_free`, and still drags
 *    Stats::fragmentation toward 1 while meaning nothing is wrong.
 * 2. **The reserved span is remembered, the requested size is returned.**
 *    BufferAlloc::size is what the caller asked for, so a copy of exactly that
 *    many bytes to exactly that offset is in bounds. The pool separately records
 *    the padded span it actually took out of the free list, keyed by the offset
 *    it handed back, so free() can return precisely what allocate() removed.
 *    Live offsets are unique within a block -- live spans are disjoint and each
 *    starts at a distinct offset -- so that key is unambiguous, and a lookup miss
 *    is a double free rather than an ambiguity.
 *
 * ### Why coalescing two neighbours is sufficient
 *
 * Let the free list F satisfy the invariants above and let R = [o, o+s) be a
 * range being returned. R is disjoint from every entry of F, because R was live
 * and live and free space never overlap.
 *
 * Take `i` = the first index with `F[i].offset > o`. Disjointness gives
 * `F[i-1].end() <= o` and `o + s <= F[i].offset`, so R sits in the gap between
 * F[i-1] and F[i] and can be adjacent to those two entries and to nothing else.
 * Merge left when `F[i-1].end() == o`, merge right when `o + s == F[i].offset`.
 *
 * The merged entry M spans exactly F[i-1] u R u F[i], which is contiguous. Its
 * new neighbours are F[i-2] and F[i+1]. Maximality held before, so
 * `F[i-2].end() < F[i-1].offset == M.offset` and
 * `M.end() == F[i].end() < F[i+1].offset`. Both are strict, so M touches neither
 * and no further merging is possible. Checking exactly two neighbours restores
 * the invariant in full; there is no cascade to chase.
 *
 * By induction over the sequence of frees, the list stays maximal forever, so a
 * block that is entirely free is always ONE entry of `capacity` bytes and can be
 * handed straight back out. This is the property that keeps a long streaming
 * session from degenerating into slivers that report megabytes free while every
 * allocation fails.
 *
 * ### Index offsets and first_index
 *
 * An index range gets an alignment floor of `sizeof(uint32_t)` that the pool
 * applies itself, ignoring a smaller alignment from the caller, because both ways
 * of addressing a suballocated index range need it:
 *
 * - **Binding offset.** `SDL_GPUBufferBinding{ block, offset }` with
 *   SDL_GPU_INDEXELEMENTSIZE_32BIT requires `offset` to land on an index element
 *   boundary. `offset % 4 != 0` is a mid-index bind, which is invalid.
 * - **first_index.** The other way to reach a range is to bind the block at 0 and
 *   pass the mesh base through `first_index` of SDL_DrawGPUIndexedPrimitives.
 *   That parameter counts INDICES, not bytes, so the base is expressible only if
 *   `offset / 4` is exact -- the same `offset % 4 == 0` constraint, arrived at
 *   from the other direction.
 *
 * GPURenderer uses the first form, which keeps SubMesh::index_offset relative to
 * the mesh and lets it pass straight through as `first_index`. The floor means
 * the second form stays available and that the two compose: a submesh's absolute
 * index is `offset / 4 + index_offset`, an exact integer for every range this
 * pool can hand out. A caller that passes `alignment = 1` to an index pool cannot
 * produce an unbindable offset.
 *
 * Range SIZES need no such rule; they are `index_count * sizeof(uint32_t)` and
 * are multiples of 4 by construction. Only offsets are constrained.
 *
 * @warning NOT thread safe, by design and not by omission. allocate(), free(),
 *          init() and shutdown() all mutate the free lists unguarded. Every
 *          caller -- GPURenderer::upload_mesh, flush_pending_uploads,
 *          drain_retired_allocs, release_mesh -- runs on the render thread, so a
 *          mutex here would be uncontended overhead on the hottest path of an
 *          import. Do not call this from a worker.
 */

#include "renderer/gpu_buffer_pool.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <utility>

namespace stratum {

namespace {

/// Byte size of one 32-bit index. Index offsets must be a multiple of this.
constexpr uint32_t kIndexElementSize = static_cast<uint32_t>(sizeof(uint32_t));

/**
 * @brief Completely empty normal blocks kept rather than released
 *
 * This is the hysteresis. Releasing a block the instant it empties makes a
 * streaming session alternate between SDL_CreateGPUBuffer and
 * SDL_ReleaseGPUBuffer on the same 32 MB allocation every time the working set
 * crosses a block boundary, which is exactly the device-allocation churn this
 * class exists to remove. Retaining one empty block means the churn cycle needs
 * the working set to swing by more than a whole block, and the retained block is
 * also what guarantees the pool keeps at least one block once it has grown one.
 */
constexpr size_t kRetainedEmptyBlocks = 1;

/// Largest alignment the pool will honour. Keeps align_up() away from overflow.
constexpr uint32_t kMaxAlignment = 1u << 20;

constexpr bool is_power_of_two(uint32_t v) {
    return v != 0u && (v & (v - 1u)) == 0u;
}

/// Smallest power of two >= @p v. Callers clamp to kMaxAlignment first, so the
/// shift chain never has to reach a value that would wrap on the final increment.
constexpr uint32_t next_power_of_two(uint32_t v) {
    if (v <= 1u) {
        return 1u;
    }
    --v;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    return v + 1u;
}

/// Round @p v up to a multiple of the power of two @p alignment. 64-bit to avoid wrap.
constexpr uint64_t align_up(uint64_t v, uint64_t alignment) {
    return (v + (alignment - 1u)) & ~(alignment - 1u);
}

} // namespace

/**
 * @brief One device buffer plus the free ranges inside it
 *
 * Deliberately defined here and not in the header: the free-list representation
 * is nobody else's business, and keeping it out of the header is what lets the
 * layout change without recompiling every renderer translation unit.
 */
struct GPUBufferPool::Block {
    /// A half-open byte range `[offset, offset + size)` inside this block
    struct Span {
        uint32_t offset = 0;
        uint32_t size = 0;
        uint32_t end() const { return offset + size; }
    };

    /// Device allocation. Null means this slot is retired and reusable.
    SDL_GPUBuffer* buffer = nullptr;

    /// Bytes in the device allocation
    uint32_t capacity = 0;

    /// Reserved bytes, alignment padding included. Always `capacity - sum(free_list)`.
    uint32_t used = 0;

    /// True for a block created to hold one oversized allocation; destroyed on free.
    bool dedicated = false;

    /// Free ranges, sorted by offset, disjoint, and never adjacent to each other.
    std::vector<Span> free_list;

    /// Returned offset -> the padded span allocate() removed from the free list.
    std::unordered_map<uint32_t, Span> live_spans;

    /// Cached size of the largest free range. Exact whenever @ref max_free_stale is false.
    uint32_t max_free = 0;
    bool max_free_stale = false;

    /// A retired slot: the device buffer is gone but the index stays valid.
    bool retired() const { return buffer == nullptr; }

    /// No live allocations. An empty block's free list is exactly one full-capacity entry.
    bool is_empty() const { return live_spans.empty(); }

    /**
     * @brief Size of the largest free range, refreshing the cache when stale
     *
     * Used as a cheap necessary condition before scanning: `largest_free() < size`
     * proves the block cannot fit the request, so a full block is rejected in O(1)
     * instead of being walked. It is only a NECESSARY condition -- a range large
     * enough may still fail once the offset is aligned up -- so a pass that gets
     * past it can still come back empty handed, and that is correct.
     */
    uint32_t largest_free() {
        if (max_free_stale) {
            max_free = 0;
            for (const Span& f : free_list) {
                max_free = std::max(max_free, f.size);
            }
            max_free_stale = false;
        }
        return max_free;
    }

    /**
     * @brief Carve @p size bytes at @p alignment out of this block
     *
     * First fit. The allocation takes the whole prefix of the chosen free range up
     * to the end of the aligned request, so the range is either erased outright or
     * shortened from the front, and never splits into two.
     *
     * @param alignment Power of two, already normalised by the caller.
     * @param out_offset Receives the aligned offset on success.
     * @return true when the block had room.
     */
    bool try_allocate(uint32_t size, uint32_t alignment, uint32_t& out_offset) {
        if (retired() || size == 0) {
            return false;
        }
        if (largest_free() < size) {
            return false;
        }

        for (size_t i = 0; i < free_list.size(); ++i) {
            const Span f = free_list[i];
            const uint64_t aligned = align_up(f.offset, alignment);
            const uint64_t end = aligned + size;
            if (end > static_cast<uint64_t>(f.end())) {
                continue;
            }

            // Charge the alignment padding to the allocation rather than leaving a
            // sliver behind. See the file header for why that matters.
            const Span reserved{ f.offset, static_cast<uint32_t>(end - f.offset) };
            const uint32_t remainder = f.size - reserved.size;
            const bool took_largest = (f.size == max_free);

            if (remainder == 0) {
                free_list.erase(free_list.begin() + static_cast<std::ptrdiff_t>(i));
            } else {
                // Shortening from the front preserves both sortedness and
                // maximality: the entry only moves right, still inside its old
                // extent, so it cannot reach its neighbours.
                free_list[i] = Span{ reserved.end(), remainder };
            }
            if (took_largest) {
                max_free_stale = true;
            }

            used += reserved.size;
            out_offset = static_cast<uint32_t>(aligned);
            live_spans.emplace(out_offset, reserved);
            return true;
        }
        return false;
    }

    /**
     * @brief Return @p s to the free list, merging it with either neighbour
     *
     * @p s must be disjoint from every existing free range, which holds for any
     * span that was live. Checking the two neighbours either side of the insertion
     * point is sufficient to keep the list maximal; the proof is in the file
     * header.
     */
    void insert_free(Span s) {
        if (s.size == 0) {
            return;
        }

        // First entry starting strictly after s.
        auto it = std::upper_bound(
            free_list.begin(), free_list.end(), s.offset,
            [](uint32_t offset, const Span& entry) { return offset < entry.offset; });

        bool merged_left = false;
        if (it != free_list.begin()) {
            auto prev = it - 1;
            if (prev->end() == s.offset) {
                prev->size += s.size;
                it = prev;
                merged_left = true;
            }
        }

        if (merged_left) {
            auto next = it + 1;
            if (next != free_list.end() && it->end() == next->offset) {
                it->size += next->size;
                free_list.erase(next); // `it` precedes the erased element, so it stays valid.
            }
        } else if (it != free_list.end() && s.end() == it->offset) {
            it->offset = s.offset;
            it->size += s.size;
        } else {
            it = free_list.insert(it, s);
        }

        if (it->size > max_free) {
            max_free = it->size;
        }
    }

    /**
     * @brief Create a device buffer and wrap it in a ready-to-use block
     *
     * @return A block whose `buffer` is null when the device refused the allocation.
     */
    static Block create(SDL_GPUDevice* device, SDL_GPUBufferUsageFlags usage, uint32_t bytes,
                        bool dedicated, const std::string& label) {
        Block block;

        SDL_GPUBufferCreateInfo info{};
        info.usage = usage;
        info.size = bytes;
        info.props = 0;

        block.buffer = SDL_CreateGPUBuffer(device, &info);
        if (block.buffer == nullptr) {
            return block;
        }
        SDL_SetGPUBufferName(device, block.buffer, label.c_str());

        block.capacity = bytes;
        block.used = 0;
        block.dedicated = dedicated;
        block.free_list.push_back(Span{ 0, bytes });
        block.max_free = bytes;
        block.max_free_stale = false;
        return block;
    }
};

// Both defined here rather than in the header: Block is complete only in this
// translation unit, and std::vector<Block> cannot be instantiated without it.
GPUBufferPool::GPUBufferPool() = default;

GPUBufferPool::~GPUBufferPool() {
    shutdown();
}

bool GPUBufferPool::init(SDL_GPUDevice* device, SDL_GPUBufferUsageFlags usage, uint32_t block_size) {
    if (m_device != nullptr) {
        spdlog::warn("GPUBufferPool::init() called on an already initialised pool");
        return false;
    }
    if (device == nullptr || block_size == 0) {
        spdlog::error("GPUBufferPool::init() needs a device and a non-zero block size");
        return false;
    }

    uint32_t effective_block = block_size;
    if ((usage & SDL_GPU_BUFFERUSAGE_INDEX) != 0) {
        // Round DOWN so the arithmetic cannot wrap. A block that is a whole number
        // of indices keeps every offset inside it reachable as a first_index.
        effective_block = effective_block & ~(kIndexElementSize - 1u);
        effective_block = std::max(effective_block, kIndexElementSize);
    }

    m_device = device;
    m_usage = usage;
    m_block_size = effective_block;
    m_blocks.clear();
    return true;
}

void GPUBufferPool::shutdown() {
    if (m_device == nullptr) {
        m_blocks.clear();
        m_usage = 0;
        m_block_size = 0;
        return;
    }

    size_t leaked_allocations = 0;
    size_t leaked_bytes = 0;
    size_t live_blocks = 0;
    for (const Block& block : m_blocks) {
        leaked_allocations += block.live_spans.size();
        leaked_bytes += block.used;
        if (!block.retired()) {
            ++live_blocks;
        }
    }

    if (leaked_allocations > 0) {
        // Not a pool bug. A caller destroyed a mesh without freeing its ranges, and
        // staying quiet about it would hide a leak that grows with every import.
        spdlog::error(
            "GPUBufferPool::shutdown(): {} allocation(s) still live across {} block(s), "
            "{} bytes reserved. A caller released geometry without returning its ranges.",
            leaked_allocations, live_blocks, leaked_bytes);
    }

    for (Block& block : m_blocks) {
        if (block.buffer != nullptr) {
            SDL_ReleaseGPUBuffer(m_device, block.buffer);
            block.buffer = nullptr;
        }
    }

    m_blocks.clear();
    m_device = nullptr;
    m_usage = 0;
    m_block_size = 0;
}

BufferAlloc GPUBufferPool::allocate(uint32_t size, uint32_t alignment) {
    BufferAlloc result;
    if (m_device == nullptr || size == 0) {
        return result;
    }

    // Normalise the alignment. A non-power-of-two is rounded up rather than
    // rejected, because the alternative is handing back an invalid alloc for a
    // request that is perfectly serviceable.
    // Clamp BEFORE rounding up: rounding first could wrap a near-2^32 request to 0,
    // and an alignment of 0 turns align_up() into a mask of all zeroes, which would
    // silently hand every allocation the offset 0 of whatever range it landed in.
    uint32_t align = (alignment == 0u) ? 1u : alignment;
    align = std::min(align, kMaxAlignment);
    if (!is_power_of_two(align)) {
        align = next_power_of_two(align);
    }
    if ((m_usage & SDL_GPU_BUFFERUSAGE_INDEX) != 0) {
        align = std::max(align, kIndexElementSize);
    }

    const bool oversized = size > m_block_size;

    if (!oversized) {
        // First fit over the existing blocks, in block order. The free list covers
        // a fresh block's whole capacity, so this pass also finds the tail of a
        // partly filled block; there is no separate high-water-mark stage.
        for (size_t i = 0; i < m_blocks.size(); ++i) {
            Block& block = m_blocks[i];
            if (block.retired() || block.dedicated) {
                continue;
            }
            uint32_t offset = 0;
            if (block.try_allocate(size, align, offset)) {
                result.buffer = block.buffer;
                result.offset = offset;
                result.size = size;
                result.block = static_cast<uint32_t>(i);
                return result;
            }
        }
    }

    // Nothing fit. A request larger than the block size gets a block sized exactly
    // to it, so one 90 MB mesh does not raise the price of every subsequent block
    // to 90 MB. Offset 0 satisfies every power-of-two alignment, so a dedicated
    // block always services its request on the first probe.
    const uint32_t bytes = oversized ? size : m_block_size;

    // Reuse a retired slot when there is one. Retired slots are only created by
    // blocks that were empty at the time, so no live BufferAlloc can name one, and
    // reusing them stops m_blocks growing without bound over a long session.
    size_t slot = m_blocks.size();
    for (size_t i = 0; i < m_blocks.size(); ++i) {
        if (m_blocks[i].retired()) {
            slot = i;
            break;
        }
    }

    const bool is_index_pool = (m_usage & SDL_GPU_BUFFERUSAGE_INDEX) != 0;
    const std::string label = std::string("stratum.pool.") + (is_index_pool ? "index" : "vertex") +
                              ".block" + std::to_string(slot) + (oversized ? ".dedicated" : "");

    Block fresh = Block::create(m_device, m_usage, bytes, oversized, label);
    if (fresh.buffer == nullptr) {
        spdlog::error("GPUBufferPool: SDL_CreateGPUBuffer failed for a {} byte block ({} live blocks): {}",
                      bytes, stats().blocks, SDL_GetError());
        return result;
    }

    if (slot == m_blocks.size()) {
        m_blocks.push_back(std::move(fresh));
    } else {
        m_blocks[slot] = std::move(fresh);
    }

    Block& block = m_blocks[slot];
    uint32_t offset = 0;
    if (!block.try_allocate(size, align, offset)) {
        // Unreachable: the block was sized to hold this request and starts empty.
        spdlog::error("GPUBufferPool: fresh {} byte block refused a {} byte request at alignment {}",
                      bytes, size, align);
        return result;
    }

    result.buffer = block.buffer;
    result.offset = offset;
    result.size = size;
    result.block = static_cast<uint32_t>(slot);
    return result;
}

void GPUBufferPool::free(const BufferAlloc& alloc) {
    if (!alloc.valid() || m_device == nullptr) {
        return;
    }
    if (alloc.block >= m_blocks.size()) {
        spdlog::error("GPUBufferPool::free(): block index {} is out of range ({} blocks). "
                      "The alloc came from a different pool.",
                      alloc.block, m_blocks.size());
        return;
    }

    Block& block = m_blocks[alloc.block];
    if (block.retired() || block.buffer != alloc.buffer) {
        spdlog::error("GPUBufferPool::free(): alloc at offset {} does not belong to block {}. "
                      "Double free, or an alloc from a different pool.",
                      alloc.offset, alloc.block);
        return;
    }

    auto it = block.live_spans.find(alloc.offset);
    if (it == block.live_spans.end()) {
        spdlog::error("GPUBufferPool::free(): no live allocation at offset {} in block {}. "
                      "This range was already freed.",
                      alloc.offset, alloc.block);
        return;
    }

    const Block::Span reserved = it->second;
    block.live_spans.erase(it);
    block.used -= reserved.size;
    block.insert_free(reserved);

    if (block.dedicated) {
        // A dedicated block exists for one allocation. Holding, say, 90 MB on the
        // chance that an equally huge mesh turns up is worse than paying for the
        // re-allocation if one does.
        SDL_ReleaseGPUBuffer(m_device, block.buffer);
        block = Block{};
        return;
    }

    if (!block.is_empty()) {
        return;
    }

    // The block is now entirely free -- one free-list entry of `capacity` bytes,
    // by the maximality invariant. Keep kRetainedEmptyBlocks of these as the
    // hysteresis against create/destroy churn, and release any beyond that. The
    // just-emptied block is included in the count, so the first empty block is
    // always retained and the pool never drops to zero blocks.
    size_t empty_blocks = 0;
    for (const Block& other : m_blocks) {
        if (!other.retired() && !other.dedicated && other.is_empty()) {
            ++empty_blocks;
        }
    }
    if (empty_blocks > kRetainedEmptyBlocks) {
        SDL_ReleaseGPUBuffer(m_device, block.buffer);
        block = Block{};
    }
}

GPUBufferPool::Stats GPUBufferPool::stats() const {
    Stats result;

    size_t total_free = 0;
    size_t largest_free = 0;

    for (const Block& block : m_blocks) {
        if (block.retired()) {
            continue;
        }
        ++result.blocks;
        result.bytes_reserved += block.capacity;
        result.bytes_used += block.used;
        result.live_allocations += block.live_spans.size();

        for (const Block::Span& span : block.free_list) {
            total_free += span.size;
            largest_free = std::max(largest_free, static_cast<size_t>(span.size));
        }
    }

    if (total_free > 0) {
        const float ratio = static_cast<float>(largest_free) / static_cast<float>(total_free);
        result.fragmentation = std::clamp(1.0f - ratio, 0.0f, 1.0f);
    }

    return result;
}

} // namespace stratum
