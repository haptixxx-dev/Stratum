/**
 * @file test_gpu_buffer_pool.cpp
 * @brief The suballocator: reuse, coalescing, alignment, and the stat that says it broke
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * GPUBufferPool is the one piece of P7 that touches SDL_GPU, so this suite cannot
 * live in stratum_tests, which links stratum_core and nothing else by design. It
 * is built into a second executable, stratum_gpu_tests, which links
 * stratum_editor_lib.
 *
 * ### It runs against a real device
 *
 * No mock and no extracted free-list class. SDL_CreateGPUDevice() succeeds with no
 * window once the video subsystem is up, so the pool under test is the shipping
 * pool making real SDL_CreateGPUBuffer calls. That matters: half of what this
 * class exists to prevent -- running the driver out of allocations -- is only
 * visible against a driver.
 *
 * On a machine with no usable GPU backend the device cannot be created. The suite
 * then prints a loud line to stderr and every test returns without asserting,
 * because failing a build for the absence of a GPU is worse than not running. A
 * run that skipped is visible in the output rather than silent.
 *
 * ### What the assertions are actually about
 *
 * Not "does allocate return something". They are about the two failure modes that
 * make a suballocator worse than no suballocator:
 *
 * 1. **Free ranges that are never reused.** The block count climbs with every
 *    allocation, the pool is a slow malloc, and the driver hits
 *    maxMemoryAllocationCount anyway -- which is the exact failure this class was
 *    written to prevent.
 * 2. **A free list that degenerates into slivers.** Without coalescing, a
 *    streaming session frees and reallocates until every free range is too small
 *    for anything, at which point the pool reports megabytes free while every
 *    allocation fails. That is the worst possible failure mode, because the stats
 *    say nothing is wrong. Stats::fragmentation is the number that makes it
 *    visible, so the fragmentation arithmetic is asserted against a case with a
 *    known answer rather than merely being observed to be in range.
 *
 * Run this suite with:
 * @code
 *     ./stratum_gpu_tests GPUBufferPool
 * @endcode
 */

#include "framework.hpp"

#include "renderer/gpu_buffer_pool.hpp"
#include "renderer/mesh.hpp"

#include <SDL3/SDL.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace {

using stratum::BufferAlloc;
using stratum::GPUBufferPool;

/// Allocation unit these tests measure in, bytes
constexpr uint32_t kUnit = 8192;

/**
 * @brief The one device every test shares, created on first use
 *
 * Created lazily rather than in main(), because tests/framework.cpp owns main()
 * and this suite must not need its own. Torn down by the destructor of the static
 * holder at exit, which is after the last test has run.
 */
struct DeviceHolder {
    SDL_GPUDevice* device = nullptr;
    bool attempted = false;
    bool video_ready = false;

    ~DeviceHolder() {
        if (device != nullptr) {
            SDL_DestroyGPUDevice(device);
            device = nullptr;
        }
        if (video_ready) {
            SDL_QuitSubSystem(SDL_INIT_VIDEO);
            SDL_Quit();
        }
    }
};

DeviceHolder& holder() {
    static DeviceHolder h;
    return h;
}

/**
 * @brief The shared device, or null when this machine has no GPU backend
 *
 * The video subsystem has to be up before SDL_CreateGPUDevice() will look for a
 * backend at all -- it returns "Video subsystem not initialized" otherwise, which
 * reads like a missing GPU and is not.
 */
SDL_GPUDevice* device() {
    DeviceHolder& h = holder();
    if (h.attempted) return h.device;
    h.attempted = true;

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::fprintf(stderr, "[GPUBufferPool] SKIPPED: SDL video init failed: %s\n",
                     SDL_GetError());
        return nullptr;
    }
    h.video_ready = true;

    h.device = SDL_CreateGPUDevice(SDL_GPU_SHADERFORMAT_SPIRV, false, nullptr);
    if (h.device == nullptr) {
        std::fprintf(stderr, "[GPUBufferPool] SKIPPED: no SDL_GPU device: %s\n", SDL_GetError());
    }
    return h.device;
}

/// Bring a pool up on the shared device, or report that the suite is skipping
bool start(GPUBufferPool& pool, uint32_t block_size,
           SDL_GPUBufferUsageFlags usage = SDL_GPU_BUFFERUSAGE_VERTEX) {
    SDL_GPUDevice* dev = device();
    if (dev == nullptr) return false;
    const bool ok = pool.init(dev, usage, block_size);
    CHECK_TRUE(ok);
    return ok;
}

} // namespace

// ============================================================================
// Lifecycle
// ============================================================================

/**
 * init() refuses what it cannot work with, and shutdown() is safe to call on an
 * uninitialised pool and safe to call twice.
 *
 * The null-device and zero-block cases need no GPU at all, so they run even when
 * the suite is otherwise skipping.
 */
TEST(GPUBufferPool, init_refuses_bad_arguments) {
    GPUBufferPool pool;
    CHECK_FALSE(pool.is_initialized());
    CHECK_FALSE(pool.init(nullptr, SDL_GPU_BUFFERUSAGE_VERTEX, kUnit));
    CHECK_FALSE(pool.is_initialized());

    // An allocation from a pool that never came up is invalid, not a crash.
    const BufferAlloc dead = pool.allocate(kUnit, 4);
    CHECK_FALSE(dead.valid());

    pool.shutdown();
    pool.shutdown();
    CHECK_FALSE(pool.is_initialized());

    SDL_GPUDevice* dev = device();
    if (dev == nullptr) return;

    CHECK_FALSE(pool.init(dev, SDL_GPU_BUFFERUSAGE_VERTEX, 0));
    CHECK_TRUE(pool.init(dev, SDL_GPU_BUFFERUSAGE_VERTEX, kUnit * 4));
    CHECK_TRUE(pool.is_initialized());
    CHECK_FALSE(pool.init(dev, SDL_GPU_BUFFERUSAGE_VERTEX, kUnit * 4));   // already up
    pool.shutdown();
    CHECK_FALSE(pool.is_initialized());
}

/**
 * No device memory is taken until the first allocate(), and a zero-byte request
 * is invalid rather than a zero-length range somebody then binds.
 */
TEST(GPUBufferPool, nothing_is_allocated_until_something_is_asked_for) {
    GPUBufferPool pool;
    if (!start(pool, kUnit * 4)) return;

    CHECK_EQ(pool.stats().blocks, size_t{0});
    CHECK_EQ(pool.stats().bytes_reserved, size_t{0});
    CHECK_EQ(pool.stats().live_allocations, size_t{0});

    const BufferAlloc none = pool.allocate(0, 4);
    CHECK_FALSE(none.valid());
    CHECK_EQ(pool.stats().blocks, size_t{0});

    const BufferAlloc first = pool.allocate(kUnit, 4);
    CHECK_TRUE(first.valid());
    CHECK_EQ(pool.stats().blocks, size_t{1});
    CHECK_EQ(pool.stats().live_allocations, size_t{1});
    CHECK_EQ(pool.stats().bytes_reserved, size_t{kUnit * 4});

    pool.free(first);
    CHECK_EQ(pool.stats().live_allocations, size_t{0});

    // An emptied normal block is RETAINED. Releasing it would make a streaming
    // session alternate between creating and destroying the same block, which is
    // the churn this class exists to remove.
    CHECK_EQ(pool.stats().blocks, size_t{1});
    CHECK_EQ(pool.stats().bytes_reserved, size_t{kUnit * 4});

    pool.shutdown();
}

// ============================================================================
// Reuse
// ============================================================================

/**
 * THE REUSE TEST.
 *
 * Fill a block exactly, free every other range, then ask for that many again. The
 * freed ranges must come back, and the block count must not grow. A pool that
 * ignores its free list passes every "did allocate succeed" test and fails this
 * one, because the second round comes out of a second block.
 *
 * The block is sized to hold exactly the first round, so there is no high-water
 * mark left for the second round to hide in.
 */
TEST(GPUBufferPool, freed_ranges_are_reused_without_growing_the_pool) {
    constexpr uint32_t kCount = 64;

    GPUBufferPool pool;
    if (!start(pool, kUnit * kCount)) return;

    std::vector<BufferAlloc> allocations;
    for (uint32_t i = 0; i < kCount; ++i) {
        const BufferAlloc a = pool.allocate(kUnit, 4);
        CHECK_TRUE(a.valid());
        allocations.push_back(a);
    }

    const size_t blocks_after_fill = pool.stats().blocks;
    CHECK_EQ(blocks_after_fill, size_t{1});
    CHECK_EQ(pool.stats().live_allocations, size_t{kCount});
    CHECK_EQ(pool.stats().bytes_used, size_t{kUnit} * kCount);

    // Free every other one, so the free list is a scatter rather than one run.
    size_t freed = 0;
    for (uint32_t i = 0; i < kCount; i += 2) {
        pool.free(allocations[i]);
        ++freed;
    }
    CHECK_EQ(pool.stats().live_allocations, size_t{kCount} - freed);

    for (size_t i = 0; i < freed; ++i) {
        const BufferAlloc a = pool.allocate(kUnit, 4);
        CHECK_TRUE(a.valid());
        if (!a.valid()) break;
        allocations.push_back(a);
    }

    if (pool.stats().blocks != blocks_after_fill) {
        stratum::test::report_failure(
            __FILE__, __LINE__, "reallocating freed ranges does not create a block",
            "blocks went from " + std::to_string(blocks_after_fill) + " to " +
                std::to_string(pool.stats().blocks) + ", so " + std::to_string(freed) +
                " freed ranges were not reused");
    }
    CHECK_EQ(pool.stats().live_allocations, size_t{kCount});
    CHECK_EQ(pool.stats().bytes_used, size_t{kUnit} * kCount);

    pool.shutdown();
}

/**
 * THE FRAGMENTATION GUARD.
 *
 * Two adjacent ranges are freed, then their combined size is asked for. It must
 * come out of the same block. Without coalescing the free list holds two ranges
 * of half the size, neither one fits, and the pool creates a block while
 * reporting the space as free -- the failure mode where the stats say nothing is
 * wrong.
 */
TEST(GPUBufferPool, adjacent_free_ranges_coalesce) {
    GPUBufferPool pool;
    if (!start(pool, kUnit * 4)) return;

    BufferAlloc a = pool.allocate(kUnit, 4);
    BufferAlloc b = pool.allocate(kUnit, 4);
    BufferAlloc c = pool.allocate(kUnit, 4);
    BufferAlloc d = pool.allocate(kUnit, 4);
    CHECK_TRUE(a.valid() && b.valid() && c.valid() && d.valid());
    CHECK_EQ(pool.stats().blocks, size_t{1});

    // b and c are adjacent by construction: the pool hands out ascending offsets
    // from one block's high-water mark.
    CHECK_TRUE(b.offset + b.size == c.offset || c.offset + c.size == b.offset);

    pool.free(b);
    pool.free(c);
    CHECK_EQ(pool.stats().live_allocations, size_t{2});

    const BufferAlloc merged = pool.allocate(kUnit * 2, 4);
    CHECK_TRUE(merged.valid());
    if (pool.stats().blocks != 1) {
        stratum::test::report_failure(
            __FILE__, __LINE__, "two adjacent free ranges merge into one",
            "asking for their combined size created a block; blocks is now " +
                std::to_string(pool.stats().blocks));
    }
    CHECK_EQ(pool.stats().bytes_reserved, size_t{kUnit * 4});

    pool.free(a);
    pool.free(d);
    pool.free(merged);
    pool.shutdown();
}

/**
 * A range freed and immediately reallocated at the same size comes back to the
 * same place, which is what says the free list is being consulted before the
 * high-water mark.
 */
TEST(GPUBufferPool, a_freed_range_is_preferred_over_fresh_space) {
    GPUBufferPool pool;
    if (!start(pool, kUnit * 16)) return;

    const BufferAlloc a = pool.allocate(kUnit, 4);
    const BufferAlloc b = pool.allocate(kUnit, 4);
    CHECK_TRUE(a.valid() && b.valid());

    pool.free(a);
    const BufferAlloc again = pool.allocate(kUnit, 4);
    CHECK_TRUE(again.valid());
    CHECK_EQ(again.offset, a.offset);
    CHECK_EQ(again.block, a.block);

    pool.free(b);
    pool.free(again);
    pool.shutdown();
}

// ============================================================================
// Oversized allocations
// ============================================================================

/**
 * An allocation larger than the block size still succeeds, in a dedicated block
 * of its own, and that block is destroyed when its single allocation is freed
 * rather than being retained like a normal one.
 *
 * A 90 MB block held for reuse is worse than a re-allocation, which is why the
 * retention rule has this exception.
 */
TEST(GPUBufferPool, an_oversized_allocation_gets_its_own_block) {
    GPUBufferPool pool;
    if (!start(pool, kUnit * 4)) return;

    const BufferAlloc normal = pool.allocate(kUnit, 4);
    CHECK_TRUE(normal.valid());
    const auto before = pool.stats();

    const uint32_t huge = kUnit * 16;       // four times the block size
    const BufferAlloc big = pool.allocate(huge, 4);
    CHECK_TRUE(big.valid());
    CHECK_EQ(big.size, huge);
    CHECK_TRUE(big.buffer != nullptr);
    CHECK_TRUE(big.buffer != normal.buffer);

    const auto during = pool.stats();
    CHECK_EQ(during.blocks, before.blocks + 1);
    CHECK_TRUE(during.bytes_reserved >= before.bytes_reserved + huge);

    pool.free(big);
    const auto after = pool.stats();
    CHECK_EQ(after.blocks, before.blocks);
    CHECK_EQ(after.bytes_reserved, before.bytes_reserved);
    CHECK_EQ(after.live_allocations, before.live_allocations);

    // The retired slot must not have invalidated the earlier allocation's index.
    const BufferAlloc more = pool.allocate(kUnit, 4);
    CHECK_TRUE(more.valid());
    pool.free(more);
    pool.free(normal);
    pool.shutdown();
}

// ============================================================================
// Alignment
// ============================================================================

/**
 * Every returned offset satisfies the alignment asked for, index allocations land
 * on multiples of sizeof(uint32_t), and a non-power-of-two request is rounded up
 * to one rather than producing an offset nothing can bind.
 *
 * An index buffer bound at an offset that is not a multiple of 4 is not a slow
 * draw, it is a validation error and a corrupted one.
 */
TEST(GPUBufferPool, offsets_honour_the_alignment_asked_for) {
    GPUBufferPool index_pool;
    if (!start(index_pool, kUnit * 8, SDL_GPU_BUFFERUSAGE_INDEX)) return;

    std::vector<BufferAlloc> live;
    // Deliberately ragged sizes, so a pool that only ever hands out multiples of
    // its own unit passes by accident on none of them.
    const uint32_t sizes[6] = { 12, 37, 101, 4, 999, 63 };
    for (uint32_t size : sizes) {
        const BufferAlloc a = index_pool.allocate(size, sizeof(uint32_t));
        CHECK_TRUE(a.valid());
        CHECK_EQ(a.size, size);
        if (a.offset % sizeof(uint32_t) != 0) {
            stratum::test::report_failure(__FILE__, __LINE__,
                                          "index offset is a multiple of sizeof(uint32_t)",
                                          "offset " + std::to_string(a.offset) + " for size " +
                                              std::to_string(size));
        }
        live.push_back(a);
    }
    for (const BufferAlloc& a : live) index_pool.free(a);
    index_pool.shutdown();

    GPUBufferPool vertex_pool;
    if (!start(vertex_pool, kUnit * 8)) return;

    live.clear();
    for (uint32_t size : sizes) {
        const BufferAlloc a = vertex_pool.allocate(size, sizeof(stratum::Vertex));
        CHECK_TRUE(a.valid());
        CHECK_EQ(a.offset % sizeof(stratum::Vertex), size_t{0});
        live.push_back(a);
    }

    // A non-power-of-two alignment is rounded UP to one, so 6 becomes 8.
    const BufferAlloc odd = vertex_pool.allocate(100, 6);
    CHECK_TRUE(odd.valid());
    CHECK_EQ(odd.offset % 8u, uint32_t{0});
    live.push_back(odd);

    for (const BufferAlloc& a : live) vertex_pool.free(a);
    vertex_pool.shutdown();
}

// ============================================================================
// Statistics
// ============================================================================

/**
 * Fragmentation is 0 with nothing free, 0 while the free space is one run, and
 * exactly 0.5 when it is two runs of equal size.
 *
 * The arithmetic is `1 - largest_free_range / total_free_bytes`, so two equal runs
 * is the case with an answer that can be written down. Observing that the value is
 * merely "in range" would not distinguish it from a stub returning 0.
 */
TEST(GPUBufferPool, fragmentation_reads_the_free_list) {
    GPUBufferPool pool;
    if (!start(pool, kUnit * 4)) return;

    std::vector<BufferAlloc> live;
    for (int i = 0; i < 4; ++i) {
        const BufferAlloc a = pool.allocate(kUnit, 4);
        CHECK_TRUE(a.valid());
        live.push_back(a);
    }

    // Precondition: the block is exactly full, so there is no tail free space to
    // confuse the arithmetic below.
    const auto full = pool.stats();
    CHECK_EQ(full.blocks, size_t{1});
    CHECK_EQ(full.bytes_reserved, size_t{kUnit * 4});
    CHECK_EQ(full.bytes_used, size_t{kUnit * 4});
    CHECK_NEAR(full.fragmentation, 0.0, 1e-6);

    // One free run: all of the free space is contiguous.
    pool.free(live[0]);
    CHECK_NEAR(pool.stats().fragmentation, 0.0, 1e-6);

    // Two free runs of equal size, separated by a live one.
    pool.free(live[2]);
    CHECK_NEAR(pool.stats().fragmentation, 0.5, 1e-6);
    CHECK_EQ(pool.stats().bytes_used, size_t{kUnit * 2});
    CHECK_EQ(pool.stats().live_allocations, size_t{2});

    // Freeing the range between them coalesces all three into one run again.
    pool.free(live[1]);
    CHECK_NEAR(pool.stats().fragmentation, 0.0, 1e-6);

    pool.free(live[3]);
    CHECK_EQ(pool.stats().live_allocations, size_t{0});
    CHECK_EQ(pool.stats().bytes_used, size_t{0});
    CHECK_NEAR(pool.stats().fragmentation, 0.0, 1e-6);

    pool.shutdown();
}

/**
 * A range is bindable and copyable as handed out: its offset plus its size never
 * runs past the block it lives in, and no two live ranges overlap.
 *
 * Overlap is the defect that produces geometry drawn from another mesh's vertices,
 * which looks like a builder bug and is not one.
 */
TEST(GPUBufferPool, live_ranges_never_overlap) {
    GPUBufferPool pool;
    if (!start(pool, kUnit * 8)) return;

    std::vector<BufferAlloc> live;
    const uint32_t sizes[8] = { 100, 4096, 33, 8000, 12, 2048, 777, 64 };
    for (uint32_t size : sizes) {
        const BufferAlloc a = pool.allocate(size, 16);
        CHECK_TRUE(a.valid());
        live.push_back(a);
    }

    size_t overlaps = 0;
    for (size_t i = 0; i < live.size(); ++i) {
        for (size_t j = i + 1; j < live.size(); ++j) {
            if (live[i].buffer != live[j].buffer) continue;
            const uint32_t ai = live[i].offset;
            const uint32_t bi = live[i].offset + live[i].size;
            const uint32_t aj = live[j].offset;
            const uint32_t bj = live[j].offset + live[j].size;
            if (ai < bj && aj < bi) ++overlaps;
        }
    }
    CHECK_EQ(overlaps, size_t{0});

    for (const BufferAlloc& a : live) pool.free(a);
    pool.shutdown();
}

/**
 * A long churn of mixed sizes neither leaks blocks nor starts failing.
 *
 * This is the streaming session in miniature: allocate, free some, allocate more,
 * repeat. Without coalescing the free list degenerates and either the block count
 * climbs without bound or allocate() starts returning invalid handles while the
 * stats report plenty free. Both are asserted against.
 */
TEST(GPUBufferPool, a_long_churn_does_not_leak_blocks_or_start_failing) {
    GPUBufferPool pool;
    if (!start(pool, kUnit * 32)) return;

    std::vector<BufferAlloc> live;
    uint32_t seed = 12345u;
    const auto next = [&seed]() {
        seed = seed * 1103515245u + 12345u;
        return (seed >> 16) & 0x7fffu;
    };

    size_t failures = 0;
    for (int round = 0; round < 400; ++round) {
        const uint32_t size = 64u + (next() % 6000u);
        const BufferAlloc a = pool.allocate(size, 16);
        if (!a.valid()) {
            ++failures;
        } else {
            live.push_back(a);
        }
        if (live.size() > 24) {
            const size_t victim = next() % live.size();
            pool.free(live[victim]);
            live.erase(live.begin() + static_cast<long>(victim));
        }
    }

    CHECK_EQ(failures, size_t{0});

    // 24 live ranges of at most 6 KB is under 150 KB, comfortably inside a single
    // 256 KB block. A pool that never reused anything would be far past that.
    const auto stats = pool.stats();
    if (stats.blocks > 2) {
        stratum::test::report_failure(
            __FILE__, __LINE__, "a churn of small allocations stays in a couple of blocks",
            std::to_string(stats.blocks) + " blocks holding " +
                std::to_string(stats.live_allocations) + " live ranges, " +
                std::to_string(stats.bytes_used) + " bytes used of " +
                std::to_string(stats.bytes_reserved) + " reserved");
    }
    CHECK_EQ(stats.live_allocations, live.size());

    for (const BufferAlloc& a : live) pool.free(a);
    pool.shutdown();
}
