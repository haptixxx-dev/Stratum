/**
 * @file test_texture_upload.cpp
 * @brief Texture readiness, the built-in fallback texels, and what attaching a map does to base_color
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Three defects live here, and none of them shows up as a validation error or a
 * crash. They show up as a surface that is silently wrong for the rest of the
 * session, which is why they get a suite of their own.
 *
 * 1. **Readiness must be contingent on the submit.** flush_uploads() records
 *    copies into a copy pass on a command buffer it does not own and does not
 *    submit. It used to set `slot.ready = true`, erase the queue entries and free
 *    the staged pixels immediately -- so a failed SDL_SubmitGPUCommandBuffer left
 *    the textures flagged ready over uninitialised device memory with their pixels
 *    gone. is_ready() then returned true forever and bind_texture() stopped
 *    substituting the fallback. The mesh half of
 *    GPURenderer::flush_pending_uploads() has always rolled its flags back on that
 *    exact failure, which is the codebase itself asserting that the failure is
 *    survivable.
 *
 * 2. **The default ORM must be a multiplicative identity in all three channels.**
 *    Its blue was 0. mesh_pbr.frag computes `metallic = pbr_params.x * orm.b`, and
 *    every material in the library binds this one texture, so every authored
 *    metallic in the project was multiplied to zero.
 *
 * 3. **base_color must stop being the albedo once an albedo map exists.** The slot
 *    table authors base_color as the surface colour for the untextured case; the
 *    generators author their pixels as the surface colour too. Keeping both
 *    multiplied two full albedos together.
 *
 * ### Why this suite needs a device and MaterialLibrary's does not
 *
 * MaterialLibrary resolution is integers and floats over a hash map, so
 * test_material_library.cpp runs with an uninitialised manager and no device.
 * Everything here is about what actually reaches the GPU: real
 * SDL_CreateGPUTexture calls, a real copy pass, and install_procedural_textures(),
 * which cannot run without one. On a machine with no backend the suite prints a
 * SKIPPED line and asserts nothing, exactly as GPUBufferPool does.
 *
 * Run this suite with:
 * @code
 *     ./stratum_gpu_tests TextureUpload
 * @endcode
 */

#include "framework.hpp"

#include "renderer/material_library.hpp"
#include "renderer/texture.hpp"

#include <SDL3/SDL.h>

#include <cstdint>
#include <cstdio>
#include <vector>

namespace {

using stratum::GPUTextureManager;
using stratum::MaterialDef;
using stratum::MaterialId;
using stratum::MaterialKey;
using stratum::MaterialLibrary;
using stratum::TextureDesc;
using stratum::TextureFormat;
using stratum::TextureHandle;

/// The one device the suite shares, created on first use and destroyed at exit.
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

SDL_GPUDevice* device() {
    DeviceHolder& h = holder();
    if (h.attempted) return h.device;
    h.attempted = true;

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::fprintf(stderr, "[TextureUpload] SKIPPED: SDL video init failed: %s\n",
                     SDL_GetError());
        return nullptr;
    }
    h.video_ready = true;

    h.device = SDL_CreateGPUDevice(SDL_GPU_SHADERFORMAT_SPIRV, false, nullptr);
    if (h.device == nullptr) {
        std::fprintf(stderr, "[TextureUpload] SKIPPED: no SDL_GPU device: %s\n", SDL_GetError());
    }
    return h.device;
}

/// A manager brought up on the shared device, torn down by the destructor.
struct ManagerFixture {
    GPUTextureManager textures;
    bool ok = false;

    ManagerFixture() {
        SDL_GPUDevice* dev = device();
        if (dev != nullptr) ok = textures.init(dev);
    }
    ~ManagerFixture() {
        if (ok) SDL_WaitForGPUIdle(device());
        textures.shutdown();
    }
};

/// Stage a small opaque texture and return its handle. Never touches the queue
/// order, so a test can reason about which entry flushes first.
TextureHandle stage(GPUTextureManager& textures, uint32_t size, uint8_t fill) {
    TextureDesc desc{};
    desc.width = size;
    desc.height = size;
    desc.mip_levels = 1;
    desc.format = TextureFormat::RGBA8;
    std::vector<uint8_t> pixels(static_cast<size_t>(size) * size * 4u, fill);
    return textures.create(desc, pixels.data(), pixels.size());
}

/// Record the staged prefix into a real copy pass and return the command buffer,
/// which the caller either submits or cancels to model a failed submit.
SDL_GPUCommandBuffer* record_flush(GPUTextureManager& textures, uint64_t frame) {
    SDL_GPUCommandBuffer* cmd = SDL_AcquireGPUCommandBuffer(device());
    if (cmd == nullptr) return nullptr;
    SDL_GPUCopyPass* copy_pass = SDL_BeginGPUCopyPass(cmd);
    textures.flush_uploads(copy_pass, frame);
    SDL_EndGPUCopyPass(copy_pass);
    return cmd;
}

} // namespace

// ============================================================================
// The built-in fallback texels
// ============================================================================

/**
 * @brief The default ORM is a multiplicative identity, blue included
 *
 * This needs no device: it is a compile-time constant, and it is a constant
 * precisely so the invariant can be asserted rather than inspected. The shader
 * multiplies every one of the three channels into an authored scalar:
 *
 *     ao = pbr_params.z * orm.r    roughness = pbr_params.y * orm.g
 *     metallic = pbr_params.x * orm.b
 *
 * Blue was 0 -- the annihilator, not the identity -- so MaterialDef::metallic,
 * the panel's Metallic slider and a set file's "metallic" field were all no-ops
 * for every material, because every material binds this texture.
 */
TEST(TextureUpload, the_default_orm_texel_is_unit_in_every_channel) {
    CHECK_EQ(int{GPUTextureManager::kDefaultOrmTexel[0]}, 255);   // occlusion
    CHECK_EQ(int{GPUTextureManager::kDefaultOrmTexel[1]}, 255);   // roughness
    CHECK_EQ(int{GPUTextureManager::kDefaultOrmTexel[2]}, 255);   // metallic
    CHECK_EQ(int{GPUTextureManager::kDefaultOrmTexel[3]}, 255);
}

/// The fallbacks are uploaded synchronously by init() precisely so that every
/// later bind has something legal to substitute. If they were not ready, the
/// neutral bind GPURenderer uses when materials are off would have nothing to bind.
TEST(TextureUpload, init_leaves_every_builtin_ready_to_sample) {
    if (device() == nullptr) return;
    ManagerFixture fx;
    CHECK_TRUE(fx.ok);
    if (!fx.ok) return;

    CHECK_TRUE(fx.textures.is_ready(fx.textures.missing_texture()));
    CHECK_TRUE(fx.textures.is_ready(fx.textures.white()));
    CHECK_TRUE(fx.textures.is_ready(fx.textures.flat_normal()));
    CHECK_TRUE(fx.textures.is_ready(fx.textures.default_orm()));

    CHECK(fx.textures.get(fx.textures.white()) != nullptr);
    CHECK(fx.textures.get(fx.textures.flat_normal()) != nullptr);
    CHECK(fx.textures.get(fx.textures.default_orm()) != nullptr);
    CHECK_EQ(fx.textures.pending_upload_count(), size_t{0});
}

// ============================================================================
// Readiness is contingent on the submit
// ============================================================================

/**
 * @brief Recording a copy is not the same event as running it
 *
 * flush_uploads() has recorded the copy and the pixels are in a transfer buffer,
 * but the command buffer carrying it has not been submitted. Until it is, the
 * texture holds uninitialised device memory and must keep binding the fallback.
 */
TEST(TextureUpload, a_recorded_copy_is_not_ready_until_it_is_committed) {
    if (device() == nullptr) return;
    ManagerFixture fx;
    if (!fx.ok) return;

    const TextureHandle h = stage(fx.textures, 8, 0x40);
    CHECK(h != stratum::kInvalidTexture);
    CHECK_FALSE(fx.textures.is_ready(h));
    CHECK_EQ(fx.textures.pending_upload_count(), size_t{1});

    SDL_GPUCommandBuffer* cmd = record_flush(fx.textures, 1);
    CHECK(cmd != nullptr);
    if (cmd == nullptr) return;

    CHECK_FALSE(fx.textures.is_ready(h));

    CHECK_TRUE(SDL_SubmitGPUCommandBuffer(cmd));
    fx.textures.commit_uploads(true);

    CHECK_TRUE(fx.textures.is_ready(h));
    CHECK_EQ(fx.textures.pending_upload_count(), size_t{0});
}

/**
 * @brief The regression: a failed submit re-queues instead of lying
 *
 * SDL_CancelGPUCommandBuffer models the failure exactly -- the recorded copies are
 * thrown away without running -- and VULKAN_Submit has three real ways to return
 * false with a live device (EndCommandBuffer, an exhausted fence pool, and
 * vkQueueSubmit), so this is not a hypothetical.
 *
 * Before the fix the texture was flagged ready here, its queue entry was erased
 * and its staged bytes were freed, so the material sampled uninitialised device
 * memory for the rest of the session with no path back.
 */
TEST(TextureUpload, a_failed_submit_leaves_the_texture_unready_and_re_queued) {
    if (device() == nullptr) return;
    ManagerFixture fx;
    if (!fx.ok) return;

    const TextureHandle h = stage(fx.textures, 8, 0x7F);
    if (h == stratum::kInvalidTexture) return;

    SDL_GPUCommandBuffer* cmd = record_flush(fx.textures, 1);
    if (cmd == nullptr) return;

    SDL_CancelGPUCommandBuffer(cmd);
    fx.textures.commit_uploads(false);

    // Not ready, and still queued with its pixels intact.
    CHECK_FALSE(fx.textures.is_ready(h));
    CHECK_EQ(fx.textures.pending_upload_count(), size_t{1});

    // bind_texture() must therefore still substitute the fallback rather than
    // hand back the uninitialised texture.
    CHECK_EQ(fx.textures.bind_texture(h, fx.textures.white()),
             fx.textures.get(fx.textures.white()));

    // And the retry succeeds, which is the point of keeping the bytes.
    SDL_GPUCommandBuffer* retry = record_flush(fx.textures, 2);
    CHECK(retry != nullptr);
    if (retry == nullptr) return;
    CHECK_TRUE(SDL_SubmitGPUCommandBuffer(retry));
    fx.textures.commit_uploads(true);

    CHECK_TRUE(fx.textures.is_ready(h));
    CHECK_EQ(fx.textures.pending_upload_count(), size_t{0});
    CHECK_EQ(fx.textures.bind_texture(h, fx.textures.white()), fx.textures.get(h));
}

/// A queue with more than one entry keeps its ORDER across a rollback, or the
/// staging-arena offsets the retry reads from no longer name the right pixels.
TEST(TextureUpload, a_rollback_preserves_queue_order) {
    if (device() == nullptr) return;
    ManagerFixture fx;
    if (!fx.ok) return;

    const TextureHandle a = stage(fx.textures, 8, 0x11);
    const TextureHandle b = stage(fx.textures, 8, 0x22);
    const TextureHandle c = stage(fx.textures, 8, 0x33);
    if (a == stratum::kInvalidTexture || b == stratum::kInvalidTexture ||
        c == stratum::kInvalidTexture) {
        return;
    }
    CHECK_EQ(fx.textures.pending_upload_count(), size_t{3});

    SDL_GPUCommandBuffer* cmd = record_flush(fx.textures, 1);
    if (cmd == nullptr) return;
    SDL_CancelGPUCommandBuffer(cmd);
    fx.textures.commit_uploads(false);

    CHECK_EQ(fx.textures.pending_upload_count(), size_t{3});
    CHECK_FALSE(fx.textures.is_ready(a));
    CHECK_FALSE(fx.textures.is_ready(b));
    CHECK_FALSE(fx.textures.is_ready(c));

    SDL_GPUCommandBuffer* retry = record_flush(fx.textures, 2);
    if (retry == nullptr) return;
    CHECK_TRUE(SDL_SubmitGPUCommandBuffer(retry));
    fx.textures.commit_uploads(true);

    CHECK_TRUE(fx.textures.is_ready(a));
    CHECK_TRUE(fx.textures.is_ready(b));
    CHECK_TRUE(fx.textures.is_ready(c));
}

// ============================================================================
// What attaching a generated map does to base_color
// ============================================================================

/**
 * @brief base_color becomes a tint the moment a real albedo map replaces white
 *
 * mesh_pbr.frag computes `albedo = frag_color.rgb * base_color.rgb * albedo_tex.rgb`
 * and road geometry carries vertex colour (1, 1, 1), so nothing else lifts the
 * product. Leaving Asphalt's authored (0.11, 0.11, 0.12) in place beside a
 * generated asphalt map authored at ~0.045 linear gave 0.005 -- about a ninth of
 * either factor -- and every other textured slot was darkened by the same squaring.
 *
 * The suite runs the generators at 64 rather than the 512 the editor uses: this is
 * about which field ends up where, not about how the pixels look, and 64 keeps the
 * run under a second.
 */
TEST(TextureUpload, attaching_a_generated_albedo_neutralises_base_color) {
    if (device() == nullptr) return;
    ManagerFixture fx;
    if (!fx.ok) return;

    MaterialLibrary lib;
    CHECK_TRUE(lib.init(&fx.textures));
    lib.load_defaults();

    // The authored untextured colour, which is correct while the 1x1 white
    // fallback is bound and is exactly what must NOT survive the attach.
    const MaterialDef before = lib.resolve(MaterialKey{ MaterialId::Asphalt, 0 });
    CHECK(before.base_color.r < 0.5f);
    CHECK_EQ(before.albedo, fx.textures.white());

    const bool generated = lib.install_procedural_textures(64u, 1234u);
    CHECK_TRUE(generated);
    if (!generated) {
        lib.shutdown();
        return;
    }

    const MaterialDef after = lib.resolve(MaterialKey{ MaterialId::Asphalt, 0 });
    CHECK(after.albedo != fx.textures.white());
    CHECK(after.albedo != stratum::kInvalidTexture);
    CHECK_EQ(after.base_color.r, 1.0f);
    CHECK_EQ(after.base_color.g, 1.0f);
    CHECK_EQ(after.base_color.b, 1.0f);
    CHECK_EQ(after.base_color.a, before.base_color.a);   // alpha is coverage, not colour

    // Every slot that got a generated map, not just the one. A slot still bound to
    // the white fallback keeps its authored colour, because for it base_color IS
    // still the albedo.
    for (const MaterialKey key : lib.keys()) {
        const MaterialDef& def = lib.resolve(key);
        if (def.albedo == fx.textures.white() || def.albedo == stratum::kInvalidTexture) {
            continue;
        }
        if (def.base_color.r != 1.0f || def.base_color.g != 1.0f || def.base_color.b != 1.0f) {
            ::stratum::test::report_failure(
                __FILE__, __LINE__,
                "a textured material's base_color is a neutral tint",
                def.name + " keeps a full albedo in base_color beside its albedo map");
        }
    }

    lib.shutdown();
}

/// Every material still binds the built-in ORM, which is the reason the unit-ORM
/// invariant above is load-bearing rather than theoretical.
TEST(TextureUpload, every_installed_material_binds_the_builtin_orm) {
    if (device() == nullptr) return;
    ManagerFixture fx;
    if (!fx.ok) return;

    MaterialLibrary lib;
    if (!lib.init(&fx.textures)) return;
    lib.load_defaults();
    if (!lib.install_procedural_textures(64u, 7u)) {
        lib.shutdown();
        return;
    }

    for (const MaterialKey key : lib.keys()) {
        CHECK_EQ(lib.resolve(key).orm, fx.textures.default_orm());
    }

    lib.shutdown();
}
