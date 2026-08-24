/**
 * @file test_procedural_texture.cpp
 * @brief The generated surface textures: tiling, determinism, layout, and the markings atlas
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * The generators are pure: no device, no SDL, no file I/O, no global state. They
 * take a size, a seed and a parameter and return a pixel buffer. That is the
 * whole reason this suite exists rather than a screenshot comparison -- every
 * property that matters about a tiling texture is a property of its bytes.
 *
 * ### The property this suite is really for
 *
 * Tileability. The plan's UV Convention divides world metres by a per-material
 * tile size, so ONE texture repeats across an entire road network -- an 8 m
 * asphalt tile repeats roughly every 8 m of carriageway, in both axes, forever. A
 * generator with a discontinuity at its wrap does not produce one visible artefact;
 * it produces a grid of them across the whole map, and it looks like a mesh bug
 * rather than a texture bug, which is where the afternoon goes.
 *
 * ### How the seam is measured, and why not by comparing the edge columns
 *
 * The natural-sounding test -- "column 0 equals column N-1" -- is wrong, and would
 * fail every correct generator here. procedural_texture.hpp specifies PERIODIC
 * noise: the field has period N, so f(0) == f(N), and column N-1 is one texel
 * BEFORE the repeat, not equal to it. Two adjacent columns of a noise field are
 * never equal. Only a mirrored or crossfaded texture has equal edge columns, and
 * the header explicitly rules that construction out.
 *
 * What must hold instead is CONTINUITY across the wrap: the step from column N-1
 * to column 0 must be an ordinary neighbouring-column step, not a jump. So the
 * seam step is compared against the steps the texture takes everywhere else, with
 * no magic threshold: it must be no worse than the largest interior step the
 * texture already contains. A generator that forgot to wrap its lattice produces
 * two uncorrelated columns at the seam, which for smooth fields exceeds the
 * interior maximum by an order of magnitude and is caught immediately.
 *
 * Run this suite with:
 * @code
 *     ./stratum_gpu_tests ProceduralTexture
 * @endcode
 */

#include "framework.hpp"

#include "osm/road/marking_atlas.hpp"
#include "renderer/procedural_texture.hpp"
#include "renderer/texture.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace {

using stratum::ProcTexResult;
using stratum::TextureDesc;
using stratum::TextureFormat;

namespace atlas = stratum::osm::road;

/// Side used for the surface generators. Small enough to keep the suite instant,
/// large enough that a lattice-wrap bug has somewhere to show.
constexpr uint32_t kSize = 128;

/// Bytes per texel of an uncompressed format.
[[nodiscard]] uint32_t channels(TextureFormat format) {
    switch (format) {
        case TextureFormat::RGBA8:
        case TextureFormat::RGBA8_SRGB: return 4;
        case TextureFormat::R8:         return 1;
        case TextureFormat::BC7:
        case TextureFormat::BC7_SRGB:
        case TextureFormat::BC5_Normal: return 0;
    }
    return 0;
}

/// One channel of one texel. No bounds checking: every caller derives x and y
/// from the desc it just validated.
[[nodiscard]] uint8_t texel(const ProcTexResult& t, uint32_t x, uint32_t y, uint32_t c) {
    const uint32_t n = channels(t.desc.format);
    return t.pixels[(static_cast<size_t>(y) * t.desc.width + x) * n + c];
}

/// One generator, named so a failure says which.
struct Generator {
    const char* name;
    ProcTexResult (*make)(uint32_t size, uint32_t seed);
    bool tiles_in_u;    ///< false for the kerb, whose two halves are face and top
};

ProcTexResult gen_asphalt_smooth(uint32_t size, uint32_t seed) {
    return stratum::make_asphalt(size, seed, 0.15f);
}
ProcTexResult gen_asphalt_worn(uint32_t size, uint32_t seed) {
    return stratum::make_asphalt(size, seed, 0.85f);
}
ProcTexResult gen_concrete(uint32_t size, uint32_t seed) {
    return stratum::make_concrete(size, seed);
}
ProcTexResult gen_cobblestone(uint32_t size, uint32_t seed) {
    return stratum::make_cobblestone(size, seed, 0.08f);
}
ProcTexResult gen_paving(uint32_t size, uint32_t seed) {
    return stratum::make_paving(size, seed, glm::vec2(0.5f, 0.25f));
}
ProcTexResult gen_gravel(uint32_t size, uint32_t seed) {
    return stratum::make_gravel(size, seed);
}
ProcTexResult gen_dirt(uint32_t size, uint32_t seed) {
    return stratum::make_dirt(size, seed);
}
ProcTexResult gen_grass(uint32_t size, uint32_t seed) {
    return stratum::make_grass(size, seed);
}
ProcTexResult gen_kerb(uint32_t size, uint32_t seed) {
    return stratum::make_kerb(size, seed);
}

/**
 * @brief Every surface generator, uniformly callable
 *
 * make_asphalt appears twice because its coarseness parameter is the axis the
 * surface= variants differ along, and a wrap bug that only bites at one end of
 * that axis is exactly the kind that ships.
 */
const Generator kGenerators[] = {
    {"asphalt(smooth)", gen_asphalt_smooth, true},
    {"asphalt(worn)",   gen_asphalt_worn,   true},
    {"concrete",        gen_concrete,       true},
    {"cobblestone",     gen_cobblestone,    true},
    {"paving",          gen_paving,         true},
    {"gravel",          gen_gravel,         true},
    {"dirt",            gen_dirt,           true},
    {"grass",           gen_grass,          true},
    // make_kerb packs the vertical FACE into the left half of U and the TOP into
    // the right half, so it tiles in V only. Nothing tiles a kerb laterally.
    {"kerb",            gen_kerb,           false},
};

void fail(const char* expr, const std::string& detail, int line) {
    ::stratum::test::report_failure(__FILE__, line, expr, detail);
}

/// Mean absolute difference, over every channel, between two columns.
[[nodiscard]] double column_step(const ProcTexResult& t, uint32_t xa, uint32_t xb) {
    const uint32_t n = channels(t.desc.format);
    double sum = 0.0;
    for (uint32_t y = 0; y < t.desc.height; ++y) {
        for (uint32_t c = 0; c < n; ++c) {
            sum += std::fabs(static_cast<double>(texel(t, xa, y, c)) -
                             static_cast<double>(texel(t, xb, y, c)));
        }
    }
    return sum / (static_cast<double>(t.desc.height) * n);
}

/// Mean absolute difference, over every channel, between two rows.
[[nodiscard]] double row_step(const ProcTexResult& t, uint32_t ya, uint32_t yb) {
    const uint32_t n = channels(t.desc.format);
    double sum = 0.0;
    for (uint32_t x = 0; x < t.desc.width; ++x) {
        for (uint32_t c = 0; c < n; ++c) {
            sum += std::fabs(static_cast<double>(texel(t, x, ya, c)) -
                             static_cast<double>(texel(t, x, yb, c)));
        }
    }
    return sum / (static_cast<double>(t.desc.width) * n);
}

/**
 * @brief The wrap step must be an ordinary step, in one axis
 *
 * See the file header for why this is a continuity test and not an equality test.
 * The bar is the largest step the texture already takes between neighbouring
 * lines, with a small allowance so a seam that happens to be the single worst
 * adjacency out of N does not fail a correct generator. A generator that failed
 * to wrap clears that bar by a wide margin, because its two seam lines are
 * statistically independent while every interior pair is correlated.
 *
 * @param name  Generator name, for the failure message
 * @param t     Texture to check
 * @param axis  'u' for the column wrap, 'v' for the row wrap
 * @param line  __LINE__ of the caller
 */
void check_wraps(const char* name, const ProcTexResult& t, char axis, int line) {
    const uint32_t n = (axis == 'u') ? t.desc.width : t.desc.height;
    if (n < 4) {
        return;
    }

    double interior_max = 0.0;
    double interior_sum = 0.0;
    for (uint32_t i = 0; i + 1 < n; ++i) {
        const double step = (axis == 'u') ? column_step(t, i, i + 1) : row_step(t, i, i + 1);
        interior_max = std::max(interior_max, step);
        interior_sum += step;
    }
    const double interior_mean = interior_sum / static_cast<double>(n - 1);

    const double seam = (axis == 'u') ? column_step(t, n - 1, 0) : row_step(t, n - 1, 0);

    // Either bar passing is enough. interior_max is the tight one for smooth
    // fields; interior_mean * 4 gives a high-contrast field such as paving, whose
    // interior maximum sits on a joint line, a sane bar of its own.
    const double bar = std::max(interior_max * 1.25, interior_mean * 4.0) + 0.5;

    if (!(seam <= bar)) {
        fail("wrap step is an ordinary neighbouring step",
             std::string{name} + ": axis " + axis + " seam step " + std::to_string(seam) +
                 " exceeds " + std::to_string(bar) + " (interior max " +
                 std::to_string(interior_max) + ", mean " + std::to_string(interior_mean) +
                 ") -- the lattice is not wrapping modulo the texture size",
             line);
    }
}

/// Population variance of one channel, as a proxy for "the generator generated".
[[nodiscard]] double channel_variance(const ProcTexResult& t, uint32_t c) {
    const uint32_t n = channels(t.desc.format);
    if (c >= n) {
        return 0.0;
    }
    const size_t count = static_cast<size_t>(t.desc.width) * t.desc.height;
    double sum = 0.0;
    for (uint32_t y = 0; y < t.desc.height; ++y) {
        for (uint32_t x = 0; x < t.desc.width; ++x) {
            sum += texel(t, x, y, c);
        }
    }
    const double mean = sum / static_cast<double>(count);
    double var = 0.0;
    for (uint32_t y = 0; y < t.desc.height; ++y) {
        for (uint32_t x = 0; x < t.desc.width; ++x) {
            const double d = texel(t, x, y, c) - mean;
            var += d * d;
        }
    }
    return var / static_cast<double>(count);
}

/// A flat RGBA8 height field: constant alpha, everything else zero.
[[nodiscard]] ProcTexResult flat_height(uint32_t size, uint8_t height) {
    ProcTexResult t;
    t.desc.width = size;
    t.desc.height = size;
    t.desc.mip_levels = 1;
    t.desc.layers = 1;
    t.desc.format = TextureFormat::RGBA8;
    t.pixels.assign(t.desc.level0_bytes(), 0);
    for (size_t i = 3; i < t.pixels.size(); i += 4) {
        t.pixels[i] = height;
    }
    return t;
}

// --- markings atlas helpers ------------------------------------------------

/// The un-inset BLOCK a sprite is painted into, in pixels, recovered from
/// sprite_rect() exactly as procedural_texture.hpp specifies. Never transcribed
/// from the layout comment: sprite_rect() is the authority both sides use.
struct Block {
    int x0, y0, x1, y1;   ///< Half-open [x0, x1) x [y0, y1)
};

[[nodiscard]] Block block_of(atlas::MarkingSprite s, uint32_t size) {
    const float px = static_cast<float>(size) / static_cast<float>(atlas::kAtlasSizePixels);
    const atlas::SpriteRect r = atlas::sprite_rect(s);
    const float inset = atlas::kAtlasInsetPixels * px;
    return Block{
        static_cast<int>(std::lround(r.u0 * static_cast<float>(size) - inset)),
        static_cast<int>(std::lround(r.v0 * static_cast<float>(size) - inset)),
        static_cast<int>(std::lround(r.u1 * static_cast<float>(size) + inset)),
        static_cast<int>(std::lround(r.v1 * static_cast<float>(size) + inset)),
    };
}

/// The INSET rect the emitter actually samples, in pixels.
[[nodiscard]] Block inset_of(atlas::MarkingSprite s, uint32_t size) {
    const atlas::SpriteRect r = atlas::sprite_rect(s);
    return Block{
        static_cast<int>(std::lround(r.u0 * static_cast<float>(size))),
        static_cast<int>(std::lround(r.v0 * static_cast<float>(size))),
        static_cast<int>(std::lround(r.u1 * static_cast<float>(size))),
        static_cast<int>(std::lround(r.v1 * static_cast<float>(size))),
    };
}

constexpr uint8_t kSpriteCount = static_cast<uint8_t>(atlas::MarkingSprite::Count);

/// Sprites drawn as pictograms, which cannot reach the left or right edge of
/// their block. The continuous sprites -- solid lines, stop lines, zebra stripes,
/// box hatching, dashes -- are painted to their block edge on purpose so the
/// inset rect lands strictly inside the paint, so they are excluded here.
const atlas::MarkingSprite kPictograms[] = {
    atlas::MarkingSprite::ArrowStraight,      atlas::MarkingSprite::ArrowLeft,
    atlas::MarkingSprite::ArrowRight,         atlas::MarkingSprite::ArrowStraightLeft,
    atlas::MarkingSprite::ArrowStraightRight, atlas::MarkingSprite::ArrowUTurn,
    atlas::MarkingSprite::BikeSymbol,         atlas::MarkingSprite::BusSymbol,
};

} // namespace

// ============================================================================
// Description and buffer agreement
// ============================================================================

/**
 * @brief The buffer is exactly the size the description implies
 *
 * A desc that disagrees with its buffer is not a cosmetic bug: create() sizes an
 * SDL transfer buffer from the desc and copies from the vector, so a short buffer
 * is a heap over-read into a GPU upload and a long one silently truncates.
 */
TEST(ProceduralTexture, every_generator_returns_a_buffer_matching_its_description) {
    for (const Generator& g : kGenerators) {
        const ProcTexResult t = g.make(kSize, 4242);

        if (!t.is_valid()) {
            fail("generator returns a valid result", std::string{g.name} + " produced nothing",
                 __LINE__);
            continue;
        }

        CHECK_EQ(t.desc.width, kSize);
        CHECK_EQ(t.desc.height, kSize);
        CHECK(t.desc.layers == 1u);
        CHECK(t.desc.mip_levels >= 1u);

        // Albedo is colour and must be sampled through an sRGB view.
        if (t.desc.format != TextureFormat::RGBA8_SRGB) {
            fail("surface albedo is RGBA8_SRGB",
                 std::string{g.name} + " is format " +
                     std::to_string(static_cast<int>(t.desc.format)),
                 __LINE__);
        }

        CHECK_EQ(t.pixels.size(), t.desc.level0_bytes());
        CHECK_EQ(t.pixels.size(), size_t{kSize} * kSize * 4u);
    }
}

/**
 * @brief feature_period() never inverts its bounds
 *
 * The generators cap their lattice and grain periods at some fraction of the
 * texture size -- one aggregate cell per four texels, one paving cell per eight.
 * Spelled the obvious way,
 * `std::clamp(desired, min_period, int(size / divisor))`, that cap is UNDEFINED
 * BEHAVIOUR at the smallest sizes three separate places declare valid: the header
 * says "power of two, >= 4", valid_surface_size() accepts 4, and
 * MaterialLibrary::install_procedural_textures() validates against a floor of 4 --
 * but at size 4 with divisor 4 the upper bound is 1, below the lower bound of 4.
 *
 * std::clamp's precondition is `!(hi < lo)`. Violating it aborts on a hardened
 * libstdc++ and, in an unhardened build like this project's, silently returns the
 * INVERTED bound, collapsing a whole noise layer to period 1 -- a constant -- with
 * no diagnostic. Six generators had it.
 *
 * The assertion is that the lower bound wins, which is the only answer that keeps
 * the feature a feature.
 */
TEST(ProceduralTexture, a_feature_period_never_falls_below_its_minimum) {
    using stratum::feature_period;

    // The exact expressions the generators use, at the smallest declared sizes.
    CHECK_EQ(feature_period(96, 4, 4u, 4u), 4);        // asphalt aggregate
    CHECK_EQ(feature_period(96, 4, 8u, 4u), 4);
    CHECK_EQ(feature_period(160, 4, 4u, 4u), 4);       // concrete tooth
    CHECK_EQ(feature_period(24, 2, 4u, 8u), 2);        // cobblestone cells
    CHECK_EQ(feature_period(288, 4, 4u, 4u), 4);       // cobblestone grain
    CHECK_EQ(feature_period(192, 4, 4u, 4u), 4);       // paving tooth
    CHECK_EQ(feature_period(128, 4, 4u, 4u), 4);       // dirt grain
    CHECK_EQ(feature_period(176, 4, 4u, 4u), 4);       // kerb tooth

    // At a size the cap actually binds, the cap is what is returned.
    CHECK_EQ(feature_period(96, 4, 128u, 4u), 32);
    CHECK_EQ(feature_period(96, 4, 512u, 4u), 96);     // desired, under the cap
    CHECK_EQ(feature_period(2, 4, 512u, 4u), 4);       // desired, under the floor

    // A zero divisor must not divide, rather than trapping.
    CHECK_EQ(feature_period(96, 4, 512u, 0u), 4);
}

/**
 * @brief Every generator runs at the smallest size its own contract declares valid
 *
 * The sweeps elsewhere in this suite use 64, 128 and 256, so they never reach the
 * branch above. Sizes 4 and 8 are the ones three validators bless and the
 * arithmetic could not survive.
 */
TEST(ProceduralTexture, the_smallest_declared_sizes_generate_a_valid_texture) {
    for (const Generator& g : kGenerators) {
        for (const uint32_t size : {4u, 8u, 16u}) {
            const ProcTexResult t = g.make(size, 5);
            if (!t.is_valid()) {
                fail("the smallest declared size generates",
                     std::string{g.name} + " failed at size " + std::to_string(size), __LINE__);
                continue;
            }
            CHECK_EQ(t.desc.width, size);
            CHECK_EQ(t.desc.height, size);
            CHECK_EQ(t.pixels.size(), size_t{size} * size * 4u);
        }
    }
}

/// A bad size returns an empty result rather than guessing a nearby power of two.
TEST(ProceduralTexture, invalid_sizes_return_an_empty_result) {
    for (const Generator& g : kGenerators) {
        for (const uint32_t bad : {0u, 1u, 2u, 3u, 100u, 129u, 1000u}) {
            const ProcTexResult t = g.make(bad, 1);
            if (t.is_valid()) {
                fail("invalid size is refused",
                     std::string{g.name} + " accepted size " + std::to_string(bad), __LINE__);
            }
            CHECK_TRUE(t.pixels.empty());
        }
    }
}

// ============================================================================
// Determinism
// ============================================================================

/// Byte-identical for the same inputs. Anything less makes a golden test
/// impossible and makes two machines disagree about what a road looks like.
TEST(ProceduralTexture, the_same_size_and_seed_produce_byte_identical_pixels) {
    for (const Generator& g : kGenerators) {
        const ProcTexResult a = g.make(kSize, 20260823);
        const ProcTexResult b = g.make(kSize, 20260823);

        CHECK_TRUE(a.is_valid());
        CHECK_EQ(a.pixels.size(), b.pixels.size());
        if (a.pixels.size() != b.pixels.size()) {
            continue;
        }
        if (a.pixels != b.pixels) {
            size_t first = 0;
            while (first < a.pixels.size() && a.pixels[first] == b.pixels[first]) {
                ++first;
            }
            fail("two runs agree byte for byte",
                 std::string{g.name} + " differs first at byte " + std::to_string(first), __LINE__);
        }
    }
}

/// And different seeds must actually differ, or the seed is decorative and every
/// road in the map shares one noise field.
TEST(ProceduralTexture, different_seeds_produce_different_pixels) {
    for (const Generator& g : kGenerators) {
        const ProcTexResult a = g.make(kSize, 1);
        const ProcTexResult b = g.make(kSize, 2);

        CHECK_TRUE(a.is_valid());
        CHECK_TRUE(b.is_valid());
        if (a.pixels == b.pixels) {
            fail("the seed changes the output", std::string{g.name} + " ignores its seed",
                 __LINE__);
        }
    }
}

// ============================================================================
// Tileability
// ============================================================================

/// The property the whole UV Convention rests on. See the file header for why
/// this is a continuity test rather than an edge-column equality test.
TEST(ProceduralTexture, every_generator_tiles_without_a_seam) {
    for (const Generator& g : kGenerators) {
        const ProcTexResult t = g.make(kSize, 77);
        if (!t.is_valid()) {
            fail("generator returns a valid result", g.name, __LINE__);
            continue;
        }

        // V always: arc length along a road runs forever, so the row wrap is the
        // one every road in the network crosses.
        check_wraps(g.name, t, 'v', __LINE__);

        if (g.tiles_in_u) {
            check_wraps(g.name, t, 'u', __LINE__);
        }
    }
}

/**
 * @brief The seam detector can actually FAIL -- the negative control
 *
 * every_generator_tiles_without_a_seam() and tiling_holds_across_texture_sizes()
 * both assert that is_tileable() says yes. If is_tileable() ever said yes
 * unconditionally -- a short-circuit, an inverted comparison, an early return on
 * a format it did not recognise -- both suites would still pass, in full, while
 * proving nothing whatsoever about the textures. Tileability is one of the two
 * properties this whole material phase rests on, so the detector is tested in the
 * direction that can expose that.
 *
 * The injected defect is a linear brightness ramp, which is the shape a
 * NON-PERIODIC layer actually produces: values climb steadily across the axis, so
 * the wrap from the last line back to the first steps by the full amplitude while
 * every interior step is amplitude/size. That is the real failure mode -- a
 * generator that hashes unreduced lattice coordinates, or clamps a gradient --
 * rather than an artificial discontinuity no bug would create.
 *
 * The amplitude is measured, not picked. Sweeping every generator, both axes and
 * every size this suite uses gives a clear floor: a noise surface like asphalt is
 * caught from 4-8/255, while a LATTICE surface needs far more, because its own
 * mortar joints legitimately make the wrap pair several times the texture average
 * and the detector's local reference is required to forgive exactly that (the
 * is_tileable() header documents this two-sided reference and states plainly that
 * it is a mistake-guard, not a certificate of seamlessness). The binding case is
 * paving at 64 px, whose V axis carries only four slabs, so its joints dominate:
 * it still reports tileable at 128/255 and is caught at 160/255. 160 is therefore
 * the matrix-wide floor, and this test uses 176 for margin so that a future tweak
 * to a generator's contrast does not make it flap.
 *
 * That floor is a real limit worth knowing rather than a defect to route around:
 * it says a seam smaller than ~160/255 in a small lattice texture will not be
 * caught by this detector, which is precisely why the generators prove periodicity
 * by construction (integer modulo on lattice coordinates before hashing) instead
 * of relying on this measurement.
 */
TEST(ProceduralTexture, the_seam_detector_rejects_an_injected_ramp) {
    // Amplitude, in 0-255 units, added across the axis under test. See the note
    // above: 160 is the measured matrix-wide floor, 176 is that plus margin.
    constexpr int kRampAmplitude = 176;

    for (const uint32_t size : {64u, 256u, kSize}) {
        for (const Generator& g : kGenerators) {
            const ProcTexResult clean = g.make(size, 77);
            if (!clean.is_valid()) {
                fail("generator returns a valid result", g.name, __LINE__);
                continue;
            }

            // Sanity: the unmodified texture must pass, or the ramp below proves
            // nothing about the ramp.
            if (!stratum::is_tileable(clean, g.tiles_in_u, true)) {
                fail("clean texture tiles", g.name, __LINE__);
                continue;
            }

            for (const char axis : {'u', 'v'}) {
                if (axis == 'u' && !g.tiles_in_u) {
                    continue;   // make_kerb: face|top by design, U seam meaningless
                }

                ProcTexResult ramped = clean;
                const uint32_t n = ramped.desc.width;
                const uint32_t ch = channels(ramped.desc.format);
                for (uint32_t y = 0; y < n; ++y) {
                    for (uint32_t x = 0; x < n; ++x) {
                        const double f =
                            static_cast<double>(axis == 'u' ? x : y) / (n - 1);
                        const int step = static_cast<int>(f * kRampAmplitude);
                        // Colour channels only: alpha carries the height field,
                        // and ramping it would test a different thing.
                        for (uint32_t c = 0; c < (ch >= 3 ? 3u : ch); ++c) {
                            uint8_t& p = ramped.pixels[(static_cast<size_t>(y) * n + x) * ch + c];
                            p = static_cast<uint8_t>(std::min(255, p + step));
                        }
                    }
                }

                const bool u = (axis == 'u');
                if (stratum::is_tileable(ramped, u, !u)) {
                    fail("seam detector rejects an injected ramp",
                         std::string{g.name} + ": axis " + axis + " at size " +
                             std::to_string(size) + " with a " +
                             std::to_string(kRampAmplitude) +
                             "/255 ramp was still reported tileable",
                         __LINE__);
                }
            }
        }
    }
}

/// Tileability must hold at every size the material system might ask for, not
/// only at the one the rest of the suite uses.
TEST(ProceduralTexture, tiling_holds_across_texture_sizes) {
    for (const uint32_t size : {32u, 64u, 256u}) {
        for (const Generator& g : kGenerators) {
            const ProcTexResult t = g.make(size, 9);
            if (!t.is_valid()) {
                fail("generator returns a valid result",
                     std::string{g.name} + " at size " + std::to_string(size), __LINE__);
                continue;
            }
            const std::string label = std::string{g.name} + "@" + std::to_string(size);
            check_wraps(label.c_str(), t, 'v', __LINE__);
            if (g.tiles_in_u) {
                check_wraps(label.c_str(), t, 'u', __LINE__);
            }
        }
    }
}

// ============================================================================
// Content sanity
// ============================================================================

/**
 * @brief Nothing degenerate comes out
 *
 * The outputs are bytes, so a NaN cannot survive into the buffer -- but the float
 * maths behind them can still produce one, and a NaN cast to uint8_t is
 * implementation-defined rather than an obvious crash. A collapsed generator
 * shows up as a texture with no variation at all, and a generator that saturated
 * shows up as one pinned at 0 or 255, so both are checked directly.
 */
TEST(ProceduralTexture, no_generator_produces_a_flat_or_saturated_texture) {
    for (const Generator& g : kGenerators) {
        const ProcTexResult t = g.make(kSize, 31337);
        if (!t.is_valid()) {
            continue;
        }

        for (uint32_t c = 0; c < 3; ++c) {
            if (channel_variance(t, c) <= 0.0) {
                fail("albedo has variation",
                     std::string{g.name} + " channel " + std::to_string(c) + " is constant",
                     __LINE__);
            }
        }

        uint8_t lo = 255;
        uint8_t hi = 0;
        for (uint32_t y = 0; y < t.desc.height; ++y) {
            for (uint32_t x = 0; x < t.desc.width; ++x) {
                for (uint32_t c = 0; c < 3; ++c) {
                    lo = std::min(lo, texel(t, x, y, c));
                    hi = std::max(hi, texel(t, x, y, c));
                }
            }
        }
        if (lo == hi) {
            fail("albedo is not a single value", g.name, __LINE__);
        }
    }
}

/**
 * @brief Albedo carries the height field in alpha
 *
 * The documented contract, and the reason a caller gets a matching albedo and
 * normal from one generation pass. A constant alpha means make_normal_from_height
 * would derive a perfectly flat normal map from it and every surface would be
 * mirror-flat with no error anywhere to point at.
 */
TEST(ProceduralTexture, surface_albedo_carries_a_height_field_in_alpha) {
    for (const Generator& g : kGenerators) {
        const ProcTexResult t = g.make(kSize, 5150);
        if (!t.is_valid()) {
            continue;
        }
        if (channel_variance(t, 3) <= 0.0) {
            fail("alpha carries height",
                 std::string{g.name} + " has a constant alpha channel; nothing to make a "
                                       "normal map from",
                 __LINE__);
        }
    }
}

// ============================================================================
// make_normal_from_height
// ============================================================================

/**
 * @brief A flat height field decodes to no perturbation at all, exactly
 *
 * (128, 128, 255) is the encoding of (0, 0, 1). It has to be exact rather than
 * approximate: an off-by-one here tilts every unmapped surface by a fraction of a
 * degree in one consistent direction, which reads as a lighting bug in the scene
 * and never as a texture bug.
 */
TEST(ProceduralTexture, a_flat_height_field_yields_the_exact_flat_normal) {
    for (const uint8_t height : {uint8_t{0}, uint8_t{128}, uint8_t{255}}) {
        const ProcTexResult flat = flat_height(64, height);
        const ProcTexResult n = stratum::make_normal_from_height(flat, 1.0f);

        CHECK_TRUE(n.is_valid());
        if (!n.is_valid()) {
            continue;
        }

        // Normals are DATA. An sRGB normal map is wrong in a way that looks
        // merely "a bit off".
        CHECK_EQ(static_cast<int>(n.desc.format), static_cast<int>(TextureFormat::RGBA8));
        CHECK_EQ(n.desc.width, flat.desc.width);
        CHECK_EQ(n.desc.height, flat.desc.height);
        CHECK_EQ(n.pixels.size(), n.desc.level0_bytes());

        size_t wrong = 0;
        for (uint32_t y = 0; y < n.desc.height; ++y) {
            for (uint32_t x = 0; x < n.desc.width; ++x) {
                if (texel(n, x, y, 0) != 128 || texel(n, x, y, 1) != 128 ||
                    texel(n, x, y, 2) != 255 || texel(n, x, y, 3) != 255) {
                    ++wrong;
                }
            }
        }
        if (wrong != 0) {
            fail("a flat height field gives exactly (128,128,255,255)",
                 "height " + std::to_string(height) + ": " + std::to_string(wrong) +
                     " texels of " + std::to_string(n.desc.width * n.desc.height) + " differ",
                 __LINE__);
        }
    }
}

/// strength 0, and any negative strength, must also flatten completely.
TEST(ProceduralTexture, zero_strength_flattens_a_real_height_field) {
    const ProcTexResult source = stratum::make_gravel(64, 12);
    for (const float strength : {0.0f, -1.0f, -100.0f}) {
        const ProcTexResult n = stratum::make_normal_from_height(source, strength);
        CHECK_TRUE(n.is_valid());
        if (!n.is_valid()) {
            continue;
        }
        for (uint32_t y = 0; y < n.desc.height; ++y) {
            for (uint32_t x = 0; x < n.desc.width; ++x) {
                CHECK_EQ(static_cast<int>(texel(n, x, y, 0)), 128);
                CHECK_EQ(static_cast<int>(texel(n, x, y, 1)), 128);
                CHECK_EQ(static_cast<int>(texel(n, x, y, 2)), 255);
            }
        }
    }
}

/**
 * @brief Decoded normals are unit length and face outward
 *
 * The float proxy for "no NaN reached the buffer". A NaN or an unnormalised
 * gradient decodes to a vector that is not unit length, and a sign error decodes
 * to one pointing into the surface -- both invisible in the raw bytes and both
 * obvious here.
 */
TEST(ProceduralTexture, generated_normals_are_unit_length_and_outward_facing) {
    for (const Generator& g : kGenerators) {
        const ProcTexResult source = g.make(kSize, 606);
        const ProcTexResult n = stratum::make_normal_from_height(source, 1.0f);
        if (!n.is_valid()) {
            fail("normal generation succeeds", g.name, __LINE__);
            continue;
        }

        double worst_len = 1.0;
        bool inward = false;
        for (uint32_t y = 0; y < n.desc.height; ++y) {
            for (uint32_t x = 0; x < n.desc.width; ++x) {
                const double nx = texel(n, x, y, 0) / 255.0 * 2.0 - 1.0;
                const double ny = texel(n, x, y, 1) / 255.0 * 2.0 - 1.0;
                const double nz = texel(n, x, y, 2) / 255.0 * 2.0 - 1.0;
                const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
                if (std::fabs(len - 1.0) > std::fabs(worst_len - 1.0)) {
                    worst_len = len;
                }
                if (nz <= 0.0) {
                    inward = true;
                }
            }
        }
        // 1/255 of quantisation per component, plus room for the encode rounding.
        if (std::fabs(worst_len - 1.0) > 0.02) {
            fail("decoded normal is unit length",
                 std::string{g.name} + " worst length " + std::to_string(worst_len), __LINE__);
        }
        if (inward) {
            fail("decoded normal points out of the surface",
                 std::string{g.name} + " has a texel with z <= 0", __LINE__);
        }
    }
}

/**
 * @brief The normal map's own gradient wraps
 *
 * A Sobel that clamps at the border instead of wrapping leaves a one-texel flat
 * frame around every tile. Flat framing is not subtle once tiled: it draws a hard
 * grid over the entire road network at exactly the tile spacing.
 */
TEST(ProceduralTexture, the_derived_normal_map_tiles_without_a_seam) {
    for (const Generator& g : kGenerators) {
        const ProcTexResult source = g.make(kSize, 808);
        const ProcTexResult n = stratum::make_normal_from_height(source, 1.0f);
        if (!n.is_valid()) {
            continue;
        }
        const std::string label = std::string{g.name} + " normal";
        check_wraps(label.c_str(), n, 'v', __LINE__);
        if (g.tiles_in_u) {
            check_wraps(label.c_str(), n, 'u', __LINE__);
        }
    }
}

/// Input this generator cannot read must produce nothing, not a buffer of zeros
/// that would light every surface from below.
TEST(ProceduralTexture, make_normal_from_height_refuses_input_it_cannot_read) {
    const ProcTexResult empty;
    CHECK_FALSE(stratum::make_normal_from_height(empty, 1.0f).is_valid());

    // Every block-compressed format, not just the first one in the enum. A
    // switch that returns {} for BC7 and falls through for BC7_SRGB reads the
    // compressed blocks as if they were loose bytes and returns a normal map made
    // of noise, which is worse than returning nothing.
    for (const TextureFormat compressed_format :
         {TextureFormat::BC7, TextureFormat::BC7_SRGB, TextureFormat::BC5_Normal}) {
        ProcTexResult compressed;
        compressed.desc.width = 64;
        compressed.desc.height = 64;
        compressed.desc.format = compressed_format;
        compressed.pixels.assign(64 * 64, 0);
        if (stratum::make_normal_from_height(compressed, 1.0f).is_valid()) {
            fail("a block-compressed height field is refused",
                 "format " + std::to_string(static_cast<int>(compressed_format)) +
                     " produced a normal map instead of an empty result",
                 __LINE__);
        }
    }

    // A desc that claims more pixels than the buffer holds must not be read.
    ProcTexResult truncated = flat_height(64, 100);
    truncated.pixels.resize(truncated.pixels.size() / 2);
    CHECK_FALSE(stratum::make_normal_from_height(truncated, 1.0f).is_valid());
}

// ============================================================================
// make_orm
// ============================================================================

/// A uniform pack, with the channels in the documented order. Getting r and b the
/// wrong way round makes every road metallic and fully unoccluded.
TEST(ProceduralTexture, make_orm_packs_occlusion_roughness_metallic_in_that_order) {
    const ProcTexResult orm = stratum::make_orm(0.25f, 0.5f, 0.75f, 8);

    CHECK_TRUE(orm.is_valid());
    CHECK_EQ(static_cast<int>(orm.desc.format), static_cast<int>(TextureFormat::RGBA8));
    CHECK_EQ(orm.desc.width, 8u);
    CHECK_EQ(orm.pixels.size(), orm.desc.level0_bytes());

    for (uint32_t y = 0; y < orm.desc.height; ++y) {
        for (uint32_t x = 0; x < orm.desc.width; ++x) {
            CHECK_NEAR(texel(orm, x, y, 0), 0.25 * 255.0, 1.0);   // occlusion
            CHECK_NEAR(texel(orm, x, y, 1), 0.5 * 255.0, 1.0);    // roughness
            CHECK_NEAR(texel(orm, x, y, 2), 0.75 * 255.0, 1.0);   // metallic
            CHECK_EQ(static_cast<int>(texel(orm, x, y, 3)), 255);
        }
    }
}

/// A uniform pack is constant, so it tiles trivially and exactly -- the one place
/// an exact edge comparison IS the right test.
TEST(ProceduralTexture, a_uniform_orm_pack_is_exactly_constant) {
    const ProcTexResult orm = stratum::make_orm(1.0f, 1.0f, 0.0f, 16);
    CHECK_TRUE(orm.is_valid());
    if (!orm.is_valid()) {
        return;
    }
    for (size_t i = 4; i < orm.pixels.size(); ++i) {
        CHECK_EQ(static_cast<int>(orm.pixels[i]), static_cast<int>(orm.pixels[i % 4]));
    }
    CHECK_NEAR(column_step(orm, 0, orm.desc.width - 1), 0.0, 1e-12);
    CHECK_NEAR(row_step(orm, 0, orm.desc.height - 1), 0.0, 1e-12);
}

/// 1x1 is legal and is what the manager's default ORM uses. Out-of-range scalars
/// clamp rather than wrap through the byte cast.
TEST(ProceduralTexture, make_orm_accepts_a_single_texel_and_clamps_its_scalars) {
    const ProcTexResult one = stratum::make_orm(1.0f, 1.0f, 0.0f, 1);
    CHECK_TRUE(one.is_valid());
    CHECK_EQ(one.pixels.size(), size_t{4});
    CHECK_EQ(static_cast<int>(one.pixels[0]), 255);
    CHECK_EQ(static_cast<int>(one.pixels[1]), 255);
    CHECK_EQ(static_cast<int>(one.pixels[2]), 0);
    CHECK_EQ(static_cast<int>(one.pixels[3]), 255);

    const ProcTexResult clamped = stratum::make_orm(-5.0f, 12.0f, -0.001f, 4);
    CHECK_TRUE(clamped.is_valid());
    if (clamped.is_valid()) {
        CHECK_EQ(static_cast<int>(clamped.pixels[0]), 0);
        CHECK_EQ(static_cast<int>(clamped.pixels[1]), 255);
        CHECK_EQ(static_cast<int>(clamped.pixels[2]), 0);
    }

    CHECK_FALSE(stratum::make_orm(1.0f, 1.0f, 0.0f, 0).is_valid());
    CHECK_FALSE(stratum::make_orm(1.0f, 1.0f, 0.0f, 12).is_valid());
}

// ============================================================================
// The markings atlas
// ============================================================================

/// Size and format first: everything below indexes into this buffer.
TEST(ProceduralTexture, markings_atlas_matches_its_description) {
    const ProcTexResult a = stratum::make_markings_atlas(atlas::kAtlasSizePixels);

    CHECK_TRUE(a.is_valid());
    if (!a.is_valid()) {
        return;
    }
    CHECK_EQ(a.desc.width, static_cast<uint32_t>(atlas::kAtlasSizePixels));
    CHECK_EQ(a.desc.height, static_cast<uint32_t>(atlas::kAtlasSizePixels));
    CHECK_EQ(static_cast<int>(a.desc.format), static_cast<int>(TextureFormat::RGBA8_SRGB));
    CHECK_EQ(a.pixels.size(), a.desc.level0_bytes());

    // Below 256 a 64 px cell is under 16 px and the arrows are unreadable.
    CHECK_FALSE(stratum::make_markings_atlas(128).is_valid());
    CHECK_FALSE(stratum::make_markings_atlas(0).is_valid());
    CHECK_FALSE(stratum::make_markings_atlas(1000).is_valid());
    CHECK_TRUE(stratum::make_markings_atlas(256).is_valid());
}

/**
 * @brief Every sprite the emitter can reference has paint inside its inset rect
 *
 * The rect is what the emitter samples. A sprite whose block was never drawn, or
 * drawn outside its inset, produces a marking quad that renders as nothing at all
 * -- geometry present, material correct, invisible.
 */
TEST(ProceduralTexture, every_sprite_rect_contains_paint) {
    const ProcTexResult a = stratum::make_markings_atlas(atlas::kAtlasSizePixels);
    CHECK_TRUE(a.is_valid());
    if (!a.is_valid()) {
        return;
    }

    for (uint8_t i = 0; i < kSpriteCount; ++i) {
        const auto s = static_cast<atlas::MarkingSprite>(i);
        const Block r = inset_of(s, a.desc.width);

        CHECK(r.x1 > r.x0);
        CHECK(r.y1 > r.y0);

        size_t painted = 0;
        for (int y = r.y0; y < r.y1; ++y) {
            for (int x = r.x0; x < r.x1; ++x) {
                if (texel(a, static_cast<uint32_t>(x), static_cast<uint32_t>(y), 3) > 0) {
                    ++painted;
                }
            }
        }
        if (painted == 0) {
            fail("sprite rect contains paint",
                 std::string{atlas::marking_sprite_name(s)} + " is entirely transparent",
                 __LINE__);
        }
    }
}

/**
 * @brief Nothing is painted outside a sprite block
 *
 * Roughly half the atlas is deliberately unpainted, reserved for growth. Paint
 * that escapes into it does not stay there once mips are generated: level 1
 * averages it into the neighbouring block, and the sprite next door gains a
 * ghost. This is the anti-bleed guarantee the inset exists to make possible.
 */
TEST(ProceduralTexture, the_free_space_between_blocks_is_untouched) {
    const ProcTexResult a = stratum::make_markings_atlas(atlas::kAtlasSizePixels);
    CHECK_TRUE(a.is_valid());
    if (!a.is_valid()) {
        return;
    }
    const uint32_t n = a.desc.width;

    // Allowed-paint mask, dilated by one texel so a block edge that rounds
    // outward by a pixel is not reported as a bleed.
    std::vector<uint8_t> allowed(static_cast<size_t>(n) * n, 0);
    for (uint8_t i = 0; i < kSpriteCount; ++i) {
        const Block b = block_of(static_cast<atlas::MarkingSprite>(i), n);
        const int x0 = std::max(0, b.x0 - 1);
        const int y0 = std::max(0, b.y0 - 1);
        const int x1 = std::min<int>(static_cast<int>(n), b.x1 + 1);
        const int y1 = std::min<int>(static_cast<int>(n), b.y1 + 1);
        for (int y = y0; y < y1; ++y) {
            for (int x = x0; x < x1; ++x) {
                allowed[static_cast<size_t>(y) * n + x] = 1;
            }
        }
    }

    size_t escaped = 0;
    for (uint32_t y = 0; y < n; ++y) {
        for (uint32_t x = 0; x < n; ++x) {
            if (!allowed[static_cast<size_t>(y) * n + x] && texel(a, x, y, 3) != 0) {
                ++escaped;
            }
        }
    }
    if (escaped != 0) {
        fail("no paint outside a sprite block",
             std::to_string(escaped) + " texels of paint in reserved atlas space", __LINE__);
    }
}

/**
 * @brief Pictogram blocks keep a transparent margin at their left and right edges
 *
 * The per-sprite half of the anti-bleed guarantee. Only the pictograms are
 * checked: the continuous sprites -- solid lines, stop lines, zebra stripes, box
 * hatching and the dashes -- are painted to their block edge on purpose, so that
 * the inset rect the emitter samples lands strictly inside the paint rather than
 * clipping it. An arrow has no such excuse.
 */
TEST(ProceduralTexture, pictogram_blocks_have_transparent_side_margins) {
    const ProcTexResult a = stratum::make_markings_atlas(atlas::kAtlasSizePixels);
    CHECK_TRUE(a.is_valid());
    if (!a.is_valid()) {
        return;
    }

    for (const atlas::MarkingSprite s : kPictograms) {
        const Block b = block_of(s, a.desc.width);
        size_t bled = 0;
        for (int y = b.y0; y < b.y1; ++y) {
            const auto yy = static_cast<uint32_t>(y);
            if (texel(a, static_cast<uint32_t>(b.x0), yy, 3) != 0) {
                ++bled;
            }
            if (texel(a, static_cast<uint32_t>(b.x1 - 1), yy, 3) != 0) {
                ++bled;
            }
        }
        if (bled != 0) {
            fail("pictogram keeps a transparent side margin",
                 std::string{atlas::marking_sprite_name(s)} + " paints " + std::to_string(bled) +
                     " texels on its block's left or right edge column",
                 __LINE__);
        }
    }
}

/**
 * @brief Transparent texels are either paint-coloured or nothing at all
 *
 * The atlas is only ever sampled through its mips, and mip generation averages
 * RGB and alpha INDEPENDENTLY. A half-covered texel at mip 2 whose transparent
 * contributors carry RGB 0 comes out with the right alpha and a colour pulled
 * toward black, so every marking grows a dark fringe as it recedes. The fix is to
 * push the paint colour into the transparent texels next to paint, leaving their
 * alpha at zero -- so the naive assertion "every transparent texel is (0,0,0,0)"
 * is the WRONG test, and passing it would mean the fringe is back.
 *
 * What must actually hold is a pair:
 *
 *  - a transparent texel touching paint carries colour, or the fringe returns;
 *  - that colour does not travel far, or the "reserved for growth" half of the
 *    atlas is quietly full of white and a future sprite drawn there inherits it.
 *
 * The far bound is measured, not assumed: a breadth-first distance transform from
 * every painted texel, so the test does not have to know the dilation radius the
 * generator chose.
 */
TEST(ProceduralTexture, transparent_texels_carry_paint_colour_only_next_to_paint) {
    const ProcTexResult a = stratum::make_markings_atlas(atlas::kAtlasSizePixels);
    CHECK_TRUE(a.is_valid());
    if (!a.is_valid()) {
        return;
    }
    const int n = static_cast<int>(a.desc.width);
    const auto idx = [n](int x, int y) { return static_cast<size_t>(y) * n + x; };

    // Multi-source BFS out of the painted texels. -1 means "not reached".
    std::vector<int> distance(static_cast<size_t>(n) * n, -1);
    std::vector<int> frontier;
    for (int y = 0; y < n; ++y) {
        for (int x = 0; x < n; ++x) {
            if (texel(a, static_cast<uint32_t>(x), static_cast<uint32_t>(y), 3) > 0) {
                distance[idx(x, y)] = 0;
                frontier.push_back(y * n + x);
            }
        }
    }
    CHECK_FALSE(frontier.empty());

    // Only the first few rings can matter, so the search is capped rather than run
    // to completion over a megapixel.
    constexpr int kMaxRing = 12;
    for (int ring = 0; ring < kMaxRing && !frontier.empty(); ++ring) {
        std::vector<int> next;
        for (const int cell : frontier) {
            const int x = cell % n;
            const int y = cell / n;
            const int dx[4] = {-1, 1, 0, 0};
            const int dy[4] = {0, 0, -1, 1};
            for (int k = 0; k < 4; ++k) {
                const int nx = x + dx[k];
                const int ny = y + dy[k];
                if (nx < 0 || ny < 0 || nx >= n || ny >= n) {
                    continue;
                }
                if (distance[idx(nx, ny)] != -1) {
                    continue;
                }
                distance[idx(nx, ny)] = ring + 1;
                next.push_back(ny * n + nx);
            }
        }
        frontier.swap(next);
    }

    size_t black_beside_paint = 0;
    size_t colour_far_from_paint = 0;
    int furthest_colour = 0;

    for (int y = 0; y < n; ++y) {
        for (int x = 0; x < n; ++x) {
            const auto xx = static_cast<uint32_t>(x);
            const auto yy = static_cast<uint32_t>(y);
            if (texel(a, xx, yy, 3) != 0) {
                continue;   // painted; its colour is its own business
            }

            const bool has_colour = texel(a, xx, yy, 0) != 0 || texel(a, xx, yy, 1) != 0 ||
                                    texel(a, xx, yy, 2) != 0;
            const int d = distance[idx(x, y)];

            if (d == 1 && !has_colour) {
                ++black_beside_paint;
            }
            if (has_colour) {
                furthest_colour = std::max(furthest_colour, d < 0 ? kMaxRing + 1 : d);
                if (d < 0 || d > kMaxRing) {
                    ++colour_far_from_paint;
                }
            }
        }
    }

    if (black_beside_paint != 0) {
        fail("transparent texels touching paint carry the paint colour",
             std::to_string(black_beside_paint) +
                 " transparent texels adjacent to paint are RGB 0; every marking will "
                 "grow a dark fringe in its mips",
             __LINE__);
    }
    if (colour_far_from_paint != 0) {
        fail("paint colour does not leak into the reserved atlas space",
             std::to_string(colour_far_from_paint) + " transparent texels more than " +
                 std::to_string(kMaxRing) + " texels from any paint carry colour",
             __LINE__);
    }

    // A guard band of a few texels is the point; a guard band of a hundred is a
    // flood fill with extra steps.
    CHECK(furthest_colour <= 8);

    // And the far field really is empty, not merely "not white": the corner of the
    // reserved band E.
    CHECK_EQ(static_cast<int>(texel(a, 0, static_cast<uint32_t>(n - 1), 3)), 0);
    CHECK_EQ(static_cast<int>(texel(a, 0, static_cast<uint32_t>(n - 1), 0)), 0);
    CHECK_EQ(static_cast<int>(texel(a, static_cast<uint32_t>(n - 1),
                                    static_cast<uint32_t>(n - 1), 3)), 0);
    CHECK_EQ(static_cast<int>(texel(a, static_cast<uint32_t>(n - 1),
                                    static_cast<uint32_t>(n - 1), 0)), 0);
}

/**
 * @brief ArrowLeft and ArrowRight are mirrors of each other
 *
 * The orientation check marking_atlas.hpp asks for by name. Every other
 * orientation mistake -- travel pointing down the image, left and right swapped
 * by a cross product whose handedness flipped in the world mapping -- is silent
 * on a symmetric sprite and loud here.
 */
TEST(ProceduralTexture, arrow_left_and_arrow_right_are_horizontal_mirrors) {
    const ProcTexResult a = stratum::make_markings_atlas(atlas::kAtlasSizePixels);
    CHECK_TRUE(a.is_valid());
    if (!a.is_valid()) {
        return;
    }

    const Block l = block_of(atlas::MarkingSprite::ArrowLeft, a.desc.width);
    const Block r = block_of(atlas::MarkingSprite::ArrowRight, a.desc.width);

    CHECK_EQ(l.x1 - l.x0, r.x1 - r.x0);
    CHECK_EQ(l.y1 - l.y0, r.y1 - r.y0);
    if (l.x1 - l.x0 != r.x1 - r.x0 || l.y1 - l.y0 != r.y1 - r.y0) {
        return;
    }

    const int w = l.x1 - l.x0;
    const int h = l.y1 - l.y0;
    size_t mismatched = 0;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const uint8_t left = texel(a, static_cast<uint32_t>(l.x0 + x),
                                       static_cast<uint32_t>(l.y0 + y), 3);
            const uint8_t right = texel(a, static_cast<uint32_t>(r.x0 + (w - 1 - x)),
                                        static_cast<uint32_t>(r.y0 + y), 3);
            if (left != right) {
                ++mismatched;
            }
        }
    }

    // Tolerant of rasteriser rounding on the mirrored diagonal, intolerant of a
    // sprite that is not a mirror at all -- a non-mirrored pair differs over the
    // whole arrowhead, tens of percent of the block.
    const double fraction = static_cast<double>(mismatched) / static_cast<double>(w * h);
    if (fraction > 0.01) {
        fail("ArrowLeft and ArrowRight are exact mirrors",
             std::to_string(mismatched) + " of " + std::to_string(w * h) +
                 " texels differ (" + std::to_string(fraction * 100.0) + "%)",
             __LINE__);
    }
}

/**
 * @brief GiveWayTriangles has its base at the top and its apex pointing down
 *
 * marking_atlas.hpp states it explicitly, and it is the one sprite whose
 * orientation contradicts the natural reading of "an arrow points the way you
 * travel". Painted at 180 degrees it still looks like a give-way triangle, on a
 * road, at the right place -- pointing at the driver instead of away.
 */
TEST(ProceduralTexture, give_way_triangle_has_its_base_at_the_top) {
    const ProcTexResult a = stratum::make_markings_atlas(atlas::kAtlasSizePixels);
    CHECK_TRUE(a.is_valid());
    if (!a.is_valid()) {
        return;
    }

    const Block b = block_of(atlas::MarkingSprite::GiveWayTriangles, a.desc.width);
    const int mid = (b.y0 + b.y1) / 2;

    double top = 0.0;
    double bottom = 0.0;
    for (int y = b.y0; y < b.y1; ++y) {
        for (int x = b.x0; x < b.x1; ++x) {
            const double alpha =
                texel(a, static_cast<uint32_t>(x), static_cast<uint32_t>(y), 3);
            (y < mid ? top : bottom) += alpha;
        }
    }

    if (!(top > bottom * 1.5)) {
        fail("the give-way triangle's base is at the top",
             "top-half coverage " + std::to_string(top) + " vs bottom-half " +
                 std::to_string(bottom) + " -- the sprite is upside down",
             __LINE__);
    }
}

/**
 * @brief The yellow sprites are yellow and the white ones are not
 *
 * Three sprites are yellow and the rest are white. Painting a yellow line white
 * is a legal-meaning change on a real road, and it is invisible in any test that
 * only looks at alpha.
 */
TEST(ProceduralTexture, yellow_sprites_are_yellow_and_white_sprites_are_neutral) {
    const ProcTexResult a = stratum::make_markings_atlas(atlas::kAtlasSizePixels);
    CHECK_TRUE(a.is_valid());
    if (!a.is_valid()) {
        return;
    }

    struct Case {
        atlas::MarkingSprite sprite;
        bool yellow;
    };
    const Case cases[] = {
        {atlas::MarkingSprite::SolidYellow, true},
        {atlas::MarkingSprite::DashedYellow, true},
        {atlas::MarkingSprite::DoubleSolidYellow, true},
        {atlas::MarkingSprite::SolidWhite, false},
        {atlas::MarkingSprite::DashWhite, false},
        {atlas::MarkingSprite::StopLine, false},
        {atlas::MarkingSprite::ArrowStraight, false},
    };

    for (const Case& c : cases) {
        const Block r = inset_of(c.sprite, a.desc.width);
        double red = 0.0;
        double blue = 0.0;
        size_t opaque = 0;
        for (int y = r.y0; y < r.y1; ++y) {
            for (int x = r.x0; x < r.x1; ++x) {
                const auto xx = static_cast<uint32_t>(x);
                const auto yy = static_cast<uint32_t>(y);
                if (texel(a, xx, yy, 3) < 200) {
                    continue;
                }
                red += texel(a, xx, yy, 0);
                blue += texel(a, xx, yy, 2);
                ++opaque;
            }
        }
        if (opaque == 0) {
            fail("sprite has opaque paint to sample",
                 std::string{atlas::marking_sprite_name(c.sprite)} + " has no opaque texel",
                 __LINE__);
            continue;
        }

        // Yellow is (0.90, 0.72, 0.12): red far above blue however it is encoded.
        // White is (0.95, 0.95, 0.93): a handful of levels apart at most.
        const double gap = (red - blue) / static_cast<double>(opaque);
        if (c.yellow && !(gap > 60.0)) {
            fail("yellow paint is yellow",
                 std::string{atlas::marking_sprite_name(c.sprite)} + " red-minus-blue is " +
                     std::to_string(gap),
                 __LINE__);
        }
        if (!c.yellow && !(gap < 30.0)) {
            fail("white paint is neutral",
                 std::string{atlas::marking_sprite_name(c.sprite)} + " red-minus-blue is " +
                     std::to_string(gap),
                 __LINE__);
        }
    }
}

/// The atlas is drawn, not sampled from noise, so it must be reproducible with
/// no seed at all -- two runs of the editor must not disagree about a stop line.
TEST(ProceduralTexture, the_markings_atlas_is_deterministic) {
    const ProcTexResult a = stratum::make_markings_atlas(512);
    const ProcTexResult b = stratum::make_markings_atlas(512);
    CHECK_TRUE(a.is_valid());
    CHECK_EQ(a.pixels.size(), b.pixels.size());
    CHECK_TRUE(a.pixels == b.pixels);
}
