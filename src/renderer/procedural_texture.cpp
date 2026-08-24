/**
 * @file procedural_texture.cpp
 * @brief Implementation of the generated surface textures and the markings atlas
 *
 * ### Why there is periodic noise in here when src/procgen/noise.hpp exists
 *
 * noise.hpp's simplex generator is the noise this project already owns and it is
 * reused below -- but simplex noise has NO PERIOD. There is no argument to it that
 * says "repeat every N units", and making 2D simplex tile requires evaluating it on
 * a 4D torus, which noise.hpp does not expose (it stops at 3D). Adding a periodic
 * entry point to it is not an option from here either: noise.hpp is stratum_core
 * and this file is stratum_editor_lib, so the dependency only runs one way, and the
 * contract agent froze this file's ownership at procedural_texture.{hpp,cpp}.
 *
 * So the LATTICE is built here and the VALUES ON IT come from noise.hpp. Every
 * lattice coordinate is reduced with an integer modulo before it is used, which is
 * what makes the field exactly periodic; the value at a reduced coordinate is one
 * simplex2d() sample, taken far enough apart that neighbouring lattice points are
 * decorrelated. Nothing here re-implements a noise function. Everything discrete --
 * cell jitter, per-stone tone, per-slab tone, per-texel grain -- uses the explicit
 * integer hash the header's determinism clause calls for.
 *
 * The lattices are PRECOMPUTED, once per generator, into period x period arrays.
 * A 1024 x 1024 texture is a million texels and an fBm is five octaves, so calling
 * simplex2d() per texel per octave would be twenty million simplex evaluations per
 * texture and the startup cost the header's "fast enough to run at startup" note
 * rules out. Precomputing turns it into at most a few tens of thousands of simplex
 * evaluations plus a bilinear fetch per octave per texel.
 *
 * ### Tileability, and what is actually provable
 *
 * See is_tileable() in the header for why "opposite edges must be byte-identical"
 * is the wrong test. The proof splits in two:
 *
 *  - EXACT, by construction: PeriodicLattice::at() and periodic_worley() reduce
 *    every lattice/cell coordinate modulo the period before hashing, so the field
 *    satisfies f(u + 1, v) == f(u, v) bit-for-bit. debug_check_primitives() asserts
 *    that directly, with `==`, at power-of-two and non-power-of-two periods.
 *  - MEASURED, on the finished pixels: every surface generator ends with an
 *    assert(is_tileable(...)) in debug builds, which catches a seam introduced by
 *    something that is NOT the noise -- a joint lattice that does not divide the
 *    tile, a gradient that clamps instead of wrapping, a lateral feature placed at
 *    an unwrapped distance.
 *
 * ### Colour space
 *
 * Surface albedo is authored directly in sRGB: the float triples below are the
 * values that end up in the RGBA8_SRGB bytes, so they read as the colour you would
 * pick in a colour picker, and the hardware linearises them at sample time. Height
 * goes in alpha, which RGBA8_SRGB leaves linear. Variation is applied in sRGB too,
 * which is deliberate -- perceptually even mottling is what these surfaces want,
 * not radiometrically even mottling.
 */

#include "renderer/procedural_texture.hpp"

#include "osm/road/marking_atlas.hpp"
#include "procgen/noise.hpp"

#include <glm/glm.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <vector>

namespace stratum {

namespace {

// ============================================================================
// Scalar helpers
// ============================================================================

inline float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }

inline float smoothstepf(float e0, float e1, float x) {
    if (e1 <= e0) {
        return x < e0 ? 0.0f : 1.0f;
    }
    const float t = clamp01((x - e0) / (e1 - e0));
    return t * t * (3.0f - 2.0f * t);
}

/// Perlin's quintic fade. C2 at the lattice points, which is what stops a value
/// noise from showing its grid as faint diagonal creases in the normal map.
inline float quintic(float t) { return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f); }

/// Euclidean modulo. The whole tiling contract rests on this being non-negative.
inline int wrap_i(int v, int n) {
    const int r = v % n;
    return r < 0 ? r + n : r;
}

inline bool is_pow2(uint32_t v) { return v != 0u && (v & (v - 1u)) == 0u; }

/// The header's size clause for every surface generator
inline bool valid_surface_size(uint32_t size) { return size >= 4u && is_pow2(size); }

/// Levels in a full chain down to 1x1
inline uint32_t full_mip_count(uint32_t size) {
    uint32_t levels = 1u;
    while (size > 1u) {
        size >>= 1u;
        ++levels;
    }
    return levels;
}

inline uint8_t to_u8(float v) {
    return static_cast<uint8_t>(std::lround(clamp01(v) * 255.0f));
}

/// Wrapped distance between two positions on the unit circle. Lateral features --
/// wheel paths, kerb joints -- must be placed with this or they seam.
inline float wrapped_dist(float a, float b) {
    float d = std::fabs(a - b);
    return std::min(d, 1.0f - d);
}

// ============================================================================
// Integer hashing
//
// The header's determinism clause: an explicit integer hash of the seed and the
// coordinate, never std::rand and never a std:: distribution. This is the
// MurmurHash3 finalizer, which avalanches well enough that adjacent lattice cells
// are visibly uncorrelated, and is exact integer arithmetic on every platform.
// ============================================================================

constexpr uint32_t hash_u32(uint32_t x) {
    x ^= x >> 16;
    x *= 0x85ebca6bu;
    x ^= x >> 13;
    x *= 0xc2b2ae35u;
    x ^= x >> 16;
    return x;
}

constexpr uint32_t hash_mix(uint32_t a, uint32_t b) {
    return hash_u32(a ^ (b + 0x9e3779b9u + (a << 6) + (a >> 2)));
}

constexpr uint32_t hash2i(int32_t x, int32_t y, uint32_t seed) {
    const uint32_t ux = static_cast<uint32_t>(x) * 0x27d4eb2du;
    const uint32_t uy = static_cast<uint32_t>(y) * 0x165667b1u;
    return hash_mix(hash_mix(ux, uy), seed);
}

/// Uniform in [0, 1). Uses the top 24 bits, which are the best-mixed ones.
inline float unit_from_hash(uint32_t h) {
    return static_cast<float>(h >> 8) * (1.0f / 16777216.0f);
}

// ============================================================================
// Periodic value noise
// ============================================================================

/**
 * @brief A period x period lattice of values, sampled with quintic interpolation
 *
 * Exactly periodic: at() reduces both coordinates modulo the period before it
 * indexes, so at(x, y) and at(x + period, y) are the SAME array element rather
 * than two elements that happen to be close. sample() reads only through at(),
 * so the interpolated field inherits that exactly.
 *
 * The values themselves come from the vendored simplex noise. kSpread is the gap
 * between the simplex coordinates of two neighbouring lattice cells: simplex noise
 * has features about one unit across, so a gap of 7.31 puts neighbours seven
 * features apart and they behave as independent draws. A gap near 1.0 would make
 * the lattice a blurred copy of the simplex field instead of a hash of it.
 */
class PeriodicLattice {
public:
    PeriodicLattice(int period, uint32_t salt, const procgen::Noise& noise)
        : m_period(std::max(1, period)),
          m_values(static_cast<size_t>(m_period) * static_cast<size_t>(m_period)) {
        constexpr float kSpread = 7.31f;
        // Offsets keep two lattices with the same period from being the same field.
        // Bounded so the largest simplex coordinate stays well inside the range
        // where a float still resolves a fraction of a noise feature.
        const float ox = static_cast<float>(hash_u32(salt) & 0x3ffu) * 0.517f;
        const float oy = static_cast<float>(hash_u32(salt ^ 0x5bf03635u) & 0x3ffu) * 0.517f;

        for (int y = 0; y < m_period; ++y) {
            for (int x = 0; x < m_period; ++x) {
                const float n = noise.simplex2d(static_cast<float>(x) * kSpread + ox,
                                                static_cast<float>(y) * kSpread + oy);
                m_values[static_cast<size_t>(y) * static_cast<size_t>(m_period) + static_cast<size_t>(x)] =
                    clamp01(0.5f + 0.5f * n);
            }
        }
    }

    /// Lattice value at an arbitrary integer coordinate, wrapped. Exactly periodic.
    [[nodiscard]] float at(int x, int y) const {
        const size_t iy = static_cast<size_t>(wrap_i(y, m_period));
        const size_t ix = static_cast<size_t>(wrap_i(x, m_period));
        return m_values[iy * static_cast<size_t>(m_period) + ix];
    }

    /// Interpolated field. @p u and @p v are in tile units; any real value works
    /// and values one apart give identical results.
    [[nodiscard]] float sample(float u, float v) const {
        const float fx = u * static_cast<float>(m_period);
        const float fy = v * static_cast<float>(m_period);
        const int x0 = static_cast<int>(std::floor(fx));
        const int y0 = static_cast<int>(std::floor(fy));
        const float tx = quintic(fx - static_cast<float>(x0));
        const float ty = quintic(fy - static_cast<float>(y0));

        // Two integer divisions rather than eight. wrap(x + 1) == wrap(x) + 1
        // except at the last cell, so the neighbour needs a compare and not a
        // modulo -- and this is the hot loop: a million texels times five octaves
        // times four corners is twenty million of these.
        const int xw = wrap_i(x0, m_period);
        const int yw = wrap_i(y0, m_period);
        const int xn = (xw + 1 == m_period) ? 0 : xw + 1;
        const int yn = (yw + 1 == m_period) ? 0 : yw + 1;

        const size_t row0 = static_cast<size_t>(yw) * static_cast<size_t>(m_period);
        const size_t row1 = static_cast<size_t>(yn) * static_cast<size_t>(m_period);
        const float a = lerpf(m_values[row0 + static_cast<size_t>(xw)],
                              m_values[row0 + static_cast<size_t>(xn)], tx);
        const float b = lerpf(m_values[row1 + static_cast<size_t>(xw)],
                              m_values[row1 + static_cast<size_t>(xn)], tx);
        return lerpf(a, b, ty);
    }

    [[nodiscard]] int period() const { return m_period; }

private:
    int m_period;
    std::vector<float> m_values;
};

/**
 * @brief Fractal sum of periodic lattices, each double the previous frequency
 *
 * Every octave has an integer period, so every octave -- and therefore the sum --
 * wraps at exactly one tile. Doubling the period rather than scaling the
 * coordinate is the whole trick: a lacunarity of 2 on an integer base period is
 * still an integer period.
 *
 * Returns [0, 1], normalised by the amplitude sum so a 3-octave and a 6-octave fBm
 * have the same mean and roughly the same range.
 */
class PeriodicFbm {
public:
    PeriodicFbm(int base_period, int octaves, float persistence, uint32_t salt,
                const procgen::Noise& noise) {
        octaves = std::clamp(octaves, 1, 8);
        persistence = std::clamp(persistence, 0.05f, 0.95f);

        int period = std::max(1, base_period);
        float amplitude = 1.0f;
        float total = 0.0f;
        m_octaves.reserve(static_cast<size_t>(octaves));
        m_amplitudes.reserve(static_cast<size_t>(octaves));

        for (int i = 0; i < octaves; ++i) {
            m_octaves.emplace_back(period, hash_mix(salt, static_cast<uint32_t>(i) + 1u), noise);
            m_amplitudes.push_back(amplitude);
            total += amplitude;
            amplitude *= persistence;
            period *= 2;
        }
        m_inv_total = total > 0.0f ? 1.0f / total : 1.0f;
    }

    [[nodiscard]] float sample(float u, float v) const {
        float sum = 0.0f;
        for (size_t i = 0; i < m_octaves.size(); ++i) {
            sum += m_amplitudes[i] * m_octaves[i].sample(u, v);
        }
        return sum * m_inv_total;
    }

private:
    std::vector<PeriodicLattice> m_octaves;
    std::vector<float> m_amplitudes;
    float m_inv_total = 1.0f;
};

// ============================================================================
// Periodic cellular (Worley) noise
// ============================================================================

/**
 * @brief The feature points of a wrapping n x n cell lattice, precomputed
 *
 * Same argument as PeriodicLattice: the feature position and identity of a cell
 * depend only on its WRAPPED index, so there are exactly n * n of them and
 * recomputing four hashes for each of nine cells at every one of a million texels
 * is the single most expensive thing this file was doing. n is at most 96 here, so
 * the table is a few tens of kilobytes.
 *
 * @param n            Cells per tile in both axes. Square cells, so distances are
 *                     isotropic in cell units.
 * @param seed         Deterministic seed
 * @param jitter       Feature displacement from the cell centre, in cell units.
 *                     0 gives a perfect grid, 0.5 gives full Worley chaos.
 * @param running_bond Offset odd cell rows by half a cell, which is what turns a
 *                     grid of stones into COURSES. @p n must be EVEN for the bond
 *                     to survive the wrap -- with an odd count the last course and
 *                     the first share a phase, which is not a seam but is a flaw
 *                     repeated down every road in the network. The callers force it.
 */
class PeriodicCells {
public:
    /// One cell's feature: its offset from the cell's own origin, in cell units,
    /// with the running-bond shift already folded in, and its identity hash.
    struct Cell {
        float dx = 0.5f;
        float dy = 0.5f;
        uint32_t hash = 0u;
    };

    PeriodicCells(int n, uint32_t seed, float jitter, bool running_bond)
        : m_n(std::max(2, n)),
          m_cells(static_cast<size_t>(std::max(2, n)) * static_cast<size_t>(std::max(2, n))) {
        jitter = std::clamp(jitter, 0.0f, 0.5f);
        for (int y = 0; y < m_n; ++y) {
            for (int x = 0; x < m_n; ++x) {
                const uint32_t h = hash2i(x, y, seed);
                Cell c;
                c.hash = h;
                c.dx = 0.5f + (unit_from_hash(h) - 0.5f) * 2.0f * jitter;
                c.dy = 0.5f + (unit_from_hash(hash_u32(h)) - 0.5f) * 2.0f * jitter;
                if (running_bond && (y & 1) != 0) {
                    c.dx += 0.5f;
                }
                m_cells[static_cast<size_t>(y) * static_cast<size_t>(m_n) +
                        static_cast<size_t>(x)] = c;
            }
        }
    }

    /// Cell at an already-wrapped index
    [[nodiscard]] const Cell& at_wrapped(int x, int y) const {
        return m_cells[static_cast<size_t>(y) * static_cast<size_t>(m_n) + static_cast<size_t>(x)];
    }

    [[nodiscard]] int n() const { return m_n; }

private:
    int m_n;
    std::vector<Cell> m_cells;
};

/// One cellular lookup: the two nearest feature distances and the owning cell
struct WorleySample {
    float f1 = 0.0f;        ///< Distance to the nearest feature, in cell units
    float f2 = 0.0f;        ///< Distance to the second nearest, in cell units
    uint32_t cell = 0u;     ///< Hash of the owning cell; the per-stone identity

    /// Distance to the boundary between the two nearest cells, in cell units.
    /// This is the mortar-joint coordinate: zero exactly on a joint, growing
    /// toward the middle of a stone, and independent of stone size.
    [[nodiscard]] float edge() const { return (f2 - f1) * 0.5f; }
};

/**
 * @brief Cellular noise that wraps at one tile
 *
 * Exactly periodic, because every cell it consults is addressed by its wrapped
 * index: the cell at (n, y) IS the cell at (0, y), not a different cell that
 * happens to be nearby.
 *
 * @param u,v   Tile coordinates, expected in [0, 1) for a texture but correct for
 *              any real value
 * @param cells Precomputed feature table
 */
WorleySample periodic_worley(float u, float v, const PeriodicCells& cells) {
    const int n = cells.n();
    const float gx = u * static_cast<float>(n);
    const float gy = v * static_cast<float>(n);
    const int ix = static_cast<int>(std::floor(gx));
    const int iy = static_cast<int>(std::floor(gy));

    // Two integer divisions for the whole 3 x 3 neighbourhood: the wrapped index of
    // a neighbour is the wrapped index of the centre plus or minus one, with the
    // ends folded round.
    const int wx0 = wrap_i(ix, n);
    const int wy0 = wrap_i(iy, n);
    const int wxs[3] = {wx0 == 0 ? n - 1 : wx0 - 1, wx0, wx0 == n - 1 ? 0 : wx0 + 1};
    const int wys[3] = {wy0 == 0 ? n - 1 : wy0 - 1, wy0, wy0 == n - 1 ? 0 : wy0 + 1};

    WorleySample out;
    out.f1 = 1e18f;     // squared distances until the two roots at the end
    out.f2 = 1e18f;

    for (int dy = -1; dy <= 1; ++dy) {
        const int wy = wys[dy + 1];
        const float base_y = static_cast<float>(iy + dy) - gy;
        for (int dx = -1; dx <= 1; ++dx) {
            const PeriodicCells::Cell& c = cells.at_wrapped(wxs[dx + 1], wy);

            // Compare SQUARED distances and take the two roots at the end: sqrt is
            // monotonic so the ordering is identical, the result is the same, and
            // eight of the nine square roots per texel disappear.
            const float ddx = static_cast<float>(ix + dx) - gx + c.dx;
            const float ddy = base_y + c.dy;
            const float d2 = ddx * ddx + ddy * ddy;
            if (d2 < out.f1) {
                out.f2 = out.f1;
                out.f1 = d2;
                out.cell = c.hash;
            } else if (d2 < out.f2) {
                out.f2 = d2;
            }
        }
    }
    out.f1 = std::sqrt(out.f1);
    out.f2 = std::sqrt(out.f2);
    return out;
}

// ============================================================================
// Pixel buffer
// ============================================================================

/// RGBA scratch buffer with the generators' write convention baked in
class Surface {
public:
    explicit Surface(uint32_t size)
        : m_size(size), m_pixels(static_cast<size_t>(size) * static_cast<size_t>(size) * 4u, 0u) {}

    /// @param albedo sRGB colour, 0..1
    /// @param height Linear height, 0..1, which is what make_normal_from_height reads
    void put(uint32_t x, uint32_t y, const glm::vec3& albedo, float height) {
        uint8_t* d = &m_pixels[(static_cast<size_t>(y) * m_size + x) * 4u];
        d[0] = to_u8(albedo.r);
        d[1] = to_u8(albedo.g);
        d[2] = to_u8(albedo.b);
        d[3] = to_u8(height);
    }

    ProcTexResult finish(TextureFormat format) {
        ProcTexResult out;
        out.desc.width = m_size;
        out.desc.height = m_size;
        out.desc.layers = 1;
        out.desc.mip_levels = full_mip_count(m_size);
        out.desc.format = format;
        out.pixels = std::move(m_pixels);
        return out;
    }

    [[nodiscard]] uint32_t size() const { return m_size; }

private:
    uint32_t m_size;
    std::vector<uint8_t> m_pixels;
};

/// Texel-centre tile coordinate. Centres, not corners: a texture sampled with a
/// repeat sampler covers one period across `size` texels, and the first texel sits
/// half a texel in, not on the boundary.
inline float texel_uv(uint32_t i, uint32_t size) {
    return (static_cast<float>(i) + 0.5f) / static_cast<float>(size);
}

/// Per-texel white noise. Trivially tileable: it is uncorrelated everywhere, so the
/// step across the wrap is drawn from the same distribution as every interior step.
inline float texel_grain(uint32_t x, uint32_t y, uint32_t salt) {
    return unit_from_hash(hash2i(static_cast<int32_t>(x), static_cast<int32_t>(y), salt));
}

// ============================================================================
// Debug: the exact half of the tiling proof
// ============================================================================

#ifndef NDEBUG
/**
 * @brief Assert that the periodic primitives are periodic bit-for-bit
 *
 * Runs once per process, on first use, and is compiled out of release builds.
 *
 * The sample points are i / 64, and the periods are 16 and 13. Both products are
 * exactly representable in a float and so is the same product plus the period, so
 * `sample(u, v) == sample(u + 1, v)` is a legitimate EXACT comparison here rather
 * than a comparison that happens to pass. That is the strongest statement
 * available about tileability: not "the seam is small" but "there is no seam,
 * because the two sides are the same arithmetic".
 */
void debug_check_primitives() {
    static const bool checked = [] {
        const procgen::Noise noise(20260823u);

        for (const int period : {16, 13}) {
            const PeriodicLattice lat(period, 0x51ed270bu, noise);

            for (int y = -period; y <= period; ++y) {
                for (int x = -period; x <= period; ++x) {
                    assert(lat.at(x, y) == lat.at(x + period, y));
                    assert(lat.at(x, y) == lat.at(x, y + period));
                    assert(lat.at(x, y) == lat.at(x - period, y - period));
                }
            }

            for (int j = 0; j < 64; ++j) {
                const float v = static_cast<float>(j) / 64.0f;
                for (int i = 0; i < 64; ++i) {
                    const float u = static_cast<float>(i) / 64.0f;
                    assert(lat.sample(u, v) == lat.sample(u + 1.0f, v));
                    assert(lat.sample(u, v) == lat.sample(u, v + 1.0f));
                    assert(lat.sample(u, v) == lat.sample(u - 1.0f, v - 1.0f));
                }
            }
        }

        const PeriodicCells cells(8, 0x2f3ab1c5u, 0.30f, true);
        for (int j = 0; j < 32; ++j) {
            const float v = static_cast<float>(j) / 32.0f;
            for (int i = 0; i < 32; ++i) {
                const float u = static_cast<float>(i) / 32.0f;
                const WorleySample a = periodic_worley(u, v, cells);
                const WorleySample b = periodic_worley(u + 1.0f, v, cells);
                const WorleySample c = periodic_worley(u, v + 1.0f, cells);
                assert(a.cell == b.cell && a.cell == c.cell);
                assert(std::fabs(a.f1 - b.f1) < 1e-5f);
                assert(std::fabs(a.f1 - c.f1) < 1e-5f);
                assert(std::fabs(a.edge() - b.edge()) < 1e-5f);
                assert(std::fabs(a.edge() - c.edge()) < 1e-5f);
            }
        }
        return true;
    }();
    (void)checked;
}
#else
inline void debug_check_primitives() {}
#endif

} // namespace

// ============================================================================
// Tileability
// ============================================================================

/**
 * @brief Seam check on finished pixels
 *
 * Invariant: measures continuity, never equality. See the header for why equality
 * would be the wrong thing to demand.
 */
bool is_tileable(const ProcTexResult& tex, bool check_u, bool check_v) {
    if (!tex.is_valid()) {
        return false;
    }

    int channels = 0;
    switch (tex.desc.format) {
        case TextureFormat::RGBA8:
        case TextureFormat::RGBA8_SRGB: channels = 4; break;
        case TextureFormat::R8:         channels = 1; break;
        case TextureFormat::BC7:
        case TextureFormat::BC7_SRGB:
        case TextureFormat::BC5_Normal: return false;   // no texels to compare
    }

    const int w = static_cast<int>(tex.desc.width);
    const int h = static_cast<int>(tex.desc.height);
    if (w < 4 || h < 4) {
        return true;    // too small for the statistic to say anything
    }

    const uint8_t* p = tex.pixels.data();
    const auto texel = [&](int x, int y, int c) -> int {
        return static_cast<int>(p[(static_cast<size_t>(y) * static_cast<size_t>(w) +
                                   static_cast<size_t>(x)) * static_cast<size_t>(channels) +
                                  static_cast<size_t>(c)]);
    };

    // Index 0 of each vector is the WRAP pair; the rest are interior pairs.
    //
    // The reference is the LOCAL neighbourhood of the wrap, not the whole texture.
    // Comparing against the global mean is wrong for exactly the textures this most
    // needs to work on: a cobble or paving lattice puts a cell boundary at u = 0 by
    // construction, so the wrap pair legitimately straddles a MORTAR JOINT and its
    // difference is several times the texture's average. Its immediate neighbours
    // straddle the same joint and are just as large, so a local comparison sees
    // nothing wrong -- while a global one rejects a perfectly tiling texture and
    // teaches whoever hits it to delete the assertion.
    //
    // A real seam is a one-boundary discontinuity, so it raises diffs[0] and
    // nothing else. That is precisely what a local comparison detects and a global
    // one dilutes.
    // Is the wrap pair, diffs[0], an outlier among the interior pairs?
    //
    // TWO criteria, and passing EITHER is enough, because they fail on different
    // textures and a seam has to beat both:
    //
    //  - Local. The median of the eight pairs nearest the wrap. This is the one
    //    that matters for a lattice texture, whose wrap pair sits on a joint by
    //    construction and is legitimately several times the texture's average.
    //  - Distributional. The interior mean plus six standard deviations. This is
    //    the one that matters at small sizes: a 256-texel line average is a noisy
    //    estimate, the local median of eight of them is noisier still, and a fixed
    //    multiple of it rejects perfectly good textures about one time in five. Six
    //    sigma over a population of hundreds is a false positive rate that will not
    //    be seen, and it calibrates itself to whatever spread the texture has.
    //
    // @param diffs Difference per adjacent line pair; index 0 is the wrap pair.
    const auto outlier_free = [](const std::vector<double>& diffs, double local_factor) {
        const size_t n = diffs.size();
        if (n < 10) {
            return true;    // too small for the statistic to say anything
        }

        double sum = 0.0;
        for (size_t i = 1; i < n; ++i) {
            sum += diffs[i];
        }
        const double mean = sum / static_cast<double>(n - 1);
        double var = 0.0;
        for (size_t i = 1; i < n; ++i) {
            const double d = diffs[i] - mean;
            var += d * d;
        }
        const double sd = std::sqrt(var / static_cast<double>(n - 1));

        constexpr size_t kNeighbourhood = 4;
        std::vector<double> local;
        local.reserve(kNeighbourhood * 2u);
        for (size_t i = 1; i <= kNeighbourhood; ++i) {
            local.push_back(diffs[i]);
            local.push_back(diffs[n - i]);
        }
        std::sort(local.begin(), local.end());
        // Median, not mean: one spike must not raise the bar it is measured against.
        const double local_reference = local[local.size() / 2] * local_factor + 1.0;
        const double spread_reference = mean + 6.0 * sd + 0.5;

        return diffs[0] <= std::max(local_reference, spread_reference);
    };

    // The second statistic. The first, applied to `col`/`row` below, averages the
    // ABSOLUTE per-texel difference,
    // which is what catches a discontinuity whose sign varies along the seam -- a
    // noise lattice that does not wrap looks different on each side in a different
    // direction at every texel. It is nearly blind, though, to the other failure:
    // a smooth OFFSET across the wrap, such as a gradient or a low-frequency layer
    // that is not periodic. A 4/255 step buried in asphalt's grain moves the mean
    // absolute difference by less than the grain's own spread.
    //
    // offset_ok() finds those by differencing the LINE MEANS instead. Averaging a
    // whole line first cancels the grain by the square root of its length and
    // leaves the offset standing, which drops the detection floor on asphalt from
    // about 12/255 to about 4/255. Per channel, not summed, so a hue shift cannot
    // be cancelled by an opposite luminance shift.
    //
    // Its reference has one extra term. A texture with a hard feature ON its
    // lattice -- paving's joint sits exactly at v = 0, and the slabs either side of
    // it genuinely differ in height -- has a legitimately large line-mean step at
    // the wrap whose only equals are at the OTHER joints, far outside the local
    // neighbourhood. So a step the texture already contains somewhere is not
    // evidence of a seam, and a quarter of the largest interior step joins the
    // local median as a floor on what counts as unremarkable.
    const auto offset_ok = [&](bool by_column) {
        const int lines = by_column ? w : h;
        const int span = by_column ? h : w;
        if (lines < 10) {
            return true;
        }

        std::vector<double> means(static_cast<size_t>(lines) * static_cast<size_t>(channels), 0.0);
        for (int i = 0; i < lines; ++i) {
            for (int j = 0; j < span; ++j) {
                for (int c = 0; c < channels; ++c) {
                    means[static_cast<size_t>(i) * static_cast<size_t>(channels) +
                          static_cast<size_t>(c)] +=
                        by_column ? texel(i, j, c) : texel(j, i, c);
                }
            }
            for (int c = 0; c < channels; ++c) {
                means[static_cast<size_t>(i) * static_cast<size_t>(channels) +
                      static_cast<size_t>(c)] /= static_cast<double>(span);
            }
        }

        std::vector<double> diffs(static_cast<size_t>(lines), 0.0);
        for (int i = 0; i < lines; ++i) {
            const int prev = (i == 0) ? (lines - 1) : (i - 1);
            double worst = 0.0;
            for (int c = 0; c < channels; ++c) {
                const double a = means[static_cast<size_t>(i) * static_cast<size_t>(channels) +
                                       static_cast<size_t>(c)];
                const double b = means[static_cast<size_t>(prev) * static_cast<size_t>(channels) +
                                       static_cast<size_t>(c)];
                worst = std::max(worst, std::abs(a - b));
            }
            diffs[static_cast<size_t>(i)] = worst;
        }

        return outlier_free(diffs, 2.0);
    };

    if (check_u) {
        std::vector<double> col(static_cast<size_t>(w), 0.0);
        for (int x = 0; x < w; ++x) {
            const int prev = (x == 0) ? (w - 1) : (x - 1);
            double acc = 0.0;
            for (int y = 0; y < h; ++y) {
                for (int c = 0; c < channels; ++c) {
                    acc += std::abs(texel(x, y, c) - texel(prev, y, c));
                }
            }
            col[static_cast<size_t>(x)] = acc / static_cast<double>(h * channels);
        }
        if (!outlier_free(col, 1.5) || !offset_ok(true)) {
            return false;
        }
    }

    if (check_v) {
        std::vector<double> row(static_cast<size_t>(h), 0.0);
        for (int y = 0; y < h; ++y) {
            const int prev = (y == 0) ? (h - 1) : (y - 1);
            double acc = 0.0;
            for (int x = 0; x < w; ++x) {
                for (int c = 0; c < channels; ++c) {
                    acc += std::abs(texel(x, y, c) - texel(x, prev, c));
                }
            }
            row[static_cast<size_t>(y)] = acc / static_cast<double>(w * channels);
        }
        if (!outlier_free(row, 1.5) || !offset_ok(false)) {
            return false;
        }
    }

    return true;
}

// ============================================================================
// Surface generators
// ============================================================================

/**
 * @brief Bituminous road surface
 *
 * Invariant: tiles in both axes; height in alpha is the aggregate relief.
 *
 * Four layers, in order of scale. The wheel paths are the one that has to be
 * placed rather than sampled, and they are placed with wrapped_dist() at tile
 * fractions rather than at metres, because this function does not know the tile is
 * 8 m wide -- it only knows the convention says it is one tile.
 *
 * The lane geometry assumed for the paths: an 8 m tile is two 4 m lanes, lane
 * centres at u = 0.25 and 0.75, a 1.5 m track width either side of each centre.
 * That lands four worn bands per tile.
 */
ProcTexResult make_asphalt(uint32_t size, uint32_t seed, float coarseness) {
    if (!valid_surface_size(size)) {
        return {};
    }
    debug_check_primitives();
    coarseness = clamp01(coarseness);

    const procgen::Noise noise(seed);

    // Aggregate cell size in tile fractions: roughly 40-60 mm chippings on an 8 m
    // tile. Capped at one cell per four texels, below which the grain stops being
    // aggregate and starts being aliasing.
    const int aggregate_period =
        feature_period(static_cast<int>(std::lround(96.0f + 96.0f * coarseness)), 4, size, 4u);

    const PeriodicFbm mottle(3, 4, 0.55f, hash_mix(seed, 0x1u), noise);
    const PeriodicLattice aggregate(aggregate_period, hash_mix(seed, 0x2u), noise);
    const PeriodicLattice patch(7, hash_mix(seed, 0x3u), noise);

    // Bitumen, not black: fresh asphalt is a very dark blue-grey and reads as
    // black only because everything around it is brighter.
    const glm::vec3 base(0.235f, 0.234f, 0.243f);

    const float chip_contrast = 0.085f + 0.155f * coarseness;
    const float chip_relief = 0.30f + 0.40f * coarseness;

    // Four worn bands: two lanes, two wheels each. The profile is a plain taper
    // with no plateau -- a band with a flat centre and a shoulder reads as a
    // painted stripe rather than as wear, which is what an earlier version of this
    // did and it turned every road into corduroy.
    const float wheel_centres[4] = {0.156f, 0.344f, 0.656f, 0.844f};
    constexpr float kWheelFalloff = 0.085f;     // ~0.7 m of an 8 m tile

    Surface surf(size);
    for (uint32_t y = 0; y < size; ++y) {
        const float v = texel_uv(y, size);
        for (uint32_t x = 0; x < size; ++x) {
            const float u = texel_uv(x, size);

            float wear = 0.0f;
            for (const float c : wheel_centres) {
                const float d = wrapped_dist(u, c);
                wear = std::max(wear, 1.0f - smoothstepf(0.0f, kWheelFalloff, d));
            }

            const float broad = mottle.sample(u, v) - 0.5f;
            const float repair = smoothstepf(0.60f, 0.86f, patch.sample(u, v));
            const float chip = aggregate.sample(u, v) * 0.62f +
                               texel_grain(x, y, hash_mix(seed, 0x4u)) * 0.38f - 0.5f;

            // Polished wheel paths: darker, and their aggregate is worn flatter.
            const float chip_here = chip * chip_contrast * (1.0f - wear * 0.45f);

            float delta = broad * 0.11f          // laying and weathering variation
                        + repair * -0.030f       // darker patch repairs
                        + chip_here              // aggregate
                        - wear * 0.026f;         // polished, dirt-filled wheel paths

            const glm::vec3 albedo = base + glm::vec3(delta);

            const float height = 0.46f
                               + chip * chip_relief
                               + broad * 0.10f
                               - wear * 0.035f;

            surf.put(x, y, albedo, height);
        }
    }

    ProcTexResult out = surf.finish(TextureFormat::RGBA8_SRGB);
    assert(is_tileable(out) && "make_asphalt produced a seam");
    return out;
}

/**
 * @brief Poured concrete
 *
 * Invariant: tiles in both axes. Deliberately low contrast -- concrete's character
 * is in the normal map's surface tooth and in broad staining, not in albedo range,
 * and a high-contrast concrete reads as damaged rather than as concrete.
 */
ProcTexResult make_concrete(uint32_t size, uint32_t seed) {
    if (!valid_surface_size(size)) {
        return {};
    }
    debug_check_primitives();

    const procgen::Noise noise(seed);

    const PeriodicFbm broad(3, 4, 0.55f, hash_mix(seed, 0x11u), noise);
    const PeriodicFbm stain(6, 3, 0.60f, hash_mix(seed, 0x12u), noise);
    const int tooth_period = feature_period(160, 4, size, 4u);
    const PeriodicLattice tooth(tooth_period, hash_mix(seed, 0x13u), noise);

    const glm::vec3 base(0.720f, 0.712f, 0.688f);
    const glm::vec3 stain_tint(-0.055f, -0.052f, -0.040f);

    Surface surf(size);
    for (uint32_t y = 0; y < size; ++y) {
        const float v = texel_uv(y, size);
        for (uint32_t x = 0; x < size; ++x) {
            const float u = texel_uv(x, size);

            const float b = broad.sample(u, v) - 0.5f;
            const float s = smoothstepf(0.56f, 0.84f, stain.sample(u, v));
            const float t = tooth.sample(u, v) * 0.60f +
                            texel_grain(x, y, hash_mix(seed, 0x14u)) * 0.40f - 0.5f;

            glm::vec3 albedo = base + glm::vec3(b * 0.070f + t * 0.038f) + stain_tint * s;

            const float height = 0.50f + t * 0.44f + b * 0.24f - s * 0.05f;
            surf.put(x, y, albedo, height);
        }
    }

    ProcTexResult out = surf.finish(TextureFormat::RGBA8_SRGB);
    assert(is_tileable(out) && "make_concrete produced a seam");
    return out;
}

/**
 * @brief Rounded setts in mortar joints
 *
 * Invariant: tiles in both axes; the cell count is forced EVEN so the running bond
 * survives the wrap. An odd count would put two same-phase courses next to each
 * other at the tile boundary -- not a hard seam, but a repeating flaw down the
 * whole road, which is worse for being subtle.
 *
 * Everything reads off WorleySample::edge(), the distance to the boundary between
 * the two nearest stones. That single field gives the joint mask, the ambient
 * darkening around each stone, and the dome that the normal map turns into a
 * lit cobble. The albedo alone is nearly flat -- cobblestone is the case the
 * header calls out where the normal map does all the work.
 */
ProcTexResult make_cobblestone(uint32_t size, uint32_t seed, float stone_size) {
    if (!valid_surface_size(size)) {
        return {};
    }
    debug_check_primitives();
    stone_size = std::clamp(stone_size, 0.02f, 0.5f);

    int cells = static_cast<int>(std::lround(1.0f / stone_size));
    cells = feature_period(cells, 2, size, 8u);
    cells += (cells & 1);   // even: see the invariant above

    const procgen::Noise noise(seed);
    const PeriodicCells stones(cells, hash_mix(seed, 0x23u), 0.16f, true);
    const PeriodicFbm damp(3, 3, 0.55f, hash_mix(seed, 0x21u), noise);
    const int grain_period = feature_period(cells * 12, 4, size, 4u);
    const PeriodicLattice grain(grain_period, hash_mix(seed, 0x22u), noise);

    // Joint width in cell units. A 0.65 m sett with a 25 mm joint is about 0.04;
    // this is wider, because a joint narrower than about three texels at the
    // resolutions in use stops being a joint and becomes aliasing.
    constexpr float kJoint = 0.075f;

    const glm::vec3 stone_dark(0.345f, 0.332f, 0.312f);
    const glm::vec3 stone_light(0.560f, 0.545f, 0.518f);
    const glm::vec3 mortar(0.215f, 0.208f, 0.194f);

    Surface surf(size);
    for (uint32_t y = 0; y < size; ++y) {
        const float v = texel_uv(y, size);
        for (uint32_t x = 0; x < size; ++x) {
            const float u = texel_uv(x, size);

            const WorleySample w = periodic_worley(u, v, stones);
            const float edge = w.edge();

            const float stone_mask = smoothstepf(kJoint * 0.45f, kJoint * 1.70f, edge);
            const float dome = std::sqrt(smoothstepf(0.0f, 0.30f, edge));

            const float tone = unit_from_hash(w.cell);
            const float warm = unit_from_hash(hash_u32(w.cell)) - 0.5f;
            const float lift = unit_from_hash(hash_u32(w.cell ^ 0x9e3779b9u)) - 0.5f;

            glm::vec3 stone = stone_dark + (stone_light - stone_dark) * tone;
            // A small hue swing per stone, not a large one: granite setts vary in
            // VALUE far more than in hue, and a wide hue spread reads as camouflage.
            stone += glm::vec3(warm * 0.024f, warm * 0.004f, -warm * 0.020f);

            const float g = grain.sample(u, v) * 0.55f +
                            texel_grain(x, y, hash_mix(seed, 0x24u)) * 0.45f - 0.5f;
            stone += glm::vec3(g * 0.055f);

            // Contact darkening where the stone rolls away into the joint.
            stone *= 0.62f + 0.38f * smoothstepf(0.0f, 0.17f, edge);

            glm::vec3 joint = mortar + glm::vec3(g * 0.045f);

            glm::vec3 albedo = joint + (stone - joint) * stone_mask;
            albedo += glm::vec3((damp.sample(u, v) - 0.5f) * 0.045f);

            const float height = clamp01(0.08f + 0.80f * dome + lift * 0.055f + g * 0.030f);
            surf.put(x, y, albedo, height);
        }
    }

    ProcTexResult out = surf.finish(TextureFormat::RGBA8_SRGB);
    assert(is_tileable(out) && "make_cobblestone produced a seam");
    return out;
}

/**
 * @brief Rectangular paving slabs
 *
 * Invariant: tiles in both axes; the slab count per tile is a WHOLE number in each
 * axis, which is what makes the joint at u = 0 the same width as every other joint
 * instead of a half joint against the next tile's half joint.
 *
 * Joint width is expressed in TEXELS rather than in tile fractions, so a 256 and a
 * 1024 texture have the same physical joint rather than the same texel joint.
 */
ProcTexResult make_paving(uint32_t size, uint32_t seed, glm::vec2 slab_size) {
    if (!valid_surface_size(size)) {
        return {};
    }
    debug_check_primitives();

    const int max_cells = feature_period(static_cast<int>(size / 8u), 1, size, 8u);
    const int nx = std::clamp(
        static_cast<int>(std::lround(1.0f / std::clamp(slab_size.x, 0.05f, 1.0f))), 1, max_cells);
    const int ny = std::clamp(
        static_cast<int>(std::lround(1.0f / std::clamp(slab_size.y, 0.05f, 1.0f))), 1, max_cells);

    const procgen::Noise noise(seed);
    const PeriodicFbm broad(3, 3, 0.55f, hash_mix(seed, 0x31u), noise);
    const int tooth_period = feature_period(192, 4, size, 4u);
    const PeriodicLattice tooth(tooth_period, hash_mix(seed, 0x32u), noise);

    // ~15 mm on a 2 m tile at 1024, and the same physical width at any resolution.
    const float joint_px = std::max(2.0f, static_cast<float>(size) / 128.0f);
    const float cell_w_px = static_cast<float>(size) / static_cast<float>(nx);
    const float cell_h_px = static_cast<float>(size) / static_cast<float>(ny);

    const glm::vec3 slab_dark(0.585f, 0.578f, 0.558f);
    const glm::vec3 slab_light(0.745f, 0.738f, 0.715f);
    const glm::vec3 joint_colour(0.352f, 0.344f, 0.328f);

    Surface surf(size);
    for (uint32_t y = 0; y < size; ++y) {
        const float v = texel_uv(y, size);
        const float gy = v * static_cast<float>(ny);
        const int iy = static_cast<int>(std::floor(gy));
        const float fy = gy - static_cast<float>(iy);
        const float dy_px = std::min(fy, 1.0f - fy) * cell_h_px;

        for (uint32_t x = 0; x < size; ++x) {
            const float u = texel_uv(x, size);
            const float gx = u * static_cast<float>(nx);
            const int ix = static_cast<int>(std::floor(gx));
            const float fx = gx - static_cast<float>(ix);
            const float dx_px = std::min(fx, 1.0f - fx) * cell_w_px;

            const float d_px = std::min(dx_px, dy_px);

            const uint32_t slab_hash = hash2i(wrap_i(ix, nx), wrap_i(iy, ny), hash_mix(seed, 0x33u));
            const float tone = unit_from_hash(slab_hash);
            const float slab_lift = unit_from_hash(hash_u32(slab_hash)) - 0.5f;

            const float g = tooth.sample(u, v) * 0.55f +
                            texel_grain(x, y, hash_mix(seed, 0x34u)) * 0.45f - 0.5f;

            glm::vec3 slab = slab_dark + (slab_light - slab_dark) * tone;
            slab += glm::vec3(g * 0.042f + (broad.sample(u, v) - 0.5f) * 0.050f);

            const float face = smoothstepf(joint_px * 0.55f, joint_px * 1.15f, d_px);
            glm::vec3 albedo = joint_colour + (slab - joint_colour) * face;

            // A chamfer, not a step: the height ramps over about two joint widths so
            // the normal map gives each slab a lit lip instead of a hard black line.
            const float height = clamp01(0.14f
                                         + 0.70f * smoothstepf(joint_px * 0.40f, joint_px * 2.40f, d_px)
                                         + slab_lift * 0.045f
                                         + g * 0.035f);
            surf.put(x, y, albedo, height);
        }
    }

    ProcTexResult out = surf.finish(TextureFormat::RGBA8_SRGB);
    assert(is_tileable(out) && "make_paving produced a seam");
    return out;
}

/**
 * @brief Loose graded gravel
 *
 * Invariant: tiles in both axes. High cell count and high jitter -- gravel is
 * cobblestone with no courses, small stones and no mortar, so the joint mask goes
 * away and the dome does everything.
 */
ProcTexResult make_gravel(uint32_t size, uint32_t seed) {
    if (!valid_surface_size(size)) {
        return {};
    }
    debug_check_primitives();

    // ~40 mm stones on a 4 m tile, capped so a stone never falls below ~8 texels.
    int cells = feature_period(96, 4, size, 8u);
    cells += (cells & 1);

    const procgen::Noise noise(seed);
    const PeriodicCells pebbles(cells, hash_mix(seed, 0x43u), 0.45f, false);
    const PeriodicFbm damp(3, 4, 0.55f, hash_mix(seed, 0x41u), noise);
    const int grain_period = std::clamp(static_cast<int>(size / 4u), 4, 256);
    const PeriodicLattice grain(grain_period, hash_mix(seed, 0x42u), noise);

    const glm::vec3 stone_dark(0.300f, 0.283f, 0.255f);
    const glm::vec3 stone_light(0.615f, 0.588f, 0.535f);

    Surface surf(size);
    for (uint32_t y = 0; y < size; ++y) {
        const float v = texel_uv(y, size);
        for (uint32_t x = 0; x < size; ++x) {
            const float u = texel_uv(x, size);

            const WorleySample w = periodic_worley(u, v, pebbles);
            const float edge = w.edge();
            const float dome = std::sqrt(smoothstepf(0.0f, 0.26f, edge));

            const float tone = unit_from_hash(w.cell);
            const float warm = unit_from_hash(hash_u32(w.cell)) - 0.5f;

            glm::vec3 stone = stone_dark + (stone_light - stone_dark) * tone;
            stone += glm::vec3(warm * 0.055f, warm * 0.020f, -warm * 0.030f);

            const float g = grain.sample(u, v) * 0.45f +
                            texel_grain(x, y, hash_mix(seed, 0x44u)) * 0.55f - 0.5f;
            stone += glm::vec3(g * 0.070f);

            // Deep shadow in the voids between stones is most of what makes loose
            // gravel read as loose rather than as a speckled slab.
            stone *= 0.40f + 0.60f * smoothstepf(0.0f, 0.20f, edge);
            stone += glm::vec3((damp.sample(u, v) - 0.5f) * 0.045f);

            const float height = clamp01(0.05f + 0.85f * dome + g * 0.070f);
            surf.put(x, y, stone, height);
        }
    }

    ProcTexResult out = surf.finish(TextureFormat::RGBA8_SRGB);
    assert(is_tileable(out) && "make_gravel produced a seam");
    return out;
}

/**
 * @brief Compacted earth
 *
 * Invariant: tiles in both axes. No cellular layer at all: compacted dirt has no
 * grain boundaries, and adding stones to it turns it into gravel.
 */
ProcTexResult make_dirt(uint32_t size, uint32_t seed) {
    if (!valid_surface_size(size)) {
        return {};
    }
    debug_check_primitives();

    const procgen::Noise noise(seed);
    const PeriodicFbm broad(3, 5, 0.58f, hash_mix(seed, 0x51u), noise);
    const PeriodicFbm damp(2, 3, 0.55f, hash_mix(seed, 0x52u), noise);
    const int grain_period = feature_period(128, 4, size, 4u);
    const PeriodicLattice grain(grain_period, hash_mix(seed, 0x53u), noise);

    const glm::vec3 dry(0.412f, 0.330f, 0.238f);
    const glm::vec3 wet(0.252f, 0.192f, 0.135f);

    Surface surf(size);
    for (uint32_t y = 0; y < size; ++y) {
        const float v = texel_uv(y, size);
        for (uint32_t x = 0; x < size; ++x) {
            const float u = texel_uv(x, size);

            const float b = broad.sample(u, v);
            const float d = smoothstepf(0.35f, 0.75f, damp.sample(u, v));
            const float g = grain.sample(u, v) * 0.50f +
                            texel_grain(x, y, hash_mix(seed, 0x54u)) * 0.50f - 0.5f;

            glm::vec3 albedo = wet + (dry - wet) * clamp01(b * 0.75f + 0.25f - d * 0.35f);
            albedo += glm::vec3(g * 0.038f);

            const float height = clamp01(0.46f + (b - 0.5f) * 0.62f + g * 0.14f);
            surf.put(x, y, albedo, height);
        }
    }

    ProcTexResult out = surf.finish(TextureFormat::RGBA8_SRGB);
    assert(is_tileable(out) && "make_dirt produced a seam");
    return out;
}

/**
 * @brief Verge and median planting
 *
 * Invariant: tiles in both axes. Clumps, not blades -- see the header. The
 * highest-frequency layer is deliberately kept above the texel size so the grass
 * does not shimmer when the camera moves, which is the failure mode of drawing
 * individual blades at this scale.
 */
ProcTexResult make_grass(uint32_t size, uint32_t seed) {
    if (!valid_surface_size(size)) {
        return {};
    }
    debug_check_primitives();

    const procgen::Noise noise(seed);
    const PeriodicFbm patches(2, 4, 0.60f, hash_mix(seed, 0x61u), noise);
    const PeriodicFbm clumps(9, 3, 0.52f, hash_mix(seed, 0x62u), noise);
    const int fine_period = std::clamp(static_cast<int>(size / 8u), 4, 96);
    const PeriodicLattice fine(fine_period, hash_mix(seed, 0x63u), noise);

    const glm::vec3 lush(0.152f, 0.302f, 0.108f);
    const glm::vec3 mid(0.255f, 0.372f, 0.158f);
    const glm::vec3 dry(0.442f, 0.436f, 0.222f);

    Surface surf(size);
    for (uint32_t y = 0; y < size; ++y) {
        const float v = texel_uv(y, size);
        for (uint32_t x = 0; x < size; ++x) {
            const float u = texel_uv(x, size);

            const float p = patches.sample(u, v);
            const float c = clumps.sample(u, v);
            const float f = fine.sample(u, v) * 0.72f +
                            texel_grain(x, y, hash_mix(seed, 0x64u)) * 0.28f - 0.5f;

            // Two-stop ramp: lush through mid to dry, so a verge has both a healthy
            // green and a scorched edge rather than one flat colour plus noise.
            const float t = clamp01(p * 0.70f + c * 0.30f);
            glm::vec3 albedo = t < 0.5f ? lush + (mid - lush) * (t * 2.0f)
                                        : mid + (dry - mid) * ((t - 0.5f) * 2.0f);
            albedo += glm::vec3(f * 0.060f * 0.7f, f * 0.075f, f * 0.040f);

            // Clumps sit proud; the patch field tilts whole areas of the verge.
            const float height = clamp01(0.42f + (c - 0.5f) * 0.70f + (p - 0.5f) * 0.28f +
                                         f * 0.16f);
            surf.put(x, y, albedo, height);
        }
    }

    ProcTexResult out = surf.finish(TextureFormat::RGBA8_SRGB);
    assert(is_tileable(out) && "make_grass produced a seam");
    return out;
}

/**
 * @brief Kerb stone, face on the left half of U and top on the right half
 *
 * Invariant: tiles in V only. is_tileable(out, false, true) is the correct
 * assertion and the header says why: the two halves in U are two different
 * surfaces, and nothing in the corridor builder tiles a kerb laterally.
 *
 * Layout, matching the header:
 *
 * ```
 *   u = 0.0        u = 0.5        u = 1.0
 *   |  FACE ------> |  TOP ------> |
 *   gutter    chamfer/nose     back edge
 * ```
 *
 * U runs UP the face over [0, 0.5) -- so u = 0 is where the kerb meets the road,
 * which is where the gutter grime is -- and laterally across the top over
 * [0.5, 1). The chamfer sits exactly on the join at u = 0.5.
 *
 * Transverse joints at v = 0.0 and v = 0.5 give two kerb stones per tile. At the
 * Curb material's tile_v of 2 m that is a 1 m kerb stone, which is the standard
 * unit, and both joints land on tile fractions that wrap exactly.
 */
ProcTexResult make_kerb(uint32_t size, uint32_t seed) {
    if (!valid_surface_size(size)) {
        return {};
    }
    debug_check_primitives();

    const procgen::Noise noise(seed);
    const PeriodicFbm broad(4, 4, 0.55f, hash_mix(seed, 0x71u), noise);
    const PeriodicFbm scuff(8, 3, 0.60f, hash_mix(seed, 0x72u), noise);
    const int tooth_period = feature_period(176, 4, size, 4u);
    const PeriodicLattice tooth(tooth_period, hash_mix(seed, 0x73u), noise);

    const glm::vec3 face_base(0.735f, 0.728f, 0.706f);
    const glm::vec3 top_base(0.672f, 0.664f, 0.640f);
    const glm::vec3 grime_tint(-0.150f, -0.140f, -0.118f);

    const float joint_px = std::max(2.0f, static_cast<float>(size) / 256.0f);

    Surface surf(size);
    for (uint32_t y = 0; y < size; ++y) {
        const float v = texel_uv(y, size);

        // Two joints per tile, at v = 0 and v = 0.5, measured with a wrapped
        // distance so the one at v = 0 is a whole joint and not two halves.
        const float jt = v * 2.0f - std::floor(v * 2.0f);
        const float joint_d_px = std::min(jt, 1.0f - jt) * 0.5f * static_cast<float>(size);
        const float joint_face = smoothstepf(joint_px * 0.55f, joint_px * 1.30f, joint_d_px);

        for (uint32_t x = 0; x < size; ++x) {
            const float u = texel_uv(x, size);
            const bool on_face = u < 0.5f;
            const float half_u = on_face ? (u * 2.0f) : ((u - 0.5f) * 2.0f);

            const float g = tooth.sample(u, v) * 0.58f +
                            texel_grain(x, y, hash_mix(seed, 0x74u)) * 0.42f - 0.5f;
            const float b = broad.sample(u, v) - 0.5f;

            // The chamfer straddles u = 0.5: a narrow raised nose, brighter because
            // it is the edge that gets swept clean and catches the sky.
            const float chamfer = 1.0f - smoothstepf(0.012f, 0.048f, std::fabs(u - 0.5f));

            glm::vec3 albedo;
            float height;

            if (on_face) {
                // half_u = 0 at the gutter, 1 at the top of the face.
                const float grime = 1.0f - smoothstepf(0.02f, 0.42f, half_u);
                albedo = face_base + grime_tint * grime;
                albedo += glm::vec3(g * 0.045f + b * 0.045f);
                height = 0.58f + g * 0.30f + b * 0.10f;
            } else {
                const float scuffing = smoothstepf(0.45f, 0.85f, scuff.sample(u, v));
                albedo = top_base + glm::vec3(-0.055f * scuffing);
                albedo += glm::vec3(g * 0.050f + b * 0.055f);
                // The back edge of the top, at half_u -> 1, meets the footway and
                // collects dirt the same way the gutter does.
                const float back = smoothstepf(0.72f, 1.0f, half_u);
                albedo += grime_tint * (back * 0.45f);
                height = 0.54f + g * 0.34f + b * 0.12f - back * 0.05f;
            }

            albedo += glm::vec3(0.055f * chamfer);
            height += 0.16f * chamfer;

            // The joint cuts through both halves: a kerb stone's end face runs from
            // the gutter to the back of the top in one line.
            albedo = albedo * (0.58f + 0.42f * joint_face);
            height = height * (0.30f + 0.70f * joint_face);

            surf.put(x, y, albedo, clamp01(height));
        }
    }

    ProcTexResult out = surf.finish(TextureFormat::RGBA8_SRGB);
    assert(is_tileable(out, false, true) && "make_kerb produced a seam along V");
    return out;
}

// ============================================================================
// Derived maps
// ============================================================================

/**
 * @brief Tangent-space normal map from a height field
 *
 * ### The green channel, stated rather than assumed
 *
 * The encoded normal is
 *
 * ```
 *   N = normalize(vec3(-dh/du, -dh/dv, 1))      encoded as N * 0.5 + 0.5
 * ```
 *
 * where v is the texture's second axis and INCREASES WITH THE ROW INDEX, because
 * row 0 of the buffer is v = 0. So +G lies along +V, which is the direction of
 * mesh_pbr.vert's `frag_bitangent = cross(frag_normal, frag_tangent) * in_tangent.w`,
 * and that is exactly what the header requires.
 *
 * Named conventions, since the two differ by precisely this sign: this is the
 * OPENGL convention in ENGINE terms -- green along the +V bitangent. If one of
 * these files is opened in a tool that assumes v = 0 is the BOTTOM row, it will
 * look like a DirectX / "Y-down" map, because the row order is flipped relative to
 * that tool's assumption, not because the sign is. The engine's basis is the
 * authority and the engine is consistent.
 *
 * Sanity check on a bump: height rising with v means dh/dv > 0, so N.y < 0 and
 * G < 128 on the near slope of a ridge -- the surface tilts back toward smaller v,
 * which is correct.
 *
 * ### Why the gradient wraps
 *
 * Texel 0's left neighbour is texel width - 1. Clamping instead would flatten a
 * one-texel border to (128, 128, 255), and since every surface here is tiled
 * across a whole road network, that border becomes a hard lit line every few
 * metres in both axes. That is the seam the tileability contract exists to
 * prevent, arriving through the back door of the derived map.
 */
ProcTexResult make_normal_from_height(const ProcTexResult& height, float strength) {
    if (!height.is_valid()) {
        return {};
    }

    int stride = 0;
    int offset = 0;
    switch (height.desc.format) {
        case TextureFormat::RGBA8:
        case TextureFormat::RGBA8_SRGB: stride = 4; offset = 3; break;
        case TextureFormat::R8:         stride = 1; offset = 0; break;
        case TextureFormat::BC7:
        case TextureFormat::BC7_SRGB:
        case TextureFormat::BC5_Normal: return {};
    }

    const int w = static_cast<int>(height.desc.width);
    const int h = static_cast<int>(height.desc.height);
    if (w < 2 || h < 2) {
        return {};
    }
    strength = std::max(0.0f, strength);

    const uint8_t* src = height.pixels.data();
    const auto at = [&](int x, int y) -> float {
        const int wx = wrap_i(x, w);
        const int wy = wrap_i(y, h);
        const size_t idx = (static_cast<size_t>(wy) * static_cast<size_t>(w) +
                            static_cast<size_t>(wx)) * static_cast<size_t>(stride) +
                           static_cast<size_t>(offset);
        return static_cast<float>(src[idx]) * (1.0f / 255.0f);
    };

    // Height 1.0 stands for this much displacement in UV units. Physically a road
    // surface is far flatter than this -- a cobble crown is under a centimetre on a
    // tile several metres across -- and a physically honest value produces a normal
    // map that is indistinguishable from flat at editor camera distances. This is a
    // deliberate exaggeration of roughly an order of magnitude, and `strength` is
    // the knob for anyone who disagrees.
    constexpr float kUvDisplacementPerUnitHeight = 0.020f;

    const float sx = strength * kUvDisplacementPerUnitHeight * static_cast<float>(w) / 8.0f;
    const float sy = strength * kUvDisplacementPerUnitHeight * static_cast<float>(h) / 8.0f;

    ProcTexResult out;
    out.desc.width = height.desc.width;
    out.desc.height = height.desc.height;
    out.desc.layers = 1;
    out.desc.mip_levels = height.desc.mip_levels;
    out.desc.format = TextureFormat::RGBA8;     // LINEAR, never sRGB
    out.pixels.resize(out.desc.level0_bytes());

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            const float h00 = at(x - 1, y - 1), h10 = at(x, y - 1), h20 = at(x + 1, y - 1);
            const float h01 = at(x - 1, y),                          h21 = at(x + 1, y);
            const float h02 = at(x - 1, y + 1), h12 = at(x, y + 1), h22 = at(x + 1, y + 1);

            // Sobel. The 1/8 is folded into sx/sy above.
            const float gx = (h20 + 2.0f * h21 + h22) - (h00 + 2.0f * h01 + h02);
            const float gy = (h02 + 2.0f * h12 + h22) - (h00 + 2.0f * h10 + h20);

            const glm::vec3 n = glm::normalize(glm::vec3(-gx * sx, -gy * sy, 1.0f));

            uint8_t* d = &out.pixels[(static_cast<size_t>(y) * static_cast<size_t>(w) +
                                      static_cast<size_t>(x)) * 4u];
            d[0] = to_u8(n.x * 0.5f + 0.5f);
            d[1] = to_u8(n.y * 0.5f + 0.5f);
            d[2] = to_u8(n.z * 0.5f + 0.5f);
            d[3] = 255u;
        }
    }

    return out;
}

/**
 * @brief A uniform ORM pack
 *
 * Invariant: r = occlusion, g = roughness, b = metallic, a = 255, LINEAR, and the
 * same value in every texel. mip_levels is 1: a constant texture gains nothing
 * from a chain, and asking for one would make the uploader run a mip generation
 * pass to produce identical data.
 */
ProcTexResult make_orm(float ao, float roughness, float metallic, uint32_t size) {
    if (size < 1u || !is_pow2(size)) {
        return {};
    }

    ProcTexResult out;
    out.desc.width = size;
    out.desc.height = size;
    out.desc.layers = 1;
    out.desc.mip_levels = 1;
    out.desc.format = TextureFormat::RGBA8;
    out.pixels.resize(out.desc.level0_bytes());

    const uint8_t r = to_u8(ao);
    const uint8_t g = to_u8(roughness);
    const uint8_t b = to_u8(metallic);

    for (size_t i = 0; i < out.pixels.size(); i += 4u) {
        out.pixels[i + 0] = r;
        out.pixels[i + 1] = g;
        out.pixels[i + 2] = b;
        out.pixels[i + 3] = 255u;
    }
    return out;
}

// ============================================================================
// The markings atlas
// ============================================================================

namespace {

using osm::road::MarkingSprite;
using osm::road::SpriteRect;

/**
 * @brief Transparent margin left inside a block, in ATLAS pixels
 *
 * On top of marking_atlas.hpp's kAtlasInsetPixels, which protects level 0 only. A
 * mip level n texel averages 2^n source texels, so one pixel of inset stops
 * bilinear filtering reaching a neighbour and does nothing at all about mip
 * filtering doing the same. Three more pixels of unpainted margin keeps the art
 * clear of its block edge through the first few levels, which is as far down the
 * chain as a marking is ever legible.
 *
 * Sprites that must read as CONTINUOUS when repeated get no margin at all -- see
 * sprite_bleeds_to_edge().
 */
constexpr float kBleedMarginPixels = 3.0f;

/// Paint colours, from the header's rule
const glm::vec3 kPaintWhite(0.95f, 0.95f, 0.93f);
const glm::vec3 kPaintYellow(0.90f, 0.72f, 0.12f);

// ---------------------------------------------------------------------------
// Signed distance primitives
//
// All of these operate in the sprite's own aspect-corrected space: x is the
// normalised block width and y has been multiplied by the block's aspect ratio, so
// distances are isotropic and one unit is one block width. The atlas layout makes
// that space metric as well -- every SDF-drawn sprite's block aspect equals its
// sprite_size() aspect -- so a radius of 0.15 here is 0.15 block widths of real
// painted road in both axes.
// ---------------------------------------------------------------------------

inline float sd_box(glm::vec2 p, glm::vec2 half) {
    const glm::vec2 d = glm::abs(p) - half;
    return glm::length(glm::max(d, glm::vec2(0.0f))) + std::min(std::max(d.x, d.y), 0.0f);
}

inline float sd_round_box(glm::vec2 p, glm::vec2 half, float r) {
    return sd_box(p, glm::max(half - glm::vec2(r), glm::vec2(0.0f))) - r;
}

inline float sd_segment(glm::vec2 p, glm::vec2 a, glm::vec2 b) {
    const glm::vec2 pa = p - a;
    const glm::vec2 ba = b - a;
    const float dd = glm::dot(ba, ba);
    const float t = dd > 0.0f ? clamp01(glm::dot(pa, ba) / dd) : 0.0f;
    return glm::length(pa - ba * t);
}

inline float sd_capsule(glm::vec2 p, glm::vec2 a, glm::vec2 b, float r) {
    return sd_segment(p, a, b) - r;
}

/// Inigo Quilez's exact triangle SDF. Winding-agnostic, which matters because this
/// file's y axis points DOWN and every triangle here is therefore wound the
/// opposite way from the usual maths convention.
float sd_triangle(glm::vec2 p, glm::vec2 p0, glm::vec2 p1, glm::vec2 p2) {
    const glm::vec2 e0 = p1 - p0, e1 = p2 - p1, e2 = p0 - p2;
    const glm::vec2 v0 = p - p0, v1 = p - p1, v2 = p - p2;

    const glm::vec2 pq0 = v0 - e0 * clamp01(glm::dot(v0, e0) / glm::dot(e0, e0));
    const glm::vec2 pq1 = v1 - e1 * clamp01(glm::dot(v1, e1) / glm::dot(e1, e1));
    const glm::vec2 pq2 = v2 - e2 * clamp01(glm::dot(v2, e2) / glm::dot(e2, e2));

    const float s = (e0.x * e2.y - e0.y * e2.x) > 0.0f ? 1.0f : -1.0f;

    const glm::vec2 d = glm::min(glm::min(glm::vec2(glm::dot(pq0, pq0), s * (v0.x * e0.y - v0.y * e0.x)),
                                          glm::vec2(glm::dot(pq1, pq1), s * (v1.x * e1.y - v1.y * e1.x))),
                                 glm::vec2(glm::dot(pq2, pq2), s * (v2.x * e2.y - v2.y * e2.x)));

    return -std::sqrt(d.x) * (d.y < 0.0f ? -1.0f : 1.0f);
}

/// Ellipse whose x radius is @p rx and y radius @p ry, measured along x
inline float sd_ellipse_x(glm::vec2 p, glm::vec2 c, float rx, float ry) {
    const glm::vec2 q(p.x - c.x, (p.y - c.y) * (ry > 0.0f ? rx / ry : 1.0f));
    return glm::length(q) - rx;
}

// ---------------------------------------------------------------------------
// Sprite shapes
//
// ORIENTATION, from marking_atlas.hpp and repeated here because getting it wrong
// is silent: y = 0 is the DOWNSTREAM (forward) edge, y = 1 the UPSTREAM edge, and
// x = 0 is LEFT of the direction of travel. An arrow's tip is therefore at small
// y, and ArrowLeft's head points toward small x.
// ---------------------------------------------------------------------------

/// ArrowStraight: a stem running back from a head whose tip touches the top edge
float sdf_arrow_straight(glm::vec2 p, float a) {
    const auto Q = [a](float x, float y) { return glm::vec2(x, y * a); };
    const float head = sd_triangle(p, Q(0.50f, 0.020f), Q(0.980f, 0.300f), Q(0.020f, 0.300f));
    const float stem = sd_box(p - Q(0.50f, 0.640f), glm::vec2(0.150f, 0.360f * a));
    return std::min(head, stem);
}

/**
 * @brief ArrowLeft: stem up the right of the block, head pointing at small x
 *
 * The head is WIDE and SHORT -- 0.90 block widths across the road against 0.62
 * along it -- which looks wrong on screen and is right on the ground. A lane arrow
 * is stretched along the carriageway so it reads correctly to a driver seeing it
 * at a shallow angle, and its 1.4 m block simply has no room for a proportionate
 * sideways head.
 *
 * ArrowRight is this function with x mirrored, so the two cannot drift apart.
 */
float sdf_arrow_left(glm::vec2 p, float a) {
    const auto Q = [a](float x, float y) { return glm::vec2(x, y * a); };
    // The head's base is at x = 0.80, PAST the stem's right edge at 0.765. A base
    // that stops short of it leaves the stem's top-left corner sticking out above
    // the head as a notch, which is the one detail that makes a turn arrow read as
    // a broken shape rather than as an arrow.
    const float stem = sd_box(p - Q(0.620f, 0.670f), glm::vec2(0.145f, 0.330f * a));
    const float head = sd_triangle(p, Q(0.040f, 0.260f), Q(0.800f, 0.100f), Q(0.800f, 0.420f));
    // The stem's top at y = 0.34 sits ABOVE the head's lower edge where the two
    // meet (y = 0.352 at the stem's left edge), so the union closes with no gap and
    // needs no elbow patch -- a patch capsule wide enough to close the gap bulges
    // out past the stem instead, and trades a notch for a lump.
    return std::min(stem, head);
}

/// ArrowStraightLeft: the straight head at the top, the branch head to the left
float sdf_arrow_straight_left(glm::vec2 p, float a) {
    const auto Q = [a](float x, float y) { return glm::vec2(x, y * a); };
    const float stem      = sd_box(p - Q(0.640f, 0.620f), glm::vec2(0.135f, 0.380f * a));
    const float head_up   = sd_triangle(p, Q(0.640f, 0.020f), Q(0.965f, 0.270f), Q(0.315f, 0.270f));
    const float branch    = sd_capsule(p, Q(0.640f, 0.620f), Q(0.500f, 0.500f), 0.125f);
    const float head_left = sd_triangle(p, Q(0.040f, 0.470f), Q(0.580f, 0.345f), Q(0.580f, 0.595f));
    return std::min(std::min(stem, head_up), std::min(branch, head_left));
}

/// ArrowUTurn: a stem, a half-ellipse hook over the top, and a head pointing back
float sdf_arrow_u_turn(glm::vec2 p, float a) {
    const auto Q = [a](float x, float y) { return glm::vec2(x, y * a); };
    constexpr float kPi = 3.14159265358979323846f;

    float d = sd_box(p - Q(0.740f, 0.660f), glm::vec2(0.125f, 0.340f * a));

    // Half ellipse from (0.74, 0.32) over (0.52, 0.12) to (0.30, 0.32), as capsules.
    constexpr int kSteps = 10;
    glm::vec2 prev = Q(0.740f, 0.320f);
    for (int i = 1; i <= kSteps; ++i) {
        const float th = kPi * static_cast<float>(i) / static_cast<float>(kSteps);
        const glm::vec2 cur = Q(0.520f + 0.220f * std::cos(th), 0.320f - 0.200f * std::sin(th));
        d = std::min(d, sd_capsule(p, prev, cur, 0.125f));
        prev = cur;
    }

    d = std::min(d, sd_capsule(p, Q(0.300f, 0.320f), Q(0.300f, 0.400f), 0.125f));
    d = std::min(d, sd_triangle(p, Q(0.300f, 0.580f), Q(0.100f, 0.400f), Q(0.500f, 0.400f)));
    return d;
}

/// GiveWayTriangles: ONE triangle, base at the DOWNSTREAM edge, apex pointing back
float sdf_give_way(glm::vec2 p, float a) {
    const auto Q = [a](float x, float y) { return glm::vec2(x, y * a); };
    return sd_triangle(p, Q(0.080f, 0.060f), Q(0.920f, 0.060f), Q(0.500f, 0.940f));
}

/**
 * @brief BoxJunctionHatch: 45 degree bands that meet the block edge on all four sides
 *
 * The band phase is frac((x + y) * n) with n a whole number, so translating by one
 * block in either axis changes the argument by exactly n and leaves the phase
 * untouched. Adjacent quads therefore continue each other's lines with no
 * bookkeeping in the emitter, which is what the atlas header asks of this sprite.
 */
float sdf_box_hatch(glm::vec2 p, float a) {
    constexpr float kBands = 2.0f;              // two diagonals per 2 m cell
    constexpr float kHalfWidth = 0.0375f;       // ~0.15 m line on a 2 m cell
    const float ny = a > 0.0f ? p.y / a : p.y;  // back to normalised, not aspect space
    const float t = (p.x + ny) * kBands;
    const float f = t - std::floor(t) - 0.5f;
    // |gradient| of (x + y) * kBands is kBands * sqrt(2); divide it out to get a
    // real perpendicular distance rather than a phase.
    const float d = std::fabs(f) / (kBands * 1.41421356f);
    return d - kHalfWidth;
}

/**
 * @brief BikeSymbol: a bicycle in profile, stretched along the carriageway
 *
 * Drawn in metres -- the block is 1.0 x 2.0 m and its aspect matches, so the
 * aspect-corrected space IS metres here. The bicycle's own "up" (saddle,
 * handlebars) points ACROSS the road toward x = 0, and its length runs along the
 * road with the front wheel DOWNSTREAM at small y, which is how a cycle-lane
 * pictogram is painted.
 *
 * The wheels are ellipses rather than circles because the whole pictogram is
 * stretched: a 1.7 x 0.7 m profile drawn into a 1.0 x 2.0 m footprint.
 */
float sdf_bike(glm::vec2 p, float a) {
    const auto M = [a](float x_m, float y_m) { return glm::vec2(x_m, (y_m / 2.0f) * a); };

    // Metres. The sprite is 1.0 m across the road by 2.0 m along it; the bicycle is
    // a 1.7 x 0.7 m profile drawn into that, so it is stretched along the road,
    // which is how a cycle pictogram is painted.
    const glm::vec2 front_hub = M(0.58f, 0.50f);
    const glm::vec2 rear_hub  = M(0.58f, 1.50f);
    const glm::vec2 bracket   = M(0.50f, 1.15f);
    const glm::vec2 saddle    = M(0.18f, 1.32f);
    const glm::vec2 head      = M(0.22f, 0.80f);

    // Wheels: 0.68 m across the road, 0.34 m along it after the stretch.
    const float wheel_rx = 0.34f;
    const float wheel_ry = 0.17f * a;
    const float ring = 0.055f;

    float d = std::fabs(sd_ellipse_x(p, front_hub, wheel_rx, wheel_ry)) - ring;
    d = std::min(d, std::fabs(sd_ellipse_x(p, rear_hub, wheel_rx, wheel_ry)) - ring);

    constexpr float kTube = 0.038f;
    d = std::min(d, sd_capsule(p, rear_hub, bracket, kTube));
    d = std::min(d, sd_capsule(p, bracket, saddle, kTube));
    d = std::min(d, sd_capsule(p, bracket, head, kTube));
    d = std::min(d, sd_capsule(p, saddle, head, kTube));
    d = std::min(d, sd_capsule(p, saddle, rear_hub, kTube));
    d = std::min(d, sd_capsule(p, head, front_hub, kTube));

    // Handlebar and saddle, the two blobs that stop it reading as a pair of hoops.
    d = std::min(d, sd_capsule(p, M(0.08f, 0.86f), M(0.30f, 0.74f), 0.040f));
    d = std::min(d, sd_capsule(p, M(0.08f, 1.36f), M(0.26f, 1.30f), 0.048f));
    return d;
}

/**
 * @brief BusSymbol: a bus in plan, nose DOWNSTREAM
 *
 * An outline rather than a filled slab: a filled 1 x 2 m rectangle of paint is
 * indistinguishable from a patch repair, while an outline with a windscreen line
 * and wheels reads as a vehicle at the distance this is seen from.
 */
float sdf_bus(glm::vec2 p, float a) {
    const auto M = [a](float x_m, float y_m) { return glm::vec2(x_m, (y_m / 2.0f) * a); };

    const glm::vec2 body_c = M(0.50f, 1.00f);
    const glm::vec2 body_h(0.360f, (0.880f / 2.0f) * a);

    float d = std::fabs(sd_round_box(p - body_c, body_h, 0.140f)) - 0.075f;

    // Windscreen and two transverse body lines.
    d = std::min(d, sd_capsule(p, M(0.20f, 0.44f), M(0.80f, 0.44f), 0.045f));
    d = std::min(d, sd_capsule(p, M(0.22f, 1.04f), M(0.78f, 1.04f), 0.035f));
    d = std::min(d, sd_capsule(p, M(0.22f, 1.60f), M(0.78f, 1.60f), 0.035f));

    // Four wheels, standing proud of the body sides.
    const glm::vec2 wheel_h(0.060f, (0.130f / 2.0f) * a);
    d = std::min(d, sd_box(p - M(0.10f, 0.62f), wheel_h));
    d = std::min(d, sd_box(p - M(0.90f, 0.62f), wheel_h));
    d = std::min(d, sd_box(p - M(0.10f, 1.58f), wheel_h));
    d = std::min(d, sd_box(p - M(0.90f, 1.58f), wheel_h));
    return d;
}

// ---------------------------------------------------------------------------
// Sprite dispatch
// ---------------------------------------------------------------------------

/**
 * @brief Does this sprite's paint run to the edge of its block?
 *
 * True for everything that must read as CONTINUOUS when its quads are repeated or
 * stretched, which is the atlas header's list plus the dash sprites: a dash quad
 * IS the dash, so its ends and sides are the paint's ends and sides and there is
 * nothing to leave a margin for. False for the pictograms and the give-way
 * triangle, which sit inside their quad with road showing around them.
 */
bool sprite_bleeds_to_edge(MarkingSprite s) {
    switch (s) {
        case MarkingSprite::DashWhite:
        case MarkingSprite::DashLongWhite:
        case MarkingSprite::SolidWhite:
        case MarkingSprite::SolidYellow:
        case MarkingSprite::DoubleSolidYellow:
        case MarkingSprite::DashedYellow:
        case MarkingSprite::StopLine:
        case MarkingSprite::ZebraStripe:
        case MarkingSprite::BoxJunctionHatch:
            return true;
        default:
            return false;
    }
}

const glm::vec3& sprite_colour(MarkingSprite s) {
    switch (s) {
        case MarkingSprite::SolidYellow:
        case MarkingSprite::DashedYellow:
        case MarkingSprite::DoubleSolidYellow:
            return kPaintYellow;
        default:
            return kPaintWhite;
    }
}

/**
 * @brief Paint coverage of one sprite at a normalised point in its art rect
 *
 * @param s     Sprite
 * @param n     Normalised art-rect coordinate. May fall outside [0, 1] in the
 *              transparent margin, which the distance fields handle without
 *              clamping.
 * @param a     Aspect of the art rect, height over width
 * @param w_px  Art-rect width in texels, used to turn a distance into pixels
 * @return Coverage in [0, 1], straight (non-premultiplied)
 */
float sprite_coverage(MarkingSprite s, glm::vec2 n, float a, float w_px) {
    switch (s) {
        // Solid fills of the whole block. The quad is the marking, so its edges are
        // the paint's edges.
        case MarkingSprite::DashWhite:
        case MarkingSprite::DashLongWhite:
        case MarkingSprite::SolidWhite:
        case MarkingSprite::SolidYellow:
        case MarkingSprite::DashedYellow:
        case MarkingSprite::StopLine:
        case MarkingSprite::ZebraStripe:
            return 1.0f;

        // Two 0.10 m lines with a 0.15 m gap in a 0.35 m sprite. Runs to the block
        // edge in y, so a run of these quads is one continuous double line.
        case MarkingSprite::DoubleSolidYellow: {
            constexpr float kInnerLeft  = 0.10f / 0.35f;
            constexpr float kInnerRight = 0.25f / 0.35f;
            const float left  = (kInnerLeft - n.x) * w_px;
            const float right = (n.x - kInnerRight) * w_px;
            return clamp01(std::max(left, right) + 0.5f);
        }

        default:
            break;
    }

    const glm::vec2 p(n.x, n.y * a);
    float d = 1e9f;
    switch (s) {
        case MarkingSprite::GiveWayTriangles:   d = sdf_give_way(p, a); break;
        case MarkingSprite::BoxJunctionHatch:   d = sdf_box_hatch(p, a); break;
        case MarkingSprite::ArrowStraight:      d = sdf_arrow_straight(p, a); break;
        case MarkingSprite::ArrowLeft:          d = sdf_arrow_left(p, a); break;
        case MarkingSprite::ArrowRight:
            d = sdf_arrow_left(glm::vec2(1.0f - p.x, p.y), a);
            break;
        case MarkingSprite::ArrowStraightLeft:  d = sdf_arrow_straight_left(p, a); break;
        case MarkingSprite::ArrowStraightRight:
            d = sdf_arrow_straight_left(glm::vec2(1.0f - p.x, p.y), a);
            break;
        case MarkingSprite::ArrowUTurn:         d = sdf_arrow_u_turn(p, a); break;
        case MarkingSprite::BikeSymbol:         d = sdf_bike(p, a); break;
        case MarkingSprite::BusSymbol:          d = sdf_bus(p, a); break;
        default:                                return 0.0f;
    }

    // Analytic coverage from the distance field: one evaluation per texel and a
    // smoother edge than supersampling gives for the same cost.
    return clamp01(0.5f - d * w_px);
}

/**
 * @brief Push paint colour into the transparent texels next to paint
 *
 * Alpha is untouched -- these texels stay fully transparent, exactly as the atlas
 * header requires. What changes is their RGB, which stops being 0 and starts being
 * the neighbouring paint's colour.
 *
 * This matters because mip generation averages RGB and alpha independently. A
 * half-covered texel at mip 2 whose transparent contributors carry RGB 0 comes out
 * with the right alpha and a colour pulled toward black, so every marking gets a
 * dark fringe as it recedes. Giving the transparent side of the boundary the paint
 * colour makes that average colour-neutral. Nothing samples these texels directly,
 * so nothing else can see the difference.
 */
void dilate_paint_colour(std::vector<uint8_t>& px, uint32_t size, int passes) {
    const int w = static_cast<int>(size);
    std::vector<uint8_t> has_colour(static_cast<size_t>(w) * static_cast<size_t>(w), 0u);
    for (size_t i = 0; i < has_colour.size(); ++i) {
        has_colour[i] = px[i * 4u + 3u] > 0u ? 1u : 0u;
    }

    for (int pass = 0; pass < passes; ++pass) {
        std::vector<uint8_t> next = has_colour;
        for (int y = 0; y < w; ++y) {
            for (int x = 0; x < w; ++x) {
                const size_t idx = static_cast<size_t>(y) * static_cast<size_t>(w) +
                                   static_cast<size_t>(x);
                if (has_colour[idx] != 0u) {
                    continue;
                }
                int r = 0, g = 0, b = 0, count = 0;
                const int dx[4] = {-1, 1, 0, 0};
                const int dy[4] = {0, 0, -1, 1};
                for (int k = 0; k < 4; ++k) {
                    const int nx2 = x + dx[k];
                    const int ny2 = y + dy[k];
                    if (nx2 < 0 || ny2 < 0 || nx2 >= w || ny2 >= w) {
                        continue;
                    }
                    const size_t n_idx = static_cast<size_t>(ny2) * static_cast<size_t>(w) +
                                         static_cast<size_t>(nx2);
                    if (has_colour[n_idx] == 0u) {
                        continue;
                    }
                    r += px[n_idx * 4u + 0u];
                    g += px[n_idx * 4u + 1u];
                    b += px[n_idx * 4u + 2u];
                    ++count;
                }
                if (count == 0) {
                    continue;
                }
                px[idx * 4u + 0u] = static_cast<uint8_t>(r / count);
                px[idx * 4u + 1u] = static_cast<uint8_t>(g / count);
                px[idx * 4u + 2u] = static_cast<uint8_t>(b / count);
                // Alpha deliberately untouched: still 0, still transparent.
                next[idx] = 1u;
            }
        }
        has_colour.swap(next);
    }
}

} // namespace

/**
 * @brief Draw the marking sprite table into one atlas
 *
 * Invariant: every block is derived from osm::road::sprite_rect() and the inset it
 * applies is undone here, so the painted block and the sampled rect agree by
 * construction. Nothing in this function knows the pixel layout in
 * marking_atlas.hpp, and changing that layout changes this output with no edit
 * here.
 */
ProcTexResult make_markings_atlas(uint32_t size) {
    if (size < 256u || !is_pow2(size)) {
        return {};
    }

    std::vector<uint8_t> px(static_cast<size_t>(size) * static_cast<size_t>(size) * 4u, 0u);

    const float scale = static_cast<float>(size) / static_cast<float>(osm::road::kAtlasSizePixels);
    const float fsize = static_cast<float>(size);
    const int isize = static_cast<int>(size);

    for (uint8_t i = 0; i < static_cast<uint8_t>(MarkingSprite::Count); ++i) {
        const MarkingSprite sprite = static_cast<MarkingSprite>(i);
        const SpriteRect r = osm::road::sprite_rect(sprite);
        if (r.du() <= 0.0f || r.dv() <= 0.0f) {
            continue;
        }

        // Undo sprite_rect()'s inset to recover the block an artist would paint.
        const float inset = osm::road::kAtlasInsetPixels * scale;
        const float bx0 = r.u0 * fsize - inset;
        const float by0 = r.v0 * fsize - inset;
        const float bx1 = r.u1 * fsize + inset;
        const float by1 = r.v1 * fsize + inset;

        const float margin = sprite_bleeds_to_edge(sprite)
                                 ? 0.0f
                                 : (osm::road::kAtlasInsetPixels + kBleedMarginPixels) * scale;

        const float ax0 = bx0 + margin;
        const float ay0 = by0 + margin;
        const float aw = (bx1 - margin) - ax0;
        const float ah = (by1 - margin) - ay0;
        if (aw <= 0.0f || ah <= 0.0f) {
            continue;
        }

        const float aspect = ah / aw;
        const glm::vec3& colour = sprite_colour(sprite);
        const uint8_t cr = to_u8(colour.r);
        const uint8_t cg = to_u8(colour.g);
        const uint8_t cb = to_u8(colour.b);

        const int y_begin = std::max(0, static_cast<int>(std::floor(by0)));
        const int y_end = std::min(isize, static_cast<int>(std::ceil(by1)));
        const int x_begin = std::max(0, static_cast<int>(std::floor(bx0)));
        const int x_end = std::min(isize, static_cast<int>(std::ceil(bx1)));

        for (int y = y_begin; y < y_end; ++y) {
            const float ny = ((static_cast<float>(y) + 0.5f) - ay0) / ah;
            for (int x = x_begin; x < x_end; ++x) {
                const float nx = ((static_cast<float>(x) + 0.5f) - ax0) / aw;

                const float cov = sprite_coverage(sprite, glm::vec2(nx, ny), aspect, aw);
                if (cov <= 0.0f) {
                    continue;
                }

                uint8_t* d = &px[(static_cast<size_t>(y) * static_cast<size_t>(size) +
                                  static_cast<size_t>(x)) * 4u];
                const uint8_t alpha = to_u8(cov);
                if (alpha <= d[3]) {
                    continue;   // blocks never overlap, but never trust that silently
                }
                d[0] = cr;
                d[1] = cg;
                d[2] = cb;
                d[3] = alpha;
            }
        }
    }

    dilate_paint_colour(px, size, static_cast<int>(std::lround(kBleedMarginPixels * scale)) + 1);

    ProcTexResult out;
    out.desc.width = size;
    out.desc.height = size;
    out.desc.layers = 1;
    out.desc.mip_levels = full_mip_count(size);
    out.desc.format = TextureFormat::RGBA8_SRGB;
    out.pixels = std::move(px);
    return out;
}

} // namespace stratum
