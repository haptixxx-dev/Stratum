#include "procgen/noise.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>

namespace stratum::procgen {

// Simplex noise constants
// Pre-computed values: sqrt(3) ≈ 1.7320508075688772
static constexpr float SQRT3 = 1.7320508075688772f;
static constexpr float F2 = 0.5f * (SQRT3 - 1.0f);  // Skewing factor for 2D ≈ 0.366025
static constexpr float G2 = (3.0f - SQRT3) / 6.0f;  // Unskewing factor for 2D ≈ 0.211325
static constexpr float F3 = 1.0f / 3.0f;
static constexpr float G3 = 1.0f / 6.0f;

// Gradient vectors for 2D (12 directions)
static constexpr float grad2_table[12][2] = {
    { 1.0f,  1.0f}, {-1.0f,  1.0f}, { 1.0f, -1.0f}, {-1.0f, -1.0f},
    { 1.0f,  0.0f}, {-1.0f,  0.0f}, { 0.0f,  1.0f}, { 0.0f, -1.0f},
    { 1.0f,  1.0f}, {-1.0f,  1.0f}, { 1.0f, -1.0f}, {-1.0f, -1.0f}
};

// Gradient vectors for 3D (12 directions on edges of cube)
static constexpr float grad3_table[12][3] = {
    { 1.0f,  1.0f,  0.0f}, {-1.0f,  1.0f,  0.0f}, { 1.0f, -1.0f,  0.0f}, {-1.0f, -1.0f,  0.0f},
    { 1.0f,  0.0f,  1.0f}, {-1.0f,  0.0f,  1.0f}, { 1.0f,  0.0f, -1.0f}, {-1.0f,  0.0f, -1.0f},
    { 0.0f,  1.0f,  1.0f}, { 0.0f, -1.0f,  1.0f}, { 0.0f,  1.0f, -1.0f}, { 0.0f, -1.0f, -1.0f}
};

Noise::Noise(uint32_t seed) : m_seed(seed) {
    generate_permutation();
}

void Noise::reseed(uint32_t seed) {
    m_seed = seed;
    generate_permutation();
}

void Noise::generate_permutation() {
    // Initialize with identity permutation
    uint8_t perm[256];
    for (int i = 0; i < 256; ++i) {
        perm[i] = static_cast<uint8_t>(i);
    }

    // Fisher-Yates shuffle with xorshift PRNG
    uint32_t state = m_seed;
    if (state == 0) state = 1; // Ensure non-zero

    for (int i = 255; i > 0; --i) {
        // xorshift32
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;

        int j = state % (i + 1);
        std::swap(perm[i], perm[j]);
    }

    // Double the permutation table to avoid modulo in lookups
    for (int i = 0; i < 256; ++i) {
        m_perm[i] = perm[i];
        m_perm[i + 256] = perm[i];
        m_perm_mod12[i] = perm[i] % 12;
        m_perm_mod12[i + 256] = perm[i] % 12;
    }
}

float Noise::grad2(int hash, float x, float y) {
    int h = hash & 11;
    return grad2_table[h][0] * x + grad2_table[h][1] * y;
}

float Noise::grad3(int hash, float x, float y, float z) {
    int h = hash % 12;
    return grad3_table[h][0] * x + grad3_table[h][1] * y + grad3_table[h][2] * z;
}

float Noise::simplex2d(float x, float y) const {
    // Skew input space to determine which simplex cell we're in
    float s = (x + y) * F2;
    int i = fast_floor(x + s);
    int j = fast_floor(y + s);

    // Unskew cell origin back to (x, y) space
    float t = (i + j) * G2;
    float X0 = i - t;
    float Y0 = j - t;

    // Position within cell
    float x0 = x - X0;
    float y0 = y - Y0;

    // Determine which simplex we're in (upper or lower triangle)
    int i1, j1;
    if (x0 > y0) {
        i1 = 1; j1 = 0;  // Lower triangle
    } else {
        i1 = 0; j1 = 1;  // Upper triangle
    }

    // Offsets for middle and last corners
    float x1 = x0 - i1 + G2;
    float y1 = y0 - j1 + G2;
    float x2 = x0 - 1.0f + 2.0f * G2;
    float y2 = y0 - 1.0f + 2.0f * G2;

    // Hash coordinates for gradient indices
    int ii = i & 255;
    int jj = j & 255;
    int gi0 = m_perm_mod12[ii + m_perm[jj]];
    int gi1 = m_perm_mod12[ii + i1 + m_perm[jj + j1]];
    int gi2 = m_perm_mod12[ii + 1 + m_perm[jj + 1]];

    // Calculate contributions from each corner
    float n0 = 0.0f, n1 = 0.0f, n2 = 0.0f;

    float t0 = 0.5f - x0 * x0 - y0 * y0;
    if (t0 >= 0.0f) {
        t0 *= t0;
        n0 = t0 * t0 * grad2(gi0, x0, y0);
    }

    float t1 = 0.5f - x1 * x1 - y1 * y1;
    if (t1 >= 0.0f) {
        t1 *= t1;
        n1 = t1 * t1 * grad2(gi1, x1, y1);
    }

    float t2 = 0.5f - x2 * x2 - y2 * y2;
    if (t2 >= 0.0f) {
        t2 *= t2;
        n2 = t2 * t2 * grad2(gi2, x2, y2);
    }

    // Scale to [-1, 1]
    return 70.0f * (n0 + n1 + n2);
}

float Noise::simplex3d(float x, float y, float z) const {
    // Skew input space
    float s = (x + y + z) * F3;
    int i = fast_floor(x + s);
    int j = fast_floor(y + s);
    int k = fast_floor(z + s);

    // Unskew cell origin
    float t = (i + j + k) * G3;
    float X0 = i - t;
    float Y0 = j - t;
    float Z0 = k - t;

    // Position within cell
    float x0 = x - X0;
    float y0 = y - Y0;
    float z0 = z - Z0;

    // Determine which simplex we're in
    int i1, j1, k1, i2, j2, k2;

    if (x0 >= y0) {
        if (y0 >= z0) {
            i1 = 1; j1 = 0; k1 = 0; i2 = 1; j2 = 1; k2 = 0;
        } else if (x0 >= z0) {
            i1 = 1; j1 = 0; k1 = 0; i2 = 1; j2 = 0; k2 = 1;
        } else {
            i1 = 0; j1 = 0; k1 = 1; i2 = 1; j2 = 0; k2 = 1;
        }
    } else {
        if (y0 < z0) {
            i1 = 0; j1 = 0; k1 = 1; i2 = 0; j2 = 1; k2 = 1;
        } else if (x0 < z0) {
            i1 = 0; j1 = 1; k1 = 0; i2 = 0; j2 = 1; k2 = 1;
        } else {
            i1 = 0; j1 = 1; k1 = 0; i2 = 1; j2 = 1; k2 = 0;
        }
    }

    // Offsets for remaining corners
    float x1 = x0 - i1 + G3;
    float y1 = y0 - j1 + G3;
    float z1 = z0 - k1 + G3;
    float x2 = x0 - i2 + 2.0f * G3;
    float y2 = y0 - j2 + 2.0f * G3;
    float z2 = z0 - k2 + 2.0f * G3;
    float x3 = x0 - 1.0f + 3.0f * G3;
    float y3 = y0 - 1.0f + 3.0f * G3;
    float z3 = z0 - 1.0f + 3.0f * G3;

    // Hash coordinates
    int ii = i & 255;
    int jj = j & 255;
    int kk = k & 255;
    int gi0 = m_perm[ii + m_perm[jj + m_perm[kk]]] % 12;
    int gi1 = m_perm[ii + i1 + m_perm[jj + j1 + m_perm[kk + k1]]] % 12;
    int gi2 = m_perm[ii + i2 + m_perm[jj + j2 + m_perm[kk + k2]]] % 12;
    int gi3 = m_perm[ii + 1 + m_perm[jj + 1 + m_perm[kk + 1]]] % 12;

    // Calculate contributions
    float n0 = 0.0f, n1 = 0.0f, n2 = 0.0f, n3 = 0.0f;

    float t0 = 0.6f - x0 * x0 - y0 * y0 - z0 * z0;
    if (t0 >= 0.0f) {
        t0 *= t0;
        n0 = t0 * t0 * grad3(gi0, x0, y0, z0);
    }

    float t1 = 0.6f - x1 * x1 - y1 * y1 - z1 * z1;
    if (t1 >= 0.0f) {
        t1 *= t1;
        n1 = t1 * t1 * grad3(gi1, x1, y1, z1);
    }

    float t2 = 0.6f - x2 * x2 - y2 * y2 - z2 * z2;
    if (t2 >= 0.0f) {
        t2 *= t2;
        n2 = t2 * t2 * grad3(gi2, x2, y2, z2);
    }

    float t3 = 0.6f - x3 * x3 - y3 * y3 - z3 * z3;
    if (t3 >= 0.0f) {
        t3 *= t3;
        n3 = t3 * t3 * grad3(gi3, x3, y3, z3);
    }

    // Scale to [-1, 1]
    return 32.0f * (n0 + n1 + n2 + n3);
}

float Noise::fbm2d(float x, float y, int octaves, float lacunarity, float persistence) const {
    float total = 0.0f;
    float amplitude = 1.0f;
    float frequency = 1.0f;
    float max_value = 0.0f;

    octaves = std::clamp(octaves, 1, 16);

    for (int i = 0; i < octaves; ++i) {
        total += amplitude * simplex2d(x * frequency, y * frequency);
        max_value += amplitude;
        amplitude *= persistence;
        frequency *= lacunarity;
    }

    // Normalize to approximately [-1, 1]
    return total / max_value;
}

float Noise::ridged2d(float x, float y, int octaves, float lacunarity, float gain) const {
    float sum = 0.0f;
    float amplitude = 0.5f;
    float frequency = 1.0f;
    float prev = 1.0f;

    octaves = std::clamp(octaves, 1, 16);

    for (int i = 0; i < octaves; ++i) {
        float n = simplex2d(x * frequency, y * frequency);
        n = 1.0f - std::abs(n);  // Create ridges
        n = n * n;               // Sharpen ridges
        sum += n * amplitude * prev;
        prev = n;
        frequency *= lacunarity;
        amplitude *= gain;
    }

    return sum;
}

float Noise::turbulence2d(float x, float y, int octaves) const {
    float total = 0.0f;
    float amplitude = 1.0f;
    float frequency = 1.0f;
    float max_value = 0.0f;

    octaves = std::clamp(octaves, 1, 16);

    for (int i = 0; i < octaves; ++i) {
        total += amplitude * std::abs(simplex2d(x * frequency, y * frequency));
        max_value += amplitude;
        amplitude *= 0.5f;
        frequency *= 2.0f;
    }

    return total / max_value;
}

} // namespace stratum::procgen
