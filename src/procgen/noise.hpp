#pragma once

#include <glm/glm.hpp>
#include <cstdint>

namespace stratum::procgen {

/**
 * @brief Noise generation utilities for procedural terrain
 * 
 * Implements Simplex noise (faster, fewer artifacts than Perlin)
 * with support for octave-based fractal noise (fBm).
 */
class Noise {
public:
    /**
     * @brief Initialize noise with a seed
     * @param seed Random seed for permutation table
     */
    explicit Noise(uint32_t seed = 0);

    /**
     * @brief 2D Simplex noise
     * @param x X coordinate
     * @param y Y coordinate
     * @return Noise value in range [-1, 1]
     */
    float simplex2d(float x, float y) const;

    /**
     * @brief 3D Simplex noise
     * @param x X coordinate
     * @param y Y coordinate
     * @param z Z coordinate
     * @return Noise value in range [-1, 1]
     */
    float simplex3d(float x, float y, float z) const;

    /**
     * @brief Fractal Brownian Motion (fBm) using 2D simplex noise
     * @param x X coordinate
     * @param y Y coordinate
     * @param octaves Number of noise layers (1-16)
     * @param lacunarity Frequency multiplier per octave (typically 2.0)
     * @param persistence Amplitude multiplier per octave (typically 0.5)
     * @return Noise value (approximately in [-1, 1] but can exceed)
     */
    float fbm2d(float x, float y, int octaves, float lacunarity = 2.0f, float persistence = 0.5f) const;

    /**
     * @brief Ridged multifractal noise (creates sharp ridges, good for mountains)
     * @param x X coordinate
     * @param y Y coordinate
     * @param octaves Number of noise layers
     * @param lacunarity Frequency multiplier per octave
     * @param gain Amplitude modifier (affects ridge sharpness)
     * @return Noise value in range [0, 1]
     */
    float ridged2d(float x, float y, int octaves, float lacunarity = 2.0f, float gain = 0.5f) const;

    /**
     * @brief Turbulence noise (absolute value of fBm, always positive)
     * @param x X coordinate
     * @param y Y coordinate
     * @param octaves Number of noise layers
     * @return Noise value in range [0, 1]
     */
    float turbulence2d(float x, float y, int octaves) const;

    /**
     * @brief Reseed the noise generator
     * @param seed New random seed
     */
    void reseed(uint32_t seed);

    /**
     * @brief Get current seed
     */
    uint32_t get_seed() const { return m_seed; }

private:
    uint32_t m_seed;
    uint8_t m_perm[512];      // Permutation table (doubled for wrapping)
    uint8_t m_perm_mod12[512]; // Permutation table mod 12 for gradient indexing

    void generate_permutation();

    // Gradient helpers
    static float grad2(int hash, float x, float y);
    static float grad3(int hash, float x, float y, float z);

    // Fast floor
    static int fast_floor(float x) {
        int xi = static_cast<int>(x);
        return (x < xi) ? xi - 1 : xi;
    }
};

} // namespace stratum::procgen
