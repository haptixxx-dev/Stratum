/**
 * @file test_material_uniforms.cpp
 * @brief The C++/GLSL uniform block contract: 48 bytes, three vec4s, in this order
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * This suite exists because of one specific failure mode, called out in
 * material_library.hpp and again in the shader itself: a mismatch between
 * `struct MaterialUniforms` and the `MaterialUniforms` block in
 * assets/shaders/mesh_pbr.frag is NOT a compile error, NOT a link error, and NOT
 * a Vulkan validation error. SDL_PushGPUFragmentUniformData takes a void* and a
 * byte count. A member inserted, reordered or widened on one side only produces
 * roughness read out of the emissive slot -- plausible-looking, wrong everywhere,
 * and invisible in a stack trace.
 *
 * So the layout is asserted twice: structurally, with offsetof, and textually,
 * against the GLSL source that the SPIR-V is compiled from.
 *
 * ### The GLSL block this file is pinned to
 *
 * Copied verbatim from assets/shaders/mesh_pbr.frag. EDIT BOTH TOGETHER.
 *
 * @code{.glsl}
 *     layout(set = 3, binding = 1) uniform MaterialUniforms {
 *         vec4 base_color;   // rgb tint, a alpha             offset  0
 *         vec4 pbr_params;   // x metallic, y roughness,
 *                            // z ao, w emissive              offset 16
 *         vec4 uv_params;    // xy uv_scale, zw unused        offset 32
 *     } material;                                          // size   48
 *
 *     layout(set = 2, binding = 0) uniform sampler2D albedo_map;
 *     layout(set = 2, binding = 1) uniform sampler2D normal_map;
 *     layout(set = 2, binding = 2) uniform sampler2D orm_map;
 * @endcode
 *
 * Every member is a vec4, so std140 needs no padding and there is no padding to
 * get wrong. That is the whole reason the scalars are packed into vectors rather
 * than declared as four floats, and why "tidying" them apart would break this.
 *
 * Run this suite with:
 * @code
 *     ./stratum_gpu_tests MaterialUniforms
 * @endcode
 */

#include "framework.hpp"

#include "renderer/gpu_renderer.hpp"
#include "renderer/material_library.hpp"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

using stratum::MaterialUniforms;

// offsetof on a type whose members are glm vectors is conditionally supported:
// glm may make vec4 non-standard-layout when it selects a SIMD representation.
// GCC and Clang both compute it correctly and only warn, so the warning is
// silenced rather than the check weakened -- the offsets are exactly what this
// file exists to pin down.
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#endif

static_assert(sizeof(MaterialUniforms) == 48,
              "MaterialUniforms must be three tightly packed vec4s; see the GLSL block "
              "quoted at the top of this file");
static_assert(alignof(MaterialUniforms) >= 16,
              "std140 requires 16-byte alignment; material_library.hpp declares alignas(16)");

static_assert(offsetof(MaterialUniforms, base_color) == 0,
              "base_color must be the first member: GLSL offset 0");
static_assert(offsetof(MaterialUniforms, pbr_params) == 16,
              "pbr_params must follow base_color with no padding: GLSL offset 16");
static_assert(offsetof(MaterialUniforms, uv_params) == 32,
              "uv_params must be last: GLSL offset 32");

static_assert(sizeof(MaterialUniforms::base_color) == 16, "base_color must be a vec4");
static_assert(sizeof(MaterialUniforms::pbr_params) == 16, "pbr_params must be a vec4");
static_assert(sizeof(MaterialUniforms::uv_params) == 16, "uv_params must be a vec4");

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

/// Whitespace-stripped copy, so the GLSL checks survive reformatting.
[[nodiscard]] std::string strip_spaces(const std::string& text) {
    std::string out;
    out.reserve(text.size());
    for (const char c : text) {
        if (c != ' ' && c != '\t' && c != '\r' && c != '\n') {
            out.push_back(c);
        }
    }
    return out;
}

/// Read a shader source from the source tree. Empty when it cannot be opened.
[[nodiscard]] std::string read_shader(const char* name) {
    std::string path = std::string{STRATUM_TEST_SHADER_DIR} + "/" + name;
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        return {};
    }
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

/// A `layout(set = S, binding = B)` qualifier, in either member order.
[[nodiscard]] bool has_layout(const std::string& stripped, int set, int binding) {
    const std::string a = "layout(set=" + std::to_string(set) +
                          ",binding=" + std::to_string(binding) + ")";
    const std::string b = "layout(binding=" + std::to_string(binding) +
                          ",set=" + std::to_string(set) + ")";
    return stripped.find(a) != std::string::npos || stripped.find(b) != std::string::npos;
}

} // namespace

// ============================================================================
// Structural layout
// ============================================================================

/// The static_asserts above are the real check; these restate them at run time so
/// a failure names the offending offset instead of only failing to compile.
TEST(MaterialUniforms, block_is_three_vec4s_at_the_documented_offsets) {
    CHECK_EQ(sizeof(MaterialUniforms), size_t{48});
    CHECK(alignof(MaterialUniforms) >= 16);

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#endif
    CHECK_EQ(offsetof(MaterialUniforms, base_color), size_t{0});
    CHECK_EQ(offsetof(MaterialUniforms, pbr_params), size_t{16});
    CHECK_EQ(offsetof(MaterialUniforms, uv_params), size_t{32});
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
}

/**
 * @brief The bytes SDL_PushGPUFragmentUniformData would actually send
 *
 * offsetof proves where the members live; this proves the struct has no hidden
 * padding between or after them, by writing twelve distinguishable floats and
 * reading the block back as a flat array exactly as the GPU does.
 */
TEST(MaterialUniforms, twelve_floats_reach_the_gpu_in_declaration_order) {
    MaterialUniforms u{};
    u.base_color = glm::vec4(1.0f, 2.0f, 3.0f, 4.0f);
    u.pbr_params = glm::vec4(5.0f, 6.0f, 7.0f, 8.0f);
    u.uv_params = glm::vec4(9.0f, 10.0f, 11.0f, 12.0f);

    float flat[12] = {};
    static_assert(sizeof(flat) == sizeof(MaterialUniforms),
                  "twelve floats is the whole block; anything else means padding");
    std::memcpy(flat, &u, sizeof(u));

    for (int i = 0; i < 12; ++i) {
        CHECK_EQ(flat[i], static_cast<float>(i + 1));
    }
}

/// The header's in-class initialisers are the "no material bound" appearance:
/// white, dielectric, matte, unoccluded, unlit, unscaled UVs.
TEST(MaterialUniforms, default_constructed_block_is_a_neutral_material) {
    const MaterialUniforms u{};

    CHECK_NEAR(u.base_color.x, 1.0, 1e-6);
    CHECK_NEAR(u.base_color.y, 1.0, 1e-6);
    CHECK_NEAR(u.base_color.z, 1.0, 1e-6);
    CHECK_NEAR(u.base_color.w, 1.0, 1e-6);

    CHECK_NEAR(u.pbr_params.x, 0.0, 1e-6);   // metallic
    CHECK_NEAR(u.pbr_params.y, 0.8, 1e-6);   // roughness
    CHECK_NEAR(u.pbr_params.z, 1.0, 1e-6);   // ao
    CHECK_NEAR(u.pbr_params.w, 0.0, 1e-6);   // emissive

    CHECK_NEAR(u.uv_params.x, 1.0, 1e-6);
    CHECK_NEAR(u.uv_params.y, 1.0, 1e-6);
    // zw are declared unused and must stay zero: the shader is free to start
    // reading them without any C++ change.
    CHECK_NEAR(u.uv_params.z, 0.0, 1e-6);
    CHECK_NEAR(u.uv_params.w, 0.0, 1e-6);
}

// ============================================================================
// The GLSL side of the contract
// ============================================================================

/**
 * @brief mesh_pbr.frag declares the block this struct claims to match
 *
 * Textual rather than reflective: reading the SPIR-V would need a reflection
 * library this project does not vendor, and the failure being guarded against --
 * someone edits one side and not the other -- is caught just as well by reading
 * the source the SPIR-V is compiled from.
 */
TEST(MaterialUniforms, glsl_declares_the_same_block_at_set_3_binding_1) {
    const std::string source = read_shader("mesh_pbr.frag");
    CHECK_FALSE(source.empty());
    if (source.empty()) {
        return;
    }

    const std::string flat = strip_spaces(source);

    CHECK_TRUE(has_layout(flat, 3, 1));
    CHECK(flat.find("uniformMaterialUniforms{") != std::string::npos);

    // Members, in order, with no extra member between them.
    const size_t block = flat.find("uniformMaterialUniforms{");
    const size_t base = flat.find("vec4base_color;", block);
    const size_t pbr = flat.find("vec4pbr_params;", block);
    const size_t uv = flat.find("vec4uv_params;", block);
    const size_t close = flat.find("}", block);

    CHECK(base != std::string::npos);
    CHECK(pbr != std::string::npos);
    CHECK(uv != std::string::npos);
    CHECK(close != std::string::npos);
    if (base == std::string::npos || pbr == std::string::npos ||
        uv == std::string::npos || close == std::string::npos) {
        return;
    }

    CHECK(base < pbr);
    CHECK(pbr < uv);
    CHECK(uv < close);

    // Exactly three members: the block body holds three semicolons.
    const std::string body = flat.substr(block, close - block);
    size_t members = 0;
    for (const char c : body) {
        if (c == ';') {
            ++members;
        }
    }
    CHECK_EQ(members, size_t{3});
}

/**
 * @brief The scene block must stay at set 3 binding 0
 *
 * Or the material block's push slot collides with it and both draws read each
 * other's bytes.
 *
 * The block moved out of mesh_pbr.frag and into sky_common.glsl when sky.frag
 * started needing the same uniforms: two copies of one std140 layout is exactly
 * the drift this file exists to catch, so there is now one copy and both shaders
 * include it. The include itself is asserted below, because a mesh_pbr.frag that
 * stopped including the header would not fail to compile -- it would fail to
 * find `scene`, which IS a compile error, but only after someone rebuilt.
 */
TEST(MaterialUniforms, glsl_keeps_the_scene_block_at_set_3_binding_0) {
    const std::string source = read_shader("sky_common.glsl");
    if (source.empty()) {
        CHECK_FALSE(source.empty());
        return;
    }
    const std::string flat = strip_spaces(source);

    CHECK_TRUE(has_layout(flat, 3, 0));
    CHECK(flat.find("uniformSceneUniforms{") != std::string::npos);

    CHECK_EQ(stratum::kSceneUniformSlot, uint32_t{0});
    CHECK_EQ(stratum::kMaterialUniformSlot, uint32_t{1});
    // Three: scene, material, shadow.
    CHECK_EQ(stratum::kPbrFragmentUniformBufferCount, uint32_t{3});
    CHECK_EQ(stratum::kShadowUniformSlot, uint32_t{2});

    const std::string pbr = strip_spaces(read_shader("mesh_pbr.frag"));
    const std::string sky = strip_spaces(read_shader("sky.frag"));
    CHECK(pbr.find("#include\"sky_common.glsl\"") != std::string::npos);
    CHECK(sky.find("#include\"sky_common.glsl\"") != std::string::npos);
}

/**
 * @brief SceneUniforms and its GLSL block hold the same members in the same order
 *
 * std140 gives a block of nothing but vec4s the same layout as the packed C++
 * struct, so member COUNT and ORDER are the whole contract -- and neither SDL nor
 * the driver will complain if they disagree, because the push is a byte blob.
 * The sky and IBL members were appended to a block that had six members and now
 * has ten; appending to one side only would have silently shifted every field
 * after the insertion point.
 */
TEST(MaterialUniforms, glsl_scene_block_members_match_the_cpp_struct) {
    const std::string flat = strip_spaces(read_shader("sky_common.glsl"));
    if (flat.empty()) {
        CHECK_FALSE(flat.empty());
        return;
    }

    const size_t block = flat.find("uniformSceneUniforms{");
    CHECK(block != std::string::npos);
    if (block == std::string::npos) return;
    const size_t close = flat.find("}scene;", block);
    CHECK(close != std::string::npos);
    if (close == std::string::npos) return;

    const std::string body = flat.substr(block, close - block);

    // In declaration order. Each must appear, and each must follow the last.
    const char* expected[] = {
        "vec4camera_position;", "vec4sun_direction;", "vec4sun_color;",
        "vec4fog_params;",      "vec4fog_color;",     "vec4pbr_params;",
        "vec4sky_zenith;",      "vec4sky_horizon;",   "vec4ground_color;",
        "vec4ibl_params;",
    };
    size_t previous = 0;
    for (const char* member : expected) {
        const size_t at = body.find(member);
        CHECK(at != std::string::npos);
        if (at == std::string::npos) return;
        CHECK(at >= previous);
        previous = at;
    }

    // And no ELEVENTH member: the body holds exactly as many semicolons as there
    // are declarations above.
    size_t members = 0;
    for (const char c : body) {
        if (c == ';') ++members;
    }
    CHECK_EQ(members, sizeof(expected) / sizeof(expected[0]));

    // The C++ side is all vec4 and nothing else, so its size is the count times 16.
    CHECK_EQ(sizeof(stratum::SceneUniforms), members * sizeof(glm::vec4));
}

/**
 * @brief The three sampler bindings agree with the C++ slot constants
 *
 * SDL_BindGPUFragmentSamplers takes a first-slot index and binds consecutively,
 * so the C++ constants ARE the GLSL binding numbers. Swapping normal_map and
 * orm_map here would light every surface from a roughness map.
 */
TEST(MaterialUniforms, glsl_sampler_bindings_match_the_slot_constants) {
    const std::string source = read_shader("mesh_pbr.frag");
    if (source.empty()) {
        CHECK_FALSE(source.empty());
        return;
    }
    const std::string flat = strip_spaces(source);

    CHECK_EQ(stratum::kAlbedoSamplerSlot, uint32_t{0});
    CHECK_EQ(stratum::kNormalSamplerSlot, uint32_t{1});
    CHECK_EQ(stratum::kOrmSamplerSlot, uint32_t{2});
    CHECK_EQ(stratum::kMaterialSamplerCount, uint32_t{3});

    CHECK(flat.find("layout(set=2,binding=0)uniformsampler2Dalbedo_map;") != std::string::npos);
    CHECK(flat.find("layout(set=2,binding=1)uniformsampler2Dnormal_map;") != std::string::npos);
    CHECK(flat.find("layout(set=2,binding=2)uniformsampler2Dorm_map;") != std::string::npos);
}

/**
 * @brief The checked-in SPIR-V is not older than the GLSL it was compiled from
 *
 * assets/shaders holds sources AND their .spv, and the runtime loads only the
 * .spv. Before this phase there was no build rule connecting them, so an edited
 * shader changed nothing until someone ran glslc by hand -- the exact trap the
 * material_library.hpp header calls out. A stale .spv here means the block
 * asserted above is not the block the GPU is running.
 */
TEST(MaterialUniforms, compiled_spirv_is_not_older_than_its_glsl_source) {
    namespace fs = std::filesystem;

    const fs::path dir{STRATUM_TEST_SHADER_DIR};
    const char* names[] = {"mesh.vert", "mesh.frag", "mesh_pbr.vert", "mesh_pbr.frag"};

    for (const char* name : names) {
        const fs::path source = dir / name;
        const fs::path spv = dir / (std::string{name} + ".spv");

        std::error_code ec;
        CHECK_TRUE(fs::exists(source, ec));
        CHECK_TRUE(fs::exists(spv, ec));
        if (!fs::exists(source, ec) || !fs::exists(spv, ec)) {
            continue;
        }

        const auto source_time = fs::last_write_time(source, ec);
        const auto spv_time = fs::last_write_time(spv, ec);
        if (ec) {
            continue;
        }
        if (!(spv_time >= source_time)) {
            ::stratum::test::report_failure(
                __FILE__, __LINE__, "spv is at least as new as its GLSL source",
                std::string{"stale: "} + name + ".spv predates " + name +
                    " -- run glslc, or configure with the shader build rule enabled");
        }
    }
}

// ============================================================================
// The SPIR-V the runtime actually loads
// ============================================================================

namespace {

/// A minimal SPIR-V reader: enough to answer "what is the uniform block at
/// (set, binding), and what are its member offsets and types?".
///
/// The textual tests above read mesh_pbr.frag. The runtime reads
/// mesh_pbr.frag.spv and never opens the GLSL at all, so a source that says one
/// thing and a binary that says another passes every check above. The mtime test
/// is only a proxy for that -- `touch` defeats it, and so does any edit that
/// rebuilds the .spv from a DIFFERENT source tree. This reads the bytes the GPU
/// is handed and reflects the std140 offsets glslc actually computed, which is
/// the "byte for byte" half of the contract that source text cannot prove.
class SpirvModule {
public:
    /// Parse a .spv file. is_valid() is false when it is missing or malformed.
    explicit SpirvModule(const std::filesystem::path& path) {
        std::ifstream f(path, std::ios::binary);
        if (!f) {
            return;
        }
        const std::vector<char> raw{std::istreambuf_iterator<char>(f),
                                    std::istreambuf_iterator<char>()};
        if (raw.size() < 20 || (raw.size() % 4) != 0) {
            return;
        }
        m_words.resize(raw.size() / 4);
        std::memcpy(m_words.data(), raw.data(), raw.size());
        // 0x07230203 little-endian. A big-endian module would need byte-swapping,
        // which no toolchain this project uses produces.
        if (m_words[0] != 0x07230203u) {
            m_words.clear();
            return;
        }
        parse();
    }

    [[nodiscard]] bool is_valid() const { return !m_words.empty(); }

    /// Result id of the uniform variable decorated with this set and binding, or
    /// 0 when no variable carries both.
    [[nodiscard]] uint32_t variable_at(uint32_t set, uint32_t binding) const {
        for (const auto& [id, d] : m_decor) {
            if (d.has_set && d.has_binding && d.set == set && d.binding == binding) {
                return id;
            }
        }
        return 0;
    }

    /// Struct type id behind a uniform variable id: OpVariable -> OpTypePointer
    /// -> the pointee. 0 when the variable is not a pointer to a struct.
    [[nodiscard]] uint32_t struct_of_variable(uint32_t var_id) const {
        const auto v = m_var_type.find(var_id);
        if (v == m_var_type.end()) {
            return 0;
        }
        const auto p = m_ptr_pointee.find(v->second);
        if (p == m_ptr_pointee.end()) {
            return 0;
        }
        return m_struct_members.count(p->second) ? p->second : 0;
    }

    /// Member type ids of a struct, in declaration order.
    [[nodiscard]] std::vector<uint32_t> struct_members(uint32_t struct_id) const {
        const auto it = m_struct_members.find(struct_id);
        return it == m_struct_members.end() ? std::vector<uint32_t>{} : it->second;
    }

    /// std140 byte offset glslc assigned to a member, or SIZE_MAX when undecorated.
    [[nodiscard]] size_t member_offset(uint32_t struct_id, uint32_t member) const {
        const auto it = m_member_offset.find({struct_id, member});
        return it == m_member_offset.end() ? SIZE_MAX : it->second;
    }

    /// True when this type id is a 4-component vector of 32-bit floats.
    [[nodiscard]] bool is_vec4(uint32_t type_id) const {
        const auto v = m_vector.find(type_id);
        if (v == m_vector.end() || v->second.count != 4) {
            return false;
        }
        const auto f = m_float_width.find(v->second.component);
        return f != m_float_width.end() && f->second == 32;
    }

    /// Result ids of every OpTypeImage-backed sampled image variable, by (set, binding).
    [[nodiscard]] bool has_sampled_image_at(uint32_t set, uint32_t binding) const {
        const uint32_t id = variable_at(set, binding);
        if (id == 0) {
            return false;
        }
        const auto v = m_var_type.find(id);
        if (v == m_var_type.end()) {
            return false;
        }
        const auto p = m_ptr_pointee.find(v->second);
        return p != m_ptr_pointee.end() && m_sampled_image.count(p->second) > 0;
    }

private:
    struct Decor {
        uint32_t set = 0;
        uint32_t binding = 0;
        bool has_set = false;
        bool has_binding = false;
    };
    struct Vec {
        uint32_t component = 0;
        uint32_t count = 0;
    };
    struct PairHash {
        size_t operator()(const std::pair<uint32_t, uint32_t>& p) const {
            return (static_cast<size_t>(p.first) << 32) ^ p.second;
        }
    };

    void parse() {
        // SPIR-V spec: Decoration DescriptorSet = 34, Binding = 33, Offset = 35.
        constexpr uint32_t kDecorationBinding = 33;
        constexpr uint32_t kDecorationDescriptorSet = 34;
        constexpr uint32_t kDecorationOffset = 35;

        size_t i = 5; // header is five words
        while (i < m_words.size()) {
            const uint32_t word = m_words[i];
            const uint16_t count = static_cast<uint16_t>(word >> 16);
            const uint16_t op = static_cast<uint16_t>(word & 0xFFFFu);
            if (count == 0 || i + count > m_words.size()) {
                return; // malformed; keep what was parsed
            }
            const uint32_t* a = &m_words[i + 1];
            const uint16_t n = static_cast<uint16_t>(count - 1);

            switch (op) {
                case 71: // OpDecorate: target, decoration, [literal]
                    if (n >= 3 && a[1] == kDecorationDescriptorSet) {
                        m_decor[a[0]].set = a[2];
                        m_decor[a[0]].has_set = true;
                    } else if (n >= 3 && a[1] == kDecorationBinding) {
                        m_decor[a[0]].binding = a[2];
                        m_decor[a[0]].has_binding = true;
                    }
                    break;
                case 72: // OpMemberDecorate: struct, member, decoration, [literal]
                    if (n >= 4 && a[2] == kDecorationOffset) {
                        m_member_offset[{a[0], a[1]}] = a[3];
                    }
                    break;
                case 22: // OpTypeFloat: result, width
                    if (n >= 2) {
                        m_float_width[a[0]] = a[1];
                    }
                    break;
                case 23: // OpTypeVector: result, component type, count
                    if (n >= 3) {
                        m_vector[a[0]] = Vec{a[1], a[2]};
                    }
                    break;
                case 25: // OpTypeImage: result, ...
                    if (n >= 1) {
                        m_image.insert(a[0]);
                    }
                    break;
                case 27: // OpTypeSampledImage: result, image type
                    if (n >= 2) {
                        m_sampled_image.insert(a[0]);
                    }
                    break;
                case 30: // OpTypeStruct: result, member types...
                    if (n >= 1) {
                        m_struct_members[a[0]] =
                            std::vector<uint32_t>(a + 1, a + n);
                    }
                    break;
                case 32: // OpTypePointer: result, storage class, type
                    if (n >= 3) {
                        m_ptr_pointee[a[0]] = a[2];
                    }
                    break;
                case 59: // OpVariable: result type, result, storage class
                    if (n >= 3) {
                        m_var_type[a[1]] = a[0];
                    }
                    break;
                default:
                    break;
            }
            i += count;
        }
    }

    std::vector<uint32_t> m_words;
    std::unordered_map<uint32_t, Decor> m_decor;
    std::unordered_map<uint32_t, std::vector<uint32_t>> m_struct_members;
    std::unordered_map<std::pair<uint32_t, uint32_t>, size_t, PairHash> m_member_offset;
    std::unordered_map<uint32_t, Vec> m_vector;
    std::unordered_map<uint32_t, uint32_t> m_float_width;
    std::unordered_map<uint32_t, uint32_t> m_ptr_pointee;
    std::unordered_map<uint32_t, uint32_t> m_var_type;
    std::unordered_set<uint32_t> m_image;
    std::unordered_set<uint32_t> m_sampled_image;
};

} // namespace

/**
 * @brief The compiled SPIR-V declares MaterialUniforms as three vec4s at 0/16/32
 *
 * This is the assertion the file's title promises and the one the textual tests
 * can only approximate. It reflects mesh_pbr.frag.spv -- the exact bytes
 * GPURenderer::load_shader() hands to SDL_CreateGPUShader -- and compares the
 * std140 offsets glslc computed against offsetof on the C++ struct. Both sides of
 * the contract are read from the artefact that ships, not from a description of it.
 */
TEST(MaterialUniforms, compiled_spirv_material_block_matches_the_cpp_struct) {
    const SpirvModule m{std::filesystem::path{STRATUM_TEST_SHADER_DIR} / "mesh_pbr.frag.spv"};
    CHECK_TRUE(m.is_valid());
    if (!m.is_valid()) {
        return;
    }

    const uint32_t var = m.variable_at(3, stratum::kMaterialUniformSlot);
    CHECK(var != 0);
    if (var == 0) {
        return;
    }

    const uint32_t block = m.struct_of_variable(var);
    CHECK(block != 0);
    if (block == 0) {
        return;
    }

    const std::vector<uint32_t> members = m.struct_members(block);
    CHECK_EQ(members.size(), size_t{3});
    if (members.size() != 3) {
        return;
    }

    for (size_t i = 0; i < members.size(); ++i) {
        if (!m.is_vec4(members[i])) {
            ::stratum::test::report_failure(
                __FILE__, __LINE__, "every MaterialUniforms member is a vec4",
                "member " + std::to_string(i) + " of the compiled block is not vec4");
        }
    }

    // The offsets glslc computed, against the offsets the C++ struct has.
    CHECK_EQ(m.member_offset(block, 0), size_t{0});
    CHECK_EQ(m.member_offset(block, 1), size_t{16});
    CHECK_EQ(m.member_offset(block, 2), size_t{32});

#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Winvalid-offsetof"
#endif
    CHECK_EQ(m.member_offset(block, 0), offsetof(MaterialUniforms, base_color));
    CHECK_EQ(m.member_offset(block, 1), offsetof(MaterialUniforms, pbr_params));
    CHECK_EQ(m.member_offset(block, 2), offsetof(MaterialUniforms, uv_params));
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

    // Last member at offset 32 plus a vec4 is the 48 bytes the C++ side pushes.
    CHECK_EQ(m.member_offset(block, 2) + 16u, sizeof(MaterialUniforms));
}

/// The scene block must still be the OTHER fragment uniform buffer, at binding 0,
/// or the two push slots collide and each draw reads the other's bytes.
TEST(MaterialUniforms, compiled_spirv_keeps_the_scene_block_at_set_3_binding_0) {
    const SpirvModule m{std::filesystem::path{STRATUM_TEST_SHADER_DIR} / "mesh_pbr.frag.spv"};
    CHECK_TRUE(m.is_valid());
    if (!m.is_valid()) {
        return;
    }

    const uint32_t scene = m.variable_at(3, stratum::kSceneUniformSlot);
    const uint32_t material = m.variable_at(3, stratum::kMaterialUniformSlot);

    CHECK(scene != 0);
    CHECK(material != 0);
    CHECK(scene != material);
    CHECK(m.struct_of_variable(scene) != 0);
    CHECK(m.struct_of_variable(scene) != m.struct_of_variable(material));
}

/**
 * @brief The four sampler bindings exist in the compiled module, as sampled images
 *
 * SDL_CreateGPUShader is told num_samplers = kPbrFragmentSamplerCount. Declaring
 * more samplers than the module actually contains wastes a descriptor; declaring
 * fewer is a hard SDL error at pipeline creation. This pins the count to what the
 * binary really has, at the bindings the C++ slot constants name.
 *
 * The fourth is the cascaded shadow map, and it is deliberately NOT part of the
 * material set: kMaterialSamplerCount is the number bind_material() binds from
 * slot 0, kPbrFragmentSamplerCount is the number the shader declares. Merging
 * them back into one constant would make every material rebind the shadow map.
 */
TEST(MaterialUniforms, compiled_spirv_declares_the_four_sampler_bindings) {
    const SpirvModule m{std::filesystem::path{STRATUM_TEST_SHADER_DIR} / "mesh_pbr.frag.spv"};
    CHECK_TRUE(m.is_valid());
    if (!m.is_valid()) {
        return;
    }

    CHECK_TRUE(m.has_sampled_image_at(2, stratum::kAlbedoSamplerSlot));
    CHECK_TRUE(m.has_sampled_image_at(2, stratum::kNormalSamplerSlot));
    CHECK_TRUE(m.has_sampled_image_at(2, stratum::kOrmSamplerSlot));
    CHECK_TRUE(m.has_sampled_image_at(2, stratum::kShadowSamplerSlot));

    CHECK_EQ(stratum::kPbrFragmentSamplerCount, stratum::kMaterialSamplerCount + 1);

    // Exactly the declared count: the slot one past the end must be absent, or
    // num_samplers is under-declared and a future map binds into nothing.
    CHECK_FALSE(m.has_sampled_image_at(2, stratum::kPbrFragmentSamplerCount));
}
