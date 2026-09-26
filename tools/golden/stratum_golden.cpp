// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file stratum_golden.cpp
 * @brief Deterministic, GPU-free snapshot of the core OSM import pipeline
 *
 * Runs exactly the stratum_core stages Editor::poll_osm_import() runs before
 * anything touches the GPU (src/editor/editor.cpp, poll_osm_import() and
 * begin_mesh_rebuild()), with the editor's own default options:
 *
 *   - osm::OSMParser with a default-constructed osm::ParserConfig (the
 *     draw_osm_panel() static `config` before the user touches a checkbox).
 *   - osm::road::RoadNetworkBuilder::build() with a default-constructed
 *     osm::road::RoadNetworkConfig. That default agrees field-for-field with
 *     Editor::make_road_network_config()'s own defaults (m_solve_junctions,
 *     m_emit_markings, m_emit_crossings, m_emit_structures and
 *     m_reduce_tessellation are all true, same as RoadNetworkConfig's own
 *     members), except height_sampler: the editor's is only non-null once
 *     chunked terrain has been generated, and this tool never generates
 *     terrain, so leaving it null reproduces a fresh import exactly. This is
 *     the "skip terrain carve" the harness is asked to do -- there is no
 *     terrain here to carve.
 *   - osm::QuadTree::init/assign_data/assign_road_pieces, with chunk LOD on
 *     and a default-constructed osm::road::ChunkLodConfig, matching
 *     Editor::m_chunk_lod's default and begin_mesh_rebuild()'s
 *     `m_quadtree.set_chunk_lod(m_chunk_lod, osm::road::ChunkLodConfig{})`.
 *   - The per-leaf building/area mesh build, via the same
 *     queue_node_build_async()/poll_async_builds() pair poll_osm_import()
 *     itself drains. Both are pure stratum_core and touch no GPU handle; only
 *     the GPU *upload* that follows in the editor is skipped.
 *
 * Usage:
 *     stratum_golden --osm <file> [--json]
 *
 * Prints one JSON object to stdout: the file name, its sha256, the parser's
 * raw and processed counts, the road graph and road network build statistics,
 * the junction solve statistics, the quadtree's spatial stats, and the
 * triangle/vertex totals per mesh category. Every field is an integer count;
 * there are no timestamps, no pointers and no wall-clock timings, so two runs
 * of the same binary against the same file produce byte-identical output.
 * `--json` is accepted and currently a no-op -- the output is JSON either
 * way -- kept so a future human-readable mode has a flag to switch on rather
 * than one to switch off.
 */

#include "osm/parser.hpp"
#include "osm/quadtree.hpp"
#include "osm/road/road_network_builder.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

namespace {

// ============================================================================
// SHA-256 -- self-contained, streaming, public-domain algorithm
// ============================================================================
// Vendored rather than linked: stratum_core has no crypto dependency and this
// tool is the only thing in the tree that needs a hash, so adding one to
// stratum_core for this alone would be backwards. This file links against
// stratum_core, never the other way round -- see CLAUDE.md's two-library
// split.
class Sha256 {
public:
    Sha256() {
        m_state = {0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u, 0xa54ff53au,
                   0x510e527fu, 0x9b05688cu, 0x1f83d9abu, 0x5be0cd19u};
    }

    void update(const uint8_t* data, size_t len) {
        m_total_len += len;
        while (len > 0) {
            const size_t take = std::min(len, size_t{64} - m_buffer_len);
            std::memcpy(m_buffer + m_buffer_len, data, take);
            m_buffer_len += take;
            data += take;
            len -= take;
            if (m_buffer_len == 64) {
                transform(m_buffer);
                m_buffer_len = 0;
            }
        }
    }

    [[nodiscard]] std::string hex_digest() {
        const uint64_t bit_len = m_total_len * 8;

        const uint8_t one = 0x80;
        update(&one, 1);
        const uint8_t zero = 0x00;
        while (m_buffer_len != 56) {
            update(&zero, 1);
        }

        uint8_t len_bytes[8];
        for (int i = 0; i < 8; ++i) {
            len_bytes[7 - i] = static_cast<uint8_t>(bit_len >> (8 * i));
        }
        update(len_bytes, 8);  // brings the buffer to exactly 64 and transforms

        std::ostringstream out;
        out << std::hex << std::setfill('0');
        for (uint32_t word : m_state) {
            out << std::setw(8) << word;
        }
        return out.str();
    }

private:
    static uint32_t rotr(uint32_t x, uint32_t n) { return (x >> n) | (x << (32 - n)); }

    void transform(const uint8_t block[64]) {
        static constexpr uint32_t k[64] = {
            0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu, 0x59f111f1u,
            0x923f82a4u, 0xab1c5ed5u, 0xd807aa98u, 0x12835b01u, 0x243185beu, 0x550c7dc3u,
            0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u, 0xc19bf174u, 0xe49b69c1u, 0xefbe4786u,
            0x0fc19dc6u, 0x240ca1ccu, 0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau,
            0x983e5152u, 0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u,
            0x06ca6351u, 0x14292967u, 0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu, 0x53380d13u,
            0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u, 0xa2bfe8a1u, 0xa81a664bu,
            0xc24b8b70u, 0xc76c51a3u, 0xd192e819u, 0xd6990624u, 0xf40e3585u, 0x106aa070u,
            0x19a4c116u, 0x1e376c08u, 0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au,
            0x5b9cca4fu, 0x682e6ff3u, 0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u,
            0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u};

        uint32_t w[64];
        for (int i = 0; i < 16; ++i) {
            w[i] = (static_cast<uint32_t>(block[i * 4]) << 24) |
                   (static_cast<uint32_t>(block[i * 4 + 1]) << 16) |
                   (static_cast<uint32_t>(block[i * 4 + 2]) << 8) |
                   static_cast<uint32_t>(block[i * 4 + 3]);
        }
        for (int i = 16; i < 64; ++i) {
            const uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
            const uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }

        auto [a, b, c, d, e, f, g, h] = std::tuple{m_state[0], m_state[1], m_state[2], m_state[3],
                                                    m_state[4], m_state[5], m_state[6], m_state[7]};

        for (int i = 0; i < 64; ++i) {
            const uint32_t s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
            const uint32_t ch = (e & f) ^ (~e & g);
            const uint32_t temp1 = h + s1 + ch + k[i] + w[i];
            const uint32_t s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
            const uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
            const uint32_t temp2 = s0 + maj;

            h = g; g = f; f = e; e = d + temp1;
            d = c; c = b; b = a; a = temp1 + temp2;
        }

        m_state[0] += a; m_state[1] += b; m_state[2] += c; m_state[3] += d;
        m_state[4] += e; m_state[5] += f; m_state[6] += g; m_state[7] += h;
    }

    std::array<uint32_t, 8> m_state{};
    uint8_t m_buffer[64]{};
    size_t m_buffer_len = 0;
    uint64_t m_total_len = 0;
};

[[nodiscard]] std::string sha256_of_file(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return "";
    }
    Sha256 hasher;
    std::vector<uint8_t> buf(1u << 20);  // 1 MiB
    while (in) {
        in.read(reinterpret_cast<char*>(buf.data()), static_cast<std::streamsize>(buf.size()));
        const auto got = in.gcount();
        if (got > 0) {
            hasher.update(buf.data(), static_cast<size_t>(got));
        }
    }
    return hasher.hex_digest();
}

// ============================================================================
// Minimal deterministic JSON writer
// ============================================================================
// Fixed field order, 2-space indent, integers only (this tool prints no
// floats). A hand-rolled writer avoids pulling a JSON library into a tool
// that links stratum_core alone, and the schema here is small and fixed.
class JsonWriter {
public:
    explicit JsonWriter(std::ostream& out) : m_out(out) {}

    void begin_object() { open('{'); }
    void end_object() { close('}'); }

    void field(const std::string& key, uint64_t value) {
        prefix(key);
        m_out << value;
    }
    void field(const std::string& key, int value) {
        prefix(key);
        m_out << value;
    }
    void field(const std::string& key, const std::string& value) {
        prefix(key);
        write_string(value);
    }
    void begin_object(const std::string& key) {
        prefix(key);
        open('{');
    }

private:
    void open(char c) {
        if (m_needs_comma) m_out << ",\n"; else m_out << "\n";
        m_out << indent() << c << "\n";
        ++m_depth;
        m_needs_comma = false;
    }
    void close(char c) {
        --m_depth;
        m_out << "\n" << indent() << c;
        m_needs_comma = true;
    }
    void prefix(const std::string& key) {
        if (m_needs_comma) m_out << ",\n"; else m_out << "\n";
        m_out << indent();
        write_string(key);
        m_out << ": ";
        m_needs_comma = true;
    }
    void write_string(const std::string& s) {
        m_out << '"';
        for (char c : s) {
            if (c == '"' || c == '\\') m_out << '\\';
            m_out << c;
        }
        m_out << '"';
    }
    [[nodiscard]] std::string indent() const { return std::string(m_depth * 2, ' '); }

    std::ostream& m_out;
    int m_depth = 0;
    bool m_needs_comma = false;
};

struct MeshTotals {
    uint64_t triangles = 0;
    uint64_t vertices = 0;
};

void accumulate(MeshTotals& totals, const stratum::Mesh& mesh) {
    totals.vertices += mesh.vertices.size();
    totals.triangles += mesh.indices.size() / 3;
}

}  // namespace

int main(int argc, char** argv) {
    // Every log line in stratum_core goes through spdlog, which defaults to
    // stdout. This tool's stdout is a JSON document meant to be diffed
    // byte-for-byte, so parser and quadtree logging is turned off entirely
    // rather than merely redirected -- there is no line of it this tool wants
    // a golden file to depend on.
    spdlog::set_level(spdlog::level::off);

    std::string osm_path;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--osm" && i + 1 < argc) {
            osm_path = argv[++i];
        } else if (arg == "--json") {
            // Accepted, currently the only output mode. See file header.
        } else {
            std::cerr << "stratum_golden: unrecognized argument '" << arg << "'\n";
            std::cerr << "usage: stratum_golden --osm <file> [--json]\n";
            return 2;
        }
    }

    if (osm_path.empty()) {
        std::cerr << "usage: stratum_golden --osm <file> [--json]\n";
        return 2;
    }

    const std::filesystem::path path(osm_path);
    if (!std::filesystem::exists(path)) {
        std::cerr << "stratum_golden: no such file: " << osm_path << "\n";
        return 1;
    }

    const std::string file_hash = sha256_of_file(path);
    if (file_hash.empty()) {
        std::cerr << "stratum_golden: could not read file: " << osm_path << "\n";
        return 1;
    }

    // ── Stage 1: parse, with the editor's default import options ──
    stratum::osm::ParserConfig parser_config;  // default-constructed == editor default
    stratum::osm::OSMParser parser;
    parser.set_config(parser_config);
    if (!parser.parse(path)) {
        std::cerr << "stratum_golden: parse failed: " << parser.get_error() << "\n";
        return 1;
    }
    const stratum::osm::ParsedOSMData& data = parser.get_data();

    // ── Stage 2: road network, with the editor's default config and no terrain ──
    stratum::osm::road::RoadNetworkConfig road_config;  // height_sampler stays null: no terrain
    stratum::osm::road::RoadNetworkBuilder builder;
    stratum::osm::road::RoadNetwork network = builder.build(data, road_config);
    const stratum::osm::road::RoadGraph::Stats graph_stats = builder.graph().stats();
    const auto network_stats = network.stats;       // copy before pieces is moved out below
    const auto junction_stats = network.junction_stats;

    // ── Stage 3: spatial index, chunk LOD on with default config (editor default) ──
    stratum::osm::QuadTree quadtree;
    quadtree.init(data);
    quadtree.assign_data(data);
    quadtree.set_chunk_lod(true, stratum::osm::road::ChunkLodConfig{});
    quadtree.assign_road_pieces(std::move(network.pieces));

    // ── Stage 4: per-leaf building/area mesh build (core-only; no GPU upload) ──
    std::vector<stratum::osm::QuadTreeNode*> leaves = quadtree.get_all_leaves();
    for (auto* leaf : leaves) {
        quadtree.queue_node_build_async(leaf);
    }
    size_t built = 0;
    while (built < leaves.size()) {
        built += quadtree.poll_async_builds();
        if (built < leaves.size()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    MeshTotals road_totals, building_totals, area_totals;
    for (const auto* leaf : leaves) {
        if (leaf->has_road_lod() && !leaf->road_lod.levels.empty()) {
            accumulate(road_totals, leaf->road_lod.levels[0]);
        } else {
            for (const auto& mesh : leaf->road_meshes) accumulate(road_totals, mesh);
        }
        for (const auto& mesh : leaf->building_meshes) accumulate(building_totals, mesh);
        for (const auto& mesh : leaf->area_meshes) accumulate(area_totals, mesh);
    }

    // ── Report ──
    JsonWriter json(std::cout);
    json.begin_object();
    json.field("file", path.filename().string());
    json.field("sha256", file_hash);

    json.begin_object("parser");
    json.field("nodes", static_cast<uint64_t>(data.stats.total_nodes));
    json.field("ways", static_cast<uint64_t>(data.stats.total_ways));
    json.field("relations", static_cast<uint64_t>(data.stats.total_relations));
    json.field("buildings", static_cast<uint64_t>(data.buildings.size()));
    json.field("roads", static_cast<uint64_t>(data.roads.size()));
    json.field("areas", static_cast<uint64_t>(data.areas.size()));
    json.end_object();

    json.begin_object("road_graph");
    json.field("nodes", static_cast<uint64_t>(graph_stats.nodes));
    json.field("edges", static_cast<uint64_t>(graph_stats.edges));
    json.field("junctions", static_cast<uint64_t>(graph_stats.junctions));
    json.field("dead_ends", static_cast<uint64_t>(graph_stats.dead_ends));
    json.field("continuations", static_cast<uint64_t>(graph_stats.continuations));
    json.field("roundabout_edges", static_cast<uint64_t>(graph_stats.roundabout_edges));
    json.field("layer_split_nodes", static_cast<uint64_t>(graph_stats.layer_split_nodes));
    json.end_object();

    json.begin_object("road_network");
    json.field("edges", static_cast<uint64_t>(network_stats.edges));
    json.field("pieces", static_cast<uint64_t>(network_stats.pieces));
    json.field("vertices", static_cast<uint64_t>(network_stats.vertices));
    json.field("triangles", static_cast<uint64_t>(network_stats.triangles));
    json.field("skipped_edges", static_cast<uint64_t>(network_stats.skipped_edges));
    json.field("elevated_edges", static_cast<uint64_t>(network_stats.elevated_edges));
    json.field("junction_pieces", static_cast<uint64_t>(network_stats.junction_pieces));
    json.field("trimmed_edges", static_cast<uint64_t>(network_stats.trimmed_edges));
    json.field("trimmed_away_edges", static_cast<uint64_t>(network_stats.trimmed_away_edges));
    json.field("markings_pieces", static_cast<uint64_t>(network_stats.markings_pieces));
    json.field("crossings", static_cast<uint64_t>(network_stats.crossings));
    json.field("dropped_kerb_spans", static_cast<uint64_t>(network_stats.dropped_kerb_spans));
    json.field("bridges", static_cast<uint64_t>(network_stats.bridges));
    json.field("tunnels", static_cast<uint64_t>(network_stats.tunnels));
    json.field("deduped_sidewalks", static_cast<uint64_t>(network_stats.deduped_sidewalks));
    json.field("vertices_welded", static_cast<uint64_t>(network_stats.vertices_welded));
    json.field("vertices_dropped", static_cast<uint64_t>(network_stats.vertices_dropped));
    json.field("triangles_before_lod", static_cast<uint64_t>(network_stats.triangles_before_lod));
    json.field("triangles_after_lod", static_cast<uint64_t>(network_stats.triangles_after_lod));
    json.field("collision_triangles", static_cast<uint64_t>(network_stats.collision_triangles));
    json.field("stations_before", static_cast<uint64_t>(network_stats.stations_before));
    json.field("stations_after", static_cast<uint64_t>(network_stats.stations_after));
    json.field("quads_merged", static_cast<uint64_t>(network_stats.quads_merged));
    json.field("triangles_before_tess", static_cast<uint64_t>(network_stats.triangles_before_tess));
    json.field("corridor_kerb_edges", static_cast<uint64_t>(network_stats.corridor_kerb_edges));
    json.end_object();

    json.begin_object("junction_stats");
    json.field("junctions", static_cast<uint64_t>(junction_stats.junctions));
    json.field("roundabouts", static_cast<uint64_t>(junction_stats.roundabouts));
    json.field("tapers", static_cast<uint64_t>(junction_stats.tapers));
    json.field("dead_ends", static_cast<uint64_t>(junction_stats.dead_ends));
    json.field("degenerate", static_cast<uint64_t>(junction_stats.degenerate));
    json.field("merged_into_neighbour", static_cast<uint64_t>(junction_stats.merged_into_neighbour));
    json.field("self_intersecting", static_cast<uint64_t>(junction_stats.self_intersecting));
    json.field("over_trimmed_edges", static_cast<uint64_t>(junction_stats.over_trimmed_edges));
    json.end_object();

    json.begin_object("quadtree");
    json.field("leaf_count", static_cast<uint64_t>(quadtree.leaf_count()));
    json.field("total_roads", static_cast<uint64_t>(quadtree.total_roads()));
    json.field("total_buildings", static_cast<uint64_t>(quadtree.total_buildings()));
    json.field("total_areas", static_cast<uint64_t>(quadtree.total_areas()));
    json.field("max_depth", static_cast<int>(quadtree.max_depth()));
    json.end_object();

    json.begin_object("meshes");
    json.begin_object("roads");
    json.field("triangles", road_totals.triangles);
    json.field("vertices", road_totals.vertices);
    json.end_object();
    json.begin_object("buildings");
    json.field("triangles", building_totals.triangles);
    json.field("vertices", building_totals.vertices);
    json.end_object();
    json.begin_object("areas");
    json.field("triangles", area_totals.triangles);
    json.field("vertices", area_totals.vertices);
    json.end_object();
    json.end_object();

    json.end_object();
    std::cout << "\n";

    return 0;
}
