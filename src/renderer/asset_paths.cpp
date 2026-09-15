// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file asset_paths.cpp
 * @brief The upward search for the assets/ tree
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Read asset_paths.hpp first: it carries the reason the root is searched for
 * rather than written down.
 */

#include "renderer/asset_paths.hpp"

#include <SDL3/SDL.h>
#include <spdlog/spdlog.h>

#include <filesystem>

namespace stratum {

namespace {

// How far up from the executable the search goes. Four covers every layout in
// the table in the header -- installed (0), plain build (2), preset build (3) --
// with one level of headroom, and stops the walk well before it reaches the
// filesystem root on a machine where none of them apply.
constexpr int kMaxSearchDepth = 4;

// The directory that proves a candidate is the real assets tree. See the header
// for why this is not just "assets".
constexpr const char* kMarker = "shaders";

std::filesystem::path find_asset_root() {
    namespace fs = std::filesystem;

    // The override wins unconditionally, including over a search that would have
    // succeeded: a run that sets it is a run that means it.
    if (const char* override_root = SDL_getenv("STRATUM_ASSET_ROOT")) {
        if (*override_root != '\0') {
            spdlog::info("Asset root from STRATUM_ASSET_ROOT: {}", override_root);
            return fs::path(override_root);
        }
    }

    const char* base = SDL_GetBasePath();
    fs::path dir = base ? fs::path(base) : fs::current_path();

    std::error_code ec;
    fs::path first_candidate;
    for (int depth = 0; depth <= kMaxSearchDepth; ++depth) {
        const fs::path candidate = dir / "assets";
        if (depth == 0) {
            first_candidate = candidate;
        }
        if (fs::is_directory(candidate / kMarker, ec)) {
            return candidate;
        }
        const fs::path parent = dir.parent_path();
        if (parent.empty() || parent == dir) {
            break;
        }
        dir = parent;
    }

    // Nothing matched. Hand back the beside-the-binary path so the failure
    // surfaces as a named missing FILE from the loader that asked for it.
    spdlog::warn("No assets/{} found above {} -- falling back to {}", kMarker,
                 base ? base : "the working directory", first_candidate.string());
    return first_candidate;
}

}  // namespace

const std::string& asset_root() {
    // Resolved once. Function-local static initialisation is thread-safe in C++11
    // and later, so the first two renderers to ask cannot race.
    static const std::string root = find_asset_root().string();
    return root;
}

std::string asset_path(std::string_view relative) {
    return (std::filesystem::path(asset_root()) / relative).string();
}

std::string shader_path(std::string_view name) {
    return (std::filesystem::path(asset_root()) / "shaders" / name).string();
}

}  // namespace stratum
