// SPDX-License-Identifier: Apache-2.0
// Copyright 2026 Seamus Mullan and the Stratum contributors

/**
 * @file asset_paths.hpp
 * @brief Locating the assets/ tree from a running binary
 * @author Stratum Team
 * @version 0.1.0
 * @date 2026
 *
 * Everything the renderer loads at run time -- the .spv above all -- lives under
 * `assets/`, and where that directory sits relative to the executable depends on
 * how the executable was produced:
 *
 *   - a preset build puts the binary at `build/<preset>/bin/stratum`, so the
 *     source tree's `assets/` is THREE levels up;
 *   - a plain `cmake -B build` puts it at `build/bin/stratum`, two levels up;
 *   - an installed or packaged tree ships `assets/` beside the binary.
 *
 * Hard-coding one of those relative paths breaks the other two, and it breaks
 * them at run time with a missing-file error rather than at build time. So the
 * root is SEARCHED for instead: walk up from the executable, and take the first
 * directory that actually contains `assets/shaders`. The marker is the shaders
 * subdirectory rather than `assets` alone, because an empty or unrelated
 * `assets` directory higher up would otherwise win and then fail on every load.
 *
 * `STRATUM_ASSET_ROOT` overrides the search outright. It is the escape hatch for
 * a layout the walk cannot reach -- assets on another volume, or a test run
 * against a tree that is not the one the binary was built from.
 */

#pragma once

#include <string>
#include <string_view>

namespace stratum {

/**
 * @brief Absolute path to the `assets` directory, WITHOUT a trailing separator.
 *
 * Resolved once on the first call and cached; the answer cannot change while the
 * process runs. Returns the best guess rather than failing when no candidate
 * contains `assets/shaders`, so the caller reports a missing FILE -- which names
 * the path it tried -- instead of this function reporting a missing directory.
 */
const std::string& asset_root();

/**
 * @brief Absolute path to @p relative under the `assets` directory.
 * @param relative Path relative to `assets`, e.g. `"shaders/mesh.vert.spv"`.
 */
std::string asset_path(std::string_view relative);

/**
 * @brief Absolute path to @p name under `assets/shaders`.
 * @param name Bare file name, e.g. `"mesh.vert.spv"`.
 */
std::string shader_path(std::string_view name);

}  // namespace stratum
