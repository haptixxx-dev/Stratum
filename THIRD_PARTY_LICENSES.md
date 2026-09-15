# Third-party licences

Stratum is licensed under Apache-2.0 (see `LICENSE`). It vendors its
dependencies as git submodules under `external/`. Each keeps its own licence;
none of them is copyleft, and all are compatible with redistributing Stratum
under Apache-2.0.

Verified against the `LICENSE` file in each checked-out submodule on
2026-09-15. Re-verify after bumping a submodule.

| Dependency | Licence | Used for |
|---|---|---|
| [assimp](https://github.com/assimp/assimp) | BSD-3-Clause | Model import (editor-side) |
| [basis_universal](https://github.com/BinomialLLC/basis_universal) | Apache-2.0 | GPU texture transcoding. Vendored transitively inside KTX-Software (`external/KTX-Software/external/basis_universal`), not as a direct submodule |
| [bvh](https://github.com/madmann91/bvh) | MIT | Ray/scene acceleration for the AO baker |
| [Clipper2](https://github.com/AngusJohnson/Clipper2) | BSL-1.0 | Polygon clipping and offsetting |
| [draco](https://github.com/google/draco) | Apache-2.0 | Mesh compression (linked into core, not yet used) |
| [earcut.hpp](https://github.com/mapbox/earcut.hpp) | ISC | Polygon triangulation |
| [enkiTS](https://github.com/dougbinks/enkiTS) | Zlib | Task scheduler |
| [EnTT](https://github.com/skypjack/entt) | MIT | ECS scene management |
| [glm](https://github.com/g-truc/glm) | MIT (dual with the Happy Bunny licence) | Vector and matrix maths |
| [im3d](https://github.com/john-chapman/im3d) | MIT | Immediate-mode 3D gizmos and debug drawing |
| [Dear ImGui](https://github.com/ocornut/imgui) | MIT | Editor UI |
| [ImGuiColorTextEdit](https://github.com/BalazsJako/ImGuiColorTextEdit) | MIT | Text editor widget |
| [ImGuiFileDialog](https://github.com/aiekick/ImGuiFileDialog) | MIT | File picker |
| [ImGuizmo](https://github.com/CedricGuillemet/ImGuizmo) | MIT | Transform gizmos |
| [imnodes](https://github.com/Nelarius/imnodes) | MIT | Node-graph widget |
| [nlohmann/json](https://github.com/nlohmann/json) | MIT | JSON, incl. the glTF exporter |
| [KTX-Software](https://github.com/KhronosGroup/KTX-Software) | Apache-2.0 | KTX2 texture containers |
| [libosmium](https://github.com/osmcode/libosmium) | BSL-1.0 | OSM `.osm` and `.pbf` parsing |
| [lz4](https://github.com/lz4/lz4) | BSD-2-Clause (`lib/`) | Block compression. **See note below** |
| [MaterialX](https://github.com/AcademySoftwareFoundation/MaterialX) | Apache-2.0 | Material definitions |
| [meshoptimizer](https://github.com/zeux/meshoptimizer) | MIT | Vertex cache optimisation, simplification |
| [mio](https://github.com/vimpunk/mio) | MIT | Memory-mapped file I/O |
| [parallel-hashmap](https://github.com/greg7mdp/parallel-hashmap) | Apache-2.0 | Fast hash maps |
| [protozero](https://github.com/mapbox/protozero) | BSD-2-Clause | Protobuf decoding for OSM PBF |
| [pybind11](https://github.com/pybind/pybind11) | BSD-3-Clause | Python bindings |
| [SDL](https://github.com/libsdl-org/SDL) | Zlib | Windowing, input, SDL_GPU (Vulkan) |
| [spdlog](https://github.com/gabime/spdlog) | MIT | Logging |
| [stb](https://github.com/nothings/stb) | MIT (dual with public domain / Unlicense) | Image loading and writing |
| [Tracy](https://github.com/wolfpld/tracy) | BSD-3-Clause | Profiler |

## Notes

**lz4 is dual-licensed by directory.** Everything under `lib/` — which is the
only part Stratum links (`CMakeLists.txt:215` links the `lz4` library target) —
is BSD-2-Clause. The command-line programs under `programs/` are
GPL-2.0-or-later. Do not link or redistribute the lz4 CLI programs with
Stratum, and do not switch the CMake target to one built from `programs/`.

**glm and stb are dual-licensed.** Stratum takes glm under the MIT option and
stb under the MIT option. No action is needed, but do not remove the dual-
licence notices from the vendored copies.

**OpenStreetMap data is not covered by any of the above.** OSM data is licensed
under the ODbL 1.0 by the OpenStreetMap Foundation. Models that Stratum
generates from OSM input are derivative works of that data, and carry the
ODbL's attribution and share-alike obligations. This affects what you ship,
not Stratum's own licence. See https://www.openstreetmap.org/copyright.
