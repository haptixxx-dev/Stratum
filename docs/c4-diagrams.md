# Stratum C4 Architecture Diagrams

This document describes the architecture of Stratum using the [C4 model](https://c4model.com/) with [C4-PlantUML](https://github.com/plantuml-stdlib/C4-PlantUML).

## Rendering the Diagrams

The diagrams are defined in `c4-architecture.puml`. To render them:

### Option 1: PlantUML CLI
```bash
# Install PlantUML
brew install plantuml  # macOS
# or download from https://plantuml.com/download

# Render all diagrams to PNG
plantuml -tpng docs/c4-architecture.puml -o images/
```

### Option 2: VS Code Extension
Install the "PlantUML" extension and use `Alt+D` to preview diagrams.

### Option 3: Online
Copy diagram content to [PlantUML Server](https://www.plantuml.com/plantuml/uml/).

---

## Level 1: System Context Diagram

**Diagram:** `C4_SystemContext`

Shows Stratum in the context of its users and external systems.

### Actors
| Actor | Description |
|-------|-------------|
| **Developer** | Creates and edits 3D city maps using the Editor GUI |
| **Game Developer** | Integrates the Core Library into game engines |

### External Systems
| System | Interaction |
|--------|-------------|
| **OpenStreetMap** | Source of geographic data (.osm/.pbf files) |
| **Game Engine (X3)** | Target for exported 3D city data |
| **DCC Tools** | 3D software for further asset editing |

---

## Level 2: Container Diagram

**Diagram:** `C4_Container`

Shows the high-level building blocks of Stratum.

### Containers

| Container | Technology | Purpose |
|-----------|------------|---------|
| **Stratum Editor** | C++20, SDL3, ImGui | Visualization and editing tool |
| **Stratum Core Library** | C++20, Static Library | Pure logic library (no graphics dependencies) |
| **Asset Cache** | File System | Cached tiles, textures, processed data |
| **Project Files** | File System | User projects and settings |

### Key Architectural Decisions

1. **Library-First Design**: Core Library has no graphics API dependencies, enabling integration into any game engine.
2. **Editor as Debug Tool**: The Editor is a thin visualization wrapper around the Core Library.
3. **Tile-Based Loading**: Geographic data is managed in tiles for efficient memory use.

---

## Level 3: Component Diagrams

### Core Library Components

**Diagram:** `C4_Component_CoreLib`

| Component | Technology | Responsibility |
|-----------|------------|----------------|
| **Scene Module** | EnTT, GLM | ECS-based scene graph and spatial indexing |
| **OSM Module** | libosmium, Clipper2 | Parsing and geographic feature extraction |
| **ProcGen Module** | C++20 | Procedural generation algorithms |
| **Mesh Builder** | earcut, meshoptimizer | 3D mesh generation from 2D data |
| **Materials Module** | MaterialX | Material definitions and shader parameters |
| **Export Module** | Assimp | Format conversion (glTF, FBX, OBJ) |

### Editor Components

**Diagram:** `C4_Component_Editor`

| Component | Technology | Responsibility |
|-----------|------------|----------------|
| **Application Core** | SDL3 | Window, events, main loop |
| **Renderer** | SDL_GPU | GPU resource management and rendering |
| **Camera System** | GLM | View matrices and frustum culling |
| **Debug Visualization** | Im3D | Immediate-mode debug rendering |
| **UI Panels** | Dear ImGui | Editor interface panels |
| **Gizmos** | ImGuizmo | 3D manipulation tools |

### Scene Module Components

**Diagram:** `C4_Component_Scene`

| Component | Responsibility |
|-----------|----------------|
| **Entity Registry** | Central ECS storage (EnTT) |
| **Spatial Index** | BVH for efficient spatial queries |
| **Transform System** | Position, rotation, scale management |
| **Component Types** | Data structures for entities |
| **Query System** | Efficient entity filtering |

### OSM Module Components

**Diagram:** `C4_Component_OSM`

| Component | Responsibility |
|-----------|----------------|
| **OSM Parser** | .osm/.pbf file parsing |
| **Coordinate System** | Lat/lon to Cartesian conversion |
| **Feature Classifier** | Tag-based feature categorization |
| **Tile Manager** | Tile loading/unloading, LOD |
| **Road Network Builder** | Road graph construction |
| **Building Extractor** | Footprint extraction |

### ProcGen Module Components

**Diagram:** `C4_Component_ProcGen`

| Component | Responsibility |
|-----------|----------------|
| **Terrain Generator** | Terrain mesh and elevation |
| **Road Network Generator** | Road segments with geometry |
| **Zoning System** | Zone type assignment |
| **Lot Subdivision** | Block to parcel splitting |
| **Building Generator** | 3D building volumes |

### Renderer Module Components

**Diagram:** `C4_Component_Renderer`

| Component | Responsibility |
|-----------|----------------|
| **GPU Device** | SDL_GPU context management |
| **Pipeline Manager** | Shader/pipeline caching |
| **Mesh Renderer** | Buffer management, draw calls |
| **Texture Manager** | GPU texture handling |
| **Camera** | View/projection matrices |
| **Culling System** | Visibility determination |

---

## Dynamic Diagrams

### Application Lifecycle

**Diagram:** `C4_Dynamic_AppLifecycle`

```
1. main() creates Application
2. Application initializes Window (SDL3)
3. Application initializes ImGui context
4. Application initializes Renderer (GPU device)
5. Application enters main loop
6. Loop: Process events -> Update -> Render
7-9. Shutdown in reverse order
```

### OSM Import Flow

**Diagram:** `C4_Dynamic_OSMImport`

```
1. User selects .osm/.pbf file in OSM Panel
2. Panel calls parser.parse_file(path)
3. Parser extracts nodes, ways, relations
4. Features classified by OSM tags
5. Mesh Builder generates geometry
6. Scene populated with entities
7. Import complete notification
```

### Frame Rendering Flow

**Diagram:** `C4_Dynamic_FrameRender`

```
1. Poll SDL events
2. Forward events to ImGui
3. Update scene systems
4. Frustum cull visible entities
5. Submit draw calls to renderer
6. Sort by material/depth
7. Execute render pass
8. Render UI overlay
9. Present frame to display
```

---

## Deployment Diagram

**Diagram:** `C4_Deployment`

Shows how Stratum is deployed on a user's machine:

```
User's Machine
├── Operating System (Windows/macOS/Linux)
│   ├── GPU Driver (Vulkan/Metal/D3D12)
│   │   └── GPU
│   └── File System
│       ├── Projects (user files)
│       ├── Cache (tiles, textures)
│       └── Settings (preferences)
└── Application Directory
    ├── stratum (executable)
    ├── SDL3 (shared library)
    └── Assets/
        ├── Shaders
        ├── Default Materials
        └── UI Assets
```

---

## Component Relationships Summary

```
┌─────────────────────────────────────────────────────────────┐
│                     STRATUM EDITOR                          │
│  ┌─────────┐  ┌──────────┐  ┌─────────┐  ┌──────────────┐  │
│  │ App     │  │ Renderer │  │ Camera  │  │  UI Panels   │  │
│  │ Core    │──│ (SDL_GPU)│──│ System  │──│  (ImGui)     │  │
│  └────┬────┘  └────┬─────┘  └─────────┘  └──────┬───────┘  │
│       │            │                            │          │
└───────┼────────────┼────────────────────────────┼──────────┘
        │            │                            │
        ▼            ▼                            ▼
┌─────────────────────────────────────────────────────────────┐
│                 STRATUM CORE LIBRARY                        │
│  ┌─────────┐  ┌──────────┐  ┌─────────┐  ┌──────────────┐  │
│  │ Scene   │◄─│ OSM      │  │ ProcGen │  │  Materials   │  │
│  │ Module  │  │ Module   │──│ Module  │  │  Module      │  │
│  └────┬────┘  └──────────┘  └─────────┘  └──────────────┘  │
│       │                                                     │
│       ▼                                                     │
│  ┌─────────┐  ┌──────────────┐                             │
│  │ Mesh    │  │   Export     │                             │
│  │ Builder │──│   Module     │                             │
│  └─────────┘  └──────────────┘                             │
└─────────────────────────────────────────────────────────────┘
```

---

## Technology Stack

| Layer | Technology | Purpose |
|-------|------------|---------|
| **Windowing** | SDL3 | Cross-platform window/input/GPU |
| **UI** | Dear ImGui | Immediate-mode editor UI |
| **Rendering** | SDL_GPU | Cross-platform GPU abstraction |
| **ECS** | EnTT | Entity Component System |
| **Math** | GLM | Linear algebra |
| **OSM Parsing** | libosmium | Geographic data parsing |
| **Geometry** | Clipper2, earcut | Polygon ops, triangulation |
| **Optimization** | meshoptimizer | LOD, mesh compression |
| **Materials** | MaterialX | Material definitions |
| **Export** | Assimp | 3D format conversion |
| **Profiling** | Tracy | Performance analysis |
| **Scripting** | pybind11 | Python bindings |
