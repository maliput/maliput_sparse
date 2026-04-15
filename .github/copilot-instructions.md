# GitHub Copilot Onboarding Instructions for maliput_sparse

This document provides coding agents with essential information to work efficiently on this repository without extensive exploration.

## Repository Overview

**maliput_sparse** is a C++ helper library for building [maliput](https://github.com/maliput/maliput) backend implementations based on **waypoints** (polylines), without requiring an analytical model of the road surface. It provides a fluent builder API, geometry primitives, a parser abstraction, and loaders to construct a complete `maliput::api::RoadGeometry` and `maliput::api::RoadNetwork` from discrete point data.

**Technologies:** C++17, CMake, Bazel, ROS 2 (ament/foxy), Google Test, Eigen3 (via maliput::math)
**Current version:** 0.4.0
**Documentation:** https://maliput.readthedocs.io/
**Known consumers:** [maliput_osm](https://github.com/maliput/maliput_osm) uses maliput_sparse as its geometry backend.

### How It Fits in the Maliput Ecosystem

```
┌─────────────────────────────────────────────────────────────┐
│                    Your Application                         │
│         (uses maliput::api interfaces)                      │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                     maliput (core API)                      │
│  - Abstract API (RoadGeometry, Lane, etc.)                  │
│  - Common utilities, math, routing                          │
│  - Plugin system for loading backends                       │
└─────────────────────────────────────────────────────────────┘
                            │
              ┌─────────────┴─────────────┐
              ▼                           ▼
┌──────────────────────┐    ┌──────────────────────────────┐
│   maliput_sparse     │    │  Analytical backends         │
│   (this repo)        │    │  (maliput_malidrive, etc.)   │
│                      │    │                              │
│  Polyline-based      │    │  Smooth analytical curves    │
│  geometry helper     │    │  from map formats            │
└──────────────────────┘    └──────────────────────────────┘
              │
              ▼
┌──────────────────────┐
│ Concrete backends    │
│ using maliput_sparse │
│                      │
│ • maliput_osm        │
│ • (your backend)     │
└──────────────────────┘
```

maliput_sparse is **not** a standalone backend — it is an intermediate library. A concrete backend (like `maliput_osm`) implements the `maliput_sparse::parser::Parser` interface to provide junction/lane/connection data, and then uses maliput_sparse's builder + loader to produce the final `RoadGeometry`.

## Build System & Commands

### Prerequisites

The repository supports two build systems:
1. **CMake with colcon** (ROS 2 workflow) — Primary development workflow
2. **Bazel** — Alternative build system

Only dependency: **maliput** (the core library).

### CMake/Colcon Build

```bash
# Create workspace
mkdir -p colcon_ws/src && cd colcon_ws/src
git clone https://github.com/maliput/maliput_sparse.git

# Install dependencies
vcs import . < maliput_sparse/.github/dependencies.repos
export ROS_DISTRO=foxy
rosdep update && rosdep install -i -y --rosdistro $ROS_DISTRO --from-paths .

# Build
colcon build --packages-up-to maliput_sparse

# Build with Doxygen docs
colcon build --packages-select maliput_sparse --cmake-args " -DBUILD_DOCS=On"
```

### Bazel Build

```bash
# Build all targets
bazel build //...

# Build specific libraries
bazel build //:geometry
bazel build //:builder
bazel build //:loader
```

### Testing

```bash
# CMake/colcon
colcon test --packages-select maliput_sparse
colcon test-result --verbose

# Bazel
bazel test //...
```

### Code Formatting

```bash
# Check format (CMake builds check automatically via ament_clang_format)
ament_clang_format --config=./.clang-format

# Reformat code
./tools/reformat_code.sh
```

## Project Architecture

### Directory Structure

```
maliput_sparse/
├── include/maliput_sparse/         # Public headers
│   ├── builder/                    # Fluent RoadGeometry builder API
│   │   └── builder.h
│   ├── geometry/                   # Polyline geometry primitives
│   │   ├── lane_geometry.h         # Lane coordinate frame on polylines
│   │   ├── line_string.h           # Core polyline class (template)
│   │   └── utility/geometry.h      # Computational geometry utilities
│   ├── loader/                     # RoadGeometry / RoadNetwork loaders
│   │   ├── builder_configuration.h
│   │   ├── config.h                # Configuration key constants
│   │   ├── road_geometry_loader.h
│   │   ├── road_network_loader.h
│   ├── parser/                     # Abstract parser interface
│   │   ├── connection.h            # Lane-end connections
│   │   ├── junction.h
│   │   ├── lane.h
│   │   ├── parser.h                # Abstract base class
│   │   ├── segment.h
│   │   └── validator.h             # Adjacency validation
│   └── test_utilties/              # Test utilities (note: typo "utilties")
│       └── maliput_sparse_types_compare.h
├── src/                            # Implementation files
│   ├── base/                       # Internal Lane & RoadGeometry
│   │   ├── lane.h / lane.cc
│   │   └── road_geometry.h
│   ├── builder/builder.cc
│   ├── geometry/                   # Geometry implementations
│   │   ├── lane_geometry.cc
│   │   ├── line_string.cc
│   │   └── utility/geometry.cc
│   ├── loader/                     # Loader implementations
│   │   ├── builder_configuration.cc
│   │   ├── road_geometry_loader.cc
│   │   ├── road_network_loader.cc
│   └── parser/                     # Parser implementations
│       ├── connection.cc
│       ├── lane.cc
│       ├── validation_methods.h
│       └── validator.cc
├── test/                           # Unit tests (mirrors src/ structure)
├── cmake/                          # CMake config (DefaultCFlags, Sanitizers)
├── bazel/                          # Bazel config (variables.bzl)
└── tools/                          # Dev scripts (reformat_code.sh)
```

### Component Dependency Graph

```
geometry  ← lowest level: LineString, LaneGeometry, utility functions
    ↑
base         ← extends geometry_base::Lane/RoadGeometry using LaneGeometry
    ↑
parser       ← abstract data model (Junction, Segment, Lane, Connection, Validator)
    ↑          uses geometry for validation (ComputeDistance)
builder      ← fluent API consuming parser types to construct base objects
    ↑
loader       ← top level: takes a Parser + config, runs validation, uses builder
```

### Library Targets (CMake aliases / Bazel labels)

| CMake alias | Bazel label | Description |
|-------------|-------------|-------------|
| `maliput_sparse::geometry` | `//:geometry` | LineString, LaneGeometry, utility functions |
| `maliput_sparse::base` | `//:base` | Internal Lane and RoadGeometry implementations |
| `maliput_sparse::parser` | `//:parser` | Abstract parser types and validator |
| `maliput_sparse::builder` | `//:builder` | Fluent builder API |
| `maliput_sparse::loader` | `//:loader` | RoadGeometry and RoadNetwork loaders |
| `maliput_sparse::test_utilities` | `//:test_utilities` | Test comparison helpers |

### Key Components in Detail

#### `geometry::LineString<CoordinateT>` (line_string.h)
Template polyline class. Requires at least 2 points, auto-removes consecutive duplicates. Internally builds:
- `Segment` structs per edge with arc-length intervals
- `Point` structs associating coordinates with arc-length parameter `p`
- A **KD-tree** for efficient nearest-point queries

Aliases: `LineString2d` (Vector2), `LineString3d` (Vector3).

#### `geometry::LaneGeometry` (lane_geometry.h)
Maps a lane's (p, r, h) coordinate frame onto polylines. Constructed from left/right boundaries (centerline computed automatically) or explicit left/right/center boundaries. Provides:
- `W(p, r, h)` → Inertial position
- `WDot(p)` → Tangent vector
- `Orientation(p)` → roll/pitch/yaw
- `WInverse(xyz)` → (p, r, h) projection
- `RBounds(p)` → lateral bounds at arc-length p

#### `parser::Parser` (parser.h)
**Abstract base class** that concrete backends must implement. Pure virtual methods:
- `DoGetJunctions()` → map of Junction structs (each containing Segments with Lanes)
- `DoGetConnections()` → list of Connection structs (lane-end to lane-end)
- `DoGetGeoReferenceInfo()` → geo-reference string (e.g., proj4)

This is the **primary extension point** — a new backend implements this interface.

#### `builder::RoadGeometryBuilder` (builder.h)
Fluent builder API for constructing `maliput::api::RoadGeometry`:
```cpp
auto rg = RoadGeometryBuilder()
    .Id("my_road")
    .LinearTolerance(1e-3)
    .AngularTolerance(1e-3)
    .StartJunction("j0")
      .StartSegment("s0")
        .StartLane("l0")
          .HeightBounds({0., 5.})
          .StartLaneGeometry()
            .LeftLineString(left_ls)
            .RightLineString(right_ls)
            .EndLaneGeometry()
          .EndLane()
        .EndSegment()
      .EndJunction()
    .StartBranchPoints()
      .Connect("l0", LaneEnd::Which::kStart, "l1", LaneEnd::Which::kFinish)
      .EndBranchPoints()
    .Build();
```

#### `loader::RoadGeometryLoader` / `RoadNetworkLoader` (loader/)
Functor classes that orchestrate the full pipeline: parse → validate → build.
- `RoadGeometryLoader`: Takes a `Parser` + `BuilderConfiguration`, runs `Validator`, builds `RoadGeometry`
- `RoadNetworkLoader`: Composes both loaders above, and also loads auxiliary books (TrafficLightBook, PhaseRingBook, IntersectionBook) from file paths in configuration

#### `parser::Validator` (validator.h)
Validates parsed data before building. Two validation types:
- `kLogicalLaneAdjacency` — checks lane ID references, left/right ordering consistency
- `kGeometricalLaneAdjacency` — checks that adjacent lanes share boundary geometry (uses `ComputeDistance`)

Geometrical validation depends on logical validation passing first.

## Maliput Feature Coverage

### Fully Implemented

| Feature | Notes |
|---------|-------|
| **RoadGeometry** | Via `geometry_base::RoadGeometry` with `DoGeoReferenceInfo()` override |
| **Junction / Segment / Lane / BranchPoint** | Full topology graph construction |
| **Lane coordinate frame** (s, r, h) | Polyline-based interpolation; p ≈ s (arc length) |
| **Position transforms** | `ToBackendPosition`, `ToLanePosition`, `ToSegmentPosition` |
| **Lane orientation** | Roll (superelevation), pitch (slope), yaw (heading) from polyline |
| **Motion derivatives** | `EvalMotionDerivatives` implemented |
| **Lane bounds / Segment bounds** | Computed from polyline boundaries; segment bounds traverse adjacent lanes |
| **Elevation bounds** | Simplified constant `HBounds` per lane |
| **BranchPoint connectivity** | Full A-side/B-side grouping with default branch connections |
| **Validation** | Logical and geometrical lane adjacency checks |
| **RoadNetwork loading** | With TrafficLightBook, RoadRulebook, PhaseRingBook, IntersectionBook, RuleRegistry from files |
| **KD-tree acceleration** | For position queries (`BruteForceStrategy` and `KdTreeStrategy`) |
| **Geo-reference info** | Passed through from parser to RoadGeometry |

### Uses Generic Maliput (Not Custom)

These features work via maliput's generic/manual implementations, loaded from configuration files. No backend-specific logic:

| Feature | Notes |
|---------|-------|
| **Routing** | Uses generic BranchPoint-based routing from maliput core |
| **Rules** (direction usage, ROW, etc.) | Loaded via `maliput::base::ManualRulebook` from YAML files when not provided by the parser |
| **Traffic lights** | Loaded via `maliput::base::ManualTrafficLightBook` |
| **Phase rings** | Loaded via `maliput::base::ManualPhaseRingBook` |
| **Intersections** | Loaded via `maliput::base` intersection book |
| **Rule state providers** | Generic `ManualDiscreteValueRuleStateProvider`, `ManualRangeValueRuleStateProvider`, `ManualPhaseProvider` |

### Not Implemented / Limitations

| Feature | Details |
|---------|---------|
| **Plugin registration** | No `MALIPUT_PLUGIN` macro — not discoverable via maliput's plugin system |
| **LaneBoundary objects** | Returns `nullptr` for lane boundary queries |
| **Curvature** | Always returns 0 (piecewise-linear geometry has no curvature) |
| **Analytical surfaces** | By design — uses polylines only |
| **Custom routing** | No cost-based or optimized routing |
| **Custom traffic/rules engines** | All rule management delegated to maliput::base |

## How to Build a New Backend Using maliput_sparse

To create a new maliput backend from waypoint/polyline data:

1. **Implement `maliput_sparse::parser::Parser`**:
   ```cpp
   class MyParser : public maliput_sparse::parser::Parser {
    private:
     std::unordered_map<Junction::Id, Junction> DoGetJunctions() const override {
       // Parse your data source into Junction/Segment/Lane structs
       // Each Lane has left/right LineString3d boundaries
     }
     std::vector<Connection> DoGetConnections() const override {
       // Return lane-end to lane-end connectivity
     }
     std::string DoGetGeoReferenceInfo() const override {
       // Return proj4 string or similar
     }
   };
   ```

2. **Use the loader to build**:
   ```cpp
   MyParser parser(/* your data source */);
   maliput_sparse::loader::BuilderConfiguration config =
       maliput_sparse::loader::BuilderConfiguration::FromMap({
           {"road_geometry_id", "my_road"},
           {"linear_tolerance", "1e-3"},
       });

   // Option A: RoadGeometry only
   auto rg = maliput_sparse::loader::RoadGeometryLoader(parser, config)();

   // Option B: Full RoadNetwork with rules
   auto rn = maliput_sparse::loader::RoadNetworkLoader(parser, config)();
   ```

3. **Lane data format**: Each `parser::Lane` requires:
   - `id`: Unique lane identifier
   - `left`, `right`: `LineString3d` boundaries (at least 2 points each)
   - `left_lane_id`, `right_lane_id`: Optional adjacent lane references
   - `successors`, `predecessors`: Lane-end connectivity maps

4. **Segments**: Lanes within a segment must be ordered **right-to-left** (increasing lateral offset).

5. **Validation**: The loader automatically validates via `Validator` before building. Logical adjacency is always checked; geometrical adjacency validates shared boundary distances.

### Adding Plugin Support

To make your backend discoverable via maliput's plugin system, you would need to:
1. Create a shared library that registers with `MALIPUT_PLUGIN` macro (from `maliput/plugin/`)
2. Implement `maliput::plugin::RoadNetworkLoader` interface
3. Install the plugin `.so` to a path discoverable by `MaliputPluginManager`

This is **not** done in maliput_sparse itself but in the concrete backend (e.g., `maliput_osm`).

## Code Style Guidelines

Follows the same conventions as the maliput core repo. See the [maliput copilot-instructions.md](https://github.com/maliput/maliput/blob/main/.github/copilot-instructions.md) for full details.

### Quick Reference

- **C++ standard:** C++17
- **Line length:** 120 characters
- **Naming:**
  - Classes/Structs: `PascalCase`
  - Public methods: `PascalCase` (e.g., `Build()`, `StartJunction()`)
  - Variables: `snake_case`
  - Member variables: `snake_case_` (trailing underscore)
  - Constants: `kPascalCase`
  - Namespaces: `lowercase`
- **Header guards:** `#pragma once`
- **Error handling:** `MALIPUT_VALIDATE(condition, "message")`, `MALIPUT_THROW_MESSAGE("msg")`
- **Copy semantics:** Use `MALIPUT_NO_COPY_NO_MOVE_NO_ASSIGN(ClassName)` or `MALIPUT_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(ClassName)`
- **Logging:** `maliput::log()->info(...)`, `maliput::log()->warn(...)`, etc.
- **Documentation:** Doxygen-style `///` comments with `@param`, `@returns`, `@throws`

### Namespace

All code lives under `namespace maliput_sparse { ... }` with sub-namespaces:
- `maliput_sparse::builder`
- `maliput_sparse::geometry`
- `maliput_sparse::loader`
- `maliput_sparse::parser`
- `maliput_sparse::test_utilities`

Internal base classes use an anonymous `namespace` within `maliput_sparse`.

## Design Patterns

| Pattern | Where | Purpose |
|---------|-------|---------|
| **Fluent Builder** | `builder/builder.h` | Declarative RoadGeometry construction via chained method calls |
| **Passkey** | `builder/builder.h` | Controls access between nested builder classes |
| **Template Method** | `parser/parser.h` | Parser defines `GetJunctions()` calling virtual `DoGetJunctions()` |
| **KD-Tree** | `geometry/line_string.h` | Spatial acceleration for nearest-point queries |
| **Functor** | `loader/*.h` | Loaders are callable objects (operator()) |
| **Validator** | `parser/validator.h` | Pre-build validation with dependency-ordered check types |

## Testing Patterns

Tests use Google Test and follow maliput conventions:

```cpp
#include <gtest/gtest.h>
#include "maliput_sparse/geometry/line_string.h"

namespace maliput_sparse {
namespace geometry {
namespace test {
namespace {

GTEST_TEST(LineString3dTest, Construction) {
  const LineString3d dut{{0., 0., 0.}, {1., 1., 0.}};
  EXPECT_EQ(2u, dut.size());
}

}  // namespace
}  // namespace test
}  // namespace geometry
}  // namespace maliput_sparse
```

Test files mirror the `src/` structure under `test/`:
- `test/base/lane_test.cc`, `test/base/road_geometry_test.cc`
- `test/builder/builder_test.cc`
- `test/geometry/lane_geometry_test.cc`, `test/geometry/line_string_test.cc`
- `test/geometry/utility/geometry_test.cc`
- `test/parser/lane_test.cc`, `test/parser/segment_test.cc`, `test/parser/validator_test.cc`
- `test/loader/road_network_loader_test.cc`

## CI/CD & Validation

### GitHub Workflows (`.github/workflows/`)

| Workflow | Trigger | Description |
|----------|---------|-------------|
| `build.yml` | Push/PR to main | Bazel build+test, CMake build+test via colcon |
| `bazel.yml` | Push/PR to main | Tests Bazel 7.x and 8.x compatibility |
| `build_macos.yaml` | Push/PR to main | macOS builds |
| `sanitizers.yml` | `do-clang-test` label | ASan, TSan, UBSan builds with clang-8 |
| `scan_build.yml` | `do-clang-test` label | Static analysis with scan-build |
| `containers.yml` | Workflow dispatch | Builds CI container images |
| `release.yaml` | Version tag | Release automation |
| `triage.yml` | Issues | Auto-triage labeling |

### Validation Checklist

Before submitting changes:

1. **Format:**
   ```bash
   ament_clang_format --config=./.clang-format
   ```

2. **Build & Test (CMake):**
   ```bash
   colcon build --packages-select maliput_sparse
   colcon test --packages-select maliput_sparse
   colcon test-result --verbose
   ```

3. **Build & Test (Bazel):**
   ```bash
   bazel build //...
   bazel test //...
   ```

## Common Gotchas

1. **Lanes are ordered right-to-left** within a Segment. The first lane in the vector has the most negative lateral offset.
2. **Centerline is auto-computed** from left/right boundaries if not provided explicitly. The algorithm is inspired by Lanelet2.
3. **Curvature is always zero** — this is by design for piecewise-linear geometry. Do not try to compute smooth curvature.
4. **The `test_utilties` directory has a typo** ("utilties" not "utilities") — this is established and should not be renamed without coordinating dependents.
5. **BranchPoint construction** groups lane-ends into A/B sides using connection analysis — the algorithm walks connections to build consistent groupings.
6. **KDTree initialization** uses a sampling step of `0.1 * average_lane_length` for the spatial index.
7. **Bidirectional connections** — `Connection{A, B}` equals `Connection{B, A}`. Don't add both.
8. **Parser is const** — `DoGet*()` methods are `const`. Parse data in the constructor or lazily cache.

## Related Projects

- [maliput](https://github.com/maliput/maliput) — Core abstract API (only dependency)
- [maliput_osm](https://github.com/maliput/maliput_osm) — OpenStreetMap backend built on maliput_sparse
- [maliput_malidrive](https://github.com/maliput/maliput_malidrive) — OpenDRIVE backend (analytical, not using maliput_sparse)
- [maliput_integration](https://github.com/maliput/maliput_integration) — Integration examples

## License

BSD 3-Clause License. See [LICENSE](../LICENSE) file.

---

**Trust these instructions.** Only perform additional exploration if information is missing or incorrect.
