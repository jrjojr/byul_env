# Byul's World / BYUL Engine

Language: [English](README.md) | [한국어](README.ko.md)

> **License notice**
>
> This repository is **source-available, not open source**.
> Personal learning, academic research, and non-commercial technical evaluation are permitted.
> Commercial use, redistribution, resale, public mirroring, and packaging as a library are prohibited without prior written permission.
> See [LICENSE](LICENSE) for the full terms.

`byul` is a C++17-based simulation engine for **Byul's World**.

It is implemented in C++ for internal flexibility, but exposes a C-compatible ABI through `extern "C"` and `BYUL_API` so that the engine can be used as a stable DLL interface from Unreal Engine, C/C++ applications, and other FFI-based environments.

`byul` began as a pathfinding core, but has grown into a modular engine for route planning, maze generation, numerical simulation, dynamic entities, trajectory prediction, projectile simulation, ground interaction, and runtime tick-based updates.

---

## Core Idea

BYUL is not only a pathfinding library.

It is a real-time simulation core that defines how objects in Byul's World:

- find planned routes,
- move through grid and continuous space,
- generate and analyze trajectories,
- react to gravity, wind, drag, and ground,
- predict projectile impact,
- and update over runtime ticks.

The current design keeps the internal modules separated as static libraries, while the final public output is a single shared library:

```text
byul.dll / libbyul.so
```

This keeps development modular and deployment simple.

---

## Implementation Model

BYUL is best described as:

```text
C++17 implementation + C-compatible ABI + single shared library
```

Public headers use `extern "C"` for symbol stability and `BYUL_API` for platform-specific export/import handling.

```c
#ifdef __cplusplus
extern "C" {
#endif

BYUL_API const char* byul_version_string();
BYUL_API void byul_print_version();

#ifdef __cplusplus
}
#endif
```

This means BYUL is not a pure C library. It is a C++ engine DLL with a C ABI boundary.

### ABI Rules

Public API design should follow these rules:

- Do not expose `std::vector`, `std::string`, or C++ classes in public headers.
- Export public functions with `BYUL_API`.
- Keep public functions inside `extern "C"` blocks.
- Objects created by BYUL should be destroyed by BYUL destroy/free functions.
- Do not throw C++ exceptions across the DLL boundary.
- Keep public struct layout changes deliberate and version-aware.

---

## Module Architecture

The engine is assembled from internal modules:

```text
byul
├─ core
├─ numal
├─ rng
├─ console
├─ navsys
├─ balix
├─ entity
├─ projectile
├─ ground
├─ byul_tick
└─ gpu_comp_tester
```

The top-level `byul.h` acts as an umbrella header:

```c
#include "byul_config.h"
#include "numal.h"
#include "navsys.h"
#include "balix.h"
#include "entity.h"
#include "projectile.h"
#include "byul_tick.h"
```

---

## Main Layers

### 1. `navsys` - grid route planning

`navsys` is the grid-based navigation layer.

It contains:

```text
coord
route
navgrid
obstacle
maze
route_finder
dstar_lite
route_carver
```

It supports multiple route search algorithms:

- A*
- BFS
- DFS
- Dijkstra
- Greedy Best First
- Fast Marching
- IDA*
- Fringe Search
- RTA*
- SMA*
- Weighted A*
- D* Lite

The primary output of this layer is `route_t`.

---

### 2. `balix` - continuous simulation layer

`balix` is the continuous motion, physics, control, and numerical layer.

It contains:

```text
bodyprops
environ
collision
numeq
xform
motion_state
trajectory
controller
```

This layer provides:

- physical properties,
- environment data,
- transforms,
- motion states,
- numerical integration,
- PID / MPC control structures,
- collision-related utilities,
- and trajectory storage/export.

---

### 3. `entity` - world object layer

`entity` connects general world objects with dynamic physical state.

Important types include:

```text
entity_t
entity_dynamic_t
```

`entity_dynamic_t` combines:

- base entity data,
- transform,
- physical properties,
- velocity,
- angular velocity,
- and grounded state.

This layer is the bridge between object identity and physical simulation.

---

### 4. `projectile` - projectile simulation layer

`projectile` is the gameplay-oriented projectile layer.

It contains:

```text
projectile_core
propulsion
guidance
projectile_predict
projectile_tick
```

It models a projectile hierarchy using C-style structure composition:

```text
projectile_t
└─ shell_projectile_t
   └─ rocket_t
      └─ missile_t
         └─ patriot_t
```

This layer supports:

- basic projectiles,
- shells with explosion radius,
- rockets with propulsion,
- missiles with guidance,
- advanced target tracking,
- full trajectory prediction,
- impact position/time output,
- and tick-based runtime updates.

---

## Route vs Trajectory

BYUL intentionally separates `route` and `trajectory`.

### `route_t`

`route_t` belongs to `navsys`.

It represents a planned movement route in grid space.

It stores:

- route coordinates,
- visited order,
- visited count,
- route cost,
- success state,
- retry count,
- and average direction data.

Use `route_t` when asking:

```text
Where should this entity go?
```

### `trajectory_t`

`trajectory_t` belongs to the continuous simulation layer.

It stores time-ordered motion samples:

```text
time + motion_state
```

It also stores predicted impact data:

```text
impact_time
impact_pos
```

Use `trajectory_t` when asking:

```text
Where will this object be over time?
```

### Naming Rule

```text
route      = planned movement path in grid/navigation space
trajectory = time-based physical motion curve
track      = actual movement history, if added later
trail      = visual afterimage or rendering effect, if added later
course     = high-level guidance direction, if added later
```

---

## Projectile Prediction Result

Projectile prediction writes into `projectile_result_t`.

The result contains:

```text
start_pos
target_pos
initial_velocity
impact_time
impact_pos
bool_impacted
trajectory
```

This means projectile prediction does not only answer whether a hit occurred. It also stores the full predicted trajectory, which can be printed, sampled, exported, or visualized.

---

## Example: Route Finding

```cpp
TEST_CASE("navsys: find astar") {
    navgrid_t* navgrid = navgrid_create();
    coord_t start = {0, 0};
    coord_t goal = {9, 9};

    for (int y = 1; y < 10; ++y) {
        navgrid_block_coord(navgrid, 5, y);
    }

    route_t* route = navsys_find_astar(navgrid, &start, &goal);
    CHECK(route_get_success(route) == true);

    route_print(route);
    navgrid_print_ascii_with_route(navgrid, route, 2);

    route_destroy(route);
    navgrid_destroy(navgrid);
}
```

---

## Example: Projectile Prediction

```cpp
TEST_CASE("projectile_predict - basic prediction") {
    projectile_t proj;
    projectile_init(&proj);

    proj.base.xf.pos = {0.0f, 10.0f, 0.0f};
    proj.base.velocity = {10.0f, 20.0f, 0.0f};

    entity_dynamic_t target;
    entity_dynamic_init(&target);
    target.xf.pos = {50.0f, 0.0f, 0.0f};

    environ_t env;
    environ_init(&env);

    projectile_result_t* result = projectile_result_create();

    bool hit = projectile_predict(
        &proj,
        0.016f,
        &target,
        &env,
        nullptr,
        nullptr,
        nullptr,
        result
    );

    projectile_result_print_detailed(result);

    projectile_result_destroy(result);
}
```

---

## Build

### Linux

```bash
git clone https://github.com/jrjojr/byul_env.git
cd byul_env/byul
mkdir build
cd build
cmake ..
make -j$(nproc)
ctest
```

### Windows / MinGW

```bash
git clone https://github.com/jrjojr/byul_env.git
cd byul_env/byul
mkdir build
cd build
cmake -G "MinGW Makefiles" ..
mingw32-make -j4
ctest
```

### Windows / MSVC

```bash
git clone https://github.com/jrjojr/byul_env.git
cd byul_env/byul
mkdir build
cd build
cmake ..
cmake --build . --config Release
ctest -C Release
```

---

## Design Direction

Current direction:

```text
Internal modules stay separated.
Final deployment stays as one BYUL shared library.
```

This is intentional.

`navsys`, `balix`, `entity`, `projectile`, `ground`, and `byul_tick` are separate internal modules, but exposing many DLLs too early would increase ABI and deployment complexity.

The preferred structure is:

```text
Development: modular static libraries
Distribution: single byul.dll / libbyul.so
```

Separate DLLs such as `byul_navsys.dll` or `byul_projectile.dll` may be considered later only if external usage clearly requires independent deployment.

---

## Project Status

BYUL is currently strongest in these areas:

- route planning,
- maze generation,
- continuous motion state modeling,
- trajectory storage and export,
- projectile prediction,
- propulsion/guidance structure,
- and DLL-oriented C ABI design.

The project is moving from a route/pathfinding engine toward a broader simulation core for Byul's World.

---

## License

Byul World Source-Available Non-Commercial License v1.0

This repository is source-available, not open source.

- Personal learning, academic research, technical review, feedback, and non-commercial evaluation are permitted.
- GitHub forks are permitted only for personal study, issue reporting, contribution discussion, or non-commercial technical review.
- Commercial use, redistribution, resale, public mirroring, sublicensing, and packaging as a library, SDK, plugin, or service are prohibited without prior written permission.
- Third-party components, if any, remain under their respective licenses.

See [LICENSE](LICENSE) for the full terms.

---

## Contact

**byuldev@outlook.kr**

---

## Developer's Note

BYUL is the movement core of Byul's World.

`route` defines how an entity plans movement through space.

`trajectory` defines how a physical object moves through time.

Together, they form the foundation for a world where navigation, motion, prediction, impact, and runtime simulation are handled by the engine itself.
