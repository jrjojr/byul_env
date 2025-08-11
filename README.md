# Byul's World – Simulation & Pathfinding Engine

`byul` is a **high-performance simulation engine** that began as a pathfinding core, and has grown into a modular system supporting **maze generation, projectile trajectory prediction, control systems (MPC/PID), and numerical simulation**.

---

## ✨ Features
- **Pathfinding**  
  Supports A*, Dijkstra, BFS, D* Lite for static and dynamic routefinding.
- **Maze Generation**  
  Includes Kruskal, Eller, Binary Tree, and other algorithms.
- **Projectile Prediction**  
  Simulates projectile motion under gravity, wind, and drag using RK4, Verlet, and Euler integrators.
- **Control Systems**  
  Includes PID, Bang-Bang, and MPC controllers.
- **Numerical Engine**  
  Provides 3D math primitives: vec3, quat, dualquat, and dualnumber.
- **Modular Architecture**  
  Organized into modules: `numeq`, `controller`, `trajectory`, `motion_state`, etc.

---

## 📈 Project Timeline
1. **Pathfinding Core**: A*, Dijkstra, D* Lite with PySide6 visualization.
2. **Maze Generator**: Binary Tree → Prim → Eller → Kruskal.
3. **Numerical Kernel**: vec3, quat, dualquat, motion_state.
4. **Control Theory**: PID, Bang-Bang, and MPC control structures.
5. **Trajectory System**: Integrators + impact prediction.

---

## 🛠 Build Instructions
### Linux / macOS
```bash
git clone https://github.com/jrjojr/byul_env.git
cd byul
mkdir build && cd build
cmake ..
make -j$(nproc)
ctest
```

### Windows (MinGW)
```bash
git clone https://github.com/jrjojr/byul_env.git
cd byul
mkdir build && cd build
cmake -G "MinGW Makefiles" ..
mingw32-make -j4
ctest
```

---

## 🧪 Tests
### A* Routefinding
```cpp
TEST_CASE("navsys: find astar") {
    navgrid_t* navgrid = navgrid_create();
    coord_t start = {0, 0};
    coord_t goal = {9, 9};

    for (int y = 1; y < 10; ++y)
        navgrid_block_coord(navgrid, 5, y);

    route_t* p = navsys_find_astar(navgrid, &start, &goal);
    CHECK(route_get_success(p) == true);

    route_print(p);
    navgrid_print_ascii_with_route(navgrid, p, 2);

    route_destroy(p);
    navgrid_destroy(navgrid);
}
```

### Maze Generation (Kruskal)
```cpp
TEST_CASE("Kruskal Algorithm Maze Generation") {
    maze_t* maze = maze_make_kruskal(0, 0, 19, 19);
    REQUIRE(maze != nullptr);

    navgrid_t* navgrid = navgrid_create();
    REQUIRE(navgrid != nullptr);

    maze_apply_to_navgrid(maze, navgrid);
    navgrid_print_ascii(navgrid);

    navgrid_destroy(navgrid);
    maze_destroy(maze);
}
```

### Projectile Prediction (RK4)
```cpp
TEST_CASE("projectile_predict - ground collision") {
    vec3_t start_pos = {0, 500, 0};
    vec3_t target_pos = {0, 0, 0};
    projectile_t proj;
    projectile_init(&proj);
    proj.base.xf.pos = start_pos;

    entity_dynamic_t entdyn;
    entity_dynamic_init(&entdyn);
    entdyn.xf.pos = target_pos;

    projectile_result_t* result = projectile_result_create();
    environ_t env;
    environ_init(&env);

    bool hit = projectile_predict(
        result, &proj, &entdyn, 500.0f, 1.0f, &env, nullptr, guidance_none);

    CHECK(hit == true);
    CHECK(result->bool_impacted == true);
    CHECK(result->impact_time > 0.0f);
    CHECK(result->impact_pos.y == doctest::Approx(0.0f).epsilon(1.0f));

    trajectory_print(result->trajectory);
    char buf[64];
    printf("impact time : %f, impact pos : %s\n", result->impact_time,
           vec3_to_string(&result->impact_pos, 64, buf));
    projectile_result_destroy(result);
}
```

---

## 📘 Why `route`, not `path`?

The term `route` is intentionally used instead of `path`:
- `path` is often associated with filesystems.
- `route` better expresses a **planned motion path** in a game simulation.
- Used consistently across `route_finder`, `dstar_lite`, and `navsys`.

---

## 📩 Contact
**byuldev@outlook.kr**

---

## 📄 License
- **Byul World Public License v1.0**  
  - Free for personal use and academic research  
  - Commercial use is prohibited  
  - See the LICENSE file for details

---

## 💬 Developer’s Note
> `byul` is not just a pathfinding tool. It has become a **real-time simulation core** 
> for creating the future foundation of **Byul's World**.
