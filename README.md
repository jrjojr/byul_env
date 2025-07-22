# Byul's World – Simulation & Pathfinding Engine

`byul` started as a **pathfinding engine**,  
and has evolved to include **maze generation, projectile trajectory prediction, control systems (MPC/PID), and numerical analysis**,  
making it a **lightweight and high-performance simulation engine**.

> 💖 If you enjoy this project, consider supporting development at [paypal.me/jrjojr](https://paypal.me/jrjojr)

---

## ✨ Key Features
- **Pathfinding**  
  Supports A*, Dijkstra, BFS, D* Lite for both static and dynamic pathfinding.
- **Maze Generation**  
  Includes Binary Tree, Eller, Kruskal, and other maze generation algorithms.
- **Projectile Trajectory Prediction**  
  Predicts projectile paths with gravity, drag, and wind using RK4/Verlet integration.
- **Control Systems**  
  PID and Model Predictive Control (MPC) for path and motion control.
- **Numerical Core**  
  Implements vec3, quat, dualquat, dualnumber-based 3D math operations.
- **Modular Architecture**  
  Includes `numeq`, `controller`, `motion_state`, and `trajectory` as independent modules.

---

## 📜 Project Evolution
1. **Pathfinding Engine**  
   Implemented A*, Dijkstra, D* Lite → 2D testing with `PySide6`.
2. **Maze Generation**  
   Added Binary Tree, Prim, Eller, Kruskal algorithms.
3. **Numerical & Physics Core**  
   Introduced vec3, quat, dualquat, and motion_state for 3D math and physics.
4. **Control Systems**  
   Integrated PID, Bang-Bang, and MPC-based control.
5. **Projectile Trajectory Prediction**  
   Added Euler, Semi-Implicit, Verlet, RK4 integrators with trajectory recording.

---

## 🚀 Build & Run
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

## 🧪 Test Examples
### Pathfinding (A*)
```c
coord_t* start = coord_create_full(0, 0);
coord_t* goal = coord_create_full(9, 9);

REQUIRE_FALSE(coord_equal(start, goal));

std::cout << "default\n";
navgrid_t* m = navgrid_create();
// Insert obstacles (vertical block)
for (int y = 1; y < 10; ++y)
    navgrid_block_coord(m, 5, y);

route_finder_t* a = route_finder_create(m);
route_finder_set_goal(a, goal);
route_finder_set_start(a, start);
route_finder_set_visited_logging(a, true);
route_t* p = nullptr;

p = route_finder_find(a);
REQUIRE(p != nullptr);
CHECK(route_get_success(p) == true);
route_print(p);
navgrid_print_ascii_with_visited_count(m, p, 5);
route_destroy(p);    
route_finder_destroy(a);
navgrid_destroy(m);

coord_destroy(start);
coord_destroy(goal);
```

### Projectile Trajectory Prediction (RK4)
```c
linear_state_t projectile = { {0,0,0}, {10,10,0}, {0,0,0} };
environ_t env = environ_default();
bodyprops_t body = bodyprops_default();

linear_state_t predicted;
numeq_model_predict_rk4(1.0f, &projectile, &env, &body, 60, &predicted);

printf("Predicted position: %.2f, %.2f, %.2f\n",
       predicted.position.x, predicted.position.y, predicted.position.z);
```

---

## 📘 Why `route` instead of `path`?

In Byul's World, the term **`route`** is used instead of `path` for the following reasons:

- `path` is commonly associated with filesystem paths, which can be confusing.
- `route` better reflects the **"strategic path to a destination"** in the context of games.
- The distinction between **static search (`route_finder`)** and **dynamic search (`dstar_lite`)** is clearer with `route`.

> This terminology is part of the design philosophy and is used consistently across the project.

---

Contact: **byuldev@outlook.kr**

---

## 📄 License
- **Byul World Public License v1.0**  
  - Free for personal learning and research  
  - Commercial use and redistribution are prohibited  
  - See the LICENSE file for details

---

## 💬 Developer’s Note
> `byul` has evolved beyond simple pathfinding to become a **real-time simulation engine**.  
> From pathfinding to projectile trajectory prediction,  
> this project serves as the foundation for creating **Byul's World**.
