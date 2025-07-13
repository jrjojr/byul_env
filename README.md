# 🌟 Byul's World – Path Finding Engine

The `byul` module is a lightweight, high-performance engine  
designed for pathfinding in real-time simulation games.  
It focuses on three primary features: maze generation, route finding,  
and dynamic path recalculation, centered around the modules  
`maze`, `dstar_lite`, and `route_finder`.

---

## 🧩 Core Components

### 🌀 1. Maze Generator – `maze/`

A standalone maze generator that creates complex structures  
which can be inserted into a map as obstacles.

- Supported algorithms: Binary Tree, Prim, Eller, Kruskal, etc.
- Usage: generate with `maze_generate()` → apply with `maze_apply_to_map()`
- Purpose: map setup, stage design, automated testing

#### Key Interface:
```c
maze_t* maze_new();
void maze_generate(maze_t* maze, maze_type_t type);
void maze_apply_to_map(const maze_t* maze, map_t* map);
void maze_free(maze_t*);
```

---

### 🧠 2. D* Lite Module – `dstar_lite/`

A fully implemented **D* Lite algorithm** for dynamic environments.  
The `dstar_lite_t` structure maintains internal state and  
automatically recalculates paths in response to obstacle changes.

- Key sorting: based on `dstar_lite_key`
- Priority queue: handled by `dstar_lite_pqueue`
- Heuristics: implemented via `dstar_lite_utils`

> Not a one-shot searcher — it's a **persistent route engine** that responds to change.

---

### 🚦 3. Static Route Finder – `route_finder/`

A unified interface for static algorithms including  
A*, Dijkstra, BFS, JPS and more.  
Best suited for single-use searches on fixed maps.

- Uses `route_finder_t` to select algorithm and extract routes
- Main API: `route_finder_find()`

---

## 🛠 Supporting Modules

### 📌 `coord/`
- 2D integer coordinate structure (`coord_t`)
- Hash support (`coord_hash`), list operations (`coord_list`)

### 📌 `map/`
- `map_t`: grid-based structure
- Determines walkability and obstacle positions
- Utility: `map_is_blocked(coord_t*)`

### 📌 `cost_coord_pq/`
- Priority queue based on `coord + cost`
- Used during route expansion
- Custom lightweight heap implementation

---

## 📘 Terminology – Why use `route` instead of `path`?

In Byul's World, we prefer **`route`** over `path`. Here's why:

- `path` is ambiguous, often interpreted as a file system path
- In a game context, **"route" means intentional movement toward a goal"**
- When distinguishing between static (`route_finder`) and dynamic (`dstar_lite`) logic,
  the term `route` conveys **purpose, direction, and designed flow**

> This is a deliberate design choice — all modules use `route` consistently.

---

## ▶️ Usage Example

### 🔹 Static A* Pathfinding

```c
coord_t* start = coord_new_full(0, 0);
coord_t* goal = coord_new_full(9, 9);

REQUIRE_FALSE(coord_equal(start, goal));

std::cout << "default\n";
map_t* m = map_new();
// Insert vertical wall
for (int y = 1; y < 10; ++y)
    map_block_coord(m, 5, y);

route_finder_t* a = route_finder_new(m);
route_finder_set_goal(a, goal);
route_finder_set_start(a, start);
route_finder_set_visited_logging(a, true);

route_t* p = route_finder_find(a);
REQUIRE(p != nullptr);
CHECK(route_get_success(p) == true);
route_print(p);
map_print_ascii_with_visited_count(m, p, 5);

route_free(p);    
route_finder_free(a);
map_free(m);
```

### 🧩 Summary

| Situation | Suggested Algorithm | Description |
|----------|----------------------|-------------|
| Obstacles change frequently | D* Lite | Persistent state, real-time adaptation |
| Fixed map | A*, Dijkstra, etc. | Fast one-shot searches |
| Unified interface | `route_finder` | Switch between algorithms automatically |

---

## ⚙️ Build Instructions

This project uses **CMake** for cross-platform builds.

```bash
git clone https://github.com/your-id/byul.git
cd byul
mkdir build && cd build
cmake ..
make -j$(nproc)
```

> Windows: `byul.dll` / Linux: `libbyul.so` or `libbyul.a`

---

## 🧪 Testing

```bash
cd byul/maze/tests
make
./test_maze
```

> All modules include unit tests using `doctest`.

---

## 📄 License – Byul World Public License v1.0

| Allowed                         | Prohibited                                     |
|---------------------------------|------------------------------------------------|
| Personal learning & analysis    | Commercial use (direct/indirect profit)        |
| Academic or demo presentation   | Redistribution, packaging, public hosting      |
| Credited reference              | Removing or misrepresenting authorship         |

Contact: **byuldev@outlook.kr**

> See LICENSE for full legal terms.

---

## 💬 Developer Note

This is not just a pathfinding module.  
It is the **core logic that gives life and movement** to Byul's World.

Mazes are the art of obstacles.  
Routes are the will to move.  
Finding them is how we ask the world a question.

© 2025 ByulPapa (byuldev@outlook.kr)  
All rights reserved.