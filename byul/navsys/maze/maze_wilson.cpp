#include "maze.h"
#include "obstacle.h"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <random>
#include <ctime>
#include <algorithm>

static const int WALL = 1;
static const int PASSAGE = 0;

struct Cell {
    int x, y;
    Cell() : x(0), y(0) {}
    Cell(int _x, int _y) : x(_x), y(_y) {}

    bool operator==(const Cell& other) const {
        return x == other.x && y == other.y;
    }
};

namespace std {
    template<>
    struct hash<Cell> {
        size_t operator()(const Cell& c) const {
            return (std::hash<int>()(c.x) << 1) ^ std::hash<int>()(c.y);
        }
    };
}

static bool is_inside(int x, int y, int w, int h) {
    return x > 0 && y > 0 && x < w - 1 && y < h - 1 && x % 2 == 1 && y % 2 == 1;
}

void maze_make_wilson(maze_t* maze) {
    if (!maze || maze->width < 3 || maze->height < 3) return;
    if (maze->width % 2 == 0 || maze->height % 2 == 0) return;

    int w = maze->width;
    int h = maze->height;
    int x0 = maze->x0;
    int y0 = maze->y0;

    std::vector<std::vector<int>> grid(h, std::vector<int>(w, WALL));
    std::unordered_set<Cell> visited;

    std::mt19937 rng(static_cast<unsigned int>(time(nullptr)));
    std::vector<Cell> all_cells;
    for (int y = 1; y < h; y += 2) {
        for (int x = 1; x < w; x += 2) {
            all_cells.emplace_back(x, y);
        }
    }

    std::uniform_int_distribution<size_t> pick_cell(0, all_cells.size() - 1);

    Cell start = all_cells[pick_cell(rng)];
    grid[start.y][start.x] = PASSAGE;
    visited.insert(start);

    const int dx[4] = { 0, 0, -2, 2 };
    const int dy[4] = { -2, 2, 0, 0 };

    while (visited.size() < all_cells.size()) {
        Cell current;
        do {
            current = all_cells[pick_cell(rng)];
        } while (visited.count(current) > 0);

        std::unordered_map<Cell, Cell> path;
        Cell walk = current;

        while (visited.count(walk) == 0) {
            std::vector<int> dirs = { 0, 1, 2, 3 };
            std::shuffle(dirs.begin(), dirs.end(), rng);

            for (int dir : dirs) {
                int nx = walk.x + dx[dir];
                int ny = walk.y + dy[dir];
                if (is_inside(nx, ny, w, h)) {
                    Cell next(nx, ny);
                    path[walk] = next;
                    walk = next;
                    break;
                }
            }
        }

        walk = current;
        while (visited.count(walk) == 0) {
            visited.insert(walk);
            grid[walk.y][walk.x] = PASSAGE;

            Cell next = path[walk];
            int mx = (walk.x + next.x) / 2;
            int my = (walk.y + next.y) / 2;
            grid[my][mx] = PASSAGE;

            walk = next;
        }
    }

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (grid[y][x] != PASSAGE) {
                coord_hash_insert(
                    maze->blocked,
                    make_tmp_coord(x + x0, y + y0),
                    nullptr
                );
            }
        }
    }
}
