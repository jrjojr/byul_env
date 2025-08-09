#include "maze_prim.h"

#include <vector>
#include <random>
#include <ctime>

static const int WALL = 1;
static const int PASSAGE = 0;

struct Cell {
    int x, y;
    Cell(int _x, int _y) : x(_x), y(_y) {}
};

static bool is_inside(int x, int y, int w, int h) {
    return x > 0 && y > 0 && x < w - 1 && y < h - 1;
}

maze_t* maze_maze_prim(int x0, int y0, int width, int height){    
    maze_t* maze = maze_create_full(x0, y0, width, height);
    if (!maze) return nullptr;

    int w = maze->width;
    int h = maze->height;

    std::vector<std::vector<int>> grid(h, std::vector<int>(w, WALL));
    std::vector<Cell> wall_list;

    std::mt19937 rng(static_cast<unsigned int>(time(nullptr)));

    std::vector<int> odd_x, odd_y;
    for (int i = 1; i < w - 1; i += 2) odd_x.push_back(i);
    for (int i = 1; i < h - 1; i += 2) odd_y.push_back(i);

    if (odd_x.empty() || odd_y.empty()) return nullptr;

    std::uniform_int_distribution<size_t> pick_x(0, odd_x.size() - 1);
    std::uniform_int_distribution<size_t> pick_y(0, odd_y.size() - 1);
    int sx = odd_x[pick_x(rng)];
    int sy = odd_y[pick_y(rng)];
    grid[sy][sx] = PASSAGE;

    const int dx[4] = { 0, 0, -1, 1 };
    const int dy[4] = { -1, 1, 0, 0 };

    for (int d = 0; d < 4; ++d) {
        int wx = sx + dx[d];
        int wy = sy + dy[d];
        if (is_inside(wx, wy, w, h) && grid[wy][wx] == WALL) {
            wall_list.emplace_back(wx, wy);
        }
    }

    while (!wall_list.empty()) {
        std::uniform_int_distribution<size_t> pick(0, wall_list.size() - 1);
        size_t idx = pick(rng);
        Cell wall = wall_list[idx];
        wall_list.erase(wall_list.begin() + idx);

        for (int d = 0; d < 4; ++d) {
            int fx = wall.x + dx[d];
            int fy = wall.y + dy[d];
            int bx = wall.x - dx[d];
            int by = wall.y - dy[d];

            if (!is_inside(fx, fy, w, h)) continue;
            if (!is_inside(bx, by, w, h)) continue;

            if (grid[fy][fx] == PASSAGE && grid[by][bx] == WALL) {
                grid[wall.y][wall.x] = PASSAGE;
                grid[by][bx] = PASSAGE;

                for (int nd = 0; nd < 4; ++nd) {
                    int nx = by + dy[nd];
                    int ny = bx + dx[nd];
                    if (is_inside(ny, nx, w, h) && grid[nx][ny] == WALL) {
                        wall_list.emplace_back(ny, nx);
                    }
                }

                break;
            }

            if (grid[by][bx] == PASSAGE && grid[fy][fx] == WALL) {
                grid[wall.y][wall.x] = PASSAGE;
                grid[fy][fx] = PASSAGE;

                for (int nd = 0; nd < 4; ++nd) {
                    int nx = fy + dy[nd];
                    int ny = fx + dx[nd];
                    if (is_inside(ny, nx, w, h) && grid[nx][ny] == WALL) {
                        wall_list.emplace_back(ny, nx);
                    }
                }

                break;
            }
        }
    }

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (grid[y][x] == WALL) {
                coord_t tmp = {x + x0, y + y0};
                coord_hash_insert(
                    maze->blocked,
                    &tmp,
                    nullptr
                );
            }
        }
    }

    return maze;
}
