#include "maze_kruskal.h"
#include "internal/maze_private.hpp"

#include <cstdint>
#include <new>
#include <vector>

static const int WALL = 1;
static const int PASSAGE = 0;

struct Wall {
    int x1, y1, x2, y2, wx, wy;
};

struct Cell {
    int x, y;
};

static int find(int x, int y, std::vector<std::vector<int>>& parent) {
    const int width = static_cast<int>(parent[0].size());
    if (parent[y][x] == y * width + x) return parent[y][x];
    int id = parent[y][x];
    int px = id % width;
    int py = id / width;
    return parent[y][x] = find(px, py, parent);
}

static void merge(int x1, int y1, int x2, int y2, 
    std::vector<std::vector<int>>& parent) {

    int p1 = find(x1, y1, parent);
    int p2 = find(x2, y2, parent);
    const int width = static_cast<int>(parent[0].size());
    if (p1 != p2) parent[p2 / width][p2 % width] = p1;
}

navsys_status_t byul_maze_generate_kruskal_internal(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_generation_context& context,
    maze_t** out_maze) noexcept {
    if (!out_maze) return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_maze = nullptr;
    const navsys_status_t initial_poll = context.poll();
    if (initial_poll != NAVSYS_STATUS_OK) return initial_poll;

    maze_t* maze = maze_create_full(
        origin_x, origin_y, static_cast<int>(width), static_cast<int>(height));
    if (!maze) return NAVSYS_STATUS_OUT_OF_MEMORY;

    try {
        const int w = static_cast<int>(width);
        const int h = static_cast<int>(height);
        std::vector<std::vector<int>> grid(h, std::vector<int>(w, WALL));
        std::vector<std::vector<int>> parent(h, std::vector<int>(w, 0));
        std::vector<Wall> walls;

        for (int y = 1; y < h; y += 2) {
            for (int x = 1; x < w; x += 2) {
                grid[y][x] = PASSAGE;
                parent[y][x] = y * w + x;

                if (x + 2 < w) {
                    walls.push_back({x, y, x + 2, y, x + 1, y});
                }
                if (y + 2 < h) {
                    walls.push_back({x, y, x, y + 2, x, y + 1});
                }
            }
        }

        for (size_t i = walls.size(); i > 1; --i) {
            const navsys_status_t poll_status = context.poll();
            if (poll_status != NAVSYS_STATUS_OK) {
                maze_destroy(maze);
                return poll_status;
            }
            const size_t selected = context.bounded(static_cast<uint32_t>(i));
            const Wall temporary = walls[i - 1];
            walls[i - 1] = walls[selected];
            walls[selected] = temporary;
        }

        for (const Wall& wall : walls) {
            const navsys_status_t step_status = context.begin_step();
            if (step_status != NAVSYS_STATUS_OK) {
                maze_destroy(maze);
                return step_status;
            }
            const int p1 = find(wall.x1, wall.y1, parent);
            const int p2 = find(wall.x2, wall.y2, parent);
            if (p1 != p2) {
                merge(wall.x1, wall.y1, wall.x2, wall.y2, parent);
                grid[wall.wy][wall.wx] = PASSAGE;
            }
        }

        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                if (grid[y][x] == WALL) {
                    const navsys_status_t poll_status = context.poll();
                    if (poll_status != NAVSYS_STATUS_OK) {
                        maze_destroy(maze);
                        return poll_status;
                    }
                    bool changed = false;
                    const navsys_status_t block_status = byul_maze_set_blocked(
                        maze, x + origin_x, y + origin_y, true, &changed);
                    if (block_status != NAVSYS_STATUS_OK) {
                        maze_destroy(maze);
                        return block_status;
                    }
                }
            }
        }
    } catch (const std::bad_alloc&) {
        maze_destroy(maze);
        return NAVSYS_STATUS_OUT_OF_MEMORY;
    } catch (...) {
        maze_destroy(maze);
        return NAVSYS_STATUS_CORRUPT_STATE;
    }

    *out_maze = maze;
    return NAVSYS_STATUS_OK;
}

maze_t* maze_make_kruskal(int x0, int y0, int width, int height) {
    if (width < 3 || height < 3) return nullptr;
    if (width % 2 == 0 || height % 2 == 0) return nullptr;

    byul_maze_generation_context context(
        byul_maze_generation_legacy_seed(), 0, nullptr, nullptr);
    maze_t* maze = nullptr;
    return byul_maze_generate_kruskal_internal(
               x0,
               y0,
               static_cast<uint32_t>(width),
               static_cast<uint32_t>(height),
               context,
               &maze)
            == NAVSYS_STATUS_OK
        ? maze
        : nullptr;
}
