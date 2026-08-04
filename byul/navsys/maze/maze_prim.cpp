#include "maze_prim.h"
#include "internal/maze_private.hpp"

#include <cstdint>
#include <new>
#include <vector>

namespace {

constexpr int wall_cell = 1;
constexpr int passage_cell = 0;

struct frontier_t {
    int wall_x;
    int wall_y;
    int target_x;
    int target_y;
};

bool is_logical_cell(int x, int y, int width, int height) {
    return x > 0 && y > 0 && x < width - 1 && y < height - 1;
}

void add_frontiers(
    int cell_x,
    int cell_y,
    int width,
    int height,
    const std::vector<int>& grid,
    std::vector<frontier_t>& frontiers) {
    static constexpr int delta_x[4] = {0, 0, -1, 1};
    static constexpr int delta_y[4] = {-1, 1, 0, 0};
    for (int direction = 0; direction < 4; ++direction) {
        const int target_x = cell_x + delta_x[direction] * 2;
        const int target_y = cell_y + delta_y[direction] * 2;
        if (is_logical_cell(target_x, target_y, width, height)
            && grid[static_cast<size_t>(target_y) * width + target_x]
                == wall_cell) {
            frontiers.push_back({
                cell_x + delta_x[direction],
                cell_y + delta_y[direction],
                target_x,
                target_y
            });
        }
    }
}

} // namespace

navsys_status_t byul_maze_generate_prim_internal(
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
        std::vector<int> grid(
            static_cast<size_t>(width) * height, wall_cell);
        std::vector<frontier_t> frontiers;

        const uint32_t logical_width = (width - 1u) / 2u;
        const uint32_t logical_height = (height - 1u) / 2u;
        const int start_x = 1 + static_cast<int>(context.bounded(logical_width)) * 2;
        const int start_y = 1 + static_cast<int>(context.bounded(logical_height)) * 2;
        grid[static_cast<size_t>(start_y) * w + start_x] = passage_cell;
        add_frontiers(start_x, start_y, w, h, grid, frontiers);

        while (!frontiers.empty()) {
            const navsys_status_t step_status = context.begin_step();
            if (step_status != NAVSYS_STATUS_OK) {
                maze_destroy(maze);
                return step_status;
            }
            const size_t selected = context.bounded(
                static_cast<uint32_t>(frontiers.size()));
            const frontier_t frontier = frontiers[selected];
            frontiers[selected] = frontiers.back();
            frontiers.pop_back();

            const size_t target_index =
                static_cast<size_t>(frontier.target_y) * w + frontier.target_x;
            if (grid[target_index] == passage_cell) continue;
            grid[static_cast<size_t>(frontier.wall_y) * w + frontier.wall_x]
                = passage_cell;
            grid[target_index] = passage_cell;
            add_frontiers(
                frontier.target_x,
                frontier.target_y,
                w,
                h,
                grid,
                frontiers);
        }

        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                if (grid[static_cast<size_t>(y) * w + x] != wall_cell) continue;
                const navsys_status_t poll_status = context.poll();
                if (poll_status != NAVSYS_STATUS_OK) {
                    maze_destroy(maze);
                    return poll_status;
                }
                bool changed = false;
                const navsys_status_t status = byul_maze_set_blocked(
                    maze, origin_x + x, origin_y + y, true, &changed);
                if (status != NAVSYS_STATUS_OK) {
                    maze_destroy(maze);
                    return status;
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

maze_t* maze_maze_prim(int x0, int y0, int width, int height) {
    if (width < 3 || height < 3) return nullptr;
    byul_maze_generation_context context(
        byul_maze_generation_legacy_seed(), 0, nullptr, nullptr);
    maze_t* maze = nullptr;
    return byul_maze_generate_prim_internal(
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
