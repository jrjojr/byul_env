#include "maze_recursive_division.h"
#include "internal/maze_private.hpp"

#include <cstdint>
#include <new>
#include <vector>

namespace {

constexpr uint8_t wall_cell = 1;
constexpr uint8_t passage_cell = 0;

int random_even(
    int minimum, int maximum, byul_maze_generation_context& context) {
    const uint32_t count = static_cast<uint32_t>((maximum - minimum) / 2 + 1);
    return minimum + static_cast<int>(context.bounded(count)) * 2;
}

int random_odd(
    int minimum, int maximum, byul_maze_generation_context& context) {
    const uint32_t count = static_cast<uint32_t>((maximum - minimum) / 2 + 1);
    return minimum + static_cast<int>(context.bounded(count)) * 2;
}

navsys_status_t divide(
    std::vector<uint8_t>& grid,
    int grid_width,
    int left,
    int top,
    int right,
    int bottom,
    byul_maze_generation_context& context) {
    const int span_x = right - left;
    const int span_y = bottom - top;
    if (span_x < 4 || span_y < 4) return context.poll();

    const navsys_status_t step_status = context.begin_step();
    if (step_status != NAVSYS_STATUS_OK) return step_status;
    const bool horizontal = span_x < span_y
        || (span_x == span_y && context.bounded(2) == 0);
    if (horizontal) {
        const int wall_y = random_even(top + 2, bottom - 2, context);
        const int passage_x = random_odd(left + 1, right - 1, context);
        for (int x = left; x <= right; ++x) {
            grid[static_cast<size_t>(wall_y) * grid_width + x] = wall_cell;
        }
        grid[static_cast<size_t>(wall_y) * grid_width + passage_x]
            = passage_cell;
        navsys_status_t status = divide(
            grid, grid_width, left, top, right, wall_y, context);
        if (status != NAVSYS_STATUS_OK) return status;
        return divide(
            grid, grid_width, left, wall_y, right, bottom, context);
    }

    const int wall_x = random_even(left + 2, right - 2, context);
    const int passage_y = random_odd(top + 1, bottom - 1, context);
    for (int y = top; y <= bottom; ++y) {
        grid[static_cast<size_t>(y) * grid_width + wall_x] = wall_cell;
    }
    grid[static_cast<size_t>(passage_y) * grid_width + wall_x] = passage_cell;
    navsys_status_t status = divide(
        grid, grid_width, left, top, wall_x, bottom, context);
    if (status != NAVSYS_STATUS_OK) return status;
    return divide(
        grid, grid_width, wall_x, top, right, bottom, context);
}

} // namespace

navsys_status_t byul_maze_generate_recursive_division_internal(
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
        std::vector<uint8_t> grid(
            static_cast<size_t>(width) * height, passage_cell);
        for (int x = 0; x < w; ++x) {
            grid[x] = wall_cell;
            grid[static_cast<size_t>(h - 1) * w + x] = wall_cell;
        }
        for (int y = 0; y < h; ++y) {
            grid[static_cast<size_t>(y) * w] = wall_cell;
            grid[static_cast<size_t>(y) * w + w - 1] = wall_cell;
        }

        navsys_status_t status = divide(
            grid, w, 0, 0, w - 1, h - 1, context);
        if (status != NAVSYS_STATUS_OK) {
            maze_destroy(maze);
            return status;
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
                status = byul_maze_set_blocked(
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

maze_t* maze_make_recursive_division(
    int x0, int y0, int width, int height) {
    if (width < 3 || height < 3 || width % 2 == 0 || height % 2 == 0) {
        return nullptr;
    }
    byul_maze_generation_context context(
        byul_maze_generation_legacy_seed(), 0, nullptr, nullptr);
    maze_t* maze = nullptr;
    return byul_maze_generate_recursive_division_internal(
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
