#include "maze_recursive.h"
#include "internal/maze_private.hpp"

#include <cstdint>
#include <new>
#include <vector>

namespace {

bool is_valid_cell(int x, int y, int width, int height) {
    return x > 0 && y > 0 && x < width - 1 && y < height - 1;
}

navsys_status_t carve_passage(
    maze_t* maze,
    int32_t origin_x,
    int32_t origin_y,
    int width,
    int height,
    int cell_x,
    int cell_y,
    std::vector<uint8_t>& visited,
    byul_maze_generation_context& context) {
    const navsys_status_t step_status = context.begin_step();
    if (step_status != NAVSYS_STATUS_OK) return step_status;

    static constexpr int delta_x[4] = {0, 0, -1, 1};
    static constexpr int delta_y[4] = {-1, 1, 0, 0};
    int directions[4] = {0, 1, 2, 3};
    for (uint32_t i = 4; i > 1; --i) {
        const uint32_t selected = context.bounded(i);
        const int temporary = directions[i - 1];
        directions[i - 1] = directions[selected];
        directions[selected] = temporary;
    }

    visited[static_cast<size_t>(cell_y) * width + cell_x] = 1;
    for (const int direction : directions) {
        const int next_x = cell_x + delta_x[direction] * 2;
        const int next_y = cell_y + delta_y[direction] * 2;
        if (!is_valid_cell(next_x, next_y, width, height)
            || visited[static_cast<size_t>(next_y) * width + next_x] != 0) {
            continue;
        }

        bool changed = false;
        navsys_status_t status = byul_maze_set_blocked(
            maze,
            origin_x + cell_x + delta_x[direction],
            origin_y + cell_y + delta_y[direction],
            false,
            &changed);
        if (status != NAVSYS_STATUS_OK) return status;
        status = byul_maze_set_blocked(
            maze,
            origin_x + next_x,
            origin_y + next_y,
            false,
            &changed);
        if (status != NAVSYS_STATUS_OK) return status;
        status = carve_passage(
            maze,
            origin_x,
            origin_y,
            width,
            height,
            next_x,
            next_y,
            visited,
            context);
        if (status != NAVSYS_STATUS_OK) return status;
    }
    return NAVSYS_STATUS_OK;
}

} // namespace

navsys_status_t byul_maze_generate_recursive_internal(
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
        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
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

        std::vector<uint8_t> visited(
            static_cast<size_t>(width) * height, uint8_t{0});
        bool changed = false;
        navsys_status_t status = byul_maze_set_blocked(
            maze, origin_x + 1, origin_y + 1, false, &changed);
        if (status == NAVSYS_STATUS_OK) {
            status = carve_passage(
                maze,
                origin_x,
                origin_y,
                static_cast<int>(width),
                static_cast<int>(height),
                1,
                1,
                visited,
                context);
        }
        if (status != NAVSYS_STATUS_OK) {
            maze_destroy(maze);
            return status;
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

maze_t* maze_make_recursive(int x0, int y0, int width, int height) {
    if (width < 3 || height < 3) return nullptr;
    byul_maze_generation_context context(
        byul_maze_generation_legacy_seed(), 0, nullptr, nullptr);
    maze_t* maze = nullptr;
    return byul_maze_generate_recursive_internal(
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
