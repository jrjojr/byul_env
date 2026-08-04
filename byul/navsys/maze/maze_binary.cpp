#include "maze_binary.h"
#include "internal/maze_private.hpp"

#include <cstdint>
#include <new>

namespace {

navsys_status_t set_blocked(
    maze_t* maze, int32_t origin_x, int32_t origin_y,
    int x, int y, bool blocked) {
    bool changed = false;
    return byul_maze_set_blocked(
        maze, origin_x + x, origin_y + y, blocked, &changed);
}

} // namespace

navsys_status_t byul_maze_generate_binary_internal(
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
                const navsys_status_t status =
                    set_blocked(maze, origin_x, origin_y, x, y, true);
                if (status != NAVSYS_STATUS_OK) {
                    maze_destroy(maze);
                    return status;
                }
            }
        }

        for (int y = 1; y < h; y += 2) {
            for (int x = 1; x < w; x += 2) {
                const navsys_status_t step_status = context.begin_step();
                if (step_status != NAVSYS_STATUS_OK) {
                    maze_destroy(maze);
                    return step_status;
                }
                navsys_status_t status =
                    set_blocked(maze, origin_x, origin_y, x, y, false);
                if (status != NAVSYS_STATUS_OK) {
                    maze_destroy(maze);
                    return status;
                }

                const bool can_east = x + 2 < w;
                const bool can_south = y + 2 < h;
                if (can_east && can_south) {
                    status = context.bounded(2) == 0
                        ? set_blocked(maze, origin_x, origin_y, x + 1, y, false)
                        : set_blocked(maze, origin_x, origin_y, x, y + 1, false);
                } else if (can_east) {
                    status = set_blocked(
                        maze, origin_x, origin_y, x + 1, y, false);
                } else if (can_south) {
                    status = set_blocked(
                        maze, origin_x, origin_y, x, y + 1, false);
                }
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

maze_t* maze_make_binary(int x0, int y0, int width, int height) {
    if (width < 3 || height < 3 || width % 2 == 0 || height % 2 == 0) {
        return nullptr;
    }
    byul_maze_generation_context context(
        byul_maze_generation_legacy_seed(), 0, nullptr, nullptr);
    maze_t* maze = nullptr;
    return byul_maze_generate_binary_internal(
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
