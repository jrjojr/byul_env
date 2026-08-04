#include "maze_hunt_and_kill.h"
#include "internal/maze_private.hpp"

#include <cstdint>
#include <new>
#include <vector>

namespace {

bool is_logical_cell(int x, int y, int width, int height) {
    return x > 0 && y > 0 && x < width - 1 && y < height - 1;
}

navsys_status_t open_cell(
    maze_t* maze,
    int32_t origin_x,
    int32_t origin_y,
    int x,
    int y) {
    bool changed = false;
    return byul_maze_set_blocked(
        maze, origin_x + x, origin_y + y, false, &changed);
}

} // namespace

navsys_status_t byul_maze_generate_hunt_and_kill_internal(
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
                const navsys_status_t block_status = byul_maze_set_blocked(
                    maze, origin_x + x, origin_y + y, true, &changed);
                if (block_status != NAVSYS_STATUS_OK) {
                    maze_destroy(maze);
                    return block_status;
                }
            }
        }
        std::vector<uint8_t> visited(
            static_cast<size_t>(width) * height, uint8_t{0});
        const uint32_t logical_width = (width - 1u) / 2u;
        const uint32_t logical_height = (height - 1u) / 2u;
        int current_x = 1 + static_cast<int>(context.bounded(logical_width)) * 2;
        int current_y = 1 + static_cast<int>(context.bounded(logical_height)) * 2;
        visited[static_cast<size_t>(current_y) * w + current_x] = 1;
        navsys_status_t status = open_cell(
            maze, origin_x, origin_y, current_x, current_y);
        if (status != NAVSYS_STATUS_OK) {
            maze_destroy(maze);
            return status;
        }

        static constexpr int delta_x[4] = {0, 0, -2, 2};
        static constexpr int delta_y[4] = {-2, 2, 0, 0};
        while (true) {
            const navsys_status_t step_status = context.begin_step();
            if (step_status != NAVSYS_STATUS_OK) {
                maze_destroy(maze);
                return step_status;
            }

            int directions[4] = {0, 1, 2, 3};
            for (uint32_t count = 4; count > 1; --count) {
                const uint32_t selected = context.bounded(count);
                const int temporary = directions[count - 1];
                directions[count - 1] = directions[selected];
                directions[selected] = temporary;
            }

            bool moved = false;
            for (const int direction : directions) {
                const int next_x = current_x + delta_x[direction];
                const int next_y = current_y + delta_y[direction];
                if (!is_logical_cell(next_x, next_y, w, h)
                    || visited[static_cast<size_t>(next_y) * w + next_x] != 0) {
                    continue;
                }
                status = open_cell(
                    maze,
                    origin_x,
                    origin_y,
                    (current_x + next_x) / 2,
                    (current_y + next_y) / 2);
                if (status == NAVSYS_STATUS_OK) {
                    status = open_cell(
                        maze, origin_x, origin_y, next_x, next_y);
                }
                if (status != NAVSYS_STATUS_OK) {
                    maze_destroy(maze);
                    return status;
                }
                visited[static_cast<size_t>(next_y) * w + next_x] = 1;
                current_x = next_x;
                current_y = next_y;
                moved = true;
                break;
            }
            if (moved) continue;

            bool found = false;
            for (int y = 1; y < h && !found; y += 2) {
                for (int x = 1; x < w; x += 2) {
                    const navsys_status_t poll_status = context.poll();
                    if (poll_status != NAVSYS_STATUS_OK) {
                        maze_destroy(maze);
                        return poll_status;
                    }
                    if (visited[static_cast<size_t>(y) * w + x] != 0) continue;

                    int adjacent[4];
                    uint32_t adjacent_count = 0;
                    for (int direction = 0; direction < 4; ++direction) {
                        const int next_x = x + delta_x[direction];
                        const int next_y = y + delta_y[direction];
                        if (is_logical_cell(next_x, next_y, w, h)
                            && visited[static_cast<size_t>(next_y) * w + next_x]
                                != 0) {
                            adjacent[adjacent_count++] = direction;
                        }
                    }
                    if (adjacent_count == 0) continue;

                    const int direction = adjacent[context.bounded(adjacent_count)];
                    status = open_cell(
                        maze,
                        origin_x,
                        origin_y,
                        x + delta_x[direction] / 2,
                        y + delta_y[direction] / 2);
                    if (status == NAVSYS_STATUS_OK) {
                        status = open_cell(maze, origin_x, origin_y, x, y);
                    }
                    if (status != NAVSYS_STATUS_OK) {
                        maze_destroy(maze);
                        return status;
                    }
                    visited[static_cast<size_t>(y) * w + x] = 1;
                    current_x = x;
                    current_y = y;
                    found = true;
                    break;
                }
            }
            if (!found) break;
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

maze_t* maze_make_hunt_and_kill(int x0, int y0, int width, int height) {
    if (width < 3 || height < 3 || width % 2 == 0 || height % 2 == 0) {
        return nullptr;
    }
    byul_maze_generation_context context(
        byul_maze_generation_legacy_seed(), 0, nullptr, nullptr);
    maze_t* maze = nullptr;
    return byul_maze_generate_hunt_and_kill_internal(
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
