#include "maze_room_blend.h"
#include "internal/maze_private.hpp"

#include <algorithm>
#include <cstdint>
#include <new>
#include <utility>
#include <vector>

namespace {

constexpr uint8_t wall_cell = 1;
constexpr uint8_t passage_cell = 0;

struct room_t_internal {
    int x;
    int y;
    int width;
    int height;

    int center_x() const { return x + width / 2; }
    int center_y() const { return y + height / 2; }
};

void open_grid_cell(
    std::vector<uint8_t>& grid, int width, int x, int y) {
    grid[static_cast<size_t>(y) * width + x] = passage_cell;
}

void dig_room(
    std::vector<uint8_t>& grid, int width, const room_t_internal& room) {
    for (int y = room.y; y < room.y + room.height; ++y) {
        for (int x = room.x; x < room.x + room.width; ++x) {
            open_grid_cell(grid, width, x, y);
        }
    }
}

bool overlaps(const room_t_internal& first, const room_t_internal& second) {
    return first.x < second.x + second.width
        && first.x + first.width > second.x
        && first.y < second.y + second.height
        && first.y + first.height > second.y;
}

navsys_status_t dig_segment(
    std::vector<uint8_t>& grid,
    int width,
    int x1,
    int y1,
    int x2,
    int y2,
    byul_maze_generation_context& context) {
    const int step_x = x1 <= x2 ? 1 : -1;
    for (int x = x1;; x += step_x) {
        const navsys_status_t status = context.begin_step();
        if (status != NAVSYS_STATUS_OK) return status;
        open_grid_cell(grid, width, x, y1);
        if (x == x2) break;
    }
    const int step_y = y1 <= y2 ? 1 : -1;
    for (int y = y1;; y += step_y) {
        const navsys_status_t status = context.begin_step();
        if (status != NAVSYS_STATUS_OK) return status;
        open_grid_cell(grid, width, x2, y);
        if (y == y2) break;
    }
    return NAVSYS_STATUS_OK;
}

navsys_status_t fill_with_maze(
    std::vector<uint8_t>& grid,
    int width,
    int height,
    byul_maze_generation_context& context) {
    std::vector<uint8_t> visited(
        static_cast<size_t>(width) * height, uint8_t{0});
    std::vector<std::pair<int, int>> stack;
    stack.emplace_back(1, 1);
    visited[static_cast<size_t>(width) + 1] = 1;
    open_grid_cell(grid, width, 1, 1);

    static constexpr int delta_x[4] = {0, 0, -2, 2};
    static constexpr int delta_y[4] = {-2, 2, 0, 0};
    while (!stack.empty()) {
        const navsys_status_t step_status = context.begin_step();
        if (step_status != NAVSYS_STATUS_OK) return step_status;
        const auto [x, y] = stack.back();
        int directions[4] = {0, 1, 2, 3};
        for (uint32_t count = 4; count > 1; --count) {
            const uint32_t selected = context.bounded(count);
            const int temporary = directions[count - 1];
            directions[count - 1] = directions[selected];
            directions[selected] = temporary;
        }

        bool moved = false;
        for (const int direction : directions) {
            const int next_x = x + delta_x[direction];
            const int next_y = y + delta_y[direction];
            if (next_x <= 0 || next_y <= 0
                || next_x >= width - 1 || next_y >= height - 1
                || visited[static_cast<size_t>(next_y) * width + next_x] != 0) {
                continue;
            }
            visited[static_cast<size_t>(next_y) * width + next_x] = 1;
            open_grid_cell(
                grid, width, (x + next_x) / 2, (y + next_y) / 2);
            open_grid_cell(grid, width, next_x, next_y);
            stack.emplace_back(next_x, next_y);
            moved = true;
            break;
        }
        if (!moved) stack.pop_back();
    }
    return NAVSYS_STATUS_OK;
}

} // namespace

navsys_status_t byul_maze_generate_room_blend_internal(
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
            static_cast<size_t>(width) * height, wall_cell);
        std::vector<room_t_internal> rooms;
        constexpr int room_attempts = 30;
        constexpr int room_minimum = 3;
        constexpr int room_maximum = 7;

        for (int attempt = 0; attempt < room_attempts; ++attempt) {
            const navsys_status_t step_status = context.begin_step();
            if (step_status != NAVSYS_STATUS_OK) {
                maze_destroy(maze);
                return step_status;
            }
            const int max_width = std::min(room_maximum, w - 2);
            const int max_height = std::min(room_maximum, h - 2);
            const int room_width = room_minimum + 2 * static_cast<int>(
                context.bounded(static_cast<uint32_t>(
                    (max_width - room_minimum) / 2 + 1)));
            const int room_height = room_minimum + 2 * static_cast<int>(
                context.bounded(static_cast<uint32_t>(
                    (max_height - room_minimum) / 2 + 1)));
            const int max_x = w - room_width - 1;
            const int max_y = h - room_height - 1;
            const int room_x = 1 + 2 * static_cast<int>(context.bounded(
                static_cast<uint32_t>((max_x - 1) / 2 + 1)));
            const int room_y = 1 + 2 * static_cast<int>(context.bounded(
                static_cast<uint32_t>((max_y - 1) / 2 + 1)));
            const room_t_internal room{
                room_x, room_y, room_width, room_height};

            bool overlap = false;
            for (const room_t_internal& existing : rooms) {
                if (overlaps(room, existing)) {
                    overlap = true;
                    break;
                }
            }
            if (!overlap) {
                dig_room(grid, w, room);
                rooms.push_back(room);
            }
        }

        for (size_t index = 1; index < rooms.size(); ++index) {
            const room_t_internal& previous = rooms[index - 1];
            const room_t_internal& current = rooms[index];
            navsys_status_t status;
            if (context.bounded(2) != 0) {
                status = dig_segment(
                    grid,
                    w,
                    previous.center_x(),
                    previous.center_y(),
                    current.center_x(),
                    previous.center_y(),
                    context);
                if (status == NAVSYS_STATUS_OK) {
                    status = dig_segment(
                        grid,
                        w,
                        current.center_x(),
                        previous.center_y(),
                        current.center_x(),
                        current.center_y(),
                        context);
                }
            } else {
                status = dig_segment(
                    grid,
                    w,
                    previous.center_x(),
                    previous.center_y(),
                    previous.center_x(),
                    current.center_y(),
                    context);
                if (status == NAVSYS_STATUS_OK) {
                    status = dig_segment(
                        grid,
                        w,
                        previous.center_x(),
                        current.center_y(),
                        current.center_x(),
                        current.center_y(),
                        context);
                }
            }
            if (status != NAVSYS_STATUS_OK) {
                maze_destroy(maze);
                return status;
            }
        }

        navsys_status_t status = fill_with_maze(grid, w, h, context);
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

maze_t* maze_make_room_blend(int x0, int y0, int width, int height) {
    if (width < 9 || height < 9) return nullptr;
    byul_maze_generation_context context(
        byul_maze_generation_legacy_seed(), 0, nullptr, nullptr);
    maze_t* maze = nullptr;
    return byul_maze_generate_room_blend_internal(
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
