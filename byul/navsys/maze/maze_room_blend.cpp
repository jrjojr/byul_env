#include "maze_room_blend.h"
#include "internal/maze_private.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
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

struct room_blend_policy_t {
    uint32_t room_attempts;
    uint32_t min_room_width;
    uint32_t min_room_height;
    uint32_t max_room_width;
    uint32_t max_room_height;
    uint32_t room_padding;
};

constexpr room_blend_policy_t legacy_policy{
    30, 3, 3, 7, 7, 0
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

bool overlaps(
    const room_t_internal& first,
    const room_t_internal& second,
    uint32_t padding) {
    const int64_t gap = padding;
    return static_cast<int64_t>(first.x) <
            static_cast<int64_t>(second.x) + second.width + gap
        && static_cast<int64_t>(first.x) + first.width + gap > second.x
        && static_cast<int64_t>(first.y) <
            static_cast<int64_t>(second.y) + second.height + gap
        && static_cast<int64_t>(first.y) + first.height + gap > second.y;
}

bool has_representable_bound(int32_t origin, uint32_t length) {
    if (length > static_cast<uint32_t>(std::numeric_limits<int32_t>::max())) {
        return false;
    }
    const int64_t last = static_cast<int64_t>(origin)
        + static_cast<int64_t>(length) - 1;
    return last <= std::numeric_limits<int32_t>::max();
}

bool has_valid_room_policy(
    uint32_t width,
    uint32_t height,
    const room_blend_policy_t& policy) {
    if (policy.min_room_width < 3 || policy.min_room_height < 3
        || (policy.min_room_width & UINT32_C(1)) == 0
        || (policy.min_room_height & UINT32_C(1)) == 0
        || (policy.max_room_width & UINT32_C(1)) == 0
        || (policy.max_room_height & UINT32_C(1)) == 0
        || policy.min_room_width > policy.max_room_width
        || policy.min_room_height > policy.max_room_height) {
        return false;
    }
    const uint64_t cells = static_cast<uint64_t>(width) * height;
    const uint64_t maximum_attempts =
        cells > std::numeric_limits<uint64_t>::max() / 32
        ? std::numeric_limits<uint64_t>::max()
        : cells * 32;
    return policy.min_room_width <= width - 2
        && policy.min_room_height <= height - 2
        && policy.room_attempts <= maximum_attempts;
}

uint32_t eligible_odd_max(uint32_t requested, uint32_t interior) {
    uint32_t result = std::min(requested, interior);
    if ((result & UINT32_C(1)) == 0) --result;
    return result;
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

namespace {

navsys_status_t generate_room_blend_configured(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    const room_blend_policy_t& policy,
    byul_maze_generation_context& context,
    maze_t** out_maze) noexcept {
    if (!out_maze) return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_maze = nullptr;
    if (width < 9 || height < 9) return NAVSYS_STATUS_UNSUPPORTED;
    if (!has_representable_bound(origin_x, width)
        || !has_representable_bound(origin_y, height)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    if (!has_valid_room_policy(width, height, policy)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const uint64_t cells = static_cast<uint64_t>(width) * height;
    if (cells > std::numeric_limits<size_t>::max()) {
        return NAVSYS_STATUS_LIMIT_REACHED;
    }
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
        const uint32_t maximum_room_width = eligible_odd_max(
            policy.max_room_width, width - 2);
        const uint32_t maximum_room_height = eligible_odd_max(
            policy.max_room_height, height - 2);

        for (uint32_t attempt = 0; attempt < policy.room_attempts; ++attempt) {
            const navsys_status_t step_status = context.begin_step();
            if (step_status != NAVSYS_STATUS_OK) {
                maze_destroy(maze);
                return step_status;
            }
            const int room_width = static_cast<int>(policy.min_room_width
                + 2 * context.bounded(
                    (maximum_room_width - policy.min_room_width) / 2 + 1));
            const int room_height = static_cast<int>(policy.min_room_height
                + 2 * context.bounded(
                    (maximum_room_height - policy.min_room_height) / 2 + 1));
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
                if (overlaps(room, existing, policy.room_padding)) {
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

uint64_t saturated_add(uint64_t left, uint64_t right) {
    return left > std::numeric_limits<uint64_t>::max() - right
        ? std::numeric_limits<uint64_t>::max()
        : left + right;
}

uint64_t saturated_multiply(uint64_t left, uint64_t right) {
    return right != 0 && left > std::numeric_limits<uint64_t>::max() / right
        ? std::numeric_limits<uint64_t>::max()
        : left * right;
}

uint64_t default_step_limit(
    uint64_t cells,
    uint32_t width,
    uint32_t height,
    uint32_t room_attempts) {
    const uint64_t maze_budget = saturated_multiply(cells, 4);
    const uint64_t per_room_budget =
        static_cast<uint64_t>(width) + height + 1;
    const uint64_t room_budget =
        saturated_multiply(room_attempts, per_room_budget);
    return saturated_add(maze_budget, room_budget);
}

} // namespace

navsys_status_t byul_maze_generate_room_blend_internal(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_generation_context& context,
    maze_t** out_maze) noexcept {
    return generate_room_blend_configured(
        origin_x, origin_y, width, height, legacy_policy, context, out_maze);
}

navsys_status_t byul_maze_generate_room_blend(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    const byul_room_blend_options_t* options,
    maze_t** out_maze) {
    if (!out_maze) return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_maze = nullptr;
    if (!options
        || options->struct_size < sizeof(byul_room_blend_options_t)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    if (options->abi_version != BYUL_ROOM_BLEND_OPTIONS_ABI_VERSION) {
        return NAVSYS_STATUS_UNSUPPORTED;
    }
    if (width < 9 || height < 9) return NAVSYS_STATUS_UNSUPPORTED;
    if (!has_representable_bound(origin_x, width)
        || !has_representable_bound(origin_y, height)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const room_blend_policy_t policy{
        options->room_attempts,
        options->min_room_width,
        options->min_room_height,
        options->max_room_width,
        options->max_room_height,
        options->room_padding
    };
    if (!has_valid_room_policy(width, height, policy)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const uint64_t cells = static_cast<uint64_t>(width) * height;
    if (options->max_cells != 0 && cells > options->max_cells) {
        return NAVSYS_STATUS_LIMIT_REACHED;
    }
    if (cells > std::numeric_limits<size_t>::max()) {
        return NAVSYS_STATUS_LIMIT_REACHED;
    }
    byul_maze_generation_context context(
        options->seed,
        options->max_steps != 0
            ? options->max_steps
            : default_step_limit(
                cells, width, height, options->room_attempts),
        options->cancel_func,
        options->cancel_userdata);
    return generate_room_blend_configured(
        origin_x, origin_y, width, height, policy, context, out_maze);
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
