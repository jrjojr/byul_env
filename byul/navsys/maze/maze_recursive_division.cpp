#include "maze_recursive_division.h"
#include "internal/maze_private.hpp"

#include <cstdint>
#include <limits>
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

struct region_t {
    int left;
    int top;
    int right;
    int bottom;
};

navsys_status_t divide_iteratively(
    std::vector<uint8_t>& grid,
    int grid_width,
    int grid_height,
    byul_maze_generation_context& context) {
    const size_t region_stack_bound =
        static_cast<size_t>(grid_width / 2)
        + static_cast<size_t>(grid_height / 2);
    std::vector<region_t> stack;
    stack.reserve(region_stack_bound);
    stack.push_back(region_t{0, 0, grid_width - 1, grid_height - 1});
    while (!stack.empty()) {
        const region_t region = stack.back();
        stack.pop_back();
        const int span_x = region.right - region.left;
        const int span_y = region.bottom - region.top;
        if (span_x < 4 || span_y < 4) {
            const navsys_status_t poll_status = context.poll();
            if (poll_status != NAVSYS_STATUS_OK) return poll_status;
            continue;
        }

        const navsys_status_t step_status = context.begin_step();
        if (step_status != NAVSYS_STATUS_OK) return step_status;
        const bool horizontal = span_x < span_y
            || (span_x == span_y && context.bounded(2) == 0);
        if (stack.size() + 2 > region_stack_bound) {
            return NAVSYS_STATUS_CORRUPT_STATE;
        }
        if (horizontal) {
            const int wall_y = random_even(
                region.top + 2, region.bottom - 2, context);
            const int passage_x = random_odd(
                region.left + 1, region.right - 1, context);
            for (int x = region.left; x <= region.right; ++x) {
                grid[static_cast<size_t>(wall_y) * grid_width + x] = wall_cell;
            }
            grid[static_cast<size_t>(wall_y) * grid_width + passage_x]
                = passage_cell;
            stack.push_back(region_t{
                region.left, wall_y, region.right, region.bottom});
            stack.push_back(region_t{
                region.left, region.top, region.right, wall_y});
            continue;
        }

        const int wall_x = random_even(
            region.left + 2, region.right - 2, context);
        const int passage_y = random_odd(
            region.top + 1, region.bottom - 1, context);
        for (int y = region.top; y <= region.bottom; ++y) {
            grid[static_cast<size_t>(y) * grid_width + wall_x] = wall_cell;
        }
        grid[static_cast<size_t>(passage_y) * grid_width + wall_x]
            = passage_cell;
        stack.push_back(region_t{
            wall_x, region.top, region.right, region.bottom});
        stack.push_back(region_t{
            region.left, region.top, wall_x, region.bottom});
    }
    return NAVSYS_STATUS_OK;
}

bool has_representable_bound(int32_t origin, uint32_t length) {
    if (length == 0) return true;
    if (length > static_cast<uint32_t>(std::numeric_limits<int32_t>::max())) {
        return false;
    }
    const int64_t last = static_cast<int64_t>(origin)
        + static_cast<int64_t>(length) - 1;
    return last <= std::numeric_limits<int32_t>::max();
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

        navsys_status_t status = divide_iteratively(grid, w, h, context);
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

navsys_status_t byul_maze_generate_recursive_division(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    const byul_maze_generate_options_t* options,
    maze_t** out_maze) {
    if (!out_maze) return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_maze = nullptr;
    if (!options
        || options->struct_size < sizeof(byul_maze_generate_options_t)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    if (options->abi_version != BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION
        || width < 3 || height < 3
        || (width & UINT32_C(1)) == 0
        || (height & UINT32_C(1)) == 0) {
        return NAVSYS_STATUS_UNSUPPORTED;
    }
    if (!has_representable_bound(origin_x, width)
        || !has_representable_bound(origin_y, height)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const uint64_t cells = static_cast<uint64_t>(width) * height;
    if (options->max_cells != 0 && cells > options->max_cells) {
        return NAVSYS_STATUS_LIMIT_REACHED;
    }
    const uint64_t default_steps =
        cells > std::numeric_limits<uint64_t>::max() / 4
        ? std::numeric_limits<uint64_t>::max()
        : cells * 4;
    byul_maze_generation_context context(
        options->seed,
        options->max_steps != 0 ? options->max_steps : default_steps,
        options->cancel_func,
        options->cancel_userdata);
    return byul_maze_generate_recursive_division_internal(
        origin_x, origin_y, width, height, context, out_maze);
}

maze_t* maze_make_recursive_division(
    int x0, int y0, int width, int height) {
    if (width < 3 || height < 3 || width % 2 == 0 || height % 2 == 0) {
        return nullptr;
    }
    const byul_maze_generate_options_t options{
        sizeof(byul_maze_generate_options_t),
        BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION,
        byul_maze_generation_legacy_seed(),
        UINT64_C(0),
        UINT64_C(0),
        nullptr,
        nullptr
    };
    maze_t* maze = nullptr;
    return byul_maze_generate_recursive_division(
               x0,
               y0,
               static_cast<uint32_t>(width),
               static_cast<uint32_t>(height),
               &options,
               &maze)
            == NAVSYS_STATUS_OK
        ? maze
        : nullptr;
}
