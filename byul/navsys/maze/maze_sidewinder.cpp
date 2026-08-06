#include "maze_sidewinder.h"
#include "internal/maze_private.hpp"

#include <cstdint>
#include <limits>
#include <new>
#include <vector>

namespace {

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

bool is_supported_sweep(byul_maze_sidewinder_sweep_t sweep) {
    const int value = static_cast<int>(sweep);
    return value >= static_cast<int>(BYUL_MAZE_SIDEWINDER_EAST_NORTH)
        && value <= static_cast<int>(BYUL_MAZE_SIDEWINDER_WEST_SOUTH);
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

navsys_status_t generate_with_sweep(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_sidewinder_sweep_t sweep,
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

        const bool east = sweep == BYUL_MAZE_SIDEWINDER_EAST_NORTH
            || sweep == BYUL_MAZE_SIDEWINDER_EAST_SOUTH;
        const bool north = sweep == BYUL_MAZE_SIDEWINDER_EAST_NORTH
            || sweep == BYUL_MAZE_SIDEWINDER_WEST_NORTH;
        const int x_step = east ? 2 : -2;
        const int y_step = north ? 2 : -2;
        const int first_x = east ? 1 : w - 2;
        const int first_y = north ? 1 : h - 2;

        for (int y = first_y; y > 0 && y < h - 1; y += y_step) {
            std::vector<int> run;
            for (int x = first_x; x > 0 && x < w - 1; x += x_step) {
                const navsys_status_t step_status = context.begin_step();
                if (step_status != NAVSYS_STATUS_OK) {
                    maze_destroy(maze);
                    return step_status;
                }
                navsys_status_t status =
                    open_cell(maze, origin_x, origin_y, x, y);
                if (status != NAVSYS_STATUS_OK) {
                    maze_destroy(maze);
                    return status;
                }
                run.push_back(x);

                const int next_x = x + x_step;
                const bool sweep_edge = next_x <= 0 || next_x >= w - 1;
                const bool boundary_row = y == first_y;
                const bool carve_sweep = !sweep_edge
                    && (boundary_row || context.bounded(2) == 0);
                if (carve_sweep) {
                    status = open_cell(
                        maze, origin_x, origin_y, x + x_step / 2, y);
                } else if (!boundary_row) {
                    const int selected = run[context.bounded(
                        static_cast<uint32_t>(run.size()))];
                    status = open_cell(
                        maze, origin_x, origin_y, selected, y - y_step / 2);
                }
                if (status != NAVSYS_STATUS_OK) {
                    maze_destroy(maze);
                    return status;
                }
                if (!carve_sweep) run.clear();
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

} // namespace

navsys_status_t byul_maze_generate_sidewinder_internal(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_generation_context& context,
    maze_t** out_maze) noexcept {
    return generate_with_sweep(
        origin_x,
        origin_y,
        width,
        height,
        BYUL_MAZE_SIDEWINDER_EAST_NORTH,
        context,
        out_maze);
}

navsys_status_t byul_maze_sidewinder_sweep_is_supported(
    byul_maze_sidewinder_sweep_t sweep,
    bool* out_supported) {
    if (!out_supported) return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_supported = false;
    if (!is_supported_sweep(sweep)) return NAVSYS_STATUS_UNSUPPORTED;
    *out_supported = true;
    return NAVSYS_STATUS_OK;
}

navsys_status_t byul_maze_generate_sidewinder(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_sidewinder_sweep_t sweep,
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
        || (height & UINT32_C(1)) == 0
        || !is_supported_sweep(sweep)) {
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
    return generate_with_sweep(
        origin_x, origin_y, width, height, sweep, context, out_maze);
}

maze_t* maze_make_sidewinder(int x0, int y0, int width, int height) {
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
    return byul_maze_generate_sidewinder(
               x0,
               y0,
               static_cast<uint32_t>(width),
               static_cast<uint32_t>(height),
               BYUL_MAZE_SIDEWINDER_EAST_NORTH,
               &options,
               &maze)
            == NAVSYS_STATUS_OK
        ? maze
        : nullptr;
}
