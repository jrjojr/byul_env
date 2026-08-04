#include "maze_binary.h"
#include "internal/maze_private.hpp"

#include <cstdint>
#include <limits>
#include <new>

namespace {

navsys_status_t set_blocked(
    maze_t* maze, int32_t origin_x, int32_t origin_y,
    int x, int y, bool blocked) {
    bool changed = false;
    return byul_maze_set_blocked(
        maze, origin_x + x, origin_y + y, blocked, &changed);
}

bool is_supported_bias(byul_maze_binary_bias_t bias) {
    const int value = static_cast<int>(bias);
    return value >= static_cast<int>(BYUL_MAZE_BINARY_BIAS_NORTH_WEST)
        && value <= static_cast<int>(BYUL_MAZE_BINARY_BIAS_SOUTH_EAST);
}

bool has_representable_bound(int32_t origin, uint32_t extent) {
    return extent <= static_cast<uint32_t>(std::numeric_limits<int32_t>::max())
        && static_cast<uint64_t>(extent)
        <= static_cast<uint64_t>(std::numeric_limits<int32_t>::max())
            - static_cast<int64_t>(origin);
}

} // namespace

navsys_status_t byul_maze_generate_binary_internal(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_generation_context& context,
    maze_t** out_maze) noexcept {
    return byul_maze_generate_binary_with_bias_internal(
        origin_x,
        origin_y,
        width,
        height,
        BYUL_MAZE_BINARY_BIAS_SOUTH_EAST,
        context,
        out_maze);
}

navsys_status_t byul_maze_generate_binary_with_bias_internal(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_binary_bias_t bias,
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

        const bool east = bias == BYUL_MAZE_BINARY_BIAS_NORTH_EAST
            || bias == BYUL_MAZE_BINARY_BIAS_SOUTH_EAST;
        const bool south = bias == BYUL_MAZE_BINARY_BIAS_SOUTH_WEST
            || bias == BYUL_MAZE_BINARY_BIAS_SOUTH_EAST;
        const int horizontal_delta = east ? 2 : -2;
        const int vertical_delta = south ? 2 : -2;

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

                const bool can_horizontal = x + horizontal_delta > 0
                    && x + horizontal_delta < w;
                const bool can_vertical = y + vertical_delta > 0
                    && y + vertical_delta < h;
                if (can_horizontal && can_vertical) {
                    status = context.bounded(2) == 0
                        ? set_blocked(maze, origin_x, origin_y,
                            x + horizontal_delta / 2, y, false)
                        : set_blocked(maze, origin_x, origin_y,
                            x, y + vertical_delta / 2, false);
                } else if (can_horizontal) {
                    status = set_blocked(
                        maze, origin_x, origin_y,
                        x + horizontal_delta / 2, y, false);
                } else if (can_vertical) {
                    status = set_blocked(
                        maze, origin_x, origin_y,
                        x, y + vertical_delta / 2, false);
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

navsys_status_t byul_maze_binary_bias_is_supported(
    byul_maze_binary_bias_t bias, bool* out_supported) {
    if (!out_supported) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!is_supported_bias(bias)) return NAVSYS_STATUS_UNSUPPORTED;
    *out_supported = true;
    return NAVSYS_STATUS_OK;
}

navsys_status_t byul_maze_generate_binary_tree(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_binary_bias_t bias,
    const byul_maze_generate_options_t* options,
    maze_t** out_maze) {
    if (!out_maze) return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_maze = nullptr;
    if (!options
        || options->struct_size < sizeof(byul_maze_generate_options_t)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    if (options->abi_version != BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION
        || !is_supported_bias(bias)
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
    const uint64_t default_steps = cells > std::numeric_limits<uint64_t>::max() / 4
        ? std::numeric_limits<uint64_t>::max()
        : cells * 4;
    byul_maze_generation_context context(
        options->seed,
        options->max_steps != 0 ? options->max_steps : default_steps,
        options->cancel_func,
        options->cancel_userdata);
    return byul_maze_generate_binary_with_bias_internal(
        origin_x, origin_y, width, height, bias, context, out_maze);
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
