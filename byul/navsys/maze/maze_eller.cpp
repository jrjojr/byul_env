#include "maze_eller.h"
#include "internal/maze_private.hpp"

#include <cstdint>
#include <limits>
#include <new>
#include <vector>

namespace {

navsys_status_t set_blocked(
    maze_t* maze, int32_t origin_x, int32_t origin_y,
    int x, int y, bool blocked) {
    bool changed = false;
    return byul_maze_set_blocked(
        maze, origin_x + x, origin_y + y, blocked, &changed);
}

void merge_sets(std::vector<int>& sets, int from, int to) {
    for (int& set : sets) {
        if (set == from) set = to;
    }
}

bool has_representable_bound(int32_t origin, uint32_t length) {
    if (length == 0) return true;
    const int64_t last = static_cast<int64_t>(origin)
        + static_cast<int64_t>(length) - 1;
    return last <= std::numeric_limits<int32_t>::max();
}

} // namespace

navsys_status_t byul_maze_generate_eller_internal(
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

        const int columns = (w - 1) / 2;
        const int rows = (h - 1) / 2;
        std::vector<int> sets(static_cast<size_t>(columns), 0);
        int next_set = 1;

        for (int row = 0; row < rows; ++row) {
            const int y = 1 + row * 2;
            const bool last_row = row + 1 == rows;
            for (int column = 0; column < columns; ++column) {
                const navsys_status_t step_status = context.begin_step();
                if (step_status != NAVSYS_STATUS_OK) {
                    maze_destroy(maze);
                    return step_status;
                }
                if (sets[column] == 0) sets[column] = next_set++;
                const navsys_status_t status = set_blocked(
                    maze, origin_x, origin_y, 1 + column * 2, y, false);
                if (status != NAVSYS_STATUS_OK) {
                    maze_destroy(maze);
                    return status;
                }
            }

            for (int column = 0; column + 1 < columns; ++column) {
                if (sets[column] == sets[column + 1]) continue;
                if (!last_row && context.bounded(2) == 0) continue;
                const int from = sets[column + 1];
                const int to = sets[column];
                merge_sets(sets, from, to);
                const navsys_status_t status = set_blocked(
                    maze, origin_x, origin_y, 2 + column * 2, y, false);
                if (status != NAVSYS_STATUS_OK) {
                    maze_destroy(maze);
                    return status;
                }
            }
            if (last_row) break;

            std::vector<int> next_sets(static_cast<size_t>(columns), 0);
            std::vector<int> handled_sets;
            for (int column = 0; column < columns; ++column) {
                const int set_id = sets[column];
                bool handled = false;
                for (const int value : handled_sets) {
                    if (value == set_id) {
                        handled = true;
                        break;
                    }
                }
                if (handled) continue;
                handled_sets.push_back(set_id);

                std::vector<int> members;
                for (int candidate = column; candidate < columns; ++candidate) {
                    if (sets[candidate] == set_id) members.push_back(candidate);
                }
                const int required = members[context.bounded(
                    static_cast<uint32_t>(members.size()))];
                for (const int member : members) {
                    if (member != required && context.bounded(2) == 0) continue;
                    next_sets[member] = set_id;
                    const int x = 1 + member * 2;
                    navsys_status_t status = set_blocked(
                        maze, origin_x, origin_y, x, y + 1, false);
                    if (status == NAVSYS_STATUS_OK) {
                        status = set_blocked(
                            maze, origin_x, origin_y, x, y + 2, false);
                    }
                    if (status != NAVSYS_STATUS_OK) {
                        maze_destroy(maze);
                        return status;
                    }
                }
            }
            sets.swap(next_sets);
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

navsys_status_t byul_maze_generate_eller(
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
    const uint64_t default_steps = cells > std::numeric_limits<uint64_t>::max() / 4
        ? std::numeric_limits<uint64_t>::max()
        : cells * 4;
    byul_maze_generation_context context(
        options->seed,
        options->max_steps != 0 ? options->max_steps : default_steps,
        options->cancel_func,
        options->cancel_userdata);
    return byul_maze_generate_eller_internal(
        origin_x, origin_y, width, height, context, out_maze);
}

maze_t* maze_make_eller(int x0, int y0, int width, int height) {
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
    return byul_maze_generate_eller(
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
