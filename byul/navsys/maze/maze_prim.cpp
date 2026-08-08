#include "maze_prim.h"
#include "internal/maze_private.hpp"

#include <cstdint>
#include <limits>
#include <new>
#include <vector>

namespace {

constexpr int wall_cell = 1;
constexpr int passage_cell = 0;

struct frontier_t {
    int wall_x;
    int wall_y;
    int target_x;
    int target_y;
};

bool is_logical_cell(int x, int y, int width, int height) {
    return x > 0 && y > 0 && x < width - 1 && y < height - 1;
}

void add_frontiers(
    int cell_x,
    int cell_y,
    int width,
    int height,
    const std::vector<int>& grid,
    std::vector<frontier_t>& frontiers) {
    static constexpr int delta_x[4] = {0, 0, -1, 1};
    static constexpr int delta_y[4] = {-1, 1, 0, 0};
    for (int direction = 0; direction < 4; ++direction) {
        const int target_x = cell_x + delta_x[direction] * 2;
        const int target_y = cell_y + delta_y[direction] * 2;
        if (is_logical_cell(target_x, target_y, width, height)
            && grid[static_cast<size_t>(target_y) * width + target_x]
                == wall_cell) {
            frontiers.push_back({
                cell_x + delta_x[direction],
                cell_y + delta_y[direction],
                target_x,
                target_y
            });
        }
    }
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

navsys_status_t byul_maze_generate_prim_profiled_internal(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_generation_context& context,
    byul_maze_prim_stats* stats,
    maze_t** out_maze) noexcept {
    if (!out_maze) return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_maze = nullptr;
    if (stats) *stats = {};
    if (width < 3 || height < 3
        || (width & UINT32_C(1)) == 0
        || (height & UINT32_C(1)) == 0) {
        return NAVSYS_STATUS_UNSUPPORTED;
    }
    if (!has_representable_bound(origin_x, width)
        || !has_representable_bound(origin_y, height)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const uint64_t cells = static_cast<uint64_t>(width) * height;
    const uint64_t columns = width / 2;
    const uint64_t rows = height / 2;
    const uint64_t edges =
        (columns - 1) * rows + (rows - 1) * columns;
    if (cells > std::numeric_limits<size_t>::max()
        || edges > std::numeric_limits<uint32_t>::max()) {
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
        std::vector<int> grid(
            static_cast<size_t>(width) * height, wall_cell);
        std::vector<frontier_t> frontiers;
        frontiers.reserve(static_cast<size_t>(edges));

        const uint32_t logical_width = (width - 1u) / 2u;
        const uint32_t logical_height = (height - 1u) / 2u;
        const int start_x = 1 + static_cast<int>(context.bounded(logical_width)) * 2;
        const int start_y = 1 + static_cast<int>(context.bounded(logical_height)) * 2;
        grid[static_cast<size_t>(start_y) * w + start_x] = passage_cell;
        add_frontiers(start_x, start_y, w, h, grid, frontiers);
        if (stats) stats->peak_frontier = frontiers.size();

        while (!frontiers.empty()) {
            const navsys_status_t step_status = context.begin_step();
            if (step_status != NAVSYS_STATUS_OK) {
                maze_destroy(maze);
                return step_status;
            }
            const size_t selected = context.bounded(
                static_cast<uint32_t>(frontiers.size()));
            const frontier_t frontier = frontiers[selected];
            frontiers[selected] = frontiers.back();
            frontiers.pop_back();
            if (stats) ++stats->frontier_pops;

            const size_t target_index =
                static_cast<size_t>(frontier.target_y) * w + frontier.target_x;
            if (grid[target_index] == passage_cell) {
                if (stats) ++stats->stale_edges;
                continue;
            }
            grid[static_cast<size_t>(frontier.wall_y) * w + frontier.wall_x]
                = passage_cell;
            grid[target_index] = passage_cell;
            add_frontiers(
                frontier.target_x,
                frontier.target_y,
                w,
                h,
                grid,
                frontiers);
            if (stats) {
                ++stats->accepted_edges;
                if (frontiers.size() > stats->peak_frontier) {
                    stats->peak_frontier = frontiers.size();
                }
            }
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
                const navsys_status_t status = byul_maze_set_blocked(
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

navsys_status_t byul_maze_generate_prim_internal(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_generation_context& context,
    maze_t** out_maze) noexcept {
    return byul_maze_generate_prim_profiled_internal(
        origin_x, origin_y, width, height, context, nullptr, out_maze);
}

navsys_status_t byul_maze_generate_randomized_prim(
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
    const uint64_t columns = width / 2;
    const uint64_t rows = height / 2;
    const uint64_t edges =
        (columns - 1) * rows + (rows - 1) * columns;
    if (cells > std::numeric_limits<size_t>::max()
        || edges > std::numeric_limits<uint32_t>::max()) {
        return NAVSYS_STATUS_LIMIT_REACHED;
    }
    const uint64_t default_steps =
        cells > std::numeric_limits<uint64_t>::max() / 16
        ? std::numeric_limits<uint64_t>::max()
        : cells * 16;
    byul_maze_generation_context context(
        options->seed,
        options->max_steps != 0 ? options->max_steps : default_steps,
        options->cancel_func,
        options->cancel_userdata);
    return byul_maze_generate_prim_internal(
        origin_x, origin_y, width, height, context, out_maze);
}

maze_t* maze_maze_prim(int x0, int y0, int width, int height) {
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
    return byul_maze_generate_randomized_prim(
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
