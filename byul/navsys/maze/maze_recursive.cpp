#include "maze_recursive.h"
#include "internal/maze_private.hpp"

#include <cstdint>
#include <limits>
#include <new>
#include <vector>

namespace {

bool is_valid_cell(int x, int y, int width, int height) {
    return x > 0 && y > 0 && x < width - 1 && y < height - 1;
}

struct dfs_frame_t {
    int cell_x;
    int cell_y;
    int directions[4];
    uint8_t next_direction;
};

navsys_status_t push_frame(
    int cell_x,
    int cell_y,
    int width,
    std::vector<uint8_t>& visited,
    std::vector<dfs_frame_t>& frames,
    byul_maze_generation_context& context) {
    const navsys_status_t step_status = context.begin_step();
    if (step_status != NAVSYS_STATUS_OK) return step_status;

    dfs_frame_t frame{cell_x, cell_y, {0, 1, 2, 3}, uint8_t{0}};
    for (uint32_t i = 4; i > 1; --i) {
        const uint32_t selected = context.bounded(i);
        const int temporary = frame.directions[i - 1];
        frame.directions[i - 1] = frame.directions[selected];
        frame.directions[selected] = temporary;
    }
    visited[static_cast<size_t>(cell_y) * width + cell_x] = 1;
    frames.push_back(frame);
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

navsys_status_t byul_maze_generate_recursive_profiled_internal(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_generation_context& context,
    byul_maze_recursive_stats* stats,
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
    const uint64_t vertices =
        static_cast<uint64_t>(width / 2) * (height / 2);
    if (cells > std::numeric_limits<size_t>::max()
        || vertices > std::numeric_limits<size_t>::max()) {
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
        std::vector<dfs_frame_t> frames;
        frames.reserve(static_cast<size_t>(vertices));
        bool changed = false;
        navsys_status_t status = byul_maze_set_blocked(
            maze, origin_x + 1, origin_y + 1, false, &changed);
        if (status == NAVSYS_STATUS_OK) {
            status = push_frame(1, 1, w, visited, frames, context);
            if (status == NAVSYS_STATUS_OK && stats) {
                stats->visited_cells = 1;
                stats->peak_frames = 1;
            }
        }
        static constexpr int delta_x[4] = {0, 0, -1, 1};
        static constexpr int delta_y[4] = {-1, 1, 0, 0};
        while (status == NAVSYS_STATUS_OK && !frames.empty()) {
            dfs_frame_t& frame = frames.back();
            if (frame.next_direction == 4) {
                frames.pop_back();
                continue;
            }
            const int direction = frame.directions[frame.next_direction++];
            const int cell_x = frame.cell_x;
            const int cell_y = frame.cell_y;
            const int next_x = cell_x + delta_x[direction] * 2;
            const int next_y = cell_y + delta_y[direction] * 2;
            if (!is_valid_cell(next_x, next_y, w, h)
                || visited[static_cast<size_t>(next_y) * w + next_x] != 0) {
                continue;
            }
            status = byul_maze_set_blocked(
                maze,
                origin_x + cell_x + delta_x[direction],
                origin_y + cell_y + delta_y[direction],
                false,
                &changed);
            if (status == NAVSYS_STATUS_OK) {
                status = byul_maze_set_blocked(
                    maze, origin_x + next_x, origin_y + next_y,
                    false, &changed);
            }
            if (status == NAVSYS_STATUS_OK) {
                status = push_frame(
                    next_x, next_y, w, visited, frames, context);
                if (status == NAVSYS_STATUS_OK && stats) {
                    ++stats->visited_cells;
                    if (frames.size() > stats->peak_frames) {
                        stats->peak_frames = frames.size();
                    }
                }
            }
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

navsys_status_t byul_maze_generate_recursive_internal(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_generation_context& context,
    maze_t** out_maze) noexcept {
    return byul_maze_generate_recursive_profiled_internal(
        origin_x, origin_y, width, height, context, nullptr, out_maze);
}

navsys_status_t byul_maze_generate_recursive_backtracker(
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
    if (cells > std::numeric_limits<size_t>::max()) {
        return NAVSYS_STATUS_LIMIT_REACHED;
    }
    const uint64_t default_steps =
        cells > std::numeric_limits<uint64_t>::max() / 8
        ? std::numeric_limits<uint64_t>::max()
        : cells * 8;
    byul_maze_generation_context context(
        options->seed,
        options->max_steps != 0 ? options->max_steps : default_steps,
        options->cancel_func,
        options->cancel_userdata);
    return byul_maze_generate_recursive_internal(
        origin_x, origin_y, width, height, context, out_maze);
}

maze_t* maze_make_recursive(int x0, int y0, int width, int height) {
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
    return byul_maze_generate_recursive_backtracker(
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
