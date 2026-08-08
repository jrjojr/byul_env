#include "maze_kruskal.h"
#include "internal/maze_private.hpp"

#include <cstdint>
#include <limits>
#include <new>
#include <vector>

namespace {

constexpr int wall_cell = 1;
constexpr int passage_cell = 0;

struct wall_t {
    int wx;
    int wy;
    size_t first;
    size_t second;
};

class disjoint_set_t final {
public:
    explicit disjoint_set_t(size_t count)
        : parent_(count), size_(count, 1) {
        for (size_t index = 0; index < count; ++index) parent_[index] = index;
    }

    size_t find(size_t value) noexcept {
        while (parent_[value] != value) {
            parent_[value] = parent_[parent_[value]];
            value = parent_[value];
        }
        return value;
    }

    bool unite(size_t first, size_t second) noexcept {
        first = find(first);
        second = find(second);
        if (first == second) return false;
        if (size_[first] < size_[second]) {
            const size_t temporary = first;
            first = second;
            second = temporary;
        }
        parent_[second] = first;
        size_[first] += size_[second];
        return true;
    }

private:
    std::vector<size_t> parent_;
    std::vector<size_t> size_;
};

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

navsys_status_t byul_maze_generate_kruskal_internal(
    int32_t origin_x,
    int32_t origin_y,
    uint32_t width,
    uint32_t height,
    byul_maze_generation_context& context,
    maze_t** out_maze) noexcept {
    if (!out_maze) return NAVSYS_STATUS_INVALID_ARGUMENT;
    *out_maze = nullptr;
    if (width < 3 || height < 3
        || (width & UINT32_C(1)) == 0
        || (height & UINT32_C(1)) == 0) {
        return NAVSYS_STATUS_UNSUPPORTED;
    }
    if (!has_representable_bound(origin_x, width)
        || !has_representable_bound(origin_y, height)) {
        return NAVSYS_STATUS_INVALID_ARGUMENT;
    }
    const navsys_status_t initial_poll = context.poll();
    if (initial_poll != NAVSYS_STATUS_OK) return initial_poll;

    const uint64_t columns = width / 2;
    const uint64_t rows = height / 2;
    const uint64_t vertex_count = columns * rows;
    const uint64_t edge_count =
        (columns - 1) * rows + (rows - 1) * columns;
    if (vertex_count > std::numeric_limits<size_t>::max()
        || edge_count > std::numeric_limits<uint32_t>::max()) {
        return NAVSYS_STATUS_LIMIT_REACHED;
    }

    maze_t* maze = maze_create_full(
        origin_x, origin_y, static_cast<int>(width), static_cast<int>(height));
    if (!maze) return NAVSYS_STATUS_OUT_OF_MEMORY;

    try {
        const int w = static_cast<int>(width);
        const int h = static_cast<int>(height);
        std::vector<std::vector<int>> grid(h, std::vector<int>(w, wall_cell));
        disjoint_set_t sets(static_cast<size_t>(vertex_count));
        std::vector<wall_t> walls;
        walls.reserve(static_cast<size_t>(edge_count));

        for (int y = 1; y < h; y += 2) {
            for (int x = 1; x < w; x += 2) {
                grid[y][x] = passage_cell;
                const size_t vertex =
                    static_cast<size_t>((y - 1) / 2) * columns
                    + static_cast<size_t>((x - 1) / 2);

                if (x + 2 < w) {
                    walls.push_back({
                        x + 1, y, vertex, vertex + 1});
                }
                if (y + 2 < h) {
                    walls.push_back({
                        x, y + 1,
                        vertex, vertex + static_cast<size_t>(columns)});
                }
            }
        }

        for (size_t i = walls.size(); i > 1; --i) {
            const navsys_status_t poll_status = context.poll();
            if (poll_status != NAVSYS_STATUS_OK) {
                maze_destroy(maze);
                return poll_status;
            }
            const size_t selected = context.bounded(static_cast<uint32_t>(i));
            const wall_t temporary = walls[i - 1];
            walls[i - 1] = walls[selected];
            walls[selected] = temporary;
        }

        size_t accepted = 0;
        for (const wall_t& wall : walls) {
            const navsys_status_t step_status = context.begin_step();
            if (step_status != NAVSYS_STATUS_OK) {
                maze_destroy(maze);
                return step_status;
            }
            if (!sets.unite(wall.first, wall.second)) continue;
            grid[wall.wy][wall.wx] = passage_cell;
            ++accepted;
            if (accepted + 1 == vertex_count) break;
        }

        for (int y = 0; y < h; ++y) {
            for (int x = 0; x < w; ++x) {
                if (grid[y][x] == wall_cell) {
                    const navsys_status_t poll_status = context.poll();
                    if (poll_status != NAVSYS_STATUS_OK) {
                        maze_destroy(maze);
                        return poll_status;
                    }
                    bool changed = false;
                    const navsys_status_t block_status = byul_maze_set_blocked(
                        maze, x + origin_x, y + origin_y, true, &changed);
                    if (block_status != NAVSYS_STATUS_OK) {
                        maze_destroy(maze);
                        return block_status;
                    }
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

navsys_status_t byul_maze_generate_randomized_kruskal(
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
    if (edges > std::numeric_limits<uint32_t>::max()) {
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
    return byul_maze_generate_kruskal_internal(
        origin_x, origin_y, width, height, context, out_maze);
}

maze_t* maze_make_kruskal(int x0, int y0, int width, int height) {
    if (width < 3 || height < 3) return nullptr;
    if (width % 2 == 0 || height % 2 == 0) return nullptr;

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
    return byul_maze_generate_randomized_kruskal(
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
