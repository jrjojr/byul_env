#include "maze_wilson.h"
#include "internal/maze_private.hpp"

#include <cstdint>
#include <limits>
#include <new>
#include <vector>

namespace {

constexpr uint8_t wall_cell = 1;
constexpr uint8_t passage_cell = 0;

struct logical_graph_t {
    int columns;
    int rows;

    int x(int node) const { return 1 + (node % columns) * 2; }
    int y(int node) const { return 1 + (node / columns) * 2; }

    uint32_t neighbors(int node, int out[4]) const {
        const int column = node % columns;
        const int row = node / columns;
        uint32_t count = 0;
        if (row > 0) out[count++] = node - columns;
        if (row + 1 < rows) out[count++] = node + columns;
        if (column > 0) out[count++] = node - 1;
        if (column + 1 < columns) out[count++] = node + 1;
        return count;
    }
};

void open_edge(
    std::vector<uint8_t>& grid,
    int grid_width,
    const logical_graph_t& graph,
    int from,
    int to) {
    const int from_x = graph.x(from);
    const int from_y = graph.y(from);
    const int to_x = graph.x(to);
    const int to_y = graph.y(to);
    grid[static_cast<size_t>(from_y) * grid_width + from_x] = passage_cell;
    grid[static_cast<size_t>(to_y) * grid_width + to_x] = passage_cell;
    grid[static_cast<size_t>((from_y + to_y) / 2) * grid_width
        + (from_x + to_x) / 2] = passage_cell;
}

int choose_unvisited(
    const std::vector<uint8_t>& in_tree,
    uint32_t unvisited_count,
    byul_maze_generation_context& context) {
    uint32_t selected = context.bounded(unvisited_count);
    for (size_t node = 0; node < in_tree.size(); ++node) {
        if (in_tree[node] != 0) continue;
        if (selected == 0) return static_cast<int>(node);
        --selected;
    }
    return -1;
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

navsys_status_t byul_maze_generate_wilson_internal(
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
        const logical_graph_t graph{(w - 1) / 2, (h - 1) / 2};
        const uint32_t total = static_cast<uint32_t>(graph.columns * graph.rows);
        std::vector<uint8_t> grid(
            static_cast<size_t>(width) * height, wall_cell);
        std::vector<uint8_t> in_tree(total, uint8_t{0});
        std::vector<int> path_position(total, -1);
        std::vector<int> path;

        const int root = static_cast<int>(context.bounded(total));
        in_tree[root] = 1;
        uint32_t tree_count = 1;
        grid[static_cast<size_t>(graph.y(root)) * w + graph.x(root)]
            = passage_cell;

        while (tree_count < total) {
            const int start = choose_unvisited(
                in_tree, total - tree_count, context);
            if (start < 0) {
                maze_destroy(maze);
                return NAVSYS_STATUS_CORRUPT_STATE;
            }
            path.clear();
            path.push_back(start);
            path_position[start] = 0;
            int walk = start;

            while (in_tree[walk] == 0) {
                const navsys_status_t step_status = context.begin_step();
                if (step_status != NAVSYS_STATUS_OK) {
                    maze_destroy(maze);
                    return step_status;
                }
                int neighbors[4];
                const uint32_t neighbor_count = graph.neighbors(walk, neighbors);
                const int next = neighbors[context.bounded(neighbor_count)];
                const int loop_position = path_position[next];
                if (loop_position >= 0) {
                    for (size_t index = static_cast<size_t>(loop_position + 1);
                         index < path.size(); ++index) {
                        path_position[path[index]] = -1;
                    }
                    path.resize(static_cast<size_t>(loop_position + 1));
                } else {
                    path_position[next] = static_cast<int>(path.size());
                    path.push_back(next);
                }
                walk = next;
            }

            for (size_t index = 0; index + 1 < path.size(); ++index) {
                const int node = path[index];
                open_edge(grid, w, graph, node, path[index + 1]);
                if (in_tree[node] == 0) {
                    in_tree[node] = 1;
                    ++tree_count;
                }
            }
            for (const int node : path) path_position[node] = -1;
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

navsys_status_t byul_maze_generate_wilson(
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
        cells > std::numeric_limits<uint64_t>::max() / 256
        ? std::numeric_limits<uint64_t>::max()
        : cells * 256;
    byul_maze_generation_context context(
        options->seed,
        options->max_steps != 0 ? options->max_steps : default_steps,
        options->cancel_func,
        options->cancel_userdata);
    return byul_maze_generate_wilson_internal(
        origin_x, origin_y, width, height, context, out_maze);
}

maze_t* maze_make_wilson(int x0, int y0, int width, int height) {
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
    return byul_maze_generate_wilson(
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
