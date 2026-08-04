#include "maze_aldous_broder.h"
#include "internal/maze_private.hpp"

#include <cstdint>
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
    grid[static_cast<size_t>(to_y) * grid_width + to_x] = passage_cell;
    grid[static_cast<size_t>((from_y + to_y) / 2) * grid_width
        + (from_x + to_x) / 2] = passage_cell;
}

} // namespace

navsys_status_t byul_maze_generate_aldous_broder_internal(
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
        std::vector<uint8_t> visited(total, uint8_t{0});

        int current = static_cast<int>(context.bounded(total));
        visited[current] = 1;
        uint32_t visited_count = 1;
        grid[static_cast<size_t>(graph.y(current)) * w + graph.x(current)]
            = passage_cell;

        while (visited_count < total) {
            const navsys_status_t step_status = context.begin_step();
            if (step_status != NAVSYS_STATUS_OK) {
                maze_destroy(maze);
                return step_status;
            }
            int neighbors[4];
            const uint32_t neighbor_count = graph.neighbors(current, neighbors);
            const int next = neighbors[context.bounded(neighbor_count)];
            if (visited[next] == 0) {
                open_edge(grid, w, graph, current, next);
                visited[next] = 1;
                ++visited_count;
            }
            current = next;
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

maze_t* maze_make_aldous_broder(int x0, int y0, int width, int height) {
    if (width < 3 || height < 3 || width % 2 == 0 || height % 2 == 0) {
        return nullptr;
    }
    byul_maze_generation_context context(
        byul_maze_generation_legacy_seed(), 0, nullptr, nullptr);
    maze_t* maze = nullptr;
    return byul_maze_generate_aldous_broder_internal(
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
