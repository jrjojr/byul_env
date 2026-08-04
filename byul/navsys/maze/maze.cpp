#include <stdlib.h>
#include <string.h>

#include <cstdint>
#include <limits>

#include "maze.h"
#include "internal/maze_private.hpp"

#include "maze_recursive.h"
#include "maze_prim.h"
#include "maze_binary.h"
#include "maze_eller.h"

#include "maze_aldous_broder.h"
#include "maze_wilson.h"
#include "maze_hunt_and_kill.h"
#include "maze_sidewinder.h"

#include "maze_recursive_division.h"
#include "maze_kruskal.h"
#include "maze_room_blend.h"

namespace {

bool is_known_algorithm(byul_maze_algorithm_t algorithm) {
    const int value = static_cast<int>(algorithm);
    return value >= static_cast<int>(BYUL_MAZE_ALGORITHM_RECURSIVE_BACKTRACKER)
        && value <= static_cast<int>(BYUL_MAZE_ALGORITHM_ROOM_BLEND);
}

bool has_supported_dimensions(
    byul_maze_algorithm_t algorithm, uint32_t width, uint32_t height) {
    if (algorithm == BYUL_MAZE_ALGORITHM_ROOM_BLEND) {
        return width >= 9 && height >= 9;
    }
    return width >= 3 && height >= 3
        && (width & UINT32_C(1)) != 0
        && (height & UINT32_C(1)) != 0;
}

bool has_representable_bound(int32_t origin, uint32_t extent) {
    return extent <= static_cast<uint32_t>(std::numeric_limits<int32_t>::max())
        && static_cast<uint64_t>(extent)
        <= static_cast<uint64_t>(std::numeric_limits<int32_t>::max())
            - static_cast<int64_t>(origin);
}

uint64_t default_step_limit(
    byul_maze_algorithm_t algorithm, uint64_t cells) {
    uint64_t multiplier = 0;
    switch (algorithm) {
        case BYUL_MAZE_ALGORITHM_RECURSIVE_BACKTRACKER:
            multiplier = 8;
            break;
        case BYUL_MAZE_ALGORITHM_RANDOMIZED_PRIM:
            multiplier = 16;
            break;
        case BYUL_MAZE_ALGORITHM_BINARY_TREE:
            multiplier = 4;
            break;
        case BYUL_MAZE_ALGORITHM_ELLER:
            multiplier = 4;
            break;
        case BYUL_MAZE_ALGORITHM_ALDOUS_BRODER:
        case BYUL_MAZE_ALGORITHM_WILSON:
            multiplier = 256;
            break;
        case BYUL_MAZE_ALGORITHM_HUNT_AND_KILL:
            multiplier = 16;
            break;
        case BYUL_MAZE_ALGORITHM_SIDEWINDER:
            multiplier = 4;
            break;
        case BYUL_MAZE_ALGORITHM_RECURSIVE_DIVISION:
            multiplier = 4;
            break;
        case BYUL_MAZE_ALGORITHM_RANDOMIZED_KRUSKAL:
            multiplier = 8;
            break;
        case BYUL_MAZE_ALGORITHM_ROOM_BLEND:
            multiplier = 32;
            break;
        default:
            return 0;
    }
    if (cells > std::numeric_limits<uint64_t>::max() / multiplier) {
        return std::numeric_limits<uint64_t>::max();
    }
    return cells * multiplier;
}

} // namespace

navsys_status_t byul_maze_algorithm_is_supported(
    byul_maze_algorithm_t algorithm, bool* out_supported) {
    if (!out_supported) return NAVSYS_STATUS_INVALID_ARGUMENT;
    if (!is_known_algorithm(algorithm)) return NAVSYS_STATUS_UNSUPPORTED;

    *out_supported = true;
    return NAVSYS_STATUS_OK;
}

navsys_status_t byul_maze_generate(
    byul_maze_algorithm_t algorithm,
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
    if (options->abi_version != BYUL_MAZE_GENERATE_OPTIONS_ABI_VERSION) {
        return NAVSYS_STATUS_UNSUPPORTED;
    }
    if (!is_known_algorithm(algorithm)
        || !has_supported_dimensions(algorithm, width, height)) {
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

    bool supported = false;
    const navsys_status_t support_status =
        byul_maze_algorithm_is_supported(algorithm, &supported);
    if (support_status != NAVSYS_STATUS_OK) return support_status;
    if (!supported) return NAVSYS_STATUS_UNSUPPORTED;

    const uint64_t step_limit = options->max_steps != 0
        ? options->max_steps
        : default_step_limit(algorithm, cells);
    byul_maze_generation_context context(
        options->seed,
        step_limit,
        options->cancel_func,
        options->cancel_userdata);
    switch (algorithm) {
        case BYUL_MAZE_ALGORITHM_RECURSIVE_BACKTRACKER:
            return byul_maze_generate_recursive_internal(
                origin_x, origin_y, width, height, context, out_maze);
        case BYUL_MAZE_ALGORITHM_RANDOMIZED_PRIM:
            return byul_maze_generate_prim_internal(
                origin_x, origin_y, width, height, context, out_maze);
        case BYUL_MAZE_ALGORITHM_BINARY_TREE:
            return byul_maze_generate_binary_internal(
                origin_x, origin_y, width, height, context, out_maze);
        case BYUL_MAZE_ALGORITHM_ELLER:
            return byul_maze_generate_eller_internal(
                origin_x, origin_y, width, height, context, out_maze);
        case BYUL_MAZE_ALGORITHM_ALDOUS_BRODER:
            return byul_maze_generate_aldous_broder_internal(
                origin_x, origin_y, width, height, context, out_maze);
        case BYUL_MAZE_ALGORITHM_WILSON:
            return byul_maze_generate_wilson_internal(
                origin_x, origin_y, width, height, context, out_maze);
        case BYUL_MAZE_ALGORITHM_HUNT_AND_KILL:
            return byul_maze_generate_hunt_and_kill_internal(
                origin_x, origin_y, width, height, context, out_maze);
        case BYUL_MAZE_ALGORITHM_SIDEWINDER:
            return byul_maze_generate_sidewinder_internal(
                origin_x, origin_y, width, height, context, out_maze);
        case BYUL_MAZE_ALGORITHM_RECURSIVE_DIVISION:
            return byul_maze_generate_recursive_division_internal(
                origin_x, origin_y, width, height, context, out_maze);
        case BYUL_MAZE_ALGORITHM_RANDOMIZED_KRUSKAL:
            return byul_maze_generate_kruskal_internal(
                origin_x, origin_y, width, height, context, out_maze);
        case BYUL_MAZE_ALGORITHM_ROOM_BLEND:
            return byul_maze_generate_room_blend_internal(
                origin_x, origin_y, width, height, context, out_maze);
        default:
            return NAVSYS_STATUS_UNSUPPORTED;
    }
}

maze_t* maze_make(int x0, int y0, int width, int height, maze_type_t type) {
    maze_t* maze = nullptr;

    switch (type) {
        case MAZE_TYPE_RECURSIVE:
            maze = maze_make_recursive(x0, y0, width, height); 
            return maze;
        case MAZE_TYPE_PRIM:
            maze = maze_maze_prim(x0, y0, width, height); 
            return maze;
        case MAZE_TYPE_BINARY:
            maze = maze_make_binary(x0, y0, width, height); 
            return maze;
        case MAZE_TYPE_ELLER:
            maze = maze_make_eller(x0, y0, width, height); 
            return maze;
        case MAZE_TYPE_ALDOUS_BRODER:
            maze = maze_make_aldous_broder(x0, y0, width, height); 
            return maze;
        case MAZE_TYPE_WILSON:
            maze = maze_make_wilson(x0, y0, width, height); 
            return maze;
        case MAZE_TYPE_HUNT_AND_KILL:
            maze = maze_make_hunt_and_kill(x0, y0, width, height); 
            return maze;
        case MAZE_TYPE_SIDEWINDER:
            maze = maze_make_sidewinder(x0, y0, width, height); 
            return maze;
        case MAZE_TYPE_RECURSIVE_DIVISION:
            maze = maze_make_recursive_division(x0, y0, width, height); 
            return maze;            
        case MAZE_TYPE_KRUSKAL:
            maze = maze_make_kruskal(x0, y0, width, height);
            return maze;            
        case MAZE_TYPE_ROOM_BLEND:
            maze = maze_make_room_blend(x0, y0, width, height);
            return maze;                        
        default:
            maze = maze_make_kruskal(x0, y0, width, height);
            return maze;            
    }
    return nullptr;
}
