#include <stdlib.h>
#include <string.h>

#include "internal/maze.h"

void maze_generate(maze_t* maze, maze_type_t type) {
    if (!maze) return;

    switch (type) {
        case MAZE_TYPE_RECURSIVE:
            maze_generate_recursive(maze); break;
        case MAZE_TYPE_PRIM:
            maze_generate_prim(maze); break;
        case MAZE_TYPE_BINARY:
            maze_generate_binary(maze); break;
        case MAZE_TYPE_ELLER:
            maze_generate_eller(maze); break;
        case MAZE_TYPE_ALDOUS_BRODER:
            maze_generate_aldous_broder(maze); break;
        case MAZE_TYPE_WILSON:
            maze_generate_wilson(maze); break;
        case MAZE_TYPE_HUNT_AND_KILL:
            maze_generate_hunt_and_kill(maze); break;
        case MAZE_TYPE_SIDEWINDER:
            maze_generate_sidewinder(maze); break;
        case MAZE_TYPE_RECURSIVE_DIVISION:
            maze_generate_recursive_division(maze); break;
        case MAZE_TYPE_KRUSKAL:
            maze_generate_kruskal(maze); break;
        case MAZE_TYPE_ROOM_BLEND:
            maze_generate_room_blend(maze); break;
        default:
            maze_generate_kruskal(maze); break;
    }
}
