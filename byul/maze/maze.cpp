#include <stdlib.h>
#include <string.h>

#include "internal/maze.h"
#include "internal/maze_room.h"

void maze_make(maze_t* maze, maze_type_t type) {
    if (!maze) return;

    switch (type) {
        case MAZE_TYPE_RECURSIVE:
            maze_make_recursive(maze); break;
        case MAZE_TYPE_PRIM:
            maze_make_prim(maze); break;
        case MAZE_TYPE_BINARY:
            maze_make_binary(maze); break;
        case MAZE_TYPE_ELLER:
            maze_make_eller(maze); break;
        case MAZE_TYPE_ALDOUS_BRODER:
            maze_make_aldous_broder(maze); break;
        case MAZE_TYPE_WILSON:
            maze_make_wilson(maze); break;
        case MAZE_TYPE_HUNT_AND_KILL:
            maze_make_hunt_and_kill(maze); break;
        case MAZE_TYPE_SIDEWINDER:
            maze_make_sidewinder(maze); break;
        case MAZE_TYPE_RECURSIVE_DIVISION:
            maze_make_recursive_division(maze); break;
        case MAZE_TYPE_KRUSKAL:
            maze_make_kruskal(maze); break;
        case MAZE_TYPE_ROOM_BLEND:
            maze_make_room_blend(maze); break;
        default:
            maze_make_kruskal(maze); break;
    }
}
