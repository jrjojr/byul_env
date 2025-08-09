#include <stdlib.h>
#include <string.h>

#include "maze.h"

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
