#ifndef MAZE_H
#define MAZE_H

#include "maze_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MAZE_TYPE_RECURSIVE,
    MAZE_TYPE_PRIM,
    MAZE_TYPE_BINARY,
    MAZE_TYPE_ELLER,

    MAZE_TYPE_ALDOUS_BRODER,
    MAZE_TYPE_WILSON,
    MAZE_TYPE_HUNT_AND_KILL,
    MAZE_TYPE_SIDEWINDER,
    
    MAZE_TYPE_RECURSIVE_DIVISION,
    MAZE_TYPE_KRUSKAL,
    MAZE_TYPE_ROOM_BLEND
} maze_type_t;

BYUL_API maze_t* maze_make(
    int x0, int y0, int width, int height, maze_type_t type);

#ifdef __cplusplus
}
#endif

#endif // MAZE_H
