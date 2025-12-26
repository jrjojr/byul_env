#ifndef MAZE_CORE_H
#define MAZE_CORE_H

#include "byul_config.h"
#include "navgrid.h"
#include "coord.h"
#include "coord_hash.h"

#ifdef __cplusplus
extern "C" {
#endif

// Maze structure definition
typedef struct s_maze {
    int x0;
    int y0;
    int width;
    int height;
    coord_hash_t* blocked;
} maze_t;

// Basic constructors / destructors
BYUL_API maze_t* maze_create();
BYUL_API maze_t* maze_create_full(int x0, int y0, int width, int height);

BYUL_API void maze_destroy(maze_t* maze);

BYUL_API void maze_clear(maze_t* maze);

// Copy and comparison
BYUL_API maze_t* maze_copy(const maze_t* maze);
BYUL_API bool maze_equal(const maze_t* a, const maze_t* b);
BYUL_API uint32_t maze_hash(const maze_t* maze);

// Origin set / get
BYUL_API void maze_set_origin(maze_t* maze, int x0, int y0);
BYUL_API void maze_get_origin(
    const maze_t* maze, int* out_x0, int* out_y0);

// Size getters
BYUL_API int maze_get_width(const maze_t* maze);
BYUL_API int maze_get_height(const maze_t* maze);

// Direct access to blocked coordinates (read-only)
BYUL_API const coord_hash_t* maze_get_blocked_coords(
    const maze_t* maze);

BYUL_API void maze_apply_to_navgrid(const maze_t* maze, navgrid_t* navgrid);

BYUL_API void maze_remove_from_navgrid(const maze_t* maze, navgrid_t* navgrid);

#ifdef __cplusplus
}
#endif

#endif // MAZE_CORE_H
