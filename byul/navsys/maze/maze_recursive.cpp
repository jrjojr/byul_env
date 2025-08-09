#include "maze_recursive.h"
#include "coord.h"
#include <stdlib.h>
#include <time.h>

static bool is_valid_cell(maze_t* maze, const int x, int y) {
    return x >= 0 && y >= 0 && x < maze->width && y < maze->height;
}

static void carve_passage(maze_t* maze, int cx, int cy, bool** visited) {
    static const int DX[4] = { 0, 0, -1, 1 };
    static const int DY[4] = { -1, 1, 0, 0 };
    int dirs[4] = { 0, 1, 2, 3 };

    for (int i = 3; i > 0; --i) {
        int j = rand() % (i + 1);
        int tmp = dirs[i];
        dirs[i] = dirs[j];
        dirs[j] = tmp;
    }

    visited[cy][cx] = true;

    for (int i = 0; i < 4; ++i) {
        int dir = dirs[i];
        int nx = cx + DX[dir] * 2;
        int ny = cy + DY[dir] * 2;

        if (is_valid_cell(maze, nx, ny) && !visited[ny][nx]) {
            int wall_x = cx + DX[dir];
            int wall_y = cy + DY[dir];

            coord_t tmp_wall = {maze->x0 + wall_x, maze->y0 + wall_y};
            coord_hash_remove(maze->blocked, 
                &tmp_wall);

            coord_t tmp_n = {maze->x0 + nx, maze->y0 + ny};
            coord_hash_remove(maze->blocked, 
                &tmp_n);

            carve_passage(maze, nx, ny, visited);
        }
    }
}

maze_t* maze_make_recursive(int x0, int y0, int width, int height) {

    srand((unsigned int)time(NULL));

    maze_t* out = nullptr;
    out = maze_create_full(x0, y0, width, height);

    int w = width;
    int h = height;

    for (int y = y0; y < h; ++y) {
        for (int x = x0; x < w; ++x) {
            coord_t tmp = {out->x0 + x, out->y0 + y};
            coord_hash_insert(out->blocked, 
                &tmp, NULL);
        }
    }

    bool** visited = (bool**)malloc(sizeof(bool*) * h);
    for (int i = 0; i < h; ++i) {
        visited[i] = (bool*)calloc(w, sizeof(bool));
    }

    carve_passage(out, 1, 1, visited);

    for (int i = 0; i < h; ++i) {
        free(visited[i]);
    }
    free(visited);

    return out;
}
