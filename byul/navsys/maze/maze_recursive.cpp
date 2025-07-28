#include "maze.h"
#include <stdlib.h>
#include <time.h>

// 내부 함수 선언
typedef struct {
    int x, y;
} cell_t;

static bool is_valid_cell(maze_t* maze, const int x, int y) {
    return x >= 0 && y >= 0 && x < maze->width && y < maze->height;
}

static void carve_passage(maze_t* maze, int cx, int cy, bool** visited) {
    static const int DX[4] = { 0, 0, -1, 1 };
    static const int DY[4] = { -1, 1, 0, 0 };
    int dirs[4] = { 0, 1, 2, 3 };

    // 방향을 섞는다
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
            coord_hash_remove(maze->blocked, 
                make_tmp_coord(maze->x0 + wall_x, maze->y0 + wall_y));            

            coord_hash_remove(maze->blocked, 
                make_tmp_coord(maze->x0 + nx, maze->y0 + ny));

            carve_passage(maze, nx, ny, visited);
        }
    }
}

void maze_make_recursive(maze_t* maze) {
    if (!maze) return;

    srand((unsigned int)time(NULL));

    int w = maze->width;
    int h = maze->height;

    // 모든 좌표를 먼저 막는다
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            coord_hash_insert(maze->blocked, 
                make_tmp_coord(maze->x0 + x, maze->y0 + y), NULL);
        }
    }

    // 방문 상태 배열
    bool** visited = (bool**)malloc(sizeof(bool*) * h);
    for (int i = 0; i < h; ++i) {
        visited[i] = (bool*)calloc(w, sizeof(bool));
    }

    carve_passage(maze, 1, 1, visited);

    for (int i = 0; i < h; ++i) {
        free(visited[i]);
    }
    free(visited);
}
