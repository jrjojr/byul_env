#include "internal/maze.h"
#include "internal/obstacle.h"
#include <random>

void maze_make_binary(maze_t* maze) {
    if (!maze || maze->width < 3 || maze->height < 3) return;
    if (maze->width % 2 == 0 || maze->height % 2 == 0) return;

    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> dist(0, 1);

    int x0 = maze->x0;
    int y0 = maze->y0;
    int w = maze->width;
    int h = maze->height;

    // 초기화: 전체 영역을 벽으로 설정
    coord_hash_clear(maze->blocked);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            coord_hash_insert(maze->blocked, 
                make_tmp_coord(x0 + x, y0 + y), NULL);
        }
    }

    // 각 셀을 기준으로 남쪽 또는 동쪽을 랜덤으로 뚫음
    for (int y = 1; y < h; y += 2) {
        for (int x = 1; x < w; x += 2) {
            coord_hash_remove(maze->blocked, make_tmp_coord(x0 + x, y0 + y));

            bool can_east = (x + 2 < w);
            bool can_south = (y + 2 < h);

            if (can_east && can_south) {
                if (dist(rng) == 0) {
                    coord_hash_remove(maze->blocked, 
                        make_tmp_coord(x0 + x + 1, y0 + y));
                } else {
                    coord_hash_remove(maze->blocked, 
                        make_tmp_coord(x0 + x, y0 + y + 1));
                }
            } else if (can_east) {
                coord_hash_remove(maze->blocked, 
                    make_tmp_coord(x0 + x + 1, y0 + y));
            } else if (can_south) {
                coord_hash_remove(maze->blocked, 
                    make_tmp_coord(x0 + x, y0 + y + 1));
            }
        }
    }
}
