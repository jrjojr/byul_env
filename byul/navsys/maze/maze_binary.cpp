#include "maze_binary.h"

#include <random>

maze_t* maze_make_binary(int x0, int y0, int width, int height) {
    if (width < 3 || height < 3) return nullptr;
    if (width % 2 == 0 || height % 2 == 0) return nullptr;

    maze_t* maze = maze_create_full(x0, y0, width, height);
    if(!maze) return nullptr;


    std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> dist(0, 1);

    int w = maze->width;
    int h = maze->height;

    coord_hash_clear(maze->blocked);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            coord_t tmp = {x0 + x, y0 + y};
            coord_hash_insert(maze->blocked, 
                &tmp, NULL);
        }
    }

    // Randomly open either south or east for each cell
    for (int y = 1; y < h; y += 2) {
        for (int x = 1; x < w; x += 2) {
            coord_t tmp = {x0 + x, y0 + y};
            coord_hash_remove(maze->blocked, &tmp);

            bool can_east = (x + 2 < w);
            bool can_south = (y + 2 < h);

            if (can_east && can_south) {
                if (dist(rng) == 0) {
                    coord_t tmp = {x0 + x + 1, y0 + y};
                    coord_hash_remove(maze->blocked, 
                        &tmp);
                } else {
                    coord_t tmp = {x0 + x, y0 + y + 1};
                    coord_hash_remove(maze->blocked, 
                        &tmp);
                }
            } else if (can_east) {
                coord_t tmp = {x0 + x + 1, y0 + y};
                coord_hash_remove(maze->blocked, 
                    &tmp);
            } else if (can_south) {
                coord_t tmp = {x0 + x, y0 + y + 1};
                coord_hash_remove(maze->blocked, 
                    &tmp);
            }
        }
    }
    return maze;
}
