#include "maze_sidewinder.h"
#include "obstacle.h"
#include <vector>
#include <random>
#include <ctime>

static const int WALL = 1;
static const int PASSAGE = 0;

static bool is_inside(int x, int y, int w, int h) {
    return x > 0 && y > 0 && x < w - 1 && y < h - 1 && x % 2 == 1 && y % 2 == 1;
}

maze_t* maze_make_sidewinder(int x0, int y0, int width, int height) {
    if (width < 3 || height < 3) return nullptr;
    if (width % 2 == 0 || height % 2 == 0) return nullptr;

    maze_t* maze = maze_create_full(x0, y0, width, height);

    int w = maze->width;
    int h = maze->height;

    std::vector<std::vector<int>> grid(h, std::vector<int>(w, WALL));
    std::mt19937 rng(static_cast<unsigned int>(time(nullptr)));

    for (int y = 1; y < h; y += 2) {
        std::vector<int> run;

        for (int x = 1; x < w; x += 2) {
            grid[y][x] = PASSAGE;
            run.push_back(x);

            bool at_east_edge = (x + 2 >= w);
            bool at_north_edge = (y - 2 < 0);

            bool carve_east = !at_east_edge && (rng() % 2 == 0);

            if (carve_east) {
                grid[y][x + 1] = PASSAGE;
            } else {
                if (!at_north_edge) {
                    int pick = run[rng() % run.size()];
                    grid[y - 1][pick] = PASSAGE;
                }
                run.clear();
            }
        }
    }

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (grid[y][x] != PASSAGE) {
                coord_t tmp = {x + x0, y + y0};
                coord_hash_insert(
                    maze->blocked,
                    &tmp,
                    nullptr
                );
            }
        }
    }
    return maze;
}
