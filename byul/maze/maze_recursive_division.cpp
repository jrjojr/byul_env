#include "internal/maze.h"
#include "internal/obstacle.h"
#include <vector>
#include <random>
#include <ctime>
#include <algorithm>

static const int WALL = 1;
static const int PASSAGE = 0;

static int random_even(int min, int max, std::mt19937& rng) {
    std::vector<int> evens;
    for (int i = min; i <= max; i += 2) evens.push_back(i);
    std::shuffle(evens.begin(), evens.end(), rng);
    return evens.empty() ? min : evens[0];
}

static int random_odd(int min, int max, std::mt19937& rng) {
    std::vector<int> odds;
    for (int i = min; i <= max; i += 2) odds.push_back(i);
    std::shuffle(odds.begin(), odds.end(), rng);
    return odds.empty() ? min : odds[0];
}

static void divide(std::vector<std::vector<int>>& grid,
                   int x, int y, int w, int h,
                   std::mt19937& rng) {
    if (w < 5 || h < 5) return;

    bool horizontal = (w < h) ? true : (w > h) ? false : (rng() % 2 == 0);

    if (horizontal) {
        int wall_y = random_even(y + 2, y + h - 3, rng);  // 내부 짝수 y
        int passage_x = random_odd(x + 1, x + w - 2, rng); // 내부 홀수 x

        for (int i = x; i < x + w; ++i)
            grid[wall_y][i] = WALL;

        grid[wall_y][passage_x] = PASSAGE;

        divide(grid, x, y, w, wall_y - y, rng);
        divide(grid, x, wall_y + 1, w, y + h - wall_y - 1, rng);
    } else {
        int wall_x = random_even(x + 2, x + w - 3, rng); // 내부 짝수 x
        int passage_y = random_odd(y + 1, y + h - 2, rng); // 내부 홀수 y

        for (int i = y; i < y + h; ++i)
            grid[i][wall_x] = WALL;

        grid[passage_y][wall_x] = PASSAGE;

        divide(grid, x, y, wall_x - x, h, rng);
        divide(grid, wall_x + 1, y, x + w - wall_x - 1, h, rng);
    }
}

void maze_make_recursive_division(maze_t* maze) {
    if (!maze || maze->width < 3 || maze->height < 3) return;
    if (maze->width % 2 == 0 || maze->height % 2 == 0) return;

    int w = maze->width;
    int h = maze->height;
    int x0 = maze->x0;
    int y0 = maze->y0;

    std::vector<std::vector<int>> grid(h, std::vector<int>(w, WALL));
    std::mt19937 rng(static_cast<unsigned int>(time(nullptr)));

    // 홀수 셀만 통로로 초기화
    for (int y = 1; y < h; y += 2)
        for (int x = 1; x < w; x += 2)
            grid[y][x] = PASSAGE;

    divide(grid, 0, 0, w, h, rng);

    // 외곽 벽 강제 고정
    for (int y = 0; y < h; ++y) {
        grid[y][0] = WALL;
        grid[y][w - 1] = WALL;
    }
    for (int x = 0; x < w; ++x) {
        grid[0][x] = WALL;
        grid[h - 1][x] = WALL;
    }

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (grid[y][x] == WALL) {
                coord_hash_insert(
                    maze->blocked,
                    make_tmp_coord(x + x0, y + y0),
                    nullptr
                );
            }
        }
    }
}
