#include "maze_eller.h"
#include "obstacle.h"
#include <vector>
#include <map>
#include <random>
#include <ctime>

static const int WALL = 1;
static const int PASSAGE = 0;

maze_t* maze_make_eller(int x0, int y0, int width, int height)
{
    if (width < 3 || height < 3) return nullptr;
    if (width % 2 == 0 || height % 2 == 0) return nullptr;

    maze_t* maze = maze_create_full(x0, y0, width, height);

    int w = maze->width;
    int h = maze->height;

    std::vector<std::vector<int>> set_id(h, std::vector<int>(w, 0));
    std::vector<std::vector<int>> grid(h, std::vector<int>(w, WALL));

    std::mt19937 rng(static_cast<unsigned int>(time(nullptr)));
    std::uniform_int_distribution<int> coin(0, 1);

    int next_set = 1;

    for (int x = 1; x < w; x += 2) {
        set_id[0][x] = next_set++;
        grid[0][x] = PASSAGE;
    }

    for (int y = 0; y < h; y += 2) {
        if (y > 0) {

            for (int x = 1; x < w; x += 2) {
                if (set_id[y][x] == 0) {
                    set_id[y][x] = next_set++;
                }
                grid[y][x] = PASSAGE;
            }
        }


        for (int x = 1; x < w - 2; x += 2) {
            if (set_id[y][x] != set_id[y][x + 2] && coin(rng)) {
                int from = set_id[y][x + 2];
                int to = set_id[y][x];


                for (int i = 1; i < w; i += 2) {
                    if (set_id[y][i] == from) set_id[y][i] = to;
                }

                set_id[y][x + 1] = to;
                grid[y][x + 1] = PASSAGE;
            }
        }

        if (y + 2 >= h) break;

        std::map<int, std::vector<int>> sets;
        for (int x = 1; x < w; x += 2) {
            sets[set_id[y][x]].push_back(x);
        }

        for (auto& [sid, xs] : sets) {
            std::vector<int> down_cells;

            for (int x : xs) {
                if (coin(rng)) {
                    set_id[y + 2][x] = sid;
                    set_id[y + 1][x] = sid;
                    grid[y + 1][x] = PASSAGE;
                    grid[y + 2][x] = PASSAGE;
                    down_cells.push_back(x);
                }
            }

            if (down_cells.empty()) {
                int x = xs[rng() % xs.size()];
                set_id[y + 2][x] = sid;
                set_id[y + 1][x] = sid;
                grid[y + 1][x] = PASSAGE;
                grid[y + 2][x] = PASSAGE;
            }
        }
    }

    int y = h - 1;
    for (int x = 1; x < w - 2; x += 2) {
        if (set_id[y][x] != set_id[y][x + 2]) {
            int from = set_id[y][x + 2];
            int to = set_id[y][x];
            for (int i = 1; i < w; i += 2) {
                if (set_id[y][i] == from) set_id[y][i] = to;
            }
            set_id[y][x + 1] = to;
            grid[y][x + 1] = PASSAGE;
        }
    }

    for (int yy = 0; yy < h; ++yy) {
        for (int xx = 0; xx < w; ++xx) {
            if (grid[yy][xx] != PASSAGE) {
                coord_t tmp = {x0 + xx, y0 + yy};
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
