#include "internal/maze.h"
#include "internal/maze_common.h"
#include <vector>
#include <map>
#include <random>
#include <ctime>

static const int WALL = 1;
static const int PASSAGE = 0;

void maze_generate_eller(maze_t* maze) {
    if (!maze || maze->width < 3 || maze->height < 3) return;
    if (maze->width % 2 == 0 || maze->height % 2 == 0) return;

    int w = maze->width;
    int h = maze->height;
    int x0 = maze->x0;
    int y0 = maze->y0;

    std::vector<std::vector<int>> set_id(h, std::vector<int>(w, 0));
    std::vector<std::vector<int>> grid(h, std::vector<int>(w, WALL));

    std::mt19937 rng(static_cast<unsigned int>(time(nullptr)));
    std::uniform_int_distribution<int> coin(0, 1);

    int next_set = 1;

    // 첫 줄 초기화
    for (int x = 1; x < w; x += 2) {
        set_id[0][x] = next_set++;
        grid[0][x] = PASSAGE;
    }

    for (int y = 0; y < h; y += 2) {
        if (y > 0) {
            // 새 줄의 셋 ID 설정 (아래로 연결된 경우 유지)
            for (int x = 1; x < w; x += 2) {
                if (set_id[y][x] == 0) {
                    set_id[y][x] = next_set++;
                }
                grid[y][x] = PASSAGE;
            }
        }

        // 수평 연결
        for (int x = 1; x < w - 2; x += 2) {
            if (set_id[y][x] != set_id[y][x + 2] && coin(rng)) {
                int from = set_id[y][x + 2];
                int to = set_id[y][x];

                // 병합
                for (int i = 1; i < w; i += 2) {
                    if (set_id[y][i] == from) set_id[y][i] = to;
                }

                set_id[y][x + 1] = to;
                grid[y][x + 1] = PASSAGE;
            }
        }

        if (y + 2 >= h) break;

        // 수직 연결
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

            // 최소 1개는 수직 연결
            if (down_cells.empty()) {
                int x = xs[rng() % xs.size()];
                set_id[y + 2][x] = sid;
                set_id[y + 1][x] = sid;
                grid[y + 1][x] = PASSAGE;
                grid[y + 2][x] = PASSAGE;
            }
        }
    }

    // 마지막 줄 병합
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

    // 벽만 blocked에 기록
    for (int yy = 0; yy < h; ++yy) {
        for (int xx = 0; xx < w; ++xx) {
            if (grid[yy][xx] != PASSAGE) {
                coord_hash_insert(
                    maze->blocked,
                    make_tmp_coord(x0 + xx, y0 + yy),
                    nullptr
                );
            }
        }
    }
}
