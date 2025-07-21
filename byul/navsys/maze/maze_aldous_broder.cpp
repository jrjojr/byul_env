#include "internal/maze.h"
#include "internal/obstacle.h"
#include <vector>
#include <random>
#include <ctime>
#include <algorithm>

static const int WALL = 1;
static const int PASSAGE = 0;

struct Cell {
    int x, y;
    Cell(int _x, int _y) : x(_x), y(_y) {}
};

static bool is_inside(int x, int y, int w, int h) {
    return x > 0 && y > 0 && x < w - 1 && y < h - 1 && x % 2 == 1 && y % 2 == 1;
}

void maze_make_aldous_broder(maze_t* maze) {
    if (!maze || maze->width < 3 || maze->height < 3) return;
    if (maze->width % 2 == 0 || maze->height % 2 == 0) return;

    int w = maze->width;
    int h = maze->height;
    int x0 = maze->x0;
    int y0 = maze->y0;

    std::vector<std::vector<int>> grid(h, std::vector<int>(w, WALL));
    std::vector<std::vector<bool>> visited(h, std::vector<bool>(w, false));
    std::mt19937 rng(static_cast<unsigned int>(time(nullptr)));

    std::vector<Cell> candidates;
    for (int y = 1; y < h; y += 2) {
        for (int x = 1; x < w; x += 2) {
            candidates.emplace_back(x, y);
        }
    }

    std::uniform_int_distribution<size_t> pick_start(0, candidates.size() - 1);
    Cell current = candidates[pick_start(rng)];
    visited[current.y][current.x] = true;
    grid[current.y][current.x] = PASSAGE;

    int visited_count = 1;
    const int total_cells = candidates.size();

    const int dx[4] = { 0, 0, -2, 2 };
    const int dy[4] = { -2, 2, 0, 0 };

    while (visited_count < total_cells) {
        std::vector<int> dirs = { 0, 1, 2, 3 };
        std::shuffle(dirs.begin(), dirs.end(), rng);

        for (int i = 0; i < 4; ++i) {
            int dir = dirs[i];
            int nx = current.x + dx[dir];
            int ny = current.y + dy[dir];

            if (!is_inside(nx, ny, w, h)) continue;

            if (!visited[ny][nx]) {
                // 뚫기: 중간 벽 + 다음 셀
                int mx = (current.x + nx) / 2;
                int my = (current.y + ny) / 2;
                grid[my][mx] = PASSAGE;
                grid[ny][nx] = PASSAGE;
                visited[ny][nx] = true;
                visited_count++;
            }

            // 이동 (항상)
            current.x = nx;
            current.y = ny;
            break;
        }
    }

    // 벽 삽입
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (grid[y][x] != PASSAGE) {
                coord_hash_insert(
                    maze->blocked,
                    make_tmp_coord(x + x0, y + y0),
                    nullptr
                );
            }
        }
    }
}
