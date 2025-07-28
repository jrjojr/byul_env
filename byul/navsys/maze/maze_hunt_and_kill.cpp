#include "maze.h"
#include "obstacle.h"
#include <vector>
#include <random>
#include <ctime>
#include <algorithm>

static const int WALL = 1;
static const int PASSAGE = 0;

static bool is_inside(int x, int y, int w, int h) {
    return x > 0 && y > 0 && x < w - 1 && y < h - 1 && x % 2 == 1 && y % 2 == 1;
}

void maze_make_hunt_and_kill(maze_t* maze) {
    if (!maze || maze->width < 3 || maze->height < 3) return;
    if (maze->width % 2 == 0 || maze->height % 2 == 0) return;

    int w = maze->width;
    int h = maze->height;
    int x0 = maze->x0;
    int y0 = maze->y0;

    std::vector<std::vector<int>> grid(h, std::vector<int>(w, WALL));
    std::vector<std::vector<bool>> visited(h, std::vector<bool>(w, false));

    std::mt19937 rng(static_cast<unsigned int>(time(nullptr)));
    if ((w - 1) / 2 <= 0 || (h - 1) / 2 <= 0) return;
    std::uniform_int_distribution<int> dist_x(0, (w - 1) / 2 - 1);
    std::uniform_int_distribution<int> dist_y(0, (h - 1) / 2 - 1);

    // 시작 셀 선택
    int cx = dist_x(rng) * 2 + 1;
    int cy = dist_y(rng) * 2 + 1;
    grid[cy][cx] = PASSAGE;
    visited[cy][cx] = true;

    const int dx[4] = { 0, 0, -2, 2 };
    const int dy[4] = { -2, 2, 0, 0 };

    while (true) {
        // --- Kill Phase ---
        bool moved = false;
        std::vector<int> dirs = { 0, 1, 2, 3 };
        std::shuffle(dirs.begin(), dirs.end(), rng);

        for (int d : dirs) {
            int nx = cx + dx[d];
            int ny = cy + dy[d];

            if (is_inside(nx, ny, w, h) && !visited[ny][nx]) {
                int mx = (cx + nx) / 2;
                int my = (cy + ny) / 2;

                grid[my][mx] = PASSAGE;
                grid[ny][nx] = PASSAGE;
                visited[ny][nx] = true;

                cx = nx;
                cy = ny;
                moved = true;
                break;
            }
        }

        if (moved) continue;

        // --- Hunt Phase ---
        bool found = false;

        for (int y = 1; y < h; y += 2) {
            for (int x = 1; x < w; x += 2) {
                if (visited[y][x]) continue;

                std::vector<int> adj;
                for (int d = 0; d < 4; ++d) {
                    int nx = x + dx[d];
                    int ny = y + dy[d];
                    if (is_inside(nx, ny, w, h) && visited[ny][nx]) {
                        adj.push_back(d);
                    }
                }

                if (!adj.empty()) {
                    int d = adj[rng() % adj.size()];
                    int mx = x + dx[d] / 2;
                    int my = y + dy[d] / 2;

                    grid[my][mx] = PASSAGE;
                    grid[y][x] = PASSAGE;
                    visited[y][x] = true;

                    cx = x;
                    cy = y;
                    found = true;
                    break;
                }
            }
            if (found) break;
        }

        if (!found) break; // 종료 조건
    }

    // 벽을 blocked에 삽입
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
