#include "internal/maze.h"
#include "internal/obstacle.h"
#include <vector>
#include <random>
#include <ctime>
#include <algorithm>

static const int WALL = 1;
static const int PASSAGE = 0;

struct Wall {
    int x1, y1, x2, y2, wx, wy;
};

struct Cell {
    int x, y;
};

static int find(int x, int y, std::vector<std::vector<int>>& parent) {
    if (parent[y][x] == y * parent[0].size() + x) return parent[y][x];
    int id = parent[y][x];
    int px = id % parent[0].size();
    int py = id / parent[0].size();
    return parent[y][x] = find(px, py, parent);
}

static void merge(int x1, int y1, int x2, int y2, 
    std::vector<std::vector<int>>& parent) {

    int p1 = find(x1, y1, parent);
    int p2 = find(x2, y2, parent);
    if (p1 != p2) parent[p2 / parent[0].size()][p2 % parent[0].size()] = p1;
}

void maze_make_kruskal(maze_t* maze) {
    if (!maze || maze->width < 3 || maze->height < 3) return;
    if (maze->width % 2 == 0 || maze->height % 2 == 0) return;

    int w = maze->width;
    int h = maze->height;
    int x0 = maze->x0;
    int y0 = maze->y0;

    std::vector<std::vector<int>> grid(h, std::vector<int>(w, WALL));
    std::vector<std::vector<int>> parent(h, std::vector<int>(w, 0));
    std::vector<Wall> walls;

    // 각 셀을 PASSAGE로 만들고 부모 초기화
    for (int y = 1; y < h; y += 2) {
        for (int x = 1; x < w; x += 2) {
            grid[y][x] = PASSAGE;
            parent[y][x] = y * w + x;

            if (x + 2 < w) walls.push_back({x, y, x + 2, y, x + 1, y});
            if (y + 2 < h) walls.push_back({x, y, x, y + 2, x, y + 1});
        }
    }

    std::mt19937 rng(static_cast<unsigned int>(time(nullptr)));
    std::shuffle(walls.begin(), walls.end(), rng);

    for (const Wall& wall : walls) {
        int p1 = find(wall.x1, wall.y1, parent);
        int p2 = find(wall.x2, wall.y2, parent);
        if (p1 != p2) {
            merge(wall.x1, wall.y1, wall.x2, wall.y2, parent);
            grid[wall.wy][wall.wx] = PASSAGE;
        }
    }

    // 벽을 blocked에 삽입
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
