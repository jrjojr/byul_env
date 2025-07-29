#include "maze.h"
#include "maze_room.h"
#include <vector>
#include <cstdlib>
#include <ctime>
#include <stack>
#include <algorithm>
#include <random>

static const int WALL = 1;
static const int PASSAGE = 0;

struct Room {
    int x, y, w, h;
    Room(int _x, int _y, int _w, int _h)
        : x(_x), y(_y), w(_w), h(_h) {}

    int cx() const { return x + w / 2; }
    int cy() const { return y + h / 2; }
};

static void dig_room(std::vector<std::vector<int>>& grid, const Room& r) {
    for (int dy = 0; dy < r.h; ++dy) {
        for (int dx = 0; dx < r.w; ++dx) {
            grid[r.y + dy][r.x + dx] = PASSAGE;
        }
    }
}

static void dig_corridor(std::vector<std::vector<int>>& grid, int x1, int y1, int x2, int y2) {
    if (rand() % 2) {
        for (int x = std::min(x1, x2); x <= std::max(x1, x2); ++x)
            grid[y1][x] = PASSAGE;
        for (int y = std::min(y1, y2); y <= std::max(y1, y2); ++y)
            grid[y][x2] = PASSAGE;
    } else {
        for (int y = std::min(y1, y2); y <= std::max(y1, y2); ++y)
            grid[y][x1] = PASSAGE;
        for (int x = std::min(x1, x2); x <= std::max(x1, x2); ++x)
            grid[y2][x] = PASSAGE;
    }
}

static void fill_with_maze(std::vector<std::vector<int>>& grid, int w, int h) {
    std::vector<std::vector<bool>> visited(h, std::vector<bool>(w, false));
    const int dx[4] = { 0, 0, -2, 2 };
    const int dy[4] = { -2, 2, 0, 0 };

    auto is_inside = [&](int x, int y) {
        return x > 0 && y > 0 && x < w - 1 && y < h - 1 && x % 2 == 1 && y % 2 == 1;
    };

    for (int y = 1; y < h; y += 2) {
        for (int x = 1; x < w; x += 2) {
            if (grid[y][x] == PASSAGE || visited[y][x]) continue;

            std::stack<std::pair<int, int>> stk;
            stk.push({x, y});
            visited[y][x] = true;
            grid[y][x] = PASSAGE;

            while (!stk.empty()) {
                auto [cx, cy] = stk.top();

                static thread_local std::mt19937 rng(std::random_device{}());
                std::vector<int> dirs = {0, 1, 2, 3};
                std::shuffle(dirs.begin(), dirs.end(), rng);
                bool moved = false;

                for (int d : dirs) {
                    int nx = cx + dx[d];
                    int ny = cy + dy[d];
                    if (is_inside(nx, ny) && !visited[ny][nx] && grid[ny][nx] == WALL) {
                        grid[(cy + ny) / 2][(cx + nx) / 2] = PASSAGE;
                        grid[ny][nx] = PASSAGE;
                        visited[ny][nx] = true;
                        stk.push({nx, ny});
                        moved = true;
                        break;
                    }
                }

                if (!moved) stk.pop();
            }
        }
    }
}

void maze_make_room_blend(maze_t* maze) {
    if (!maze || maze->width < 9 || maze->height < 9) return;

    int w = maze->width;
    int h = maze->height;
    int x0 = maze->x0;
    int y0 = maze->y0;

    std::vector<std::vector<int>> grid(h, std::vector<int>(w, WALL));
    std::vector<Room> rooms;

    srand((unsigned)time(NULL));

    const int room_attempts = 30;
    const int room_min = 3, room_max = 7;

    for (int i = 0; i < room_attempts; ++i) {
        int rw = room_min + rand() % ((room_max - room_min) / 2 + 1) * 2 + 1;
        int rh = room_min + rand() % ((room_max - room_min) / 2 + 1) * 2 + 1;
        int rx = (rand() % ((w - rw - 1) / 2)) * 2 + 1;
        int ry = (rand() % ((h - rh - 1) / 2)) * 2 + 1;

        Room r(rx, ry, rw, rh);

        bool overlapped = false;
        for (const Room& other : rooms) {
            if (rx < other.x + other.w && rx + rw > other.x &&
                ry < other.y + other.h && ry + rh > other.y) {
                overlapped = true;
                break;
            }
        }
        if (!overlapped) {
            dig_room(grid, r);
            rooms.push_back(r);
        }
    }

    for (size_t i = 1; i < rooms.size(); ++i) {
        dig_corridor(grid, rooms[i - 1].cx(), rooms[i - 1].cy(),
                           rooms[i].cx(),     rooms[i].cy());
    }

    fill_with_maze(grid, w, h);

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (grid[y][x] == WALL) {
                coord_hash_insert(maze->blocked,
                    make_tmp_coord(x + x0, y + y0), nullptr);
            }
        }
    }

    // maze->type = MAZE_TYPE_ROOM_BLEND;
}
