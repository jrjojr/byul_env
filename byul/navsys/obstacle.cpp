#include "internal/obstacle.h"
#include <stdlib.h>
#include <string.h>
#include <time.h>

obstacle_t* obstacle_make_rect_all_blocked(int x0, int y0, int width, int height) {
    if (width <= 0 || height <= 0) return nullptr;

    obstacle_t* obstacle = obstacle_create_full(x0, y0, width, height);
    if (!obstacle) return nullptr;

    for (int dy = 0; dy < height; ++dy) {
        for (int dx = 0; dx < width; ++dx) {
            coord_t c = { x0 + dx, y0 + dy };
            coord_hash_insert(obstacle->blocked, &c, nullptr);
        }
    }
    return obstacle;
}

obstacle_t* obstacle_make_rect_random_blocked(
    int x0, int y0, int width, int height, float ratio) {

    if (width <= 0 || height <= 0 || ratio <= 0.0f) return nullptr;
    if (ratio > 1.0f) ratio = 1.0f;

    obstacle_t* obstacle = obstacle_create_full(x0, y0, width, height);
    if (!obstacle) return nullptr;

    srand((unsigned int)time(nullptr));

    for (int dy = 0; dy < height; ++dy) {
        for (int dx = 0; dx < width; ++dx) {
            if ((float)rand() / RAND_MAX <= ratio) {
                coord_t c = { x0 + dx, y0 + dy };
                coord_hash_insert(obstacle->blocked, &c, nullptr);
            }
        }
    }
    return obstacle;
}

obstacle_t* obstacle_make_beam(
    const coord_t* start, const coord_t* goal, int range){

    if (!start || !goal) return nullptr;

    int width = goal->x - start->x;
    int height = goal->y - start->y;
    obstacle_t* obstacle = obstacle_create_full(
        start->x, start->y, width, height);

    coord_t* cur = coord_copy(start);

    if(range <= 0){
        while(!coord_equal(cur, goal)){
            coord_t* next = coord_clone_next_to_goal(cur, goal);

            if (!obstacle_is_coord_blocked(obstacle, next->x, next->y)) {
                obstacle_block_coord(obstacle, next->x, next->y);
            }
            coord_set(cur, next->x, next->y);
            coord_destroy(next);
        }
    }

    // while(cur != goal){
    while(!coord_equal(cur, goal)){
        coord_t* next = coord_clone_next_to_goal(cur, goal);

        coord_list_t* neighbors = obstacle_clone_neighbors_all_range(
            obstacle, next->x, next->y, range-1);
        for(int i=0; i < coord_list_length(neighbors); i++){
            const coord_t* c = coord_list_get(neighbors, i);

            if (!obstacle_is_coord_blocked(obstacle, c->x, c->y)){
                obstacle_block_coord(obstacle, c->x, c->y);
            }
        }
        coord_set(cur, next->x, next->y);
        coord_list_destroy(neighbors);
        coord_destroy(next);
    }
    coord_destroy(cur);
    return obstacle;        
}

obstacle_t* obstacle_make_torus(
    const coord_t* start, const coord_t* goal, int thickness)
{
    if (!start || !goal || thickness <= 0) return NULL;

    int min_x = (start->x < goal->x) ? start->x : goal->x;
    int max_x = (start->x > goal->x) ? start->x : goal->x;
    int min_y = (start->y < goal->y) ? start->y : goal->y;
    int max_y = (start->y > goal->y) ? start->y : goal->y;

    int width = max_x - min_x + 1;
    int height = max_y - min_y + 1;

    if (width <= thickness * 2 || height <= thickness * 2)
        return NULL; // 내부 구멍이 생기지 않음

    obstacle_t* obs = obstacle_create_full(min_x, min_y, width, height);

    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            bool outer =
                x < thickness || x >= width - thickness ||
                y < thickness || y >= height - thickness;

            bool inner =
                x >= thickness && x < width - thickness &&
                y >= thickness && y < height - thickness;

            if (outer && !inner) {
                obstacle_block_coord(obs, min_x + x, min_y + y);
            }
        }
    }

    return obs;
}

obstacle_t* obstacle_make_enclosure(
    const coord_t* start, const coord_t* goal, int thickness,
    enclosure_open_dir_t open)
{
    if (!start || !goal || thickness <= 0) return NULL;

    int min_x = (start->x < goal->x) ? start->x : goal->x;
    int max_x = (start->x > goal->x) ? start->x : goal->x;
    int min_y = (start->y < goal->y) ? start->y : goal->y;
    int max_y = (start->y > goal->y) ? start->y : goal->y;

    int width = max_x - min_x + 1;
    int height = max_y - min_y + 1;

    obstacle_t* obs = obstacle_create_full(min_x, min_y, width, height);

    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            bool block = false;

            // 테두리 여부
            bool is_top = (y < thickness);
            bool is_bottom = (y >= height - thickness);
            bool is_left = (x < thickness);
            bool is_right = (x >= width - thickness);

            // 기본적으로 테두리는 막되, 열린 방향은 열어둔다
            if (is_top && open != ENCLOSURE_OPEN_UP)
                block = true;
            else if (is_bottom && open != ENCLOSURE_OPEN_DOWN)
                block = true;
            else if (is_left && open != ENCLOSURE_OPEN_LEFT)
                block = true;
            else if (is_right && open != ENCLOSURE_OPEN_RIGHT)
                block = true;

            if (block)
                obstacle_block_coord(obs, min_x + x, min_y + y);
        }
    }

    return obs;
}

obstacle_t* obstacle_make_cross(
    const coord_t* center, int length, int range)
{
    if (!center || length < 0 || range < 0)
        return NULL;

    int min_x = center->x - length - range;
    int max_x = center->x + length + range;
    int min_y = center->y - length - range;
    int max_y = center->y + length + range;

    int width = max_x - min_x + 1;
    int height = max_y - min_y + 1;

    obstacle_t* obs = obstacle_create_full(min_x, min_y, width, height);
    if (!obs) return NULL;

    // 중심점만 block하는 경우
    if (length == 0 && range == 0) {
        obstacle_block_coord(obs, center->x, center->y);
        return obs;
    }

    // 중심점 포함, 상하좌우로 뻗는 십자형
    for (int d = -length; d <= length; ++d) {
        // 수직 방향 (↑↓)
        for (int r = -range; r <= range; ++r) {
            obstacle_block_coord(obs, center->x + r, center->y + d);
        }
        // 수평 방향 (←→)
        for (int r = -range; r <= range; ++r) {
            obstacle_block_coord(obs, center->x + d, center->y + r);
        }
    }

    return obs;
}

obstacle_t* obstacle_make_spiral(
    const coord_t* center,
    int radius,
    int turns,
    int range,
    int gap,
    spiral_dir_t direction)
{
    if (!center || radius <= 0 || turns <= 0 || range < 0 || gap < 0)
        return NULL;

    int min_x = center->x - radius;
    int max_x = center->x + radius;
    int min_y = center->y - radius;
    int max_y = center->y + radius;

    int width = max_x - min_x + 1;
    int height = max_y - min_y + 1;

    obstacle_t* obs = obstacle_create_full(min_x, min_y, width, height);
    if (!obs) return NULL;

    // 방향 설정
    const int dx_cw[4]  = { 1, 0, -1,  0}; // → ↓ ← ↑
    const int dy_cw[4]  = { 0, 1,  0, -1};

    const int dx_ccw[4] = { 0, 1,  0, -1}; // ↓ → ↑ ←
    const int dy_ccw[4] = { 1, 0, -1,  0};

    const int* dx = (direction == SPIRAL_COUNTER_CLOCKWISE) ? dx_ccw : dx_cw;
    const int* dy = (direction == SPIRAL_COUNTER_CLOCKWISE) ? dy_ccw : dy_cw;

    int cx = center->x;
    int cy = center->y;

    int len = 1;           // 현재 방향으로 이동할 길이
    int dir = 0;           // 0~3 (방향 인덱스)
    int step = 0;
    int max_steps = turns * 4;

    // 중심점부터 블로킹
    if (range == 0)
        obstacle_block_coord(obs, cx, cy);
    else
        obstacle_block_range(obs, cx, cy, range);

    while (step < max_steps) {
        // 현재 step이 gap 조건에 해당하면 블로킹 생략
        bool active_turn = (gap == 0 || step % (gap + 1) == 0);

        for (int i = 0; i < len; ++i) {
            cx += dx[dir];
            cy += dy[dir];

            if (!active_turn)
                continue;

            if (range == 0)
                obstacle_block_coord(obs, cx, cy);
            else
                obstacle_block_range(obs, cx, cy, range);
        }

        dir = (dir + 1) % 4;
        step++;

        if (step % 2 == 0)
            len++;
    }

    return obs;
}

static bool is_point_in_triangle(int px, int py, const coord_t* a, const coord_t* b, const coord_t* c) {
    int ax = a->x, ay = a->y;
    int bx = b->x, by = b->y;
    int cx = c->x, cy = c->y;

    int v0x = cx - ax;
    int v0y = cy - ay;
    int v1x = bx - ax;
    int v1y = by - ay;
    int v2x = px - ax;
    int v2y = py - ay;

    int dot00 = v0x * v0x + v0y * v0y;
    int dot01 = v0x * v1x + v0y * v1y;
    int dot02 = v0x * v2x + v0y * v2y;
    int dot11 = v1x * v1x + v1y * v1y;
    int dot12 = v1x * v2x + v1y * v2y;

    int denom = dot00 * dot11 - dot01 * dot01;
    if (denom == 0) return false;

    float u = (float)(dot11 * dot02 - dot01 * dot12) / denom;
    float v = (float)(dot00 * dot12 - dot01 * dot02) / denom;

    return (u >= 0 && v >= 0 && u + v <= 1.0f);
}

obstacle_t* obstacle_make_triangle(
    const coord_t* a, const coord_t* b, const coord_t* c)
{
    if (!a || !b || !c) return NULL;

    int min_x = a->x;
    int max_x = a->x;
    int min_y = a->y;
    int max_y = a->y;

    // 전체 경계 계산
    const coord_t* pts[3] = {a, b, c};
    for (int i = 1; i < 3; ++i) {
        if (pts[i]->x < min_x) min_x = pts[i]->x;
        if (pts[i]->x > max_x) max_x = pts[i]->x;
        if (pts[i]->y < min_y) min_y = pts[i]->y;
        if (pts[i]->y > max_y) max_y = pts[i]->y;
    }

    int width = max_x - min_x + 1;
    int height = max_y - min_y + 1;

    obstacle_t* obs = obstacle_create_full(min_x, min_y, width, height);
    if (!obs) return NULL;

    for (int x = min_x; x <= max_x; ++x) {
        for (int y = min_y; y <= max_y; ++y) {
            if (is_point_in_triangle(x, y, a, b, c)) {
                obstacle_block_coord(obs, x, y);
            }
        }
    }

    return obs;
}

static void block_line_segment(obstacle_t* obs, int x0, int y0, int x1, int y1, int thickness) {
    int dx = abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = -abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (true) {
        if (thickness <= 0)
            obstacle_block_coord(obs, x0, y0);
        else
            obstacle_block_range(obs, x0, y0, thickness);

        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

obstacle_t* obstacle_make_triangle_torus(
    const coord_t* a,
    const coord_t* b,
    const coord_t* c,
    int thickness)
{
    if (!a || !b || !c || thickness < 0) return NULL;

    // 전체 영역 계산 (a, b, c 중 최대/최소 좌표)
    int min_x = a->x, max_x = a->x;
    int min_y = a->y, max_y = a->y;

    const coord_t* pts[3] = {a, b, c};
    for (int i = 1; i < 3; ++i) {
        if (pts[i]->x < min_x) min_x = pts[i]->x;
        if (pts[i]->x > max_x) max_x = pts[i]->x;
        if (pts[i]->y < min_y) min_y = pts[i]->y;
        if (pts[i]->y > max_y) max_y = pts[i]->y;
    }

    int width = max_x - min_x + 1 + thickness * 2;
    int height = max_y - min_y + 1 + thickness * 2;

    obstacle_t* obs = obstacle_create_full(min_x - thickness, min_y - thickness, width, height);
    if (!obs) return NULL;

    // 삼각형 외곽 세 선분만 block
    block_line_segment(obs, a->x, a->y, b->x, b->y, thickness);
    block_line_segment(obs, b->x, b->y, c->x, c->y, thickness);
    block_line_segment(obs, c->x, c->y, a->x, a->y, thickness);

    return obs;
}

static bool point_in_polygon(int x, int y, const coord_list_t* list) {
    int count = coord_list_length(list);
    if (count < 3) return false;

    bool inside = false;

    for (int i = 0, j = count - 1; i < count; j = i++) {
        const coord_t* a = coord_list_get(list, j);
        const coord_t* b = coord_list_get(list, i);
        if (!a || !b) continue;

        int x1 = a->x, y1 = a->y;
        int x2 = b->x, y2 = b->y;

        if ((y1 > y) != (y2 > y)) {
            float intersect = (float)(x2 - x1) * (y - y1) / (float)(y2 - y1) + x1;
            if (x < intersect)
                inside = !inside;
        }
    }

    return inside;
}

obstacle_t* obstacle_make_polygon(coord_list_t* list) {
    int count = coord_list_length(list);
    if (count < 3) return NULL;

    const coord_t* c0 = coord_list_get(list, 0);
    if (!c0) return NULL;

    int min_x = c0->x, max_x = c0->x;
    int min_y = c0->y, max_y = c0->y;

    for (int i = 1; i < count; ++i) {
        const coord_t* c = coord_list_get(list, i);
        if (!c) continue;

        if (c->x < min_x) min_x = c->x;
        if (c->x > max_x) max_x = c->x;
        if (c->y < min_y) min_y = c->y;
        if (c->y > max_y) max_y = c->y;
    }

    int width = max_x - min_x + 1;
    int height = max_y - min_y + 1;

    obstacle_t* obs = obstacle_create_full(min_x, min_y, width, height);
    if (!obs) return NULL;

    for (int x = min_x; x <= max_x; ++x) {
        for (int y = min_y; y <= max_y; ++y) {
            if (point_in_polygon(x, y, list)) {
                obstacle_block_coord(obs, x, y);
            }
        }
    }

    return obs;
}

obstacle_t* obstacle_make_polygon_torus(coord_list_t* list, int thickness) {
    int count = coord_list_length(list);
    if (count < 3 || thickness < 0) return NULL;

    const coord_t* first = coord_list_get(list, 0);
    if (!first) return NULL;

    int min_x = first->x, max_x = first->x;
    int min_y = first->y, max_y = first->y;

    for (int i = 1; i < count; ++i) {
        const coord_t* c = coord_list_get(list, i);
        if (!c) continue;
        if (c->x < min_x) min_x = c->x;
        if (c->x > max_x) max_x = c->x;
        if (c->y < min_y) min_y = c->y;
        if (c->y > max_y) max_y = c->y;
    }

    int width = max_x - min_x + 1 + thickness * 2;
    int height = max_y - min_y + 1 + thickness * 2;

    obstacle_t* obs = obstacle_create_full(min_x - thickness, min_y - thickness, width, height);
    if (!obs) return NULL;

    for (int i = 0; i < count; ++i) {
        const coord_t* a = coord_list_get(list, i);
        const coord_t* b = coord_list_get(list, (i + 1) % count);
        if (a && b) {
            block_line_segment(obs, a->x, a->y, b->x, b->y, thickness);
        }
    }

    return obs;
}
