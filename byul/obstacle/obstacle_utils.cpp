#include "internal/obstacle.h"
#include "internal/obstacle_utils.h"
#include <stdlib.h>
#include <time.h>

obstacle_t* make_rect_all_blocked(int x0, int y0, int width, int height) {
    if (width <= 0 || height <= 0) return nullptr;

    obstacle_t* obstacle = obstacle_new_full(x0, y0, width, height);
    if (!obstacle) return nullptr;

    for (int dy = 0; dy < height; ++dy) {
        for (int dx = 0; dx < width; ++dx) {
            coord_t c = { x0 + dx, y0 + dy };
            coord_hash_insert(obstacle->blocked, &c, nullptr);
        }
    }
    return obstacle;
}

obstacle_t* make_rect_random_blocked(
    int x0, int y0, int width, int height, float ratio) {

    if (width <= 0 || height <= 0 || ratio <= 0.0f) return nullptr;
    if (ratio > 1.0f) ratio = 1.0f;

    obstacle_t* obstacle = obstacle_new_full(x0, y0, width, height);
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
