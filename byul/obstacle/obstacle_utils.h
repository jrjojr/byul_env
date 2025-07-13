#ifndef OBSTACLE_UTILS_H
#define OBSTACLE_UTILS_H

#include "byul_config.h"
#include "internal/map.h"
#include "internal/coord.h"
#include "internal/coord_hash.h"
#include "internal/obstacle.h"

#ifdef __cplusplus
extern "C" {
#endif

BYUL_API obstacle_t* make_rect_all_blocked(
    int x0, int y0, int width, int height);

// ratio는 0.0 ~ 1.0의 범위 1.0이면 풀 블락
BYUL_API obstacle_t* make_rect_random_blocked(
    int x0, int y0, int width, int height, float ratio);

#ifdef __cplusplus
}
#endif

#endif // OBSTACLE_UTILS_H
