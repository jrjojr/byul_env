#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "byul_config.h"
#include "internal/obstacle_common.h"

#ifdef __cplusplus
extern "C" {
#endif

BYUL_API obstacle_t* obstacle_make_rect_all_blocked(
    int x0, int y0, int width, int height);

// ratio는 0.0 ~ 1.0의 범위 1.0이면 풀 블락
BYUL_API obstacle_t* obstacle_make_rect_random_blocked(
    int x0, int y0, int width, int height, float ratio);

// range 0 : 해당 좌표만 , 1이상 주변까지 포함해서 장애물 생성한다.
// start, goal의 가상의 라인을 따라서...
BYUL_API obstacle_t* obstacle_make_beam(
    const coord_t* start, const coord_t* goal, int range);

#ifdef __cplusplus
}
#endif

#endif // OBSTACLE_H
