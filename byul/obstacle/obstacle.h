#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "byul_config.h"
#include "internal/map.h"
#include "internal/coord.h"
#include "internal/coord_hash.h"

#ifdef __cplusplus
extern "C" {
#endif

// obstacle 구조체 정의
typedef struct s_obstacle {
    int x0;
    int y0;
    int width;
    int height;
    coord_hash_t* blocked;
} obstacle_t;

// 기본 생성자 / 소멸자
BYUL_API obstacle_t* obstacle_new();
BYUL_API obstacle_t* obstacle_new_full(
    int x0, int y0, int width, int height);
BYUL_API void obstacle_free(obstacle_t* obstacle);

BYUL_API void obstacle_clear(obstacle_t* obstacle);

// 복사 및 비교
BYUL_API obstacle_t* obstacle_copy(const obstacle_t* obstacle);
BYUL_API bool obstacle_equal(const obstacle_t* a, const obstacle_t* b);
BYUL_API uint32_t obstacle_hash(const obstacle_t* obstacle);

// origin 설정 / 조회
BYUL_API void obstacle_set_origin(obstacle_t* obstacle, int x0, int y0);
BYUL_API void obstacle_get_origin(
    const obstacle_t* obstacle, int* out_x0, int* out_y0);

// 크기 조회
BYUL_API int obstacle_get_width(const obstacle_t* obstacle);
BYUL_API int obstacle_get_height(const obstacle_t* obstacle);

// 차단 좌표 직접 접근 (읽기 전용)
BYUL_API const coord_hash_t* obstacle_get_blocked_coords(
    const obstacle_t* obstacle);

BYUL_API void obstacle_apply_to_map(const obstacle_t* obstacle, map_t* map);

BYUL_API void obstacle_remove_from_map(const obstacle_t* obstacle, map_t* map);

#ifdef __cplusplus
}
#endif

#endif // OBSTACLE_H
