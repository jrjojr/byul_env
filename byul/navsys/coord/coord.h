#ifndef COORD_H
#define COORD_H

#include <stdint.h>
#include <stdbool.h>
#include "byul_config.h"

#ifdef __cplusplus
extern "C" {
#endif

// ------------------------ 구조체 정의 ------------------------

typedef struct s_coord {
    int x;
    int y;
} coord_t;

// ------------------------ 생성/해제 ------------------------

BYUL_API coord_t* coord_create_full(int x, int y);
BYUL_API coord_t* coord_create(void);
BYUL_API void     coord_destroy(coord_t* c);
BYUL_API coord_t* coord_copy(const coord_t* c);

// ------------------------ 비교/해시 ------------------------

BYUL_API unsigned coord_hash(const coord_t* c);
BYUL_API bool     coord_equal(const coord_t* c1, const coord_t* c2);
BYUL_API int      coord_compare(const coord_t* c1, const coord_t* c2);

/// 유클리드 거리 계산
BYUL_API float      coord_distance(const coord_t* a, const coord_t* b);

BYUL_API int coord_manhattan_distance(const coord_t* a, const coord_t* b);

// 360도 반환 좌표들간의 각도를...
BYUL_API double   coord_degree(const coord_t* a, const coord_t* b);

// 시작 좌표에서 목표좌표로 가기위해 가장 가까운 이웃을 반환한다.
BYUL_API coord_t* coord_clone_next_to_goal(
    const coord_t* start, const coord_t* goal);

// ------------------------ 좌표 접근자/설정자 ------------------------

BYUL_API int      coord_get_x(const coord_t* c);
BYUL_API void     coord_set_x(coord_t* c, int x);

BYUL_API int      coord_get_y(const coord_t* c);
BYUL_API void     coord_set_y(coord_t* c, int y);

BYUL_API void     coord_set(coord_t* c, int x, int y);
BYUL_API void     coord_fetch(const coord_t* c, int* out_x, int* out_y);

BYUL_API const coord_t* make_tmp_coord(int x, int y);

#ifdef __cplusplus
}
#endif

#endif // COORD_H
