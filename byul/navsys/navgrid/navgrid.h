#ifndef NAVGRID_H
#define NAVGRID_H

#include "byul_config.h"
#include "internal/coord.h"
#include "internal/coord_list.h"
#include "internal/coord_hash.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 좌표가 차단되었는지(blocked) 여부를 확인하는 함수 포인터입니다.
 *
 * 이 함수는 특정 좌표 `(x, y)`가 길찾기나 범위 연산 등에서
 * 이동 불가능한 셀인지 판단하는 용도로 사용됩니다.
 *
 * @param context 좌표 판단에 필요한 외부 데이터 (예: navgrid, 맵 등)
 * @param x 검사할 X 좌표
 * @param y 검사할 Y 좌표
 * @param userdata 사용자 정의 데이터 (옵션)
 * @return true - 해당 좌표는 차단(blocked)됨  
 *         false - 해당 좌표는 이동 가능
 */
typedef bool (*is_coord_blocked_func)(
    const void* context, int x, int y, void* userdata);

/**
 * @brief navgrid 기준으로 좌표가 차단되었는지 여부를 반환합니다.
 *
 * navgrid 내부 셀 정보에 따라 해당 좌표가
 * 벽이나 장애물 등으로 이동 불가능한지 판단합니다.
 *
 * @param context navgrid 객체 포인터 (const navgrid_t*)
 * @param x 검사할 X 좌표
 * @param y 검사할 Y 좌표
 * @param userdata 사용자 정의 데이터 (사용하지 않을 수 있음)
 * @return true - 해당 좌표는 차단됨  
 *         false - 해당 좌표는 이동 가능
 */
bool is_coord_blocked_navgrid(
    const void* context, int x, int y, void* userdata);

typedef enum {
    NAVGRID_DIR_4,
    NAVGRID_DIR_8
} navgrid_dir_mode_t;

struct s_navgrid {
    int width;
    int height;
    navgrid_dir_mode_t mode;

    coord_hash_t* blocked_coords;

    is_coord_blocked_func is_coord_blocked_fn;
};

typedef struct s_navgrid navgrid_t;

// 생성자 및 소멸자

// 0 x 0 , NAVGRID_DIR_8
BYUL_API navgrid_t* navgrid_new();

BYUL_API navgrid_t* navgrid_new_full(int width, int height, navgrid_dir_mode_t mode,
    is_coord_blocked_func is_coord_blocked_fn);

BYUL_API void navgrid_free(navgrid_t* m);

// 복사 및 비교
BYUL_API navgrid_t* navgrid_copy(const navgrid_t* m);
BYUL_API uint32_t navgrid_hash(const navgrid_t* m);
BYUL_API bool navgrid_equal(const navgrid_t* a, const navgrid_t* b);

// 속성 접근
BYUL_API int navgrid_get_width(const navgrid_t* m);
BYUL_API void navgrid_set_width(navgrid_t* m, int width);

BYUL_API int navgrid_get_height(const navgrid_t* m);
BYUL_API void navgrid_set_height(navgrid_t* m, int height);

BYUL_API void navgrid_set_is_coord_blocked_func(navgrid_t* m, is_coord_blocked_func fn);
BYUL_API is_coord_blocked_func navgrid_get_is_coord_blocked_fn(const navgrid_t* m);

BYUL_API navgrid_dir_mode_t navgrid_get_mode(const navgrid_t* m);
BYUL_API void navgrid_set_mode(navgrid_t* m);

// 장애물 관련
BYUL_API bool navgrid_block_coord(navgrid_t* m, int x, int y);
BYUL_API bool navgrid_unblock_coord(navgrid_t* m, int x, int y);
BYUL_API bool navgrid_is_inside(const navgrid_t* m, int x, int y);
// BYUL_API bool navgrid_is_blocked(const navgrid_t* m, int x, int y);
BYUL_API void navgrid_clear(navgrid_t* m);

// 차단 좌표 집합 반환
BYUL_API const coord_hash_t* navgrid_get_blocked_coords(const navgrid_t* m);

// 이웃 탐색
BYUL_API coord_list_t* navgrid_clone_adjacent(const navgrid_t* m, int x, int y);
BYUL_API coord_list_t* navgrid_clone_adjacent_all(const navgrid_t* m, int x, int y);

// max_range가 0이면 navgrid_clone_adjacent_all과 같다 주변 이웃만 확인
BYUL_API coord_list_t* navgrid_clone_adjacent_all_range(
    navgrid_t* m, int x, int y, int range);

BYUL_API coord_t* navgrid_clone_neighbor_at_degree(const navgrid_t* m, 
    int x, int y, double degree);
    
BYUL_API coord_t* navgrid_clone_neighbor_at_goal(const navgrid_t* m, 
    const coord_t* center, const coord_t* goal);

BYUL_API coord_list_t* navgrid_clone_adjacent_at_degree_range(
    const navgrid_t* m,
    const coord_t* center, const coord_t* goal,
    double start_deg, double end_deg,
    int range);

#ifdef __cplusplus
}
#endif

#endif // NAVGRID_H
