#ifndef OBSTACLE_COMMON_h
#define OBSTACLE_COMMON_h

#include "byul_config.h"
#include "internal/coord.h"
#include "internal/coord_hash.h"
#include "internal/navgrid.h"

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
BYUL_API void obstacle_fetch_origin(
    const obstacle_t* obstacle, int* out_x0, int* out_y0);

// 속성 접근
BYUL_API int obstacle_get_width(const obstacle_t* m);
BYUL_API void obstacle_set_width(obstacle_t* m, int width);

BYUL_API int obstacle_get_height(const obstacle_t* m);
BYUL_API void obstacle_set_height(obstacle_t* m, int height);


// 차단 좌표 직접 접근 (읽기 전용)
BYUL_API const coord_hash_t* obstacle_get_blocked_coords(
    const obstacle_t* obstacle);

BYUL_API bool obstacle_block_coord(obstacle_t* m, int x, int y);
BYUL_API bool obstacle_unblock_coord(obstacle_t* m, int x, int y);
BYUL_API bool obstacle_is_inside(const obstacle_t* m, int x, int y);
// BYUL_API bool obstacle_is_blocked(const obstacle_t* m, int x, int y);
BYUL_API void obstacle_clear(obstacle_t* m);

// 이웃 탐색
BYUL_API coord_list_t* obstacle_clone_neighbors(
    const obstacle_t* m, int x, int y);

BYUL_API coord_list_t* obstacle_clone_neighbors_all(
    const obstacle_t* m, int x, int y);

// max_range가 0이면 obstacle_clone_neighbors_all과 같다 주변 이웃만 확인
BYUL_API coord_list_t* obstacle_clone_neighbors_all_range(
    obstacle_t* m, int x, int y, int range);

BYUL_API coord_t* obstacle_clone_neighbor_at_degree(const obstacle_t* m, 
    int x, int y, double degree);
    
BYUL_API coord_t* obstacle_clone_neighbor_at_goal(const obstacle_t* m, 
    const coord_t* center, const coord_t* goal);

BYUL_API coord_list_t* obstacle_clone_neighbors_at_degree_range(
    const obstacle_t* m,
    const coord_t* center, const coord_t* goal,
    double start_deg, double end_deg,
    int range);

bool obstacle_is_coord_blocked(const obstacle_t* obstacle, int x, int y);

BYUL_API void obstacle_apply_to_navgrid(const obstacle_t* obstacle, navgrid_t* navgrid);

BYUL_API void obstacle_remove_from_navgrid(
    const obstacle_t* obstacle, navgrid_t* navgrid);

/**
 * @brief 지정 좌표를 중심으로 range 범위 안의 좌표를 모두 block 처리합니다.
 *
 * 이 함수는 `(x, y)`를 중심으로 `range` 반경 안의 모든 셀을
 * 사각형 영역 기준으로 block 처리합니다. 총 (2×range+1)² 개의 좌표가 처리됩니다.
 *
 * @param obs    장애물 객체
 * @param x      중심 좌표 x
 * @param y      중심 좌표 y
 * @param range  반경 범위 (0이면 자기 자신만 block)
 */
BYUL_API void obstacle_block_range(obstacle_t* obs, int x, int y, int range);

// 직진 방향으로 목표까지 반경 range 만큼 블락한다.
// range == 0 이면 해당 좌표만, 1 이상이면 주변 좌표도 블락한다.
BYUL_API void obstacle_block_straight(obstacle_t* obs, 
    int x0, int y0, int x1, int y1, int range);

#ifdef __cplusplus
}
#endif

#endif // OBSTACLE_COMMON_h
