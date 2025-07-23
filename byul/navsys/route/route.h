#ifndef ROUTE_H
#define ROUTE_H

#include "byul_common.h"
#include "internal/coord.h"
#include "internal/coord_list.h"
#include "internal/coord_hash.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 방향 열거형 **/
typedef enum e_route_dir {
    ROUTE_DIR_UNKNOWN, 
    ROUTE_DIR_RIGHT,
    ROUTE_DIR_UP_RIGHT,
    ROUTE_DIR_UP,
    ROUTE_DIR_UP_LEFT,
    ROUTE_DIR_LEFT,
    ROUTE_DIR_DOWN_LEFT,
    ROUTE_DIR_DOWN,
    ROUTE_DIR_DOWN_RIGHT,
    ROUTE_DIR_COUNT
} route_dir_t;

struct s_route {
    coord_list_t* coords;
    coord_list_t* visited_order;
    coord_hash_t* visited_count;
    float cost;
    bool success;
    int total_retry_count;

    float avg_vec_x;
    float avg_vec_y;
    int vec_count;
};

typedef struct s_route route_t;

/** 생성 및 해제 **/
BYUL_API route_t* route_create(void);
BYUL_API route_t* route_create_full(float cost);
BYUL_API void  route_destroy(route_t* p);

/** 복사 및 비교 **/
BYUL_API route_t* route_copy(const route_t* p);
BYUL_API uintptr_t route_hash(const route_t* a);
BYUL_API int route_equal(const route_t* a, const route_t* b);

/** 기본 정보 **/
BYUL_API void  route_set_cost(route_t* p, float cost);
BYUL_API float route_get_cost(const route_t* p);
BYUL_API void  route_set_success(route_t* p, int success);
BYUL_API int   route_get_success(const route_t* p);

/** 좌표 리스트 접근 **/
BYUL_API const coord_list_t* route_get_coords(const route_t* p);

/** 방문 로그 **/
BYUL_API const coord_list_t* route_get_visited_order(const route_t* p);
BYUL_API const coord_hash_t*  route_get_visited_count(const route_t* p);

BYUL_API int route_get_total_retry_count(const route_t* p);

BYUL_API void route_set_total_retry_count(route_t* p, int retry_count);

/** 좌표 조작 **/
BYUL_API int  route_add_coord(route_t* p, const coord_t* c);
BYUL_API void route_clear_coords(route_t* p);
BYUL_API const coord_t* route_get_last(const route_t* p);
BYUL_API const coord_t* route_get_coord_at(const route_t* p, int index);
BYUL_API int   route_length(const route_t* p);

/** 방문 조작 **/
BYUL_API int  route_add_visited(route_t* p, const coord_t* c);
BYUL_API void route_clear_visited(route_t* p);

/** 병합 및 편집 **/
BYUL_API void route_append(route_t* dest, const route_t* src);

// 여러개의 경로를 병합할때 시작과 끝이 서로 중복되는 경로가 있으면 시작과 끝을
// 중복제거해서 하나의 좌표로 생성한다 중간에 중복되는 경로는 병합하지 않는다
// 오로지 시작과 끝만 병합한다.
BYUL_API void route_append_nodup(route_t* dest, const route_t* src);

BYUL_API void route_insert(route_t* p, int index, const coord_t* c);
BYUL_API void route_remove_at(route_t* p, int index);
BYUL_API void route_remove_value(route_t* p, const coord_t* c);
BYUL_API int  route_contains(const route_t* p, const coord_t* c);
BYUL_API int  route_find(const route_t* p, const coord_t* c);

// BYUL_API void route_slice(route_t* p, int start, int end);

// route에서 시작 인덱스와 끝 인덱스를 부분적으로 잘라내서 반환한다.
// 원본은 유지 하고 새로운 route_t*가 생성되는거다.
BYUL_API route_t* route_slice(const route_t* p, int start, int end);

/** 출력 및 디버깅 **/
BYUL_API void route_print(const route_t* p);

/** 방향 계산 **/
BYUL_API coord_t* route_make_direction(route_t* p, int index);
BYUL_API route_dir_t route_get_direction_by_dir_coord(const coord_t* dxdy);
BYUL_API route_dir_t route_get_direction_by_index(route_t* p, int index);
BYUL_API route_dir_t route_calc_average_facing(route_t* p, int history);
BYUL_API float route_calc_average_dir(route_t* p, int history);

BYUL_API coord_t* direction_to_coord(route_dir_t route_dir);

/** 방향 변화 판단 **/
BYUL_API int route_has_changed(
    route_t* p, const coord_t* from,
    const coord_t* to, float angle_threshold_deg);

BYUL_API int route_has_changed_with_angle(
    route_t* p, const coord_t* from,
    const coord_t* to, float angle_threshold_deg,
    float* out_angle_deg);

BYUL_API int route_has_changed_by_index(
    route_t* p, int index_from,
    int index_to, float angle_threshold_deg);

BYUL_API int route_has_changed_with_angle_by_index(
    route_t* p, int index_from, int index_to,
    float angle_threshold_deg, float* out_angle_deg);

/** 평균 벡터 누적 **/
BYUL_API void route_update_average_vector(
    route_t* p, const coord_t* from, const coord_t* to);

BYUL_API void route_update_average_vector_by_index(
    route_t* p, int index_from, int index_to);

BYUL_API route_dir_t calc_direction(
    const coord_t* start, const coord_t* goal);

/// @brief goal → start 방향으로 came_from을 따라 경로를 복원하여 route에 채운다.
/// @param route 결과 경로 구조체 (output)
/// @param came_from coord_hash_t* (coord* → coord*)
/// @param start 시작 좌표
/// @param goal 도착 좌표
/// @return 성공 여부 (true: 경로 복원 성공, false: 실패)
BYUL_API bool route_reconstruct_path(
    route_t* route, const coord_hash_t* came_from,
    const coord_t* start, const coord_t* goal);


#ifdef __cplusplus
}
#endif

#endif // ROUTE_H
