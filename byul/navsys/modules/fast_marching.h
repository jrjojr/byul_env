#ifndef FAST_MARCHING_H
#define FAST_MARCHING_H

#include "internal/route_finder_common.h"
#include "internal/cost_coord_pq.h"
#include "internal/navgrid.h"

#ifdef __cplusplus
extern "C" {
#endif

/// 최대 확산 반경 제한값 (0 이하 또는 이 값을 초과하는 반경은 MAX_RADIUS로 대체)
#define MAX_RADIUS 1e6f

typedef enum e_fmm_state {
    FMM_FAR = 0,
    FMM_NARROW,
    FMM_KNOWN
} fmm_state_t;

typedef struct s_fmm_cell {
    fmm_state_t state;
    float value;  // 거리값 T
} fmm_cell_t;

typedef struct {
    int width;
    int height;
    coord_hash_t* cells;  // coord_t* → fmm_cell_t*
    coord_list_t* visit_order;  //  방문 기록
    int total_retry_count;
} fmm_grid_t;

/**
 * Fast Marching Method(FMM)를 이용해 시작 지점으로부터의 거리장을 계산합니다.
 *
 * @param m             맵 정보
 * @param start         시작 좌표
 * @param cost_fn       이동 비용 함수 (nullptr이면 1.0 고정)
 * @param radius_limit  최대 탐색 반경 (0 이하이면 MAX_RADIUS 사용)
 * @param max_retry     최대 반복 횟수 제한 (0 이하이면 무제한)
 *
 * @return fmm_grid_t* 계산된 거리장 구조체 (동적 할당됨)
 */
BYUL_API fmm_grid_t* fmm_compute(const navgrid_t* m, const coord_t* start, 
    cost_func cost_fn, float radius_limit, int max_retry);

/**
 * fmm_grid_t 구조체가 소유한 모든 동적 메모리를 해제합니다.
 *
 * @param grid fmm_compute()에 의해 생성된 거리장 결과 구조체
 */
BYUL_API void fmm_grid_destroy(fmm_grid_t* grid);

/**
 * 거리장 결과를 ASCII 형태로 stdout에 출력합니다.
 * 셀이 존재하지 않는 경우 " .. " 로 표시되며,
 * 값이 있는 셀은 정수부만 표시됩니다.
 *
 * @param grid 거리장 구조체
 */
BYUL_API void fmm_dump_ascii(const fmm_grid_t* grid);

/**
 * Fast Marching 기반으로 start → goal까지 최단 경로를 복원합니다.
 * 목표지점까지 도달 가능하지 않으면 실패한 route를 반환합니다.
 * visited_logging이 true일 경우, 
 * fmm_grid_t의 visit_order 순서대로 route->visited에 기록합니다.
 *
 * @param m               맵 정보
 * @param start           시작 좌표
 * @param goal            목표 좌표
 * @param cost_fn         이동 비용 함수 (nullptr이면 1.0 고정)
 * @param max_retry     최대 반복 횟수 (무한 루프 방지를 위한 제한).
 *                      일반적으로 width * height 정도의 값을 권장합니다.
 * @param visited_logging 방문 좌표의 카운트 기록 여부 
 *
 * @return route_t* 경로 구조체 (성공 여부 포함)
 */
BYUL_API route_t* find_fast_marching(const navgrid_t* m, 
    const coord_t* start, const coord_t* goal,
    cost_func cost_fn, int max_retry, bool visited_logging);

#ifdef __cplusplus
}
#endif

#endif // FAST_MARCHING_H
