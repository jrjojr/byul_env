#ifndef ASTAR_H
#define ASTAR_H

#include "byul_config.h"

#include "internal/route_finder_common.h"
#include "internal/navgrid.h"
#include "internal/coord.h"
#include "internal/route.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief A* 알고리즘을 이용해 최단 경로를 탐색합니다.
 *
 * A*는 실제 이동 비용 g(n)과 목표까지의 추정 거리 h(n)을 더한 f(n) 값을 기준으로  
 * 경로를 탐색하는 휴리스틱 기반 알고리즘입니다.  
 *
 *     f(n) = g(n) + h(n)
 *
 * 이 함수는 맵(@p m) 상의 @p start 좌표에서 @p goal 좌표까지의 경로를 탐색하며,  
 * 비용 함수(@p cost_fn)와 휴리스틱 함수(@p heuristic_fn)를 외부에서 주입받아  
 * 경로의 누적 비용(g)과 휴리스틱(h)을 계산합니다.
 *
 * 내부적으로 우선순위 큐, 비용 테이블, 경로 추적 테이블을 구성하며,  
 * 최종 경로는 route_t* 구조체로 반환됩니다.
 *
 * @param m                탐색 대상 맵 객체
 * @param start            시작 좌표
 * @param goal             목표 좌표
 * @param cost_fn          좌표 간 이동 비용을 계산하는 함수. NULL이면 기본값 1.0이 사용됩니다.
 * @param heuristic_fn     휴리스틱 거리 계산 함수. NULL이면 기본 유클리드 거리를 사용합니다.
 * @return                 경로 객체. @c route_get_success(route_t*) 가 TRUE이면 탐색에 성공한 것입니다.
 *
 * @see route_get_success(), cost_func, heuristic_func
 */
BYUL_API route_t* find_astar(const navgrid_t* m, const coord_t* start, const coord_t* goal,
    cost_func cost_fn, heuristic_func heuristic_fn,
    int max_retry, bool visited_logging);


#ifdef __cplusplus
}
#endif

#endif // ASTAR_H
