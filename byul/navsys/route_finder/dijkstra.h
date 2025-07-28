#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include "byul_common.h"
#include "coord.h"
#include "navgrid.h"
#include "route.h"
#include "route_finder_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Dijkstra 알고리즘을 사용하여 최단 경로를 탐색합니다.
 *
 * 이 함수는 휴리스틱 없이 실제 누적 비용만을 기준으로 경로를 탐색하는
 * Dijkstra 알고리즘을 구현합니다.  
 * 각 인접 좌표에 대한 이동 비용은 @p cost_fn 함수에 따라 계산되며,  
 * 이를 누적하여 가장 낮은 총 비용 경로를 찾습니다.
 *
 * A* 알고리즘과 달리 휴리스틱을 사용하지 않기 때문에 항상 최단 경로를 보장하지만,  
 * 탐색에 필요한 노드 수가 많아질 수 있습니다.
 *
 * 함수는 내부적으로 우선순위 큐, 비용 테이블, 경로 추적 테이블을 생성 및 관리하며,  
 * 입력으로는 맵, 시작/목표 좌표, 그리고 비용 함수만 제공하면 됩니다.
 *
 * @param m         탐색 대상 맵
 * @param start     시작 좌표
 * @param goal      목표 좌표
 * @param cost_fn   좌표 간 이동 비용을 계산하는 함수. 
 *                  NULL인 경우 기본 비용 1.0f가 사용됩니다.
 * @return          경로 객체.  
 *                  @c route_get_success(route_t*) 가 
 *                  TRUE인 경우 탐색에 성공한 것입니다.
 */
BYUL_API route_t* find_dijkstra(const navgrid_t* m, 
    const coord_t* start, const coord_t* goal, cost_func cost_fn,
    int max_retry, bool visited_logging);

#ifdef __cplusplus
}
#endif

#endif // DIJKSTRA_H
