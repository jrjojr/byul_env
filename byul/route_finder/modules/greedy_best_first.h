#ifndef GREEDY_BEST_FIRST_H
#define GREEDY_BEST_FIRST_H

#include "internal/route_finder_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Greedy Best-First Search 알고리즘으로 경로를 탐색합니다.
 *
 * 이 알고리즘은 누적 비용(g)을 무시하고, 휴리스틱(h) 값만을 기준으로
 * 우선순위 큐를 사용하여 목적지까지 가장 "가까워 보이는" 경로를 탐색합니다.
 *
 * - heuristic_fn은 반드시 설정되어야 합니다 (예: default_heuristic).
 * - 퍼포먼스 제한을 위해 max_retry 이상 반복되면 탐색을 종료합니다.
 * - visited_logging이 true이면 route->visited에 탐색 순서가 기록됩니다.
 *
 * @param m               맵 정보
 * @param start           시작 좌표
 * @param goal            도착 좌표
 * @param heuristic_fn    휴리스틱 함수 (필수)
 * @param max_retry       최대 반복 횟수 (0 이하이면 무제한)
 * @param visited_logging 탐색 순서 기록 여부
 *
 * @return route_t* 경로 결과. 실패 시 success == false로 설정됨
 */

BYUL_API route_t* find_greedy_best_first(const map_t* m, 
    const coord_t* start, const coord_t* goal,
    heuristic_func heuristic_fn,
    int max_retry, bool visited_logging);

#ifdef __cplusplus
}
#endif

#endif // GREEDY_BEST_FIRST_H
