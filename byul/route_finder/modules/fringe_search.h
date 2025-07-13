#ifndef FRINGE_SEARCH_H
#define FRINGE_SEARCH_H

#include "internal/route_finder_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Fringe Search 알고리즘을 사용하여 경로를 탐색합니다.
 *
 * Fringe Search는 A*의 Open List 정렬 병목을 제거하고,
 * f = g + h 값 기반 threshold를 점진적으로 확장하며 경로를 탐색합니다.
 * 두 개의 우선순위 큐를 번갈아 사용하여 현재 임계값(threshold) 내 노드를 처리하며,
 * threshold + delta_epsilon을 초과하는 노드는 다음 라운드로 이월됩니다.
 *
 * A*보다 빠른 수행 속도를 가지지만,
 * f 값 정렬이 없기 때문에 경로가 항상 최적은 아닐 수 있습니다.
 *
 * ### delta_epsilon의 의미와 추천값
 * - `delta_epsilon`은 threshold와의 허용 오차입니다.
 * - 이 값이 크면 더 많은 노드가 확장 대상으로 간주되어 탐색은 넓어지나, 
 *      pruning 효과는 줄어듭니다.
 * - 이 값이 작으면 효율적인 pruning이 가능하지만 경로를 못 찾을 위험이 커집니다.
 *
 * 예시:
 * - 맵의 크기와 상관없다. 10x10이고 유클리드 휴리스틱을 사용하는 경우
 *   - `delta_epsilon = 0.5f`: 정밀한 pruning, 경로 실패 가능성 증가
 *   - `delta_epsilon = 1.5f`: 균형 잡힌 설정 (추천)
 *   - `delta_epsilon = 3.0f 이상`: 넓은 탐색, 성능 저하 위험
 *
 * @param m               맵 정보
 * @param start           시작 좌표
 * @param goal            목표 좌표
 * @param cost_fn         이동 비용 함수 (NULL이면 default_cost)
 * @param heuristic_fn    휴리스틱 함수 (NULL이면 default_heuristic)
 * @param delta_epsilon   threshold 오차 허용값 (0 이하이면 0.5f로 고정)
 * @param max_retry       최대 반복 횟수 (0 이하이면 무제한)
 * @param visited_logging 방문 기록 여부 (true일 경우 route에 방문 순서 저장)
 *
 * @return route_t* 탐색 결과. success가 true면 경로를 찾은 것.
 *         실패 시에도 마지막으로 탐색된 경로까지는 기록됨.
 */
BYUL_API route_t* find_fringe_search(const map_t* m, 
    const coord_t* start, const coord_t* goal,
    cost_func cost_fn, heuristic_func heuristic_fn, float delta_epsilon,
    int max_retry, bool visited_logging);

#ifdef __cplusplus
}
#endif

#endif // FRINGE_SEARCH_H
