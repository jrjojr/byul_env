#ifndef IDA_STAR_H
#define IDA_STAR_H

#include "internal/route_finder_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief IDA* (Iterative Deepening A*) 알고리즘을 사용하여 경로를 탐색합니다.
 *
 * IDA*는 깊이 우선 탐색(DFS)과 A*의 휴리스틱 기반 비용 평가를 결합한 알고리즘입니다.
 * 각 반복에서 f = g + h 값이 임계값(threshold)을 초과하지 않는 노드들만 탐색하며,
 * threshold를 점차 증가시키며 최적 경로에 도달할 때까지 반복합니다.
 *
 * 이 알고리즘은 A*에 비해 메모리 사용량이 적고, 복잡한 자료구조 없이도
 * 비교적 간단한 방식으로 최적 경로 탐색을 수행할 수 있도록 설계되어 있습니다.
 *
 * ### 장점
 * - 메모리 사용량이 매우 적습니다. 
 *      A*와 달리 전체 open/closed 리스트를 유지하지 않기 때문에
 *   대형 맵, 모바일, 임베디드 환경에 적합합니다.
 * - 최적 경로 보장: 반복적으로 threshold를 조정함으로써 최단 경로에 도달합니다.
 * - 구현이 간단합니다. DFS 구조 위에 threshold 조건만 추가되며, 
 *      별도 자료구조 없이도 구현 가능합니다.
 *
 * ### 단점
 * - 반복 횟수가 많을 수 있습니다. 
 *  특히 휴리스틱이 완만하게 증가할 경우 반복이 수백 번 이상 발생할 수 있습니다.
 * - threshold를 점진적으로 증가시키기 때문에 실시간성에 부적합할 수 있습니다.
 * - A*처럼 우선순위 큐에 기반한 최적 확장을 하지 않기 때문에 성능이 불안정할 수 있습니다.
 *
 * ### 휴리스틱 함수 선택
 * - heuristic_fn은 NULL로 설정 가능하며, 
 *      이 경우 내부에서 자동으로 맨해튼 거리 함수를 사용합니다.
 * - IDA*는 맨해튼 거리와의 상성이 가장 좋습니다. 예시로, 동일한 맵에서:
 *   - 유클리드 거리 사용 시 760회 반복
 *   - 맨해튼 거리 사용 시 88회 반복으로 동일한 목표에 도달하였습니다.
 * - 따라서 휴리스틱을 모르거나 따로 정의하지 않으실 경우 NULL로 설정하는 것이 
 *  가장 안전합니다.
 *
 * @param m               맵 정보
 * @param start           시작 좌표
 * @param goal            도착 좌표
 * @param cost_fn         이동 비용 함수 (NULL 가능)
 * @param heuristic_fn    휴리스틱 함수 (NULL 가능, 기본은 맨해튼 거리)
 * @param max_retry       최대 반복 횟수 (0 이하이면 무제한)
 * @param visited_logging 방문 경로 기록 여부 (true 시 route->visited에 저장됨)
 * @return route_t* 탐색 결과 경로. 실패 시 success == false
 */
BYUL_API route_t* find_ida_star(const navgrid_t* m, 
    const coord_t* start, const coord_t* goal,
    cost_func cost_fn, heuristic_func heuristic_fn,
    int max_retry, bool visited_logging);

#ifdef __cplusplus
}
#endif

#endif // IDA_STAR_H
