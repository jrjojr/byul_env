#ifndef WEIGHTED_ASTAR_H
#define WEIGHTED_ASTAR_H

#include "byul_config.h"
#include "internal/common.h"

#include "internal/route_finder_common.h"
#include "internal/navgrid.h"
#include "internal/coord.h"
#include "internal/route.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Weighted A* 알고리즘을 사용하여 경로를 탐색합니다.
 *
 * Weighted A*는 고전적인 A* 알고리즘에서 휴리스틱 h(n)의 영향력을 
 * 조절할 수 있도록 가중치(weight)를 도입한 확장 알고리즘입니다.
 * f-score 계산식은 다음과 같습니다:
 *
 *     f(n) = g(n) + weight × h(n)
 *
 * - g(n): 시작점부터 현재 노드까지의 누적 비용
 * - h(n): 현재 노드에서 목표 지점까지의 휴리스틱 추정값
 * - weight: h(n)의 영향력을 조절하는 계수 (1.0 이상 권장)
 *
 * 일반 A*는 weight = 1.0일 때와 동일하며, 이보다 높은 값을 사용할수록
 * 휴리스틱 기반의 빠른 탐색을 유도하지만 경로 품질은 떨어질 수 있습니다.
 *
 * @par 추천 가중치 범위
 * - weight = 1.0 : 기본 A* (경로 최적화, 느릴 수 있음)
 * - weight = 1.2 ~ 2.5 : 적절한 성능-품질 균형
 * - weight = 5.0 이상 : Greedy 탐색에 가까움 (비최적 경로 가능)
 *
 * @note
 * - @c weight <= 0.0 이면 내부적으로 1.0으로 보정됩니다.
 * - @c cost_fn, @c heuristic_fn 이 nullptr인 경우 내부 기본 함수가 사용됩니다.
 * - 실패 시에도 가능한 최종 좌표까지 경로를 복원합니다.
 *
 * @param m               맵 정보
 * @param start           시작 좌표
 * @param goal            도착 좌표
 * @param cost_fn         이동 비용 함수 (nullptr 가능)
 * @param heuristic_fn    휴리스틱 함수 (nullptr 가능)
 * @param weight          휴리스틱에 곱해지는 가중치 계수
 * @param max_retry       최대 반복 횟수 (0 이하이면 무제한)
 * @param visited_logging 방문 셀 기록 여부
 * @return route_t*       경로 객체 (성공 여부는 success 필드로 확인)
 */
BYUL_API route_t* find_weighted_astar(const navgrid_t* m,
    const coord_t* start, const coord_t* goal,
    cost_func cost_fn, heuristic_func heuristic_fn,
    float weight,
    int max_retry, bool visited_logging);

#ifdef __cplusplus
}
#endif

#endif // WEIGHTED_ASTAR_H
