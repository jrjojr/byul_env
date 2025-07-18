#ifndef SMA_STAR_H
#define SMA_STAR_H

#include "internal/route_finder_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SMA* 알고리즘을 사용하여 메모리 제한 하에 최단 경로를 탐색합니다.
 *
 * SMA* (Simplified Memory-Bounded A*)는 A* 알고리즘과 유사하게
 * f = g + h 기반의 우선순위 탐색을 수행하지만, 
 * 사용 가능한 메모리(노드 수)를 제한하여,
 * 해당 제한을 초과할 경우 우선순위가 가장 낮은 노드를 제거하며 탐색을 지속합니다.
 *
 * 이 방식은 제한된 환경에서의 경로 탐색에 적합하며, 
 * 메모리 한계 내에서 가능한 최적의 경로를 찾도록 설계되었습니다.
 *
 * 제거된 노드는 `came_from` 경로 추적 정보가 사라지므로, 
 * 일부 경로 복원이 불가능할 수 있습니다. 
 * 이에 따라 제한이 클수록 A*에 가까운 품질을 제공하며, 
 * 제한이 작을수록 경로 품질이 떨어지거나 탐색에 실패할 수 있습니다.
 *
 * @param m             맵 객체 (navgrid_new 또는 navgrid_load 등으로 생성된 맵)
 * @param start         시작 좌표 (coord_new 또는 coord_new_full로 생성)
 * @param goal          도착 좌표 (coord_t 구조체, const로 전달됨)
 * @param cost_fn       이동 비용 함수 (NULL일 경우 default_cost 사용)
 * @param heuristic_fn  휴리스틱 함수 (NULL일 경우 default_heuristic 사용)
 * @param memory_limit  탐색 중 유지할 수 있는 최대 노드 수
 *                      (0 또는 너무 작으면 탐색 실패 가능성 있음)
 * @param max_retry     경로 실패 시 재시도 최대 횟수 (0이면 단일 시도)
 * @param visited_logging TRUE인 경우, 방문한 셀의 수를 내부 기록에 저장함
 *
 * @return 경로 결과 route_t* 
 *         - 성공 시: route_get_success(route_t*) == true
 *         - 실패 시: route는 비어 있으며, success == false
 *
 * @note
 * memory_limit은 맵 크기 및 난이도에 따라 달라져야 하며,
 * 일반적으로 다음을 권장합니다:
 *   - memory_limit ≈ max(L × (1 + ε), N × α)
 *     (L: 예상 경로 길이, N: 맵 셀 수)
 *     (ε ∈ [0.5, 1.0], α ∈ [0.01, 0.05])
 *
 * @par 권장 메모리 제한 예시
 *   - 10x10 맵  : memory_limit ≈ 20 ~ 30
 *   - 100x100 맵: memory_limit ≈ 500 ~ 1000
 *   - 1000x1000 맵: memory_limit ≈ 50,000 ~ 100,000
 *
 */
BYUL_API route_t* find_sma_star(const navgrid_t* m,
    const coord_t* start, const coord_t* goal,
    cost_func cost_fn, heuristic_func heuristic_fn,
    int memory_limit,
    int max_retry, bool visited_logging);

#ifdef __cplusplus
}
#endif

#endif // SMA_STAR_H
