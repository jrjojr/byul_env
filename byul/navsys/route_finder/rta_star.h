#ifndef RTA_STAR_H
#define RTA_STAR_H

#include "internal/route_finder_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct s_rta_star_config{
    int depth_limit; // 탐색 제한 깊이
} rta_star_config_t;
typedef rta_star_config_t* rta_star_config;

BYUL_API rta_star_config rta_star_config_create();

/**
 * @brief RTA* 알고리즘에서 사용할 깊이 제한 설정을 생성합니다.
 *
 * RTA* (Real-Time A*)는 전체 경로를 한 번에 탐색하지 않고,
 * 현재 위치에서 일정 깊이만큼 앞을 미리 탐색하여
 * 가장 유망한 다음 방향으로 한 칸씩 이동하는 알고리즘입니다.
 *
 * 이 함수는 해당 "앞을 미리 보는 깊이"를 설정합니다.
 * 설정된 깊이만큼 순차적으로 이웃 노드를 평가하여,
 * g + h 값이 가장 낮은 방향을 선택합니다.
 *
 * 깊이 제한(depth_limit)은 장애물의 구조와 크게 연관되어 있습니다.
 *
 * - 깊이 제한이 낮을수록 빠르게 반응하나, 복잡한 장애물을 피하지 못할 수 있습니다.
 * - 예를 들어, 장애물이 5칸 이상을 막고 있다면, depth_limit이 최소 6 이상이어야
 *   우회 경로를 탐지할 수 있습니다.
 * - depth_limit이 너무 작으면, 알고리즘은 막힌 방향을 최선이라 판단해
 *   경로 탐색에 실패할 수 있습니다.
 *
 * 일반적으로:
 * - 실시간 반응형 이동(NPC AI 등): 2~4 추천
 * - 복잡한 장애물 우회가 필요한 상황: 6~8 추천
 * - 미로와 같은 구조에서는 A*나 Dijkstra를 사용하는 것이 더 적합합니다.
 *
 * @param depth_limit 앞을 미리 보는 깊이 (1 이상 권장)
 * @return rta_star_config 설정 객체 (free 필요)
 */
BYUL_API rta_star_config rta_star_config_create_full(int depth_limit);


BYUL_API void rta_star_config_destroy(rta_star_config cfg);

/**
 * @brief RTA* (Real-Time A*) 알고리즘을 사용하여 경로를 탐색합니다.
 *
 * 이 알고리즘은 전체 경로를 미리 계산하지 않고,
 * 현재 위치에서 일정 깊이까지만 휴리스틱 기반으로 앞을 예측한 뒤,
 * 가장 유망한 방향으로 한 칸씩 이동하는 실시간 근사 탐색 방식입니다.
 *
 * 각 탐색 스텝에서는 다음과 같은 과정이 수행됩니다:
 * - 현재 위치에서 최대 depth_limit만큼의 lookahead 탐색 수행
 * - g + h 평가 값이 가장 낮은 이웃 노드를 선택
 * - 한 칸 이동 후 반복
 *
 * 깊이 제한이 충분하지 않으면 장애물을 피하지 못하고
 * 경로 탐색에 실패할 수 있으며,
 * 예측된 경로가 잘못되어 일찍 종료될 수 있습니다.
 *
 * 예시: 중앙에 세로 장애물이 있는 10x10 맵에서,
 *       깊이 제한이 6 이하면 우회로를 탐지하지 못하고 실패,
 *       7 이상이면 장애물을 피해서 성공적으로 도착 지점에 도달합니다.
 *
 * 사용 예:
 * @code
 * coord_t* start = coord_create_full(0, 0);
 * coord_t* goal = coord_create_full(9, 9);
 *
 * rta_star_config cfg = rta_star_config_create_full(7); // 깊이 제한 7
 * route_finder al = route_finder_create_full(
 *     10, 10,
 *     NAVGRID_DIR_8,
 *     ROUTE_FINDER_RTA_STAR,
 *     default_cost,
 *     default_heuristic,
 *     NULL,
 *     cfg,
 *     true
 * );
 *
 * // 장애물: 중앙 수직 벽
 * for (int y = 1; y < 10; y++)
 *     navgrid_block_coord(al->m, 5, y);
 *
 * route_t* p = route_finder_run(al, start, goal);
 * navgrid_print_ascii_with_visited_count(al->m, p, start, goal);
 *
 * route_destroy(p);
 * coord_destroy(start);
 * coord_destroy(goal);
 * route_finder_destroy(al);
 * rta_star_config_destroy(cfg);
 * @endcode
 *
 * @param al    알고리즘 컨텍스트 (route_finder_create_full로 생성)
 * @param start  시작 좌표
 * @param goal    도착 좌표
 * @return 탐색 결과 route_t*. 경로를 찾았으면 success == true, 실패 시 false.
 */
BYUL_API route_t* find_rta_star(const navgrid_t* m,
    const coord_t* start, const coord_t* goal,
    cost_func cost_fn, heuristic_func heuristic_fn,
    int depth_limit, // 탐색 제한 깊이
    int max_retry, bool visited_logging);

#ifdef __cplusplus
}
#endif

#endif // RTA_STAR_H
