#ifndef COORD_RADAR_H
#define COORD_RADAR_H

#include "byul_config.h"
#include "internal/coord.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_RANGE_LIMIT 256

// 🔍 사용자가 지정하는 도달 가능성 판단 함수
typedef bool (*is_reachable_func)(const coord_t* coord, void* user_data);

/**
 * @brief 기준 좌표 주변에서 가장 가까운 reachable 셀을 BFS로 탐색합니다.
 * 
 * @param start           기준 좌표 (일반적으로 클릭된 좌표)
 * @param is_reachable    도달 가능 여부를 판단하는 콜백 함수
 * @param user_data       콜백에 전달할 사용자 데이터
 * @param max_range       최대 탐색 반경 (예: 5 → 5칸)
 * @param out_result      결과 좌표 포인터 (성공 시 셋팅됨, 실패 시 x=-1, y=-1)
 * 
 * @return bool           true: 찾음, false: 실패
 */
BYUL_API bool find_goal_bfs(const coord_t* start,
                             is_reachable_func is_reachable_fn,
                             void* user_data,
                             int max_range,
                             coord_t* out_result);

typedef struct s_astar_node astar_node_t;

BYUL_API int astar_node_compare(const astar_node_t* a, const astar_node_t* b);

/**
 * @brief std::priority_queue 기반 A* 방식으로 가장 가까운 reachable 좌표를 탐색
 */
BYUL_API bool find_goal_astar(const coord_t* start,
                               is_reachable_func is_reachable_fn,
                               void* user_data,
                               int max_range,
                               coord_t* out_result);

#ifdef __cplusplus
}
#endif

#endif // COORD_RADAR_H
