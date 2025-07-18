#ifndef ROUTE_FINDER_H
#define ROUTE_FINDER_H

#include "internal/route_finder_common.h"

// 모든 알고리즘 모듈 포함
#include "internal/astar.h"
#include "internal/bfs.h"
#include "internal/dfs.h"
#include "internal/dijkstra.h"
#include "internal/fast_marching.h"
#include "internal/fringe_search.h"
#include "internal/greedy_best_first.h"
#include "internal/ida_star.h"
#include "internal/rta_star.h"
#include "internal/sma_star.h"
#include "internal/weighted_astar.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum e_route_finder_type{
    ROUTE_FINDER_UNKNOWN = 0,

    // 1950s~1960s
    ROUTE_FINDER_BELLMAN_FORD,            // 1958
    ROUTE_FINDER_DFS,                     // 1959
    ROUTE_FINDER_BFS,                     // 1959
    ROUTE_FINDER_DIJKSTRA,                // 1959
    ROUTE_FINDER_FLOYD_WARSHALL,          // 1959~
    ROUTE_FINDER_ASTAR,                   // 1968

    // 1970s
    ROUTE_FINDER_BIDIRECTIONAL_DIJKSTRA,  // 1971
    ROUTE_FINDER_BIDIRECTIONAL_ASTAR,     // 1971
    ROUTE_FINDER_WEIGHTED_ASTAR,          // 1977~
    ROUTE_FINDER_JOHNSON,                 // 1977
    ROUTE_FINDER_K_SHORTEST_PATH,         // 1977~
    ROUTE_FINDER_DIAL,                    // 1969

    // 1980s
    ROUTE_FINDER_ITERATIVE_DEEPENING,     // 1980
    ROUTE_FINDER_GREEDY_BEST_FIRST,       // 1985
    ROUTE_FINDER_IDA_STAR,                // 1985

    // 1990s
    ROUTE_FINDER_RTA_STAR,                // 1990
    ROUTE_FINDER_SMA_STAR,                // 1991
    ROUTE_FINDER_DSTAR,                   // 1994
    ROUTE_FINDER_FAST_MARCHING,           // 1996
    ROUTE_FINDER_ANT_COLONY,              // 1996
    ROUTE_FINDER_FRINGE_SEARCH,           // 1997

    // 2000s
    ROUTE_FINDER_FOCAL_SEARCH,            // 2001
    ROUTE_FINDER_DSTAR_LITE,              // 2002
    ROUTE_FINDER_LPA_STAR,                // 2004
    ROUTE_FINDER_HPA_STAR,                // 2004
    ROUTE_FINDER_ALT,                     // 2005
    ROUTE_FINDER_ANY_ANGLE_ASTAR,         // 2005~
    ROUTE_FINDER_HCA_STAR,                // 2005
    ROUTE_FINDER_RTAA_STAR,               // 2006
    ROUTE_FINDER_THETA_STAR,              // 2007
    ROUTE_FINDER_CONTRACTION_HIERARCHIES,// 2008

    // 2010s
    ROUTE_FINDER_LAZY_THETA_STAR,         // 2010
    ROUTE_FINDER_JUMP_POINT_SEARCH,       // 2011
    ROUTE_FINDER_SIPP,                    // 2011
    ROUTE_FINDER_JPS_PLUS,                // 2012
    ROUTE_FINDER_EPEA_STAR,               // 2012
    ROUTE_FINDER_MHA_STAR,                // 2012
    ROUTE_FINDER_ANYA,                    // 2013

    // 특수 목적 / 확장형
    ROUTE_FINDER_DAG_SP,                  // 1960s (DAG 최단경로 O(V+E))
    ROUTE_FINDER_MULTI_SOURCE_BFS,        // 2000s (복수 시작점 BFS)
    ROUTE_FINDER_MCTS                     // 2006
} route_finder_type_t;

BYUL_API const char* get_route_finder_name(route_finder_type_t pa);

/** 
 * @brief 정적 길찾기 설정 구조체
 */
typedef struct s_route_finder {
    navgrid_t* navgrid;                        ///< 경로를 탐색할 지도
    route_finder_type_t type;
    coord_t* start;                     ///< 시작 좌표
    coord_t* goal;                      ///< 도착 좌표
    cost_func cost_fn;                ///< 비용 함수
    heuristic_func heuristic_fn;      ///< 휴리스틱 함수
    int max_retry;                    ///< 최대 반복 횟수
    bool visited_logging;             ///< 방문한 노드 로깅 여부
    void* userdata;                   ///< 사용자 정의 데이터
} route_finder_t;

/**
 * @brief 기본 설정으로 route_finder_t 구조체를 생성합니다.
 *
 * 이 함수는 ROUTE_FINDER_ASTAR를 기본 알고리즘으로 설정하고,
 * 다음과 같은 기본값을 포함한 route_finder_t 객체를 생성합니다:
 * - cost 함수: default_cost
 * - 휴리스틱 함수: euclidean_heuristic
 * - 최대 반복 횟수: 10000
 * - 방문 노드 로깅: false
 *
 * @return 초기화된 route_finder_t 포인터 (heap에 생성되며, 사용 후 route_finder_free로 해제해야 함)
 */
BYUL_API route_finder_t* route_finder_new(navgrid_t* navgrid);

BYUL_API route_finder_t* route_finder_new_full(navgrid_t* navgrid, 
    route_finder_type_t type, 
    coord_t* start, coord_t* goal,
    cost_func cost_fn, heuristic_func heuristic_fn,
    int max_retry, bool visited_logging, void* userdata);

BYUL_API void route_finder_free(route_finder_t* a);

BYUL_API route_finder_t* route_finder_copy(const route_finder_t* src);

/**
 * @brief 설정값 세터/게터
 */
BYUL_API void route_finder_set_navgrid(route_finder_t* a, navgrid_t* navgrid);
BYUL_API void route_finder_set_start(route_finder_t* a, const coord_t* start);
BYUL_API void route_finder_set_goal(route_finder_t* a, const coord_t* goal);

BYUL_API navgrid_t* route_finder_get_navgrid(const route_finder_t* a);
BYUL_API coord_t* route_finder_get_start(const route_finder_t* a);
BYUL_API coord_t* route_finder_get_goal(const route_finder_t* a);

BYUL_API void route_finder_set_userdata(route_finder_t* a, void* userdata);
BYUL_API void* route_finder_get_userdata(const route_finder_t* a);

BYUL_API void route_finder_set_type(route_finder_t* a, route_finder_type_t type);
BYUL_API route_finder_type_t route_finder_get_type(const route_finder_t* a);

BYUL_API void route_finder_set_visited_logging(route_finder_t* a, bool is_logging);
BYUL_API bool route_finder_is_visited_logging(route_finder_t* a);

BYUL_API void route_finder_set_cost_func(route_finder_t* a, cost_func cost_fn);
BYUL_API cost_func route_finder_get_cost_func(route_finder_t* a);

BYUL_API void route_finder_set_heuristic_func(route_finder_t* a, heuristic_func heuristic_fn);
BYUL_API heuristic_func route_finder_get_heuristic_func(route_finder_t* a);

BYUL_API void route_finder_set_max_retry(route_finder_t* a, int max_retry);
BYUL_API int route_finder_get_max_retry(route_finder_t* a);

/**
 * @brief 설정값 기본화 및 검증
 */
BYUL_API void route_finder_clear(route_finder_t* a);

/**
 * @brief route_finder_t 구조체의 기본값을 설정합니다.
 *
 * - cost 함수는 default_cost,
 * - 휴리스틱 함수는 euclidean_heuristic,
 * - 최대 반복 횟수는 10000,
 * - visited_logging은 false로 초기화됩니다.
 *
 * @param a 기본값을 설정할 route_finder_t 포인터
 */
BYUL_API void route_finder_set_defaults(route_finder_t* a);

BYUL_API bool route_finder_is_valid(const route_finder_t* a);
BYUL_API void route_finder_print(const route_finder_t* a);

/**
 * @brief 정적 길찾기 실행 (알고리즘 유형 분기 포함)
 */
BYUL_API route_t* route_finder_find_with_type(route_finder_t* a, route_finder_type_t type);

BYUL_API route_t* route_finder_find(route_finder_t* a);

/**
 * @brief 알고리즘별 직접 실행 함수 (정적 길찾기 전용)
 */
BYUL_API route_t* route_finder_find_astar(route_finder_t* a);
BYUL_API route_t* route_finder_find_bfs(route_finder_t* a);
BYUL_API route_t* route_finder_find_dfs(route_finder_t* a);
BYUL_API route_t* route_finder_find_dijkstra(route_finder_t* a);

/**
 * @brief Fringe Search 알고리즘을 실행합니다.
 *
 * 이 알고리즘은 fringe threshold를 넘기며 탐색하는 방식으로,
 * 탐색 효율을 높이기 위해 사용자 정의 매개변수 delta_epsilon을 사용합니다.
 *
 * @param a 실행 설정이 포함된 route_finder_t 포인터
 *          - userdata는 float* 타입이며, fringe 확장 임계값인 
 *                  delta_epsilon을 가리켜야 합니다.
 *          - 추천값: 0.1 ~ 0.5 (기본값 없음, 사용자 설정 필요)
 *
 * @return 계산된 경로(route_t*) 또는 실패 시 NULL
 */
BYUL_API route_t* route_finder_find_fringe_search(route_finder_t* a);

BYUL_API route_t* route_finder_find_greedy_best_first(route_finder_t* a);

BYUL_API route_t* route_finder_find_ida_star(route_finder_t* a);

/**
 * @brief Real-Time A* (RTA*) 알고리즘을 실행합니다.
 *
 * 이 알고리즘은 제한된 탐색 깊이 내에서만 탐색을 진행하고
 * 실시간 반응성을 확보합니다.
 *
 * @param a 실행 설정이 포함된 route_finder_t 포인터
 *          - userdata는 int* 타입이며, 탐색 깊이 제한(depth_limit)을 가리켜야 합니다.
 *          - 추천값: 3 ~ 10 (높을수록 정확하지만 느려짐)
 *
 * @return 계산된 경로(route_t*) 또는 실패 시 NULL
 */
BYUL_API route_t* route_finder_find_rta_star(route_finder_t* a);

/**
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
BYUL_API route_t* route_finder_find_sma_star(route_finder_t* a);

/**
 * @brief Weighted A* 알고리즘을 실행합니다.
 *
 * 이 알고리즘은 A*의 휴리스틱에 가중치를 적용해
 * 더 빠른 경로 계산을 유도합니다.
 *
 * @param a 실행 설정이 포함된 route_finder_t 포인터
 *          - userdata는 float* 타입이며, 휴리스틱 가중치(weight)를 가리켜야 합니다.
 *          - 추천값: 1.0 (기본 A*), 1.2 ~ 2.5 (속도 향상), 5.0 이상은 부정확할 수 있음
 *
 * @return 계산된 경로(route_t*) 또는 실패 시 NULL
 */
BYUL_API route_t* route_finder_find_weighted_astar(route_finder_t* a);

BYUL_API route_t* route_finder_find_fast_marching(route_finder_t* a);

#ifdef __cplusplus
}
#endif

#endif // ROUTE_FINDER_H