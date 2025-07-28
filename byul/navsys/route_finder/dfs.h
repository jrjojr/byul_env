#ifndef DFS_H
#define DFS_H

#include "byul_common.h"
#include "float_common.h"
#include "coord.h"
#include "navgrid.h"
#include "route.h"
#include "route_finder_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief DFS(깊이 우선 탐색)를 이용해 맵 상의 경로를 탐색합니다.
 *
 * 이 함수는 후입선출(LIFO) 방식의 스택 기반 DFS 알고리즘을 사용하여,
 * 주어진 맵(@p m)에서 시작 좌표(@p start)부터 목표 좌표(@p goal)까지의 경로를 탐색합니다.
 *
 * DFS는 가능한 한 깊이 우선으로 탐색한 뒤, 더 이상 진행할 수 없을 경우
 * 되돌아가는(backtracking) 방식으로 경로를 구성합니다.
 * 가중치나 휴리스틱을 고려하지 않기 때문에 항상 최적 경로를 보장하지는 않지만,
 * 간단하고 빠른 탐색에 유용합니다.
 *
 * - 경로를 찾은 경우: start → goal로 이어지는 경로를 반환하고 success가 true입니다.
 * - 경로가 없더라도: 마지막으로 탐색된 좌표까지의 경로를 반환하며 success는 false입니다.
 * - @p visited_logging 가 true이면 방문한 좌표들의 카운트를 기록합니다.
 *
 * @param m             탐색 대상 맵 객체
 * @param start         시작 좌표
 * @param goal          목표 좌표
 * @param max_retry     최대 반복 횟수 (무한 루프 방지를 위한 제한).
 *                      일반적으로 width * height 정도의 값을 권장합니다.
 * @param visited_logging 방문 좌표의 카운트 기록 여부
 * 
 * @return 생성된 @c route_t* 객체.  
 *         - @c route_get_success(result) 가 true이면 목표까지 도달한 경로입니다.
 *         - false이면 탐색이 실패했지만 마지막 탐색 지점까지의 경로가 포함됩니다.
 */
BYUL_API route_t* find_dfs(const navgrid_t* m, 
    const coord_t* start, const coord_t* goal, 
    int max_retry, bool visited_logging);


#ifdef __cplusplus
}
#endif

#endif // DFS_H
