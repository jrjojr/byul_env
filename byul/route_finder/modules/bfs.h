#ifndef BFS_H
#define BFS_H

#include "byul_config.h"
#include "internal/coord.h"
#include "internal/map.h"
#include "internal/route.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BFS(너비 우선 탐색)를 이용하여 맵 상의 최단 경로를 탐색합니다.
 *
 * 이 함수는 주어진 맵(@p m)에서 시작 좌표(@p start)에서 목표 좌표(@p goal)까지
 * 최단 거리(이동 횟수 기준)의 경로를 BFS(FIFO 큐) 방식으로 탐색합니다.
 *
 * 내부적으로 방문 좌표 집합과 이동 경로를 추적하며,
 * @p max_retry 회 이상 반복하지 않도록 제한합니다.
 *
 * - 경로를 찾은 경우: start → goal로 이어지는 경로를 반환하고 success가 true입니다.
 * - 경로가 없더라도: 마지막으로 탐색된 좌표까지의 경로를 반환하고 success는 false입니다.
 * - @p visited_logging 가 true이면 모든 방문 좌표에 대해 카운트를 기록합니다.
 *
 * @param m             경로를 탐색할 맵 객체
 * @param start         시작 좌표
 * @param goal          목표 좌표
 * @param max_retry     최대 반복 횟수 (무한 루프 방지를 위한 제한).
 *                      일반적으로 width * height 정도의 값을 권장합니다.
 * @param visited_logging 방문 좌표의 카운트 기록 여부
 * 
 * @return 생성된 @c route_t* 객체.
 *         - @c route_get_success(result) 가 true이면 경로 탐색에 성공한 것입니다.
 *         - 실패한 경우에도 마지막으로 도달한 좌표까지의 경로를 포함합니다.
 */
BYUL_API route_t* find_bfs(const map_t* m, 
    const coord_t* start, const coord_t* goal, 
    int max_retry, bool visited_logging);


#ifdef __cplusplus
}
#endif

#endif // BFS_H
