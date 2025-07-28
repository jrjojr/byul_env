#ifndef CONSOLE_H
#define CONSOLE_H

#include "byul_common.h"
#include "float_common.h"

#include "navgrid.h"
#include "coord.h"
#include "route.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 맵을 ASCII 형태로 출력합니다.
 *
 * 차단된 좌표는 `#`, 그 외는 `.`으로 출력되며,
 * 경로, 시작/도착 지점은 표시되지 않습니다.
 *
 * @param m 출력할 맵
 */
BYUL_API void navgrid_print_ascii(const navgrid_t* m);

/**
 * @brief 경로 정보를 포함하여 맵을 ASCII로 출력합니다.
 *
 * 경로는 `*`, 시작점은 `S`, 도착점은 `E`로 표시되며,
 * 차단된 좌표는 `#`, 나머지는 `.`으로 출력됩니다.
 *
 * @param m 맵 객체
 * @param p 경로 객체 (route_get_success(p)가 TRUE인 경우만 표시)
 */
BYUL_API void navgrid_print_ascii_with_route(
    const navgrid_t* m, const route_t* p, int margin);

/**
 * @brief 방문 횟수를 ASCII 맵 형식으로 출력합니다.
 *
 * route_t* 내부의 visited_logging 해시테이블을 기반으로,
 * 각 좌표의 방문 횟수를 1~99 사이의 2자리 숫자로 출력합니다.
 *
 * 출력 형식:
 * - 시작 좌표는 " S"
 * - 도착 좌표는 " E"
 * - 장애물은 "#" (1자리)
 * - 방문된 좌표는 "%2d" (두 자리 숫자, 99 이상은 99로 고정)
 * - 미방문 좌표는 " ."
 *
 * @param m    맵 객체
 * @param p    경로 결과 객체 (route_get_visited_count() 사용)
 */
BYUL_API void navgrid_print_ascii_with_visited_count(
    const navgrid_t* m, const route_t* p, int margin);

#ifdef __cplusplus
}
#endif

#endif // CONSOLE_H
