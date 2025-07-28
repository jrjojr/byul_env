#ifndef DSTAR_LITE_UTILS_H
#define DSTAR_LITE_UTILS_H

#include "byul_common.h"
#include "float_common.h"
#include "navgrid.h"
#include "coord.h"
#include "route.h"
#include "dstar_lite.h"
#include "coord_hash.h"
#include "dstar_lite_pqueue.h"

#ifdef __cplusplus
extern "C" {
#endif

// ------------------ 디버그용 테이블 출력 ------------------

/// @brief g 테이블 출력 (좌표별 g값)
BYUL_API void dsl_debug_print_g_table(const navgrid_t* m, coord_hash_t* g_table);

/// @brief rhs 테이블 출력 (좌표별 rhs값)
BYUL_API void dsl_debug_print_rhs_table(
    const navgrid_t* m, coord_hash_t* rhs_table);

// ------------------ D* Lite 상태 전체 출력 ------------------

/// @brief D* Lite 전체 내부 상태를 출력합니다.
/// @param dsl               D* Lite 객체
/// @param goal             목표 지점
/// @param km               현재 km 값
/// @param g_table          g값 테이블
/// @param rhs_table        rhs값 테이블
/// @param frontier         우선순위 큐
/// @param max_range        탐색 최대 범위
/// @param retry_limit      최대 재시도 횟수
/// @param debug_mode       디버그 모드 여부
/// @param update_counter   update_vertex 횟수 테이블
BYUL_API void dsl_debug_print_full_state(
    const dstar_lite_t* dsl,
    const coord_t* goal,
    float km,
    coord_hash_t* g_table,
    coord_hash_t* rhs_table,
    dstar_lite_pqueue_t* frontier,
    int max_range,
    int retry_limit,
    bool debug_mode,
    coord_hash_t* update_counter);

// ------------------ 간략 출력 ------------------

/// @brief 핵심 변수만 간략히 출력 (g/rhs/priority queue만)
BYUL_API void dsl_debug_print_state(
    const dstar_lite_t* dsl,
    const coord_t* goal,
    float km,
    coord_hash_t* g_table,
    coord_hash_t* rhs_table,
    dstar_lite_pqueue_t* frontier);

// ------------------ ASCII 맵 출력 ------------------

BYUL_API void dsl_print_info(const dstar_lite_t* dsl);

/// @brief 맵만 출력 (`#`, `.`만 표시)
BYUL_API void dsl_print_ascii_only_navgrid(const dstar_lite_t* dsl);

/// @brief 시작, 목표, 경로까지 포함한 맵 출력 (`S`, `G`, `*`, `.`, `#`)
BYUL_API void dsl_print_ascii_route(
    const dstar_lite_t* dsl, const route_t* route, int margin);

/// @brief update_vertex 횟수 포함한 맵 출력 (`S`, `G`, `*`, `#`, 숫자 등)
BYUL_API void dsl_print_ascii_update_count(
    const dstar_lite_t* dsl, route_t* route, int margin);

#ifdef __cplusplus
}
#endif

#endif // DSTAR_LITE_UTILS_H
