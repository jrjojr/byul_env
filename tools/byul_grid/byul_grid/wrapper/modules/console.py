from ffi_core import ffi, C
from pathlib import Path
import os
import platform

from coord import c_coord
from coord_list import c_coord_list
from coord_hash import c_coord_hash
from route import c_route
from navgrid import c_navgrid

from dstar_lite_pqueue import c_dstar_lite_pqueue

from dstar_lite import c_dstar_lite

ffi.cdef("""
/**
 * @brief 맵을 ASCII 형태로 출력합니다.
 *
 * 차단된 좌표는 `#`, 그 외는 `.`으로 출력되며,
 * 경로, 시작/도착 지점은 표시되지 않습니다.
 *
 * @param m 출력할 맵
 */
void navgrid_print_ascii(const navgrid_t* m);

/**
 * @brief 경로 정보를 포함하여 맵을 ASCII로 출력합니다.
 *
 * 경로는 `*`, 시작점은 `S`, 도착점은 `E`로 표시되며,
 * 차단된 좌표는 `#`, 나머지는 `.`으로 출력됩니다.
 *
 * @param m 맵 객체
 * @param p 경로 객체 (route_get_success(p)가 TRUE인 경우만 표시)
 */
void navgrid_print_ascii_with_route(
    const navgrid_t* m, const route_t* p, int margin);

/**
 * @brief 방문 횟수를 ASCII 맵 형식으로 출력합니다.
 *
 * route_t* 내부의 debug_mode 해시테이블을 기반으로,
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
void navgrid_print_ascii_with_visited_count(
    const navgrid_t* m, const route_t* p, int margin);
         
// ------------------ Debug Table Output ------------------

/// @brief Print g table (g values per coordinate)
void dsl_debug_print_g_table(const navgrid_t* m, coord_hash_t* g_table);

/// @brief Print rhs table (rhs values per coordinate)
void dsl_debug_print_rhs_table(
    const navgrid_t* m, coord_hash_t* rhs_table);

// ------------------ D* Lite Full State Output ------------------

/// @brief Print all internal states of D* Lite.
/// @param dsl               D* Lite object
/// @param goal              Goal coordinate
/// @param km                Current km value
/// @param g_table           g value table
/// @param rhs_table         rhs value table
/// @param frontier          Priority queue
/// @param max_range         Maximum search range
/// @param retry_limit       Maximum retry count
/// @param debug_mode        Debug mode flag
/// @param update_counter    update_vertex count table
void dsl_debug_print_full_state(
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

// ------------------ Simplified Output ------------------

/// @brief Print only core variables (g/rhs/priority queue)
void dsl_debug_print_state(
    const dstar_lite_t* dsl,
    const coord_t* goal,
    float km,
    coord_hash_t* g_table,
    coord_hash_t* rhs_table,
    dstar_lite_pqueue_t* frontier);

// ------------------ ASCII Map Output ------------------

void dsl_print_info(const dstar_lite_t* dsl);

/// @brief Print only the navgrid (# and .)
void dsl_print_ascii_only_navgrid(const dstar_lite_t* dsl);

/// @brief Print navgrid including start, goal, and path (S, G, *, ., #)
void dsl_print_ascii_route(
    const dstar_lite_t* dsl, const route_t* route, int margin);

/// @brief Print navgrid including update_vertex count (S, G, *, #, numbers)
void dsl_print_ascii_update_count(
    const dstar_lite_t* dsl, route_t* route, int margin);

         

""", override=True)

class c_console:
    # ───── g / rhs 테이블 출력 ─────
    @staticmethod
    def print_g_table(m: c_navgrid, g_table: c_coord_hash):
        C.dsl_debug_print_g_table(m.ptr(), g_table.ptr())

    @staticmethod
    def print_rhs_table(m: c_navgrid, rhs_table: c_coord_hash):
        C.dsl_debug_print_rhs_table(m.ptr(), rhs_table.ptr())

    # ───── D* Lite 전체 상태 출력 ─────
    @staticmethod
    def print_full_state(
        dsl: c_dstar_lite, goal: c_coord, km: float,
        g_table: c_coord_hash, rhs_table: c_coord_hash,
        frontier: c_dstar_lite_pqueue, max_range: int, retry_limit: int,
        debug_mode: bool, update_counter: c_coord_hash
    ):
        C.dsl_debug_print_full_state(
            dsl.ptr(), goal.ptr(), km,
            g_table.ptr(), rhs_table.ptr(),
            frontier.ptr(), max_range, retry_limit,
            debug_mode, update_counter.ptr()
        )

    @staticmethod
    def print_state(
        dsl: c_dstar_lite, goal: c_coord, km: float,
        g_table: c_coord_hash, rhs_table: c_coord_hash,
        frontier: c_dstar_lite_pqueue
    ):
        C.dsl_debug_print_state(
            dsl.ptr(), goal.ptr(), km,
            g_table.ptr(), rhs_table.ptr(), frontier.ptr()
        )

    # ───── D* Lite 기본 정보 출력 ─────
    @staticmethod
    def print_info(dsl: c_dstar_lite):
        C.dsl_print_info(dsl.ptr())

    # ───── 맵 ASCII 출력 (경로 없음) ─────
    @staticmethod
    def print_ascii_only_map(dsl: c_dstar_lite):
        C.dsl_print_ascii_only_navgrid(dsl.ptr())

    # ───── 맵 + 경로 출력 ─────
    @staticmethod
    def print_ascii_route(dsl: c_dstar_lite, route: c_route, margin: int = 0):
        C.dsl_print_ascii_route(dsl.ptr(), route.ptr(), margin)

    # ───── 맵 + 방문 횟수 출력 ─────
    @staticmethod
    def print_ascii_update_count(dsl: c_dstar_lite, route: c_route, margin: int = 0):
        C.dsl_print_ascii_update_count(dsl.ptr(), route.ptr(), margin)

    # ───── 단독 맵 출력 (navgrid용) ─────
    @staticmethod
    def print_ascii_navgrid_only(m):
        C.navgrid_print_ascii(m.ptr())

    # ───── 경로 포함 출력 ─────
    @staticmethod
    def print_ascii_with_route(m : c_navgrid, route: c_route, margin: int = 0):
        C.navgrid_print_ascii_with_route(m.ptr(), route.ptr(), margin)

    # ───── 방문횟수 포함 출력 ─────
    @staticmethod
    def print_ascii_with_visited_count(m : c_navgrid, route: c_route, margin: int = 0):
        C.navgrid_print_ascii_with_visited_count(m.ptr(), route.ptr(), margin)
