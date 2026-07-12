from ffi_core import ffi, C

from typing import Any

from route_finder_common import g_RouteFuncReg
from coord import c_coord
from coord_list import c_coord_list
from coord_hash import c_coord_hash

from route import c_route
from navgrid import c_navgrid, NavgridDirMode

from dstar_lite_key import c_dstar_lite_key
from dstar_lite_pqueue import c_dstar_lite_pqueue
from byul_tick import c_tick

import weakref

ffi.cdef("""
typedef void (*move_func)(const coord_t* c, void* userdata);

typedef coord_list_t* (*changed_coords_func)(void* userdata);

typedef struct s_dstar_lite dstar_lite_t;

/**
 * @brief Creates a D* Lite configuration object with default settings.
 *
 * Returns NULL if navgrid is not provided.
 * 
 * This function creates a D* Lite configuration structure 
 * with the following default values:
 * - start : (0, 0)
 * - goal : (0, 0)
 * - km : 0.0f
 * - max_range : 0 (only neighbors of the center coordinate are checked)
 * - real_loop_max_retry : 0 (no repeated searches)
 * - width : 0 (infinite navgrid)
 * - height : 0 (infinite navgrid)
 *
 * 8-directional movement, Euclidean distance, 
 * D* Lite cost, debug mode disabled.
 * The created configuration object is then used by the algorithm.
 *
 * @return Newly created dstar_lite_t* object.
 *         Must be freed using dstar_lite_destroy() after use.
 */
dstar_lite_t* dstar_lite_create(navgrid_t* navgrid);

/**
 * @brief Creates a D* Lite configuration object with user-defined settings.
 *
 * Returns NULL if navgrid is not provided.
 * 
 * @param debug_mode_enabled  Whether to enable debug mode.
 * @return Newly created dstar_lite_t* object.
 *         Must be freed using dstar_lite_destroy() after use.
 */
dstar_lite_t* dstar_lite_create_full(
    navgrid_t* navgrid, 
    const coord_t* start, 
    const coord_t* goal, 
    cost_func cost_fn, 
    heuristic_func heuristic_fn,
    bool debug_mode_enabled);

void dstar_lite_destroy(dstar_lite_t* dsl);

dstar_lite_t* dstar_lite_copy(dstar_lite_t* src);

int dstar_lite_fetch_start(const dstar_lite_t* dsl, coord_t* out);

void  dstar_lite_set_start(dstar_lite_t* dsl, const coord_t* c);

int dstar_lite_fetch_goal(const dstar_lite_t* dsl, coord_t* out);

void  dstar_lite_set_goal(dstar_lite_t* dsl, const coord_t* c);

coord_hash_t* dstar_lite_get_g_table(const dstar_lite_t* dsl);

coord_hash_t* dstar_lite_get_rhs_table(const dstar_lite_t* dsl);

dstar_lite_pqueue_t* dstar_lite_get_frontier(
    const dstar_lite_t* dsl);

void     dstar_lite_set_frontier(
    dstar_lite_t* dsl, dstar_lite_pqueue_t* frontier);

float dstar_lite_get_km(const dstar_lite_t* dsl);
void   dstar_lite_set_km(dstar_lite_t* dsl, float km);

int   dstar_lite_get_max_range(const dstar_lite_t* dsl);
void   dstar_lite_set_max_range(dstar_lite_t* dsl, int value);

// Maximum number of loops inside the find_loop function
// When moving in real-time at 4 km/h, it loops continuously
// for the time equal to interval_sec multiplied by this value.
// This needs improvement; for a 10x10 navgrid, around 100 might be needed.
int   dstar_lite_get_real_loop_max_retry(const dstar_lite_t* dsl);
void  dstar_lite_set_real_loop_max_retry(
    dstar_lite_t* dsl, int value);
int   dstar_lite_real_loop_retry_count(const dstar_lite_t* dsl);

// On a 10x10 navgrid, 100 seems to work well.
int dstar_lite_get_max_retry(const dstar_lite_t* dsl);
void dstar_lite_set_max_retry(
    dstar_lite_t* dsl, int v);

int dstar_lite_proto_compute_retry_count(const dstar_lite_t* dsl);

int dstar_lite_real_compute_retry_count(const dstar_lite_t* dsl);

// When creating proto route_t*, reconstruct_route is used.
// For a 10x10 navgrid, 100 is too much and 10 is too small;
// around 40 seems reasonable.
int dstar_lite_get_reconstruct_max_retry(const dstar_lite_t* dsl);
void dstar_lite_set_reconstruct_max_retry(dstar_lite_t* dsl, int v);
int dstar_lite_reconstruct_retry_count(const dstar_lite_t* dsl);

bool dstar_lite_is_debug_mode_enabled(const dstar_lite_t* dsl);

void     dstar_lite_enable_debug_mode(
    dstar_lite_t* dsl, bool enabled);

coord_hash_t* dstar_lite_get_update_count_table(
    const dstar_lite_t* dsl);

void         dstar_lite_add_update_count(
    dstar_lite_t* dsl, const coord_t* c);

void         dstar_lite_clear_update_count(dstar_lite_t* dsl);

int         dstar_lite_get_update_count(
    dstar_lite_t* dsl, const coord_t* c);

const navgrid_t*    dstar_lite_get_navgrid(const dstar_lite_t* dsl);
void    dstar_lite_set_navgrid(dstar_lite_t* dsl, navgrid_t* navgrid);

const route_t* dstar_lite_get_proto_route(const dstar_lite_t* dsl);

const route_t* dstar_lite_get_real_route(const dstar_lite_t* dsl);

// Resets and initializes settings such as start, goal, and navgrid,
// as well as hash tables and the priority queue.
void dstar_lite_reset(dstar_lite_t* dsl);

float dstar_lite_get_interval_sec(dstar_lite_t* dsl);

void dstar_lite_set_interval_sec(
    dstar_lite_t* dsl, float interval_sec);

// 가장 기본적인 비용 함수
float dstar_lite_cost(
    const navgrid_t* navgrid, 
    const coord_t* start, 
    const coord_t* goal, 
    void* userdata);

/**
 * @brief D* Lite 알고리즘용 동적 비용 함수
 *
 * 두 좌표 간의 유클리드 거리를 기반으로 경로 탐색 비용을 계산합니다.
 * 목표 좌표(goal)가 장애물일 경우, 비용은 FLT_MAX로 설정되어 경로 탐색에서 제외됩니다.
 *
 * userdata를 통해 방문 횟수 기반 페널티나 사용자 정의 데이터를 활용한
 * 비용 가중치 적용해야 한다
 * 이미 방문한 기록을 얻어야 한다 coord_hash_t로 값은 true, false또는 방문 횟수
 * 방문 횟수로 정해야 겠다 여러번 반복해서 방문하는 상황이 분명히 있다.
 * 현재 동적 경로 탐색이 (3,3) (4,4) 무한 반복 (5,5)가 장애물이 추가되었을 때
 * (0,0) ~ (9,9)로 가는 경로를 생성해야 할때 proto_route는 정상 생성된다
 * find_loop(), find_tick으로 동적 생성할때 에...
 *
 * @param[in] navgrid     네비게이션 그리드 정보 (장애물 판정 포함)
 * @param[in] start       시작 좌표
 * @param[in] goal        도착 좌표
 * @param[in] userdata    사용자 정의 데이터 (예: 방문 카운터 해시맵)
 * @return float          거리 기반 비용, 또는 장애물일 경우 FLT_MAX
 *
 * @note
 * 이 함수는 dstar_lite 구조에서 동적으로 등록되며, 
 * `dsl->cost_fn`에 설정하여 사용됩니다.
 * 
 * @see dstar_lite_t, coord_hash_t
 */    
float dstar_lite_dynamic_cost(
    const navgrid_t* navgrid, 
    const coord_t* start, 
    const coord_t* goal, 
    void* userdata);    

cost_func    dstar_lite_get_cost_func(const dstar_lite_t* dsl);
void dstar_lite_set_cost_func(dstar_lite_t* dsl, cost_func fn);

void*    dstar_lite_get_cost_func_userdata(const dstar_lite_t* dsl);

void dstar_lite_set_cost_func_userdata(
    dstar_lite_t* dsl, void* userdata);    

bool dstar_lite_is_blocked(
    dstar_lite_t* dsl, int x, int y, void* userdata);    

is_coord_blocked_func dstar_lite_get_is_blocked_func(
    dstar_lite_t* dsl);

void dstar_lite_set_is_blocked_func(
    dstar_lite_t* dsl, is_coord_blocked_func fn);
void* dstar_lite_get_is_blocked_func_userdata(dstar_lite_t* dsl);
void dstar_lite_set_is_blocked_func_userdata(
    dstar_lite_t* dsl, void* userdata);

float dstar_lite_heuristic(
    const coord_t* start, const coord_t* goal, void* userdata);
heuristic_func dstar_lite_get_heuristic_func(
    const dstar_lite_t* dsl);
void         dstar_lite_set_heuristic_func(
    dstar_lite_t* dsl, heuristic_func func);
void* dstar_lite_get_heuristic_func_userdata(dstar_lite_t* dsl);
void dstar_lite_set_heuristic_func_userdata(
    dstar_lite_t* dsl, void* userdata);    

// move_func 함수 포인터에 대입할 기본형 함수이다.
void move_to(const coord_t* c, void* userdata);

move_func dstar_lite_get_move_func(const dstar_lite_t* dsl);
void dstar_lite_set_move_func(dstar_lite_t* dsl, move_func fn);
void* dstar_lite_get_move_func_userdata(const dstar_lite_t* dsl);
void dstar_lite_set_move_func_userdata(
    dstar_lite_t* dsl, void* userdata);

// changed_coords_func 함수 포인터에 대입할 기본형 함수이다.
// 1개의 coord 좌표를 제공한다. userdata : coord* c
coord_list_t* get_changed_coord(void* userdata);

// changed_coords_func 함수 포인터에 대입할 기본형 함수이다.
// coord_list를 작성해서 여러개의 coord 좌표를 제공한다.
// userdata : coord_list_t* list;
coord_list_t* get_changed_coords(void* userdata);

changed_coords_func dstar_lite_get_changed_coords_func(
    const dstar_lite_t* dsl);
void dstar_lite_set_changed_coords_func(
    dstar_lite_t* dsl, changed_coords_func fn);
void* dstar_lite_get_changed_coords_func_userdata(
    const dstar_lite_t* dsl);
void dstar_lite_set_changed_coords_func_userdata(
    dstar_lite_t* dsl, void* userdata);

/**
 * @brief Priority key calculation function for D* Lite
 *
 * Sets k2 to the smaller value between g[s] and rhs[s],
 * and calculates k1 = k2 + heuristic(start, s) + km.
 *
 * @param dsl D* Lite object
 * @param s   Target coordinate
 * @return Calculated dstar_lite_key_t structure
 */
dstar_lite_key_t* dstar_lite_calc_key(
    dstar_lite_t* dsl, const coord_t* s);

void dstar_lite_init(dstar_lite_t* dsl);

/**
 * @brief Recalculates the rhs value of the given node 
 *      and updates the open list if necessary
 * @param al Algorithm context
 * @param u  Coordinate to update
 */
void dstar_lite_update_vertex(dstar_lite_t* dsl, const coord_t* u);

/**
 * @brief Performs update_vertex() for all coordinates 
 *      within the given range from a center coordinate
 * 
 * @param al         Algorithm context
 * @param s          Center coordinate
 * @param max_range  Range (0 means only the s coordinate is updated)
 */
void dstar_lite_update_vertex_range(dstar_lite_t* dsl, 
    const coord_t* s, int max_range);

/**
 * @brief Executes update_vertex_range() 
 *      based on the max_range defined in the config
 * 
 * @param al Algorithm context
 * @param s  Center coordinate
 */
void dstar_lite_update_vertex_auto_range(
    dstar_lite_t* dsl, const coord_t* s);

/**
 * @brief Computes the shortest route based on the open list
 * 
 * @param al Algorithm context
 */    
void dstar_lite_compute_shortest_route(dstar_lite_t* dsl);

/**
 * @brief Reconstructs the route between two coordinates.
 *
 * When the condition g ~= rhs is satisfied, 
 * the actual route is extracted and returned.
 * If the condition is not satisfied, NULL is returned.
 *
 * @param dsl Algorithm context
 * @return route_t* A valid route object, or NULL if it fails
 */
route_t* dstar_lite_reconstruct_route(dstar_lite_t* dsl);

// One-time pathfinding in its simplest form, 
// equivalent to static pathfinding.
route_t* dstar_lite_find(dstar_lite_t* dsl);

// Integrated pathfinding, combining find_proto and find_loop.
void dstar_lite_find_full(dstar_lite_t* dsl);

// Generates an initial route for dynamic pathfinding.
void dstar_lite_find_proto(dstar_lite_t* dsl);

// Using the initial route created by dstar_lite_find_proto, 
// this function searches for a dynamic route.
// dstar_lite_find_proto must be executed first.
// Otherwise, pathfinding will fail.
// If asked why find_proto must be called separately, 
// the answer is that this is for advanced users.
// Some users may want to perform certain operations 
// between find_proto and find_loop.
// If asked why callbacks are not used, 
// the answer is that refactoring for that is undesirable for now.
// Callback support may be added later.
// If you simply want a dynamic route, use dstar_lite_find_full().
void dstar_lite_find_loop(dstar_lite_t* dsl);

/**
 * @brief Executes update_vertex for all coordinates 
 * included in the given route.
 *
 * @param al Algorithm context
 * @param p  Route to update
 */
void dstar_lite_update_vertex_by_route(
    dstar_lite_t* dsl, route_t* p);

// Forcefully terminates the loop.
void dstar_lite_force_quit(dstar_lite_t* dsl);

// Checks if a forced termination has been requested.
bool dstar_lite_is_quit_forced(dstar_lite_t* dsl);

void dstar_lite_set_force_quit(dstar_lite_t* dsl, bool v);

// byul_tick에 dstar_lite_find_tickv 부착한다. 기본 설정 완료한다.
void dstar_lite_find_tick_prepare(dstar_lite_t* dsl, tick_t* tk);

void dstar_lite_find_tick(dstar_lite_t* dsl, float dt);

// byul_tick에서 dstar_lite_find_tick 제거한다. 정리한다.
void dstar_lite_find_tick_complete(dstar_lite_t* dsl, tick_t* tk);
""", override=True)

class c_dstar_lite:
    def __init__(self, 
                 m: c_navgrid = None, 
                 start: c_coord = None,
                 goal: c_coord = None,
                 cost_fn=None, 
                 heuristic_fn=None, 
                 debug=False, 
                 raw_ptr=None, 
                 own=False):

        if raw_ptr:
            self._c = raw_ptr
            self._own = own
        else:
            if m is None:
                raise ValueError("navgrid (m) must be provided")

            if start and goal:
                self._c = C.dstar_lite_create_full(
                    m.ptr(),
                    start.ptr(),
                    goal.ptr(),
                    cost_fn if cost_fn else ffi.NULL,
                    heuristic_fn if heuristic_fn else ffi.NULL,
                    debug
                )
            else:
                self._c = C.dstar_lite_create(m.ptr())

            self._own = True

        if not self._c:
            raise MemoryError("dstar_lite allocation failed")

        if self._own:
            self._finalizer = weakref.finalize(
                self, C.dstar_lite_destroy, self._c
            )
        else:
            self._finalizer = None


    def ptr(self):
        return self._c

       # ───── Start/Goal ─────
    def get_start(self) -> c_coord:
        out = ffi.new("coord_t*")
        if C.dstar_lite_fetch_start(self._c, out) == 0:
            return c_coord(raw_ptr=out)
        else:
            raise RuntimeError("Failed to get start coordinate")

    def set_start(self, coord: c_coord):
        C.dstar_lite_set_start(self._c, coord.ptr())

    def get_goal(self) -> c_coord:
        out = ffi.new("coord_t*")
        if C.dstar_lite_fetch_goal(self._c, out) == 0:
            return c_coord(raw_ptr=out)
        else:
            raise RuntimeError("Failed to get goal coordinate")

    def set_goal(self, coord: c_coord):
        C.dstar_lite_set_goal(self._c, coord.ptr())

    # ───── 테이블 접근 ─────
    def g_table(self):
        return c_coord_hash(
            raw_ptr=C.dstar_lite_get_g_table(self._c), own=False)

    def rhs_table(self):
        return c_coord_hash(
            raw_ptr=C.dstar_lite_get_rhs_table(self._c), own=False)

    def frontier(self):
        return c_dstar_lite_pqueue(
            raw_ptr=C.dstar_lite_get_frontier(self._c), own=False)

    def set_frontier(self, frontier: c_dstar_lite_pqueue):
        C.dstar_lite_set_frontier(self._c, frontier.ptr())

    # ───── 설정값들 ─────
    def get_km(self):
        return C.dstar_lite_get_km(self._c)

    def set_km(self, v):
        C.dstar_lite_set_km(self._c, v)

    def get_max_range(self):
        return C.dstar_lite_get_max_range(self._c)

    def set_max_range(self, v):
        C.dstar_lite_set_max_range(self._c, v)

    @property
    def real_loop_max_retry(self):
        # gint   dstar_lite_get_real_loop_max_retry(const dstar_lite dsl);
        return C.dstar_lite_get_real_loop_max_retry(self.ptr())

    @real_loop_max_retry.setter
    def real_loop_max_retry(self, value:int):
        # void   dstar_lite_set_real_loop_max_retry(
        #     dstar_lite dsl, gint value);
        C.dstar_lite_set_real_loop_max_retry(self.ptr(), value)

    @property
    def compute_max_retry(self):
        # // 10x10의 맵에서 100은 되어야 잘 찾는거 같다.
        # gint dstar_lite_get_compute_max_retry(const dstar_lite dsl);
        return C.dstar_lite_get_compute_max_retry(self.ptr())

    @compute_max_retry.setter
    def compute_max_retry(self, value:int):
        # void dstar_lite_set_compute_max_retry(
        #     const dstar_lite dsl, gint v);
        C.dstar_lite_set_compute_max_retry(self.ptr(), value)

    @property
    def reconstruct_max_retry(self):
        # // proto route 생성할때 reconstruct_route한다. 여기에 사용하는 루프
        # // 10x10에서 100은 오버고 10은 너무 작고 대충 40 정도면 되겠다.
        # gint dstar_lite_get_reconstruct_max_retry(const dstar_lite dsl);
        return C.dstar_lite_get_reconstruct_max_retry(self.ptr())

    @reconstruct_max_retry.setter
    def reconstruct_max_retry(self, value:int):
        # void dstar_lite_set_reconstruct_max_retry(
        #     const dstar_lite dsl, gint v);
        C.dstar_lite_set_reconstruct_max_retry(self.ptr(), value)

    
    @property
    def proto_compute_retry_count(self):
        # gint dstar_lite_proto_compute_retry_count(dstar_lite dsl);
        return C.dstar_lite_proto_compute_retry_count(self.ptr())

    @property
    def real_compute_retry_count(self):
        # gint dstar_lite_real_compute_retry_count(dstar_lite dsl);
        return C.dstar_lite_real_compute_retry_count(self.ptr())

    @property
    def real_loop_retry_count(self):
        # gint dstar_lite_real_loop_retry_count(dstar_lite dsl);
        return C.dstar_lite_real_loop_retry_count(self.ptr())

    @property
    def reconstruct_retry_count(self):
        # gint dstar_lite_reconstruct_retry_count(dstar_lite dsl);         
        return C.dstar_lite_reconstruct_retry_count(self.ptr())


    def set_interval_sec(self, v):
        C.dstar_lite_set_interval_sec(self._c, v)

    def get_interval_sec(self):
        return C.dstar_lite_get_interval_sec(self._c)

    def set_debug_mode_enabled(self, v: bool):
        C.dstar_lite_enable_debug_mode(self._c, v)

    def is_debug_mode_enabled(self):
        return bool(C.dstar_lite_get_debug_mode_enabled(self._c))

    # ───── 경로 ─────
    def get_proto_route(self):
        return c_route(
            raw_ptr=C.dstar_lite_get_proto_route(self._c), own=False)

    def get_real_route(self):
        return c_route(
            raw_ptr=C.dstar_lite_get_real_route(self._c), own=False)

    # ───── 경로 탐색 함수들 ─────
    def find(self):
        ptr = C.dstar_lite_find(self._c)
        return c_route(raw_ptr=ptr, own=True) if ptr != ffi.NULL else None

    def find_proto(self):
        C.dstar_lite_find_proto(self._c)

    def find_loop(self):
        C.dstar_lite_find_loop(self._c)

    def find_full(self):
        C.dstar_lite_find_full(self._c)

    # ───── 유틸 함수 ─────
    def init(self):
        C.dstar_lite_init(self._c)

    def reset(self):
        C.dstar_lite_reset(self._c)

    def compute_shortest_route(self):
        C.dstar_lite_compute_shortest_route(self._c)

    def reconstruct_route(self):
        ptr = C.dstar_lite_reconstruct_route(self._c)
        return c_route(raw_ptr=ptr, own=True) if ptr != ffi.NULL else None

    def calculate_key(self, s: c_coord):
        ptr = C.dstar_lite_calculate_key(self._c, s.ptr())
        return c_dstar_lite_key(
            raw_ptr=ptr, own=True) if ptr != ffi.NULL else None

    def update_vertex(self, coord: c_coord):
        C.dstar_lite_update_vertex(self._c, coord.ptr())

    def update_vertex_range(self, center: c_coord, range_val: int):
        C.dstar_lite_update_vertex_range(self._c, center.ptr(), range_val)

    def update_vertex_auto_range(self, center: c_coord):
        C.dstar_lite_update_vertex_auto_range(self._c, center.ptr())

    def update_vertex_by_route(self, route: c_route):
        C.dstar_lite_update_vertex_by_route(self._c, route.ptr())

    # ───── 상태 통계 및 종료 ─────
    def is_quit_forced(self):
        return bool(C.dstar_lite_is_quit_forced(self._c))

    def set_force_quit(self, v: bool):
        C.dstar_lite_set_force_quit(self._c, v)

    def force_quit(self):
        C.dstar_lite_force_quit(self._c)

    def update_count_table(self):
        return c_coord_hash(
            raw_ptr=C.dstar_lite_get_update_count_table(self._c), own=False)

    def add_update_count(self, coord: c_coord):
        C.dstar_lite_add_update_count(self._c, coord.ptr())

    def get_update_count(self, coord: c_coord):
        return C.dstar_lite_get_update_count(self._c, coord.ptr())

    def clear_update_count(self):
        C.dstar_lite_clear_update_count(self._c)

    def navgrid(self):
        return c_navgrid(raw_ptr=C.dstar_lite_get_navgrid(self._c), own=False)

    def set_map(self, m: c_navgrid):
        C.dstar_lite_set_map(self._c, m.ptr())

    # ───── 메모리 관리 ─────
    def __del__(self):
        if self._own and self._finalizer and self._finalizer.alive:
            self._finalizer()

    def close(self):
        if self._own and self._finalizer and self._finalizer.alive:
            self._finalizer()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def __repr__(self):
        return f'''c_dstar_lite(
start={self.get_start()}, goal={self.get_goal()}, km={self.get_km():.2f})'''

    # ───── 비용 함수 (cost_fn) 등록 ─────
    def set_cost_func(self, py_func: callable):
        self._py_cost_func = py_func
        # self._cost_func_userdata = userdata
        # self._ffi_cost_func_userdata = ffi.new_handle(userdata)
        # self._ffi_cost_func_userdata = (
        #     ffi.new_handle(userdata) if userdata is not None else ffi.NULL
        # )        

        @ffi.callback(
                "float(const navgrid_t*, const coord_t*, const coord_t*, void*)")
        def _wrapped(m_ptr, s_ptr, g_ptr, udata_ptr):
            map_obj = c_navgrid(raw_ptr=m_ptr, own=False)
            start = c_coord(raw_ptr=s_ptr)
            goal = c_coord(raw_ptr=g_ptr)
            # user = ffi.from_handle(udata_ptr)
            # This wrapper does not register a Python handle as userdata.
            # Treat the native userdata value as opaque; attempting to pass it
            # to ffi.from_handle() can abort Python if C returns a stale or
            # non-Python pointer.
            user = None
            return py_func(map_obj, start, goal, user)

        self._ffi_cost_func = _wrapped
        C.dstar_lite_set_cost_func(self._c, _wrapped)
        # C.dstar_lite_set_cost_func_userdata(
        #     self._c, self._ffi_cost_func_userdata)
        


    # ───── 휴리스틱 함수 (heuristic_fn) 등록 ─────
    def set_heuristic_func(self, py_func: callable):
        self._py_heuristic_func = py_func
        # self._heuristic_func_userdata = userdata
        # self._ffi_heuristic_func_userdata = ffi.new_handle(userdata)
        # self._ffi_heuristic_func_userdata = (
        #     ffi.new_handle(userdata) if userdata is not None else ffi.NULL
        # )                

        @ffi.callback("float(const coord_t*, const coord_t*, void*)")
        def _wrapped(s_ptr, g_ptr, udata_ptr):
            start = c_coord(raw_ptr=s_ptr)
            goal = c_coord(raw_ptr=g_ptr)
            user = None
            return py_func(start, goal, user)

        self._ffi_heuristic_func = _wrapped
        C.dstar_lite_set_heuristic_func(self._c, _wrapped)
        # C.dstar_lite_set_heuristic_func_userdata(
        #     self._c, self._ffi_heuristic_func_userdata)

    # ───── 장애물 여부 함수 (is_blocked_fn) 등록 ─────
    def set_is_blocked_func(self, py_func: callable):
        self._py_is_blocked_func = py_func
        # self._is_blocked_func_userdata = userdata
        # self._ffi_is_blocked_func_userdata = ffi.new_handle(userdata)
        # self._ffi_is_blocked_func_userdata = (
        #     ffi.new_handle(userdata) if userdata is not None else ffi.NULL
        # )

        @ffi.callback("bool(const navgrid_t*, int, int, void*)")
        def _wrapped(m_ptr, x, y, udata_ptr):
            map_obj = c_navgrid(raw_ptr=m_ptr, own=False)
            user = None
            return bool(py_func(map_obj, x, y, user))

        self._ffi_is_blocked_func = _wrapped
        C.dstar_lite_set_is_blocked_func(self._c, _wrapped)
        # C.dstar_lite_set_is_blocked_func_userdata(
        #     self._c, self._ffi_is_blocked_func_userdata)

    # ───── 이동 콜백 함수 (move_func) 등록 ─────
    def set_move_func(self, py_func: callable):
        self._py_move_func = py_func

        @ffi.callback("void(const coord_t*, void*)")
        def _wrapped(c_ptr, udata_ptr):
            coord = c_coord(raw_ptr=c_ptr)
            user = None
            py_func(coord, user)

        self._ffi_move_func = _wrapped
        C.dstar_lite_set_move_func(self._c, _wrapped)
        # C.dstar_lite_set_move_func_userdata(
        #     self._c, self._ffi_move_func_userdata)

    # ───── 동적 장애물 갱신 함수 (changed_coords_func) 등록 ─────
    def set_changed_coords_func(self, py_func: callable):
        self._py_changed_coords_func = py_func
        # self._changed_coords_func_userdata = userdata
        # self._ffi_changed_coords_func_userdata = ffi.new_handle(userdata)
        # self._ffi_changed_coords_func_userdata = (
        #     ffi.new_handle(userdata) if userdata is not None else ffi.NULL
        # )

        @ffi.callback("coord_list_t*(void*)")
        def _wrapped(udata_ptr):
            user = None
            clist = py_func(user)
            if isinstance(clist, c_coord_list):
                return clist.ptr()
            return ffi.NULL

        self._ffi_changed_coords_func = _wrapped
        C.dstar_lite_set_changed_coords_func(self._c, _wrapped)
        # C.dstar_lite_set_changed_coords_func_userdata(
        #     self._c, self._ffi_changed_coords_func_userdata)

if __name__ == '__main__':
    navgrid = c_navgrid(width=10, height=10, mode=NavgridDirMode.DIR_8.value)
    finder = c_dstar_lite(navgrid)
    finder.set_debug_mode_enabled(True)

    # 1️⃣ 초기 경로
    print("\n▶ 최초 경로:")
    start = c_coord(0,0)
    goal = c_coord(9,9)
    finder.set_start(start)
    finder.set_goal(goal)

    result = finder.find()
    result.print()

    result.close()    

g_RouteFuncReg.register_cost("dstar_lite", C.dstar_lite_cost)

g_RouteFuncReg.register_heuristic("dstar_lite", C.dstar_lite_heuristic)
