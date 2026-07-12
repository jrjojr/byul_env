from ffi_core import ffi, C

from coord import c_coord
from coord_list import c_coord_list
from coord_hash import c_coord_hash

from route import c_route
from navgrid import c_navgrid
from route_finder_common import g_RouteFuncReg

import weakref

ffi.cdef("""

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
    ROUTE_FINDER_CONTRACTION_HIERARCHIES, // 2008

    // 2010s
    ROUTE_FINDER_LAZY_THETA_STAR,         // 2010
    ROUTE_FINDER_JUMP_POINT_SEARCH,       // 2011
    ROUTE_FINDER_SIPP,                    // 2011
    ROUTE_FINDER_JPS_PLUS,                // 2012
    ROUTE_FINDER_EPEA_STAR,               // 2012
    ROUTE_FINDER_MHA_STAR,                // 2012
    ROUTE_FINDER_ANYA,                    // 2013

    // Special Purpose / Extended
    ROUTE_FINDER_DAG_SP,                  // 1960s (DAG shortest path O(V+E))
    ROUTE_FINDER_MULTI_SOURCE_BFS,        // 2000s (multi-source BFS)
    ROUTE_FINDER_MCTS                     // 2006
} route_finder_type_t;

const char* get_route_finder_name(route_finder_type_t pa);

/** 
 * @brief Static pathfinding configuration structure.
 */
typedef struct s_route_finder route_finder_t;

/**
 * @brief Creates a route_finder_t structure with default settings.
 *
 * This function sets ROUTE_FINDER_ASTAR as the default algorithm,
 * and creates a route_finder_t object with the following defaults:
 * - cost function: default_cost
 * - heuristic function: euclidean_heuristic
 * - max_retry: MAX_RETRY
 * - debug_mode_enabled: false
 * - typedata: 특정 길찾기 알고리즘 전용 데이타 nullptr기본값
 *
 * @return A pointer to the initialized route_finder_t (allocated on heap).
 *         Must be freed using route_finder_destroy.
 */
route_finder_t* route_finder_create(navgrid_t* navgrid);

route_finder_t* route_finder_create_full(
    navgrid_t* navgrid, 
    const coord_t* start, 
    const coord_t* goal,
    
    route_finder_type_t type,
    void* typedata,

    int max_retry, 
    bool debug_mode_enabled,

    cost_func cost_fn,
    void* cost_fn_userdata,

    heuristic_func heuristic_fn,
    void* heuristic_fn_userdata
);

int route_finder_init(route_finder_t* out, navgrid_t* navgrid);

int route_finder_init_full(
    route_finder_t* out, 
    navgrid_t* navgrid, 
    const coord_t* start, 
    const coord_t* goal,
    route_finder_type_t type, 
    void* typedata,    
    int max_retry, 
    bool debug_mode_enabled,
    
    cost_func cost_fn,
    void* cost_fn_userdata,

    heuristic_func heuristic_fn,
    void* heuristic_fn_userdata    
);

int route_finder_free(route_finder_t* out);

int route_finder_destroy(route_finder_t* a);

route_finder_t* route_finder_copy(const route_finder_t* src);

/**
 * @brief Getters/Setters for route_finder_t configuration.
 */
void route_finder_set_navgrid(route_finder_t* a, navgrid_t* navgrid);
void route_finder_set_start(route_finder_t* a, const coord_t* start);
void route_finder_set_goal(route_finder_t* a, const coord_t* goal);

const navgrid_t* route_finder_get_navgrid(const route_finder_t* a);
int route_finder_fetch_start(const route_finder_t* a, coord_t* out);
int route_finder_fetch_goal(const route_finder_t* a, coord_t* out);

void route_finder_set_type(
    route_finder_t* a, route_finder_type_t type);

route_finder_type_t route_finder_get_type(const route_finder_t* a);

void route_finder_set_typedata(
    route_finder_t* a, void* typedata);

void* route_finder_get_typedata(const route_finder_t* a);    

void route_finder_set_max_retry(route_finder_t* a, int max_retry);
int route_finder_get_max_retry(route_finder_t* a);

void route_finder_enable_debug_mode(
    route_finder_t* a, bool is_logging);

bool route_finder_is_debug_mode_enabled(route_finder_t* a);

void route_finder_set_cost_func(
    route_finder_t* a, cost_func cost_fn);

cost_func route_finder_get_cost_func(route_finder_t* a);

void route_finder_set_cost_fn_userdata(
    route_finder_t* a, void* cost_fn_userdata);

void* route_finder_get_cost_fn_userdata(const route_finder_t* a);

void route_finder_set_heuristic_func(
    route_finder_t* a, heuristic_func heuristic_fn);

heuristic_func route_finder_get_heuristic_func(route_finder_t* a);

void route_finder_set_heuristic_fn_userdata(
    route_finder_t* a, void* heuristic_fn_userdata);

void* route_finder_get_heuristic_fn_userdata(
    const route_finder_t* a);

/**
 * @brief Resets and validates the configuration.
 */
void route_finder_clear(route_finder_t* a);

/**
 * @brief Sets the default values for a route_finder_t structure.
 *
 * - cost function: default_cost
 * - heuristic function: euclidean_heuristic
 * - max_retry: 10000
 * - debug_mode_enabled: false
 *
 * @param a Pointer to the route_finder_t to initialize.
 */
void route_finder_set_defaults(route_finder_t* a);

bool route_finder_is_valid(const route_finder_t* a);
void route_finder_print(const route_finder_t* a);

/**
 * @brief Direct run functions for specific algorithms.
 */
route_t* route_finder_run(route_finder_t* a);

""")

MAX_RETRY = 1000

from enum import IntEnum

class RouteFinderType(IntEnum):
    UNKNOWN = 0

    # // 1950s~1960s
    BELLMAN_FORD = 1 #            // 1958
    DFS = 2 #                     // 1959
    BFS = 3 #                     // 1959
    DIJKSTRA = 4 #                // 1959
    FLOYD_WARSHALL = 5 #          // 1959~
    ASTAR = 6 #                   // 1968

    # // 1970s
    BIDIRECTIONAL_DIJKSTRA = 7 # ,  // 1971
    BIDIRECTIONAL_ASTAR =8 #,     // 1971
    WEIGHTED_ASTAR = 9 #,          // 1977~
    JOHNSON = 10 #,                 // 1977
    K_SHORTEST_PATH = 11 #,         // 1977~
    DIAL = 12 #,                    // 1969

    # // 1980s
    ITERATIVE_DEEPENING = 13 #,     // 1980
    GREEDY_BEST_FIRST = 14 #,       // 1985
    IDA_STAR = 15 #,                // 1985

    # // 1990s
    RTA_STAR = 16 #,                // 1990
    SMA_STAR = 17 #,                // 1991
    DSTAR = 18 #,                   // 1994
    FAST_MARCHING = 19 #,           // 1996
    ANT_COLONY = 20 #,              // 1996
    FRINGE_SEARCH = 21 #,           // 1997

    # // 2000s
    FOCAL_SEARCH = 22 #,            // 2001
    DSTAR_LITE = 23 #,              // 2002
    LPA_STAR = 24 #,                // 2004
    HPA_STAR = 25 #,                // 2004
    ALT = 26 #,                     // 2005
    ANY_ANGLE_ASTAR = 27 #,         // 2005~
    HCA_STAR = 28 #,                // 2005
    RTAA_STAR = 29 #,               // 2006
    THETA_STAR = 30 #,              // 2007
    CONTRACTION_HIERARCHIES = 31 # ,// 2008

    # // 2010s
    LAZY_THETA_STAR = 32 #,         // 2010
    JUMP_POINT_SEARCH = 33 #,       // 2011
    SIPP = 34 #,                    // 2011
    JPS_PLUS = 35 #,                // 2012
    EPEA_STAR = 36 #,               // 2012
    MHA_STAR = 37 #              // 2012
    ANYA = 38 #,                    // 2013

    # // 특수 목적 / 확장형
    DAG_SP = 39 #,                  // 1960s (DAG 최단경로 O(V+E))
    MULTI_SOURCE_BFS =40 #,        // 2000s (복수 시작점 BFS)
    MCTS =41 #                     // 2006


class c_route_finder:
    def __init__(self,
                 navgrid: c_navgrid,
                 start: c_coord = None,
                 goal: c_coord = None,
                 type: RouteFinderType = RouteFinderType.ASTAR,
                 typedata=None,
                 cost_fn=None,
                 cost_userdata=None,
                 heuristic_fn=None,
                 heuristic_userdata=None,
                 max_retry: int = MAX_RETRY,
                 debug=False,
                 raw_ptr=None, own=False):

        if raw_ptr:
            self._c = raw_ptr
            self._own = own
        else:
            if navgrid is None:
                raise ValueError("navgrid (navgrid) must be provided")

            if not start:
                start = c_coord(0, 0)
            if not goal:
                goal = c_coord(0, 0)

            if not cost_fn:
                cost_fn = C.default_cost
            if not heuristic_fn:
                heuristic_fn = C.euclidean_heuristic

            self._c = C.route_finder_create_full(
                navgrid.ptr(),
                start.ptr(),
                goal.ptr(),
                type,
                ffi.NULL if typedata is None else ffi.new_handle(typedata),
                max_retry,
                debug,
                cost_fn,
                ffi.NULL if cost_userdata is None else ffi.new_handle(cost_userdata),
                heuristic_fn,
                ffi.NULL if heuristic_userdata is None else ffi.new_handle(heuristic_userdata)
            )
            self._own = True

        if not self._c:
            raise MemoryError("route_finder allocation failed")

        if self._own:
            self._finalizer = weakref.finalize(
                self, C.route_finder_destroy, self._c)
        else:
            self._finalizer = None

        self.type = type


    def ptr(self):
        return self._c

    def find(self):
        result = C.route_finder_run(self._c)
        if result != ffi.NULL:
            return c_route(raw_ptr=result, own=True)
        return None

    def set_type(self, type: RouteFinderType):
        C.route_finder_set_type(self._c, type)

    def get_type(self):
        return RouteFinderType(C.route_finder_get_type(self._c))

    def set_max_retry(self, max_retry: int):
        C.route_finder_set_max_retry(self._c, max_retry)

    def get_max_retry(self):
        return C.route_finder_get_max_retry(self._c)

    def enable_debug(self, is_logging: bool):
        C.route_finder_enable_debug_mode(self._c, is_logging)

    def is_debug_enabled(self):
        return C.route_finder_is_debug_mode_enabled(self._c)

    def set_cost_func(self, name: str):
        fn = g_RouteFuncReg.get_cost_func(name)
        C.route_finder_set_cost_func(self._c, fn)

    def set_heuristic_func(self, name: str):
        fn = g_RouteFuncReg.get_heuristic_func(name)
        C.route_finder_set_heuristic_func(self._c, fn)

    def set_start(self, coord: c_coord):
        C.route_finder_set_start(self._c, coord._c)

    def set_goal(self, coord: c_coord):
        C.route_finder_set_goal(self._c, coord._c)

    def set_userdata(self, obj):
        C.route_finder_set_userdata(self._c, ffi.new_handle(obj))

    def get_start(self):
        out = ffi.new("coord_t*")
        C.route_finder_fetch_start(self._c, out)
        return c_coord(raw_ptr=out)

    def get_goal(self):
        out = ffi.new("coord_t*")
        C.route_finder_fetch_goal(self._c, out)
        return c_coord(raw_ptr=out)

    def get_navgrid(self):
        return c_navgrid(raw_ptr=C.route_finder_get_navgrid(self._c))

    def is_valid(self):
        return bool(C.route_finder_is_valid(self._c))

    def clear(self):
        C.route_finder_clear(self._c)

    def print(self):
        C.route_finder_print(self._c)

    def name(self):
        name_ptr = C.get_route_finder_name(self.get_type())
        return ffi.string(name_ptr).decode("utf-8") if name_ptr != ffi.NULL else "UNKNOWN"

    def __del__(self):
        self.close()

    def close(self):
        if self._own and self._finalizer and self._finalizer.alive:
            self._finalizer()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    @staticmethod
    def list_route_finders():
        return list(RouteFinderType)

    @staticmethod
    def find_by_name(name: str):
        for a in RouteFinderType:
            if ffi.string(C.get_route_finder_name(a)).decode("utf-8").lower() == name.lower():
                return a
        return RouteFinderType.UNKNOWN
