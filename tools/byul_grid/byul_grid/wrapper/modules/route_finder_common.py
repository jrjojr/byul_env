from ffi_core import ffi, C

from coord import c_coord
from coord_list import c_coord_list
from coord_hash import c_coord_hash

from route import c_route
from navgrid import c_navgrid

import weakref

ffi.cdef("""
         
// #define DIAGONAL_COST 1.4142135f  // Approximation of sqrt 2

/**
 * @brief Cost function type.
 * 
 * @param m        Map object.
 * @param start    Start coordinate.
 * @param goal     Goal coordinate.
 * @param userdata User-defined data.
 * @return float   Cost value.
 */
typedef float (*cost_func)(
    const navgrid_t*, const coord_t*, const coord_t*, void*);

/**
 * @brief Heuristic function type.
 * 
 * @param start    Start coordinate.
 * @param goal     Goal coordinate.
 * @param userdata User-defined data.
 * @return float   Estimated distance.
 */
typedef float (*heuristic_func)(const coord_t*, const coord_t*, void*);

/**
 * @brief Default cost function (always returns 1.0).
 */
float default_cost(
    const navgrid_t*, const coord_t*, const coord_t*, void*);

/**
 * @brief Cost function returning 0 (all paths have equal cost).
 */
float zero_cost(const navgrid_t*, const coord_t*, const coord_t*, void*);

/**
 * @brief Diagonal movement cost function (uses sqrt 2 approximation).
 */
float diagonal_cost(
    const navgrid_t*, const coord_t*, const coord_t*, void*);

/**
 * @brief Euclidean distance heuristic.
 */
float euclidean_heuristic(const coord_t*, const coord_t*, void*);

/**
 * @brief Manhattan distance heuristic.
 */
float manhattan_heuristic(const coord_t*, const coord_t*, void*);

/**
 * @brief Chebyshev distance heuristic.
 */
float chebyshev_heuristic(const coord_t*, const coord_t*, void*);

/**
 * @brief Octile distance heuristic (8-direction movement).
 */
float octile_heuristic(const coord_t*, const coord_t*, void*);

/**
 * @brief Heuristic function that always returns 0 (for minimal search).
 */
float zero_heuristic(const coord_t*, const coord_t*, void*);

/**
 * @brief Default heuristic (Euclidean).
 */
float default_heuristic(const coord_t*, const coord_t*, void*);
                 
""")

# Diagonal cost constant (sqrt(2) approx)
DIAGONAL_COST = 1.4142135


class c_route_func_registry:
    def __init__(self):
        self._cost_funcs = {}
        self._heuristic_funcs = {}

    def register_cost(self, name: str, func):
        self._cost_funcs[name] = func

    def register_heuristic(self, name: str, func):
        self._heuristic_funcs[name] = func

    def get_cost_func(self, name: str):
        if name not in self._cost_funcs:
            raise ValueError(f"[CostFunc] Unknown function: '{name}'")
        return self._cost_funcs[name]

    def get_heuristic_func(self, name: str):
        if name not in self._heuristic_funcs:
            raise ValueError(f"[HeuristicFunc] Unknown function: '{name}'")
        return self._heuristic_funcs[name]

    def all_cost_names(self):
        return list(self._cost_funcs.keys())

    def all_heuristic_names(self):
        return list(self._heuristic_funcs.keys())


g_RouteFuncReg = c_route_func_registry()

# Cost Functions
g_RouteFuncReg.register_cost("default", C.default_cost)
g_RouteFuncReg.register_cost("zero", C.zero_cost)
g_RouteFuncReg.register_cost("diagonal", C.diagonal_cost)

# Heuristic Functions
g_RouteFuncReg.register_heuristic("euclidean", C.euclidean_heuristic)
g_RouteFuncReg.register_heuristic("manhattan", C.manhattan_heuristic)
g_RouteFuncReg.register_heuristic("chebyshev", C.chebyshev_heuristic)
g_RouteFuncReg.register_heuristic("octile", C.octile_heuristic)
g_RouteFuncReg.register_heuristic("zero", C.zero_heuristic)
g_RouteFuncReg.register_heuristic("default", C.default_heuristic)