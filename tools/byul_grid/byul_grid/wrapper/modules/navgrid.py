import weakref

from ffi_core import ffi, C

from coord import c_coord
from coord_list import c_coord_list
from coord_hash import c_coord_hash

from enum import IntEnum

class NavgridDirMode(IntEnum):
    DIR_4 = 0
    DIR_8 = 1

ffi.cdef("""
/**
 * @brief Function pointer to check if a coordinate is blocked.
 *
 * This function determines whether a specific coordinate `(x, y)`
 * is an impassable cell for pathfinding or range operations.
 *
 * @param context External data required for coordinate checking (e.g., navgrid, navgrid)
 * @param x X coordinate to check
 * @param y Y coordinate to check
 * @param userdata Optional user-defined data
 * @return true - The coordinate is blocked  
 *         false - The coordinate is passable
 */
typedef bool (*is_coord_blocked_func)(
    const void* context, int x, int y, void* userdata);

/**
 * @brief Checks if a coordinate is blocked based on navgrid.
 *
 * Determines whether a coordinate is impassable due to walls or obstacles
 * using the internal cell information of the navgrid.
 *
 * @param context Pointer to navgrid object (const navgrid_t*)
 * @param x X coordinate to check
 * @param y Y coordinate to check
 * @param userdata Optional user-defined data (can be unused)
 * @return true - The coordinate is blocked  
 *         false - The coordinate is passable
 */
bool is_coord_blocked_navgrid(
    const void* context, int x, int y, void* userdata);

typedef enum {
    NAVGRID_DIR_4,
    NAVGRID_DIR_8
} navgrid_dir_mode_t;

struct s_navgrid {
    int width;
    int height;
    navgrid_dir_mode_t mode;

    coord_hash_t* blocked_coords;

    is_coord_blocked_func is_coord_blocked_fn;
};

typedef struct s_navgrid navgrid_t;

// Constructors and Destructors

// Default: 0 x 0 , NAVGRID_DIR_8
navgrid_t* navgrid_create();

navgrid_t* navgrid_create_full(int width, int height, navgrid_dir_mode_t mode,
    is_coord_blocked_func is_coord_blocked_fn);

void navgrid_destroy(navgrid_t* m);

// Copy and Comparison
navgrid_t* navgrid_copy(const navgrid_t* m);
uint32_t navgrid_hash(const navgrid_t* m);
bool navgrid_equal(const navgrid_t* a, const navgrid_t* b);

// Property Access
int navgrid_get_width(const navgrid_t* m);
void navgrid_set_width(navgrid_t* m, int width);

int navgrid_get_height(const navgrid_t* m);
void navgrid_set_height(navgrid_t* m, int height);

void navgrid_set_is_coord_blocked_func(navgrid_t* m, is_coord_blocked_func fn);
is_coord_blocked_func navgrid_get_is_coord_blocked_fn(const navgrid_t* m);

navgrid_dir_mode_t navgrid_get_mode(const navgrid_t* m);
void navgrid_set_mode(navgrid_t* m);

// Obstacle Management
bool navgrid_block_coord(navgrid_t* m, int x, int y);
bool navgrid_unblock_coord(navgrid_t* m, int x, int y);
bool navgrid_is_inside(const navgrid_t* m, int x, int y);
// bool navgrid_is_blocked(const navgrid_t* m, int x, int y);
void navgrid_clear(navgrid_t* m);

// Get blocked coordinates
const coord_hash_t* navgrid_get_blocked_coords(const navgrid_t* m);

// Neighbor Search
coord_list_t* navgrid_copy_neighbors(const navgrid_t* m, int x, int y);
coord_list_t* navgrid_copy_neighbors_all(const navgrid_t* m, int x, int y);

// If max_range is 0, it behaves the same as navgrid_copy_neighbors_all (only checks neighbors)
coord_list_t* navgrid_copy_neighbors_all_range(
    navgrid_t* m, int x, int y, int range);

coord_t* navgrid_copy_neighbor_at_degree(const navgrid_t* m, 
    int x, int y, double degree);
    
coord_t* navgrid_copy_neighbor_at_goal(const navgrid_t* m, 
    const coord_t* center, const coord_t* goal);

coord_list_t* navgrid_copy_neighbors_at_degree_range(
    const navgrid_t* m,
    const coord_t* center, const coord_t* goal,
    double start_deg, double end_deg,
    int range);

""")

class c_navgrid:
    def __init__(self, raw_ptr=None, own=False,
                 width=None, height=None, mode=NavgridDirMode.DIR_8,
                 py_func=None):
        self._own = own
        self._py_is_coord_blocked_func = None
        self._ffi_is_coord_blocked_func = None

        if raw_ptr:
            self._c = raw_ptr
        elif width is not None and height is not None:
            fn = self._wrap_py_func(py_func) if py_func else ffi.NULL
            self._c = C.navgrid_create_full(width, height, mode.value, fn)
        else:
            self._c = C.navgrid_create()

        if not self._c:
            raise MemoryError("navgrid allocation failed")

        if own:
            self._finalizer = weakref.finalize(self, C.navgrid_destroy, self._c)
        else:
            self._finalizer = None

    def _wrap_py_func(self, py_func):
        self._py_is_coord_blocked_func = py_func

        @ffi.callback("bool(const void*, int, int, void*)")
        def _wrapped(m_ptr, x, y, udata_ptr):
            grid = c_navgrid(raw_ptr=m_ptr, own=False)
            return bool(py_func(grid, x, y, None))

        self._ffi_is_coord_blocked_func = _wrapped
        return _wrapped

    # ───── 속성 접근 ─────
    @property
    def width(self):
        return C.navgrid_get_width(self._c)

    @width.setter
    def width(self, w):
        C.navgrid_set_width(self._c, w)

    @property
    def height(self):
        return C.navgrid_get_height(self._c)

    @height.setter
    def height(self, h):
        C.navgrid_set_height(self._c, h)

    def mode(self):
        return NavgridDirMode(C.navgrid_get_mode(self._c))

    def set_mode(self, mode: NavgridDirMode):
        C.navgrid_set_mode(self._c, mode.value)

    def set_is_coord_blocked_fn(self, py_func):
        fn = self._wrap_py_func(py_func)
        C.navgrid_set_is_coord_blocked_func(self._c, fn)

    def get_is_coord_blocked_fn(self):
        return self._py_is_coord_blocked_func

    # ───── 장애물 관련 ─────
    def block(self, x, y):
        return bool(C.navgrid_block_coord(self._c, x, y))

    def unblock(self, x, y):
        return bool(C.navgrid_unblock_coord(self._c, x, y))

    def is_blocked(self, x, y):
        return bool(C.is_coord_blocked_navgrid(self._c, x, y, ffi.NULL))

    def is_inside(self, x, y):
        return bool(C.navgrid_is_inside(self._c, x, y))

    def clear(self):
        C.navgrid_clear(self._c)

    def blocked_coords(self):
        return c_coord_hash(raw_ptr=C.navgrid_get_blocked_coords(self._c), own=False)

    # ───── 이웃 탐색 ─────
    def neighbors(self, x, y):
        ptr = C.navgrid_copy_neighbors(self._c, x, y)
        return c_coord_list(raw_ptr=ptr, own=True)

    def neighbors_all(self, x, y):
        ptr = C.navgrid_copy_neighbors_all(self._c, x, y)
        return c_coord_list(raw_ptr=ptr, own=True)

    def neighbors_range(self, x, y, range_val):
        ptr = C.navgrid_copy_neighbors_all_range(self._c, x, y, range_val)
        return c_coord_list(raw_ptr=ptr, own=True)

    def neighbor_at_degree(self, x, y, degree):
        ptr = C.navgrid_copy_neighbor_at_degree(self._c, x, y, degree)
        return c_coord(raw_ptr=ptr) if ptr != ffi.NULL else None

    def neighbor_at_goal(self, center: c_coord, goal: c_coord):
        ptr = C.navgrid_copy_neighbor_at_goal(self._c, center.ptr(), goal.ptr())
        return c_coord(raw_ptr=ptr) if ptr != ffi.NULL else None

    def neighbors_at_degree_range(self, center: c_coord, goal: c_coord,
        start_deg: float, end_deg: float, range_val: int):

        ptr = C.navgrid_copy_neighbors_at_degree_range(
            self._c, center.ptr(), goal.ptr(), start_deg, end_deg, range_val)

        return c_coord_list(raw_ptr=ptr, own=True)

    # ───── 복사 및 비교 ─────
    def copy(self):
        new_ptr = C.navgrid_copy(self._c)
        return c_navgrid(raw_ptr=new_ptr, own=True)

    def equals(self, other):
        return isinstance(other, c_navgrid) and C.navgrid_equal(self._c, other._c)

    def __eq__(self, other):
        return self.equals(other)

    def __hash__(self):
        return int(C.navgrid_hash(self._c))

    def ptr(self):
        return self._c

    def __repr__(self):
        return f"c_navgrid({self.width}x{self.height}, mode={self.mode().name})"

    def __del__(self):
        self.close()

    def close(self):
        if self._own and self._finalizer and self._finalizer.alive:
            self._finalizer()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
