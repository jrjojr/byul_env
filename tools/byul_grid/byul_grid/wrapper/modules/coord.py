from ffi_core import ffi, C
import weakref

ffi.cdef("""
// ------------------------ Coordinate limits ------------------------
// #ifndef COORD_MAX
// #define COORD_MAX   (200000000)   ///< Maximum absolute value of coord
// #endif

// // #ifndef COORD_MIN
// #define COORD_MIN   (-200000000)  ///< Minimum absolute value of coord
// #endif

// ------------------------ Struct Definition ------------------------
/**
 * @struct s_coord
 * @brief 2D cell coordinate (integer-based)
 */
typedef struct s_coord {
    int x; ///< X coordinate
    int y; ///< Y coordinate
} coord_t;

// ------------------------ Create/Destroy ------------------------
coord_t* coord_create_full(int x, int y);
coord_t* coord_create(void);
void     coord_destroy(coord_t* c);
coord_t* coord_copy(const coord_t* c);

// ------------------------ Initialization and Copy ------------------------
/**
 * @brief Initialize to (0,0)
 */
void coord_init(coord_t* c);

/**
 * @brief Initialize with given values
 */
void coord_init_full(coord_t* c, int x, int y);

/**
 * @brief Copy coordinates
 */
void coord_assign(coord_t* dst, const coord_t* src);

// ------------------------ Arithmetic Operations ------------------------
/**
 * @brief Store the result of a + b in dst
 */
void coord_add(coord_t* dst, const coord_t* a, const coord_t* b);

/**
 * @brief Store the result of a - b in dst
 */
void coord_sub(coord_t* dst, const coord_t* a, const coord_t* b);

/**
 * @brief Store the result of a * scalar in dst
 */
void coord_mul(coord_t* dst, const coord_t* a, int scalar);

/**
 * @brief Store the result of a / scalar in dst (integer division)
 */
void coord_div(coord_t* dst, const coord_t* a, int scalar);

// ------------------------ In-place Operations ------------------------
/**
 * @brief c += other
 */
void coord_iadd(coord_t* c, const coord_t* other);

/**
 * @brief c -= other
 */
void coord_isub(coord_t* c, const coord_t* other);

/**
 * @brief c *= scalar
 */
void coord_imul(coord_t* c, int scalar);

/**
 * @brief c /= scalar (integer division)
 */
void coord_idiv(coord_t* c, int scalar);

// ------------------------ Comparison/Hash ------------------------
unsigned coord_hash(const coord_t* c);
bool     coord_equal(const coord_t* c1, const coord_t* c2);
int      coord_compare(const coord_t* c1, const coord_t* c2);

/// Euclidean distance calculation
float    coord_distance(const coord_t* a, const coord_t* b);

int      coord_manhattan_distance(const coord_t* a, const coord_t* b);

/**
 * @brief Return the angle between two coordinate vectors in radians (0 ~ 2PI)
 *
 * @param a Start coordinate (vector origin)
 * @param b Target coordinate (vector end)
 * @return double (0.0 ~ 2PI)
 *
 * Reference:
 *   - (0,0) -> (1,0) direction is 0 rad
 *   - (0,0) -> (0,1) direction is PI/2 rad
 *   - (0,0) -> (-1,0) direction is PI rad
 *   - (0,0) -> (0,-1) direction is 3PI/2 rad
 *
 * @note Returns atan2(y, x), negative angles are converted by adding 2PI.
 */
double coord_angle(const coord_t* a, const coord_t* b);

/**
 * @brief Return the angle between two coordinate vectors in degrees (0 ~ 360)
 *
 * @param a Start coordinate (vector origin)
 * @param b Target coordinate (vector end)
 * @return double (0.0 ~ 360.0)
 *
 * Reference:
 *   - (0,0) -> (1,0) direction is 0 degrees
 *   - (0,0) -> (0,1) direction is 90 degrees
 *   - (0,0) -> (-1,0) direction is 180 degrees
 *   - (0,0) -> (0,-1) direction is 270 degrees
 *
 * @note Uses atan2(y, x) to compute the radian angle,
 *       then multiplies by (180.0 / PI) to convert to degrees.
 *       Negative angles are converted by adding 360 degrees.
 */
double coord_degree(const coord_t* a, const coord_t* b);

/**
 * @brief From start to goal, move one step towards goal
 *        and store the closest neighbor coordinate in out.
 *
 * @param[out] out   Computed next coordinate
 * @param[in]  start Start coordinate
 * @param[in]  goal  Target coordinate
 *
 * @note If start and goal are equal, out = start.
 */
void coord_next_to_goal(
    coord_t* out,
    const coord_t* start,
    const coord_t* goal);

// ------------------------ Getters/Setters ------------------------
int      coord_get_x(const coord_t* c);
void     coord_set_x(coord_t* c, int x);

int      coord_get_y(const coord_t* c);
void     coord_set_y(coord_t* c, int y);

void     coord_set(coord_t* c, int x, int y);
void     coord_fetch(const coord_t* c, int* out_x, int* out_y);

// For backward compatibility, will be removed later.
const coord_t* make_tmp_coord(int x, int y);

// For backward compatibility, will be removed later.
// Returns the closest neighbor moving from start to goal.
coord_t* coord_clone_next_to_goal(
    const coord_t* start, const coord_t* goal);

/**
 * @brief Convert coord value to a string
 * @param c coord to convert
 * @param buffer output buffer (recommend at least 32 bytes)
 * @param buffer_size buffer size
 * @return buffer (returned for convenience)
 */
char* coord_to_string(
    const coord_t* c, char* buffer, size_t buffer_size);

/**
 * @brief Print coord value to console
 * @param c coord to print
 */
void coord_print(const coord_t* c);
         
""")

COORD_MAX = 200000000
COORD_MIN = -200000000

class c_coord:
    def __init__(self, x=0, y=0, raw_ptr=None, own=False):
        if raw_ptr is not None:
            self._c = raw_ptr
            self._own = own
            self._finalizer = None
        else:
            self._c = C.coord_create_full(x, y)
            if not self._c:
                raise MemoryError("coord allocation failed")
            self._own = True

        if self._own:
            self._finalizer = weakref.finalize(self, C.coord_destroy, self._c)
        else:
            self._finalizer = None

    @property
    def x(self):
        return C.coord_get_x(self._c)

    @x.setter
    def x(self, value):
        C.coord_set_x(self._c, value)

    @property
    def y(self):
        return C.coord_get_y(self._c)

    @y.setter
    def y(self, value):
        C.coord_set_y(self._c, value)

    def init_full(self, x: int, y: int):
        C.coord_init_full(self._c, x, y)

    def set(self, x: int, y: int):
        C.coord_set(self._c, x, y)

    def copy(self):
        c = c_coord(raw_ptr=C.coord_copy(self._c), own=True)
        return c

    def distance(self, other: 'c_coord'):
        return C.coord_distance(self.ptr(), other.ptr())

    def manhattan_distance(self, other: 'c_coord'):
        return C.coord_manhattan_distance(self.ptr(), other.ptr())

    def degree(self, other: 'c_coord'):
        return C.coord_degree(self._c, other._c)

    def __eq__(self, other):
        return C.coord_equal(self._c, other._c) != 0

    def __lt__(self, other):
        return C.coord_compare(self._c, other._c) < 0

    def __ge__(self, other):
        return C.coord_compare(self._c, other._c) >= 0

    def __hash__(self):
        return C.coord_hash(self._c)

    def __del__(self):
        self.close()

    def close(self):
        if self._own and self._finalizer and self._finalizer.alive:
            self._finalizer()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def __add__(self, other):
        result = self.copy()
        C.coord_iadd(result._c, other._c)
        return result

    def __sub__(self, other):
        result = self.copy()
        C.coord_isub(result._c, other._c)
        return result

    def __mul__(self, scalar: int):
        result = self.copy()
        C.coord_imul(result._c, scalar)
        return result

    def __floordiv__(self, scalar: int):
        result = self.copy()
        C.coord_idiv(result._c, scalar)
        return result

    def __str__(self):
        return f"c_coord(x={self.x}, y={self.y})"

    def __repr__(self):
        return str(self)

    def to_tuple(self):
        return (self.x, self.y)

    def ptr(self):
        return self._c

    @staticmethod
    def from_tuple(t: tuple):
        return c_coord(*t)

# Constants accessible from Python side
c_coord.COORD_MAX = COORD_MAX
c_coord.COORD_MIN = COORD_MIN
