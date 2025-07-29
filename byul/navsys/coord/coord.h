#ifndef COORD_H
#define COORD_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>   // C style

#include "byul_common.h"

#ifdef __cplusplus
extern "C" {
#endif

// ------------------------ Coordinate limits ------------------------
#ifndef COORD_MAX
#define COORD_MAX   (200000000)   ///< Maximum absolute value of coord
#endif

#ifndef COORD_MIN
#define COORD_MIN   (-200000000)  ///< Minimum absolute value of coord
#endif

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
BYUL_API coord_t* coord_create_full(int x, int y);
BYUL_API coord_t* coord_create(void);
BYUL_API void     coord_destroy(coord_t* c);
BYUL_API coord_t* coord_copy(const coord_t* c);

// ------------------------ Initialization and Copy ------------------------
/**
 * @brief Initialize to (0,0)
 */
BYUL_API void coord_init(coord_t* c);

/**
 * @brief Initialize with given values
 */
BYUL_API void coord_init_full(coord_t* c, int x, int y);

/**
 * @brief Copy coordinates
 */
BYUL_API void coord_assign(coord_t* dst, const coord_t* src);

// ------------------------ Arithmetic Operations ------------------------
/**
 * @brief Store the result of a + b in dst
 */
BYUL_API void coord_add(coord_t* dst, const coord_t* a, const coord_t* b);

/**
 * @brief Store the result of a - b in dst
 */
BYUL_API void coord_sub(coord_t* dst, const coord_t* a, const coord_t* b);

/**
 * @brief Store the result of a * scalar in dst
 */
BYUL_API void coord_mul(coord_t* dst, const coord_t* a, int scalar);

/**
 * @brief Store the result of a / scalar in dst (integer division)
 */
BYUL_API void coord_div(coord_t* dst, const coord_t* a, int scalar);

// ------------------------ In-place Operations ------------------------
/**
 * @brief c += other
 */
BYUL_API void coord_iadd(coord_t* c, const coord_t* other);

/**
 * @brief c -= other
 */
BYUL_API void coord_isub(coord_t* c, const coord_t* other);

/**
 * @brief c *= scalar
 */
BYUL_API void coord_imul(coord_t* c, int scalar);

/**
 * @brief c /= scalar (integer division)
 */
BYUL_API void coord_idiv(coord_t* c, int scalar);

// ------------------------ Comparison/Hash ------------------------
BYUL_API unsigned coord_hash(const coord_t* c);
BYUL_API bool     coord_equal(const coord_t* c1, const coord_t* c2);
BYUL_API int      coord_compare(const coord_t* c1, const coord_t* c2);

/// Euclidean distance calculation
BYUL_API float    coord_distance(const coord_t* a, const coord_t* b);

BYUL_API int      coord_manhattan_distance(const coord_t* a, const coord_t* b);

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
BYUL_API double coord_angle(const coord_t* a, const coord_t* b);

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
BYUL_API double coord_degree(const coord_t* a, const coord_t* b);

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
BYUL_API void coord_next_to_goal(
    coord_t* out,
    const coord_t* start,
    const coord_t* goal);

// ------------------------ Getters/Setters ------------------------
BYUL_API int      coord_get_x(const coord_t* c);
BYUL_API void     coord_set_x(coord_t* c, int x);

BYUL_API int      coord_get_y(const coord_t* c);
BYUL_API void     coord_set_y(coord_t* c, int y);

BYUL_API void     coord_set(coord_t* c, int x, int y);
BYUL_API void     coord_fetch(const coord_t* c, int* out_x, int* out_y);

// For backward compatibility, will be removed later.
BYUL_API const coord_t* make_tmp_coord(int x, int y);

// For backward compatibility, will be removed later.
// Returns the closest neighbor moving from start to goal.
BYUL_API coord_t* coord_clone_next_to_goal(
    const coord_t* start, const coord_t* goal);

/**
 * @brief Convert coord value to a string
 * @param c coord to convert
 * @param buffer output buffer (recommend at least 32 bytes)
 * @param buffer_size buffer size
 * @return buffer (returned for convenience)
 */
BYUL_API char* coord_to_string(
    const coord_t* c, char* buffer, size_t buffer_size);

/**
 * @brief Print coord value to console
 * @param c coord to print
 */
BYUL_API void coord_print(const coord_t* c);

#ifdef __cplusplus
}
#endif

#endif // COORD_H
