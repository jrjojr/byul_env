#ifndef ROUTE_H
#define ROUTE_H

#include <stddef.h>

#include "byul_config.h"
#include "coord.h"
#include "coord_list.h"
#include "coord_hash.h"
#include "navsys_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Direction enumeration **/
typedef enum e_route_dir {
    ROUTE_DIR_UNKNOWN, 
    ROUTE_DIR_RIGHT,
    ROUTE_DIR_UP_RIGHT,
    ROUTE_DIR_UP,
    ROUTE_DIR_UP_LEFT,
    ROUTE_DIR_LEFT,
    ROUTE_DIR_DOWN_LEFT,
    ROUTE_DIR_DOWN,
    ROUTE_DIR_DOWN_RIGHT,
    ROUTE_DIR_COUNT
} route_dir_t;

struct s_route {
    coord_list_t* coords;
    coord_list_t* visited_order;
    coord_hash_t* visited_count;
    float cost;
    bool success;
    int total_retry_count;

    float avg_vec_x;
    float avg_vec_y;
    int vec_count;
};

typedef struct s_route route_t;

/** Creation and Destruction **/
BYUL_API route_t* route_create();

BYUL_API void  route_destroy(route_t* p);

/** Copy and Comparison **/
BYUL_API route_t* route_copy(const route_t* p);
BYUL_API uintptr_t route_hash(const route_t* a);
BYUL_API int route_equal(const route_t* a, const route_t* b);

/** Basic Information **/
BYUL_API void  route_set_cost(route_t* p, float cost);
BYUL_API float route_get_cost(const route_t* p);
BYUL_API void  route_set_success(route_t* p, int success);
BYUL_API int   route_get_success(const route_t* p);

/** Coordinate List Access **/
BYUL_API const coord_list_t* route_get_coords(const route_t* p);

/** Visit Logs **/
BYUL_API const coord_list_t* route_get_visited_order(const route_t* p);
BYUL_API const coord_hash_t*  route_get_visited_count(const route_t* p);

BYUL_API int route_get_total_retry_count(const route_t* p);

BYUL_API void route_set_total_retry_count(route_t* p, int retry_count);

/** Coordinate Manipulation **/
BYUL_API int  route_add_coord(route_t* p, const coord_t* c);
BYUL_API void route_clear_coords(route_t* p);
BYUL_API const coord_t* route_get_last(const route_t* p);
BYUL_API const coord_t* route_get_coord_at(const route_t* p, int index);
BYUL_API int   route_length(const route_t* p);

/**
 * @brief route에 저장된 좌표 수를 반환한다.
 *
 * @param[in] route 조회할 route.
 * @return 좌표 element 수. route가 NULL이면 0이다.
 * @byul.nullable route true
 * @byul.side_effect none
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API size_t route_get_coord_count(const route_t* route);

/**
 * @brief 지정한 index의 좌표를 caller storage로 복사한다.
 *
 * 실패하면 out_coord를 변경하지 않는다.
 *
 * @param[in] route 조회할 route.
 * @param[in] index 조회할 zero-based 좌표 index.
 * @param[out] out_coord 좌표를 받을 caller storage.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK 좌표를 복사했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT route 또는 out_coord가 NULL이다.
 * @retval NAVSYS_STATUS_NOT_FOUND index가 좌표 범위를 벗어났다.
 * @byul.nullable route false
 * @byul.nullable out_coord false
 * @byul.side_effect writes:out_coord-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API navsys_status_t route_fetch_coord(
    const route_t* route,
    size_t index,
    coord_t* out_coord);

/**
 * @brief Copies route coordinates into caller-provided storage.
 *
 * Pass NULL output with zero capacity to query the required element count.
 * When capacity is smaller than the route length, the function copies the
 * prefix that fits, reports the full required count, and returns
 * NAVSYS_STATUS_INCOMPLETE.
 *
 * @param[in] route Route to export.
 * @param[out] output Caller-provided coordinate array, or NULL for a query.
 * @param[in] capacity Number of coord_t elements available in output.
 * @param[out] out_required_count Full number of route coordinates.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK The query or complete copy succeeded.
 * @retval NAVSYS_STATUS_INCOMPLETE A prefix was copied into a short buffer.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT An argument combination is invalid.
 * @byul.nullable route false
 * @byul.nullable output query-only
 * @byul.nullable out_required_count false
 * @byul.side_effect writes:output,out_required_count
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API navsys_status_t route_export_coords(
    const route_t* route,
    coord_t* output,
    size_t capacity,
    size_t* out_required_count);

/** Visit Manipulation **/
BYUL_API int  route_add_visited(route_t* p, const coord_t* c);
BYUL_API void route_clear_visited(route_t* p);

/** Merge and Edit **/
BYUL_API void route_append(route_t* dest, const route_t* src);

// When merging multiple routes, if there are overlapping start and end points,
// only merge the start and end into a single coordinate.
// Intermediate duplicate paths are not merged, only start and end are.
BYUL_API void route_append_nodup(route_t* dest, const route_t* src);

BYUL_API void route_insert(route_t* p, int index, const coord_t* c);
BYUL_API void route_remove_at(route_t* p, int index);
BYUL_API void route_remove_value(route_t* p, const coord_t* c);
BYUL_API int  route_contains(const route_t* p, const coord_t* c);
BYUL_API int  route_find(const route_t* p, const coord_t* c);

// BYUL_API void route_slice(route_t* p, int start, int end);

// Returns a new route_t* sliced from the original route between start and end indices.
// The original route remains unchanged.
BYUL_API route_t* route_slice(const route_t* p, int start, int end);

/** Output and Debugging **/
BYUL_API void route_print(const route_t* p);

/** Direction Calculation **/
BYUL_API coord_t* route_make_direction(route_t* p, int index);
BYUL_API route_dir_t route_get_direction_by_dir_coord(const coord_t* dxdy);
BYUL_API route_dir_t route_get_direction_by_index(route_t* p, int index);
BYUL_API route_dir_t route_calc_average_facing(route_t* p, int history);
BYUL_API float route_calc_average_dir(route_t* p, int history);

BYUL_API coord_t* direction_to_coord(route_dir_t route_dir);

/** Direction Change Detection **/
BYUL_API int route_has_changed(
    route_t* p, const coord_t* from,
    const coord_t* to, float angle_threshold_deg);

BYUL_API int route_has_changed_with_angle(
    route_t* p, const coord_t* from,
    const coord_t* to, float angle_threshold_deg,
    float* out_angle_deg);

BYUL_API int route_has_changed_by_index(
    route_t* p, int index_from,
    int index_to, float angle_threshold_deg);

BYUL_API int route_has_changed_with_angle_by_index(
    route_t* p, int index_from, int index_to,
    float angle_threshold_deg, float* out_angle_deg);

/** Average Vector Update **/
BYUL_API void route_update_average_vector(
    route_t* p, const coord_t* from, const coord_t* to);

BYUL_API void route_update_average_vector_by_index(
    route_t* p, int index_from, int index_to);

BYUL_API route_dir_t calc_direction(
    const coord_t* start, const coord_t* goal);

/// @brief Reconstruct the route by following came_from from goal -> start 
///        and fill it into the route.
/// @param route Output route structure
/// @param came_from coord_hash_t* (coord* -> coord*)
/// @param start Start coordinate
/// @param goal Goal coordinate
/// @return Success status (true: reconstruction successful, false: failed)
BYUL_API bool route_reconstruct(
    route_t* route, const coord_hash_t* came_from,
    const coord_t* start, const coord_t* goal);

#ifdef __cplusplus
}
#endif

#endif // ROUTE_H
