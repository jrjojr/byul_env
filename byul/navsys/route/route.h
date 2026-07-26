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

/**
 * @brief route 결과의 완료 상태를 나타낸다.
 *
 * @byul.storage basic-value
 * @byul.zero_valid true
 * @byul.copy_semantics trivial-copy
 * @byul.thread_safety thread-compatible
 */
typedef enum e_route_completion {
    ROUTE_COMPLETION_NONE = 0, /**< 경로 결과가 비어 있고 완료되지 않았다. */
    ROUTE_COMPLETION_COMPLETE = 1, /**< 목표에 도달한 완성 경로다. */
    ROUTE_COMPLETION_PARTIAL = 2 /**< 좌표가 있지만 목표에 도달하지 못했다. */
} route_completion_t;

/**
 * @brief builder append 시 경계 좌표를 처리하는 정책이다.
 *
 * @byul.storage basic-value
 * @byul.zero_valid true
 * @byul.copy_semantics trivial-copy
 * @byul.thread_safety thread-compatible
 */
typedef enum e_route_join_policy {
    ROUTE_JOIN_KEEP_ALL = 0, /**< source의 모든 좌표를 유지한다. */
    ROUTE_JOIN_DEDUP_BOUNDARY = 1 /**< 양쪽 경계가 같으면 source의 첫 좌표를 생략한다. */
} route_join_policy_t;

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
typedef struct s_route_builder route_builder_t;
typedef struct s_navsys_search_trace navsys_search_trace_t;

/** Creation and Destruction **/
BYUL_API route_t* route_create(void);

BYUL_API void  route_destroy(route_t* p);

/** Copy and Comparison **/
BYUL_API route_t* route_copy(const route_t* p);

/**
 * @brief route를 독립적으로 deep-copy한다.
 *
 * 실패하면 out_route를 변경하지 않는다. 성공한 결과는 caller가 route_destroy()로
 * 해제한다.
 *
 * @param[in] source 복사할 route.
 * @param[out] out_route 새 route를 받을 caller storage.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK 복사했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT source 또는 out_route가 NULL이다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY allocation에 실패했다.
 * @retval NAVSYS_STATUS_CORRUPT_STATE source의 owning child가 유효하지 않다.
 * @byul.nullable source false
 * @byul.nullable out_route false
 * @byul.lifetime out_route caller-owned
 * @byul.side_effect writes:out_route-on-success
 */
BYUL_API navsys_status_t route_clone_ex(
    const route_t* source,
    route_t** out_route);
BYUL_API uintptr_t route_hash(const route_t* a);
BYUL_API int route_equal(const route_t* a, const route_t* b);

/** Basic Information **/
BYUL_API void  route_set_cost(route_t* p, float cost);
BYUL_API float route_get_cost(const route_t* p);
BYUL_API void  route_set_success(route_t* p, int success);
BYUL_API int   route_get_success(const route_t* p);

/** Coordinate List Access **/
/**
 * @brief route가 소유한 coordinate list의 borrowed view를 반환한다.
 *
 * 반환 pointer는 route보다 먼저 파괴할 수 없고, route coordinate mutation 또는
 * route_destroy() 뒤에는 사용할 수 없다.
 *
 * @param[in] p 조회할 route.
 * @return Borrowed coordinate list. p가 NULL이면 NULL이다.
 * @byul.nullable p true
 * @byul.nullable return true
 * @byul.lifetime return borrowed-from:p
 * @byul.invalidates route-coordinate-mutation,route_destroy
 */
BYUL_API const coord_list_t* route_get_coords(const route_t* p);

/** Visit Logs **/
/**
 * @brief route가 소유한 방문 순서 list의 borrowed view를 반환한다.
 * @param[in] p 조회할 route.
 * @return Borrowed 방문 순서 list. p가 NULL이면 NULL이다.
 * @byul.nullable p true
 * @byul.nullable return true
 * @byul.lifetime return borrowed-from:p
 * @byul.invalidates route_clear_visited,route_destroy
 */
BYUL_API const coord_list_t* route_get_visited_order(const route_t* p);
/**
 * @brief route가 소유한 방문 횟수 hash의 borrowed view를 반환한다.
 * @param[in] p 조회할 route.
 * @return Borrowed 방문 횟수 hash. p가 NULL이면 NULL이다.
 * @byul.nullable p true
 * @byul.nullable return true
 * @byul.lifetime return borrowed-from:p
 * @byul.invalidates route_clear_visited,route_destroy
 */
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
 * @brief route의 누적 cost를 caller storage로 복사한다.
 *
 * 저장된 float 값을 double로 정확히 승격한다. 실패하면 out_total_cost를 변경하지 않는다.
 *
 * @param[in] route 조회할 route.
 * @param[out] out_total_cost 누적 cost를 받을 caller storage.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK cost를 복사했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT route 또는 out_total_cost가 NULL이다.
 * @byul.nullable route false
 * @byul.nullable out_total_cost false
 * @byul.side_effect writes:out_total_cost-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API navsys_status_t route_fetch_total_cost(
    const route_t* route,
    double* out_total_cost);

/**
 * @brief route 결과의 완료 상태를 caller storage로 복사한다.
 *
 * Legacy success가 true이면 COMPLETE, success가 false이고 좌표가 있으면 PARTIAL,
 * 그 외에는 NONE이다. 실패하면 out_completion을 변경하지 않는다.
 *
 * @param[in] route 조회할 route.
 * @param[out] out_completion 완료 상태를 받을 caller storage.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK 완료 상태를 복사했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT route 또는 out_completion이 NULL이다.
 * @byul.nullable route false
 * @byul.nullable out_completion false
 * @byul.side_effect writes:out_completion-on-success
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API navsys_status_t route_fetch_completion(
    const route_t* route,
    route_completion_t* out_completion);

/**
 * @brief Copies route coordinates into caller-provided storage.
 *
 * Pass NULL output with zero capacity to query the required element count.
 * When capacity is smaller than the route length, the function preserves the
 * output buffer, reports the full required count, and returns
 * NAVSYS_STATUS_INCOMPLETE.
 *
 * @param[in] route Route to export.
 * @param[out] output Caller-provided coordinate array, or NULL for a query.
 * @param[in] capacity Number of coord_t elements available in output.
 * @param[out] out_required_count Full number of route coordinates.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK The query or complete copy succeeded.
 * @retval NAVSYS_STATUS_INCOMPLETE The short output buffer was preserved.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT An argument combination is invalid.
 * @byul.nullable route false
 * @byul.nullable output query-only
 * @byul.nullable out_required_count false
 * @byul.side_effect writes:output-on-success,out_required_count-on-nonargument-status
 * @byul.thread_safety thread-compatible
 * @byul.blocking false
 * @byul.reentrant true
 */
BYUL_API navsys_status_t route_export_coords(
    const route_t* route,
    coord_t* output,
    size_t capacity,
    size_t* out_required_count);

/**
 * @brief 빈 transactional route builder를 생성한다.
 *
 * 생성된 builder는 caller가 route_builder_destroy()로 해제한다. 이 API는
 * production allocator 주입을 노출하지 않으며 프로젝트의 공통 allocation 경계를
 * 사용한다. 실패하면 out_builder를 보존한다.
 *
 * @param[out] out_builder 생성한 builder를 받을 caller storage.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK builder를 생성했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT out_builder가 NULL이다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY allocation에 실패했다.
 * @byul.nullable out_builder false
 * @byul.lifetime out_builder caller-owned
 * @byul.side_effect writes:out_builder-on-success
 */
BYUL_API navsys_status_t route_builder_create(route_builder_t** out_builder);

/**
 * @brief immutable route의 좌표와 결과 metadata로 builder를 생성한다.
 *
 * search trace, retry count와 heading observation history는 편집 결과에 포함하지 않는다.
 * source와 새 builder는 독립 owner다. 실패하면 out_builder를 보존한다.
 *
 * @param[in] source 복제할 route.
 * @param[out] out_builder 생성한 builder를 받을 caller storage.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK builder를 생성했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer가 NULL이다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY allocation에 실패했다.
 * @retval NAVSYS_STATUS_CORRUPT_STATE source의 owning child가 유효하지 않다.
 * @byul.nullable source false
 * @byul.nullable out_builder false
 * @byul.lifetime out_builder caller-owned
 * @byul.side_effect writes:out_builder-on-success
 */
BYUL_API navsys_status_t route_builder_create_from_route(
    const route_t* source,
    route_builder_t** out_builder);

/**
 * @brief builder를 해제한다. NULL은 no-op이다.
 * @param[in,out] builder 해제할 builder.
 * @byul.nullable builder true
 * @byul.invalidates builder
 */
BYUL_API void route_builder_destroy(route_builder_t* builder);

/**
 * @brief builder 끝에 좌표를 추가한다.
 *
 * 실패 시 builder는 변경되지 않는다.
 *
 * @param[in,out] builder 편집할 builder.
 * @param[in] coord 복사할 좌표.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK 좌표를 추가했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer가 NULL이다.
 * @retval NAVSYS_STATUS_INVALIDATED builder가 이미 finish됐다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY allocation에 실패했다.
 * @byul.nullable builder false
 * @byul.nullable coord false
 * @byul.side_effect mutates:builder-on-success
 */
BYUL_API navsys_status_t route_builder_push_coord(
    route_builder_t* builder,
    const coord_t* coord);

/**
 * @brief builder의 index 위치에 좌표를 삽입한다.
 *
 * index는 현재 count까지 허용한다. 실패 시 builder는 변경되지 않는다.
 *
 * @param[in,out] builder 편집할 builder.
 * @param[in] index 삽입할 zero-based index.
 * @param[in] coord 복사할 좌표.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK 좌표를 삽입했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer 또는 index가 유효하지 않다.
 * @retval NAVSYS_STATUS_INVALIDATED builder가 이미 finish됐다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY allocation에 실패했다.
 * @byul.nullable builder false
 * @byul.nullable coord false
 * @byul.side_effect mutates:builder-on-success
 */
BYUL_API navsys_status_t route_builder_insert_coord(
    route_builder_t* builder,
    size_t index,
    const coord_t* coord);

/**
 * @brief builder에서 index 좌표를 제거한다.
 *
 * 실패 시 builder와 out_removed를 보존한다.
 *
 * @param[in,out] builder 편집할 builder.
 * @param[in] index 제거할 zero-based index.
 * @param[out] out_removed 제거한 좌표를 받을 optional storage.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK 좌표를 제거했다.
 * @retval NAVSYS_STATUS_NOT_FOUND index가 범위를 벗어났다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT builder가 NULL이다.
 * @retval NAVSYS_STATUS_INVALIDATED builder가 이미 finish됐다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY allocation에 실패했다.
 * @byul.nullable builder false
 * @byul.nullable out_removed true
 * @byul.side_effect mutates:builder-on-success,writes:out_removed-on-success
 */
BYUL_API navsys_status_t route_builder_remove_coord(
    route_builder_t* builder,
    size_t index,
    coord_t* out_removed);

/**
 * @brief source route를 builder에 transactional하게 append한다.
 *
 * create_from_route(source) 뒤 같은 source를 append하는 self-append 패턴도 안전하다.
 * 좌표가 변경되면 cost는 0, completion은 좌표 유무에 따른 PARTIAL/NONE으로
 * 재계산되며 trace와 retry metadata는 포함하지 않는다.
 *
 * @param[in,out] builder 편집할 builder.
 * @param[in] source append할 immutable route.
 * @param[in] join_policy 경계 중복 처리 정책.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK append했거나 추가할 좌표가 없었다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer 또는 정책이 유효하지 않다.
 * @retval NAVSYS_STATUS_INVALIDATED builder가 이미 finish됐다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY allocation에 실패했다.
 * @retval NAVSYS_STATUS_CORRUPT_STATE route storage가 유효하지 않다.
 * @byul.nullable builder false
 * @byul.nullable source false
 * @byul.side_effect mutates:builder-on-success
 */
BYUL_API navsys_status_t route_builder_append(
    route_builder_t* builder,
    const route_t* source,
    route_join_policy_t join_policy);

/**
 * @brief source의 [begin,end) 좌표로 builder 내용을 교체한다.
 *
 * 빈 범위를 허용한다. 성공 시 cost와 completion은 기본값으로 재계산되고 search
 * trace, retry count와 heading history는 포함하지 않는다. 실패 시 builder는 보존된다.
 *
 * @param[in,out] builder 편집할 builder.
 * @param[in] source slice 원본 immutable route.
 * @param[in] begin 첫 포함 index.
 * @param[in] end 마지막 다음 index.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK slice로 교체했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer 또는 범위가 유효하지 않다.
 * @retval NAVSYS_STATUS_INVALIDATED builder가 이미 finish됐다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY allocation에 실패했다.
 * @retval NAVSYS_STATUS_CORRUPT_STATE source storage가 유효하지 않다.
 * @byul.nullable builder false
 * @byul.nullable source false
 * @byul.side_effect mutates:builder-on-success
 */
BYUL_API navsys_status_t route_builder_assign_slice(
    route_builder_t* builder,
    const route_t* source,
    size_t begin,
    size_t end);

/**
 * @brief builder 결과 cost를 설정한다.
 * @param[in,out] builder 편집할 builder.
 * @param[in] total_cost finite float 범위의 결과 cost.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK cost를 설정했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT builder 또는 cost가 유효하지 않다.
 * @retval NAVSYS_STATUS_INVALIDATED builder가 이미 finish됐다.
 * @byul.nullable builder false
 * @byul.side_effect mutates:builder-on-success
 */
BYUL_API navsys_status_t route_builder_set_total_cost(
    route_builder_t* builder,
    double total_cost);

/**
 * @brief builder 결과 completion을 설정한다.
 *
 * NONE은 빈 route, PARTIAL/COMPLETE는 non-empty route에만 유효하다.
 *
 * @param[in,out] builder 편집할 builder.
 * @param[in] completion 설정할 완료 상태.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK completion을 설정했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT builder, enum 또는 좌표 조합이 유효하지 않다.
 * @retval NAVSYS_STATUS_INVALIDATED builder가 이미 finish됐다.
 * @byul.nullable builder false
 * @byul.side_effect mutates:builder-on-success
 */
BYUL_API navsys_status_t route_builder_set_completion(
    route_builder_t* builder,
    route_completion_t completion);

/**
 * @brief builder 결과를 immutable canonical route로 넘긴다.
 *
 * 성공 시 ownership이 out_route로 이동하고 builder는 invalidated된다. 실패하면
 * out_route와 builder를 보존한다.
 *
 * @param[in,out] builder 완료할 builder.
 * @param[out] out_route 결과 route를 받을 caller storage.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK route ownership을 넘겼다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer가 NULL이다.
 * @retval NAVSYS_STATUS_INVALIDATED builder가 이미 finish됐다.
 * @byul.nullable builder false
 * @byul.nullable out_route false
 * @byul.lifetime out_route caller-owned
 * @byul.side_effect invalidates:builder,writes:out_route-on-success
 */
BYUL_API navsys_status_t route_builder_finish(
    route_builder_t* builder,
    route_t** out_route);

/**
 * @brief route value와 독립된 빈 search trace를 생성한다.
 * @param[out] out_trace 생성한 trace를 받을 caller storage.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK trace를 생성했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT out_trace가 NULL이다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY allocation에 실패했다.
 * @byul.nullable out_trace false
 * @byul.lifetime out_trace caller-owned
 * @byul.side_effect writes:out_trace-on-success
 */
BYUL_API navsys_status_t navsys_search_trace_create(
    navsys_search_trace_t** out_trace);

/**
 * @brief search trace를 해제한다. NULL은 no-op이다.
 * @param[in,out] trace 해제할 trace.
 * @byul.nullable trace true
 * @byul.invalidates trace
 */
BYUL_API void navsys_search_trace_destroy(navsys_search_trace_t* trace);

/**
 * @brief search trace를 독립 owner로 deep-copy한다.
 * @param[in] source 복제할 trace.
 * @param[out] out_trace 복제본을 받을 caller storage.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK trace를 복제했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer가 NULL이다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY allocation에 실패했다.
 * @retval NAVSYS_STATUS_CORRUPT_STATE source storage가 유효하지 않다.
 * @byul.nullable source false
 * @byul.nullable out_trace false
 * @byul.lifetime out_trace caller-owned
 * @byul.side_effect writes:out_trace-on-success
 */
BYUL_API navsys_status_t navsys_search_trace_clone_ex(
    const navsys_search_trace_t* source,
    navsys_search_trace_t** out_trace);

/**
 * @brief trace의 ordered visit event 수를 반환한다.
 * @param[in] trace 조회할 trace.
 * @return event 수. trace가 NULL이면 0이다.
 * @byul.nullable trace true
 * @byul.side_effect none
 */
BYUL_API size_t navsys_search_trace_get_visit_count(
    const navsys_search_trace_t* trace);

/**
 * @brief ordered visit event 하나를 caller storage로 복사한다.
 * @param[in] trace 조회할 trace.
 * @param[in] index zero-based event index.
 * @param[out] out_coord 좌표를 받을 caller storage.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK 좌표를 복사했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer가 NULL이다.
 * @retval NAVSYS_STATUS_NOT_FOUND index가 범위를 벗어났다.
 * @retval NAVSYS_STATUS_CORRUPT_STATE trace storage가 유효하지 않다.
 * @byul.nullable trace false
 * @byul.nullable out_coord false
 * @byul.side_effect writes:out_coord-on-success
 */
BYUL_API navsys_status_t navsys_search_trace_fetch_visit(
    const navsys_search_trace_t* trace,
    size_t index,
    coord_t* out_coord);

/**
 * @brief 한 좌표의 누적 visit count를 조회한다.
 * @param[in] trace 조회할 trace.
 * @param[in] coord 조회할 좌표.
 * @param[out] out_count 누적 count를 받을 caller storage.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK count를 복사했다.
 * @retval NAVSYS_STATUS_NOT_FOUND 좌표가 trace에 없다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer가 NULL이다.
 * @retval NAVSYS_STATUS_CORRUPT_STATE trace storage가 유효하지 않다.
 * @byul.nullable trace false
 * @byul.nullable coord false
 * @byul.nullable out_count false
 * @byul.side_effect writes:out_count-on-success
 */
BYUL_API navsys_status_t navsys_search_trace_fetch_coord_visit_count(
    const navsys_search_trace_t* trace,
    const coord_t* coord,
    size_t* out_count);

/**
 * @brief ordered visit events를 caller buffer로 export한다.
 *
 * NULL/0 query를 지원한다. 부족한 buffer에는 기록하지 않고 required count만 쓴다.
 *
 * @param[in] trace 조회할 trace.
 * @param[out] output caller 제공 좌표 배열 또는 query용 NULL.
 * @param[in] capacity output의 coord_t element 수.
 * @param[out] out_required_count 전체 event 수를 받을 storage.
 * @return Common Navsys status value.
 * @retval NAVSYS_STATUS_OK query 또는 전체 복사에 성공했다.
 * @retval NAVSYS_STATUS_INCOMPLETE buffer가 부족해 output을 보존했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT argument 조합이 유효하지 않다.
 * @retval NAVSYS_STATUS_CORRUPT_STATE trace storage가 유효하지 않다.
 * @byul.nullable trace false
 * @byul.nullable output query-only
 * @byul.nullable out_required_count false
 * @byul.side_effect writes:output-on-success,out_required_count-on-nonargument-status
 */
BYUL_API navsys_status_t navsys_search_trace_export_visits(
    const navsys_search_trace_t* trace,
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

/**
 * @brief source의 [begin,end) coordinate 범위를 새 route로 복사한다.
 *
 * 좌표만 복사하며 legacy slice와 동일하게 cost, completion, retry와 trace metadata는
 * 기본값이다. 빈 범위는 유효한 빈 route를 만든다. 실패하면 out_route를 변경하지 않는다.
 *
 * @retval NAVSYS_STATUS_OK slice를 생성했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT NULL 또는 잘못된 범위다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY allocation에 실패했다.
 * @retval NAVSYS_STATUS_CORRUPT_STATE source coordinate storage가 유효하지 않다.
 * @byul.lifetime out_route caller-owned
 * @byul.side_effect writes:out_route-on-success
 */
/**
 * @param[in] source route to slice.
 * @param[in] begin First included coordinate index.
 * @param[in] end One-past-the-last coordinate index.
 * @param[out] out_route Caller storage for the new route.
 * @return Common Navsys status value.
 * @byul.nullable source false
 * @byul.nullable out_route false
 */
BYUL_API navsys_status_t route_slice_ex(
    const route_t* source,
    size_t begin,
    size_t end,
    route_t** out_route);

/** Output and Debugging **/
BYUL_API void route_print(const route_t* p);

/** Direction Calculation **/
/**
 * @brief Create the direction vector at index.
 * @param[in] p Source route.
 * @param[in] index Coordinate index used for the direction.
 * @return A caller-owned coordinate destroyed with coord_destroy(), or NULL.
 * @byul.nullable p false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_API coord_t* route_make_direction(route_t* p, int index);
BYUL_API route_dir_t route_get_direction_by_dir_coord(const coord_t* dxdy);
BYUL_API route_dir_t route_get_direction_by_index(route_t* p, int index);
BYUL_API route_dir_t route_calc_average_facing(route_t* p, int history);
BYUL_API float route_calc_average_dir(route_t* p, int history);

/**
 * @brief Convert a route direction to a coordinate vector.
 * @param[in] route_dir Direction value to convert.
 * @return A caller-owned coordinate destroyed with coord_destroy(), or NULL.
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
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

/**
 * @brief predecessor chain을 따라 route 끝에 경로를 원자적으로 추가한다.
 *
 * missing predecessor, cycle 또는 allocation 실패 시 route를 변경하지 않는다.
 *
 * @retval NAVSYS_STATUS_OK 경로를 추가했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT required pointer가 NULL이다.
 * @retval NAVSYS_STATUS_NO_PATH predecessor가 start 전에 끊겼다.
 * @retval NAVSYS_STATUS_CORRUPT_STATE predecessor cycle 또는 내부 storage 손상이다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY temporary/candidate storage allocation에 실패했다.
 * @byul.side_effect mutates:route-on-success
 */
/**
 * @param[in,out] route Route extended only when reconstruction succeeds.
 * @param[in] came_from Predecessor mapping.
 * @param[in] start Path start coordinate.
 * @param[in] goal Goal coordinate where backtracking starts.
 * @return Common Navsys status value.
 * @byul.nullable route false
 * @byul.nullable came_from false
 * @byul.nullable start false
 * @byul.nullable goal false
 */
BYUL_API navsys_status_t route_reconstruct_ex(
    route_t* route,
    const coord_hash_t* came_from,
    const coord_t* start,
    const coord_t* goal);

#ifdef __cplusplus
}
#endif

#endif // ROUTE_H
