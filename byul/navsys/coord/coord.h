/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file coord.h
 * @brief Navsys의 2차원 정수 좌표를 위한 public C ABI를 선언한다.
 *
 * Grid 좌표의 값 layout, 생명주기, checked 수치 연산과 기존 ABI 1.x 호환 함수를
 * 제공하는 public component다.
 */

#ifndef BYUL_COORD_H
#define BYUL_COORD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "byul_config.h"
#include "navsys_status.h"

#ifdef __cplusplus
extern "C" {
#endif

// ------------------------ Coordinate limits ------------------------
#define BYUL_COORD_COMPONENT_MAX INT32_C(200000000)
#define BYUL_COORD_COMPONENT_MIN (-INT32_C(200000000))

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

// ------------------------ ABI Layout Diagnostics ------------------------

/**
 * @brief 현재 SDK에서 coord_t의 크기를 반환한다.
 *
 * @return 바이트 단위의 coord_t 크기
 */
BYUL_API size_t coord_sizeof(void);

/**
 * @brief 현재 SDK에서 coord_t의 정렬 요구사항을 반환한다.
 *
 * @return 바이트 단위의 coord_t 정렬 크기
 */
BYUL_API size_t coord_alignof(void);

/**
 * @brief coord_t의 x field offset을 반환한다.
 *
 * @return 바이트 단위의 x field offset
 */
BYUL_API size_t coord_offsetof_x(void);

/**
 * @brief coord_t의 y field offset을 반환한다.
 *
 * @return 바이트 단위의 y field offset
 */
BYUL_API size_t coord_offsetof_y(void);

// ------------------------ Checked Value API ------------------------

/**
 * @brief 유효 범위의 component로 caller storage 좌표를 초기화한다.
 *
 * 실패하면 out_coord를 변경하지 않는다.
 *
 * @param[out] out_coord 초기화할 caller storage.
 * @param[in] x Grid cell 단위의 X 좌표.
 * @param[in] y Grid cell 단위의 Y 좌표.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 좌표를 초기화했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT out_coord가 NULL이거나 component가
 *     [BYUL_COORD_COMPONENT_MIN, BYUL_COORD_COMPONENT_MAX] 밖이다.
 *
 * @byul.nullable out_coord false
 * @byul.range x [BYUL_COORD_COMPONENT_MIN,BYUL_COORD_COMPONENT_MAX]
 * @byul.range y [BYUL_COORD_COMPONENT_MIN,BYUL_COORD_COMPONENT_MAX]
 * @byul.unit x grid-cells
 * @byul.unit y grid-cells
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_coord-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t coord_init_checked(
    coord_t* out_coord, int32_t x, int32_t y);

/**
 * @brief 유효 범위의 component를 가진 좌표를 할당한다.
 *
 * 실패하면 out_coord가 가리키는 값을 변경하지 않는다. 성공한 결과는 caller가
 * coord_destroy()로 해제한다.
 *
 * @param[in] x Grid cell 단위의 X 좌표.
 * @param[in] y Grid cell 단위의 Y 좌표.
 * @param[out] out_coord 새 좌표 pointer를 받을 storage.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 좌표를 할당했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT out_coord가 NULL이거나 component가
 *     유효 범위 밖이다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY 좌표 storage를 할당하지 못했다.
 *
 * @byul.nullable out_coord false
 * @byul.range x [BYUL_COORD_COMPONENT_MIN,BYUL_COORD_COMPONENT_MAX]
 * @byul.range y [BYUL_COORD_COMPONENT_MIN,BYUL_COORD_COMPONENT_MAX]
 * @byul.unit x grid-cells
 * @byul.unit y grid-cells
 * @byul.lifetime out_coord caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_coord-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t coord_create_checked(
    int32_t x, int32_t y, coord_t** out_coord);

/**
 * @brief 좌표의 독립 복사본을 할당한다.
 *
 * 실패하면 out_coord가 가리키는 값을 변경하지 않는다. 성공한 결과는 caller가
 * coord_destroy()로 해제한다.
 *
 * @param[in] source 복사할 좌표.
 * @param[out] out_coord 새 좌표 pointer를 받을 storage.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 좌표를 복사했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT source 또는 out_coord가 NULL이다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY 좌표 storage를 할당하지 못했다.
 *
 * @byul.nullable source false
 * @byul.nullable out_coord false
 * @byul.lifetime out_coord caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_coord-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t coord_copy_checked(
    const coord_t* source, coord_t** out_coord);

/**
 * @brief 두 좌표 component를 checked 방식으로 더한다.
 *
 * out은 a 또는 b와 같은 pointer일 수 있다. 입력 또는 결과가 유효 범위 밖이면 모든
 * 값을 보존한다.
 *
 * @param[out] out 결과 좌표를 받을 storage.
 * @param[in] a 왼쪽 피연산자.
 * @param[in] b 오른쪽 피연산자.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 덧셈 결과를 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer가 NULL이거나 component 범위를 벗어났다.
 *
 * @byul.nullable out false
 * @byul.nullable a false
 * @byul.nullable b false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t coord_add_checked(
    coord_t* out, const coord_t* a, const coord_t* b);

/**
 * @brief 두 좌표 component를 checked 방식으로 뺀다.
 *
 * out은 a 또는 b와 같은 pointer일 수 있다. 입력 또는 결과가 유효 범위 밖이면 모든
 * 값을 보존한다.
 *
 * @param[out] out 결과 좌표를 받을 storage.
 * @param[in] a 왼쪽 피연산자.
 * @param[in] b 오른쪽 피연산자.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 뺄셈 결과를 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer가 NULL이거나 component 범위를 벗어났다.
 *
 * @byul.nullable out false
 * @byul.nullable a false
 * @byul.nullable b false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t coord_sub_checked(
    coord_t* out, const coord_t* a, const coord_t* b);

/**
 * @brief 좌표 component에 scalar를 checked 방식으로 곱한다.
 *
 * out은 a와 같은 pointer일 수 있다. 입력 또는 결과가 유효 범위 밖이면 모든 값을
 * 보존한다.
 *
 * @param[out] out 결과 좌표를 받을 storage.
 * @param[in] a 입력 좌표.
 * @param[in] scalar 곱할 정수 값.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 곱셈 결과를 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer가 NULL이거나 component 범위를 벗어났다.
 *
 * @byul.nullable out false
 * @byul.nullable a false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t coord_mul_checked(
    coord_t* out, const coord_t* a, int32_t scalar);

/**
 * @brief 좌표 component를 scalar로 checked 정수 나눗셈한다.
 *
 * C의 0 방향 절삭을 사용한다. out은 a와 같은 pointer일 수 있으며 실패하면 모든 값을
 * 보존한다.
 *
 * @param[out] out 결과 좌표를 받을 storage.
 * @param[in] a 입력 좌표.
 * @param[in] scalar 0이 아닌 제수.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 나눗셈 결과를 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer가 NULL이거나 scalar가 0이거나 component
 *     범위를 벗어났다.
 *
 * @byul.nullable out false
 * @byul.nullable a false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t coord_div_checked(
    coord_t* out, const coord_t* a, int32_t scalar);

/**
 * @brief 두 좌표를 x, y 순서의 canonical lexicographic order로 비교한다.
 *
 * 성공하면 out_order에 정확히 -1, 0 또는 1을 기록한다.
 *
 * @param[in] a 왼쪽 좌표.
 * @param[in] b 오른쪽 좌표.
 * @param[out] out_order 비교 결과를 받을 storage.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 비교 결과를 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer가 하나라도 NULL이다.
 *
 * @byul.nullable a false
 * @byul.nullable b false
 * @byul.nullable out_order false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_order-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t coord_compare_canonical(
    const coord_t* a, const coord_t* b, int* out_order);

/**
 * @brief 두 좌표 사이의 Euclidean 거리를 double로 계산한다.
 *
 * @param[in] a 첫 좌표.
 * @param[in] b 둘째 좌표.
 * @param[out] out_distance 거리를 받을 storage.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 거리를 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer가 하나라도 NULL이다.
 *
 * @byul.nullable a false
 * @byul.nullable b false
 * @byul.nullable out_distance false
 * @byul.unit return grid-cells
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_distance-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t coord_distance_f64(
    const coord_t* a, const coord_t* b, double* out_distance);

/**
 * @brief 두 좌표 사이의 Manhattan 거리를 64-bit 정수로 계산한다.
 *
 * @param[in] a 첫 좌표.
 * @param[in] b 둘째 좌표.
 * @param[out] out_distance 거리를 받을 storage.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 거리를 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer가 하나라도 NULL이다.
 *
 * @byul.nullable a false
 * @byul.nullable b false
 * @byul.nullable out_distance false
 * @byul.unit return grid-cells
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_distance-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t coord_manhattan_distance_i64(
    const coord_t* a, const coord_t* b, int64_t* out_distance);

/**
 * @brief from에서 to를 향하는 각도를 [0, 2π) radian으로 계산한다.
 *
 * 같은 좌표에는 방향이 없으므로 실패하며 out_angle을 보존한다.
 *
 * @param[in] from 시작 좌표.
 * @param[in] to 목표 좌표.
 * @param[out] out_angle 각도를 받을 storage.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 각도를 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer가 NULL이거나 두 좌표가 같다.
 *
 * @byul.nullable from false
 * @byul.nullable to false
 * @byul.nullable out_angle false
 * @byul.range out_angle [0,2pi)
 * @byul.unit out_angle radians
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_angle-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t coord_angle_rad(
    const coord_t* from, const coord_t* to, double* out_angle);

/**
 * @brief from에서 to를 향하는 각도를 [0, 360) degree로 계산한다.
 *
 * 같은 좌표에는 방향이 없으므로 실패하며 out_angle을 보존한다.
 *
 * @param[in] from 시작 좌표.
 * @param[in] to 목표 좌표.
 * @param[out] out_angle 각도를 받을 storage.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 각도를 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer가 NULL이거나 두 좌표가 같다.
 *
 * @byul.nullable from false
 * @byul.nullable to false
 * @byul.nullable out_angle false
 * @byul.range out_angle [0,360)
 * @byul.unit out_angle degrees
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_angle-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t coord_angle_deg(
    const coord_t* from, const coord_t* to, double* out_angle);

/**
 * @brief start에서 goal 방향의 8-neighbor 한 칸을 계산한다.
 *
 * 서로 다른 각 축을 정확히 한 component씩 goal 방향으로 이동한다. out_next는 start
 * 또는 goal과 같은 pointer일 수 있다.
 *
 * @param[in] start 시작 좌표.
 * @param[in] goal 목표 좌표.
 * @param[out] out_next 다음 좌표를 받을 storage.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 다음 좌표를 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer가 하나라도 NULL이다.
 *
 * @byul.nullable start false
 * @byul.nullable goal false
 * @byul.nullable out_next false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_next-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t coord_step_toward(
    const coord_t* start, const coord_t* goal, coord_t* out_next);

// ------------------------ Create/Destroy ------------------------

/**
 * @brief 지정한 값으로 좌표를 생성한다.
 *
 * @param[in] x Grid cell 단위의 X 좌표.
 * @param[in] y Grid cell 단위의 Y 좌표.
 * @return Caller가 소유하는 새 좌표.
 *
 * @byul.nullable return false
 * @byul.lifetime return caller-owned
 */
BYUL_API coord_t* coord_create_full(int x, int y);

/**
 * @brief 원점 좌표를 생성한다.
 *
 * @return Caller가 소유하는 새 좌표.
 *
 * @byul.nullable return false
 * @byul.lifetime return caller-owned
 */
BYUL_API coord_t* coord_create(void);

/**
 * @brief 좌표를 해제한다.
 *
 * @param[in] c 해제할 좌표. NULL이면 아무 작업도 하지 않는다.
 *
 * @byul.nullable c true
 * @byul.side_effect consumes:c
 */
BYUL_API void coord_destroy(coord_t* c);

/**
 * @brief 좌표의 독립 복사본을 생성한다.
 *
 * @param[in] c 복사할 좌표.
 * @return Caller가 소유하는 새 좌표.
 * @retval NULL c가 NULL이다.
 *
 * @byul.nullable c true
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.error null-return
 */
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
BYUL_API coord_t make_tmp_coord(int x, int y);

// For backward compatibility, will be removed later.
// Returns the closest neighbor moving from start to goal.
BYUL_API coord_t* coord_clone_next_to_goal(
    const coord_t* start, const coord_t* goal);

/**
 * @brief Convert coord value to a string
 * @param c coord to convert
 * @param buffer_size buffer size
 * @param buffer output buffer (recommend at least 32 bytes) 
 * @return buffer (returned for convenience)
 */
BYUL_API char* coord_to_string(
    const coord_t* c, size_t buffer_size, char* buffer);

/**
 * @brief Print coord value to console
 * @param c coord to print
 */
BYUL_API void coord_print(const coord_t* c);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_COORD_H */
