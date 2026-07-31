/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file navcell.h
 * @brief Navigation grid cell values and their checked public C ABI.
 *
 * Declares the stable terrain enum and trivial Navcell value layout together with
 * checked construction, validation, copying, and ABI 1.x compatibility functions.
 */

#ifndef BYUL_NAVCELL_H
#define BYUL_NAVCELL_H

#include <stdbool.h>
#include <stdint.h>

#include "byul_config.h"
#include "navsys_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Stable terrain identifiers stored by navcell_t.
 *
 * Navcell validates these identifiers but does not assign traversal costs or
 * elevation semantics. Navgrid and route-finder policies interpret the value.
 *
 * @byul.storage basic-value
 * @byul.zero_valid true
 * @byul.copy_semantics trivial-copy
 * @byul.thread_safety thread-safe
 * @byul.enum_support type query:navcell_is_terrain_supported
 */
typedef enum e_terrain_type {
    TERRAIN_TYPE_NORMAL = 0, /**< Default passable terrain identifier. */
    TERRAIN_TYPE_WATER = 1, /**< Water terrain identifier; no built-in cost. */
    TERRAIN_TYPE_FOREST = 2, /**< Forest terrain identifier; no built-in cost. */
    TERRAIN_TYPE_MOUNTAIN = 3, /**< Mountain terrain identifier; no built-in cost. */
    TERRAIN_TYPE_FORBIDDEN = 100 /**< Built-in Navgrid policy treats this as blocked. */
} terrain_type_t;

/**
 * @brief Trivial navigation cell value stored in a Navgrid.
 *
 * Zero initialization produces NORMAL terrain with height 0. The height field is
 * unitless ABI 1.x metadata and is not interpreted by Navcell or Navgrid.
 *
 * @byul.storage basic-value
 * @byul.zero_valid true
 * @byul.copy_semantics trivial-copy
 * @byul.thread_safety thread-compatible
 */
typedef struct s_navcell {
    terrain_type_t terrain; /**< Validated terrain identifier. */
    int height; /**< Unitless application-defined metadata. */
} navcell_t;

/**
 * @brief terrain 값이 현재 Navcell ABI에서 지원되는지 조회한다.
 *
 * 알 수 없는 enum 값은 오류가 아니며 out_supported에 false를 기록한다.
 *
 * @param[in] terrain 조회할 terrain 값.
 * @param[out] out_supported 지원 여부를 받을 storage.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 지원 여부를 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT out_supported가 NULL이다.
 * @byul.nullable out_supported false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_supported-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t navcell_is_terrain_supported(
    terrain_type_t terrain, bool* out_supported);

/**
 * @brief Navcell 값이 현재 terrain 계약을 만족하는지 검증한다.
 *
 * height는 ABI 1.x의 unitless metadata이며 모든 int 값이 유효하다.
 *
 * @param[in] cell 검증할 Navcell 값.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK cell이 유효하다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT cell이 NULL이다.
 * @retval NAVSYS_STATUS_UNSUPPORTED terrain 값이 지원되지 않는다.
 * @byul.nullable cell false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect none
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t navcell_validate(const navcell_t* cell);

/**
 * @brief caller storage에 검증된 Navcell 값을 초기화한다.
 *
 * 실패하면 out_cell을 변경하지 않는다.
 *
 * @param[out] out_cell 초기화할 caller storage.
 * @param[in] terrain 저장할 terrain 값.
 * @param[in] height 저장할 unitless metadata.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 값을 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT out_cell이 NULL이다.
 * @retval NAVSYS_STATUS_UNSUPPORTED terrain 값이 지원되지 않는다.
 * @byul.nullable out_cell false
 * @byul.enum_support terrain query:navcell_is_terrain_supported
 * @byul.unit height unitless
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_cell-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t navcell_init_checked(
    navcell_t* out_cell, terrain_type_t terrain, int32_t height);

/**
 * @brief 검증된 Navcell 값을 할당한다.
 *
 * 실패하면 out_cell이 가리키는 값을 변경하지 않는다. 성공한 결과는 caller가
 * navcell_destroy()로 해제한다.
 *
 * @param[in] terrain 저장할 terrain 값.
 * @param[in] height 저장할 unitless metadata.
 * @param[out] out_cell 새 Navcell pointer를 받을 storage.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK Navcell을 할당했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT out_cell이 NULL이다.
 * @retval NAVSYS_STATUS_UNSUPPORTED terrain 값이 지원되지 않는다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY storage를 할당하지 못했다.
 * @byul.nullable out_cell false
 * @byul.enum_support terrain query:navcell_is_terrain_supported
 * @byul.unit height unitless
 * @byul.lifetime out_cell caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_cell-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t navcell_create_checked(
    terrain_type_t terrain, int32_t height, navcell_t** out_cell);

/**
 * @brief 유효한 Navcell의 독립 복사본을 할당한다.
 *
 * 실패하면 out_cell이 가리키는 값을 변경하지 않는다. 성공한 결과는 caller가
 * navcell_destroy()로 해제한다.
 *
 * @param[in] source 복사할 Navcell 값.
 * @param[out] out_cell 새 Navcell pointer를 받을 storage.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK Navcell을 복사했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT source 또는 out_cell이 NULL이다.
 * @retval NAVSYS_STATUS_UNSUPPORTED source의 terrain 값이 지원되지 않는다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY storage를 할당하지 못했다.
 * @byul.nullable source false
 * @byul.nullable out_cell false
 * @byul.lifetime out_cell caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_cell-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t navcell_copy_checked(
    const navcell_t* source, navcell_t** out_cell);

/**
 * @brief 유효한 Navcell 값을 caller storage에 대입한다.
 *
 * source가 유효하지 않으면 out_cell을 변경하지 않는다. Self assignment를 허용한다.
 *
 * @param[out] out_cell 값을 받을 caller storage.
 * @param[in] source 복사할 Navcell 값.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 값을 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer가 NULL이다.
 * @retval NAVSYS_STATUS_UNSUPPORTED source의 terrain 값이 지원되지 않는다.
 * @byul.nullable out_cell false
 * @byul.nullable source false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_cell-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t navcell_assign_checked(
    navcell_t* out_cell, const navcell_t* source);

/**
 * @brief ABI 1.x adapter that allocates a validated Navcell value.
 *
 * New code should use navcell_create_checked() to distinguish invalid terrain from
 * allocation failure. A successful result is released with navcell_destroy().
 *
 * @deprecated Use navcell_create_checked().
 * @param[in] terrain Terrain identifier to store.
 * @param[in] height Unitless application-defined metadata.
 * @return Caller-owned Navcell, or NULL for unsupported terrain or allocation failure.
 * @byul.nullable return true
 * @byul.enum_support terrain query:navcell_is_terrain_supported
 * @byul.unit height unitless
 * @byul.lifetime return caller-owned
 * @byul.memory return navcell_destroy
 * @byul.error null-return
 * @byul.side_effect allocates:return-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navcell_t* navcell_create_full(terrain_type_t terrain, int height);

/**
 * @brief ABI 1.x adapter that allocates the zero-valid default Navcell.
 *
 * The result contains NORMAL terrain and height 0 and is released with
 * navcell_destroy().
 *
 * @deprecated Use navcell_create_checked().
 * @return Caller-owned default Navcell, or NULL on allocation failure.
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.memory return navcell_destroy
 * @byul.error null-return
 * @byul.side_effect allocates:return-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navcell_t* navcell_create();

/**
 * @brief Releases a Navcell allocated by this library.
 *
 * @param[in] nc Navcell to release. NULL is accepted and has no effect.
 * @byul.nullable nc true
 * @byul.side_effect consumes:nc
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API void navcell_destroy(navcell_t* nc);

/**
 * @brief ABI 1.x adapter that allocates an independent validated copy.
 *
 * New code should use navcell_copy_checked() to distinguish invalid input from
 * allocation failure. A successful result is released with navcell_destroy().
 *
 * @deprecated Use navcell_copy_checked().
 * @param[in] nc Navcell value to copy.
 * @return Caller-owned copy, or NULL for NULL/unsupported input or allocation failure.
 * @byul.nullable nc true
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.memory return navcell_destroy
 * @byul.error null-return
 * @byul.side_effect allocates:return-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navcell_t* navcell_copy(const navcell_t* nc);

/**
 * @brief ABI 1.x adapter that initializes caller storage after validation.
 *
 * The destination is preserved on failure. New code should use
 * navcell_init_checked().
 *
 * @deprecated Use navcell_init_checked().
 * @param[out] nc Caller storage to initialize.
 * @param[in] terrain Terrain identifier to store.
 * @param[in] height Unitless application-defined metadata.
 * @return 0 on success or a negative Navsys status value on failure.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT nc is NULL.
 * @retval NAVSYS_STATUS_UNSUPPORTED terrain is unsupported.
 * @byul.nullable nc false
 * @byul.enum_support terrain query:navcell_is_terrain_supported
 * @byul.unit height unitless
 * @byul.error negative-status
 * @byul.side_effect mutates:nc-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API int navcell_init_full(
    navcell_t* nc, terrain_type_t terrain, int height);

/**
 * @brief ABI 1.x adapter that initializes caller storage to NORMAL/0.
 *
 * @deprecated Use navcell_init_checked().
 * @param[out] nc Caller storage to initialize.
 * @return 0 on success or NAVSYS_STATUS_INVALID_ARGUMENT when nc is NULL.
 * @byul.nullable nc false
 * @byul.error negative-status
 * @byul.side_effect mutates:nc-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API int navcell_init(navcell_t* nc);

/**
 * @brief ABI 1.x adapter that assigns a validated Navcell value.
 *
 * The destination is preserved on failure and self-assignment is supported. New
 * code should use navcell_assign_checked().
 *
 * @deprecated Use navcell_assign_checked().
 * @param[out] nc Destination caller storage.
 * @param[in] src Source Navcell value.
 * @return 0 on success or a negative Navsys status value on failure.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT nc or src is NULL.
 * @retval NAVSYS_STATUS_UNSUPPORTED src contains an unsupported terrain.
 * @byul.nullable nc false
 * @byul.nullable src false
 * @byul.error negative-status
 * @byul.side_effect mutates:nc-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API int navcell_assign(navcell_t* nc, const navcell_t* src);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_NAVCELL_H */
