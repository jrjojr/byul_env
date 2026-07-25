/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file dstar_lite_key.h
 * @brief D* Lite 우선순위 key 값의 public C ABI를 선언한다.
 *
 * 두 개의 float 성분으로 이루어진 기본 값, ABI layout 진단, canonical 초기화와
 * exact equality, lexicographic ordering 및 hash 연산을 제공하는 public component다.
 */

#ifndef BYUL_DSTAR_LITE_KEY_H
#define BYUL_DSTAR_LITE_KEY_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "byul_config.h"
#include "navsys_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief D* Lite open list에서 사용하는 두 성분 우선순위 key다.
 *
 * Canonical key는 finite 값과 양의 infinity만 허용하며 음의 zero는 양의 zero로
 * 정규화한다. NaN과 음의 infinity는 canonical key가 아니다.
 *
 * @byul.storage basic-value
 * @byul.zero_valid true
 * @byul.copy_semantics trivial-copy
 * @byul.thread_safety thread-safe
 */
typedef struct s_dstar_lite_key {
    float k1; /**< 첫 번째 lexicographic 우선순위 성분. */
    float k2; /**< 두 번째 lexicographic 우선순위 성분. */
} dstar_lite_key_t;

/* ------------------------ ABI Layout Diagnostics ------------------------ */

/**
 * @brief 현재 SDK에서 dstar_lite_key_t의 크기를 반환한다.
 * @return byte 단위의 dstar_lite_key_t 크기.
 * @byul.unit return bytes
 * @byul.error none
 * @byul.side_effect none
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API size_t dstar_lite_key_sizeof(void);

/**
 * @brief 현재 SDK에서 dstar_lite_key_t의 정렬 요구사항을 반환한다.
 * @return byte 단위의 dstar_lite_key_t 정렬 크기.
 * @byul.unit return bytes
 * @byul.error none
 * @byul.side_effect none
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API size_t dstar_lite_key_alignof(void);

/**
 * @brief dstar_lite_key_t의 k1 field offset을 반환한다.
 * @return byte 단위의 k1 field offset.
 * @byul.unit return bytes
 * @byul.error none
 * @byul.side_effect none
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API size_t dstar_lite_key_offsetof_k1(void);

/**
 * @brief dstar_lite_key_t의 k2 field offset을 반환한다.
 * @return byte 단위의 k2 field offset.
 * @byul.unit return bytes
 * @byul.error none
 * @byul.side_effect none
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API size_t dstar_lite_key_offsetof_k2(void);

/* ------------------------ Canonical Value API ------------------------ */

/**
 * @brief caller storage에 canonical D* Lite key를 초기화한다.
 *
 * 음의 zero 성분은 양의 zero로 정규화한다. 실패하면 out_key를 변경하지 않는다.
 *
 * @param[out] out_key 초기화할 caller storage.
 * @param[in] k1 첫 번째 우선순위 성분.
 * @param[in] k2 두 번째 우선순위 성분.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK key를 초기화했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT out_key가 NULL이거나 성분에 NaN 또는 음의
 *     infinity가 포함됐다.
 * @byul.nullable out_key false
 * @byul.unit k1 unitless
 * @byul.unit k2 unitless
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_key-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t dstar_lite_key_init(
    dstar_lite_key_t* out_key, float k1, float k2);

/**
 * @brief canonical D* Lite key를 할당한다.
 *
 * 실패하면 out_key가 가리키는 값을 변경하지 않는다. 성공한 결과는 caller가
 * dstar_lite_key_destroy()로 해제한다.
 *
 * @param[in] k1 첫 번째 우선순위 성분.
 * @param[in] k2 두 번째 우선순위 성분.
 * @param[out] out_key 새 key pointer를 받을 storage.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK key를 할당했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT out_key가 NULL이거나 성분이 canonical domain
 *     밖이다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY key storage를 할당하지 못했다.
 * @byul.nullable out_key false
 * @byul.unit k1 unitless
 * @byul.unit k2 unitless
 * @byul.lifetime out_key caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_key-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t dstar_lite_key_create_ex(
    float k1, float k2, dstar_lite_key_t** out_key);

/**
 * @brief canonical D* Lite key의 독립 복사본을 할당한다.
 *
 * 실패하면 out_key가 가리키는 값을 변경하지 않는다. 성공한 결과는 caller가
 * dstar_lite_key_destroy()로 해제한다.
 *
 * @param[in] source 복사할 canonical key.
 * @param[out] out_key 새 key pointer를 받을 storage.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK key를 복사했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT source 또는 out_key가 NULL이거나 source가
 *     canonical domain 밖이다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY key storage를 할당하지 못했다.
 * @byul.nullable source false
 * @byul.nullable out_key false
 * @byul.lifetime out_key caller-owned
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_key-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t dstar_lite_key_copy_ex(
    const dstar_lite_key_t* source,
    dstar_lite_key_t** out_key);

/**
 * @brief 두 canonical key가 exact equality relation에서 같은지 반환한다.
 * @param[in] lhs 왼쪽 key.
 * @param[in] rhs 오른쪽 key.
 * @return 두 key가 유효하고 canonical field 값이 같으면 true, 아니면 false.
 * @byul.nullable lhs false
 * @byul.nullable rhs false
 * @byul.error false-return
 * @byul.side_effect none
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API bool dstar_lite_key_equal_exact(
    const dstar_lite_key_t* lhs,
    const dstar_lite_key_t* rhs);

/**
 * @brief 두 canonical key의 exact lexicographic 순서를 비교한다.
 *
 * 성공하면 out_compare에 lhs가 rhs보다 작으면 -1, 같으면 0, 크면 1을 기록한다.
 * 실패하면 out_compare를 변경하지 않는다.
 *
 * @param[in] lhs 왼쪽 key.
 * @param[in] rhs 오른쪽 key.
 * @param[out] out_compare 비교 결과를 받을 storage.
 * @return 공통 Navsys 상태 값.
 * @retval NAVSYS_STATUS_OK 비교 결과를 기록했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT pointer가 NULL이거나 key가 canonical domain
 *     밖이다.
 * @byul.nullable lhs false
 * @byul.nullable rhs false
 * @byul.nullable out_compare false
 * @byul.error enum:navsys_status_t
 * @byul.side_effect mutates:out_compare-on-success
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API navsys_status_t dstar_lite_key_compare_exact(
    const dstar_lite_key_t* lhs,
    const dstar_lite_key_t* rhs,
    int* out_compare);

/**
 * @brief canonical key의 exact field hash를 반환한다.
 * @param[in] key hash를 계산할 key.
 * @return canonical key의 32-bit hash. key가 NULL이거나 유효하지 않으면 0.
 * @byul.nullable key false
 * @byul.error none
 * @byul.side_effect none
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API uint32_t dstar_lite_key_hash_exact(
    const dstar_lite_key_t* key);

/* ------------------------ ABI 1.x Compatibility API ------------------------ */

/**
 * @brief 두 성분이 0인 legacy D* Lite key를 할당한다.
 * @return 할당한 key. 할당 실패 시 NULL.
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.error null-return
 * @byul.side_effect allocates
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API dstar_lite_key_t* dstar_lite_key_create(void);

/**
 * @brief 입력 성분을 그대로 저장한 legacy D* Lite key를 할당한다.
 *
 * 이 ABI는 NaN, 양과 음의 infinity, 음의 zero를 정규화나 거부 없이 저장한다.
 *
 * @param[in] k1 첫 번째 key 성분.
 * @param[in] k2 두 번째 key 성분.
 * @return 할당한 key. 할당 실패 시 NULL.
 * @byul.unit k1 unitless
 * @byul.unit k2 unitless
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.error null-return
 * @byul.side_effect allocates
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API dstar_lite_key_t* dstar_lite_key_create_full(float k1, float k2);

/**
 * @brief legacy key의 입력 비트를 보존하는 복사본을 할당한다.
 * @param[in] key 복사할 key.
 * @return 할당한 복사본. key가 NULL이거나 할당에 실패하면 NULL.
 * @byul.nullable key false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 * @byul.error null-return
 * @byul.side_effect allocates
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API dstar_lite_key_t* dstar_lite_key_copy(const dstar_lite_key_t* key);

/**
 * @brief dstar_lite_key_create 계열이 할당한 key를 해제한다.
 * @param[in,out] key 해제할 key. NULL은 허용된다.
 * @byul.nullable key true
 * @byul.lifetime key consumed
 * @byul.error none
 * @byul.side_effect frees:key
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API void dstar_lite_key_destroy(dstar_lite_key_t* key);

/**
 * @brief 두 legacy key를 상대 허용 오차 1e-5로 비교한다.
 * @param[in] dsk0 왼쪽 key.
 * @param[in] dsk1 오른쪽 key.
 * @return 두 pointer가 유효하고 각 성분이 legacy 근사 비교에서 같으면 true.
 * @byul.nullable dsk0 false
 * @byul.nullable dsk1 false
 * @byul.error false-return
 * @byul.side_effect none
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API bool dstar_lite_key_equal(
    const dstar_lite_key_t* dsk0,
    const dstar_lite_key_t* dsk1);

/**
 * @brief legacy 근사 동등성 뒤에 lexicographic 비교를 적용한다.
 * @param[in] dsk0 왼쪽 key.
 * @param[in] dsk1 오른쪽 key.
 * @return 왼쪽이 작으면 -1, legacy 근사 비교에서 같으면 0, 크면 1.
 * @byul.nullable dsk0 false
 * @byul.nullable dsk1 false
 * @byul.error undefined-behavior-on-null
 * @byul.side_effect none
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API int dstar_lite_key_compare(
    const dstar_lite_key_t* dsk0,
    const dstar_lite_key_t* dsk1);

/**
 * @brief 두 float의 원시 비트로 legacy hash를 계산한다.
 * @param[in] key hash를 계산할 key.
 * @return 32-bit legacy hash. key가 NULL이면 0.
 * @byul.nullable key true
 * @byul.error sentinel-return:0
 * @byul.side_effect none
 * @byul.thread_safety thread-safe
 * @byul.blocking false
 */
BYUL_API unsigned int dstar_lite_key_hash(const dstar_lite_key_t* key);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_DSTAR_LITE_KEY_H */
