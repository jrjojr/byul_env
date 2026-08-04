/*
 * Copyright (c) 2025-2026 ByulPapa (byuldev@outlook.kr)
 * This file is part of the Byul World project.
 * Licensed under the Byul World Source-Available Non-Commercial License v1.0 (2025).
 * See the LICENSE file in the project root for full license terms.
 */

/**
 * @file coord_list.h
 * @brief 2차원 좌표 sequence를 위한 public C ABI를 선언한다.
 *
 * Opaque 좌표 list의 생성, 값 복사, 안전한 조회와 mutation API를 제공한다.
 * 개별 canonical operation은 내부 동기화된다. Legacy borrowed pointer를 사용하는
 * 동안에는 caller가 같은 list의 mutation을 외부 동기화해야 한다.
 */

#ifndef BYUL_COORD_LIST_H
#define BYUL_COORD_LIST_H

#include <stdbool.h>
#include <stddef.h>

#include "byul_config.h"
#include "coord.h"
#include "navsys_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BYUL이 소유하는 opaque 좌표 sequence다.
 *
 * @byul.storage opaque-object
 * @byul.copy_semantics deep-copy
 * @byul.thread_safety thread-safe
 */
typedef struct s_coord_list coord_list_t;

/**
 * @brief 빈 좌표 list를 생성한다.
 *
 * @param[out] out_list 성공할 때 새 list를 받는다.
 * @return Navsys 상태 코드.
 * @retval NAVSYS_STATUS_OK 생성했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT out_list가 NULL이다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY storage를 할당하지 못했다.
 * @byul.nullable out_list false
 * @byul.lifetime out_list caller-owned
 * @byul.error NAVSYS_STATUS_INVALID_ARGUMENT
 * @byul.error NAVSYS_STATUS_OUT_OF_MEMORY
 */
BYUL_API navsys_status_t coord_list_create_ex(coord_list_t** out_list);

/**
 * @brief 좌표 list를 deep-copy한다.
 *
 * @param[in] source 복사할 list다.
 * @param[out] out_list 성공할 때 새 list를 받는다.
 * @return Navsys 상태 코드.
 * @retval NAVSYS_STATUS_OK 복사했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 인자가 NULL이다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY storage를 할당하지 못했다.
 * @byul.nullable source false
 * @byul.nullable out_list false
 * @byul.lifetime out_list caller-owned
 * @byul.error NAVSYS_STATUS_INVALID_ARGUMENT
 * @byul.error NAVSYS_STATUS_OUT_OF_MEMORY
 */
BYUL_API navsys_status_t coord_list_copy_ex(
    const coord_list_t* source, coord_list_t** out_list);

/**
 * @brief list의 좌표 수를 반환한다.
 *
 * @param[in] list 조회할 list다. NULL이면 0을 반환한다.
 * @return 저장된 좌표 수다.
 * @byul.nullable list true
 */
BYUL_API size_t coord_list_size(const coord_list_t* list);

/**
 * @brief index의 좌표를 caller storage로 복사한다.
 *
 * @param[in] list 조회할 list다.
 * @param[in] index 0부터 시작하는 index다.
 * @param[out] out_coord 성공할 때 좌표를 받는다.
 * @return Navsys 상태 코드.
 * @retval NAVSYS_STATUS_OK 좌표를 복사했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT list 또는 out_coord가 NULL이다.
 * @retval NAVSYS_STATUS_NOT_FOUND index가 범위를 벗어났다.
 * @byul.nullable list false
 * @byul.nullable out_coord false
 * @byul.error NAVSYS_STATUS_INVALID_ARGUMENT
 * @byul.error NAVSYS_STATUS_NOT_FOUND
 */
BYUL_API navsys_status_t coord_list_fetch(
    const coord_list_t* list, size_t index, coord_t* out_coord);

/**
 * @brief 첫 좌표를 caller storage로 복사한다.
 *
 * @param[in] list 조회할 list다.
 * @param[out] out_coord 성공할 때 좌표를 받는다.
 * @return Navsys 상태 코드.
 * @retval NAVSYS_STATUS_OK 좌표를 복사했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 인자가 NULL이다.
 * @retval NAVSYS_STATUS_NOT_FOUND list가 비었다.
 * @byul.nullable list false
 * @byul.nullable out_coord false
 * @byul.error NAVSYS_STATUS_INVALID_ARGUMENT
 * @byul.error NAVSYS_STATUS_NOT_FOUND
 */
BYUL_API navsys_status_t coord_list_fetch_front(
    const coord_list_t* list, coord_t* out_coord);

/**
 * @brief 마지막 좌표를 caller storage로 복사한다.
 *
 * @param[in] list 조회할 list다.
 * @param[out] out_coord 성공할 때 좌표를 받는다.
 * @return Navsys 상태 코드.
 * @retval NAVSYS_STATUS_OK 좌표를 복사했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 인자가 NULL이다.
 * @retval NAVSYS_STATUS_NOT_FOUND list가 비었다.
 * @byul.nullable list false
 * @byul.nullable out_coord false
 * @byul.error NAVSYS_STATUS_INVALID_ARGUMENT
 * @byul.error NAVSYS_STATUS_NOT_FOUND
 */
BYUL_API navsys_status_t coord_list_fetch_back(
    const coord_list_t* list, coord_t* out_coord);

/**
 * @brief 좌표를 list 끝에 복사해 추가한다.
 *
 * @param[in,out] list 변경할 list다.
 * @param[in] coord 복사할 좌표다. list 내부 element를 가리켜도 된다.
 * @return Navsys 상태 코드.
 * @retval NAVSYS_STATUS_OK 추가했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 인자가 NULL이다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY storage를 늘리지 못했다.
 * @byul.nullable list false
 * @byul.nullable coord false
 * @byul.invalidates list element-pointers
 * @byul.error NAVSYS_STATUS_INVALID_ARGUMENT
 * @byul.error NAVSYS_STATUS_OUT_OF_MEMORY
 */
BYUL_API navsys_status_t coord_list_push_back_ex(
    coord_list_t* list, const coord_t* coord);

/**
 * @brief 마지막 좌표를 제거해 caller storage로 복사한다.
 *
 * @param[in,out] list 변경할 list다.
 * @param[out] out_coord 성공할 때 제거된 좌표를 받는다.
 * @return Navsys 상태 코드.
 * @retval NAVSYS_STATUS_OK 제거했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 인자가 NULL이다.
 * @retval NAVSYS_STATUS_NOT_FOUND list가 비었다.
 * @byul.nullable list false
 * @byul.nullable out_coord false
 * @byul.invalidates list element-pointers
 * @byul.error NAVSYS_STATUS_INVALID_ARGUMENT
 * @byul.error NAVSYS_STATUS_NOT_FOUND
 */
BYUL_API navsys_status_t coord_list_try_pop_back(
    coord_list_t* list, coord_t* out_coord);

/**
 * @brief 첫 좌표를 제거해 caller storage로 복사한다.
 *
 * @param[in,out] list 변경할 list다.
 * @param[out] out_coord 성공할 때 제거된 좌표를 받는다.
 * @return Navsys 상태 코드.
 * @retval NAVSYS_STATUS_OK 제거했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 인자가 NULL이다.
 * @retval NAVSYS_STATUS_NOT_FOUND list가 비었다.
 * @byul.nullable list false
 * @byul.nullable out_coord false
 * @byul.invalidates list element-pointers
 * @byul.error NAVSYS_STATUS_INVALID_ARGUMENT
 * @byul.error NAVSYS_STATUS_NOT_FOUND
 */
BYUL_API navsys_status_t coord_list_try_pop_front(
    coord_list_t* list, coord_t* out_coord);

/**
 * @brief index 앞에 좌표를 복사해 삽입한다.
 *
 * index가 size와 같으면 append한다.
 *
 * @param[in,out] list 변경할 list다.
 * @param[in] index 0 이상 size 이하의 index다.
 * @param[in] coord 복사할 좌표다. list 내부 element를 가리켜도 된다.
 * @return Navsys 상태 코드.
 * @retval NAVSYS_STATUS_OK 삽입했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 인자가 NULL이거나 index가 size보다 크다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY storage를 늘리지 못했다.
 * @byul.nullable list false
 * @byul.nullable coord false
 * @byul.invalidates list element-pointers
 * @byul.error NAVSYS_STATUS_INVALID_ARGUMENT
 * @byul.error NAVSYS_STATUS_OUT_OF_MEMORY
 */
BYUL_API navsys_status_t coord_list_insert_ex(
    coord_list_t* list, size_t index, const coord_t* coord);

/**
 * @brief index의 좌표를 제거한다.
 *
 * @param[in,out] list 변경할 list다.
 * @param[in] index 제거할 index다.
 * @param[out] out_removed 성공할 때 제거된 좌표를 받는다.
 * @return Navsys 상태 코드.
 * @retval NAVSYS_STATUS_OK 제거했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 인자가 NULL이다.
 * @retval NAVSYS_STATUS_NOT_FOUND index가 범위를 벗어났다.
 * @byul.nullable list false
 * @byul.nullable out_removed false
 * @byul.invalidates list element-pointers
 * @byul.error NAVSYS_STATUS_INVALID_ARGUMENT
 * @byul.error NAVSYS_STATUS_NOT_FOUND
 */
BYUL_API navsys_status_t coord_list_remove_at_ex(
    coord_list_t* list, size_t index, coord_t* out_removed);

/**
 * @brief 처음 일치하는 좌표 하나를 제거한다.
 *
 * @param[in,out] list 변경할 list다.
 * @param[in] coord 찾을 좌표다. list 내부 element를 가리켜도 된다.
 * @param[out] out_removed 일치 항목을 제거했는지 받는다.
 * @return Navsys 상태 코드.
 * @retval NAVSYS_STATUS_OK 검색을 완료했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 인자가 NULL이다.
 * @byul.nullable list false
 * @byul.nullable coord false
 * @byul.nullable out_removed false
 * @byul.invalidates list element-pointers when-removed
 * @byul.error NAVSYS_STATUS_INVALID_ARGUMENT
 */
BYUL_API navsys_status_t coord_list_remove_value_ex(
    coord_list_t* list, const coord_t* coord, bool* out_removed);

/**
 * @brief 처음 일치하는 좌표의 index를 조회한다.
 *
 * 찾지 못하면 out_found만 false로 쓰고 out_index는 보존한다.
 *
 * @param[in] list 조회할 list다.
 * @param[in] coord 찾을 좌표다.
 * @param[out] out_index 찾았을 때 index를 받는다.
 * @param[out] out_found 일치 여부를 받는다.
 * @return Navsys 상태 코드.
 * @retval NAVSYS_STATUS_OK 검색을 완료했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 인자가 NULL이다.
 * @byul.nullable list false
 * @byul.nullable coord false
 * @byul.nullable out_index false
 * @byul.nullable out_found false
 * @byul.error NAVSYS_STATUS_INVALID_ARGUMENT
 */
BYUL_API navsys_status_t coord_list_find_ex(
    const coord_list_t* list, const coord_t* coord,
    size_t* out_index, bool* out_found);

/**
 * @brief 최소 capacity를 미리 확보한다.
 *
 * @param[in,out] list 변경할 list다.
 * @param[in] capacity 요청할 최소 element capacity다.
 * @return Navsys 상태 코드.
 * @retval NAVSYS_STATUS_OK capacity를 확보했거나 이미 충분하다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT list가 NULL이다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY storage를 할당하지 못했다.
 * @retval NAVSYS_STATUS_LIMIT_REACHED capacity가 구현 한계를 넘는다.
 * @byul.nullable list false
 * @byul.invalidates list element-pointers on-reallocation
 * @byul.error NAVSYS_STATUS_INVALID_ARGUMENT
 * @byul.error NAVSYS_STATUS_OUT_OF_MEMORY
 * @byul.error NAVSYS_STATUS_LIMIT_REACHED
 */
BYUL_API navsys_status_t coord_list_reserve(
    coord_list_t* list, size_t capacity);

/**
 * @brief source의 [begin,end) 범위를 새 list로 deep-copy한다.
 *
 * begin과 end가 같으면 유효한 빈 list를 만든다.
 *
 * @param[in] source 복사할 list다.
 * @param[in] begin 첫 element index다.
 * @param[in] end 마지막 다음 index다.
 * @param[out] out_list 성공할 때 새 list를 받는다.
 * @return Navsys 상태 코드.
 * @retval NAVSYS_STATUS_OK slice를 생성했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT NULL 또는 잘못된 범위다.
 * @retval NAVSYS_STATUS_OUT_OF_MEMORY storage를 할당하지 못했다.
 * @byul.nullable source false
 * @byul.nullable out_list false
 * @byul.lifetime out_list caller-owned
 * @byul.error NAVSYS_STATUS_INVALID_ARGUMENT
 * @byul.error NAVSYS_STATUS_OUT_OF_MEMORY
 */
BYUL_API navsys_status_t coord_list_create_slice(
    const coord_list_t* source, size_t begin, size_t end,
    coord_list_t** out_list);

/**
 * @brief 두 list의 좌표 sequence가 같은지 비교한다.
 *
 * @param[in] a 비교할 첫 list다.
 * @param[in] b 비교할 둘째 list다.
 * @param[out] out_equal 성공할 때 비교 결과를 받는다.
 * @return Navsys 상태 코드.
 * @retval NAVSYS_STATUS_OK 비교했다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 인자가 NULL이다.
 * @byul.nullable a false
 * @byul.nullable b false
 * @byul.nullable out_equal false
 * @byul.error NAVSYS_STATUS_INVALID_ARGUMENT
 */
BYUL_API navsys_status_t coord_list_equal(
    const coord_list_t* a, const coord_list_t* b, bool* out_equal);

/**
 * @brief 좌표를 caller buffer로 순서대로 export한다.
 *
 * out_coords가 NULL이고 capacity가 0이면 필요한 count만 반환한다. Buffer가
 * 짧으면 좌표 buffer는 보존하고 out_count에 필요한 count를 쓴 뒤
 * NAVSYS_STATUS_INCOMPLETE를 반환한다.
 *
 * @param[in] list export할 list다.
 * @param[out] out_coords 좌표를 받을 caller buffer 또는 NULL이다.
 * @param[in] capacity out_coords의 element capacity다.
 * @param[out] out_count 필요한 좌표 수를 받는다.
 * @return Navsys 상태 코드.
 * @retval NAVSYS_STATUS_OK query 또는 전체 export가 완료됐다.
 * @retval NAVSYS_STATUS_INVALID_ARGUMENT 인자 조합이 잘못됐다.
 * @retval NAVSYS_STATUS_INCOMPLETE buffer가 짧다.
 * @byul.nullable list false
 * @byul.nullable out_coords true
 * @byul.nullable out_count false
 * @byul.error NAVSYS_STATUS_INVALID_ARGUMENT
 * @byul.error NAVSYS_STATUS_INCOMPLETE
 */
BYUL_API navsys_status_t coord_list_export(
    const coord_list_t* list, coord_t* out_coords,
    size_t capacity, size_t* out_count);

/** @brief Legacy 빈 list 생성 adapter다.
 * @return 새 caller-owned list 또는 allocation 실패 시 NULL이다.
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_DEPRECATED("Use coord_list_create_ex; removal is planned for ABI 2.")
BYUL_API coord_list_t* coord_list_create(void);

/** @brief list를 파괴한다.
 * @param[in,out] list 파괴할 list 또는 NULL이다.
 * @byul.nullable list true
 * @byul.lifetime list consumed
 */
BYUL_API void coord_list_destroy(coord_list_t* list);

/** @brief Legacy deep-copy adapter다.
 * @param[in] list 복사할 list다.
 * @return 새 caller-owned list 또는 실패 시 NULL이다.
 * @byul.nullable list false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_DEPRECATED("Use coord_list_copy_ex; removal is planned for ABI 2.")
BYUL_API coord_list_t* coord_list_copy(const coord_list_t* list);

/** @brief Legacy int 길이 adapter다.
 * @param[in] list 조회할 list 또는 NULL이다.
 * @return element 수이며 int 범위를 넘으면 INT_MAX다.
 * @byul.nullable list true
 */
BYUL_DEPRECATED("Use coord_list_size; removal is planned for ABI 2.")
BYUL_API int coord_list_length(const coord_list_t* list);

/** @brief list가 NULL이거나 비었는지 반환한다.
 * @param[in] list 조회할 list 또는 NULL이다.
 * @return 비었으면 true다.
 * @byul.nullable list true
 */
BYUL_API bool coord_list_empty(const coord_list_t* list);

/** @brief Legacy borrowed element 조회 adapter다.
 * @param[in] list 조회할 list다.
 * @param[in] index legacy signed index다.
 * @return borrowed element 또는 실패 시 NULL이다.
 * @byul.nullable list false
 * @byul.nullable return true
 * @byul.lifetime return borrowed:list
 */
BYUL_DEPRECATED("Use coord_list_fetch; removal is planned for ABI 2.")
BYUL_API const coord_t* coord_list_get(const coord_list_t* list, int index);

/** @brief Legacy borrowed first element 조회 adapter다.
 * @param[in] list 조회할 list다.
 * @return borrowed element 또는 empty일 때 NULL이다.
 * @byul.nullable list false
 * @byul.nullable return true
 * @byul.lifetime return borrowed:list
 */
BYUL_DEPRECATED("Use coord_list_fetch_front; removal is planned for ABI 2.")
BYUL_API const coord_t* coord_list_front(const coord_list_t* list);

/** @brief Legacy borrowed last element 조회 adapter다.
 * @param[in] list 조회할 list다.
 * @return borrowed element 또는 empty일 때 NULL이다.
 * @byul.nullable list false
 * @byul.nullable return true
 * @byul.lifetime return borrowed:list
 */
BYUL_DEPRECATED("Use coord_list_fetch_back; removal is planned for ABI 2.")
BYUL_API const coord_t* coord_list_back(const coord_list_t* list);

/** @brief Legacy append adapter다.
 * @param[in,out] list 변경할 list다.
 * @param[in] c 복사할 좌표다.
 * @return 성공하면 1, 실패하면 0이다.
 * @byul.nullable list false
 * @byul.nullable c false
 * @byul.invalidates list element-pointers on-reallocation
 */
BYUL_DEPRECATED("Use coord_list_push_back_ex; removal is planned for ABI 2.")
BYUL_API int coord_list_push_back(coord_list_t* list, const coord_t* c);

/** @brief Legacy last-element pop adapter다.
 * @param[in,out] list 변경할 list 또는 NULL이다.
 * @return 제거한 좌표이며 empty/NULL이면 유효한 (0,0)이다.
 * @byul.nullable list true
 * @byul.invalidates list element-pointers
 */
BYUL_DEPRECATED("Use coord_list_try_pop_back; removal is planned for ABI 2.")
BYUL_API coord_t coord_list_pop_back(coord_list_t* list);

/** @brief Legacy first-element pop adapter다.
 * @param[in,out] list 변경할 list 또는 NULL이다.
 * @return 제거한 좌표이며 empty/NULL이면 유효한 (0,0)이다.
 * @byul.nullable list true
 * @byul.invalidates list element-pointers
 */
BYUL_DEPRECATED("Use coord_list_try_pop_front; removal is planned for ABI 2.")
BYUL_API coord_t coord_list_pop_front(coord_list_t* list);

/** @brief Legacy signed-index insert adapter다.
 * @param[in,out] list 변경할 list다.
 * @param[in] index 삽입 index다.
 * @param[in] c 복사할 좌표다.
 * @return 성공하면 1, 실패하면 0이다.
 * @byul.nullable list false
 * @byul.nullable c false
 * @byul.invalidates list element-pointers on-reallocation
 */
BYUL_DEPRECATED("Use coord_list_insert_ex; removal is planned for ABI 2.")
BYUL_API int coord_list_insert(coord_list_t* list, int index, const coord_t* c);

/** @brief Legacy signed-index remove adapter다.
 * @param[in,out] list 변경할 list다.
 * @param[in] index 제거할 index다.
 * @byul.nullable list false
 * @byul.invalidates list element-pointers when-removed
 */
BYUL_DEPRECATED("Use coord_list_remove_at_ex; removal is planned for ABI 2.")
BYUL_API void coord_list_remove_at(coord_list_t* list, int index);

/** @brief Legacy first-match remove adapter다.
 * @param[in,out] list 변경할 list다.
 * @param[in] c 제거할 좌표다.
 * @byul.nullable list false
 * @byul.nullable c false
 * @byul.invalidates list element-pointers when-removed
 */
BYUL_DEPRECATED("Use coord_list_remove_value_ex; removal is planned for ABI 2.")
BYUL_API void coord_list_remove_value(coord_list_t* list, const coord_t* c);

/** @brief 모든 element를 제거한다.
 * @param[in,out] list 변경할 list 또는 NULL이다.
 * @byul.nullable list true
 * @byul.invalidates list element-pointers
 */
BYUL_API void coord_list_clear(coord_list_t* list);

/** @brief element 순서를 뒤집는다.
 * @param[in,out] list 변경할 list 또는 NULL이다.
 * @byul.nullable list true
 * @byul.invalidates list element-pointers
 */
BYUL_API void coord_list_reverse(coord_list_t* list);

/** @brief Legacy contains adapter다.
 * @param[in] list 조회할 list다.
 * @param[in] c 찾을 좌표다.
 * @return 찾으면 1, 아니면 0이다.
 * @byul.nullable list false
 * @byul.nullable c false
 */
BYUL_DEPRECATED("Use coord_list_find_ex; removal is planned for ABI 2.")
BYUL_API int coord_list_contains(
    const coord_list_t* list, const coord_t* c);

/** @brief Legacy first-index find adapter다.
 * @param[in] list 조회할 list다.
 * @param[in] c 찾을 좌표다.
 * @return 첫 index 또는 찾지 못하면 -1이다.
 * @byul.nullable list false
 * @byul.nullable c false
 */
BYUL_DEPRECATED("Use coord_list_find_ex; removal is planned for ABI 2.")
BYUL_API int coord_list_find(
    const coord_list_t* list, const coord_t* c);

/** @brief Legacy non-empty [start,end) slice adapter다.
 * @param[in] list 복사할 list다.
 * @param[in] start 첫 index다.
 * @param[in] end 마지막 다음 index다.
 * @return 새 caller-owned list이며 empty/invalid/실패 시 NULL이다.
 * @byul.nullable list false
 * @byul.nullable return true
 * @byul.lifetime return caller-owned
 */
BYUL_DEPRECATED("Use coord_list_create_slice; removal is planned for ABI 2.")
BYUL_API coord_list_t* coord_list_sublist(
    const coord_list_t* list, int start, int end);

/** @brief Legacy equality adapter다.
 * @param[in] a 비교할 list다.
 * @param[in] b 비교할 list다.
 * @return 두 non-NULL sequence가 같으면 true다.
 * @byul.nullable a false
 * @byul.nullable b false
 */
BYUL_DEPRECATED("Use coord_list_equal; removal is planned for ABI 2.")
BYUL_API bool coord_list_equals(const coord_list_t* a, const coord_list_t* b);

#ifdef __cplusplus
}
#endif

#endif /* BYUL_COORD_LIST_H */
