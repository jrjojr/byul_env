#ifndef DSTAR_LITE_PQUEUE_H
#define DSTAR_LITE_PQUEUE_H

#include "byul_config.h"
#include "internal/coord.h"
#include "internal/dstar_lite_key.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct s_dstar_lite_pqueue dstar_lite_pqueue_t;

// ------------------------ 생성/소멸/복사 ------------------------

BYUL_API dstar_lite_pqueue_t* dstar_lite_pqueue_create(void);

BYUL_API void dstar_lite_pqueue_destroy(dstar_lite_pqueue_t* q);

BYUL_API dstar_lite_pqueue_t* dstar_lite_pqueue_copy(const dstar_lite_pqueue_t* src);

// ------------------------ 삽입/조회/삭제 ------------------------

BYUL_API void dstar_lite_pqueue_push(
    dstar_lite_pqueue_t* q,
    const dstar_lite_key_t* key,
    const coord_t* c);

BYUL_API const coord_t* dstar_lite_pqueue_peek(dstar_lite_pqueue_t* q);

BYUL_API coord_t* dstar_lite_pqueue_pop(dstar_lite_pqueue_t* q);

BYUL_API bool dstar_lite_pqueue_is_empty(dstar_lite_pqueue_t* q);

// ------------------------ 삭제/조회 함수 ------------------------

/// @brief 해당 coord_t* 를 가진 요소 제거
BYUL_API bool dstar_lite_pqueue_remove(dstar_lite_pqueue_t* q, const coord_t* u);

/// @brief key + coord 쌍이 정확히 일치하는 항목 제거
BYUL_API bool dstar_lite_pqueue_remove_full(
    dstar_lite_pqueue_t* q,
    const dstar_lite_key_t* key,
    const coord_t* c);

/// @brief coord에 해당하는 key 복사본 반환 (없으면 NULL)
BYUL_API dstar_lite_key_t* dstar_lite_pqueue_get_key_by_coord(
    dstar_lite_pqueue_t* q, const coord_t* c);

/// @brief top 우선순위의 key 복사본 반환
BYUL_API dstar_lite_key_t* dstar_lite_pqueue_top_key(dstar_lite_pqueue_t* q);

/// @brief 해당 coord가 큐에 존재하는지 확인
BYUL_API bool dstar_lite_pqueue_contains(
    dstar_lite_pqueue_t* q, const coord_t* u);

#ifdef __cplusplus
}
#endif

#endif // DSTAR_LITE_PQUEUE_H
