#ifndef DSTAR_LITE_PQUEUE_H
#define DSTAR_LITE_PQUEUE_H

#include "byul_common.h"
#include "coord.h"
#include "dstar_lite_key.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct s_dstar_lite_pqueue dstar_lite_pqueue_t;

BYUL_API dstar_lite_pqueue_t* dstar_lite_pqueue_create(void);

BYUL_API void dstar_lite_pqueue_destroy(dstar_lite_pqueue_t* q);

BYUL_API dstar_lite_pqueue_t* dstar_lite_pqueue_copy(const dstar_lite_pqueue_t* src);

BYUL_API void dstar_lite_pqueue_push(
    dstar_lite_pqueue_t* q,
    const dstar_lite_key_t* key,
    const coord_t* c);

BYUL_API const coord_t* dstar_lite_pqueue_peek(dstar_lite_pqueue_t* q);

BYUL_API coord_t* dstar_lite_pqueue_pop(dstar_lite_pqueue_t* q);

BYUL_API bool dstar_lite_pqueue_is_empty(dstar_lite_pqueue_t* q);

BYUL_API bool dstar_lite_pqueue_remove(dstar_lite_pqueue_t* q, const coord_t* u);

BYUL_API bool dstar_lite_pqueue_remove_full(
    dstar_lite_pqueue_t* q,
    const dstar_lite_key_t* key,
    const coord_t* c);

BYUL_API dstar_lite_key_t* dstar_lite_pqueue_get_key_by_coord(
    dstar_lite_pqueue_t* q, const coord_t* c);

BYUL_API dstar_lite_key_t* dstar_lite_pqueue_top_key(dstar_lite_pqueue_t* q);

BYUL_API bool dstar_lite_pqueue_contains(
    dstar_lite_pqueue_t* q, const coord_t* u);

#ifdef __cplusplus
}
#endif

#endif // DSTAR_LITE_PQUEUE_H
