#ifndef COST_COORD_PQ_H
#define COST_COORD_PQ_H

#include "internal/coord.h"   // coord_t
#include "byul_config.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct s_cost_coord_pq cost_coord_pq_t;

// ------------------------ 생성/해제 ------------------------

/// @brief float 우선순위 기반 coord_t* 저장 큐 생성
BYUL_API cost_coord_pq_t* cost_coord_pq_new();

/// @brief 큐 해제
BYUL_API void cost_coord_pq_free(cost_coord_pq_t* pq);

// ------------------------ 삽입/조회 ------------------------

/// @brief (비용, 좌표) 쌍 삽입
BYUL_API void cost_coord_pq_push(cost_coord_pq_t* pq, float cost, 
    const coord_t* c);

/// @brief 현재 최소 비용 좌표 조회 (삭제하지 않음)
BYUL_API coord_t* cost_coord_pq_peek(cost_coord_pq_t* pq);

/// @brief 현재 최소 비용 좌표 제거 후 반환
BYUL_API coord_t* cost_coord_pq_pop(cost_coord_pq_t* pq);

/// @brief 최소 비용 값 (float)만 조회
BYUL_API float cost_coord_pq_peek_cost(cost_coord_pq_t* pq);

// ------------------------ 검사/삭제 ------------------------

/// @brief 큐가 비었는지 여부
BYUL_API bool cost_coord_pq_is_empty(cost_coord_pq_t* pq);

/// @brief 해당 좌표가 큐에 존재하는지 확인
BYUL_API bool cost_coord_pq_contains(cost_coord_pq_t* pq, const coord_t* c);

/// @brief 해당 좌표의 비용을 제거 (비용 값은 알아야 함)
BYUL_API bool cost_coord_pq_remove(
    cost_coord_pq_t* pq, float cost, const coord_t* c);

BYUL_API int cost_coord_pq_length(cost_coord_pq_t* pq);
BYUL_API void cost_coord_pq_trim_worst(cost_coord_pq_t* pq, int n);


#ifdef __cplusplus
}
#endif

#endif // COST_COORD_PQ_H
