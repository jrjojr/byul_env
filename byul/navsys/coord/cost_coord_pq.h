#ifndef COST_COORD_PQ_H
#define COST_COORD_PQ_H

#include "coord.h"   // coord_t
#include "byul_common.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct s_cost_coord_pq cost_coord_pq_t;

// ------------------------ Create / Destroy ------------------------

/// @brief Create a queue that stores coord_t* based on float priority
BYUL_API cost_coord_pq_t* cost_coord_pq_create();

/// @brief Destroy the queue
BYUL_API void cost_coord_pq_destroy(cost_coord_pq_t* pq);

// ------------------------ Insert / Retrieve ------------------------

/// @brief Insert a (cost, coordinate) pair
BYUL_API void cost_coord_pq_push(cost_coord_pq_t* pq, float cost, 
    const coord_t* c);

/// @brief Get the current minimum-cost coordinate (does not remove)
BYUL_API coord_t* cost_coord_pq_peek(cost_coord_pq_t* pq);

/// @brief Remove and return the current minimum-cost coordinate
BYUL_API coord_t* cost_coord_pq_pop(cost_coord_pq_t* pq);

/// @brief Get only the minimum cost value (float)
BYUL_API float cost_coord_pq_peek_cost(cost_coord_pq_t* pq);

// ------------------------ Check / Remove ------------------------

/// @brief Check if the queue is empty
BYUL_API bool cost_coord_pq_is_empty(cost_coord_pq_t* pq);

/// @brief Check if a given coordinate exists in the queue
BYUL_API bool cost_coord_pq_contains(cost_coord_pq_t* pq, const coord_t* c);

/// @brief Remove a coordinate with the given cost (cost value must match)
BYUL_API bool cost_coord_pq_remove(
    cost_coord_pq_t* pq, float cost, const coord_t* c);

BYUL_API int cost_coord_pq_length(cost_coord_pq_t* pq);
BYUL_API void cost_coord_pq_trim_worst(cost_coord_pq_t* pq, int n);


#ifdef __cplusplus
}
#endif

#endif // COST_COORD_PQ_H
