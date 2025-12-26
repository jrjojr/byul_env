#ifndef COORD_LIST_H
#define COORD_LIST_H

#include <stdint.h>
#include <stdbool.h>
#include "byul_config.h"
#include "coord.h"

#ifdef __cplusplus
extern "C" {
#endif

// Opaque structure definition
typedef struct s_coord_list coord_list_t;

// Creation/Destruction
BYUL_API coord_list_t* coord_list_create(void);
BYUL_API void coord_list_destroy(coord_list_t* list);
BYUL_API coord_list_t* coord_list_copy(const coord_list_t* list);

// Information
BYUL_API int coord_list_length(const coord_list_t* list);
BYUL_API bool coord_list_empty(const coord_list_t* list);
BYUL_API const coord_t* coord_list_get(const coord_list_t* list, int index);
BYUL_API const coord_t* coord_list_front(const coord_list_t* list);
BYUL_API const coord_t* coord_list_back(const coord_list_t* list);

// Modification
BYUL_API int coord_list_push_back(coord_list_t* list, const coord_t* c);

/// @brief Removes and returns the last element of the list (NULL if none)
BYUL_API coord_t coord_list_pop_back(coord_list_t* list);

/// @brief Removes and returns the first element of the list (NULL if none)
BYUL_API coord_t coord_list_pop_front(coord_list_t* list);

BYUL_API int coord_list_insert(coord_list_t* list, int index, const coord_t* c);
BYUL_API void coord_list_remove_at(coord_list_t* list, int index);
BYUL_API void coord_list_remove_value(coord_list_t* list, const coord_t* c);
BYUL_API void coord_list_clear(coord_list_t* list);
BYUL_API void coord_list_reverse(coord_list_t* list);

// Search
BYUL_API int  coord_list_contains(const coord_list_t* list, const coord_t* c);  // returns 1 if contains
BYUL_API int  coord_list_find(const coord_list_t* list, const coord_t* c);      // returns index, -1 if not found

// Sublist extraction
BYUL_API coord_list_t* coord_list_sublist(const coord_list_t* list, int start, int end);

// Comparison
BYUL_API bool coord_list_equals(const coord_list_t* a, const coord_list_t* b);

#ifdef __cplusplus
}
#endif

#endif // COORD_LIST_H
