#ifndef COORD_LIST_H
#define COORD_LIST_H

#include <stdint.h>
#include <stdbool.h>
#include "byul_config.h"
#include "internal/coord.h"

#ifdef __cplusplus
extern "C" {
#endif

// opaque 구조체 정의
typedef struct s_coord_list coord_list_t;

// 생성/소멸
BYUL_API coord_list_t* coord_list_new(void);
BYUL_API void coord_list_free(coord_list_t* list);
BYUL_API coord_list_t* coord_list_copy(const coord_list_t* list);

// 정보 조회
BYUL_API int coord_list_length(const coord_list_t* list);
BYUL_API bool coord_list_empty(const coord_list_t* list);
BYUL_API const coord_t* coord_list_get(const coord_list_t* list, int index);
BYUL_API const coord_t* coord_list_front(const coord_list_t* list);
BYUL_API const coord_t* coord_list_back(const coord_list_t* list);

// 수정
BYUL_API int coord_list_push_back(coord_list_t* list, const coord_t* c);

/// @brief 리스트의 마지막 요소를 제거하고 반환 (NULL이면 없음)
BYUL_API coord_t* coord_list_pop_back(coord_list_t* list);

/// @brief 리스트의 첫 번째 요소를 제거하고 반환 (NULL이면 없음)
BYUL_API coord_t* coord_list_pop_front(coord_list_t* list);

BYUL_API int coord_list_insert(coord_list_t* list, int index, const coord_t* c);
BYUL_API void coord_list_remove_at(coord_list_t* list, int index);
BYUL_API void coord_list_remove_value(coord_list_t* list, const coord_t* c);
BYUL_API void coord_list_clear(coord_list_t* list);
BYUL_API void coord_list_reverse(coord_list_t* list);

// 탐색
BYUL_API int  coord_list_contains(const coord_list_t* list, const coord_t* c);  // 1이면 포함
BYUL_API int  coord_list_find(const coord_list_t* list, const coord_t* c);      // index 반환, 없으면 -1

// 부분 추출
BYUL_API coord_list_t* coord_list_sublist(const coord_list_t* list, int start, int end);

// 비교
BYUL_API bool coord_list_equals(const coord_list_t* a, const coord_list_t* b);

#ifdef __cplusplus
}
#endif

#endif // COORD_LIST_H
