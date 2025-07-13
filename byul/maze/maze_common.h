#ifndef MAZE_COMMON_H
#define MAZE_COMMON_H

#include "byul_config.h"
#include "internal/coord.h"
#include "internal/coord_hash.h"
#include "internal/map.h"

#ifdef __cplusplus
extern "C" {
#endif

// maze 구조체 정의
typedef struct s_maze {
    int x0;
    int y0;
    int width;
    int height;
    coord_hash_t* blocked;
} maze_t;

// 기본 생성자 / 소멸자
BYUL_API maze_t* maze_new();
BYUL_API maze_t* maze_new_full(
    int x0, int y0, int width, int height);
BYUL_API void maze_free(maze_t* maze);

BYUL_API void maze_clear(maze_t* maze);

// 복사 및 비교
BYUL_API maze_t* maze_copy(const maze_t* maze);
BYUL_API bool maze_equal(const maze_t* a, const maze_t* b);
BYUL_API uint32_t maze_hash(const maze_t* maze);

// origin 설정 / 조회
BYUL_API void maze_set_origin(maze_t* maze, int x0, int y0);
BYUL_API void maze_get_origin(const maze_t* maze, int* out_x0, int* out_y0);

// 크기 조회
BYUL_API int maze_get_width(const maze_t* maze);
BYUL_API int maze_get_height(const maze_t* maze);

// 차단 좌표 직접 접근 (읽기 전용)
BYUL_API const coord_hash_t* maze_get_blocked_coords(const maze_t* maze);

BYUL_API void maze_apply_to_map(const maze_t* maze, map_t* map);

BYUL_API void maze_remove_from_map(const maze_t* maze, map_t* map);

#ifdef __cplusplus
}
#endif

#endif // MAZE_COMMON_H
