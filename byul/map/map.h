#ifndef MAP_H
#define MAP_H

#include "byul_config.h"
#include "internal/coord.h"
#include "internal/coord_list.h"
#include "internal/coord_hash.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef bool (*is_coord_blocked_func)(
    const void* context, int x, int y, void* userdata);

bool is_coord_blocked_map(const void* context, 
    int x, int y, void* userdata);

typedef enum {
    MAP_NEIGHBOR_4,
    MAP_NEIGHBOR_8
} map_neighbor_mode_t;

struct s_map {
    int width;
    int height;
    map_neighbor_mode_t mode;

    coord_hash_t* blocked_coords;

    is_coord_blocked_func is_coord_blocked_fn;
};

typedef struct s_map map_t;

// 생성자 및 소멸자

// 0 x 0 , MAP_NEIGHBOR_8
BYUL_API map_t* map_new();

BYUL_API map_t* map_new_full(int width, int height, map_neighbor_mode_t mode,
    is_coord_blocked_func is_coord_blocked_fn);

BYUL_API void map_free(map_t* m);

// 복사 및 비교
BYUL_API map_t* map_copy(const map_t* m);
BYUL_API uint32_t map_hash(const map_t* m);
BYUL_API bool map_equal(const map_t* a, const map_t* b);

// 속성 접근
BYUL_API int map_get_width(const map_t* m);
BYUL_API void map_set_width(map_t* m, int width);

BYUL_API int map_get_height(const map_t* m);
BYUL_API void map_set_height(map_t* m, int height);

BYUL_API void map_set_is_coord_blocked_func(map_t* m, is_coord_blocked_func fn);
BYUL_API is_coord_blocked_func map_get_is_coord_blocked_fn(const map_t* m);

BYUL_API map_neighbor_mode_t map_get_mode(const map_t* m);
BYUL_API void map_set_mode(map_t* m);

// 장애물 관련
BYUL_API bool map_block_coord(map_t* m, int x, int y);
BYUL_API bool map_unblock_coord(map_t* m, int x, int y);
BYUL_API bool map_is_inside(const map_t* m, int x, int y);
// BYUL_API bool map_is_blocked(const map_t* m, int x, int y);
BYUL_API void map_clear(map_t* m);

// 차단 좌표 집합 반환
BYUL_API const coord_hash_t* map_get_blocked_coords(const map_t* m);

// 이웃 탐색
BYUL_API coord_list_t* map_make_neighbors(const map_t* m, int x, int y);
BYUL_API coord_list_t* map_make_neighbors_all(const map_t* m, int x, int y);
BYUL_API coord_list_t* map_make_neighbors_all_range(
    map_t* m, int x, int y, int range);

BYUL_API coord_t* map_make_neighbor_at_degree(const map_t* m, 
    int x, int y, double degree);
    
BYUL_API coord_t* map_make_neighbor_at_goal(const map_t* m, 
    const coord_t* center, const coord_t* goal);

BYUL_API coord_list_t* map_make_neighbors_at_degree_range(
    const map_t* m,
    const coord_t* center, const coord_t* goal,
    double start_deg, double end_deg,
    int range);

#ifdef __cplusplus
}
#endif

#endif // MAP_H
