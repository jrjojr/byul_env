#ifndef NAVCELL_H
#define NAVCELL_H

#include "byul_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum e_terrain_type {
    TERRAIN_TYPE_NORMAL,
    TERRAIN_TYPE_WATER,
    TERRAIN_TYPE_FOREST,
    TERRAIN_TYPE_MOUNTAIN,
    TERRAIN_TYPE_FORBIDDEN = 100    
} terrain_type_t;

typedef struct {
    terrain_type_t terrain;  // terrain type
    int height;           
} navcell_t;

BYUL_API navcell_t* navcell_create_full(terrain_type_t terrain, int height);

BYUL_API navcell_t* navcell_create();

BYUL_API void navcell_destroy(navcell_t* nc);

BYUL_API navcell_t* navcell_copy(const navcell_t* nc);

BYUL_API int navcell_init_full(
    navcell_t* nc, terrain_type_t terrain, int height);

BYUL_API int navcell_init(navcell_t* nc);
BYUL_API int navcell_assign(navcell_t* nc, const navcell_t* src);

#ifdef __cplusplus
}
#endif

#endif // NAVCELL_H
