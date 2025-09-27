#ifndef GROUND_H
#define GROUND_H

#ifdef __cplusplus
extern "C" {
#endif

#include "byul_common.h"
#include "vec3.h"
#include "bodyprops.h"
#include "plane.h"
#include "coord.h"
#include "coord_hash.h"

/**
 * @file ground.h
 * @brief Ground representation and query API (uniform, tiles, heightfield).
 *
 * External contract:
 *  - ground_sample_at(): get surface point, normal, and bodyprops at pos
 *  - ground_raycast(): ray vs ground, returns hit point, normal, bodyprops
 *  - ground_material_at(): only material lookup at pos
 *
 * Internal representations are hidden behind ground_mode_t.
 */

/* ------------------------------------------------------------------ */
/* Modes                                                              */
/* ------------------------------------------------------------------ */
typedef enum ground_mode_e {
    GROUND_MODE_UNIFORM = 0,   /* one infinite plane + one bodyprops */
    GROUND_MODE_TILES   = 1,   /* per-cell overrides via coord_hash */
    GROUND_MODE_HEIGHTFIELD = 2/* regular grid heightmap */
} ground_mode_t;

/* ------------------------------------------------------------------ */
/* Tiles world->coord mapper                                          */
/* ------------------------------------------------------------------ */
typedef void (*ground_tile_mapper_cb)(
    const void* context,
    const vec3_t* pos_world,
    coord_t* out_coord);

/* ------------------------------------------------------------------ */
/* Concrete payloads (publicly visible to allow stack allocation)     */
/* ------------------------------------------------------------------ */

/* Uniform plane + material */
typedef struct s_ground_uniform {
    bodyprops_t body;
    plane_t     plane;
} ground_uniform_t;

/* Tiles: sparse overrides for bodyprops and/or plane */
typedef struct s_ground_tiles {
    coord_hash_t* bodyprops_table; /* coord_t -> bodyprops_t (value copy) */
    coord_hash_t* plane_table;     /* coord_t -> plane_t (value copy) */
    ground_tile_mapper_cb map_cb;  /* required for world->coord mapping */
    const void* map_context;       /* opaque mapper context */
} ground_tiles_t;

/* Heightfield: regular grid, world units */
typedef struct s_ground_heightfield {
    int   w, h;          /* grid resolution in cells */
    float cell;          /* cell size in world meters */
    float* height;       /* length = w*h, row-major: H[y*w + x] */
    bool  owns_buffer;   /* if true, ground_free() frees height */
} ground_heightfield_t;

/* ------------------------------------------------------------------ */
/* Public handle                                                      */
/* ------------------------------------------------------------------ */
typedef struct s_ground {
    ground_mode_t mode;
    union {
        ground_uniform_t     uniform;
        ground_tiles_t       tiles;
        ground_heightfield_t heightfield;
    } u;
} ground_t;

/* ------------------------------------------------------------------ */
/* Lifecycle                                                          */
/* ------------------------------------------------------------------ */

// normal (0, 1,0) plane at y=0
BYUL_API void ground_init(ground_t* g);

/**
 * @brief Initialize as uniform ground.
 * Copies body and plane by value.
 */
BYUL_API void ground_init_uniform(
    ground_t* g,
    const bodyprops_t* body,
    const plane_t* plane);

/**
 * @brief Initialize as tiles ground.
 * The tables are borrowed; caller retains ownership and lifetime.
 * map_cb must not be NULL.
 */
BYUL_API void ground_init_tiles(
    ground_t* g,
    coord_hash_t* bodyprops_table,
    coord_hash_t* plane_table,
    ground_tile_mapper_cb map_cb,
    const void* map_context);

/**
 * @brief Initialize as heightfield ground.
 * If owns_buffer is true, ground_free() will free 'height'.
 * Otherwise, the pointer is borrowed.
 */
BYUL_API void ground_init_heightfield(
    ground_t* g,
    int w, int h, float cell,
    float* height,
    bool owns_buffer);

BYUL_API void ground_assign(ground_t* out, const ground_t* src);

/**
 * @brief Reset to zero state (no deallocation).
 */
BYUL_API void ground_reset(ground_t* g);

/**
 * @brief Release owned resources (heightfield buffer if owns_buffer).
 * Safe to call on any mode; leaves g in reset state.
 */
BYUL_API void ground_free(ground_t* g);

/* ------------------------------------------------------------------ */
/* Queries                                                            */
/* ------------------------------------------------------------------ */

/**
 * @brief Sample ground at world position.
 *
 * @param g          Ground handle.
 * @param pos_world  World position (x,y used; z ignored as query height).
 * @param out_point  Surface point in world space (nullable).
 * @param out_normal Surface normal (unit) in world space (nullable).
 * @param out_body   Body properties at this location (nullable).
 * @return true if sampling succeeded; false if outside/invalid.
 */
BYUL_API bool ground_sample_at(
    const ground_t* g,
    const vec3_t* pos_world,
    vec3_t* out_point,
    vec3_t* out_normal,
    bodyprops_t* out_body);

/**
 * @brief Raycast against ground.
 *
 * @param g          Ground handle.
 * @param origin     Ray origin (world).
 * @param dir        Ray direction (need not be normalized).
 * @param max_dist   Maximum distance along the ray.
 * @param out_point  Hit point (nullable).
 * @param out_normal Hit normal (unit) (nullable).
 * @param out_body   Body properties at hit (nullable).
 * @param out_t      Parametric distance along ray (nullable).
 * @return true on hit, false otherwise.
 */
BYUL_API bool ground_raycast(
    const ground_t* g,
    const vec3_t* origin,
    const vec3_t* dir,
    float max_dist,
    vec3_t* out_point,
    vec3_t* out_normal,
    bodyprops_t* out_body,
    float* out_t);

/**
 * @brief Fetch only material properties at world position.
 * Fallback order: tiles override -> heightfield default -> uniform.
 */
BYUL_API bool ground_material_at(
    const ground_t* g,
    const vec3_t* pos_world,
    bodyprops_t* out_body);

#ifdef __cplusplus
}
#endif

#endif /* GROUND_H */
