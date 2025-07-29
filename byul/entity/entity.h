#ifndef ENTITY_H
#define ENTITY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "byul_common.h"
#include "coord.h"

// ---------------------------------------------------------
// Basic Entity Structure
// ---------------------------------------------------------
/**
 * @struct s_entity
 * @brief Minimal common properties for all game objects + spatial influence.
 *
 * - coord: Center coordinate of the entity (grid-based).
 * - width_range, height_range: Influence range in horizontal/vertical directions (grid units).
 *   * 0 means no additional range (single cell).
 *   * 1 means center cell + 1 cell on both sides (total 3 cells).
 * - influence_ratio:
 *   * A value from 0.0f upward, representing the influence factor.
 *   * 0 means no influence, 1 means standard influence.
 * - age/lifetime: Lifetime management.
 */
typedef struct s_entity {
    int32_t id;             ///< Unique ID (-1 means unassigned)
    coord_t coord;          ///< Center coordinate (grid-based)
    void* owner;            ///< Owner (another entity_t* or system object)

    // --- Spatial / influence properties ---
    int32_t width_range;    ///< X-axis influence range (grid units, default 0)
    int32_t height_range;   ///< Y-axis influence range (grid units, default 0)
    float   influence_ratio;///< Influence ratio (0 = none, 1 = default, >1 = extended)

    // --- Time properties ---
    float age;              ///< Time elapsed since creation (seconds)
    float lifetime;         ///< Lifetime (0 means unlimited)
} entity_t;


// ---------------------------------------------------------
// Function Declarations
// ---------------------------------------------------------

/**
 * @brief Initialize an entity with default values.
 *
 * - id = -1 (unassigned)
 * - coord = {0, 0}
 * - owner = NULL
 * - age = 0.0f
 * - lifetime = 0.0f (infinite)
 * - width_range = 0
 * - height_range = 0
 * - influence_ratio = 1.0f
 */
BYUL_API void entity_init(entity_t* e);

/**
 * @brief Initialize an entity with specified values.
 *
 * @param[out] e          Entity to initialize
 * @param[in]  coord      Initial coordinate (NULL means {0,0})
 * @param[in]  id         Unique ID
 * @param[in]  owner      Pointer to owner (can be NULL)
 * @param[in]  age        Age since creation (negative values are set to 0)
 * @param[in]  lifetime   Lifetime (0 means unlimited)
 * @param[in]  width      Width range (grid units, minimum 0)
 * @param[in]  height     Height range (grid units, minimum 0)
 * @param[in]  influence  Influence ratio (>= 0.0f)
 */
void entity_init_full(
    entity_t* e,
    const coord_t* coord,
    int32_t id,
    void* owner,
    float age,
    float lifetime,
    int width,
    int height,
    float influence
);

/**
 * @brief Copy an entity_t.
 */
BYUL_API void entity_assign(entity_t* dst, const entity_t* src);

/**
 * @brief Check if the entity's lifetime has expired.
 *
 * @return true if expired (lifetime <= age).
 */
BYUL_API bool entity_is_expired(const entity_t* e);

/**
 * @brief Increment age by dt and check expiration.
 *
 * @return true if the lifetime has expired.
 */
BYUL_API bool entity_tick(entity_t* e, float dt);

/**
 * @brief Calculate the effective size of the entity.
 *
 * - Combines width_range and height_range using Euclidean distance.
 * - Applies influence_ratio as a scaling factor.
 * - Ensures the minimum size is 0.
 *
 * Formula:
 *   sqrt(1 + width_range^2 + height_range^2) * influence_ratio
 *
 * @param[in] e  Entity to calculate size for.
 * @return The size with influence applied (float).
 */
BYUL_API float entity_size(const entity_t* e);

#ifdef __cplusplus
}
#endif

#endif // ENTITY_H
