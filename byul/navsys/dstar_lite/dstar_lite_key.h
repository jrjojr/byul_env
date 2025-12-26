#ifndef DSTAR_LITE_KEY_H
#define DSTAR_LITE_KEY_H

#include "byul_config.h"
#include "coord.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct s_dstar_lite_key {
    float k1;
    float k2;
} dstar_lite_key_t;

// ------------------------ Create / Copy / Destroy ------------------------

BYUL_API dstar_lite_key_t* dstar_lite_key_create(void);

BYUL_API dstar_lite_key_t* dstar_lite_key_create_full(float k1, float k2);

BYUL_API dstar_lite_key_t* dstar_lite_key_copy(const dstar_lite_key_t* key);

BYUL_API void dstar_lite_key_destroy(dstar_lite_key_t* key);

// ------------------------ Comparison Functions ------------------------

/// @brief Check if keys are approximately equal (with float tolerance)
/// @return true = almost equal
BYUL_API bool dstar_lite_key_equal(const dstar_lite_key_t* dsk0,
                                   const dstar_lite_key_t* dsk1);
                                   
/**
 * @brief D* Lite key comparison function.
 *
 * Compares two keys to determine their sorting priority.
 * k1 is compared first, and if they are equal, k2 is compared.
 *
 * @note Both arguments must not be NULL, and NULL checks
 *       must be handled by the caller.
 *
 * @param dsk0 Key 1 for comparison (left operand)
 * @param dsk1 Key 2 for comparison (right operand)
 * @return Negative: dsk0 < dsk1  
 *         0: Both keys are equal  
 *         Positive: dsk0 > dsk1
 */
BYUL_API int dstar_lite_key_compare(const dstar_lite_key_t* dsk0,
                                    const dstar_lite_key_t* dsk1);

/// @brief Calculate the hash value of a key (for navgrid hash)
BYUL_API unsigned int dstar_lite_key_hash(const dstar_lite_key_t* key);

#ifdef __cplusplus
}
#endif

#endif // DSTAR_LITE_KEY_H
