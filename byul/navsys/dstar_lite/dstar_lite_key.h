#ifndef DSTAR_LITE_KEY_H
#define DSTAR_LITE_KEY_H

#include "byul_config.h"
#include "internal/coord.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct s_dstar_lite_key {
    float k1;
    float k2;
} dstar_lite_key_t;

// ------------------------ 생성/복사/해제 ------------------------

BYUL_API dstar_lite_key_t* dstar_lite_key_new(void);

BYUL_API dstar_lite_key_t* dstar_lite_key_new_full(float k1, float k2);

BYUL_API dstar_lite_key_t* dstar_lite_key_copy(const dstar_lite_key_t* key);

BYUL_API void dstar_lite_key_free(dstar_lite_key_t* key);

// ------------------------ 비교 함수 ------------------------

/// @brief 키가 동등한지 확인 (float 오차 허용)
/// @return true = 거의 같다
BYUL_API bool dstar_lite_key_equal(const dstar_lite_key_t* dsk0,
                                   const dstar_lite_key_t* dsk1);
                                   
/**
 * @brief D* Lite 키 비교 함수.
 *
 * 두 개의 키를 비교하여 정렬 우선순위를 결정합니다.
 * 우선적으로 k1 값을 비교하고, 동일한 경우 k2 값을 비교합니다.
 *
 * @note 두 인자는 모두 NULL이 아니어야 하며, NULL 확인은 호출자가 직접 수행해야 합니다.
 *
 * @param dsk0 비교 대상 키 1 (비교의 왼쪽 피연산자)
 * @param dsk1 비교 대상 키 2 (비교의 오른쪽 피연산자)
 * @return 음수: dsk0 < dsk1  
 *         0: 두 키가 동일  
 *         양수: dsk0 > dsk1
 */
BYUL_API int dstar_lite_key_compare(const dstar_lite_key_t* dsk0,
                                    const dstar_lite_key_t* dsk1);

/// @brief 키의 해시 값 계산 (hash navgrid용)
BYUL_API unsigned int dstar_lite_key_hash(const dstar_lite_key_t* key);

#ifdef __cplusplus
}
#endif

#endif // DSTAR_LITE_KEY_H
