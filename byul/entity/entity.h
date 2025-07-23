#ifndef ENTITY_H
#define ENTITY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "byul_common.h"
#include "internal/coord.h"

// ---------------------------------------------------------
// 기본 엔티티 구조체
// ---------------------------------------------------------
/**
 * @struct s_entity
 * @brief 모든 게임 객체의 최소 공통 속성
 *
 * - 격자 좌표(coord_t)
 * - ID
 * - 소유자(owner)
 * - 생성 후 경과 시간(age)
 * - 유효 시간(lifetime)
 */
typedef struct s_entity {
    int32_t id;        ///< 고유 ID (-1은 미할당)
    coord_t coord;     ///< 격자 기반 좌표 (길찾기 및 로직용)
    void* owner;       ///< 소유자 (다른 entity_t* 또는 시스템 객체)
    float age;         ///< 생성 후 경과 시간 (초)
    float lifetime;    ///< 유효 시간 (0이면 무제한)
} entity_t;

// ---------------------------------------------------------
// 함수 선언
// ---------------------------------------------------------

/**
 * @brief entity_t를 기본값으로 초기화합니다.
 *
 * 기본값:
 * - id = -1
 * - coord = (0,0)
 * - owner = NULL
 * - age = 0
 * - lifetime = 0 (무제한)
 */
BYUL_API void entity_init(entity_t* e);

/**
 * @brief entity_t를 사용자 지정 값으로 초기화합니다.
 *
 * @param[out] e        초기화할 entity_t 포인터
 * @param[in]  coord    초기 좌표 (NULL이면 (0,0))
 * @param[in]  id       고유 ID
 * @param[in]  owner    소유자 포인터
 * @param[in]  age      초기 경과 시간 (0 이상)
 * @param[in]  lifetime 유효 시간 (0이면 무제한)
 */
BYUL_API void entity_init_full(
    entity_t* e,
    const coord_t* coord,
    int32_t id,
    void* owner,
    float age,
    float lifetime
);

/**
 * @brief entity_t 복사
 */
BYUL_API void entity_assign(entity_t* dst, const entity_t* src);

/**
 * @brief entity의 수명 만료 여부를 확인합니다.
 *
 * @return true이면 만료(lifetime <= age)
 */
BYUL_API bool entity_is_expired(const entity_t* e);

/**
 * @brief tick마다 age를 증가시키고 만료 여부를 반환
 *
 * @return true이면 lifetime이 만료됨
 */
BYUL_API bool entity_tick(entity_t* e, float dt);

#ifdef __cplusplus
}
#endif

#endif // ENTITY_H
