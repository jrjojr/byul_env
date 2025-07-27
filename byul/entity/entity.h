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
 * @brief 모든 게임 객체의 최소 공통 속성 + 공간 영향 속성
 *
 * - coord: 엔티티의 중심 좌표 (격자 기준)
 * - width_range, height_range: 좌우/상하로 확장되는 영향 범위 (격자 단위)
 *   * 0이면 해당 축에 대한 추가 범위 없음 (단일 셀)
 *   * 1이면 중심 좌표 포함 + 양옆 1칸씩 총 3칸 범위
 * - influence_ratio: 
 *   * 0.0f ~ 값으로, 엔티티가 미치는 영향 비율을 나타냄
 *   * 0이면 완전히 영향 없음, 1이면 최대 영향
 * - age/lifetime: 존재 시간 관리
 */
typedef struct s_entity {
    int32_t id;           ///< 고유 ID (-1은 미할당)
    coord_t coord;        ///< 엔티티 중심 좌표 (격자 기반)
    void* owner;          ///< 소유자 (다른 entity_t* 또는 시스템 객체)

    // --- 공간/영향 관련 속성 ---
    int32_t width_range;   ///< X축 영향 범위 (격자 단위, 기본값 0)
    int32_t height_range;  ///< Y축 영향 범위 (격자 단위, 기본값 0)
    float   influence_ratio; ///< 영향 비율 (0 = 없음, 1 = 기본, >1 = 더 큰 범위)

    // --- 시간 관련 속성 ---
    float age;             ///< 생성 후 경과 시간 (초)
    float lifetime;        ///< 유효 시간 (0이면 무제한)
} entity_t;


// ---------------------------------------------------------
// 함수 선언
// ---------------------------------------------------------

/**
 * @brief 엔티티를 기본값으로 초기화합니다.
 *
 * - id = -1 (미할당)
 * - coord = {0, 0}
 * - owner = NULL
 * - age = 0.0f
 * - lifetime = 0.0f (무한수명)
 * - width_range = 0
 * - height_range = 0
 * - influence_ratio = 1.0f
 */
BYUL_API void entity_init(entity_t* e);

/**
 * @brief 엔티티를 지정된 값으로 초기화합니다.
 *
 * @param[out] e          초기화할 엔티티
 * @param[in]  coord      초기 좌표 (NULL이면 {0,0})
 * @param[in]  id         고유 ID
 * @param[in]  owner      소유자 포인터 (NULL 가능)
 * @param[in]  age        생성 후 경과 시간 (음수면 0으로 설정)
 * @param[in]  lifetime   유효 시간 (0이면 무한)
 * @param[in]  width      가로 범위 (격자 단위, 최소 0)
 * @param[in]  height     세로 범위 (격자 단위, 최소 0)
 * @param[in]  influence  영향 비율 (0.0f ~ )
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

/**
 * @brief 엔티티의 실제 크기를 계산합니다.
 *
 * - 유클리드 거리 기반으로 width_range, height_range를 결합.
 * - influence_ratio로 가중치를 부여.
 * - 최소 크기 제한(0) 처리를 포함.
 * entity의 크기를 얻는다
 * sqrt(1 + width_range^2 + height_range^2) * influence_ratio
 *
 * @param[in] e  크기를 계산할 엔티티
 * @return influence_ratio가 적용된 크기 (float)
 */
BYUL_API float entity_size(const entity_t* e);

#ifdef __cplusplus
}
#endif

#endif // ENTITY_H
