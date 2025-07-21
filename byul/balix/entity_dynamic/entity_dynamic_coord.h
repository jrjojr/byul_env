#ifndef ENTITY_DYNAMIC_COORD_H
#define ENTITY_DYNAMIC_COORD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "internal/entity_dynamic.h"

/**
 * @brief 현재 coord와 xform의 상대 좌표를 더해 절대 좌표(coord_t)를 계산합니다.
 *
 * 절대좌표 = base.coord + round(xform.translation)
 *
 * @param[in]  ed   기준 엔티티 (coord와 xform 정보 포함)
 * @param[out] out  계산된 절대 좌표
 *
 * @note wrap-around는 coord 연산에서 자동 처리됩니다.
 */
BYUL_API void entity_dynamic_get_world_coord(
    const entity_dynamic_t* ed, coord_t* out);

/**
 * @brief xform 이동 적용 후 coord를 갱신
 *
 * xform.translation이 1 셀 이상 누적되면 coord에 반영하고
 * xform.translation은 남은 소수점 이동량만 유지.
 *
 * @note coord가 COORD_MAX 또는 COORD_MIN을 초과하면 wrap-around 처리.
 */
BYUL_API void entity_dynamic_commit_coord(entity_dynamic_t* ed);

/**
 * @brief 두 엔티티의 coord 기반 거리 계산 (XFORM_MAX,MIN 초과 시 실패)
 *
 * @return 거리 (float), 범위 초과 시 INFINITY 반환
 * @note 좌표 차이가 COORD_MAX/COORD_MIN을 넘어가면 wrap 고려 후 계산 취소.
 */
BYUL_API float entity_dynamic_coord_distance(
    const entity_dynamic_t* a, const entity_dynamic_t* b);

/**
 * @brief 두 엔티티의 coord 거리가 유효 범위(XFORM_MAX,MIN) 내인지 체크
 *
 * @return true = 범위 내, false = 범위 초과
 */
BYUL_API bool entity_dynamic_coord_in_range(
    const entity_dynamic_t* a, const entity_dynamic_t* b);

#ifdef __cplusplus
}
#endif

#endif // ENTITY_DYNAMIC_COORD_H
