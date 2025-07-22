#ifndef ROUTE_FORGE_H
#define ROUTE_FORGE_H

#include "byul_config.h"
#include "internal/navgrid.h"     // navgrid_t 정의 필요
#include "internal/coord.h"   // coord_t 정의 필요

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief start → goal 직선 방향으로, 반경 range 안의 장애물을 직선으로 제거합니다.
 *        넓은 영역을 직진 관통하며 개척합니다.
 *
 * @param navgrid 대상 맵
 * @param start 시작 좌표
 * @param goal 목표 좌표
 * @param range 선 중심의 반경 (0=좌표만, 1이상 주변 반경)
 * @return 제거된 장애물 수
 */
BYUL_API int route_carve_beam(navgrid_t* navgrid, 
    const coord_t* start, const coord_t* goal, int range);

/**
 * @brief 지정된 중심 좌표를 기준으로 반경 range 내 block 셀을 폭격하여 제거합니다.
 *        장애물이 흩어진 곳을 강제로 열거나, 공간 확보용으로 사용됩니다.
 *
 * @param navgrid 대상 맵
 * @param center 중심 좌표
 * @param range 폭파 반경 (0 : 해당 좌표만,  1이상 주변 반경)
 * @return 제거된 장애물 수
 */
BYUL_API int route_carve_bomb(navgrid_t* navgrid, const coord_t* center, int range);

#ifdef __cplusplus
}
#endif

#endif // ROUTE_FORGE_H
