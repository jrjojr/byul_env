#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "byul_common.h"
#include "internal/obstacle_common.h"

#ifdef __cplusplus
extern "C" {
#endif

BYUL_API obstacle_t* obstacle_make_rect_all_blocked(
    int x0, int y0, int width, int height);

// ratio는 0.0 ~ 1.0의 범위 1.0이면 풀 블락
BYUL_API obstacle_t* obstacle_make_rect_random_blocked(
    int x0, int y0, int width, int height, float ratio);

// range 0 : 해당 좌표만 , 1이상 주변까지 포함해서 장애물 생성한다.
// start, goal의 가상의 라인을 따라서...
BYUL_API obstacle_t* obstacle_make_beam(
    const coord_t* start, const coord_t* goal, int range);

/**
 * @brief 도넛(토러스) 형태의 폐쇄 장애물을 생성합니다.
 *
 * 이 함수는 사각형 영역 내에서 안쪽은 비워두고, 
 * 바깥 경계만 일정한 두께로 막는 고리형(토러스) 장애물을 생성합니다.
 * 내부와 외부는 완전히 분리되어 있어 경로가 없습니다.
 * 
 * @note 최소 생성 크기: 
 *       - 너비(width) ≥ thickness × 2 + 1
 *       - 높이(height) ≥ thickness × 2 + 1
 *       이보다 작을 경우 내부 공간이 없어 도넛 구조를 만들 수 없습니다.
 *
 * @note `thickness` 값이 너무 커서 내부 공간이 생성되지 못할 경우,
 *       함수는 NULL을 반환합니다.
 *
 * @param start      사각형 영역의 한 꼭짓점 좌표입니다.
 * @param goal       반대편 꼭짓점 좌표입니다.
 * @param thickness  경계 벽의 두께 (1 이상). 
 *                   이 값이 클수록 외곽 고리의 너비가 두꺼워집니다.
 *
 * @return torus 형태의 장애물 포인터 (성공 시), 또는 NULL (실패 시).
 */
BYUL_API obstacle_t* obstacle_make_torus(
    const coord_t* start, const coord_t* goal, int thickness);

typedef enum e_enclosure_open_dir{
    ENCLOSURE_OPEN_UNKNOWN,
    ENCLOSURE_OPEN_RIGHT,
    ENCLOSURE_OPEN_UP,
    ENCLOSURE_OPEN_LEFT,
    ENCLOSURE_OPEN_DOWN,
}enclosure_open_dir_t;

/**
 * @brief 한쪽이 열린 사각형 외곽 장애물을 생성합니다.
 *
 * 이 함수는 사각형 경계의 네 면 중 하나를 개방한 
 * "냄비", "U자형" 형태의 장애물을 만듭니다.
 * 플레이어나 NPC가 내부로 출입할 수 있는 구조를 만들 때 유용합니다.
 *
 * @note 최소 생성 크기:
 *       - 너비(width) ≥ thickness × 2 + 1
 *       - 높이(height) ≥ thickness × 2 + 1
 *       벽을 설치할 수 있는 여유 공간이 없으면 생성이 실패할 수 있습니다.
 * 
 * `open` 인자를 통해 열린 방향을 지정할 수 있으며, 
 * ENCLOSURE_OPEN_UNKNOWN 을 전달하면 네 면 모두를 막습니다.
 *
 * @param start      사각형 영역의 한 꼭짓점 좌표입니다.
 * @param goal       반대편 꼭짓점 좌표입니다.
 * @param thickness  벽 두께 (1 이상). 
 *                   값이 커질수록 각 면의 벽이 두꺼워집니다.
 * @param open       열려 있을 방향 (위, 아래, 왼쪽, 오른쪽 중 선택).
 *
 * @return enclosure 형태의 장애물 포인터 (성공 시), 또는 NULL (실패 시).
 */
BYUL_API obstacle_t* obstacle_make_enclosure(
    const coord_t* start, const coord_t* goal, int thickness, 
    enclosure_open_dir_t open);

/**
 * @brief 중심 좌표를 기준으로 십자(+)형 장애물을 생성합니다.
 *
 * 이 함수는 `center`를 기준으로 상하좌우 방향으로 `length`만큼 뻗은
 * 십자형 구조를 생성하며, 각 팔의 두께는 반경`range`로 설정됩니다.
 *
 * @param center    십자의 중심 좌표
 * @param length    각 팔의 길이 (0 이면 점만 찍는다)
 * @param range 각 팔의 너비 (0이면 해당 좌표만 ,1 이상 주변 좌표)
 *
 * @return 생성된 obstacle_t 포인터 또는 실패 시 NULL
 */
BYUL_API obstacle_t* obstacle_make_cross(
    const coord_t* center, int length, int range);


typedef enum e_spiral_dir {
    SPIRAL_CLOCKWISE,        ///< 시계 방향 (기본)
    SPIRAL_COUNTER_CLOCKWISE ///< 반시계 방향
} spiral_dir_t;

/**
 * @brief 중심을 기준으로 나선형(spiral) 장애물을 생성합니다.
 *
 * 이 함수는 격자 기반의 정사각형 나선 구조를 생성하며, 
 * 시계 또는 반시계 방향으로 회전하면서 각 경로를 장애물로 블로킹합니다.
 * 회전 수(`turns`)에 따라 전체 나선의 길이가 결정되며, 
 * `gap`을 설정하면 회전 사이에 공백을 두어 공간성을 확보할 수 있습니다.
 *
 * `range`를 사용하면 경로의 중심 외에도 주변까지 넓게 블로킹할 수 있습니다.
 * 방향은 `direction`을 통해 시계 또는 반시계로 지정할 수 있습니다.
 *
 * @param center   나선의 중심 좌표입니다.
 * @param radius   나선이 퍼질 수 있는 최대 반지름 (격자 거리 기준)
 * @param turns    전체 회전 수 (1회전 = 4방향 회전). 최소 1 이상
 * @param range    각 경로의 반경 (0이면 경로 중심점만 block, 1 이상이면 면적 포함)
 * @param gap      회전 단계 간 간격 (0이면 연속 회전, 1 이상이면 일부 회전 건너뜀)
 * @param direction 회전 방향 (SPIRAL_CLOCKWISE 또는 SPIRAL_COUNTER_CLOCKWISE)
 *
 * @return 생성된 obstacle_t 포인터 (성공 시), 또는 실패 시 NULL
 *
 * @note gap이 클수록 회전 사이에 여백이 생기며,
 *       range가 클수록 장애물의 두께가 넓어집니다.
 */
BYUL_API obstacle_t* obstacle_make_spiral(
    const coord_t* center,
    int radius,
    int turns,
    int range,
    int gap,
    spiral_dir_t direction
);

/**
 * @brief 주어진 세 꼭짓점으로 구성된 삼각형 영역을 블로킹하는 장애물을 생성합니다.
 *
 * 이 함수는 `a`, `b`, `c` 세 좌표를 꼭짓점으로 하는 삼각형 내부를
 * 격자 기반으로 블로킹하여 triangle-shaped obstacle을 생성합니다.
 *
 * @param a  첫 번째 꼭짓점
 * @param b  두 번째 꼭짓점
 * @param c  세 번째 꼭짓점
 *
 * @return 생성된 obstacle_t 포인터 또는 실패 시 NULL
 */
BYUL_API obstacle_t* obstacle_make_triangle(
    const coord_t* a,
    const coord_t* b,
    const coord_t* c);

/**
 * @brief 삼각형 외곽만 블로킹한 torus 형태의 장애물을 생성합니다.
 *
 * 주어진 세 꼭짓점 `a`, `b`, `c`로 정의된 삼각형의 외곽을 따라
 * 선형 경계를 구성하고, 내부는 block하지 않습니다.
 * `thickness`가 1 이상일 경우, 각 선분 경계를 기준으로 주변까지 두껍게 막습니다.
 *
 * @param a         삼각형 꼭짓점 A
 * @param b         삼각형 꼭짓점 B
 * @param c         삼각형 꼭짓점 C
 * @param thickness 외곽선 블록 두께 (0이면 선만, 1 이상이면 주변도 포함)
 *
 * @return 생성된 obstacle_t 포인터 또는 실패 시 NULL
 */
BYUL_API obstacle_t* obstacle_make_triangle_torus(
    const coord_t* a,
    const coord_t* b,
    const coord_t* c,
    int thickness);

/**
 * @brief 주어진 다각형 꼭짓점 리스트를 기반으로 내부를 블로킹하는 장애물을 생성합니다.
 *
 * 리스트에 포함된 좌표들을 순서대로 연결하여 폐곡선을 형성하고,
 * 그 내부 영역을 모두 블로킹합니다. 최소 3개 이상의 좌표가 필요하며,
 * 리스트는 자동으로 폐곡선으로 간주됩니다 (마지막 → 첫점 연결).
 *
 * @param list  다각형 꼭짓점 리스트 (coord_list_t*)
 * @return 생성된 obstacle_t 포인터 또는 실패 시 NULL
 */
BYUL_API obstacle_t* obstacle_make_polygon(coord_list_t* list);

/**
 * @brief 다각형 꼭짓점 리스트를 기반으로 외곽선만 블로킹하는 torus 장애물을 생성합니다.
 *
 * 리스트에 포함된 좌표들을 순서대로 연결하여 폐곡선을 구성한 후,
 * 각 선분을 따라 외곽선만 블로킹하며 내부는 비워둡니다.
 * `thickness`가 1 이상일 경우 외곽선의 두께를 확장할 수 있습니다.
 *
 * @param list       다각형 꼭짓점 리스트 (최소 3점)
 * @param thickness  외곽선의 블로킹 두께 (0이면 선만, 1 이상은 영역)
 * @return 생성된 obstacle_t 포인터 또는 실패 시 NULL
 */
BYUL_API obstacle_t* obstacle_make_polygon_torus(
    coord_list_t* list, int thickness);


#ifdef __cplusplus
}
#endif

#endif // OBSTACLE_H
