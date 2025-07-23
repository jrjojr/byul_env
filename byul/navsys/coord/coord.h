#ifndef COORD_H
#define COORD_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>   // C 스타일

#include "byul_common.h"

#ifdef __cplusplus
extern "C" {
#endif

// ------------------------ 좌표 한계값 정의 ------------------------
#ifndef COORD_MAX
#define COORD_MAX   (200000000)   ///< coord의 최대 절대값
#endif

#ifndef COORD_MIN
#define COORD_MIN   (-200000000)  ///< coord의 최소 절대값
#endif

// ------------------------ 구조체 정의 ------------------------
/**
 * @struct s_coord
 * @brief 2D 셀 좌표 (정수 기반)
 */
typedef struct s_coord {
    int x; ///< X 좌표
    int y; ///< Y 좌표
} coord_t;

// ------------------------ 생성/해제 ------------------------
BYUL_API coord_t* coord_create_full(int x, int y);
BYUL_API coord_t* coord_create(void);
BYUL_API void     coord_destroy(coord_t* c);
BYUL_API coord_t* coord_copy(const coord_t* c);

// ------------------------ 초기화 및 복사 ------------------------
/**
 * @brief (0,0)으로 초기화
 */
BYUL_API void coord_init(coord_t* c);

/**
 * @brief 지정 값으로 초기화
 */
BYUL_API void coord_init_full(coord_t* c, int x, int y);

/**
 * @brief 좌표 복사
 */
BYUL_API void coord_assign(coord_t* dst, const coord_t* src);

// ------------------------ 가감승제 ------------------------
/**
 * @brief a + b 결과를 dst에 저장
 */
BYUL_API void coord_add(coord_t* dst, const coord_t* a, const coord_t* b);

/**
 * @brief a - b 결과를 dst에 저장
 */
BYUL_API void coord_sub(coord_t* dst, const coord_t* a, const coord_t* b);

/**
 * @brief a * scalar 결과를 dst에 저장
 */
BYUL_API void coord_mul(coord_t* dst, const coord_t* a, int scalar);

/**
 * @brief a / scalar 결과를 dst에 저장 (정수 나눗셈)
 */
BYUL_API void coord_div(coord_t* dst, const coord_t* a, int scalar);

// ------------------------ 자기 연산 ------------------------
/**
 * @brief c += other
 */
BYUL_API void coord_iadd(coord_t* c, const coord_t* other);

/**
 * @brief c -= other
 */
BYUL_API void coord_isub(coord_t* c, const coord_t* other);

/**
 * @brief c *= scalar
 */
BYUL_API void coord_imul(coord_t* c, int scalar);

/**
 * @brief c /= scalar (정수 나눗셈)
 */
BYUL_API void coord_idiv(coord_t* c, int scalar);

// ------------------------ 비교/해시 ------------------------
BYUL_API unsigned coord_hash(const coord_t* c);
BYUL_API bool     coord_equal(const coord_t* c1, const coord_t* c2);
BYUL_API int      coord_compare(const coord_t* c1, const coord_t* c2);

/// 유클리드 거리 계산
BYUL_API float    coord_distance(const coord_t* a, const coord_t* b);

BYUL_API int      coord_manhattan_distance(const coord_t* a, const coord_t* b);

/**
 * @brief 두 좌표 벡터 간의 각도를 라디안(0 ~ 2π)으로 반환
 *
 * @param a 시작 좌표 (벡터의 시작점)
 * @param b 목표 좌표 (벡터의 끝점)
 * @return double (0.0 ~ 2π)
 *
 * 기준:
 *   - (0,0) → (1,0) 방향이 0 rad
 *   - (0,0) → (0,1) 방향은 π/2 rad
 *   - (0,0) → (-1,0) 방향은 π rad
 *   - (0,0) → (0,-1) 방향은 3π/2 rad
 *
 * @note atan2(y, x)를 직접 반환하며, 음수 각도는 2π를 더해 양수 범위로 변환.
 */
BYUL_API double coord_angle(const coord_t* a, const coord_t* b);

/**
 * @brief 두 좌표 벡터 간의 각도를 0~360도로 반환
 *
 * @param a 시작 좌표 (벡터의 시작점)
 * @param b 목표 좌표 (벡터의 끝점)
 * @return double (0.0 ~ 360.0)
 *
 * 기준:
 *   - (0,0) → (1,0) 방향이 0도
 *   - (0,0) → (0,1) 방향은 90도
 *   - (0,0) → (-1,0) 방향은 180도
 *   - (0,0) → (0,-1) 방향은 270도
 *
 * @note atan2(y, x)를 사용해 라디안 각도를 계산한 뒤
 *       (180.0 / PI)를 곱해 도 단위로 변환.
 *       음수 각도는 360도를 더해 양수 범위로 변환.
 */
BYUL_API double coord_degree(const coord_t* a, const coord_t* b);

/**
 * @brief 시작 좌표(start)에서 목표 좌표(goal)로 한 칸 이동했을 때
 *        가장 가까운 이웃 좌표를 out에 반환한다.
 *
 * @param[out] out   계산된 다음 좌표
 * @param[in]  start 시작 좌표
 * @param[in]  goal  목표 좌표
 *
 * @note start와 goal이 같으면 out = start가 된다.
 */
BYUL_API void coord_next_to_goal(
    coord_t* out,
    const coord_t* start,
    const coord_t* goal);

// ------------------------ 좌표 접근자/설정자 ------------------------
BYUL_API int      coord_get_x(const coord_t* c);
BYUL_API void     coord_set_x(coord_t* c, int x);

BYUL_API int      coord_get_y(const coord_t* c);
BYUL_API void     coord_set_y(coord_t* c, int y);

BYUL_API void     coord_set(coord_t* c, int x, int y);
BYUL_API void     coord_fetch(const coord_t* c, int* out_x, int* out_y);

// 호환성 확보 목적 나중에 제거한다.
BYUL_API const coord_t* make_tmp_coord(int x, int y);

// 호환성 확보 목적 나중에 제거한다.
// 시작 좌표에서 목표좌표로 가기위해 가장 가까운 이웃을 반환한다.
BYUL_API coord_t* coord_clone_next_to_goal(
    const coord_t* start, const coord_t* goal);

/**
 * @brief coord 값을 문자열로 변환
 * @param c 변환할 coord
 * @param buffer 출력 버퍼 (최소 32바이트 권장)
 * @param buffer_size 버퍼 크기
 * @return buffer (편의상 반환)
 */
BYUL_API char* coord_to_string(
    const coord_t* c, char* buffer, size_t buffer_size);

/**
 * @brief coord 값을 콘솔에 출력
 * @param c 출력할 coord
 */
BYUL_API void coord_print(const coord_t* c);

#ifdef __cplusplus
}
#endif

#endif // COORD_H
