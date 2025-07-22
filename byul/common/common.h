#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "byul_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/// @brief float 비교를 위한 epsilon
// 상대 오차이다 절대 오차가 아니다 그래서 1e-6f가 아니라 1e-5f인 것이다 오해하지 마라
// 1.000001, 1.000002 이게 같은거다.
// CHECK(float_equal(1.00001f, 1.000019f));
// CHECK(float_equal(1.00001f, 1.000001f));
// CHECK_FALSE(float_equal(1.00001f, 1.000020f));
// CHECK_FALSE(float_equal(1.00001f, 1.000000f));
#define FLOAT_EPSILON 1e-5f

/**
 * @def FLOAT_EPSILON_TINY
 * @brief 매우 작은 값 판정에 사용되는 절대 오차 하한
 */
#define FLOAT_EPSILON_TINY      1e-8f

#define SQRT2_INV 0.70710678118f

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923  // PI/2
#endif

/**
 * @def M_TWO_PI
 * @brief 2π
 */
#define M_TWO_PI    (2.0f * M_PI)

/**
 * @def M_HALF_PI
 * @brief π / 2
 */
#define M_HALF_PI   (0.5f * M_PI)

/** @brief deg → rad */
#define DEG2RAD(x) ((x) * 0.017453292519943295f)

/**
 * @brief 라디안(radian)을 도(degree)로 변환
 * RAD2DEG(rad)  ((rad) * (180.0f / M_PI))
 */
#define RAD2DEG(rad)  ((rad) * 57.29577951308232f)

BYUL_API int  float_compare(float a, float b, void* userdata);

BYUL_API int  int_compare(int a, int b, void* userdata);

// ---------------------------------------------------------
// 함수 선언
// ---------------------------------------------------------

/**
 * @brief 상대 오차 기반 부동소수점 비교
 * @param a 첫 번째 값
 * @param b 두 번째 값
 * @return 값이 충분히 가까우면 true
 */
BYUL_API bool float_equal(float a, float b);

/**
 * @brief 공차(tolerance) 기반 부동소수점 비교
 *
 * @param a   첫 번째 값
 * @param b   두 번째 값
 * @param tol 허용 오차 (예: 1e-5f)
 * @return 값 차이가 tol 이하이면 true
 */
BYUL_API bool float_equal_tol(float a, float b, float tol);

/**
 * @brief 기준값 a에 대해 b가 양수/음수 공차 범위 내에 있는지 비교
 *
 * @param a       기준 값
 * @param b       비교 대상 값
 * @param tol_pos 양수 방향 공차 (b >= a일 때 허용 오차)
 * @param tol_neg 음수 방향 공차 (b < a일 때 허용 오차)
 * @return 허용 범위 내이면 true, 아니면 false
 *
 * 예시:
 *   a = 1.0, tol_pos = 0.002, tol_neg = 0.001
 *   b = 1.002  → true  (1.002 - 1.0 = 0.002 ≤ tol_pos)
 *   b = 0.999  → true  (1.0 - 0.999 = 0.001 ≤ tol_neg)
 *   b = 0.998  → false (1.0 - 0.998 = 0.002 > tol_neg)
 *
 * @note tol_pos, tol_neg가 음수로 들어오면 fabsf()로 자동 보정.
 */
BYUL_API bool float_equal_tol_all(
    float a, float b, float tol_pos, float tol_neg);

/**
 * @brief 값이 0에 가까운지 확인
 * @param x 비교할 값
 * @return |x| < EPSILON이면 true
 */
BYUL_API bool float_zero(float x);

/**
 * @brief 안전한 나눗셈
 * @param a 분자
 * @param b 분모
 * @param fallback b가 0일 경우 반환할 값
 * @return a / b 또는 fallback
 */
BYUL_API float float_safe_div(float a, float b, float fallback);

/**
 * @brief 제곱 계산 (x²)
 */
BYUL_API float square(float x);

/**
 * @brief 값을 지정된 범위로 클램프 (제한)
 */
BYUL_API float clamp(float x, float min_val, float max_val);

/**
 * @brief 부호 반환
 * @return 양수면 1.0f, 음수면 -1.0f, 0이면 0.0f
 */
BYUL_API float sign(float x);

/**
 * @brief 도(degree)를 라디안(radian)으로 변환
 */
BYUL_API float deg2rad(float deg);

/**
 * @brief 라디안(radian)을 도(degree)로 변환
 */
BYUL_API float rad2deg(float rad);

/**
 * @brief 선형 보간 (Linear Interpolation)
 * @param a 시작 값
 * @param b 종료 값
 * @param t 0~1 범위의 보간 인자
 */
BYUL_API float lerp(float a, float b, float t);

/**
 * @brief 선형 보간 역함수 (Inverse Lerp)
 * @param a 시작 값
 * @param b 종료 값
 * @param value 대상 값
 * @return value가 [a,b] 사이에서 몇 퍼센트에 위치하는지 (0~1)
 */
BYUL_API float inv_lerp(float a, float b, float value);

/**
 * @brief 범위 재매핑 (Remap)
 * @param in_min 입력 최소
 * @param in_max 입력 최대
 * @param out_min 출력 최소
 * @param out_max 출력 최대
 * @param value 입력 값
 * @return 변환된 출력 값
 */
BYUL_API float remap(float in_min, float in_max, 
                     float out_min, float out_max, float value);

/**
 * @brief 주어진 값을 0.0과 1.0 사이로 제한합니다.
 *
 * 이 함수는 입력값이 1보다 크면 1로, 0보다 작으면 0으로 잘라냅니다.
 * 예를 들어 보간 계수 t값, 투명도, 정규화 값 등
 * [0.0, 1.0] 범위 내에 있어야 하는 수치를 안정적으로 제한할 때 사용됩니다.
 *
 * @param x 입력 값
 * @return 0.0 이상 1.0 이하로 제한된 값
 *
 * @note clamp(x, 0.0f, 1.0f)과 동일한 기능입니다.
 */
BYUL_API float clamp01(float x);


/**
 * @brief 부드러운 보간 함수 (smoothstep)
 * @param edge0 시작 경계
 * @param edge1 끝 경계
 * @param x 현재 값
 * @return 부드럽게 보간된 결과 (0~1)
 */
BYUL_API float smoothstep(float edge0, float edge1, float x);

#ifdef __cplusplus
}
#endif

#endif // COMMON_H
