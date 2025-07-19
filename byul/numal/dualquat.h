#ifndef DUALQUAT_H
#define DUALQUAT_H

#include "internal/quat.h"
#include "internal/vec3.h"
#include "byul_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 듀얼 회전 구조체 전방 선언 (회전 + 위치 통합 표현)
 */
typedef struct dualquat_t {
    quat_t real;
    quat_t dual;
} dualquat_t;

// -----------------------------
// 생성 / 해제 / 복사
// -----------------------------

/** @brief 단위 듀얼 회전 (회전=단위, 위치=0) 생성 */
BYUL_API void dualquat_init(dualquat_t* out);

/**
 * @brief 회전(quaternion)과 위치(vec3)로부터 듀얼 쿼터니언을 생성합니다.
 *
 * @details
 * 다음 수식 기반으로 듀얼 쿼터니언을 구성합니다:
 * - real = rot
 * - dual = (translation_quat * rot) * 0.5
 *
 * 이 함수는 위치 벡터가 없는 경우(즉, `vec == NULL`)에는
 * 회전만 포함된 듀얼 쿼터니언으로 구성됩니다.
 * 반대로 `rot == NULL`이면 단위 회전으로 간주합니다.
 *
 * @param[out] out 결과 듀얼 쿼터니언 포인터 (NULL이면 아무 작업도 수행하지 않음)
 * @param[in]  rot 회전 쿼터니언 (NULL이면 단위 회전으로 처리됨)
 * @param[in]  vec 위치 벡터 (NULL이면 위치 없음으로 처리됨)
 */
BYUL_API void dualquat_init_quat_vec(
    dualquat_t* out, const quat_t* rot, const vec3_t* vec);

/** @brief 복사 */
BYUL_API void dualquat_copy(dualquat_t* out, const dualquat_t* src);

// -----------------------------
// 비교 / 해시
// -----------------------------

/** @brief 두 듀얼 회전이 같은지 비교 */
BYUL_API int dualquat_equal(const dualquat_t* a, const dualquat_t* b);

/** @brief 해시값 계산 */
BYUL_API uint32_t dualquat_hash(const dualquat_t* dq);

/**
 * @brief 듀얼 쿼터니언으로부터 회전(quat)과 위치(vec3)를 추출합니다.
 *
 * @details
 * - 회전 성분은 `real`에서 직접 추출합니다.
 * - 위치는 dual = (translation_quat * real_conj), 따라서:
 *   - t = 2 × (dual * real⁻¹).xyz
 *
 * `rot` 또는 `vec` 중 NULL인 경우 해당 출력을 생략합니다.
 * 예: `rot == NULL`이면 회전은 추출하지 않고, `vec == NULL`이면 위치만 추출됩니다.
 *
 * @param[in]  dq  듀얼 쿼터니언 포인터 (NULL이면 아무 작업도 하지 않음)
 * @param[out] rot 추출된 회전 쿼터니언 포인터 (NULL이면 무시)
 * @param[out] vec 추출된 위치 벡터 포인터 (NULL이면 무시)
 */
BYUL_API void dualquat_to_quat_vec(
    const dualquat_t* dq,
    quat_t* rot,
    vec3_t* vec
);

// -----------------------------
// 연산
// -----------------------------

// io 본인을 정규화한다.
BYUL_API void dualquat_normalize(dualquat_t* io);

/** @brief 정규화 */
BYUL_API void dualquat_unit(dualquat_t* out, const dualquat_t* dq);

/** @brief 켤레 */
BYUL_API void dualquat_conjugate(dualquat_t* out, const dualquat_t* dq);

// -----------------------------
// 변환 / 적용
// -----------------------------

/**
 * @brief dual quaternion을 사용하여 포인트를 변환합니다 (자기 자신 in-place 덮어쓰기).
 *
 * @param dq        변환에 사용할 dual quaternion
 * @param io_point  변환 전 입력이자 변환 후 결과로 덮어쓸 포인트
 *
 * @note 내부적으로 복사 방지를 위해 임시 변수 없이 직접 계산됩니다.
 *       결과적으로 io_point는 변환된 값만 유지되며, 원래 값은 사라집니다.
 *
 * @code
 * vec3_t p = {1, 0, 0};
 * dualquat_apply_to_point_inplace(&dq, &p);  // p가 회전 + 이동된 좌표로 덮어짐
 * @endcode
 */
BYUL_API void dualquat_apply_to_point_inplace(
    const dualquat_t* dq, vec3_t* io_point);

BYUL_API void dualquat_apply_to_point(
    const dualquat_t* dq, const vec3_t* in, vec3_t* out);

/** @brief 4x4 행렬로 변환 (column-major) */
BYUL_API void dualquat_to_mat4(const dualquat_t* dq, float* out_mat4);

/** @brief 4x4 행렬로부터 생성 */
BYUL_API void dualquat_init_mat4(dualquat_t* out, const float* mat4x4);

/** @brief 3x3 회전 행렬 변환 (이동 정보는 제외) */
BYUL_API void dualquat_to_mat3(const dualquat_t* dq, float* out_mat3);

/** @brief 3x3 행렬로부터 생성 (이동은 0으로 설정) */
BYUL_API void dualquat_init_mat3(dualquat_t* out, const float* mat3);

// -----------------------------
// 보간
// -----------------------------

/** @brief 선형 보간 (LERP) */
BYUL_API void dualquat_lerp(dualquat_t* out, 
    const dualquat_t* a, const dualquat_t* b, float t);

/** @brief 구면 선형 보간 (SLERP) */
BYUL_API void dualquat_slerp(dualquat_t* out, 
    const dualquat_t* a, const dualquat_t* b, float t);

/** @brief 듀얼 쿼터니언 덧셈: out = a + b */
BYUL_API void dualquat_add(dualquat_t* out, 
    const dualquat_t* a, const dualquat_t* b);

/** @brief 듀얼 쿼터니언 뺄셈: out = a - b */
BYUL_API void dualquat_sub(dualquat_t* out, 
    const dualquat_t* a, const dualquat_t* b);

/** @brief 곱셈 (합성 변환) */
BYUL_API void dualquat_mul(dualquat_t* out, 
    const dualquat_t* a, const dualquat_t* b);

/** @brief 듀얼 쿼터니언 스케일: out = a * scalar */
BYUL_API void dualquat_scale(dualquat_t* out, 
    const dualquat_t* a, float scalar);

/** @brief 듀얼 쿼터니언의 내적 (real + dual dot) */
BYUL_API float dualquat_dot(const dualquat_t* a, const dualquat_t* b);

/** @brief 듀얼 쿼터니언의 크기(norm): sqrt(real.norm^2 + dual.norm^2) */
BYUL_API float dualquat_length(const dualquat_t* a);

/**
 * @brief 듀얼 쿼터니언의 역을 계산합니다.
 * 
 * conj(real) + ε·(-conj(dual)) 방식으로 역변환을 구성합니다.
 * 이 연산은 rigid transform의 역행렬에 해당하며,
 * 회전과 이동 모두를 반대로 적용하는 데 사용됩니다.
 * 
 * @param[out] out 결과를 저장할 듀얼 쿼터니언
 * @param[in] dq 원본 듀얼 쿼터니언
 */
BYUL_API void dualquat_inverse(dualquat_t* out, const dualquat_t* dq);

/**
 * @brief 듀얼 쿼터니언의 부호를 정렬합니다.
 * 
 * real.w < 0일 경우 dualquat 전체에 -1을 곱해 정규화된 방향성을 유지합니다.
 * SLERP 등의 보간 과정에서 방향 뒤틀림을 방지하기 위해 사용됩니다.
 * 쿼터니언은 두 방향으로 같은 회전을 표현할 수 있어요!
 * quat(w, x, y, z)  ==  quat(-w, -x, -y, -z)
 * 문제 발생: 보간(SLERP)할 때 갑자기 방향이 "휙" 바뀜
 * quat a = (0.707, 0.0, 0.707, 0.0);   // 어떤 회전
 * quat b = (-0.707, 0.0, -0.707, 0.0); // 같은 회전인데 부호 반대
 * SLERP 전에 반드시 호출
 * dualquat_slerp()는 quat_slerp()가 내부적으로 align을 처리하므로 따로 넣을 필요 없음.
 * dualquat_lerp()는 반드시 align을 먼저 수행하고 사용하는 것을 권장
 * 이미 함수 내부에 들어갔으니까 굳이 호출할 필요는 없다.
 * 
 * @param[out] out 부호 정렬된 결과
 * @param[in] dq 원본 듀얼 쿼터니언
 */
BYUL_API void dualquat_align(dualquat_t* out, const dualquat_t* dq);

/**
 * @brief 두 듀얼 쿼터니언의 가중 평균을 계산합니다.
 * 
 * 단순 선형 보간 w1·a + w2·b 후 정규화 과정을 거쳐 결과를 생성합니다.
 * 이는 물리적으로 정확한 rigid transform 보간은 아니지만,
 * 다중 샘플 블렌딩 등에서 부드러운 결과를 생성하는 데 유용합니다.
 * 
 * @param[out] out 결과 듀얼 쿼터니언
 * @param[in] a 첫 번째 듀얼 쿼터니언
 * @param[in] w1 첫 번째 가중치
 * @param[in] b 두 번째 듀얼 쿼터니언
 * @param[in] w2 두 번째 가중치
 */
BYUL_API void dualquat_blend_weighted(dualquat_t* out, 
    const dualquat_t* a, float w1, const dualquat_t* b, float w2);

// -----------------------------
// dualquat_identity: 단위 회전 + 위치 0
// -----------------------------
void dualquat_identity(dualquat_t* out);

#ifdef __cplusplus
}
#endif

#endif // DUALQUAT_H
