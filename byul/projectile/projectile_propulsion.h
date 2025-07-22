#ifndef PROJECTILE_PROPULSION_H
#define PROJECTILE_PROPULSION_H

#include "internal/controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @struct propulsion_t
 * @brief 발사체 추진 장치(Propulsion) 상태 구조체
 *
 * thrust 제어기(controller)와 연료 상태를 포함해 발사체의 추진력을 관리합니다.
 */
typedef struct s_propulsion {
    // --- 기본 성능 ---
    float max_thrust;            ///< 최대 출력 (N). 0.0f 이상
    float current_thrust;        ///< 현재 출력 (controller가 계산)
    float fuel_capacity;         ///< 총 연료량 (kg)
    float fuel_remaining;        ///< 남은 연료 (kg)
    float burn_rate;             ///< 연료 소모율 (kg/s). 출력 1N당 소모 비율 (비효율 반영 전)

    // --- 효율 관련 ---
    float efficiency;            ///< 추진 효율 (0.0 ~ 1.0). 예: 0.7 = 70% 효율
    float thermal_loss;          ///< 열 손실 계수 (0.0 ~ 0.2). 출력 시 손실되는 비율
    float energy_density;        ///< 연료 에너지 밀도 (MJ/kg). thrust 계산 시 참고

    // --- 동적 응답 ---
    float response_time;         ///< 목표 thrust 도달에 필요한 반응 시간 (초)
    float max_thrust_rate;       ///< 최대 추력 변화율 (N/s). 추력이 점진적으로 증가/감소하는 속도 제한
    float delay_time;            ///< 제어 입력 후 실제 출력이 반영되기까지의 지연 (초)

    // --- 열 및 마모 ---
    float heat;                  ///< 현재 축적된 열량 (가상 단위)
    float heat_dissipation_rate; ///< 열 방출 속도 (단위 시간당 감소량)
    float wear_level;            ///< 마모율 (0.0 ~ 1.0). 1.0이면 성능 저하 발생

    // --- 컨트롤러 ---
    controller_t* controller;    ///< thrust 크기 제어기 (PID, MPC 등)
    bool active;                 ///< 추진 시스템 활성 여부
} propulsion_t;


// ---------------------------------------------------------------------------
// 초기화 및 상태 관리
// ---------------------------------------------------------------------------

/**
 * @brief propulsion_t 기본 초기화
 *
 * 이 함수는 propulsion_t 구조체의 모든 필드를 현실적인 초기값으로 설정합니다.
 * 초기화 후, 엔진은 비활성화(active=false) 상태이며, 연료와 기본 성능 파라미터가
 * 지정된 초기값으로 세팅됩니다.
 *
 * ### 기본 설정값:
 * - **max_thrust = 120.0f**  
 *   - 최대 출력(추력) 값 (단위: N)
 *   - 소형 로켓 엔진/드론 모터 수준의 평균값
 *
 * - **current_thrust = 0.0f**  
 *   - 현재 추력 (초기에는 0)
 *
 * - **fuel_capacity = 50.0f**  
 *   - 총 연료량 (kg)
 *
 * - **fuel_remaining = 50.0f**  
 *   - 현재 남은 연료량 (초기값 = 연료탱크 풀)
 *
 * - **burn_rate = 0.05f**  
 *   - 연료 소모율 (kg/s). 100N 출력 시 약 2분 지속 가능
 *
 * - **efficiency = 0.7f**  
 *   - 추진 효율 (0.0 ~ 1.0). 70% 에너지 변환 효율
 *
 * - **thermal_loss = 0.05f**  
 *   - 열 손실 비율 (5%)
 *
 * - **energy_density = 42.0f**  
 *   - 연료의 에너지 밀도 (MJ/kg, 케로신 기준값)
 *
 * - **response_time = 0.8f**  
 *   - 목표 추력까지 도달하는 시간 (초)
 *
 * - **max_thrust_rate = 30.0f**  
 *   - 초당 추력 변화 한계치 (N/s)
 *
 * - **delay_time = 0.2f**  
 *   - 제어 명령 반영 지연시간 (초)
 *
 * - **heat = 0.0f**  
 *   - 초기 열량
 *
 * - **heat_dissipation_rate = 0.3f**  
 *   - 열 방출 속도 (단위 시간당 감소량)
 *
 * - **wear_level = 0.0f**  
 *   - 마모율 (0.0 ~ 1.0, 1.0일 때 성능 저하 최대)
 *
 * - **controller = NULL**  
 *   - 기본적으로 제어기 없음 (직접 제어 필요)
 *
 * - **active = false**  
 *   - 초기 상태에서 비활성화
 *
 * @param p 초기화할 propulsion_t 포인터 (NULL이면 동작 없음)
 */
BYUL_API void propulsion_init(propulsion_t* p);

/**
 * @brief propulsion_t 상세 초기화
 *
 * 사용자가 지정한 파라미터로 propulsion_t를 초기화합니다.  
 * max_thrust, fuel_capacity, burn_rate 등 주요 성능 파라미터는
 * **표준 범위**를 참고하여 설정하는 것을 권장합니다.
 *
 * ---
 * ### **표준 권장 범위 (참고용):**
 *
 * - **max_thrust:** 10.0f ~ 1000.0f (N)  
 *   - 10N: 소형 드론 프로펠러 수준  
 *   - 100N: 소형 로켓/대형 드론 수준  
 *   - 1000N: 대형 소형로켓 엔진 수준
 *
 * - **fuel_capacity:** 1.0f ~ 500.0f (kg)  
 *   - 1~10kg: 경량 드론, 소형 로켓  
 *   - 50~100kg: 중형 추진체  
 *   - 500kg 이상: 고중량 엔진
 *
 * - **burn_rate:** 0.01f ~ 5.0f (kg/s)  
 *   - 0.01~0.1 kg/s: 저연료 소비형 모터  
 *   - 0.5~2 kg/s: 일반 로켓 엔진  
 *   - 5 kg/s 이상: 고성능 엔진
 *
 * - **controller:**  
 *   - NULL이면 제어기 없이 수동 thrust 제어  
 *   - PID, MPC 등 controller_t 포인터를 연결 가능
 *
 * - **active:**  
 *   - true: 초기화 직후 활성화  
 *   - false: 초기화 직후 비활성화
 *
 * ---
 * @param p 초기화할 propulsion_t 포인터 (NULL이면 동작 없음)
 * @param max_thrust 최대 출력 (N)
 * @param fuel_capacity 총 연료량 (kg)
 * @param burn_rate 연료 소모율 (kg/s)
 * @param ctrl 제어기 포인터 (NULL 가능)
 * @param active 초기 활성화 여부 (true = ON)
 *
 * @note 표준 범위를 벗어난 값도 허용되지만, 비현실적인 시뮬레이션 결과를
 *       초래할 수 있으니 주의가 필요합니다.
 */
BYUL_API void propulsion_init_full(propulsion_t* p,
                                   float max_thrust,
                                   float fuel_capacity,
                                   float burn_rate,
                                   controller_t* ctrl,
                                   bool active);

/**
 * @brief propulsion_t 상태 복사
 */
BYUL_API void propulsion_assign(propulsion_t* dst, const propulsion_t* src);

/**
 * @brief propulsion_t를 초기 상태로 리셋 (연료/출력 초기화)
 */
BYUL_API void propulsion_reset(propulsion_t* p);

// ---------------------------------------------------------------------------
// 갱신 및 상태 조회
// ---------------------------------------------------------------------------

/**
 * @brief 추진 장치 상태 갱신
 *
 * 주어진 시간 간격(`dt`) 동안 `target_thrust`를 목표로 추진 장치의 상태를 업데이트합니다.  
 * 제어기(controller)가 연결되어 있으면 이를 사용하여 실제 추력을 계산하며,  
 * 연료 소모, 효율, 마모, 열 축적 등을 반영합니다.
 *
 * ---
 * ### **주요 연산 과정**
 * 1. **목표 추력 제한:**  
 *    - `0 <= target_thrust <= max_thrust`
 *
 * 2. **컨트롤러 연산:**  
 *    - PID, MPC 등 연결된 `controller_t`를 사용해 실제 추력값을 보정합니다.  
 *      `desired_thrust = controller_compute(ctrl, target_thrust, current_thrust, dt)`
 *
 * 3. **효율 및 손실 반영:**  
 *    - `desired_thrust *= efficiency`  
 *    - `desired_thrust *= (1.0 - thermal_loss)`  
 *    - `desired_thrust *= (1.0 - wear_level * 0.3)`  
 *      (마모율 1.0이면 30% 성능 저하)
 *
 * 4. **추력 변화율 제한:**  
 *    - `max_delta = max_thrust_rate * dt`  
 *    - `current_thrust += clamp(desired_thrust - current_thrust, -max_delta, max_delta)`
 *
 * 5. **연료 소모 계산:**  
 *    - `fuel_needed = burn_rate * current_thrust * dt`  
 *    - 연료 부족 시 가능한 최대 추력만 적용하고 active=false로 전환
 *
 * 6. **열 축적 및 방출:**  
 *    - `heat += current_thrust * 0.05`  
 *    - `heat -= heat_dissipation_rate * dt`
 *
 * 7. **마모율 증가:**  
 *    - `wear_level += 0.0001 * current_thrust * dt`
 *
 * ---
 *
 * @param p propulsion_t 포인터 (NULL이면 동작 없음)
 * @param target_thrust 목표 추력 (N), 0 ~ max_thrust 범위
 * @param dt 시뮬레이션 시간 간격 (초), 0보다 커야 함
 *
 * @note
 * - 연료가 부족하면 `current_thrust`는 가능한 값으로 강제 축소됩니다.
 * - `active=false` 상태에서는 항상 `current_thrust=0`으로 설정됩니다.
 * - 실제 thrust는 효율(efficiency)과 손실(thermal_loss, wear_level)에 의해 줄어들 수 있습니다.
 */
BYUL_API void propulsion_update(propulsion_t* p, float target_thrust, float dt);


/**
 * @brief 현재 추진력(N)을 반환
 */
BYUL_API float propulsion_get_thrust(const propulsion_t* p);

/**
 * @brief 연료가 모두 소진되었는지 확인
 */
BYUL_API bool propulsion_is_empty(const propulsion_t* p);

/**
 * @brief 연료 잔량 비율 (0.0 ~ 1.0)
 */
BYUL_API float propulsion_get_fuel_ratio(const propulsion_t* p);

/**
 * @brief 현재 thrust로 최대 지속 가능 시간(초)을 반환
 */
BYUL_API float propulsion_get_max_runtime(const propulsion_t* p);

// ---------------------------------------------------------------------------
// 연료 관리
// ---------------------------------------------------------------------------

/**
 * @brief 연료 보충
 */
BYUL_API void propulsion_refuel(propulsion_t* p, float amount);

/**
 * @brief 연료 강제 소모
 */
BYUL_API void propulsion_consume(propulsion_t* p, float amount);

/**
 * @brief 남은 연료로 발휘 가능한 총 임펄스 (N·s)
 */
BYUL_API float propulsion_get_remaining_impulse(const propulsion_t* p);

// ---------------------------------------------------------------------------
// 예측 관련
// ---------------------------------------------------------------------------

/**
 * @brief 현재 연료량으로 desired_thrust(N)를 몇 초간 유지 가능한지 예측
 *
 * 지정한 목표 추력(desired_thrust)을 유지할 경우,
 * 현재 연료량으로 몇 초 동안 작동할 수 있는지 계산합니다.
 *
 * ---
 * **계산 공식**
 *   runtime = fuel_remaining / (burn_rate * desired_thrust)
 *
 * ---
 * @param p propulsion_t 포인터
 * @param desired_thrust 유지하고자 하는 추력 (N)
 *
 * @return 유지 가능한 시간 (초), 연료가 부족하거나 입력값이 잘못된 경우 0.0f
 *
 * @note
 * - desired_thrust가 0 이하이면 항상 0.0f를 반환합니다.
 * - burn_rate가 0인 비정상 상태도 0.0f 반환.
 */
BYUL_API float propulsion_predict_runtime(
    const propulsion_t* p, float desired_thrust);

/**
 * @brief 현재 thrust가 유지될 때 연료가 소진되는 시간(초) 예측
 *
 * 현재 current_thrust 값으로 연료가 완전히 소진될 때까지 걸리는
 * 시간을 계산합니다.
 *
 * ---
 * **계산 공식**
 *   time_to_empty = fuel_remaining / (burn_rate * current_thrust)
 *
 * ---
 * @param p propulsion_t 포인터
 * @return 연료 소진까지의 시간(초). thrust가 0이거나 연료가 없으면 0.0f
 */
BYUL_API float propulsion_predict_empty_time(const propulsion_t* p);

/**
 * @brief 특정 기간 동안 발휘할 수 있는 평균 thrust(N) 예측
 *
 * 주어진 시간 duration 동안 가능한 평균 추력을 예측합니다.
 * 현재 남은 연료와 burn_rate를 기반으로 계산됩니다.
 *
 * ---
 * **계산 공식**
 *   avg_thrust = min(max_thrust,
 *                    fuel_remaining / (burn_rate * duration))
 *
 * ---
 * @param p propulsion_t 포인터
 * @param duration 목표 시간 (초)
 *
 * @return 예측 가능한 평균 추력 (N). 연료 부족 또는 잘못된 입력 시 0.0f
 */
BYUL_API float propulsion_predict_max_thrust(
    const propulsion_t* p, float duration);

// ---------------------------------------------------------------------------
// 제어기 관리
// ---------------------------------------------------------------------------

/**
 * @brief 추진기 활성/비활성 설정
 */
BYUL_API void propulsion_set_active(propulsion_t* p, bool active);

/**
 * @brief 새로운 controller를 추진기에 연결
 */
BYUL_API void propulsion_attach_controller(
    propulsion_t* p, controller_t* ctrl);

/**
 * @brief 현재 controller를 제거
 */
BYUL_API void propulsion_detach_controller(propulsion_t* p);

// ---------------------------------------------------------------------------
// 디버깅/로깅
// ---------------------------------------------------------------------------

/**
 * @brief 현재 추진기의 상태를 콘솔에 출력
 */
BYUL_API void propulsion_print(const propulsion_t* p);

/**
 * @brief 현재 추진기 상태를 문자열로 반환
 */
BYUL_API const char* propulsion_to_string(
    const propulsion_t* p, char* buffer, size_t buffer_size);

/**
 * @brief 현재 추진기 상태를 JSON 포맷 문자열로 변환
 * 예: {"thrust":80.0, "fuel":45.0, "capacity":100.0, "active":1}
 */
BYUL_API const char* propulsion_to_json(
    const propulsion_t* p, char* buffer, size_t buffer_size);

#ifdef __cplusplus
}
#endif

#endif // PROJECTILE_PROPULSION_H
