#ifndef PROJECTILE_COMMON_H
#define PROJECTILE_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include "byul_common.h"
#include "entity_dynamic.h"

// ---------------------------------------------------------
// 구조체 및 콜백 타입 선언
// ---------------------------------------------------------
typedef struct s_projectile projectile_t;

/**
 * @brief 충돌 콜백 함수 타입
 * @param proj      충돌이 발생한 발사체
 * @param userdata  사용자 지정 데이터
 */
// typedef void (*projectile_hit_cb)(const projectile_t* proj, void* userdata);
typedef void (*projectile_hit_cb)(const void* projectile, void* userdata);


/**
 * @enum projectile_attr_t
 * @brief 발사체의 물리적 특성을 정의하는 열거형 (비트 플래그 방식).
 *
 * 이 열거형은 발사체가 가질 수 있는 
 * 여러 속성을 비트 플래그(bit flag) 방식으로 표현합니다.
 * 따라서 하나의 발사체가 여러 속성을 동시에 가질 수 있으며, 
 * 이를 통해 다양한 전략적 활용이 가능합니다.
 *
 * ---
 * 📌 속성(attribute) 설명
 * 🚩 IMPACT (충격)
 * 의도적으로 만든 인공 발사체로, 강력한 타격을 주기 위해 특수 제작됩니다. 
 * 무거운 질량과 단단한 재질로 인해, 
 * 적을 맞추는 순간 그 자리에서 운동 에너지를 집중적으로 방출하여 큰 피해를 줍니다.
 * 
 * 대표 발사체: 쇠망치, 철퇴, 중량 금속탄환 등
 * 
 * 🚩 PIERCE (관통)
 * 빠르고 날카로운 형태의 발사체로, 맞아도 멈추지 않고 목표물을 날카롭게 뚫고 지나갑니다. 
 * 속도와 날카로움으로 무장하여, 방어구나 두꺼운 장벽조차도 손쉽게 관통할 수 있습니다.
 * 
 * 대표 발사체: 총알, 철갑탄(AP), 고속 석궁 볼트 등
 * 
 * 🚩 ANCHOR (고정)
 * 표적에 맞으면 즉시 표면이나 몸체에 깊이 박혀 고정되는 발사체입니다. 
 * 맞은 이후에도 지속적으로 상대의 움직임을 제한하고, 
 * 추가적인 전술적 효과를 부여합니다.
 * 
 * 대표 발사체: 표창, 투창, 던지는 나이프, 독침 등
 *
 * 🚩 NONE (무속성) 🌿✨ (순수한 자연의 힘)
 * 인공적인 설계나 가공을 전혀 거치지 않고, 
 * 자연 그대로의 모습으로 사용되는 발사체입니다.
 * 무속성 발사체는 인간이 만든 발사체와 달리, 
 * 자연의 질감과 형태, 예측 불가능한 움직임과 효과로 적을 혼란스럽게 합니다.
 * 
 * 주요 특징과 강점
 * 
 * 🌱 즉각적이고 유연한 획득
 * 
 * 전투 환경 주변 어디서든 손쉽게 확보할 수 있어 전투 상황에서 
 * 가장 빠르고 효과적으로 사용 가능합니다.
 * 
 * 🌀 예측 불가능한 효과
 * 
 * 자연적으로 형성된 독특한 모양과 불규칙한 질량 분포로 인해 궤적이 불규칙할 수 있으며, 
 * 적이 이를 쉽게 예측하지 못해 방어 대응이 어렵습니다.
 * 
 * ✨ 자연 속성의 저항 무시
 * 
 * 인공적 특성에 대한 방어 효과나 속성 저항의 영향을 전혀 받지 않아, 
 * 어떤 환경이나 상대에게도 꾸준히 안정적인 효과를 발휘합니다.
 * 
 * 🍃 환경적 상호작용 극대화
 * 
 * 전장 주변 환경과 시너지를 일으켜 지형을 바꾸거나 장애물을 무너뜨리는 등 
 * 다양한 전술적 가능성을 제공합니다.
 * 
 * 대표 발사체: 돌멩이, 바위 조각, 나뭇가지, 자연 파편, 흙덩이 등
 * (순수한 자연의 힘을 그대로 활용한 전략적 무기)
 * 
 * ---
 * ## 🔗 복합 속성 설명
 *
 * 발사체는 **IMPACT**, **PIERCE**, **ANCHOR**의 
 * 조합으로 복합적 특성을 가질 수 있습니다.
 *
 * ### 1️⃣ IMPACT + PIERCE (충격 + 관통)
 * - **특징:** 뚫고 지나가며 강한 충격력도 동시에 가짐.
 * - **대표 무기:** 슬러그 탄환, 중량 석궁 볼트, 스파이크 철퇴.
 * - **전투 활용:** 두꺼운 장갑을 뚫으면서 충격 피해까지 가함.
 *
 * ### 2️⃣ IMPACT + ANCHOR (충격 + 고정)
 * - **특징:** 강한 충격과 함께 목표물에 깊게 박혀 고정.
 * - **대표 무기:** 던지는 도끼, 대형 표창, 금속 스파이크.
 * - **전투 활용:** 즉시 제압 및 지속적인 방해 효과 유발.
 *
 * ### 3️⃣ PIERCE + ANCHOR (관통 + 고정)
 * - **특징:** 관통 후 표적 내부에 박혀 고정됨.
 * - **대표 무기:** 화살(중속 시), 던지는 나이프, 강철 창.
 * - **전투 활용:** 관통 후 고정 효과로 지속 피해 및 제압.
 *
 * ### 4️⃣ IMPACT + PIERCE + ANCHOR (삼중 복합)
 * - **특징:** 충격, 관통, 박힘의 모든 특성을 가진 특수 발사체.
 * - **대표 무기:** 강화 투창, 스파이크 해머, 드릴형 투사체.
 * - **전투 활용:** 매우 강력하지만 제작과 사용 조건이 까다로운 특수 무기.
 *
 * ---
 * ## ⚔️ 속성 상성 (가위바위보 관계)
 *
 * - **IMPACT ➜ ANCHOR**  
 *   무거운 충격형 발사체가 박힌 발사체를 튕겨내거나 파괴.
 *
 * - **ANCHOR ➜ PIERCE**  
 *   고정형 발사체가 관통형 발사체의 궤도를 방해하거나 멈춤.
 *
 * - **PIERCE ➜ IMPACT**  
 *   관통형 발사체가 충격형 발사체를 쉽게 뚫고 지나감.
 *
 * ---
 * @note 화살은 속도나 상황에 따라 **PIERCE와 ANCHOR를 동시에 가질 수 있습니다.**
 */
typedef enum {
    PROJECTILE_ATTR_NONE   = 0,       ///< 무속성: 자연 상태 그대로의 발사체 (돌멩이 등)
    PROJECTILE_ATTR_IMPACT = 1 << 0,  ///< 충격: 맞고 즉시 강력한 타격을 주는 인공 발사체
    PROJECTILE_ATTR_PIERCE = 1 << 1,  ///< 관통: 맞고 뚫고 지나가는 날카로운 발사체
    PROJECTILE_ATTR_ANCHOR = 1 << 2   ///< 고정: 맞으면 박혀서 고정되는 발사체
} projectile_attr_t;

/**
 * @struct s_projectile
 * @brief 포탄, 미사일 등 **모든 발사체의 공통 속성**을 정의한 구조체.
 *
 * 발사체는 크게 **충격(Impact)**, **관통(Pierce)**, **고정(Anchor)** 
 * 세 가지 속성으로 분류됩니다.
 *
 * @note radius는 폭발 반경이 아닌, **영향 범위(명중 판정 반경)**로 사용됩니다.
 */
struct s_projectile {
    /**
     * @brief 동적 엔티티 기반 구조체.
     * @details 위치, 속도, 회전 등 물리 정보를 포함합니다.
     */
    entity_dynamic_t base;

    /**
     * @brief 기본 피해량. 1.0f
     * @details 발사체가 목표에 명중했을 때의 기본 피해 값입니다.
     */
    float damage;

    /**
     * @brief 발사체 속성 플래그.
     * @details PROJECTILE_ATTR_* 매크로(IMPACT / PIERCE / ANCHOR)를 조합하여 설정합니다.
     */
    projectile_attr_t attrs;

    /**
     * @brief 충돌 콜백 함수.
     * @details 발사체가 목표 또는 장애물에 충돌했을 때 호출되는 함수입니다.
     * NULL이면 충돌 시 아무 동작도 수행하지 않습니다.
     */
    projectile_hit_cb on_hit;

    /**
     * @brief 충돌 콜백용 사용자 데이터.
     * @details on_hit 호출 시 함께 전달됩니다.
     */
    void* hit_userdata;
};

// ---------------------------------------------------------
// 초기화 함수
// ---------------------------------------------------------

/**
 * @brief projectile_t 구조체를 **기본값**으로 초기화합니다.
 *
 * 이 함수는 `projectile_t` 인스턴스를 안전하게 초기화하기 위해 사용됩니다.  
 * 내부적으로 `entity_dynamic_init()`을 호출하여 `base` 필드를 초기화하고,  
 * 발사체의 **속성(attrs)**은 `PROJECTILE_ATTR_NONE`으로 설정됩니다.
 *
 * **기본 동작**
 * - `proj->base` : entity_dynamic_init()으로 초기화
 * - `proj->damage` : 1.0f
 * - `proj->attrs` : PROJECTILE_ATTR_NONE (속성 없음)
 * - `proj->on_hit` : NULL
 * - `proj->hit_userdata` : NULL
 *
 * @param[out] proj 초기화할 projectile_t 포인터 (NULL이면 동작하지 않음)
 *
 * **예시**
 * @code
 * projectile_t arrow;
 * projectile_init(&arrow);
 * arrow.attrs = PROJECTILE_ATTR_PIERCE | PROJECTILE_ATTR_ANCHOR; // 화살 속성 설정
 * arrow.damage = 25.0f;
 * @endcode
 */
BYUL_API void projectile_init(projectile_t* proj);

/**
 * @brief projectile_t를 **사용자 지정 값으로 완전 초기화**합니다.
 *
 * 주어진 파라미터를 기반으로 발사체의 **속성(attrs)**, `base`, `damage`,  
 * `on_hit`, `hit_userdata` 등을 한 번에 설정할 수 있습니다.  
 * `base`가 NULL이면 `entity_dynamic_init()`이 호출되어 기본값으로 초기화됩니다.
 *
 * @param[out] proj         초기화할 발사체 포인터 (NULL이면 동작하지 않음)
 * @param[in]  base         동적 엔티티 정보 (NULL이면 기본값 사용)
 * @param[in]  attrs   발사체 속성 (PROJECTILE_ATTR_IMPACT | PIERCE | ANCHOR 조합)
 * @param[in]  damage       발사체의 기본 피해량
 * @param[in]  on_hit       충돌 시 호출될 콜백 함수 (NULL 가능)
 * @param[in]  hit_userdata 콜백에 전달할 사용자 데이터 (NULL 가능)
 *
 * **예시**
 * @code
 * entity_dynamic_t dyn;
 * entity_dynamic_init(&dyn);
 *
 * projectile_t spear;
 * projectile_init_full(&spear, &dyn,
 *     PROJECTILE_ATTR_IMPACT | PROJECTILE_ATTR_ANCHOR, // 속성
 *     50.0f,                                           // 피해량
 *     on_spear_hit, user_data);
 * @endcode
 */
BYUL_API void projectile_init_full(
    projectile_t* proj,
    const entity_dynamic_t* base,
    projectile_attr_t attrs,
    float damage,
    projectile_hit_cb on_hit,
    void* hit_userdata
);

/**
 * @brief projectile_t를 다른 projectile_t로 복사합니다.
 * @param[out] out 복사 대상
 * @param[in]  src 원본
 */
BYUL_API void projectile_assign(projectile_t* out, const projectile_t* src);

// ---------------------------------------------------------
// 업데이트 및 동작 함수
// ---------------------------------------------------------
/**
 * @brief 발사체 상태 갱신
 *
 * - 위치 = 위치 + 속도 * dt
 * - 회전 = angular_velocity * dt 적용
 * - 수명(lifetime) 체크 후 만료 시 on_hit 콜백 호출
 *
 * @param proj 발사체
 * @param dt   시간 간격 (초)
 */
BYUL_API void projectile_update(projectile_t* proj, float dt);

// ---------------------------------------------------------
// 기본 충돌 콜백
// ---------------------------------------------------------
/**
 * @brief 기본 충돌 콜백
 * 충돌시 데미지를 출력한다.
 */
BYUL_API void projectile_default_hit_cb(
    const void* projectile, void* userdata);


#ifdef __cplusplus
}
#endif

#endif // PROJECTILE_COMMON_H
