/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* si_basic_unit 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK
*
* SI basic unit (SI 기본 단위)
* Quantity(량)                            Name(이름)          Symbol
* -------------------------------------------------------------------------
* Time(시간)                              초(second)          s
* Length(길이)                            미터(metre)         m
* Mass(질량)                              킬로그램(kilogram)  kg
* ElectricCurrent(전류)                   암페어(ampere)      A
* ThermodynamicTemperature(열역학적온도)  켈빈(kelvin)        K
* LuminousIntensity(광도)                 몰(mole)            mol
* AmountOfSubstance(물질량)               칸델라(candela)     cd
* 
* 기본량	이름	기호	정의
* 길이	미터	m	
*   1 m는 빛이 진공에서 1/299,792,458초 동안 진행한 경로의 길이이다.
* 
* 질량	킬로그램	kg	
*   1 kg은 질량의 단위이며 
*   플랑크 상수 h가 정확히 
*   6.626 070 15 * 10^-34 J * s  (J = kg * m^2 * s^-2)
*   이 되도록 하는 값이다.
* 
* 시간	초	s	
*   1초는 온도가 0K인 세슘-133 원자의 바닥 상태에 있는 
*   두 초미세 준위 사이의 전이에 대응하는 복사선의 
*   9,192,631,770
*   주기의 지속 시간이다.
* 
* 전류	암페어	A	
*   암페어(기호: A)는 전류의 SI 단위이다. 
*   암페어는 기본전하 e를 C(쿨롬)단위로 나타낼 때 
*   1.602 176 634 * 10^-19
*   이 되도록 하는 전류로 정의된다. 
*   여기에서 C는 A × s와 같은 유도 단위이다.
* 
* 온도	켈빈	K	
*   켈빈(기호: K)은 열역학적 온도의 SI 단위이다. 
*   켈빈은 볼츠만 상수 k를 J * K^-1 (J = kg * m^2 * s^-2) 단위로 나타낼 때 
*   1. 380 649 × 10^-23이 되도록 정의된다.
* 
* 물질량	몰	mol	
*   아보가드로 상수가 NA = 6.022 140 76 * 10^23 mol^?1가 되도록 하는 단위.
*   1몰에 해당하는 입자의 수인 6.022 140 76 * 10^23은 아보가드로 수라고 부른다.
*   몰을 사용할 때에는 구성요소를 반드시 명시해야 하며 이 구성 요소는 
*   원자, 분자, 이온, 전자, 기타 입자 또는 이 입자들의 특정한 집합체가 될 수 있다.
* 
* 광도	칸델라	cd	
*   1 cd는 진동수 540 * 10^12 헤르츠인 단색광을 방출하는 광원의 복사도가 
*   어떤 주어진 방향으로 스테라디안당 1/683 와트일 때 이 방향에 대한 광도이다.
* 
***************************************************************************/

#ifndef SI_BASIC_UNIT_H
#define SI_BASIC_UNIT_H

#include "unit/unit_config.h"

#include <stdbool.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UNIT_BITS 4

typedef struct s_si_basic_unit{
    int mTime:UNIT_BITS;
    int mLength:UNIT_BITS;
    int mMass:UNIT_BITS;
    int mElectricCurrent:UNIT_BITS;
    int mThermodynamicTemperature:UNIT_BITS;
    int mAmountOfSubstance:UNIT_BITS;
    int mLuminousIntensity:UNIT_BITS;    
    int mDummy:UNIT_BITS;
}si_basic_unit_t, SiBasicUnit, *SiBasicUnitPtr;

extern const SiBasicUnit EmptyUnit;
extern const SiBasicUnit Time;
extern const SiBasicUnit Length;
extern const SiBasicUnit Mass;
extern const SiBasicUnit ElectricCurrent;
extern const SiBasicUnit ThermodynamicTemperature;
extern const SiBasicUnit AmountOfSubstance;
extern const SiBasicUnit LuminousIntensity;

LIBAPI void print_si_basic_unit_version(char* buf);
LIBAPI const char* get_si_basic_unit_version(char* buf);

LIBAPI int init_si_basic_unit(SiBasicUnit* tSiBasicUnit);
LIBAPI int release_si_basic_unit(SiBasicUnit* tSiBasicUnit);

LIBAPI int assign_si_basic_unit(SiBasicUnit* tSiBasicUnit, 
    const SiBasicUnit* tOther);

LIBAPI bool is_si_basic_unit_empty(const SiBasicUnit* tSiBasicUnit);

LIBAPI bool is_si_basic_unit_equal(const SiBasicUnit* tSiBasicUnit, 
    const SiBasicUnit* tOther);

LIBAPI const char* get_si_basic_unit_symbol(const SiBasicUnit* tSiBasicUnit);

LIBAPI const char* get_si_basic_unit_name(const SiBasicUnit* tSiBasicUnit);

LIBAPI const char* get_si_basic_unit_quantity_name(const SiBasicUnit* tSiBasicUnit);

#ifdef __cplusplus
}
#endif

#endif // SI_BASIC_UNIT_H
