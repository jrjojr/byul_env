/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* si_unit
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
* ElectricCharge                  coulomb	        C	    s*A, F*V                s*A
* QuantityOfElectricity           coulomb	        C		s*A, F*V	            s*A
* Voltage                         volt	            V		W/A, J/C	            kg*m2*s^-3*A^-1
* ElectricalPotentialDifference   volt	            V		W/A, J/C	            kg*m2*s^-3*A^-1
* ElectromotiveForce              volt	            V		W/A, J/C	            kg*m2*s^-3*A^-1
* ElectricalCapacitance           farad	            F		C/V, s/Ω	            kg^-1*m^-2*s4*A2
* ElectricalResistance            ohm	            Ω		1/S, V/A	            kg*m2*s^-3*A^-2
* Impedance                       ohm	            Ω		1/S, V/A	            kg*m2*s^-3*A^-2
* Reactance                       ohm	            Ω		1/S, V/A	            kg*m2*s^-3*A^-2
* ElectricalConductance           siemens	        S		1/Ω, A/V	            kg^-1*m^-2*s3*A2
* MagneticFlux                    weber	            Wb		J/A, T*m2,V*s           kg*m2*s^-2*A^-1
* MagneticInduction               tesla	            T		V*s/m2, Wb/m2, N/(A*m)	kg*s^-2*A^-1
* MagneticFluxDensity             tesla	            T		V*s/m2, Wb/m2, N/(A*m)	kg*s^-2*A^-1
* ElectricalInductance            henry	            H       V*s/A, Ω*s, Wb/A	    kg*m2*s^-2*A^-2
* TemperatureRelativeTo273dot15K  degree Celsius	°C		K	                    K
* LuminousFlux                    lumen	            lm		cd*sr	                cd
* Illuminance                     lux	            lx		lm/m2	                cd*m^-2
* CatalyticActivity               katal	            kat		mol/s	                s^-1*mol.
* 
***************************************************************************/

#ifndef SI_UNIT_H
#define SI_UNIT_H

#include "unit.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define A(dec)     SET_NIBBLE4(dec)
#define K(dec)     SET_NIBBLE5(dec)
#define mol(dec)   SET_NIBBLE6(dec)
#define cd(dec)    SET_NIBBLE7(dec)

typedef struct s_si_unit{
    Unit mUnit;
    USHORT mElectricCurrent:UNIT_BITS;
    USHORT mThermodynamicTemperature:UNIT_BITS;
    USHORT mAmountOfSubstance:UNIT_BITS;
    USHORT mLuminousIntensity:UNIT_BITS;
}si_unit_t;

typedef union u_si_unit{
    long   mBits;
    si_unit_t       mSiUnit_t;
}si_unit_n, SiUnit, *SiUnitPtr;

// extern const SiUnit EmptyUnit;
// 
// extern const SiUnit Length;
// extern const SiUnit Mass;
// extern const SiUnit Time;
// extern const SiUnit Angle;

extern const SiUnit ElectricCurrent;
extern const SiUnit ThermodynamicTemperature;   
extern const SiUnit AmountOfSubstance;
extern const SiUnit LuminousIntensity;

extern const SiUnit ElectricCharge;
extern const SiUnit QuantityOfElectricity;
extern const SiUnit Voltage;
extern const SiUnit ElectricalPotentialDifference;
extern const SiUnit ElectromotiveForce;
extern const SiUnit ElectricalCapacitance;
extern const SiUnit ElectricalResistance;
extern const SiUnit Impedance;
extern const SiUnit Reactance;
extern const SiUnit ElectricalConductance;
extern const SiUnit MagneticFlux;
extern const SiUnit MagneticInduction;
extern const SiUnit MagneticFluxDensity;
extern const SiUnit ElectricalInductance;
extern const SiUnit TemperatureRelativeTo273dot15K;
extern const SiUnit LuminousFlux;
extern const SiUnit Illuminance;
extern const SiUnit CatalyticActivity;

LIBAPI void print_si_unit_version(char* buf);
LIBAPI const char* get_si_unit_version(char* buf);

LIBAPI int init_si_unit(SiUnit* tSiUnit);
LIBAPI int release_si_unit(SiUnit* tSiUnit);

LIBAPI int assign_si_unit(SiUnit* tSiUnit, 
    const SiUnit* tOther);

LIBAPI int assign_si_unit_unit(SiUnit* tSiUnit, 
    const Unit* tOther);

LIBAPI bool is_si_unit_empty(const SiUnit* tSiUnit);

LIBAPI bool is_si_unit_equal(const SiUnit* tSiUnit, 
    const SiUnit* tOther);

LIBAPI int set_si_unit_bits(SiUnit* tSiUnit, const unsigned long tBits);

LIBAPI int set_si_unit_unit(SiUnit* tSiUnit, const Unit* tUnit);

LIBAPI int set_si_unit_electric_current(SiUnit* tSiUnit, 
    const USHORT tElectricCurrent);

LIBAPI int set_si_unit_thermodynamic_temperature(SiUnit* tSiUnit, 
    const USHORT tThermodynamicTemperature);

LIBAPI int set_si_unit_amount_of_substance(SiUnit* tSiUnit, 
    const USHORT tAmountOfSubstance);

LIBAPI int set_si_unit_luminous_intensity(SiUnit* tSiUnit, 
    const USHORT tLuminousIntensity);

LIBAPI const long  get_si_unit_bits(const SiUnit* tSiUnit);

const Unit LIBAPI get_si_unit_unit(const SiUnit* tSiUnit);

LIBAPI const USHORT get_si_unit_electric_current(const SiUnit* tSiUnit);

LIBAPI const USHORT get_si_unit_thermodynamic_temperature(const SiUnit* tSiUnit);

LIBAPI const USHORT get_si_unit_amount_of_substance(const SiUnit* tSiUnit);

LIBAPI const USHORT get_si_unit_luminous_intensity(const SiUnit* tSiUnit);

// 량 계산을 할때만 덧셈이 의미가 있다.
// 하지만, 유닛은 량이 아니다. 단지 량 속에 포함된 것이다.
LIBAPI int add_si_unit(SiUnit* tSiUnit, const SiUnit* tOther);

// 량 계산을 할때만 뺄셈이 의미가 있다.
// 하지만, 유닛은 량이 아니다. 단지 량 속에 포함된 것이다.
LIBAPI int sub_si_unit(SiUnit* tSiUnit, const SiUnit* tOther);

// 유닛을 곱하는 것은 유닛과 아더의 원소들을 각각 덧셈 한다는 것이다.
// 성공하면, SUCCESS, 실패하면 UNIT_OVERFLOW를 반환한다.
// UNIT_OVERFLOW일 때만, 실패한다. +-UNIT_MAX_ABS+1 부터 오버플로우이다. 
// 원소 중에 하나라도 오버플로우이면, 실패한다.
// -UNIT_MAX_ABS <= 각각의 원소 결과값 <= UNIT_MAX_ABS가 정상 범위이다.
LIBAPI int mul_si_unit(SiUnit* tSiUnit, const SiUnit* tOther);

// 유닛을 나누는 것은 유닛과 아더의 원소들을 각각 뺄셈 다는 것이다.
// 성공하면, SUCCESS, 실패하면 UNIT_OVERFLOW를 반환한다.
// UNIT_OVERFLOW일 때만, 실패한다. +-UNIT_MAX_ABS+1 부터 오버플로우이다. 
// 원소 중에 하나라도 오버플로우이면, 실패한다.
// -UNIT_MAX_ABS <= 각각의 원소 결과값 <= UNIT_MAX_ABS가 정상 범위이다.
LIBAPI int div_si_unit(SiUnit* tSiUnit, const SiUnit* tOther);

LIBAPI const char* si_unit_symbol(const SiUnit* tSiUnit);
LIBAPI const char* si_unit_name(const SiUnit* tSiUnit);
LIBAPI const char* si_unit_quantity_name(const SiUnit* tSiUnit);

LIBAPI const char* si_unit_symbol_derived(const SiUnit* tSiUnit);
LIBAPI const char* si_unit_name_derived(const SiUnit* tSiUnit);
LIBAPI const char* si_unit_quantity_name_derived(const SiUnit* tSiUnit);

LIBAPI const char* si_unit_symbol_basic(const SiUnit* tSiUnit);
LIBAPI const char* si_unit_name_basic(const SiUnit* tSiUnit);
LIBAPI const char* si_unit_quantity_name_basic(const SiUnit* tSiUnit);

LIBAPI const char* si_unit_symbol_non_si(const SiUnit* tSiUnit);
LIBAPI const char* si_unit_name_non_si(const SiUnit* tSiUnit);
LIBAPI const char* si_unit_quantity_name_non_si(const SiUnit* tSiUnit);

// si unit koherent 는 기호가 없다.
LIBAPI const char* si_unit_name_koherent(const SiUnit* tSiUnit);
LIBAPI const char* si_unit_quantity_name_koherent(const SiUnit* tSiUnit);

LIBAPI void print_si_unit(const SiUnit* tSelf);

#ifdef __cplusplus
}
#endif

#endif // SI_UNIT_H
