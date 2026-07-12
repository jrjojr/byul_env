/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* unit 1.0.2.3
*
* Quantity(유도량)
* ------------
* Length(길이)
* Mass(질량)
* Time(시간) 
* Angle(각도)
* 
* SolidAngle                                                        Angle^2
* 
* Frequency	                                          Time^-1
* Force	                    Length      * Mass      * Time^-2
* Weight	                Length      * Mass      * Time^-2
* Pressure	                Length^-1   * Mass      * Time^-2
* Stress	                Length^-1   * Mass      * Time^-2
* Energy	                Length^2    * Mass      * Time^-2
* Work	                    Length^2    * Mass      * Time^-2
* Heat	                    Length^2    * Mass      * Time^-2
* Power	                    Length^2    * Mass      * Time^-3
* RadiantFlux               Length^2    * Mass      * Time^-3
*  
* Speed                     Length                  * Time^-1
* Velocity                  Length                  * Time^-1
* Acceleration              Length                  * Time^-2
* Jerk                      Length                  * Time^-3
* Jolt                      Length                  * Time^-3
* Snap                      Length                  * Time^-4
* Jounce                    Length                  * Time^-4
* Angularvelocity                                     Time^-1   * Angle
* Angularacceleration                                 Time^-2   * Angle
* VolumetricFlow	        Length^3                  Time^-1
* FrequencyDrift                                      Time^-2
* 
* Area	                    Length^2
* Volume	                Length^3
* Momentum                  Length      * Mass      * Time^-1
* Impulse	                Length      * Mass      * Time^-1
* AngularMomentum	        Length^2    * Mass      * Time^-1
* Torque                    Length^2    * Mass      * Time^-2
* MomentOfForce	            Length^2    * Mass      * Time^-2
* Yank	                    Length      * Mass      * Time^-3
* Wavenumber                Length^-1
* OpticalPower              Length^-1
* Curvature                 Length^-1
* SpatialFrequency	        Length^-1
* AreaDensity	            Length^-2   * Mass
* Density                   Length^-3   * Mass
* MassDensity	            Length^-3   * Mass
* SpecificVolume	        Length^3    * Mass^-1
* Action	                Length^2    * Mass      * Time^-1
* SpecificEnergy	        Length^2                * Time^-2
* EnergyDensity	            Length^-1   * Mass      * Time^-2
* SurfaceTension                          Mass      * Time^-2
* Stiffness       	                      Mass      * Time^-2
* HeatFluxDensity                         Mass      * Time^-3
* Irradiance	                          Mass      * Time^-3
* Kinematicviscosity        Length^2                * Time^-1
* ThermalDiffusivity        Length^2                * Time^-1
* DiffusionCoefficient	    Length^2                * Time^-1
* DynamicViscosity	        Length^-1   * Mass      * Time^-1
* LinearMassDensity	        Length^-1   * Mass
* MassFlowRate	                          Mass      * Time^-1
* Radiance	                              Mass      * Time^-3
* SpectralPower	            Length      * Mass      * Time^-3
* AbsorbedDoseRate	        Length^2                * Time^-3
* FuelEfficiency	        Length^-2
* SpectralIrradiance        Length^-1   * Mass      * Time^-3
* PowerDensity       	    Length^-1   * Mass      * Time^-3
* EnergyFluxDensity	                      Mass      * Time^-3
* Compressibility	        Length      * Mass^-1   * Time^2
* RadiantExposure	                      Mass      * Time^-2
* MomentOfInertia	        Length^2    * Mass
* SpecificAngularMomentum   Length^2                * Time^-1
* RadiantIntensity	        Length^2    * Mass      * Time^-3
* SpectralIntensity	        Length      * Mass      * Time^-3
* 
becquerel	Bq	activity referred to a radionuclide (decays per unit time)	s?1	
gray	Gy	absorbed dose (of ionising radiation)	m2?s?2	J/kg
sievert	Sv	equivalent dose (of ionising radiation)	m2?s?2	J/kg
***************************************************************************/

#ifndef UNIT_H
#define UNIT_H

#include "unit/unit_config.h"
#include "nibble.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USHORT          unsigned short

#define UNIT_BITS       NIBBLE_BITS
#define UNIT_MAX_ABS    NIBBLE_MAX_ABS

#define m(dec)     SET_NIBBLE0(dec)
#define g(dec)     SET_NIBBLE1(dec)
#define s(dec)     SET_NIBBLE2(dec)
#define rad(dec)   SET_NIBBLE3(dec)

#define get_m(dec)     NIBBLE0(dec)
#define get_g(dec)     NIBBLE1(dec)
#define get_s(dec)     NIBBLE2(dec)
#define get_rad(dec)   NIBBLE3(dec)

typedef struct s_unit{
    signed mLength :UNIT_BITS;
    signed mMass   :UNIT_BITS;
    signed mTime   :UNIT_BITS;
    signed mAngle  :UNIT_BITS;
}unit_t;

typedef union u_unit{
    USHORT          mBits;
    unit_t          mUnit_t;
}unit_n;

typedef unit_n Unit;
typedef Unit* UnitPtr;

typedef enum e_unit_op_result{
    UNIT_OP_RESULT_IMPOSSIBLE                 = -2,
    UNIT_OP_RESULT_FAIL                       = -1,
    UNIT_OP_RESULT_SUCCESS                    =  0,
    UNIT_OP_RESULT_SUCCESS_BUT_NOT_CHANGED    =  1,
    UNIT_OP_RESULT_OVERFLOW              =  2
}unit_op_result_m;

typedef unit_op_result_m UnitOpResult;

extern const Unit EmptyUnit;

extern const Unit Length;
extern const Unit Mass;
extern const Unit Time;
extern const Unit Angle;

extern const Unit SolidAngle;

extern const Unit Frequency;
extern const Unit Force;
extern const Unit Weight;
extern const Unit Pressure;
extern const Unit Stress;

extern const Unit Energy;
extern const Unit Work;
extern const Unit Heat;
extern const Unit Power;
extern const Unit RadiantFlux;
 
extern const Unit Speed;
extern const Unit Velocity;
extern const Unit Acceleration;
extern const Unit Jerk;
extern const Unit Jolt;
extern const Unit Snap;
extern const Unit Jounce;
extern const Unit AngularVelocity;
extern const Unit AngularAcceleration;
extern const Unit VolumetricFlow;
extern const Unit FrequencyDrift;

extern const Unit Area;
extern const Unit Volume;
extern const Unit Momentum;
extern const Unit Impulse;
extern const Unit AngularMomentum;
extern const Unit Torque;
extern const Unit MomentOfForce;
extern const Unit Yank;
extern const Unit WaveNumber;
extern const Unit OpticalPower;
extern const Unit SpatialFrequency;
extern const Unit AreaDensity;
extern const Unit Density;
extern const Unit MassDensity;
extern const Unit SpecificVolume;
extern const Unit Action;
extern const Unit SpecificEnergy;
extern const Unit EnergyDensity;
extern const Unit SurfaceTension;
extern const Unit Stiffness;
extern const Unit HeatFluxDensity;
extern const Unit Irradiance;
extern const Unit KinematicViscosity;
extern const Unit ThermalDiffusivity;
extern const Unit DiffusionCoefficient;
extern const Unit DynamicViscosity;
extern const Unit LinearMassDensity;
extern const Unit MassFlowRate;
extern const Unit Radiance;
extern const Unit SpectralPower;
extern const Unit AbsorbedDoseRate;
extern const Unit FuelEfficiency;
extern const Unit SpectralIrradiance;
extern const Unit PowerDensity;
extern const Unit EnergyFluxDensity;
extern const Unit Compressibility;
extern const Unit RadiantExposure;
extern const Unit MomentOfInertia;
extern const Unit SpecificAngularMomentum;
extern const Unit RadiantIntensity;
extern const Unit SpectralIntensity;

LIBAPI void print_unit_version();

LIBAPI const char* unit_version(char* buf);

LIBAPI int init_unit(Unit* tUnit);

LIBAPI int release_unit(Unit* tUnit);

LIBAPI int assign_unit(Unit* tUnit, const Unit* tOther);

LIBAPI bool unit_is_empty(const Unit* tUnit);

LIBAPI bool unit_is_equal(const Unit* tUnit, const Unit* tOther);

LIBAPI int set_unit(Unit* tUnit, const USHORT tBits);

LIBAPI int set_unit_time(Unit* tUnit, const short tTime);

LIBAPI int set_unit_length(Unit* tUnit, const short tLength);

LIBAPI int set_unit_mass(Unit* tUnit, const short tMass);

LIBAPI int set_unit_angle(Unit* tUnit, const short tAngle);

LIBAPI int get_unit(const Unit* tUnit, USHORT* retBits);

LIBAPI int get_unit_time(const Unit* tUnit, short* retTime);

LIBAPI int get_unit_length(const Unit* tUnit, short* retLength);

LIBAPI int get_unit_mass(const Unit* tUnit, short* retMass);

LIBAPI int get_unit_angle(const Unit* tUnit, short* retAngle);

LIBAPI const USHORT unit(const Unit* tUnit);

LIBAPI const short unit_time(const Unit* tUnit);

LIBAPI const short unit_length(const Unit* tUnit);

LIBAPI const short unit_mass(const Unit* tUnit);

LIBAPI const short unit_angle(const Unit* tUnit);

// 량 계산을 할때만 덧셈이 의미가 있다.
// 하지만, 유닛은 량이 아니다. 단지 량 속에 포함된 것이다.
// 유닛이 같으면, UNIT_OP_RESULT_SUCCESS_BUT_NOT_CHANGED, 
// 다르면, UNIT_OP_RESULT_IMPOSSIBLE 반환한다.
LIBAPI int add_unit(Unit* tUnit, const Unit* tOther);

// 량 계산을 할때만 뺄셈이 의미가 있다.
// 하지만, 유닛은 량이 아니다. 단지 량 속에 포함된 것이다.
// 유닛이 같으면, UNIT_OP_RESULT_SUCCESS_BUT_NOT_CHANGED, 
// 다르면, UNIT_OP_RESULT_IMPOSSIBLE 반환한다.
LIBAPI int sub_unit(Unit* tUnit, const Unit* tOther);

// 유닛을 곱하는 것은 유닛과 아더의 원소들을 각각 덧셈 한다는 것이다.
// 성공하면, UNIT_OP_RESULT_SUCCESS, 실패하면 UNIT_OP_RESULT_OVERFLOW 반환한다.
// OVERFLOW일 때만, 실패한다. +-8부터 오버플로우이다. 
// 원소 중에 하나라도 오버플로우이면, 실패한다.
// -7 <= 각각의 원소 결과값 <= 7가 정상 범위이다.
LIBAPI int mul_unit(Unit* tUnit, const Unit* tOther);

// 유닛을 나누는 것은 유닛과 아더의 원소들을 각각 뺄셈 다는 것이다.
// 성공하면, UNIT_OP_RESULT_SUCCESS, 실패하면 UNIT_OP_RESULT_OVERFLOW 반환한다.
// OVERFLOW일 때만, 실패한다. +-8부터 오버플로우이다. 
// 원소 중에 하나라도 오버플로우이면, 실패한다.
// -7 <= 각각의 원소 결과값 <= 7가 정상 범위이다.
LIBAPI int div_unit(Unit* tUnit, const Unit* tOther);

LIBAPI void print_unit(const Unit* tSelf);

// m(DEC) | g(DEC) | s(DEC) | rad(DEC)으로 초기화 한다.
// DEC의 범위는 -15 ~ 15 이다.
// 범위는 4비트이다. 4비트를 넘어서면 다섯번째 비트부터는 잘린다.
LIBAPI Unit* create_unit(const USHORT tBits);

LIBAPI Unit* create_unit_default();

LIBAPI int delete_unit(Unit* tSelf);

LIBAPI const char* unit_length_name(const Unit* tSelf);
LIBAPI const char* unit_mass_name(const Unit* tSelf);
LIBAPI const char* unit_time_name(const Unit* tSelf);
LIBAPI const char* unit_angle_name(const Unit* tSelf);

LIBAPI const char* unit_length_symbol(const Unit* tSelf);
LIBAPI const char* unit_mass_symbol(const Unit* tSelf);
LIBAPI const char* unit_time_symbol(const Unit* tSelf);
LIBAPI const char* unit_angle_symbol(const Unit* tSelf);

const int unit_count(const Unit* tSelf);
const int unit_dim_at(const Unit* tSelf, int tIndex);

#ifdef __cplusplus
}
#endif

#endif // UNIT_H
