/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* unit 
*
***************************************************************************/

#include "unit.h"

#include <stdio.h>

#include <stdlib.h> // malloc, free
#include <string.h>

// mLength :UNIT_BITS;
// mMass   :UNIT_BITS;
// mTime   :UNIT_BITS;
// mAngle  :UNIT_BITS;

const Unit EmptyUnit        = {0};

const Unit Length       = { m(1) };
const Unit Mass         = { g(1) };
const Unit Time         = { s(1) };
const Unit Angle        = { rad(1) };

// SolidAngle                       Angle^2
const Unit SolidAngle           = { rad(2) };

// Frequency                        Time^-1
const Unit Frequency            = { s(-1) };

// Force                            Length * Mass * Time^-2
const Unit Force                = { m(1) | g(1) | s(-2) };

// Weight	                        Length      * Mass      * Time^-2
const Unit Weight               = { m(1) | g(1) | s(-2) };

// Pressure	                        Length^-1   * Mass      * Time^-2
const Unit Pressure             = { m(-1) | g(1) | s(-2) };

// Stress	                        Length^-1   * Mass      * Time^-2
const Unit Stress               = { m(-1) | g(1) | s(-2) };

// Energy	                        Length^2    * Mass      * Time^-2
const Unit Energy               = { m(2) | g(1) | s(-2) };

// Work	                            Length^2    * Mass      * Time^-2
const Unit Work                 = { m(2) | g(1) | s(-2) };

// Heat	                            Length^2    * Mass      * Time^-2
const Unit Heat                 = { m(2) | g(1) | s(-2) };

// Power	                        Length^2    * Mass      * Time^-3
const Unit Power                = { m(2) | g(1) | s(-3) };

// RadiantFlux                      Length^2    * Mass      * Time^-3
const Unit RadiantFlux          = { m(2) | g(1) | s(-3) };

// Speed                            Length * Time^-1
const Unit Speed                = { m(1) | s(-1) };

// Velocity                         Length * Time^-1
const Unit Velocity             = { m(1) | s(-1) };

// Acceleration                     Length * Time^-2
const Unit Acceleration         = { m(1) | s(-2) };

// Jerk                             Length * Time^-3
const Unit Jerk                 = { m(1) | s(-3) };

// Jolt                             Length * Time^-3
const Unit Jolt                 = { m(1) | s(-3) };

// Snap                             Length * Time^-4
const Unit Snap                 = { m(1) | s(-4) };

// Jounce                           Length * Time^-4
const Unit Jounce               = { m(1) | s(-4) };

// AngularVelocity                  Time^-1   * Angle
const Unit AngularVelocity      = { s(-1) | rad(1) };

// Angularacceleration              Time^-2  * Angle
const Unit AngularAcceleration  = { s(-2) | rad(1) };

// VolumetricFlow	                Length^3 * Time^-1
const Unit VolumetricFlow       = { m(3) | s(-1) };

// FrequencyDrift                   Time^-2
const Unit FrequencyDrift       = { s(-2) };

// Area	                            Length^2
const Unit Area                 = { m(2) };

// Volume	                        Length^3
const Unit Volume               = { m(3) };

// Momentum                         Length * Mass * Time^-1
const Unit Momentum             = { m(1) | g(1) | s(-1) };

// Impulse	                        Length * Mass * Time^-1
const Unit Impulse              = { m(1) | g(1) | s(-1) };

// AngularMomentum	                Length^2 * Mass * Time^-1
const Unit AngularMomentum      = { m(2) | g(1) | s(-1) };

// Torque                           Length^2 * Mass * Time^-2
const Unit Torque               = { m(2) | g(1) | s(-2) };

// MomentOfForce	                Length^2 * Mass * Time^-2
const Unit MomentOfForce        = { m(2) | g(1) | s(-2) };

// Yank	                            Length * Mass * Time^-3
const Unit Yank                 = { m(1) | g(1) | s(-3) };

// WaveNumber                       Length^-1
const Unit WaveNumber           = { m(-1) };

// OpticalPower                     Length^-1
const Unit OpticalPower         = { m(-1) };

// Curvature                        Length^-1
const Unit Curvature            = { m(-1) };

// SpatialFrequency	                Length^-1
const Unit SpatialFrequency     = { m(-1) };

// AreaDensity	                    Length^-2 * Mass
const Unit AreaDensity          = { m(-2) | g(1) };

// Density                          Length^-3 * Mass
const Unit Density              = { m(-3) | g(1) };

// MassDensity	                    Length^-3 * Mass
const Unit MassDensity          = { m(-3) | g(1) };

// SpecificVolume	                Length^3     * Mass^-1
const Unit SpecificVolume       = { m(3) | g(-1) };

// Action	                        Length^2 * Mass * Time^-1
const Unit Action               = { m(2) | g(1) | s(-1) };

// SpecificEnergy	                Length^2 * Time^-2
const Unit SpecificEnergy       = { m(2) | s(-2) };

// EnergyDensity	                Length^-1 * Mass * Time^-2
const Unit EnergyDensity        = { m(-1) | g(1) | s(-2) };

// SurfaceTension                   Mass * Time^-2
const Unit SurfaceTension       = { g(1) | s(-2) };

// Stiffness       	                Mass * Time^-2
const Unit Stiffness            = { g(1) | s(-2) };

// HeatFluxDensity                  Mass * Time^-3
const Unit HeatFluxDensity      = { g(1) | s(-3) };

// Irradiance	                    Mass * Time^-3
const Unit Irradiance           = { g(1) | s(-3) };

// Kinematicviscosity               Length^2 * Time^-1
const Unit KinematicViscosity   = { m(2) | s(-1) };

// ThermalDiffusivity               Length^2 * Time^-1
const Unit ThermalDiffusivity   = { m(2) | s(-1) };

// DiffusionCoefficient	            Length^2 * Time^-1
const Unit DiffusionCoefficient = { m(2) | s(-1) };

// DynamicViscosity	                Length^-1 * Mass * Time^-1
const Unit DynamicViscosity     = { m(-1) | g(1) | s(-1) };

// LinearMassDensity	            Length^-1 * Mass
const Unit LinearMassDensity    = { m(-1) | g(1) };

// MassFlowRate	                    Mass * Time^-1
const Unit MassFlowRate         = { g(1) | (-1) };

// Radiance	                        Mass * Time^-3
const Unit Radiance             = { g(1) | s(-3) };

// SpectralPower	                Length * Mass * Time^-3
const Unit SpectralPower        = { m(1) | g(1) | s(-3) };

// AbsorbedDoseRate	                Length^2 * Time^-3
const Unit AbsorbedDoseRate     = { m(2) | s(-3) };

// FuelEfficiency	                Length^-2
const Unit FuelEfficiency       = { m(-2) };

// SpectralIrradiance               Length^-1 * Mass * Time^-3
const Unit SpectralIrradiance   = { m(-1) | g(1) | s(-3) };

// PowerDensity       	            Length^-1 * Mass * Time^-3
const Unit PowerDensity         = { m(-1) | g(1) | s(-3) };

// EnergyFluxDensity	            Mass * Time^-3
const Unit EnergyFluxDensity    = { g(1) | s(-3) };

// Compressibility	                Length * Mass^-1 * Time^2
const Unit Compressibility      = { m(1) | g(-1) | s(2) };

// RadiantExposure	                Mass * Time^-2
const Unit RadiantExposure      = { g(1) | s(-2) };

// MomentOfInertia	                Length^2 * Mass
const Unit MomentOfInertia      = { m(2) | g(1) };

// SpecificAngularMomentum              Length^2 * Time^-1
const Unit SpecificAngularMomentum  = { m(2) | s(-1) };

// RadiantIntensity	                Length^2 * Mass * Time^-3
const Unit RadiantIntensity     = { m(2) | g(1) | s(-3) };

// SpectralIntensity	            Length * Mass * Time^-3
const Unit SpectralIntensity    = { m(1) | g(1) | s(-3) };

LIBAPI void print_unit_version(){
    printf("%s version : %d.%d.%d.%d\n", "unit", 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK);
}

LIBAPI const char* unit_version(char* buf){
    sprintf(buf, "%d.%d.%d.%d", 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK
        );
    return buf;
}

LIBAPI int init_unit(Unit* tUnit){
    return assign_unit(tUnit, &EmptyUnit);
}

LIBAPI int release_unit(Unit* tUnit){
    return assign_unit(tUnit, &EmptyUnit);
}

LIBAPI int assign_unit(Unit* tUnit, const Unit* tOther){
    if( tOther == NULL){
        DEBUG_PRINT("error, assign_unit : tOther is NULL");
        return -1;
    }
    tUnit->mBits = tOther->mBits;
    return 0;
}

LIBAPI bool unit_is_empty(const Unit* tUnit)
{
    if(tUnit == NULL){
        DEBUG_PRINT("error, unit_is_empty : tUnit is NULL");
        return false;
    }

    return (tUnit->mBits == 0);
}

LIBAPI bool unit_is_equal(const Unit* tUnit, const Unit* tOther)
{
    if(tUnit == NULL){
        DEBUG_PRINT("error, unit_is_equal : tUnit is NULL");
        return false;
    }

    if( tOther == NULL){
        DEBUG_PRINT("error, unit_is_equal : tOther is NULL");
        return false;
    }
        
    return (tUnit->mBits == tOther->mBits);
}

LIBAPI int set_unit(Unit* tUnit, const USHORT tBits){
    tUnit->mBits = tBits;
    return 0;
}

LIBAPI int set_unit_time(Unit* tUnit, const short tTime){
    tUnit->mBits = s(tTime);
    return 0;
}

LIBAPI int set_unit_length(Unit* tUnit, const short tLength){
    tUnit->mBits = m(tLength);
    return 0;
}

LIBAPI int set_unit_mass(Unit* tUnit, const short tMass){
    tUnit->mBits = g(tMass);
    return 0;
}

LIBAPI int set_unit_angle(Unit* tUnit, const short tAngle){
    tUnit->mBits = rad(tAngle);
    return 0;
}

LIBAPI int get_unit(const Unit* tUnit, USHORT* retBits){
    *retBits = tUnit->mBits;
    return 0;
}

LIBAPI int get_unit_time(const Unit* tUnit, short* retTime){
    *retTime = tUnit->mUnit_t.mTime;
    return 0;
}

LIBAPI int get_unit_length(const Unit* tUnit, short* retLength){
    *retLength = tUnit->mUnit_t.mLength;
    return 0;
}

LIBAPI int get_unit_mass(const Unit* tUnit, short* retMass){
    *retMass = tUnit->mUnit_t.mMass;
    return 0;
}

LIBAPI int get_unit_angle(const Unit* tUnit, short* retAngle){
    *retAngle = tUnit->mUnit_t.mAngle;
    return 0;
}

LIBAPI const USHORT unit(const Unit* tUnit){
    return tUnit->mBits;
}

LIBAPI const short unit_time(const Unit* tUnit){
    return tUnit->mUnit_t.mTime;
}

LIBAPI const short unit_length(const Unit* tUnit){
    return tUnit->mUnit_t.mLength;
}

LIBAPI const short unit_mass(const Unit* tUnit){
    return tUnit->mUnit_t.mMass;
}

LIBAPI const short unit_angle(const Unit* tUnit){
    return tUnit->mUnit_t.mAngle;
}

LIBAPI int add_unit(Unit* tUnit, const Unit* tOther){
// 량 계산을 할때만 덧셈이 의미가 있다.
// 하지만, 유닛은 량이 아니다. 단지 량 속에 포함된 것이다.
// 유닛이 같으면, SUCCESS_BUT_NOT_CHANGED, 다르면, IMPOSSILE을 반환한다.

    if (unit_is_equal(tUnit, tOther)){
        return UNIT_OP_RESULT_SUCCESS_BUT_NOT_CHANGED;
    }
    return UNIT_OP_RESULT_IMPOSSIBLE;
}

LIBAPI int sub_unit(Unit* tUnit, const Unit* tOther){
// 량 계산을 할때만 뺄셈이 의미가 있다.
// 하지만, 유닛은 량이 아니다. 단지 량 속에 포함된 것이다.
// 유닛이 같으면, SUCCESS_BUT_NOT_CHANGED, 다르면, 
// UNIT_OP_RESULT_IMPOSSIBLE 반환한다.

    if (unit_is_equal(tUnit, tOther)){
        return UNIT_OP_RESULT_SUCCESS_BUT_NOT_CHANGED;
    }
    return UNIT_OP_RESULT_IMPOSSIBLE;
}

LIBAPI int mul_unit(Unit* tUnit, const Unit* tOther){
// 유닛을 곱하는 것은 유닛과 아더의 원소들을 각각 덧셈 한다는 것이다.
// 성공하면, SUCCESS, 실패하면 UNIT_OVERFLOW를 반환한다.
// UNIT_OVERFLOW일 때만, 실패한다. +-8부터 오버플로우이다. 
// 원소 중에 하나라도 오버플로우이면, 실패한다.
// -7 <= 각각의 원소 결과값 <= 7가 정상 범위이다.

if ( tUnit->mUnit_t.mAngle + tOther->mUnit_t.mAngle > 7){
    return UNIT_OP_RESULT_OVERFLOW;
}

if ( tUnit->mUnit_t.mLength + tOther->mUnit_t.mLength > 7){
    return UNIT_OP_RESULT_OVERFLOW;
}

if ( tUnit->mUnit_t.mMass + tOther->mUnit_t.mMass > 7){
    return UNIT_OP_RESULT_OVERFLOW;
}

if ( tUnit->mUnit_t.mTime + tOther->mUnit_t.mTime > 7){
    return UNIT_OP_RESULT_OVERFLOW;
}

tUnit->mUnit_t.mAngle += tOther->mUnit_t.mAngle;
tUnit->mUnit_t.mLength += tOther->mUnit_t.mLength;
tUnit->mUnit_t.mMass += tOther->mUnit_t.mMass;
tUnit->mUnit_t.mTime += tOther->mUnit_t.mTime;

return UNIT_OP_RESULT_SUCCESS;

}

LIBAPI int div_unit(Unit* tUnit, const Unit* tOther){
// 유닛을 나누는 것은 유닛과 아더의 원소들을 각각 뺄셈 다는 것이다.
// 성공하면, SUCCESS, 실패하면 UNIT_OVERFLOW를 반환한다.
// UNIT_OVERFLOW일 때만, 실패한다. +-8부터 오버플로우이다. 
// 원소 중에 하나라도 오버플로우이면, 실패한다.
// -7 <= 각각의 원소 결과값 <= 7가 정상 범위이다.

if ( tUnit->mUnit_t.mAngle - tOther->mUnit_t.mAngle < -7){
    return UNIT_OP_RESULT_OVERFLOW;
}

if ( tUnit->mUnit_t.mLength - tOther->mUnit_t.mLength < -7){
    return UNIT_OP_RESULT_OVERFLOW;
}

if ( tUnit->mUnit_t.mMass - tOther->mUnit_t.mMass < -7){
    return UNIT_OP_RESULT_OVERFLOW;
}

if ( tUnit->mUnit_t.mTime - tOther->mUnit_t.mTime < -7){
    return UNIT_OP_RESULT_OVERFLOW;
}

tUnit->mUnit_t.mAngle -= tOther->mUnit_t.mAngle;
tUnit->mUnit_t.mLength -= tOther->mUnit_t.mLength;
tUnit->mUnit_t.mMass -= tOther->mUnit_t.mMass;
tUnit->mUnit_t.mTime -= tOther->mUnit_t.mTime;

return UNIT_OP_RESULT_SUCCESS;

} 

LIBAPI void print_unit(const Unit* tSelf){
    printf("unit { %d, %d, %d, %d }\n",
        tSelf->mUnit_t.mLength, 
        tSelf->mUnit_t.mMass, 
        tSelf->mUnit_t.mTime, 
        tSelf->mUnit_t.mAngle);
}

LIBAPI Unit* create_unit(const USHORT tBits){
    Unit* r;
    r = (Unit*)malloc(sizeof(Unit));
    init_unit(r);
    set_unit(r, tBits);
    return r;
}

LIBAPI Unit* create_unit_default(){
    Unit* r;
    r = (Unit*)malloc(sizeof(Unit));
    init_unit(r);
    return r;    
}

LIBAPI int delete_unit(Unit* tSelf){
    if(tSelf != NULL){
        free(tSelf);
    }
    return 0;
}

LIBAPI const char* unit_length_name(const Unit* tSelf){
    int aLength = tSelf->mUnit_t.mLength;
    if( aLength > 0){
        switch(aLength){
            case 0:
                // ""
                return "";

            case 1:
                // metre
                return "metre";

            case 2:
                // square metre
                return "square metre";

            case 3:
                // cubic metre
                return "cubic metre";

            case 4:
                // fourth power metre
                return "fourth power metre";

            case 5:
                // fifth power metre
                return "fifth power metre";

            case 6:
                // sixth power metre
                return "sixth power metre";

            case 7:
                // seventh power metre
                return "seventh power metre";
        }
    }
    else{
        switch(aLength){
            case -1:
                // minus metre
                return "minus metre";

            case -2:
                // minus square metre
                return "minus square metre";

            case -3:
                // minus cubic metre
                return "minus cubic metre";

            case -4:
                // minus fourth power metre
                return "minus fourth power metre";

            case -5:
                // minus fith power metre
                return "minus fith power metre";

            case -6:
                // minus sixth power metre
                return "minus sixth power metre";

            case -7:
                // minus seventh power metre
                return "minus seventh power metre";
        }
    }
    return "";
}

LIBAPI const char* unit_mass_name(const Unit* tSelf){
    int aMass = tSelf->mUnit_t.mMass;
    if( aMass > 0){
        switch(aMass){
            case 0:
                // ""
                return "";

            case 1:
                // gram
                return "gram";

            case 2:
                // square gram
                return "square gram";

            case 3:
                // cubic gram
                return "cubic gram";

            case 4:
                // fourth power gram
                return "fourth power gram";

            case 5:
                // fifth power gram
                return "fifth power gram";

            case 6:
                // sixth power gram
                return "sixth power gram";

            case 7:
                // seventh power gram
                return "seventh power gram";
        }
    }
    else{
        switch(aMass){
            case -1:
                // minus gram
                return "minus gram";

            case -2:
                // minus square gram
                return "minus square gram";

            case -3:
                // minus cubic gram
                return "minus cubic gram";

            case -4:
                // minus fourth power gram
                return "minus fourth power gram";

            case -5:
                // minus fith power gram
                return "minus fith power gram";

            case -6:
                // minus sixth power gram
                return "minus sixth power gram";

            case -7:
                // minus seventh power gram
                return "minus seventh power gram";
        }
    }
    return "";
}

LIBAPI const char* unit_time_name(const Unit* tSelf){
    int aTime = tSelf->mUnit_t.mTime;
    if( aTime > 0){
        switch(aTime){
            case 0:
                // ""
                return "";

            case 1:
                // second
                return "second";

            case 2:
                // square second
                return "square second";

            case 3:
                // cubic second
                return "cubic second";

            case 4:
                // fourth power second
                return "fourth power second";

            case 5:
                // fifth power second
                return "fifth power second";

            case 6:
                // sixth power second
                return "sixth power second";

            case 7:
                // seventh power second
                return "seventh power second";
        }
    }
    else{
        switch(aTime){
            case -1:
                // minus second
                return "minus second";

            case -2:
                // minus square second
                return "minus square second";

            case -3:
                // minus cubic second
                return "minus cubic second";

            case -4:
                // minus fourth power second
                return "minus fourth power second";

            case -5:
                // minus fith power second
                return "minus fith power second";

            case -6:
                // minus sixth power second
                return "minus sixth power second";

            case -7:
                // minus seventh power second
                return "minus seventh power second";
        }
    }
    return "";
}

LIBAPI const char* unit_angle_name(const Unit* tSelf){
    int aAngle = tSelf->mUnit_t.mAngle;
    if( aAngle > 0){
        switch(aAngle){
            case 0:
                // ""
                return "";

            case 1:
                // radian
                return "radian";

            case 2:
                // square radian
                return "square radian";

            case 3:
                // cubic radian
                return "cubic radian";

            case 4:
                // fourth power radian
                return "fourth power radian";

            case 5:
                // fifth power radian
                return "fifth power radian";

            case 6:
                // sixth power radian
                return "sixth power radian";

            case 7:
                // seventh power radian
                return "seventh power radian";
        }
    }
    else{
        switch(aAngle){
            case -1:
                // minus radian
                return "minus radian";

            case -2:
                // minus square radian
                return "minus square radian";

            case -3:
                // minus cubic radian
                return "minus cubic radian";

            case -4:
                // minus fourth power radian
                return "minus fourth power radian";

            case -5:
                // minus fith power radian
                return "minus fith power radian";

            case -6:
                // minus sixth power radian
                return "minus sixth power radian";

            case -7:
                // minus seventh power radian
                return "minus seventh power radian";
        }
    }
    return "";    
}

LIBAPI const char* unit_length_symbol(const Unit* tSelf){
    // m  m^2 m^3 m^4 m^5 m^6 m^7
    //    m^-2 m^-3 m^-4 m^-5 m^-6 m^-7
    // if( tSelf->mUnit_t.mLength == 0){
    //     return "";
    // }
    // else if( tSelf->mUnit_t.mLength == 1)
    // {
    //     return "m";
    // }
    // else{
    //     char buf[8] = "m^";
    //     char bufOther[4];
    //     itoa(tSelf->mUnit_t.mLength, bufOther, 10);
    //     strcat(buf, bufOther);
    //     return buf;
    // }

    int a = tSelf->mUnit_t.mLength;
    if( a > 0){
        switch(a){
            case 0:
                // ""
                return "";

            case 1:
                // metre
                return "m";

            case 2:
                // square metre
                return "m^2";

            case 3:
                // cubic metre
                return "m^3";

            case 4:
                // fourth power metre
                return "m^4";

            case 5:
                // fifth power metre
                return "m^5";

            case 6:
                // sixth power metre
                return "m^6";

            case 7:
                // seventh power metre
                return "m^7";
        }
    }
    else{
        switch(a){
            case -1:
                // minus metre
                return "m^-1";

            case -2:
                // minus square metre
                return "m^-2";

            case -3:
                // minus cubic metre
                return "m^-3";

            case -4:
                // minus fourth power metre
                return "m^-4";

            case -5:
                // minus fith power metre
                return "m^-5";

            case -6:
                // minus sixth power metre
                return "m^-6";

            case -7:
                // minus seventh power metre
                return "m^-7";
        }
    }
    return "";    
}

LIBAPI const char* unit_mass_symbol(const Unit* tSelf){
    int a = tSelf->mUnit_t.mMass;
    if( a > 0){
        switch(a){
            case 0:
                // ""
                return "";

            case 1:
                // gram
                return "g";

            case 2:
                // square gram
                return "g^2";

            case 3:
                // cubic gram
                return "g^3";

            case 4:
                // fourth power gram
                return "g^4";

            case 5:
                // fifth power gram
                return "g^5";

            case 6:
                // sixth power gram
                return "g^6";

            case 7:
                // seventh power gram
                return "g^7";
        }
    }
    else{
        switch(a){
            case -1:
                // minus gram
                return "g^-1";

            case -2:
                // minus square gram
                return "g^-2";

            case -3:
                // minus cubic gram
                return "g^-3";

            case -4:
                // minus fourth power gram
                return "g^-4";

            case -5:
                // minus fith power gram
                return "g^-5";

            case -6:
                // minus sixth power gram
                return "g^-6";

            case -7:
                // minus seventh power gram
                return "g^-7";
        }
    }
    return "";    
}

LIBAPI const char* unit_time_symbol(const Unit* tSelf){
    int a = tSelf->mUnit_t.mTime;
    if( a > 0){
        switch(a){
            case 0:
                // ""
                return "";

            case 1:
                // second
                return "s";

            case 2:
                // square second
                return "s^2";

            case 3:
                // cubic second
                return "s^3";

            case 4:
                // fourth power second
                return "s^4";

            case 5:
                // fifth power second
                return "s^5";

            case 6:
                // sixth power second
                return "s^6";

            case 7:
                // seventh power second
                return "s^7";
        }
    }
    else{
        switch(a){
            case -1:
                // minus second
                return "s^-1";

            case -2:
                // minus square second
                return "s^-2";

            case -3:
                // minus cubic second
                return "s^-3";

            case -4:
                // minus fourth power second
                return "s^-4";

            case -5:
                // minus fith power second
                return "s^-5";

            case -6:
                // minus sixth power second
                return "s^-6";

            case -7:
                // minus seventh power second
                return "s^-7";
        }
    }
    return "";    
}

LIBAPI const char* unit_angle_symbol(const Unit* tSelf){
    int a = tSelf->mUnit_t.mAngle;
    if( a > 0){
        switch(a){
            case 0:
                // ""
                return "";

            case 1:
                // radian
                return "rad";

            case 2:
                // square radian
                return "rad^2";

            case 3:
                // cubic radian
                return "rad^3";

            case 4:
                // fourth power radian
                return "rad^4";

            case 5:
                // fifth power radian
                return "rad^5";

            case 6:
                // sixth power radian
                return "rad^6";

            case 7:
                // seventh power radian
                return "rad^7";
        }
    }
    else{
        switch(a){
            case -1:
                // minus radian
                return "rad^-1";

            case -2:
                // minus square radian
                return "rad^-2";

            case -3:
                // minus cubic radian
                return "rad^-3";

            case -4:
                // minus fourth power radian
                return "rad^-4";

            case -5:
                // minus fith power radian
                return "rad^-5";

            case -6:
                // minus sixth power radian
                return "rad^-6";

            case -7:
                // minus seventh power radian
                return "rad^-7";
        }
    }
    return "";    
}

const int unit_count(const Unit* tSelf){
    return 4;
}

const int unit_dim_at(const Unit* tSelf, int tIndex){
    int i=0;
    int count = unit_count(tSelf);

    if(tIndex > count){
        tIndex = count;
    }
    else if(tIndex < 0){
        tIndex = 0;
    }

    switch(tIndex){
         case 0:
         i = get_m(tSelf->mBits);
         break;

         case 1:
         i = get_g(tSelf->mBits);
         break;

         case 2:
         i = get_s(tSelf->mBits);
         break;

         case 3:
         i = get_rad(tSelf->mBits);
         break;
    }
    return i;
}