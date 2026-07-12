/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* si_unit
*
***************************************************************************/

#include "si_unit.h"

#include <stdio.h>

// const SiUnit EmptyUnit                = {0x00000000};

// const SiUnit Length                   = {m(1)};
// const SiUnit Mass                     = {g(1)};
// const SiUnit Time                     = {s(1)};
// const SiUnit Angle                    = {rad(1)};

const SiUnit ElectricCurrent            = {A(1)};
const SiUnit ThermodynamicTemperature   = {K(1)};
const SiUnit AmountOfSubstance          = {mol(1)};
const SiUnit LuminousIntensity          = {cd(1)};

// ElectricCharge                  	s*A
const SiUnit ElectricCharge                     = {s(1) | A(1)};

// QuantityOfElectricity           	s*A
const SiUnit QuantityOfElectricity              = {s(1) | A(1)};

// Voltage                         	kg*m2*s^-3*A^-1
const SiUnit Voltage                            = {g(1) | m(2) | s(-3) | A(-1)};

// ElectricalPotentialDifference   	kg*m2*s^-3*A^-1
const SiUnit ElectricalPotentialDifference      = {g(1) | m(2) | s(-3) | A(-1)};

// ElectromotiveForce              	kg*m2*s^-3*A^-1
const SiUnit ElectromotiveForce                 = {g(1) | m(2) | s(-3) | A(-1)};

// ElectricalCapacitance           	kg^-1*m^-2*s4*A2
const SiUnit ElectricalCapacitance              = {g(-1) | m(-2) | s(4) | A(2)};

// ElectricalResistance            	kg*m2*s^-3*A^-2
const SiUnit ElectricalResistance               = {g(1) | m(2) | s(-3) | A(-2)};

// Impedance                       	kg*m2*s^-3*A^-2
const SiUnit Impedance                          = {g(1) | m(2) | s(-3) | A(-2)};

// Reactance                       	kg*m2*s^-3*A^-2
const SiUnit Reactance                          = {g(1) | m(2) | s(-3) | A(-2)};

// ElectricalConductance           	kg^-1*m^-2*s3*A2
const SiUnit ElectricalConductance              = {g(-1) | m(-2) | s(3) | A(2)};

// MagneticFlux                     kg*m2*s^-2*A^-1
const SiUnit MagneticFlux                       = {g(1) | m(2) | s(-2) | A(-1)};

// MagneticInduction                kg*s^-2*A^-1
const SiUnit MagneticInduction                  = {g(1) | s(-2) | A(-1)};

// MagneticFluxDensity              kg*s^-2*A^-1
const SiUnit MagneticFluxDensity                = {g(1) | s(-2) | A(-1)};

// ElectricalInductance             kg*m2*s^-2*A^-2
const SiUnit ElectricalInductance               = {g(1) | m(2) | s(-2) | A(-2)};

// TemperatureRelativeTo273dot15K  	K
const SiUnit TemperatureRelativeTo273dot15K     = {K(1)};

// LuminousFlux                    	cd
const SiUnit LuminousFlux                       = {cd(1)};

// Illuminance                     	cd*m^-2
const SiUnit Illuminance                        = {cd(1) | m(-2)};

// CatalyticActivity               	s^-1*mol.
const SiUnit CatalyticActivity                  = {s(-1) | mol(1)};

LIBAPI int init_si_unit(SiUnit* tSiUnit){
    // tSiUnit = (SiUnit*) malloc(sizeof(SiUnit));
    if (tSiUnit == NULL){
        DEBUG_PRINT("init_si_unit malloc error.\n");
        return -1;
    }
    set_si_unit_bits(tSiUnit, 0);
    // assign_si_unit(tSiUnit, &EmptySiUnit);
    return 0;
}

LIBAPI int release_si_unit(SiUnit* tSiUnit){
    if(tSiUnit != NULL){
        DEBUG_PRINT("release_si_unit tSiUnit is not NULL.\n");
        // free(tSiUnit);
        return 0;
    }
    else{
        DEBUG_PRINT("error, release_si_unit tSiUnit is NULL.\n");
        return -1;
    }
}

LIBAPI int assign_si_unit(SiUnit* tSiUnit, 
    const SiUnit* tOther){
    if(tSiUnit == NULL){
        DEBUG_PRINT("error, assign_si_unit : tSiUnit is NULL");
        return -1;        
    }

    if( tOther == NULL){
        DEBUG_PRINT("error, assign_si_unit : tOther is NULL");
        return -2;
    }
    
    assign_unit(&tSiUnit->mSiUnit_t.mUnit, &tOther->mSiUnit_t.mUnit);
    tSiUnit->mSiUnit_t.mElectricCurrent = tOther->mSiUnit_t.mElectricCurrent;
    tSiUnit->mSiUnit_t.mThermodynamicTemperature = 
        tOther->mSiUnit_t.mThermodynamicTemperature;
    tSiUnit->mSiUnit_t.mLuminousIntensity = 
        tOther->mSiUnit_t.mLuminousIntensity;
    tSiUnit->mSiUnit_t.mAmountOfSubstance = 
        tOther->mSiUnit_t.mAmountOfSubstance;
    return 0;
}

LIBAPI int assign_si_unit_unit(SiUnit* tSiUnit, 
    const Unit* tOther){
    init_si_unit(tSiUnit);
    assign_unit(&tSiUnit->mSiUnit_t.mUnit, tOther);
    return 0;
}

LIBAPI bool is_si_unit_empty(const SiUnit* tSiUnit)
{
    if(tSiUnit == NULL){
        DEBUG_PRINT("error, is_si_unit_empty : tSiUnit is NULL");
        return false;
    }

    return unit_is_empty(&tSiUnit->mSiUnit_t.mUnit)
        && (tSiUnit->mSiUnit_t.mElectricCurrent == 0)
        && (tSiUnit->mSiUnit_t.mThermodynamicTemperature == 0)
        && (tSiUnit->mSiUnit_t.mLuminousIntensity == 0)
        && (tSiUnit->mSiUnit_t.mAmountOfSubstance == 0);
}

LIBAPI bool is_si_unit_equal(const SiUnit* tSiUnit, 
    const SiUnit* tOther)
{
    if(tSiUnit == NULL){
        DEBUG_PRINT("error, is_si_unit_equal : tSiUnit is NULL");
        return false;
    }

    if( tOther == NULL){
        DEBUG_PRINT("error, is_si_unit_equal : tOther is NULL");
        return false;
    }
        
    return unit_is_equal(&tSiUnit->mSiUnit_t.mUnit, &tOther->mSiUnit_t.mUnit)
        && (tSiUnit->mSiUnit_t.mElectricCurrent == 
            tOther->mSiUnit_t.mElectricCurrent)
        && (tSiUnit->mSiUnit_t.mThermodynamicTemperature == 
            tOther->mSiUnit_t.mThermodynamicTemperature)
        && (tSiUnit->mSiUnit_t.mLuminousIntensity == 
            tOther->mSiUnit_t.mLuminousIntensity)
        && (tSiUnit->mSiUnit_t.mAmountOfSubstance == 
            tOther->mSiUnit_t.mAmountOfSubstance);
}

LIBAPI int set_si_unit_bits(SiUnit* tSiUnit, const unsigned long tBits){
    tSiUnit->mBits = tBits;
    return 0;
}

LIBAPI int set_si_unit_unit(SiUnit* tSiUnit, const Unit* tUnit){
    return assign_unit(&tSiUnit->mSiUnit_t.mUnit, tUnit);
}

LIBAPI int set_si_unit_time(SiUnit* tUnit, const USHORT tTime){
    tUnit->mSiUnit_t.mUnit.mTime = tTime;
    return 0;
}

LIBAPI int set_si_unit_length(SiUnit* tUnit, const USHORT tLength){
    tUnit->mSiUnit_t.mUnit.mLength = tLength;
    return 0;
}

LIBAPI int set_si_unit_mass(SiUnit* tUnit, const USHORT tMass){
    tUnit->mSiUnit_t.mUnit.mMass = tMass;
    return 0;
}

LIBAPI int set_si_unit_angle(SiUnit* tUnit, const USHORT tAngle){
    tUnit->mSiUnit_t.mUnit.mAngle = tAngle;
    return 0;
}

LIBAPI int set_si_unit_electric_current(SiUnit* tSiUnit, 
    const USHORT tElectricCurrent){
        tSiUnit->mSiUnit_t.mElectricCurrent = tElectricCurrent;
        return 0;
    }

LIBAPI int set_si_unit_thermodynamic_temperature(SiUnit* tSiUnit, 
    const USHORT tThermodynamicTemperature){
        tSiUnit->mSiUnit_t.mThermodynamicTemperature = 
            tThermodynamicTemperature;
        return 0;
    }

LIBAPI int set_si_unit_amount_of_substance(SiUnit* tSiUnit, 
    const USHORT tAmountOfSubstance){
        tSiUnit->mSiUnit_t.mAmountOfSubstance = tAmountOfSubstance;
        return 0;
    }

LIBAPI int set_si_unit_luminous_intensity(SiUnit* tSiUnit, 
    const USHORT tLuminousIntensity){
        tSiUnit->mSiUnit_t.mLuminousIntensity = tLuminousIntensity;
        return 0;
    }

LIBAPI const long  get_si_unit_bits(const SiUnit* tSiUnit){
    return tSiUnit->mBits;
}    

const Unit LIBAPI get_si_unit_unit(const SiUnit* tSiUnit){
    return tSiUnit->mSiUnit_t.mUnit;
}

LIBAPI const USHORT get_si_unit_length(const SiUnit* tSiUnit){
    return tSiUnit->mSiUnit_t.mUnit.mLength;
}

LIBAPI const USHORT get_si_unit_mass(const SiUnit* tSiUnit){
    return tSiUnit->mSiUnit_t.mUnit.mMass;
}

LIBAPI const USHORT get_si_unit_angle(const SiUnit* tSiUnit){
    return tSiUnit->mSiUnit_t.mUnit.mAngle;
}

LIBAPI const USHORT get_si_unit_electric_current(const SiUnit* tSiUnit){
    return tSiUnit->mSiUnit_t.mElectricCurrent;
}

LIBAPI const USHORT get_si_unit_thermodynamic_temperature(const SiUnit* tSiUnit){
    return tSiUnit->mSiUnit_t.mThermodynamicTemperature;
}

LIBAPI const USHORT get_si_unit_amount_of_substance(const SiUnit* tSiUnit){
    return tSiUnit->mSiUnit_t.mAmountOfSubstance;
}

LIBAPI const USHORT get_si_unit_luminous_intensity(const SiUnit* tSiUnit){
    return tSiUnit->mSiUnit_t.mLuminousIntensity;
}

// 량 계산을 할때만 덧셈이 의미가 있다.
// 하지만, 유닛은 량이 아니다. 단지 량 속에 포함된 것이다.
// 유닛이 같으면, SUCCESS_BUT_NOT_CHANGED, 다르면, IMPOSSILE을 반환한다.
LIBAPI int add_si_unit(SiUnit* tSiUnit, const SiUnit* tOther){
    if (is_si_unit_equal(tSiUnit, tOther)){
        return UNIT_OP_RESULT_SUCCESS_BUT_NOT_CHANGED;
    }
    return UNIT_OP_RESULT_IMPOSSIBLE;
}

// 량 계산을 할때만 뺄셈이 의미가 있다.
// 하지만, 유닛은 량이 아니다. 단지 량 속에 포함된 것이다.
// 유닛이 같으면, SUCCESS_BUT_NOT_CHANGED, 다르면, IMPOSSILE을 반환한다.
LIBAPI int sub_si_unit(SiUnit* tSiUnit, const SiUnit* tOther){
    if (is_si_unit_equal(tSiUnit, tOther)){
        return UNIT_OP_RESULT_SUCCESS_BUT_NOT_CHANGED;
    }
    return UNIT_OP_RESULT_IMPOSSIBLE;
}

// 유닛을 곱하는 것은 유닛과 아더의 원소들을 각각 덧셈 한다는 것이다.
// 성공하면, SUCCESS, 실패하면 UNIT_OVERFLOW를 반환한다.
// UNIT_OVERFLOW일 때만, 실패한다. +-UNIT_MAX_ABS+1 부터 오버플로우이다. 
// 원소 중에 하나라도 오버플로우이면, 실패한다.
// -UNIT_MAX_ABS <= 각각의 원소 결과값 <= UNIT_MAX_ABS가 정상 범위이다.
LIBAPI int mul_si_unit(SiUnit* tSiUnit, const SiUnit* tOther){
    if (mul_unit(&tSiUnit->mSiUnit_t.mUnit, &tOther->mSiUnit_t.mUnit) == 
        UNIT_OP_RESULT_OVERFLOW){
        return UNIT_OP_RESULT_OVERFLOW;
    }

    if ( tSiUnit->mSiUnit_t.mAmountOfSubstance + 
        tOther->mSiUnit_t.mAmountOfSubstance > UNIT_MAX_ABS){
        return UNIT_OP_RESULT_OVERFLOW;
    }

    if ( tSiUnit->mSiUnit_t.mElectricCurrent + 
        tOther->mSiUnit_t.mElectricCurrent > UNIT_MAX_ABS){
        return UNIT_OP_RESULT_OVERFLOW;
    }

    if ( tSiUnit->mSiUnit_t.mLuminousIntensity + 
        tOther->mSiUnit_t.mLuminousIntensity > UNIT_MAX_ABS){
        return UNIT_OP_RESULT_OVERFLOW;
    }

    if ( tSiUnit->mSiUnit_t.mThermodynamicTemperature + 
        tOther->mSiUnit_t.mThermodynamicTemperature > UNIT_MAX_ABS){
        return UNIT_OP_RESULT_OVERFLOW;
    }

    tSiUnit->mSiUnit_t.mAmountOfSubstance += 
        tOther->mSiUnit_t.mAmountOfSubstance;
    
    tSiUnit->mSiUnit_t.mElectricCurrent += 
        tOther->mSiUnit_t.mElectricCurrent;

    tSiUnit->mSiUnit_t.mLuminousIntensity += 
        tOther->mSiUnit_t.mLuminousIntensity;

    tSiUnit->mSiUnit_t.mThermodynamicTemperature += 
        tOther->mSiUnit_t.mThermodynamicTemperature;

    return UNIT_OP_RESULT_SUCCESS;
}

// 유닛을 나누는 것은 유닛과 아더의 원소들을 각각 뺄셈 다는 것이다.
// 성공하면, SUCCESS, 실패하면 UNIT_OVERFLOW를 반환한다.
// UNIT_OVERFLOW일 때만, 실패한다. +-UNIT_MAX_ABS+1 부터 오버플로우이다. 
// 원소 중에 하나라도 오버플로우이면, 실패한다.
// -UNIT_MAX_ABS <= 각각의 원소 결과값 <= UNIT_MAX_ABS가 정상 범위이다.
LIBAPI int div_si_unit(SiUnit* tSiUnit, const SiUnit* tOther){
    if (div_unit(&tSiUnit->mSiUnit_t.mUnit, &tOther->mSiUnit_t.mUnit) == 
        UNIT_OP_RESULT_OVERFLOW){
        return UNIT_OP_RESULT_OVERFLOW;
    }

    if ( tSiUnit->mSiUnit_t.mAmountOfSubstance - 
        tOther->mSiUnit_t.mAmountOfSubstance > UNIT_MAX_ABS){
        return UNIT_OP_RESULT_OVERFLOW;
    }

    if ( tSiUnit->mSiUnit_t.mElectricCurrent - 
        tOther->mSiUnit_t.mElectricCurrent > UNIT_MAX_ABS){
        return UNIT_OP_RESULT_OVERFLOW;
    }

    if ( tSiUnit->mSiUnit_t.mLuminousIntensity - 
        tOther->mSiUnit_t.mLuminousIntensity > UNIT_MAX_ABS){
        return UNIT_OP_RESULT_OVERFLOW;
    }

    if ( tSiUnit->mSiUnit_t.mThermodynamicTemperature - 
        tOther->mSiUnit_t.mThermodynamicTemperature > UNIT_MAX_ABS){
        return UNIT_OP_RESULT_OVERFLOW;
    }

    tSiUnit->mSiUnit_t.mAmountOfSubstance -= 
        tOther->mSiUnit_t.mAmountOfSubstance;
    
    tSiUnit->mSiUnit_t.mElectricCurrent -= 
        tOther->mSiUnit_t.mElectricCurrent;

    tSiUnit->mSiUnit_t.mLuminousIntensity -= 
        tOther->mSiUnit_t.mLuminousIntensity;

    tSiUnit->mSiUnit_t.mThermodynamicTemperature -= 
        tOther->mSiUnit_t.mThermodynamicTemperature;

    return UNIT_OP_RESULT_SUCCESS;

}

LIBAPI const char* si_unit_symbol(const SiUnit* tSiUnit)
{
    if(tSiUnit == NULL){
        DEBUG_PRINT("error, si_unit_symbol : tSiUnit is NULL");
        return NULL;
    }

    return "";
}

LIBAPI const char* si_unit_name(const SiUnit* tSiUnit){
    if(tSiUnit == NULL){
        DEBUG_PRINT("error, si_unit_name : tSiUnit is NULL");
        return NULL;
    }

    return "";
}

LIBAPI const char* si_unit_quantity_name(const SiUnit* tSiUnit)
{
    if(tSiUnit == NULL){
        DEBUG_PRINT("error, si_unit_quantity_name : tSiUnit is NULL");
        return NULL;
    }

    return "";
}

LIBAPI const char* si_unit_symbol_derived(const SiUnit* tSiUnit){
    return "";
}

LIBAPI const char* si_unit_name_derived(const SiUnit* tSiUnit){
    if(tSiUnit == NULL){
        DEBUG_PRINT(
                "error, si_unit_name_derived : tSiUnit is NULL");
        return NULL;
    }

    if (is_si_unit_equal(tSiUnit, &ElectricCharge)){
        return "ElectricCharge";
    }

    if (is_si_unit_equal(tSiUnit, &QuantityOfElectricity)){
        return "QuantityOfElectricity";
    }

    if (is_si_unit_equal(tSiUnit, &Voltage)){
        return "Voltage";
    }    

    if (is_si_unit_equal(tSiUnit, &ElectricalPotentialDifference)){
        return "ElectricalPotentialDifference";
    }    

    if (is_si_unit_equal(tSiUnit, &ElectromotiveForce)){
        return "ElectromotiveForce";
    }        

    if (is_si_unit_equal(tSiUnit, &ElectricalCapacitance)){
        return "ElectricalCapacitance";
    }            

    if (is_si_unit_equal(tSiUnit, &ElectricalResistance)){
        return "ElectricalResistance";
    }            

    if (is_si_unit_equal(tSiUnit, &Impedance)){
        return "Impedance";
    }            

    if (is_si_unit_equal(tSiUnit, &Reactance)){
        return "Reactance";
    }            

    if (is_si_unit_equal(tSiUnit, &ElectricalConductance)){
        return "ElectricalConductance";
    }            

    if (is_si_unit_equal(tSiUnit, &MagneticFlux)){
        return "MagneticFlux";
    }            

    if (is_si_unit_equal(tSiUnit, &MagneticInduction)){
        return "MagneticInduction";
    }                

    if (is_si_unit_equal(tSiUnit, &MagneticFluxDensity)){
        return "MagneticFluxDensity";
    }                

    if (is_si_unit_equal(tSiUnit, &ElectricalInductance)){
        return "ElectricalInductance";
    }                

    if (is_si_unit_equal(tSiUnit, &TemperatureRelativeTo273dot15K)){
        return "TemperatureRelativeTo273dot15K";
    }                    

    if (is_si_unit_equal(tSiUnit, &LuminousFlux)){
        return "LuminousFlux";
    }                        

    if (is_si_unit_equal(tSiUnit, &Illuminance)){
        return "Illuminance";
    }                            

    if (is_si_unit_equal(tSiUnit, &CatalyticActivity)){
        return "CatalyticActivity";
    }                 

    return "";           
}

LIBAPI const char* si_unit_quantity_name_derived(const SiUnit* tSiUnit){
    if(tSiUnit == NULL){
        DEBUG_PRINT(
                "error, si_unit_quantity_name_derived : tSiUnit is NULL");
        return NULL;
    }

    if (is_si_unit_equal(tSiUnit, &ElectricCharge)){
        return "ElectricCharge";
    }

    if (is_si_unit_equal(tSiUnit, &QuantityOfElectricity)){
        return "QuantityOfElectricity";
    }

    if (is_si_unit_equal(tSiUnit, &Voltage)){
        return "Voltage";
    }    

    if (is_si_unit_equal(tSiUnit, &ElectricalPotentialDifference)){
        return "ElectricalPotentialDifference";
    }    

    if (is_si_unit_equal(tSiUnit, &ElectromotiveForce)){
        return "ElectromotiveForce";
    }        

    if (is_si_unit_equal(tSiUnit, &ElectricalCapacitance)){
        return "ElectricalCapacitance";
    }            

    if (is_si_unit_equal(tSiUnit, &ElectricalResistance)){
        return "ElectricalResistance";
    }            

    if (is_si_unit_equal(tSiUnit, &Impedance)){
        return "Impedance";
    }            

    if (is_si_unit_equal(tSiUnit, &Reactance)){
        return "Reactance";
    }            

    if (is_si_unit_equal(tSiUnit, &ElectricalConductance)){
        return "ElectricalConductance";
    }            

    if (is_si_unit_equal(tSiUnit, &MagneticFlux)){
        return "MagneticFlux";
    }            

    if (is_si_unit_equal(tSiUnit, &MagneticInduction)){
        return "MagneticInduction";
    }                

    if (is_si_unit_equal(tSiUnit, &MagneticFluxDensity)){
        return "MagneticFluxDensity";
    }                

    if (is_si_unit_equal(tSiUnit, &ElectricalInductance)){
        return "ElectricalInductance";
    }                

    if (is_si_unit_equal(tSiUnit, &TemperatureRelativeTo273dot15K)){
        return "TemperatureRelativeTo273dot15K";
    }                    

    if (is_si_unit_equal(tSiUnit, &LuminousFlux)){
        return "LuminousFlux";
    }                        

    if (is_si_unit_equal(tSiUnit, &Illuminance)){
        return "Illuminance";
    }                            

    if (is_si_unit_equal(tSiUnit, &CatalyticActivity)){
        return "CatalyticActivity";
    }                            
        
    return "";
}

LIBAPI const char* si_unit_symbol_basic(const SiUnit* tSiUnit){
    if(tSiUnit == NULL){
        DEBUG_PRINT("error, si_unit_symbol_basic : tSiUnit is NULL");
        return NULL;
    }

    if (unit_is_equal(&tSiUnit->mSiUnit_t.mUnit, &Length)){
        return "m";
    }
    
    if (unit_is_equal(&tSiUnit->mSiUnit_t.mUnit, &Mass)){
        return "g";      
    }

    if (unit_is_equal(&tSiUnit->mSiUnit_t.mUnit, &Time)){
        return "s";
    }

    if (unit_is_equal(&tSiUnit->mSiUnit_t.mUnit, &Angle)){
        return "rad";
    }    

    if (is_si_unit_equal(tSiUnit, &ElectricCurrent)){
        return "A";      
    }

    if (is_si_unit_equal(tSiUnit, &ThermodynamicTemperature)){
        return "K";
    }

    if (is_si_unit_equal(tSiUnit, &LuminousIntensity)){
        return "cd";
    }

    if (is_si_unit_equal(tSiUnit, &AmountOfSubstance)){
        return "mol";
    }

    return "";
}

LIBAPI const char* si_unit_name_basic(const SiUnit* tSiUnit){
    if(tSiUnit == NULL){
        DEBUG_PRINT("error, si_unit_name_basic : tSiUnit is NULL");
        return NULL;
    }

    if (unit_is_equal(&tSiUnit->mSiUnit_t.mUnit, &Length)){
        return "metre";
    }
    
    if (unit_is_equal(&tSiUnit->mSiUnit_t.mUnit, &Mass)){
        return "gram";      
    }

    if (unit_is_equal(&tSiUnit->mSiUnit_t.mUnit, &Time)){
        return "second";
    }

    if (unit_is_equal(&tSiUnit->mSiUnit_t.mUnit, &Angle)){
        return "radian";
    }    

    if (is_si_unit_equal(tSiUnit, &ElectricCurrent)){
        return "ampere";      
    }

    if (is_si_unit_equal(tSiUnit, &ThermodynamicTemperature)){
        return "kelvin";
    }

    if (is_si_unit_equal(tSiUnit, &LuminousIntensity)){
        return "candela";
    }

    if (is_si_unit_equal(tSiUnit, &AmountOfSubstance)){
        return "mole";
    }

    return "";
}

LIBAPI const char* si_unit_quantity_name_basic(const SiUnit* tSiUnit){
    if(tSiUnit == NULL){
        DEBUG_PRINT("error, si_unit_quantity_name_basic : tSiUnit is NULL");
        return NULL;
    }

    if (unit_is_equal(&tSiUnit->mSiUnit_t.mUnit, &Length)){
        return "Length";
    }

    if (unit_is_equal(&tSiUnit->mSiUnit_t.mUnit, &Mass)){
        return "Mass";
    }

    if (unit_is_equal(&tSiUnit->mSiUnit_t.mUnit, &Time)){
        return "Time";
    }

    if (unit_is_equal(&tSiUnit->mSiUnit_t.mUnit, &Angle)){
        return "Angle";
    }        

    if (is_si_unit_equal(tSiUnit, &ElectricCurrent)){
        return "ElectricCurrent";
    }

    if (is_si_unit_equal(tSiUnit, &ThermodynamicTemperature)){
        return "ThermodynamicTemperature";
    }

    if (is_si_unit_equal(tSiUnit, &LuminousIntensity)){
        return "LuminousIntensity";
    }

    if (is_si_unit_equal(tSiUnit, &AmountOfSubstance)){
        return "AmountOfSubstance";
    }

    return "";    
}

LIBAPI const char* si_unit_symbol_non_si(const SiUnit* tSiUnit){
    return "";
}

LIBAPI const char* si_unit_name_non_si(const SiUnit* tSiUnit){
    return "";
}

LIBAPI const char* si_unit_quantity_name_non_si(const SiUnit* tSiUnit){
    return "";
}

// si unit koherent 는 기호가 없다.
LIBAPI const char* si_unit_name_koherent(const SiUnit* tSiUnit){
    return "";
}

LIBAPI const char* si_unit_quantity_name_koherent(const SiUnit* tSiUnit){
    return "";
}

LIBAPI void print_si_unit(const SiUnit* tSelf){
    printf("si_unit { %d, %d, %d, %d, %d, %d, %d, %d }\n",
    tSelf->mSiUnit_t.mUnit.mLength, tSelf->mSiUnit_t.mUnit.mMass,
    tSelf->mSiUnit_t.mUnit.mTime, tSelf->mSiUnit_t.mUnit.mAngle,
    tSelf->mSiUnit_t.mElectricCurrent, 
    tSelf->mSiUnit_t.mThermodynamicTemperature,
    tSelf->mSiUnit_t.mAmountOfSubstance, tSelf->mSiUnit_t.mLuminousIntensity);
}