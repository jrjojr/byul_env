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
***************************************************************************/

#include "si_basic_unit.h"

#include <stdio.h>

#include <stdlib.h> // malloc, free
#include "ustring.h"
// mTime;     mLength;     mMass;     mElectricCurrent;     mThermodynamicTemperature;     mAmountOfSubstance;     mLuminousIntensity;
const SiBasicUnit EmptyUnit =                       {0, 0, 0, 0, 0, 0, 0, 0};
const SiBasicUnit Time =                        {1, 0, 0, 0, 0, 0, 0, 0};
const SiBasicUnit Length =                      {0, 1, 0, 0, 0, 0, 0, 0};
const SiBasicUnit Mass =                        {0, 0, 1, 0, 0, 0, 0, 0};
const SiBasicUnit ElectricCurrent =             {0, 0, 0, 1, 0, 0, 0, 0};
const SiBasicUnit ThermodynamicTemperature =    {0, 0, 0, 0, 1, 0, 0, 0};
const SiBasicUnit AmountOfSubstance =           {0, 0, 0, 0, 0, 1, 0, 0};
const SiBasicUnit LuminousIntensity =           {0, 0, 0, 0, 0, 0, 1, 0};

LIBAPI void print_si_basic_unit_version(char* buf){
    printf("%s version : %d.%d.%d.%d\n", "si_basic_unit", 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK);
}

LIBAPI const char* get_si_basic_unit_version(char* buf){
    char buf[16];
    sprintf(buf, "%d.%d.%d.%d", 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK
        );
    return buf;
}

LIBAPI int init_si_basic_unit(SiBasicUnit* tSiBasicUnit){
    // tSiBasicUnit = (SiBasicUnit*) malloc(sizeof(SiBasicUnit));
    if (tSiBasicUnit == NULL){
        DEBUG_PRINT("init_si_basic_unit malloc error.\n");
        return -1;
    }
    assign_si_basic_unit(tSiBasicUnit, &EmptyUnit);
    return 0;
}

LIBAPI int release_si_basic_unit(SiBasicUnit* tSiBasicUnit){
    if(tSiBasicUnit != NULL){
        DEBUG_PRINT("release_si_basic_unit tSiBasicUnit is not NULL.\n");
        // free(tSiBasicUnit);
        return 0;
    }
    else{
        DEBUG_PRINT("error, release_si_basic_unit tSiBasicUnit is NULL.\n");
        return -1;
    }
}

LIBAPI int assign_si_basic_unit(SiBasicUnit* tSiBasicUnit, 
    const SiBasicUnit* tOther){
    if(tSiBasicUnit == NULL){
        DEBUG_PRINT("error, assign_si_basic_unit : tSiBasicUnit is NULL");
        return -1;        
    }

    if( tOther == NULL){
        DEBUG_PRINT("error, assign_si_basic_unit : tOther is NULL");
        return -2;
    }
    tSiBasicUnit->mLength = tOther->mLength;
    tSiBasicUnit->mMass = tOther->mMass;
    tSiBasicUnit->mTime = tOther->mTime;
    tSiBasicUnit->mElectricCurrent = tOther->mElectricCurrent;
    tSiBasicUnit->mThermodynamicTemperature = tOther->mThermodynamicTemperature;
    tSiBasicUnit->mLuminousIntensity = tOther->mLuminousIntensity;
    tSiBasicUnit->mAmountOfSubstance = tOther->mAmountOfSubstance;
    return 0;
}

LIBAPI bool is_si_basic_unit_empty(const SiBasicUnit* tSiBasicUnit)
{
    if(tSiBasicUnit == NULL){
        DEBUG_PRINT("error, is_si_basic_unit_empty : tSiBasicUnit is NULL");
        return false;
    }

    return (tSiBasicUnit->mLength == 0)
        && (tSiBasicUnit->mMass == 0)
        && (tSiBasicUnit->mTime == 0)
        && (tSiBasicUnit->mElectricCurrent == 0)
        && (tSiBasicUnit->mThermodynamicTemperature == 0)
        && (tSiBasicUnit->mLuminousIntensity == 0)
        && (tSiBasicUnit->mAmountOfSubstance == 0);
}

LIBAPI bool is_si_basic_unit_equal(const SiBasicUnit* tSiBasicUnit, 
    const SiBasicUnit* tOther)
{
    if(tSiBasicUnit == NULL){
        DEBUG_PRINT("error, is_si_basic_unit_equal : tSiBasicUnit is NULL");
        return false;
    }

    if( tOther == NULL){
        DEBUG_PRINT("error, is_si_basic_unit_equal : tOther is NULL");
        return false;
    }
        
    return (tSiBasicUnit->mLength == tOther->mLength)
        && (tSiBasicUnit->mMass == tOther->mMass)
        && (tSiBasicUnit->mTime == tOther->mTime)
        && (tSiBasicUnit->mElectricCurrent == tOther->mElectricCurrent)
        && (tSiBasicUnit->mThermodynamicTemperature == tOther->mThermodynamicTemperature)
        && (tSiBasicUnit->mLuminousIntensity == tOther->mLuminousIntensity)
        && (tSiBasicUnit->mAmountOfSubstance == tOther->mAmountOfSubstance);
}

LIBAPI const char* get_si_basic_unit_symbol(const SiBasicUnit* tSiBasicUnit)
{
    // char* aStr;
    if(tSiBasicUnit == NULL){
        DEBUG_PRINT("error, get_si_basic_unit_symbol : tSiBasicUnit is NULL");
        return NULL;
    }

    if (is_si_basic_unit_equal(tSiBasicUnit, &Length)){
        // aStr = "m";
        // aStr = (char*)malloc(strlen(aStr));
        // aStr = "m";
        return "m";
    }
    
    if (is_si_basic_unit_equal(tSiBasicUnit, &Mass)){
        // aStr = (char*)malloc(strlen("g"));
        // aStr = "g";
        // return aStr;  
        return "g";      
    }

    if (is_si_basic_unit_equal(tSiBasicUnit, &Time)){
        // aStr = (char*)malloc(strlen("s"));
        // aStr = "s";
        // return aStr;       
        return "s";
    }

    if (is_si_basic_unit_equal(tSiBasicUnit, &ElectricCurrent)){
        // aStr = (char*)malloc(strlen("A"));
        // aStr = "A";
        // return aStr;  
        return "A";      
    }

    if (is_si_basic_unit_equal(tSiBasicUnit, &ThermodynamicTemperature)){
        // aStr = (char*)malloc(strlen("K"));
        // aStr = "K";
        // return aStr;        
        return "K";
    }

    if (is_si_basic_unit_equal(tSiBasicUnit, &LuminousIntensity)){
        // aStr = (char*)malloc(strlen("cd"));
        // aStr = "cd";
        // return aStr;        
        return "cd";
    }

    if (is_si_basic_unit_equal(tSiBasicUnit, &AmountOfSubstance)){
        // aStr = (char*)malloc(strlen("mol"));
        // aStr = "mol";
        // return aStr;
        return "mol";
    }
    // aStr = (char*)malloc(strlen(""));
    return "";
}

LIBAPI const char* get_si_basic_unit_name(const SiBasicUnit* tSiBasicUnit){
    if(tSiBasicUnit == NULL){
        DEBUG_PRINT("error, get_si_basic_unit_name : tSiBasicUnit is NULL");
        return NULL;
    }

    if (is_si_basic_unit_equal(tSiBasicUnit, &Length)){
        return "metre";
    }
    
    if (is_si_basic_unit_equal(tSiBasicUnit, &Mass)){
        // aStr = (char*)malloc(strlen("g"));
        // aStr = "g";
        // return aStr;  
        return "gram";      
    }

    if (is_si_basic_unit_equal(tSiBasicUnit, &Time)){
        // aStr = (char*)malloc(strlen("s"));
        // aStr = "s";
        // return aStr;       
        return "second";
    }

    if (is_si_basic_unit_equal(tSiBasicUnit, &ElectricCurrent)){
        // aStr = (char*)malloc(strlen("A"));
        // aStr = "A";
        // return aStr;  
        return "ampere";      
    }

    if (is_si_basic_unit_equal(tSiBasicUnit, &ThermodynamicTemperature)){
        // aStr = (char*)malloc(strlen("K"));
        // aStr = "K";
        // return aStr;        
        return "kelvin";
    }

    if (is_si_basic_unit_equal(tSiBasicUnit, &LuminousIntensity)){
        // aStr = (char*)malloc(strlen("cd"));
        // aStr = "cd";
        // return aStr;        
        return "candela";
    }

    if (is_si_basic_unit_equal(tSiBasicUnit, &AmountOfSubstance)){
        // aStr = (char*)malloc(strlen("mol"));
        // aStr = "mol";
        // return aStr;
        return "mole";
    }
    // aStr = (char*)malloc(strlen(""));
    return "";
}

LIBAPI const char* get_si_basic_unit_quantity_name(const SiBasicUnit* tSiBasicUnit)
{
    Ustring aUstring;

    if(tSiBasicUnit == NULL){
        DEBUG_PRINT("error, get_si_basic_unit_name : tSiBasicUnit is NULL");
        return NULL;
    }

    if (is_si_basic_unit_equal(tSiBasicUnit, &Length)){
        // init_ustring(&aUstring, "Length");
        // return aUstring.mStr;
        return "length";
    }

    if (is_si_basic_unit_equal(tSiBasicUnit, &Mass)){
        // init_ustring(&aUstring, "Mass");
        // return aUstring.mStr;
        return "mass";
    }

    if (is_si_basic_unit_equal(tSiBasicUnit, &Time)){
        // init_ustring(&aUstring, "Time");
        // return aUstring.mStr;
        return "time";
    }

    if (is_si_basic_unit_equal(tSiBasicUnit, &ElectricCurrent)){
        // init_ustring(&aUstring, "ElectricCurrent");
        // return aUstring.mStr;
        return "electric current";
    }

    if (is_si_basic_unit_equal(tSiBasicUnit, &ThermodynamicTemperature)){
        // init_ustring(&aUstring, "ThermodynamicTemperature");
        // return aUstring.mStr;
        return "thermodynamic temperature";
    }

    if (is_si_basic_unit_equal(tSiBasicUnit, &LuminousIntensity)){
        // init_ustring(&aUstring, "LuminousIntensity");
        // return aUstring.mStr;
        return "luminous intensity";
    }

    if (is_si_basic_unit_equal(tSiBasicUnit, &AmountOfSubstance)){
        // init_ustring(&aUstring, "AmountOfSubstance");
        // return aUstring.mStr;
        return "amount of substance";
    }

    // return aUstring.mStr;
    return "";
}
