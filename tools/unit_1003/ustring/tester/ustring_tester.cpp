/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* ustring
*
***************************************************************************/

#define BOOST_TEST_MODULE ustring tester
#include <boost/test/included/unit_test.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <iostream>
#include <string>

#include "ustring.h"
#include "ustring_tester.hpp"

namespace ut = boost::unit_test;

BOOST_AUTO_TEST_CASE(test_ustring_tester){
    boost::posix_time::ptime datetime;
    datetime = boost::posix_time::microsec_clock::local_time();
    std::cout << "Current Time and Date: " << datetime << std::endl;

    std::cout << "test ustring starts.\n";
    
    #ifdef __cplusplus
        print_extern();
    #endif

    Ustring aUstring;
    Ustring aOtherUstring;
    Ustring aSubUstring;

    const char aChar = 'a';
    const char* aStr = 
        " this is a stinr  atest a for a the a ustring and the unit.";
    const char* aOtherStr = "Finds the first ther the occtheurrence of ch";
    const char* aSubStr = "the";

    init_ustring(&aSubUstring);
    set_ustring(&aSubUstring, aSubStr);

    BOOST_TEST(aSubUstring.mStr == aSubStr);
    BOOST_TEST(aSubUstring.mSize == strlen(aSubStr)+1);

    init_ustring(&aUstring);
    BOOST_TEST(aUstring.mStr == "");
    BOOST_TEST(aUstring.mSize == 1);
    
    set_ustring(&aUstring,  aStr);
    BOOST_TEST(aUstring.mStr == aStr);
    BOOST_TEST(aUstring.mSize == strlen(aStr)+1);

    init_ustring(&aOtherUstring);
    set_ustring(&aSubUstring, aOtherStr);
    BOOST_TEST(aOtherUstring.mStr == aOtherStr);
    BOOST_TEST(aOtherUstring.mSize == strlen(aOtherStr)+1);

    DEBUG_PRINT("before assign_ustring(&aUstring, &aOtherUstring) :\n");
    DEBUG_PRINT("   aUstring.mStr : %s\n", aUstring.mStr);
    DEBUG_PRINT("   aOtherUstring.mStr : %s\n", aOtherUstring.mStr);
    assign_ustring(&aUstring, &aOtherUstring);
    DEBUG_PRINT("assign_ustring(&aUstring, &aOtherUstring) :\n");
    DEBUG_PRINT("   aUstring.mStr : %s\n", aUstring.mStr);
    DEBUG_PRINT("   aOtherUstring.mStr : %s\n\n", aOtherUstring.mStr);
    BOOST_TEST(aUstring.mStr == aOtherUstring.mStr);

    DEBUG_PRINT("before add_ustring(&aUstring,  &aOtherUstring) :\n");
    DEBUG_PRINT("   aUstring.mStr : %s\n", aUstring.mStr);
    DEBUG_PRINT("   aOtherUstring.mStr : %s\n", aOtherUstring.mStr);
    add_ustring(&aUstring,  &aOtherUstring);
    DEBUG_PRINT("add_ustring(&aUstring,  &aOtherUstring) :\n");
    DEBUG_PRINT("   aUstring.mStr : %s\n", aUstring.mStr);
    DEBUG_PRINT("   aOtherUstring.mStr : %s\n", aOtherUstring.mStr);    
    BOOST_TEST(aUstring.mStr ==
        (std::string(aOtherStr)+std::string(aOtherStr)).c_str());    

    DEBUG_PRINT("before append_ustring_from(&aUstring,  aStr) :\n");
    DEBUG_PRINT("   aUstring.mStr : %s\n", aUstring.mStr);
    DEBUG_PRINT("   aStr : %s\n", aStr);
    append_ustring_from(&aUstring,  aStr);
    DEBUG_PRINT("append_ustring_from(&aUstring,  aStr) :\n");
    DEBUG_PRINT("   aUstring.mStr : %s\n", aUstring.mStr);
    DEBUG_PRINT("   aStr : %s\n", aStr);
    BOOST_TEST(aUstring.mStr ==
        (std::string(aOtherStr)+std::string(aStr)).c_str());            

    DEBUG_PRINT("before remove_ustring_from_char(&aUstring, aChar) :\n");
    DEBUG_PRINT("   aUstring.mStr : %s\n", aUstring.mStr);
    DEBUG_PRINT("   aChar : %c\n", aChar);

    DEBUG_PRINT("before remove_ustring_from(&aUstring, aSubStr) :\n");
    DEBUG_PRINT("   aUstring.mStr : %s\n", aUstring.mStr);
    DEBUG_PRINT("   aSubStr : %s\n", aSubStr);
    remove_ustring_from(&aUstring, aSubStr);
    DEBUG_PRINT("remove_ustring_from(&aUstring, aSubStr) :\n");
    DEBUG_PRINT("   aUstring.mStr : %s\n", aUstring.mStr);
    DEBUG_PRINT("   aSubStr : %s\n\n", aSubStr);

    DEBUG_PRINT("before remove_ustring_all_from(&aUstring, aSubStr) :\n");
    DEBUG_PRINT("   aUstring.mStr : %s\n", aUstring.mStr);
    DEBUG_PRINT("   aSubStr : %s\n", aSubStr);
    remove_ustring_all_from(&aUstring, aSubStr);
    DEBUG_PRINT("remove_ustring_all_from(&aUstring, aSubStr) :\n");
    DEBUG_PRINT("   aUstring.mStr : %s\n", aUstring.mStr);
    DEBUG_PRINT("   aSubStr : %s\n\n", aSubStr);

    DEBUG_PRINT("before sub_ustring(&aUstring, &aSubUstring) : \n"); 
    DEBUG_PRINT("   aUstring.mStr : %s\n", aUstring.mStr);
    DEBUG_PRINT("   aSubUstring.mStr : %s\n", aSubUstring.mStr);
    sub_ustring(&aUstring, &aSubUstring);
    DEBUG_PRINT("sub_ustring(&aUstring, &aSubUstring) : \n"); 
    DEBUG_PRINT("   aUstring.mStr : %s\n", aUstring.mStr);
    DEBUG_PRINT("   aSubUstring.mStr : %s\n\n", aSubUstring.mStr);

    // BOOST_TEST(aUstring.mStr != aUstring.mStr);

    DEBUG_PRINT("before sub_ustring_all(&aUstring, &aSubUstring) : \n"); 
    DEBUG_PRINT("   aUstring.mStr : %s\n", aUstring.mStr);
    DEBUG_PRINT("   aSubUstring.mStr : %s\n", aSubUstring.mStr);
    sub_ustring_all(&aUstring, &aSubUstring);
    DEBUG_PRINT("sub_ustring_all(&aUstring, &aSubUstring) : \n"); 
    DEBUG_PRINT("   aUstring.mStr : %s\n", aUstring.mStr);
    DEBUG_PRINT("   aSubUstring.mStr : %s\n\n", aSubUstring.mStr);        
    // BOOST_TEST(aUstring.mStr != aUstring.mStr);    

    release_ustring(&aUstring);
    release_ustring(&aOtherUstring);
    release_ustring(&aSubUstring);

}

BOOST_AUTO_TEST_CASE(ustring_tester_create, 
    * ut::fixture<Stopwatch>(std::string("ustring_tester_create"))
    * ut::fixture<Logger>(std::string("ustring_tester_create"))
    ){

    // 여기부터 실제 테스트할 코드를 작성한다.

    Ustring* aSelf = NULL;
    aSelf = create_ustring_default();

    append_ustring_from(aSelf, "EmptyUnit, ");
    // assign_unit(&aSelf->mUnit, &EmptyUnit);
    // aSymbol.append(units_symbol(aSelf));
    BOOST_TEST_INFO("aSelf : " << ustring(aSelf));
    BOOST_TEST(false);
    
    append_ustring_from(aSelf, "Length, ");
    BOOST_TEST_INFO("aSelf : " << ustring(aSelf));
    BOOST_TEST(false);    

// extern const Unit Mass;
// extern const Unit Time;
// extern const Unit Angle;

// extern const Unit SolidAngle;

// extern const Unit Frequency;
// extern const Unit Force;
// extern const Unit Weight;
// extern const Unit Pressure;
// extern const Unit Stress;

// extern const Unit Energy;
// extern const Unit Work;
// extern const Unit Heat;
// extern const Unit Power;
// extern const Unit RadiantFlux;
 
// extern const Unit Speed;
// extern const Unit Velocity;
// extern const Unit Acceleration;
// extern const Unit Jerk;
// extern const Unit Jolt;
// extern const Unit Snap;
// extern const Unit Jounce;
// extern const Unit AngularVelocity;
// extern const Unit AngularAcceleration;
// extern const Unit VolumetricFlow;
// extern const Unit FrequencyDrift;

// extern const Unit Area;
// extern const Unit Volume;
// extern const Unit Momentum;
// extern const Unit Impulse;
// extern const Unit AngularMomentum;
// extern const Unit Torque;
// extern const Unit MomentOfForce;
// extern const Unit Yank;
// extern const Unit WaveNumber;
// extern const Unit OpticalPower;
// extern const Unit SpatialFrequency;
// extern const Unit AreaDensity;
// extern const Unit Density;
// extern const Unit MassDensity;
// extern const Unit SpecificVolume;
// extern const Unit Action;
// extern const Unit SpecificEnergy;
// extern const Unit EnergyDensity;
// extern const Unit SurfaceTension;
// extern const Unit Stiffness;
// extern const Unit HeatFluxDensity;
// extern const Unit Irradiance;
// extern const Unit KinematicViscosity;
// extern const Unit ThermalDiffusivity;
// extern const Unit DiffusionCoefficient;
// extern const Unit DynamicViscosity;
// extern const Unit LinearMassDensity;
// extern const Unit MassFlowRate;
// extern const Unit Radiance;
// extern const Unit SpectralPower;
// extern const Unit AbsorbedDoseRate;
// extern const Unit FuelEfficiency;
// extern const Unit SpectralIrradiance;
// extern const Unit PowerDensity;
// extern const Unit EnergyFluxDensity;
// extern const Unit Compressibility;
// extern const Unit RadiantExposure;
// extern const Unit MomentOfInertia;
// extern const Unit SpecificAngularMomentum;
// extern const Unit RadiantIntensity;
// extern const Unit SpectralIntensity;

    delete_ustring(aSelf);

    // 여기까지 실제 테스트할 코드를 작성한다.
}
