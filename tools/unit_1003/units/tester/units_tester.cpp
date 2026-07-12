/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* units
*
***************************************************************************/
#define BOOST_TEST_MODULE units tester
#include <boost/test/included/unit_test.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "units.h"
#include "units_tester.hpp"

#include <string>

#include "ustring.h"

namespace ut = boost::unit_test;

BOOST_AUTO_TEST_CASE(units_tester_first, 
    * ut::fixture<Stopwatch>(std::string("units_tester_first"))
    * ut::fixture<Logger>(std::string("units_tester_first"))
    ){

    // 여기부터 실제 테스트할 코드를 작성한다.

    std::cout << "units version is below." << std::endl;
    print_units_version();

    Unit* aSelfUnit = NULL;
    Units* aSelf = NULL;

    aSelfUnit = create_unit_default();
    aSelf = create_units_default();

    delete_unit(aSelfUnit);
    delete_units(aSelf);

    // 여기까지 실제 테스트할 코드를 작성한다.
}

BOOST_AUTO_TEST_CASE(units_tester_symbol, 
    * ut::fixture<Stopwatch>(std::string("units_tester_symbol"))
    * ut::fixture<Logger>(std::string("units_tester_symbol"))
    ){

    // 여기부터 실제 테스트할 코드를 작성한다.

    Units* aSelf = NULL;
    Ustring* aStr = NULL;
    Ustring* aTempStr = NULL;
    std::vector<const char*> aQtys;
    std::vector<UnitPrefixEnum> aPrefixList;
    
aQtys.push_back("EmptyUnit");

aQtys.push_back("Length");
aQtys.push_back("Mass");
aQtys.push_back("Time");
aQtys.push_back("Angle");

aQtys.push_back("SolidAngle");

aQtys.push_back("Frequency");
aQtys.push_back("Force");
aQtys.push_back("Weight");
aQtys.push_back("Pressure");
aQtys.push_back("Stress");

aQtys.push_back("Energy");
aQtys.push_back("Work");
aQtys.push_back("Heat");
aQtys.push_back("Power");
aQtys.push_back("RadiantFlux");
 
aQtys.push_back("Speed");
aQtys.push_back("Velocity");
aQtys.push_back("Acceleration");
aQtys.push_back("Jerk");
aQtys.push_back("Jolt");
aQtys.push_back("Snap");
aQtys.push_back("Jounce");
aQtys.push_back("AngularVelocity");
aQtys.push_back("AngularAcceleration");
aQtys.push_back("VolumetricFlow");
aQtys.push_back("FrequencyDrift");

aQtys.push_back("Area");
aQtys.push_back("Volume");
aQtys.push_back("Momentum");
aQtys.push_back("Impulse");
aQtys.push_back("AngularMomentum");
aQtys.push_back("Torque");
aQtys.push_back("MomentOfForce");
aQtys.push_back("Yank");
aQtys.push_back("WaveNumber");
aQtys.push_back("OpticalPower");
aQtys.push_back("SpatialFrequency");
aQtys.push_back("AreaDensity");
aQtys.push_back("Density");
aQtys.push_back("MassDensity");
aQtys.push_back("SpecificVolume");
aQtys.push_back("Action");
aQtys.push_back("SpecificEnergy");
aQtys.push_back("EnergyDensity");
aQtys.push_back("SurfaceTension");
aQtys.push_back("Stiffness");
aQtys.push_back("HeatFluxDensity");
aQtys.push_back("Irradiance");
aQtys.push_back("KinematicViscosity");
aQtys.push_back("ThermalDiffusivity");
aQtys.push_back("DiffusionCoefficient");
aQtys.push_back("DynamicViscosity");
aQtys.push_back("LinearMassDensity");
aQtys.push_back("MassFlowRate");
aQtys.push_back("Radiance");
aQtys.push_back("SpectralPower");
aQtys.push_back("AbsorbedDoseRate");
aQtys.push_back("FuelEfficiency");
aQtys.push_back("SpectralIrradiance");
aQtys.push_back("PowerDensity");
aQtys.push_back("EnergyFluxDensity");
aQtys.push_back("Compressibility");
aQtys.push_back("RadiantExposure");
aQtys.push_back("MomentOfInertia");
aQtys.push_back("SpecificAngularMomentum");
aQtys.push_back("RadiantIntensity");
aQtys.push_back("SpectralIntensity");

    aPrefixList.push_back( UNIT_PREFIX_QUETTA   );
    aPrefixList.push_back( UNIT_PREFIX_RONNA    );
    aPrefixList.push_back( UNIT_PREFIX_YOTTA    );
    aPrefixList.push_back( UNIT_PREFIX_ZETTA    );
    aPrefixList.push_back( UNIT_PREFIX_EXA      );
    aPrefixList.push_back( UNIT_PREFIX_PETA     );
    aPrefixList.push_back( UNIT_PREFIX_TERA     );
    aPrefixList.push_back( UNIT_PREFIX_GIGA     );
    aPrefixList.push_back( UNIT_PREFIX_MEGA     );
    aPrefixList.push_back( UNIT_PREFIX_KILO     );
    aPrefixList.push_back( UNIT_PREFIX_HECTO    );
    aPrefixList.push_back( UNIT_PREFIX_DECA     );
    aPrefixList.push_back( UNIT_PREFIX_ZERO     );
    aPrefixList.push_back( UNIT_PREFIX_DECI     );
    aPrefixList.push_back( UNIT_PREFIX_CENTI    );
    aPrefixList.push_back( UNIT_PREFIX_MILLI    );
    aPrefixList.push_back( UNIT_PREFIX_MICRO    );
    aPrefixList.push_back( UNIT_PREFIX_NANO     );
    aPrefixList.push_back( UNIT_PREFIX_PICO     );
    aPrefixList.push_back( UNIT_PREFIX_FEMTO    );
    aPrefixList.push_back( UNIT_PREFIX_ATTO     );
    aPrefixList.push_back( UNIT_PREFIX_ZEPTO    );
    aPrefixList.push_back( UNIT_PREFIX_YOCTO    );
    aPrefixList.push_back( UNIT_PREFIX_RONTO    );
    aPrefixList.push_back( UNIT_PREFIX_QUECTO   );

    aSelf = create_units_default();
    aStr = create_ustring_default();
    aTempStr = create_ustring_default();

    // BOOST_TEST_CONTEXT("context aStr : "){
    // BOOST_TEST_INFO(ustring(aStr));
    // BOOST_TEST(false);

    assign_unit(&aSelf->mUnit, &EmptyUnit);
    get_units_symbol(aSelf, aTempStr);
    assign_ustring(aStr, aTempStr);
    BOOST_TEST_INFO("EmptyUnit : " << ustring(aStr));
    BOOST_TEST(false);

    assign_unit(&aSelf->mUnit, &Length);
    get_units_length_symbol(aSelf, aTempStr);
    assign_ustring(aStr, aTempStr);        
    BOOST_TEST_INFO("Length : " << ustring(aStr));
    BOOST_TEST(false);

    // extern const Unit Mass;
    assign_unit(&aSelf->mUnit, &Mass);
    // append_ustring(aStr, units_symbol(aSelf));
    get_units_symbol(aSelf, aTempStr);
    assign_ustring(aStr, aTempStr);        
    BOOST_TEST_INFO("Mass : " << ustring(aStr));
    BOOST_TEST(false);

    // extern const Unit Time;
    assign_unit(&aSelf->mUnit, &Time);
    // append_ustring(aStr, units_symbol(aSelf));
    get_units_symbol(aSelf, aTempStr);
    assign_ustring(aStr, aTempStr);
    BOOST_TEST_INFO("Time : " << ustring(aStr));
    BOOST_TEST(false);

    // extern const Unit Angle;
    assign_unit(&aSelf->mUnit, &Angle);
    // append_ustring(aStr, units_symbol(aSelf));
    get_units_symbol(aSelf, aTempStr);
    assign_ustring(aStr, aTempStr);
    BOOST_TEST_INFO("Angle : " << ustring(aStr));
    BOOST_TEST(false);

    // extern const Unit SolidAngle;
    assign_unit(&aSelf->mUnit, &SolidAngle);
    // append_ustring(aStr, units_symbol(aSelf));
    get_units_symbol(aSelf, aTempStr);
    assign_ustring(aStr, aTempStr);
    BOOST_TEST_INFO("SolidAngle : " << ustring(aStr));
    BOOST_TEST(false);

    // extern const Unit Frequency;
    assign_unit(&aSelf->mUnit, &Frequency);
    // append_ustring(aStr, units_symbol(aSelf));
    get_units_symbol(aSelf, aTempStr);
    assign_ustring(aStr, aTempStr);        
    BOOST_TEST_INFO("Frequency : " << ustring(aStr));
    BOOST_TEST(false);

    // extern const Unit Force;
    assign_unit(&aSelf->mUnit, &Force);
    // append_ustring(aStr, units_symbol(aSelf));
    get_units_symbol(aSelf, aTempStr);
    assign_ustring(aStr, aTempStr);
    BOOST_TEST_INFO("Force : " << ustring(aStr));
    BOOST_TEST(false);

    // extern const Unit Weight;
    assign_unit(&aSelf->mUnit, &Weight);
    // append_ustring(aStr, units_symbol(aSelf));
    get_units_symbol(aSelf, aTempStr);
    assign_ustring(aStr, aTempStr);
    BOOST_TEST_INFO("Weight : " << ustring(aStr));
    BOOST_TEST(false);

    // extern const Unit Pressure;
    assign_unit(&aSelf->mUnit, &Pressure);
    // append_ustring(aStr, units_symbol(aSelf));
    get_units_symbol(aSelf, aTempStr);
    assign_ustring(aStr, aTempStr);
    BOOST_TEST_INFO("Pressure : " << ustring(aStr));
    BOOST_TEST(false);

    // extern const Unit Stress;
    assign_unit(&aSelf->mUnit, &Stress);
    // append_ustring(aStr, units_symbol(aSelf));
    get_units_symbol(aSelf, aTempStr);
    assign_ustring(aStr, aTempStr);
    BOOST_TEST_INFO("Stress : " << ustring(aStr));
    BOOST_TEST(false);

    // extern const Unit Energy;
    assign_unit(&aSelf->mUnit, &Energy);
    // append_ustring(aStr, units_symbol(aSelf));
    get_units_symbol(aSelf, aTempStr);
    assign_ustring(aStr, aTempStr);
    BOOST_TEST_INFO("Energy : " << ustring(aStr));
    BOOST_TEST(false);

    // extern const Unit Work;
    assign_unit(&aSelf->mUnit, &Work);
    get_units_symbol(aSelf, aTempStr);
    assign_ustring(aStr, aTempStr);
    BOOST_TEST_INFO("Work : " << ustring(aStr));
    BOOST_TEST(false);

// extern const Unit Heat;
    assign_unit(&aSelf->mUnit, &Heat);
    get_units_symbol(aSelf, aTempStr);
    assign_ustring(aStr, aTempStr);
    BOOST_TEST_INFO("Heat : " << ustring(aStr));
    BOOST_TEST(false);

// extern const Unit Power;
    assign_unit(&aSelf->mUnit, &Power);
    get_units_symbol(aSelf, aTempStr);
    assign_ustring(aStr, aTempStr);
    BOOST_TEST_INFO("Power : " << ustring(aStr));
    BOOST_TEST(false);

    char buf[64];

// extern const Unit RadiantFlux;
    for(auto it=aPrefixList.begin(); it != aPrefixList.end(); it++){
        BOOST_TEST_INFO("it : " << *it);
        set_units_prefix_length(aSelf, *it);
        set_units_prefix_mass(aSelf, *it);
        set_units_prefix_time(aSelf, *it);
        set_units_prefix_angle(aSelf, *it);

        assign_unit(&aSelf->mUnit, &RadiantFlux);
        get_units_symbol(aSelf, aTempStr);
        assign_ustring(aStr, aTempStr);
        BOOST_TEST_INFO(", RadiantFlux : " << ustring(aStr));        
        BOOST_TEST(false);

        BOOST_TEST_INFO(", RadiantFlux : " << units_symbol(aSelf, buf));        
        BOOST_TEST(false);        

        assign_unit(&aSelf->mUnit, &Angle);
        get_units_symbol(aSelf, aTempStr);
        assign_ustring(aStr, aTempStr);
        BOOST_TEST_INFO("Angle : " << ustring(aStr));
        BOOST_TEST(false);                   
    }
 
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
    for(auto it=aPrefixList.begin(); it != aPrefixList.end(); it++){
        BOOST_TEST_INFO("it : " << *it);
        set_units_prefix_length(aSelf, *it);
        set_units_prefix_mass(aSelf, *it);
        set_units_prefix_time(aSelf, *it);
        set_units_prefix_angle(aSelf, *it);

        assign_unit(&aSelf->mUnit, &EnergyDensity);
        set_ustring(aStr, units_symbol(aSelf, buf));
        BOOST_TEST_INFO(", EnergyDensity : ustring(aStr))" << ustring(aStr));        
        BOOST_TEST_INFO(", EnergyDensity : units_symbol(aSelf, buf))" << units_symbol(aSelf, buf));        

        BOOST_TEST(false);        
    }

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
    delete_ustring(aTempStr);
    delete_ustring(aStr);
    delete_units(aSelf);

    // 여기까지 실제 테스트할 코드를 작성한다.
}
