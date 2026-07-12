/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* si_derived_unit 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK
*
***************************************************************************/

#include "si_derived_unit.h"

#include <stdio.h>    
     // mTime;     mLength;     mMass;     mElectricCurrent;     mThermodynamicTemperature;     mAmountOfSubstance;     mLuminousIntensity;

LIBAPI void print_si_derived_unit_version(char* buf){
    printf("%s version : %d.%d.%d.%d\n", "si_derived_unit", 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK);
}

LIBAPI const char* get_si_derived_unit_version(char* buf){
    char buf[16];
    sprintf(buf, "%d.%d.%d.%d", 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK
        );
    return buf;
}

LIBAPI const char* get_si_derived_unit_symbol(const SiBasicUnit* tSiBasicUnit){
    if(tSiBasicUnit == NULL){
        DEBUG_PRINT("error, get_si_derived_unit_symbol : tSiBasicUnit is NULL");
        return NULL;
    }

    // Hertz;           //Hz
    if(is_si_basic_unit_equal(tSiBasicUnit, &Hertz)){
        return "Hz";
    }

    // Radian;          //rad
    if(is_si_basic_unit_equal(tSiBasicUnit, &Radian)){
        return "rad";
    }

    // Steradian;       //sr
    if(is_si_basic_unit_equal(tSiBasicUnit, &Steradian)){
        return "sr";
    }

    // Newton;          //N
    if(is_si_basic_unit_equal(tSiBasicUnit, &Newton)){
        return "N";
    }

    // Pascal;          //Pa
    if(is_si_basic_unit_equal(tSiBasicUnit, &Pascal)){
        return "Pa";
    }    

    // Joule;           //J
    if(is_si_basic_unit_equal(tSiBasicUnit, &Joule)){
        return "J";
    }

    // Watt;            //W
    if(is_si_basic_unit_equal(tSiBasicUnit, &Watt)){
        return "W";
    }

    // Coulomb;         //C
    if(is_si_basic_unit_equal(tSiBasicUnit, &Coulomb)){
        return "C";
    }
    
    // Volt;            //V
    if(is_si_basic_unit_equal(tSiBasicUnit, &Volt)){
        return "V";
    }
    
    // Farad;           //F
    if(is_si_basic_unit_equal(tSiBasicUnit, &Farad)){
        return "F";
    }
    
    // Ohm;     	    //��
    if(is_si_basic_unit_equal(tSiBasicUnit, &Ohm)){
        return "��";
    }
    
    // Siemens;         //S
    if(is_si_basic_unit_equal(tSiBasicUnit, &Siemens)){
        return "S";
    }
    
    // Weber;           //Wb
    if(is_si_basic_unit_equal(tSiBasicUnit, &Weber)){
        return "Wb";
    }

    // Tesla;           //T
    if(is_si_basic_unit_equal(tSiBasicUnit, &Tesla)){
        return "T";
    }

    // Henry;           //H
    if(is_si_basic_unit_equal(tSiBasicUnit, &Henry)){
        return "H";
    }

    // DegreeCelsius;   //��C
    if(is_si_basic_unit_equal(tSiBasicUnit, &DegreeCelsius)){
        return "��C";
    }

    // Lumen;           //lm
    if(is_si_basic_unit_equal(tSiBasicUnit, &Lumen)){
        return "lm";
    }

    // Lux;     	    //lx
    if(is_si_basic_unit_equal(tSiBasicUnit, &Lux)){
        return "lx";
    }

    // Becquerel;       //Bq
    if(is_si_basic_unit_equal(tSiBasicUnit, &Becquerel)){
        return "Bq";
    }

    // Gray;            //Gy
    if(is_si_basic_unit_equal(tSiBasicUnit, &Gray)){
        return "Gy";
    }

    // Sievert;         //Sv
    if(is_si_basic_unit_equal(tSiBasicUnit, &Sievert)){
        return "Sv";
    }

    // Katal;           //kat
    if(is_si_basic_unit_equal(tSiBasicUnit, &Katal)){
        return "kat";
    }

    return "";
}

LIBAPI const char* get_si_derived_unit_name(const SiBasicUnit* tSiBasicUnit){
    if(tSiBasicUnit == NULL){
        DEBUG_PRINT("error, get_si_derived_unit_name : tSiBasicUnit is NULL");
        return NULL;
    }

    // Hertz;           //Hz
    if(is_si_basic_unit_equal(tSiBasicUnit, &Hertz)){
        return "Hertz";
    }

    // Radian;          //rad
    if(is_si_basic_unit_equal(tSiBasicUnit, &Radian)){
        return "Radian";
    }

    // Steradian;       //sr
    if(is_si_basic_unit_equal(tSiBasicUnit, &Steradian)){
        return "Steradian";
    }

    // Newton;          //N
    if(is_si_basic_unit_equal(tSiBasicUnit, &Newton)){
        return "Newton";
    }

    // Pascal;          //Pa
    if(is_si_basic_unit_equal(tSiBasicUnit, &Pascal)){
        return "Pascal";
    }    

    // Joule;           //J
    if(is_si_basic_unit_equal(tSiBasicUnit, &Joule)){
        return "Joule";
    }

    // Watt;            //W
    if(is_si_basic_unit_equal(tSiBasicUnit, &Watt)){
        return "Watt";
    }

    // Coulomb;         //C
    if(is_si_basic_unit_equal(tSiBasicUnit, &Coulomb)){
        return "Coulomb";
    }
    
    // Volt;            //V
    if(is_si_basic_unit_equal(tSiBasicUnit, &Volt)){
        return "V";
    }
    
    // Farad;           //F
    if(is_si_basic_unit_equal(tSiBasicUnit, &Farad)){
        return "Volt";
    }
    
    // Ohm;     	    //��
    if(is_si_basic_unit_equal(tSiBasicUnit, &Ohm)){
        return "Ohm";
    }
    
    // Siemens;         //S
    if(is_si_basic_unit_equal(tSiBasicUnit, &Siemens)){
        return "Siemens";
    }
    
    // Weber;           //Wb
    if(is_si_basic_unit_equal(tSiBasicUnit, &Weber)){
        return "Weber";
    }

    // Tesla;           //T
    if(is_si_basic_unit_equal(tSiBasicUnit, &Tesla)){
        return "Tesla";
    }

    // Henry;           //H
    if(is_si_basic_unit_equal(tSiBasicUnit, &Henry)){
        return "Henry";
    }

    // DegreeCelsius;   //��C
    if(is_si_basic_unit_equal(tSiBasicUnit, &DegreeCelsius)){
        return "DegreeCelsius";
    }

    // Lumen;           //lm
    if(is_si_basic_unit_equal(tSiBasicUnit, &Lumen)){
        return "Lumen";
    }

    // Lux;     	    //lx
    if(is_si_basic_unit_equal(tSiBasicUnit, &Lux)){
        return "Lux";
    }

    // Becquerel;       //Bq
    if(is_si_basic_unit_equal(tSiBasicUnit, &Becquerel)){
        return "Becquerel";
    }

    // Gray;            //Gy
    if(is_si_basic_unit_equal(tSiBasicUnit, &Gray)){
        return "Gray";
    }

    // Sievert;         //Sv
    if(is_si_basic_unit_equal(tSiBasicUnit, &Sievert)){
        return "Sievert";
    }

    // Katal;           //kat
    if(is_si_basic_unit_equal(tSiBasicUnit, &Katal)){
        return "Katal";
    }

    return "";
}

LIBAPI const char* get_si_derived_unit_quantity_name(const SiBasicUnit* tSiBasicUnit){
    if(tSiBasicUnit == NULL){
        DEBUG_PRINT("error, get_si_derived_unit_name : tSiBasicUnit is NULL");
        return NULL;
    }

    // Hertz;           //Hz
    // Hertz;           //Hz      	frequency       	                                            
    if(is_si_basic_unit_equal(tSiBasicUnit, &Hertz)){
        return "frequency";
    }

    // Radian;          //rad
    // Radian;          //rad     	angle       	                                                
    if(is_si_basic_unit_equal(tSiBasicUnit, &Radian)){
        return "angle";
    }

    // Steradian;       //sr
    // Steradian;       //sr      	solid angle                                                     
    if(is_si_basic_unit_equal(tSiBasicUnit, &Steradian)){
        return "solid angle";
    }

    // Newton;          //N
    // Newton;          //N       	force, weight                                                   
    if(is_si_basic_unit_equal(tSiBasicUnit, &Newton)){
        return "force";
    }

    // Pascal;          //Pa
    // Pascal;          //Pa      	pressure, stress                                                
    if(is_si_basic_unit_equal(tSiBasicUnit, &Pascal)){
        return "pressure";
    }    

    // Joule;           //J
    // Joule;           //J       	energy, work, heat                                              
    if(is_si_basic_unit_equal(tSiBasicUnit, &Joule)){
        return "work";
    }

    // Watt;            //W
    // Watt;            //W       	power, radiant flux     	                                    
    if(is_si_basic_unit_equal(tSiBasicUnit, &Watt)){
        return "power";
    }

    // Coulomb;         //C
    // Coulomb;         //C       	electric charge or quantity of electricity                      
    if(is_si_basic_unit_equal(tSiBasicUnit, &Coulomb)){
        return "electric charge";
    }
    
    // Volt;            //V
    // Volt;            //V       	voltage, electrical potential difference, electromotive force	
    if(is_si_basic_unit_equal(tSiBasicUnit, &Volt)){
        return "voltage";
    }
    
    // Farad;           //F
    // Farad;           //F       	electrical capacitance	                                        
    if(is_si_basic_unit_equal(tSiBasicUnit, &Farad)){
        return "electrical capacitance";
    }
    
    // Ohm;     	    //��
    // Ohm;     	    //��       	electrical resistance, impedance, reactance	                    
    if(is_si_basic_unit_equal(tSiBasicUnit, &Ohm)){
        return "electrical resistance";
    }
    
    // Siemens;         //S
    // Siemens;         //S       	electrical conductance	                                        
    if(is_si_basic_unit_equal(tSiBasicUnit, &Siemens)){
        return "electrical conductance";
    }
    
    // Weber;           //Wb
    // Weber;           //Wb      	magnetic flux	                                                    
    if(is_si_basic_unit_equal(tSiBasicUnit, &Weber)){
        return "magnetic flux";
    }

    // Tesla;           //T
    // Tesla;           //T       	magnetic induction, magnetic flux density	                    
    if(is_si_basic_unit_equal(tSiBasicUnit, &Tesla)){
        return "magnetic induction";
    }

    // Henry;           //H
    // Henry;           //H       	electrical inductance	                                        
    if(is_si_basic_unit_equal(tSiBasicUnit, &Henry)){
        return "electrical inductance";
    }

    // DegreeCelsius;   //��C
    // DegreeCelsius;   //��C      	temperature relative to 273.15 K	                            
    if(is_si_basic_unit_equal(tSiBasicUnit, &DegreeCelsius)){
        return "temperature relative to 273.15 K";
    }

    // Lumen;           //lm
    // Lumen;           //lm      	luminous flux	                                                
    if(is_si_basic_unit_equal(tSiBasicUnit, &Lumen)){
        return "luminous flux";
    }

    // Lux;     	    //lx
    // Lux;     	    //lx      	illuminance     	                                            
    if(is_si_basic_unit_equal(tSiBasicUnit, &Lux)){
        return "illuminance";
    }

    // Becquerel;       //Bq
    // Becquerel;       //Bq      	radioactivity (decays per unit time)	                        
    if(is_si_basic_unit_equal(tSiBasicUnit, &Becquerel)){
        return "radioactivity";
    }

    // Gray;            //Gy
    // Gray;            //Gy      	absorbed dose (of ionizing radiation)	                        
    if(is_si_basic_unit_equal(tSiBasicUnit, &Gray)){
        return "absorbed dose";
    }

    // Sievert;         //Sv
    // Sievert;         //Sv      	equivalent dose (of ionizing radiation)	                        
    if(is_si_basic_unit_equal(tSiBasicUnit, &Sievert)){
        return "equivalent dose";
    }

    // Katal;           //kat
    // Katal;           //kat     	catalytic activity	                      
    if(is_si_basic_unit_equal(tSiBasicUnit, &Katal)){
        return "catalytic activity";
    }

    return "";
}