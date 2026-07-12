/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* si_unit_def 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK
*
***************************************************************************/

#include "si_unit_def.h"

#include <stdio.h>

// mTime;
// mLength;
// mMass;
// mElectricCurrent;
// mThermodynamicTemperature;
// mAmountOfSubstance;
// mLuminousIntensity;
// mDummy;

const SiUnit EmptyUnit                       = {  0,  0,  0,  0,  0,  0,  0,  0 };

// * SI basic unit (SI 기본 단위)
// Symbol   Name(이름)  Quantity(량)                            

// s sec second 
const SiUnit Time                        = {  1,  0,  0,  0,  0,  0,  0,  0 };

// m metre meter
const SiUnit Length                      = {  0,  1,  0,  0,  0,  0,  0,  0 };

// g gram
const SiUnit Mass                        = {  0,  0,  1,  0,  0,  0,  0,  0 };

// A ampere
const SiUnit ElectricCurrent             = {  0,  0,  0,  1,  0,  0,  0,  0 };

// K kelvin
const SiUnit ThermodynamicTemperature    = {  0,  0,  0,  0,  1,  0,  0,  0 };

// mol mole
const SiUnit AmountOfSubstance           = {  0,  0,  0,  0,  0,  1,  0,  0 };

// cd candela
const SiUnit LuminousIntensity           = {  0,  0,  0,  0,  0,  0,  1,  0 };

// radian       rad     angle           m/m	        1
// steradian    sr      solid angle     m^2/m^2     1

// rad radian m/m
const SiUnit Radian                      = {  0,  1,  0,  0,  0,  0,  0,  0 };

// sr steradian m^2/m^2
const SiUnit Steradian                   = {  0,  1,  0,  0,  0,  0,  0,  0 };

// SI 기본 단위로부터 유도된 단위
// 기호     이름        물리량 다른 단위로 표시        SI 기본 단위로 표시

// hertz           Hz      	frequency       	                                            1/s	                        s^-1
// newton          N       	force, weight                                                   kg*m/s2	                    kg*m*s^-2
// pascal          Pa      	pressure, stress                                                N/m^2	                    kg*m^-1*s^-2
// joule           J       	energy, work, heat                                              m*N, C*V, W*s	            kg*m^2*s^-2
// watt            W       	power, radiant flux     	                                    J/s, V*A	                kg*m^2*s^-3
// coulomb         C       	electric charge or quantity of electricity                      s*A, F*V                    s*A
// volt            V       	voltage, electrical potential difference, electromotive force	W/A, J/C	                kg*m^2*s^-3*A^-1
// farad           F       	electrical capacitance	                                        C/V, s/Ω	                kg^-1*m^-2*s^4*A2
// ohm     	       Ω       	electrical resistance, impedance, reactance	                    1/S, V/A	                kg*m^2*s^-3*A^-2
// siemens         S       	electrical conductance	                                        1/Ω, A/V	                kg^-1*m^-2*s^3*A2
// weber           Wb      	magnetic flux	                                                J/A, T*m^2,V*s	            kg*m^2*s^-2*A^-1
// tesla           T       	magnetic induction, magnetic flux density	                    V*s/m^2, Wb/m^2, N/(A*m)    kg*s^-2*A^-1
// henry           H       	electrical inductance	                                        V*s/A, Ω*s, Wb/A	        kg*m^2*s^-2*A^-2
// degree Celsius  °C      	temperature relative to 273.15 K	                            K	                        K
// lumen           lm      	luminous flux	                                                cd*sr	                    cd
// lux     	       lx      	illuminance     	                                            lm/m^2	                    cd*m^-2
// becquerel       Bq      	radioactivity (decays per unit time)	                        1/s	                        s^-1
// gray            Gy      	absorbed dose (of ionizing radiation)	                        J/kg	                    m^2*s^-2
// sievert         Sv      	equivalent dose (of ionizing radiation)	                        J/kg	                    m^2*s^-2
// katal           kat     	catalytic activity	                                            mol/s	                    s^-1*mol. 
const SiUnit Hertz                       = { -1,  0,  0,  0,  0,  0,  0,  0 };  // s^-1
const SiUnit Newton                      = { -2,  1,  1,  0,  0,  0,  0,  0 };  // kg*m*s^-2
const SiUnit Pascal                      = { -2, -1,  1,  0,  0,  0,  0,  0 };  // kg*m^-1*s^-2
const SiUnit Joule                       = { -2,  2,  1,  0,  0,  0,  0,  0 };  // kg*m^2*s^-2
const SiUnit Watt                        = { -3,  2,  1,  0,  0,  0,  0,  0 };  // kg*m^2*s^-3
const SiUnit Coulomb                     = {  1,  0,  0,  1,  0,  0,  0,  0 };  // s*A
const SiUnit Volt                        = { -3,  2,  1, -1,  0,  0,  0,  0 };  // kg*m^2*s^-3*A^-1
const SiUnit Farad                       = {  4, -2, -1,  2,  0,  0,  0,  0 };  // kg^-1*m^-2*s^4*A^2
const SiUnit Ohm                         = { -3,  2,  1, -2,  0,  0,  0,  0 };  // kg*m^2*s^-3*A^-2
const SiUnit Siemens                     = {  3, -2, -1,  2,  0,  0,  0,  0 };  // kg^-1*m^-2*s^3*A^2
const SiUnit Weber                       = { -2,  2,  1, -1,  0,  0,  0,  0 };  // kg*m^2*s^-2*A^-1
const SiUnit Tesla                       = { -2,  0,  1, -1,  0,  0,  0,  0 };  // kg*s^-2*A^-1
const SiUnit Henry                       = { -2,  2,  1, -2,  0,  0,  0,  0 };  // kg*m^2*s^-2*A^-2
const SiUnit DegreeCelsius               = {  0,  0,  0,  0,  1,  0,  0,  0 };  // K	                 
const SiUnit Lumen                       = {  0,  0,  0,  0,  0,  0,  1,  0 };  // cd
const SiUnit Lux                         = {  0, -2,  0,  0,  0,  0,  1,  0 };  // cd*m^-2
const SiUnit Becquerel                   = { -1,  0,  0,  0,  0,  0,  0,  0 };  // s^-1
const SiUnit Gray                        = { -2,  2,  0,  0,  0,  0,  0,  0 };  // m^2*s^-2
const SiUnit Sievert                     = { -2,  2,  0,  0,  0,  0,  0,  0 };  // m^2*s^-2
const SiUnit Katal                       = { -1,  0,  0,  0,  0,  1,  0,  0 };  // s^-1*mol

// Examples of coherent derived units in terms of base units
// Name	Symbol	Derived quantity	Typical symbol
// ----------------------------------------------------------------------------
// SquareMetre	m^2	area	A
// CubicMetre	m^3	volume	V
// MetrePerSecond	m/s	speed, velocity	v
// MetrePerSecondSquared	m/s^2	acceleration	a
// ReciprocalMetre	m^-1	wavenumber	σ, ?
//                           vergence (optics)	V, 1/f 
// KilogramPerCubicMetre	kg/m^3	density	ρ mass concentration	ρ, γ
// KilogramPerSquareMetre	kg/m^2	surface density	ρA
// CubicMetrePerKilogram	m^3/kg	specific volume	v
// AmperePerSquareMetre	A/m^2	current density	j
// AmperePerMetre	A/m	magnetic field strength	H
// MolePerCubicMetre	mol/m^3	concentration	c
// CandelaPerSquareMetre	cd/m^2	luminance	Lv
const SiUnit SquareMetre                 = {  0,  2,  0,  0,  0,  0,  0,  0 };  //	m^2	        area	A
const SiUnit CubicMetre                  = {  0,  3,  0,  0,  0,  0,  0,  0 };  //	m^3	        volume	V
const SiUnit MetrePerSecond              = { -1,  1,  0,  0,  0,  0,  0,  0 };  //	m/s	        speed, velocity	v
const SiUnit MetrePerSecondSquared       = { -2,  1,  0,  0,  0,  0,  0,  0 };  //	m/s^2	    acceleration	a
const SiUnit ReciprocalMetre             = {  0, -1,  0,  0,  0,  0,  0,  0 };  //	m^-1	    wavenumber	σ, ?
const SiUnit KilogramPerCubicMetre       = {  0, -3,  1,  0,  0,  0,  0,  0 };  //	kg/m^3	    density	ρ
const SiUnit KilogramPerSquareMetre      = {  0, -2,  1,  0,  0,  0,  0,  0 };  //	kg/m^2	    surface density	ρA
const SiUnit CubicMetrePerKilogram       = {  0,  3, -1,  0,  0,  0,  0,  0 };  //	m^3/kg	    specific volume	v
const SiUnit AmperePerSquareMetre        = {  0, -2,  0,  1,  0,  0,  0,  0 };  //	A/m^2	    current density	j
const SiUnit AmperePerMetre              = {  0, -1,  0,  1,  0,  0,  0,  0 };  //	A/m	        magnetic field strength	H
const SiUnit MolePerCubicMetre           = {  0, -3,  0,  0,  0,  1,  0,  0 };  //	mol/m^3	    concentration	c
const SiUnit CandelaPerSquareMetre       = {  0, -2,  0,  0,  0,  0,  1,  0 };  //	cd/m^2	    luminance	Lv

// Examples of derived units that include units with special names
// Name	Symbol	Quantity	In SI base units
// ----------------------------------------------------------------------------
// PascalSecond	Pa*s	dynamic viscosity	m^-1*kg*s^-1
// NewtonMetre	N*m	moment of force	m^2*kg*s^-2
// NewtonPerMetre	N/m	surface tension	kg*s^-2
// RadianPerSecond	rad/s	angular velocity, angular frequency	s^-1
// RadianPerSecondSquared	rad/s^2	angular acceleration	s^-2
// WattPerSquareMetre	W/m^2	heat flux density, irradiance	kg*s^-3
// JoulePerKelvin	J/K	entropy, heat capacity	m^2*kg*s^-2*K^-1
// JoulePerKilogramKelvin	J/(kg*K)	specific heat capacity, specific entropy	m^2*s^-2*K^-1
// JoulePerKilogram	J/kg	specific energy	m^2*s^-2
// WattPerMetreKelvin	W/(m*K)	thermal conductivity	m*kg*s^-3*K^-1
// JoulePerCubicMetre	J/m^3	energy density	m^-1*kg*s^-2
// VoltPerMetre	V/m	electric field strength	m*kg*s^-3*A^-1
// CoulombPerCubicMetre	C/m^3	electric charge density	m^-3*s*A
// CoulombPerSquareMetre	C/m^2	surface charge density, electric flux density, electric displacement	m^-2*s*A
// FaradPerMetre	F/m	permittivity	m^-3*kg^-1*s^4*A^2
// HenryPerMetre	H/m	permeability	m*kg*s^-2*A^-2
// JoulePerMole	J/mol	molar energy	m^2*kg*s^-2*mol^-1
// JoulePerMoleKelvin	J/(mol*K)	molar entropy, molar heat capacity	m^2*kg*s^-2-K^-1*mol^-1
// CoulombPerKilogram	C/kg	exposure (x- and γ-rays)	kg^-1*s*A
// GrayPerAecond	        Gy/s	absorbed dose rate	m^2*s^-3
// WattPerSteradian	W/sr	radiant intensity	m^2*kg*s^-3
// WattPerSquareMetreSteradian	W/(m^2*sr)	radiance	kg*s^-3
// KatalPerCubicMetre	kat/m^3	catalytic activity concentration	m^-3*s^-1*mol
const SiUnit PascalSecond                = { -1, -1,  1,  0,  0,  0,  0,  0 };  // 	m^-1*kg*s^-1                Pa*s	    dynamic viscosity
const SiUnit NewtonMetre                 = { -2,  2,  1,  0,  0,  0,  0,  0 };  // 	m^2*kg*s^-2                 N*m	        moment of force
const SiUnit NewtonPerMetre              = { -2,  0,  1,  0,  0,  0,  0,  0 };  // 	kg*s^-2                     N/m	        surface tension
const SiUnit RadianPerSecond             = { -1,  0,  0,  0,  0,  0,  0,  0 };  // 	s^-1                        rad/s	    angular velocity, angular frequency	                                    
const SiUnit RadianPerSecondSquared      = { -2,  0,  0,  0,  0,  0,  0,  0 };  // 	s^-2                        rad/s^2	    angular acceleration	                                                
const SiUnit WattPerSquareMetre          = { -3,  0,  1,  0,  0,  0,  0,  0 };  // 	kg*s^-3                     W/m^2	    heat flux density, irradiance	                                        
const SiUnit JoulePerKelvin              = { -2,  2,  1,  0, -1,  0,  0,  0 };  // 	m^2*kg*s^-2*K^-1            J/K	        entropy, heat capacity	                                                
const SiUnit JoulePerKilogramKelvin      = { -2,  2,  0,  0, -1,  0,  0,  0 };  // 	m^2*s^-2*K^-1               J/(kg*K)    specific heat capacity, specific entropy	                            
const SiUnit JoulePerKilogram            = { -2,  2,  0,  0,  0,  0,  0,  0 };  // 	m^2*s^-2                    J/kg	    specific energy	                                                        
const SiUnit WattPerMetreKelvin          = { -3,  1,  1,  0, -1,  0,  0,  0 };  // 	m*kg*s^-3*K^-1              W/(m*K)	    thermal conductivity	                                                
const SiUnit JoulePerCubicMetre          = { -2, -1,  1,  0,  0,  0,  0,  0 };  // 	m^-1*kg*s^-2                J/m^3	    energy density	                                                        
const SiUnit VoltPerMetre                = { -3,  1,  1, -1,  0,  0,  0,  0 };  // 	m*kg*s^-3*A^-1              V/m	        electric field strength	                                                
const SiUnit CoulombPerCubicMetre        = {  1, -3,  0,  1,  0,  0,  0,  0 };  // 	m^-3*s*A                    C/m^3	    electric charge density	                                                
const SiUnit CoulombPerSquareMetre       = {  1, -2,  0,  1,  0,  0,  0,  0 };  // 	m^-2*s*A                    C/m^2	    surface charge density, electric flux density, electric displacement	
const SiUnit FaradPerMetre               = {  4, -3, -1,  2,  0,  0,  0,  0 };  // 	m^-3*kg^-1*s^4*A^2          F/m	        permittivity	                                                        
const SiUnit HenryPerMetre               = { -2,  1,  1, -2,  0,  0,  0,  0 };  // 	m*kg*s^-2*A^-2              H/m	        permeability	                                                        
const SiUnit JoulePerMole                = { -2,  2,  1,  0,  0, -1,  0,  0 };  // 	m^2*kg*s^-2*mol^-1          J/mol	    molar energy	                                                        
const SiUnit JoulePerMoleKelvin          = { -2,  2,  1,  0, -1, -1,  0,  0 };  // 	m^2*kg*s^-2-K^-1*mol^-1     J/(mol*K)	molar entropy, molar heat capacity	                                    
const SiUnit CoulombPerKilogram          = {  1,  0, -1,  1,  0,  0,  0,  0 };  // 	kg^-1*s*A                   C/kg	    exposure (x- and γ-rays)	                                            
const SiUnit GrayPerAecond               = { -3,  2,  0,  0,  0,  0,  0,  0 };  // 	m^2*s^-3                    Gy/s	    absorbed dose rate	                                                    
const SiUnit WattPerSteradian            = { -3,  2,  1,  0,  0,  0,  0,  0 };  // 	m^2*kg*s^-3                 W/sr	    radiant intensity	                                                    
const SiUnit WattPerSquareMetreSteradian = { -3,  0,  1,  0,  0,  0,  0,  0 };  // 	kg*s^-3                     W/(m^2*sr)	radiance	                                                            
const SiUnit KatalPerCubicMetre          = { -1, -3,  0,  0,  0,  1,  0,  0 };  // 	m^-3*s^-1*mol               kat/m^3	    catalytic activity concentration	                                    

LIBAPI void print_si_unit_def_version(){
    printf("%s version : %d.%d.%d.%d\n", "si_unit_def", 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK);
}

LIBAPI const char* get_si_unit_def_version(){
    char buf[16];
    sprintf(buf, "%d.%d.%d.%d", 
        UNIT_VERSION_MAJOR,
        UNIT_VERSION_MINOR,
        UNIT_VERSION_PATCH,
        UNIT_VERSION_TWEAK
        );
    return buf;
}
