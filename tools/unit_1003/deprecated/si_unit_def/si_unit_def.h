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
* 이 lib는 si unit 에서 사용되는 물리량 이름, 단위량 이름, 단위 기호를 정의한다.
* si unit에 필요한 상수들도 정의한다.
* non si unit는 부가적으로 정의한다.
* koherent si unit는 단위 기호가 없다. 
*
* 아스키로 이름들을 정의한다. 기본값이다.
* euc-kr과 utf8에서 사용되는 이름들도 정의한다.
* locale에 따라서, 해당되는 lib를 로드한다.
*
* QuantityName      Name    Symbol
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
* SI 기본 단위로부터 유도된 단위
* 이름      	기호	물리량	                        다른 단위로 표시	            SI 기본 단위로 표시
* ------------------------------------------------------------------------------------------------------
* 라디안        rad	    평면각	                        m*m^-1	                        무차원
* 스테라디안    sr	    입체각	                        m^2*m^-2	                    무차원
* -----------------------------------------------------------------------------------------------------
*
* 헤르츠        Hz	    진동수	                        1/s	                            s^-1
* 뉴턴      	N	    힘, 무게	                    kg*m/s2	                        kg*m*s^-2
* 파스칼        Pa	    압력, 응력	                    N/m^2	                        m^-1*kg*s^-2
* 줄        	J	    에너지, 일, 열량	            N*m = C*V = W*s = VA?s	        m^2*kg*s^-2
* 와트      	W	    일률, 전력, 방사속	            J/s = V*A	                    m^2*kg*s^-3
* 쿨롱      	C	    전하 또는 전하량	            A*s	                            A*s
* 볼트      	V	    전압, 전위, 기전력	            W/A = J/C	                    m^2*kg*s^-3*A^-1
* 패럿      	F	    전기 용량	                    C/V	                            m^-2*kg^-1*s^4*A2
* 옴        	Ω	    전기 저항, 임피던스, 리액턴스   V/A	                            m^2*kg*s^-3*A^-2
* 지멘스        S	    전도율	                        1/Ω = Ω-1	                    m^-2*kg^-1*s^3*A2
* 웨버      	Wb	    자기 선속 (자기력 선속)	        J/A = J?A^-1 = V?s = N?m?A-1	    m^2*kg*s^-2*A^-1
* 테슬라        T	    자기장세기, 자기력선속밀도	     V*s/m^2 = Wb/m^2 = N/(A?m)	    kg*s^-2*A^-1
* 헨리      	H	    인덕턴스	                    V*s/A = Wb/A	                m^2*kg*s^-2*A^-2
* 섭씨      	°C	    섭씨 온도	                    K ^- 273.15	                    K ^- 273.15
* 루멘      	lm	    광선속	                        lx*m^2	                        cd*sr
* 럭스      	lx	    조도	                        lm/m^2	                        m^-2*cd*sr
* 베크렐        Bq	    방사능	                        1/s	                            s^-1
* 그레이        Gy	    흡수선량	                    J/kg	                        m^2*s^-2
* 시버트        Sv	    등가선량	                    J/kg	                        m^2*s^-2
* 캐탈      	kat	    촉매 활성도	                    mol/s	                        s^-1*mol
* 
* Named units derived from SI base units
* ------------------------------------------------------------------------------------------------------------------------------------------
* Name	        Symbol	    Quantity	                                                    Equivalents                 SI base unit Equivalents
* ------------------------------------------------------------------------------------------------------------------    ------------------------
* hertz           Hz      	frequency       	                                            1/s	                        s^-1
* radian          rad     	angle       	                                                m/m	                        1
* steradian       sr      	solid angle                                                     m^2/m^2	                    1
* newton          N       	force, weight                                                   kg*m/s2	                    kg*m*s^-2
* pascal          Pa      	pressure, stress                                                N/m^2	                    kg*m^-1*s^-2
* joule           J       	energy, work, heat                                              m*N, C*V, W*s	            kg*m^2*s^-2
* watt            W       	power, radiant flux     	                                    J/s, V*A	                kg*m^2*s^-3
* coulomb         C       	electric charge or quantity of electricity                      s*A, F*V                    s*A
* volt            V       	voltage, electrical potential difference, electromotive force	W/A, J/C	                kg*m^2*s^-3*A^-1
* farad           F       	electrical capacitance	                                        C/V, s/Ω	                kg^-1*m^-2*s^4*A2
* ohm     	      Ω       	electrical resistance, impedance, reactance	                    1/S, V/A	                kg*m^2*s^-3*A^-2
* siemens         S       	electrical conductance	                                        1/Ω, A/V	                kg^-1*m^-2*s^3*A2
* weber           Wb      	magnetic flux	                                                J/A, T*m^2,V*s	            kg*m^2*s^-2*A^-1
* tesla           T       	magnetic induction, magnetic flux density	                    V*s/m^2, Wb/m^2, N/(A*m)    kg*s^-2*A^-1
* henry           H       	electrical inductance	                                        V*s/A, Ω*s, Wb/A	        kg*m^2*s^-2*A^-2
* degree Celsius  °C      	temperature relative to 273.15 K	                            K	                        K
* lumen           lm      	luminous flux	                                                cd*sr	                    cd
* lux     	      lx      	illuminance     	                                            lm/m^2	                    cd*m^-2
* becquerel       Bq      	radioactivity (decays per unit time)	                        1/s	                        s^-1
* gray            Gy      	absorbed dose (of ionizing radiation)	                        J/kg	                    m^2*s^-2
* sievert         Sv      	equivalent dose (of ionizing radiation)	                        J/kg	                    m^2*s^-2
* katal           kat     	catalytic activity	                                            mol/s	                    s^-1*mol.
* 
* Examples of coherent derived units in terms of base units
* Name	Symbol	Derived quantity	Typical symbol
* ----------------------------------------------------------------------------
* SquareMetre	m^2	area	A
* CubicMetre	m^3	volume	V
* MetrePerSecond	m/s	speed, velocity	v
* MetrePerSecondSquared	m/s^2	acceleration	a
* ReciprocalMetre	m^-1	wavenumber	σ, ?
*                           vergence (optics)	V, 1/f 
* KilogramPerCubicMetre	kg/m^3	density	ρ mass concentration	ρ, γ
* KilogramPerSquareMetre	kg/m^2	surface density	ρA
* CubicMetrePerKilogram	m^3/kg	specific volume	v
* AmperePerSquareMetre	A/m^2	current density	j
* AmperePerMetre	A/m	magnetic field strength	H
* MolePerCubicMetre	mol/m^3	concentration	c
* CandelaPerSquareMetre	cd/m^2	luminance	Lv
*
* Examples of derived units that include units with special names
* Name	Symbol	Quantity	In SI base units
* ----------------------------------------------------------------------------
* PascalSecond	Pa*s	dynamic viscosity	m^-1*kg*s^-1
* NewtonMetre	N*m	moment of force	m^2*kg*s^-2
* NewtonPerMetre	N/m	surface tension	kg*s^-2
* RadianPerSecond	rad/s	angular velocity, angular frequency	s^-1
* RadianPerSecondSquared	rad/s^2	angular acceleration	s^-2
* WattPerSquareMetre	W/m^2	heat flux density, irradiance	kg*s^-3
* JoulePerKelvin	J/K	entropy, heat capacity	m^2*kg*s^-2*K^-1
* JoulePerKilogramKelvin	J/(kg*K)	specific heat capacity, specific entropy	m^2*s^-2*K^-1
* JoulePerKilogram	J/kg	specific energy	m^2*s^-2
* WattPerMetreKelvin	W/(m*K)	thermal conductivity	m*kg*s^-3*K^-1
* JoulePerCubicMetre	J/m^3	energy density	m^-1*kg*s^-2
* VoltPerMetre	V/m	electric field strength	m*kg*s^-3*A^-1
* CoulombPerCubicMetre	C/m^3	electric charge density	m^-3*s*A
* CoulombPerSquareMetre	C/m^2	surface charge density, electric flux density, electric displacement	m^-2*s*A
* FaradPerMetre	F/m	permittivity	m^-3*kg^-1*s^4*A^2
* HenryPerMetre	H/m	permeability	m*kg*s^-2*A^-2
* JoulePerMole	J/mol	molar energy	m^2*kg*s^-2*mol^-1
* JoulePerMoleKelvin	J/(mol*K)	molar entropy, molar heat capacity	m^2*kg*s^-2-K^-1*mol^-1
* CoulombPerKilogram	C/kg	exposure (x- and γ-rays)	kg^-1*s*A
* GrayPerAecond	        Gy/s	absorbed dose rate	m^2*s^-3
* WattPerSteradian	W/sr	radiant intensity	m^2*kg*s^-3
* WattPerSquareMetreSteradian	W/(m^2*sr)	radiance	kg*s^-3
* KatalPerCubicMetre	kat/m^3	catalytic activity concentration	m^-3*s^-1*mol
* 
***************************************************************************/

#ifndef SI_UNIT_DEF_H
#define SI_UNIT_DEF_H

#include "unit/unit_config.h"
#include "si_unit.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum e_si_unit_type{
    BASIC_UNIT,
    DERIVED_UNIT,
    NON_SI_UNIT,
    KOHERENT_UNIT
}si_unit_type_m;

typedef si_unit_type_m SiUnitType;

typedef struct s_si_unit_def{
    SiUnitType mUnitType;
    char* mQuantityName;
    char* mUnitName;
    char* mUnitSymbol;
    char* mUnitSybolAliasList[];
}si_unit_def_t;

typedef si_unit_def_t SiUnitDef;

extern const SiUnit EmptyUnit;

// si basic unit
extern const SiUnit Length;
extern const SiUnit Mass;
extern const SiUnit Time;
extern const SiUnit ElectricCurrent;
extern const SiUnit ThermodynamicTemperature;
extern const SiUnit AmountOfSubstance;
extern const SiUnit LuminousIntensity;

// si derived unit
extern const SiUnit Radian;          //rad     	angle       	                                                m/m	                        1
extern const SiUnit Steradian;       //sr      	solid angle                                                     m^2/m^2	                    1

extern const SiUnit Hertz;           //Hz      	frequency       	                                            1/s	                        s^-1
extern const SiUnit Newton;          //N       	force, weight                                                   kg*m/s2	                    kg*m*s^-2
extern const SiUnit Pascal;          //Pa      	pressure, stress                                                N/m^2	                    kg*m^-1*s^-2
extern const SiUnit Joule;           //J       	energy, work, heat                                              m*N, C*V, W*s	            kg*m^2*s^-2
extern const SiUnit Watt;            //W       	power, radiant flux     	                                    J/s, V*A	                kg*m^2*s^-3

extern const SiUnit Coulomb;         //C       	electric charge or quantity of electricity                      s*A, F*V                    s*A
extern const SiUnit Volt;            //V       	voltage, electrical potential difference, electromotive force	W/A, J/C	                kg*m^2*s^-3*A^-1
extern const SiUnit Farad;           //F       	electrical capacitance	                                        C/V, s/Ω	                kg^-1*m^-2*s^4*A2
extern const SiUnit Ohm;     	    //Ω       	electrical resistance, impedance, reactance	                    1/S, V/A	                kg*m^2*s^-3*A^-2
extern const SiUnit Siemens;         //S       	electrical conductance	                                        1/Ω, A/V	                kg^-1*m^-2*s^3*A2

extern const SiUnit Weber;           //Wb      	magnetic flux	                                                J/A, T*m^2,V*s	            kg*m^2*s^-2*A^-1
extern const SiUnit Tesla;           //T       	magnetic induction, magnetic flux density	                    V*s/m^2, Wb/m^2, N/(A*m)    kg*s^-2*A^-1
extern const SiUnit Henry;           //H       	electrical inductance	                                        V*s/A, Ω*s, Wb/A	        kg*m^2*s^-2*A^-2
extern const SiUnit DegreeCelsius;        // degree; //Celsius  °C      	temperature relative to 273.15 K	                            K	                        K
extern const SiUnit Lumen;           //lm      	luminous flux	                                                cd*sr	                    cd

extern const SiUnit Lux;     	    //lx      	illuminance     	                                            lm/m^2	                    cd*m^-2
extern const SiUnit Becquerel;       //Bq      	radioactivity (decays per unit time)	                        1/s	                        s^-1
extern const SiUnit Gray;            //Gy      	absorbed dose (of ionizing radiation)	                        J/kg	                    m^2*s^-2
extern const SiUnit Sievert;         //Sv      	equivalent dose (of ionizing radiation)	                        J/kg	                    m^2*s^-2
extern const SiUnit Katal;           //kat     	catalytic activity	                                            mol/s	                    s^-1*mol.

// koherent si unit 은 기호가 없다.
// 단지, basic unit 와 derived unit 의 기호를 조합해서 사용한다.
// koherent si unit
extern const SiUnit SquareMetre; //	m^2	area	A
extern const SiUnit CubicMetre; //	m^3	volume	V
extern const SiUnit MetrePerSecond; //	m/s	speed, velocity	v
extern const SiUnit MetrePerSecondSquared; //	m/s^2	acceleration	a
extern const SiUnit ReciprocalMetre; //	m^-1	wavenumber	σ, ? 
extern const SiUnit KilogramPerCubicMetre; //	kg/m^3	density	ρ //	kg/m^3	mass concentration	ρ, γ
extern const SiUnit KilogramPerSquareMetre; //	kg/m^2	surface density	ρA
extern const SiUnit CubicMetrePerKilogram; //	m^3/kg	specific volume	v
extern const SiUnit AmperePerSquareMetre; //	A/m^2	current density	j
extern const SiUnit AmperePerMetre; //	A/m	magnetic field strength	H
extern const SiUnit MolePerCubicMetre; //	mol/m^3	concentration	c
extern const SiUnit CandelaPerSquareMetre; //	cd/m^2	luminance	Lv

// koherent si unit with derived si unit
extern const SiUnit PascalSecond;  // 	Pa*s	dynamic viscosity	m^-1*kg*s^-1
extern const SiUnit NewtonMetre;  // 	N*m	moment of force	m^2*kg*s^-2
extern const SiUnit NewtonPerMetre;  // 	N/m	surface tension	kg*s^-2
extern const SiUnit RadianPerSecond;  // 	rad/s	angular velocity, angular frequency	s^-1
extern const SiUnit RadianPerSecondSquared;  // 	rad/s^2	angular acceleration	s^-2
extern const SiUnit WattPerSquareMetre;  // 	W/m^2	heat flux density, irradiance	kg*s^-3
extern const SiUnit JoulePerKelvin;  // 	J/K	entropy, heat capacity	m^2*kg*s^-2*K^-1
extern const SiUnit JoulePerKilogramKelvin;  // 	J/(kg*K)	specific heat capacity, specific entropy	m^2*s^-2*K^-1
extern const SiUnit JoulePerKilogram;  // 	J/kg	specific energy	m^2*s^-2
extern const SiUnit WattPerMetreKelvin;  // 	W/(m*K)	thermal conductivity	m*kg*s^-3*K^-1
extern const SiUnit JoulePerCubicMetre;  // 	J/m^3	energy density	m^-1*kg*s^-2
extern const SiUnit VoltPerMetre;  // 	V/m	electric field strength	m*kg*s^-3*A^-1
extern const SiUnit CoulombPerCubicMetre;  // 	C/m^3	electric charge density	m^-3*s*A
extern const SiUnit CoulombPerSquareMetre;  // 	C/m^2	surface charge density, electric flux density, electric displacement	m^-2*s*A
extern const SiUnit FaradPerMetre;  // 	F/m	permittivity	m^-3*kg^-1*s^4*A^2
extern const SiUnit HenryPerMetre;  // 	H/m	permeability	m*kg*s^-2*A^-2
extern const SiUnit JoulePerMole;  // 	J/mol	molar energy	m^2*kg*s^-2*mol^-1
extern const SiUnit JoulePerMoleKelvin;  // 	J/(mol*K)	molar entropy, molar heat capacity	m^2*kg*s^-2-K^-1*mol^-1
extern const SiUnit CoulombPerKilogram;  // 	C/kg	exposure (x- and γ-rays)	kg^-1*s*A
extern const SiUnit GrayPerAecond;  // 	        Gy/s	absorbed dose rate	m^2*s^-3
extern const SiUnit WattPerSteradian;  // 	W/sr	radiant intensity	m^2*kg*s^-3
extern const SiUnit WattPerSquareMetreSteradian;  // 	W/(m^2*sr)	radiance	kg*s^-3
extern const SiUnit KatalPerCubicMetre;  // 	kat/m^3	catalytic activity concentration	m^-3*s^-1*mol

LIBAPI void print_si_unit_def_version();
LIBAPI const char* get_si_unit_def_version();

#ifdef __cplusplus
}
#endif

#endif // SI_UNIT_DEF_H
