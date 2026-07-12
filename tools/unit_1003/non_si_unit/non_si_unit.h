/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* non_si_unit
*
* Non-SI unit used together with SI (SI와 같이 쓰는 비SI 단위)
*
* 국 제 단 위 계
* The International
* System of Units
* (SI)
* 제9판 2019
* 한 국 표 준 과 학 연 구 원
33페이지 : 표 8. SI와 함께 사용되는 것이 인정되는 비SI 단위
양              명칭            기호    SI 단위로 나타낸 값
시간            분              min     1 min = 60 s
                시간            h       1 h = 60 min = 3600 s
                일              d       1 d = 24 h = 86 400 s
길이            천문단위(가)    au      1 au = 149 597 870 700 m
평면각 및 위상각 도             °       1° = (π/180) rad
                분              '       1' = (1/60)° = (π/10 800) rad
                초(나)          "       1" = (1/60)' = (π/648 000) rad

면적            헥타르(다)      ha      1 ha = 1 hm^2 = 10^4 m^2
부피            리터)        l, L    1 l = 1 L = 1 dm^3 = 10^3 cm^3 = 10^-3 m^3
질량            톤(마)          t       1 t = 103 kg
                돌턴(바)        Da      1 Da = 1.660 538 86 (28) × 10^-27 kg
에너지          전자볼트(사)    eV      1 eV = 1.602 176 53 (14) × 10^-19 J

로그비 양       네퍼(아)        Np      주석 참조
                벨(아)          B       si 단위에 해당되지 않는다.
                데시벨(아)      dB      하지만, 음압을 표현할 때 자주 사용해서,
                                        압력으로 표현하는 것이 좋겠다.

*
* 아래는 위키페디아이다. 위의 내용과 다른 건 위의 것을 쓰는게 좋을 것 같다.
* 
* Units officially accepted for use with the SI
* Name	            Symbol	Quantity	                    Value in SI units
* --------------------------------------------------------------------------
* minute	        min	    time	                        1 min = 60 s
* hour	            h	                                    1 h = 60 min = 3600 s
* day	            d	                                    1 d = 24 h = 1440 min = 86400 s
* astronomical unit	au	    length	                        1 au = 149597870700 m
* degree	        °	    plane angle and phase angle	    1° = (π/180) rad
* arcminute	        ′	                                    1′ = (1/60)° = (π/10800) rad
* arcsecond	        ″	                                    1″ = (1/60)′ = (1/3600)° = (π/648000) rad
* hectare	        ha	    area	                        1 ha = 1 hm2 = 10000 m2
* litre	            l, L	volume	                        1 L = 1 dm3 = 1000 cm3 = 0.001 m^3
* tonne	            t	    mass	                        1 t = 1 Mg = 1000 kg
* dalton	        Da	                                    1 Da = 1.66053906892(52)×10^-27 kg
* electronvolt	    eV	    energy	                        1 eV = 1.602176634×10^-19 J
* neper	            Np	    logarithmic ratio quantity	    1 Np = 1
* bel, decibel	    B, dB	
* 
***************************************************************************/

#ifndef NON_SI_UNIT_H
#define NON_SI_UNIT_H

#include "non_si_unit/non_si_unit_config.h"
#include "unit.h"

#ifdef __cplusplus
extern "C" {
#endif

extern const Unit Minute;	            
extern const Unit Hour;	                
extern const Unit Day;	                
extern const Unit AstronomicalUnit;    
extern const Unit Degree;	  

extern const Unit Arcminute;	        
extern const Unit Arcsecond;	        
extern const Unit Hectare;	            
extern const Unit Litre;	            
extern const Unit Tonne;	          

extern const Unit Dalton;	            
extern const Unit ElectronVolt;	        
extern const Unit Neper;	            
extern const Unit Bel;	                
extern const Unit DeciBel;	                

LIBAPI void non_si_unit_print_version();
LIBAPI const char* non_si_unit_version(char* buf);

#ifdef __cplusplus
}
#endif

#endif // NON_SI_UNIT_H
