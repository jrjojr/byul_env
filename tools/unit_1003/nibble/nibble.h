/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* nibble
*
***************************************************************************/

#ifndef NIBBLE_H
#define NIBBLE_H

#include "nibble/nibble_config.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NIBBLE_BITS 4

#define NIBBLE_MAX_ABS   7

#define LIMIT(dec)   (dec > NIBBLE_MAX_ABS) ? NIBBLE_MAX_ABS : \
                        ( (dec < -NIBBLE_MAX_ABS) ? -NIBBLE_MAX_ABS : dec )

#define SET_NIBBLE0(dec)    (((LIMIT((dec))) << NIBBLE_BITS * 0 ) & 0x0000000f)
#define SET_NIBBLE1(dec)    (((LIMIT((dec))) << NIBBLE_BITS * 1 ) & 0x000000f0)
#define SET_NIBBLE2(dec)    (((LIMIT((dec))) << NIBBLE_BITS * 2 ) & 0x00000f00)
#define SET_NIBBLE3(dec)    (((LIMIT((dec))) << NIBBLE_BITS * 3 ) & 0x0000f000)
#define SET_NIBBLE4(dec)    (((LIMIT((dec))) << NIBBLE_BITS * 4 ) & 0x000f0000)
#define SET_NIBBLE5(dec)    (((LIMIT((dec))) << NIBBLE_BITS * 5 ) & 0x00f00000)
#define SET_NIBBLE6(dec)    (((LIMIT((dec))) << NIBBLE_BITS * 6 ) & 0x0f000000)
#define SET_NIBBLE7(dec)    (((LIMIT((dec))) << NIBBLE_BITS * 7 ) & 0xf0000000)

#define NIBBLE0(dec)        ( (dec) & 0x0000000f )
#define NIBBLE1(dec)        ( (dec) & 0x000000f0 )
#define NIBBLE2(dec)        ( (dec) & 0x00000f00 )
#define NIBBLE3(dec)        ( (dec) & 0x0000f000 )
#define NIBBLE4(dec)        ( (dec) & 0x000f0000 )
#define NIBBLE5(dec)        ( (dec) & 0x00f00000 )
#define NIBBLE6(dec)        ( (dec) & 0x0f000000 )
#define NIBBLE7(dec)        ( (dec) & 0xf0000000 )

typedef struct s_nibble8{
    signed mNibble0:NIBBLE_BITS;
    signed mNibble1:NIBBLE_BITS;
    signed mNibble2:NIBBLE_BITS;
    signed mNibble3:NIBBLE_BITS;
    signed mNibble4:NIBBLE_BITS;
    signed mNibble5:NIBBLE_BITS;
    signed mNibble6:NIBBLE_BITS;
    signed mNibble7:NIBBLE_BITS;
}nibble8_t;

typedef union u_nibble8{
    unsigned long mBits;
    nibble8_t  mNibble8_t;
}nibble8_n, Nibble8, *Nibble8Ptr;

LIBAPI void nibble_print_version();
LIBAPI const char* nibble_version(char* buf);

LIBAPI int init_nibble8(Nibble8* tNibble8);
LIBAPI int release_nibble8(Nibble8* tNibble8);

LIBAPI int assign_nibble8(Nibble8* tNibble8, const Nibble8* tOther);

LIBAPI bool is_nibble8_empty(const Nibble8* tNibble8);

LIBAPI bool is_nibble8_equal(const Nibble8* tNibble8, const Nibble8* tOther);

LIBAPI int set_nibble8(Nibble8* tNibble8, const unsigned long mBits);

LIBAPI int set_nibble8_zero(Nibble8* tNibble8, const long tZero);

LIBAPI int set_nibble8_one(Nibble8* tNibble8, const long tOne);

LIBAPI int set_nibble8_two(Nibble8* tNibble8, const long tTwo);

LIBAPI int set_nibble8_three(Nibble8* tNibble8, const long tThree);

LIBAPI int set_nibble8_four(Nibble8* tNibble8, const long tFour);

LIBAPI int set_nibble8_five(Nibble8* tNibble8, const long tFive);

LIBAPI int set_nibble8_six(Nibble8* tNibble8, const long tSix);

LIBAPI int set_nibble8_seven(Nibble8* tNibble8, const long tSeven);

LIBAPI const long get_nibble8(const Nibble8* tNibble8);

LIBAPI const long get_nibble8_zero(const Nibble8* tNibble8);

LIBAPI const long get_nibble8_one(const Nibble8* tNibble8);

LIBAPI const long get_nibble8_two(const Nibble8* tNibble8);

LIBAPI const long get_nibble8_three(const Nibble8* tNibble8);

LIBAPI const long get_nibble8_four(const Nibble8* tNibble8);

LIBAPI const long get_nibble8_five(const Nibble8* tNibble8);

LIBAPI const long get_nibble8_six(const Nibble8* tNibble8);

LIBAPI const long get_nibble8_seven(const Nibble8* tNibble8);

#ifdef __cplusplus
}
#endif

#endif // NIBBLE_H
