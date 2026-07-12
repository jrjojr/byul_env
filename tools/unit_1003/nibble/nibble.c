/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* nibble 
*
***************************************************************************/

#include "nibble.h"

#include <stdio.h>

LIBAPI void nibble_print_version(){
    printf("%s version : %d.%d.%d.%d\n", "nibble", 
        NIBBLE_VERSION_MAJOR,
        NIBBLE_VERSION_MINOR,
        NIBBLE_VERSION_PATCH,
        NIBBLE_VERSION_TWEAK);
}

LIBAPI const char* nibble_version(char* buf){
    sprintf(buf, "%d.%d.%d.%d", 
        NIBBLE_VERSION_MAJOR,
        NIBBLE_VERSION_MINOR,
        NIBBLE_VERSION_PATCH,
        NIBBLE_VERSION_TWEAK
        );
    return buf;
}

LIBAPI int init_nibble8(Nibble8* tNibble8){
    if (tNibble8 == NULL){
        DEBUG_PRINT("init_nibble8 tNibble8 == NULL error.\n");
        return -1;
    }
    set_nibble8(tNibble8, 0);
    return 0;
}

LIBAPI int release_nibble8(Nibble8* tNibble8){
    if(tNibble8 != NULL){
        DEBUG_PRINT("release_nibble8 tNibble8 is not NULL.\n");
        // free(tNibble8);
        return 0;
    }
    else{
        DEBUG_PRINT("error, release_nibble8 tNibble8 is NULL.\n");
        return -1;
    }
}

LIBAPI int assign_nibble8(Nibble8* tNibble8, const Nibble8* tOther){
    if(tNibble8 == NULL){
        DEBUG_PRINT("error, assign_nibble8 : tNibble8 is NULL");
        return -1;        
    }

    if( tOther == NULL){
        DEBUG_PRINT("error, assign_nibble8 : tOther is NULL");
        return -2;
    }

    tNibble8->mNibble8_t.mNibble0 = tOther->mNibble8_t.mNibble0;
    tNibble8->mNibble8_t.mNibble1 = tOther->mNibble8_t.mNibble1;
    tNibble8->mNibble8_t.mNibble2 = tOther->mNibble8_t.mNibble2;
    tNibble8->mNibble8_t.mNibble3 = tOther->mNibble8_t.mNibble3;
    tNibble8->mNibble8_t.mNibble4 = tOther->mNibble8_t.mNibble4;
    tNibble8->mNibble8_t.mNibble5 = tOther->mNibble8_t.mNibble5;
    tNibble8->mNibble8_t.mNibble6 = tOther->mNibble8_t.mNibble6;
    tNibble8->mNibble8_t.mNibble7 = tOther->mNibble8_t.mNibble7;
    return 0;
}

LIBAPI bool is_nibble8_empty(const Nibble8* tNibble8){
    if(tNibble8 == NULL){
        DEBUG_PRINT("error, is_nibble8_empty : tNibble8 is NULL");
        return false;
    }

    return (tNibble8->mNibble8_t.mNibble0 == 0)
        && (tNibble8->mNibble8_t.mNibble1 == 0)
        && (tNibble8->mNibble8_t.mNibble2 == 0)
        && (tNibble8->mNibble8_t.mNibble3 == 0)
        && (tNibble8->mNibble8_t.mNibble4 == 0)
        && (tNibble8->mNibble8_t.mNibble5 == 0)
        && (tNibble8->mNibble8_t.mNibble6 == 0)
        && (tNibble8->mNibble8_t.mNibble7 == 0);
}

LIBAPI bool is_nibble8_equal(const Nibble8* tNibble8, const Nibble8* tOther){
    if(tNibble8 == NULL){
        DEBUG_PRINT("error, is_nibble8_equal : tNibble8 is NULL");
        return false;
    }

    if( tOther == NULL){
        DEBUG_PRINT("error, is_nibble8_equal : tOther is NULL");
        return false;
    }
        
    return (tNibble8->mNibble8_t.mNibble0 == tOther->mNibble8_t.mNibble0)
        && (tNibble8->mNibble8_t.mNibble1 == tOther->mNibble8_t.mNibble1)
        && (tNibble8->mNibble8_t.mNibble2 == tOther->mNibble8_t.mNibble2)
        && (tNibble8->mNibble8_t.mNibble3 == tOther->mNibble8_t.mNibble3)
        && (tNibble8->mNibble8_t.mNibble4 == tOther->mNibble8_t.mNibble4)
        && (tNibble8->mNibble8_t.mNibble5 == tOther->mNibble8_t.mNibble5)
        && (tNibble8->mNibble8_t.mNibble6 == tOther->mNibble8_t.mNibble6)
        && (tNibble8->mNibble8_t.mNibble7 == tOther->mNibble8_t.mNibble7);        
}

LIBAPI int set_nibble8(Nibble8* tNibble8, const unsigned long mBits){
    tNibble8->mBits = mBits;
    return 0;
}

LIBAPI int set_nibble8_zero(Nibble8* tNibble8, const long tZero){
    tNibble8->mNibble8_t.mNibble0 = tZero;
    return 0;
}

LIBAPI int set_nibble8_one(Nibble8* tNibble8, const long tOne){
    tNibble8->mNibble8_t.mNibble1 = tOne;
    return 0;
}

LIBAPI int set_nibble8_two(Nibble8* tNibble8, const long tTwo){
    tNibble8->mNibble8_t.mNibble2 = tTwo;
    return 0;
}

LIBAPI int set_nibble8_three(Nibble8* tNibble8, const long tThree){
    tNibble8->mNibble8_t.mNibble3 = tThree;
    return 0;
}

LIBAPI int set_nibble8_four(Nibble8* tNibble8, const long tFour){
    tNibble8->mNibble8_t.mNibble4 = tFour;
    return 0;
}

LIBAPI int set_nibble8_five(Nibble8* tNibble8, const long tFive){
    tNibble8->mNibble8_t.mNibble5 = tFive;
    return 0;
}

LIBAPI int set_nibble8_six(Nibble8* tNibble8, const long tSix){
    tNibble8->mNibble8_t.mNibble6 = tSix;
    return 0;
}

LIBAPI int set_nibble8_seven(Nibble8* tNibble8, const long tSeven){
    tNibble8->mNibble8_t.mNibble7 = tSeven;
    return 0;
}

LIBAPI const long  get_nibble8(const Nibble8* tNibble8){
    return tNibble8->mBits;
}

LIBAPI const long get_nibble8_zero(const Nibble8* tNibble8){
    return tNibble8->mNibble8_t.mNibble0;
}

LIBAPI const long get_nibble8_one(const Nibble8* tNibble8){
    return tNibble8->mNibble8_t.mNibble1;
}

LIBAPI const long get_nibble8_two(const Nibble8* tNibble8){
    return tNibble8->mNibble8_t.mNibble2;
}

LIBAPI const long get_nibble8_three(const Nibble8* tNibble8){
    return tNibble8->mNibble8_t.mNibble3;
}

LIBAPI const long get_nibble8_four(const Nibble8* tNibble8){
    return tNibble8->mNibble8_t.mNibble4;
}

LIBAPI const long get_nibble8_five(const Nibble8* tNibble8){
    return tNibble8->mNibble8_t.mNibble5;
}

LIBAPI const long get_nibble8_six(const Nibble8* tNibble8){
    return tNibble8->mNibble8_t.mNibble6;
}

LIBAPI const long get_nibble8_seven(const Nibble8* tNibble8){
    return tNibble8->mNibble8_t.mNibble7;
}
