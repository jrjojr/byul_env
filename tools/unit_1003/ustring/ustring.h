/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* ustring 
*
***************************************************************************/

#ifndef USTRING_H
#define USTRING_H

#include "ustring/ustring_config.h"

#ifdef __cplusplus
extern "C" {
        void print_extern(){
        DEBUG_PRINT("this is extern print \n");
        printf("defined __cplusplus\n");
    }
#endif

#define USTRING_EMPTY_SIZE 1

typedef struct s_ustring{
    size_t mSize;    
    char* mStr;    
}ustring_t, Ustring, *UstringPtr;

LIBAPI void print_ustring_version();
LIBAPI const char* get_ustring_version();

LIBAPI Ustring* create_ustring(const char* tSrc);
LIBAPI Ustring* create_ustring_default();

LIBAPI int delete_ustring(UstringPtr tSelf);

LIBAPI int init_ustring(UstringPtr tUstring);
LIBAPI int release_ustring(UstringPtr tUstring);

LIBAPI int set_ustring(UstringPtr tUstring, const char* tSrc, ...);

LIBAPI int get_ustring(const UstringPtr tUstring, char** retSrc);

LIBAPI const char* ustring(const UstringPtr tUstring);
LIBAPI const size_t ustring_size(const UstringPtr tUstring);

LIBAPI int assign_ustring(UstringPtr tUstring, const UstringPtr tOther);
LIBAPI int add_ustring(UstringPtr tUstring, const UstringPtr tOther);
LIBAPI int sub_ustring(UstringPtr tUstring, const UstringPtr tSrc);
LIBAPI int sub_ustring_all(UstringPtr tUstring, const UstringPtr tSrc);

LIBAPI int remove_ustring_from(UstringPtr tUstring, const char *substr);
LIBAPI int remove_ustring_all_from(UstringPtr tUstring, const char *substr);

LIBAPI int append_ustring_from(UstringPtr tUstring, const char* tSrc);
LIBAPI int append_ustring(UstringPtr tUstring, const UstringPtr tOther);

#ifdef __cplusplus
}
#endif

#endif // USTRING_H
