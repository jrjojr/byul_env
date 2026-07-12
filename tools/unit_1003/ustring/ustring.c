/***************************************************************************
* Copyright 2024. Jungrai Jo <jungraijo@gmail.com> 
* All Rights Reserved.
*
* ustring 
*
***************************************************************************/

#include "ustring.h"

#include <stdio.h>
#include <stdlib.h> //realloc, malloc, free
#include <string.h>
#include <stdarg.h> // va_list.

#ifdef __cplusplus
    void print_extern(){
        DEBUG_PRINT("this is extern print \n");
        printf("defined __cplusplus\n");
    }
#endif

LIBAPI void print_ustring_version(){
    printf("%s version : %d.%d.%d.%d\n", "ustring", 
        USTRING_VERSION_MAJOR,
        USTRING_VERSION_MINOR,
        USTRING_VERSION_PATCH,
        USTRING_VERSION_TWEAK);
}

LIBAPI const char* get_ustring_version(){
    char buf[16];
    sprintf(buf, "%d.%d.%d.%d", 
        USTRING_VERSION_MAJOR,
        USTRING_VERSION_MINOR,
        USTRING_VERSION_PATCH,
        USTRING_VERSION_TWEAK
        );
    return buf;
}

LIBAPI Ustring* create_ustring(const char* tSrc){
    Ustring* r = NULL;
    r = create_ustring_default();
    set_ustring(r, tSrc);
    return r;
}

LIBAPI Ustring* create_ustring_default(){
    Ustring* r = NULL;
    r = (Ustring*)malloc(sizeof(Ustring));
    init_ustring(r);
    return r;
}

LIBAPI int delete_ustring(UstringPtr tSelf){
    release_ustring(tSelf);
    if(tSelf != NULL){
        free(tSelf);
    }
    return 0;
}

LIBAPI int init_ustring(UstringPtr tUstring){
    tUstring->mStr = (char*)malloc(USTRING_EMPTY_SIZE);
    strcpy(tUstring->mStr, "");
    tUstring->mSize = USTRING_EMPTY_SIZE;
    return 0;    
}

LIBAPI int release_ustring(UstringPtr tUstring){
        if(tUstring->mStr){
            // DEBUG_PRINT("release_ustring : tUstring->mStr : not NULL.\n");
            free(tUstring->mStr);
            tUstring->mStr = NULL;
            // DEBUG_PRINT("release_ustring : tUstring->mStr : free complete.\n");
            return 0;
        }
    // DEBUG_PRINT("release_ustring : tUstring->mStr : is NULL.\n");
    return -1;
}

LIBAPI int set_ustring(UstringPtr tUstring, const char* tSrc, ...){
    // DEBUG_PRINT("set_ustring : tSrc is %s\n", tSrc);
  va_list args;
  char buf[256];
  va_start (args, tSrc);
  vsprintf (buf, tSrc, args);
  va_end (args);

    size_t aSize = strlen(buf) + 1;
    tUstring->mStr = (char*)realloc(tUstring->mStr, aSize);
    // DEBUG_PRINT("set_ustring : tUstring->mSize is %zd\n", tUstring->mSize);
    strcpy(tUstring->mStr, buf);
    // DEBUG_PRINT("set_ustring : tUstring->mStr is %s\n", tUstring->mStr);
    // release_ustring(tUstring);
    tUstring->mSize = aSize;
    
    return 0;
}

LIBAPI int get_ustring(const UstringPtr tUstring, char** retSrc){

    *retSrc = tUstring->mStr;
    return 0;
}

LIBAPI const char* ustring(const UstringPtr tUstring){
    // const size_t aSize = ustring_size(tUstring);
    // char *r = (char*)malloc(sizeof(char)*aSize);
    // strcpy_s(r, aSize, tUstring->mStr);
    // return r;
    return tUstring->mStr;
}

LIBAPI const size_t ustring_size(const UstringPtr tUstring){
    return tUstring->mSize;
}

LIBAPI int assign_ustring(UstringPtr tUstring, const UstringPtr tOther){
    size_t aSize = strlen(tOther->mStr) + 1;
    tUstring->mStr = (char*)realloc(tUstring->mStr, aSize);

    strcpy(tUstring->mStr, tOther->mStr);

    tUstring->mSize = aSize;
    
    return 0;    
}

LIBAPI int add_ustring(UstringPtr tUstring, const UstringPtr tOther){
    return append_ustring_from(tUstring, tOther->mStr);
}

LIBAPI int sub_ustring(UstringPtr tUstring, const UstringPtr tSrc){
    if (!tUstring || !tSrc) {
        // Handle invalid pointers (e.g., return an error code)
        // DEBUG_PRINT("error : sub_ustring : !tUstring || !tSrc.\n");
        return -1;
    }
    return remove_ustring_from(tUstring, tSrc->mStr);
}

LIBAPI int sub_ustring_all(UstringPtr tUstring, const UstringPtr tSrc){
    if (!tUstring || !tSrc) {
        // Handle invalid pointers (e.g., return an error code)
        // DEBUG_PRINT("error : sub_ustring : !tUstring || !tSrc.\n");
        return -1;
    }
    return remove_ustring_all_from(tUstring, tSrc->mStr);
    }

LIBAPI int remove_ustring_from(UstringPtr tUstring, const char *substr) {
    char* pos = strstr(tUstring->mStr, substr);
    if (pos) {
        size_t substrLen = strlen(substr);
        size_t remainingLen = strlen(pos + substrLen);
        // +1 for the null terminator
        memmove(pos, pos + substrLen, remainingLen + 1); 
        return 1;
    }
    return 0;
}

LIBAPI int remove_ustring_all_from(UstringPtr tUstring, const char *substr) {
    char *src = tUstring->mStr;
    char *dst = tUstring->mStr;
    int substrLen = strlen(substr);
    int aCount = 0;

    while (*src) {
        if (strncmp(src, substr, substrLen) == 0) {
            src += substrLen;
        } else {
            *dst = *src;
            dst++;
            src++;
        }
        aCount++;
    }
    *dst = '\0';
    return aCount;
}

LIBAPI int append_ustring_from(UstringPtr tUstring, const char* tSrc){
    size_t aSize = strlen(tUstring->mStr) + strlen(tSrc) + 1;

    tUstring->mStr = (char*)realloc(tUstring->mStr, aSize);
    tUstring->mSize = aSize;
    tUstring->mStr = strcat(tUstring->mStr, tSrc);
    return 0;
}

LIBAPI int append_ustring(UstringPtr tUstring, const UstringPtr tOther){
    size_t aSize = strlen(tUstring->mStr) + strlen(tOther->mStr) + 1;

    tUstring->mStr = (char*)realloc(tUstring->mStr, aSize);
    tUstring->mSize = aSize;
    tUstring->mStr = strcat(tUstring->mStr, tOther->mStr);
    return 0;    
}