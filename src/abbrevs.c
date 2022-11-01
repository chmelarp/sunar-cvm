/*
 * File:   abbrevs.h
 * Author: chmelarp
 *
 * Created on 16. duben 2008, 15:36
 */
#include "abbrevs.h"

// A nice C itoa (not a part of the ANSI C standard)
char* itoa(int val, char* buf) {
    if (buf == null) buf = alloc(char, 32);
    sprintf(buf, "%d", val);
    return buf;
}

// A nice C ftoa (not a part of the ANSI C standard)
char* ftoa(double val, char* buf) {
    if (buf == null) buf = alloc(char, 32);
    sprintf(buf, "%f", val);
    return buf;
}

// A nice C substring (not a part of the ANSI C standard)
char* substring(const char* str, size_t begin, size_t len)
{
    if (str == 0 || strlen(str) == 0 || strlen(str) < begin || strlen(str) < (begin+len))
        return NULL;

    return strndup(str + begin, len);
}
