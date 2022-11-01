/*
 * File:   abbrevs.h
 * Author: chmelarp
 *         ipesek
 *
 * Created on 16. duben 2008, 15:36
 */


#ifndef ABBREVS_H
#define	ABBREVS_H

// Step for video processing, normaly: 2 for debug, 1 for release... has NO effect anymore!!!!
#define VPSTEP 1
// well... if to skip some frames
#define STARTFRAME 1

//#define _iLIDS



/**
 * General (C, C++) abbreviations
 *
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#if 1
    #include "opencv2/highgui/highgui.hpp"

    // define colors
    #define cvWhite  CV_RGB(255,255,255)
    #define cvBlue   CV_RGB(000,000,255)
    #define cvGreen  CV_RGB(000,255,000)
    #define cvYellow CV_RGB(255,255,000)
    #define cvRed    CV_RGB(255,000,000)
    #define cvBlack  CV_RGB(000,000,000)


    // define to draw a cross
    #define cvCross( img, x, y, color, dim, wid )                  \
        cvLine( img, cvPoint( x - dim, y       ),                  \
                     cvPoint( x + dim, y       ), color, wid, 0 ); \
        cvLine( img, cvPoint( x      , y - dim ),                  \
                     cvPoint( x      , y + dim ), color, wid, 0 )
#endif

// take it easy(TM) (Java like)
#define null NULL
#ifdef	__cplusplus
#define String string
using namespace std;
#endif

// easy boolean(TM)
// use boolean in stead of bool, because of C - it doesn't have it :)
// easy something else
#ifdef	__cplusplus
#define boolean bool
#else
#define boolean int
#define false   0
#define true    1
#endif

#ifdef	__cplusplus
    // to be a byte (unsigned char)
    #define byte unsigned char
    // word (unsigned int 32b a nikdy jinak :)
    #define word unsigned int;
#else
    // to be a byte (unsigned char)
    #define byte uint8_t
    // word (unsigned int 32b a nikdy jinak :)
    #define word uint32_t;
#endif

// easy malloc(TM)
// use alloc and free instead of new and delete in case of structures
// because it doesn't initialize anything and C doesn't have it
#define alloc(type,n) (type *)malloc((n)*sizeof(type))
#define allinit(pointer, type, n) (pointer)=(type *)malloc((n)*sizeof(type)), memset((pointer), 0, (n)*sizeof(type)), (pointer)

// short strings length
#define MAXChars 400
#define MAXString 4000

// use pi
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// absolute value, signum
#define ABS(a) ((a) >= 0 ? (a) : (-(a)))
#define SIGN(a) ((a) > 0 ? 1 : -1)

// maximum, minimum
#ifndef MIN
#define MIN(a,b) ((a) > (b) ? (b) : (a))
#endif
#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

// round (returns long integer)
#define ROUND(x) floor(0.5 + (x))
// swap two values
#define SWAP(a,b) swp_temp=(a);(a)=(b);(b)=swp_temp
// square x^2
#define SQR(x) (x)*(x)


    
#ifdef	__cplusplus
extern "C" {
#endif

// A nice C itoa (not a part of the ANSI C standard)
char* itoa(int val, char* buf);
// char* itoa(int val, char* buf, int base);
// A nice C ftoa (not a part of the ANSI C standard)
char* ftoa(double val, char* buf);
// char* ftoa(double val, char* buf, int base);

// A nice C substring (not a part of the ANSI C standard)
// Don't forget to free the result!
char* substring(const char* str, size_t begin, size_t len);
    
#ifdef	__cplusplus
}
#endif

#endif	/* ABBREVS_H */

