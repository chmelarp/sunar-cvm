//
// File:   colorextraction.h
// Author: chmelarp
//
// Created on 27. červen 2008, 11:55
//

#ifndef COLOREXTRACTION_H
#define	COLOREXTRACTION_H

// P3k: use 15 + 10x2 = 35 coeffs
#define YCOEFS 15
#define UVCOEF 10

#include "abbrevs.h"

#include <string>

static const byte zig_zag[64] = {           // zig_zag scan pattern (JPEG)
    0,1,8,16,9,2,3,10,17,24,32,25,18,11,4,5,
    12,19,26,33,40,48,41,34,27,20,13,6,7,14,21,28,
    35,42,49,56,57,50,43,36,29,22,15,23,30,37,44,51,
    58,59,52,45,38,31,39,46,53,60,61,54,47,55,62,63
};

static const double cos_c[8][8] = {       // Cosine coeficients for the transformation
    {3.535534e-01, 3.535534e-01, 3.535534e-01, 3.535534e-01, 3.535534e-01, 3.535534e-01, 3.535534e-01, 3.535534e-01},
    {4.903926e-01, 4.157348e-01, 2.777851e-01, 9.754516e-02, -9.754516e-02, -2.777851e-01, -4.157348e-01, -4.903926e-01},
    {4.619398e-01, 1.913417e-01, -1.913417e-01, -4.619398e-01, -4.619398e-01, -1.913417e-01, 1.913417e-01, 4.619398e-01},
    {4.157348e-01, -9.754516e-02, -4.903926e-01, -2.777851e-01,	2.777851e-01, 4.903926e-01, 9.754516e-02, -4.157348e-01},
    {3.535534e-01, -3.535534e-01, -3.535534e-01, 3.535534e-01, 3.535534e-01, -3.535534e-01, -3.535534e-01, 3.535534e-01},
    {2.777851e-01, -4.903926e-01, 9.754516e-02, 4.157348e-01, -4.157348e-01, -9.754516e-02, 4.903926e-01, -2.777851e-01},
    {1.913417e-01, -4.619398e-01, 4.619398e-01, -1.913417e-01, -1.913417e-01, 4.619398e-01, -4.619398e-01, 1.913417e-01},
    {9.754516e-02, -2.777851e-01, 4.157348e-01, -4.903926e-01, 4.903926e-01, -4.157348e-01, 2.777851e-01, -9.754516e-02}
};


/**
 * Extracts color coefficients from Zig-Zag of DCT eg. in JPEG and MPEG
 *
 */
class ColorLayoutExtraction  {

public:
    ColorLayoutExtraction(int w, int h);
    virtual ~ColorLayoutExtraction();

// variables
public:
    int width, height;

    // images
    byte* ir, *ig, *ib, *ia;
    // coeficients
    byte* cy, *cu, *cv;
            
// FeatureExtraction
public:
    boolean FeatureExtraction();
    byte* AlphaChannel();
    
private:
    
    // fast discrete cosine transform
    void fdct(short *block);
    // coeficient quantization
    int quant_ac(int i);
    // 
    void createSmallImage(short small_img[3][64]);

};

#endif	/* COLOREXTRACTION_H */

