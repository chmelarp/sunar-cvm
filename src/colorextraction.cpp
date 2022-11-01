
#include "colorextraction.h"

// construtor
ColorLayoutExtraction::ColorLayoutExtraction(int w, int h) {
    width = w;
    height = h;
    
    ir = alloc(byte, w*h);
    ig = alloc(byte, w*h);
    ib = alloc(byte, w*h);
    ia = null;

    cy = alloc(byte, 64);
    cu = alloc(byte, 64);
    cv = alloc(byte, 64);
}

// destructor
ColorLayoutExtraction::~ColorLayoutExtraction() {
    width = 0;
    height = 0;
    
    free(ir);
    free(ig);
    free(ib);
    if (ia != null) free(ia);

    free(cy);
    free(cu);
    free(cv);
}

// construtor
byte* ColorLayoutExtraction::AlphaChannel() {
    
    if(ia == null) ia = alloc(byte, width*height);
    return ia;
}

// feature extractor
boolean ColorLayoutExtraction::FeatureExtraction() {
	// check for media
	if (!width || !height || !ir || !ig || !ib) return false;

	short small_img[3][64];
	createSmallImage(small_img);

    fdct(small_img[0]);
    fdct(small_img[1]);
    fdct(small_img[2]);

	// quantize to 8 bits
    cy[0] = (byte)(small_img[0][0]/8);
    cu[0] = (byte)(small_img[1][0]/8);
    cv[0] = (byte)(small_img[2][0]/8);

    // quantize other coefs
	for(int i = 1; i < 64; i++) {
		cy[i] = quant_ac((small_img[0][(zig_zag[i])])/2);
		cu[i] = quant_ac(small_img[1][(zig_zag[i])]);
		cv[i] = quant_ac(small_img[2][(zig_zag[i])]);
	}

#ifdef __DEBUG
	fprintf(stdout, "CLE Y(%d %d %d %d %d %d) U(%d %d %d) V(%d %d %d)\n",
		cy[0], cy[1], cy[2], cy[3], cy[4], cy[5],
		cu[0], cu[1], cu[2], 
		cv[0], cv[1], cv[2]);
#endif
    
    return true;
}


///////////////////////////////////////////////////////////////////////////////
//
// This software module was originally developed by
//
// NEC Corp., Akio Yamada, Eiji Kasutani
// (contributing organizations names)
//
// in the course of development of the MPEG-7 Experimentation Model.
//
// This software module is an implementation of a part of one or more MPEG-7
// Experimentation Model tools as specified by the MPEG-7 Requirements.
//
// ISO/IEC gives users of MPEG-7 free license to this software module or
// modifications thereof for use in hardware or software products claiming
// conformance to MPEG-7.
//
// Those intending to use this software module in hardware or software products
// are advised that its use may infringe existing patents. The original
// developer of this software module and his/her company, the subsequent
// editors and their companies, and ISO/IEC have no liability for use of this
// software module or modifications thereof in an implementation.
//
// Copyright is not released for non MPEG-7 conforming products. The
// organizations named above retain full right to use the code for their own
// purpose, assign or donate the code to a third party and inhibit third parties
// from using the code for non MPEG-7 conforming products.
//
// Copyright (c) 1998-1999.
//
// This notice must be included in all copies or derivative works.
//
int ColorLayoutExtraction::quant_ac(int i) {
	int j;
	if(i>239) i= 239;
	if(i<-256) i= -256;
	if ((abs(i)) > 127) j= 64 + (abs(i))/4;
	else if ((abs(i)) > 63) j=32+(abs(i))/2;
	else j=abs(i);
	j = (i<0)?-j:j;
	j+=132;
	return j;
}

void ColorLayoutExtraction::createSmallImage(short small_img[3][64]) {
	int y_axis, x_axis;
	int i, j, k ;
	int x, y;
	long small_block_sum[3][64];
	int cnt[64];

	byte* pR = ir;
	byte* pG = ig;
	byte* pB = ib;
	byte* pA = ia;

	for(i = 0; i < (8 * 8) ; i++){
		cnt[i]=0;
		for(j=0;j<3;j++){
			small_block_sum[j][i]=0;
			small_img[j][i] = 0;
		}
	}

	// upsampling for small pictures (less than 8x8) to avoid
	// floating point exception     
//	int rep_width=(width<8)?  7:0;
//	int rep_height=(width<8)? 7:0;
//	width=(width<8)? 8*width:width;
//	height=(height<8)? 8*height:height;
	byte *buffer, *ptrR, *ptrG, *ptrB, *ptrA;

	// modify the code to use new operation to alloc memory for 3D array
	// arbitrary shape
        // P3 new -> alloc
	buffer = alloc(byte, 4*width*height);
	//	    buffer = new byte[3*width*height];
	// end modification

	ptrR=buffer;
	ptrG=buffer+width*height;
	ptrB=buffer+2*width*height;
	// arbitrary shape
	if(pA) 
		ptrA=buffer+3*width*height;
	// end modification

	for(y=0; y<height; y++){
		for(x=0; x<width; x++){
			*ptrR++ =*pR; *ptrG++ =*pG; *ptrB++ =*pB;
			// arbitrary shape
			if(pA) 
				*ptrA++ =*pA;
			// end modification

//			for(i=0; i<rep_width; i++){
//				*ptrR++ =*pR; *ptrG++ =*pG; *ptrB++ =*pB;
//				// arbitrary shape
//				if(pA)
//					*ptrA++ = *pA;
//				// end modification
//			}
			pR++; pG++; pB++;
			// arbitrary shape
			if(pA) 
				pA++;
			// end modification
		}
                
//		for(j=0; j<rep_height; j++) {
//                      memcpy(buffer+0*(width*height)+(8*y+j+1)*width+0,
//				buffer+0*(width*height)+(8*y)*width+0,
//				width);
//			memcpy(buffer+1*(width*height)+(8*y+j+1)*width+0,
//				buffer+1*(width*height)+(8*y)*width+0,
//				width);
//			memcpy(buffer+2*(width*height)+(8*y+j+1)*width+0,
//				buffer+2*(width*height)+(8*y)*width+0,
//				width);
//			// arbitrary shape
//			if(pA)
//				memcpy(buffer+3*(width*height)+(8*y+j+1)*width+0,
//				buffer+3*(width*height)+(8*y)*width+0,
//				width);
//			// end modification
//			ptrR+= width; ptrG+= width; ptrB+= width;
//			// arbitrary shape
//			if(pA)
//				ptrA+= width;
//			// end modification
//		}
	}
	pR=buffer;
	pG=buffer+width*height;
	pB=buffer+2*width*height;
	// arbitrary shape
	if(pA) 
		pA=buffer+3*width*height;
	// end modification
	// end of upsampling 

	short R, G, B;
	// arbitrary shape
	short A;
	// end modification
	double yy;

	for(y=0; y<height; y++){
		for(x=0; x<width; x++){
			y_axis = (int)(y/(height/8.0));
			x_axis = (int)(x/(width/8.0));
			k = y_axis * 8 + x_axis;

			G = *pG++;
			B = *pB++;
			R = *pR++;
			// arbitrary shape
			if(pA){ 
				A = *pA++;
				if(A==0) continue;
			}
			// end modification

			// RGB to YCbCr
			yy = ( 0.299*R + 0.587*G + 0.114*B)/256.0; 
			small_block_sum[0][k] += (int)(219.0 * yy + 16.5); // Y
			small_block_sum[1][k] += (int)(224.0 * 0.564 * (B/256.0*1.0 - yy) + 128.5); // Cb
			small_block_sum[2][k] += (int)(224.0 * 0.713 * (R/256.0*1.0 - yy) + 128.5); // Cr

			cnt[k]++;
		}
	}


	// arbitrary shape
	double total_sum[3]={0.0, 0.0, 0.0};
	int valid_cell=0;
	// end modification

	// create 8x8 image
	for(i=0; i<8; i++){
		for(j=0; j<8; j++){
			for(k=0; k<3; k++){
				if(cnt[i*8+j]) 
					small_img[k][i*8+j] = (small_block_sum[k][i*8+j] / cnt[i*8+j]);
				else 
					small_img[k][i*8+j] = 0;
				// arbitrary shape
				total_sum[k]+=small_img[k][i*8+j];
				// end modification
			}
			// arbitrary shape
			if(cnt[i*8+j]!=0) valid_cell++;
			// end modification
		}
	}

	// arbitrary shape
	if(pA) {
		for(k=0; k<3; k++){
			if(valid_cell)
				total_sum[k]=total_sum[k]/valid_cell;
			else
				total_sum[k]=0;
		}
		for(i=0; i<8; i++){
			for(j=0; j<8; j++){
				for(k=0; k<3; k++){
					if(small_img[k][i*8+j]==0){
						small_img[k][i*8+j]=(short)total_sum[k];
					}
				}
			}
		}
	}
	// end modification

	free(buffer);
}

/* Copyright (C) 1996, MPEG Software Simulation Group. All Rights Reserved.
*
* Disclaimer of Warranty
*
* These software programs are available to the user without any license fee or
* royalty on an "as is" basis.  The MPEG Software Simulation Group disclaims
* any and all warranties, whether express, implied, or statuary, including any
* implied warranties or merchantability or of fitness for a particular
* purpose.  In no event shall the copyright-holder be liable for any
* incidental, punitive, or consequential damages of any kind whatsoever
* arising from the use of these programs.
*
* This disclaimer of warranty extends to the user of these programs and user's
* customers, employees, agents, transferees, successors, and assigns.
*
* The MPEG Software Simulation Group does not represent or warrant that the
* programs furnished hereunder are free of infringement of any third-party
* patents.
*
* Commercial implementations of MPEG-1 and MPEG-2 video, including shareware,
* are subject to royalty fees to patent holders.  Many of these patents are
* general enough such that they are unavoidable regardless of implementation
* design.
*/
void ColorLayoutExtraction::fdct(short *block)
{
	int i, j, k;
	double s;
	double tmp[64];
	for (i=0; i<8; i++){
		for (j=0; j<8; j++){
			s = 0.0;
			for (k=0; k<8; k++) s += cos_c[j][k] * block[8*i+k];
			tmp[8*i+j] = s;
		}
	}
	for (j=0; j<8; j++){
		for (i=0; i<8; i++){
			s = 0.0;
			for (k=0; k<8; k++) s += cos_c[i][k] * tmp[8*k+j];
			block[8*i+j] = (int)ROUND(s);
		}
	}
}
