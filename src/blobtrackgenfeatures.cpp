/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of Intel Corporation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/


#include <iostream>
#include <string.h>

#include <vector>

// includes used in version 2.1
//#include "cvaux.h"
//#include "highgui.h"

// includes from precomp.hpp (version 2.4.2)
#include "opencv2/legacy/legacy.hpp"
#include "opencv2/core/internal.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/video/background_segm.hpp"
#include "opencv2/legacy/blobtrack.hpp"
#include "opencv2/legacy/compat.hpp"
//

#include "abbrevs.h"
#include "colorextraction.h"

// P3k
// #define _DEBUG_BTGF

#ifdef _DEBUG_BTGF
#include <math.h>
#endif


/* The Track of a Blob */
typedef struct DefBlobTrack
{
    CvBlob      blob;
    CvBlobSeq*  pSeq;
    int         FrameBegin;
    int         FrameLast;
    int         Saved; /* flag */
    std::vector<int> *frames;
    std::vector<string> *positions;
    std::vector<string> *sizes;
    std::vector<string> *colors;
    std::vector<string> *shapes;
} DefBlobTrack;


//#define savetrack(datasetId, camId, videoId, trackId, frameFirst, frameLast) fprintf(out,"\nINSERT INTO sunar.tracks(dataset, camera, video, track, firsts, lasts) VALUES(%d, %d, %d, %d, ARRAY[%d], ARRAY[%d]);\n\n", (datasetId), (camId), (videoId), (trackId), (frameFirst), (frameLast))


class CvBlobTrackGenFeatures:public CvBlobTrackGen
{

public:
    CvBlobTrackGenFeatures(int BlobSizeNorm = 0):m_TrackList(sizeof(DefBlobTrack))
    {
        m_BlobSizeNorm = BlobSizeNorm;
        m_Frame = STARTFRAME;  // VPSTEP-1; // 1
        m_pFileName = NULL;
    };

    ~CvBlobTrackGenFeatures()
    {
        int i;
         
        // Check next track
        for(i=m_TrackList.GetBlobNum();i>0;--i)
        {
            DefBlobTrack* pTrack = (DefBlobTrack*)m_TrackList.GetBlob(i-1);
            if(!pTrack->Saved) // save track
            {
                //savetrack(dataset_id, cam_id, video_id, i, pTrack->FrameBegin, pTrack->FrameLast);
                SaveTrack(pTrack, i);
            }
        }  
        
        // Delete tracks
        for(i=m_TrackList.GetBlobNum();i>0;--i)
        {
            DefBlobTrack* pTrack = (DefBlobTrack*)m_TrackList.GetBlob(i-1);
            /* Delete sequence: */
            delete pTrack->pSeq;
            pTrack->pSeq = NULL;
            delete pTrack->frames;
            pTrack->frames = NULL;
            delete pTrack->positions;
            pTrack->positions = NULL;
            delete pTrack->sizes;
            pTrack->sizes = NULL;
            delete pTrack->colors;
            pTrack->colors = NULL;
            delete pTrack->shapes;
            pTrack->shapes = NULL;        }
        
        // log processed video information
        //fprintf(out,"\n\nINSERT INTO sunar.videos(dataset, camera, video, \"name\", length, fps) VALUES (%d, %d, %d, '%s', %d, %d);", dataset_id, cam_id, video_id, video_name, m_Frame , 25);
        // TODO: count offset! (not in paralell!)
        // close the output
        fclose(out);
    }   //  Destructor
    
    
    void SaveTrack(DefBlobTrack* pTrack, int trackId)
    {   /* Save blob track: */

        fprintf(out,"\n\nINSERT INTO sin12.tracks(seqname, t1, t2, track, frames, positions, sizes, colors, shapes) VALUES('%s', %d, %d, %d, \nARRAY[%d", video_name, pTrack->FrameBegin, pTrack->FrameLast, trackId, pTrack->frames->at(0));
        int j;
        for(j = 1; j < pTrack->frames->size(); ++j) {
            fprintf(out, ",%d", pTrack->frames->at(j));
        }
        fprintf(out, "], \nARRAY[POINT(%s)", pTrack->positions->at(0).c_str());
        for(j = 1; j < pTrack->positions->size(); ++j) {
            fprintf(out, ",POINT(%s)", pTrack->positions->at(j).c_str());
        }
        fprintf(out, "], \nARRAY[POINT(%s)", pTrack->sizes->at(0).c_str());
        for(j = 1; j < pTrack->sizes->size(); ++j) {
            fprintf(out, ",POINT(%s)", pTrack->sizes->at(j).c_str());
        }
        fprintf(out, "], \nARRAY[%s", pTrack->colors->at(0).c_str());
        for(j = 1; j < pTrack->colors->size(); ++j) {
            fprintf(out, ",%s", pTrack->colors->at(j).c_str());
        }
        fprintf(out, "], \nARRAY[%s", pTrack->shapes->at(0).c_str());
        for(j = 1; j < pTrack->shapes->size(); ++j) {
            fprintf(out, ",%s", pTrack->shapes->at(j).c_str());
        }
        fprintf(out, "]);");
        
        pTrack->Saved = 1;

    }   /* Save blob track. */
    
    
    /**
     * Saves file name, opens the output and parses the dataset, camera and video IDs.
     * If no valid conversion is performed, zeros are returned.
     * Process i-LIDS MCTR data... *[MCT_TR_02/MCTTR02a/]MCTTR0201a.mov.deint.mpeg.txt
     * @param pFileName
     */
    void    SetFileName(char* pFileName)
    {
        m_pFileName = pFileName;

        // read input
        if(m_pFileName == NULL) out = NULL;
        else out = fopen(m_pFileName,"at");

        if(out == NULL)
        {
            printf("Error! Cannot open %s file for track output\n", m_pFileName);
            return;
        }
#ifdef _DEBUG_BTGF
        printf("DEBUG BTGF: File %s opened sucesfully\n", m_pFileName);
#endif

        // prepare the video filename
        video_name = (strstr(m_pFileName, "MCT_TR_")); // find the beginning of the normlalized filename and path
        if (video_name) { // i-LIDS MCTR data :)
            char* video_str = strstr(video_name+1, "MCTTR");
            video_str = strstr(video_str+1, "MCTTR");

            // trim the video file src/enteringblobdetection.cpp:758: error: ‘cerr’ was not declared in this scopename to MCT_TR_02/MCTTR02a/MCTTR0201a.mov.deint.mpeg
            if (strstr(m_pFileName, ".txt") || strstr(m_pFileName, ".sql")) {
                video_name = substring(video_name, 0, strlen(video_name)-4);
            }
#ifdef _DEBUG_BTGF
            printf("DEBUG BTGF: Normalized video name recognized as %s\n", video_name);
#endif
        }
        else {
           video_name = m_pFileName;
           printf("Warning! BTFG cannot normalize %s file name\n", video_name);
        }


        // find the proper file name (not the path) of the form MCTTR0201a.mov.deint.mpeg.txt
        char* video_str = strrchr(pFileName, '/');
        if (video_str == NULL) {    // nenasli
            video_str = m_pFileName;
        }
        else video_str++;
#ifdef _DEBUG_BTGF
        printf("DEBUG BTGF: Proper video name recognized as %s\n", video_str);
#endif
        // find the dataset, camera, video IDs (if no valid conversion is performed, zeros are returned)
        dataset_id = atoi(substring(video_str, 5, 2));
        cam_id = atoi(substring(video_str, 7, 2));
        video_id = -(int)'a'+1 + (int)(video_str[9]);
        if (dataset_id <= 0 || cam_id <= 0 || video_id <= 0) {
            printf("FWarning! BTFG cannot recognize dataset %d, camera %d and video %d IDs\n", dataset_id, cam_id, video_id);
        }
        // else delete tracks and states beeing processed if there are some
        else {
            //fprintf(out,"DELETE FROM ONLY sunar.states WHERE dataset=%d AND camera=%d AND video=%d;\n", dataset_id, cam_id, video_id);
            //fprintf(out,"DELETE FROM ONLY sunar.tracks WHERE dataset=%d AND camera=%d AND video=%d;\n", dataset_id, cam_id, video_id);
            //fprintf(out,"DELETE FROM ONLY sunar.videos WHERE dataset=%d AND camera=%d AND video=%d;\n\n", dataset_id, cam_id, video_id);
            
            fprintf(out,"INSERT INTO sunar.sequences(seqname, seqnum, seqlocation, seqtyp, mct$dataset, mct$camera, mct$video) VALUES ('%s', %d%02d%02d, '%s', 'video', %d, %d, %d);\n", video_name, dataset_id, cam_id, video_id, video_name, dataset_id, cam_id, video_id);
        }
#ifdef _DEBUG_BTGF
        printf("DEBUG BTGF: Dataset %d, camera %d and video %d IDs were recognized\n", dataset_id, cam_id, video_id);
#endif
    };
    
    /* AddBlob */
    void    AddBlob(CvBlob* pBlob)
    {
        DefBlobTrack* pTrack = (DefBlobTrack*)m_TrackList.GetBlobByID(CV_BLOB_ID(pBlob));

        if(pTrack==NULL)
        {   /* Add new track: */
            DefBlobTrack    Track;
            Track.blob = pBlob[0];
            Track.FrameBegin = m_Frame;
            Track.pSeq = new CvBlobSeq;
            Track.Saved = 0;
            Track.frames = new std::vector<int>;
            Track.positions = new std::vector<string>;
            Track.sizes = new std::vector<string>;
            Track.colors = new std::vector<string>;
            Track.shapes = new std::vector<string>;
            m_TrackList.AddBlob((CvBlob*)&Track);
            pTrack = (DefBlobTrack*)m_TrackList.GetBlobByID(CV_BLOB_ID(pBlob));
        }   /* Add new track. */

        assert(pTrack);
        pTrack->FrameLast = m_Frame;
        assert(pTrack->pSeq);
        pTrack->pSeq->AddBlob(pBlob);
    };
    
    /** ************************************************************************
     * Process the blob, imediatelly save to DB and count features
     */
    void    Process(IplImage* pImg = NULL, IplImage* pFG = NULL) //  = NULL
    {
        // this function is called each VPSTEP frame
        printf("GF");

#ifdef _DEBUG_BTGF
        /* Draw debug info:
         * Draw all information about test sequence: */
        char        str[1024];
        int         line_type = CV_AA;   // Change it to 8 to see non-antialiased graphics.
        CvFont      font;
        IplImage*   pI = cvCloneImage(pImg);

        cvInitFont( &font, CV_FONT_HERSHEY_PLAIN, 0.9, 0.9, 0, 1, line_type );
        CvSize  TextSize;

        // print frame number (to the debug image)
        sprintf(str, "%d", m_Frame);
        cvPutText(pI, str, cvPoint(10,20), &font, cvRed);
#endif

        //
        // for each track i
        int i;
        for(i=m_TrackList.GetBlobNum(); i>0; --i)
        {
            DefBlobTrack* pTrack = (DefBlobTrack*)m_TrackList.GetBlob(i-1);

            // if the current track has finished => save track
            if(pTrack->FrameLast < m_Frame && !pTrack->Saved)
            {   
                //savetrack(dataset_id, cam_id, video_id, i, pTrack->FrameBegin, pTrack->FrameLast);
                SaveTrack(pTrack, i);
                
                /* // TODO: uncomment if necessary, but ID???
                {   // delete sequence
                    delete pTrack->pSeq;
                    pTrack->pSeq = NULL;
                    m_TrackList.DelBlob(i-1);
                } */
                
            }
            else if (!pTrack->Saved/* && (m_Frame % 2 == 1)*/) {  // if it is an active track // (not finished) for frames 1 .. 3 .. 5 ...

                CvBlob* pB = (pTrack->pSeq)? pTrack->pSeq->GetBlob(pTrack->pSeq->GetBlobNum()-1) : NULL;
                if(!pB) continue;

                // FIXED: get the rectangle enlarged by 30% compared to...
                CvRect r = CV_BLOB_RECT(pB);
                r.x -= r.width*0.15;
                r.y -= r.height*0.15;
                r.width *= 1.3;
                r.height *= 1.3;

                //if (r.width < 8 || r.height < 8) - do not save it when smaller than 8x8!!!
                if (r.width >= 8 && r.height >= 8) {

                    // this fixes some SIGSEQs ... necessary for OpenCV, unnecessary for Intel IPL
                    if (r.x < 0) {
                        r.width += r.x;
                        r.x = 0;
                    }
                    if (r.y < 0) {
                        r.height += r.y;
                        r.y = 0;
                    }
                    if ((r.x + r.width) >= pImg->width) { // both +1
                        r.width -= (r.x + r.width) - pImg->width;
                    }
                    if ((r.y + r.height) >= pImg->height) { // both +1
                        r.height -= (r.y + r.height) - pImg->height;
                    }
                    // printf(" [%d, %d, %d, %d] ", r.x, r.y, r.width, r.height);

                    //
                    // ColorExtraction (feature extractor)
                    //
                    ColorLayoutExtraction* cle = new ColorLayoutExtraction(r.width, r.height);

                    // go through the data and fill the extractor images
                    for (int y = 0; y < r.height; y++) {      // rows
                        // source image
                        byte* cvImgRow = (byte*)(pImg->imageData + (y+r.y)*pImg->widthStep);
                        // foreground image
                        byte* cvFGRow = (byte*)(pFG->imageData + (y+r.y)*pFG->widthStep);

                        // BGR images
                        byte* pb = (byte*)(cle->ib + y*cle->width);
                        byte* pg = (byte*)(cle->ig + y*cle->width);
                        byte* pr = (byte*)(cle->ir + y*cle->width);

                        // Alpha channel
                        byte* pa = (byte*)(cle->AlphaChannel() + y*cle->width);

                        // go through pixels
                        for (int x = 0; x < r.width; x++) {     // cols

                            pb[x] = cvImgRow[3*(x+r.x)    ];
                            pg[x] = cvImgRow[3*(x+r.x) + 1];
                            pr[x] = cvImgRow[3*(x+r.x) + 2];

                            pa[x] = cvFGRow[   (x+r.x) ];
                        }
                    }

                    cle->FeatureExtraction();   // start extracting, but check the value returned, maybe ...

                    // WARNING: incompatible with previous versions!

                    pTrack->frames->push_back(m_Frame);
                    stringstream ssPositions;
                    ssPositions << r.x + (r.width/2) << "," << r.y + (r.height/2); // uses central representation!!!!
                    pTrack->positions->push_back(ssPositions.str());
                    stringstream ssSizes;
                    ssSizes << r.width << "," << r.height;
                    pTrack->sizes->push_back(ssSizes.str());
                    stringstream ssColors;
                    ssColors << "ARRAY[" << (int)cle->cy[0];

                    // Insert to the database (first create an insert string)
                    //fprintf("INSERT INTO tv2_color_hlf(video, frame, features) values(%d", ) // + itoa(videoId, buf, 10);
                    //fprintf(out,"INSERT INTO sunar.states(dataset, camera, video, track, \"time\", \"position\", \"size\", color) VALUES(%d, %d, %d, %d, %d, '(%d, %d)', '(%d, %d)', ARRAY[%d",
                    //        dataset_id, cam_id, video_id, i, m_Frame, r.x, r.y, r.width, r.height, cle->cy[0]);
                    for(int j = 1; j < YCOEFS; j++) {		// 15(-1) Y-values
                        //fprintf(out, ",%d", cle->cy[j]);
                        ssColors << "," << (int)cle->cy[j];
                    }
                    // U-values
                    for(int j = 0; j < UVCOEF; j++) {		// 10 U-values
                        //fprintf(out, ",%d", cle->cu[j]);
                        ssColors << "," << (int)cle->cu[j];
                    }
                    // V-values
                    for(int j = 0; j < UVCOEF; j++) {		// 10 V-values
                        //fprintf(out, ",%d", cle->cv[j]);
                        ssColors << "," << (int)cle->cv[j];
                    }
                    //fprintf(out, "]);\n");
                    ssColors << "]";

                    pTrack->colors->push_back(ssColors.str());


                    //
                    // ShapeExtraction (Hu moments)
                    //

                    // set ROI to pFG
                    cvSetImageROI(pFG, r);

                    // Copy the image
                    IplImage* pShape = cvCreateImage(cvSize(r.width, r.height), IPL_DEPTH_8U, 1);
                    cvCopyImage(pFG, pShape);

                    // Count the moments
                    cv::Moments m = cv::moments(cv::Mat(pShape), true);

                    // central normalized moments
                    stringstream ssShapes;
                    ssShapes << "ARRAY[" << ROUND(m.nu02*1000000) << "," << ROUND(m.nu03*1000000) << ","
                            << ROUND(m.nu11*1000000) << "," << ROUND(m.nu12*1000000) << "," << ROUND(m.nu20*1000000) << ","
                            << ROUND(m.nu21*1000000) << "," << ROUND(m.nu30*1000000) << "]"; // spatial moments
                    // TODO: find out the best moments possible - this omits spetial and central moments (unnormalized)

                    pTrack->shapes->push_back(ssShapes.str());
                    // cout << ssShapes.str() << " ";

                    // delete ROI of pFG
                    cvResetImageROI(pFG);

#ifdef _DEBUG_BTGF
                    sprintf(str,"%d",i);
                    cvGetTextSize( str, &font, &TextSize, NULL );
                    cvPutText( pFG, str, cvPoint(3, 3), &font, CV_RGB(0,255,255));
/*
                    cvNamedWindow("Shape", 0);
                    cvShowImage("Shape", pShape);
                    cvWaitKey(50);
*/
                    cvRectangleR(pFG, r, cvWhite);
#endif

                    // clean up
                    delete (cle);
                    cle = null;
                    cvReleaseImage(&pShape);
                    pShape = null;
                }
            }   // else beyond a track

        }   //  Check next track.


        printf("%d ", m_Frame); // log
        m_Frame += VPSTEP; // += 1
        
#ifdef _DEBUG_BTGF
        cvNamedWindow( "Features FG", 0);
        cvShowImage( "Features FG", pFG);
        cvWaitKey(10);
#endif
    }
    
    /* */
    void Release()
    {
        delete this;
    };


protected:
    int         m_Frame;
    char*       m_pFileName;
    CvBlobSeq   m_TrackList;
    int         m_BlobSizeNorm;
    // P3: database export
    FILE*       out;            // file writer
    int         dataset_id;
    int         cam_id;
    int         video_id;
    char*       video_name;    // MCT_TR_02/MCTTR02a/MCTTR0201a.mov.deint.mpeg
};  /* class CvBlobTrackGen1 */


CvBlobTrackGen* cvCreateModuleBlobTrackGenFeatures()
{
    return (CvBlobTrackGen*) new CvBlobTrackGenFeatures(0);
}
