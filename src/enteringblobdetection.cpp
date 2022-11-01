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

/*
This file implements the virtual interface defined as "CvBlobDetector".
This implementation based on a simple algorithm:
A new blob is detected when several successive frames contains connected components
which have uniform motion not at an unreasonably high speed.
Separation from border and already tracked blobs are also considered.

For an entrypoint into the literature see:

     Appearance Models for Occlusion Handling
     Andrew Senior &t al, 8p 2001
     http://www.research.ibm.com/peoplevision/PETS2001.pdf

*/

// TODO: object detector based on peopledetect for splitting blobs
//       which actually correspond to groups of objects
// #define USE_OBJECT_DETECTOR

// P3k
// #define _iLIDS

// debug information
// #define _DEBUG_EBD

#include "abbrevs.h"

// includes used in version 2.1
//#include "cvaux.h"
//#include "cvaux.hpp"
//#include "highgui.h"
//#include "cvvidsurv.hpp"

// includes from precomp.hpp (version 2.4.2)
#include "opencv2/legacy/legacy.hpp"
#include "opencv2/core/internal.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/video/background_segm.hpp"
#include "opencv2/legacy/blobtrack.hpp"
#include "opencv2/legacy/compat.hpp"
//

#ifdef _iLIDS
    #include <string>

    // PostgreSQL
    #include <postgresql/libpq-fe.h>
    // minerva2
    #define CONNINFO "host=minerva3.fit.vutbr.cz port=5432 dbname=trecvid user=trecvid password='f2wipufi'" // sslmode=require

    using namespace std;
#endif


/* Simple blob detector2.  */
/* Number of successive frames to analyse: */
#define SEQ_SIZE_MAX    30
#define SEQ_NUM         1000
typedef struct
{
    int     size;
    CvBlob* pBlobs[SEQ_SIZE_MAX];
} DefSeq;


class CvBlobDetectorCCiLIDS:public CvBlobDetector
{
public:
    CvBlobDetectorCCiLIDS();
   ~CvBlobDetectorCCiLIDS() {};
    int DetectNewBlob(IplImage* pImg, IplImage* pFGMask, CvBlobSeq* pNewBlobList, CvBlobSeq* pOldBlobList);
    void Release() {delete this;};

    // sets the filename, dataset, camera, video... and runs FindStarters if _iLIDS defined
    void SetFileName(char* pFileName);

    // predefined
    virtual void ParamUpdate();

private:
    /* Lists of connected components detected on previous frames: */
    CvBlobSeq*      m_pBlobLists[SEQ_SIZE_MAX];
    DefSeq          m_TrackSeq[SEQ_NUM];
    int             m_TrackNum;
    float           m_HMin;
    float           m_WMin;
    float           m_MinDistToBorder;
    int             m_Clastering;
    int             SEQ_SIZE;

    /** If not 0 then the detector is loaded from the specified file
     * and it is applied for splitting blobs which actually correspond
     * to groups of objects:
     */
    char*           m_param_split_detector_file_name;

    /* If not 0, contains the name of the file beeing processed */
    char*           m_param_video_file_name;
    int             frame_no; // = -VPSTEP+1
#ifdef _iLIDS
    int             dataset_id;
    int             video_id;
    int             camera_id;
    CvBlobSeq       starters;
    // int          starter_tracks[5][5];   // SS nactene pripadne z DB (frame, x, y, w, h)
#endif

    float           m_param_roi_scale;
    int             m_param_only_roi;

    CvObjectDetector* m_split_detector;
    CvSize          m_min_window_size;
    int             m_max_border;

    CvBlobSeq       m_detected_blob_seq;
    CvSeq*          m_roi_seq;

    CvBlobSeq       m_DEBUG_EBD_blob_seq;

#ifdef _iLIDS

    /**
     * Finds starter tracks in the DB (connects, selects if there are any in sunar.evaluation_states, disconnect)
     */
    void FindStarters();
#endif

    /**
     * Counts the overlapping ratio
     * @param blob1
     * @param blob2
     * @return ratio of intersection to union of two rectangles
     */
    static float Overlaps(CvBlob* blob1, CvBlob* blob2);

    /** 
     * some static functions
     */
    static int CompareContour(const void* a, const void* b, void* );
    void cvFindBlobsByCCClasters(IplImage* pImg, IplImage* pFG, CvBlobSeq* pBlobs, CvMemStorage* storage);
};


/* Blob detector creator (sole interface function for this file): */
CvBlobDetector* cvCreateBlobDetectorCCiLIDS() {return new CvBlobDetectorCCiLIDS;}

/* Constructor for BlobDetector: */
CvBlobDetectorCCiLIDS::CvBlobDetectorCCiLIDS() : m_split_detector(0), m_detected_blob_seq(sizeof(CvDetectedBlob)), m_roi_seq(0), 
                                                 m_DEBUG_EBD_blob_seq(sizeof(CvDetectedBlob))
{
    /*CvDrawShape shapes[] =
    {
        { CvDrawShape::RECT,    {{255,255,255}} },
        { CvDrawShape::RECT,    {{0,0,255}} },
        { CvDrawShape::ELLIPSE, {{0,255,0}} }
    };
    int num_shapes = sizeof(shapes) / sizeof(shapes[0]);*/

    int i = 0;
    SEQ_SIZE = 10;
    AddParam("Latency",&SEQ_SIZE);
    for(i=0;i<SEQ_SIZE_MAX;++i)m_pBlobLists[i] = NULL;
    for(i=0;i<SEQ_NUM;++i)m_TrackSeq[i].size = 0;
    m_TrackNum = 0;

    m_HMin = 0.020f;
    m_WMin = 0.015f;    // P3k <- 0.1
    AddParam("HMin",&m_HMin);
    AddParam("WMin",&m_WMin);
    m_MinDistToBorder = 1.1f;
    AddParam("MinDistToBorder",&m_MinDistToBorder);
    CommentParam("MinDistToBorder","Minimal allowed distance from blob center to image border in blob sizes");

    m_Clastering=1;
    AddParam("Clastering",&m_Clastering);
    CommentParam("Clastering","Minimal allowed distance from blob center to image border in blob sizes");

    m_param_split_detector_file_name = 0;
#ifdef USE_OBJECT_DETECTOR
    AddParam("Detector", (const char**)&m_param_split_detector_file_name);
    CommentParam("Detector", "Detector file name");
#endif

    m_param_roi_scale = 1.5F;
    AddParam("ROIScale", &m_param_roi_scale);
    CommentParam("ROIScale", "Determines the size of search window around a blob");

    m_param_only_roi = 1;
    AddParam("OnlyROI", &m_param_only_roi);
    CommentParam("OnlyROI", "Shows the whole debug image (0) or only ROIs where the detector was applied (1)");

    m_min_window_size = cvSize(0,0);
    m_max_border = 0;
    m_roi_seq = cvCreateSeq( 0, sizeof(*m_roi_seq), sizeof(CvRect), cvCreateMemStorage() );

    SetModuleName("CC");

    // P3k
    m_param_video_file_name = 0;

    frame_no = STARTFRAME-VPSTEP; //1-2 = -1 OK // OR  -VPSTEP+1; // == -2+1=-1

#ifdef _iLIDS
    AddParam("FileName", (const char**)&m_param_video_file_name);
    CommentParam("FileName", "Name of the file beeing processed");

    dataset_id = 0;
    video_id = 0;
    camera_id = 0;
    // starters = new CvBlobSeq();
#endif
}


// predefined
void CvBlobDetectorCCiLIDS::ParamUpdate()
{
    if(SEQ_SIZE<1)SEQ_SIZE=1;
    if(SEQ_SIZE>SEQ_SIZE_MAX)SEQ_SIZE=SEQ_SIZE_MAX;

#ifdef USE_OBJECT_DETECTOR
    if( m_param_split_detector_file_name )
    {
        m_split_detector = new CvObjectDetector();
        if( !m_split_detector->Load( m_param_split_detector_file_name ) )
        {
            delete m_split_detector;
            m_split_detector = 0;
        }
        else
        {
            m_min_window_size = m_split_detector->GetMinWindowSize();
            m_max_border = m_split_detector->GetMaxBorderSize();
        }
    }
#endif
#ifdef _iLIDS
    if(m_param_video_file_name) {
        SetFileName(m_param_video_file_name);
    }
#endif

}



/* cvDetectNewBlobs
 * Return 1 and fill blob pNewBlob  with
 * blob parameters if new blob is detected:
 */
int CvBlobDetectorCCiLIDS::DetectNewBlob(IplImage* pImg, IplImage* pFGMask, CvBlobSeq* pNewBlobList, CvBlobSeq* pOldBlobList)
{
    int         result = 0;
    CvSize      S = cvSize(pFGMask->width,pFGMask->height);

    // this function is called each VPSTEP frame
    frame_no += VPSTEP; // += 2
    printf("BD"); // log

    /* Shift blob list: */
    {
        int     i;
        if(m_pBlobLists[SEQ_SIZE-1]) delete m_pBlobLists[SEQ_SIZE-1];

        for(i=SEQ_SIZE-1; i>0; --i)  m_pBlobLists[i] = m_pBlobLists[i-1];

        m_pBlobLists[0] = new CvBlobSeq;

    }   /* Shift blob list. */

    /* Create contours and add new blobs to blob list: */
    {   /* Create blobs: */
        CvBlobSeq       Blobs;
        CvMemStorage*   storage = cvCreateMemStorage();

        if(m_Clastering)                                                // allways
        {   /* Glue contours: */
            cvFindBlobsByCCClasters(pImg, pFGMask, &Blobs, storage );   // P3k: pImg added for debug proposes
        }   /* Glue contours. */
        else                                                            // this never occurs
        { /**/
            IplImage*       pIB = cvCloneImage(pFGMask);
            CvSeq*          cnts = NULL;
            CvSeq*          cnt = NULL;
            cvThreshold(pIB,pIB,128,255,CV_THRESH_BINARY);
            cvFindContours(pIB,storage, &cnts, sizeof(CvContour), CV_RETR_EXTERNAL);

            /* Process each contour: */
            for(cnt = cnts; cnt; cnt=cnt->h_next)
            {
                CvBlob  NewBlob;
                /* Image moments: */
                double      M00,X,Y,XX,YY;
                CvMoments   m;
                CvRect      r = ((CvContour*)cnt)->rect;
                CvMat       mat;
                if(r.height < S.height*m_HMin || r.width < S.width*m_WMin) continue;
                cvMoments( cvGetSubRect(pFGMask,&mat,r), &m, 0 );
                M00 = cvGetSpatialMoment( &m, 0, 0 );
                if(M00 <= 0 ) continue;
                X = cvGetSpatialMoment( &m, 1, 0 )/M00;
                Y = cvGetSpatialMoment( &m, 0, 1 )/M00;
                XX = (cvGetSpatialMoment( &m, 2, 0 )/M00) - X*X;
                YY = (cvGetSpatialMoment( &m, 0, 2 )/M00) - Y*Y;
                NewBlob = cvBlob(r.x+(float)X,r.y+(float)Y,(float)(4*sqrt(XX)),(float)(4*sqrt(YY)));
                Blobs.AddBlob(&NewBlob);

            }   /* Next contour. */

            cvReleaseImage(&pIB);

        }   /* One contour - one blob. */

        // TODO: find my old blob intersected with these
        // TODO: pridat check, jestli je tam ten blob (2-5)

        {   /* Delete small and intersected blobs: */ 
            int i;
            for(i=Blobs.GetBlobNum(); i>0; i--)
            {
                CvBlob* pB = Blobs.GetBlob(i-1);

                if(pB->h < S.height*m_HMin || pB->w < S.width*m_WMin)
                {
                    Blobs.DelBlob(i-1);
                    continue;
                }

                if(pOldBlobList)
                {
                    int j;
                    for(j=pOldBlobList->GetBlobNum(); j>0; j--)
                    {
                        CvBlob* pBOld = pOldBlobList->GetBlob(j-1); // toz takhle udelam prekryv, intersection
                        if((fabs(pBOld->x-pB->x) < (CV_BLOB_RX(pBOld)+CV_BLOB_RX(pB))) &&
                           (fabs(pBOld->y-pB->y) < (CV_BLOB_RY(pBOld)+CV_BLOB_RY(pB))))
                        {   /* Intersection detected, delete blob from list: */
                            Blobs.DelBlob(i-1);
                            break;
                        }
                    }   /* Check next old blob. */
                }   /*  if pOldBlobList. */
            }   /*  Check next blob. */
        }   /*  Delete small and intersected blobs. */


        {   /* Bubble-sort blobs by size: */
            int N = Blobs.GetBlobNum();
            int i,j;
            for(i=1; i<N; ++i)
            {
                for(j=i; j>0; --j)
                {
                    CvBlob  temp;
                    float   AreaP, AreaN;
                    CvBlob* pP = Blobs.GetBlob(j-1);
                    CvBlob* pN = Blobs.GetBlob(j);
                    AreaP = CV_BLOB_WX(pP)*CV_BLOB_WY(pP);
                    AreaN = CV_BLOB_WX(pN)*CV_BLOB_WY(pN);
                    if(AreaN < AreaP)break;
                    temp = pN[0];
                    pN[0] = pP[0];
                    pP[0] = temp;
                }
            }

            /* Copy only first 10 blobs: */ // P3k - vezme ty nejvetsi, co se neprekryvaly se staryma, ale proc 10????
            for(i=0; i<MIN(N,10); ++i)
            {
                m_pBlobLists[0]->AddBlob(Blobs.GetBlob(i));
            }

            // TODO: tady musim zajistit, aby tu byl muj blob z Blobs!

        }   /* Sort blobs by size. */

        cvReleaseMemStorage(&storage);

    }   /* Create blobs. */

    {   /* Shift each track: */ // tohle posune bloby tak, aby nejnovejsi byl vzdy [0].
        int j;
        for(j=0; j<m_TrackNum; ++j)
        {
            int     i;
            DefSeq* pTrack = m_TrackSeq+j;

            for(i=SEQ_SIZE-1; i>0; --i)
                pTrack->pBlobs[i] = pTrack->pBlobs[i-1];

            pTrack->pBlobs[0] = NULL;
            if(pTrack->size == SEQ_SIZE)pTrack->size--;
        }
    }   /* Shift each track. */

    printf("a");

    /* Analyze blob list to find best blob trajectory: */ // tady se nejde ten novy 0. blob
    {
        double      BestError = -1;
        int         BestTrack = -1;;
        CvBlobSeq*  pNewBlobs = m_pBlobLists[0];
        int         NewTrackNum = 0;

        for(int i=pNewBlobs->GetBlobNum(); i>0; --i)    // prirad bloby ke starym trajektoriim
        {
            CvBlob* pBNew = pNewBlobs->GetBlob(i-1);
            int     j;
            int     AsignedTrack = 0;
            for(j=0; j<m_TrackNum; ++j)
            {
                double  dx,dy;
                DefSeq* pTrack = m_TrackSeq+j;
                CvBlob* pLastBlob = pTrack->size>0?pTrack->pBlobs[1]:NULL;
                if(pLastBlob == NULL) continue;
                dx = fabs(CV_BLOB_X(pLastBlob)-CV_BLOB_X(pBNew));
                dy = fabs(CV_BLOB_Y(pLastBlob)-CV_BLOB_Y(pBNew));
                if(dx > 2*CV_BLOB_WX(pLastBlob) || dy > 2*CV_BLOB_WY(pLastBlob)) continue;
                AsignedTrack++;

                if(pTrack->pBlobs[0]==NULL)
                {   /* Fill existed track: */
                    pTrack->pBlobs[0] = pBNew;
                    pTrack->size++;
                }
                else if((m_TrackNum+NewTrackNum)<SEQ_NUM)
                {   /* Duplicate existed track: */
                    m_TrackSeq[m_TrackNum+NewTrackNum] = pTrack[0];
                    m_TrackSeq[m_TrackNum+NewTrackNum].pBlobs[0] = pBNew;
                    NewTrackNum++;
                }
            }   /* Next track. */

            if(AsignedTrack==0 && (m_TrackNum+NewTrackNum)<SEQ_NUM )    // kdyz nebyla stara, tak udela novou
            {   /* Initialize new track: */
                m_TrackSeq[m_TrackNum+NewTrackNum].size = 1;
                m_TrackSeq[m_TrackNum+NewTrackNum].pBlobs[0] = pBNew;
                NewTrackNum++;
            }
        }   /* Next new blob. */

        m_TrackNum += NewTrackNum;


        /* Check each track: */
        for(int i=0; i<m_TrackNum; ++i)
        {
            int     Good = 1;
            DefSeq* pTrack = m_TrackSeq+i;
            CvBlob* pBNew = pTrack->pBlobs[0];
            if(pTrack->size != SEQ_SIZE) continue; // tohle zpracuje jen ty dlouhe 10... TODO: zjistit, co kdyz je to jinak
            if(pBNew == NULL ) continue;

            /* Check intersection last blob with existed: */
            if(Good && pOldBlobList)
            {
                int k;
                for(k=pOldBlobList->GetBlobNum(); k>0; --k)
                {
                    CvBlob* pBOld = pOldBlobList->GetBlob(k-1);
                    if((fabs(pBOld->x-pBNew->x) < (CV_BLOB_RX(pBOld)+CV_BLOB_RX(pBNew))) && // tohle je test prekryti jako v clanku IBM 2003
                       (fabs(pBOld->y-pBNew->y) < (CV_BLOB_RY(pBOld)+CV_BLOB_RY(pBNew))))
                        Good = 0;
                }
            }   /* Check intersection last blob with existed. */

            /* Check distance to image border: */
            if(Good)
            {   /* Check distance to image border: */
                float    dx = MIN(pBNew->x,S.width-pBNew->x)/CV_BLOB_RX(pBNew);
                float    dy = MIN(pBNew->y,S.height-pBNew->y)/CV_BLOB_RY(pBNew);
                if(dx < m_MinDistToBorder || dy < m_MinDistToBorder) Good = 0;
            }   /* Check distance to image border. */

            /* Check uniform motion: */
            if(Good)
            {   /* Check uniform motion: */
                double      Error = 0;
                int         N = pTrack->size;
                CvBlob**    pBL = pTrack->pBlobs;
                float       sum[2] = {0,0};
                float       jsum[2] = {0,0};
                float       a[2],b[2]; /* estimated parameters of moving x(t) = a*t+b*/
                int         j;

                for(j=0; j<N; ++j)
                {
                    float   x = pBL[j]->x;
                    float   y = pBL[j]->y;
                    sum[0] += x;
                    jsum[0] += j*x;
                    sum[1] += y;
                    jsum[1] += j*y;
                }

                a[0] = 6*((1-N)*sum[0]+2*jsum[0])/(N*(N*N-1));
                b[0] = -2*((1-2*N)*sum[0]+3*jsum[0])/(N*(N+1));
                a[1] = 6*((1-N)*sum[1]+2*jsum[1])/(N*(N*N-1));
                b[1] = -2*((1-2*N)*sum[1]+3*jsum[1])/(N*(N+1));

                for(j=0; j<N; ++j)
                {
                    Error +=
                        pow(a[0]*j+b[0]-pBL[j]->x,2)+
                        pow(a[1]*j+b[1]-pBL[j]->y,2);
                }

                Error = sqrt(Error/N);

                if( Error > S.width*0.01 ||
                    fabs(a[0])>S.width*0.1 ||
                    fabs(a[1])>S.height*0.1)
                    Good = 0;

                /* New best trajectory: */
                if(Good && (BestError == -1 || BestError > Error))
                {   /* New best trajectory: */
                    BestTrack = i;
                    BestError = Error;
                }   /* New best trajectory. */
            }   /*  Check uniform motion. */ // ???
        }   /*  Next track. */


#ifdef _DEBUG_EBD_ALL
        {   /**/
            printf("BlobDetector configurations (%d) = %d [", frame_no, m_TrackNum);
            int i;
            for(i=0; i<SEQ_SIZE; ++i)
            {
                printf("%d,",m_pBlobLists[i]?m_pBlobLists[i]->GetBlobNum():0);
            }
            printf("]\n");
        }
#endif
        printf("s");

// P3k
#ifdef _iLIDS
        // add the SS (starter track) blobs at the end of the blob list (just ONE)
        // tady ji strcim i kdyz neni Best... tak, jak dole, pokud je BestTrack, tak ho nech byt

        int starters_time = (starters.GetBlobNum() > 0)? starters.GetBlob(starters.GetBlobNum()-1)->ID : -999;
        if ((frame_no >= starters_time) && (frame_no <= starters_time+1)) {

            // projdi pOldBlobList... zjisti, jestli tam neni nejaky slusny overlapping (ob1) -spocitej best
            // debug nutny, abych zjistil, jak je to tam ulozene
            int bestOlda = -1;
            float bestOverlap = 0;

            // check the next blobs according to the previous
            // TODO: report an error if a new SS object lost
            for(int i=pOldBlobList->GetBlobNum(); i>0; --i)
            {
                CvBlob* pBOld = pOldBlobList->GetBlob(i-1);
                if (!pBOld) continue; // NULL - SIGSEQ?
                pBOld->ID; // DEBUG

                // musi se prekryvat aspon tak z 15%
                float overlap = Overlaps(starters.GetBlob(starters.GetBlobNum()-1), pBOld);
                
                if (overlap > bestOverlap) {                 // nasli?
                    bestOverlap = overlap;
                    bestOlda = i-1;
                }
            }

            // ale jo, nasli
            if (bestOverlap >= 0.1) { // TODO: dat  0.15 a i to se mi zda hodne malo
                result = 1;
#ifdef _DEBUG_EBD
                printf("Starter blob was found as a track %d at frame %d\n", pOldBlobList->GetBlob(bestOlda)->ID+1, starters_time);
#endif
            }
            else {
                bestOlda = -1;
                bestOverlap = 0;
            }

            // check each (temporary) track for iLIDS starter
            int bestNew = -1;
            bestOverlap = 0;
            if (bestOlda < 0) {
                for (int i=0; i<m_TrackNum; ++i) {
                    if(m_TrackSeq[i].pBlobs[0] == NULL) continue; // toz todle uz je pridane nebo nejak jinak smazane

                    // projdi ten track a rekni nakolik se prekryvaji
                    float overlap = 0;
                    int reverse = starters.GetBlobNum()-1;
                    for (int b = 0; b < starters.GetBlobNum() && b < (m_TrackSeq[i].size/VPSTEP); ++b) {
                        CvBlob* pBNew = m_TrackSeq[i].pBlobs[b*VPSTEP]; // jdi ob VPSTEP
                        if (pBNew == NULL) continue;
                        overlap += Overlaps(starters.GetBlob(reverse-b), pBNew);
                    }

                    if (overlap > bestOverlap) {                 // nasli?
                        bestOverlap = overlap;
                        bestNew = i;
                    }
                }
    
                if (bestOverlap > 0.1) { // no, neco jsme nasli, ale nejspis zadna slava, polni trava
                    result = 1;
#ifdef _DEBUG_EBD
                    printf("New starter blob found at frame %d\n", starters_time);
#endif
                    if (bestNew != BestTrack) { // ale ta metoda tez, tak se na to vykasli, prida se nize, asi takto
                        // assert(m_TrackSeq[i].size == SEQ_SIZE);  // toz tohle je tam naco?
                        // assert(m_TrackSeq[i].pBlobs[0]);         // todle uz tam je
                        pNewBlobList->AddBlob(m_TrackSeq[bestNew].pBlobs[0]);
                        m_TrackSeq[bestNew].pBlobs[0] = NULL;
                        m_TrackSeq[bestNew].size--;
                    }
                }
            }

            // nenasli... ajajajaj, zahlas chybu a rezignuj na to
            if (bestOlda < 0 && bestNew < 0) {
                fprintf(stderr, "Error! Starter blob NOT FOUND at video %s frame %d!\n\n", m_param_video_file_name, starters_time);
            }
        }
#endif // P3k _iLIDS

        // jestli ten (nebo nejaky jiny) nasel sam
        if(BestTrack >= 0)
        {   /* Put new blob to output and delete from blob list: */
            assert(m_TrackSeq[BestTrack].size == SEQ_SIZE);
            assert(m_TrackSeq[BestTrack].pBlobs[0]);
            pNewBlobList->AddBlob(m_TrackSeq[BestTrack].pBlobs[0]);
            m_TrackSeq[BestTrack].pBlobs[0] = NULL;
            m_TrackSeq[BestTrack].size--;
            result = 1;
        }   /* Put new blob to output and mark in blob list to delete. */
    }   /*  Analyze blob list to find best blob trajectory. */

    {   /* Delete bad tracks: */
        int i;
        for(i=m_TrackNum-1; i>=0; --i)
        {   /* Delete bad tracks: */
            if(m_TrackSeq[i].pBlobs[0]) continue;
            if(m_TrackNum>0)
                m_TrackSeq[i] = m_TrackSeq[--m_TrackNum];
        }   /* Delete bad tracks: */
    }

#ifdef USE_OBJECT_DETECTOR
    if( m_split_detector && pNewBlobList->GetBlobNum() > 0 )
    {
        int num_new_blobs = pNewBlobList->GetBlobNum();
        int i = 0;

        if( m_roi_seq ) cvClearSeq( m_roi_seq );
        m_DEBUG_EBD_blob_seq.Clear();
        for( i = 0; i < num_new_blobs; ++i )
        {
            CvBlob* b = pNewBlobList->GetBlob(i);
            CvMat roi_stub;
            CvMat* roi_mat = 0;
            CvMat* scaled_roi_mat = 0;

            CvDetectedBlob d_b = cvDetectedBlob( CV_BLOB_X(b), CV_BLOB_Y(b), CV_BLOB_WX(b), CV_BLOB_WY(b), 0 );
            m_DEBUG_EBD_blob_seq.AddBlob(&d_b);

            float scale = m_param_roi_scale * m_min_window_size.height / CV_BLOB_WY(b);

            float b_width =   MAX(CV_BLOB_WX(b), m_min_window_size.width / scale)
                            + (m_param_roi_scale - 1.0F) * (m_min_window_size.width / scale)
                            + 2.0F * m_max_border / scale;
            float b_height = CV_BLOB_WY(b) * m_param_roi_scale + 2.0F * m_max_border / scale;

            CvRect roi = cvRectIntersection( cvRect( cvFloor(CV_BLOB_X(b) - 0.5F*b_width),
                                                     cvFloor(CV_BLOB_Y(b) - 0.5F*b_height),
                                                     cvCeil(b_width), cvCeil(b_height) ),
                                             cvRect( 0, 0, pImg->width, pImg->height ) );
            if( roi.width <= 0 || roi.height <= 0 )
                continue;

            if( m_roi_seq ) cvSeqPush( m_roi_seq, &roi );

            roi_mat = cvGetSubRect( pImg, &roi_stub, roi );
            scaled_roi_mat = cvCreateMat( cvCeil(scale*roi.height), cvCeil(scale*roi.width), CV_8UC3 );
            cvResize( roi_mat, scaled_roi_mat );

            m_detected_blob_seq.Clear();
            m_split_detector->Detect( scaled_roi_mat, &m_detected_blob_seq );
            cvReleaseMat( &scaled_roi_mat );

            for( int k = 0; k < m_detected_blob_seq.GetBlobNum(); ++k )
            {
                CvDetectedBlob* b = (CvDetectedBlob*) m_detected_blob_seq.GetBlob(k);

                /* scale and shift each detected blob back to the original image coordinates */
                CV_BLOB_X(b) = CV_BLOB_X(b) / scale + roi.x;
                CV_BLOB_Y(b) = CV_BLOB_Y(b) / scale + roi.y;
                CV_BLOB_WX(b) /= scale;
                CV_BLOB_WY(b) /= scale;

                CvDetectedBlob d_b = cvDetectedBlob( CV_BLOB_X(b), CV_BLOB_Y(b), CV_BLOB_WX(b), CV_BLOB_WY(b), 1,
                        b->response );
                m_DEBUG_EBD_blob_seq.AddBlob(&d_b);
            }

            if( m_detected_blob_seq.GetBlobNum() > 1 )
            {
                /*
                 * Split blob.
                 * The original blob is replaced by the first detected blob,
                 * remaining detected blobs are added to the end of the sequence:
                 */
                CvBlob* first_b = m_detected_blob_seq.GetBlob(0);
                CV_BLOB_X(b)  = CV_BLOB_X(first_b);  CV_BLOB_Y(b)  = CV_BLOB_Y(first_b);
                CV_BLOB_WX(b) = CV_BLOB_WX(first_b); CV_BLOB_WY(b) = CV_BLOB_WY(first_b);

                for( int j = 1; j < m_detected_blob_seq.GetBlobNum(); ++j )
                {
                    CvBlob* detected_b = m_detected_blob_seq.GetBlob(j);
                    pNewBlobList->AddBlob(detected_b);
                }
            }
        }   /* For each new blob. */

        for( i = 0; i < pNewBlobList->GetBlobNum(); ++i )
        {
            CvBlob* b = pNewBlobList->GetBlob(i);
            CvDetectedBlob d_b = cvDetectedBlob( CV_BLOB_X(b), CV_BLOB_Y(b), CV_BLOB_WX(b), CV_BLOB_WY(b), 2 );
            m_DEBUG_EBD_blob_seq.AddBlob(&d_b);
        }
    }   // if( m_split_detector )
#endif

    printf("%d ", result);
    return result;

}   /* cvDetectNewBlob */


#ifdef _iLIDS
/**
 * Saves file name and parses the dataset, camera and video IDs
 * If no valid conversion is performed, zeros are returned.
 * Process i-LIDS MCTR data... *[MCT_TR_02/MCTTR02a/]MCTTR0201a.mov.deint.mpeg.txt
 * @param pFileName
 */
void CvBlobDetectorCCiLIDS::SetFileName(char* pFileName)
{
    m_param_video_file_name = pFileName;

    // find the file name (not the path) of the form MCTTR0201a.mov.deint.mpeg.txt
    char* video_str = strrchr(pFileName, '/');
    if (video_str == NULL) {    // nenasli
        video_str = pFileName;
    }
    else video_str++;
#ifdef _DEBUG_EBD
    printf("DEBUG EBD: Proper video name recognized as %s\n", video_str);
#endif

    // find the dataset, camera, video IDs (if no valid conversion is performed, zeros are returned)
    dataset_id = atoi(substring(video_str, 5, 2));
    camera_id = atoi(substring(video_str, 7, 2));
    video_id = -(int)'a'+1 + (int)(video_str[9]);
    if (dataset_id <= 0 || camera_id <= 0 || video_id <= 0) {
        fprintf(stderr,"Warning! EBD cannot recognize dataset %d, camera %d and video %d IDs\n", dataset_id, camera_id, video_id);
    }
#ifdef _DEBUG_EBD
    printf("DEBUG EBD: Dataset %d, camera %d and video %d IDs were recognized\n", dataset_id, camera_id, video_id);
#endif

    // find starter tracks in the database
    FindStarters();
/*
    CvBlob* starter = new CvBlob;
    // 2, 2, 1, 8, 127, '(648, 142)', '(68, 166)'
    starter->ID = 121;
    starter->x  = 648;
    starter->y  = 142;
    starter->w  = 68;
    starter->h  = 166;
    starters.AddBlob(starter);

    // 2, 2, 1, 8, 129, '(646, 140)', '(68, 166)
    starter->ID = 125;
    starter->x  = 646;
    starter->y  = 140;
    starter->w  = 68;
    starter->h  = 166;
    starters.AddBlob(starter);
*/
};


void CvBlobDetectorCCiLIDS::FindStarters() {
    // SELECT "time", "position"[0], "position"[1], size[0], size[1]
    // FROM sunar.evaluation_states
    // WHERE dataset=10 AND camera=2 AND video=1 AND "position" IS NOT NULL
    // ORDER BY track, "time"
    // LIMIT 5;

    // Make a connection to the database
    PGconn* conn = PQconnectdb(CONNINFO);
    if (PQstatus(conn) != CONNECTION_OK) {
        fprintf(stderr, "Error! Connection to database failed: %s", PQerrorMessage(conn));
        return;
    }

    char dataset_str[32]; itoa(dataset_id, dataset_str);
    char camera_str[32]; itoa(camera_id, camera_str);
    char video_str[32]; itoa(video_id, video_str);

    // Eg. "select * from pg_database"
    String startersQuery = String("SELECT \"time\", \"position\"[0], \"position\"[1], size[0], size[1] \n") +
            " FROM ONLY sunar.evaluation_states \n" +
            " WHERE dataset=" + dataset_str +" AND camera="+ camera_str +" AND video="+ video_str +" AND \"position\" IS NOT NULL \n" +
            " ORDER BY track, \"time\" \n" +
            " LIMIT 5;";
    // printf("\n%s\n\n", startersQuery.c_str());

    // results
    PGresult* startersRes = PQexec(conn, startersQuery.c_str());
    if (PQresultStatus(startersRes) != PGRES_TUPLES_OK || PQntuples(startersRes) == 0) {
        printf("NULL: (no point) (no size) found\n");
        starters.Clear();   // J4fun
        return;     // there is nothing to do here
    }

    //
    // go through annotations (videos, tracks and states)
    //
    for (int sj = 0; sj < PQntuples(startersRes); sj++) {

        CvBlob* starter = new CvBlob;
        starter->ID = atoi(PQgetvalue(startersRes, sj, 0));
        starter->x  = atoi(PQgetvalue(startersRes, sj, 1));
        starter->y  = atoi(PQgetvalue(startersRes, sj, 2));
        starter->w  = atoi(PQgetvalue(startersRes, sj, 3));
        starter->h  = atoi(PQgetvalue(startersRes, sj, 4));
        starters.AddBlob(starter);

        printf("%d: (%3.0f,%3.0f), (%3.0f,%3.0f)\n", starter->ID, starter->x, starter->y, starter->w, starter->h);
    }

    PQclear(startersRes);
    PQfinish(conn);
}

#endif // _iLIDS

/**
 * Counts the overlapping ratio
 * Count intersection to union ratio
 * Idea: correlation = (A1 * A2) / (A1 + A2) = SI / SU
 *
 * @param blob1
 * @param blob2
 * @return ratio of intersection to union of two rectangles
 */
float CvBlobDetectorCCiLIDS::Overlaps(CvBlob* blob1, CvBlob* blob2)
{
        // count 4 distances
	float x1 = (blob1->x + blob1->w/2) - (blob2->x - blob2->w/2);
	float x2 = (blob2->x + blob2->w/2) - (blob1->x - blob1->w/2);
	float y1 = (blob1->y + blob1->h/2) - (blob2->y - blob2->h/2);
	float y2 = (blob2->y + blob2->h/2) - (blob1->y - blob1->h/2);

#ifdef _DEBUG_EBD
        printf("t%d-%d x%3.0f-%3.0f y%3.0f-%3.0f ", blob1->ID, blob2->ID, blob1->x, blob2->x, blob1->y, blob2->y);
#endif
        // if it doesnt overlap, return 0
	if ((x1 <= 0) || (x2 <= 0) || (y1 <= 0) || (y2 <= 0)) {
#ifdef _DEBUG_EBD
            printf("not overlapping\n");
#endif
            return 0;
        }

	// count square of intersection SI
        float xd, yd;
        
        if (x1 < x2) xd = x1;
        else xd = x2;

        if (y1 < y2) yd = y1;
        else yd = y2;

	// fix if in center (xd/yd)
	if (xd > blob1->w) xd = blob1->w;
        else if (xd > blob2->w) xd = blob2->w;
	if (yd > blob1->h) yd = blob1->h;
        else if (yd > blob2->h) yd = blob2->h;

	float si = xd*yd;

	// count square of union SU
	float s1 = (blob1->w*blob1->h);
	float s2 = (blob2->w*blob2->h);
	float su = s1 + s2 - si;

#ifdef _DEBUG_EBD
        printf("si%3.0f / su%3.0f-si = %3.2f\n", si, s1 + s2, si/su);
#endif
	// return the ratio of squares
	return si / su;
}

/**
 * CompareContour
 * @param a
 * @param b
 * @param
 * @return
 */
int CvBlobDetectorCCiLIDS::CompareContour(const void* a, const void* b, void* )
{
    float           dx,dy;
    float           h,w,ht,wt;
    CvPoint2D32f    pa,pb;
    CvRect          ra,rb;
    CvSeq*          pCA = *(CvSeq**)a;
    CvSeq*          pCB = *(CvSeq**)b;
    ra = ((CvContour*)pCA)->rect;
    rb = ((CvContour*)pCB)->rect;
    pa.x = ra.x + ra.width*0.5f;
    pa.y = ra.y + ra.height*0.5f;
    pb.x = rb.x + rb.width*0.5f;
    pb.y = rb.y + rb.height*0.5f;
    w = (ra.width+rb.width)*0.5f;
    h = (ra.height+rb.height)*0.5f;

    dx = (float)(fabs(pa.x - pb.x)-w);
    dy = (float)(fabs(pa.y - pb.y)-h);

    //wt = MAX(ra.width,rb.width)*0.1f;
    wt = 0;
    ht = MAX(ra.height,rb.height)*0.3f;
    return (dx < wt && dy < ht);
}


/**
 * cvFindBlobsByCCClasters
 * @param pImg
 * @param pFG
 * @param pBlobs
 * @param storage
 */
void CvBlobDetectorCCiLIDS::cvFindBlobsByCCClasters(IplImage* pImg, IplImage* pFG, CvBlobSeq* pBlobs, CvMemStorage* storage)
{   /* Create contours: */
    IplImage*       pIB = NULL;
    CvSeq*          cnt = NULL;
    CvSeq*          cnt_list = cvCreateSeq(0,sizeof(CvSeq),sizeof(CvSeq*), storage );
    CvSeq*          clasters = NULL;
    int             claster_cur, claster_num;

    pIB = cvCloneImage(pFG);
    cvThreshold(pIB,pIB,128,255,CV_THRESH_BINARY);
    cvFindContours(pIB,storage, &cnt, sizeof(CvContour), CV_RETR_EXTERNAL);
    cvReleaseImage(&pIB);

    /* Create cnt_list.      */
    /* Process each contour: */
    for(; cnt; cnt=cnt->h_next)
    {
        cvSeqPush( cnt_list, &cnt);
    }

    claster_num = cvSeqPartition( cnt_list, storage, &clasters, CompareContour, NULL );

    for(claster_cur=0; claster_cur<claster_num; ++claster_cur)
    {
        int         cnt_cur;
        CvBlob      NewBlob;
        double      M00,X,Y,XX,YY; /* image moments */
        CvMoments   m;
        CvRect      rect_res = cvRect(-1,-1,-1,-1);
        CvMat       mat;

        for(cnt_cur=0; cnt_cur<clasters->total; ++cnt_cur)
        {
            CvRect  rect;
            CvSeq*  cnt;
            int k = *(int*)cvGetSeqElem( clasters, cnt_cur );
            if(k!=claster_cur) continue;
            cnt = *(CvSeq**)cvGetSeqElem( cnt_list, cnt_cur );
            rect = ((CvContour*)cnt)->rect;

            if(rect_res.height<0)
            {
                rect_res = rect;
            }
            else
            {   /* Unite rects: */
                int x0,x1,y0,y1;
                x0 = MIN(rect_res.x,rect.x);
                y0 = MIN(rect_res.y,rect.y);
                x1 = MAX(rect_res.x+rect_res.width,rect.x+rect.width);
                y1 = MAX(rect_res.y+rect_res.height,rect.y+rect.height);
                rect_res.x = x0;
                rect_res.y = y0;
                rect_res.width = x1-x0;
                rect_res.height = y1-y0;
            }
        }

        if(rect_res.height < 1 || rect_res.width < 1)
        {
            X = 0;
            Y = 0;
            XX = 0;
            YY = 0;
        }
        else
        {
            cvMoments( cvGetSubRect(pFG,&mat,rect_res), &m, 0 );
            M00 = cvGetSpatialMoment( &m, 0, 0 );
            if(M00 <= 0 ) continue;
            X = cvGetSpatialMoment( &m, 1, 0 )/M00;
            Y = cvGetSpatialMoment( &m, 0, 1 )/M00;
            XX = (cvGetSpatialMoment( &m, 2, 0 )/M00) - X*X;
            YY = (cvGetSpatialMoment( &m, 0, 2 )/M00) - Y*Y;
        }
        NewBlob = cvBlob(rect_res.x+(float)X,rect_res.y+(float)Y,(float)(4*sqrt(XX)),(float)(4*sqrt(YY)));
        pBlobs->AddBlob(&NewBlob);

    }   /* Next cluster. */

#ifdef _DEBUG_EBD
    {   // Debug info:
        IplImage* pI = cvCreateImage(cvSize(pFG->width,pFG->height),IPL_DEPTH_8U,3);
        cvConvertScale(pImg, pI, 0.3, 30); // P3k (c) 2010
        // cvZero(pI);
        cvCopy(pImg, pI, pFG); // copy where not masked

        // P3k frame info
        CvFont font;
        cvInitFont( &font, CV_FONT_HERSHEY_PLAIN, 0.9, 0.9, 0, 1, CV_AA );
        char str[1024];
        sprintf(str, "%03d", frame_no);
        cvPutText(pI, str, cvPoint(5, 15), &font, cvYellow);

        // print clusters
        for(claster_cur=0; claster_cur<claster_num; ++claster_cur)
        {
            int         cnt_cur;
            CvScalar    color = CV_RGB(rand()%256,rand()%256,rand()%256);

            // P3k this paints the contours
            for(cnt_cur=0; cnt_cur<clasters->total; ++cnt_cur)
            {
                CvSeq*  cnt;
                int k = *(int*)cvGetSeqElem( clasters, cnt_cur );
                if(k!=claster_cur) continue;
                cnt = *(CvSeq**)cvGetSeqElem( cnt_list, cnt_cur );
                cvDrawContours( pI, cnt, color, color, 0, 1, 8);
            }

            // P3k this paints the cluster ellipses
            CvBlob* pB = pBlobs->GetBlob(claster_cur);
            int x = cvRound(CV_BLOB_RX(pB)), y = cvRound(CV_BLOB_RY(pB));
            cvEllipse( pI,
                cvPointFrom32f(CV_BLOB_CENTER(pB)),
                cvSize(MAX(1,x), MAX(1,y)),
                0, 0, 360,
                color, 1 );

            // P3k this paints the cluster numbers
            sprintf(str, "%03d", frame_no);
            cvPutText(pI, str, cvPointFrom32f(CV_BLOB_CENTER(pB)), &font, color);
        }

#ifdef _iLIDS
        // if there are any starters
        if (starters.GetBlobNum() > 0) {
            int starters_time = starters.GetBlob(starters.GetBlobNum()-1)->ID;
            if ((frame_no >= starters.GetBlob(0)->ID -1) && (frame_no <= starters_time+1)) { // inside starters

                for (int i = 0; i < starters.GetBlobNum(); i++) {
                    CvBlob* pB = starters.GetBlob(i);

                    // jestli se ma kreslit v tomhle case
                    if ((frame_no >= pB->ID -2) && (frame_no <= pB->ID +2)) { // VPSTEP je asi tak 2 a anotace asi tak po 5
                        cvRectangle(pI, cvPoint(cvRound(pB->x - pB->w*0.5), cvRound(pB->y - pB->h*0.5)), cvPoint(cvRound(pB->x + pB->w*0.5), (pB->y + pB->h*0.5)), cvGreen, 2, CV_AA);
                        break;
                    }
                }
            }
        }
#endif

        cvNamedWindow( "Clusters", 0);
        cvShowImage( "Clusters", pI);

        cvReleaseImage(&pI);

    }   /* Debug info. */
#endif

}   /* cvFindBlobsByCCClasters */
