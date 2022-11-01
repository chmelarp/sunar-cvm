/*
 * File:   blobtrack.h
 * Author: chmelarp
 *         ipesek
 *
 * Created on 21. květen 2009, 16:59
 */

#ifndef BLOBTRACK_H
#define	BLOBTRACK_H

// the _DEBUG (and _GUI) preprocessor definitions are specified in the project properties (Debug only)
// #define _DEBUG_BT
//#define _GUI

#include "abbrevs.h"

#include "opencv2/video/background_segm.hpp"
#include "opencv2/legacy/blobtrack.hpp"
#include "opencv2/legacy/legacy.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc_c.h>

#include <stdio.h>

/* Select appropriate case insensitive string comparison function: */
#if defined WIN32 || defined _MSC_VER
  #define MY_STRNICMP strnicmp
  #define MY_STRICMP stricmp
#else
  #define MY_STRNICMP strncasecmp
  #define MY_STRICMP strcasecmp
#endif

CvBlobDetector* cvCreateBlobDetectorCCiLIDS();
CvBlobTrackGen* cvCreateModuleBlobTrackGenFeatures();

/* List of foreground (FG) DETECTION modules: */
static CvFGDetector* cvCreateFGDetector0      () { return cvCreateFGDetectorBase(CV_BG_MODEL_FGD,        NULL); }
static CvFGDetector* cvCreateFGDetector0Simple() { return cvCreateFGDetectorBase(CV_BG_MODEL_FGD_SIMPLE, NULL); }
static CvFGDetector* cvCreateFGDetector1      () { return cvCreateFGDetectorBase(CV_BG_MODEL_MOG,        NULL); }

typedef struct DefModule_FGDetector
{
    CvFGDetector* (*create)();
    const char* nickname;
    const char* description;
} DefModule_FGDetector;

DefModule_FGDetector FGDetector_Modules[] =
{
    {cvCreateFGDetector0,"FG_0","Foreground Object Detection from Videos Containing Complex Background. ACM MM2003."},
    {cvCreateFGDetector0Simple,"FG_0S","Simplified version of FG_0"},
    {cvCreateFGDetector1,"FG_1","Adaptive background mixture models for real-time tracking. CVPR1999"},
    {NULL,NULL,NULL}
};

/* List of BLOB DETECTION modules: */
typedef struct DefModule_BlobDetector
{
    CvBlobDetector* (*create)();
    const char* nickname;
    const char* description;
} DefModule_BlobDetector;

DefModule_BlobDetector BlobDetector_Modules[] =
{
    {cvCreateBlobDetectorCCiLIDS,"BD_CCiLIDS","Detect new blob by tracking CC of FG mask; including the iLIDS devel&eval strar frames"},
    {cvCreateBlobDetectorCC,"BD_CC","Detect new blob by tracking CC of FG mask"},
    {cvCreateBlobDetectorSimple,"BD_Simple","Detect new blob by uniform moving of connected components of FG mask"},
    {NULL,NULL,NULL}
};

/* List of BLOB TRACKING modules: */
typedef struct DefModule_BlobTracker
{
    CvBlobTracker* (*create)();
    const char* nickname;
    const char* description;
} DefModule_BlobTracker;

DefModule_BlobTracker BlobTracker_Modules[] =
{
    {cvCreateBlobTrackerCCMSPF,"CCMSPF","connected component tracking and MSPF resolver for collision"},
    {cvCreateBlobTrackerCC,"CC","Simple connected component tracking"},
    {cvCreateBlobTrackerMS,"MS","Mean shift algorithm "},
    {cvCreateBlobTrackerMSFG,"MSFG","Mean shift algorithm with FG mask using"},
    {cvCreateBlobTrackerMSPF,"MSPF","Particle filtering based on MS weight"},
    {NULL,NULL,NULL}
};

/* List of BLOB TRAJECTORY GENERATION modules: */
typedef struct DefModule_BlobTrackGen
{
    CvBlobTrackGen* (*create)();
    const char* nickname;
    const char* description;
} DefModule_BlobTrackGen;

DefModule_BlobTrackGen BlobTrackGen_Modules[] =
{
    {cvCreateModuleBlobTrackGenFeatures,"Features","Generate track record in database (sunar.tracks) including features (sunar.states)"},
    {cvCreateModuleBlobTrackGenYML,"YML","Generate track record in YML format as synthetic video data"},
    {cvCreateModuleBlobTrackGen1,"RawTracks","Generate raw track record (x,y,sx,sy),()... in each line"},
    {NULL,NULL,NULL}
};

/* List of BLOB TRAJECTORY POST PROCESSING modules: */
typedef struct DefModule_BlobTrackPostProc
{
    CvBlobTrackPostProc* (*create)();
    const char* nickname;
    const char* description;
} DefModule_BlobTrackPostProc;

DefModule_BlobTrackPostProc BlobTrackPostProc_Modules[] =
{
    {cvCreateModuleBlobTrackPostProcKalman,"Kalman","Kalman filtering of blob position and size"},
    {NULL,"None","No post processing filter"},
//    {cvCreateModuleBlobTrackPostProcTimeAverRect,"TimeAverRect","Average by time using rectangle window"},
//    {cvCreateModuleBlobTrackPostProcTimeAverExp,"TimeAverExp","Average by time using exponential window"},
    {NULL,NULL,NULL}
};

/* List of BLOB TRAJECTORY ANALYSIS modules: */
CvBlobTrackAnalysis* cvCreateModuleBlobTrackAnalysisDetector();

typedef struct DefModule_BlobTrackAnalysis
{
    CvBlobTrackAnalysis* (*create)();
    const char* nickname;
    const char* description;
} DefModule_BlobTrackAnalysis;

DefModule_BlobTrackAnalysis BlobTrackAnalysis_Modules[] =
{
    {NULL,"None","No trajectory analiser"},
    {cvCreateModuleBlobTrackAnalysisHistPVS,"HistPVS","Histogram of 5D feature vector analysis (x,y,vx,vy,state)"},
    {cvCreateModuleBlobTrackAnalysisHistP,"HistP","Histogram of 2D feature vector analysis (x,y)"},
    {cvCreateModuleBlobTrackAnalysisHistPV,"HistPV","Histogram of 4D feature vector analysis (x,y,vx,vy)"},
    {cvCreateModuleBlobTrackAnalysisHistSS,"HistSS","Histogram of 4D feature vector analysis (startpos,endpos)"},
    {cvCreateModuleBlobTrackAnalysisTrackDist,"TrackDist","Compare tracks directly"},
    {cvCreateModuleBlobTrackAnalysisIOR,"IOR","Integrator (by OR operation) of several analysers "},
    {NULL,NULL,NULL}
};

/* List of Blob Trajectory ANALYSIS modules: */
/*================= END MODULES DECRIPTION ===================================*/

#endif	/* BLOBTRACK_H */

