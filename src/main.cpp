
#include "blobtrack.h"

/* Run pipeline on all frames: */
static int RunBlobTrackingAuto( CvCapture* pCap, CvBlobTrackerAuto* pTracker, char* avi_name, char* fgavi_name = NULL, char* btavi_name = NULL, char* bgfgavi_name = NULL )
{
    int                     OneFrameProcess = 0;
    int                     key;
    int                     FrameNum = 0;
    CvVideoWriter*          pFGAvi = NULL;
    CvVideoWriter*          pBTAvi = NULL;
    CvVideoWriter*          pBGFGAvi = NULL;

    //cvNamedWindow( "FG", 0 ); // see #ifdef _GUI

    /* Main loop: end after ESC*/
    for( FrameNum=0; pCap && (key=cvWaitKey(OneFrameProcess?0:1))!='q';
         FrameNum++)
    {   /* Main loop: */
        IplImage*   pImg  = NULL;
        IplImage*   pMask = NULL;

        if(key!=-1)
        {
            OneFrameProcess = 1;
            if(key=='r')OneFrameProcess = 0;
        }

        pImg = cvQueryFrame(pCap);
        if(pImg == NULL) break;


        /* Process: */
        pTracker->Process(pImg, pMask);

        if (fgavi_name) { // FG AVI - simplified
            if (pTracker->GetFGMask()) { /* Debug FG: */
                IplImage* pFG = pTracker->GetFGMask();
                CvSize S = cvSize(pFG->width, pFG->height);
                static IplImage* pI = NULL;

                if (pI == NULL)pI = cvCreateImage(S, pFG->depth, 3);
                cvCvtColor(pFG, pI, CV_GRAY2BGR);

                if (pFGAvi == NULL) {
                    pFGAvi = cvCreateVideoWriter(fgavi_name, CV_FOURCC('X', 'V', 'I', 'D'), 25/VPSTEP, S);
                }
                cvWriteFrame(pFGAvi, pI);

#ifdef _GUI
                cvNamedWindow("FG", 0);
                cvShowImage("FG", pI);
#endif
                // cvWaitKey(20);
            } /* Debug FG. */
        } // FG AVI - useless after P3k combined them below :)


        /* Draw debug info: */
        if (pImg) { /* Draw all information about test sequence: */
            char str[1024];
            int line_type = CV_AA; // Change it to 8 to see non-antialiased graphics.
            CvFont font;
            int i;

            // prepare background and foreground images
            IplImage* pI = cvCreateImage(cvSize(pImg->width, pImg->height), pImg->depth, 3);
            // background: cvCvtColor(pTracker->GetFGMask(), pI, CV_GRAY2BGR);
            cvConvertScale(pImg, pI, 0.3, 30); // P3k (c) 2010
            // foreground: IplImage* pI = cvCloneImage(pImg);
            cvCopy(pImg, pI, pTracker->GetFGMask()); // copy where not masked

            // NEW: bgfg image is stored here!
            if (bgfgavi_name && pI) {
                CvSize S = cvSize(pI->width, pI->height);
                if (pFGAvi == NULL) {
                    pFGAvi = cvCreateVideoWriter(bgfgavi_name, CV_FOURCC('X', 'V', 'I', 'D'), 25/VPSTEP, S);
                }
                cvWriteFrame(pFGAvi, pI);
            }

            // frame info
            cvInitFont( &font, CV_FONT_HERSHEY_PLAIN, 0.9, 0.9, 0, 1, line_type );
            sprintf(str, "%03d", FrameNum);
            cvPutText(pI, str, cvPoint(5, 15), &font, cvYellow);

            cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 0.7, 0.7, 0, 1, line_type);
            // projdi bloby
            for (i = pTracker->GetBlobNum(); i > 0; i--) {

                CvBlob* pB = pTracker->GetBlob(i - 1);

                cvRectangle(pI, cvPoint(cvRound(pB->x - pB->w*0.7), cvRound(pB->y - pB->h*0.7)), cvPoint(cvRound(pB->x + pB->w*0.7), (pB->y + pB->h*0.7)), cvGreen, 1, CV_AA);
                // cvEllipse(pI, p, s, 0, 0, 360, CV_RGB(c, 255 - c, 0), cvRound(1 + (3 * 0) / 255), CV_AA, 8);

                sprintf(str, "%03d", CV_BLOB_ID(pB)+1); // zaciname od 1 (SQL)
                cvPutText(pI, str, cvPoint(cvRound(pB->x - pB->w*0.7), cvRound(pB->y - pB->h*0.7-1)), &font, cvGreen);
                
                // TODO: never? - just for classification purposes by OpenCV...
                const char* pS = pTracker->GetStateDesc(CV_BLOB_ID(pB));
                if (pS) {
                    CvPoint p = cvPoint(cvRound(pB->x * 256), cvRound(pB->y * 256));
                    CvSize s = cvSize(MAX(1, cvRound(CV_BLOB_RX(pB)*256)), MAX(1, cvRound(CV_BLOB_RY(pB)*256)));
                    int c = cvRound(255 * pTracker->GetState(CV_BLOB_ID(pB)));

                    CvSize TextSize;
                    p.x >>= 8;
                    p.y >>= 8;
                    s.width >>= 8;
                    s.height >>= 8;
                    cvGetTextSize(str, &font, &TextSize, NULL);
                    p.y -= s.height;
                    
                    char* pStr = strdup(pS);
                    char* pStrFree = pStr;

                    while (pStr && strlen(pStr) > 0) {
                        char* str_next = strchr(pStr, '\n');

                        if (str_next) {
                            str_next[0] = 0;
                            str_next++;
                        }

                        p.y += TextSize.height + 1;
                        cvPutText(pI, pStr, p, &font, CV_RGB(0, 255, 255));
                        pStr = str_next;
                    }
                    free(pStrFree);
                }
            } /* Next blob. */;
#ifdef _GUI
            cvNamedWindow("Tracking", 0);
            cvShowImage("Tracking", pI);
#endif
            key  = cvWaitKey(OneFrameProcess ? 0 : 20);

            if (btavi_name && pI) { /* Save to avi file: */
                CvSize S = cvSize(pI->width, pI->height);
                if (pBTAvi == NULL) {
                    pBTAvi = cvCreateVideoWriter(btavi_name, CV_FOURCC('X', 'V', 'I', 'D'), 25/VPSTEP, S);
                }
                cvWriteFrame(pBTAvi, pI);
            }

            cvReleaseImage(&pI);
        } /* Draw all information about test sequence. */

        if (key == ' ' || key == 'b') {
            OneFrameProcess = 1;
        }
        else if (key == 'c') {
            OneFrameProcess = 0;
        }


        printf("OK\n"); // log

    } /*  Main loop. */

    printf("\n%s: ALL OK\n", avi_name);

    if(pFGAvi)cvReleaseVideoWriter( &pFGAvi );
    if(pBTAvi)cvReleaseVideoWriter( &pBTAvi );
    if(pBGFGAvi)cvReleaseVideoWriter( &pBGFGAvi );
    return 0;
}   /* RunBlobTrackingAuto */

/* Read parameters from command line
 * and transfer to specified module:
 */
static void set_params(int argc, char* argv[], CvVSModule* pM, const char* prefix, const char* module)
{
    int prefix_len = (int)strlen(prefix);
    int i;
    for(i=0; i<argc; ++i)
    {
        int j;
        char* ptr_eq = NULL;
        int   cmd_param_len=0;
        char* cmd = argv[i];
        if(MY_STRNICMP(prefix,cmd,prefix_len)!=0) continue;
        cmd += prefix_len;
        if(cmd[0]!=':')continue;
        cmd++;

        ptr_eq = strchr(cmd,'=');
        if(ptr_eq)
            cmd_param_len = (int)(ptr_eq-cmd);

        for(j=0; ; ++j)
        {
            int     param_len;
            const char*   param = pM->GetParamName(j);
            if(param==NULL) break;
            param_len = (int)strlen(param);
            if(cmd_param_len!=param_len) continue;
            if(MY_STRNICMP(param,cmd,param_len)!=0) continue;
            cmd+=param_len;
            if(cmd[0]!='=')continue;
            cmd++;
            pM->SetParamStr(param,cmd);
            printf("%s:%s param set to %g\n",module,param,pM->GetParam(param));
        }
    }

    pM->ParamUpdate();

}   /* set_params */

/* Print all parameter values for given module: */
static void print_params(CvVSModule* pM, const char* module, const char* log_name)
{
    FILE* log = log_name?fopen(log_name,"at"):NULL;
    int i;
    if(pM->GetParamName(0) == NULL ) return;


    printf("%s(%s) module parameters:\n",module,pM->GetNickName());
    if(log)
        fprintf(log,"%s(%s) module parameters:\n",module,pM->GetNickName());

    for (i=0; ; ++i)
    {
        const char*   param = pM->GetParamName(i);
        const char*   str = param?pM->GetParamStr(param):NULL;
        if(param == NULL)break;
        if(str)
        {
            printf("  %s: %s\n",param,str);
            if(log)
                fprintf(log,"  %s: %s\n",param,str);
        }
        else
        {
            printf("  %s: %g\n",param,pM->GetParam(param));
            if(log)
                fprintf(log,"  %s: %g\n",param,pM->GetParam(param));
        }
    }

    if(log) fclose(log);

}   /* print_params */

int main(int argc, char* argv[])
{   /* Main function: */
    CvCapture*                  pCap = NULL;
    CvBlobTrackerAutoParam1     param = {0,0,0,0,0,0,0,0};
    CvBlobTrackerAuto*          pTracker = NULL;

    //float       scale = 1;
    const char* scale_name = NULL;
    char*       yml_name = NULL;
    char**      yml_video_names = NULL;
    int         yml_video_num = 0;
    char*       avi_name = NULL;
    const char* fg_name = NULL;
    char*       fgavi_name = NULL;
    char*       btavi_name = NULL;
    char*       bgfgavi_name = NULL;
    const char* bd_name = NULL;
    const char* bt_name = NULL;
    const char* btgen_name = NULL;
    const char* btpp_name = NULL;
    const char* bta_name = NULL;
    char*       bta_data_name = NULL;
    char*       track_name = NULL;
    //char*       comment_name = NULL;
    char*       FGTrainFrames = NULL;
    char*       log_name = NULL;
    char*       savestate_name = NULL;
    char*       loadstate_name = NULL;
    const char* bt_corr = NULL;
    DefModule_FGDetector*           pFGModule = NULL;
    DefModule_BlobDetector*         pBDModule = NULL;
    DefModule_BlobTracker*          pBTModule = NULL;
    DefModule_BlobTrackPostProc*    pBTPostProcModule = NULL;
    DefModule_BlobTrackGen*         pBTGenModule = NULL;
    DefModule_BlobTrackAnalysis*    pBTAnalysisModule = NULL;

    cvInitSystem(argc, argv);

    if(argc < 2)
    {   /* Print help: */
        int i;
        printf("blobtrack [fg=<fg_name>] [bd=<bd_name>]\n"
            "          [bt=<bt_name>] [btpp=<btpp_name>]\n"
            "          [bta=<bta_name>\n"
            "          [bta_data=<bta_data_name>\n"
            "          [bt_corr=<bt_corr_way>]\n"
            "          [btgen=<btgen_name>]\n"
            "          [track=<track_file_name>]\n"
            "          [scale=<scale val>] [noise=<noise_name>] [IVar=<IVar_name>]\n"
            "          [FGTrainFrames=<FGTrainFrames>]\n"
            "          [btavi=<avi output on BT>] [fgavi=<avi output on FG>]\n"
            "          [bgfgavi=<avi output on the alpha-enhanced BG/FG>]\n"
            "          <avi_file>\n");

        printf("  <bt_corr_way> is the method of blob position correction for the \"Blob Tracking\" module\n"
            "     <bt_corr_way>=none,PostProcRes\n"
            "  <FGTrainFrames> is number of frames for FG training\n"
            "  <track_file_name> is file name for save tracked trajectories\n"
            "  <bta_data> is file name for data base of trajectory analysis module\n"
            "  <avi_file> is file name of avi to process by BlobTrackerAuto\n");

        puts("\nModules:");
#define PR(_name,_m,_mt)\
        printf("<%s> is \"%s\" module name and can be:\n",_name,_mt);\
        for(i=0; _m[i].nickname; ++i)\
        {\
            printf("  %d. %s",i+1,_m[i].nickname);\
            if(_m[i].description)printf(" - %s",_m[i].description);\
            printf("\n");\
        }

        PR("fg_name",FGDetector_Modules,"FG/BG Detection");
        PR("bd_name",BlobDetector_Modules,"Blob Entrance Detection");
        PR("bt_name",BlobTracker_Modules,"Blob Tracking");
        PR("btpp_name",BlobTrackPostProc_Modules, "Blob Trajectory Post Processing");
        PR("btgen_name",BlobTrackGen_Modules, "Blob Trajectory Generation");
        PR("bta_name",BlobTrackAnalysis_Modules, "Blob Trajectory Analysis");
#undef PR
        return 0;
    }   /* Print help. */

    {   /* Parse arguments: */
        int i;
        for(i=1; i<argc; ++i)
        {
            int bParsed = 0;
            size_t len = strlen(argv[i]);
#define RO(_n1,_n2) if(strncmp(argv[i],_n1,strlen(_n1))==0) {_n2 = argv[i]+strlen(_n1);bParsed=1;};
            RO("fg=",fg_name);
            RO("fgavi=",fgavi_name);
            RO("btavi=",btavi_name);
            RO("bgfgavi=",bgfgavi_name);
            RO("bd=",bd_name);
            RO("bt=",bt_name);
            RO("bt_corr=",bt_corr);
            RO("btpp=",btpp_name);
            RO("bta=",bta_name);
            RO("bta_data=",bta_data_name);
            RO("btgen=",btgen_name);
            RO("track=",track_name);
            //RO("comment=",comment_name);
            RO("FGTrainFrames=",FGTrainFrames);
            RO("log=",log_name);
            RO("savestate=",savestate_name);
            RO("loadstate=",loadstate_name);
#undef RO
            {
                if( strrchr(argv[i],'=') == NULL &&
                    !bParsed &&
                    (len>3))
                {
                    avi_name = argv[i];
                    break;
                }
            }   /* Next argument. */
        }
    }   /* Parse arguments. */

    if(track_name)
    {   /* Set Trajectory Generator module: */
        int i;
        if(!btgen_name)btgen_name=BlobTrackGen_Modules[0].nickname;

        for(i=0; BlobTrackGen_Modules[i].nickname; ++i)
        {
            if(MY_STRICMP(BlobTrackGen_Modules[i].nickname,btgen_name)==0)
                pBTGenModule = BlobTrackGen_Modules + i;
        }
    }   /* Set Trajectory Generato module. */

    /* Initialize postprocessing module if tracker
     * correction by postprocessing is required.
     */
    if(bt_corr && MY_STRICMP(bt_corr,"PostProcRes")!=0 && !btpp_name)
    {
        btpp_name = bt_corr;
        if(MY_STRICMP(btpp_name,"none")!=0)bt_corr = "PostProcRes";
    }

    {   /* Set default parameters for one processing: */
        if(!bt_corr) bt_corr = "none";
        if(!fg_name) fg_name = FGDetector_Modules[0].nickname;
        if(!bd_name) bd_name = BlobDetector_Modules[0].nickname;
        if(!bt_name) bt_name = BlobTracker_Modules[0].nickname;
        if(!btpp_name) btpp_name = BlobTrackPostProc_Modules[0].nickname;
        if(!bta_name) bta_name = BlobTrackAnalysis_Modules[0].nickname;
        if(!scale_name) scale_name = "1";
    }

//    if(scale_name)
  //      scale = (float)atof(scale_name);

    for(pFGModule=FGDetector_Modules; pFGModule->nickname; ++pFGModule)
        if( fg_name && MY_STRICMP(fg_name,pFGModule->nickname)==0 ) break;

    for(pBDModule=BlobDetector_Modules; pBDModule->nickname; ++pBDModule)
        if( bd_name && MY_STRICMP(bd_name,pBDModule->nickname)==0 ) break;

    for(pBTModule=BlobTracker_Modules; pBTModule->nickname; ++pBTModule)
        if( bt_name && MY_STRICMP(bt_name,pBTModule->nickname)==0 ) break;

    for(pBTPostProcModule=BlobTrackPostProc_Modules; pBTPostProcModule->nickname; ++pBTPostProcModule)
        if( btpp_name && MY_STRICMP(btpp_name,pBTPostProcModule->nickname)==0 ) break;

    for(pBTAnalysisModule=BlobTrackAnalysis_Modules; pBTAnalysisModule->nickname; ++pBTAnalysisModule)
        if( bta_name && MY_STRICMP(bta_name,pBTAnalysisModule->nickname)==0 ) break;

    /* Create source video: */
    if(avi_name)
        pCap = cvCaptureFromFile(avi_name);

    if(pCap==NULL)
    {
        printf("Can't open %s file\n",avi_name);
        return -1;
    }


    {   /* Display parameters: */
        int i;
        FILE* log = log_name?fopen(log_name,"at"):NULL;
        if(log)
        {   /* Print to log file: */
            fprintf(log,"\n=== Blob Tracking pipline in processing mode===\n");
            if(avi_name)
            {
                fprintf(log,"AVIFile: %s\n",avi_name);
            }
            fprintf(log,"FGDetector:   %s\n", pFGModule->nickname);
            fprintf(log,"BlobDetector: %s\n", pBDModule->nickname);
            fprintf(log,"BlobTracker:  %s\n", pBTModule->nickname);
            fprintf(log,"BlobTrackPostProc:  %s\n", pBTPostProcModule->nickname);
            fprintf(log,"BlobCorrection:  %s\n", bt_corr);

            fprintf(log,"Blob Trajectory Generator:  %s (%s)\n",
                pBTGenModule?pBTGenModule->nickname:"None",
                track_name?track_name:"none");

            fprintf(log,"BlobTrackAnalysis:  %s\n", pBTAnalysisModule->nickname);
            fclose(log);
        }

        printf("\n=== Blob Tracking pipline in %s mode===\n","processing");
        if(yml_name)
        {
            printf("ConfigFile: %s\n",yml_name);
            printf("BG: %s\n",yml_video_names[0]);
            printf("FG: ");
            for(i=1;i<(yml_video_num);++i){printf("%s",yml_video_names[i]);if((i+1)<yml_video_num)printf("|");};
            printf("\n");
        }
        if(avi_name)
        {
            printf("AVIFile: %s\n",avi_name);
        }
        printf("FGDetector:   %s\n", pFGModule->nickname);
        printf("BlobDetector: %s\n", pBDModule->nickname);
        printf("BlobTracker:  %s\n", pBTModule->nickname);
        printf("BlobTrackPostProc:  %s\n", pBTPostProcModule->nickname);
        printf("BlobCorrection:  %s\n", bt_corr);

        printf("Blob Trajectory Generator:  %s (%s)\n",
            pBTGenModule?pBTGenModule->nickname:"None",
            track_name?track_name:"none");

        printf("BlobTrackAnalysis:  %s\n", pBTAnalysisModule->nickname);

    }   /* Display parameters. */

    {   /* Create autotracker module and its components: */
        param.FGTrainFrames = FGTrainFrames?atoi(FGTrainFrames):0;

        /* Create FG Detection module: */
        param.pFG = pFGModule->create();
        if(!param.pFG)
            puts("Can not create FGDetector module");
        param.pFG->SetNickName(pFGModule->nickname);
        set_params(argc, argv, param.pFG, "fg", pFGModule->nickname);

        /* Create Blob Entrance Detection module: */
        param.pBD = pBDModule->create();
        if(!param.pBD)
            puts("Can not create BlobDetector module");
        param.pBD->SetNickName(pBDModule->nickname);
#ifdef _iLIDS
        param.pBD->SetParamStr("FileName", avi_name);
        // param.pBD ->ParamUpdate(); // this is called in the set_params() next
        printf("%s:FileName param set to %s\n", param.pBD->GetNickName(), param.pBD->GetParamStr("FileName"));
#endif
        set_params(argc, argv, param.pBD, "bd", pBDModule->nickname);

        /* Create blob tracker module: */
        param.pBT = pBTModule->create();
        if(!param.pBT)
            puts("Can not create BlobTracker module");
        param.pBT->SetNickName(pBTModule->nickname);
        set_params(argc, argv, param.pBT, "bt", pBTModule->nickname);

        /* Create blob trajectory generation module: */
        // TODO: Features custom parameters 2B here...
        param.pBTGen = NULL;
        if(pBTGenModule && track_name && pBTGenModule->create)
        {
            param.pBTGen = pBTGenModule->create();
            param.pBTGen->SetFileName(track_name);
        }
        if(param.pBTGen)
        {
            param.pBTGen->SetNickName(pBTGenModule->nickname);
            set_params(argc, argv, param.pBTGen, "btgen", pBTGenModule->nickname);
        }

        /* Create blob trajectory post processing module: */
        param.pBTPP = NULL;
        if(pBTPostProcModule && pBTPostProcModule->create)
        {
            param.pBTPP = pBTPostProcModule->create();
        }
        if(param.pBTPP)
        {
            param.pBTPP->SetNickName(pBTPostProcModule->nickname);
            set_params(argc, argv, param.pBTPP, "btpp", pBTPostProcModule->nickname);
        }

        param.UsePPData = (bt_corr && MY_STRICMP(bt_corr,"PostProcRes")==0);

        /* Create blob trajectory analysis module: */
        param.pBTA = NULL;
        if(pBTAnalysisModule && pBTAnalysisModule->create)
        {
            param.pBTA = pBTAnalysisModule->create();
            param.pBTA->SetFileName(bta_data_name);
        }
        if(param.pBTA)
        {
            param.pBTA->SetNickName(pBTAnalysisModule->nickname);
            set_params(argc, argv, param.pBTA, "bta", pBTAnalysisModule->nickname);
        }

        /* Create whole pipline: */
        pTracker = cvCreateBlobTrackerAuto1(&param);
        if(!pTracker)
            puts("Can not create BlobTrackerAuto");
    }

    {   /* Load states of each module from state file: */
        CvFileStorage* fs = NULL;
        if(loadstate_name)
            fs=cvOpenFileStorage(loadstate_name,NULL,CV_STORAGE_READ);
        if(fs)
        {
            printf("Load states for modules...\n");
            if(param.pBT)
            {
                CvFileNode* fn = cvGetFileNodeByName(fs,NULL,"BlobTracker");
                param.pBT->LoadState(fs,fn);
            }

            if(param.pBTA)
            {
                CvFileNode* fn = cvGetFileNodeByName(fs,NULL,"BlobTrackAnalyser");
                param.pBTA->LoadState(fs,fn);
            }

            if(pTracker)
            {
                CvFileNode* fn = cvGetFileNodeByName(fs,NULL,"BlobTrackerAuto");
                pTracker->LoadState(fs,fn);
            }

            cvReleaseFileStorage(&fs);
            printf("... Modules states loaded\n");
        }
    }   /* Load states of each module. */

    {   /* Print module parameters: */
        struct DefMMM
        {
            CvVSModule* pM;
            const char* name;
        } Modules[] = {
            {(CvVSModule*)param.pFG,"FGdetector"},
            {(CvVSModule*)param.pBD,"BlobDetector"},
            {(CvVSModule*)param.pBT,"BlobTracker"},
            {(CvVSModule*)param.pBTGen,"TrackGen"},
            {(CvVSModule*)param.pBTPP,"PostProcessing"},
            {(CvVSModule*)param.pBTA,"TrackAnalysis"},
            {NULL,NULL}
        };
        int     i;
        for(i=0; Modules[i].name; ++i)
        {
            if(Modules[i].pM)
                print_params(Modules[i].pM,Modules[i].name,log_name);
        }
    }   /* Print module parameters. */

    /* Run pipeline: */
    RunBlobTrackingAuto( pCap, pTracker, avi_name, fgavi_name, btavi_name, bgfgavi_name );

    {   /* Save state and release modules: */
        CvFileStorage* fs = NULL;
        if(savestate_name)
        {
            fs=cvOpenFileStorage(savestate_name,NULL,CV_STORAGE_WRITE);
        }
        if(fs)
        {
            cvStartWriteStruct(fs,"BlobTracker",CV_NODE_MAP);
            if(param.pBT)param.pBT->SaveState(fs);
            cvEndWriteStruct(fs);
            cvStartWriteStruct(fs,"BlobTrackerAuto",CV_NODE_MAP);
            if(pTracker)pTracker->SaveState(fs);
            cvEndWriteStruct(fs);
            cvStartWriteStruct(fs,"BlobTrackAnalyser",CV_NODE_MAP);
            if(param.pBTA)param.pBTA->SaveState(fs);
            cvEndWriteStruct(fs);
            cvReleaseFileStorage(&fs);
        }
        if(param.pBT)cvReleaseBlobTracker(&param.pBT);
        if(param.pBD)cvReleaseBlobDetector(&param.pBD);
        if(param.pBTGen)cvReleaseBlobTrackGen(&param.pBTGen);
        if(param.pBTA)cvReleaseBlobTrackAnalysis(&param.pBTA);
        if(param.pFG)cvReleaseFGDetector(&param.pFG);
        if(pTracker)cvReleaseBlobTrackerAuto(&pTracker);

    }   /* Save state and release modules. */

    if(pCap)
        cvReleaseCapture(&pCap);

    return 0;

}   /* main() */
