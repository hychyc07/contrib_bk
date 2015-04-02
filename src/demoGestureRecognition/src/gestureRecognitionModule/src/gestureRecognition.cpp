#include "gestureRecognition.h"

using namespace std;
using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;

GestRecognition::GestRecognition()
{
    out.open("/GestRecognition/scores");
    outImage.open("/GestRecognition/outImage");
    nbins=64;
    minimum=-1.00001;
    maximum=1.00001;
    thModFlow=(float)0.5;
    tol=50;
    outDir="C:/Users/Utente/Desktop/dataKinect/";
    frameNumber=1;
    bufferCount=1;
    countFile=1;
    showflow=false;
    save=false;
    saveVideo=true;
    rec=true;
    seatedMode=false;
    bodyHist.resize(nbins);
    rightHist.resize(nbins);
    leftHist.resize(nbins);
    writer=NULL;
    Skeletonwriter=NULL;

    mutex=new Semaphore(1);

    color = cvCreateImageHeader(cvSize(COLOR_WIDTH,COLOR_HEIGHT),IPL_DEPTH_8U,4);
    depthCurrentF = cvCreateImageHeader(cvSize(DEPTH_WIDTH,DEPTH_HEIGHT),IPL_DEPTH_32F,CHANNEL);
    depthPrevious = NULL;
    RGBCurrent= cvCreateImage(cvSize(COLOR_WIDTH,COLOR_HEIGHT),IPL_DEPTH_8U,3);
    RGBsmallCurrent= cvCreateImage(cvSize(COLOR_WIDTH/2,COLOR_HEIGHT/2),IPL_DEPTH_8U,3);
    RGBsmallPrevious= NULL;
    skeleton=cvCreateImage(cvSize(COLOR_WIDTH/2,COLOR_HEIGHT/2),IPL_DEPTH_8U,3);
    foo=cvCreateImage(cvSize(COLOR_WIDTH,COLOR_HEIGHT),IPL_DEPTH_8U,1);
    r=cvCreateImage(cvSize(COLOR_WIDTH,COLOR_HEIGHT),IPL_DEPTH_8U,1);
    g=cvCreateImage(cvSize(COLOR_WIDTH,COLOR_HEIGHT),IPL_DEPTH_8U,1);
    b=cvCreateImage(cvSize(COLOR_WIDTH,COLOR_HEIGHT),IPL_DEPTH_8U,1);
    rightHand=cvCreateImage(cvSize(COLOR_WIDTH/2,COLOR_HEIGHT/2),IPL_DEPTH_32F,1);
    leftHand=cvCreateImage(cvSize(COLOR_WIDTH/2,COLOR_HEIGHT/2),IPL_DEPTH_32F,1);
    optFlow.create(COLOR_HEIGHT,COLOR_WIDTH,CV_32FC2);

    remappedDepth=NULL;
    visualizeFlow=NULL;

    init=true;

    cvNamedWindow("color image",CV_WINDOW_AUTOSIZE);
    cvMoveWindow("color image", 160, 100);
    cvNamedWindow("depth image",CV_WINDOW_AUTOSIZE);
    cvMoveWindow("depth image", 510, 100);
    cvNamedWindow("skeleton image", CV_WINDOW_AUTOSIZE);
    cvMoveWindow("skeleton image", 860, 100);
    cvNamedWindow("left hand", CV_WINDOW_AUTOSIZE);
    cvMoveWindow("left hand", 320, 400);
    cvNamedWindow("right hand", CV_WINDOW_AUTOSIZE);
    cvMoveWindow("right hand", 670, 400);

    initC=false;
    initD=false;

    frameC=0;
    frameD=0;
}

bool GestRecognition::configure(ResourceFinder &rf)
{
    context=rf.getContextPath();
    string rpcName=rf.check("rpcName",Value("/gest_rec/rpc")).asString().c_str();
    dimension=rf.check("dimension",Value(5)).asInt();

    player=-1;

    yarp::sig::Vector num(11), den(11);
    num[0]=4.62344856977647e-10;
    num[1]=4.62344856977647e-09;
    num[2]=2.08055185639941e-08;
    num[3]=5.54813828373176e-08;
    num[4]=9.70924199653059e-08;
    num[5]=1.16510903958367e-07;
    num[6]=9.70924199653059e-08;
    num[7]=5.54813828373176e-08;
    num[8]=2.08055185639941e-08;
    num[9]=4.62344856977647e-09;
    num[10]=4.62344856977647e-10;

    den[0]=1.0;
    den[1]=-8.39368255078839;
    den[2]=31.8158646581919;
    den[3]=-71.7026569821430;
    den[4]=106.381617694638;
    den[5]=-108.553825650279;
    den[6]=77.1444606240244;
    den[7]=-37.6960546719428;
    den[8]=12.1197601392885;
    den[9]=-2.31493776310228;
    den[10]=0.199454975553812;
    recognizer.setFilter(num,den);

    computeEdges();

    Mapper.create(DEPTH_HEIGHT,DEPTH_WIDTH,CV_32FC2);

    rpc.open(rpcName.c_str());
    attach(rpc);

    HRESULT hr = NuiInitialize( NUI_INITIALIZE_FLAG_USES_SKELETON| NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX |NUI_INITIALIZE_FLAG_USES_COLOR);
     
    if( hr != S_OK )
    {
         fprintf(stdout,"NuiInitialize failed\n");
         return false;
    }

    h1 = CreateEvent( NULL, TRUE, FALSE, NULL );
    h2 = NULL;
    hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR,NUI_IMAGE_RESOLUTION_640x480, 0, 2, h1, &h2);

    if( FAILED( hr ) )
    {
        fprintf(stdout,"Could not open image stream video\n");
        return false;
    }

    h3 = CreateEvent( NULL, TRUE, FALSE, NULL );
    h4 = NULL;
     
    hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, NUI_IMAGE_RESOLUTION_320x240, 0, 2, h3, &h4);
    
    if(NuiImageStreamSetImageFrameFlags(h4, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE )!=S_OK)
        fprintf(stdout,"NO NEAR MODE\n");
    
    if( FAILED( hr ) )
    {
       fprintf(stdout,"Could not open depth stream video\n");
        return false;
    }

    Classifiers=new ModelSVM(context);
    ResourceFinder ActionModelsFile;
    ActionModelsFile.setDefaultContext("demoGestureRecognition/conf");
    ActionModelsFile.setDefaultConfigFile("SVMModels.ini");
    int argc=0; char *argv[1];
    ActionModelsFile.configure("ICUB_ROOT",argc,argv);

    Bottle& bgeneral=ActionModelsFile.findGroup("GENERAL");
    SVMBuffer=bgeneral.check("bufferSize",Value(-1)).asInt();

    featuresSize=bgeneral.check("dictionarySize",Value(-1)).asInt();
    nFeatures=bgeneral.check("nFeatures",Value(-1)).asInt();

    frameFeatureSize=bgeneral.find("frameFeatureSize").asInt();
    frameFeatures.resize(frameFeatureSize);
    imgT=cvCreateImage(cvSize(COLOR_WIDTH/2,COLOR_HEIGHT/2),IPL_DEPTH_8U,3);

    if(seatedMode)
        NuiSkeletonTrackingEnable(h3,NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT);

    return true;
}

bool GestRecognition::close()
{
    rpc.interrupt();
    rpc.close();

    outImage.interrupt();
    outImage.close();

    out.interrupt();
    out.close();

    cvReleaseImageHeader(&depthCurrentF);
    cvReleaseImageHeader(&color);
    cvReleaseImage(&RGBCurrent);
    cvReleaseImage(&RGBsmallCurrent);
    cvReleaseImage(&skeleton);
    cvReleaseImage(&foo);
    cvReleaseImage(&r);
    cvReleaseImage(&g);
    cvReleaseImage(&b);
    cvReleaseImage(&imgT);

    if(RGBsmallPrevious!=NULL)
        cvReleaseImage(&RGBsmallPrevious);

    if(depthPrevious!=NULL)
        cvReleaseImage(&depthPrevious);

    if(visualizeFlow!=NULL)
        cvReleaseImage(&visualizeFlow);

    if(remappedDepth!=NULL)
        cvReleaseImage(&remappedDepth);

    if(leftHand!=NULL)
        cvReleaseImage(&leftHand);

    if(rightHand!=NULL)
        cvReleaseImage(&rightHand);

    cvDestroyAllWindows();

    NuiShutdown();

    if(writer!=NULL)
        cvReleaseVideoWriter(&writer);

    if(Skeletonwriter!=NULL)
        cvReleaseVideoWriter(&Skeletonwriter);

    delete Classifiers;
    delete mutex;
    return true;
}

bool GestRecognition::updateModule()
{
    const NUI_IMAGE_FRAME *colIm=retrieveImg(h2);
    const NUI_IMAGE_FRAME *depthIm=retrieveImg(h4);

    if(colIm!=NULL)
    {
        /*if(frameC>frameD)
        {
            NuiImageStreamReleaseFrame(h2, colIm);
        }
        else*/
        {
            drawColor(h2,color,colIm);
            cvSplit(color,r,g,b,foo);
            cvMerge(r,g,b,NULL,RGBCurrent);
            initC=true;
            frameC=colIm->dwFrameNumber;
            NuiImageStreamReleaseFrame(h2, colIm);
        }
    }

    if(depthIm!=NULL)
    {
        /*if(frameD>frameC)
        {
            NuiImageStreamReleaseFrame(h4, depthIm);
        }
        else*/
        {
            drawDepth(h4,depthCurrentF,depthIm);
            initD=true;
            frameD=depthIm->dwFrameNumber;
            NuiImageStreamReleaseFrame(h4, depthIm );
        }
    }

    if(initC && initD )//&& abs(frameC-frameD)==0)
    {
        
        cvResize(RGBCurrent,RGBsmallCurrent);
        if(remappedDepth!=NULL)
            cvReleaseImage(&remappedDepth);
        remappedDepth=(IplImage *) cvClone(depthCurrentF);
        //int64 t = getTickCount();
        calibrateDepthColor(depthCurrentF,depthCurrentF);
        //t = getTickCount() - t;
        //printf("Remap Depth: %f ms\n", t*1000/getTickFrequency());
        
        if(init)
        {
            RGBsmallPrevious=(IplImage*) cvClone(RGBsmallCurrent);
            depthPrevious=(IplImage*) cvClone(depthCurrentF);
            init=false;
            return true;
        }

        IplImage* depthU=cvCreateImage(cvSize(COLOR_WIDTH/2,COLOR_HEIGHT/2),IPL_DEPTH_8U,1);
        cvConvertScale(remappedDepth,depthU,255);
        cvMerge(depthU,depthU,depthU,NULL,skeleton);
        trackSkeleton(skeleton);
        

        //t = getTickCount();
        computeFlowDense(RGBsmallPrevious,RGBsmallCurrent,optFlow);
        //t = getTickCount() - t;
        //printf("Compute Flow 2D: %f ms\n", t*1000/getTickFrequency());

        //t = getTickCount();
        computeSceneFlow(depthPrevious,depthCurrentF,optFlow);
        //t = getTickCount() - t;
        //printf("Compute Scene Flow: %f ms\n", t*1000/getTickFrequency());

        //t = getTickCount();
        computeFlowHistogram();
        //t = getTickCount() - t;
        //printf("Compute Flow Histogram: %f ms\n", t*1000/getTickFrequency());

        //showHist1D(bodyHist, nbins, 25, "Hist PGH"); // for visualization


        segmentHand(remappedDepth,rightHand,leftHand,handPositions2D,handPositions3D,skeleton);

        if(leftInImage==false && rightInImage==false)
        {
            int idx=0;

            for (int i=0; i<frameFeatureSize; i++)
            {
                frameFeatures[idx]=0.0;
                idx++;
            }
        }
        else
        {   
            //t = getTickCount();
            computeHOG(rightHand, rightHist,0.0,-1);
            computeHOG(leftHand, leftHist,0.0,-1);
            computeHOG(remappedDepth, bodyHist,0.0,-1);

            int idx=0;
            for (int i=0; i<flowHist.size(); i++)
            {
                frameFeatures[idx]=flowHist[i];
                idx++;
            }

            for (int i=0; i<bodyHist.size(); i++)
            {
                frameFeatures[idx]=bodyHist[i];
                idx++;
            }

            for (int i=0; i<rightHist.size(); i++)
            {
                frameFeatures[idx]=rightHist[i];
                idx++;
            }

            for (int i=0; i<leftHist.size(); i++)
            {
                frameFeatures[idx]=leftHist[i];
                idx++;
            }

            //t = getTickCount() - t;
            //printf("Compute HOGS: %f ms\n", t*1000/getTickFrequency());
        }

        if(bufferCount>SVMBuffer)
             featuresBuffer.pop_front();
        featuresBuffer.push_back(frameFeatures);
        bufferCount++;

        mutex->wait();
        if(bufferCount>SVMBuffer && rec)
        {
            scores.clear();
            Classifiers->computeOutput(featuresBuffer,scores);
            //saveClassifierScores(scores);
            recognize();
        }
        mutex->post();
        

        if(outImage.getOutputCount()>0) {
            cvCvtColor(depthU,imgT,CV_GRAY2BGR);
            ImageOf<PixelBgr>& outim=outImage.prepare();
            outim.wrapIplImage(imgT);
            outImage.write();
        }
        cvReleaseImage(&depthU);

        cvShowImage("right hand",rightHand);
        cvShowImage("left hand",leftHand);
        cvShowImage("depth image",remappedDepth);
        cvShowImage("skeleton image",skeleton);

        cvWaitKey(1);

        mutex->wait();
        if(save)
        {
            if(saveVideo)
            {
                IplImage* depthU=cvCreateImage(cvSize(COLOR_WIDTH/2,COLOR_HEIGHT/2),IPL_DEPTH_8U,1);
                IplImage* depthRGB=cvCreateImage(cvSize(COLOR_WIDTH/2,COLOR_HEIGHT/2),IPL_DEPTH_8U,3);
                cvConvertScale(remappedDepth,depthU,255);
                cvMerge(depthU,depthU,depthU,NULL,depthRGB);
                cvWriteFrame(writer,RGBsmallCurrent);
                cvWriteFrame(Skeletonwriter,skeleton);
                cvReleaseImage(&depthU);
                cvReleaseImage(&depthRGB);
            }
            saveFeatures(frameFeatures);

        }
        mutex->post();

        if(showflow)
        {
            if(visualizeFlow!=NULL)
                cvReleaseImage(&visualizeFlow);
            visualizeFlow=(IplImage*) cvClone(RGBsmallCurrent);
            drawMotionField(visualizeFlow, 7, 7, 0.0, 5, CV_RGB(255,0,0));
        }

        if(showflow)
            cvShowImage("color image",visualizeFlow);
        else
            cvShowImage("color image",RGBsmallCurrent);

        initC=initD=false;

        if(RGBsmallPrevious!=NULL)
            cvReleaseImage(&RGBsmallPrevious);

        if(depthPrevious!=NULL)
            cvReleaseImage(&depthPrevious);

        RGBsmallPrevious=(IplImage *) cvClone(RGBsmallCurrent);
        depthPrevious=(IplImage *) cvClone(depthCurrentF);
    }

    return true;
}

void GestRecognition::recognize()
{
    //scores are the SVM scores at time t
    int start=0;
    int end=0;
    int index=recognizer.recognize(scores,start,end);

    if (index!=-1)
    {
        Bottle winner;
        winner.addInt(index+1);
        out.write(winner);
        cout << "Action " << index+1  << " recognized" << " starting frame: " << start << " end frame: " << end  << endl;
    }
}

double GestRecognition::getPeriod()
{
    return 0.001;
}

bool GestRecognition::respond(const Bottle& cmd, Bottle& reply) 
{
    switch (cmd.get(0).asVocab())
    {
        case VOCAB_CMD_REC:
        {
            mutex->wait();
            rec=true;
            save=false;
            reply.addVocab(VOCAB_CMD_ACK);
            mutex->post();
            return true;
        }
        case VOCAB_CMD_SAVE:
        {
            mutex->wait();
            stringstream o;
            o << countFile;
            countFile=countFile+1;
            string s = o.str();
            string filename=outDir+s+".txt";
            string videoPath=outDir+s+".avi";
            string skelVideoPath=outDir+s+"S.avi";
            if(saveVideo)
            {
                writer=cvCreateVideoWriter(videoPath.c_str(), CV_FOURCC('M','J','P','G'), 10, cvSize(RGBsmallCurrent->width, RGBsmallCurrent->height), 1);
                Skeletonwriter=cvCreateVideoWriter(skelVideoPath.c_str(), CV_FOURCC('M','J','P','G'), 10, cvSize(skeleton->width, skeleton->height), 1);
            }
            featuresFile.open(filename.c_str(), ios_base::trunc); 
            save=true;
            rec=false;
            reply.addVocab(VOCAB_CMD_ACK);
            mutex->post();
            return true;
        }
        case VOCAB_CMD_STOP:
        {
            mutex->wait();
            save=false;
            frameNumber=1;
            featuresFile.close();

            if(saveVideo)
            {
                cvReleaseVideoWriter(&Skeletonwriter);
                cvReleaseVideoWriter(&writer);
            }
            reply.addVocab(VOCAB_CMD_ACK);
            mutex->post();
            return true;
        }
        case VOCAB_CMD_SET:
        {

        }
    }
    reply.addVocab(VOCAB_CMD_NACK);
    return false;
}

void GestRecognition::calibrateDepthColor(IplImage* depth, IplImage* depthMapped)
{
    IplImage* tmpD=(IplImage*) cvClone(depth);
    cvZero(depth);
    Mapper=Mapper.setTo(Scalar(0,0,0,0));
    for (int i=0; i< tmpD->height; i++)
    {
        for (int j=0; j< tmpD->width; j++)
        {
            CvScalar s=cvGet2D(tmpD,i,j);

            if(s.val[0]==0.0)
                continue;
            
            LONG u=j;
            LONG v=i;

            LONG uN;
            LONG vN;
            USHORT realdepth =(USHORT)((s.val[0])*0xFFF8/1.0);

            HRESULT isok=NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(NUI_IMAGE_RESOLUTION_320x240,
            NUI_IMAGE_RESOLUTION_320x240,
            NULL,
            u,
            v,
            realdepth,
            &uN,
            &vN);

            if(isok!=S_OK)
                continue;

            int uNI=(int) uN;
            int vNI=(int) vN;
            if(vNI<depth->height && uNI<depth->width)
            {
                Mapper.ptr<float>(vNI)[2*uNI]=j;
                Mapper.ptr<float>(vNI)[2*uNI +1]=i;
                cvSet2D(depth,vNI,uNI,s);
            }
        }
    }
    cvReleaseImage(&tmpD);
}

void GestRecognition::computeContrastandOrientation(IplImage* img, IplImage* arctan, IplImage* contrast, double contrastTh, int aperture) 
{
    double max =0;
    double min =0;
    IplImage *img_t = cvCreateImage(cvGetSize(img),img->depth,1);
    IplImage *img_f = cvCreateImage(cvGetSize(img),32,1);

    IplImage * derivativeX= cvCreateImage(cvGetSize(img),32,1);
    IplImage * derivativeY= cvCreateImage(cvGetSize(img),32,1);

    if(img->nChannels>1)
        cvCvtColor(img,img_t,CV_RGB2GRAY);
    else
        cvCopy(img,img_t,0);

    cvMinMaxLoc(img_t,&min,&max,NULL,NULL,NULL);
    cvConvertScale(img_t,img_f,1.0/max,0);

    cvSobel(img_f,derivativeX,1,0,aperture);
    cvSobel(img_f,derivativeY,0,1,aperture);

    cvZero(arctan);
    cvZero(contrast);
    cvCartToPolar(derivativeX, derivativeY, contrast, arctan, 0);

    cvThreshold(contrast,contrast,contrastTh,1,CV_THRESH_BINARY);

    cvReleaseImage(&img_f);
    cvReleaseImage(&derivativeX);
    cvReleaseImage(&derivativeY);
    cvReleaseImage(&img_t);
}

void GestRecognition::computeEdges()
{
    edges.push_back(minimum);
    double div=(maximum-minimum)/dimension;
    double temp=minimum;
    for (int i=0; i<dimension-1; i++)
    {
        temp+=div;
        edges.push_back(temp);
    }
    edges.push_back(maximum);
}

void GestRecognition::computeFlowDense(IplImage* previous, IplImage* current, Mat &optFlow)
{
    /*Mat leftPrevGray;
    Mat leftNextGray;
    Mat leftPrev(previous);
    Mat leftNext(current);
    cvtColor(leftPrev,leftPrevGray,CV_RGB2GRAY,1);
    cvtColor(leftNext,leftNextGray,CV_RGB2GRAY,1);
    int flag;
    if(optFlow.empty())
        flag=0;
    else
        flag=OPTFLOW_USE_INITIAL_FLOW;
    calcOpticalFlowFarneback(leftPrevGray,leftNextGray,optFlow,0.25,1,3,2,7,1.5,flag);
*/
    IplImage* velx=cvCreateImage(cvGetSize(previous), IPL_DEPTH_32F, 1);
    IplImage* vely=cvCreateImage(cvGetSize(previous), IPL_DEPTH_32F, 1);
    IplImage* pGray=cvCreateImage(cvGetSize(previous), IPL_DEPTH_8U, 1);
    IplImage* cGray=cvCreateImage(cvGetSize(previous), IPL_DEPTH_8U, 1);
    double lamda=0.5;

    cvCvtColor(previous, pGray, CV_BGR2GRAY);
    cvCvtColor(current, cGray, CV_BGR2GRAY);
    cvCalcOpticalFlowHS(pGray, cGray, 0, velx, vely, lamda, cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 10, 1 ));

    int scale=1;

    if(optFlow.rows==480 && optFlow.cols==640)
        scale=2;

    flow2D.clear();
    for (int i=0; i<previous->height; i++)
    {
        for (int j=0; j<previous->width; j++)
        {
            Vector4 v;
            v.x=(float)j/scale; // prev X
            v.y=(float)i/scale; // prev Y
            v.z=((float*)(velx->imageData + i*velx->widthStep))[j]/scale; //deltaX
            v.w=((float*)(vely->imageData + i*vely->widthStep))[j]/scale; // deltaY
            optFlow.ptr<float>(i)[2*j]=v.z;
            optFlow.ptr<float>(i)[2*j+1]=v.w;

            double distance=sqrt(v.z*v.z + v.w*v.w);

            if(distance>thModFlow)
                flow2D.push_back(v);
            else
            {
                optFlow.ptr<float>(i)[2*j]=0;
                optFlow.ptr<float>(i)[2*j+1]=0;
            }
        }
    }
    cvReleaseImage(&velx);
    cvReleaseImage(&vely);
    cvReleaseImage(&pGray);
    cvReleaseImage(&cGray);
}

void GestRecognition::computeFlowHistogram()
{
    flowHist.clear();

    if (flow3D.size()==0)
    {
        for (int i=0; i<dimension*dimension*dimension; i++)
            flowHist.push_back(0);
        return;
    }

    vector<Mat> matHistogram;
    for (int i=0; i<dimension; i++)
    {
        Mat matrix=Mat::zeros(dimension,dimension,CV_32F);
        matHistogram.push_back(matrix);
    }

    for (unsigned int i=0; i<flow3D.size(); i++)
    {
        int x,y,z;
        double n=sqrt((flow3D.at(i).x*flow3D.at(i).x)+(flow3D.at(i).y*flow3D.at(i).y)+(flow3D.at(i).z*flow3D.at(i).z));
        generateMatIndex((flow3D.at(i).x/n),x);
        generateMatIndex((flow3D.at(i).y/n),y);
        generateMatIndex((flow3D.at(i).z/n),z);
        matHistogram.at(x).ptr<float>(y)[z]+=1;
    }
    
    double normalizer=0;
    for (unsigned int i=0; i<matHistogram.size(); i++)
    {
        for (int j=0; j<dimension; j++)
        {
            for (int k=0; k<dimension; k++)
            {
                flowHist.push_back(matHistogram.at(i).ptr<float>(j)[k]);
                normalizer=normalizer+matHistogram.at(i).ptr<float>(j)[k];
            }
        }
    }

    for (unsigned int i=0; i<flowHist.size(); i++)
        flowHist[i]=flowHist[i]/normalizer;
}

void GestRecognition::computeHOG(IplImage* image, vector<double> &currHist, double Th, int aperture, bool show) 
{
    IplImage * contrastTemplate= cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
    IplImage * arctanTemplate= cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
    IplImage * maskTemplate= cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
    IplImage * edges= cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
    IplImage * img=cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);

    //cvConvertScale(image,img,255.0,0);
    //cvCanny(img,edges,7,7,3);

    computeContrastandOrientation(image,arctanTemplate,contrastTemplate,Th,aperture);

    cvConvertScale(contrastTemplate,maskTemplate,255,0);

    // cvErode(maskTemplate,maskTemplate,0,1);
    //cvThreshold(maskTemplate,maskTemplate,100,255,CV_THRESH_BINARY);

    if(show)
    {
        cvNamedWindow("Contrast",1); 
        cvShowImage("Contrast", maskTemplate);
    }

    int n_bins[]={nbins};
    float range[]={0,2*CV_PI };
    float * ranges[] = {range};
    CvHistogram* histTemp=cvCreateHist(1, n_bins, CV_HIST_ARRAY, ranges,1);
    cvCalcHist(&arctanTemplate, histTemp, 0, maskTemplate);
    cvNormalizeHist(histTemp,1);

    for(int i=0; i<nbins; i++)
    {   
        float bin_value=cvQueryHistValue_1D(histTemp,i);
        currHist[i]=bin_value;
    }

    cvReleaseImage(&contrastTemplate);
    cvReleaseImage(&arctanTemplate);
    cvReleaseImage(&maskTemplate);
    cvReleaseImage(&edges);
    cvReleaseImage(&img);
    cvReleaseHist(&histTemp);
}

void GestRecognition::computeSceneFlow(IplImage* previousMask,IplImage* currentMask, Mat &optFlow)
{
    flow3D.clear();
    Mat maskTempO(previousMask);
    vector<Vector4> filteredFlow;
    for(unsigned int i=0; i<flow2D.size(); i++)
        filteredFlow.push_back(flow2D[i]);

    flow2D.clear();
    Mat maskTempN(currentMask);
    for(unsigned int i=0; i<filteredFlow.size(); i++)
    {
        float x,y,deltaX,deltaY=0;
        x=filteredFlow[i].x;
        y=filteredFlow[i].y;
        deltaX=filteredFlow[i].z;
        deltaY=filteredFlow[i].w;

        if((cvRound(y+deltaY))<0 || (cvRound(x+deltaX))<0 || (cvRound(y+deltaY))>=currentMask->height || (cvRound(x+deltaX))>=currentMask->width  )
        {
            optFlow.ptr<float>(cvRound(y))[2*cvRound(x)]=0;
            optFlow.ptr<float>(cvRound(y))[2*cvRound(x)+1]=0;
            continue;
        }

        if(cvRound(y)<0 || cvRound(x)<0 || cvRound(y)>=previousMask->height || cvRound(x)>=previousMask->width)
        {
            optFlow.ptr<float>(cvRound(y))[2*cvRound(x)]=0;
            optFlow.ptr<float>(cvRound(y))[2*cvRound(x)+1]=0;
            continue;
        }
        float valOld= maskTempO.ptr<float>(cvRound(y))[cvRound(x)];
        float valNew= maskTempN.ptr<float>(cvRound(y+deltaY))[cvRound(x+deltaX)];

        //cout << valOld << " " << valNew << " " << u << " " << v << " " << v+cvRound(flow2D[i].w) << " " << u+cvRound(flow2D[i].z) << endl;
        if (valOld==0 || valNew==0)
        {
            optFlow.ptr<float>(cvRound(y))[2*cvRound(x)]=0;
            optFlow.ptr<float>(cvRound(y))[2*cvRound(x)+1]=0;
            continue;
        }

        Vector4 oldPoint=retrieve3DPoint(previousMask, x,y);
        Vector4 currPoint=retrieve3DPoint(currentMask, (x+deltaX), (y+deltaY));

        Point3f flow;
        flow.x=currPoint.x-oldPoint.x;
        flow.y=currPoint.y-oldPoint.y;
        flow.z=currPoint.z-oldPoint.z;

        if(flow.x>0.001 || flow.y>0.001 || flow.z>0.001)
        {
            flow3D.push_back(flow);
            flow2D.push_back(filteredFlow[i]);
        }
    }
}

int GestRecognition::drawColor(HANDLE h,IplImage* color,const NUI_IMAGE_FRAME * pImageFrame)
{
    INuiFrameTexture  * pTexture=pImageFrame->pFrameTexture;
    NUI_LOCKED_RECT LockedRect;
    pTexture->LockRect( 0, &LockedRect, NULL, 0 );
    if( LockedRect.Pitch != 0 )
    {
         BYTE * pBuffer = (BYTE*) LockedRect.pBits;
         cvSetData(color,pBuffer,LockedRect.Pitch);
    }
    return 0;
}

int GestRecognition::drawDepth(HANDLE h,IplImage* depth, const NUI_IMAGE_FRAME * pImageFrame ) 
{
    INuiFrameTexture  * pTexture=pImageFrame->pFrameTexture;

    NUI_LOCKED_RECT LockedRect;
    pTexture->LockRect( 0, &LockedRect, NULL, 0 );

    if( LockedRect.Pitch != 0 )
    {
        USHORT * pBuff = (USHORT*) LockedRect.pBits;
        /*int k=320*140+190;
        USHORT realDepth = pBuff[k]&0xFFF8;
        LONG x=190;
        LONG y=140;
        Vector4 v = NuiTransformDepthImageToSkeleton(x,y,realDepth,NUI_IMAGE_RESOLUTION_320x240);
        cout << v.x << " " << v.y  << " " << v.z  << " " << ((realDepth>>3)) << endl;*/

        for(int i=0;i<DEPTH_WIDTH*DEPTH_HEIGHT;i++)
        {
            USHORT realDepth = (pBuff[i]&0xFFF8);
            int p=(pBuff[i]&0x0007);
            
            float scale;
            if(p==(player+1) && p>0)
                scale= ((1.0*realDepth/0xFFF8));
            else
                scale=0;
            buf[CHANNEL*i]=scale;
        }

        cvSetData(depth,buf,DEPTH_WIDTH*CHANNEL*4);
    }
    return 0;
}

void GestRecognition::drawMotionField(IplImage* imgMotion, int xSpace, int ySpace, float cutoff, float multiplier, CvScalar color)
{
    CvPoint p0 = cvPoint(0,0);
    CvPoint p1 = cvPoint(0,0);
    float deltaX, deltaY, angle, hyp;

    for(unsigned int i=0; i<flow2D.size(); i++) 
    {
        p0.x = (int)flow2D[i].x;
        p0.y = (int)flow2D[i].y;
        deltaX = flow2D[i].z;
        deltaY = -(flow2D[i].w);
        angle = atan2(deltaY, deltaX);
        hyp = sqrt(deltaX*deltaX + deltaY*deltaY);

        if(hyp > cutoff)
        {
            p1.x = p0.x + cvRound(multiplier*hyp*cos(angle));
            p1.y = p0.y + cvRound(multiplier*hyp*sin(angle));
        //p1.x = p0.x + cvRound(multiplier*cos(angle));
       // p1.y = p0.y + cvRound(multiplier*sin(angle));
            cvLine( imgMotion, p0, p1, color,1, CV_AA, 0);
            p0.x = p1.x + cvRound(3*cos(angle-CV_PI + CV_PI/4));
            p0.y = p1.y + cvRound(3*sin(angle-CV_PI + CV_PI/4));
            cvLine( imgMotion, p0, p1, color,1, CV_AA, 0);

            p0.x = p1.x + cvRound(3*cos(angle-CV_PI - CV_PI/4));
            p0.y = p1.y + cvRound(3*sin(angle-CV_PI - CV_PI/4));
            cvLine( imgMotion, p0, p1, color,1, CV_AA, 0);
        }
    }
}

int GestRecognition::findNearest(Vector4 &currPoint, vector<Point3f> &handPositions3D)
{
    double minDist = 0.2;
    int index=-1;
    double x1=currPoint.x;
    double y1=currPoint.y;
    double z1=currPoint.z;

    for (int i=0; i<handPositions3D.size(); i++)
    {
        double x2=handPositions3D[i].x;
        double y2=handPositions3D[i].y;
        double z2=handPositions3D[i].z;

        double dist=(x1-x2)*(x1-x2);
        dist=dist+(y1-y2)*(y1-y2);
        dist=dist+(z1-z2)*(z1-z2);

        dist=sqrt(dist);

        if(dist<minDist)
        {
            minDist=dist;
            index=i;
        }
    }

    return index;
}

void GestRecognition::generateMatIndex(double num, int &index)
{
    index=-1;
    unsigned int i;
    for (i=0; i<edges.size()-1; i++)
        if((num>=edges.at(i)) && (num<edges.at(i+1)))
            break;
    index=i;
}

Vector4 GestRecognition::retrieve3DPoint(IplImage* depth, int u,int v, bool remap)
{
    if(cvRound(v)<0)
        v=0;
    if(cvRound(u)<0)
        u=0;
    if(cvRound(v)>=depth->height)
        v=(float)depth->height-1;
    if(cvRound(u)>=depth->width)
        u=(float)depth->width-1;

    CvScalar val= cvGet2D(depth,v,u);
    USHORT realdepth =(USHORT)((val.val[0])*0xFFF8/1.0);

    LONG x;
    LONG y;

    if(remap)
    {
        x=(LONG) Mapper.ptr<float>(v)[2*u];
        y= (LONG) Mapper.ptr<float>(v)[2*u+1];
    }
    else
    {
        x=u;
        y=v;
    }
    Vector4 point = NuiTransformDepthImageToSkeleton(x,y,realdepth,NUI_IMAGE_RESOLUTION_320x240);
    return point;
}

const NUI_IMAGE_FRAME * GestRecognition::retrieveImg(HANDLE h) 
{
    const NUI_IMAGE_FRAME * pImageFrame = NULL;
    HRESULT hr = NuiImageStreamGetNextFrame( h, 0, &pImageFrame );

    if( FAILED( hr ) )
        return NULL;
    else
        return pImageFrame;
}

void GestRecognition::saveFeatures(yarp::sig::Vector &Features) 
{
    featuresFile << frameNumber << " ";
    for(unsigned int i=0; i<Features.size(); i++) 
        featuresFile << " " << Features[i];

    featuresFile << "\n";
    frameNumber++;
}

void GestRecognition::segmentHand(IplImage * depth, IplImage* resR, IplImage* resL, vector<Point2f> &handPositions2D, vector<Point3f> &handPositions3D,IplImage* skel)
{
    cvZero(resR);
    cvZero(resL);

    if(handPositions3D.size()==0)
    {
        rightInImage=false;
        leftInImage=false;
        return;
    }

    int indexR=NUI_SKELETON_POSITION_WRIST_RIGHT;
    int indexL=NUI_SKELETON_POSITION_WRIST_LEFT;



    rightInImage=checkHand(handPositions3D[NUI_SKELETON_POSITION_WRIST_RIGHT], handPositions3D[NUI_SKELETON_POSITION_ELBOW_RIGHT],handPositions3D[NUI_SKELETON_POSITION_SHOULDER_RIGHT]);
    leftInImage=checkHand(handPositions3D[NUI_SKELETON_POSITION_WRIST_LEFT], handPositions3D[NUI_SKELETON_POSITION_ELBOW_LEFT],handPositions3D[NUI_SKELETON_POSITION_SHOULDER_LEFT]);
    /*rightInImage=(handPositions2D[indexR].x>=0 && handPositions2D[indexR].x<DEPTH_WIDTH && handPositions2D[indexR].y>=0 && handPositions2D[indexR].y+tol<DEPTH_HEIGHT);
    rightInImage=rightInImage && (handPositions2D[indexR+1].x>=0 && handPositions2D[indexR+1].x<DEPTH_WIDTH && handPositions2D[indexR+1].y>=0 && handPositions2D[indexR+1].y<DEPTH_HEIGHT);

    leftInImage=(handPositions2D[indexL].x>=0 && handPositions2D[indexL].x<DEPTH_WIDTH && handPositions2D[indexL].y>=0 && handPositions2D[indexL].y+tol<DEPTH_HEIGHT);
    leftInImage=leftInImage && (handPositions2D[indexL+1].x>=0 && handPositions2D[indexL+1].x<DEPTH_WIDTH && handPositions2D[indexL+1].y>=0 && handPositions2D[indexL+1].y<DEPTH_HEIGHT);*/

    for (int u=0; u<depth->width; u++)
    {
        for (int v=0; v<depth->height; v++)
        {
            float val=((float*)(depth->imageData + v*depth->widthStep))[u];

            if(val==0.0)
                continue;

            Vector4 currPoint=retrieve3DPoint(depth,u,v,false);
            int currIndex=findNearest(currPoint,handPositions3D);

            if((currIndex==indexR || currIndex==(indexR+1)) && rightInImage)
            {
                ((float*)(resR->imageData + v*resR->widthStep))[u]=val;
                if(skel!=NULL)
                {
                    ((char*)(skel->imageData + v*skel->widthStep))[u*skel->nChannels]=211;
                    ((char*)(skel->imageData + v*skel->widthStep))[u*skel->nChannels+1]=0;
                    ((char*)(skel->imageData + v*skel->widthStep))[u*skel->nChannels+2]=148;
                }
            }

            if((currIndex==indexL || currIndex==(indexL+1)) && leftInImage)
            {
                ((float*)(resL->imageData + v*resL->widthStep))[u]=val;
                if(skel!=NULL)
                {
                    ((char*)(skel->imageData + v*skel->widthStep))[u*skel->nChannels]=255;
                    ((char*)(skel->imageData + v*skel->widthStep))[u*skel->nChannels+1]=255;
                    ((char*)(skel->imageData + v*skel->widthStep))[u*skel->nChannels+2]=0;
                }
            }
        }
    }

    double depthValueH=handPositions3D[indexR+1].z;
    double depthValueW=handPositions3D[indexR].z;
    thresholdBW(depth,resR,(depthValueH),0.1);

    depthValueH=handPositions3D[indexL+1].z;
    depthValueW=handPositions3D[indexL].z;
    thresholdBW(depth,resL,(depthValueH),0.1);
}

void GestRecognition::thresholdBW(IplImage *img, IplImage* res, float distance, float metersTh) 
{
    USHORT mmdist=(USHORT) (distance*1000);
    USHORT mmth=(USHORT) (metersTh*1000);
    mmdist=mmdist<<3;
    mmth=mmth<<3;
    distance=((1.0*mmdist/0xFFF8));
    metersTh=((1.0*mmth/0xFFF8));
    CvSize size =cvGetSize(img);

    IplImage *up= cvCreateImage(size,img->depth,1);
    IplImage * down=cvCreateImage(size,img->depth,1);
    IplImage * mask=cvCreateImage(size,IPL_DEPTH_8U,1);

    cvThreshold(img, up, distance+metersTh, 255, CV_THRESH_TOZERO_INV);
    cvThreshold(img, down, distance-metersTh, 255, CV_THRESH_TOZERO);

    cvConvertScale(res,mask,255,0);
    cvThreshold(mask,mask,0,255,CV_THRESH_BINARY);

    cvAnd(up,down,res,mask);

    cvReleaseImage(&up);
    cvReleaseImage(&down);
    cvReleaseImage(&mask);
}

int GestRecognition::trackSkeleton(IplImage* skeleton)
{
    NUI_SKELETON_FRAME SkeletonFrame;
    CvPoint pt[20];
    handPositions3D.clear();
    handPositions2D.clear();
    HRESULT hr = NuiSkeletonGetNextFrame( 0, &SkeletonFrame );

    bool bFoundSkeleton = false;
    player=-1;
    double closestDist=2000;

    for( int i = 0 ; i < NUI_SKELETON_COUNT ; i++)
    {
        if( SkeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED )
        {
            bFoundSkeleton = true;
            if(SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER].z<closestDist)
            {
                closestDist=SkeletonFrame.SkeletonData[i].SkeletonPositions[NUI_SKELETON_POSITION_SHOULDER_CENTER].z;
                player=i;
            }
        }
    }
     
    if( bFoundSkeleton )
    {
        NuiTransformSmooth(&SkeletonFrame,NULL);
        int i=player;
        {
            if( SkeletonFrame.SkeletonData[i].eTrackingState == NUI_SKELETON_TRACKED )
            {
                for (int j = 0; j < NUI_SKELETON_POSITION_COUNT; j++)
                {
                    float fx,fy;

                    NuiTransformSkeletonToDepthImage( SkeletonFrame.SkeletonData[i].SkeletonPositions[j], &fx, &fy,NUI_IMAGE_RESOLUTION_320x240 );
                    pt[j].x = (int) ( fx);
                    pt[j].y = (int) ( fy);
                    cvCircle(skeleton,pt[j],5,CV_RGB(255,0,0),-1);

                    Point3f sk(SkeletonFrame.SkeletonData[i].SkeletonPositions[j].x,SkeletonFrame.SkeletonData[i].SkeletonPositions[j].y,SkeletonFrame.SkeletonData[i].SkeletonPositions[j].z);
                    handPositions3D.push_back(sk);
                    handPositions2D.push_back(Point2f(pt[j]));
                }

                cvLine(skeleton,pt[NUI_SKELETON_POSITION_HEAD],pt[NUI_SKELETON_POSITION_SHOULDER_CENTER],CV_RGB(0,255,0));
                cvLine(skeleton,pt[NUI_SKELETON_POSITION_HAND_RIGHT],pt[NUI_SKELETON_POSITION_WRIST_RIGHT],CV_RGB(0,255,0));
                cvLine(skeleton,pt[NUI_SKELETON_POSITION_WRIST_RIGHT],pt[NUI_SKELETON_POSITION_ELBOW_RIGHT],CV_RGB(0,255,0));
                cvLine(skeleton,pt[NUI_SKELETON_POSITION_ELBOW_RIGHT],pt[NUI_SKELETON_POSITION_SHOULDER_RIGHT],CV_RGB(0,255,0));
                cvLine(skeleton,pt[NUI_SKELETON_POSITION_SHOULDER_RIGHT],pt[NUI_SKELETON_POSITION_SHOULDER_CENTER],CV_RGB(0,255,0));
                cvLine(skeleton,pt[NUI_SKELETON_POSITION_SHOULDER_CENTER],pt[NUI_SKELETON_POSITION_SHOULDER_LEFT],CV_RGB(0,255,0));
                cvLine(skeleton,pt[NUI_SKELETON_POSITION_SHOULDER_LEFT],pt[NUI_SKELETON_POSITION_ELBOW_LEFT],CV_RGB(0,255,0));
                cvLine(skeleton,pt[NUI_SKELETON_POSITION_ELBOW_LEFT],pt[NUI_SKELETON_POSITION_WRIST_LEFT],CV_RGB(0,255,0));
                cvLine(skeleton,pt[NUI_SKELETON_POSITION_WRIST_LEFT],pt[NUI_SKELETON_POSITION_HAND_LEFT],CV_RGB(0,255,0));
                
                if(!seatedMode)
                {
                    cvLine(skeleton,pt[NUI_SKELETON_POSITION_SHOULDER_CENTER],pt[NUI_SKELETON_POSITION_SPINE],CV_RGB(0,255,0));
                    cvLine(skeleton,pt[NUI_SKELETON_POSITION_SPINE],pt[NUI_SKELETON_POSITION_HIP_CENTER],CV_RGB(0,255,0));
                    cvLine(skeleton,pt[NUI_SKELETON_POSITION_HIP_CENTER],pt[NUI_SKELETON_POSITION_HIP_RIGHT],CV_RGB(0,255,0));
                    cvLine(skeleton,pt[NUI_SKELETON_POSITION_HIP_RIGHT],pt[NUI_SKELETON_POSITION_KNEE_RIGHT],CV_RGB(0,255,0));
                    cvLine(skeleton,pt[NUI_SKELETON_POSITION_KNEE_RIGHT],pt[NUI_SKELETON_POSITION_ANKLE_RIGHT],CV_RGB(0,255,0));
                    cvLine(skeleton,pt[NUI_SKELETON_POSITION_ANKLE_RIGHT],pt[NUI_SKELETON_POSITION_FOOT_RIGHT],CV_RGB(0,255,0));
                    cvLine(skeleton,pt[NUI_SKELETON_POSITION_HIP_CENTER],pt[NUI_SKELETON_POSITION_HIP_LEFT],CV_RGB(0,255,0));
                    cvLine(skeleton,pt[NUI_SKELETON_POSITION_HIP_LEFT],pt[NUI_SKELETON_POSITION_KNEE_LEFT],CV_RGB(0,255,0));
                    cvLine(skeleton,pt[NUI_SKELETON_POSITION_KNEE_LEFT],pt[NUI_SKELETON_POSITION_ANKLE_LEFT],CV_RGB(0,255,0));
                    cvLine(skeleton,pt[NUI_SKELETON_POSITION_ANKLE_LEFT],pt[NUI_SKELETON_POSITION_FOOT_LEFT],CV_RGB(0,255,0));
                }
            }
        }
    }
    return 0;
}

void GestRecognition::saveClassifierScores(const yarp::sig::Vector &scores)
{
    string path=outDir+"/scores.txt";
    ofstream outFile(path.c_str(), ios_base::app);

    for (unsigned int i=0; i<scores.size(); i++)
        outFile << scores[i] <<  " ";

    outFile << endl;
    outFile.close();
}

bool GestRecognition::checkHand(Point3f &wrist, Point3f &elbow, Point3f &shoulder)
{
    bool inImg=false;

    Point2f we;
    we.x=wrist.x-elbow.x;
    we.y=wrist.z-elbow.z;

    double wey=wrist.y-elbow.y;

    Point2f ws;
    ws.x=wrist.x-shoulder.x;
    ws.y=wrist.z-shoulder.z;
    
    double wsy=wrist.y-shoulder.y;


    Point2f es;
    es.x=elbow.x-shoulder.x;
    es.y=elbow.z-shoulder.z;

    double esy=elbow.y-shoulder.y;

    double n1=norm(we);
    double n2=norm(ws);
    double n3=norm(es);

    if((n1>0.20 || n2>0.20 || n3>0.20) || (wey>0 || wsy>0 || esy>0) || abs(wey)<0.05 || abs(wsy)<0.05 || abs(esy)<0.05)
        inImg=true;

    return inImg;

}

