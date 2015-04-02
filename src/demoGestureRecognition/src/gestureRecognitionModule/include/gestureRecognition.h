#include <string>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>

#include <yarp/math/Math.h>
// Windows Header Files:
#include <windows.h>
#include <ole2.h>
// C RunTime Header Files
#include <stdlib.h>
#include <malloc.h>
#include <memory.h>
#include <tchar.h>
#include "NuiApi.h"
#include "NuiImageCamera.h"
#include "NuiSensor.h"
#include "NuiSkeleton.h"
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>
#include <cxcore.h>
#include <iCub/ctrl/math.h>

#include "Recognizer.h"
#include "ModelSVM.h"

#define VOCAB_CMD_ACK           VOCAB3('a','c','k')
#define VOCAB_CMD_NACK          VOCAB4('n','a','c','k')
#define VOCAB_CMD_REC           VOCAB3('r','e','c')
#define VOCAB_CMD_SAVE          VOCAB4('s','a','v','e')
#define VOCAB_CMD_STOP          VOCAB4('s','t','o','p')
#define VOCAB_CMD_SET           VOCAB3('s','e','t')
#define COLOR_WIDTH 640 
#define COLOR_HEIGHT 480
#define DEPTH_WIDTH 320
#define DEPTH_HEIGHT 240
#define SKELETON_WIDTH 320
#define SKELETON_HEIGHT 240
#define CHANNEL 1

class GestRecognition: public yarp::os::RFModule
{
    Recognizer recognizer;
    yarp::os::Port rpc;
    yarp::os::Port out;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > outImage;
    CvVideoWriter * writer;
    CvVideoWriter * Skeletonwriter;
    yarp::os::Semaphore* mutex;

    IplImage* color;
    IplImage* depthCurrentF;
    IplImage* depthPrevious;
    IplImage* RGBCurrent;
    IplImage* RGBsmallCurrent;
    IplImage* RGBsmallPrevious;
    IplImage* skeleton;
    IplImage* foo;
    IplImage* r;
    IplImage* g;
    IplImage* b;
    IplImage* rightHand;
    IplImage* leftHand;
    IplImage* remappedDepth;
    IplImage* visualizeFlow;
    IplImage * imgT;

    float buf[DEPTH_WIDTH*DEPTH_HEIGHT*CHANNEL];
    int nbins;
    double minimum;
    double maximum;
    std::vector<double> edges;
    int dimension;
    float thModFlow;
    int tol;
    int player;

    //HRESULT hr;
    HANDLE h1,h2,h3,h4;

    std::vector<cv::Point3f> flow3D;
    std::vector<Vector4> flow2D;
    std::vector<cv::Point3f> handPositions3D;
    std::vector<cv::Point2f> handPositions2D;

    cv::Mat Mapper;
    cv::Mat optFlow;
    std::vector<double> bodyHist;
    std::vector<double> leftHist;
    std::vector<double> rightHist;
    std::vector<double> flowHist;
    yarp::sig::Vector scores;
    ModelSVM *Classifiers;

    yarp::sig::Vector frameFeatures; 
    int featuresSize;
    int nFeatures;
    int frameFeatureSize;
    std::deque<yarp::sig::Vector> featuresBuffer;

    std::string outDir;
    std::string context;
    int frameNumber;
    int bufferCount;
    int countFile;
    ofstream featuresFile;
    int SVMBuffer;

    bool showflow;
    bool save;
    bool saveVideo;
    bool rec;
    bool initC;
    bool initD;
    bool init;
    bool rightInImage;
    bool leftInImage;
    bool seatedMode;

    double frameC;
    double frameD;

private:
    void computeContrastandOrientation(IplImage* img, IplImage* arctan, IplImage* contrast, double contrastTh=0.0, int aperture=-1);
    void calibrateDepthColor(IplImage* depth, IplImage* depthMapped);
    void computeEdges();
    void computeFlowDense(IplImage* previous, IplImage* current, cv::Mat &optFlow);
    void computeFlowHistogram();
    void computeHOG(IplImage* image, std::vector<double> &currHist, double Th=0., int aperture=-1, bool show=false);
    void computeSceneFlow(IplImage* previousMask,IplImage* currentMask, cv::Mat &optFlow);
    int drawColor(HANDLE h,IplImage* color,const NUI_IMAGE_FRAME * pImageFrame);
    int drawDepth(HANDLE h,IplImage* depth, const NUI_IMAGE_FRAME * pImageFrame );
    void drawMotionField(IplImage* imgMotion, int xSpace, int ySpace, float cutoff, float multiplier, CvScalar color);
    int findNearest(Vector4 &currPoint, std::vector<cv::Point3f> &handPositions3D);
    void generateMatIndex(double num, int &index);
    Vector4 retrieve3DPoint(IplImage* depth, int u,int v, bool remap=true);
    const NUI_IMAGE_FRAME * retrieveImg(HANDLE h);
    void saveFeatures(yarp::sig::Vector &Features);
    void segmentHand(IplImage * depth, IplImage* resR, IplImage* resL, std::vector<cv::Point2f> &handPositions2D, std::vector<cv::Point3f> &handPositions3D,IplImage* skel=NULL);
    void thresholdBW(IplImage *img, IplImage* res, float distance, float metersTh);
    int trackSkeleton(IplImage* skeleton);
    void recognize();
    void saveClassifierScores(const yarp::sig::Vector &scores);

    bool checkHand(cv::Point3f &wrist, cv::Point3f &elbow, cv::Point3f &shoulder);

public:

    GestRecognition();
    bool configure(yarp::os::ResourceFinder &rf);
    bool close();
    bool updateModule();
    double getPeriod();
    bool respond(const yarp::os::Bottle& cmd, yarp::os::Bottle& reply);
};
