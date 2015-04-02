

#ifndef __VISUO_THREAD__
#define __VISUO_THREAD__



#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Semaphore.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>

#include <cv.h>

#include <string>
#include <deque>
#include <map>

#include "utils.h"


#define LEFT        0
#define RIGHT       1

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


struct StereoTracker
{
    Vector      vec;
    int         side;
};

class VisuoThread: public RateThread
{
private:
    ResourceFinder              &rf;

    Resource                    &res;


    Port                                    outPort[2];
    BufferedPort<ImageOf<PixelRgb> >        imgPort[2];
    BufferedPort<Bottle>                    mCUTPort[2];
    BufferedPort<Vector>                    pftInPort;
    Port                                    pftOutPort;         //send template to the pft throught this port

    //MIL Ports
    Port                                    boundMILPort;
    Port                                    cmdMILPort;
    BufferedPort<Bottle>                    recMILPort;

    //multiSensoryObjectRecognition Ports
    Port                                    cmdMSRPort;
    BufferedPort<Bottle>                    recMSRPort;

    Semaphore                   imgMutex;
    Semaphore                   motMutex;
    Semaphore                   MILMutex;
    Semaphore                   trackMutex;

    unsigned int                minMotionBufSize;
    unsigned int                minTrackBufSize;
    unsigned int                maxTrackBufSize;
    double                      timeTol;
    double                      motionStdThresh;
    double                      speedStdThresh;
    double                      stereoDistThresh;

    int                         dominant_eye;
    ImageOf<PixelRgb>           *img[2];
    ImageOf<PixelBgr>           tpl;

    StereoTracker               stereoTracker;
    deque<Vector>               trackBuffer;
    bool                        tracking;

    bool                        newImage[2];

    bool                        show;

    struct Item
    {
        double  t;
        double  size;
        CvPoint p;
    };

    deque<Item>                 buffer[2];
    //map<string,CvPoint>         locations[2];
    map<string,CvPoint>         locations;




    void close();
    void updateImages();
    void updateLocationsMIL();
    void updateMotionCUT();
    void updatePFTracker();
public:
    VisuoThread(ResourceFinder &_rf, Resource &_res);

    virtual bool threadInit();
    virtual void run();
    virtual void threadRelease();


    bool isTracking()
    {
        trackMutex.wait();
        bool track=tracking;
        trackMutex.post();

        return track;
    }

    bool checkTracker(Vector *vec);

    void startTracker(const Vector &stereo, const int &side);
    void restartTracker();


    Vector getFixation();
    Vector getMotion();
    Vector getTrack();
    Vector getObject(const string &obj_name);

    Bottle recogMSR(string &obj_name);


    void doShow()
    {
        show=!show;
    }



    bool startLearningMIL(const string &obj_name);
    bool suspendLearningMIL();
    bool resumeLearningMIL();
    bool trainMIL();

    bool startLearningMSR(const string &obj_name, const int &arm);
    bool startRecogMSR(const string &obj_name, const int &arm);
    bool suspendLearningMSR();
    bool resumeLearningMSR();
    bool stopMSR();
    void checkDoneMSR(bool &done);


};

#endif
