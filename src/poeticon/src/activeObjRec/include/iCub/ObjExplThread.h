
#ifndef __OBJEXPL_THREAD_H__
#define __OBJEXPL_THREAD_H__

#include <iostream>
#include <vector>
#include <map>

#include <opencv2/core/core.hpp>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#include "iCub/KeyframeExtractor.h"
#include "iCub/FeatureExtractor.h"
#include "iCub/ViewSphere.h"
#include "iCub/GlobalSettings.h"
#include "iCub/Object.h"

class ObjRecModule;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

class ObjExplThread : public yarp::os::RateThread
{
public:
    ObjExplThread( 
            ObjRecModule *module_,
            int period,
            PolyDriver &armCartDriver_,
            ResourceFinder *rf_,
            bool isSimulation_) : 
        RateThread(period),
        armCartDriver(armCartDriver_),
        rf(rf_), 
        isSimulation(isSimulation_), 
        module(module_)
    {}

    inline void setObjectName(const std::string &name) { objectName = name; }

    virtual bool threadInit();     
    virtual void threadRelease();
    virtual void run(); 

private:
    static const int NUM_JOINTS = 16;
    double DISTINCT_VIEW_THRESH;
    
    std::string moduleName;
    GlobalSettings settings;

    std::string objectName;
    
    ObjRecModule *module;

    ResourceFinder *rf;

    PolyDriver &armCartDriver;
    ICartesianControl *armCart;

    BufferedPort<Bottle> explorationPort;
    
    bool isSimulation;

    ViewSphere explorationMap;
    Object newObject;
    int explProgress;
    double explorationGain;
    double stopExplorationThresh;
    
    std::vector<cv::Mat> keyframeFeatures;
    std::vector<cv::Vec2f> explViewpoints;
    std::vector<cv::Mat> explKeyframes;

    std::string objectViewDir;

    double gazeOffsetX;
    double gazeOffsetY;
    double gazeOffsetZ;

    std::vector<cv::Vec2f> explPath;
    
    KeyframeExtractor keyframeExtractor;

    void exploreObject();
    
    
};

#endif


