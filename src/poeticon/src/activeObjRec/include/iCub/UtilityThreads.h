#ifndef _UTILITY_THREADS_H
#define _UTILITY_THREADS_H

#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;

class ObjRecModule;

class OpenHandThread : public yarp::os::RateThread
{
public:
    OpenHandThread(ObjRecModule *module_, IPositionControl *armCtrl_, IPositionControl *torsoCtrl_,
            IControlLimits *limArm_, IControlLimits *limTorso_, const Vector &startPosition_) : 
        RateThread(50), module(module_), 
        armCtrl(armCtrl_), torsoCtrl(torsoCtrl_), 
        limArm(limArm_), limTorso(limTorso_), 
        startPosition(startPosition_) {}

    virtual bool threadInit()
    {
        gotoStartPos();
        openHand();
        return true;
    }
    virtual void threadRelease()
    {
        armCtrl->stop();
        torsoCtrl->stop();
    }
    virtual void run();

private:
    static const int NUM_ARM_JOINTS = 7;
    static const int NUM_JOINTS = 16;

    ObjRecModule *module;

    IPositionControl *armCtrl;
    IPositionControl *torsoCtrl;
    IControlLimits *limArm;
    IControlLimits *limTorso;

    Vector startPosition;

    void gotoStartPos();
    void openHand();
};


class CalibGazeThread : public yarp::os::RateThread
{
public:
    CalibGazeThread(ObjRecModule *module_, ResourceFinder *rf_) : 
        RateThread(50), module(module_), rf(rf_) 
        {}

    virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();

private:
    int sliderPosGazeOffsetX;
    int sliderPosGazeOffsetY;
    int sliderPosGazeOffsetZ;
    int viewWindowOffsetX; 
    int viewWindowOffsetY; 

    ObjRecModule *module;
    ResourceFinder *rf;
};

class HandControlThread : public yarp::os::RateThread
{
public:
    HandControlThread(ObjRecModule *module_) : 
        RateThread(50), module(module_) 
    {}

    virtual bool threadInit();
    virtual void threadRelease();
    virtual void run();

private:

    int sliderPosElevation;
    int sliderPosRotation;
    int sliderPosHandPosX;
    int sliderPosHandPosY;
    int sliderPosHandPosZ;
    int sliderPosWAngDist;
    int sliderPosWEuclDist;
    int sliderPosWMoveDist;
    int sliderPosKNN;

    ObjRecModule *module; 
};

class CreateObjectThread : public yarp::os::RateThread
{
    public:
        CreateObjectThread(ObjRecModule *module_, BufferedPort<Bottle> &worldPort_, Port &worldPortSync_) : 
            RateThread(50), module(module_), worldPort(worldPort_), worldPortSync(worldPortSync_) 
        {}

        inline double getObjectRotation() const { return rotation; }

        inline void set3DModelParams(const std::string &objectName_, const std::string &textureName_, double rotation_)
        {
            objectName = objectName_;
            textureName = textureName_;
            rotation = rotation_;
        }

        virtual bool threadInit()
        {
            createObject();
        }

        virtual void run();

    private:

        static const double BOX_SIZE;
        static const double COLOR_RED[];
        static const double COLOR_GREEN[];
        static const double COLOR_BLUE[];
        static const double COLOR_YELLOW[];
        static const double COLOR_WHITE[];

        std::string objectName;
        std::string textureName;
        double rotation;

        ObjRecModule *module;
        BufferedPort<Bottle> &worldPort;
        Port &worldPortSync;

        bool createObject();
};

 
#endif

//----- end-of-file --- ( next line intentionally left blank ) ------------------

