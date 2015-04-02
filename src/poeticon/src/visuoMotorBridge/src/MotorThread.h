


#ifndef __MOTOR_THREAD__
#define __MOTOR_THREAD__

#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/sig/Vector.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/pids.h>
#include <iCub/ctrl/neuralNetworks.h>
#include <iCub/action/actionPrimitives.h>

#include <vector>

#include "utils.h"

#define HEAD_MODE_IDLE              0
#define HEAD_MODE_GO_HOME           1
#define HEAD_MODE_TRACK_HAND        2
#define HEAD_MODE_TRACK_TEMP        3
#define HEAD_MODE_TRACK_FIX         4

#define ARM_MODE_IDLE               0
#define ARM_MODE_LEARN              1
#define ARM_MODE_FINE_REACHING      2
#define ARM_MODE_FLIP_HAND          3

#define GRASP_STATE_IDLE            0
#define GRASP_STATE_ABOVE           1
#define GRASP_STATE_SIDE            2


#define ARM_HOMING_PERIOD           1.5     // [s]


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::action;


class MotorThread: public RateThread
{
private:
    ResourceFinder              &rf;

    Resource                    &res;

    PolyDriver                  *drvHead;
    PolyDriver                  *drvTorso;
    PolyDriver                  *drvGazeCtrl;
    PolyDriver                  *drvArm[2];
    PolyDriver                  *drvCartArm[2];

    IEncoders                   *head;
    IEncoders                   *armEnc[2];
    IPositionControl            *torsoPos;
    IGazeControl                *gazeCtrl;
    IControlMode                *armCtrlMode[2];
    ITorqueControl              *armTorqueCtrl[2];
	IImpedanceControl           *armImpedenceCtrl[2];
    IVelocityControl            *armVelocityCtrl[2];

    int                         initial_gaze_context;
    int                         default_gaze_context;

    bool                        gazeInControl;

    bool                        stereo_track;
    int                         dominant_eye;


    double                      flipHand;

    ff2LayNN_tansig_purelin     net;

    ActionPrimitivesLayer2      *action[2];

    Vector                      homeFix;
    Vector                      reachAboveDisp;
    Vector                      graspAboveRelief;
    double                      targetInRangeThresh;
    double                      extForceThresh[2];

    Vector                      homePos[2];
    Vector                      homeOrient[2];
    Vector                      reachSideDisp[2];
    Vector                      reachAboveOrient[2];
    Vector                      reachAboveCata[2];
    Vector                      reachSideOrient[2];
    Vector                      deployPos[2];
    Vector                      pushDisp[2];
    Vector                      drawNearPos[2];
    Vector                      drawNearOrient[2];
    Vector                      homographyOffset[2];

    Vector                      graspDetectMean[2];
    Vector                      graspOpenMean[2];
    double                      graspDetectThresh[2];

    //for tactile sensing 
    Vector                      compensatedData;
    BufferedPort<Vector>        compensatedTactilePortLeft;
    BufferedPort<Vector>        compensatedTactilePortRight;
    std::vector<float>          percentile[2];	     // 95 percentile of each taxel (minus taxel baseline
    std::vector<float>          touchPerFinger;  // max touch value of each finge
    static const int            MAX_SKIN = 255;	 // max value you can read from the skin sensors
    static const int            SKIN_DIM = 60;   // number of taxels in one hand (actually they are 192)
    static const int            NUM_FINGERS = 5; // number of fingers used
    double                      touch_threshold; // if the compensated touch detected is greater than this, 
                                                 // then a contact has happened (default value 1)	
    Semaphore                   percentileSem;
    //tactile sensing ends


    Port                        eyeOutPort;
    BufferedPort<Bottle>        eyeInPort;

    BufferedPort<Vector>        graspDetectPort[2];

    Port                        wrenchPort;

    vector<Vector>              torsoPoses;
    vector<Vector>              handPoses;
    vector<Vector>              headPoses;

    double                      table_height;
    double                      table_height_tolerance;

    bool                        impedanceAvailable;

    bool                        running[2];
    int                         armInUse;
    int                         head_mode;
    int                         arm_mode;

    int                         grasp_state;



    //drag stuff
    struct Dragger
    {
        double                      extForceThresh;
        double                      currTime;
        double                      samplingRate;
        Vector                      initialPos;
        int                         armType;
        Bottle                      actions;
        string                      actionName;
        ICartesianControl           *ctrl;
        double                      min;
        double                      max;
    } dragLearner;

    Vector lp_filter(Vector input);

    bool loadExplorationPoses(const string &file_name);

    int checkArm(int arm)
    {
        if(arm!=ARM_IN_USE && arm!=ARM_MOST_SUITED && action[arm]!=NULL)
            armInUse=arm;

        return armInUse;
    }

    int checkArm(int arm, Vector &xd)
    {
        if(arm==ARM_MOST_SUITED)
            arm=xd[1]<0.0?LEFT:RIGHT;

        arm=checkArm(arm);

        if(arm==LEFT)
        {
            xd[0]+=homographyOffset[LEFT][0];
            xd[1]+=homographyOffset[LEFT][1];
        }
        else
        {

            xd[0]+=homographyOffset[RIGHT][0];
            xd[1]+=homographyOffset[RIGHT][1];
        }

        return arm;
    }
    Vector visionToMotorHomography(const Vector &stereo);
    Vector visionToMotorNet(const Vector &stereo);
    Vector posToEyesOrient(const Vector &originalPose);
    Vector randomDeployOffset();
    bool getGeneralOptions(Bottle &b);
    bool getArmOptions(Bottle &b, const int &arm);
    void close();
    bool isTactileHolding(int arm=ARM_IN_USE);

public:
    MotorThread(ResourceFinder &_rf, Resource &_res)
        :RateThread(20),rf(_rf),res(_res)
    {}

    virtual bool threadInit();
    virtual void run();
    virtual void threadRelease();
    virtual void onStop();


    void lookAtHand(bool look)
    {
        gazeCtrl->setTrackingMode(look);
        head_mode=look?HEAD_MODE_TRACK_HAND:HEAD_MODE_IDLE;
    }

    bool setArmInUse(int arm)
    {
        if(action[arm]==NULL)
            return false;

        armInUse=arm;
        return true;
    }

    void interrupt()
    {
        // since a call to checkActionsDone() blocks
        // the execution until it's done, we need to 
        // take control and exit from the waiting state
        action[LEFT]->syncCheckInterrupt(true);
        action[RIGHT]->syncCheckInterrupt(true);
    }

    void keepFixation(const bool fix)
    {
        gazeCtrl->setTrackingMode(fix);
    }

    void trackObject(const bool track)
    {
        if(track)
        {
            gazeCtrl->setEyesTrajTime(1.0);
            gazeCtrl->setNeckTrajTime(2.0);
            head_mode=HEAD_MODE_TRACK_TEMP;
        }
        else
        {
            gazeCtrl->restoreContext(default_gaze_context);
            head_mode=HEAD_MODE_IDLE;
        }
    }

    void setGazeIdle()
    {
        head_mode=HEAD_MODE_IDLE;
        gazeCtrl->restoreContext(initial_gaze_context);
        gazeInControl=false;
    }

    // basic commands
    void reachAbove(const Vector &stereo, int arm=ARM_MOST_SUITED, bool pregrasp_hand=false);
    void reachAboveNotSafe(const Vector &stereo, int arm=ARM_MOST_SUITED, bool pregrasp_hand=false);
    void reachAboveSide(const Vector &stereo, int arm=ARM_MOST_SUITED);
    void reachSide(const Vector &stereo, int arm=ARM_MOST_SUITED);
    void reachSpace(const Vector &stereo, int arm=ARM_MOST_SUITED);
    void push(const Vector &stereo, int arm=ARM_MOST_SUITED);
    void point(const Vector &stereo, int arm=ARM_MOST_SUITED);
    void grasp(int arm=ARM_IN_USE);
    void release(int arm=ARM_IN_USE);
    void goHome(bool home_head=true);
    void deploy(bool random=true, int arm=ARM_IN_USE, const Vector &stereo=Vector());
    void putAway(bool random=true);
    void drawNear(int arm=ARM_IN_USE);
    bool isHolding(int arm=ARM_IN_USE);
    bool setPercentile(vector<float> newPercentile, int arm);
    bool calibTable(int arm=ARM_IN_USE);
    void exploreHand(const double &trial_time, int arm=ARM_IN_USE);
    void exploreTorso(const double &trial_time);
    int getArmInUse(){return armInUse;}


    bool startDragMode(const string &action_name, int arm=ARM_IN_USE);
    void suspendDragMode();

    bool imitateAction(const string &action_name, int arm=ARM_IN_USE);


    void torsoPrepareForGrasp()
    {
        torsoPos->positionMove(2,25.0);
    }


    bool exploreTable(int arm, Vector xd);

    void shakeHand()
    {
        arm_mode=(arm_mode==ARM_MODE_FLIP_HAND)?ARM_MODE_IDLE:ARM_MODE_FLIP_HAND;
    }

    void setImpedance(bool turn_on)
    {
        if(turn_on && impedanceAvailable)
        { 
            for(int i=0; i<5; i++)
            {
                if(action[LEFT]!=NULL)
                    armCtrlMode[LEFT]->setImpedanceVelocityMode(i);
                if(action[RIGHT]!=NULL)
                    armCtrlMode[RIGHT]->setImpedanceVelocityMode(i);
            }
        }
        else if(impedanceAvailable)
        {
            for(int i=0; i<5; i++)
            {
                if(action[LEFT]!=NULL)
                    armCtrlMode[LEFT]->setVelocityMode(i);
                if(action[RIGHT]!=NULL)
                    armCtrlMode[RIGHT]->setVelocityMode(i);
            }
        }
    }

    void extract()
    {
        Vector x,o;
        action[armInUse]->getPose(x,o);   

        x[2]+=0.15;
        action[armInUse]->pushAction(x,o,2.0);

        bool f;
        action[armInUse]->checkActionsDone(f,true);
    }
};


#endif
