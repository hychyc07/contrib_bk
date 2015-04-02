// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author:  Eric Sauser
 * email:   eric.sauser@a3.epfl.ch
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef HandSkinControllerTHREAD_H_
#define HandSkinControllerTHREAD_H_

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IAnalogSensor.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

#include "MathLib/MathLib.h"
#include "MathLib/GMR2.h"

#define FINGERS_COUNT    3
#define SKIN_SIZE        192
#define SKIN_FULL_SIZE   (SKIN_SIZE*3)
#define SKIN_FTIP_OFFSET 0


class HandSkinControllerThread: public RateThread
{
public:
    enum CtrlMode{CM_IDLE=0, CM_REST, CM_RUN , CM_REC,CM_RECRUN,CM_REPLAY,CM_REPLAYSIM};

public: 
    /// Shows some debug information
    bool                    bDebugMode;
    /// Disable the use of the reliability measure (false by default)
    bool                    bNoMissingCheck;
    /// Enable the gradient ascent (true by default)
    bool                    bGradientAscent;

    /// Switch the streamed output data to testing mode (more data, but not for learning phase) (true by default)
    bool                    bTestingOutput;

    /// Simulate a bad skin input
    bool                    bBadSkin;
    /// Which fingertip should be bad
    int                     mBadFingerId;

    // Allows autosaving of data when start/stopping (false by default)
    bool                    bAutoSaveTrainingData;    

private:    
    

    // Some general thread variables
    Semaphore               mMutex;
    int                     mPeriod;
    char                    mBaseName[256];

    double                  mTime;
    double                  mPrevTime;
    double                  mStartTime;

    // Some robot controller interfaces and others
    PolyDriver              *mDriver;
    IEncoders               *mEncoders;
    IControlLimits          *mLimitsController;
    IPidControl             *mPIDController;
    IOpenLoopControl        *mOpenLoopController;
    IControlMode            *mControlModeController;

    /// The part (left or right) that we control
    char                    mPartName[256];

    /// The base path for datafiles
    char                    mBasePath[256];

    /// Current control mode of the joints
    int                     mJointControlMode[256];
    /// Number of joints
    int                     mJointsSize;    
    /// Offset counter from the icub's arm part to reach the fingers
    int                     mJointsFingerOffset;


    /// Current position
    Vector                  mJointsPos;
    /// Current target
    Vector                  mJointsTargetPos;
    /// Current target, but predicted
    Vector                  mJointsTargetPredPos;
    /// Current target command
    Vector                  mJointsTargetCmd;
    /// Current desired output position
    Vector                  mJointsOutputPos;
    /// Joint limits
    Vector                  mJointsPosLimits[2];
    /// Joint limits
    Vector                  mJointsVelLimits[2];
    /// Joint range
    Vector                  mJointsRange;

    /// Rest position
    Vector                  mJointsRest;

    /// Did we just started
    bool                    bFirst;
    /// Control mode or current state
    CtrlMode                mCtrlMode;

    // PID Controller variables,
    // and sorry if I mixed a bit with a consistant
    // naming of the variables below

    /// PID direction
    Vector                  mJointsPidFactor;
    /// PID position: P gain
    Vector                  mJointsPidKpP;
    /// PID position: D gain
    Vector                  mJointsPidKdP;
    /// PID position: I gain
    Vector                  mJointsPidKdI;
    
    /// PID pressure: P gain
    Vector                  mJointsPidKpS;
    /// PID pressure: D gain
    Vector                  mJointsPidKdS;

    /// Global PID gain for all (may change for different execution modes)
    double                  mJointsPidGain;
public:    
    /// Global PID gain for all (fixed and constant.) Help compensating for untensioning of the fingers cables
    double                  mJointsPidGainGlobal;
private:
    /// Switching value between force-position control (a.k.a model membership value)
    double                  mCtrlSwitch;


    /// Shall the pressure part of the controller be used
    bool                    bPidUsePressure;
    /// Shall the integral part of the controller be used
    bool                    bPidUseI;
    
    /// Shall we filter the skin signal?
    bool                    bFilterSkin;
    /// Filter time constant
    double                  mSkinFilterTau;
    

    /// The GMM
    MathLib::GaussianMixture2       mGMModel;
    

    MathLib::Vector                 mSFingerPos[FINGERS_COUNT];
    MathLib::Vector                 mSFingerTip[FINGERS_COUNT];
    MathLib::Vector                 mSFingerTipDir[FINGERS_COUNT];
    MathLib::Vector                 mSFingerCmd[FINGERS_COUNT];

    MathLib::Vector                 mSFingerDesPos[FINGERS_COUNT];
    MathLib::Vector                 mSFingerDesTip[FINGERS_COUNT];

    MathLib::Matrix                 mSFingerDesCoupling;

    MathLib::Vector                 mPrevSFingerTipErr[FINGERS_COUNT];

    MathLib::Vector                 mPrevSFingerPosErr[FINGERS_COUNT];
    MathLib::Vector                 mIntSFingerPosErr[FINGERS_COUNT];
    
    double                          mPrevThumbAbdPosErr;
    double                          mIntThumbAbdPosErr;
    
    double                          mPrevFingerSpreadPosErr;


	BufferedPort<Vector>            mFingerTipPort;
    Vector                          mFingerTip;
    Vector                          mFingerTipBaseline;
    Vector                          mFingerTipRaw;
    int                             mFingerTipCalibCount;

    /// Did we read some skin part already?
    bool                            bGotSkin;

    /// Output data port related variables
	BufferedPort<Vector>            mOutputDataPort;
    Vector                          mOutputData;
    bool                            bSendOutputData;


    
    /// Data matrix for replay
    MathLib::Matrix                 mReplayData;
    /// Replay dt
    double                          mReplayDt;
    /// Replay start time
    double                          mReplayStartTime;
    /// Replay done flag
    bool                            bReplayDone;

    
    /// Storage variable for the error signal
    MathLib::Vector                 mErrorSignal;
    
    /// Data storage for saving the training data
    MathLib::Matrix                 mTrainingData;
    /// Its current position
    int                             mTrainingDataPos;
public:
    /// The base name of that training data
    char                            mTrainingDataName[256];

public:
    HandSkinControllerThread(int period, const char* baseName, const char* part);
    virtual ~HandSkinControllerThread();


    virtual void run();
    virtual bool threadInit();
    virtual void threadRelease();
    
            void    SetDriver(PolyDriver* driver);
            
            /// Change the state of the system
            void    SetCtrlMode(CtrlMode mode);   

            /// Stop the motor gracefully         
            void    Stop();
            
            /// Init the finger data structures
            void    ResetSFingerData();
            
            /// Read the skin ports
            void    ReadFromPort();    
            /// Fill in variables
            void    PrepareSFingers();

            /// Predict
            void    PredictPoseFromAngle();

            /// Set the control mode to PIS
            void    SetPIDMode();
            /// Send some PID commands
            void    SendPIDControl();
            
            // The open-loop controller...
            void    SendOpenLoopControl(MathLib::Matrix * Weights = NULL);
            
            /// Start/stop sending data button
            void    SwitchSendOutputData();

            /// Send some data on an output port
            void    SendOutputData();

            /// Re-calibrate the skin
            void    ResetSkin();
            
            /// Load the gmm
            void    LoadGMM(const char *name);
            /// Load the replay data
            void    LoadReplay(const char *name);

            /// Set the base path for I/O files
            void    SetBasePath(const char *path);

};


#endif

