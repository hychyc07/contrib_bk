// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
 * Copyright (C) 2008 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Eric Sauser
 * email:   eric.sauser@a3.epf.ch
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

#include <iostream>
#include <vector>
#include <queue>
#include <yarp/os/all.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>


using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;




class VelocityController
{
public:
    enum VCMode{VC_IDLE=0, VC_VELOCITY, VC_POSITION, VC_COMBINED, VC_REST};

private:
    char                        mBaseName[256];
    char                        mName[256];
    PolyDriver                 *mDriver;

    Semaphore                   mMutex;

    IEncoders                   *mEncoders;
    IControlLimits              *mLimitsController;
    IVelocityControl            *mVelocityController; 
    IPositionControl            *mPositionController; 
    IPidControl                 *mPIDController;
    IControlMode                *mControlModeController;

    double                      mTime;
    double                      mPrevTime;
    
    double                      mLastPosCommandTime;
    double                      mLastVelCommandTime;
    double                      mLastIdleCommandTime;

    double                      mCommandTimeout;
    double                      mMinimumLoopTime;
    double                      mCummulativeDt;

    double                      mLastPosCtrlTime;

    
    bool                        bIsReady;

    bool                        bPosTimeoutPause;
    bool                        bVelTimeoutPause;
    bool                        bIdleTimeoutPause;
    bool                        bPosTargetSet;
    bool                        bVelTargetSet;
    bool                        bIdleModeSet;
    
    VCMode                      mMode;

    int                         mIdleStep;
          
    int                         mJointsSize;
    Vector                      mJointsPos;
    Vector                      mJointsVel;
    
    Vector                      mJointsTargetPos;
    Vector                      mJointsTargetVel;

    Vector                      mJointsInternalPos;
    Vector                      mJointsInternalVel;

    Vector                      mJointsOutputVel;
    Vector                      mJointsPrevOutputVel;

    Vector                      mJointsOutputPos;
    double                      mJointsPosCtrlTimeToGo;

    Vector                      mJointsPosLimits[2];
    Vector                      mJointsVelLimits[2];
    Vector                      mJointsRange;   
    
    Vector                      mJointsRest;   
    
    Vector                      mJointsError;
    
    Vector                      mJointsMask;

    Vector                      mJointsKp;
    Vector                      mJointsKd;
    

    bool                        bFirst;
    bool                        bReset;

    BufferedPort<Vector>        mDebugPort;


    class PositionElement{
    public:
        Vector  mPos;
        double  mTime;
        bool    bValid;
    public:
        PositionElement(int size=0);
        PositionElement(const Vector& pos, double time, bool valid = false);

        PositionElement(const PositionElement & p);

        void    Set(int size);
        void    Set(const Vector& pos, double time);
        void    Set(const Vector& pos, double time, bool valid);
        bool    IsValid();

    };

    queue<PositionElement>      mPosQueue;
    PositionElement             mFirstPos;
    PositionElement             mLastPos;
    double                      mPosTimeToGo;
    Vector                      mJointsStartPos;

public:

    VelocityController();
    ~VelocityController();
    
    bool    Init(PolyDriver *driver, const char* name, const char* basename = NULL);
    void    Free();

    char*   GetBaseName();

    void    Update();
    void    Update(double dt);       

    void    Stop();

    void    SetPositionTarget(Vector &target);
    void    SetVelocityTarget(Vector &target);
    void    GetPosition(Vector &pos);
    void    GetVelocity(Vector &vel);
    
    void    SetMask(Vector &mask);
    void    SetMaskAll();
    void    SetMaskNone();
         
    void    SetControlMode(VCMode mode);

    void    SetKp(double kps);
    void    SetKd(double kds);
    
    void    SetCommandTimeout(double timeout);
    void    SetMinimumLoopTime(double time);

    int     GetJointsSize();
    

    void PQueue();

};









