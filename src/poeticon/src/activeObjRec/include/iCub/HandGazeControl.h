/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Bjoern Browatzki
 * email:   bjoern.browatzki@tuebingen.mpg.de
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

/**
 * @file HandGazeControl.h
 * @brief 
 */

#ifndef __HANDGAZECONTROL_H__
#define __HANDGAZECONTROL_H__

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/flann/flann.hpp> 

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>

using namespace yarp::sig;
using namespace yarp::dev;


class HandGazeControl : public yarp::os::RateThread
{
public:
    HandGazeControl(
            double period, 
            const std::string &filenameTrainingData_, 
            PolyDriver &arm_, 
            PolyDriver &torso_, 
            PolyDriver &armCartDriver_,
            IGazeControl *gazeCtrl_,
            bool isLeftHand_) : 
        RateThread(period), filenameTrainingData(filenameTrainingData_), arm(arm_), armCartDriver(armCartDriver_),
        torso(torso_), gazeCtrl(gazeCtrl_), isLeftHand(isLeftHand_),
        wGaze(2.0), wTargetX(1.0), wDeltaQ(0.0), kNN(100),  isNewTarget(false)
    { 
        targetX.resize(3);
        targetX[0] = -0.3;
        targetX[1] = 0;
        targetX[2] = 0.2;
        //nnSearch = 0;
    }

    virtual bool threadInit() 
    {
        if (! armCartDriver.view(armCart))
        {
            std::cout <<  "Cannot get cartesian interface to the arm" << std::endl;  
            armCartDriver.close();
            return false;
        }

        if (! arm.isValid()) 
        {
            std::cout << "HandGazeControl::init(): Cannot connect to robot arm" << std::endl;  
            std::cout << "Device not available.  Here are the known devices:" << std::endl;
            std::cout << Drivers::factory().toString() << std::endl;
            return false;
        }

        if (! arm.view(encArm))
        {
            std::cout <<  "HandGazeControl::init(): Cannot get interface to arm" << std::endl;  
            return false;
        }

        if (! torso.isValid()) 
        {
            std::cout << "HandGazeControl::init(): Cannot connect to robot torso" << std::endl;  
            std::cout << "Device not available.  Here are the known devices:" << std::endl;
            std::cout << Drivers::factory().toString() << std::endl;
            return false;
        }

        if (! torso.view(encTorso))
        {
            std::cout <<  "HandGazeControl::init(): Cannot get interface to torso" << std::endl;  
            return false;
        }

        if (! init())
        {
            return false;
        }

        starttime = 0;
    }

    virtual void run() 
    {
        fixateObject();
        
        if (starttime > 1)
        {
            double movetime = yarp::os::Time::now() - starttime;
            if (movetime > timeout)
            {
                stopControl();
                starttime = 0;
            }
        }
    }

    virtual void threadRelease()
    {
        stop();
        gazeCtrl->stopControl();
    }

    bool init();

    inline void stopControl()
    {
        armCart->stopControl();
    }

    inline void lookAtViewpoint(double elevation, double rotation)
    {
        elevation = std::max(0.0, elevation);
        elevation = std::min(180.0, elevation);
        rotation  = std::max(0.0, rotation);
        rotation  = std::min(360.0, rotation);

        targetElevation = elevation;
        targetRotation  = rotation;

        Vector handX, handO;
        if (! calcTargetPose(targetElevation, targetRotation, handX, handO))
        {
            std::cerr << "ERROR - HandGazeControl::run: Cannot calculate robot configuration for desired viewpoint!";
            return;
        }

        if (armCart)
        {
            starttime = yarp::os::Time::now();
            armCart->goToPose(handX, handO);
        }
        else
            std::cerr << "ERROR - HandGazeControl::run: armCart null" << std::endl;
        //isNewTarget = true;
    }

    void fixateObject(); 

    bool targetReached();

    inline void setGazeOffsets(double offsetX, double offsetY, double offsetZ)
    {
        gazeOffsetX = offsetX;
        gazeOffsetY = offsetY;
        gazeOffsetZ = offsetZ;
    }

    inline void getGazeOffsets(double &offsetX, double &offsetY, double &offsetZ)
    {
        offsetX = gazeOffsetX;
        offsetY = gazeOffsetY;
        offsetZ = gazeOffsetZ;
    }

    inline void setK(int k)
    {
        kNN = k;
        if (kNN < 1)
            kNN = 1;
    }

    inline void setWeights(double wGaze, double wTargetX, double wDeltaQ)
    {
        this->wGaze  = wGaze;
        this->wTargetX = wTargetX;
        this->wDeltaQ  = wDeltaQ;
    }

    inline void setHandTargetPos(const Vector &targetX)
    {
        this->targetX = targetX;
    }
    inline Vector getHandTargetPos()
    {
        return this->targetX;
    }


private:
    static const double timeout = 10; // sec

    double compDistance(double e, double r, const Vector &curJoints, int index);
    Vector getHandPose(const Vector &torsoQ, const Vector &armQ);

    // normalization constants
    static const double NORM_GAZE;
    static const double NORM_TARGET_X;
    static const double NORM_DELTA_Q;

    double starttime;

    IEncoders *encArm;
    IEncoders *encTorso;
    ICartesianControl *armCart;
    IGazeControl *gazeCtrl;

    PolyDriver &armCartDriver;
    PolyDriver &arm;
    PolyDriver &torso;

    Vector targetX;

    double targetElevation;
    double targetRotation;
    double gazeOffsetX;
    double gazeOffsetZ;
    double gazeOffsetY;
    
    double wGaze;
    double wTargetX;
    double wDeltaQ;

    bool isNewTarget;

    bool isLeftHand;

    //cv::flann::Index *nnSearch;
    int kNN;

    std::vector<double> samplesE;
    std::vector<double> samplesR;
    std::vector<Vector> samplesHandX;
    std::vector<Vector> samplesHandO;
    std::vector<Vector> samplesEyeX;
    std::vector<Vector> samplesJoints;

    std::string filenameTrainingData;

    bool calcTargetPose(double e, double r, Vector &handX, Vector &handO);
};

#endif  //__HANDGAZECONTROL_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

