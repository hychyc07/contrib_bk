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

#ifndef __INVKIN_THREAD_H__
#define __INVKIN_THREAD_H__

#include <opencv/cv.h>
#include <iostream>
#include <vector>

#include "iCub/Util.h"
#include "iCub/ViewSphere.h"

#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

class ObjRecModule;

using namespace yarp::dev;


class InvKinThread : public yarp::os::RateThread
{
public:

      InvKinThread( 
              ObjRecModule *module_,
              int period,
              PolyDriver &robotArm_,
              PolyDriver &robotTorso_,
              PolyDriver &armCartDriver_,
              ResourceFinder *rf_) :
          RateThread(period),
          module(module_),
          robotArm(robotArm_),
          robotTorso(robotTorso_),
          armCartDriver(armCartDriver_),
          rf(rf_),
          velCtrl(0),
          torsoVelCtrl(0)
    {}

    virtual bool threadInit();     
    virtual void threadRelease();
    virtual void run(); 

    inline void setSampleFile(const std::string &filename)
    {
        filenameTrainingData = filename;
    }

private:
    static const int NUM_ARM_JOINTS = 7;
    static const int NUM_JOINTS = 16;
      
    ObjRecModule *module;

    ResourceFinder *rf;

    PolyDriver &robotArm;
    PolyDriver &robotTorso;
    PolyDriver &armCartDriver;
    IControlLimits *limArm;
    IControlLimits *limTorso;
    IPositionControl *armCtrl;
    IPositionControl *torsoCtrl;
    IVelocityControl *velCtrl;
    IVelocityControl *torsoVelCtrl;
    IEncoders *encArm;
    IEncoders *encTorso;
    ICartesianControl *armCart;

    std::string filenameTrainingData;

    double threadStartTime;
    ViewSphere kinExpSphere; // shows exploration targets
    int explSampleCount; 
    double lastInvKinPrintTime;
    Vector kinExpTargetQ;
    Vector nextQ;
    cv::Mat invKinExplorationMap;

    void invKinExploration(double time);
	//void safeVelocityMove(int encIndex, double speed);
    //
    void saveTrainingSample(const std::string &filename, double e, double r, 
        const Vector &handX, const Vector &eyeX, 
        const Vector &torsoQ, const Vector &armQ,
        const Vector &handO);
    void deleteTrainingData();
};

#endif

