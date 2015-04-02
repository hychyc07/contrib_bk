/* 
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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

#ifndef __VTM_THREAD_H__
#define __VTM_THREAD_H__

#include <vector>

#include "iCub/KeyframeExtractor.h"

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

class ObjRecModule;

class VtmThread : public yarp::os::RateThread
{
public:

      VtmThread( 
              ObjRecModule *module_,
              int period,
              PolyDriver &robotArm_,
              ResourceFinder *rf_) :
          RateThread(period),
          module(module_),
          robotArm(robotArm_),
          rf(rf_)
    {}

    virtual bool threadInit();   
    virtual void run() {};
    virtual void threadRelease()
    {
        armCtrl->stop();
    }

    inline void setObjectName(const std::string &name) { objectName = name; }

protected:
    static const int NUM_ARM_JOINTS = 7;
    static const int NUM_JOINTS = 16;
    
    static const double POSITION_0[];
    static const double POSITION_1[];
    static const double POSITION_2[];
    static const double POSITION_3[];
    static const double POSITION_4[];
    static const double POSITION_5[];
      
    ObjRecModule *module;

    ResourceFinder *rf;

    KeyframeExtractor keyframeExtractor;
    
    PolyDriver &robotArm;
    IControlLimits *limArm;
    IPositionControl *armCtrl;
    IEncoders *encArm;

    void moveArm();

    std::string vtmDir;
    std::string objectName; 

    std::vector<Vector> trajectory;
    int trajectoryPos;
    bool trajectoryCompleted;

    inline bool checkArmMotionDone() const
    {
        bool done;
        armCtrl->checkMotionDone(&done);
        return done;
    }
    
};

#endif

