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


/**
 * @file ObjRecThread.h
 * @brief 
 */

#ifndef __OBJREC_THREAD_H__
#define __OBJREC_THREAD_H__

#include <vector>

#include <opencv2/core/core.hpp>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#include "iCub/ActiveRec.h"
#include "iCub/GlobalSettings.h"

using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::os;


class ObjRecModule;

class ObjRecThread : public yarp::os::RateThread
{
public:

    /* class methods */
    ObjRecThread(
            ObjRecModule *module_,
            int period,
            ResourceFinder *_rf,
            bool sim)
        :  module(module_), RateThread(period), rf(_rf), 
        isSimulation(sim), activeRec(settings)
    {   
    }

    virtual bool threadInit();     
    virtual void threadRelease();
    virtual void run(); 

    inline void startRecogTrials(){ recogTrialNo = 0; }
    inline void setTrueLabel(const std::string &label) { trueLabel = label;}
    inline void setObjectRotation(double rot) { objectRotation = rot; }

    inline void setMotionPlanning(bool status) { doMotionPlanning = status; }
    inline void setBoosting(bool status) { settings.doBoosting = status; }
    inline bool isBoosting() const { return settings.doBoosting; };
    //inline void setFastPlanning(bool status) { fastPlanning = status; }

    inline void setResultDir(const std::string &dir)
    {
        recogResultDir = dir;
    }
private:

    /* constants */
    static const int NUM_ARM_JOINTS = 7;
    static const int NUM_JOINTS = 16;

    double DESIRED_ENTROPY;

    ObjRecModule *module;
    ResourceFinder *rf;

    /* Ports */
    //BufferedPort<ImageOf<PixelBgr> > entropyMapPort;
    // Ports to record recognition data
    BufferedPort<Bottle> recogResultPort;

    GlobalSettings settings;
    bool isSimulation;

    bool doMotionPlanning;
    bool fastPlanning;
    bool waitOnIter;
    int Nb_Iteration;
    int maxRecogTrials;
    int recogTrialNo;

    //ViewSphere viewSphere;

    std::string recogResultDir;
    std::string objectViewDir;
    std::string trueLabel;
    double objectRotation;

    ActiveRec activeRec;

    cv::Vec2f last_gaze;
    int recogIteration;

    void runRecognition();
    bool restartRecognition();

    inline bool isRecogDone() const
    {
        //double H = activeRec.getEntropy();
        //if (H < DESIRED_ENTROPY || recogIteration >= Nb_Iteration)
        return (recogIteration >= Nb_Iteration);
    }
};

#endif  //__OBJREC_THREAD_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

