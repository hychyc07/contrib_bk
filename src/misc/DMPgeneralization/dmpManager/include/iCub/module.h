/*
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff Elena Ceseracciu
 * email:  vadim.tikhanoff@iit.it elena.ceseracciu@iit.it
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

#ifndef __DMPMANAGER_MODULE_H__
#define __DMPMANAGER_MODULE_H__

#include <string>

#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>

#include <cv.h>

#include "iCub/utils.h"
#include "iCub/dmpManagerInterface.h"
#include "iCub/dmpLearnerInterface.h"
#include "iCub/dmpExecutorInterface.h"

/**********************************************************/
class Manager : public yarp::os::RFModule, iCub::dmpManagerInterface
{
protected:
    yarp::os::Port              thriftPort;         // TEST! for thrift-interface commands
    bool attach(yarp::os::Port &source);
    bool train(const std::string& action, const std::string& target, const std::string& tool, const std::string& hand);
    bool stop_training();
    bool s(){return stop_training();};
    bool test(const std::string& action, const std::string& target, const std::string& tool, const std::string& hand);
    std::string observe_state();
    void go_home();                                 //TEST up to here
    void quit();
    
    yarp::os::Port             dmpLearnerPort;     // also a TEST
    iCub::dmpLearnerInterface  dmpLearner;         //  ""
    
    yarp::os::Port             dmpExecutorPort;     // also a TEST
    iCub::dmpExecutorInterface  dmpExecutor;         //  ""
    
    std::string                 name;               //name of the module
    std::string                 hand;               //name of the module
    std::string                 camera;             //name of the camera
    yarp::os::Port              rpcHuman;           //human rpc port (receive commands via rpc)
    yarp::os::RpcClient         rpcMotorAre;        //rpc motor port ARE
    yarp::os::RpcClient         iolStateMachine;    //rpc to iol state machine
    yarp::os::RpcClient         rpcMIL;             //rpc mil port
    yarp::os::RpcClient         rpcWBD;

    ParticleFilter              particleFilter;     //class to receive positions from the templateTracker module
    SegmentationPoint           segmentPoint;       //class to request segmentation from activeSegmentation module
    PointedLocation             pointedLoc;         //port class to receive pointed locations
    ObjectPropertiesCollectorPort opcPort;
    yarp::sig::Vector           initPos;
    
    
    yarp::os::BufferedPort<yarp::os::Bottle>        blobExtractor;
    yarp::os::BufferedPort<yarp::os::Bottle>        particleTracks;

    yarp::os::Semaphore         mutexResources;     //mutex for ressources
    bool                        pointGood;          //boolean for if got a point location
    CvPoint                     pointLocation;      //x and y of the pointed location
    bool                        init;
    yarp::os::Bottle            lastBlobs;
    yarp::os::Bottle            lastTool;
    
    std::string                 obj;
    std::string                 action;

    yarp::os::Bottle            getBlobs();
    CvPoint                     getBlobCOG(const yarp::os::Bottle &blobs, const int i);
    bool                        get3DPosition(yarp::sig::Vector &point, yarp::sig::Vector &x);
    yarp::os::Bottle            findClosestBlob(const yarp::os::Bottle &blobs, const CvPoint &loc);
    int                         processHumanCmd(const yarp::os::Bottle &cmd, yarp::os::Bottle &b);
    int                         executeOnLoc(bool shouldTrain);
    void                        segmentAndTrack( int x, int y );
    
    yarp::os::Bottle            getOffset(yarp::os::Bottle &closestBlob, double actionOrient, yarp::sig::Vector &initPos);
    
    void                        getPraticleTracks();
    void                        goHome();
    void                        goHomeArmsHead();
    void                        wbdRecalibration();
    
    void                        acquireImage(const bool request=false);
    yarp::os::Bottle            classify(const yarp::os::Bottle &blobs, int index);
    yarp::os::Bottle            getType(const yarp::os::Bottle *mils, int index);

public:
    bool    configure(yarp::os::ResourceFinder &rf);
    bool    interruptModule();
    bool    close();
    bool    updateModule();
    double  getPeriod();
    
};
#endif

