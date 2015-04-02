/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff Ugo Pattacini
 * email:  vadim.tikhanoff@iit.it
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

#ifndef __MODULE_H__
#define __MODULE_H__

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

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/CartesianControl.h>


#include <cv.h>

#include "iCub/utils.h"

#define LEFTARM             0
#define RIGHTARM            1

/**********************************************************/
struct blobsData
{
        int         index;
        double      lenght;
        double      vdrawError;
        CvPoint     posistion;
        double      bestDistance;
        double      bestAngle;
        std::string name;
};
/**********************************************************/
class Manager : public yarp::os::RFModule
{
protected:

    std::string                 name;               //name of the module
    std::string                 hand;               //name of the module
    std::string                 camera;             //name of the camera
    yarp::os::Port              rpcHuman;           //human rpc port (receive commands via rpc)
    yarp::os::RpcClient         rpcMotorAre;        //rpc motor port ARE
    yarp::os::RpcClient         rpcMotorKarma;      //rpc motor port KARMA    
    yarp::os::RpcClient         iolStateMachine;    //rpc to iol state machine
    yarp::os::RpcClient         rpcMIL;             //rpc mil port
    yarp::os::RpcClient         rpcKarmaLearn;      //rpc mil port
    yarp::os::RpcClient         rpcReconstruct;     //rpc reconstruct
    yarp::os::RpcClient         rpcGraspEstimate;   //rpc graspEstimate
    yarp::os::RpcClient         rpcOPC;   //rpc graspEstimate

    ParticleFilter              particleFilter;     //class to receive positions from the templateTracker module
    SegmentationPoint           segmentPoint;       //class to request segmentation from activeSegmentation module
    PointedLocation             pointedLoc;         //port class to receive pointed locations
    
    yarp::os::BufferedPort<yarp::os::Bottle>        blobExtractor;
    yarp::os::BufferedPort<yarp::os::Bottle>        particleTracks;

    yarp::os::Semaphore         mutexResources;     //mutex for ressources
    bool                        pointGood;          //boolean for if got a point location
    CvPoint                     pointLocation;      //x and y of the pointed location
    bool                        init;
    yarp::os::Bottle            lastBlobs;
    yarp::os::Bottle            lastTool;
    yarp::sig::Vector           objectPos;
    yarp::sig::Vector           toolSmall, toolBig;
    
    std::string                 obj;
    double                      userTheta;
    blobsData                   *blobsDetails;

    std::map<int, double>       randActions;

    yarp::os::Bottle            getBlobs();
    CvPoint                     getBlobCOG(const yarp::os::Bottle &blobs, const int i);
    double                      getBlobLenght(const yarp::os::Bottle &blobs, const int i);

   // void                        acquireImage(const bool rtlocalization=false);
    bool                        get3DPosition(const CvPoint &point, yarp::sig::Vector &x);
    yarp::os::Bottle            findClosestBlob(const yarp::os::Bottle &blobs, const CvPoint &loc);
    int                         processHumanCmd(const yarp::os::Bottle &cmd, yarp::os::Bottle &b);
    int                         executeOnLoc(bool shouldTrain);
    int                         executeToolOnLoc();
    yarp::os::Bottle            executeToolLearning();
    int                         executeToolSearchOnLoc( const std::string &objName );
    yarp::os::Bottle            executeBlobRecog( const std::string &objName );

    bool                        executeCloseHand(int ARM);
    bool                        executeDropAway(int ARM);
    bool                        executeGiveAction(int ARM);
    bool                        executeSpeech( const std::string &speech );
    double                      executeVirtualDraw(blobsData &blobsDetails);
    double                      executeToolDrawNear(blobsData &blobsDetails);
    int                         executeToolAttach(const yarp::sig::Vector &tool);
    yarp::os::Bottle            executeKarmaOptimize( const yarp::sig::Vector &tool, const std::string &objName);
    yarp::os::Bottle            classifyThem();

    int                         startup_context_id_left;
    int                         startup_context_id_right;

    yarp::os::Bottle            findBlobLoc();
    yarp::os::Bottle            blobLoc;
    yarp::os::Bottle            blobList;
    
    //just for testing     
    yarp::dev::PolyDriver           drvCartLeftArm, drvCartRightArm;
    yarp::dev::ICartesianControl    *cartArm;

    bool                        executePCLGrasp( const std::string &objName );
    double                      latchTimer, idleTmo;

    void                        takeMotionARE();
    void                        segmentAndTrack( int x, int y );
    
    yarp::os::Bottle            getOffset(yarp::os::Bottle &closestBlob, double actionOrient, yarp::sig::Vector &initPos);
    
    void                        getPraticleTracks();
    void                        goHome();
    void                        goHomeArmsHead();
    double                      wrapAng (const double ang);
    
    //void                        acquireImage(const bool request=false);
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

