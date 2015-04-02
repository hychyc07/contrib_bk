
#include "grasp.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

#include <math.h>
#define MIN_X_RANGE         -0.20   // 20cm

const int OPEN_POSES[8] = {90, 10, 10, 10, 10, 10, 10, 10}; 
const int CLOSE_POSES[8] = {90, 45, 45, 30, 45, 30, 60, 90};


void Grasp::loop()
{
   Bottle* cmd = graspCmdPort.read(false);
   if(cmd && (status != BUSY))
   {
       if(cmd->toString() == ConstString(GRASP_CMD))
       {
            Vector handPos,handOrient;
            iArm->getPose(handPos, handOrient);
            handPos[0] = handPos[0] + graspXOffset;
            handPos[1] = handPos[1] + graspYOffset;
            handPos[2] = handPos[2] + graspZOffset;               
            handPos[0] = (handPos[0]>MIN_X_RANGE) ? MIN_X_RANGE :  handPos[0];
            iArm->goToPose(handPos, handOrient, 1.0);
            yarp::os::Time::delay(1.5);
            closeHand();
            action = GRASPED;
       }
       else if(cmd->toString() == ConstString(RELEASE_CMD))
       {
            openHand();
            action = RELEASED;
       }
       status = BUSY;
       actionTime = yarp::os::Time::now();
   }


    if(!withTactile)
    {  
        bool motionDone;   
        if(iHand->checkMotionDone(&motionDone))
        {
            if(motionDone && 
                (yarp::os::Time::now() - actionTime) >= 3.0 )
                status = action;
        }        
    }
    else
    {
        bool motionDone;   
        double sumTactile;
        if(iHand->checkMotionDone(&motionDone))
            if(motionDone && getTactileValue(sumTactile))
            {
                printf("Value: %.3f, Threshold:%.3f, Reset:%.3f \n", sumTactile, tactileThreshold, tactileReset);
                if((action == GRASPED) && (fabs(sumTactile-tactileReset) > tactileThreshold)) // it is really grasped
                    status = GRASPED;
                else
                {
                    status = RELEASED;
                    tactileReset = (tactileReset*0.9) +(sumTactile*0.1);
                }
            }
    }

    if(status == GRASPED)
    {
        Bottle &status = graspStatusPort.prepare();
        status.clear();
        status.addDouble(1.0);
        graspStatusPort.write();  
        printf("grasped\n");
    }
}

void Grasp::closeHand(void)
{
    for(int i=0; (i<closePoses.size()) && (i<8); i++)
    {        
        iHand->setRefSpeed(i+8, 80);
        iHand->positionMove(i+8, closePoses[i]);
    }        
}


void Grasp::openHand(void)
{
    for(int i=1; (i<openPoses.size()) && (i<8); i++)
    {
        iHand->setRefSpeed(i+8, 80);
        iHand->positionMove(i+8, openPoses[i]);
    }        
    yarp::os::Time::delay(1.0);
    iHand->setRefSpeed(8, 30);
    iHand->positionMove(8, openPoses[0]);
}


bool Grasp::getTactileValue(double &sumTactile)
{
    Vector* compensatedData = tactilePort.read(false);
    if(compensatedData)
    {
        sumTactile = 0;
        int end_index = (compensatedData->size() < 60) ? compensatedData->size() : 60;
        for(int i=0; i<end_index; i++)
            sumTactile += (*compensatedData)[i];
        /*        
        printf("Value: %.3f ", sumTactile);
        printf("\n");
        */
        return true;
    }
    return false;
}

bool Grasp::open(yarp::os::ResourceFinder &rf)
{   
    string ctrlName;
    string robotName;
    string remoteName;
    string localName;

    //Time::turboBoost();

    // get params from the RF    
    partName = rf.check("hand",Value("right")).asString().c_str();
    withTactile = rf.check("tactile");    
    tactileThreshold  = rf.check("tactile_threshold",Value(50.0)).asDouble();
    graspXOffset  = rf.check("x_offset",Value(0.00)).asDouble();
    graspYOffset  = rf.check("y_offset",Value(0.00)).asDouble();
    graspZOffset  = rf.check("z_offset",Value(0.00)).asDouble();

    if(Bottle *bt=rf.find("open_hand").asList())
    {
        for(int i=0; i<bt->size(); i++)
            openPoses.push_back(bt->get(i).asDouble());
    }
    else
        openPoses.assign(OPEN_POSES, OPEN_POSES+8); 
        
    if(Bottle *bt=rf.find("close_hand").asList())
    {
        for(int i=0; i<bt->size(); i++)
            closePoses.push_back(bt->get(i).asDouble());
    }
    else
        closePoses.assign(CLOSE_POSES, CLOSE_POSES+8); 
 
    // opening hand controller
    robotName = rf.check("robot",Value("icub")).asString().c_str();
    remoteName = string("/")+robotName+string("/")+partName+string("_arm");
    localName = string("/graspObject/")+partName+string("_hand");
 
    // open the client
    Property option("(device remote_controlboard)");
    option.put("remote",remoteName.c_str());
    option.put("local",localName.c_str());
    if (!driver.open(option))
        return false;
    // open the view
    driver.view(iHand);

    // opening arm controller
    remoteName=string("/")+robotName+"/cartesianController/"+partName+string("_arm");
    localName=string("/graspObject/")+partName+string("_arm");
    // open the client
    Property armOption("(device cartesiancontrollerclient)");
    armOption.put("remote",remoteName.c_str());
    armOption.put("local",localName.c_str());
    if (!armDriver.open(armOption))
        return false;

    // open the view
    armDriver.view(iArm);
    iArm->setTrackingMode(false);


    //Rand::init();

    openHand();
    action = status = RELEASED;


    bool ret=true;   
    ret = graspCmdPort.open("/graspObject/cmd/in");
    ret &= graspStatusPort.open("/graspObject/status/out");     
    ret &= tactilePort.open("/graspObject/tactile/in");      
    return ret;
}

bool Grasp::close()
{
    //iHand->stop();
    openHand();
    driver.close();
    armDriver.close();
    graspCmdPort.close();
    graspStatusPort.close();
    tactilePort.close();
    return true;
}

bool Grasp::interrupt()
{
    graspCmdPort.interrupt();
    tactilePort.interrupt();
    return true;
}


