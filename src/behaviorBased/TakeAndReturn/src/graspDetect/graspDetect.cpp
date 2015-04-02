// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Ali Paikan
 * email:  ali.paikan@iit.it
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
#include "graspDetect.h"
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

#define GRASP_ECU_DIST    0.04

void GraspDetect::loop()
{
    if(Bottle *objPos = objectPosPort.read())
    {
        Vector handPos,handOrient;
        iarm->getPose(handPos, handOrient);

        printf("Obj: (%f, %f, %f)\n", objPos->get(0).asDouble(), objPos->get(1).asDouble(), objPos->get(2).asDouble());
        printf("Hnd: (%f, %f, %f)\n", handPos[0], handPos[1], handPos[2]);

        double ecu_dist = sqrt((objPos->get(0).asDouble()-handPos[0])*(objPos->get(0).asDouble()-handPos[0])
                                 + (objPos->get(1).asDouble()-handPos[1])*(objPos->get(1).asDouble()-handPos[1])
                                + (objPos->get(2).asDouble()-handPos[2])*(objPos->get(2).asDouble()-handPos[2]) );
        printf("Ecu_Dist:%.04f\n", ecu_dist);
        
        bool shouldGrasp = (ecu_dist <= detectThreshold);
        
        if(shouldGrasp)
        {
            printf("\n\ngrasp\n\n");
            // sending grasp command
            Bottle &grasp = graspCmdPort.prepare();
            grasp.clear();
            grasp.addString("grasp");
            graspCmdPort.write();       

            // sending gaze position
            Bottle &target=targetPort.prepare();
            //target = *objPos;
            target.clear();
            target.addDouble(handPos[0]);
            target.addDouble(handPos[1]);
            target.addDouble(handPos[2]);
            targetPort.write();
            
            //sending face emotion
            Bottle &cmd=faceExpPort.prepare();
            cmd.addVocab(Vocab::encode("set"));
            cmd.addVocab(Vocab::encode("all"));
            cmd.addVocab(Vocab::encode(faceExpression.c_str()));
            faceExpPort.write();          
        }
    }

}


bool GraspDetect::open(yarp::os::ResourceFinder &rf)
{   
    string ctrlName;
    string robotName;
    string remoteName;
    string localName;

    //Time::turboBoost();

    // get params from the RF    
    faceExpression = rf.check("expression", Value("evi")).asString().c_str();
    detectThreshold = rf.check("threshold",Value(GRASP_ECU_DIST)).asDouble();
    robotName = rf.check("robot",Value("icub")).asString().c_str();
    partName = rf.check("part",Value("right_arm")).asString().c_str();
    remoteName=string("/")+robotName+"/cartesianController/"+partName;
    localName=string("/graspDetect/")+partName;

    // open the client
    Property option("(device cartesiancontrollerclient)");
    option.put("remote",remoteName.c_str());
    option.put("local",localName.c_str());
    if (!driver.open(option))
        return false;

    // open the view
    driver.view(iarm);
    iarm->setTrackingMode(false);

    bool ret=true;   
    ret = targetPort.open("/graspDetect/gazeXd");    
    ret &= faceExpPort.open("/graspDetect/face:rpc");    
    ret &= objectPosPort.open("/graspDetect/xd:i");    
    ret &= graspCmdPort.open("/graspDetect/graspCmd");    

    return ret;
}

bool GraspDetect::close()
{
    iarm->stopControl();
    driver.close();

    targetPort.close();   
    faceExpPort.close();
    objectPosPort.close();
    graspCmdPort.close();
    return true;
}

bool GraspDetect::interrupt()
{
    objectPosPort.interrupt();
    return true;
}

