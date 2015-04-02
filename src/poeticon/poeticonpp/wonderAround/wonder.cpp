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
#include "wonder.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

#define GAZE_REST_X               -7.0
#define GAZE_REST_Y               0.0
#define GAZE_REST_Z               0.0

#define GAZE_X_MIN                -3.0
#define GAZE_X_MAX                -2.0
#define GAZE_Y_MIN                -1.0
#define GAZE_Y_MAX                1.0
#define GAZE_Z_MIN                0.0
#define GAZE_Z_MAX                0.7	

 
const double RIGHT_RELAX_POS[7] = { -0.29, 0.20, 0.01, -0.145, -0.78, 0.6, 3.0 }; 
const double LEFT_RELAX_POS[7] = { -0.29, -0.20, 0.01, 0.0, -0.707, 0.707, 3.1 }; 


void Wonder::loop()
{

    Bottle &target=targetPort.prepare();
    target.clear();
    target.addDouble(Rand::scalar(GAZE_X_MIN,GAZE_X_MAX));
    target.addDouble(Rand::scalar(GAZE_Y_MIN,GAZE_Y_MAX));
    target.addDouble(Rand::scalar(GAZE_Z_MIN,GAZE_Z_MAX));
    targetPort.write();

    if(rightArmEnabled)
    {
        Bottle &rightRelax = rightArmRelaxPort.prepare();
        rightRelax.clear();
        for(int i=0; i<7; i++)
            rightRelax.addDouble(RIGHT_RELAX_POS[i]);
        rightArmRelaxPort.write();
    }

    if(leftArmEnabled)
    {
        Bottle &leftRelax = leftArmRelaxPort.prepare();
        leftRelax.clear();
        for(int i=0; i<7; i++)
            leftRelax.addDouble(LEFT_RELAX_POS[i]);
        leftArmRelaxPort.write();
    }


    Bottle &cmd=faceExpPort.prepare();
    cmd.addVocab(Vocab::encode("set"));
	cmd.addVocab(Vocab::encode("all"));
	cmd.addVocab(Vocab::encode(faceExpression.c_str()));
	faceExpPort.write();

    Bottle &grasp = graspCmdPort.prepare();
    grasp.clear();
    grasp.addString("release");
    graspCmdPort.write();
}



bool Wonder::open(yarp::os::ResourceFinder &rf)
{
    faceExpression = rf.check("expression", Value("shy")).asString().c_str();
    string partName = rf.check("arm",Value("none")).asString().c_str();

    rightArmEnabled = leftArmEnabled = false;

    if(partName == "left")
        leftArmEnabled = true;
    else if(partName == "right")
        rightArmEnabled = true;
    else if(partName == "both")
    {
        rightArmEnabled = leftArmEnabled = true;
    }

    Rand::init();

    bool ret=true;   
    ret = targetPort.open("/wonderAround/gazeXd");    
    ret &= faceExpPort.open("/wonderAround/face:rpc");
    ret &= leftArmRelaxPort.open("/wonderAround/leftArmXd");    
    ret &= rightArmRelaxPort.open("/wonderAround/rightArmXd");    
    ret &= graspCmdPort.open("/wonderAround/graspCmd");    

    return ret;
}

bool Wonder::close()
{
    
    gazeRest();

    if(rightArmEnabled)
    {
        Bottle &rightRelax = rightArmRelaxPort.prepare();
        rightRelax.clear();
        for(int i=0; i<7; i++)
            rightRelax.addDouble(RIGHT_RELAX_POS[i]);
        rightArmRelaxPort.setStrict();
        rightArmRelaxPort.write();
        rightArmRelaxPort.close();
    }

    if(leftArmEnabled)
    {
        Bottle &leftRelax = leftArmRelaxPort.prepare();
        leftRelax.clear();
        for(int i=0; i<7; i++)
            leftRelax.addDouble(LEFT_RELAX_POS[i]);          
        leftArmRelaxPort.setStrict();
        leftArmRelaxPort.write();
        leftArmRelaxPort.close();
    }
 
    targetPort.close();   
    faceExpPort.close();
    graspCmdPort.close();
    return true;
}

bool Wonder::interrupt()
{
    return true;
}

void Wonder::gazeRest()
{
    Bottle &target=targetPort.prepare();
    target.clear();
    target.addDouble(GAZE_REST_X);
    target.addDouble(GAZE_REST_Y);
    target.addDouble(GAZE_REST_Z);
    targetPort.setStrict();
    targetPort.write();    
}   


