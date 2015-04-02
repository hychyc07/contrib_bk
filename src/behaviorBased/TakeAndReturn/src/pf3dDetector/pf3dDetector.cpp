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

#include "pf3dDetector.h"
#include <iCub/iKin/iKinFwd.h>

#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::iKin;

#define REACH_OFFSET_Y      0.05    // 5cm 
#define REACH_OFFSET_Z      0.00 
#define MIN_X_RANGE         -0.30   // 30cm
#define MAX_X_RANGE         -0.80   // 80cm
#define MIN_Z_RANGE         0.05    // 5cm
#define MAX_Z_RANGE         0.50    // 50cm


const double RIGHT_ROT_POS[4] = {-0.14, -0.79, 0.59, 3.06}; 
const double LEFT_ROT_POS[4] = {-0.03, 0.6, -0.79, 2.85};

void Pf3dDetector::loop()
{
    Bottle *posEye = inputPort.read();
    if(posEye)
    {
        if((posEye->size()>6) && (posEye->get(6).asDouble()==1.0) )
        {
            Vector fp(4);
            fp[0]=posEye->get(0).asDouble();
            fp[1]=posEye->get(1).asDouble();
            fp[2]=posEye->get(2).asDouble();
            fp[3]=1.0;
            Vector x,o;
            if (eye == "left")
                iGaze->getLeftEyePose(x,o);
            else
                iGaze->getRightEyePose(x,o);

            Matrix T = axis2dcm(o);
            T(0,3)=x[0];
            T(1,3)=x[1];
            T(2,3)=x[2];
            Vector posRoot = T*fp;

            //printf("dbg 0:%s\n",T.toString(3,3).c_str());            

            Bottle &target=targetPort.prepare();
            target.clear();

            target.addDouble(posRoot[0]+x_offset);
            if(leftArmEnabled)
                target.addDouble(posRoot[1]-fabs(y_offset));
            else
                target.addDouble(posRoot[1]+fabs(y_offset));            
            target.addDouble(posRoot[2]+z_offset);

            if(leftArmEnabled)
            {
                for(int i=0; i<4; i++)
                  target.addDouble(LEFT_ROT_POS[i]);
            }
            else
            {
                for(int i=0; i<4; i++)
                  target.addDouble(RIGHT_ROT_POS[i]);
            }

            Bottle &cmd=faceExpPort.prepare();
            cmd.addVocab(Vocab::encode("set"));
            cmd.addVocab(Vocab::encode("all"));
            cmd.addVocab(Vocab::encode(faceExpression.c_str()));

            if((posRoot[0] <= MIN_X_RANGE)  && (posRoot[0] >= MAX_X_RANGE) && 
              (posRoot[2] >= MIN_Z_RANGE)  && (posRoot[2] <= MAX_Z_RANGE) )  
            {
                targetPort.write();                
                faceExpPort.write();
            }                
        }
    }
}

bool Pf3dDetector::open(yarp::os::ResourceFinder &rf)
{
    
    string name = rf.check("name",Value("pf3dDetector")).asString().c_str();
    string robot = rf.check("robot",Value("icub")).asString().c_str();
    eye = rf.check("eye",Value("left")).asString().c_str();
    leftArmEnabled = (rf.check("part",Value("right_arm")).asString() != "right_arm");
    faceExpression = rf.check("expression", Value("hap")).asString().c_str();

    x_offset = rf.check("x_offset",Value("0.0")).asDouble();
    y_offset = rf.check("y_offset",Value("0.05")).asDouble();
    z_offset = rf.check("z_offset",Value("0.0")).asDouble();


    Property optGaze("(device gazecontrollerclient)");
    optGaze.put("remote","/iKinGazeCtrl");
    optGaze.put("local",("/"+name+"/gazeClient").c_str());

    if (!clientGaze.open(optGaze))
        return false;

    clientGaze.view(iGaze);
    iGaze->blockNeckRoll(0.0);
    

    bool ret=true; 
    ret = targetPort.open("/pf3dDetector/gazeXd");    
    ret &= inputPort.open("/pf3dDetector/data/in");    
    ret &= faceExpPort.open("/pf3dDetector/face:rpc"); 
    return ret;
}

bool Pf3dDetector::close()
{
    clientGaze.close();

    targetPort.close();   
    inputPort.close();
    faceExpPort.close();
    return true;
}

bool Pf3dDetector::interrupt()
{
    inputPort.interrupt();
    return true;
}


