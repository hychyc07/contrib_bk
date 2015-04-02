/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author: Marco Randazzo
  * email: marco.randazzo@iit.it
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

#include "iCub/skinForceControl/robot_interfaces.h"
#include <string>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace iCub::skinDynLib;
using namespace iCub::skinForceControl;

RobotInterfaces::RobotInterfaces(const char* _name, const char* _robotName): name(_name), robot(_robotName)
{
    bodyParts.resize(5);
    bodyParts[0] = TORSO;
    bodyParts[1] = LEFT_ARM;
    bodyParts[2] = RIGHT_ARM;
    bodyParts[3] = LEFT_LEG;
    bodyParts[4] = RIGHT_LEG;    
}

RobotInterfaces::RobotInterfaces(const char* _name, const char* _robotName, BodyPart bp1): name(_name), robot(_robotName)
{
    bodyParts.resize(1);
    bodyParts[0] = bp1;
}

RobotInterfaces::RobotInterfaces(const char* _name, const char* _robotName, BodyPart bp1, BodyPart bp2): 
                                    name(_name), robot(_robotName)
{
    bodyParts.resize(2);
    bodyParts[0] = bp1;
    bodyParts[1] = bp2;
}

RobotInterfaces::RobotInterfaces(const char* _name, const char* _robotName, BodyPart bp1, BodyPart bp2, BodyPart bp3): 
                                    name(_name), robot(_robotName)
{
    bodyParts.resize(3);
    bodyParts[0] = bp1;
    bodyParts[1] = bp2;
    bodyParts[2] = bp3;
}

RobotInterfaces::RobotInterfaces(const char* _name, const char* _robotName, BodyPart bp1, BodyPart bp2, 
                                   BodyPart bp3, BodyPart bp4): name(_name), robot(_robotName)
{
    bodyParts.resize(4);
    bodyParts[0] = bp1;
    bodyParts[1] = bp2;
    bodyParts[2] = bp3;
    bodyParts[3] = bp4;
}

RobotInterfaces::RobotInterfaces(const char* _name, const char* _robotName, BodyPart bp1, BodyPart bp2, 
                                   BodyPart bp3, BodyPart bp4, BodyPart bp5)
                                   : name(_name), robot(_robotName)
{
    bodyParts.resize(5);
    bodyParts[0] = bp1;
    bodyParts[1] = bp2;
    bodyParts[2] = bp3;
    bodyParts[3] = bp4;
    bodyParts[4] = bp5;
}

RobotInterfaces::RobotInterfaces(const char* _name, const char* _robotName, BodyPart *bps, int size): name(_name), robot(_robotName)
{
    bodyParts.resize(size);
    for(int i=0; i<size; i++)
        bodyParts[i] = bps[i];
}

bool RobotInterfaces::init()
{
    string part, localPort, localPort2, remotePort;
    BodyPart i;
    bool ok = true;
    for (unsigned int iii=0; iii<bodyParts.size(); iii++)
    {
        i = bodyParts[iii];
        
        ipos[i]=0;
        itrq[i]=0;
        iimp[i]=0;
        icmd[i]=0;
        ienc[i]=0;
        ipid[i]=0;
        ivel[i]=0;
        iamp[i]=0;
        iopl[i]=0;
        idbg[i]=0;
        driverRemoteControlBoard[i]=0;
        driverDebugInterface[i]=0;

        part = BodyPart_s[i];
        localPort  = "/" + name + "/" + part;
        remotePort = "/" + robot + "/" + part;
        Property options;
        options.put("robot",robot.c_str());
        options.put("part",part.c_str());
        options.put("device","remote_controlboard");
        options.put("local",localPort.c_str());
        options.put("remote",remotePort.c_str());
        options.put("carrier", "udp");
        driverRemoteControlBoard[i] = new PolyDriver(options);
        if(!driverRemoteControlBoard[i] || !(driverRemoteControlBoard[i]->isValid()))
            fprintf(stderr,"Problems instantiating driver remote_controlboard for body part %s\n", part.c_str());
        ok = ok & driverRemoteControlBoard[i]->view(ipos[i]);
        ok = ok & driverRemoteControlBoard[i]->view(itrq[i]);
        ok = ok & driverRemoteControlBoard[i]->view(iimp[i]);
        ok = ok & driverRemoteControlBoard[i]->view(icmd[i]);
        ok = ok & driverRemoteControlBoard[i]->view(ivel[i]);
        ok = ok & driverRemoteControlBoard[i]->view(ienc[i]);
        ok = ok & driverRemoteControlBoard[i]->view(ipid[i]);
        ok = ok & driverRemoteControlBoard[i]->view(iamp[i]);
        ok = ok & driverRemoteControlBoard[i]->view(iopl[i]);

#ifndef _SIMULATION
        localPort2=localPort;
		// the following complex line of code performs a substring substitution (see example below)
		// "/icub/robotMotorGui2/right_arm" -> "/icub/robotMotorGui2/debug/right_arm"
		localPort2.replace( localPort2.find(part), part.length(), string("debug/")+part);
        options.clear();
        options.put("robot",robot.c_str());
        options.put("part",part.c_str());
        options.put("device","debugInterfaceClient");
        options.put("local",localPort2.c_str());
        options.put("remote",remotePort.c_str());
        options.put("carrier", "udp");
        driverDebugInterface[i] = new PolyDriver(options);
        if(!driverDebugInterface[i] || !(driverDebugInterface[i]->isValid()))
            fprintf(stderr,"Problems instantiating driver debugInterfaceClient for body part %s\n", part.c_str());
        ok = ok & driverDebugInterface[i]->view(idbg[i]);
#endif
    }
    return ok;
}

bool RobotInterfaces::close(){
    BodyPart i;
    bool ok = true;
    for (unsigned int iii=0; iii<bodyParts.size(); iii++)
    {
        i = bodyParts[iii];
        if(driverRemoteControlBoard[i])
        {
            ok = ok && driverRemoteControlBoard[i]->close();
            delete driverRemoteControlBoard[i];
            driverRemoteControlBoard[i] = 0;
        }
        if(driverDebugInterface[i])
        {
            ok = ok && driverDebugInterface[i]->close();
            delete driverDebugInterface[i];
            driverDebugInterface[i] = 0;
        }
    }
    return ok;
}
