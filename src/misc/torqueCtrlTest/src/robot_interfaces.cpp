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

#include "iCub/torqueCtrlTest/robot_interfaces.h"
#include <yarp/os/Stamp.h>
#include <yarp/os/Log.h>
#include <yarp/dev/ControlBoardHelpers.h>
#include <string>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace iCub::skinDynLib;
using namespace iCub::torqueCtrlTest;

robot_interfaces::robot_interfaces(const char* _name, const char* _robotName): name(_name), robot(_robotName)
{
    bodyParts.resize(5);
    bodyParts[0] = TORSO;
    bodyParts[1] = LEFT_ARM;
    bodyParts[2] = RIGHT_ARM;
    bodyParts[3] = LEFT_LEG;
    bodyParts[4] = RIGHT_LEG;
}

robot_interfaces::robot_interfaces(const char* _name, const char* _robotName, BodyPart bp1): name(_name), robot(_robotName)
{
    bodyParts.resize(1);
    bodyParts[0] = bp1;
}

robot_interfaces::robot_interfaces(const char* _name, const char* _robotName, BodyPart bp1, BodyPart bp2): 
                                    name(_name), robot(_robotName)
{
    bodyParts.resize(2);
    bodyParts[0] = bp1;
    bodyParts[1] = bp2;
}

robot_interfaces::robot_interfaces(const char* _name, const char* _robotName, BodyPart bp1, BodyPart bp2, BodyPart bp3): 
                                    name(_name), robot(_robotName)
{
    bodyParts.resize(3);
    bodyParts[0] = bp1;
    bodyParts[1] = bp2;
    bodyParts[2] = bp3;
}

robot_interfaces::robot_interfaces(const char* _name, const char* _robotName, BodyPart bp1, BodyPart bp2, 
                                   BodyPart bp3, BodyPart bp4): name(_name), robot(_robotName)
{
    bodyParts.resize(4);
    bodyParts[0] = bp1;
    bodyParts[1] = bp2;
    bodyParts[2] = bp3;
    bodyParts[3] = bp4;
}

robot_interfaces::robot_interfaces(const char* _name, const char* _robotName, BodyPart bp1, BodyPart bp2, 
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

robot_interfaces::robot_interfaces(const char* _name, const char* _robotName, BodyPart *bps, int size): name(_name), robot(_robotName)
{
    bodyParts.resize(size);
    for(int i=0; i<size; i++)
        bodyParts[i] = bps[i];
}

bool robot_interfaces::init()
{
    std::string part;    
    std::string localPort;
    std::string remotePort;
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
        ienct[i]=0;
        ipid[i]=0;
        ivel[i]=0;
        iamp[i]=0;
        iopl[i]=0;
        dd[i]=0;

        part = BodyPart_s[i];
        localPort  = "/" + name + "/" + part;
        remotePort = "/" + robot + "/" + part;
        options[i].put("robot",robot.c_str());
        options[i].put("part",part.c_str());
        options[i].put("device","remote_controlboard");
        options[i].put("local",localPort.c_str());
        options[i].put("remote",remotePort.c_str());

        dd[i] = new PolyDriver(options[i]);
        if(!dd[i] || !(dd[i]->isValid()))
            fprintf(stderr,"Problems instantiating the device driver %s\n", part.c_str());
        
        ok = ok & dd[i]->view(ipos[i]);
        ok = ok & dd[i]->view(itrq[i]);
        ok = ok & dd[i]->view(iimp[i]);
        ok = ok & dd[i]->view(icmd[i]);
        ok = ok & dd[i]->view(ivel[i]);
        ok = ok & dd[i]->view(ienc[i]);
        ok = ok & dd[i]->view(ienct[i]);
        ok = ok & dd[i]->view(ipid[i]);
        ok = ok & dd[i]->view(iamp[i]);
        ok = ok & dd[i]->view(iopl[i]);
        if(!ok)
            printf("Problem initializing drivers of %s\n", part.c_str());
        
        if(!rpcPorts[i].open((localPort+"_rpc").c_str()))
            printf("Problem while opening rpc port for %s.\n", part.c_str());
        else
            ok = ok && Network::connect((remotePort+"/rpc:i").c_str(), (localPort+"_rpc").c_str());

        gravityPorts[i] = new BufferedPort<Vector>;
        ok = ok & gravityPorts[i]->open(("/"+name+"/"+part+"_gravity_torques:i").c_str());
        ok = ok & Network::connect(("/gravityCompensator/"+part+"_torques:o").c_str(), gravityPorts[i]->getName().c_str());
        Vector *temp;
        int counter = 10;
        while( (temp=gravityPorts[i]->read(false))==0 && counter>0) { counter--; }
        if(temp!=0)
            gravityTorques[i] = *temp;
        else
            printf("Problem reading gravity torques for %s\n", part.c_str());
    }

    return ok;
}

bool robot_interfaces::getTimeStamp(Bottle &bot, Stamp &st)
{
    if (bot.get(3).asVocab()==VOCAB_TIMESTAMP)
    {
        //yup! we have a timestamp
        int fr=bot.get(4).asInt();
        double ts=bot.get(5).asDouble();
        st=Stamp(fr,ts);
        return true;
    }
    return false;
}

bool robot_interfaces::readPwm(BodyPart bp, double *pwm, double *timestamps)
{
    Bottle cmd, response;
    cmd.addVocab(VOCAB_GET);
    cmd.addVocab(VOCAB_OUTPUTS);
    bool ok = rpcPorts[bp].write(cmd, response);
    if (CHECK_FAIL(ok, response)) 
    {
        Bottle& l = *(response.get(2).asList());
        if (&l == 0)
            return false;

        for (int i = 0; i < l.size(); i++)
            pwm[i] = l.get(i).asDouble();

        Stamp stamp;
        if(getTimeStamp(response, stamp))
            timestamps[0] = stamp.getTime();

        return true;
    }
    return false;
}