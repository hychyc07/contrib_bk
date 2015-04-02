/* 
 * Copyright (C)2013  iCub Facility - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
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

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>
#include <yarp/os/Stamp.h>

#include <iostream>
#include <iomanip>
#include <string.h>

#include "torqueObserver.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace std;


SensToTorques::SensToTorques(int _rate, PolyDriver *_dd):RateThread(_rate), dd(_dd)
{
    port_FT = new BufferedPort<Vector>;
    port_FT->open("/tqObs/FT:i");
    port_TAU = new BufferedPort<Vector>;
    port_TAU->open("/tqObs/TAU:o");
    port_LOG = new BufferedPort<Vector>;
    port_LOG->open("/tqObs/log:o");

    // Checking device
    dd->view(iEncs);
    dd->view(iPos);
    dd->view(iPid);
    encoders.resize(2,0.0);
    FTsensor.resize(6,0.0);
    Tau.resize(2,0.0);

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool SensToTorques::threadInit()
{         
    printf("Moving the joint to home position...");
    iPos->positionMove(0,0.0);
    Time::delay(10.0);
    printf("completed.\n");
    calibrateOffset(10);
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SensToTorques::run()
{
    readAndUpdate(false);
    timestamp.update();

    //compute torques
    double Tau_y = FTsensor[4];
    double r = 0.06; //m
    double F_x = FTsensor[0];
    Tau[0]= Tau_y + F_x*r; //@@@@@check this

    yarp::sig::Vector v;
    v.resize(7,0.0);
    v[0]=1;
    

    double enc0, enc1;
    this->iEncs->getEncoder(0,&enc0);
    this->iEncs->getEncoder(1,&enc1);
    
    Tau[1] = enc1 - this->SPsensor_offset;
    Tau[1] /= 2400.0;
    Tau[1] *= 9.3;

    for (int j=0; j<6; j++) v[j+1]=-Tau[0];
    v[2]=-Tau[1];
    port_TAU->prepare() = v;
    port_TAU->setEnvelope(this->timestamp);
    port_TAU->write();

    yarp::sig::Vector l;
    l.push_back(-Tau[0]);
    l.push_back(enc0);
    l.push_back(-Tau[1]);
    l.push_back(enc1);

    char pause[255];
    static int count = 0;
    //cout << "press key to continue"<< count++ <<endl;
    //cin >> pause;

    port_LOG->prepare() = l;
    port_LOG->setEnvelope(this->timestamp);
    port_LOG->write();
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SensToTorques::threadRelease()
{
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SensToTorques::calibrateOffset(const unsigned int Ntrials)
{
    FTsensor_offset.resize(6,0.0);
    for (int i=0; i< Ntrials; i++)
    {
        printf("calibrating FT %d \n",i);
        Vector* v = port_FT->read(true);
        FTsensor_offset+= *v;
    }
    FTsensor_offset/=Ntrials;
    printf("FT offset %s\n", FTsensor_offset.toString().c_str());

    iEncs->getEncoder(1,&SPsensor_offset);

}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void SensToTorques::readAndUpdate(bool waitMeasure)
{
    Vector* v = port_FT->read(false);
    if (v!=NULL)
    {
        FTsensor = *v;
        FTsensor -= FTsensor_offset;
    }
}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
TorqueObserver::TorqueObserver()
{
    sens2torques= NULL;
    dd        = NULL;
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool TorqueObserver::checkDriver(PolyDriver *_dd)
{
    // check driver
    if(!_dd || !(_dd->isValid()))
    {
        cerr<<"TorqueObserver: error: could not instantiate the device driver. Closing."<<endl;
        return false;
    }
    //check encoders
    IEncoders *encs;
    if(!_dd->view(encs))
    {
        cerr<<"TorqueObserver: error: could not view encoders properly. Closing."<<endl;
        return false;
    }
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool TorqueObserver::configure(ResourceFinder &rf)
{
    string moduleName;    // tqObs
    string robotName;    // icub
    string robotPart;    // left_arm, right_leg
    string limb;        // leg, arm
    string type;        // right, left
    int rate;            // rate

    // getting parameters from RF: name, robot, part, rate
    if(rf.check("name"))         moduleName = rf.find("name").asString();
    else         moduleName = "tqObs";
    if (rf.check("robot"))        robotName = rf.find("robot").asString().c_str();
    else        robotName = "icub";
    if (rf.check("rate"))        rate = rf.find("rate").asInt();
    else        rate = 10;
    if (rf.check("part"))        robotPart = rf.find("part").asString().c_str();
    else        robotPart = "right_leg";

    OptionsLimb.put("robot",robotName.c_str());
    OptionsLimb.put("part",robotPart.c_str());
    OptionsLimb.put("device","remote_controlboard");
    string s = "/";
    s+=robotName;
    s+="/";
    s+=robotPart;
    string s2 = s;
    s2+="/client";
    OptionsLimb.put("local",s2.c_str());
    OptionsLimb.put("remote",s.c_str());
    dd = new PolyDriver(OptionsLimb);
    if(!checkDriver(dd)) 
    {
        printf("TorqueObserver: error: unable to create device driver...quitting\n");
        return false;
    }


    // note: the arm needs the torso
    
    sens2torques = new SensToTorques(rate, dd);
    
    Time::delay(2.0);
    Network::connect("/icub/right_leg/analog:o","/tqObs/FT:i");
    Network::connect("/tqObs/TAU:o","/icub/joint_vsens/right_leg:i");
    //now the thread can start
    sens2torques->start();
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
double TorqueObserver::getPeriod()
{ 
    return 1;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool TorqueObserver::updateModule() 
{ 
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool TorqueObserver::close()
{
    cout<<"TorqueObserver is closing: "<<endl;
    if (sens2torques) 
    {
        sens2torques->stop();
        //delete sens2torques; 
        //sens2torques=NULL;
    }

    if (dd) {delete dd; dd=NULL;}

    Time::delay(1.0);

    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool TorqueObserver::interruptModule()
{
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
bool TorqueObserver::respond(const Bottle& command, Bottle& reply)
{
    return true;
}
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

