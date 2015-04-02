/*
 * Copyright (C) 2013 Istituto Italiano di Tecnologia
 * Author:  Elena Ceseracciu
 * email: elena.ceseracciu@iit.it
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

#include "iCub/KinestheticTeaching.h"

#include <yarp/os/Time.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;


KinTeachingThread::KinTeachingThread(int period, const yarp::os::Property& properties) : RateThread(period), initializer(properties)
{
    YARP_REGISTER_DEVICES(icubmod)
    I=NULL;
    robot=initializer.check("robot", yarp::os::Value("icub")).asString();
    name=initializer.check("threadName", yarp::os::Value("kinTeacher")).asString();
}

KinTeachingThread::~KinTeachingThread()
{
    if(I!=NULL)
        delete I;
};
    
bool KinTeachingThread::threadInit()
{

    Bottle *bImpedanceArmStiff=initializer.find("impedence_arm_stiffness").asList();
    Bottle *bImpedanceArmDamp=initializer.find("impedence_arm_damping").asList();
    
    if (bImpedanceArmDamp==NULL || bImpedanceArmStiff==NULL)
    {
        std::cerr << "Damping and stiffness values not available!" <<std::endl;
        return false;
    }
    
    std::string slash="/";
    yarp::os::Property optionArm;
    std::string partName=initializer.find("part").asString().c_str();
    optionArm.put("device","cartesiancontrollerclient");
    optionArm.put("remote",(slash + robot +"/cartesianController/" + partName).c_str());
    optionArm.put("local",(slash + name +"/cartesianClient/" + partName).c_str());

    armCartCtrlPD.open(optionArm);
    
    if (armCartCtrlPD.isValid()) 
    {
        armCartCtrlPD.view(armCartCtrl);
        armCartCtrl->storeContext(&initial_context);
    }
    else 
        return false;
    
    optionArm.put("device","remote_controlboard");
    optionArm.put("remote",(slash + robot +"/" + partName).c_str());
    optionArm.put("local",(slash + name +"/client/" + partName).c_str());

    drv_arm.open(optionArm);
    
    if (drv_arm.isValid()) 
    {
        drv_arm.view(ctrl_mode_arm);
        drv_arm.view(ctrl_impedance_arm);
        
    }
    else 
    {
        armCartCtrlPD.close();
        return false;
    }
    

   for(int i=0; i<bImpedanceArmStiff->size(); i++)
               ctrl_impedance_arm->setImpedance(i,bImpedanceArmStiff->get(i).asDouble(),bImpedanceArmDamp->get(i).asDouble());
    
    yarp::os::Property optTorso("(device remote_controlboard)");
    optTorso.put("remote",(slash+robot+"/torso").c_str());
    optTorso.put("local",(slash+name+"/torso").c_str());
    drv_torso.open(optTorso);
    if (drv_torso.isValid())
    {
        drv_torso.view(ctrl_mode_torso);
        drv_torso.view(ctrl_impedance_torso);
    }
    else
    {
        armCartCtrlPD.close();
        drv_arm.close();
        return false;
    }
    status_impedance_on=false;
    //bool impedance_from_start=false;
    bool impedance_from_start=initializer.check("impedance",Value("off")).asString()=="on";
    setImpedance(impedance_from_start);
    fprintf(stdout,"Impedance set %s\n",(status_impedance_on?"on":"off"));

    samplingRate=initializer.check("subsampling_rate",Value(500.0)).asDouble();

    Vector zeros3d(3);
    zeros3d=0.0;
    I=new iCub::ctrl::Integrator(getRate()/1000.0,zeros3d);

    actionName="";
    return true;
}
    
  
void KinTeachingThread::threadRelease()
{
    //TODO
    setImpedance(false);
    if (armCartCtrl!=NULL) 
         armCartCtrl->restoreContext(initial_context);


    armCartCtrlPD.close();
    drv_arm.close();
    drv_torso.close();
}



void KinTeachingThread::run()
{

    Vector x(3),o(4);

    armCartCtrl->getPose(x,o);

    fprintf(stdout,"curr arm pos= %s \n",x.toString().c_str());

    if(Time::now()-t0 > samplingRate)
    {
        mutex.wait();
        //add the new position to the list of actions
        Bottle &tmp_action=actions.addList();

        for(size_t i=0; i<x.size(); i++)
            tmp_action.addDouble(x[i]);

        for(size_t i=0; i<o.size(); i++)
            tmp_action.addDouble(o[i]);

        //here add timestamp to the files
        tmp_action.addDouble(t0);

        mutex.post();
        t0=Time::now();
    }

}
 /*******************************************************/
 bool KinTeachingThread::startRecordingAction(std::string actionName)
    {
        if (!mutex.check())
            return false;
                  
        this->actionName=actionName;
        actions.clear();
        
        mutex.post();
        
        if(armCartCtrl==NULL)
        {
            std::cerr << "Error! Could not find the cartesian arm interface!\n";
            return false;
        }
        
        if(!setTorque(true))
        {
            std::cerr << "Error! Could not set torque control mode\n";
            return false;
        }

        t0=yarp::os::Time::now();  
        resume();

        return true;
        
    }

/*******************************************************/
 bool KinTeachingThread::stopRecodingAction(yarp::os::Bottle& bAction)
{
    mutex.wait();
    bAction=actions;
    mutex.post();
    suspend();
    return setTorque(false);
}

/*******************************************************/
bool KinTeachingThread::setImpedance(bool turn_on)
{
    bool done=false;

    //if the system is asked to turn on impedance control
    if(turn_on)
    {
        done=true;

        for(int i=0; i<5; i++)
        {
            //if (!ctrl_mode_arm)
                //std::cout << "control mode pointer error"; fflush(stdout);
                done=done && ctrl_mode_arm->setImpedanceVelocityMode(i);
        }
        
        done=done && ctrl_mode_torso->setVelocityMode(0);
        done=done && ctrl_mode_torso->setVelocityMode(2);

        //update the system status
        status_impedance_on=done;
    }

    //if the system is asked to turn off impedance control
    if(!turn_on)
    {
        done=true;

        for(int i=0; i<5; i++)
        {
//             if (ctrl_mode_arm!=NULL)
//                 std::cout << "control mode pointer error"; fflush(stdout);

                done=done && ctrl_mode_arm->setVelocityMode(i);
        }

        for(int i=0; i<3; i++)
            if(ctrl_mode_torso!=NULL)
                done=done && ctrl_mode_torso->setVelocityMode(i);

        status_impedance_on=!done;
    }

    return done;
}

/**********************************************************/

bool KinTeachingThread::setTorque(bool turn_on)
{
    bool done=false;

    //if the system is asked to turn on impedance control
    if(turn_on)
    {
        done=true;

        for(int i=0; i<4; i++)
            done=done && ctrl_mode_arm->setTorqueMode(i);
        
        if (done) std::cout << "arm torque ok\n"; else std::cout << "error in arm torque";

        done=done && ctrl_mode_torso->setTorqueMode(0);
        done=done && ctrl_mode_torso->setImpedanceVelocityMode(2);
    }

    //if the system is asked to turn off impedance control
    if(!turn_on)
    {
        done=setImpedance(status_impedance_on);
    }

    return done;
}