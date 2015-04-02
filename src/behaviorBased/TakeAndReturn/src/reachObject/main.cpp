/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

/** 
\defgroup cartesianInterfaceExample cartesianInterfaceExample
 
@ingroup icub_module  
 
Just an example of how to control the iCub arm in the 
operational space making use of the Cartesian Interface.
 
Copyright (C) 2010 RobotCub Consortium
 
Author: Ugo Pattacini 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
 
This module relies on the <a 
href="http://eris.liralab.it/iCub/main/dox/html/icub_cartesian_interface.html">Cartesian 
Interface</a> to provide a kind of easy-to-use port-based 
front-end for controlling the iCub's arm in the operational 
space. 
 
\section lib_sec Libraries 
- YARP libraries. 

\section parameters_sec Parameters
--ctrlName \e name 
- The parameter \e name identifies the controller's name; all 
  the open ports will be tagged with the prefix
  /<ctrlName>/<part>/. If not specified \e armCtrl is assumed.
 
--robot \e name 
- The parameter \e name selects the robot name to connect to; if
  not specified \e icub is assumed.
 
--part \e type 
- The parameter \e type selects the robot's arm to work with. It
  can be \e right_arm or \e left_arm; if not specified
  \e right_arm is assumed.
 
--T \e time
- specify the task execution time in seconds; by default \e time
  is 2.0 seconds. Note that this is just an approximation of
  execution time since there exists a controller running
  underneath.
 
--DOF8
- enable the control of torso yaw joint. 
 
--DOF9
- enable the control of torso yaw/pitch joints. 
 
--DOF10
- enable the control of torso yaw/roll/pitch joints. 
 
--onlyXYZ  
- disable orientation control. 
 
\section portsa_sec Ports Accessed
 
Assumes that \ref icub_iCubInterface (with ICartesianControl 
interface implemented) is running. 
 
\section portsc_sec Ports Created 
 
The module creates the ports required for the communication with
the robot (through interfaces) and the following ports: 
 
- \e /<ctrlName>/<part>/xd:i receives the target end-effector 
  pose. It accepts 7 double (also as a Bottle object): 3 for xyz
  and 4 for orientation in axis/angle mode.

- \e /<ctrlName>/<part>/rpc remote procedure call. \n
    Recognized remote commands: \n
    -'quit' quit the module

\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files 
None. 
 
\section conf_file_sec Configuration Files
None. 
 
\section tested_os_sec Tested OS
Windows, Linux

\author Ugo Pattacini
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>


#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IImpedanceControl.h>

#include <gsl/gsl_math.h>

#include <string>
#include <stdio.h>

#define MAX_TORSO_PITCH     60.0    // [deg]
#define EXECTIME_THRESDIST  0.3     // [m]
#define PRINT_STATUS_PER    1.0     // [s]

#define MIN_X_RANGE         -0.30   // 30cm


YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


/************************************************************************/
class CtrlThread: public RateThread
{
protected:
    ResourceFinder      &rf;
    PolyDriver          driver;
    PolyDriver          boardDriver;

    ICartesianControl   *iarm;
    IControlMode      *imode;
    IImpedanceControl *iimp;

    BufferedPort<Bottle> port_xd;

    bool ctrlCompletePose;
    string remoteName;
    string localName;

    int startup_context_id;

    Vector xd;
    Vector od;

    double defaultExecTime;
    double t0;

    bool impVelMode;
    Vector armJointsStiffness;
    Vector armJointsDamping;
    
public:
    /************************************************************************/
    CtrlThread(unsigned int _period, ResourceFinder &_rf,
               string _remoteName, string _localName) :
               RateThread(_period),     rf(_rf),
               remoteName(_remoteName), localName(_localName) {}

    /************************************************************************/
    bool threadInit()
    {

        // stiffness and damping
        armJointsStiffness.resize(5,0.0);
        armJointsDamping.resize(5,0.0);
        Bottle &b = rf.findGroup("arm_params");
        impVelMode = b.check("impedance_velocity_mode",Value("off"),"Getting arm impedance-velocity-mode").asString()=="on"?true:false;
        if (b.check("impedance_stiffness","Getting joints stiffness"))
        {
            Bottle &grp = b.findGroup("impedance_stiffness");
            size_t sz=grp.size()-1;
            size_t len=sz>armJointsStiffness.length()?armJointsStiffness.length():sz;

            for (size_t i=0; i<len; i++)
                armJointsStiffness[i]=grp.get(1+i).asDouble();
        }

        if(b.check("impedance_damping","Getting joints damping"))
        {
            Bottle &grp=b.findGroup("impedance_damping");
            size_t sz=grp.size()-1;
            size_t len=sz>armJointsDamping.length()?armJointsDamping.length():sz;

            for (size_t i=0; i<len; i++)
                armJointsDamping[i]=grp.get(1+i).asDouble();
        }

        // get params from the RF
        if (rf.check("onlyXYZ"))
            ctrlCompletePose=false;
        else
            ctrlCompletePose=true;


        
        if(impVelMode)
        {
            string robotName = rf.check("robot",Value("icub")).asString().c_str();
            string partName = rf.check("part",Value("right_arm")).asString().c_str();
            string local = string("/reachObject/board/")+partName;
            string remote = string("/")+robotName+string("/")+partName;
            
            Property opt;
            opt.put("device", "remote_controlboard");
            opt.put("local", local.c_str());   //local port names
            opt.put("remote", remote.c_str());         //where we connect to
            // create a device
            if(!boardDriver.open(opt))
                return false;

            bool ok = boardDriver.view(imode);
            ok &= boardDriver.view(iimp);
            if (!ok) 
            {
                printf("Problems acquiring interfaces\n");
                boardDriver.close();
                return false;
            }

            int len=armJointsStiffness.length()<armJointsDamping.length()?
                    armJointsStiffness.length():armJointsDamping.length();
            for (int j=0; j<len; j++)
            {
                printf("stiff:%.2f, damp:%.2f\n", armJointsStiffness[j], armJointsDamping[j]);
                imode->setImpedanceVelocityMode(j);
                iimp->setImpedance(j, armJointsStiffness[j], armJointsDamping[j]);
            }
        }        

        // open the client
        Property option("(device cartesiancontrollerclient)");
        option.put("remote",remoteName.c_str());
        option.put("local",localName.c_str());
        if (!driver.open(option))
        {
            boardDriver.close();
            return false;
        }            

        // open the view
        driver.view(iarm);
        // latch the controller context
        iarm->storeContext(&startup_context_id);
        // set trajectory time
        defaultExecTime=rf.check("T",Value(2.0)).asDouble();

        // set torso dofs
        Vector newDof, curDof;
        iarm->getDOF(curDof);
        newDof=curDof;

        if (rf.check("DOF10"))
        {    
            // torso joints completely enabled
            newDof[0]=1;
            newDof[1]=1;
            newDof[2]=1;

            limitTorsoPitch();
        }
        else if (rf.check("DOF9"))
        {    
            // torso yaw and pitch enabled
            newDof[0]=1;
            newDof[1]=0;
            newDof[2]=1;

            limitTorsoPitch();
        }
        else if (rf.check("DOF8"))
        {                
            // only torso yaw enabled
            newDof[0]=0;
            newDof[1]=0;
            newDof[2]=1;
        }
        else
        {    
            // torso joints completely disabled
            newDof[0]=0;
            newDof[1]=0;
            newDof[2]=0;
        }

        iarm->setDOF(newDof,curDof);

        // set tracking mode
        iarm->setTrackingMode(false);

        // init variables
        while (true)
        {
            if (iarm->getPose(xd,od))
                break;

            Time::delay(0.1);
        }

        // open ports
        port_xd.open("/reachObject/xd:i");

        return true;
    }

    /************************************************************************/
    void afterStart(bool s)
    {
        if (s)
            printf("Thread started successfully\n");
        else
            printf("Thread did not start\n");

        t0=Time::now();
    }

    /************************************************************************/
    void run()
    {
        if (Bottle *b=port_xd.read(false))
        {                
            if (b->size()>=3)
            {                   
                for (int i=0; i<3; i++)
                    xd[i]=b->get(i).asDouble();
                
                if (ctrlCompletePose && b->size()>=7)
                {    
                    for (int i=0; i<4; i++)
                        od[i]=b->get(3+i).asDouble();
                }

                const double execTime=calcExecTime(xd);

                // the simplest self-body colision  avoiadance 
                xd[0] = (xd[0]>MIN_X_RANGE) ? MIN_X_RANGE :  xd[0];

                if (ctrlCompletePose)
                    iarm->goToPose(xd,od,execTime);
                else
                    iarm->goToPosition(xd,execTime);

                printf("dbg 0: (%s)\n",xd.toString(3,3).c_str());
            }
        }

        //printStatus();
    }

    /************************************************************************/
    void threadRelease()
    {   
        if(impVelMode)
        {
            IControlMode *imode;
            boardDriver.view(imode);
            int len=armJointsStiffness.length()<armJointsDamping.length()?
                    armJointsStiffness.length():armJointsDamping.length();
            for (int j=0; j<len; j++)
                imode->setVelocityMode(j);
        }
        boardDriver.close();

        iarm->stopControl();
        iarm->restoreContext(startup_context_id);
        driver.close();

        port_xd.interrupt();
        port_xd.close();
    }

    /************************************************************************/
    void limitTorsoPitch()
    {
        int axis=0; // pitch joint
        double min, max;

        iarm->getLimits(axis,&min,&max);
        iarm->setLimits(axis,min,MAX_TORSO_PITCH);
    }

    /************************************************************************/
    double calcExecTime(const Vector &xd)
    {
        /*
        Vector x,o;
        iarm->getPose(x,o);
        
        printf("\n\nNORM:%.2f\n\n", norm(xd-x));

        if (norm(xd-x)<EXECTIME_THRESDIST)
            return defaultExecTime;
        else
            return 1.5*defaultExecTime;
        */            
        return defaultExecTime;
    }

    /************************************************************************/
    void printStatus()
    {
        double t=Time::now();

        if (t-t0>=PRINT_STATUS_PER)
        {
            Vector x,o,xdot,odot;
            Vector xdhat,odhat,qdhat;

            iarm->getPose(x,o);
            iarm->getTaskVelocities(xdot,odot);
            iarm->getDesired(xdhat,odhat,qdhat);
            double e_x=norm(xdhat-x);

            printf("xd          [m]   = %s\n",xd.toString().c_str());
            printf("xdhat       [m]   = %s\n",xdhat.toString().c_str());
            printf("x           [m]   = %s\n",x.toString().c_str());
            printf("xdot        [m/s] = %s\n",xdot.toString().c_str());
            printf("norm(e_x)   [m]   = %g\n",e_x);

            if (ctrlCompletePose)
            {
                double e_o=norm(odhat-o);

                printf("od        [rad]   = %s\n",od.toString().c_str());
                printf("odhat     [rad]   = %s\n",odhat.toString().c_str());
                printf("o         [rad]   = %s\n",o.toString().c_str());
                printf("odot      [rad/s] = %s\n",odot.toString().c_str());
                printf("norm(e_o) [rad]   = %g\n",e_o);
            }

            printf("\n");

            t0=t;
        }
    }
};


/************************************************************************/
class CtrlModule: public RFModule
{
protected:
    CtrlThread *thr;
    Port        rpcPort;

public:
    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string slash="/";
        string ctrlName;
        string robotName;
        string partName;
        string remoteName;
        string localName;

        Time::turboBoost();

        // get params from the RF
        ctrlName=rf.check("ctrlName",Value("reachObject")).asString().c_str();
        robotName=rf.check("robot",Value("icub")).asString().c_str();
        partName=rf.check("part",Value("right_arm")).asString().c_str();

        remoteName=slash+robotName+"/cartesianController/"+partName;
        localName=slash+ctrlName+slash+partName;

        thr=new CtrlThread(20,rf,remoteName,localName);
        if (!thr->start())
        {
            delete thr;
            return false;
        }

        rpcPort.open("/reachObject/rpc");
        attach(rpcPort);

        return true;
    }

    /************************************************************************/
    bool close()
    {
        thr->stop();
        delete thr;

        rpcPort.interrupt();
        rpcPort.close();

        return true;
    }

    /************************************************************************/
    double getPeriod()
    {
        return 1.0;
    }

    /************************************************************************/
    bool updateModule()
    {
        return true;
    }
};


/************************************************************************/
int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("reachObject/conf");
    rf.setDefaultConfigFile("config.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help"))
    {
        printf("Options:\n\n");
        printf("\t--ctrlName name: controller name (default armCtrl)\n");
        printf("\t--robot    name: robot name to connect to (default: icub)\n");
        printf("\t--part     type: robot arm type, left_arm or right_arm (default: right_arm)\n");
        printf("\t--T        time: specify the task execution time in seconds (default: 2.0)\n");
        printf("\t--DOF10        : control the torso yaw/roll/pitch as well\n");
        printf("\t--DOF9         : control the torso yaw/pitch as well\n");
        printf("\t--DOF8         : control the torso yaw as well\n");
        printf("\t--onlyXYZ      : disable orientation control\n");

        return 0;
    }

    Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("YARP server not available!\n");
        return -1;
    }

    YARP_REGISTER_DEVICES(icubmod)

    CtrlModule mod;
    return mod.runModule(rf);
}



