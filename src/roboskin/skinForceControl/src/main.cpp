/**
@ingroup icub_module
\defgroup skinForceControl skinForceControl
Control the robot arm using the F/T sensor and the skin. 
Copyright (C) 2008 RobotCub Consortium
Author: Andrea Del Prete
Date: first release 08/2011
CopyPolicy: Released under the terms of the GNU GPL v2.0.

\author Andrea Del Prete
*/ 

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Network.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/GazeControl.h>
#include <iCub/ctrl/math.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

#include <iostream>
#include <sstream>
#include <iomanip>
#include <string.h>

#include "iCub/skinForceControl/controlConstants.h"
#include "iCub/skinForceControl/controlThread.h"
#include "iCub/skinForceControl/controlPlanner.h"

YARP_DECLARE_DEVICES(icubmod)


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace std;

namespace iCub
{

namespace skinForceControl
{

class controlModule: public RFModule
{
private:
    
    double avgTime, stdDev, avgTimeUsed, stdDevUsed; 
    double period, gazePeriod;
	controlThread *contrThread;
    controlPlanner *contrPlanner;
	PolyDriver *dd_gaze;
    IGazeControl *igaze;
    BodyPart bodyPart;
    bool gaze_enabled;
    int gazeCtrlContext;
    Port rpcPort;        	

public:
    controlModule()
    {
		bodyPart			= BODY_PART_UNKNOWN;
		igaze               = 0;
        gaze_enabled        = false;
        contrThread         = 0;
        contrPlanner        = 0;
        period              = 10;
    }
    

    bool configure(ResourceFinder &rf)
    {		
		string fwdSlash = "/";

        //-----------------GET THE MODULE NAME-------------------//
        string name = "skinForceControl";
        if (rf.check("name"))
            name = rf.find("name").asString().c_str();
        setName(name.c_str());
        
        //-----------------GET THE PERIOD-------------------//
        period = DEFAULT_CTRL_PERIOD;
        if (rf.check("period"))
            period = rf.find("period").asInt();
        gazePeriod = DEFAULT_GAZE_PERIOD;
        if (rf.check("gaze_period"))
            gazePeriod = rf.find("gaze_period").asInt();

		//-----------------GET THE ROBOT NAME-------------------//
		string robot_name = "icub";
		if (rf.check("robot"))
            robot_name = rf.find("robot").asString().c_str();

        //-----------------GET WHOLE BODY TORQUE OBSERVER NAME-------------------//
		string wholeBodyName = "wholeBodyDynamics";
		if (rf.check("wholeBodyName"))
            wholeBodyName = rf.find("wholeBodyName").asString().c_str();

		//------------------CHECK WHICH ARM IS ENABLED-----------//
		if (rf.check("left_arm")){
			bodyPart = LEFT_ARM;
			fprintf(stderr,"'left_arm' option found. Left arm will be enabled.\n");
		}
        else if (rf.check("right_arm")){
			bodyPart = RIGHT_ARM;
			fprintf(stderr,"'right_arm' option found. Right arm will be enabled.\n");
		}
        else{
			bodyPart = LEFT_ARM;
            fprintf(stderr, "No arm specified. Left arm will be used by default.\n");
        }

        //---------------------DEVICES--------------------------//
		Property OptionGaze;
        OptionGaze.put("device", "gazecontrollerclient");
        OptionGaze.put("remote", "/iKinGazeCtrl");
        OptionGaze.put("local", (fwdSlash+name+"/gaze_client").c_str());         
        dd_gaze = new PolyDriver(OptionGaze);
        if (dd_gaze->isValid()) {
            if(!dd_gaze->view(igaze))
               fprintf(stderr, "WARNING: unable to create gaze device driver! Going on anyway.\n");
            else{
                igaze->storeContext(&gazeCtrlContext);
                igaze->blockNeckRoll(0.0);
                igaze->setNeckTrajTime(2.5);
                igaze->setEyesTrajTime(2.5);
            }
        }else
			fprintf(stderr, "WARNING: unable to create gaze device driver! Going on anyway.\n");

        // by default disable the gaze control
        gaze_enabled = false;
        if (rf.check("gaze_on")){
			gaze_enabled = true;
			fprintf(stderr,"Gaze will be enabled.\n");
		}

        //---------------------RPC PORT--------------------------//
        rpcPort.open(("/"+name+"/rpc").c_str());
        attach(rpcPort);

        //--------------------------THREAD--------------------------
        DSemaphore::registerThread("Module");
        contrThread = new controlThread(name, robot_name, (int)period, wholeBodyName, bodyPart, VERBOSE);
        fprintf(stderr,"control thread istantiated...\n");
        contrPlanner = new controlPlanner(name+"Planner", contrThread, (int)period*2, VERBOSE);
        fprintf(stderr,"control planner istantiated...\n");
        if(!contrThread->start()){
            fprintf(stderr, "Error while initializing control thread. Closing module.\n");
            return false;
        }
        fprintf(stderr,"control thread started\n");
        if(!contrPlanner->start()){
            fprintf(stderr, "Error while initializing control planner. Closing module.\n");
            return false;
        }
        fprintf(stderr,"control planner started\n");

        Time::delay(1.0);
		/*contrThread->setNewFirmwarePidGainsBlack();*/
        contrThread->setNewFirmwarePidGainsWhite();
		contrThread->setSimMode(false);

		if (rf.check("float")){
			Time::delay(1.0);
			contrThread->setNewFirmwarePidGainsWhite();
			contrThread->setSimMode(false);
		    contrThread->setCtrlLaw(FLOAT_CTRL);
	    }
        return true;
    }

    bool respond(const Bottle& command, Bottle& reply) 
    {
	    unsigned int cmdId;
        Bottle param;
        vector<string> cl(SkinForceCtrlCommand_s, SkinForceCtrlCommand_s+SkinForceCtrlCommandSize);
	    if(!identifyCommand(command, cl, cmdId, param)){
            if( contrPlanner->respond(command, reply) )
                return true;
            reply.clear();
            if( contrThread->respond(command, reply) )
                return true;
            reply.clear();
		    reply.addString("Unknown command. Input 'help' to get a list of the available commands.");
		    return true;
	    }

	    switch( cmdId ){
		    case quit:          reply.addString("quitting");    return false;
		    case help:
                {
                    reply.addVocab(Vocab::encode("many"));  // print every string added to the bottle on a new line
                    reply.addString((string(getName().c_str()) + " commands are: ").c_str());
                    vector<string> cd = getCommandDesc();
                    cl = getCommandList();
			        for(unsigned int i=0; i< cl.size(); i++)
				        reply.addString( ("- "+cl[i]+": "+cd[i]).c_str() );
			        return true;
                }
            case gaze_on:   
                gaze_enabled = true; 
                break;
            case gaze_off:  
                gaze_enabled = false;
                if(igaze)
                    igaze->lookAtFixationPoint(buildVector(3, -5.0, 0.0, 0.5));
                break;
		    default: 
                reply.addString("ERROR: This command is known but it is not managed in the code."); 
                return true;
	    }

	    reply.addString( (SkinForceCtrlCommand_s[cmdId]+" command received.").c_str());
	    return true;	
    }

    vector<string> getCommandList(){
        vector<string> cl(SkinForceCtrlCommand_s, SkinForceCtrlCommand_s+SkinForceCtrlCommandSize);
        vector<string> cl_2 = contrPlanner->getCommandList();
        cl.insert(cl.end(), cl_2.begin(), cl_2.end());
        cl_2 = contrThread->getCommandList();
        cl.insert(cl.end(), cl_2.begin(), cl_2.end());
        return cl;
    }

    vector<string> getCommandDesc(){
        vector<string> cd(SkinForceCtrlCommand_desc, SkinForceCtrlCommand_desc+SkinForceCtrlCommandSize);
        vector<string> cd_2 = contrPlanner->getCommandDesc();
        cd.insert(cd.end(), cd_2.begin(), cd_2.end());
        cd_2 = contrThread->getCommandDesc();
        cd.insert(cd.end(), cd_2.begin(), cd_2.end());
        return cd;
    }

    bool close(){
		//stop threads
		if(contrThread){ 
#ifndef _SIMULATION
            contrThread->setOldFirmwarePidGains(); 
#endif
            contrThread->stop();
        }
        if(contrPlanner) contrPlanner->stop();
        if(contrThread){ delete contrThread; contrThread = 0; }
        if(contrPlanner){ delete contrPlanner; contrPlanner = 0; }
		
		//closing interfaces
        if (igaze){     
            igaze->lookAtFixationPoint(cat(-10.0, 0.0, 0.5));
            igaze->restoreContext(gazeCtrlContext);
        }
        if (dd_gaze){   dd_gaze->close();   delete dd_gaze;     dd_gaze=0;}
        /*if (igaze){     delete igaze; igaze=0; }*/
		
		//closing ports
        rpcPort.interrupt();
		rpcPort.close();

        printf("[PERFORMANCE INFORMATION]:\n");
        printf("Expected period %3.1f ms.\nReal period: %3.1f+/-%3.1f ms.\n", period, avgTime, stdDev);
        printf("Real duration of 'run' method: %3.1f+/-%3.1f ms.\n", avgTimeUsed, stdDevUsed);
        if(avgTimeUsed<0.5*period)
            printf("Next time you could set a lower period to improve the controller performance.\n");
        else if(avgTime>1.3*period)
            printf("The period you set was impossible to attain. Next time you could set a higher period.\n");

        return true;
    }

    double getPeriod()  { return gazePeriod*1e-3;  }

    bool updateModule()
	{
        if (contrThread==0 || contrPlanner==0){
            printf("ControlThread or ControlPlanner pointers are zero\n");
            return false;
        }

        contrThread->getEstPeriod(avgTime, stdDev);
        contrThread->getEstUsed(avgTimeUsed, stdDevUsed);     // real duration of run()
        if(avgTime > 1.3 * period){
            printf("[WARNING] Control loop is too slow. Real period: %3.3f+/-%3.3f. Expected period %3.3f.\n", avgTime, stdDev, period);
            printf("Duration of 'run' method: %3.3f+/-%3.3f.\n", avgTimeUsed, stdDevUsed);
        }

        if(gaze_enabled && igaze){
            Vector x = contrThread->getX().subVector(0,2);
            if(x(0)!=0.0 || x(1)!=0.0 || x(2)!=0.0){
                igaze->lookAtFixationPoint(x);
            }
        }

        Status thread_status = contrThread->getStatus() && contrPlanner->getThreadStatus();
		if (thread_status)
            return true;

		return false;     
	}   
    
};

}
} // end namespace

int main(int argc, char * argv[])
{
    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\t--context context: where to find the called resource (referred to $ICUB_ROOT/app)"                             << endl;
        cout << "\t--from       from: the name of the file.ini to be used for configuration"                                      << endl;
        cout << "\t--name       name: the name of the module used for the port names. default skinForceControl"	                  << endl;
        cout << "\t--robot      robot: the name of the robot. default icub"	                                					  << endl;
        cout << "\t--period     period: the period used by the module. default 5 ms"						                      << endl;
		cout << "\t--left_arm    control the left arm (default)" << endl;
		cout << "\t--right_arm   control the right arm" << endl;
        cout << "\t--gaze_on    activate the gaze control so that the robot looks at its hand"  <<endl;
        cout << "\t--float      start float control (gravity compensation) on sholder and elbow as soon as the module is launched"<< endl;
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    iCub::skinForceControl::controlModule module;
    module.runModule(rf);

#ifdef _DEBUG
    cin.get();
#endif

    return 0;
}

