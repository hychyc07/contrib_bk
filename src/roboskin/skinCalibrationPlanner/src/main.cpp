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

#include "iCub/skinForceControl/util.h"
#include "iCub/skinCalibrationPlanner/calibrationPlannerConstants.h"
#include "iCub/skinCalibrationPlanner/calibrationPlannerThread.h"

YARP_DECLARE_DEVICES(icubmod)


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::dev;
using namespace iCub::ctrl;
using namespace iCub::iDyn;
using namespace iCub::skinForceControl;
using namespace std;

namespace iCub
{

namespace skinCalibrationPlanner
{

class controlModule: public RFModule
{
private:
    
    double avgTime, stdDev, avgTimeUsed, stdDevUsed; 
    double period;
    calibrationPlannerThread *planner;
    SkinPart skinPart;
    Port rpcPort;

public:
    controlModule()
    {
        skinPart            = SKIN_PART_UNKNOWN;
		planner             = 0;
        period              = 10;
    }
    

    bool configure(ResourceFinder &rf)
    {		
		string fwdSlash = "/";

        //-----------------GET THE MODULE NAME-------------------//
        string name = "skinCalibrationPlanner";
        if (rf.check("name"))
            name = rf.find("name").asString().c_str();
        setName(name.c_str());
        
        //-----------------GET THE PERIOD-------------------//
        period = DEFAULT_PERIOD;
        if (rf.check("period"))
            period = rf.find("period").asInt();
        
		//-----------------GET THE ROBOT NAME-------------------//
		string robot_name = "icub";
		if (rf.check("robot"))
            robot_name = rf.find("robot").asString().c_str();

        //-----------------GET WHOLE BODY DYNAMICS NAME-------------------//
		string wholeBodyName = "wholeBodyDynamics";
		if (rf.check("wholeBodyName"))
            wholeBodyName = rf.find("wholeBodyName").asString().c_str();

        //-----------------GET SKIN MANAGER NAME-------------------//
		string skinManagerName = "skinManager";
		if (rf.check("skinManagerName"))
            skinManagerName = rf.find("skinManagerName").asString().c_str();

        //------------------CHECK WHICH SKIN PART HAS TO BE CALIBRATED-----------//
		if (rf.check("left_forearm")){
			skinPart = SKIN_LEFT_FOREARM;
			fprintf(stderr,"'upper_arm' option found. Going to calibrate skin on upper arm.\n");
		}
        else if (rf.check("right_forearm")){
			skinPart = SKIN_RIGHT_FOREARM;
			fprintf(stderr,"'forearm' option found. Going to calibrate skin on forearm.\n");
		}
        else{
			skinPart = SKIN_RIGHT_FOREARM;
            fprintf(stderr, "No skin part specified. Going to calibrate skin on right forearm by default.\n");
        }

        //---------------------RPC PORT--------------------------//
        rpcPort.open(("/"+name+"/rpc").c_str());
        attach(rpcPort);

        //--------------------------THREAD--------------------------
        planner = new calibrationPlannerThread(name, (int)period, skinPart, VERBOSE);
        fprintf(stderr,"calibration planner thread istantiated...\n");
        if(!planner->start()){
            fprintf(stderr, "Error while initializing calibration planner thread. Closing module.\n");
            return false;
        }
        fprintf(stderr,"calibration planner thread started\n");
        return true;
    }

    bool respond(const Bottle& command, Bottle& reply) 
    {
	    unsigned int cmdId;
        Bottle param;
	    if(!identifyCommand(command, skinCalibrationPlannerCommand_s, skinCalibrationPlannerCommandSize, cmdId, param))
        {
            reply.addString("Unknown command. Input 'help' to get a list of the available commands.");
		    return true;
	    }

	    switch( cmdId ){
		    case quit:          reply.addString("quitting");    return false;
		    case help:
                {
                    reply.addVocab(Vocab::encode("many"));  // print every string added to the bottle on a new line
                    reply.addString((string(getName().c_str()) + " commands are: ").c_str());
			        for(unsigned int i=0; i<skinCalibrationPlannerCommandSize; i++)
				        reply.addString( ("- "+skinCalibrationPlannerCommand_s[i]+": "+skinCalibrationPlannerCommand_desc[i]).c_str() );
			        return true;
                }
            case start:
                planner->startCalibration();
                break;
            case stop:
                planner->stopCalibration();
                break;
            default: 
                reply.addString("ERROR: This command is known but it is not managed in the code."); 
                return true;
	    }

	    reply.addString( (skinCalibrationPlannerCommand_s[cmdId]+" command received.").c_str());
	    return true;	
    }

    bool close(){
		//stop threads
		if(planner)
        { 
            planner->stop();
            delete planner; 
            planner = 0; 
        }
		
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

    double getPeriod()  { return period;  }

    bool updateModule()
	{
        if (planner==0){
            printf("CalibrationPlannerThread pointer is zero\n");
            return false;
        }

        planner->getEstPeriod(avgTime, stdDev);
        planner->getEstUsed(avgTimeUsed, stdDevUsed);     // real duration of run()
        if(avgTime > 1.3 * period){
            printf("[WARNING] Control loop is too slow. Real period: %3.3f+/-%3.3f. Expected period %3.3f.\n", avgTime, stdDev, period);
            printf("Duration of 'run' method: %3.3f+/-%3.3f.\n", avgTimeUsed, stdDevUsed);
        }

        Status thread_status = planner->getStatus();
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
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    iCub::skinCalibrationPlanner::controlModule module;
    module.runModule(rf);

#ifdef _DEBUG
    cin.get();
#endif

    return 0;
}

