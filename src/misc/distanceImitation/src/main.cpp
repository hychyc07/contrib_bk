/**
@ingroup icub_module
\defgroup distanceImitation distanceImitation
Perform reaching movements with the end-effector while guaranteeing a contact-consistent behavior in case of contact.
Copyright (C) 2008 RobotCub Consortium
Author: Andrea Del Prete
Date: first release 08/2012
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

#include "iCub/distanceImitation/distanceImitationConstants.h"
#include "iCub/distanceImitation/distanceImitationThread.h"

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

namespace distanceImitation
{

class distanceImitationModule: public RFModule
{
private:
    
    double period;
    TrialInfo trialInfo;
    distanceImitationThread *thread;
    Port rpcPort;

public:
    distanceImitationModule()
    {
		thread = 0;
    }
    

    bool configure(ResourceFinder &rf)
    {		
		string fwdSlash = "/";

        //-----------------GET THE MODULE NAME-------------------//
        string name = "distanceImitation";
        if (rf.check("name"))
            name = rf.find("name").asString().c_str();
        setName(name.c_str());
        
		//-----------------GET THE ROBOT NAME-------------------//
		string robot_name = "icub";
		if (rf.check("robot"))
            robot_name = rf.find("robot").asString().c_str();

        //filename: name of the text file to save data; date and time are added to the file name to avoid overwriting previous data
        trialInfo.filename = "data";
        if (rf.check("filename"))
            trialInfo.filename = rf.find("filename").asString().c_str();

        //T: time waited between trials (sec)
        trialInfo.T = DEFAULT_T;
        if (rf.check("T"))
            trialInfo.T = rf.find("T").asDouble();
        
        //k_max: upper bound of the interval in which k ranges
        trialInfo.k_max = DEFAULT_K_MAX;
        if (rf.check("k_max"))
            trialInfo.k_max = rf.find("k_max").asInt();

        //d_min: minimum distance between x1 and x2
        trialInfo.d_min = DEFAULT_D_MIN;
        if (rf.check("d_min"))
            trialInfo.d_min = rf.find("d_min").asDouble();

        //step: step used for generating different distances
        trialInfo.step = DEFAULT_STEP;
        if (rf.check("step"))
            trialInfo.step = rf.find("step").asDouble();

        //speed: reference speed of the movement
        trialInfo.speed = DEFAULT_SPEED;
        if (rf.check("speed"))
            trialInfo.speed = rf.find("speed").asDouble();

        //N: number of trial for each value of k
        trialInfo.N = DEFAULT_N;
        if (rf.check("N"))
            trialInfo.N = rf.find("N").asInt();

        //gaze_home: 3d point to gaze when not moving (root reference frame)
        trialInfo.gaze_home = DEFAULT_GAZE_HOME;
        if (rf.check("gaze_home"))
        {
            bottleToVector(*(rf.find("gaze_home").asList()), trialInfo.gaze_home);
        }

        //q_home: joint angles defining arm home configuration
        trialInfo.q_home = DEFAULT_Q_ARM_HOME;
        if (rf.check("q_home"))
            bottleToVector(*(rf.find("q_home").asList()), trialInfo.q_home);

        //x0: reference pose
        if (rf.check("x0"))
            bottleToVector(*(rf.find("x0").asList()), trialInfo.x0);


        //---------------------RPC PORT--------------------------//
        rpcPort.open(("/"+name+"/rpc").c_str());
        attach(rpcPort);

        //--------------------------THREAD--------------------------
        thread = new distanceImitationThread(name, robot_name, trialInfo, VERBOSE);
        fprintf(stderr,"distanceImitationThread istantiated...\n");
        if(!thread->start()){
            fprintf(stderr, "Error while initializing distanceImitationThread. Closing module.\n");
            return false;
        }
        fprintf(stderr,"distanceImitationThread started\n");
        return true;
    }

    bool respond(const Bottle& command, Bottle& reply) 
    {
	    unsigned int cmdId;
        Bottle param;
	    if(!identifyCommand(command, distanceImitationCommand_s, distanceImitationCommandSize, cmdId, param))
        {
            reply.addString("Unknown command. Input 'help' to get a list of the available commands.");
		    return true;
	    }

        Status s;
	    switch( cmdId ){
		    case quit:          reply.addString("quitting");    return false;
		    case help:
                {
                    reply.addVocab(Vocab::encode("many"));  // print every string added to the bottle on a new line
                    reply.addString((string(getName().c_str()) + " commands are: ").c_str());
			        for(unsigned int i=0; i<distanceImitationCommandSize; i++)
				        reply.addString( ("- "+distanceImitationCommand_s[i]+": "+distanceImitationCommand_desc[i]).c_str() );
			        return true;
                }
            case calib_start:
                s=thread->calibStart();
                break;
            case calib_done:
                s=thread->calibDone();
                break;
            case start:
                s=thread->startTrials();
                break;
            case start2:
                s=thread->startTrials2();
                break;
            case pause:
                s=thread->pauseTrials();
                break;
            case home:
                s=thread->pauseTrials();
                break;
            default: 
                reply.addString("ERROR: This command is known but it is not managed in the code."); 
                return true;
	    }
        
        if(s)
	        reply.addString( (distanceImitationCommand_s[cmdId]+" command received.").c_str());
        else
            reply.addString(s.toString());

	    return true;	
    }

    bool close(){
		//stop threads
		if(thread)
        { 
            thread->stop();
            delete thread; 
            thread = 0; 
        }
		
		//closing ports
        rpcPort.interrupt();
		rpcPort.close();

        return true;
    }

    double getPeriod()  { return period;  }

    bool updateModule()
	{
        if (thread==0){
            printf("distanceImitationThread pointer is zero\n");
            return false;
        }

        Status thread_status = thread->getStatus();
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
        cout << "\t--context    context: where to find the called resource (referred to $ICUB_ROOT/app)"                        << endl;
        cout << "\t--from       from: the name of the file.ini to be used for configuration"                                    << endl;
        cout << "\t--name       name: the name of the module used for the port names (default distanceImitation)"	            << endl;
        cout << "\t--robot      robot: the name of the robot. default icub"	                                					<< endl;
        cout << "\t--period     period: the period used by the module. default 5 ms"						                    << endl;
		cout << "\t--filename:  name of the text file to save data; date and time are added to the file name to avoid overwriting previous data"<< endl;
		cout << "\t--T:         time waited between trials (sec)"	                        << endl;
		cout << "\t--k_max:     upper bound of the interval in which k ranges"	            << endl;
		cout << "\t--d_min:     minimum distance between x1 and x2"	                        << endl;
		cout << "\t--step:      step used for generating different distances"	            << endl;
		cout << "\t--speed:     reference speed of the movement"	                        << endl;
		cout << "\t--N:         number of trial for each value of k"	                    << endl;
		cout << "\t--gaze_home: 3d point to gaze when not moving (root reference frame)"	<< endl;
		cout << "\t--q_home:    joint angles defining arm home configuration"	            << endl;
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
        return -1;

    iCub::distanceImitation::distanceImitationModule module;
    module.runModule(rf);

#ifdef _DEBUG
    cin.get();
#endif

    return 0;
}

