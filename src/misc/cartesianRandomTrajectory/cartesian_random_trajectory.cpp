// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// Author: Silvio Traversaro ( silvio dot traversaro at iit dot it ) 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Random.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IImpedanceControl.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <iCub/ctrl/math.h>


#include <gsl/gsl_math.h>

#include <stdio.h>

#include <string>

using namespace iCub::ctrl;

#define PRINT_STATUS_PER    1.0     // [s]
#define MAX_TORSO_PITCH     30.0    // [deg]

std::string robotName;
bool gaze_enabled;

double ctrl_thread_period = 1.5;
double trajectory_time = 1.2;


YARP_DECLARE_DEVICES(icubmod)

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


class CtrlThread: public RateThread,
                  public CartesianEvent
{
protected:
	PolyDriver 		robotDevice;
    PolyDriver         client;
    PolyDriver         client_gaze;
    IPositionControl *ipos;
    ICartesianControl *icart;
    IImpedanceControl *iimp;
    IPidControl * ipid;
    ITorqueControl *ipid_trq;
    IControlMode *ictrl;
    IEncoders * iencs;
    IGazeControl *igaze;
    
    
    Pid old_pids[30];
    
    Vector starting_enc;
        
    Vector starting_pose;
    Vector starting_orient;
    
    Vector starting_fixation_point;
    
    bool got_starting_enc;
    
    Vector xd;
    Vector od;

    int startup_context_id;
    int startup_context_id_gaze;

    double t;
    double t0;
    double t1;
    
    // the event callback attached to the "motion-ongoing"
    virtual void cartesianEventCallback()
    {
        fprintf(stdout,"20%% of trajectory attained\n");
    }

public:
    CtrlThread(const double period) : RateThread(int(period*1000.0))
    {
        // we wanna raise an event each time the arm is at 20%
        // of the trajectory (or 70% far from the target)
        cartesianEventParameters.type="motion-ongoing";
        cartesianEventParameters.motionOngoingCheckPoint=0.2;
        
    }
    
    void closeRightHand(IPositionControl * ipos) {
        /*
        BLACK ICUB
        ipos->positionMove(7,7);
		ipos->positionMove(8,20);
		ipos->positionMove(9,30);
		ipos->positionMove(10,80);
		ipos->positionMove(11,61);
		ipos->positionMove(12,120);
		ipos->positionMove(13,83);
		ipos->positionMove(14,117);
		ipos->positionMove(15,200);
        */
        //RED ICUB
        ipos->positionMove(7,7);
		ipos->positionMove(8,20);
		ipos->positionMove(9,30);
		ipos->positionMove(10,80);
		ipos->positionMove(11,61);
		ipos->positionMove(12,120);
		ipos->positionMove(13,83);
		ipos->positionMove(14,160);
		ipos->positionMove(15,230);
        return;
    }

    virtual bool threadInit()
    {
        
        starting_pose = Vector(3);
        starting_orient = Vector(4);
        
        starting_fixation_point = Vector(3);
        
        got_starting_enc = false;
        
		std::string remotePorts="/";
		remotePorts+=robotName;
		remotePorts+="/right_arm";

		std::string localPorts="/test/client";

		Property options;
		options.put("device", "remote_controlboard");
		options.put("local", localPorts.c_str());   //local port names
		options.put("remote", remotePorts.c_str());         //where we connect to
		
		robotDevice.open(options);
		
		if (!robotDevice.isValid()) {
			printf("Device not available.  Here are the known devices:\n");
			printf("%s", Drivers::factory().toString().c_str());
			return 0;
		}
		
		bool ok;
		ok = robotDevice.view(ictrl);
		ok = ok && robotDevice.view(iimp);
		ok = ok && robotDevice.view(ipid);
		ok = ok && robotDevice.view(ipos);
		ok = ok && robotDevice.view(iencs);
		ok = ok && robotDevice.view(ipid_trq);

		if (!ok) {
			printf("Problems acquiring interfaces\n");
			return 0;
		}
		
		int nj = 0;
		
		iimp->getAxes(&nj);
	
		starting_enc.resize(nj);
		starting_enc.zero();
		if(iencs==0) {
			printf("Problems with encoder interface\n");
			return 0;
		}
		
	
		
		//Set torque pid
		Pid my_pid;
		for(int j = 0; j < 5; j++ ) {
			double buf;
			ipid_trq->getTorquePid(j,&(old_pids[j]));
			my_pid = old_pids[j];
			my_pid.max_output = 100;
			ipid_trq->setTorquePid(j,my_pid);
			//printf("Starting position for encoder %d is  %lf\n",j,start_enc[j]);
		}
		
		for (int i = 0; i < 5; i++) {
			ictrl->setImpedancePositionMode(i);
			iimp->setImpedance(i, 0.111, 0.014);
		}
		
		
        // open a client interface to connect to the cartesian server of the simulator
        // we suppose that:
        //
        // 1 - the iCub simulator is running
        //     (launch iCub_SIM)
        //
        // 2 - the cartesian server is running
        //     (launch simCartesianControl)
        //     
        // 3 - the cartesian solver for the left arm is running too
        //     (launch iKinCartesianSolver --context simCartesianControl/conf --part right_arm)
        // iKinCartesianSolver --context simCartesianControl/conf --part right_arm
        Property option("(device cartesiancontrollerclient)");
        std::string cart_contr_port = "/"+robotName+"/cartesianController/right_arm";
        option.put("remote",cart_contr_port.c_str());
        option.put("local","/cartesian_client/right_arm");

        if (!client.open(option))
            return false;

        // open the view
        client.view(icart);

        // latch the controller context in order to preserve
        // it after closing the module
        // the context contains the dofs status, the tracking mode,
        // the resting positions, the limits and so on.
        icart->storeContext(&startup_context_id);

        // set trajectory time
        icart->setTrajTime(trajectory_time);

        // get the torso dofs
        Vector newDof, curDof;
        icart->getDOF(curDof);
        newDof=curDof;

        // enable the torso yaw and pitch
        // disable the torso roll
        newDof[0]=0;
        newDof[1]=0;
        newDof[2]=0;

        // impose some restriction on the torso pitch
        limitTorsoPitch();

        // send the request for dofs reconfiguration
        icart->setDOF(newDof,curDof);
       


        // print out some info about the controller
        Bottle info;
        icart->getInfo(info);
        fprintf(stdout,"info = %s\n",info.toString().c_str());

        // register the event, attaching the callback
        icart->registerEvent(*this);
        
        // get initial pose and orientation
        icart->getPose(starting_pose,starting_orient);
        
        bool stat=true;
		stat = iencs->getEncoders(starting_enc.data());
		if(!stat) {
			printf("problem acquiring encoders\n");
            printf("Number of axes: %d\n",nj);
            //return 0;
        } else {
            got_starting_enc = true;
        }
        
        closeRightHand(ipos);

        
        //setting gaze interface
        //For simulation 
        //iKinGazeCtrl --from configSim.ini 
        if( gaze_enabled ) {
            Property option_gaze;
            option_gaze.put("device","gazecontrollerclient");
            option_gaze.put("remote","/iKinGazeCtrl");
            option_gaze.put("local","/client/gaze");
 
            if( !client_gaze.open(option_gaze) ) {
                return false;
            }

            if (client_gaze.isValid()) {
                client_gaze.view(igaze);
            }
        
            igaze->storeContext(&startup_context_id_gaze);

            // set trajectory time:
            igaze->setNeckTrajTime(0.8);
            igaze->setEyesTrajTime(0.4);

            igaze->setTrackingMode(true);
 
            igaze->getFixationPoint(starting_fixation_point);
        }

        xd.resize(3);
        od.resize(4);

		yarp::os::Random::seed(Time::now());
		
		//Rest position and weighting
		/*
		Vector curRestPos;
		icart->getRestPos(curRestPos);
		printf("[%s]\n",curRestPos.toString().c_str());  // [0 0 0 0 0 0 0 0 0 0] will be printed out

		Vector curRestWeights;
		icart->getRestWeights(curRestWeights);
		printf("[%s]\n",curRestWeigths.toString().c_str())  // [1 1 1 0 0 0 0 0 0 0] will be printed out
		*/

        return true;
    }
    
    /**
     * If a fixation point brings the head near to the limits, returns 
     * a more "relaxed" fixation point
     */
    Vector fixationFilter(Vector xd) {
        Vector ret = xd;
        if( ret[2] < 0.2 ) ret[2] = 0.2;
        return ret;
    }
    

    virtual void afterStart(bool s)
    {
        if (s)
            fprintf(stdout,"Thread started successfully\n");
        else
            fprintf(stdout,"Thread did not start\n");

        t=t0=t1=Time::now();
    }

    virtual void run()
    {
        t=Time::now();

        generateTarget();

        // go to the target
        // (in streaming)
        printf("Streaming new position %s\n",xd.toString().c_str());
  
        icart->goToPosition(xd);
        
        if( gaze_enabled ) {
            igaze->lookAtFixationPoint(fixationFilter(xd));
        }
        
        // some verbosity
        printStatus();
    }

    virtual void threadRelease()
    {    
        //Return to initial pose
        printf("Returning to initial position");
        icart->goToPose(starting_pose,starting_orient);
        
        if( gaze_enabled ) {
            igaze->lookAtFixationPoint(starting_fixation_point);
        }
        
        yarp::os::Time::delay(5);

        
        
		int nj= 0;
		
		iimp->getAxes(&nj);

        //restore old torque PIDS
		for(int j = 0; j < 5; j++ ) {
			ipid_trq->setTorquePid(j,old_pids[j]);
		}
        // we require an immediate stop
        // before closing the client for safety reason
        icart->stopControl();

        // it's a good rule to restore the controller
        // context as it was before opening the module
        icart->restoreContext(startup_context_id);
        
        
        if( got_starting_enc ) {
            for (int i = 0; i < nj; i++) {
                ictrl->setPositionMode(i);
                ipos->positionMove(i,starting_enc[i]);
                printf("Restoring position for encoder %d to  %lf\n",i,starting_enc[i]);
            }
        }
		
		bool motionDone = true;
		while( !motionDone ) {
			yarp::os::Time::delay(1);
			ipos->checkMotionDone(&motionDone);
			
		}

        client.close();
        
        //Closing gaze interface
        if( gaze_enabled ) {
            igaze->stopControl();
 
            // it's a good rule to restore the controller
            // context as it was before opening the module
            igaze->restoreContext(startup_context_id_gaze);
 
            client_gaze.close();
        }
        
        
    }

    void generateTarget()
    {   
		static long int called = 0;

		double max_x, min_x, max_y, min_y, max_z, min_z;
		max_x = -0.20;
        min_x = -0.34;
        max_y = 0.35;
        min_y = 0.20;
        max_z = 0.40;
        min_z = 0.20;
        
		xd[0]=(max_x - min_x)*(yarp::os::Random::uniform()) + min_x;

        xd[1]=(max_y - min_y)*yarp::os::Random::uniform() + min_y;

        xd[2]=(max_z - min_z)*yarp::os::Random::uniform() + min_z;  
                 
        called++;
	}
        
    double norm(const Vector &v)
    {
        return sqrt(dot(v,v));
    }

    void limitTorsoPitch()
    {
        int axis=0; // pitch joint
        double min, max;

        // sometimes it may be helpful to reduce
        // the range of variability of the joints;
        // for example here we don't want the torso
        // to lean out more than 30 degrees forward

        // we keep the lower limit
        icart->getLimits(axis,&min,&max);
        icart->setLimits(axis,min,MAX_TORSO_PITCH);
    }

    void printStatus()
    {        
        if (t-t1>=PRINT_STATUS_PER)
        {
            Vector x,o,xdhat,odhat,qdhat;

            // we get the current arm pose in the
            // operational space
            icart->getPose(x,o);

            // we get the final destination of the arm
            // as found by the solver: it differs a bit
            // from the desired pose according to the tolerances
            icart->getDesired(xdhat,odhat,qdhat);

            double e_x=norm(xdhat-x);

            fprintf(stdout,"+++++++++\n");
            fprintf(stdout,"xd          [m] = %s\n",xd.toString().c_str());
            fprintf(stdout,"xdhat       [m] = %s\n",xdhat.toString().c_str());
            fprintf(stdout,"x           [m] = %s\n",x.toString().c_str());
            fprintf(stdout,"norm(e_x)   [m] = %g\n",e_x);
            fprintf(stdout,"---------\n\n");

            t1=t;
        }
    }
};



class CtrlModule: public RFModule
{
protected:
    CtrlThread *thr;

public:
    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        thr=new CtrlThread(ctrl_thread_period);
        if (!thr->start())
        {
            delete thr;
            return false;
        }

        return true;
    }

    virtual bool close()
    {
        thr->stop();
        delete thr;

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};



int main(int argc, char ** argv)
{   
    // we need to initialize the drivers list 
    YARP_REGISTER_DEVICES(icubmod)
    
    Property params;
    params.fromCommand(argc, argv);
    
    if( params.check("help") ) {
        fprintf(stdout,"Options:\n");
        fprintf(stdout,"\t--robot robot: the robot name\n");
        fprintf(stdout,"\t--trajectory_time time: the time used for doing a cartesian trajectory (default: 1.2)\n");
        fprintf(stdout,"\t--ctrl_thread_period period: the period used for the control thread (default: 1.5)\n");
        fprintf(stdout,"\t--gaze_enabled : enables the gaze following the hand movements (default: off)\n");
        return 0;
    }

    if (!params.check("robot"))
    {
        fprintf(stderr, "Please specify the name of the robot\n");
        fprintf(stderr, "--robot name (e.g. icub)\n");
        return -1;
    }
    
    robotName=params.find("robot").asString().c_str();

    gaze_enabled = params.check("gaze_enabled");
    
    if( params.check("trajectory_time") ) {
        fprintf(stdout,"Using trajectory_time from configuration: ");
        trajectory_time = params.find("trajectory_time").asDouble();
        fprintf(stdout,"%lf\n",trajectory_time);

    }
    
    if( params.check("ctrl_thread_period") ) {
        fprintf(stdout,"Using ctrl_thread_period from configuration: ");
        ctrl_thread_period = params.find("ctrl_thread_period").asDouble();
        fprintf(stdout,"%lf\n",ctrl_thread_period);
    }
    
    
    Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf(stdout,"Error: yarp server does not seem available\n");
        return -1;
    }

    CtrlModule mod;

    ResourceFinder rf;
    return mod.runModule(rf);
}



