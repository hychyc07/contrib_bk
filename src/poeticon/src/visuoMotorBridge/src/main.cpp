/** 
@ingroup poeticon_module
\defgroup poeticon

A module based upon \ref ActionPrimitives library available from
the iCub repository that executes some basic actions on 
identified objects usually placed on a table in front of the 
robot. 

\section intro_sec Description 
This module makes use of \ref ActionPrimitives library in order
to execute a sequence of simple actions: for example it can 
reach for an object, try to grasp it, then lift it and finally
release it. 
 
The commands sent as bottles to the module port /<modName>/cmd:i
are the following: 
 
(notation: [.] identifies a vocab, <.> specifies a double) 
 
<b>TOUCH</b> 
format: [touch] <x> <y> <z> 
action: a touch is executed on the object placed at (x,y,z) 
cartesian coordinates. These coordinates are previsously 
modified according to the offsets found during kinematic 
calibration. 
 
<b>GRASP</b> 
format: [grasp] <x> <y> <z> 
action: a grasp is executed on the object placed at (x,y,z) 
cartesian coordinates; the object is then lifted and released 
afterwards only in case the grasp was successful. The (x,y,z) 
coordinates are previsously modified according to the offsets 
found during kinematic calibration. 
 
<b>TAP</b> 
format: [tap] <x> <y> <z> 
action: a tap is executed on the object placed at (x,y,z) 
cartesian coordinates. These coordinates are previsously 
modified according to the offsets found during kinematic 
calibration. 
 
<b>CALIB_TABLE</b> 
format: [calib] [table] 
action: the robot will try to find out the table height 
exploiting the contact detection based on force control. If the 
contact is detected then the new table height is sent to the 
eye2world module that is in charge of extracting the 3d 
coordinates of the object from its 2d projection on the image 
plane. The file containing the table information is also updated 
accordingly to avoid calibrating always at start-up. 
 
<b>CALIB_KINEMATIC</b> 
This command is splitted in two consecutive sub-commands: 
 
format subcmd1: [calib] [kin] [start] [left]/[right] <x> <y> <z> 
action: the robot reaches the position (x,y,z) with the 
specified arm and waits for the interaction with human based on 
force control in order to store the corresponding kinematic 
offset; in other words, through this command the user can build 
on-line a map between the 3d input (x,y,z) and the resulting 2d 
quantity (dx,dy) that is required to compensate for the unknown
kinematic offsets that affect the reaching on the table. In this
phase, the user can move the robot arm just by exerting a push 
on it: the quantities acquired during this exploration will be 
used by the tap, grasp, etc. (Note that the compensation is 
achieved only on the position not on the orientation). 
 
format subcmd2: [calib] [kin] [stop] 
action: terminate the calibration phase and update the map. 

\section lib_sec Libraries 
- YARP libraries. 
- \ref ActionPrimitives library. 
 
\section portsa_sec Ports Accessed
Assumes that \ref icub_iCubInterface (with ICartesianControl 
interface implemented) is running. 
 
\section portsc_sec Ports Created 
Aside from the internal ports created by \ref ActionPrimitives 
library, we also have: 
 
- \e /<modName>/cmd:i receives a bottle containing commands 
  whose formats are specified in the previous section.
 
- \e /<modName>/rpc remote procedure call. 
    Recognized remote commands:
    -[status]: returns 1 iff an action is still under way.
    -[track] [on]/[off]: enable/disable the tracking mode of the
     cartesian interface (use with care).
 
\section parameters_sec Parameters 
The following are the options that are not usually contained 
within the configuration file. 
 
--name \e name
- specify the module name, which is \e icubDemo2010MotorIF by 
  default.
 
--part \e type 
- specify which arm has to be used: type can be \e left_arm, \e 
  right_arm, \e both_arms (default=both_arms).
 
--hand_sequences_file \e file 
- specify the path to the file containing the hand motion 
  sequences relative to the current context ( \ref
  ActionPrimitives ).
 
--table_file \e file 
- specify the path to the file containing information about 
  table position that can be updated on-line and that are read
  by the eye2world module. (Relative to the current context)
 
--homography_port \e port 
- specify the port name of the module eye2world which is in 
  charge of doing the homography (transformation from 2d to 3d
  coordinates on the table).
 
--opdbServerName \e port 
- specify the port name of the OPDB server from which some 
  information can be retrieved at start-up depending on the
  value of a switch in the configuration file. The Object
  Properties DataBase (OPDB) contains indeed the posture for
  closing the hand during a grasp action that can be used to
  initialize the grasp primitive by replacing the corresponding
  posture present within the hand sequence file. The OPDB needs
  to be started prior to launching this module.
 
--from \e file 
- specify the configuration file (use \e --context option to 
  select the current context).
 
\section conf_file_sec Configuration Files 
Hereafter the parameters given through the configuration file 
are listed: 
 
\code
[general]
robot                           icub	// the robot name to connect to (it can be icubSim too)
thread_period                   50		// the period given in [ms]
default_exec_time               3.0		// the point-to-point time in [s] for the task space movements
reach_tol                       0.01	// the tolerance to determine the end of reaching task
torso_pitch                     on		// enable/disable the torso pitch
torso_roll                      off		// enable/disable the torso roll
torso_yaw                       on		// enable/disable the torso yaw
torso_pitch_min                 0.0		// minimum value for the pitch of torso [deg]
torso_pitch_max                 40.0	// maximum value for the pitch of torso [deg]; similar switch exist for roll and yaw
tracking_mode                   off		// enable/disable the tracking mode of cartesian interface
verbosity                       on		// enable/disable the verbosity level
use_opdb                        off		// enable/disable the initialization of grasp posture from OPDB
calib_kin_gain                  0.005	// the gain ([m/N]) that multiplies the sensed force while calibrating the kinematic offsets
target_in_range_thres           0.6		// a threshold identifying the reachable area given in [m]; beyond it the robot will execute a pointing action 

[left_arm]
kinematic_offsets_file          kinematic_offsets_left.ini	// the name of the file containing the kinematic offsets
grasp_displacement              (0.0 0.0 0.17)				// the (x,y,z) displacement relative to the object position to be reached before executing the grasp
grasp_relief                    (0.0 0.0 0.045)				// while grasping the relief specifies the (x,y,z) point relative to object position to be reached after having detected the contact, just before closing the fingers
lifting_displacement            (0.0 0.0 0.2)				// the amount of lifting after a successful grasp
touching_displacement           (0.0 0.0 0.17)				// the same as grasp_displacement but for the touch action
tapping_displacement            (0.0 -0.1 0.05)             // the same as grasp_displacement but for the tap action 
home_position                   (-0.25 -0.25 0.04)			// specify the (x,y,z) home position for the palm
dropping_length                 0.1							// the robot will drop the grasped object in a range determined by this parameter (given in [m])
force_calib_table_thres         3.0							// the threshold for contact detection given in [N] to which it is compared the norm of external forces acting on the end-effector while touching the table
force_calib_kin_thres           2.25						// the threshold given in [N] while calibrating the kinematics: forces at the end-effector lower in norm than this value are not considered

[right_arm]
...
\endcode

\section in_files_sec Input/Output Data Files
-- table.ini contains the table height and is updated on-line as 
   result of an exploration phase
-- kinematic_offsets.ini are arm-dependent files that contain 
   the maps of kinematic offsets for compensating the reaching
   on the table: they are read at start-up and they can be
   updated on-line as result of calibration phase.
 
\section tested_os_sec Tested OS
Windows, Linux

\author Carlo Ciliberto, Vadim Tikhanoff
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <string>

#include "MotorThread.h"
#include "VisuoThread.h"
#include "utils.h"



#define CMD_IDLE                    VOCAB4('i','d','l','e')
#define CMD_HOME                    VOCAB4('h','o','m','e')
#define CMD_HOME_NO_HANDS           VOCAB4('b','a','c','k')
#define CMD_CALIB_TABLE             VOCAB4('c','a','t','a')
#define CMD_DROP                    VOCAB4('d','r','o','p')
#define CMD_PUT_AWAY                VOCAB4('a','w','a','y')
#define CMD_HOLD                    VOCAB4('h','o','l','d')
#define CMD_RECOG_MSR               VOCAB4('r','e','c','o')

#define CMD_LEARN_MIL               VOCAB4('l','e','a','r')
#define CMD_LEARN_MSR               VOCAB4('e','x','p','l')

#define CMD_TAKE                    VOCAB4('t','a','k','e')
#define CMD_TOUCH                   VOCAB4('t','o','u','c')
#define CMD_GRASP                   VOCAB4('g','r','a','s')
#define CMD_PICK                    VOCAB4('p','i','c','k')
#define CMD_PUSH                    VOCAB4('p','u','s','h')
#define CMD_POINT                   VOCAB4('p','o','i','n')
#define CMD_TRACK                   VOCAB4('t','r','a','c')
#define CMD_REACH                   VOCAB4('r','e','a','c')
#define CMD_FILL                    VOCAB4('f','i','l','l')



#define CMD_LEARN_ACTION            VOCAB4('t','e','a','c')
#define CMD_LEARN_ACTION_STOP       VOCAB4('s','t','o','p')
#define CMD_IMITATE_ACTION          VOCAB4('i','m','i','t')


#define PARAM_FIXATION              VOCAB4('f','i','x','a')
#define PARAM_MOTION                VOCAB4('m','o','t','i')
#define PARAM_TRACK                 VOCAB4('t','r','a','c')
#define PARAM_TABLE                 VOCAB4('t','a','b','l')


#define CMD_TORSO_GRASP             VOCAB4('t','o','g','a')

#define TEST                        VOCAB4('t','e','s','t')
#define TEST2                       VOCAB4('t','e','s','a')
#define CMD_FLIP                    VOCAB4('f','l','i','p')

#define CMD_WAVE                    VOCAB4('w','a','v','e')

#define CMD_BEAR_MIL                VOCAB4('b','e','a','r')

#define CMD_YANNIS                  VOCAB4('y','a','n','n')

#define CMD_EXTRACT                 VOCAB4('e','x','t','r')


#ifdef WIN32
    #pragma warning(disable:4996)
#endif

YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace iCub::action;

const string GET_PERCENTILE_COMMAND[] = {"get", "percentile"};
const unsigned int GET_PERCENTILE_COMMAND_LENGTH = 2;	// size of GET_PERCENTILE_COMMAND array

class VisuoMotorModule: public RFModule
{
protected:
    MotorThread                 *motorThr;
    VisuoThread                 *visuoThr;

    Resource                    visuoMotorResource;

    Port                        rpcPort;
    Port                        skinDriftCompRpcPortLeft;
    Port                        skinDriftCompRpcPortRight;

    Port                        wavePort;

    double                      motionDetectionWaitThresh;
    double                      objectDetectionWaitThresh;

    double                      explorationMSRWaitThresh;
    bool                        percentileHasBeenSet;

public:
    VisuoMotorModule()
    {}

    virtual bool configure(ResourceFinder &rf)
    {
        Bottle bManager=rf.findGroup("manager");

        string name=rf.find("name").asString().c_str();
        setName(name.c_str());

        wavePort.open(("/"+name+"/wave:o").c_str());


        motionDetectionWaitThresh=bManager.check("motion_detection_wait_thresh",Value(3.0)).asDouble();
        objectDetectionWaitThresh=bManager.check("object_detection_wait_thresh",Value(3.0)).asDouble();
        explorationMSRWaitThresh=bManager.check("exploration_MSR_wait_thresh",Value(300.0)).asDouble();
        rpcPort.open(("/"+name+"/rpc").c_str());
        attach(rpcPort);
        
        Network::connect(("/"+name+"/seg:o").c_str(),"/objSegMod/fixPoint:i");
        
        //open rpc port to connect to skinDriftCompensator left
        string skinDriftCompRpcPortNameLeft = "/";//+ name + "/";
        skinDriftCompRpcPortNameLeft += getName(rf.check("skinDriftCompRpcPortLeft", Value("/calibrationRpcPortLeft:o")).asString());

        //open rpc port to connect to skinDriftCompensator right
        string skinDriftCompRpcPortNameRight = "/"; //+ name + "/";
        skinDriftCompRpcPortNameRight += getName(rf.check("skinDriftCompRpcPortRight", Value("/calibrationRpcPortRight:o")).asString());
        
        if (! skinDriftCompRpcPortLeft.open(skinDriftCompRpcPortNameLeft.c_str())) 
        {
            fprintf(stdout, "unable to open the compensated left tactile input port \n");
            return false; 
        }

        if (! skinDriftCompRpcPortRight.open(skinDriftCompRpcPortNameRight.c_str())) 
        {
            fprintf(stdout, "unable to open the compensated right tactile input port \n");
            return false; 
        }

        fprintf(stdout, "finding the port name %s\n", skinDriftCompRpcPortRight.getName().c_str());

        Network::connect(skinDriftCompRpcPortNameLeft.c_str(), "/leftHandSkinDriftComp/rpc");
        Network::connect(skinDriftCompRpcPortNameRight.c_str(), "/rightHandSkinDriftComp/rpc");

        // message to send to the skinDrifCompensation module in order to get the percentile
        percentileHasBeenSet = false;

        motorThr=new MotorThread(rf,visuoMotorResource);
        visuoThr=new VisuoThread(rf,visuoMotorResource);
        if(!motorThr->start() || !visuoThr->start())
        {
         
            close();
            return false;
        }
                
        motorThr->release(RIGHT);
        motorThr->release(LEFT);
        motorThr->goHome(false);

        return true;
    }

    virtual bool close()
    {
        if(motorThr!=NULL)
        {
            motorThr->stop();
            delete motorThr;
        }

        if(visuoThr!=NULL)
        {
            visuoThr->stop();
            delete visuoThr;
        }

        wavePort.close();

        rpcPort.close();
        skinDriftCompRpcPortLeft.close();
        skinDriftCompRpcPortRight.close();

        return true;
    }

    virtual double getPeriod()
    {
        return 0.1;
    }

    // we don't need a thread since the actions library already
    // incapsulates one inside dealing with all the tight time constraints
    virtual bool updateModule()
    {
        //for now I don't have any clue on what to do here
        return true;
    }

    bool interruptModule()
    {
        rpcPort.interrupt();
        skinDriftCompRpcPortLeft.interrupt();
        skinDriftCompRpcPortRight.interrupt();
       
        motorThr->interrupt();

        return true;
    }

    virtual bool respond(const Bottle &command, Bottle &reply)
    {

        if(command.size()!=0 && command.get(0).asString()=="wave")
        {
            Vector stereo=visuoThr->getTrack();
            if(stereo.size()==4)
            {
                Bottle b;
                b.clear();
                b.addString("fix");
                b.addInt(stereo[0]);
                b.addInt(stereo[1]);

                int size=command.size()>1?command.get(1).asInt():100;
                b.addInt(size);
                b.addInt(size);

                wavePort.write(b);
            }

            reply.addString("waved");
            return true;
        }

        if(command.size()>1 && command.get(0).asString()=="impedance")
        {
            bool turn_on=command.get(1).asString()=="on";
            motorThr->setImpedance(turn_on);
            reply.addString("impedance updated");
            return true;        
        }

        if(command.size()==0)
        {
            reply.addString("No command received.");
            return true;
        }
        else if(command.size()==1) // all commands that do not need parameters
        {

            // the first time a command is received, update the percentile
	        if(!percentileHasBeenSet){		
                fprintf(stdout,"setting percentile\n");
		        updatePercentileLeft();
                updatePercentileRight();
                percentileHasBeenSet = true;
	        }
            switch(command.get(0).asVocab())
            {
                case CMD_FLIP:
                {
                    Bottle b;
                    b.clear();
                    b.addString("again");

                    wavePort.write(b);
                    reply.addString("flipped");
                    return true;
                }

                case CMD_HOLD:
                {
                    if(motorThr->isHolding())
                        reply.addString("holding");
                    else
                       reply.addString("failed");

                    return true;
                }
                
                case CMD_EXTRACT:
                {
                    motorThr->extract();
                    reply.addString("extracted");
                    return true;
                }

                case CMD_IDLE:
                {
                    motorThr->setGazeIdle();
                    reply.addString("idle");
                    return true;
                }

                case CMD_HOME:
                {
                    motorThr->release();
                    motorThr->goHome();
                    reply.addString("home");
                    return true;
                }

                case CMD_HOME_NO_HANDS:
                {
                    motorThr->goHome();
                    reply.addString("home");
                    return true;
                }

                case CMD_CALIB_TABLE:
                {
                    if(motorThr->calibTable())
                        reply.addString("table height found\n");
                    else
                        reply.addString("table height not found\n");
                    return true;
                }


                case CMD_LEARN_ACTION_STOP:
                {
                    motorThr->suspendDragMode();
                    motorThr->lookAtHand(false);
                    reply.addString("stopped drag");
                    return true;
                }

                case CMD_DROP:
                {
                    if(!motorThr->isHolding())
                    {
                        reply.addString("Nothing to drop. Not holding anything");
                        motorThr->release();
                        motorThr->goHome();
                        return true;
                    }

                    motorThr->trackObject(false);
                    motorThr->deploy();
                    motorThr->keepFixation(true);
                    motorThr->setGazeIdle();
                    motorThr->goHome();
                    motorThr->keepFixation(false);


                    Vector s=visuoThr->getTrack();
                    if(s.size()==4)
                    {
                        Bottle b;
                        b.clear();
                        b.addString("fix");
                        b.addInt(s[0]);
                        b.addInt(s[1]);
                        b.addInt(100);
                        b.addInt(100);
                        wavePort.write(b);
                    }

                    reply.addString("dropped");
                    return true;

                }

                case CMD_PUT_AWAY:
                {
                    if(!motorThr->isHolding())
                    {
                        reply.addString("Nothing to drop. Not holding anything");
                        motorThr->release();
                        motorThr->goHome();
                        return true;
                    }

                    motorThr->trackObject(false);
                    motorThr->putAway(true);
                    motorThr->setGazeIdle();
                    motorThr->goHome();


                    reply.addString("dropped");
                    return true;

                }

                case CMD_TORSO_GRASP:
                {
                    motorThr->torsoPrepareForGrasp();

                    reply.addString("torsoed");
                    return true;
                }

                case VOCAB4('s','h','o','w'):
                {
                    visuoThr->doShow();
                    reply.addString("ack");
                    return true;
                }
            }
        }
        else
        {
            // the first time a command is received, update the percentile
	        if(!percentileHasBeenSet){	
                fprintf(stdout,"setting percentile\n");	
		        updatePercentileLeft();
                updatePercentileRight();
                percentileHasBeenSet = true;
	        }
            // learning commands
        
            switch(command.get(0).asVocab())
            {
                case CMD_RECOG_MSR:
                {

                    string orig_name=command.get(1).asString().c_str();
                    fprintf(stdout,"$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ orig obj name is  %s.\n",orig_name.c_str());

                    if(!motorThr->isHolding() )
                    {
                        fprintf(stdout,"Nothing to recognize. Not holding anything");
                        reply.addString("Nothing to recognize. Not holding anything");
                        motorThr->release();
                        motorThr->goHome();
                        return true;
                    }

                    
                    
                    if (orig_name=="")
                    {
                        reply.addString("please set the true object name for data comparison");
                        return true;
                    }            
                    
                    //if it is not currently tracking anything, start learning what it has in fixation
                    if(!visuoThr->isTracking())
                        visuoThr->getFixation();

                    motorThr->setGazeIdle();
                    int arm=motorThr->getArmInUse();
                    visuoThr->resumeLearningMSR();
                    if(visuoThr->startRecogMSR(orig_name, arm))
                    {
                        string obj_name="";
                        
                        double t=Time::now();
                        while(obj_name=="" && !isStopping())// && (Time::now()-t<explorationMSRWaitThresh))
                            visuoThr->recogMSR(obj_name);

                        if(obj_name=="")
                        {
                            visuoThr->stopMSR();
                            reply.addString("unable");
                            reply.addString("to recognize this object");
                        }
                        else
                        {
                            reply.addString(obj_name.c_str());
                            fprintf(stdout,"Recognized %s.\n",obj_name.c_str());
                        }
                    }
                    else
                    {
                        reply.addString("Error starting the MSR recognition procedure!");
                        fprintf(stdout,"Error starting the MSR recognition procedure!\n");
                    }

                    visuoThr->suspendLearningMSR();
                    motorThr->drawNear();
                    return true;
                }

                case CMD_LEARN_MIL:
                {
                    string obj_name=command.get(1).asString().c_str();
                    if(motorThr->isHolding()) // 
                    {
                        fprintf(stdout,"Deploying %s.\n",obj_name.c_str());
                        motorThr->deploy();
                    }

                    //if it is not currently tracking anything, start learning what it has in fixation
                    if(!visuoThr->isTracking())
                        visuoThr->getFixation();

                    visuoThr->startLearningMIL(obj_name.c_str());

                    fprintf(stdout,"Looking at %s.\n",obj_name.c_str());
                    motorThr->exploreTorso(10.0);

                    visuoThr->trainMIL();

                    reply.addString((obj_name + " learned").c_str());
                    fprintf(stdout,"'%s' learned.\n",obj_name.c_str());

                    return true;
                }

                case CMD_BEAR_MIL:
                {
                    string obj_name=command.get(1).asString().c_str();
                    if(motorThr->isHolding()) // 
                    {
                        fprintf(stdout,"Deploying %s.\n",obj_name.c_str());
                        motorThr->deploy();
                    }

                    //if it is not currently tracking anything, start learning what it has in fixation
                    if(!visuoThr->isTracking())
                        visuoThr->getFixation();

                    //start learning
                    visuoThr->startLearningMIL(obj_name.c_str());

                    //first exploration
                    fprintf(stdout,"Looking at %s.\n",obj_name.c_str());
                    motorThr->exploreTorso(5.0);
                
                    //first push
                    visuoThr->suspendLearningMIL();

                    Vector s=visuoThr->getTrack();

                    if(s.size()==4)
                    {
                        Bottle b;
                        b.clear();
                        b.addString("fix");
                        b.addInt(s[0]);
                        b.addInt(s[1]);

                        b.addInt(100);
                        b.addInt(100);

                        wavePort.write(b);

                    }


                    Time::delay(1.0);

                    s=visuoThr->getTrack();

                    motorThr->push(s);
                    motorThr->goHome();
                    //end first push

                    //get again a more precise point
                    s=visuoThr->getTrack();

                    if(s.size()==4)
                    {
                        Bottle b;
                        b.clear();
                        b.addString("fix");
                        b.addInt(s[0]);
                        b.addInt(s[1]);

                        b.addInt(100);
                        b.addInt(100);

                        wavePort.write(b);

                    }
                
                    //second exploration
                    visuoThr->resumeLearningMIL();
 
                    fprintf(stdout,"Looking at %s.\n",obj_name.c_str());
                    motorThr->exploreTorso(5.0);
                
                    //second push
                    visuoThr->suspendLearningMIL();

                    s=visuoThr->getTrack();

                    if(s.size()==4)
                    {
                        Bottle b;
                        b.clear();
                        b.addString("fix");
                        b.addInt(s[0]);
                        b.addInt(s[1]);

                        b.addInt(100);
                        b.addInt(100);

                        wavePort.write(b);

                    }


                    Time::delay(1.0);

                    s=visuoThr->getTrack();


                    motorThr->push(s);
                    motorThr->goHome();
                    //end second push

                    //get again a more precise point
                    s=visuoThr->getTrack();

                    if(s.size()==4)
                    {
                        Bottle b;
                        b.clear();
                        b.addString("fix");
                        b.addInt(s[0]);
                        b.addInt(s[1]);

                        b.addInt(100);
                        b.addInt(100);

                        wavePort.write(b);

                    }
                
                    //second exploration
                    visuoThr->resumeLearningMIL();
 
                    fprintf(stdout,"Looking at %s.\n",obj_name.c_str());
                    motorThr->exploreTorso(5.0);

                    visuoThr->trainMIL();

                    motorThr->setGazeIdle();
                    motorThr->goHome();

                    reply.addString((obj_name + " learned").c_str());
                    fprintf(stdout,"'%s' learned.\n",obj_name.c_str());

                    return true;
                }

                case CMD_LEARN_MSR:
                {
                    string obj_name=command.get(1).asString().c_str();
                    if(!motorThr->isHolding())
                    {
                        reply.addString("failed");
                        motorThr->release();
                        motorThr->goHome();
                        return true;
                    }

                    if(!visuoThr->isTracking())
                    {
                        motorThr->lookAtHand(true);
                        visuoThr->getFixation();
                    }

                    fprintf(stdout,"Learning multiSensory structure of %s.\n",obj_name.c_str());
                    motorThr->setGazeIdle();

                    //visuoThr->resumeLearningMSR();
                    int arm=motorThr->getArmInUse();
                    if(visuoThr->startLearningMSR(obj_name.c_str(),arm))
                    {
                        bool done=false;
                        double t=Time::now();
                        visuoThr->resumeLearningMSR();
                        while(!done && !isStopping() && (Time::now()-t<explorationMSRWaitThresh))
                            visuoThr->checkDoneMSR(done);

                        if(!done)
                            visuoThr->stopMSR();
                         
                        fprintf(stdout,"Explored %s.\n",obj_name.c_str());
                        reply.addString((obj_name + " learned").c_str());
                        fprintf(stdout,"Setting gaze IDlE after MSR!\n");
                        
                    }
                    else
                    {
                        fprintf(stdout,"Error starting the MSR learning procedure!\n");
                        reply.addString("failed");
                    }

                    visuoThr->suspendLearningMSR();

                    motorThr->setImpedance(true);

                    motorThr->trackObject(true);
                    motorThr->drawNear();
                    motorThr->setGazeIdle();
                    return true;
                }

                case CMD_LEARN_ACTION:
                {
                    string action_name=command.get(1).asString().c_str();
                    int action_arm=ARM_IN_USE;
                    if(command.size()>2)
                        action_arm=command.get(2).asString()=="left"?LEFT:RIGHT;

                    motorThr->setGazeIdle();
                    if(!motorThr->startDragMode(action_name,action_arm))
                    {
                        motorThr->goHome();
                        reply.addString(("action "+action_name+" unknown").c_str());
                    }
                    else
                    {
                        motorThr->lookAtHand(true);
                        reply.addString("start drag");
                    }

                    return true;
                }

                case CMD_IMITATE_ACTION:
                {
                    string action_name=command.get(1).asString().c_str(); 
                    
                    int action_arm=ARM_IN_USE;
                    if(command.size()>2)
                        action_arm=command.get(2).asString()=="left"?LEFT:RIGHT;

                    motorThr->lookAtHand(true);
                    if(!motorThr->imitateAction(action_name,action_arm))
                    {
                        reply.addString("failed");
                    }
                    else
                    {
                        reply.addString(("action "+action_name+" done").c_str());
                    }                
                    motorThr->lookAtHand(false);

                    return true;
                }
            }

            //get the action target point in the images reference frame
            Vector stereo;

            switch(command.get(1).asVocab())
            {
                case PARAM_TABLE:
                {
                    stereo=visuoThr->getFixation();
                    motorThr->setGazeIdle();
                    break;
                }
                case PARAM_FIXATION:
                {
                    stereo=visuoThr->getFixation();
                    fprintf(stdout,"Getting fixation point.\n");
                    break;
                }
                case PARAM_MOTION:
                {
                    double t=Time::now();
                    while(!isStopping() && stereo.size()==0 && (Time::now()-t<motionDetectionWaitThresh))
                        stereo=visuoThr->getMotion();
                    break;
                }
                case PARAM_TRACK:
                {
                    stereo=visuoThr->getTrack();
                    break;
                }

                default:
                {
                    double t=Time::now();
                    while(!isStopping() && stereo.size()==0 && (Time::now()-t<objectDetectionWaitThresh))
                        stereo=visuoThr->getObject(command.get(1).asString().c_str());

                    break;
                }
            }

            // if no target point was obtained return
            if(stereo.size()==0)
            {
                reply.addString("failed");
                return true;
            }

            //send point to the maryland's segmentator
            Bottle b;
            b.clear();
            b.addString("fix");
            b.addInt(stereo[0]);
            b.addInt(stereo[1]);

            b.addInt(100);
            b.addInt(100);

            wavePort.write(b);

            // track the object
            motorThr->trackObject(true);

            // send the command
            switch(command.get(0).asVocab())
            {
                case TEST:
                {
                    motorThr->reachAbove(stereo,true);

                    string action_name="test";

                    motorThr->setGazeIdle();
                    if(!motorThr->startDragMode(action_name,ARM_IN_USE))
                    {
                        motorThr->goHome();
                        reply.addString(("action "+action_name+" unknown").c_str());
                    }
                    else
                    {
                        reply.addString("start drag");
                    }

                    return true;
                }

                case CMD_TAKE:
                {
                    motorThr->reachSpace(stereo);
                    motorThr->lookAtHand(true);
                    motorThr->grasp();

                    if(motorThr->isHolding())
                        motorThr->drawNear();

                    if(motorThr->isHolding())
                    {
                        motorThr->lookAtHand(false);
                        reply.addString("holding");
                    }
                    else
                    {
                       motorThr->lookAtHand(false);
                       motorThr->release();
                       motorThr->goHome();
                       reply.addString("failed");
                    }

                    return true;
                }

                case CMD_TOUCH:
                {
                    motorThr->reachAbove(stereo);
                    Time::delay(2.0);
                    motorThr->goHome();

                    reply.addString("touched");

                    return true;
                } 

                case CMD_REACH:
                {
                    motorThr->reachAboveSide(stereo,ARM_IN_USE);
                    motorThr->setGazeIdle();
                    
                    reply.addString("reached");
                    return true;
                }

                case CMD_PICK:
                {
                    motorThr->reachAbove(stereo,ARM_MOST_SUITED,true);
                    motorThr->lookAtHand(true);
                    motorThr->grasp();

                    if(motorThr->isHolding())
                    {
                        motorThr->drawNear();
                        if(motorThr->isHolding())
                        {
                            motorThr->lookAtHand(false);
                            reply.addString("holding");
                        }
                        else
                        {
                           motorThr->lookAtHand(false);
                           motorThr->release();
                           motorThr->goHome();
                           reply.addString("failed");
                        }
                    }
                    else
                    {
                       motorThr->lookAtHand(false);
                       motorThr->release();
                       motorThr->goHome();
                       reply.addString("failed");
                    }

                    

                    return true;
                }

                case CMD_GRASP:
                {
                    motorThr->reachSide(stereo);
                    motorThr->lookAtHand(true);
                    motorThr->grasp();

                    
                    if(motorThr->isHolding())
                    {
                        motorThr->drawNear();
                        if(motorThr->isHolding())
                        {
                            motorThr->lookAtHand(false);
                            reply.addString("holding");
                        }
                        else
                        {
                           motorThr->lookAtHand(false);
                           motorThr->release();
                           motorThr->goHome();
                           reply.addString("failed");
                        }
                    }
                    else
                    {
                       motorThr->lookAtHand(false);
                       motorThr->release();
                       motorThr->goHome();
                       reply.addString("failed");
                    }

                    return true;
                }

                case CMD_PUSH:
                {
                    motorThr->push(stereo);
                    motorThr->keepFixation(true);
                    motorThr->goHome();
                    motorThr->keepFixation(false);


                    

                    Bottle b;
                    b.clear();
                    b.addString("again");
                    //b.addString("fix");
                    //b.addInt(160);
                    //b.addInt(120);

                    //b.addInt(100);
                    //b.addInt(100);

                    wavePort.write(b);

                

                    reply.addString("Object pushed");
                    return true;
                }

                case CMD_POINT:
                {
                    motorThr->point(stereo);
                    reply.addString("pointing");
                    return true;
                }

                case CMD_FILL:
                {

                    if(!motorThr->isHolding())
                    {
                        reply.addString("Nothing to put. Not holding anything");
                        motorThr->release();
                        motorThr->goHome();
                        return true;
                    }

                    motorThr->trackObject(false);
                    motorThr->deploy(false,ARM_IN_USE,stereo);
                    motorThr->keepFixation(true);
                    //motorThr->lookAtHand(false);
                    motorThr->goHome();
                    motorThr->keepFixation(false);

                    reply.addString("put");
                    return true;
                }

                case CMD_TRACK:
                {
                    reply.addString("tracking");
                    return true;           
                }
            }
        }
        
        fprintf(stdout,"Command not recognized.\n");
        reply.addString("command not recognized.");
        return true;
    }
    /*
     * Get the percentile values from the skinDriftCompensation module and set the read values
     * in the grasping thread.
     * Return true if the operation succeeds, false otherwise.
     */
    bool updatePercentileLeft(){
	    Bottle commandBot, percentileBot;
	    for(unsigned int i=0; i<GET_PERCENTILE_COMMAND_LENGTH; i++)
		    commandBot.addString(GET_PERCENTILE_COMMAND[i].c_str());
        
	    skinDriftCompRpcPortLeft.write(commandBot, percentileBot);// send command, wait for reply

        fprintf(stdout, "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ done and size %d\n", percentileBot.size());
	    vector<float> percentile;
	    for(int i=0; i<percentileBot.size(); i++)
		    percentile.push_back((float)percentileBot.get(i).asDouble());
	
	    if(motorThr->setPercentile(percentile, LEFT)){
		    fprintf(stdout, "Set new percentile: %s\n\n", percentileBot.toString().c_str());
		    return true;
	    }	
	    fprintf(stderr, "ERROR while setting the new left percentile: %s\n", percentileBot.toString().c_str());
	    return false;
    }

    bool updatePercentileRight(){
	    Bottle commandBot, percentileBot;
	    for(unsigned int i=0; i<GET_PERCENTILE_COMMAND_LENGTH; i++)
		    commandBot.addString(GET_PERCENTILE_COMMAND[i].c_str());
        
	    skinDriftCompRpcPortRight.write(commandBot, percentileBot);// send command, wait for reply

        fprintf(stdout, "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$ done and size %d\n", percentileBot.size());
	    vector<float> percentile;
	    for(int i=0; i<percentileBot.size(); i++)
		    percentile.push_back((float)percentileBot.get(i).asDouble());
	
	    if(motorThr->setPercentile(percentile, RIGHT)){
		    fprintf(stdout, "Set new percentile: %s\n\n", percentileBot.toString().c_str());
		    return true;
	    }	
	    fprintf(stderr, "ERROR while setting the new right percentile: %s\n", percentileBot.toString().c_str());
	    return false;
    }
};


int main(int argc, char *argv[])
{
    Network yarp;   

    if (!yarp.checkNetwork())
        return -1;

    YARP_REGISTER_DEVICES(icubmod)

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("poeticonVMB/conf");
    rf.setDefaultConfigFile("config.ini");
    rf.setDefault("name","poeticonVMB");
    rf.configure("ICUB_ROOT",argc,argv);

    VisuoMotorModule mod;

    return mod.runModule(rf);
}


