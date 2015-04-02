/** 
@ingroup chris_module
\defgroup chris_icubDemo2010MotorIF icubDemo2010MotorIF
 
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

\author Ugo Pattacini
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/Drivers.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>
#include <iCub/ctrl/math.h>
#include <iCub/action/actionPrimitives.h>
#include <yarp/dev/GazeControl.h>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <deque>
#include <map>

#define USE_LEFT                    0
#define USE_RIGHT                   1
                                    
#define CMD_TAKE                    VOCAB4('t','a','k','e')
#define CMD_TOUCH                   VOCAB4('t','o','u','c')
#define CMD_GRASP                   VOCAB4('g','r','a','s')
#define CMD_TAP                     VOCAB3('t','a','p')
#define CMD_CALIB                   VOCAB4('c','a','l','i')
                                    
#define OPT_TABLE                   VOCAB4('t','a','b','l')
#define OPT_KIN                     VOCAB3('k','i','n')
#define OPT_START                   VOCAB4('s','t','a','r')
#define OPT_STOP                    VOCAB4('s','t','o','p')
#define OPT_CLEAR                   VOCAB4('c','l','e','a')
#define OPT_LEFT                    VOCAB4('l','e','f','t')
#define OPT_RIGHT                   VOCAB4('r','i','g','h')
#define OPT_ON                      VOCAB2('o','n')
#define OPT_OFF                     VOCAB3('o','f','f')

#define KINCALIB_PERIOD             20      // [ms]
#define HOMING_PERIOD               2.0     // [s]

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


// this class manages the kinematic offsets
class kinematicOffset
{
protected:
    struct offsElement
    {
        Vector point;
        Vector offset;
    };

    deque<offsElement> offsList;
    string fileName;
    bool toBeSaved;

public:
    kinematicOffset()
    {
        fileName="";
        toBeSaved=false;
    }

    void clear()
    {
        if (offsList.size())
        {
            offsList.clear();
            toBeSaved=true;
        }
    }

    void insert(const Vector &point, const Vector &offset)
    {
        offsElement el;
        el.point=point;
        el.offset=offset;

        offsList.push_back(el);
        toBeSaved=true;

        cout<<"########## Kinematic offset added: ["<<el.offset.toString()<<"]"<<endl;
    }

    Vector get(const Vector &in)
    {
        Vector out(3);
        out=0.0;

        if (offsList.size())
        {
            double sumWeight=0.0;

            for (unsigned int i=0; i<offsList.size(); i++)
            {
                Vector &point=offsList[i].point;
                Vector &offset=offsList[i].offset;

                double weight=norm(in-point);

                if (weight)
                    weight=1.0/weight;
                else
                    return out=offset;

                out=out+weight*offset;
                sumWeight+=weight;
            }

            out=(1.0/sumWeight)*out;
        }

        return out;
    }

    void load(const string &_fileName)
    {        
        fileName=_fileName;

        Property config;
        config.fromConfigFile(fileName.c_str());
        if (config.isNull())
            return;

        // parsing general part
        Bottle &bGeneral=config.findGroup("general");
        if (bGeneral.isNull())
            return;
       
        int numItems=bGeneral.check("numItems",Value(0)).asInt();

        // parsing general config options
        for (int i=0; i<numItems; i++)
        {
            char item[255];
            sprintf(item,"item_%d",i);
            Bottle &bItem=config.findGroup(item);

            if (bItem.isNull())
                continue;

            Vector point, offset;
            point.resize(3,0.0);
            offset.resize(3,0.0);

            if (Bottle *pB=bItem.find("point").asList())
            {
                int sz=pB->size();
                int len=sz<3?sz:3;

                for (int i=0; i<pB->size(); i++)
                    point[i]=pB->get(i).asDouble();
            }

            if (Bottle *pB=bItem.find("offset").asList())
            {
                int sz=pB->size();
                int len=sz<3?sz:3;

                for (int i=0; i<pB->size(); i++)
                    offset[i]=pB->get(i).asDouble();
            }

            insert(point,offset);
        }

        toBeSaved=false;
    }

    void save()
    {
        if (toBeSaved)
        {
            ofstream fout;
            fout.open(fileName.c_str());
    
            // general part
            fout<<endl;
            fout<<"[general]"<<endl;
            fout<<"numItems\t"<<offsList.size()<<endl;
            fout<<endl;
    
            // items part
            for (unsigned int i=0; i<offsList.size(); i++)
            {
                Vector &point=offsList[i].point;
                Vector &offset=offsList[i].offset;
    
                fout<<"[item_"<<i<<"]"<<endl;
                fout<<"point\t("<<point.toString()<<")"<<endl;
                fout<<"offset\t("<<offset.toString()<<")"<<endl;
                fout<<endl;
            }
    
            fout.close();
        }
    }

    ~kinematicOffset()
    {
        save();
        offsList.clear();
    }
};


// this class handles the offset to be added to the
// current table heigth according to the feedback
// coming from contact detection
class tableManager
{
protected:
    string fileName;
    double heightOffset;

    string id;
    Vector position;

    string sender;
    string receiver;

public:
    tableManager()
    {
        fileName="";
        heightOffset=0.0;

        id="";
        position.resize(3,0.0);

        sender="";
        receiver="";
    }

    void initTxParams(const string &_sender, const string &_receiver)
    {
        sender=_sender;
        receiver=_receiver;
    }

    void load(const string &_fileName)
    {        
        fileName=_fileName;

        Property config;
        config.fromConfigFile(fileName.c_str());
        if (config.isNull())
            return;

        // parsing id option
        id=config.check("id",Value("")).asString().c_str();

        // parsing POSITION part
        Bottle &bPosition=config.findGroup("POSITION");
        if (bPosition.isNull())
            return;

        position[0]=bPosition.check("x",Value(0.0)).asDouble();
        position[1]=bPosition.check("y",Value(0.0)).asDouble();
        position[2]=bPosition.check("z",Value(0.0)).asDouble();

        heightOffset=0.0;
    }

    void save()
    {
        if (heightOffset)
        {
            ofstream fout;
            fout.open(fileName.c_str());

            // id option
            fout<<endl;
            fout<<"id\t"<<id<<endl;
            fout<<endl;

            // position part
            fout<<"[POSITION]"<<endl;
            fout<<"x\t"<<position[0]<<endl;
            fout<<"y\t"<<position[1]<<endl;
            fout<<"z\t"<<(position[2]+heightOffset)<<endl;

            fout.close();
        }
    }

    void setTableHeight(const double height)
    {
        heightOffset=height-position[2];

        if (heightOffset)
        {
            Port port;
            port.open(sender.c_str());

            if (Network::connect(sender.c_str(),receiver.c_str()))
            {
                Bottle cmd, reply;
                cmd.addString("set");
                cmd.addString("heightOffset");
                cmd.addDouble(heightOffset);
                   
                port.write(cmd,reply);
            }
            else
                cout<<"Warning: unable to connect to "<<receiver<<endl;

            port.close();
        }
    }

    ~tableManager()
    {
        save();
    }
};


// this class handles the manual calibration of the arm
// with the goal to find out the kinematic offsets
class kinematicCalibrator : public RateThread
{
protected:
    ActionPrimitivesLayer2 *action;

    double forceThres;
    double gain;

    Vector x0;
    Vector o0;
    Vector x;

public:
    kinematicCalibrator() : RateThread(KINCALIB_PERIOD)
    {
        action=NULL;
        forceThres=1e9;
        gain=0.0;

        x0.resize(3,0.0);
        o0.resize(4,0.0);
        x.resize(3,0.0);
    }

    void init(ActionPrimitivesLayer2 *_action, const Vector &_x0,
              const double thres)
    {
        action=_action;
        forceThres=thres;

        x0=_x0;
        action->getPose(x,o0);
    }

    void   set_gain(const double _gain) { gain=_gain;    }
    Vector get_x0() const               { return x0;     }
    Vector get_x() const                { return x;      }
    Vector get_offset() const           { return (x-x0); }

    virtual void run()
    {
        if (action!=NULL)
        {
            Vector wrench, force(3);

            // get the wrench at the end-effector
            action->getExtWrench(wrench);            
            force[0]=wrench[0];
            force[1]=wrench[1];
            force[2]=wrench[2];

            // yield a small displacement
            // iff the force overcomes the threshold
            if (norm(force)>forceThres)
            {
                Vector o;
                action->getPose(x,o);
                action->reachPose(x+gain*force,o0);
            }
        }
    }

    virtual void threadRelease()
    {
        if (action!=NULL)
            action->stopControl();
    }
};


class graspModule: public RFModule
{
protected:
    ResourceFinder *rf;
    string partUsed;
    string armToBeUsed;

    PolyDriver        *drvGazeCtrl;
    IGazeControl      *gazeCtrl;
    int               initial_gaze_context;

    ActionPrimitivesLayer2       *actionL;
    ActionPrimitivesLayer2       *actionR;
    ActionPrimitivesLayer2       *action;

    map<string,Matrix>            palmOrientations;

    BufferedPort<Bottle>          cmdPort;
    Port                          rpcPort;

    kinematicCalibrator           kinCalib;
    kinematicOffset               dOffsL;
    kinematicOffset               dOffsR;
    tableManager                  tableMan;

    Vector graspOrienL,           graspOrienR;
    Vector graspDispL,            graspDispR;
    Vector graspReliefL,          graspReliefR;
    Vector dLiftL,                dLiftR;
    Vector dTouchL,               dTouchR;
    Vector dTapL,                 dTapR;
    Vector home_xL,               home_xR;
    double dropLengthL,           dropLengthR;
    double forceCalibTableThresL, forceCalibTableThresR;
    double forceCalibKinThresL,   forceCalibKinThresR;

    Vector *graspOrien;
    Vector *graspDisp;
    Vector *graspRelief;
    Vector *dLift;
    Vector *dTouch;
    Vector *dTap;
    Vector *home_x;
    double *dropLength;
    double *forceCalibKinThres;
    kinematicOffset *dOffs;

    Vector home_fp;

    double targetInRangeThres;    

    bool openPorts;
    bool firstRun;
    bool running;
    bool trackMode;
    bool use_opdb;

public:
    graspModule()
    {
        computePalmOrientations();
           
        graspOrienL.resize(4);     graspOrienR.resize(4);
        graspDispL.resize(4);      graspDispR.resize(3);
        graspReliefL.resize(4);    graspReliefR.resize(3);
        dLiftL.resize(3);          dLiftR.resize(3);
        dTouchL.resize(3);         dTouchR.resize(3);
        dTapL.resize(3);           dTapR.resize(3);
        home_xL.resize(3);         home_xR.resize(3);
        home_fp.resize(3);

        graspOrienL=dcm2axis(palmOrientations["left_down"]);
        graspOrienR=dcm2axis(palmOrientations["right_down"]);

        // default values for arm-dependent quantities
        graspDispL[0]=0.0;         graspDispR[0]=0.0;
        graspDispL[1]=0.0;         graspDispR[1]=0.0; 
        graspDispL[2]=0.05;        graspDispR[2]=0.05;

        graspReliefL[0]=0.0;       graspReliefR[0]=0.0; 
        graspReliefL[1]=0.0;       graspReliefR[1]=0.0; 
        graspReliefL[2]=0.02;      graspReliefR[2]=0.02;

        dLiftL[0]=0.0;             dLiftR[0]=0.0;  
        dLiftL[1]=0.0;             dLiftR[1]=0.0;  
        dLiftL[2]=0.15;            dLiftR[2]=0.15; 
                                    
        dTouchL[0]=0.0;            dTouchR[0]=0.0;
        dTouchL[1]=0.0;            dTouchR[1]=0.0;  
        dTouchL[2]=0.0;            dTouchR[2]=0.0;
                                   
        dTapL[0]=0.0;              dTapR[0]=0.0;
        dTapL[1]=0.0;              dTapR[1]=0.0;  
        dTapL[2]=0.0;              dTapR[2]=0.0;

        dropLengthL=0.0;           dropLengthR=0.0;
        forceCalibTableThresL=1e9; forceCalibTableThresR=1e9;
        forceCalibKinThresL=1e9;   forceCalibKinThresR=1e9;

        home_xL[0]=-0.29;          home_xR[0]=-0.29;
        home_xL[1]=-0.21;          home_xR[1]= 0.24;
        home_xL[2]= 0.11;          home_xR[2]= 0.07;

        home_fp[0]=-0.3;
        home_fp[1]=0.0;
        home_fp[2]=0.3;

        action=actionL=actionR=NULL;
        graspOrien=NULL;
        graspDisp=NULL;
        graspRelief=NULL;
        dOffs=NULL;
        dLift=NULL;
        dTouch=NULL;
        dTap=NULL;
        home_x=NULL;
        dropLength=NULL;

        openPorts=false;
        firstRun=true;
        running=false;

        Rand::init();
    }

    void getArmDependentOptions(Bottle &b, kinematicOffset &_dOffs,
                                Vector &_gDisp, Vector &_gRelief,
                                Vector &_dLift, Vector &_dTouch,
                                Vector &_dTap, Vector &_home_x, double &_dropLength,
                                double &_forceCalibTableThres, double &_forceCalibKinThres)
    {
        string kinOffsfileName=b.find("kinematic_offsets_file").asString().c_str();
        _dOffs.load(rf->findFile(kinOffsfileName.c_str()).c_str());

        if (Bottle *pB=b.find("grasp_displacement").asList())
        {
            int sz=pB->size();
            int len=_gDisp.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _gDisp[i]=pB->get(i).asDouble();
        }

        if (Bottle *pB=b.find("grasp_relief").asList())
        {
            int sz=pB->size();
            int len=_gRelief.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _gRelief[i]=pB->get(i).asDouble();
        }

        if (Bottle *pB=b.find("lifting_displacement").asList())
        {
            int sz=pB->size();
            int len=_dLift.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _dLift[i]=pB->get(i).asDouble();
        }

        if (Bottle *pB=b.find("touching_displacement").asList())
        {
            int sz=pB->size();
            int len=_dTouch.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _dTouch[i]=pB->get(i).asDouble();
        }

        if (Bottle *pB=b.find("tapping_displacement").asList())
        {
            int sz=pB->size();
            int len=_dTap.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _dTap[i]=pB->get(i).asDouble();
        }

        if (Bottle *pB=b.find("home_position").asList())
        {
            int sz=pB->size();
            int len=_home_x.length();
            int l=len<sz?len:sz;

            for (int i=0; i<l; i++)
                _home_x[i]=pB->get(i).asDouble();
        }

        if (b.check("dropping_length"))
            _dropLength=b.find("dropping_length").asDouble();

        if (b.check("force_calib_table_thres"))
            _forceCalibTableThres=b.find("force_calib_table_thres").asDouble();

        if (b.check("force_calib_kin_thres"))
            _forceCalibKinThres=b.find("force_calib_kin_thres").asDouble();
    }

    virtual bool configure(ResourceFinder &rf)
    {
        this->rf=&rf;

        string name=rf.find("name").asString().c_str();
        setName(name.c_str());

        // configure the gaze controller
        Property optGazeCtrl("(device gazecontrollerclient)");

        optGazeCtrl.put("remote","/iKinGazeCtrl");
        optGazeCtrl.put("local",("/"+name+"/gaze").c_str());

        drvGazeCtrl=new PolyDriver;
        if (!drvGazeCtrl->open(optGazeCtrl))
        {
            close();
            return false;
        }

        drvGazeCtrl->view(gazeCtrl);

        gazeCtrl->storeContext(&initial_gaze_context);
        gazeCtrl->setEyesTrajTime(0.5);
        gazeCtrl->setNeckTrajTime(1.0);
        gazeCtrl->setTrackingMode(true);
        gazeCtrl->lookAtFixationPoint(home_fp);

        Property optCartArm("(device cartesiancontrollerclient)");

        optCartArm.put("remote","/icub/cartesianController/right_arm");
        optCartArm.put("local",("/"+name+"/cartesian").c_str());

        partUsed=rf.check("part",Value("both_arms")).asString().c_str();
        if (partUsed!="both_arms" && partUsed!="left_arm" && partUsed!="right_arm")
        {
            cout<<"Invalid part requested !"<<endl;
            return false;
        }

        Property config; config.fromConfigFile(rf.findFile("from").c_str());
        Bottle &bGeneral=config.findGroup("general");
        if (bGeneral.isNull())
        {
            cout<<"Error: group general is missing!"<<endl;
            return false;
        }

        // parsing general config options
        Property option;
        for (int i=1; i<bGeneral.size(); i++)
        {
            Bottle *pB=bGeneral.get(i).asList();
            if (pB->size()==2)
                option.put(pB->get(0).asString().c_str(),pB->get(1));
            else
            {
                cout<<"Error: invalid option!"<<endl;
                return false;
            }
        }

        option.put("local",name.c_str());
        option.put("hand_sequences_file",rf.findFile("hand_sequences_file").c_str());

        Property optionL(option); optionL.put("part","left_arm");
        Property optionR(option); optionR.put("part","right_arm");

    
        fprintf(stdout,"################# %s",optionL.toString().c_str());

        // get tracking mode
        trackMode=(option.check("tracking_mode",Value("off")).asString()=="on");

        // get kinematic calibration options
        kinCalib.set_gain(option.check("calib_kin_gain",Value(0.0)).asDouble());

        // get in range threshold
        targetInRangeThres=option.check("target_in_range_thres",Value(1e9)).asDouble();

        // get table configuration
        tableMan.load(rf.findFile("table_file").c_str());

        // init table parameters transmission
        tableMan.initTxParams(("/"+name+"/table:o").c_str(),
                              rf.find("homography_port").asString().c_str());

        // parsing left_arm config options
        Bottle &bLeft=config.findGroup("left_arm");
        if (bLeft.isNull())
        {
            cout<<"Error: group left_arm is missing!"<<endl;
            return false;
        }
        else
            getArmDependentOptions(bLeft,dOffsL,graspDispL,graspReliefL,
                                   dLiftL,dTouchL,dTapL,home_xL,dropLengthL,
                                   forceCalibTableThresL,forceCalibKinThresL);

        // parsing right_arm config options
        Bottle &bRight=config.findGroup("right_arm");
        if (bRight.isNull())
        {
            cout<<"Error: group right_arm is missing!"<<endl;
            return false;
        }
        else
            getArmDependentOptions(bRight,dOffsR,graspDispR,graspReliefR,
                                   dLiftR,dTouchR,dTapR,home_xR,dropLengthR,
                                   forceCalibTableThresR,forceCalibKinThresR);

        // set up the reaching timeout
        double reachingTimeout=2.0*option.check("default_exec_time",Value("3.0")).asDouble();

        if (partUsed=="both_arms" || partUsed=="left_arm")
        {
            cout<<"***** Instantiating primitives for left_arm"<<endl;
            actionL=new ActionPrimitivesLayer2(optionL);
			actionL->setExtForceThres(forceCalibTableThresL);
            actionL->enableReachingTimeout(reachingTimeout);

            if (!actionL->isValid())
            {
                close();
                return false;
            }
            else
                useArm(USE_LEFT);
        }

        if (partUsed=="both_arms" || partUsed=="right_arm")
        {
            cout<<"***** Instantiating primitives for right_arm"<<endl;
            actionR=new ActionPrimitivesLayer2(optionR);
            actionR->setExtForceThres(forceCalibTableThresR);
            actionR->enableReachingTimeout(reachingTimeout);

            if (!actionR->isValid())
            {
                close();
                return false;
            }
            else
                useArm(USE_RIGHT);
        }

        // print out the hand sequences
        deque<string> q=action->getHandSeqList();
        cout<<"***** List of available hand sequence keys:"<<endl;
        for (size_t i=0; i<q.size(); i++)
        {
            Bottle sequence;
            action->getHandSequence(q[i],sequence);

            cout<<"***** "<<q[i]<<":"<<endl;
            cout<<sequence.toString()<<endl;
        }

        cmdPort.open(("/"+name+"/cmd:i").c_str());
        rpcPort.open(("/"+name+"/rpc").c_str());
        attach(rpcPort);

        openPorts=true;

        return true;
    }

    virtual bool close()
    {
        gazeCtrl->restoreContext(initial_gaze_context);
        delete drvGazeCtrl;

        if (actionL!=NULL)
            delete actionL;

        if (actionR!=NULL)
            delete actionR;        

        if (openPorts)
        {
            cmdPort.close();
            rpcPort.close();
        }

        return true;
    }

    virtual double getPeriod()
    {
        return 0.1;
    }

    void computePalmOrientations()
    {
        Matrix Ry(3,3);
        Ry.zero();
        Ry(0,0)=cos(M_PI);
        Ry(0,2)=sin(M_PI);
        Ry(1,1)=1.0;
        Ry(2,0)=-Ry(0,2);
        Ry(2,2)=Ry(0,0);
        
        palmOrientations["right_down"]=Ry;
        
        Matrix Rx(3,3);
        Rx.zero();
        Rx(0,0)=1.0;
        Rx(1,1)=cos(M_PI);
        Rx(1,2)=-sin(M_PI);
        Rx(2,1)=-Rx(1,2);
        Rx(2,2)=Rx(1,1);
        
        palmOrientations["left_down"]=Ry*Rx;
        
        Rx(1,1)=cos(M_PI/2.0);
        Rx(1,2)=-sin(M_PI/2.0);
        Rx(2,1)=-Rx(1,2);
        Rx(2,2)=Rx(1,1);
        
        palmOrientations["right_base"]=palmOrientations["right_down"]*Rx;
        
        Rx(1,1)=cos(-M_PI/2.0);
        Rx(1,2)=-sin(-M_PI/2.0);
        Rx(2,1)=-Rx(1,2);
        Rx(2,2)=Rx(1,1);
        
        palmOrientations["left_base"]=palmOrientations["left_down"]*Rx;
        
        Matrix Rz(3,3);
        Rz.zero();
        Rz(0,0)=cos(M_PI/4.0);
        Rz(0,1)=-sin(M_PI/4.0);
        Rz(1,0)=-Rz(0,1);
        Rz(1,1)=Rz(0,0);
        Rz(2,2)=1.0;
        
        palmOrientations["right_starttap"]=palmOrientations["right_base"];
        palmOrientations["left_starttap"] =palmOrientations["left_base"];
        
        Rx(1,1)=cos(M_PI/8.0);
        Rx(1,2)=-sin(M_PI/8.0);
        Rx(2,1)=-Rx(1,2);
        Rx(2,2)=Rx(1,1);
        
        palmOrientations["right_stoptap"]=palmOrientations["right_starttap"];
        
        Rx(1,1)=cos(-M_PI/8.0);
        Rx(1,2)=-sin(-M_PI/8.0);
        Rx(2,1)=-Rx(1,2);
        Rx(2,2)=Rx(1,1);
        
        palmOrientations["left_stoptap"]=palmOrientations["left_starttap"];
    }

    void useArm(const int arm)
    {
        if (arm==USE_LEFT)
        {
            action=actionL;
            armToBeUsed="left";

            graspOrien=&graspOrienL;
            graspDisp=&graspDispL;
            graspRelief=&graspReliefL;
            dOffs=&dOffsL;
            dLift=&dLiftL;
            dTouch=&dTouchL;
            dTap=&dTapL;
            home_x=&home_xL;
            dropLength=&dropLengthL;
            forceCalibKinThres=&forceCalibKinThresL;
        }
        else if (arm==USE_RIGHT)
        {
            action=actionR;
            armToBeUsed="right";

            graspOrien=&graspOrienR;
            graspDisp=&graspDispR;
            graspRelief=&graspReliefR;
            dOffs=&dOffsR;
            dLift=&dLiftR;
            dTouch=&dTouchR;
            dTap=&dTapR;
            home_x=&home_xR;
            dropLength=&dropLengthR;
            forceCalibKinThres=&forceCalibKinThresR;
        }
    }

    void goHome()
    {
        bool f;
        // regardless of the current settings
        // go home in tracking mode
        bool latchTrackMode=trackMode;
        setTrackingMode(true);

        if (partUsed=="both_arms" || partUsed=="right_arm")
        {
            useArm(USE_RIGHT);
            action->pushAction(*home_x,"open_hand",HOMING_PERIOD);
            action->checkActionsDone(f,true);
            action->enableArmWaving(*home_x);
        }

        if (partUsed=="both_arms" || partUsed=="left_arm")
        {
            useArm(USE_LEFT);
            action->pushAction(*home_x,"open_hand",HOMING_PERIOD);
            action->checkActionsDone(f,true);
            action->enableArmWaving(*home_x);
        }

        gazeCtrl->lookAtFixationPoint(home_fp);

        setTrackingMode(latchTrackMode);
    }

    Vector retrieveTargetAndPrepareArm(Bottle *b)
    {
        Vector xd(3);
        xd[0]=b->get(1).asDouble();
        xd[1]=b->get(2).asDouble();
        xd[2]=b->get(3).asDouble();

        // switch only if it's allowed
        if (partUsed=="both_arms")
        {
            if (xd[1]>0.0)
                useArm(USE_RIGHT);
            else
                useArm(USE_LEFT);
        }

        // maintain the height in order
        // to use contact detection

        // distance safe thresholding
        xd[0]=xd[0]>-0.1?-0.1:xd[0];

        return xd;
    }

    Vector getReachGraspOrien(const Vector &xd)
    {
        Vector x(3),y(3),z(3);
        x=0.0; x[0]=-1.0;
        y=0.0; y[2]=-1.0;
        z=0.0; z[1]=-1.0;

        Matrix T(3,3);
        T.setCol(0,x);
        T.setCol(1,y);
        T.setCol(2,z);

        return dcm2axis(T);
    }

    Vector getObj2Eyes(const Vector &xd)
    {
        Vector x(3),z(3),head(3);
        head=0.0; head[2]=0.4;
        if(armToBeUsed=="right")
        {
            x=0.0; x[1]=-1.0;
            z=head-xd;
        }
        else
        {
            x=0.0; x[1]=1.0;
            z=xd-head;
        }
        Vector y=cross(z,x);
        fprintf(stdout,"x: %s\n",x.toString().c_str());
        fprintf(stdout,"y: %s\n",y.toString().c_str());
        fprintf(stdout,"z: %s\n",z.toString().c_str());

        y=(1.0/norm(y))*y;
        z=(1.0/norm(z))*z;

        Matrix T(3,3);
        T.setCol(0,x);
        T.setCol(1,y);
        T.setCol(2,z);

        return dcm2axis(T);
    }

    bool isTargetInRange(const Vector &xd)
    {
        if (norm(xd)<targetInRangeThres)
            return true;
        else
            return false;
    }

    void take(const Vector &xd)
    {
        bool f=false;

        Vector exploreObjPos(3),exploreObjPosFp(3);
        exploreObjPos[0]=-0.28;
        exploreObjPos[1]=armToBeUsed=="right"?0.12:-0.12;
        exploreObjPos[2]=0.14;
        
        exploreObjPosFp[0]=-0.28;
        exploreObjPosFp[1]=armToBeUsed=="right"?0.12:-0.12;
        exploreObjPosFp[2]=0.25;



        Vector deployZone(3);
        deployZone[0]=-0.3;
        deployZone[1]=0.0;
        deployZone[2]=-0.1;

        gazeCtrl->lookAtFixationPoint(xd);

        Vector reachGraspOrient=getReachGraspOrien(xd);

        Vector d(3);
        d=0.0;
        //d[1]=armToBeUsed=="right"?0.05:-0.05;

        action->latchWrenchOffset();
        action->enableContactDetection();
        action->grasp(xd,reachGraspOrient,d);
        action->checkActionsDone(f,true);
        action->areFingersInPosition(f);

        // if fingers are not in position then
        // this means that we have hopefully grabbed
        // the object
        if (true)//!f)
        {
            Vector randOffs=*dropLength*Rand::vector(3);
            randOffs[0]-=*dropLength/2.0;
            randOffs[1]-=*dropLength/2.0;
            randOffs[2]=0.0;

            // look at the object
            gazeCtrl->lookAtFixationPoint(exploreObjPosFp);

            Vector orientTowardEyes=getObj2Eyes(exploreObjPos);
            action->pushAction(exploreObjPos,orientTowardEyes);
            action->pushWaitState(3.0);
            action->checkActionsDone(f,true);

            // prepare hand for deployment
            action->pushAction(exploreObjPos,*graspOrien,2.0);

            // deploy the object
            gazeCtrl->lookAtFixationPoint(deployZone);

            action->pushWaitState(2.0);
            action->pushAction(deployZone/*+randOffs*/,*graspOrien,3.0);
            action->checkActionsDone(f,true);
            
            Vector x,o;
            action->getPose(x,o);
            x[2]+=0.1;            
            action->pushAction(x,*graspOrien,2.0);
            action->checkActionsDone(f,true);
        }

        action->disableContactDetection();

        gazeCtrl->lookAtFixationPoint(home_fp);

        action->pushAction("release_hand");
        action->pushWaitState(2.0);
        action->pushAction(*home_x,HOMING_PERIOD);
        action->checkActionsDone(f,true);
        goHome();
    }

    void point(const Vector &xd)
    {
        bool f=false;

        action->latchWrenchOffset();
        action->enableContactDetection();
        action->pushWaitState(1.0);
        action->pushAction(xd,*graspOrien,"open_hand");
        action->checkActionsDone(f,true);
        action->pushWaitState(2.0);
        action->disableContactDetection();
        action->pushAction(*home_x,HOMING_PERIOD);
        action->checkActionsDone(f,true);
        goHome();
    }

    void touch(const Vector &xd)
    {        
        bool f=false;

        action->touch(xd,*graspOrien,*dTouch);
        action->pushWaitState(2.0);
        action->pushAction(xd+*dTouch,*graspOrien,HOMING_PERIOD);
        action->pushAction(*home_x,"open_hand",HOMING_PERIOD);
        action->checkActionsDone(f,true);
        goHome();
    }

    void grasp(const Vector &xd)
    {
        bool f=false;

        action->grasp(xd,*graspOrien,*graspDisp,*graspRelief);
        action->checkActionsDone(f,true);
        action->areFingersInPosition(f);

        // if fingers are not in position then
        // this means that we have hopefully grabbed
        // the object
        if (!f)
        {
            Vector randOffs=*dropLength*Rand::vector(3);
            randOffs[0]-=*dropLength/2.0;
            randOffs[1]-=*dropLength/2.0;
            randOffs[2]=0.0;

            action->pushAction(xd+*dLift,*graspOrien);
            action->pushWaitState(2.0);
            action->checkActionsDone(f,true);
            action->latchWrenchOffset();
            action->enableContactDetection();
            action->pushWaitState(1.0);
            action->pushAction(xd+randOffs,*graspOrien,3.0);
            action->checkActionsDone(f,true);
            action->disableContactDetection();

            Vector x,o;
            action->getPose(x,o);
            x[2]+=0.02;
            action->pushAction(x,*graspOrien);
            action->pushWaitState(2.0);
        }

        action->pushAction("release_hand");
        action->pushWaitState(2.0);
        action->pushAction(xd+*graspDisp,*graspOrien,HOMING_PERIOD);
        action->pushAction(*home_x,HOMING_PERIOD);
        action->checkActionsDone(f,true);
        goHome();
    }

    void tap(const Vector &xd)
    {
        bool f=false;

        Vector startPos=xd+*dTap;
        Vector endPos=startPos;
        endPos[1]-=2.0*(*dTap)[1];

        Vector startOrientation=dcm2axis(palmOrientations[armToBeUsed+"_starttap"]);
        Vector stopOrientation=dcm2axis(palmOrientations[armToBeUsed+"_stoptap"]);

        action->tap(startPos,startOrientation,endPos,stopOrientation,1.0);
        action->pushWaitState(2.0);
        action->pushAction(*home_x,dcm2axis(palmOrientations[armToBeUsed+"_down"]),"open_hand",HOMING_PERIOD);
        action->checkActionsDone(f,true);
        goHome();
    }

    // we don't need a thread since the actions library already
    // incapsulates one inside dealing with all the tight time constraints
    virtual bool updateModule()
    {
        // do it only once
        if (firstRun)
        {
            goHome();
            firstRun=false;
        }

        // get a target object position from a YARP port
        Bottle *b=cmdPort.read(true); 

        if (b!=NULL)
        {
            int cmd=b->get(0).asVocab();

            switch (cmd)
            {
                // execute a reach and then a grasp
                case CMD_TAKE:
                {
                    Vector xd=retrieveTargetAndPrepareArm(b);

                    running=true;

                    if (isTargetInRange(xd))
                        take(xd);
                    else
                    {
                        cout<<"########## Target out of range ... pointing"<<endl;
                        //point(xd);
                    }

                    running=false;
                    break;
                }

                // execute a touch
                case CMD_TOUCH:
                {
                    Vector xd=retrieveTargetAndPrepareArm(b);

                    running=true;

                    if (isTargetInRange(xd))
                        touch(xd);
                    else
                    {
                        cout<<"########## Target out of range ... pointing"<<endl;
                        point(xd);
                    }

                    running=false;
                    break;
                }

                // execute a grasp
                case CMD_GRASP:
                {
                    Vector xd=retrieveTargetAndPrepareArm(b);

                    running=true;

                    if (isTargetInRange(xd))
                        grasp(xd);
                    else
                    {
                        cout<<"########## Target out of range ... pointing"<<endl;
                        point(xd);
                    }

                    running=false;
                    break;
                }

                // execute a tap
                case CMD_TAP:
                {
                    Vector xd=retrieveTargetAndPrepareArm(b);
    
                    running=true;
    
                    if (isTargetInRange(xd))
                        tap(xd);
                    else
                    {
                        cout<<"########## Target out of range ... pointing"<<endl;
                        point(xd);
                    }
    
                    running=false;
                    break;
                }

                // perform calibration
                case CMD_CALIB:
                {
                    int kind=b->get(1).asVocab();

                    // find out the table height
                    if (kind==OPT_TABLE)
                    {    
                        Vector x0(3), x1(3);
                        x0[0]=x1[0]=Rand::scalar(-0.35,-0.30);
                        x0[1]=x1[1]=Rand::scalar(-0.05,0.05);
                        x0[2]=0.1; x1[2]=-0.2;

                        // switch only if it's allowed
                        if (partUsed=="both_arms")
                        {
                            if (x1[1]>0.0)
                                useArm(USE_RIGHT);
                            else
                                useArm(USE_LEFT);
                        }

                        running=true;
                        bool f=false;
    
                        action->pushAction(x0,*graspOrien,"open_hand");
                        action->checkActionsDone(f,true);
                        action->latchWrenchOffset();
                        action->enableContactDetection();
                        action->pushWaitState(1.0);
                        action->pushAction(x1,*graspOrien,3.0);
                        action->checkActionsDone(f,true);
                        action->pushWaitState(2.0);
                        action->disableContactDetection();
                        action->checkContact(f);

                        if (f)
                        {
                            Vector x,o;
                            action->getPose(x,o);
                            double tableHeight=x[2];

                            cout<<"########## Table height found = "<<tableHeight<<endl;
                            tableMan.setTableHeight(tableHeight);
                        }
                        else
                            cout<<"########## Table not found"<<endl;

                        action->pushAction(x0,*graspOrien,HOMING_PERIOD);
                        action->pushAction(*home_x,HOMING_PERIOD);
                        action->checkActionsDone(f,true);
                        goHome();

                        running=false;
                    }
                    // kinematic offset calibration
                    else if (kind==OPT_KIN)
                    {
                        int subcmd=b->get(2).asVocab();

                        // start part
                        if (subcmd==OPT_START)
                        {
                            int type=b->get(3).asVocab();

                            Vector xd(3);
                            xd[0]=b->get(4).asDouble();
                            xd[1]=b->get(5).asDouble();
                            xd[2]=b->get(6).asDouble();
    
                            // switch only if it's allowed
                            if (partUsed=="both_arms")
                            {
                                if (type==OPT_RIGHT)
                                    useArm(USE_RIGHT);
                                else
                                    useArm(USE_LEFT);
                            }

                            running=true;
                            bool f=false;

                            // apply systematic offset
                            // due to uncalibrated kinematic
                            Vector offs=dOffs->get(xd);
                            offs[2]=0.0;

                            action->pushAction(xd+offs+*graspDisp,*graspOrien,"open_hand");
                            action->checkActionsDone(f,true);
                            action->latchWrenchOffset();
                            action->enableContactDetection();
                            action->pushWaitState(1.0);
                            action->pushAction(xd+offs,*graspOrien,3.0);
                            action->checkActionsDone(f,true);
                            action->disableContactDetection();

                            kinCalib.init(action,xd,*forceCalibKinThres);

                            if (kinCalib.isRunning())
                                kinCalib.resume();
                            else
                                kinCalib.start();
                        }
                        // stop part
                        else if (subcmd==OPT_STOP)
                        {
                            kinCalib.suspend();

                            // update kinematic offsets map
                            dOffs->insert(kinCalib.get_x0(),kinCalib.get_offset());

                            Vector xd=kinCalib.get_x();
                            xd[2]=kinCalib.get_x0()[2]; // keep the original height
                            grasp(xd);
                            running=false;
                        }
                        // clear part
                        else if (subcmd==OPT_CLEAR)
                        {
                            int type=b->get(3).asVocab();

                            if (type==OPT_RIGHT)
                                dOffsR.clear();
                            else
                                dOffsL.clear();
                        }
                    }

                    break;
                }

                default:
                {
                    cout<<"Error: command not recognized!"<<endl;
                    break;
                }
            }
        }

        return true;
    }

    void setTrackingMode(const bool sw)
    {
        if (partUsed=="both_arms" || partUsed=="left_arm")
            actionL->setTrackingMode(sw);

        if (partUsed=="both_arms" || partUsed=="right_arm")
            actionR->setTrackingMode(sw);

        trackMode=sw;
    }

    bool interruptModule()
    {
        // since a call to checkActionsDone() blocks
        // the execution until it's done, we need to 
        // take control and exit from the waiting state
        action->syncCheckInterrupt(true);        

        cmdPort.interrupt();
        rpcPort.interrupt();

        if (kinCalib.isRunning())
            kinCalib.stop();

        return true;
    }

    virtual bool respond(const Bottle &command, Bottle &reply)
    {
        switch (command.get(0).asVocab())
        {
            case VOCAB4('s','t','a','t'):
            {
                reply.addInt((int)running);
                return true;
            }

            case VOCAB4('t','r','a','c'):
            {
                if (command.size()>1)
                {
                    bool sw=(command.get(1).asVocab()==OPT_ON);
                    setTrackingMode(sw);
                }
            }

            default:
                return RFModule::respond(command,reply);
        }

        return true;
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
    rf.setDefaultContext("icubDemo2010MotorIF/conf");
    rf.setDefaultConfigFile("motorIF.ini");
    rf.setDefault("hand_sequences_file","hand_sequences.ini");    
    rf.setDefault("table_file","table.ini");
    //rf.setDefault("homography_port","/eye2world");
    //rf.setDefault("name","icubDemo2010MotorIF");
    rf.configure("CHRIS_ROOT",argc,argv);

    graspModule mod;

    return mod.runModule(rf);
}



