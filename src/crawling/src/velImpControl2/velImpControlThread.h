// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#ifndef __velImpControlThread__
#define __velImpControlThread__

#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Bottle.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/ConstString.h>
#include <yarp/os/Time.h>
#include <yarp/dev/PreciselyTimed.h>


//class yarp::dev::PolyDriver;

class velImpControlThread: public yarp::os::RateThread
{
private:
    char robotName[255];
    yarp::os:: ConstString limbName;
    yarp::dev::IVelocityControl *ivel;
	yarp::dev::IPreciselyTimed  *itime;
    yarp::dev::IEncoders        *ienc;
    yarp::dev::IImpedanceControl *iimp;
    yarp::dev::IControlMode *ictrl;
    yarp::dev::IPidControl *ipid;
   
    int nJoints;
    yarp::dev::PolyDriver *driver;

    yarp::sig::Vector encoders;
    yarp::sig::Vector encoders_speed;
    yarp::sig::Vector encoders_ref; //the reference pos from ipid
    yarp::sig::Vector Kp;
    yarp::sig::Vector Kd; //derivative term
    
    yarp::sig::Vector targets;
    yarp::sig::Vector ffVelocities;
    yarp::sig::Vector command;

    yarp::sig::Vector error;
    yarp::sig::Vector error_d;

    yarp::sig::Vector maxVel; //added ludo    
    
    bool suspended;
    int first_command;

    int nb_void_loops;

    yarp::os::Semaphore _mutex;

    int control_rate; //in ms
    
    int state;

    yarp::os::BufferedPort<yarp::os::Bottle> command_port; //deprecated
    yarp::os::BufferedPort<yarp::sig::Vector> command_port2; //new
	yarp::os::Port stiffness_port; //new
	yarp::os::Port damping_port; //new
	yarp::os::Port velocity_port; //new

    double time_watch;
    double time_loop;
    int count;
    double stiff;
    
    FILE *currentSpeedFile;
    FILE *targetSpeedFile;
	FILE *stiffFile;
public:
    velImpControlThread(int rate);
    ~velImpControlThread();

    bool init(yarp::dev::PolyDriver *d, yarp::os::ConstString partName,
              yarp::os::ConstString robotName, bool _imepdance_enabled);
     
    //parameters for impedance control
	bool impedance_enabled;
    int njoints;
    yarp::sig::Vector impContr;
    yarp::sig::Vector contrJoints;	//numbers of joint controlled 
    yarp::sig::Vector swingStiff;	//stiffness for swing 
    yarp::sig::Vector stanceStiff;	//stiffness for stance
    yarp::sig::Vector swingDamp;	//damping
    yarp::sig::Vector stanceDamp;	//damping
	yarp::sig::Vector requestedStiff;	//requested stiffness
    yarp::sig::Vector requestedDamp;	//requested damp
	yarp::sig::Vector currStiff;	//current stiffness
    yarp::sig::Vector currDamp;		//current damp             
     

    void halt();
    void go();
    void setRef(int i, double pos);

    void setVel(int i, double vel); //added ludovic to set max velocity
    void setGain(int i, double gain); //to set the Kp gains
    void switchImp(double vel); // to switch between different stiffnesses  

    void run();
    bool threadInit();
    void threadRelease();

    void limitSpeed(yarp::sig::Vector &command);
	void compute_stiffness(yarp::sig::Vector stiff_in,yarp::sig::Vector damp_in,yarp::sig::Vector& stiff_out,yarp::sig::Vector& damp_out);
};

#endif

