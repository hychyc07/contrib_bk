#ifndef MESSAGE_DRIVER_H
#define MESSAGE_DRIVER_H

#include <iostream>
#include <string>

#include <yarp/os/all.h>
#include <yarp/os/Thread.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Event.h>
#include <yarp/os/BufferedPort.h>

#include <yarp/sig/all.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

namespace Darwin{
namespace pmp{

#define NUM_FINGER_TAXELS 12
	
	typedef enum {position, velocity, torque, impedancePosition, ctrlMode} CTRLen;
	typedef enum{rightArm, leftArm, rightHand, leftHand, torso, head, null} CTRLpart;
	typedef enum{thumb, index, middle, ring, pinky} fingers;

	Vector Bottle2Vector(Bottle Bot);	
	Bottle Vector2Bottle(Vector v);

	class EventHandler : public RateThread
	{
	public:
		// variables for event handling
		Event what;
		BufferedPort<Bottle> inputB;
		BufferedPort<Vector> inputV;
		Port output;
		Port state;
		string portName;
		bool inputBottle;

		// functions for event handling
		EventHandler(string _portName, string portType, int period);
		~EventHandler(){};
		
		bool threadInit();
		void threadRelease();

		virtual void run()=0;
	};

	class TouchHandler : public EventHandler
	{
	public:

		TouchHandler(string _portName, string portType, int period);
		~TouchHandler(){};
		fingers activeFinger;
		bool wait4reading;

		void run();
		fingers getActiveFinger();

	private:

		Vector indexS;
		Vector middleS;
		Vector ringS;
		Vector pinkyS;
		Vector thumbS;

		Semaphore clear;
		Semaphore runSem;
		Bottle info;
	};


	class CTRLdevice : public RateThread
	{
	public:

		PolyDriver driver;

		IEncoders			*ienc;
		IPositionControl	*ipos;
		IVelocityControl	*ivel;
		ITorqueControl		*itrq;
		IControlMode		*ictrl;
		IImpedanceControl   *iimp;

		Vector encoders;
		Vector command;
		Vector vel;
		Vector acc;
		Vector torques;
		Vector stiffness;
		Vector damping;

		bool hasEncoders;
		bool hasPosition;
		bool hasVelocity;
		bool hasTorque;
		bool hasImpPosition;

		int  Nj;
		bool idle;
		bool enableTouch;
		bool ask2stop;
		bool isGrasping;
		fingers activeF;

		// currently used for position recordings
		Event blockingTouchDetected;

		CTRLen activeMode;
		Property opt;

		CTRLdevice(int period = 20);
		~CTRLdevice(){};

		bool open(Property options,CTRLpart part);
		bool open(CTRLpart part); // open using values stored in opt
		// view a choosen controller interface for the current device
		bool PositionView(Bottle params);
		bool VelocityView(Bottle params);
		bool PositionView(Property params);
		bool VelocityView(Property params);
		bool TorqueView(Property params);
		bool ImpedancePositionView(Property params);
		// set a control mode for the current device (specify type of control and joint lists)
		bool setCtrlMode(CTRLen mode, Bottle params);
		Bottle getCtrlModes();
		bool close();

		void reset(bool enableFlag = false);
		void run();
		bool threadInit();
		void threadRelease();
	
		void goIdle();
		void disableFinger(fingers finger);
		void enableFinger(fingers finger);
		//bool isDisabled(fingers finger)
		void ignoreFinger(fingers finger);		
		void activateFinger(fingers finger);
		void activateFingers();
		void sensitiveGrasp(Vector FinalAngles, Vector speeds);

		Semaphore runSem;
		Semaphore updateSem;

		bool disabledFingers[4];

	private:
	
		TouchHandler * touch;
		bool hasTouch;
		BufferedPort <Bottle> BlockSignaler;
		//Port BlockSignaler;
		
		//Semaphore runSem;
		bool ignoreIDX, ignoreMDL, ignorePNK, ignoreTMB;
		
	};

	class MessageDevDriver : public Thread
	{
	public:
		
		CTRLdevice rightDevice,leftDevice,torsoDevice,headDevice;

		static const int	MOTOR_MIN[];
		static const int	MOTOR_MAX[];
		static const int	VELOCITY_MAX = 70;			// max velocity used for moving the fingers
		static const int	VELOCITY_MIN = -70;			// min velocity used for moving the fingers

		Event recordedPosition;
		Bottle recordedEncoders;

		// state flags
		bool graspSucceeded;

		//bool commandPass();
		bool checkLimits(CTRLpart part, Vector command);
		bool hasFinished(bool & rd, bool & ld, bool & td, bool & hd);
		bool Release();		
	
		bool openDevice				(CTRLpart part, string Device, string remote, string ModuleName);

		bool PositionView			(CTRLpart part, Bottle params);
		bool VelocityView			(CTRLpart part, Bottle params);
		bool PositionView			(CTRLpart part, double refVel, double refAcc);
		bool VelocityView			(CTRLpart part, double refAcc);
		bool TorqueView				(CTRLpart part, double refTorques);
		bool ImpedancePositionView  (CTRLpart part, double stiffness, double damping);

		// creare una funzione Move che muove secondo il modo impostato nel device!
		bool PositionMove	(CTRLpart part, Vector angles);
		bool VelocityMove	(CTRLpart part, Vector vels);
		Vector getEncoders	(CTRLpart part);
		void sensitiveGrasp (Vector FinalAngles, Vector speeds, CTRLpart part);

		// set control mode for some joints only of a specified body part
		bool setJointsCtrlMode(CTRLpart part, CTRLen mode, Vector joints);
		Bottle getJointsCtrlMode(CTRLpart part);

		// manage touch event detection
		bool enableTouch(string side);
		bool disableTouch(string side);
		void setActiveFingers(string side, Bottle fingersList, bool enable);

		// manage position recording (triggered by index touching)
		bool InitRecording(CTRLpart part, fingers activeF);
		void LaunchRecording();
		bool StopRecording();

		// service methods
		string CTRLen2string(CTRLen ctrl);
		CTRLpart string2enum(string part);
		fingers string2fingers(string finger);

		MessageDevDriver(){isRecording = false;};
		~MessageDevDriver(){};

	private:
		
		// grasping thread:
		CTRLpart activeHand;
		bool * activeFingers;
		Vector * GraspSpeeds;
		Vector * GraspTargetAngles;
		bool EndOfMotion;
		Semaphore runSem;

		// recording position:
		bool isRecording;

		bool threadInit();
		void threadRelease();
		void run();

		void initActiveFingers   (CTRLdevice & device, bool & isMotionFinished);
		void updateActiveFingers (CTRLdevice & device, bool & isMotionFinished);

		bool view(CTRLpart part, CTRLen mode, Property params);

	};

} // end namespare pmp
} // end namespace Darwin

#endif
