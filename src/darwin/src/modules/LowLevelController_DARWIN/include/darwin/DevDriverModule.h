#ifndef DEV_DRIVER_MODULE_H
#define DEV_DRIVER_MODULE_H

#include "darwin/MessageDriver.h"

#include <iostream>
#include <string>

#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Semaphore.h>
#include <iCub/iKin/iKinFwd.h>

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;

namespace Darwin{
namespace pmp{

	const double INDEX_OS_L[]  = {0.089637, -0.019565, 0.004647};
	const double INDEX_OS_R[]  = {0.089637, -0.019565, -0.004647};
	const double MIDDLE_OS_L[] = {0.091726,  0.0028695, 0.0075};
	const double MIDDLE_OS_R[] = {0.091726,  0.0028695, -0.0075};
	const double RING_OS_L[]   = {0.086401,  0.019767, 0.0040089};
	const double RING_OS_R[]   = {0.086401,  0.019767, -0.0040089};
	const double PINKY_OS_L[]  = {0.06816,  -0.038735, 0.0076349};
	const double PINKY_OS_R[]  = {0.06816,  -0.038735, -0.0076349}; //error

	class DevDriverModule:public RFModule
	{
		//RpcServer Encoders;

	public:

		typedef enum {rightDev, leftDev, torsoDev, headDev} devList;
		typedef enum { ACK, getEncoders, getControlModes, compliant, stiff, enable, disable,
					   grasp, safeGrasp, move, initHead, initRightArm, initLeftArm, initTorso,
					   record, checkMotionFinished, checkGraspFinished, stop, quit
					 } DEVCommands;
		
		DevDriverModule();
		~DevDriverModule();

		bool configure(ResourceFinder &rf);
		double getPeriod();
		bool updateModule();
		bool interruptModule();
		bool close();
		bool respond(const Bottle &command, Bottle &reply);

		bool openPorts();
		void streamEncoders();
		bool checkMotion(CTRLpart part, Vector &actualAngles);

	private:
		
		MessageDevDriver msg;
		ResourceFinder rf;

		static const string CMD_LIST[];
		static const string CMD_DESC[];
		static const unsigned int CMD_SIZE;

		static const string MODULE_NAME;
		static const string DEVICE_NAME;
		static const string PORT_NAME;
		static const double REF_ACC;
		static const double REF_VEL;
		static const double FINGERS_VEL[];
		static const int	MOTOR_MIN[];
		static const int	MOTOR_MAX[];
		static const int	VELOCITY_MAX = 70;			// max velocity used for moving the fingers
		static const int	VELOCITY_MIN = -70;			// min velocity used for moving the fingers
		static const int	VELOCITY_TOUGH = 50;		// velocity used when touch is detected during the tough grasp
		static const int	VELOCITY_SOFT = 0;			// velocity used when touch is detected during the soft grasp
		static const int	VELOCITY_COMPLIANT = -50;	// velocity used when touch is detected during the compliant grasp

		string moduleName;
		string rpcPortName;
		Port rpcPort;
		Port encStream;

		Semaphore encSem;

		Bottle Bright, Bleft, Btorso, Bhead;

		double ref_acc, ref_vel;
		double maxError;

		Bottle getArmsChainEncoders();
		Bottle GetEncoders(string part);
		Bottle GetEncoders(CTRLpart part);
		bool setCompliant(CTRLpart part);
		bool setStiff(CTRLpart part);
		Bottle Grasp(Bottle angles, string part);
		Bottle SafeGrasp(Bottle FinalAngles, Vector speed, string part);
		Bottle Record3dPosition(CTRLpart part, fingers activeF);
		bool identifyCmd(Bottle cmdBot, DEVCommands &cmd);
		
		//bool initDevice(devList device, string mode);

	};
} // end namespace pmp
} // end namespace Darwin

#endif