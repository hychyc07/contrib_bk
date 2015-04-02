#ifndef DEMO1_MODULE_H
#define DEMO1_MODULE_H

#include "Darwin/Demo1Helper.h"

#include <iostream>
#include <string>
#include <map>
#include <vector>

#include <yarp/sig/Vector.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <yarp/os/RateThread.h>

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;

namespace Darwin{
namespace Demo1{

	class Demo1Module: public RateThread
	{

	public:

		Event pointRecorded;
		string activeChain;

		typedef enum {idle, grasping, reaching, recording, lifting} PlannerState;
		volatile PlannerState state;

		Demo1Module(ResourceFinder &rf, int period);
		~Demo1Module(){};
		
		bool threadInit();
		void threadRelease();
		void run();


		/* unstacking methods = recycle */
		// read from vision port and fill in the map (with consistency check)
		// find the topmost point from a list of 3D points
		// check point consistency
		// is stack: read from map and discarded points list

			






		
		/* one hand */
		int Record(bool left = true);
		int userRecordStart(bool left = true);
		int userRecordStop(bool left = true);
		// primitive for reaching a target point and grasping it
//		int Reach2Grasp(const string & _label, const string & _side, const string & _mode = "top");
		int FingerReach(const string & _label, const string & _side, const string & _mode = "top", bool initUp = false);
		int SequenceReach(Bottle labels);

		// close the hand for grasping
		Bottle Grasp();
		bool Release(const string & _side);
		// Lift an object
		int Lift(const string & _side, bool keepContact = false);
		// go a little far away from an object
		int goFar(const string & _side, bool keepContact = false);
		int Relax();
		int Prepare();
		// delete an object from the map
		int Forget(const string & _label);
		// label a new object
		int Label(const string & _label);
		// remember all known object saved in the map
		Bottle Recall();

		int Carry(const string & _side, const string & _tgLabel, bool keepContact);
		int Recycle(const string & _label, const string & _mode = "top");

		/* bimanual */
		int initBimanualReach(const string & _label1, const string & _label2);
		int checkBimanualReachingSuccess(const string & _label1, const string & _label2);
		int BimanualReach(	Matrix tg,
							const string & _mode1 = "top",  const string & _mode2 = "top",
							bool indexR = true,				bool indexL = true,
							bool touchActive = true);
		int BimanualGrasp();
		int BimanualLift(bool keepContact);
		int BimanualRelease();
		int BimanualCarry(const string & _label1, const string & _tgLabel, bool keepContact);
		int KeepAndReach();
		int KeepAndGrasp();

		bool stopMotion();
		// check if the hand is in position for grasping a target object
		// if not, reach the target point before closing the hand for grasping.
		int prepare4Grasping(const string & _label, const string & _mode = "top");
		int checkReachingSuccess(const string & _label);
		int initReaching(bool fingerAsEE, bool touchActive, Bottle fingers, string side);
		bool contactCheck();
		int add2map(const string & label, const double & x, const double & y, const double & z);
		bool startPMP();
		bool setWrist(const string & _mode, const string & _side);
		int initIcubUp();

		bool arePortsConnected();

		void setObjectGeometry(const double & W, const double & H);
		Bottle getObjectGeometry();

	private:

		//typedef enum {record, reach, grasp, release, lift, carry, recycle, forget} DemoCMD;

		// module variables
		string Name;
		string OutPortName;
		Bottle msg;
		Port msgOut;

		Demo1Helper helper;
		double Width;
		double Height;
		static const double SAFETY_THR;
		static const double SAFETY_Z;

		static const double OFFSET_R_TOP [];
		static const double OFFSET_L_TOP [];

		// for communication between module and lower levels modules
		Port toPmp;
		Port toDevDriver;
		Port toGrasper;
		Port toFace;
		BufferedPort <Bottle> fromTouchR;
		BufferedPort <Bottle> fromTouchL;
		BufferedPort <Bottle> fromVision;

		// face commands:
		Bottle sad, happy, think;

		// internal variables:
		//string activeChain;
		OneHandStreamer reach1hand;
		TwoHandStreamer reach2hands;

		// map for storing 3D coordinates of known points in the space
		std::map< string,vector<double> > Memory;
		vector<double> danglingPoint;

		Bottle map2Bottle(map< string,vector<double> > Map);
		string whichHandOnTarget(const Bottle & handsPos, const double & x, const double & y, const double & z);
		int sendPMPstream(Bottle b, Bottle answ);
		Bottle getHandsPose();
		bool hasFinished();
		bool isPMPrunning();
		Vector setGraspingTarget(const string & _label, const string & _side);
	};
} // end namespace Demo1
} // end namespace Darwin

#endif