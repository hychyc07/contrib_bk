// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
* Copyright (C) 2011 Robotcub Consortium, European Commissiom FP6 Project IST-004370
* Authors: Dalia De Santis, Jacopo Zenzeri
* email:   dalia.desantis@iit.it, jacopo.zenzeri@iit.it
*
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*
*/


#ifndef PMPTHREAD_H
#define PMPTHREAD_H

#include <iostream>
#include <string>
#include <math.h>

#include "darwin/PMP.h"
#include "darwin/VTGS.h"

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Event.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Property.h>
#include <time.h>

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

namespace Darwin{
namespace pmp{

// friend class of PMP class
class PMPthread : public RateThread
{
public:
	//class types
	typedef enum {simulate, execute} PMPstate;

	typedef enum {xr_target, xl_target, q_rightArm, q_leftArm, qr_initial, ql_initial, 
				  Kr_virt,   Kl_virt,   A_rightArm, A_leftArm, Kr_int,     Kl_int,
				  K_virt_bimanual, A_bimanual, T_init, T_dur, SlopeRamp, alpha, wrist, pose
				 } PMPparam;

	typedef enum {target, weights,
				  T_init1, T_dur1, SlopeRamp1, alpha1,
				  T_init2, T_dur2, SlopeRamp2, alpha2
				 } VTGSparam;

	// class variables
	bool readyToStart; // true if both VTGS reached their target points.
	bool hasTarget; // true if VTGS has set a new target point.
	bool chainSelected;
	bool enableIndex;
	//Vector encoders;

	// class methods
	PMPthread(Property *propPMP_right, Property *propPMP_left,   Property *propPMP_bimanual,
			  Property *propPMP_tbg,   Property *propVTGS_right, Property *propVTGS_left,
			  string name, string RPCclient, string RPCServer, string rpcServerName, int period);

	bool threadInit();
	void run();
	void threadRelease();
	bool initialize();
	void interrupt();

	Bottle getPMPparam(Bottle cmdBot);
	Bottle getVTGSparam(Bottle cmdBot, string side);
	bool setPMPparam(Bottle cmdBot);
	bool setVTGSparam(Bottle cmdBot, string side);
	string getActiveChain();
	bool setActiveChain(string side);
	void setState(PMPstate _state);
	PMPstate getState();
	bool updateCurrentAngles(bool updateVTGSstartingPoint = false);
	Bottle initIcubUp();
	Bottle initHead(Bottle angles);
	Bottle initArm(Bottle angles);
	Bottle initTorso(Bottle angles);
	bool setIndexAsEndEffector(const string & _side, bool updateVTGSstartingPoint = true);
	bool setPalmAsEndEffector (const string & _side, bool updateVTGSstartingPoint = true);
	
	//Semaphore mutex;
	Event sleep;

private:
	//class constants	
	static const string PMP_CMD_LIST[];
	static const string VTGS_CMD_LIST[];
	static const unsigned int PMP_CMD_SIZE;
	static const unsigned int VTGS_CMD_SIZE;

	static const double INDEX_OS[];
	
	//class variables
	PMPparam par_PMP;
	VTGSparam par_VTGSr, par_VTGSl;

	VirtualTrajectoryGenerator *VTGS_r, *VTGS_l;  // two end-effectors, right arm and left arm
	PassiveMotionParadigm  *PMP;

	Property *propPMP_right,  *propPMP_left, *propPMP_bimanual, *propPMP_tbg;
	Property *propVTGS_right, *propVTGS_left;

	string threadName;
	string rpcServerName;
	string rpcClientPort, rpcServerPort;
	PMPstate state;
	Semaphore runSem, rpcSem, updateSem;

	unsigned int maxIterNumber_right;
	unsigned int maxIterNumber_left;

	time_t T0;
	time_t TE;

	// Ports to send commands to the deviceDriver (remoteCotrolBoard)
	Port rightPort;
	Port leftPort;
	Port torsoPort;
	Port headPort;
	
	Port rpcPort;

	// class methods
	bool messagePass(bool rpc = false);
	bool RPCmessagePass(double* r, double* l, double* t, double* h = NULL, int nr=7, int nl=7, int nt=3, int nh=6);
	
	bool identifyCmd(Bottle cmdBot, PMPparam &cmd);
	bool identifyCmd(Bottle cmdBot, VTGSparam &cmd);
	string PropertyGroup2String(Bottle group);
	Vector Bottle2Vector(Bottle Bot);
	void Bottle2Matrix(Bottle Bot, Matrix &m);
	Bottle Vector2Bottle(Vector v);
	Bottle Matrix2Bottle(Matrix m);
	bool isZero(Vector v);
	
	Vector getIndexInTorsoFrame(string side);
};

}// end namespace pmp
}// end namespace Darwin

#endif
