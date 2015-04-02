#include "darwin/MessageDriver.h"

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

using namespace Darwin::pmp;

/**-------------------------- GLOBAL FUNCTIONS -----------------------------------------------**/
Vector Darwin::pmp::Bottle2Vector(Bottle Bot)
{
	Vector v(Bot.size());
	for (int i = 0; i < Bot.size(); i++)
	{
		v(i) = Bot.get(i).asDouble();
	}

	return v;
}

Bottle Darwin::pmp::Vector2Bottle(Vector v)
{
	Bottle b;
	for (unsigned int i = 0; i < v.size(); i++)
	{
		b.addDouble(v(i));
	}

	return b;
}


/**-------------------------- EVENTHANDLER SECTION STARTS HERE---------------------------------**/

EventHandler::EventHandler(string _portName, string portType, int period) : RateThread(period)
{
	portName = _portName;
	if (portType == "bottle")	inputBottle = true;
	else						inputBottle = false;

	cout << "event costruttore" << endl;
}

bool EventHandler::threadInit()
{
	cout << "event init" << endl;
	string outputName = portName+":o";
	string stateName = portName+":state";
	string inputName = portName+":i";
	if( inputBottle )	inputB.open(inputName.c_str());
	else				inputV.open(inputName.c_str());
	output.open(outputName.c_str());
	state.open(stateName.c_str());
	what.reset();
	return true;
}

void EventHandler::threadRelease()
{
	if( inputBottle )	
	{
		inputB.interrupt();
		inputB.close();
		
	}
	else
	{
		inputV.interrupt();
		inputV.close();
	}

	output.interrupt();
	output.close();
}

/**-------------------------- TOUCH HANDLER SECTION STARTS HERE---------------------------------**/

TouchHandler::TouchHandler(std::string _portName, std::string portType, int period): EventHandler(_portName, portType, period)
{
	cout << "touch costruttore" << endl;
	indexS.resize(NUM_FINGER_TAXELS,0.0);
	middleS.resize(NUM_FINGER_TAXELS,0.0);
	ringS.resize(NUM_FINGER_TAXELS,0.0);
	pinkyS.resize(NUM_FINGER_TAXELS,0.0);
	thumbS.resize(NUM_FINGER_TAXELS,0.0);

	wait4reading = false;
}

// working with binarized skin signal using threshold = 6
void TouchHandler::run()
{
	info.clear();
	// input port name: /icub/skin/right_hand_comp
	Vector * sensors = inputV.read(false);

	if (!sensors) return;

	runSem.wait();
	indexS = sensors->subVector(0,11);
	middleS = sensors->subVector(12,23);
	ringS  = sensors->subVector(24,35);
	pinkyS = sensors->subVector(36,47);
	thumbS = sensors->subVector(48,59);

	// check if a sensor has been activated on a finger
	for (unsigned int i=0; i<NUM_FINGER_TAXELS; i++)
	{
		if ( indexS(i) != 0)
		{
			if (!wait4reading)	
			{
				activeFinger = index;
				wait4reading = true;
			}

			//cout << "index!!!!" << endl;
			info.addString("index");
			//wait4reading = true;
			runSem.post();
			what.signal();
			state.write(info);
			
			return;
		}

		if ( middleS(i) != 0)
		{
			if (!wait4reading)	
			{
				activeFinger = middle;
				wait4reading = true;
			}
			
			//cout << "middle!!!!" << endl;
			info.addString("middle");
			//wait4reading = true;
			runSem.post();
			what.signal();
			state.write(info);
			
			return;
		}

		if ( ringS(i) != 0)
		{
			if (!wait4reading)	
			{
				activeFinger = ring;
				wait4reading = true;
			}
			
			//cout << "ring!!!!" << endl;
			info.addString("ring");
			//wait4reading = true;
			runSem.post();
			what.signal();
			state.write(info);
			
			return;
		}

		if ( pinkyS(i) != 0)
		{
			if (!wait4reading)	
			{
				activeFinger = pinky;
				wait4reading = true;
			}
			
			//cout << "pinky!!!!" << endl;
			info.addString("pinky");
			//wait4reading = true;
			runSem.post();
			what.signal();
			state.write(info);
			
			return;
		}

		if ( thumbS(i) != 0)
		{
			if (!wait4reading)	
			{
				activeFinger = thumb;
				wait4reading = true;
			}
			
			//cout << "thumb!!!!" << endl;
			info.addString("thumb");
			//wait4reading = true;
			runSem.post();
			what.signal();
			state.write(info);
			
			return;
		}
	}

	Bottle contact;
	contact.addInt(0);
	output.write(contact);

	runSem.post();
	
	// check if a sensor has been activated on a finger (index implemented)
	/*
	Vector index = sensors->subVector(0,11);
	for (unsigned int i=0; i<index.size(); i++)
	{
		if ( (*sensors)(i) != 0)
		{
			what.signal();
			cout << "TOCCATO!!!!" << endl;
			return;
		}
	}
	*/
}

fingers TouchHandler::getActiveFinger()
{
	fingers active = activeFinger;
	clear.wait();
		wait4reading = false;
		//cout << "wait4reading " << std::boolalpha << wait4reading << endl;
	clear.post();

	return active;
}


/** ------------------------- CTRLDEVICE SECTION STARTS HERE -----------------------------------**/

CTRLdevice::CTRLdevice(int period) : RateThread(period)
{
	//enableTouch = false;
	//ask2stop = false;

	// interfaces flags
	hasEncoders = false;
	hasPosition = false;
	hasVelocity = false;
	hasTorque = false;
	hasImpPosition = false;

	hasTouch = false;
	isGrasping = false;
	idle = false;

	// start with fingers all disabled
	disabledFingers[0] = disabledFingers[1] = true;
	disabledFingers[2] = disabledFingers[3] = true;
	ignoreTMB = ignoreIDX = ignoreMDL = ignorePNK = true;
}

bool CTRLdevice::threadInit()
{
	enableTouch = false;
	ask2stop = false;
	idle = false;

	activateFingers();

	return true;
}

void CTRLdevice::threadRelease()
{
}

void CTRLdevice::run()
{
	//cout << "run" << endl;
	if(!enableTouch) return;
	if(idle)		 return;

	// check if touch has been activated: in case stop all controllers motion
	touch->what.wait();
	cout << "Touch activated! "<< endl;

	runSem.wait();

		goIdle();			
		
	runSem.post();
}

void CTRLdevice::goIdle()
{
	//check that no stopping signal was raised:
	if (ask2stop) return;

	cout << "in idle!" << endl;
	cout << "disabled: " << std::boolalpha << disabledFingers[0] << " ";
	cout << disabledFingers[1] << " " << disabledFingers[2] << " " << disabledFingers[3] << endl;

	cout << "ignore: " << std::boolalpha << ignoreTMB << " ";
	cout << ignoreIDX << " " << ignoreMDL << " " << ignorePNK << endl;
	
	// read active finger information
	//cout << "sentito" << endl;
	activeF = touch->getActiveFinger();
	cout << "activeF: " << (int)activeF << endl;

	switch(activeF)
	{
		case thumb:	 
			if(ignoreTMB) { cout << "ignoring thumb" << endl; return;}
			else break;
		case index:  
			if(ignoreIDX){ cout << "ignoring index" << endl; return;}
			else break;
		case middle: 
			if(ignoreMDL) { cout << "ignoring middle" << endl; return;}
			else break;
		case ring:
		case pinky:  
			if(ignorePNK) { cout << "ignoring little" << endl; return;}
			else break;
	}

	// signal to listeners that a stopping touch has been detected
	blockingTouchDetected.signal();
	
	// stop all motions. (Not done during grasping)
	if(!isGrasping)
	{
		for (int j=0; j<this->Nj; j++)		ictrl->setPositionMode(j);
		ipos->stop();
		Bottle & msg = BlockSignaler.prepare();
		msg.clear();
		msg.fromString("blocked");
		cout << msg.toString() << endl;
		BlockSignaler.writeStrict();
	}

	idle = true;

	return;

	// set all controller joints to Torque with 0 reference torque
	// only joints number 0-4! No wrist!
/*	for (int i=0; i<5; i++)
	{
		this->itrq->setRefTorque(i,0.0);
		this->ictrl->setTorqueMode(i);
	}
*/
	
}

void CTRLdevice::activateFinger(fingers finger)
{
	runSem.wait();

		switch(finger)
		{
		case thumb:	 
			if(!disabledFingers[0]) {ignoreTMB = false; cout << "activating thumb" << endl;}
			else {ignoreTMB = true; cout << "ignoring thumb" << endl;}
			break;
		case index:  
			if(!disabledFingers[1]) {ignoreIDX = false; cout << "activating index" << endl;}
			else {ignoreIDX = true; cout << "ignoring index" << endl;}
			break;
		case middle:
			if(!disabledFingers[2]) {ignoreMDL = false; cout << "activating middle" << endl;} 
			else {ignoreMDL = true; cout << "ignoring middle" << endl;}
			break;
		case ring:
		case pinky:
			if(!disabledFingers[3]) {ignorePNK = false; cout << "activating little" << endl;} 
			else {ignorePNK = true; cout << "ignoring little" << endl;}
			break;
		}

	runSem.post();
	return;
}

void CTRLdevice::activateFingers()
{
	runSem.wait();

	if(!disabledFingers[0]) {ignoreTMB = false; cout << "activating thumb" << endl;}
	else {ignoreTMB = true; cout << "ignoring thumb" << endl;}
	if(!disabledFingers[1]) {ignoreIDX = false; cout << "activating index" << endl;}
	else {ignoreIDX = true; cout << "ignoring index" << endl;}
	if(!disabledFingers[2]) {ignoreMDL = false; cout << "activating middle" << endl;}
	else {ignoreMDL = true; cout << "ignoring middle" << endl;}
	if(!disabledFingers[3]) {ignorePNK = false; cout << "activating little" << endl;}
	else {ignorePNK = true; cout << "ignoring little" << endl;}

	runSem.post();
	return;
}

void CTRLdevice::ignoreFinger(fingers finger)
{
	runSem.wait();

		switch(finger)
		{
		case thumb:	 ignoreTMB = true; cout << "ignore thumb" << endl; break;
		case index:  ignoreIDX = true; cout << "ignore index" << endl; break;
		case middle: ignoreMDL = true; cout << "ignore middle" << endl; break;
		case ring:
		case pinky:  ignorePNK = true; cout << "ignore little" << endl; break;
		}

	runSem.post();
	return;
}

void CTRLdevice::disableFinger(fingers finger)
{
	runSem.wait();

		switch(finger)
		{
		case thumb:	 disabledFingers[0] = true; cout << "thumb disabled" << endl; break;
		case index:  disabledFingers[1] = true; cout << "index disabled" << endl; break;
		case middle: disabledFingers[2] = true; cout << "middle disabled" << endl; break;
		case ring:
		case pinky:  disabledFingers[3] = true; cout << "little disabled" << endl; break;
		}

	runSem.post();
	return;
}

void CTRLdevice::enableFinger(fingers finger)
{
	runSem.wait();

		switch(finger)
		{
		case thumb:	 disabledFingers[0] = false; cout << "thumb enabled" << endl; break;
		case index:  disabledFingers[1] = false; cout << "index enabled" << endl; break;
		case middle: disabledFingers[2] = false; cout << "middle enabled" << endl; break;
		case ring:
		case pinky:  disabledFingers[3] = false; cout << "little ensabled" << endl; break;
		}

	runSem.post();
	return;
}

// open a polydriver and by default view encoders and ControlMode interfaces.
bool CTRLdevice::close()
{
	// only for hands: stop touch sensing thread
	ask2stop = true;
	
	// reset all joints to position mode
	if(!driver.isValid()) return true;

	cout << "--> resetting to position mode" << endl;
	for (int i=0; i<Nj; i++)
		ictrl->setPositionMode(i);
	driver.close();
	
	cout << "check if is running" << endl;
	if (this->isRunning())	{cout << "running" << endl; idle = false; this->askToStop();}
	if (hasTouch)
	{
		cout << "has touch" << endl;
		if (touch->isRunning())
		{
			touch->what.signal();//.reset();
			touch->stop();
		}
	}
	
	BlockSignaler.interrupt();
	BlockSignaler.close();
	
	return true;
}

// open device and by default view iEncoders and iControlMode interfaces
// using what is specified in options
bool CTRLdevice::open(yarp::os::Property options, CTRLpart part)
{
	driver.open(options);
	if( !driver.isValid())
	{
		fprintf(stderr, "Device not available. Here are the known devices:\n");
		printf("%s", Drivers::factory().toString().c_str());
		return false;
	}
	bool ok = driver.view(this->ienc);
	ok = ok && driver.view(this->ictrl);
	
	hasEncoders = true;

	if(!ok)
	{
		printf("Problems acquiring interfaces\n");
		return false;
	}

	ienc->getAxes(&Nj);

	cout << "Nj " << Nj << endl;
	this->encoders.resize(Nj);
	this->command.resize(Nj);
	this->command.zero();

	opt = options;

	if (part == rightArm)	
	{
		touch = new TouchHandler("/touchHandler/right_hand","vector",20);
		BlockSignaler.open("/rightHandDevice/state:o");
		ok = ok && touch->start();
	
		cout << "--> Touch thread started" << endl;
		hasTouch = true;
	}
	else if (part == leftArm)
	{
		touch = new TouchHandler("/touchHandler/left_hand","vector",20);
		BlockSignaler.open("/leftHandDevice/state:o");
		ok = ok && touch->start();

		cout << "--> Touch thread started" << endl;
		hasTouch = true;
	}

	return ok;
}

bool CTRLdevice::open(CTRLpart part)
{
	return this->open(opt,part);
}

// view an interface for a particular device
bool CTRLdevice::PositionView(yarp::os::Bottle params)
{
	if (!params.check("refVel") || !params.check("refAcc"))
		{
			cout << "Error: missing ref velocities/accelerations" << endl;
			return false;
		}
	if (params.find("refVel").asList()->size() != Nj || params.find("refAcc").asList()->size() != Nj)
	{
		cout << "Error: wrong ref velocities/acceleration size" << endl;
	}

	bool ok = driver.view(this->ipos);
	if(!ok)
	{
		printf("Problems acquiring interfaces\n");
		return false;
	}

	this->vel.resize(Nj);
	this->acc.resize(Nj);
	vel = Bottle2Vector(*params.find("refVel").asList());
	acc = Bottle2Vector(*params.find("refAcc").asList());

	cout << "refV: " << vel.toString() << endl;
	cout << "refA: " << acc.toString() << endl;
	ipos->setRefAccelerations(acc.data());
	ipos->setRefSpeeds(vel.data());
	
	//ipos->setPositionMode();
	//activeMode = position;
	cout << "position interface ON" << endl;
	hasPosition = true;

	return true;
}
bool CTRLdevice::VelocityView(yarp::os::Bottle params)
{
	if (!params.check("refAcc"))
		{
			cout << "Error: missing ref velocities/accelerations" << endl;
			return false;
		}
	if (params.find("refAcc").asList()->size() != Nj)
	{
		cout << "Error: wrong ref velocities/acceleration size" << endl;
	}

	bool ok = driver.view(this->ivel);
	if(!ok)
	{
		printf("Problems acquiring interfaces\n");
		return false;
	}
	
	acc.resize(Nj);
	acc = Bottle2Vector(*params.find("refAcc").asList());

	ivel->setRefAccelerations(acc.data());
	
	//activeMode = velocity;
	cout << "velocity interface ON" << endl;
	hasVelocity = true;

	return true;
}


bool CTRLdevice::PositionView(yarp::os::Property params)
{
	if (!params.check("refVel") || !params.check("refAcc"))
		{
			cout << "Error: missing ref velocities/accelerations" << endl;
			return false;
		}

	double ref_vel = params.find("refVel").asDouble();
	double ref_acc = params.find("refAcc").asDouble();

	bool ok = driver.view(this->ipos);
	if(!ok)
	{
		printf("Problems acquiring interfaces\n");
		return false;
	}

	this->vel.resize(Nj,ref_vel);
	this->acc.resize(Nj,ref_acc);

	cout << "refV: " << vel.toString() << endl;
	cout << "refA: " << acc.toString() << endl;
	ipos->setRefAccelerations(acc.data());
	ipos->setRefSpeeds(vel.data());
	
	//ipos->setPositionMode();
	//activeMode = position;
	cout << "position interface ON" << endl;
	hasPosition = true;

	return true;
}


bool CTRLdevice::VelocityView(yarp::os::Property params)
{
	if (!params.check("refAcc"))
		{
			cout << "Error: missing ref accelerations" << endl;
			return false;
		}

	double ref_acc = params.find("refAcc").asDouble();

	bool ok = driver.view(this->ivel);
	if(!ok)
	{
		printf("Problems acquiring interfaces\n");
		return false;
	}
	
	acc.resize(Nj,ref_acc);
	ivel->setRefAccelerations(acc.data());
	
	//activeMode = velocity;
	cout << "velocity interface ON" << endl;
	hasVelocity = true;

	return true;
}

bool CTRLdevice::TorqueView(yarp::os::Property params)
{
	double ref_trq = 0;
	if (!params.check("refTorques"))
	{
		cout << "Missing ref Torques" << endl;
		cout << "Using zero-reference torques instead" << endl;		
	}
	else
		ref_trq = params.find("refTorques").asDouble();

	bool ok = driver.view(this->itrq);

	if(!ok)
	{
		printf("Problems acquiring interfaces\n");
		return false;
	}

	this->torques.resize(Nj,ref_trq);
	itrq->setRefTorques(torques.data());	
	
	//activeMode = torque;
	cout << "torque interface ON" << endl;
	hasTorque = true;

	return true;
}

bool CTRLdevice::ImpedancePositionView(yarp::os::Property params)
{
	double stiff, damp;

	if (!params.check("stiffness") || !params.check("damping"))
	{
		cout << "Missing stiffness/damping values" << endl;
		cout << "using default values of 0.111 Nm/deg and 0.014 Nm/(deg/s)" << endl;
		//0.111 is the stiffness coefficient. units:   Nm/deg
		//0.014 is the damping coefficient. units:     Nm/(deg/s)
		stiff = 0.111;
		damp  = 0.014;
	}
	else
	{
		stiff = params.find("stiffness").asDouble();
		damp = params.find("damping").asDouble();
	}

	bool ok = driver.view(this->iimp);
	if(!ok)
	{
		printf("Problems acquiring interfaces\n");
		return false;
	}

	// da cambiare in caso di stiffness diverse per ciascun giunto
	this->stiffness.resize(Nj,stiff);
	this->damping.resize(Nj,damp);

	for (int i=0; i<Nj; i++)
		iimp->setImpedance(i,stiffness(i),damping(i));
	
	//activeMode = impedancePosition;
	cout << "impedance interface ON" << endl;
	hasImpPosition = true;

	return true;
}

// set the active ctrlMode for all/some joints. params should specify the mode and the number of joints
// that will follow that control modality:
// impedancePosition (1 2 3) torque (4 6)

bool CTRLdevice::setCtrlMode(Darwin::pmp::CTRLen mode, yarp::os::Bottle params)
{
	// parsing params content:
	//bool p, t;//, ip, it;
	//p = params.check("position");
	//t = params.check("torque");
	//ip = params.check("impedancePosition");
	//it = params.check("impedanceTorque");

	switch(mode)
	{
	case position:

		if (!hasPosition)	
		{
			cout << "Error: no position interface open" << endl;
			return false;
		}

		for (int i=0; i<params.size(); i++)
		{
			ictrl->setPositionMode(params.get(i).asInt());
			cout << "joint "<< params.get(i).asInt() << " set to position mode" << endl;
		}
		break;

	case velocity:
		if (!hasVelocity)
		{
			cout << "Error: no velocity interface open" << endl;
			return false;
		}

		for (int i=0; i<params.size(); i++)
		{
			ictrl->setVelocityMode(params.get(i).asInt());
			cout << "joint "<< params.get(i).asInt() << " set to velocity mode" << endl;
		}
		break;

	case torque:

		if (!hasTorque)
		{
			cout << "Error: no torque interface open" << endl;
			return false;
		}

		for (int i=0; i<params.size(); i++)
		{
			// check that only arm and forearm pronosupination are requested in torque mode
			if ( params.get(i).asInt() < 5)
			{
				ictrl->setTorqueMode(params.get(i).asInt());
				cout << "joint "<< params.get(i).asInt() << " set to torque mode" << endl;
			}
		}
		break;
	
	case impedancePosition:

		if (!hasImpPosition)	
		{
			cout << "Error: no impedance position interface open" << endl;
			return false;
		}

		for (int i=0; i<params.size(); i++)
		{
			ictrl->setImpedancePositionMode(params.get(i).asInt());
			cout << "joint "<< params.get(i).asInt() << " set to impedance position mode" << endl;
		}
		break;
	}

/*
	// check if there are idle joints:
		int * modes;
		ictrl->g->getControlModes(modes);
*/

	//idle = false;
	//touch->what.reset();

	return true;
}

Bottle CTRLdevice::getCtrlModes()
{
	Bottle ctrl_modes;
	int mode;

	for (int i=0; i<Nj; i++)
	{
		ictrl->getControlMode(i,&mode);
		switch (mode)
		{
			case VOCAB_CM_IDLE:				ctrl_modes.addString("idle");				break;
			case VOCAB_CM_POSITION:			ctrl_modes.addString("position");			break;
			case VOCAB_CM_VELOCITY:			ctrl_modes.addString("velocity");			break;
			case VOCAB_CM_TORQUE:			ctrl_modes.addString("torque");				break;
			case VOCAB_CM_IMPEDANCE_POS:	ctrl_modes.addString("impedancePosition");	break;
			case VOCAB_CM_IMPEDANCE_VEL:	ctrl_modes.addString("impedanceVelocity");	break;
			case VOCAB_CM_OPENLOOP:			ctrl_modes.addString("openLoop");			break;
			default:
			case VOCAB_CM_UNKNOWN:			ctrl_modes.addString("unkown");				break;
		}
	}	

	return ctrl_modes;
}

void CTRLdevice::reset(bool enableFlag)
{
	touch->suspend();
	//cout << "sospeso touch" << endl;

	if (enableFlag)	enableTouch = true;
	else			enableTouch = false;

	cout << "enable touch: " << std::boolalpha << enableTouch << endl;

	touch->what.reset();
	blockingTouchDetected.reset();
	touch->wait4reading = false;
	idle = false;
	//cout << "touch event resettato!" << endl;

	touch->resume();

	//touch->wait4reading = false;
	
}


/** ---------------------------- MESSAGEDEVDRIVER SECTION STARTS HERE -----------------------------------------**/

const int MessageDevDriver::MOTOR_MIN[] = {0,  10, 0,  14,   0,  0,  0,  0,  0};
const int MessageDevDriver::MOTOR_MAX[] = {25, 90, 35, 40, 50, 50, 40, 60, 140};

bool MessageDevDriver::openDevice(CTRLpart part, string Device, string remote, string ModuleName)
{
	Property p;
	p.put("device", Device.c_str());

	switch(part)
	{
	case rightHand:
	case rightArm:
		p.put("local",  ("/" + ModuleName + "/right").c_str());	  //local port names
		p.put("remote", ("/" + remote + "/right_arm").c_str());   //where we connect to
		this->rightDevice.open(p,rightArm);
		if (!rightDevice.driver.isValid()) return false;
		break;
	case leftHand:
	case leftArm:
		p.put("local",  ("/" + ModuleName + "/left").c_str());	  //local port names
		p.put("remote", ("/" + remote + "/left_arm").c_str());   //where we connect to
		this->leftDevice.open(p,leftArm);
		if (!leftDevice.driver.isValid()) return false;
		break;
	case torso:
		p.put("local",  ("/" + ModuleName + "/torso").c_str());	  //local port names
		p.put("remote", ("/" + remote + "/torso").c_str());   //where we connect to
		this->torsoDevice.open(p,torso);
		if (!torsoDevice.driver.isValid()) return false;
		break;
	case head:
		p.put("local",  ("/" + ModuleName + "/head").c_str());	  //local port names
		p.put("remote", ("/" + remote + "/head").c_str());   //where we connect to
		this->headDevice.open(p,head);
		if (!headDevice.driver.isValid()) return false;
		break;
	default:
		return false;
	}
	
	return true;
}


bool MessageDevDriver::PositionView(CTRLpart part, Bottle params)
{	
	switch(part)
	{
	case rightHand:
	case rightArm:
		
		if(rightDevice.hasPosition)
		{
			cout << "iPosition interface already running" << endl;
			return true;
		}
		return this->rightDevice.PositionView(params);

	case leftHand:
	case leftArm:
		if(leftDevice.hasPosition)
		{
			cout << "iPosition interface already running" << endl;
			return true;
		}
		return this->leftDevice.PositionView(params);

	case torso:
		if(torsoDevice.hasPosition)
		{
			cout << "iPosition interface already running" << endl;
			return true;
		}
		return this->torsoDevice.PositionView(params);

	case head:
		if(headDevice.hasPosition)
		{
			cout << "iPosition interface already running" << endl;
			return true;
		}
		return this->headDevice.PositionView(params);

	default:
		cout << "Error: unknown part" << endl;
		return false;
	}

	//return this->view(part,position,params);
}

bool MessageDevDriver::VelocityView(CTRLpart part, Bottle params)
{
	switch(part)
	{
	case rightHand:
	case rightArm:
		if(rightDevice.hasVelocity)
		{
			cout << "iVelocity interface already running" << endl;
			return true;
		}
		return this->rightDevice.VelocityView(params);

	case leftHand:
	case leftArm:
		if(leftDevice.hasVelocity)
		{
			cout << "iVelocity interface already running" << endl;
			return true;
		}
		return this->leftDevice.VelocityView(params);

	case torso:
		cout << "not implemented!" << endl;
		return false;

	case head:
		cout << "not implemented!" << endl;
		return false;

	default:
		cout << "Error: unknown part" << endl;
		return false;
	}

	//return this->view(part,position,params);
}


bool MessageDevDriver::PositionView(CTRLpart part, double refVel, double refAcc)
{
	Property params;
	params.put("refVel",refVel);
	params.put("refAcc",refAcc);
	
	switch(part)
	{
	case rightHand:
	case rightArm:
		
		if(rightDevice.hasPosition)
		{
			cout << "iPosition interface already running" << endl;
			return true;
		}
		return this->rightDevice.PositionView(params);

	case leftHand:
	case leftArm:
		if(leftDevice.hasPosition)
		{
			cout << "iPosition interface already running" << endl;
			return true;
		}
		return this->leftDevice.PositionView(params);

	case torso:
		if(torsoDevice.hasPosition)
		{
			cout << "iPosition interface already running" << endl;
			return true;
		}
		return this->torsoDevice.PositionView(params);

	case head:
		if(headDevice.hasPosition)
		{
			cout << "iPosition interface already running" << endl;
			return true;
		}
		return this->headDevice.PositionView(params);

	default:
		cout << "Error: unknown part" << endl;
		return false;
	}

	//return this->view(part,position,params);
}

bool MessageDevDriver::VelocityView(CTRLpart part, double refAcc)
{
	Property params;
	params.put("refAcc",refAcc);
	
	switch(part)
	{
	case rightHand:
	case rightArm:
		if(rightDevice.hasVelocity)
		{
			cout << "iVelocity interface already running" << endl;
			return true;
		}
		return this->rightDevice.VelocityView(params);

	case leftHand:
	case leftArm:
		if(leftDevice.hasVelocity)
		{
			cout << "iVelocity interface already running" << endl;
			return true;
		}
		return this->leftDevice.VelocityView(params);

	case torso:
		cout << "not implemented!" << endl;
		return false;

	case head:
		cout << "not implemented!" << endl;
		return false;

	default:
		cout << "Error: unknown part" << endl;
		return false;
	}

	//return this->view(part,position,params);
}

bool MessageDevDriver::TorqueView(CTRLpart part, double refTorques)
{
	Property params;
	params.put("refTorques",refTorques);
	
	switch(part)
	{
	case rightHand:
	case rightArm:
		if(rightDevice.hasTorque)
		{
			cout << "iTorque interface already running" << endl;
			return true;
		}
		return this->rightDevice.TorqueView(params);

	case leftHand:
	case leftArm:
		if(leftDevice.hasTorque)
		{
			cout << "iTorque interface already running" << endl;
			return true;
		}
		return this->leftDevice.TorqueView(params);

	case torso:
		if(torsoDevice.hasTorque)
		{
			cout << "iTorque interface already running" << endl;
			return true;
		}
		return this->torsoDevice.TorqueView(params);

	case head:
		cout << "not implemented!" << endl;
		return false;

	default:
		cout << "Error: unknown part" << endl;
		return false;
	}
}

bool MessageDevDriver::ImpedancePositionView(CTRLpart part, double stiffness, double damping)
{
	Property params;
	params.put("stiffness",stiffness);
	params.put("damping",damping);
	
	switch(part)
	{
	case rightHand:
	case rightArm:
		if(rightDevice.hasImpPosition)
		{
			cout << "iImpedancePosition interface already running" << endl;
			return true;
		}
		return this->rightDevice.ImpedancePositionView(params);

	case leftHand:
	case leftArm:
		if(leftDevice.hasImpPosition)
		{
			cout << "iImpedancePosition interface already running" << endl;
			return true;
		}
		return this->leftDevice.ImpedancePositionView(params);

	case torso:
	case head:
		cout << "Impedance position is disabled for torso/head parts" << endl;
		return false;

	//case torso:
	//	return this->torsoDevice.ImpedancePositionView(params);

	//case head:
	//	return this->headDevice.ImpedancePositionView(params);

	default:
		cout << "Error: unknown part" << endl;
		return false;
	}
}


Vector MessageDevDriver::getEncoders(CTRLpart part)
{
	// send encoders of active parts:
	Vector hand;
	Vector null(1);
	null(0) = 0.0;

	switch(part)
	{
	case rightHand:
		if(rightDevice.driver.isValid())
		{
			rightDevice.ienc->getEncoders(rightDevice.encoders.data());
			cout << "valori letti" << endl;
			cout << "rightHand" << endl;
			hand = (rightDevice.encoders).subVector(7,rightDevice.encoders.size()-1);
			cout << hand.toString() << endl;
			return hand;
		}
		else
		{
			cout << "Attention: Right device not active" << endl;
			return null;
		}

	case rightArm:
		if(rightDevice.driver.isValid())
		{
			rightDevice.ienc->getEncoders(rightDevice.encoders.data());
			cout << "valori letti" << endl;
			cout << "rightArm" << endl << rightDevice.encoders.toString() << endl;
			return rightDevice.encoders;
		}
		else
		{
			cout << "Attention: Right device not active" << endl;
			return null;
		}

	case leftHand:
		if(leftDevice.driver.isValid())
		{
			leftDevice.ienc->getEncoders(leftDevice.encoders.data());
			cout << "valori letti" << endl;
			cout << "leftHand" << endl;
			hand =  (leftDevice.encoders).subVector(7,leftDevice.encoders.size()-1);
			cout << hand.toString() << endl;
			return hand;
		}
		else
		{
			cout << "Attention: Left device not active" << endl;
			return null;
		}

	case leftArm:
		if(leftDevice.driver.isValid())
		{
			leftDevice.ienc->getEncoders(leftDevice.encoders.data());
			cout << "valori letti" << endl;
			cout << "leftArm" << endl << leftDevice.encoders.toString() << endl;
			return leftDevice.encoders;
		}
		else
		{
			cout << "Attention: Left device not active" << endl;
			return null;
		}

	case torso:
		//cout << "torso" << endl;
		if(torsoDevice.driver.isValid())
		{
			//cout << "valid" << endl;
			torsoDevice.ienc->getEncoders(torsoDevice.encoders.data());
			return torsoDevice.encoders;
		}
		else
		{
			cout << "Attention: Torso device not active" << endl;
			return null;
		}

	case head:
		if(headDevice.driver.isValid())
		{
			headDevice.ienc->getEncoders(headDevice.encoders.data());
			return headDevice.encoders;
		}
		else
		{
			cout << "Attention: Head device not active" << endl;
			return null;
		}
	}

	return null;
}

bool MessageDevDriver::PositionMove(CTRLpart part, Vector angles)
{
	cout << "PositionMove:" << endl;
	switch(part)
	{
	case rightHand:
	case rightArm:
		cout << "right arm" << endl;
		if (rightDevice.idle)						{cout << "idle" << endl;return false;}
		if (angles.size() != rightDevice.Nj)		{cout << "false" << endl;return false;}
		if (!rightDevice.hasPosition)				{cout << "interface not available" << endl; return false;}
		cout << "angles: " << endl << angles.toString() << endl;
		//return true;
		return rightDevice.ipos->positionMove(angles.data());

	case leftHand:
	case leftArm:
		cout << "left arm" << endl;
		if (leftDevice.idle)						{cout << "idle" << endl;return false;}
		if (angles.size() != leftDevice.Nj)			return false;
		if (!leftDevice.hasPosition)				{cout << "interface not available" << endl; return false;}
		cout << "angles: " << endl << angles.toString() << endl;
		//return true;
		return leftDevice.ipos->positionMove(angles.data());

	case torso:
		cout << "torso" << endl;
		if (torsoDevice.idle)						{cout << "idle" << endl;return false;}
		if (angles.size() != torsoDevice.Nj)		return false;
		if (!torsoDevice.hasPosition)				{cout << "interface not available" << endl; return false;}
		cout << "angles: " << endl << angles.toString() << endl;
		//return true;
		return torsoDevice.ipos->positionMove(angles.data());

	case head:
		cout << "head" << endl;
		if (headDevice.idle)						{cout << "idle" << endl;return false;}
		if (angles.size() != headDevice.Nj)			return false;
		if (!headDevice.hasPosition)				{cout << "interface not available" << endl; return false;}
		cout << "angles: " << endl << angles.toString() << endl;
		//return true;
		return headDevice.ipos->positionMove(angles.data());

	default:
		cout << "Error: unknown part" << endl;
		return false;
	}

}

bool MessageDevDriver::VelocityMove(CTRLpart part, Vector vels)
{
	cout << "VelocityMove:" << endl;
	switch(part)
	{
	case rightHand:
	case rightArm:
		cout << "right arm" << endl;
		if (rightDevice.idle)					{cout << "idle" << endl;return false;}
		if (vels.size() != rightDevice.Nj)		{cout << "false" << endl;return false;}
		if (!rightDevice.hasVelocity)			{cout << "interface not available" << endl; return false;}
		cout << "velocities: " << endl << vels.toString() << endl;
		return rightDevice.ivel->velocityMove(vels.data());

	case leftHand:
	case leftArm:
		cout << "left arm" << endl;
		if (leftDevice.idle)					{cout << "idle" << endl;return false;}
		if (vels.size() != leftDevice.Nj)		return false;
		if (!leftDevice.hasVelocity)			{cout << "interface not available" << endl; return false;}
		cout << "velocities: " << endl << vels.toString() << endl;
		return leftDevice.ivel->velocityMove(vels.data());

	case torso:
		cout << "not implemented" << endl;
		return false;

	case head:
		cout << "not implemented" << endl;
		return false;

	default:
		cout << "Error: unknown part" << endl;
		return false;
	}

}


bool MessageDevDriver::threadInit()
{
	EndOfMotion = false;
	return true;
}

void MessageDevDriver::run()
{
	cout << "msgDriver run" << endl;
	int iter = 0;
	EndOfMotion = false;

	// attenzione: chiamare funzione ignore(dito) per dirlo al touch: non deve scriverlo come active!!!!
	runSem.wait();
	cout << "nel runsem run" << endl;
	switch(activeHand)
	{
	case rightHand:
		//rightDevice.reset(true);
		initActiveFingers(rightDevice,EndOfMotion);
		if (EndOfMotion) 
		{
			cout << "End of motion!" << endl;
			rightDevice.isGrasping = false;
			rightDevice.activateFingers(); 
			return;
		}
		break;

	case leftHand:
		//leftDevice.reset(true);
		initActiveFingers(leftDevice,EndOfMotion);
		if (EndOfMotion)
		{
			cout << "End of motion!" << endl;
			leftDevice.isGrasping = false;
			leftDevice.activateFingers();
			return;
		}
		break;
	}
	runSem.post();

	while(!isStopping() || EndOfMotion)
	{
		cout << iter << endl;
		if (iter == 100) 
		{
			for (unsigned int i=0; i<GraspSpeeds->size(); i++)
				(*GraspSpeeds)(i) = 0;
		}
		iter ++;

		// check if there is at least a speed != 0
		unsigned int zeros = 0;
		for(unsigned int i=0; i<GraspTargetAngles->size(); i++)
		{
			if ((*GraspSpeeds)(i) != 0) 	break;
			else							zeros ++;
		}
		
		// if all joints have reached zero velocity, then exit
		if (zeros == GraspTargetAngles->size())		
		{
			EndOfMotion = true; 
			cout << "--> end of motion!" << endl;
			if(activeHand == rightHand)		
			{
				rightDevice.isGrasping = false;
				for (unsigned int i=7; i<GraspSpeeds->size()+7; i++) 
					rightDevice.ictrl->setPositionMode(i);
				rightDevice.activateFingers();
			}
			else if(activeHand == leftHand)
			{
				leftDevice.isGrasping  = false;
				for (unsigned int i=7; i<GraspSpeeds->size()+7; i++) 
					leftDevice.ictrl->setPositionMode(i);
				leftDevice.activateFingers();
			}
			return;
		}
	
		// move the hand
		switch (activeHand)
		{
		case rightHand:
			if (rightDevice.idle) 
			{
				cout << "rightDev idle, stopping motion..." << endl;
				cout << "active finger was: " << rightDevice.activeF << endl;
				
				updateActiveFingers(rightDevice,EndOfMotion);
				if (EndOfMotion) 
				{
					cout << "End of motion!" << endl;
					rightDevice.activateFingers(); 
					return;
				}
			}

			rightDevice.ienc->getEncoders(rightDevice.encoders.data());
			cout << rightDevice.encoders.toString() << endl;

			for(int i=0; i<9; i++)
			{
				// check if final position has been reached:
				//cout << rightDevice.encoders(i+7) << " " << (*GraspTargetAngles)(i) << endl;

				if     ((*GraspSpeeds)(i)>0 && rightDevice.encoders(i+7) >= (*GraspTargetAngles)(i))	(*GraspSpeeds)(i) = 0;
				else if((*GraspSpeeds)(i)<0 && rightDevice.encoders(i+7) <= (*GraspTargetAngles)(i))	(*GraspSpeeds)(i) = 0;
		
				if     (rightDevice.encoders(i+7) <= MOTOR_MIN[i])	(*GraspSpeeds)(i) = ((*GraspSpeeds)(i)<0) ? 0 : (*GraspSpeeds)(i);
				else if(rightDevice.encoders(i+7) >= MOTOR_MAX[i])	(*GraspSpeeds)(i) = ((*GraspSpeeds)(i)>0) ? 0 : (*GraspSpeeds)(i);

				//cout << "moving joint " << i+7 << " with velocity " << (*GraspSpeeds)(i) << endl;
				if ((*GraspSpeeds)(i) != 0 )	rightDevice.ivel->velocityMove(i+7,(*GraspSpeeds)(i));
			}

			cout << (*GraspSpeeds).toString() << endl;
			break;

		case leftHand:
			if (leftDevice.idle) 
			{
				cout << "leftDev idle, stopping motion..." << endl;
				cout << "active finger was: " << leftDevice.activeF << endl;
				
				updateActiveFingers(leftDevice,EndOfMotion);
				if (EndOfMotion) 
				{
					cout << "End of motion!" << endl; 
					leftDevice.activateFingers();
					return;
				}
				//return;
			}

			leftDevice.ienc->getEncoders(leftDevice.encoders.data());

			for(int i=0; i<9; i++)
			{
				// check if final position has been reached:
				//if     (leftDevice.encoders(i+7) == (*GraspTargetAngles)(i))		(*GraspSpeeds)(i) = 0;
				//if     ((*GraspSpeeds)(i)>0 && (rightDevice.encoders(i+7) >= (*GraspTargetAngles)(i)) )		{cout << "oltre" << endl;*(GraspSpeeds)(i) = 0;}
				//else if((*GraspSpeeds)(i)<0 && rightDevice.encoders(i+7) <= (*GraspTargetAngles)(i))		(*GraspSpeeds)(i) = 0;

				if ((*GraspSpeeds)(i) != 0)
				{
					if     ((*GraspSpeeds)(i)>0 && leftDevice.encoders(i+7) >= (*GraspTargetAngles)(i))		(*GraspSpeeds)(i) = 0;
					else if((*GraspSpeeds)(i)<0 && leftDevice.encoders(i+7) <= (*GraspTargetAngles)(i))		(*GraspSpeeds)(i) = 0;

						
					if     (leftDevice.encoders(i+7) <= MOTOR_MIN[i])		(*GraspSpeeds)(i) = ((*GraspSpeeds)(i)<0) ? 0 : (*GraspSpeeds)(i);
					else if(leftDevice.encoders(i+7) >= MOTOR_MAX[i])		(*GraspSpeeds)(i) = ((*GraspSpeeds)(i)>0) ? 0 : (*GraspSpeeds)(i);
	
					//cout << MOTOR_MIN[i] << " " << MOTOR_MAX[i] << " " << (*GraspSpeeds)(i) << endl;
					if ((*GraspSpeeds)(i) != 0 )	leftDevice.ivel->velocityMove(i+7,(*GraspSpeeds)(i));
				}
			}
			break;
		}

		Time::delay(0.1);
	}

	cout << "end of run" << endl;
}

void MessageDevDriver::threadRelease()
{
	delete GraspSpeeds;
	delete GraspTargetAngles;
	delete activeFingers;
}

void MessageDevDriver::initActiveFingers(CTRLdevice & device, bool & isMotionFinished)
{
	// ignore touch signals from fingers that are not moving
	// speeds: 0 = all; 1,2,3 = thumb; 4,5 = index; 6,7 = middle; 8 = ring & pinky;
	device.isGrasping = true;
	if(!activeFingers[0])	device.ignoreFinger(thumb);
	if(!activeFingers[1])	device.ignoreFinger(index);
	if(!activeFingers[2])	device.ignoreFinger(middle);
	if(!activeFingers[3])	device.ignoreFinger(pinky);

	// if at least a finger has nonzero velocity, reset the device thread:
	if(	activeFingers[0] || activeFingers[1] || activeFingers[2] || activeFingers[3] )
	{
		device.reset(true);
		isMotionFinished = false;
	}
	else 
	{
		device.isGrasping = false;
		isMotionFinished = true;
	}

	return;
}

void MessageDevDriver::updateActiveFingers(CTRLdevice & device, bool & isMotionFinished)
{
	// per ogni dito segnato come true controlla se corrisponde all'active finger
	// se sì, azzera la velocità del dito corrispondente e il flag diventa false
	// se tutti sono a false allora isMotionFinished diventa true.

	// speeds: 0 = all; 1,2,3 = thumb; 4,5 = index; 6,7 = middle; 8 = ring & pinky;

	cout << "in active finger" << endl;

	switch(device.activeF)
	{
	case thumb:
		(*GraspSpeeds)(1) = 0;
		(*GraspSpeeds)(2) = 0;
		(*GraspSpeeds)(3) = 0;
		activeFingers[0] = false;
		device.ignoreFinger(thumb);
		device.ictrl->setPositionMode(8);
		device.ictrl->setPositionMode(9);
		device.ictrl->setPositionMode(10);
		cout << "thumb was active, velocity set to zero!" << endl;
		break;
	case index:
		(*GraspSpeeds)(4) = 0;
		(*GraspSpeeds)(5) = 0;
		activeFingers[1] = false;
		device.ignoreFinger(index);
		device.ictrl->setPositionMode(11);
		device.ictrl->setPositionMode(12);
		cout << "index was active, velocity set to zero!" << endl;
		break;
	case middle:
		(*GraspSpeeds)(6) = 0;
		(*GraspSpeeds)(7) = 0;
		activeFingers[2] = false;
		device.ignoreFinger(middle);
		device.ictrl->setPositionMode(13);
		device.ictrl->setPositionMode(14);
		cout << "middle was active, velocity set to zero!" << endl;
		break;
	case ring:
	case pinky:
		(*GraspSpeeds)(8) = 0;
		activeFingers[3] = false;
		device.ignoreFinger(pinky);
		device.ictrl->setPositionMode(15);
		cout << "little was active, velocity set to zero!" << endl;
		break;
	}

	// if there is istill a finger that has to complete the motion, then reset the device thread:
	if(	activeFingers[0] || activeFingers[1] || activeFingers[2] || activeFingers[3] )
	{
		device.reset(true);
		isMotionFinished = false;
	}
	else 
	{
		device.isGrasping = false;
		graspSucceeded = true;
		isMotionFinished = true;
	}
}

void MessageDevDriver::sensitiveGrasp (Vector FinalAngles, Vector speeds, CTRLpart part)
{
	cout << "sensitive grasp" << endl;
	cout << speeds.size() << endl;
	Vector nonNullJoints;
	bool idx,mdl,pnk,tmb;
	idx = mdl = pnk = tmb = false;

	unsigned int j;
	for(unsigned int i=0; i<speeds.size(); i++)
	{
		if (speeds(i) > VELOCITY_MAX)  speeds(i) = VELOCITY_MAX;
		if (speeds(i) < VELOCITY_MIN)  speeds(i) = VELOCITY_MIN;

		if(speeds(i)!=0)
		{
			j = i+7;
			nonNullJoints.push_back(j);

			if (j==8 || j==9 || j==10) tmb = true;
			if (j==11|| j==12) idx = true;
			if (j==13|| j==14) mdl = true;
			if (j==15) pnk = true;
		}
	}

	cout << "max min velocity checked" << endl;
	if (nonNullJoints.size() == 0)
	{
		cout << "Velocities all zero" << endl;
		return;
	}


	// start the grasping thread

	if(!isRunning()) 
	{
		GraspSpeeds = new Vector(7);
		GraspTargetAngles = new Vector(7);
		activeFingers = new bool[4];
		activeFingers[0] = tmb;
		activeFingers[1] = idx;
		activeFingers[2] = mdl;
		activeFingers[3] = pnk;

		cout << "bool vector is: " << std::boolalpha << activeFingers[0] <<  " " << activeFingers[1] << " ";
		cout << activeFingers[2] <<  " " << activeFingers[3] << endl;

		GraspSpeeds->zero();
		GraspTargetAngles->zero();
		*GraspTargetAngles = FinalAngles;
		*GraspSpeeds = speeds;

		cout << "--> starting grasping thread" << endl;

		activeHand = part;
		graspSucceeded = false;

		this->start();
	}
	else
	{
		*GraspTargetAngles = FinalAngles;
		*GraspSpeeds = speeds;
		activeFingers[0] = tmb;
		activeFingers[1] = idx;
		activeFingers[2] = mdl;
		activeFingers[3] = pnk;

		activeHand = part;

		cout << "bool vector is: " << std::boolalpha << activeFingers[0] <<  " " << activeFingers[1] << " ";
		cout << activeFingers[2] <<  " " << activeFingers[3] << endl;

		graspSucceeded = false;
		this->run();
	}
}

string MessageDevDriver::CTRLen2string(CTRLen ctrl)
{
	switch(ctrl)
	{
	case position:
		return "position";
	case velocity:
		return "velocity";
	case torque:
		return "torque";
	default:
		return "error";
	}
}


bool MessageDevDriver::setJointsCtrlMode(CTRLpart part, CTRLen mode, Vector joints)
{
	bool done;
	Bottle par;

	if (CTRLen2string(mode) == "error")	return false;

	cout << CTRLen2string(mode) << endl;

	par.addString(CTRLen2string(mode).c_str());
	Bottle &j = par.addList();
	for (unsigned int i=0; i<joints.size(); i++)
		j.addDouble(joints(i));

	cout << "joints: " << j.toString() << endl;

	switch(part)
	{
	case rightHand:
	case rightArm:
		done = rightDevice.setCtrlMode(mode, j);
		break;

	case leftHand:
	case leftArm:
		done = leftDevice.setCtrlMode(mode, j);
		break;

	case torso:
		done = torsoDevice.setCtrlMode(mode, j);
		break;

	case head:
		done = headDevice.setCtrlMode(mode, j);
		break;

	default:
		cout << "Error: unknown part" << endl;
		return false;
	}

	if (!done) cout << "Error in setting joint control mode" << endl;
	return done;
}

Bottle MessageDevDriver::getJointsCtrlMode(CTRLpart part)
{
	Bottle error;
	error.addString("Device not available");

	switch(part)
	{
	case rightHand:
	case rightArm:
		if (rightDevice.driver.isValid())
			return this->rightDevice.getCtrlModes();
		else
			return error;

	case leftHand:
	case leftArm:
		if (leftDevice.driver.isValid())
			return this->leftDevice.getCtrlModes();
		else
			return error;

	case torso:
		if (torsoDevice.driver.isValid())
			return this->torsoDevice.getCtrlModes();
		else
			return error;

	case head:
		if (headDevice.driver.isValid())
			return this->headDevice.getCtrlModes();
		else
			return error;
	}

	Bottle b("Error");
	return b;
}


void MessageDevDriver::setActiveFingers(string side, Bottle fingersList, bool enable)
{
	cout << "in set active fingers" << endl;
	if (side == "right")
	{
		if (fingersList.get(0).asString() == "all")
		{
			if (enable)
			{
				cout << "enable all" << endl;
				rightDevice.enableFinger(thumb);
				rightDevice.enableFinger(index);
				rightDevice.enableFinger(middle);
				rightDevice.enableFinger(pinky);
			}
			else
			{
				cout << "disable all" << endl;
				rightDevice.disableFinger(thumb);
				rightDevice.disableFinger(index);
				rightDevice.disableFinger(middle);
				rightDevice.disableFinger(pinky);
			}
		}
		else
		{
			if (enable)
			{
				rightDevice.disableFinger(thumb);
				rightDevice.disableFinger(index);
				rightDevice.disableFinger(middle);
				rightDevice.disableFinger(pinky);

				for (int i=0; i<fingersList.size(); i++)
					rightDevice.enableFinger(string2fingers(fingersList.get(i).asString().c_str()));
			}
			else
			{
				for (int i=0; i<fingersList.size(); i++)
					rightDevice.disableFinger(string2fingers(fingersList.get(i).asString().c_str()));
			}
		}
		rightDevice.activateFingers();
	}
	else
	{
		if (fingersList.get(0).asString() == "all")
		{
			if (enable)
			{
				leftDevice.enableFinger(thumb);
				leftDevice.enableFinger(index);
				leftDevice.enableFinger(middle);
				leftDevice.enableFinger(pinky);
			}
			else
			{
				leftDevice.disableFinger(thumb);
				leftDevice.disableFinger(index);
				leftDevice.disableFinger(middle);
				leftDevice.disableFinger(pinky);
			}
		}
		else
		{
			if (enable)
			{
				leftDevice.enableFinger(thumb);
				leftDevice.enableFinger(index);
				leftDevice.enableFinger(middle);
				leftDevice.enableFinger(pinky);
				for (int i=0; i<fingersList.size(); i++)
					leftDevice.enableFinger(string2fingers(fingersList.get(i).asString().c_str()));
			}
			else
			{
				for (int i=0; i<fingersList.size(); i++)
					leftDevice.disableFinger(string2fingers(fingersList.get(i).asString().c_str()));
			}
		}
		leftDevice.activateFingers();
	}
	
	return;
}

bool MessageDevDriver::enableTouch(string side)
{
	if (side == "right") 
	{
		if(rightDevice.driver.isValid())
		{
			// check if device thread was running. if not, start it
			if(!rightDevice.isRunning())	rightDevice.start();
			rightDevice.reset(true);
			cout << "Right hand Touch sensing enabled" << endl;
			return true;
		}
		else
		{
			cout << "Error: device not available" << endl;
			return false;
		}
	}
	if (side == "left")
	{
		if(leftDevice.driver.isValid())
		{
			// check if device thread was running. if not, start it
			if(!leftDevice.isRunning())	leftDevice.start();
			//{bool ok = leftDevice.start();cout << "now is running: " <<std::boolalpha << ok << endl;}

			leftDevice.reset(true);
			cout << "Left hand Touch sensing enabled" << endl;
			return true;
		}
		else
		{
			cout << "Error: device not available" << endl;
			return false;
		}
	}	
	return false;
}

bool MessageDevDriver::disableTouch(string side)
{
	if (side == "right")
	{
		if(rightDevice.driver.isValid())
		{
			// check if device thread was running. if not, start it
			rightDevice.reset();
			cout << "Right hand Touch sensing disabled" << endl;
			return true;
		}
		else
		{
			cout << "Error: device not available" << endl;
			return false;
		}
	}

	if (side == "left")
	{
		if(leftDevice.driver.isValid())
		{
			leftDevice.reset();		
			cout << "Left hand Touch sensing disabled" << endl;
			return true;
		}
		else
		{
			cout << "Error: device not available" << endl;
			return false;
		}
	}

	return false;
	
}



bool MessageDevDriver::InitRecording(CTRLpart part, fingers activeF)
{
	// enable touch recordings from the selected arm
	// enable only index event detection
	CTRLdevice * dev;

	if (part == rightArm)		dev = &rightDevice;
	else if (part == leftArm)	dev = &leftDevice;
	else	{cout << "part not available" << endl; return false;}

	// wait for the thread to finish its execution to update parameters
	cout << "waiting device" << endl;
	dev->updateSem.wait();
		cout << "my turn" << endl;

		// activate selected finger
		switch(activeF)
		{
		case thumb:  if(dev->disabledFingers[0]) {cout << "Error: thumb is disabled" << endl; dev->updateSem.post();return false;} break;
		case index:  if(dev->disabledFingers[1]) {cout << "Error: index is disabled" << endl; dev->updateSem.post();return false;} break;
		case middle: if(dev->disabledFingers[2]) {cout << "Error: middle is disabled" << endl; dev->updateSem.post();return false;} break;
		case ring:
		case pinky:  if(dev->disabledFingers[3]) {cout << "Error: little is disabled" << endl; dev->updateSem.post();return false;} break;
		}

		dev->ignoreFinger(thumb);
		dev->ignoreFinger(index);
		dev->ignoreFinger(middle);
		dev->ignoreFinger(pinky);
		dev->activateFinger(activeF);		

	// set active part: rightArm/leftArm. 
	// NB: Grasping is setting this value to rightHand/leftHand, so that no interaction is possible
	// to be sure put EndOfMotion flag to true, to sto all grasping actions
		EndOfMotion = true;
	
		bool ok;
		if (part == rightArm)		{ok = enableTouch("right"); activeHand = rightArm;}
		else						{ok = enableTouch("left");  activeHand = leftArm;}

		if (!ok) 
		{
			cout << "touch not available" << endl; 
			dev->updateSem.post(); 
			return false;
		}

		// set the corresponding arm and torso in torque mode
		cout << "setting torque mode" << endl;
		Bottle tjoints;
		Bottle ajoints;
		for(int i=0; i<5; i++)
		{
			ajoints.addInt(i);
		}
		tjoints.addInt(2);
		ok = ok && dev->setCtrlMode(torque,ajoints);
		ok = ok && torsoDevice.setCtrlMode(torque,tjoints);

	dev->updateSem.post();

	return ok;
}

void MessageDevDriver::LaunchRecording()
{
	// create joints bottles to be used in setting position mode
	Bottle tjoints;
	Bottle ajoints;
	for(int i=0; i<5; i++)
		ajoints.addInt(i);
	tjoints.addInt(2);

	// activate recording mode: wait for the touch event to be detected
	cout << "Launching recording" << endl;
	isRecording = true;
	switch(activeHand)
	{
	case rightArm: 
		cout << "rightArm waiting" << endl;
		rightDevice.blockingTouchDetected.wait();
		rightDevice.setCtrlMode(position,ajoints);
		torsoDevice.setCtrlMode(position,tjoints);
		break;
	case leftArm:
		cout << "leftArm waiting" << endl;
		leftDevice.blockingTouchDetected.wait();
		leftDevice.setCtrlMode(position,ajoints);
		torsoDevice.setCtrlMode(position,tjoints);
		break;
	}

	// read from encoders:
	recordedEncoders = Vector2Bottle(getEncoders(torso));

	if (activeHand == rightArm)
		recordedEncoders.append( Vector2Bottle(getEncoders(rightArm).subVector(0,6)) );
	else
		recordedEncoders.append( Vector2Bottle(getEncoders(leftArm).subVector(0,6)) );

	cout << "Encoders: " << endl;
	cout << recordedEncoders.toString() << endl;

	// signal that encoders have been saved in recordedEncoders
	recordedPosition.signal();
	isRecording = false;
}

bool MessageDevDriver::StopRecording()
{
	// check if isRecording is true:
	//if(isRecording)
	cout << "try to stop recording" << endl;

	switch(activeHand)
	{
	case rightArm: 
		rightDevice.blockingTouchDetected.signal();
		break;
	case leftArm:
		leftDevice.blockingTouchDetected.signal();
		break;
	}

	return true;
	//recordedPosition.signal();
	//return isRecording;
}

CTRLpart MessageDevDriver::string2enum(string part)
{
	CTRLpart p;
	if (part == "rightArm")		return p = rightArm;
	if (part == "rightHand")	return p = rightHand;
	if (part == "leftArm")		return p = leftArm;
	if (part == "leftHand")		return p = leftHand;
	if (part == "torso")		return p = torso;
	if (part == "head")			return p = head;

	return p = null;
}

fingers MessageDevDriver::string2fingers(string finger)
{
	fingers p;
	if (finger == "thumb")  return p = thumb;
	if (finger == "index")  return p = index;
	if (finger == "middle") return p = middle;
	if (finger == "ring")   return p = ring;
	if (finger == "pinky")  return p = pinky;

	return p = index;
}

bool MessageDevDriver::checkLimits(CTRLpart part, Vector command)
{
	// check if angles given are in the ROM of the joints 
	// from iKin library:
	cout << "controllo limiti" << endl;

	switch (part)
	{
	case rightArm:
		if(command(0) < -95.5 || command(0) > 5){cout << "R0" << endl;return false;}
		if(command(0) <   0 || command(0) > 160.8){cout << "R1" << endl;return false;}
		if(command(0) < -37 || command(0) > 90)	{cout << "R2" << endl;return false;}
		if(command(0) < 5.5 || command(0) > 106){cout << "R3" << endl;return false;}
		if(command(0) < -90 || command(0) > 90)	{cout << "R4" << endl;return false;}
		if(command(0) < -90 || command(0) > 0)	{cout << "R5" << endl;return false;}
		if(command(0) < -20 || command(0) > 40)	{cout << "R6" << endl;return false;}
		break;
	case leftArm:
		if(command(0) < -95.5 || command(0) > 5){cout << "L0" << endl;return false;}
		if(command(1) <   0 || command(1) > 160.8){cout << "L1" << endl;return false;}
		if(command(2) < -37 || command(2) > 90)	{cout << "L2" << endl;return false;}
		if(command(3) < 5.5 || command(3) > 106){cout << "L3" << endl;return false;}
		if(command(4) < -90 || command(4) > 90)	{cout << "L4" << endl;return false;}
		if(command(5) < -90 || command(5) > 0)	{cout << "L5" << endl;return false;}
		if(command(6) < -20 || command(6) > 40)	{cout << "L6" << endl;return false;}
		break;
	case torso:
		if(command(0) < -22 || command(0) > 84)	{cout << "T0" << endl;return false;}
		if(command(1) < -39 || command(1) > 39)	{cout << "T1" << endl;return false;}
		if(command(2) < -59 || command(2) > 59)	{cout << "T2" << endl;return false;}
		break;
	case head:
		if(command(0) < -40 || command(0) > 30)	{cout << "H0" << endl;return false;}
		if(command(1) < -70 || command(1) > 60)	{cout << "H1" << endl;return false;}
		if(command(2) < -55 || command(2) > 55)	{cout << "H2" << endl;return false;}
		if(command(3) < -35 || command(3) > 15)	{cout << "H3" << endl;return false;}
		if(command(4) < -50 || command(4) > 50)	{cout << "H4" << endl;return false;}
		if(command(5) < 0   || command(5) > 90)	{cout << "H5" << endl;return false;}
		break;
	}

	return true;
}
	
bool MessageDevDriver::hasFinished(bool & rd, bool & ld, bool & td, bool & hd)
{	
	rd = ld = td = hd = true;
	if (rightDevice.driver.isValid())
	{
		rightDevice.ipos->checkMotionDone(&rd);
		rd = rd && !rightDevice.isGrasping;
	}
	if (leftDevice.driver.isValid())
	{
		leftDevice.ipos->checkMotionDone(&ld);
		ld = ld && !leftDevice.isGrasping;
	}
	if (torsoDevice.driver.isValid())	torsoDevice.ipos->checkMotionDone(&td);
	if (headDevice.driver.isValid())	headDevice.ipos->checkMotionDone(&hd);

	cout << "has finished is: " << std::boolalpha << (rd && ld && td && hd) << endl;

	return rd && ld && td && hd;
}


bool MessageDevDriver::Release()
	{
		if(isRunning())	this->stop();

		if (rightDevice.driver.isValid())
		{
			rightDevice.close();
			rightDevice.stop();
		}

		if (leftDevice.driver.isValid())
		{
			leftDevice.close();
			leftDevice.stop();
		}

		if (torsoDevice.driver.isValid())
		{
			torsoDevice.close();
		}

		if (headDevice.driver.isValid())
		{
			headDevice.close();
		}
		printf("\nModule successfully closed\n");
		return true;
	};
