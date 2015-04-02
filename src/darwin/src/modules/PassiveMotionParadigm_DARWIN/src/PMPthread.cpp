#include "darwin/PMPthread.h"

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace Darwin::pmp;


const string PMPthread::PMP_CMD_LIST[] = { "xr_target", "xl_target", "q_rightArm", "q_leftArm", "qr_initial", "ql_initial", 
										   "Kr_virt",   "Kl_virt",   "A_rightArm", "A_leftArm", "Kr_int",	  "Kl_int",
										   "K_virt_bimanual", "A_bimanual", "T_init", "T_dur", "SlopeRamp", "alpha", "wrist",
										   "pose"
										 };

const string PMPthread::VTGS_CMD_LIST[] = { "target", "weights",
											"T_init1", "T_dur1", "SlopeRamp1", "alpha1",
											"T_init2", "T_dur2", "SlopeRamp2", "alpha2"
										  };

const unsigned int PMPthread::PMP_CMD_SIZE  = 20;
const unsigned int PMPthread::VTGS_CMD_SIZE = 10;

const double PMPthread::INDEX_OS[] = {0.089637, -0.019565, 0.004647}; //0.089637148, -0.019564565, 0.004647362};
//const double INDEX_OS_L[]  = {0.089637, -0.019565, 0.004647};
//const double INDEX_OS_R[]  = {0.089637, -0.019565, -0.004647};

PMPthread::PMPthread(Property *propPMP_right, Property *propPMP_left,   Property *propPMP_bimanual,
					 Property *propPMP_tbg,   Property *propVTGS_right, Property *propVTGS_left,
					 string name, string RPCclient, string RPCserver, string serverName, int period) 
					 : RateThread(period), threadName(name), rpcClientPort(RPCclient), rpcServerPort(RPCserver), rpcServerName(serverName)
{
	this->propPMP_right    = propPMP_right;
	this->propPMP_left     = propPMP_left;
	this->propPMP_bimanual = propPMP_bimanual;
	this->propPMP_tbg      = propPMP_tbg;

	this->propVTGS_right   = propVTGS_right;
	this->propVTGS_left    = propVTGS_left;

	if(!initialize())
		printf("Error\n");

	// Open input and output ports: remote controlboard communication ports ( DARWIN prefix?? )
	string rightName = "/" + threadName + "/right:o";
	string leftName  = "/" + threadName + "/left:o";
	string torsoName = "/" + threadName + "/torso:o";
	string headName  = "/" + threadName + "/head:o";
	if( !rightPort.open(rightName.c_str()) || !leftPort.open(leftName.c_str()) || 
		!torsoPort.open(torsoName.c_str()) || !headPort.open(headName.c_str()) )
	{
		printf("Error: Unable to open output ports");
		//return false;
	}
	
	string rpcName = "/" + threadName + rpcClientPort;
	if( !rpcPort.open(rpcName.c_str()) )
	{
		printf("Error: Unable to open rpc port");
		//return false;
	}

	//encoders.resize(17,0.0);
	T0 = 0;
	TE = 0;

	// By default use hand as the end-effector
	enableIndex = false;
}

bool PMPthread::initialize()
{
	// Initialize PMP and VTGS objects
	readyToStart = false;
	hasTarget = false;

	if(propVTGS_right->isNull() || propVTGS_left->isNull())
	{
		printf("Error: couldn't read property object, closing thread");
		return false;
	}

	//cout << propVTGS_right->toString() << endl;
	VTGS_r = new VirtualTrajectoryGenerator(propVTGS_right);
	VTGS_l = new VirtualTrajectoryGenerator(propVTGS_left);

	// set the maximum number of thread iteration before reading the next target point and reinitializing PMP
	// with a new target. When doing a bimanual task, the iteration number will be equal to the maximum value 
	// between the two (for the one whith tbg time that will exceed duration, Gamma function is zero)
	maxIterNumber_right = (unsigned int)((VTGS_r->tbg1.getT_dur() + VTGS_r->tbg1.getT_init())/VTGS_r->tbg1.getSlopeRamp()+1);
	maxIterNumber_left  = (unsigned int)((VTGS_l->tbg1.getT_dur() + VTGS_l->tbg1.getT_init())/VTGS_l->tbg1.getSlopeRamp()+1);

	// Before making the PMP object run, it is necessary to set the active chain. Default is right
	PMP = new PassiveMotionParadigm(propPMP_right, propPMP_left, propPMP_bimanual, propPMP_tbg);

	return true;
}

bool PMPthread::threadInit()
{
	// check if ports are working
	if( !Network::exists(rightPort.getName().c_str()) || 
		!Network::exists(leftPort.getName().c_str()) || 
		!Network::exists(torsoPort.getName().c_str()) || 
		!Network::exists(headPort.getName().c_str()) )
	{
		printf("Error: Unable to open output ports");
		return false;
	}
	
	if( !Network::exists(rpcPort.getName().c_str()) )
	{
		printf("Error: Unable to open rpc port");
		return false;
	}

	//Network::connect(rightPort.getName().c_str(), "/DevDriver/right:i");
	//Network::connect(leftPort.getName().c_str(), "/DevDriver/left:i");
	//Network::connect(torsoPort.getName().c_str(), "/DevDriver/torso:i");
	//Network::connect(headPort.getName().c_str(), "/DevDriver/head:i");

	// Update VTGS starting point with the EE position given by direct kinematics of
	// PMP joint angles: the are equal to q_initial if this function is called for the
	// first time after initialization from properties, and then will become equal to the
	// last position reached by PMP.
	// PMP end-effector may be also a tool

	Vector pmpPos_r(3);
	Vector pmpPos_l(3);

	pmpPos_r = PMP->get_EEPose("right");
	pmpPos_l = PMP->get_EEPose("left");

	VTGS_r->setStartingPoint(pmpPos_r);
	VTGS_l->setStartingPoint(pmpPos_l);

	return true;
}

void PMPthread::run()
{
	runSem.wait();
	Vector par,tgR,tgL;
	Vector pmpPos_r(3);
	Vector pmpPos_l(3);
	unsigned int iter = getIterations();
	
	Vector Zero(3);
	Zero.zero();

	// check the iteration number: only if both the VTGSs have reached their target, in that case
	// VTGS target is updated with the next critical point in the shape sequence (if present).
	if(iter > maxIterNumber_right && iter > maxIterNumber_left)
	{
		time(&TE);
		double diff = difftime(TE,T0);		
		cout << "Time elapsed: " << diff << endl;

		cout << "Estimated pose: " << endl;
		if (enableIndex)	cout << "(of the tool EE) " << endl;
	
		pmpPos_r = PMP->get_EEPose("right");
		pmpPos_l = PMP->get_EEPose("left");

		cout << pmpPos_r.toString() << endl;
		cout << pmpPos_l.toString() << endl;

		cout << "right" << endl << (PMP->q_rightArm*CTRL_RAD2DEG).toString() << endl;
		cout << "left" << endl << (PMP->q_leftArm*CTRL_RAD2DEG).toString() << endl;

		// check for safety: if x bound (-0.1) is excedeed, reset
		if (PMP->getActiveChain() == "right"    && pmpPos_r(2) < -0.1 || 
			PMP->getActiveChain() == "left"     && pmpPos_l(2) < -0.1 ||
			PMP->getActiveChain() == "bimanual" && (pmpPos_r(2) < -0.1 || pmpPos_l(2) < -0.1) )
		{
			cout << "ATTENTION: risky motion, not executed" << endl;
			// activate end-of-reaching flags:
			if (getActiveChain() == "right" || getActiveChain() == "bimanual")	VTGS_r->hasTarget = false;
			if (getActiveChain() == "left"  || getActiveChain() == "bimanual")	VTGS_l->hasTarget = false;
			
			VTGS_r->numTargetPoints = 0;
			VTGS_l->numTargetPoints = 0;

			// signal to module that the thread is ready to start providing that it receives new target points
			readyToStart = true;
		}

		// else if everything is fine, continue:
		else
		{
			// if in execute mode, send joint angles to motors
			if(state == execute)	
			{
				messagePass(true);
				
			}

			// update initial position in VTGS
			VTGS_r->setStartingPoint(pmpPos_r);
			VTGS_l->setStartingPoint(pmpPos_l);

			// check if PMP has reached all the target points given at initialization
			// if not, continue the trajectory and delete the just-reached point
			// else wait for a new target matrix to be specified
			cout << "Ntg: " << VTGS_r->numTargetPoints << endl;

			// decrement the number of point yet to be reached
			if (VTGS_r->numTargetPoints > 0)	VTGS_r->numTargetPoints --;
			if (VTGS_l->numTargetPoints > 0)	VTGS_l->numTargetPoints --;

			cout << "Ntg: " << VTGS_r->numTargetPoints << endl;

			// if more points are available, continue, else end reaching task
			if( VTGS_r->numTargetPoints > 0 || VTGS_l->numTargetPoints > 0)
			{
				// add a zero vector at the tail, shifting away the last reached point				
				VTGS_r->setTarget(Zero);
				VTGS_l->setTarget(Zero);
				VTGS_r->setWeights(Zero);
				VTGS_l->setWeights(Zero);

				// signal to module that the thread is ready to be restarted to complete the sequence of targets
				hasTarget = true;
				readyToStart = true;
			}
			else
			{
				// activate end-of-reaching flags:
				if (getActiveChain() == "right" || getActiveChain() == "bimanual")	VTGS_r->hasTarget = false;
				if (getActiveChain() == "left"  || getActiveChain() == "bimanual")	VTGS_l->hasTarget = false;	

				// signal to module that the thread is ready to start providing that it receives new target points
				readyToStart = true;
				cout << "ready to start" << endl;
			}
		}

		runSem.post();
	
		// put the event in a waiting state (waiting for the module activation signal)
		sleep.wait();
	}		
	
	// starting of the iteration loop
	else
	{
		if(iter == 1) {time(&T0);}
		// reset all flags
		readyToStart = false;
		VTGS_r->hasTarget = false;
		VTGS_l->hasTarget = false;
		hasTarget = false;

		// do a PMP iteration
		par.push_back(iter);		

		// be sure that no zero vector is passed to PMP
		//if( (getActiveChain() == "right" && VTGS_r->getTarget().getRow(0) == Zero) &&
		//	(getActiveChain() == "left"  && VTGS_l->getTarget().getRow(0) == Zero) &&
		//	(getActiveChain() == "bimanual" && VTGS_r->getTarget().getRow(0) == Zero && VTGS_l->getTarget().getRow(0) == Zero)
		//	)

		if( (getActiveChain() == "right" && isZero((VTGS_r->getTarget()).getRow(0)) ) &&
			(getActiveChain() == "left"  && isZero((VTGS_l->getTarget()).getRow(0)) ) &&
			(getActiveChain() == "bimanual" && isZero((VTGS_r->getTarget()).getRow(0)) && isZero((VTGS_l->getTarget()).getRow(0)) )
			)
		{
			cout << " Target is Zero, stopping!" << endl;
			runSem.post();
			sleep.wait();
		}
		else
		{
			VTGS_r->run(iter,tgR);
			VTGS_l->run(iter,tgL);

			//cout << "tgR: " << tgR.toString() << endl;
			PMP->run(par,&tgR,&tgL);
			
			// if it is not a mental simulation, send commands to motors through ports:
			if(iter % 5 == 0)
			{
				//cout << "right" << endl << (PMP->q_rightArm*CTRL_RAD2DEG).toString() << endl;
				//cout << "left" << endl << (PMP->q_leftArm*CTRL_RAD2DEG).toString() << endl;
				if(state == execute)
					messagePass(true);
			}	
		}

		runSem.post();
	}
}

bool PMPthread::isZero(Vector v)
{
	bool yes = true;
	for (unsigned int i=0; i<v.size(); i++)
	{
		yes = yes && (v(i)==0);
		if (!yes) return true;
	}
	return yes;
}

void PMPthread::interrupt()
{
	cout << "interrupting..." << endl;
	// interrupt ports
	rightPort.interrupt();
	leftPort.interrupt();
	torsoPort.interrupt();
	headPort.interrupt();
	rpcPort.interrupt();
}

void PMPthread::threadRelease()
{
	cout << "thread Release" << endl;

	// close ports
	rightPort.close();
	leftPort.close();
	torsoPort.close();
	headPort.close();
	rpcPort.close();

	cout << "sono ai delete" << endl;
	delete PMP;
	cout << "pmp deleted" << endl;
	delete VTGS_r;
	delete VTGS_l;
	cout << "vtgs deleted" << endl;
	cout << "fine thread release" << endl;
}

bool PMPthread::setPMPparam(Bottle cmdBot)
{
	// mettere tutti i printf in 1 stringa messaggio che viene letta dal modulo ogni volta che respond riceve un false
	// così scrivo tutto da un unico programma, il modulo, ed elimino tutti i printf.
	PMPparam cmd;
	if(cmdBot.size() < 2)
	{
		printf("Error: No value specified\n");
		return false;
	}
	if(!identifyCmd(cmdBot,cmd))
	{
		printf("Error: Parameter not recognized\n");
		return false;
	}
	
	// remove the keyword from bottle, so as to have only parameter value inside
	Bottle bot = cmdBot.tail();
	Bottle test;
	Bottle &list = test.addList();

	bool wasSuspended = isSuspended();

	switch(cmd)
	{
		// PMP parameters
		updateSem.wait();
		if(!isSuspended())	suspend();

		case xr_target:
			if(bot.size() != PMP->x_tgR.size())
			{
				printf("Error: parameter value size mismatch\n");
				if(!wasSuspended) resume();
				updateSem.post();
				return false;
			}
			PMP->x_tgR = Bottle2Vector(bot);
			break;

		case xl_target:
			if(bot.size() != PMP->x_tgL.size())
			{
				printf("Error: parameter value size mismatch\n");
				if(!wasSuspended) resume();
				updateSem.post();
				return false;
			}
			PMP->x_tgL = Bottle2Vector(bot);
			break;

		case q_rightArm:
			if(bot.size() != PMP->q_rightArm.size())
			{
				printf("Error: parameter value size mismatch\n");
				if(!wasSuspended) resume();
				updateSem.post();
				return false;
			}
			PMP->set_q_rightArm(Bottle2Vector(bot));
			break;

		case q_leftArm:
			if(bot.size() != PMP->q_leftArm.size())
			{
				printf("Error: parameter value size mismatch\n");
				if(!wasSuspended) resume();
				updateSem.post();
				return false;
			}
			PMP->set_q_leftArm(Bottle2Vector(bot));
			break;

		case qr_initial:
			if(bot.size() != PMP->q_0R.size())
			{
				printf("Error: parameter value size mismatch\n");
				if(!wasSuspended) resume();
				updateSem.post();
				return false;
			}
			PMP->q_0R = Bottle2Vector(bot);
			break;

		case ql_initial:
			if(bot.size() != PMP->q_0L.size())
			{
				printf("Error: parameter value size mismatch\n");
				if(!wasSuspended) resume();
				updateSem.post();
				return false;
			}
			PMP->q_0L = Bottle2Vector(bot);
			break;

		case Kr_virt:
			if(bot.size() != PMP->K_right.cols())
			{
				printf("Error: parameter value size mismatch\n");
				if(!wasSuspended) resume();
				updateSem.post();
				return false;
			}
			PMP->set_K_right(Bottle2Vector(bot));
			break;

		case Kl_virt:
			if(bot.size() != PMP->K_left.cols())
			{
				printf("Error: parameter value size mismatch\n");
				if(!wasSuspended) resume();
				updateSem.post();
				return false;
			}
			PMP->set_K_left(Bottle2Vector(bot));
			break;

		case A_rightArm:
			if(bot.size() != PMP->A_rightArm.cols())
			{
				printf("Error: parameter value size mismatch\n");
				if(!wasSuspended) resume();
				updateSem.post();
				return false;
			}
			PMP->set_A_rightArm(Bottle2Vector(bot));
			break;

		case A_leftArm:
			if(bot.size() != PMP->A_leftArm.cols())
			{
				printf("Error: parameter value size mismatch\n");
				if(!wasSuspended) resume();
				updateSem.post();
				return false;
			}
			PMP->set_A_leftArm(Bottle2Vector(bot));
			break;

		case Kr_int:
			if(bot.size() != PMP->Kint_right.cols())
			{
				printf("Error: parameter value size mismatch\n");
				if(!wasSuspended) resume();
				updateSem.post();
				return false;
			}
			PMP->set_Kint_right(Bottle2Vector(bot));
			break;

		case Kl_int:
			if(bot.size() != PMP->Kint_left.cols())
			{
				printf("Error: parameter value size mismatch\n");
				if(!wasSuspended) resume();
				updateSem.post();
				return false;
			}
			PMP->set_Kint_left(Bottle2Vector(bot));
			break;

		case K_virt_bimanual:
			if(bot.size() != PMP->K_biman.cols())
			{
				printf("Error: parameter value size mismatch\n");
				if(!wasSuspended) resume();
				updateSem.post();
				return false;
			}
			PMP->set_K_biman(Bottle2Vector(bot));
			break;

		case A_bimanual:
			if(bot.size() != PMP->A_biman.cols())
			{
				printf("Error: parameter value size mismatch\n");
				if(!wasSuspended) resume();
				updateSem.post();
				return false;
			}
			PMP->set_A_biman(Bottle2Vector(bot));
			break;

		// tbg parameters
		case T_init:
			PMP->tbg.setT_init(bot.get(0).asDouble());
			break;

		case T_dur:
			PMP->tbg.setT_dur(bot.get(0).asDouble());
			break;

		case SlopeRamp:
			PMP->tbg.setSlopeRamp(bot.get(0).asDouble());
			break;

		case alpha:
			PMP->tbg.setAlpha(bot.get(0).asDouble());
			break;
		// others
		case wrist:
			if(bot.size() != 2)
			{
				printf("Error: parameter value size mismatch\n");
				if(!wasSuspended) resume();
				updateSem.post();
				return false;
			}
			
			if (bot.get(0).asString() == "right")
			{
				PMP->q_ref_right(7) = bot.get(1).asDouble()*CTRL_DEG2RAD;
				list = Vector2Bottle(PMP->q_ref_right);
				PMP->opt_right->put("q_ref_right",test.get(0));
				cout << PMP->opt_right->toString()<< endl;
				
				if(!wasSuspended) resume();
				updateSem.post();
				return true;
			}
			else if (bot.get(0).asString() == "left")
			{
				PMP->q_ref_left(7) = bot.get(1).asDouble()*CTRL_DEG2RAD;
				list = Vector2Bottle(PMP->q_ref_left);
				PMP->opt_left->put("q_ref_left",test.get(0));
				if(!wasSuspended) resume();
				updateSem.post();
				return true;
			}
			else if (bot.get(0).asString() == "bimanual")
			{
				PMP->q_ref_right(7) = PMP->q_ref_left(7) = bot.get(1).asDouble()*CTRL_DEG2RAD;
				list = Vector2Bottle(PMP->q_ref_right);
				PMP->opt_right->put("q_ref_right",test.get(0));
				list.clear();
				list = Vector2Bottle(PMP->q_ref_left);
				PMP->opt_left->put("q_ref_left",test.get(0));
				if(!wasSuspended) resume();
				updateSem.post();
				return true;
			}
			else
			{
				printf("Error: unknown chain parameter value\n");
				if(!wasSuspended) resume();
				updateSem.post();
				return false;
			}
			
		default:
			printf("idle\n");
			if(!wasSuspended) resume();
			updateSem.post();
			return true;
	}

	if(!wasSuspended) resume();
	updateSem.post();
	return true;
}

bool PMPthread::setVTGSparam(Bottle cmdBot, string side)
{
	VTGSparam cmd;

	if(cmdBot.size() < 2)
	{
		printf("Error: No value specified\n");
		return false;
	}
	if(!identifyCmd(cmdBot,cmd))
	{
		printf("Error: Parameter not recognized\n");
		return false;
	}

	VirtualTrajectoryGenerator * VTGS;

	if (side == "right")	VTGS = VTGS_r;
	if (side == "left")		VTGS = VTGS_l;
	
	// remove the keyword from bottle, so as to have only parameter value inside
	Bottle bot = cmdBot.tail();
	Vector vtemp(9);
	Matrix mtemp(3,3);

	bool wasSuspended = isSuspended();
	string active;
	Vector deltaindex(3);

	switch(cmd)
	{
		// VTGS parameters
		// Before any change, suspend the thread. Probably not effective ...
		// At the end of the switch the thread initial state will be restored
		updateSem.wait();
		if(!isSuspended())	suspend();

		case target:

			// set target as first in matrix
			if(bot.size() == 3)
			{
				mtemp.resize(1,3);
				mtemp.zero();

				Bottle2Matrix(bot,mtemp);
				VTGS->setTarget(mtemp);
				cout << mtemp.toString() << endl;

				updateSem.post();
				break;
			}

			else if(bot.get(0).isInt() && (bot.size()-1)%3 == 0)
			{
				mtemp.resize(bot.get(0).asInt(),3);
				mtemp.zero();
				if (mtemp.rows() <= 3)
				{	
					Bottle2Matrix(bot.tail(),mtemp);
					VTGS->setTarget(mtemp);
					cout << mtemp.toString() << endl;
				}
				else
				{
					printf("Error: parameter value size mismatch\n");
					if(!wasSuspended) resume();
					updateSem.post();
					return false;
				}

				updateSem.post();
				break;
			}

			// set target in tail: (FIFO mode)
/*			else if(bot.size() == 3)
			{
				// new critical point coordinates: if at least one VTGS has a new target, then the thread flag
				// is set to false: tha target saved has not yet been reached.
				// the target should be set only when the thread is not running (is suspended).
					
				VTGS->setTarget(Bottle2Vector(bot));
				VTGS->numTargetPoints++;
				//readyToStart = false;

				updateSem.post();
				break;
			}
*/
			else
			{
				updateSem.post();
				return false;
			}

		case weights:
			if(bot.get(0).isInt() && (bot.size()-1)%3 == 0)
			{
				mtemp.resize(bot.get(0).asInt(),3);
				mtemp.zero();
				if (mtemp.rows() <= 3)
				{	
					Bottle2Matrix(bot.tail(),mtemp);
					VTGS->setWeights(mtemp);
				}
				else
				{
					printf("Error: parameter value size mismatch\n");
					if(!wasSuspended) resume();
					updateSem.post();
					return false;
				}

				updateSem.post();
				break;
			}
			// set one weight vector in tail
			else if(bot.size() == 3)
			{
				VTGS->setWeights(Bottle2Vector(bot));

				updateSem.post();
				break;
			}			
			else if (bot.size() == VTGS->weights.rows()*VTGS->weights.cols())
			{
				Bottle2Matrix(bot,VTGS->weights);

				updateSem.post();
				break;
			}
			else
			{
				printf("Error: parameter value size mismatch\n");
				if(!wasSuspended) resume();
				updateSem.post();
				return false;
			}
			
			break;

		// tbg1 parameters
		case T_init1:
			VTGS->tbg1.setT_init(bot.get(0).asDouble());
			break;

		case T_dur1:
			VTGS->tbg1.setT_dur(bot.get(0).asDouble());
			break;

		case SlopeRamp1:
			VTGS->tbg1.setSlopeRamp(bot.get(0).asDouble());
			updateSem.post();
			break;

		case alpha1:
			VTGS->tbg1.setAlpha(bot.get(0).asDouble());
			break;

		// tbg2 parameters
		case T_init2:
			VTGS->tbg2.setT_init(bot.get(0).asDouble());
			break;

		case T_dur2:
			VTGS->tbg2.setT_dur(bot.get(0).asDouble());
			break;

		case SlopeRamp2:
			VTGS->tbg2.setSlopeRamp(bot.get(0).asDouble());
			break;

		case alpha2:
			VTGS->tbg2.setAlpha(bot.get(0).asDouble());
			break;
	}

	// check if hasTarget Flag can be set to true
	string Achain = getActiveChain();
	if		(Achain == "right" && VTGS_r->hasTarget) hasTarget = true;
	else if (Achain == "left"  && VTGS_l->hasTarget) hasTarget = true;
	else if (VTGS_r->hasTarget && VTGS_l->hasTarget) hasTarget = true;

	// don't change previous thread state
	if(!wasSuspended) resume();
	updateSem.post();
	return true;
}

Bottle PMPthread::getPMPparam(Bottle cmdBot)
{
	PMPparam cmd;
	Bottle reply;

	if(!identifyCmd(cmdBot, cmd))
	{
		reply.addString("Unknown command\n");
	}
	else
	{
		switch(cmd)
		{
			// PMP parameters
			case xr_target:			
				reply.append(Vector2Bottle(PMP->x_tgR));
				break;
	
			case xl_target:
				reply.append(Vector2Bottle(PMP->x_tgL));
				break;

			case q_rightArm:
				reply.append(Vector2Bottle(PMP->get_q_rightArm()));
				break;

			case q_leftArm:
				reply.append(Vector2Bottle(PMP->get_q_rightArm()));
				break;

			case qr_initial:
				reply.append(Vector2Bottle(PMP->q_0R));
				break;

			case ql_initial:
				reply.append(Vector2Bottle(PMP->q_0L));
				break;

			case Kr_virt:
				reply.append(Matrix2Bottle(PMP->get_K_right()));
				break;

			case Kl_virt:
				reply.append(Matrix2Bottle(PMP->get_K_left()));
				break;

			case A_rightArm:
				reply.append(Matrix2Bottle(PMP->get_A_rightArm()));
				break;

			case A_leftArm:
				reply.append(Matrix2Bottle(PMP->get_A_rightArm()));
				break;

			case Kr_int:
				reply.append(Matrix2Bottle(PMP->get_Kint_right()));
				break;

			case Kl_int:
				reply.append(Matrix2Bottle(PMP->get_Kint_left()));
				break;

			case K_virt_bimanual:
				reply.append(Matrix2Bottle(PMP->get_K_biman()));
				break;

			case A_bimanual:
				reply.append(Matrix2Bottle(PMP->get_A_biman()));
				break;

			case wrist:

				if(cmdBot.size() != 2)
				{
					reply.append("Error: parameter value size mismatch");
					break;
				}

				cout << cmdBot.toString() << endl;
				if (cmdBot.get(1).asString() == "right")
				{
					reply.addDouble(PMP->q_ref_right(7)*CTRL_RAD2DEG);
					break;
				}
				else if (cmdBot.get(1).asString() == "left")
				{
					reply.addDouble(PMP->q_ref_left(7)*CTRL_RAD2DEG);
					break;
				}
				else if (cmdBot.get(1).asString() == "bimanual")
				{
					reply.addDouble(PMP->q_ref_right(7)*CTRL_RAD2DEG);
					reply.addDouble(PMP->q_ref_left(7)*CTRL_RAD2DEG);
					break;
				}
				else
				{
					printf("Error: unknown chain parameter value\n");
					reply.append("error");
					break;
				}
	
			// tbg parameters
			case T_init:
				reply.addDouble(PMP->tbg.getT_init());
				break;
	
			case T_dur:
				reply.addDouble(PMP->tbg.getT_dur());
				break;
	
			case SlopeRamp:
				reply.addDouble(PMP->tbg.getSlopeRamp());
				break;
	
			case alpha:
				reply.addDouble(PMP->tbg.getAlpha());
				break;
			case pose:
				if(!updateCurrentAngles(true)) reply.append("Error in communication");
				else
				{
					reply.append( Vector2Bottle( PMP->get_EEPose("right") ) );
					reply.append( Vector2Bottle( PMP->get_EEPose("left") ) );
				}
				break;
		}
	}
	return reply;
}

Bottle PMPthread::getVTGSparam(Bottle cmdBot, string side)
{
	VirtualTrajectoryGenerator * VTGS;

	if (side == "right")	VTGS = VTGS_r;
	if (side == "left")		VTGS = VTGS_l;

	VTGSparam cmd;
	Bottle reply;

	if(!identifyCmd(cmdBot, cmd))
	{
		reply.addString("Unknown command\n");
	}
	else
	{
		switch(cmd)
		{
			// VTGS parameters
			case target:
				reply.append(Matrix2Bottle(VTGS->getTarget()));
				break;
	
			case weights:
				reply.append(Matrix2Bottle(VTGS->getWeights()));
				break;
	
			// tbg1 parameters
			case T_init1:
				reply.addDouble(VTGS->tbg1.getT_init());
				break;
	
			case T_dur1:
				reply.addDouble(VTGS->tbg1.getT_dur());
				break;
	
			case SlopeRamp1:
				reply.addDouble(VTGS->tbg1.getSlopeRamp());
				break;
	
			case alpha1:
				reply.addDouble(VTGS->tbg1.getAlpha());
				break;

			// tbg2 parameters
			case T_init2:
				reply.addDouble(VTGS->tbg2.getT_init());
				break;
	
			case T_dur2:
				reply.addDouble(VTGS->tbg2.getT_dur());
				break;

			case SlopeRamp2:
				reply.addDouble(VTGS->tbg2.getSlopeRamp());
				break;
	
			case alpha2:
				reply.addDouble(VTGS->tbg2.getAlpha());
				break;
		}
	}
	return reply;
}

string PMPthread::getActiveChain()
{
	return PMP->getActiveChain();
}


bool PMPthread::setActiveChain(string side)
{
	return PMP->setActiveChain(side);
}

void PMPthread::setState(PMPstate _state)
{
	state = _state;
}

PMPthread::PMPstate PMPthread::getState()
{
	return state;
}
// Convert a command in a bottle into the correspondent enum value
bool PMPthread::identifyCmd(Bottle cmdBot, PMPparam &cmd)
{
	unsigned int i=0;
	string word = cmdBot.get(0).asString().c_str();
	
	for (unsigned int i=0; i < PMP_CMD_SIZE; i++)
	{
		if (word == PMP_CMD_LIST[i])
		{
			cmd = (PMPparam)i;
			return true;
		}
	}
	return false;
}


bool PMPthread::identifyCmd(Bottle cmdBot, VTGSparam &cmd)
{
	unsigned int i=0;
	string word = cmdBot.get(0).asString().c_str();
	
	for (unsigned int i=0; i < VTGS_CMD_SIZE; i++)
	{
		if (word == VTGS_CMD_LIST[i])
		{
			cmd = (VTGSparam)i;
			return true;
		}
	}

	return false;
}

bool PMPthread::messagePass(bool rpc)
{
	Vector right = PMP->q_rightArm.subVector(3,9)*180.0/M_PI;
	Vector left  = PMP->q_leftArm.subVector(3,9)*180.0/M_PI;

	//cout << left.toString() << endl;
	//cout << "qleft " << PMP->q_leftArm.toString() << endl;
	//cout << "left " << PMP->q_leftArm.subVector(3,9).toString() << endl;
	//double temp = right(0);
	//right(0) = right(2);
	//right(2) = temp;

	Vector torso(3);
	if (PMP->side.compare("right") == 0)
	{
		torso(0) =  PMP->q_rightArm(2)*180.0/M_PI;
		torso(1) =  PMP->q_rightArm(1)*180.0/M_PI;
		torso(2) =  PMP->q_rightArm(0)*180.0/M_PI;
		//torso(0) = -PMP->q_rightArm(2)*180.0/M_PI;
		//torso(1) =  PMP->q_rightArm(1)*180.0/M_PI;
		//torso(2) =  PMP->q_rightArm(0)*180.0/M_PI;
	}
	else
	{
		torso(0) =  PMP->q_leftArm(2)*180.0/M_PI;
		torso(1) =  PMP->q_leftArm(1)*180.0/M_PI;
		torso(2) =  PMP->q_leftArm(0)*180.0/M_PI;
		//torso(0) = -PMP->q_leftArm(2)*180.0/M_PI;
		//torso(1) =  PMP->q_leftArm(1)*180.0/M_PI;
		//torso(2) =  PMP->q_leftArm(0)*180.0/M_PI;
	}
	
	if(rpc)
	{
		return RPCmessagePass(right.data(), left.data(), torso.data());
	}

	//  don't wait until any previous sends are complete (reply)
	rightPort.write(right);
	leftPort.write(left);
	torsoPort.write(torso);

	//cout << "ho scritto sulle porte" << endl;
	//Time::delay(0.1);
	
	return true;
}

bool PMPthread::RPCmessagePass(double* r, double* l, double* t, double* h, int nr, int nl, int nt, int nh)
{
	cout << "RPC mp" << endl;
		Bottle angles, reply;
		angles.clear();
		reply.clear();

		//updateSem.wait();

		angles.addString("move");

		// fill in torso list
		if(t!=NULL)
		{
			angles.addString("torso");
			Bottle &listT = angles.addList();
			for (int i=0;i<nt;i++)
				listT.addDouble(t[i]);
		}
		// fill in right arm list 
		if(r!=NULL)
		{
			angles.addString("right");
			Bottle &listR = angles.addList();
			for (int i=0;i<nr;i++)
				listR.addDouble(r[i]);
		}
		// fill in left arm list 
		if(l!=NULL)
		{
			angles.addString("left");
			Bottle &listL = angles.addList();
			for (int i=0;i<nl;i++)
				listL.addDouble(l[i]);
		}
		// fill in head list
		if(h!=NULL)
		{
			angles.addString("head");
			Bottle &listH = angles.addList();
			for (int i=0;i<nh;i++)
				listH.addDouble(h[i]);	
		}

		//cout << "scrivo!" << endl;
		//cout << angles.toString() << endl;
		rpcPort.write(angles,reply);

		if (reply.get(0).asString()=="Done")
			return true;
		else
		{
			cout << reply.toString() << endl;
			return false;
		}
}

bool PMPthread::updateCurrentAngles(bool updateVTGSstartingPoint)
{
	//check if the connection is established. If not, try to connect to DevDriver/rpc port
	if (!Network::exists(rpcServerPort.c_str()) )
	{
		printf("Error: rpc Server %s is not active\n", rpcServerPort.c_str());
		return false;
	}
	if( !Network::isConnected(rpcPort.getName().c_str(), rpcServerPort.c_str()) )
	{
		if(!Network::connect(rpcPort.getName().c_str(), rpcServerPort.c_str()) )
		{
			printf("Error: unable to connect to rpc Server %s\n", rpcServerPort.c_str());
			return false;
		}
	}
	
	Bottle msg, reply;
	msg.clear();
	reply.clear();

	msg.addString("getEncoders");

	rpcSem.wait();
	rpcPort.write(msg,reply);

	Bottle *right = reply.find("right").asList();
	Bottle *left = reply.find("left").asList();

	if(!right || !left)
	{
		printf("Encoder vectors not all received\n");
		return false;
	}

	if(right->size() == 1 || left->size() == 1)
	{
		printf("Error: Device not all connected\n");
		return false;
	}
	
	Vector R = Bottle2Vector(*right);
	Vector L = Bottle2Vector(*left);
	
	double temp = R(0);
	R(0) = R(2);
	R(2) = temp;
	temp = L(0);
	L(0) = L(2);
	L(2) = temp;

	// update PMP angles values and PMP cartesian EE position (EE might be a tool if any has been connected):
	cout << (R.subVector(0,9)*CTRL_DEG2RAD).toString() << endl;
	cout << (L.subVector(0,9)*CTRL_DEG2RAD).toString() << endl;
	PMP->set_q_rightArm(R.subVector(0,9)*CTRL_DEG2RAD);
	PMP->set_q_leftArm (L.subVector(0,9)*CTRL_DEG2RAD);

	/*
	if (reply.size() != encoders.size())
	{
		printf("Encoders vector size not compatible\n");
		return false;
	}
	
	for(int i= 0; i<encoders.size(); i++)
	{
		encoders(i) = reply.get(i).asDouble();
		if( i<3) // torso
		{
			PMP->q_rightArm(i) = encoders(i)*CTRL_DEG2RAD;
			PMP->q_leftArm(i) = encoders(i)*CTRL_DEG2RAD;
		}
		else if ( i<10 ) // right Arm
			PMP->q_rightArm(i) = encoders(i)*CTRL_DEG2RAD;
		else // left Arm
			PMP->q_leftArm(i-7) = encoders(i)*CTRL_DEG2RAD;
	}

	*/
	rpcSem.post();

	cout << "right: " << PMP->get_q_rightArm().toString() << endl;
	cout << "left: "  << PMP->get_q_leftArm().toString() << endl;

	//update VTGS starting point if flag updateVTGSstartingPoint active
	if(updateVTGSstartingPoint)
	{
		VTGS_r->setStartingPoint(PMP->x_0R);
		VTGS_l->setStartingPoint(PMP->x_0L);

		cout << "VTGS xo = " << PMP->x_0R.toString() << endl;
		cout << "VTGS xo = " << PMP->x_0L.toString() << endl;
	}

	return true;
}


Bottle PMPthread::initIcubUp()
{
	Bottle reply;

	if(state == execute)
	{
		//double arm[7] = {-100.0, 45.0, 0.0, 70.0, 0.0, 0.0, 0.0};
		//double arm[7] = {-80.0, 45.0, 20.0, 70.0, 0.0, 0.0, 0.0};
		//double t[3] = {0.0, 0.0, 0.0};
		//double h[6] = {-32.0, 0.0, 0.0, 0.0, 0.0, 0.0};
		//double h[6] = {-32.0, 0.0, 0.0, 0.0, 0.0, 16.0};

		double arm[7];
		for (int i=0; i<7; i++)
		{
			arm[i] = PMP->q_homeR(i+3);
		}
		double t[3] = {0.0, 0.0, 0.0};
		double h[6] = {-39.0, 0.0, 0.0, 0.0, 0.0, 0.0};

		updateSem.wait();
		cout << "icubUp" << endl;

		// update PMP joint angles
		if (RPCmessagePass(arm, arm, t, h))
		{
			for (int i=0; i<3; i++)
				PMP->q_rightArm(i) = PMP->q_leftArm(i) = t[i]*CTRL_DEG2RAD;
			for (int i=3; i<10; i++)
				PMP->q_rightArm(i) = PMP->q_leftArm(i) = arm[i-3]*CTRL_DEG2RAD;

			// update PMP initial position
			PMP->x_0R = PMP->get_EEPose("right");
			PMP->x_0L = PMP->get_EEPose("left");

			// update VTGS starting position
			VTGS_r->setStartingPoint(PMP->x_0R);
			VTGS_l->setStartingPoint(PMP->x_0L);

			cout << "--> PMP initial position updated" << endl;
			reply.addString("Done");
		}
		updateSem.post();
	}
	else
	{
		double arm[7];
		for (int i=0; i<7; i++)
		{
			arm[i] = PMP->q_homeR(i+3);
		}
		double t[3] = {0.0, 0.0, 0.0};

		for (int i=0; i<3; i++)
				PMP->q_rightArm(i) = PMP->q_leftArm(i) = t[i]*CTRL_DEG2RAD;
		for (int i=3; i<10; i++)
				PMP->q_rightArm(i) = PMP->q_leftArm(i) = arm[i-3]*CTRL_DEG2RAD;

		PMP->Rchain->setAng(PMP->q_rightArm);
		PMP->Lchain->setAng(PMP->q_leftArm);

		// update PMP initial position
		PMP->x_0R = PMP->get_EEPose("right");
		PMP->x_0L = PMP->get_EEPose("left");

		// update VTGS starting position
		VTGS_r->setStartingPoint(PMP->x_0R);
		VTGS_l->setStartingPoint(PMP->x_0L);

		cout << "--> PMP initial position updated" << endl;
		reply.addString("Done");
	}

	updateSem.post();

	return reply;
}


Bottle PMPthread::initHead(Bottle angles)
{
	Bottle reply;
	reply.clear();

	string msg;

	//check if the connection is established. If not, try to connect to DevDriver input ports
	if (!Network::exists(("/" + rpcServerName + "/rpc").c_str()) )
	{
		//printf("Error: device ports not active\n");
		reply.addString("Error: device ports not active");
		return reply;
	}
	if( !Network::isConnected(rpcPort.getName().c_str(), ("/" + rpcServerName + "/rpc").c_str()) )
	{
		if(!Network::connect(rpcPort.getName().c_str(), ("/" + rpcServerName + "/rpc").c_str()) )
		{
			//printf("Error: unable to connect to device port %s\n", ("/" + rpcServerName + "/head:i").c_str());
			msg = "Error: unable to connect to device port /" + rpcServerName + "/rpc";
			reply.addString(msg.c_str());
			return reply;
		}
	}

	//cout << "thread: " << angles.toString() << endl;

	updateSem.wait();
	rpcPort.write(angles,reply);
	//cout << "out of thread!!!!!!!" << endl;

	updateSem.post();
	return reply;
}

Bottle PMPthread::initTorso(Bottle angles)
{
	Bottle reply;
	reply.clear();

	string msg;

	//check if the connection is established. If not, try to connect to DevDriver input ports
	if (!Network::exists(("/" + rpcServerName + "/rpc").c_str()) )
	{
		//printf("Error: device ports not active\n");
		reply.addString("Error: device ports not active");
		return reply;
	}
	if( !Network::isConnected(rpcPort.getName().c_str(), ("/" + rpcServerName + "/rpc").c_str()) )
	{
		if(!Network::connect(rpcPort.getName().c_str(), ("/" + rpcServerName + "/rpc").c_str()) )
		{
			//printf("Error: unable to connect to device port %s\n", ("/" + rpcServerName + "/head:i").c_str());
			msg = "Error: unable to connect to device port /" + rpcServerName + "/rpc";
			reply.addString(msg.c_str());
			return reply;
		}
	}

	double t[3] = {0.0, 0.0, 0.0};

	// default initialization
	if (angles.size() == 1)
	{		
		for (int i=0; i<3; i++)		angles.addDouble(t[i]);
	}
	// else user defined initialization

	updateSem.wait();
	rpcPort.write(angles,reply);

	// update PMP joint angles 
	if (reply.get(0).asString() == "Done")
	{
		PMP->q_rightArm(0) = PMP->q_leftArm(0) = angles.get(0).asDouble()*M_PI/180;
		PMP->q_rightArm(1) = PMP->q_leftArm(1) = angles.get(1).asDouble()*M_PI/180;
		PMP->q_rightArm(2) = PMP->q_leftArm(2) = angles.get(2).asDouble()*M_PI/180;

		PMP->Rchain->setAng(PMP->q_rightArm);
		PMP->Lchain->setAng(PMP->q_leftArm);

		PMP->x_0R = PMP->get_EEPose("right");
		PMP->x_0L = PMP->get_EEPose("left");

		//update VTGS starting position
		VTGS_r->setStartingPoint(PMP->x_0R);
		VTGS_l->setStartingPoint(PMP->x_0L);
	}

	updateSem.post();
	return reply;
}

Bottle PMPthread::initArm(Bottle angles)
{
	Bottle reply;
	reply.clear();

	bool right = angles.get(0).asString() == "initRightArm";
	bool left  = angles.get(0).asString() == "initLeftArm";
	bool done = false;

	string msg;

	double arm[7];
	for (int i=0; i<7; i++)
	{
		arm[i] = PMP->q_homeR(i+3);
	}

	if (angles.size() == 1)
	{	
		for (int i=0; i<7; i++)		angles.addDouble(arm[i]);
	}
	// else: user-defined initialization

	if(state == execute)
	{
		//check if the connection is established. If not, try to connect to DevDriver input ports
		if (!Network::exists(("/" + rpcServerName + "/rpc").c_str()) )
		{
			//printf("Error: device ports not active\n");
			reply.addString("Error: device ports not active");
			return reply;
		}
		if( !Network::isConnected(rpcPort.getName().c_str(), ("/" + rpcServerName + "/rpc").c_str()) )
		{
			if(!Network::connect(rpcPort.getName().c_str(), ("/" + rpcServerName + "/rpc").c_str()) )
			{
				//printf("Error: unable to connect to device port %s\n", ("/" + rpcServerName + "/head:i").c_str());
				msg = "Error: unable to connect to device port /" + rpcServerName + "/rpc";
				reply.addString(msg.c_str());
				return reply;
			}
		}		

		// default initialization
		if (angles.size() == 1)
		{		
			for (int i=0; i<7; i++)		angles.addDouble(arm[i]);
		}
		// else user defined initialization (TODO)

		updateSem.wait();
		rpcPort.write(angles,reply);
		done = reply.get(0).asString() == "Done";
	}

	else	done = true;


	if(done)
	{
		if(right)
		{
			for (int i=0; i<angles.size(); i++)
			{
				PMP->q_rightArm(i+3) = angles.get(i+1).asDouble()*M_PI/180;
			}

			cout << PMP->q_rightArm.toString() << endl;

			//update PMP starting conditions
			PMP->Rchain->setAng(PMP->q_rightArm);
			PMP->x_0R = PMP->get_EEPose("right");

			//update VTGS starting position
			VTGS_r->setStartingPoint(PMP->x_0R);			
		}

		else if (left)
		{
			for (int i=0; i<angles.size(); i++)
			{
				PMP->q_leftArm(i+3)  = angles.get(i+1).asDouble()*M_PI/180;
			}

			cout << angles.toString() << endl;
			cout << PMP->q_leftArm.toString() << endl;
			cout << PMP->x_0L.toString() << endl;

			//update PMP starting conditions
			PMP->Lchain->setAng(PMP->q_leftArm);
			PMP->x_0L = PMP->get_EEPose("left");

			//update VTGS starting position
			VTGS_l->setStartingPoint(PMP->x_0L);
			cout << PMP->x_0L.toString() << endl;
		}
		else
		{
			reply.addString("Error");
			return reply;
		}

	}

	if (state == execute)	updateSem.post();

		/*
			for (int i=0; i<7; i++)
			{
				if (right)
				{
					PMP->q_rightArm(i+3) = angles.get(i).asDouble()*M_PI/180;
				}
				else
				{
					PMP->q_leftArm(i+3)  = angles.get(i).asDouble()*M_PI/180;
				}
			}
			//update PMP starting conditions
			PMP->set_q_rightArm(PMP->q_rightArm);
			PMP->set_q_leftArm(PMP->q_leftArm);
		
			//update VTGS starting position
			VTGS_r->setStartingPoint(PMP->x_0R);
			VTGS_l->setStartingPoint(PMP->x_0L);
		}

		updateSem.post();
		*/
	reply.addString("Done");

	return reply;
}

Vector PMPthread::getIndexInTorsoFrame(string side)
{
	Vector index(4);
	Matrix H(4,4);
	H.eye();
	
	updateCurrentAngles(true);
	
	if (side == "right")	H = PMP->Rchain->getH();
	if (side == "left")		H = PMP->Lchain->getH();
	
	index(0) = INDEX_OS[0];
	index(1) = INDEX_OS[1];
	index(2) = INDEX_OS[2];
	index(3) = 1;

	Vector result = (H*index).subVector(0,2);

	cout << "index value in torso frame: " << result.toString() << endl;
	return result;
}

bool   PMPthread::setIndexAsEndEffector(const string & _side, bool updateVTGSstartingPoint)
{
	if (!isSuspended()) return false;

	Vector index(3);

	index(0) = INDEX_OS[0];
	index(1) = INDEX_OS[1];
	index(2) = INDEX_OS[2];

	PMP->set_Tool(index,_side);
	PMP->use_Tool(_side);

	// update VTGS starting point
	if (updateVTGSstartingPoint)
	{
		if(_side == "right")	VTGS_r->setStartingPoint(PMP->get_ToolPose(_side));
		if(_side == "left")		VTGS_l->setStartingPoint(PMP->get_ToolPose(_side));
	}


	// using HN
	/*
	Matrix Hindex(4,4);
	Hindex.eye();

	Hindex(0,3) = INDEX_OS[0];
	Hindex(1,3) = INDEX_OS[1];
	Hindex(2,3) = INDEX_OS[2];	
	
	if (_side == "right") 
	{
		cout << PMP->Rchain->getHN().toString() << endl;
		PMP->Rchain->setHN(Hindex); 
		cout << PMP->Rchain->getHN().toString() << endl;
	}
	if (_side == "left")  PMP->Lchain->setHN(Hindex);
	*/

	enableIndex = true;

	return true;
}

bool   PMPthread::setPalmAsEndEffector(const string & _side, bool updateVTGSstartingPoint)
{
	if (!isSuspended()) return false;

	PMP->leave_Tool(_side);
	
	// update VTGS starting point
	if (updateVTGSstartingPoint)
	{
		if(_side == "right")	VTGS_r->setStartingPoint(PMP->get_EEPose(_side));
		if(_side == "left")		VTGS_l->setStartingPoint(PMP->get_EEPose(_side));
	}

	//PMP->delete_Tool();

	// Con HN
	/*
	Matrix Hindex(4,4);
	Hindex.eye();

	if (side == "right") 
	{
		cout << PMP->Rchain->getHN().toString() << endl;
		PMP->Rchain->setHN(Hindex);	
		cout << PMP->Rchain->getHN().toString() << endl;
	}
	if (side == "left")	 PMP->Lchain->setHN(Hindex);
	*/
	enableIndex = false;

	return true;
}

string PMPthread::PropertyGroup2String(Bottle group)
{
	Bottle bot;
	bot.clear();

	for (int i = 1; i < group.size(); i++)
	{
		bot.add(group.get(i));
	}

	string s = bot.toString().c_str();
	return s;
}
Vector PMPthread::Bottle2Vector(Bottle Bot)
{
	Vector v(Bot.size());
	for (int i = 0; i < Bot.size(); i++)
	{
		v(i) = Bot.get(i).asDouble();
	}

	return v;
}

void PMPthread::Bottle2Matrix(Bottle Bot, Matrix &m)
{
	int k=0;
	for(int i=0; i<m.rows(); i++)
	{
		for(int j=0; j<m.cols(); j++)
		{
			m(i,j) = Bot.get(k).asDouble();
			k++;
		}
	}
}

Bottle PMPthread::Vector2Bottle(Vector v)
{
	Bottle bot;
	for (unsigned int i=0; i<v.size(); i++)
	{
		bot.addDouble(v(i));
	}
	return bot;
}

Bottle PMPthread::Matrix2Bottle(Matrix m)
{
	Bottle bot;
	for (int i=0; i<m.rows(); i++)
	{
		Bottle &temp = bot.addList();
		for(int j=0; j<m.cols(); j++)
		{
			temp.addDouble(m(i,j));
		}
	}
	return bot;
}
