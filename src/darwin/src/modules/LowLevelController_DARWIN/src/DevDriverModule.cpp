#include "darwin/DevDriverModule.h"
#include <yarp/math/Math.h>

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

using namespace iCub::iKin;

using namespace Darwin::pmp;

const string DevDriverModule::CMD_LIST[] = {"ACK", "getEncoders", "getControlModes", "compliant", "stiff", "enable", "disable",
											"grasp", "safeGrasp", "move", "initHead", "initRightArm", "initLeftArm", "initTorso",
											"record","checkMotionFinished","checkGraspFinished", "stop", "quit"};
const string DevDriverModule::CMD_DESC[] = { "Send an acnowledgment to test the connection",
											 "Get all active chains (rightArm-torso/leftArm-torso) encoders values",
											 "Get current control modes of the specified part (right/left/torso/head)",
											 "Set in torque mode the specified joints of a part(right/left/torso): syntax is:"
											 "<cmd> <part> (<joints list>)",
											 "Set in position mode the specified joints of a part(right/left/torso): syntax is:"
											 "<cmd> <part> (<joints list>)",
											 "Enable fingertips touch detection on one side (right/left). Syntax is:"
											 "<cmd> touch <side> <fingers_list>"
											 "Argument <finger_list> can be 'all'",
											 "Disable fingertips touch detection on one side (right/left). Syntax is:"
											 "<cmd> touch <side> <fingers_list>"
											 "Argument <finger_list> can be 'all'",
											 "Send joints angles to move the fingers in position mode. Syntax is:"
											 "<cmd> (<angles list>) <side>",
											 "Send joints angles to move the fingers in velocity mode using touch to sense"
											 "when the object has been grasped. The final joints angles might not refect the"
											 "actual ones. Syntax is:"
											 "<cmd> (<final angles list>) (<reference velocities list>) <side>",
											 "Move the desired part/s in position mode. Parts are right/left/torso/head. Syntax is:"
											 "<cmd> <part1> (<angles list1>) <part2> (<angles list2>) ...",
											 "Set head initial position. Joints that are not specified are set to zero. Syntax is:"
											 "<cmd> <joints values list>",
											 "Set right arm initial position. Joints that are not specified are set to zero. Syntax is:"
											 "<cmd> <joints values list>",
											 "Set left arm initial position. Joints that are not specified are set to zero. Syntax is:"
											 "<cmd> <joints values list>",
											 "Set torso initial position. Joints that are not specified are set to zero. Syntax is:"
											 "<cmd> <joints values list>",
											 "Get a 3D location respect to root reference frame given the index location. "
											 "The robot can be freely moved by the user until touch is detected on a finger. "
											 "Syntax is: <cmd> <string> <finger>, where string can be right/left/stop "
											 "and finger thumb/index/middle/pinky. If no finger is specified index is selected.",
											 "Returns true if no motion is ongoing",
											 "Returns true if grasping was succesfully performed",
											 "Stop all ongoing motion",
											 "Quit the module"
											};
const unsigned int DevDriverModule::CMD_SIZE = 19;

const string DevDriverModule::MODULE_NAME = "DevDriver";
const string DevDriverModule::DEVICE_NAME = "remote_controlboard";
const string DevDriverModule::PORT_NAME   = "/rpc";
const double DevDriverModule::REF_ACC = 40.0;
const double DevDriverModule::REF_VEL = 20.0;
const double DevDriverModule::FINGERS_VEL[] = {2.0, 2.0, 2.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};

DevDriverModule::DevDriverModule()
{
}

DevDriverModule::~DevDriverModule()
{
}

bool DevDriverModule::configure(ResourceFinder &rf)
{
	this->rf = rf;
	if(!rf.check("remote"))
	{
		printf("Error: no remote device specified in the config file. Closing...\n");
		return false;
	}
	string remote = rf.find("remote").asString().c_str();

	rf.check("moduleName") ? moduleName = rf.find("moduleName").asString() : moduleName = MODULE_NAME;
	printf("--> Module name set as %s\n", moduleName.c_str());

	cout << "connecting to device : " << DEVICE_NAME << endl;

	// in futuro inserire in config file quali device aprire nella sessione corrente
	bool done = true;
	done = msg.openDevice(rightArm,DEVICE_NAME,remote,MODULE_NAME);
	done = done && msg.openDevice(leftArm,DEVICE_NAME,remote,MODULE_NAME);
	done = done && msg.openDevice(torso,DEVICE_NAME,remote,MODULE_NAME);
	done = done && msg.openDevice(head,DEVICE_NAME,remote,MODULE_NAME);
	if (!done)
	{
		cout << "Error: cannot open interfaces" << endl;
		return false;
	}

	// in property inserire la modalità di controllo iniziale desiderata (def: position)
	// set reference motor acceleration and velocity

	Bright = rf.findGroup("RIGHT");
	Bleft  = rf.findGroup("LEFT");
	Btorso = rf.findGroup("TORSO");
	Bhead  = rf.findGroup("HEAD");
	
	//rf.check("ref_acc") ? ref_acc = rf.find("ref_acc").asDouble() : ref_acc = REF_ACC;
	//rf.check("ref_vel") ? ref_vel = rf.find("ref_vel").asDouble() : ref_vel = REF_VEL;

	// open position and torque interfaces:
	//done = done && msg.PositionView(rightArm, ref_vel, ref_acc);
	//done = done && msg.VelocityView(rightArm, ref_acc);
	done = done && msg.PositionView(rightArm, Bright);
	done = done && msg.VelocityView(rightArm, Bright);
	done = done && msg.TorqueView(rightArm, 0);

	done = done && msg.PositionView(leftArm, Bleft);
	done = done && msg.VelocityView(leftArm, Bleft);
	done = done && msg.TorqueView(leftArm, 0);

	done = done && msg.PositionView(torso, Btorso);
	done = done && msg.TorqueView(torso, 0);

	done = done && msg.PositionView(head, Bhead);

	if (!done)
	{
		cout << "Error: cannot open interfaces" << endl;
		return false;
	}

	cout << "--> devices open in position mode (default)" << endl;

	if(!openPorts()) return false;

	cout << "--> Module successfully configured" << endl;
	return true;
	
}

bool DevDriverModule::openPorts()
{
	// open module ports

	rf.check("rpcPortName") ? rpcPortName = rf.find("rpcPortName").asString() : rpcPortName = "/" + MODULE_NAME + PORT_NAME;
	
	cout << rpcPortName << endl;
	if( !rpcPort.open(rpcPortName.c_str()) )
	{
		printf("Unable to open rpc port\n");
		printf("Closing module\n");
		return false;
	}
	
	if(rf.find("EnableTorsoHeadStream").asInt())
	{
		string streamPortName;
		rf.check("streamPortName") ? streamPortName = rf.find("streamPortName").asString() : streamPortName = "/" + MODULE_NAME + "/state:o";
		if( !encStream.open(streamPortName.c_str()) )
		{
			cout << "Error: Unable to open torso-head encoders streaming port!" << endl;
		}
		cout << "-->Head-Torso encoders stream enabled at port " << streamPortName << endl;
	}

	if(!rf.check("maxError"))
	{
		cout << "Maximum positioning error parameter missing, using default value of 1 degree" << endl;
		maxError = 1;
	}
	else
	{
		maxError = rf.find("maxError").asDouble();
		cout << "Maximum positioning error set as 1 degrees" << endl;
	}

	attach(rpcPort);
	printf("--> Port %s listening\n",rpcPortName.c_str());

	return true;
}

double DevDriverModule::getPeriod()
{
	return 0.01;
}
	
bool DevDriverModule::updateModule()
{
	// fare controllo su streming durante il movimento del robot
	streamEncoders();
	//if(msg.commandPass())
	//{
	//	printf("Comando inviato\n");
	//}

	return true;
}

bool DevDriverModule::interruptModule()
{
	encStream.interrupt();

	return true;
}

bool DevDriverModule::close()
{
	printf("\n....Trying to stop module.....\n");

	//while(!msg.hasFinished());
	bool ok = msg.Release();

	encStream.close();
	rpcPort.interrupt();
	rpcPort.close();

	return true;
}


Bottle DevDriverModule::Grasp(Bottle angles, string side)
{
	CTRLpart part;
	Bottle enc;
	
	//read current position from encoders and modify hand joints:
	if (side == "right") 
	{
		part = rightHand;
		enc = GetEncoders(rightArm);
		cout << "rightHand selected" << endl;
	}
	if (side == "left")
	{
		part = leftHand;
		enc = GetEncoders(leftArm);
		cout << "leftHand selected" << endl;
	}
	
	//cout <<"angles: " << angles.toString() << endl;
	//cout <<"encoders: " << enc.toString() << endl;

	Bottle out;
	Vector commands;

	commands.resize(7+angles.size());
	for (int i=0; i<7; i++)
		commands(i) = enc.get(i).asDouble();
	for(int i=0; i<angles.size(); i++)
		commands(i+7) = angles.get(i).asDouble();

	cout << "sending: " << endl;
	cout << commands.toString() << endl;

	// one-shot grasp mode
	if (!msg.PositionMove(part,commands))   out.append("Error!");
	else									out.append("Done");

	return out;
}

Bottle DevDriverModule::SafeGrasp(Bottle FinalAngles, Vector FingerSpeed, string side)
{
	CTRLpart part;
	Bottle enc;
	
	if (FingerSpeed.size() != 9) 
	{
		Bottle out("Error, speed vector size mismatch");
		return out;
	}
	if (FinalAngles.size() != 9) 
	{
		Bottle out("Error, final angles vector size mismatch");
		return out;
	}

	//read current position from encoders and modify hand joints:
	if (side == "right") 
	{
		part = rightHand;
		enc = GetEncoders(rightArm);
		cout << "rightHand selected" << endl;
	}
	if (side == "left")
	{
		part = leftHand;
		enc = GetEncoders(leftArm);
		cout << "leftHand selected" << endl;
	}

	Vector handEnc(9);
	Vector speed(15);
	speed.zero();

	cout << "finger speed" << endl;
	cout << FingerSpeed.toString() << endl;

	msg.sensitiveGrasp(Bottle2Vector(FinalAngles),FingerSpeed, part);

	Bottle out("Done");

	return out;
}

bool DevDriverModule::respond(const Bottle &command, Bottle &reply)
{
	reply.clear();
	cout << "Received: " << command.get(0).asString() << endl;

	DEVCommands cmd;

	//identifyCmd reads only the first keyword
	if(!identifyCmd(command, cmd))
	{
		reply.addString("Unknown command");//. Input 'help' to get a list of the available commands.");
		return true;
	}

	// useful variables:
	bool ok,error;
	bool rightL, leftL, torsoL, headL;
	string part, what;
	Vector init;
	CTRLpart p;
	fingers f;
	Bottle * joints;
	Bottle * fingersVel;
	Bottle list;

	switch (cmd)
	{
	case ACK:		// answer to a connection check			
			reply.addString("ACK");
			return true;
			
	case getEncoders:		// read and send encoders values
			while(!msg.hasFinished(rightL, leftL, torsoL, headL));
			encSem.wait();
			if( command.size() == 2 )
				reply.append(GetEncoders(command.get(1).asString().c_str()));
			else
				reply.append(getArmsChainEncoders());
			encSem.post();
			return true;
	
	case getControlModes:
			if(command.size() < 2)
			{
				reply.append("Error: wrong command syntax");
				return false;
			}

			part = command.get(1).asString();
			if (part == "right")			p = rightArm;
			if (part == "left" )			p = leftArm;
			if (part == "torso")			p = torso;
			if (part == "head")				p = head;

			reply = msg.getJointsCtrlMode(p);
			return true;
		
	case compliant:
			if (command.size() < 2)
			{
				reply.append("Error: wrong command syntax");
				return false;
			}

			part = command.get(1).asString();
			if (part == "right")			p = rightArm;
			if (part == "left" )			p = leftArm;
			if (part == "torso")			p = torso;

			if (command.size() == 3)
			{
				joints = command.get(2).asList();
				cout << "joints: " << joints->toString() << endl;
				ok = msg.setJointsCtrlMode(p,torque,Bottle2Vector(*joints));
				if(ok)
				{
					reply.append("Done");
					return true;
				}
			}
			error = false;
			error = error && setCompliant(p);

			if (error)
			{
				reply.append("Error: couldn't set compliant mode");
				return true;
			}
			else
			{
				reply.append("Done");
				return true;
			}


	case stiff:
			if (command.size() < 2)
			{
				reply.append("Error: wrong command syntax");
				return false;
			}

			part = command.get(1).asString();
			if (part == "right")			p = rightArm;
			else if (part == "left" )			p = leftArm;
			else if (part == "torso")			p = torso;
			else return false;

			if (command.size() == 3)
			{
				joints = command.get(2).asList();
				cout << "joints: " << joints->toString() << endl;
				ok = msg.setJointsCtrlMode(p,position,Bottle2Vector(*joints));
				if(ok)
				{
					reply.append("Done");
					return true;
				}
			}
			else if (command.size() == 2)
			{
				bool error = false;
				cout << "set Stiff" << endl;
				error = error && setStiff(p);
				if (error)
				{
					reply.append("Error: couldn't set stiff mode");
					return true;
				}
				else
				{
					reply.append("Done");
					return true;
				}
			}
			else
			{
				reply.append("Error: wrong command syntax.");
				return false;
			}
		/*		
				string part = command.get(2).asString();
				if (part == "right")	error = error && setStiff(rightArm);
				if (part == "left" )	error = error && setStiff(leftArm);
				if (part == "torso")	error = error && setStiff(torso);

				if (error)
				{
					reply.append("Error: couldn't set stiff mode");
					return true;
				}
				else
				{
					reply.append("Done");
					return true;
				}
		*/

	case enable:		// enable touch sensors feedback
			if(command.size() != 4)
			{
				reply.append("Error: wrong command syntax");
				return false;
			}
			if (!command.get(3).isList())
			{
				reply.append("Error: wrong command syntax");
				return false;
			}
			what = command.get(1).asString();
			if (what == "touch")	
			{
				cout << command.toString() << endl;
				cout << "touch" << endl;
				msg.setActiveFingers(command.get(2).asString().c_str(), *(command.get(3).asList()), true);
				msg.enableTouch(command.get(2).asString().c_str());
			}

			reply.append("Done");
			return true;
			
	case disable:			// disable touch sensors feedback
			if(command.size() != 4)
			{
				reply.append("Error: wrong command syntax");
				return false;
			}
			if (!command.get(3).isList())
			{
				reply.append("Error: wrong command syntax");
				return false;
			}

			what = command.get(1).asString();
			if (what == "touch")	
			{
				if (command.get(3).asList()->size() == 1 && command.get(3).asList()->get(0).asString() == "all")
				{
					if (command.get(3).asList()->get(0).asString() == "all")
						msg.disableTouch(command.get(2).asString().c_str());

				}
				else
				{
					msg.setActiveFingers(command.get(2).asString().c_str(),*(command.get(3).asList()), false);
					msg.enableTouch(command.get(2).asString().c_str());
				}
			}

			reply.append("Done");
			return true;

	case grasp:
			if (command.size() != 3)
			{
				reply.append("Error: wrong command syntax");
				return false;
			}		
				
			encSem.wait();

				joints = command.get(1).asList(); // angles list
				part = command.get(2).asString();

				reply = Grasp(*joints,part);
					
			encSem.post();				
			return true;

	case safeGrasp:
			if (command.size() != 4)
			{
				reply.append("Error: wrong command syntax");
				return false;
			}		
			
			encSem.wait();

				joints = command.get(1).asList();
				fingersVel = command.get(2).asList();
				part = command.get(3).asString();

				reply = SafeGrasp(*joints,Bottle2Vector(*fingersVel),part);
						
			encSem.post();				
			return true;
	
	case move:			// send received joint angles to motors in position mode
			encSem.wait();

				cout << "dentro" << endl;
				ok = true;

				list = command.tail();

				rightL = list.check("right");
				leftL = list.check("left");
				torsoL = list.check("torso");
				headL = list.check("head");

				if( !rightL && !leftL && !torsoL && !headL)
				{
					reply.addString("ERROR");
					encSem.post();
					return false;
				}

				//move torso:
				if (torsoL)
				{
					joints = list.find("torso").asList();
					cout << "moving torso..." << endl << joints->toString() << endl;

					ok = ok && msg.PositionMove(torso,Bottle2Vector(*joints));
				}

				//move right arm:
				if (rightL)
				{
					joints = list.find("right").asList();
					joints->append(GetEncoders(rightHand));
					cout << "moving right arm..." << endl << joints->toString() << endl;
						
					ok = ok && msg.PositionMove(rightArm,Bottle2Vector(*joints));
				}

				//move left arm:
				if (leftL)
				{
					joints = list.find("left").asList();	
					joints->append(GetEncoders(leftHand));
					cout << "moving left arm..." << endl << joints->toString() << endl;
						
					ok = ok && msg.PositionMove(leftArm,Bottle2Vector(*joints));
				}

				//move head:
				if (headL)
				{
					joints = list.find("head").asList();
					if (joints->size() != msg.headDevice.Nj)
					{
						Bottle encH = GetEncoders(head);
						int size = joints->size();
						for (int i=size; i< msg.headDevice.Nj; i++)
							joints->addDouble(encH.get(i).asDouble());
					}
					cout << "moving head..." << endl << joints->toString() << endl;

					ok = ok && msg.PositionMove(head,Bottle2Vector(*joints));
					if (ok) cout << "mosso!" << endl;

				}

					// check motion is finished:
			//		do
			//		{check = check && msg.hasFinished();}
			//		while(!check);
					
				

			/*
					if (R->size() != 7 || L->size() != 7 || T->size() != 3 || H->size() != 6)
					{
						reply.addString("ERROR");
						encSem.post();
						return false;
					}

					for (int i = 0; i < R->size(); i++)
						msg.commandR(i) = R->get(i).asDouble();
					for (int i = 0; i < L->size(); i++)
						msg.commandL(i) = L->get(i).asDouble();
					for (int i = 0; i < T->size(); i++)
						msg.commandT(i) = T->get(i).asDouble();
					for (int i = 0; i < H->size(); i++)
						msg.commandH(i) = H->get(i).asDouble();

					/*
					for (int i=1; i <= command.size(); i++)
					{
						if(i<=7)			right(i-1) = command.get(i).asDouble();
						if(i>=8 && i<=15)	left(i-8)  = command.get(i).asDouble();
						if(i>15)			torso(i-15) = command.get(i).asDouble();
					}
					*/
				/*
					if(!msg.checkLimits())
					{
						reply.addString("ERROR");
						encSem.post();
						return false;
					}
			*/
					/*
					bool check = msg.posR->positionMove(msg.commandR.data());
					check =  check && msg.posL->positionMove(msg.commandL.data());
					check =  check && msg.posT->positionMove(msg.commandT.data());
					check =  check && msg.posH->positionMove(msg.commandH.data());
					*/

				if (ok)		reply.addString("Done");
				else		reply.addString("Error");

			encSem.post();
			return true;
			
	case initHead:			// set head position
			encSem.wait();
				cout << "DevDr InitHead" << endl;
				cout << command.toString() << endl;

				init.resize(msg.headDevice.Nj,0.0);

				for (int i=1; i <= command.size(); i++)
				{
					init(i-1) = command.get(i).asDouble();
				}

				cout << "head " << init.toString() << endl;
				ok = msg.PositionMove(head,init);

				reply.addString("Done");
			encSem.post();
			return true;

	case initRightArm:			// set right Arm position
			encSem.wait();

				/*init.resize(7,0.0);

				for (int i=1; i <= command.size(); i++)
				{
					init(i-1) = command.get(i).asDouble();
				}
				*/

				init.resize(msg.rightDevice.Nj,0.0);

				for (int i=1; i <= command.size(); i++)
				{
					init(i-1) = command.get(i).asDouble();
				}

				cout << "right arm " << init.toString() << endl;
				ok = msg.PositionMove(rightArm,init);

				reply.addString("Done");
			encSem.post();
			return true;

	case initLeftArm:			// set left Arm position
			encSem.wait();

				//init.resize(7,0.0);
				init.resize(msg.leftDevice.Nj,0.0);

				for (int i=1; i <= command.size(); i++)
				{
					init(i-1) = command.get(i).asDouble();
				}

				cout << "left arm " << init.toString() << endl;
				ok = msg.PositionMove(leftArm,init);

				reply.addString("Done");
			encSem.post();
			return true;
	
	case initTorso:			// set Torso position
			encSem.wait();

				init.resize(msg.torsoDevice.Nj,0.0);

				for (int i=1; i <= command.size(); i++)
				{
					init(i-1) = command.get(i).asDouble();
				}

				cout << "torso " << init.toString() << endl;
				ok = msg.PositionMove(torso,init);
				reply.addString("Done");

			encSem.post();
			return true;

	case record:			

			if (command.size() == 2)
			{
				f = index;
			}
			else if (command.size() != 3) 
			{
				reply.addInt(-3);
				reply.append("Error: wrong syntax");
				return false;
			}
			else
			{
				f = msg.string2fingers(command.get(2).asString().c_str());
			}

			part = command.get(1).asString();
			cout << part << endl;

			if (part == "stop")
			{
				ok = msg.StopRecording();
				ok ? reply.addInt(0) : reply.addInt(-4);
				return true;
			}

			encSem.wait();
				if (part == "right") 
					//Record3dPosition(rightArm);
					reply.append(Record3dPosition(rightArm,f));
				else
					//Record3dPosition(leftArm);
					reply.append(Record3dPosition(leftArm,f));
			encSem.post();
			//reply.append("Command sent");
			return true;

	case checkMotionFinished:
		encSem.wait();
			ok = msg.hasFinished(rightL, leftL, torsoL, headL);
		encSem.post();
		if(ok) reply.append("true");
		else   reply.append("false");
		return true;

	case checkGraspFinished:
		encSem.wait();
			ok = msg.hasFinished(rightL, leftL, torsoL, headL);
			ok = ok && msg.graspSucceeded;
		encSem.post();
		if(ok) reply.append("true");
		else   reply.append("false");
		return true;

	case stop:
		encSem.wait();
			ok = msg.hasFinished(rightL, leftL, torsoL, headL);
			if(!ok)
			{
				ok = true;
				if(!rightL)
				{
					init.resize(msg.rightDevice.Nj);
					msg.stop();
					for(unsigned int i=0; i<init.size(); i++) init(i) = i;
					msg.setJointsCtrlMode(rightArm,position,init);
					ok = ok && msg.rightDevice.ipos->stop();
				}
				if(!leftL)
				{
					init.resize(msg.leftDevice.Nj);
					if(rightL)	msg.stop();
					for(unsigned int i=0; i<init.size(); i++) init(i) = i;
					msg.setJointsCtrlMode(leftArm,position,init);
					ok = ok && msg.leftDevice.ipos->stop();
				}
				if(!torsoL)
				{
					init.resize(msg.torsoDevice.Nj);
					for(unsigned int i=0; i<init.size(); i++) init(i) = i;
					msg.setJointsCtrlMode(torso,position,init);
					ok = ok && msg.torsoDevice.ipos->stop();
				}
				if(!headL)
				{
					init.resize(msg.headDevice.Nj);
					for(unsigned int i=0; i<init.size(); i++) init(i) = i;
					msg.setJointsCtrlMode(head,position,init);
					ok = ok && msg.headDevice.ipos->stop();
				}

				if (ok) reply.append("Done");
				else	reply.append("Error, could not stop motion");
			}
			else
				reply.append("Done");
			
			encSem.post();
			return true;
	}

	return true;
}

bool DevDriverModule::setCompliant(CTRLpart part)
{
	Vector joints(5);
	if (part == torso)	joints.resize(3);
	for (unsigned int i=0; i<joints.size(); i++)
		joints(i) = i;

	cout << joints.toString() << endl;

	return msg.setJointsCtrlMode(part,torque,joints);	
	//return true;
}

bool DevDriverModule::setStiff(CTRLpart part)
{
	Vector joints(5);
	if (part == torso)	joints.resize(3);
	for (unsigned int i=0; i<joints.size(); i++)
		joints(i) = i;
	
	cout << joints.toString() << endl;

	return msg.setJointsCtrlMode(part,position,joints);	
	//return true;
}

bool DevDriverModule::checkMotion(CTRLpart part, Vector &actualAngles)
{
	bool done = false;
	bool rd,ld,td,hd;
	unsigned int t = 0;
	while(!done)
	{
		if( t==20 )break;
		done = msg.hasFinished(rd, ld, td, hd);
		Time::delay(0.1);
		t++;
	}
	//read current position from encoders and modify hand joints:
	Bottle enc = GetEncoders(part);
	double diff = 0;
	bool ok = true;

	cout << "diff:" << endl;
	for (unsigned int i=0; i<actualAngles.size(); i++)
	{
		diff = actualAngles(i)-enc.get(i).asDouble();
		cout << diff << " " << endl;
		if( fabs(diff) > maxError )
		{
			actualAngles(i) = enc.get(i).asDouble();
			ok = false;
		}
	}
	return ok;
}

// send a stream for head and torso encoders only
void DevDriverModule::streamEncoders()
{
	Vector T = msg.getEncoders(torso);
	Vector H = msg.getEncoders(head);

	Bottle stream;
	stream.clear();

	Bottle &list1 = stream.addList();
	Bottle &list2 = stream.addList();

	for (unsigned int i=0;i<T.size();i++)
		list1.addDouble(T(i));
	for (unsigned int i=0;i<H.size();i++)
		list2.addDouble(H(i));

	encStream.write(stream);
}

// get a specific part ecnoders vector
Bottle DevDriverModule::GetEncoders(CTRLpart part)
{
	Bottle enc;

	enc = Vector2Bottle(msg.getEncoders(part));
	cout << "encoders: " << endl;
	cout << enc.toString() << endl;
	
	return enc;
}

Bottle DevDriverModule::GetEncoders(string p)
{
	CTRLpart part;
	part = msg.string2enum(p);

	return GetEncoders(part);
}


// get right arm- left arm and torso encoders vector
Bottle DevDriverModule::getArmsChainEncoders()
{
	cout << "getArmsChain enc" << endl;
	Bottle enc;
	enc.clear();

	enc.addString("right");
	Bottle& R = enc.addList();
	enc.addString("left");
	Bottle& L = enc.addList();

	cout << "torso request" << endl;

	R = L = Vector2Bottle(msg.getEncoders(torso));

	// check if all values have been read from encoders
	if (R.size() == 1 || L.size() == 1) 
	{
		enc.clear();
		enc.addString("error");
		return enc;
	}

	R.append(Vector2Bottle(msg.getEncoders(rightArm)));
	L.append(Vector2Bottle(msg.getEncoders(leftArm)));

	if (R.size() == 4 || L.size() == 4)
	{
		enc.clear();
		enc.addString("error");
		return enc;
	}

	cout << "valori letti" << endl;
	cout << "right chain: " << R.toString() << endl;
	cout << "left chain: " <<  L.toString() << endl;
	cout << "sending: " << enc.toString() << endl;
		
	return enc;
}

Bottle DevDriverModule::Record3dPosition(CTRLpart part, fingers activeF)
{
	cout << "in record position" << endl;
	Bottle pos;

	// create ikin chain
	iCubArm * arm;
	if(part == rightArm)
		arm = new iCubArm("right");
	else
		arm = new iCubArm("left");

	iKinChain * chain = arm->asChain();		
	chain->releaseLink(0);
	chain->releaseLink(1);
	chain->releaseLink(2);

	// initialize the recorder
	bool ok = msg.InitRecording(part, activeF);
	if(!ok) 	{pos.addInt(-5);pos.append("Error! couldn't initialize the recorder"); return pos;}
	else		cout << "Recording initialization done" << endl;

	// get encoders values
	cout << "launch recording" << endl;
	msg.LaunchRecording();

	// wait for touch event
	cout << "waiting ... " << endl;
	msg.recordedPosition.wait();
	cout << "I have position" << endl;

	// for encoders debug:
	if(msg.recordedEncoders.size() !=10) {pos.addInt(-6); pos.append("Error in reading from encoders"); return pos;}
	Vector q = Bottle2Vector(msg.recordedEncoders);
	double temp = q(0);
	q(0) = q(2);
	q(2) = temp;

	cout << q.toString() << endl;
	chain->setAng(q*CTRL_DEG2RAD);
	cout << ((chain->getAng())*CTRL_RAD2DEG).toString() << endl;
	pos = Vector2Bottle( (chain->getH().getCol(3)).subVector(0,2) );
	cout << pos.toString() << endl;
	pos.clear();

	// add index offsets to the end-effector matrix

	Matrix EE(4,4);
	EE.eye();
	switch (activeF)
	{
	case index:
		cout << "index" << endl;
		if (part == rightArm)
		{
			EE(0,3) = INDEX_OS_R[0];
			EE(1,3) = INDEX_OS_R[1];
			EE(2,3) = INDEX_OS_R[2];
		}
		else
		{
			EE(0,3) = INDEX_OS_L[0];
			EE(1,3) = INDEX_OS_L[1];
			EE(2,3) = INDEX_OS_L[2];
		}
		break;
	case middle:
		cout << "middle" << endl;
		if (part == rightArm)
		{
			EE(0,3) = MIDDLE_OS_R[0];
			EE(1,3) = MIDDLE_OS_R[1];
			EE(2,3) = MIDDLE_OS_R[2];
		}
		else
		{
			EE(0,3) = MIDDLE_OS_L[0];
			EE(1,3) = MIDDLE_OS_L[1];
			EE(2,3) = MIDDLE_OS_L[2];
		}
		break;
	case ring:
		cout << "ring" << endl;
		if (part == rightArm)
		{
			EE(0,3) = RING_OS_R[0];
			EE(1,3) = RING_OS_R[1];
			EE(2,3) = RING_OS_R[2];
		}
		else
		{
			EE(0,3) = RING_OS_L[0];
			EE(1,3) = RING_OS_L[1];
			EE(2,3) = RING_OS_L[2];
		}
		break;

	case pinky:
		cout << "pinky" << endl;
		if (part == rightArm)
		{
			EE(0,3) = PINKY_OS_R[0];
			EE(1,3) = PINKY_OS_R[1];
			EE(2,3) = PINKY_OS_R[2];
		}
		else
		{
			EE(0,3) = PINKY_OS_L[0];
			EE(1,3) = PINKY_OS_L[1];
			EE(2,3) = PINKY_OS_L[2];
		}
		break;
	}
	
	cout << EE.toString() << endl;
	chain->setHN(EE);
	chain->setAng(q*CTRL_DEG2RAD);
	pos.addInt(0);
	pos.append( Vector2Bottle( (chain->getH().getCol(3)).subVector(0,2) ) );
	cout << pos.toString() << endl;

	return pos;
}

// Convert a command in a bottle into the correspondent enum value
bool DevDriverModule::identifyCmd(Bottle cmdBot, DEVCommands &cmd)
{
	unsigned int i=0;
	string word = cmdBot.get(0).asString().c_str();
	
	for (unsigned int i=0; i < CMD_SIZE; i++)
	{
		if (word == CMD_LIST[i])
		{
			cmd = (DEVCommands)i;
			return true;
		}
	}

	return false;
}
