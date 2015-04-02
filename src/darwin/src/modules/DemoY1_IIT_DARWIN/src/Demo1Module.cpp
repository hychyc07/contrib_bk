#include "darwin/Demo1Module.h"
#include <math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace Darwin::Demo1;

const double Demo1Module::SAFETY_THR = -0.11;
const double Demo1Module::SAFETY_Z   = -0.02;

const double Demo1Module::OFFSET_R_TOP [] = {0.015, 0.0, 0.01};
const double Demo1Module::OFFSET_L_TOP [] = {0.015, 0.0, 0.01};


int Demo1Module::Unstack()
{
	int exists = Memory.count("bin");
	if (exists == 0)	return 1;

	double xb = (Memory.find("bin")->second)[0];
	double yb = (Memory.find("bin")->second)[1];
	double zb = (Memory.find("bin")->second)[2];
	double p[3];
	vector <double> point;
	point.resize(3);

	p[0] = xb;
	p[1] = yb;
	p[2] = zb;
	point.assign(p,p+3);

	Memory.clear();
	int ok = updateMap();

	if (ok!=0) 
	{
		Memory["bin"] = point;
		return ok;
	}
	 
	string label = findHighest();
	Memory["bin"] = point;
	if (label == "") return -2;

	ok = Recycle(label);
	if (ok!=0) return ok;

	return 0;
}

string Demo1Module::findHighest()
{
	string label = "";
	map< string,vector<double> >::iterator it;
	map< string,vector<double> >::iterator it2;
	double z = SAFETY_THR;

	for (it=Memory.begin(); it!=Memory.end(); it++)
	{
		if(((*it).second)[2] > z)
		{
			z = ((*it).second)[2];
			label = (*it).first;
		}
	}

	cout << "max object is: " << label << endl;
	return label;
}

int Demo1Module::updateMap()
{
	string label;
	vector < double > point;
	point.resize(3);
	double p [3];
	double z = -0.09;
	int position = 0;
	Bottle * msg;
	msg = fromVision.read(false);

	while (msg == NULL)
	{
		Time::delay(0.5);
		msg = fromVision.read(false);
	}
	cout << msg->toString() << endl;
	int i = 0;

	for (i=0; i< msg->size(); i++)
	{
		Bottle * obj = msg->get(i).asList();
		cout << obj->toString().c_str() << endl;

		label = obj->get(0).asString().c_str();

		cout << label << endl;
		for (int j=1; j<obj->size()-2; j+=3)
		{
			if(obj->get(j+2).asDouble() > z)
			{
				z = obj->get(j+2).asDouble();
				cout << z << endl;
				position = j;
			}
		}

		p[0] = obj->get(position).asDouble();
		p[1] = obj->get(position+1).asDouble();
		p[2] = obj->get(position+2).asDouble();

		point.assign(p,p+3);		

		cout << point[0] << " " << point[1] << " " << point[2] << endl;

		// check point consistency:
		if (point[0] > -0.2 || point[0] < -0.5) return -12;
		if (point[1] < -0.4 || point[1] > 0.4) return -12;
		if (point[2] < -0.05 || point[2] > 0.4) return -12;

		Memory[label] = point;

		point.clear();
	}

	
	if (i == 0) return -13;

	return 0;
}














Demo1Module::Demo1Module(ResourceFinder &rf, int period): RateThread(period)
{
	// configure from ResourceFinder and create thread options:
	Property opt;

	string rpcName;

	if(!rf.check("threadName")) Name = "planner";	
	else Name = rf.find("threadName").asString();
	printf("--> Thread name is %s\n", Name.c_str());

	OutPortName = "/" + Name;
	rf.check("OutPortName") ? OutPortName += rf.find("OutPortName").asString() : OutPortName += "/msg:o";

	// facial expressions
	sad.addString("set");
	sad.addString("all");
	sad.addString("sad");

	happy.addString("set");
	happy.addString("all");
	happy.addString("hap");

	think.addString("set");
	think.addString("mou");
	think.addString("neu");

	Width = 0.10;
	Height = 0.15;
	state = idle;
}

bool Demo1Module::arePortsConnected()
{
	bool reply =     toPmp.getOutputCount() != 0;
	reply = reply && toDevDriver.getOutputCount() != 0;
	reply = reply && toGrasper.getOutputCount() != 0;
//	reply = reply && fromTouchR.getOutputCount() != 0;
//	reply = reply && fromTouchL.getOutputCount() != 0;

	return reply;
}

bool Demo1Module::threadInit()
{
	
	if(!msgOut.open(OutPortName.c_str()))
	{
		printf("Error: unable to open output communication port, closing\n");
		return false;
	}
	printf("--> Output communication port name set as %s\n", OutPortName.c_str());

	// open rpc communication ports with lower level modules
	if ( !toPmp.open( ("/" + Name + "/pmp:rpc").c_str() ) ||
		 !toDevDriver.open( ("/" + Name + "/dev:rpc").c_str() ) ||
		 !toGrasper.open( ("/" + Name + "/grasp:rpc").c_str() ) ||
		 !fromVision.open( ("/" + Name + "/vision:i").c_str() ) ||
		 !toFace.open( ("/" + Name + "/face:o").c_str() )		||
		 !fromTouchR.open( ("/" + Name + "/touchR:i").c_str() ) ||
		 !fromTouchL.open( ("/" + Name + "/touchL:i").c_str())
		 )

	{
		cout << "Error configuring thread ports" << endl;
		return false;
	}

	// set ports timeout:
	toPmp.setTimeout(60.0);
	toDevDriver.setTimeout(60.0);
	toGrasper.setTimeout(60.0);

	// other initializations
	danglingPoint.resize(0);

	return true;
}


void Demo1Module::threadRelease()
{
	toPmp.interrupt();
	toDevDriver.interrupt();
	toGrasper.interrupt();
	toFace.interrupt();
	fromTouchR.interrupt();
	fromTouchL.interrupt();
	msgOut.interrupt();

	toPmp.close();
	toDevDriver.close();
	toGrasper.close();
	toFace.close();
	fromTouchR.close();
	fromTouchL.close();
	msgOut.close();
}

bool Demo1Module::stopMotion()
{
	Bottle cmd, reply;
	cmd.append("stop");
	toDevDriver.write(cmd, reply);
	if (reply == "Done")	return true;
	else					return false;
}

string Demo1Module::whichHandOnTarget(const Bottle & handsPos, const double & x, const double & y, const double & z)
{
	string hand = "none";
	double handx, handy, handz;

	// check if right hand:
		handx = handsPos.get(0).asDouble();
		handy = handsPos.get(1).asDouble();
		handz = handsPos.get(2).asDouble();

		if ( (fabs(handx-x) <= 0.07) && (fabs(handy-y) <= 0.04) && (fabs(handz-z) <= 0.05) )
			hand = "right";
		else
		{
			handx = handsPos.get(3).asDouble();
			handy = handsPos.get(4).asDouble();
			handz = handsPos.get(5).asDouble();
			if ( (fabs(handx-x) <= 0.07) && (fabs(handy-y) <= 0.04) && (fabs(handz-z) <= 0.05) )
				hand = "left";
		}

		cout << hand << endl;
	return hand;
}

int Demo1Module::prepare4Grasping(const string & _label, const string & _mode)
{
	string mode = _mode;
	// check if the object exists in memory
	int exists = Memory.count(_label);
	if (exists == 0)	return -1;

	double x = (Memory.find(_label)->second)[0];
	double y = (Memory.find(_label)->second)[1];
	double z = (Memory.find(_label)->second)[2];

	// check the hands position: are they close enaugh to the object to grasp it?
		Bottle pose = getHandsPose();
		if(pose.size() != 6)	return -2;

		activeChain = whichHandOnTarget(pose,x,y,z);

		if (_mode == "bin")
		{
			if( (Memory.find("bin")->second)[1] > 0) activeChain = "right";
			else activeChain = "left";	
			mode = "top";
		}

		cout << activeChain << endl;
	// if one hand is close enough to grasp an object, then return to grasp it
	// otherwise reach the target before grasping
	if (activeChain != "none")
	{
		int done = Reach2Grasp(_label,activeChain,mode);
		while(!hasFinished()) Time::delay(0.1);

		Bottle cmd, reply;
		cmd.addString("grasp");
		cmd.addString("graspStart"); // primitiva
		cmd.addString(activeChain.c_str());
		toGrasper.write(cmd, reply);

		if (reply == 0);
		while(!hasFinished()) Time::delay(0.1);

		return 0;
	}
	else
	{
		int done = Reach2Grasp(_label,"",mode);
		while(!hasFinished()) Time::delay(0.1);
		Bottle cmd, reply;
		cmd.addString("grasp");
		cmd.addString("graspStart"); // primitiva
		cmd.addString(activeChain.c_str());

		toGrasper.write(cmd, reply);
		if (reply == 0);
		while(!hasFinished()) Time::delay(0.1);
		//int done = -8;
		return done;
	}
}

int Demo1Module::initIcubUp()
{
	Bottle cmd, reply;
	cmd.clear();
	reply.clear();
	cmd.addString("initIcubUp");
	toPmp.write(cmd,reply);
	if (reply.get(0).asString() != "Done")	return -8;

	while(!hasFinished()) Time::delay(0.05);
	return 0;
}

int Demo1Module::SequenceReach(Bottle labels)
{
	// parse bottle
	Bottle cmd, reply;
	int done;

	// check if the object exists in memory
	int exists = 0;
	for (int i=0; i< labels.size(); i++)
		exists += Memory.count(labels.get(i).asString().c_str());

	if (exists != labels.size())	return -1;

	state = reaching;

	// initialize the robot with prepare:
	done = Prepare();
	if(done != 0) return done;

	// get the target sequence:
	Vector tg(3);
	for (int i=0; i<labels.size(); i++)
	{
		cout << "label" << endl;
		tg(0) = (Memory.find(labels.get(i).asString().c_str())->second)[0];
		tg(1) = (Memory.find(labels.get(i).asString().c_str())->second)[1];
		tg(2) = (Memory.find(labels.get(i).asString().c_str())->second)[2];

		reach1hand.SetIndexActive(true);
		Bottle stream = reach1hand.activate(tg);
		activeChain   = reach1hand.GetActiveChain();
		cout << "stream is: " << endl;
		cout << stream.toString() << endl;

		// initialize for reaching
		if(!setWrist("top",activeChain)) return -9;

		Bottle fingers("index");

		if (activeChain == "right") 	done = initReaching(true,true,fingers,activeChain);
		if (activeChain == "left")	 	done = initReaching(true,true,fingers,activeChain);
		if (done != 0) return done;

		// send commands to PMP and check reaching performance
		done = sendPMPstream(*(stream.get(0).asList()), *(stream.get(1).asList()));
		if (done != 0) return done;
		done = checkReachingSuccess(labels.get(i).asString().c_str());
		if (done != 0) return done;

	// go far from the reached target
		done = goFar(activeChain);
		if (done != 0) return done;
	}
	
	done = Relax();	

	state = idle;
	return done;
	
}

int Demo1Module::initReaching(bool fingerAsEE, bool touchActive, Bottle fingers, string side)
{
	Bottle cmd, reply;

	// always disable all fingers first
		cmd.append("disable touch left (all)");
		toDevDriver.write(cmd,reply);
		if (reply.get(0).asString() != "Done")	return -8;
		cmd.clear();
		reply.clear();
		cmd.append("disable touch right (all)");	
		toDevDriver.write(cmd,reply);
		if (reply.get(0).asString() != "Done")	return -8;

	if (fingerAsEE)
	{
		cout << "finger as EE " << endl;
		cmd.clear();
		reply.clear();
		cmd.append("grasp point");
		cmd.addString(side.c_str());

		toGrasper.write(cmd, reply);
		if (reply.get(0).asInt() != 0)	return -8;
	}
	else
	{
		cout << "not finger as EE " << endl;
		cmd.clear();
		reply.clear();
		cmd.append("release");
		cmd.addString(side.c_str());

		toGrasper.write(cmd, reply);
		if (reply.get(0).asInt() != 0)	return -8;
	}

	if (activeChain == "right")
	{
		cmd.clear();
		cmd.append("initLeftArm -18 70 -20 43");
		toDevDriver.write(cmd, reply);
	}
	if (activeChain == "left")
	{
		cout << "init arm" << endl;
		cmd.clear();
		cmd.append("initRightArm -18 70 -20 43");
		toDevDriver.write(cmd, reply);
	}

	while(!hasFinished()) Time::delay(0.05);

	if(touchActive)
	{
		cout << "touch active" << endl;
		cmd.clear();
		reply.clear();
		cmd.addString("enable");
		cmd.addString("touch");
		cmd.addString(side.c_str());
		Bottle & L = cmd.addList();
		L.append(fingers);

		toDevDriver.write(cmd,reply);
		if (reply.get(0).asString() != "Done")	return -8;
	}

	return 0;
}

int Demo1Module::FingerReach(const string & _label, const string & _side, const string & _mode, bool initUp)
{
	Bottle cmd, reply;

	// check if the object exists in memory
	int exists = Memory.count(_label);
	if (exists == 0)	return -1;

	Vector tg(3);
	tg(0) = (Memory.find(_label)->second)[0];
	tg(1) = (Memory.find(_label)->second)[1];
	tg(2) = (Memory.find(_label)->second)[2];

	state = reaching;

	while(!hasFinished()) Time::delay(0.05);

	// generate the command stream for reaching and save the active chain for grasping
	reach1hand.SetIndexActive(true);
	Bottle stream = reach1hand.activate(tg);
	activeChain   = reach1hand.GetActiveChain();
	cout << "stream is: " << endl;
	cout << stream.toString() << endl;

	// initialize for reaching
	if(!setWrist(_mode,activeChain)) return -9;
	Bottle fingers;
	Bottle & L = fingers.addList();
	L.addString("index");

	int done;
	if (activeChain == "right") 	done = initReaching(true,true,L,activeChain);
	if (activeChain == "left")	 	done = initReaching(true,true,L,activeChain);

	if (done != 0) return done;

	while(!hasFinished()) Time::delay(0.05);	

	int result = sendPMPstream(*(stream.get(0).asList()), *(stream.get(1).asList()));

	state = idle;
	return result;
}

int Demo1Module::Reach2Grasp(const string & _label, const string & _side, const string & _mode)
{
	Bottle cmd, reply;

	// check if the object exists in memory
	int exists = Memory.count(_label);
	if (exists == 0)	return -1;

	// generate the command stream for reaching and save the active chain for grasping
	reach1hand.SetIndexActive(false);
	Vector tg(3);
	tg(0) = (Memory.find(_label)->second)[0];
	tg(1) = (Memory.find(_label)->second)[1];
	tg(2) = (Memory.find(_label)->second)[2];

	if (_side != "none")
	{
		cout << "non none" << endl;
		Vector tg = setGraspingTarget(_label,activeChain);
		reach1hand.SetActiveChain(activeChain);
	}

	Bottle stream = reach1hand.activate(tg);
	activeChain   = reach1hand.GetActiveChain();
	cout << activeChain << endl;
	cout << "stream is: " << endl;
	cout << stream.toString() << endl;

	if (_side == "none")
		Vector tg = setGraspingTarget(_label,activeChain);

	state = reaching;
	while(!hasFinished()) Time::delay(0.05);
	cmd.clear();
	reply.clear();

	// set wrist orientation
	if(!setWrist(_mode,activeChain)) return -9;

	// initialize for reaching (the hand is released here)
	Bottle fingers;
	Bottle & L = fingers.addList();
	L.addString("all");
	
	int done;
	if (activeChain == "right") 	done = initReaching(false,true,L,activeChain);
	if (activeChain == "left")	 	done = initReaching(false,true,L,activeChain);	
	
	int result = sendPMPstream(*(stream.get(0).asList()), *(stream.get(1).asList()));
	return 0;

	state = idle;
	//return result;
}

Vector Demo1Module::setGraspingTarget(const string & _label, const string & _side)
{
	Vector tg(3);
	tg(0) = (Memory.find(_label)->second)[0];
	tg(1) = (Memory.find(_label)->second)[1];
	tg(2) = (Memory.find(_label)->second)[2];

	cout << "tg prima: " << tg.toString() << endl;
	if (_side == "right")
	{
		tg(0) += OFFSET_R_TOP[0];
		tg(1) += OFFSET_R_TOP[1];
		tg(2) += OFFSET_R_TOP[2];
	}
	else
	{
		tg(0) += OFFSET_L_TOP[0];
		tg(1) += OFFSET_L_TOP[1];
		tg(2) += OFFSET_L_TOP[2];
	}

	cout << "tg dopo: " << tg.toString() << endl;
	return tg;
}

int Demo1Module::sendPMPstream(Bottle b, Bottle answ)
{
	if (toPmp.getOutputCount() != 1)	
	{
		cout << "no output connection" << endl;
		return -4;
	}
	
	Bottle reply;
	for (int i=0; i<b.size(); i++)
	{
		Bottle *cmd = b.get(i).asList();
		cout << cmd->toString() << endl;

		toPmp.write(*cmd,reply);

		if (reply.isNull())
		{
			cout << "Communication error!" << endl;
			return -4;
		}

		if (reply.get(0).asString() != answ.get(i).asString())
		{
			cout << reply.toString() << endl;
			return -5;
		}
	}
	
	// wait for PMP to start the comuputation
	while(!isPMPrunning()) Time::delay(0.05);
	cout << "pmp va" << endl;
	while(isPMPrunning()) Time::delay(0.05);
	
	return 0;
}

Bottle Demo1Module::getHandsPose()
{
	while(!hasFinished()) Time::delay(0.05);
	Bottle cmd, reply;
	cmd.addString("PMP_get");
	cmd.addString("pose");
	toPmp.write(cmd,reply);
	cout << reply.toString() << endl;

	return reply;
}

bool Demo1Module::isPMPrunning()
{
	Bottle msg,reply;
	msg.addString("ready");

	toPmp.write(msg,reply);
	if(reply.get(0).asString() == "no")  return true;
	else								 return false;
}

bool Demo1Module::hasFinished()
{
	Bottle msg,reply;
	msg.addString("checkMotionFinished");

	toDevDriver.write(msg,reply);
	if(reply.get(0).asString() == "true") return true;
	else								  return false;
}

bool Demo1Module::startPMP()
{
	Bottle cmd, reply;
	if(toPmp.getOutputCount() == 0)
		return false;
	reply.clear();
	cmd.clear();
	cmd.append("startPMP execute");
	toPmp.write(cmd, reply);
	if(reply != "Done")	return false;
	return true;
}

bool Demo1Module::setWrist(const string & _mode, const string & _side)
{
	Bottle cmd, reply;
	if(toPmp.getOutputCount() == 0)
		return false;
	reply.clear();
	cmd.clear();
	if (_mode == "top")
	{
		cmd.addString("PMP_set");
		cmd.addString("wrist");
		cmd.addString(_side.c_str());
		cmd.addDouble(50);
		cout << "top set" << endl;
		cout << cmd.toString() << endl;

		toPmp.write(cmd, reply);

		cmd.clear();
		reply.clear();
		cmd.append("PMP_get wrist");
		cmd.addString(_side.c_str());
		cout << cmd.toString() << endl;
		toPmp.write(cmd, reply);
		cout << reply.toString() << endl;

		if (reply.get(0).asDouble() != 50.0)	return false;
		else									return true;
	}
	else if (_mode == "side")
	{
		cmd.addString("PMP_set");
		cmd.addString("wrist");
		cmd.addString(_side.c_str());
		cmd.addDouble(-45);
		cout << "side set" << endl;
		cout << cmd.toString() << endl;

		toPmp.write(cmd, reply);

		cmd.clear();
		reply.clear();
		cmd.append("PMP_get wrist");
		cmd.addString(_side.c_str());
		cout << cmd.toString() << endl;
		toPmp.write(cmd, reply);

		cout << reply.toString() << endl;
		if (reply.get(0).asDouble() != -45.0)	return false;
		else									return true;
	}
	else return false;

	// toPmp.write(cmd, reply);
	//if(reply.get(0).asString() != "Done")	return false;
	//return true;
}

int Demo1Module::Relax()
{
	// wait for previous actions to be finished
	while(!hasFinished()) Time::delay(0.1);

	Bottle cmd,cmd1,reply,reply1;
	if(toDevDriver.getOutputCount() == 0)
		return -1;
	cmd.append("disable touch left (all)");
	cmd1.append("disable touch right (all)");
	toDevDriver.write(cmd, reply);
	if(reply != "Done")		return -1;
	toDevDriver.write(cmd1, reply1);
	if(reply1 != "Done")	return -1;

	cmd.clear();
	cmd.append("initTorso");
	toDevDriver.write(cmd, reply);

	cmd.clear();
	cmd.append("initHead");
	toDevDriver.write(cmd, reply);
   
	cmd.clear();
//	cmd.append("initLeftArm -18 49 -20 43");
	cmd.append("initLeftArm -18 70 -20 43");
	toDevDriver.write(cmd, reply);
	
	cmd.clear();
//	cmd.append("initRightArm -18 49 -20 43");
	cmd.append("initRightArm -18 70 -20 43");
	toDevDriver.write(cmd, reply);

	return 0;
}

int Demo1Module::Prepare()
{
	Bottle cmd,cmd1,reply,reply1;
	if(toDevDriver.getOutputCount() == 0)
		return -1;
	cmd.append("disable touch left (all)");
	cmd1.append("disable touch right (all)");
	toDevDriver.write(cmd, reply);
	if(reply != "Done")		return -1;
	toDevDriver.write(cmd1, reply1);
	if(reply1 != "Done")	return -1;

	initIcubUp();

	reply.clear();
	cmd.clear();
	reply1.clear();
	cmd1.clear();
	cmd.append("enable touch left (all)");
	cmd1.append("enable touch right (all)");
	toDevDriver.write(cmd, reply);
	if(reply != "Done")		return -1;
	toDevDriver.write(cmd1, reply1);
	if(reply1 != "Done")	return -1;

	while(!hasFinished()) Time::delay(0.05);

	return 0;
}

int Demo1Module::checkReachingSuccess(const string & _label)
{
	// check PMP has finished the computation
	while(isPMPrunning())  Time::delay(0.05);

	// check if the object exists in memory
	int exists = Memory.count(_label);
	if (exists == 0)	return -1;

	double x = (Memory.find(_label)->second)[0];
	double y = (Memory.find(_label)->second)[1];
	double z = (Memory.find(_label)->second)[2];

	// always check that motion has finished before launch grasping
	while(!hasFinished()) Time::delay(0.05);

	Bottle pose = getHandsPose();
	if(pose.size() != 6)		return -2;
	//cout << pose.toString() << endl;

	string currentChain = whichHandOnTarget(pose,x,y,z);

	if (currentChain == activeChain)	return 0;
	else								return -8;
}

int Demo1Module::checkBimanualReachingSuccess(const string & _label1, const string & _label2)
{

	// check PMP has finished the computation
	while(isPMPrunning())  Time::delay(0.05);

	// check if the object exists in memory
	int exists = Memory.count(_label1) + Memory.count(_label2);
	if (exists != 2)	return -1;

	Vector point(3);
	point(0) = (Memory.find(_label1)->second)[0];
	point(1) = (Memory.find(_label1)->second)[1];
	point(2) = (Memory.find(_label1)->second)[2];

	Bottle pose = getHandsPose();
	if(pose.size() != 6)		return -2;
	//cout << pose.toString() << endl;

	if (_label1 == _label2)
	{
		Matrix tg(2,3);
		tg(0,0) = point(0) + 0.04;
		tg(0,1) = point(1) + Width*0.5 + 0.03;
		tg(0,2) = point(2) - Height * 0.3;

		string currentChain = whichHandOnTarget(pose,tg(0,0),tg(0,1),tg(0,2));
		if (currentChain != "right") return -8;

		tg(1,0) = point(0) + 0.04;
		tg(1,1) = point(1) - Width*0.5 - 0.03;
		tg(1,2) = point(2) - Height * 0.3;

		currentChain = whichHandOnTarget(pose,tg(1,0),tg(1,1),tg(1,2));
		if (currentChain != "left") return -8;
	}
	
	else
	{
		string currentChain = whichHandOnTarget(pose,point(0),point(1),point(2));
		if (currentChain == "none") return -8;

		point(0) = (Memory.find(_label2)->second)[0];
		point(1) = (Memory.find(_label2)->second)[1];
		point(2) = (Memory.find(_label2)->second)[2];

		string currentChain1 = whichHandOnTarget(pose,point(0),point(1),point(2));
		if (currentChain == "none") return -8;

		if (currentChain == currentChain1) return -8;
	}	

	return 0;
}

Bottle Demo1Module::Grasp()
{
	state = grasping;
	// always check that motion has finished before launch grasping
	Bottle cmd, reply;

	while(!hasFinished()) Time::delay(0.05);

	cmd.clear();
	reply.clear();

	cmd.addString("grasp");
	//cmd.addString("dtight");
	cmd.addString("safe");
	cmd.addString("max"); // primitiva
	cmd.addString(activeChain.c_str());

	toGrasper.write(cmd, reply);

	state = idle;
	return reply;
}

bool Demo1Module::Release(const string & _side)
{
	Bottle cmd, reply;
	while(!hasFinished()) Time::delay(0.05);

	cmd.addString("grasp");
	cmd.addString("home"); // primitiva
	cmd.addString(_side.c_str());

	toGrasper.write(cmd, reply);
	if (reply.get(0).asInt() == 0) return true;
	
	return false;
}

int Demo1Module::Lift(const string & _side, bool keepContact)
{
	Bottle pose = getHandsPose();
	if(pose.size() != 6)		return -2;

	while(!hasFinished()) Time::delay(0.05);

	// disable touch first: dev driver will signal if contact is lost anyway
	Bottle cmd, reply;
	if(toDevDriver.getOutputCount() == 0)
		return -4;

	if     (_side == "right")		cmd.append("disable touch right (all)");
	else if(_side == "left")		cmd.append("disable touch left (all)");
	else	
		return -6;
	
	toDevDriver.write(cmd, reply);
	if(reply != "Done")		return -4;

	// get current hand position and increment z
	Vector tg(3);
	if (_side == "right")
	{
		tg(0) = pose.get(0).asDouble();
		tg(1) = pose.get(1).asDouble();
		tg(2) = pose.get(2).asDouble() + 0.1;
	}
	else
	{
		tg(0) = pose.get(3).asDouble();
		tg(1) = pose.get(4).asDouble();
		tg(2) = pose.get(5).asDouble() + 0.1;
	}

	cmd.clear();
	reach1hand.IgnoreEE(true);
	cmd = reach1hand.activate(tg);
	activeChain = reach1hand.GetActiveChain();

	int done = sendPMPstream(*cmd.get(0).asList(),*cmd.get(1).asList());
	if (done == 0)
	{
		if (keepContact)	
		{
			bool ok = contactCheck();
			if (!ok) return -1;
		}
	}
	
	return done;
}

int Demo1Module::goFar(const string & _side, bool keepContact)
{
	while(!hasFinished()) Time::delay(0.05);

	Bottle pose = getHandsPose();
	if(pose.size() != 6)		return -2;

	// disable touch first: dev driver will signal if contact is lost anyway
	Bottle cmd,reply;
	if(toDevDriver.getOutputCount() == 0)
		return -4;

	if     (_side == "right")		cmd.append("disable touch right (all)");
	else if(_side == "left")		cmd.append("disable touch left (all)");
	else	
		return -6;
	
	toDevDriver.write(cmd, reply);
	if(reply != "Done")		return -4;

	// get current hand position and increment z
	Vector tg(3);
	double xos, yos;

	if (_side == "right")
	{
		if (pose.get(0).asDouble() < -0.40 )		xos = 0.10;
		else if (pose.get(0).asDouble() < -0.30 )	xos = 0.05;
		else if (pose.get(0).asDouble() < -0.25 )	xos = -0.05;
		else										xos = -0.10;
		if (pose.get(1).asDouble() > 0.25)			yos = -0.05;
		else										yos = 0.05;
		

		tg(0) = pose.get(0).asDouble() + xos;
		tg(1) = pose.get(1).asDouble() + yos;

		if (pose.get(2).asDouble() < 0.05)			tg(2) = 0.2;
		else if (pose.get(2).asDouble() < 0.15)		tg(2) = pose.get(2).asDouble() + 0.12;
		else if (pose.get(2).asDouble() < 0.20)		tg(2) = pose.get(2).asDouble() + 0.07;
		else										tg(2) = pose.get(2).asDouble() + 0.03;
	}
	else
	{
		if (pose.get(3).asDouble() < -0.40 )		xos = 0.10;
		else if (pose.get(3).asDouble() < -0.30 )	xos = 0.05;
		else if (pose.get(3).asDouble() < -0.25 )	xos = -0.05;
		else										xos = -0.10;
		if (pose.get(4).asDouble() < -0.25)			yos = 0.05;
		else										yos = -0.05;

		tg(0) = pose.get(3).asDouble() + xos;
		tg(1) = pose.get(4).asDouble() + yos;
		
		if (pose.get(5).asDouble() < 0.05)			tg(2) = 0.2;
		else if (pose.get(5).asDouble() < 0.15)		tg(2) = pose.get(5).asDouble() + 0.12;
		else if (pose.get(5).asDouble() < 0.20)		tg(2) = pose.get(5).asDouble() + 0.07;
		else										tg(2) = pose.get(5).asDouble() + 0.03;
	}

	cmd.clear();
	reach1hand.IgnoreEE(true);
	cmd = reach1hand.activate(tg);
	activeChain = reach1hand.GetActiveChain();

	int done = sendPMPstream(*cmd.get(0).asList(),*cmd.get(1).asList());
	if (done == 0)
	{
		if (keepContact)	
		{
			bool ok = contactCheck();
			if (!ok) return -1;
		}
	}
			
	while(!hasFinished()) Time::delay(0.05);
	cout << "finished far" << endl;
	
	return done;
}


bool Demo1Module::contactCheck()
{
	resume();
	while (!hasFinished())
	{
		if (isSuspended())	return false;
		Time::delay(0.05);
	}
	suspend();
	return true;
}



// reaching con mantenimento contact
int Demo1Module::Carry(const string & _side, const string & _tgLabel, bool keepContact)
{
	// check that the target point really exists in memory
	int exists = Memory.count(_tgLabel);
	if (exists == 0)	return -1;

	Vector tg(3);
	tg(0) = (Memory.find(_tgLabel)->second)[0];
	tg(1) = (Memory.find(_tgLabel)->second)[1];
	tg(2) = (Memory.find(_tgLabel)->second)[2];

	if (_tgLabel == "bin")		tg(2) += 0.15;


	while(!hasFinished()) Time::delay(0.05);

	// disable touch first: dev driver will signal if contact is lost anyway
	Bottle cmd, cmd1, reply, reply1;
	if(toDevDriver.getOutputCount() == 0)
		return -4;

	if     (_side == "right")		cmd.append("disable touch right (all)");
	else if(_side == "left")		cmd.append("disable touch left (all)");
	else	
		return -6;
/*	
	toDevDriver.write(cmd, reply);
	if(reply != "Done")		return -4;
	toDevDriver.write(cmd1, reply1);
	if(reply1 != "Done")	return -4;
*/

	cmd.clear();
	reach1hand.IgnoreEE(true);
	reach1hand.SetActiveChain(_side);
	cmd = reach1hand.activate(tg);
	activeChain = reach1hand.GetActiveChain();

	int done = sendPMPstream(*cmd.get(0).asList(),*cmd.get(1).asList());
	if (done == 0)
	{
		if (keepContact)	
		{
			bool ok = contactCheck();
			if (!ok) return -10;
		}
	}
	
	return done;
	
	return 0;
}

int Demo1Module::Recycle(const string & _label, const string & _mode)
{
	// check that the object and bin exist in memory
	int exists = Memory.count(_label);
	if (exists == 0)	return -1;

	exists = Memory.count("bin");
	if (exists == 0)	return 1;

	// grasping (reach2grasp if necessary)
	int done = prepare4Grasping(_label,"bin");
	string chain = activeChain;

	cout << activeChain << endl;

	if (done != 0)	return done;
	else			done = checkReachingSuccess(_label);

	if (done < 0)	return done;
	else			done = Grasp().get(0).asInt();

	// lifting with contact
	cout << "lift" << endl;
	if (done != 0)	return done;
	else			done = Lift(activeChain,true);
	// check PMP has finished the computation
	while(isPMPrunning())  Time::delay(0.05);

	// carry to bin
	cout << "carry" << endl;
	if (done != 0)	return done;
	else			done = Carry(activeChain,"bin", true);
	// check PMP has finished the computation
	while(isPMPrunning())  Time::delay(0.05);

	// throw
	cout << "throw" << endl;
	while(!hasFinished()) Time::delay(0.05);
	if (done != 0)	return done;
	else			done = Release(activeChain);

	return 0;
}


int Demo1Module::userRecordStart(bool left)
{
	state = recording;
	Bottle cmd, reply;

	cout << "record: " << endl;
	// concentrate
	toFace.write(think,reply);
	reply.clear();	

	if (left)
	{
		// set index as end effector in PMP
		cmd.append("setActiveChain left");
		toPmp.write(cmd,reply);
		cmd.clear();
		reply.clear();
		cmd.append("useIndex left on");
		toPmp.write(cmd,reply);
		if (reply != "Done") return -5;

		// cmd to dev driver
		cmd.clear();
		cmd.append("compliant left (0 1 2 3 4)");
		toDevDriver.write(cmd, reply);
		cmd.clear();
		cmd.append("compliant torso (2)");
		toDevDriver.write(cmd, reply);
	}
	else
	{
		// set index as end effector in PMP
		cmd.append("setActiveChain right");
		toPmp.write(cmd,reply);
		cmd.clear();
		reply.clear();
		cmd.append("useIndex right on");
		toPmp.write(cmd,reply);
		if (reply != "Done") return -5;

		// cmd to dev driver
		cmd.clear();
		cmd.append("compliant right (0 1 2 3 4)");
		toDevDriver.write(cmd, reply);
		cmd.clear();
		cmd.append("compliant torso (2)");
		toDevDriver.write(cmd, reply);
	}

	return 0;
}

int Demo1Module::userRecordStop(bool left)
{
	Bottle cmd, reply;
	Bottle pose;

	if (left)
	{
		// cmd to dev driver
		cmd.append("stiff left (0 1 2 3 4)");
		toDevDriver.write(cmd, reply);
		cmd.clear();
		cmd.append("stiff torso (2)");
		toDevDriver.write(cmd, reply);

		// ask to pmp the pose using index:
		pose = getHandsPose();
		if(pose.size() != 6)		return -6;

	}
	else
	{
		// cmd to dev driver
		cmd.append("stiff right (0 1 2 3 4)");
		toDevDriver.write(cmd, reply);
		cmd.clear();
		cmd.append("stiff torso (2)");
		toDevDriver.write(cmd, reply);

		pose = getHandsPose();
		if(pose.size() != 6)		return -6;
	}
	
	danglingPoint.clear();
	toFace.write(happy,reply);

	danglingPoint.push_back(pose.get(1).asDouble());
	danglingPoint.push_back(pose.get(2).asDouble());
	int val;
	if(reply.get(3).asDouble() <= SAFETY_THR)
	{
		danglingPoint.push_back(SAFETY_Z);
		val = 1;
	}
	else
	{
		danglingPoint.push_back(pose.get(3).asDouble());
		val = 0;
	}

	state = idle;
	return val;
}

bool Demo1Module::enableTouch()
{
	Bottle cmd, reply;
	cmd.append("enable touch left (all)");
	cmd.clear();
	toDevDriver.write(cmd,reply);
	cmd.clear();

	cmd.append("enable touch right (all)");
	toDevDriver.write(cmd,reply);

	return true;
}

int Demo1Module::Record(bool left)
{
	state = recording;
	Bottle cmd, reply;

	cout << "record: " << endl;
	// concentrate
	toFace.write(think,reply);
	reply.clear();

	// be sure touch is enabled:

	if (left)
	{
		// cmd to dev driver
		cmd.addString("record");
		cmd.addString("left");
	}
	else
	{
		// cmd to dev driver
		cmd.addString("record");
		cmd.addString("right");
	}
	toDevDriver.write(cmd, reply);
	cout << "reply: " << reply.toString() << endl;

	// cmd to initialize again the robot
	// intiLeftArm + initTorso
	
	// check point consistency: if not consistent, return false;
	if(reply.get(0).asInt() != 0)//size() != 3)
	{
		toFace.write(sad);
		state = idle;
		return reply.get(0).asInt();
	}

	// check that position is not zero
	if(reply.size() != 4 || reply.get(1).asDouble() == 0.0)
	{
		toFace.write(sad,reply);
		state = idle;
		return -5;
	}

	danglingPoint.clear();
	toFace.write(happy,reply);

	danglingPoint.push_back(reply.get(1).asDouble());
	danglingPoint.push_back(reply.get(2).asDouble());
	int val;
	if(reply.get(3).asDouble() <= SAFETY_THR)
	{
		danglingPoint.push_back(SAFETY_Z);
		val = 1;
	}
	else
	{
		danglingPoint.push_back(reply.get(3).asDouble());
		val = 0;
	}

	this->pointRecorded.signal();
	state = idle;
	return val;
}

int Demo1Module::Label(const string & _label)
{
	// check if there is a point waiting for labelling
	if(danglingPoint.size() !=3 ) return -1;

	// check that an element with this label does not alreay exist
	int exists = Memory.count(_label);
	if (exists == 1)	return -2;
	
	Memory[_label] = danglingPoint;
	cout << "Point " << (Memory.find(_label)->second)[0];
	cout << " " << (Memory.find(_label)->second)[1] ;
	cout << " " << (Memory.find(_label)->second)[2] << " saved under name " << Memory.find(_label)->first << endl;

	danglingPoint.clear();

	return 0;
}

int Demo1Module::Forget(const string & _label)
{
	if (_label == "all")
	{
		Memory.clear();
		return 0;
	}

	// check that an element with this label does not alreay exist
	int exists = Memory.count(_label);
	if (exists == 0)	return -1;
	
	Memory.erase(_label);
	return 0;
}

Bottle Demo1Module::Recall()
{
	return map2Bottle(Memory);
}

void Demo1Module::run()
{
	// check if a blocking command was send to motors due to a touch event
	Bottle * inputR = fromTouchR.read(false);
	Bottle * inputL = fromTouchL.read(false);
				
	// was right
	if ( (inputR != NULL && activeChain == "right") ||
		 (inputL != NULL && activeChain == "left" )  )
	{
		Bottle reply;
		Bottle msg("stop");
		toDevDriver.write(msg,reply);
		toFace.write(sad,reply);

		cout << "I hitted something!!" << endl;
		this->suspend();
	}
}

Bottle Demo1Module::map2Bottle(map< string,vector<double> > Map)
{
	Bottle b;
	map< string,vector<double> >::iterator it;

	for (it=Map.begin(); it!=Map.end(); it++)
	{
		b.addString((*it).first.c_str());
		Bottle & L = b.addList();
		L.addDouble( ((*it).second)[0] );
		L.addDouble( ((*it).second)[1] );
		L.addDouble( ((*it).second)[2] );
	}
	return b;
}

int Demo1Module::add2map(const string & label, const double & x, const double & y, const double & z)
{
	// check that an element with this label does not alreay exist
	int exists = Memory.count(label);
	if (exists == 1)	return -2;
	
	vector< double > point(3);
	point[0] = x;
	point[1] = y;
	int val;
	if(z <= SAFETY_THR)
	{
		point[2] = SAFETY_Z;
		val = 1;
	}
	else
	{
		point[2] = z;
		val = 0;
	}

	Memory[label] = point;
	
	return val;
}



/* bimanuali*/

int Demo1Module::initBimanualReach(const string & _label1, const string & _label2)
{
	int exists = Memory.count(_label1) + Memory.count(_label2);
	if (exists != 2)	return -1;

	Vector point(3); 
	point(0) = (Memory.find(_label1)->second)[0];
	point(1) = (Memory.find(_label1)->second)[1];
	point(2) = (Memory.find(_label1)->second)[2];

	Matrix tg(2,3);
	if (_label1 == _label2)
	{
		tg(0,0) = point(0) + 0.04;
		tg(0,1) = point(1) + Width*0.5 + 0.03;
		tg(0,2) = point(2) - Height * 0.3;

		tg(1,0) = point(0) + 0.04;
		tg(1,1) = point(1) - Width*0.5 - 0.03;
		tg(1,2) = point(2) - Height * 0.3;

		return BimanualReach(tg, "side", "side", false, false, true);
	}
	else
	{
		tg(0,0) = point(0);
		tg(0,1) = point(1);
		tg(0,2) = point(2);

		point(0) = (Memory.find(_label2)->second)[0];
		point(1) = (Memory.find(_label2)->second)[1];
		point(2) = (Memory.find(_label2)->second)[2];

		tg(1,0) = point(0);
		tg(1,1) = point(1);
		tg(1,2) = point(2);
		return BimanualReach(tg);
	}
}

int Demo1Module::BimanualReach(Matrix tg, 	const string & _mode1,   const string & _mode2,
								bool indexR, bool indexL,	bool touchActive)
{
	Bottle cmd, reply;

	state = reaching;

	while(!hasFinished()) Time::delay(0.05);

	// generate the command stream for reaching and save the active chain for grasping
	if (indexR && indexL)	reach2hands.SetIndexActive(true);
	else if(indexR)			reach2hands.SetIndexActive("right");
	else if(indexL)			reach2hands.SetIndexActive("left");
	else					
	{
		reach2hands.SetIndexActive(false);
		cmd.clear();

		cmd.append("grasp bimanualStart right");
		toGrasper.write(cmd, reply);

		cmd.clear();
		cmd.append("grasp bimanualStart left");
		toGrasper.write(cmd, reply);

		cmd.clear();
		reply.clear();
	}

	Bottle stream = reach2hands.activate(tg);
	string side = reach2hands.getFirstHand();

	if (side == "right")
	{
		if(!setWrist(_mode1,"right")) return -9;
		if(!setWrist(_mode2,"left"))  return -9;
	}
	else
	{
		if(!setWrist(_mode1,"left"))  return -9;
		if(!setWrist(_mode2,"right")) return -9;
	}
	
	cout << "stream is: " << endl;
	cout << stream.toString() << endl;

	Bottle fingersR("(all)");
	Bottle fingersL("(all)");
	
	// always disable all fingers first
	cmd.append("disable touch left (all)");
	toDevDriver.write(cmd,reply);
	if (reply.get(0).asString() != "Done")	return -8;
	cmd.clear();
	reply.clear();
	cmd.append("disable touch right (all)");	
	toDevDriver.write(cmd,reply);
	if (reply.get(0).asString() != "Done")	return -8;

	if (indexR)
	{
		cmd.clear();
		reply.clear();
		cmd.append("grasp point right");
		toGrasper.write(cmd, reply);
		if (reply.get(0).asInt() != 0)	return -8;
		
		fingersR.clear();
		fingersR.append("(index)");
	}
	if (indexL)
	{
		cmd.clear();
		reply.clear();
		cmd.append("grasp point left");
		toGrasper.write(cmd, reply);
		if (reply.get(0).asInt() != 0)	return -8;
		fingersL.clear();
		fingersL.append("(index)");
	}

	while(!hasFinished()) Time::delay(0.05);

	if(touchActive)
	{
		cout << "touch active" << endl;
		cmd.clear();
		reply.clear();
		cmd.append("enable touch right");
		cmd.append(fingersR);

		toDevDriver.write(cmd,reply);
		if (reply.get(0).asString() != "Done")	return -8;

		cmd.clear();
		reply.clear();
		cmd.append("enable touch left");
		cmd.append(fingersL);

		toDevDriver.write(cmd,reply);
		if (reply.get(0).asString() != "Done")	return -8;
	}

	while(!hasFinished()) Time::delay(0.05);

	int result = sendPMPstream(*(stream.get(0).asList()), *(stream.get(1).asList()));

	state = idle;
	return result;
}

int Demo1Module::BimanualRelease()
{
	while(!hasFinished()) Time::delay(0.05);

	Bottle pose = getHandsPose();
	if(pose.size() != 6)		return -2;

	Matrix tg(2,3);
	tg(0,0) = pose.get(0).asDouble();
	tg(0,1) = pose.get(1).asDouble()+0.1;
	tg(0,2) = pose.get(2).asDouble();

	tg(1,0) = pose.get(3).asDouble();
	tg(1,1) = pose.get(4).asDouble()-0.1;
	tg(1,2) = pose.get(5).asDouble();

	Bottle cmd, reply;

	cmd.append("grasp bimanualStart right");
	toGrasper.write(cmd, reply);

	cmd.clear();
	cmd.append("grasp bimanualStart left");
	toGrasper.write(cmd, reply);

	int done = BimanualReach(tg, "side", "side", false, false, false);
	while(!hasFinished()) Time::delay(0.05);

	if (done != 0) return done;

	done = Relax();

	return done;
}


int Demo1Module::BimanualGrasp()
{
	Bottle cmd, reply;
	while(!hasFinished()) Time::delay(0.05);

	cmd.append("grasp bimanual right");
	toGrasper.write(cmd, reply);
	if (reply.get(0).asInt() != 0)  return reply.get(0).asInt();
	reply.clear();
	cmd.clear();
	cmd.append("grasp bimanual left");
	if (reply.get(0).asInt() != 0)  return reply.get(0).asInt();

	return 0;
}

int Demo1Module::BimanualLift(bool keepContact)
{
	Bottle pose = getHandsPose();
	if(pose.size() != 6)		return -2;

	Matrix tg(2,3);
	tg(0,0) = pose.get(0).asDouble();
	tg(0,1) = pose.get(1).asDouble();
	tg(0,2) = pose.get(2).asDouble() + 0.1;

	tg(1,0) = pose.get(3).asDouble();
	tg(1,1) = pose.get(4).asDouble();
	tg(1,2) = pose.get(5).asDouble() + 0.1;

	int done = BimanualReach(tg, "side", "side", false, false, false);
	if (done != 0) return done;
	
	if (keepContact)	
	{
		bool ok = contactCheck();
		if (!ok) return -10;
	}

	while(!hasFinished()) Time::delay(0.05);
	return done;
}
// initUp, reach, grasp, lift, carry to target, release, relax
int Demo1Module::BimanualCarry(const std::string &_label1, const std::string &_tgLabel, bool keepContact)
{
	// check if the object exists in memory
	int exists = Memory.count(_label1) + Memory.count(_tgLabel);
	if (exists != 2)	return -1;

	Vector point(3); 
	point(0) = (Memory.find(_label1)->second)[0];
	point(1) = (Memory.find(_label1)->second)[1];
	point(2) = (Memory.find(_label1)->second)[2];

	// set target position for two hands

	Matrix tg(2,3);
	tg(0,0) = point(0) + 0.02;
	tg(0,1) = point(1) + Width*0.5 + 0.01;
	tg(0,2) = point(2) - Height * 0.3;

	tg(1,0) = point(0) + 0.02;
	tg(1,1) = point(1) - Width*0.5 + 0.01;
	tg(1,2) = point(2) - Height * 0.3;

	// Initicub
	if (initIcubUp() != 0) return -8;

	//reach the targets
	int done = BimanualReach(tg, "side", "side", false, false, true);
	if (done != 0) return done;

	// grasp
	while(!hasFinished()) Time::delay(0.05);
	Bottle cmd, reply;
	
	done = BimanualGrasp();
	if (done != 0) return done;

	while(!hasFinished()) Time::delay(0.05);

	// lift the object: check actual pose and give a 20 cm higer target
	Bottle pose = getHandsPose();
	if(pose.size() != 6)		return -2;

	tg(0,0) = pose.get(0).asDouble();
	tg(0,1) = pose.get(1).asDouble();
	tg(0,2) = pose.get(2).asDouble() + 0.1;

	tg(1,0) = pose.get(3).asDouble();
	tg(1,1) = pose.get(4).asDouble();
	tg(1,2) = pose.get(5).asDouble() + 0.1;

	done = BimanualReach(tg, "side", "side", false, false, false);
	if (done != 0) return done;
	
	if (keepContact)	
	{
		bool ok = contactCheck();
		if (!ok) return -10;
	}

	// carry the object to the target point 
	point(0) = (Memory.find(_tgLabel)->second)[0];
	point(1) = (Memory.find(_tgLabel)->second)[1];
	point(2) = (Memory.find(_tgLabel)->second)[2];

	// set target position for two hands
	tg(0,0) = point(0);
	tg(0,1) = point(1);
	tg(0,2) = point(2) + Height * 0.6;

	tg(1,0) = point(0);
	tg(1,1) = point(1);
	tg(1,2) = point(2) + Height * 0.6;

	done = BimanualReach(tg, "side", "side", false, false, false);
	if (done != 0) return done;
	
	if (keepContact)	
	{
		bool ok = contactCheck();
		if (!ok) return -10;
	}

	// release the object and go away from it
	Release("right");
	Release("left");
	
	pose = getHandsPose();
	if(pose.size() != 6)		return -2;

	tg(0,0) = pose.get(0).asDouble();
	tg(0,1) = pose.get(1).asDouble() + 0.1;
	tg(0,2) = pose.get(2).asDouble() + 0.15;

	tg(1,0) = pose.get(3).asDouble();
	tg(1,1) = pose.get(4).asDouble() - 0.1;
	tg(1,2) = pose.get(5).asDouble() + 0.15;

	done = BimanualReach(tg, "side", "side", false, false, false);
	if (done != 0) return done;
	
	// Initicub
	if (initIcubUp() != 0) return -8;

	return 0;
}

void Demo1Module::setObjectGeometry(const double & W, const double & H)
{
	Width = W;
	Height = H;
}

Bottle Demo1Module::getObjectGeometry()
{
	Bottle geo;
	geo.addDouble(Width);
	geo.addDouble(Height);
	return geo;
}