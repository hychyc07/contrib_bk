
#include "darwin/Grasp.h"
#include <yarp/os/ConstString.h>
#include <math.h>

using namespace Darwin::grasper;

//#define round(x) ((x-floor(x))>0.5 ? ceil(x) : floor(x))

Vector Darwin::grasper::Bottle2Vector(Bottle Bot)
{
	Vector v(Bot.size());
	for (int i = 0; i < Bot.size(); i++)
	{
		v(i) = Bot.get(i).asDouble();
	}

	return v;
}

Bottle Darwin::grasper::Vector2Bottle(Vector v)
{
	Bottle b;
	for (unsigned int i = 0; i < v.size(); i++)
	{
		b.addDouble(v(i));
	}

	return b;
}


Grasper::Grasper(string portName)
{
	this->portName = portName;
}

Grasper::~Grasper()
{	
}
 
bool Grasper::StartPorts()
{
	if( !cmdOut.open(portName.c_str()) )
	{
		printf("Error: unable to open ports, closing\n");
		return false;
	}
	cmdOut.setTimeout(10.0);

	return true;
}

bool Grasper::InterruptPorts()
{
	cmdOut.interrupt();
	return true;
}

bool Grasper::ClosePorts()
{
	cmdOut.close();
	return true;
}

bool Grasper::sendACK()
{
	Bottle cmd;
	Bottle* reply;
	reply = new Bottle();
	
	cmd.addString("ACK");
	cmdOut.write(cmd,*reply);

	if (reply->size() == 0)
	{
		cout << "Error: no output connection" << endl;
		return false;
	}
	return true;
}

void Grasper::setGraspSpeeds(const Bottle & _speeds)
{
	speeds.copy(_speeds);
	cout << "Speeds set to: " << endl << speeds.toString() << endl;
}


Bottle Grasper::getGraspSpeeds()
{
	return speeds;
}

void Grasper::setOffset(const Bottle & _offset)
{
	offset.copy(_offset);
	cout << "Offsets set to: " << endl << offset.toString() << endl;
}


Bottle Grasper::getOffset()
{
	return offset;
}

Bottle Grasper::Tighten(string side)
{
	Bottle msg;
	
	int Nconnections = cmdOut.getOutputCount();
	if (Nconnections == 0)
	{
		msg.addInt(-4);
		msg.addString("No output connections available!");
		return msg;
	}

	if (!sendACK())
	{
		msg.addInt(-4);
		msg.addString("No active output connection available!");
		return msg;
	}
		
	if ( !( (side == "right") || (side == "left") ) )
	{
		msg.addInt(-6);
		msg.addString(("No side named "+side+" found!").c_str());
		return msg;
	}

	// build the grasping sequence:
	// 1- disable touch events detection
	Bottle graspCmd, reply, temp;

	graspCmd.addString("disable");
	graspCmd.addString("touch");
	graspCmd.addString(side.c_str());
	Bottle &L = graspCmd.addList();
	L.addString("all");

	cmdOut.write(graspCmd,reply);
	if(reply.get(0).asString()!= "Done")
	{
		msg.addInt(-8);
		msg.addString("Sensors not disabled!");
		msg.append(reply);
		return msg;
	}

	// 2- send the grasping command
	temp = getTightenAngles(side);
	if (temp.size() == 1 && temp.get(0).asInt() == -3)
		return temp;

	graspCmd.clear();
	reply.clear();
	graspCmd.addString("grasp");
	Bottle & ang = graspCmd.addList();
	ang.append(temp);	
	graspCmd.addString(side.c_str());

	cmdOut.write(graspCmd,reply);
	if (reply.get(0).asString() != "Done")
	{
		msg.addInt(-7);
		msg.append(reply);
	}
	else
	{
		msg.clear();
		msg.addInt(0);
	}

	return msg;
}

Bottle Grasper::Grasp(Bottle primitives, string type, string side, bool sensitive)
{
	// send an ack to see if connected
	Bottle msg;
	
	int Nconnections = cmdOut.getOutputCount();
	if (Nconnections == 0)
	{
		msg.addInt(-4);
		msg.addString("No output connections available!");
		return msg;
	}

	if (!sendACK())
	{
		msg.addInt(-4);
		msg.addString("No active output connection available!");
		return msg;
	}

	if( !primitives.check(type.c_str()) )
	{		
		msg.addInt(-5);
		msg.addString(("No primitive named "+type+" found!").c_str());
		return msg;
	}	

	if ( !( (side == "right") || (side == "left") ) )
	{
		msg.addInt(-6);
		msg.addString(("No side named "+side+" found!").c_str());
		return msg;
	}

	Bottle * angles = primitives.find(type.c_str()).asList();
	cout << angles->toString() << endl;	

	// build the grasping sequence:
	Bottle graspCmd;

	if(sensitive)
	{
		// be sure that touch is enabled
		Bottle reply;
		graspCmd.addString("enable");
		graspCmd.addString("touch");
		graspCmd.addString(side.c_str());
		Bottle &L = graspCmd.addList();
		L.addString("all");

		cmdOut.write(graspCmd,reply);
		if(reply.get(0).asString()!= "Done")
		{
			msg.addInt(-8);
			msg.addString("Sensors not enabled!");
			msg.append(reply);
			return msg;
		}

		// send the command 'safeGrasp (joint angles 7-15) (joint velocities) side'
		graspCmd.clear();
		reply.clear();
		graspCmd.addString("safeGrasp");
		Bottle &ang = graspCmd.addList();
		Bottle &vel = graspCmd.addList();
		ang.append(*angles);
		vel.append(speeds);
	}
	else
	{
		// disable touch first, then grasp
		Bottle reply;
		graspCmd.addString("disable");
		graspCmd.addString("touch");
		graspCmd.addString(side.c_str());
		Bottle &L = graspCmd.addList();
		L.addString("all");

		cmdOut.write(graspCmd,reply);
		if(reply.get(0).asString()!= "Done")
		{
			msg.addInt(-8);
			msg.addString("Sensors not disabled!");
			msg.append(reply);
			return msg;
		}
		// send the command 'grasp (joint angles 7-15) side'
		graspCmd.clear();
		reply.clear();
		graspCmd.addString("grasp");
		Bottle &ang = graspCmd.addList();
		ang.append(*angles);
	}

	Bottle reply;
	graspCmd.addString(side.c_str());
	cmdOut.write(graspCmd,reply);

	if (reply.get(0).asString() != "Done")
	{
		msg.addInt(-7);
		msg.append(reply);
	}
	else
	{
		msg.clear();
		msg.addInt(0);
	}
	
	return msg;
}
bool Grasper::graspSucceeded()
{
	if (cmdOut.getOutputCount() == 0)
		return true;

	Bottle msg,reply;
	msg.addString("checkGraspFinished");

	cmdOut.write(msg,reply);

	if(reply.get(0).asString() == "true") return true;
	else								  return false;
}

bool Grasper::hasFinished()
{
	if (cmdOut.getOutputCount() == 0)
		return true;

	Bottle msg,reply;
	msg.addString("checkMotionFinished");

	cmdOut.write(msg,reply);
	if(reply.get(0).asString() == "true") return true;
	else								  return false;
}

Bottle Grasper::getEncoders(string side)
{
	Bottle msg,reply;
	msg.addString("getEncoders");
	msg.addString((side+"Hand").c_str());

	cmdOut.write(msg,reply);
	return reply;
}


Bottle Grasper::getTightenAngles(string side)
{
	Bottle msg;
	Bottle enc = getEncoders(side);
	if (enc.size() == 9)
	{
		Vector act = Bottle2Vector(enc);
		Vector off = Bottle2Vector(offset);
		msg = Vector2Bottle(act+off);
	}
	else 
		msg.addInt(-3);

	return msg;
}

Bottle Grasper::Release(Bottle primitives, string side)
{
	Bottle msg;

	msg = Grasp(primitives,"home",side,false);

	return msg;
}

graspType Grasper::string2enum(string type)
{
	graspType gtype;
	if (type == "top") gtype = top;
	else if (type == "side") gtype = side;
	else if (type == "null") gtype = null;
	else gtype = null;

	return gtype;
}

string Grasper::enum2string(graspType type)
{
	string stype;
	if(type == top) stype = "top";
	else if(type == side) stype = "side";
	else if(type == null) stype = "null";
	else stype = "null";

	return stype;
}


