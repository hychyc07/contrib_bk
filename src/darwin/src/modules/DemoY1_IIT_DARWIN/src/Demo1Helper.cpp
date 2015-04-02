
#include "darwin/Demo1Helper.h"

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace Darwin::Demo1;

/*---------------- PMP CMD STREAMERS -------------------*/



/*----------------- HELPER ----------------------*/
Demo1Helper::Demo1Helper()
{
	// transform matrix from head to torso coordinate frame
	HTmatrix.resize(4,4);
	HTmatrix.eye();
	eye = "left";
}

Bottle Demo1Helper::Initialmsg()
{
	Bottle msg;
	msg.append("-- Hello! --");
	msg.append("-- I'm ready, what you want me to do?");
	return msg;
}

Bottle Demo1Helper::idleMsg()
{
	Bottle msg;
	msg.append("-- What you want me to do now?");
	return msg;
}

bool Demo1Helper::updateHTmatrix()
{
	Bottle cmd, reply;
	cmd.addString("get torso");
	HTtransform.write(cmd, reply);

	if (reply.size() != 2)	 return false;

	Bottle *T ;
	if (eye == "right")	T = reply.get(0).asList();
	if (eye == "left")	T = reply.get(1).asList();

	if(T->size() == HTmatrix.rows()*HTmatrix.cols())
	{
		for (int i=0; i<T->size(); i++)
		{
			*(HTmatrix.data()+i) = T->get(i).asDouble();
		}
	}
	else
		return false;

	cout << "HT matrix: " << endl;
	cout << HTmatrix.getRow(0).toString() << endl;
	cout << HTmatrix.getRow(1).toString() << endl;
	cout << HTmatrix.getRow(2).toString() << endl;
	cout << HTmatrix.getRow(3).toString() << endl << endl;

	return true;
}

bool Demo1Helper::activateVision3D()
{
	// get 3d points of interest on the object in eye reference frame;
	Bottle cmd, reply;
	cmd.addString("capture");
	targetsIn.write(cmd, reply);

	if(reply.size()==0)	return false;
	Matrix points = Bottle2Matrix(reply);

	// get 3d points in torso reference frame:
	for (int i=0; i<points.rows(); i++)
		points.setRow(i, HTmatrix * points.getRow(i) );

	// select the topmost point:
	double z = points(0,2);
	int row = 0;
	for(int i=1; i<points.rows(); i++)
	{
		if(points(i,2) > z)
		{
			z=points(i,2);
			row = i;
		}
	}

	target = (points.getRow(row)).subVector(0,2);
	Bottle b;
	b.addString("-- I've found the topmost object: target point is ");
	b.addDouble(target(0));
	b.addDouble(target(1));
	b.addDouble(target(2));

	msg.append(b);
	return true;
}

bool Demo1Helper::reach()
{
	Bottle cmd,reply;
	Bottle &list = cmd.addList();
	list.addDouble(target(0));
	list.addDouble(target(1));
	list.addDouble(target(2));

	// send the target to connection module:
	// check that outport is connected:
	if (targetOut.getOutputCount() == 0) 
	{
		cout << "target output port not connected" << endl;
		return false;
	}
	targetOut.write(cmd);

	do
	{
		Time::delay(1);
		Bottle ack;
		ack.addString("ready");
		targetOut.write(ack,reply);
	}
	while(reply.get(0).asString() == "yes");

	msg.append("-- I have reached the object");
	return true;
}

bool Demo1Helper::setCompliantArm(string side)
{
	Bottle cmd, reply;
	cmd.addString("compliant");
	cmd.addString(side.c_str());

	driverOut.write(cmd, reply);
	if (reply == "Done")	return true;
	else					return false;
}

bool Demo1Helper::setStiffArm(string side)
{
	Bottle cmd, reply;
	cmd.addString("stiff");
	cmd.addString(side.c_str());

	driverOut.write(cmd, reply);
	if (reply == "Done")	return true;
	else					return false;
}

bool Demo1Helper::grasp(bool compliant)
{
	// go will be true if grasp is compliant: after a loose grasp, when go will become true,
	// it will execute the complete tight grasp
	Bottle cmd, side, reply;

	if(compliant)
	{
		if(go == 0)
		{
			// find the active pmp chain:
			
			cmd.addString("getActiveChain");
			pmpOut.write(cmd,side);

			this->side = side.get(0).asString();
			
			// set compliant mode for the arm:
			if(!setCompliantArm(side.get(0).asString().c_str()))
			{
				msg.append("I Couldn't set compliant arm");
				return false;
			}
			
			cmd.clear();
			cmd.addString("grasp");
			cmd.addString("dloose");
			cmd.append(side);
			graspCmd.write(cmd,reply);

			if (reply != "done") return false;

			msg.append("-- Can I proceed? (go grasp)");
			return true;
			
		}

		if (go == 1)
		{ 			
			cmd.clear();
			cmd.addString("grasp");
			cmd.addString("dmiddle");
			cmd.append(side);
			graspCmd.write(cmd,reply);

			if (reply != "done") return false;

			msg.append("-- Can I proceed? (go grasp)");
			return true;
		}

		if (go == 2)
		{
			cmd.clear();
			cmd.addString("grasp");
			cmd.addString("dtight");
			cmd.append(side);
			graspCmd.write(cmd,reply);

			if (reply != "done") return false;
		}

		else
		{
			msg.append("-- I have already grasped as tight as possible!");
			return false;
		}
	}// end if (compliant)	

	else // not compliant, execute one-shot:
	{
		// find the active pmp chain:
		Bottle cmd, side, reply;
		cmd.addString("getActiveChain");
		pmpOut.write(cmd,side);

		// grasp loose:
		cmd.clear();
		cmd.addString("grasp");
		cmd.addString("dloose");
		cmd.append(side);
		graspCmd.write(cmd,reply);

		if (reply != "done") return false;

		// grasp middle:
		Time::delay(0.5);
		cmd.clear();
		cmd.addString("grasp");
		cmd.addString("dmiddle");
		cmd.append(side);
		graspCmd.write(cmd,reply);

		if (reply != "done") return false;

		// grasp tight:
		else			Time::delay(0.5);

		cmd.clear();
		cmd.addString("grasp");
		cmd.addString("dtight");
		cmd.append(side);
		graspCmd.write(cmd,reply);

		if (reply != "done") return false;
	}

	msg.append("-- I think I grasped the object, am I right?");

	return true;
}

bool Demo1Helper::updatePMPstate()
{
	// update PMP internal current angles:
	Bottle pmpCmd, pmpReply;
	pmpCmd.addString("update_angles");
	pmpOut.write(pmpCmd,pmpReply);

	if(pmpReply == "Done")	
	{
		msg.append("-- I have updated my curret position");
		return true;
	}
	else
	{
		msg.append("-- Sorry, I couldn't update my current position");
		return false;
	}
}

bool Demo1Helper::release()
{
	Bottle cmd, side, reply;

	cmd.addString("getActiveChain");
	pmpOut.write(cmd,side);

	cmd.clear();
	cmd.addString("release");
	cmd.append(side);

	graspCmd.write(cmd,reply);

	msg.append(reply);
	return true;
}

bool Demo1Helper::lift()
{
	Bottle cmd, reply;
	Vector target(3);

	if (this->side.length() == 0)
	{
		Bottle cmd1, side;

		cmd1.addString("getActiveChain");
		pmpOut.write(cmd1,side);
		this->side = side.get(0).asString();
	}

	// get current position:
	Bottle cmd2, pose;
	cmd2.addString("PMP_get");
	cmd2.addString("pose");
	pmpOut.write(cmd2,pose);

	// pose = xr yr zr xl yl zl
	// fill in the target and add 10 cm to z coordinate
	if(pose.size() == 6)
	{
		int j = 0;
		if(this->side == "left") j = 3;
		target(0) = pose.get(0+j).asDouble();
		target(1) = pose.get(1+j).asDouble();
		target(2) = pose.get(2+j).asDouble()+0.1;

		cout << target.toString() << endl;
	}
	else
	{	
		msg.append("-- Sorry, I couldn't get the EE position");
		return false;
	}

	if (this->side == "right")		cmd.addString("VTGSright_set");
	if (this->side == "left")		cmd.addString("VTGSleft_set");

	cmd.addString("target");
	cmd.addInt(1);
	cmd.addDouble(target(0));
	cmd.addDouble(target(1));
	cmd.addDouble(target(2));
	cout << cmd.toString() << endl;

	pmpOut.write(cmd,reply);
	msg.append(reply);

	return true;
}

Vector Demo1Helper::Bottle2Vector(Bottle Bot)
{
	Vector v(Bot.size());
	for (int i = 0; i < Bot.size(); i++)
	{
		v(i) = Bot.get(i).asDouble();
	}

	return v;
}

Matrix Demo1Helper::Bottle2Matrix(Bottle Bot)
{
	Matrix m;
	Bottle *temp = Bot.get(0).asList();
	m.resize(Bot.size(), temp->size());
	m.setRow(0,Bottle2Vector(*temp));
	for (int i = 1; i < Bot.size(); i++)
	{
		temp = Bot.get(i).asList();
		m.setRow(i,Bottle2Vector(*temp));
	}

	return m;
}

/*
Demo1Helper::Demo1Helper()
{
	// transform matrix from head to torso coordinate frame
	HTmatrix.resize(4,4);
	HTmatrix.eye();
	eye = "left";
}

Bottle Demo1Helper::Initialmsg()
{
	Bottle msg;
	msg.append("-- Hello! --");
	msg.append("-- I'm ready, what you want me to do?");
	return msg;
}

Bottle Demo1Helper::idleMsg()
{
	Bottle msg;
	msg.append("-- What you want me to do now?");
	return msg;
}

bool Demo1Helper::updateHTmatrix()
{
	Bottle cmd, reply;
	cmd.addString("get torso");
	HTtransform.write(cmd, reply);

	if (reply.size() != 2)	 return false;

	Bottle *T ;
	if (eye == "right")	T = reply.get(0).asList();
	if (eye == "left")	T = reply.get(1).asList();

	if(T->size() == HTmatrix.rows()*HTmatrix.cols())
	{
		for (int i=0; i<T->size(); i++)
		{
			*(HTmatrix.data()+i) = T->get(i).asDouble();
		}
	}
	else
		return false;

	cout << "HT matrix: " << endl;
	cout << HTmatrix.getRow(0).toString() << endl;
	cout << HTmatrix.getRow(1).toString() << endl;
	cout << HTmatrix.getRow(2).toString() << endl;
	cout << HTmatrix.getRow(3).toString() << endl << endl;

	return true;
}

bool Demo1Helper::activateVision3D()
{
	// get 3d points of interest on the object in eye reference frame;
	Bottle cmd, reply;
	cmd.addString("capture");
	targetsIn.write(cmd, reply);

	if(reply.size()==0)	return false;
	Matrix points = Bottle2Matrix(reply);

	// get 3d points in torso reference frame:
	for (int i=0; i<points.rows(); i++)
		points.setRow(i, HTmatrix * points.getRow(i) );

	// select the topmost point:
	double z = points(0,2);
	int row = 0;
	for(int i=1; i<points.rows(); i++)
	{
		if(points(i,2) > z)
		{
			z=points(i,2);
			row = i;
		}
	}

	target = (points.getRow(row)).subVector(0,2);
	Bottle b;
	b.addString("-- I've found the topmost object: target point is ");
	b.addDouble(target(0));
	b.addDouble(target(1));
	b.addDouble(target(2));

	msg.append(b);
	return true;
}

bool Demo1Helper::reach()
{
	Bottle cmd,reply;
	Bottle &list = cmd.addList();
	list.addDouble(target(0));
	list.addDouble(target(1));
	list.addDouble(target(2));

	// send the target to connection module:
	// check that outport is connected:
	if (targetOut.getOutputCount() == 0) 
	{
		cout << "target output port not connected" << endl;
		return false;
	}
	targetOut.write(cmd);

	do
	{
		Time::delay(1);
		Bottle ack;
		ack.addString("ready");
		targetOut.write(ack,reply);
	}
	while(reply.get(0).asString() == "yes");

	msg.append("-- I have reached the object");
	return true;
}

bool Demo1Helper::setCompliantArm(string side)
{
	Bottle cmd, reply;
	cmd.addString("compliant");
	cmd.addString(side.c_str());

	driverOut.write(cmd, reply);
	if (reply == "Done")	return true;
	else					return false;
}

bool Demo1Helper::setStiffArm(string side)
{
	Bottle cmd, reply;
	cmd.addString("stiff");
	cmd.addString(side.c_str());

	driverOut.write(cmd, reply);
	if (reply == "Done")	return true;
	else					return false;
}

bool Demo1Helper::grasp(bool compliant)
{
	// go will be true if grasp is compliant: after a loose grasp, when go will become true,
	// it will execute the complete tight grasp
	Bottle cmd, side, reply;

	if(compliant)
	{
		if(go == 0)
		{
			// find the active pmp chain:
			
			cmd.addString("getActiveChain");
			pmpOut.write(cmd,side);

			this->side = side.get(0).asString();
			
			// set compliant mode for the arm:
			if(!setCompliantArm(side.get(0).asString().c_str()))
			{
				msg.append("I Couldn't set compliant arm");
				return false;
			}
			
			cmd.clear();
			cmd.addString("grasp");
			cmd.addString("dloose");
			cmd.append(side);
			graspCmd.write(cmd,reply);

			if (reply != "done") return false;

			msg.append("-- Can I proceed? (go grasp)");
			return true;
			
		}

		if (go == 1)
		{ 			
			cmd.clear();
			cmd.addString("grasp");
			cmd.addString("dmiddle");
			cmd.append(side);
			graspCmd.write(cmd,reply);

			if (reply != "done") return false;

			msg.append("-- Can I proceed? (go grasp)");
			return true;
		}

		if (go == 2)
		{
			cmd.clear();
			cmd.addString("grasp");
			cmd.addString("dtight");
			cmd.append(side);
			graspCmd.write(cmd,reply);

			if (reply != "done") return false;
		}

		else
		{
			msg.append("-- I have already grasped as tight as possible!");
			return false;
		}
	}// end if (compliant)	

	else // not compliant, execute one-shot:
	{
		// find the active pmp chain:
		Bottle cmd, side, reply;
		cmd.addString("getActiveChain");
		pmpOut.write(cmd,side);

		// grasp loose:
		cmd.clear();
		cmd.addString("grasp");
		cmd.addString("dloose");
		cmd.append(side);
		graspCmd.write(cmd,reply);

		if (reply != "done") return false;

		// grasp middle:
		Time::delay(0.5);
		cmd.clear();
		cmd.addString("grasp");
		cmd.addString("dmiddle");
		cmd.append(side);
		graspCmd.write(cmd,reply);

		if (reply != "done") return false;

		// grasp tight:
		else			Time::delay(0.5);

		cmd.clear();
		cmd.addString("grasp");
		cmd.addString("dtight");
		cmd.append(side);
		graspCmd.write(cmd,reply);

		if (reply != "done") return false;
	}

	msg.append("-- I think I grasped the object, am I right?");

	return true;
}

bool Demo1Helper::updatePMPstate()
{
	// update PMP internal current angles:
	Bottle pmpCmd, pmpReply;
	pmpCmd.addString("update_angles");
	pmpOut.write(pmpCmd,pmpReply);

	if(pmpReply == "Done")	
	{
		msg.append("-- I have updated my curret position");
		return true;
	}
	else
	{
		msg.append("-- Sorry, I couldn't update my current position");
		return false;
	}
}

bool Demo1Helper::release()
{
	Bottle cmd, side, reply;

	cmd.addString("getActiveChain");
	pmpOut.write(cmd,side);

	cmd.clear();
	cmd.addString("release");
	cmd.append(side);

	graspCmd.write(cmd,reply);

	msg.append(reply);
	return true;
}

bool Demo1Helper::lift()
{
	Bottle cmd, reply;
	Vector target(3);

	if (this->side.length() == 0)
	{
		Bottle cmd1, side;

		cmd1.addString("getActiveChain");
		pmpOut.write(cmd1,side);
		this->side = side.get(0).asString();
	}

	// get current position:
	Bottle cmd2, pose;
	cmd2.addString("PMP_get");
	cmd2.addString("pose");
	pmpOut.write(cmd2,pose);

	// pose = xr yr zr xl yl zl
	// fill in the target and add 10 cm to z coordinate
	if(pose.size() == 6)
	{
		int j = 0;
		if(this->side == "left") j = 3;
		target(0) = pose.get(0+j).asDouble();
		target(1) = pose.get(1+j).asDouble();
		target(2) = pose.get(2+j).asDouble()+0.1;

		cout << target.toString() << endl;
	}
	else
	{	
		msg.append("-- Sorry, I couldn't get the EE position");
		return false;
	}

	if (this->side == "right")		cmd.addString("VTGSright_set");
	if (this->side == "left")		cmd.addString("VTGSleft_set");

	cmd.addString("target");
	cmd.addInt(1);
	cmd.addDouble(target(0));
	cmd.addDouble(target(1));
	cmd.addDouble(target(2));
	cout << cmd.toString() << endl;

	pmpOut.write(cmd,reply);
	msg.append(reply);

	return true;
}

Vector Demo1Helper::Bottle2Vector(Bottle Bot)
{
	Vector v(Bot.size());
	for (int i = 0; i < Bot.size(); i++)
	{
		v(i) = Bot.get(i).asDouble();
	}

	return v;
}

Matrix Demo1Helper::Bottle2Matrix(Bottle Bot)
{
	Matrix m;
	Bottle *temp = Bot.get(0).asList();
	m.resize(Bot.size(), temp->size());
	m.setRow(0,Bottle2Vector(*temp));
	for (int i = 1; i < Bot.size(); i++)
	{
		temp = Bot.get(i).asList();
		m.setRow(i,Bottle2Vector(*temp));
	}

	return m;
}
*/
