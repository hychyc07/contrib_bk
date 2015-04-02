#ifndef DEMO1_HELPER_H
#define DEMO1_HELPER_H

#include <iostream>
#include <string>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>
#include <yarp/os/Event.h>
#include <yarp/os/Time.h>

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

namespace Darwin{
namespace Demo1{

class Demo1Helper
{
public:
	Bottle msg;

	Port HTtransform;
	Port targetsIn;
	Port targetOut;
	Port graspCmd;
	Port driverOut;
	Port pmpOut;

	Matrix HTmatrix;
	Vector target;

	// to pace the grasping:
	int go;
	
	Demo1Helper();
	~Demo1Helper(){};

	Bottle Initialmsg();
	Bottle idleMsg();
	bool updateHTmatrix();
	bool activateVision3D();
	bool reach();
	bool setCompliantArm(string side);
	bool setStiffArm(string side);
	bool grasp(bool compliant = false);
	bool release();
	bool lift();
	bool updatePMPstate();

	string eye;
	string side;

private:
	Vector Bottle2Vector(Bottle Bot);
	Matrix Bottle2Matrix(Bottle Bot);
};

class PMPcmdStreamer
{
public:
	Bottle cmdStream;
	Bottle answStream;
	Vector * targetV;
	Matrix * targetM;
	bool isVector;
	bool isMatrix;
	
	virtual Bottle activate(Matrix tgM){return Bottle();};
	virtual Bottle activate(Vector tgV){return Bottle();};
	virtual void SetTarget()=0;

	bool SetActiveChain(string side)
	{
		if (side == "right")	{chain = "right"; hasChain = true;  return true;}
		if (side == "left")		{chain = "left";  hasChain = true;  return true;}
		if (side == "none")		{chain = "none";  hasChain = false; return true;}
		else					{chain = "none";  hasChain = false; return false;}
	};

	string GetActiveChain()
	{
		return chain;
	};

	void SetIndexActive(bool active)
	{
		useIndex = active;
		side = "bimanual";
		ignoreEE = false;
	};

	void SetIndexActive(const string & _side)
	{
		useIndex = true;
		side = _side;
		ignoreEE = false;
	};

	void IgnoreEE(bool ignore)
	{
		ignoreEE = ignore;
	};
	
	PMPcmdStreamer()
	{
		isVector=isMatrix=false; 
		chain = "none"; 
		useIndex = false; 
		ignoreEE = false;
		hasChain = false; 
	};

	~PMPcmdStreamer(){};

	void SetChain()
	{
		cmdStream.clear();
		answStream.clear();
		Bottle &list1 = cmdStream.addList();

		list1.addString("setActiveChain");

		if (isMatrix)
		{
			cout << "tg matrix piena" << endl;
			// select chain
			list1.addString("bimanual");
			answStream.addString("Done");
		}
		else if(isVector)
		{
			cout << "tg vector pieno" << endl;

			if (!hasChain)
			{
				// select chain and initialize the other hand
				if ((*targetV)(1) > 0)	{chain = "right"; list1.addString("right");}
				else					{chain = "left"; list1.addString("left"); }
			}
			answStream.addString("Done");	
/*			cout << "tg vector pieno" << endl;
			Bottle &list2 = cmdStream.addList();

			if (!hasChain)
			{
				// select chain and initialize the other hand
				if ((*targetV)(1) > 0)	{chain = "right"; list1.addString("right"); list2.addString("initLeftArm");}
				else					{chain = "left"; list1.addString("left");  list2.addString("initRightArm");}
			}
			else
			{
				list1.addString(chain.c_str());
				if (chain == "right")	list2.addString("initLeftArm");
				else					list2.addString("initRightArm");
			}

			answStream.addString("Done");
			answStream.addString("Done");	
*/
		}
		else
			cout << "error!" << endl;
	};

protected:
	
	bool hasChain;
	string chain;
	bool useIndex;
	string side;
	bool ignoreEE;
	virtual void streamCreator()=0;	
};


// cmd streamer for one hand reaching task
class OneHandStreamer: public PMPcmdStreamer
{

private:
	Vector point;

	void streamCreator()
	{
		SetChain();
		SetTarget();		
	};

public:

	OneHandStreamer(){};
	~OneHandStreamer(){};

	Bottle activate(Vector point)
	{
		this->targetV = new Vector(point);
		this->isVector = true;
		this->point = point;
		this->cmdStream.clear();

		streamCreator();

		Bottle out;
		Bottle & cmd  = out.addList();
		Bottle & answ = out.addList();

		cmd.append(cmdStream);
		answ.append(answStream);

		delete this->targetV;
		return out;
	};

	void SetTarget()
	{

		if(!ignoreEE)
		{
			if(useIndex)
			{
				Bottle &listI = cmdStream.addList();
				listI.addString("useIndex");
				listI.addString(chain.c_str());
				listI.addString("on");
				answStream.addString("Done");
			}
			else
			{
				Bottle &listI = cmdStream.addList();
				listI.addString("useIndex");
				listI.addString(chain.c_str());
				listI.addString("off");
				answStream.addString("Done");
			}
		}

		Bottle &list = cmdStream.addList();
		
		if (hasChain)
		{
			if (chain == "right")	list.addString("VTGSright_set");
			else					list.addString("VTGSleft_set");
		}
		else
		{
			if (point(1) > 0)		list.addString("VTGSright_set");
			else					list.addString("VTGSleft_set");
		}
		list.addString("target");
		//list.addInt(1);
		list.addDouble(point(0));
		list.addDouble(point(1));
		list.addDouble(point(2));

		answStream.addString("Done");
		
		cout << list.toString() << endl;
	};
};



// cmd streamer for one hand reaching task
class TwoHandStreamer: public PMPcmdStreamer
{

private:
	//PMPcmdStreamer streamer;
	Matrix points;
	bool rightIsFirst;

	void streamCreator()
	{
		SetChain();
		SetTarget();
	};

public:

	string getFirstHand()
	{
		if (rightIsFirst)	return "right";
		else				return "left";
	}
	
	//Bottle cmdStream;

	TwoHandStreamer(){rightIsFirst = true;};
	~TwoHandStreamer(){};

	Bottle activate(Matrix points)
	{
		this->targetM = new Matrix(points);
		this->isMatrix = true;
		this->points = points;
		this->cmdStream.clear();

		streamCreator();

		Bottle out;
		Bottle & cmd  = out.addList();
		Bottle & answ = out.addList();

		cmd.append(cmdStream);
		answ.append(answStream);

		delete this->targetM;
		return out;
	};

	void SetTarget()
	{
		if(!ignoreEE)
		{
			if(useIndex)
			{
				if (side == "right")
				{
					Bottle &listIr = cmdStream.addList();
					listIr.append("useIndex right on");
					answStream.addString("Done");
				
				}
				else if (side == "left")
				{
					Bottle &listIl = cmdStream.addList();
					listIl.append("useIndex left on");
					answStream.addString("Done");
				}
				else
				{
					Bottle &listIr = cmdStream.addList();
					Bottle &listIl = cmdStream.addList();
					listIr.append("useIndex right on");
					listIl.append("useIndex left on");
					answStream.addString("Done");
					answStream.addString("Done");
				}
			}
			else
			{
				Bottle &listIr = cmdStream.addList();
				Bottle &listIl = cmdStream.addList();
				listIr.append("useIndex right off");
				listIl.append("useIndex left off");
				answStream.addString("Done");
				answStream.addString("Done");
			}
		}

		Bottle &list1 = cmdStream.addList();
		Bottle &list2 = cmdStream.addList();

		list1.addString("VTGSright_set");
		list1.addString("target");

		list2.addString("VTGSleft_set");
		list2.addString("target");

		rightIsFirst = points(0,1) > points(1,1);
		if(rightIsFirst)
		{
			list1.addInt(1);
			list1.addDouble(points(0,0));
			list1.addDouble(points(0,1));
			list1.addDouble(points(0,2));

			list2.addInt(1);
			list2.addDouble(points(1,0));
			list2.addDouble(points(1,1));
			list2.addDouble(points(1,2));
		}
		else
		{
			list2.addInt(1);
			list2.addDouble(points(0,0));
			list2.addDouble(points(0,1));
			list2.addDouble(points(0,2));

			list1.addInt(1);
			list1.addDouble(points(1,0));
			list1.addDouble(points(1,1));
			list1.addDouble(points(1,2));
		}

		cout << list1.toString() << endl;
		cout << list2.toString() << endl;

		answStream.addString("Done");
		answStream.addString("Done");
	};
};

}
}

#endif
