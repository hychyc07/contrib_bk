#include "crawlHeadControl.h"

#include <yarp/math/Math.h>
using namespace yarp::math;
//#include "ctrlMath.h";

#include <yarp/dev/ControlBoardInterfaces.h>

#include <iostream>
using namespace std;

crawlHeadControl::crawlHeadControl()
{
	
}

crawlHeadControl::~crawlHeadControl(void)
{
}

bool crawlHeadControl::configure(yarp::os::ResourceFinder &rf)
{
    /* Process all parameters from both command-line and .ini file */
    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("crawlHeadControl"), 
                           "module name (string)").asString();
	 /*
    * sets the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
    //setName(moduleName.c_str());

    /* gets the rest of the parameters */
    /*
    * gets the robot name which will form the stem of the robot ports names
    * and appends the specific part and device required
    */
    robotName             = rf.check("robot", 
                           Value("icub"), 
                           "Robot name (string)").asString();

    robotPortName         = "/" + robotName + "/head";

	/* gets the name of the input and output ports, automatically prefixing the module name by using getName() */
   // inputPortName         = "/";
    inputPortName        += getName(
                           rf.check("input_port", 
                           Value("/crawlHeadControl/in"),
                           "Input Head Control port (string)").asString()
                           );

   // outputPortName        = "/";
    outputPortName       += getName(
                           rf.check("output_port", 
                           Value("/crawlHeadControl/out"),
                           "Output Head Control port (string)").asString()
                           );

	headControlCommandCode= rf.check("head_control_command_code", 
                           Value("55"), 
                           "Command code (string)").asInt();

	headControlModeDist   = rf.check("head_control_mode_dist", 
                           Value("0.95"), 
                           "Distance to start the head control (string)").asDouble();
	
	objectID              = rf.check("object_ID", 
                           Value("76"), 
                           "ID of the object to follow with the head. (string)").asInt();

	cout << "headControlModeDist : " << headControlModeDist << endl; 
	
	/* Initialization */
	inPort.open(inputPortName.c_str());
	outPort.open(outputPortName.c_str());

	InitPolydriver();

	return true;
}


bool crawlHeadControl::close()
{
	inPort.close();
    outPort.close();

	/*for(map<string, Value *>::iterator it = parameters.begin(); it!= parameters.end(); it++)
	{
		delete it->second;
	}*/
	return true;
}


bool crawlHeadControl::interruptModule()
{
	inPort.close();
    outPort.close();

	/*for(map<string, Value *>::iterator it = parameters.begin(); it!= parameters.end(); it++)
	{
		delete it->second;
	}*/
	return true;
}


bool crawlHeadControl::updateModule()
{ 
	Bottle *visionBottle = inPort.read();

	double minDistanceSQR = 999999;
	bool patchToReach = false;

	Vector nearestPatchPosition(3);

	for(int i=0; i<visionBottle->size(); ++i)
	{
		
		Bottle *patchBottle=visionBottle->get(i).asList();
				
		if(patchBottle->size() != 4 || patchBottle->get(0).asDouble() > 100)
		{
			cout << "--------ERROR GETTING THE 3D POSITION OF THE OBJECT ---------- " << endl;
			return true;
		}

		if(patchBottle->get(3).asInt() != objectID)
		{
			continue;
		}
cout << "list: " << visionBottle->get(0).asDouble() << endl;
		Vector position(3);
		position[0] = patchBottle->get(0).asDouble();
		position[1] = patchBottle->get(1).asDouble();
		position[2] = patchBottle->get(2).asDouble();

		double distanceSQR = position[0]*position[0] + position[1]*position[1] + position[2]*position[2];

		if(distanceSQR<minDistanceSQR)
		{
			nearestPatchPosition[0] = position[0];
			nearestPatchPosition[1] = position[1];
			nearestPatchPosition[2] = position[2];
			patchToReach = true;
			minDistanceSQR = distanceSQR;
		}
	}

	if(!patchToReach)
	{
		return true;
	}

	map<string, double> headAngles = GetHeadAngles();


	cout << "distance : " << sqrt(minDistanceSQR) << endl;
	cout << "min distance : " << headControlModeDist << endl;
	if(minDistanceSQR<pow(headControlModeDist,2))
	{
		if(nearestPatchPosition[2]<0.001 && nearestPatchPosition[2]>-0.001)
		{
			return true;
		}
		double turnPitchAngle = - atan(nearestPatchPosition[1]/nearestPatchPosition[2]);
		double turnYawAngle = - atan(nearestPatchPosition[0]/nearestPatchPosition[2]);
		double headPitchAngle = headAngles["pitch"] + turnPitchAngle ;
		double headYawAngle = headAngles["yaw"] + turnYawAngle; 
		cout << "current pitch angle : " << headAngles["pitch"] << endl;
		cout << "current yaw angle : " << headAngles["yaw"] << endl;
		cout << "turn pitch angle : " << turnPitchAngle << endl;
		cout << "turn yaw angle : " << turnYawAngle << endl;
		cout << "Head angle pitch : " << headPitchAngle << endl;
		cout << "Head angle Yaw : " << headYawAngle << endl;
		Bottle &outBottle = outPort.prepare();
		outBottle.clear();
		outBottle.addInt(headControlCommandCode);
		outBottle.addDouble(headPitchAngle);
		outBottle.addDouble(headYawAngle);
		outBottle.addDouble(nearestPatchPosition[2]);
		outPort.write();
	}

	return true;
}

double crawlHeadControl::getPeriod(void)
{
	return MODULE_PERIOD;
}


/* CT(15/3/2011): GetValueFromConfig was replaced by RF configure */
/*Value crawlHeadControl::GetValueFromConfig(Searchable& config, string valueName)
{
	if(!config.check(valueName.c_str()))
	{
		cout << "ERROR with config file : couldn't find value : \"" << valueName << "\"." << endl;
		return false;
 	}
	return config.find(valueName.c_str());
}
*/

void crawlHeadControl::InitPolydriver()
{
	Property options;
    options.put("device", "remote_controlboard");
	options.put("local", "/head_control/head");   //local port names
	

	string remotePortName = "/" + (string)robotName + "/head";
	options.put("remote", remotePortName.c_str());         //where we connect to

    // create a device
    polydriver = new PolyDriver(options);
    if (!polydriver->isValid()) 
	{
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return;
    }

    IPositionControl *pos;
    IEncoders *encs;

    if (!(polydriver->view(pos) || polydriver->view(encs))) 
	{
        printf("Problems acquiring interfaces\n");
        return;
    }

    pos->getAxes(&nbJoints);
}

map<string, double> crawlHeadControl::GetHeadAngles(void)
{ 
	map<string, double> headAngles;
	IPositionControl *pos;
    IEncoders *encs;

    if (!(polydriver->view(pos) && polydriver->view(encs))) 
	{
        printf("Problems acquiring interfaces\n");
        return headAngles;
    }
	double *encoders = new double[nbJoints];
	encs->getEncoders(encoders);
	headAngles["pitch"] = encoders[0]*3.14/180;
	headAngles["yaw"] = encoders[2]*3.14/180;
	return headAngles;
	delete[] encoders;
}