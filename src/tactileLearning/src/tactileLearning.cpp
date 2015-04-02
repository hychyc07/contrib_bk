// system includes
#include <sstream>

// local includes
#include <iCub/tactileLearning/tactileLearning.h>
 
using namespace iCub::tactileLearning;

// module default values
const string TactileLearning::MODULE_NAME_DEFAULT = "TactileLearning";
const string TactileLearning::ROBOT_NAME_DEFAULT = "icubSim";
const string TactileLearning::RPC_PORT_DEFAULT = "/rpc";
const string TactileLearning::HAND_DEFAULT = "left";

bool TactileLearning::configure(yarp::os::ResourceFinder& rf)
{    
	// Process all parameters from both command-line and .ini file

	// Get the module name which will form the stem of all module port names
	moduleName = rf.check("name", Value(MODULE_NAME_DEFAULT.c_str()), "module name (string)").asString();
	robotName = rf.check("robot", Value(ROBOT_NAME_DEFAULT.c_str()), "name of the robot (string)").asString();
	
	/*
	* Before continuing, set the module name before getting any other parameters, 
	* specifically the port names which are dependent on the module name
	*/
	setName(moduleName.c_str());

	/* 
	 * Attach a port of the same name as the module (prefixed with a /) to the module
	 * so that messages received from the port are redirected to the respond method
	 */
	string handlerPortName = "/";
	handlerPortName += getName(rf.check("handlerPort", Value(RPC_PORT_DEFAULT.c_str())).asString());
	if (!handlerPort.open(handlerPortName.c_str()))
	{
		cout << getName() << ": Unable to open port " << handlerPortName << endl;  
		return false;
	}

	// Attach to port
	attach(handlerPort);
    handlerPort.setRpcMode(true);

	// Get the remaining parameters
	string hand = rf.check("hand", Value(HAND_DEFAULT.c_str()), "hand used (string)").asString();
	if(hand.compare("left") == 0) leftHand = true;
	else leftHand = false;

	if(!getFingersAndJoints(rf)) return false;

	/*
	// modify the resource finder to read the configuration file needed for PI^2 initialization
	rf.setDefaultConfigFile("piSquareConf.ini");
	rf.setDefaultContext("piSquare/conf");
	*/

	// Create the thread and pass pointers to the module parameters
	thread = new TactileLearningThread(&rf, moduleName, robotName, leftHand, activeFingers, activeJoints);

	// Call threadInit() and if it returns true, then call run()
	thread->start();

	return true;
}

bool TactileLearning::interruptModule()
{
	handlerPort.interrupt();
	return true;
}

bool TactileLearning::close()
{
	// stop the thread and close the port
	if(thread)
	{
		thread->stop();
		delete thread;
    }

	handlerPort.close();   

	return true;
}

bool TactileLearning::respond(const Bottle& command, Bottle& reply) 
{
	stringstream temp;

	// reply contains a "command not recognized" by default
	reply.clear();

	TactileLearningCommand com;
    Bottle params;
    if(command.get(0).isInt())
	{
        // if first value is int then it is the id of the command
        com = (TactileLearningCommand)command.get(0).asInt();
        params = command.tail();
    }
	else if(!identifyCommand(command, com, params))
	{
		reply.addString("Unknown command. Input 'help' to get a list of the available commands.");
		return true;
	}

	// external declarations
	vector<Vector> referenceTrajectory;
	vector<Vector> tactileState;

	switch(com)
	{
		case get_optimal_trajectory:
			reply.addString("Module should provide the optimal trajectory if computation is finished");
			//thread->getState(tactileState);
			//addToBottle(reply, tactileState);
			return true;

		case get_info:
            reply.addString("Sorry but there is no info available for the moment!");
			return true;

		case get_help:
            reply.addString("Sorry but there is no help available for the moment!");
			return true;

		case quit:
			reply.addString("quitting...");
			return false;

		default:
			reply.addString("ERROR: This command is known but it is not managed in the code.");
			return true;
	}

	return true;
}

bool TactileLearning::identifyCommand(Bottle commandBot, TactileLearningCommand &com, Bottle& params)
{
	for(unsigned int i = 0; i < tactileLearningCommandSize; i++)
	{
		stringstream stream(TactileLearningCommandList[i]);
		string word;
		int j = 0;
		bool found = true;

		while(stream>>word)
		{
			if (commandBot.get(j).asString() != word.c_str())
			{
				found=false;
				break;
			}
			j++;
		}
        if(found)
		{
			com = (TactileLearningCommand)i;
            params = commandBot.tail();
            for(int k = 1; k < j; k++) params = params.tail();
			return true;
		}
	}

	return false;
}

bool TactileLearning::updateModule()
{
    return true;
}

bool TactileLearning::getFingersAndJoints(ResourceFinder& rf)
{
	/**
	* There are two mutual exclusive ways to specify which fingers and joints are involved in the problem.
	* 1. Checking which fingers are active and setting the joints properly.
	* 2. Setting by hand the joint vector.
	* In both cases checks for consistency have to be done, which higher priority to the fingers vector. 
	*/

	string tmp;
	int jointIndex;

	if(!rf.findGroup("FINGERS").isNull())
	{
		// check which fingers are specified in the configuration file and set the activeFingers vector accordingly
		for(int i = 0; i < NUM_FINGERS; i++)
		{
			tmp = rf.findGroup("FINGERS").find(FINGER_TYPES[i].c_str()).asString();
			if(tmp.compare("on") == 0) activeFingers[i] = true;
			else activeFingers[i] = false;
		}

		/**
		* Set the joints vector so that it is consistent with the fingers vector values. This step requires 
		* knowledge about hardware implementation of the robot hand.
		*/
		jointIndex = 0;

		//------------------------------------------------------------------------------------------------------------
		// WARNING! The joint of the thumb to be ignored is the 9-th in the simulator, but the 8-th in the real robot!
		//------------------------------------------------------------------------------------------------------------
		// thumb
		if(activeFingers[0] == true)
		{
			for(int i = 0; i < 3; i++)
			{
				// for simulator: ignore second joint (9-th for the whole arm)
				// for real robot: ignore first joint (8-th for the whole arm)
				if(i != 0) activeJoints[jointIndex++] = true;
				else activeJoints[jointIndex++] = false;
			}
		}
		else
		{
			for(int i = 0; i < 3; i++) activeJoints[jointIndex++] = false;
		}

		// index
		if(activeFingers[1] == true)
		{
			for(int i = 0; i < 2; i++) activeJoints[jointIndex++] = true;
		}
		else
		{
			for(int i = 0; i < 2; i++) activeJoints[jointIndex++] = false;
		}

		// middle
		if(activeFingers[2] == true)
		{
			for(int i = 0; i < 2; i++) activeJoints[jointIndex++] = true;
		}
		else
		{
			for(int i = 0; i < 2; i++) activeJoints[jointIndex++] = false;
		}

		// ring + pinky
		if(activeFingers[3] == true) activeJoints[jointIndex] = true;
		else activeJoints[jointIndex] = false;

	}

	else if(rf.check("joints"))
	{
		Bottle joints = rf.findGroup("joints").tail();

		// check which joints are specified in the configuration file and set the activeJoints vector accordingly.
		// brute force but who cares
		for(int i = 0; i < NUM_HAND_JOINTS; i++)
		{
			activeJoints[i] = false;
			for(int j = 0; j < joints.size(); j++)
			{
				if(i == joints.get(j).asInt() - HAND_JOINTS_OFFSET)
				{
					activeJoints[i] = true;
					break;
				}
			}
		}

		/**
		* activate each finger if at least a joint related to it has been activated.
		* This step requires knowledge about hardware implementation of the robot hand.
		*/

		// thumb
		if(activeJoints[0] || activeJoints[1] || activeJoints[2]) activeFingers[0] = true;
		else activeFingers[0] = false;
		
		// index
		if(activeJoints[3] || activeJoints[4]) activeFingers[1] = true;
		else activeFingers[1] = false;

		// middle
		if(activeJoints[5] || activeJoints[6]) activeFingers[2] = true;
		else activeFingers[2] = false;

		// pinky
		if(activeJoints[7]) activeFingers[3] = true;
		else activeFingers[3] = false;
	}
	else
	{
		printf("ERROR: no fingers were set, please check configuration file.\n");
		return false;
	}

	// check for at least one active finger and in this case print both active fingers and active joints
	for(int i = 0; i < NUM_FINGERS; i++)
	{
		if(activeFingers[i])
		{
			printFingersAndJoints();
			return true;
		}
	}

	// if no fingers are active the initialization is not successful
	printf("ERROR: no fingers were set, please check configuration file.\n");
	return false;
}

void TactileLearning::printFingersAndJoints()
{
	string str;

	// print active fingers
	printf("ACTIVE FINGERS:\t");
	for(int i = 0; i < NUM_FINGERS; i++)
	{
		if(activeFingers[i])
		{
			str = FINGER_TYPES[i];
			printf("%s\t", str.c_str());
		}
	}
	printf("\n");

	// print active joints
	printf("ACTIVE JOINTS:\t");
	for(int i = 0; i < NUM_HAND_JOINTS; i++)
	{
		if(activeJoints[i]) printf("%i ", i);
	}
	printf("\n");
}