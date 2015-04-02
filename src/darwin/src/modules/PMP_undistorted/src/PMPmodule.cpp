#include "darwin/PMPmodule.h"

using namespace std;
using namespace yarp::os;
using namespace Darwin::pmp;


//l'ordine da rispettare è quello dell'enum
// fare i casi set!
const string PMPmodule::CMD_LIST[] = {"update", "back2default", "getModulePeriod", "getThreadPeriod", "getThreadMode",
									  "setMaxIterNumber_right", "setMaxIterNumber_left", "setActiveChain",
									  "getActiveChain", "PMP_set", "VTGSright_set", "VTGSleft_set", 
									  "PMP_get", "VTGSright_get", "VTGSleft_get", "startPMP", "stopPMP", 
									  "resetPMP", "initIcubUp", "initHead", "initRightArm", "initLeftArm", "initTorso", 
									  "ready", "update_angles", "useIndex", "exit"
								};

const string PMPmodule::CMD_DESC[] = {"Update module parameters reading from update config file",
									  "Set default module parameters",
									  "Get the module periodicity, default value is 0.10 s",
									  "Get the PMP thread periodicity, default value is 20 ms",
									  "Get the PMP thread status, running or suspended",

									  "Set the maximum number of VTGS_right iterations (intermediate points computed)"
									  "for each critical point-to-critical point path",

									  "Set the maximum number of VTGS_left iterations (intermediate points computed)"
									  "for each critical point-to-critical point path",
									
									  "Set the chain with which the movement is executed: values are right, left, bimanual",

									  "Get the chain with which the movement is executed: values are right, left, bimanual",

									  "Set parameters of the PMP object inside the PMP thread.\n "
									  "To set a parameter type \"PMP_set param_name\", where param_name can be:\n "
									  "xr_target, xl_target, q_rightArm, q_leftArm, qr_initial, ql_initial, "
									  "Kr_virt, Kl_virt, A_rightArm, A_leftArm, Kr_int, Kl_int, K_virt_bimanual, "
									  "A_bimanual, T_init, T_dur, SlopeRamp, alpha",

									  "Set parameters of the VTGS object relative to right hand inside the PMP thread.\n "
									  "To set a parameter type \"VTGS_set param_name\", where param_name can be: \n"
									  "target, weights, T_init1, T_dur1, SlopeRamp1, alpha1, T_init2, T_dur2, SlopeRamp2, alpha2",

									  "Set parameters of the VTGS object relative to left hand inside the PMP thread.\n "
									  "To set a parameter type \"VTGS_set param_name\", where param_name can be: \n"
									  "target, weights, T_init1, T_dur1, SlopeRamp1, alpha1, T_init2, T_dur2, SlopeRamp2, alpha2",

									  "Get parameters of the PMP object inside the PMP thread.\n "
									  "To set a parameter type \"PMP_get param_name\", where param_name can be:\n "
									  "xr_target, xl_target, q_rightArm, q_leftArm, qr_initial, ql_initial, "
									  "Kr_virt, Kl_virt, A_rightArm, A_leftArm, Kr_int, Kl_int, K_virt_bimanual, "
									  "A_bimanual, T_init, T_dur, SlopeRamp, alpha",

									  "Get parameters of the VTGS object relative to right hand inside the PMP thread.\n "
									  "To get a parameter type \"VTGS_set param_name\", where param_name can be: \n"
									  "target, weights, T_init1, T_dur1, SlopeRamp1, alpha1, T_init2, T_dur2, SlopeRamp2, alpha2",

									  "Get parameters of the VTGS object relative to left hand inside the PMP thread.\n "
									  "To get a parameter type \"VTGS_set param_name\", where param_name can be: \n"
									  "target, weights, T_init1, T_dur1, SlopeRamp1, alpha1, T_init2, T_dur2, SlopeRamp2, alpha2",

									  "Start the PMP thread in the selected configuration: values are \"execute\" or \"simulate\"",
									  "Suspend the PMP thread, thread statistic is not changed",
									  "Stop and reset the PMP thread statistics and the execution configuration (execute-simulate)",
									  "Initialize icub hands up.",
									  "Initialize icub head.",
									  "Initialize icub right arm.",
									  "Initialize icub left arm.",
									  "Initialize icub torso.",
									  "Ask the module if is ready to start. If not, a computation is running. (answer yes/no)",
									  "Update joint position and VTGS starting point reading from encoders",
									  "Use index finger as the end-effector (useIndex right/left on/off)",
									  "Quit the module"
								};

const unsigned int PMPmodule::CMD_SIZE = 28;

// module default values
const string PMPmodule::MODULE_NAME			= "PMPmodule";
const string PMPmodule::CMD_PORT_NAME		= "/PMPmodule/rpc";
const string PMPmodule::RF_UPDATE_NAME		= "PMPupdates.ini";
const string PMPmodule::THREAD_NAME			= "PMPthread";
const int    PMPmodule::THREAD_PERIOD		= 10;
const string PMPmodule::THREAD_RPC_CLI		= "/rpc";
const string PMPmodule::THREAD_RPC_SRV      = "/DevDriver/rpc";
const string PMPmodule::THREAD_RPC_SRV_NAME = "DevDriver";

bool PMPmodule::configure(yarp::os::ResourceFinder &rf_default)
{
	isConfigured = false;
	if(rf_default.isNull())
	{
		printf("No configuration file found, closing... /n");
		return false;
	}

	this->rf_default = rf_default;

	if(!initialize("update"))	
		return false;

	isConfigured = true;

	attachTerminal();

	// create and configure the PMP thread

	myPMP = new PMPthread(&threadPropPMP_right, &threadPropPMP_left, &threadPropPMP_bimanual, &threadPropPMP_tbg,
						  &threadPropVTGS_right, &threadPropVTGS_left, 
						  options.find("threadName").asString().c_str(),
						  options.find("threadRPCclient").asString().c_str(),
						  options.find("threadRPCserver").asString().c_str(),
						  options.find("threadRPCserverName").asString().c_str(),
						  options.find("threadPeriod").asInt()
						  );
	//myPMP->start();	// call to threadInit() and, if true, to run();
	//myPMP->threadInit();
	//myPMP->initIcubUp();
	
	//Thread created but not started yet. isRunning() would return false.
	isStarted = false;

	startThread = false;
	stopThread = false;
	resetThread = false;

	needToClose = false;

	return true;
}

bool PMPmodule::interruptModule()
{
	cout << "asking to interrupt" << endl;
	myPMP->interrupt();
	if (myPMP->isRunning())
	{ 
		cout << "Running, reset sleep" << endl;
		myPMP->sleep.signal();
		cout << "stoppo" << endl;
		myPMP->stop();
		cout << "fine stop" << endl;		
	}
	//detachTerminal();
	return true;
}

bool PMPmodule::close()
{
	cout << "sono nel close" << endl;
	//delete myPMP;
	//myPMP->threadRelease();
	cmdPort.interrupt();
	cmdPort.close();
	return true;
}

bool PMPmodule::respond(const Bottle &command, Bottle &reply)
{
	reply.clear();
	PMPCommands cmd;
	Property opt;

	//identifyCmd reads only the first keyword
	if(!identifyCmd(command, cmd))
	{
		reply.addString("Unknown command. Input 'help' to get a list of the available commands.\n");
		return true;
	}

	bool wasRunning = myPMP->isRunning();
	bool wasSuspended = myPMP->isSuspended() && wasRunning;

	ResourceFinder rf_foo;
	string s;
	bool fromDefault = true;

	switch (cmd)
	{
		case exit:
			reply.addString("--> quitting");
			needToClose = true;
			//interruptModule();
			return true;

		case update:
			configSem.wait();

			// check if value associated to new rf update path is present:
			if(command.size() == 2)
			{
				rf_update_path = command.get(1).asString();
				fromDefault = false;
			}

			if(wasRunning)
			{
				wasRunning = true;
				myPMP->suspend();
				myPMP->resetStat();
				myPMP->threadRelease();
				reply.addString("--> Thread suspended");
			}
			
			// update module variables and PMPthread properties
			if(!initialize("update",fromDefault))
			{
				reply.addString("Error, couldn't update module parameters");
				myPMP->threadInit();
				configSem.post();
				
				return false;
			}
			
			// initialize thread with new properties, but don't start it.
			if(wasRunning)
			{
				myPMP->initialize();
				myPMP->threadInit();
				reply.addString("--> Thread initialized");
			}

			reply.addString("--> Update finished");
			configSem.post();
			return true;

		case back2default:
			configSem.wait();

			// suspend the thread and initialize new properties
			if(wasRunning)
			{
				wasRunning = true;
				myPMP->suspend();
				myPMP->resetStat();
				myPMP->threadRelease();
				reply.addString("--> Thread suspended");
			}
			
			// initialize module variables to default
			// initialize PMPthread properties to default values
			if(!createPMPproperty(rf_foo,"default"))
			{
				reply.addString("Error, couldn't create PMP property object");
				configSem.post();
				return false;
			}
			if(!createVTGSproperty(rf_foo,"default"))
			{
				reply.addString("Error, couldn't create VTGS property object");
				configSem.post();
				return false;
			}
			// initialize thread with new properties, but don't start it.
			// chiamare initialize del thread per ricreare oggetti con nuove proprietà
			if(!myPMP->isRunning())
			{
				myPMP->initialize();
				myPMP->threadInit();
				reply.addString("--> Thread initialized");
			}

			reply.addString("--> Default configuration restored");
			configSem.post();
			return true;

		case getModulePeriod:
			reply.addDouble(getPeriod());
			break;

		case getThreadPeriod:
			reply.addInt(options.find("threadPeriod").asInt());
			reply.addString("ms");
			break;

		case getThreadMode:
			switch(myPMP->getState())
			{
				case PMPthread::simulate:
					reply.addString("Simulate");
					break;
				case PMPthread::execute:
					reply.addString("Execute");
					break;
			}
			break;

		//case setMaxIterNumber_right:
		//	myPMP->maxIterNumber_right
		//case setMaxIterNumber_left:
		//	myPMP->maxIterNumber_left

		case setActiveChain:
			if(command.size() > 1)
			{
				myPMP->setActiveChain(command.get(1).asString().c_str());
				reply.addString("Done");
			}
			else
			{
				reply.addString("Error: no value specified!");
				break;
				//return false;
			}
			
			reply.addString(myPMP->getActiveChain().c_str());
			//printf("Active chain is %s",s.c_str());
			break;

		case getActiveChain:
			reply.addString(myPMP->getActiveChain().c_str());
			//printf("Active chain is %s",s.c_str());
			break;

		case PMP_set:
			if(command.size() > 1)
			{
				if(myPMP->setPMPparam(command.tail()))
				{
					reply.addString("Done");
					break;
				}
				else
				{
					reply.addString("Error!");
					break;
					//return false;
				}
				
			}
			else
			{
				reply.addString("Command syntax not correct: read PMP_set command description below.");
				reply.addString(CMD_DESC[PMP_set].c_str());
				break;
				//return false;
			}

		case VTGSright_set:
			if(command.size() > 1)
			{
				reply.addString("Done");
				reply.addInt(myPMP->setVTGSparam(command.tail(),"right"));
				break;
			}
			else
			{
				reply.addString("Command syntax not correct: read VTGSright_set command description below.");
				reply.addString(CMD_DESC[VTGSright_set].c_str());
				break;
				//return false;
			}

		case VTGSleft_set:
			if(command.size() > 1)
			{
				reply.addString("Done");
				reply.addInt(myPMP->setVTGSparam(command.tail(),"left"));
				break;
			}
			else
			{
				reply.addString("Command syntax not correct: read VTGSright_set command description below.");
				reply.addString(CMD_DESC[VTGSleft_set].c_str());
				break;
				//return false;
			}

		case PMP_get:
			if(command.size() >= 2)
			{
				reply.append(myPMP->getPMPparam(command.tail()));
				break;
			}
			else
			{
				reply.addString("Command syntax not correct: read PMP_get command description below.");
				reply.addString(CMD_DESC[PMP_get].c_str());
				break;
				//return false;
			}

		case VTGSright_get:
			if(command.size() == 2)
			{
				reply.append(myPMP->getVTGSparam(command.tail(),"right"));
				break;
			}
			else
			{
				reply.addString("Command syntax not correct: read VTGSright_get command description below.");
				reply.addString(CMD_DESC[VTGSright_get].c_str());
				break;
				//return false;
			}

		case VTGSleft_get:
			if(command.size() == 2)
			{
				reply.append(myPMP->getVTGSparam(command.tail(),"left"));
				break;
			}
			else
			{
				reply.addString("Command syntax not correct: read VTGSleft_get command description below.");
				reply.addString(CMD_DESC[VTGSleft_get].c_str());
				break;
				//return false;
			}
			break;

		case startPMP:
			cout << "running: " << myPMP->isRunning() << endl;
			cout << "suspended: " << myPMP->isSuspended() << endl;

			if(command.size() == 2)
			{
				s = command.get(1).asString();
				if (s == "execute")
				{
					threadState = PMPthread::execute;
					//startThread = true;
					startPMPthread();
					reply.addString("Done");
				}
				else if (s == "simulate")
				{
					threadState = PMPthread::simulate;
					//startThread = true;
					startPMPthread();
					reply.addString("Done");
				}
				else
				{
					reply.addString("Error: state value not known");
				}
			}
			else
			{
				reply.addString("Error: no value specified");
				return false;
			}
			break;

		case stopPMP:
			//stopThread = true;
			stopPMPthread();
			reply.addString("Done");
			reply.addString("Sending command");
			break;

		case resetPMP:
			//resetThread = true;
			resetPMPthread();
			reply.addString("Done");
			reply.addString("Sending command");
			break;

		case initIcubUp:
			//if(!myPMP->isRunning())
			//	myPMP->threadInit();
			if(!wasSuspended)	myPMP->suspend();
			myPMP->initIcubUp();
			reply.addString("Done");
			break;

		case initHead:
			if (command.size() != 7)
			{
				reply.addString("Error: wrong parameters number");
				break;
			}

			if(!wasSuspended)	myPMP->suspend();
			//cout << "PMP: " << command.toString() << endl;
			reply.copy(myPMP->initHead(command));			
			break;
		
		case initRightArm:
		case initLeftArm:
			if (command.size() == 1)
			{
				if(!wasSuspended)	myPMP->suspend();

				reply.copy(myPMP->initArm(command));
			}
			// user-defined initialization (not implemented)
			else if (command.size() == 8)
			{
				if(!wasSuspended)	myPMP->suspend();

				reply.copy(myPMP->initArm(command));
			}
			else
			{
				reply.addString("Error: wrong parameters number");
				break;
			}			
			break;

		case initTorso:
			if (command.size() == 1 || command.size() == 4)
			{
				if(!wasSuspended)	myPMP->suspend();

				reply.copy(myPMP->initTorso(command));
			}
			else
			{
				reply.addString("Error: wrong parameters number");
				break;
			}						
			break;
		
		case ready:
			if (myPMP->readyToStart)	reply.addString("yes");
			else						reply.addString("no");
			break;

		case update_angles:
			if(myPMP->updateCurrentAngles(true))	reply.addString("Done");
			else									reply.addString("Error");
			break;

		case useIndex:
			if (command.size() != 3)
			{
				reply.addString("Error: wrong parameters number");
				break;
			}
			
			// to modify the end effector all current computations are stopped and deleted:
			if(!wasSuspended)	myPMP->suspend();

			
			if (command.get(2).asString() == "on")
			{
				if(!myPMP->setIndexAsEndEffector(command.get(1).asString().c_str())) reply.addString("Error");
				else																 reply.addString("Done");
			}
			else if (command.get(2).asString() == "off")
			{
				if(!myPMP->setPalmAsEndEffector(command.get(1).asString().c_str()))  reply.addString("Error");
				else																 reply.addString("Done");
			}
			else
			{
				reply.addString("Error: commands allowed are on/off");
			}

			cout << "fine chiamata a metodo" << endl;

			myPMP->resetStat();	
			myPMP->readyToStart = true;

			break;

		default:
			reply.addString("Default case, idle");
			break;
	}

	if(!wasSuspended) myPMP->resume();

	return true;
}

double PMPmodule::getPeriod()
{
	return 0.1;
}

// give VTGS the new critical point coordinates when readyToStart flag is set to true
// Before setting new target points PMPthread is suspended and thread stats reset.
bool PMPmodule::updateModule()
{

	if (needToClose)
	{
		interruptModule();
		return false;
	}

	// if PMP has reached the target, stop the thread execution
	// until a new target has given to the VTGS object (Thread hasTarget flag true)	
	if (startThread && !myPMP->isRunning())
	{
		// occurring only at the beginning
		if( myPMP->hasTarget )
		{
			myPMP->resetStat();
			startPMPthread();
			//printf("--> Thread started\n");
		}
	}
	else if (myPMP->readyToStart)
	{
		updateSem.wait();
		
		// check if thread has already been suspended
		if( !myPMP->isSuspended() )
		{
			myPMP->suspend();	
		}

		// check if a new target is present: if yes, reset and resume PMPthread
		if( myPMP->hasTarget )
		{
			// reset PMP and VTGS iteration number
			myPMP->resetStat();

			if(startPMPthread())
			{
				// update VTGS starting point and PMP joint angles according to the encoder's values
				myPMP->updateCurrentAngles(true);				

				// signal the thread to start
				myPMP->sleep.signal();
			}

			printf("--> Thread started\n");
			cout << "--- Flag State---" << endl;
			cout << "readyToStart: " << myPMP->readyToStart << endl;
			cout << "HasTarget: " << myPMP->hasTarget << endl;
		}

		updateSem.post();
	}

	return true;
}

// If is called in configuration procedure, it initializes the module name and opens the cmdPort.
bool PMPmodule::initialize(string mode, bool fromDefaultRF)
{
	if (mode == "default")
	{
		printf("--> Initializing, using default configuration\n");
		options.fromString(rf_default.toString());
		return true;
	}

	if( !mergeProperties(fromDefaultRF))
			return false;

	cout <<"fuori da merge"<< endl;
	// module name and ports name are not changed after the module configuration
	if(!isConfigured)
	{
		// set module name: if no name exists, set PMPmodule as the module name
		moduleName = options.find("moduleName").asString();
		moduleName.empty() ? setName(MODULE_NAME.c_str()) : setName(moduleName.c_str());
		printf("--> Module name set as: %s\n", moduleName.c_str());

		// Attach a port to the module to redirect the received messages to the respond method
		options.check("cmdPortName") ? cmdPortName = options.find("cmdPortName").asString() : cmdPortName = CMD_PORT_NAME.c_str();

		if(!cmdPort.open(cmdPortName.c_str()))
		{
			printf("Unable to open port %s\n", cmdPortName.c_str());
			printf("Closing module");
			return false;
		}
		printf("--> Command port name set as: %s\n", cmdPort.getName().c_str());

		attach(cmdPort);

		// set thread name: if no name exists, set PMPthread as the module name
		// set thread period: if no period specified, set 20 ms;
		if(!options.check("threadName"))	      options.put("threadName",THREAD_NAME.c_str());
		if(!options.check("threadPeriod"))        options.put("threadPeriod",THREAD_PERIOD);
		if(!options.check("threadRPCclient"))     options.put("threadRPCclient",THREAD_RPC_CLI.c_str());
		if(!options.check("threadRPCserver"))     options.put("threadRPCserver",THREAD_RPC_SRV.c_str());
		if(!options.check("threadRPCserverName")) options.put("threadRPCserverName",THREAD_RPC_SRV_NAME.c_str());

		printf("--> Thread name set as: %s\n", options.find("threadName").toString().c_str());
		printf("--> Thread period set as: %d ms\n", options.find("threadPeriod").asInt());
		//cout << "THREAD_PERIOD" << " " << options.find("threadPeriod").asDouble() << endl;
	}
	
	cout << "fuori da initialize" << endl;
	return true;

}

// Create property objects for module configuration and thread initialization
bool PMPmodule::mergeProperties(bool fromDefaultRF)
{
	// if no updates are present, use default parameters
	ResourceFinder updates; // module variable


	options.fromString(rf_default.toString());

	// try to open the path and modify the properties: default case is fromDefaultRF = true.
	// if the method is called by update command in respond method, then, if a path is specified as a parameter
	// saved in variable rf_update_path, try to open the specified config file. 
	// ( rf_update_path = complete_path/file_name.config )

	if (fromDefaultRF)
	{
		if (!rf_default.check("update_path"))
		{
			printf("--> No update path found, using default configuration\n");
			//options.fromString(rf_default.toString());
			if(!createPMPproperty(updates,"default") || !createVTGSproperty(updates,"default"))
				return false;

			return true;
		}
	
		rf_update_path.clear();
		rf_update_path = rf_default.find("update_path").asString();
//		rf_update_path.append("/");

		rf_default.check("update_fileName") ? rf_update_path.append(rf_default.find("update_fileName").asString().c_str()) : 
										  rf_update_path.append(RF_UPDATE_NAME.c_str());
	}

	printf("--> Found Update file name: %s\n", rf_update_path.c_str());
	
	updates.setVerbose(true);
	updates.setDefaultConfigFile(rf_update_path.c_str());
	updates.setDefaultContext("PassiveMotionParadigm_DARWIN/conf");
	updates.configure("ICUB_ROOT",0,NULL);
	
	if(updates.isNull())
	{
		printf("Error: couldn't find the following config-file path: %s",rf_update_path.c_str());
		printf("--> No update path found, using default configuration \n");
		//options.fromString(rf_default.toString());
		if(!createPMPproperty(updates,"default") || !createVTGSproperty(updates,"default"))
			return false;
	
		return true;
	}

	// overwrite parameters in module property
	if (updates.check("moduleName"))      options.put("moduleName",updates.find("moduleName"));
	if (updates.check("update_path"))     options.put("update_path",updates.find("update_path"));
	if (updates.check("update_fileName")) options.put("update_fileName",updates.find("update_fileName"));

	// fill in thread properties
	if (updates.check("threadName"))      options.put("threadName",updates.find("threadName"));
	if (updates.check("threadPeriod"))    options.put("threadPeriod",updates.find("threadPeriod"));

	cout <<"prima del create"<< endl;
	if(!createPMPproperty(updates, "update") || !createVTGSproperty(updates,"update"))
		return false;

	cout <<"fine merge"<< endl;
	return true;
}

bool PMPmodule::createPMPproperty(ResourceFinder &rf_update, string mode)
{
	cout <<"nel create"<< endl;
	bool found = rf_default.check("PMP_right") && rf_default.check("PMP_left") &&
				 rf_default.check("PMP_bimanual") && rf_default.check("PMP_tbg");

	/*
	Bottle &PMP_right    = rf_default.findGroup("PMP_right");
	Bottle &PMP_left     = rf_default.findGroup("PMP_left");
	Bottle &PMP_bimanual = rf_default.findGroup("PMP_bimanual");
	Bottle &PMP_tbg		 = rf_default.findGroup("PMP_tbg");

	if(!PMP_right.isNull() || PMP_left.isNull() || PMP_bimanual.isNull() || PMP_tbg.isNull())
	{
		printf("Config-file error: some PMP properties may be missing/n");
		printf("Error, not enough PMP parameters field to initialize thread, closing/n");
		return false;
	}
	*/
	if(mode == "default")
	{
		if(!found)
		{
			printf("Config-file error: some PMP properties may be missing\n");
			printf("Error, not enough PMP parameters field to initialize thread, closing\n");
			return false;
		}
		threadPropPMP_right.fromString(PropertyGroup2String(rf_default.findGroup("PMP_right")).c_str());
		threadPropPMP_left.fromString(PropertyGroup2String(rf_default.findGroup("PMP_left")).c_str());
		threadPropPMP_bimanual.fromString(PropertyGroup2String(rf_default.findGroup("PMP_bimanual")).c_str());
		threadPropPMP_tbg.fromString(PropertyGroup2String(rf_default.findGroup("PMP_tbg")).c_str());
		return true;
	}

	// if there exist an update for a group, then initialize PMPproperty to this value,
	// unless a default configuration is required

	// check PMP_right group
	if(rf_update.check("PMP_right"))
		threadPropPMP_right.fromString(PropertyGroup2String(rf_update.findGroup("PMP_right")).c_str());
	else
	{
		printf("--> PMP_right parameters update absent, using default parameter instead\n");
		//cout << PropertyGroup2String(rf_default.findGroup("PMP_right")) << endl;
		threadPropPMP_right.fromString(PropertyGroup2String(rf_default.findGroup("PMP_right")).c_str());
	}

	// check PMP_left group
	if(rf_update.check("PMP_left"))
		threadPropPMP_left.fromString(PropertyGroup2String(rf_update.findGroup("PMP_left")).c_str());
	else
	{
		printf("--> PMP_left parameters update absent, using default parameter instead\n");
		threadPropPMP_left.fromString(PropertyGroup2String(rf_default.findGroup("PMP_left")).c_str());
	}
	
	// check PMP_bimanual group
	if(rf_update.check("PMP_bimanual"))
		threadPropPMP_bimanual.fromString(PropertyGroup2String(rf_update.findGroup("PMP_bimanual")).c_str());
	else
	{
		printf("--> PMP_bimanual parameters update absent, using default parameter instead\n");
		threadPropPMP_bimanual.fromString(PropertyGroup2String(rf_default.findGroup("PMP_bimanual")).c_str());
	}

	// check PMP_tbg group
	if(rf_update.check("PMP_tbg"))
		threadPropPMP_tbg.fromString(PropertyGroup2String(rf_update.findGroup("PMP_tbg")).c_str());
	else
	{
		printf("--> PMP_tbg parameters update absent, using default parameter instead\n");
		threadPropPMP_tbg.fromString(PropertyGroup2String(rf_default.findGroup("PMP_tbg")).c_str());
	}
	return true;
}

bool PMPmodule::createVTGSproperty(ResourceFinder &rf_update,string mode)
{
	bool found = rf_default.check("VTGS_right") && rf_default.check("VTGS_left");

	cout <<"nel create VTGS"<< endl;

	if(mode == "default")
	{
		if(!found)
		{
			printf("Config-file error: some VTGS properties may be missing\n");
			printf("Error, not enough VTGS parameters field to initialize thread, closing\n");
			return false;
		}
		threadPropVTGS_right.fromString(PropertyGroup2String(rf_default.findGroup("VTGS_right")).c_str());
		threadPropVTGS_left.fromString(PropertyGroup2String(rf_default.findGroup("VTGS_left")).c_str());

		return true;
	}

	// if there exist an update for a group, then initialize VTGSproperty to this value,
	// unless a default configuration is required

	// check VTGS_right group
	if(rf_update.check("VTGS_right"))
		threadPropVTGS_right.fromString(PropertyGroup2String(rf_update.findGroup("VTGS_right")).c_str());
	else
	{
		printf("--> VTGS_right parameters update absent, using default parameter instead\n");
		threadPropVTGS_right.fromString(PropertyGroup2String(rf_default.findGroup("VTGS_right")).c_str());	
	}
	//cout << threadPropVTGS_right.toString() << endl;
	//cout << PropertyGroup2Bottle(rf_default.findGroup("VTGS_right")).toString() << endl;

	// check VTGS_left group
	if(rf_update.check("VTGS_left"))
		threadPropVTGS_left.fromString(PropertyGroup2String(rf_update.findGroup("VTGS_left")).c_str());
	else
	{
		printf("--> VTGS_left parameters update absent, using default parameter instead\n");
		threadPropVTGS_left.fromString(PropertyGroup2String(rf_default.findGroup("VTGS_left")).c_str());
	}

	return true;

/*

	Bottle &VTGS_right    = rf_default.findGroup("VTGS_right");
	Bottle &VTGS_left     = rf_default.findGroup("VTGS_left");

	if(VTGS_right.isNull() || VTGS_left.isNull())
	{
		printf("Config-file error: some VTGS properties may be missing/n");
		printf("Error, not enough VTGS parameters field to initialize thread, closing/n");
		return false;
	}

	// if there exist an update for a group, then initialize VTGSproperty to this value,
	// unless a default configuration is required

	if(mode == "default")
	{
		threadPropVTGS.put("VTGS_right",VTGS_right.toString());
		threadPropVTGS.put("VTGS_left",VTGS_left.toString());
		return true;
	}

	Bottle &VTGS_rightUD  = rf_update.findGroup("VTGS_right");
	Bottle &VTGS_leftUD   = rf_update.findGroup("VTGS_left");

	// if an updated configuration is absent, use the default values
	if(VTGS_rightUD.isNull())
	{
		printf("-->VTGS_right parameters update absent, using default parameter instead\n");
		threadPropVTGS.put("VTGS_right",VTGS_right.toString());
	}
	else
		threadPropVTGS.put("VTGS_right",VTGS_rightUD.toString());

	if(VTGS_leftUD.isNull())
	{
		printf("-->VTGS_left parameters update absent, using default parameter instead\n");
		threadPropVTGS.put("VTGS_left",VTGS_left.toString());
	}
	else
		threadPropVTGS.put("VTGS_left",VTGS_leftUD.toString());
													

	return true;
	*/
}

string PMPmodule::PropertyGroup2String(Bottle group)
{
	Bottle bot;
	bot.clear();

	for (int i = 1; i < group.size(); i++)
	{
		bot.add(group.get(i));
	}

	string s = bot.toString().c_str();
	//cout << s << endl;
	return s;
}

// Convert a command in a bottle into the correspondent enum value
bool PMPmodule::identifyCmd(Bottle cmdBot, PMPCommands &cmd)
{
	unsigned int i=0;
	string word = cmdBot.get(0).asString().c_str();
	
	for (unsigned int i=0; i < CMD_SIZE; i++)
	{
		if (word == CMD_LIST[i])
		{
			cmd = (PMPCommands)i;
			return true;
		}
	}

	return false;
}


Value PMPmodule::PropertyGroup2Bottle(Bottle group)
{
	Bottle bot;
	bot.clear();

	for (int i = 1; i < group.size(); i++)
	{
		bot.add(group.get(i));
	}

	Value val;
	val.fromString(bot.toString());
	return val;
}


bool PMPmodule::startPMPthread()
{
	startSem.wait();
	startThread = true;
	//cout << "running: " << myPMP->isRunning() << endl;
	//cout << "suspended: " << myPMP->isSuspended() << endl;

	myPMP->setState(threadState);
	//cout << "target" << myPMP->hasTarget << endl;

	if(myPMP->hasTarget)
	{
		// if thread is not running, start it. If it is running but is currently suspended resume
		if(!myPMP->isRunning())
		{
			myPMP->start();
			cout << "start" << endl;
			printf("--> Thread started\n");
			isStarted = true;

			//if(!myPMP->hasTarget)
			//{
			//	myPMP->suspend();
			//}
		}

		else if(myPMP->isSuspended())
		{		
			myPMP->resume();
			cout << "resume" << endl;
			printf("--> Thread started\n");
		}
	
		else 
		{
			cout << "Attention: Thread has already started" << endl;
			startSem.post();
			return false;
		}
	}
	else
	{
		cout << "Attention: no target assigned, Thread can't start" << endl;
		startSem.post();
		return false;
	}

	startSem.post();
	return true;
}

bool PMPmodule::stopPMPthread()
{
	updateSem.wait();
	stopThread = false;

	// Option not available if thread is not running or already suspended
	if(myPMP->isRunning())
	{
		if(!myPMP->isSuspended())
			myPMP->suspend();

		printf("--> Thread stopped\n");
	}
	else
		printf("Error: thread is not running\n");

	updateSem.post();
	return true;
}

bool PMPmodule::resetPMPthread()
{
	updateSem.wait();
	resetThread = false;

	// Suspend the thread if not suspended already and reset thread statistics
	if(myPMP->isRunning())
	{
		if(!myPMP->isSuspended())
			myPMP->suspend();
	
		myPMP->resetStat();

		printf("--> Thread reset\n");
	}
	else
		printf("Error: Thread is not running\n");

	updateSem.post();
	return true;
}
