
#include "SimoxGraspingPipelineControlModule.h"
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/Grasping/Grasp.h>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace std;
using namespace VirtualRobot;


SimoxGraspingPipelineControlModule::SimoxGraspingPipelineControlModule()
{
	fixedHandEyeOffset << 0.0f,0.0f,-100.0f;
	isLeft = true;
	encLeftArm = encRightArm = encHead = encTorso = NULL;
	posLeftArm = posRightArm = posHead = posTorso = NULL;
	connected = false;
	liftHandDeltaMM = 0;
}



bool SimoxGraspingPipelineControlModule::configure( yarp::os::ResourceFinder &rf )
{
	moduleName            = rf.check("name", 
		Value("SimoxGraspingPipelineControlModule"), 
		"module name (string)").asString();
	setName(moduleName.c_str());

	clientNameGraspExecution =  "/";
	clientNameGraspExecution += getName();
	clientNameGraspExecution += "/SimoxGraspExecution/rpc:o";
	serverNameGraspExecution = "/SimoxGraspExecution/rpc:i";

	clientNameMotionPlanner =  "/";
	clientNameMotionPlanner += getName();
	clientNameMotionPlanner += "/SimoxMotionPlanner/rpc:o";
	serverNameMotionPlanner = "/SimoxMotionPlanner/rpc:i";

	clientNameRobotViewer =  "/";
	clientNameRobotViewer += getName();
	clientNameRobotViewer += "/SimoxRobotViewer/rpc:o";
	serverNameRobotViewer = "/SimoxRobotViewer/rpc:i";

	clientNameIkSolver =  "/";
	clientNameIkSolver += getName();
	clientNameIkSolver += "/SimoxIkSolver/rpc:o";
	serverNameIkSolver = "/SimoxIkSolver/rpc:i";

	clientNameLegoLocalizer =  "/";
	clientNameLegoLocalizer += getName();
	clientNameLegoLocalizer += "/SimoxLegoLocalizer/rpc:o";
	serverNameLegoLocalizer = "/SimoxLegoLocalizer/rpc:i";

	robotBase = rf.check("robot", 
		Value("icubSim"), 
		"robot name (string)").asString();

	isLeft = true;
	std::string side = rf.check("side",yarp::os::Value("left"),"Which side should be used (string: left or right)").asString().c_str();
	if (side=="right")
		isLeft = false;
	if (isLeft)
	{
		cout << "Configuring module for left arm and hand" << endl;
	} else
	{
		cout << "Configuring module for right arm and hand" << endl;
	}
	eefPreshapeName = rf.check("eefPreshape", 
		Value("Grasp Preshape"), 
		"eef preshape name (string)").asString();

	fixedHandEyeOffset[0] = rf.check("hand_eye_calib_offset_x", 
		Value(0.0), 
		"hand eye offset x (double)").asDouble();
	fixedHandEyeOffset[1] = rf.check("hand_eye_calib_offset_y", 
		Value(0.0), 
		"hand eye offset y (double)").asDouble();
	fixedHandEyeOffset[2] = rf.check("hand_eye_calib_offset_z", 
		Value(0.0), 
		"hand eye offset z (double)").asDouble();

	fixedHandEyeOffsetRot[0] = rf.check("hand_eye_calib_offset_roll", 
		Value(0.0), 
		"hand eye offset roll (double)").asDouble();
	fixedHandEyeOffsetRot[1] = rf.check("hand_eye_calib_offset_pitch", 
		Value(0.0), 
		"hand eye offset pitch (double)").asDouble();
	fixedHandEyeOffsetRot[2] = rf.check("hand_eye_calib_offset_yaw", 
		Value(0.0), 
		"hand eye offset yaw (double)").asDouble();

	liftHandDeltaMM = rf.check("lift_hand_delta", 
		Value(20.0), 
		"lift hand by this value, in mm (double)").asDouble();

	if (rf.check("StartConfigTorsoArmFingers","Start configuration for torso and arm (10/19 Dof) [degree]"))
	{
		yarp::os::Bottle *v = rf.find("StartConfigTorsoArmFingers").asList();
		if (v->size()!=10 && v->size()!=19)
		{
			cout << "Expecting start configuration with 10 or with 19 values..." << endl;
		} else
		{
			initConfigTorsoArmFingers_rad.clear();
			for (int i=0;i<(int)v->size();i++)
			{
				initConfigTorsoArmFingers_rad.push_back((float)(v->get(i).asDouble()) * (float)M_PI / 180.0f);
			}

		}
	}
	VR_INFO << "Using robot base string " << robotBase << endl;

	// rpc handler port
	handlerPortName =  "/";
	handlerPortName += getName();         // use getName() rather than a literal 
	handlerPortName += "rpc:i";
	if (!handlerPort.open(handlerPortName.c_str())) {           
		cout << getName() << ": Unable to open port " << handlerPortName << endl;  
		return false;
	}

	attach(handlerPort);                  // attach to port

	setupPorts();
	if (!setupCartInterface())
		return false;
	
	checkConnections(false);

	controlWindow.reset(new SimoxGraspingPipelineControlWindow(this));

	return true;
}

bool SimoxGraspingPipelineControlModule::setupPorts()
{
	bool result = true;
	if (!yarp.exists(clientNameRobotViewer.c_str()))
	{
		if (!simoxRobotViewerPort.open(clientNameRobotViewer.c_str()))
		{
			VR_ERROR << "Could not open simoxRobotViewerPort..." << endl;
			result = false;
		}
	}
	if (yarp.exists(clientNameRobotViewer.c_str()) && !yarp.isConnected(clientNameRobotViewer.c_str(),serverNameRobotViewer.c_str()))
	{
		tryToConnect(clientNameRobotViewer,serverNameRobotViewer);
	}

	if (!yarp.exists(clientNameIkSolver.c_str()))
	{
		if (!simoxIkSolverPort.open(clientNameIkSolver.c_str()))
		{
			VR_ERROR << "Could not open port2..." << endl;
			result = false;
		}
	}
	if (yarp.exists(clientNameIkSolver.c_str()) && !yarp.isConnected(clientNameIkSolver.c_str(),serverNameIkSolver.c_str()))
	{
		if (tryToConnect(clientNameIkSolver,serverNameIkSolver))
		{
			// get name of kinematic chain (RobotNodeSet)
			currentRNSName = getIkRNS();
		}
	}

	// motion planner	
	if (!yarp.exists(clientNameMotionPlanner.c_str()))
	{
		if (!simoxMotionPlannerPort.open(clientNameMotionPlanner.c_str()))
		{
			VR_ERROR << "Could not open port3..." << endl;
			result = false;
		}
	}
	if (yarp.exists(clientNameMotionPlanner.c_str()) && !yarp.isConnected(clientNameMotionPlanner.c_str(),serverNameMotionPlanner.c_str()))
	{
		tryToConnect(clientNameMotionPlanner,serverNameMotionPlanner);
	}

	// GraspExecution	
	if (!yarp.exists(clientNameGraspExecution.c_str()))
	{
		if (!simoxGraspExecutionPort.open(clientNameGraspExecution.c_str()))
		{
			VR_ERROR << "Could not open port to GraspExecution..." << endl;
			result = false;
		}
	}
	if (yarp.exists(clientNameGraspExecution.c_str()) && !yarp.isConnected(clientNameGraspExecution.c_str(),serverNameGraspExecution.c_str()))
	{
		tryToConnect(clientNameGraspExecution,serverNameGraspExecution);
	}

	// lego localizer	
	if (!yarp.exists(clientNameLegoLocalizer.c_str()))
	{
		if (!simoxLegoLocalizerPort.open(clientNameLegoLocalizer.c_str()))
		{
			VR_ERROR << "Could not open port to LegoLocalizer..." << endl;
			result = false;
		}
	}
	if (yarp.exists(clientNameLegoLocalizer.c_str()) && !yarp.isConnected(clientNameLegoLocalizer.c_str(),serverNameLegoLocalizer.c_str()))
	{
		tryToConnect(clientNameLegoLocalizer,serverNameLegoLocalizer);
	}

	if (result)
		setupJointLimits();

	return result;
}

bool SimoxGraspingPipelineControlModule::tryToConnect(std::string &clientName, std::string &serverName)
{
	return yarp.connect(clientName.c_str(),serverName.c_str());
}
	

bool SimoxGraspingPipelineControlModule::setupCartInterface()
{
	bool result = true;
	// setup LEFT ARM
	std::string localLeftArmName = "/";
	localLeftArmName += getName();
	localLeftArmName += "/left_arm";
	Property optionsLeftArm;
	optionsLeftArm.put("device", "remote_controlboard");
	optionsLeftArm.put("local", localLeftArmName.c_str());      //local port names
	std::string remoteLeftArm = ("/" + robotBase + "/left_arm");
	optionsLeftArm.put("remote",remoteLeftArm.c_str());        //where we connect to
	if (!robotDeviceLeftArm.open(optionsLeftArm)) {
		printf("Device not available.  Here are the known devices:\n");
		printf("%s", Drivers::factory().toString().c_str());
		result = false;
	}
	robotDeviceLeftArm.view(encLeftArm);
	robotDeviceLeftArm.view(posLeftArm);
	int axesLeftArm;
	if (!encLeftArm || !encLeftArm->getAxes(&axesLeftArm) || axesLeftArm<=0) {
		printf("Could not get encoder values from left_arm\n");
		result = false;
	}
	jointValuesLeftArm.resize(axesLeftArm,0.0);
	posLeftArm->setPositionMode();

	// setup Right ARM
	std::string localRightArmName = "/";
	localRightArmName += getName();
	localRightArmName += "/right_arm";
	Property optionsRightArm;
	optionsRightArm.put("device", "remote_controlboard");
	optionsRightArm.put("local", localRightArmName.c_str());      //local port names
	std::string remoteRightArm = ("/" + robotBase + "/right_arm");
	optionsRightArm.put("remote",remoteRightArm.c_str());        //where we connect to
	if (!robotDeviceRightArm.open(optionsRightArm)) {
		printf("Device not available.  Here are the known devices:\n");
		printf("%s", Drivers::factory().toString().c_str());
		result = false;
	}
	robotDeviceRightArm.view(encRightArm);
	robotDeviceRightArm.view(posRightArm);
	int axesRightArm;
	if (!encRightArm || !encRightArm->getAxes(&axesRightArm) || axesRightArm<=0) {
		printf("Could not get encoder values from right_arm\n");
		result = false;
	}
	jointValuesRightArm.resize(axesRightArm,0.0);
	posRightArm->setPositionMode();


	// setup TORSO
	std::string localTorsoName = "/";
	localTorsoName += getName();
	localTorsoName += "/torso";
	Property optionsTorso;
	optionsTorso.put("device", "remote_controlboard");
	optionsTorso.put("local", localTorsoName.c_str());      //local port names
	std::string remoteTorso = ("/" + robotBase + "/torso");
	optionsTorso.put("remote", remoteTorso.c_str());     //where we connect to
	if (!robotDeviceTorso.open(optionsTorso)) {
		printf("Device not available.  Here are the known devices:\n");
		printf("%s", Drivers::factory().toString().c_str());
		result = false;
	}	
	robotDeviceTorso.view(encTorso);
	robotDeviceTorso.view(posTorso);
	int axesTorso;
	if (!encTorso || !encTorso->getAxes(&axesTorso) || axesTorso<=0) {
		printf("Could not get encoder values from Torso\n");
		result = false;
	}
	jointValuesTorso.resize(axesTorso,0.0);
	posTorso->setPositionMode();


	// setup Head
	std::string localHeadName = "/";
	localHeadName += getName();
	localHeadName += "/head";
	Property optionsHead;
	optionsHead.put("device", "remote_controlboard");
	optionsHead.put("local", localHeadName.c_str());      //local port names
	std::string remoteHead = ("/" + robotBase + "/head");
	optionsHead.put("remote", remoteHead.c_str());     //where we connect to
	if (!robotDeviceHead.open(optionsHead)) {
		printf("Device not available.  Here are the known devices:\n");
		printf("%s", Drivers::factory().toString().c_str());
		result = false;
	}	
	robotDeviceHead.view(encHead);
	robotDeviceHead.view(posHead);
	int axesHead;
	if (!encHead || !encHead->getAxes(&axesHead) || axesHead<=0) {
		printf("Could not get encoder values from Head\n");
		result = false;
	}
	jointValuesHead.resize(axesHead,0.0);
	posHead->setPositionMode();

	return result;
}

bool SimoxGraspingPipelineControlModule::writeToPort(yarp::os::Port& port, yarp::os::Bottle& cmd, yarp::os::Bottle& response, bool checkFirstResponseFor1, bool tryReconnect )
{
	if (port.getOutputCount()<=0)
	{
		VR_ERROR << "Port "  << port.getName().c_str() << " is not connected. Trying to reconnect...";
		if (checkConnections(tryReconnect))
		{
			cout << "OK" << endl;
		} else
		{
			cout << "FAILED" << endl;
			return false;
		}
	}
	bool writeOK = port.write(cmd,response);

	if (writeOK && checkFirstResponseFor1)
	{
		if (response.get(0).isVocab())
			writeOK = response.get(0).asVocab() == VOCAB_OK;
		else
			writeOK = response.get(0).asInt()==1;
	}
	return writeOK;
}

std::string SimoxGraspingPipelineControlModule::getIkRNS()
{
	std::string result;
	Bottle cmd;
	cmd.addString("get");
	cmd.addString("joints");
	Bottle response;

	bool writeOK = writeToPort(simoxIkSolverPort,cmd,response);
	cout << "Sending command <get> <joints>  to simoxIkSolverPort:" << writeOK << endl;
	cout << "Response: " << response.toString().c_str() << endl;
	if (writeOK)
	{
		result = response.get(1).asString().c_str();
		cout << "Retrieved name of RNS:" << result << endl;
	}
	return result;
}

bool SimoxGraspingPipelineControlModule::setupJointLimits()
{
	std::string result;
	Bottle cmd;
	cmd.addString("get");
	cmd.addString("jointLimits");
	Bottle response;
	bool writeOK = writeToPort(simoxIkSolverPort,cmd,response,true,false);
	cout << "Sending command <get> <jointLimits>  to simoxIkSolverPort:" << writeOK << endl;
	cout << "Response: " << response.toString().c_str() << endl;

	bool writeOK2 = false;
	bool writeOK3 = false;
	if (writeOK)
	{
		int joints = (response.size()-1)/2;
		cout << "nr joints:" << joints << endl;
		std::vector<float> lo,hi;
		for (int i=0;i<joints;i++)
		{
			lo.push_back((float)response.get(i*2+1).asDouble());
			hi.push_back((float)response.get(i*2+2).asDouble());
		}
	

		Bottle cmd2;
		cmd2.addString("set");
		cmd2.addString("jointLimits");
		cmd2.addString(currentRNSName.c_str());
		for (int i=0;i<joints;i++)
		{
			cmd2.addDouble((double)lo[i]);
			cmd2.addDouble((double)hi[i]);
		}

		// send to robot viewer
		Bottle response2;
		writeOK2 = writeToPort(simoxRobotViewerPort,cmd2,response2,true,false);
		
		if (!writeOK2)
		{
			cout << "Failed to setup joint limits for RobotViewer..." << endl;
		} else
			cout << "ok: Set joint limits for RobotViewer." << endl;

		// send to motion planner
		Bottle response3;
		writeOK3 = writeToPort(simoxMotionPlannerPort,cmd2,response3,true,false);
		
		if (!writeOK3)
		{
			cout << "Failed to setup joint limits for simoxMotionPlannerPort..." << endl;
		} else
			cout << "ok: Set joint limits for simoxMotionPlannerPort." << endl;

	}

	// set eef

	Bottle cmdEEF,responseEEF;
	cmdEEF.addString("set");
	cmdEEF.addString("eef");
	if (isLeft)
		cmdEEF.addString("Left Hand");
	else
		cmdEEF.addString("Right Hand");
	bool writeOKEEF = writeToPort(simoxRobotViewerPort,cmdEEF,responseEEF,true,false);
	if (!writeOKEEF)
	{
		cout << "Failed to set eef in RobotViewer..." << endl;
	} else
		cout << "ok: selecting EEF in RobotViewer:" << responseEEF.toString().c_str() << endl;

	// setup preshape
	Bottle cmdPreshape,responsePreshape;
	cmdPreshape.addString("enable");
	cmdPreshape.addString("updateFingers");
	if (isLeft)
		cmdPreshape.addString("left");
	else
		cmdPreshape.addString("right");
	cmdPreshape.addString("off");
	bool writeOKPreshape = writeToPort(simoxRobotViewerPort,cmdPreshape,responsePreshape,true,false);
	if (!writeOKPreshape)
	{
		cout << "Failed to disable finger updates in RobotViewer..." << endl;
	} else
		cout << "ok: disable finger updates in RobotViewer." << endl;

	Bottle cmdPreshape2,responsePreshape2;
	cmdPreshape2.addString("set");
	cmdPreshape2.addString("eefPreshape");
	cmdPreshape2.addString(eefPreshapeName.c_str());
	bool writeOKPreshape2 = writeToPort(simoxRobotViewerPort,cmdPreshape2,responsePreshape2,true,false);
	if (!writeOKPreshape2)
	{
		cout << "Failed to set eef preshape in RobotViewer..." << endl;
	} else
		cout << "ok: set eef preshape in RobotViewer." << endl;

	// clean up any objects if present
	Bottle cmdClean,responseClean;
	cmdClean.addString("resetObjects");
	bool wo = writeToPort(simoxRobotViewerPort,cmdClean,responseClean,false,false);
	
	return writeOK && writeOK2 && writeOK3 && writeOKEEF && writeOKPreshape && writeOKPreshape2;

}

bool SimoxGraspingPipelineControlModule::updateModule()
{
	if (controlWindow && controlWindow->wasClosed())
	{
		cout << "Window closed, exiting..." << endl;
		return false;
	}
	controlWindow->ping();
	if (qApp)
		qApp->processEvents();
	return true;
}

bool SimoxGraspingPipelineControlModule::respond( const Bottle& command, Bottle& reply )
{
	std::vector<std::string> helpMessages;

	helpMessages.push_back(string(getName().c_str()) + " commands are: \n" );
	helpMessages.push_back("help");
	helpMessages.push_back("quit");
	helpMessages.push_back("add object <name> <filename> ... load object from file <filename> and add to viewer. Chose <name> as you want, it will be key for further access.");
	helpMessages.push_back("remove object <name> ... remove object <name> from viewer");
	helpMessages.push_back("set object position <name> x y z ... set object <name> to position (x,y,z)");
	helpMessages.push_back("set object orientation <name> roll pitch yaw ... set RPY orientation of object <name>");
	helpMessages.push_back("show grasp <object> <grasp> ... show grasp of object <object> with name <grasp> in viewer");
	//helpMessages.push_back("show reachablegrasps <object> on/off ... show reachable grasps of object <object> in viewer");

	string helpMessage;
	for (size_t i=0;i<helpMessages.size();i++)
		helpMessage = helpMessage + helpMessages[i] + string("\n");

	bool eaten = false;
	reply.clear(); 

	if (!controlWindow)
	{
		reply.addString ("Window closed, could not perform any actions. Quitting...");
		return false;   
	}

	if (command.get(0).asString()=="quit") {
		reply.addString("quitting");
		return false;     
	}
	else if (command.get(0).asString()=="help") {
		cout << helpMessage;
		reply.addString("Help printed...");
		eaten = true;
	}
	else if (command.get(0).asString()=="add") 
	{
		if (command.get(1).asString()=="object") 
		{
			ConstString name = command.get(2).asString();
			ConstString filename = command.get(3).asString();
			reply.addString("Loading object <");
			reply.addString(name.c_str());
			reply.addString(">...");
			bool loadOK = controlWindow->setObject(name.c_str(),filename.c_str());
			if (loadOK)
				reply.addString("ok");
			else
				reply.addString("failed");
			eaten = true;
		}
	} else if (command.get(0).asString()=="show") 
	{
		if (command.get(1).asString()=="grasp") 
		{
			ConstString objName = command.get(2).asString();
			ConstString graspName = command.get(3).asString();
			reply.addString("Showing grasp <");
			reply.addString(graspName.c_str());
			reply.addString(">...");
			controlWindow->selectGrasp(objName.c_str(),graspName.c_str());
			eaten = true;
		} /*else if (command.get(1).asString()=="reachablegrasps") 
		{
			ConstString objName = command.get(2).asString();
			ConstString onOff = command.get(3).asString();
			bool on = true;
			if (onOff == "off")
				on = false;
			if (on)
				reply.addString("Showing reachable grasps for object <");
			else
				reply.addString("Hiding reachable grasps for object <");
			reply.addString(objName.c_str());
			reply.addString(">...");
			controlWindow->showReachableGrasps(objName.c_str(),on);
			eaten = true;
		}*/
	} else if (command.get(0).asString()=="remove") 
	{
		if (command.get(1).asString()=="object") 
		{
			ConstString name = command.get(2).asString();
			reply.addString("Removing object <");
			reply.addString(name.c_str());
			reply.addString(">");
			controlWindow->removeObject(name.c_str());
			eaten = true;
		}
	} else if (command.get(0).asString()=="set") 
	{
		if (command.get(1).asString()=="object") 
		{
			if (command.get(2).asString()=="position") 
			{
				ConstString name = command.get(3).asString();
				double x = command.get(4).asDouble();
				double y = command.get(5).asDouble();
				double z = command.get(6).asDouble();

				reply.addString("Setting object <");
				reply.addString(name);
				reply.addString("> to position ");
				reply.addDouble(x);
				reply.addDouble(y);
				reply.addDouble(z);
	
				Eigen::Matrix4f p = controlWindow->getObjectPose(name.c_str());
				p.block(0,3,3,1) = Eigen::Vector3f(x,y,z);
				controlWindow->setObjectPose(name.c_str(),p);
				eaten = true;
			} else if (command.get(2).asString()=="orientation") 
			{
				ConstString name = command.get(3).asString();
				double r = command.get(4).asDouble();
				double p = command.get(5).asDouble();
				double y = command.get(6).asDouble();

				reply.addString("Setting object <");
				reply.addString(name);
				reply.addString("> to RPY orientation ");
				reply.addDouble(r);
				reply.addDouble(p);
				reply.addDouble(y);
				Eigen::Matrix4f po = controlWindow->getObjectPose(name.c_str());
				Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
				MathTools::rpy2eigen4f(r,p,y,m);
				m.block(0,3,3,1) = po.block(0,3,3,1);
				controlWindow->setObjectPose(name.c_str(),m);
				eaten = true;
			}
		}
	} 

	if (!eaten || reply.isNull())
	{
		reply.addString("unknown command:\n");
		reply.addString(command.toString());
		reply.addString("\nTry 'help'");
		cout << helpMessage;
	}
	return true;
}

bool SimoxGraspingPipelineControlModule::interruptModule()
{
	handlerPort.interrupt();
	printf ("INTERRUPT\n");
	return true;
}

bool SimoxGraspingPipelineControlModule::close()
{
	handlerPort.close();

	if (controlWindow)
		controlWindow->quit();

	// reports a memory leak in Coin's font manager?!
	//SoQt::done();

	return true;
}

double SimoxGraspingPipelineControlModule::getPeriod()
{
	// 50 fps
	return 0.02;
}

bool SimoxGraspingPipelineControlModule::setObject( const std::string &objectName, std::string filename )
{
	bool writeOK = false;
	bool writeOK2 = false;
	bool writeOK3 = false;

	Bottle cmd;
	cmd.addString("add");
	cmd.addString("object");
	cmd.addString(objectName.c_str());
	cmd.addString(filename.c_str());
	Bottle response;
	writeOK = writeToPort(simoxRobotViewerPort,cmd,response);
	cout << "Sending command <add> <object> <" << objectName << "> <" << filename << "> to simoxRobotViewerPort:" << writeOK << endl;
	cout << "Response: " << response.toString().c_str() << endl;
		
	Bottle response2;
	writeOK2 = writeToPort(simoxIkSolverPort,cmd,response2);
	cout << "Sending command <add> <object> <" << objectName << "> <" << filename << "> to simoxIkSolverPort:" << writeOK2 << endl;
	cout << "Response: " << response2.toString().c_str() << endl;
	
	Bottle response3;
	writeOK3 = writeToPort(simoxMotionPlannerPort,cmd,response3);
	cout << "Sending command <add> <object> <" << objectName << "> <" << filename << "> to simoxMotionPlannerPort:" << writeOK3 << endl;
	cout << "Response: " << response3.toString().c_str() << endl;

	
	return (writeOK && writeOK2 && writeOK3);
}


bool SimoxGraspingPipelineControlModule::showGrasp( const std::string &objectName, const std::string &graspName )
{
	Bottle cmd;
	cmd.addString("show");
	cmd.addString("grasp");
	cmd.addString(objectName.c_str());
	cmd.addString("on");
	cmd.addString(graspName.c_str());
	Bottle response;
	bool writeOK = writeToPort(simoxRobotViewerPort,cmd,response);
	cout << "Sending command <show> <grasp> <" << objectName << "> <" << graspName << "> to simoxRobotViewerPort:" << writeOK << endl;
	cout << "Response: " << response.toString().c_str() << endl;
	
	/*Bottle response2;
	bool writeOK2 = simoxIkSolverPort.write(cmd,response2);
	cout << "Sending command <show> <grasp> <" << objectName << "> <" << graspName << "> to simoxIkSolverPort:" << writeOK2 << endl;
	cout << "Response: " << response2.toString().c_str() << endl;
	*/
	//bool responseOK2 = response2.get(0).asInt()==1;

	return (writeOK);
}


bool SimoxGraspingPipelineControlModule::showReachableGrasps( const std::string &objectName, std::map< std::string, float> &grasps,  bool enable )
{
	Bottle cmd;
	cmd.addString("show");
	cmd.addString("grasp");
	cmd.addString(objectName.c_str());
	if (enable)
		cmd.addString("on");
	else
		cmd.addString("off");


	std::map< std::string, float>::iterator i = grasps.begin();
	while (i!=grasps.end())
	{
		cmd.addString(i->first.c_str());
		i++;
	}
	Bottle response;
	bool writeOK = writeToPort(simoxRobotViewerPort,cmd,response);
	cout << "Sending command: <" << cmd.toString().c_str() << "> to simoxRobotViewerPort:" << writeOK << endl;
	cout << "Response: " << response.toString().c_str() << endl;
	
	/*
	cmd.addString("reachablegrasps");
	cmd.addString(objectName.c_str());
	if (enable)
		cmd.addString("on");
	else
		cmd.addString("off");
	Bottle response;
	bool writeOK = writeToPort(simoxRobotViewerPort,cmd,response);
	cout << "Sending command <show> <reachablegrasps> <" << objectName << "> to simoxRobotViewerPort:" << writeOK << endl;
	cout << "Response: " << response.toString().c_str() << endl;
	*/


	/*Bottle response2;
	bool writeOK2 = simoxIkSolverPort.write(cmd,response2);
	cout << "Sending command <show> <reachablegrasps> <" << objectName << "> to simoxIkSolverPort:" << writeOK2 << endl;
	cout << "Response: " << response2.toString().c_str() << endl;*/

	return (writeOK);
}

bool SimoxGraspingPipelineControlModule::removeObject( const std::string &objectName )
{
	Bottle cmd;
	cmd.addString("remove");
	cmd.addString("object");
	cmd.addString(objectName.c_str());
	Bottle response;
	bool writeOK = writeToPort(simoxRobotViewerPort,cmd,response);
	cout << "Sending command <remove> <object> <" << objectName << ">to simoxRobotViewerPort:" << writeOK << endl;
	cout << "Response: " << response.toString().c_str() << endl;

	Bottle response2;
	bool writeOK2 = writeToPort(simoxIkSolverPort,cmd,response2);
	cout << "Sending command <remove> <object> <" << objectName << "> to simoxIkSolverPort:" << writeOK2 << endl;
	cout << "Response: " << response2.toString().c_str() << endl;

	Bottle response3;
	bool writeOK3 = writeToPort(simoxMotionPlannerPort,cmd,response3);
	cout << "Sending command <remove> <object> <" << objectName << "> to simoxMotionPlannerPort:" << writeOK3 << endl;
	cout << "Response: " << response3.toString().c_str() << endl;

	return (writeOK && writeOK2 &&  writeOK3);
}

bool SimoxGraspingPipelineControlModule::setObjectPose( const std::string &objectName, const Eigen::Matrix4f &pose )
{
	Eigen::Vector3f pos = MathTools::getTranslation(pose);
	Eigen::Vector3f rpy;
	MathTools::eigen4f2rpy(pose,rpy);

	Bottle cmdOri;
	cmdOri.addString("set");
	cmdOri.addString("object");
	cmdOri.addString("orientation");
	cmdOri.addString(objectName.c_str());
	cmdOri.addDouble(rpy(0));cmdOri.addDouble(rpy(1));cmdOri.addDouble(rpy(2));
	Bottle response;
	bool writeOK = writeToPort(simoxRobotViewerPort,cmdOri,response);
	cout << "Sending command <set> <object> <orientation> <" << objectName << "> to simoxRobotViewerPort:" << writeOK << endl;
	cout << "Response: " << response.toString().c_str() << endl;

	Bottle response2;
	bool writeOK2 = writeToPort(simoxIkSolverPort,cmdOri,response2);
	cout << "Sending command <set> <object> <orientation> <" << objectName << "> to simoxIkSolverPort:" << writeOK2 << endl;
	cout << "Response: " << response2.toString().c_str() << endl;

	Bottle response6;
	bool writeOK6 = writeToPort(simoxMotionPlannerPort,cmdOri,response6);
	cout << "Sending command <set> <object> <orientation> <" << objectName << "> to simoxMotionPlannerPort:" << writeOK6 << endl;
	cout << "Response: " << response6.toString().c_str() << endl;
	
	Bottle cmdPos;
	cmdPos.addString("set");
	cmdPos.addString("object");
	cmdPos.addString("position");
	cmdPos.addString(objectName.c_str());
	cmdPos.addDouble(pos(0));cmdPos.addDouble(pos(1));cmdPos.addDouble(pos(2));
	Bottle response3;
	bool writeOK3 = writeToPort(simoxRobotViewerPort,cmdPos,response3);
	cout << "Sending command <set> <object> <position> <" << objectName << "> to simoxRobotViewerPort:" << writeOK3 << endl;
	cout << "Response: " << response3.toString().c_str() << endl;

	Bottle response4;
	bool writeOK4 = writeToPort(simoxIkSolverPort,cmdPos,response4);
	cout << "Sending command <set> <object> <position> <" << objectName << "> to simoxIkSolverPort:" << writeOK4 << endl;
	cout << "Response: " << response4.toString().c_str() << endl;

	Bottle response5;
	bool writeOK5 = writeToPort(simoxMotionPlannerPort,cmdPos,response5);
	cout << "Sending command <set> <object> <position> <" << objectName << "> to simoxMotionPlannerPort:" << writeOK5 << endl;
	cout << "Response: " << response5.toString().c_str() << endl;


	return (writeOK && writeOK2 && writeOK3 && writeOK4 && writeOK5 && writeOK6);
}

Eigen::Matrix4f SimoxGraspingPipelineControlModule::getObjectPose( const std::string &objectName )
{
	Eigen::Matrix4f m = Eigen::Matrix4f::Identity();

	Bottle cmd2;
	cmd2.addString("get");
	cmd2.addString("object");
	cmd2.addString("orientation");
	cmd2.addString(objectName.c_str());
	Bottle response2;
	bool writeOK2 = writeToPort(simoxIkSolverPort,cmd2,response2);
	cout << "Sending command <get> <object> <orientation> <" << objectName << "> to simoxIkSolverPort:" << writeOK2 << endl;
	cout << "Response: " << response2.toString().c_str() << endl;
	if (writeOK2)
	{
		float r = (float)response2.get(1).asDouble();
		float p = (float)response2.get(2).asDouble();
		float y = (float)response2.get(3).asDouble();
		MathTools::rpy2eigen4f(r,p,y,m);
	}

	Bottle cmd;
	cmd.addString("get");
	cmd.addString("object");
	cmd.addString("position");
	cmd.addString(objectName.c_str());
	Bottle response;
	bool writeOK = writeToPort(simoxIkSolverPort,cmd,response);
	cout << "Sending command <get> <object> <position> <" << objectName << "> to simoxIkSolverPort:" << writeOK << endl;
	cout << "Response: " << response.toString().c_str() << endl;
	
	if (writeOK)
	{
		Eigen::Vector3f pos;
		pos(0) = (float)response.get(1).asDouble();
		pos(1) = (float)response.get(2).asDouble();
		pos(2) = (float)response.get(3).asDouble();
		m.block(0,3,3,1) = pos;
	}

	return m;
}

bool SimoxGraspingPipelineControlModule::getReachableGrasps( const std::string &objectName, std::vector<std::string> &storeGrasps, std::vector<float> &storeQuality )
{
	storeGrasps.clear();
	storeQuality.clear();
	Bottle cmd;
	Bottle response;
	cmd.clear();
	cmd.addString("has");
	cmd.addString("manipulability");
	response.clear();
	bool writeOK = writeToPort(simoxIkSolverPort,cmd,response);
	cout << "Sending command <has> <manipulability> to simoxIkSolverPort:" << writeOK << endl;
	cout << "Response: " << response.toString().c_str() << endl;
	if (response.get(0).asInt()==1)
	{
		cout << "Using manipulability measures" << endl;
		cmd.clear();
		cmd.addString("get");
		cmd.addString("manipulabilityGrasps");
		cmd.addString(objectName.c_str());
		response.clear();
		writeOK = writeToPort(simoxIkSolverPort,cmd,response);
		cout << "Sending command <get> <manipulabilityGrasps> <" << objectName << "> to simoxIkSolverPort:" << writeOK << endl;
		cout << "Response: " << response.toString().c_str() << endl;

		if (writeOK)
		{
			int nr =  response.get(1).asInt();
			cout << "Received " << nr << " reachable grasps with manipulability score" << endl;
			for (int i=0;i<nr;i++)
			{
				std::string g = response.get(2+ i*2).asString().c_str();
				float q = (float)response.get(3+ i*2).asDouble();

				cout << "Grasp " << i <<": " << g << " : " << q << endl;
				storeGrasps.push_back(g);
				storeQuality.push_back(q);
			}
			return true;
		}

	} else
	{
		cout << "Using reachability..." << endl;
		cmd.clear();
		cmd.addString("get");
		cmd.addString("reachablegrasps");
		cmd.addString(objectName.c_str());
		response.clear();
		writeOK = writeToPort(simoxIkSolverPort,cmd,response);
		cout << "Sending command <get> <reachablegrasps> <" << objectName << "> to simoxIkSolverPort:" << writeOK << endl;
		cout << "Response: " << response.toString().c_str() << endl;

		if (writeOK)
		{
			int nr =  response.get(1).asInt();
			cout << "Received " << nr << " reachable grasps" << endl;
			for (int i=0;i<nr;i++)
			{
				std::string g = response.get(i+2).asString().c_str();
				storeGrasps.push_back(g);
				storeQuality.push_back(0.0f);
			}
			return true;
		}
	}
	return false;
}

bool SimoxGraspingPipelineControlModule::showCurrentRobotState()
{
	Bottle cmd;
	cmd.addString("show");
	cmd.addString("robotState");
	Bottle response;
	bool writeOK = writeToPort(simoxRobotViewerPort,cmd,response);
	cout << "Sending command <show> <robotState> to simoxRobotViewerPort:" << writeOK << endl;
	cout << "Response: " << response.toString().c_str() << endl;

	return writeOK;
}

bool SimoxGraspingPipelineControlModule::showConfiguration(std::vector<float> &config)
{
	if (currentRNSName.empty())
	{
		cout << "Do not know which RNS (kin chain) is used for IK solving, could not send config to viewer..." << endl;
		return false;
	}
	Bottle cmd;
	cmd.addString("show");
	cmd.addString("config");
	cmd.addString(currentRNSName.c_str());
	for (size_t i=0;i<config.size();i++)
	{
		cmd.addDouble((double)config[i]);
	}

	Bottle response;
	bool writeOK = writeToPort(simoxRobotViewerPort,cmd,response);
	cout << "Sending command <show> <config> <" << currentRNSName <<"> <j1...jn> to simoxRobotViewerPort:" << writeOK << endl;
	cout << "Response: " << response.toString().c_str() << endl;
	
	return writeOK;
}


bool SimoxGraspingPipelineControlModule::searchIK( const std::string &objectName, const std::string &graspName, std::vector<float> &storeJointValues, float& storeManipulabilityQuality )
{
	storeJointValues.clear();
	storeManipulabilityQuality = 0.0f;
	Bottle cmd;
	cmd.addString("get");
	cmd.addString("graspikSolution");
	cmd.addString(objectName.c_str());
	cmd.addString(graspName.c_str());
	Bottle response;
	bool writeOK = writeToPort(simoxIkSolverPort,cmd,response);
	cout << "Sending command <get> <graspikSolution> <" << objectName << ">  <" << graspName << "> to simoxIkSolverPort:" << writeOK << endl;
	cout << "Response: " << response.toString().c_str() << endl;

	if (writeOK)
	{
		cout << "ik solution:" << endl;
		for (int i=0;i<response.size()-2;i++)
		{
			float v = (float)response.get(i+1).asDouble();
			storeJointValues.push_back(v);
		}
		MathTools::print(storeJointValues);
		storeManipulabilityQuality = (float)response.get(response.size()-1).asDouble();
		cout << "Manipulability: " << storeManipulabilityQuality << endl;
	} else
	{
		cout << "IK search failed" << endl;
	}
	return writeOK;
}

bool SimoxGraspingPipelineControlModule::planMotion( const std::vector<float> startConfig, const std::vector<float> goalConfig, std::vector< std::vector<float> > &storePath)
{
	storePath.clear();
	if (goalConfig.size()!=startConfig.size() || startConfig.size()==0)
	{
		VR_ERROR << "Wrong size... start:" << startConfig.size() << ", goal:" << goalConfig.size() << endl;
		return false;
	}

	Bottle cmd;
	cmd.addString("planMotion");
	for (size_t i=0;i<startConfig.size();i++)
		cmd.addDouble((double)startConfig[i]);
	for (size_t i=0;i<goalConfig.size();i++)
		cmd.addDouble((double)goalConfig[i]);

	Bottle response;
	bool writeOK = writeToPort(simoxMotionPlannerPort,cmd,response);
	cout << "Sending command <planMotion> <startConf> <goalConf> to simoxMotionPlannerPort:" << writeOK << endl;
	cout << "Response: " << response.toString().c_str() << endl;

	if (writeOK)
	{
		int nrJoints = (int)startConfig.size();
		yarp::os::Bottle* l = response.get(1).asList();
		int nrPts = l->size();

		cout << "Successfully planned motion with " << nrPts << " path points (c-space)" << endl;
		for (int i=0;i<nrPts;i++)
		{
			std::vector<float> p;
			yarp::os::Bottle* c = l->get(i).asList();
			if (c->size()!=nrJoints)
			{
				cout << "Error in result config " << i << ". Expecting " << nrJoints << " joint values, but received " << c->size() << endl;
				return false;
			}
			cout << "## Config " << i << ":" << endl;
			for (int j=0;j<nrJoints;j++)
			{
				float v = (float)c->get(j).asDouble();
				p.push_back(v);
				cout << v << ",";
			}
			cout << endl;
			storePath.push_back(p);
		}
	} else
	{
		cout << "Motion Planning failed" << endl;
	}
	return writeOK;
}


bool SimoxGraspingPipelineControlModule::goToPoseHipArm(bool left, std::vector<float> jV_rad)
{
	// jV_rad.size() = 10 = 3(torso) + 7(arm)
	// jV_rad.size() = 19 = 3(torso) + 7(arm) + 9(fingers)

	yarp::sig::Vector* jv_arm;
	yarp::dev::IEncoders* enc_arm;
	yarp::dev::IPositionControl* pos_arm;
	if (left)
	{
		jv_arm = &jointValuesLeftArm;
		enc_arm = encLeftArm;
		pos_arm = posLeftArm;
	}
	else
	{
		jv_arm = &jointValuesRightArm;
		enc_arm = encRightArm;
		pos_arm = posRightArm;
	}
	if ( (jV_rad.size()!=10 && jV_rad.size()!=19) || jv_arm->size()<7 || 
		jointValuesTorso.size()<3 || 
		!robotDeviceTorso.isValid() || !enc_arm || !encTorso ||
		!pos_arm || !posTorso)
	{
		VR_WARNING << "Not initialized correctly..." << endl;
		return false;
	}

	if (!enc_arm->getEncoders(jv_arm->data()))
	{
		VR_ERROR << "Could not get joint values of arm..." << endl;
		return false;
	}
	if (!encTorso->getEncoders(jointValuesTorso.data()))
	{
		VR_ERROR << "Could not get joint values of torso..." << endl;
		return false;
	}

	cout << "Joint values [rad]:";
	for (int i=0;i<(int)jV_rad.size();i++)
		cout << jV_rad[i] << ",";
	cout << endl;
	jointValuesTorso[0] = (double)jV_rad[2] * 180.0 / M_PI; // xchange yaw and pitch
	jointValuesTorso[1] = (double)jV_rad[1] * 180.0 / M_PI;
	jointValuesTorso[2] = (double)jV_rad[0] * 180.0 / M_PI;
	int armJ = 7;
	if (jV_rad.size()==19)
		armJ = 16;
	for (int i=0;i<armJ;i++)
		(*jv_arm)[i] = (double)jV_rad[i+3] * 180.0 / M_PI;

	cout << "JV Torso:" << endl;
	for (unsigned int i=0;i<jointValuesTorso.size();i++)
		cout << jointValuesTorso[i] << ",";
	cout << endl;
	cout << "JV Arm:" << endl;
	for (unsigned int i=0;i<jv_arm->size();i++)
		cout << (*jv_arm)[i] << ",";
	cout << endl;

	bool res = true;
	if (!posTorso->positionMove(jointValuesTorso.data()))
		res = false;
	if (!pos_arm->positionMove(jv_arm->data()))
		res = false;
	return res;
}

bool SimoxGraspingPipelineControlModule::goToPoseHipArm( std::vector<float> jV_rad )
{
	return goToPoseHipArm(isLeft,jV_rad);
}

bool SimoxGraspingPipelineControlModule::queryHandEyeOffset( const std::string &objectName, float &x, float &y, float &z, float &ro, float &pi, float &ya)
{
	/*Eigen::Matrix4f objPose = getObjectPose(objectName);
	if (objPose.isIdentity())
	{
		cout << "Do not know object " << objectName << endl;
		return false;
	}*/
	//std::string handEyeCalibPort;
	cout << "TODO: Change this, currently a fixed calibration offset is used: " << fixedHandEyeOffset(0) << "," << fixedHandEyeOffset(1) << "," << fixedHandEyeOffset(2) << endl;
	x = fixedHandEyeOffset[0];
	y = fixedHandEyeOffset[1];
	z = fixedHandEyeOffset[2];
	ro = fixedHandEyeOffsetRot[0];
	pi = fixedHandEyeOffsetRot[1];
	ya = fixedHandEyeOffsetRot[2];
	//handEyeOffsetChanged(objectName,x,y,z);
	return true;
}

bool SimoxGraspingPipelineControlModule::getCurrentConfigArm( bool left, std::vector<float> &jV_rad, bool withTorso )
{
	yarp::sig::Vector* jv_arm;
	yarp::dev::IEncoders* enc_arm;
	yarp::dev::IPositionControl* pos_arm;
	if (left)
	{
		jv_arm = &jointValuesLeftArm;
		enc_arm = encLeftArm;
		pos_arm = posLeftArm;
	}
	else
	{
		jv_arm = &jointValuesRightArm;
		enc_arm = encRightArm;
		pos_arm = posRightArm;
	}
	if ( jv_arm->size()<7 || 
		jointValuesTorso.size()<3 || 
		!robotDeviceTorso.isValid() || !enc_arm || !encTorso ||
		!pos_arm || !posTorso)
	{
		VR_WARNING << "Not initialized correctly..." << endl;
		return false;
	}

	jV_rad.clear();

	if (!enc_arm->getEncoders(jv_arm->data()))
	{
		VR_ERROR << "Could not get joint values of arm..." << endl;
		return false;
	}

	if (withTorso)
	{
		if (!encTorso->getEncoders(jointValuesTorso.data()))
		{
			VR_ERROR << "Could not get joint values of torso..." << endl;
			return false;
		}

		jV_rad.push_back( (float)jointValuesTorso[2] * (float)M_PI / 180.0f );// xchange yaw and pitch
		jV_rad.push_back( (float)jointValuesTorso[1] * (float)M_PI / 180.0f );
		jV_rad.push_back( (float)jointValuesTorso[0] * (float)M_PI / 180.0f );
	}


	for (size_t i=0;i<7;i++)
		jV_rad.push_back( (float)(*jv_arm)[i] * (float)M_PI / 180.0f );
	return true;
}

bool SimoxGraspingPipelineControlModule::getCurrentConfigArm( std::vector<float> &jV_rad, bool withTorso /*= true*/ )
{
	return getCurrentConfigArm(isLeft,jV_rad,withTorso);
}

void SimoxGraspingPipelineControlModule::goToInitPose()
{
	if (initConfigTorsoArmFingers_rad.size()==10 || initConfigTorsoArmFingers_rad.size()==19)
	{
		goToPoseHipArm(isLeft,initConfigTorsoArmFingers_rad);
	}
}

bool SimoxGraspingPipelineControlModule::showMotion( bool enable, std::vector< std::vector<float> > path )
{
	Bottle cmd;
	cmd.addString("show");
	cmd.addString("motion");
	cmd.addString("GraspingMotion");
	cmd.addString(currentRNSName.c_str());
	if (!enable)
	{
		cmd.addString("off");
	} else
	{
		cmd.addString("on");
		for (size_t i=0;i<path.size();i++)
		{
			yarp::os::Bottle &c = cmd.addList();
			for (size_t j=0;j<path[i].size();j++)
			{
				c.addDouble((double)path[i][j]);
			}
		}
	}
	
	Bottle response;
	bool writeOK = writeToPort(simoxRobotViewerPort,cmd,response);
	cout << "Sending command <show> <motion> <GraspingMotion> ";
	if (enable)
		cout << "<on>";
	else
		cout << "<off>";
	cout << " <c1> .. <cn> to simoxRobotViewerPort:" << writeOK << endl;
	cout << "Response: " << response.toString().c_str() << endl;

	return writeOK;
}

bool SimoxGraspingPipelineControlModule::checkConnections(bool tryToReconnect)
{
	bool result = true;
	if (!yarp.exists(clientNameRobotViewer.c_str()))
		result = false;
	if (!yarp.exists(clientNameIkSolver.c_str()))
		result = false;
	if (!yarp.exists(clientNameMotionPlanner.c_str()))
		result = false;
	if (!yarp.exists(clientNameLegoLocalizer.c_str()))
		result = false;
	if (!yarp.exists(clientNameGraspExecution.c_str()))
		result = false;

	if (!yarp.isConnected(clientNameRobotViewer.c_str(),serverNameRobotViewer.c_str()))
		result = false;
	if (!yarp.isConnected(clientNameIkSolver.c_str(),serverNameIkSolver.c_str()))
		result = false;
	if (!yarp.isConnected(clientNameMotionPlanner.c_str(),serverNameMotionPlanner.c_str()))
		result = false;
	if (!yarp.isConnected(clientNameLegoLocalizer.c_str(),serverNameLegoLocalizer.c_str()))
		result = false;
	if (!yarp.isConnected(clientNameGraspExecution.c_str(),serverNameGraspExecution.c_str()))
		result = false;
	if (!result && tryToReconnect)
	{
		setupPorts();
		connected = checkConnections(false);
		return connected;
	}

	connected = result;
	return connected;
}

bool SimoxGraspingPipelineControlModule::localizeLego( bool updatePose, Eigen::Matrix4f &m, bool searchForObstacle, Eigen::Matrix4f &obst_pose  )
{
	if (!checkConnections(true))
		return false;

	Bottle cmd,response;
	if (updatePose)
		cmd.addString("updateLocalize");
	else
		cmd.addString("localize");

	if (searchForObstacle)
		cmd.addString("withObstacle");

	bool writeOK = simoxLegoLocalizerPort.write(cmd,response);
	cout << "Sending command <" << cmd.toString() << "> to simoxLegoLocalizerPort:" << writeOK << endl;
	cout << "Response: " << response.toString().c_str() << endl;
	bool responseOK = response.get(0).asInt()==1;
	if (!responseOK)
	{
		VR_ERROR << "Could not localize LEGO" << endl;
		return false;
	}
	Eigen::Vector3f p;
	p(0) = (float)response.get(1).asDouble();//[m]
	p(1) = (float)response.get(2).asDouble();//[m]
	p(2) = (float)response.get(3).asDouble();//[m]
	MathTools::Quaternion q;
	q.x = (float)response.get(4).asDouble();
	q.y = (float)response.get(5).asDouble();
	q.z = (float)response.get(6).asDouble();
	q.w = (float)response.get(7).asDouble();


	// convert to global pose
	Bottle cmd1,response1;
	cmd1.addString("convertCoordinates");
	// icubRoot to global
	cmd1.addString("iCubRoot");
	cmd1.addString("globalpose");
	cmd1.addDouble((double)p(0));//[m]
	cmd1.addDouble((double)p(1));//[m]
	cmd1.addDouble((double)p(2));//[m]
	cmd1.addDouble((double)q.x);
	cmd1.addDouble((double)q.y);
	cmd1.addDouble((double)q.z);
	cmd1.addDouble((double)q.w);
	cout << "SEND " << cmd1.toString() << endl;
	simoxRobotViewerPort.write(cmd1,response1);
	cout << "RECEIVE  " << response1.toString() << endl;
	if (response1.get(0).asInt() == 1)
	{
		// convert ok
		p(0) = (float)response1.get(1).asDouble();
		p(1) = (float)response1.get(2).asDouble();
		p(2) = (float)response1.get(3).asDouble();
		q.x = (float)response1.get(4).asDouble();
		q.y = (float)response1.get(5).asDouble();
		q.z = (float)response1.get(6).asDouble();
		q.w = (float)response1.get(7).asDouble();
	}

	p *= 1000.0f; // mm

	m = MathTools::quat2eigen4f(q);
	m.block(0,3,3,1) = p;


	if (searchForObstacle && response.size()>=15)
	{
		Eigen::Vector3f po;
		po(0) = (float)response.get(8).asDouble();//[m]
		po(1) = (float)response.get(9).asDouble();//[m]
		po(2) = (float)response.get(10).asDouble();//[m]
		MathTools::Quaternion qo;
		qo.x = (float)response.get(11).asDouble();
		qo.y = (float)response.get(12).asDouble();
		qo.z = (float)response.get(13).asDouble();
		qo.w = (float)response.get(14).asDouble();
		// convert to global pose
		Bottle cmd2,response2;
		cmd2.addString("convertCoordinates");
		// icubRoot to global
		cmd2.addString("iCubRoot");
		cmd2.addString("globalpose");
		cmd2.addDouble((double)po(0));//[m]
		cmd2.addDouble((double)po(1));//[m]
		cmd2.addDouble((double)po(2));//[m]
		cmd2.addDouble((double)qo.x);
		cmd2.addDouble((double)qo.y);
		cmd2.addDouble((double)qo.z);
		cmd2.addDouble((double)qo.w);
		cout << "SEND " << cmd2.toString() << endl;
		simoxRobotViewerPort.write(cmd2,response2);
		cout << "RECEIVE  " << response2.toString() << endl;
		if (response2.get(0).asInt() == 1)
		{
			// convert ok
			po(0) = (float)response2.get(1).asDouble();
			po(1) = (float)response2.get(2).asDouble();
			po(2) = (float)response2.get(3).asDouble();
			qo.x = (float)response2.get(4).asDouble();
			qo.y = (float)response2.get(5).asDouble();
			qo.z = (float)response2.get(6).asDouble();
			qo.w = (float)response2.get(7).asDouble();
		}

		po *= 1000.0f; // mm

		obst_pose = MathTools::quat2eigen4f(qo);
		obst_pose.block(0,3,3,1) = po;
	}

	return true;
}

void SimoxGraspingPipelineControlModule::setCurrentObject( const std::string &objectName )
{
	currentObjectName = objectName;
}

/*
void SimoxGraspingPipelineControlModule::handEyeOffsetChanged( const std::string& objectName, float x, float y, float z )
{
	fixedHandEyeOffset[0] = x;
	fixedHandEyeOffset[1] = y;
	fixedHandEyeOffset[2] = z;
	Bottle cmd;
	cmd.addString("set");
	cmd.addString("object");
	cmd.addString("displacement");
	cmd.addString(objectName.c_str());
	cmd.addDouble(x);
	cmd.addDouble(y);
	cmd.addDouble(z);
	Bottle response;
	bool writeOK = simoxIkSolverPort.write(cmd,response);
	cout << "Sending command <set> <object> <displacement> <" << objectName << ">  <" << x << " " << y << " " << z << "> to simoxIkSolverPort:" << writeOK << endl;
	cout << "Response: " << response.toString().c_str() << endl;
	bool responseOK = response.get(0).asInt()==1;
	if (!responseOK)
		cout << "FAILED...." << endl;
}*/


bool SimoxGraspingPipelineControlModule::getConnectionStatus()
{
	return connected;
}

void SimoxGraspingPipelineControlModule::saveScene( const std::string &filename )
{
	Bottle cmd;
	cmd.addString("saveScene");
	cmd.addString(filename.c_str());
	Bottle response;
	bool writeOK = simoxRobotViewerPort.write(cmd,response);
	cout << "Sending command <saveScene> <" << filename << ">  to simoxRobotViewerPort:" << writeOK << endl;
	cout << "Response: " << response.toString().c_str() << endl;
	bool responseOK = response.get(0).asInt()==1;
	if (!responseOK)
		cout << "FAILED...." << endl;
}

bool SimoxGraspingPipelineControlModule::stopMotionExecution(bool stopPathExecution, bool stopAllParts)
{
	bool ok = true;
	if (stopPathExecution)
	{
		Bottle cmd;
		cmd.addString("stop");

		Bottle response;
		ok = writeToPort(simoxGraspExecutionPort,cmd,response);
		cout << "Sending command <stop> to simoxGraspExecutionPort:" << ok << endl;
		cout << "Response: " << response.toString().c_str() << endl;
	}

	if (stopAllParts)
	{
		cout << "stopping motion directly" << endl;
		bool res = false;
		if (isLeft)
		{
			res = posLeftArm->stop();
		}
		else
		{
			res = posRightArm->stop();
		}
		res = res & posTorso->stop();
		ok = res & ok;
	}
	return ok;
}

bool SimoxGraspingPipelineControlModule::startMotionExecution( std::vector< std::vector<float> > & path )
{

	Bottle cmd;
	cmd.addString("executeMotion");

	int dof = 0;
	for (size_t i=0;i<path.size();i++)
	{
		Bottle &l = cmd.addList();
		if (dof==0)
			dof = (int)path[i].size();
		if (dof!=(int)path[i].size())
		{
			VR_ERROR << "point " << i << ": dof != path entry, aborting..." << endl;
			return false;
		}
		for (int j=0;j<dof;j++)
		{
			l.addDouble((double)path[i][j]);
		}
	}
	
	Bottle response;
	bool writeOK = writeToPort(simoxGraspExecutionPort,cmd,response);
	cout << "Sending command <executeMotion> to simoxGraspExecutionPort:" << writeOK << endl;
	cout << "Response: " << response.toString().c_str() << endl;

	return writeOK;
}

void SimoxGraspingPipelineControlModule::setSegThresh(float thresh)
{
	if (!checkConnections(true))
		return;

	Bottle cmd,response;
	cmd.addString("set");
	cmd.addString("segmentationThreshold");
	cmd.addDouble((double)thresh);
	bool writeOK = simoxLegoLocalizerPort.write(cmd,response);
	cout << "Sending command <" << cmd.toString() << "> to simoxLegoLocalizerPort:" << writeOK << endl;
	cout << "Response: " << response.toString().c_str() << endl;
	return;
}

void SimoxGraspingPipelineControlModule::openHand()
{
	if (!checkConnections(true))
		return;

	Bottle cmd,response;
	cmd.addString("openHand");
	bool writeOK = simoxGraspExecutionPort.write(cmd,response);
	cout << "Sending command <" << cmd.toString() << "> to simoxGraspExecutionPort:" << writeOK << endl;
	cout << "Response: " << response.toString().c_str() << endl;
	return;
}

void SimoxGraspingPipelineControlModule::closeHand()
{
	if (!checkConnections(true))
		return;

	Bottle cmd,response;
	cmd.addString("closeHand");
	bool writeOK = simoxGraspExecutionPort.write(cmd,response);
	cout << "Sending command <" << cmd.toString() << "> to simoxGraspExecutionPort:" << writeOK << endl;
	cout << "Response: " << response.toString().c_str() << endl;
	return;
}

void SimoxGraspingPipelineControlModule::liftHand()
{
	if (!checkConnections(true))
		return;

	Bottle cmd,response;
	cmd.addString("moveEef");
	cmd.addDouble(0); //delta x
	cmd.addDouble(0); //delta y
	cmd.addDouble((double)liftHandDeltaMM/1000.0); //delta z
	bool writeOK = simoxGraspExecutionPort.write(cmd,response);
	cout << "Sending command <" << cmd.toString() << "> to simoxGraspExecutionPort:" << writeOK << endl;
	cout << "Response: " << response.toString().c_str() << endl;
	return;
}

void SimoxGraspingPipelineControlModule::moveHeadStandardPose(float joint0)
{
	//encHead->getEncoders(jointValuesHead.data());
	jointValuesHead.zero();
	jointValuesHead[0] = joint0;
	posHead->positionMove(jointValuesHead.data());
}

