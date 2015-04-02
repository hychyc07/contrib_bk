// 23
// -20
// -23
#include "SimoxGraspExecutionModule.h"
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Trajectory.h>

#include <yarp/math/Math.h>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::math;
using namespace std;
using namespace VirtualRobot;

SimoxGraspExecutionModule::SimoxGraspExecutionModule()
{
	if (!SoDB::isInitialized())
		SoDB::init();
	executionState = eNotMoving;
	controlMode = eCartControl;
	currentTrajectoryPos = -1;
	iCubCartesianControl = NULL;
	encArm = encTorso = NULL;
	velArm = velTorso = NULL;
	timeUpdateEncodersS = 0.1f;
	timeStampOfLastPathPoint = yarp::os::Time::now();
	verbose = true;
	moveHeadLookTable = false;
}


bool SimoxGraspExecutionModule::configure( yarp::os::ResourceFinder &rf )
{
	moduleName            = rf.check("name", 
		Value("SimoxGraspExecution"), 
		"module name (string)").asString();
	setName(moduleName.c_str());

	robotBase = rf.check("robot", 
		Value("icubSim"), 
		"robot name (string)").asString();

	VR_INFO << "Using robot base string " << robotBase << endl;

	// rpc handler port
	handlerPortName =  "/";
	handlerPortName += getName();         // use getName() rather than a literal 
	handlerPortName += "/rpc:i";
	if (!handlerPort.open(handlerPortName.c_str())) {           
		cout << getName() << ": Unable to open port " << handlerPortName << endl;  
		return false;
	}

	attach(handlerPort);                  // attach to port

	std::string execJoints = rf.check("executionJoints", 
		Value("TorsoRightArm"), 
		"set of joints that should be used for moving (string). Could be TorsoLeftArm or TorsoRightArm.").asString().c_str();

	if (execJoints == "TorsoLeftArm")
	{
		VR_INFO << "Setting up executionJoints: eTorsoLeftArm " << endl;
		executionJoints = eTorsoLeftArm;
	} else
	{
		VR_INFO << "Setting up executionJoints: eTorsoRightArm " << endl;
		executionJoints = eTorsoRightArm;
	}


	std::string contrMode = rf.check("controlMode", 
		Value("CartesianControl"), 
		"execution control mode (string). Could be CartesianControl or VelocityControl").asString().c_str();

	if (contrMode == "VelocityControl")
	{
		VR_INFO << "Setting up controlMode: VelocityControl " << endl;
		controlMode = eVelControl;
	} else
	{
		VR_INFO << "Setting up controlMode: CartesianControl " << endl;
		controlMode = eCartControl;
	}

	moveHeadLookTable = rf.check("moveHead", 
		Value(0), 
		"Move head in order to fixate point in front of robot while moving (0 or 1).").asInt() != 0;

	maxDelayExecutionSec = rf.check("execution_MaxDelayBetweenPathPoints", 
		Value(10.0), 
		"Delay in seconds, until a point on the trajectory should be reached (double).").asDouble();

		
	distSwitchToNextPoseMM = (float)rf.check("execution_switchToNextPathPoint_distMM", 
		Value(50.0), 
		"Max distance (in mm) to current path point that has to be reached until the next path point can be considered. (double).").asDouble();

	distSwitchToNextPoseDeg = (float)rf.check("execution_switchToNextPathPoint_distDeg", 
		Value(15.0), 
		"Max orientation distance (in deg) to current path point that has to be reached until the next path point can be considered. (double).").asDouble();

	distGoalReachedMM = (float)rf.check("execution_goalReached_distMM", 
		Value(20.0), 
		"Max distance (in mm) to goal. (double).").asDouble();

	distGoalReachedDeg = (float)rf.check("execution_goalReached_distDeg", 
		Value(10.0), 
		"Max orientation distance (in deg) to goal. (double).").asDouble();

	/*VR_INFO << "DEBUG: setting max delay between pathpoints to 1e10" << endl;
	VR_INFO << "DEBUG: setting execution_switchToNextPathPoint_distMM to 20" << endl;
	maxDelayExecutionSec = 1e10;
	distSwitchToNextPoseMM = 20.0f;*/

	// grasp part
	Bottle &bGrasp=rf.findGroup("grasp");
	
	openHandConfig.resize(9,0.0); 
	closeHandConfig.resize(9,0.0);
	handVel.resize(9,0.0);
	getGraspOptions(bGrasp,openHandConfig,closeHandConfig,handVel);


	if (rf.check("SimoxDataPath"))
	{
		ConstString dataPath=rf.find("SimoxDataPath").asString();
		VR_INFO << "Adding rf.SimoxDataPath: " << dataPath.c_str() << endl;
		VirtualRobot::RuntimeEnvironment::addDataPath(dataPath.c_str());
	}
	ConstString robotFile=rf.find("RobotFile").asString();
	std::string robFileComplete = robotFile.c_str();
	if (!VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFileComplete))
	{
		VR_ERROR << " Could not find file: " << robFileComplete << endl;
	} else
	{
		std::cout << "Using robot file: " << robFileComplete << endl;
		loadRobot(robFileComplete);
	}
	if (rf.check("RobotNodeSet"))
	{
		ConstString rns=rf.find("RobotNodeSet").asString();
		VR_INFO << "Using RobotNodeSet: " << rns.c_str() << endl;
		selectRNS(rns.c_str());
	} else
	{
		VR_ERROR << "Need a RobotNodeSet defined for planning..." << endl;
	}
	if (robot && rf.check("RootCoordSystem"))
	{
		ConstString rootC=rf.find("RootCoordSystem").asString();
		VR_INFO << "Using RootCoordSystem: " << rootC.c_str() << endl;
		if (!robot->hasRobotNode(rootC.c_str()))
		{
			VR_ERROR << "no root coord system found?!" << endl;
		} else
			rootNode = robot->getRobotNode(rootC.c_str());
	} else
	{
		VR_ERROR << "Need a RobotNodeSet defined for planning..." << endl;
	}

	if (robot && rf.check("EndEffector"))
	{
		std::string eef = rf.find("EndEffector").asString().c_str();
		VR_INFO << "Selecting rf.EndEffector: " << eef << endl;
		if (robot && rf.check("EEF_Preshape"))
		{
			std::string eefPreshape = rf.find("EEF_Preshape").asString().c_str();
			VR_INFO << "Selecting rf.EEF_Preshape: " << eefPreshape << endl;

			selectEEF(eef,eefPreshape);
		} else
			selectEEF(eef);
	}

	if (robot && !currentEEF)
	{
		// select first eef

		std::vector<EndEffectorPtr> eefs;
		robot->getEndEffectors(eefs);
		if (eefs.size()>0)
		{
			VR_INFO << "Selecting first EEF: " << eefs[0]->getName() << endl;
			selectEEF(eefs[0]->getName());
		}
	}
	if (!currentEEF)
		VR_INFO << "Skipping EEF definition..." << endl;


	// VISUALIZATION FLAGS
	bool enableVisu = false;
	if (robot)
	{
		if (rf.check("EnableVisualization"))
		{
			std::string cdE = rf.find("EnableVisualization").asString().c_str();
			if (cdE=="true" || cdE=="on" || cdE=="1")
				enableVisu = true;			
		}
	}
	if (enableVisu)
	{
		VR_INFO << "Viewer is ON..." << endl;
		setupViewer();
	} else
		VR_INFO << "Viewer is OFF..." << endl;

	bool cartOK = setupCartesianControl();
	if (!cartOK)
	{
		VR_ERROR << " Could not init iCub's Cartesian Controller!!" << endl;
	}
	lastEncoderUpdateS = Time::now();


	// setup ARM
	std::string localArmName = "/";
	localArmName += getName();
	if (executionJoints == eTorsoLeftArm)
		localArmName += "/left_arm";
	else
		localArmName += "/right_arm";
	Property optionsArm;
	optionsArm.put("device", "remote_controlboard");
	optionsArm.put("local", localArmName.c_str());      //local port names
	std::string remoteArm = ("/" + robotBase);
	if (executionJoints == eTorsoLeftArm)
		remoteArm += "/left_arm";
	else
		remoteArm += "/right_arm";
	optionsArm.put("remote",remoteArm.c_str());        //where we connect to
	if (!robotDeviceArm.open(optionsArm)) {
		printf("Device not available.  Here are the known devices:\n");
		printf("%s", Drivers::factory().toString().c_str());
		close();
		return false;
	}
	robotDeviceArm.view(velArm);
	robotDeviceArm.view(encArm);
	int axesArm;
	if (!encArm || !encArm->getAxes(&axesArm) || axesArm<=0) {
		printf("Could not get encoder values from arm\n");
		close();
		return false;
	}
	jointValuesArm.resize(axesArm,0.0);

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
		close();
		return false;
	}	
	robotDeviceTorso.view(velTorso);
	robotDeviceTorso.view(encTorso);
	int axesTorso;
	if (!encTorso || !encTorso->getAxes(&axesTorso) || axesTorso<=0) {
		printf("Could not get encoder values from Torso\n");
		close();
		return false;
	}
	jointValuesTorso.resize(axesTorso,0.0);

	timeStampOfLastPathPoint = yarp::os::Time::now();
	return true;
}

bool SimoxGraspExecutionModule::setupViewer()
{
	SoQt::init("SimoxGraspExecutionModule");
	viewer.reset(new SimoxRobotViewer("Simox Grasp Execution Results"));
	if (robot)
	{
		viewer->setRobot(robot->clone("SimoxGraspExecution_Visu"));
		viewer->showRobot(true,VirtualRobot::SceneObject::Collision);
		viewer->viewAll();
	}

	return true;
}

bool SimoxGraspExecutionModule::updateJointValues()
{
	if (!currentRNS)
		return false;

	mutex.wait();

	std::vector<float> jv;

	if (!encArm || !encArm->getEncoders(jointValuesArm.data()) || jointValuesArm.size()!=16)
	{
		mutex.post();
		return false;
	}

	if (!encTorso || !encTorso->getEncoders(jointValuesTorso.data()) || jointValuesTorso.size()!=3)
	{
		mutex.post();
		return false;
	}

	jv.push_back(jointValuesTorso[2]*M_PI/180.0);// xchange yaw and pitch
	jv.push_back(jointValuesTorso[1]*M_PI/180.0);
	jv.push_back(jointValuesTorso[0]*M_PI/180.0);
	for (int i=0;i<7;i++)
		jv.push_back(jointValuesArm[i]*M_PI/180.0);

	currentRNS->setJointValues(jv);

	mutex.post();

	return true;
}

void SimoxGraspExecutionModule::showCurrentRobotState()
{
	if (!currentRNS)
		return;

	mutex.wait();

	if (viewer)
		viewer->setJoints(currentRNS->getName(),currentRNS->getJointValues());

	mutex.post();

}

bool SimoxGraspExecutionModule::updateModule()
{
	bool jointsUpdated = false;
	if (executionState == eMoving)
	{
		updateJointValues();
		jointsUpdated = true;
		executeMotion();
	}

	// perform joint updates and motion checks with 10fps
	double delay = Time::now() - lastEncoderUpdateS;
	//if (delay > timeUpdateEncodersS)
	{
		if (!jointsUpdated)
			updateJointValues();
		jointsUpdated = true;
		showCurrentRobotState();
		lastEncoderUpdateS = Time::now();
	}

	if (viewer)
	{
		if (viewer->wasClosed())
		{
			cout << "Viewer was closed, quitting..." << endl;
			return false;
		}

		// send alive signal
		viewer->ping();

		// update gui
		if (qApp)
			qApp->processEvents();
	}
	return true;
}

bool SimoxGraspExecutionModule::respond( const Bottle& command, Bottle& reply )
{
	std::vector<std::string> helpMessages;

	helpMessages.push_back(string(getName().c_str()) + " commands are: \n" );
	helpMessages.push_back("help");
	helpMessages.push_back("quit");
	helpMessages.push_back("info ... list information about internal state");

	helpMessages.push_back("set jointLimits <RNSname> lo_1 hi_1 lo_2 hi_2 ... [rad] set lower and upper joint limit for RobotNodeSet with name <RNSname> (must be defined in robot's XML file)");
	helpMessages.push_back("show coordsystem <jointname> on/off ... enable/disable coordinate system visualization for joint <jointname>");
	
	helpMessages.push_back("get joints ... returns number of joints followed by joint names of current kinematic chain (as defined in robot's XML file) ");
	helpMessages.push_back("closeHand ... close hand ");
	helpMessages.push_back("openHand ... open hand ");
	
	helpMessages.push_back("executeMotion (jv_1_1 ... jv_1_n) ... (jv_i_1 ... jv_i_n)  Execute the joint space trajectory.  ");
	helpMessages.push_back("moveEef delta_x delta_y delta_z  Move the hand by the given delta [in iCub's root coordinate system] [m].  ");
	helpMessages.push_back("stop ... Interrupt any currently executed motions and stop.  ");

	helpMessages.push_back("RETURN: First return value is always 0/1 indicating success or failure.");
	string helpMessage;
	for (size_t i=0;i<helpMessages.size();i++)
		helpMessage = helpMessage + helpMessages[i] + string("\n");

	reply.clear(); 
	bool commandProcessed = false;
	bool responseOK = false;
	bool customResponse = false;
	stringstream responseStream;

	if (command.get(0).asString()=="quit") 
	{
		reply.addString("quitting");
		return false;     
	} else if (command.get(0).asString()=="help") 
	{
		cout << helpMessage;
		responseStream << helpMessage;
		responseOK = true;
		commandProcessed = true;
	} else if (command.get(0).asString()=="info") {
		print();
		responseOK = true;
		responseStream << "printing info";
		commandProcessed = true;
	} else if (command.get(0).asString()=="stop") {
		stopMotion();
		responseOK = true;
		responseStream << "Stopped motion";
		commandProcessed = true;
	} else if (command.get(0).asString()=="closeHand") {
		closeHand();
		responseOK = true;
		responseStream << "Closing hand";
		commandProcessed = true;
	} else if (command.get(0).asString()=="openHand") {
		openHand();
		responseOK = true;
		responseStream << "Opening hand";
		commandProcessed = true;
	} else if (command.get(0).asString()=="moveEef") {
		float x = (float)command.get(1).asDouble();
		float y = (float)command.get(2).asDouble();
		float z = (float)command.get(3).asDouble();
		responseOK = moveEef(x,y,z);
		responseStream << "EEF Moved";
		commandProcessed = true;
	}else if (command.get(0).asString()=="get") 
	{
		if (command.get(1).asString()=="joints") 
		{
			customResponse = true;
			responseOK = true;
			reply.addInt(1); // means OK
			if (!currentRNS)
			{
				cout << "No kinematic chain (RobotNodeSet) defined..." << endl;
				reply.addInt(0); // zero joints
			} else
			{
				cout << "Number of joints: " << currentRNS->getSize() << endl;
				reply.addInt((int)currentRNS->getSize()); // nr joints
				for (unsigned int i=0;i<currentRNS->getSize();i++)
				{
					cout << currentRNS->getNode((int)i)->getName() << ",";
					reply.addString(currentRNS->getNode((int)i)->getName().c_str());
				}
				cout << endl;
			}
			commandProcessed = true;
		} 
	} else if (command.get(0).asString()=="show") 
	{
		if (command.get(1).asString()=="coordsystem") 
		{
			ConstString name = command.get(2).asString();
			ConstString onOff = command.get(3).asString();
		
			responseStream <<"Showing coord system: " << name;
			bool showC = !(onOff == "off");
			reply.addInt((int)(!(onOff == "off")));
			if (viewer)
				responseOK = viewer->showCoordSystem(name.c_str(),showC);
			else
				responseOK = false;
			if (responseOK)
				responseStream << " ok";
			else
				responseStream << " failed";
			commandProcessed = true;
		}
	} else if (command.get(0).asString()=="set") 
	{
		if (command.get(1).asString()=="jointLimits") 
		{
			ConstString name = command.get(2).asString();
			int nrJoints = (command.size()-3) / 2;
			std::vector<float> lo,hi;
			for (int i=0;i<nrJoints;i++)
			{
				lo.push_back((float)command.get(3+i*2).asDouble());
				hi.push_back((float)command.get(4+i*2).asDouble());
			}
			responseOK = setJointLimits(name.c_str(),lo,hi);
			if (responseOK)
			{
				responseStream << "Setting joint limits OK ";
			} else
			{
				responseStream << "Setting joint limits FAILED ";
			}
			responseStream << name;
			commandProcessed = true;
		}
	} else if (command.get(0).asString()=="executeMotion") 
	{
		int pathPoints = command.size()-1;
		if (!currentRNS)
		{
		    cout << "no rns defined.." << endl;
		} else
		{
		    VirtualRobot::TrajectoryPtr t(new VirtualRobot::Trajectory(currentRNS,"motion"));
			int dof = (int)currentRNS->getSize();
			Eigen::VectorXf pathpoint(dof);
			bool ok = true;
			for (int i=0;i<pathPoints;i++)
			{
				yarp::os::Bottle *p = command.get(1+i).asList();
				if (!p || p->size()!=dof)
				{
					VR_ERROR << "Path point " << i << " is NULL or has wrong size... Expecting " << dof << " dof..." << endl;
					ok = false;
					break;
				}

				for (int j=0;j<dof;j++)
				{
					pathpoint[j] = ((float)p->get(j).asDouble());
				}
				t->addPoint(pathpoint);
			}
			if (ok)
			{
				responseOK = startExecuteMotion(t);
			} else
				responseOK = false;
			commandProcessed = true;

		}
	} else
		commandProcessed = false;

	if (!customResponse)
	{
		reply.addInt((int)responseOK);

		if (!commandProcessed)
		{
			responseStream << "Unknown command: " << command.toString().c_str() << "\n Try 'help' \n";
			cout << helpMessage;
		}
	}
	return true;
}

bool SimoxGraspExecutionModule::interruptModule()
{
	stopMotion();
	handlerPort.interrupt();
	printf ("INTERRUPT\n");
	return true;
}

bool SimoxGraspExecutionModule::close()
{	
	handlerPort.close();

	return true;
}

double SimoxGraspExecutionModule::getPeriod()
{
	// 50 fps
	return 0.02;
}


bool SimoxGraspExecutionModule::selectRNS(const std::string &rns)
{
	if (!robot || !robot->hasRobotNodeSet(rns))
	{
		VR_ERROR << "Robot does not have RNS with name " << rns << endl;
		return false;
	}
	currentRNS = robot->getRobotNodeSet(rns);
	
	return true;
}

bool SimoxGraspExecutionModule::loadRobot( const std::string &filename )
{
	VR_INFO << "Loading robot from " << filename << endl;
	try
	{
		// we don't need the visualization model
		robot = RobotIO::loadRobot(filename,VirtualRobot::RobotIO::eCollisionModel);
	}
	catch (VirtualRobotException &e)
	{
		VR_ERROR << " ERROR while creating robot" << endl;
		VR_ERROR << e.what();
		return false;
	}

	if (!robot)
	{
		VR_ERROR << " ERROR while creating robot" << endl;
		return false;
	}

	return true;
}

void SimoxGraspExecutionModule::print()
{
	cout << "***** SimoxGraspExecutionModule *****" << endl;
	cout << "Robot: ";
	if (robot)
		cout << robot->getName();
	else
		cout << "<none>";
	cout << endl;

	cout << "Kinematic Chain (RobotNodeSet): ";
	if (currentRNS)
		cout << currentRNS->getName();
	else
		cout << "<none>";
	cout << endl;

	cout << "TCP: ";
	if (currentRNS && currentRNS->getTCP())
		cout << currentRNS->getTCP()->getName();
	else
		cout << "<none>";
	cout << endl;
}

bool SimoxGraspExecutionModule::setJointLimits( const std::string &robotNodeSet, std::vector<float> &min, std::vector<float> &max )
{
	if (!robot)
	{
		VR_ERROR << "No robot..." << endl;
		return false;
	}
	cout << "Setting joint limits for " << robotNodeSet << " (currentRNS:" << (currentRNS?currentRNS->getName():"<not set>") << ")" << endl;
	RobotNodeSetPtr rns = robot->getRobotNodeSet(robotNodeSet);
	if (!rns)
	{
		VR_ERROR << "No robot node set with name " << robotNodeSet << endl;
		return false;
	}
	if (rns->getSize()!=min.size() || rns->getSize()!=max.size())
	{
		VR_ERROR << "Wrong sizes. RNS (" << robotNodeSet << ") : " << rns->getSize() <<", minSize:" << min.size() << ", maxSize:" << max.size() << endl;
		return false;
	}
	for (size_t i=0;i<min.size();i++)
	{
		RobotNodePtr rn = rns->getNode(i);
		if (!rn)
			return false;
		rn->setJointLimits(min[i],max[i]);
	}
	bool res = true;
	if (viewer)
		res = viewer->setJointLimits(robotNodeSet,min,max);
	return res;
}


bool SimoxGraspExecutionModule::selectEEF(const std::string &eef, const std::string &eefPreshape)
{
	if (!selectEEF(eef) || !currentEEF)
	{
		VR_ERROR << "Error selecting eef, aborting" << endl;
		return false;
	}

	// select preshape in viewer
	if (viewer)
		viewer->selectEEFPreshape(eefPreshape);
	return currentEEF->setPreshape(eefPreshape);
}

bool SimoxGraspExecutionModule::selectEEF(const std::string &eef)
{
	if (!robot || !robot->hasEndEffector(eef))
	{
		VR_ERROR << "Robot does not have EEF with name " << eef << endl;
		return false;
	}
	currentEEF = robot->getEndEffector(eef);
	// select eef in viewer
	if (viewer)
		viewer->selectEEF(eef);
	return true;
}

bool SimoxGraspExecutionModule::getCurrentDistToPose(const Eigen::Matrix4f &globalPose, float &storeDistPosMM, float &storeDistOriDeg, bool lockMutex)
{
	if (!currentRNS || !currentRNS->getTCP() || !robot || !rootNode)
		return false;

	//Eigen::Matrix4f poseRoot = rootNode->toLocalCoordinateSystem(globalPose);
	if (lockMutex)
		mutex.wait();
	Eigen::Matrix4f poseTCP = currentRNS->getTCP()->getGlobalPose();

	MathTools::getDelta(poseTCP,globalPose,storeDistPosMM,storeDistOriDeg);
	storeDistOriDeg = fabs(MathTools::rad2deg(storeDistOriDeg));
	storeDistPosMM = fabs(storeDistPosMM);
	if (lockMutex)
		mutex.post();
	return true;
}

bool SimoxGraspExecutionModule::stopMotion()
{
	VR_INFO << "STOPPING..." << endl;

	mutex.wait();
	executionState = eNotMoving;
	currentTcpTrajectory.clear();
	currentTrajectory.reset();
	currentTrajectoryPos = -1;
	// stop
	if (iCubCartesianControl)
		iCubCartesianControl->stopControl();
	if (velArm)
		velArm->stop();
	if (velTorso)
		velTorso->stop();
	mutex.post();
	return true;
}

bool SimoxGraspExecutionModule::startExecuteMotion( VirtualRobot::TrajectoryPtr t )
{
	if (!t)
	{
		VR_WARNING << "Null data?!" << endl;
		stopMotion();
		return false;
	}

	int points = t->getNrOfPoints();
	if (points==0)
	{
		VR_WARNING << "Zero points?!" << endl;
		stopMotion();
		return false;
	}
	mutex.wait();

	if (executionState==eMoving)
	{
		VR_WARNING << "Switching to new trajectory!!!" << endl;
	}

	if (viewer)
	{
		viewer->removePath("execution");
		viewer->showPath("execution",t);
		if (currentEEF)
			viewer->setEEFVisu("target",currentEEF->getName(),VirtualRobot::SceneObject::Collision);
	}

	currentTrajectory = t;
	currentTrajectoryPos = 0;
	executionState = eMoving;
	currentTcpTrajectory = t->createWorkspaceTrajectory();

	if ((int)currentTcpTrajectory.size() != points)
	{
		VR_ERROR << "Could not generate workspace trajectory..." << endl;
		mutex.post();
		stopMotion();
		return false;
	}

	if (verbose)
	{
		cout << "TCP trajectory (global):\n";
		for (int k=0;k<(int)currentTcpTrajectory.size();k++)
			cout << "pos:" << k << endl << currentTcpTrajectory[k] << endl;
	}

	float distPosMM, distOriDeg;
	getCurrentDistToPose(currentTcpTrajectory[0], distPosMM, distOriDeg,false);
	if (verbose)
		VR_INFO << "Current TCP: dist to start of new trajectory: distMM:" << distPosMM << ", distOri:" << distOriDeg << endl;
	timeStampOfLastPathPoint = yarp::os::Time::now();

	// set tracking mode
	//iCubCartesianControl->setTrackingMode(true);
	mutex.post();
	return true;
}


bool SimoxGraspExecutionModule::executeMotion( )
{
	// some checks....
	mutex.wait();
	if (!currentTrajectory || 
		currentTrajectory->getNrOfPoints() != (int)currentTcpTrajectory.size() ||
		executionState!=eMoving || 
		currentTrajectoryPos <0 ||
		currentTrajectoryPos >= (int)currentTrajectory->getNrOfPoints())
	{
		VR_WARNING << "Not in correct state?!" << endl;
		mutex.post();
		stopMotion();
		return false;
	}

	// check for timeout
	double delay = yarp::os::Time::now() - timeStampOfLastPathPoint;
	if (delay>maxDelayExecutionSec)
	{
		VR_ERROR << "Could not achieve the requested pose within " << delay << "seconds. Aborting (todo:maybe we want to handle this case as a warning instead of an error? Then we should switch to the next path point here..." << endl;
		mutex.post();
		stopMotion();
		return false;
	}
	mutex.post();

	if (moveHeadLookTable)
		lookToTable();

	switch (controlMode)
	{
	case eCartControl:
		return executeMotionCartControl();
		break;
	case eVelControl:
		return executeMotionVelControl();
		break;
	default:
		VR_ERROR << "nyi..." << endl;
	}
	return false;
}

bool SimoxGraspExecutionModule::executeMotionVelControl( )
{
	mutex.wait();
	Eigen::Matrix4f currentTcpPos = currentTcpTrajectory[currentTrajectoryPos];
	float distPosMM, distOriDeg;
	if (!getCurrentDistToPose(currentTcpPos, distPosMM, distOriDeg,false))
	{
		VR_ERROR << "internal error..." << endl;
		mutex.post();
		stopMotion();
		return false;
	}
	//if (verbose)
		cout << " Point " << currentTrajectoryPos << " (max:" << currentTrajectory->getNrOfPoints() << "): distMM:" << distPosMM << ", distOri:" << distOriDeg << endl;

	bool switchToNextPoint = false;
	
	float testMM = distSwitchToNextPoseMM;
	float testDeg = distSwitchToNextPoseDeg;
	
	if (currentTrajectoryPos == (int)currentTrajectory->getNrOfPoints()-1)
	{
		cout << "LAST point, trying to reach with high accuracy" << endl;
		testMM = distGoalReachedMM;
		testDeg = distGoalReachedDeg;
	}
	if (distPosMM < testMM && distOriDeg < testDeg)
	{
		if (verbose)
			VR_INFO << "Near to current trajectory point. Switching to next one." << endl;
		switchToNextPoint = true;
	}

	if (switchToNextPoint)
	{
		if (verbose)
			VR_INFO << "Switching to next position" << endl;
		currentTrajectoryPos++;
		timeStampOfLastPathPoint = yarp::os::Time::now();
		if (currentTrajectoryPos >= (int)currentTrajectory->getNrOfPoints())
		{
			cout << "GOAL REACHED" << endl;

			// send a last command to cart interface with final pose (better accuracy)
			//VR_INFO << "Check if that is ok: send a last command to cart interface with final pose..." << endl;
			//goToCartesianPose(currentTcpTrajectory[currentTcpTrajectory.size()-1],false);

			//stopMotion();
			// -> remain in tracking mode

			// send zero
			Eigen::VectorXf z(10);
			z.setZero();
			safeVelocityMoveTorsoArm(z);

			executionState = eNotMoving;
			currentTcpTrajectory.clear();
			currentTrajectory.reset();
			currentTrajectoryPos = -1;
			mutex.post();

			return true;
		}

		// either we can set the new target once here, or we continuously set the target later
	} 

	// we continuously update the velocities
	Eigen::VectorXf goal = currentTrajectory->getPoint(currentTrajectoryPos);
	Eigen::VectorXf current;
	currentRNS->getJointValues(current);

	float alpha = 1.5f;
	Eigen::VectorXf velvect = alpha * (goal - current);
	float maxVel = 0.3f;
	if (velvect.norm()>maxVel)
	{
		VR_INFO << "Velocity too high: " << velvect.norm() << ". Normalizing to maxVel = " << maxVel << endl;
		//if (verbose)
		//	cout << "vel before:\n" << velvect << endl;
		velvect.normalize();
		velvect *= maxVel;
		//if (verbose)
		//	cout << "vel after:\n" << velvect << endl;
	}

	//VR_INFO << "VEL [rad]:\n" << velvect << endl;
	for (int i=0;i<(int)velvect.size();i++)
	{
		velvect[i] = velvect[i] / (float)M_PI * 180.0f;
	}
	//VR_INFO << "VEL [de]:\n" << velvect << endl;
	safeVelocityMoveTorsoArm(velvect);
	
	mutex.post();
	return true;
}

/**
  * Set the velocity of the specified joint taking into account the min and max value specified for the encoder
  * and the max and min values specified for the velocities.
  */
bool SimoxGraspExecutionModule::safeVelocityMoveTorsoArm(Eigen::VectorXf &vel)
{
	int joints = 10;
	if (vel.size()!=(size_t)joints)
		return false;
	const float MAX_JOINT_DIST = 2.0f;// max joint limit distance [deg]
	const float VELOCITY_MAX = 30.0f;// max vel [deg]
	const float VELOCITY_MIN = -30.0f;// min vel [deg]

	for (int i=0;i<joints;i++)
	{
		float speed = vel[i];
		if (i<3)
		{
			if(fabs(jointValuesTorso[i]-jointLimitsTorsoArmMin[i]) < MAX_JOINT_DIST)
			{
				if (speed<0)
				{
					cout << "Torso Joint " << i << " near lower limit. setting vel to 0" << endl;
					speed = 0;
				}
			}
			else if( fabs(jointValuesTorso[i] - jointLimitsTorsoArmMax[i]) < MAX_JOINT_DIST)
			{
				if (speed>0)
				{
					cout << "Torso Joint " << i << " near upper limit. setting vel to 0" << endl;
					speed = 0;
				}
			}
		} else
		{
			if(fabs(jointValuesArm[i-3]-jointLimitsTorsoArmMin[i]) < MAX_JOINT_DIST)
			{
				if (speed<0)
				{
					cout << "Arm Joint " << i << " near lower limit. setting vel to 0" << endl;
					speed = 0;
				}
			}
			else if( fabs(jointValuesArm[i-3] - jointLimitsTorsoArmMax[i]) < MAX_JOINT_DIST)
			{
				if (speed>0)
				{
					cout << "Arm Joint " << i << " near upper limit. setting vel to 0" << endl;
					speed = 0;
				}
			}
		}
		speed = (speed>VELOCITY_MAX) ? VELOCITY_MAX : speed;
		speed = (speed<VELOCITY_MIN) ? VELOCITY_MIN : speed;
		if (i<3)
		{
			velTorso->velocityMove(2-i, speed); //xchange yaw and pitch
		} else
			velArm->velocityMove(i-3, speed);
	}
	return true;
}



bool SimoxGraspExecutionModule::executeMotionCartControl( )
{
	mutex.wait();
	Eigen::Matrix4f currentTcpPos = currentTcpTrajectory[currentTrajectoryPos];
	float distPosMM, distOriDeg;
	if (!getCurrentDistToPose(currentTcpPos, distPosMM, distOriDeg,false))
	{
		VR_ERROR << "internal error..." << endl;
		mutex.post();
		stopMotion();
		return false;
	}
	if (verbose)
		cout << " Point " << currentTcpPos << " : distMM:" << distPosMM << ", distOri:" << distOriDeg << endl;

	bool switchToNextPoint = false;
	// check if cart controller reports "motion done"
	bool md;
	if (iCubCartesianControl && iCubCartesianControl->checkMotionDone(&md) && md)
	{
		if (verbose)
			VR_INFO << " Cartesian Controller reports <motion done>. Switching to next point on trajectory." << endl;
		switchToNextPoint = true;
	}

	if (distPosMM < distSwitchToNextPoseMM && distOriDeg < distSwitchToNextPoseDeg)
	{
		if (verbose)
			VR_INFO << "Near to current trajectory point. Switching to next one." << endl;
		switchToNextPoint = true;
	}

	if (switchToNextPoint)
	{
		if (verbose)
			VR_INFO << "Switching to next position" << endl;
		currentTrajectoryPos++;
		timeStampOfLastPathPoint = yarp::os::Time::now();
		if (currentTrajectoryPos >= (int)currentTrajectory->getNrOfPoints())
		{
 			VR_INFO << "GOAL REACHED" << endl;

			// send a last command to cart interface with final pose (better accuracy)
			goToCartesianPose(currentTcpTrajectory[currentTcpTrajectory.size()-1],false);
			
			// stopMotion();
			// -> remain in tracking mode
			
			executionState = eNotMoving;
			currentTcpTrajectory.clear();
			currentTrajectory.reset();
			currentTrajectoryPos = -1;
			mutex.post();

			return true;
		}

		// either we can set the new target once here, or we continuously set the target later
		
		// set it here
		currentTcpPos = currentTcpTrajectory[currentTrajectoryPos];
		mutex.post();
		return goToCartesianPose(currentTcpPos);
		
	} 

	// if here, we are waiting until the execution is done
	mutex.post();
	return true;
}

bool SimoxGraspExecutionModule::setupCartesianControl()
{
	Property option("(device cartesiancontrollerclient)");
	std::string remoteStr = "/" + robotBase;
	std::string localStr = "/";
	localStr += getName();
	localStr += "/cartesian_client";
	if (executionJoints == eTorsoLeftArm)
	{
		remoteStr += "/cartesianController/left_arm";
		localStr += "/left_arm";				
	} else
	{
		remoteStr += "/cartesianController/right_arm";
		localStr += "/right_arm";				
	}
	option.put("remote",remoteStr.c_str());
	option.put("local",localStr.c_str());
	if (!iCubCartesianControlClient.open(option))
		return false;

	// open the view
	iCubCartesianControlClient.view(iCubCartesianControl);
	if (!iCubCartesianControl)
		return false;

	iCubCartesianControl->setTrajTime(1.0);
	iCubCartesianControl->setInTargetTol(1e-5);

	// get the torso dofs
	Vector newDof, curDof;
	iCubCartesianControl->getDOF(curDof);
	newDof=curDof;

	// enable the torso yaw and pitch
	newDof[0]=1;
	newDof[1]=1;//0;// todo: why? disable the torso roll
	newDof[2]=1;
	// send the request for dofs reconfiguration
	iCubCartesianControl->setDOF(newDof,curDof);

	// setup joint limits
	jointLimitsTorsoArmMin.clear();
	jointLimitsTorsoArmMax.clear();
	double minJV, maxJV;
	//cout << "Setting joint limits of iCubCartesianSolver" << endl;
	cout << "Setting joint limits of simox iCub model w.r.t. iCubCartesianSolver limits" << endl;
	std::streamsize pr = std::cout.precision(2);
	if (currentRNS)
	{
		for (unsigned int i=0;i<currentRNS->getSize();i++)
		{
			iCubCartesianControl->getLimits(i,&minJV,&maxJV);
			jointLimitsTorsoArmMin.push_back(minJV);
			jointLimitsTorsoArmMax.push_back(maxJV);
			RobotNodePtr rn = currentRNS->getNode(i);
			if (!rn)
				continue;
					
			double lo = (double)rn->getJointLimitLo() * 180.0 / M_PI;
			double hi = (double)rn->getJointLimitHi() * 180.0 / M_PI;
			if (abs(lo-minJV)>0.1 || abs(hi-maxJV)>0.1)
			{
						
				cout << "Changing joint limits of joint " << rn->getName() << " from (" << lo << "," << hi << ") to (" << minJV << "," << maxJV << ") [degree]" << endl;
				rn->setJointLimits((float)minJV/180.0f*(float)M_PI, (float)maxJV/180.0f*(float)M_PI);
				if (viewer)
					viewer->setJointLimit(rn->getName(),(float)minJV/180.0f*(float)M_PI, (float)maxJV/180.0f*(float)M_PI);
			}	
		}
	}
	std::cout.precision(pr);

	// disable tracking mode
	iCubCartesianControl->setTrackingMode(false);


	// connect to gaze control
	Property optionGaze;
	optionGaze.put("device","gazecontrollerclient");
	optionGaze.put("remote","/iKinGazeCtrl");
	optionGaze.put("local","/client_graspExecution/gaze");
	if (!iCubGazeClient.open(optionGaze))
	{
		VR_ERROR << " Could not open iKinGazeCtrl for GazeClient" << endl;
		return false;
	}
	// open the view
	iCubGazeClient.view(iCubGazeControl);
	if (!iCubGazeControl)
	{
		VR_ERROR << "Could not get iCub gaze controller..." << endl;
		return false;
	}

	return true;
}

bool SimoxGraspExecutionModule::goToCartesianPose(const Eigen::Matrix4f &globalPose, bool lockMutex)
{
	if (!iCubCartesianControl || !rootNode)
		return false;

	if (!iCubCartesianControl)
	{
		VR_INFO << "Trying to initialize Cartesian controller..." << endl;
		if (!setupCartesianControl())
		{
			VR_ERROR << "Could not init cart control" << endl;
			return false;
		}
	}

	bool verbose = true;

	if (lockMutex)
		mutex.wait();
	
	// query iCub ik solver
	Vector xd,od;//,resX,resO,resQ;
	xd.resize(3);
	od.resize(4);
	Eigen::Vector3f axis;
	Eigen::Vector3f pos;
	float angle;

	if (verbose)
		cout << "goToPose_global:\n" << globalPose << endl;

	Eigen::Matrix4f mTcp = rootNode->toLocalCoordinateSystem(globalPose);
	MathTools::eigen4f2axisangle(mTcp,axis,angle);
	axis.normalize();
	pos = MathTools::getTranslation(mTcp);
	xd[0] = pos(0) / 1000.0; // mm -> m 
	xd[1] = pos(1) / 1000.0; // mm -> m 
	xd[2] = pos(2) / 1000.0; // mm -> m 
	od[0] = axis(0);
	od[1] = axis(1);
	od[2] = axis(2);
	od[3] = angle;

	if (verbose)
	{
		cout << "goToPose\nPose: "<< xd(0) << "," << xd(1) << "," << xd(2) << endl;
		cout << "Ori: "<< od(0) << "," << od(1) << "," << od(2) << "," << od(3) << endl;

		Vector x0,o0;
		if (iCubCartesianControl->getPose(x0,o0))
		{
			cout << "CurrentPos:" << x0(0) << "," << x0(1) << "," << x0(2) << endl;
			cout << "CurrentOri:" << o0(0) << "," << o0(1) << "," << o0(2) << "," << o0(3) << endl;
		} else
			cout <<" Could not get current pose!!!" << endl;
	}
	bool res = iCubCartesianControl->goToPoseSync(xd,od); 
	if (!res)
	{
		VR_WARNING << "iCubCartesianControl->goToPoseSync reported FALSE" << endl;
	}

	if (viewer)
	{
		viewer->setEEFVisuPose("target",globalPose);
	}
	if (lockMutex)
		mutex.post();
	return true;
}

void SimoxGraspExecutionModule::getGraspOptions(Bottle &b, Vector &openPoss, Vector &closePoss, Vector &vels)
{
	cout << "checking for open_hand" << endl;
	if (b.check("open_hand","Getting openHand poss"))
	{
		cout << "found" << endl;
		Bottle &grp=b.findGroup("open_hand");
		int sz=grp.size()-1;
		int len=sz>9?9:sz;
		cout << "data: ";
		for (int i=0; i<len; i++)
		{
			openPoss[i]=grp.get(1+i).asDouble();
			cout << openPoss[i] << ",";
		}
		cout << endl;		
	} else
		cout << "failed" << endl;

	cout << "checking for close_hand" << endl;
	if (b.check("close_hand","Getting closeHand poss"))
	{
		cout << "found" << endl;
		Bottle &grp=b.findGroup("close_hand");
		int sz=grp.size()-1;
		int len=sz>9?9:sz;

		for (int i=0; i<len; i++)
			closePoss[i]=grp.get(1+i).asDouble();
	} else
		cout << "failed" << endl;


	cout << "checking for vels_hand" << endl;
	if (b.check("vels_hand","Getting hand vels"))
	{
		cout << "found" << endl;
		Bottle &grp=b.findGroup("vels_hand");
		int sz=grp.size()-1;
		int len=sz>9?9:sz;

		for (int i=0; i<len; i++)
			vels[i]=grp.get(1+i).asDouble();
	} else
		cout << "failed" << endl;

}



bool SimoxGraspExecutionModule::moveHand(yarp::sig::Vector *p)
{
	cout << "move Hand..."<< endl;
	Vector *poss=NULL;       
	string actionStr, type;
	IPositionControl *ipos;

	robotDeviceArm.view(ipos);
	int armJoints = 7;
	int handJoints = 9;

	for (int j=0; j<handJoints; j++)
	{
		int k=armJoints+j;
		cout << "joint " << k << endl;
		cout << "Setting hand ref speed to " <<  handVel[j] << endl;
		cout << "Setting hand positionMove  to " <<  (*p)[j] << endl;

		ipos->setRefSpeed(k,handVel[j]);
		ipos->positionMove(k,(*p)[j]);
	}
	return true;
}

void SimoxGraspExecutionModule::lookToTable()
{
	if (!iCubGazeControl)
		return;
	Vector fp(3);
	fp[0]=-1.00;    // x-component [m]
	fp[1]=+0.00;    // y-component [m]
	fp[2]=-0.2;    // z-component [m]

	iCubGazeControl->lookAtFixationPoint(fp); // move the gaze to the desired fixation point
}

bool SimoxGraspExecutionModule::closeHand()
{
	cout << "close Hand..."<< endl;
	stopMotion();
	return moveHand(&closeHandConfig);
}

bool SimoxGraspExecutionModule::openHand()
{
	stopMotion();
	return moveHand(&openHandConfig);
}

bool SimoxGraspExecutionModule::moveEef( float delta_x, float delta_y, float delta_z )
{
	if (fabs(delta_x)>1.0f || fabs(delta_y)>1.0f || fabs(delta_z)>1.0f)
	{
		VR_ERROR << "Large displacement, aborting?!" << endl;
		return false;
	}

	Vector x0,o0;
	if (!iCubCartesianControl->getPose(x0,o0))
	{
		VR_ERROR <<" Could not get current pose!!!" << endl;
		return false;
	}
	cout << "CurrentPos:" << x0(0) << "," << x0(1) << "," << x0(2) << endl;
	cout << "CurrentOri:" << o0(0) << "," << o0(1) << "," << o0(2) << "," << o0(3) << endl;
	x0(0) += (double)delta_x;
	x0(1) += (double)delta_y;
	x0(2) += (double)delta_z;
	bool res = iCubCartesianControl->goToPoseSync(x0,o0); 
	if (!res)
	{
		VR_WARNING << "iCubCartesianControl->goToPoseSync reported FALSE" << endl;
		return false;
	}
	// wait some time
	iCubCartesianControl->waitMotionDone(0.1,4.0);
	return true;
}


