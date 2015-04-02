
#include "SimoxLegoLocalizerModule.h"
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/RobotNodeSet.h>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace std;
using namespace VirtualRobot;


SimoxLegoLocalizerModule::SimoxLegoLocalizerModule()
{
	timeUpdateEncodersS = 0.1;
	lastEncoderUpdateS = Time::now();
	encHead = encTorso = NULL;
	legoModel = eNotSet;
	canSendResultsToRobotViewer = false;
	sendResultsToRobotViewer = false;
}



bool SimoxLegoLocalizerModule::configure( yarp::os::ResourceFinder &rf )
{
	moduleName            = rf.check("name", 
		Value("SimoxLegoLocalizer"), 
		"module name (string)").asString();
	setName(moduleName.c_str());

	robotBase = rf.check("robot", 
		Value("icubSim"), 
		"robot name (string)").asString();

	VR_INFO << "Using robot base string " << robotBase << endl;

	// rpc handler port
	handlerPortName =  "/";
	handlerPortName += getName();         // use getName() rather than a literal 
	handlerPortName +=  "/rpc:i";
	if (!handlerPort.open(handlerPortName.c_str())) {           
		cout << getName() << ": Unable to open port " << handlerPortName << endl;  
		return false;
	}

	attach(handlerPort);                  // attach to port

	// setup HEAD
	std::string localHeadName = "/";
	localHeadName += getName();
	localHeadName += "/head";
	Property optionsHead;
	optionsHead.put("device", "remote_controlboard");
	optionsHead.put("local", localHeadName.c_str());      //local port names
	std::string remoteHead = ("/" + robotBase + "/head");
	optionsHead.put("remote", remoteHead.c_str());         //where we connect to
	if (!robotDeviceHead.open(optionsHead)) {
		printf("Device not available.  Here are the known devices:\n");
		printf("%s", Drivers::factory().toString().c_str());
		close();
		return false;
	}	
	robotDeviceHead.view(encHead);
	int axesHead;
	if (!encHead || !encHead->getAxes(&axesHead) || axesHead<=0) {
		printf("Could not get encoder values from head\n");
		close();
		return false;
	}
	jointValuesHead.resize(axesHead,0.0);


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
	robotDeviceTorso.view(encTorso);
	int axesTorso;
	if (!encTorso || !encTorso->getAxes(&axesTorso) || axesTorso<=0) {
		printf("Could not get encoder values from Torso\n");
		close();
		return false;
	}
	jointValuesTorso.resize(axesTorso,0.0);

	// connect to gaze control
	Property optionGaze;
	optionGaze.put("device","gazecontrollerclient");
	optionGaze.put("remote","/iKinGazeCtrl");
	optionGaze.put("local","/client_legoLocalizer/gaze");
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

	std::string legoModelString =  rf.check("LegoModelType", 
		Value("XWing"), 
		"Lego Model that should be localized (string)").asString().c_str();
	
	if (legoModelString=="Gate")
		legoModel = eGate;
	else 
		legoModel = eXWing;
	VR_INFO << "Setting lego model type: " << getModelString(legoModel) << endl;
	
	rnsNameTorso =  rf.check("RobotNodeSet_Torso", 
		Value("Hip"), 
		"VirtualRobot RNS for setting torso joints (string)").asString();

	rnsNameHead =  rf.check("RobotNodeSet_Head", 
		Value("Head"), 
		"VirtualRobot RNS for setting head joints (string)").asString();

	robotFilename =  rf.check("RobotFile", 
		Value("robots/iCub/iCub.xml"), 
		"VirtualRobot XML robot file, that is used to transform coordinates (string)").asString();
	VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robotFilename);

	rootCoordName =  rf.check("RootCoordSystem", 
		Value("iCubRoot"), 
		"VirtualRobot robot node that specifies the root coordinate system (string)").asString();

	camLeftCoordName =  rf.check("CamLeftCoordSystem", 
		Value("EyeLeftCam"), 
		"VirtualRobot robot node that specifies the coordinate system of the left camera (string)").asString();

	camRightCoordName =  rf.check("CamRightCoordSystem", 
		Value("EyeRightCam"), 
		"VirtualRobot robot node that specifies the coordinate system of the right camera (string)").asString();

	int sr = (rf.check("showResults", 
		Value("0"), 
		"Show results in simox robot viewer (silently ignored if no robotViewer is started) (0/1)").asInt());
	sendResultsToRobotViewer =  sr != 0;


	cout << "* robot file:    " << robotFilename << endl;
	cout << "* RNS Head:      " << rnsNameHead << endl;
	cout << "* RNS Torso:     " << rnsNameTorso << endl;
	cout << "* Root Coord:    " << rootCoordName << endl;
	cout << "* CamLeft Coord: " << camLeftCoordName << endl;
	cout << "* CamRight Coord:" << camRightCoordName << endl;
	cout << "* sendResultsToRobotViewer:" << sendResultsToRobotViewer << endl;

	// port names
	serverNameDisparity = "/";
	serverNameDisparity += rf.check("StereoDisparityModule",Value("stereoDisparity"),"Stereo Disparity module").asString().c_str();
	serverNameDisparity += "/rpc";
	clientNameDisparity =  "/";
	clientNameDisparity += getName();
	clientNameDisparity += "/stereoDisparity/rpc:o";

	serverNameBlobExtractorLeft = "/";
	serverNameBlobExtractorLeft += rf.check("BlobExtractorLeft",Value("blobExtractorLeft"),"Blob Extractor module (left cam image)").asString().c_str();
	serverNameBlobExtractorLeft += "/rpc";
	clientNameBlobExtractorLeft =  "/";
	clientNameBlobExtractorLeft += getName();
	clientNameBlobExtractorLeft += "/blobExtractorLeft/rpc:o";

	serverNameBlobExtractorRight = "/";
	serverNameBlobExtractorRight += rf.check("BlobExtractorRight",Value("blobExtractorRight"),"Blob Extractor module (right cam image)").asString().c_str();
	serverNameBlobExtractorRight += "/rpc";
	clientNameBlobExtractorRight =  "/";
	clientNameBlobExtractorRight += getName();
	clientNameBlobExtractorRight += "/blobExtractorRight/rpc:o";

	clientNameRobotViewer =  "/";
	clientNameRobotViewer += getName();
	clientNameRobotViewer += "/SimoxRobotViewer/rpc:o";
	serverNameRobotViewer = "/SimoxRobotViewer/rpc:i";


	setupPorts();
	
	return setupRobot(robotFilename, rnsNameHead, rnsNameTorso, rootCoordName, camLeftCoordName, camRightCoordName);
}

bool SimoxLegoLocalizerModule::setupRobot(const std::string &filename, const std::string &rnsHeadName, const std::string &rnsTorsoName, const std::string &rootName, const std::string &camLeftCoordName, const std::string &camRightCoordName)
{
	robot.reset();
	rnsHead.reset();
	rnsTorso.reset();
	rootCoordNode.reset();
	RobotPtr r;
	try
	{
		r = RobotIO::loadRobot(filename,VirtualRobot::RobotIO::eStructure);
	}
	catch (VirtualRobotException &e)
	{
		VR_ERROR << " ERROR while creating robot" << endl;
		VR_ERROR << e.what();
		return false;
	}

	if (!r)
	{
		VR_ERROR << " ERROR while creating robot" << endl;
		return false;
	}

	robot = r;
	if (!robot->hasRobotNodeSet(rnsTorsoName))
	{
		cout << "No rns with name " << rnsTorsoName << endl;
	} else
		rnsTorso = robot->getRobotNodeSet(rnsTorsoName);
	if (!robot->hasRobotNodeSet(rnsHeadName))
	{
		cout << "No rns with name " << rnsHeadName << endl;
	} else
		rnsHead = robot->getRobotNodeSet(rnsHeadName);

	if (!robot->hasRobotNode(rootName))
	{
		cout << "Failed to get root coordinate node: No RobotNode with name " << rootName << endl;
	} else
	{
		cout << " Setting root coordinate system to " << rootName << endl;
		rootCoordNode = robot->getRobotNode(rootName);
	}
	if (!robot->hasRobotNode(camLeftCoordName) || !robot->hasRobotNode(camRightCoordName) )
	{
		cout << "Failed to get cam coordinate nodes: No RobotNode with name " << camLeftCoordName << " or " << camRightCoordName << endl;
	} else
	{
		cout << " Setting cam left coordinate system to " << camLeftCoordName << endl;
		cout << " Setting cam right coordinate system to " << camRightCoordName << endl;
		camLeftCoordNode = robot->getRobotNode(camLeftCoordName);
		camRightCoordNode = robot->getRobotNode(camRightCoordName);
	}
	return (robot && rnsHead && rnsTorso && rootCoordNode && camLeftCoordNode && camRightCoordNode);
}

bool SimoxLegoLocalizerModule::updateModule()
{
	// we update on request
	
	double delay = Time::now() - lastEncoderUpdateS;
	if (delay> timeUpdateEncodersS)
	{
		lastEncoderUpdateS = Time::now();
		updateRobotModel(false);
	}
	
	return true;
}

bool SimoxLegoLocalizerModule::updateRobotModel(bool updateModel)
{
	bool res1 = false;
	bool res2 = false;
	if (rnsHead && encHead)
	{
		encHead->getEncoders(jointValuesHead.data()); // ignore timeouts...
		if (jointValuesHead.size()==6)
		{
			res1 = true;
			if (updateModel)
			{
				std::vector<float> jv;
				jv.push_back((float)(-jointValuesHead[2]*M_PI/180.0));
				jv.push_back((float)(jointValuesHead[0]*M_PI/180.0));
				jv.push_back((float)(jointValuesHead[1]*M_PI/180.0));
				jv.push_back((float)(jointValuesHead[3]*M_PI/180.0));
				jv.push_back((float)(jointValuesHead[4]*M_PI/180.0)); // ? test this
				jv.push_back((float)(jointValuesHead[3]*M_PI/180.0));
				jv.push_back((float)(jointValuesHead[4]*M_PI/180.0)); // ? test this

				rnsHead->setJointValues(jv);
			}

		}
	}
	if (rnsTorso && encTorso)
	{
		encTorso->getEncoders(jointValuesTorso.data()); // ignore timeouts
		if (jointValuesTorso.size()==3)
		{
			res2 = true;
			if (updateModel)
			{
				std::vector<float> jv;
				jv.push_back((float)(jointValuesTorso[2]*M_PI/180.0));// xchange yaw and pitch
				jv.push_back((float)(jointValuesTorso[1]*M_PI/180.0));
				jv.push_back((float)(jointValuesTorso[0]*M_PI/180.0));

				rnsTorso->setJointValues(jv);
			}
		}
	}

	return (res1 && res2);
}

bool SimoxLegoLocalizerModule::respond( const Bottle& command, Bottle& reply )
{
	cout << "RESPONSE START" << endl;
	std::vector<std::string> helpMessages;

	helpMessages.push_back(string(getName().c_str()) + " commands are: \n" );
	helpMessages.push_back("help");
	helpMessages.push_back("quit");
	//helpMessages.push_back("set object <name> <filename> ... load object from file <filename> and add to viewer. Chose <name> as you want, it will be key for further access.");
	//helpMessages.push_back("remove object <name> ... remove object <name> from viewer");
	//helpMessages.push_back("set object position <name> x y z ... set object <name> to position (x,y,z)");
	//helpMessages.push_back("set object orientation <name> roll pitch yaw ... set RPY orientation of object <name>");
	helpMessages.push_back("set legomodel <type> ... Set lego model type (string could be Gate or XWing)");
	helpMessages.push_back("set jointLimits <RNSname> lo_1 hi_1 lo_2 hi_2 ... [rad] set lower and upper joint limit for RobotNodeSet with name <RNSname> (must be defined in robot's XML file)");
	helpMessages.push_back("set segmentationThreshold t ... Set blobExtractor's segmentation thresholds to t");
	helpMessages.push_back("localize <withObstacle>... Try to localize current object. Returns 1/0 to indicate success or failure. On success the pose follows as 3 positions (x y z) [meters] and 4 quaternions (q1 q2 q3 w). If the optional parameter withObstacle is set, an obstacle is assumed in the scene (the corresponding blob must be on the right side) and the obstacles coordinates are additionally returned. Coordinates are given in RootCoord system (as defined in setup).   ");
	helpMessages.push_back("updateLocalize <withObstacle>... Updates localization of current object. The localizations of previos runs are used to compute a mean value. Returns 1/0 to indicate success or failure. On success the pose follows as 3 positions (x y z) [meters] and 4 quaternions (q1 q2 q3 w). If the optional parameter withObstacle is set, an obstacle is assumed in the scene (the corresponding blob must be on the right side) and the obstacles coordinates are additionally returned. Coordinates are given in RootCoord system (as defined in setup).  ");

	//helpMessages.push_back("viewer show on/off ... show/hide viewer");

	string helpMessage;
	for (size_t i=0;i<helpMessages.size();i++)
		helpMessage = helpMessage + helpMessages[i] + string("\n");

	reply.clear(); 
	bool eaten = false;
	bool responseOK = false;
	bool customResponse = false;
	std::stringstream responseStream;

	if (command.get(0).asString()=="quit") {
		reply.addString("quitting");
		responseOK = true;
		return false;     
	}
	else if (command.get(0).asString()=="help") {
		cout << helpMessage;
		responseStream << helpMessage;
		responseOK = true;
		eaten = true;
	}
	else if (command.get(0).asString()=="localize" || command.get(0).asString()=="updateLocalize")
	{
		Eigen::Matrix4f m_root;
		Eigen::Matrix4f m_obst_root;
		customResponse = true;
		eaten = true;
		bool update = command.get(0).asString()=="updateLocalize";
		bool obstacle = false;
		if (command.size()>1 && command.get(1).asString()=="withObstacle")
			obstacle = true;
		if (localize(m_root, update,obstacle,m_obst_root))
		{
			reply.addInt(1); // 1 : success
			responseOK = true;
			cout << "Found lego..." << endl;
			MathTools::Quaternion q = MathTools::eigen4f2quat(m_root);
			reply.addDouble((double)m_root(0,3)*0.001);
			reply.addDouble((double)m_root(1,3)*0.001);
			reply.addDouble((double)m_root(2,3)*0.001);
			reply.addDouble((double)q.x);
			reply.addDouble((double)q.y);
			reply.addDouble((double)q.z);
			reply.addDouble((double)q.w);
			if (obstacle)
			{
				q = MathTools::eigen4f2quat(m_obst_root);
				reply.addDouble((double)m_obst_root(0,3)*0.001);
				reply.addDouble((double)m_obst_root(1,3)*0.001);
				reply.addDouble((double)m_obst_root(2,3)*0.001);
				reply.addDouble((double)q.x);
				reply.addDouble((double)q.y);
				reply.addDouble((double)q.z);
				reply.addDouble((double)q.w);
			}
		} else
		{
			reply.addInt(0); // 0 : failure
			responseOK = false;
			cout << "Did not find lego..." << endl;
		}
	}
	/*else if (command.get(0).asString()=="viewer") 
	{
		if (command.get(1).asString()=="show") 
		{
			bool on = true;
			if (command.get(2).asString()=="off")
			{
				responseStream << "Hiding viewer window";
				on = false;
			} else
			{
				responseStream << "Showing viewer window";
				on = true;
			}
			if (viewer)
				viewer->showViewer(on);
			responseOK = true;
			eaten = true;			
		}
	}*/
	/*else if (command.get(0).asString()=="add") 
	{
		if (command.get(1).asString()=="object") 
		{
			ConstString name = command.get(2).asString();
			ConstString filename = command.get(3).asString();
			responseStream << "Loading object <" << name << ">...";
			if (viewer->setObject(name.c_str(),filename.c_str()))
			{
				responseOK = true;
				responseStream << "ok";
			} else
			{
				responseOK = false;
				responseStream << "failed";
			}
			eaten = true;
		}
	} else if (command.get(0).asString()=="remove") 
	{
		if (command.get(1).asString()=="object") 
		{
			ConstString name = command.get(2).asString();
			viewer->removeObject(name.c_str());
			responseStream << "Removing object <" << name << ">...";
			responseOK = true;
			eaten = true;
		}
	} */
	else if (command.get(0).asString()=="set") 
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
			eaten = true;
		} else if (command.get(1).asString()=="segmentationThreshold") 
		{
			double t  = command.get(2).asDouble();
			responseOK = setSegThresh(t);
			if (responseOK)
			{
				responseStream << "Setting threshold OK ";
			} else
			{
				responseStream << "Setting threshold FAILED ";
			}
			eaten = true;
		} else if (command.get(1).asString()=="legomodel") 
		{
			std::string t  = command.get(2).asString().c_str();
			if (t=="Gate")
				setLegoModel(eGate);
			else
				setLegoModel(eXWing);
			responseOK = true;
			responseStream << "Setting Lego model to " << getModelString(legoModel);

			eaten = true;
		}
	}
	if (!customResponse)
	{
		reply.addInt((int)responseOK);
		reply.addString(responseStream.str().c_str());
	}
	if (!eaten)
	{
		responseStream << "Unknown command: " << command.toString().c_str() << "\n Try 'help' \n";
		cout << helpMessage;
	}
	cout << "reply: " << reply.toString() << endl;
	cout << "RESPONSE END" << endl;
	return true;
}

bool SimoxLegoLocalizerModule::interruptModule()
{
	handlerPort.interrupt();
	printf ("INTERRUPT\n");
	return true;
}

bool SimoxLegoLocalizerModule::close()
{
	robotDeviceHead.close();
	robotDeviceTorso.close();
	encHead = encTorso = NULL;
	
	handlerPort.close();
	
	return true;
}

double SimoxLegoLocalizerModule::getPeriod()
{
	// 50 fps
	return 0.02;
}


bool SimoxLegoLocalizerModule::setJointLimit( const std::string &robotNode, float min, float max )
{
	if (!robot)
	{
		VR_ERROR << "No robot..." << endl;
		return false;
	}
	RobotNodePtr rn = robot->getRobotNode(robotNode);
	if (!rn)
	{
		VR_ERROR << "No robot node with name " << robotNode << endl;
		return false;
	}

	rn->setJointLimits(min,max);
	return true;
}

bool SimoxLegoLocalizerModule::setJointLimits( const std::string &robotNodeSet, std::vector<float> &min, std::vector<float> &max )
{
	if (!robot)
	{
		VR_ERROR << "No robot..." << endl;
		return false;
	}
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
	return true;
}



bool SimoxLegoLocalizerModule::setupPorts()
{
	bool result = true;
	if (robotBase!="icubSim")
	{
		if (!yarp.exists(clientNameDisparity.c_str()))
		{
			if (!stereoDisparityPort.open(clientNameDisparity.c_str()))
			{
				VR_ERROR << "Could not open disparityPort..." << endl;
				result = false;
			}
		}
		if (yarp.exists(clientNameDisparity.c_str()) && !yarp.isConnected(clientNameDisparity.c_str(),serverNameDisparity.c_str()))
		{
			tryToConnect(clientNameDisparity,serverNameDisparity);
		}
	}

	if (!yarp.exists(clientNameBlobExtractorLeft.c_str()))
	{
		if (!blobExtractorLeftPort.open(clientNameBlobExtractorLeft.c_str()))
		{
			VR_ERROR << "Could not open port for left blob extractor RPC with name " << clientNameBlobExtractorLeft << "..." << endl;
			result = false;
		}
	}
	if (yarp.exists(clientNameBlobExtractorLeft.c_str()) && !yarp.isConnected(clientNameBlobExtractorLeft.c_str(),serverNameBlobExtractorLeft.c_str()))
	{
		tryToConnect(clientNameBlobExtractorLeft,serverNameBlobExtractorLeft);
	}

	if (!yarp.exists(clientNameBlobExtractorRight.c_str()))
	{
		if (!blobExtractorRightPort.open(clientNameBlobExtractorRight.c_str()))
		{
			VR_ERROR << "Could not open port for Right blob extractor RPC with name " << clientNameBlobExtractorRight << "..." << endl;
			result = false;
		}
	}
	if (yarp.exists(clientNameBlobExtractorRight.c_str()) && !yarp.isConnected(clientNameBlobExtractorRight.c_str(),serverNameBlobExtractorRight.c_str()))
	{
		tryToConnect(clientNameBlobExtractorRight,serverNameBlobExtractorRight);
	}

	if (sendResultsToRobotViewer)
	{
		if (!yarp.exists(clientNameRobotViewer.c_str()))
		{
			canSendResultsToRobotViewer = false;
			if (!simoxRobotViewerPort.open(clientNameRobotViewer.c_str()))
			{
				VR_ERROR << "Could not open port for Robot Viewer with name " << clientNameRobotViewer << "..." << endl;
			}
		}
		if (yarp.exists(clientNameRobotViewer.c_str()) && !yarp.isConnected(clientNameRobotViewer.c_str(),serverNameRobotViewer.c_str()))
		{
			canSendResultsToRobotViewer = tryToConnect(clientNameRobotViewer,serverNameRobotViewer);
		}
	}

	return result;
}

bool SimoxLegoLocalizerModule::tryToConnect(std::string &clientName, std::string &serverName)
{
	return yarp.connect(clientName.c_str(),serverName.c_str());
}

bool SimoxLegoLocalizerModule::checkConnections(bool reconnect)
{
	bool result = true;


	if (robotBase != "icubSim")
	{
		if (!yarp.exists(clientNameDisparity.c_str()))
			result = false;
		if (!yarp.isConnected(clientNameDisparity.c_str(),serverNameDisparity.c_str()))
			result = false;
	}
	if (!yarp.isConnected(clientNameBlobExtractorLeft.c_str(),serverNameBlobExtractorLeft.c_str()))
		result = false;

	if (!yarp.isConnected(clientNameBlobExtractorRight.c_str(),serverNameBlobExtractorRight.c_str()))
		result = false;

	if (!result && reconnect)
	{
		setupPorts();
		return checkConnections(false);
	}

	return result;

}
bool SimoxLegoLocalizerModule::setSegThresh(double t)
{
	if (!checkConnections(true))
	{
		VR_ERROR << "No connection to blobExtractor or disparityModule ?!" << endl;
		return false;
	}
	
	yarp::os::Bottle cmdLeft;
	yarp::os::Bottle responseLeft;
	yarp::os::Bottle cmdRight;
	yarp::os::Bottle responseRight;
	cmdLeft.addString("thresh");
	cmdLeft.addDouble(t);
	cmdRight.addString("thresh");
	cmdRight.addDouble(t);
	bool ok1 = blobExtractorLeftPort.write(cmdLeft,responseLeft);
	bool ok2 = blobExtractorRightPort.write(cmdRight,responseRight);
	cout << "responseLeft:" << responseLeft.toString() << endl;
	cout << "responseRight:" << responseRight.toString() << endl;
	return ok1 & ok2;
}

bool SimoxLegoLocalizerModule::localize(Eigen::Matrix4f &m, bool update, bool withObstacle, Eigen::Matrix4f &storeObstBlob)
{
	if (!robot)
	{
		VR_ERROR << "No robot..." << endl;
		return false;
	}

	if (!checkConnections(true))
	{
		VR_ERROR << "No connection to blobExtractor or disparityModule ?!" << endl;
		return false;
	}

	if (!updateRobotModel(true))
	{
		VR_ERROR << "Could not update robot model..." << endl;
		return false;
	}
	switch (legoModel)
	{
	case eGate:
		//return localizeGate_OneLargeBlob(m);
		return localizeGate_TwoSmallBlobs(m, update, withObstacle, storeObstBlob);
		break;
	case eXWing:
		return localizeXWing_TwoSmallBlobs(m, update, withObstacle,storeObstBlob);
		break;
	default:
		VR_ERROR << "Lego model not setup correctly ..." << endl;
		return false;
		break;
	}
	return false;

}

Eigen::Vector2f SimoxLegoLocalizerModule::getBlobCenter(yarp::os::Bottle *b)
{
	if (!b || b->size()!=4)
	{
		VR_ERROR << "internal error" << endl;
		return Eigen::Vector2f::Zero();
	}
	Eigen::Vector2f res;
	res[0] = 0.5f * (float)(b->get(0).asDouble() + b->get(2).asDouble());
	res[1] = 0.5f * (float)(b->get(1).asDouble() + b->get(3).asDouble());
	return res;
}

bool SimoxLegoLocalizerModule::localizeGate_TwoSmallBlobs(Eigen::Matrix4f &m, bool update, bool withObstacle, Eigen::Matrix4f &storeObstBlob)
{
	// assuming we have a two small legos at each upper end of the lego gate
	// the position is the mean
	// The orientation is computed by determining the orientation of both 3d points. 

	m.setIdentity();
	Eigen::Vector3f p3d_1_root = localized_p3d_1_root;
	Eigen::Vector3f p3d_2_root = localized_p3d_2_root;
	if (!localizeTwoBlobs(p3d_1_root,p3d_2_root,update,withObstacle,storeObstBlob))
	{
		return false;
	}
	localized_p3d_1_root = p3d_1_root;
	localized_p3d_2_root = p3d_2_root;
	/*
	yarp::os::Bottle cmdLeft;
	yarp::os::Bottle responseLeft;
	yarp::os::Bottle cmdRight;
	yarp::os::Bottle responseRight;
	cmdLeft.addString("segment");

	bool ok1 = blobExtractorLeftPort.write(cmdLeft,responseLeft);
	cmdRight.addString("segment");

	bool ok2 = blobExtractorRightPort.write(cmdRight,responseRight);
	
	if (!update)
	{
		// set stored points to zero
		localized_p3d_1_root.setZero();
		localized_p3d_2_root.setZero();
	}

	if (!ok1 || !ok2)
	{
		VR_ERROR << " Could not send rpc call to blob extractors..." << endl;
		return false;
	}

	// check results
	ok1 = (responseLeft.size()>0 && responseLeft.get(0).asVocab()!=Vocab::encode("empty"));
	ok2 = (responseRight.size()>0 && responseRight.get(0).asVocab()!=Vocab::encode("empty"));

	if (!ok1 || !ok2)
	{
		VR_ERROR << " Empty result from blob extractors..." << endl;
		cout << "responseLeft:" << responseLeft.toString().c_str() << endl;
		cout << "responseRight:" << responseRight.toString().c_str() << endl;
		return false;
	}

	filterBlobs(responseLeft,2);
	filterBlobs(responseRight,2);
	
	if (responseLeft.size()<2 || responseRight.size()<2)
	{
		VR_ERROR << " Expecting at least 2 blobs from blob extractors..." << endl;
		cout << "responseLeft:" << responseLeft.toString().c_str() << endl;
		cout << "responseRight:" << responseRight.toString().c_str() << endl;
		return false;
	}


	yarp::os::Bottle *blob1L = responseLeft.get(0).asList();
	yarp::os::Bottle *blob1R = responseRight.get(0).asList();
	yarp::os::Bottle *blob2L = responseLeft.get(1).asList();
	yarp::os::Bottle *blob2R = responseRight.get(1).asList();
	if (!blob1L || !blob1R || !blob2L || !blob2R)
	{
		VR_ERROR << " Empty result from blob extractors..." << endl;
		cout << "responseLeft:" << responseLeft.toString().c_str() << endl;
		cout << "responseRight:" << responseRight.toString().c_str() << endl;
		return false;
	}

	m.setIdentity();

	Eigen::Vector2f l1 = getBlobCenter(blob1L);
	Eigen::Vector2f l2 = getBlobCenter(blob2L);
	Eigen::Vector2f r1 = getBlobCenter(blob1R);
	Eigen::Vector2f r2 = getBlobCenter(blob2R);

	cout << "Left result: 1) " << l1[0] << "," << l1[1] << ", 2) " << l2[0] << "," << l2[1] << endl;
	cout << "Right result: 1) " << r1[0] << "," << r1[1] << ", 2) " << r2[0] << "," << r2[1] << endl;
	
	// todo: check if they are ordered correctly
	cout << "Todo: assuming that one blob is more left in *both* cam images that the second one..." << endl;
	if (l1[0] > l2[0])
	{
		cout << "xchanging LEFT blobs" << endl;
		Eigen::Vector2f tmp = l1;
		l1 = l2;
		l2 = tmp;
	}
	if (r1[0] > r2[0])
	{
		cout << "xchanging RIGHT blobs" << endl;
		Eigen::Vector2f tmp = r1;
		r1 = r2;
		r2 = tmp;
	}
	Eigen::Vector3f p3d_1_root;
	Eigen::Vector3f p3d_2_root;
	ok1 = get3DPoint(l1[0],l1[1],r1[0],r1[1], p3d_1_root);
	ok2 = get3DPoint(l2[0],l2[1],r2[0],r2[1], p3d_2_root);
	if (!ok1 || !ok2)
	{
		VR_ERROR << "No 3d points, aborting..." << endl;
		return false;
	}

	cout << "Point 1:";
	MathTools::print(p3d_1_root);
	cout << "Point 2:";
	MathTools::print(p3d_2_root);
	
	if (update && !localized_p3d_1_root.isZero() && !localized_p3d_2_root.isZero())
	{
		float lastFactor = 0.8f;
		cout << "considering previously localized positions (factor:" << lastFactor << ":" << endl;
		cout << "Old Point 1:";
		MathTools::print(localized_p3d_1_root);
		cout << "Old Point 2:";
		MathTools::print(localized_p3d_2_root);
		
		if ((localized_p3d_1_root-p3d_1_root).norm() > 200.0f)
		{
			cout << " WARNING LARGE DISTANCE BETWEEN OLD AND NEW POINT 1 !!!!!!!!!!!!!!!!!!" << endl;
		}
		if ((localized_p3d_2_root-p3d_2_root).norm() > 200.0f)
		{
			cout << " WARNING LARGE DISTANCE BETWEEN OLD AND NEW POINT 2 !!!!!!!!!!!!!!!!!!" << endl;
		}
		
		p3d_1_root = localized_p3d_1_root*lastFactor + p3d_1_root * (1.0f-lastFactor);
		p3d_2_root = localized_p3d_2_root*lastFactor + p3d_2_root * (1.0f-lastFactor);
		
		cout << "Point 1:";
		MathTools::print(p3d_1_root);
		cout << "Point 2:";
		MathTools::print(p3d_2_root);

	}
	*/
	localized_p3d_1_root = p3d_1_root;
	localized_p3d_2_root = p3d_2_root;


	Eigen::Vector3f oriDir = p3d_2_root - p3d_1_root;

	if (sendResultsToRobotViewer && canSendResultsToRobotViewer && rootCoordNode)
	{
		sendOrientationRobotViewer(p3d_1_root,p3d_2_root);
	}

	Eigen::Matrix4f rotM;
	Eigen::Vector3f dirLocked;
	createRotatedPose(oriDir,dirLocked,rotM); // assuming standard dir as (0/1/0) (in root as well as in global pos)

	m = rotM;

	cout << "RotM:\n" << rotM << endl;
	
	// store point to result
	m.block(0,3,3,1) = 0.5f * (p3d_2_root + p3d_1_root);

	cout << "m:\n" << m << endl;

	Eigen::Vector3f rpy;
	MathTools::eigen4f2rpy(m,rpy);

	cout << "RPY (root): ";
	MathTools::print(rpy);
	
	// print in global pose
	if (rootCoordNode)
	{
		Eigen::Matrix4f mG = rootCoordNode->toGlobalCoordinateSystem(m);
		MathTools::eigen4f2rpy(mG,rpy);
		cout << "POS (global):" << mG(0,3) << "," << mG(1,3) << "," << mG(2,3) << endl;
		cout << "RPY (global): ";
		MathTools::print(rpy);
	}
	
	if (sendResultsToRobotViewer && canSendResultsToRobotViewer && rootCoordNode)
	{
		Eigen::Vector3f pt = m.block(0,3,3,1);
		Eigen::Vector3f p_global = rootCoordNode->toGlobalCoordinateSystemVec(pt);
		Eigen::Vector3f z = Eigen::Vector3f::Zero();
		Eigen::Vector3f zeroRoot_global = rootCoordNode->toGlobalCoordinateSystemVec(z);
		Eigen::Vector3f oriDir_global = rootCoordNode->toGlobalCoordinateSystemVec(dirLocked);
		// since the orientation is a vector we have to re-adjust the values
		oriDir_global -= zeroRoot_global;

		sendObjectRobotViewer(p_global,oriDir_global,"GATE");
		if (withObstacle)
		{
			Eigen::Vector3f pt2 = storeObstBlob.block(0,3,3,1);
			Eigen::Vector3f p_global2 = rootCoordNode->toGlobalCoordinateSystemVec(pt2);
			Eigen::Vector3f ori;
			ori << 0,0,1.0f;
			sendObjectRobotViewer(p_global2,ori,"Obstacle");
		}
	}

	return true;
}
bool SimoxLegoLocalizerModule::sendOrientationRobotViewer(Eigen::Vector3f p3d_1_root, Eigen::Vector3f p3d_2_root)
{
	if (!rootCoordNode)
		return false;
	Eigen::Vector3f oriDir = p3d_2_root - p3d_1_root;
	Eigen::Vector3f p1_global = rootCoordNode->toGlobalCoordinateSystemVec(p3d_1_root);
	Eigen::Vector3f p2_global = rootCoordNode->toGlobalCoordinateSystemVec(p3d_2_root);
	Eigen::Vector3f z = Eigen::Vector3f::Zero();
	Eigen::Vector3f zeroRoot_global = rootCoordNode->toGlobalCoordinateSystemVec(z);
	Eigen::Vector3f oriDir_global = rootCoordNode->toGlobalCoordinateSystemVec(oriDir);
	// since the orientation is a vector we have to re-adjust the values
	oriDir_global -= zeroRoot_global;

	Bottle cmd;
	cmd.addString("show");
	cmd.addString("vector");
	cmd.addString("Object1");
	cmd.addString("on");
	cmd.addDouble((double)p1_global[0]);
	cmd.addDouble((double)p1_global[1]);
	cmd.addDouble((double)p1_global[2]);
	cmd.addDouble((double)oriDir_global[0]);
	cmd.addDouble((double)oriDir_global[1]);
	cmd.addDouble((double)oriDir_global[2]);
	cmd.addDouble(0.4);
	// send to robot viewer
	Bottle response;
	bool ok = simoxRobotViewerPort.write(cmd,response);
	if (!ok)
		cout << "failed writing to simox robot viewer rpc port..." << endl;
	Bottle cmd2;
	cmd2.addString("show");
	cmd2.addString("vector");
	cmd2.addString("Object2");
	cmd2.addString("on");
	cmd2.addDouble((double)p2_global[0]);
	cmd2.addDouble((double)p2_global[1]);
	cmd2.addDouble((double)p2_global[2]);
	cmd2.addDouble((double)-oriDir_global[0]);
	cmd2.addDouble((double)-oriDir_global[1]);
	cmd2.addDouble((double)-oriDir_global[2]);
	cmd2.addDouble(0.4);

	// send to robot viewer
	Bottle response2;
	bool ok2 = simoxRobotViewerPort.write(cmd2,response2);
	if (!ok2)
		cout << "failed writing to simox robot viewer rpc port2..." << endl;

	return ok2;
}

bool SimoxLegoLocalizerModule::localizeTwoBlobs(Eigen::Vector3f &p3d_1_root, Eigen::Vector3f &p3d_2_root, bool update, bool withObstacle, Eigen::Matrix4f &storeObstPose)
{
	yarp::os::Bottle cmdLeft;
	yarp::os::Bottle responseLeft;
	yarp::os::Bottle cmdRight;
	yarp::os::Bottle responseRight;
	cmdLeft.addString("segment");
	/*cmdLeft.addInt(2); // nr blobs
	cmdLeft.addInt(20); // minPixels
	cmdLeft.addInt(40); // maxPixels*/
	bool ok1 = blobExtractorLeftPort.write(cmdLeft,responseLeft);
	cmdRight.addString("segment");
	/*cmdRight.addInt(2); // nr blobs
	cmdRight.addInt(20); // minPixels
	cmdRight.addInt(40); // maxPixels*/
	bool ok2 = blobExtractorRightPort.write(cmdRight,responseRight);
	Eigen::Vector3f oldPoints1 = p3d_1_root;
	Eigen::Vector3f oldPoints2 = p3d_2_root;
	if (!update)
	{
		// set stored points to zero
		oldPoints1.setZero();
		oldPoints2.setZero();
	}


	if (!ok1 || !ok2)
	{
		VR_ERROR << " Could not send rpc call to blob extractors..." << endl;
		return false;
	}

	// check results
	ok1 = (responseLeft.size()>0 && responseLeft.get(0).asVocab()!=Vocab::encode("empty"));
	ok2 = (responseRight.size()>0 && responseRight.get(0).asVocab()!=Vocab::encode("empty"));

	if (!ok1 || !ok2)
	{
		VR_ERROR << " Empty result from blob extractors..." << endl;
		cout << "responseLeft:" << responseLeft.toString().c_str() << endl;
		cout << "responseRight:" << responseRight.toString().c_str() << endl;
		return false;
	}
	int nrBlobs =2;
	if (withObstacle)
		nrBlobs = 3;
	filterBlobs(responseLeft,nrBlobs);
	filterBlobs(responseRight,nrBlobs);

	if (responseLeft.size()<2 || responseRight.size()<2)
	{
		VR_ERROR << " Expecting at least 2 blobs from blob extractors..." << endl;
		cout << "responseLeft:" << responseLeft.toString().c_str() << endl;
		cout << "responseRight:" << responseRight.toString().c_str() << endl;
		return false;
	}

	if (withObstacle)
	{
		storeObstPose.setIdentity();
		if (responseLeft.size()<3 || responseRight.size()<3)
		{
			VR_WARNING << " Expecting at least 3 blobs from blob extractors for obstacle detection..." << endl;
			
		} else
		{
			yarp::os::Bottle *blobOl = responseLeft.get(2).asList();
			yarp::os::Bottle *blobOr = responseRight.get(2).asList();
			Eigen::Vector2f ol = getBlobCenter(blobOl);
			Eigen::Vector2f ori = getBlobCenter(blobOr);

			cout << "OBSTACLE result: 1) " << ol[0] << "," << ol[1] << ", 2) " << ori[0] << "," << 
ori[1] << endl;
			Eigen::Vector3f p3d_obst_root;
			bool ok3 = get3DPoint(ol[0],ol[1],ori[0],ori[1], p3d_obst_root);
			if (!ok3)
			{
				VR_ERROR << "No 3d point for obstacle, skipping obstacle detection..." << endl;
			} else
			{
				cout << "Obstacle point [root]:";
				MathTools::print(p3d_obst_root);
				storeObstPose.block(0,3,3,1) = p3d_obst_root;
			}
		}
	}


	yarp::os::Bottle *blob1L = responseLeft.get(0).asList();
	yarp::os::Bottle *blob1R = responseRight.get(0).asList();
	yarp::os::Bottle *blob2L = responseLeft.get(1).asList();
	yarp::os::Bottle *blob2R = responseRight.get(1).asList();
	if (!blob1L || !blob1R || !blob2L || !blob2R)
	{
		VR_ERROR << " Empty result from blob extractors..." << endl;
		cout << "responseLeft:" << responseLeft.toString().c_str() << endl;
		cout << "responseRight:" << responseRight.toString().c_str() << endl;
		return false;
	}


	Eigen::Vector2f l1 = getBlobCenter(blob1L);
	Eigen::Vector2f l2 = getBlobCenter(blob2L);
	Eigen::Vector2f r1 = getBlobCenter(blob1R);
	Eigen::Vector2f r2 = getBlobCenter(blob2R);

	cout << "Left result: 1) " << l1[0] << "," << l1[1] << ", 2) " << l2[0] << "," << l2[1] << endl;
	cout << "Right result: 1) " << r1[0] << "," << r1[1] << ", 2) " << r2[0] << "," << r2[1] << endl;

	// todo: check if they are ordered correctly
	cout << "Todo: assuming that one blob is more left in *both* cam images that the second one..." << endl;
	if (l1[0] > l2[0])
	{
		cout << "xchanging LEFT blobs" << endl;
		Eigen::Vector2f tmp = l1;
		l1 = l2;
		l2 = tmp;
	}
	if (r1[0] > r2[0])
	{
		cout << "xchanging RIGHT blobs" << endl;
		Eigen::Vector2f tmp = r1;
		r1 = r2;
		r2 = tmp;
	}
	ok1 = get3DPoint(l1[0],l1[1],r1[0],r1[1], p3d_1_root);
	ok2 = get3DPoint(l2[0],l2[1],r2[0],r2[1], p3d_2_root);
	if (!ok1 || !ok2)
	{
		VR_ERROR << "No 3d points, aborting..." << endl;
		return false;
	}

	cout << "Point 1:";
	MathTools::print(p3d_1_root);
	cout << "Point 2:";
	MathTools::print(p3d_2_root);

	if (update && !oldPoints1.isZero() && !oldPoints2.isZero())
	{
		float lastFactor = 0.8f;
		cout << "considering previously localized positions (factor:" << lastFactor << ":" << endl;
		cout << "Old Point 1:";
		MathTools::print(oldPoints1);
		cout << "Old Point 2:";
		MathTools::print(oldPoints2);

		if ((oldPoints1-p3d_1_root).norm() > 200.0f)
		{
			cout << " WARNING LARGE DISTANCE BETWEEN OLD AND NEW POINT 1 !!!!!!!!!!!!!!!!!!" << endl;
		}
		if ((oldPoints2-p3d_2_root).norm() > 200.0f)
		{
			cout << " WARNING LARGE DISTANCE BETWEEN OLD AND NEW POINT 2 !!!!!!!!!!!!!!!!!!" << endl;
		}

		p3d_1_root = oldPoints1*lastFactor + p3d_1_root * (1.0f-lastFactor);
		p3d_2_root = oldPoints2*lastFactor + p3d_2_root * (1.0f-lastFactor);

		cout << "Point 1:";
		MathTools::print(p3d_1_root);
		cout << "Point 2:";
		MathTools::print(p3d_2_root);

	}
	return true;
}


bool SimoxLegoLocalizerModule::localizeXWing_TwoSmallBlobs(Eigen::Matrix4f &m, bool update, bool withObstacle, Eigen::Matrix4f &storeObstBlob)
{
	// assuming we have a two small legos at the connection of the x-wings
	// the position is the mean
	// The orientation is computed by determining the orientation of both 3d points. 

	m.setIdentity();
	Eigen::Vector3f p3d_1_root = localized_p3d_1_root;
	Eigen::Vector3f p3d_2_root = localized_p3d_2_root;
	if (!localizeTwoBlobs(p3d_1_root, p3d_2_root, update, withObstacle, storeObstBlob ))
	{
		return false;
	}
	localized_p3d_1_root = p3d_1_root;
	localized_p3d_2_root = p3d_2_root;


	Eigen::Vector3f oriDir = p3d_2_root - p3d_1_root;

	if (sendResultsToRobotViewer && canSendResultsToRobotViewer && rootCoordNode)
	{
		sendOrientationRobotViewer(p3d_1_root,p3d_2_root);
	}

	Eigen::Matrix4f rotM;
	Eigen::Vector3f dirLocked;
	createRotatedPose(oriDir,dirLocked,rotM); // assuming standard dir as (0/1/0) (in root as well as in global pos)

	m = rotM;

	cout << "RotM:\n" << rotM << endl;
	
	// store point to result
	m.block(0,3,3,1) = 0.5f * (p3d_2_root + p3d_1_root);

	cout << "m:\n" << m << endl;

	Eigen::Vector3f rpy;
	MathTools::eigen4f2rpy(m,rpy);

	cout << "RPY (root): ";
	MathTools::print(rpy);
	
	// print in global pose
	if (rootCoordNode)
	{
		Eigen::Matrix4f mG = rootCoordNode->toGlobalCoordinateSystem(m);
		MathTools::eigen4f2rpy(mG,rpy);
		cout << "POS (global):" << mG(0,3) << "," << mG(1,3) << "," << mG(2,3) << endl;
		cout << "RPY (global): ";
		MathTools::print(rpy);
	}
	
	if (sendResultsToRobotViewer && canSendResultsToRobotViewer && rootCoordNode)
	{
		Eigen::Vector3f pt = m.block(0,3,3,1);
		Eigen::Vector3f p_global = rootCoordNode->toGlobalCoordinateSystemVec(pt);
		Eigen::Vector3f z = Eigen::Vector3f::Zero();
		Eigen::Vector3f zeroRoot_global = rootCoordNode->toGlobalCoordinateSystemVec(z);
		Eigen::Vector3f oriDir_global = rootCoordNode->toGlobalCoordinateSystemVec(dirLocked);
		// since the orientation is a vector we have to re-adjust the values
		oriDir_global -= zeroRoot_global;

		sendObjectRobotViewer(p_global,oriDir_global,"XWing");

		if (withObstacle)
		{
			Eigen::Vector3f pt2 = storeObstBlob.block(0,3,3,1);
			Eigen::Vector3f p_global2 = rootCoordNode->toGlobalCoordinateSystemVec(pt2);
			Eigen::Vector3f ori;
			ori << 0,0,1.0f;
			sendObjectRobotViewer(p_global2,ori,"Obstacle");
		}
	}

	return true;
}


bool SimoxLegoLocalizerModule::filterBlobs(yarp::os::Bottle &b, int nrBlobs)
{
	if (b.size() < nrBlobs)
		return false;

	cout << "Sorting blob list considering distance from img center (160/200) :" << b.toString().c_str() << endl;

	yarp::os::Bottle tmp = b;
	yarp::os::Bottle res;

	int cx = 160;
	int cy = 200;
	while (res.size() < tmp.size())
	{
		float minDist = 10000000;
		int minDistIndx = -1;
		for (int i=0;i<tmp.size();i++)
		{
			yarp::os::Bottle * e = tmp.get(i).asList();
			if (!e || e->size()==0)
				continue;

			float x = (float)e->get(0).asDouble();
			float y = (float)e->get(1).asDouble();
			float dx = x - cx; 
			float dy = y - cy;
			float d = dx*dx + dy*dy;
			if (d<minDist)
			{
				minDist = d;
				minDistIndx = i;
			}
		}
		if (minDistIndx<0)
			return false;
		yarp::os::Bottle * e = tmp.get(minDistIndx).asList();
		if (!e)
		{
			VR_ERROR << "NULL bottle?!" << endl;
			return false;
		}
		yarp::os::Bottle &l = res.addList();
		l = *e;
		e->clear();
		cout << "new res list: " << res.toString().c_str() << endl;
	}

	b = res;
	cout << "Sorted blob list:" << b.toString().c_str() << endl;
	
	cout << "Sorting " << nrBlobs << " blobs from left to right " << endl;
	std::map<int,yarp::os::Bottle> sortedBlobs;
	if (b.size() < nrBlobs)
		nrBlobs = b.size();
	tmp = b;
	res.clear();
	for (int i=0; i<nrBlobs;i++)
	{
		float minX = 10000000;
		int minIndx = -1;
		for (int i=0;i<tmp.size();i++)
		{
			yarp::os::Bottle * e = tmp.get(i).asList();
			if (!e || e->size()==0)
				continue;

			float x = (float)e->get(0).asDouble();
	
			if (x<minX)
			{
				minX = x;
				minIndx = i;
			}
		}
		if (minIndx<0)
			return false;
		yarp::os::Bottle * e = tmp.get(minIndx).asList();
		if (!e)
		{
			VR_ERROR << "NULL bottle?!" << endl;
			return false;
		}
		yarp::os::Bottle &l = res.addList();
		l = *e;
		e->clear();
		cout << "new res list: " << res.toString().c_str() << endl;

	}
	b = res;
	return true;
}

bool SimoxLegoLocalizerModule::localizeGate_OneLargeBlob(Eigen::Matrix4f &m)
{
	// assuming we have a large long lego that we can localize
	// The orientation is computed by intersecting two planes which are constructed with the orientation in the cam images and the 3d point
	yarp::os::Bottle cmdLeft;
	yarp::os::Bottle responseLeft;
	yarp::os::Bottle cmdRight;
	yarp::os::Bottle responseRight;
	cmdLeft.addString("oriSegment");
	bool ok1 = blobExtractorLeftPort.write(cmdLeft,responseLeft);
	cmdRight.addString("oriSegment");
	bool ok2 = blobExtractorRightPort.write(cmdRight,responseRight);

	if (!ok1 || !ok2)
	{
		VR_ERROR << " Could not send rpc call to blob extractors..." << endl;
		return false;
	}

	// check results
	ok1 = (responseLeft.size()>0 && responseLeft.get(0).asVocab()!=Vocab::encode("empty"));
	ok2 = (responseRight.size()>0 && responseRight.get(0).asVocab()!=Vocab::encode("empty"));

	if (!ok1 || !ok2)
	{
		VR_ERROR << " Empty result from blob extractors..." << endl;
		cout << "responseLeft:" << responseLeft.toString().c_str() << endl;
		cout << "responseRight:" << responseRight.toString().c_str() << endl;
		return false;
	}

	if (responseLeft.size()!=2 || responseRight.size()!=2)
	{
		VR_ERROR << " Expecting 1 blob from blob extractors..." << endl;
		cout << "responseLeft:" << responseLeft.toString().c_str() << endl;
		cout << "responseRight:" << responseRight.toString().c_str() << endl;
		return false;
	}


	yarp::os::Bottle *blobL = responseLeft.get(0).asList();
	yarp::os::Bottle *blobR = responseRight.get(0).asList();
	yarp::os::Bottle *angleL = responseLeft.get(1).asList();
	yarp::os::Bottle *angleR = responseRight.get(1).asList();
	if (!blobL || !blobR || !angleR || !angleR)
	{
		VR_ERROR << " Empty result from blob extractors..." << endl;
		cout << "responseLeft:" << responseLeft.toString().c_str() << endl;
		cout << "responseRight:" << responseRight.toString().c_str() << endl;
		return false;
	}

	m.setIdentity();

	Eigen::Vector2f l1 = getBlobCenter(blobL);
	float xL = l1[0];
	float yL = l1[1];
	float aL = (float)angleL->get(0).asDouble();
	
	Eigen::Vector2f r1 = getBlobCenter(blobR);
	float xR = r1[0];
	float yR = r1[1];
	float aR = (float)angleR->get(0).asDouble();

	cout << "Left result:" << xL << "," << yL << ", a:" << aL << endl;
	cout << "Right result:" << xR << "," << yR << ", a:" << aR << endl;

	Eigen::Vector3f p3d;
	ok1 = get3DPoint(xL,yL,xR,yR, p3d);
	if (!ok1)
	{
		VR_ERROR << "No 3d point, aborting..." << endl;
		return false;
	}
	Eigen::Matrix4f rotM;
	ok2 = getGlobalOrientation(xL, yL, aL, xR, yR, aR, p3d, rotM);
	if (ok2)
	{
		cout << "RotM:\n" << rotM << endl;
		if (rootCoordNode)
			rotM = rootCoordNode->toLocalCoordinateSystem(rotM);
		m = rotM; // the position is wrong here, but it is set later!
	}
	// store point to result
	m.block(0,3,3,1) = p3d;

	// check icub Sim coords:
	Eigen::Matrix4f tr;
	tr <<	0, -1.0f, 0, 0, 
			0, 0, 1.0f, 597.6f,
			-1.0f, 0, 0, -26.0f, 
			0, 0, 0, 1; // - -1.0f  0, 0, -26.0f, ?
	Eigen::Matrix4f poseSimA = tr * m;
	cout << "Pose in simulator:\n" << poseSimA << endl;
	/*Eigen::Matrix4f poseSim = tr.inverse() * m;
	cout << "Pose in simulator:\n" << poseSim << endl;
	Eigen::Matrix4f poseSim2 = m * tr.inverse();
	cout << "Pose in simulator2:\n" << poseSim2 << endl;
	Eigen::Matrix4f poseSim3 = m * tr;
	cout << "Pose in simulator3:\n" << poseSim3 << endl;*/

	return true;
}

bool SimoxLegoLocalizerModule::getGlobalOrientation(float xL, float yL, float aL, float xR, float yR, float aR, const Eigen::Vector3f &p3d_root,  Eigen::Matrix4f &storeRot)
{
	if (!camLeftCoordNode || !camRightCoordNode)
		return false;

	aL = -aL/180.0f * (float)M_PI; // we are assuming a different orientation
	aR = -aR/180.0f * (float)M_PI;

	// create a vector, pointing in rotated 2d dir
	float xnl = 1.0f * cos(aL) - 0.0f * sin(aL);
	float ynl = 1.0f * sin(aL) - 0.0f * cos(aL);
	float xnr = 1.0f * cos(aR) - 0.0f * sin(aR);
	float ynr = 1.0f * sin(aR) - 0.0f * cos(aR);

	// create orientation vector in cam coord system
	Eigen::Vector3f oL;
	oL << xnl,ynl,0;
	Eigen::Vector3f oR;
	oR << xnr,ynr,0;

	cout << "orientation vec in left cam coord system:";
	MathTools::print(oL);
	cout << "orientation vec in right cam coord system:";
	MathTools::print(oR);

	// zero point in cam coord system
	Eigen::Vector3f zL = Eigen::Vector3f::Zero();
	Eigen::Vector3f zR = Eigen::Vector3f::Zero();

	// convert to global coord system
	oL = camLeftCoordNode->toGlobalCoordinateSystemVec(oL);
	zL = camLeftCoordNode->toGlobalCoordinateSystemVec(zL);
	oR = camRightCoordNode->toGlobalCoordinateSystemVec(oR);
	zR = camRightCoordNode->toGlobalCoordinateSystemVec(zR);
	// since the orientation is a vector we have to re-adjust the values
	oL -= zL;
	oR -= zR;

	cout << "orientation vec in global coord system L:";
	MathTools::print(oL);
	cout << "orientation vec in global coord system R:";
	MathTools::print(oR);

	// now we compute the vector from cam to p3d
	Eigen::Vector3f p3d_global = rootCoordNode->toGlobalCoordinateSystemVec(p3d_root);
	Eigen::Vector3f cam2pL = p3d_global - zL;
	Eigen::Vector3f cam2pR = p3d_global - zR;

	cout << "pos in global coord system:";
	MathTools::print(p3d_global);

	// check if vectors are orthogonal
	if ( (fabs(oL.dot(cam2pL)) < 1e-9) || (fabs(oR.dot(cam2pR)) < 1e-9))
	{
		VR_ERROR << "Orthogonal vectors (cam-to-p3d and orientation), could not create plane..." << endl;
		return false;
	}

	Eigen::Vector3f nL = oL.cross(cam2pL);
	Eigen::Vector3f nR = oR.cross(cam2pR);


	// orthogonal to this
	/*float tmp = xnl;
	xnl = -ynl;
	ynl = tmp;
	tmp = xnr;
	xnr = -ynr;
	ynr = tmp;*/

	if (sendResultsToRobotViewer && canSendResultsToRobotViewer)
	{
		Bottle cmd;
		cmd.addString("show");
		cmd.addString("plane");//vector
		cmd.addString("Object_Left_Cam");
		cmd.addString("on");
		cmd.addDouble((double)p3d_global[0]);
		cmd.addDouble((double)p3d_global[1]);
		cmd.addDouble((double)p3d_global[2]);
		cmd.addDouble((double)nL[0]);
		cmd.addDouble((double)nL[1]);
		cmd.addDouble((double)nL[2]);
		// send to robot viewer
		Bottle response;
		bool ok = simoxRobotViewerPort.write(cmd,response);
		if (!ok)
			cout << "failed writing to simox robot viewer rpc port..." << endl;
		Bottle cmd2;
		cmd2.addString("show");
		cmd2.addString("plane");//vector
		cmd2.addString("Object_Right_Cam");
		cmd2.addString("on");
		cmd2.addDouble((double)p3d_global[0]);
		cmd2.addDouble((double)p3d_global[1]);
		cmd2.addDouble((double)p3d_global[2]);
		cmd2.addDouble((double)nR[0]);
		cmd2.addDouble((double)nR[1]);
		cmd2.addDouble((double)nR[2]);
		// send to robot viewer
		Bottle response2;
		bool ok2 = simoxRobotViewerPort.write(cmd2,response2);
		if (!ok2)
			cout << "failed writing to simox robot viewer rpc port2..." << endl;
	}
	if (sendResultsToRobotViewer && canSendResultsToRobotViewer)
	{
		Bottle cmd;
		cmd.addString("show");
		cmd.addString("vector");
		cmd.addString("Object_Left_Cam_V");
		cmd.addString("on");
		cmd.addDouble((double)p3d_global[0]);
		cmd.addDouble((double)p3d_global[1]);
		cmd.addDouble((double)p3d_global[2]);
		cmd.addDouble((double)nL[0]);
		cmd.addDouble((double)nL[1]);
		cmd.addDouble((double)nL[2]);
		// send to robot viewer
		Bottle response;
		bool ok = simoxRobotViewerPort.write(cmd,response);
		if (!ok)
			cout << "failed writing to simox robot viewer rpc port..." << endl;
		Bottle cmd2;
		cmd2.addString("show");
		cmd2.addString("vector");
		cmd2.addString("Object_Right_Cam_V");
		cmd2.addString("on");
		cmd2.addDouble((double)p3d_global[0]);
		cmd2.addDouble((double)p3d_global[1]);
		cmd2.addDouble((double)p3d_global[2]);
		cmd2.addDouble((double)nR[0]);
		cmd2.addDouble((double)nR[1]);
		cmd2.addDouble((double)nR[2]);
		// send to robot viewer
		Bottle response2;
		bool ok2 = simoxRobotViewerPort.write(cmd2,response2);
		if (!ok2)
			cout << "failed writing to simox robot viewer rpc port2..." << endl;
	}
	VirtualRobot::MathTools::Plane pL_global(p3d_global,nL);
	VirtualRobot::MathTools::Plane pR_global(p3d_global,nR);

	VirtualRobot::MathTools::Line l = VirtualRobot::MathTools::intersectPlanes(pL_global,pR_global);

	if (!l.isValid())
	{
		VR_ERROR << "Invalid intersection?! Parallel planes?! Aborting..." << endl;
		return false;
	}
	if (sendResultsToRobotViewer && canSendResultsToRobotViewer)
	{
		Bottle cmd;
		cmd.addString("show");
		cmd.addString("vector");
		cmd.addString("Object_Ori_3D");
		cmd.addString("on");
		cmd.addDouble((double)l.p[0]);
		cmd.addDouble((double)l.p[1]);
		cmd.addDouble((double)l.p[2]);
		cmd.addDouble((double)l.d[0]);
		cmd.addDouble((double)l.d[1]);
		cmd.addDouble((double)l.d[2]);
		// send to robot viewer
		Bottle response;
		bool ok = simoxRobotViewerPort.write(cmd,response);
		if (!ok)
			cout << "failed writing to simox robot viewer rpc port3..." << endl;
	}

	cout << "Intersection:" << endl;
	cout << "p:" << endl;
	MathTools::print(l.p);
	cout << "dir:" << endl;
	MathTools::print(l.d);

	// create pose from direction vector
	Eigen::Vector3f dirLocked;
	return createRotatedPose(l.d,dirLocked,storeRot);
}

bool SimoxLegoLocalizerModule::createRotatedPose(const Eigen::Vector3f &dir_global, Eigen::Vector3f &locked_dir_global, Eigen::Matrix4f &poseRot)
{
	if (dir_global.norm()<1e-9)
		return false;

	Eigen::Vector3f standardOri;
	switch (legoModel)
	{
	case eGate:
		cout << "Assuming an upright standing gate..." << endl;
		//standardOri << 0,1.0f,0; // orient points in y-dir (right)
		standardOri << 1.0f,0,0; // orient points in x-dir (towards)
		break;
	case eXWing:
		cout << "Assuming a lying XWing..." << endl;
		standardOri << 1.0f,0,0; // orient points in x-dir (towards)
		//standardOri << 0,0,1.0f; // orient points in z-dir
		break;
	default:
		VR_ERROR << "Model nyi..." << endl;
		return false;
	}

	cout << "Orientation (root):";
	MathTools::print (dir_global);
	Eigen::Vector3f pt(0,0,0);
	Eigen::Vector3f no(0,0,1.0f);
	MathTools::Plane p(pt,no);
	locked_dir_global = dir_global;
	locked_dir_global.normalize();
	if (fabs(locked_dir_global.dot(no)) > 1e-6f)
		locked_dir_global = MathTools::projectPointToPlane(locked_dir_global,p);
	cout << "Orientation locked to upright position (root):";
	MathTools::print (locked_dir_global);
	
	MathTools::Quaternion q = MathTools::getRotation(standardOri,locked_dir_global);
	poseRot = MathTools::quat2eigen4f(q);
	return true;
}

bool SimoxLegoLocalizerModule::get3DPoint(float xL, float yL, float xR, float yR, Eigen::Vector3f &store3d)
{
	Bottle cmd,response;
	cmd.addDouble((double)xL);
	cmd.addDouble((double)yL);
	cmd.addDouble((double)xR);
	cmd.addDouble((double)yR);
	bool ok = false;

	//TEST check the triangulate result:
	Vector px_left(2);
	px_left[0] = xL;
	px_left[1] = yL;	
	Vector px_right(2);
	px_right[0] = xR;
	px_right[1] = yR;
	Vector res_tri;
	if (iCubGazeControl->triangulate3DPoint(px_left,px_right,res_tri))
	{
		Eigen::Vector3f res_tri_v;
		res_tri_v << (float)res_tri[0], (float)res_tri[1], (float)res_tri[2];
		res_tri_v *= 1000.0f;
		cout << "TEST: 3d pos (iKinGazeCntrl STEREO (root_coord):";
		MathTools::print(res_tri_v);
	} else
	{
		cout << "iKinGazeCntrl STEREO FAILED..." << endl;
	}
	// END TEST

	if (robotBase!="icubSim")
	{

		cout << "sending " << cmd.toString().c_str() << " to stereoDisparityPort" << endl;
		ok = stereoDisparityPort.write(cmd,response);
		cout << "respond: " << response.toString().c_str() << endl;
		if (!ok)
		{
			cout << "Error requesting 3d stereo data..." << endl;
			//return false;
		} else
		{
			int nrPoints = response.size() / 3;
			cout << "Retrieving " << nrPoints << " stereo 3d points_rootCoord:" << endl;
			if (nrPoints!=1)
			{
				cout << "Expecting 1 point, trying to use mono approach..." << endl;
				ok = false;
			} else
			{
				int i = 0;
				store3d(0) = (float)response.get(i*3).asDouble() * 1000.0f; // we use mm
				store3d(1) = (float)response.get(i*3+1).asDouble() * 1000.0f;
				store3d(2) = (float)response.get(i*3+2).asDouble() * 1000.0f;
				if (!MathTools::isValid(store3d))
				{
					cout << "invalid point data, switching to mono approach..." << endl;
					ok = false;
				} else
					ok = true;
			}
		}
	}

	if (!ok && iCubGazeControl)
	{
		cout << "Stereo failed, using mono approach to get 3d pos..." << endl;
		Vector posL;
		Vector posR;
		Vector px(2);
		px[0] = xL;
		px[1] = yL;
		float z = 0.5f; // m
		// 0: left, 1: right, gives point in root_coord
		if (!iCubGazeControl->get3DPoint(0,px,z,posL))
		{
			VR_ERROR << "Error in retrieving 3d pos (mono,l)" << endl;
			return false;
		}
		px[0] = xR;
		px[1] = yR;
		if (!iCubGazeControl->get3DPoint(1,px,z,posR))
		{
			VR_ERROR << "Error in retrieving 3d pos (mono,r)" << endl;
			return false;
		}


		Eigen::Vector3f pL;
		pL << (float)posL[0], (float)posL[1], (float)posL[2];
		pL *= 1000.0f;
		Eigen::Vector3f pR;
		pR << (float)posR[0], (float)posR[1], (float)posR[2];
		pR *= 1000.0f;
		cout << "3d pos (left iKinGazeCntrl (root_coord):";
		MathTools::print(pL);
		cout << "3d pos (right iKinGazeCntrl (root_coord):";
		MathTools::print(pR);
		store3d = 0.5f * (pL + pR);
		ok = true;
	}
	if (ok)
	{
		cout << "** Point in 3D (root coord):" << store3d[0] << "," << store3d[1] << "," << store3d[2] << endl;
		if (sendResultsToRobotViewer && canSendResultsToRobotViewer && rootCoordNode)
		{
			Eigen::Vector3f posGlob = rootCoordNode->toGlobalCoordinateSystemVec(store3d);
			Bottle cmd;
			cmd.addString("show");
			cmd.addString("vector");
			cmd.addString("Object3DPos");
			cmd.addString("on");
			cmd.addDouble((double)posGlob[0]);
			cmd.addDouble((double)posGlob[1]);
			cmd.addDouble((double)posGlob[2]);
			cmd.addDouble(0); // only pos
			cmd.addDouble(0);
			cmd.addDouble(0);
			// send to robot viewer
			Bottle response;
			bool ok = simoxRobotViewerPort.write(cmd,response);
			if (!ok)
				cout << "failed writing to simox robot viewer rpc port..." << endl;
		}
	}
	return ok;
}

void SimoxLegoLocalizerModule::setLegoModel( LegoModel lm )
{
	legoModel = lm;
	cout << "Setting Lego Model to " << getModelString(lm) << endl;
}

std::string SimoxLegoLocalizerModule::getModelString( LegoModel lm )
{
	std::string result = "<not set>";
	switch (lm)
	{
	case eGate:
		result = "Gate";
		break;
	case eXWing:
		result = "XWing";
		break;
	default:
		result = "unknown";
	}
	return result;
}

bool SimoxLegoLocalizerModule::sendObjectRobotViewer( Eigen::Vector3f p_global, Eigen::Vector3f oriDir_global, std::string name )
{
	Bottle cmd;
	cmd.addString("show");
	cmd.addString("vector");
	cmd.addString(name.c_str());
	cmd.addString("on");
	cmd.addDouble((double)p_global[0]);
	cmd.addDouble((double)p_global[1]);
	cmd.addDouble((double)p_global[2]);
	cmd.addDouble((double)oriDir_global[0]);
	cmd.addDouble((double)oriDir_global[1]);
	cmd.addDouble((double)oriDir_global[2]);
	cmd.addDouble(0.8);
	// send to robot viewer
	Bottle response;
	bool ok = simoxRobotViewerPort.write(cmd,response);
	if (!ok)
		cout << "failed writing to simox robot viewer rpc port..." << endl;
	return ok;
}

