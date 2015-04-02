
#include "SimoxRobotViewerModule.h"
#include <VirtualRobot/MathTools.h>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace std;
using namespace VirtualRobot;


SimoxRobotViewerModule::SimoxRobotViewerModule()
{
	if (!SoDB::isInitialized())
		SoDB::init();
	SoQt::init("RobotViewer");
	timeUpdateEncodersS = 0.1;
	lastEncoderUpdateS = Time::now();
	encRightArm = encLeftArm = encHead = encTorso = NULL;
	visuMode = eShowCurrentRobotState;
	updateFingersLeft = updateFingersRight = true;
}



bool SimoxRobotViewerModule::configure( yarp::os::ResourceFinder &rf )
{
	moduleName            = rf.check("name", 
		Value("SimoxRobotViewer"), 
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
		close();
		return false;
	}
	robotDeviceLeftArm.view(encLeftArm);
	int axesLeftArm;
	if (!encLeftArm || !encLeftArm->getAxes(&axesLeftArm) || axesLeftArm<=0) {
		printf("Could not get encoder values from left_arm\n");
		close();
		return false;
	}
	jointValuesLeftArm.resize(axesLeftArm,0.0);


	// setup RIGHT ARM
	std::string localRightArmName = "/";
	localRightArmName += getName();
	localRightArmName += "/right_arm";
	Property optionsRightArm;
	optionsRightArm.put("device", "remote_controlboard");
	optionsRightArm.put("local", localRightArmName.c_str());      //local port names
	std::string remoteRightArm = ("/" + robotBase + "/right_arm");
	optionsRightArm.put("remote", remoteRightArm.c_str());         //where we connect to
	if (!robotDeviceRightArm.open(optionsRightArm)) {
		printf("Device not available.  Here are the known devices:\n");
		printf("%s", Drivers::factory().toString().c_str());
		close();
		return false;
	}	
	robotDeviceRightArm.view(encRightArm);
	int axesRightArm;
	if (!encRightArm || !encRightArm->getAxes(&axesRightArm) || axesRightArm<=0) {
		printf("Could not get encoder values from right_arm\n");
		close();
		return false;
	}
	jointValuesRightArm.resize(axesRightArm,0.0);



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


	rnsArmLeft = rf.find("RobotNodeSet_LeftArm").asString();
	rnsArmRight = rf.find("RobotNodeSet_RightArm").asString();
	rnsHandLeft = rf.find("RobotNodeSet_LeftHand").asString();
	rnsHandRight = rf.find("RobotNodeSet_RightHand").asString();
	rnsTorso = rf.find("RobotNodeSet_Torso").asString();
	rnsLegLeft = rf.find("RobotNodeSet_LeftLeg").asString();
	rnsLegRight = rf.find("RobotNodeSet_RightLeg").asString();
	rnsHead = rf.find("RobotNodeSet_Head").asString();

	viewer.reset(new SimoxRobotViewer("Simox iCub Viewer",rf));
	return true;
}


std::vector<float> SimoxRobotViewerModule::getFingerJoints(yarp::sig::Vector &enc)
{
	std::vector<float> jvHand;
	if (enc.size()!=16)
	{
		VR_ERROR << " need a 16 dim vector: " << enc.size() << endl;
		jvHand.resize(16,0.0f);
		return jvHand;
	}
	// fingers
	float hand_finger = (float)(enc[7]*M_PI/180.0);
	// strange values
	hand_finger *= 0.05f;
	float thumb_oppose = (float)(enc[8]*M_PI/180.0);
	float thumb_proximal = (float)(enc[9]*M_PI/180.0);
	float thumb_distal = (float)(enc[10]*M_PI/180.0);
	float index_proximal = (float)(enc[11]*M_PI/180.0);
	float index_distal = (float)(enc[12]*M_PI/180.0);
	float middle_proximal = (float)(enc[13]*M_PI/180.0);
	float middle_distal = (float)(enc[14]*M_PI/180.0);
	float pinky = (float)(enc[15]*M_PI/180.0);

	// 4*thumb
	jvHand.push_back(thumb_oppose);
	jvHand.push_back(thumb_proximal);
	jvHand.push_back(thumb_distal);
	jvHand.push_back(thumb_distal); // coupled with thumb_distal

	// 4*index
	jvHand.push_back(hand_finger);
	jvHand.push_back(index_proximal);
	jvHand.push_back(index_distal);
	jvHand.push_back(index_distal); // coupled with index_distal

	// 4*middle
	jvHand.push_back(hand_finger);
	jvHand.push_back(middle_proximal);
	jvHand.push_back(middle_distal);
	jvHand.push_back(middle_distal); // coupled with middle_distal

	// 4*ring
	jvHand.push_back(-hand_finger);
	jvHand.push_back(pinky); // coupled ring
	jvHand.push_back(pinky); // coupled ring
	jvHand.push_back(pinky); // coupled ring

	// 4*pinky
	jvHand.push_back(-hand_finger);
	jvHand.push_back(pinky); // coupled pinky
	jvHand.push_back(pinky); // coupled pinky
	jvHand.push_back(pinky); // coupled pinky

	return jvHand;
}

bool SimoxRobotViewerModule::updateModule()
{
	if (!viewer || viewer->wasClosed())
	{
		cout << "Viewer was closed, quitting..." << endl;
		return false;
	}
	// get joint values
	switch (visuMode)
	{
	case eShowCurrentRobotState:
	{
		double delay = Time::now() - lastEncoderUpdateS;
		if (delay> timeUpdateEncodersS)
		{
			lastEncoderUpdateS = Time::now();
	
			if (encLeftArm)
			{
				if (encLeftArm->getEncoders(jointValuesLeftArm.data()) && jointValuesLeftArm.size()==16)
				{
					std::vector<float> jv;
					// get first 7 values
					for (int i=0;i<7;i++)
						jv.push_back(jointValuesLeftArm[i]*M_PI/180.0);
					viewer->setJoints(rnsArmLeft,jv);
				}
				if (updateFingersLeft)
				{
					std::vector<float> jvHand = getFingerJoints(jointValuesLeftArm);
					viewer->setJoints(rnsHandLeft,jvHand);
				}
			}
			if (encRightArm)
			{
				if (encRightArm->getEncoders(jointValuesRightArm.data()) && jointValuesRightArm.size()==16)
				{
					std::vector<float> jv;
					// get first 7 values
					for (int i=0;i<7;i++)
						jv.push_back(jointValuesRightArm[i]*M_PI/180.0);
					viewer->setJoints(rnsArmRight,jv);
				}
				if (updateFingersRight)
				{
					std::vector<float> jvHand = getFingerJoints(jointValuesRightArm);
					viewer->setJoints(rnsHandRight,jvHand);
				}
			}
			if (encHead)
			{
				if (encHead->getEncoders(jointValuesHead.data()) && jointValuesHead.size()==6)
				{
					std::vector<float> jv;
					jv.push_back(-jointValuesHead[2]*M_PI/180.0);
					jv.push_back(jointValuesHead[0]*M_PI/180.0);
					jv.push_back(jointValuesHead[1]*M_PI/180.0);
					jv.push_back(jointValuesHead[3]*M_PI/180.0);
					jv.push_back(jointValuesHead[4]*M_PI/180.0); // ? test this
					jv.push_back(jointValuesHead[3]*M_PI/180.0);
					jv.push_back(jointValuesHead[4]*M_PI/180.0); // ? test this
				
					viewer->setJoints(rnsHead,jv);
				}
			}
			if (encTorso)
			{
				if (encTorso->getEncoders(jointValuesTorso.data()) && jointValuesTorso.size()==3)
				{
					std::vector<float> jv;
					jv.push_back(jointValuesTorso[2]*M_PI/180.0);// xchange yaw and pitch
					jv.push_back(jointValuesTorso[1]*M_PI/180.0);
					jv.push_back(jointValuesTorso[0]*M_PI/180.0);

					viewer->setJoints(rnsTorso,jv);
				}
			}
		}
	}
	break;
	case eShowStaticConfig:
		//do nothing
		break;
	default:
		break;
	}

	// send alive signal
	viewer->ping();

	// update gui
	if (qApp)
		qApp->processEvents();
	return true;
}

bool SimoxRobotViewerModule::respond( const Bottle& command, Bottle& reply )
{
	cout << "RESPONSE START" << endl;
	std::vector<std::string> helpMessages;

	helpMessages.push_back(string(getName().c_str()) + " commands are: \n" );
	helpMessages.push_back("help");
	helpMessages.push_back("quit");

	helpMessages.push_back("add object <name> <filename> ... load object from file <filename> and add to viewer. Chose <name> as you want, it will be key for further access.");
	helpMessages.push_back("remove object <name> ... remove object <name> from viewer");
	helpMessages.push_back("resetObjects ... remove all objects from viewer");
	helpMessages.push_back("set object position <name> x y z ... set object <name> to position (x,y,z) (global pose)");
	helpMessages.push_back("set object orientation <name> roll pitch yaw ... set RPY orientation of object <name> (global pose)");
	helpMessages.push_back("set object positionLocalCoord <name> <coordSystem> x y z ... set object <name> to position (x,y,z) (local coordinate system == RobotNode with name coordSystem)");
	helpMessages.push_back("set object orientationLocalCoord <name> <coordSystem> roll pitch yaw ... set RPY orientation of object <name> (local coordinate system == RobotNode with name coordSystem)");
	helpMessages.push_back("set eef <name> ... set current EEF (must be defined in robot's XML file)");
	helpMessages.push_back("set eefPreshape <name> ... set current EEF to preshape with name <name>. The preshape must be defined in the EEF definition at the robot's XML file.");
	helpMessages.push_back("set jointLimits <RNSname> lo_1 hi_1 lo_2 hi_2 ... [rad] set lower and upper joint limit for RobotNodeSet with name <RNSname> (must be defined in robot's XML file)");
	helpMessages.push_back("enable updateFingers left/right on/off ... Enable/Disable finger joint updates. When disabled the fingers can be set by setting the eef preshape");

	helpMessages.push_back("show reachability on/off ... show reachability data (if loaded)");
	helpMessages.push_back("show grasps <name> on/off ... show grasps that are stored for object <name> for the current EEF");
	helpMessages.push_back("show grasp <objname> on/off <graspname_1> ... <graspname_n>... show specific grasp(s) for the current EEF");
	helpMessages.push_back("show reachablegrasps <name> on/off ... show reachable (w.r.t. reachability data) grasps that are stored for object <name> for the current EEF");

	helpMessages.push_back("show robotState ... continuously show current state of robot (standard)");
	helpMessages.push_back("show config <RNS> j1 j2 .. jn ... [rad] disable robot updates, shows a static configuration. The config is built by setting the joints (j1 ... jn) of the RobotNodeSet RNS (must be defined in the robot's XML file) ");
	
	helpMessages.push_back("show motion <name> <RNS> on/off (c1_1 .. c1_n) (c2_1 .. c2_n) .. (ck_1 .. ck_n) ... Enable/Disable motion visualization. <name> A string to identify the motion visualization. <RNS>: the RobotNodeSet (with n joints) that is considered (must be defined in the robot's XML definition). c1 .. ck Joint configurations.");
	helpMessages.push_back("show vector <name> on/off x y z vx vy vz [scaling]... Enable/Disable vector visualization. <name> A string to identify the vector visualization. x y z The position. vx vy vz The orientation of the vector. Scaling factor is optional");
	helpMessages.push_back("show plane <name> on/off x y z nx ny nz ... Enable/Disable plane visualization. <name> A string to identify the plane visualization. x y z The position. nx ny nz The normal of the plane.");

	helpMessages.push_back("convertCoordinates <from> <to> x y z qx qy qz qw ... Convert coordinates from RobotNode <from> to RobotNode <to>. The current robot visualization is used! The strings <from> or <to> could be 'globalpose' for transforming poses from/to global coordinate system. Values are given in [meters]. Response will start with 1 or 0 indicating success/failure. On success the pose in <to> coord system follows as x y z qx qy qz qw [m].");

	helpMessages.push_back("saveScene filename ... save current scene to XML file. The scene is encoded in VirtualRobot's scene file format, which can be loaded with the tool SceneViewer. Robot, objects and paths are porcessed.");

	helpMessages.push_back("viewer show on/off ... show/hide viewer");
	helpMessages.push_back("viewer show on/off ... show/hide viewer");

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
	else if (command.get(0).asString()=="saveScene") {
		std::string filename = command.get(1).asString().c_str();
		responseOK = viewer->saveScene(filename,"RobotViewerScene");
		eaten = true;
	}
	else if (command.get(0).asString()=="resetObjects") {
		responseOK = viewer->resetObjects();
		eaten = true;
	}
	else if (command.get(0).asString()=="convertCoordinates") {

		std::string fromS = command.get(1).asString().c_str();
		std::string toS = command.get(2).asString().c_str();
		if (fromS == "globalpose")
			fromS = "";
		if (toS == "globalpose")
			toS = "";
		Eigen::Matrix4f m;
		MathTools::Quaternion q;
		q.x = (float)command.get(6).asDouble();
		q.y = (float)command.get(7).asDouble();
		q.z = (float)command.get(8).asDouble();
		q.w = (float)command.get(9).asDouble();
		m = MathTools::quat2eigen4f(q);
		m(0,3) = (float)command.get(3).asDouble() * 1000.0f;
		m(1,3) = (float)command.get(4).asDouble() * 1000.0f;
		m(2,3) = (float)command.get(5).asDouble() * 1000.0f;
		responseOK = false;

		if (viewer)
			responseOK = viewer->convertCoords(fromS,toS,m);

		if (responseOK)
		{
			customResponse = true;
			reply.addInt(1); // ok
			reply.addDouble((double)m(0,3)*0.001);
			reply.addDouble((double)m(1,3)*0.001);
			reply.addDouble((double)m(2,3)*0.001);
			q = MathTools::eigen4f2quat(m);
			reply.addDouble((double)q.x);
			reply.addDouble((double)q.y);
			reply.addDouble((double)q.z);
			reply.addDouble((double)q.w);

		} else
		{
			customResponse = true;
			reply.addInt(0); // failed
		}
		responseOK = true; // we already added reply values, ignore response later
		eaten = true;
	}
	else if (command.get(0).asString()=="viewer") 
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
	} else if (command.get(0).asString()=="enable") 
	{
		if (command.get(1).asString()=="updateFingers") 
		{
			std::string side = command.get(2).asString().c_str();
			std::string on = command.get(3).asString().c_str();
			bool leftB = (side == "left");
			bool onB = (on == "on");
			if (leftB)
			{
				updateFingersLeft = onB;
			} else
				updateFingersRight = onB;
			responseOK = true;
			eaten = true;	
		}
	}
	else if (command.get(0).asString()=="add") 
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
	} else if (command.get(0).asString()=="set") 
	{
		if (command.get(1).asString()=="object") 
		{
			if (command.get(2).asString()=="position" || command.get(2).asString()=="positionLocalCoord") 
			{
				ConstString name = command.get(3).asString();
				std::string coordSystem;
				int start = 4;
				if (command.get(2).asString()=="positionLocalCoord")
				{
					start++;
					coordSystem = command.get(4).asString().c_str();
				}
				double x = command.get(start).asDouble();
				double y = command.get(start+1).asDouble();
				double z = command.get(start+2).asDouble();
				responseStream << "Setting object <" << name << "> to position " << x << "," << y << "," << z;
				Eigen::Matrix4f p = viewer->getObjectPose(name.c_str(),coordSystem);
				p.block(0,3,3,1) = Eigen::Vector3f(x,y,z);
				responseOK = viewer->setObjectPose(name.c_str(),p,coordSystem);
				eaten = true;
			} else if (command.get(2).asString()=="orientation"  || command.get(2).asString()=="orientationLocalCoord") 
			{
				ConstString name = command.get(3).asString();
				std::string coordSystem;
				int start = 4;
				if (command.get(2).asString()=="orientationLocalCoord")
				{
					start++;
					coordSystem = command.get(4).asString().c_str();
				}
				double r = command.get(start).asDouble();
				double p = command.get(start+1).asDouble();
				double y = command.get(start+2).asDouble();

				responseStream << "Setting object <" << name << "> to RPY orientation " << p << "," << p << "," << y;

				Eigen::Matrix4f po = viewer->getObjectPose(name.c_str(),coordSystem);
				Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
				MathTools::rpy2eigen4f(r,p,y,m);
				m.block(0,3,3,1) = po.block(0,3,3,1);
				responseOK = viewer->setObjectPose(name.c_str(),m,coordSystem);
				eaten = true;
			}
		} else if (command.get(1).asString()=="eef") 
		{
			ConstString name = command.get(2).asString();
			responseOK = viewer->selectEEF(name.c_str());
			if (responseOK)
			{
				responseStream << "Setting current eef to ";
			} else
			{
				responseStream << "No EEF found with name ";
			}
			responseStream << name;
			eaten = true;
		}  else if (command.get(1).asString()=="eefPreshape") 
		{
			ConstString name = command.get(2).asString();
			responseOK = viewer->selectEEFPreshape(name.c_str());
			if (responseOK)
			{
				responseStream << "Setting current eef to ";
			} else
			{
				responseStream << "No EEF found with name ";
			}
			responseStream << name;
			eaten = true;
		} else if (command.get(1).asString()=="jointLimits") 
		{
			ConstString name = command.get(2).asString();
			int nrJoints = (command.size()-3) / 2;
			std::vector<float> lo,hi;
			for (int i=0;i<nrJoints;i++)
			{
				lo.push_back((float)command.get(3+i*2).asDouble());
				hi.push_back((float)command.get(4+i*2).asDouble());
			}
			responseOK = viewer->setJointLimits(name.c_str(),lo,hi);
			if (responseOK)
			{
				responseStream << "Setting joint limits OK ";
			} else
			{
				responseStream << "Setting joint limits FAILED ";
			}
			responseStream << name;
			eaten = true;
		}
	} else if (command.get(0).asString()=="show") 
	{
		if (command.get(1).asString()=="grasps" || command.get(1).asString()=="reachablegrasps") 
		{
			ConstString name = command.get(2).asString();
			bool on = true;
			if (command.get(3).asString()=="off")
			{
				responseStream << "Hiding grasps for object ";
				on = false;
			} else
			{
				responseStream << "Showing grasps for object ";
				on = true;
			}
			responseStream << name;
			bool allOn = on;
			bool reachbaleOn = false;
			if (command.get(1).asString()=="reachablegrasps")
			{
				allOn = false;
				reachbaleOn = on;
			}

			responseOK = viewer->showGrasps(name.c_str(),allOn,reachbaleOn);
			eaten = true;
		}else if (command.get(1).asString()=="grasp") 
		{
			ConstString name = command.get(2).asString();

			bool on = true;
			if (command.get(3).asString()=="off")
			{
				responseStream << "Hiding grasps for object ";
				on = false;
			} else
			{
				responseStream << "Showing grasps for object ";
				on = true;
			}
			responseStream << name;

			std::vector <std::string> gr;
			for (int i=4;i<command.size();i++)
			{
				ConstString graspname = command.get(i).asString();
				gr.push_back(graspname.c_str());
			}

			responseOK = viewer->showGrasp(name.c_str(),gr,on);
			eaten = true; 
		} else if (command.get(1).asString()=="reachability") 
		{
			bool on = true;
			if (command.get(2).asString()=="off")
			{
				responseStream << "Hiding reachability ";
				on = false;
			} else
			{
				responseStream << "Showing reachability ";
				on = true;
			}
			viewer->showReachability(on,Eigen::Vector3f(1.0f,0,0));
			responseOK = true;
			eaten = true;
		} else if (command.get(1).asString()=="robotState") 
		{
			visuMode = eShowCurrentRobotState;
			responseStream << "Showing robot state...";
			responseOK = true;
			eaten = true;
		} else if (command.get(1).asString()=="config") 
		{
			visuMode = eShowStaticConfig;
			std::string rns = command.get(2).asString().c_str();
			int joints = command.size()-3;
			std::vector<float> jv;
			for (int i=0;i<joints;i++)
				jv.push_back((float)command.get(3+i).asDouble());
			responseOK = viewer->setJoints(rns,jv);
			eaten = true;
		} else if (command.get(1).asString()=="vector" || command.get(1).asString()=="plane") 
		{
			std::string name = command.get(2).asString().c_str();
			bool on = true;
			if (command.get(3).asString()=="off")
				on = false;
			if (!on)
			{
				if (command.get(1).asString()=="vector")
					responseOK = viewer->removeVector(name);
				else
					responseOK = viewer->removePlane(name);
				eaten = true;
			} else
			{
				float scaling = 1.0f;
				Eigen::Vector3f x;
				x[0] = (float)command.get(4).asDouble();
				x[1] = (float)command.get(5).asDouble();
				x[2] = (float)command.get(6).asDouble();
				Eigen::Vector3f v;
				v[0] = (float)command.get(7).asDouble();
				v[1] = (float)command.get(8).asDouble();
				v[2] = (float)command.get(9).asDouble();
				if (command.size()>10)
					scaling = (float)command.get(10).asDouble();
				if (command.get(1).asString()=="vector")
				{
					cout << "showing vector " << endl;
					responseOK = viewer->showVector(name,x,v,scaling);
				} else
				{
					cout << "showing plane " << endl;
					responseOK = viewer->showPlane(name,x,v);
				}
				eaten = true;
			}
		} else if (command.get(1).asString()=="motion") 
		{
			std::string name = command.get(2).asString().c_str();
			std::string rns = command.get(3).asString().c_str();
			bool on = true;
			if (command.get(4).asString()=="off")
				on = false;
			if (!on)
			{
				responseOK = viewer->removePath(name);
				eaten = true;
			} else
			{
			
				std::vector< std::vector<float> > path;
				int pts = command.size()-5;
				cout << "getting " << pts << " points" << endl;
				if (pts>0)
				{
					int joints = command.get(5).asList()->size();
					cout << "Nr joints:" << joints << endl;
					if (joints>0)
					{
						for (int i=0;i<pts;i++)
						{
							yarp::os::Bottle *c = command.get(5+i).asList();
							if (c->size() != joints)
							{
								cout << "Skipping path point " << i << ". Invalid list size:" << c->size() << endl;
								continue;
							}
							std::vector<float> jv;
							for (int j=0;j<joints;j++)
								jv.push_back((float)c->get(j).asDouble());
							path.push_back(jv);
						}
						responseOK = viewer->showPath(name,path,rns);
						eaten = true;
					}
				}
			}
		}
	} 
	reply.addInt((int)responseOK);
	if (!eaten)
	{
		responseStream << "Unknown command: " << command.toString().c_str() << "\n Try 'help' \n";
		cout << helpMessage;
	}
	if (!customResponse)
		reply.addString(responseStream.str().c_str());
	cout << "reply: " << reply.toString() << endl;
	cout << "RESPONSE END" << endl;
	return true;
}

bool SimoxRobotViewerModule::interruptModule()
{
	handlerPort.interrupt();
	printf ("INTERRUPT\n");
	return true;
}

bool SimoxRobotViewerModule::close()
{
	if (viewer)
		viewer->quit();
	robotDeviceLeftArm.close();
	robotDeviceRightArm.close();
	robotDeviceHead.close();
	robotDeviceTorso.close();
	encRightArm = encLeftArm = encHead = encTorso = NULL;
	
	handlerPort.close();
	viewer.reset();

	// reports a memory leak in Coin's font manager?!
	//SoQt::done();

	return true;
}

double SimoxRobotViewerModule::getPeriod()
{
	// 50 fps
	return 0.02;
}
