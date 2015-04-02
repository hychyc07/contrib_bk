
#include "SimoxIkSolverModule.h"
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/CollisionDetection/CDManager.h>
#include <VirtualRobot/XML/ObjectIO.h>
#include <VirtualRobot/Grasping/GraspSet.h>
#include <yarp/math/Math.h>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::math;
using namespace std;
using namespace VirtualRobot;

SimoxIkSolverModule::SimoxIkSolverModule()
{
	if (!SoDB::isInitialized())
		SoDB::init();
	isLeft = true;
}


bool SimoxIkSolverModule::configure( yarp::os::ResourceFinder &rf )
{
	moduleName            = rf.check("name", 
		Value("SimoxIkSolver"), 
		"module name (string)").asString();
	setName(moduleName.c_str());

	robotBase = rf.check("robot", 
		Value("icubSim"), 
		"robot name (string)").asString();

	VR_INFO << "Using robot base string " << robotBase << endl;

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

	// rpc handler port
	handlerPortName =  "/";
	handlerPortName += getName();         // use getName() rather than a literal 
	handlerPortName += "/rpc:i";
	if (!handlerPort.open(handlerPortName.c_str())) {           
		cout << getName() << ": Unable to open port " << handlerPortName << endl;  
		return false;
	}

	attach(handlerPort);                  // attach to port

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

	if (robot && currentEEF && rf.check("ReachabilityFile"))
	{
		std::string reachFile = rf.find("ReachabilityFile").asString().c_str();
		std::string reachFileComplete = reachFile.c_str();
		if (!VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(reachFileComplete))
		{
			VR_ERROR << " Could not find file: " << reachFileComplete << endl;
		} else
			loadReachability(reachFileComplete);
	} 
	if (!reachSpace)
		VR_INFO << "Skipping reachability information..." << endl;

#ifdef EXTENDED_MANIPULABILITY_MEASURE
	if (robot && currentEEF && rf.check("ManipulabilityFile"))
	{
		std::string manipFile = rf.find("ManipulabilityFile").asString().c_str();
		std::string manipFileComplete = manipFile.c_str();
		if (!VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(manipFileComplete))
		{
			VR_ERROR << " Could not find file: " << manipFileComplete << endl;
		} else
			loadManipulability(manipFileComplete);
	} 
	if (!manipulability)
		VR_INFO << "Skipping manipulability information..." << endl;

	cout << "Using PoseQualityExtendedManipulability 2" << endl;
	qualityMeasure.reset(new PoseQualityExtendedManipulability(currentRNS,
		PoseQualityManipulability::eMinMaxRatio/*,
		PoseQualityExtendedManipulability::eScaledJacobianEntries,
		(float)(M_PI/4.0)*/));
#else
	cout << "Using PoseQualityManipulability with JL" << endl;
	float globalPenFactor = 50000.0f;
	qualityMeasure.reset(new PoseQualityManipulability(currentRNS));
	qualityMeasure->penalizeJointLimits(true,globalPenFactor);
#endif
	ikInitConfig.clear();
	Value *val;
	if (robot && currentRNS && rf.check("IkInitConfig",val))
	{
		//yarp::os::Bottle *jv = rf.find("IkInitConfig").asList();
		if (val->isList())
		{
			yarp::os::Bottle *jv = val->asList();
			if (jv->size() == currentRNS->getSize())
			{
				VR_INFO << "Joint config used as start for gradient based IK solving:";
				for (int i=0;i<jv->size();i++)
				{
					float v = (float)jv->get(i).asDouble();
					ikInitConfig.push_back(v);
					cout << v << ", ";
				}
				cout << endl;
			} else
			{
				VR_ERROR << "Wrong IkInitConfig joint value list size: jv->size() != currentRNS->getSize()" << endl;
			}
		}
	}

	// IK METHOD
	ikMethod  = SimoxGenericIkSolver;
	int ikSolverInt = rf.check("IkMethod",0,"The ik method: 0:Simox ik solver, 1: iCub CartesianControl ikSolver (module must be running!)").asInt();
	if (ikSolverInt == 1)
	{
		cout << "Using iCubCartesianIkSolver" << endl;
		ikMethod = iCubCartesianIkSolver;
	} else
		cout << "Using Simox Generic Ik Solver" << endl;

	// VISUALIZATION FLAGS
	yarp::os::Bottle visu = rf.findGroup("Visualization");
	bool enableVisu = false;
	if (!visu.isNull() && robot)
	{
		if (visu.check("EnableVisualization"))
		{
			std::string cdE = visu.find("EnableVisualization").asString().c_str();
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

	// COLLISION DETECTION FLAGS
	yarp::os::Bottle cd = rf.findGroup("CollisionDetection");

	cdEnvironment = false;
	cdEnabled = false;
	if (!cd.isNull() && robot)
	{
		if (cd.check("EnableCD"))
		{
			std::string cdE = cd.find("EnableCD").asString().c_str();
			if (cdE=="true" || cdE=="on" || cdE=="1")
				cdEnabled = true;			
		}
	}
	if (!cdEnabled)
	{
		VR_INFO << "Collision Detection is disabled..." << endl;
	} else
	{
		VR_INFO << "Collision Detection is ON..." << endl;

		if (cd.check("ConsiderEnvironment"))
		{
			std::string cdEnv = cd.find("ConsiderEnvironment").asString().c_str();
			if (cdEnv=="true" || cdEnv=="on" || cdEnv=="1")
				cdEnvironment = true;			
		}
		if (cdEnvironment)
			VR_INFO << "Considering environment for collision detection" << endl;
		else
			VR_INFO << "Disabling collision detection with environment" << endl;

		if (cd.check("RobotNodeSet_IkChain"))
		{
			std::string rnsCD_ikChainName = cd.find("RobotNodeSet_IkChain").asString().c_str();
			if (!rnsCD_ikChainName.empty())
			{
				if (robot->hasRobotNodeSet(rnsCD_ikChainName))
				{
					rnsCD_ikChain = robot->getRobotNodeSet(rnsCD_ikChainName);
				} else
					VR_ERROR << "Robot " << robot->getName() << " does not know RNS with name " << rnsCD_ikChainName << endl;
			}
		}
		if (!rnsCD_ikChain)
		{
			VR_INFO << "(Self-)Collision Detection is disabled." << endl;
		}

		if (cd.check("RobotNodeSet_Robot"))
		{
			std::string rnsCD_robotName = cd.find("RobotNodeSet_Robot").asString().c_str();
			if (!rnsCD_robotName.empty())
			{
				if (robot->hasRobotNodeSet(rnsCD_robotName))
				{
					rnsCD_robot = robot->getRobotNodeSet(rnsCD_robotName);
				} else
					VR_ERROR << "Robot " << robot->getName() << " does not know RNS with name " << rnsCD_robotName << endl;
			}
		}
		if (rnsCD_ikChain && !rnsCD_robot)
		{
			VR_INFO << "Self-Collision Detection is disabled due to missing RNS for remaining robot." << endl;
		}
	}

	setupIkSolver();


	return true;
}

bool SimoxIkSolverModule::setupViewer()
{
	SoQt::init("SimoxIkSolverModule");
	viewer.reset(new SimoxRobotViewer("Simox IK Solver Results"));
	if (robot)
	{
		viewer->setRobot(robot->clone("SimoxIKSolver_Visu"));
		viewer->showRobot(true,VirtualRobot::SceneObject::Full);
		// select eef in viewer
		if (currentEEF)
			viewer->selectEEF(currentEEF->getName());
		viewer->viewAll();
	}
	return true;
}

bool SimoxIkSolverModule::updateModule()
{

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

bool SimoxIkSolverModule::respond( const Bottle& command, Bottle& reply )
{
	std::vector<std::string> helpMessages;

	helpMessages.push_back(string(getName().c_str()) + " commands are: \n" );
	helpMessages.push_back("help");
	helpMessages.push_back("quit");
	helpMessages.push_back("info ... List information about internal state");
	helpMessages.push_back("add object <name> <filename> ... Load object from file <filename> and add to viewer. Chose <name> as you want, it will be key for further access.");
	helpMessages.push_back("remove object <name> ... Remove object <name> from viewer");
	helpMessages.push_back("set object position <name> x y z ... Set object <name> to position (x,y,z)");
	helpMessages.push_back("set object orientation <name> roll pitch yaw ... Set RPY orientation of object <name>");
	helpMessages.push_back("set eef <name> ... Set current EEF (must be defined in robot's XML file)");
	helpMessages.push_back("get reachablegrasps <name> ... Returns 0/1 followed by nr of reachable grasps followed by their names (as defined in manipulation object's <name> XML file) w.r.t. currentEEF, reachability data and the object <name>");
	helpMessages.push_back("get manipulabilityGrasps <name> ... Returns 0/1 followed by nr of reachable grasps followed by names (as defined in manipulation object's <name> XML file) and determined manipulability w.r.t. currentEEF, manipulability data and the object <name>");
	helpMessages.push_back("show reachablegrasps <name> on/off ... Enables/disables reachable grasps visualization w.r.t. currentEEF, reachability data and the object <name>");
	helpMessages.push_back("show grasp <name> <graspname> ... Shows grasp <graspname> visualization for object <name>");
	helpMessages.push_back("show coordsystem <jointname> on/off ... Enable/disable coordinate system visualization for joint <jointname>");
	helpMessages.push_back("get joints ... Returns 1 followed by <name of name of RNS>, followed by <number of joints> followed by all <joint names> of current kinematic chain (as defined in robot's XML file) ");
	helpMessages.push_back("get jointLimits ... Returns 1 followed by j0_lo j0_hi j1_lo j1_hi... [rad] (If mode==SimoxIkSolver: as defined in robot's XML file, if mode==iCubCartesianSolver: limits of iCubIkSolver) ");

	helpMessages.push_back("get ikSolution <name> ... Returns 0/1 as first value indicating the success of the Ik-solver call. On success the chosen grasp name followed by joint values of the ik solution w.r.t. currentEEF, reachability data and the object <name>. Additionally the manipulability is replied as last value.");
	helpMessages.push_back("get graspikSolution <name> <graspname> ... Returns 0/1 as first value indicating the success of the Ik-solver call. On success the joint values of the ik solution for grasp <graspname> are returned (using currenteEEF and object <name>). Additionally the manipulability is replied as last value.");
	helpMessages.push_back("get object position <name> <CoordinateSystem>... Returns 0/1 followed by object's position (x,y,z) in coordinate system of robots RobotNode with name <CoordinateSystem>. If RobotNode cannot be found or <CoordinateSystem> is not set, the global coord system is used.");
	helpMessages.push_back("get object orientation <name> <CoordinateSystem>... Returns 0/1 followed by object's orientation as three RPY values in coordinate system of robots RobotNode with name <CoordinateSystem>. If RobotNode cannot be found or <CoordinateSystem> is not set, the global coord system is used.");
	helpMessages.push_back("has reachability ... returns 1 if reachability data is loaded, otherwise 0");
	helpMessages.push_back("has manipulability ... returns 1 if manipulability data is loaded, otherwise 0");

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
	}
	else if (command.get(0).asString()=="has") 
	{
		if (command.get(1).asString()=="reachability") 
		{
			bool yes = hasReachability();
			customResponse = true;
			responseOK = true;
			if (yes)
				reply.addInt(1);
			else
				reply.addInt(0);
			commandProcessed = true;
		} else if (command.get(1).asString()=="manipulability") 
		{
			bool yes = hasManipulability();
			customResponse = true;
			responseOK = true;
			if (yes)
				reply.addInt(1);
			else
				reply.addInt(0);
			commandProcessed = true;
		}
	} else if (command.get(0).asString()=="add") 
	{
		if (command.get(1).asString()=="object") 
		{
			ConstString name = command.get(2).asString();
			ConstString filename = command.get(3).asString();
			responseStream << "Loading object <" << name << ">...";
			responseOK = setObject(name.c_str(),filename.c_str());
			if (responseOK)
				responseStream << "ok";
			else
				responseStream << "failed";
			commandProcessed = true;
		}
	} else if (command.get(0).asString()=="remove") 
	{
		if (command.get(1).asString()=="object") 
		{
			ConstString name = command.get(2).asString();

			setObject(name.c_str(),"");
			responseOK = true;
			responseStream << "Removing object <" << name << ">";
			commandProcessed = true;
		}
	} else if (command.get(0).asString()=="get") 
	{
		if (command.get(1).asString()=="reachablegrasps") 
		{
			ConstString name = command.get(2).asString();
			GraspSetPtr gs = getReachableGrasps(name.c_str());
			customResponse = true;
			responseOK = true;
			reply.addInt(1); // means OK
			if (gs)
			{
				cout << "Reachable grasps:" << gs->getSize() << endl;
				reply.addInt((int)gs->getSize()); // nr of grasps
				for (unsigned int i=0;i<gs->getSize();i++)
				{
					cout << gs->getGrasp(i)->getName() << ",";
					reply.addString( gs->getGrasp(i)->getName().c_str() );
				} 
				cout << endl;
			} else
			{
				reply.addInt(0); // 0 grasps
				cout << "No grasps reachable" << endl;
			}
			commandProcessed = true;
		}  else if (command.get(1).asString()=="manipulabilityGrasps") 
		{
			ConstString name = command.get(2).asString();
			std::vector<float> manip;
			std::vector<GraspPtr> grasps;
			bool ok = getManipulabilityGrasps(name.c_str(), manip, grasps);
			customResponse = true;
			responseOK = true;
			reply.addInt(1); // means OK
			if (ok && manip.size() == grasps.size() && manip.size()>0)
			{
				cout << "Manipulability: Nr of grasps:" << manip.size() << endl;
				reply.addInt((int)manip.size()); // nr of grasps
				for (int i=0;i<(int)manip.size();i++)
				{
					cout << grasps[i]->getName() << ": " << manip[i] << " ,";
					reply.addString( grasps[i]->getName().c_str() );
					reply.addDouble((double) manip[i] );
				} 
				cout << endl;
			} else
			{
				reply.addInt(0); // 0 grasps
				cout << "No grasps reachable" << endl;
			}
			commandProcessed = true;
		}  else if (command.get(1).asString()=="joints") 
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
				cout << "Name of RNS: " << currentRNS->getName() << endl;
				reply.addString(currentRNS->getName().c_str());
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
		} else if (command.get(1).asString()=="jointLimits") 
		{
			customResponse = true;
			responseOK = true;
			reply.addInt(1); // means OK
			if (!currentRNS)
			{
				cout << "No kinematic chain (RobotNodeSet) defined..." << endl;
			} else
			{
				for (unsigned int i=0;i<currentRNS->getSize();i++)
				{
					cout << i << ": " << currentRNS->getNode((int)i)->getJointLimitLo() << "," << currentRNS->getNode((int)i)->getJointLimitHi() << endl;
		
					reply.addDouble(currentRNS->getNode((int)i)->getJointLimitLo());
					reply.addDouble(currentRNS->getNode((int)i)->getJointLimitHi());
				}
			}
			commandProcessed = true;
		} else if (command.get(1).asString()=="ikSolution") 
		{
			ConstString name = command.get(2).asString();
			GraspPtr g = searchIK(name.c_str());
			customResponse = true;
			responseOK = true;

			if (!g || !currentRNS)
			{
				cout << "IK search failed..." << endl;
				reply.addInt(0); // failed
			} else
			{
				reply.addInt(1); // means OK
				//reply.addInt(1); // nr of solutions
				//reply.addInt(currentRNS->getSize()); // size of solution

				// first solution
				reply.addString(g->getName().c_str()); // name of grasp
				cout << "IK search succeeded. Selected grasp " << g->getName() << endl;
				cout << "Joint Values:" << endl;
				// add joint values
				for (unsigned int i=0; i<currentRNS->getSize();i++)
				{
					cout << currentRNS->getNode((int)i)->getJointValue() << ",";
					reply.addDouble((double)currentRNS->getNode((int)i)->getJointValue());
				}
				float manip = getManipulability();
				cout << "Manipulability: " << manip << endl;
				reply.addDouble((double)manip);
				cout << endl;
			}
			commandProcessed = true;
		} else if (command.get(1).asString()=="graspikSolution") 
		{
			ConstString name = command.get(2).asString();
			ConstString graspname = command.get(3).asString();
			bool ok = searchIKGrasp(name.c_str(),graspname.c_str());
			customResponse = true;
			responseOK = true;

			if (!ok || !currentRNS)
			{
				cout << "IK search failed..." << endl;
				reply.addInt(0); // failed
			} else
			{
				reply.addInt(1); // means OK

				//reply.addString(g->getName().c_str()); // name of grasp
				cout << "IK search succeeded." << endl;
				cout << "Joint Values:" << endl;
				// add joint values
				for (unsigned int i=0; i<currentRNS->getSize();i++)
				{
					cout << currentRNS->getNode((int)i)->getJointValue() << ",";
					reply.addDouble((double)currentRNS->getNode((int)i)->getJointValue());
				}
				float manip = getManipulability();
				cout << "Manipulability: " << manip << endl;
				reply.addDouble((double)manip);
				cout << endl;
			}
			commandProcessed = true;
		} else if (command.get(1).asString()=="object") 
		{
			ConstString what = command.get(2).asString();
			ConstString name = command.get(3).asString();
			ConstString coordsystem = command.get(4).asString();
			if (what=="position")
			{
				customResponse = true;
				responseOK = true;
				commandProcessed = true;
				Eigen::Matrix4f m = getObjectPose(name.c_str());
				Eigen::Vector3f pos = MathTools::getTranslation(m);
				reply.addInt(1); // means OK
				reply.addDouble(pos(0));
				reply.addDouble(pos(1));
				reply.addDouble(pos(2));

			} else if (what=="orientation")
			{
				customResponse = true;
				responseOK = true;
				commandProcessed = true;
				Eigen::Matrix4f m = getObjectPose(name.c_str());
				Eigen::Vector3f rpy;
				MathTools::eigen4f2rpy(m,rpy);
				reply.addInt(1); // means OK
				reply.addDouble(rpy(0));
				reply.addDouble(rpy(1));
				reply.addDouble(rpy(2));		
			}
		}
	} else if (command.get(0).asString()=="show") 
	{
		if (command.get(1).asString()=="reachablegrasps") 
		{
			ConstString name = command.get(2).asString();
			ConstString onOff = command.get(3).asString();
			bool enable = false;
			GraspSetPtr gs;
			if (onOff=="on")
			{
				enable = true;
				gs = getReachableGrasps(name.c_str());
				if (!gs || gs->getSize()==0)
					enable = false;
				responseStream << "Showing " << gs->getSize() << " reachable grasps";
			} else
				responseStream << "Hiding grasps...";
			if (gs)
				cout << "Reachable grasps:" << gs->getSize() << endl;
			else
				cout << "Reachable grasps: <none>" << endl;
			if (viewer)
				responseOK = viewer->showGrasps(name.c_str(),gs,enable);
			else
				responseOK = true;
			commandProcessed = true;
		} else if (command.get(1).asString()=="grasp") 
		{
			ConstString name = command.get(2).asString();
			ConstString graspname = command.get(3).asString();
			GraspPtr g = getGrasp(name.c_str(),graspname.c_str());
			if (!g)
			{
				cout << "No grasp with name " << graspname << " stored with object " << name << endl;
				responseStream << "Showing grasp <" << graspname << "> failed...";
				responseOK = false;
			} else
			{
				responseStream << "Showing grasp: " << g->getName();
				if (viewer)
					responseOK = viewer->showGrasp(name.c_str(),g,true);
				else
					responseOK = true;
				commandProcessed = true;
			}
		} else if (command.get(1).asString()=="coordsystem") 
		{
			ConstString name = command.get(2).asString();
			ConstString onOff = command.get(3).asString();
		
			responseStream <<"Showing coord system: " << name;
			bool showC = !(onOff == "off");
			reply.addInt((int)(!(onOff == "off")));
			if (viewer)
				responseOK = viewer->showCoordSystem(name.c_str(),showC);
			else
				responseOK = true;
			if (responseOK)
				responseStream << " ok";
			else
				responseStream << " failed";
			commandProcessed = true;
		}
	} else if (command.get(0).asString()=="set") 
	{
		if (command.get(1).asString()=="object") 
		{
			if (command.get(2).asString()=="position") 
			{
				ConstString name = command.get(3).asString();
				float x = (float)command.get(4).asDouble();
				float y = (float)command.get(5).asDouble();
				float z = (float)command.get(6).asDouble();
				responseStream << "Setting object <" << name << "> to position " << x << "," << y << "," << z;
				Eigen::Matrix4f p = getObjectPose(name.c_str());
				p.block(0,3,3,1) = Eigen::Vector3f(x,y,z);
				responseOK = setObjectPose(name.c_str(),p);
				commandProcessed = true;
			} else if (command.get(2).asString()=="orientation") 
			{
				ConstString name = command.get(3).asString();
				float r = (float)command.get(4).asDouble();
				float p = (float)command.get(5).asDouble();
				float y = (float)command.get(6).asDouble();

				responseStream << "Setting object <" << name << "> to RPY orientation " << r << "," << p << "," << y;

				Eigen::Matrix4f po = getObjectPose(name.c_str());
				Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
				MathTools::rpy2eigen4f(r,p,y,m);
				m.block(0,3,3,1) = po.block(0,3,3,1);
				responseOK = setObjectPose(name.c_str(),m);
				commandProcessed = true;
			}
		} else if (command.get(1).asString()=="eef") 
		{
			ConstString name = command.get(2).asString();
			responseOK = selectEEF(name.c_str());
			if (responseOK)
			{
				responseStream << "Setting current eef to ";
			} else
			{
				responseStream << "No EEF found with name ";
			}
			responseStream << name;
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

bool SimoxIkSolverModule::interruptModule()
{
	handlerPort.interrupt();
	printf ("INTERRUPT\n");
	return true;
}

bool SimoxIkSolverModule::close()
{	
	handlerPort.close();

	return true;
}

double SimoxIkSolverModule::getPeriod()
{
	// 50 fps
	return 0.02;
}



bool SimoxIkSolverModule::selectEEF(const std::string &eef, const std::string &eefPreshape)
{
	if (!selectEEF(eef) || !currentEEF)
	{
		VR_ERROR << "Error selecting eef, aborting" << endl;
		return false;
	}
	return currentEEF->setPreshape(eefPreshape);
	// select preshape in viewer
	if (viewer)
		viewer->selectEEFPreshape(eefPreshape);
}

bool SimoxIkSolverModule::selectEEF(const std::string &eef)
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

bool SimoxIkSolverModule::selectRNS(const std::string &rns)
{
	if (!robot || !robot->hasRobotNodeSet(rns))
	{
		VR_ERROR << "Robot does not have RNS with name " << rns << endl;
		return false;
	}
	currentRNS = robot->getRobotNodeSet(rns);
	if (!currentRNS->isKinematicChain())
	{
		VR_WARNING << "RNS " << rns << " is not a strictly defined kinematic chain!" << endl;
	}
	// we have to re-setup the ik solver
	if (ikSolver)
		setupIkSolver();

	return true;
}

bool SimoxIkSolverModule::loadReachability( const std::string &filename )
{
	if (!robot)
		return false;
	VR_INFO << "Loading reachability data from " << filename << endl;

	reachSpace.reset(new Reachability(robot));
	try
	{
		reachSpace->load(filename);
	}
	catch (VirtualRobotException &e)
	{
		VR_ERROR << " Could not load reachability space" << endl;
		VR_ERROR << e.what();
		reachSpace.reset();
		return false;
	}
	reachSpace->print();
	if (reachSpace->getNodeSet())
	{
		VR_INFO << "Using RNS: " << reachSpace->getNodeSet()->getName() << endl;
		if (!selectRNS(reachSpace->getNodeSet()->getName()))
		{
			VR_ERROR << "Could not select ReachSpace RNS: " << reachSpace->getNodeSet()->getName() << endl;
			reachSpace.reset();
			return false;
		}
		if (!currentRNS || !currentRNS->getTCP())
		{
			VR_ERROR << "Could not select ReachSpace RNS (or TCP not present): " << reachSpace->getNodeSet()->getName() << endl;
			reachSpace.reset();
			return false;
		}

		if (currentRNS->getTCP()->getName() != currentEEF->getTcpName())
		{
			VR_ERROR << "Expecting reachability space's tcp (" << currentRNS->getTCP()->getName() << ") == currentEEF's tcp (" << currentEEF->getTcpName() << ")" << endl;
			reachSpace.reset();
			return false;
		}
	}
	if (!reachSpace)
	{
		VR_ERROR << " ERROR while creating reachability data" << endl;
		return false;
	}

	// we have to re-setup the ik solver
	if (ikSolver)
		setupIkSolver();

	return true;
}

bool SimoxIkSolverModule::loadManipulability( const std::string &filename )
{
	if (!robot)
		return false;
#ifdef EXTENDED_MANIPULABILITY_MEASURE
	VR_INFO << "Loading manipulability data from " << filename << endl;

	manipulability.reset(new Manipulability(robot));
	try
	{
		manipulability->load(filename);
	}
	catch (VirtualRobotException &e)
	{
		VR_ERROR << " Could not load manipulability space" << endl;
		VR_ERROR << e.what();
		manipulability.reset();
		return false;
	}
	manipulability->print();
	if (manipulability->getNodeSet())
	{
		VR_INFO << "Using RNS: " << manipulability->getNodeSet()->getName() << endl;
		if (manipulability->getNodeSet()->getName() != currentRNS->getName())
		{
			VR_ERROR << "Wrong RNS: " << manipulability->getNodeSet()->getName() << "!=" << currentRNS->getName() << endl;
			manipulability.reset();
			return false;
		}
	}
	if (!manipulability)
	{
		VR_ERROR << " ERROR while creating manipulability data" << endl;
		return false;
	}

	return true;
#else
	cout << "MANIPULABILITY CHECK DISABLED..." << endl;
	return false;
#endif
}


bool SimoxIkSolverModule::loadRobot( const std::string &filename )
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

void SimoxIkSolverModule::print()
{
	cout << "***** SimoxIkSolverModule *****" << endl;
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

	cout << "End Effector: ";
	if (currentEEF)
		cout << currentEEF->getName();
	else
		cout << "<none>";
	cout << endl;

	cout << "Reachability Data: ";
	if (reachSpace)
		reachSpace->print();
	else
		cout << "<none>";
	cout << endl;

	cout << "IK Method: ";
	switch (ikMethod)
	{
		case SimoxGenericIkSolver: 
			cout << "SimoxGenericIkSolver" << endl;
			break;
		case iCubCartesianIkSolver: 
			cout << "iCubCartesianIkSolver" << endl;
			break;
		default:
			cout << "nyi..." << endl;
	}
	cout << endl;

	cout << "Objects:" << endl;
	std::map< std::string, VirtualRobot::ManipulationObjectPtr >::iterator it = objects.begin();
	while (it!=objects.end())
	{
		cout << "** Object: " << endl;
		it->second->print(false);
		cout << endl;
		it++;
	}
}

bool SimoxIkSolverModule::setupIkSolver()
{
	if (!robot || !currentRNS)
		return false;

	// setup Collision detection
	if (cdEnabled)
	{
		cdm.reset(new CDManager());
		if (cdEnvironment && rnsCD_ikChain)
		{
			std::map< std::string, VirtualRobot::ManipulationObjectPtr >::iterator it = objects.begin();
			while (it!=objects.end())
			{

				cdm->addCollisionModelPair(it->second,rnsCD_ikChain);
				it++;
			}
		}
		if (rnsCD_ikChain && rnsCD_robot)
			cdm->addCollisionModelPair(rnsCD_ikChain, rnsCD_robot);
	}

	switch (ikMethod)
	{
		case SimoxGenericIkSolver:
			ikSolver.reset(new GenericIKSolver(currentRNS));

			// setup reachability data
			if (reachSpace)
				ikSolver->setReachabilityCheck(reachSpace);

			// set collision detection
			if (cdEnabled && cdm)
				ikSolver->collisionDetection(cdm);

			ikSolver->setMaximumError(5.0f,0.05f);
			ikSolver->setupJacobian(0.9f,20);
			return true;
			break;
		case iCubCartesianIkSolver:
		{


			Property option("(device cartesiancontrollerclient)");
			std::string remoteStr = "/" + robotBase;
			std::string localStr = "/";
			localStr += getName();
			localStr += "/cartesian_client";
			if (isLeft)
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
			if (!iCubIkSolverClient.open(option))
				return false;

			// open the view
			iCubIkSolverClient.view(iCubIkSolverControl);
			if (!iCubIkSolverControl)
				return false;

			iCubIkSolverControl->setTrajTime(1.0);
			iCubIkSolverControl->setInTargetTol(1e-5);

			// get the torso dofs
			Vector newDof, curDof;
			iCubIkSolverControl->getDOF(curDof);
			newDof=curDof;

			// enable the torso yaw and pitch
			newDof[0]=1;
			newDof[1]=1;//0;// todo: why? disable the torso roll
			newDof[2]=1;
			// send the request for dofs reconfiguration
			iCubIkSolverControl->setDOF(newDof,curDof);

			// setup joint limits
			double minJV, maxJV;
			//cout << "Setting joint limits of iCubCartesianSolver" << endl;
			cout << "Setting joint limits of simox iCub model w.r.t. iCubCartesianSolver limits" << endl;
			std::streamsize pr = std::cout.precision(2);
			if (currentRNS)
			{
				for (unsigned int i=0;i<currentRNS->getSize();i++)
				{
					iCubIkSolverControl->getLimits(i,&minJV,&maxJV);

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
						/*
						cout << "Changing joint limits of joint " << rn->getName() << " from (" << minJV << "," << maxJV << ") to (" << lo << "," << hi << ") [degree]" << endl;
						cout << "before set lo: " << lo << ", hi: " << hi << endl;
						bool setOK = iCubIkSolverControl->setLimits(i,lo,hi);
						cout << "Set joint limits ";
						if (setOK)
							cout << "ok" << endl;
						else
							cout << "failed!!!" << endl;
						iCubIkSolverControl->getLimits(i,&minJV,&maxJV);
						cout << "after set lo: " << minJV << ", hi: " << maxJV << endl;*/
					}	
				}
			}
			std::cout.precision(pr);

			/*
			// impose some restriction on the torso pitch
			int axis=0; // pitch joint
			double min, max;

			// sometimes it may be helpful to reduce
			// the range of variability of the joints;
			// for example here we don't want the torso
			// to lean out more than 30 degrees forward

			// we keep the lower limit
			iCubIkSolverControl->getLimits(axis,&min,&max);
			iCubIkSolverControl->setLimits(axis,min,MAX_TORSO_PITCH);
			*/
			
			return true;
		}
			break;
		default:
			cout << "ikMethod nyi..." << endl;
	}
	// if here->failure
	return false;
}

VirtualRobot::GraspPtr SimoxIkSolverModule::searchIK(const std::string &objectName)
{
	if (objects.find(objectName) == objects.end())
	{
		VR_INFO << "Do not know object " << objectName << endl;
		return GraspPtr();
	}
	return searchIK(objects[objectName]);
}

bool SimoxIkSolverModule::searchIKGrasp(const std::string &objectName, const std::string &graspName)
{
	if (objects.find(objectName) == objects.end())
	{
		VR_INFO << "Do not know object " << objectName << endl;
		return false;
	}
	ManipulationObjectPtr o = objects[objectName];
	if (!o)
	{
		VR_INFO << "?!Do not know object " << objectName << endl;
		return false;
	}
	GraspSetPtr gs = o->getGraspSet(currentEEF);
	if (!gs)
	{
		VR_INFO << "Could not get grasp set for EEF " << currentEEF->getName() << " and object " << objectName << endl;
		return false;
	}
	if (!gs->hasGrasp(graspName))
	{
		VR_INFO << "Grasp set " << gs->getName() << " does not know Grasp " << graspName << " for object " << objectName << endl;
		return false;
	}
	GraspPtr g = gs->getGrasp(graspName);
	return searchIKGrasp(objects[objectName],g);
}


VirtualRobot::GraspPtr SimoxIkSolverModule::searchIK(VirtualRobot::ManipulationObjectPtr object)
{
	switch (ikMethod)
	{
	case SimoxGenericIkSolver:
		return searchIKSimox(object);
		break;
	case iCubCartesianIkSolver:
		return searchIKiCubModule(object);
		break;
	default:
		cout << "ikMethod nyi..." << endl;
	}
	return GraspPtr();
}

bool SimoxIkSolverModule::searchIKGrasp(VirtualRobot::ManipulationObjectPtr object, VirtualRobot::GraspPtr g)
{
	std::vector<float> jV;
	switch (ikMethod)
	{
	case SimoxGenericIkSolver:
		return searchGraspIKSimox(object,g,jV);
		break;
	case iCubCartesianIkSolver:
		return searchGraspIKiCubModule(object,g,jV);
		break;
	default:
		cout << "ikMethod nyi..." << endl;
	}
	return false;
}



bool SimoxIkSolverModule::setObject( const std::string &objectName, std::string filename )
{
	if (objects.find(objectName) != objects.end())
	{
		VR_INFO << "removing object " << objectName << endl;

		objects.erase(objectName);
	}
	if (filename=="")
	{
		if (viewer)
			viewer->removeObject(objectName);
		return false;
	}
	if (!VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filename))
	{
		VR_WARNING << "Could not find file: " << filename;
		return false;
	} 

	ManipulationObjectPtr o = ObjectIO::loadManipulationObject(filename);
	if (!o)
	{
		VR_WARNING << "Could not load object from file: " << filename;
		return false;
	}

	objects[objectName] = o;
	if (viewer)
		viewer->setObject(objectName,o->clone(o->getName()));
	updateViewer(objectName);

	return true;
}


bool SimoxIkSolverModule::setObjectPose( const std::string &objectName, Eigen::Matrix4f &pose )
{
	if (objects.find(objectName) == objects.end())
	{
		VR_INFO << "Do not know object " << objectName << endl;
		return false;
	}
	objects[objectName]->setGlobalPose(pose);
	updateViewer(objectName);

	return true;
}

void SimoxIkSolverModule::updateViewer( const std::string &objectName )
{
	if (objects.find(objectName) == objects.end())
	{
		VR_INFO << "Do not know object " << objectName << endl;
		return;
	}
	Eigen::Matrix4f pose = objects[objectName]->getGlobalPose();
	
	if (viewer)
		viewer->setObjectPose(objectName,pose);
}

Eigen::Matrix4f SimoxIkSolverModule::getObjectPose( const std::string &objectName )
{
	if (objects.find(objectName) == objects.end())
	{
		VR_INFO << "Do not know object " << objectName << endl;
		return Eigen::Matrix4f::Identity();
	}

	return objects[objectName]->getGlobalPose();
}

VirtualRobot::GraspPtr SimoxIkSolverModule::getGrasp( const std::string &objectName, const std::string &graspName )
{
	if (objects.find(objectName) == objects.end())
	{
		VR_INFO << "Do not know object " << objectName << endl;
		return GraspPtr();
	}
	GraspSetPtr grasps = objects[objectName]->getGraspSet(currentEEF);
	if (!grasps)
	{
		VR_INFO << "No grasps for object " << objectName << endl;
		return GraspPtr();
	}
	for (unsigned int i=0;i<grasps->getSize();i++)
	{
		if (grasps->getGrasp(i)->getName() == graspName)
			return grasps->getGrasp(i);
	}

	VR_INFO << "No grasp with name " << graspName << " found for object " << objectName << endl;
	return GraspPtr();
}

VirtualRobot::GraspSetPtr SimoxIkSolverModule::getReachableGrasps( const std::string &objectName )
{
	if (objects.find(objectName) == objects.end())
	{
		VR_INFO << "Do not know object " << objectName << endl;
		return GraspSetPtr();
	}
	GraspSetPtr grasps = objects[objectName]->getGraspSet(currentEEF);

	if (!grasps)
	{
		VR_ERROR << "No grasps stored for object " << objectName << " !!" << endl;
		return grasps;
	}	
	if (!reachSpace)
	{
		VR_ERROR << "No reach space, returning all grasps!!" << endl;
		return grasps;
	}


	GraspSetPtr rg = reachSpace->getReachableGrasps(grasps,objects[objectName]);
	return rg;
}

bool SimoxIkSolverModule::searchGraspIKiCubModule( VirtualRobot::ManipulationObjectPtr object, VirtualRobot::GraspPtr g, std::vector<float> &storeJointValues )
{
	if (!robot || !object || !g)
		return false;

	std::string robotRootNode("iCubRoot");
	float maxPosDiff = 0.01f; // im [m]
	float maxOriDiff = 0.1f;
	cout << "Checking for max error: translation:" << maxPosDiff << "[m] / orientation:" << maxOriDiff << " [rad]" << endl;
	bool verbose = true;


	// get iCubs root frame
	RobotNodePtr rootiCub = robot->getRobotNode(robotRootNode);
	if (!rootiCub)
	{
		VR_ERROR << "No robot node with name " << robotRootNode << " found... Aborting..." << endl;
		return false;
	}

	// query iCub ik solver
	Vector xd,od,resX,resO,resQ;
	xd.resize(3);
	od.resize(4);
	Eigen::Vector3f axis;
	Eigen::Vector3f pos;
	float angle;

	Eigen::Matrix4f mTcp = g->getTcpPoseGlobal(object->getGlobalPose());
	mTcp = rootiCub->toLocalCoordinateSystem(mTcp);
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
		cout << "IkRequest\nPose: "<< xd(0) << "," << xd(1) << "," << xd(2) << endl;
		cout << "Ori: "<< od(0) << "," << od(1) << "," << od(2) << "," << od(3) << endl;
		
		Vector x0,o0;
		if (iCubIkSolverControl->getPose(x0,o0))
		{
			cout << "CurrentPos:" << x0(0) << "," << x0(1) << "," << x0(2) << endl;
			cout << "CurrentOri:" << o0(0) << "," << o0(1) << "," << o0(2) << "," << o0(3) << endl;
		} else
			cout <<" Could not get current pose!!!" << endl;
	}
	bool askOK = iCubIkSolverControl->askForPose(xd,od,resX,resO,resQ); 
	if (!askOK)
	{
		VR_ERROR << "iCubIkSolverControl->askForPose failed..." << endl;
		return false;
	}
	float distPos = sqrtf(yarp::math::dot(resX-xd,resX-xd));
	MathTools::Quaternion q1 = MathTools::axisangle2quat(Eigen::Vector3f(resO[0],resO[1],resO[2]),resO[3]);
	MathTools::Quaternion q2 = MathTools::axisangle2quat(axis,angle);
	MathTools::Quaternion deltaQ = MathTools::getDelta(q1,q2);
	float distOri = MathTools::getAngle(deltaQ);
	//sqrtf(yarp::math::dot(resO-od,resO-od));
	if (verbose)
	{
		cout << "Result: " << endl;
		cout << "norm (pos-xd) [m]:" << distPos << endl;
		cout << "norm (ori-od) [rad]:" << distOri << endl;
		cout << "resX:" << resX(0) << "," << resX(1) << "," << resX(2) << endl;
		cout << "resO:" << resO(0) << "," << resO(1) << "," << resO(2) << "," << resO(3) << endl;
		cout << "resQ:";
		for (size_t i=0;i<resQ.size();i++)
			cout << resQ(i) << ",";
		cout << endl;
	}
	if (fabs(distPos)<=maxPosDiff && fabs(distOri)<=maxOriDiff)
	{
		cout << "IK solution OK" << endl;
		//std::vector<float> jV;
		double minJV, maxJV;
		if (resQ.size() != 10)
		{
			VR_ERROR << "Expecting 10 dof result?!" << endl;
			return false;
		} else
		{
			// checking joint limits 
			storeJointValues.clear();
			for (size_t i=0;i<resQ.size();i++)
			{
				storeJointValues.push_back(resQ[i]*(float)M_PI/180.0f);
				iCubIkSolverControl->getLimits(i,&minJV,&maxJV);
				if (resQ[i]<minJV || resQ[i]>maxJV)
				{
					cout << "Joint limits violated (iCubIkSolverControl): joint " << i << ", min:" << minJV << ", max:" << maxJV << ", joint value:" << resQ[i] << endl;
				}
			}

			bool jointLimitsOK = currentRNS->checkJointLimits(storeJointValues,true);
			if (!jointLimitsOK)
				cout << "Joint limits violated (Simox)" << endl;
			currentRNS->setJointValues(storeJointValues);

			// set visualization in viewer
			if (viewer)
				viewer->setJoints(currentRNS->getName(),storeJointValues);

			Eigen::Matrix4f tcp_root = currentEEF->getTcp()->getGlobalPose();
			tcp_root = rootiCub->toLocalCoordinateSystem(tcp_root);
			float errX = tcp_root(0,3) - xd[0]*1000.0f;
			float errY = tcp_root(1,3) - xd[1]*1000.0f;
			float errZ = tcp_root(2,3) - xd[2]*1000.0f;
			float e = sqrtf(errX*errX + errY*errY + errZ*errZ);
			if (e>50.0f)
			{
				cout << "ERROR (TCP of model to requested position) [mm]:" << e << endl;
				cout << "By observation, I recognized that the ik solver sometimes reports correct x and o values, but serves the current joint config instead of the calculated one. Therefore I added this check which now failed..." << endl;
				cout << "TCP pose (root)" << endl;
				cout << tcp_root << endl;
				return false;
			}	
		}

		return true;
	}

	return false;
}

VirtualRobot::GraspPtr SimoxIkSolverModule::searchIKiCubModule( VirtualRobot::ManipulationObjectPtr object )
{
	if (!robot || !reachSpace || !iCubIkSolverControl || !object)
	{
		VR_ERROR << "Not initialized correctly..." << endl;
		return GraspPtr();
	}

	bool verbose = true;

	// check for reachable grasp
	GraspSetPtr grasps = object->getGraspSet(currentEEF);
	GraspSetPtr gs = reachSpace->getReachableGrasps(grasps,object);
	if (!gs || gs->getSize()==0)
	{
		VR_WARNING << "Did not determine any reachable grasps..." << endl;
		return GraspPtr();
	}
	VR_INFO << "Nr of reachable grasps:" << gs->getSize();

	std::vector<float> jV;
	while (gs->getSize()>0)
	{
		// chose grasp randomly (todo: better solution?!)
		unsigned int nr = rand() % gs->getSize();
		GraspPtr g = gs->getGrasp(nr);
		if (!g)
		{
			VR_ERROR << "Internal error, aborting IK search..." << endl;
			return GraspPtr();
		}
		if (verbose)
		{
			cout << "Checking grasp " << nr << ":" << g->getName() << endl;
		}

		bool resOK = searchGraspIKiCubModule(object, g, jV );
		if (resOK)
		{
			return g;
		}

		gs->removeGrasp(g);
	}

	VR_INFO << "IK search failed..." << endl;
	return GraspPtr();
}

VirtualRobot::GraspPtr SimoxIkSolverModule::searchIKSimox(VirtualRobot::ManipulationObjectPtr object)
{
	if (!robot || !currentRNS || !ikSolver)
	{
		VR_ERROR << "Not initialized correctly..." << endl;
		return GraspPtr();
	}
	if (ikInitConfig.size() == currentRNS->getSize())
	{
		currentRNS->setJointValues(ikInitConfig);
	}
	GraspPtr grasp = ikSolver->solve(object,IKSolver::All,10);
	if (grasp)
	{
		VR_INFO << "IK successful..." << endl;
		std::vector<float> jV;
		currentRNS->getJointValues(jV);
		// set visualization in viewer
		if (viewer)
			viewer->setJoints(currentRNS->getName(),jV);
	} else
	{
		VR_INFO << "IK failed..." << endl;
	}
	return grasp;
}

bool SimoxIkSolverModule::searchGraspIKSimox(VirtualRobot::ManipulationObjectPtr object, VirtualRobot::GraspPtr g, std::vector<float> &storeJointValues)
{
	if (!robot || !currentRNS || !ikSolver || !g || !object)
	{
		VR_ERROR << "Not initialized correctly..." << endl;
		return GraspPtr();
	}
	if (ikInitConfig.size() == currentRNS->getSize())
	{
		currentRNS->setJointValues(ikInitConfig);
	}
	bool res = ikSolver->solve(object,g);
	if (res)
	{
		VR_INFO << "IK successful..." << endl;
		currentRNS->getJointValues(storeJointValues);
		// set visualization in viewer
		if (viewer)
			viewer->setJoints(currentRNS->getName(),storeJointValues);
	} else
	{
		VR_INFO << "IK failed..." << endl;
	}
	return res;
}

bool SimoxIkSolverModule::getManipulabilityGrasps( const std::string &objectName, std::vector<float> &manip, std::vector<VirtualRobot::GraspPtr> &grasps )
{
#ifdef EXTENDED_MANIPULABILITY_MEASURE
	if (!manipulability)
		return false;

	if (objects.find(objectName) == objects.end())
	{
		VR_INFO << "Do not know object " << objectName << endl;
		return false;
	}
	GraspSetPtr gr = objects[objectName]->getGraspSet(currentEEF);

	if (!gr)
	{
		VR_ERROR << "No grasps stored for object " << objectName << " !!" << endl;
		return false;
	}	
	std::vector< Manipulability::ManipulabiliyGrasp > res = manipulability->analyseGrasps(gr,objects[objectName]);
	grasps.clear();
	manip.clear();
	for (size_t i = 0;i<res.size();i++)
	{
		grasps.push_back(res[i].grasp);
		manip.push_back(res[i].manipulability);
	}
	return true;

#else
	cout << "MANIPULABILITY CHECK DISABLED..." << endl;
	return false;
#endif
}

bool SimoxIkSolverModule::hasReachability()
{
	if (reachSpace)
		return true;
	else
		return false;
}

bool SimoxIkSolverModule::hasManipulability()
{
#ifdef EXTENDED_MANIPULABILITY_MEASURE
	if (manipulability)
		return true;
#endif
	return false;
}

float SimoxIkSolverModule::getManipulability()
{
	if (!qualityMeasure)
		return 0.0f;

	return qualityMeasure->getPoseQuality();
}

