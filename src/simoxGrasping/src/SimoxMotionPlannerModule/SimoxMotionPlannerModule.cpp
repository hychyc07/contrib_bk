
#include "SimoxMotionPlannerModule.h"
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/CollisionDetection/CDManager.h>
#include <VirtualRobot/XML/ObjectIO.h>
#include <MotionPlanning/CSpace/CSpaceSampled.h>
#include <MotionPlanning/Saba.h>
#include <MotionPlanning/Planner/BiRrt.h>
#include <MotionPlanning/PostProcessing/ShortcutProcessor.h>
#include "iCubConfigurationConstraint.h"
#include <yarp/math/Math.h>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::math;
using namespace std;
using namespace VirtualRobot;

SimoxMotionPlannerModule::SimoxMotionPlannerModule()
{
	if (!SoDB::isInitialized())
		SoDB::init();
}


bool SimoxMotionPlannerModule::configure( yarp::os::ResourceFinder &rf )
{
	moduleName            = rf.check("name", 
		Value("SimoxMotionPlanner"), 
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

	pathOptimizingSteps = rf.check("PostProcessingSteps",yarp::os::Value(200),"Post Processing Steps, 0 to disable (Int)").asInt();
	
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

	// COLLISION DETECTION FLAGS
	yarp::os::Bottle cd = rf.findGroup("CollisionDetection");

	cdEnvironment = false;
	cdEnabled = false;
	searchColFreeObjectPose = false;
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

		if (cd.check("RobotNodeSet_robotMove"))
		{
			std::string rnsCD_robotMoveName = cd.find("RobotNodeSet_robotMove").asString().c_str();
			if (!rnsCD_robotMoveName.empty())
			{
				if (robot->hasRobotNodeSet(rnsCD_robotMoveName))
				{
					rnsCD_robotMove = robot->getRobotNodeSet(rnsCD_robotMoveName);
				} else
					VR_ERROR << "Robot " << robot->getName() << " does not know RNS with name " << rnsCD_robotMoveName << endl;
			}
		}
		if (!rnsCD_robotMove)
		{
			VR_INFO << "(Self-)Collision Detection is disabled." << endl;
		}

		if (cd.check("RobotNodeSet_robotStatic"))
		{
			std::string rnsCD_robotStaticName = cd.find("RobotNodeSet_robotStatic").asString().c_str();
			if (!rnsCD_robotStaticName.empty())
			{
				if (robot->hasRobotNodeSet(rnsCD_robotStaticName))
				{
					rnsCD_robotStatic = robot->getRobotNodeSet(rnsCD_robotStaticName);
				} else
					VR_ERROR << "Robot " << robot->getName() << " does not know RNS with name " << rnsCD_robotStaticName << endl;
			}
		}
		if (rnsCD_robotMove && !rnsCD_robotStatic)
		{
			VR_INFO << "Self-Collision Detection is disabled due to missing RNS for remaining robot." << endl;
		}

		std::string moveObjectPose = cd.find("SearchColFreeObjectPose").asString().c_str();
		if (moveObjectPose=="true" || moveObjectPose=="on" || moveObjectPose=="1")
		{
			cout << "Enabling search for collision free target object pose" << endl;
			searchColFreeObjectPose = true;
		}

	}

	return true;
}

bool SimoxMotionPlannerModule::setupViewer()
{
	SoQt::init("SimoxMotionPlannerModule");
	viewer.reset(new SimoxRobotViewer("Simox Motion Planner Results"));
	if (robot)
	{
		viewer->setRobot(robot->clone("SimoxMotionPlanner_Visu"));
		viewer->showRobot(true,VirtualRobot::SceneObject::Collision);
		viewer->viewAll();
	}
	return true;
}

bool SimoxMotionPlannerModule::updateModule()
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

bool SimoxMotionPlannerModule::respond( const Bottle& command, Bottle& reply )
{
	std::vector<std::string> helpMessages;

	helpMessages.push_back(string(getName().c_str()) + " commands are: \n" );
	helpMessages.push_back("help");
	helpMessages.push_back("quit");
	helpMessages.push_back("info ... list information about internal state");

	helpMessages.push_back("add object <name> <filename> ... load object from file <filename> and add to viewer. Chose <name> as you want, it will be key for further access.");
	helpMessages.push_back("remove object <name> ... remove object <name> from viewer");
	helpMessages.push_back("set object position <name> x y z ... set object <name> to position (x,y,z)");
	helpMessages.push_back("set object orientation <name> roll pitch yaw ... set RPY orientation of object <name>");
	
	helpMessages.push_back("set jointLimits <RNSname> lo_1 hi_1 lo_2 hi_2 ... [rad] set lower and upper joint limit for RobotNodeSet with name <RNSname> (must be defined in robot's XML file)");
	helpMessages.push_back("show coordsystem <jointname> on/off ... enable/disable coordinate system visualization for joint <jointname>");
	
	helpMessages.push_back("get joints ... returns number of joints followed by joint names of current kinematic chain (as defined in robot's XML file) ");
	helpMessages.push_back("get object position <name> <CoordinateSystem>... Returns object's position (x,y,z) in coordinate system of robots RobotNode with name <CoordinateSystem>. If RobotNode cannot be found or <CoordinateSystem> is not set, the global coord system is used.");
	helpMessages.push_back("get object orientation <name> <CoordinateSystem>... Returns object's orientation as three RPY values in coordinate system of robots RobotNode with name <CoordinateSystem>. If RobotNode cannot be found or <CoordinateSystem> is not set, the global coord system is used.");
	
	helpMessages.push_back("planMotion  start_1 ... start_i goal_1 ... goal_i   A collision-free motion is planned for current RobotNodeSet from start to goal configuration. On success (first result=VOCAB_OK), the path in configuration space is returned as set of configs ((start1 ... start_i) (c1_1 ... c1_i) ... (cn_1 ... cn_i) (goal_1 ... goal_i)) ");

	helpMessages.push_back("RETURN: First return value is always VOCAB_OK/VOCAB_FAILED indicating success or failure.");
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
	else if (command.get(0).asString()=="add") 
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
		if (command.get(1).asString()=="joints") 
		{
			customResponse = true;
			responseOK = true;
			reply.addVocab(VOCAB_OK); // OK
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
				reply.addVocab(VOCAB_OK); // OK
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
				reply.addVocab(VOCAB_OK); // OK
				reply.addDouble(rpy(0));
				reply.addDouble(rpy(1));
				reply.addDouble(rpy(2));		
			}
		}
	} else if (command.get(0).asString()=="show") 
	{
		if (command.get(1).asString()=="coordsystem") 
		{
			ConstString name = command.get(2).asString();
			ConstString onOff = command.get(3).asString();
		
			responseStream <<"Showing coord system: " << name;
			bool showC = !(onOff == "off");
			reply.addVocab(VOCAB_OK); // OK
			responseOK = viewer->showCoordSystem(name.c_str(),showC);
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
	} else if (command.get(0).asString()=="planMotion") 
	{
		int joints = (command.size()-1) / 2;
		if (!currentRNS || joints!=currentRNS->getSize())
		{
			cout << "no rns or wrongly defined nr of joints: " << joints << endl; 
			cout << "current config:" << endl;
			print();

		} else
		{
			std::vector<float> start,goal;
			for (int i=0;i<joints;i++)
			{
				start.push_back((float)command.get(1+i).asDouble());
				goal.push_back((float)command.get(1+joints+i).asDouble());
			}
			responseOK = planMotion(start,goal);
			commandProcessed = true;
			if (responseOK)
			{
				customResponse = true;
				reply.addVocab(VOCAB_OK); // OK
				unsigned int pts = currentSolutionOpti->getNrOfPoints();
				yarp::os::Bottle &result = reply.addList();
				cout << "Adding " << pts << " path points (joint configs) to result" << endl;
				for (unsigned int i=0;i<pts;i++)
				{
					yarp::os::Bottle &c = result.addList();
					Eigen::VectorXf config = currentSolutionOpti->getPoint(i);
					for (int j=0;j<config.rows();j++)
					{
						c.addDouble((double)config[j]);
					}					
				}
				//reply.addList(result);
			}
		}
	} else
		commandProcessed = false;

	if (!customResponse)
	{
		reply.addInt((int)responseOK);
		if (responseOK)
			reply.addVocab(VOCAB_OK); // OK
		else
			reply.addVocab(VOCAB_FAILED); 

		if (!commandProcessed)
		{
			responseStream << "Unknown command: " << command.toString().c_str() << "\n Try 'help' \n";
			cout << helpMessage;
		}
	}
	return true;
}

bool SimoxMotionPlannerModule::interruptModule()
{
	handlerPort.interrupt();
	printf ("INTERRUPT\n");
	return true;
}

bool SimoxMotionPlannerModule::close()
{	
	handlerPort.close();

	return true;
}

double SimoxMotionPlannerModule::getPeriod()
{
	// 50 fps
	return 0.02;
}


bool SimoxMotionPlannerModule::selectRNS(const std::string &rns)
{
	if (!robot || !robot->hasRobotNodeSet(rns))
	{
		VR_ERROR << "Robot does not have RNS with name " << rns << endl;
		return false;
	}
	currentRNS = robot->getRobotNodeSet(rns);
	
	// we have to re-setup the planner
	//cout << "TODO: SETUP planner..." << endl;

	return true;
}

bool SimoxMotionPlannerModule::loadRobot( const std::string &filename )
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

void SimoxMotionPlannerModule::print()
{
	cout << "***** SimoxMotionPlannerModule *****" << endl;
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

bool SimoxMotionPlannerModule::setObject( const std::string &objectName, std::string filename )
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

	return true;
}


bool SimoxMotionPlannerModule::setObjectPose( const std::string &objectName, Eigen::Matrix4f &pose )
{
	if (objects.find(objectName) == objects.end())
	{
		VR_INFO << "Do not know object " << objectName << endl;
		return false;
	}
	objects[objectName]->setGlobalPose(pose);
	if (viewer)
		viewer->setObjectPose(objectName,pose);
	return true;
}

Eigen::Matrix4f SimoxMotionPlannerModule::getObjectPose( const std::string &objectName )
{
	if (objects.find(objectName) == objects.end())
	{
		VR_INFO << "Do not know object " << objectName << endl;
		return Eigen::Matrix4f::Identity();
	}

	return objects[objectName]->getGlobalPose();
}

bool SimoxMotionPlannerModule::planMotion( const std::vector<float> startConfig, const std::vector<float> goalConfig )
{
	if (!robot || !currentRNS)
		return false;

	if (startConfig.size() != currentRNS->getSize())
	{
		cout << "size of start config does not match size of kin chain: "<< startConfig.size() << "!=" << currentRNS->getSize() << "..." << endl;
		return false;
	}
	if (goalConfig.size() != currentRNS->getSize())
	{
		cout << "size of goal config does not match size of kin chain: "<< goalConfig.size() << "!=" << currentRNS->getSize() << "..." << endl;
		return false;
	}

	// setup Collision detection
	cdm.reset(new VirtualRobot::CDManager());
	if (cdEnabled)
	{

		if (cdEnvironment && rnsCD_robotMove)
		{
			std::map< std::string, VirtualRobot::ManipulationObjectPtr >::iterator it = objects.begin();
			while (it!=objects.end())
			{
				cdm->addCollisionModelPair(it->second,rnsCD_robotMove);
				it++;
			}
		

		} else
		{
			cout << "Collision detection with Environment is disabled..." << endl;
		}
		if (rnsCD_robotMove && rnsCD_robotStatic)
			cdm->addCollisionModelPair(rnsCD_robotMove, rnsCD_robotStatic);
		else
			cout << "Self-Collision-Detection is disabled!" << endl;
	} else
	{
		cout << "Collision detection is disabled!" << endl;
	}



	iCubConfigurationConstraint::ConstraintConfig kinChain;
	if (currentRNS->getSize() == 7)
	{
		kinChain = iCubConfigurationConstraint::eArm;
		cout << "Assuming kin chain covering the arm (no torso): 7 dof" << endl;
	} else if (currentRNS->getSize() == 10)
	{
		cout << "Assuming kin chain covering torso and arm (10 dof)" << endl;
		kinChain = iCubConfigurationConstraint::eTorsoArm;
	} else
	{
		cout << "Expecting kin chain with either 7 or 10 dof... Aborting" << endl;
		return false;
	}

	Saba::ConfigurationConstraintPtr constraint(new iCubConfigurationConstraint(kinChain,currentRNS->getSize()));

	Saba::CSpaceSampledPtr cspace(new Saba::CSpaceSampled(robot,cdm,currentRNS));
	cspace->addConstraintCheck(constraint);
	cspace->setSamplingSize(0.2f);
	cspace->setSamplingSizeDCD(0.1f);

	Saba::BiRrtPtr rrt(new Saba::BiRrt(cspace));

	Eigen::VectorXf s(currentRNS->getSize());
	Eigen::VectorXf g(currentRNS->getSize());
	for (size_t i=0;i<startConfig.size();i++)
	{
		s[i] = startConfig[i];
		g[i] = goalConfig[i];
	}
	std::map<VirtualRobot::ManipulationObjectPtr,std::vector<float> > origPoses;
	if (!checkCollisionGoalConfig(goalConfig,rnsCD_robotMove,origPoses))
	{
		restoreCollisionGoalConfig(origPoses);
		cout << "Goal config in collision. Aborting..." << endl;
		return false;
	}

	rrt->setStart(s);
	rrt->setGoal(g);
	bool res = rrt->plan();

	restoreCollisionGoalConfig(origPoses);

	if (!res)
	{
		cout << "Planning failed..." << endl;
		return false;
	}

	cout << "Planning succeeded..." << endl;
	currentSolution = rrt->getSolution();
	if (viewer)
	{
		viewer->showPath("solution", currentSolution);
	}

	if (pathOptimizingSteps>0)
	{
		cout << "PostPrecessing:" << pathOptimizingSteps << " steps..." << endl;
		Saba::ShortcutProcessor spp(currentSolution,cspace,false); // enable/disable verbose output
		currentSolutionOpti = spp.optimize(pathOptimizingSteps);
		cout << "done." << endl;
	} else
		currentSolutionOpti = currentSolution;
	if (viewer)
	{
		viewer->showPath("solutionOpti", currentSolutionOpti,VirtualRobot::VisualizationFactory::Color::Green());
	}
	return true;
}

bool SimoxMotionPlannerModule::checkObjectMove(VirtualRobot::ManipulationObjectPtr o,  VirtualRobot::SceneObjectSetPtr colModel, float dx, float dy, float dz )
{
	Eigen::Matrix4f p = o->getGlobalPose();
	Eigen::Matrix4f pNew = p;
	pNew(0,3) += dx;
	pNew(1,3) += dy;
	pNew(2,3) += dz;
	o->setGlobalPose(pNew);
	if (!colModel->getCollisionChecker()->checkCollision(o->getCollisionModel(),colModel))
	{
		cout << "found collision free pose. dx: " << dx << ", dy:" << dy << ", dz:" << dz << endl;
		return true;
	}
	o->setGlobalPose(p);
	return false;
}

bool SimoxMotionPlannerModule::checkCollisionGoalConfig( const std::vector<float> &config, VirtualRobot::SceneObjectSetPtr colModel, std::map<VirtualRobot::ManipulationObjectPtr, std::vector<float> > &storeOriginalPoses )
{
	if (!colModel || config.size()!=currentRNS->getSize() || !robot)
		return false;
	if (!searchColFreeObjectPose)
	{
		cout << "Search for col free configs disabled, skipping..." << endl;
		return true;
	}
	std::vector<float> origC;
	currentRNS->getJointValues(origC);
	currentRNS->setJointValues(config);
	

	float maxMove = 55.0f; // 5 cm
	float step = 5.0f;
	VirtualRobot::CollisionCheckerPtr cc = robot->getCollisionChecker();
	std::map< std::string, VirtualRobot::ManipulationObjectPtr >::iterator i = objects.begin();
	while (i!=objects.end())
	{
		if (cc->checkCollision(i->second->getCollisionModel(),colModel))
		{
			cout << "Collision at goal configuration detected. Moving around object to find a col free config...Object:" << i->first << endl;


			Eigen::Matrix4f p = i->second->getGlobalPose();
			Eigen::Matrix4f pNew = p;
			float d = step;
			float dx,dy,dz;
			while (d<maxMove)
			{
				// check all directions
				dx=d;dy=0;dz=0;
				if (checkObjectMove(i->second,colModel, dx,dy,dz))
					break;
				dx=-d;dy=0;dz=0;
				if (checkObjectMove(i->second,colModel, dx,dy,dz))
					break;
				dx=0;dy=d;dz=0;
				if (checkObjectMove(i->second,colModel, dx,dy,dz))
					break;
				dx=0;dy=-d;dz=0;
				if (checkObjectMove(i->second,colModel, dx,dy,dz))
					break;
				dx=0;dy=0;dz=d;
				if (checkObjectMove(i->second,colModel, dx,dy,dz))
					break;
				dx=0;dy=0;dz=-d;
				if (checkObjectMove(i->second,colModel, dx,dy,dz))
					break;
			
				d += step;
			}
			if (d>=maxMove)
			{
				cout << "Could not determine col free goal position!!!" << endl;
				i->second->setGlobalPose(p);
				currentRNS->setJointValues(origC);
				return false;
			}
			std::vector<float> o;
			o.push_back(p(0,3));
			o.push_back(p(1,3));
			o.push_back(p(2,3));
			storeOriginalPoses[i->second] = o;
		}
	
		i++;
	}
	currentRNS->setJointValues(origC);
	return true;

}

bool SimoxMotionPlannerModule::restoreCollisionGoalConfig( std::map<VirtualRobot::ManipulationObjectPtr, std::vector<float> > &originalPoses )
{
	std::map<VirtualRobot::ManipulationObjectPtr, std::vector<float> >::iterator i = originalPoses.begin();
	while (i!=originalPoses.end())
	{
		Eigen::Matrix4f g = i->first->getGlobalPose();
		g(0,3) = i->second[0];
		g(1,3) = i->second[1];
		g(2,3) = i->second[2];
		i->first->setGlobalPose(g);
		i++;
	}
	return true;
}

bool SimoxMotionPlannerModule::setJointLimits( const std::string &robotNodeSet, std::vector<float> &min, std::vector<float> &max )
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


bool SimoxMotionPlannerModule::selectEEF(const std::string &eef, const std::string &eefPreshape)
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

bool SimoxMotionPlannerModule::selectEEF(const std::string &eef)
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


