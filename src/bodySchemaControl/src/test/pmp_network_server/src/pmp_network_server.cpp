#include "pmp_network_server.h"
#include <pmp_lib/pmp_utils.h>

using namespace iCub::pmplib;

// module default values
const string PMPnetwork_server::IF_NAME			= "PMPserver";
const string PMPnetwork_server::CMD_PORT_NAME	= "/PMPserver/rpc";

bool PMPnetwork_server::openInterface(yarp::os::Property &options)
{
	string policy;

	if (!options.check("Verbose"))		rf_default.setVerbose(true);
	else								rf_default.setVerbose((bool)options.find("Verbose").asInt());
	if (!options.check("ConfigFile"))	rf_default.setDefaultConfigFile("PMPnetworkConfiguration.ini");
	else								rf_default.setDefaultConfigFile(options.find("ConfigFile").asString().c_str());
	if (!options.check("Context"))		rf_default.setDefaultContext("pmp_network_server/conf");
	else								rf_default.setDefaultContext(options.find("Context").asString().c_str());
	if (!options.check("Policy"))		policy = "ICUB_ROOT";
	else								policy = options.find("Policy").asString().c_str();

	bool ok = rf_default.configure(policy.c_str(),0,NULL);

	bool done;
	if(ok)	done = configure(rf_default);
	else		return false;

	if(!done)	return false;

	// open nodes ports:


	return true;
}

void PMPnetwork_server::closeInterface()
{
	if(runner.isRunning()) runner.stop();
	stopModule();
}

bool PMPnetwork_server::configure(yarp::os::ResourceFinder &rf_default)
{
	isConfigured = false;
	if(rf_default.isNull())
	{
		printf("No configuration file found, closing... /n");
		return false;
	}

	this->rf_default = rf_default;

	// create PMPnetwork:
	if(!build())	return false;
	//create pmp network view
	view = new Network_view<>(*net.graph(),net.net.root);
	//initialize pmp network runner
	initRunner();	

	// initialize pose matrices
	Rroot2palm.resize(3,3);
	Rroot2palm.zero();
	Rroot2palm(2,2) = Rroot2palm(0,0) = -1;
	Rroot2palm(1,1) = 1;


	attachTerminal();
	if (!cmdPort.open(CMD_PORT_NAME.c_str())) cout << "Error: unable to open rpc port" << endl;
    attach(cmdPort);

	isConfigured = true;
	needToClose = false;

	return true;
}

bool PMPnetwork_server::interruptModule()
{
	if (runner.isRunning())	runner.stop();
	cout << "interrupting .. " << endl;
	
	return true;
}

bool PMPnetwork_server::close()
{
	cout << "closing..." << endl;

	cmdPort.interrupt();
	cmdPort.close();
	return true;
}
double PMPnetwork_server::getPeriod()
{
	return 0.1;
}
bool PMPnetwork_server::updateModule()
{
	if (needToClose)
	{
		interruptModule();
		return false;
	}

	return true;
}
bool PMPnetwork_server::respond(const Bottle &command, Bottle &reply)
{
	reply.clear();
	Property opt;

	string s = command.get(0).asString().c_str();
	if (s == "test")
	{
		cout << "test" << endl;
		test();
		reply.addString("ok");
		return true;
	}
	return true;
	// check that runner is not running to set/get values.
	//if (runner.isRunning()) return false;
	//mutex.wait();
}
// --------------------------------------------------------
bool PMPnetwork_server::buildNodes(Bottle * nodeList)
{
	string file;
	Property prop;
	
	for (int i=0; i<nodeList->size(); i++)
	{
		file = rf_default.getContextPath().c_str();
		file += "/";
		file = file + (nodeList->get(i).asString().c_str());
		file += ".ini";
		prop.fromConfigFile(file.c_str());

		//cout << file << endl;
		string name = nodeList->get(i).asString().c_str();
		
		pmpBlock b(name);

		if (!prop.check("pmp")		||
			!prop.check("tbg")		)
		{
			cout << "error in "<< file << ": some parameters missing" << endl;
			return false;
		}	

		Bottle pmp = prop.findGroup("pmp");
		if (!pmp.check("Admittance")		||
			!pmp.check("VirtualStiffness")	||
			!pmp.check("q_initial")		||
			!pmp.check("q_ref")			||
			!pmp.check("JointStiffness")	)
		{
			cout << "some pmp parameters missing" << endl;
			cout << pmp.toString() << endl;
			return false;
		}

		Vector q = Bottle2Vector(*pmp.find("q_initial").asList());
		Vector q_ref = Bottle2Vector(*pmp.find("q_ref").asList());
		Vector A = Bottle2Vector(*pmp.find("Admittance").asList());		
		Vector K = Bottle2Vector(*pmp.find("VirtualStiffness").asList());
		Vector K_int = Bottle2Vector(*pmp.find("JointStiffness").asList());
		
		if ( (q.size() != q_ref.size()) || (q.size() != A.size()) || (q.size() != K_int.size()) )
		{
			cout << "parameter size error" << endl;
			return false;
		}

		b.initializeBlock(&pmp);
		b.initializeTbg(&prop.findGroup("tbg"));

		net.push_node(b, name, prop.check("root"));

/*		b.set_K(K);
		b.set_A(A);
		b.set_q(q);
		b.set_q_ref(q_ref);
		b.set_Kint(K_int);			
*/		
		
	}

	return true;
}

bool PMPnetwork_server::buildArcs(Bottle * arcList)
{
	for (int i=0; i<arcList->size(); i++)
	{
		string name = arcList->get(i).asString().c_str();
		Bottle block = rf_default.findGroup(name.c_str());

		string source = block.find("source").asString().c_str();
		string target = block.find("target").asString().c_str();
		// check that are existing nodes:
		//cout << net.map()->nodeIdFromName(source) << endl;
		//cout << net.map()->nodeIdFromName(target) << endl;

		if ( (net.map()->nodeIdFromName(source) < 0) || (net.map()->nodeIdFromName(target) < 0) )
		{
			cout << "non existing nodes" << endl;
			return false;
		}
		// check if other parameters are specified
		if (block.size()>2)
		{
			Matrix a;
			
			if (block.check("kine") && block.check("Admittance"))
			{
				string kineType = block.find("kine").asString().c_str();
				arc_block b(kineType);
				Vector A = Bottle2Vector(*block.find("Admittance").asList());
				a.resize(A.size(), A.size());
				a.diagonal(A);
				b.setA(a);
				net.connect(source, target, b);
				//cout << "A and kine" << endl;
			}
			else if (block.check("kine"))
			{
				string kineType = block.find("kine").asString().c_str();
				net.connect(source, target, kineType);
				//cout << "kine" << endl;
			}
			else if (block.check("Admittance"))
			{
				Vector A = Bottle2Vector(*block.find("Admittance").asList());
				a.resize(A.size(), A.size());
				a.diagonal(A);
				net.connect(source, target, a);
				//cout << "A" << endl;
			}	
			else
				net.connect(source, target);
		}
		else
		{
			cout << "not enough parameters" << endl;
			return false;
		}
	}

	return true;
}

bool PMPnetwork_server::build(string mode, bool fromDefaultRF)
{
	Bottle * nodeList = rf_default.find("nodeList").asList();
	Bottle * connectionList = rf_default.find("connectionList").asList();

	if (!rf_default.check("vtgs")){cout << "vtgs parameters missing" << endl; return false;}
	vtgs_options = rf_default.findGroup("vtgs");
	vtgs_prop.fromString(vtgs_options.toString());
	
	// create network nodes and arches/connections between nodes
	if (buildNodes(nodeList))
	{if(!buildArcs(connectionList))	return false;}
	else							return false;
	
	cout << "total nodes: " << lemon::countNodes(*net.graph()) << endl;
	cout << "total arcs: " << lemon::countArcs(*net.graph()) << endl;
	
	return true;
}

// --------------------------------------------------------
inline int PMPnetwork_server::nodeIdFromName(const string &nodeName)
{
	return net.map()->nodeIdFromName(nodeName);
}
inline ListDigraphBase::Node PMPnetwork_server::nodeFromId(const int &nodeId)
{
	return net.graph()->nodeFromId(nodeId);
}

inline ListDigraphBase::Node PMPnetwork_server::nodeFromName(const string &nodeName)
{
	return nodeFromId(nodeIdFromName(nodeName));
}


// --------------------------------------------------------
void PMPnetwork_server::initRunner()
{
	runner.init(view->pview(),view->Root(),net.map()->nodes,net.map()->arcs,&view->visitor);
	runner.start();
}


// --------------------------------------------------------
bool PMPnetwork_server::createNode(yarp::os::Property &options)
{
	string name = options.find("chain").asString().c_str();
	if (net.map()->nodeIdFromName(name) >= 0) return false;

	if (!options.check("Admittance")		||
		!options.check("VirtualStiffness")	||
		!options.check("q_initial")		||
		!options.check("q_ref")			||
		!options.check("JointStiffness")||
		!options.check("chain"))
	{
		cout << "some pmp parameters missing" << endl;
		cout << options.toString() << endl;
		return false;
	}

	Vector q = Bottle2Vector(*options.find("q_initial").asList());
	Vector q_ref = Bottle2Vector(*options.find("q_ref").asList());
	Vector A = Bottle2Vector(*options.find("Admittance").asList());		
	Vector K = Bottle2Vector(*options.find("VirtualStiffness").asList());
	Vector K_int = Bottle2Vector(*options.find("JointStiffness").asList());
	
	if ( (q.size() != q_ref.size()) || (q.size() != A.size()) || (q.size() != K_int.size()) )
	{
		cout << "parameter size error" << endl;
		return false;
	}

	pmpBlock b(name);
	b.initializeBlock(&options);
	b.initializeTbg(&options);

	net.push_node(b, name, options.check("root"));
	return true;
}

bool PMPnetwork_server::createConnection(yarp::os::Property &options)
{
	string source = options.find("source").asString().c_str();
	string target = options.find("target").asString().c_str();
	
	// check that are existing nodes:
	if ( (net.map()->nodeIdFromName(source) < 0) || (net.map()->nodeIdFromName(target) < 0) )
	{
		cout << "non existing nodes" << endl;
		return false;
	}
	// check if other parameters are specified
	bool hasChain = options.check("chain");
	bool hasAdmitt = options.check("Admittance");

	Matrix a;
		
	if (hasChain && hasAdmitt)
	{
		string kineType = options.find("chain").asString().c_str();
		arc_block b(kineType);
		Vector A = Bottle2Vector(*options.find("Admittance").asList());
		a.resize(A.size(), A.size());
		a.diagonal(A);
		b.setA(a);
		return net.connect(source, target, b);
	}
	else if (hasChain)
	{
		string kineType = options.find("chain").asString().c_str();
		return net.connect(source, target, kineType);
	}
	else if (hasAdmitt)
	{
		Vector A = Bottle2Vector(*options.find("Admittance").asList());
		a.resize(A.size(), A.size());
		a.diagonal(A);
		return net.connect(source, target, a);
	}	
	else
		return net.connect(source, target);
}
bool PMPnetwork_server::destroyNode(const std::string &name)
{
	if (net.map()->nodeIdFromName(name) < 0) return false;

	// check that the nodes are available for manipulation
	if (!runner.isWaiting()) return false;
	if (!runner.clear()) return false;
	
	net.pop_node(name);
	return true;
}
bool PMPnetwork_server::destroyConnection(const std::string &source, const std::string &target)
{
	if (net.map()->nodeIdFromName(source) < 0 || net.map()->nodeIdFromName(target) < 0) return false;

	// check that the conenctions are available for manipulation
	if (!runner.isWaiting()) return false;
	if (!runner.clear()) return false;

	net.pop_arc(source, target);
	return true;
}
// --------------------------------------------------------
bool PMPnetwork_server::enableNode(const std::string &name, yarp::os::Property *options)
{
	if (nodeIdFromName(name) < 0) return false;

	// check that the nodes are available for manipulation
	if (!runner.isWaiting()) return false;

	view->enable(nodeFromName(name));
	if (options!=NULL)	setNodeParameters(name, *options);

	return true;
}

bool PMPnetwork_server::enableConnection(const std::string &source, const std::string &target, yarp::os::Property *options)
{
	if (nodeIdFromName(source) < 0 || nodeIdFromName(target) < 0) return false;

	// check that the conenctions are available for manipulation
	if (!runner.isWaiting()) return false;

	view->enable(nodeFromName(source),nodeFromName(target));
	if (options!=NULL)	setConnectionParameters(source, target, *options);

	return true;
}

bool PMPnetwork_server::disableNode(const std::string &name)
{
	if (nodeIdFromName(name) < 0) return false;

	// check that the nodes are available for manipulation
	if (!runner.isWaiting()) return false;
	if (!runner.clear()) return false;

	view->disable(nodeFromName(name));//, false);
	return true;
}

bool PMPnetwork_server::disableConnection(const std::string &source, const std::string &target)
{
	if (nodeIdFromName(source) < 0 || nodeIdFromName(target) < 0) return false;

	// check that the conenctions are available for manipulation
	if (!runner.isWaiting()) return false;
	if (!runner.clear()) return false;

	view->disable(nodeFromName(source),nodeFromName(target));//, false);
	return true;
}

// --------------------------------------------------------
bool PMPnetwork_server::getPosition(const std::string &nodeName, yarp::sig::Vector & x)
{
	int NodeId = nodeIdFromName(nodeName);
	if (NodeId < 0) return false;

	// check that the nodes are available for reading
	if (!runner.isWaiting()) return false;

	// if node is root:
	if ( NodeId == net.graph()->id(net.map()->visitor.Root()) )
		x = net.map()->nodes[nodeFromId(NodeId)].getEEPosition();
	// else:
	else
	{
		if (!runner.initialized) initRunner();
		pathChain pch = runner.getPathChain(NodeId);
		x = pch.getH().subcol(0,3,3);
	}

	return true;
}
bool PMPnetwork_server::getPose(const std::string &nodeName, yarp::sig::Matrix & x)
{
	if (nodeIdFromName(nodeName) < 0) return false;

	// check that the nodes are available for reading
	if (!runner.isWaiting()) return false;

	// if node is root:
	x = net.map()->nodes[nodeFromName(nodeName)].getEEPose();
	// else:
	if (!runner.initialized) initRunner();
	pathChain pch = runner.getPathChain(nodeIdFromName(nodeName));
	x = pch.getH().submatrix(0,2,0,2);

	return true;
}
// --------------------------------------------------------

bool PMPnetwork_server::getAng(const std::string &nodeName, yarp::sig::Vector &x)
{
	if (nodeIdFromName(nodeName) < 0) return false;

	// check that nodes are available for reading
	if (!runner.isWaiting()) return false;

	x = net.map()->nodes[nodeFromName(nodeName)].NodeChain()->getAng();
	return true;
}

bool PMPnetwork_server::setAng(const std::string &nodeName, const yarp::sig::Vector &x)
{
	if (nodeIdFromName(nodeName) < 0) return false;

	// check that nodes are available for writing
	if (!runner.isWaiting()) return false;

	// check that the joint angles vector size is compatible
	if (net.map()->nodes[nodeFromName(nodeName)].NodeChain()->getDOF()!= x.size())
		return false;

	net.map()->nodes[nodeFromName(nodeName)].updateState(x);
	return true;
}

// --------------------------------------------------------
bool PMPnetwork_server::setTarget(const std::string &name, yarp::os::Property &options) // deve ctrl se nodo è abilitato!
{
	// check that the algorithm is not already running
	if (!runner.isWaiting()) return false;

	int nodeId = nodeIdFromName(name);
	if (nodeId < 0) return false;
	if (!view->view().status(nodeFromId(nodeId))) return false;

	// start the runnner if it has not been initialized before
	if (!runner.isRunning()) initRunner();

	bool compensate = true;
	string vtgsMode = "default";
	Bottle vtgsOpt;
	bool hasPose = false;
	bool hasPosition = false;
	Matrix pose(3,3);

	// check the target options:
	if (options.check("pose"))
	{
		hasPose = true;
		pose = Rroot2palm*RPY2Rot(Bottle2Vector(*options.find("pose").asList()));
	}
	if (options.check("position"))	hasPosition = true;
	if (!hasPose && !hasPosition)	return false;

	if (options.check("compensate")) compensate = (bool)options.find("compensate").asInt();

	if (options.check("vtgsMode"))   vtgs_prop.put("vtgsMode",options.find("vtgsMode").asString().c_str());
	if (options.check("T_init1"))	 vtgs_prop.put("T_init1",options.find("T_init1").asDouble());
	if (options.check("T_init2"))	 vtgs_prop.put("T_init2",options.find("T_init2").asDouble());
	if (options.check("T_dur1"))	 vtgs_prop.put("T_dur1",options.find("T_dur1").asDouble());
	if (options.check("T_dur2"))	 vtgs_prop.put("T_dur2",options.find("T_dur2").asDouble());
	if (options.check("SlopeRamp1")) vtgs_prop.put("SlopeRamp1",options.find("SlopeRamp1").asDouble());
	if (options.check("SlopeRamp2")) vtgs_prop.put("SlopeRamp2",options.find("SlopeRamp2").asDouble());
	if (options.check("alpha1"))	 vtgs_prop.put("alpha1",options.find("alpha1").asDouble());
	if (options.check("alpha2"))	 vtgs_prop.put("alpha2",options.find("alpha2").asDouble());
	

	vtgsOpt.fromString(vtgs_prop.toString());
	int id;

	// assign the target to the runner:
	if (hasPose && hasPosition)
		runner.setTarget(id,
						 Bottle2Vector(*options.find("position").asList()),
						 pose,
						 nodeId,
						 vtgsOpt,
						 compensate);
	else if (hasPose)
		runner.setTarget(id,
						 pose,
						 nodeId,
						 vtgsOpt,
						 compensate);
	else
		runner.setTarget(id,
						 Bottle2Vector(*options.find("position").asList()),
						 nodeId,
						 vtgsOpt,
						 compensate);

	// restore default values
	vtgs_prop.fromString(vtgs_options.toString());

	return true;
}
bool PMPnetwork_server::setTargetPosition(const std::string &name, const yarp::sig::Vector &v)
{
	// check that the algorithm is not already running
	if (!runner.isWaiting()) return false;

	int nodeId = nodeIdFromName(name);
	if (nodeId < 0) return false;

	// check that the node is enabled
	if (!view->view().status(nodeFromId(nodeId))) return false;	
	if (v.size() != 3) return false;

	// start the runnner if it has not been initialized before
	if (!runner.isRunning()) initRunner();

	int id;
	runner.setTarget(id, v, nodeId, vtgs_options);
	
	return true;
}

bool PMPnetwork_server::setTargetPose(const std::string &name, const yarp::sig::Vector &rpy)
{
	// check that the algorithm is not already running
	if (!runner.isWaiting()) return false;

	int nodeId = nodeIdFromName(name);
	if (nodeId < 0) return false;
	if (!view->view().status(nodeFromId(nodeId))) return false;
	
	if (rpy.size() != 3) return false;

	// start the runnner if it has not been initialized before
	if (!runner.isRunning()) initRunner();

	int id;
	Matrix pose (3,3);
	pose = Rroot2palm*RPY2Rot(rpy);
	runner.setTarget(id, pose, nodeId, vtgs_options);
	return true;
}
// --------------------------------------------------------
bool PMPnetwork_server::start()
{
	// check that the algorithm is not already running
	if (!runner.isWaiting())
	{
		cout << "thread is busy" << endl;
		return false;
	}

	return runner.go();
}
void PMPnetwork_server::test()
{
	Network_drawer<> drawer;
	drawer.init(net.graph(), net.map()->nameMap);
	drawer.draw("Network");

	//runner.getPathChain(nodeIdFromName("right_arm"));
	
	//this->disableNode("left_arm");
	Vector x(3,0.0);
	bool ok = this->getPosition("right_arm",x);
	cout << "right_arm position: " << x.toString() << endl;		

	Network_drawer<SubGraph> Vdrawer;
	Vdrawer.init(&view->view(), net.map()->nameMap);
	Vdrawer.draw("NetworkView");

	Vector tg(3);
	tg(0) = -0.53;
	tg(1) = 0.1;
	tg(2) = 0.1;

	//cout << "angoli : " << net.map()->nodes[net.graph()->nodeFromId(id)].NodeDefaultChain().getAng().toString() << endl;

	// attenzione: un sottografo non ha la funzione valid come il grafo
	// quindi il runner non può controllare se il nodo è disabilitato o non valido.
	// la funzione di interfaccia set Target deve controllare se il nodo è valido

	Vector RPY(3,0.0);
	// tutto zero = mano palmo in giù in avanti lungo -xroot
	// pollice in direzione -yroot e z parallelo a zroot:
	// roll = Rotz(R), pitch = Roty(P), yaw = Rotx(Y)

	// posa con RPY pollice in su: rotazione attorno x di 90°
	// palmo appartenente al piano x,z
	RPY(2) = 90;
	//RPY(2) = 0;

	// get corrisponding Rotation matrix respect root reference frame:
	Matrix pose = Rroot2palm*pmpBlock::RPY2Rot(RPY);
	cout << "pose: " << endl << pose.toString() << endl;
	

	runner.useDynamicCreation(false);
	int pathId;
	runner.setTarget(pathId, tg, pose, nodeIdFromName("right_arm"), vtgs_options);
	tg(1) = -0.1;
	//runner.setTarget(pathId, tg, nodeIdFromName("left_arm"), vtgs_options);
    cout << "set target called from test"<< endl;
    Time::delay(2);
    //runner.go();
    //cout << "go about to be called from test"<< endl;

	RPY(2) = 0;
    while (!start()) {Time::delay(0.5);}

    //do{start();Time::delay(0.5);}
    while (!runner.isWaiting());
    cout << " about to set new target" << endl;
    Time::delay(2);
	runner.setTarget(pathId, pose, nodeIdFromName("right_arm"), vtgs_options);

    //do{start();Time::delay(0.5);}
    while (!start()) {Time::delay(0.5);}
    while (!runner.isWaiting());//{cout << "busy"<< endl;}

	//runner.start();
	//while (runner.isRunning());


}
// --------------------------------------------------------
			//virtual bool setNodeParameters(const std::string &name, const yarp::os::Property &params);
			//virtual bool getNodeParameters(const std::string &name, yarp::os::Property &params);
			//virtual bool setConnectionParameters(const std::string &source, const std::string &target, const yarp::os::Property &params);
			//virtual bool getConnectionParameters(const std::string &source, const std::string &target, yarp::os::Property &params);
// --------------------------------------------------------
			//virtual bool enableSimulation();
			//virtual bool enableExecution();
			//virtual bool getSuccess();
