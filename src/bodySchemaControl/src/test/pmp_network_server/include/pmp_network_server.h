#ifndef PMP_NETWORK_SERVER_H
#define PMP_NETWORK_SERVER_H

#include <pmp_lib/pmp_interface.h>

#include <iostream>

#include "pmp_network.h"
#include "pmp_runner.h"
#include <pmp_lib/core/VTGS.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::pmplib;
using namespace iCub::pmplib::core;
using namespace iCub::pmp_runner;
using namespace iCub::pmp_network;


class PMPnetwork_server :  public RFModule, iCub::pmplib::PMP_interface
	{
	private:

		// module default values:
		static const string CMD_PORT_NAME;
		static const string IF_NAME;

		// network objects declaration
		PMP_network net;
		Network_view<ListDigraph> *view;
		typedef SubDigraph	< ListDigraph,
							  Network_view<ListDigraph>::NodeBoolMap,
							  Network_view<ListDigraph>::ArcBoolMap >	SubGraph;
					    
		Network_runner < SubGraph > runner; // this is a thread

		// objects for storing configuration info
		ResourceFinder rf_default;
		Bottle vtgs_options;
		Property vtgs_prop;

		// flags and semaphores
		bool isConfigured;
		bool needToClose;
		Semaphore mutex;
		Port cmdPort;

		// pose control
		Matrix Rroot2palm;

		// pmp_network graph manipulation
		bool build(string mode = "default", bool fromDefaultRF = true);
		bool buildNodes(Bottle * nodeList);
		bool buildArcs(Bottle * archList);
		inline int nodeIdFromName(const string &nodeName);
		inline ListDigraphBase::Node nodeFromId(const int &nodeId);
		inline ListDigraphBase::Node nodeFromName(const string &nodeName);

		// pmp runner manipulation
		void initRunner();

		// pose computation:
		yarp::sig::Matrix getPose(const yarp::sig::Vector &rpy);

		// RF module:
		virtual bool configure(yarp::os::ResourceFinder &rf_default);
		virtual bool updateModule();
		virtual bool interruptModule();
		virtual bool close();
		virtual bool respond(const Bottle &command, Bottle &reply);
		virtual double getPeriod();

		void test();

	public:

		virtual bool openInterface(yarp::os::Property &options);//{cout << "Not implemented" << endl;return true;};
		virtual void closeInterface();//{cout << "Not implemented" << endl;};
		virtual bool createNode(yarp::os::Property &options);
		virtual bool createConnection(yarp::os::Property &options);
		virtual bool destroyNode(const std::string &name);
		virtual bool destroyConnection(const std::string &source, const std::string &target);
		virtual bool enableNode(const std::string &name, yarp::os::Property *options = NULL);
		virtual bool enableConnection(const std::string &src, const std::string &tg, yarp::os::Property *options = NULL);
		virtual bool disableNode(const std::string &name);
		virtual bool disableConnection(const std::string &src, const std::string &tg);

		virtual bool getPosition(const std::string &nodeName, yarp::sig::Vector &x);
		virtual bool getPosition(const std::string &nodeName, const int & joint, yarp::sig::Vector &x){cout << "Not implemented" << endl;return true;};
		virtual bool getPose(const std::string &nodeName, yarp::sig::Matrix &x);
		virtual bool getPose(const std::string &nodeName, const int & joint, yarp::sig::Matrix &x){cout << "Not implemented" << endl;return true;};

		virtual bool getAng(const std::string &nodeName, yarp::sig::Vector &x);
		virtual bool setAng(const std::string &nodeName, const yarp::sig::Vector &x);

		virtual bool setTarget(const std::string &name, yarp::os::Property &options); // deve ctrl se nodo è abilitato!
		virtual bool setTargetPosition(const std::string &name, const yarp::sig::Vector &v);
		virtual bool setTargetPose(const std::string &name, const yarp::sig::Vector &rpy);

		virtual bool start();

		virtual bool setNodeParameters(const std::string &name, yarp::os::Property &params){cout << "Not implemented" << endl;return true;};
		virtual bool getNodeParameters(const std::string &name, yarp::os::Property &params){cout << "Not implemented" << endl;return true;};
		virtual bool setConnectionParameters(const std::string &source, const std::string &target, yarp::os::Property &params){cout << "Not implemented" << endl;return true;};
		virtual bool getConnectionParameters(const std::string &source, const std::string &target, yarp::os::Property &params){cout << "Not implemented" << endl;return true;};
		
		virtual bool enableSimulation(){cout << "Not implemented" << endl;return true;};
		virtual bool enableExecution(){cout << "Not implemented" << endl;return true;};
		virtual bool getSuccess(){cout << "Not implemented" << endl;return true;};
	};

#endif
