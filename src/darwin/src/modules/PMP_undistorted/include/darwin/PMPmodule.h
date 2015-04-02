#ifndef PMP_MODULE_H
#define PMP_MODULE_H

#include <iostream>
#include <string>

#include "darwin/PMPthread.h"
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Semaphore.h>

using namespace std;
using namespace yarp::os;
//using namespace yarp::sig;

namespace Darwin{

namespace pmp{

class PMPmodule : public RFModule
{
public:
	// aggiungere anche 1 cmd getshapeDescription: arriva da modulo shape recognition come 1 bottiglia contenente una
	// sequenza di punti critici i.e. descritti come |xyz|weights|tbg_param|activeChain.
	// Data questa bottiglia fare 1 funzione getNextCriticalPoint() che legge tutti i parametri e setta i 
	// corrispondenti valori del PMPthread.

	// aggiungere comando isConnected o simile per verificare connessione a devdriver. Anche cmd connectTo e prefisso porta.

	typedef enum {update, back2default, getModulePeriod, getThreadPeriod, getThreadMode, setMaxIterNumber_right,
				  setMaxIterNumber_left, setActiveChain, getActiveChain, PMP_set, VTGSright_set, VTGSleft_set, 
				  PMP_get, VTGSright_get, VTGSleft_get, startPMP, stopPMP, resetPMP, initIcubUp, initHead,
				  initRightArm, initLeftArm, initTorso, ready, update_angles, useIndex, exit
				 } PMPCommands; //module/threadName

	bool configure(yarp::os::ResourceFinder &rf_default);
	bool interruptModule();
	bool close();
	bool respond(const Bottle &command, Bottle &reply);
	double getPeriod();
	bool updateModule();

private:

	static const string CMD_LIST[];
	static const string CMD_DESC[];
	static const unsigned int CMD_SIZE;

	// module default values:
	static const string CMD_PORT_NAME;
	static const string MODULE_NAME;
	static const string RF_UPDATE_NAME;
	static const string THREAD_NAME;
	static const int    THREAD_PERIOD;
	static const string THREAD_RPC_CLI;
	static const string THREAD_RPC_SRV;
	static const string THREAD_RPC_SRV_NAME;

	string moduleName;
	string cmdPortName;
	Port cmdPort;
	bool isConfigured;
	bool isStarted, isSuspended;
	bool startThread, stopThread, resetThread;
	bool needToClose;
	int threadPeriod;
	PMPthread::PMPstate threadState;

	ResourceFinder rf_default;

	string rf_update_path;
	Property options;
	Property threadPropPMP_right,  threadPropPMP_left, threadPropPMP_bimanual, threadPropPMP_tbg;
	Property threadPropVTGS_right, threadPropVTGS_left;

	Semaphore configSem;
	Semaphore updateSem;
	Semaphore startSem;

	PMPthread *myPMP;

	Bottle critical_points; // critical point sequence ( |xyz|xyz| .. ): to be filled by shape module

	bool initialize(string mode = "default", bool fromDefaultRF = true);
	bool mergeProperties(bool fromDefaultRF = true);
	bool identifyCmd(Bottle cmdBot, PMPCommands &cmd);
	bool createPMPproperty(ResourceFinder &rf_update,  string mode = "default");
	bool createVTGSproperty(ResourceFinder &rf_update, string mode = "default");
	string PropertyGroup2String(Bottle group);
	Value PropertyGroup2Bottle(Bottle group);
	bool startPMPthread();
	bool stopPMPthread();
	bool resetPMPthread();

};

}// end namespace pmp
}// end namespace Darwin

#endif
