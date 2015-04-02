#include <iostream>
#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>

#include <yarp/os/Time.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>

using namespace std;
using namespace yarp::os;

class MyModule:public RFModule
{
	BufferedPort<Bottle>	vergencePort;
    Port					handlerPort; 
	Port					suspendPort;
	double					timeout;
	volatile bool			vergence_accomplished;

public:

    double getPeriod()
    {
        return 0.25; //module periodicity (seconds)
    }

    /*
    * This is our main function. Will be called periodically every getPeriod() seconds.
    */
    bool updateModule()
    {
		Bottle *bot=vergencePort.read(false);

		if(bot!=NULL && bot->toString()=="vergence_accomplished")
			vergence_accomplished=true;
		else
			vergence_accomplished=false;

        return true;
    }

    /*
    * Message handler. Just echo all received messages.
    */
    bool respond(const Bottle& command, Bottle& reply) 
    {
        //cout<<"Got something, echo is on"<<endl;
		if (command.get(0).asString()=="status"){
			double t=Time::now();
			while(Time::now()-t<timeout && !vergence_accomplished)
				Time::delay(0.03);

			if(vergence_accomplished){
				reply.addString("vergence_accomplished");
				Bottle b("sus");
				Bottle r;
				suspendPort.write(b,r);
			}
			else
				reply.addString("failed");

		}
		if (command.get(0).asString()=="attresume"){
			Bottle b("res");
			//cout<<"Got something, res is on"<<endl;
			Bottle r;
			suspendPort.write(b,r);
			cout<<"res sent"<<endl;
			reply.addString("resumed");
		}
		if (command.get(0).asString()=="attsuspend"){
			Bottle b("sus");
			//cout<<"Got something, sus is on"<<endl;
			Bottle r;
			suspendPort.write(b,r);
			cout<<"sus sent"<<endl;
			reply.addString("suspended");
		}
		return true;
    }

    /* 
    * Configure function. Receive a previously initialized
    * resource finder object. Use it to configure your module.
    * Open port and attach it to message handler.
    */
    bool configure(yarp::os::ResourceFinder &rf)
    {
        timeout=rf.check("timeout",Value(60.0)).asDouble();
        vergencePort.open("/radHelper/vergence:i");
        handlerPort.open("/radHelper/attentionStatus:io");
        suspendPort.open("/radHelper/suspend:o");
        attach(handlerPort);

        //Network::connect("/gazeArbiter/icub/left_cam/status:o","/radHelper/vergence:i","udp");
        //Network::connect("/radHelper/suspend:o","/gazeArbiter/icub/left_cam");
        return true;
    }

    /*
    * Interrupt function.
    */
    bool interruptModule()
    {
        vergencePort.interrupt();
        handlerPort.interrupt();
		suspendPort.interrupt();
        return true;
    }
    /*
    * Close function, to perform cleanup.
    */
    bool close()
    {
        vergencePort.close();
		handlerPort.close();
		suspendPort.close();
		return true;
    }
};

int main(int argc, char * argv[])
{
    Network yarp;

    MyModule module;
    ResourceFinder rf;
    rf.configure("ICUB_ROOT", argc, argv);
    rf.setVerbose(true);

    cout<<"Configure module..."<<endl;
    module.configure(rf);
    cout<<"Start module..."<<endl;
    module.runModule();

    cout<<"Main returning..."<<endl;
    return 0;
}
