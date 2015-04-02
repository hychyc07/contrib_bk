#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <iostream>
#include <iomanip>
#include <string>

#include "bridgeThread.h"

using namespace std;
using namespace yarp::os;

class BridgeModule: public RFModule
{
    protected:
    BridgeThread     *bridge_thr;
    Port             rpcPort;

    public:
    BridgeModule() 
    {
        bridge_thr=0;
    }

    virtual bool configure(ResourceFinder &rf)
    {
        string slash="/";
        string ctrlName;
        string robotName;
        string partName;
        string remoteName;
        string localName;

        Time::turboBoost();

        // get params from the RF
        ctrlName=rf.check("ctrlName",Value("ikart")).asString();
        robotName=rf.check("robot",Value("ikart")).asString();

        remoteName=slash+robotName+"/wheels";
        localName=slash+ctrlName;//+"/local/";

        // reads the configuration file
        Property options;
        ConstString configFile=rf.findFile("from");
        if (configFile=="") 
        {
            printf("\nError! Cannot find .ini configuration file. \nBy default I'm searching for iKartCtrl.ini\n");
        }
        else
        {
            options.fromConfigFile(configFile.c_str());
        }

        // set the thread rate
        int rate = rf.check("rate",Value(30)).asInt();
        printf("bridge thread rate: %d ms.\n",rate);

        // create the bridge thread
        bridge_thr=new BridgeThread(rate,rf,options,remoteName,localName);
        if (!bridge_thr->start())
        {
            delete bridge_thr;
            return false;
        }

        rpcPort.open("/ikart_ros_bridge/rpc");
        attach(rpcPort);

        return true;
    }

    bool respond(const Bottle& command, Bottle& reply) 
    {
        reply.clear(); 
        
        if (!bridge_thr)
        {
            reply.addString("nack");
            return true;
        }
   
		if (command.get(0).asString()=="gotoAbs")
        {
			int v = bridge_thr->setGoal(command.get(1).asDouble(),command.get(2).asDouble(),command.get(3).asDouble());
			reply.addString("ack");
			return  true;
		}
		else
		if (command.get(0).asString()=="gotoRel")
		{
			reply.addString("not yet implemeented");
			return  true;
		}
		else
		if (command.get(0).asString()=="set")
        {
            if (command.get(1).asString()=="current_home")
            {
                bridge_thr->setHome();
                reply.addString("ack");
                reply.addString("setting current position as new home");
                return  true;
            }
            if (command.get(1).asString()=="frame")
            {
                bridge_thr->setUserTarget(command.get(2).asInt(),command.get(3).asDouble(),command.get(4).asDouble(),command.get(5).asDouble());
                reply.addString("ack");
                reply.addString("setting frame");
                return  true;
            }
            else if (command.get(1).asString()=="home")
            {
                bridge_thr->setHome(command.get(2).asDouble(),command.get(3).asDouble(),command.get(4).asDouble());
                reply.addString("ack");
                reply.addString("setting new home position");
                return  true;
            }
            else if (command.get(1).asString()=="goal")
            {
                int v = bridge_thr->setGoal(command.get(2).asDouble(),command.get(3).asDouble(),command.get(4).asDouble());
                reply.addString("ack");
                return  true;
            }
        }
        else
		if (command.get(0).asString()=="stop")
        {
            int v = bridge_thr->navigationStop();
            reply.addString("ack");
            return  true;        
        }
        else
		if (command.get(0).asString()=="get")
        {
            if (command.get(1).asString()=="home")
            {
                double x,y,angle;
                bridge_thr->getHome(x,y,angle);
                reply.addString("ack");
                reply.addDouble(x);
                reply.addDouble(y);
                reply.addDouble(angle);                    
                return  true;
            }     
            else if (command.get(1).asString()=="navigation_status")
            {
                string s = bridge_thr->getNavigationStatus();
                reply.addString("ack");
                reply.addString(s.c_str());
                return true;
            }
        }
        else if (command.get(0).asString()=="help")
        {
            reply.addString("Available commands:");
            reply.addString("set goal <x> <y> <angle>");
            reply.addString("set home <x> <y> <angle>");
            reply.addString("set frame <name> <x> <y> <angle>");
            reply.addString("set current_home");
            reply.addString("get home");
            reply.addString("get navigation_status");
            reply.addString("stop");
			reply.addString("gotoAbs <x> <y> <angle>");
			reply.addString("gotoRel <x> <y> <angle>");
            return true;
        }
        
        reply.addString("Unknown command. Type 'help' to have the list");
        return true;
    }

    virtual bool close()
    {
        if (bridge_thr)
        {
            bridge_thr->stop();
            delete bridge_thr;
        }
        rpcPort.interrupt();
        rpcPort.close();

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }

    virtual bool   updateModule()
    { 
        if (bridge_thr)
        {
            bridge_thr->printStats();
        }
        else
        {
            fprintf(stdout,"bridge thread not running\n");
        }
        return true;
    }
};



int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);
    rf.setDefaultContext("iKart/conf");
    rf.setDefaultConfigFile("bridgeConf.ini");

    if (rf.check("help"))
    {
        printf("\n");
        printf("Available options:");
        printf("--laser_resample <n>\n");
        printf("--no_odom_tf\n");
        printf("\n");
        return 0;
    }

    Network yarp;

    if (!yarp.checkNetwork())
    {
        fprintf(stderr, "Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
    }

    BridgeModule mod;

    return mod.runModule(rf);
}

