// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <stdio.h>
#include "iCub/cartesianForce/cartesianForce.h"

#include <iostream>
#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>

using namespace std;
using namespace yarp::os;

class MyModule:public RFModule
{
    cartesianForce imp;
public:

    double getPeriod()
    {
        return 0.02; //module periodicity (seconds)
    }

    /*
     * This is our main function. Will be called periodically every getPeriod() seconds.
     */
    bool updateModule()
    {
//      cout << " updateModule... "<<endl;
//		double t1 = Time::now();
        if (imp.send_cmd)
            imp.loop();
        else
            fflush(stdout);

//		double t2 = Time::now();
//		printf("Time: %lf\n",t2-t1);
        return true;
    }

    /*
     * Message handler. Just echo all received messages.
     */
    bool respond(const Bottle& command, Bottle& reply)
    {

        fflush(stdout);
        if (command.get(0).asString()=="quit")
            return false;
        else if (command.get(0).asString()=="q") {
            imp.close();
            detachTerminal();
            interruptModule();
            //stopModule();
            //exit(0);
            return false;
        }
        else if ((command.get(0).asString()=="go")||(command.get(0).asString()=="g"))
        {
            imp.start();
            imp.send_cmd = true;
            reply.addString("[OK]");
            fflush(stdout);
            return true;
        }
        else if ((command.get(0).asString()=="stop")||(command.get(0).asString()=="s"))
        {
            imp.send_cmd = false;
            imp.stop();
            reply.addString("[OK]");
            fflush(stdout);
            return true;
        }
        else if (command.get(0).asString()=="set")
        {
            if (command.get(1).asString()=="Kp")
            {
                if (command.size() != 3)
                {
                    reply.addString("Need 1 argument for Kp");
                    fflush(stdout);
                    return false;
                }
                imp.Kp = command.get(2).asDouble();

                reply.addString("[OK]");
                fflush(stdout);
                return true;
            }
            else if (command.get(1).asString() == "Kd")
            {
                if (command.size() != 3)
                {
                    reply.addString("Need 1 argument for Kd");
                    fflush(stdout);
                    return false;
                }
                imp.Kd = command.get(2).asDouble();

                reply.addString("[OK]");
                fflush(stdout);
                return true;
            }
            else if (command.get(1).asString() == "Fd")
            {
                if (command.size() != 8)
                {
                    reply.addString("Need 8 arguments for Fd");
                    fflush(stdout);
                    return false;
                }
                imp.Fd(0) = command.get(2).asDouble();
                imp.Fd(1) = command.get(3).asDouble();
                imp.Fd(2) = command.get(4).asDouble();

                imp.Fd(3) = command.get(5).asDouble();
                imp.Fd(4) = command.get(6).asDouble();
                imp.Fd(5) = command.get(7).asDouble();

                reply.addString("[OK]");
                fflush(stdout);
                return true;

            }
            else
            {
                reply.addString("Sub command not understood");
                fflush(stdout);
                return false;
            }
            fflush(stdout);
            return true;
        }
        else if (command.get(0).asString()=="get")
        {
            if (command.get(1).asString()=="Kp")
            {
                reply.addDouble(imp.Kp);
                fflush(stdout);
                return true;
            }
            else if (command.get(1).asString() == "Kd")
            {
                reply.addDouble(imp.Kd);
                fflush(stdout);
                return true;
            }
            else if (command.get(1).asString() == "Fd")
            {
                reply.addString(imp.Fd.toString().c_str());
                fflush(stdout);
                return true;
            }
            else
            {
                reply.addString("Sub command not understood");
                fflush(stdout);
                return true;
            }
        }
        else if (command.get(0).asString() == "verbose")
        {
            if (command.get(1).asString() == "on")
            {
                imp.verbose = true;
                reply.addString("[OK]");
            }
            else if (command.get(1).asString() == "off")
            {
                imp.verbose = false;
                reply.addString("[OK]");
            }
            else
                reply.addString("Sub-command not understood");

            fflush(stdout);
            return true;

        }
        else if ((command.get(0).asString() == "home")||(command.get(0).asString()=="h"))
        {
            imp.home();
            reply.addString("[OK]");
            fflush(stdout);
            return true;
        }
        else
        {
            reply.addString("Command not understood");
            fflush(stdout);
            return true;
        }

    }

    /*
     * Configure function. Receive a previously initialized
     * resource finder object. Use it to configure your module.
     * Open port and attach it to message handler.
     */
    bool configure(yarp::os::ResourceFinder &rf)
    {

        imp.controlled_part = rf.find("part").asString();
        imp.robotname = rf.find("robot").asString();

        if (imp.controlled_part == "right_arm")
        {
            fprintf(stderr, "Using right arm\n");
        }
        else if (imp.controlled_part == "left_arm")
        {
            fprintf(stderr, "Using left arm\n");
        }
        else
        {
            fprintf(stderr, "part not recognised\n");
            return false;
        }

        if (!imp.open())
        {
            fprintf(stderr, "Error opening detector\n");
            return false;
        }
        fflush(stdout);
        return true;
    }

    /*
     * Interrupt function.
     */
    bool interruptModule()
    {
        cout<<"Interrupting your module, for port cleanup"<<endl;
        close();
        return true;
    }

    /*
     * Close function, to perform cleanup.
     */
    bool close()
    {
        imp.close();
        detachTerminal();
        return true;
    }
};

int main(int argc, char * argv[])
{
    Network yarp;
    if (!yarp.checkNetwork()) {
        printf("No yarp network, bye!\n");
        return -1;
    }

    MyModule module;
    ResourceFinder rf;
    rf.configure("ICUB_ROOT", argc, argv);
    rf.setVerbose(true);

    cout<<"Configure module..."<<endl;
    fflush(stdout);
    if (!module.configure(rf))
    {
        printf("Error configuring module\n");
        return 1;
    }
    printf("done.\n");
    fflush(stdout);
    module.attachTerminal();
    fflush(stdout);
    module.runModule();
    cout<<"Main returning..."<<endl;
    return 0;
}
