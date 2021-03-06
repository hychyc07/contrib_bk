/* 
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Port.h>

#include "ikartPlanner.h"
#include <math.h>

class iKartPlannerModule : public yarp::os::RFModule
{
protected:
    PlannerThread     *plannerThread;
    yarp::os::Port     rpcPort;

public:
    iKartPlannerModule()
    {
        plannerThread=NULL;
    }

    virtual bool configure(yarp::os::ResourceFinder &rf)
    {
        yarp::os::Time::turboBoost();

        Property p;
        ConstString configFile = rf.findFile("from");
        if (configFile!="") p.fromConfigFile(configFile.c_str());

        plannerThread = new PlannerThread(20,rf,p);

        rpcPort.open("/ikartPathPlanner/rpc:i");
        attach(rpcPort);
        //attachTerminal();

        if (!plannerThread->start())
        {
            delete plannerThread;
            return false;
        }

        return true;
    }

    virtual bool interruptModule()
    {
        rpcPort.interrupt();

        return true;
    }

    virtual bool close()
    {
        rpcPort.interrupt();
        rpcPort.close();

        //gotoThread->shutdown();
        plannerThread->stop();
        delete plannerThread;
        plannerThread=NULL;

        return true;
    }

    virtual double getPeriod()
    { 
        return 3.0; 
    }
    
    virtual bool updateModule()
    { 
        if (isStopping())
        {
            plannerThread->stop();   
            return false;
        }
        
        bool err = false;
        if (plannerThread->laser_timeout_counter>TIMEOUT_MAX)
        {
            printf ("ERROR: timeout, no laser data received!\n");
            err= true;
        }
        if (plannerThread->loc_timeout_counter>TIMEOUT_MAX)
        {
            printf ("ERROR: timeout, no localization data received!\n");
            err= true;
        }
        if (plannerThread->inner_status_timeout_counter>TIMEOUT_MAX)
        {
            printf ("ERROR: timeout, no status info received!\n");
            err= true;
        }
        
        if (err==false)
        printf ("module running, ALL ok\n");

        return true; 
    }

    virtual bool respond(const yarp::os::Bottle& command,yarp::os::Bottle& reply) 
    {
        reply.clear(); 

        plannerThread->mutex.wait();
        if (command.get(0).isVocab())
        {
            if (command.get(0).asVocab() == VOCAB3('p','n','t'))
            {
                yarp::sig::Vector v;
                v.push_back(command.get(1).asDouble());
                v.push_back(command.get(2).asDouble());
                plannerThread->setNewAbsTarget(v);
                reply.addString("new absolute target received from gui");
            }
        }
    
        else if (command.get(0).isString())
        {
            if (command.get(0).asString()=="quit")
            {
                plannerThread->mutex.post();
                return false;
            }

            else if (command.get(0).asString()=="help")
            {
                reply.addVocab(Vocab::encode("many"));
                reply.addString("Available commands are:");
                reply.addString("gotoAbs <x> <y> <angle>");
                reply.addString("gotoRel <x> <y> <angle>");
                reply.addString("stop");
                reply.addString("pause");
                reply.addString("resume");
                reply.addString("quit");
            }

            else if (command.get(0).asString()=="gotoAbs")
            {
                yarp::sig::Vector v;
                v.push_back(command.get(1).asDouble());
                v.push_back(command.get(2).asDouble());
                if (command.size()==4) v.push_back(command.get(3).asDouble());
                plannerThread->setNewAbsTarget(v);
                reply.addString("new absolute target received");
            }

            else if (command.get(0).asString()=="gotoRel")
            {
                yarp::sig::Vector v;
                v.push_back(command.get(1).asDouble());
                v.push_back(command.get(2).asDouble());
                if (command.size()==4) v.push_back(command.get(3).asDouble());
                plannerThread->setNewRelTarget(v);
                reply.addString("new relative target received");
            }

            else if (command.get(0).asString()=="get")
            {
                if (command.get(1).asString()=="navigation_status")
                {
                    string s = plannerThread->getNavigationStatus();
                    reply.addString(s.c_str());
                }
            }
            else if (command.get(0).asString()=="stop")
            {
                plannerThread->stopMovement();
                reply.addString("Stopping movement.");
            }
            else if (command.get(0).asString()=="pause")
            {
                double time = -1;
                if (command.size() > 1)
                    time = command.get(1).asDouble();
                plannerThread->pauseMovement(time);
                reply.addString("Pausing.");
            }
            else if (command.get(0).asString()=="resume")
            {
                plannerThread->resumeMovement();
                reply.addString("Resuming.");
            }
            else
            {
                reply.addString("Unknown command.");
            }
        }
        plannerThread->mutex.post();
        return true;
    }
};

int main(int argc, char *argv[])
{
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf(stderr,"ERROR: check Yarp network.\n");
        return -1;
    }

    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("iKartPathPlanner.ini");           //overridden by --from parameter
    rf.setDefaultContext("iKartPathPlanner/conf");             //overridden by --context parameter
    rf.configure("ICUB_ROOT",argc,argv);
    
    iKartPlannerModule iKartPlanner;

    return iKartPlanner.runModule(rf);
}

 
