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

#include <stdio.h>
#include <yarp/os/Network.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/RFModule.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <string>

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

class laserUtilsModule: public RFModule
{
    Port                  rpcPort;
    BufferedPort<Vector>  port_laser_i1;
    BufferedPort<Vector>  port_laser_i2;
    BufferedPort<Vector>  port_laser_o1;
    Vector                laser1;
    Vector                laser2;
    Vector                laser_out;

    bool                  option_merge_lasers;

public:
    laserUtilsModule()
    {
        laser1.resize(1080,1000.0);
        laser2.resize(1080,1000.0);
        laser_out.resize(1080,1000.0);
        option_merge_lasers = true;
    }

    virtual bool configure(ResourceFinder &rf)
    {
        rpcPort.open("/laserUtils/rpc");
        port_laser_i1.open("/laserUtils/laser1:i");
        port_laser_i2.open("/laserUtils/laser2:i");
        port_laser_o1.open("/laserUtils/laser1:o");       
        attach(rpcPort);
        return true;
    }

    bool respond(const Bottle& command, Bottle& reply) 
    {
        reply.clear(); 
        if (command.get(0).asString()=="help")
        {
            reply.addString("Available commands are:");
            reply.addString("no commands");
         }
        else
        {
            reply.addString("Unknown command.");
        }
        return true;
    }

    virtual bool close()
    {
        rpcPort.close();
        port_laser_i1.close();
        port_laser_i2.close();
        port_laser_o1.close();
        return true;
    }

    virtual double getPeriod()
    {
        return 0.01;
    }

    virtual bool   updateModule() 
    {
        Vector* v1 = port_laser_i1.read(false);
        if (v1) laser1 = *v1;
        Vector* v2 = port_laser_i2.read(false);
        if (v2) laser2 = *v2;
        if (option_merge_lasers == true)
        for (int i=0; i<1080; i++)
            {
                if (laser2[i] < laser1[i]) laser_out[i] = laser2[i];
                else laser_out[i] = laser1[i];
            }
        return true; 
    }

};

int main(int argc, char *argv[]) 
{
    Network yarp;

    if (!yarp.checkNetwork())
    {
        fprintf(stderr, "Sorry YARP network does not seem to be available, is the yarp server available?\n");
        return -1;
    }

    laserUtilsModule mod;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    return mod.runModule(rf);

    return 0;
}
