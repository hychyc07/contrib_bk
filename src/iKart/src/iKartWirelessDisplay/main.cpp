/* 
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo Vadim Tikhanoff
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

/** 
\defgroup iKartWireless iKartWireless
 
@ingroup icub_module  
 
iKart wireless display.
 
Copyright (C) 2010 RobotCub Consortium
 
Author: Marco Randazzo Vadim Tikhanoff

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
 
@@@TODO
 
\section portsa_sec Ports Accessed
 
@@@TODO
 
\section portsc_sec Ports Created 
 
@@@TODO

\section in_files_sec Input Data Files

@@@TODO

\section out_data_sec Output Data Files 

@@@TODO
 
\section conf_file_sec Configuration Files

@@@TODO

\section tested_os_sec Tested OS
Windows, Linux

\author Marco Randazzo Vadim Tikhanoff
*/ 

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Os.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <yarp/dev/SerialInterfaces.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>

#include <iostream>
#include <iomanip>
#include <string>
#include <stdlib.h>
#include <time.h>
#include <stdio.h>

#include "graphics.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

class CtrlModule: public RFModule
{
public:
    yarp::os::Network                           yarp;
    Gtk::Main*                                  gtk_main;
    GraphicsManager*                            graphics;
    yarp::os::BufferedPort<yarp::os::Bottle>    monitorOutput;
    sigc::connection                            m_timer;

    std::string                                 moduleName;
    int                                         period;

    yarp::os::ConstString                       picBackground;
    yarp::os::ConstString                       picBlocks;
    yarp::os::ConstString                       picNumbers;

    //yarp::os::ConstString                       systemCmd;

public:
    CtrlModule() 
    {
        gtk_main = new Gtk::Main(0,0);
    }

    virtual bool on_timeout()
    {
        char cmd[] = "lynx -auth=admin:password -dump \" http://10.0.0.250/cgi-bin/cgi?req=frm&frm=info.html&rand=1616217005\" | grep -i status";
        FILE* pipe =popen(cmd,"r");
        if (!pipe) return "ERROR";
        char buffer[128];
        string result = "";
        while(!feof(pipe)) {
    	if(fgets(buffer, 128, pipe) != NULL)
    		result += buffer;
        }
        pclose(pipe);
        
        static int signal=0;
        char tmp[20];
        static double strenght=0.0;

        sscanf(result.c_str(),"%*s %*s %d %*s %s",&signal,tmp );
        strenght = atof (tmp+1);
   
        Bottle& outBot = monitorOutput.prepare();
        outBot.clear();
        outBot.addInt(signal);    
        outBot.addDouble(strenght);
        monitorOutput.write();

        graphics->update_graphics(signal,strenght);

        return true;
    }

    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        //check if the yarp networ is running
        if (yarp.checkNetwork()==false)
        {
            return false;
        }

        moduleName = rf.check("name", Value(1), "module name (string)").asString();
        setName(moduleName.c_str());
        
        period = rf.check("period", Value(5000), "update period (int)").asInt();

        string pname  = "/" + moduleName + ":o";
        monitorOutput.open(pname.c_str());
         
        picBlocks = rf.findFile(rf.check("pic_blocks", Value(1), "module name (string)").asString());
        picBackground = rf.findFile(rf.check("pic_background", Value(1), "module name (string)").asString());
        picNumbers = rf.findFile(rf.check("pic_numbers", Value(1), "module name (string)").asString());

        graphics = new GraphicsManager(picBackground.c_str(),picBlocks.c_str(),picNumbers.c_str());
        m_timer = Glib::signal_timeout().connect(sigc::mem_fun(*this, &CtrlModule::on_timeout), period);
        on_timeout();

        //start GTK loop
        gtk_main->run(*graphics);

        return true;
    }
    virtual bool interruptModule()
    {
        gtk_main->quit();
        monitorOutput.interrupt();
        if (graphics) delete graphics;
        delete gtk_main;
        close();
        return true;
    }
    
    virtual bool close()
    {
        monitorOutput.close();
        m_timer.disconnect();
        fprintf(stdout,"done...\n");
        yarp::os::exit(1);        
        return true;
    }

    virtual double getPeriod()    { return 60.0;  }
    virtual bool   updateModule() { return true; }
};


int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile( "conf/config.ini" ); //overridden by --from parameter
    rf.setDefaultContext( "iKartWirelessDisplay" );   //overridden by --context parameter
    rf.configure("ICUB_ROOT",argc,argv);

    if (rf.check("help"))
    {
        cout << "Options:" << endl << endl;
        cout << "\tNo options at the moment"<< endl;
        return 0;
    }

    CtrlModule mod;

    return mod.runModule(rf);
}



