// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Katrin Lohan
  * email: katrin.lohan@iit.it
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

#include "iCub/InterfaceModule.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool InterfaceModule::configure(yarp::os::ResourceFinder &rf) {

    interface  = 0;

    if(rf.check("help")) {
        printf("HELP \n");
        printf("--name : changes the rootname of the module ports \n");
        printf("--robot : changes the name of the robot where the module interfaces to  \n");
        printf("--name : rootname for all the connection of the module \n");
        printf("====== \n");
        printf("press CTRL-C to continue.. \n");
        return true;
    }

    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */
    moduleName = rf.check("name",
                           Value("ObjectAura"),
                           "module name (string)").asString();

    /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
    setName(moduleName.c_str());
    printf("module will be activated with the name: %s \n", getName().c_str());

    /*
    * get the robot name which will form the stem of the robot ports names
    * and append the specific part and device required
    */
    robotName = rf.check("robot",
                           Value("icubSim"),
                           "Robot name (string)").asString();
    robotPortName = "/" + robotName + "/head";
    printf("robotName: %s \n", robotName.c_str());
    
    /*configName = rf.check("config",
                           Value("icubEyes.ini"), 
                           "Config file for intrinsic parameters (string)").asString();
    printf("configFile: %s \n", configName.c_str());

    if (strcmp(configName.c_str(),"")) {
        printf("looking for the config file \n");
        configFile=rf.findFile(configName.c_str());
        printf("config file %s \n", configFile.c_str());
        if (configFile=="") {
            printf("ERROR: file not found");
            return false;
        }
    }
    else {
        configFile.clear();
    }*/

    inputPortName1   = "/";
	inputPortName1	+= getName(
					rf.check("InPort1",
					Value("/balltracker1:i"),
					"Input image port (string)").asString()
					);
    
    inputPortName2   = "/";
	inputPortName2	+= getName(
					rf.check("InPort2",
					Value("/balltracker2:i"),
					"Input image port (string)").asString()
					);

    inputPortName3   = "/";
	inputPortName3	+= getName(
					rf.check("InPort3",
					Value("/balltracker3:i"),
					"Input image port (string)").asString()
					);

    inputPortName4   = "/";
	inputPortName4	+= getName(
					rf.check("InPort4",
					Value("/handtracker:i"),
					"Input image port (string)").asString()
					);

	outputPortName  = "/";
	outputPortName  += getName(
					rf.check("OutPort",
					Value("/data:o"),
					"Output image port (string)").asString()
					);

    outputPortName2  = "/";
	outputPortName2  += getName(
					rf.check("OutPort2",
					Value("/log:o"),
					"Log port (string)").asString()
					);


    //opening Ports
    if (!port1.open(inputPortName1.c_str())) {  
    cout << getName() << ": unable to open port " << inputPortName1 << endl;       
    return false;     
    }
    if (!port2.open(inputPortName2.c_str())) {  
    cout << getName() << ": unable to open port " << inputPortName2 << endl;       
    return false;     
    }
    if (!port3.open(inputPortName3.c_str())) {  
    cout << getName() << ": unable to open port " << inputPortName3 << endl;       
    return false;     
    }
    if (!port4.open(inputPortName4.c_str())) {  
    cout << getName() << ": unable to open port " << inputPortName4 << endl;       
    return false;     
    }

    if (!outport.open(outputPortName.c_str())) {  
    cout << getName() << ": unable to open port " << outputPortName << endl;       
    return false;     
    }

    if (!outport2.open(outputPortName2.c_str())) {  
    cout << getName() << ": unable to open port " << outputPortName2 << endl;       
    return false;     
    }
    //launching components
    printf("running the InterfaceThread \n");
    interface=new InterfaceThread(&port1, &port2, &port3, &port4, &outport, &outport2);
    interface->start(); // this calls threadInit() and it if returns true, it then calls run()

//    interface->setName(getName().c_str());
//    interface->setRobotName(robotName);
    

    /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */
//    handlerPortName =  "";
//    handlerPortName += getName();         // use getName() rather than a literal
//
//    if (!handlerPort.open(handlerPortName.c_str())) {
//        cout << getName() << ": Unable to open port " << handlerPortName << endl();
//        return false;
//    }
//    attach(handlerPort);                  // attach to port
    //attach(Port);                       // attach to port

    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool InterfaceModule::interruptModule() {    
    port1.interrupt();  
    port2.interrupt();  
    port3.interrupt();  
    port4.interrupt();  
    outport.interrupt();  
    outport2.interrupt(); 
    return true;
}

bool InterfaceModule::close() {
    port1.close();  
    port2.close();  
    port3.close();  
    port4.close();  
    outport.close(); 
    outport2.close(); 
    printf("stopping threads \n ");
    if(0 != interface) {
        interface->stop();
        delete interface;
    }
   

    printf("InterfaceModule::close:success in closing \n");
    return true;
}

bool InterfaceModule::respond() {
    return true;
}

/* Called periodically every getPeriod() seconds */
bool InterfaceModule::updateModule() {
    return true;
}

double InterfaceModule::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */
    return 1.0;
}

