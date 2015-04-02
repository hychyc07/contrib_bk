// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea
 * email:   francesco.rea@iit.it
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
 * @file dvsGrabberModule.cpp
 * @brief Implementation of the dvsGrabberModule (see header file).
 */

#include <iCub/dvsGrabberModule.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool dvsGrabberModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("/dvsGrabber2"), 
                           "module name (string)").asString();
    /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
    setName(moduleName.c_str());

    /*
    * get the robot name which will form the stem of the robot ports names
    * and append the specific part and device required
    */
    robotName             = rf.check("robot", 
                           Value("icub"), 
                           "Robot name (string)").asString();
    robotPortName         = "/" + robotName + "/head";

    /*
    * get the device name which will be used to read events
    */
    deviceName             = rf.check("deviceName", 
                           Value("/dev/retina0"), 
                           "Device name (string)").asString();
    devicePortName         =  deviceName ;
    printf("trying to connect to the %s \n",devicePortName.c_str());

       /*
    * get the number to attach to the portname
    */
    deviceNumber             = rf.check("retinaNumber", 
                           Value(0), 
                           "Device number (int)").asInt();
    printf("opening port with device number %d \n", deviceNumber);

    /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */
    handlerPortName =  "";
    handlerPortName += getName();         // use getName() rather than a literal 

    if (!handlerPort.open(handlerPortName.c_str())) {           
        cout << getName() << ": Unable to open port " << handlerPortName << endl;  
        //return false;
    }
    attach(handlerPort);                  // attach to port

    bool _save = false;
    string deviceNum = "0";
    string fileName = "raw_events.bin";
    /*
     * get the name of the file where events are saved
     */
    std::string dumpNameComplete = "";
    std::string dumpName;

    dumpName = rf.check("dumpFile", 
                        Value("none"), 
                        "filename of the binary (string)").asString();

    printf("trying to save events in %s  \n",dumpName.c_str());
    

    //dumpNameComplete = "contrib/src/eMorph/src/dvsGrabber/xyz";
    //dumpName = "xyz.dat";
    //dumpNameComplete = "/usr/local/src/robot/iCub/contrib/src/eMorph/src/dvsGrabber/xyz.dat";
    
    if(!strcmp(dumpName.c_str(),"none")) {
        printf("not reading from binary \n");
        D2Y=new device2yarp(devicePortName, false,dumpNameComplete);
    }
    else {
        printf("reading from binary \n");
        dumpNameComplete = rf.findFile(dumpName.c_str());
        //D2Y->setFromBinary(true);
        D2Y=new device2yarp(devicePortName, true, dumpNameComplete);     
    }

    if (rf.check("verbosity")) {
        D2Y->setVerbosity(true);
        printf("verbosity required \n");
    }
    else {
        printf("verbosity  not required \n");
        //the default value for arbiter->visualCorrection is false
    }

    D2Y->setDeviceNumber(deviceNumber);
    printf("starting the thread \n");
    D2Y->start();

    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool dvsGrabberModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool dvsGrabberModule::close() {
    handlerPort.close();
    D2Y->stop();
    /* stop the thread */
    return true;
}

bool dvsGrabberModule::respond(const Bottle& command, Bottle& reply) {
    string helpMessage =  string(getName().c_str()) + 
                        " commands are: \n" +  
                        "help \n" + 
                        "quit \n" + 
                        "set thr <n> ... set the threshold \n" + 
                        "(where <n> is an integer number) \n";

    reply.clear(); 

    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;     
    }
    else if (command.get(0).asString()=="help") {
        cout << helpMessage;
        reply.addString("ok");
    }
    
    return true;
}

/* Called periodically every getPeriod() seconds */
bool dvsGrabberModule::updateModule() {
    return true;
}

double dvsGrabberModule::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */
    return 0.0;
}

