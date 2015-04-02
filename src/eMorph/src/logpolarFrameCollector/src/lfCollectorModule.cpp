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
 * @file cfCollectorModule.cpp
 * @brief Implementation of the cfCollectorModule (see header file).
 */

#include <iCub/lfCollectorModule.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool lfCollectorModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("/logpolarFrameCollector"), 
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
    robotPortName         = "/" + robotName + "/head";\
    
    /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */
    handlerPortName =  "";
    handlerPortName += getName();         // use getName() rather than a literal 

    if (!handlerPort.open(handlerPortName.c_str())) {           
        cout << getName() << ": Unable to open port " << handlerPortName << endl;  
        return false;
    }

    attach(handlerPort);                  // attach to port
    

    // --------------------------------------------
    lfThread=new lfCollectorThread();
    lfThread->setName(getName().c_str());
    
    /*
    * set the period between two successive synchronisations between viewer and events
    */
    synchPeriod            = rf.check("synchPeriod", 
                           Value(10000), 
                           "synchronisation period (int)").asInt();
    lfThread->setSynchPeriod(synchPeriod);

    /*
    * set the retinaSize (considering squared retina)
    */
    retinalSize            = rf.check("retinalSize", 
                           Value(128), 
                           "retinalSize (int)").asInt();
    lfThread->setRetinalSize(retinalSize);

    
    /* checking whether the module synchronizes with single camera or stereo camera
     */
    if( rf.check("stereo")) {
        lfThread->setStereo(true);
    }
    else {
        lfThread->setStereo(false);
    }

    /* checking whether the module synchronizes with single camera or stereo camera
     */
    if( rf.check("em")) {
        lfThread->setEM(true);
    }
    else {
        lfThread->setEM(false);
    }
    
    /**
     * checking whether the viewer represent log-polar information
     */
    if( rf.check("logpolar")) {
        lfThread->setLogPolar(true);
    }
    else {
        lfThread->setLogPolar(false);
    }
    lfThread->start();

    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool lfCollectorModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool lfCollectorModule::close() {
    handlerPort.close();
    /* stop the thread */
    lfThread->stop();
    printf("stopped the collector thread \n");
    //delete lfThread;
    return true;
}

bool lfCollectorModule::respond(const Bottle& command, Bottle& reply) {
    string helpMessage =  string(getName().c_str()) + 
                        " commands are: \n" +  
                        "help \n" + 
                        "quit \n";

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
bool lfCollectorModule::updateModule() {
    return true;
}



//----- end-of-file --- ( next line intentionally left blank ) ------------------

