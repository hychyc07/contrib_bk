// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Ajaz Ahmad Bhat
  * email: ajaz.bhat@iit.it
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
 * @file shapeSelectorModule.cpp
 * @brief Implementation of the shapeSelectorModule (see header file).
 */

#include "shapeSelectorModule.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool shapeSelectorModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */   

    
    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("/shapeSelector"), 
                           "module name (string)").asString();
    /* get the module name which will form the stem of all module port names */
    width            = rf.check("width", 
                           Value(320), 
                           "image width size (int)").asInt();
    /* get the module name which will form the stem of all module port names */
    height            = rf.check("height", 
                           Value(240), 
                           "image height size (int)").asInt();                       
    
    dimX            = rf.check("dimX", 
                           Value(640), 
                           "image width size (int)").asInt();
    /* get the module name which will form the stem of all module port names */
    dimY            = rf.check("dimY", 
                           Value(480), 
                           "image height size (int)").asInt();                       
    
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

    inputPortName           = rf.check("inputPortName",
			                Value(":i"),
                            "Input port name (string)").asString();
    

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
    if (rf.check("config")) {
        configFile=rf.findFile(rf.find("config").asString().c_str());
        if (configFile=="") {
            return false;
        }
    }
    else {
        configFile.clear();
    }

    Bottle      *leftBottle, *rightBottle;
    //Semaphore   *leftMutex, *rightMutex;
    
    leftBottle      = new Bottle();
    rightBottle     = new Bottle();
    //leftMutex       = new Semaphore();
    //rightMutex      = new Semaphore();
    
 

    /* create the thread and pass pointers to the module parameters */
    rThread = new shapeSelectorRatethread(robotName, configFile);
    rThread->setName(getName().c_str());
    rThread->setCoordinates(dimX, dimY);
    rThread->setOutputDimensions(width, height);
    //rThread->setInputPortName(inputPortName.c_str());
     
    /* get the module name which will form the stem of all module port names */
  /*  if (rf.check("whitebox") ) {
        rThread->setWhiteBox(true);                       
    }
    else {
        rThread->setWhiteBox(false);
    }
 */   
    /* share the resources and semaphores between the threads*/ 
    rThread->setSharingBottle(leftBottle, rightBottle);
    
    
    //rThread->setSemaphore(leftMutex, rightMutex);
    
    //rThread->setSharingBottle(leftBottle, rightBottle);
    //rThread->setSemaphore(leftMutex, rightMutex);
    
    /* now start the thread to do the work */
    
    rThread->start(); // this calls threadInit() and it if returns true, it then calls run()
    
  
        
    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool shapeSelectorModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool shapeSelectorModule::close() {
    handlerPort.close();
    /* stop the thread */
    printf("stopping the threads \n");

    
    rThread->stop();

    return true;
}

bool shapeSelectorModule::respond(const Bottle& command, Bottle& reply) 
{
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
bool shapeSelectorModule::updateModule()
{
    return true;
}

double shapeSelectorModule::getPeriod()
{
    /* module periodicity (seconds), called implicitly by myModule */
    return 1;
}

