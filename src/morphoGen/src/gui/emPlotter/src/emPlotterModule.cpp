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
 * @file emPlotterModule.cpp
 * @brief Implementation of the emPlotterModule (see header file).
 */

#include "emPlotterModule.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool emPlotterModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */   

    
    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("/emPlotter"), 
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

    Bottle*     partialBottle[5], *remBottle[5], *hubTop,*hubBottomAll, *hubBottom[5], *planA, *planB;
    Semaphore*  pmutex[5], *rmutex[5], *mutexTop, *mutexBottomAll, *mutexBottom[5], *mutexA, *mutexB;
    
    for (int i = 0; i < 5; i++) {
        partialBottle[i]    = new Bottle();
        remBottle[i]        = new Bottle();
        hubBottom[i]        = new Bottle();
        pmutex[i]           = new Semaphore();
        rmutex[i]           = new Semaphore();
        mutexBottom[i]      = new Semaphore();
    }
    
    hubTop          = new Bottle();
    hubBottomAll    = new Bottle();
    planA           = new Bottle();
    planB           = new Bottle();
    mutexTop        = new Semaphore();
    mutexBottomAll  = new Semaphore();
    mutexA          = new Semaphore();
    mutexB          = new Semaphore();
       
 

    /* create the thread and pass pointers to the module parameters */
    rThread = new emPlotterRatethread(robotName, configFile);
    rThread->setName(getName().c_str());
    //rThread->setInputPortName(inputPortName.c_str());
     
    
    remThread = new rememberedxThread();
    remThread->setName(getName().c_str());
    
    hThread   = new hubThread();
    hThread->setName(getName().c_str());
    
    pThread   = new planThread();
    pThread->setName(getName().c_str());
    
    /* share the resources and semaphores between the threads*/ 
    rThread->setSharingBottle(partialBottle, remBottle, hubBottom, hubTop, hubBottomAll, planA, planB);
    rThread->setSemaphore(pmutex, rmutex, mutexBottom, mutexTop, mutexBottomAll, mutexA, mutexB);
    
    remThread->setSharingBottle(remBottle);
    remThread->setSemaphore(rmutex);
    
    hThread->setSharingBottle(hubTop, hubBottomAll, hubBottom);
    hThread->setSemaphore(mutexTop, mutexBottomAll, mutexBottom);
    
    pThread->setSharingBottle(planA, planB);
    pThread->setSemaphore(mutexA, mutexB);
    
    rThread->start(); // this calls threadInit() and it if returns true, it then calls run()
    remThread->start(); 
    hThread->start();
    pThread->start();
    
    for (int i = 0; i < 5; i++){
        
        pt[i] = new partialThread();
        std::string tempName = pt[i]->getName(getName().c_str());
        tempName.append("/partialQ");
        
        tempName.append(static_cast<ostringstream*>( &(ostringstream() << i ) )->str());
        pt[i]->setName(tempName.c_str());
        pt[i]->setSharingBottle1(partialBottle[i]);
        pt[i]->setSemaphore(pmutex[i]);
        pt[i]->start();
    }
    /* now start the thread to do the work */
    
    Time::delay(5);

    
        
    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool emPlotterModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool emPlotterModule::close() {
    handlerPort.close();
    /* stop the thread */
    printf("stopping the threads \n");

    for (int i = 0; i < 5; i++){
        pt[i]->stop();
    }
    remThread->stop();
    rThread->stop();
    hThread->stop();
    pThread->stop();
    return true;
}

bool emPlotterModule::respond(const Bottle& command, Bottle& reply) 
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
bool emPlotterModule::updateModule()
{
    return true;
}

double emPlotterModule::getPeriod()
{
    /* module periodicity (seconds), called implicitly by myModule */
    return 1;
}

