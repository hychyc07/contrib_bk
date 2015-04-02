// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.reak@iit.it
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
 * @file detectorModule.cpp
 * @brief Implementation of the detectorModule (see header file).
 */

#include "detectorModule.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool detectorModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */
    printf("detectorModule::configure.... \n");
    
    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("/detector"), 
                           "module name (string)").asString();
    /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
    printf("generated moduleName: %s \n", moduleName.c_str());
    setName(moduleName.c_str());

    /*
    * get the robot name which will form the stem of the robot ports names
    * and append the specific part and device required
    */
    printf("extracting the parameters ... \n");
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
    printf("attaching the handler port.... \n");
    handlerPortName =  moduleName;
    //handlerPortName += getName();         // use getName() rather than a literal 
    printf("opening the defined port name %s \n", getName().c_str());
    
    Port handlerPort;
    handlerPort.open("/detector");
    
    //if (!handlerPort.open(handlerPortName.c_str())) {           
    //    cout << "opening port error" <<endl;
        //cout << getName() << ": Unable to open port " << handlerPortName << endl;  
    //    return false;
    //}
    printf("attaching the port \n");
    attach(handlerPort);                  // attach to port
    printf("success in attaching the handler port \n");
    
    
    if (rf.check("config")) {
        configFile=rf.findFile(rf.find("config").asString().c_str());
        if (configFile=="") {
            return false;
        }
    }
    else {
        configFile.clear();
    }
    printf("checking the param file path ...");
    
    paramPath              = rf.check("param", 
                           Value("para_blocks.yml"), 
                           "para_blocks path (string)").asString();
    paramFile = rf.findFile(paramPath.c_str());
    if (paramFile=="") {
        printf("the paramFile was not found. Sorry. \n");
        return false;
    }
    else {
        printf("Found paramFile %s \n", paramFile.c_str());
    }

    trainPath              = rf.check("train", 
                           Value("tr_data_blocks.bin"), 
                           "train_data_blocks path (string)").asString();
    printf("name of the trainPath known %s \n", trainPath.c_str());
    ConstString trainF = rf.findFile(trainPath.c_str());
    printf("Found training file %s \n", trainF.c_str());
    
    std::string trainFile;
    trainFile.append(trainF.c_str());
    
    
    /*
    if (trainFile=="") {
        printf("the trainFile was not found. Sorry. \n");
        return false;
    }
    else {
        printf("Found trainFile \n");
    }
    */
    

    /* create the thread and pass pointers to the module parameters */
    printf("Starting the detector thread ... \n");
    dThread = new detectorThread(robotName, configFile);
    dThread->setName(moduleName.c_str());
    dThread->setParaFile(paramFile);
    dThread->setTrainFile(trainFile);
    //rThread->setInputPortName(inputPortName.c_str());
    
    /* now start the thread to do the work */
    bool res = dThread->start(); // this calls threadInit() and it if returns true, it then calls run()
    if(!res)
        return false;

    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool detectorModule::interruptModule() {
    printf("interrupting the thread \n");
    
    printf("portInterrupted \n");
    dThread->stop();
    
    //handlerPort.interrupt();
    return true;
}

bool detectorModule::close() {
    printf("stopping the thread \n");
    //handlerPort.close();
    /* stop the thread */
    
    //dThread->stop();
    return true;
}

bool detectorModule::respond(const Bottle& command, Bottle& reply) 
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
bool detectorModule::updateModule()
{
    return true;
}

double detectorModule::getPeriod()
{
    /* module periodicity (seconds), called implicitly by myModule */
    return 1;
}

