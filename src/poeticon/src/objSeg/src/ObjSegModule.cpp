// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Bjoern Browatzki
 * email:   bjoern.Browatzki@tuebingen.mpg.de
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
 * @file ObjSegModule.cpp
 * @brief 
 */

#include <yarp/dev/Drivers.h>
YARP_DECLARE_DEVICES(icubmod)

#include "iCub/ObjSegModule.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;


void ObjSegModule::defineCommands()
{
    commands.clear();

    //                            command id,   rpc text,       desc     
    commands.push_back(CommandDef(HELP,         "help",         "get this list")); 
    commands.push_back(CommandDef(QUIT,         "quit",         "quit the module")); 
    commands.push_back(CommandDef(SUSPEND,      "suspend",      "suspends the module"));
    commands.push_back(CommandDef(RESUME,       "resume",       "resumes the module"));
    commands.push_back(CommandDef(TRAIN,        "train",        "train model"));
    commands.push_back(CommandDef(UPDATE,       "update",       "update model"));
    commands.push_back(CommandDef(SEGMENT,      "segment",      "segment image into bkg/hand/object"));
}

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 * equivalent of the "open" method.
 */

bool ObjSegModule::configure(yarp::os::ResourceFinder &rf) {    

    YARP_REGISTER_DEVICES(icubmod)

    /* Process all parameters from both command-line and .ini file */
    /* get the module name which will form the stem of all module port names */
    moduleName = rf.check("name", Value("objSeg"), "module name (string)").asString();

    /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
    setName(moduleName.c_str());

    /* now, get the rest of the parameters */
    /*
    * get the robot name which will form the stem of the robot ports names
    * and append the specific part and device required
    */
    robotName = rf.check("robot", Value("icubSim"), "Robot name (string)").asString();
    isSimulation = (robotName == "icubSim");

    std::string arm = rf.check("arm", Value("right"), "left or right arm (string)").asString().c_str();
    bool rightArm = (arm == "right") ? true : false;

    /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */
    std::string handlerPortName = "/";
	handlerPortName	+= getName(rf.check("handlerPort", Value("/rpc:i")).asString());
    if (!handlerPort.open(handlerPortName.c_str())) {           
        cout << getName() << ": Unable to open port " << handlerPortName << endl;  
        return false;
    }

    std::string camImgPortName = "/";
    std::string segImgPortName  = "/";
    std::string objImgPortName  = "/";
    std::string disparityImgPortName  = "/";
	camImgPortName += getName(rf.check("camImgPort", Value("/img:i")).asString());
	segImgPortName += getName(rf.check("segImgPort", Value("/seg:o")).asString());
	objImgPortName += getName(rf.check("objImgPort", Value("/obj:o")).asString());
	disparityImgPortName += getName(rf.check("disparityImgPort", Value("/disparity:i")).asString());
    if (!camImgPort.open(camImgPortName.c_str())) {           
        cout << getName() << ": Unable to open port " << camImgPortName << endl;  
        return false;
    }
    //std::string camRightImgPortName = "/";
	//camRightImgPortName += getName(rf.check("camRightImgPort", Value("/imgRight:i")).asString());
    //if (!camRightImgPort.open(camRightImgPortName.c_str())) {           
        //cout << getName() << ": Unable to open port " << camRightImgPortName << endl;  
        //return false;
    //}
    if (!segImgPort.open(segImgPortName.c_str())) {           
        cout << getName() << ": Unable to open port " << segImgPortName << endl;  
        return false;
    }
    if (!objImgPort.open(objImgPortName.c_str())) {           
        objImgPort.interrupt();
        cout << getName() << ": Unable to open port " << objImgPortName << endl;  
        return false;
    }
    if (!disparityImgPort.open(disparityImgPortName.c_str())) {           
        disparityImgPort.interrupt();
        cout << getName() << ": Unable to open port " << disparityImgPortName << endl;  
        return false;
    }

    attach(handlerPort);                  // attach to port

    /* connect to robot head  */ 

    std::string localHeadPortName = "/";
    localHeadPortName += getName("/head");
    std::string remoteHeadPortName = "/" + robotName + "/head";
    Property optionsHead;
    optionsHead.put("device", "remote_controlboard");
    optionsHead.put("local", localHeadPortName.c_str());
    optionsHead.put("remote", remoteHeadPortName.c_str());

    robotHead.open(optionsHead);
    if (! robotHead.isValid())
    {
        cout << getName() << ": Cannot connect to robot head" << endl;  
        cout << "Device not available.  Here are the known devices:" << endl;
        cout << Drivers::factory().toString() << endl;
        return false;
    }


    // Torso
    std::string localTorsoPortName = "/";
    localTorsoPortName += getName("/torso");
    std::string remoteTorsoPortName = "/" + robotName + "/torso";
    Property torsoOptions;
    torsoOptions.put("device", "remote_controlboard");
    torsoOptions.put("local", localTorsoPortName.c_str());  
    torsoOptions.put("remote", remoteTorsoPortName.c_str());

    robotTorso.open(torsoOptions);
    if (! robotTorso.isValid()) 
    {
        cout << getName() << ": Cannot connect to robot torso" << endl;  
        cout << "Device not available.  Here are the known devices:" << endl;
        cout << Drivers::factory().toString() << endl;
        return false;
    }

    // Arm
    std::string localArmPortName = "/";
    std::string remoteArmPortName;
    if (rightArm)
    {
        localArmPortName += getName("/right_arm");
        remoteArmPortName = "/" + robotName + "/right_arm";
    }
    else
    {
        localArmPortName += getName("/left_arm");
        remoteArmPortName = "/" + robotName + "/left_arm";
    }
    Property armOptions;
    armOptions.put("device", "remote_controlboard");
    armOptions.put("local", localArmPortName.c_str());
    armOptions.put("remote", remoteArmPortName.c_str());

    robotArm.open(armOptions);
    if (! robotArm.isValid()) 
    {
        std::cout << getName() << ": Cannot connect to robot right arm" << std::endl;  
        std::cout << "Device not available.  Here are the known devices:" << std::endl;
        std::cout << Drivers::factory().toString() << std::endl;
        return false;
    }

    /* set up cartesian interface */
    std::string localCartArmPortName = "/";
    std::string remoteCartArmPortName;
    if (rightArm)
    {
        localCartArmPortName += getName("/cartesian/right_arm");
        remoteCartArmPortName = "/" + robotName + "/cartesianController/right_arm";
    }
    else
    {
        localCartArmPortName += getName("/cartesian/left_arm");
        remoteCartArmPortName = "/" + robotName + "/cartesianController/left_arm";
    }
    Property option("(device cartesiancontrollerclient)");
    option.put("local", localCartArmPortName.c_str());
    option.put("remote", remoteCartArmPortName.c_str());

    armCartDriver.open(option);
    if (!armCartDriver.isValid()) 
    {
        cout << getName() << ": Cannot open cartesian interface driver for arm" << endl;  
        cout << "Device not available.  Here are the known devices:" << endl;
        cout << Drivers::factory().toString() << endl;
        return false;
    }

    std::string gazeCtrlName = rf.check("gazeCtrl", Value("iKinGazeCtrl"), "port name of gaze controller").asString().c_str();

    IGazeControl *gazeCtrl;
    std::string localGazePortName = "/";
    localGazePortName += getName("/gaze");
    std::string remoteGazePortName = "/"+gazeCtrlName;

    Property gazeOption;
    gazeOption.put("device","gazecontrollerclient");
    gazeOption.put("local", localGazePortName.c_str());
    gazeOption.put("remote", remoteGazePortName.c_str());

    gazeControlDriver.open(gazeOption);
    if (! gazeControlDriver.isValid()) 
    {
        std::cout << getName() << ": Cannot open gaze control driver" << std::endl;  
        std::cout << "Device not available.  Here are the known devices:" << std::endl;
        std::cout << Drivers::factory().toString() << std::endl;
        return false;
    }
    if (! gazeControlDriver.view(gazeCtrl))
    {
        std::cout << "Cannot get gaze control interface" << std::endl;  
        gazeControlDriver.close();
        return false;
    }
    gazeCtrl->setSaccadesStatus(false);

    if (isSimulation)
    {
        Network::connect("/icubSim/cam/right", camImgPortName.c_str(), "tcp" );
    }


    defineCommands();

    /* create the thread and pass pointers to the module parameters */
    std::cout << "Creating thread..." << std::endl;
    int period_ms = 20;
    objSegThread = new ObjSegThread(period_ms, rf, &camImgPort, &camRightImgPort, 
            &objImgPort, &disparityImgPort, robotArm, robotHead, robotTorso, armCartDriver, gazeCtrl, sendMutex);

    std::cout << "Running thread..." << std::endl;
    /* now start the thread to do the work */
    objSegThread->start(); // this calls threadInit() and it if returns true, it then calls run()

    return true ;     // let the RFModule know everything went well
                      // so that it will then run the module
}

bool ObjSegModule::interruptModule() 
{
    cout << "Interrupting module" << endl;    
    sendMutex.wait();
    handlerPort.interrupt();
    camImgPort.interrupt();
    camRightImgPort.interrupt();
    segImgPort.interrupt();
    disparityImgPort.interrupt();
    sendMutex.post();
    cout << "Module interruped" << endl;    
    return true;
}

bool ObjSegModule::close()
{
    cout << "Closing module" << endl;    
    
    /* stop the thread */
    objSegThread->stop();

    handlerPort.close();
    camImgPort.close();
    camRightImgPort.close();
    segImgPort.close();
 
    disparityImgPort.close();

    robotArm.close();
    robotHead.close();
    robotTorso.close();
    armCartDriver.close();
    gazeControlDriver.close();

    //delete objSegThread;
    cout << "Module closed" << endl;    

    return true;
}

bool ObjSegModule::respond(const Bottle& command, Bottle& reply) 
{
    reply.clear();

    CommandType com;
    if(!identifyCommand(command, com))
    {
        reply.addString("Unknown command. Input 'help' to get a list of the available commands.");
        return true;
    }

    switch(com)
    {
        case SUSPEND:
            reply.addString("suspending");
            objSegThread->suspend();
            break;
        case RESUME:
            reply.addString("resuming");
            objSegThread->resume();
            break;
        case QUIT:
            reply.addString("quitting");
            return false;
        case HELP:
            buildHelpMessage(reply);
            return true;
    }

    reply.addString( (command.toString()+" command received.").c_str() );

    return true;
}

/* Called periodically every getPeriod() seconds */
bool ObjSegModule::updateModule() 
{
    return true;
}

double ObjSegModule::getPeriod() 
{
    /* module periodicity (seconds), called implicitly by myModule */    
    return 0.1;
}

void ObjSegModule::buildHelpMessage(Bottle &bottle)
{
    bottle.addString("many");				// print every string added to the bottle on a new line
    bottle.addString((string(getName().c_str()) + " commands are: ").c_str());
    CommandTable::const_iterator cmd;
    for (cmd = commands.begin(); cmd != commands.end(); ++cmd)
    {
        bottle.addString( ("- "+cmd->rpctxt+": "+cmd->desc).c_str() );
    }
}

/**
  * Identify the command in the bottle and return the correspondent enum value.
  */
bool ObjSegModule::identifyCommand(const Bottle &commandBot, CommandType &com)
{ 
    CommandTable::const_iterator cmd;
    for (cmd = commands.begin(); cmd != commands.end(); ++cmd)
    {
        int pos = commandBot.toString().find(cmd->rpctxt.c_str());
        if (pos != ConstString::npos)
        {
            com = cmd->id;
            return true;
        }
    }
    return false;
}

