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
 * @file ObjSegModule.h
 * @brief this file contains the definition of the main module code (that is the module yarp implmentation using the RFModule class.
 */

#ifndef _DEMO_MODULE_H_
#define _DEMO_MODULE_H_

#include <iostream>
#include <string>
#include <vector>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
 
#include "iCub/ObjSegThread.h"

using namespace yarp::os;
using namespace yarp::sig;

/**
 * Tacks and segments object in robot hand
 */
class ObjSegModule : public yarp::os::RFModule
{
    enum CommandType
    {
        HELP,
        STOP,
        SUSPEND,
        RESUME,
        QUIT,
        TRAIN,
        UPDATE,
        SEGMENT,
        SET_POS
    };
    class CommandDef
    {
    public:
        CommandType id;
        std::string rpctxt;
        std::string desc;
        CommandDef(const CommandType _id, const std::string& _rpctxt, const std::string _desc)
            : id(_id), rpctxt(_rpctxt), desc(_desc)
        {}
    };

    typedef std::vector<CommandDef> CommandTable;
    
    std::string moduleName;
    std::string robotName; 
    bool isSimulation;

    Semaphore                        sendMutex;

    yarp::os::Port handlerPort;      //a port to handle messages 

    BufferedPort<ImageOf<PixelBgr> > camImgPort;
    BufferedPort<ImageOf<PixelBgr> > camRightImgPort;
    BufferedPort<ImageOf<PixelBgr> > segImgPort;
    BufferedPort<ImageOf<PixelBgr> > objImgPort;
    BufferedPort<ImageOf<PixelBgr> > disparityImgPort;
    BufferedPort<Bottle> comPort;

    PolyDriver robotArm;
    PolyDriver robotHead;
    PolyDriver robotTorso;
    PolyDriver armCartDriver;
    PolyDriver gazeControlDriver;
    

    /* pointer to a new thread to be created and started in configure() and stopped in close() */
    ObjSegThread *objSegThread;

    CommandTable commands;
    void defineCommands();
    void buildHelpMessage(Bottle &bottle);
    bool identifyCommand(const Bottle &commandBot, CommandType &com);

public:
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod(); 
    bool updateModule();
};


#endif

//----- end-of-file --- ( next line intentionally left blank ) ------------------

