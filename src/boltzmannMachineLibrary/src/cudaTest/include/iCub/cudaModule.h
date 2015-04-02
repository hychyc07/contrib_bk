// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
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
 * @file cudaModule.h
 * @brief A module that controls the process of this application
 */

#ifndef _CUDA_MODULE_H_
#define _CUDA_MODULE_H_


#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>

//within project includes
#include <iCub/cudaThread.h>

class cudaModule:public yarp::os::RFModule {
    std::string moduleName;                     // name of the module (rootname of ports)
    std::string robotName;                      // name of the robot
    std::string robotPortName;                  // reference to the head of the robot
    std::string handlerPortName;                // name of the handler port (comunication with respond function)
    std::string filenameStart;                  // first part of the name of images
    std::string filenameEnd;                    // last part of the name of images
    int numberImages;                           // number of read images
    int ratethread;                             // time constant for ratethread
    cudaThread* ifThread;                // ratethread in charge of sending images
    yarp::os::Port handlerPort;                 // a port to handle messages 
    int argc;
    char* argv[];
public:
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod(); 
    bool updateModule();
    void setArgc(int argc);
    void setArgv(char* argv[]);
};


#endif // __CUDA_MODULE_H__
//----- end-of-file --- ( next line intentionally left blank ) ------------------

