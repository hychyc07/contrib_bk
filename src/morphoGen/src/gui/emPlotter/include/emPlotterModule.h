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
 * @file emPlotterModule.h
 * @brief Simple module for plotting the hubs and plans.
 */

#ifndef _EMPLOTTER_MODULE_H_
#define _EMPLOTTER_MODULE_H_

/** 
 *
 * \defgroup icub_code
 * @ingroup icub_morphoGen
 *
 * This is a module that receives the bottles (patial cues) from input connection and sends the activated hubs and plans to output connection.
 * The purpose of the module is to check into the remembered instances.
 * 
 *
 * 
 * \section
 *
 * YARP.
 *
 * \section
 * 
 * <b>Command-line Parameters</b> 
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - \c from \c emPlotter.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c emPlotter/conf \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c emPlotter \n 
 *   specifies the name of the emPlotter (used to form the stem of emPlotter port names)  
 *
 * - \c robot \c icub \n 
 *   specifies the name of the robot (used to form the root of robot port names)
 *
 *
 * <b>Configuration File Parameters</b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 *
 * 
 * \section portsa_sec Ports Accessed
 * 
 * - None
 *                      
 * \section portsc_sec Ports Created
 *
 *  <b>Input ports</b>
 *
 *  - \c /emPlotter \n
 *    This port is used to change the parameters of the emPlotter at run time or stop the emPlotter. \n
 *    The following commands are available
 * 
 *  -  \c help \n
 *  -  \c quit \n
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other emPlotters but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /emPlotter
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - \c /emPlotter/image:i \n
 *
 * <b>Output ports</b>
 *
 *  - \c /emPlotter \n
 *    see above
 *
 *  - \c /emPlotter/image:o \n
 *
 * <b>Port types</b>
 *
 * The functional specification only names the ports to be used to communicate with the emPlotter 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * \c BufferedPort<ImageOf<PixelRgb> >   \c myInputPort; \n 
 * \c BufferedPort<ImageOf<PixelRgb> >   \c myOutputPort;       
 *
 * \section in_files_sec Input Data Files
 *
 * None
 *
 * \section out_data_sec Output Data Files
 *
 * None
 *
 * \section conf_file_sec Configuration Files
 *
 * \c emPlotter.ini  in \c $ICUB_ROOT/app/emPlotter/conf \n
 * \c icubEyes.ini  in \c $ICUB_ROOT/app/emPlotter/conf
 * 
 * \section tested_os_sec Tested OS
 *
 * Linux
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>emPlotter --name emPlotter --context emPlotter/conf --from emPlotter.ini --robot icub</tt>
 *
 * \author Ajaz Bhat
 *
 * Copyright (C) 2013 RobotCub Consortium\n
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
 * This file can be edited at \c $ICUB_ROOT/main/src/emPlotters/emPlotter/include/iCub/emPlotter.h
 * 
 */


#include <iostream>
#include <string>
#include <sstream> 

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Semaphore.h>
 
//within project includes  
#include <emPlotterRatethread.h>
#include <partialThread.h>
#include <rememberedxThread.h>
#include <hubThread.h>
#include <planThread.h>

class emPlotterModule:public yarp::os::RFModule {
    
    std::string moduleName;                 // name of the module
    std::string robotName;                  // name of the robot 
    std::string robotPortName;              // name of robot port
    std::string inputPortName;              // name of the input port for events
    std::string outputPortName;             // name of output port
    std::string handlerPortName;            // name of handler port
    std::string configFile;                 // name of the configFile that the resource Finder will seek
    
    yarp::os::Port handlerPort;             // a port to handle messages 
    emPlotterRatethread* rThread;           // pointer to a new thread to be created and started in configure() and stopped in... 
                                            // close(). This thread opens input ports and communicates with the other threads
    rememberedxThread*  remThread;          // pointer to a new thread that generates gui of incoming bottles-- from remembered Experiences
    partialThread*      pt[5];              // pointer to a new thread that generates gui of incoming bottles-- from partial cues
    hubThread*          hThread;            // pointer to a new thread that generates gui of incoming bottles-- from hubThread
    planThread*         pThread;            // pointer to a new thread that generates gui of incoming bottles-- from planThread
        
public:
    /**
    *  configure all the emPlotter parameters and return true if successful
    * @param rf reference to the resource finder
    * @return flag for the success
    */
    bool configure(yarp::os::ResourceFinder &rf); 
   
    /**
    *  interrupt, e.g., the ports 
    */
    bool interruptModule();                    

    /**
    *  close and shut down the emPlotter
    */
    bool close();

    /**
    *  to respond through rpc port
    * @param command reference to bottle given to rpc port of module, alongwith parameters
    * @param reply reference to bottle returned by the rpc port in response to command
    * @return bool flag for the success of response else termination of module
    */
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

    /**
    *  unimplemented
    */
    double getPeriod();

    /**
    *  unimplemented
    */ 
    bool updateModule();
};


#endif // _EMPLOTTER_MODULE_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

