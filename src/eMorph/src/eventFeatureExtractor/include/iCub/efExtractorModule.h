// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.rea@iit.it
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
 * @file efExtractorModule.h
 * @brief A module that lauches the eventExtractorThread handling all the user commands
 */

#ifndef _EF_EXTRACTOR_MODULE_H_
#define _EF_EXTRACTOR_MODULE_H_

/** 
 *
 * \defgroup icub_efExtractor efExtractor
 * @ingroup icub_eMorph
 *
 *
 * This is a module that receives events and extract features using the policy set by the user
 * If the policy requires a look-up-table to redirect event in the right image the module looks for the map in the conf section of the application
 * 
 * \section reference
 * The address-event representation communication protocol AER 0.02, Caltech, Pasadena, CA, Internal Memo, Feb. 1993 [Online]. Available:
 * http://www.ini.uzh.ch/~amw/scx/std002.pdf
 * 
 * S. R. Deiss, T. Delbr�ck, R. J. Douglas, M. Fischer, M. Mahowald, T. Matthews, and A. M. Whatley, Address-event asynchronous local broadcast protocol, Inst. Neuroinform., Zurich, Switzerland, 1994 [Online].
 * Available: http://www.ini.uzh.ch/~amw/scx/aeprotocol.html
 * 
 * A. M. Whatley, PCI-AER Board Driver, Library & Documentation, Inst. Neuroinform., Zurich, Switzerland, 2007 [Online]. Available:
 * http://www.ini.uzh.ch/~amw/pciaer/
 * 
 * S. R. Deiss, R. J. Douglas, and A. M. Whatley, "A pulse-coded communications infrastructure for neuromorphic systems", in Pulsed Neural Networks, W. Maass and C. M. Bishop, Eds. Cambridge, MA: MIT Press, 1998, ch. 6, pp. 157�178.
 * 
 * V. Dante, P. Del Giudice, and A. M. Whatley, �PCI-AER�hardware and software for interfacing to address-event based neuromorphic systems,� The Neuromorphic Engineer vol. 2, no. 1, pp.
 * 5�6, 2005 [Online]. Available: http://ine-web.org/research/newsletters/index.html
 * 
 * 
 * 
 * \section Description
 * DVS cameras extract event from the scene, these events are represented as images. DVS images coming from the ocular system in the iCub have to be
 * aligned with images produced by traditional cameras (dragonfly). However these cameras are located in a different position in the iCub's head.
 * This module defines the geometry of the alignment and puts together the different stereo images.
 *
 * \section lib_sec Libraries
 *
 * YARP.
 *
 * \section parameters_sec Parameters
 * 
 * <b>Command-line Parameters</b> 
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - \c from \c efExtractor.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c logPolarAttentionSystem/conf \n
 *   specifies the sub-path from \c /app to the configuration file
 *
 * - \c name \c efExtractor \n 
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * - \c robot \c icub \n 
 *   specifies the name of the robot (used to form the root of robot port names)
 * 
 * - \c mode \c intensity,horizontal66,vertical66
 *
 * - \c verbose 
 *    indicates if the events must be dumped in debug files
 *
 * <b>Configuration File Parameters</b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
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
 *  - \c /efExtractor \n
 *    This port is used to change the parameters of the module at run time or stop the module. \n
 *    The following commands are available
 * 
 *  -  \c help \n
 *  -  \c quit \n
 *  
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /cartesianFrameCollector
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - \c /efExtractor/dvsLeft:i \n
 *  - \c /efExtractor/dvsRight:i \n
 *  - \c /efExtractor/dragonLeft:i \n
 *  - \c /efExtractor/dragonRight:i \n
 *
 * <b>Output ports</b>
 *
 *  - \c /efExtractor \n
 *    see above
 *
 *  - \c /efExtractor/image:o \n
 *
 * <b>Port types</b>
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
 * \c efExtractor.ini  in \c /eMorphApplication/conf \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows, Linux
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>efExtractor --name /efExtractor --context logPolarAttentionSystem/conf --from efExtractor.ini --robot icub</tt>
 *
 * \author Rea Francesco
 *
 * Copyright (C) 2011 RobotCub Consortium\n
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
 * 
 */

/**
* /section change_log CHANGE LOG
* 24/08/11 : created the module                                                                       author: Rea \n
* 31/08/11 : added parameter mode for the command line                                                author: Rea \n 
* 13/09/11 : added the eventBuffer that contains the collection of unmasked events                    author: Rea \n
* 07/11/11 : corrected the output for debug images                                                    author: Rea \n
* 08/11/11 : added lines that send the feature events on the yarp network                             author: Rea \n
* 21/02/12 : added interactive verbose mode                                                           author: Rea \n
* 27/02/12 : added a new thread for Bottle Handling                                                   author: Rea \n
* 02/03/12 : added fuction for bottle unmasking                                                       author: Rea \n
*/

#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>

//within project includes
#include <iCub/efExtractorThread.h>

class efExtractorModule:public yarp::os::RFModule {
    std::string moduleName;                     //name of the module (rootname of ports)
    std::string robotName;                      //name of the robot
    std::string robotPortName;                  //reference to the head of the robot
    std::string handlerPortName;                //name of the handler port (comunication with respond function)
    std::string mapName;                        //name of the operating mode corresponds to the map
    std::string mapNameComplete;                //name of complete of the map 
    int ratethread;                             //time constant for ratethread

    //yarp::os::Port handlerPort;                 // a port to handle messages 
    yarp::os::RpcServer handlerPort;            // a port to handle messages 
    efExtractorThread* efeThread;               //efExtractorThread for processing events

public:
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    bool updateModule();
};


#endif // _EF_EXTRACTOR_MODULE_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------
