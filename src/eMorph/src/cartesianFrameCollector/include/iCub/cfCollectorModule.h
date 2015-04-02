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
 * @file cfCollectorModule.h
 * @brief A module that read independent asynchronous events from a yarp port and represents them as an image
 */

#ifndef _CF_COLLECTOR_MODULE_H_
#define _CF_COLLECTOR_MODULE_H_

/** 
 *
 * \defgroup icub_cartesianFrameCollector cartesianFrameCollector
 * @ingroup icub_eMorph
 *
 *
 * This is a module that reads independent event-driven response to changes in the luminance sensed by DVS cameras.
 * These events are present on a port, they have to be collected during the frame time costant and visualised on an image.
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
 * \section Description
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
 * - \c from \c cartesianFramCollector.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c dvsGrabber/conf \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c cartesianFrameCollector \n 
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * - \c robot \c icub \n 
 *   specifies the name of the robot (used to form the root of robot port names)
 * 
 * - \c asvMode \n
 *   activates the ASV mode
 *
 * - \c dvsMode \n
 *   acrivates the DVS mode
 * 
 * - \c stereo
 *   enables the right output as well and analyses timestamp coming from another camera
 * 
 * - \c retinalSize \c 128 \n 
 *   set the dimension of chip to define the size of the representation 
 *
 * - \c responseGradient \c 127 \n 
 *   set the dimension of pixel gradient in the image as response of any received event 
 *
 * - \c tristate \n 
 *   option that allows the representation to be mapped to 3 level -127,0, 127 
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
 *  - \c /cartesianFrameCollector \n
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
 *  - \c /cartesianFrameCollector/image:i \n
 *
 * <b>Output ports</b>
 *
 *  - \c /cartesianFrameCollector \n
 *    see above
 *
 *  - \c /cartesianFrameCollector/image:o \n
 *
 * <b>Port types</b>
 *
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
 * \c cartesianFrameCollector.ini  in \c $ICUB_ROOT/app/cartesianFrameCollector/conf \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows, Linux
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>cartesianFrameCollector --name cartesianFrameCollector --context cartesianFrameCollector/conf --from cartesianFrameCollector.ini --robot icub</tt>
 *
 * \author Rea Francesco
 *
 * Copyright (C) 2010 RobotCub Consortium\n
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
 * This file can be edited at \c $ICUB_ROOT/main/src/modules/cartesianFrameCollector/include/iCub/cfCollectorModule.h
 * 
 */


/**
 * \section change_log CHANGE LOG 
 * 
 * 25/07/11 : added common interface between DVS on USB and AEX on IHEAD                                                author : Rea \n
 * 30/07/11 : corrected the circular buffer using three thresholds                                                      author : Rea \n 
 * 12/09/11 : added the stereo flag for stereo synchronization                                                          author : Rea \n
 * 19/09/11 : added logpolar representation of the read coordinates                                                     author : Rea \n
 * 21/09/11 : added the VERBOSE mode for plotting out debugging messages                                                author : Rea \n
 * 23/09/11 : added parameter in order to define retina dimension                                                       author : Rea \n      
 * 21/11/11 : made the synch_period a parameter rather than hardcoded variable                                          author : Rea \n
 * 09/02/12 : improved the user interface                                                                               author : Rea \n
 * 22/02/12 : added extra port for handling events sent as bottle                                                       author : Rea \n
 * 04/09/12 : added the windowSize for the collection of events                                                         author : Rea \n
*/

#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>

//within project includes
#include <iCub/cfCollectorThread.h>

class cfCollectorModule:public yarp::os::RFModule {
    std::string moduleName;                     // name of the module (rootname of ports)
    std::string robotName;                      // name of the robot
    std::string robotPortName;                  // reference to the head of the robot
    std::string handlerPortName;                // name of the handler port (comunication with respond function)
    int ratethread;                             // time constant for ratethread
    int synchPeriod;                            // synchronization period between events and viewer
    int retinalSize;                            // dimension of the retina
    int windowSize;                             // dimension of temporal window for gathering events 
    int responseGradient;                       // amplitude of the response gradient for every event in the image
    yarp::os::Port handlerPort;                 // a port to handle messages 
    cfCollectorThread* cfThread;                // cfCollectorThread for processing events

public:
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply); 
    bool updateModule();
};


#endif // __CF_COLLECTOR_MODULE_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

