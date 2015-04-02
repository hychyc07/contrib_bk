// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author:  Eric Sauser
 * email:   eric.sauser@a3.epfl.ch
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
 *
\defgroup icub_YarpVectorBag YarpVectorBag
@ingroup icub_lasaBox_tools

Utility for recording, replaying and looping ports streaming yarp vectors values. Inspired from the rosbag utility from ROS.

\section intro_sec Description

In record mode, this module read from the specified ports and save data into a bag file upon exit. 
A bag file is just a tar.gz archive which has been renamed. You may look inside to see...


Rpc commands are
\verbatim
run:     Start plaing, or recording the bag (if option --pause is used)
stop:    Stop :)
cont:    Resume recording or replaying

\endverbatim

\section dependencies_sec Dependencies

- YARP

\section parameters_sec Parameters

\verbatim
--name <string>:    the module base name for port creation
--period <double>:  control loop period (0.01 sec by default)
--rec:              record mode and save a bag
--play:             play bag once mode
--loop:             play bag in looping mode
--bag <string>:     set the bag name
--ports <string>:   the list of ports to be recorded
                    example: --ports "/icub/skin/left_arm /icub/skin/right_arm"
--pause             doesn't start the replay directly (use the commands run/cont/stop 
                    in the terminal for running, pausing, resuming and so on)
--renames <string>: during replay, renames the port specified using the --ports option
                    useful to avoid port conflicts (see instantation examples below)
\endverbatim

\section portsc_sec Ports Created

Input ports:

- /YarpVectorBag/moduleName/rpc : command port 
- /YarpVectorBag/moduleName/onePortName : input vector port that connects to real port onePortName 


Output ports:

- /onePortName : in replay mode, creates the recorded port (when using the rename option, uses the new port name)
  


\section in_files_sec Input Data Files

the bag file

\section out_data_sec Output Data Files

the bag file

\section conf_file_sec Configuration Files

None

\section tested_os_sec Tested OS

Linux only (the code uses /tmp folder to save temporary data, so compatibility with windows is difficult

\section example_sec Example Instantiation of the Module

Records from ports /asd and /wtf at a period of 0.01 sec and save the result in the file my.bag
\verbatim
YarpVectorBag --rec --period 0.01 --ports "/asd /wtf" --bag my.bag
\endverbatim
Replay the bag my.bag
\verbatim
YarpVectorBag --play --bag my.bag
\endverbatim
Loops the bag my.bag but only replay the port /original_port and also renames it to /new_port_name_for_replay 
\verbatim
YarpVectorBag --loop --bag my.bag --ports "/original_port" --renames "/new_port_name_for_replay"
\endverbatim

\author Eric Sauser

Copyright (C) 2011 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at $ICUB_ROOT/contrib/src/lasaBox/src/tools/YarpVectorBag
**/


#ifndef YarpVectorBagMODULE_H_
#define YarpVectorBagMODULE_H_

#include <yarp/os/Module.h>

using namespace yarp;
using namespace yarp::os;

#include "YarpVectorBagThread.h"

class YarpVectorBagModule: public Module
{
private:
    Property                    mParams;
    double                      mPeriod;
    bool                        bIsReady;

    BufferedPort<Bottle>        mControlPort;

    YarpVectorBagThread        *mThread;
    
public:
            YarpVectorBagModule();
    virtual ~YarpVectorBagModule();


    virtual bool    open(Searchable &s);
    virtual bool    close();
    virtual bool    respond(const Bottle& command, Bottle& reply);
    virtual double  getPeriod();
    virtual bool    updateModule();
    virtual int     runModule(int argc, char *argv[], bool skipFirst = true);
};

#endif

