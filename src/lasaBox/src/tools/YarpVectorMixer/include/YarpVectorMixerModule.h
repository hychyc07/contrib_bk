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
\defgroup icub_YarpVectorMixer YarpVectorMixer
@ingroup icub_lasaBox_tools

Utility for mixing several stream of yarp vector into a single output stream. A mixer!!!

\section intro_sec Description

This utility for mixing several stream has some options (see below).
An additiojnal column can be added to get the time.
Additional ports can be created on the fly or be a fixed number.


The only command tha can be used in the terminal of rpc port is 
\verbatim
<return>:    Turn the streaming on/off (if the time column is used, time is reset)
\endverbatim

\section dependencies_sec Dependencies

- YARP

\section parameters_sec Parameters

\verbatim
--name <string>:    the module base name for port creation
--period <double>:  control loop period (0.01 sec by default)
--time:             add a time column to the output stream 
--auto:             by default the outpu streaming is paused, --auto runs it directly
--portNum <int>:    the number of output ports (if you want them for be a fixed number)
                    is not specified, ports are created as they get connected to...
\endverbatim

\section portsc_sec Ports Created

Input ports:

- /YarpVectorMixer/moduleName/rpc : command port 
- /YarpVectorMixer/moduleName/inputXX : input vector port with a number from 00 to ??


Output ports:

- /YarpVectorMixer/moduleName/output : Output streaming port
  


\section in_files_sec Input Data Files

None

\section out_data_sec Output Data Files

None

\section conf_file_sec Configuration Files

None

\section tested_os_sec Tested OS

Linux

\section example_sec Example Instantiation of the Module

\verbatim
YarpVectorMixer --period 0.01 --portNum 2 --auto
\endverbatim

\author Eric Sauser

Copyright (C) 2011 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at $ICUB_ROOT/contrib/src/lasaBox/src/tools/YarpVectorMixer
**/


#ifndef YarpVectorMixerMODULE_H_
#define YarpVectorMixerMODULE_H_

#include <yarp/os/Module.h>

using namespace yarp;
using namespace yarp::os;

#include "YarpVectorMixerThread.h"

class YarpVectorMixerModule: public Module
{
private:
    Property                    mParams;
    double                      mPeriod;
    bool                        bIsReady;

    BufferedPort<Bottle>        mControlPort;

    YarpVectorMixerThread      *mThread;
    
public:
            YarpVectorMixerModule();
    virtual ~YarpVectorMixerModule();


    virtual bool    open(Searchable &s);
    virtual bool    close();
    virtual bool    respond(const Bottle& command, Bottle& reply);
    virtual double  getPeriod();
    virtual bool    updateModule();
    virtual int     runModule(int argc, char *argv[], bool skipFirst = true);
};

#endif

