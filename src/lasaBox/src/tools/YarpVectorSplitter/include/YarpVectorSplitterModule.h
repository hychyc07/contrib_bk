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
\defgroup icub_YarpVectorSplitter YarpVectorSplitter
@ingroup icub_lasaBox_tools

Utility for splitting a stream of yarp vector values into smaller bits and to stream these forward.

\section intro_sec Description

\section dependencies_sec Dependencies

- YARP

\section parameters_sec Parameters

\verbatim
--name <string>:    the module base name for port creation
--period <double>:  control loop period (0.01 sec by default)
--splits <string>:  size of the splits (the sum must equal that of the input vector)
                    notation: --splits "20 20 10" (splits the input vector into 3 parts of 20, 20 and 10 units respectively) or
                              --splits 20_20_10" (splits the input vector into 3 parts of 20, 20 and 10 units respectively)
\endverbatim

\section portsc_sec Ports Created

Input ports:


- /YarpVectorSplitter/moduleName/rpc : command port (however it's useless since there's no commands for now)
- /YarpVectorSplitter/moduleName/input : input vector port


Output ports:

- /YarpVectorSplitter/moduleName/outputX : N output streaming ports where X is the port number
  


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
YarpVectorSplitter --period 0.01 --splits 20_10_20
\endverbatim

\author Eric Sauser

Copyright (C) 2011 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at $ICUB_ROOT/contrib/src/lasaBox/src/tools/YarpVectorSplitter
**/


#ifndef YarpVectorSplitterMODULE_H_
#define YarpVectorSplitterMODULE_H_

#include <yarp/os/Module.h>

using namespace yarp;
using namespace yarp::os;

#include "YarpVectorSplitterThread.h"

class YarpVectorSplitterModule: public Module
{
private:
    Property                    mParams;
    double                      mPeriod;
    bool                        bIsReady;

    BufferedPort<Bottle>        mControlPort;

    YarpVectorSplitterThread      *mThread;
    
public:
            YarpVectorSplitterModule();
    virtual ~YarpVectorSplitterModule();


    virtual bool    open(Searchable &s);
    virtual bool    close();
    virtual bool    respond(const Bottle& command, Bottle& reply);
    virtual double  getPeriod();
    virtual bool    updateModule();
    virtual int     runModule(int argc, char *argv[], bool skipFirst = true);
};

#endif

