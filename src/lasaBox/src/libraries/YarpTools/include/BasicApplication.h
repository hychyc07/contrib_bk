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

#ifndef BasicApplication_H_
#define BasicApplication_H_

#include <yarp/os/Module.h>

using namespace yarp;
using namespace yarp::os;

#include "BasicApplicationThread.h"




class BasicApplication: public Module
{
protected:
    Property                        mParams;
    double                          mPeriod;
    bool                            bIsReady;
    char                            mAppName[256];

    BufferedPort<Bottle>            mControlPort;

    BasicApplicationThread         *mThread;
    
public:
            BasicApplication(const char* name, BasicApplicationThread *basicAppplicationThreadPtr);
    virtual ~BasicApplication();


    virtual bool    open(Searchable &s);
    virtual bool    close();
    virtual bool    respond(const Bottle& command, Bottle& reply);
    virtual double  getPeriod();
    virtual bool    updateModule();
    virtual int     runModule(int argc, char *argv[], bool skipFirst = true);
};


#define APPLICATION_MAIN_FUNCTION (ModuleName,ThreadClass)  int main(int argc, char *argv[]) { \
                                                                Network yarp; \
                                                                if(!yarp.checkNetwork()) return 0; \
                                                                ThreadClass * thread = new ThreadClass(); \
                                                                BasicApplication module(ModuleName,thread); \
                                                                return module.runModule(argc,argv); \
                                                            }


#endif

