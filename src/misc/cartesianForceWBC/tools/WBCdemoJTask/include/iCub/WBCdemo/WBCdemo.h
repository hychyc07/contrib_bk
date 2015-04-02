// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2011 RBCS ITT
 *
 * Author:  Juan G Victores
 * Contrib: Andrea del Priete (patience, guidelines, chunks of code from skinDyn)
 *          Roland Philippsen (author of original WBC library and examples)
 *          http://cs.stanford.edu/group/manips/
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

#ifndef WBC_DEMO
#define WBC_DEMO

#include <stdio.h>
#include <math.h>
#include <string>
#include <iostream>

#include <yarp/os/RFModule.h>
#include <yarp/os/Module.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
//#include <yarp/dev/CartesianControl.h>

#include <iCub/WBCdemo/WBCthread.h>

#define DEFAULT_ROBOT "icub"
#define DEFAULT_PART "left_arm"

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::math;

class qSetPort : public BufferedPort<Vector> {
protected:
    sharedArea *pMem;
    virtual void onRead(Vector& v) {
//        printf("[Debug] Data arrived on qSetPort\n");
        pMem->setQ(v);
    }
public:
    void setSharedArea(sharedArea* _pMem) {
        pMem = _pMem;
    }
};

class mSetPort : public BufferedPort<Bottle> {
protected:
    WBCthread *pThread;
    virtual void onRead(Bottle& b) {
        if (b.get(0).asString()=="s") {
            pThread->stop();
            printf("Emergency stopped.\n");
        } else if (b.get(0).asString()=="set") {
            pThread->setMode(b.get(1).asInt());
            printf("mode set: %d\n", pThread->getMode());
        } else if (b.get(0).asString()=="get") {
            printf("got mode: %d\n", pThread->getMode());
        } else if (b.get(0).asString()=="verbose") {
            if (b.get(1).asString()=="on") pThread->setVerbose(true);
            else if (b.get(1).asString()=="off") pThread->setVerbose(false);
        }
    }
public:
    void setWBCthread(WBCthread* _pThread) {
        pThread = _pThread;
    }
};

class WBCdemo : public RFModule {
protected:
    yarp::dev::PolyDriver robotDevice;
//    yarp::dev::PolyDriver cartesianDevice;

    yarp::dev::IPositionControl *pos;
    yarp::dev::IControlMode *ictrl;
    yarp::dev::ITorqueControl *itrq;
//    yarp::dev::ICartesianControl *icart;

	qSetPort qPort;  // to connect to robot arm, encoders
	mSetPort mPort;  // to set its mode from outside
    sharedArea mem;

    WBCthread wThread;

    double getPeriod();
    bool updateModule();
    bool interruptModule();
    int period;

public:
    WBCdemo();
    bool configure(ResourceFinder &rf);
};

#endif
