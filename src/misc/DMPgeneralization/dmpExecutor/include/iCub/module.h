/*
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff Elena Ceseracciu
 * email:  vadim.tikhanoff@iit.it elena.ceseracciu@iit.it
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

#ifndef DMPEXECUTOR_MODULE_H
#define DMPEXECUTOR_MODULE_H

#include <string>

#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>


#include "iCub/utils.h"
#include "iCub/dmpExecutorInterface.h"
#include "iCub/DMP/DMP.h"
#include "iCub/KinestheticTeaching.h"


class DmpExecutorThread: public yarp::os::RateThread
{
    yarp::os::Property optPD;
    dmp::DMP* currentDMP;
    int initial_context;
    yarp::dev::ICartesianControl *cartCtrl;
    yarp::dev::PolyDriver cartCtrlPD;
    
    std::string name;
    yarp::os::Semaphore mutex;
    
    yarp::os::BufferedPort<yarp::sig::Vector> outputPort;
    
public:
    DmpExecutorThread(int period, yarp::os::Property &opt);
    void setDMP(dmp::DMP* newDMP);
    bool threadInit();
    void threadRelease();
    void run();
    //void stop();
    //void afterStart(bool success);
    
};

/**********************************************************/
class DmpExecutor : public yarp::os::RFModule, iCub::dmpExecutorInterface
{
protected:
    yarp::os::Port              thriftPort;     
    ObjectPropertiesCollectorPort opcPort;
    
    bool attach(yarp::os::Port &source);
    bool run();
    bool is_running();
    bool stop();
    bool s() {return stop();};
    bool execute_OPC(const int32_t id);
    bool waitMotionDone(const double period = 0.5, const double timeout = 0);
    bool set_hand(const iCub::Hand newHand);
    iCub::Hand get_hand();

    bool teach_start(const std::string& actionName, const iCub::Hand handToUse = iCub::RIGHT);
    
    void quit();

    iCub::Hand currentHand;
    iCub::Hand defaultHand;
    dmp::DMP* currentDMP;
    double period;
    
    DmpExecutorThread* execThreadLeft;
    DmpExecutorThread* execThreadRight;
    
    KinTeachingThread* kinTeachLeft;
    KinTeachingThread* kinTeachRight;
    
    yarp::os::Semaphore mutex;

    std::string actionName;
    bool teachingAvailable; //needed??
public:
    DmpExecutor();
    bool    configure(yarp::os::ResourceFinder &rf);
    bool    interruptModule();
    bool    close();
    bool    updateModule();
    double  getPeriod();
    
};
#endif

