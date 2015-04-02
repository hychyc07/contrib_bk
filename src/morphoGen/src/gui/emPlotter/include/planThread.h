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
 * @file planThread.h
 * @brief Definition of a thread that receives a bottle from input port and sends the corresponding plans and hubs to the output port.
 */


#ifndef _PLAN_THREAD_H_
#define _PLAN_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/RateThread.h>
#include <iostream>
#include <fstream>
#include <sstream> 
#include <time.h>


class planThread : public yarp::os::RateThread {
private:
    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber
    std::string name;
    
    
    //yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inputCallbackPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outputPort[2];     // output port to plot event
  
    yarp::os::Bottle  *planA1, *planB1;
    yarp::os::Semaphore *mutexA1, *mutexB1;
    double plan[20][50];            // matrix to represent the hub or plan

    bool idle;                      // flag that indicates whether the thread is active

public:
    /**
    * constructor default
    */
    planThread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    planThread(std::string robotname,std::string configFile);

    /**
     * destructor
     */
    ~planThread();

    /**
    *  initialises the thread
    */
    bool threadInit();

    /**
    *  correctly releases the thread
    */
    void threadRelease();

    /**
    *  active part of the thread
    */
    void run(); 

    /*
    * function that sets the rootname of all the ports that are going to be created by the thread
    * @param str rootnma
    */
    void setName(std::string str);
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /*
    * function that sets the inputPort name
    */
    void setInputPortName(std::string inpPrtName);

    /* 
    * function that receives the input bottles and assigns them to the corresponding thread
    */
    void setSharingBottle(yarp::os::Bottle *c, yarp::os::Bottle *d);
    
    
    /*
    * function that sets the semaphores to the corresponding thread
    */
    void setSemaphore(yarp::os::Semaphore *a, yarp::os::Semaphore *b);

    /*
    * function that updates when every new cue comes in
    */
    void updatePlan(yarp::os::Bottle *queueBottle);
    
    /*
    * function that plots the contents of the cue
    */
    void planPlotting(int i);
};

#endif  //_PLAN_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

