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
 * @file emPlotterThread.h
 * @brief Definition of a thread that receives a bottle from input port and sends the corresponding plans and hubs to the output port.
 */


#ifndef _EMPLOTTER_RATETHREAD_H_
#define _EMPLOTTER_RATETHREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/RateThread.h>
#include <iostream>
#include <fstream>
#include <time.h>


class emPlotterRatethread : public yarp::os::RateThread {
private:
    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber

    

    //yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inputCallbackPort;
    //yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outputPort;     // output port to plot event
    
                                                               
    yarp::os::BufferedPort<yarp::os::Bottle> inputPortcue, inputPortrem, inputPorthub, inputPortplan; // input recieving ports
    yarp::os::Bottle *cueIncoming, *remIncoming, *hubIncoming, *planIncoming, *pbot[5], *rbot[5], *hbot[5], *hubTop, *hubBottomAll, *planA, *planB;
    yarp::os::Semaphore *pmutex[5], *rmutex[5],*hmutex[5], *mutexTop, *mutexBottomAll, *mutexA, *mutexB;
    std::string name;
    bool idle;

public:
    /**
    * constructor default
    */
    emPlotterRatethread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    emPlotterRatethread(std::string robotname,std::string configFile);

    /**
     * destructor
     */
    ~emPlotterRatethread();

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
    void setSharingBottle(yarp::os::Bottle *bottle1[], yarp::os::Bottle *bottle2[], yarp::os::Bottle *bottle3[], yarp::os::Bottle * a,  yarp::os::Bottle *b, yarp::os::Bottle *c, yarp::os::Bottle *d);
    /*
    * Function that associates semaphores to the correct bottles
    */
    void setSemaphore(yarp::os::Semaphore *mutex1[], yarp::os::Semaphore *mutex2[], yarp::os::Semaphore *mutex3[], yarp::os::Semaphore *a, yarp::os::Semaphore *b, yarp::os::Semaphore *c, yarp::os::Semaphore *d);



};

#endif  //_EMPLOTTER_RATETHREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

