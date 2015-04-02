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
 * @file prioCollectorThread.h
 * @brief RateThread which collects gaze request from the lower level as commands and foward those to the arbiter
 * 
 */

#ifndef _PRIO_COLLECTOR_THREAD_H_
#define _PRIO_COLLECTOR_THREAD_H_

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/sig/all.h>
#include <iostream>
#include <string>

//within project includes
#include <iCub/observer.h>
#include <iCub/observable.h>

#define MAXCOUNTERMOTION  20    // counter for resetting of magnocellular response suppression
#define MAXCOUNTERMOTIONP  21    // counterMotion

class prioCollectorThread : public yarp::os::Thread, public observable{
private:
    bool reinit_flag;
    bool idle;
    std::string name;       // rootname of all the ports opened by this thread
    yarp::os::BufferedPort<yarp::os::Bottle> inCommandPort;                              // port where all the low level commands are sent
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >* map1Port;         // port for the image of contrast  
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >* map2Port;         // port for the image of motion 
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >  testPort;         // port for testing  
    int timing;
    int k1,k2;                                                                           // weights   
    int width, height;                                                                   // image dimension 
    yarp::sig::ImageOf<yarp::sig::PixelMono> *map1_yarp, *map2_yarp;                     // contrast and motion map
    yarp::sig::ImageOf<yarp::sig::PixelMono> *linearCombinationImage;

public:
    /**
    * default constructor
    */
    prioCollectorThread();

    /**
     * destructor
     */
    ~prioCollectorThread();

    /**
     * function that reinitiases some attributes of the class
     */
    void reinitialise(int width, int height);

    /**
    * function that initialise the thread
    */
    bool threadInit();

    /**
    * function called when the thread is stopped
    */
    void threadRelease();

    /**
    * function called every time constant defined by rateThread
    */
    void run(); 

    /**
    * function called when the module is poked with an interrupt command
    */
    void interrupt();

    /**
    * function which is automatically executed when the stop function of the thread is called
    */
    void onStop();

    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /**
    * function that set the rootname for the ports that will be opened
    * @param str rootname as a string
    */
    void setName(std::string str);
    
    /**
     * set the reference to the contrast map port
     */
    void setContrastMap(yarp::sig::ImageOf<yarp::sig::PixelMono> * ref){
        map1_yarp = ref;
        width = ref->width(); 
        height = ref->height();
    };

    
    /**
     * function that sets reference to the motion map port
     */       
    void setMotionMap(yarp::sig::ImageOf<yarp::sig::PixelMono> * ref) {
        map2_yarp = ref;
    };

    void setLinearMap(yarp::sig::ImageOf<yarp::sig::PixelMono>* ref ) {
        linearCombinationImage = ref;
    };
    

    /**
     * function that navigates in the logpolar image looking for maxima
     * @param map1 map coming from contrast feature map
     * @param map2 map coming from motion feature map
     * @param linearCombination combinationof the linear maps
     */
    bool earlyFilter(yarp::sig::ImageOf<yarp::sig::PixelMono>* map1,yarp::sig::ImageOf<yarp::sig::PixelMono>* map2, yarp::sig::ImageOf<yarp::sig::PixelMono> linearCombination, double& xm, double& ym );


};

#endif  //_PRIO_COLLECTOR_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

