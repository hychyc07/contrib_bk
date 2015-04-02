// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea
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
 * @file dataCollectorThread.h
 * @brief ratethread that collects images from the input port, preprocessed and stores them into the dataset
 */

#ifndef _DATA_COLLECTOR_THREAD_H_
#define _DATA_COLLECTOR_THREAD_H_


#include <stdio.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Thread.h>
#include <yarp/sig/all.h>

#include <string>

//within project includes
#include <iCub/MachineBoltzmann.h>

/**
* Thread that reads the port and traslate the value into a colour
* The corrispective element in the output image is highlighted proportionally to the value read
* @author Francesco Rea
*/


class dataCollectorThread : public yarp::os::Thread {
private:
    bool inhibit;                                   // inibition flag in order to pause the acquisition
    std::string name;                               // name of the thread and name of the port
    yarp::os::BufferedPort <yarp::sig::ImageOf <yarp::sig::PixelMono> >* sampleImagePort;       //pointer to the sample as image
    yarp::os::BufferedPort <yarp::os::Bottle>* sampleGeneralPort;                               //pointer to the sample as any sort of data
    MachineBoltzmann *mb;                           //pointer to the deep belief network that this thread is going to update

public:
    /**
    * default constructor of the class 
    */
    dataCollectorThread(){};

    /**
    * constructor of the class
    * @param r rate of thread update
    */
    dataCollectorThread(int r){};

    /**
    * constructor of the class
    * @param r rate of thread update
    */
    dataCollectorThread(int r,std::string name){ setName(name); };

    /**
    * initialise the thread
    */
    virtual bool threadInit();

    /**
    * code that is executed after the thread starts
    * @param s is true if the thread started
    */
    virtual void afterStart(bool s);

    /**
    * function whose code is executed only when the thread is asked to stop
    */
    void onStop();
    
    /**
    * running code of the thread
    */
    virtual void run();

    /**
    * code executed when the thread is released
    */
    virtual void threadRelease();

    /**
    * returns the name of the thread
    */
    std::string getName();
    
    /**
    * returns the name of the thread after appending a string to it
    */
    std::string getName(const char* p);

    /**
    * sets the name of the thread
    * @param name string contains the name of the thread
    */
    void setName(std::string name);

    /**
    * function that sets the input image as the input of the dataCollector
    * @param port reference to the port of the input image
    */
    void setInputData(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >* port);

    /**
    * function that sets the reference to the boltzmann machine
    * @param mb reference to Boltzmann Machine
    */
    void setMachineBoltzmann(MachineBoltzmann* mb);

    /** 
     * function that sets the inhibit flag and pauses the acquisition
     */
    void setInhibit(bool value) { inhibit = value; };

};

#endif //_DATA_COLLECTOR_THREAD_H
//----- end-of-file --- ( next line intentionally left blank ) ------------------
