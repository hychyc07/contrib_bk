// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.reak@iit.it
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
 * @file detectorThread.h
 * @brief Definition of a thread that receives an RGB image from input port and extract learnt shapes
 */


#ifndef _DETECTOR_THREAD_H_
#define _DETECTOR_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/Thread.h>
#include <TLdetector.hpp>
#include <iostream>
#include <fstream>
#include <cstring>
#include <time.h>


class detectorThread : public yarp::os::Thread {
private:
    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber

    yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputImage;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *imageInputPort;         // input port for image input stream
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > *imageOutputPort;        // output port for the image output stream
    yarp::os::BufferedPort<yarp::os::Bottle >                        bbOutputPort;      // port for the output of the info
    yarp::os::BufferedPort<yarp::os::Bottle >                        dataPortMec;       // port for the output of the features out    

    std::string name;                                                                   // rootname of all the ports opened by this thread
    std::string para_yml_file;     // file contains training data
    std::string tr_bin_file;       // file that contains the training data

    CTLdetector* detector;         // reference to the class CTLdetector
    
    bool firstRun;                 // first run for initialization

public:
    /**
    * constructor default
    */
    detectorThread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    detectorThread(std::string robotname,std::string configFile);

    /**
     * destructor
     */
    ~detectorThread();

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

    /*
    * function that sets the parametric file path
    * @param str path of the file
    */
    void setParaFile(std::string path) {para_yml_file = path; };

    /*
    * function that sets the training file path
    * @param str path of the file
    */
    void setTrainFile(std::string path) {tr_bin_file = path; };
    
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

    /*function called in the event of closing for the thread*/
    void onStop();

};

#endif  //_DETECTOR_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

