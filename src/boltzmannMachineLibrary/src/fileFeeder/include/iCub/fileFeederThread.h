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
 * @file fileFeederThread.h
 * @brief Definition of a thread that reads a txt file and extracts data eventually sent on a port
 * (see fileFeederModule.h).
 */

#ifndef _FILE_FEEDER_THREAD_H_
#define _FILE_FEEDER_THREAD_H_

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/sig/all.h>
#include <iostream>

//OPENCV INCLUDES
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>
#include <fstream>
#include <cstring>
#include <cassert>
#include <cstdio>


class fileFeederThread : public yarp::os::RateThread {
private:
    int count,maxcount;                                              // loop counter of the thread
    int width, height;                                      // dimension of the extended input image (extending)
    int height_orig, width_orig;                            // original dimension of the input and output images
    yarp::os::BufferedPort<yarp::os::Bottle>  outPort;      // port where the output is sent
    IplImage* img;                                          // image read from the file name
    std::string filename;                                   // url of the image to be read
    std::string name;                                       // rootname of all the ports opened by this thread
    bool resized;                                           // flag to check if the variables have been already resized
    std::string filenameStart;                              // first part of the name of images
    std::string filenameEnd;                                // last part of the name of images
    int numberImages;                                       // number of read images
    std::ifstream* file;                                    // file reference
    yarp::sig::Vector samplesE;                             // vector buffer of samples
public:
    /**
    * default constructor
    */
    fileFeederThread();

    /**
     * destructor
     */
    ~fileFeederThread();

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
    * function that set the rootname for the ports that will be opened
    * @param str rootname as a string
    */
    void setName(std::string str);
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /**
    * function that sets the width and the height of the images based on the dimension of the input image
    * @param width width of the input image
    * @return height height of the input image
    */
    void resize(int width, int height);

    /**
    * function that sets the number of images that are going to be read
    * @param numberImage the actual number of images
    */
    void setNumberOfImages(int number) { numberImages=number; };

    /**
    * function that sets the first part of the name of the file
    * @param name string which cointains the first part of the filename
    */
    void setStartFilename(std::string name) { filenameStart=name; };

    /**
    * function that sets the last part of the name of the file
    * @param name string which cointains the last part of the filename
    */
    void setLastFilename(std::string name) {filenameEnd = name; };

};

#endif  //_FILE_FEEDER_THREAD_H_
//----- end-of-file --- ( next line intentionally left blank ) ------------------

