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
 * @file stereoAttThread.h
 * @brief Definition of a thread that receives images and does the computation for the
 * stereo Attention module (see stereoAttModule.h).
 */

#ifndef _STEREO_ATT_THREAD_H_
#define _STEREO_ATT_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include <iostream>

// outside project includes
//#include <ipp.h>


class stereoAttThread : public yarp::os::RateThread {
private:
    //IppiSize srcsize;                   // ROI for the images in the processing

    int psb;
    int width, height;                  // dimension of the extended input image (extending)
    int shiftvalue;                     //value of the shift of one image with respect to the other
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *tmp; //tmprarily image used to upload the relative buffer
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *inputLeft; //input image reference left
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *inputRight; //input image reference right
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *outImage; //pointer to the output image
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inLeftPort; //a port for the inputLeftImage 
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inRightPort; //a port for the inputRightImage 
    yarp::os::BufferedPort<yarp::os::Bottle> shiftPort; // port that receives how much the shift is
    yarp::os::BufferedPort<yarp::os::Bottle> vergenceInPort; //port used to receive the vergence commands
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outPort; //port whre the output is sent
    yarp::dev::IGazeControl *igaze; //Ikin controller of the gaze
    yarp::dev::PolyDriver* clientGazeCtrl; //polydriver for the gaze controller
    std::string name;       // rootname of all the ports opened by this thread
    bool resized;           // flag to check if the variables have been already resized

public:
    /**
    * default constructor
    */
    stereoAttThread();

    /**
    * constructor
    */
    stereoAttThread(int delay);

    /**
     * destructor
     */
    ~stereoAttThread();

    bool threadInit();
    void threadRelease();
    void run(); 
    void onStop();

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
    *   function that shift the right image and combine the rest with left
    *   @shift number of pixels in the direction of the shift
    *   @outImage image output of the merging
    */
    void shift(int shift, yarp::sig::ImageOf<yarp::sig::PixelRgb>& outImage);

    /**
    *   function that combines left and right images
    *   @outImage image output of the merging
    */
    void fuse(yarp::sig::ImageOf<yarp::sig::PixelRgb>& outImage);
 

};

#endif  //_STEREO_ATT_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

