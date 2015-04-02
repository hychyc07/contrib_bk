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
 * @file shapeSelectorRatethread.h
 * @brief Definition of a thread that receives a bottle from input port and sends the corresponding plans and hubs to the output port.
 */


#ifndef _SHAPESELECTOR_RATETHREAD_H_
#define _SHAPESELECTOR_RATETHREAD_H_
#define CV_IMAGE_ELEM( image_header, elemtype, y, x_Nc )
#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/RateThread.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <cv.h>                     // These header files include openCV libraries
#include <cvaux.h>                  //
#include <highgui.h>                //

class shapeSelectorRatethread : public yarp::os::RateThread {
private:
    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber

    double startTime, endTime;

    //yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inputCallbackPort;
    //yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outputPort;     // output port to plot event

     // input recieving ports  "recieve coordinates of objects' bounding rectangles"                                                         
    yarp::os::BufferedPort<yarp::os::Bottle> inputBottlePort[2];
    
    // ports for recieving and sending image data
    //yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >   inputImagePort[2], outputImagePort[2];
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >    inputImagePort[2], outputImagePort[2];
    //yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outputImagePortLeft;
    //yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outputImagePortRight;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >    outputImagePortLeft, outputImagePortRight;
    
    
    //yarp::sig::ImageOf<yarp::sig::PixelMono> *outputImage,*tempImage, *inputImage[2];
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *outputImage,*tempImage, *inputImage[2];
    yarp::os::Bottle *incomingBottle, *leftBot, *rightBot;
    yarp::os::Semaphore *leftMutex, *rightMutex;
    std::string name;
    int objectCount, xLeft[2][3], yTop[2][3], xWidth[2][3], yHeight[2][3],objectID[2][3],padding;
    int dimX;                              // name of the width of the input
    int dimY;                             // name of the  height of the input
    bool flag;
    int height, width,scaleX,scaleY;
    bool idle,similar;
    unsigned char* oproc, *inproc;
    cv::Mat img0, mask, foreground,temp,temp1;
    IplImage inputIplImage, outputIplImage;
    cv::Rect rect;
public:
    /**
    * constructor default
    */
    shapeSelectorRatethread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    shapeSelectorRatethread(std::string robotname,std::string configFile);

    /**
     * destructor
     */
    ~shapeSelectorRatethread();

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
    * function that sets the rootname of all the ports that are going to be created by the thread
    * @param width ...
    * @param height ...
    */
    void setCoordinates(int w, int h) {dimX = w; dimY = h; } ;
    
    
    /*
    * function that sets the rootname of all the ports that are going to be created by the thread
    * @param width ...
    * @param height ...
    */
    void setOutputDimensions(int w, int h) {width = w; height = h; } ;
    
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
    void setSharingBottle(yarp::os::Bottle *l, yarp::os::Bottle *r);
    
     
    /**
    * Function that adds new objects into the image for output
    */    
    void updateObjects( yarp::sig::ImageOf<yarp::sig::PixelMono> *in, yarp::sig::ImageOf<yarp::sig::PixelMono> *out, int i, int x);


};

#endif  //_EMPLOTTER_RATETHREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

