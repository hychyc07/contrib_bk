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
 * @file imageThread.h
 * @brief main code for having a thread that sends images of the layers
 */

#ifndef _IMAGETHREAD_H_
#define _IMAGETHREAD_H_


#include <stdio.h>

//within project includes
#include <iCub/YARPImgRecv.h>

//bml library includes
#include <iCub/MachineBoltzmann.h>
#include <iCub/Layer.h>

//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//openCV include
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>


#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Thread.h>

#include <string>


/**
* Thread that reads the port and traslate the value into a colour
* The corrispective element in the output image is highlighted proportionally to the value read
* @author Francesco Rea
*/


class imageThread : public yarp::os::RateThread {
public:
    /**
    * default constructor of the imageThread class 
    */
    imageThread() : yarp::os::RateThread(50){};
    /**
    * constructor of the imageThread class
    * @param r rate of thread update
    */
    imageThread(int r) : yarp::os::RateThread(r){};
    /**
    * constructor of the imageThread class
    * @param r rate of thread update
    */
    imageThread(int r,string name) : yarp::os::RateThread(r){setName(name);};
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
    * sets the name of the thread
    * @param name string contains the name of the thread
    */
    void setName(std::string name);
    /** 
    * set the attribute options of class Property
    * @param options option to be set
    */
    void setOptions(yarp::os::Property options); //set the attribute options of class Property
    /** 
    * function that istantiate the TrackerThread
    * @param property of the thread
    */
    void istantiateThread(yarp::os::Property options); //set the attribute options of class Property
    
    /** 
    * function that sets the scaleFactorX
    * @param value new value of the scaleFactorX
    */
    void setScaleFactorX(int value);
    
    /** 
    * function that sets the scaleFactorY
    * @param value new value of the scaleFactorY
    */
    void setScaleFactorY(int value);
    
    /** 
    * function that set the number of the layer active 
    * @param value number of the layer actually active
    */
    void setCurrentLayer(int value);
    
    /** 
    * function that set and reset the boolean for the control of just the eyes
    * @param value of the flag to be set
    */
    void setJustEyes(bool value);
    
    /** 
    * function that set the  vector of the tracker thread
    * @param a first element of the vector
    * @param b second element of the vector
    * @param c third element of the vector
    */
    void setTrackerVector(double a, double b, double c);
    /** 
    * function that set the left vector of the tracker thread
    */
    void setTrackerLeftVector();
    /** 
    * function that set the right vector of the tracker thread
    */
    void setTrackerRightVector();
    /** 
    * function that stop the tracker where it is
    */
    void stopTracker();
    /**
    * function that sets the pointer to the related layer
    */
    void setLayer(Layer* layer);
    /**
    * returns the yarp image
    */
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* getYarpImage();

    yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputImage;              // input Image which is mapped onto the selected layer
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputImage2;             // input Image which is mapped onto the selected layer
    int countLayer;                                 // number of layers already istantiated
    Layer* plottedLayer;                            // layer which the image is represented
    yarp::os::Property options;                               // option for the tracker thread
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > statePort;     // input port for the representation of the state
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > weightPort;    // input port for the representation of the weights between this layer and the next
    yarp::os::BufferedPort<yarp::os::Bottle> portCmd;         // port where the commands are vehiculated from controller to engine
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > port_plane;   // port plane
    int ct;                                         // counter for the update step
    int scaleFactorX;                               // scale factor for the output image representing a layer (X axis)
    int scaleFactorY;                               // scale factor for the output image representing a layer (Y axis)
    int currentLayer;                               // sinchronized with the number of layer active
    int count;                                      // counter incremented inside the updateModule
    yarp::sig::ImageOf<yarp::sig::PixelRgb> img_tmp;                      // temporary image
    yarp::sig::ImageOf<yarp::sig::PixelRgb> img; 
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *img0;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *image2;                      // image where the state of the layer will be represented
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *imageWeights;                // image where the weights of the connection between this layer and the upper layer will be represented
    IplImage *cvImage;                              // openCv image where all the objects are drawn sequentially
    bool enableDraw;                                // flag that enable the drawing of the layer present in the simulation
    std::string name;                               // name of the thread and name of the port
};

#endif //_IMAGETHREAD_H
//----- end-of-file --- ( next line intentionally left blank ) ------------------
