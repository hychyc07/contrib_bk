// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2012 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @file ofThread.h
 * @brief Definition of a thread that every time constant sends commands to the 
 * kinematic gaze controller based on the prioritise requests that have arrived
 * (see ofModule.h).
 */

#ifndef _OPTIC_FLOW_THREAD_H_
#define _OPTIC_FLOW_THREAD_H_

#include <iostream>
#include <string>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/all.h>
#include <yarp/sig/Matrix.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include <cv.h>
#include <highgui.h>

//within project includes
//#include <iCub/trackerThread.h>
#include <iCub/observer.h>
#include <iCub/observable.h>


/**
* thread that given a series of collected optic flow spatial characteristic saves them
* thread replies to queries about subportion of spatial motion field
*/

class ofThread : public yarp::os::RateThread, public observer{
private:
    std::string name;                       // rootname of all the ports opened by this thread
    std::string robot;                      // name of the robot read by the ResourceFinder
    std::string configFile;                 // configuration file of cameras (LEFT RIGHT)
    yarp::sig::Matrix stateTransition;      // matrix of the state transition; weights of the transition
    yarp::sig::Vector stateRequest;         // buffer of requests  (vergence, smooth pursuit, saccade)
    yarp::sig::Vector state;                // vector where just one element can be 1 indicating the state in which the system is
    yarp::sig::Vector allowedTransitions;   // vector of allowed transitions
    yarp::sig::Vector xFix;                 // fixation coordinates
    short numberState;                      // stores the number of the state in which the control can be
    bool done;                              // flag set to true when an gaze action is completed
    bool executing;                         // flag that is set during the execution of motion
    bool firstConsistencyCheck;             // boolean flag that check whether consistency happened
    bool availableVisualCorr;               // flag that indicates whether the visual correction is available
    bool visualCorrection;                  // boolean flag for allowing visual correction of the fine position
    bool isOnWings;                         // flag that gives information on where the cameras are mounted
    bool onDvs;                             // flag for remapping dvs location into standard dimension
    int u,v;                                // values passed for saccades
    int originalContext;                    // original context for the gaze Controller
    double xObject,yObject,zObject;         // coordinates of the object 
    double zDistance;                       // estimated distance of the object from the eye
    double varDistance;                     // calculated distance of the object from the eye 
    double blockNeckPitchValue;             // value for blocking the pitch of the neck
    double xOffset;                         // offset for the 3D point along x
    double yOffset;                         // offset for the 3D point along y
    double zOffset;                         // offset for the 3D point along z
    double xmax, xmin;                      // limits in fixation point
    double ymax, ymin;                      // limits in fixation point
    double zmax, zmin;                      // limits in fixation point
    int width, height;                      // dimension of the image
    int countVerNull;                       // counter of the null vergence angles
    double phi;                             // value passed for vergence
    double phiTOT;                          // accumulator of increments of vergence angles
    bool mono;                              // flag that indicates whether the saccade is mono or not
    bool firstVer;                          // flag check during the vergence that indicates whether eye correction comes after a monoSaccadic event
    bool accomplished_flag;                 // flag for the accomplished vergence
    double timeoutStart,timeoutStop;        // start and stop timing to avoid that saccadic event can stuck
    double timetotStart,timetotStop;        // start and stop timing for the complete fixation task
    double timeout;                         // actual timer of the saccadic action
    double timetot;                         // actual timer of the complete fixation task

    int template_size;                      // size of the template
    int search_size;                        // area over the search is performed

    CvRect  template_roi;                   // region of interest of the template
    CvRect  search_roi;                     // region of interest of the search
    CvPoint point;                          // point result of the search
    
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* imgLeftIn;                                 // input image 3 channel
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* imgRightIn;                                // input mono image
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>  > inLeftPort;       // input image port
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>  > inRightPort;      // output image port
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > templatePort;     // port for the segmented object of the zdf
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > forgettingPort;   // port for the segm
    yarp::sig::ImageOf<yarp::sig::PixelMono> *forgettingImage;                          // image associated to the forgetting factor
    yarp::os::BufferedPort<yarp::os::Bottle> statusPort;                                // port necessary to communicate the status of the system
    yarp::os::BufferedPort<yarp::os::Bottle> timingPort;                                // port where the timing of the fixation point redeployment is sent
    yarp::os::BufferedPort<yarp::os::Bottle> *velocityPort;                                // port where the timing of the fixation point redeployment is sent
    yarp::sig::ImageOf<yarp::sig::PixelMono> *inhibitionImage;                          // image for the inhibition of return
    yarp::os::Port blobDatabasePort;                // port where the novel location in 3d space is sent
    yarp::os::Semaphore mutex;                      // semaphore on the resource stateRequest
    
    double *spatialMemoryU;                         // spatial memory of the u component of the velocity
    double *spatialMemoryV;                         // spatial memory of the v component of the velocity
    //unsigned char *forgettingFactor;                // spatial forgetting factor of the memory                       

    //trackerThread* tracker;                       //reference to the object in charge of tracking a tamplete surrounding a point
public:
    /**
    * default constructor
    */
    ofThread(std::string configFile);

    /**
     * destructor
     */
    ~ofThread();

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
    * function that set the robotname for the ports to which the module connects
    * @param str robotname as a string
    */
    void setRobotName(std::string str);

    /**
    * function that set the variable needed for image dimension
    * @param width set the dimension width of the input image
    * @param height set the dimension height of the input image
    */
    void setDimension(int width, int height);
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /**
    * function that defines what has to be done once any observeble interrupts
    * @param o observable that has just interrupted the observer
    * @param arg Bottle that has passed from the observable to the observer
    */
    void update(observable* o, yarp::os::Bottle * arg);
 
    /**
     * @brief function that returns a velocity for a particular portion of space
     * @param topleftx x coordinate of the top-left vertix of the portion of space
     * @param toplefty y coordinate of the top-left vertix of the portion of space
     * @param bottomrightx x coordinate of the bottom-right vertix of the portion of space
     * @param bottomrighty y coordinate of the bottom-right vertix of the portion of space
     * @param u velocity component of the optic flow
     * @param v velocity component of the optic flow
     */
    void getVelocity(int topleftx, int toplefty, int bottomrightx, int bottomrighty, double& u, double& v);

    /**
     * initialise the process of tracking with the coordinates of the initial point
     */
    void init(int x, int y);

    /**
     * function that performs tracking of a point and its surroundings
     */
    void sqDiff(CvPoint &minloc);


    /**
    * function that sets the value of the parameter xOffset
    */
    void setXOffset(double  value) { xOffset = value; };

    /**
    * function that sets the value of the parameter yOffset
    */
    void setYOffset(double  value) { yOffset = value; };

    /**
    * function that sets the value of the parameter zOffset
    */
    void setZOffset(double  value) { zOffset = value; };

    /**
    * function that sets the value of the parameter x limirs
    */
    void setXLimits(double max, double min) { xmax = max; xmin = min; };

    /**
    * function that sets the value of the parameter y limits
    */
    void setYLimits(double max,double  min) { ymax = max; ymin = min; };


    /**
    * function that sets the value of the parameter onWings
    */
    void setOnWings(int value) { value?isOnWings=true:isOnWings=false; };
    
    /**
    * function that sets the value of the parameter onDvs
    */
    void setOnDvs  (int value) { value?onDvs=true:onDvs=false; };

    /**
    * function that sets the value of the visualCorrection for a visual feedback in saccade
    */
    void setVisualFeedback(bool value) { visualCorrection = availableVisualCorr?value:false; };

    /**
     * function that sets the value head pitch to which the head is blocked
     * @param blockPitch value of the blockPitch
     */
    void setBlockPitch(double blockPitch);

    /**
    * function that sets the value of the parameter z limits
    */
    void setZLimits(double max,double min) { zmax = max; zmin = min; };

    
};

#endif  //_OPTIC_FLOW_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

