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
 * @file wingsTranslatorThread.h
 * @brief Definition of a thread that receives events and extracts features
 * (see efExtractorModule.h).
 */

#ifndef _WINGS_TRANSLATOR_THREAD_H_
#define _WINGS_TRANSLATOR_THREAD_H_

#include <iostream>

#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include <iCub/emorph/eventConversion.h>

#include <iCub/iKin/iKinInv.h>
#include <iCub/iKin/iKinIpOpt.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/ctrl/pids.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>

using namespace iCub::ctrl;
using namespace iCub::iKin;

class wingsTranslatorThread : public yarp::os::Thread {
private:
    bool idle;                          // flag that exclude code from the execution loop
    bool isOnWings;                     // indicates when the camera is mounted on the wings
    bool headV2;                        // indicates whether the robot has head version2
    int count;                          // loop counter of the thread
    int width, height;                  // dimension of the extended input image (extending)
    int height_orig, width_orig;        // original dimension of the input and output images
    
    std::string configFile;             // configuration file of cameras (LEFT RIGHT)
    std::string wingsLeftFile;          // complete path for the kinematic of the left cam
    std::string wingsRightFile;          // complete path for the kinematic of the right cam
    yarp::os::BufferedPort<yarp::os::Bottle> inPort;                                     // port where the left event image is received
    yarp::os::BufferedPort<yarp::os::Bottle> inRightPort;                                // port where the right event image is received
    yarp::os::BufferedPort<yarp::os::Bottle> outPort;           // port where the output edge (left) is sent
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outRightPort;      // port where the output edge (right) is sent

    yarp::sig::ImageOf <yarp::sig::PixelMono>* leftInputImage;                           // image input left 
    yarp::sig::ImageOf <yarp::sig::PixelMono>* rightInputImage;                          // image input right 
    std::string robot;                      // name of the robot read by the ResourceFinder
    
    std::string name;                       // rootname of all the ports opened by this thread
    std::string mapURL;                     // mode name and name of the map
    bool resized;                           // flag to check if the variables have been already resized
    int shiftValue;                         // value of the shift between dragonfly (this is vergence related)
    FILE *pFile;                            // file that contains the rules for the LUT
    FILE *fout;                             // file where the extracted LUT is saved
    FILE *fdebug;                           // file for debug
    FILE *fmatch;                           // file for matching
    
    int monBufSize_b;                       // dimension of the bufferFEA in bytes
    int countEvent;                         // counter of event that are going to be sent
    int countMap;                           // counter of the mapped events

    iCub::iKin::iCubEye *eyeL;              // iCubEye object used in the module left
    iCub::iKin::iCubEye *eyeR;              // iCubEye object used in the module right
    iCub::iKin::iCubEye  *ikl;              // support object for temporarely configuration
    iCub::iKin::iCubEye  *ikr;              // support object for temporarely configuration

    yarp::os::Property optionsHead;
    yarp::os::ResourceFinder* rf;                   // resorceFinder reference for configuration files
    yarp::os::Semaphore mutexP0;                    // semaphore for resource p0
    yarp::os::Semaphore mutexN;                     // semaphore for resource n

    yarp::sig::Vector n;                            // vector normal to the plane
    yarp::sig::Vector p0;                           // point belonging to the plane
    yarp::sig::Matrix eyeCAbsFrame;                 // projection matrix center eye
    yarp::sig::Matrix invEyeCAbsFrame;              // inverse projection matrix center eye
    yarp::sig::Matrix *invPrjL, *invPrjR;           // inverse of prjection matrix left and right
    yarp::sig::Matrix *PrjL, *PrjR;                 // projection matrix left and right
    yarp::dev::IGazeControl *igaze;                 // Ikin controller of the gaze
    yarp::dev::PolyDriver* clientGazeCtrl;          // polydriver for the gaze controller
    yarp::dev::PolyDriver *polyTorso, *robotHead;   // polydriver for the control of the head
    yarp::dev::IEncoders *encTorso, *encHead;       // measure of the encoder  (head and torso)
    yarp::dev::IControlLimits *limTorso, *limHead;  // limits in the device

    int originalContext;                    // original context for the gaze Controller
    double blockNeckPitchValue;             // value for blocking the pitch of the neck
    double eyesHalfBaseline;                // half distance between the eyes
    double valueInput[12];                  // vector of 12 values read from the input port
    double cxl,cyl;                         // position of the center of the image left
    double cxr,cyr;                         // position of the center of the image right
    double tableHeight;                     // height of the table
    double xTarget;                         // position of the target on the xAxis
    double yTarget;                         // position of the target on the yAxis

    char* bufferCopy;                       // local copy of the events read
    char* flagCopy;                         // copy of the unreadBuffer
    char* resultCopy;                       // buffer resulting out of the selection
    char* buffer;                           // buffer where the events to send are stored

public:
    /**
    * default constructor
    */
    wingsTranslatorThread();

    /**
     * destructor
     */
    ~wingsTranslatorThread();

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
     * stopping function for the thread
     */
    void onStop();

    /**
    * function that set operating mode
    * @param str name of the mode
    */
    void setMapURL(std::string str) { mapURL = str; };

    /**
    * function that set the rootname for the ports that will be opened
    * @param str rootname as a string
    */
    void setName(std::string str);

    /**
    * function that set the robotname for the ports to which the module connects
    * @param str robotname as a string
    */
    void setRobotName(std::string str) {robot = str; };

    /**
    * function that set the confiFile
    * @param str robotname as a string
    */
    void setConfigFile(std::string str) {configFile = str; };

    /**
     *@brief function that set the z height of the plane
     */
    void setTableHeight(double _height) {tableHeight = _height; printf("wingsTranslatorThread::setTableHeight %f %f \n",_height, tableHeight); };
    
    /**
     * set the reference file where the kinematics chain of the left camera is defined
     */
    void setWingsLeftFile(std::string str) {wingsLeftFile = str; };

    /**
     * set the reference file where the kinematics chain of the right camera is defined
     */
    void setWingsRightFile(std::string str) {wingsRightFile = str; };

    /**
     * function that indicates when the camera is on the wings otherwise default kinematic chain of eyes are used
     */
    void setIsOnWings(bool value) { isOnWings = value; };
    
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
    * shift one image with respect to the other 
    * @param shift number of pixel of shifts
    * @param outImage reference to the output image
    */
    void shift(int shift, yarp::sig::ImageOf<yarp::sig::PixelRgb>& outImage);

    /**
     * @brief simple function that extracts azimuth, elevation and vergence from a 3d position of fixation
     * @param x 3D position of fixation in space using icub convention
     * @return vector of angles
     */
    yarp::sig::Vector getAbsAngles(const yarp::sig::Vector &x);

    /**
     * @brief extracts 3d point point starting from azimuth, elevation and vergence angles.
     * @param type type of trasformation from rel/abs angles
     * @param ang  vector of angles values
     */
    yarp::sig::Vector get3DPoint(const std::string &type, const yarp::sig::Vector &ang); 

    /**
     * @brief function that returns the 3d position of an object on the left camera
     * @param u retina position of the object on the x-axis
     * @param v retina position of the object on the y-axis
     */
    yarp::sig::Vector get3dWingsLeft(int u , int v) ;
    
    /**
     * @brief function that returns the 3d position of an object on the right camera
     * @param u retina position of the object on the x-axis
     * @param v retina position of the object on the y-axis
     */
    yarp::sig::Vector get3dWingsRight(int u , int v) ;

    /**
     * function that sets the target for parameter extraction
     */
    void set3DTarget(double x, double y) { xTarget = x; yTarget = y;};
    
};



// Describe the kinematic of the straight line
// coming out from the point located between eyes.
class iCubHeadCenter : public iCub::iKin::iCubEye {
protected:
    void allocate(const std::string &_type);

public:
    iCubHeadCenter()                                { allocate("right"); }
    iCubHeadCenter(const std::string &_type)        { allocate(_type);   }
    iCubHeadCenter(const iCubHeadCenter &head)      { clone(head);       }
};




#endif  //_TARGET_FINDER_THREAD_H_
//----- end-of-file --- ( next line intentionally left blank ) ------------------
