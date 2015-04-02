// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2012  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.rea@iit.it
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
 * @file velocityExtractorThread.h
 * @brief Definition of a thread that extracts a SMOOTH_PURSUIT command from the optical flow
 * (see bottleRepeaterModule.h).
 */

#ifndef _VELOCITY_EXTRACTOR_THREAD_H_
#define _VELOCITY_EXTRACTOR_THREAD_H_

#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <iostream>
#include <fstream>
#include <time.h>

#include <iCub/plotterThread.h>
#include <iCub/velocityBottleHandler.h>

#define NUMANGLES 360    
#define PI        3.1415

typedef yarp::os::NetUint32 u32;

class velocityExtractorThread : public yarp::os::Thread {
private:    
    static const int    numberOfAngles = 36;
    static const int    umin           = 32;
    static const int    vmin           = 32;
    static const int    umax           = 96;
    static const int    vmax           = 96;
    static const int    maxFiringRate  = 120;
    static const int    medianDim      = 5;

    double alfa;
   
    int count;                          // loop counter of the thread
    int countMedian;                  // counter of the median
 
    //struct timeval tvstart,tvend;
    //struct timespec start_time, stop_time;
    
    unsigned long precl;
    unsigned long lc;
    unsigned long lcprev;
    unsigned long rcprev;
    unsigned long rc;
    
    double microseconds;
    double microsecondsPrev;
    double egoMotionU;                  // component of self motion on U
    double egoMotionV;                  // component of self motion on V
    int countStop;                      // counter of equal timestamp
    int countDivider;                   // divider of the count
    int retinalSize;                    // dimension of the retina device
    int width, height;                  // dimension of the extended input image (extending)
    int height_orig, width_orig;        // original dimension of the input and output images
    int synchPeriod;                    // synchronization period between events and viewer
    int responseGradient;               // responseGradient parameter
    int velWTA_direction;               // direction of the winning speed vector
    int countSent;                      // counter of the command sent
    double velWTA_magnitude;            // magnitude of the winning speed vector
    short  histogram[numberOfAngles];   // histogram of velocity direction
    short  counter  [numberOfAngles];   // counter of the direction
    double magnitude[numberOfAngles];   // store memory for the mean magnitude of direction
    double sumHist;                     // sum of all the values in the histogram in this packet
    double meanHist;                    // mean value of the histogram
    double medianVector[medianDim];     // list of value where the median is extracted
    double medianMag[medianDim];        // magnitude of the value in the medianVector
    
    yarp::os::Bottle* receivedBottle;      // bottle currently extracted from the buffer
    yarp::os::Bottle* bottleToSend;        // bottle ready to be sent to the outputport 
                                
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outImagePort;            // port whre the output (left) is sent
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outPortRight;       // port whre the output (right) is sent
    yarp::os::BufferedPort<yarp::os::Bottle>                          outBottlePort;     
    yarp::os::BufferedPort<yarp::os::Bottle>                          inBottlePort;
    yarp::sig::ImageOf<yarp::sig::PixelMono>* imageOut;                                  //image representing the signal on the leftcamera
    yarp::sig::ImageOf<yarp::sig::PixelMono>* imageRight;                                 //image representing the signal on the right camera
    
    std::string name;                    // rootname of all the ports opened by this thread
    bool suspendFlag;                    // flag that suspend the computation
    bool verb;
    bool synchronised;                   // flag to check whether the microsecond counter has been synchronised
    bool greaterHalf;                    // indicates whether the counter has passed the half of the range
    bool idle;                           // controls idle mode
    bool firstRun;                       // flag that check whether the run is a first useful run    
    bool logPolar;                       // flag that indicates whether the viewer represent logpolar information
    bool stereo;                         // flag that indicates whether the synchronization is stereo
    bool asvFlag, dvsFlag;               // flag for operating mode
    bool tristate;                       // option that represent the image with three baselines
    bool maxReached;                     // indicates when a new max is present in the memory
    bool firstCycle;                     // flag that indicates the first cycle after resume for time::delay

    unsigned long minCount;              // minimum timestamp allowed for the current frame
    unsigned long maxCount;              // maximum timestamp allowed for the current frame
    unsigned long minCountRight;
    unsigned long maxCountRight;

    double startTimer;
    double interTimer;
    double endTimer;    
    
    yarp::os::Semaphore mutex;           // semaphore thar regulates the access to the buffer resource
    clock_t endTime,startTime;
    long T1,T2;

    velocityBottleHandler* vbh;          // handler for packets of velocity information  
    plotterThread* pt;
    
    char* bufferRead;                    // buffer of events read from the port
    char* bufferCopy;                    // local copy of the events read
    FILE* fout;                          // file for temporarely savings of events
    FILE* raw;                           // file dumper for debug
public:
    /**
    * default constructor
    */
    velocityExtractorThread();

    /**
     * destructor
     */
    ~velocityExtractorThread();

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
    * function called when the thread is stopped
    */
    void onStop();

    /**
     * function that suspends the execution 
     */
    void suspend(){suspendFlag = true; };

    /**
     * function that resumes the execution
     */
    void resume(){firstCycle = true;  suspendFlag = false; };
    
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
    * @brief returns a mono image of the output of the dvs camera (either left or right)
    * @param pixelMono reference to the image contains the counts of events
    * @param minCount reference to the min timestamp in the frame
    * @param maxCount reference to the max timestamp in the frame
    * @param camera reference to the camera the image belongs LEFT 1, RIGHT 1
    */
    void getMonoImage(yarp::sig::ImageOf<yarp::sig::PixelMono>* image, unsigned long minCount,unsigned long maxCount, bool camera);

    /**
     * @brief function that describes whether the synchronization is stereo
     * @param value boolean value to assign to the variable
     */
    void setStereo(bool value) {stereo = value; };

    /**
     * @brief function thatset the dimension of the output image
     * @param value the dimension in pixels of the retina device
     */
    void setRetinalSize(int value) {
        retinalSize = value;
    }

    /**
     * @brief function that set the flag for the ASV chip
     * @param value value to assign to the flag
     */
    void setASVMode(bool value) {
        asvFlag = value;
        printf("setting ASVMode \n");
        //unmask_events->setASVMode(value);
        printf("success in setting ASVMode \n");
    } 

    /**
     * @brief function that set the flag for the portable DVS chip
     * @param value value to assign to the flag
     */
    void setDVSMode(bool value) {
        dvsFlag = value;
        //unmask_events->setDVSMode(value);
    } 

    /**
     * @brief function that set the option that maps events in three states
     * @param value value to assign to the flag
     */
    void setTristate(bool value) {
        tristate = value;
        //unmask_events->setDVSMode(value);
    } 

    /**
     * @brief function that sets the synchronization period between the events and viewer
     * @param value integer representing the synchronization period (minim. 1 runcycle)
     */
    void setSynchPeriod(int value) {synchPeriod = value; };

    /**
     * @brief function that indicates whether the viewer reppresent logpolar information
     */
    void setLogPolar(int value) {logPolar = value; };

    /**
     * @brief function that indicates whether the viewer reppresent logpolar information
     */
    void setResponseGradient(int value) {responseGradient = value; }; 

    /**
     * @brief function that sets the valeus of the egoMotion components on the retina
     */
    void setEgoMotion(double egoU, double egoV) {egoMotionU = egoU; egoMotionV =  egoV; }; 
    
    
    /**
     * @brief function that given a section of the buffer creates a bottle
     */
    int prepareUnmasking(char* bufferCopy, yarp::os::Bottle* res);

    /**
     * @brief function that prepare the histovalues for the plotter and sends the pointer to the plotter
     * @param mean value of the histogram
     */
    void setHistoValue();

    /**
     * @brief that reduces the response along with the time
     */
    void decayingProcess();
    
};

#endif  //_VELOCITY_EXTRACTOR_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

