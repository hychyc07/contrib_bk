// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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
 * @file eventSelectorThread.h
 * @brief Definition of a thread that receive events from DVS camera and compute the saliency map
 * (see eventSelectorModule.h).
 */

#ifndef _EVENT_SELECTOR_THREAD_H_
#define _EVENT_SELECTOR_THREAD_H_

#include <iostream>
#include <fstream>
#include <time.h>
//#include <sys/time.h>
//#include <sys/types.h>
//#include <inttypes.h>
#include <stdlib.h>

//yarp includes
#include <yarp/os/RateThread.h>
#include <yarp/sig/all.h>

#include <iCub/emorph/eventCodec.h>

//within the project includes
#include <iCub/eventCartesianCollector.h>
#include <iCub/plotterThread.h>
#include <iCub/eventBottleHandler.h>
#include <iCub/bottleProcessorThread.h>

//typedef unsigned long long int uint64_t;
#define u64 long


class eventSelectorThread : public yarp::os::RateThread {
private:
    
    int count;                          // loop counter of the thread
    //struct timeval tvstart,tvend;
    //struct timespec start_time, stop_time;
    u64 Tnow;
    unsigned long precl;
    unsigned long lc;
    unsigned long lcprev;
    unsigned long rcprev;
    unsigned long rc;
    
    double forgettingFactor;            // forgetting factor subtracted to pixel of feature map [0.0,1.0]
    double microseconds;
    double microsecondsPrev;
    int countStop;                      // counter of equal timestamp
    int countDivider;                   // divider of the count
    int countCommands;                  // counter of sent commands; 
    int retinalSize;                    // dimension of the retina device
    int width, height;                  // dimension of the extended input image (extending)
    int height_orig, width_orig;        // original dimension of the input and output images
    int synchPeriod;                    // synchronization period between events and viewer
    int responseGradient;               // responseGradient parameter
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outPort;           // port whre the output (left) is sent
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outPortRight;      // port whre the output (right) is sent
    yarp::sig::ImageOf<yarp::sig::PixelRgb>*  imageLeft;                                  //image representing the signal on the leftcamera
    yarp::sig::ImageOf<yarp::sig::PixelRgb>*  imageRight;                                 //image representing the signal on the right camera
    yarp::sig::ImageOf<yarp::sig::PixelMono>* imageLeftBW;                               //image representing the signal on the leftcamera
    yarp::sig::ImageOf<yarp::sig::PixelMono>* imageRightBW;                              //image representing the signal on the right camera
    
    std::string name;                   // rootname of all the ports opened by this thread
    bool verb;
    bool synchronised;                  // flag to check whether the microsecond counter has been synchronised
    bool greaterHalf;                   // indicates whether the counter has passed the half of the range
    bool idle;                          // controls idle mode
    bool firstRun;                      // flag that check whether the run is a first useful run    
    bool logPolar;                      // flag that indicates whether the viewer represent logpolar information
    bool stereo;                        // flag that indicates whether the synchronization is stereo
    bool asvFlag, dvsFlag;              // flag for operating mode
    bool tristate;                      // option that represent the image with three baselines
    bool bottleHandler;                 // flag that indicates whether events are sent as bottle exclusively
    unsigned long minCount;             // minimum timestamp allowed for the current frame
    unsigned long maxCount;             // maximum timestamp allowed for the current frame
    unsigned long minCountRight;
    unsigned long maxCountRight;
    unsigned long* lasttimestamp;       // timestamp of the last event represented

    emorph::ecodec::eEventQueue* txQueue;  // queue of event to be sent
    emorph::ecodec::eEventQueue* rxQueue;  // queue of event to be sent

    double maxDistance;                  // distance from the center of the WTA 
    double startTimer;
    double interTimer;
    double endTimer;
    yarp::os::Semaphore mutex;           // semaphore thar regulates the access to the buffer resource
    clock_t endTime,startTime;
    long T1,T2;
    
    unmask* unmask_events;               // object that unmask events
    char* bufferRead;                    // buffer of events read from the port
    char* bufferCopy;                    // local copy of the events read
    FILE* fout;                          // file for temporarely savings of events
    FILE* fstore;                        // file for the saving of wta position with timestamps
    FILE* istore;                        // file for the saving of ini
    FILE* raw;                           // file dumper for debug
    
    double* saliencyMapLeft;             // saliencyMap of the left camera
    double* saliencyMapRight;            // saliencyMap of the right camera
    double* featureMap41Left;            // 1 feature map from the type 4 (left):
    double* featureMap41Right;           // 1 feature map from the type 4 (right);
    double* featureMap42Left;            // 2 feature map from the type 4 (left):
    double* featureMap42Right;           // 2 feature map from the type 4 (right);
    double* featureMap43Left;            // 3 feature map from the type 4 (left):
    double* featureMap43Right;           // 3 feature map from the type 4 (right);
    double* featureMapA1Left;            // 1 feature map from the type A (left);
    double* featureMapA1Right;           // 1 feature map from the type A (right);
    double* featureMapA2Left;            // 2 feature map from the type A (left);
    double* featureMapA2Right;           // 2 feature map from the type A (right);
    
    unsigned char* saliencyMap;          // saliencyMap collection of responses in different feature maps
    int* featureMap;                     // map of the feature;
    unsigned long* timestampMapLeft;     // timestamp reference for the map of the feature 
    unsigned long* timestampMap41Left;   // pointer to the copy of the timestamp map (41-Left)
    unsigned long* timestampMap41Right;  // pointer to the copy of the timestamp map (41-Right)
    unsigned long* timestampMap42Left;   // pointer to the copy of the timestamp map (42-Left)
    unsigned long* timestampMap42Right;  // pointer to the copy of the timestamp map (42-Right)
    unsigned long* timestampMap43Left;   // pointer to the copy of the timestamp map (43-Left)
    unsigned long* timestampMap43Right;  // pointer to the copy of the timestamp map (43-Right)
    unsigned long* timestampMapA1Left;   // pointer to the copy of the timestamp map (A1-Left)
    unsigned long* timestampMapA1Right;  // pointer to the copy of the timestamp map (A1-Right)
    unsigned long* timestampMapA2Left;   // pointer to the copy of the timestamp map (A2-Left)
    unsigned long* timestampMapA2Right;  // pointer to the copy of the timestamp map (A2-Right)
    AER_struct* unmaskedEvents;          // trained of unmasked events
    
    plotterThread* pThread;                                              // plotterThread for the trasformation of the event in images
    yarp::os::Bottle* receivedBottle;                                    // bottle currently extracted from the buffer
    yarp::os::BufferedPort<yarp::os::Bottle > outputCmdPort;             // port that is dedicated to sending the typology of the gaze behaviour and some params
    eventCartesianCollector* cfConverter;                                // receives real-time events
    eventBottleHandler *ebHandler;                                       // handler of received events as bottle
    
    bottleProcessorThread* bptA1;                                        // processor thread of the bottle whole retina events 1
    bottleProcessorThread* bptA2;                                        // processor thread of the bottle whole retina events 2
    bottleProcessorThread* bpt41;                                        // processor thread of the bottle feature map 1
    bottleProcessorThread* bpt42;                                        // processor thread of the bottle feature map 2
    bottleProcessorThread* bpt43;                                        // processor thread of the bottle feature map 3

    bool plotLatency;
    double timeStart, timeStop;                                          // variables that measures the computation load
    FILE* latencyFile;                                                   // file where all the latency measurements are saved

public:
    /**
    * default constructor
    */
    eventSelectorThread();

    /**
     * destructor
     */
    ~eventSelectorThread();

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
    * @brief returns a mono image of the output of the dvs camera (either left or right)
    * @param pixelMono reference to the image contains the counts of events
    * @param minCount reference to the min timestamp in the frame
    * @param maxCount reference to the max timestamp in the frame
    * @param camera reference to the camera the image belongs LEFT 1, RIGHT 1
    */
    void getMonoImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>* image, unsigned long minCount,unsigned long maxCount, bool camera);

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
     * @brief function that given a train of events represent them in the spatial domaain
     * @param buffer    pointer to the train of events
     * @param dimension size of the buffer of event passed as input  
     * @param w         weight associated to the particolar feature map whose events belong to.
     */ 
    void spatialSelection(AER_struct* buffer,int dimension, double w, unsigned long minCount, unsigned long maxCount);
    /**
     * @brief function that given a train of events represent them in the spatial domaain
     @param q reference to the queue of events
     */
    void spatialSelection(emorph::ecodec::eEventQueue *q);

    /** 
     * function that allocates events in the spatial memory for left camera
     */
    void memorizeLeft(int cartX,int cartY,int polarity,unsigned long ts) {};

    /** 
     * function that allocates events in the spatial memory for right camera
     */
    void memorizeRight(int cartX,int cartY,int polarity, unsigned long ts) {};

    /**
     * function that reduces the response using a function of difference in timestamp
     */
    void forgettingMemory();

};

#endif  //_CF_COLLECTOR_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

