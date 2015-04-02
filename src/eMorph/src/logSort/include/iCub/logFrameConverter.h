// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
 * email:   francesco.rea@iit.it, charles.clercq@iit.it
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
 * @file logFrameConverter.h
 * @brief A class inherited from the bufferefPort class created in order to read events.
 */

#ifndef _LOG_FRAME_CONVERTER_H
#define _LOG_FRAME_CONVERTER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <cstring>
#include <ctime>
#include <list>

#include <iCub/emorph/eventBuffer.h>

#include <iCub/logUnmask.h>
#include <iCub/convert.h>

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/all.h>


class logFrameConverter:public yarp::os::BufferedPort<emorph::ebuffer::eventBuffer> {
public:
    logFrameConverter();
   
    ~logFrameConverter();

    
    /**
     * function that changes the semaphore affected by the copy
     * @param previousSemaphore semaphore id that was in charge of handling copy chunk
     * @param newerSemaphore semaphore id that will be in charge of handling copy chunk
     */
    void commuteSemaphore(int previousSemaphore, int newerSemaphore) {
        semaphore_id = newerSemaphore;
    }

    /**
     * function that call the wait function of the semaphore in charge of copychunk
     */
    void waitSemaphore();

    /**
     * function that call the post function of the semaphore in charge of copychunk
     */
    void postSemaphore(); 
    

    /**
    * overwritten function for handling events as soon as they arrive
    */
    virtual void onRead(emorph::ebuffer::eventBuffer& b);


    /**
    * function that returns a pointer to read buffer 
    * @return pointer to the buffer that has to be read
    */
    char* getBuffer() { return pcRead; };

    /**
    * function that copies a chunk of elements
    * @param pointerCopy pointer to the buffer that has to be read
    * @param pointerFlag pointer to the buffer of flags 
    */
    void copyChunk(char* pointerCopy, char* pointerFlag);

    /**
    * function that copies a chunk of elements
    * @param pointerCopy pointer to the buffer that has to be read
    * @param dimPacket dimension of the packet to be read 
    */
    void copyChunk(char* pointerCopy, int dimPacket);

    /**
    * @brief returns a mono image of the output of the dvs camera (either left or right)
    * @param pixelMono reference to the image contains the counts of events
    * @param minCount reference to the min timestamp in the frame
    * @param maxCount reference to the max timestamp in the frame
    * @param camera reference to the camera the image belongs LEFT 1, RIGHT 1
    */
    void getMonoImage(yarp::sig::ImageOf<yarp::sig::PixelMono>* image, unsigned long minCount,unsigned long maxCount, bool camera);
    
    /**
    * function that return the last time stamp saved in the buffer for left camera
    * @return the unsigned int representing the last event timestamp
    */
     unsigned long getLastTimeStamp();

    /**
    * function that return the last time stamp saved in the buffer for right camera
    * @return the unsigned int representing the last event timestamp
    */
     unsigned long getLastTimeStampRight();

    /**
    * function that return the last time stamp saved in the buffer
    * @return the unsigned int representing the last event timestamp
    */
     unsigned long getEldestTimeStamp();

    /**
     * function that returns the counter of the pushes and pops in the buffer
     */
    int getCountBuffer() { return countBuffer; };

    /**
     * function that returns the dimension of the last read packet
     */
    int getPacketSize() { return dimPacket; };

    /**
    * @brief clears monoImage collection of events
    * @param pixelMono reference to the image contains the counts of events
    */
     void clearMonoImage();

    /**
       function that restarts from zero the list of memorised timestamp
     */
    void resetTimestamps();

    /**
     * @brief return whether the unmasking is valid
     */
    bool isValid(){return valid;};

private:
    FILE* fout;                                                 // file for dumping at the on read
    bool valid;
    short state;
    int retinalSize;                                            // dimension of the retina default 128x128
    int totDim;                                                 // total dimension of the read buffer
    int outputWidth, outputHeight;                              // dimension of the output image default 320x240
    int countBuffer;                                            // counter of pushes and pops in the buffer
    int dimPacket;                                              // dimension of the last read packate          
    unsigned long previousTimeStamp;                            // timestamp at the previous run
    char* converterBuffer;                                      // buffer used as saved
    char* converterBuffer_copy;                                 // copy of the buffer pointer, points to the location for freeing
    char* unreadBuffer;                                         // flag buffer that indicates which element of the buffer has been read
    char* receivedBuffer;                                       // temporarely pointer to the received buffer
    char* pcRead;                                               // pointer to the location where to read events
    char* pcBuffer;                                             // pointer where to buffer events
    char *flagCopy, *flagRead;                                  // pointer to the location of reading and copying  

    logUnmask unmask_events;           // object in charge of unmasking the events
    converter convert_events;       // object in charge of converting the events into an image
    int semaphore_id;                   // identifier of the semaphore in charge of copying chunk of event

    yarp::os::Semaphore mutex;      // semaphore for thehandling resource buffer
    yarp::os::Semaphore mutex1;         // semaphore thar regulates the access to the section 1 buffer resource
    yarp::os::Semaphore mutex2;         // semaphore thar regulates the access to the section 2 buffer resource
    yarp::os::Semaphore mutex3;         // semaphore thar regulates the access to the section 3 the buffer resource
    int countSemaphore;                 // counter for the semaphore in onRead function

    clock_t start_u;
    clock_t start_p;
    clock_t stop;
};

#endif //_LOG_FRAME_CONVERTER_H
//----- end-of-file --- ( next line intentionally left blank ) ------------------

