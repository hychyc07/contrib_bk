/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Vadim Tikhanoff & Ajay Mishra
 * email:   vadim.tikhanoff@iit.it
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

#ifndef __ICUB_OBJSEGMODULE_H__
#define __ICUB_OBJSEGMODULE_H__

#include <iostream>
#include <string>

#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Port.h>
#include <yarp/os/Time.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <stdlib.h>
#include "iCub/segmentationLayer.h"
//#include <iCub/functionEncoder.h>

//#define WAVE_R 10.0

//using namespace ctrl;
  
class OBJSEGThread : public yarp::os::Thread
{
private:

    std::string moduleName;            //string containing module name 
    std::string inputNameImage;          //string containing input port name 
    std::string outputNameSegCrop;          //string containing input port name 
    std::string outputNameSeg;         //string containing output port name
    std::string fixationNamePort;

    /* thread parameters: they are pointers so that they refer to the original variables */
	yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >  imageIn;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> >  imageOut;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> >  imageOutCrop;
    yarp::os::Port fixationPort;

    //yarp::os::BufferedPort < yarp::sig::Vector >    portInCog;

	bool allocated; // flag to check if the variables have been already allocated
    segLayer frame;
    IplImage *seg, *segOnly, *segOnlyRgb, *segRgb;
    bool once, first;
    double fix_x_prev;
    double fix_y_prev;

public:

    ~OBJSEGThread();
    /* class methods */
    OBJSEGThread( std::string moduleName );
    
    void initAll();
    bool threadInit();     
    void threadRelease();
    void run(); 
    void onStop();
    void segmentWithFixation(IplImage *img_in, double x, double y);
    void sendSegOnly(IplImage *img, IplImage *imgOrig);

};

class objSegMod:public yarp::os::RFModule
{
    /* module parameters */
    std::string moduleName;      
    std::string handlerName;
	
    yarp::os::Port handlerPort;      //a port to handle messages 
    
    /* pointer to a new thread to be created and started in configure() and stopped in close() */
    OBJSEGThread *objSegThread;

public:

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod(); 
    bool updateModule();
};

#endif
//empty line to make gcc happy
