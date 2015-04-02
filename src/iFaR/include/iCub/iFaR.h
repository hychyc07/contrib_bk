/** 
 * @ingroup icub_module
 *
 * \defgroup icub_myModule myModule
 *
 * This is a simple example to illustrate a module that is compliant with iCub Software Standards, addressing:
 *
 * - configuration
 * - graceful shut-down
 * - thread-based execution
 * - run-time user interaction
 * - documentation and coding standards
 *
 * Functionally, the module just converts an input image to a binary image based on the supplied threshold
 *
 * A complete tutorial for this example is available on the iCub wiki at 
 * http://wiki.icub.org/wiki/Summary_of_iCub_Software_Development_Guidelines
 *
 * 
 * \section lib_sec Libraries
 *
 * YARP.
 *
 * \section parameters_sec Parameters
 * 
 * Command-line Parameters
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing -- to the key 
 * (e.g. --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - from myModule.ini       
 *   specifies the configuration file
 *
 * - context myModule/conf   
 *   specifies the sub-path from $ICUB_ROOT/icub/app to the configuration file
 *
 * - name myModule          
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * - robot icub             
 *   specifies the name of the robot (used to form the root of robot port names)
 *
 *
 * Configuration File Parameters
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 * - myInputPort /image:i     
 *   specifies the input port name (this string will be prefixed by /myModule/ 
 *   or whatever else is specifed by the name parameter
 *
 * - myOutputPort /image:o     
 *   specifies the input port name (this string will be prefixed by /myModule/ 
 *   or whatever else is specifed by the name parameter
 *
 * - cameraConfig icubEyes.ini
 *   specifies the camera configuration file containing the intrinsic parameters of
 *   the left and right cameras
 *
 * - threshold 7             
 *   specifies the threshold value
 *
 * 
 * \section portsa_sec Ports Accessed
 * 
 * - None
 *                      
 * \section portsc_sec Ports Created
 *
 *  Input ports
 *
 *  - /myModule
 *    This port is used to change the parameters of the module at run time or stop the module
 *    The following commands are available
 * 
 *    help
 *    quit
 *    set thr <n>   ... set the threshold for binary segmentation of the input RGB image
 *    (where <n> is an integer number)
 *
 *    Note that the name of this port mirrors whatever is provided by the --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: yarp rpc /myModule
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - /myModule/image:i
 *
 * Output ports
 *
 *  - /myModule
 *    see above
 *
 *  - /myModule/image:o
 *
 * Port types 
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * - BufferedPort<ImageOf<PixelRgb> >   myInputPort;     
 * - BufferedPort<ImageOf<PixelRgb> >   myOutputPort;       
 *
 * \section in_files_sec Input Data Files
 *
 * None
 *
 * \section out_data_sec Output Data Files
 *
 * None
 *
 * \section conf_file_sec Configuration Files
 *
 * myModule.ini  in $ICUB_ROOT/icub/app/myModule/conf
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * myModule --name myModule --context myModule/conf --from myModule.ini --robot icub
 *
 * \author David Vernon
 * 
 * Copyright (C) 2009 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at src/myModule/src/myModule.h
 * 
 */  


/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: David Vernon
 * email:   david@vernon.eu
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
 

/*
 * Audit Trail
 * -----------
 * 26/08/09  First version validated   DV
 * 12/09/09  Added functionality to read additional configuration file DV
 */  


#ifndef __ICUB_IFARMODULE_MODULE_H__
#define __ICUB_IFARMODULE_MODULE_H__ 
#define YARP_CVTYPES_H_
#include <iostream>
#include <string> 

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Property.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <include/iCub/centerSurround.h>
#include "include/iCub/convolve.h"

#include <cv.h>
#include <cvaux.h>
#include <cxcore.h>
#include <highgui.h>
#include <time.h>

#include <string>
#include <iostream>

using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;
using namespace yarp::dev;

// wrapper class. implement any particular filter as a subclass
// override the filter() method
class MonoFilter /*: public convolve*/
{
private:  

   /* class variables */ 
   ImageOf<PixelMono> *imageIn;
   IplImage *imageIPLIn; 
   IplImage *imageIPLOut;
   CenterSurround *myFilter;
 
public: 

   /* class methods */
   MonoFilter(ImageOf<PixelMono> *imageInput, IplImage *imageOutput, CenterSurround *csFilter);
   void filter(); //override in your subclass
};

class BinocFilter
{
private:  

   /* class variables */ 

   IplImage *imageL;
   IplImage *imageR;
   IplImage *imageBoth;

   CvMat *filtBoth; CvMat *filtBoth2; CvMat *matL; CvMat *matR;
   
public: 

   /* class methods */

   BinocFilter(IplImage *imageInputL, IplImage *imageInputR, IplImage *imageOutput);
   void filter(); //override in subclasses to implement fucntionality
};


class SalienceMap
{
private:  

   /* class variables */ 

   int *attentionX, *attentionY;
   IplImage *imageIn;
   /* thread parameters: they are pointers so that they refer to the original variables in myModule */
 
public: 
    /* class methods */
	
	SalienceMap(IplImage *imageInput, int *attX, int *attY);
	void setROI(); //override in subclasses
};

class Ifar: public RateThread
{
private:  

  /* class variables */ 

  int *attentionX; int *attentionY;
  double *maxVal; double *minVal;
  
  Bottle *attentionOutBottle;
  ImageOf<PixelMono> *imageL;
  ImageOf<PixelMono> *imageR;
  ImageOf<PixelBgr> *salienceMap;
  
  IplImage *leftOut8u, *rightOut8u, *imageBoth;
  
  MonoFilter *leftFilter1;
  MonoFilter *rightFilter1;
  BinocFilter *binocFilter1;
  SalienceMap *mySalienceMap;
  CenterSurround* filterA;
  
  convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,yarp::sig::ImageOf<yarp::sig::PixelMono> ,uchar >* gaborPosHorConvolution;
  convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,yarp::sig::ImageOf<yarp::sig::PixelMono> ,uchar >* gaborPosVerConvolution;
  convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,yarp::sig::ImageOf<yarp::sig::PixelMono> ,uchar >* gaborNegHorConvolution;
  convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,yarp::sig::ImageOf<yarp::sig::PixelMono> ,uchar >* gaborNegVerConvolution;
  convolve<yarp::sig::ImageOf<yarp::sig::PixelFloat>,float,yarp::sig::ImageOf<yarp::sig::PixelFloat> ,float >* gaborFiveByFive[4];
  
  /* thread parameters: they are pointers so that they refer to the original variables in myModule */
  
  yarp::os::BufferedPort<ImageOf<PixelMono> > *imagePortInL;
  yarp::os::BufferedPort<ImageOf<PixelMono> > *imagePortInR;  
  yarp::os::BufferedPort<ImageOf<PixelBgr> > *salienceMapOutPort;  
  yarp::os::BufferedPort<Bottle> *attentionPortOut;
  
public: 
	
	/* class methods */
	Ifar(int ratePeriod, yarp::os::BufferedPort<ImageOf<PixelMono> > *imageL3,  yarp::os::BufferedPort<ImageOf<PixelMono> > *imageR3, yarp::os::BufferedPort<Bottle> *attentionPortOut, yarp::os::BufferedPort<ImageOf<PixelBgr> > *salMapPort);
	bool threadInit();     
	void threadRelease();
	void run(); 
};

class IfarModule:public RFModule
{
  /* module parameters */ 
  string moduleName, robotName, robotPortName, inputPortNameL, inputPortNameR, outputPortName, handlerPortName, salMapPortName;	
  /* class variables */ 
  BufferedPort<ImageOf<PixelMono> > imageInL; 
  BufferedPort<ImageOf<PixelMono> > imageInR; 
  BufferedPort<ImageOf<PixelBgr> > salienceMapOut; 
  BufferedPort<Bottle> attentionOut;     
  Port handlerPort;      //a port to handle messages  
	
   /* pointer to a new thread to be created and started in configure() and stopped in close() */ 
    Ifar *myIfarThread;

public:
   
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const Bottle& command, Bottle& reply);
    double getPeriod(); 
    bool updateModule();
}; 


#endif // __ICUB_MYMODULE_MODULE_H__

/* empty line to make gcc happy */
