/** 
 * @ingroup icub_module
 *
 * \defgroup icub_opticalFlow opticalFlow
 *
 * Computes the instantaneous optical flow between two images in an Gaussian-apodized window at a sample of points.
 * The flow field is computed using a phase-based Fourier tranform technique (cross-power spectrum), 
 * interpolating the resultant flow field to generate velocity magnitude and velocity direction (phase) images.
 * See the following publications for details of the technique.
 * 
 * D. Vernon, "Computation of Instantaneous Optical Flow using the Phase of Fourier Components”, 
 * Image and Vision Computing, Vol. 17, No. 3-4, pp. 189-198, 1999.
 * 
 * D. Vernon, Fourier Vision – Segmentation and Velocity Measurement using the Fourier Transform, 
 * Kluwer Academic Publishers, Norwell, Mass., 195 pages, 2001, (ISBN: 0-7923-7413-4), 
 * and The Springer International Series in Engineering and Computer Science, Vol. 623 (ISBN: 978-0-7923-7413-8) 
 *            
 * The sampling period, window size, and standard deviation of the Gaussian apodization function 
 * are provided as parameters to the module.
 *
 * A rectangular region of interest within the image can be specifed by providing the top left and bottom right
 * coordinates of the region as module parameters. Optical flow is computed only in this region of interest.  
 * The default region of interest is the entire image; this is specified when both top left and bottom right 
 * coordinates have values (0, 0) or when no region of interest is specified.
 *
 * The module takes two inputs: the port on which images at time t0 and t1 are acquired and the port on which the encode values are acquired.
 *
 * The module produces three outputs: the magnitude and phase of the interpolated optical flow field,
 * and a plot of the flow field. Optionally, the plot can be superimposed on the first of the two input images.
 *
 * \section lib_sec Libraries
 *
 * YARP.
 *
 * \section parameters_sec Parameters
 * 
 * <b>Command-line Parameters</b>
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from \c file.ini ). The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - \c from \c opticalFlow.ini       \n
 *   specifies the configuration file
 *
 * - \c context \c opticalFlow/conf  \n 
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c opticalFlow \n         
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * - \c robot \c icub \n          
 *   specifies the name of the robot (used to form the root of robot port names)
 *
 * 
 * <b>Module Configuration File Parameters</b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file e.g. \c opticalFlow.ini 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 * - \c visualImageInPort \c /visualImage:i \n  
 *   specifies the input port name for the visual images to be processed by the module
 *
 * - \c headStateInPort   \c /headState:i  \n  
 *   specifies the input port name of the head encoder values  
 * 
 * - \c magnitudeImageOutPort \c /magnitudeImage:o   \n 
 *   specifies the output port name for the image representing the magnitude of the interpolated optical flow   
 *
 * - \c phaseImageOutPort \c /phaseImage:o   \n 
 *   specifies the output port name for the image representing the phase of the interpolated optical flow   
 *
 * - \c plotImageOutPort \c /plotImage:o   \n 
 *   specifies the output port name for the image representing the plot of the optical flow field
 *
 * - \c samplingPeriod \c 16 \n           
 *   specifies the horizontal and vertical spatial period in pixels at which the optical flow is computed. 
 *
 * - \c windowSize \c 64 \n           
 *   specifies the size of the window, centred at each sample, in which the optical flow is estimated at each sample point.
 *   The value of this parameter must be a power of two since the windowed image is used as an input to a FFT algorithm 
 *   (after it has been apodized, i.e. weighted, with a Gaussian function).
 *
 * - \c sigma \c 16 \n           
 *   specifies the standard deviation of the Gaussian to be used to apodize the windowed image.
 *
 * - \c x1ROI \c 0 \n           
 *   specifies the x coordinate of the top left coordinate of the rectangular region of interest (ROI).
 *
 * - \c y1ROI \c 0 \n           
 *   specifies the y coordinate of the top left coordinate of the rectangular region of interest (ROI).
 *
  * - \c x2ROI \c 0 \n           
 *   specifies the x coordinate of the bottom left coordinate of the rectangular region of interest (ROI).
 *
 * - \c y2ROI \c 0 \n           
 *   specifies the y coordinate of the bottom left coordinate of the rectangular region of interest (ROI).
 *
 * - \c plotOnInput \n           
 *   a flag which, if present, causes the plot to be superimposed on the first of the two input images.
 *
 * - \c contrastStretch  \n
 *   a flag which, if present, causes the optical flow magnitude image to be contrast-stretched before output
 * 
 * - \c noMagnitude  \n
 *   a flag which, if present, inhibits the generation and output of the magnitude image 
 *
 * - \c noPhase  \n
 *   a flag which, if present, inhibits the generation and output of the phase image 
 *
 * - \c noPlot  \n   
 *   a flag which, if present, inhibits the generation and output of the plot image 
 *
 * - \c encoderSamplingPeriod              \c 0.05 \n
 *   The period to allow between two successive samples of the motor encoders when determining whether or not
 *   the motors have stopped and the gazed is fixed.
 *
 *
 * All the port names will be prefixed by \c /opticalFlow or whatever else is specifed by the name parameter.
 *
 * \section portsa_sec Ports Accessed
 * 
 * - \c /headState:i 
 *                      
 * \section portsc_sec Ports Created
 *
 * <b>Input ports</b>
 *
 * - \c /opticalFlow \n
 *   This port is used to change the parameters of the module at run time or stop the module
 *   The following commands are available
 * 
 *   \c help \n
 *   \c quit
 *   \c set \c period \c <n>   ... set the sampling period; this must be in the range 4-32 pixels 
 *   \c set \c window \c <n>   ... set the window size; this must be either 32 or 64 pixels
 *   \c set \c sigma  \c <m>   ... set the standard deviation of the Gaussian apodization function; this must be in the range 16-64 pixels
 *
 *   Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *   The port is attached to the terminal so that you can type in commands and receive replies.
 *   The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /opticalFlow
 *   This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 * - \c /opticalFlow/visualImage:i 
 *
 * <b>Output ports</b>
 *
 * - \c /opticalFlow \n
 *   see above
 *
 * - \c /opticalFlow/magnitudeImage:o 
 *
 * - \c /opticalFlow/phaseImage:o 
 *
 * - \c /opticalFlow/plotImage:o 
 *
 * <b>Port types </b>
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * \c BufferedPort<ImageOf<PixelRgb> >        \c visualImageInPort;        \n
 * \c BufferedPort<Vector>                    \c headStateInPort           \n
 * \c BufferedPort<ImageOf<PixelRgbFloat> >   \c magnitudeImageOutPort;    \n
 * \c BufferedPort<ImageOf<PixelRgbFloat> >   \c phaseImageOutPort;        \n
 * \c BufferedPort<ImageOf<PixelRgb> >        \c plotImageOutPort;         \n
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
 * \c opticalFlow.ini         in \c $ICUB_ROOT/app/opticalFlow/conf \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>opticalFlow --name opticalFlow --context opticalFlow/conf --from opticalFlow.ini --robot icub</tt>
 *
 * \author 
 *
 * David Vernon
 * 
 * Copyright (C) 2011 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at \c $ICUB_ROOT/contrib/src/opticalFlow/include/iCub/opticalFlow.h
 * 
 */


/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: David Vernon
 * email:   david@vernon.eu
 * website: www.vernon.eu 
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


#ifndef __ICUB_OPTICALFLOW_MODULE_H__
#define __ICUB_OPTICALFLOW_MODULE_H__

#define FRAME_SAMPLE_PERIOD 0.0 // number of milliseconds between the two frames that are input to the flow computation

/* System includes */

#include <iostream>
#include <string>


/* YARP includes */

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/dev/PolyDriver.h>

using namespace std;
using namespace yarp::os; 
using namespace yarp::sig;
using namespace yarp::dev;


/* fourierVision includes */

#include "iCub/fourierVision.h"


class OpticalFlowThread : public Thread
{
private:

   /* class variables */

   int    x, y;
   int    width, height, depth;    // dimensions of the image
   float  velocity_value;
   int    i;
   bool   debug; 
   bool   stable;  	    
   double allowableEncoderVariation;

   PixelRgb          rgbPixel;
   PixelRgbFloat     floatRgbPixel;
   ImageOf<PixelRgb> *t0Image;
   ImageOf<PixelRgb> *t1Image;
   DVimage           *t0Input;
   DVimage           *t1Input;
   DVimage           *magnitudeOutput;
   DVimage           *filteredMagnitudeOutput;
   DVimage           *interpolatedMagnitudeOutput;
   DVimage           *phaseOutput;
   DVimage           *interpolatedPhaseOutput;
   DVimage           *plotOutput;
   Vector            *encoderPositions;
   Vector            *tempEncoderPositions;
   /* thread parameters: they are pointers so that they refer to the original variables in opticalFlow */

   BufferedPort<ImageOf<PixelRgb> >      *visualImagePortIn;
   BufferedPort<ImageOf<PixelRgbFloat> > *magnitudeImagePortOut;
   BufferedPort<ImageOf<PixelRgbFloat> > *phaseImagePortOut;
   BufferedPort<ImageOf<PixelRgb> >      *plotImagePortOut;
   BufferedPort<Vector>                  *robotPortIn;        // port for reading version and vergence angles

   int   *samplingPeriod;          
   int   *windowSize; 
   float *sigma;    
   int   *x1ROI;
   int   *y1ROI;
   int   *x2ROI;
   int   *y2ROI;
   bool  *plotOnInput;
   bool  *contrastStretch;
   bool  *noMagnitude;
   bool  *noPhase;
   bool  *noPlot;
   double *encoderSamplingPeriod;    

public:

   /* class methods */

   OpticalFlowThread(       BufferedPort<ImageOf<PixelRgb> >      *visualImageIn, 
                            BufferedPort<Vector>                  *robotPortIn,
                            BufferedPort<ImageOf<PixelRgbFloat> > *magnitudeImageOut, 
                            BufferedPort<ImageOf<PixelRgbFloat> > *phaseImageOut, 
                            BufferedPort<ImageOf<PixelRgb> >      *plotImageOut, 
                            int   *samplingPeriod, 
                            int   *windowSize,
                            float *sigma,
                            int   *x1ROI,
                            int   *y1ROI,
                            int   *x2ROI,
                            int   *y2ROI,
                            double *encoderSamplingPeriod,
                            bool  *plotOnInput,
                            bool  *contrastStretch,
                            bool  *noMagnitude,
                            bool  *noPhase,
                            bool  *noPlot); 
   bool threadInit();     
   void threadRelease();
   void run(); 
};


class opticalFlow:public RFModule
{
   /* module parameters */

   string moduleName;
   string robotName; 
   string visualImageInputPortName;
   string magnitudeOutputPortName;  
   string phaseOutputPortName;  
   string plotOutputPortName;  
   string handlerPortName;
   string robotPortName;  
   int    samplingPeriod;
   int    windowSize;
   float  sigma;
   int    x1ROI;
   int    y1ROI;
   int    x2ROI;
   int    y2ROI;
   bool   plotOnInput;
   bool   contrastStretch;
   bool   noMagnitude;
   bool   noPhase;
   bool   noPlot;
   double encoderSamplingPeriod;     



   /* class variables */

   bool debug;

   BufferedPort<ImageOf<PixelRgb> >      visualImageIn;      
   BufferedPort<ImageOf<PixelRgbFloat> > magnitudeImageOut;     
   BufferedPort<ImageOf<PixelRgbFloat> > phaseImageOut;     
   BufferedPort<ImageOf<PixelRgb> >      plotImageOut;     
   Port                                  handlerPort;      
   BufferedPort<Vector>                  robotPortIn;     

   /* pointer to a new thread to be created and started in configure() and stopped in close() */

   OpticalFlowThread *opticalFlowThread;

public:
   opticalFlow();
   bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
   bool interruptModule();                       // interrupt, e.g., the ports 
   bool close();                                 // close and shut down the module
   bool respond(const Bottle& command, Bottle& reply);
   double getPeriod(); 
   bool updateModule();
};


#endif // __ICUB_OPTICALFLOW_MODULE_H__
//empty line to make gcc happy

