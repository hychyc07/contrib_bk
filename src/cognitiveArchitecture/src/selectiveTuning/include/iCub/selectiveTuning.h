/** 
 * @ingroup icub_module
 *
 * \defgroup icub_selectiveTuning selectiveTuning
 *
 * This module implements a limited version of the Selective Tuning Model (STM) of visual attention,
 * a model developed by John Tsotsos, York University, Canada. 
 * The model is described in detail in a paper published in Artificial Intelligence:
 *
 *     J. K. Tsotsos, S. Culhane, W. Wai, Y. Lai, N. David, and F. Nuflo.
 *     Modeling visual attention via selective tuning. Artificial Intelligence, 78:507–547, 1995.
 *
 *  Additional background material may be found on the STM website at
 *      http://web.me.com/john.tsotsos/Visual_Attention/Selective_Tuning.html
 *
 * The current implementation exploits two simplifications:
 *
 * 1. The resolution of the layers of the visual pyramid reduces by a fixed factor of two on each level, 
 *    rather than allowing the specification of arbitrary reductions in resolution, as envisaged in the AI paper.  
 *    This simplification has the significant consequence that each assembly of units on any one level is connected 
 *    to at most one assembly on the level above and, thus, the gating control unit and the bias unit of that assembly 
 *    have only one input from the level above. This simplifies considerably the selective tuning process, 
 *    requiring the computation of the interpretive, gating, gating control, and bias units only at locations 
 *    which are covered by the winning location in the initial global winner-take all competition at the top level 
 *    of the pyramid. 
 *
 * 2. The receptive fields are optionally square, rather than rectangular.  
 *    Again, this simplification reduces the computational complexity of the module, 
 *    reducing the number of interpretive units from (n2 - n1 + 1)^2 to (n2 - n1 +1),
 *    where n1 and n2 are the minimum and maximum lengths of the sides of the receptive fields, respectively.
 *
 * The current implementataion also gives the option of constructing the image pyramid by using the maximum value
 * rather than the average value when computing the level l+1 from level l.  This option allows very small but intense 
 * points of salience to propagated from level to level, avoiding their loss due to averaging.
 *
 * The module takes two inputs: 
 *
 * 1. the port on which an image with the visual data for level 1 of the pyramid is acquired. 
 *    Typically, this image represents some saliency feature, be it simple or complex.
 *    The width and height of this image should match the resolution of level 1 
 * 
 * 2. the port on which an image containing the bias unit data for level 1 of the pyramid is acquired. 
 *    The width and height of this image should match the resolution of level 1 of the pyramid. 
 *    Note that providing the bias unit data at level 1 rather than level L is different from that envisaged in the 
 *    original paper which suggested that the bias image be input at level L. 
 *    We do it this way to keep the scale of the bias and visual data compatible.
 *    This also means that the height of the processing pyramid can be changed without having to change the size of the
 *    bias data.
 *    The bias image is optional (see biasImageRequired parameter below).
 *
 * The module produces two outputs: 
 *
 * 1. the port on which a bottle containing the number of winning units and the x and y coordinates for each one is sent.
 *    The syntax of this bottle message is "size" <number_of_winning_units> {"x" <x_coordinate> "y" <y_coordinate>}
 *    where the braces {} means repeat zero or more times, depending on the value of <number_of_winning_units>
 *
 * 2. the port on which an image of the visual data for level 1 of the pyramid is sent; 
 *    this differs from the input image in that the winning units are depicted graphically with a cross-hairs.
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
 * - \c from \c selectiveTuning.ini       \n
 *   specifies the configuration file
 *
 * - \c context \c selectiveTuning/conf  \n 
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c selectiveTuning \n         
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * - \c robot \c icub \n          
 *   specifies the name of the robot (used to form the root of robot port names)
 *
 * 
 * <b>Module Configuration File Parameters</b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file e.g. \c selectiveTuning.ini 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 * - \c visualImageInPort \c /visualImage:i \n  
 *   specifies the input port name for the visual image representing the interpretive units at level 1 of the processing pyramid \n
 *
 * - \c biasImageInPort \c /biasImage:i   \n 
 *   specifies the input port name for the image representing the bias units at level 1 of the processing pyramid \n  
 *
 * - \c wtaUnitsOutPort \c /wtaUnits:o  \n
 *   specifies the output port name for the data specifying the number and coordinates of the winning units  \n 
 *
 * - \c wtaImageOutPort \c /wtaImage:o   \n 
 *   specifies the output port name for the input visual image with the graphic display of the winning units added \n
 *
 * - \c numerOfLevels \c 4     \n           
 *   specifies the number of levels in the STM pyramid (1 to L)\n
 *
 * - \c minRFSize \c 6 \n           
 *   specifies the minimum size of the (square) receptive fields;  \n
 *
 * - \c maxRFSize \c 25 \n           
 *   specifies the maximum size of the (square) receptive fields; \n
 *
 * - \c rectangularRF \c 0 \n
 *   specifies whether or not to use rectangular receptive fields 
 *   (default 0 is not to do so, in which case the receptive fields are square)
 *
 * - \c localMax \c 0 \n
 *   specifies whether or not to use the maximum local value when constructing the pyramidal parent of a 2x2 region
 *   (default 0 is not to do so, in which case the parent value is the average of the 2x2 region)
 *
 * - \c biasImageRequired  \n   
 *   a flag which, if present, requires input of the bias image before performing the selective tuning; the module will block until this is received.
 *   Otherwise, by default, the bias image is not required.
 *
 * All the port names will be prefixed by \c /selectiveTuning or whatever else is specifed by the name parameter.
 *
 * \section portsa_sec Ports Accessed
 * 
 * - none
 *                      
 * \section portsc_sec Ports Created
 *
 * <b>Input ports</b>
 *
 * - \c /selectiveTuning \n
 *   This port is used to change the parameters of the module at run time or stop the module  \n
 *
 *   The following commands are available  \n
 * 
 *   \c help  \n
 *   \c quit  \n
 *   \c set \c minRFSize \c <n>   ... set minimum value of the side of the rectangual receptive fields\n
 *   \c set \c maxRFSize \c <n>   ... set maximum value of the side of the rectangual receptive fields\n
 *
 *   Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *   The port is attached to the terminal so that you can type in commands and receive replies.
 *   The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /selectiveTuning
 *   This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 * - \c /selectiveTuning/visualImage:i   \n
 *
 * - \c /selectiveTuning/biasImage:i     \n
 *
 * <b>Output ports</b>
 *
 * - \c /selectiveTuning \n
 *   see above
 *
 * - \c /selectiveTuning/wtaUnits:o      \n
 *
 * - \c /selectiveTuning/wtaImage:o      \n
 *
 *
 * <b>Port types </b>
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * \c BufferedPort<ImageOf<PixelRgbFloat> >   \c visualImageInPort;        \n
 * \c BufferedPort<ImageOf<PixelRgbFloat> >   \c biasImageInPort;          \n
 * \c BufferedPort<Bottle>                    \c wtaUnitsOutPort;          \n
 * \c BufferedPort<ImageOf<PixelRgb> >        \c wtaImageOutPort;          \n
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
 * \c selectiveTuning.ini         in \c $ICUB_ROOT/app/selectiveTuning/conf \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>selectiveTuning --name selectiveTuning --context selectiveTuning/conf --from selectiveTuning.ini --robot icub</tt>
 *
 * \author 
 *
 * David Vernon
 * 
 * Copyright (C) 2012 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at \c $ICUB_ROOT/contrib/src/selectiveTuning/include/iCub/selectiveTuning.h
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


/*
 * Audit Trail
 * -----------
 * 30/05/11  Started development    DV
 * 20/07/11  First version operational DV
 * 26/07/12  Added biasImageRequired parameter DV
 */ 


#ifndef __ICUB_selectiveTuning_MODULE_H__
#define __ICUB_selectiveTuning_MODULE_H__


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
using namespace yarp::sig::draw;

/* fourierVision includes */

#include "iCub/fourierVision.h"


class SelectiveTuningThread : public Thread
{
private:

   /* class variables */

   int x, y;
   int visual_width, visual_height, visual_depth;     
   int bias_width,   bias_height,   bias_depth;
   int width, height;
   int attention_x;
   int attention_y;
   int temp;
   float float_temp;

   maxima_data_type winning_units[MAX_WINNING_UNITS];
   int number_of_winning_units;

   Bottle wta_bottle;

   bool debug; 
   
   PixelRgb               rgbPixel;
   PixelRgbFloat          floatRgbPixel;
   ImageOf<PixelRgbFloat> *visualImage;
   ImageOf<PixelRgbFloat> *biasImage;
   DVimage                *visualInput;
   DVimage                *biasInput;
   Bottle                 wtaBottle;

  	    
   /* thread parameters: they are pointers so that they refer to the original variables in selectiveTuning */

   BufferedPort<ImageOf<PixelRgbFloat> > *visualImagePortIn;
   BufferedPort<ImageOf<PixelRgbFloat> > *biasImagePortIn;
   BufferedPort<ImageOf<PixelRgbFloat> > *wtaImagePortOut;
   BufferedPort<Bottle >                 *wtaUnitsPortOut;   

   int   *numberOfLevels;          
   int   *minRFSize;    
   int   *maxRFSize;
   int   *rectangularRF;
   int   *localMax;
   bool  *biasImageRequired;


public:

   /* class methods */

   SelectiveTuningThread(   BufferedPort<ImageOf<PixelRgbFloat> > *visualImageIn, 
                            BufferedPort<ImageOf<PixelRgbFloat> > *biasImageIn, 
                            BufferedPort<ImageOf<PixelRgbFloat> > *wtaImageOut, 
                            BufferedPort<Bottle >                 *wtaUnitsOut, 
                            int   *numberOfLevels, 
                            int   *minRFSize,
                            int   *maxRFSize,
                            int   *rectangularRF,
                            int   *localMax,
                            bool  *biasImageRequired); 
   bool threadInit();     
   void threadRelease();
   void run(); 
};


class selectiveTuning:public RFModule
{
   /* module parameters */

   string moduleName;
   string robotName; 
   string visualImageInputPortName;
   string biasImageInputPortName;  
   string wtaImageOutputPortName;  
   string wtaUnitsOutputPortName;  
   string handlerPortName;
   int    numberOfLevels;
   int    minRFSize;
   int    maxRFSize;
   int    rectangularRF;
   int    localMax;
   bool   biasImageRequired;



   /* class variables */

   bool debug;

   BufferedPort<ImageOf<PixelRgbFloat> > visualImageIn;      
   BufferedPort<ImageOf<PixelRgbFloat> > biasImageIn;     
   BufferedPort<ImageOf<PixelRgbFloat> > wtaImageOut;     
   BufferedPort<Bottle >                 wtaUnitsOut;     
   Port                                  handlerPort;      //a port to handle messages 

   /* pointer to a new thread to be created and started in configure() and stopped in close() */

   SelectiveTuningThread *selectiveTuningThread;

public:
   selectiveTuning();
   bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
   bool interruptModule();                       // interrupt, e.g., the ports 
   bool close();                                 // close and shut down the module
   bool respond(const Bottle& command, Bottle& reply);
   double getPeriod(); 
   bool updateModule();
};


#endif // __ICUB_selectiveTuning_MODULE_H__
//empty line to make gcc happy

