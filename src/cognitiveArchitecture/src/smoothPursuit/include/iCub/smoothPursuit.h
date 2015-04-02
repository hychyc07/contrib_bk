/**
 *
 * @ingroup icub_module
 * \defgroup icub_smoothPursuit smoothPursuit
 *
 * Convert the coordinates of a desired fixation point to the relative version (azimuth and elevation) angles 
 * of the rotation required to centre eye gaze on that point, effect the rotation by sending these angles to 
 * a controller.
 *
 * \section intro_sec Description
 *
 * Given as input the results of a some attentional process (such as the Selective Tuning Model [Tstotsos et al 1995]), 
 * select one of a set of winning units (i.e. potential foci of attention) as the desired fixation point, 
 * and convert its associated coordinates to the relative gaze angles required to fix the gaze on that fixation point.
 * Then effect the change in gaze by sending as output the azimuth, elevation, and vergence angles of desired fixation point
 * in a form that is compatible with the iKinGazeCtrl contoller. These are output as relative angles, 
 * i.e. relative the current gaze direction, so that the azimuth and elevation angles correspond to the required rotation.
 *
 * azimuth    =  k * arctan[ (target_x - camera_centre_x) , focal_length_x]
 * elevation  = -k * arctan[ (target_y - camera_centre_y) , focal_length_y]
 *
 * The formula for elevation is negative because "down" (i.e. increasing value of y) is a negative angle. 
 * k is a user-supplied constant of proportionality to allow the gain to be adjusted. The default value is 0.5.
 *
 * The vergence angle is set to zero (no change) since it is computed separately using the binocularVergence module.
 * This output should be connected to the /angles:i port of the iKinGazeCtrol module (e.g. /icub/iKinGazeCtrl/head/angles:i)
 * 
 * \section lib_sec Libraries
 * 
 * YARP.
 *
 *
 * \section parameters_sec Parameters
 * 
 * <b>Command Line Parameters</b>
 *
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from file.ini . The value part can be changed to suit your needs; the default values are shown below. 
 * 
 * - \c from \c smoothPursuit.ini \n
 *   specifies the configuration file 
 * 
 * - \c context \c smoothPursuit/conf \n
 *   specifies the sub-path from $ICUB_ROOT/icub/app to the configuration file
 * 
 * - \c name \c smoothPursuit \n
 *   specifies the name of the module (used to form the stem of module port names)
 *
 * - \c cameraConfig \c iCubEyes.ini \n
 *
 *   This is a file with at least the intrinsic camera parameters for the left and right cameras,
 *   listed under to group headings \c [CAMERA_CALIBRATION_LEFT] and \c [CAMERA_CALIBRATION_RIGHT]
 *   The parameters that are required for computation of the version angles are \c fx \c fy \c cx  \c cy
 *   Also, we assume that the right eye/camera is dominant so we use \c fx \c fy \c cx  \c cy from the right camera 
 *   calibration group, viz:
 *
 *   \c [CAMERA_CALIBRATION_RIGHT]
 * 
 *   \c w \c 320 \n
 *   \c h \c 240 \n
 *   \c fx \c 225.904 \n
 *   \c fy \c 227.041 \n
 *   \c cx \c 157.858 \n
 *   \c cy \c 113.51 \n
 *   \c k1 \c -0.377318 \n
 *   \c k2 \c 0.155149 \n
 *   \c p1 \c -0.000726514 \n
 *   \c p2 \c 0.000317338 \n
 * 
 * 
 * <b>Module Configuration File Parameters</b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 * 
 * 
 * - \c gain                             \c 1 \n
 *   The constant of proportionality that is used when computing the gaze angles from the required fixation coordinates
 *   and the camera calibration parameters.  The default value is 0.5.
 *
 * - \c wtaUnitsInPort                   \c /wtaUnits:i  \n
 *   specifies the input port name for the data specifying the number and coordinates of the potential fixation points  \n 
 *
 * - \c headStateInPort                  \c /headState:i  \n  
 *   specifies the input port name of the head encoder values  
 * 
 * - \c gazeAnglesOutPort                \c /gazeAngles:o  \n
 *   The azimuth, elevation, and vergence angles of the fixation point on which the eye gaze should converge.
 *   Typically, this will be connected to the /angles:i port of the iKinGazeCtrl module (e.g. /icub/iKinGazeCtrl/head/angles:i)
 *
 * All these port names will be prefixed by \c /smoothPursuit or whatever else is specifed by the name parameter.
 *
 * \section portsa_sec Ports Accessed
 * 
 * - \c /headState:i 
 *                      
 * \section portsc_sec Ports Created
 *
 * <b>Input ports</b>
 *
 * - \c /smoothPursuit \n
 *   This port is used to change the parameters of the module at run time or stop the module  \n
 *
 *   The following commands are available  \n
 * 
 *   \c help  \n
 *   \c quit  \n
 *   \c set \c gain   \c <n>   ... set the constant of proportionality for the angle computation (possibly fractional) \n
 *
 *   Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *   The port is attached to the terminal so that you can type in commands and receive replies.
 *   The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /selectiveTuning
 *   This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 * - \c /smoothPursuit/wtaUnits:i     \n
 *
 *
 * <b>Output ports</b>
 *
 * - \c /smoothPursuit/gazeAngles:o   \n
 *
 *
 * <b>Port types </b>
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * \c BufferedPort<Bottle>               \c wtaUnitsInPort;           \n
 * \c BufferedPort<Vector>               \c headStateInPort           \n
 * \c BufferedPort<Bottle>               \c gazeAnglesOutPort: ["rel" azimuth elevation vergence]  \n
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
 * \c smoothPursuit.ini
 * \c icubEyes.ini       
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows
 *
 * \section example_sec Example Instantiation of the Module
 *
 * <tt>smoothPursuit  --context smoothPursuit/conf  --from smoothPursuit.ini --configCamera icubEyes.ini</tt>
 *
 * \author 
 *
 * David Vernon  
 * 
 * Copyright (C) 2011 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at \c $ICUB_ROOT/contrib/src/cognitiveArchitecture/src/smoothPursuit/include/iCub/smoothPursuit.h
 *
**/

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: David Vernon
 * email:   david@vernon.eu
 * website: wwww.vernon.eu
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



#ifndef __ICUB_SMOOTHPURSUIT_MODULE_H__
#define __ICUB_SMOOTHPURSUIT_MODULE_H__


/* System includes */

#include <assert.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/timeb.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string>
#include <iostream>

/* YARP includes */

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;


/* fourierVision includes */

#include "iCub/fourierVision.h"
  
class SmoothPursuitThread : public Thread
{
private:

    /* class variables */
  
    int debug;

    int temp;
    int i, j, p, q;
    int x, y;
    double azimuth, elevation, vergence;
    double current_azimuth, current_elevation, current_vergence;
    double max_azimuth, max_elevation;
    double min_azimuth, min_elevation;
    double previous_azimuth, previous_elevation;
    double temp_azimuth, temp_elevation;
    double f_x;
    double f_y;
    float controlGaze_x;
    float controlGaze_y;
    double allowableEncoderVariation;
    Vector  *encoderPositions;
    Bottle  *wtaBottle;

    double  encoderSamplingPeriod;

    bool limitingMotion;
    Vector  *tempEncoderPositions;

    maxima_data_type winning_units[MAX_WINNING_UNITS];
    int number_of_winning_units;

    int selected_fixation_point;
 	    
    /* thread parameters: they are all pointers so that they refer to the original variables in the smoothPursuit */

    BufferedPort<Bottle >            *wtaUnitsPortIn;     // port for acquiring WTA covert foci of attention
    BufferedPort<Bottle>             *gazeAnglesPortOut;  // port for sending servo data to iKinGazeCtrl
    BufferedPort<Vector>             *robotPortIn;        // port for reading version and vergence angles

    double *gain;                                         // constant of proportionality in the angle computation
    double *fxRight;                                      // focal length in x and y dimension of right camera lens
    double *fyRight;
    double *cxRight;                                      // camera centre coordinates of right camera
    double *cyRight;

public:

    /* class methods  */

    SmoothPursuitThread(BufferedPort<Bottle > *wtaUnitsPortIn,   
                        BufferedPort<Vector>  *robotPortIn,
                        BufferedPort<Bottle>  *gazeAnglesPortOut, 
                        double                *gain,
                        double                *fxRight,
                        double                *fyRight,
                        double                *cxRight,
                        double                *cyRight);

   bool threadInit();     
   void threadRelease();
   void run(); 
};


class SmoothPursuit: public RFModule

{
private:
    
    /* class variables  */
 
    bool debug;

    double fxRight;
    double fyRight;
    double cxRight;
    double cyRight;

    /* ports */

    BufferedPort<Bottle >            wtaUnitsPortIn;     // port for acquiring WTA covert foci of attention
    BufferedPort<Bottle>             gazeAnglesPortOut;  // port for sending servo data to iKinGazeCtrl
    BufferedPort<Vector>             robotPortIn;        // port to get version and version angles
    Port handlerPort;                                    // port to handle messages

    /* module parameter */

    double gain;                                         // constant of proportionality in the angle computation

    /* module name */

    string moduleName;

    /* port names */

    string wtaUnitsPortName;
    string gazeAnglesPortName;
    string handlerPortName; 
    string cameraConfigFilename;
    string robotPortName;  

    /* pointer to a new thread to be created and started in configure() and stopped in close() */

    SmoothPursuitThread *smoothPursuitThread;
 
 public:

    /* class methods */

    SmoothPursuit();
    ~SmoothPursuit();
    double getPeriod();
    bool respond(const Bottle& command, Bottle& reply);
    bool configure(yarp::os::ResourceFinder &rf);
    bool close();
    bool interruptModule();
    bool updateModule();
};


#endif // __ICUB_CROSSPOWERSPECTRUMVERGENCE_MODULE_H__
/* empty line to make gcc happy */


