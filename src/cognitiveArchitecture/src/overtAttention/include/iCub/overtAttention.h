/**
 *
 * @ingroup icub_module
 * \defgroup icub_overtAttention overtAttention
 *
 * Convert the coordinates of a covert focus of attention to the relative version (azimuth and elevation) angles 
 * of the saccade required to centre eye gaze on that point, effect the saccade, acquire an image after the saccade,
 * and output that image. An inhibition of return (IOR) image based on previous saccades is output (typically to the covert
 * focus of attention module) BEFORE the coordinates are input (typically from the covert focus of attention module).
 *
 * \section intro_sec Description
 *
 * Given as input the results of a covert attentional process (such as the Selective Tuning Model [Tstotsos et al 1995]), 
 * select one of a set of winning units (i.e. potential foci of attention) as the overt focus of attention, 
 * and convert its associated coordinates to the relative gaze angles required to saccade to that fixation point.
 *
 * The saccade is effected by sending as output the azimuth, elevation, and vergence angles of desired fixation point
 * in a form that is compatible with the iKinGazeCtrl contoller. These are output as absolute angles to avoid problems with drift.
 * Since the required gaze direction corresponding to the saccade is relative to the current gaze, the absolute angles are computed 
 * by reading the current encoder values and adding the required change in gaze angles derived from the covert attentional process.
 * Note that if the required relative gaze angles are less than a minimum angular threshold, no request is sent to iKinGazeCtrl.
 * This threshold is provided as an argument to the module.
 *
 * The mapping from the covert focus of attention coordinates to gaze angles is computed from the extrinsic parameters of the camera
 * (specifically the focal length), assuming a simple pin-hole camera model.
 * Since this simple model is not accurate and to compensate for this, the saccade movement is damped by weighting the angles by
 * a fractional damping factor. This damping factor can be specified as a module parameter.
 * 
 * The vergence angle is set to the encoder value (implying no change) since it is computed separately using the binocularVergence module.
 *
 * The output should be connected to the /angles:i port of the iKinGazeCtrol module (e.g. /icub/iKinGazeCtrl/head/angles:i)
 *
 * After the saccade has been effected, the module then acquires a visual image and outputs it.
 *
 * The module implements an inhibition of return (IOR) mechanism to ensure that the selected focus for overt attention 
 * is not one that was selected in the very recent past.  This mechanism is implemented by positioning a Gaussian-weighted
 * inhibitory function at each fixation point.  This function decays with time, with the time constant for decay
 * being provided as a parameter to the module, along with the standard deviation and support for the Gaussian fuction.
 * The locations of previous Gaussian inhibitory functions are adjusted after each new saccade to ensure they are positioned
 * properly with respect to the new fixation point (which will be the centre of the image after the saccade has completed).
 * The standard deviation (in pixels) of the IOR Gaussian and the decay time (in seconds) can all be set by module parameters.
 *
 * The module also implements lateral inhibition to deemphasize scene information either side of the saccade direction.
 * This is a form of selective attention in which attention is focussed on the area in the scene where the robot will act.
 * The standard deviation of the Gaussian function for lateral inhibition is set as a module parameter.
 * 
 * Finally, the module also implements a capacity for habituation to prevent the attention from dwelling too long on one fixation point.
 * This is implemented in a similar way to IOR, with a Gaussian-weighted function centred on the current fixation point.
 * In this case, the amplitude of the Gaussian increases with time.
 *
 * The inhibition of return, lateral inhibition, and habituation values are output as a single image either for use by other modules 
 * (or for visualization purposes), such as the selectiveTuning module which can use the weights as a negative bias in subsequent 
 * winner-take-all competitions.
 *
 * These values are also overlaid on the input visual image and output on a separate image for visualization purposes. 
 *
 * NB: the inhibition of return / lateral inhibition / habituation image is output BEFORE the coordinates are input. 
 * This allows the overtAttention module to control the covert attent process (typically through the selectiveTuning module).
 * Depending on the IOR parameter values (decay time, standard deviation), the lateral inhibition parameter value, 
 * and the habituation parameter values (growth time, standard deviation), 
 * the overtAttention and the selectiveTuning modules together will cooperate to scan some or all of the most salient points in the image.
 *
 * A covert attention mode is also provided. This suppresses the associated adjustment of the IOR locations
 * so that the positions of all IOR locations remains at the original focus of attention. In addition, the input image
 * is shifted so that the selected focus of attention is centred before being output.  This mode is useful if you 
 * want the iCub to scan a scene with a fixed gaze and without saccadic eye movements.
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
 * - \c from \c overtAttention.ini \n
 *   specifies the configuration file 
 * 
 * - \c context \c overtAttention/conf \n
 *   specifies the sub-path from $ICUB_ROOT/icub/app to the configuration file
 * 
 * - \c name \c overtAttention \n
 *   specifies the name of the module (used to form the stem of module port names)
 *
 * - \c cameraConfig \c iCubEyes.ini \n
 *
 *   This is a file with at least the intrinsic camera parameters for the left and right cameras,
 *   listed under to group headings \c [CAMERA_CALIBRATION_LEFT] and \c [CAMERA_CALIBRATION_RIGHT]
 *   The parameters that are required for computation of the version angles are \c fx and \c fy 
 *   Also, we assume that the right eye/camera is dominant so we use \c fx and \c fy  from the right camera 
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
 * - \c iorDecay                           \c 1 \n
 *   The time interval over which the mean value of the Gaussian function should decay to zero (seconds).
 * 
 * - \c iorStdDev                         \c 5  \n
 *   The standard deviation of the Gaussian function used to determine the time-varying level of inhibition 
 *   return to a previously-selected focus of overt attention (pixels).
 *
 * - \c habituationGrowth                  \c 5 \n
 *   The time interval over which the mean value of the Gaussian function should grow to a value of 1.0 (seconds).
 *
 * - \c habituationStdDev                  \c 5  \n
 *   The standard deviation of the Gaussian function used to determine the time-varying level of habituation (pixels).
 *
 * - \c lateralInhibitionStdDev            \c 100 \n
 *   The time interval over which the mean value of the Gaussian function should decay to zero (seconds).
 * 
 * - \c dampingFactor                       \c 0.4 \n
 *   multiplicative constant to compute an underestimate of the required gaze angles and avoid overshoot (0 <= dF <= 1.0)
 *
 * - \c minimumAngularChange                \c 1.0 \n
 *   a saccade is executed only if the change in azimuth or elevation angles is greater than this value (degrees)
 *
 * - \c covertMode \c 0 \n
 *   specifies whether or not to engage covert attention mode (see above)
 *   (default 0 is not to do so, in which case overt attention with saccadic eye movements is engaged)
 *
 * - \c visualImageInPort                \c /visualImage:i \n  
 *   specifies the input port name of the image coming from the camera
 * 
 * - \c wtaUnitsInPort                   \c /wtaUnits:i  \n
 *   specifies the input port name for the data specifying the number and coordinates of the potential foci of attention  \n 
 *
 * - \c headStateInPort                  \c /headState:i  \n  
 *   specifies the input port name of the head encoder values  
 * 
 * - \c iorImageOutPort                  \c /iorImage:o   \n 
 *   specifies the output port name for image of the IOR function values. \n
 *
 * - \c visualImageOutPort               \c /visualImage:o   \n 
 *   specifies the output port name for the visual image that is acquired after the saccade. \n
 *
 * - \c weightedImageOutPort             \c /weightedImage:o   \n 
 *   specifies the output port name for the input visual image weighted by the IOR function values.
 *   Typically, this is used for visualization purposes. \n
 *
 * - \c gazeAnglesOutPort                \c /gazeAngles:o  \n
 *   The azimuth, elevation, and vergence angles of the fixation point on which the eye gaze should converge.
 *   Typically, this will be connected to the /angles:i port of the iKinGazeCtrl module (e.g. /icub/iKinGazeCtrl/head/angles:i)
 *
 * All these port names will be prefixed by \c /overtAttention or whatever else is specifed by the name parameter.
 *
 * \section portsa_sec Ports Accessed
 * 
 * - \c /headState:i 
 *                      
 * \section portsc_sec Ports Created
 *
 * <b>Input ports</b>
 *
 * - \c /overtAttention \n
 *   This port is used to change the parameters of the module at run time or stop the module  \n
 *
 *   The following commands are available  \n
 * 
 *   \c help  \n
 *   \c quit  \n
 *   \c set \c iordecay  \c <n>   ... set the IOR Gaussian decay time in seconds (seconds; possibly fractional) \n
 *   \c set \c iorstd    \c <n>   ... set the IOR Gaussian standard deviation (pixels, possibly fractional) \n
 *   \c set \c habgrowth \c <n>   ... set the habituation Gaussian growth time (seconds, possibly fractional)  \n
 *   \c set \c habstd    \c <n>   ... set the habituation Gaussian standard deviation (pixels, possibly fractional) \n
 *   \c set \c listd     \c <n>   ... set the lateral inhibition Gaussian standard deviation (pixels, possibly fractional) \n
 *   \c set \c damping   \c <n>   ... set the saccade damping factor(fractional) \n
 *   \c set \c minang    \c <n>   ... set the minimum angular  change implemented (degrees; possibly fractional) \n
 *   \c set \c covert    \c <n>   ... set the covert attention mode (0 for overt; 1 for covert) \n
 *
 *   Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *   The port is attached to the terminal so that you can type in commands and receive replies.
 *   The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /selectiveTuning
 *   This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 * - \c /overtAttention/visualImage:i   \n
 * 
 * - \c /overtAttention/wtaUnits:i      \n
 *
 *
 * <b>Output ports</b>
 *
 * - \c /overtAttention \n
 *   see above
 *
 * - \c /overtAttention/iorImage:o      \n
 *
 * - \c /overtAttention/visualImage:o   \n
 *
 * - \c /overtAttention/weightedImage:o \n
 *
 * - \c /overtAttention/gazeAngles:o    \n
 *
 *
 * <b>Port types </b>
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * \c BufferedPort<ImageOf<PixelRgb> >   \c visualImageInPort;        \n
 * \c BufferedPort<Bottle>               \c wtaUnitsInPort;           \n
 * \c BufferedPort<Vector>               \c headStateInPort           \n
 * \c BufferedPort<ImageOf<PixelRgb> >   \c iorImageOutPort;          \n
 * \c BufferedPort<ImageOf<PixelRgb> >   \c visualImageOutPort;       \n
 * \c BufferedPort<ImageOf<PixelRgb> >   \c weightedImageOutPort;     \n
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
 * \c overtAttention.ini
 * \c icubEyes.ini       
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows
 *
 * \section example_sec Example Instantiation of the Module
 *
 * <tt>overtAttention  --context overtAttention/conf  --from overtAttention.ini --configCamera icubEyes.ini</tt>
 *
 * \author 
 *
 * David Vernon
 * 
 * Copyright (C) 2011 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at \c $ICUB_ROOT/contrib/src/cognitiveArchitecture/src/overtAttention/include/iCub/overtAttention.h
 *
**/

/* 
 * Copyright (C) 2012 RobotCub Consortium, European Commission FP6 Project IST-004370
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



#ifndef __ICUB_OVERTATTENTION_MODULE_H__
#define __ICUB_OVERTATTENTION_MODULE_H__


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
  
#define MAX_SACCADES              10000   // number of saccades
#define MAX_WINNING_UNITS           10   // number of winning units in WTA competitions
#define MAX_MAPPING_COEFFICIENTS     6   // number of coefficients in the polynomial mapping between gaze coordinates and gaze angles

typedef  struct {
   double azimuth;
   double elevation;
   double t;  // time of saccade
   bool   start;
} saccade_data_type;

typedef  struct {
   double fx;
   double fy;      
   double a[MAX_MAPPING_COEFFICIENTS];
   double b[MAX_MAPPING_COEFFICIENTS];
} mapping_data_type;


void imagePositionToGazeAngles(int x, int y, mapping_data_type mapping_parameters, double *azimuth, double *elevation); 
void gazeAnglesToImagePosition(double azimuth, double elevation, mapping_data_type mapping_parameters, int *x, int *y); 
void initializeMappingCoefficients(mapping_data_type *mapping_parameters);

class OvertAttentionThread : public Thread
{
private:

    /* class variables */
  
    int debug;

    int visual_width;
    int visual_height;
    int visual_depth;
    unsigned char pixel_value;
    float float_pixel_value;
    int temp;
    int i, j, p, q;
    int x, y;
    double azimuth, elevation, vergence;
    double current_azimuth, current_elevation, current_vergence;
    double new_azimuth, new_elevation, new_vergence;
    double max_azimuth, max_elevation;
    double min_azimuth, min_elevation;
    double previous_azimuth, previous_elevation;
    double temp_azimuth, temp_elevation;
    double f_x;
    double f_y;
    float sigma;
    int controlGaze_x;
    int controlGaze_y;
    Vector  *encoderPositions;
    DVimage *iorImage;
    Bottle  *wtaBottle;

    double allowableEncoderVariation;
    double encoderSamplingPeriod;                  
 
    PixelRgb      rgbPixel;
    PixelRgbFloat floatRgbPixel;

    ImageOf<PixelRgb> *visualImage;
    ImageOf<PixelRgb> *weightedImage;
      
    bool stable;
    bool fixating;
    bool limitingMotion;
    Vector  *tempEncoderPositions;

    maxima_data_type winning_units[MAX_WINNING_UNITS];
    int number_of_winning_units;

    saccade_data_type saccades[MAX_SACCADES];
    int number_of_saccades;
    int selected_saccade;
    bool decay;
    double habituation_time;
 	    
    mapping_data_type mapping_parameters;


    /* thread parameters: they are all pointers so that they refer to the original variables in the overtAttention */

    BufferedPort<ImageOf<PixelRgb> > *visualImagePortIn;    // port for acquiring images 
    BufferedPort<Bottle >            *wtaUnitsPortIn;       // port for acquiring WTA covert foci of attention
    BufferedPort<ImageOf<PixelRgb> > *iorImagePortOut;      // port for sending image of inhibition of return weights 
    BufferedPort<ImageOf<PixelRgb> > *visualImagePortOut;   // port for sending post-saccade image
    BufferedPort<ImageOf<PixelRgb> > *weightedImagePortOut; // port for sending post-saccade image
    BufferedPort<Bottle>             *gazeAnglesPortOut;    // port for sending servo data to iKinGazeCtrl
    BufferedPort<Vector>             *robotPortIn;          // port for reading version and vergence angles

    double *iorDecay;                                       // decay of Gaussian inhibition function
    double *iorStdDev;                                      // standard deviation of Gaussian inhibition function
    double *habituationGrowth;                              // growth period of the Gaussian habituation function (seconds)
    double *habituationStdDev;                              // standard deviation of the Gaussian habituation function (pixels)
    double *lateralInhibitionStdDev;                        // standard deviation of the Gaussian function for lateral inhibition (pixels)
    double *dampingFactor;                                  // multiplicative constant < 1 to compute an underestimate of the required gaze angles and avoid overshoot
    double *minimumAngularChange;                           // a saccade is executed only if the change in azimuth or elevation is greater than this value (degrees)
    int    *covertMode;                                     // flag to indication engagement of covert mode (1)
    double *fxRight;                                        // focal lenght in x and y dimension of right camera lens
    double *fyRight;

public:

    /* class methods  */

   OvertAttentionThread(BufferedPort<ImageOf<PixelRgb> > *visualImagePortIn,  
                        BufferedPort<Bottle >            *wtaUnitsPortIn,   
                        BufferedPort<Vector>             *robotPortIn,
                        BufferedPort<ImageOf<PixelRgb> > *iorImagePortOut,  
                        BufferedPort<ImageOf<PixelRgb> > *visualImagePortOut,  
                        BufferedPort<ImageOf<PixelRgb> > *weightedImagePortOut,  
                        BufferedPort<Bottle>             *gazeAnglesPortOut, 
                        double *iorDecay,
                        double *iorStdDev,                             
                        double *habituationGrowth,  
                        double *habituationStdDev,     
                        double *lateralInhibitionStdDev,      
                        double *dampingFactor,            
                        double *minimumAngularChange,  
                        int    *covertMode,
                        double *fxRight,
                        double *fyRight);

   bool threadInit();     
   void threadRelease();
   void run(); 
};


class OvertAttention: public RFModule

{
private:
    
    /* class variables  */
 
    bool debug;

    double fxRight;
    double fyRight;

    /* ports */

    BufferedPort<ImageOf<PixelRgb> > visualImagePortIn;    // ports for acquiring and sending images
    BufferedPort<Bottle >            wtaUnitsPortIn;       // port for acquiring WTA covert foci of attention
    BufferedPort<ImageOf<PixelRgb> > iorImagePortOut;      // for sending image of inhibition of return weights 
    BufferedPort<ImageOf<PixelRgb> > visualImagePortOut;   // port for sending post-saccade image
    BufferedPort<ImageOf<PixelRgb> > weightedImagePortOut; // port for sending post-saccade image
    BufferedPort<Bottle>             gazeAnglesPortOut;    // port for sending servo data to iKinGazeCtrl
    BufferedPort<Vector>             robotPortIn;          // port to get version and version angles
    Port handlerPort;                                      // port to handle messages

    /* module parameters */

    double iorDecay;                                       // decay of Gaussian inhibition function
    double iorStdDev;                                      // standard deviation of Gaussian inhibition function
    double habituationGrowth;                              // growth period of the Gaussian habituation function (seconds)
    double habituationStdDev;                              // standard deviation of the Gaussian habituation function (pixels)
    double lateralInhibitionStdDev;                        // standard deviation of the Gaussian function for lateral inhibition (pixels)
    double dampingFactor;                                  // multiplicative constant < 1 to compute an underestimate of the required gaze angles and avoid overshoot
    double minimumAngularChange;                           // a saccade is executed only if the change in azimuth or elevation is greater than this value (degrees)
    int    covertMode;                                     // flag to indicate engagement of covert attention

    /* module name */

    string moduleName;

    /* port names */

    string visualImagePortName;
    string weightedImagePortName;
    string wtaUnitsPortName;
    string iorImagePortName; 
    string postSaccadeImagePortName; 
    string gazeAnglesPortName;
    string handlerPortName; 
    string cameraConfigFilename;
    string robotPortName;  

    /* pointer to a new thread to be created and started in configure() and stopped in close() */

    OvertAttentionThread *overtAttentionThread;
 
 public:

    /* class methods */

    OvertAttention();
    ~OvertAttention();
    double getPeriod();
    bool respond(const Bottle& command, Bottle& reply);
    bool configure(yarp::os::ResourceFinder &rf);
    bool close();
    bool interruptModule();
    bool updateModule();
};


#endif // __ICUB_CROSSPOWERSPECTRUMVERGENCE_MODULE_H__
/* empty line to make gcc happy */


