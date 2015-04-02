/**
 *
 * @ingroup icub_module
 * \defgroup icub_binocularVergence binocularVergence
 *
 * Compute histogram of disparity values and select the local maximum value which corresponds 
 * to the regions nearest to the cameras to control vergence
 *
 *
 * \section intro_sec Description
 * Determine the relative shift required to register one or more regions in two input images using the cross-power spectrum.
 *
 * The cross-power spectrum of two images is defined as
 *
 *
 * F(w_x, w_y) G*(w_x, w_y) / (|F(w_x, w_y) G(w_x, w_y)|)
 *
 *
 * where F(w_x, w_y) and G(w_x, w_y) are the Fourier tranforms of images f(x, y) and g(x, y), 
 * and G*(w_x, w_y) is the complex conjugate of G(w_x, w_y)
 * 
 *
 * The positions of local maxima in the cross-power spectrum, 
 * specifically the offset of a detected maximum from the centre of the CPS image, 
 * indicates the relative image shift required to register regions in the image.
 * This can be used to control the vergence of the two eyes.
 *
 * Typically, there will be several regions in the image with different disparities 
 * and hence different vergence angles, each with its own corresponding maximum,
 * we need a strategy to choose one of the maxima to control the fixation/vergence
 *
 * The possibilities are: 
 *
 * 1 choose the largest maximum (number 0); \n
 *   this is probably going to correspond to the object that occupies 
 *   the largest amount of the field of view (or largest energy in the image)
 *
 * 2 choose the maximum that is nearest to the centre; \n
 *   the corresponds to the object that is nearest to the current fixation point
 *
 * 3 choose the maximum that is furthest to the LEFT of the cross-power spectrum; \n
 *   this corresponds to the object that is nearest to the cameras
 *
 * Option 3 is the only option currently implemented.  In the future, some dynamic combination of these 
 * will be implemented so that the vergence behaviour tracks an object upon which gaze is directed.
 *
 * The module outputs the information to be used to control vergence in two way, one which is compatible with 
 * the controlGaze2 controller module and one which is compatible with the iKinGazeCtrl contoller.
 *
 * In the case of controlGaze2, the disparity of the selected region is ouput.  
 * This is effectively the position of the corresponding maximum in the cross-power spectrum image. 
 * It is expressed in normalized coordinates (-1, +1) so that it can be connected to the /dis port of the 
 * controlGaze2 module (e.g. /icub/controlgaze/dis)
 *
 * In the case of iKinGazeCtrl, the azimuth, elevation, and vergence anlges of desired fixation point are output.
 * These are output as relative angles, i.e. relative the current gaze direction, so that the azimuth and elevation angles 
 * are set to zero (i.e. no change) and the vergence angle is computed from the disparity and the focal length of the left camera lens.
 * This output can be connected to the /angles:i port of the iKinGazeCtrol module (e.g. /icub/iKinGazeCtrl/head/angles:i)
 *
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
 * - \c from \c binocularVergence.ini \n
 *   specifies the configuration file 
 * 
 * - \c context \c binocularVergence/conf \n
 *   specifies the sub-path from $ICUB_ROOT/icub/app to the configuration file
 * 
 * - \c name \c binocularVergence \n
 *   specifies the name of the module (used to form the stem of module port names)
 *
 * - \c cameraConfig \c iCubEyes.ini \n
 *
 *   This is a file with at least the intrinsic camera parameters for the left and right cameras,
 *   listed under to group headings \c [CAMERA_CALIBRATION_LEFT] and \c [CAMERA_CALIBRATION_RIGHT]
 *   The parameter that is required for computation of the vergence angle is \c fx since it is horizontal
 *   disparity that determines the vergence angle.  Also, we assume that the right eye/camera is dominant
 *   so we use \c fx from the right camera calibration group, viz:
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
 * <b>Configuration File Parameters</b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 * 
 * 
 * - \c std_dev                         \c 15 \n
 *   Standard deviation of the Gaussian mask used to apodize the input images; 
 *   the apodized images are output to \c /left_image:o and \c /right_image:o
 *
 * - \c number_of_maxima                 \c 2 \n 
 *   Number of local maxima (i.e. image regions a given disparity or depth) to consider in the final selection.
 * 
 * - \c threshold                       \c 20 \n
 *   Threshold for detection of maxima: integer % of global maximum.
 * 
 * - \c filter_radius                    \c 2 \n 
 *   Radius in pixels of filter used to amplify local maxima.
 * 
 * - \c non_maxima_suppression_radius    \c 5 \n
 *   Radius in pixels of the non-maxima suppression filter.
 *
 * - \c max_vergence                      \c 15 \n
 *   Maximum vergence angle (equivalent to minimum viewing distance of 25cm).
 *
 *
 * <b>Port names</b>
 *
 * - \c left_camera                      \c /left_camera:i \n
 *   Input from the left camera
 * 
 * - \c right_camera                     \c /right_camera:i \n 
 *   Input from the right camera
 * 
 * - \c head_state                       \c /head_state:i       \n  
 *   Input of the head encoder values  
 * 
 * - \c left_output                      \c /left_image:o \n 
 *   Output of the Gaussian-apodized image from the left camera 
 * 
 * - \c right_output                     \c /right_image:o \n 
 *   Output of the Gaussian-apodized image from the right camera
 * 
 * - \c cross-power_spectrum             \c /cross-power_spectrum:o \n 
 *   Output of the filtered cross-power spectrum with maxima enhancement, non-maxima suppression, and cross-hairs showing selected maxima 
 *
 * - \c vergence_disparity               \c /vergence_disparity:o  \n
 *   The disparity of the object on which the eye gaze should converge.
 *   Typically, this will be connected to the /dis port of the controlGaze2 module (e.g. /icub/controlgaze/dis)
 *
 * - \c vergence_angles                  \c /vergence_angles:o  \n
 *   The azimuth, elevation, and vergence angles of the object on which the eye gaze should converge.
 *   Typically, this will be connected to the /angles:i port of the iKinGazeCtrl module (e.g. /icub/iKinGazeCtrl/head/angles:i)
 *
 * All these port names will be prefixed by \c /binocularVergence or whatever else is specifed by the name parameter.
 *
 * <b>Port types</b>
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * \c BufferedPort<ImageOf<PixelRgb> >   left_camera     \n   
 * \c BufferedPort<ImageOf<PixelRgb> >   right_camera    \n
 * \c BufferedPort<Vector>               head_state      \n
 * \c BufferedPort<ImageOf<PixelRgb> >   left_output     \n 
 * \c BufferedPort<ImageOf<PixelRgb> >   right_output    \n   
 * \c BufferedPort<ImageOf<PixelRgb> >   cross-power_spectrum \n       
 * \c BufferedPort<Vector>               vergence_disparity: [horizontal_disparity vertical_disparity]\n        
 * \c BufferedPort<Bottle>               vergence_angles: ["rel" azimuth elevation vergence]       
 *         
 * 
 * \section portsa_sec Ports Accessed
 * 
 * None.
 *                      
 * \section portsc_sec Ports Created
 *
 *  <b>Input ports</b>
 *
 *  - \c /binocularVergence \n
 *    This port is used to change the parameters of the module at run time or stop the module.
 *    The following commands are available
 * 
 *    help \n
 *    quit \n
 *    set std <n>   ... set the standard deviation \n 
 *    set max <n>   ... set the number of maxima to detect \n
 *    set thr <n>   ... set the threshold for detection \n
 *    (where <n> is an integer number)
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name \c parameter \c value .
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    You can also connect to it using \c yarp \c rpc \c /binocularVergence 
 *
 *  - \c /binocularVergence/left_camera:i
 * 
 *  - \c /binocularVergence/right_camera:i
 *
 * - \c /binocularVergence/head_state:i \n
 *   This port needs to be connected to \c /icub/head/state:o to get the current version and vergence values.
 *
 * <b>Output ports</b>
 *
 * - \c /binocularVergence/binocularVergence
 * 
 * - \c /binocularVergence/left_image:o
 * 
 * - \c /binocularVergence/right_image:o
 * 
 * - \c /binocularVergence/cross-power_spectrum:o
 *
 * - \c /binocularVergence/vergence_disparity:o
 * 
 * - \c /binocularVergence/vergence_angles:o
 *
 * \section in_files_sec Input Data Files
 * None
 *
 * \section out_data_sec Output Data Files
 *
 * None
 *
 * \section conf_file_sec Configuration Files
 *
 * \c binocularVergence.ini
 * \c icubEyes.ini       
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows
 *
 * \section example_sec Example Instantiation of the Module
 *
 * <tt>binocularVergence  --context binocularVergence/conf  --from binocularVergence.ini --configCamera icubEyes.ini</tt>
 *
 * \author 
 *
 * David Vernon
 * 
 * Copyright (C) 2010 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at \c $ICUB_ROOT/contrib/src/cognitiveArchitecture/src/binocularVergence/include/iCub/binocularVergence.h
 *
**/

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
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


/*
 * Audit Trail
 * -----------
 * 18/07/07  Started work on the development of a YARP version of this module DV
 * 30/07/09  Migrated to the RFModule class to allow use of the resource finder DV
 * 17/08/09  Amended to comply with iCub Software Development Guidelines DV
 * 24/08/09  Implemented Thread-based execution DV
 * 25/08/09  Implemented run-time modification of parameters using portHandler DV
 * 13/10/10  Migrated to /contrib  fourierVision.h is now in /iCub/vis instead of /iCub DV
 * 20/10/10  Added vergence_angles output for compatibility with iKinGazeCtrl      DV
 * 24/10/10  Added camera calibration configuration file to provide the focal length 
 *           of the cameras to be used in the computation of the vergence angle DV
 * 11/11/10  Improved code efficiency. Removed raw cross-powerspectrum output DV
 * 12/11/10  Added check on maximum absolute vergence angle: input from the /head port and added a new module parameter
 * 20/04/11  Fixed bug in orderly shutdown of input ports (robotPort, leftImage, rightImage) 
 *           by making acquisition loops include a termination check by calling isStopping() DV
 */


#ifndef __ICUB_CROSSPOWERSPECTRUMVERGENCE_MODULE_H__
#define __ICUB_CROSSPOWERSPECTRUMVERGENCE_MODULE_H__


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

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;


/* fourierVision includes */

#include "iCub/fourierVision.h"
  
class BinocularVergenceThread : public Thread
{
private:

    /* class variables */
  
    int debug;

    DVimage   *image_a;
    DVimage   *image_b;
    DVimage   *image_c;
    DVimage   *image_d;

    int width;
    int height;
    int depth;
    int image_size;
    unsigned char pixel_value;
    float float_pixel_value;
    int temp;
    int i, j, p, q;
    int x, y;
    float disparity_x, disparity_y;
    double azimuth, elevation, vergence;
    double current_azimuth, current_elevation, current_vergence;
    double f_x;
    double f_y;
    float sigma;
    float controlGaze_x;
    float controlGaze_y;
    Vector  *encoderPositions;

    PixelRgb rgb_pixel;

    ImageOf<PixelRgb> *imgIn1;
    ImageOf<PixelRgb> *imgIn2;
  
    maxima_data_type maxima[10];
 	    
    /* thread parameters: they are all pointers so that they refer to the original variables in the binocularVergence */

    BufferedPort<ImageOf<PixelRgb> > *portIn1;  // ports for acquiring and sending images
    BufferedPort<ImageOf<PixelRgb> > *portIn2;  //  
    BufferedPort<ImageOf<PixelRgb> > *portOut1; //
    BufferedPort<ImageOf<PixelRgb> > *portOut2; //
    BufferedPort<ImageOf<PixelRgb> > *portOut3; //
    BufferedPort<Vector>             *portOut4; // port for sending servo data to controlGaze
    BufferedPort<Bottle>             *portOut5; // port for sending servo data to iKinGazeCtrl
    BufferedPort<Vector>             *robotPort;// port for reading version and vergence angles

    int *threshold;                            // % of maximum value
    int *filter_radius;                        // pixels
    int *number_of_maxima;                     // cardinal number
    int *non_maxima_suppression_radius;        // pixels
    int *std_dev;                              // % of image width
    double *fx_right;                          // focal length, x axis, right camera
    double *max_vergence;                      // maximum absolute vergence angle


public:

    /* class methods  */

   BinocularVergenceThread(BufferedPort<ImageOf<PixelRgb> > *imageIn1,  
                                    BufferedPort<ImageOf<PixelRgb> > *imageIn2,   
                                    BufferedPort<Vector>             *robotPort,
                                    BufferedPort<ImageOf<PixelRgb> > *imageOut1,  
                                    BufferedPort<ImageOf<PixelRgb> > *imageOut2, 
                                    BufferedPort<ImageOf<PixelRgb> > *imageOut3,  
                                    BufferedPort<Vector>             *vergenceOut4, 
                                    BufferedPort<Bottle>             *vergenceOut5, 
                                    int *thresh,                             
                                    int *filterRadius,                 
                                    int *numMaxima,
                                    int *suppressionRadius, 
                                    int *stdDev,
                                    double *fxRight,
                                    double *maxVergence);

   bool threadInit();     
   void threadRelease();
   void run(); 
};


class BinocularVergence: public RFModule

{
private:
    
    /* class variables  */
 
    bool debug;

    /* ports */

    BufferedPort<ImageOf<PixelRgb> > portIn1;  // ports for acquiring and sending images
    BufferedPort<ImageOf<PixelRgb> > portIn2;  //  
    BufferedPort<ImageOf<PixelRgb> > portOut1; //
    BufferedPort<ImageOf<PixelRgb> > portOut2; //
    BufferedPort<ImageOf<PixelRgb> > portOut3; //
    BufferedPort<Vector>             portOut4; // port for sending servo data to controlGaze
    BufferedPort<Bottle>             portOut5; // port for sending servo data to iKinGazeCtrl
    BufferedPort<Vector>             robotPort;// port to get version and version angles
    Port handlerPort;                          // port to handle messages

    /* module parameters */

    int threshold;                            // % of maximum value
    int filter_radius;                        // pixels
    int number_of_maxima;                     // cardinal number
    int non_maxima_suppression_radius;        // pixels
    int std_dev;                              // % of image width
    double fx_right;                          // focal length, x axis, right camera
    double max_vergence;                      // maxiumum vergence angle

    /* module name */

    string moduleName;

    /* port names */

    string leftCameraPortName;
    string rightCameraPortName;
    string leftImagePortName; 
    string rightImagePortName;
    string binocularPortName; 
    string vergenceDisparityPortName; 
    string vergenceAnglesPortName; 
    string handlerPortName; 
    string cameraConfigFilename;
    string robotPortName;  

    /* pointer to a new thread to be created and started in configure() and stopped in close() */

    BinocularVergenceThread *binocularVergenceThread;
 
 public:

    /* class methods */

    BinocularVergence();
    ~BinocularVergence();
    double getPeriod();
    bool respond(const Bottle& command, Bottle& reply);
    bool configure(yarp::os::ResourceFinder &rf);
    bool close();
    bool interruptModule();
    bool updateModule();
};


#endif // __ICUB_CROSSPOWERSPECTRUMVERGENCE_MODULE_H__
/* empty line to make gcc happy */


