/** 
 * @ingroup icub_module
 *
 * \defgroup icub_laplacianOfGaussian laplacianOfGaussian
 *
 * Computes the Laplacian of Gaussian and zero-crossings of an input intensity image.
 * See the following publications for details of the technique
 *
 * Vernon, D. and Sandini, G. (Eds.) 1992. Parallel Computer Vision: The VIS à VIS System, Ellis-Horwood, UK
 * 
 * The standard deviation of the Gaussian function 
 * are provided as parameters to the module.
 *
 * The module takes one input: the port on which image is acquired.
 *
 * The module produces three outputs: the Laplacian of Gaussian and zero-crossings,
 * and a plot of the zero-crossing superimposed on the input image.
 * The generation of the zero-crossings and plot images are optional.
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
 * - \c from \c laplacianOfGaussian.ini       \n
 *   specifies the configuration file
 *
 * - \c context \c laplacianOfGaussian/conf  \n 
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c laplacianOfGaussian \n         
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * - \c robot \c icub \n          
 *   specifies the name of the robot (used to form the root of robot port names)
 *
 * 
 * <b>Module Configuration File Parameters</b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file e.g. \c laplacianOfGaussian.ini 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 * - \c visualImageInPort \c /visualImage:i \n  
 *   specifies the input port name for the visual images to be processed by the module
 *
 * - \c logImageOutPort \c /logImage:o   \n 
 *   specifies the output port name for the image representing the Laplacian of Gaussian (LoG) of the input image   
 *
 * - \c zcImageOutPort \c /zcImage:o   \n 
 *   specifies the output port name for the image representing the zero-crossings of the LoG of the input image
 *
 * - \c plotImageOutPort \c /plotImage:o   \n 
 *   specifies the output port name for the zero-crossings superimposed on the input image
 *
 * - \c sigma \c 16 \n           
 *   specifies the standard deviation of the Gaussian.
 *
 * - \c significantZC \n           
 *   a flag which, if present, causes the zero-crossing to be filtered, removing those that are not significant
 *
 * - \c noZC  \n
 *   a flag which, if present, inhibits the generation and output of the zero-crossing image 
 *
 * - \c noPlot  \n   
 *   a flag which, if present, inhibits the generation and output of the plot image with the zero-crossing superimposed on the input image
 *
 *
 * All the port names will be prefixed by \c /laplacianOfGaussian or whatever else is specifed by the name parameter.
 *
 * \section portsa_sec Ports Accessed
 * 
 * - none
 *                      
 * \section portsc_sec Ports Created
 *
 * <b>Input ports</b>
 *
 * - \c /laplacianOfGaussian \n
 *   This port is used to change the parameters of the module at run time or stop the module
 *   The following commands are available
 * 
 *   \c help \n
 *   \c quit
 *   \c set \c sigma  \c <m>   ... set the standard deviation of the Gaussian
 *
 *   Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *   The port is attached to the terminal so that you can type in commands and receive replies.
 *   The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /laplacianOfGaussian
 *   This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 * - \c /laplacianOfGaussian/visualImage:i 
 *
 * <b>Output ports</b>
 *
 * - \c /laplacianOfGaussian \n
 *   see above
 *
 * - \c /laplacianOfGaussian/logImage:o 
 *
 * - \c /laplacianOfGaussian/zcImage:o 
 *
 * - \c /laplacianOfGaussian/plotImage:o 
 *
 * <b>Port types </b>
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * \c BufferedPort<ImageOf<PixelRgb> >        \c visualImageInPort;        \n
 * \c BufferedPort<ImageOf<PixelRgbFloat> >   \c logImageOutPort;    \n
 * \c BufferedPort<ImageOf<PixelRgbFloat> >   \c zcImageOutPort;        \n
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
 * \c laplacianOfGaussian.ini         in \c $ICUB_ROOT/app/laplacianOfGaussian/conf \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>laplacianOfGaussian --name laplacianOfGaussian --context laplacianOfGaussian/conf --from laplacianOfGaussian.ini --robot icub</tt>
 *
 * \author 
 *
 * David Vernon
 * 
 * Copyright (C) 2011 RobotCub Consortium
 * 
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 * 
 * This file can be edited at \c $ICUB_ROOT/contrib/src/laplacianOfGaussian/include/iCub/laplacianOfGaussian.h
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


#ifndef __ICUB_LAPLACIANOFGAUSSIAN_MODULE_H__
#define __ICUB_LAPLACIANOFGAUSSIAN_MODULE_H__

/* System includes */

#include <stdlib.h>
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


class LaplacianOfGaussianThread : public Thread
{
private:

   /* class variables */

   int    x, y;
   int    width, height, depth;    // dimensions of the image

   bool debug; 
   
   PixelRgb          rgbPixel;
   PixelRgbFloat     floatRgbPixel;
   ImageOf<PixelRgb> *intensityImage;
   DVimage           *intensityInput;
   DVimage           *logOutput;
   DVimage           *zcOutput;
   DVimage           *plotOutput;

  	    
   /* thread parameters: they are pointers so that they refer to the original variables in laplacianOfGaussian */

   BufferedPort<ImageOf<PixelRgb> >      *visualImagePortIn;
   BufferedPort<ImageOf<PixelRgbFloat> > *logImagePortOut;
   BufferedPort<ImageOf<PixelRgbFloat> > *zcImagePortOut;
   BufferedPort<ImageOf<PixelRgb> >      *plotImagePortOut;

   float *sigma;    
   bool  *significantZC;
   bool  *noZC;
   bool  *noPlot;

public:

   /* class methods */

   LaplacianOfGaussianThread(BufferedPort<ImageOf<PixelRgb> >      *visualImageIn, 
                             BufferedPort<ImageOf<PixelRgbFloat> > *logImageOut, 
                             BufferedPort<ImageOf<PixelRgbFloat> > *zcImageOut, 
                             BufferedPort<ImageOf<PixelRgb> >      *plotImageOut, 
                             float *sigma,
                             bool  *significantZC,
                             bool  *noZC,
                             bool  *noPlot); 
   bool threadInit();     
   void threadRelease();
   void run(); 
};


class LaplacianOfGaussian:public RFModule
{
   /* module parameters */

   string moduleName;
   string robotName; 
   string visualImageInputPortName;
   string logOutputPortName;  
   string zcOutputPortName;  
   string plotOutputPortName;  
   string handlerPortName;
   float  sigma;
   bool   significantZC;
   bool   noZC;
   bool   noPlot;
 

   /* class variables */

   bool debug;

   BufferedPort<ImageOf<PixelRgb> >      visualImageIn;      
   BufferedPort<ImageOf<PixelRgbFloat> > logImageOut;     
   BufferedPort<ImageOf<PixelRgbFloat> > zcImageOut;     
   BufferedPort<ImageOf<PixelRgb> >      plotImageOut;     
   Port                                  handlerPort;      //a port to handle messages 

   /* pointer to a new thread to be created and started in configure() and stopped in close() */

   LaplacianOfGaussianThread *laplacianOfGaussianThread;

public:
   LaplacianOfGaussian();
   bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
   bool interruptModule();                       // interrupt, e.g., the ports 
   bool close();                                 // close and shut down the module
   bool respond(const Bottle& command, Bottle& reply);
   double getPeriod(); 
   bool updateModule();
};




/*****************************************************************************/
/*                                                                           */
/*              +------------------------------------------+                 */
/*              |                                          |                 */
/*              |    IMAGE AND MOVEMENT UNDERSTANDING      |                 */
/*              |                                          |                 */
/*              |           ESPRIT PROJECT 419             |                 */
/*              |                                          |                 */
/*              +------------------------------------------+                 */
/*                                                                           */
/*      COPYRIGHT NOTICE                                                     */
/*      ----------------                                                     */
/*                                                                           */
/*      This software was developed at the Department of Computer Science,   */
/*      Trinity College Dublin, Ireland, in co-operation with the            */
/*                                                                           */
/*      Department of Communication, Computer and Systems Science,           */
/*      University of Genoa, Italy;                                          */
/*                                                                           */
/*      Department of Experimental Psychology,                               */
/*      University of Nijmegen, The Netherlands;                             */
/*                                                                           */
/*      Video Display Systems, Florence, Italy;                              */
/*                                                                           */
/*      Computer Applied Techniques Ltd., Dublin, Ireland.                   */
/*                                                                           */
/*      This research was funded by the Commission of the European           */
/*      Communities under the European Strategic Program for Research and    */
/*      Development in Information Technologies (ESPRIT).                    */
/*      This software is classified as foreground information and is subject */
/*      to the copyright conditions laid down by Article 5 and Annex 4 of    */
/*      the Esprit contract for project 419.                                 */
/*                                                                           */
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*      Date **06-07-85**                                                    */
/*                                                                           */
/*      Module Name:    dsdef                                                */
/*      Version Number: 1.1                                                  */
/*                                                                           */
/*			Two new fields in contour_descriptor: mean_curvature             */
/*                                                s_d_curvature              */
/*     				                                                         */
/*      Machine dependent code: *No*                                         */
/*                                                                           */
/*      Module Type:  Data Structure.                                        */
/*      Module Description: Definition of Virtual Image System               */
/*                          data structure types.                            */
/*      Module Index: [stub data structures]                                 */
/*                    array_image_type                                       */
/*                    int_array_image_type                                   */
/*                    contour_image_type                                     */
/*                    contour_descriptor_type                                */
/*                    slope_image_type                                       */
/*                    orientation_image_type                                 */
/*                    range_image_type                                       */
/*                    depth_image_type                                       */
/*                    disparity_image_type                                   */
/*                    velocity_image_type                                    */
/*                    intensity_image_type                                   */
/*                    convolution_image_type                                 */
/*                    zero_crossing_image_type                               */
/*                    pyramid_descriptor_type                                */
/*                    image_descriptor_type                                  */
/*                    framestore_descriptor_type                             */
/*                    view_type                                              */
/*                    related_images_type                                    */
/*                    region_image_type                                      */
/*                    region_crossing_image_type                             */
/*                    region_table_type                                      */
/*                    region_tree_type                                       */
/*                    raw_primal_image_type                                  */
/*                                                                           */
/*      Related Documents: "Provisional Specification for a Virtual          */
/*                          Image System", Internal Report.                  */
/*                                                                           */
/*****************************************************************************/

#define DOS
#define ZERO 0

/*****************************************************************************/
/*                                                                           */
/* ---  File Name: dsdef.h                                                   */
/*                                                                           */
/* ---  Functional Description: Data Structure Type Definitions for          */
/*                              Virtual Image System.                        */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced: none                 */
/*                                                                           */
/* ---  Calling Procedure:  #include "dsdef.h"                               */
/*                                                                           */
/* ---  Input Parameters:  n/a                                               */
/*                                                                           */
/* ---  Output Parameters: n/a                                               */
/*                                                                           */
/* ---  Global Parameters: n/a                                               */
/*                                                                           */
/* ---  Local Variables:   n/a                                               */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: David Vernon - TCD.                                          */
/*                                                                           */
/* ---  Revisions                                                            */
/*                                                                           */
/*      date:      28/8/86                                                   */
/*      revision:  FG100AT frame grabber added - Jon Kennedy.                */
/*      reason:                                                              */
/*                                                                           */
/*      date:      10/10/86                                                  */
/*      revision:  Virtual Frame Store added - Massimo Tistarelli            */
/*      reason:                                                              */
/*                                                                           */
/*      date:      31/10/86                                                  */
/*      revision:  Eidobrain framestore added - Massimo Tistarelli           */
/*      reason:                                                              */
/*                                                                           */
/*      date:      12/11/86                                                  */
/*      revision:  Added conditional compile flag to typedef of              */
/*                 array_image_type                                          */
/*                                              Sean O'Neill.                */
/*      reason:    To force char definition on VAX VMS from 8 bits signed to */
/*                 8 bits unsigned. This should also work for VAX'S running  */
/*                 UNIX.                                                     */
/*                                                                           */
/*      date:      19/11/86                                                  */
/*      revision:  velocity_image_type re-implemented - David Vernon         */
/*      reason:                                                              */
/*                                                                           */
/*      date:      03/12/86                                                  */
/*      revision:  Added definition of the number of angular values for      */
/*                 orientation (ANGULAR_VALUES = 254) Massimo Tistarelli     */
/*      reason:                                                              */
/*                                                                           */
/*      date:      28/02/87                                                  */
/*      revision:  Depth_image_type implemented                              */
/*                                                                           */
/*      reason:    David Vernon                                              */
/*                                                                           */
/*      date:      10/03/87                                                  */
/*      revisor:   David Vernon                                              */
/*      reason:    int_array_image_type implemented.                         */
/*                                                                           */
/*      date:      10/03/87                                                  */
/*      revisor:   David Vernon                                              */
/*      reason:    region_image_type and region_crossing_image_type          */
/*                 implemented; dummy region_table_type and region_tree_type */
/*                 implemented.                                              */
/*                                                                           */
/*      date:      26/04/87                                                  */
/*      revision:  range_image_type implemented.                             */
/*                                                                           */
/*      date:      08/05/87                                                  */
/*      revision:  Added region_data structure to contain the regions        */
/*                 statistics                         Massimo Tistarelli     */
/*      reason:                                                              */
/*                                                                           */
/*      date:      23/07/87                                                  */
/*      revision:  MAX_PYRAMID_NUMBER defined equal to 100                   */
/*      reason:                                                              */
/*                                                                           */
/*      date:      30/07/87                                                  */
/*      revisor:   David Vernon                                              */
/*      reason:    contour descriptor updated to include mean and            */
/*                 standard deviation of depth values.                       */
/*                                                                           */
/*      date:      08/08/87                                                  */
/*      revisor:   David Vernon                                              */
/*      reason:    Raw primal sketch data-structures integrated.             */
/*                                                                           */
/*      date:      17/11/87                                                  */
/*      revisor:   Mairead Flanagan                                          */
/*      reason:    MAX_NUM_RELATIONSHIPS changed to 20                       */
/*                                                                           */
/*      date:      04/07/88                                                  */
/*      revisor:   David Vernon                                              */
/*      reason:    New field in region tree node: select                     */
/*                                                                           */
/*      date:      12/07/88                                                  */
/*      revisor:   David Vernon                                              */
/*      reason:    New structure which is used for parsing lines of text.    */
/*                                                                           */
/*      date:      03/11/88                                                  */
/*      revisor:   David Vernon                                              */
/*      reason:    New field in framestore descriptor: serial number         */
/*                                                                           */
/*      date:      17/01/89                                                  */
/*      revisor:   Ken Dawson                                                */
/*      revision:  New fields in raw_primal_descriptor_type:                 */
/*                    depth1, depth2, depth3, depth4                         */
/*                 New fields in temp_descriptor_list_item_type              */
/*                    edge1_depth, edge2_depth, curr1_depth, curr2_depth     */
/*      reason:    To allow integration of depth information.                */
/*                                                                           */
/*      date:      02/03/89                                                  */
/*      revision:  PCVISIONplus framestore added - Mairead Flanagan          */
/*      reason:                                                              */
/*                                                                           */
/*****************************************************************************/

#define DIM(x)   sizeof(x)/sizeof(x[0]) /* return the size of an array */

/*****************************************************************************/
/*                                                                           */
/*                            STUB STRUCTURES                                */
/*                                                                           */
/*****************************************************************************/

typedef int framestore_image_type      ;

/*****************************************************************************/
/*                                                                           */
/*                            ARRAY IMAGE TYPE                               */
/*                                                                           */
/*                   also     INTENSITY IMAGE TYPE                           */
/*                            CONVOLUTION  IMAGE TYPE                        */
/*                            ZERO-CROSSING IMAGE TYPE                       */
/*                                                                           */
/*   An array image is defined as a set of 1-D vectors of bytes; each vector */
/*   (1-D array) is pointed to be an explicit pointer which itself is stored */
/*   in a 1-D array of pointers.                                             */
/*   Thus, an array image (of bytes) is just an array of pointers to arrays  */
/*   of bytes and any variable of this type is just a pointer to a char      */
/*   pointer.  Note, however, that one may still reference an element in the */
/*   image (2-D array) using the familiar double subscript, e.g.:            */
/*                                                                           */
/*                            image[i][j] = 0;                               */
/*                                                                           */
/*****************************************************************************/

/*   Force all machine char definitions into 8 bit unsigned.  Son  12/11/86  */

 
typedef unsigned char *image_row;  /* this is a useful type definition when dealing   */
                                   /* with image arrays: it defines IMAGE_ROW as      */
                                   /* pointer to characters (or bytes).               */
                                   /* An image then is simply an array of these       */
                                   /* pointers, suitably initialised.                 */
                                   /* Refer to routine generate_image in module       */
                                   /* datasub for further details.                    */
 

typedef image_row *array_image_type;

typedef array_image_type intensity_image_type;
typedef array_image_type convolution_image_type;
typedef array_image_type zero_crossing_image_type;

/*****************************************************************************/
/*                                                                           */
/*                        INT ARRAY IMAGE TYPE                               */
/*                                                                           */
/*   An int array image is defined as a set of 1-D vectors of ints; each     */
/*   vector is pointed to be an explicit pointer which itself is stored      */
/*   in a 1-D array of pointers.                                             */
/*   Thus, an int array image (of ints) is just an array of pointer to arrays*/
/*   of ints  and any variable of this type is just a pointer to an int      */
/*   pointer.  Note, however, that one may still reference an element in the */
/*   image (2-D array) using the familiar double subscript, e.g.:            */
/*                                                                           */
/*                            image[i][j] = 0;                               */
/*                                                                           */
/*****************************************************************************/

typedef short int  *int_image_row; 

typedef int_image_row *int_array_image_type;


/*****************************************************************************/
/*                                                                           */
/*                            CONTOUR IMAGE TYPE                             */
/*                                                                           */
/*   A contour image contains exactly the same information as the zero-      */
/*   crossing image, but it represents it in a different manner.             */
/*   Specifically, the contours are represented, not by a 2-D array, but by  */
/*   a series of lists.  Each list element contains the Freeman chain code   */
/*   direction required to generate the next point on the contour; each list */
/*   represents a single contour in the image.  The lists themselves, which  */
/*   are represented by a 1-D array of words  (int types), are organised as  */
/*   a linked-list (of contours).  Each list has a header comprising the     */
/*   link fields to the next contour, to the previous contour, and to the    */
/*   associated contour descriptor.  In addition, the header contains the    */
/*   the coordinates of the contour origin and its length.  Note that this   */
/*   information is redundant in that it is also contained in the contour    */
/*   descriptor but it is convenient for some processing to have it          */
/*   available locally in the "logical" contour image.                       */
/*                                                                           */
/*   Since the lists are built dynamically, and are variable in length,      */
/*   an intermediate linked-list representation must be utilised for their   */
/*   construction.  The chain code directions are then transferred to the    */
/*   an array of suitable length and the linked list is deleted.  This       */
/*   process continues until all contours have been generated.               */
/*   The auxiliary (and temporary) linked list structures are defined here   */
/*   also.                                                                   */
/*                                                                           */
/*   David Vernon (TCD):  16-4-86.                                           */
/*                                                                           */
/*****************************************************************************/

struct contour_node_type {
   int start_x, start_y,
       length;
   int *value;
   struct contour_descriptor_type *descriptor_link;
   struct contour_node_type *next;
   struct contour_node_type *prev;
};

/*** a contour image is simply a pointer to (the first) such node in the list ***/

typedef struct contour_node_type *contour_image_type;


/* We initially construct the chain code as a linked list: the required */
/* structures are defined here also.                                    */

struct cc_node {
   int direction;
   struct cc_node *flink, *rlink;
};

#define CC_NODE_SIZE      sizeof(struct cc_node)
#define CONTOUR_NODE_SIZE sizeof(struct contour_node_type)

/*****************************************************************************/
/*                                                                           */
/*                            CONTOUR DESCRIPTOR TYPE                        */
/*                                                                           */
/*   The contour descriptor forms logical links between contours in several  */
/*   contour-based images.   The contour descriptor comprises two sets of    */
/*   fields.  The first set contains explicit links to the corresponding     */
/*   contour in each contour-based image while the second set contains       */
/*   information regarding gross contour statistics.                         */
/*                                                                           */
/*   David Vernon (TCD):  26-5-86.                                           */
/*                                                                           */
/*****************************************************************************/

struct contour_descriptor_type {
   struct contour_node_type *contour,
                            *slope,
                            *orientation,
                            *depth,
                            *disparity;
   struct velocity_node_type *velocity;

   struct contour_descriptor_type *next;
   struct contour_descriptor_type *prev;

   short int label,                     /* contour label */
             select,                    /* select/deselect flag */
             start_x, start_y,    	/* start of contour */
             length,			/* length of contour */
             mean_slope, 		/* mean slope of zero-crossing */
             s_d_slope,			/* standard deviation of same  */
             mean_orientation,		/* mean orientation of zero-crossing */
             s_d_orientation,		/* standard deviation of same  */
             mean_curvature,		/* mean curvature of contour */
             s_d_curvature,		/* standard deviation of same  */ 
             mean_depth,    		/* mean depth of contour */
             s_d_depth;     	/* standard deviation of same  */ 
};

#define CONTOUR_DESCRIPTOR_SIZE sizeof(struct contour_descriptor_type)

typedef struct contour_descriptor_type *ptr_contour_descriptor;


/*****************************************************************************/
/*                                                                           */
/*                            SLOPE IMAGE TYPE                               */
/*                            ORIENTATION IMAGE TYPE                         */
/*                            DISPARITY IMAGE TYPE                           */
/*                            DEPTH IMAGE TYPE                               */
/*                                                                           */
/*   These images are all based on the contours extracted during the         */
/*   zero-crossing phase and they make explicit some intrinsic property      */
/*   based on an analysis of (a series) of these contours.                   */
/*   These images are represented, not by 2-D arrays, but by a               */
/*   representation similar to that of the contour image, i.e., a series of  */
/*   lists, each element of the list containing the intrinsic information    */
/*   appropriate to the image type, and each list representing a single      */
/*   contour in the image.                                                   */
/*                                                                           */
/*   To reduce the software overhead in handling these image types, they are */
/*   implemented using exactly the same primitive data-structures            */
/*   and software handlers as the contour image type.                        */
/*                                                                           */
/*   David Vernon (TCD):  16-4-86.                                           */
/*                                                                           */
/*****************************************************************************/

typedef contour_image_type slope_image_type;
typedef contour_image_type orientation_image_type;
typedef contour_image_type disparity_image_type;
typedef contour_image_type depth_image_type;

/*****************************************************************************/
/* Now the angular values of an orientation image range from 1 to 254.....   */
/*                                                                           */
/*   Massimo Tistarelli (at TCD)  3/12/1986                                  */
/*                                                                           */
/*****************************************************************************/

#define ANGULAR_VALUES 254

/*****************************************************************************/
/*                                                                           */
/*                            VELOCITY IMAGE TYPE                            */
/*                                                                           */
/*   This image is similar in structure to the slope, orientation, and       */
/*   disparity images and it, too, makes explicit some contour based feature */
/*   (in this case: the flow or velocity vectors associated with image       */
/*    motion)								     */
/*   Since velocity is a vector quantity, rather than a scalar, two          */
/*   components are now required.  These are, by convention, the vector      */
/*   magnitude and phase angle.  A motion study, and the resultant velocity  */
/*   image, are based on an analysis of a sequence of contour-type images    */
/*   and, to make explicit this derivation, the velocity image also          */
/*   comprises a set of "coordinates" describing the contour BCC node of the */
/*   contour point corresponding to each velocity point.                     */
/*   In summary, a velocity image comprises a set of vector magnitudes,      */
/*   phase angles, pyramid numbers, image level numbers, contour numbers,    */
/*   and contour node offsets; there is one six-tuple for every velocity     */
/*   contour point.                                                          */
/*                                                                           */
/*   David Vernon (TCD):  19-11-86                                           */
/*                                                                           */
/*****************************************************************************/

struct velocity_node_type {
   int start_x, start_y,
       length;

   float *vector_magnitude;
   int   *phase_angle,
         *related_pyramid,
         *related_image,
         *related_contour,
         *contour_offset;

   struct contour_descriptor_type *descriptor_link;
   struct velocity_node_type *next;
   struct velocity_node_type *prev;
};

/*** a velocity image is simply a pointer to (the first) such node in the list ***/

typedef struct velocity_node_type *velocity_image_type;

#define VELOCITY_NODE_SIZE sizeof(struct velocity_node_type)

/*****************************************************************************/
/*                                                                           */
/*                            RANGE   IMAGE TYPE                             */
/*                                                                           */
/*   A range image is simply an array of integers (shorts) where each        */
/*   element represents the distance from the focal plane to a surface       */
/*   point in the image.                                                     */
/*                                                                           */
/*   David Vernon (TCD):  26-4-87.                                           */
/*                                                                           */
/*****************************************************************************/

typedef int_array_image_type range_image_type;

/*****************************************************************************/
/*                                                                           */
/*                            REGION  IMAGE TYPE                             */
/*                                                                           */
/*   A region image comprises three parts: an array of integers which        */
/*   represent the specific regions of a convolution image (this is the      */
/*   so-called region-crossing image type), an n-way tree of regional        */
/*   descriptors (representing the topology of the regional relationships    */
/*   and some useful region properties), and a table of pointers, indexed    */
/*   by the values of the region-crossing image and referencing the          */
/*   corresponding node in the region tree.                                  */
/*                                                                           */
/*   To facilitate this, a region image type is simply a structure with      */
/*   three pointer fields, each forming a link to the region_crossing,       */
/*   table, and tree.                                                        */
/*                                                                           */
/*   David Vernon (TCD):  11-3-87.                                           */
/*                                                                           */
/*   Structure containing region statistics added - MT 8/5/87                */
/*                                                                           */
/*****************************************************************************/

typedef int_array_image_type region_crossing_image_type;
typedef struct region_tree_type  *region_table_element;
typedef region_table_element     *region_table_type;


struct region_tree_type {
   short int id_number;
   int select; 
   double  area,
           average,
           sq_average;
   int  x_bar,
        y_bar;
   int  background;
   struct region_pointer *offspring;
   struct region_tree_type *parent;
};


/* structure region_pointer is simply a linked list node containing */
/* a pointer to a region descriptor.                                */

struct region_pointer {
   struct region_tree_type *region_tree;
   struct region_pointer *link;
};

struct region_image_header {
   region_crossing_image_type  array;
   region_table_type           table;
   struct region_tree_type     *tree;
	int number_of_regions;
};

typedef struct region_image_header *region_image_type;

#define REGION_IMAGE_SIZE sizeof(struct region_image_header)
#define REGION_TREE_SIZE sizeof(struct region_tree_type)
#define REGION_POINTER_SIZE sizeof(struct region_pointer)
#define REGION_TABLE_ELEMENT_SIZE sizeof(region_table_element)




/*****************************************************************************/
/*                                                                           */
/*                            PYRAMID DESCRIPTOR TYPE                        */
/*                                                                           */
/*   Comprises: pyramid number                                               */
/*              alphanumeric description                                     */
/*              link to next pyramid                                         */
/*              linked list of pointers to image descriptors.                */
/*                                                                           */
/*****************************************************************************/

#define MAX_PYRAMID_NUMBER 1000
#define DESCRIPTION_LENGTH 80

struct pyramid_descriptor_type {
   int pyramid_number;
   char description[DESCRIPTION_LENGTH];
   struct pyramid_descriptor_type *pyramid_link;
   struct pyr_node *image_link;
};


/* structure pyr_node is simply a linked list node containing */
/* a pointer to an image descriptor.                          */

struct pyr_node {
   struct image_descriptor_type *image_descriptor;
   struct pyr_node *link;
};

#define PYRAMID_DESCRIPTOR_SIZE sizeof(struct pyramid_descriptor_type)
#define PYR_NODE_SIZE sizeof(struct pyr_node)

/*****************************************************************************/
/*                                                                           */
/*                           IMAGE DESCRIPTOR TYPE                           */
/*                                                                           */
/*   Comprises: internal data-type label                                     */
/*              alphanumeric description                                     */
/*              size (rows x columns: assume row=column=integer dividend of  */
/*                                    1024)                                  */
/*              level number (note: level n = 1024/size)                     */
/*              window specification: x1,y1 ... coordinates of bottom        */
/*                                              left-hand corner             */
/*                                    x2,y2 ... coordinates of top           */
/*                                              right-hand corner            */
/*              image bit-plane mask (type integer => 32 bits)               */
/*              number of bits in image                                      */
/*              pointer to frame-store descriptor                            */
/*              pointer to image                                             */
/*              pointer to pyramid descriptor (parent structure)             */
/*                                                                           */
/*****************************************************************************/

#define FRAMESTORE_IMAGE_TYPE      0
#define INTENSITY_IMAGE_TYPE       1
#define CONVOLUTION_IMAGE_TYPE     2
#define ZERO_CROSSING_IMAGE_TYPE   3
#define CONTOUR_IMAGE_TYPE         4
#define SLOPE_IMAGE_TYPE           5
#define ORIENTATION_IMAGE_TYPE     6 
#define DISPARITY_IMAGE_TYPE       7
#define VELOCITY_IMAGE_TYPE        8
#define DEPTH_IMAGE_TYPE           9
#define REGION_IMAGE_TYPE          10
#define RANGE_IMAGE_TYPE           11

struct window_type {
   short int x1,y1,
             x2,y2;
};

union image_pointer_type {
   framestore_image_type      *framestore_link;
   array_image_type           array_link;         /* for intensity, convolution, and zero_crossing types */
   contour_image_type         contour_link;
   slope_image_type           slope_link;
   orientation_image_type     orientation_link;
   disparity_image_type       disparity_link;
   depth_image_type           depth_link;
   range_image_type           range_link;
   velocity_image_type        velocity_link;
   region_image_type          region_link;
};

struct image_descriptor_type {
   short int image_type;       /* internal data-type - see definitions above */
   char description[DESCRIPTION_LENGTH];
   short int size;
   short int level_number;
   struct window_type window;
   long int mask;
   short int number_of_bits;
   struct framestore_descriptor_type *framestore;
   struct pyramid_descriptor_type *pyramid;
   union image_pointer_type image;
   int serial_number;
};

#define IMAGE_DESCRIPTOR_SIZE sizeof(struct image_descriptor_type)

/*****************************************************************************/
/*                                                                           */
/*                         FRAMESTORE DESCRIPTOR TYPE                        */
/*                                                                           */
/*   Comprises: Address of framestore                                        */
/*              access type (memory map / input-ouput / combination)         */
/*              addresses of registers used                                  */
/*              data values used for register manipulation.                  */
/*              image bit-plane mask (type integer => 32 bits)               */
/*              number of bits in image                                      */
/*              serial number                                                */
/*                                                                           */
/*****************************************************************************/


/*                                         */
/*     define framestore access types      */
/*                                         */

#define MEMORY_MAP 0
#define INPUT_OUTPUT 1
#define COMBINATION 2

/*                                         */
/*  define framestore identification types */
/*                                         */

#define PCVISION    0
#define VDS_701     1
#define EIDOBRAIN   2
#define MICROEYE    3
#define VICOM       4
#define FG100AT     5
#define VFS         6
#define X_WINDOW    7
#define PCPLUS      8

/*   framestore-specific structures containing addresses and data  */
/*   for all registers required to handle framestore device.       */


/*** microeye ***/

struct microeye_type {
   int  y_register_high,
        y_register_low,
        x_register_high,
        x_register_low,
        store_select,
        data_register,
        r_w_pulse,
        check,
        frame_grab,
        crtc,
        ra4;

   char crtc_values[9],
        ra4_values[9],
        read_r_w_pulse,
        write_r_w_pulse;

};

/*** PC Vision ***/

struct pcvision_type {
   int c_s_low_byte,                   /* address of control/status register; low byte  */
       c_s_high_byte,                  /* address of control/status register; high byte */
       lut_address_register,           /* address of LUT address register               */
       lut_data_register,              /* address of LUT data register                  */
       mask_register,                  /* address of mask register                      */
       frame_memory_block_select,      /* address of frame memory block select register */
       reset_vertical_blank_interrupt; /* address of reset vertical blank interrupt reg.*/

   long frame_memory_base_address;     /* frame memory base address                     */

   char clear_value,                   /* clear value: written to control/status word.    */
        init_value,                    /* init value: written to control/status word.     */
        syn_value,                     /* syn value   written to control/status word.     */
        acq_value,                     /* acq value   written to control/status word.     */

        lut0,                          /* LUT 0        */
        lut1,                          /* LUT 1        */
        lut2,                          /* LUT 2        */
        lut3;                          /* LUT 3        */

};

/*** PC Vision PLUS ***/

struct pcplus_type {

   int control_register,               /* address of control                         */
       lut_control_register,           /* address of LUT control register            */
       lut_address_register,           /* address of LUT address register            */
       lut_data_register,              /* address of LUT data register               */
       gain_offset_counter,            /* address of gain and  offset counter        */
       acquire_status_register,        /* address of acquire and status register     */
       pan_register,                   /* address of pan register                    */
       scroll_register,                /* address of scroll register                 */
       mem_access_control_register,    /* address of memory access control register  */
       host_mask_register,             /* address of host mask register              */
       video_mask_register,            /* address of video mask register             */
       pixel_buffer_register;          /* address of pixel buffer register           */

  long frame_memory_base_address;      /* frame memory base address */

  char init_value,                     /* init value : written to aquire and status register  */
       clear_value,                    /* clear value : written to aquire and status register */
       syn_value,                      /* syn value : written to aquire and status register   */
       acq_value,                      /* acq value : written to aquire and status register   */
       control_value,                  /* init value for control register                     */
       mem_access_control,             /* init value for memory access control register       */

        lut0,                          /* LUT 0        */
        lut1,                          /* LUT 1        */
        lut2,                          /* LUT 2        */
        lut3,                          /* LUT 3        */
        lut4,                          /* LUT 4        */
        lut5,                          /* LUT 5        */
        lut6,                          /* LUT 6        */
        lut7;                          /* LUT 7        */

};

/*** FG100AT ***/

struct fg100at_type {
   int
       mac,       /* memory access control */
       hmr,       /* host mask register */
       vam,       /* video aquisition mask */
       pbr,       /* pixel buffer register */
       xpr,       /* x pointer register */
       ypr,       /* y pointer register */
       pcr,       /* pointer control register */
       cpu,       /* cpu address control */
       xsr,       /* x spin register */
       ysr,       /* y spin register */
       pan,       /* pan register */
       lcr,       /* lut control register */
       scr,       /* scroll register */
       bsc,       /* board status & control */
       zcr,       /* zoom control register */
       fmd;       /* frame memory data */

   long
       fma,       /* frame memory address */
       lut_red,   /* red lut address */
       lut_blu,   /* blue lut address */
       lut_gre,   /* green lut address */
       lut_inp;   /* input & feedback lut address */

   int
       mac_ini,   /* memory access z mode, pixel buffer disabled */
       pcr_ini,   /* pointer control init value */
       pcr_yai,   /* pointer control y auto increment */
       cpu_ini,   /* cpu register init */
       lcr_fmd,   /* frame memory enabled, static mode */
       lcr_lut,   /* lut memory enabled, static mode */
       bsc_acq,   /* grab images continuously */
       bsc_syn,   /* snap an image */
       bsc_rdy;   /* board ready */
};

/*** Bitmapped X-Window ***/

struct Xwindow_type {
		short size;
};

/*** Eidobrain framestore ***/

struct eido_type {
		short channel,
		      look_up,
			  size;
};

/*** Virtual FrameStore ***/

struct VFS_type {
	char file_name[40];
       };


struct framestore_descriptor_type {
   short int framestore_id;
   short int access_type;           /* refer to definitions above */
   long int mask;
   short int number_of_bits;
   int serial_number;
   union framestore_data_type {
      struct microeye_type *microeye;
      struct pcvision_type *pcvision;
      struct fg100at_type  *fg100at;
      struct eido_type *eidobrain;
      struct VFS_type      *vfs;
      struct Xwindow_type *Xwindow;
      struct pcplus_type *pcplus;
   } framestore_data;
};

#define FRAMESTORE_DESCRIPTOR_SIZE sizeof(struct framestore_descriptor_type)

/*****************************************************************************/
/*                                                                           */
/*                               VIEW TYPE                                   */
/*                                                                           */
/*   The view type is a very simple structure defining the x and y extent    */
/*   of a rectangular sub-region in am image.  It is used for delimiting     */
/*   such sub-regions during primitive image manipulation.  Several views    */
/*   can be defined at any one time; an array of view structures is defined  */
/*   in the data-structure data file dsdata.h.                               */
/*                                                                           */
/*****************************************************************************/

#define MAX_NUMBER_VIEWS 10

struct view_type {
   int x,y;
};

/*****************************************************************************/
/*                                                                           */
/*                           RELATED_IMAGES_TYPE                             */
/*                                                                           */
/*   This type defines an array of structures comprising a set of pyramid    */
/*   and image level numbers defining those contour-type images which are    */
/*   logically related according to the associated contour descriptors.      */
/*                                                                           */
/*****************************************************************************/

#define MAX_NUM_RELATIONSHIPS 20

struct related_images_type {
   int p_contour, i_contour,
       p_orientation, i_orientation,
       p_slope, i_slope,
       p_disparity, i_disparity,
       p_velocity, i_velocity,
       p_depth, i_depth;
};


/*****************************************************************************/
/*                                                                           */
/*                            RAW PRIMAL IMAGE TYPE                          */
/*                                                                           */
/*   A raw primal image contains the primitives of the raw primal sketch     */
/*   in an explicit form.  There are four primitives which are supported,    */
/*   and they are edge segments, terminations, blobs, and bars.              */
/*                                                                           */
/*   Ken Dawson (TCD): 10-12-86                                              */
/*                                                                           */
/*****************************************************************************/

struct raw_primal_image_type {
   struct raw_primal_des_list_type *contour_list;
   struct raw_primal_des_list_type *bar_list;

   int size;
};

#define RAW_PRIMAL_IMAGE_SIZE sizeof(struct raw_primal_image_type)

/*****************************************************************************/
/*                                                                           */
/*                         RAW PRIMAL DES LIST TYPE                          */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

struct raw_primal_des_list_type {
   struct raw_primal_descriptor_type *descriptor_list;

   struct raw_primal_des_list_type *next_contour;
   struct raw_primal_des_list_type *prev_contour;

   short int label,                     /* contour label        */
             select;                    /* select/deselect flag */
};

#define RAW_PRIMAL_DES_LIST_SIZE sizeof(struct raw_primal_des_list_type)

/*****************************************************************************/
/*                                                                           */
/*                         RAW PRIMAL DESCRIPTOR TYPE                        */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

struct raw_primal_descriptor_type {

   struct raw_primal_descriptor_type *next_descriptor;
   struct raw_primal_descriptor_type *prev_descriptor;
   short int primitive,
             x1,y1,depth1,
             x2,y2,depth2,
             x3,y3,depth3,
             x4,y4,depth4,
             constant_x,
             select;
   int size,
       width,
       orientation,
       slope,
       second_slope;
   double alpha,beta;
};

#define RAW_PRIMAL_DESCRIPTOR_SIZE sizeof(struct raw_primal_descriptor_type)


#define UNKNOWN_PRIMITIVE  0
#define EDGE_SEGMENT       1
#define TERMINATION        2
#define BLOB               3
#define BAR                4


#define ORIEN_MOD          ANGULAR_VALUES

/*****************************************************************************/
/*                                                                           */
/*                      TEMP DESCRIPTOR LIST ITEM TYPE                       */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

struct temp_descriptor_list_item_type {

   struct raw_primal_descriptor_type *the_descriptor;
   struct temp_descriptor_list_item_type *next;
   struct temp_descriptor_list_item_type *prev;
   struct temp_descriptor_list_type *matched_list;
   int distance,
       distance_matched;
   int edge1_depth,edge2_depth,curr1_depth,curr2_depth;
   short int edge_x1, edge_y1,
             edge_x2, edge_y2,
             curr_x1, curr_y1,
             curr_x2, curr_y2;

};

#define TEMP_DESCRIPTOR_LIST_ITEM_SIZE sizeof(struct temp_descriptor_list_item_type)



/*****************************************************************************/
/*                                                                           */
/*                        TEMP DESCRIPTOR LIST TYPE                          */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

struct temp_descriptor_list_type {

   struct temp_descriptor_list_item_type *first;
   struct temp_descriptor_list_item_type *last;

};

#define TEMP_DESCRIPTOR_LIST_SIZE sizeof(struct temp_descriptor_list_type)

#define BY_LENGTH     0
#define BY_DISTANCE   1

#define BAR_INTENSITY 75



/*****************************************************************************/
/*                                                                           */
/*                STRUCTURE USED FOR PARSING LINE OF TEXT                    */
/*                                                                           */
/*                                                                           */
/*****************************************************************************/

#define MAX_NUM_FIELDS   10   
#define MAX_FIELD_LENGTH 25

struct fieldinfo {
   int trunc;
   int len;
   char ch[MAX_FIELD_LENGTH+1];
};

struct pb {
   int too_many_fields;
   int num_fields;
   struct fieldinfo field[MAX_NUM_FIELDS];
};


struct stereo_descriptor_type {

  struct image_descriptor_type *l_1_slope_image,
                               *r_1_slope_image,
                               *l_2_slope_image,
                               *r_2_slope_image,
                               *l_3_slope_image,
                               *r_3_slope_image;

  struct image_descriptor_type *l_1_orientation_image,
                               *r_1_orientation_image,
                               *l_2_orientation_image,
                               *r_2_orientation_image,
                               *l_3_orientation_image,
                               *r_3_orientation_image;

  struct image_descriptor_type  *l_1_conv_image,
                                *r_1_conv_image,
                                *l_2_conv_image,
                                *r_2_conv_image,
                                *l_3_conv_image,
                                *r_3_conv_image;

  struct image_descriptor_type   *shift_image,
                                 *disp_prob_image;
  int no_of_steps;
  int  pyramidal, thresh_of_patch;
  int get_average;
  int conversion_factor;

  double focal_length, distance_between_cameras;
  double angle_between_cameras;

  double sigma;
  struct image_descriptor_type *left_zc_image,
                               *right_zc_image;

  struct image_descriptor_type *output_image; 

  struct image_descriptor_type *disparity_image;

};


#define sign(x,y) ((y >= 0.0) ? (x) : (-x))  /* calculate sign of x */ 
#define maximum(a,b) ((a) > (b) ? (a) : (b))
#define minimum(a,b) ((a) < (b) ? (a) : (b))
#define BLACK 0
#define WHITE 255

/*****************************************************************************/
/*                                                                           */
/* ---  File name : vismach.h                                                */
/*                                                                           */
/* ---  Functional Description : Symbolic constant definitions required to   */
/*                               interface with the operating system         */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced:        none          */
/*                                                                           */
/* ---  Calling Procedure : #include "vismach.h"                             */
/*                                                                           */
/* ---  Input Parameters :   not applicable                                  */
/*                                                                           */
/* ---  Output Parameters :  not applicable                                  */
/*                                                                           */
/* ---  Global Parameters :  not applicable                                  */
/*                                                                           */
/* ---  Local Variables :    not applicable                                  */
/*                                                                           */
/* ---  Bugs :                                                               */
/*                                                                           */
/* ---  Author : Giulio Sandini UG_DIST                                      */
/*                                                                           */
/* ---  Revisions                                                            */
/*            date: 20-10-86                                                 */
/*            revisor: Massimo Tistarelli                                    */
/*            reason: Included constant definition for list directory of     */
/*						    VFS files 	     	                                      */
/*                                                                           */
/*            date: 6-11-86                                                  */
/*            revisor: Sean O'Neill                                          */
/*            reason: Included constant definition for Vicom image proc.     */
/*                                                                           */
/*            date: 13-12-86                                                 */
/*            revisor: David Vernon                                          */
/*            reason: new operating system: xenix                            */
/*                                                                           */
/*****************************************************************************/
 
#ifdef DOS

#define READ_MODE  "rb"
#define WRITE_MODE "wb"

#define CMD_DIRECTORY "dir *.cmd"   /* o.s. command to list directory of
						 files with .cmd extension */
#define SYS_DIRECTORY "dir *.sys"   /* o.s. command to list directory of
						 files with .sys extension */
#define VFS_DIRECTORY "dir *.vfs"   /* o.s. command to list directory of
						 files with .vfs extension */
/*                                                                           */
/*                                                                           */
/* Symbolic constant definitions for screen manipulation under DOS o.s.      */
/*                                                                           */
/*                                                                           */

#define START_ROW        1        /* row start location */
#define MAX_NUM_ROWS     24       /* number of rows in the screen */
#define START_COL        1        /* column start location */
#define MAX_NUM_COL      79       /* maximum number of columns in the screen */
#define CLR_SCN          "\33[2J" /* clear screen */
#define CLR_SCROLL_AREA  "\33[K" /* clear scroll area at bottom of screen */
#define CUR_POS1         "\33["   /* direct cursor positioning part 1 */
#define CUR_POS2         ";"      /* direct cursor positioning part 2 */
#define CUR_POS3         "H"      /* direct cursor positioning part 3 */
#define CUR_POS4         " "       /* scroll setup last character - dummy for AT */
#define GRA_SET_ON       " "      /* special graphics set on - dummy for AT */
#define GRA_SET_OFF      " "      /* special graphics set off               */
#define BOTTOM_BAR_1     "\333\334\334\334\334\334\334\334\334\334\334\334\334\334\334\334\334\334\334\334\
\334\334\334\334\334\334\334\334\334\334\334\334\334\334\334\334\334\334\334"

#define BOTTOM_BAR_2     "\334\334\334\334\334\334\334\334\334\334\334\334\334\334\334\334\334\334\334\334\
\334\334\334\334\334\334\334\334\334\334\334\334\334\334\334\334\334\334\334\333"

#define TOP_BAR_1        "\333\337\337\337\337\337\337\337\337\337\337\337\337\337\337\337\337\337\337\337\
\337\337\337\337\337\337\337\337\337\337\337\337\337\337\337\337\337\337\337"

#define TOP_BAR_2        "\337\337\337\337\337\337\337\337\337\337\337\337\337\337\337\337\337\337\337\337\
\337\337\337\337\337\337\337\337\337\337\337\337\337\337\337\337\337\337\337\333"

#define VERTICAL_BAR     "\333"
#define BLANK_1          "                                       "
#define BLANK_2          "                                      "
#define CLEAR_MENU       "\33[6;1H\33[K\33[B\33[K\33[B\33[K\33[B\33[K\33[B\33[K\33[B\33[K\
\33[B\33[K\33[B\33[K\33[B\33[K\33[B\33[K\33[B\33[K\33[B\33[K\33[B\33[K\33[B\33[K\
\33[B\33[K\33[B\33[K\33[B\33[K\33[B\33[K\33[B\33[K\33[B\33[K\33[B\33[K"
#define SCROLL_TOP       23
#define SCROLL_BOTTOM    23
#define TRUE             1
#define FALSE            0
#define MAXBUF           128

#endif

#ifndef TDS
#define putstr printf
#endif
 
 
/* Macro definitions for region filling functions */

#define save_x(x)          X_SAVE = x
#define rest_x(x)          x = X_SAVE
#define save_xy(x,y)       X_SAVE = x; Y_SAVE = y
#define rest_xy(x,y)       x = X_SAVE; y = Y_SAVE

/*	end macros	*/

/* global definitions */

#define JUST_PASSED 1
#define PASSED 2
#define NLOC_STACK 1024
#define CMASK 0377

/* structure region_data is a collection of statitistics related    */
/*              ro the region ........                              */

struct region_data {
	       double area;
          double average;
	       double sq_average;
	       int x_bar;
	       int y_bar;
};




int  A_to_A_scale_view (int view_number_1, int x_1, int y_1, struct image_descriptor_type *image_descriptor_1, int view_number_2, int x_2, int y_2, struct image_descriptor_type *image_descriptor_2);
int  acq (int pyramid_number, short int image_level);
void add_image_to_pyramid (short int image_type, char i_description[], short int size, short int image_level, struct window_type *window, long int mask, short int bits, short int framestore_type, short int access_type, struct VFS_type *vfs_data, int pyramid_number, char p_description[], struct pyramid_descriptor_type *(*pyramid_list)); 
void append_to_cc(struct cc_node *(*cc), struct cc_node *(*end),int dirn);
void blank_heading();
void blank_line();
void blank_menu();
void blank_screen();
void build_contour_descriptors(struct image_descriptor_type *contour_image_descriptor, struct image_descriptor_type *slope_image_descriptor, struct image_descriptor_type *orientation_image_descriptor, struct image_descriptor_type *disparity_image_descriptor, struct image_descriptor_type *velocity_image_descriptor, struct image_descriptor_type *depth_image_descriptor);
void build_depth_descriptors(struct image_descriptor_type *depth_image_descriptor);
int  camera_motion(int velocity_view, int xv, int yv,struct image_descriptor_type *velocity_image_descriptor, struct camera_parameters *first_frame, struct camera_parameters *second_frame);
void chain_calc(int code, int *x, int *y);
int  chain_code  (int from_x, int from_y, int to_x, int to_y);
void clean_convolution (int view_number, int x_1, int y_1, struct image_descriptor_type *image_descriptor);
void clear_screen();
void clockwise_8_neighbour (int about_x, int about_y, int from_x, int from_y, int *to_x, int *to_y);
void compute_stereo_disparity(struct stereo_descriptor_type *stereo_values);
void contour_matching(int view1, int x1, int y1, struct image_descriptor_type *id_1, int view2, int x2, int y2, struct image_descriptor_type *id_2, double sigma);
void conv1(char input_buffer[], short nptlin, int norm_factor, char output_buffer[], int *maskad, int nptmask, int nptmas2);
void conv2(char input_buffer[], short nptlin, int norm_factor, char output_buffer[], int *maskad, int nptmask, int nptmas2);
int  convert_orientation(struct image_descriptor_type *orientation_image_descriptor, int old_range, int new_range);
void cursor_position(int x, int y);
void delete_array_image (short size, array_image_type image);
void delete_cc(struct cc_node *(*cc));
void delete_contour_image (contour_image_type *contour_image);
void delete_framestore_descriptor (struct framestore_descriptor_type *framestore_descr);
void delete_framestore_image(framestore_image_type *framestore_link);
void delete_image (struct pyramid_descriptor_type *(*pyramid_list), int pyramid_number, short int image_level);
void delete_image_descriptor (struct image_descriptor_type *image_descr);
void delete_int_array_image (short int size, int_array_image_type image);
void delete_raw_primal_image(struct raw_primal_image_type *the_image);
void delete_region_image (short int size, region_image_type *region_image);
void delete_region_tree  (struct region_tree_type *(*region_tree));
void delete_velocity_image (velocity_image_type *velocity_image);
int  depth_computation(int contour_view, int xc, int yc, struct image_descriptor_type *contour_image_descriptor,int depth_view, int xd, int yd, struct image_descriptor_type *depth_image_descriptor, struct pyramid_descriptor_type *pyramid_list,struct camera_parameters *first_frame, struct camera_parameters *second_frame,  int num_images, double scale);         
int  depth_interp(int view1, int x1, int y1, struct image_descriptor_type *id_1, int view2, int x2, int y2, struct image_descriptor_type *id_2);
int  depth_plot(int contour_view_number, int xc, int yc, struct image_descriptor_type *contour_image_descriptor, int zc_view_number, int xz, int yz, struct image_descriptor_type *zero_crossing_image_descriptor, int  grey_scale_value, int plane, float scale);
void determine_threshold(struct image_descriptor_type *greyscale_id, char *threshold);
void display_contour_summary (struct pyramid_descriptor_type *pyramid_list, int file_option);
void display_system_status (struct pyramid_descriptor_type *pyramid_list);
int  draw_polar_histogram  (int values[], int n, struct image_descriptor_type *fs_image_descriptor);
void draw_vector(array_image_type image, int x1, int y1, int x2, int y2, int gray_level);
int expression();
void extract_raw_primal_primitives(struct image_descriptor_type *contour_image,struct raw_primal_image_type *rrp_image, int orien_leeway,int required_length,int slope_diff, int bar_percent,int dist_factor,int max_blob_dim,int include_depth, int include_blobs_and_bars);
int  factor();
void fillreg(int seed_x, int seed_y, int valin, int valout, int _left, int _right, int _top, int _bottom, array_image_type input_image);
struct image_descriptor_type *find_image (struct pyramid_descriptor_type *pyramid_list, int pyramid_number, int image_level);
struct pyramid_descriptor_type *find_pyramid (struct pyramid_descriptor_type *pyramid_list, int pyramid_number);
void follow(struct cc_node *(*cc),struct cc_node *(*end), int *npix, int *nx, int *ny, array_image_type z_c);
void framegrab();
int  gaussian_smoothing(int source_view, int xc, int yc, struct image_descriptor_type *source_image_descriptor, double sigma);
struct contour_node_type *generate_contour_node (int x, int y, int length);
struct framestore_descriptor_type *generate_framestore_descriptor (short int framestore_id, short int access_type,long int mask, short int number_of_bits, short int size, struct VFS_type *vfs_data);
int  generate_int_array_image (short int size, int_array_image_type *image);
struct image_descriptor_type *generate_image_descriptor(short int image_type, char description[], short int size, short int level_number, struct window_type *window, long int mask, short int number_of_bits, struct framestore_descriptor_type *framestore, struct pyramid_descriptor_type *pyramid_link, union image_pointer_type *image_link);
int  generate_array_image (short int size, array_image_type *image);
struct raw_primal_image_type *generate_raw_primal_image(int size);
region_image_type generate_region_image (short int size);
void get_next_char();
void get_response(char *s, int limit);
void getstring(char *s, int limit);
int  global_threshold(struct image_descriptor_type *source,  int source_view, int xs, int ys, struct image_descriptor_type *dest, int dest_view, int xd, int yd, int threshold_value);
int  histogram_of_orientation  (struct image_descriptor_type *orientation_image_descriptor, struct image_descriptor_type *fs_image_descriptor);
void if_statement(struct pyramid_descriptor_type *(*pyramid_list),struct raw_primal_image_type *(*raw_primal_sketch));
void init(struct image_descriptor_type *fs);
void insert_contour (contour_image_type *contour_image, struct contour_node_type *contour_node);
int  interp_contour(int source_view, int xc, int yc, struct image_descriptor_type *source_image_descriptor);
void interpret(struct pyramid_descriptor_type *(*pyramid_list),struct raw_primal_image_type *(*raw_primal_sketch));
void line(int left_x, int right_x, int y_start, int val, array_image_type image);
int  laplacian_of_gaussian(int view_number_1, int x_1, int y_1, struct image_descriptor_type *image_descriptor_1, int view_number_2, int x_2, int y_2, struct image_descriptor_type *image_descriptor_2, double sigma);
void line_fire(int x0, int y0, int x_limit, int y_limit, int x, int y, int_array_image_type depth_image, array_image_type image);
int  linear_motion(int velocity_view,int xv,int yv,struct image_descriptor_type *velocity_image_descriptor);
int  link_trajectory(int velocity_view,int xv,int yv,struct image_descriptor_type *velocity_image_descriptor, int image_view,int x1,int y1,struct image_descriptor_type *image_descriptor, struct pyramid_descriptor_type *pyramid_list, int num_images, int gray_value, int density, int clear);
void or_select_contours (struct pyramid_descriptor_type *pyramid_list,short int low[],short int high[], struct image_descriptor_type *i_d);
void orthogonal_component(struct image_descriptor_type *contour_image_descriptor,struct image_descriptor_type *velocity_image_descriptor,int view_1, int x_1, int y_1, struct image_descriptor_type *frame_1,int view_2, int x_2, int y_2, struct image_descriptor_type *frame_2,int view_3, int x_3, int y_3, struct image_descriptor_type *frame_3,int view_4, int x_4, int y_4, struct image_descriptor_type *frame_4,int view_5, int x_5, int y_5, struct image_descriptor_type *out_image,double  sigma);
void p_plot(array_image_type image, int xs, int ys, int xe, int ye, array_image_type map, int xst, int yst, int size_x, int size_y);
int  parallel_projection();
void partial_contour_selection (struct pyramid_descriptor_type *pyramid_list);
void pop(int *curr_x, int *curr_y);
void push(int curr_x, int curr_y);
void proc_end();
void raw_primal_information(struct raw_primal_image_type *rp_image);
void readcol(int irow,int icol, short length,char buffer[], array_image_type image);
void readlin(int irow, int icol, short length, char buffer[], array_image_type image);
char read_pixel(struct image_descriptor_type *image_descriptor,  int i, int j);
void realtime_display();
int  relational_exp();
void request_image_identification(char *p_message, char *i_message, struct image_descriptor_type *(*i_d), int *quit);
void run_cc(struct cc_node *cc,struct cc_node *end);
void repeat_statement(struct pyramid_descriptor_type *(*pyramid_list),struct raw_primal_image_type *(*raw_primal_sketch));
void request_source_destination_pair(char *s_p_message, char *s_i_message, struct image_descriptor_type *(*s_i_d), char *d_p_message, char *d_i_message, struct image_descriptor_type *(*d_i_d), int *quit);
void scan();
int  sample_framestore(struct image_descriptor_type *image_descriptor, int origin_row, int origin_col);
int  search_access_types(char  *a);
int  search_p_type_table(char a[]);
int  search_k_table(char  *a);
int  search_framestore_types(char  *a);
int  search_image_types(char  *a);
void select_coincident_contours(struct image_descriptor_type *small_image,struct image_descriptor_type *large_image,int segment_size, int percent, int leeway, int orien_leeway);
void select_contours (struct pyramid_descriptor_type *pyramid_list, short int value, struct image_descriptor_type *i_d);
int  select_significant_contours(struct image_descriptor_type *contour_id);
int  select_vfs_parameters(struct VFS_type *vfs_data);
void set_view(int view_number,int size_x, int size_y);
int  slope_interp(int view1, int x1, int y1, struct image_descriptor_type *id_1, int view2, int x2, int y2, struct image_descriptor_type *id_2);
int  smooth_contour(int source_view,  int xc, int yc, struct image_descriptor_type *source_image_descriptor, int neighboors);
int  smooth_velocity(int source_view, int xs, int ys, struct image_descriptor_type *source_image_descriptor, int neighboors);
void statement(struct pyramid_descriptor_type *(*pyramid_list),struct raw_primal_image_type *(*raw_primal_sketch));
int  syn (int pyramid_number, short int image_level);
void system_error(char *error_description);
int  term();
void transfer_rp_image_to_intensity(struct raw_primal_image_type *rp_image, struct image_descriptor_type *int_image);
void transfer_view(int view_1, int x1, int y1, struct image_descriptor_type *image_descriptor_1, int view_2, int x2, int y2, struct image_descriptor_type *image_descriptor_2, int grey_level, int plane, float scale, double sigma, int contour_flag, struct image_descriptor_type *contour_image_descriptor, struct image_descriptor_type *slope_image_descriptor,struct image_descriptor_type *orientation_image_descriptor,struct image_descriptor_type *convolution_image_descriptor, int iconic_flag, struct image_descriptor_type *zero_crossing_image_descriptor);
void user_determine_threshold(struct pyramid_descriptor_type *pyramid_list);
void user_gauss_smooth(struct pyramid_descriptor_type *pyramid_list);
void user_global_threshold();
int  user_histogram_of_orientation  (struct pyramid_descriptor_type *pyramid_list);
void user_interp_contour(struct pyramid_descriptor_type *pyramid_list);
void user_or_select_contours (struct pyramid_descriptor_type *pyramid_list);
void user_select_contours (struct pyramid_descriptor_type *pyramid_list, short int value);
void user_select_significant_contours(struct pyramid_descriptor_type *pyramid_list);
void user_smooth_contour(struct pyramid_descriptor_type *pyramid_list);
void user_pause();
int  variable();
int  valid_transfer_combination(struct image_descriptor_type *image_descriptor_1,struct image_descriptor_type *image_descriptor_2);
void variable_definition();
int  velocity_gauss_smooth(int source_view, int xc, int yc,  struct image_descriptor_type *source_image_descriptor, double sigma);
int  velocity_plot(int contour_view_number, int xc, int yc,struct image_descriptor_type *contour_image_descriptor,int view_number, int x1, int y1, struct image_descriptor_type *image_descriptor, int gray_value, int density, double scale, int clear,double threshold);
void visicl_add_image_to_pyramid(struct pyramid_descriptor_type *(*pyramid_list));
void visicl_camera_motion(struct pyramid_descriptor_type *pyramid_list);
void visicl_choose_RPS_primitives(struct pyramid_descriptor_type *pyramid_list, struct raw_primal_image_type *rp_image);
void visicl_coincident_contours(struct pyramid_descriptor_type *pyramid_list);
void visicl_contour_smoothing(struct pyramid_descriptor_type *pyramid_list);
void visicl_delete_image_from_pyramid(struct pyramid_descriptor_type *(*pyramid_list));
void visicl_depth_comp(struct pyramid_descriptor_type *pyramid_list);
void visicl_display_system_status(struct pyramid_descriptor_type *pyramid_list);
void visicl_draw_RPS_primitives(struct pyramid_descriptor_type *pyramid_list, struct raw_primal_image_type *raw_primal_sketch);
void visicl_generate_raw_primal_sketch(struct pyramid_descriptor_type *pyramid_list,struct raw_primal_image_type *(*raw_primal_sketch));
void visicl_select_all_contours(struct pyramid_descriptor_type (*pyramid_list),int flag);
void visicl_select_RPS_primitives(struct pyramid_descriptor_type *pyramid_list,struct raw_primal_image_type *rp_image);
void visicl_select_significant_contours(struct pyramid_descriptor_type (*pyramid_list));
void visicl_specify_window(struct pyramid_descriptor_type *pyramid_list);
void visicl_stereo_disparity(struct pyramid_descriptor_type *pyramid_list);
void visicl_transfer_image(struct pyramid_descriptor_type *pyramid_list);
void visicl_terminate();
void visicl_par_transfer_image(struct pyramid_descriptor_type *pyramid_list);
void visicl_velocity_plot(struct pyramid_descriptor_type *pyramid_list);
void visicl_x_y_table_home();
void visicl_x_y_table_move();
void visicl_list_RPS_information(struct pyramid_descriptor_type *pyramid_list,struct raw_primal_image_type *raw_primal_sketch);
void visicl_link_trajectory(struct pyramid_descriptor_type *pyramid_list);
void visicl_linear_motion(struct pyramid_descriptor_type *pyramid_list);
void visicl_echo_program();
void visicl_edge_information(struct raw_primal_image_type *rp_image);
void visicl_framegrab(struct pyramid_descriptor_type (*pyramid_list));
void visicl_or_select_contours(struct pyramid_descriptor_type *pyramid_list);
void visicl_orthogonal_comp(struct pyramid_descriptor_type *pyramid_list);
void while_statement(struct pyramid_descriptor_type *(*pyramid_list),struct raw_primal_image_type *(*raw_primal_sketch));
void write_statement();
void writecol(int irow, int icol, short length, char buffer[], array_image_type image);
void writelin(int irow, int icol, short length, char buffer[], array_image_type image);
int  write_pixel(struct image_descriptor_type *image_descriptor, int i, int j, char pixel_value);
int  zc_features(struct image_descriptor_type *contour_image_descriptor, struct image_descriptor_type *slope_image_descriptor,struct image_descriptor_type *, array_image_type slope, array_image_type orientation);
int  zc_fol(int view_number, int xv, int yv, struct image_descriptor_type *image_descriptor_1, struct image_descriptor_type *image_descriptor_2);
int  zc_plot(int contour_view_number, int xc, int yc,struct image_descriptor_type *contour_image_descriptor,int zc_view_number, int xz, int yz, struct image_descriptor_type *zero_crossing_image_descriptor,int grey_scale_value);       
int  zero_crossings(int view_number_1, int x_1, int y_1, struct image_descriptor_type *image_descriptor_1,int view_number_2, int x_2, int y_2, struct image_descriptor_type *image_descriptor_2,double sigma,int contour_flag,struct image_descriptor_type *contour_image_descriptor,  struct image_descriptor_type *slope_image_descriptor,struct image_descriptor_type *orientation_image_descriptor,int iconic_flag, struct image_descriptor_type *zero_crossing_image_descriptor);




// interface routines

void laplacianOfGaussianWrapper(DVimage *intensity, float sigma, bool significantZC, bool noZC, bool noPlot, DVimage *log, DVimage *zc, DVimage *plot);

#endif // __ICUB_OPTICALFLOW_MODULE_H__
//empty line to make gcc happy

