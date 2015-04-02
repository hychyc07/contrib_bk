/**
 * @ingroup icub_module
 *
 * \defgroup icub_blobDescriptor blobDescriptor
 *
 * Receive a raw image and a labelled image from an Object Segmentation
 * module, compute a list of affordance descriptors.
 *
 * \section lib_sec Libraries
 *
 * OpenCV, YARP.
 *
 * \section parameters_sec Parameters
 *
 * <b>Command-line Parameters</b>
 *
 * The following key-value pairs can be specified as command-line parameters by prefixing -- to the key
 * (e.g., <tt>--from file.ini</tt>). The value part can be changed to suit your needs; default values are shown below.
 *
 * - <tt>from blobDescriptor.ini</tt> \n
 *   Configuration file
 *
 * - <tt>context blobDescriptor/conf</tt> \n
 *   Sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - <tt>name blobDescriptor</tt> \n
 *   Root name of the module (used to form the stem of module port names)
 *
 * <b>Configuration File Parameters</b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file
 * (they can also be specified as command-line parameters if you so wish).
 * The value part can be changed to suit your needs; the default values are shown below.
 *
 * - <tt>conf_port /blobDescriptor/conf</tt> \n
 *   Complete configuration and message handling port name
 *
 * - <tt>raw_image_input_port /blobDescriptor/rawImg:i</tt> \n
 *   Complete raw image input port name
 *
 * - <tt>labeled_image_input_port /blobDescriptor/labeledImg:i</tt> \n
 *   Complete labeled image input port name
 *
 * - <tt>user_selection_port /blobDescriptor/userSelection:i</tt> \n
 *   Complete user selection intput port name - for debug
 *
 * - <tt>raw_image_output_port /blobDescriptor/rawImg:o</tt> \n
 *   Complete raw image output port name
 *
 * - <tt>view_image_output_port /blobDescriptor/viewImg:o</tt> \n
 *   Complete port name of output image, including overlay edges
 *
 * - <tt>aff_descriptor_output_port /blobDescriptor/affDescriptor:o</tt> \n
 *   Complete affordance object descriptor output port name
 *
 * - <tt>tracker_init_output_port /blobDescriptor/trackerInit:o</tt> \n
 *   Complete tracker initialization parameter output port name
 *
 * - <tt>tracker_init_single_obj_output_port /blobDescriptor/trackerInitSingleObj:o</tt> \n
 *   Single object (from user click - debug) tracker initialization parameter output port name
 *
 * - <tt>min_area_threshold 100</tt> \n
 *   Minimum number of pixels allowed for foreground objects
 *
 * - <tt>max_area_threshold 20000</tt> \n
 *   Maximum number of pixels allowed for foreground objects
 *
 * - <tt>max_objects 20</tt> \n
 *   Maximum number of objects to process
 *
 * - <tt>invalidate_boundary_objects 0</tt> \n
 *   Flag to make invalid objects that are touch the image boundaries 
 *
 * - <tt>draw_holes 0</tt> \n
 *   Flag to draw the holes of the valid objects in the display image 
 *
 * - <tt>normalized_coords 1</tt> \n
 *   Normalize coordinates of enclosing rectangle to unit? (RobotCub: yes==1, First-MM: no==0)
 *
 * \section portsa_sec Ports Accessed
 *
 * - <tt>/edisonSegm/rawImg:o</tt> \n
 *   Raw image port, previously created by a segmentation module
 *
 * - <tt>/edisonSegm/labelImg:o</tt> \n
 *   Labeled image port, previously created by a segmentation module
 *
 * \section portsc_sec Ports Created
 *
 *  <b>Input ports</b>
 *
 * - <tt>/blobDescriptor/rawImg:i</tt> \n
 *   Raw image input port
 *
 * - <tt>/blobDescriptor/labeledImg:i</tt> \n
 *   Labeled image input port
 *
 * - <tt>/blobDescriptor/userSelection:i</tt> \n
 *   User selection input port - for debug 
 *
 * <b>Output ports</b>
 *
 * - <tt>/blobDescriptor/rawImg:o</tt> \n
 *   Raw image output port
 *
 * - <tt>/blobDescriptor/viewImg:o</tt> \n
 *   Port to display output image, including overlay edges
 *   Valid objects are outlined with a RED contour.
 *   Invalid object are outlined with a GRAY contour.
 *   User selected objects are outlines with a BLUE contour.
 *   HOles of valid objects can be outlined in YELLOW (when enabled in configuration file).
 * - <tt>/blobDescriptor/affDescriptor:o</tt> \n
 *   Affordance object descriptor output port \n
 *   The message is a Bottle containing several values.
 *   The first value is an integer indicating the number of objects (N).
 *   The consecutive N values (one per object) are lists (Bottles) containing 
 *   the objects descriptors, with the following order:
 *		- 0 (double) - normalized x coordinate of the center of the enclosing rectangle (between -1 and 1 w.r.t. the image size).
 *		- 1 (double) - normalized y coordinate of the center of the enclosing rectangle (between -1 and 1 w.r.t. the image size). 
 *		- 2 (double) - normalized width of the enclosing rectangle (between 0 and 1 w.r.t. the the image size)
 *		- 3 (double) - normalized height of the enclosing rectangle (between 0 and 1 w.r.t. the the image size)
 *		- 4 (double) - angle (orientation) of the object's enclosing rectangle, according to OpenCV definition (see CvBox2D).
 *		- 5 (double) - x coordinate of the center of the bounding rectangle.
 *		- 6 (double) - y coordinate of the center of the bounding rectangle.
 *		- 7 (double) - value of the 1st bin of the Hue normalized color histogram of the pixels inside the object's region.
 *		- ...
 *		- 22 (double) - value of the 16th (last) bin of the Hue normalized color histogram of the pixels inside the object's region.
 *		- 23 (double) - area
 *		- 24 (double) - convexity - ratio between the perimeter of the object's convex hull and the perimeter of the object's contour.
 *		- 25 (double) - eccentricity - ratio between the minor and major axis of the minimum area enclosing rectangle
 *		- 26 (double) - compactness - ratio between the object area and its squared perimeter.
 *		- 27 (double) - circleness - ratio between the object area and the area of its enclosing circle.
 *		- 28 (double) - squareness - ratio between the object area and the area of its minimum-area enclosing rectangle
 *
 * - <tt>/blobDescriptor/trackerInit:o</tt> \n
 *   Tracker initialization parameter output port \n
 *   The message is a Bottle containing several values.
 *   The first value is an integer indicating the number of objects (N).
 *   The consecutive N values (one per object) are lists (Bottles) containing 
 *   the values required for the OpenCV CAMSHIFT tracker, with the following order:
 *		- 0 (double) - x coordinate of the tracking box
 *		- 1 (double) - y coordinate of the tracking box. 
 *		- 2 (double) - width of the tracking box.
 *		- 3 (double) - height of the tracking box
 *		- 4 (double) - value of the 1st bin of the Hue normalized color histogram of the pixels inside the object's region.
 *		- ...
 *		- 19 (double) - value of the 16th (last) bin of the Hue normalized color histogram of the pixels inside the object's region.
 *		- 20 (double) - value of the minimum intensity value in the object region (v_min).
 *		- 21 (double) - value of the maximum intensity value in the object region (v_max).
 *		- 22 (double) - value of the minimum saturation value in the object region (s_min).
 *
 *
 * <b>Input/Output ports</b>
 *
 * - <tt>/blobDescriptor/conf</tt> \n
 *   Complete configuration and message handling port name
 *
 * <b>Port Types</b>
 *
 * - <tt>BufferedPort<ImageOf<PixelRgb> >  _rawImgInputPort</tt>
 * - <tt>BufferedPort<ImageOf<PixelInt> >  _labeledImgInputPort</tt>
 * - <tt>BufferedPort<Bottle>              _userSelectionInputPort</tt>
 * - <tt>BufferedPort<ImageOf<PixelRgb> >  _rawImgOutputPort</tt>
 * - <tt>BufferedPort<ImageOf<PixelRgb> >  _viewImgOutputPort</tt>
 * - <tt>BufferedPort<Bottle>              _affDescriptorOutputPort</tt>
 * - <tt>BufferedPort<Bottle>              _trackerInitOutputPort</tt>
 * - <tt>BufferedPort<Bottle>              _trackerInitSingleObjOutputPort</tt>
 * - <tt>Port                              _handlerPort</tt>
 *
 * \section in_data_sec Input Data Files
 *
 * None
 *
 * \section out_data_sec Output Data Files
 *
 * None
 *
 * \section conf_file_sec Configuration Files
 *
 * - \c blobDescriptor.ini in \c $ICUB_ROOT/contrib/src/poeticon/poeticonpp/blobDescriptorModule/app/conf
 * - \c blobDescriptor.ini in \c backup/oldBuild/app/demoAffv2/conf
 *
 * \section tested_os_sec Tested OS
 *
 * Linux
 *
 * \section example_sec Example Instantiation of the Module
 *
 * - <tt>blobDescriptor --name blobDescriptor --context blobDescriptor/conf --from blobDescriptor.ini </tt>\n
 * - <tt>blobDescriptor --context demoAffv2/conf</tt> (Object Affordances demo)
 *
 * \author Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>, Ivana Cingovska, Alexandre Bernardino
 *
 * Copyright (C) 2013 POETICON++, European Commission FP7 project ICT-288382
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * CopyPolicy: Released under the terms of the GNU GPL v2.0
 *
 * This file can be edited at src/blobDescriptor/include/iCub/BlobDescriptorModule.h
 *
 */

#ifndef __ICUB_BLOB_DESC_MODULE_H__
#define __ICUB_BLOB_DESC_MODULE_H__

/* YARP */
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Time.h>
using namespace yarp::os;
#include <yarp/sig/Image.h>
using namespace yarp::sig;

/* iCub */
#include <iCub/BlobDescriptorSupport.h>

/* OpenCV */
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

/* system */
//#include <fstream>
//#include <iostream>
//#include <sstream>
#include <string>
#include <stdio.h>
//#include <vector>
using namespace std;

class BlobDescriptorModule : public RFModule
{
	/* private class variables and module parameters */
	string                            _moduleName;
	string                            _robotName;
	string                            _robotPortName;
	string                            _rawImgInputPortName;
	string                            _labeledImgInputPortName;
    string                            _userSelectionInputPortName; /* for debug */
    string                            _rawImgOutputPortName;
    string                            _viewImgOutputPortName;
	string                            _affDescriptorOutputPortName;
	string                            _trackerInitOutputPortName;
    string                            _trackerInitSingleObjOutputPortName; /* from user click - debug */
    string                            _handlerPortName;
    Port                              _handlerPort; /* a port to handle messages */
	BufferedPort<ImageOf<PixelRgb> >  _rawImgInputPort;
	BufferedPort<ImageOf<PixelInt> >  _labeledImgInputPort;
    BufferedPort<Bottle>              _userSelectionInputPort; /* for debug */
	BufferedPort<ImageOf<PixelRgb> >  _rawImgOutputPort;
	BufferedPort<ImageOf<PixelRgb> >  _viewImgOutputPort;
	BufferedPort<Bottle>              _affDescriptorOutputPort;
	BufferedPort<Bottle>              _trackerInitOutputPort;
    BufferedPort<Bottle>              _trackerInitSingleObjOutputPort; /* from user click - debug */
	/* yarp image pointers to access image ports */
	ImageOf<PixelRgb>                *_yarpRawInputPtr;
	ImageOf<PixelInt>                *_yarpLabeledInputPtr;
	/* yarp internal image buffers */
    ImageOf<PixelRgb>                 _yarpRawImg;
    ImageOf<PixelRgb>                 _yarpViewImg;
    ImageOf<PixelRgb>                 _yarpHSVImg;
    ImageOf<PixelMono>                _yarpHueImg;
	ImageOf<PixelMono>				  _yarpSatImg;
	ImageOf<PixelMono>                _yarpValImg;
    ImageOf<PixelInt>                 _yarpLabeledImg;
    ImageOf<PixelMono>                _yarpTempImg;

	Bottle                            _affDescriptor;
	Bottle                            _trackerInit;
    Bottle                            _trackerInitSingleObj; /* from user click - debug */
	/* OpenCV images - not needed anymore */
	//IplImage                       *_opencvRawImg;
	//IplImage                       *_opencvLabeledImg32;
    //IplImage                       *_opencvLabeledImg8;
	int                               _w, _h;
	CvSize                            _sz;

	ObjectDescriptor                 *_objDescTable;
	int                               _numObjects;
    //int                               _hist_size[2];
	//float                             _h_ranges[2], _s_ranges[2], _v_ranges[2];
	
	/* other parameters that can be user-specified (besides port names) */
	int _minAreaThreshold; /* min. number of pixels allowed for foreground objects */
	int _maxAreaThreshold; /* min. number of pixels allowed for foreground objects */
	int _maxObjects;       /* maximum number of object to process */
	int	_invalidate_boundary_objects; /*  Flag to invalidate objects touching the image boundaries */
	int _draw_holes;       /* Flag to draw the holes of the valid objects in the display image */
	
	// added for First-MM in Jan. 2011
	bool normalized_coords;

public:
	virtual bool configure(ResourceFinder &rf); /* configure module parameters, return true if successful */
	virtual bool interruptModule();             /* interrupt, e.g., ports */
	virtual bool close();                       /* close and shut down module */
	virtual bool respond(const Bottle &command, Bottle &reply);
	virtual bool updateModule();
	virtual double getPeriod();
};

#endif // __ICUB_BLOB_DESC_MODULE_H__
