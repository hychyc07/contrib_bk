/**
 *
 * @ingroup icub_module
 *
 * \defgroup icub_sequentialLabeller sequentialLabeller
 *
 * Receive a raw image and a binary image from an object segmentation
 * application, compute a labelled image.
 *
 * \section intro_sec Description
 *
 * Use this module in combination with your favourite segmentation algorithm
 * (usually lumaChroma+blobExtractor or EDISON), obtain a labelled image
 * suitable for computing object affordances (blobDescriptor module).
 *
 * Binary image: an image containing 0=background, 1=blobs
 * Labelled image: an image containing 0=background, 1=object, 2=another object,
 * 3=another object, etc.
 *
 * \section lib_sec Libraries
 *
 * YARP, iCub (?), OpenCV.
 *
 * <b>Command-line Parameters</b>
 *
 * - --from name_of_configuration_file.ini (default sequentialLabeller.ini)
 * - --context name_of_context_dir (default sequentialLabeller/conf)
 *
 * <b>Configuration File Parameters</b>
 *
 * - name: prefix of ports created by this module (default /sequentialLabeler)
 * - raw_image_input_port (default <name>/rawImg:i)
 * - raw_image_output_port (default <name>/rawImg:o)
 * - binary_image_input_port (default <name>binImg:i)
 * - labeled_image_output_port (default <name>/labeledImg:o)
 *
 * \section portsa_sec Ports Accessed
 *
 * - <tt>propImg:o</tt> or <tt>rawImg:o</tt> \n
 *   Raw image port, propagated by a segmentation application (usually
 *   /blobExtractor/propImg:o in the case of lumaChroma,
 *   /edisonSegmentation/rawImg:o in the case of EDISON)
 *
 * - <tt>binary:o</tt> \n
 *   Binary image port, previously created by a segmentation application
 *   (usually /blobExtractor/binary:o in the case of lumaChroma)
 *
 * \section portsc_sec Ports Created
 *
 * <b>Input ports</b>
 *
 * - <tt>/sequentialLabeller/rawImg:i</tt> \n
 *   Raw image input port (propagated image)
 *
 * - <tt>/sequentialLabeller/binImg:i</tt> \n
 *   Binary image input port
 *
 * <b>Output ports</b>
 *
 * - <tt>/sequentialLabeller/rawImg:o</tt> \n
 *   Raw image output port (propagated image)
 *
 * - <tt>/sequentialLabeller/labeledImg:o</tt> \n
 *   Labelled image output port
 *
 * <b>Input/Output ports</b>
 *
 * None
 *
 * <b>Port Types</b>
 *
 * - <tt>BufferedPort<ImageOf<PixelRgb> >  rawImgInputPort</tt>
 * - <tt>BufferedPort<ImageOf<PixelMono> > binaryImgInputPort</tt>  
 * - <tt>BufferedPort<ImageOf<PixelRgb> >  rawImgOutputPort</tt>
 * - <tt>BufferedPort<ImageOf<PixelMono> >  labeledImgOutputPort</tt>
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
 * sequentialLabeller.ini (optional)
 *
 * \section tested_os_sec Tested OS
 *
 * Linux
 *
 * \section example_sec Example Instantiation of the Module
 * - <tt>sequentialLabeller</tt>\n
 *
 * \author Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 *
 * Copyright (C) 2013 POETICON++, European Commission FP7 project ICT-288382
 * CopyPolicy: Released under the terms of the GNU GPL v2.0 
 *
 */

#ifndef __SEQUENTIAL_LABELLER_MODULE_H__
#define __SEQUENTIAL_LABELLER_MODULE_H__

/* YARP */
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Stamp.h>
using namespace yarp::os;
#include <yarp/sig/Image.h>
using namespace yarp::sig;

/* OpenCV */
// TODO: use OpenCV 2 APIs
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include "cvSeqLabel.h"

/* system */
#include <string>
using namespace std;

class SequentialLabellerModule : public RFModule
{
	/* private class variables and module parameters */
	string                            _moduleName;
	string                            _handlerPortName;
    Port                              _handlerPort; /* a port to handle messages */

	string                            rawImgInputPortName;
    string                            rawImgOutputPortName;
  	string                            binaryImgInputPortName;
  	string                            labeledImgOutputPortName;
	/* yarp image pointers to access image ports */
	ImageOf<PixelRgb>                *yarpRawInputPtr;
	ImageOf<PixelMono>               *yarpBinaryInputPtr;
	/* yarp internal image buffers */ 
    ImageOf<PixelRgb>                 yarpRawImg;
    ImageOf<PixelMono>                yarpBinaryImg;
    ImageOf<PixelMono>                 yarpLabeledImg;

	int                               w, h;
	CvSize                            sz;
    
	BufferedPort<ImageOf<PixelRgb> >  rawImgInputPort;
  	BufferedPort<ImageOf<PixelMono> > binaryImgInputPort;    
	BufferedPort<ImageOf<PixelRgb> >  rawImgOutputPort;
  	BufferedPort<ImageOf<PixelMono> >  labeledImgOutputPort;

public:
	virtual bool configure(ResourceFinder &rf); /* configure module parameters, return true if successful */
	virtual bool interruptModule();             /* interrupt, e.g., ports */
	virtual bool close();                       /* close and shut down module */
	virtual bool respond(const Bottle &command, Bottle &reply);
	virtual bool updateModule();
	virtual double getPeriod();
};

#endif // __SEQUENTIAL_LABELLER_MODULE_H__
