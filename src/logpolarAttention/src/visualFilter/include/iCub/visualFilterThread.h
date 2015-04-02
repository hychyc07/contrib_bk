// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
 * email:   francesco.rea@iit.it
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

/**
 * @file visualFilterThread.h
 * @brief Definition of a thread that receives images and does the computation for the
 * visual filter module (see visualFilterModule.h).
 */

#ifndef _VISUAL_FILTER_THREAD_H_
#define _VISUAL_FILTER_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <iostream>

// outside project includes
#include <ippi.h>
#include <ipp.h>
#include <ipps.h>
#include <ippcv.h>
#include <ippcore.h>

#define CHAR_LIMIT 256

class visualFilterThread : public yarp::os::Thread
{
private:
    IppiSize srcsize;                   // ROI for the images in the processing
    IppiSize originalSrcsize;           // ROI of he input image

    int psb;
    int width_orig, height_orig;        // dimension of the input image (original)
    int width, height;                  // dimension of the extended input image (extending)
    int size1;                          // size of the buffer
    int psb16s;                         // step size of the Ipp16s vectors
    float lambda;                       // costant for the temporal filter
   
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *inputImage;            // input image
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *inputImageFiltered;    // time filtered input image
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *inputExtImage;         // extended input image
        
    yarp::sig::ImageOf<yarp::sig::PixelMono> *redPlane;             // image of the red channel
    yarp::sig::ImageOf<yarp::sig::PixelMono> *redPlane2;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *redPlane3;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *greenPlane;           // image of the green channel
    yarp::sig::ImageOf<yarp::sig::PixelMono> *greenPlane2;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *greenPlane3;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *bluePlane;            // image of the blue channel
    yarp::sig::ImageOf<yarp::sig::PixelMono> *bluePlane2;           // image of the blue channel
    yarp::sig::ImageOf<yarp::sig::PixelMono> *bluePlane3;           // image of the blue channel
    yarp::sig::ImageOf<yarp::sig::PixelMono> *yellowPlane;          // image of the yellow channel
    yarp::sig::ImageOf<yarp::sig::PixelMono> *yellowPlane2;

    yarp::sig::ImageOf<yarp::sig::PixelMono> *redPlus;              // positive gaussian-convolved red image 
    yarp::sig::ImageOf<yarp::sig::PixelMono> *redMinus;             // negative gaussian-convolved red image

    yarp::sig::ImageOf<yarp::sig::PixelMono> *greenPlus;            // positive gaussian-convolved green image 
    yarp::sig::ImageOf<yarp::sig::PixelMono> *greenMinus;           // negative gaussian-convolved green image

    yarp::sig::ImageOf<yarp::sig::PixelMono> *bluePlus;             // positive gaussian-convolved red image 
    yarp::sig::ImageOf<yarp::sig::PixelMono> *yellowMinus;          // negative gaussian-convolved red image

    yarp::sig::ImageOf<yarp::sig::PixelMono> *redGreen;             // colour opponency map (R+G-)
    yarp::sig::ImageOf<yarp::sig::PixelMono> *greenRed;             // colour opponency map (G+R-)
    yarp::sig::ImageOf<yarp::sig::PixelMono> *blueYellow;           // colour opponency map (B+Y-)
    yarp::sig::ImageOf<yarp::sig::PixelMono> *edges;                // edges of colour opponency maps 

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortIn;       // input port
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > imagePortOut;     // output port   
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortExt;      // extended image port
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > rgPort;           // Colour opponency map R+G-
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > grPort;           // Colour opponency map G+R-
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > byPort;           // Colour opponency map B+Y-

    Ipp16s* redGreenH16s, *redGreenV16s;                            // memory allocated for redGreen edges
    Ipp16s* greenRedH16s, *greenRedV16s;                            // memory allocated for greenRed edges
    Ipp16s* blueYellowH16s, *blueYellowV16s;                        // memory allocated for blueYellow edges
    Ipp8u* buffer;                                                  // buffer necessary for the sobel operation

    Ipp8u edges_LUT[CHAR_LIMIT*CHAR_LIMIT];                        // a look up table to calculate edges (to optimize)
    Ipp8u filter_LUT[CHAR_LIMIT*CHAR_LIMIT];                       // a look up table to calculate filtering (to optimize)

    std::string name;       // rootname of all the ports opened by this thread
    bool resized;           // flag to check if the variables have been already resized

public:
    /**
    * constructor
    */
    visualFilterThread();

    /**
     * destructor
     */
    ~visualFilterThread();

    bool threadInit();     
    void threadRelease();
    void run(); 
    void onStop();

    /**
    * function that set the rootname for the ports that will be opened
    * @param str rootname as a string
    */
    void setName(std::string str);
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /**
    * function that resizes the necessary and already allocated images
    * @param width width of the input image
    * @param height height of the input image
    */
    void resize(int width, int height);

    /**
    * function that extendes the original image of the desired value for future convolutions (in-place operation)
    * @param origImage originalImage
    * @param extDimension dimension of the extention on each of the sides of the image
    */
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* extender(yarp::sig::ImageOf<yarp::sig::PixelRgb>* origImage,int extDimension); 

    /**
    * extracting RGB and Y planes
    */
    void extractPlanes();

    /**
    * function that filters the input image in time 
    */
    void filterInputImage();

    /**
    * gaussing filtering of the of RGBY
    */
    void filtering();

    /**
    * function which constructs colourOpponency maps
    */
    void colourOpponency();

    /**
    * applying sobel operators on the colourOpponency maps and combining via maximisation of the 3 edges
    */
    void edgesExtract();
};

#endif  //_VISUAL_FILTER_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

