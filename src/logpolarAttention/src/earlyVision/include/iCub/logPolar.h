// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco, Shashank Pathak
 * email:   francesco.rea@iit.it, shashank.pathak@iit.it
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
 * @file logPolar.h
 * @brief Implements logpolar to cartesian and vice versa, for monochromatic image, taken from previous implementation
 */

#ifndef _LOG_POLAR_H
#define _LOG_POLAR_H

#include <iostream>
#include <string>
#include <cstring>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>


#define PI_LOGPOLAR 3.1415926535897932384626433832795


#define MONO_PI_LOGPOLARXEL_SIZE 1
#define KERNEL_ROW 7
#define KERNEL_COL 7
// patches for now
#ifndef YARP_IMAGE_ALIGN
#define YARP_IMAGE_ALIGN 8
#endif

// Some general purpose definitions for logpolar transformation
enum {
    RADIAL = 0, /** \def RADIAL Each receptive field in fovea will be tangent to a receptive field in the previous ring and one in the next ring */
    TANGENTIAL = 1, /** \def TANGENTIAL Each receptive field in fovea will be tangent to the previous and the next receptive fields on the same ring. */
    ELLIPTICAL = 2 /** \def ELLIPTICAL Each receptive field in fovea will be tangent to all its neighbors, having then an elliptical shape. */
};

enum {
   C2L = 1,     // 2^0
   L2C = 2,     // 2^1
   BOTH = 3,    // 2^0 + 2^1
};

/**
 * \struct cart2LpPixel 
 * \brief It contains the look-up table for the creation of a log polar image. 
 *
 */
struct cart2LpPixel
{
    int divisor;    /**< Number of cartesian pixels corresponding to the current log polar one.*/
    int *iweight;   /**< Array containing the weight of each cartesian pixel.*/
    int *position;  /**< Array containing the position of each cartesian pixel. \n
                         Note that the plane information is included in this field 
                         (i.e. when only one plane is present it contains
                         the value  (\p y*xSize+x), while in case of three planes, 
                         the value will be \p 3*(\p y*xize+x) ).*/
};

/**
 * \struct lp2CartPixel 
 * \brief It contains the look-up table for the remapping of a log polar image into a cartesian one. 
 *
 */
struct lp2CartPixel
{
    int iweight;    /**< Number of pixels in the array of positions */
    int *position;  /**< Array containing the cartesian position of each log 
                         polar pixel. \n
                         Note that the plane information is included in this 
                         field (i.e. the value will be \p 3*(\p y*xize+x) ).*/
};

/**
 * replicate borders on a logpolar image before filtering (similar in spirit to IPP or OpenCV replication).
 * @param dest is the image with replicated borders of size w+2*maxkernelsize, h+maxkernelsize
 * @param src is the input image
 * @param maxkernelsize is half of the kernel size (i.e. ceil(kernel/2))
 * @return true iff the replication is possible (images must be correctly sized and allocated)
 */
bool replicateBorderLogpolar(yarp::sig::Image& dest, const yarp::sig::Image& src, int maxkernelsize);

/**
 * make an image of the fovea starting from the original cartesian image.
 * @param dest is the foveal image (the size of the fovea is given by the destination image size
 * @param src is the input image
 * @return true iff the image sizes are compatible with the operation requested
 */
bool subsampleFovea(yarp::sig::ImageOf<yarp::sig::PixelRgb>& dst, const yarp::sig::ImageOf<yarp::sig::PixelRgb>& src);

// End of general purpose definitions for logpolar transformations


// Class for log polar transformation, as in logpolar library (RC_DIST_FB_logpolar_mapper)
class logpolarTransformVisual {
private:
    cart2LpPixel *c2lTable;
    lp2CartPixel *l2cTable;
    int necc_;
    int nang_;
    int width_;
    int height_;
    int mode_;

    /*
    * \arg \b 0 All the RF's are tangent each other. 
    * \arg \b -1 All their sizes are 1 pixel
    * \arg \b 1 The edges of the RF's pass through the centers of neighboring RF's
    * Allowed values: float numbers between -1 and +infinity
    */
    double overlap_;

    // forbid copies.
    logpolarTransformVisual(const logpolarTransformVisual& x);
    void operator=(const logpolarTransformVisual& x);

    /**
    * \brief Frees the memory from the look-up table.
    */
    void RCdeAllocateC2LTable ();

    /**
    * \brief Frees the memory from the look-up table. 
    */
    void RCdeAllocateL2CTable ();

    /**
    * \brief Generates the look-up table for the transformation from a cartesian image to a log polar one, both images are color images
    * @param scaleFact the ratio between the size of the smallest logpolar pixel and the cartesian ones
    * @param mode is one of the following : RADIAL, TANGENTIAL or ELLIPTICAL
    * @param padding is the input image row byte padding (cartesian image paddind)
    * @return 0 when there are no errors
    * @return 1 in case of wrong parameters
    * @return 2 in case of allocation problems
    */
    int RCbuildC2LMap (double scaleFact, int mode, int padding);

    /**
    * \brief Generates the look-up table for the transformation from a log polar image to a cartesian one.
    * @param scaleFact the ratio between the size of the smallest logpolar pixel and the cartesian ones
    * @param hOffset is the horizontal shift in pixels
    * @param vOffset is the vertical shift in pixels
    * @param mode is one of the following : RADIAL, TANGENTIAL or ELLIPTICAL
    * @param padding is the number of pad bytes of the input image (logpolar)
    * @return 0 when there are no errors
    * @return 1 in case of wrong parameters
    * @return 2 in case of allocation problems
    */
    int RCbuildL2CMap (double scaleFact, int hOffset, int vOffset, int mode, int padding);

    /**
    * \brief Generates a log polar image from a cartesian one
    * @param lpImg is the output LogPolar image
    * @param cartImg is the input Cartesian image
    * @param Table is the LUT used for the transformation
    * @param padding is the padding of the logpolar image (output)
    is generated otherways
    */
    void RCgetLpImg (unsigned char *lpImg,
                     unsigned char *cartImg,
                     cart2LpPixel * Table, 
                     int padding);

    /**
    * \brief Remaps a log polar image to a cartesian one
    * @param cartImg is the output Cartesian image
    * @param lpImg is the input LogPolar image
    * @param Table is the LUT used for the transformation
    * @param cartSize is the size of the log polar image
    */
    void RCgetCartImg (unsigned char *cartImg, unsigned char *lpImg, lp2CartPixel * Table, int cartSize);

    /**
    * \brief Computes the logarithm index
    * @param nAng is the number of pixels per ring 
    * @return the logarithm index.
    */
    double RCgetLogIndex ();

    /**
    * \brief Computes the ratio between the size of the smallest logpolar pixel and the cartesian ones
    * @return the scale factor.
    */
    double RCcomputeScaleFactor ();

public:
    /**
     * default constructor.
     */
    logpolarTransformVisual() {
        c2lTable = 0;
        l2cTable = 0;
        necc_ = 0;
        nang_ = 0;
        width_ = 0;
        height_ = 0;
        overlap_ = 0.;
        mode_ = BOTH;
    }

    /** destructor */
    virtual ~logpolarTransformVisual() {
        freeLookupTables();
    }

    /** 
     * check whether the LUT have been previously allocated.
     * @return true iff one or both LUTs are different from zero.
     */
    virtual const bool allocated() const {
        if (c2lTable != 0 || l2cTable != 0)
            return true;
        else
            return false;
    }

    /**
     * alloc the lookup tables and stores them in memory.
     * @param necc is the number of eccentricities of the logpolar image.
     * @param nang is the number of angles of the logpolar image.
     * @param w is the width of the original rectangular image.
     * @param h is the height of the original rectangular image.
     * @param overlap is the degree of overlap of the receptive fields (>0.).
     * @return true iff successful.
     */
    virtual bool allocLookupTables(int mode = BOTH, int necc = 152, int nang = 252, int w = 640, int h = 480, double overlap = 1.);

    /**
    * free the lookup tables from memory.
    * @return true iff successful.
    */
    virtual bool freeLookupTables();

    /**
     * converts an image from rectangular to logpolar.
     * @param lp is the logpolar image (destination).
     * @param cart is the cartesian image (source data).
     * @return true iff successful. Beware that tables must be
     * allocated in advance.
     */
    virtual bool cartToLogpolar(yarp::sig::ImageOf<yarp::sig::PixelMono>& lp, 
                                const yarp::sig::ImageOf<yarp::sig::PixelMono>& cart);

    /**
     * converts an image from logpolar to cartesian (rectangular).
     * @param cart is the cartesian image (destination).
     * @param lp is the logpolar image (source).
     * @return true iff successful. Beware that tables must be
     * allocated in advance.
     */
    virtual bool logpolarToCart(yarp::sig::ImageOf<yarp::sig::PixelMono>& cart,
                                const yarp::sig::ImageOf<yarp::sig::PixelMono>& lp);

    /**
     * check the number of eccentricities (rings).
     * @return the number of rings in the logpolar mapping (default 152).
     */
    int necc(void) const { return necc_; }

    /**
     * check the number of angles (radii).
     * @return the number of radii in the logpolar mapping (default 252).
     */
    int nang(void) const { return nang_; }

    /**
     * check the width of the original or remapped cartesian image.
     * @return the width of the cartesian image (defualt = 640).
     */
    int width(void) const { return width_; }

    /**
     * check the height of the original or remapped cartesian image.
     * @return the height of the cartesian image (default = 480).
     */
    int height(void) const { return height_; }

    /**
     * return the desired overlap btw receptive fields.
     * @return the desired overlap (default = 1.0).
     */
    double overlap(void) const { return overlap_; }

    /**
     * return the operating mode, one of BOTH, C2L, L2C.
     * @return the value of mode (default = BOTH).
     */
    int mode(void) const { return mode_; }

    /**
     * computes the ratio between the size of the smallest logpolar pixel and the cartesian one.
     * WARNING: this method is deprecated and maintained only for compatibility with old code.
     * @return the scale factor.
     */
    double computeScaleFactor(void) {
        return RCcomputeScaleFactor();
    }

};

#endif

//----- end-of-file --- ( next line intentionally left blank ) ------------------


