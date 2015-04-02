// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea, Giorgio Metta and Francesco Orabona
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
 * @file WatershedOperator.h
 * @brief definition of the watershed operator class.
 */

#ifndef _WATERSHEDOPERATOR_H_
#define _WATERSHEDOPERATOR_H_

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/sig/Image.h>
#include <yarp/math/Math.h>

#include <iCub/ColorVQ.h>
#include <iCub/YARPBox.h>

/**
* Operator that manages the watershed operations (rain falling) and extracts blobs
* @author Francesco Rea
*/
class WatershedOperator {
    bool logpolar;
    int neighSize;
    int *neigh;
    int *neighL;
    int *neighR;

    /**
    * value of the padding considering the input image
    */
    int padding;
    
    /**
    * size of the input image
    */
    int imageSize;
    
    /**
    * temporary input image
    */
    int* tempRegion;
    
    /**
    * intensity value considered as a threshold
    */
    yarp::sig::PixelMono threshold;
    
    /** 
    * flag that indicates if the neighborhood has dimension 8
    */
    bool neighborhood8;
    
    /**
    * height on the input image
    */
    int height;
    
    /**
    * width on the input image
    */
    int width;
    int widthStep;
    int watershedColor;
    int basinColor;

    yarp::sig::ImageOf<yarp::sig::PixelInt> downPos2;
    yarp::sig::ImageOf<yarp::sig::PixelInt> downPos;
    yarp::sig::ImageOf<yarp::sig::PixelMono> tmp;

    /**
    * reference to the colour quantizator
    */
    ColorVQ colorVQ;

    /**
    * create the neighbourhood considering the 4 proximity or the 8 proximity
    */
    void createNeighborhood(const int widthStep, const bool neigh8);

    //__OLD//void initBorderLUT(const int width, const int height);
    //__OLD//inline bool invalidNeighbor(const int currentPoint, const int currentNeighbor) const;
    //__OLD//inline bool validNeighbor(const int currentPoint, const int currentNeighbor) const;
    int markMinimas(yarp::sig::ImageOf<yarp::sig::PixelInt>& result);

    /**
    * starts the actual process or watershed (rain falling)
    */
    void letsRain(yarp::sig::ImageOf<yarp::sig::PixelInt>& result);

    /**
    * find the neighbour with the minumum level of intensity
    */
    void findLowerNeigh(const yarp::sig::ImageOf<yarp::sig::PixelMono>& src);
    void createTmpImage(const yarp::sig::ImageOf<yarp::sig::PixelMono>& src);

public:
    /**
    * destructor
    */
    ~WatershedOperator();

    /**
    * constructor
    */
    WatershedOperator(const bool lp, const int width1, const int height1, const int wstep, const yarp::sig::PixelMono th);
 
    /*
    * resize all the image attributes of the class watershedOperator
    */
    void resize(const int width1, const int height1, const int wstep, const yarp::sig::PixelMono th);
    
    /**
    * set threashold
    */
    inline void setThreshold(yarp::sig::PixelMono th) { threshold=th; }
    //__OLD//bool apply(ImageOf<PixelMono>& srcdest);
    //__OLD//bool apply(const ImageOf<PixelMono>& src, ImageOf<PixelMono>& dest);

    /**
    * applies the rain watershed from the edge picture, and returns the tagged image of integer
    */
    int apply(const yarp::sig::ImageOf<yarp::sig::PixelMono> &src, yarp::sig::ImageOf<yarp::sig::PixelInt> &result); //
    
    /**
    * applies the rain watershed from the edge picture, and returns the tagged image of integer (OLD)
    */
    int applyOnOld(const yarp::sig::ImageOf<yarp::sig::PixelMono> &src, yarp::sig::ImageOf<yarp::sig::PixelInt> &result);
    
    /*
    *converts the tag image into mono watershed image
    */
    void tags2Watershed(const yarp::sig::ImageOf<yarp::sig::PixelInt>& src, yarp::sig::ImageOf<yarp::sig::PixelMono>& dest);
    
    /**
    * find neighbours to the position x,y
    */
    void findNeighborhood(yarp::sig::ImageOf<yarp::sig::PixelInt>& tagged, int x, int y, char *blobList);
    
    /**
    * extracts the connectivity graph
    */
    void connectivityGraph(const yarp::sig::ImageOf<yarp::sig::PixelInt>& src, bool *matr, int max_tag);
    
    /**
    * get the first plane of the input image
    */
    yarp::sig::ImageOf<yarp::sig::PixelMono>* getPlane ( yarp::sig::ImageOf<yarp::sig::PixelRgb>* src ); //
    
    /*
    * public attribute that represent the input image
    */
    yarp::sig::ImageOf<yarp::sig::PixelMono> tSrc;
    
    /**
    * public attribute that represent the output image
    */
    yarp::sig::ImageOf<yarp::sig::PixelMono>* outputImage;
};

#endif //_WATERSHEDOPERATOR_H_


//----- end-of-file --- ( next line intentionally left blank ) ------------------
