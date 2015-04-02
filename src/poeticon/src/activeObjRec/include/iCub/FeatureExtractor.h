// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Bjoern Browatzki
 * email:   bjoern.browatzki@tuebingen.mpg.de
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
 * @file FeatureExtractor.h
 * @brief definition of the FeatureExtractor methods.
 */

#ifndef __FEATURE_EXTRACTOR_H__
#define __FEATURE_EXTRACTOR_H__

class FeatureExtractor 
{
public:
    enum FeatureType
    {
        UNKNOWN_FEATURE = 0,
        FOURIER,
        PHOG,
        SURF,
        COLOR,
        COMPOSITE
    };
    virtual ~FeatureExtractor () {};

    // Fourier feature parameter
    static const int WIN_WIDTH = 32;
    static const int WIN_HEIGHT = 32;
    static const int FEATURE_LENGTH = WIN_WIDTH*WIN_HEIGHT*3;
    static const int FEATURE_LENGTH_PHOG = 100;
    static const int N_BINS_A = 15; // Histogram bins for color hists (2D hist)
    static const int N_BINS_B = 15;
    static const int N_BINS_COLORPLANE = 100; // 1D color hist
    static const int N_GRID_POINTS_X = 10; // for SURF
    static const int N_GRID_POINTS_Y = 10; // for SURF
    static const int DIMS_SURF = 64;

    static cv::Mat extract(const cv::Mat &src, FeatureType type = FOURIER);
    static cv::Mat extractPhog(const cv::Mat &src);
    static cv::Mat extractFourier(const cv::Mat &src);
    static cv::Mat extractSurf(const cv::Mat &src);
    static cv::Mat extractColor(const cv::Mat &src);
    static cv::Mat extractComposite(const cv::Mat &src);

    // generic comparison for all feature types
    static double compareFeatures(const cv::Mat &ft1, const cv::Mat &ft2);

    static FeatureType featureNameToType(const std::string &name);

    static int featureLength(FeatureType type = FOURIER)
    {
        switch (type)
        {
            case FOURIER:
                return FEATURE_LENGTH;
                break;
            case PHOG:
                return FEATURE_LENGTH_PHOG;
                break;
            case SURF:
                return DIMS_SURF*N_GRID_POINTS_X*N_GRID_POINTS_X;
            case COLOR:
                return 2*N_BINS_COLORPLANE;//N_BINS_A*N_BINS_B;
            case COMPOSITE:
                return featureLength(SURF) + featureLength(COLOR);
            default:
                CV_Assert(false);
        }
    };


private:
    FeatureExtractor () {};
    static cv::Mat extractFromSingleChannel(const cv::Mat &img);
    static cv::Mat transformImage(const cv::Mat &srcImg); // executes Fourier transform
};

#endif  //__FEATURE_EXTRACTOR_H__

//----- end-of-file --- ( next line intentionally left blank ) ----------------- 

