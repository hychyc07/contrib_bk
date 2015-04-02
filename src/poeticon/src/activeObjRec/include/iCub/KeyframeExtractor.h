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
 * @file KeyframeExtractor.h
 * @brief definition of the KeyframeExtractor methods.
 */

#ifndef __KEYFRAME_EXTRACTOR_H__
#define __KEYFRAME_EXTRACTOR_H__

#include <opencv/cv.h>


class KeyframeExtractor
{
private:
    static const double DEFAULT_NEW_KEYFRAME_THRES;
    bool createKeyframe;

    cv::Mat lastKeyframeFeature;
    cv::Mat lastKeyframe;

    // vars for GUI
    int newKeyframeSliderPos;

    bool trackFt(const cv::Mat &frame);


public:
    KeyframeExtractor();
    void init();
    bool isKeyframe(const cv::Mat &frame);
};

#endif  //__KEYFRAME_EXTRACTOR_H__

//----- end-of-file --- ( next line intentionally left blank ) -----------------

