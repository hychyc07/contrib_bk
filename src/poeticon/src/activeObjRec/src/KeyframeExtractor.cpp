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
 * @file KeyframeExtractor.cpp
 * @brief implementation of the KeyframeExtractor (for the demoModule) methods.
 */

#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iCub/KeyframeExtractor.h>
#include <iCub/FeatureExtractor.h>

const double KeyframeExtractor::DEFAULT_NEW_KEYFRAME_THRES = 89;

KeyframeExtractor::KeyframeExtractor() 
{
    init();

    // create trackbar
    cv::namedWindow("Keyframe Settings", 0);
    cv::createTrackbar("Sensitivity", "Keyframe Settings", &newKeyframeSliderPos, 100); 
}

void KeyframeExtractor::init()
{
    createKeyframe = true;
    newKeyframeSliderPos = DEFAULT_NEW_KEYFRAME_THRES;
}

bool KeyframeExtractor::trackFt(const cv::Mat &frame)
{
    cv::Mat currentFeature = FeatureExtractor::extract(frame);

    if (createKeyframe)
    {
        lastKeyframeFeature = currentFeature.clone();
        lastKeyframe = frame.clone();
        createKeyframe = false;
    }
    else
    {
        // show tracked features
        bool displayFeaturePoints = false;
        if (displayFeaturePoints)
        {
            cv::imshow("current frame", frame);
            cv::imshow("last keyframe", lastKeyframe);
            cv::waitKey(10);
        }

        // compare descriptor of current frame with descriptor of last keyframe  
        double sim = FeatureExtractor::compareFeatures(lastKeyframeFeature, currentFeature);
        //std::cout << "sim: " << sim << std::endl;

        double thresh = (double)newKeyframeSliderPos/100.0;
        //std::cout << sim << " " << thresh << std::endl;
        if (sim < thresh)
        {
            createKeyframe = true;
            //std::cout << "new keyframe detected" << std::endl;
        }
    }
    return createKeyframe;
    
}


bool KeyframeExtractor::isKeyframe(const cv::Mat &img)
{
    //trackKlt();
    return trackFt(img);
}
