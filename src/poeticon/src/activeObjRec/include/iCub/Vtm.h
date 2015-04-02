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
 * @file Vtm.h
 * @brief definintion of the Vtm methods.
 */

#ifndef __VTM_H__
#define __VTM_H__

#include <string>
#include <vector>
#include <map>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#include <opencv/cv.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;


typedef std::map<std::pair<int, int>, Vector> TransitionMap;
typedef Vector Transition;

class Vtm
{
private:
    std::string objName;
    std::vector<cv::Mat> keyframes;
    std::vector<cv::Mat> features;
    std::vector<Vector> jointStates;
    TransitionMap transitions;
    
public:
    void clear();
    inline int size() const { return keyframes.size(); }
    inline const std::vector<cv::Mat>& getKeyframes() const { return keyframes; }
    inline const std::vector<cv::Mat>& getFeatures() const { return features; }
    inline const TransitionMap& getTransitions() const { return transitions; }
    void addKeyframe(const cv::Mat& keyframe, const Vector& joints);
    bool save(const std::string &path, const std::string &objectName) const; 
    bool load(const std::string &path, const std::string &objectName);
    std::string objectName() const { return objName; }
    static cv::Mat extractFeature(const cv::Mat& img);
    inline Vector getTransition(int i, int j) const
    {
        assert(i >= 0);
        assert(j >= 0);
        assert(i < size());
        assert(j < size());
        TransitionMap::const_iterator it = transitions.find(std::make_pair(i,j));
        return it->second;
    }

    inline double calcTransitionLength(int i, int j) const
    {
        return calcTransitionLength(getTransition(i,j));
    }

    static inline double calcTransitionLength(const Transition &t) 
    {
        double transitionLength = 0;
        for (int jointNo = 0; jointNo < t.size(); ++jointNo)
        {
            transitionLength += t[jointNo] * t[jointNo];
        }
        return sqrt(transitionLength);
    }

    static inline double calcTransitionDifference(const Transition &t1, const Transition &t2)
    {
        CV_Assert(t1.size() == t2.size());

        double tdiff = 0;
        for (int jointNo = 0; jointNo < t1.size(); ++jointNo)
        {
            tdiff += (t1[jointNo] - t2[jointNo]) * (t1[jointNo] - t2[jointNo]);
        }
        return std::sqrt(tdiff); 
    }

    static inline double compareKeyframes(const cv::Mat &kf1, const cv::Mat &kf2)
    {
        return cv::norm(kf1, kf2, cv::NORM_L2);
    }
    

};

#endif  //__VTM_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

