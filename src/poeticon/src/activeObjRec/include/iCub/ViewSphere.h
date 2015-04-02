/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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
 * @file ViewSphere.h
 * @brief 
 */

#ifndef __VIEWSPHERE_H__
#define __VIEWSPHERE_H__

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <iCub/ctrl/math.h>
#include <yarp/math/Math.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;
using namespace iCub;

class ViewSphere
{
public:
    enum ViewType
    {
        DEFAULT = 0,
        KEYFRAME
    };

    static const double MAX_ELEVATION;
    static const double MAX_ROTATION;

    ViewSphere();

    void clear()
    {
        gazeMap.setTo(cv::Scalar(0));
        viewImage.setTo(cv::Scalar(0));
        keyframeViews.clear();
        elevation = -1;
        rotation = -1;
    }

    void markGaze(double e, double r);
    void markKeyframe(double e, double r);
    double getViewValue(double e, double r);

    void show(const std::string &wndName, int delay);
    //void showCurrentGaze(double elevation, rotation)

    void gazeAnglesToViewSphereCoords(double e, double r, double &x, double &y);
private:
    double sphereRadius;
    double elevation;
    double rotation;

    cv::Mat gazeMap;
    cv::Mat viewImage;

    std::vector<std::pair<double, double> > keyframeViews;

    inline double SQR(double x) { return x*x; }
};

#endif  //__VIEWSPHERE_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

