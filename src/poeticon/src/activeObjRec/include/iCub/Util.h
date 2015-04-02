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
 * @file Util.h
 * @brief 
 */

#ifndef __UTIL_H__
#define __UTIL_H__

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#include <iostream>
#include <cmath>
#include <opencv2/core/core.hpp>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

namespace Util
{

    const double PI = std::atan(1.0)*4;
    const double PI_2 = PI / 2.0;
    const double DEG2RAD = PI / 180;
    const double RAD2DEG = 180.0 / PI;

    Vector normalize(const Vector &v);

    void safePositionMove(Vector &target, IPositionControl *pos, IControlLimits *lim);
    void safePositionMove(int jnt, double val, IPositionControl *pos, IControlLimits *lim);
    std::string toString(const Vector &vec);

    inline double sqr(double x) { return x*x; }
    double calcCentralAngle(const cv::Vec2f &vp1, const cv::Vec2f &vp2);
    double calcCentralAngle(double e1_deg, double e2_deg, double r1_deg, double r2_deg);

    // center of gravity
    cv::Point cog(const std::vector<cv::Point> &points);
    cv::Point cog(const cv::Mat &img);

    cv::Mat removeBorder(const cv::Mat &img, const cv::Vec3b &color);

    void imagesc(const cv::Mat &img_, const std::string &wndname = "imagesc", 
        float scaleX = 1, float scaleY = 1, int delay = 0);
    cv::Mat GetColorcoded(const cv::Mat& img_32F);
    cv::Mat GetColorcoded(const cv::Mat& img_32F, double min, double max);

    cv::Mat vstack(const std::vector<cv::Mat> &mats);

    void writeMatTxt(const std::string &filename, const cv::Mat &m);

    cv::Mat tileImgs(const std::vector<cv::Mat> &imgs, int nRows, int nCols, 
        const std::vector<std::string> captions = std::vector<std::string>(), const cv::Scalar &clBgr = cv::Scalar::all(0));

    
    cv::Mat drawHistogram(const cv::Mat &hist, int width = 256, int height = 64);
        
    inline Vector sphericalToCartesian(const Vector &p)
    {
        Vector cart(3);
        double theta = (90-p[0]) * DEG2RAD;
        double rot = p[1] * DEG2RAD;
        double r = p[2];

        cart[0] = -r * sin(theta)*cos(rot);
        cart[1] = r * sin(theta)*sin(rot);
        cart[2] = r * cos(theta);
        return cart;
    }

    inline bool gazeAnglesTo2D(double e, double r, double &x, double &y)
    {
        
        bool backside = false;
        if (e > 90)
        {
            e = 180 - e;
            backside = true;
        }
        //if (e < 0)
        //{
            //e *= -1;
            //r = 180 + (180-r);
        //}
        double angle_rad = e * Util::DEG2RAD;

        double d = (angle_rad / (Util::PI_2));
        //double d = sin(angle_rad);
        double rot = r * Util::DEG2RAD;

        x = -d * sin(rot);
        y = -d * cos(rot);

        return backside;
    };
}


#endif  //__HANDGAZECONTROL_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

