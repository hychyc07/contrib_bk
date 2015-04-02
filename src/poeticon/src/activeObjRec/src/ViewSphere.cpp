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
 * @file ViewSphere.cpp
 * @brief
 */

#include <algorithm>
#include <stdlib.h>

#include "iCub/ViewSphere.h"
#include "iCub/Util.h"

const double ViewSphere::MAX_ELEVATION = 180;
const double ViewSphere::MAX_ROTATION = 360;

ViewSphere::ViewSphere()
{
    elevation = -1;
    rotation = -1;
    sphereRadius = 100;
    gazeMap = cv::Mat::zeros(MAX_ELEVATION, MAX_ROTATION, CV_32F); 
    viewImage = cv::Mat::zeros(sphereRadius*2, sphereRadius*4, CV_32F); 
}

void ViewSphere::markKeyframe(double e, double r)
{
    e = std::max(0.0, e);
    e = std::min(MAX_ELEVATION, e);
    r = std::max(0.0, r);
    r = std::min(MAX_ROTATION, r);
    keyframeViews.push_back(std::make_pair(e, r));
}

void ViewSphere::markGaze(double e, double r)
{
    e = std::max(0.0, e);
    e = std::min(MAX_ELEVATION, e);
    r = std::max(0.0, r);
    r = std::min(MAX_ROTATION, r);

    this->elevation = e;
    this->rotation = r;

    // mark on angle map
    gazeMap.at<float>(e,r) = 1.0;
    double fovSize = 45; //degrees
    int minFovE = std::max(e-fovSize, 0.0);
    int maxFovE = std::min(e+fovSize, 180.0);
    int minFovR = 0;//std::max(r-fovSize, 0.0);
    int maxFovR = 360;//std::min(r+fovSize, 360.0);
    for (int fovE = minFovE; fovE < maxFovE; fovE++) 
    {
        for (int fovR = minFovR; fovR < maxFovR; fovR++) 
        {
            double dist = Util::calcCentralAngle(fovE, e, fovR, r);
            if (dist < fovSize)
            {
                double u, v;
                gazeAnglesToViewSphereCoords(fovE, fovR, u, v);

                //double intensity = exp(-1*(dist/fovSize));
                double intensity = (1.0-(dist/fovSize));
                intensity = std::max((double)viewImage.at<float>(v, u), intensity);
                viewImage.at<float>(v, u) = std::min(intensity, 1.0); 
                gazeMap.at<float>(fovE,fovR) =  std::min(intensity, 1.0);
            }
        }
    }
}


double ViewSphere::getViewValue(double e, double r)
{
    e = std::max(0.0, e);
    e = std::min(MAX_ELEVATION, e);
    r = std::max(0.0, r);
    r = std::min(MAX_ROTATION, r);

    //double x, y;
    //gazeAnglesToViewSphereCoords(e, r, x, y);   

    return gazeMap.at<float>(e, r);
}

//cv::Mat ObjRecThread::getExplorationMap()
//{
    //cv::Mat smoothedExpMap = cv::Mat::zeros(explorationMapER.size(), CV_32F);
    //double radius = 81;
    //for (int x = 0; x < smoothedExpMap.cols; x++)
    //{
        //for (int y = 0; y < smoothedExpMap.rows; y++)
        //{
            //if (explorationMapER.at<float>(y,x) < 0.9)
                //continue;

            //for (int i = -radius; i < radius; i++) 
            //{
                //for (int j = -radius; j < radius; j++) 
                //{
                    //double sx_wnd = x + i;
                    //double sy_wnd = y + j;
                    //double dist = sqrt(SQR(i) + SQR(j));
                    //if (dist > radius)
                        //continue;
                    //if ((sx_wnd < smoothedExpMap.cols) && (sy_wnd < smoothedExpMap.rows) &&
                        //(sx_wnd >= 0) && (sy_wnd >= 0))
                    //{
                        //double intensity = (radius > 0) ? exp(-6*SQR(dist/radius)) : 1.0;
                        //intensity = std::max((double)smoothedExpMap.at<float>(sy_wnd, sx_wnd), intensity);
                        //smoothedExpMap.at<float>(sy_wnd, sx_wnd) = std::min(intensity, 1.0); 
                    //}
                //}
            //}
        //}
    //}

    //return smoothedExpMap;
//}




void ViewSphere::show(const std::string &wndName, int delay)
{
    cv::Mat displaySphereImg(sphereRadius*2, sphereRadius*4, CV_8UC3, cv::Scalar(0,0,0));

    double centerX = sphereRadius;
    double centerY = sphereRadius;

    // draw data from exploration map
    for (int y = 0; y < viewImage.rows; y++) 
    {
        for (int x = 0; x < viewImage.cols; x++) 
        {
            centerX = sphereRadius;
            if (x > 2*sphereRadius)
            {
                centerX += 2*sphereRadius;
            }
            if (SQR(x-centerX) + SQR(y-centerY) < SQR(sphereRadius))
            {
                float intensity = viewImage.at<float>(y,x);
                // convert intensity to hsv-color
                cv::Scalar color(intensity*180/3, 150, 255);
                cv::circle(displaySphereImg, cv::Point(x,y), 1, color);
                // show gaze trail
                if (intensity >= .98)
                {
                    cv::circle(displaySphereImg, cv::Point(x,y), 2, cv::Scalar::all(0), -1);
                }
            }
        }
    }

    cv::cvtColor(displaySphereImg, displaySphereImg, CV_HSV2BGR);

    // show keyframe positions
    for (int k = 0; k < keyframeViews.size(); k++)
    {
        double kEl = keyframeViews[k].first;
        double kRot = keyframeViews[k].second;
        double kX;
        double kY;
        gazeAnglesToViewSphereCoords(kEl, kRot, kX, kY);
        cv::Scalar color(0, 0, 0);
        cv::circle(displaySphereImg, cv::Point(kX, kY), 1, color, 2);
    }

    // show current gaze
    if (elevation >= 0 && rotation >= 0)
    {
        double curVsX;
        double curVsY;
        gazeAnglesToViewSphereCoords(elevation, rotation, curVsX, curVsY);
        cv::Scalar color(255, 0, 0);
        cv::circle(displaySphereImg, cv::Point(curVsX, curVsY), 1, color);
    }

    int sphereSize = sphereRadius*2;

    // draw axis
    cv::Scalar axisColor = cv::Scalar(255,255,255);
    cv::circle(displaySphereImg, cv::Point(sphereRadius, sphereRadius), sphereRadius, axisColor,1);
    cv::circle(displaySphereImg, cv::Point(sphereRadius + sphereRadius*2, sphereRadius), sphereRadius, axisColor, 1);
    cv::circle(displaySphereImg, cv::Point(sphereRadius, sphereRadius), 1, axisColor);
    cv::circle(displaySphereImg, cv::Point(sphereRadius + sphereRadius*2, sphereRadius), 1, axisColor);

    // x/y axes
    cv::line(displaySphereImg, cv::Point(0, sphereRadius), cv::Point(sphereSize, sphereRadius), axisColor);
    cv::line(displaySphereImg, cv::Point(sphereRadius, 0), cv::Point(sphereRadius, sphereSize), axisColor);

    // elevation cicles
    centerX = sphereRadius; 
    for (int limE = 0; limE <= 90; limE+=30)
    {
        double x,y;
        gazeAnglesToViewSphereCoords(limE, 0, x, y);
        cv::circle(displaySphereImg, cv::Point(centerX, centerY), abs(y-sphereRadius), axisColor, 1);
    }
    // rotation lines
    for (int limR = 0; limR <= 180; limR+=45)
    {
        cv::Point2d startLine;
        cv::Point2d endLine;
        gazeAnglesToViewSphereCoords(90, limR, startLine.x, startLine.y);
        gazeAnglesToViewSphereCoords(90, limR+180, endLine.x, endLine.y);
        cv::line(displaySphereImg, startLine, endLine, axisColor, 1);
    }

    // backside 
    centerX += sphereSize; 
    // x/y axes
    cv::line(displaySphereImg, cv::Point(centerX-sphereRadius, centerY), cv::Point(centerX+sphereRadius, centerY), axisColor);
    cv::line(displaySphereImg, cv::Point(centerX, centerY-sphereRadius), cv::Point(centerX, centerY+sphereRadius), axisColor);
    // elevation cicles
    for (int limE = 90; limE <= 180; limE+=30)
    {
        double x,y;
        gazeAnglesToViewSphereCoords(limE, 0, x, y);
        cv::circle(displaySphereImg, cv::Point(centerX, centerY), abs(y-sphereRadius), axisColor, 1);
    }
    // rotation lines
    for (int limR = 0; limR <= 180; limR+=45)
    {
        cv::Point2d startLine;
        cv::Point2d endLine;
        gazeAnglesToViewSphereCoords(91, limR, startLine.x, startLine.y);
        gazeAnglesToViewSphereCoords(91, limR+180, endLine.x, endLine.y);
        cv::line(displaySphereImg, startLine, endLine, axisColor, 1);
    }

    
    // show view sphere
    cv::imshow(wndName, displaySphereImg);

    //cv::Mat imgExporationMap = getExplorationMap();
    //cv::imshow(wndName+" gaze map", gazeMap);

    cv::waitKey(delay);
}



void ViewSphere::gazeAnglesToViewSphereCoords(double e, double r, double &x, double &y)
{
    double sx, sy;
    bool backside = Util::gazeAnglesTo2D(e, r, sx, sy);

    double center = sphereRadius;
    x = center + sx * sphereRadius;
    y = center + sy * sphereRadius;

    x += backside ? sphereRadius*2 : 0;
}




