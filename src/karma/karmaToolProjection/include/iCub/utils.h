/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff Ugo Pattacini
 * email:  vadim.tikhanoff@iit.it
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

#ifndef __UTILS_H__
#define __UTILS_H__

#include <string>

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/PortReport.h>

#include <cv.h>

class Manager;  //forward declaration

/**********************************************************/
class MotionFeatures : public yarp::os::BufferedPort<yarp::os::Bottle>
{
protected:
    Manager *manager;
    void onRead(yarp::os::Bottle &b);
public:
    MotionFeatures();
    void setManager(Manager *manager);
    bool getFeatures();
};
/**********************************************************/
class ParticleFilter : public yarp::os::BufferedPort<yarp::os::Bottle>
{
protected:
    CvPoint loc;
    void onRead(yarp::os::Bottle &b);
public:
    ParticleFilter();
    bool getTraker(CvPoint &loc);
};
/**********************************************************/
class SegmentationPoint : public yarp::os::Port
{
public:
    void segment(yarp::os::Bottle &b);
};
/**********************************************************/
#endif