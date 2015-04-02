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

#ifndef __MODULE_H__
#define __MODULE_H__

#include <string>

#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>

#include <cv.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"

#include "iCub/utils.h"

/**********************************************************/
struct lineData
    {
        double gradient;
        double intercept;
    };

/**********************************************************/
class Manager : public yarp::os::RFModule
{
    protected:

    std::string                 name;                                                   //name of the module
    yarp::os::Port              rpcHuman;                                               //human rpc port (receive commands via rpc)

    yarp::os::Port              motionFilter;                                           //port that receives an image containing blobs from motion
    yarp::os::Port              toolPoint;                                              //port that receives an image containing blobs from motion
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imgOutPort;        //port that sends out img

    MotionFeatures              motionFeatures;         //class to receive points from motionFilter

    lineData                    *lineDetails;

    int                         processHumanCmd(const yarp::os::Bottle &cmd, yarp::os::Bottle &b);
    void                        processBlobs(yarp::os::Bottle &b, cv::Mat &dest, lineData *lineDetails);
    void                        processMotionPoints(yarp::os::Bottle &b);
    yarp::os::Bottle            processImage(yarp::os::Bottle &b, cv::Mat &dest, cv::Mat &clean, lineData *lineDetails);
    void                        getIntersection(cv::Mat &dest,  lineData *lineDetails );

    friend class                MotionFeatures;

    

    public:

    bool    configure(yarp::os::ResourceFinder &rf);
    bool    interruptModule();
    bool    close();
    bool    updateModule();
    double  getPeriod();

    

};
#endif