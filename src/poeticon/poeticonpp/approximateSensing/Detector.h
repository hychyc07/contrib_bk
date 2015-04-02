/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Vadim Tikhanoff, Ali Paikan
 * email:  vadim.tikhanoff@iit.it, ali.paikan@iit.it
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

#include <string>
#include <yarp/os/Bottle.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageDraw.h>
#include <opencv/cvaux.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>

#include <opencv/highgui.h>
#include <opencv/cxcore.h>
//#include <opencv2/objdetect/objdetect.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/imgproc/imgproc.hpp>

YARP_DECLARE_DEVICES(icubmod)

class Detector
{
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > disparityPort;  
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgbFloat> > stereoWorldPort;    
    yarp::os::Port blobPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outImgPort;  
    yarp::os::BufferedPort<yarp::os::Bottle> faceExpPort;
    yarp::os::BufferedPort<yarp::os::Bottle> targetPort;
    yarp::os::BufferedPort<yarp::os::Bottle> fixationPort;

public:

    Detector()
    {
    }

    bool open(yarp::os::ResourceFinder &rf);

    bool close();

    void loop(); 

    bool interrupt();

protected:

private:
    double range;
    int                         gaussian_winsize;
    double                      window_ratio;
    double                      threshold;
    int                         erode_itr;
    int                         dilate_itr; 
    std::string                 faceExpression;
    yarp::dev::PolyDriver clientGaze;
    yarp::dev::IGazeControl *iGaze;

};

   
   



   
