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

#include <yarp/os/Time.h>

#include "iCub/utils.h"
#include "iCub/module.h"

using namespace std;
using namespace yarp::os;

/**********************************************************/
ParticleFilter::ParticleFilter() 
{
    useCallback();
}
/**********************************************************/
void ParticleFilter::onRead(Bottle &target)
{
    //CvPoint trackLoc;
    if (target.size()>1)
    {
        loc.x=(int)target.get(0).asDouble();
        loc.y=(int)target.get(1).asDouble();
        //fprintf( stdout, "particle something: %s \n\n",target.toString().c_str() );
    }
}
/**********************************************************/
bool ParticleFilter::getTraker(CvPoint &loc)
{
    loc=this->loc;
    return true;
}
/**********************************************************/
void SegmentationPoint::segment(Bottle &b)
{
    if (getOutputCount()>0)
    {
        //send 2D x y coordinates to segmentator
        Bottle request;
        request.addDouble(b.get(0).asDouble());
        request.addDouble(b.get(1).asDouble());
        write(request);
    }
}
/**********************************************************/
void PointedLocation::onRead(Bottle &b)
{
    fprintf(stdout, "got read from points size %d \n",b.size());
    if (b.size()>1)
    {
        loc.x=(int)b.get(0).asDouble();
        loc.y=(int)b.get(1).asDouble();
        rxTime=Time::now();
    }
}
/**********************************************************/
PointedLocation::PointedLocation()
{
    useCallback();
    rxTime=-1.0;
    timeout=2.0;
}
/**********************************************************/
bool PointedLocation::getLoc(CvPoint &loc)
{
    double t0=Time::now();

    if ((rxTime>0.0) && (t0-rxTime<timeout))
    {
        loc=this->loc;
        return true;
    }

    while (Time::now()-t0<timeout)
    {
        if ((rxTime>0.0) && (Time::now()-rxTime<timeout))
        {
            loc=this->loc;
            return true;
        }
        Time::delay(0.1);
    }
    return false;
}
