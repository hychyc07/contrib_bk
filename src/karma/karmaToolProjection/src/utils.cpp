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
MotionFeatures::MotionFeatures() 
{
    manager=NULL;
    useCallback();
}
void MotionFeatures::setManager(Manager *manager)
{
    this->manager=manager;
}
/**********************************************************/
void MotionFeatures::onRead(Bottle &target)
{
    if (target.size()>1)
    {
        //fprintf( stdout, "got something throught the port with size: %d\n",target.size() );
        manager->processMotionPoints(target);
        //manager->processBlobs(target);
    }
}

