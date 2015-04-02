/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author: Andrea DelPrete
  * email: andrea.delprete@iit.it
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

#include "iCub/safeReach/util.h"
#include <string>
#include <stdarg.h>     // va_list va_arg va_end va_start
#include <yarp/os/Time.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <yarp/dev/PolyDriver.h>

using namespace iCub::safeReach;
using namespace iCub::skinDynLib;
using namespace iCub::iDyn;
using namespace iCub::ctrl;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::math;
using namespace std;


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

