/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Andrea Del Prete
 * email:  andrea.delprete@iit.it
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

#ifndef SFC_LIB
#define SFC_LIB

#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

namespace iCub
{

namespace skinForceControl
{

enum ControlMode { POS_CTRL_MODE, VEL_CTRL_MODE, TORQUE_CTRL_MODE, CTRL_MODE_SIZE };
const std::string ControlMode_desc[CTRL_MODE_SIZE] = { "POS_CTRL_MODE", "VEL_CTRL_MODE", "TORQUE_CTRL_MODE" };

enum ControlLaw
{ 
    NO_CONTROL, 
    FLOAT_CTRL, JPOS_CTRL, POS_CTRL, TORQUE_CTRL, FORCE_CTRL, 
    PARAL_CTRL, DYN_PARAL_CTRL, CONTACT_CTRL, PRESS_CTRL, 
    SAFE_REACH_RIGID_CTRL, SAFE_REACH_SPRING_CTRL, SAFE_REACH_ORTHO_CTRL, 
    CONTROL_LAW_SIZE
};
const std::string ControlLaw_desc[CONTROL_LAW_SIZE] = 
{ 
    "NO_CONTROL", 
    "FLOAT_CTRL", "JPOS_CTRL", "POS_CTRL", "TORQUE_CTRL", "FORCE_CTRL", 
    "PARAL_CTRL", "DYN_PARAL_CTRL", "CONTACT_CTRL", "PRESS_CTRL",
    "SAFE_REACH_RIGID_CTRL", "SAFE_REACH_SPRING_CTRL", "SAFE_REACH_ORTHO_CTRL" 
};

}

} // end namespace



#endif
