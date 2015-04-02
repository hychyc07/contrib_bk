// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @file GlobalSettings.h
 * @brief 
 */

#ifndef __GLOBAL_SETTINGS_H__
#define __GLOBAL_SETTINGS_H__

class GlobalSettings 
{
public:
    double doBoosting;
    double boostingRate;
    double minElevation;
    double maxElevation;
    double minRotation;
    double maxRotation;
    double particlesPerObject;
    double rotationOffset;
};

#endif
