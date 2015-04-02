/* 
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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

#include <iCub/VtmThread.h>
#include <iCub/Vtm.h>
#include <iCub/ObjRecModule.h>

//const double VtmThread::POSITION_0[]={-35, 30, 0, 80, -80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 
//const double VtmThread::POSITION_1[]={-35, 30, 0, 80,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//const double VtmThread::POSITION_2[]={-35, 30, 0, 80, -80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//const double VtmThread::POSITION_3[]={-35, 30, 0, 80, -80, -60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//const double VtmThread::POSITION_4[]={-35, 30, 0, 80,   0, -60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//const double VtmThread::POSITION_5[]={-35, 30, 0, 80, -80, -60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

const double VtmThread::POSITION_0[]={-50, 30, 0, 80, -80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; 
const double VtmThread::POSITION_1[]={-50, 30, 0, 80,  10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const double VtmThread::POSITION_2[]={-50, 30, 0, 80, -80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const double VtmThread::POSITION_3[]={-50, 30, 0, 80, -80, -60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const double VtmThread::POSITION_4[]={-50, 30, 0, 80,   10, -60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
const double VtmThread::POSITION_5[]={-50, 30, 0, 80, -80, -60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

bool VtmThread::threadInit() 
{
    if (! robotArm.isValid()) 
    {
        std::cout << "Cannot connect to robot right arm" << std::endl;  
        std::cout << "Device not available.  Here are the known devices:" << std::endl;
        std::cout << Drivers::factory().toString() << std::endl;
        return false;
    }

    if (! robotArm.view(armCtrl))
    {
        std::cout << "Cannot get interface to the arm" << std::endl;  
        robotArm.close();
        return false;
    }

    if (! robotArm.view(limArm))
    {
        std::cout << "Cannot get interface to the arm" << std::endl;  
        robotArm.close();
        return false;
    }
    if (! robotArm.view(encArm))
    {
        std::cout << "Cannot get interface to the arm" << std::endl;  
        robotArm.close();
        return false;
    }

    trajectoryPos = 0; 
    trajectoryCompleted = false;

    keyframeExtractor.init();

    trajectory.clear();
    trajectory.push_back(Vector(NUM_JOINTS, POSITION_0));
    trajectory.push_back(Vector(NUM_JOINTS, POSITION_1));
    trajectory.push_back(Vector(NUM_JOINTS, POSITION_2));
    trajectory.push_back(Vector(NUM_JOINTS, POSITION_3));
    trajectory.push_back(Vector(NUM_JOINTS, POSITION_4));
    trajectory.push_back(Vector(NUM_JOINTS, POSITION_5));
    
    vtmDir = rf->check("vtmDir", Value("./data/vtms"), "directory containing Vtms").asString().c_str();
    std::cout << "Vtm directory: " << vtmDir << std::endl;
    if (vtmDir == "")
        return false;

    for (int i=0; i<7; i++)
    {
        armCtrl->setRefSpeed(i, 14);
        armCtrl->setRefAcceleration(i, 14);
    }
    return true;
}


void VtmThread::moveArm()
{
    if (! checkArmMotionDone())
        return;

    if (trajectoryPos >= trajectory.size())
    {
        trajectoryPos = 0;
        trajectoryCompleted = true;
    }

    //Vector arm(16);
    //if (! encArm->getEncoders(arm.data()))
    //{
        //std::cerr << "moveArm: Could not get arm data" << std::endl;
        //return;
    //}

    for (int joint = 0; joint < 7; ++joint)
    {
        double pos = trajectory[trajectoryPos][joint];
        Util::safePositionMove(joint, pos, armCtrl, limArm);
    }

    std::cout << "New trajectory pos: " << trajectoryPos << std::endl;
    trajectoryPos++;
}



