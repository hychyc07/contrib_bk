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

#include <iCub/BuildVtmThread.h>
#include <iCub/ObjRecModule.h>

bool BuildVtmThread::threadInit() 
{
    if (! VtmThread::threadInit())
        return false;

    vtm.clear(); 

    return true;
}

void BuildVtmThread::run()
{
    // exploration completed?
    if (trajectoryCompleted)
    {
        std::cout << "exploration completed" << std::endl;

        // save data
        saveVtm(vtmDir, objectName);

        // send done message
        module->sendActionDoneMsg("vtm build");

        stop();
        return;
    }

    moveArm();
    buildVtm();
}


bool BuildVtmThread::saveVtm(const std::string &vtmDir, const std::string &objectName) const
{
    // Save VTM data
    std::cout << "Saving Vtm data of object '" << objectName << "' to '" << vtmDir << "'..." << std::endl;
    if (! vtm.save(vtmDir, objectName))
    {
        std::cerr << "Could not save Vtm!" << std::endl;
        return false;
    }
    std::cout << "Done" << std::endl;

    return true;
}


void BuildVtmThread::buildVtm()
{
    cv::Mat frame = module->getCameraImage();
    if (frame.empty())
        return;

    if (! keyframeExtractor.isKeyframe(frame))
        return;

    Vector armState(16);
    if (! encArm->getEncoders(armState.data()))
    {
        fprintf(stderr, "Could not get arm data\n"); 
        return;
    }

    std::cout << "==== Adding Keyframe: " << vtm.size()  << " ====" << std::endl;
    vtm.addKeyframe(frame, armState); 
}

