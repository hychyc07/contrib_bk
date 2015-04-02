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

#ifndef __BUILDVTM_THREAD_H__
#define __BUILDVTM_THREAD_H__

#include "iCub/Vtm.h"
#include "iCub/VtmThread.h"

class ObjRecModule;

class BuildVtmThread : public VtmThread
{
public:
      BuildVtmThread(
              ObjRecModule *module_,
              int period,
              PolyDriver &robotArm_,
              ResourceFinder *rf_) :
          VtmThread(module_, period, robotArm_, rf_)
    {
    }

    virtual bool threadInit();     
    virtual void run(); 

private:
    bool saveVtm(const std::string &vtmDir, const std::string &objectName) const;
    void buildVtm();

    Vtm vtm;
};

#endif

